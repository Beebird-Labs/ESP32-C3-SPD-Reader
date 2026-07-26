#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "ble_prov.h"
#include "project_config.h"
#if APP_ENABLE_OLED
#include "oled.h"
#endif
#include "ota_manager.h"
#include "speed_output.h"

#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// Calibration: 70 Hz = 100 KPH = 62.1371 MPH. Yields speed in 0.1 MPH units
// when used as: (accepted_period_count * K_SPEED_X10) / accepted_period_sum_us.
static const uint32_t K_SPEED_X10 = APP_K_SPEED_X10;
static const uint32_t MIN_VALID_PERIOD_US = APP_K_SPEED_X10 / APP_MAX_INPUT_SPEED_X10;

// ---------------------------------------------------------------------------
// Logic Globals
// ---------------------------------------------------------------------------

static const char *TAG = "SPEEDOMETER";

static volatile uint32_t s_accepted_period_sum_us = 0;
static volatile uint32_t s_accepted_period_count = 0;
static volatile uint32_t s_accepted_window_start_us = 0;
static volatile uint32_t s_last_accepted_pulse_us = 0;
static volatile uint32_t s_period_baseline_us = 0;
static volatile uint32_t s_period_baseline_count = 0;
static volatile uint32_t s_consistency_long_reject_count = 0;
static volatile bool s_seen_pulse = false;
#if APP_ENABLE_SPEED_DIAGNOSTICS
static volatile uint32_t s_diag_raw_edges = 0;
static volatile uint32_t s_diag_accepted_edges = 0;
static volatile uint32_t s_diag_near_edge_rejects = 0;
static volatile uint32_t s_diag_too_fast_rejects = 0;
static volatile uint32_t s_diag_short_period_rejects = 0;
static volatile uint32_t s_diag_long_period_rejects = 0;
static volatile uint32_t s_diag_period_count = 0;
static volatile uint32_t s_diag_period_sum_us = 0;
static volatile uint32_t s_diag_min_period_us = UINT32_MAX;
static volatile uint32_t s_diag_max_period_us = 0;
static volatile uint32_t s_diag_total_raw_edges = 0;
static volatile uint32_t s_diag_total_accepted_edges = 0;
static volatile uint32_t s_diag_total_near_edge_rejects = 0;
static volatile uint32_t s_diag_total_too_fast_rejects = 0;
static volatile uint32_t s_diag_total_short_period_rejects = 0;
static volatile uint32_t s_diag_total_long_period_rejects = 0;
#endif
static portMUX_TYPE s_pulse_mux = portMUX_INITIALIZER_UNLOCKED;

static int32_t s_smoothed_speed_x10 = 0;
static int32_t s_last_valid_speed_x10 = 0;
static uint32_t s_last_send_us = 0;
static esp_timer_handle_t s_sample_timer;
static TaskHandle_t s_main_task;
#if SOC_GPIO_SUPPORT_PIN_GLITCH_FILTER
static gpio_glitch_filter_handle_t s_speed_glitch_filter;
#endif

#if APP_ENABLE_SPEED_DIAGNOSTICS
static uint32_t s_last_diag_ms = 0;
#endif

// ---------------------------------------------------------------------------
// Time Helpers
// ---------------------------------------------------------------------------

static uint32_t IRAM_ATTR now_us(void)
{
    return (uint32_t)esp_timer_get_time();
}

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

static void sample_timer_cb(void *arg)
{
    (void)arg;
    if (s_main_task != NULL)
    {
        xTaskNotifyGive(s_main_task);
    }
}

#if APP_ENABLE_OLED
static void oled_speed_task(void *arg)
{
    (void)arg;

    while (true)
    {
        int32_t speed_x10 = s_smoothed_speed_x10;
        char speed[24];

#if APP_OUTPUT_KPH
        speed_x10 = (int32_t)(((uint64_t)speed_x10 * APP_KPH_PER_MPH_PPM + 500000ULL) / 1000000ULL);
#endif
        int32_t display_speed = (speed_x10 + 5) / 10;
        if (display_speed > 99)
        {
            display_speed = 99;
        }
        snprintf(speed, sizeof(speed), "%ld", (long)display_speed);

        oled_clear();
        oled_print_scaled((72 - ((int)strlen(speed) * 6 - 1) * 5) / 2, 2, speed, 5);
        oled_flush();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
#endif

// ---------------------------------------------------------------------------
// ISR
// ---------------------------------------------------------------------------

static void IRAM_ATTR reset_period_consistency(void)
{
    s_period_baseline_us = 0;
    s_period_baseline_count = 0;
    s_consistency_long_reject_count = 0;
}

static void IRAM_ATTR update_period_baseline(uint32_t period_us)
{
    if (s_period_baseline_count == 0)
    {
        s_period_baseline_us = period_us;
        s_period_baseline_count = 1;
    }
    else if (s_period_baseline_count < APP_PERIOD_CONSISTENCY_MIN_COUNT)
    {
        uint32_t count = s_period_baseline_count + 1;
        s_period_baseline_us =
            (uint32_t)(((uint64_t)s_period_baseline_us * s_period_baseline_count + period_us) / count);
        s_period_baseline_count = count;
    }
    else
    {
        s_period_baseline_us = (uint32_t)(((uint64_t)s_period_baseline_us * 3 + period_us + 2) / 4);
    }

    s_consistency_long_reject_count = 0;
}

static void IRAM_ATTR speed_isr(void *arg)
{
    (void)arg;

    uint32_t now = now_us();
    portENTER_CRITICAL_ISR(&s_pulse_mux);
#if APP_ENABLE_SPEED_DIAGNOSTICS
    s_diag_raw_edges++;
    s_diag_total_raw_edges++;
#endif
    if (s_last_accepted_pulse_us == 0)
    {
        s_seen_pulse = true;
#if APP_ENABLE_SPEED_DIAGNOSTICS
        s_diag_accepted_edges++;
        s_diag_total_accepted_edges++;
#endif
        s_last_accepted_pulse_us = now;
        portEXIT_CRITICAL_ISR(&s_pulse_mux);
        return;
    }

    uint32_t period_us = now - s_last_accepted_pulse_us;
    if (period_us < MIN_VALID_PERIOD_US)
    {
#if APP_ENABLE_SPEED_DIAGNOSTICS
        if (period_us <= APP_SPEED_DIAG_DEADZONE_US)
        {
            s_diag_near_edge_rejects++;
            s_diag_total_near_edge_rejects++;
        }
        else
        {
            s_diag_too_fast_rejects++;
            s_diag_total_too_fast_rejects++;
        }
#endif
        portEXIT_CRITICAL_ISR(&s_pulse_mux);
        return;
    }

    if (s_period_baseline_count >= APP_PERIOD_CONSISTENCY_MIN_COUNT)
    {
        uint32_t min_consistent_period =
            (uint32_t)(((uint64_t)s_period_baseline_us * (100 - APP_PERIOD_CONSISTENCY_PERCENT)) / 100);
        uint32_t max_consistent_period =
            (uint32_t)(((uint64_t)s_period_baseline_us * (100 + APP_PERIOD_CONSISTENCY_PERCENT)) / 100);

        if (period_us < min_consistent_period)
        {
#if APP_ENABLE_SPEED_DIAGNOSTICS
            s_diag_short_period_rejects++;
            s_diag_total_short_period_rejects++;
#endif
            portEXIT_CRITICAL_ISR(&s_pulse_mux);
            return;
        }

        if (period_us > max_consistent_period)
        {
#if APP_ENABLE_SPEED_DIAGNOSTICS
            s_diag_long_period_rejects++;
            s_diag_total_long_period_rejects++;
#endif
            s_seen_pulse = true;
            s_last_accepted_pulse_us = now;
            s_consistency_long_reject_count++;

            if (s_consistency_long_reject_count < APP_PERIOD_CONSISTENCY_REACQUIRE_COUNT)
            {
                portEXIT_CRITICAL_ISR(&s_pulse_mux);
                return;
            }

            reset_period_consistency();
        }
    }

    s_seen_pulse = true;
    if (s_accepted_period_count == 0)
    {
        s_accepted_window_start_us = s_last_accepted_pulse_us;
    }
    s_accepted_period_sum_us += period_us;
    s_accepted_period_count++;
    update_period_baseline(period_us);
#if APP_ENABLE_SPEED_DIAGNOSTICS
    s_diag_period_sum_us += period_us;
    s_diag_period_count++;
    if (period_us < s_diag_min_period_us)
    {
        s_diag_min_period_us = period_us;
    }
    if (period_us > s_diag_max_period_us)
    {
        s_diag_max_period_us = period_us;
    }
    s_diag_accepted_edges++;
    s_diag_total_accepted_edges++;
#endif
    s_last_accepted_pulse_us = now;
    portEXIT_CRITICAL_ISR(&s_pulse_mux);
}

#if APP_ENABLE_SPEED_DIAGNOSTICS
static void maybe_log_speed_diagnostics(int32_t current_speed_x10)
{
    uint32_t ms = now_ms();
    if ((ms - s_last_diag_ms) < APP_SPEED_DIAGNOSTICS_INTERVAL_MS)
    {
        return;
    }
    s_last_diag_ms = ms;

    portENTER_CRITICAL(&s_pulse_mux);
    uint32_t raw_edges = s_diag_raw_edges;
    uint32_t accepted_edges = s_diag_accepted_edges;
    uint32_t near_edge_rejects = s_diag_near_edge_rejects;
    uint32_t too_fast_rejects = s_diag_too_fast_rejects;
    uint32_t short_period_rejects = s_diag_short_period_rejects;
    uint32_t long_period_rejects = s_diag_long_period_rejects;
    uint32_t period_count = s_diag_period_count;
    uint32_t period_sum_us = s_diag_period_sum_us;
    uint32_t min_period_us = s_diag_min_period_us;
    uint32_t max_period_us = s_diag_max_period_us;
    uint32_t total_raw_edges = s_diag_total_raw_edges;
    uint32_t total_accepted_edges = s_diag_total_accepted_edges;
    uint32_t total_near_edge_rejects = s_diag_total_near_edge_rejects;
    uint32_t total_too_fast_rejects = s_diag_total_too_fast_rejects;
    uint32_t total_short_period_rejects = s_diag_total_short_period_rejects;
    uint32_t total_long_period_rejects = s_diag_total_long_period_rejects;
    uint32_t last_accepted_pulse_us = s_last_accepted_pulse_us;
    uint32_t period_baseline_us = s_period_baseline_us;
    bool seen_pulse = s_seen_pulse;

    s_diag_raw_edges = 0;
    s_diag_accepted_edges = 0;
    s_diag_near_edge_rejects = 0;
    s_diag_too_fast_rejects = 0;
    s_diag_short_period_rejects = 0;
    s_diag_long_period_rejects = 0;
    s_diag_period_count = 0;
    s_diag_period_sum_us = 0;
    s_diag_min_period_us = UINT32_MAX;
    s_diag_max_period_us = 0;
    portEXIT_CRITICAL(&s_pulse_mux);

    uint32_t avg_period_us = period_count > 0 ? (period_sum_us / period_count) : 0;
    uint32_t hz_x100 = period_sum_us > 0 ? (uint32_t)(((uint64_t)period_count * 100000000ULL) / period_sum_us) : 0;
    int32_t period_speed_x10 = avg_period_us > 0 ? (int32_t)(K_SPEED_X10 / avg_period_us) : 0;
    uint32_t since_last_ms = last_accepted_pulse_us > 0 ? ((now_us() - last_accepted_pulse_us) / 1000UL) : 0;
    int level = gpio_get_level(APP_SPEED_PIN);

    ESP_LOGI(TAG,
             "speed_diag pin=%d level=%d seen=%d raw=%lu ok=%lu near=%lu too_fast=%lu short=%lu long=%lu "
             "total_raw=%lu total_ok=%lu total_near=%lu total_too_fast=%lu total_short=%lu total_long=%lu "
             "since_last_ms=%lu hz=%lu.%02lu avg_us=%lu min_us=%lu max_us=%lu baseline_us=%lu "
             "period_mph=%ld.%ld current_mph=%ld.%ld smooth_mph=%ld.%ld",
             (int)APP_SPEED_PIN, level, seen_pulse ? 1 : 0,
             (unsigned long)raw_edges, (unsigned long)accepted_edges, (unsigned long)near_edge_rejects,
             (unsigned long)too_fast_rejects, (unsigned long)short_period_rejects, (unsigned long)long_period_rejects,
             (unsigned long)total_raw_edges, (unsigned long)total_accepted_edges,
             (unsigned long)total_near_edge_rejects, (unsigned long)total_too_fast_rejects,
             (unsigned long)total_short_period_rejects, (unsigned long)total_long_period_rejects,
             (unsigned long)since_last_ms,
             (unsigned long)(hz_x100 / 100), (unsigned long)(hz_x100 % 100),
             (unsigned long)avg_period_us, (unsigned long)(period_count > 0 ? min_period_us : 0),
             (unsigned long)max_period_us, (unsigned long)period_baseline_us,
             (long)(period_speed_x10 / 10), (long)(period_speed_x10 % 10),
             (long)(current_speed_x10 / 10), (long)(current_speed_x10 % 10), (long)(s_smoothed_speed_x10 / 10),
             (long)(s_smoothed_speed_x10 % 10));
}
#endif

// ---------------------------------------------------------------------------
// Test Mode Logic (Dynamically scaled by APP_SAMPLE_INTERVAL_MS)
// ---------------------------------------------------------------------------

typedef struct
{
    int duration_s;
    int max_speed;
} test_cycle_t;

static const test_cycle_t TEST_CYCLES[] = {
    {30, 120}, // 30s ramp
    {20, 120}, // 20s ramp
    {15, 60},  // 15s ramp
};
static const int NUM_TEST_CYCLES = sizeof(TEST_CYCLES) / sizeof(TEST_CYCLES[0]);

static bool s_test_mode = false;
static int s_test_tick = 0;
static int32_t s_test_speed_x10 = 0;

static void test_mode_tick(void)
{
    s_test_tick++;

    // Calculate total ticks needed based on APP_SAMPLE_INTERVAL_MS
    int rem = s_test_tick;
    int cycle;
    for (cycle = 0; cycle < NUM_TEST_CYCLES; cycle++)
    {
        int cycle_ticks = (TEST_CYCLES[cycle].duration_s * 1000) / APP_SAMPLE_INTERVAL_MS;
        if (rem <= cycle_ticks)
        {
            break;
        }
        rem -= cycle_ticks;
    }

    if (cycle >= NUM_TEST_CYCLES)
    {
        s_test_tick = 0;
        s_test_speed_x10 = 0;
        return;
    }

    int total_cycle_ticks = (TEST_CYCLES[cycle].duration_s * 1000) / APP_SAMPLE_INTERVAL_MS;
    int half = total_cycle_ticks / 2;
    int max = TEST_CYCLES[cycle].max_speed;

    int ramp_ticks = (rem <= half) ? rem : (total_cycle_ticks - rem);
    s_test_speed_x10 = ((int32_t)ramp_ticks * max * 10 + (half / 2)) / half;
}

// ---------------------------------------------------------------------------
// Sampling & Delivery
// ---------------------------------------------------------------------------

static void sample_and_send(void)
{
    int32_t current_speed_x10;
    uint32_t now = now_us();

    if (s_test_mode)
    {
        test_mode_tick();
        current_speed_x10 = s_test_speed_x10;
        s_last_valid_speed_x10 = current_speed_x10;

        // Keep accumulators clear while in test mode to prevent a flood of stale data when test mode ends
        portENTER_CRITICAL(&s_pulse_mux);
        s_accepted_period_sum_us = 0;
        s_accepted_period_count = 0;
        s_accepted_window_start_us = 0;
        s_last_accepted_pulse_us = 0;
        reset_period_consistency();
        portEXIT_CRITICAL(&s_pulse_mux);
    }
    else
    {
        // 1. Safely grab accumulators
        portENTER_CRITICAL(&s_pulse_mux);
        uint32_t accepted_period_sum = s_accepted_period_sum_us;
        uint32_t accepted_period_count = s_accepted_period_count;
        uint32_t accepted_window_start = s_accepted_window_start_us;
        uint32_t last_accepted_pulse = s_last_accepted_pulse_us;

        uint32_t time_since_last = (last_accepted_pulse > 0) ? (now - last_accepted_pulse) : 0;
        bool timed_out = time_since_last > APP_SNAP_TO_ZERO_US;
        bool enough_periods = accepted_period_count >= APP_MIN_PERIODS_PER_SPEED_SAMPLE;
        bool window_expired = accepted_period_count > 0 &&
                              (now - accepted_window_start) >= APP_MAX_SPEED_SAMPLE_WINDOW_US;
        bool consume_periods = !timed_out && (enough_periods || window_expired);

        // SAFE TIMEOUT RESET: Must be inside the critical section to prevent ISR race conditions
        if (timed_out)
        {
            s_accepted_period_sum_us = 0;
            s_accepted_period_count = 0;
            s_accepted_window_start_us = 0;
            s_last_accepted_pulse_us = 0;
            reset_period_consistency();
            accepted_period_count = 0;
            last_accepted_pulse = 0;
        }
        else if (consume_periods)
        {
            s_accepted_period_sum_us = 0;
            s_accepted_period_count = 0;
            s_accepted_window_start_us = 0;
        }
        portEXIT_CRITICAL(&s_pulse_mux);

        // 2. Calculate Raw Speed
        if (consume_periods && accepted_period_count > 0)
        {
            current_speed_x10 = (int32_t)((uint64_t)accepted_period_count * K_SPEED_X10 / accepted_period_sum);
            s_last_valid_speed_x10 = current_speed_x10;
        }
        else if (last_accepted_pulse == 0)
        {
            current_speed_x10 = 0;
            s_last_valid_speed_x10 = 0;
        }
        else
        {
            // No pulses this window, but not yet timed out. Hold the last known
            // speed — computing a lower speed from time_since_last causes false
            // sagging during acceleration when a pulse falls outside the sample
            // window. The SNAP_TO_ZERO_US timeout above already handles the
            // genuine stopped case.
            current_speed_x10 = s_last_valid_speed_x10;
        }
    }

    // 3. Apply fixed-point exponential smoothing
    s_smoothed_speed_x10 =
        ((current_speed_x10 * APP_FILTER_WEIGHT_NUM) +
         (s_smoothed_speed_x10 * (APP_FILTER_WEIGHT_DEN - APP_FILTER_WEIGHT_NUM)) + (APP_FILTER_WEIGHT_DEN / 2)) /
        APP_FILTER_WEIGHT_DEN;

    if (current_speed_x10 < 5 && s_smoothed_speed_x10 < 5)
    {
        s_smoothed_speed_x10 = 0;
    }

#if APP_ENABLE_SPEED_DIAGNOSTICS
    maybe_log_speed_diagnostics(current_speed_x10);
#endif

    uint32_t time_since_send = (s_last_send_us > 0) ? (now - s_last_send_us) : APP_SEND_INTERVAL_MS * 1000UL;
    if (time_since_send < APP_SEND_INTERVAL_MS * 1000UL)
    {
        return;
    }
    s_last_send_us = now;

    // 4. Dispatch
#if APP_OUTPUT_KPH
    speed_output_unit_t unit = SPEED_OUTPUT_UNIT_KPH;
    uint16_t speed_x10 = (uint16_t)(((uint64_t)s_smoothed_speed_x10 * APP_KPH_PER_MPH_PPM + 500000ULL) / 1000000ULL);
#else
    speed_output_unit_t unit = SPEED_OUTPUT_UNIT_MPH;
    uint16_t speed_x10 = (uint16_t)s_smoothed_speed_x10;
#endif

    (void)speed_output_send(unit, speed_x10);
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

static void restart_after_error(const char *event)
{
    ESP_LOGE(TAG, "%s action=restart", event);
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
}

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        err = nvs_flash_erase();
        if (err != ESP_OK)
        {
            return err;
        }
        err = nvs_flash_init();
    }
    return err;
}

static esp_err_t init_wifi(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK)
    {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK)
    {
        return err;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK)
    {
        return err;
    }
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK)
    {
        return err;
    }
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK)
    {
        return err;
    }
    err = esp_wifi_start();
    if (err != ESP_OK)
    {
        return err;
    }
    return esp_wifi_set_channel(APP_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
}

static esp_err_t init_speed_gpio(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << APP_SPEED_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK)
    {
        return err;
    }

#if SOC_GPIO_SUPPORT_PIN_GLITCH_FILTER
    gpio_pin_glitch_filter_config_t filter_config = {
        .clk_src = GLITCH_FILTER_CLK_SRC_DEFAULT,
        .gpio_num = APP_SPEED_PIN,
    };
    err = gpio_new_pin_glitch_filter(&filter_config, &s_speed_glitch_filter);
    if (err != ESP_OK)
    {
        return err;
    }
    err = gpio_glitch_filter_enable(s_speed_glitch_filter);
    if (err != ESP_OK)
    {
        return err;
    }
#endif

    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }
    return gpio_isr_handler_add(APP_SPEED_PIN, speed_isr, NULL);
}

static esp_err_t init_sample_timer(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = sample_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "speed_sample",
        .skip_unhandled_events = true,
    };

    esp_err_t err = esp_timer_create(&timer_args, &s_sample_timer);
    if (err != ESP_OK)
    {
        return err;
    }
    return esp_timer_start_periodic(s_sample_timer, APP_SAMPLE_INTERVAL_MS * 1000);
}

static void handle_serial_input(void)
{
    uint8_t c;
    int len = read(STDIN_FILENO, &c, 1);

    if (len == 1 && (c == 't' || c == 'T'))
    {
        if (s_test_mode)
        {
            s_test_mode = false;
            ESP_LOGI(TAG, "test_mode_changed enabled=0");
        }
        else
        {
            s_test_tick = 0;
            s_test_mode = true;
            ESP_LOGI(TAG, "test_mode_changed enabled=1");
        }
    }
}

void app_main(void)
{
    s_main_task = xTaskGetCurrentTaskHandle();

    setvbuf(stdin, NULL, _IONBF, 0);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

#if APP_ENABLE_OLED
    bool oled_ok = oled_init();
    if (!oled_ok)
    {
        ESP_LOGW(TAG, "oled_unavailable continuing=headless");
    }
    else if (xTaskCreate(oled_speed_task, "oled_speed", 3072, NULL, 2, NULL) != pdPASS)
    {
        ESP_LOGW(TAG, "task_create_failed task=oled_speed continuing=headless");
    }
#else
    ESP_LOGI(TAG, "oled_disabled");
#endif

    esp_err_t err = init_nvs();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_init_failed err=%s", esp_err_to_name(err));
        restart_after_error("nvs_init_failed");
    }

    err = init_wifi();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "wifi_init_failed err=%s", esp_err_to_name(err));
        restart_after_error("wifi_init_failed");
    }

    err = speed_output_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "speed_output_init_failed err=%s", esp_err_to_name(err));
        restart_after_error("speed_output_init_failed");
    }

    err = ble_prov_init();
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "ble_init_failed err=%s continuing=1", esp_err_to_name(err));
    }

    err = init_speed_gpio();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "speed_gpio_init_failed err=%s", esp_err_to_name(err));
        restart_after_error("speed_gpio_init_failed");
    }

    err = init_sample_timer();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "sample_timer_init_failed err=%s", esp_err_to_name(err));
        restart_after_error("sample_timer_init_failed");
    }

    ota_manager_init();

    ESP_LOGI(TAG, "app_ready sample_interval_ms=%d send_interval_ms=%d", APP_SAMPLE_INTERVAL_MS, APP_SEND_INTERVAL_MS);
#if APP_ENABLE_SPEED_DIAGNOSTICS
    ESP_LOGI(TAG,
             "speed_diag_enabled pin=%d edge=falling internal_pullup=0 diag_near_edge_us=%lu min_valid_period_us=%lu snap_to_zero_us=%lu "
             "max_input_mph=%d.%d interval_ms=%lu",
             (int)APP_SPEED_PIN, (unsigned long)APP_SPEED_DIAG_DEADZONE_US, (unsigned long)MIN_VALID_PERIOD_US,
             (unsigned long)APP_SNAP_TO_ZERO_US,
             APP_MAX_INPUT_SPEED_X10 / 10, APP_MAX_INPUT_SPEED_X10 % 10,
             (unsigned long)APP_SPEED_DIAGNOSTICS_INTERVAL_MS);
#endif

    static bool s_ota_active = true;

    while (true)
    {
        uint32_t samples_due = ulTaskNotifyTake(pdTRUE, 1);
        if (samples_due > 4)
        {
            samples_due = 4;
        }

        while (samples_due > 0)
        {
            sample_and_send();
            samples_due--;
        }

        if (s_ota_active && s_smoothed_speed_x10 > 0)
        {
            s_ota_active = false;
            ble_prov_disable();
        }

        handle_serial_input();

        if (!g_ota_in_progress && now_ms() - speed_output_last_success_ms() > APP_SPEED_OUTPUT_WATCHDOG_MS)
        {
            ESP_LOGW(TAG, "speed_output_watchdog_timeout action=restart");
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
        }
    }
}
