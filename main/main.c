#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include "oled.h"

#include "esp_attr.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_idf_version.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

#define SPEED_PIN GPIO_NUM_3
#define WIFI_CHANNEL 1

// Sampling frequency
#define SAMPLE_INTERVAL_MS 100

// Speed sensor tuning
#define FILTER_WEIGHT_NUM 4 // 0.40 smoothing weight, expressed as 4 / 10
#define FILTER_WEIGHT_DEN 10
// Calibration: 70 Hz = 100 KPH = 62.1371 MPH. Yields speed in 0.1 MPH units
// when used as: (acc_pulses * K_SPEED_X10) / acc_period_us.
static const uint32_t K_SPEED_X10 = 8876731UL;
#define KPH_PER_MPH_PPM 1609344UL
#define SPEED_DEADZONE_US 2000UL
#define SNAP_TO_ZERO_US 500000UL
#define MAX_INPUT_SPEED_X10 1220 // Reject pulse intervals faster than 122.0 MPH
#define OUTPUT_KPH 0             // Set to 1 to output KPH instead of MPH
#define ENABLE_SPEED_DIAGNOSTICS 0

static const uint32_t MIN_VALID_PERIOD_US = K_SPEED_X10 / MAX_INPUT_SPEED_X10;

static const uint8_t receiver_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct __attribute__((packed))
{
  uint8_t unit;   // 'M' or 'K'
  uint16_t speed; // speed in 0.1 unit increments (e.g. 657 = 65.7 MPH)
} speed_packet_t;

// ---------------------------------------------------------------------------
// Logic Globals
// ---------------------------------------------------------------------------

static const char *TAG = "speedometer";

static volatile uint32_t s_acc_period_us = 0;
static volatile uint32_t s_acc_pulses = 0;
static volatile uint32_t s_last_pulse_us = 0;
static volatile bool s_seen_pulse = false;
#if ENABLE_SPEED_DIAGNOSTICS
static volatile uint32_t s_diag_raw_edges = 0;
static volatile uint32_t s_diag_accepted_edges = 0;
static volatile uint32_t s_diag_deadzone_rejects = 0;
static volatile uint32_t s_diag_fast_rejects = 0;
static volatile uint32_t s_diag_period_count = 0;
static volatile uint32_t s_diag_period_sum_us = 0;
static volatile uint32_t s_diag_min_period_us = UINT32_MAX;
static volatile uint32_t s_diag_max_period_us = 0;
#endif
static portMUX_TYPE s_pulse_mux = portMUX_INITIALIZER_UNLOCKED;

static int32_t s_smoothed_speed_x10 = 0;
static int32_t s_last_valid_speed_x10 = 0;
static esp_timer_handle_t s_sample_timer;
static TaskHandle_t s_main_task;
#if SOC_GPIO_SUPPORT_PIN_GLITCH_FILTER
static gpio_glitch_filter_handle_t s_speed_glitch_filter;
#endif

static uint32_t s_last_update_us = 0;
#if ENABLE_SPEED_DIAGNOSTICS
static uint32_t s_last_diag_ms = 0;
#endif

static volatile uint32_t s_last_send_ok_ms = 0;
static const uint32_t RADIO_WATCHDOG_MS = 10000;
static bool s_oled_ok = false;

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

static void on_send(
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    const esp_now_send_info_t *tx_info,
#else
    const uint8_t *mac,
#endif
    esp_now_send_status_t status)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
  (void)tx_info;
#else
  (void)mac;
#endif

  if (status == ESP_NOW_SEND_SUCCESS)
  {
    s_last_send_ok_ms = now_ms();
  }
}

static void sample_timer_cb(void *arg)
{
  (void)arg;
  if (s_main_task != NULL)
  {
    xTaskNotifyGive(s_main_task);
  }
}

static void oled_print_centered(int page, const char *text)
{
  int px = (72 - ((int)strlen(text) * 6)) / 2;
  oled_print(px < 0 ? 0 : px, page, text);
}

static void oled_waiting_task(void *arg)
{
  (void)arg;
  static const char *dots[] = {".", "..", "..."};
  int dot_idx = 0;

  while (!s_seen_pulse)
  {
    oled_clear();
    oled_print_centered(1, "Waiting");
    oled_print_centered(2, dots[dot_idx]);
    oled_flush();

    dot_idx = (dot_idx + 1) % 3;
    vTaskDelay(pdMS_TO_TICKS(600));
  }

  while (true)
  {
    int32_t speed_x10 = s_smoothed_speed_x10;
    char speed[24];

#if OUTPUT_KPH
    speed_x10 = (int32_t)(((uint64_t)speed_x10 * KPH_PER_MPH_PPM + 500000ULL) / 1000000ULL);
    snprintf(speed, sizeof(speed), "%ld.%ld KPH", (long)(speed_x10 / 10), (long)(speed_x10 % 10));
#else
    snprintf(speed, sizeof(speed), "%ld.%ld MPH", (long)(speed_x10 / 10), (long)(speed_x10 % 10));
#endif

    oled_clear();
    oled_print_centered(1, "Speed");
    oled_print_centered(2, speed);
    oled_flush();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ---------------------------------------------------------------------------
// ISR
// ---------------------------------------------------------------------------

static void IRAM_ATTR speed_isr(void *arg)
{
  (void)arg;

  uint32_t now = now_us();
  portENTER_CRITICAL_ISR(&s_pulse_mux);
#if ENABLE_SPEED_DIAGNOSTICS
  s_diag_raw_edges++;
#endif
  if ((now - s_last_pulse_us) > SPEED_DEADZONE_US)
  {
    uint32_t period_us = now - s_last_pulse_us;
    s_seen_pulse = true;

    if (s_last_pulse_us > 0)
    {
      if (period_us < MIN_VALID_PERIOD_US)
      {
#if ENABLE_SPEED_DIAGNOSTICS
        s_diag_fast_rejects++;
#endif
        portEXIT_CRITICAL_ISR(&s_pulse_mux);
        return;
      }

      s_acc_period_us += period_us;
      s_acc_pulses++;
#if ENABLE_SPEED_DIAGNOSTICS
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
#endif
    }
#if ENABLE_SPEED_DIAGNOSTICS
    s_diag_accepted_edges++;
#endif
    s_last_pulse_us = now;
  }
#if ENABLE_SPEED_DIAGNOSTICS
  else
  {
    s_diag_deadzone_rejects++;
  }
#endif
  portEXIT_CRITICAL_ISR(&s_pulse_mux);
}

#if ENABLE_SPEED_DIAGNOSTICS
static void maybe_log_speed_diagnostics(int32_t current_speed_x10)
{
  uint32_t ms = now_ms();
  if ((ms - s_last_diag_ms) < 1000)
  {
    return;
  }
  s_last_diag_ms = ms;

  portENTER_CRITICAL(&s_pulse_mux);
  uint32_t raw_edges = s_diag_raw_edges;
  uint32_t accepted_edges = s_diag_accepted_edges;
  uint32_t deadzone_rejects = s_diag_deadzone_rejects;
  uint32_t fast_rejects = s_diag_fast_rejects;
  uint32_t period_count = s_diag_period_count;
  uint32_t period_sum_us = s_diag_period_sum_us;
  uint32_t min_period_us = s_diag_min_period_us;
  uint32_t max_period_us = s_diag_max_period_us;

  s_diag_raw_edges = 0;
  s_diag_accepted_edges = 0;
  s_diag_deadzone_rejects = 0;
  s_diag_fast_rejects = 0;
  s_diag_period_count = 0;
  s_diag_period_sum_us = 0;
  s_diag_min_period_us = UINT32_MAX;
  s_diag_max_period_us = 0;
  portEXIT_CRITICAL(&s_pulse_mux);

  uint32_t avg_period_us = period_count > 0 ? (period_sum_us / period_count) : 0;
  uint32_t hz_x100 = period_sum_us > 0 ? (uint32_t)(((uint64_t)period_count * 100000000ULL) / period_sum_us) : 0;
  int32_t period_speed_x10 = avg_period_us > 0 ? (int32_t)(K_SPEED_X10 / avg_period_us) : 0;

  ESP_LOGI(TAG,
           "speed diag: raw=%lu ok=%lu dead=%lu fast=%lu hz=%lu.%02lu avg=%luus min=%luus max=%luus period=%ld.%ldmph current=%ld.%ldmph smooth=%ld.%ldmph",
           (unsigned long)raw_edges,
           (unsigned long)accepted_edges,
           (unsigned long)deadzone_rejects,
           (unsigned long)fast_rejects,
           (unsigned long)(hz_x100 / 100),
           (unsigned long)(hz_x100 % 100),
           (unsigned long)avg_period_us,
           (unsigned long)(period_count > 0 ? min_period_us : 0),
           (unsigned long)max_period_us,
           (long)(period_speed_x10 / 10),
           (long)(period_speed_x10 % 10),
           (long)(current_speed_x10 / 10),
           (long)(current_speed_x10 % 10),
           (long)(s_smoothed_speed_x10 / 10),
           (long)(s_smoothed_speed_x10 % 10));
}
#endif

// ---------------------------------------------------------------------------
// Test Mode Logic (Dynamically scaled by SAMPLE_INTERVAL_MS)
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

  // Calculate total ticks needed based on SAMPLE_INTERVAL_MS
  int rem = s_test_tick;
  int cycle;
  for (cycle = 0; cycle < NUM_TEST_CYCLES; cycle++)
  {
    int cycle_ticks = (TEST_CYCLES[cycle].duration_s * 1000) / SAMPLE_INTERVAL_MS;
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

  int total_cycle_ticks = (TEST_CYCLES[cycle].duration_s * 1000) / SAMPLE_INTERVAL_MS;
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
    s_acc_period_us = 0;
    s_acc_pulses = 0;
    s_last_pulse_us = 0;
    portEXIT_CRITICAL(&s_pulse_mux);

    s_last_update_us = now;
  }
  else
  {
    // 1. Safely grab accumulators
    portENTER_CRITICAL(&s_pulse_mux);
    uint32_t acc_period = s_acc_period_us;
    uint32_t acc_pulses = s_acc_pulses;
    uint32_t last_pulse = s_last_pulse_us;

    if (acc_pulses > 0)
    {
      s_acc_period_us = 0;
      s_acc_pulses = 0;
    }

    uint32_t time_since_last = (last_pulse > 0) ? (now - last_pulse) : 0;

    // SAFE TIMEOUT RESET: Must be inside the critical section to prevent ISR race conditions
    if (time_since_last > SNAP_TO_ZERO_US)
    {
      s_last_pulse_us = 0;
      last_pulse = 0;
    }
    portEXIT_CRITICAL(&s_pulse_mux);

    // 2. Calculate Raw Speed
    if (acc_pulses > 0)
    {
      current_speed_x10 = (int32_t)((uint64_t)acc_pulses * K_SPEED_X10 / acc_period);

      // Catch skipped/elongated pulses by enforcing real-world physics caps.
      if (s_last_update_us > 0)
      {
        uint32_t dt_us = now - s_last_update_us;
        if (dt_us > 10000UL)
        {
          int32_t max_drop = (int32_t)((350ULL * dt_us + 500000ULL) / 1000000ULL); // 35 MPH/s in 0.1 MPH units
          int32_t max_jump = (int32_t)((250ULL * dt_us + 500000ULL) / 1000000ULL); // 25 MPH/s in 0.1 MPH units

          if (current_speed_x10 < (s_last_valid_speed_x10 - max_drop))
          {
            current_speed_x10 = s_last_valid_speed_x10 - max_drop;
          }
          else if (current_speed_x10 > (s_last_valid_speed_x10 + max_jump))
          {
            current_speed_x10 = s_last_valid_speed_x10 + max_jump;
          }
        }
      }
      s_last_valid_speed_x10 = current_speed_x10;
    }
    else if (last_pulse == 0)
    {
      current_speed_x10 = 0;
      s_last_valid_speed_x10 = 0;
    }
    else
    {
      // No pulses this window: Decay check with 50% buffer
      if (s_last_valid_speed_x10 > 5)
      {
        uint32_t expected_period_us = K_SPEED_X10 / (uint32_t)s_last_valid_speed_x10;

        if (time_since_last > (expected_period_us + (expected_period_us >> 1)))
        {
          current_speed_x10 = (int32_t)(K_SPEED_X10 / time_since_last);
        }
        else
        {
          current_speed_x10 = s_last_valid_speed_x10;
        }
      }
      else
      {
        current_speed_x10 = 0;
      }
    }

    s_last_update_us = now;
  }

  // 3. Apply fixed-point exponential smoothing
  s_smoothed_speed_x10 = ((current_speed_x10 * FILTER_WEIGHT_NUM) +
                          (s_smoothed_speed_x10 * (FILTER_WEIGHT_DEN - FILTER_WEIGHT_NUM)) +
                          (FILTER_WEIGHT_DEN / 2)) /
                         FILTER_WEIGHT_DEN;

  if (current_speed_x10 < 5 && s_smoothed_speed_x10 < 5)
  {
    s_smoothed_speed_x10 = 0;
  }

#if ENABLE_SPEED_DIAGNOSTICS
  maybe_log_speed_diagnostics(current_speed_x10);
#endif

  // 4. Dispatch
  speed_packet_t pkt;
#if OUTPUT_KPH
  pkt.unit = 'K';
  pkt.speed = (uint16_t)(((uint64_t)s_smoothed_speed_x10 * KPH_PER_MPH_PPM + 500000ULL) / 1000000ULL);
#else
  pkt.unit = 'M';
  pkt.speed = (uint16_t)s_smoothed_speed_x10;
#endif

  esp_err_t err = esp_now_send(receiver_mac, (uint8_t *)&pkt, sizeof(pkt));
  if (err != ESP_OK)
  {
    ESP_LOGW(TAG, "esp_now_send failed: %s", esp_err_to_name(err));
  }
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

static void init_nvs(void)
{
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
}

static void init_wifi(void)
{
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

static void init_esp_now(void)
{
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_send_cb(on_send));

  esp_now_peer_info_t peer_info = {0};
  memcpy(peer_info.peer_addr, receiver_mac, sizeof(receiver_mac));
  peer_info.ifidx = WIFI_IF_STA;
  peer_info.channel = WIFI_CHANNEL;
  peer_info.encrypt = false;

  ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
}

static void init_speed_gpio(void)
{
  gpio_config_t io_conf = {
      .pin_bit_mask = 1ULL << SPEED_PIN,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_NEGEDGE,
  };
  ESP_ERROR_CHECK(gpio_config(&io_conf));

#if SOC_GPIO_SUPPORT_PIN_GLITCH_FILTER
  gpio_pin_glitch_filter_config_t filter_config = {
      .clk_src = GLITCH_FILTER_CLK_SRC_DEFAULT,
      .gpio_num = SPEED_PIN,
  };
  ESP_ERROR_CHECK(gpio_new_pin_glitch_filter(&filter_config, &s_speed_glitch_filter));
  ESP_ERROR_CHECK(gpio_glitch_filter_enable(s_speed_glitch_filter));
#endif

  ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
  ESP_ERROR_CHECK(gpio_isr_handler_add(SPEED_PIN, speed_isr, NULL));
}

static void init_sample_timer(void)
{
  const esp_timer_create_args_t timer_args = {
      .callback = sample_timer_cb,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "speed_sample",
      .skip_unhandled_events = true,
  };

  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_sample_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(s_sample_timer, SAMPLE_INTERVAL_MS * 1000));
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
      ESP_LOGI(TAG, "Test mode OFF");
    }
    else
    {
      s_test_tick = 0;
      s_test_mode = true;
      ESP_LOGI(TAG, "Test mode ON");
    }
  }
}

void app_main(void)
{
  s_main_task = xTaskGetCurrentTaskHandle();

  setvbuf(stdin, NULL, _IONBF, 0);
  fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

  s_oled_ok = oled_init();
  if (!s_oled_ok)
  {
    ESP_LOGW(TAG, "OLED unavailable; continuing headless");
  }
  else if (xTaskCreate(oled_waiting_task, "oled_wait", 3072, NULL, 2, NULL) != pdPASS)
  {
    ESP_LOGW(TAG, "OLED task create failed; continuing headless");
  }

  init_nvs();
  init_wifi();
  init_esp_now();
  init_speed_gpio();
  init_sample_timer();

  s_last_send_ok_ms = now_ms();

  ESP_LOGI(TAG, "ESP32-C3 VSS System initialized at %d ms intervals.", SAMPLE_INTERVAL_MS);

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

    handle_serial_input();

    if (now_ms() - s_last_send_ok_ms > RADIO_WATCHDOG_MS)
    {
      ESP_LOGW(TAG, "Watchdog: radio hang detected, rebooting.");
      vTaskDelay(pdMS_TO_TICKS(100));
      esp_restart();
    }
  }
}
