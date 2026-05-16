#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

#define SPEED_PIN GPIO_NUM_5
#define WIFI_CHANNEL 1

// Sampling frequency
#define SAMPLE_INTERVAL_MS 100

// Speed sensor tuning
#define FILTER_WEIGHT_NUM 4 // 0.40 smoothing weight, expressed as 4 / 10
#define FILTER_WEIGHT_DEN 10
// Precomputed: 10,000,000 / 1.139 (PULSE_FREQ_TO_MPH). Yields speed in 0.1 MPH units
// when used as: (acc_pulses * K_SPEED_X10) / acc_period_us
static const uint32_t K_SPEED_X10 = 8779631UL;
#define KPH_PER_MPH_PPM 1609344UL
#define SPEED_DEADZONE_US 2000UL
#define SNAP_TO_ZERO_US 500000UL
#define OUTPUT_KPH 0 // Set to 1 to output KPH instead of MPH

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
static portMUX_TYPE s_pulse_mux = portMUX_INITIALIZER_UNLOCKED;

static int32_t s_smoothed_speed_x10 = 0;
static int32_t s_last_valid_speed_x10 = 0;
static esp_timer_handle_t s_sample_timer;
static TaskHandle_t s_main_task;

static uint32_t s_last_update_us = 0;

static volatile uint32_t s_last_send_ok_ms = 0;
static const uint32_t RADIO_WATCHDOG_MS = 10000;

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

// ---------------------------------------------------------------------------
// ISR
// ---------------------------------------------------------------------------

static void IRAM_ATTR speed_isr(void *arg)
{
  (void)arg;

  uint32_t now = now_us();
  portENTER_CRITICAL_ISR(&s_pulse_mux);
  if ((now - s_last_pulse_us) > SPEED_DEADZONE_US)
  {
    if (s_last_pulse_us > 0)
    {
      s_acc_period_us += (now - s_last_pulse_us);
      s_acc_pulses++;
    }
    s_last_pulse_us = now;
  }
  portEXIT_CRITICAL_ISR(&s_pulse_mux);
}

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

  init_nvs();
  init_wifi();
  init_esp_now();
  init_speed_gpio();
  init_sample_timer();

  s_last_send_ok_ms = now_ms();

  ESP_LOGI(TAG, "ESP32-C3 VSS System initialized at %d ms intervals.", SAMPLE_INTERVAL_MS);

  while (true)
  {
    uint32_t samples_due = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
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
