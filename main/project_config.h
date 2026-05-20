#pragma once

#include "driver/gpio.h"

#define APP_SPEED_PIN GPIO_NUM_3
#define APP_WIFI_CHANNEL 1

#define APP_OLED_SDA GPIO_NUM_5
#define APP_OLED_SCL GPIO_NUM_6
#define APP_OLED_ADDR 0x3C

#define APP_SAMPLE_INTERVAL_MS 100

#define APP_FILTER_WEIGHT_NUM 4
#define APP_FILTER_WEIGHT_DEN 10

#define APP_K_SPEED_X10 8876731UL
#define APP_KPH_PER_MPH_PPM 1609344UL
#define APP_SPEED_DEADZONE_US 2000UL
#define APP_SNAP_TO_ZERO_US 500000UL
#define APP_MAX_INPUT_SPEED_X10 1220

#define APP_OUTPUT_KPH 0
#define APP_ENABLE_SPEED_DIAGNOSTICS 0

#define APP_RADIO_WATCHDOG_MS 10000UL

#define APP_RECEIVER_MAC_BYTES \
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF

// OTA update server — must be a GitHub release asset named ESP32-C3-SPD-Reader.bin
#define APP_OTA_FIRMWARE_URL \
  "https://github.com/beebird-labs/esp32-c3-spd-reader/releases/latest/download/ESP32-C3-SPD-Reader.bin"

// BLE device name (shown during scan)
#define APP_BLE_DEVICE_NAME "SPD-Reader"

// NVS namespace/keys for WiFi credentials used during OTA
#define APP_OTA_NVS_NAMESPACE "ota_creds"
#define APP_OTA_NVS_KEY_SSID  "ssid"
#define APP_OTA_NVS_KEY_PASS  "pass"
