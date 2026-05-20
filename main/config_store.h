#pragma once

#include <stdint.h>
#include "esp_err.h"

#define CONFIG_OTA_URL_MAX 256

typedef struct {
    uint32_t filter_weight_num;
    uint32_t filter_weight_den;
    uint32_t k_speed_x10;
    uint32_t speed_deadzone_us;
    uint32_t snap_to_zero_us;
    uint8_t  enable_speed_diagnostics;
    uint32_t radio_watchdog_ms;
    uint8_t  receiver_mac[6];
    char     ota_url[CONFIG_OTA_URL_MAX];
} app_config_t;

extern app_config_t g_config;

esp_err_t config_store_init(void);
esp_err_t config_store_save(void);
