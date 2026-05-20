#include "config_store.h"
#include "project_config.h"

#include "esp_log.h"
#include "nvs.h"

#include <string.h>

#define NVS_NS "app_config"

static const char *TAG = "config_store";

app_config_t g_config;

static const uint8_t s_default_mac[6] = {APP_RECEIVER_MAC_BYTES};

esp_err_t config_store_init(void)
{
    // Apply compile-time defaults first
    g_config.filter_weight_num        = APP_FILTER_WEIGHT_NUM;
    g_config.filter_weight_den        = APP_FILTER_WEIGHT_DEN;
    g_config.k_speed_x10              = APP_K_SPEED_X10;
    g_config.speed_deadzone_us        = APP_SPEED_DEADZONE_US;
    g_config.snap_to_zero_us          = APP_SNAP_TO_ZERO_US;
    g_config.enable_speed_diagnostics = APP_ENABLE_SPEED_DIAGNOSTICS;
    g_config.radio_watchdog_ms        = APP_RADIO_WATCHDOG_MS;
    memcpy(g_config.receiver_mac, s_default_mac, 6);
    strlcpy(g_config.ota_url, APP_OTA_FIRMWARE_URL, sizeof(g_config.ota_url));

    // Override with any NVS-stored values
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No stored config — using compile-time defaults");
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    uint32_t u32;
    uint8_t  u8;
    size_t   sz;

    if (nvs_get_u32(nvs, "flt_num",   &u32) == ESP_OK) g_config.filter_weight_num   = u32;
    if (nvs_get_u32(nvs, "flt_den",   &u32) == ESP_OK) g_config.filter_weight_den   = u32;
    if (nvs_get_u32(nvs, "k_speed",   &u32) == ESP_OK) g_config.k_speed_x10         = u32;
    if (nvs_get_u32(nvs, "deadzone",  &u32) == ESP_OK) g_config.speed_deadzone_us   = u32;
    if (nvs_get_u32(nvs, "snap_zero", &u32) == ESP_OK) g_config.snap_to_zero_us     = u32;
    // enable_speed_diagnostics is intentionally not persisted — always resets to default
    if (nvs_get_u32(nvs, "watchdog",  &u32) == ESP_OK) g_config.radio_watchdog_ms   = u32;

    sz = sizeof(g_config.receiver_mac);
    nvs_get_blob(nvs, "rcvr_mac", g_config.receiver_mac, &sz);

    sz = sizeof(g_config.ota_url);
    nvs_get_str(nvs, "ota_url", g_config.ota_url, &sz);

    nvs_close(nvs);
    ESP_LOGI(TAG, "Config loaded from NVS");
    return ESP_OK;
}

esp_err_t config_store_save(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    nvs_set_u32 (nvs, "flt_num",   g_config.filter_weight_num);
    nvs_set_u32 (nvs, "flt_den",   g_config.filter_weight_den);
    nvs_set_u32 (nvs, "k_speed",   g_config.k_speed_x10);
    nvs_set_u32 (nvs, "deadzone",  g_config.speed_deadzone_us);
    nvs_set_u32 (nvs, "snap_zero", g_config.snap_to_zero_us);
    // enable_speed_diagnostics intentionally omitted — not persisted
    nvs_set_u32 (nvs, "watchdog",  g_config.radio_watchdog_ms);
    nvs_set_blob(nvs, "rcvr_mac",  g_config.receiver_mac, 6);
    nvs_set_str (nvs, "ota_url",   g_config.ota_url);

    err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}
