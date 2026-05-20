#include "ota_manager.h"
#include "project_config.h"
#include "wifi_sta.h"

#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#include <string.h>

static const char *TAG = "ota_manager";

volatile bool g_ota_in_progress = false;

static ota_status_cb_t s_status_cb;

static void notify(const char *msg)
{
    ESP_LOGI(TAG, "%s", msg);
    if (s_status_cb) {
        s_status_cb(msg);
    }
}

static esp_err_t load_credentials(char *ssid, size_t ssid_sz,
                                   char *pass, size_t pass_sz)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(APP_OTA_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_get_str(nvs, APP_OTA_NVS_KEY_SSID, ssid, &ssid_sz);
    if (err == ESP_OK) {
        err = nvs_get_str(nvs, APP_OTA_NVS_KEY_PASS, pass, &pass_sz);
    }
    nvs_close(nvs);
    return err;
}

static void ota_task(void *arg)
{
    g_ota_in_progress = true;

    char ssid[33] = {0};
    char pass[65] = {0};

    notify("Loading credentials");
    esp_err_t err = load_credentials(ssid, sizeof(ssid), pass, sizeof(pass));
    if (err != ESP_OK) {
        notify("No WiFi credentials saved");
        goto done;
    }

    notify("Connecting to WiFi");
    err = wifi_sta_connect(ssid, pass, 30000);
    if (err != ESP_OK) {
        notify("WiFi connect failed");
        goto done;
    }

    notify("Downloading firmware");
    esp_http_client_config_t http_cfg = {
        .url = APP_OTA_FIRMWARE_URL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 60000,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    err = esp_https_ota(&ota_cfg);
    if (err == ESP_OK) {
        notify("Update complete, rebooting");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(err));
    notify("OTA failed");
    wifi_sta_disconnect();

done:
    g_ota_in_progress = false;
    vTaskDelete(NULL);
}

esp_err_t ota_manager_init(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK) {
        if (state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "First boot after OTA — marking app valid");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }
    return ESP_OK;
}

esp_err_t ota_manager_save_credentials(const char *ssid, const char *pass)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(APP_OTA_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_str(nvs, APP_OTA_NVS_KEY_SSID, ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(nvs, APP_OTA_NVS_KEY_PASS, pass);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

void ota_manager_trigger(ota_status_cb_t cb)
{
    if (g_ota_in_progress) {
        ESP_LOGW(TAG, "OTA already in progress");
        return;
    }
    s_status_cb = cb;
    BaseType_t rc = xTaskCreate(ota_task, "ota_task", 8192, NULL, 5, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Failed to create OTA task");
    }
}
