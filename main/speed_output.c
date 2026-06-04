#include "speed_output.h"

#include <string.h>

#include "project_config.h"

#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"

static const char *TAG = "SPEED_OUTPUT";

static const uint8_t s_receiver_mac[ESP_NOW_ETH_ALEN] = {APP_RECEIVER_MAC_BYTES};
static volatile uint32_t s_last_send_ok_ms = 0;

typedef struct __attribute__((packed))
{
    uint8_t unit;
    uint16_t speed;
} speed_packet_t;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

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

esp_err_t speed_output_init(void)
{
    esp_err_t err = esp_now_init();
    if (err != ESP_OK)
    {
        return err;
    }

    err = esp_now_register_send_cb(on_send);
    if (err != ESP_OK)
    {
        return err;
    }

    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, s_receiver_mac, sizeof(s_receiver_mac));
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.channel = APP_WIFI_CHANNEL;
    peer_info.encrypt = false;

    err = esp_now_add_peer(&peer_info);
    if (err != ESP_OK)
    {
        return err;
    }

    s_last_send_ok_ms = now_ms();
    ESP_LOGI(TAG, "speed_output_ready backend=espnow channel=%d", APP_WIFI_CHANNEL);
    return ESP_OK;
}

esp_err_t speed_output_send(speed_output_unit_t unit, uint16_t speed_x10)
{
    speed_packet_t pkt = {
        .unit = (uint8_t)unit,
        .speed = speed_x10,
    };

    esp_err_t err = esp_now_send(s_receiver_mac, (uint8_t *)&pkt, sizeof(pkt));
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "espnow_send_failed err=%s", esp_err_to_name(err));
    }
    return err;
}

uint32_t speed_output_last_success_ms(void)
{
    return s_last_send_ok_ms;
}
