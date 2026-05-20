#include "ble_prov.h"
#include "ota_manager.h"
#include "project_config.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include <string.h>

static const char *TAG = "ble_prov";

// 128-bit UUIDs, little-endian (base: E8F0xxxx-1B25-4F47-82AB-DE1E2AB9A87C)
// E8F00001 = provisioning service
// E8F00002 = WiFi SSID (write)
// E8F00003 = WiFi password (write, saves credentials to NVS)
// E8F00004 = OTA trigger (write any byte)
// E8F00005 = status (read + notify)
#define UUID_SVC  BLE_UUID128_DECLARE(0x7C,0xA8,0xB9,0x2A,0x1E,0xDE,0xAB,0x82,0x47,0x4F,0x25,0x1B,0x01,0x00,0xF0,0xE8)
#define UUID_SSID BLE_UUID128_DECLARE(0x7C,0xA8,0xB9,0x2A,0x1E,0xDE,0xAB,0x82,0x47,0x4F,0x25,0x1B,0x02,0x00,0xF0,0xE8)
#define UUID_PASS BLE_UUID128_DECLARE(0x7C,0xA8,0xB9,0x2A,0x1E,0xDE,0xAB,0x82,0x47,0x4F,0x25,0x1B,0x03,0x00,0xF0,0xE8)
#define UUID_TRIG BLE_UUID128_DECLARE(0x7C,0xA8,0xB9,0x2A,0x1E,0xDE,0xAB,0x82,0x47,0x4F,0x25,0x1B,0x04,0x00,0xF0,0xE8)
#define UUID_STAT BLE_UUID128_DECLARE(0x7C,0xA8,0xB9,0x2A,0x1E,0xDE,0xAB,0x82,0x47,0x4F,0x25,0x1B,0x05,0x00,0xF0,0xE8)

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_status_val_handle;
static uint8_t s_own_addr_type;
static bool s_enabled = true;

static char s_status_buf[64] = "idle";
static char s_ssid_buf[33]   = {0};
static char s_pass_buf[65]   = {0};

// ---------------------------------------------------------------------------
// GATT characteristic callbacks
// ---------------------------------------------------------------------------

static int chr_ssid_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len >= sizeof(s_ssid_buf)) {
        len = sizeof(s_ssid_buf) - 1;
    }
    os_mbuf_copydata(ctxt->om, 0, len, s_ssid_buf);
    s_ssid_buf[len] = '\0';
    ESP_LOGI(TAG, "SSID set: %s", s_ssid_buf);
    return 0;
}

static int chr_pass_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len >= sizeof(s_pass_buf)) {
        len = sizeof(s_pass_buf) - 1;
    }
    os_mbuf_copydata(ctxt->om, 0, len, s_pass_buf);
    s_pass_buf[len] = '\0';
    ESP_LOGI(TAG, "Password received — saving credentials");

    esp_err_t err = ota_manager_save_credentials(s_ssid_buf, s_pass_buf);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save credentials: %s", esp_err_to_name(err));
        return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}

static int chr_trigger_cb(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    ESP_LOGI(TAG, "OTA trigger received");
    ota_manager_trigger(ble_prov_notify_status);
    return 0;
}

static int chr_status_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }
    return os_mbuf_append(ctxt->om, s_status_buf, strlen(s_status_buf));
}

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = UUID_SVC,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = UUID_SSID,
                .access_cb  = chr_ssid_cb,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid       = UUID_PASS,
                .access_cb  = chr_pass_cb,
                .flags      = BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid       = UUID_TRIG,
                .access_cb  = chr_trigger_cb,
                .flags      = BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid       = UUID_STAT,
                .access_cb  = chr_status_cb,
                .val_handle = &s_status_val_handle,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 },
        },
    },
    { 0 },
};

// ---------------------------------------------------------------------------
// Advertising
// ---------------------------------------------------------------------------

static void start_advertising(void)
{
    if (!s_enabled) {
        return;
    }
    // Advertising payload: flags + complete local name
    struct ble_hs_adv_fields adv_fields = {0};
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.name = (uint8_t *)APP_BLE_DEVICE_NAME;
    adv_fields.name_len = (uint8_t)strlen(APP_BLE_DEVICE_NAME);
    adv_fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields failed: %d", rc);
        return;
    }

    // Scan response: 128-bit service UUID so companion apps can filter by service
    static ble_uuid128_t svc_uuid128 = {
        .u.type = BLE_UUID_TYPE_128,
        .value  = {0x7C,0xA8,0xB9,0x2A,0x1E,0xDE,0xAB,0x82,
                   0x47,0x4F,0x25,0x1B,0x01,0x00,0xF0,0xE8},
    };
    struct ble_hs_adv_fields rsp_fields = {0};
    rsp_fields.uuids128 = &svc_uuid128;
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp_fields);

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min  = BLE_GAP_ADV_ITVL_MS(100);
    adv_params.itvl_max  = BLE_GAP_ADV_ITVL_MS(200);

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising as '%s'", APP_BLE_DEVICE_NAME);
    }
}

// ---------------------------------------------------------------------------
// GAP event handler
// ---------------------------------------------------------------------------

int ble_gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "BLE connected, conn=%d", event->connect.conn_handle);
            s_conn_handle = event->connect.conn_handle;
        } else {
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected, reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        start_advertising();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        break;

    default:
        break;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// NimBLE host callbacks
// ---------------------------------------------------------------------------

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset, reason=%d", reason);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed: %d", rc);
        return;
    }
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    start_advertising();
}

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void ble_prov_notify_status(const char *msg)
{
    strlcpy(s_status_buf, msg, sizeof(s_status_buf));
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, strlen(msg));
    if (!om) {
        return;
    }
    int rc = ble_gatts_notify_custom(s_conn_handle, s_status_val_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "notify failed: %d", rc);
    }
}

void ble_prov_disable(void)
{
    if (!s_enabled) {
        return;
    }
    s_enabled = false;
    ESP_LOGI(TAG, "OTA disabled — vehicle in motion");
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    ble_gap_adv_stop();
}

esp_err_t ble_prov_init(void)
{
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        return err;
    }

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb  = on_sync;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) {
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) {
        return ESP_FAIL;
    }

    ble_svc_gap_device_name_set(APP_BLE_DEVICE_NAME);

    nimble_port_freertos_init(ble_host_task);
    return ESP_OK;
}
