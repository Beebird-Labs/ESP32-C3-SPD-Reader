#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef void (*ota_status_cb_t)(const char *msg);

    esp_err_t ota_manager_init(void);
    esp_err_t ota_manager_save_credentials(const char *ssid, const char *pass);
    void ota_manager_trigger(ota_status_cb_t cb);

    extern volatile bool g_ota_in_progress;

#ifdef __cplusplus
}
#endif
