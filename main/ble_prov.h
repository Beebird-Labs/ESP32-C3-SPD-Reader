#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    esp_err_t ble_prov_init(void);
    void ble_prov_notify_status(const char *msg);
    void ble_prov_disable(void);

#ifdef __cplusplus
}
#endif
