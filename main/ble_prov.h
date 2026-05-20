#pragma once

#include "esp_err.h"

esp_err_t ble_prov_init(void);
void ble_prov_notify_status(const char *msg);
