#pragma once

#include "esp_err.h"

esp_err_t wifi_sta_connect(const char *ssid, const char *password, int timeout_ms);
void wifi_sta_disconnect(void);
