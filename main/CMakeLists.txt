idf_component_register(
    SRCS "main.c" "oled.c" "ble_prov.c" "ota_manager.c" "wifi_sta.c" "speed_output.c"
    INCLUDE_DIRS "."
    REQUIRES
        app_update
        bt
        esp-tls
        esp_driver_gpio
        esp_driver_i2c
        esp_event
        esp_http_client
        esp_https_ota
        esp_netif
        esp_timer
        esp_wifi
        mbedtls
        nvs_flash
)
