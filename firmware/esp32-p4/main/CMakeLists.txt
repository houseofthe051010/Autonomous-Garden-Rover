idf_component_register(SRCS "main.c" "network_ota.c" "rover_link.c" "stepper_link.c" "odesc_link.c" "speaker.c"
                       INCLUDE_DIRS "."
                       REQUIRES app_update driver esp_driver_i2c esp_driver_i2s
                                esp_driver_sdmmc esp_event esp_http_server
                                esp_netif esp_timer esp_wifi fatfs heap lwip mbedtls
                                nvs_flash sdmmc wear_levelling)
