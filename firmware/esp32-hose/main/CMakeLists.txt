idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_adc esp_driver_ledc esp_event esp_netif esp_timer esp_wifi nvs_flash
)

