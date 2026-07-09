idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_event esp_http_server esp_lcd esp_netif esp_timer esp_wifi
             esp_driver_gpio esp_driver_spi esp_driver_uart nvs_flash spi_flash
)
