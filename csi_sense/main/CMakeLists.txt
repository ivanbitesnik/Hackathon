idf_component_register(SRCS "display.c" "main.c"
                       PRIV_REQUIRES esp_lcd esp_driver_spi esp_driver_i2s esp_wifi esp_netif esp_event esp_timer nvs_flash esp_driver_gpio
                       INCLUDE_DIRS "")
