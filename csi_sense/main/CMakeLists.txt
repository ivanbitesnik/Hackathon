idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES esp_wifi esp_netif esp_event esp_timer nvs_flash esp_driver_gpio
                       INCLUDE_DIRS "")
