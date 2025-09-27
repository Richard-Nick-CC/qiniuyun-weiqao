idf_component_register(SRCS "esp32_s3_szp.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_lcd driver spi_flash esp_psram nvs_flash esp_system freertos esp32-camera esp_wifi esp_netif esp_event esp_http_server)
