idf_component_register(SRCS "ultrasonic_sensor.c" "ultrasonic_test.c" "motor_test.c" "motor_driver.c" "camera_wifi.c" "main.c" "esp32_s3_szp.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_lcd driver spi_flash esp_psram nvs_flash esp_system freertos esp32-camera esp_wifi esp_netif esp_event esp_http_server esp_timer esp_rom)
