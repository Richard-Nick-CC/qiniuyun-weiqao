idf_component_register(SRCS "esp32_s3_szp.c" "main.c" "motor_driver.c" "ultrasonic_sensor.c" "line_follower.c" "camera_wifi.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_lcd driver spi_flash esp_psram nvs_flash esp_system freertos esp32-camera esp_wifi esp_netif esp_event esp_http_server esp_timer)
