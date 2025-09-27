#include <stdio.h>
#include "esp32_s3_szp.h"
#include "logo_en_240x240_lcd.h"
#include "yingwu.h"
#include <esp_system.h>
#include <nvs_flash.h>
#include <string.h>
#include <esp_psram.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "img_converters.h"
#include "camera_wifi.h"
#include "motor_driver.h"
#include "ultrasonic_sensor.h"

// static const char *TAG = "main";




// WiFi 连接状态标志
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// 全局变量声明 - 用于在其他文件中访问
EventGroupHandle_t get_wifi_event_group(void)
{
    return s_wifi_event_group;
}

// ------------------ app_main ------------------
void app_main(void)
{
    // 创建 WiFi 事件组
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE("main", "Failed to create WiFi event group");
    }
    
    // LCD 初始化
    bsp_i2c_init();
    pca9557_init();
    bsp_lcd_init();
    lcd_draw_pictrue(0, 0, 320, 240, gImage_yingwu);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // 摄像头初始化
    bsp_camera_init();
    app_camera_lcd();

    size_t psram_size = esp_psram_get_size();
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // 电机驱动初始化
    ESP_LOGI("main", "Initializing motor driver...");
    esp_err_t motor_ret = motor_driver_init();
    if (motor_ret != ESP_OK) {
        ESP_LOGE("main", "Failed to initialize motor driver: %s", esp_err_to_name(motor_ret));
    } else {
        ESP_LOGI("main", "Motor driver initialized successfully");
        // 停止所有电机（确保初始状态安全）
        motor_stop_all();
    }
    
    // 超声波传感器初始化
    ESP_LOGI("main", "Initializing ultrasonic sensor...");
    esp_err_t ultrasonic_ret = ultrasonic_sensor_init();
    if (ultrasonic_ret != ESP_OK) {
        ESP_LOGE("main", "Failed to initialize ultrasonic sensor: %s", esp_err_to_name(ultrasonic_ret));
    } else {
        ESP_LOGI("main", "Ultrasonic sensor initialized successfully");
    }
    
    // WiFi 初始化
    ESP_LOGI("main", "Initializing WiFi...");
    wifi_init_sta();
    
    // 等待 WiFi 连接成功或失败
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY
    );
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI("main", "Connected to WiFi successfully");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE("main", "Failed to connect to WiFi");
    }
    
    // 清理资源
    vEventGroupDelete(s_wifi_event_group);
}
