#include "ultrasonic_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ultrasonic_test";

/**
 * @brief 超声波传感器测试任务
 * 
 * @param pvParameters 任务参数（未使用）
 */
void ultrasonic_test_task(void *pvParameters)
{
    esp_err_t ret;
    float distance = 0.0f;
    
    // 初始化超声波传感器
    ret = ultrasonic_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ultrasonic sensor: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Ultrasonic sensor test started");
    
    while (1) {
        // 测量距离
        ret = ultrasonic_sensor_measure(&distance);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Distance: %.2f cm", distance);
        } else {
            ESP_LOGE(TAG, "Failed to measure distance: %s", esp_err_to_name(ret));
        }
        
        // 每500ms测量一次
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief 启动超声波传感器测试任务
 * 
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t start_ultrasonic_test(void)
{
    // 创建超声波传感器测试任务
    if (xTaskCreate(ultrasonic_test_task, "ultrasonic_test_task", 2048, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ultrasonic test task");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}