#include "motor_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "motor_test";

/**
 * @brief 电机测试任务
 * 
 * @param pvParameters 任务参数（未使用）
 */
void motor_test_task(void *pvParameters)
{
    esp_err_t ret;
    
    // 初始化电机驱动
    ret = motor_driver_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize motor driver: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Motor test started");
    
    while (1) {
        // 测试1: 前进3秒
        ESP_LOGI(TAG, "Test 1: Moving forward");
        ret = motor_control_both(MOTOR_FORWARD, 50, MOTOR_FORWARD, 50);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to move forward: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        // 停止1秒
        ESP_LOGI(TAG, "Stopping");
        ret = motor_stop_all();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop motors: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 测试2: 后退3秒
        ESP_LOGI(TAG, "Test 2: Moving backward");
        ret = motor_control_both(MOTOR_BACKWARD, 50, MOTOR_BACKWARD, 50);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to move backward: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        // 停止1秒
        ESP_LOGI(TAG, "Stopping");
        ret = motor_stop_all();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop motors: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 测试3: 左转3秒
        ESP_LOGI(TAG, "Test 3: Turning left");
        ret = motor_control(MOTOR_LEFT, MOTOR_BACKWARD, 40);
        ret |= motor_control(MOTOR_RIGHT, MOTOR_FORWARD, 40);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to turn left: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        // 停止1秒
        ESP_LOGI(TAG, "Stopping");
        ret = motor_stop_all();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop motors: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 测试4: 右转3秒
        ESP_LOGI(TAG, "Test 4: Turning right");
        ret = motor_control(MOTOR_LEFT, MOTOR_FORWARD, 40);
        ret |= motor_control(MOTOR_RIGHT, MOTOR_BACKWARD, 40);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to turn right: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        // 停止5秒
        ESP_LOGI(TAG, "Stopping");
        ret = motor_stop_all();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop motors: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/**
 * @brief 启动电机测试任务
 * 
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t start_motor_test(void)
{
    // 创建电机测试任务
    if (xTaskCreate(motor_test_task, "motor_test_task", 2048, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create motor test task");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}