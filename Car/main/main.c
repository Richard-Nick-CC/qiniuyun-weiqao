#include <stdio.h>
#include "esp32_s3_szp.h"
#include "logo_en_240x240_lcd.h"
#include <esp_system.h>
#include <nvs_flash.h>
#include <string.h>
#include <esp_psram.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_driver.h"
#include "ultrasonic_sensor.h"
#include "line_follower.h"
#include "camera_wifi.h"

static const char *TAG = "main";










/**
 * @brief 打印超声波距离和循迹状态任务
 * 
 * @param pvParameters 任务参数（未使用）
 */
static void print_sensor_data_task(void *pvParameters)
{
    float distance = 0.0f;
    uint8_t sensor_state[4] = {0};
    int counter = 0;
    
    while (1) {
        counter++;
        ESP_LOGI(TAG, "--- Sensor Data Update #%d ---", counter);
        
        // 获取最新的距离测量结果
        if (ultrasonic_sensor_get_last_distance(&distance) == ESP_OK) {
            printf("超声波距离: %.2f cm\n", distance);
        } else {
            printf("获取距离失败\n");
        }
        
        // 直接读取一次原始传感器值进行对比
        uint8_t raw_value = read_line_follow();
        ESP_LOGI(TAG, "直接读取的原始传感器值: 0x%02X", raw_value);
        
        // 获取循迹状态
        esp_err_t state_ret = line_follower_get_state(sensor_state);
        if (state_ret == ESP_OK) {
            printf("循迹状态 - 传感器1: %d, 传感器2: %d, 传感器3: %d, 传感器4: %d\n", 
                   sensor_state[0], sensor_state[1], sensor_state[2], sensor_state[3]);
            
            // 根据原始值计算的状态（用于交叉验证）
            uint8_t calc_state[4] = {
                !(raw_value & 0x01),
                !((raw_value >> 1) & 0x01),
                !((raw_value >> 2) & 0x01),
                !((raw_value >> 3) & 0x01)
            };
            printf("根据原始值计算的状态 - 传感器1: %d, 传感器2: %d, 传感器3: %d, 传感器4: %d\n", 
                   calc_state[0], calc_state[1], calc_state[2], calc_state[3]);
        } else {
            printf("获取循迹状态失败: %s\n", esp_err_to_name(state_ret));
        }
          
        // 每秒打印一次
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ------------------ app_main ------------------
void app_main(void)
{ 
    // 首先初始化I2C，确保所有I2C设备工作正常
    ESP_LOGI(TAG, "Initializing I2C...");
    esp_err_t i2c_ret = bsp_i2c_init();
    if (i2c_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(i2c_ret));
    }
    
    motor_driver_init();        // 电机驱动初始化
    ultrasonic_sensor_init();   // 超声波初始化
    ultrasonic_sensor_create_task(1, 4096, NULL); // 增加堆栈大小到4096
    
    // 启动循迹任务（内部会调用line_follower_init()）
    printf("=== 准备启动循迹任务 ===\n");
    ESP_LOGI(TAG, "Starting line follower task...");
    // 提升 line_follower 模块日志级别，确保速度与原始值日志可见
    esp_log_level_set("line_follower", ESP_LOG_DEBUG);
    
    // 检查可用内存
    printf("启动循迹任务前的可用内存: %lu bytes\n", esp_get_free_heap_size());
    printf("调用 line_follower_start_task(4096, 3, 55, 20)\n");
    esp_err_t line_follower_ret = line_follower_start_task(4096, 3, 55, 20); // 提高基础速度到55%
    printf("line_follower_start_task 返回值: %d\n", line_follower_ret);
    if (line_follower_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start line follower task: %s", esp_err_to_name(line_follower_ret));
        printf("!!! 循迹任务启动失败 !!!\n");
    } else {
        printf("循迹任务启动成功\n");
    }
    printf("启动循迹任务后的可用内存: %lu bytes\n", esp_get_free_heap_size());
    
    // 添加直接测试：立即读取一次循迹状态
    uint8_t test_state[4] = {0};
    vTaskDelay(pdMS_TO_TICKS(100)); // 短暂延迟，确保任务已启动
    esp_err_t test_ret = line_follower_get_state(test_state);
    if (test_ret == ESP_OK) {
        ESP_LOGI(TAG, "Initial sensor state test - S1:%d, S2:%d, S3:%d, S4:%d", 
                 test_state[0], test_state[1], test_state[2], test_state[3]);
    } else {
        ESP_LOGE(TAG, "Initial sensor state test failed: %s", esp_err_to_name(test_ret));
    }
    
    // 创建打印传感器数据任务（包括超声波距离和循迹状态）
    xTaskCreate(print_sensor_data_task, "print_sensor_data", 4096, NULL, 2, NULL); // 增加栈大小到4096以防止栈溢出
    // 添加上电默认电机控制：以较低速度前进，确保电机能正常工作
    // 这样即使在未检测到黑线时，电机也能运转
    ESP_LOGI(TAG, "Setting motors to move forward at 20%% speed by default");
    motor_control_both(MOTOR_FORWARD, 60, MOTOR_FORWARD, 60);
    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // 摄像头初始化
    bsp_camera_init();
    app_camera_lcd();

    size_t psram_size = esp_psram_get_size();
    printf("PSRAM size: %d bytes\n", psram_size);

    ESP_ERROR_CHECK(nvs_flash_init());

    // WiFi 初始化
    camera_wifi_init();
}