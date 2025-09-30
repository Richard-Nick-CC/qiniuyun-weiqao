#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

/*******************************************************************************/
/***************************  超声波传感器 ↓   ****************************/

// 超声波传感器引脚定义
#define ULTRASONIC_ECHO_PIN     (GPIO_NUM_35)   // ECHO引脚
#define ULTRASONIC_TRIG_PIN     (GPIO_NUM_38)   // TRIG引脚

// 超声波传感器参数
#define ULTRASONIC_TIMEOUT_MS   (50)            // 超时时间(ms)
#define ULTRASONIC_MAX_DISTANCE (400)           // 最大测量距离(cm)
#define SOUND_SPEED             (343.0f)        // 声速(m/s)

/**
 * @brief 初始化超声波传感器
 * 
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t ultrasonic_sensor_init(void);

/**
 * @brief 测量距离
 * 
 * @param distance 测量的距离(cm)，成功返回有效距离，失败返回-1
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t ultrasonic_sensor_measure(float *distance);

/**
 * @brief 启动超声波传感器触发
 * 
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t ultrasonic_sensor_trigger(void);

/**
 * @brief 获取最新的距离测量结果
 * 
 * @param distance 最新的距离测量结果(cm)
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t ultrasonic_sensor_get_last_distance(float *distance);

/**
 * @brief 创建超声波测距任务
 * 
 * @param task_priority 任务优先级
 * @param task_stack_size 任务栈大小
 * @param task_handle 任务句柄（可选，可传入NULL）
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t ultrasonic_sensor_create_task(uint8_t task_priority, uint32_t task_stack_size, TaskHandle_t *task_handle);

/***************************  超声波传感器 ↑   ****************************/
/*******************************************************************************/