#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 电机引脚定义 - 每个电机使用两个PWM引脚
#define MOTOR_LEFT_PWM1_PIN    (GPIO_NUM_11)  // 左电机PWM引脚1
#define MOTOR_LEFT_PWM2_PIN    (GPIO_NUM_12)  // 左电机PWM引脚2

#define MOTOR_RIGHT_PWM1_PIN   (GPIO_NUM_14)  // 右电机PWM引脚1
#define MOTOR_RIGHT_PWM2_PIN   (GPIO_NUM_13)  // 右电机PWM引脚2

// LEDC配置
#define LEDC_TIMER             LEDC_TIMER_0
#define LEDC_MODE              LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ           (10000)  // 10kHz PWM频率
#define LEDC_RESOLUTION        LEDC_TIMER_10_BIT  // 10位分辨率 (0-1023)

// LEDC通道定义
#define LEDC_CHANNEL_LEFT1     LEDC_CHANNEL_0  // 左电机PWM1通道
#define LEDC_CHANNEL_LEFT2     LEDC_CHANNEL_1  // 左电机PWM2通道
#define LEDC_CHANNEL_RIGHT1    LEDC_CHANNEL_2  // 右电机PWM1通道
#define LEDC_CHANNEL_RIGHT2    LEDC_CHANNEL_3  // 右电机PWM2通道

// 电机方向定义
typedef enum {
    MOTOR_FORWARD = 0,
    MOTOR_BACKWARD = 1
} motor_direction_t;

// 电机ID定义
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} motor_id_t;

/**
 * @brief 初始化电机驱动
 * 
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t motor_driver_init(void);

/**
 * @brief 控制单个电机
 * 
 * @param motor_id 电机ID（MOTOR_LEFT或MOTOR_RIGHT）
 * @param direction 电机方向（MOTOR_FORWARD或MOTOR_BACKWARD）
 * @param speed 电机速度（0-100，百分比）
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t motor_control(motor_id_t motor_id, motor_direction_t direction, uint8_t speed);

/**
 * @brief 同时控制两个电机
 * 
 * @param left_dir 左电机方向
 * @param left_speed 左电机速度（0-100，百分比）
 * @param right_dir 右电机方向
 * @param right_speed 右电机速度（0-100，百分比）
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t motor_control_both(motor_direction_t left_dir, uint8_t left_speed, 
                           motor_direction_t right_dir, uint8_t right_speed);

/**
 * @brief 停止单个电机
 * 
 * @param motor_id 电机ID
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t motor_stop(motor_id_t motor_id);

/**
 * @brief 停止所有电机
 * 
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t motor_stop_all(void);

#endif /* MOTOR_DRIVER_H */
