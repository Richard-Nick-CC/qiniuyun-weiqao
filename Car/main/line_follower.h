#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 循迹模块I2C地址
#define LineFollower4ch_ADDR         0x78

// 循迹模块寄存器地址
#define LineFollower4ch_data_reg_ADDR    0x01
#define LineFollower4ch_config_reg_ADDR  0x02

// 循迹传感器通道定义
#define LINE_FOLLOWER_CHANNEL_1     (1 << 0)
#define LINE_FOLLOWER_CHANNEL_2     (1 << 1)
#define LINE_FOLLOWER_CHANNEL_3     (1 << 2)
#define LINE_FOLLOWER_CHANNEL_4     (1 << 3)

// 循迹模式定义
typedef enum {
    LINE_FOLLOWER_MODE_READ = 0,    // 读取模式
    LINE_FOLLOWER_MODE_CALIBRATE    // 校准模式
} line_follower_mode_t;

// 循迹参数结构体
typedef struct {
    uint8_t sensor_value;           // 原始传感器值
    uint8_t sensor_state[4];        // 各传感器状态 (0=检测到黑线, 1=检测到白线)
    line_follower_mode_t mode;      // 工作模式
    uint8_t speed;                  // 基础速度
    uint8_t turn_speed_diff;        // 转弯时的速度差值
} line_follower_t;

/**
 * @brief 初始化循迹模块
 * 
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t line_follower_init(void);

/**
 * @brief 读取循迹传感器值
 * 
 * @return uint8_t 传感器读数，每一位代表一个传感器
 */
uint8_t read_line_follow(void);

/**
 * @brief I2C写入一个字节
 * 
 * @param reg_address 寄存器地址
 * @param val 要写入的值
 */
void writeByte(unsigned char reg_address, unsigned char val);

/**
 * @brief I2C读取一个字节
 * 
 * @param reg_address 寄存器地址
 * @return unsigned char 读取到的值
 */
unsigned char readByte(unsigned char reg_address);

/**
 * @brief 启动循迹任务
 * 
 * @param stack_size 任务栈大小
 * @param priority 任务优先级
 * @param speed 基础速度
 * @param turn_speed_diff 转弯速度差值
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t line_follower_start_task(uint16_t stack_size, UBaseType_t priority, uint8_t speed, uint8_t turn_speed_diff);

/**
 * @brief 停止循迹任务
 * 
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t line_follower_stop_task(void);

/**
 * @brief 设置循迹速度
 * 
 * @param speed 基础速度 (0-100)
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t line_follower_set_speed(uint8_t speed);

/**
 * @brief 获取循迹传感器状态
 * 
 * @param state 传感器状态数组 (0=检测到黑线, 1=检测到白线)
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t line_follower_get_state(uint8_t state[4]);

#endif // LINE_FOLLOWER_H