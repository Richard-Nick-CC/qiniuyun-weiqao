#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 循迹模块I2C地址
#define LineFollower4ch_ADDR         0x78

// 避障配置常量
#define OBSTACLE_DETECTION_DISTANCE  20.0f     // 障碍物检测距离(cm)
#define OBSTACLE_SAFE_DISTANCE       30.0f     // 安全距离(cm)
#define OBSTACLE_BACKUP_TIME         500       // 后退时间(ms)
#define OBSTACLE_TURN_TIME           800       // 转向时间(ms)
#define OBSTACLE_RETURN_TIME         1000      // 返回循迹时间(ms)
#define OBSTACLE_CHECK_INTERVAL      100       // 障碍物检测间隔(ms)

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

// 避障状态定义
typedef enum {
    OBSTACLE_STATE_NONE = 0,        // 无障碍物
    OBSTACLE_STATE_DETECTED,        // 检测到障碍物
    OBSTACLE_STATE_AVOIDING,        // 正在避障
    OBSTACLE_STATE_RETURNING        // 返回循迹
} obstacle_state_t;

// 避障行为定义
typedef enum {
    OBSTACLE_ACTION_STOP = 0,       // 停止
    OBSTACLE_ACTION_BACKUP,         // 后退
    OBSTACLE_ACTION_TURN_LEFT,      // 左转
    OBSTACLE_ACTION_TURN_RIGHT,     // 右转
    OBSTACLE_ACTION_CONTINUE        // 继续前进
} obstacle_action_t;

// 循迹参数结构体
typedef struct {
    uint8_t sensor_value;           // 原始传感器值
    uint8_t sensor_state[4];        // 各传感器状态 (0=检测到黑线, 1=检测到白线)
    line_follower_mode_t mode;      // 工作模式
    uint8_t speed;                  // 基础速度
    uint8_t turn_speed_diff;        // 转弯时的速度差值
    
    // 避障相关字段
    obstacle_state_t obstacle_state;    // 避障状态
    obstacle_action_t obstacle_action;  // 当前避障行为
    float obstacle_distance;            // 检测到的障碍物距离(cm)
    uint32_t obstacle_start_time;       // 避障开始时间(ms)
    uint8_t obstacle_enabled;           // 避障功能使能标志
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
 * @param state 传感器状态数组，长度为4
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t line_follower_get_state(uint8_t state[4]);

/**
 * @brief 启用或禁用避障功能
 * 
 * @param enabled 1=启用避障，0=禁用避障
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t line_follower_set_obstacle_avoidance(uint8_t enabled);

/**
 * @brief 获取当前避障状态
 * 
 * @param state 当前避障状态
 * @param distance 当前检测到的障碍物距离
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t line_follower_get_obstacle_state(obstacle_state_t *state, float *distance);

#endif // LINE_FOLLOWER_H