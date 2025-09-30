#include "motor_driver.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "motor_driver";

/**
 * @brief 初始化电机驱动
 * 
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t motor_driver_init(void)
{
    esp_err_t ret = ESP_OK;
    
    // 初始化LEDC定时器
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_RESOLUTION,
        .freq_hz          = LEDC_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 初始化左电机PWM1通道
    ledc_channel_config_t ledc_channel_left1 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_LEFT1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_LEFT_PWM1_PIN,
        .duty           = 0,  // 初始占空比为0，因为后续会调用motor_stop_all()
        .hpoint         = 0,
    };
    
    ret = ledc_channel_config(&ledc_channel_left1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure left motor PWM1 channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 初始化左电机PWM2通道
    ledc_channel_config_t ledc_channel_left2 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_LEFT2,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_LEFT_PWM2_PIN,
        .duty           = 0,  // 初始占空比为0，因为后续会调用motor_stop_all()
        .hpoint         = 0,
    };
    
    ret = ledc_channel_config(&ledc_channel_left2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure left motor PWM2 channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 初始化右电机PWM1通道
    ledc_channel_config_t ledc_channel_right1 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_RIGHT1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_RIGHT_PWM1_PIN,
        .duty           = 0,  // 初始占空比为0，因为后续会调用motor_stop_all()
        .hpoint         = 0,
    };
    
    ret = ledc_channel_config(&ledc_channel_right1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure right motor PWM1 channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 初始化右电机PWM2通道
    ledc_channel_config_t ledc_channel_right2 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_RIGHT2,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_RIGHT_PWM2_PIN,
        .duty           = 0,  // 初始占空比为0，因为后续会调用motor_stop_all()
        .hpoint         = 0,
    };
    
    ret = ledc_channel_config(&ledc_channel_right2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure right motor PWM2 channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 确保所有电机初始状态为停止
    motor_stop_all();
    
    ESP_LOGI(TAG, "Motor driver initialized successfully with dual PWM mode");
    return ESP_OK;
}

/**
 * @brief 控制单个电机
 * 
 * @param motor_id 电机ID（MOTOR_LEFT或MOTOR_RIGHT）
 * @param direction 电机方向（MOTOR_FORWARD或MOTOR_BACKWARD）
 * @param speed 电机速度（0-100，百分比）
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t motor_control(motor_id_t motor_id, motor_direction_t direction, uint8_t speed)
{
    esp_err_t ret = ESP_OK;
    
    // 检查参数有效性
    if (speed > 100) {
        ESP_LOGE(TAG, "Invalid speed value: %d (must be 0-100)", speed);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 计算占空比（转换为LEDC分辨率范围）
    uint32_t duty = (speed * (1 << LEDC_RESOLUTION)) / 100;
    
    if (motor_id == MOTOR_LEFT) {
        if (direction == MOTOR_FORWARD) {
            // 前进方向：PWM1为speed，PWM2为0
            ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT1, duty);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set left motor PWM1 duty: %s", esp_err_to_name(ret));
                return ret;
            }
            ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LEFT1);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to update left motor PWM1 duty: %s", esp_err_to_name(ret));
                return ret;
            }
            
            ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT2, 0);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set left motor PWM2 duty: %s", esp_err_to_name(ret));
                return ret;
            }
            ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LEFT2);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to update left motor PWM2 duty: %s", esp_err_to_name(ret));
                return ret;
            }
        } else {
            // 后退方向：PWM1为0，PWM2为speed
            ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT1, 0);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set left motor PWM1 duty: %s", esp_err_to_name(ret));
                return ret;
            }
            ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LEFT1);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to update left motor PWM1 duty: %s", esp_err_to_name(ret));
                return ret;
            }
            
            ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT2, duty);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set left motor PWM2 duty: %s", esp_err_to_name(ret));
                return ret;
            }
            ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LEFT2);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to update left motor PWM2 duty: %s", esp_err_to_name(ret));
                return ret;
            }
        }
    } else if (motor_id == MOTOR_RIGHT) {
        if (direction == MOTOR_FORWARD) {
            // 前进方向：PWM1为speed，PWM2为0
            ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT1, duty);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set right motor PWM1 duty: %s", esp_err_to_name(ret));
                return ret;
            }
            ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT1);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to update right motor PWM1 duty: %s", esp_err_to_name(ret));
                return ret;
            }
            
            ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT2, 0);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set right motor PWM2 duty: %s", esp_err_to_name(ret));
                return ret;
            }
            ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT2);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to update right motor PWM2 duty: %s", esp_err_to_name(ret));
                return ret;
            }
        } else {
            // 后退方向：PWM1为0，PWM2为speed
            ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT1, 0);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set right motor PWM1 duty: %s", esp_err_to_name(ret));
                return ret;
            }
            ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT1);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to update right motor PWM1 duty: %s", esp_err_to_name(ret));
                return ret;
            }
            
            ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT2, duty);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set right motor PWM2 duty: %s", esp_err_to_name(ret));
                return ret;
            }
            ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT2);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to update right motor PWM2 duty: %s", esp_err_to_name(ret));
                return ret;
            }
        }
    } else {
        ESP_LOGE(TAG, "Invalid motor ID: %d", motor_id);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "Motor %d set to %s at speed %d%%", 
             motor_id, direction == MOTOR_FORWARD ? "forward" : "backward", speed);
    
    return ESP_OK;
}

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
                           motor_direction_t right_dir, uint8_t right_speed)
{
    esp_err_t ret = ESP_OK;
    
    // 先控制左电机
    ret = motor_control(MOTOR_LEFT, left_dir, left_speed);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 再控制右电机
    ret = motor_control(MOTOR_RIGHT, right_dir, right_speed);
    
    return ret;
}

/**
 * @brief 停止单个电机
 * 
 * @param motor_id 电机ID
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t motor_stop(motor_id_t motor_id)
{
    esp_err_t ret = ESP_OK;
    
    if (motor_id == MOTOR_LEFT) {
        // 设置左电机两个PWM通道占空比为0
        ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT1, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set left motor PWM1 duty to 0: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LEFT1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to update left motor PWM1 duty: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT2, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set left motor PWM2 duty to 0: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LEFT2);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to update left motor PWM2 duty: %s", esp_err_to_name(ret));
            return ret;
        }
    } else if (motor_id == MOTOR_RIGHT) {
        // 设置右电机两个PWM通道占空比为0
        ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT1, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set right motor PWM1 duty to 0: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to update right motor PWM1 duty: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT2, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set right motor PWM2 duty to 0: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT2);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to update right motor PWM2 duty: %s", esp_err_to_name(ret));
            return ret;
        }
    } else {
        ESP_LOGE(TAG, "Invalid motor ID: %d", motor_id);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "Motor %d stopped", motor_id);
    
    return ESP_OK;
}

/**
 * @brief 停止所有电机
 * 
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t motor_stop_all(void)
{
    esp_err_t ret = ESP_OK;
    
    // 停止左电机
    ret = motor_stop(MOTOR_LEFT);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 停止右电机
    ret = motor_stop(MOTOR_RIGHT);
    
    return ret;
}