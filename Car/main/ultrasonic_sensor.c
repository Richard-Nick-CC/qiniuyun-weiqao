#include "ultrasonic_sensor.h"

static const char *TAG = "ultrasonic_sensor";

esp_err_t ultrasonic_sensor_init(void)
{
    esp_err_t ret = ESP_OK;
    
    // 配置TRIG引脚为输出模式
    gpio_config_t trig_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << ULTRASONIC_TRIG_PIN),
        .pull_down_en = false,
        .pull_up_en = false,
    };
    ret = gpio_config(&trig_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure TRIG pin: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 配置ECHO引脚为输入模式
    gpio_config_t echo_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ULTRASONIC_ECHO_PIN),
        .pull_down_en = false,
        .pull_up_en = true,  // 使用上拉电阻，防止悬空
    };
    ret = gpio_config(&echo_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ECHO pin: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 初始化TRIG引脚为低电平
    ret = gpio_set_level(ULTRASONIC_TRIG_PIN, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set TRIG pin level: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Ultrasonic sensor initialized successfully");
    return ESP_OK;
}

esp_err_t ultrasonic_sensor_trigger(void)
{
    esp_err_t ret = ESP_OK;
    
    // 发送10us的高电平脉冲
    ret = gpio_set_level(ULTRASONIC_TRIG_PIN, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set TRIG pin high: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 延时10us
    // 使用esp_rom提供的延时函数
    esp_rom_delay_us(10);
    
    // 发送结束，设置为低电平
    ret = gpio_set_level(ULTRASONIC_TRIG_PIN, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set TRIG pin low: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t ultrasonic_sensor_measure(float *distance)
{
    esp_err_t ret = ESP_OK;
    int64_t start_time = 0;
    int64_t end_time = 0;
    int64_t duration = 0;
    uint32_t timeout_count = 0;
    
    if (distance == NULL) {
        ESP_LOGE(TAG, "Distance pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 触发超声波传感器
    ret = ultrasonic_sensor_trigger();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger ultrasonic sensor: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 等待ECHO引脚变为高电平
    while (gpio_get_level(ULTRASONIC_ECHO_PIN) == 0) {
        if (esp_timer_get_time() - start_time > ULTRASONIC_TIMEOUT_MS * 1000) {  // 微秒级超时检测
            ESP_LOGE(TAG, "Timeout waiting for ECHO to go high");
            *distance = -1.0f;
            return ESP_FAIL;
        }
        // 增加延时时间，减少CPU占用
        esp_rom_delay_us(10);
    }
    
    // 记录ECHO引脚变为高电平的时间
    int64_t high_start_time = esp_timer_get_time();
    
    // 等待ECHO引脚变为低电平
    while (gpio_get_level(ULTRASONIC_ECHO_PIN) == 1) {
        if (esp_timer_get_time() - start_time > ULTRASONIC_TIMEOUT_MS * 1000) {  // 微秒级超时检测
            ESP_LOGE(TAG, "Timeout waiting for ECHO to go low");
            *distance = -1.0f;
            return ESP_FAIL;
        }
        // 增加延时时间，减少CPU占用
        esp_rom_delay_us(10);
    }
    
    // 记录ECHO引脚变为低电平的时间
    end_time = esp_timer_get_time();
    
    // 计算超声波往返时间(微秒)
    duration = end_time - high_start_time;
    
    // 计算距离(cm)：距离 = (时间 * 声速) / 2，声速343m/s = 0.0343cm/us
    *distance = (duration * 0.0343f) / 2.0f;
    
    // 检查距离是否在有效范围内
    if (*distance < 0 || *distance > ULTRASONIC_MAX_DISTANCE) {
        ESP_LOGE(TAG, "Measured distance out of range: %.2f cm", *distance);
        *distance = -1.0f;
        return ESP_FAIL;
    }
    
    ESP_LOGD(TAG, "Measured distance: %.2f cm", *distance);
    
    return ESP_OK;
}