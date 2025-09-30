#include "ultrasonic_sensor.h"

static const char *TAG = "ultrasonic_sensor";

// 全局变量，用于存储最新的距离测量结果
static float g_last_distance = 0.0f;
// 互斥锁，用于保护共享资源
static SemaphoreHandle_t g_distance_mutex = NULL;
// 默认的测量间隔(ms)
#define ULTRASONIC_DEFAULT_INTERVAL_MS (100)

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
    
    // 初始化开始时间用于超时检测
    start_time = esp_timer_get_time();
    
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

/**
 * @brief 获取最新的距离测量结果
 * 
 * @param distance 最新的距离测量结果(cm)
 * @return esp_err_t 成功返回ESP_OK，失败返回相应错误码
 */
esp_err_t ultrasonic_sensor_get_last_distance(float *distance)
{
    if (distance == NULL) {
        ESP_LOGE(TAG, "Distance pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_distance_mutex == NULL) {
        ESP_LOGE(TAG, "Mutex not initialized");
        return ESP_FAIL;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(g_distance_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_FAIL;
    }
    
    // 复制最新的距离测量结果
    *distance = g_last_distance;
    
    // 释放互斥锁
    xSemaphoreGive(g_distance_mutex);
    
    return ESP_OK;
}

/**
 * @brief 超声波测距任务函数
 * 
 * @param pvParameters 任务参数（未使用）
 */
static void ultrasonic_sensor_task(void *pvParameters)
{
    float distance = 0.0f;
    
    // 初始化超声波传感器
    if (ultrasonic_sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ultrasonic sensor");
        vTaskDelete(NULL);
        return;
    }
    
    while (1) {
        // 测量距离
        if (ultrasonic_sensor_measure(&distance) == ESP_OK) {
            // 获取互斥锁
            if (xSemaphoreTake(g_distance_mutex, portMAX_DELAY) == pdTRUE) {
                // 更新最新的距离测量结果
                g_last_distance = distance;
                
                // 释放互斥锁
                xSemaphoreGive(g_distance_mutex);
            }
        } else {
            ESP_LOGE(TAG, "Failed to measure distance");
        }
        
        // 延时一段时间后再次测量
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_DEFAULT_INTERVAL_MS));
    }
}

esp_err_t ultrasonic_sensor_create_task(uint8_t task_priority, uint32_t task_stack_size, TaskHandle_t *task_handle)
{
    esp_err_t ret = ESP_OK;
    
    // 初始化互斥锁
    if (g_distance_mutex == NULL) {
        g_distance_mutex = xSemaphoreCreateMutex();
        if (g_distance_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_FAIL;
        }
    }
    
    // 创建超声波测距任务
    if (xTaskCreate(ultrasonic_sensor_task, "ultrasonic_task", 
                   task_stack_size, NULL, task_priority, task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ultrasonic sensor task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Ultrasonic sensor task created successfully");
    
    return ESP_OK;
}