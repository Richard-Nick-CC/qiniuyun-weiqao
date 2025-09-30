#include <stdio.h>
#include "esp32_s3_szp.h"
#include "logo_en_240x240_lcd.h"
#include "yingwu.h"
#include <esp_system.h>
#include <nvs_flash.h>
#include <string.h>
#include <esp_psram.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "img_converters.h"
#include "motor_driver.h"
#include "ultrasonic_sensor.h"
#include "line_follower.h"

static const char *TAG = "main";

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ------------------ MJPE ------------------
static esp_err_t mjpeg_stream_handler(httpd_req_t *req)
{
    esp_err_t res = ESP_OK;
    char part_buf[64];

    httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        uint8_t *jpeg_buf = NULL;
        size_t jpeg_len = 0;

        if (!fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, 80, &jpeg_buf, &jpeg_len)) {
            ESP_LOGE(TAG, "JPEG compression failed");
            esp_camera_fb_return(fb);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        esp_camera_fb_return(fb);

        // 发送分段边界和头
        snprintf(part_buf, sizeof(part_buf), _STREAM_PART, jpeg_len);
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        if (res != ESP_OK) break;
        res = httpd_resp_send_chunk(req, part_buf, strlen(part_buf));
        if (res != ESP_OK) break;

        // 发送 JPEG 数据
        res = httpd_resp_send_chunk(req, (const char*)jpeg_buf, jpeg_len);
        free(jpeg_buf);
        if (res != ESP_OK) break;

        vTaskDelay(50 / portTICK_PERIOD_MS); // 每 50ms 发送一帧 (~20fps)
    }

    return res;
}

// ------------------ 网页 HTML ------------------
static const char *html_content =
"<!DOCTYPE html>"
"<html>"
"<head><title>ESP32 Camera Stream</title></head>"
"<body>"
"<h1>ESP32 Camera Stream</h1>"
"<img src=\"/stream\" style=\"max-width:100%;height:auto;\" />"
"</body>"
"</html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html_content, strlen(html_content));
}

// ------------------ 启动 HTTP Server ------------------
void start_camera_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 5120;
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = index_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &index_uri);

        httpd_uri_t stream_uri = {
            .uri = "/stream",
            .method = HTTP_GET,
            .handler = mjpeg_stream_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &stream_uri);

        ESP_LOGI(TAG, "Camera stream server started at http://<ESP32_IP>/");
    }
}

// ------------------ WiFi STA 配置 ------------------
#define WIFI_SSID "DESKTOP-8K91PH2 4377"
#define WIFI_PASS "12345678"
#define WIFI_MAXIMUM_RETRY 5
static int s_retry_num = 0;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            ESP_LOGI(TAG, "connect to the AP failed");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        start_camera_server(); // WiFi 获取 IP 后启动流
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished. Connecting to %s", WIFI_SSID);
}


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
    // LCD 初始化
    // pca9557_init();
    // bsp_lcd_init();
    // lcd_draw_pictrue(0, 0, 320, 240, gImage_yingwu);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // // 摄像头初始化
    // bsp_camera_init();
    // app_camera_lcd();

    size_t psram_size = esp_psram_get_size();
    printf("PSRAM size: %d bytes\n", psram_size);

    ESP_ERROR_CHECK(nvs_flash_init());

    // WiFi 初始化
    wifi_init_sta();
}