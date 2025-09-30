#include "camera_wifi.h"
#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_log.h"

static const char *TAG = "camera_wifi";

// MJPEG流相关常量定义
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// WiFi配置
#define WIFI_SSID "DESKTOP-8K91PH2 4377"
#define WIFI_PASS "12345678"
#define WIFI_MAXIMUM_RETRY 5
static int s_retry_num = 0;

// MJPEG流处理函数
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

// 网页HTML内容
static const char *html_content =
"<!DOCTYPE html>"
"<html>"
"<head><title>ESP32 Camera Stream</title></head>"
"<body>"
"<h1>ESP32 Camera Stream</h1>"
"<img src=\"/stream\" style=\"max-width:100%;height:auto;\" />"
"</body>"
"</html>";

// 网页首页处理函数
static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html_content, strlen(html_content));
}

// 启动HTTP服务器
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

// WiFi事件处理函数
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

// WiFi STA初始化
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

// 初始化WiFi并启动摄像头服务器
void camera_wifi_init(void)
{
    wifi_init_sta();
}