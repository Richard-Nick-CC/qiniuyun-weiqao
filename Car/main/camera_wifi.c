#include "camera_wifi.h"

// WiFi事件组位定义
#define WIFI_FAIL_BIT BIT1
#define WIFI_CONNECTED_BIT BIT0

// 从main.c导入WiFi事件组访问函数
EventGroupHandle_t get_wifi_event_group(void);


static const char *TAG = "wifi_camera"; // static 更好，避免外部引用冲突


#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ------------------ MJPEG 流处理函数 ------------------
esp_err_t mjpeg_stream_handler(httpd_req_t *req)
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
"<!DOCTYPE html>\n"
"<html>\n"
"<head>\n"
"<title>ESP32 Camera Stream</title>\n"
"<style>\n"
"body { font-family: Arial, sans-serif; margin: 20px; }\n"
".status { background-color: #f0f0f0; padding: 10px; margin-bottom: 20px; border-radius: 5px; }\n"
".connected { color: green; font-weight: bold; }\n"
".stream-container { margin-top: 20px; }\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<h1>ESP32 Camera Web Server</h1>\n"
"<div class='status'>\n"
"<p>Device: ESP32S3 Camera Module</p>\n"
"<p>Status: <span class='connected'>Connected to WiFi</span></p>\n"
"<p>IP Address: 192.168.137.229</p>\n"
"</div>\n"
"<div class='stream-container'>\n"
"<h2>Live Camera Stream</h2>\n"
"<img src=\"/stream\" style=\"max-width:100%;height:auto; border: 1px solid #ccc;\" />\n"
"</div>\n"
"<div style='margin-top: 20px;'>\n"
"<p>To access this page from another device, open a browser and enter: <strong>http://192.168.137.229</strong></p>\n"
"</div>\n"
"<script>\n"
"// 定期检查连接状态\n"
"setInterval(function() {\n"
"  fetch('/')\n"
"    .then(response => {\n"
"      if (!response.ok) throw new Error('Connection error');\n"
"      return response.text();\n"
"    })\n"
"    .catch(error => {\n"
"      console.error('Connection check failed:', error);\n"
"    });\n"
"}, 5000);\n"
"</script>\n"
"</body>"
"</html>";

esp_err_t index_handler(httpd_req_t *req)
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
int s_retry_num = 0;

void wifi_event_handler(void* arg, esp_event_base_t event_base,
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
            // 通知主任务连接失败
            EventGroupHandle_t wifi_event_group = get_wifi_event_group();
            if (wifi_event_group != NULL) {
                xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        
        // 通知主任务连接成功
        EventGroupHandle_t wifi_event_group = get_wifi_event_group();
        if (wifi_event_group != NULL) {
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        }
        
        start_camera_server(); // WiFi 获取 IP 后启动流
    }
}

void wifi_init_sta(void)
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