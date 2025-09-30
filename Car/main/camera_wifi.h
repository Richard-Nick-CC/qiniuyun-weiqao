#ifndef CAMERA_WIFI_H
#define CAMERA_WIFI_H

#include "esp_http_server.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 WiFi 并在获取 IP 后启动摄像头 HTTP 服务器
 */
void camera_wifi_init(void);

/**
 * @brief 启动摄像头 HTTP 服务器
 */
void start_camera_server(void);

#ifdef __cplusplus
}
#endif

#endif // CAMERA_WIFI_H