#pragma once

#include <stdio.h>
#include "esp32_s3_szp.h"


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

esp_err_t index_handler(httpd_req_t *req);
void wifi_event_handler(void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data);
esp_err_t mjpeg_stream_handler(httpd_req_t *req);
void start_camera_server(void);
void wifi_init_sta(void);