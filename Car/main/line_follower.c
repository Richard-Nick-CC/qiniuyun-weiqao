#include "line_follower.h"
#include "esp32_s3_szp.h"
#include "motor_driver.h"
#include "ultrasonic_sensor.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

static const char *TAG = "line_follower";

// ===================== 参数 =====================
#define BASE_SPEED 55
#define MIN_SPEED 45
#define MAX_SPEED 65
#define MAX_TURN_SPEED 45
#define DEAD_ZONE 0.02
#define LOST_LINE_THRESHOLD 3
#define CORNER_ERROR_THRESHOLD 0.3
#define CORNER_SPEED_REDUCTION 10
#define CURVE_SPEED_FACTOR 10  // 弯道速度降低系数

static line_follower_t g_line_follower = {
    .sensor_value = 0,
    .sensor_state = {0,0,0,0},
    .mode = LINE_FOLLOWER_MODE_READ,
    .speed = BASE_SPEED,
    .turn_speed_diff = MAX_TURN_SPEED,
    .obstacle_state = OBSTACLE_STATE_NONE,
    .obstacle_action = OBSTACLE_ACTION_CONTINUE,
    .obstacle_distance = 0.0f,
    .obstacle_start_time = 0,
    .obstacle_enabled = 1  // 默认启用避障功能
};

static TaskHandle_t g_line_follower_task_handle = NULL;

// ===================== I2C =====================
void writeByte(unsigned char reg_address, unsigned char val) {
    uint8_t write_buf[2] = {reg_address, val};
    i2c_master_write_to_device(BSP_I2C_NUM, LineFollower4ch_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
}

unsigned char readByte(unsigned char reg_address) {
    unsigned char val;
    i2c_master_write_read_device(BSP_I2C_NUM, LineFollower4ch_ADDR, &reg_address, 1, &val, 1, 1000 / portTICK_PERIOD_MS);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    return val;
}

uint8_t read_line_follow(void) {
    return readByte(LineFollower4ch_data_reg_ADDR);
}

// ===================== 初始化 =====================
esp_err_t line_follower_init(void) {
    writeByte(LineFollower4ch_config_reg_ADDR, 0x00);
    uint8_t test_values[5];
    int valid_reads = 0;
    for(int i=0;i<5;i++){
        test_values[i]=read_line_follow();
        if(test_values[i]!=0x00 && test_values[i]!=0xFF) valid_reads++;
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    if(valid_reads==0){
        ESP_LOGW(TAG,"All sensor reads extreme values, check hardware.");
    }
    ESP_LOGI(TAG,"Line follower initialized");
    return ESP_OK;
}

// ===================== 控制变量 =====================
static int last_direction = 0;
static float last_error = 0.0;
static int lost_line_counter = 0;

// ===================== PD误差计算 =====================
float calc_error_pd(uint8_t sensor_value){
    float weights[4]={-0.8,-1.0,1.0,0.8};
    bool line_detected=false;
    float sum=0.0,count=0.0;

    for(int i=0;i<4;i++){
        g_line_follower.sensor_state[i] = ((sensor_value>>i)&0x01)?1:0;
        if(g_line_follower.sensor_state[i]==0){
            sum+=weights[i];
            count++;
            line_detected=true;
        }
    }

    float current_error=0.0;
    if(line_detected){
        current_error=sum/count;
        lost_line_counter=0;
    }else{
        lost_line_counter++;
        current_error = last_direction * 0.6;
    }

    if(fabs(current_error)<DEAD_ZONE) current_error=0.0;
    return current_error;
}

// ===================== 避障辅助函数 =====================
// 获取动态避障距离（根据当前速度调整）
static float get_dynamic_obstacle_distance(void) {
    float base_distance = OBSTACLE_DETECTION_DISTANCE;
    
    // 根据当前速度动态调整检测距离
    if (g_line_follower.speed > BASE_SPEED * 0.8) {
        base_distance *= 1.3; // 高速时增加检测距离
    } else if (g_line_follower.speed < BASE_SPEED * 0.5) {
        base_distance *= 0.8; // 低速时减少检测距离
    }
    
    return base_distance;
}

// 多次测量取平均值，提高检测准确性
static float get_average_distance(int samples) {
    float total = 0;
    int valid_samples = 0;
    
    for (int i = 0; i < samples; i++) {
        float distance = 0;
        esp_err_t ret = ultrasonic_sensor_measure(&distance);
        if (ret == ESP_OK && distance > 0 && distance < 400) { // 有效测量范围
            total += distance;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 短暂延时
    }
    
    return (valid_samples > 0) ? (total / valid_samples) : -1;
}

// ===================== 避障检测 =====================
static bool check_obstacle(void) {
    if (!g_line_follower.obstacle_enabled) {
        return false;
    }
    
    // 使用多次测量的平均值
    float distance = get_average_distance(3);
    g_line_follower.obstacle_distance = distance;
    
    if (distance > 0 && distance < get_dynamic_obstacle_distance()) {
        ESP_LOGD(TAG, "Obstacle detected at %.1f cm (threshold: %.1f cm)", 
                distance, get_dynamic_obstacle_distance());
        return true;
    }
    
    return false;
}

// ===================== 避障行为处理 =====================
static void handle_obstacle_avoidance(void) {
    uint32_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒
    uint32_t elapsed_time = current_time - g_line_follower.obstacle_start_time;
    
    switch (g_line_follower.obstacle_state) {
        case OBSTACLE_STATE_NONE:
            if (check_obstacle()) {
                ESP_LOGI(TAG, "Obstacle detected at %.1f cm", g_line_follower.obstacle_distance);
                g_line_follower.obstacle_state = OBSTACLE_STATE_DETECTED;
                g_line_follower.obstacle_action = OBSTACLE_ACTION_STOP;
                g_line_follower.obstacle_start_time = current_time;
            }
            break;
            
        case OBSTACLE_STATE_DETECTED:
            if (elapsed_time > 200) { // 停止200ms后立即开始避障
                g_line_follower.obstacle_state = OBSTACLE_STATE_AVOIDING;
                g_line_follower.obstacle_action = OBSTACLE_ACTION_BACKUP;
                g_line_follower.obstacle_start_time = current_time;
                ESP_LOGI(TAG, "Starting obstacle avoidance - backing up");
            }
            break;
            
        case OBSTACLE_STATE_AVOIDING:
            if (g_line_follower.obstacle_action == OBSTACLE_ACTION_BACKUP) {
                if (elapsed_time > OBSTACLE_BACKUP_TIME) {
                    // 后退完成，一律右转
                    g_line_follower.obstacle_action = OBSTACLE_ACTION_TURN_RIGHT;
                    g_line_follower.obstacle_start_time = current_time;
                    ESP_LOGI(TAG, "Backing up complete - turning right");
                }
            } else if (g_line_follower.obstacle_action == OBSTACLE_ACTION_TURN_RIGHT) {
                if (elapsed_time > OBSTACLE_TURN_TIME) {
                    // 右转完成，进入返回状态
                    g_line_follower.obstacle_state = OBSTACLE_STATE_RETURNING;
                    g_line_follower.obstacle_action = OBSTACLE_ACTION_CONTINUE;
                    g_line_follower.obstacle_start_time = current_time;
                    ESP_LOGI(TAG, "Right turn complete - returning to line following");
                }
            }
            break;
            
        case OBSTACLE_STATE_RETURNING:
            // 在返回阶段，检查是否找到了线
            if (elapsed_time > 500) { // 给一些时间让传感器稳定
                uint16_t line_data = read_line_follow();
                bool line_found = false;
                
                // 检查是否有传感器检测到线
                for (int i = 0; i < 5; i++) {
                    if ((line_data >> i) & 0x01) {
                        line_found = true;
                        break;
                    }
                }
                
                if (line_found) {
                    // 找到线了，立即恢复循迹
                    g_line_follower.obstacle_state = OBSTACLE_STATE_NONE;
                    g_line_follower.obstacle_action = OBSTACLE_ACTION_CONTINUE;
                    ESP_LOGI(TAG, "Line detected - obstacle avoidance complete");
                } else if (elapsed_time > OBSTACLE_RETURN_TIME) {
                    // 超时仍未找到线，检查障碍物情况
                    if (!check_obstacle()) {
                        g_line_follower.obstacle_state = OBSTACLE_STATE_NONE;
                        g_line_follower.obstacle_action = OBSTACLE_ACTION_CONTINUE;
                        ESP_LOGI(TAG, "Timeout - resuming normal operation");
                    } else {
                        // 如果还有障碍物，重新开始避障
                        g_line_follower.obstacle_state = OBSTACLE_STATE_DETECTED;
                        g_line_follower.obstacle_start_time = current_time;
                        ESP_LOGI(TAG, "Obstacle still present - restarting avoidance");
                    }
                }
            }
            break;
    }
}

// ===================== 执行避障动作 =====================
static void execute_obstacle_action(void) {
    switch (g_line_follower.obstacle_action) {
        case OBSTACLE_ACTION_STOP:
            motor_stop_all();
            ESP_LOGD(TAG, "Obstacle action: STOP");
            break;
            
        case OBSTACLE_ACTION_BACKUP:
            // 后退速度适中，确保稳定
            motor_control_both(MOTOR_BACKWARD, BASE_SPEED * 0.6, MOTOR_BACKWARD, BASE_SPEED * 0.6);
            ESP_LOGD(TAG, "Obstacle action: BACKUP (speed: %d)", (int)(BASE_SPEED * 0.6));
            break;
            
        case OBSTACLE_ACTION_TURN_LEFT:
            // 原地左转，左轮后退，右轮前进
            motor_control_both(MOTOR_BACKWARD, BASE_SPEED * 0.5, MOTOR_FORWARD, BASE_SPEED * 0.5);
            ESP_LOGD(TAG, "Obstacle action: TURN_LEFT");
            break;
            
        case OBSTACLE_ACTION_TURN_RIGHT:
            // 原地右转，左轮前进，右轮后退
            motor_control_both(MOTOR_FORWARD, BASE_SPEED * 0.5, MOTOR_BACKWARD, BASE_SPEED * 0.5);
            ESP_LOGD(TAG, "Obstacle action: TURN_RIGHT");
            break;
            
        case OBSTACLE_ACTION_CONTINUE:
            // 不执行任何动作，让循迹算法接管
            ESP_LOGD(TAG, "Obstacle action: CONTINUE - handing over to line following");
            return; // 直接返回，不设置电机速度
    }
}

// ===================== 循迹处理 =====================
static void line_follower_process(uint8_t sensor_value){
    // 首先处理避障逻辑
    handle_obstacle_avoidance();
    
    // 如果正在避障，执行避障动作而不是循迹
    if (g_line_follower.obstacle_state != OBSTACLE_STATE_NONE && 
        g_line_follower.obstacle_action != OBSTACLE_ACTION_CONTINUE) {
        execute_obstacle_action();
        return; // 避障期间不执行循迹逻辑
    }
    
    // 失线检测
    bool line_detected=false;
    for(int i=0;i<4;i++){
        if(((sensor_value>>i)&0x01)==0){
            line_detected=true;
            break;
        }
    }

    if(!line_detected){
        motor_stop_all(); // 停止
        int scan_dir = (last_direction>=0)?1:-1;
        while(1){
            // 缓慢原地旋转扫描
            if(scan_dir>0)
                motor_control_both(MOTOR_FORWARD, BASE_SPEED/2, MOTOR_BACKWARD, BASE_SPEED/2);
            else
                motor_control_both(MOTOR_BACKWARD, BASE_SPEED/2, MOTOR_FORWARD, BASE_SPEED/2);

            uint8_t new_val = read_line_follow();
            g_line_follower.sensor_state[0] = ((new_val>>0)&0x01)?1:0;
            g_line_follower.sensor_state[1] = ((new_val>>1)&0x01)?1:0;
            g_line_follower.sensor_state[2] = ((new_val>>2)&0x01)?1:0;
            g_line_follower.sensor_state[3] = ((new_val>>3)&0x01)?1:0;

            // 中间传感器检测到线
            if(g_line_follower.sensor_state[1]==0 || g_line_follower.sensor_state[2]==0){
                motor_stop_all();
                break;
            }

            vTaskDelay(5/portTICK_PERIOD_MS);
        }
    }

    // 正常 PD 控制
    float current_error = calc_error_pd(sensor_value);

    // PD参数
    float Kp = 16.0;
    float Kd = 2.5;
    float derivative = current_error - last_error;
    float pid_output = Kp*current_error + Kd*derivative;

    // 曲线增强非线性
    float abs_err = fabs(current_error);
    if(abs_err > CORNER_ERROR_THRESHOLD)
        pid_output = copysign(pow(fabs(pid_output),1.5), pid_output);

    if(pid_output>1.0) pid_output=1.0;
    if(pid_output<-1.0) pid_output=-1.0;

    // 动态减速，曲线越大速度越慢
    int dynamic_base_speed = BASE_SPEED - (int)(abs_err*CURVE_SPEED_FACTOR);
    if(dynamic_base_speed<MIN_SPEED) dynamic_base_speed=MIN_SPEED;

    int left_speed = dynamic_base_speed;
    int right_speed = dynamic_base_speed;

    if(abs_err > CORNER_ERROR_THRESHOLD){
        left_speed -= CORNER_SPEED_REDUCTION;
        right_speed -= CORNER_SPEED_REDUCTION;
    }

    if(lost_line_counter>LOST_LINE_THRESHOLD){
        int turn = last_direction * MAX_TURN_SPEED;
        left_speed = dynamic_base_speed - turn/2;
        right_speed = dynamic_base_speed + turn/2;
        if(left_speed<MIN_SPEED) left_speed=MIN_SPEED;
        if(right_speed<MIN_SPEED) right_speed=MIN_SPEED;
    }else{
        int turn = (int)(pid_output * MAX_TURN_SPEED);
        if(turn>MAX_TURN_SPEED) turn=MAX_TURN_SPEED;
        if(turn<-MAX_TURN_SPEED) turn=-MAX_TURN_SPEED;
        left_speed -= turn;
        right_speed += turn;
    }

    if(left_speed<MIN_SPEED) left_speed=MIN_SPEED;
    if(left_speed>MAX_SPEED) left_speed=MAX_SPEED;
    if(right_speed<MIN_SPEED) right_speed=MIN_SPEED;
    if(right_speed>MAX_SPEED) right_speed=MAX_SPEED;

    motor_control_both(MOTOR_FORWARD,(uint8_t)left_speed,MOTOR_FORWARD,(uint8_t)right_speed);

    if(current_error<-0.15) last_direction=-1;
    else if(current_error>0.15) last_direction=1;
    else last_direction=0;

    last_error=current_error;
}

// ===================== 循迹任务 =====================
static void line_follower_task(void *pvParameters){
    ESP_LOGI(TAG,"Line follower task started");
    if(motor_driver_init()!=ESP_OK){
        ESP_LOGE(TAG,"Motor driver init failed");
        vTaskDelete(NULL);
        return;
    }
    
    // 初始化超声波传感器（用于避障）
    if(ultrasonic_sensor_init()!=ESP_OK){
        ESP_LOGE(TAG,"Ultrasonic sensor init failed");
        g_line_follower.obstacle_enabled = 0; // 禁用避障功能
        ESP_LOGW(TAG,"Obstacle avoidance disabled due to sensor init failure");
    } else {
        ESP_LOGI(TAG,"Ultrasonic sensor initialized for obstacle avoidance");
    }

    while(1){
        g_line_follower.sensor_value = read_line_follow();
        line_follower_process(g_line_follower.sensor_value);
        vTaskDelay(5/portTICK_PERIOD_MS);
    }
}

// ===================== 任务启动/停止 =====================
esp_err_t line_follower_start_task(uint16_t stack_size,UBaseType_t priority,
                                   uint8_t speed,uint8_t turn_speed_diff){
    if(g_line_follower_task_handle!=NULL) return ESP_OK;

    if(line_follower_init()!=ESP_OK) return ESP_FAIL;

    g_line_follower.speed = BASE_SPEED;
    g_line_follower.turn_speed_diff = MAX_TURN_SPEED;

    BaseType_t ret=xTaskCreate(line_follower_task,"line_follower_task",
                               stack_size,NULL,priority,&g_line_follower_task_handle);
    if(ret!=pdPASS){
        g_line_follower_task_handle=NULL;
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t line_follower_stop_task(void){
    if(g_line_follower_task_handle==NULL) return ESP_OK;
    motor_stop_all();
    vTaskDelete(g_line_follower_task_handle);
    g_line_follower_task_handle=NULL;
    return ESP_OK;
}

esp_err_t line_follower_set_speed(uint8_t speed){
    if(speed>100) return ESP_ERR_INVALID_ARG;
    g_line_follower.speed = speed;
    return ESP_OK;
}

esp_err_t line_follower_get_state(uint8_t state[4]){
    if(state==NULL) return ESP_ERR_INVALID_ARG;
    memcpy(state,g_line_follower.sensor_state,4);
    return ESP_OK;
}

esp_err_t line_follower_set_obstacle_avoidance(uint8_t enabled) {
    g_line_follower.obstacle_enabled = enabled ? 1 : 0;
    if (!enabled) {
        // 禁用避障时，重置避障状态
        g_line_follower.obstacle_state = OBSTACLE_STATE_NONE;
        g_line_follower.obstacle_action = OBSTACLE_ACTION_CONTINUE;
        ESP_LOGI(TAG, "Obstacle avoidance %s", enabled ? "enabled" : "disabled");
    }
    return ESP_OK;
}

esp_err_t line_follower_get_obstacle_state(obstacle_state_t *state, float *distance) {
    if (state == NULL || distance == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *state = g_line_follower.obstacle_state;
    *distance = g_line_follower.obstacle_distance;
    return ESP_OK;
}
