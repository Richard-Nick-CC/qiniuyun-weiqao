#include "line_follower.h"
#include "esp32_s3_szp.h"
#include "motor_driver.h"
#include "ultrasonic_sensor.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include <stdio.h>
#include <math.h>

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

// ===================== 循迹处理 =====================
static void line_follower_process(uint8_t sensor_value){
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
