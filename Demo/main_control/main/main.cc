#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "car_control.h"
#include "track_sensor.h"
#include "ultrasonic.h"
#include "uart_comm.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "MAIN";

// 系统状态机
typedef enum {
    STATE_TRACKING,        // 循迹前进（初始状态）
    STATE_STOP_AT_GOAL,    // 到达抓取点，等待抓取完成
    STATE_ROTATE_AFTER_GRAB,  // 抓取完成后旋转180度
    STATE_GO_TO_RELEASE,   // 旋转完成，前往释放点
    STATE_STOP_AT_RELEASE, // 到达释放点，等待释放完成
    STATE_ROTATE_AFTER_RELEASE, // 释放完成后旋转180度
} State;

State current_state = STATE_TRACKING;
int cycle_count = 0;  // 记录循环次数

// 超声波传感器共享数据
volatile int g_ultrasonic_distance = -1;  // -1 表示未初始化

void car_task(void*);
void ultrasonic_task(void*);

extern "C" void app_main() {
    ESP_LOGI(TAG, "==== 智能仓储搬运车 - 下层底盘启动 ====");
    CarControl::getInstance().initAll();
    TrackSensor::getInstance().init();
    Ultrasonic::getInstance().init();
    UartComm::getInstance().init();
    xTaskCreate(car_task, "car_task", 8192, NULL, 5, NULL);
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, 4, NULL);
}

// 原地旋转180度（中间循迹模块对准黑线）
void turn_180() {
    auto& car = CarControl::getInstance();
    auto& track = TrackSensor::getInstance();
    ESP_LOGI(TAG, "==== 开始旋转 180 度 ====");

    // 第一步：快速旋转约180度
    car.spinLeft(100);
    vTaskDelay(pdMS_TO_TICKS(100));
    car.stop();
    vTaskDelay(pdMS_TO_TICKS(100));

    // 第二步：精确修正到黑线中心
    ESP_LOGI(TAG, "正在对齐中线...");
    while (1) {
        int s1,s2,s3,s4,s5;
        track.read(s1,s2,s3,s4,s5);
        
        // 检测中间传感器是否检测到黑线
        if (s3 == 0) {
            car.stop();
            ESP_LOGI(TAG, "==== 旋转完成，已对齐中线 ====");
            break;
        }
        
        // 缓慢旋转寻找黑线（使用更大的速度以确保能转动）
        car.spinLeft(200);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// 获取状态名称字符串
const char* get_state_name(State state) {
    switch(state) {
        case STATE_TRACKING: return "STATE_TRACKING";
        case STATE_STOP_AT_GOAL: return "STATE_STOP_AT_GOAL";
        case STATE_GO_TO_RELEASE: return "STATE_GO_TO_RELEASE";
        case STATE_STOP_AT_RELEASE: return "STATE_STOP_AT_RELEASE";
        default: return "UNKNOWN";
    }
}

// 超声波传感器任务 - 独立运行，检测距离并输出日志
void ultrasonic_task(void*) {
    auto& ultra = Ultrasonic::getInstance();
    static const char* TAG_ULTRA = "ULTRASONIC_TASK";
    
    ESP_LOGI(TAG_ULTRA, "超声波传感器任务启动");
    
    while (1) {
        int distance = ultra.getDistance();
        g_ultrasonic_distance = distance;
        
        if (distance > 0) {
            ESP_LOGI(TAG_ULTRA, "距离检测: %d cm", distance);
            
            if (distance <= 10) {
                ESP_LOGW(TAG_ULTRA, "警告：检测到近距离障碍物！距离: %d cm", distance);
            }
        } else {
            ESP_LOGW(TAG_ULTRA, "距离检测失败");
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms 检测一次
    }
}

// 主任务
void car_task(void*) {
    auto& car   = CarControl::getInstance();
    auto& track = TrackSensor::getInstance();
    auto& uart  = UartComm::getInstance();

    ESP_LOGI(TAG, "==== 进入主循环，开始循迹 ====");
    ESP_LOGI(TAG, "初始状态: %s", get_state_name(current_state));
    
    while (1) {
        int s1,s2,s3,s4,s5;
        track.read(s1,s2,s3,s4,s5);

        // 从超声波任务获取距离数据
        int distance = g_ultrasonic_distance;
        bool obstacle_near = (distance > 0 && distance <= 10);
        
        // 状态1：循迹前进
        if (current_state == STATE_TRACKING) {
            ESP_LOGD(TAG, "当前状态: STATE_TRACKING - 循迹中...");
            
            // 检测障碍物，10厘米以内自动停车
            if (obstacle_near) {
                ESP_LOGW(TAG, "==== 检测到近距离障碍物！距离: %d cm，立即停车！ ====", distance);
                car.setSpeed(0, 0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(30));
                continue;
            }
            
            if (track.isTarget()) {
                ESP_LOGI(TAG, "==== 检测到目标点，立即刹车！ ====");
                // car.brake();  // 使用刹车实现立即停止
                ESP_LOGI(TAG, "==== 到达抓取目标点！ ====");
                uart.send("START_GRAB\r\n");
                ESP_LOGI(TAG, "已发送：START_GRAB");
                ESP_LOGI(TAG, "==== 状态转换: STATE_TRACKING -> STATE_STOP_AT_GOAL ====");
                current_state = STATE_STOP_AT_GOAL;
                continue;
            }

            // 循迹PID
            int weights[] = {-2,-1,0,1,2};
            int sum_w=0, cnt=0;
            int sen[]={s1,s2,s3,s4,s5};
            for(int i=0;i<5;i++) if(sen[i]==0) { sum_w+=weights[i]; cnt++; }
            int dev = cnt ? sum_w/cnt : 0;

            int m1 = BASE_SPEED - dev*TUNE_SPEED;
            int m2 = BASE_SPEED - dev*TUNE_SPEED;
            int m3 = BASE_SPEED + dev*TUNE_SPEED;
            int m4 = BASE_SPEED + dev*TUNE_SPEED;

            auto clamp = [](int v){ return v<0?0:(v>MAX_SPEED?MAX_SPEED:v); };
            car.setSpeed(clamp(m1),clamp(m2),clamp(m3),clamp(m4));
        }

        // 状态2：到达抓取点
        else if (current_state == STATE_STOP_AT_GOAL) {
            ESP_LOGD(TAG, "当前状态: STATE_STOP_AT_GOAL - 等待抓取完成...");
            car.setSpeed(0,0,0,0);

            char buf[32];
            int len = uart.read(buf, sizeof(buf)-1);
            if (len > 0) {
                buf[len] = '\0';
                ESP_LOGI(TAG, "接收到串口数据: %s", buf);
                if (strstr(buf, "GRAB_OK")) {
                    ESP_LOGI(TAG, "==== 抓取完成！准备旋转180度 ====");
                    turn_180();
                    ESP_LOGI(TAG, "==== 状态转换: STATE_STOP_AT_GOAL -> STATE_GO_TO_RELEASE ====");
                    current_state = STATE_GO_TO_RELEASE;
                    ESP_LOGI(TAG, "==== 开始前往释放点 ====");
                }
            }
        }

        // 状态3：前往释放点
        else if (current_state == STATE_GO_TO_RELEASE) {
            ESP_LOGD(TAG, "当前状态: STATE_GO_TO_RELEASE - 前往释放点中...");
            
            // 检测障碍物，10厘米以内自动停车
            if (obstacle_near) {
                ESP_LOGW(TAG, "==== 检测到近距离障碍物！距离: %d cm，立即停车！ ====", distance);
                car.setSpeed(0, 0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(30));
                continue;
            }
            
            if (track.isTarget()) {
                ESP_LOGI(TAG, "==== 检测到释放点，立即刹车！ ====");
                // car.brake();  // 使用刹车实现立即停止
                ESP_LOGI(TAG, "==== 到达释放目标点！ ====");
                uart.send("START_RELEASE\r\n");
                ESP_LOGI(TAG, "已发送：START_RELEASE");
                ESP_LOGI(TAG, "==== 状态转换: STATE_GO_TO_RELEASE -> STATE_STOP_AT_RELEASE ====");
                current_state = STATE_STOP_AT_RELEASE;
                continue;
            }

            // 循迹
            int weights[] = {-2,-1,0,1,2};
            int sum_w=0, cnt=0;
            int sen[]={s1,s2,s3,s4,s5};
            for(int i=0;i<5;i++) if(sen[i]==0) { sum_w+=weights[i]; cnt++; }
            int dev = cnt ? sum_w/cnt : 0;

            int m1 = BASE_SPEED - dev*TUNE_SPEED;
            int m2 = BASE_SPEED - dev*TUNE_SPEED;
            int m3 = BASE_SPEED + dev*TUNE_SPEED;
            int m4 = BASE_SPEED + dev*TUNE_SPEED;

            auto clamp = [](int v){ return v<0?0:(v>MAX_SPEED?MAX_SPEED:v); };
            car.setSpeed(clamp(m1),clamp(m2),clamp(m3),clamp(m4));
        }

        // 状态4：到达释放点
        else if (current_state == STATE_STOP_AT_RELEASE) {
            ESP_LOGD(TAG, "当前状态: STATE_STOP_AT_RELEASE - 等待释放完成...");
            car.setSpeed(0,0,0,0);

            char buf[32];
            int len = uart.read(buf, sizeof(buf)-1);
            if (len > 0) {
                buf[len] = '\0';
                ESP_LOGI(TAG, "接收到串口数据: %s", buf);
                if (strstr(buf, "RELEASE_OK")) {
                    ESP_LOGI(TAG, "==== 释放完成！准备旋转180度 ====");
                    turn_180();
                    
                    cycle_count++;
                    ESP_LOGI(TAG, "==== 状态转换: STATE_STOP_AT_RELEASE -> STATE_TRACKING ====");
                    ESP_LOGI(TAG, "==== 第 %d 次循环完成，开始新的循迹操作 ====", cycle_count);
                    current_state = STATE_TRACKING;
                    
                    // 清空串口缓冲区，避免残留数据影响下一次循环
                    char dummy[64];
                    while (uart.read(dummy, sizeof(dummy)-1) > 0);
                    ESP_LOGI(TAG, "串口缓冲区已清空");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}