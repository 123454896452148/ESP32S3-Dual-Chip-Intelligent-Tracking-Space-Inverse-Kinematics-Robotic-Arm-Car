#ifndef CONFIG_H
#define CONFIG_H
#include "driver/gpio.h"

// 电机1 左后
#define PWMA        GPIO_NUM_14
#define AIN1        GPIO_NUM_12
#define AIN2        GPIO_NUM_13
#define E1A         GPIO_NUM_11
#define E1B         GPIO_NUM_10

// 电机2 左前
#define PWB         GPIO_NUM_9
#define BIN1        GPIO_NUM_46
#define BIN2        GPIO_NUM_3
#define E2A         GPIO_NUM_8
#define E2B         GPIO_NUM_18

// 电机3 右后
#define PWC         GPIO_NUM_19
#define CIN1        GPIO_NUM_20
#define CIN2        GPIO_NUM_21
#define E3A         GPIO_NUM_47
#define E3B         GPIO_NUM_48

// 电机4 右前
#define PWD         GPIO_NUM_17
#define DIN1        GPIO_NUM_16
#define DIN2        GPIO_NUM_15
#define E4A         GPIO_NUM_7
#define E4B         GPIO_NUM_6

// 循迹传感器
#define S1          GPIO_NUM_42
#define S2          GPIO_NUM_41
#define S3          GPIO_NUM_40
#define S4          GPIO_NUM_39
#define S5          GPIO_NUM_38

// 超声波
#define TRIG_PIN    GPIO_NUM_1
#define ECHO_PIN    GPIO_NUM_2

// 串口
#define UART_PORT       UART_NUM_1
#define UART_TX_PIN     GPIO_NUM_4
#define UART_RX_PIN     GPIO_NUM_5

// 运行参数
#define LEDC_MODE       LEDC_LOW_SPEED_MODE       // 低速模式
#define BASE_SPEED      200                       // 基础速度 (提高到200)
#define TUNE_SPEED      50                        // 调整速度 (减小，避免速度波动过大)
#define MAX_SPEED       300                       // 最大速度
#define OBSTACLE_DIST   15                        // 障碍物距离阈值

#endif