#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "pid_controller.h"

class CarControl {
public:
    static CarControl& getInstance();
    void initAll();
    void setSpeed(int m1, int m2, int m3, int m4);
    void stop();
    void brake();
    void moveStraight(int speed);
    void moveBack(int speed);
    void turnLeft();
    void turnRight();
    void spinLeft(int speed);
    void spinRight(int speed);

private:
    CarControl();
    void initGPIO();
    void initPWM();
    void initAllEncoders();
    int getSpeed(pcnt_unit_handle_t unit);

    // 新增：声明setMotor
    void setMotor(gpio_num_t in1, gpio_num_t in2, ledc_channel_t ch, int pwm);

    pcnt_unit_handle_t unit1, unit2, unit3, unit4;
    pcnt_channel_handle_t enc1, enc2, enc3, enc4;
    PIDController pid1, pid2, pid3, pid4;
};

#endif