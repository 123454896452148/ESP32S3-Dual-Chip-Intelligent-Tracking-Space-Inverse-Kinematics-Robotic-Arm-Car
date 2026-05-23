#include "car_control.h"
#include "config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "Car";

CarControl::CarControl()
    : pid1(6.0f, 0.8f, 1.5f, 500),
      pid2(6.0f, 0.8f, 1.5f, 500),
      pid3(6.0f, 0.8f, 1.5f, 500),
      pid4(6.0f, 0.8f, 1.5f, 500) {}

CarControl& CarControl::getInstance() {
    static CarControl inst;
    return inst;
}

void CarControl::initGPIO() {
    ESP_LOGD(TAG, "初始化GPIO...");
    gpio_config_t o{};
    o.mode = GPIO_MODE_OUTPUT;
    o.pin_bit_mask = (1ULL << AIN1) | (1ULL << AIN2) | (1ULL << BIN1) | (1ULL << BIN2)
                   | (1ULL << CIN1) | (1ULL << CIN2) | (1ULL << DIN1) | (1ULL << DIN2);
    gpio_config(&o);
    ESP_LOGD(TAG, "GPIO初始化完成");
}

void CarControl::initPWM() {
    ESP_LOGD(TAG, "初始化PWM...");
    ledc_timer_config_t t{};
    t.speed_mode = LEDC_LOW_SPEED_MODE;
    t.duty_resolution = LEDC_TIMER_10_BIT;
    t.freq_hz = 2000;
    t.timer_num = LEDC_TIMER_0;
    t.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&t);

    ledc_channel_config_t ch{};
    ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.timer_sel = LEDC_TIMER_0;
    ch.duty = 0;

    ch.gpio_num = PWMA; ch.channel = LEDC_CHANNEL_0; ledc_channel_config(&ch);
    ch.gpio_num = PWB;  ch.channel = LEDC_CHANNEL_1; ledc_channel_config(&ch);
    ch.gpio_num = PWC;  ch.channel = LEDC_CHANNEL_2; ledc_channel_config(&ch);
    ch.gpio_num = PWD;  ch.channel = LEDC_CHANNEL_3; ledc_channel_config(&ch);
    ESP_LOGD(TAG, "PWM初始化完成");
}

void CarControl::initAllEncoders() {
    ESP_LOGD(TAG, "初始化编码器...");
    pcnt_unit_config_t unit_cfg{};
    unit_cfg.low_limit = -1000;
    unit_cfg.high_limit = 1000;

    pcnt_new_unit(&unit_cfg, &unit1);
    pcnt_new_unit(&unit_cfg, &unit2);
    pcnt_new_unit(&unit_cfg, &unit3);
    pcnt_new_unit(&unit_cfg, &unit4);

    pcnt_chan_config_t chan_cfg{};
    chan_cfg.edge_gpio_num = E1A; chan_cfg.level_gpio_num = E1B; pcnt_new_channel(unit1, &chan_cfg, &enc1);
    chan_cfg.edge_gpio_num = E2A; chan_cfg.level_gpio_num = E2B; pcnt_new_channel(unit2, &chan_cfg, &enc2);
    chan_cfg.edge_gpio_num = E3A; chan_cfg.level_gpio_num = E3B; pcnt_new_channel(unit3, &chan_cfg, &enc3);
    chan_cfg.edge_gpio_num = E4A; chan_cfg.level_gpio_num = E4B; pcnt_new_channel(unit4, &chan_cfg, &enc4);

    pcnt_unit_enable(unit1); pcnt_unit_enable(unit2);
    pcnt_unit_enable(unit3); pcnt_unit_enable(unit4);
    pcnt_unit_clear_count(unit1); pcnt_unit_clear_count(unit2);
    pcnt_unit_clear_count(unit3); pcnt_unit_clear_count(unit4);
    ESP_LOGD(TAG, "编码器初始化完成");
}

void CarControl::initAll() {
    ESP_LOGI(TAG, "初始化汽车控制模块...");
    initGPIO(); initPWM(); initAllEncoders(); stop();
    ESP_LOGI(TAG, "汽车控制模块初始化完成");
}

int CarControl::getSpeed(pcnt_unit_handle_t unit) {
    int cnt;
    pcnt_unit_get_count(unit, &cnt);
    pcnt_unit_clear_count(unit);
    return cnt;
}

void CarControl::setMotor(gpio_num_t in1, gpio_num_t in2, ledc_channel_t ch, int pwm) {
    if (pwm > 0) {
        gpio_set_level(in1, 1); gpio_set_level(in2, 0);
    } else if (pwm < 0) {
        gpio_set_level(in1, 0); gpio_set_level(in2, 1);
    } else {
        gpio_set_level(in1, 0); gpio_set_level(in2, 0);
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, abs(pwm));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

void CarControl::setSpeed(int m1, int m2, int m3, int m4) {
    setMotor(AIN1, AIN2, LEDC_CHANNEL_0, m1);
    setMotor(BIN1, BIN2, LEDC_CHANNEL_1, m2);
    setMotor(CIN1, CIN2, LEDC_CHANNEL_2, m3);
    setMotor(DIN1, DIN2, LEDC_CHANNEL_3, m4);
}
//  停止
void CarControl::stop() {
    setSpeed(0,0,0,0);
}
//  刹车
void CarControl::brake() {
    setSpeed(255,255,255,255);
    vTaskDelay(pdMS_TO_TICKS(50));
    stop();
}
//  直行
void CarControl::moveStraight(int speed) {
    setSpeed(speed,speed,speed,speed);
}
//  后退
void CarControl::moveBack(int speed) {
    setSpeed(-speed,-speed,-speed,-speed);
}
//  左转
void CarControl::turnLeft() {
    setSpeed(100,100,160,160);
}
//  右转
void CarControl::turnRight() {
    setSpeed(160,160,100,100);
}
//  顺时针旋转
void CarControl::spinLeft(int speed) {
    setSpeed(-speed,-speed,speed,speed);
}
//  逆时针旋转
void CarControl::spinRight(int speed) {
    setSpeed(speed,speed,-speed,-speed);
}