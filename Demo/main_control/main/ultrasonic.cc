#include "ultrasonic.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "config.h"
#include "esp_log.h"

static const char* TAG = "Ultrasonic";

Ultrasonic& Ultrasonic::getInstance() { static Ultrasonic inst; return inst; }

void Ultrasonic::init() {
    ESP_LOGI(TAG, "初始化超声波传感器...");
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_level(TRIG_PIN, 0);
    ESP_LOGI(TAG, "超声波传感器初始化完成");
}

int Ultrasonic::getDistance() {
    gpio_set_level(TRIG_PIN, 1); ets_delay_us(10); gpio_set_level(TRIG_PIN, 0);
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0 && esp_timer_get_time() - start < 50000);
    start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1 && esp_timer_get_time() - start < 50000);
    int distance = (esp_timer_get_time() - start) * 0.034 / 2;
    ESP_LOGV(TAG, "测量距离: %d cm", distance);
    return distance;
}

bool Ultrasonic::isObstacle(int limit) { 
    int dist = getDistance();
    bool obstacle = (dist < limit && dist > 0);
    if (obstacle) {
        ESP_LOGW(TAG, "检测到障碍物，距离: %d cm (阈值: %d cm)", dist, limit);
    }
    return obstacle; 
}