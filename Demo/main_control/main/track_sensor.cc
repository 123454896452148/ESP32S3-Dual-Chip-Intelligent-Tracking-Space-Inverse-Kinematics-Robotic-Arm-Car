#include "track_sensor.h"
#include "config.h"
#include "esp_log.h"

static const char* TAG = "Track";

TrackSensor& TrackSensor::getInstance() { static TrackSensor inst; return inst; }

void TrackSensor::init() {
    ESP_LOGI(TAG, "初始化循迹传感器...");
    gpio_config_t cfg{};
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pin_bit_mask = (1ULL << S1) | (1ULL << S2) | (1ULL << S3) | (1ULL << S4) | (1ULL << S5);
    gpio_config(&cfg);
    ESP_LOGI(TAG, "循迹传感器初始化完成");
}

void TrackSensor::read(int &s1, int &s2, int &s3, int &s4, int &s5) {
    s1 = gpio_get_level(S1); s2 = gpio_get_level(S2); s3 = gpio_get_level(S3);
    s4 = gpio_get_level(S4); s5 = gpio_get_level(S5);
    ESP_LOGV(TAG, "读取传感器: S1=%d S2=%d S3=%d S4=%d S5=%d", s1, s2, s3, s4, s5);
}

bool TrackSensor::isTarget() {
    int s1,s2,s3,s4,s5; read(s1,s2,s3,s4,s5);
    bool target = (s1==0 && s2==0 && s3==0 && s4==0 && s5==0);
    if (target) {
        ESP_LOGI(TAG, "检测到目标点");
    }
    return target;
}