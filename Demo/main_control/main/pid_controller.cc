#include "pid_controller.h"
#include "esp_log.h"

static const char* TAG = "PID";

PIDController::PIDController(float kp, float ki, float kd, float max_out)
    : m_kp(kp), m_ki(ki), m_kd(kd), m_max_out(max_out),
      m_err(0), m_last_err(0), m_integral(0) {
    ESP_LOGD(TAG, "PID控制器创建: Kp=%.2f Ki=%.2f Kd=%.2f 最大输出=%.2f", kp, ki, kd, max_out);
}

float PIDController::compute(float target, float current) {
    m_err = target - current;
    m_integral += m_err;

    float out = m_kp * m_err + m_ki * m_integral + m_kd * (m_err - m_last_err);
    m_last_err = m_err;

    if (out > m_max_out) out = m_max_out;
    if (out < -m_max_out) out = -m_max_out;

    ESP_LOGV(TAG, "PID计算: 目标=%.2f 当前=%.2f 误差=%.2f 输出=%.2f", target, current, m_err, out);
    return out;
}

void PIDController::reset() {
    ESP_LOGD(TAG, "PID控制器重置");
    m_integral = 0;
    m_last_err = 0;
}