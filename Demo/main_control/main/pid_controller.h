#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float max_out);
    float compute(float target, float current);
    void reset();

private:
    float m_kp, m_ki, m_kd;
    float m_max_out;
    float m_err, m_last_err, m_integral;
};

#endif