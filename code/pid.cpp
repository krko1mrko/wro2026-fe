#include "pid.h"
#include <algorithm>

PID::PID(double kp, double ki, double kd, double al) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->last_error = 0;
    this->integral_error = 0;
    this->accel_limit = al;
}

float PID::Calculate(float target, float current) {
    auto now = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(last_time - now).count() > 50) {
        this->integral_error = 0;
    }
    this->last_time = now;
    double er = target - current;
    this->integral_error += er;
    double derivative_error = er - this->last_error;
    this->last_error = er;
    double power = kp * er + ki * integral_error + kd * derivative_error;

    return (float)power;
}