#pragma once
#include <chrono>

class PID {
    double kp;
    double ki;
    double kd;
    double last_error;
    double integral_error;
    double accel_limit;
    std::chrono::_V2::system_clock::time_point last_time;
public:
    PID(double kp, double ki, double kd, double al);
    float Calculate(float target, float current);
    void Reset();
};