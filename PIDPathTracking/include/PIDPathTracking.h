#pragma once
#include "PIDController.h"
#include <utility>
#include <string>

struct PWM{
    double left;
    double right;
};

class PIDPathTracking {
public:
    PIDPathTracking();
    PWM computePWM(double goal_x_r, double goal_y_r);
    void reset();

private:
    double v = 0.1;     // 並進速度
    double L = 0.15;     // トレッド（車輪間距離）
    double dt = 0.1; // 制御周期
    double kp = 0.1; // 比例ゲイン
    double ki = 0.0; // 積分ゲイン
    double kd = 0.0; // 微分ゲイン
    PIDController pid_;
};

