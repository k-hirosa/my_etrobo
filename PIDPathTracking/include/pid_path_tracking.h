#pragma once
#include "pid_controller.h"
#include <utility>
#include <string>

class PIDPathTracking {
public:
    PIDPathTracking(const std::string& yaml_path);
    std::pair<double, double> computePWM(double goal_x_r, double goal_y_r);
    void reset();

private:
    double v_;     // 並進速度
    double L_;     // トレッド（車輪間距離）
    PIDController pid_;
};

