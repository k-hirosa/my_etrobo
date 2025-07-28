#ifndef OBSTACLE_AWARE_PID_CONTROLLER_H
#define OBSTACLE_AWARE_PID_CONTROLLER_H

#include "obstacle_map.h"
#include "pid_controller.h"
#include <utility>

class ObstacleAwarePIDController {
public:
    ObstacleAwarePIDController(ObstacleMap* obstacle_map,
                                double kp = 1.0, double ki = 0.0, double kd = 0.0,
                                double tread_half = 0.1,
                                double kv = 0.5, double v_min = 0.0, double v_max = 0.3);

    std::pair<double, double> compute(double dx, double dy, double dyaw, double dt);

private:
    ObstacleMap* obstacle_map_;
    PIDController heading_pid_;
    double tread_half_;  // トレッド幅の半分 [m]
    double kv_;          // 並進速度スケーリング係数
    double v_min_, v_max_; // 並進速度の範囲
};

#endif
