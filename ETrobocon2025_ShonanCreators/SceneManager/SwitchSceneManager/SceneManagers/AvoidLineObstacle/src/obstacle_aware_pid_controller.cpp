#include "obstacle_aware_pid_controller.h"
#include <cmath>
#include <algorithm>

ObstacleAwarePIDController::ObstacleAwarePIDController(
    ObstacleMap* obstacle_map,
    double kp, double ki, double kd,
    double tread_half,
    double kv, double v_min, double v_max)
    : obstacle_map_(obstacle_map),
      heading_pid_(kp, ki, kd, -1.0, 1.0, 0.0, 0.01),
      tread_half_(tread_half),
      kv_(kv), v_min_(v_min), v_max_(v_max) {}

std::pair<double, double> ObstacleAwarePIDController::compute(
    double dx, double dy, double dyaw, double dt) {

    double avoid_x = 0.0, avoid_y = 0.0;
    // auto obstacles = obstacle_map_->getObstacles();  // 相対座標と仮定
    auto obstacles = obstacle_map_->getObstaclesRelative();  // ロボット座標系の相対位置を取得

    for (const auto& [obs_x, obs_y] : obstacles) {
        double dist_obs = std::hypot(obs_x, obs_y);
        if (dist_obs < 1.0) {
            avoid_x -= obs_x / dist_obs;
            avoid_y -= obs_y / dist_obs;
        }
    }

    double combined_x = dx + avoid_x;
    double combined_y = dy + avoid_y;

    double desired_theta = std::atan2(combined_y, combined_x);
    double heading_error = std::atan2(std::sin(desired_theta - dyaw),
                                      std::cos(desired_theta - dyaw));

    double omega = heading_pid_.process(0.0, -heading_error);

    // 並進速度はロボット前方方向（dx）に応じてスケーリング
    double v = std::clamp(dx * kv_, v_min_, v_max_);

    // 左右の速度計算（差動駆動モデル）
    double left = v - omega * tread_half_;
    double right = v + omega * tread_half_;

    return {left, right};
}
