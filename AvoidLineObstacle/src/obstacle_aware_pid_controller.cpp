#include "obstacle_aware_pid_controller.h"
#include <cmath>

ObstacleAwarePIDController::ObstacleAwarePIDController(ObstacleMap* obstacle_map,
                                                       double kp, double ki, double kd)
    : obstacle_map_(obstacle_map),
      heading_pid_(kp, ki, kd, -1.0, 1.0, 0, 0.01) {}

std::pair<double, double> ObstacleAwarePIDController::compute(
    double x, double y, double theta,
    double goal_x, double goal_y, double dt) {

    double dx = goal_x - x;
    double dy = goal_y - y;

    double avoid_x = 0.0, avoid_y = 0.0;
    auto obstacles = obstacle_map_->getObstacles();

    for (const auto& [obs_x, obs_y] : obstacles) {
        double dx_obs = obs_x - x;
        double dy_obs = obs_y - y;
        double dist_obs = std::hypot(dx_obs, dy_obs);
        if (dist_obs < 1.0) {
            avoid_x -= dx_obs / dist_obs;
            avoid_y -= dy_obs / dist_obs;
        }
    }

    double combined_x = dx + avoid_x;
    double combined_y = dy + avoid_y;

    double desired_theta = std::atan2(combined_y, combined_x);
    double heading_error = std::atan2(std::sin(desired_theta - theta), std::cos(desired_theta - theta));

    double omega = heading_pid_.process(0.0, -heading_error);

    double v = 0.3;

    double left = v - omega * 0.1;
    double right = v + omega * 0.1;

    return {left, right};
}
