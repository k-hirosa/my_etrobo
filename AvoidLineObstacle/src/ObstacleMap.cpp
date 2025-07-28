#include "obstacle_map.h"
#include <cmath>

ObstacleMap::ObstacleMap(double merge_threshold)
    : merge_threshold_(merge_threshold) {}

void ObstacleMap::updateRobotPose(double x, double y, double theta) {
    robot_x_ = x;
    robot_y_ = y;
    robot_theta_ = theta;
}

void ObstacleMap::addObstacle(double robot_x, double robot_y, double robot_theta,
                              double us_distance, double us_angle) {
    double global_x = robot_x + us_distance * std::cos(us_angle + robot_theta);
    double global_y = robot_y + us_distance * std::sin(us_angle + robot_theta);

    for (auto& obs : obstacles_) {
        double dx = obs.x - global_x;
        double dy = obs.y - global_y;
        if (std::hypot(dx, dy) < merge_threshold_) {
            obs.x = 0.5 * (obs.x + global_x);
            obs.y = 0.5 * (obs.y + global_y);
            return;
        }
    }

    obstacles_.push_back({global_x, global_y});
}

void ObstacleMap::initialize(const std::vector<std::pair<double, double>>& known_obstacles) {
    obstacles_.clear();
    for (const auto& [x, y] : known_obstacles) {
        obstacles_.push_back({x, y});
    }
}

std::vector<std::pair<double, double>> ObstacleMap::getObstacles() const {
    std::vector<std::pair<double, double>> out;
    for (const auto& obs : obstacles_) {
        out.emplace_back(obs.x, obs.y);
    }
    return out;
}

std::vector<std::pair<double, double>> ObstacleMap::getObstaclesRelative() const {
    std::vector<std::pair<double, double>> out;
    for (const auto& obs : obstacles_) {
        double dx = obs.x - robot_x_;
        double dy = obs.y - robot_y_;

        double rel_x = std::cos(-robot_theta_) * dx - std::sin(-robot_theta_) * dy;
        double rel_y = std::sin(-robot_theta_) * dx + std::cos(-robot_theta_) * dy;

        out.emplace_back(rel_x, rel_y);
    }
    return out;
}