#include "obstacle_map.h"
#include <cmath>
#include <algorithm>

ObstacleMap::ObstacleMap(double merge_threshold)
    : merge_threshold_(merge_threshold) {}

void ObstacleMap::initialize(const std::vector<std::pair<double, double>>& known_obstacles) {
    obstacles_.clear();
    for (const auto& [x, y] : known_obstacles) {
        obstacles_.push_back({x, y});
    }
}

void ObstacleMap::addObstacle(double robot_x, double robot_y, double robot_theta,
                              double us_distance, double us_angle) {
    double obs_x = robot_x + us_distance * std::cos(robot_theta + us_angle);
    double obs_y = robot_y + us_distance * std::sin(robot_theta + us_angle);

    obstacles_.erase(
        std::remove_if(obstacles_.begin(), obstacles_.end(),
            [&](const Obstacle& obs) {
                double dx = obs.x - obs_x;
                double dy = obs.y - obs_y;
                return std::hypot(dx, dy) < merge_threshold_;
            }),
        obstacles_.end()
    );

    obstacles_.push_back({obs_x, obs_y});
}

std::vector<std::pair<double, double>> ObstacleMap::getObstacles() const {
    std::vector<std::pair<double, double>> out;
    for (const auto& obs : obstacles_) {
        out.emplace_back(obs.x, obs.y);
    }
    return out;
}