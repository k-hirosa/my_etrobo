#include "local_goal_manager.h"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <limits>

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

LocalGoalManager::LocalGoalManager(const std::string& yaml_path, int lookahead)
    : current_index_(0), lookahead_(lookahead) {
    YAML::Node config = YAML::LoadFile(yaml_path);
    for (const auto& node : config["goals"]) {
        LocalGoal g;
        g.x = node["x"].as<double>();
        g.y = node["y"].as<double>();
        g.yaw_rad = node["yaw"].as<double>();
        g.radius_thresh = node["radius_thresh"].as<double>();
        g.yaw_thresh_rad = node["yaw_thresh_rad"].as<double>();
        goals_.push_back(g);
    }
}

void LocalGoalManager::update(double robot_x, double robot_y, double robot_yaw_rad) {
    if (goals_.empty()) return;

    int nearest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (int i = 0; i < goals_.size(); ++i) {
        double dx = goals_[i].x - robot_x;
        double dy = goals_[i].y - robot_y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }

    current_index_ = std::min(nearest_idx + lookahead_, (int)goals_.size() - 1);
}

LocalGoal LocalGoalManager::getCurrentGoal() const {
    return goals_[current_index_];
}

int LocalGoalManager::getCurrentGoalIndex() const {
    return current_index_;
}
