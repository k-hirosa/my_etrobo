#include "local_goal_manager.h"
#include <fstream>
#include <sstream>
#include <limits>

LocalGoalManager::LocalGoalManager(const std::string& goal_file, int look_ahead)
    : look_ahead_(look_ahead)
{
    std::ifstream file(goal_file);
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        LocalGoal goal;
        iss >> goal.x >> goal.y >> goal.yaw_rad >> goal.radius_thresh >> goal.yaw_thresh_rad;
        goals_.push_back(goal);
    }
}

void LocalGoalManager::update(double robot_x, double robot_y, double robot_yaw_rad)
{
    if (goals_.empty()) return;

    // 最近傍ゴールのインデックスを探索
    int nearest_index = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(goals_.size()); ++i) {
        double dx = goals_[i].x - robot_x;
        double dy = goals_[i].y - robot_y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_index = i;
        }
    }

    // look-aheadの適用（ゴールの末尾を越えないように制限）
    current_index_ = std::min(nearest_index + look_ahead_, static_cast<int>(goals_.size()) - 1);
}

LocalGoalManager::GoalDelta LocalGoalManager::getDeltaToGoal(double robot_x, double robot_y, double robot_yaw_rad) const
{
    GoalDelta delta{0.0, 0.0, 0.0};
    if (current_index_ >= static_cast<int>(goals_.size())) return delta;

    const auto& goal = goals_[current_index_];
    double dx = goal.x - robot_x;
    double dy = goal.y - robot_y;
    double rel_x = std::cos(-robot_yaw_rad) * dx - std::sin(-robot_yaw_rad) * dy;
    double rel_y = std::sin(-robot_yaw_rad) * dx + std::cos(-robot_yaw_rad) * dy;

    delta.dx = rel_x;
    delta.dy = rel_y;
    delta.dyaw_rad = normalize_angle(goal.yaw_rad - robot_yaw_rad);
    return delta;
}

bool LocalGoalManager::isGoalReached(double robot_x, double robot_y, double robot_yaw_rad) const
{
    if (goals_.empty() || current_index_ < static_cast<int>(goals_.size()) - 1) return false;

    const auto& goal = goals_.back();
    double dx = goal.x - robot_x;
    double dy = goal.y - robot_y;
    double dist = std::hypot(dx, dy);
    double dyaw = normalize_angle(goal.yaw_rad - robot_yaw_rad);

    return dist < goal.radius_thresh && std::abs(dyaw) < goal.yaw_thresh_rad;
}
