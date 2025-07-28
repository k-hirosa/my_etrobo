#include "LocalGoalCreator.h"
#include <cmath>
#include <limits>
#include <cstdio> // for printf

LocalGoalCreator::LocalGoalCreator() {
    loadGlobalPath();
}

void LocalGoalCreator::loadGlobalPath() {
    for (int i = 0; i < LOCAL_PATH_COUNT; ++i) {
        m_global_path[i].x = static_cast<double>(LOCAL_PATH[i].x);
        m_global_path[i].y = static_cast<double>(LOCAL_PATH[i].y);
        m_global_path[i].yaw = static_cast<double>(LOCAL_PATH[i].yaw);
    }
}

PathPoint LocalGoalCreator::createLocalGoalRobotFrame(
    double robot_x, double robot_y, double robot_yaw,
    double dist_threshold
) const {
    double min_dist = std::numeric_limits<double>::max();
    int nearest_index = 0;

    for (int i = 0; i < LOCAL_PATH_COUNT; ++i) {
        double px = m_global_path[i].x;
        double py = m_global_path[i].y;
        double dist = std::sqrt((px - robot_x) * (px - robot_x) + (py - robot_y) * (py - robot_y));
        if (dist < min_dist) {
            min_dist = dist;
            nearest_index = i;
        }
    }
    // printf("Nearest point index: %d, distance: %.2f\n", nearest_index, min_dist);
    for (int i = nearest_index + 1; i < LOCAL_PATH_COUNT; ++i) {
        double gx = m_global_path[i].x;
        double gy = m_global_path[i].y;
        double dist = std::sqrt((gx - robot_x) * (gx - robot_x) + (gy - robot_y) * (gy - robot_y));
        if (dist >= dist_threshold) {
            double dx = gx - robot_x;
            double dy = gy - robot_y;
            double x_r =  std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
            double y_r = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

            // printf("Local goal found at index %d: x=%.2f, y=%.2f\n", i, x_r, y_r);
            return {x_r, y_r};
        }
    }



    // 最後の点を使う
    double gx = m_global_path[LOCAL_PATH_COUNT - 1].x;
    double gy = m_global_path[LOCAL_PATH_COUNT - 1].y;
    double dx = gx - robot_x;
    double dy = gy - robot_y;

    double x_r =  std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
    double y_r = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

    PathPoint local_goal;

    if (nearest_index == LOCAL_PATH_COUNT - 1 && min_dist < dist_threshold) {
        // 最後の点が近い場合は、ロボットの位置をそのままゴールとする
        x_r = 0.0;
        y_r = 0.0;
    }
    local_goal.x = x_r;
    local_goal.y = y_r;
    local_goal.yaw = 0.0;
    return local_goal;
}
