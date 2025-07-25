#include <yaml-cpp/yaml.h>
#include <vector>
#include <tuple>
#include <string>
#include <cmath>
#include <limits>

using PathPoint = std::tuple<double, double, double>;  // x, y, yaw（yawは未使用）

std::vector<PathPoint> loadGlobalPath(const std::string& yaml_path) {
    std::vector<PathPoint> path;
    YAML::Node yaml = YAML::LoadFile(yaml_path);

    for (const auto& point : yaml["path"]) {
        double x = point["x"].as<double>();
        double y = point["y"].as<double>();
        double yaw = point["yaw"].as<double>();
        path.emplace_back(x, y, yaw);
    }
    return path;
}

// ロボット座標系でローカルゴールを返す
std::pair<double, double> createLocalGoalRobotFrame(
    const std::vector<PathPoint>& path,
    double robot_x, double robot_y, double robot_yaw,
    double dist_threshold)
{
    double min_dist = std::numeric_limits<double>::max();
    size_t nearest_index = 0;

    // 最近傍点を探す
    for (size_t i = 0; i < path.size(); ++i) {
        double px = std::get<0>(path[i]);
        double py = std::get<1>(path[i]);
        double dist = std::hypot(px - robot_x, py - robot_y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_index = i;
        }
    }

    // dist_threshold以上離れた点を探す
    for (size_t i = nearest_index + 1; i < path.size(); ++i) {
        double gx = std::get<0>(path[i]);
        double gy = std::get<1>(path[i]);
        double dist = std::hypot(gx - robot_x, gy - robot_y);
        if (dist >= dist_threshold) {
            // ロボット座標系に変換
            double dx = gx - robot_x;
            double dy = gy - robot_y;
            double x_r =  std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
            double y_r = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
            return {x_r, y_r};
        }
    }

    // ゴールに近すぎる場合は末尾をロボット座標系で返す
    double gx = std::get<0>(path.back());
    double gy = std::get<1>(path.back());
    double dx = gx - robot_x;
    double dy = gy - robot_y;
    double x_r =  std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
    double y_r = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
    return {x_r, y_r};
}

