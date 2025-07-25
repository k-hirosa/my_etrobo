#include "avoid_line_obstacle.h"
#include <yaml-cpp/yaml.h>
#include <csignal>

AvoidLineObstacle::AvoidLineObstacle(const std::string& config_path)
{
    YAML::Node config = YAML::LoadFile(config_path);

    double merge_threshold = config["obstacle_map"]["merge_threshold"].as<double>();
    double kp = config["pid_controller"]["kp"].as<double>();
    double ki = config["pid_controller"]["ki"].as<double>();
    double kd = config["pid_controller"]["kd"].as<double>();
    double tread_half = config["robot"]["tread_half"].as<double>();
    double kv = config["pid_controller"]["kv"].as<double>();
    double v_min = config["pid_controller"]["v_min"].as<double>();
    double v_max = config["pid_controller"]["v_max"].as<double>();
    std::string goal_file = config["local_goal_manager"]["goal_file"].as<std::string>();

    map_ = std::make_unique<ObstacleMap>(merge_threshold);
    goal_manager_ = std::make_unique<LocalGoalManager>(goal_file);

    if (config["obstacle_map"]["known_obstacles"]) {
        std::vector<std::pair<double, double>> known_obs;
        for (const auto& node : config["obstacle_map"]["known_obstacles"]) {
            double x = node[0].as<double>();
            double y = node[1].as<double>();
            known_obs.emplace_back(x, y);
        }
        map_->initialize(known_obs);
    }

    controller_ = std::make_unique<ObstacleAwarePIDController>(map_.get(), kp, ki, kd, tread_half, kv, v_min, v_max);
}

bool AvoidLineObstacle::GetIsGoalReached() const {
    return is_goal_reached_;
}

std::pair<double, double> AvoidLineObstacle::process(double x, double y, double theta, double dt)
{
    is_goal_reached_ = local_goal_manager_->isGoalReached(x, y, theta)
    map_->updateRobotPose(x, y, theta);
    goal_manager_->update(x, y, theta);
    GoalDelta delta = goal_manager_->getDeltaToGoal(x, y, theta);
    return controller_->compute(delta.dx, delta.dy, delta.dyaw_rad, dt);
}
