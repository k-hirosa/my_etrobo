
#include "obstacle_map.h"
#include "obstacle_aware_pid_controller.h"
#include "local_goal_manager.h"
#include "logger.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

Logger logger;

void sigint_handler(int) {
    std::cout << "\nSIGINT caught! Saving log...\n";
    logger.saveCsv("log.csv");
    std::exit(0);
}

int main() {
    YAML::Node config = YAML::LoadFile("config.yaml");

    double merge_threshold = config["obstacle_map"]["merge_threshold"].as<double>();
    double kp = config["pid_controller"]["kp"].as<double>();
    double ki = config["pid_controller"]["ki"].as<double>();
    double kd = config["pid_controller"]["kd"].as<double>();
    std::string goal_file = config["local_goal_manager"]["goal_file"].as<std::string>();

    ObstacleMap map(merge_threshold);

    if (config["obstacle_map"]["known_obstacles"]) {
        std::vector<std::pair<double, double>> known_obs;
        for (const auto& node : config["obstacle_map"]["known_obstacles"]) {
            double x = node[0].as<double>();
            double y = node[1].as<double>();
            known_obs.emplace_back(x, y);
        }
        map.initialize(known_obs);
    }

    ObstacleAwarePIDController controller(&map, kp, ki, kd);
    LocalGoalManager goal_manager(goal_file);

    signal(SIGINT, sigint_handler);

    double x = 0.0, y = 0.0, theta = 0.0;
    double dt = 0.01;

    for (int i = 0; i < 1000; ++i) {
        goal_manager.update(x, y, theta);
        LocalGoal current_goal = goal_manager.getCurrentGoal();

        double us_distance = 1.0;
        double us_angle = 0.0;

        map.addObstacle(x, y, theta, us_distance, us_angle);

        auto [pwm_l, pwm_r] = controller.compute(x, y, theta,
                                                 current_goal.x, current_goal.y, dt);

        logger.addLog({
            {"time", i*dt},
            {"x", x},
            {"y", y},
            {"theta", theta},
            {"goal_x", current_goal.x},
            {"goal_y", current_goal.y},
            {"pwm_l", pwm_l},
            {"pwm_r", pwm_r}
        });

        std::cout << "Step " << i
                  << " PWM_L: " << pwm_l << ", PWM_R: " << pwm_r << "\n";

        x += 0.01 * std::cos(theta);
        y += 0.01 * std::sin(theta);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    logger.saveCsv("log.csv");

    return 0;
}
