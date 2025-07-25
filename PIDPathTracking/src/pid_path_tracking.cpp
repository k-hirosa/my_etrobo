#include "pid_path_tracking.h"
#include <yaml-cpp/yaml.h>
#include <cmath>

PIDPathTracking::PIDPathTracking(const std::string& yaml_path)
    : v_(0.0), L_(0.0), pid_(0, 0, 0, -1, 1, 10, 0.1)
{
    YAML::Node config = YAML::LoadFile(yaml_path);
    const auto& node = config["controller"];

    double kp = node["kp"].as<double>();
    double ki = node["ki"].as<double>();
    double kd = node["kd"].as<double>();
    double output_limit = node["output_limit"].as<double>();
    double dt = node["dt"].as<double>();
    L_ = node["tread"].as<double>();
    v_ = node["linear_velocity"].as<double>();

    pid_ = PIDController(kp, ki, kd, -output_limit, output_limit, 10, dt);
}

std::pair<double, double> PIDPathTracking::computePWM(double goal_x_r, double goal_y_r) {
    double y_error = goal_y_r;
    double omega = pid_.process(0.0, y_error);

    double v_r = v_ + (omega * L_ / 2.0);
    double v_l = v_ - (omega * L_ / 2.0);
    return {v_l, v_r};
}

void PIDPathTracking::reset() {
    pid_.reset();
}

