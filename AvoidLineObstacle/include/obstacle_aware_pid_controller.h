#ifndef OBSTACLE_AWARE_PID_CONTROLLER_H
#define OBSTACLE_AWARE_PID_CONTROLLER_H

#include "obstacle_map.h"
#include "pid_controller.h"
#include <utility>

class ObstacleAwarePIDController {
public:
    ObstacleAwarePIDController(ObstacleMap* obstacle_map,
                                double kp = 1.0, double ki = 0.0, double kd = 0.0);

    std::pair<double, double> compute(double x, double y, double theta,
                                      double goal_x, double goal_y, double dt);

private:
    ObstacleMap* obstacle_map_;
    PIDController heading_pid_;
};

#endif
