#ifndef LOCAL_GOAL_MANAGER_H
#define LOCAL_GOAL_MANAGER_H

#include <vector>
#include <string>
#include <cmath>

struct LocalGoal {
    double x;
    double y;
    double yaw_rad;
    double radius_thresh;
    double yaw_thresh_rad;
};

class LocalGoalManager {
public:
    explicit LocalGoalManager(const std::string& goal_file, int look_ahead = 0);

    void update(double robot_x, double robot_y, double robot_yaw_rad);

    struct GoalDelta {
        double dx;
        double dy;
        double dyaw_rad;
    };

    GoalDelta getDeltaToGoal(double robot_x, double robot_y, double robot_yaw_rad) const;

    bool isGoalReached(double robot_x, double robot_y, double robot_yaw_rad) const;

private:
    std::vector<LocalGoal> goals_;
    int current_index_ = 0;
    int look_ahead_ = 0;

    double normalize_angle(double angle) const {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

#endif
