#ifndef LOCAL_GOAL_MANAGER_H
#define LOCAL_GOAL_MANAGER_H

#include <vector>
#include <string>

struct LocalGoal {
    double x;
    double y;
    double yaw_rad;
    double radius_thresh;
    double yaw_thresh_rad;
};

class LocalGoalManager {
public:
    LocalGoalManager(const std::string& yaml_path, int lookahead = 0);

    void update(double robot_x, double robot_y, double robot_yaw_rad);

    LocalGoal getCurrentGoal() const;

    int getCurrentGoalIndex() const;

private:
    std::vector<LocalGoal> goals_;
    int current_index_;
    int lookahead_;
};

#endif
