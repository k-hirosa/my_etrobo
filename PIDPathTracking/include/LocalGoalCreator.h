#ifndef LOCAL_GOAL_CREATOR_H
#define LOCAL_GOAL_CREATOR_H

#include "LocalGoals.h"
struct PathPoint {
    double x;
    double y;
    double yaw;
};

class LocalGoalCreator {
public:
    LocalGoalCreator();

    PathPoint createLocalGoalRobotFrame(
        double robot_x, double robot_y, double robot_yaw,
        double dist_threshold
    ) const;

private:
    PathPoint m_global_path[LOCAL_PATH_COUNT];
    void loadGlobalPath();
};

#endif  // LOCAL_GOAL_CREATOR_H
