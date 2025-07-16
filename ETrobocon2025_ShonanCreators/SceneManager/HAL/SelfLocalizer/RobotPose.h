#ifndef ROBOT_POSE_H
#define ROBOT_POSE_H

struct RobotPose
{
    float m_x;
    float m_y;
    float m_yaw;
    RobotPose(float x_init = 0.0, float y_init = 0.0, float yaw_init = 0.0)
        : m_x(x_init), m_y(y_init), m_yaw(yaw_init) {}
};

#endif // ROBOT_POSE_H