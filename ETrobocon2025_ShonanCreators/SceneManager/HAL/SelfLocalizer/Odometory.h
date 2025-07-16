#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "EncoderWrapper.h"
#include "RobotPose.h"
#include "Clock.h"

class Odometry {
public:
    Odometry(float wheel_radius, EncoderWrapper* leftEncoder, EncoderWrapper* rightEncoder,
             const RobotPose& initial_pose, Clock* clock);
    void update(float omega_gyro, float theta_gyro);

    RobotPose getPose() const;

private:
    RobotPose m_pose;                 // ロボットの位置と姿勢
    float m_wheel_radius;          // 車輪半径
    float m_t_prev;

    float m_rotationAngle_left_prev;
    float m_rotationAngle_right_prev;

    EncoderWrapper* m_leftEncoder;
    EncoderWrapper* m_rightEncoder;
    Clock* m_clock;
};

#endif // ODOMETRY_H
