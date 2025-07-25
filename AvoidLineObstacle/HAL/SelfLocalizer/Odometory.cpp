#include "Odometry.h"
#include <cmath>
#include <chrono>

Odometry::Odometry(float wheel_radius, Encoder* leftEncoder, Encoder* rightEncoder,
                    const RobotPose& initial_pose, Clock* clock)
                    : m_wheel_radius(wheel_radius), m_leftEncoder(left), m_rightEncoder(right), 
                    m_pose(initial_pose), m_clock(clock), m_rotationAngle_left_prev(0), m_rotationAngle_right_prev(0)
{
    m_t_prev = m_clock->now();
}

void Odometry::update(float omega_gyro, float theta_gyro) {
    float m_t_now = m_clock->now();
    float dt = m_t_now - m_t_prev;

    //左右輪の回転角度
    float rotationAngle_left = m_leftEncoder->getCount();
    float rotationAngle_right = m_rightEncoder->getCount();

    //左右輪の角速度
    float omega_L = (rotationAngle_left - m_rotationAngle_left_prev) / dt;
    float omega_R = (rotationAngle_right - m_rotationAngle_right_prev) / dt;

    //左右輪の直進速度
    float v_L = r * omega_L;
    float v_R = r * omega_R;

    //車体中心の並進速度
    float v = (v_L + v_R) / 2.0;

    //poseの更新
    m_pose.yaw += omega_gyro * dt;
    m_pose.x += v * std::cos(m_pose.yaw) * dt;
    m_pose.y += v * std::sin(m_pose.yaw) * dt;

    //値の更新
    m_rotationAngle_left_prev = rotationAngle_left;
    m_rotationAngle_right_prev = rotationAngle_right;
    m_t_prev = m_t_now;
}

RobotPose Odometry::getPose() const {
    return m_pose;
}
