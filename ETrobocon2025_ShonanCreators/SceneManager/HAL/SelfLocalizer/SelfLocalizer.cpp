#include "SelfLocalizer.h"
#include "Clock.h"
#include <cmath>

SelfLocalizer::SelfLocalizer(float wheel_radius, const RobotPose& initial_pose, spikeapi::Motor* rightMotor, spikeapi::Motor* leftMotor)
                    : m_leftEncoder(leftMotor), m_rightEncoder(rightMotor), m_currentYaw(), m_wheel_radius(wheel_radius), 
                    m_pose(initial_pose), m_rotationAngle_left_prev(0), m_rotationAngle_right_prev(0)
{
}

SelfLocalizer::~SelfLocalizer()
{
}

void SelfLocalizer::updateLocalization()
{    
    //前回時刻との差を計算
    uint64_t now = m_clock.now();
    if(now > 100000) {
        now = 1000;
    } //初回など周期から外れすぎている場合は補正(10msにする)
    m_clock.reset();
    float dt = (float)now/1000;  //要変更

    //左右輪の回転角度
    float rotationAngle_left = m_leftEncoder.getCount();
    float rotationAngle_right = m_rightEncoder.getCount();

    //左右輪の角速度
    float omega_L = (rotationAngle_left - m_rotationAngle_left_prev) * DEG2RAD / dt;    
    float omega_R = (rotationAngle_right - m_rotationAngle_right_prev) * DEG2RAD / dt;

    //左右輪の直進速度
    float v_L = m_wheel_radius * omega_L;
    float v_R = m_wheel_radius * omega_R;

    //車体中心の並進速度
    float v = (v_L + v_R) / 2.0;
    
    //現在旋回角度
    m_currentYaw.calcAngle();  
    
    //poseの更新
    m_pose.m_yaw = m_currentYaw.getAngle() * DEG2RAD;
    m_pose.m_x += v * std::cos(m_pose.m_yaw) * dt;
    m_pose.m_y += v * std::sin(m_pose.m_yaw) * dt;

    //値の更新
    m_rotationAngle_left_prev = rotationAngle_left;
    m_rotationAngle_right_prev = rotationAngle_right;
}

RobotPose SelfLocalizer::getLocalization()
{
    return m_pose;
}