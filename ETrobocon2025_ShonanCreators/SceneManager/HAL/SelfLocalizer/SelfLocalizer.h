#ifndef SELF_LOCALIZER_H
#define SELF_LOCALIZER_H

#include "RobotPose.h"
#include "Clock.h"
#include "IMUWrapper.h"
#include "EncoderWrapper.h"
#include "TwinMotorWrapper.h"

using namespace spikeapi;

class Motor;

class SelfLocalizer
{
public:
    SelfLocalizer(float wheel_radius, const RobotPose& initial_pose, spikeapi::Motor* rightMotor, 
    spikeapi::Motor* leftMotor);    
    ~SelfLocalizer();
    void updateLocalization();
    RobotPose getLocalization();

private:
    RobotPose m_pose;                   // ロボットの位置と姿勢
    float m_wheel_radius;               // 車輪半径
    float m_t_prev;                     // 前回の時刻
    float m_rotationAngle_left_prev;    // 前回の左輪回転角度
    float m_rotationAngle_right_prev;   // 前回の右輪回転角度
    EncoderWrapper m_leftEncoder;
    EncoderWrapper m_rightEncoder;
    IMUWrapper m_currentYaw;
    spikeapi::Clock m_clock; //時間変化測定用
};

#endif // SELF_LOCALIZER_H