#ifndef IMU_WRAPPER_H
#define IMU_WRAPPER_H

#include "IMU.h"
#include "spikeapi.h"
#include "Clock.h"
#include <cmath>

#define NUM_SAMPLE_DATA 1000
#define IMU_PITCH -50 //50degで取り付けられているため
#define DEG2RAD 3.14159265f / 180.0f


class IMUWrapper 
{
     
    public:
        IMUWrapper();
        ~IMUWrapper();
        void calcConversionAngularVel(); //ローカル座標系に変換
        spikeapi::IMU::AngularVelocity getAngularVelocity(); //角速度出力
        void calcAngle(); //角度算出
        float getAngle(); //角度出力
        void calcBias(); //バイアス算出


    private:
        spikeapi::IMU* m_Imu;
        spikeapi::IMU::AngularVelocity m_angularvel;
        float m_bias;
        float m_angle;
        spikeapi::Clock m_clock; //時間変化測定用
};


#endif // IMU_WRAPPER_H
