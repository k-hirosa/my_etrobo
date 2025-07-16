#include "IMUWrapper.h"
#include "Clock.h"
#include <iostream>
// #include <vector>


IMUWrapper::IMUWrapper():m_angle(0), m_angularvel{0,0,0}, m_bias(0){
    m_Imu = new spikeapi::IMU();
    spikeapi::Clock clock;
    for(int i=0;i<NUM_SAMPLE_DATA;i++){
        calcBias();
        clock.sleep(10*1000);
    }
}

IMUWrapper::~IMUWrapper(){
    delete m_Imu;
}

void IMUWrapper::calcConversionAngularVel(){
    float pitch = DEG2RAD * IMU_PITCH;
    spikeapi::IMU::AngularVelocity raw_angular_vel;
    m_Imu->getAngularVelocity(raw_angular_vel);
    // 変換後の角速度算出
    m_angularvel.x = cos(pitch) * raw_angular_vel.x + sin(pitch) * raw_angular_vel.z;
    m_angularvel.y = raw_angular_vel.y;
    m_angularvel.z = -sin(pitch) * raw_angular_vel.x + cos(pitch) * raw_angular_vel.z;
    //m_angularvel.z = raw_angular_vel.z;
}   


spikeapi::IMU::AngularVelocity IMUWrapper::getAngularVelocity(){
    calcConversionAngularVel();
    m_angularvel.z -= m_bias;
    return m_angularvel;
}


void IMUWrapper::calcAngle(){
    spikeapi::IMU::AngularVelocity angular_vel = getAngularVelocity();
    // m_Imu->getAngularVelocity(angular_vel);
    //前回時刻との差を計算
    uint64_t now = m_clock.now();
    if(now > 100000) {
        now = 10000;
    } //初回など周期から外れすぎている場合は補正(100msにする)
    //printf("%d\n",now);
    m_clock.reset();
    m_angle += angular_vel.z * ((float)now / 1000000); // 10msec周期の場合
    m_angle = std::fmod(m_angle, 360); //0～360degにする
}

float IMUWrapper::getAngle(){
    return m_angle;
}

void IMUWrapper::calcBias(){
    calcConversionAngularVel();
    m_bias += m_angularvel.z / NUM_SAMPLE_DATA;
    // std::vector<float> bias;
    
    // bias.push_back(angular_v.z);
    // float sum = std::accumulate(bias.begin(), bias.end(), 0);
    // m_bias = sum / bias.size(); 
}
