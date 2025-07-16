#include "TwinMotorWrapper.h"

#define CW spikeapi::Motor::EDirection::CLOCKWISE
#define CCW spikeapi::Motor::EDirection::COUNTERCLOCKWISE

TwinMotorWrapper::TwinMotorWrapper(EPort rightPort,EPort leftPort)
{
    mRightMotor = new spikeapi::Motor(rightPort,CW, true);
    mLeftMotor = new spikeapi::Motor(leftPort,CCW,true);
}

TwinMotorWrapper::~TwinMotorWrapper()
{
    delete mRightMotor;
    delete mLeftMotor;
}

void TwinMotorWrapper::setPower(int rightPower, int leftPower) //デューティ比セット
{
    mRightMotor -> setPower(rightPower);
    mLeftMotor -> setPower(leftPower);
}

void TwinMotorWrapper::hold() //ホールドして角度固定
{
    mRightMotor -> hold();
    mLeftMotor -> hold();
}

void TwinMotorWrapper::brake() //ブレーキ
{
    mRightMotor -> brake();
    mLeftMotor -> brake();
}

void TwinMotorWrapper::stop() //モーターストップ
{
    mRightMotor -> stop();
    mLeftMotor -> stop();
}

void TwinMotorWrapper::setDutyLimit(int dutyRight, int dutyLeft)
{
    mRightMotor -> setDutyLimit(dutyRight);
    mLeftMotor -> setDutyLimit(dutyLeft);
}

void TwinMotorWrapper::restoreDutyLimit(int oldValueRight,int oldValueLeft)
{
    mRightMotor -> restoreDutyLimit(oldValueRight);
    mLeftMotor -> restoreDutyLimit(oldValueLeft);
}

spikeapi::Motor* TwinMotorWrapper::getRightMotorPtr() //右モーターのポインタ返す
{
    return mRightMotor;
}
        
spikeapi::Motor* TwinMotorWrapper::getLeftMotorPtr() //左モーターのポインタ返す
{
    return mLeftMotor;
}