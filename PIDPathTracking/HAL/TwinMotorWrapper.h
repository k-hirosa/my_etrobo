#ifndef TWIN_MOTOR_WRAPPER_H
#define TWIN_MOTOR_WRAPPER_H

#include "Motor.h"

class TwinMotorWrapper{
    public:
        TwinMotorWrapper(EPort rightMotor, EPort leftMotor); //モーターを指定
        ~TwinMotorWrapper(); //デストラクタ
        void setPower(int rightPower, int leftPower); //デューティ比セット
        void hold(); //ホールドして角度固定
        void brake(); //ブレーキ
        void stop(); //モーターストップ
        void setDutyLimit(int dutyRight, int dutyLeft); //デューティ比セット
        void restoreDutyLimit(int oldValueRight, int oldValueLeft); //デューティ比リセット
        spikeapi::Motor* getRightMotorPtr(); //右モーターのポインタ返す
        spikeapi::Motor* getLeftMotorPtr(); //左モーターのポインタ返す
    private:
        spikeapi::Motor* mRightMotor;
        spikeapi::Motor* mLeftMotor;

};

#endif