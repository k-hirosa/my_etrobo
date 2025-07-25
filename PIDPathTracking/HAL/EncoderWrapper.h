#ifndef ENCODER_WRAPPER_H
#define ENCODER_WRAPPER_H

#pragma once
#include "Motor.h"

class EncoderWrapper {
public:
    EncoderWrapper(spikeapi::Motor* motor);
    ~EncoderWrapper();
    void resetCount();       //エンコーダリセット
    int getCount();      // 角度[deg]
    int getSpeed();      // 角速度[deg/s]
    int getPower();      // パワー
    bool isStalled();        // ストール状態

private:
    spikeapi::Motor* mMotor;
};

#endif