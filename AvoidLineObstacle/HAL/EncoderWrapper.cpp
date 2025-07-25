#include "EncoderWrapper.h"

EncoderWrapper::EncoderWrapper(spikeapi::Motor* motor)
    : mMotor(motor)
{}

EncoderWrapper::~EncoderWrapper() {}

void EncoderWrapper::resetCount()
{
    mMotor -> resetCount();
}

int EncoderWrapper::getCount() {
    return (int) mMotor->getCount();
}

int EncoderWrapper::getSpeed() {
    return (int) mMotor->getSpeed();
}

int EncoderWrapper::getPower() {
    return (int) mMotor->getPower();
}

bool EncoderWrapper::isStalled() {
    return mMotor->isStalled();
}