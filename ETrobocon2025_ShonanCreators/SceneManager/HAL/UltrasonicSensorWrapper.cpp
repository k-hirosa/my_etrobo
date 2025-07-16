#include "UltrasonicSensorWrapper.h"

UltrasonicSensorWrapper::UltrasonicSensorWrapper(EPort port):mSensor(port){
}

int32_t UltrasonicSensorWrapper::measureDistance() const{
    return mSensor.getDistance();
}


