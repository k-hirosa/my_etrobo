#ifndef ULTRASONIC_SENSOR_WRAPPER_H
#define ULTRASONIC_SENSOR_WRAPPER_H

#include <stdint.h>
#include "UltrasonicSensor.h"

using namespace spikeapi;

class UltrasonicSensorWrapper
{
    public:
        UltrasonicSensorWrapper(EPort port);
        int32_t measureDistance() const;
    private:
        UltrasonicSensor mSensor;
};
#endif // ULTRASONIC_SENSOR_WRAPPER_H