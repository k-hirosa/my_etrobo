#ifndef FORCE_SENSOR_WRAPPER_H
#define FORCE_SENSOR_WRAPPER_H

#include "ForceSensor.h"

using namespace spikeapi;

class ForceSensorWrapper {
public:
    ForceSensorWrapper(EPort port);

    bool getTouched();
private:
    ForceSensor force_sensor;
};

#endif