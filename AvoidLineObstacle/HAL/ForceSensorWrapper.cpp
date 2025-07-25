#include "ForceSensorWrapper.h"

using namespace spikeapi;

ForceSensorWrapper::ForceSensorWrapper(EPort port): force_sensor(port) {}

bool ForceSensorWrapper::getTouched() {
    return force_sensor.isTouched();
}