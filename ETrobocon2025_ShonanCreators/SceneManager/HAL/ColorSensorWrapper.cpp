#include "ColorSensorWrapper.h"

using namespace spikeapi;

ColorSensorWrapper::ColorSensorWrapper(EPort port): color_sensor(port) {}

void ColorSensorWrapper::update() {
    color_sensor.getRGB(rgb);
    color_sensor.getHSV(hsv);
}

const ColorSensor::RGB& ColorSensorWrapper::getRGB() const {
    return rgb;
}

const ColorSensor::HSV& ColorSensorWrapper::getHSV() const {
    return hsv;
}
