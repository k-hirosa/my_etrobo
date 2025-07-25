#ifndef COLOR_SENSOR_WRAPPER_H
#define COLOR_SENSOR_WRAPPER_H

#include "ColorSensor.h"

using namespace spikeapi;

class ColorSensorWrapper {
public:
    explicit ColorSensorWrapper(EPort port);

    void update();
    const ColorSensor::RGB& getRGB() const;
    const ColorSensor::HSV& getHSV() const;

private:
    ColorSensor color_sensor;
    ColorSensor::RGB rgb;
    ColorSensor::HSV hsv;
};

#endif // COLOR_SENSOR_WRAPPER_
