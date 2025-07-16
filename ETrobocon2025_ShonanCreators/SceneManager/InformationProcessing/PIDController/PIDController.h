#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <deque>

class PIDController {
public:
    PIDController(double kp=0.0, double ki=0.0, double kd=0.0,
                  double output_min=-1e9, double output_max=1e9,
                  int integral_window=0, double dt=0.001);

    void reset();
    double process(double target, double actual);

private:
    double kp_, ki_, kd_;
    double output_min_, output_max_;
    int integral_window_;
    double dt_;

    std::deque<double> integral_buffer_;
    double integral_sum_;
    double prev_error_;
};

#endif // PID_CONTROLLER_H