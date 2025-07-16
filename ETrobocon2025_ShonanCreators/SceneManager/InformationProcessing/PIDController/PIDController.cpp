#include "PIDController.h"
#include <algorithm>  // for std::min, std::max
#include <iostream>   // for std::cout

PIDController::PIDController(double kp, double ki, double kd,
                             double output_min, double output_max,
                             int integral_window, double dt)
    : kp_(kp), ki_(ki), kd_(kd),
      output_min_(output_min), output_max_(output_max),
      integral_window_(integral_window), dt_(dt),
      integral_sum_(0.0), prev_error_(0.0) {}

void PIDController::reset() {
    integral_buffer_.clear();
    integral_sum_ = 0.0;
    prev_error_ = 0.0;
}

double PIDController::process(double target, double actual) {

    double error = target - actual;
    double error_dt = error * dt_;
    if (integral_window_ > 0) {
        if (integral_buffer_.size() == static_cast<size_t>(integral_window_)) {
            integral_sum_ -= integral_buffer_.front();
            integral_buffer_.pop_front();
        }
        integral_buffer_.push_back(error_dt);
        integral_sum_ += error_dt;
    } else {
        integral_sum_ += error_dt;
    }

    double derivative = 0.0;
    double d_term = 0.0;
    if (prev_error_ != 0.0) {
        derivative = (error - prev_error_) / dt_;
        d_term = kd_ * derivative;
    }
    prev_error_ = error;

    double p_term = kp_ * error;
    double i_term = ki_ * integral_sum_;
    double output = p_term + i_term + d_term;
    output = std::max(output_min_, std::min(output_max_, output));

    return output;
}