#include "PIDController.h"
#include <algorithm>
#include <iostream>

PIDController::PIDController(double kp, double ki, double kd,
                             double output_min, double output_max,
                             int integral_window, double dt)
    : m_kp(kp), m_ki(ki), m_kd(kd),
      m_output_min(output_min), m_output_max(output_max),
      m_integral_window(integral_window), m_dt(dt),
      m_buffer_size(0), m_buffer_index(0),
      m_integral_sum(0.0), m_prev_error(0.0) {
    for (int i = 0; i < kMaxWindow; ++i) {
        m_integral_buffer[i] = 0.0;
    }
}

void PIDController::reset() {
    for (int i = 0; i < kMaxWindow; ++i) {
        m_integral_buffer[i] = 0.0;
    }
    m_buffer_size = 0;
    m_buffer_index = 0;
    m_integral_sum = 0.0;
    m_prev_error = 0.0;
}

double PIDController::process(double target, double actual) {
    double error = target - actual;
    double error_dt = error * m_dt;
    double derivative = 0.0;

    if (m_integral_window > 0) {
        if (m_buffer_size == m_integral_window) {
            m_integral_sum -= m_integral_buffer[m_buffer_index];
        } else {
            ++m_buffer_size;
        }
        m_integral_buffer[m_buffer_index] = error_dt;
        m_integral_sum += error_dt;

        m_buffer_index = (m_buffer_index + 1) % m_integral_window;
    } else {
        m_integral_sum += error_dt;
    }
    
    if(m_prev_error != 0.0)
    {
        derivative = (error - m_prev_error) / m_dt;
    }
    m_prev_error = error;

    double p_term = m_kp * error;
    double i_term = m_ki * m_integral_sum;
    double d_term = m_kd * derivative;

    double output = p_term + i_term + d_term;
    output = std::max(m_output_min, std::min(m_output_max, output));

    return output;
}

