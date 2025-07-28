#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(double kp=0.0, double ki=0.0, double kd=0.0,
                  double output_min=-1e9, double output_max=1e9,
                  int integral_window=0, double dt=0.001);

    void reset();
    double process(double target, double actual);

private:
    double m_kp, m_ki, m_kd;
    double m_output_min, m_output_max;
    int m_integral_window;
    double m_dt;

    static const int kMaxWindow = 100;
    double m_integral_buffer[kMaxWindow];
    int m_buffer_size;
    int m_buffer_index;
    double m_integral_sum;
    double m_prev_error;
};

#endif // PID_CONTROLLER_H

