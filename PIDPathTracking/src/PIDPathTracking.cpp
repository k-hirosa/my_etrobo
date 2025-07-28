#include "PIDPathTracking.h"
#include <cmath>

PIDPathTracking::PIDPathTracking(): pid_(kp, ki, kd){}

PWM PIDPathTracking::computePWM(double goal_x_r, double goal_y_r) {
    double y_error = goal_y_r;
    double omega = pid_.process(0.0, y_error);

    double v_r = v + (omega * L / 2.0);
    double v_l = v - (omega * L / 2.0);

    v_r *= 100;
    v_l *= 100;
    int normalized_v_r = std::max(0, std::min(int(v_r), 100));
    int normalized_v_l = std::max(0, std::min(int(v_l), 100));
    PWM pwm;
    pwm.left = normalized_v_l;
    pwm.right = normalized_v_r;

    // 目標位置が原点の場合はモーターを停止
    if (goal_y_r == 0.0 && goal_x_r == 0.0) {
        pwm.left = 0;
        pwm.right = 0;
    }

    return pwm;
}

void PIDPathTracking::reset() {
    pid_.reset();
}
