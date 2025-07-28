#include "PIDController.h"
#include <iostream>
#include <vector>

int main() {
    // PIDゲインと制限値を設定
    double kp = 0.0, ki = 0.0, kd = 1.0;
    double output_min = -10.0, output_max = 10.0;
    int integral_window = 3;
    double dt = 0.1;

    PIDController pid(kp, ki, kd, output_min, output_max, integral_window, dt);

    std::vector<double> targets = {1, 1, 1, 1, 1, 1};
    std::vector<double> actuals = {0, 1, 0, 0, 0, 0};

    std::cout << "Time\tTarget\tActual\tOutput\n";
    for (size_t i = 0; i < targets.size(); ++i) {
        double output = pid.process(targets[i], actuals[i]);
        std::cout << i*dt << "\t" << targets[i] << "\t" << actuals[i] << "\t" << output << "\n";
    }

    return 0;
}

