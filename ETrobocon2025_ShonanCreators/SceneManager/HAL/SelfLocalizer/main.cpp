#include "Encoder.h"
#include "Odometry.h"
#include "RobotPose.h"
#include "Clock.h"

int main() {
    Encoder left_encoder;
    Encoder right_encoder;

    RobotPose initial_pose(0.0, 0.0, 0.0);
    Odometry odom(0.05, &left_encoder, &right_encoder, initial_pose, &system_clock);

    for (int i = 0; i < 100; ++i) {
        left_encoder.update(1.0 + 0.01 * i);
        right_encoder.update(1.0 + 0.01 * i);

        odom.update(0.05, 0.05 * i);
        RobotPose pose = odom.getPose();

        std::cout << "x: " << pose.x << ", y: " << pose.y << ", theta: " << pose.theta << std::endl;

    }

    return 0;
}
