#ifndef OBSTACLE_MAP_H
#define OBSTACLE_MAP_H

#include <vector>
#include <utility>

struct Obstacle {
    double x;
    double y;
};

class ObstacleMap {
public:
    ObstacleMap(double merge_threshold);

    void updateRobotPose(double x, double y, double theta);  // 自己位置の保存

    void addObstacle(double robot_x, double robot_y, double robot_theta,
                     double us_distance, double us_angle);

    void initialize(const std::vector<std::pair<double, double>>& known_obstacles);

    std::vector<std::pair<double, double>> getObstacles() const;

    std::vector<std::pair<double, double>> getObstaclesRelative() const;  //引数なし

private:
    std::vector<Obstacle> obstacles_;
    double merge_threshold_;

    double robot_x_ = 0.0;     //自己位置の内部保持
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;
};

#endif