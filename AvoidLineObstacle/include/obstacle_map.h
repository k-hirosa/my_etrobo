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

    void addObstacle(double robot_x, double robot_y, double robot_theta,
                     double us_distance, double us_angle);

    void initialize(const std::vector<std::pair<double, double>>& known_obstacles);

    std::vector<std::pair<double, double>> getObstacles() const;

private:
    std::vector<Obstacle> obstacles_;
    double merge_threshold_;
};

#endif
