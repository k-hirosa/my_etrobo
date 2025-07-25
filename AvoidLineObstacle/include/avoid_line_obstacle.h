#ifndef AVOID_LINE_OBSTACLE_H
#define AVOID_LINE_OBSTACLE_H

#include "obstacle_map.h"
#include "local_goal_manager.h"
#include "obstacle_aware_pid_controller.h"
#include <memory>
#include <utility>
#include <string>

class AvoidLineObstacle {
public:
    explicit AvoidLineObstacle(const std::string& config_path);

    // 毎周期呼び出される処理（並進・回転速度を返す）
    std::pair<double, double> process(double x, double y, double theta, double dt);

    // ゴールに到達したかどうかの取得
    bool GetIsGoalReached() const;

private:
    std::unique_ptr<ObstacleMap> map_;
    std::unique_ptr<LocalGoalManager> goal_manager_;
    std::unique_ptr<ObstacleAwarePIDController> controller_;

    bool is_goal_reached_ = false;
};

#endif // AVOID_LINE_OBSTACLE_H
