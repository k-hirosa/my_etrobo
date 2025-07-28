#ifndef AVOID_LINE_OBSTACLE_SCENE_MANAGER_H
#define AVOID_LINE_OBSTACLE_SCENE_MANAGER_H

#include "SceneManager.h"
#include "AvoidLineObstacle.h"
#include <memory>
#include <string>

class AvoidLineObstacleSceneManager : public SceneManager {
public:
    explicit AvoidLineObstacleSceneManager(const std::string& config_path);
    ~AvoidLineObstacleSceneManager();

    bool runScene() override;

private:
    int sceneCounter_;
    std::unique_ptr<AvoidLineObstacle> alo_;
};

#endif // AVOID_LINE_OBSTACLE_SCENE_MANAGER_H
