#include "avoid_line_obstacle_scene_manager.h"
#include <stdio.h>

AvoidLineObstacleSceneManager::AvoidLineObstacleSceneManager(const std::string& config_path)
    : sceneCounter_(0), alo_(std::make_unique<AvoidLineObstacle>(config_path)) {}

AvoidLineObstacleSceneManager::~AvoidLineObstacleSceneManager() = default;

bool AvoidLineObstacleSceneManager::runScene() {
    printf("AvoidLineObstacle Scene\n");

    bool isGoalReached = alo->isGoalReached();
    alo->process();

    if (isGoalReached) {
        printf("Goal! end line obstacle, go next\n");
        return true;
    }

    return false;
}