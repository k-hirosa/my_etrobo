#ifndef SWITCH_SCENE_MANAGER_H
#define SWITCH_SCENE_MANAGER_H

#include "SceneManager.h"

class SwitchSceneManager {
public:
    SwitchSceneManager();
    void runSceneManager(); //シーン実行関数
    void nextSceneManager(); //次のシーンへ移行
private:
    enum SceneManager {
        LineObstacle,
        DoubleLoop,
        SmartCarry,
        Goal
    };
    SceneManager currentSceneManager;
};

#endif