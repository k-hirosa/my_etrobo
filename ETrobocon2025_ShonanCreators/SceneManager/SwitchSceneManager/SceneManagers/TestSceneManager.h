#ifndef TEST_SCENE_MANAGER
#define TEST_SCENE_MANAGER

#include "SceneManager.h"
#include <stdio.h>

class TestSceneManager : public SceneManager {
private:
    int sceneCounter;
public:
    TestSceneManager(/* args */);
    ~TestSceneManager();
    bool runScene();
};

TestSceneManager::TestSceneManager(/* args */) : sceneCounter(0)
{
}

TestSceneManager::~TestSceneManager()
{
}

//200サイクル待ったら次のシーンに行く
bool TestSceneManager::runScene() {
    printf("Test Scene\n");
    sceneCounter++;
    if(sceneCounter >= 200) {
        printf("go next");
        return true;
    }
    return false;
}

#endif