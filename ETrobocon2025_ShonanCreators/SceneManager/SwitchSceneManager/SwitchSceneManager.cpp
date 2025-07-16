#include "SwitchSceneManager.h"

#include <stdio.h>

//各シーンマネージャーの読み込み
#include "TestSceneManager.h" //テスト用消してよい

//シーンマネージャーのインスタンス化
TestSceneManager testSceneManager;

SwitchSceneManager::SwitchSceneManager(): currentSceneManager(SceneManager::LineObstacle) {}

void SwitchSceneManager::runSceneManager() {
    switch (currentSceneManager)
    {
    case SceneManager::LineObstacle:
        //テスト用消してよい
        if(testSceneManager.runScene()) {
            nextSceneManager();
        }
        //ここまでテスト
        //nextSceneManager(); //この難所を飛ばす
        /* code */
        break;
    case SceneManager::DoubleLoop:
        /* code */
        printf("Go to DoubleLoop\n"); //テスト用消してよい
        //nextSceneManager(); //この難所を飛ばす
        break;
    case SceneManager::SmartCarry:
        /* code */
        break;
    case SceneManager::Goal:
        break;
    default:
        break;
    }
}

void SwitchSceneManager::nextSceneManager() {
    switch (currentSceneManager)
    {
    case SceneManager::LineObstacle:
        currentSceneManager = SceneManager::DoubleLoop;
        break;
    case SceneManager::DoubleLoop:
        currentSceneManager = SceneManager::SmartCarry;
        break;
    case SceneManager::SmartCarry:
        currentSceneManager = SceneManager::Goal;
        break;
    
    default:
        break;
    }
}