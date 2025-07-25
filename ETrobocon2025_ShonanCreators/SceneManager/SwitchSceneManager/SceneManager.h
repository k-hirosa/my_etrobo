#ifndef SCENE_MANAGER_H
#define SCENE_MANAGER_H

class SceneManager{
    public:
        virtual ~SceneManager() {}
        virtual bool runScene() = 0; //シーン実行　難所が終了したかの判定を返す
};

#endif