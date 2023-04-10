#pragma once
#include "Scene.h"
#include <condition_variable>
#include <thread_pool.hpp>

class PBDScene :
    public Scene
{
public:
    PBDScene( bool mt );
    virtual ~PBDScene() = default;
    virtual void Update();
private:
    bool _mt = true;
};

