#pragma once
#include "Scene.h"
#include <condition_variable>

namespace PD
{
class SpatialHash;
}

class PBDScene :
    public Scene
{
public:
    PBDScene( bool mt );
    virtual ~PBDScene() = default;
    virtual void Update();
private:
    bool _mt = true;
    std::unique_ptr<PD::SpatialHash> _spatial_hash;
};

