#pragma once
#include "ECS.h"

namespace PD
{
class SpatialHash;
}

class PhysicsSystem :
    public System
{
public:
    PhysicsSystem();
    virtual void Update();

protected:
    int _substeps{1};
    std::unique_ptr<PD::SpatialHash> _spatial_hash;
};

