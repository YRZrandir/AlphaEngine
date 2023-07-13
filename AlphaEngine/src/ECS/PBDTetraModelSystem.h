#pragma once
#include "ECS.h"
#include "PBD/PBDTetraModel.h"

class PBDTetraModelSystem : public System
{
    // Inherited via System
    virtual void Update() override;
};