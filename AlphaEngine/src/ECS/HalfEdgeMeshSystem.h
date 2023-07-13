#pragma once
#include "ECS/ECS.h"
#include "model/HalfEdgeMesh.h"

class HalfEdgeMeshSystem :
    public System
{
public:
    HalfEdgeMeshSystem();
    // Inherited via System
    virtual void Update() override;
};

