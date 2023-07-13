#pragma once
#include "ECS.h"

class HalfEdgeMeshRendererSystem :
    public System
{
public:
    HalfEdgeMeshRendererSystem();
    // Inherited via System
    virtual void OnPreRender() override;
    virtual void OnRender() override;
};

