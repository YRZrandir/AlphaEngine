#pragma once
#include "ECS.h"
class RenderingSystem :
    public System
{
public:
    virtual void Start();
    virtual void Update();
    virtual void OnPreRender();
    virtual void OnRender();
    virtual void OnPostRender();
};

