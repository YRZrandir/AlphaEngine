#pragma once
#include "ECS/ECS.h"

class SceneObjectRenderer : public Component
{
public:
    SceneObjectRenderer() {}
    virtual ~SceneObjectRenderer() = default;
    virtual void Render() = 0;
    virtual void RenderShadowDepth() = 0;
};