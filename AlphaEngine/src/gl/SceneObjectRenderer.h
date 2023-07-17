#pragma once
#include "ECS/ECS.h"

class SceneObjectRenderer : public Component
{
public:
    SceneObjectRenderer() {}
    virtual ~SceneObjectRenderer() = default;
    virtual void Render() const = 0;
    virtual void RenderShadowDepth() const = 0;
};