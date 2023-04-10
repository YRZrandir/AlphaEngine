#pragma once

class SceneObjectRenderer
{
public:
    virtual void Render() = 0;
    virtual void RenderShadowDepth() = 0;
};