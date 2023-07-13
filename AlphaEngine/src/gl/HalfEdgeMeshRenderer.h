#pragma once
#include "SceneObjectRenderer.h"
#include "model/HalfEdgeMesh.h"
#include "material/Material.h"

class HalfEdgeMeshRenderer : public SceneObjectRenderer
{
public:
    HalfEdgeMeshRenderer();
    virtual void Render() override;
    virtual void RenderShadowDepth() override;

protected:
    virtual void Start() override;
    HalfEdgeMesh* _mesh{ nullptr };
};