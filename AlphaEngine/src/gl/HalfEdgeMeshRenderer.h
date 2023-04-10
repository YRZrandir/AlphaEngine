#pragma once
#include "SceneObjectRenderer.h"
#include "model/HalfEdgeMesh.h"
#include "material/Material.h"

class HalfEdgeMeshRenderer : public SceneObjectRenderer
{
public:
    HalfEdgeMeshRenderer( HalfEdgeMesh* mesh );
    virtual void Render() override;
    virtual void RenderShadowDepth() override;

protected:
    HalfEdgeMesh* _mesh;
};