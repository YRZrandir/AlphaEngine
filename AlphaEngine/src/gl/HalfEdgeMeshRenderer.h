#pragma once
#include "SceneObjectRenderer.h"
#include "model/HalfEdgeMesh.h"
#include "material/Material.h"

class HalfEdgeMeshRenderer : public SceneObjectRenderer
{
public:
    HalfEdgeMeshRenderer();
    virtual void Render() const override;
    virtual void RenderShadowDepth() const override;
    void SetCastShadow( bool cast_shadow ) { _cast_shadow = cast_shadow; }

protected:
    virtual void Start() override;
    HalfEdgeMesh* _mesh{ nullptr };
    bool _cast_shadow{ true };
};