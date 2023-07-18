#pragma once
#include "ECS/ECS.h"
#include "SceneObjectRenderer.h"
#include "model/HalfEdgeMesh.h"
#include "PD/PDMetaballModelFC.h"

class PDMetaballHalfEdgeMesh;
namespace PD {
class PDMetaballModelFC;
}

class MBSkinMeshRenderer :
    public SceneObjectRenderer
{
public:
    virtual void Render() const override;
    virtual void RenderShadowDepth() const override;
    void SetCastShadow( bool cast_shadow ) { _cast_shadow = cast_shadow; }

protected:
    virtual void Start() override;

    bool _cast_shadow{ true };
    PDMetaballHalfEdgeMesh* _mesh{ nullptr };
    PD::PDMetaballModelFC* _pd_model{ nullptr };
};

