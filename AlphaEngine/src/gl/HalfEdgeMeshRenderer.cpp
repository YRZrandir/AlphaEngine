#include "HalfEdgeMeshRenderer.h"
#include "util/Shader.h"
#include "gl/Renderer.h"

HalfEdgeMeshRenderer::HalfEdgeMeshRenderer()
{
}

void HalfEdgeMeshRenderer::Start()
{
    _mesh = GetComponent<HalfEdgeMesh>();
}

void HalfEdgeMeshRenderer::Render()
{
    Renderer::Get().SetTransform( GetComponent<Transform>()->GetModelMat() );
    Renderer::Get().UpdateTranformUniform();
    Renderer::Get().Draw( *_mesh->_vao, *GetComponent<Material>() );
}

void HalfEdgeMeshRenderer::RenderShadowDepth()
{
    Renderer::Get().DrawShadowDepth( *_mesh->_vao );
}