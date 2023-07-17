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

void HalfEdgeMeshRenderer::Render() const
{
    Renderer::Get().Draw( *_mesh->_vao, *GetComponent<Material>() );
}

void HalfEdgeMeshRenderer::RenderShadowDepth() const
{
    if (_cast_shadow)
    {
        Renderer::Get().DrawShadowDepth( *_mesh->_vao );
    }
}