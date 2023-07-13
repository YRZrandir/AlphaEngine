#include "HalfEdgeMeshRenderer.h"
#include "util/Shader.h"

HalfEdgeMeshRenderer::HalfEdgeMeshRenderer()
{
}

void HalfEdgeMeshRenderer::Start()
{
    _mesh = GetComponent<HalfEdgeMesh>();
}

void HalfEdgeMeshRenderer::Render()
{
    std::cout << "Render" << std::endl;
    _mesh->_vao->Bind();
    auto shader = Shader::Find( _mesh->_material_main->mShader );
    shader->use();
    _mesh->_material_main->SetShaderUniforms( *shader );
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    glDrawArrays( GL_TRIANGLES, 0, _mesh->mRenderingVtxCount );
}

void HalfEdgeMeshRenderer::RenderShadowDepth()
{
    std::cout << "Calc depth for shadow" << std::endl;
    _mesh->_vao->Bind();
    auto shader = Shader::Find( "depth" );
    shader->use();
    _mesh->_material_main->SetShaderUniforms( *shader );
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    glDrawArrays( GL_TRIANGLES, 0, _mesh->mRenderingVtxCount );
}