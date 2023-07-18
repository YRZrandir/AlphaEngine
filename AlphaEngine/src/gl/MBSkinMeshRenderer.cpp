#include "MBSkinMeshRenderer.h"
#include "PD/PDMetaballModelFC.h"
#include "PD/PDGPUMetaballModel.h"
#include "gl/ShaderStorageBuffer.h"

void MBSkinMeshRenderer::Start()
{
    _mesh = GetComponent<PDMetaballHalfEdgeMesh>();
    _pd_model = GetComponent<PD::PDMetaballModelFC>();
}

void MBSkinMeshRenderer::Render() const
{
    _pd_model->UpdateSkinInfoBuffer();
    _pd_model->GetVtxSkinBuffer().BindBufferBase( 0 );
    _pd_model->GetBallSkinBuffer().BindBufferBase( 1 );
    Renderer::Get().Draw( _mesh->GetVAO(), *GetComponent<Material>() );
}

void MBSkinMeshRenderer::RenderShadowDepth() const
{
    _pd_model->GetVtxSkinBuffer().BindBufferBase( 0 );
    _pd_model->GetBallSkinBuffer().BindBufferBase( 1 );
    Renderer::Get().DrawShadowDepth( _mesh->GetVAO(), "depth_pd" );
}

void MBGPUSkinMeshRenderer::Start()
{
    _mesh = GetComponent<PDMetaballHalfEdgeMesh>();
    _pd_model = GetComponent<PD::PDGPUMetaballModel>();
}

void MBGPUSkinMeshRenderer::Render() const
{
    _pd_model->GetVtxSkinBuffer().BindBufferBase( 0 );
    _pd_model->GetBallSkinBuffer().BindBufferBase( 1 );
    Renderer::Get().Draw( _mesh->GetVAO(), *GetComponent<Material>() );
}

void MBGPUSkinMeshRenderer::RenderShadowDepth() const
{
    _pd_model->GetVtxSkinBuffer().BindBufferBase( 0 );
    _pd_model->GetBallSkinBuffer().BindBufferBase( 1 );
    Renderer::Get().DrawShadowDepth( _mesh->GetVAO(), "depth_pd" );
}

