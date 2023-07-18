#include "RenderingSystem.h"
#include "gl/Renderer.h"
#include "gl/HalfEdgeMeshRenderer.h"
#include "gl/MBSkinMeshRenderer.h"

void RenderingSystem::Start()
{
}

void RenderingSystem::Update()
{
}

void RenderingSystem::OnPreRender()
{
    Renderer::Get().SetCamera( Camera::current->mTransform.GetPosition(), Camera::current->GetViewMatrix(), Camera::current->GetProjectionMatrix() );

    auto dir_lights = EntityManager::Get().GetAllComponentOfType<DirLight>();
    Renderer::Get().ClearDirLights();
    for (const auto& light : dir_lights)
    {
        Renderer::Get().AddDirLight( light->dir, light->ambient, light->diffuse, light->specular, light->intensity, light->GetLightSpaceMat() );
    }

    auto point_lights = EntityManager::Get().GetAllComponentOfType<PointLight>();
    Renderer::Get().ClearPointLights();
    for (const auto& light : point_lights)
    {
        Renderer::Get().AddPointLight( light->mTransform.GetPosition(), light->_color, light->_intensity, light->_att_const, light->_att_linear, light->_att_exp );
    }

    Renderer::Get().UpdateEnvUniforms();
}

void RenderingSystem::OnRender()
{
    auto all_halfedge_mesh_renderer = EntityManager::Get().GetComponentsRange<HalfEdgeMeshRenderer>();
    auto all_pdskin_mesh_renderer = EntityManager::Get().GetComponentsRange<MBSkinMeshRenderer>();

    //shadow depth textures
    float vp[4];
    glGetFloati_v( GL_VIEWPORT, 0, vp );
    glm::vec2 viewport_size = glm::vec2( vp[2], vp[3] );
    for (auto& light : EntityManager::Get().GetComponentsRange<DirLight>())
    {
        light.GetShadowDepthBuffer()->Bind();
        light.GetShadowDepthBuffer()->Clear();
        Renderer::Get().SetCamera( light.mTransform.GetPosition(), light.GetLightSpaceViewMat(), light.GetLightSpaceProjMat() );
        Renderer::Get().UpdateCameraUniform();
        glViewport( 0, 0, light.GetShadowDepthBuffer()->Width(), light.GetShadowDepthBuffer()->Height() );

        for (const auto& renderer : all_halfedge_mesh_renderer)
        {
            auto transform = renderer.GetComponent<Transform>();
            Renderer::Get().SetTransform( transform->GetModelMat() );
            Renderer::Get().UpdateTranformUniform();
            renderer.RenderShadowDepth();
        }

        Renderer::Get().SetTransform( glm::identity<glm::mat4>() );
        Renderer::Get().UpdateTranformUniform();
        for (const auto& renderer : all_pdskin_mesh_renderer)
        {
            renderer.RenderShadowDepth();
        }
    }

    //rendering
    FrameBuffer::Unbind();
    Renderer::Get().SetCamera( Camera::current->mTransform.GetPosition(), Camera::current->GetViewMatrix(), Camera::current->GetProjectionMatrix() );
    Renderer::Get().UpdateCameraUniform();
    glViewport( 0, 0, static_cast<GLsizei>(viewport_size.x), static_cast<GLsizei>(viewport_size.y) );

    for (const auto& renderer : all_halfedge_mesh_renderer)
    {
        auto transform = renderer.GetComponent<Transform>();
        Renderer::Get().SetTransform( transform->GetModelMat() );
        Renderer::Get().UpdateTranformUniform();
        renderer.Render();
    }

    Renderer::Get().SetTransform( glm::identity<glm::mat4>() );
    Renderer::Get().UpdateTranformUniform();
    for (const auto& renderer : all_pdskin_mesh_renderer)
    {
        renderer.Render();
    }
}

void RenderingSystem::OnPostRender()
{
}
