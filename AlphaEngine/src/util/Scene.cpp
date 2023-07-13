#include "Scene.h"
#include "Camera.h"
#include "lighting/Light.h"


std::unique_ptr<Scene> Scene::active = nullptr;

Scene::Scene()
{
}

void Scene::Update()
{
    for (auto& child : _children)
    {
        child->Update();
    }
}

void Scene::Draw()
{
    SetUniformBuffers();

    //DrawShadowDepthBuffer();

    FrameBuffer::Unbind();

    Renderer::Get().SetCamera( Camera::current->mTransform.GetPosition(), Camera::current->GetViewMatrix(), Camera::current->GetProjectionMatrix() );
    Renderer::Get().UpdateCameraUniform();

    int max_layer = 0;
    for (int layer = 0; layer <= max_layer; layer++)
    {
        for (auto& child : _children)
        {
            max_layer = glm::max( max_layer, child->mLayer );
            if (child->mLayer == layer)
            {
                DrawChild( *child );
            }
        }
    }
}

void Scene::SetUniformBuffers()
{
    Renderer::Get().SetCamera( Camera::current->mTransform.GetPosition(), Camera::current->GetViewMatrix(), Camera::current->GetProjectionMatrix() );

    auto dir_lights = GetAllChildOfType<DirLight>();
    Renderer::Get().ClearDirLights();
    for (const auto& light : dir_lights)
    {
        Renderer::Get().AddDirLight( light->dir, light->ambient, light->diffuse, light->specular, light->intensity, light->GetLightSpaceMat() );
    }

    auto point_lights = GetAllChildOfType<PointLight>();
    for (const auto& light : point_lights)
    {
        Renderer::Get().AddPointLight( light->mTransform.GetPosition(), light->_color, light->_intensity, light->_att_const, light->_att_linear, light->_att_exp );
    }

    Renderer::Get().UpdateEnvUniforms();
}

void Scene::SetUniformBuffersForObject( const SceneObject& obj )
{
    Renderer::Get().SetTransform( obj.mTransform.GetModelMat() );
    Renderer::Get().UpdateTranformUniform();
}

void Scene::DrawShadowDepthBuffer()
{
    float vp[4];
    glGetFloati_v( GL_VIEWPORT, 0, vp );
    glm::vec2 viewport_size = glm::vec2( vp[2], vp[3] );

    auto dir_lights = GetAllChildOfType<DirLight>();
    for (int i = 0; i < dir_lights.size(); i++)
    {
        auto& light = dir_lights[i];

        light->GetShadowDepthBuffer()->Bind();
        light->GetShadowDepthBuffer()->Clear();

        Renderer::Get().SetCamera( light->mTransform.GetPosition(), light->GetLightSpaceViewMat(), light->GetLightSpaceProjMat() );
        Renderer::Get().UpdateCameraUniform();

        glViewport( 0, 0, light->GetShadowDepthBuffer()->Width(), light->GetShadowDepthBuffer()->Height() );
        for (auto& child : _children)
        {
            SetUniformBuffersForObject( *child );
            child->DrawShadowDepth();
        }

    }
    FrameBuffer::Unbind();
    glViewport( 0, 0, viewport_size.x, viewport_size.y );
}

void Scene::DrawChild( SceneObject& obj )
{
    SetUniformBuffersForObject( obj );
    obj.Draw();
}

bool Scene::RemoveChild( const std::string& name )
{
    for (auto it = _children.begin(); it != _children.end(); it++)
    {
        if ((*it)->mName == name)
        {
            _children.erase( it );
            return true;
        }
    }
    return false;
}