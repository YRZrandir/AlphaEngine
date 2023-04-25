#include "Scene.h"
#include "Camera.h"
#include "lighting/Light.h"

std::unique_ptr<Scene> Scene::active = nullptr;

Scene::Scene()
{
    _camera_ubo = std::make_unique<UniformBuffer>( sizeof( CameraUniformBlock ), static_cast<void*>(&_camera_ubo_info), GL_DYNAMIC_DRAW );
    _transform_ubo = std::make_unique<UniformBuffer>( sizeof( TransformUniformBlock ), static_cast<void*>(&_transform_ubo_info), GL_DYNAMIC_DRAW );
    _lights_ubo = std::make_unique<UniformBuffer>( sizeof( LightUniformBlock ), static_cast<void*>(&_lights_ubo_info), GL_DYNAMIC_DRAW );
    _camera_ubo->BindBase( 0 );
    _transform_ubo->BindBase( 1 );
    _lights_ubo->BindBase( 2 );
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

    _camera_ubo_info.viewproj_mat = Camera::current->GetProjectionMatrix() * Camera::current->GetViewMatrix();
    _camera_ubo->SetData( sizeof( CameraUniformBlock ), static_cast<void*>(&_camera_ubo_info), GL_DYNAMIC_DRAW );

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
    _camera_ubo_info.pos = Camera::current->mTransform.GetPosition();
    _camera_ubo_info.view_mat = Camera::current->GetViewMatrix();
    _camera_ubo_info.proj_mat = Camera::current->GetProjectionMatrix();
    _camera_ubo_info.viewproj_mat = _camera_ubo_info.proj_mat * _camera_ubo_info.view_mat;

    auto dir_lights = GetAllChildOfType<DirLight>();
    _lights_ubo_info.nb_dirlights = std::min( dir_lights.size(), _lights_ubo_info.dirlights.size() );
    for (int i = 0; i < _lights_ubo_info.nb_dirlights; i++)
    {
        _lights_ubo_info.dirlights[i].dir = dir_lights[i]->dir;
        _lights_ubo_info.dirlights[i].ambient = dir_lights[i]->ambient;
        _lights_ubo_info.dirlights[i].diffuse = dir_lights[i]->diffuse;
        _lights_ubo_info.dirlights[i].specular = dir_lights[i]->specular;
        _lights_ubo_info.dirlights[i].intensity = dir_lights[i]->intensity;
        _lights_ubo_info.dirlights[i].light_space_mat = dir_lights[i]->GetLightSpaceMat();

    }
    auto point_lights = GetAllChildOfType<PointLight>();
    _lights_ubo_info.nb_pointlights = std::min( point_lights.size(), _lights_ubo_info.pointlights.size() );
    for (int i = 0; i < _lights_ubo_info.nb_pointlights; i++)
    {
        _lights_ubo_info.pointlights[i].pos = point_lights[i]->mTransform.GetPosition();
        _lights_ubo_info.pointlights[i].color = point_lights[i]->_color;
        _lights_ubo_info.pointlights[i].intensity = point_lights[i]->_intensity;
        _lights_ubo_info.pointlights[i].att_const = point_lights[i]->_att_const;
        _lights_ubo_info.pointlights[i].att_linear = point_lights[i]->_att_linear;
        _lights_ubo_info.pointlights[i].att_exp = point_lights[i]->_att_exp;
    }

    _camera_ubo->SetData( sizeof( CameraUniformBlock ), static_cast<void*>(&_camera_ubo_info), GL_DYNAMIC_DRAW );
    _lights_ubo->SetData( sizeof( LightUniformBlock ), static_cast<void*>(&_lights_ubo_info), GL_DYNAMIC_DRAW );
}

void Scene::SetUniformBuffersForObject( const SceneObject& obj )
{
    _transform_ubo_info.world_mat = obj.mTransform.GetModelMat();
    _transform_ubo_info.normal_mat = glm::transpose( glm::inverse( _transform_ubo_info.world_mat ) );
    _transform_ubo->SetData( sizeof( TransformUniformBlock ), static_cast<void*>(&_transform_ubo_info), GL_DYNAMIC_DRAW );
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
        _camera_ubo_info.viewproj_mat = light->GetLightSpaceMat();
        _camera_ubo->SetData( sizeof( CameraUniformBlock ), static_cast<void*>(&_camera_ubo_info), GL_DYNAMIC_DRAW );

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