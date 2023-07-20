#include "Renderer.h"
#include "util/Shader.h"

std::unique_ptr<Renderer> Renderer::_instance = nullptr;

Renderer& Renderer::Get()
{
    if (Renderer::_instance == nullptr)
        Renderer::_instance = std::unique_ptr<Renderer>( new Renderer() );  //do this or have make_unique as a friend function.
    return *Renderer::_instance;
}

Renderer::Renderer()
{
    _camera_ubo = std::make_unique<UniformBuffer>( sizeof( CameraUniformBlock ), static_cast<void*>(&_camera_ubo_info), GL_DYNAMIC_DRAW );
    _transform_ubo = std::make_unique<UniformBuffer>( sizeof( TransformUniformBlock ), static_cast<void*>(&_transform_ubo_info), GL_DYNAMIC_DRAW );
    _lights_ubo = std::make_unique<UniformBuffer>( sizeof( LightUniformBlock ), static_cast<void*>(&_lights_ubo_info), GL_DYNAMIC_DRAW );
    _camera_ubo->BindBase( 0 );
    _transform_ubo->BindBase( 1 );
    _lights_ubo->BindBase( 2 );
}

void Renderer::Draw( const VertexArray& vao, const Material& material )
{
    vao.Bind();
    auto shader = Shader::Find( material.mShader );
    shader->use();
    material.SetShaderUniforms( *shader );
    Light::SetAllLightUniforms( *shader );

    glDrawArrays( GL_TRIANGLES, 0, vao.GetVertexNumber() );
}

void Renderer::DrawShadowDepth( const VertexArray& vao, std::string shader )
{
    vao.Bind();
    auto s = Shader::Find( shader );
    s->use();
    glDrawArrays( GL_TRIANGLES, 0, vao.GetVertexNumber() );
}

void Renderer::UpdateEnvUniforms()
{
    _camera_ubo->SetData( sizeof( CameraUniformBlock ), static_cast<void*>(&_camera_ubo_info), GL_DYNAMIC_DRAW );
    _lights_ubo->SetData( sizeof( LightUniformBlock ), static_cast<void*>(&_lights_ubo_info), GL_DYNAMIC_DRAW );
    _transform_ubo->SetData( sizeof( TransformUniformBlock ), static_cast<void*>(&_transform_ubo_info), GL_DYNAMIC_DRAW );
}

void Renderer::UpdateTranformUniform()
{
    _transform_ubo->SetData( sizeof( TransformUniformBlock ), static_cast<void*>(&_transform_ubo_info), GL_DYNAMIC_DRAW );
}

void Renderer::UpdateCameraUniform()
{
    _camera_ubo->SetData( sizeof( CameraUniformBlock ), static_cast<void*>(&_camera_ubo_info), GL_DYNAMIC_DRAW );
}

void Renderer::AddDirLight( glm::vec3 dir, glm::vec3 ambient, glm::vec3 diffuse, float intensity, glm::mat4 light_space_mat )
{
    if (_lights_ubo_info.nb_dirlights >= _lights_ubo_info.dirlights.size())
        return;
    auto& light = _lights_ubo_info.dirlights[_lights_ubo_info.nb_dirlights];
    light.dir = glm::vec4( dir, 1.f );
    light.ambient = glm::vec4( ambient, 1.f );
    light.diffuse = glm::vec4( diffuse, 1.f );
    light.intensity = intensity;
    light.light_space_mat = light_space_mat;
    _lights_ubo_info.nb_dirlights++;
}

void Renderer::SetDirLight( unsigned id, glm::vec3 dir, glm::vec3 ambient, glm::vec3 diffuse, float intensity, glm::mat4 light_space_mat )
{
    if (id >= _lights_ubo_info.nb_dirlights)
        return;
    auto& light = _lights_ubo_info.dirlights[id];
    light.dir = glm::vec4( dir, 1.f );
    light.ambient = glm::vec4( ambient, 1.f );
    light.diffuse = glm::vec4( diffuse, 1.f );
    light.intensity = intensity;
    light.light_space_mat = light_space_mat;
}

void Renderer::ClearDirLights()
{
    _lights_ubo_info.nb_dirlights = 0;
}

void Renderer::AddPointLight( glm::vec3 pos, glm::vec3 color, float intensity, float att_const, float att_linear, float att_exp )
{
    if (_lights_ubo_info.nb_pointlights >= _lights_ubo_info.dirlights.size())
        return;
    auto& light = _lights_ubo_info.pointlights[_lights_ubo_info.nb_pointlights];
    light.pos = pos;
    light.color = color;
    light.intensity = intensity;
    light.att_const = att_const;
    light.att_linear = att_linear;
    light.att_exp = att_exp;
}

void Renderer::SetPointLight( unsigned id, glm::vec3 pos, glm::vec3 color, float intensity, float att_const, float att_linear, float att_exp )
{
    if (id >= _lights_ubo_info.nb_pointlights)
        return;
    auto& light = _lights_ubo_info.pointlights[id];
    light.pos = pos;
    light.color = color;
    light.intensity = intensity;
    light.att_const = att_const;
    light.att_linear = att_linear;
    light.att_exp = att_exp;
}

void Renderer::ClearPointLights()
{
    _lights_ubo_info.nb_pointlights = 0;
}

void Renderer::SetCamera( glm::vec3 pos, glm::mat4 view_mat, glm::mat4 proj_mat )
{
    _camera_ubo_info.pos = pos;
    _camera_ubo_info.view_mat = view_mat;
    _camera_ubo_info.proj_mat = proj_mat;
    _camera_ubo_info.projview_mat = proj_mat * view_mat;
}

void Renderer::SetTransform( glm::mat4 world_mat )
{
    _transform_ubo_info.world_mat = world_mat;
    _transform_ubo_info.normal_mat = glm::transpose( glm::inverse( _transform_ubo_info.world_mat ) );
}