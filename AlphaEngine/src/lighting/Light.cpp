#include "Light.h"
#include "util/Scene.h"
#include "util/Shader.h"
#include "util/util.h"
#include "input/Input.h"


void Light::SetAllLightUniforms( Shader& shader )
{
    shader.use();
    DirLight::_shadow_depth_texarray->BindToUnit( 10 );
}

std::unique_ptr<Texture2DArray> DirLight::_shadow_depth_texarray = nullptr;
int DirLight::_instance_count = 0;

DirLight::DirLight( glm::vec3 ambient, glm::vec3 diffuse, float intensity )
    :ambient( ambient ), diffuse( diffuse ), _intensity( intensity )
{
    if (_shadow_depth_texarray == nullptr)
    {
        _shadow_depth_texarray = std::make_unique<Texture2DArray>( 1024, 1024, 1, 5 );
    }
    _shadow_depth_buffer = std::make_unique<ShadowFrameBuffer>( *_shadow_depth_texarray, _instance_count );
    _instance_count++;
}

void DirLight::SetShaderUniforms( Shader& shader )
{
    shader.use();
    shader.setInt( "shadow_depth_textures", 10 );
    _shadow_depth_buffer->GetDepthTexture()->BindToUnit( 10 );
}

void DirLight::DrawGUI()
{
    ImGui::DragFloat( "Intensity", &_intensity, 0.1f, 0.0f, 30.0f );
    ImGui::Checkbox( "Cast shadow", &_cast_shadow );
}

glm::mat4 DirLight::GetLightSpaceMat() const
{
    return GetLightSpaceProjMat() * GetLightSpaceViewMat();
}

glm::mat4 DirLight::GetLightSpaceViewMat() const
{
    glm::vec3 dir = GetComponent<Transform>()->Forward();
    glm::vec3 up = glm::vec3( 0, 1, 0 );
    if (std::abs( dir.x ) < 1e-4 && std::abs( dir.z ) < 1e-4)
    {
        up = glm::vec3( 0, 1, 0 );
    }
    return glm::lookAt( -dir, glm::vec3( 0, 0, 0 ), up );
}

glm::mat4 DirLight::GetLightSpaceProjMat() const
{
    return glm::ortho( -4.f, 4.f, -4.f, 4.f, -5.f, 5.f );
}

FrameBuffer* DirLight::GetShadowDepthBuffer()
{
    return _shadow_depth_buffer.get();
}

bool DirLight::CastShadow() const
{
    return _cast_shadow;
}

void DirLight::CastShadow( bool value )
{
    if (!_cast_shadow && value)
    {
        _shadow_depth_buffer = std::make_unique<FrameBuffer>( 2048, 2048, false, true, false );
    }
    _cast_shadow = value;
}

PointLight::PointLight( glm::vec3 color, float intensity, float att_const, float att_linear, float att_exp )
    :_color( color ), _intensity( intensity ), _att_const( att_const ), _att_linear( att_linear ), _att_exp( att_exp )
{
}

void PointLight::SetShaderUniforms( Shader& shader )
{
}

glm::mat4 PointLight::GetLightSpaceMat() const
{
    return GetLightSpaceProjMat() * GetLightSpaceViewMat();
}

glm::mat4 PointLight::GetLightSpaceViewMat() const
{
    return glm::mat4( 1.f );
}

glm::mat4 PointLight::GetLightSpaceProjMat() const
{
    return glm::mat4( 1.f );
}