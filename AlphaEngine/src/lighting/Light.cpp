#include "Light.h"
#include "util/Scene.h"
#include "util/Shader.h"
#include "util/util.h"
#include "input/Input.h"

Light::Light( const std::string& name )
    :SceneObject( name )
{
}

void Light::SetAllLightUniforms( Shader& shader )
{
    shader.use();
    auto lights = Scene::active->GetAllChildOfType<Light>();
    for (auto light : lights)
    {
        light->SetShaderUniforms( shader );
    }
}

DirLight::DirLight( const std::string& name, glm::vec3 dir, glm::vec3 ambient, glm::vec3 diffuse, float intensity, glm::vec3 specular )
    : Light( name ), dir( dir ), ambient( ambient ), diffuse( diffuse ), specular( specular ), intensity( intensity )
{
    mTransform.LookAt( dir );
    if (_cast_shadow)
    {
        _shadow_depth_buffer = std::make_unique<FrameBuffer>( 2048, 2048, false, true, false );
    }
}

void DirLight::SetShaderUniforms( Shader& shader )
{
    shader.use();
    shader.setVec( mName + ".dir", mTransform.Forward() );
    shader.setVec( mName + ".ambient", ambient );
    shader.setVec( mName + ".diffuse", diffuse );
    shader.setVec( mName + ".specular", specular );
    if (_cast_shadow)
    {
        shader.setInt( "uShadowDepthMap[" + std::to_string( id ) + "]", id + 10 );
        _shadow_depth_buffer->GetDepthTexture()->BindToUnit( id + 10 );
    }
}

void DirLight::Update()
{
    if (Input::IsKeyHeld( Input::Key::LEFT_ALT ) && Input::IsMouseButtonHeld( Input::MouseButton::Right ))
    {
        glm::vec2 cursordelta = Input::GetMousePosDelta();
        float xoffset = cursordelta.x * 0.2;
        float yoffset = cursordelta.y * 0.2;
        xoffset = glm::radians( xoffset );
        yoffset = glm::radians( yoffset );
        glm::vec3 r = {
            cos( xoffset ) * cos( yoffset ),
            sin( yoffset ),
            sin( xoffset ) * cos( yoffset )
        };
        mTransform.Rotate( xoffset, yoffset, 0 );
        glm::quat q( glm::vec3( 1, 0, 0 ), r );
        dir = glm::rotate( q, dir );
    }
}

void DirLight::Draw()
{
}

glm::mat4 DirLight::GetLightSpaceMat() const
{
    return GetLightSpaceProjMat() * GetLightSpaceViewMat();
}

glm::mat4 DirLight::GetLightSpaceViewMat() const
{
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

PointLight::PointLight( const std::string& name, glm::vec3 pos, glm::vec3 color, float intensity, float att_const, float att_linear, float att_exp )
    :Light( name ), _color( color ), _intensity( intensity ), _att_const( att_const ), _att_linear( att_linear ), _att_exp( att_exp )
{
    mTransform.SetPos( pos );
}

void PointLight::SetShaderUniforms( Shader& shader )
{
    shader.use();
    shader.setVec( mName + ".pos", mTransform.GetPosition() );
    shader.setVec( mName + ".color", _color );
    shader.setFloat( mName + ".intensity", _intensity );
    shader.setFloat( mName + ".att_const", _att_const );
    shader.setFloat( mName + ".att_linear", _att_linear );
    shader.setFloat( mName + ".att_exp", _att_exp );
}

void PointLight::Update()
{
    if (Input::IsKeyHeld( Input::Key::UP ) && Input::IsKeyHeld( Input::Key::LEFT_ALT ))
    {
        mTransform.Translate( glm::vec3( 0, 0.01, 0 ) );
    }
    if (Input::IsKeyHeld( Input::Key::DOWN ) && Input::IsKeyHeld( Input::Key::LEFT_ALT ))
    {
        mTransform.Translate( glm::vec3( 0, -0.01, 0 ) );
    }
    if (Input::IsKeyHeld( Input::Key::LEFT ) && Input::IsKeyHeld( Input::Key::LEFT_ALT ))
    {
        mTransform.Translate( glm::vec3( 0.01, 0, 0 ) );
    }
    if (Input::IsKeyHeld( Input::Key::RIGHT ) && Input::IsKeyHeld( Input::Key::LEFT_ALT ))
    {
        mTransform.Translate( glm::vec3( -0.01, 0, 0 ) );
    }
    if (Input::IsKeyHeld( Input::Key::COMMA ) && Input::IsKeyHeld( Input::Key::LEFT_ALT ))
    {
        mTransform.Translate( glm::vec3( 0, 0, 0.01 ) );
    }
    if (Input::IsKeyHeld( Input::Key::PERIOD ) && Input::IsKeyHeld( Input::Key::LEFT_ALT ))
    {
        mTransform.Translate( glm::vec3( 0, 0, -0.01 ) );
    }

    if (Input::IsKeyDown( Input::Key::F ))
    {
        std::cout << "Light:\n" << mTransform.GetPosition() << std::endl;
    }
}

void PointLight::Draw()
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