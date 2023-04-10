#include "Camera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "../vr/OpenVRWrapper.h"
#include "../util/util.h"
#include "../input/Input.h"
#include "../vr/OpenVRWrapper.h"
#include <imgui/imgui.h>
#include <glad/glad.h>

Camera* Camera::main = nullptr;
Camera* Camera::current = nullptr;

Camera::Camera( const std::string& name )
    :SceneObject( name )
{
    _fov = glm::radians( 60.0f );
    _far_plane = 1000.0f;
    _near_plane = 0.001f;
    _viewport_corner = glm::vec2( 0, 0 );
    float vp[4];
    glGetFloati_v( GL_VIEWPORT, 0, vp );
    _viewport_size = glm::vec2( vp[2], vp[3] );
    _yaw = 0.f;
    _pitch = 0.f;
}

glm::mat4 Camera::GetViewMatrix() const
{
    return glm::lookAt( mTransform.GetPosition(), mTransform.GetPosition() + mTransform.Forward(), mTransform.Up() );
}

glm::mat4 Camera::GetProjectionMatrix() const
{
    return glm::perspectiveFov( _fov, _viewport_size.x, _viewport_size.y, _near_plane, _far_plane );
}

glm::vec3 Camera::WorldToScreenPoint( glm::vec3 p ) const
{
    glm::vec4 pos = glm::vec4( GetProjectionMatrix() * GetViewMatrix() * glm::vec4( p, 1.0f ) );
    pos /= pos.w;
    pos.x = (pos.x + 1.0f) * 0.5f * _viewport_size[0];
    pos.y = (pos.y + 1.0f) * 0.5f * _viewport_size[1];
    pos.y = _viewport_size.y - pos.y;
    pos.x = (int)pos.x;
    pos.y = (int)pos.y;
    return glm::vec3( pos );
}

glm::vec3 Camera::WorldToViewportPoint( glm::vec3 p ) const
{
    glm::vec4 pos = glm::vec4( GetViewMatrix() * glm::vec4( p, 1.0f ) );
    pos /= pos.w;
    return glm::vec3( pos );
}

Ray Camera::ScreenPointToRay( glm::vec2 p/*屏幕空间位置*/ ) const
{
    glm::vec3 ori = mTransform.GetPosition();   //摄像机位置
    glm::vec3 cursor_pos = glm::vec3( p.x, p.y, 0.f );
    glm::ivec4 viewport = glm::ivec4( _viewport_corner.x, _viewport_corner.y, _viewport_size.x, _viewport_size.y );
    glm::vec3 end = glm::unProject( cursor_pos, GetViewMatrix(), GetProjectionMatrix(), viewport );
    return Ray{ ori/*起点*/, glm::normalize( end - ori ) /*方向*/ };
}

SurroundCamera::SurroundCamera( const std::string& name, glm::vec3 pos, float yaw, float pitch )
    :Camera( name )
{
    _yaw = yaw;
    _pitch = pitch;
    mTransform.SetPos( pos );
    mTransform.SetYawPitchRow( glm::radians( _yaw ), glm::radians( _pitch ), 0.f );
}

void SurroundCamera::Update()
{
    float vp[4];
    glGetFloati_v( GL_VIEWPORT, 0, vp );
    _viewport_size = glm::vec2( vp[2], vp[3] );
    ProcessMouseScroll( Input::GetMouseScrollDelta() );
    if (Input::IsMouseButtonHeld( Input::MouseButton::Right ) && Input::IsKeyReleased( Input::Key::LEFT_ALT ))
    {
        glm::vec2 cursordelta = Input::GetMousePosDelta();
        ProcessMouseMovement( cursordelta.x, cursordelta.y );
    }
    float x = cos( glm::radians( _yaw ) ) * cos( glm::radians( _pitch ) );
    float y = sin( glm::radians( _pitch ) );
    float z = sin( glm::radians( _yaw ) ) * cos( glm::radians( _pitch ) );
    mTransform.SetPos( glm::vec3( x, y, z ) * _zoom );
    mTransform.LookAt( glm::vec3( 0.f ) );
}

void SurroundCamera::Draw()
{
}

void SurroundCamera::ProcessMouseMovement( float xoffset, float yoffset )
{
    xoffset *= _mouse_speed;
    yoffset *= _mouse_speed;
    _yaw += xoffset;
    _pitch += yoffset;
    _pitch = glm::clamp( _pitch, -89.0f, 89.0f );
    if (_yaw > 360.0f)
        _yaw -= 360.0f;
    if (_yaw < -360.0f)
        _yaw += 360.0f;
}

void SurroundCamera::ProcessMouseScroll( float yoffset )
{
    _zoom -= yoffset * _mouse_speed * 0.01f;
    _zoom = glm::clamp( _zoom, 0.1f, 5.0f );
}

VRCamera::VRCamera( const std::string& name, int eye )
    : Camera( name ), mEye( eye )
{
}

glm::mat4 VRCamera::GetViewMatrix() const
{
    return mViewMat;
}

glm::mat4 VRCamera::GetProjectionMatrix() const
{
    return mProjMat;
}

void VRCamera::Update()
{
    float vp[4];
    glGetFloati_v( GL_VIEWPORT, 0, vp );
    _viewport_size = glm::vec2( vp[2], vp[3] );
    if (!OpenVRWrapper::UseVr())
    {
        return;
    }
    if (mEye == 0)  //left
    {
        mViewMat = OpenVRWrapper::GetLeftEyePose() * OpenVRWrapper::GetHMDMat();
        mProjMat = OpenVRWrapper::GetLeftProjection();
        mTransform.SetPos( OpenVRWrapper::GetHMDPose() );
    }
    else
    {
        mViewMat = OpenVRWrapper::GetRightEyePose() * OpenVRWrapper::GetHMDMat();
        mProjMat = OpenVRWrapper::GetRightProjection();
        mTransform.SetPos( OpenVRWrapper::GetHMDPose() );
    }
}

void VRCamera::Draw()
{

}

FreeCamera::FreeCamera( const std::string& name, glm::vec3 pos, float speed )
    :Camera( name ), mSpeed( speed )
{
    mTransform.SetPos( pos );
    mTransform.SetYawPitchRow( 0, 0, 0.f );
}

void FreeCamera::Update()
{
    float vp[4];
    glGetFloati_v( GL_VIEWPORT, 0, vp );
    _viewport_size = glm::vec2( vp[2], vp[3] );
    float speed_factor = 20.0f * ImGui::GetIO().DeltaTime;

    if (Input::IsKeyHeld( Input::Key::LEFT_SHIFT ))
    {
        speed_factor *= 3.0f;
    }
    if (Input::IsKeyHeld( Input::Key::W ))
    {
        mTransform.Translate( mTransform.Forward() * mSpeed * speed_factor );
    }
    if (Input::IsKeyHeld( Input::Key::A ))
    {
        mTransform.Translate( mTransform.Left() * mSpeed * speed_factor );
    }
    if (Input::IsKeyHeld( Input::Key::S ))
    {
        mTransform.Translate( -mTransform.Forward() * mSpeed * speed_factor );
    }
    if (Input::IsKeyHeld( Input::Key::D ))
    {
        mTransform.Translate( -mTransform.Left() * mSpeed * speed_factor );
    }

    if (Input::IsMouseButtonHeld( Input::MouseButton::Right ))
    {
        glm::vec2 cursordelta = Input::GetMousePosDelta();
        float xoffset = cursordelta.x;
        float yoffset = cursordelta.y;
        xoffset *= 0.2f;
        yoffset *= 0.2f;

        _yaw -= xoffset;
        _pitch += yoffset;
        _pitch = glm::clamp( _pitch, -89.0f, 89.0f );
        if (_yaw > 360.0f)
            _yaw -= 360.0f;
        if (_yaw < -360.0f)
            _yaw += 360.0f;
    }
    mTransform.SetYawPitchRow( glm::radians( _yaw ), glm::radians( _pitch ), 0.f );

    if (Input::IsKeyDown( Input::Key::F ))
    {
        std::cout << "Camera:\n" << mTransform.GetPosition() << '\n'
            << _yaw << ", " << _pitch << std::endl;
    }
}

void FreeCamera::Draw()
{
}

void FreeCamera::DrawGUI()
{
    ImGui::Begin( "camera" );
    glm::vec3 pos = mTransform.GetPosition();
    if (ImGui::DragFloat3( "pos", &pos.x ))
    {
        mTransform.SetPos( pos );
    }
    ImGui::DragFloat( "yaw", &_yaw );
    ImGui::DragFloat( "pitch", &_pitch );
    ImGui::End();
}
