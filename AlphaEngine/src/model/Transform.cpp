#include "Transform.h"

Transform::Transform()
    :mPos( 0.f, 0.f, 0.f ),
    mScale( 1.f, 1.f, 1.f ),
    mRotation( glm::quat_identity<float, glm::defaultp>() )
{
    UpdateModelMat();
}

Transform::Transform( glm::vec3 pos, glm::vec3 scale, glm::quat rotation )
    :mPos( pos ), mScale( scale ), mRotation( rotation )
{
    UpdateModelMat();
}

void Transform::SetPos( glm::vec3 pos )
{
    mPos = pos;
    UpdateModelMat();
}

void Transform::SetScale( glm::vec3 scale )
{
    mScale = scale;
    UpdateModelMat();
}

void Transform::SetRotation( glm::quat rotation )
{
    mRotation = rotation;
    UpdateModelMat();
}

void Transform::SetRotation( glm::vec3 eulerRadians )
{
    SetRotation( glm::quat( eulerRadians ) );
}

void Transform::SetYawPitchRow( float yaw_rd, float pitch_rd, float row_rd )
{
    mRotation = glm::yawPitchRoll( yaw_rd, pitch_rd, row_rd );
    UpdateModelMat();
}

void Transform::Translate( glm::vec3 move )
{
    mPos += move;
    UpdateModelMat();
}

void Transform::Scale( glm::vec3 scale )
{
    mScale *= scale;
    UpdateModelMat();
}

void Transform::Scale( float scale )
{
    mScale *= scale;
    UpdateModelMat();
}

void Transform::Rotate( glm::quat rotation )
{
    mRotation = mRotation * rotation;
    UpdateModelMat();
}

void Transform::Rotate( glm::vec3 axis, float degree )
{
    Rotate( glm::quat( glm::radians( degree ), glm::normalize( axis ) ) );
}

void Transform::Rotate( float x_rad, float y_rad, float z_rad )
{
    Rotate( glm::vec3( x_rad, y_rad, z_rad ) );
}

void Transform::Rotate( glm::mat4 rotation )
{
    Rotate( glm::toQuat( rotation ) );
}

void Transform::Rotate( glm::vec3 eulerRadians )
{
    Rotate( glm::orientate4( eulerRadians ) );
}

void Transform::RotateAround( glm::vec3 point, glm::vec3 axis, float radian )
{
    glm::quat r( radian, axis );
    mPos = glm::rotate( r, mPos - point ) + point;
    Rotate( r );
}

void Transform::LookAt( glm::vec3 target )
{
    SetRotation( glm::quatLookAt( glm::normalize( -(target - mPos)/*negative-z-axis*/ ), glm::vec3( 0, 1, 0 ) ) );
}

void Transform::LookAt( glm::vec3 target, glm::vec3 worldup )
{
    SetRotation( glm::quatLookAt( glm::normalize( -(target - mPos)/*negative-z-axis*/ ), worldup ) );
}

void Transform::UpdateModelMat()
{
    glm::mat4 translationMat = glm::translate( glm::mat4( 1.0f ), mPos );
    glm::mat4 rotation = glm::mat4_cast( mRotation );
    glm::mat4 scaleMat = glm::scale( glm::mat4( 1.0f ), mScale );
    mModelMat = translationMat * rotation * scaleMat;
}

glm::mat4 Transform::GetModelMat() const
{
    return mModelMat;
}

glm::vec3 Transform::Forward() const
{
    return glm::normalize( glm::rotate( mRotation, glm::vec3( 0, 0, 1 ) ) );
}

glm::vec3 Transform::Up() const
{
    return glm::normalize( glm::rotate( mRotation, glm::vec3( 0, 1, 0 ) ) );
}

glm::vec3 Transform::Left() const
{
    return glm::normalize( glm::rotate( mRotation, glm::vec3( 1, 0, 0 ) ) );
}

glm::mat4 Transform::GetRotationMat() const
{
    return glm::toMat4( mRotation );
}

glm::quat Transform::GetRotation() const
{
    return mRotation;
}

glm::vec3 Transform::GetEulerAngleRadian() const
{
    return glm::eulerAngles( mRotation );
}

glm::mat4 Transform::GetPositionMat() const
{
    return glm::translate( glm::mat4( 1.0f ), mPos );
}

glm::vec3 Transform::GetPosition() const
{
    return mPos;
}

glm::mat4 Transform::GetScaleMat() const
{
    return glm::scale( glm::mat4( 1.0f ), mScale );
}

glm::vec3 Transform::GetScale() const
{
    return mScale;
}
