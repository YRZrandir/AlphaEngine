#include "CuttingTool.h"

#include "../util/Camera.h"
#include "../util/util.h"
#include "../util/Shader.h"
#include "../gl/IndexBuffer.h"
#include "../gl/VertexArray.h"
#include "../gl/VertexBuffer.h"
#include "../vr/OpenVRWrapper.h"
#include "../model/HalfEdgeMesh.h"
#include "../input/Input.h"

//STATIC Members
glm::vec3 CuttingTool::sPoints[4] = {
    {-0.5f, 0.f, -0.5f},
    { 0.5f, 0.f, -0.5f},
    { 0.5f, 0.f,  0.5f},
    {-0.5f, 0.f,  0.5f}
};
GLuint CuttingTool::sIndices[6] = { 0, 1, 2, 0, 2, 3 };
std::unique_ptr<VertexArray> CuttingTool::sVAO;
std::unique_ptr<VertexBuffer> CuttingTool::sVBO;
std::unique_ptr<IndexBuffer> CuttingTool::sIBO;
Shader* CuttingTool::sShader;
bool CuttingTool::sInitialized = false;

void CuttingTool::Init( Shader* shader )
{
    sVAO = std::make_unique<VertexArray>();
    sVBO = std::make_unique<VertexBuffer>( sPoints, (unsigned)(sizeof( glm::vec3 )) * 4u );
    sIBO = std::make_unique<IndexBuffer>( sIndices, 6u );
    VertexBufferLayout layout;
    layout.Push( GL_FLOAT, 3, 0 );
    sVBO->SetLayout( layout );
    sVAO->AddBuffer( *sVBO );
    sVAO->BindElementBuffer( *sIBO );
    sShader = shader;
    sInitialized = true;
}

CuttingTool::CuttingTool( glm::vec3 p, float _width, float _height )
    :width( _width ), height( _height )
{
    if (!sInitialized)
        __debugbreak();
    mTransform.SetPos( p );
    mTransform.SetScale( glm::vec3( width, 1, height ) );
    mInverseRotation = glm::inverse( mTransform.GetRotation() );
    mCuttingDirection = glm::normalize( glm::rotate( mTransform.GetRotation(), glm::vec3( 0.0f, 0.0f, -1.0f ) ) );
    UpdateWorldSpacePoints();
}

glm::vec3 CuttingTool::PlaneSpace( glm::vec3 value ) const
{
    return glm::rotate( mInverseRotation, value - mTransform.GetPosition() );
}

glm::vec3 CuttingTool::PlaneSpaceToWorldSpace( glm::vec3 value ) const
{
    return glm::rotate( mTransform.GetRotation(), value ) + mTransform.GetPosition();
}

bool CuttingTool::InPlane( glm::vec3 point ) const
{
    if (point.x < width * 0.5f && point.x > -width * 0.5f &&
        point.z < height * 0.5f && point.z > -height * 0.5f)
    {
        return true;
    }
    else
        return false;
}

glm::vec3 CuttingTool::GetCuttingDirection() const
{
    return mCuttingDirection;
}

glm::vec3 CuttingTool::LineIntersection( glm::vec3 ori, glm::vec3 end ) const
{
    float diff_x = end.x - ori.x;
    float diff_z = end.z - ori.z;
    if (glm::FloatEqual( diff_x, 0.0f ))
        diff_x = 0.001f * glm::sign( diff_x );
    if (glm::FloatEqual( diff_z, 0.0f ))
        diff_z = 0.001f * glm::sign( diff_z );
    float t[4] = {
        (width * 0.5f - ori.x) / diff_x,
        (-width * 0.5f - ori.x) / diff_x,
        (height * 0.5f - ori.z) / diff_z,
        (-height * 0.5f - ori.z) / diff_z
    };
    int idx = 0;
    for (size_t i = 0; i < _countof( t ); i++)
    {
        if ((t[i] >= 0 && t[i] < t[idx]) || t[idx] < 0)
            idx = static_cast<int>(i);
    }

    return ori + t[idx] * (end - ori);
}

void CuttingTool::UpdateWorldSpacePoints()
{
    mBoundingBox = AABB();
    for (int i = 0; i < 4; i++)
    {
        mWorldSpacePoints[i] = PlaneSpaceToWorldSpace( glm::vec3( sPoints[i].x * width, 0, sPoints[i].z * height ) );
        mBoundingBox.Expand( mWorldSpacePoints[i] );
    }
}

void CuttingTool::Update()
{
    //  if (OpenVRWrapper::UseVr())
    //  {
          //mTransform.SetPos(OpenVRWrapper::GetRightHandPos() * 4.0f);
          //mTransform.SetRotation(OpenVRWrapper::GetRightHandRotate());
    //  }
    mInverseRotation = glm::inverse( mTransform.GetRotation() );
    mCuttingDirection = glm::normalize( glm::rotate( mTransform.GetRotation(), glm::vec3( 0.0f, 0.0f, -1.0f ) ) );
    normal = glm::normalize( glm::rotate( mTransform.GetRotation(), glm::vec3( 0.f, 1.f, 0.f ) ) );
    UpdateWorldSpacePoints();
}

void CuttingTool::Draw()
{
    glDisable( GL_CULL_FACE );
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    sShader->use();
    sVAO->Bind();
    sIBO->Bind();
    sShader->setMat( "uModelMat", mTransform.GetModelMat() );
    sShader->setMat( "uViewMat", Camera::current->GetViewMatrix() );
    sShader->setMat( "uProjectionMat", Camera::current->GetProjectionMatrix() );
    sShader->setVec( "uColor", glm::vec4( glm::GREEN, 0.5f ) );
    glDrawElements( GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr );
}

glm::vec3 CuttingTool::GetCorner( int index ) const
{
    return mWorldSpacePoints[index];
}

const AABB& CuttingTool::GetAABB() const
{
    return mBoundingBox;
}

Scalpel::Scalpel()
{
    mModel = std::make_unique<HalfEdgeMesh>( "res/models/scalpel.obj" );
}

CuttingTool Scalpel::GetCuttingPlane()
{
    glm::vec3 pos = (mLastTransform.GetPosition() + mTransform.GetPosition()) * 0.5f;
    float width = glm::distance( mLastTransform.GetPosition(), mTransform.GetPosition() );
    float height = 0.4f;
    CuttingTool plane( pos, width, height );
    glm::vec3 up = glm::cross( mLastTransform.Forward(), mTransform.GetPosition() - mLastTransform.GetPosition() );
    plane.mTransform.LookAt( pos - mLastTransform.Forward(), up );
    return plane;
}

void Scalpel::Move( glm::vec3 p, glm::vec3 dir )
{
    if (mStatus == Scalpel::Status::FREE)
    {
        mTransform.SetPos( p );
        if (glm::FloatEqual( dir.x, 0.f ) && glm::FloatEqual( dir.z, 0.f ))
        {
            mTransform.LookAt( p + dir, glm::cross( dir, glm::vec3( 1, 0, 0 ) ) );
        }
        else
        {
            mTransform.LookAt( p + dir );
        }
    }
    else if (mStatus == Scalpel::Status::CUTTING)
    {
        glm::vec3 n = mLastTransform.Forward();
        glm::vec3 o = mLastTransform.GetPosition();
        float t = (glm::dot( n, o ) - glm::dot( n, p )) / (glm::length2( n ));
        glm::vec3 p_ = p + t * n;
        mTransform.SetPos( p_ );
    }
}

bool Scalpel::IsCutting() const
{
    return mStatus == Scalpel::Status::CUTTING;
}

bool Scalpel::IsFree() const
{
    return mStatus == Scalpel::Status::FREE;
}

void Scalpel::Update()
{
    using VRIO = OpenVRWrapper::IO;
    if (OpenVRWrapper::UseVr())
    {
        if (mStatus == Scalpel::Status::FREE)
        {
            if (VRIO::IsButtonPressed( VRIO::Button::A ))
            {
                mStatus = Scalpel::Status::CUTTING;
                mLastTransform = mTransform;
            }
        }
        else
        {
            if (VRIO::IsButtonUp( VRIO::Button::A ))
            {
                mStatus = Scalpel::Status::FREE;
            }
            if (VRIO::GetJoystickValue( VRIO::Hand::Right ).y > 0)
            {
                mLastTransform.Translate( mLastTransform.Forward() * 0.001f );
                mTransform.Translate( mTransform.Forward() * 0.001f );
            }
            if (VRIO::GetJoystickValue( VRIO::Hand::Right ).y < 0)
            {
                mLastTransform.Translate( -mLastTransform.Forward() * 0.001f );
                mTransform.Translate( -mTransform.Forward() * 0.001f );
            }
        }
    }
    else
    {
        if (mStatus == Scalpel::Status::FREE)
        {
            if (Input::IsMouseButtonDown( Input::MouseButton::Left ))
            {
                mStatus = Scalpel::Status::CUTTING;
                mLastTransform = mTransform;
            }
        }
        else
        {
            if (Input::IsMouseButtonUp( Input::MouseButton::Left ))
            {
                mStatus = Scalpel::Status::FREE;
            }
            if (Input::IsKeyHeld( Input::Key::W ))
            {
                mLastTransform.Translate( mLastTransform.Forward() * 0.001f );
                mTransform.Translate( mTransform.Forward() * 0.001f );
            }
            if (Input::IsKeyHeld( Input::Key::S ))
            {
                mLastTransform.Translate( -mLastTransform.Forward() * 0.001f );
                mTransform.Translate( -mTransform.Forward() * 0.001f );
            }
        }
    }
}

void Scalpel::Draw()
{
    mModel->mTransform = mTransform;
    mModel->mTransform.SetScale( glm::vec3( 0.2f ) );
    mModel->Draw();
}
