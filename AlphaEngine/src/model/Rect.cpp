#include "Rect.h"
#include "../util/Camera.h"
#include "../util/Shader.h"

const glm::vec3 Rect::kPoints[4] = { glm::vec3( 0.5, 0, 0.5 )/*RightUp*/, glm::vec3( -0.5f, 0, 0.5f )/*LeftUp*/,
glm::vec3( -0.5f, 0, -0.5f )/*LeftDown*/, glm::vec3( 0.5f, 0, -0.5f )/*RightDown*/ };

std::unique_ptr<VertexArray> Rect::kVAO = nullptr;
std::unique_ptr<VertexBuffer> Rect::kVBO = nullptr;

Rect::Rect( float width, float height )
    :_width( width ), _height( height )
{
    if (!kVAO)
    {
        kVAO = std::make_unique<VertexArray>();
        kVBO = std::make_unique<VertexBuffer>( kPoints, sizeof( kPoints[0] ) * 4 );
        VertexBufferLayout layout;
        layout.Push( GL_FLOAT, 3, 0 );
        kVBO->SetLayout( layout );
        kVAO->AddBuffer( *kVBO );
    }
    mTransform.SetScale( glm::vec3( _width, 1.f, _height ) );
}

void Rect::Update()
{
}

void Rect::Draw()
{
    kVAO->Bind();
    auto shader = Shader::Find( "line" );
    shader->use();
    shader->setMat( "uModelMat", mTransform.GetModelMat() );
    shader->setMat( "uViewMat", Camera::current->GetViewMatrix() );
    shader->setMat( "uProjectionMat", Camera::current->GetProjectionMatrix() );
    shader->setVec( "uColor", glm::vec3( 1, 0, 0 ) );
    glDrawArrays( GL_LINE_LOOP, 0, 4 );
}

glm::vec3 Rect::RightUp() const
{
    return mTransform.GetModelMat() * glm::vec4( kPoints[0], 1.0f );
}

glm::vec3 Rect::LeftUp() const
{
    return mTransform.GetModelMat() * glm::vec4( kPoints[1], 1.0f );
}

glm::vec3 Rect::LeftDown() const
{
    return mTransform.GetModelMat() * glm::vec4( kPoints[2], 1.0f );
}

glm::vec3 Rect::RightDown() const
{
    return mTransform.GetModelMat() * glm::vec4( kPoints[3], 1.0f );
}

glm::vec3 Rect::Center() const
{
    return mTransform.GetPosition();
}

glm::vec3 Rect::Normal() const
{
    return mTransform.Up();
}

bool Rect::PointInRect( glm::vec3 p ) const
{
    glm::vec3 proj = p - Normal() * glm::dot( Normal(), p - Center() );
    glm::vec3 LU = LeftUp();
    glm::vec3 u = glm::normalize( RightUp() - LU );
    glm::vec3 v = glm::normalize( LeftDown() - LU );
    glm::vec2 uv( glm::dot( proj - LU, u ), glm::dot( proj - LU, v ) );
    return uv.x >= 0 && uv.x <= Width() && uv.y >= 0 && uv.y <= Height();
}

void Rect::SetWidth( float width )
{
    _width = width;
    mTransform.SetScale( glm::vec3( _width, 1, _height ) );
}

float Rect::Width() const
{
    return _width;
}

void Rect::SetHeight( float height )
{
    _height = height;
    mTransform.SetScale( glm::vec3( _width, 1, _height ) );
}

float Rect::Height() const
{
    return _height;
}
