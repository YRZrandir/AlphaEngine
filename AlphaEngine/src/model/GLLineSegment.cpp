#include "GLLineSegment.h"
#include "../util/Shader.h"
#include "../util/Camera.h"

GLLineSegment::GLLineSegment()
{
    _vao = std::make_unique<VertexArray>();
    _vbo = std::make_unique<VertexBuffer>( nullptr, 0 );
    VertexBufferLayout layout;
    layout.Push( GL_FLOAT, 3, offsetof( Point, p ) );
    layout.Push( GL_FLOAT, 3, offsetof( Point, color ) );
    _vbo->SetLayout( layout );
    _vao->AddBuffer( *_vbo );
    this->mLayer = -1;
}

void GLLineSegment::AddPoint( glm::vec3 p, glm::vec3 color )
{
    _mem.push_back( { p, color } );
}

void GLLineSegment::ReservePoint( int count )
{
    _mem.reserve( count );
}

void GLLineSegment::Clear()
{
    _mem.clear();
}

void GLLineSegment::UpdateMem()
{
    _vbo->UpdateData( _mem.data(), _mem.size() * sizeof( _mem[0] ) );
}

void GLLineSegment::Update()
{
}

void GLLineSegment::Draw()
{
    _vao->Bind();
    auto shader = Shader::Find( "GLLineSegments" );
    shader->use();
    shader->setMat( "uModelMat", mTransform.GetModelMat() );
    shader->setMat( "uViewMat", Camera::current->GetViewMatrix() );
    shader->setMat( "uProjectionMat", Camera::current->GetProjectionMatrix() );
    shader->setVec( "uCameraPos", Camera::current->mTransform.GetPosition() );
    glLineWidth( 2.0f );
    glEnable( GL_PROGRAM_POINT_SIZE );
    glDrawArrays( GL_LINES, 0, _mem.size() );
    glLineWidth( 1.0f );
}
