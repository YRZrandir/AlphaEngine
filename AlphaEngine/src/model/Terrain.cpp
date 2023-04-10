#include "Terrain.h"
#include "../util/util.h"
#include "../util/Shader.h"
#include "../util/Camera.h"

Terrain::Terrain( float width, float height, int res )
{
    for (size_t i = 0; i < res + 1; i++)
    {
        mPoints.push_back( glm::vec3( width / 2, 0.0f, (float)i / res * height - height / 2 ) );
        mPoints.push_back( glm::vec3( -width / 2, 0.0f, (float)i / res * height - height / 2 ) );
        mPoints.push_back( glm::vec3( (float)i / res * width - width / 2, 0.f, height / 2 ) );
        mPoints.push_back( glm::vec3( (float)i / res * width - width / 2, -0.f, -height / 2 ) );
    }
    mVBO = std::make_unique<VertexBuffer>( mPoints.data(), mPoints.size() * sizeof( glm::vec3 ) );
    VertexBufferLayout layout;
    layout.Push( GL_FLOAT, 3, 0 );
    mVBO->SetLayout( layout );
    mVAO = std::make_unique<VertexArray>();
    mVAO->AddBuffer( *mVBO );
}

Terrain::~Terrain()
{
}

void Terrain::Update()
{
}

void Terrain::Draw()
{
    glDisable( GL_CULL_FACE );
    mVAO->Bind();
    auto shader = Shader::Find( "line" );
    shader->use();
    shader->setMat( "uModelMat", mTransform.GetModelMat() );
    shader->setMat( "uViewMat", Camera::current->GetViewMatrix() );
    shader->setMat( "uProjectionMat", Camera::current->GetProjectionMatrix() );
    shader->setVec( "uColor", glm::LIGHTGREY );
    glDrawArrays( GL_LINES, 0, mPoints.size() );
}
