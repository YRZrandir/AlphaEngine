#include "Scalpel.h"
#include "input/Input.h"
#include "util/Camera.h"
#include "util/Shader.h"
#include "model/HalfEdgeMesh.h"
#include "material/Material.h"

PBD::Scalpel::Scalpel()
{
    _points[0] = glm::vec3( 0, 0, -0.2 );
    _points[1] = glm::vec3( 0, 0, 0.2 );
    _last_trans = mTransform;
    _vbo = std::make_unique<VertexBuffer>( nullptr, sizeof( glm::vec3 ) * 2, GL_DYNAMIC_DRAW );
    _vbo->UpdateData( static_cast<void*>(_points), sizeof( glm::vec3 ) * 2 );
    _vao = std::make_unique<VertexArray>();
    VertexBufferLayout layout;
    layout.Push( GL_FLOAT, 3, 0 );
    _vbo->SetLayout( layout );
    _vao->AddBuffer( *_vbo );

    _model = std::make_unique<HalfEdgeMesh>( "res/models/scalpel.obj" );
    _model->Normalize();

    mTransform.SetScale( glm::vec3( 1.0f ) );
    MaterialInfos matinfos;
    matinfos.diffuseColor = glm::vec3( 0.98, 0.97, 0.95 );
    matinfos.metallic = 0.99f;
    matinfos.roughness = 0.01f;
    auto mat = std::make_unique<Material>( "", "material", matinfos );
    _model->SetMainMaterial( std::move( mat ) );
}

glm::vec3 PBD::Scalpel::GetStartPos() const
{
    return glm::vec3( mTransform.GetModelMat() * glm::vec4( _points[0], 1.0f ) );
}

glm::vec3 PBD::Scalpel::GetEndPos() const
{
    return glm::vec3( mTransform.GetModelMat() * glm::vec4( _points[1], 1.0f ) );
}

glm::vec3 PBD::Scalpel::GetLastStartPos() const
{
    return glm::vec3( _last_trans.GetModelMat() * glm::vec4( _points[0], 1.0f ) );
}

glm::vec3 PBD::Scalpel::GetLastEndPos() const
{
    return glm::vec3( _last_trans.GetModelMat() * glm::vec4( _points[1], 1.0f ) );
}

std::pair<TriPoint, TriPoint> PBD::Scalpel::GetSweepFace() const
{
    return std::make_pair( TriPoint( GetLastStartPos(), GetStartPos(), GetLastEndPos() ),
        TriPoint( GetStartPos(), GetEndPos(), GetLastEndPos() ) );
}

void PBD::Scalpel::Update()
{
    _last_trans = mTransform;
    //constexpr float SPEED = 0.03f;
    //if (Input::IsKeyHeld( Input::Key::RIGHT ))
    //{
    //    mTransform.Translate( -mTransform.Left() * SPEED );
    //}
    //if (Input::IsKeyHeld( Input::Key::LEFT ))
    //{
    //    mTransform.Translate( mTransform.Left() * SPEED );
    //}
    //if (Input::IsKeyHeld( Input::Key::UP ))
    //{
    //    mTransform.Translate( mTransform.Forward() * SPEED );
    //}
    //if (Input::IsKeyHeld( Input::Key::DOWN ))
    //{
    //    mTransform.Translate( -mTransform.Forward() * SPEED );
    //}
}

void PBD::Scalpel::Draw()
{
    auto shader = Shader::Find( "line" );
    shader->use();
    _vao->Bind();
    shader->setMat( "uModelMat", mTransform.GetModelMat() );
    shader->setMat( "uViewMat", Camera::current->GetViewMatrix() );
    shader->setMat( "uProjectionMat", Camera::current->GetProjectionMatrix() );
    shader->setVec( "uColor", glm::vec3( 1, 0.2f, 0.2f ) );
    glDrawArrays( GL_LINES, 0, 2 );

    _model->mTransform = mTransform;
    _model->mTransform.SetScale( glm::vec3( 0.3f ) );
    _model->Draw();
}
