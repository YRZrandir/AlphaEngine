#include "ScalpelRect.h"
#include "../input/Input.h"
#include "../util/Camera.h"
#include "../util/util.h"
#include "../util/Shader.h"

PBD::ScalpelRect::ScalpelRect()
{
    _points[0] = glm::vec3( 0, 0, -0.1 );
    _points[1] = glm::vec3( 0, 0, 0.1 );
    _last_trans = mTransform;
    _vbo = std::make_unique<VertexBuffer>( nullptr, (unsigned int)(sizeof( glm::vec3 ) * 2), GL_DYNAMIC_DRAW );
    _vbo->UpdateData( static_cast<void*>(_points), sizeof( glm::vec3 ) * 2 );
    _vao = std::make_unique<VertexArray>();
    VertexBufferLayout layout;
    layout.Push( GL_FLOAT, 3, 0 );
    _vbo->SetLayout( layout );
    _vao->AddBuffer( *_vbo );

    _model = std::make_unique<HalfEdgeMesh>( "res/models/scalpel.obj" );
    _model->Normalize();
    _model->UpdateNormal();
    _model->UpdatePosBuffer();
}

void PBD::ScalpelRect::Update()
{
    if (!_enabled)
    {
        return;
    }
    if (_is_cutting)
    {
        UpdateWhenIsCutting();
    }
    else
    {
        UpdateWhenNotCutting();
    }

}

void PBD::ScalpelRect::Draw()
{
    if (!_enabled)
    {
        return;
    }
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

    if (_total_rect.has_value())
    {
        _total_rect->Draw();
    }
}

glm::vec3 PBD::ScalpelRect::GetPos0() const
{
    return glm::vec3( mTransform.GetModelMat() * glm::vec4( _points[0], 1.0f ) );
}

glm::vec3 PBD::ScalpelRect::GetPos1() const
{
    return glm::vec3( mTransform.GetModelMat() * glm::vec4( _points[1], 1.0f ) );
}

glm::vec3 PBD::ScalpelRect::GetLastPos0() const
{
    return glm::vec3( _last_trans.GetModelMat() * glm::vec4( _points[0], 1.0f ) );
}

glm::vec3 PBD::ScalpelRect::GetLastPos1() const
{
    return glm::vec3( _last_trans.GetModelMat() * glm::vec4( _points[1], 1.0f ) );
}

glm::vec3 PBD::ScalpelRect::GetStartPos0() const
{
    return glm::vec3( _start_trans.GetModelMat() * glm::vec4( _points[0], 1.0f ) );
}

glm::vec3 PBD::ScalpelRect::GetStartPos1() const
{
    return glm::vec3( _start_trans.GetModelMat() * glm::vec4( _points[1], 1.0f ) );
}

std::optional<Rect> PBD::ScalpelRect::GetStepRect() const
{
    return _step_rect;
}

std::optional<Rect> PBD::ScalpelRect::GetTotalRect() const
{
    return _total_rect;
}

bool PBD::ScalpelRect::IsCutting() const
{
    return _is_cutting;
}

void PBD::ScalpelRect::Enable()
{
    _enabled = true;
}

void PBD::ScalpelRect::Disable()
{
    _enabled = false;
}

void PBD::ScalpelRect::UpdateWhenIsCutting()
{
    if (Input::IsMouseButtonUp( Input::MouseButton::Left ))
    {
        _is_cutting = false;

    }
    if (Input::IsMouseButtonHeld( Input::MouseButton::Left ))
    {
        glm::vec2 mouse_move = Input::GetMousePosDelta();

        if (!glm::FloatEqual( mouse_move.x, 0.f, 1e-5 ))
        {
            glm::vec2 cursor = Input::GetMousePosition();
            cursor.y = Camera::current->_viewport_size.y - cursor.y;
            Ray ray = Camera::current->ScreenPointToRay( cursor );

            IntersectionRec rec;
            int id;
            if (tester.RayIntersect( ray, &rec, 0.f, std::numeric_limits<float>::max(), &id ))
            {
                glm::vec3 dist = mTransform.GetPosition() - rec.p;
                glm::vec3 proj = glm::dot( dist, glm::normalize( GetPos1() - GetPos0() ) ) * glm::normalize( GetPos1() - GetPos0() );
                mTransform.SetPos( rec.p + proj );
                //mTransform.LookAt( rec.p - rec.normal );
            }
            UpdateCuttingRect();
        }
        else
        {
            _step_rect.reset();
        }
    }
}

void PBD::ScalpelRect::UpdateWhenNotCutting()
{
    if (Input::IsMouseButtonDown( Input::MouseButton::Left ))
    {
        _is_cutting = true;
        _start_trans = mTransform;
        _last_trans = mTransform;
        _step_rect.reset();
        _total_rect.reset();
    }
}

void PBD::ScalpelRect::UpdateCuttingRect()
{
    glm::vec3 P0 = GetPos0();
    glm::vec3 LP0 = GetLastPos0();
    glm::vec3 P1 = GetPos1();
    glm::vec3 LP1 = GetLastPos1();
    glm::vec3 SP0 = GetStartPos0();
    glm::vec3 SP1 = GetStartPos1();

    Rect rect( glm::distance( P0, LP0 ), glm::distance( P0, P1 ) );
    rect.mTransform.SetPos( (P0 + LP0 + P1 + LP1) / 4.0f );
    rect.mTransform.LookAt( rect.mTransform.GetPosition() + glm::normalize( P0 - P1 ) ), glm::normalize( glm::cross( LP1 - P0, LP0 - P0 ) );
    _step_rect.emplace( rect );

    Rect rect_total( glm::distance( P0, SP0 ), glm::distance( P0, P1 ) );
    rect_total.mTransform.SetPos( (P0 + SP0 + P1 + SP1) / 4.0f );
    rect_total.mTransform.LookAt( rect_total.mTransform.GetPosition() + glm::normalize( SP1 - SP0 ),
        glm::normalize( glm::cross( SP1 - SP0, P0 - SP0 ) ) );
    _total_rect.emplace( rect_total );
}

//SceneObject* PBD::ScalpelRect::virtual_clone() const
//{
//    return nullptr;
//}
