#include "MetaballCuttingModel.h"
#include <exception>
#include "../input/Input.h"
#include "../util/Camera.h"

namespace PBD
{
MetaballCuttingModel::MetaballCuttingModel( MetaballModel* metaball_model, HalfEdgeMesh* surface, ScalpelRect* scalpel )
    :_metaball_model( metaball_model ), _surface( surface ), _scalpel( scalpel )
{
    if (!_metaball_model || !_surface)
    {
        throw std::runtime_error( std::string( __FILE__ ) + "\n" + std::to_string( __LINE__ ) + "\tMetaball Model or Surface is nullptr!" );
    }
    _metaball_cutter = std::make_unique<MetaballCutter>( _metaball_model, _scalpel );
    _surface_cutter = std::make_unique<CutManager>( _surface, _scalpel );
    _scalpel->tester = HalfEdgeSurfaceTester( _surface );
}

void MetaballCuttingModel::Update()
{
    if (!_enabled)
        return;

    if (Input::IsMouseButtonReleased( Input::MouseButton::Left ))
    {
        glm::vec2 cursor = Input::GetMousePosition();
        cursor.y = Camera::current->_viewport_size.y - cursor.y;
        Ray ray = Camera::current->ScreenPointToRay( cursor );
        HalfEdgeSurfaceTester tester( _surface );

        IntersectionRec rec;
        int id;
        if (tester.RayIntersect( ray, &rec, 0.f, std::numeric_limits<float>::max(), &id ))
        {
            _scalpel->mTransform.SetPos( rec.p );
            _scalpel->mTransform.LookAt( rec.p - rec.normal );

        }
    }
    if (Input::IsMouseButtonHeld( Input::MouseButton::Left ))
    {
        std::optional<Rect> cut_rect = _scalpel->GetStepRect();
        if (cut_rect.has_value())
        {
            //scalpel move
            //ProcessOneStepCutRect(cut_rect.value());
        }
        else
        {
            //scalpel stay
        }
    }

    if (Input::IsMouseButtonUp( Input::MouseButton::Left ))
    {
        std::optional<Rect> rect = _scalpel->GetTotalRect();
        using clock = std::chrono::high_resolution_clock;
        if (rect.has_value())
        {
            auto tstart = clock::now();

            _metaball_cutter->ProcessOneStepCutRect( rect.value() );
            _metaball_cutter->ProcessTotalCutRect( rect.value() );

            _surface_cutter->CutMesh();
            //RecalcSurfaceX0();

            _surface->UpdatePosBuffer();
            _surface->UpdateAttrBuffer();

            _metaball_model->CreateSurfaceMapping2( rect.value() );

            auto t = std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - tstart);
            std::cout << "Cutting Time Cost: " << t << std::endl;
        }
    }

}

void MetaballCuttingModel::Draw()
{
    if (!_enabled)
        return;
}

void MetaballCuttingModel::SetEnable( bool value )
{
    _enabled = value;
}

bool MetaballCuttingModel::IsEnable() const
{
    return _enabled;
}

void MetaballCuttingModel::RecalcSurfaceX0()
{
    for (int i = 0; i < _surface->GetVertexNumber(); i++)
    {
        auto& v = _surface->_vertices[i];
        if (v.recal_x0)
        {
            for (auto& b : _metaball_model->BallList())
            {
                if (glm::distance( v.pos, b.x ) < b.r)
                {
                    v.rest_pos = b.x0 + glm::rotate( glm::inverse( b.q ), (v.pos - b.x) );
                    break;
                }
            }
        }
    }
}

}
