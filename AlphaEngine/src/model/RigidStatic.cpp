#include "RigidStatic.h"
#include "../acceleration/BVH.h"
#include "ModelLoader.h"
#include "../util/Intersection.h"
#include "../input/Input.h"
#include "util/util.h"


RigidStatic::RigidStatic( std::string path )
{
    _surface = std::make_unique<HalfEdgeMesh>( path );
    _surface->_use_face_normal = true;
    _surface->UpdateNormal();
    _surface->UpdatePosBuffer();
    _surface->UpdateAttrBuffer();
    _trans_pos.resize( _surface->GetVertexNumber() );
    for (int i = 0; i < _surface->GetVertexNumber(); i++)
    {
        _trans_pos[i] = _surface->GetPosition( i );
    }

    _bvh = std::make_unique<BVHTree_HalfEdgeMesh>( _surface.get() );

    _surface->mRenderConfig._draw_face = true;
    _surface->mRenderConfig._draw_line = false;
    _surface->mRenderConfig._face_mattype = RenderConfig::MatType::Texture;
}

void RigidStatic::Update()
{
}

void RigidStatic::Draw()
{
    _surface->Draw();
}

HalfEdgeMesh& RigidStatic::Surface()
{
    return *_surface;
}

BVHTree& RigidStatic::BVH()
{
    return *_bvh;
}

bool RigidStatic::CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const
{
    rec->t = glm::distance( p0, p1 );
    bool ret = false;
    for (int i = 0; i < _surface->GetFaceNumber(); i++)
    {
        auto [ia, ib, ic] = _surface->GetFaceIndices( i );
        glm::vec3 pa = _trans_pos[ia];
        glm::vec3 pb = _trans_pos[ib];
        glm::vec3 pc = _trans_pos[ic];
        glm::vec3 normal = glm::cross( pb - pa, pc - pa );
        if (glm::dot( p1 - p0, normal ) < 0)
        {
            Ray r( p0, glm::normalize( p1 - p0 ) );

            if (RayTriIntersect( r, pa, pb, pc, rec, 0.0f, rec->t ))
            {
                ret = true;
            }
        }
    }

    return ret;
}

std::vector<PBD::CollisionConstraint> RigidStatic::CheckBall( glm::vec3 c, float r, int id ) const
{
    std::vector<PBD::CollisionConstraint> result;

    for (int i = 0; i < _surface->GetFaceNumber(); i++)
    {
        auto [ia, ib, ic] = _surface->GetFaceIndices( i );
        glm::vec3 pa = _trans_pos[ia];
        glm::vec3 pb = _trans_pos[ib];
        glm::vec3 pc = _trans_pos[ic];
        glm::vec3 normal = glm::normalize( glm::cross( pb - pa, pc - pa ) );
        float depth = 0.f;
        glm::vec3 p;
        if (BallTriIntersect( c, r, pa, pb, pc, &p, &depth ))
        {
            result.push_back( PBD::CollisionConstraint( id, p, normal, depth ) );
        }
    }

    return result;
}

bool RigidStatic::CheckInside( glm::vec3 p, glm::vec3* normal, float* depth, int* id ) const
{
    Ray r( p, glm::normalize( glm::vec3( 0.123, 0.456, 0.789 ) ) );
    int count = 0;
    bool inside = false;
    for (int i = 0, s = _surface->GetFaceNumber(); i < s; i++)
    {
        auto [ia, ib, ic] = _surface->GetFaceIndices( i );

        glm::vec3 p0 = _trans_pos[ia];
        glm::vec3 p1 = _trans_pos[ib];
        glm::vec3 p2 = _trans_pos[ic];
        IntersectionRec rec;
        if (RayTriIntersect( r, p0, p1, p2, &rec ))
        {
            count++;
        }
    }
    if (count % 2 == 0)
    {
        inside = false;
    }
    else
    {
        inside = true;

    }
    if (inside)
    {
        int face_id = -1;
        float min_dist = FLT_MAX;
        int min_i = -1;
        glm::vec3 min_normal;
        for (int i = 0, s = _surface->GetFaceNumber(); i < s; i++)
        {

            auto [ia, ib, ic] = _surface->GetFaceIndices( i );
            glm::vec3 p0 = _trans_pos[ia];
            glm::vec3 p1 = _trans_pos[ib];
            glm::vec3 p2 = _trans_pos[ic];
            float dist = glm::MinDistToTriangle( p, p0, p1, p2 );

            glm::vec3 normal = glm::normalize( glm::cross( p1 - p0, p2 - p0 ) );
            glm::vec3 proj = p + glm::dot( p0 - p, normal );
            glm::vec3 cm = glm::BarycentricPos( p0, p1, p2, proj );
            if (cm.x >= 0 && cm.y >= 0 && cm.z >= 0 && cm.x <= 1 && cm.y <= 1 && cm.z <= 1)
            {
                float dist = glm::dot( p0 - p, normal );
                if (dist >= 0)
                {
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        min_i = i;
                        min_normal = normal;
                    }
                }
            }
        }
        if (min_i == -1)
            return false;
        face_id = min_i;
        *normal = min_normal;
        *depth = min_dist;
        *id = face_id;
        return true;
    }
    return false;
}

void RigidStatic::UpdateTransPos()
{
    glm::mat4 T = mTransform.GetModelMat();
    _surface->mTransform = mTransform;
    for (int i = 0; i < _surface->GetVertexNumber(); i++)
    {
        _trans_pos[i] = T * glm::vec4( _surface->GetPosition( i ), 1.f );
    }
    _bvh->Refit();
}

RigidBall::RigidBall( glm::vec3 pos, float r )
    :_ori_pos( glm::vec3( 0.f ) ), _trans_pos( pos ), _r( r )
{
    _surface = std::make_unique<HalfEdgeMesh>( "res/models/ball960.obj" );
    //_surface->mRenderConfig._draw_face = false;
    //_surface->mRenderConfig._draw_line = true;
    mTransform.SetScale( glm::vec3( r * 2 ) );
    mTransform.SetPos( pos );
    UpdateTransPos();
}

void RigidBall::Update()
{
}

void RigidBall::Draw()
{
    _surface->Draw();
}

void RigidBall::UpdateTransPos()
{
    _surface->mTransform = mTransform;
    _trans_pos = glm::vec3( mTransform.GetModelMat() * glm::vec4( _ori_pos, 1.f ) );
}
