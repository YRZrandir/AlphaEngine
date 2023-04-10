#include "HalfEdgeSurfaceTester.h"
#include "../model/HalfEdgeMesh.h"
#include "../util/Intersection.h"
#include "../util/util.h"

HalfEdgeSurfaceTester::HalfEdgeSurfaceTester( HalfEdgeMesh* surface )
    :_surface( surface )
{
}

void HalfEdgeSurfaceTester::SetSurface( HalfEdgeMesh* surface )
{
    _surface = surface;
}

bool HalfEdgeSurfaceTester::PointIsInSurface( glm::vec3 p ) const
{
    if (!_surface)
    {
        __debugbreak();
        return false;
    }

    Ray r( p, glm::normalize( glm::vec3( 0.123, 0.456, 0.789 ) ) );
    int count = 0;

    for (int i = 0, s = _surface->GetFaceNumber(); i < s; i++)
    {
        if (!_surface->TriangleIsValid( i ))
        {
            continue;
        }
        auto [ia, ib, ic] = _surface->GetFaceIndices( i );
        if (ia < 0)
        {
            continue;
        }
        glm::vec3 p0 = _surface->GetPosition( ia );
        glm::vec3 p1 = _surface->GetPosition( ib );
        glm::vec3 p2 = _surface->GetPosition( ic );
        IntersectionRec rec;
        if (RayTriIntersect( r, p0, p1, p2, &rec ))
        {
            count++;
        }
    }
    if (count % 2 == 0)
    {
        return false;
    }
    return true;
}

float HalfEdgeSurfaceTester::MinDistToSurface( glm::vec3 p, int* face_idx ) const
{
    if (!_surface)
    {
        __debugbreak();
        return 0.0f;
    }
    float min_dist = FLT_MAX;
    int min_i = -1;
    for (int i = 0, s = _surface->GetFaceNumber(); i < s; i++)
    {
        if (!_surface->TriangleIsValid( i ))
        {
            continue;
        }
        auto [ia, ib, ic] = _surface->GetFaceIndices( i );
        glm::vec3 p0 = _surface->GetPosition( ia );
        glm::vec3 p1 = _surface->GetPosition( ib );
        glm::vec3 p2 = _surface->GetPosition( ic );
        float dist = glm::MinDistToTriangle( p, p0, p1, p2 );
        if (dist < min_dist)
        {
            min_dist = dist;
            min_i = i;
        }
    }

    if (face_idx)
    {
        *face_idx = min_i;
    }
    return min_dist;
}

bool HalfEdgeSurfaceTester::RayIntersect( Ray ray, IntersectionRec* rec, float t_min, float t_max, int* id )
{
    if (!_surface)
    {
        __debugbreak();
        return false;
    }

    bool intersect = false;
    if (rec)
    {
        rec->t = t_max;
    }
    for (int i = 0, size = _surface->GetFaceNumber(); i < size; ++i)
    {
        if (!_surface->TriangleIsValid( i ))
        {
            continue;
        }
        auto [ia, ib, ic] = _surface->GetFaceIndices( i );
        const glm::vec3& p0 = _surface->GetPosition( ia );
        const glm::vec3& p1 = _surface->GetPosition( ib );
        const glm::vec3& p2 = _surface->GetPosition( ic );
        if (rec)
        {
            if (RayTriIntersect( ray, p0, p1, p2, rec, t_min, rec->t ))
            {
                intersect = true;
                *id = i;
            }
        }
        else
        {
            if (RayTriIntersect( ray, p0, p1, p2, nullptr, t_min, t_max ))
            {
                return true;
            }
        }
    }

    return intersect;
}

bool HalfEdgeSurfaceTester::LinesegIntersect( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec )
{
    float len = glm::distance( p0, p1 );
    if (glm::FloatEqual( len, 0.f ))
    {
        return false;
    }
    return RayIntersect( Ray( p0, glm::normalize( p1 - p0 ) ), rec, 0.f, len );
}

ProjectToSurfaceRet HalfEdgeSurfaceTester::ProjectToSurface( glm::vec3 p ) const
{
    float min_dist = FLT_MAX;
    int min_i = -1;
    int min_topo = -1;
    glm::vec3 min_proj;

    for (int i = 0, s = _surface->GetFaceNumber(); i < s; i++)
    {
        if (!_surface->TriangleIsValid( i ))
        {
            continue;
        }
        auto [ia, ib, ic] = _surface->GetFaceIndices( i );
        const glm::vec3 p0 = _surface->GetPosition( ia );
        const glm::vec3 p1 = _surface->GetPosition( ib );
        const glm::vec3 p2 = _surface->GetPosition( ic );
        int topo = -1;
        glm::vec3 proj;
        const float dist = glm::MinDistToTriangle( p, p0, p1, p2, &topo, &proj );
        if (dist < min_dist)
        {
            min_dist = dist;
            min_i = i;
            min_topo = topo;
            min_proj = proj;
        }
    }

    ProjectToSurfaceRet ret{};
    ret._dist = min_dist;
    ret._faceid = min_i;
    ret._pos = min_proj;
    return ret;
}
