#include "Intersection.h"
#include <iostream>
#include <algorithm>
#include <vector>
#include "../util/util.h"
#include "../free_cutting/CuttingTool.h"

bool CutFaceIntersect( const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
    const glm::vec3& cutface_a, const glm::vec3& cutface_b, const glm::vec3& cutface_c,
    TriTriIntersectInfo* start, TriTriIntersectInfo* end )
{
    float dets[2][3] =
    {
        {
            Orient3D( cutface_a, cutface_b, cutface_c, a ),
            Orient3D( cutface_a, cutface_b, cutface_c, b ),
            Orient3D( cutface_a, cutface_b, cutface_c, c )
        },
        {
            Orient3D( a, b, c, cutface_a ),
            Orient3D( a, b, c, cutface_b ),
            Orient3D( a, b, c, cutface_c )
        }
    };
    if (dets[0][0] > 0.f && dets[0][1] > 0.f && dets[0][2] > 0.f ||
        dets[0][0] < 0.f && dets[0][1] < 0.f && dets[0][2] < 0.f ||
        dets[0][0] == 0.f && dets[0][1] == 0.f && dets[0][2] == 0.f)
    {
        return false;
    }
    if (dets[1][0] > 0.f && dets[1][1] > 0.f && dets[1][2] > 0.f ||
        dets[1][0] < 0.f && dets[1][1] < 0.f && dets[1][2] < 0.f ||
        dets[1][0] == 0.f && dets[1][1] == 0.f && dets[1][2] == 0.f)
    {
        return false;
    }
    bool det_is_zero[3] =
    {
        glm::FloatEqual( dets[0][0], 0.f ),
        glm::FloatEqual( dets[0][1], 0.f ),
        glm::FloatEqual( dets[0][2], 0.f )
    };
    int zero_count = std::count( std::cbegin( det_is_zero ), std::cend( det_is_zero ), true );

    if (zero_count == 0)
    {
        glm::vec3 tri[3] = { a, b, c };
        glm::vec3 cutface[3] = { cutface_a, cutface_b, cutface_c };
        int idx_offset[2] = { 0, 0 };
        for (int f = 0; f < 2; f++)
        {
            for (int i = 0; i < 3; i++)
            {
                int idx0 = (0 + i) % 3;
                int idx1 = (1 + i) % 3;
                int idx2 = (2 + i) % 3;
                if (dets[f][idx0] * dets[f][idx1] <= 0.0f && dets[f][idx0] * dets[f][idx2] <= 0.0f)
                {
                    idx_offset[f] = i;
                }
            }
        }
        glm::vec3 reordered_tri[3] = { tri[(0 + idx_offset[0]) % 3],
            tri[(1 + idx_offset[0]) % 3], tri[(2 + idx_offset[0]) % 3] };
        glm::vec3 reordered_cutface[3] = { cutface[(0 + idx_offset[1]) % 3],
            cutface[(1 + idx_offset[1]) % 3], cutface[(2 + idx_offset[1]) % 3] };
        glm::vec3 normal1 = glm::normalize( glm::cross( b - a, c - a ) );
        glm::vec3 normal2 = glm::normalize( glm::cross( cutface_b - cutface_a, cutface_c - cutface_a ) );

    }
}

bool TriTriIntersect( const glm::vec3& a1, const glm::vec3& b1, const glm::vec3& c1,
    const glm::vec3& a2, const glm::vec3& b2, const glm::vec3& c2,
    TriTriIntersectInfo* start, TriTriIntersectInfo* end, int main )
{
    float ori_a1 = glm::sign( Orient3D( a2, b2, c2, a1 ) );
    float ori_b1 = glm::sign( Orient3D( a2, b2, c2, b1 ) );
    float ori_c1 = glm::sign( Orient3D( a2, b2, c2, c1 ) );
    if ((ori_a1 == ori_b1 && ori_b1 == ori_c1) ||
        (ori_a1 == 0.f && ori_b1 == ori_c1) ||
        (ori_b1 == 0.f && ori_a1 == ori_c1) ||
        (ori_c1 == 0.f && ori_a1 == ori_b1))
    {
        return false;
    }

    float ori_a2 = glm::sign( Orient3D( a1, b1, c1, a2 ) );
    float ori_b2 = glm::sign( Orient3D( a1, b1, c1, b2 ) );
    float ori_c2 = glm::sign( Orient3D( a1, b1, c1, c2 ) );
    if ((ori_a2 == ori_b2 && ori_b2 == ori_c2) ||
        (ori_a2 == 0.f && ori_b2 == ori_c2) ||
        (ori_b2 == 0.f && ori_a2 == ori_c2) ||
        (ori_c2 == 0.f && ori_a2 == ori_b2))
    {
        return false;
    }

    glm::vec3 tri1[3] = { a1, b1, c1 };
    glm::vec3 tri2[3] = { a2, b2, c2 };
    int idx1[3] = { 0, 1, 2 };
    int idx2[3] = { 0, 1, 2 };
    bool swap1 = false;
    bool swap2 = false;
    if (ori_a1 * ori_b1 <= 0.f && ori_a1 * ori_c1 <= 0.f && ori_a1 != 0.f)
    {
        swap1 = ori_a1 < 0;
    }
    else if (ori_b1 * ori_a1 <= 0.f && ori_b1 * ori_c1 <= 0.f && ori_b1 != 0.f)
    {
        idx1[0] = 1; idx1[1] = 2; idx1[2] = 0;
        swap1 = ori_b1 < 0;
    }
    else
    {
        idx1[0] = 2; idx1[1] = 0; idx1[2] = 1;
        swap1 = ori_c1 < 0;
    }
    if (ori_a2 * ori_b2 <= 0.f && ori_a2 * ori_c2 <= 0.f && ori_a2 != 0.f)
    {
        swap2 = ori_a2 < 0;
    }
    else if (ori_b2 * ori_a2 <= 0.f && ori_b2 * ori_c2 <= 0.f && ori_c2 != 0.f)
    {
        idx2[0] = 1; idx2[1] = 2; idx2[2] = 0;
        swap2 = ori_b2 < 0;
    }
    else
    {
        idx2[0] = 2; idx2[1] = 0; idx2[2] = 1;
        swap2 = ori_c2 < 0;
    }

    if (swap1)
    {
        std::swap( idx2[1], idx2[2] );
    }
    if (swap2)
    {
        std::swap( idx1[1], idx1[2] );
    }

    glm::vec3 p1 = tri1[idx1[0]];
    glm::vec3 q1 = tri1[idx1[1]];
    glm::vec3 r1 = tri1[idx1[2]];
    glm::vec3 p2 = tri2[idx2[0]];
    glm::vec3 q2 = tri2[idx2[1]];
    glm::vec3 r2 = tri2[idx2[2]];
    glm::vec3 normal1 = glm::normalize( glm::cross( q1 - p1, r1 - p1 ) );
    glm::vec3 normal2 = glm::normalize( glm::cross( q2 - p2, r2 - p2 ) );
    glm::vec3 planePos1 = (a1 + b1 + c1) / 3.0f;
    glm::vec3 planePos2 = (a2 + b2 + c2) / 3.0f;

    LinesegIntersectInfo infos[2];
    if (Orient3D( p1, q1, r2, p2 ) <= 0)
    {
        if (Orient3D( p1, q1, q2, p2 ) >= 0)
        {
            if (Orient3D( p1, r1, q2, p2 ) < 0)
            {
                //i k j l
                assert( LinesegPlaneIntersect( p2, q2, normal1, planePos1, &infos[0] ) );
                assert( LinesegPlaneIntersect( p1, q1, normal2, planePos2, &infos[1] ) );
                start->parent_tri = 1;
                end->parent_tri = 0;
                start->ele_idx = idx2[0];
                end->ele_idx = idx1[0];
                if (infos[0].topo_type == 1)
                {
                    start->ele_idx = idx2[(0 + infos[0].idx) % 3];
                }
                if (infos[1].topo_type == 1)
                {
                    end->ele_idx = idx1[(0 + infos[1].idx) % 3];
                }
            }
            else
            {
                //k i j l
                assert( LinesegPlaneIntersect( r1, p1, normal2, planePos2, &infos[0] ) );
                assert( LinesegPlaneIntersect( p1, q1, normal2, planePos2, &infos[1] ) );
                start->parent_tri = 0;
                end->parent_tri = 0;
                start->ele_idx = idx1[2];
                end->ele_idx = idx1[0];
                if (infos[0].topo_type == 1)
                {
                    start->ele_idx = idx1[(2 + infos[0].idx) % 3];
                }
                if (infos[1].topo_type == 1)
                {
                    end->ele_idx = idx1[(0 + infos[1].idx) % 3];
                }
            }
        }
        else
        {
            return false;
        }
    }
    else
    {
        if (Orient3D( p1, r1, r2, p2 ) <= 0)
        {
            if (Orient3D( p1, r1, q2, p2 ) < 0)
            {
                //i k l j
                assert( LinesegPlaneIntersect( p2, q2, normal1, planePos1, &infos[0] ) );
                assert( LinesegPlaneIntersect( r2, p2, normal1, planePos1, &infos[1] ) );
                start->parent_tri = 1;
                end->parent_tri = 1;
                start->ele_idx = idx2[0];
                end->ele_idx = idx2[2];
                if (infos[0].topo_type == 1)
                {
                    start->ele_idx = idx2[(0 + infos[0].idx) % 3];
                }
                if (infos[1].topo_type == 1)
                {
                    end->ele_idx = idx2[(2 + infos[1].idx) % 3];
                }
            }
            else
            {
                //k i l j
                assert( LinesegPlaneIntersect( r1, p1, normal2, planePos2, &infos[0] ) );
                assert( LinesegPlaneIntersect( r2, p2, normal1, planePos1, &infos[1] ) );
                start->parent_tri = 0;
                end->parent_tri = 1;
                start->ele_idx = idx1[2];
                end->ele_idx = idx2[2];
                if (infos[0].topo_type == 1)
                {
                    start->ele_idx = idx1[(2 + infos[0].idx) % 3];
                }
                if (infos[1].topo_type == 1)
                {
                    end->ele_idx = idx2[(2 + infos[1].idx) % 3];
                }
            }
        }
        else
        {
            return false;
        }
    }

    LinesegIntersectInfo* start_info = &infos[0];
    LinesegIntersectInfo* end_info = &infos[1];
    start->pos = start_info->pos;
    end->pos = end_info->pos;
    if (start_info->topo_type == 0)
    {
        start->topo_type = 1;
    }
    else
    {
        start->topo_type = 2;

    }
    if (end_info->topo_type == 0)
    {
        end->topo_type = 1;
    }
    else
    {
        end->topo_type = 2;
    }
    return true;
}

bool TriTriIntersect2( const glm::vec3& a1, const glm::vec3& b1, const glm::vec3& c1,
    const glm::vec3& a2, const glm::vec3& b2, const glm::vec3& c2,
    TriTriIntersectInfo* start, TriTriIntersectInfo* end )
{
    Orient3D( a2, b2, c2, a1 );
    Orient3D( a2, b2, c2, b1 );
    return false;
}

bool LinesegTriIntersect( const glm::vec3& start, const glm::vec3& end,
    const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, glm::vec3* pos, glm::vec3* normal )
{
    glm::vec3 dir = glm::normalize( end - start );
    float dist = glm::distance( start, end );

    glm::vec3 e1, e2, s, s1, s2;
    float s2e2, s1s, s2d;
    glm::vec3 tbb;

    e1 = b - a;
    e2 = c - a;
    s = start - a;
    s1 = glm::cross( dir, e2 );
    s2 = glm::cross( s, e1 );

    s2e2 = glm::dot( s2, e2 );
    s1s = glm::dot( s1, s );
    s2d = glm::dot( s2, dir );
    float dt = glm::dot( s1, e1 );
    if (dt == 0.0f)
    {
        return false;
    }
    tbb = glm::vec3{ s2e2, s1s, s2d } / dt;

    if (tbb.x >= 0.f && tbb.x < dist &&
        tbb.y >= 0.f &&
        tbb.z >= 0.f &&
        (1 - tbb.y - tbb.z) >= 0.f)
    {
        *pos = start + tbb.x * dir;
        *normal = glm::normalize( glm::cross( b - a, c - a ) );
        return true;
    }
    return false;
}

bool LinePlaneIntersect( const glm::vec3& lineP, const glm::vec3& lineDir, const glm::vec3& planeNormal,
    const glm::vec3 planePos, glm::vec3* p )
{
    float denom = glm::dot( lineDir, planeNormal );
    if (denom == 0.f)
    {
        return false;
    }
    float d = glm::dot( planePos - lineP, planeNormal ) / denom;
    *p = lineDir * d + lineP;
    return true;
}

bool LinesegPlaneIntersect( const glm::vec3& start, const glm::vec3& end,
    const glm::vec3& planeNormal, const glm::vec3& planePos, LinesegIntersectInfo* info )
{
    float denom = glm::dot( end - start, planeNormal );
    if (denom == 0.f)
    {
        return false;
    }
    float d = glm::dot( planePos - start, planeNormal ) / denom;
    if (glm::FloatEqual( d, 1.0f ))
    {
        d = 1.0f;
    }
    if (glm::FloatEqual( d, 0.0f ))
    {
        d = 0.0f;
    }
    if (d < 0.0f || d > 1.0f)
    {
        return false;
    }
    if (d == 0.f)
    {
        info->topo_type = 1;
        info->idx = 0;
    }
    else if (d == 1.f)
    {
        info->topo_type = 1;
        info->idx = 1;
    }
    else
    {
        info->topo_type = 0;
    }
    info->pos = start + d * (end - start);
    return true;
}

bool LinesegRectIntersect( const glm::vec3& p0, const glm::vec3& p1, const Rect& rect, glm::vec3* pos )
{
    LinesegIntersectInfo info;
    if (!LinesegPlaneIntersect( p0, p1, rect.Normal(), rect.Center(), &info ))
    {
        return false;
    }

    glm::vec3 LU = rect.LeftUp();
    glm::vec3 LD = rect.LeftDown();
    glm::vec3 RD = rect.RightDown();
    glm::vec3 RU = rect.RightUp();
    glm::vec3 u = glm::normalize( RD - LD );
    glm::vec3 v = glm::normalize( LU - LD );
    glm::vec3 proj = info.pos;
    glm::vec2 uv( glm::dot( u, proj - LD ), glm::dot( v, proj - LD ) );

    if (uv.x >= 0 && uv.x <= rect.Width() && uv.y >= 0 && uv.y <= rect.Height())
    {
        if (pos)
        {
            *pos = proj;
        }
        return true;
    }
    return false;
}

bool LinesegAABBIntersect( const glm::vec3& p0, const glm::vec3& p1, const AABB& aabb )
{
    glm::vec3 c = aabb.GetCenter();
    glm::vec3 e = aabb.max_corner - c;
    glm::vec3 m = (p0 + p1) * 0.5f;
    glm::vec3 d = p1 - m;
    m -= c;

    float adx = glm::abs( d.x );
    if (glm::abs( m.x ) > e.x + adx)
        return false;
    float ady = glm::abs( d.y );
    if (glm::abs( m.y ) > e.y + ady)
        return false;
    float adz = glm::abs( d.z );
    if (glm::abs( m.z ) > e.z + adz)
        return false;

    adx += 1e-5f;
    ady += 1e-5f;
    adz += 1e-5f;

    if (glm::abs( m.y * d.z - m.z * d.y ) > e.y * adz + e.z * ady)
        return false;
    if (glm::abs( m.z * d.x - m.x * d.z ) > e.x * adz + e.z * adx)
        return false;
    if (glm::abs( m.x * d.y - m.y * d.x ) > e.x * ady + e.y * adx)
        return false;

    return true;
}

bool RayTriIntersect( const Ray& r, const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2, IntersectionRec* rec, float tmin, float tmax )
{
    glm::vec3 e1, e2, s, s1, s2;
    float s2e2, s1s, s2d;
    glm::vec3 tbb;

    e1 = p1 - p0;
    e2 = p2 - p0;
    s = r.o - p0;
    s1 = glm::cross( r.d, e2 );
    s2 = glm::cross( s, e1 );

    s2e2 = glm::dot( s2, e2 );
    s1s = glm::dot( s1, s );
    s2d = glm::dot( s2, r.d );

    tbb = glm::vec3{ s2e2, s1s, s2d } / glm::dot( s1, e1 );
    if (tbb.x > tmin && tbb.x < tmax &&
        tbb.y >= 0.f &&
        tbb.z >= 0.f &&
        (1 - tbb.y - tbb.z) >= 0.f)
    {
        if (rec)
        {
            rec->t = tbb.x;
            rec->p = r.at( rec->t );
            rec->normal = glm::normalize( glm::cross( p1 - p0, p2 - p0 ) );
        }
        return true;
    }
    return false;
}

bool RayBallIntersect( const Ray& ray, const glm::vec3& center, float radius, IntersectionRec* rec, float tmin, float tmax )
{
    glm::vec3 oc = ray.o - center;
    float a = 1.f;
    float b = 2.f * glm::dot( oc, ray.d );
    float c = glm::length2( oc ) - radius * radius;
    float discriminant = b * b - 4 * a * c;
    if (discriminant > 0)
    {
        float sqrt_d = glm::sqrt( discriminant );
        float t0 = (-b - sqrt_d) / (2.f * a);
        float t1 = (-b + sqrt_d) / (2.f * a);
        float t = 0.f;
        if (t0 >= 0 && t1 >= 0)
        {
            t = glm::min( t0, t1 );
        }
        else if (t0 > 0)
        {
            t = t0;
        }
        else if (t1 > 0)
        {
            t = t1;
        }
        else
        {
            return false;
        }

        if (t > tmin && t < tmax)
        {
            if (rec)
            {
                rec->p = ray.at( t );
                rec->t = t;
                rec->normal = glm::normalize( rec->p - center );
            }
            return true;
        }
    }
    return false;
}

bool RayAABBIntersect( const Ray& r, const AABB& aabb, float t_max, float* t )
{
    float tmax = FLT_MAX;
    float tmin = 0.f;
    for (int i = 0; i < 3; i++)
    {
        if (glm::abs( r.d[i] ) < 1e-7f)
        {
            if (r.o[i] < aabb.min_corner[i] || r.o[i] > aabb.max_corner[i])
            {
                return false;
            }
        }
        else
        {
            float ood = 1.f / r.d[i];
            float t1 = (aabb.min_corner[i] - r.o[i]) * ood;
            float t2 = (aabb.max_corner[i] - r.o[i]) * ood;
            if (t1 > t2)
            {
                std::swap( t1, t2 );
            }
            if (t1 > tmin)
            {
                tmin = t1;
            }
            if (t2 < tmax)
            {
                tmax = t2;
            }
            if (tmin > tmax)
            {
                return false;
            }
        }
    }
    if (tmin > t_max)
    {
        return false;
    }
    if (t != nullptr)
        *t = tmin;
    return true;
}

float Orient3D( const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d )
{
    return glm::determinant( glm::mat3x3( a - d, b - d, c - d ) );
}

bool LinesegCuttoolIntersection( const glm::vec3& start, const glm::vec3& end, const CuttingTool& rect, LinesegIntersectInfo* info )
{
    if (!LinesegPlaneIntersect( start, end, rect.mTransform.Up(), rect.mTransform.GetPosition(), info ))
    {
        return false;
    }
    if (!rect.InPlane( rect.PlaneSpace( info->pos ) ))
    {
        return false;
    }
    return true;
}

bool BallRectIntersect( glm::vec3 c, float r, const Rect& rect )
{
    if (glm::MinDistToRect( c, rect ) < r)
    {
        return true;
    }
    return false;
}

bool BallTriIntersect( glm::vec3 c, float r, const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2, glm::vec3* pc, float* depth )
{
    float dist = 0.f;
    glm::vec3 n = glm::normalize( glm::cross( p1 - p0, p2 - p0 ) );
    glm::vec3 proj = c - glm::dot( c - p0, n ) * n;
    glm::vec3 bcpos = BarycentricPos( p0, p1, p2, proj );
    if (bcpos[0] >= 0.f && bcpos[0] <= 1.f &&
        bcpos[1] >= 0.f && bcpos[1] <= 1.f &&
        bcpos[2] >= 0.f && bcpos[2] <= 1.f)
    {
        dist = glm::distance( proj, c );
    }
    else
    {
        float all_distances[] = {
            MinDistToLineSeg( c, p0, p1 ),
            MinDistToLineSeg( c, p1, p2 ),
            MinDistToLineSeg( c, p2, p0 )
        };
        dist = glm::min( all_distances[0], glm::min( all_distances[1], all_distances[2] ) );
    }

    if (dist < r)
    {
        *depth = r - glm::dot( c - p0, n );
        *pc = proj;
        return true;
    }
    return false;
}

bool BallAABBIntersect( glm::vec3 c, float r, const AABB& aabb )
{
    float sqdist = 0.f;
    for (int i = 0; i < 3; i++)
    {
        if (c[i] < aabb.min_corner[i])
            sqdist += (aabb.min_corner[i] - c[i]) * (aabb.min_corner[i] - c[i]);
        else if (c[i] > aabb.max_corner[i])
            sqdist += (c[i] - aabb.max_corner[i]) * (c[i] - aabb.max_corner[i]);
    }

    if (glm::sqrt( sqdist ) < r)
        return true;
    else
        return false;
}

std::optional<BallBallIntersectInfo> BallBallIntersect( glm::vec3 c1, float r1, glm::vec3 c2, float r2 )
{
    float dist_c1_c2 = glm::distance( c1, c2 );
    std::optional<BallBallIntersectInfo> result;
    if ((r1 + r2) > dist_c1_c2)
    {
        BallBallIntersectInfo info;
        glm::vec3 norm_c1_c2 = glm::normalize( c2 - c1 );
        info.c1toc2 = norm_c1_c2;
        info.d = r1 + r2 - dist_c1_c2;
        info.mid = c1 + norm_c1_c2 * (r1 - info.d * 0.5f);
        result = info;
    }
    return result;
}

bool AABBIntersect( const AABB& aabb1, const AABB& aabb2 )
{
    if (aabb1.max_corner[0] < aabb2.min_corner[0] || aabb1.min_corner[0] > aabb2.max_corner[0])
        return false;
    if (aabb1.max_corner[1] < aabb2.min_corner[1] || aabb1.min_corner[1] > aabb2.max_corner[1])
        return false;
    if (aabb1.max_corner[2] < aabb2.min_corner[2] || aabb1.min_corner[2] > aabb2.max_corner[2])
        return false;
    return true;
}

bool MovingSphereTriIntersect( glm::vec3 C, float r, glm::vec3 v, glm::vec3 p0, glm::vec3 p1, glm::vec3 p2, MovingSphereTriIntersectInfo* info )
{
    glm::vec3 n = glm::normalize( glm::cross( p1 - p0, p2 - p0 ) );
    //if (glm::dot( v, n ) >= 0.f)
    //    return false;

    glm::vec3 D = C - r * n;
    Ray DP( D, glm::normalize( v ) );

    float d = glm::dot( n, p0 );

    float t = (d - glm::dot( n, D )) / glm::dot( n, v );
    glm::vec3 P = D + t * v;
    if (t >= 0.f && t <= 1.f)
    {
        glm::vec3 bc = glm::BarycentricPos( p0, p1, p2, P );
        if (bc.x >= 0 && bc.x <= 1 && bc.y >= 0 && bc.y <= 1 && bc.z >= 0 && bc.z <= 1)
        {
            info->t = t * glm::length( v );
            info->p = P;
            info->nc = n;
            return true;
        }
    }
    if (t < 0.f || t > 1.f)
        return false;

    glm::vec3 Q;
    int topo = 0;
    glm::MinDistToTriangle( P, p0, p1, p2, &topo, &Q );
    Ray QR( Q, -glm::normalize( v ) );
    IntersectionRec rayball;
    if (RayBallIntersect( QR, C, r, &rayball, 0.f, glm::length( v ) ))
    {
        info->t = glm::length( v ) - rayball.t;
        info->p = Q;
        info->nc = glm::normalize( C - rayball.p );
        //info->nc = n;
        return true;
    }

    return false;
}


