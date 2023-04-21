#pragma once
#include <optional>
#include <glm/glm.hpp>
#include "../model/Ray.h"
#include "../model/Rect.h"
#include "../acceleration/AABB.h"

class CuttingTool;

float Orient3D( const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d );

class TriTriIntersectInfo
{
public:
    glm::vec3 pos;
    int parent_tri = -1;
    int ele_idx = -1;
    int topo_type = 0;//0=inside, 1=edge, 2=point
};

struct MovingSphereTriIntersectInfo
{
    float t;
    glm::vec3 p;
    glm::vec3 nc;
};

class LinesegIntersectInfo
{
public:
    glm::vec3 pos;
    int topo_type = 0;//0=inside, 1=point;
    int idx = 0; //0=start point, 1=end point
};

struct BallBallIntersectInfo
{
    glm::vec3 mid;
    glm::vec3 c1toc2;
    float d;
};

bool CutFaceIntersect( const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
    const glm::vec3& cut_face_a, const glm::vec3& cut_face_b, const glm::vec3& cut_face_c,
    TriTriIntersectInfo* start, TriTriIntersectInfo* end );

bool TriTriIntersect( const glm::vec3& a1, const glm::vec3& b1, const glm::vec3& c1,
    const glm::vec3& a2, const glm::vec3& b2, const glm::vec3& c2,
    TriTriIntersectInfo* start, TriTriIntersectInfo* end, int main = -1 );

bool TriTriIntersect2( const glm::vec3& a1, const glm::vec3& b1, const glm::vec3& c1,
    const glm::vec3& a2, const glm::vec3& b2, const glm::vec3& c2,
    TriTriIntersectInfo* start, TriTriIntersectInfo* end );

bool LinesegTriIntersect( const glm::vec3& start, const glm::vec3& end,
    const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, glm::vec3* pos, glm::vec3* normal );

bool LinePlaneIntersect( const glm::vec3& lineP, const glm::vec3& lineDir,
    const glm::vec3& planeNormal, const glm::vec3 planePos, glm::vec3* p );

bool LinesegPlaneIntersect( const glm::vec3& start, const glm::vec3& end,
    const glm::vec3& planeNormal, const glm::vec3& planePos, LinesegIntersectInfo* info );

bool LinesegRectIntersect( const glm::vec3& p0, const glm::vec3& p1, const Rect& rect, glm::vec3* pos );

bool LinesegAABBIntersect( const glm::vec3& p0, const glm::vec3& p1, const AABB& aabb );

bool RayTriIntersect( const Ray& r, const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2, IntersectionRec* rec, float tmin = 0.f, float tmax = FLT_MAX );

bool RayBallIntersect( const Ray& ray, const glm::vec3& center, float radius, IntersectionRec* rec, float tmin = 0.f, float tmax = FLT_MAX );

bool RayAABBIntersect( const Ray& r, const AABB& aabb, float t_max, float* t );

template <typename Scalar>
bool TestRayAABB( glm::vec<3, Scalar> p, glm::vec<3, Scalar> d, AABB a, float& tmin, glm::vec<3, Scalar>& q )
{
    tmin = 0.0;
    Scalar tmax = std::numeric_limits<Scalar>::max();
    for (int i = 0; i < 3; i++)
    {
        if (glm::abs( d[i] ) < 1e-6)
        {
            if (p[i] < a.min_corner[i] || p[i] > a.max_corner[i])
                return 0;
        }
        else
        {
            Scalar ood = 1.0 / d[i];
            Scalar t1 = (a.min_corner[i] - p[i]) * ood;
            Scalar t2 = (a.max_corner[i] - p[i]) * ood;
            if (t1 > t2)
                std::swap( t1, t2 );
            tmin = glm::max( t1, tmin );
            tmax = glm::min( t2, tmax );
            if (tmin > tmax)
                return false;
        }
    }
    q = p + d * tmin;
    return true;
}

template <typename Scalar>
bool TestRaySphere( glm::vec<3, Scalar> p, glm::vec<3, Scalar> d, glm::vec<3, Scalar> c, float r, float& t, glm::vec<3, Scalar>& q )
{
    auto m = p - c;
    Scalar b = glm::dot( m, d );
    Scalar cc = glm::dot( m, m ) - r * r;
    if (cc > 0.0 && b > 0.0)
        return false;
    Scalar discr = b * b - cc;
    if (discr < 0.0)
        return false;

    t = -b - glm::sqrt( discr );
    if (t < 0.0)
        t = 0.0;
    q = p + t * d;
    return true;
}

template <typename Scalar>
bool TestSegmentSphere( glm::vec<3, Scalar> sa, glm::vec<3, Scalar> sb, glm::vec<3, Scalar> c, float r, float& t, glm::vec<3, Scalar>& q )
{
    auto d = sb - sa;
    Scalar norm_d = glm::length( d );
    TestRaySphere( sa, d / norm_d, c, r, t, q );
    if (t >= 0.0 && t <= norm_d)
        return true;
    return false;
}

bool LinesegCuttoolIntersection( const glm::vec3& start, const glm::vec3& end, const CuttingTool& rect, LinesegIntersectInfo* info );

bool BallRectIntersect( glm::vec3 c, float r, const Rect& rect );

bool BallTriIntersect( glm::vec3 c, float r, const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2, glm::vec3* pc, float* depth );

bool BallAABBIntersect( glm::vec3 c, float r, const AABB& aabb );

std::optional<BallBallIntersectInfo> BallBallIntersect( glm::vec3 c1, float r1, glm::vec3 c2, float r2 );

template <typename Scalar>
Scalar SqDistPointAABB( glm::vec<3, Scalar> p, const AABB& aabb )
{
    Scalar sq_dist = 0.0;
    for (int i = 0; i < 3; i++)
    {
        auto v = p[i];
        if (v < aabb.min_corner[i])
        {
            sq_dist += (aabb.min_corner[i] - v) * (aabb.min_corner[i] - v);
        }
        if (v > aabb.max_corner[i])
        {
            sq_dist += (v - aabb.max_corner[i]) * (v - aabb.max_corner[i]);
        }
    }
    return sq_dist;
}

template <typename Scalar>
glm::vec<3, Scalar> ClosestPointToAABB( glm::vec<3, Scalar> p, const AABB& aabb )
{
    glm::vec<3, Scalar> result( 0.0 );
    for (int i = 0; i < 3; i++)
    {
        auto v = p[i];
        if (v < aabb.min_corner[i])
            v = aabb.min_corner[i];
        if (v > aabb.max_corner[i])
            v = aabb.max_corner[i];
        result[i] = v;
    }
    return result;
}

template <typename Scalar>
bool TestBallAABB( glm::vec<3, Scalar> c, Scalar r, const AABB& aabb, glm::vec<3, Scalar>* p )
{
    auto closest_pt = ClosestPointToAABB( c, aabb );
    *p = closest_pt;
    return glm::distance2( closest_pt, c ) <= r * r;
}

template <typename Scalar>
glm::vec<3, Scalar> ClosestPointToTriangle( glm::vec<3, Scalar> p, glm::vec<3, Scalar> a, glm::vec<3, Scalar> b, glm::vec<3, Scalar> c )
{
    auto ab = b - a;
    auto ac = c - a;
    auto ap = p - a;
    Scalar d1 = glm::dot( ab, ap );
    Scalar d2 = glm::dot( ac, ap );
    if (d1 <= 0.f && d2 < 0.f)
        return a;

    auto bp = p - b;
    Scalar d3 = glm::dot( ab, bp );
    Scalar d4 = glm::dot( ac, bp );
    if (d3 >= 0.f && d4 < d3)
        return b;

    Scalar vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
    {
        Scalar v = d1 / (d1 - d3);
        return a + v * ab;
    }

    auto cp = p - c;
    Scalar d5 = glm::dot( ab, cp );
    Scalar d6 = glm::dot( ac, cp );
    if (d6 >= 0.f && d5 <= d6)
        return c;

    Scalar vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
    {
        Scalar w = d2 / (d2 - d6);
        return a + w * ac;
    }

    Scalar va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
    {
        Scalar w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);
    }

    Scalar denom = 1.f / (va + vb + vc);
    Scalar v = vb * denom;
    Scalar w = vc * denom;
    return a + ab * v + ac * w;
}

template <typename Scalar>
bool TestBallTri( glm::vec<3, Scalar> sc, Scalar r, glm::vec<3, Scalar> a, glm::vec<3, Scalar> b, glm::vec<3, Scalar> c, glm::vec<3, Scalar>* p )
{
    auto closest_pt = ClosestPointToTriangle( sc, a, b, c );
    if (glm::distance2( closest_pt, sc ) <= r * r)
    {
        *p = closest_pt;
        return true;
    }
    return false;
}

bool AABBIntersect( const AABB& aabb1, const AABB& aabb2 );

bool MovingSphereTriIntersect( glm::vec3 c, float r, glm::vec3 v, glm::vec3 p0, glm::vec3 p1, glm::vec3 p2, MovingSphereTriIntersectInfo* info );

template <typename Scalar>
bool TestMovingSphereSphere( glm::vec<3, Scalar> c0, Scalar r0, glm::vec<3, Scalar> v0, glm::vec<3, Scalar> c1, Scalar r1, glm::vec<3, Scalar> v1, float* t )
{
    auto s = c1 - c0;
    auto v = v1 - v0;
    Scalar r = r0 + r1;
    Scalar c = glm::length2( s ) - r * r;
    if (c < 0.0f)
    {
        *t = 0.0f;
        return true;
    }

    Scalar a = glm::length2( v );
    if (a < 1e-6f)
    {
        return false;
    }
    Scalar b = glm::dot( v, s );
    if (b >= 0.f)
    {
        return false;
    }

    Scalar d = b * b - a * c;
    if (d < 0.0f)
    {
        return false;
    }

    *t = (-b - glm::sqrt( d )) / a;
    if (*t >= 0.f && *t <= 1.f)
        return true;
    return false;
}

template <typename Scalar>
bool TestSegmentCylinder( glm::vec<3, Scalar> sa, glm::vec<3, Scalar> sb, glm::vec<3, Scalar> p, glm::vec<3, Scalar> q, float r, float* t )
{
    auto d = p - q;
    auto m = sa - p;
    auto n = sb - sa;

    Scalar md = glm::dot( m, d );
    Scalar nd = glm::dot( n, d );
    Scalar dd = glm::dot( d, d );

    if (md < 0.0 && md + nd < 0.0)
        return false;
    if (md > dd && md + nd > dd)
        return false;

    float nn = glm::dot( n, n );
    float mn = glm::dot( m, n );

    float a = dd * nn - nd * nd;
    float k = glm::dot( m, m ) - r * r;
    float c = dd * k - md * md;

    if (glm::abs( a ) < 1e-6)
    {
        if (c > 0.0)
            return false;
        if (md < 0.0)
            *t = -mn / nn;
        else if (md > dd)
            *t = (nd - mn) / nn;
        else
            *t = 0.f;
        return true;
    }

    float b = dd * mn - nd * md;
    float discr = b * b - a * c;
    if (discr < 0.0)
        return false;
    *t = (-b - glm::sqrt( discr )) / a;
    if (*t < 0.0 || *t > 1.0)
        return false;
    if (md + *t * nd < 0.0)
    {
        if (nd <= 0.0)
            return false;
        *t = -md / nd;
        return k + 2 * *t * (mn + *t * nn) <= 0.0;
    }
    else if (md + *t * nd > dd)
    {
        if (nd >= 0.0)
            return false;
        *t = (dd - md) / nd;
        return k + dd - 2 * md + *t * (2 * (mn - nd) + *t * nn) <= 0.0;
    }
    return true;
}

template <typename Scalar>
bool TestSegmentCapsule( glm::vec<3, Scalar> sa, glm::vec<3, Scalar> sb, glm::vec<3, Scalar> p, glm::vec<3, Scalar> q, float r, float* t )
{
    Scalar t0 = std::numeric_limits<Scalar>::max();
    Scalar t1 = t0;
    Scalar t2 = t0;
    Scalar tt = t0;
    glm::vec<3, Scalar> p0;
    glm::vec<3, Scalar> p1;
    bool b0 = false;
    bool b1 = false;
    bool b2 = false;

    b0 = TestSegmentSphere( sa, sb, p, r, t0, p0 );
    if (b0)
    {
        tt = t0;
    }
    b1 = TestSegmentSphere( sa, sb, q, r, t1, p1 );
    if (b1 && t1 < t0)
    {
        tt = t1;
    }
    b2 = TestSegmentCylinder( sa, sb, p, q, r, &t2 );
    if (b2 && t2 < tt)
    {
        tt = t2;
    }
    *t = tt;
    return b0 || b1 || b2;
}

inline glm::vec3 AABBCorner( const AABB& aabb, int n )
{
    glm::vec3 p;
    p.x = ((n & 1) ? aabb.max_corner.x : aabb.min_corner.x);
    p.y = ((n & 2) ? aabb.max_corner.y : aabb.min_corner.y);
    p.z = ((n & 4) ? aabb.max_corner.z : aabb.min_corner.z);
    return p;
}

template <typename Scalar>
bool TestMovingSphereAABB( glm::vec<3, Scalar> c, Scalar r, glm::vec<3, Scalar> d, AABB aabb, float* t )
{
    AABB e = aabb;
    e.min_corner -= glm::vec<3, Scalar>( r );
    e.max_corner += glm::vec<3, Scalar>( r );

    glm::vec<3, Scalar> p;
    if (!TestRayAABB( c, d, e, *t, p ) || *t > 1.0)
    {
        return false;
    }

    int u = 0;
    int v = 0;
    if (p.x < aabb.min_corner.x)
        u |= 1;
    if (p.x > aabb.max_corner.x)
        v |= 1;
    if (p.y < aabb.min_corner.y)
        u |= 2;
    if (p.y > aabb.max_corner.y)
        v |= 2;
    if (p.z < aabb.min_corner.z)
        u |= 4;
    if (p.z > aabb.max_corner.z)
        v |= 4;

    int m = u + v;

    glm::vec<3, Scalar> sa = c;
    glm::vec<3, Scalar> sb = c + d;
    if (m == 7)
    {
        Scalar tmin = std::numeric_limits<Scalar>::max();
        if (TestSegmentCapsule( sa, sb, AABBCorner( aabb, v ), AABBCorner( aabb, v ^ 1 ), r, t ))
            tmin = glm::min( *t, tmin );
        if (TestSegmentCapsule( sa, sb, AABBCorner( aabb, v ), AABBCorner( aabb, v ^ 2 ), r, t ))
            tmin = glm::min( *t, tmin );
        if (TestSegmentCapsule( sa, sb, AABBCorner( aabb, v ), AABBCorner( aabb, v ^ 4 ), r, t ))
            tmin = glm::min( *t, tmin );
        if (tmin == std::numeric_limits<Scalar>::max())
            return false;
        *t = tmin;
        return true;
    }

    if ((m & (m - 1)) == 0)
    {
        return true;
    }

    return TestSegmentCapsule( sa, sb, AABBCorner( aabb, u ^ 7 ), AABBCorner( aabb, v ), r, t );
}