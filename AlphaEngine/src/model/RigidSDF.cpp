#include "RigidSDF.h"

RigidSDF::RigidSDF( const std::string& path, float dx )
    :_sdf( CreateSDF( path, dx ) )
{
    _mesh = std::make_unique<CMesh>( path );
}

bool RigidSDF::CheckPoint( glm::vec3 p, glm::vec3* n, float* depth ) const
{
    if (p.x <= _sdf._min[0] || p.x >= _sdf._max[0] ||
        p.y <= _sdf._min[1] || p.y >= _sdf._max[1] ||
        p.z <= _sdf._min[2] || p.z >= _sdf._max[2])
        return false;
    float sdf = _sdf( p.x, p.y, p.z );
    if (sdf > 0)
        return false;

    Vec3f g = _sdf.Grad( p.x, p.y, p.z );
    *n = glm::normalize( glm::vec3( g[0], g[1], g[2] ) );
    *depth = -sdf;
    return true;
}

bool RigidSDF::CheckBall( glm::vec3 c, float r, glm::vec3* n, float* depth ) const
{
    if (c.x <= _sdf._min[0] || c.x >= _sdf._max[0] ||
        c.y <= _sdf._min[1] || c.y >= _sdf._max[1] ||
        c.z <= _sdf._min[2] || c.z >= _sdf._max[2])
        return false;
    float sdf = _sdf( c.x, c.y, c.z );
    if (sdf < 0)
    {
        Vec3f g = _sdf.Grad( c.x, c.y, c.z );
        *n = glm::normalize( glm::vec3( g[0], g[1], g[2] ) );
        *depth = -sdf;
        return true;
    }
    else if (sdf > 0 && sdf < r)
    {
        Vec3f g = _sdf.Grad( c.x, c.y, c.z );
        *n = glm::normalize( glm::vec3( g[0], g[1], g[2] ) );
        *depth = r - sdf;
        return true;
    }
    return false;
}

void RigidSDF::Update()
{
}

void RigidSDF::Draw()
{
    _mesh->Draw();
}
