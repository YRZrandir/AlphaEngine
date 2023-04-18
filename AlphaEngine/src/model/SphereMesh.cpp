#include "SphereMesh.h"

SpatialHashing::SpatialHashing( float dx )
    :_dx( dx )
{

}

void SpatialHashing::Insert( SphereBase* s )
{

}

std::vector<SphereBase*> SpatialHashing::CheckIntersection( SphereBase* s ) const
{
    std::vector<SphereBase*> result;

    return result;
}

int SpatialHashing::GridCoord( float p ) const
{
    return 0;
}