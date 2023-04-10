#include "AABB.h"
#include <algorithm>

AABB::AABB() :max_corner( glm::vec3( std::numeric_limits<float>::lowest() ) ), min_corner( glm::vec3( FLT_MAX ) )
{

}

void AABB::Expand( glm::vec3 p )
{
    for (int i = 0; i < 3; i++)
    {
        max_corner[i] = glm::max( max_corner[i], p[i] );
        min_corner[i] = glm::min( min_corner[i], p[i] );
    }
}

unsigned int AABB::LongestAxis() const
{
    float diffx = max_corner.x - min_corner.x;
    float diffy = max_corner.y - min_corner.y;
    float diffz = max_corner.z - min_corner.z;
    if (diffx > diffy && diffx > diffz)
    {
        return 0;
    }
    else if (diffy > diffz)
    {
        return 1;
    }
    else
    {
        return 2;
    }
}

glm::vec3 AABB::GetCenter() const
{
    return (max_corner + min_corner) * 0.5f;
}

bool AABB::Intersect( glm::vec3 point ) const
{
    return (point[0] > min_corner[0] && point[0] < max_corner[0] &&
        point[1] > min_corner[1] && point[1] < max_corner[1] &&
        point[2] > min_corner[2] && point[2] < max_corner[2]);
}
