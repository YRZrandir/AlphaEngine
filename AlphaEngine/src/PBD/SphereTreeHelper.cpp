#include "SphereTreeHelper.h"
#include "../PBD/MetaballModel.h"

std::vector<glm::vec3> PointsToMetaball( std::vector<glm::vec3>& points,
    const std::vector<PBD::Metaball>& all_balls, const std::vector<int>& removed_balls, const Rect& rect, bool upper)
{
    Voronoi3D vor;
    Array<Surface::Point> samples;
    AABB aabb;

    samples.resize( points.size() );
    for (size_t i = 0; i < points.size(); i++)
    {
        samples.index( i ) = { points[i].x, points[i].y, points[i].z };
        aabb.Expand( points[i] );
    }

    glm::vec3 pMin = { aabb.min_corner.x, aabb.min_corner.y, aabb.min_corner.z };
    glm::vec3 pMax = { aabb.max_corner.x, aabb.max_corner.y, aabb.max_corner.z };
    glm::vec3 pc = (pMax + pMin) / 2.0f;
    Point3D pC{ pc.x, pc.y, pc.z };

    vor.initialise( pC, 10.0 * glm::distance(pMin, pc) );
    vor.randomInserts( samples, 500 );

    auto in_ball = [&](glm::vec3 p)->int
    {
        for (int i : removed_balls)
        {
            auto& ball = all_balls[i];
            if (glm::distance( p, ball.x ) < ball.r)
            {
                return i;
            }
        }
        return -1;
    };

    glm::vec3 rect_n = rect.Normal();

    std::vector<glm::vec3> results;
    int numVerts = vor.vertices.getSize();
    for (int i = 0; i < numVerts; i++) {
        Voronoi3D::Vertex* v = &vor.vertices.index( i );
        glm::vec3 pos( v->s.c.x, v->s.c.y, v->s.c.z );
        int ball_idx = in_ball( pos );

        bool above = glm::dot( rect_n, pos - rect.Center() ) > 0;

        if ( ball_idx != -1 && above == upper) {
            results.push_back( pos );
        }
    }

    return results;
}