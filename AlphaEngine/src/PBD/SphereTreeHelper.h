#pragma once
#include <vector>
#include "glm/glm.hpp"
#include "sphere_tree/SphereTree/SphereTree.h"
#include "sphere_tree/MedialAxis/MedialTester.h"
#include "sphere_tree/MedialAxis/Voronoi3D.h"
#include "../model/Rect.h"

namespace PBD
{
    class Metaball;
    class MeshlessPoint;
    struct BoundaryPoint;
}
class Triangle;

std::vector<glm::vec3> PointsToMetaball( std::vector<glm::vec3>& points,
    const std::vector<PBD::Metaball>& all_balls, const std::vector<int>& removed_balls, const Rect& rect, bool upper );
