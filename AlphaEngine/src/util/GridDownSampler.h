#pragma once
#include <vector>
#include "glm/glm.hpp"
#include "../acceleration/AABB.h"

class GridDownSampler
{
public:
    GridDownSampler( std::vector<glm::vec3>* points, float d );
    std::vector<glm::vec3> GetSamples();

private:
    std::vector<glm::vec3>* _points;
    float _d;
    AABB _aabb;
};

