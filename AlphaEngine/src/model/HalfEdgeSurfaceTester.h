#pragma once
#include <glm/glm.hpp>
#include "../model/Ray.h"

class HalfEdgeMesh;

struct ProjectToSurfaceRet
{
    glm::vec3 _pos;
    int _faceid;
    float _dist;
};

class HalfEdgeSurfaceTester
{
public:
    HalfEdgeSurfaceTester() = default;
    HalfEdgeSurfaceTester( HalfEdgeMesh* surface );
    void SetSurface( HalfEdgeMesh* surface );
    bool PointIsInSurface( glm::vec3 p ) const;
    float MinDistToSurface( glm::vec3 p, int* face_idx = nullptr ) const;
    bool RayIntersect( Ray ray, IntersectionRec* rec = nullptr, float t_min = 0.f, float t_max = FLT_MAX, int* id = nullptr );
    bool LinesegIntersect( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec = nullptr );
    ProjectToSurfaceRet ProjectToSurface( glm::vec3 p ) const;
private:
    HalfEdgeMesh* _surface = nullptr;
};