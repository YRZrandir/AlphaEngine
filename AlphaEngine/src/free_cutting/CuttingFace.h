#pragma once
#include <glm/glm.hpp>
#include <utility>

class CuttingFace
{
public:
    class IntersectInfo
    {
    public:
        glm::vec3 p;
        bool onedge = false;
        int edge = -1;
    };
    glm::vec3 pa;
    glm::vec3 pb;
    glm::vec3 pc;
    glm::vec3 normal;
public:
    CuttingFace() = default;
    CuttingFace(glm::vec3 a, glm::vec3 b, glm::vec3 c);
    bool TriangleIntersect(glm::vec3 a, glm::vec3 b, glm::vec3 c, std::pair<IntersectInfo, IntersectInfo>* result) const;

private:
    bool LineIntersect(glm::vec3 a, glm::vec3 b, glm::vec3* pos) const;
    glm::vec3 BarycentricPos(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 pos) const;
};

