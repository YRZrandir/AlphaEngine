#pragma once
#include <string>
#include <memory>
#include "util/SceneObject.h"
#include "HalfEdgeMesh.h"
#include "HalfEdgeSurfaceTester.h"
#include "acceleration/BVH.h"
#include "PBD/PBDTetraModel.h"

class RigidStatic :
    public SceneObject
{
public:
    RigidStatic( std::string path );
    virtual void Update() override;
    virtual void Draw() override;
    HalfEdgeMesh& Surface();
    BVHTree& BVH();
    glm::vec3 GetPos( size_t i ) const { return _trans_pos[i]; }
    bool CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const;
    std::vector<PBD::CollisionConstraint> CheckBall( glm::vec3 c, float r, int id ) const;
    bool CheckInside( glm::vec3 p, glm::vec3* normal, float* depth, int* id ) const;
    void UpdateTransPos();
private:
    std::unique_ptr<HalfEdgeMesh> _surface = nullptr;
    std::unique_ptr<BVHTree> _bvh = nullptr;
    std::vector<glm::vec3> _trans_pos;
};

class RigidBall :
    public SceneObject
{
public:
    RigidBall( glm::vec3 pos, float r );
    virtual void Update() override;
    virtual void Draw() override;
    void UpdateTransPos();
    glm::vec3 GetPos() const { return _trans_pos; };
    float GetRadius() const { return _r; }
private:
    std::unique_ptr<HalfEdgeMesh> _surface = nullptr;
    glm::vec3 _trans_pos;
    float _r;
    glm::vec3 _ori_pos;
};

class RigidAABB : public SceneObject
{
public:
    RigidAABB( glm::vec3 min_corner, glm::vec3 max_corner );

private:
    glm::vec3 min_corner;
    glm::vec3 max_corner;
};