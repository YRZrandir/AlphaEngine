#pragma once
#include <iostream>
#include <memory>
#include <vector>
#include <array>

#include <glad/glad.h>
#include <glm/glm.hpp>

#include "AABB.h"
#include "../model/HalfEdgeMesh.h"
#include "../model/Ray.h"

class VertexArray;
class VertexBuffer;
class IndexBuffer;

namespace PBD
{
class MetaballModel;
}

struct CollisionInfo
{
    glm::vec3 p;
    glm::vec3 n;
    float d;
    int id;
};

class BVHNode
{
public:
    BVHNode() = default;
    virtual ~BVHNode() = default;
    void Draw( int level ) const;
    void ResetColor() const;
    bool IsPointInBVH( const glm::vec3& p ) const;
    AABB BoundingBox() const;
    virtual void Refit() = 0;
    virtual bool CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const = 0;
    virtual bool CheckBall( glm::vec3 c, float r, std::vector<CollisionInfo>* infos ) const = 0;
    virtual bool CheckRay( glm::vec3 o, glm::vec3 d, CollisionInfo* info, float* t ) const = 0;
protected:
    static void Init();

    AABB mBoundingBox;
    std::unique_ptr<BVHNode> mLeft = nullptr;
    std::unique_ptr<BVHNode> mRight = nullptr;
    std::vector<int> mIndices;
    mutable glm::vec3 mColor = glm::vec3( 0.97, 0.97, 0.58 );

    static const int THRESHOLD;
    static bool sInit;
    static std::unique_ptr<VertexArray> sVAO;
    static std::unique_ptr<VertexBuffer> sVBO;
    static std::unique_ptr<IndexBuffer> sIBO;
    static std::array<glm::vec3, 8> sVertexData;
    static std::array<unsigned int, 24> sIndexData;
};

class BVHTree
    : public SceneObject
{
public:
    BVHTree() = default;
    virtual ~BVHTree() = default;
    virtual void Update() override;
    virtual void Draw() override;
    virtual void Refit() = 0;
    virtual bool CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const = 0;

    AABB BoundingBox() const;

protected:
    std::unique_ptr<BVHNode> _root;
};


class BVHTree_HalfEdgeMesh
    : public BVHTree
{
public:
    BVHTree_HalfEdgeMesh( HalfEdgeMesh* mesh );
    virtual void Refit() override;
    virtual bool CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const override;

private:
    HalfEdgeMesh* _mesh = nullptr;
};

class BVHNode_HalfEdgeMesh
    : public BVHNode
{
public:
    BVHNode_HalfEdgeMesh( std::vector<int> indices, HalfEdgeMesh* mesh );
    virtual void Refit() override;
    virtual bool CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const override;
    virtual bool CheckBall( glm::vec3 c, float r, std::vector<CollisionInfo>* infos ) const override;
    virtual bool CheckRay( glm::vec3 o, glm::vec3 d, CollisionInfo* info, float* t ) const override;

private:
    HalfEdgeMesh* _mesh;
};


class BVHTree_Metaball
    : public BVHTree
{
public:
    BVHTree_Metaball( PBD::MetaballModel* model );
    virtual void Refit() override;
    virtual bool CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const override;
    bool CheckBall( glm::vec3 c, float r, std::vector<CollisionInfo>* infos ) const;
    bool CheckRay( glm::vec3 o, glm::vec3 d, CollisionInfo* info ) const;
private:
    PBD::MetaballModel* _model;
};

class BVHNode_Metaball
    :public BVHNode
{
public:
    BVHNode_Metaball( std::vector<int> indices, PBD::MetaballModel* model );
    virtual void Refit() override;
    virtual bool CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const override;
    virtual bool CheckBall( glm::vec3 c, float r, std::vector<CollisionInfo>* infos ) const override;
    virtual bool CheckRay( glm::vec3 o, glm::vec3 d, CollisionInfo* info, float* t ) const override;

private:
    PBD::MetaballModel* _model;

};

//
//template <SphereType Sphere>
//class BVHTree_SphereMesh
//    :public BVHTree
//{
//public:
//    BVHTree_SphereMesh( SphereMesh<Sphere>* mesh );
//    virtual void Refit() override;
//    virtual bool CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const override;
//
//protected:
//    SphereMesh<Sphere>* _mesh;
//};
//
//template <SphereType Sphere>
//BVHTree_SphereMesh<Sphere>::BVHTree_SphereMesh( SphereMesh<Sphere>* mesh )
//{
//
//}
//
//template <SphereType Sphere>
//void BVHTree_SphereMesh<Sphere>::Refit()
//{
//
//}
//
//template <SphereType Sphere>
//bool BVHTree_SphereMesh<Sphere>::CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const
//{
//    return false;
//}
//
//template <SphereType Sphere>
//class BVHNode_SphereMesh
//    :public BVHNode
//{
//public:
//    BVHNode_SphereMesh( std::vector<int> indices, SphereMesh<Sphere>* mesh );
//
//
//    // Inherited via BVHNode
//    virtual void Refit() override;
//
//    virtual bool CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const override;
//
//    virtual bool CheckBall( glm::vec3 c, float r, std::vector<CollisionInfo>* infos ) const override;
//
//    virtual bool CheckRay( glm::vec3 o, glm::vec3 d, CollisionInfo* info, float* t ) const override;
//
//};
//
//template<SphereType Sphere>
//inline BVHNode_SphereMesh<Sphere>::BVHNode_SphereMesh( std::vector<int> indices, SphereMesh<Sphere>* mesh )
//{
//
//}
//
//template<SphereType Sphere>
//inline void BVHNode_SphereMesh<Sphere>::Refit()
//{
//
//}
//
//template<SphereType Sphere>
//inline bool BVHNode_SphereMesh<Sphere>::CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const
//{
//    return false;
//}
//
//template<SphereType Sphere>
//inline bool BVHNode_SphereMesh<Sphere>::CheckBall( glm::vec3 c, float r, std::vector<CollisionInfo>* infos ) const
//{
//
//}
//
//template<SphereType Sphere>
//inline bool BVHNode_SphereMesh<Sphere>::CheckRay( glm::vec3 o, glm::vec3 d, CollisionInfo* info, float* t ) const
//{
//    return false;
//}
