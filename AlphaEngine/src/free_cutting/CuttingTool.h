#pragma once
#include <glad/glad.h>
#include <glm/gtc/quaternion.hpp>

#include "../acceleration/AABB.h"
#include "../util/SceneObject.h"
#include "../model/Transform.h"

class VertexArray;
class VertexBuffer;
class IndexBuffer;
class Shader;
class HalfEdgeMesh;

class CuttingTool : public SceneObject
{
public:
    glm::vec3 normal;
    float width;
    float height;
private:
    static glm::vec3 sPoints[4];
    static GLuint sIndices[6];
    static std::unique_ptr<VertexArray> sVAO;
    static std::unique_ptr<VertexBuffer> sVBO;
    static std::unique_ptr<IndexBuffer> sIBO;
    static Shader* sShader;
    static bool sInitialized;
    glm::quat mInverseRotation;
    glm::vec3 mCuttingDirection;
    glm::vec3 mWorldSpacePoints[4];
    AABB mBoundingBox;

public:
    static void Init( Shader* shader );
    CuttingTool( glm::vec3 p, float _width, float _height );
    glm::vec3 GetCuttingDirection() const;
    glm::vec3 GetCorner( int index ) const;
    const AABB& GetAABB() const;
    glm::vec3 PlaneSpace( glm::vec3 value ) const;
    glm::vec3 PlaneSpaceToWorldSpace( glm::vec3 value ) const;
    bool InPlane( glm::vec3 point ) const;
    glm::vec3 LineIntersection( glm::vec3 ori, glm::vec3 end ) const;
    // 通过 SceneObject 继承
    virtual void Update() override;
    virtual void Draw() override;
private:
    void UpdateWorldSpacePoints();
};

class Scalpel
    : public SceneObject
{
public:
    enum class Status { FREE, CUTTING };
    Scalpel();
    CuttingTool GetCuttingPlane();
    void Move( glm::vec3 p, glm::vec3 dir );
    bool IsCutting() const;
    bool IsFree() const;
    // 通过 SceneObject 继承
    virtual void Update() override;
    virtual void Draw() override;

private:
    std::shared_ptr<HalfEdgeMesh> mModel = nullptr;
    Transform mLastTransform;
    Status mStatus = Status::FREE;
};