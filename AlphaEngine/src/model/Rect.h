#pragma once
#include <memory>
#include "../util/SceneObject.h"
#include "../gl/VertexArray.h"
#include "../gl/VertexBuffer.h"

class Rect :
    public SceneObject
{
public:
    static const glm::vec3 kPoints[4];
    static std::unique_ptr<VertexArray> kVAO;
    static std::unique_ptr<VertexBuffer> kVBO;

    Rect( float width, float height );
    virtual void Update() override;
    virtual void Draw() override;
    glm::vec3 RightUp() const;
    glm::vec3 LeftUp() const;
    glm::vec3 LeftDown() const;
    glm::vec3 RightDown() const;
    glm::vec3 Center() const;
    glm::vec3 Normal() const;
    bool PointInRect( glm::vec3 p ) const;
    void SetWidth( float width );
    float Width() const;
    void SetHeight( float height );
    float Height() const;
private:
    float _width = 1.f;
    float _height = 1.f;
};

