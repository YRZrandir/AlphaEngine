#pragma once
#include "../util/SceneObject.h"
#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include "../gl/VertexArray.h"
#include "../gl/VertexBuffer.h"

class Terrain :
    public SceneObject
{
private:
    std::vector<glm::vec3> mPoints;
    std::unique_ptr<VertexArray> mVAO;
    std::unique_ptr<VertexBuffer> mVBO;

public:
    Terrain( float width, float height, int res );
    ~Terrain();

    // 通过 SceneObject 继承
    virtual void Update() override;
    virtual void Draw() override;
};

