#ifndef GLLINESEGMENT_H
#define GLLINESEGMENT_H
#include <memory>
#include <vector>
#include "glm/glm.hpp"
#include "../util/SceneObject.h"
#include "../gl/VertexArray.h"
#include "../gl/VertexBuffer.h"

class GLLineSegment : public SceneObject
{
public:
    struct Point
    {
        glm::vec3 p;
        glm::vec3 color;
    };

    GLLineSegment();
    void AddPoint( glm::vec3 p, glm::vec3 color = glm::vec3( 1, 0, 0 ) );
    void ReservePoint( int count );
    void Clear();
    void UpdateMem();
    virtual void Update() override;
    virtual void Draw() override;

private:
    std::vector<Point> _mem;
    std::unique_ptr<VertexArray> _vao = nullptr;
    std::unique_ptr<VertexBuffer> _vbo = nullptr;

};
#endif