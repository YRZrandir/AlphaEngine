#pragma once
#include <memory>
#include <vector>
#include "../model/Triangle.h"
#include "../util/SceneObject.h"
#include "../gl/VertexArray.h"
#include "../gl/VertexBuffer.h"
#include "../model/HalfEdgeMesh.h"

namespace PBD
{
    class Scalpel :
        public SceneObject
    {
    public:
        Scalpel();

        glm::vec3 GetStartPos() const;
        glm::vec3 GetEndPos() const;
        glm::vec3 GetLastStartPos() const;
        glm::vec3 GetLastEndPos() const;
        std::pair<TriPoint, TriPoint> GetSweepFace() const;
        virtual void Update() override;
        virtual void Draw() override;
    private:
        glm::vec3 _points[2];
        Transform _last_trans;

        std::unique_ptr<VertexArray> _vao;
        std::unique_ptr<VertexBuffer> _vbo;
        std::shared_ptr<HalfEdgeMesh> _model;
    };
}

