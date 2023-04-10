#pragma once
#include <optional>
#include <glm/glm.hpp>
#include "../model/Rect.h"
#include "../model/HalfEdgeMesh.h"
#include "../model/Transform.h"
#include "../util/SceneObject.h"
#include "../gl/VertexArray.h"
#include "../gl/VertexBuffer.h"
#include "../model/HalfEdgeSurfaceTester.h"

namespace PBD
{

class ScalpelRect :
    public SceneObject
{
public:
    ScalpelRect();
    virtual void Update() override;
    virtual void Draw() override;
    glm::vec3 GetPos0() const;
    glm::vec3 GetPos1() const;
    glm::vec3 GetLastPos0() const;
    glm::vec3 GetLastPos1() const;
    glm::vec3 GetStartPos0() const;
    glm::vec3 GetStartPos1() const;
    std::optional<Rect> GetStepRect() const;
    std::optional<Rect> GetTotalRect() const;
    bool IsCutting() const;
    void Enable();
    void Disable();

    HalfEdgeSurfaceTester tester;

private:
    void UpdateWhenIsCutting();
    void UpdateWhenNotCutting();
    void UpdateCuttingRect();

    bool _enabled = false;
    //Model
    glm::vec3 _points[2];
    std::unique_ptr<VertexArray> _vao;
    std::unique_ptr<VertexBuffer> _vbo;
    std::shared_ptr<HalfEdgeMesh> _model;
    float _speed = 0.05f;
    //CUT
    bool _is_cutting = false;
    Transform _start_trans;
    Transform _last_trans;
    std::optional<Rect> _step_rect;
    std::optional<Rect> _total_rect;

    // Inherited via SceneObject
    //virtual SceneObject* virtual_clone() const override;
};
}

