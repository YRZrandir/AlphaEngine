#pragma once
#include "../util/SceneObject.h"
#include "../model/HalfEdgeMesh.h"

class Hand :
    public SceneObject
{
public:
    Hand();

    // Inherited via SceneObject
    virtual void Update() override;

    virtual void Draw() override;

private:
    std::shared_ptr<HalfEdgeMesh> _hand1;
    std::shared_ptr<HalfEdgeMesh> _hand2;
};

