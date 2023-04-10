#pragma once
#include <iostream>
#include <sdfgen/makelevelset3.h>
#include "util/SceneObject.h"
#include "model/CHalfEdgeMesh.h"

class RigidSDF : public SceneObject
{
public:
    RigidSDF( const std::string& path, float dx );
    bool CheckPoint( glm::vec3 p, glm::vec3* n, float* depth ) const;
    bool CheckBall( glm::vec3 c, float r, glm::vec3* n, float* depth ) const;
    virtual void Update() override;
    virtual void Draw() override;

private:
    using CMesh = CHalfEdgeMesh<>;
    SDF _sdf;
    std::unique_ptr<CMesh> _mesh;
};