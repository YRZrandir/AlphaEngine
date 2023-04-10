#pragma once
#include "util/SceneObject.h"
#include <Eigen/Core>
#include "model/HalfEdgeMesh.h"
#include "model/TetraMesh.h"
#include "PBDTetraModel.h"
#include "CudaPbdSolver.cuh"

namespace PBD
{


class GPUTetraModel : public SceneObject
{
public:
    GPUTetraModel( const std::string& path, const std::string& surface, float density );
    virtual void Update() override;
    virtual void Draw() override;

private:
    void ExtrudeTetMesh();
    void InitParticles();
    void InitConstraints();
    void InitCudaBuffer();
    void PhysicalUpdate();

private:
    float _density;

    Eigen::Matrix3Xf _x0;
    Eigen::Matrix3Xf _x;
    Eigen::Matrix3Xf _p;
    Eigen::Matrix3Xf _v;
    Eigen::Matrix3Xf _f;
    Eigen::VectorXf  _m;
    Eigen::VectorXf  _invm;

    std::vector<EdgeConstraint>          _stretch_constraints;
    std::vector<VolumeConservConstraint>    _volume_constraints;
    std::vector<AttachmentConstraint>       _attach_constraints;
    std::vector<CollisionConstraint>        _collision_constraints;

    std::shared_ptr<HalfEdgeMesh>   _surface;
    std::shared_ptr<TetraMesh>      _tet_mesh;

    PBDSystem _sys;

    bool _simulate = false;
};
}