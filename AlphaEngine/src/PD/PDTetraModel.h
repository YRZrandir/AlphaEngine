#pragma once
#include <glm/glm.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "../util/Math.h"
#include "../util/SceneObject.h"
#include "../util/Intersection.h"
#include "../model/HalfEdgeMesh.h"
#include "../model/CHalfEdgeMesh.h"
#include "../model/TetraMesh.h"
#include "Constraints.h"
#include "../util/util.h"

namespace PD
{
class PDTetraModel :
    public SceneObject
{
public:
    using Real = float;
    using Vector2 = Eigen::Vector2<Real>;
    using Vector3 = Eigen::Vector3<Real>;
    using Vector4 = Eigen::Vector4<Real>;
    using Matrix3 = Eigen::Matrix3<Real>;
    using Matrix4 = Eigen::Matrix4<Real>;
    using VectorX = Eigen::VectorX<Real>;
    using MatrixX = Eigen::MatrixX<Real>;
    using Matrix3X = Eigen::Matrix3X<Real>;
    using MatrixX3 = Eigen::MatrixX3<Real>;
    using SparseMatrix = Eigen::SparseMatrix<Real>;

    struct SurfaceSkinInfo
    {
        int tid = -1;
        glm::vec4 bc;
    };
    using CSurface = CHalfEdgeMesh<>;
    PDTetraModel( const std::string& sur_path, const std::string& coarse_path, float density, std::function<bool( glm::vec3 )> attach_filter );
    void Init();

    virtual void Update() override;
    virtual void Draw() override;
    void DrawGUI();
    CSurface& Surface() { return *_surface; };

    float _stiffness = 120.0f;
    float _stiffness2 = 0.f;
    float _att_stiffness = 500.f;

private:
    void PhysicalUpdate();
    Matrix3X CollisionDetection( const Matrix3X& x );
    void CreateSurfaceMapping();
    void MapToSurface();

public:
    std::unique_ptr<TetraMesh> _mesh;
    Matrix3X _rest_pos;          //3*n

private:
    bool _simulate = false;
    float _timestep = 0.01f;
    float _density = 1.0f;
    std::function<bool( glm::vec3 )> _attach_filter;

    SparseMatrix _mass_matrix; //3n*3n
    SparseMatrix _mass_matrix_inv; //3n*3n

    Matrix3X _current_pos;       //3*n
    Matrix3X _last_pos;
    Matrix3X _current_vel;       //3*n
    Matrix3X _inertia_y;         //3*n
    Matrix3X _external_force;    //3*n
    Eigen::SimplicialLDLT<SparseMatrix> _llt;
    SparseMatrix _At;
    SparseMatrix _N;
    Matrix3X _projections;

    std::vector<std::unique_ptr<Constraint<Real>>> _constraints;

    std::unordered_map<int, Vector3> _ext_forces;

    std::unique_ptr<CSurface> _surface;
    std::string _surface_path;
    std::unique_ptr<HalfEdgeMesh> _coarse_surface;
    std::vector<SurfaceSkinInfo> _skininfos;

    bool _show_surface = true;
    bool _show_tet = true;

    //force
    float _force = 1.0f;
    int _hold_vertex = -1;
    glm::vec2 _cursor_start_pos;
    glm::vec3 _oripos;
    std::vector<std::pair<int, Vector3>> _array_ext_forces;
    std::unique_ptr<HalfEdgeMesh> _simple_ball;
    std::unique_ptr<HalfEdgeMesh> _simple_cylin;
};
}
