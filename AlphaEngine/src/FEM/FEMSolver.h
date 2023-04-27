#pragma once
#include <mutex>
#include <glm/glm.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "../util/SceneObject.h"
#include "../util/Intersection.h"
#include "../model/HalfEdgeMesh.h"
#include "../model/TetraMesh.h"
#include "../util/util.h"

struct FEMConfig
{
    std::string path;
    std::string surface_path;
    float density = 1.f;
    float dt = 0.03f;
    int substep = 10;
    float mu = 580.f;
    float lambda = 380.f;
    bool show_tet = false;
    bool show_surface = true;
    bool simulate = false;
};

class FEMSolver : public SceneObject
{
public:
    FEMSolver( FEMConfig config );
    void PhysicalUpdate();
    void Update();
    void Draw();
    void DrawGUI();

    FEMConfig _config;
    std::unique_ptr<TetraMesh> _mesh;
    std::unique_ptr<HalfEdgeMesh> _surface;
    Eigen::Matrix3Xf _x;
    Eigen::Matrix3Xf _x0;
    Eigen::Matrix3Xf _v;
    Eigen::Matrix3Xf _f;
    Eigen::VectorXf  _m;
    Eigen::VectorXf  _w;
    std::vector<Eigen::Matrix3f> _invDm;
    std::vector<Eigen::Matrix3f> _Hs;
private:
    struct SkinInfo
    {
        int tid = -1;
        Eigen::Vector4f bc;
    };
    void ExtrudeTetMesh( float d );
    void CreateSurfaceMapping();
    void MapToSurface();

    std::vector<SkinInfo> _skininfos;
    int _hold_vertex = -1;
    glm::vec2 _cursor_start_pos;
    glm::vec3 _oripos;
};