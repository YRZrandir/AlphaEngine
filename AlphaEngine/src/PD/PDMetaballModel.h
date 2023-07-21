#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "util/SceneObject.h"
#include "util/Intersection.h"
#include "model/HalfEdgeMesh.h"
#include "model/SphereMesh.h"
#include "model/GLLineSegment.h"
#include "Constraints.h"
#include "acceleration/AABB.h"


namespace PD
{
struct PDMetaballModelConfig
{
    int _method = 0; //0=optimization, 1=uniform, 2=sphere-tree
    std::string _coarse_surface;//A triangle mesh when _method==0 or 1
    std::string _metaball_path;//only when method == 2
    float _sample_dx = 0.05f; //only used when _method==0 or 1
    float _density = 1.0f;
    float _k_stiff = 100.0f;
    float _k_attach = 100.0f;
    float _k_edge_stiff = 0.8f;
    float _force = 100.f;
    float _dt = 0.033f; // time for a physical step
    int _nb_solve = 5;
    int _nb_points = 100; //only used when _method==0;
    int _nb_lloyd = 5; //only used when _method==0;
    std::string _attach_points_filter;
    int _const_type = 0; //0=meshless, 1=edge
    glm::vec3 _displacement = glm::vec3( 0.f );
    bool _newton = false;
};

class PDMetaballModel :
    public SceneObject
{
public:
    using Real = double;
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

    class Particle : public SphereBase
    {
    public:
        //Inherit From SphereBase
        /*
            bool deleted = false;
            glm::vec3 x = glm::vec3( 0.f );
            glm::vec3 x0 = glm::vec3( 0.f );
            glm::quat q;
            glm::quat q0;
            float r = 0.f;
            float m = 0.f;
            glm::vec3 color = glm::vec3( 0.9f );
            std::vector<int> neighbors;
        */
        glm::vec3 v = glm::vec3( 0.f );
        glm::vec3 f = glm::vec3( 0.f );
        float rc = 0.f;
        bool isborder = false;
        float m_rel = 1.f;
        Matrix3 gu = Matrix3::Identity();
        Matrix3 R = Matrix3::Identity();
        glm::vec3 outside = glm::vec3( 0.f );
        float sdf = 0.f;
    };

    static const int NEICOUNT = 10;
    struct VtxSkinningInfo
    {
        std::array<int, NEICOUNT> indices;
        std::array<float, NEICOUNT> weights;
    };
    struct BallSkinningInfo
    {
        glm::vec4 u = glm::vec4( 0.f );
        glm::vec4 xi;
        glm::mat4 gu = glm::mat4( 1.0f );
        glm::mat4 R = glm::mat4( 1.0f );
    };

    PDMetaballModel( const PDMetaballModelConfig& cfg, PDMetaballHalfEdgeMesh* mesh );
    PDMetaballModel( const PDMetaballModelConfig& cfg, const SphereMesh<Particle>& balls, PDMetaballHalfEdgeMesh* mesh );
    virtual ~PDMetaballModel() override;
    void Init();

    virtual void Update() override;
    virtual void Draw() override;
    void DrawGUI();
    PDMetaballHalfEdgeMesh& Surface() { return *_surface; };
    PDMetaballModelConfig& Config() { return _cfg; };
    void SetSurface( PDMetaballHalfEdgeMesh* mesh ) { _surface = mesh; };

    void PhysicalUpdate();
    void CollisionDetection();
    void PostPhysicalUpdate();

private:
    void CreateSurfaceMapping();
    void UpdateSkinInfoBuffer();
    void MapSurface();
    void SampleFromVoxel( float steplen );
    void ComputeBallOrit();
    void ComputeBallOrit2();
    void ComputeAinvForEdgeConsts();
    void ComputeBallOutsideVec();

public:
    std::unique_ptr<SphereMesh<Particle>> _mesh;
    Matrix3X _x0;          //3*n
    bool _simulate = false;

private:
    PDMetaballModelConfig _cfg;
    SparseMatrix _mass_matrix; //3n*3n
    SparseMatrix _mass_matrix_inv; //3n*3n

    Matrix3X _x_last;
    Matrix3X _x;       //3*n
    Matrix3X _v;       //3*n
    Matrix3X _pene;
    Matrix3X _friction;
    Matrix3X _momentum;         //3*n
    Matrix3X _external_force;    //3*n
    Eigen::SimplicialLDLT<SparseMatrix> _llt;
    SparseMatrix _AS;
    SparseMatrix _StAt;
    SparseMatrix _P;
    Matrix3X _p;

    SparseMatrix _StAtAS;
    SparseMatrix _J;
    Matrix3X _g;
    std::vector<SparseMatrix> _CStAtAS;
    std::vector<SparseMatrix> _CStAt;
    std::vector<SparseMatrix> _CAS;
    Eigen::SimplicialLDLT<SparseMatrix> _newtonllt;


    Eigen::Vector3f _err = Eigen::Vector3f( -1.f, -1.f, -1.f );

    std::vector<std::unique_ptr<Constraint<double>>> _constraints;
    std::vector<Matrix3> _Ainv_for_edge_consts;
    std::vector<std::vector<float>> _weights_for_edge_consts;

    int _hold_idx = -1;
    glm::vec2 _init_cursor;
    std::unordered_map<int, Vector3> _ext_forces;
    std::unordered_set<int> _select_balls;
    std::vector<int> _attached_balls;
    std::vector<std::pair<int, Eigen::Vector3<float>>> _array_ext_forces;
    std::unique_ptr<HalfEdgeMesh> _simple_ball;
    std::unique_ptr<HalfEdgeMesh> _simple_cylin;

    std::string _surface_path;
    PDMetaballHalfEdgeMesh* _surface;
    std::unique_ptr<HalfEdgeMesh> _coarse_surface;
    std::vector<VtxSkinningInfo>        _vtx_skinning_table;
    std::vector<BallSkinningInfo>       _ball_skinning_infos;
    std::unique_ptr<ShaderStorageBuffer> _skin_vtx_buffer;
    std::unique_ptr<ShaderStorageBuffer> _skin_ball_buffer;

    bool _show_surface = true;
    bool _show_balls = false;
    std::unique_ptr<GLLineSegment> _line_segments;

    AABB _aabb;

    glm::vec3 _color;
};
}
