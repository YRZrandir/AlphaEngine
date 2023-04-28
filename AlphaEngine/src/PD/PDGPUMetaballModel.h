#pragma once
#include "PDMetaballModel.h"
#include "util/CudaMatrixHelpers.h"
#include "CudaPDSolver.cuh"

namespace PD
{

class PDGPUMetaballModel :
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


    PDGPUMetaballModel( const PDMetaballModelConfig& cfg, PDMetaballHalfEdgeMesh* mesh );
    virtual ~PDGPUMetaballModel() override;
    void Init();

    virtual void Update() override;
    virtual void Draw() override;
    void DrawGUI();
    PDMetaballHalfEdgeMesh& Surface() { return *_surface; };
    PDMetaballModelConfig& Config() { return _cfg; };
    void SetSurface( PDMetaballHalfEdgeMesh* mesh ) { _surface = mesh; };

    void PhysicalUpdate();
    void CudaPhysicalUpdate();
    void CollisionDetection();
    void PostPhysicalUpdate();
    float Compare( const PDGPUMetaballModel* other );

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
    Matrix3X _rest_pos;          //3*n
    bool _simulate = false;

private:
    PDMetaballModelConfig _cfg;
    SparseMatrix _mass_matrix; //3n*3n
    SparseMatrix _mass_matrix_inv; //3n*3n
    Matrix3X _last_pos;
    Matrix3X _last_pos1;
    Matrix3X _last_pos2;
    Matrix3X _x;       //3*n
    Matrix3X _v;       //3*n
    Matrix3X _pene;
    Matrix3X _friction;
    Matrix3X _momentum;         //3*n
    Matrix3X _f_ext;    //3*n
    std::vector<std::unique_ptr<Constraint<Real>>> _constraints;
    SparseMatrix _AS;
    SparseMatrix _StAt;
    SparseMatrix _N;
    Matrix3X _p;
    Eigen::SimplicialLDLT<SparseMatrix> _llt;

    SparseMatrix _StAtAS;
    SparseMatrix _J;
    Matrix3X _g;
    std::vector<SparseMatrix> _CStAtAS;
    std::vector<SparseMatrix> _CStAt;
    std::vector<SparseMatrix> _CAS;
    Eigen::SimplicialLDLT<SparseMatrix> _newtonllt;

    SparseMatrix _Dinv;
    SparseMatrix _LU;
    SparseMatrix _B;
    CudaPDSystem _cudapd;
    cusparseSpMatDescr_t _d_At;
    std::array<CudaBuffer<Real>, 3> _d_q;
    std::array<cusparseDnVecDescr_t, 3> _d_proj;
    std::array<CudaBuffer<Real>, 3> _d_proj_buf;
    std::array<cusparseDnVecDescr_t, 3> _d_rhvec;
    std::array<CudaBuffer<Real>, 3> _d_rhvec_buf;
    std::vector<CudaAttachConst> _host_attach_consts;
    CudaBuffer<CudaAttachConst> _cuda_attach_consts;
    std::vector<CudaMetaballConst> _host_metaball_consts;
    CudaBuffer<CudaMetaballConst> _cuda_metaball_consts;
    std::vector<CudaEdgeConst> _host_edge_consts;
    CudaBuffer<CudaEdgeConst> _cuda_edge_consts;
    std::vector<CudaMetaballConstNeiInfo> _host_metaball_neiinfos;
    CudaBuffer<CudaMetaballConstNeiInfo> _cuda_metaball_neiinfos;
    CudaBuffer<CudaSkinningInfo> _cuda_skinning_info;

    float _rho = 0.99;
    cusparseSpMatDescr_t _Jacobi_B;
    cusparseSpMatDescr_t _Jacobi_Dinv;
    std::array<CudaBuffer<unsigned char>, 3> _Spmv_buf;
    std::array<cusparseDnVecDescr_t, 3> _Jacobi_x;
    std::array<CudaBuffer<Real>, 3>    _Jacobi_x_buf;
    std::array<cusparseDnVecDescr_t, 3> _Jacobi_y;
    std::array<CudaBuffer<Real>, 3>    _Jacobi_y_buf;
    std::array<cusparseDnVecDescr_t, 3> _Jacobi_Dinvb;
    std::array<CudaBuffer<Real>, 3>    _Jacobi_Dinvb_buf;
    std::array<cusparseDnVecDescr_t, 3> _Jacobi_b;
    std::array<CudaBuffer<Real>, 3>    _Jacobi_b_buf;

    std::vector<Matrix3> _Ainv_for_edge_consts;
    std::vector<std::vector<float>> _weights_for_edge_consts;

    int _hold_idx = -1;
    glm::vec2 _init_cursor;
    std::unordered_map<int, Vector3> _ext_forces;
    std::unordered_set<int> _select_balls;
    std::vector<int> _attached_balls;
    std::vector<std::pair<int, Eigen::Vector3f>> _array_ext_forces;
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
