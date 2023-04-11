#pragma once
#include "PDMetaballModel.h"

namespace PD
{
class PDMetaballModelFC : public SceneObject
{
public:
    class Particle : public SphereBase
    {
    public:
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

    struct Contact
    {
        Contact( Vector3 n, Vector3 t1, int id ) : n( n ), t1( t1 ), id( id )
        {
            t2 = n.cross( t1 );
            R.col( 0 ) = t1;
            R.col( 1 ) = n;
            R.col( 2 ) = t2;
        }

        float NormalComponent( const Vector3& v ) const
        {
            return v.y();
            //return (v.transpose() * R.transpose()).dot( Vector3( 0, 1, 0 ) );
        }

        Vector3 TangentialComponent( const Vector3& v ) const
        {
            return Vector3( v.x(), 0, v.z() );
            //return v - NormalComponent( v ) * R.transpose() * Vector3( 0, 1, 0 );
        }
        Vector3 n;
        Vector3 t1;
        Vector3 t2;
        Matrix3 R;
        Vector3 p;
        int id;
    };

    PDMetaballModelFC( PDMetaballModelConfig config, PDMetaballHalfEdgeMesh* surface );
    void Init();
    void PhysicalUpdate();
    void CollisionDetection();
    void PostPhysicalUpdate();
    virtual void Update() override;
    virtual void Draw() override;
    void DrawGUI();

protected:
    void CreateSurfaceMapping();
    void UpdateSkinInfoBuffer();
    void ComputeAinvForEdgeConsts();

public:
    bool _simulate = false;

protected:
    PDMetaballModelConfig _cfg;

    std::unique_ptr<SphereMesh<Particle>> _mesh;
    Matrix3X _x0;          //3*n
    SparseMatrix _M; //3n*3n
    SparseMatrix _Minv; //3n*3n

    Matrix3X _x_last;;
    Matrix3X _x;       //3*n
    Matrix3X _v;       //3*n
    Matrix3X _pene;
    Matrix3X _friction;
    Matrix3X _momentum;         //3*n
    Matrix3X _fext;    //3*n
    Eigen::SimplicialLDLT<SparseMatrix> _llt;
    SparseMatrix _AS;
    SparseMatrix _StAt;
    SparseMatrix _C;
    SparseMatrix _P;
    Matrix3X _p;
    Matrix3X _bn_tilde;
    Matrix3X _f;
    Matrix3X _ksi;
    Matrix3X _r;
    MatrixX _J;

    SparseMatrix _Dinv;
    SparseMatrix _LU;
    SparseMatrix _B;

    std::unique_ptr<HalfEdgeMesh> _coarse_surface;
    PDMetaballHalfEdgeMesh* _surface;
    std::vector<VtxSkinningInfo>        _vtx_skinning_table;
    std::vector<BallSkinningInfo>       _ball_skinning_infos;
    std::unique_ptr<ShaderStorageBuffer> _skin_vtx_buffer;
    std::unique_ptr<ShaderStorageBuffer> _skin_ball_buffer;

    std::vector<std::unique_ptr<Constraint>> _constraints;
    std::vector<Matrix3> _Ainv_for_edge_consts;
    std::vector<std::vector<float>> _weights_for_edge_consts;

    std::unordered_set<int> _select_balls;
    std::vector<int> _attached_balls;

    int _hold_idx = -1;
    glm::vec2 _init_cursor;
    std::unordered_map<int, Vector3> _ext_forces;

    bool _show_surface = true;
    bool _show_balls = false;
    bool _show_contacts = false;

    AABB _aabb;

    std::vector<Contact> _contacts;
    std::unique_ptr<GLLineSegment> _contacts_vis;
};

}