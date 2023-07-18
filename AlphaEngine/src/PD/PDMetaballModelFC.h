#pragma once
#include "PDMetaballModel.h"
#include "ECS/ECS.h"

namespace PD
{
class SpatialHash;

class PDMetaballModelFC : public SceneObject, public Component
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

    class Particle : public SphereBase
    {
    public:
        Vector3 x_last;
        glm::vec3 v = glm::vec3( 0.f );
        glm::vec3 f = glm::vec3( 0.f );
        float rc = 0.f;
        bool isborder = false;
        float m_rel = 1.f;
        Matrix3 gu = Matrix3::Identity();
        Matrix3 R = Matrix3::Identity();
        glm::vec3 outside = glm::vec3( 0.f );
        float sdf = 0.f;
        PDMetaballModelFC* pmodel;
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
        Contact( Vector3 n, Vector3 t1, int id, int type ) : n( n ), t1( t1 ), id( id ), type( type )
        {
            t2 = n.cross( t1 ).normalized();
            R.col( 0 ) = t1;
            R.col( 1 ) = n;
            R.col( 2 ) = t2;
        }

        Contact( Vector3 n, Vector3 t1, int id, int id2, int type ) : n( n ), t1( t1 ), id( id ), id2( id2 ), type( type )
        {
            t2 = n.cross( t1 ).normalized();
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
        Vector3 uf;
        int id;
        int id2;
        int type = 0;
    };

    friend class PBDScene;

    PDMetaballModelFC( PDMetaballModelConfig config, PDMetaballHalfEdgeMesh* surface );
    virtual void Start() override;
    void Init();
    void PhysicalUpdate();
    void UpdateSn();
    void PDSolve();
    void CollisionDetection( SpatialHash* table );
    void PostPhysicalUpdate();
    virtual void Update() override;
    virtual void DrawShadowDepth() override;
    virtual void Draw() override;
    virtual void DrawGUI() override;
    SphereMesh<Particle>& GetMetaballModel();
    const SphereMesh<Particle>& GetMetaballModel() const;
    const ShaderStorageBuffer& GetVtxSkinBuffer() const;
    const ShaderStorageBuffer& GetBallSkinBuffer() const;
    void UpdateSkinInfoBuffer();

protected:
    void CreateSurfaceMapping();
    void ComputeAinvForEdgeConsts();

public:
    bool _simulate = false;

protected:
    friend class PD::SpatialHash;
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
    glm::vec3 _color;
    std::vector<VtxSkinningInfo>        _vtx_skinning_table;
    std::vector<BallSkinningInfo>       _ball_skinning_infos;
    std::unique_ptr<ShaderStorageBuffer> _skin_vtx_buffer;
    std::unique_ptr<ShaderStorageBuffer> _skin_ball_buffer;

    std::vector<std::unique_ptr<Constraint<Real>>> _constraints;
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
    std::vector<std::vector<int>> _contact_bylayer;
    std::vector<int> _contact_counter;
    std::unique_ptr<GLLineSegment> _contacts_vis;
};

class SpatialHash
{
    using Sphere = PDMetaballModelFC::Particle;
    using GridPos = std::tuple<int, int, int>;
    struct GridHash
    {
        size_t operator()( const GridPos& p ) const
        {
            return std::abs( std::get<0>( p ) * 73856093 ^ std::get<1>( p ) * 19349663 ^ std::get<2>( p ) * 83492791 );
        }
    };
    struct GridPred
    {
        bool operator()( const GridPos& p1, const GridPos& p2 ) const
        {
            return std::get<0>( p1 ) == std::get<0>( p2 ) && std::get<1>( p1 ) == std::get<1>( p2 ) && std::get<2>( p1 ) == std::get<2>( p2 );
        }
    };

public:
    SpatialHash( float dx );
    void Insert( Sphere* s );
    void Clear();
    std::vector<Sphere*> CheckIntersection( Sphere* s ) const;
    void SetDx( float dx );

protected:
    int GridCoord( float p ) const;

protected:
    float _dx;
    std::unordered_map<GridPos, std::vector<Sphere*>, GridHash, GridPred> _table;

};

}