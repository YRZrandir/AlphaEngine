#pragma once
#include <array>
#include <glm/glm.hpp>
#include <vector>
#include <memory>
#include "acceleration/BVH.h"
#include "free_cutting/FreeCuttingManager.h"
#include "gl/ShaderStorageBuffer.h"
#include "util/SceneObject.h"
#include "model/HalfEdgeMesh.h"
#include "model/HalfEdgeSurfaceTester.h"
#include "model/Rect.h"
#include "model/GLLineSegment.h"
#include "util/Instrumentor.h"
#include "Scalpel.h"
#include "PBDTetraModel.h"

namespace PBD
{
class Metaball
{
public:
    glm::vec3   x = glm::vec3( 0.f );
    glm::vec3   x0 = glm::vec3( 0.f );
    glm::vec3   x_pred = glm::vec3( 0.f );
    glm::quat   q = glm::quat_identity<float, glm::qualifier::packed_highp>();
    glm::quat   q_pred = glm::quat_identity<float, glm::qualifier::packed_highp>();
    glm::vec3   v = glm::vec3( 0.f );
    glm::vec3   v_pred = glm::vec3( 0.f );
    glm::vec3   w = glm::vec3( 0.f );
    glm::vec3   w_pred = glm::vec3( 0.f );
    float       r = 0.f;
    float       m = 0.f;
    bool        valid = true;
    bool        visible = true;
    std::vector<int> neighbors;
    glm::vec3   color = glm::vec3( 0.9f ); //only for test

    Metaball() = default;

    Metaball( glm::vec3 c_, float r_ )
        :x( c_ ), r( r_ ), x0( c_ ), x_pred( c_ )
    {
    }

    class Hash
    {
    public:
        size_t operator()( const Metaball& b ) const
        {
            return std::hash<float>()(b.x.x) ^ std::hash<float>()(b.x.y) ^ std::hash<float>()(b.x.z);
        }
    };

    class Pred
    {
    public:
        bool operator()( const Metaball& lh, const Metaball& rh ) const
        {
            return lh.x == rh.x;
        }
    };
};

class IdxPair
{
public:
    int i0;
    int i1;

    class Hash
    {
    public:
        unsigned operator()( const IdxPair& info ) const
        {
            std::hash<int>* p = nullptr;
            return p->operator()( info.i0 ) ^ p->operator()( info.i1 );
        }
    };

    class Pred
    {
    public:
        bool operator()( const IdxPair& info1, const IdxPair& info2 ) const
        {
            return ((info1.i0 == info2.i0 && info1.i1 == info2.i1) ||
                (info1.i0 == info2.i1 && info1.i1 == info2.i0));
        }
    };
};
class MetaballTreeNode
{
public:
    std::array<MetaballTreeNode*, 8> children;
    Metaball metaball;

    MetaballTreeNode( const std::vector<Metaball>& ball_list, int idx );
    ~MetaballTreeNode();
    void GetLeafNodes( std::vector<Metaball>& balls );
};

class LaplacianConstraint
{
public:
    int index;
    glm::vec3 L;
};

class BallCollisionConstraint
{
public:
    class Hash
    {
    public:
        size_t operator()( const BallCollisionConstraint& c ) const
        {
            return std::hash<size_t>()((size_t)(c.ball1)) ^ std::hash<size_t>()((size_t)(c.ball2));
        }
    };

    class Pred
    {
    public:
        bool operator()( const BallCollisionConstraint& lh, const BallCollisionConstraint& rh ) const
        {
            return (lh.ball1 == rh.ball1 && lh.ball2 == rh.ball2) || (lh.ball1 == rh.ball2 && lh.ball2 == rh.ball1);
        }
    };
    Metaball* ball1;
    Metaball* ball2;
};

class MeshlessPoint
{
public:
    glm::vec3 p;
    float type = 0;   //0=normal, 1=border, 2=sweepface
    glm::vec3 normal = glm::vec3( 0.f );
    glm::vec3 v = glm::vec3( 0.f );
    float m = 0.f;
    int ball_id = -1;

    MeshlessPoint( glm::vec3 p, int type, int id )
        :p( p ), type( type ), ball_id( id )
    {

    }
};

struct BoundaryPoint
{
    glm::vec3 pos;
    glm::vec3 normal;
    int type;
};



class MetaballModel
    : public SceneObject
{
public:
    static const int NEICOUNT = 4;

    struct VtxSkinningInfo
    {
        std::array<int, NEICOUNT> indices;
        std::array<float, NEICOUNT> weights;
        glm::vec3 disp;
    };

    struct BallSkinningInfo
    {
        glm::vec4 x;
        glm::quat q;
    };

    enum class Mode { None, Hand, Scalpel };

    MetaballModel( const std::vector<Metaball>& metaball_list, std::unique_ptr<MetaballHalfEdgeMesh>&& surface, float density );
    MetaballModel( const std::string& surface_path, const std::string& coarse_path, float density, int lloyd, int num );
    MetaballModel( const std::string& path, const std::string& surface_path, float density );
    virtual void Update() override;
    virtual void Draw() override;
    void DrawGUI();
    void SampleFromSurface();
    void CreateSurfaceMapping();
    void CreateSurfaceMapping( const Rect& rect );
    void CreateSurfaceMapping2();
    void CreateSurfaceMapping2( const Rect& rect );
    Metaball& Ball( int handle );
    std::vector<Metaball>& BallList();
    HalfEdgeMesh& Surface();
    const BVHTree_Metaball& BVH() const;
    void CreateTopo();
    void CreateTopo( const Rect& rect );
    void InitVertices();
    void InitConstraints();

    //Config
    void SetHandMode();
    void SetScalpelMode();
    void SetNoneMode();
    void ShowMetaball( bool value );
    void ShowSurface( bool value );
    void ShowOrit( bool value );
    bool ShowMetaball() const;
    bool ShowSurface() const;
    bool ShowOrit() const;
    void SetStretchStiff( float value );
    void SetDampCoeff( float value );
    void SwitchPointDamping( bool open );
    void SwitchLineDamping( bool open );
    void SwitchTotalDamping( bool open );
    void SwitchHangUp( bool value );
    void SwitchSimulation( bool value );
    void SetShowRestPos( bool value );
    float GetStretchSitff() const;
    float GetDampCoeff() const;
    bool IsPointDamping() const;
    bool IsLineDamping() const;
    bool IsTotalDamping() const;
    bool IsHangUp() const;
    bool IsSimulating() const;

    //Simulation
    void PBDPrediction( float dt );
    void PBDCheckCollision( float dt );
    void PBDSolveConstraints( float dt );
    void CheckCursorRay( Ray r );
    void MapToSurface();
    void MapToSurface2();
    void UpdateSkinInfoBuffer();

private:
    void Preprocess();
    void RadiusAdjustment();
    void VacantSpaceFilling();
    void SphereMerging();
    void ElectroAttractOptimize();
    std::vector<glm::vec3>  Voxelize();
    std::vector<glm::vec3>  GetHollowVoxel( const std::vector<glm::vec3>& voxels );
    std::vector<int>        Neighbors( int metaball, int k ) const;

    void PhysicalUpdate( float dt );

    //cutting
    std::vector<MeshlessPoint> SamplePointsInBall( int ball_id, float step = 0.01f );
    std::vector<BoundaryPoint> SamplePointsInTriangle( glm::vec3& a, glm::vec3& b, glm::vec3& c ) const;

private:
    float _density;
    float _show_threshold = -10.f;
    bool  _show_surface = true;
    bool  _show_metaball = false;
    bool  _show_q = false;
    bool  _simulation = false;
    bool  _point_damping = false;
    bool  _line_damping = true;
    bool  _total_damping = false;
    bool  _hang_up = true;
    bool  _show_restpos = false;

    float _stretch_stiffness = 0.3f;
    float _laplacian_stiffness = 0.05f;
    float _pull_force = 10.0f;
    float _damping_coff = 1.0f;

    int _hold_idx = -1;
    glm::vec2 _start_pos;
    glm::vec3 _hold_dist;

    bool _is_cutting = false;
    Mode _mode = Mode::Hand;

    std::unique_ptr<HalfEdgeMesh>       _ball_mesh;
    std::unique_ptr<HalfEdgeMesh>       _hand1;
    std::unique_ptr<HalfEdgeMesh>       _hand2;

    std::vector<Metaball>               _metaballs;
    std::unique_ptr<GLLineSegment>      _ball_orit_lines = nullptr;
    std::unique_ptr<BVHTree_Metaball>   _bvh = nullptr;

    std::unique_ptr<MetaballHalfEdgeMesh> _surface = nullptr;
    std::unique_ptr<HalfEdgeMesh> _coarse_surface = nullptr;
    HalfEdgeSurfaceTester               _surface_tester;
    std::vector<std::vector<int>>       _attach_ball_table;
    std::vector<VtxSkinningInfo>        _vtx_skinning_table;
    std::vector<BallSkinningInfo>       _ball_skinning_infos;
    std::unique_ptr<ShaderStorageBuffer> _skin_vtx_buffer;
    std::unique_ptr<ShaderStorageBuffer> _skin_ball_buffer;

    std::unordered_map<int, glm::vec3>  _extra_force;
    std::unordered_set<StretchConstraint, StretchConstraint::Hash, StretchConstraint::Pred> _stretch_consts;
    std::vector<CollisionConstraint>    _colli_consts;
    std::vector<LaplacianConstraint>    _laplacian_consts;
    std::vector<AttachmentConstraint>   _attach_consts;
    std::unordered_set<BallCollisionConstraint, BallCollisionConstraint::Hash, BallCollisionConstraint::Pred> _ball_colli_consts;
};
}

