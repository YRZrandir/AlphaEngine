#pragma once
#include <initializer_list>
#include <memory>
#include <vector>
#include <mutex>
#include "../acceleration/RayIntersectSolver.h"
#include "../model/HalfEdgeMesh.h"
#include "../model/TetraMesh.h"
#include "../util/SceneObject.h"
#include "../free_cutting/FreeCuttingManager.h"
#include "../PBD/Scalpel.h"
#include "../PBD/Hand.h"

namespace PBD
{
class StretchConstraint
{
public:
    struct Hash
    {
        size_t operator()( const StretchConstraint& sc ) const
        {
            return (size_t)sc._indices[0] + (size_t)sc._indices[1];
        }
    };
    struct Pred
    {
        bool operator()( const StretchConstraint& lhs, const StretchConstraint& rhs ) const
        {
            return lhs._indices[0] == rhs._indices[0] && lhs._indices[1] == rhs._indices[1]
                || lhs._indices[0] == rhs._indices[1] && lhs._indices[1] == rhs._indices[0];
        }
    };

    int _indices[2];
    float _k = 1.0f;
    float _d = 0.0f;

    StretchConstraint( int i1, int i2, float k, float d )
        :_d( d ), _k( k )
    {
        _indices[0] = i1;
        _indices[1] = i2;
    }
};

class EdgeConstraint
{
public:
    EdgeConstraint( int i0, int i1, float k, float d )
        :_i0( i0 ), _i1( i1 ), _k( k ), _d( d )
    {

    }

    int _i0;
    int _i1;
    float _k;
    float _d;
};

class VolumeConservConstraint
{
public:
    int _indices[4];
    float _k = 1.0f;
    float _V = 0.0f;

    VolumeConservConstraint( int i1, int i2, int i3, int i4, float k, float V )
        :_k( k ), _V( V )
    {
        _indices[0] = i1;
        _indices[1] = i2;
        _indices[2] = i3;
        _indices[3] = i4;
    }
};

class CollisionConstraint
{
public:
    int _index;
    glm::vec3 _pc;
    glm::vec3 _n;
    float _depth;
    float _m;   //the other particle's mass
    CollisionConstraint( int index, glm::vec3 pc, glm::vec3 n, float depth = 0.f, float m = FLT_MAX )
        :_index( index ), _pc( pc ), _n( n ), _depth( depth ), _m( m )
    {
    }
};

class ShapeMatchConstraint
{
public:
    float _k = 1.0f;
    glm::vec3 _Xcm;
    glm::mat3 _As;

    ShapeMatchConstraint() = default;

    ShapeMatchConstraint( glm::vec3 Xcm, glm::mat3 As, float k )
        :_Xcm( Xcm ), _As( As ), _k( k )
    {
    }
};

class AttachmentConstraint
{
public:
    int _index = -1;
    glm::vec3 _p;

    AttachmentConstraint( int index, glm::vec3 p )
        :_index( index ), _p( p )
    {
    }
};

class FEMConstraint
{
public:
    int _indices[4];
    glm::mat3 _inv_rest;
    glm::mat4x3 _grad;
    float _lambda = 0.f;
public:
    FEMConstraint( int i0, int i1, int i2, int i3, glm::mat3 inv_rest )
    {
        _indices[0] = i0;
        _indices[1] = i1;
        _indices[2] = i2;
        _indices[3] = i3;
        _inv_rest = inv_rest;
    }
};


class PBDTetraModel :
    public SceneObject
{
public:
    PBDTetraModel( const std::string& path, const std::string& surface, float density );
    virtual ~PBDTetraModel() = default;
    void PhysicalUpdate( float dt );

    HalfEdgeMesh* GetSurface();
    void MapToSurface();
    void InitSurfaceMapping( bool all = false );
    void SetCutMode();
    void SetHandMode();
    void SetStretchStiffness( float value );
    void SetVolumeStiffness( float value );
    const std::vector<glm::vec3>& Points() const;
    const TetraMesh* Mesh() const;
    void AddExternalForce( int i, glm::vec3 force );
    // Inherited via SceneObject
    virtual void Update() override;
    virtual void Draw() override;

    std::unordered_map<int, glm::vec3> _external_forces;
    std::mutex _external_forces_lock;
private:
    float _density;
    std::shared_ptr<HalfEdgeMesh>   _surface;
    std::shared_ptr<TetraMesh>      _tet_mesh;
    std::unique_ptr<RayIntersectSolver> _intersect_solver;
    std::unique_ptr<FreeCuttingManager> _cutting_manager;

    std::unique_ptr<Scalpel> _scalpel;
    std::unique_ptr<Hand> _hand;

    std::vector<glm::vec3>      _position_list;
    std::vector<glm::vec3>      _rest_position;
    std::vector<glm::vec3>      _velocity_list;
    std::vector<glm::vec3>      est_positions;
    std::vector<glm::vec3>      est_velocities;
    std::vector<float>          _mass_list;

    std::vector<std::pair<int, glm::vec4>>  _pos_mapping;

    std::vector<StretchConstraint>          _stretch_constraints;
    std::vector<VolumeConservConstraint>    _volume_constraints;
    std::vector<CollisionConstraint>        _collision_constraints;
    ShapeMatchConstraint                    _shape_match_constraint;
    std::vector<FEMConstraint> _fem_constraints;

    //Constraints Visualization
    std::vector<glm::vec3> _stretch_vis_buffer;
    std::unique_ptr<VertexBuffer> _stretch_vis_vbo;
    std::unique_ptr<VertexArray> _stretch_vis_vao;

    int _mode = 1;//0 for cut, 1 for drag
    std::vector<int> _anchors;
    std::vector<unsigned> _hold_vertex;
    glm::vec3 _orinormal;
    std::vector<glm::vec3> _oripos;
    glm::vec2 _cursor_start_pos;

    std::vector<unsigned> _press_vertex;
    glm::vec3 _press_force;

    bool _simulate = false;
    float _stretch_stiffness = 0.1f;
    float _volume_stiffness = 0.1f;

    void TetMeshPreprocess();
    void InitParticles();
    void InitConstraints();

};

}

