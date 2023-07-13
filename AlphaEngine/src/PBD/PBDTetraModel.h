#pragma once
#include <initializer_list>
#include <memory>
#include <vector>
#include <mutex>
#include "acceleration/RayIntersectSolver.h"
#include "model/HalfEdgeMesh.h"
#include "model/TetraMesh.h"
#include "util/SceneObject.h"
#include "free_cutting/FreeCuttingManager.h"
#include "PBD/Scalpel.h"
#include "PBD/Hand.h"
#include "PBDConstraints.h"

namespace PBD
{
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

