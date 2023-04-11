#include "PDMetaballModel.h"
#include <iomanip>
#include <imgui/imgui.h>
#include <tinycolormap.hpp>
#include <omp.h>
#include <cstdlib>

#include "CVT/WeightedCVT.h"
#include "input/Input.h"
#include "model/HalfEdgeMesh.h"
#include "model/HalfEdgeSurfaceTester.h"
#include "model/RigidStatic.h"
#include "model/RigidSDF.h"
#include "util/Array3D.h"
#include "util/Camera.h"
#include "util/Intersection.h"
#include "util/util.h"
#include "util/Instrumentor.h"
#include "stb_image/Image.h"
#include "polar_decom/polar_decomposition_3x3.hpp"

PD::PDMetaballModel::PDMetaballModel( const PDMetaballModelConfig& cfg, PDMetaballHalfEdgeMesh* mesh )
    :_cfg( cfg ), _surface( mesh )
{
    Instrumentor::Get().BeginSession( "PDMetaballModel", "PDMetaballModel.json" );
    _line_segments = std::make_unique<GLLineSegment>();

    _simple_ball = std::make_unique<HalfEdgeMesh>( "res/models/ball960.obj" );
    _simple_ball->_material_main->SetDiffuseColor( 0.8f, 0.f, 0.f );
    _simple_ball->mTransform.SetScale( glm::vec3( 0.02f ) );
    _simple_cylin = std::make_unique<HalfEdgeMesh>( "res/models/cylinder.obj" );
    _simple_cylin->_material_main->SetDiffuseColor( 0.f, 0.f, 0.f );

    const static glm::vec3 COLORS[] = {
        glm::vec3( 99, 178, 238 ),
        glm::vec3( 118, 218, 145 ),
        glm::vec3( 248, 203, 127 ),
        glm::vec3( 248, 149, 136 ),
        glm::vec3( 124, 214, 207 ),
        glm::vec3( 145, 146, 171 ),
        glm::vec3( 120, 152, 225 ),
        glm::vec3( 239, 166, 102 ),
        glm::vec3( 237, 221, 134 ),
        glm::vec3( 153, 135, 206 ),
        glm::vec3( 99, 178, 238 ),
        glm::vec3( 118, 218, 145 )
    };


    std::srand( std::time( 0 ) );
    _color = COLORS[rand() % _countof( COLORS )] / 255.f;

    _mesh = std::make_unique<SphereMesh<Particle>>();
    _coarse_surface = std::make_unique<HalfEdgeMesh>( cfg._coarse_surface );
    _surface_path = cfg._coarse_surface;

    InstrumentationTimer timer( "Model Construction" );
    switch (cfg._method)
    {
    case 0:
    {
        _mesh->CreateFromSurfaceVoroOptimize( _coarse_surface.get(), cfg._coarse_surface, cfg._nb_points, cfg._nb_lloyd, cfg._sample_dx );
        break;
    }
    case 1:
    {
        _mesh->CreateFromSurfaceUniform( _coarse_surface.get(), cfg._sample_dx );
        break;
    }
    case 2:
    {
        _mesh->LoadFromSphereTreeFile( cfg._metaball_path );
        break;
    }
    case 3:
        SampleFromVoxel( cfg._sample_dx );
        break;
    }
    _skin_ball_buffer = std::make_unique<ShaderStorageBuffer>( nullptr, 0 );
    _skin_vtx_buffer = std::make_unique<ShaderStorageBuffer>( nullptr, 0 );
    CreateSurfaceMapping();
    UpdateSkinInfoBuffer();

    for (int i = 0; i < _mesh->BallsNum(); i++)
    {
        _mesh->Ball( i ).x += _cfg._displacement;
        _mesh->Ball( i ).x0 = _mesh->Ball( i ).x;
    }

    float m_max = 0.f;
    for (int i = 0; i < _mesh->BallsNum(); ++i)
    {
        auto& ball = _mesh->Ball( i );
        float r = ball.r;
        ball.m = 4.f / 3.f * 3.14f * r * r * r * cfg._density;
        m_max = std::max( m_max, ball.m );
    }
    for (int i = 0; i < _mesh->BallsNum(); ++i)
    {
        _mesh->Ball( i ).m_rel = _mesh->Ball( i ).m / m_max;
    }
    Init();

    //std::cout << "//////////////////////////////" << std::endl;
    //std::vector<float> mass_table( 20, 0.f );
    //std::vector<int> counts( 20, 0 );
    //HalfEdgeSurfaceTester tester( _coarse_surface.get() );
    //for (int i = 0; i < _mesh->BallsNum(); i++)
    //{
    //    float dist = tester.MinDistToSurface( _mesh->Ball( i ).x0 );
    //    int idx = (int)(dist / 0.02f);
    //    mass_table[idx] += _mesh->Ball( i ).m;
    //    counts[idx] += 1;
    //}

    //for (int i = 0; i < 20; i++)
    //{
    //    std::cout << mass_table[i] / counts[i] << std::endl;
    //}
}

PD::PDMetaballModel::~PDMetaballModel()
{
    Instrumentor::Get().EndSession();
}

void PD::PDMetaballModel::Init()
{
    int nb_points = _mesh->BallsNum();
    std::vector<SparseMatrixTriplet> m_triplets;
    std::vector<SparseMatrixTriplet> m_inv_triplets;
    _current_pos.resize( 3, nb_points );
    _rest_pos.resize( 3, nb_points );
    _current_vel.resize( 3, nb_points );
    _pene.resize( 3, nb_points );
    _external_force.resize( 3, nb_points );
    _last_pos.resize( 3, nb_points );
    _last_pos1.resize( 3, nb_points );
    _momentum.resize( 3, nb_points );
    _current_vel.setZero();
    _external_force.setZero();

    for (int i = 0; i < nb_points; ++i)
    {
        m_triplets.push_back( { i, i, _mesh->Ball( i ).m } );
        m_inv_triplets.push_back( { i, i, 1.f / _mesh->Ball( i ).m } );
        _current_pos.col( i ) = Vector3( _mesh->Ball( i ).x0.x, _mesh->Ball( i ).x0.y, _mesh->Ball( i ).x0.z );
        _rest_pos.col( i ) = _current_pos.col( i );
        _last_pos.col( i ) = _current_pos.col( i );
        _last_pos1.col( i ) = _current_pos.col( i );
    }
    _mass_matrix.resize( nb_points, nb_points );
    _mass_matrix_inv.resize( nb_points, nb_points );
    _mass_matrix.setFromTriplets( m_triplets.cbegin(), m_triplets.cend() );
    _mass_matrix_inv.setFromTriplets( m_inv_triplets.cbegin(), m_inv_triplets.cend() );

    for (int i = 0; i < nb_points; i++)
    {
        _mesh->Ball( i ).gu = Matrix3::Identity();
        _mesh->Ball( i ).R = Matrix3::Identity();
    }
    //for (int i = 0; i < _mesh->BallsNum(); i++)
    //{
    //    glm::vec3 pi = _mesh->Ball( i ).x0;
    //    for (int nei : _mesh->Ball( i ).neighbors)
    //    {
    //        glm::vec3 pj = _mesh->Ball( nei ).x0;
    //        _line_segments->AddPoint( pi );
    //        _line_segments->AddPoint( pj );
    //    }
    //}
    //_line_segments->UpdateMem();

    _constraints.clear();
    if (_cfg._const_type == 0)
    {
        for (int i = 0; i < nb_points; ++i)
        {
            std::vector<int> indices;
            indices.push_back( i );
            indices.insert( indices.end(), _mesh->Ball( i ).neighbors.begin(), _mesh->Ball( i ).neighbors.end() );
            if (_cfg._method == 3)
            {
                float k = 0.f;
                if (glm::distance( _mesh->Ball( i ).color, glm::vec3( 235, 177, 132 ) / 255.f )
                    < glm::distance( _mesh->Ball( i ).color, glm::vec3( 76, 35, 35 ) / 255.f ))
                {
                    k = _cfg._k_stiff;
                }
                else
                {
                    k = _cfg._k_stiff * 0.01f;
                }
                _constraints.push_back( std::make_unique<PD::MeshlessStrainConstraint<Particle>>( indices, k, _current_pos, _mesh.get(), &_rest_pos ) );
            }
            else
            {
                _constraints.push_back( std::make_unique<PD::MeshlessStrainConstraint<Particle>>( indices, _cfg._k_stiff, _current_pos, _mesh.get(), &_rest_pos ) );
            }
        }
    }
    else
    {
        ComputeAinvForEdgeConsts();
        std::unordered_set<IndexPair, IndexPair::Hash, IndexPair::Pred> edges;
        for (int i = 0; i < nb_points; i++)
        {
            Particle& ball = _mesh->Ball( i );
            for (int j : ball.neighbors)
            {
                edges.insert( IndexPair{ i, j } );
            }
        }
        for (const IndexPair& pair : edges)
        {
            _constraints.push_back( std::make_unique<PD::EdgeConstraint>( pair.i0, pair.i1, _cfg._k_stiff, _current_pos ) );
        }
    }
    if (_cfg._attach_filter != nullptr)
    {
        for (int i = 0; i < nb_points; ++i)
        {
            if (_cfg._attach_filter( _mesh->Ball( i ).x0 ))
            {
                _constraints.push_back( std::make_unique<AttachConstraint>( i, _cfg._k_attach, _rest_pos.col( i ) ) );
            }
        }
    }
    for (int i : _select_balls)
    {
        _constraints.push_back( std::make_unique<AttachConstraint>( i, _cfg._k_attach, _rest_pos.col( i ) ) );
    }
    _attached_balls = std::vector<int>( _select_balls.begin(), _select_balls.end() );
    _select_balls.clear();

    std::vector<SparseMatrixTriplet> triplets;
    int total_id = 0;
    for (auto& c : _constraints)
    {
        c->AddConstraint( triplets, total_id );
    }
    _projections.setZero( 3, total_id );

    SparseMatrix A( total_id, nb_points );
    A.setFromTriplets( triplets.begin(), triplets.end() );
    _At = A.transpose();
    _N = _At * A + _mass_matrix / _cfg._dt / _cfg._dt;
    _llt.compute( _N );
    if (_llt.info() != Eigen::ComputationInfo::Success)
        std::cout << "ERROR: " << _llt.info() << std::endl;

    std::cout << "Metaballs: " << nb_points << std::endl;
    std::cout << "ProjectVariable size: " << total_id << std::endl;
}

void PD::PDMetaballModel::Update()
{
    InstrumentationTimer timer( "Update" );
    if (Input::IsKeyDown( Input::Key::P ))
    {
        _simulate = !_simulate;
    }
    if (Input::IsMouseButtonDown( Input::MouseButton::Left ))
    {
        _hold_idx = -1;
        glm::vec2 cursor = Input::GetMousePosition();
        cursor.y = Camera::current->_viewport_size.y - cursor.y;
        Ray r = Camera::current->ScreenPointToRay( cursor );

        IntersectionRec rec;
        rec.t = FLT_MAX;
        bool intersect = false;
        int id = -1;
        for (int i = 0; i < _mesh->BallsNum(); ++i)
        {
            const auto& ball = _mesh->Ball( i );
            if (RayBallIntersect( r, ball.x, ball.r, &rec, 0.f, rec.t ))
            {
                intersect = true;
                id = i;
            }
        }

        if (intersect)
        {
            for (int i = 0; i < _mesh->BallsNum(); ++i)
            {
                //_mesh->Ball( i ).color = glm::vec3( 0.9f );
            }
            for (int nei : _mesh->Ball( id ).neighbors)
            {
                //_mesh->Ball( nei ).color = glm::vec3( 0.5, 0.5, 1 );
            }
            //_mesh->Ball( id ).color = glm::vec3( 0, 1, 0 );
            _init_cursor = Input::GetMousePosition();
        }
        _hold_idx = id;

        if (_hold_idx != -1 && Input::IsKeyHeld( Input::Key::LEFT_CONTROL ))
        {
            _select_balls.insert( _hold_idx );
        }
    }
    if (_hold_idx != -1)
    {
        if (Input::IsMouseButtonHeld( Input::MouseButton::Left ))
        {
            glm::vec2 delta = Input::GetMousePosition() - _init_cursor;
            glm::vec3 delta3d = -delta.x * Camera::current->mTransform.Left() - delta.y * Camera::current->mTransform.Up();
            delta3d *= _cfg._force;
            _ext_forces.insert( { _hold_idx, ToEigen( delta3d ) } );
            std::cout << "Fext " << _hold_idx << ":" << delta3d << std::endl;
        }
    }
    if (Input::IsMouseButtonReleased( Input::MouseButton::Left ))
    {
        _hold_idx = -1;
    }
    if (Input::IsKeyDown( Input::Key::E ))
    {
        static float deg = 0.f;
        glm::mat3 R = glm::rotate( glm::mat4( 1.f ), glm::radians( deg ), glm::normalize( glm::vec3( 0.2, 1, 0.5 ) ) );
        deg += 10.f;
        Matrix3 ER = ToEigen( R );
        Vector3 cm0;
        Vector3 cm;
        cm0.setZero();
        cm.setZero();
        float m_sum = 0.f;
        for (int i = 0; i < _mesh->BallsNum(); i++)
        {
            cm0 += _mesh->Ball( i ).m * _rest_pos.col( i );
            cm += _mesh->Ball( i ).m * _current_pos.col( i );
            m_sum += _mesh->Ball( i ).m;
        }
        cm0 /= m_sum;
        cm /= m_sum;

        for (int i = 0; i < _mesh->BallsNum(); i++)
        {
            Vector3 d0 = _rest_pos.col( i ) - cm0;
            _current_pos.col( i ) = cm0 + ER * d0;
        }

        _simulate = false;
    }
    if (Input::IsKeyDown( Input::Key::O ))
    {
        _show_balls = !_show_balls;
    }
    if (Input::IsKeyDown( Input::Key::U ))
    {
        _show_surface = !_show_surface;
    }
    if (Input::IsKeyDown( Input::Key::L ))
    {
        PhysicalUpdate();
    }
    if (Input::IsKeyDown( Input::Key::K ))
    {
        CollisionDetection();
        PostPhysicalUpdate();
    }

    if (_simulate)
    {
        //for (size_t i = 0; i < _cfg._physical_step; i++)
        //{
        //    PhysicalUpdate();
        //}
    }


    for (int i = 0; i < _mesh->BallsNum(); ++i)
    {
        Vector3 p = _current_pos.col( i );
        _mesh->Ball( i ).x = glm::vec3( p[0], p[1], p[2] );
    }
    //if (_simulate)
    //{
    //    MapSurface();
    //}
    if (_cfg._const_type == 1)
        ComputeBallOrit2();

    UpdateSkinInfoBuffer();
}

void PD::PDMetaballModel::Draw()
{
    InstrumentationTimer timer( "Render" );
    if (_show_balls)
    {
        _mesh->Draw();
        //_line_segments->Draw();
    }

    if (_show_surface)
    {
        _skin_vtx_buffer->BindBufferBase( 0 );
        _skin_ball_buffer->BindBufferBase( 1 );
        //_surface->_material_main->SetDiffuseColor( _color.x, _color.y, _color.z );
        _surface->Draw();
    }

    for (auto& pair : _array_ext_forces)
    {
        glm::vec3 start_pos = ToGLM( _current_pos.col( pair.first ).eval() );
        glm::vec3 end_pos = ToGLM( (_current_pos.col( pair.first ) + pair.second.normalized() * 0.2f).eval() );
        start_pos = start_pos + 0.05f * (end_pos - start_pos);
        //_simple_ball->mTransform.SetPos( start_pos );
        //_simple_ball->Draw();
        _simple_ball->mTransform.SetPos( end_pos );
        _simple_ball->Draw();
        _simple_cylin->mTransform.SetPos( (start_pos + end_pos) * 0.5f );
        _simple_cylin->mTransform.SetRotation( glm::fquat( glm::vec3( 0, 0, 1 ), glm::normalize( end_pos - start_pos ) ) );
        _simple_cylin->mTransform.SetScale( glm::vec3( 0.1f, 0.1f, 0.145f ) );
        _simple_cylin->Draw();
    }
}

void PD::PDMetaballModel::DrawGUI()
{
    ImGui::Begin( "metaball pd" );
    if (ImGui::Button( "Init" ))
    {
        Init();
    }
    ImGui::Checkbox( "simulate", &_simulate );
    ImGui::DragInt( "solve", &_cfg._nb_solve );
    ImGui::DragFloat( "stiffness", &_cfg._k_stiff, 1.0f, 0.0f, 5000.0f );
    ImGui::DragFloat( "force", &_cfg._force, 1.0f, 0.0f, 200.0f );
    if (ImGui::Button( "AddForce" ))
    {
        _array_ext_forces.push_back( { 0, Vector3::Zero() } );
    }
    for (int i = 0; i < _array_ext_forces.size(); i++)
    {
        ImGui::InputInt( (std::string( "id" ) + std::to_string( i )).c_str(), &_array_ext_forces[i].first );
        ImGui::DragFloat3( (std::string( "F" ) + std::to_string( i )).c_str(), _array_ext_forces[i].second.data(), 1.0f, -200.f, 200.f );
    }
    if (ImGui::Button( "ClearForce" ))
    {
        _array_ext_forces.clear();
    }
    ImGui::End();
}

void PD::PDMetaballModel::PhysicalUpdate()
{
    InstrumentationTimer timer( "PhysicalUpdate" );
    //External force
    for (int i = 0; i < _mesh->BallsNum(); ++i)
    {
        _external_force.col( i ).y() -= 9.8f;
    }
    for (auto& pair : _ext_forces)
    {
        _external_force.col( pair.first ) += pair.second;
    }
    for (auto& pair : _array_ext_forces)
    {
        _external_force.col( pair.first ) += pair.second;
    }

    if (Input::IsKeyHeld( Input::Key::R ))
    {
        static float rad = 0.0f;
        rad += 0.01f;
        if (rad <= 3.14 / 6)
        {
            auto q = Eigen::Quaternionf( Eigen::AngleAxisf( 0.00f, Eigen::Vector3f( 0, 1, 0 ) ) );
            Eigen::Matrix3f m = q.toRotationMatrix();
            for (int i = 0; i < _constraints.size(); i++)
            {
                if (typeid(*_constraints[i]) == typeid(AttachConstraint))
                {
                    AttachConstraint* c = dynamic_cast<AttachConstraint*>(_constraints[i].get());
                    int id = c->_indices[0];
                    if (_rest_pos.col( id ).y() < -0.2f)
                    {
                        auto p0 = c->_fixed_pos;
                        p0 = m * p0;
                        p0 += Vector3( 0, -0.010f, 0 );
                        c->_fixed_pos = p0;
                    }
                }
            }
        }
    }

#pragma omp parallel for
    for (int c = 0; c < _mesh->BallsNum(); c++)
    {
        _momentum.col( c ) = _current_pos.col( c ) + _current_vel.col( c ) * _cfg._dt;
        _momentum.col( c ) += _cfg._dt * _cfg._dt * _external_force.col( c );
        _last_pos.col( c ) = _current_pos.col( c );
        _current_pos.col( c ) = _momentum.col( c );
    }

    for (int i = 0; i < _cfg._nb_solve; i++)
    {
#pragma omp parallel
        {
#pragma omp for 
            for (int j = 0; j < _constraints.size(); j++)
            {
                _constraints[j]->Project( _current_pos, _projections );
            }
            auto start_t = std::chrono::high_resolution_clock::now();
#pragma omp for
            for (int r = 0; r < 3; r++)
            {
                Eigen::VectorXf rh_vec = _At * _projections.row( r ).transpose() + _mass_matrix / (_cfg._dt * _cfg._dt) * _momentum.row( r ).transpose();
                _current_pos.row( r ) = _llt.solve( rh_vec ).transpose();
            }
            //std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_t).count() << std::endl;
        }
    }

#pragma omp parallel
    {
#pragma omp for
        for (int c = 0; c < _mesh->BallsNum(); c++)
        {
            _current_vel.col( c ) = 0.9f / _cfg._dt * (_current_pos.col( c ) - _last_pos.col( c ));
        }
    }

    _aabb.max_corner = glm::vec3( -FLT_MAX );
    _aabb.min_corner = glm::vec3( FLT_MAX );
    for (int i = 0; i < _mesh->BallsNum(); i++)
    {
        _aabb.Expand( glm::vec3( _current_pos.col( i )(0), _current_pos.col( i )(1), _current_pos.col( i )(2) ) + glm::vec3( _mesh->Ball( i ).r ) );
        _aabb.Expand( glm::vec3( _current_pos.col( i )(0), _current_pos.col( i )(1), _current_pos.col( i )(2) ) - glm::vec3( _mesh->Ball( i ).r ) );
    }

    _external_force.setZero();
}

//Matrix3X PD::PDMetaballModel::ProjectVolumeConst( int i, const Matrix3X& current_pos ) const
//{
//    Matrix3 F = ComputeF( i, current_pos );
//    Particle& pi = (*_mesh)[i];
//    pi.gu = F - Matrix3::Identity();
//
//    Matrix3 U, V;
//    Vector3 S;
//    SVD( &U, &S, &V, F );
//    Vector3 singular_values = S;
//
//    float detF = F.determinant();
//
//    Vector3 d( 0.f, 0.f, 0.f );
//    for (int j = 0; j < 10; j++)
//    {
//        Real v = S( 0 ) * S( 1 ) * S( 2 );
//        Real f = v - std::clamp( v, 1.0f, 1.0f );
//        Vector3 g( S( 1 ) * S( 2 ), S( 0 ) * S( 2 ), S( 0 ) * S( 1 ) );
//        d = -((f - g.dot( d )) / g.dot( g )) * g;
//        S = singular_values + d;
//    }
//
//    if (detF < 0)
//    {
//        S[2] = -S[2];
//
//        float highv = S[0];
//        float midv = S[1];
//        float lowv = S[2];
//
//        if (midv < lowv) {
//            std::swap( midv, lowv );
//        }
//        if (highv < lowv) {
//            std::swap( highv, lowv );
//        }
//
//        if (highv < midv) {
//            std::swap( highv, midv );
//        }
//
//        S[0] = highv;
//        S[1] = midv;
//        S[2] = lowv;
//    }
//
//    F = U * S.asDiagonal() * V.transpose();
//
//    Matrix3X new_pos;
//    new_pos.resize( 3, (_mesh->Ball( i ).neighbors.size() + 1) );
//    new_pos.col( 0 ) = current_pos.col( i );
//    for (int j = 0; j < _mesh->Ball( i ).neighbors.size(); j++)
//    {
//        int nei = _mesh->Ball( i ).neighbors[j];
//        Vector3 x0ij = _rest_pos.col( nei ) - _rest_pos.col( i );
//        Vector3 after_rot = F * x0ij;
//        new_pos.col( j + 1 ) = current_pos.col( i ) + after_rot;
//    }
//
//    return new_pos;
//}

void PD::PDMetaballModel::CollisionDetection()
{
    InstrumentationTimer timer( "CollisionDetection" );
    std::vector<RigidStatic*> rigid_bodys = Scene::active->GetAllChildOfType<RigidStatic>();
    std::vector<PD::PDMetaballModel*> pd_models = Scene::active->GetAllChildOfType<PD::PDMetaballModel>();
    std::vector<RigidSDF*> rigid_sdfs = Scene::active->GetAllChildOfType<RigidSDF>();
    std::vector<RigidBall*> rigid_balls = Scene::active->GetAllChildOfType<RigidBall>();

    _pene.fill( 0.f );

#pragma omp parallel for
    for (int i = 0; i < _mesh->BallsNum(); i++)
    {
        auto& pi = _mesh->Ball( i );
        //pi.color = glm::vec3( 0, 1, 1 );

        for (RigidStatic* rigid : rigid_bodys)
        {
            //if (!AABBIntersect( rigid->BVH().BoundingBox(), _aabb ))
                //continue;

            //IntersectionRec rec;
            //int id = -1;
            //bool intersect = false;
            //if (rigid->CheckLineseg( pos - glmdx, pos, &rec, &id ))
            //{
            //    if (glm::dot( glmdx, rec.normal ) < 0)
            //    {
            //        Vector3 normal( rec.normal.x, rec.normal.y, rec.normal.z );
            //        Vector3 v = _current_vel.col( i );
            //        Vector3 vn = normal * v.dot( normal );
            //        Vector3 vt = v - vn;

            //        float dist = glm::distance( pos, rec.p );
            //        glm::vec3 glmpene = rec.p - pos;
            //        _pene.col( i ) += Vector3( glmpene.x, glmpene.y, glmpene.z );
            //        _current_vel.col( i ) -= 1.1 * vn;
            //        _mesh->Ball( i ).color = glm::vec3( 1, 0, 0 );
            //        intersect = true;
            //    }

            //}

            //auto result = rigid->CheckBall( pos, particle.r, i );
            //for (auto& ret : result)
            //{
            //    Vector3 normal{ ret._n.x, ret._n.y, ret._n.z };
            //    Vector3 v = _current_vel.col( i );
            //    Vector3 vn = normal * v.dot( normal );
            //    Vector3 vt = v - vn;

            //    Vector3 dx_t = dx - normal * dx.dot( normal );

            //    float dist = ret._depth;
            //    float s = std::min( 100 * dist, 1.0f );

            //    _current_vel.col( i ) = -0.8 * vn + 0.8 * vt;
            //    _current_pos.col( i ) += normal * ret._depth - s * dx_t;
            //    //_external_force.col( i ) += -vt;
            //}

            //pos = glm::vec3( _current_pos.col( i )(0), _current_pos.col( i )(1), _current_pos.col( i )(2) );
            //dx = _current_pos.col( i ) - _last_pos.col( i );
            ////////////////////////////////////////////////////////////////////////////////////////////////////////

            HalfEdgeMesh& sur = rigid->Surface();
            glm::vec3 pos( _current_pos.col( i )(0), _current_pos.col( i )(1), _current_pos.col( i )(2) );
            glm::vec3 last_pos( _last_pos.col( i )(0), _last_pos.col( i )(1), _last_pos.col( i )(2) );
            Vector3 dist( 0.f, 0.f, 0.f );
            for (int j = 0; j < sur.GetFaceNumber(); j++)
            {
                auto [ia, ib, ic] = sur.GetFaceIndices( j );
                glm::vec3 p0 = sur.GetPosition( ia );
                glm::vec3 p1 = sur.GetPosition( ib );
                glm::vec3 p2 = sur.GetPosition( ic );

                MovingSphereTriIntersectInfo info;
                if (MovingSphereTriIntersect( last_pos, pi.r, pos - last_pos, p0, p1, p2, &info ))
                {
                    //pi.color = glm::vec3( 1, 0, 0 );
                    glm::vec3 dx = pos - last_pos;
                    if (glm::length( pos - last_pos ) > 1e-6f)
                    {
                        dx = glm::normalize( dx );
                    }
                    Vector3 newpos = ToEigen( last_pos + dx * info.t );
                    dist += newpos - _current_pos.col( i );
                    Vector3 n = ToEigen( info.nc );
                    Vector3 v = _current_vel.col( i );
                    Vector3 vn = n * v.dot( n );
                    Vector3 vt = v - vn;
                    _current_vel.col( i ) = -vn + vt;

                }
            }
            _current_pos.col( i ) += dist;
            //_last_pos.col( i ) = _current_pos.col( i );
            //glm::vec3 normal;
            //float depth;
            //int id;
            //if (rigid->CheckInside( pos, &normal, &depth, &id ))
            //{
            //    Vector3 n( normal.x, normal.y, normal.z );
            //    Vector3 v = _current_vel.col( i );
            //    Vector3 vn = n * v.dot( n );
            //    Vector3 vt = v - vn;

            //    //float s = std::min( 500 * depth, 1.0f );

            //    Vector3 dx = _current_pos.col( i ) - _last_pos.col( i );
            //    Vector3 dx_n = n * dx.dot( n );
            //    Vector3 dx_t = dx - dx_n;
            //    //Vector3 ft = -dx_t * s;

            //    float s = 0.99f;
            //    if (vt.norm() < 0.1f)
            //        s = 1.0f;

            //    _last_pos.col( i ) = _current_pos.col( i );
            //    _current_pos.col( i ) += n * depth - s * dx_t;
            //    _current_vel.col( i ) = -0.5 * vn + (1.f - s) * vt;
            //}



        }

        //for (RigidStatic* rigid : rigid_bodys)
        //{
        //    //if (!AABBIntersect( rigid->BVH().BoundingBox(), _aabb ))
        //        //continue;

        //    glm::vec3 pos( _current_pos.col( i )(0), _current_pos.col( i )(1), _current_pos.col( i )(2) );

        //    glm::vec3 normal;
        //    float depth;
        //    int id;
        //    if (rigid->CheckInside( pos, &normal, &depth, &id ))
        //    {
        //        Vector3 n( normal.x, normal.y, normal.z );
        //        _current_pos.col( i ) += n * depth;
        //        _last_pos.col( i ) = _current_pos.col( i );
        //    }
        //}

        for (RigidSDF* rigid_sdf : rigid_sdfs)
        {
            glm::vec3 normal;
            float depth;
            glm::vec3 pos( _current_pos.col( i )(0), _current_pos.col( i )(1), _current_pos.col( i )(2) );

            if (rigid_sdf->CheckBall( pos, pi.r, &normal, &depth ))
            {
                Vector3 n( normal.x, normal.y, normal.z );
                Vector3 dx = _current_pos.col( i ) - _last_pos.col( i );
                Vector3 dx_n = n * dx.dot( n );
                Vector3 dx_t = dx - dx_n;
                float s = std::min( 1000 * depth, 1.0f );
                Vector3 fric = -s * dx_t;

                Vector3 v = _current_vel.col( i );
                Vector3 vn = n * v.dot( n );
                Vector3 vt = v - vn;

                _current_pos.col( i ) += n * depth - s * dx_t;
                _current_vel.col( i ) = -1.0 * vn + (1.f - s) * vt;
            }
        }

        for (RigidBall* rigid : rigid_balls)
        {
            glm::vec3 pos = ToGLM( _current_pos.col( i ).eval() );
            glm::vec3 last_pos = ToGLM( _last_pos.col( i ).eval() );
            auto info = BallBallIntersect( pos, pi.r, rigid->GetPos(), rigid->GetRadius() );
            if (info.has_value())
            {
                glm::vec3 newpos = pos - info->d * info->c1toc2;
                _current_pos.col( i ) = ToEigen( newpos );
                Vector3 v = _current_vel.col( i );
                Vector3 n = ToEigen( info->c1toc2 );
                Vector3 vn = n * v.dot( n );
                Vector3 vt = v - vn;
                _current_vel.col( i ) = -vn + vt;
            }
        }

        //for (PD::PDMetaballModel* model : pd_models)
        //{
        //    if (model == this)
        //        continue;
        //    if (!AABBIntersect( _aabb, model->_aabb ))
        //        continue;
        //    glm::vec3 p0 = ToGLM( _current_pos.col( i ).eval() );
        //    glm::vec3 lastp0 = ToGLM( _last_pos.col( i ).eval() );
        //    Vector3 vel0 = _current_vel.col( i );
        //    float r0 = pi.r;
        //    //continue;
        //    for (int j = 0; j < model->_mesh->BallsNum(); j++)
        //    {
        //        Particle& pj = model->_mesh->Ball( j );
        //        glm::vec3 p1 = ToGLM( model->_current_pos.col( j ).eval() );
        //        glm::vec3 lastp1 = ToGLM( model->_last_pos.col( j ).eval() );
        //        Vector3 vel1 = model->_current_vel.col( i );
        //        float r1 = model->_mesh->Ball( j ).r;

        //        float t = 0.f;
        //        glm::vec3 v0 = p0 - lastp0;
        //        glm::vec3 v1 = p1 - lastp1;
        //        if (TestMovingSphereSphere( lastp0, r0, v0, lastp1, r1, v1, &t ))
        //        {
        //            if (t == 0.f)
        //            {
        //                Vector3 cp0 = ToEigen( lastp0 );
        //                Vector3 cp1 = ToEigen( lastp1 );
        //                Vector3 cn = (cp0 - cp1).normalized();
        //                float depth = r0 + r1 - (cp0 - cp1).norm();

        //                _current_pos.col( i ) += depth * 0.5f * cn;
        //                model->_current_pos.col( j ) -= depth * 0.5f * cn;

        //                Vector3 vel0n = cn * vel0.dot( cn );
        //                Vector3 vel0t = vel0 - vel0n;
        //                Vector3 vel1n = cn * vel1.dot( cn );
        //                Vector3 vel1t = vel1 - vel1n;
        //                _current_vel.col( i ) = -vel0n + vel0t;
        //                model->_current_vel.col( j ) = -vel1n + vel1t;
        //            }
        //            else
        //            {
        //                Vector3 cp0 = ToEigen( lastp0 + t * v0 );
        //                Vector3 cp1 = ToEigen( lastp1 + t * v1 );
        //                Vector3 cn = (cp0 - cp1).normalized();
        //                Vector3 pene0 = _current_pos.col( i ) - cp0;
        //                Vector3 pene0n = cn * pene0.dot( cn );
        //                Vector3 pene0t = pene0 - pene0n;
        //                Vector3 pene1 = model->_current_pos.col( j ) - cp1;
        //                Vector3 pene1n = cn * pene1.dot( cn );
        //                Vector3 pene1t = pene1 - pene1n;
        //                Vector3 dx0 = ToEigen( p0 - lastp0 );
        //                Vector3 dx0n = cn * dx0.dot( cn );
        //                Vector3 dx0t = dx0 - dx0n;
        //                Vector3 dx1 = ToEigen( p1 - lastp1 );
        //                Vector3 dx1n = cn * dx1.dot( cn );
        //                Vector3 dx1t = dx1 - dx1n;
        //                _current_pos.col( i ) -= pene0n + 0.5 * pene0t;
        //                _current_pos.col( i ) = cp0 + 0.5f * (-dx0n + 0.5f * dx0t) * (1.0f - t);
        //                model->_current_pos.col( j ) -= pene1n + 0.5 * pene1t;
        //                model->_current_pos.col( j ) = cp1 + 0.5 * (-dx1n + 0.5f * dx1t) * (1.0f - t);

        //                Vector3 vel0n = cn * vel0.dot( cn );
        //                Vector3 vel0t = vel0 - vel0n;
        //                Vector3 vel1n = cn * vel1.dot( cn );
        //                Vector3 vel1t = vel1 - vel1n;
        //                _current_vel.col( i ) = -vel0n + vel0t;
        //                model->_current_vel.col( j ) = -vel1n + vel1t;

        //                p0 = ToGLM( _current_pos.col( i ).eval() );
        //                vel0 = _current_vel.col( i );
        //            }
        //        }
        //    }
        //}

        glm::vec3 pos = ToGLM( _current_pos.col( i ).eval() );
        if (pos.y < -2.f)
        {
            Vector3 n( 0.f, 1.f, 0.f );
            Vector3 dx = _current_pos.col( i ) - _last_pos.col( i );
            Vector3 dxn = n * dx.dot( n );
            Vector3 dxt = dx - dxn;
            _current_pos.col( i ) -= dxt * 0.5f;
            _current_pos.col( i ).y() = -2.f;
            Vector3 v = _current_vel.col( i );
            Vector3 vn = n * v.dot( n );
            Vector3 vt = v - vn;
            _current_vel.col( i ) = -vn + 0.01 * vt;
        }
    }

    for (PD::PDMetaballModel* model : pd_models)
    {
        if (model == this)
            continue;
        if (!AABBIntersect( _aabb, model->_aabb ))
            continue;
        for (int i = 0; i < _mesh->BallsNum(); i++)
        {
            auto& pi = _mesh->Ball( i );

            for (int j = 0; j < model->_mesh->BallsNum(); j++)
            {
                Particle& pj = model->_mesh->Ball( j );
                Vector3 xj = model->_current_pos.col( j );
                float r = model->_mesh->Ball( j ).r;
                float dist = r + pi.r;
                Vector3 distvec = xj - _current_pos.col( i );
                float dist2 = (distvec).norm();
                float h = dist - dist2;

                if (h > 0.000001)
                {
                    //intersect
                    //Vector3 n = distvec / dist2;
                    //float mass_ratio = model->_mesh->Ball( j ).m / (model->_mesh->Ball( j ).m + particle.m);
                    //Vector3 move = -n * h * mass_ratio;
                    //_pene.col( i ) += move;

                    //Vector3 v = _current_vel.col( i );
                    //Vector3 vn = v.dot( n ) * n;
                    //Vector3 vt = v - vn;

                    //_current_vel.col( i ) = -0.8 * vn + 0.8 * vt /** std::clamp( 1.f - h, 0.0f, 1.0f )*/;

                    float sdfi = pi.sdf;
                    float sdfj = pj.sdf;
                    Vector3 xji = _current_pos.col( i ) - xj;
                    float mass_ratio = pj.m / (pj.m + pi.m);
                    //float d = std::min( sdfi, sdfj );

                    Vector3 outi = pi.R * Vector3( pi.outside.x, pi.outside.y, pi.outside.z );
                    Vector3 outj = pj.R * Vector3( pj.outside.x, pj.outside.y, pj.outside.z );

                    if (/*xji.dot( outj ) < 0*/false)
                    {
                        //nagative side of j
                        Vector3 n = xji - 2 * (xji.dot( outj ) * outj);
                        n = -xji;
                        float d = pj.r + pi.r - xji.norm();
                        d = pj.sdf;

                        _current_pos.col( i ) += d * mass_ratio * n;
                        _last_pos.col( i ) = _pene.col( i );
                        model->_current_pos.col( j ) -= d * (1 - mass_ratio) * n;
                        model->_last_pos.col( j ) = model->_current_pos.col( j );

                        Vector3 v = _current_vel.col( i );
                        Vector3 vn = v.dot( n ) * n;
                        Vector3 vt = v - vn;
                        _current_vel.col( i ) = 1.0 * vn + 0.5 * vt;

                        Vector3 vj = model->_current_vel.col( j );
                        Vector3 vjn = vj.dot( n ) * n;
                        Vector3 vjt = vj - vjn;
                        model->_current_vel.col( j ) = -vjn + 0.5 * vjt;

                    }
                    else
                    {
                        //positive side of j
                        Vector3 n = xji.normalized();
                        Vector3 dx = _current_pos.col( i ) - _last_pos.col( i );
                        Vector3 dx_n = dx.dot( n ) * n;
                        Vector3 dx_t = dx - dx_n;

                        float d = pj.r + pi.r - xji.norm();
                        _current_pos.col( i ) += d * mass_ratio * n/* - dx_t * 0.8f*/;
                        _last_pos.col( i ) = _current_pos.col( i );

                        Vector3 dxj = model->_current_pos.col( j ) - model->_last_pos.col( j );
                        Vector3 dxjn = dxj.dot( n ) * n;
                        Vector3 dxjt = dxj - dxjn;
                        model->_current_pos.col( j ) -= d * (1 - mass_ratio) * n/* - dxjt * 0.8*/;
                        model->_last_pos.col( j ) = model->_current_pos.col( j );

                        Vector3 v = _current_vel.col( i );
                        Vector3 vn = v.dot( n ) * n;
                        Vector3 vt = v - vn;
                        _current_vel.col( i ) = -0.8 * vn + 0.2 * vt;

                        Vector3 vj = model->_current_vel.col( j );
                        Vector3 vjn = vj.dot( n ) * n;
                        Vector3 vjt = vj - vjn;
                        model->_current_vel.col( j ) = -0.8 * vjn + 0.2 * vjt;

                    }
                    //if (true)
                    //{
                    //    if (sdfi < sdfj)
                    //        n = pi.R * Vector3( pi.outside.x, pi.outside.y, pi.outside.z );
                    //    else
                    //        n = -pj.R * Vector3( pj.outside.x, pj.outside.y, pj.outside.z );
                    //}
                    /*else if (pi.isborder && pj.isborder)
                    {
                        if (sdfi < sdfj)
                            n = pi.R * Vector3( pi.outside.x, pi.outside.y, pi.outside.z );
                        else
                            n = -pj.R * Vector3( pj.outside.x, pj.outside.y, pj.outside.z );

                        if (xij.dot( n ) < 0)
                        {
                            n = xij - 2 * (xij.dot( n )) * n;
                            pi.color = glm::vec3( 1, 0, 0 );
                            pj.color = glm::vec3( 1, 0, 0 );
                        }
                        else
                            n = xij;

                        d = xij.norm();

                    }
                    else if (pi.isborder && !pj.isborder)
                    {
                        if (sdfi < sdfj)
                        {
                            n = pi.R * Vector3( pi.outside.x, pi.outside.y, pi.outside.z );
                            if (xij.dot( n ) < 0)
                            {
                                n = xij - 2 * (xij.dot( n )) * n;
                                pi.color = glm::vec3( 1, 0, 0 );
                                pj.color = glm::vec3( 1, 0, 0 );
                            }
                            else
                                n = xij;

                            d = xij.norm();
                        }
                        else
                            n = -pj.R * Vector3( pj.outside.x, pj.outside.y, pj.outside.z );

                    }
                    else if (!pi.isborder && pj.isborder)
                    {
                        if (sdfi < sdfj)
                        {
                            n = pi.R * Vector3( pi.outside.x, pi.outside.y, pi.outside.z );
                        }
                        else
                        {
                            n = -pj.R * Vector3( pj.outside.x, pj.outside.y, pj.outside.z );
                            if (xij.dot( n ) < 0)
                            {
                                pi.color = glm::vec3( 1, 0, 0 );
                                pj.color = glm::vec3( 1, 0, 0 );
                                n = xij - 2 * (xij.dot( n )) * n;
                            }
                            else
                                n = xij;

                            d = xij.norm();
                        }

                    }*/
                }
            }

        }
    }

}

void PD::PDMetaballModel::PostPhysicalUpdate()
{
    //#pragma omp parallel for
    //    for (int c = 0; c < _current_pos.cols(); c++)
    //    {
    //        _current_pos.col( c ) += _pene.col( c );
    //    }
    //_current_pos += _pene;
    _ext_forces.clear();
    //MapSurface();
}

void PD::PDMetaballModel::CreateSurfaceMapping()
{
    auto start = std::chrono::high_resolution_clock::now();
    _vtx_skinning_table.resize( _surface->GetVertexNumber() );
#pragma omp parallel for
    for (int i = 0; i < _surface->GetVertexNumber(); i++)
    {
        if (_surface->_vertices[i].edge == -1)
            continue;

        std::map<float, int, std::greater<float>> min_heap;
        for (int j = 0; j < _mesh->BallsNum(); j++)
        {
            auto& ball = _mesh->Ball( j );
            float dot = glm::dot( _surface->_edges[_surface->_vertices[i].edge].normal, ball.x - _surface->_vertices[i].pos );
            float dist = glm::distance( ball.x0, _surface->GetRestPos( i ) );
            float w = glm::exp( -((dist / ball.r) * (dist / ball.r) / 1.5f) );
            //if (dot > 0)
            //{
            //    w = 0.f;
            //}
            min_heap.insert( std::make_pair( w, j ) );
        }
        std::array<float, NEICOUNT> weights;
        weights.fill( 0.f );
        std::array<int, NEICOUNT> indices;
        indices.fill( -1 );

        int k = 0;
        float weight_sum = 0.f;
        for (const auto& pair : min_heap)
        {
            weights[k] = pair.first;
            indices[k] = pair.second;
            weight_sum += weights[k];
            k++;
            if (k == NEICOUNT)
                break;
        }

        for (int j = 0; j < k; j++)
        {
            weights[j] /= weight_sum;
        }
        _vtx_skinning_table[i].indices = indices;
        _vtx_skinning_table[i].weights = weights;
    }
    _skin_vtx_buffer->UpdateData( _vtx_skinning_table.data(), _vtx_skinning_table.size() * sizeof( _vtx_skinning_table[0] ) );
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
    std::cout << "CreateSurfaceMapping: " << duration.count() << std::endl;
}

void PD::PDMetaballModel::UpdateSkinInfoBuffer()
{
    /*
    * single thread: 1000balls, 40~50 microsec.
    * multithread: 1000balls, around 20 microsec.
    */
    //using hrc = std::chrono::high_resolution_clock;
    //auto start_t = hrc::now();

    _ball_skinning_infos.resize( _mesh->BallsNum() );
#pragma omp parallel
    {
#pragma omp for
        for (int i = 0; i < _mesh->BallsNum(); i++)
        {
            Matrix3 gu = _mesh->Ball( i ).gu;
            glm::mat4 m( 0.f );
            glm::mat4 R( 0.f );
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                {
                    m[j][k] = gu( k, j );
                    R[j][k] = _mesh->Ball( i ).R( k, j );
                }

            _ball_skinning_infos[i] = BallSkinningInfo{
                glm::vec4( _mesh->Ball( i ).x, 1.0f ),
                glm::vec4( _mesh->Ball( i ).x0 - _cfg._displacement, 1.0 ),
                m, R };
        }
    }
    for (int i : _attached_balls)
    {
        _ball_skinning_infos[i].R = glm::mat4( 1.f );
        _ball_skinning_infos[i].gu = glm::mat4( 1.f );
    }
    //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(hrc::now() - start_t);
    //if (Input::IsKeyDown( Input::Key::F ))
    //{
    //    std::cout << duration.count() << std::endl;
    //}
    _skin_ball_buffer->UpdateData( _ball_skinning_infos.data(), _ball_skinning_infos.size() * sizeof( _ball_skinning_infos[0] ), GL_DYNAMIC_DRAW );
}

void PD::PDMetaballModel::MapSurface()
{
    for (int i = 0; i < _surface->GetVertexNumber(); i++)
    {
        const auto& skin_info = _vtx_skinning_table[i];
        glm::vec3 p( 0.f, 0.f, 0.f );
        for (int j = 0; j < NEICOUNT; j++)
        {
            int ballid = skin_info.indices[j];
            float weight = skin_info.weights[j];
            glm::mat3 R = ToGLM( _mesh->Ball( ballid ).R );

            p += weight * (_mesh->Ball( ballid ).x + R * (_surface->GetRestPos( i ) - _mesh->Ball( ballid ).x0));
        }
        _surface->SetPosition( i, p );
    }

    //_surface->UpdateNormal();
    //_surface->UpdatePosBuffer();
    //_surface->UpdateAttrBuffer();
}

void PD::PDMetaballModel::SampleFromVoxel( float steplen )
{
    std::string folder = "C:\\Dev\\knees_data\\images\\";
    steplen = steplen * 947.f;

    std::ifstream ifs( "D:/models/knee.obj" );
    if (ifs.is_open())
    {
        std::string line;
        while (std::getline( ifs, line ))
        {
            std::stringstream ss( line );
            glm::vec3 p;
            glm::vec3 c;
            char temp;
            ss >> temp >> p.x >> p.y >> p.z >> c.x >> c.y >> c.z;
            _mesh->AddBall( p, steplen / 947.f );
            _mesh->Ball( _mesh->BallsNum() - 1 ).color = c;
        }
    }
    else
    {
        Array3D<glm::vec3> grid( 583, 874, 947 );
        AABB aabb;
        for (int i = 0; i < 947; i++)
        {
            std::cout << i << std::endl;
            int idx = 3580 + i * 3;
            std::string path = folder + std::to_string( idx ) + ".png";
            Image<unsigned char> image = LoadImageFile( path );
            for (int x = 0; x < 583; x++)
            {
                for (int y = 0; y < 874; y++)
                {
                    glm::vec3 c( image.Get( x, y, 0 ), image.Get( x, y, 1 ), image.Get( x, y, 2 ) );
                    if (c != glm::vec3( 0.f ))
                    {
                        aabb.Expand( glm::vec3( x, y, i ) );
                    }
                    c /= 255.f;
                    grid( x, y, i ) = c;
                }
            }
        }

        std::cout << "AABB: " << aabb.min_corner << ". " << aabb.max_corner << ". " << aabb.GetCenter() << std::endl;


        int dimx = 583.f / steplen;
        int dimy = 874.f / steplen;
        int dimz = 947.f / steplen;

        std::ofstream ofs( "D:/models/knee.obj" );
        for (int i = 0; i < dimx; i++)
        {
            for (int j = 0; j < dimy; j++)
            {
                for (int k = 0; k < dimz; k++)
                {
                    float px = i * steplen;
                    float py = j * steplen;
                    float pz = k * steplen;
                    int ix = (int)px;
                    int iy = (int)py;
                    int iz = (int)pz;
                    glm::vec3 c = grid( 582 - ix, iy, iz );
                    if (c.x != 0 || c.y != 0 || c.z != 0)
                    {
                        px /= 947.f;
                        py /= 947.f;
                        pz /= 947.f;

                        _mesh->AddBall( glm::vec3( px, py, pz ), steplen / 947.f );
                        _mesh->Ball( _mesh->BallsNum() - 1 ).color = c;
                        ofs << "v " << px << ' ' << py << ' ' << pz << ' ' << c.x << ' ' << c.y << ' ' << c.z << '\n';
                    }
                }
            }
        }
        ofs.flush();
        ofs.close();
    }

    std::cout << "ParticleNum=" << _mesh->BallsNum() << std::endl;

    constexpr int MIN_NEI = 8;
    for (int i = 0; i < _mesh->BallsNum(); i++)
    {
        std::vector<int>& neighbors = _mesh->Ball( i ).neighbors;

        std::multimap<float, int> min_heap;
        for (int j = 0; j < _mesh->BallsNum(); j++)
        {
            if (i == j)
                continue;
            if (std::find( std::begin( neighbors ), std::end( neighbors ), j ) != std::end( neighbors ))
                continue;

            float dist = glm::distance( _mesh->Ball( i ).x0, _mesh->Ball( j ).x0 );
            min_heap.insert( { dist, j } );
        }

        int counter = MIN_NEI - neighbors.size();
        for (auto& pair : min_heap)
        {
            if (counter <= 0)
                break;
            neighbors.push_back( pair.second );
            counter--;
        }
    }
}

void PD::PDMetaballModel::ComputeBallOrit()
{
#pragma omp parallel for
    for (int i = 0; i < _mesh->BallsNum(); i++)
    {
        Particle& ball = _mesh->Ball( i );
        float m_sum = ball.m;
        glm::vec3 cm = ball.m * ball.x;
        glm::vec3 cm0 = ball.m * ball.x0;
        for (int j : ball.neighbors)
        {
            const Particle& nei = _mesh->Ball( j );
            m_sum += nei.m;
            cm += nei.m * nei.x;
            cm0 += nei.m * nei.x0;
        }
        cm /= m_sum;
        cm0 /= m_sum;

        glm::mat3 A1 = ball.m * glm::TensorProduct( ball.x, ball.x0 );
        //glm::mat3 A2 = ball.m * ball.r * ball.r * glm::toMat3( ball.q );
        for (int j : ball.neighbors)
        {
            const Particle& nei = _mesh->Ball( j );
            A1 += nei.m * glm::TensorProduct( nei.x, nei.x0 );
            //A2 += nei.m * nei.r * nei.r * glm::toMat3( nei.q );
        }
        glm::mat3 Ac = m_sum * glm::TensorProduct( cm, cm0 );
        A1 -= Ac;

        glm::mat3 A = A1;

        Mat3 A_( A[0][0], A[1][0], A[2][0],
            A[0][1], A[1][1], A[2][1],
            A[0][2], A[1][2], A[2][2] );

        Polar_decomposition<true> polar_decom( A_ );
        Mat3 R_ = polar_decom.matrix_R();

        glm::mat3 R;
        for (size_t c = 0; c < 3; c++)
        {
            R[c][0] = R_( 0, c );
            R[c][1] = R_( 1, c );
            R[c][2] = R_( 2, c );
        }

        ball.q = glm::normalize( glm::toQuat( R ) );
        Matrix3 eigenR;
        for (size_t c = 0; c < 3; c++)
        {
            eigenR( 0, c ) = R[c][0];
            eigenR( 1, c ) = R[c][1];
            eigenR( 2, c ) = R[c][2];
        }
        ball.R = eigenR;
        ball.gu = eigenR;
    }
}

void PD::PDMetaballModel::ComputeBallOrit2()
{
#pragma omp parallel for
    for (int i = 0; i < _mesh->BallsNum(); i++)
    {
        Vector3 ui = _current_pos.col( i ) - _rest_pos.col( i );
        Vector3 sx = Vector3::Zero();
        Vector3 sy = Vector3::Zero();
        Vector3 sz = Vector3::Zero();
        float wsum = 0.f;
        int cnt = 0;
        for (int j : _mesh->Ball( i ).neighbors)
        {
            Vector3 uj = _current_pos.col( j ) - _rest_pos.col( j );
            Vector3 xij = _rest_pos.col( j ) - _rest_pos.col( i );
            float wij = _weights_for_edge_consts[i][cnt++];
            wsum += wij;
            sx += (uj[0] - ui[0]) * xij * wij;
            sy += (uj[1] - ui[1]) * xij * wij;
            sz += (uj[2] - ui[2]) * xij * wij;
        }

        sx /= wsum;
        sy /= wsum;
        sz /= wsum;

        Vector3 dux = _Ainv_for_edge_consts[i] * sx;
        Vector3 duy = _Ainv_for_edge_consts[i] * sy;
        Vector3 duz = _Ainv_for_edge_consts[i] * sz;

        Matrix3 F;
        F.col( 0 ) = dux;
        F.col( 1 ) = duy;
        F.col( 2 ) = duz;
        F.transposeInPlace();
        F( 0, 0 ) += 1.f;
        F( 1, 1 ) += 1.f;
        F( 2, 2 ) += 1.f;

        Matrix3 U, V;
        Vector3 S;
        SVD( &U, &S, &V, F );

        _mesh->Ball( i ).R = U * V.transpose();
        _mesh->Ball( i ).gu = F;
    }

}

void PD::PDMetaballModel::ComputeAinvForEdgeConsts()
{
    _Ainv_for_edge_consts.resize( _mesh->BallsNum() );
    _weights_for_edge_consts.resize( _mesh->BallsNum() );
#pragma omp parallel for
    for (int i = 0; i < _mesh->BallsNum(); i++)
    {
        Matrix3 A = Matrix3::Zero();
        float avg_dist = 0.f;
        const auto& ball = _mesh->Ball( i );
        _weights_for_edge_consts[i].resize( ball.neighbors.size() );

        for (int j : ball.neighbors)
            avg_dist += (_rest_pos.col( i ) - _rest_pos.col( j )).norm();
        avg_dist /= (ball.neighbors.size());

        float wsum = 0.f;
        int cnt = 0;
        for (int j : ball.neighbors)
        {
            Vector3 xij = _rest_pos.col( j ) - _rest_pos.col( i );
            float r = xij.norm();
            float h = avg_dist * 3;
            float wij = 0.f;
            if (r < h)
                wij = 315.f * glm::pow( h * h - r * r, 3 ) / (64.f * glm::pi<float>() * glm::pow( h, 9 ));
            wij *= _mesh->Ball( i ).m;

            _weights_for_edge_consts[i][cnt++] = wij;
            wsum += wij;
            A += wij * xij * xij.transpose();
        }
        A /= wsum;

        double detA = A.determinant();

        if (std::abs( detA ) < 1e-8f)
        {
            _Ainv_for_edge_consts[i] = EigenSafeInverse( A );
        }
        else
        {
            _Ainv_for_edge_consts[i] = A.inverse();
        }
    }
}

void PD::PDMetaballModel::ComputeBallOutsideVec()
{
    for (int i = 0; i < _mesh->BallsNum(); i++)
    {
        Particle& ball = _mesh->Ball( i );
        if (!ball.isborder)
            continue;

        glm::vec3 outside( 0.f );
        for (int inei : ball.neighbors)
        {
            glm::vec3 d = ball.x0 - _mesh->Ball( inei ).x0;
            outside += glm::normalize( d );
        }

        ball.outside = glm::normalize( outside );
        _line_segments->AddPoint( ball.x0 );
        _line_segments->AddPoint( ball.x0 + ball.outside * 0.1f );
    }
    _line_segments->UpdateMem();
}
