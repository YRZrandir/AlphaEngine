#include "MetaballModel.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <unordered_set>
#include <list>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/EigenValues>
#include <unsupported/Eigen/MatrixFunctions>
#include <pcl/common/centroid.h>
#include <pcl/filters/covariance_sampling.h>
#include <omp.h>
#include <imgui/imgui.h>

#include "SphereTreeHelper.h"
#include "acceleration/AABB.h"
#include "model/ModelLoader.h"
#include "model/RigidStatic.h"
#include "util/Intersection.h"
#include "util/util.h"
#include "util/Camera.h"
#include "util/Shader.h"
#include "input/Input.h"
#include "MetaballTester.h"
#include "tinycolormap.hpp"
#include "polar_decom/polar_decomposition_3x3.hpp"
#include "CVT/WeightedCVT.h"
#include "material/Material.h"

namespace PBD
{
MetaballModel::MetaballModel( const std::vector<Metaball>& metaball_list, std::unique_ptr<MetaballHalfEdgeMesh>&& surface, float density )
    :_metaballs( metaball_list ), _surface( std::move( surface ) ), _density( density )
{

    _surface->mRenderConfig._draw_line = false;
    _surface->mRenderConfig._draw_face = true;
    _surface->mRenderConfig._face_mattype = RenderConfig::MatType::Texture;
    _surface->mRenderConfig._line_mattype = RenderConfig::MatType::VtxColor;
    _surface_tester.SetSurface( _surface.get() );
    //SampleFromSurface();

    _ball_mesh = std::make_unique<HalfEdgeMesh>( "res/models/ball960.obj" );
    _ball_mesh->Normalize();
    _ball_mesh->UpdatePosBuffer();
    _ball_mesh->UpdateAttrBuffer();

    _hand1 = std::make_unique<HalfEdgeMesh>( "res/models/hand1.obj" );
    _hand2 = std::make_unique<HalfEdgeMesh>( "res/models/hand2.obj" );
    _hand1->Normalize();
    _hand2->Normalize();
    _hand1->UpdatePosBuffer();
    _hand2->UpdatePosBuffer();
    _hand1->mTransform.SetScale( glm::vec3( 0.2f ) );
    _hand2->mTransform.SetScale( glm::vec3( 0.2f ) );
    _hand1->mRenderConfig._face_mattype = RenderConfig::MatType::PlainColor;
    _hand2->mRenderConfig._face_mattype = RenderConfig::MatType::PlainColor;
    _hand1->_material_main->mDiffuseColor = glm::vec3( 0.5, 0.5, 1.0 );
    _hand2->_material_main->mDiffuseColor = glm::vec3( 0.5, 0.5, 1.0 );
    _hand1->_material_main->mAlpha = 0.5f;
    _hand2->_material_main->mAlpha = 0.5f;

    _ball_orit_lines = std::make_unique<GLLineSegment>();
    _ball_orit_lines->ReservePoint( _metaballs.size() * 2 );

    _skin_ball_buffer = std::make_unique<ShaderStorageBuffer>( nullptr, 0, GL_DYNAMIC_DRAW );
    InitVertices();
    CreateTopo();
    InitConstraints();
    CreateSurfaceMapping2();
    _skin_vtx_buffer = std::make_unique<ShaderStorageBuffer>(
        _vtx_skinning_table.data(), sizeof( _vtx_skinning_table[0] ) * _vtx_skinning_table.size(), GL_DYNAMIC_DRAW );

    _bvh = std::make_unique<BVHTree_Metaball>( this );
}

MetaballModel::MetaballModel( const std::string& surface_path, const std::string& coarse_path, float density, int lloyd, int num )
    :_density( density )
{
    _surface = std::make_unique<MetaballHalfEdgeMesh>( surface_path );
    _coarse_surface = std::make_unique<HalfEdgeMesh>( coarse_path );

    AABB aabb;
    for (int i = 0; i < _coarse_surface->_vertices.size(); i++)
    {
        aabb.Expand( _coarse_surface->_vertices[i].pos );
    }

    WeightedSamplePointsResult result = WeightedSamplePoints( coarse_path, num, lloyd, 0.02f );
    std::vector<glm::vec3>& pts = result.pts;
    std::vector<float>& radius = result.radius;
    std::vector<std::vector<int>>& nei_lists = result.nei_lists;
    std::vector<int>& is_border = result.border_flags;


    for (int i = 0; i < pts.size(); i++)
    {
        float r = radius[i];
        if (r < 1e-3f)
        {
            r = 1e-3f;
        }
        _metaballs.push_back( PBD::Metaball( pts[i], r ) );
        _metaballs.back().neighbors = nei_lists[i];
    }

    //Make sure every ball has a minimum number of neighbors.
    constexpr int MIN_NEI = 4;
    for (int i = 0; i < _metaballs.size(); i++)
    {
        std::vector<int>& neighbors = _metaballs[i].neighbors;
        if (neighbors.size() >= MIN_NEI)
            continue;

        std::multimap<float, int> min_heap;
        for (int j = 0; j < _metaballs.size(); j++)
        {
            if (i == j)
                continue;
            if (std::find( std::begin( neighbors ), std::end( neighbors ), j ) != std::end( neighbors ))
                continue;

            float dist = glm::distance( _metaballs[i].x0, _metaballs[j].x0 );
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

    //make sure that every link is double sided
    //for (int i = 0; i < _metaballs.size(); i++)
    //{
    //    std::vector<int>& neilist_i = _metaballs[i].neighbors;
    //    for (int nei : neilist_i)
    //    {
    //        std::vector<int>& neilist_nei = _metaballs[nei].neighbors;
    //        if (std::find( neilist_nei.begin(), neilist_nei.end(), i ) == neilist_nei.end())
    //        {
    //            neilist_nei.push_back( i );
    //        }
    //    }
    //}

    _surface->mRenderConfig._draw_line = false;
    _surface->mRenderConfig._draw_face = true;
    _surface->mRenderConfig._face_mattype = RenderConfig::MatType::Texture;
    _surface->mRenderConfig._line_mattype = RenderConfig::MatType::VtxColor;
    _surface_tester.SetSurface( _surface.get() );

    _ball_mesh = std::make_unique<HalfEdgeMesh>( "res/models/ball960.obj" );
    _ball_mesh->Normalize();
    _ball_mesh->UpdatePosBuffer();
    _ball_mesh->UpdateAttrBuffer();

    _hand1 = std::make_unique<HalfEdgeMesh>( "res/models/hand1.obj" );
    _hand2 = std::make_unique<HalfEdgeMesh>( "res/models/hand2.obj" );
    _hand1->Normalize();
    _hand2->Normalize();
    _hand1->UpdatePosBuffer();
    _hand2->UpdatePosBuffer();
    _hand1->mTransform.SetScale( glm::vec3( 0.2f ) );
    _hand2->mTransform.SetScale( glm::vec3( 0.2f ) );
    _hand1->mRenderConfig._face_mattype = RenderConfig::MatType::PlainColor;
    _hand2->mRenderConfig._face_mattype = RenderConfig::MatType::PlainColor;
    _hand1->_material_main->mDiffuseColor = glm::vec3( 0.5, 0.5, 1.0 );
    _hand2->_material_main->mDiffuseColor = glm::vec3( 0.5, 0.5, 1.0 );
    _hand1->_material_main->mAlpha = 0.5f;
    _hand2->_material_main->mAlpha = 0.5f;

    _ball_orit_lines = std::make_unique<GLLineSegment>();
    _ball_orit_lines->ReservePoint( _metaballs.size() * 2 );

    _skin_ball_buffer = std::make_unique<ShaderStorageBuffer>( nullptr, 0, GL_DYNAMIC_DRAW );
    InitVertices();
    //CreateTopo();
    InitConstraints();
    CreateSurfaceMapping2();
    _skin_vtx_buffer = std::make_unique<ShaderStorageBuffer>(
        _vtx_skinning_table.data(), sizeof( _vtx_skinning_table[0] ) * _vtx_skinning_table.size(), GL_DYNAMIC_DRAW );

    _bvh = std::make_unique<BVHTree_Metaball>( this );
    _surface->UpdatePosBuffer();
    _surface->UpdateAttrBuffer();
}

MetaballModel::MetaballModel( const std::string& path, const std::string& surface_path, float density )
{
    std::ifstream ifs( path, std::ios_base::in );
    std::string first_line;
    std::getline( ifs, first_line );
    std::stringstream ss( first_line );

    int depth = 0;
    int degree = 0;
    ss >> depth >> degree;

    if (depth <= 0 || degree <= 0)
    {
        throw std::runtime_error( "File format incorrect" );
    }

    int node_num = 0;
    for (size_t i = 0; i < depth; i++)
    {
        node_num += (int)glm::pow( degree, i );
    }

    std::vector<PBD::Metaball> metaballs;
    for (size_t i = 0; i < node_num; i++)
    {
        std::string line;
        std::getline( ifs, line );
        std::stringstream ss( line );
        float x, y, z, r;
        ss >> x >> y >> z >> r;
        metaballs.push_back( PBD::Metaball( glm::vec3( x, y, z ), r ) );
    }

    PBD::MetaballTreeNode root( metaballs, 0 );
    metaballs.clear();
    root.GetLeafNodes( metaballs );
    std::cout << "Metaball num: " << metaballs.size() << std::endl;


    std::unordered_set<PBD::Metaball, PBD::Metaball::Hash, PBD::Metaball::Pred> ballset;
    ballset.insert( metaballs.begin(), metaballs.end() );
    metaballs = std::vector<PBD::Metaball>( ballset.begin(), ballset.end() );

    _surface = std::make_unique<MetaballHalfEdgeMesh>( surface_path );

    _surface->mRenderConfig._draw_line = false;
    _surface->mRenderConfig._draw_face = true;
    _surface->mRenderConfig._face_mattype = RenderConfig::MatType::Texture;
    _surface->mRenderConfig._line_mattype = RenderConfig::MatType::VtxColor;
    _surface_tester.SetSurface( _surface.get() );
    //SampleFromSurface();

    _ball_mesh = std::make_unique<HalfEdgeMesh>( "res/models/ball960.obj" );
    _ball_mesh->Normalize();
    _ball_mesh->UpdatePosBuffer();
    _ball_mesh->UpdateAttrBuffer();

    _hand1 = std::make_unique<HalfEdgeMesh>( "res/models/hand1.obj" );
    _hand2 = std::make_unique<HalfEdgeMesh>( "res/models/hand2.obj" );
    _hand1->Normalize();
    _hand2->Normalize();
    _hand1->UpdatePosBuffer();
    _hand2->UpdatePosBuffer();
    _hand1->mTransform.SetScale( glm::vec3( 0.2f ) );
    _hand2->mTransform.SetScale( glm::vec3( 0.2f ) );
    _hand1->mRenderConfig._face_mattype = RenderConfig::MatType::PlainColor;
    _hand2->mRenderConfig._face_mattype = RenderConfig::MatType::PlainColor;
    _hand1->_material_main->mDiffuseColor = glm::vec3( 0.5, 0.5, 1.0 );
    _hand2->_material_main->mDiffuseColor = glm::vec3( 0.5, 0.5, 1.0 );
    _hand1->_material_main->mAlpha = 0.5f;
    _hand2->_material_main->mAlpha = 0.5f;

    _ball_orit_lines = std::make_unique<GLLineSegment>();
    _ball_orit_lines->ReservePoint( _metaballs.size() * 2 );

    _skin_ball_buffer = std::make_unique<ShaderStorageBuffer>( nullptr, 0, GL_DYNAMIC_DRAW );
    InitVertices();
    CreateTopo();
    InitConstraints();
    CreateSurfaceMapping2();
    _skin_vtx_buffer = std::make_unique<ShaderStorageBuffer>(
        _vtx_skinning_table.data(), sizeof( _vtx_skinning_table[0] ) * _vtx_skinning_table.size(), GL_DYNAMIC_DRAW );

    _bvh = std::make_unique<BVHTree_Metaball>( this );
}

void MetaballModel::Update()
{
    UpdateSkinInfoBuffer();

    if (_show_q)
    {
        _ball_orit_lines->Clear();
        for (auto& ball : _metaballs)
        {
            if (ball.visible)
            {
                if (_show_restpos)
                {
                    glm::vec3 px = glm::normalize( glm::rotate( ball.q, glm::vec3( 1, 0, 0 ) ) );
                    glm::vec3 py = glm::normalize( glm::rotate( ball.q, glm::vec3( 0, 1, 0 ) ) );
                    glm::vec3 pz = glm::normalize( glm::rotate( ball.q, glm::vec3( 0, 0, 1 ) ) );
                    _ball_orit_lines->AddPoint( ball.x0 );
                    _ball_orit_lines->AddPoint( ball.x0 + px * 0.3f );
                    _ball_orit_lines->AddPoint( ball.x0, glm::vec3( 0, 1, 0 ) );
                    _ball_orit_lines->AddPoint( ball.x0 + py * 0.3f, glm::vec3( 0, 1, 0 ) );
                    _ball_orit_lines->AddPoint( ball.x0, glm::vec3( 0, 0, 1 ) );
                    _ball_orit_lines->AddPoint( ball.x0 + pz * 0.3f, glm::vec3( 0, 0, 1 ) );
                }
                else
                {
                    glm::vec3 px = glm::normalize( glm::rotate( ball.q, glm::vec3( 1, 0, 0 ) ) );
                    glm::vec3 py = glm::normalize( glm::rotate( ball.q, glm::vec3( 0, 1, 0 ) ) );
                    glm::vec3 pz = glm::normalize( glm::rotate( ball.q, glm::vec3( 0, 0, 1 ) ) );
                    _ball_orit_lines->AddPoint( ball.x );
                    _ball_orit_lines->AddPoint( ball.x + px * 0.3f );
                    _ball_orit_lines->AddPoint( ball.x, glm::vec3( 0, 1, 0 ) );
                    _ball_orit_lines->AddPoint( ball.x + py * 0.3f, glm::vec3( 0, 1, 0 ) );
                    _ball_orit_lines->AddPoint( ball.x, glm::vec3( 0, 0, 1 ) );
                    _ball_orit_lines->AddPoint( ball.x + pz * 0.3f, glm::vec3( 0, 0, 1 ) );
                }
            }
        }
        _ball_orit_lines->UpdateMem();
    }

    if (_mode == Mode::Hand)
    {
        glm::vec2 cursor = Input::GetMousePosition();
        cursor.y = Camera::current->_viewport_size.y - cursor.y;
        Ray ray = Camera::current->ScreenPointToRay( cursor );
        IntersectionRec rec;
        auto it = MetaballTester::NearestRayIntersection( _metaballs.begin(), _metaballs.end(), ray, &rec );
        if (it != _metaballs.end() && _hold_idx == -1)
        {
            _hand1->mTransform.SetPos( rec.p + rec.normal * 0.05f );
            _hand1->mTransform.LookAt( rec.p - rec.normal );
            _hand2->mTransform.SetPos( rec.p + rec.normal * 0.05f );
            _hand2->mTransform.LookAt( rec.p - rec.normal );
            _hold_dist = _hand1->mTransform.GetPosition() - it->x;
        }
        if (_hold_idx != -1)
        {
            _hand1->mTransform.SetPos( _metaballs[_hold_idx].x + _hold_dist );
            _hand2->mTransform.SetPos( _metaballs[_hold_idx].x + _hold_dist );
        }
        if (Input::IsMouseButtonDown( Input::MouseButton::Left ))
        {
            if (it != _metaballs.end())
            {
                _hold_idx = std::distance( _metaballs.begin(), it );
                _start_pos = Input::GetMousePosition();

                //show weight
                for (int i = 0; i < _surface->GetVertexNumber(); i++)
                {
                    if (_surface->_vertices[i].edge == -1)
                        continue;

                    auto& weights = _vtx_skinning_table[i].weights;
                    auto& indices = _vtx_skinning_table[i].indices;
                    float x = 0.f;
                    for (int j = 0; j < _vtx_skinning_table[i].weights.size(); j++)
                    {
                        if (indices[j] == _hold_idx)
                        {

                            float d = glm::abs( glm::distance( _metaballs[indices[j]].x0, _surface->GetRestPos( i ) ) - _metaballs[indices[j]].r );
                            x = 1.f / glm::clamp( d, 0.00001f, 999999.f );
                            break;
                        }
                    }
                    tinycolormap::Color color = tinycolormap::GetHeatColor( x );
                    auto [cr, cg, cb] = color.data;
                    _surface->SetVtxColor( i, glm::vec3( cr, cg, cb ) );
                }
            }
        }
        if (Input::IsMouseButtonHeld( Input::MouseButton::Left ))
        {
            if (_hold_idx > 0)
            {
                auto& cam_trans = Camera::current->mTransform;
                glm::vec2 delta_cursor = Input::GetMousePosition() - _start_pos;
                glm::vec3 move( -delta_cursor.x / Camera::current->_viewport_size.x, -delta_cursor.y / Camera::current->_viewport_size.y, 0.f );
                move = cam_trans.Left() * move.x + cam_trans.Up() * move.y;
                _extra_force[_hold_idx] = move * _pull_force * 2000.f * _metaballs[_hold_idx].m;
            }
        }
        if (Input::IsMouseButtonUp( Input::MouseButton::Left ))
        {
            _hold_idx = -1;
            _extra_force.clear();
        }
    }

    if (_mode == Mode::Scalpel)
    {
        if (Input::IsMouseButtonUp( Input::MouseButton::Left ))
        {
            if (_is_cutting)
            {
                _is_cutting = false;
            }
        }
        if (Input::IsMouseButtonHeld( Input::MouseButton::Left ))
        {

        }
    }

    if (Input::IsKeyDown( Input::Key::P ))
    {
        _simulation = !_simulation;
    }
    _surface->Update();
}

void MetaballModel::PhysicalUpdate( float dt )
{
    size_t ball_num = _metaballs.size();

    //ESTIMATION
    for (size_t i = 0; i < ball_num; i++)
    {
        auto& ball = _metaballs[i];
        if (!ball.valid)
        {
            continue;
        }
        glm::vec3 ext_force( 0.0f );
        ext_force += glm::vec3( 0, -1, 0 ) * 9.8f * ball.m;
        if (_extra_force.find( i ) != _extra_force.end())
        {
            ext_force += _extra_force[i];
        }

        ball.v_pred = ball.v + dt * ext_force / ball.m;

        ball.x_pred = ball.x;

        float len_w = glm::length( ball.w );
        if (len_w > 0.001f)
        {
            ball.q_pred = glm::normalize( glm::quat( glm::cos( len_w * dt / 2.f ), ball.w / len_w * glm::sin( len_w * dt / 2.f ) ) * ball.q );
        }
        else
        {
            ball.q_pred = ball.q;
        }
        ball.q_pred = glm::normalize( ball.q_pred );
    }

    //DAMPING
    if (_point_damping)
    {
        for (auto& ball : _metaballs)
        {
            if (!ball.valid)
                continue;

            glm::vec3 v = ball.v_pred;
            float len = glm::length( ball.v_pred );
            if (glm::FloatEqual( len, 0.f, 1e-3 ))
            {
                ball.v_pred = glm::vec3( 0.f );
            }
            else
            {
                float dv = len * glm::min( 1.0f, _damping_coff * dt / ball.m );
                ball.v_pred *= (1.f - dv / len);
            }

            if (!glm::AllFloatNumValid( ball.v_pred ) || glm::length2( ball.v_pred ) > glm::length2( v ))
                __debugbreak();
        }
    }

    if (_line_damping)
    {
        for (const auto& C : _stretch_consts)
        {
            Metaball& ball0 = _metaballs[C._indices[0]];
            Metaball& ball1 = _metaballs[C._indices[1]];

            if (!ball0.valid || !ball1.valid)
            {
                continue;
            }
            glm::vec3 diff = ball1.x_pred - ball0.x_pred;
            if (glm::length2( diff ) < 0.00001f)
            {
                continue;
            }
            diff = glm::normalize( diff );
            float v0 = glm::dot( diff, ball0.v_pred );
            float v1 = glm::dot( diff, ball1.v_pred );
            float dv0 = (v1 - v0) * glm::min( 0.5f, _damping_coff * dt / ball0.m );
            float dv1 = (v0 - v1) * glm::min( 0.5f, _damping_coff * dt / ball1.m );
            ball0.v_pred += diff * dv0;
            ball1.v_pred += diff * dv1;

            if (!glm::AllFloatNumValid( ball0.v_pred ))
            {
                __debugbreak();
            }
        }
    }

    if (_total_damping)
    {
        glm::vec3 Xcm( 0.f );
        glm::vec3 Vcm( 0.f );
        float M_sum = 0.f;
        for (Metaball& ball : _metaballs)
        {
            if (!ball.valid)
                continue;
            Xcm += ball.x_pred * ball.m;
            Vcm += ball.v_pred * ball.m;
            M_sum += ball.m;
        }
        Xcm /= M_sum;
        Vcm /= M_sum;

        glm::vec3 L( 0.f );
        glm::mat3 I( 0.f );
        for (Metaball& ball : _metaballs)
        {
            if (!ball.valid)
                continue;

            glm::vec3 r = ball.x_pred - Xcm;
            L += glm::cross( r, ball.v_pred * ball.m );

            glm::mat3 r_ = glm::mat3( { 0, r.z, -r.y }, { -r.z, 0, r.x }, { r.y, -r.x, 0 } );
            I += r_ * glm::transpose( r_ ) * ball.m;
        }

        //angular velocity
        glm::vec3 w = glm::inverse( I ) * L;

        for (Metaball& ball : _metaballs)
        {
            if (!ball.valid)
                continue;
            glm::vec3 dv = Vcm + glm::cross( w, ball.x_pred - Xcm ) - ball.v_pred;
            ball.v_pred += _damping_coff * dv;
        }
    }

    for (const auto& cons : _stretch_consts)
    {
        Metaball& ball0 = _metaballs[cons._indices[0]];
        Metaball& ball1 = _metaballs[cons._indices[1]];
        glm::quat q0 = glm::slerp( ball0.q_pred, ball1.q_pred, 0.5f );
        glm::quat q1 = glm::slerp( ball1.q_pred, ball0.q_pred, 0.5f );
        ball0.q_pred = q0;
        ball1.q_pred = q1;
    }

    //SET POSITION
    for (Metaball& ball : _metaballs)
    {
        ball.x_pred = ball.x + dt * ball.v_pred;
    }


    //GEN COLLISION CONSTRAINTS
    _colli_consts.clear();
    std::vector<RigidStatic*> rigid_bodys = Scene::active->GetAllChildOfType<RigidStatic>();
    for (RigidStatic* rigid : rigid_bodys)
    {
        for (size_t i = 0; i < ball_num; i++)
        {
            if (!_metaballs[i].valid)
                continue;
            auto& ball = _metaballs[i];

            auto result = rigid->CheckBall( ball.x_pred, ball.r, i );
            _colli_consts.insert( _colli_consts.end(), result.begin(), result.end() );
        }
    }

    //SOLVE CONSTRAINTS
#ifdef _DEBUG
    int solver_count = 1;
#else
    int solver_count = 5;
#endif
    for (int solver_it = 0; solver_it < solver_count; solver_it++)
    {
        for (const auto& C : _stretch_consts)
        {
            int i1 = C._indices[0];
            int i2 = C._indices[1];
            if (!_metaballs[i1].valid || !_metaballs[i2].valid)
            {
                continue;
            }
            glm::vec3 p1 = _metaballs[i1].x_pred;
            glm::vec3 p2 = _metaballs[i2].x_pred;
            float w1 = 1.0f / _metaballs[i1].m;
            float w2 = 1.0f / _metaballs[i2].m;
            glm::vec3 n = glm::normalize( p1 - p2 );
            float s = (glm::distance( p1, p2 ) - C._d) / (w1 + w2);
            glm::vec3 dP1 = -w1 * s * n;
            glm::vec3 dP2 = w2 * s * n;
            _metaballs[i1].x_pred += dP1 * (float)(1 - glm::pow( 1 - _stretch_stiffness, solver_it + 1 ));
            _metaballs[i2].x_pred += dP2 * (float)(1 - glm::pow( 1 - _stretch_stiffness, solver_it + 1 ));
    }


        //std::vector<glm::vec3> temp( _metaballs.size() );
        //for (const auto& C : _laplacian_consts)
        //{
        //    Metaball& ball = _metaballs[C.index];
        //    if (!ball.valid)
        //    {
        //        continue;
        //    }
        //    glm::vec3 center( 0.f );
        //    for (int n : ball.neighbors)
        //    {
        //        if (!_metaballs[n].valid)
        //            continue;
        //        center += _metaballs[n].x_pred;
        //    }
        //    center /= ball.neighbors.size();

        //    glm::vec3 move = C.L + center - ball.x_pred;
        //    temp[C.index] = ball.x_pred + (move * _laplacian_stiffness / ball.m);

        //    if (!glm::AllFloatNumValid( temp[C.index] ))
        //        __debugbreak();
        //}
        //for (int i = 0; i < temp.size(); i++)
        //{
        //    _metaballs[i].x_pred = temp[i];
        //}
        // 

        //Shape matching
        for (Metaball& ball : _metaballs)
        {
            if (!ball.valid)
                continue;

            float m_sum = ball.m;
            glm::vec3 cm = ball.m * ball.x_pred;
            glm::vec3 cm0 = ball.m * ball.x0;
            for (int j : ball.neighbors)
            {
                auto& b = _metaballs[j];
                m_sum += b.m;
                cm += b.m * _metaballs[j].x_pred;
                cm0 += b.m * b.x0;
            }
            cm /= m_sum;
            cm0 /= m_sum;

            glm::mat3 A1 = ball.m * glm::TensorProduct( ball.x_pred, ball.x0 );
            glm::mat3 A2 = ball.m * ball.r * ball.r * glm::toMat3( ball.q_pred );

            for (int j : ball.neighbors)
            {
                glm::mat3 a1 = _metaballs[j].m * glm::TensorProduct( _metaballs[j].x_pred, _metaballs[j].x0 );
                glm::mat3 a2 = _metaballs[j].m * (float)glm::pow( _metaballs[j].r, 2 ) * glm::toMat3( _metaballs[j].q_pred );
                A1 += a1;
                A2 += a2;
            }
            glm::mat3 Ac = m_sum * glm::TensorProduct( cm, cm0 );
            A1 -= Ac;

            glm::mat3 A = A1 + A2;

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
            ball.q_pred = glm::normalize( glm::toQuat( R ) );
            glm::vec3 goal = glm::toMat3( ball.q_pred ) * (ball.x0 - cm0) + cm;
            //ball.x_pred += (goal - ball.x_pred) * (float)(1 - glm::pow( 1 - _stretch_stiffness, solver_it + 1 ));
        }

        for (const auto& C : _colli_consts)
        {
            Metaball& ball = _metaballs[C._index];
            glm::vec3 p = ball.x_pred;
            float C_value = glm::dot( (p - C._pc), C._n ) - ball.r;
            if (C_value >= 0)
            {
                continue;
            }

            glm::vec3 grad = C._n;
            glm::vec3 dP = -grad * C_value;
            ball.x_pred += dP;

            glm::vec3 x_diff = ball.x_pred - ball.x;
            if (!glm::FloatEqual( glm::length( x_diff ), 0.f, 1e-5 ))
            {
                glm::vec3 x_norm = glm::dot( x_diff, C._n ) * C._n;
                glm::vec3 x_perp = x_diff - x_norm;
                glm::vec3 dx;

                if (glm::length( x_perp ) < 0.5f * glm::abs( C_value ))
                {
                    //static 
                    dx = x_perp;
                }
                else
                {
                    dx = x_perp * glm::min( 1.f, 0.3f * glm::abs( C_value ) / glm::length( x_perp ) );
                }
                dx *= ball.m;
                ball.x_pred -= dx;
            }
        }
}

    for (Metaball& ball : _metaballs)
    {
        if (!ball.valid)
            continue;
        ball.v = (ball.x_pred - ball.x) / dt;
        ball.x = ball.x_pred;
        glm::quat r = ball.q_pred * glm::inverse( ball.q );
        if (r.w < 0)
        {
            r = -r;
        }
        glm::vec3 axis = glm::axis( r );
        float angle = glm::angle( r );
        if (glm::abs( angle ) < 0.0001f)
        {
            ball.w = glm::vec3( 0.f );
        }
        else
        {
            ball.w = axis * angle / dt;
        }
        ball.q = ball.q_pred;
    }


    for (const auto& C : _colli_consts)
    {
        Metaball& ball = _metaballs[C._index];
        glm::vec3 n = C._n;
        glm::vec3 r2 = n * ball.r;
        ball.w += 0.5f * glm::cross( r2 / glm::length2( r2 ), ball.v - glm::cross( ball.w, r2 ) );
    }

}

void MetaballModel::Draw()
{
    if (_show_metaball)
    {
        for (auto& ball : _metaballs)
        {
            if (ball.valid && ball.visible)
            {
                _ball_mesh->_material_main->mDiffuseColor = ball.color;

                if (_show_restpos)
                {
                    _ball_mesh->mTransform.SetPos( ball.x0 );
                    _ball_mesh->mTransform.SetScale( glm::vec3( ball.r * 2 ) );
                    _ball_mesh->Draw();
                }
                else
                {
                    _ball_mesh->mTransform.SetPos( ball.x );
                    _ball_mesh->mTransform.SetScale( glm::vec3( ball.r * 2 ) );
                    _ball_mesh->Draw();
                }
                ball.color = glm::vec3( 0.8f );
            }
        }
    }
    if (_show_surface)
    {
        _surface->_show_restpos = _show_restpos;
        _skin_vtx_buffer->BindBufferBase( 0 );
        _skin_ball_buffer->BindBufferBase( 1 );
        _surface->Draw();
    }
    if (_show_q)
    {
        _ball_orit_lines->Draw();
    }

    //if (_mode == Mode::Hand)
    //{
    //    if (Input::IsMouseButtonHeld( Input::MouseButton::Left ))
    //    {
    //        _hand2->Draw();
    //    }
    //    else
    //    {
    //        _hand1->Draw();
    //    }
    //}

    //_bvh->Draw();
}

void MetaballModel::DrawGUI()
{
    ImGui::Begin( "metaball pbd" );
    ImGui::Checkbox( "simulate", &_simulation );
    ImGui::Checkbox( "show surface", &_show_surface );
    ImGui::Checkbox( "show balls", &_show_metaball );
    ImGui::DragFloat( "stiff", &_stretch_stiffness, 0.01f, 0.0f, 10.0f );
    ImGui::Checkbox( "point damp", &_point_damping );
    ImGui::Checkbox( "edge damp", &_line_damping );
    ImGui::DragFloat( "damp", &_damping_coff, 0.f, 1.f );
    ImGui::End();
}

void MetaballModel::SampleFromSurface()
{
    HalfEdgeSurfaceTester tester( _surface.get() );
    std::cout << "Uniform Sampling...\n";
    auto start_t = std::chrono::high_resolution_clock::now();
    AABB aabb;
    for (int i = 0; i < _surface->_vertices.size(); i++)
    {
        aabb.Expand( _surface->_vertices[i].pos );
    }
    float steplen = 0.06f;
    _metaballs.clear();

    const int dimx = (aabb.max_corner.x - aabb.min_corner.x) / steplen + 1;
    const int dimy = (aabb.max_corner.y - aabb.min_corner.y) / steplen + 1;
    const int dimz = (aabb.max_corner.z - aabb.min_corner.z) / steplen + 1;

    int count = 0;
    for (int ix = 0; ix < dimx; ++ix)
    {
        for (int iy = 0; iy < dimy; ++iy)
        {
            for (int iz = 0; iz < dimz; ++iz)
            {
                const float px = aabb.min_corner.x + ix * steplen;
                const float py = aabb.min_corner.y + iy * steplen;
                const float pz = aabb.min_corner.z + iz * steplen;
                const glm::vec3 x( px, py, pz );
                if (tester.PointIsInSurface( x ))
                {
                    _metaballs.push_back( PBD::Metaball( x, steplen * 0.8f ) );
                }
            }
        }
    }
}

void MetaballModel::Preprocess()
{
    //SphereMerging();
    //VacantSpaceFilling();
    //ElectroAttractOptimize( _surface->mPoints, _surface->mTriangles );
    //CreateTopo();

    //std::vector<Metaball> temp;
    //for (int i = 0; i < _metaballs.size(); i++)
    //{
    //    if (!_metaballs[i].neighbors.empty())
    //    {
    //        temp.push_back( _metaballs[i] );
    //    }
    //}
    //_metaballs = temp;


    InitVertices();
    //CreateTopo();
    InitConstraints();
    CreateSurfaceMapping2();

    //Shape matching
    /*for (int i = 0, size = _metaballs.size(); i < size; i++)
    {
        auto& ball = _metaballs[i];
        if (!ball.valid)
            continue;
        const auto& nei = _metaballs[i].neighbors;

        float m_sum = ball.m;
        glm::vec3 c = ball.m * ball.x;
        glm::vec3 rest_c = ball.m * ball.x0;
        glm::mat3 A_sum = 0.2f * ball.m * ball.r * ball.r * glm::toMat3( ball.q );
        glm::mat3 temp( 0.f );
        temp[0][0] += ball.x[0] * ball.x0[0]; temp[1][0] += ball.x[0] * ball.x0[1]; temp[2][0] += ball.x[0] * ball.x0[2];
        temp[0][1] += ball.x[1] * ball.x0[0]; temp[1][1] += ball.x[1] * ball.x0[1]; temp[2][1] += ball.x[1] * ball.x0[2];
        temp[0][2] += ball.x[2] * ball.x0[0]; temp[1][2] += ball.x[2] * ball.x0[1]; temp[2][2] += ball.x[2] * ball.x0[2];
        glm::mat3 mx_sum = ball.m * temp;

        for (int j : nei)
        {
            auto& b = _metaballs[j];
            m_sum += b.m;
            c += b.m * b.x;
            rest_c += b.m * b.x0;
            A_sum += 0.2f * b.m * b.r * b.r * glm::toMat3( b.q );

            glm::mat3 temp( 0.f );
            temp[0][0] += b.x[0] * b.x0[0]; temp[1][0] += b.x[0] * b.x0[1]; temp[2][0] += b.x[0] * b.x0[2];
            temp[0][1] += b.x[1] * b.x0[0]; temp[1][1] += b.x[1] * b.x0[1]; temp[2][1] += b.x[1] * b.x0[2];
            temp[0][2] += b.x[2] * b.x0[0]; temp[1][2] += b.x[2] * b.x0[1]; temp[2][2] += b.x[2] * b.x0[2];
            mx_sum += b.m * temp;
        }

        c /= m_sum;
        rest_c /= m_sum;

        glm::mat3 c_cT( 0.f );
        c_cT[0][0] += c[0] * rest_c[0]; c_cT[1][0] += c[0] * rest_c[1]; c_cT[2][0] += c[0] * rest_c[2];
        c_cT[0][1] += c[1] * rest_c[0]; c_cT[1][1] += c[1] * rest_c[1]; c_cT[2][1] += c[1] * rest_c[2];
        c_cT[0][2] += c[2] * rest_c[0]; c_cT[1][2] += c[2] * rest_c[1]; c_cT[2][2] += c[2] * rest_c[2];
        c_cT *= m_sum;

        glm::mat3 A = A_sum + mx_sum - c_cT;

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

        ball.q = glm::toQuat( R );
        ball.rest_q = ball.q;
    }*/
}

void MetaballModel::SetHandMode()
{
    _mode = Mode::Hand;
}

void MetaballModel::SetScalpelMode()
{
    _mode = Mode::Scalpel;
}

void MetaballModel::SetNoneMode()
{
    _mode = Mode::None;
}

void MetaballModel::ShowMetaball( bool value )
{
    _show_metaball = value;
}

void MetaballModel::ShowSurface( bool value )
{
    _show_surface = value;
}

void MetaballModel::ShowOrit( bool value )
{
    _show_q = value;
}

bool MetaballModel::ShowMetaball() const
{
    return _show_metaball;
}

bool MetaballModel::ShowSurface() const
{
    return _show_surface;
}

bool MetaballModel::ShowOrit() const
{
    return _show_q;
}

void MetaballModel::SetStretchStiff( float value )
{
    _stretch_stiffness = value;
}

void MetaballModel::SetDampCoeff( float value )
{
    _damping_coff = value;
}

void MetaballModel::SwitchPointDamping( bool open )
{
    _point_damping = open;
}

void MetaballModel::SwitchLineDamping( bool open )
{
    _line_damping = open;
}

void MetaballModel::SwitchTotalDamping( bool open )
{
    _total_damping = open;
}

void MetaballModel::SwitchHangUp( bool value )
{
    _hang_up = value;
}

void MetaballModel::SwitchSimulation( bool value )
{
    _simulation = value;
}

void MetaballModel::SetShowRestPos( bool value )
{
    _show_restpos = value;
    _surface->_show_restpos = value;
    _surface->UpdatePosBuffer();
}

float MetaballModel::GetStretchSitff() const
{
    return _stretch_stiffness;
}

float MetaballModel::GetDampCoeff() const
{
    return _damping_coff;
}

bool MetaballModel::IsPointDamping() const
{
    return _point_damping;
}

bool MetaballModel::IsLineDamping() const
{
    return _line_damping;
}

bool MetaballModel::IsTotalDamping() const
{
    return _total_damping;
}

bool MetaballModel::IsHangUp() const
{
    return _hang_up;
}

bool MetaballModel::IsSimulating() const
{
    return _simulation;
}

void MetaballModel::PBDPrediction( float dt )
{
    if (!_simulation)
        return;
    size_t ball_num = _metaballs.size();

    //ESTIMATION
    for (size_t i = 0; i < ball_num; i++)
    {
        auto& ball = _metaballs[i];
        if (!ball.valid)
        {
            continue;
        }
        glm::vec3 ext_force( 0.0f );
        ext_force += glm::vec3( 0, -1, 0 ) * 9.8f * ball.m;
        if (_extra_force.find( i ) != _extra_force.end())
        {
            ext_force += _extra_force[i];
        }

        ball.v_pred = ball.v + dt * ext_force / ball.m;

        ball.x_pred = ball.x;

        float len_w = glm::length( ball.w );

        if (len_w > 1e-5f)
        {
            ball.q_pred = glm::normalize( glm::quat( glm::cos( len_w * dt / 2.f ), ball.w / len_w * glm::sin( len_w * dt / 2.f ) ) * ball.q );
        }
        else
        {
            ball.q_pred = ball.q;
        }
        ball.q_pred = glm::normalize( ball.q_pred );
    }

    //DAMPING
    if (_point_damping)
    {
        for (auto& ball : _metaballs)
        {
            if (!ball.valid)
                continue;

            glm::vec3 v = ball.v_pred;
            float len = glm::length( ball.v_pred );
            if (glm::FloatEqual( len, 0.f, 1e-5 ))
            {
                ball.v_pred = glm::vec3( 0.f );
            }
            else
            {
                float dv = len * glm::min( 1.0f, _damping_coff * dt / ball.m );
                ball.v_pred *= (1.f - dv / len);
            }

        }
    }

    if (_line_damping)
    {
        for (const auto& C : _stretch_consts)
        {
            Metaball& ball0 = _metaballs[C._indices[0]];
            Metaball& ball1 = _metaballs[C._indices[1]];

            if (!ball0.valid || !ball1.valid)
            {
                continue;
            }
            glm::vec3 diff = ball1.x_pred - ball0.x_pred;
            if (glm::length2( diff ) < 0.0001f)
            {
                continue;
            }
            diff = glm::normalize( diff );
            float v0 = glm::dot( diff, ball0.v_pred );
            float v1 = glm::dot( diff, ball1.v_pred );
            float dv0 = (v1 - v0) * glm::min( 0.5f, _damping_coff * dt / ball0.m );
            float dv1 = (v0 - v1) * glm::min( 0.5f, _damping_coff * dt / ball1.m );
            ball0.v_pred += diff * dv0;
            ball1.v_pred += diff * dv1;

        }
    }

    if (_total_damping)
    {
        glm::vec3 Xcm( 0.f );
        glm::vec3 Vcm( 0.f );
        float M_sum = 0.f;
        for (Metaball& ball : _metaballs)
        {
            if (!ball.valid)
                continue;
            Xcm += ball.x_pred * ball.m;
            Vcm += ball.v_pred * ball.m;
            M_sum += ball.m;
        }
        Xcm /= M_sum;
        Vcm /= M_sum;

        glm::vec3 L( 0.f );
        glm::mat3 I( 0.f );
        for (Metaball& ball : _metaballs)
        {
            if (!ball.valid)
                continue;

            glm::vec3 r = ball.x_pred - Xcm;
            L += glm::cross( r, ball.v_pred * ball.m );

            glm::mat3 r_ = glm::mat3( { 0, r.z, -r.y }, { -r.z, 0, r.x }, { r.y, -r.x, 0 } );
            I += r_ * glm::transpose( r_ ) * ball.m;
        }

        //angular velocity
        glm::vec3 w = glm::inverse( I ) * L;

        for (Metaball& ball : _metaballs)
        {
            if (!ball.valid)
                continue;
            glm::vec3 dv = Vcm + glm::cross( w, ball.x_pred - Xcm ) - ball.v_pred;
            ball.v_pred += _damping_coff * dv;
        }
    }

    for (const auto& cons : _stretch_consts)
    {
        Metaball& ball0 = _metaballs[cons._indices[0]];
        Metaball& ball1 = _metaballs[cons._indices[1]];
        glm::quat q0 = glm::slerp( ball0.q_pred, ball1.q_pred, 0.5f );
        glm::quat q1 = glm::slerp( ball1.q_pred, ball0.q_pred, 0.5f );
        ball0.q_pred = q0;
        ball1.q_pred = q1;
    }

    //SET POSITION
    for (Metaball& ball : _metaballs)
    {
        ball.x_pred = ball.x + dt * ball.v_pred;
    }

    _bvh->Refit();
}

void MetaballModel::PBDCheckCollision( float dt )
{
    if (!_simulation)
        return;

    size_t ball_num = _metaballs.size();

    //GEN COLLISION CONSTRAINTS
    _colli_consts.clear();
    _ball_colli_consts.clear();
    std::vector<RigidStatic*> rigid_bodys = Scene::active->GetAllChildOfType<RigidStatic>();
    for (RigidStatic* rigid : rigid_bodys)
    {
        for (size_t i = 0; i < ball_num; i++)
        {
            if (!_metaballs[i].valid)
                continue;
            auto& ball = _metaballs[i];
            glm::vec3 p0 = ball.x;
            glm::vec3 p1 = ball.x_pred;
            float y0 = p0.y;
            float y1 = p1.y;
            float r = ball.r;
            float len = glm::distance( p0, p1 );

            IntersectionRec rec;
            int id;
            auto result = rigid->CheckBall( ball.x_pred, ball.r, i );
            _colli_consts.insert( _colli_consts.end(), result.begin(), result.end() );
        }
    }

    std::vector<PBD::MetaballModel*> metaball_models = Scene::active->GetAllChildOfType<PBD::MetaballModel>();

    for (auto model : metaball_models)
    {
        if (model == this)
            continue;

        if (!AABBIntersect( _bvh->BoundingBox(), model->BVH().BoundingBox() ))
            continue;

        for (int i = 0; i < ball_num; i++)
        {
            auto& ball = _metaballs[i];
            std::vector<CollisionInfo> infos;
            model->BVH().CheckBall( ball.x_pred, ball.r, &infos );
            for (auto& info : infos)
            {
                CollisionConstraint c( i, info.p, info.n, info.d, model->Ball( info.id ).m );
                _ball_colli_consts.insert( BallCollisionConstraint{ &ball, &model->_metaballs[info.id] } );
                //_colli_consts.push_back( c );
            }
            //for (auto& ball2 : model->BallList())
            //{
            //    float dist = ball.r + ball2.r;
            //    float dist2 = glm::distance( ball.x_pred, ball2.x_pred );
            //    glm::vec3 dir = glm::normalize( ball2.x_pred - ball.x_pred );
            //    if (dist2 < dist)
            //    {
            //        CollisionConstraint c( i,
            //            ball2.x_pred - dir * ball2.r,
            //            -dir, (dist - dist2) * 0.5f );
            //        _colli_consts.push_back( c );
            //    }
            //}
        }
    }

    return;
    //self-collision
    for (int i = 0; i < ball_num; i++)
    {
        auto& ball = _metaballs[i];
        std::vector<CollisionInfo> infos;
        _bvh->CheckBall( ball.x_pred, ball.r, &infos );
        for (auto& info : infos)
        {
            if (info.id != i && glm::distance2( ball.x0, _metaballs[info.id].x0 ) > ( ball.r + _metaballs[info.id].r ) * (ball.r + _metaballs[info.id].r))
            {
                CollisionConstraint c( i, info.p, info.n, info.d, _metaballs[info.id].m );
                _colli_consts.push_back( c );
            }
        }
    }
}

void MetaballModel::PBDSolveConstraints( float dt )
{
    if (!_simulation)
        return;

    //SOLVE CONSTRAINTS
#ifdef _DEBUG
    int solver_count = 1;
#else
    int solver_count = 10;
#endif
    for (int solver_it = 0; solver_it < solver_count; solver_it++)
    {
        auto start_t = std::chrono::high_resolution_clock::now();
        for (const auto& C : _stretch_consts)
        {
            int i1 = C._indices[0];
            int i2 = C._indices[1];
            glm::vec3 p1 = _metaballs[i1].x_pred;
            glm::vec3 p2 = _metaballs[i2].x_pred;
            float w1 = 1.0f / _metaballs[i1].m;
            float w2 = 1.0f / _metaballs[i2].m;
            glm::vec3 n = glm::normalize( p1 - p2 );
            float s = (glm::distance( p1, p2 ) - C._d) / (w1 + w2);

            glm::vec3 dP1 = -w1 * s * n;
            glm::vec3 dP2 = w2 * s * n;

            _metaballs[i1].x_pred += dP1 * (float)(1 - glm::pow( 1 - _stretch_stiffness, solver_it + 1 ));
            _metaballs[i2].x_pred += dP2 * (float)(1 - glm::pow( 1 - _stretch_stiffness, solver_it + 1 ));
    }

        //std::vector<glm::vec3> temp( _metaballs.size() );
        //for (const auto& C : _laplacian_consts)
        //{
        //    Metaball& ball = _metaballs[C.index];
        //    if (!ball.valid)
        //    {
        //        continue;
        //    }
        //    glm::vec3 center( 0.f );
        //    for (int n : ball.neighbors)
        //    {
        //        if (!_metaballs[n].valid)
        //            continue;
        //        center += _metaballs[n].x_pred;
        //    }
        //    center /= ball.neighbors.size();
        //    glm::vec3 move = glm::rotate( ball.q_pred, C.L ) + center - ball.x_pred;
        //    temp[C.index] = ball.x_pred + (move * (float)(1 - glm::pow( 1 - _laplacian_stiffness, solver_it + 1 )));
        //    if (!glm::AllFloatNumValid( temp[C.index] ))
        //        __debugbreak();
        //}
        //for (int i = 0; i < temp.size(); i++)
        //{
        //    _metaballs[i].x_pred = temp[i];
        //}

        //Shape matching
        //TODO: pre-computation

        for (Metaball& ball : _metaballs)
        {
            if (!ball.valid)
                continue;

            float m_sum = ball.m;
            glm::vec3 cm = ball.m * ball.x_pred;
            glm::vec3 cm0 = ball.m * ball.x0;
            for (int j : ball.neighbors)
            {
                auto& b = _metaballs[j];
                m_sum += b.m;
                cm += b.m * _metaballs[j].x_pred;
                cm0 += b.m * b.x0;
            }
            cm /= m_sum;
            cm0 /= m_sum;

            glm::mat3 A1 = ball.m * glm::TensorProduct( ball.x_pred, ball.x0 );
            glm::mat3 A2 = ball.m * ball.r * ball.r * glm::toMat3( ball.q_pred );

            for (int j : ball.neighbors)
            {
                glm::mat3 a1 = _metaballs[j].m * glm::TensorProduct( _metaballs[j].x_pred, _metaballs[j].x0 );
                glm::mat3 a2 = _metaballs[j].m * (float)glm::pow( _metaballs[j].r, 2 ) * glm::toMat3( _metaballs[j].q_pred );
                A1 += a1;
                A2 += a2;
            }
            glm::mat3 Ac = m_sum * glm::TensorProduct( cm, cm0 );
            A1 -= Ac;

            glm::mat3 A = A1 + A2;

            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 3; col++)
                {
                    //if (A[col][row] < 1e-5f)
                    //{
                    //    A[col][row] = 0;
                    //}
                }
            }
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
            ball.q_pred = glm::normalize( glm::toQuat( R ) );
            glm::vec3 goal = R * (ball.x0 - cm0) + cm;
            //ball.x_pred += (goal - ball.x_pred) * (float)(1 - glm::pow( 1 - _stretch_stiffness, solver_it + 1 ));

            //for (int j : ball.neighbors)
            //{
            //    Metaball& b = _metaballs[j];
            //    glm::vec3 g = R * (b.x0 - cm0) + cm;
            //    b.x_pred += (g - b.x_pred) * (float)(1 - glm::pow( 1 - _stretch_stiffness, solver_it + 1 ));
            //}

        }

        for (const auto& C : _ball_colli_consts)
        {
            Metaball& ball1 = *C.ball1;
            Metaball& ball2 = *C.ball2;
            float C_value = glm::length( ball1.x_pred - ball2.x_pred ) - (ball1.r + ball2.r);
            if (C_value >= 0)
                continue;

            glm::vec3 n = glm::normalize( ball1.x_pred - ball2.x_pred );
            float d = -C_value;
            ball1.x_pred += n * d * (1.f / ball1.m) / (1.f / ball1.m + 1.f / ball2.m);
            ball2.x_pred += -n * d * (1.f / ball2.m) / (1.f / ball1.m + 1.f / ball2.m);

            glm::vec3 x1_diff = ball1.x_pred - ball1.x;
            glm::vec3 x1_norm = glm::dot( x1_diff, n ) * n;
            glm::vec3 x1_perp = x1_diff - x1_norm;

            glm::vec3 x2_diff = ball2.x_pred - ball2.x;
            glm::vec3 x2_norm = glm::dot( x2_diff, n ) * n;
            glm::vec3 x2_perp = x2_diff - x2_norm;

            if (glm::length( x1_perp ) < 1 * glm::abs( C_value ))
            {
                //static 
                glm::vec3 dx = ((1.f / ball1.m) / (1.f / ball1.m + 1.f / ball2.m)) * x1_perp;
                ball1.x_pred -= dx;
                ball2.x_pred += ((1.f / ball2.m) / (1.f / ball1.m + 1.f / ball2.m)) * dx;
                ball1.color = glm::vec3( 0, 0, 1 );
                ball2.color = glm::vec3( 0, 0, 1 );
                //ball.q_pred = ball.q;
            }
            else
            {
                glm::vec3 dx = ((1.f / ball1.m) / (1.f / ball1.m + 1.f / ball2.m)) * x1_perp * glm::min( 1.f, 20 * glm::abs( C_value ) );
                ball1.x_pred -= dx;
                ball2.x_pred += ((1.f / ball2.m) / (1.f / ball1.m + 1.f / ball2.m)) * dx;
                ball1.color = glm::vec3( 1, 0, 0 );
                ball2.color = glm::vec3( 0, 0, 1 );
            }
        }

        for (const auto& C : _colli_consts)
        {
            Metaball& ball = _metaballs[C._index];
            glm::vec3 p = ball.x_pred;
            //p.y -= _metaballs[C._index].r;
            float C_value = glm::dot( (p - C._pc), C._n ) - ball.r;

            if (C_value >= 0)
            {
                continue;
            }

            glm::vec3 grad = C._n;
            glm::vec3 dP = grad * -C_value;
            ball.x_pred += dP;

            glm::vec3 x_diff = ball.x_pred - ball.x;
            glm::vec3 x_norm = glm::dot( x_diff, C._n ) * C._n;
            glm::vec3 x_perp = x_diff - x_norm;

            //Ks=1, Kd=20
            if (glm::length( x_perp ) < 1 * glm::abs( C_value ))
            {
                //static 
                ball.x_pred -= x_perp;
                ball.color = glm::vec3( 0, 0, 1 );
                //ball.q_pred = ball.q;
            }
            else
            {
                ball.x_pred -= x_perp * glm::min( 1.f, 20 * glm::abs( C_value ) );
                ball.color = glm::vec3( 1, 0, 0 );
            }
        }

        if (_hang_up)
        {
            for (const auto& C : _attach_consts)
            {
                Metaball& ball = _metaballs[C._index];
                ball.x_pred = C._p;
                ball.q_pred = glm::quat();
                ball.q = glm::quat();
            }
        }

        if (Input::IsKeyDown( Input::Key::F ))
        {
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_t);
            std::cout << duration.count() << std::endl;
        }
}

    for (Metaball& ball : _metaballs)
    {
        if (!ball.valid)
            continue;
        ball.v = (ball.x_pred - ball.x) / dt;
        ball.x = ball.x_pred;

        glm::quat r = ball.q_pred * glm::inverse( ball.q );
        if (r.w < 0)
        {
            r = -r;
        }
        glm::vec3 axis = glm::axis( r );
        float angle = glm::angle( r );
        if (glm::abs( angle ) < 0.0001f)
        {
            ball.w = glm::vec3( 0.f );
        }
        else
        {
            ball.w = axis * angle / dt;
        }
        ball.q = ball.q_pred;
    }

    for (const auto& C : _colli_consts)
    {
        Metaball& ball = _metaballs[C._index];

        glm::vec3 n = C._n;
        glm::vec3 v_norm = n * glm::dot( n, ball.v );
        glm::vec3 v_perp = ball.v - v_norm;
        float s_perp = glm::length( v_perp );
        float f = C._depth;
        f *= ball.m;

        //if (s_perp <= f)
        //{
        //    v_perp = glm::vec3( 0.f );
        //}
        //else
        //{
        //    v_perp = v_perp - (f * glm::normalize( v_perp ));
        //}
        //v_norm *= 0.8f;
        glm::vec3 ori_v = ball.v;
        //ball.v = v_perp - v_norm * 0.8f;

        glm::vec3 r2 = n * ball.r;
        //ball.w += 0.5f * glm::cross( r2 / glm::length2( r2 ), ori_v - glm::cross( ball.w, r2 ) );
    }

    //MapToSurface2();
}

void MetaballModel::CheckCursorRay( Ray r )
{
    CollisionInfo info;
    bool intersect = _bvh->CheckRay( r.o, r.d, &info );

}

Metaball& MetaballModel::Ball( int handle )
{
    return _metaballs[handle];
}

std::vector<Metaball>& MetaballModel::BallList()
{
    return _metaballs;
}

HalfEdgeMesh& MetaballModel::Surface()
{
    return *_surface;
}

const BVHTree_Metaball& MetaballModel::BVH() const
{
    return *_bvh;
}

void MetaballModel::CreateSurfaceMapping()
{
    _attach_ball_table.clear();
    _vtx_skinning_table.resize( _surface->GetVertexNumber() );
    for (int i = 0; i < _surface->GetVertexNumber(); i++)
    {
        if (_surface->_vertices[i].edge == -1)
        {
            _attach_ball_table.push_back( std::vector<int>() );
            _vtx_skinning_table[i].disp = glm::vec3( 0.f );
            continue;
        }
        std::vector<int> attach_balls;
        glm::vec3 center( 0.f );
        float gauss_sum = 0.f;
        int min_id = -1;
        float min_dist = 99999.f;
        for (int j = 0; j < _metaballs.size(); j++)
        {
            if (!_metaballs[j].valid)
            {
                continue;
            }
            //if (glm::dot( _surface->GetNormal(_surface->_vertices[i].edge), _metaballs[j].rest_pos - _surface->GetRestPos( i ) ) > 0)
            //{
            //    continue;
            //}
            float d = glm::distance( _metaballs[j].x0, _surface->GetRestPos( i ) );
            float r = _metaballs[j].r;
            float gauss = glm::exp( -((d - r) * (d - r)) / (r * r) );
            if (gauss >= 0.7f)
            {
                attach_balls.push_back( j );
                center += gauss * _metaballs[j].x0;
                gauss_sum += gauss;
            }
            if (glm::abs( d - r ) < min_dist)
            {
                min_dist = glm::abs( d - r );
                min_id = j;
            }
        }
        if (attach_balls.empty())
        {
            center = _metaballs[min_id].x0;
            gauss_sum = 1;
            attach_balls.push_back( min_id );
        }
        center /= gauss_sum;
        glm::vec3 disp = _surface->GetRestPos( i ) - center;
        _attach_ball_table.push_back( attach_balls );
        _vtx_skinning_table[i].disp = disp;
    }
}

void MetaballModel::CreateSurfaceMapping( const Rect& rect )
{
    _attach_ball_table.clear();
    _vtx_skinning_table.resize( _surface->GetVertexNumber() );
    for (int i = 0; i < _surface->GetVertexNumber(); i++)
    {
        if (_surface->_vertices[i].edge == -1)
        {
            _attach_ball_table.push_back( std::vector<int>() );
            _vtx_skinning_table[i].disp = glm::vec3( 0.f );
            continue;
        }
        std::vector<int> attach_balls;
        glm::vec3 center( 0.f );
        float gauss_sum = 0.f;
        int min_id = -1;
        float min_dist = 99999.f;

        bool upper = glm::dot( rect.Normal(), _surface->GetPosition( i ) - rect.Center() ) > 0;
        bool lower = glm::dot( rect.Normal(), _surface->GetPosition( i ) - rect.Center() ) < 0;
        bool in_rect = rect.PointInRect( _surface->GetPosition( i ) );

        float gauss_min = 999999.f;

        for (int j = 0; j < _metaballs.size(); j++)
        {
            if (!_metaballs[j].valid)
            {
                continue;
            }
            if (_surface->_vertices[i].topo >= 1 && glm::dot( rect.Normal(), _metaballs[j].x - rect.Center() ) > 0)
            {
                continue;
            }
            else if (_surface->_vertices[i].topo <= -1 && glm::dot( rect.Normal(), _metaballs[j].x - rect.Center() ) < 0)
            {
                continue;
            }
            else if (_surface->_vertices[i].topo == 0 && in_rect)
            {
                if (upper && glm::dot( rect.Normal(), _metaballs[j].x - rect.Center() ) < 0)
                {
                    continue;
                }
                else if (lower && glm::dot( rect.Normal(), _metaballs[j].x - rect.Center() ) > 0)
                {
                    continue;
                }
                //if (LinesegRectIntersect( _metaballs[j].c, _surface->GetPosition( i ), rect, nullptr ))
                //{
                //    continue;
                //}
            }

            float d = glm::distance( _metaballs[j].x0, _surface->GetRestPos( i ) );
            float r = _metaballs[j].r;
            float gauss = glm::exp( -((d - r) * (d - r)) / (r * r) );
            if (gauss < gauss_min)
            {
                gauss_min = gauss;
                min_id = j;
            }
            /*if (gauss >= 0.8f)
            {
                attach_balls.push_back( j );
                center += gauss * _metaballs[j].rest_pos;
                gauss_sum += gauss;
            }
            if (glm::abs( d - r ) < min_dist)
            {
                min_dist = glm::abs( d - r );
                min_id = j;
            }*/
        }
        if (attach_balls.empty())
        {
            center = _metaballs[min_id].x0;
            gauss_sum = 1;
            attach_balls.push_back( min_id );
        }
        center /= gauss_sum;
        glm::vec3 disp = _surface->GetRestPos( i ) - center;
        _attach_ball_table.push_back( std::move( attach_balls ) );
        _vtx_skinning_table[i].disp = disp;

    }

    for (auto& v : _surface->_vertices)
    {
        v.topo = 0;
    }

    std::cout << "Finish Create Surface Mapping" << std::endl;
}

void MetaballModel::CreateSurfaceMapping2()
{
    _vtx_skinning_table.resize( _surface->GetVertexNumber() );
#pragma omp parallel for
    for (int i = 0; i < _surface->GetVertexNumber(); i++)
    {
        _vtx_skinning_table[i].disp = _surface->GetRestPos( i );
        if (_surface->_vertices[i].edge == -1)
        {
            continue;
        }
        std::multimap<float, int, std::greater<float>> min_heap;

        for (int j = 0; j < _metaballs.size(); j++)
        {
            auto& ball = _metaballs[j];
            if (!ball.valid)
                continue;

            double dist = glm::abs( glm::distance( ball.x0, _surface->GetRestPos( i ) ) );
            double h = ball.r * 3;
            float w = glm::exp( -dist * dist / (ball.r * ball.r) );
            min_heap.insert( std::make_pair( w, j ) );
        }
        std::array<float, NEICOUNT> weights;
        std::array<int, NEICOUNT> indices;
        indices.fill( -1 );

        int k = 0;
        float weight_sum = 0.f;
        glm::vec3 cm( 0.f );
        for (auto& pair : min_heap)
        {
            weights[k] = pair.first;
            indices[k] = pair.second;
            k++;
            weight_sum += pair.first;
            if (k == NEICOUNT)
                break;
        }

        for (int j = 0; j < NEICOUNT; j++)
        {
            weights[j] /= weight_sum;
            cm += _metaballs[indices[j]].x0 * weights[j];
        }
        _vtx_skinning_table[i].indices = indices;
        _vtx_skinning_table[i].weights = weights;
        _vtx_skinning_table[i].disp = _surface->GetRestPos( i ) - cm;
    }
}

void MetaballModel::CreateSurfaceMapping2( const Rect& rect )
{
    _vtx_skinning_table.resize( _surface->GetVertexNumber() );
#pragma omp parallel for
    for (int i = 0; i < _surface->GetVertexNumber(); i++)
    {
        _vtx_skinning_table[i].disp = _surface->GetRestPos( i );
        if (_surface->_vertices[i].edge == -1)
        {
            continue;
        }
        bool upper = glm::dot( rect.Normal(), _surface->GetPosition( i ) - rect.Center() ) > 0;
        bool lower = glm::dot( rect.Normal(), _surface->GetPosition( i ) - rect.Center() ) < 0;
        bool in_rect = rect.PointInRect( _surface->GetPosition( i ) );

        std::multimap<float, int, std::greater<float>> min_heap;

        for (int j = 0; j < _metaballs.size(); j++)
        {
            auto& ball = _metaballs[j];
            if (!ball.valid)
                continue;
            if (_surface->_vertices[i].topo >= 1 && glm::dot( rect.Normal(), _metaballs[j].x - rect.Center() ) > 0)
                continue;
            else if (_surface->_vertices[i].topo <= -1 && glm::dot( rect.Normal(), _metaballs[j].x - rect.Center() ) < 0)
                continue;
            else if (_surface->_vertices[i].topo == 0 && in_rect)
            {
                if (upper && glm::dot( rect.Normal(), _metaballs[j].x - rect.Center() ) < 0)
                    continue;
                else if (lower && glm::dot( rect.Normal(), _metaballs[j].x - rect.Center() ) > 0)
                    continue;
            }
            double dist = glm::abs( glm::distance( ball.x0, _surface->GetRestPos( i ) )/* - ball.r*/ );
            double h = ball.r * 3;
            dist = glm::pow( h * h - dist * dist, 3.0 ) / glm::pow( h, 9.0 );
            dist = glm::max( 0.0, dist );
            //dist = glm::exp( -dist * dist  /*/ (ball.r * ball.r)*/ );
            min_heap.insert( std::make_pair( dist, j ) );
        }
        std::array<float, NEICOUNT> weights;
        std::array<int, NEICOUNT> indices;
        indices.fill( -1 );

        int k = 0;
        float weight_sum = 0.f;
        glm::vec3 cm( 0.f );

        for (auto& pair : min_heap)
        {
            weights[k] = pair.first;
            indices[k] = pair.second;
            k++;
            weight_sum += pair.first;
            if (k == NEICOUNT)
                break;
        }

        for (int j = 0; j < NEICOUNT; j++)
        {
            weights[j] /= weight_sum;
            cm += _metaballs[indices[j]].x0 * weights[j];
        }
        _vtx_skinning_table[i].indices = indices;
        _vtx_skinning_table[i].weights = weights;
        _vtx_skinning_table[i].disp = _surface->GetRestPos( i ) - cm;
    }
    _skin_vtx_buffer->UpdateData( _vtx_skinning_table.data(), sizeof( _vtx_skinning_table[0] ) * _vtx_skinning_table.size(), GL_DYNAMIC_DRAW );
}

void MetaballModel::MapToSurface()
{
    for (int i = 0, s = _surface->GetVertexNumber(); i < s; i++)
    {
        if (_surface->_vertices[i].edge == -1)
        {
            continue;
        }
        glm::vec3 point = _surface->GetPosition( i );
        std::vector<int>& attached = _attach_ball_table[i];
        float gauss_sum = 0.f;
        glm::vec3 center( 0.f );
        glm::quat q = glm::quat( 0.f, 0.f, 0.f, 0.f );
        for (int ball : attached)
        {
            if (!_metaballs[ball].valid)
            {
                continue;
            }
            float d = glm::distance( _metaballs[ball].x, point );
            float r = _metaballs[ball].r;
            float gauss = glm::exp( -((d - r) * (d - r) / (r * r)) );
            gauss = glm::max( gauss, 1e-5f );
            gauss_sum += gauss;
            center += gauss * _metaballs[ball].x;
            q += _metaballs[ball].q * gauss;
        }
        center /= gauss_sum;
        q = glm::normalize( q );

        glm::vec3 d = glm::rotate( q, _vtx_skinning_table[i].disp );

        glm::vec3 final_pos = center + d;

        if (!glm::AllFloatNumValid( final_pos ))
        {
            __debugbreak();
        }

        _surface->SetPosition( i, final_pos );
    }
}

void MetaballModel::MapToSurface2()
{
    for (int i = 0, s = _surface->GetVertexNumber(); i < s; i++)
    {
        if (_surface->_vertices[i].edge == -1)
        {
            continue;
        }

        const auto& info = _vtx_skinning_table[i];

        glm::mat4 T( 0.0f );
        glm::vec3 cm( 0.f );
        for (int j = 0; j < info.weights.size(); j++)
        {
            auto& ball = _metaballs[info.indices[j]];
            cm += info.weights[j] * ball.x;
            T += info.weights[j] * glm::toMat4( glm::normalize( ball.q ) );
        }
        glm::vec3 displace = glm::vec3( T * glm::vec4( info.disp, 1.0f ) );
        _surface->SetPosition( i, cm + displace );
    }
}

void MetaballModel::UpdateSkinInfoBuffer()
{
    _ball_skinning_infos.resize( _metaballs.size() );
    for (int i = 0, size = _metaballs.size(); i != size; i++)
    {
        _ball_skinning_infos[i] = BallSkinningInfo{ glm::vec4( _metaballs[i].x, 1.0f ), glm::normalize( _metaballs[i].q ) };
    }
    _skin_ball_buffer->UpdateData( _ball_skinning_infos.data(), _ball_skinning_infos.size() * sizeof( _ball_skinning_infos[0] ), GL_DYNAMIC_DRAW );
}

void MetaballModel::RadiusAdjustment()
{
    for (auto& ball : _metaballs)
    {
        float min_dist = _surface_tester.MinDistToSurface( ball.x );
        if (ball.r > min_dist)
        {
            ball.r = min_dist;
        }
    }
}

void MetaballModel::VacantSpaceFilling()
{
    std::cout << "space filling: " << _metaballs.size();
    std::list<int> metaball_ids;
    for (int i = 0; i < _metaballs.size(); i++)
    {
        metaball_ids.push_back( i );
    }

    while (!metaball_ids.empty())
    {
        int id = metaball_ids.back();
        metaball_ids.pop_back();

        std::vector<int> neighbors = Neighbors( id, 12 );
        for (int neighbor : neighbors)
        {
            glm::vec3 c1 = _metaballs[id].x;
            glm::vec3 c2 = _metaballs[neighbor].x;
            float r1 = _metaballs[id].r;
            float r2 = _metaballs[neighbor].r;
            float dc1c2 = glm::distance( c1, c2 );
            glm::vec3 cnew = ((dc1c2 + r2 - r1) * c1 + (dc1c2 + r1 - r2) * c2) / (2 * dc1c2);

            bool cnew_in_ball = false;
            for (auto& ball : _metaballs)
            {
                if (glm::distance( cnew, ball.x ) < ball.r)
                {
                    cnew_in_ball = true;
                    break;
                }
            }

            if (!cnew_in_ball && _surface_tester.PointIsInSurface( cnew ))
            {
                float rnew = _surface_tester.MinDistToSurface( cnew );
                _metaballs.push_back( Metaball( cnew, rnew ) );
                metaball_ids.push_back( _metaballs.size() - 1 );
            }
        }
    }
    std::cout << " -> " << _metaballs.size() << std::endl;
}

void MetaballModel::SphereMerging()
{
    std::cout << "before merge: " << _metaballs.size() << std::endl;
    for (auto it = _metaballs.begin(); it != _metaballs.end();)
    {
        if (it->r < 0)
        {
            it++;
            continue;
        }
        bool found = false;
        for (auto it2 = std::next( it ); it2 != _metaballs.end(); it2++)
        {
            if (it2->r < 0)
            {
                continue;
            }
            float dist = glm::distance( it->x, it2->x );
            float min_r = glm::min( it->r, it2->r );
            if (dist < min_r)
            {
                glm::vec3 cnew = (it->x + it2->x) / 2.f;
                if (_surface_tester.PointIsInSurface( cnew ))
                {
                    float rnew = glm::min( glm::max( it->r, it2->r ), _surface_tester.MinDistToSurface( cnew ) );
                    it2->r = -1;
                    *it = Metaball( cnew, rnew );
                    found = true;
                }
            }
        }
        if (!found)
        {
            it++;
        }
    }

    _metaballs.erase( std::remove_if( _metaballs.begin(), _metaballs.end(),
        []( Metaball& ball ) { return ball.r < 0; } ), _metaballs.end() );
    std::cout << "after merge: " << _metaballs.size() << std::endl;
}

static const float voxel_step = 0.005f;
std::vector<glm::vec3> MetaballModel::Voxelize()
{
    AABB aabb;
    for (auto& ball : _metaballs)
    {
        aabb.Expand( ball.x + glm::vec3( ball.r ) );
        aabb.Expand( ball.x - glm::vec3( ball.r ) );
    }

    std::vector<glm::vec3> voxels;

    for (float x = aabb.min_corner.x; x <= aabb.max_corner.x; x += voxel_step)
    {
        std::cout << x << "\t";
        auto start_t = std::chrono::high_resolution_clock::now();
        for (float y = aabb.min_corner.y; y <= aabb.max_corner.y; y += voxel_step)
        {
            for (float z = aabb.min_corner.z; z <= aabb.max_corner.z; z += voxel_step)
            {
                glm::vec3 p( x, y, z );
                if (_surface_tester.PointIsInSurface( p ))
                {
                    voxels.push_back( p );
                }
            }
        }
        auto duration = std::chrono::high_resolution_clock::now() - start_t;
        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "ms" << std::endl;
    }

    std::cout << "voxel num: " << voxels.size() << std::endl;
    return voxels;
}

std::vector<glm::vec3> MetaballModel::GetHollowVoxel( const std::vector<glm::vec3>& voxels )
{
    std::vector<glm::vec3> v_hollow;
    for (auto& p : voxels)
    {
        bool in_sphere = false;
        for (const auto& ball : _metaballs)
        {
            if (glm::distance( p, ball.x ) <= ball.r)
            {
                in_sphere = true;
                break;
            }
        }
        if (!in_sphere)
        {
            v_hollow.push_back( p );
        }
    }
    return v_hollow;
}

void MetaballModel::ElectroAttractOptimize()
{
    std::vector<glm::vec3> voxels = Voxelize();
    std::vector<glm::vec3> v_hollow = GetHollowVoxel( voxels );
    while (true)
    {
        int v_sum = v_hollow.size();
        std::cout << v_sum << std::endl;
        std::vector<glm::vec3> forces( _metaballs.size() );
        for (int i = 0; i < v_hollow.size(); i++)
        {
            glm::vec3 voxel = v_hollow[i];

            auto nearest_metaball = std::min_element( _metaballs.begin(), _metaballs.end(),
                [&voxel]( const Metaball& lh, const Metaball& rh )
                {
                    return glm::distance( voxel, lh.x ) < glm::distance( voxel, rh.x );
                } );
            float min_dist = glm::distance( nearest_metaball->x, voxel );
            float threshold = glm::max( min_dist, 0.1f );

            std::vector<int> neighbor_balls;
            for (int j = 0; j < _metaballs.size(); j++)
            {
                if (glm::distance( _metaballs[j].x, voxel ) < _metaballs[j].r + threshold)
                {
                    neighbor_balls.push_back( j );
                }
            }

            for (int idx : neighbor_balls)
            {
                const Metaball& ball = _metaballs[idx];
                float neg_charge = 4.f / 3.f * glm::pi<float>() * ball.r * ball.r * ball.r;
                glm::vec3 F = glm::normalize( voxel - ball.x ) * neg_charge;
                forces[idx] += F;
            }
        }

        int k = 0;
        for (; k < forces.size(); k++)
        {
            glm::vec3 dir = glm::normalize( forces[k] );
            glm::vec3 new_pos = _metaballs[k].x + dir * voxel_step;
            if (_surface_tester.MinDistToSurface( new_pos ) >= _metaballs[k].r)
            {
                _metaballs[k].x = new_pos;
                v_hollow = GetHollowVoxel( voxels );
                if (v_hollow.size() < v_sum)
                {
                    v_sum = v_hollow.size();
                    break;
                }
            }
        }
        if (k == forces.size())
        {
            break;
        }

    }
}

void MetaballModel::CreateTopo()
{
    //static const int N = 16;

    //for (int i = 0, size = _metaballs.size(); i < size; i++)
    //{
    //    Metaball& ball = _metaballs[i];
    //    if (!ball.valid)
    //    {
    //        continue;
    //    }
    //    std::multimap<float, int> max_heap;
    //    ball.neighbors.clear();

    //    int min_i = -1;
    //    float min_d = FLT_MAX;
    //    for (int j = 0; j < size; j++)
    //    {
    //        if (i == j)
    //            continue;

    //        const Metaball& ball2 = _metaballs[j];
    //        if (!ball2.valid)
    //            continue;

    //        float d = glm::distance( ball.x0, ball2.x0 );
    //        float diff = d - (ball.r + ball2.r);
    //        if (diff < glm::min( ball.r, ball2.r ))
    //        {
    //            if (max_heap.size() < N || diff < max_heap.begin()->first)
    //            {
    //                max_heap.insert( std::make_pair( diff, j ) );
    //            }
    //            if (max_heap.size() > N)
    //            {
    //                max_heap.erase( std::prev( max_heap.end() ) );
    //            }
    //        }
    //    }

    //    for (auto& pair : max_heap)
    //    {
    //        ball.neighbors.push_back( pair.second );
    //    }
    //    //if (ball.neighbors.size() > N)
    //    //{
    //    //    std::sort( ball.neighbors.begin(), ball.neighbors.end(),
    //    //        [this, &ball]( int lh, int rh ) {
    //    //            return glm::distance2( _metaballs[lh].x0, ball.x0 ) < glm::distance2( _metaballs[rh].x0, ball.x0 );
    //    //        } );
    //    //    ball.neighbors.resize( N );
    //    //}
    //}
    static const int N = 999;
    static const int MIN_N = 8;

    std::unordered_set<IdxPair, IdxPair::Hash, IdxPair::Pred> idx_pairs;

    for (int i = 0, size = _metaballs.size(); i < size; i++)
    {
        Metaball& ball = _metaballs[i];
        if (!ball.valid)
            continue;

        std::multimap<float, int>  max_heap;

        ball.neighbors.clear();

        for (int j = 0; j < size; j++)
        {
            if (i == j)
                continue;

            const Metaball& ball2 = _metaballs[j];
            if (!ball2.valid)
                continue;

            float d = glm::distance( ball.x0, ball2.x0 );
            float diff = d - (ball.r + ball2.r);
            max_heap.insert( std::make_pair( diff, j ) );
        }

        int count = 0;
        for (auto& pair : max_heap)
        {
            if (count < MIN_N)
            {
                idx_pairs.insert( { i, pair.second } );
                count++;
            }
            else if (pair.first < 0)
            {
                idx_pairs.insert( { i, pair.second } );
            }
            else
            {
                break;
            }
        }
    }

    for (const IdxPair& pair : idx_pairs)
    {
        _metaballs[pair.i0].neighbors.push_back( pair.i1 );
        _metaballs[pair.i1].neighbors.push_back( pair.i0 );
    }
}

void MetaballModel::CreateTopo( const Rect& rect )
{
    static const int N = 32;
    for (int i = 0, size = _metaballs.size(); i < size; i++)
    {
        Metaball& ball = _metaballs[i];
        if (!ball.valid)
        {
            continue;
        }
        ball.neighbors.clear();

        int min_i = -1;
        float min_d = FLT_MAX;
        for (int j = 0; j < size; j++)
        {
            if (i == j)
            {
                continue;
            }
            const Metaball& ball2 = _metaballs[j];
            if (!ball2.valid)
            {
                continue;
            }
            glm::vec3 intersect_pos;
            if (LinesegRectIntersect( ball.x, ball2.x, rect, &intersect_pos ))
            {
                continue;
            }
            float d = glm::distance( ball.x0, ball2.x0 );
            if (d < (ball.r + ball2.r))
            {
                ball.neighbors.push_back( j );
            }
            if (d < min_d)
            {
                min_i = j;
                min_d = d;
            }
        }

        if (ball.neighbors.empty())
        {
            //ball.neighbors.push_back( min_i );
        }

        if (ball.neighbors.size() > N)
        {
            std::sort( ball.neighbors.begin(), ball.neighbors.end(),
                [this, &ball]( int lh, int rh ) {
                    return glm::distance2( _metaballs[lh].x0, ball.x0 ) < glm::distance2( _metaballs[rh].x0, ball.x0 );
                } );
            ball.neighbors.resize( N );
        }
    }
}

void MetaballModel::InitVertices()
{
    for (auto& ball : _metaballs)
    {
        if (!ball.valid)
            continue;
        float r = ball.r;
        double volume = (4.f / 3.f) * glm::pi<float>() * r * r * r;
        ball.m = volume * _density;
        ball.v = glm::vec3( 0.f );
    }

    UpdateSkinInfoBuffer();

    _extra_force.clear();

    /*
    Init Orientation
    */
    //std::unordered_set<int> processed_balls;
    //for (int i = 0; i < _metaballs.size(); i++)
    //{
    //    auto& ball = _metaballs[i];
    //    pcl::PointCloud<pcl::PointXYZ> cloud;
    //    for (int j = 0; j < _surface->_vertices.size(); j++)
    //    {
    //        glm::vec3 rest_x = _surface->_vertices[j].rest_pos;
    //        float dist = glm::distance( rest_x, ball.x0 );
    //        if (dist < ball.r)
    //        {
    //            cloud.push_back( pcl::PointXYZ( rest_x.x, rest_x.y, rest_x.z ) );
    //        }
    //    }
    //    if (cloud.size() > 0)
    //    {
    //        processed_balls.insert( i );
    //        // Placeholder for the 3x3 covariance matrix at each surface patch
    //        Eigen::Matrix3f covariance_matrix;
    //        // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    //        Eigen::Vector4f xyz_centroid;

    //        // Estimate the XYZ centroid
    //        pcl::compute3DCentroid( cloud, xyz_centroid );

    //        xyz_centroid = Eigen::Vector4f( ball.x0.x, ball.x0.y, ball.x0.z, 1.0f );

    //        // Compute the 3x3 covariance matrix
    //        computeCovarianceMatrix( cloud, xyz_centroid, covariance_matrix );

    //        Eigen::JacobiSVD<Eigen::Matrix3f> svd( covariance_matrix, Eigen::ComputeFullU );
    //        Eigen::Matrix3f eigen_vecs = svd.matrixU();

    //        Eigen::Quaternion<float> q( eigen_vecs );
    //        //q.normalize();
    //        ball.q0 = glm::quat( q.w(), q.x(), q.y(), q.z() );
    //        //ball.q = ball.q0;
    //    }
    //}

    //while (processed_balls.size() < _metaballs.size())
    //{
    //    for (int i = 0; i < _metaballs.size(); i++)
    //    {
    //        if (processed_balls.find( i ) != processed_balls.end())
    //        {
    //            continue;
    //        }
    //        auto& ball = _metaballs[i];
    //        glm::quat q( 0.f, 0.f, 0.f, 0.f );
    //        float weight_sum = 0.f;
    //        bool get_orit = false;
    //        for (auto& nei : ball.neighbors)
    //        {
    //            if (processed_balls.find( nei ) != processed_balls.end())
    //            {
    //                q += _metaballs[nei].q0 * _metaballs[nei].m;
    //                weight_sum += _metaballs[nei].m;
    //                get_orit = true;
    //            }
    //        }
    //        if (get_orit)
    //        {
    //            ball.q0 = q / weight_sum;
    //            //ball.q = ball.q0;
    //            processed_balls.insert( i );
    //        }
    //    }
    //}
}

void MetaballModel::InitConstraints()
{
    _stretch_consts.clear();
    for (int i = 0; i < _metaballs.size(); i++)
    {
        if (!_metaballs[i].valid)
        {
            continue;
        }
        const auto& conn_list = _metaballs[i].neighbors;
        for (int j : conn_list)
        {
            _stretch_consts.insert( StretchConstraint( i, j, _stretch_stiffness, glm::distance( _metaballs[i].x0, _metaballs[j].x0 ) ) );
        }
    }

    std::cout << "Stretch const: " << _stretch_consts.size() << std::endl;

    _laplacian_consts.clear();
    for (int i = 0; i < _metaballs.size(); i++)
    {
        glm::vec3 center{ 0.f };
        for (int j = 0, size = _metaballs[i].neighbors.size(); j < size; j++)
        {
            center += _metaballs[_metaballs[i].neighbors[j]].x0;
        }
        center /= _metaballs[i].neighbors.size();
        glm::vec3 L = _metaballs[i].x0 - center;
        _laplacian_consts.push_back( LaplacianConstraint{ i, L } );
    }

    _attach_consts.clear();
    for (int i = 0; i < _metaballs.size(); i++)
    {
        if (_metaballs[i].x0.x > 0.3f)
        {
            _attach_consts.push_back( AttachmentConstraint( i, _metaballs[i].x0 ) );
        }
    }
}

std::vector<int> MetaballModel::Neighbors( int metaball, int k ) const
{
    const auto& ball = _metaballs[metaball];
    std::unordered_set<int> neighbor_set;
    for (int i = 0; i < k; i++)
    {
        float dist = 999999.f;
        float min_i = 0;
        for (int b = 0; b < _metaballs.size(); b++)
        {
            if (neighbor_set.find( b ) != std::end( neighbor_set ) || b == metaball)
            {
                continue;
            }
            float cur_d = glm::distance( ball.x0, _metaballs[b].x0 ) - ball.r - _metaballs[b].r;
            if (cur_d < dist)
            {
                dist = cur_d;
                min_i = b;
            }
        }
        neighbor_set.insert( min_i );
    }
    if (neighbor_set.empty())
    {
        __debugbreak();
    }
    return std::vector<int>( neighbor_set.begin(), neighbor_set.end() );
}

std::vector<MeshlessPoint> MetaballModel::SamplePointsInBall( int ball_id, float step )
{
    AABB aabb;
    auto& ball = _metaballs[ball_id];
    aabb.Expand( ball.x + glm::vec3( ball.r ) );
    aabb.Expand( ball.x - glm::vec3( ball.r ) );

    glm::vec3 start_pos = aabb.min_corner;
    glm::vec3 end_pos = aabb.max_corner;
    std::vector<MeshlessPoint> points;
    for (float x = start_pos.x; x <= end_pos.x; x += step)
    {
        for (float y = start_pos.y; y <= end_pos.y; y += step)
        {
            for (float z = start_pos.z; z <= end_pos.z; z += step)
            {
                glm::vec3 p( x, y, z );
                float d = glm::distance( ball.x, p );

                if (d < ball.r)
                {
                    points.push_back( MeshlessPoint( p, 0, ball_id ) );
                    break;
                }
            }
        }
    }

    return points;
}

std::vector<BoundaryPoint> MetaballModel::SamplePointsInTriangle( glm::vec3& a, glm::vec3& b, glm::vec3& c ) const
{
    std::vector<BoundaryPoint> samples;

    glm::vec3 i = glm::normalize( b - a );
    glm::vec3 j = glm::normalize( c - a );
    float len_ab = glm::distance( a, b );
    float len_ac = glm::distance( a, c );
    glm::vec3 normal = glm::normalize( glm::cross( b - a, c - a ) );
    for (float k = 0; k <= len_ab; k += 0.03f)
    {
        for (float l = 0; l <= len_ac * (1.f - k / len_ab); l += 0.03f)
        {
            glm::vec3 point = a + k * i + l * j;
            samples.push_back( { point, normal, 1 } );
        }
    }
    return samples;
}
}

namespace PBD
{
MetaballTreeNode::MetaballTreeNode( const std::vector<Metaball>& ball_list, int idx )
{
    metaball = ball_list[idx];
    children.fill( nullptr );
    for (unsigned i = 1; i < 9; i++)
    {
        unsigned child_idx = idx * 8 + i;
        if (child_idx < ball_list.size() && ball_list[child_idx].r > 0)
        {
            children[i - 1] = new MetaballTreeNode( ball_list, child_idx );
        }
    }
}

MetaballTreeNode::~MetaballTreeNode()
{
    for (int i = 0; i < 8; i++)
    {
        delete children[i];
    }
}

void MetaballTreeNode::GetLeafNodes( std::vector<Metaball>& balls )
{
    bool have_child = false;
    for (int i = 0; i < 8; i++)
    {
        if (children[i])
        {
            have_child = true;
            children[i]->GetLeafNodes( balls );
        }
    }

    if (!have_child)
    {
        balls.push_back( metaball );
    }
}
}