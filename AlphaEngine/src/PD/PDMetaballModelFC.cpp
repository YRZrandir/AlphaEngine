#include "PDMetaballModelFC.h"
#include <cstdlib>
#include <iomanip>
#include <boost/locale.hpp>
#include <string>
#include <imgui/imgui.h>
#include <tinycolormap.hpp>
#include <omp.h>
#include <muParser.h>

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


namespace PD
{
PDMetaballModelFC::PDMetaballModelFC( PDMetaballModelConfig config )
    :_cfg( config )
{

}

void PDMetaballModelFC::Start()
{
    _surface = GetComponent<PDMetaballHalfEdgeMesh>();
    _mesh = std::make_unique<SphereMesh<Particle>>();
    _coarse_surface = std::make_unique<HalfEdgeMesh>( _cfg._coarse_surface );

    switch (_cfg._method)
    {
    case 0:
    {
        _mesh->CreateFromSurfaceVoroOptimize( _coarse_surface.get(), _cfg._coarse_surface, _cfg._nb_points, _cfg._nb_lloyd, _cfg._sample_dx );
        break;
    }
    case 1:
    {
        _mesh->CreateFromSurfaceUniform( _coarse_surface.get(), _cfg._sample_dx );
        break;
    }
    case 2:
    {
        _mesh->LoadFromSphereTreeFile( _cfg._metaball_path );
        break;
    }
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
    float r_avg = 0.f;
    for (int i = 0; i < _mesh->BallsNum(); ++i)
    {
        auto& ball = _mesh->Ball( i );
        float r = ball.r;
        ball.m = 4.f / 3.f * 3.14f * r * r * r * _cfg._density;
        m_max = std::max( m_max, ball.m );
        r_avg += r;
    }
    r_avg /= _mesh->BallsNum();
    for (int i = 0; i < _mesh->BallsNum(); ++i)
    {
        _mesh->Ball( i ).m_rel = _mesh->Ball( i ).m / m_max;
    }
    Init();

    _contacts_vis = std::make_unique<GLLineSegment>();
}

void PDMetaballModelFC::Init()
{
    int nb_points = _mesh->BallsNum();
    std::vector<Eigen::Triplet<Real, int>> m_triplets;
    std::vector<Eigen::Triplet<Real, int>> m_inv_triplets;
    _x.resize( 3, nb_points );
    _x0.resize( 3, nb_points );
    _v.resize( 3, nb_points );
    _fext.resize( 3, nb_points );
    _x_last.resize( 3, nb_points );
    _momentum.resize( 3, nb_points );
    _v.setZero();
    _fext.setZero();
    _bn_tilde.resize( 3, nb_points );
    _f.resize( 3, nb_points );
    _ksi.resize( 3, nb_points );
    _contact_counter.resize( nb_points, 0 );

    for (int i = 0; i < nb_points; ++i)
    {
        m_triplets.push_back( { i, i, _mesh->Ball( i ).m } );
        m_inv_triplets.push_back( { i, i, 1.f / _mesh->Ball( i ).m } );
        _x.col( i ) = Vector3( _mesh->Ball( i ).x0.x, _mesh->Ball( i ).x0.y, _mesh->Ball( i ).x0.z );
        _x0.col( i ) = _x.col( i );
        _x_last.col( i ) = _x.col( i );
    }
    _M.resize( nb_points, nb_points );
    _M.setFromTriplets( m_triplets.cbegin(), m_triplets.cend() );

    for (int i = 0; i < nb_points; i++)
    {
        _mesh->Ball( i ).gu = Matrix3::Identity();
        _mesh->Ball( i ).R = Matrix3::Identity();
        _mesh->Ball( i ).pmodel = this;
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
                _constraints.push_back( std::make_unique<PD::MeshlessStrainConstraint<Particle, Real>>( indices, k, _x0, _mesh.get(), &_x0 ) );
            }
            else
            {
                _constraints.push_back( std::make_unique<PD::MeshlessStrainConstraint<Particle, Real>>( indices, _cfg._k_stiff, _x0, _mesh.get(), &_x0 ) );
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
            _constraints.push_back( std::make_unique<PD::EdgeConstraint<Real>>( pair.i0, pair.i1, _cfg._k_stiff, _x0 ) );
        }
    }
    if (!_cfg._attach_points_filter.empty())
    {
        mu::Parser parser;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        std::wstring expr = boost::locale::conv::utf_to_utf<wchar_t>( _cfg._attach_points_filter );
        parser.DefineVar( L"x", &x );
        parser.DefineVar( L"y", &y );
        parser.DefineVar( L"z", &z );
        parser.SetExpr( expr );
        for (int i = 0; i < nb_points; ++i)
        {
            x = _x0.coeff( 0, i );
            y = _x0.coeff( 1, i );
            z = _x0.coeff( 2, i );
            if (static_cast<bool>(parser.Eval()))
            {
                _attached_balls.insert( i );
            }
        }
    }
    for (int i : _attached_balls)
    {
        _constraints.push_back( std::make_unique<AttachConstraint<Real>>( i, _cfg._k_attach, _x0.col( i ) ) );
    }

    std::vector<Eigen::Triplet<Real, int>> triplets;
    int total_id = 0;
    for (auto& c : _constraints)
    {
        c->AddConstraint( triplets, total_id, 0.0f );
    }
    _p.setZero( 3, total_id );
    _AS.resize( total_id, nb_points );
    _AS.setFromTriplets( triplets.begin(), triplets.end() );

    triplets.clear();
    Eigen::SparseMatrix<Real> temp_StAt;
    temp_StAt.resize( total_id, nb_points );
    total_id = 0;
    for (auto& c : _constraints)
    {
        c->AddConstraint( triplets, total_id, 1.0f );
    }
    temp_StAt.setFromTriplets( triplets.begin(), triplets.end() );
    _StAt.resize( nb_points, total_id );
    _StAt = temp_StAt.transpose();

    _C = _cfg._dt * _cfg._dt * _StAt * _AS;
    _P = _M + _C;
    _llt.compute( _P );
    if (_llt.info() != Eigen::ComputationInfo::Success)
        std::cout << "ERROR: " << _llt.info() << std::endl;

    std::cout << "Metaballs: " << nb_points << std::endl;
    std::cout << "ProjectVariable size: " << total_id << std::endl;
}

void PDMetaballModelFC::DrawGUI()
{
    if (ImGui::Button( "Init" ))
    {
        Init();
    }
    ImGui::Checkbox( "simulate", &_simulate );
    ImGui::DragInt( "solve", &_cfg._nb_solve );
    ImGui::DragFloat( "stiffness", &_cfg._k_stiff, 1.0f, 0.0f, 5000.0f );
    ImGui::DragFloat( "force", &_cfg._force, 1.0f, 0.0f, 200.0f );
    ImGui::Checkbox( "show balls", &_show_balls );
    if (ImGui::Button( "one step" ))
    {
        SimulateOneStep();
    }
    //if (ImGui::Button( "AddForce" ))
    //{
    //    _array_ext_forces.push_back( { 0, Vector3::Zero() } );
    //}
    //for (int i = 0; i < _array_ext_forces.size(); i++)
    //{
    //    ImGui::InputInt( (std::string( "id" ) + std::to_string( i )).c_str(), &_array_ext_forces[i].first );
    //    ImGui::DragFloat3( (std::string( "F" ) + std::to_string( i )).c_str(), _array_ext_forces[i].second.data(), 1.0f, -200.f, 200.f );
    //}
    //if (ImGui::Button( "ClearForce" ))
    //{
    //    _array_ext_forces.clear();
    //}
}

SphereMesh<PDMetaballModelFC::Particle>& PDMetaballModelFC::GetMetaballModel()
{
    return *_mesh;
}

const SphereMesh<PDMetaballModelFC::Particle>& PDMetaballModelFC::GetMetaballModel() const
{
    return *_mesh;
}

void PDMetaballModelFC::PhysicalUpdate()
{
    //External force
    for (int i = 0; i < _mesh->BallsNum(); ++i)
        _fext.col( i ).y() -= 9.8f;
    for (auto& pair : _ext_forces)
        _fext.col( pair.first ) += pair.second;

#pragma omp parallel for
    for (int c = 0; c < _mesh->BallsNum(); c++)
    {
        _momentum.col( c ) = _x.col( c ) + _v.col( c ) * _cfg._dt;
        _momentum.col( c ) += _cfg._dt * _cfg._dt * _fext.col( c );
        _x_last.col( c ) = _x.col( c );
        _x.col( c ) = _momentum.col( c );
    }

    CollisionDetection( nullptr );

    for (int i = 0; i < _cfg._nb_solve; i++)
    {
#pragma omp parallel for
        for (int j = 0; j < _constraints.size(); j++)
        {
            _constraints[j]->Project( _x, _p );
        }

#pragma omp parallel for
        for (int r = 0; r < 3; r++)
        {
            VectorX bn = _cfg._dt * _cfg._dt * _StAt * _p.row( r ).transpose() + _M * _momentum.row( r ).transpose();
            _bn_tilde.row( r ) = (bn - _P * _x_last.row( r ).transpose()) / _cfg._dt;
            _f.row( r ) = _bn_tilde.row( r ) - (_C * _v.row( r ).transpose()).transpose();
            _ksi.row( r ).setZero();
        }

#pragma omp parallel for
        for (int c = 0; c < _contacts.size(); c++)
        {
            const Contact& contact = _contacts[c];
            if (contact.type == 0)
            {
                Vector3 dj = contact.R.transpose() * _f.col( contact.id );
                float djn = contact.NormalComponent( dj );
                Vector3 djt = contact.TangentialComponent( dj );
                Vector3 rj = Vector3::Zero();
                if (djn < 0)
                {
                    rj.y() = -djn;
                    if (djt.norm() <= -djn * 0.6f)
                    {
                        rj += -djt;
                    }
                    else
                    {
                        rj += -0.6f * (-djn) * djt.normalized();
                    }
                }
                _ksi.col( contact.id ) += contact.R * rj;
            }
        }

#pragma omp parallel for
        for (int r = 0; r < 3; r++)
        {
            _v.row( r ) = _llt.solve( _bn_tilde.row( r ).transpose() + _ksi.row( r ).transpose() );
            _x.row( r ) = _x_last.row( r ) + _cfg._dt * _v.row( r );
        }
    }

    _v *= 0.999f;

    //_aabb.max_corner = glm::vec3( -FLT_MAX );
    //_aabb.min_corner = glm::vec3( FLT_MAX );
    //for (int i = 0; i < _mesh->BallsNum(); i++)
    //{
    //    _aabb.Expand( glm::vec3( _x.col( i )(0), _x.col( i )(1), _x.col( i )(2) ) + glm::vec3( _mesh->Ball( i ).r ) );
    //    _aabb.Expand( glm::vec3( _x.col( i )(0), _x.col( i )(1), _x.col( i )(2) ) - glm::vec3( _mesh->Ball( i ).r ) );
    //}

    _fext.setZero();
}

void PDMetaballModelFC::UpdateSn()
{
    for (int i = 0; i < _mesh->BallsNum(); ++i)
        _fext.col( i ).y() -= 9.8f;
    for (auto& pair : _ext_forces)
        _fext.col( pair.first ) += pair.second;

    //#pragma omp parallel for
    for (int c = 0; c < _mesh->BallsNum(); c++)
    {
        _momentum.col( c ) = _x.col( c ) + _v.col( c ) * _cfg._dt;
        _momentum.col( c ) += _cfg._dt * _cfg._dt * _fext.col( c );
        _x_last.col( c ) = _x.col( c );
        _x.col( c ) = _momentum.col( c );
        _mesh->Ball( c ).x = ToGLM( _x.col( c ) );
        _mesh->Ball( c ).x_last = _x_last.col( c );
        _mesh->Ball( c ).v = ToGLM( _v.col( c ) );
    }

    _aabb.max_corner = glm::vec3( -FLT_MAX );
    _aabb.min_corner = glm::vec3( FLT_MAX );
    for (int i = 0; i < _mesh->BallsNum(); i++)
    {
        _aabb.Expand( glm::vec3( _x.col( i )(0), _x.col( i )(1), _x.col( i )(2) ) + glm::vec3( _mesh->Ball( i ).r ) );
        _aabb.Expand( glm::vec3( _x.col( i )(0), _x.col( i )(1), _x.col( i )(2) ) - glm::vec3( _mesh->Ball( i ).r ) );
    }
}

void PDMetaballModelFC::PDSolve()
{
    for (int i = 0; i < _cfg._nb_solve; i++)
    {
#pragma omp parallel for
        for (int j = 0; j < _constraints.size(); j++)
        {
            _constraints[j]->Project( _x, _p );
        }

#pragma omp parallel for
        for (int r = 0; r < 3; r++)
        {
            VectorX bn = _cfg._dt * _cfg._dt * _StAt * _p.row( r ).transpose() + _M * _momentum.row( r ).transpose();
            _bn_tilde.row( r ) = (bn - _P * _x_last.row( r ).transpose()) / _cfg._dt;
            _f.row( r ) = _bn_tilde.row( r ) - (_C * _v.row( r ).transpose()).transpose();
            _ksi.row( r ).setZero();
        }

        //#pragma omp parallel for
        for (int c = 0; c < _contacts.size(); c++)
        {
            const Contact& contact = _contacts[c];
            if (contact.type == 0)
            {
                Vector3 dj = contact.R.transpose() * (_f.col( contact.id ) + _ksi.col( contact.id ));
                float djn = contact.NormalComponent( dj );
                Vector3 djt = contact.TangentialComponent( dj );
                Vector3 rj = Vector3::Zero();
                if (djn < 0)
                {
                    rj.y() = -djn;
                    if (djt.norm() <= -djn * 0.5f)
                    {
                        rj += -djt;
                    }
                    else
                    {
                        rj += -0.5f * (-djn) * djt.normalized();
                    }
                }
                _ksi.col( contact.id ) += contact.R * rj;
            }
        }
        for (int c = 0; c < _contacts.size(); c++)
        {
            const Contact& contact = _contacts[c];
            if (contact.type == 1)
            {
                Vector3 fi = _f.col( contact.id ) + _ksi.col( contact.id ) + _mesh->Ball( contact.id ).m * contact.uf;
                Vector3 dj = contact.R.transpose() * fi;
                float djn = contact.NormalComponent( dj );
                Vector3 djt = contact.TangentialComponent( dj );
                Vector3 rj = Vector3::Zero();
                if (djn < 0)
                {
                    rj.y() = -djn;
                    if (djt.norm() <= -djn * 0.5f)
                    {
                        rj += -djt;
                    }
                    else
                    {
                        rj += -0.5f * (-djn) * djt.normalized();
                    }
                }
                _ksi.col( contact.id ) += contact.R * rj;
            }
        }
        //#pragma omp parallel for
        //        for (int i = 0; i < _contact_counter.size(); i++)
        //        {
        //            if (_contact_counter[i] > 1)
        //            {
        //                _ksi.col( i ) /= _contact_counter[i];
        //            }
        //        }
#pragma omp parallel for
        for (int r = 0; r < 3; r++)
        {
            _v.row( r ) = _llt.solve( _bn_tilde.row( r ).transpose() + _ksi.row( r ).transpose() );
            _x.row( r ) = _x_last.row( r ) + _cfg._dt * _v.row( r );
        }
    }

    _v *= 0.9999f;

    //_aabb.max_corner = glm::vec3( -FLT_MAX );
    //_aabb.min_corner = glm::vec3( FLT_MAX );
    //for (int i = 0; i < _mesh->BallsNum(); i++)
    //{
    //    _aabb.Expand( glm::vec3( _x.col( i )(0), _x.col( i )(1), _x.col( i )(2) ) + glm::vec3( _mesh->Ball( i ).r ) );
    //    _aabb.Expand( glm::vec3( _x.col( i )(0), _x.col( i )(1), _x.col( i )(2) ) - glm::vec3( _mesh->Ball( i ).r ) );
    //}

    _fext.setZero();
}

void PDMetaballModelFC::CollisionDetection( SpatialHash* table )
{
    std::vector<RigidStatic*> rigid_bodys = Scene::active->GetAllChildOfType<RigidStatic>();
    std::vector<PD::PDMetaballModelFC*> pd_models = Scene::active->GetAllChildOfType<PD::PDMetaballModelFC>();
    std::vector<RigidSDF*> rigid_sdfs = Scene::active->GetAllChildOfType<RigidSDF>();
    std::vector<RigidBall*> rigid_balls = Scene::active->GetAllChildOfType<RigidBall>();

    _contacts.clear();

    //for (int i = 0; i < _mesh->BallsNum(); i++)
    //{
    //    Vector3 p = _x.col( i );
    //    float r = _mesh->Ball( i ).r;

    //    Vector3 plane( 0, -1.0, 0 );
    //    Vector3 plane_n( 0.0, 1, 0 );
    //    plane_n.normalize();

    //    float test = (p - plane).dot( plane_n );
    //    if (test < r)
    //    {
    //        _contacts.emplace_back( plane_n, (Vector3( 0, 0, 1 ).cross( plane_n )).normalized(), i, 0 );
    //        _contacts.back().p = p + plane_n * (r - test);
    //    }
    //}

#pragma omp parallel for
    for (int i = 0; i < _mesh->BallsNum(); i++)
    {
        Vector3 xi0 = _x_last.col( i );
        Vector3 xi = _x.col( i );
        float r = _mesh->Ball( i ).r;

        for (auto rigid : rigid_bodys)
        {
            //            auto ret = rigid->CheckMovingBall( ToGLM( xi0 ), ToGLM( xi ), r );
            //            if (ret.has_value())
            //            {
            //                Vector3 n = ToEigen( ret->nc ).cast<Real>();
            //                Vector3 tan = GetAnyOrthoVec( n ).normalized();
            //#pragma omp critical
            //                {
            //                    _contacts.emplace_back( n, tan, i, 0 );
            //                    _contacts.back().p = ToEigen( ret->p ).cast<Real>();
            //                }
            //            }
            auto result = rigid->CheckBall( ToGLM( xi.cast<float>() ), r, 0 );
            for (auto& c : result)
            {
                Vector3 n = ToEigen( c._n ).cast<Real>();
                Vector3 tan = GetAnyOrthoVec( n ).normalized();
#pragma omp critical
                {
                    _contacts.emplace_back( n, tan, i, 0 );
                }
            }
        }
    }
    if (_show_contacts)
    {
        _contacts_vis->Clear();
        for (const auto& c : _contacts)
        {
            _contacts_vis->AddPoint( ToGLM( c.p ), glm::RED );
            _contacts_vis->AddPoint( ToGLM( c.p + c.n * 0.05f ), glm::RED );
            _contacts_vis->AddPoint( ToGLM( c.p ), glm::GREEN );
            _contacts_vis->AddPoint( ToGLM( c.p + c.t1 * 0.05f ), glm::GREEN );
            _contacts_vis->AddPoint( ToGLM( c.p ), glm::BLUE );
            _contacts_vis->AddPoint( ToGLM( c.p + c.t2 * 0.05f ), glm::BLUE );
        }
        _contacts_vis->UpdateMem();
    }

#pragma omp parallel for
    for (int i = 0; i < _mesh->BallsNum(); i++)
    {
        Vector3 p = _x.col( i );
        float r = _mesh->Ball( i ).r;

        for (auto rigid : rigid_balls)
        {
            if (glm::distance2( rigid->GetPos(), ToGLM( p.cast<float>() ) ) < (r + rigid->GetRadius()) * (r + rigid->GetRadius()))
            {
                Vector3 n = (p - ToEigen( rigid->GetPos() ).cast<Real>()).normalized();
                Vector3 t = GetAnyOrthoVec( n ).normalized();
#pragma omp critical
                {
                    _contacts.emplace_back( n, t, i, 0 );
                    _contacts.back().p = ToEigen( rigid->GetPos() ).cast<Real>() + n * rigid->GetRadius();
                }
            }
        }
    }
    //return;
    if (table != nullptr)
    {
#pragma omp parallel for
        for (int i = 0; i < _mesh->BallsNum(); i++)
        {
            Vector3 pi = _x.col( i );
            Vector3 pi0 = _x_last.col( i );
            Real ri = _mesh->Ball( i ).r;
            Particle* si = &_mesh->Ball( i );
            si->color = glm::vec3( 0.8f );
            auto spheres_to_check = table->CheckIntersection( si );

            for (auto sj : spheres_to_check)
            {
                if (si == sj)
                    continue;
                if (si->pmodel == sj->pmodel)
                    continue;
                if (glm::distance( si->x0, sj->x0 ) < (si->r * 1.1f + sj->r * 1.1f))
                    continue;
                if (std::any_of( si->neighbors.cbegin(), si->neighbors.cend(), [this, sj]( int nei ) { return &(_mesh->Ball( nei )) == sj; } ))
                    continue;

                Vector3 pj = ToEigen( sj->x ).cast<Real>();
                Vector3 pj0 = sj->x_last;
                Real rj = sj->r;
                float t = 0.f;
                if (TestMovingSphereSphere( ToGLM( pi0 ), ri * 1.1f, ToGLM( pi - pi0 ), ToGLM( pj0 ), rj * 1.1f, ToGLM( pj - pj0 ), &t ))
                {
                    Vector3 pci = pi0 + t * (pi - pi0);
                    Vector3 pcj = pj0 + t * (pj - pj0);
                    Vector3 n = (pci - pcj).normalized();
                    Vector3 tan = GetAnyOrthoVec( n ).normalized();
                    si->color = glm::vec3( 1, 0, 0 );
#pragma omp critical
                    {
                        _contacts.emplace_back( n, tan, i, 1 );
                        _contacts.back().uf = _contacts.back().R.transpose() * ToEigen( -sj->v ).cast<Real>();
                        _contacts.back().p = pci;
                    }
                }
            }
        }
    }
    else
    {
#pragma omp parallel for
        for (int i = 0; i < _mesh->BallsNum(); i++)
        {
            Vector3 pi = _x.col( i );
            Vector3 pi0 = _x_last.col( i );
            Real ri = _mesh->Ball( i ).r;
            Particle* si = &_mesh->Ball( i );
            si->color = glm::vec3( 0.8 );
            for (auto model : pd_models)
            {
                //if (model == this)
                //    continue;
                //if (!BallAABBIntersect( ToGLM( pi ), ri, model->_aabb ))
                //    continue;
                for (int j = 0; j < model->_mesh->BallsNum(); j++)
                {
                    Particle* sj = &model->_mesh->Ball( j );
                    Vector3 pj = ToEigen( sj->x ).cast<Real>();
                    Vector3 pj0 = sj->x_last;
                    Real rj = sj->r;
                    if (si == sj)
                        continue;
                    if (glm::distance( si->x0, sj->x0 ) < (si->r + sj->r))
                        continue;
                    if (std::any_of( si->neighbors.cbegin(), si->neighbors.cend(), [this, sj]( int nei ) { return &(_mesh->Ball( nei )) == sj; } ))
                        continue;

                    float t = 0.f;
                    if (TestMovingSphereSphere( ToGLM( pi0 ), ri, ToGLM( pi - pi0 ), ToGLM( pj0 ), rj, ToGLM( pj - pj0 ), &t ))
                    {
                        Vector3 pci = pi0 + t * (pi - pi0);
                        Vector3 pcj = pj0 + t * (pj - pj0);
                        Vector3 n = (pci - pcj).normalized();
                        Vector3 tan = GetAnyOrthoVec( n ).normalized();
                        si->color = glm::vec3( 1, 0, 0 );
#pragma omp critical
                        {
                            _contacts.emplace_back( n, tan, i, 1 );
                            _contacts.back().uf = _contacts.back().R.transpose() * -ToEigen( -sj->v ).cast<Real>();
                            _contacts.back().p = pci;
                        }
                    }
                    //                    auto ret = BallBallIntersect( ToGLM( pi ), ri, ToGLM( pj ), rj );
                    //                    if (ret.has_value())
                    //                    {
                    //                        Vector3 n = (pi - pj).normalized();
                    //                        Vector3 t = GetAnyOrthoVec(n).normalized();
                    //#pragma omp critical
                    //                        {
                    //                            _contacts.emplace_back( n, t, i, 1 );
                    //                            _contacts.back().uf = _contacts.back().R.transpose() * model->_v.col( j );
                    //                            _contacts.back().p = pi;
                    //                        }
                    //                    }
                }
            }
        }
    }

    //for (const auto& c : _contacts)
    //{
    //    _contact_counter[c.id]++;
    //}

}

void PDMetaballModelFC::PostPhysicalUpdate()
{
    _ext_forces.clear();
}

void PDMetaballModelFC::StartSimulation()
{
    _simulate = true;
}

void PDMetaballModelFC::StopSimulation()
{
    _simulate = false;
}

bool PDMetaballModelFC::IsSimulating() const
{
    return _simulate;
}

void PDMetaballModelFC::SimulateOneStep()
{
    UpdateSn();
    CollisionDetection( nullptr );
    PDSolve();
    PostPhysicalUpdate();
}

void PDMetaballModelFC::AddExtForce( int ball_id, Vector3 f )
{
    if (_ext_forces.contains( ball_id ))
        _ext_forces[ball_id] += f;
    else
        _ext_forces.insert( { ball_id, f } );
}

void PDMetaballModelFC::SetExtForce( int ball_id, Vector3 f )
{
    _ext_forces.insert( { ball_id, f } );
}

PDMetaballModelConfig& PDMetaballModelFC::GetConfig()
{
    return _cfg;
}

const PDMetaballModelConfig& PDMetaballModelFC::GetConfig() const
{
    return _cfg;
}

const PDMetaballModelFC::Matrix3X& PDMetaballModelFC::GetRestPos() const
{
    return _x0;
}

const PDMetaballModelFC::Matrix3X& PDMetaballModelFC::GetCurrPos() const
{
    return _x;
}

void PDMetaballModelFC::ClearAttachConstraints()
{
    _attached_balls.clear();
}

void PDMetaballModelFC::AddAttachConstraints( int ball_id )
{
    _attached_balls.insert( ball_id );
}

void PDMetaballModelFC::CreateSurfaceMapping()
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

void PDMetaballModelFC::UpdateSkinInfoBuffer()
{
    /*
    * single thread: 1000balls, 40~50 microsec.
    * multithread: 1000balls, around 20 microsec.
    */
    //using hrc = std::chrono::high_resolution_clock;
    //auto start_t = hrc::now();

    //TODO: Add orientation computation when use edge constraints.

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

void PDMetaballModelFC::ComputeAinvForEdgeConsts()
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
            avg_dist += (_x0.col( i ) - _x0.col( j )).norm();
        avg_dist /= (ball.neighbors.size());

        float wsum = 0.f;
        int cnt = 0;
        for (int j : ball.neighbors)
        {
            Vector3 xij = _x0.col( j ) - _x0.col( i );
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

void PDMetaballFCBehavior::Start()
{
    _model = GetComponent<PDMetaballModelFC>();
}

void PDMetaballFCBehavior::Update()
{
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
        for (int i = 0; i < _model->GetMetaballModel().BallsNum(); ++i)
        {
            const auto& ball = _model->GetMetaballModel().Ball( i );
            if (RayBallIntersect( r, ball.x, ball.r, &rec, 0.f, rec.t ))
            {
                intersect = true;
                id = i;
            }
        }

        if (intersect)
        {
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
            delta3d *= _model->GetConfig()._force;
            _model->SetExtForce( _hold_idx, ToEigen( delta3d ).cast<Real>() );
        }
    }
    if (Input::IsMouseButtonReleased( Input::MouseButton::Left ))
    {
        _hold_idx = -1;
    }
}

void PDMetaballFCBehavior::DrawGUI()
{
    std::stringstream ss;
    ss << "Selection: ";
    for (int id : _select_balls)
    {
        ss << id << " ";
    }
    ImGui::Text( ss.str().c_str() );
    if (ImGui::Button( "Clear Selection" ))
    {
        _select_balls.clear();
    }
    if (ImGui::Button( "AttachSelectedBalls" ))
    {
        _model->ClearAttachConstraints();
        for (int id : _select_balls)
        {
            _model->AddAttachConstraints( id );
        }
        _model->Init();
    }
}

const ShaderStorageBuffer& PDMetaballModelFC::GetVtxSkinBuffer() const
{
    return *_skin_vtx_buffer;
}

const ShaderStorageBuffer& PDMetaballModelFC::GetBallSkinBuffer() const
{
    return *_skin_ball_buffer;
}

SpatialHash::SpatialHash( float dx )
    :_dx( dx )
{
    //_table.rehash( 1000000 );
    //_table.max_load_factor( 100 );
}

void SpatialHash::Insert( Sphere* s )
{
    Eigen::Vector3f aabb_min( std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() );
    Eigen::Vector3f aabb_max = -aabb_min;
    for (int i = 0; i < 3; i++)
    {
        aabb_max[i] = std::max( aabb_max[i], s->x[i] + s->r * 1.1f );
        aabb_min[i] = std::min( aabb_min[i], s->x[i] - s->r * 1.1f );
        aabb_max[i] = std::max( aabb_max[i], (float)(s->x_last[i] + s->r * 1.1f) );
        aabb_min[i] = std::min( aabb_min[i], (float)(s->x_last[i] - s->r * 1.1f) );
    }

    int xmax = GridCoord( aabb_max.x() );
    int ymax = GridCoord( aabb_max.y() );
    int zmax = GridCoord( aabb_max.z() );
    int xmin = GridCoord( aabb_min.x() );
    int ymin = GridCoord( aabb_min.y() );
    int zmin = GridCoord( aabb_min.z() );

    for (int i = xmin; i <= xmax; i++)
    {
        for (int j = ymin; j <= ymax; j++)
        {
            for (int k = zmin; k <= zmax; k++)
            {
                _table[{ i, j, k }].push_back( s );
            }
        }
    }
}

void SpatialHash::Clear()
{
    _table.clear();
}

std::vector<SpatialHash::Sphere*> SpatialHash::CheckIntersection( Sphere* s ) const
{
    Eigen::Vector3f aabb_min( std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() );
    Eigen::Vector3f aabb_max = -aabb_min;
    for (int i = 0; i < 3; i++)
    {
        aabb_max[i] = std::max( aabb_max[i], s->x[i] + s->r * 1.1f );
        aabb_min[i] = std::min( aabb_min[i], s->x[i] - s->r * 1.1f );
        aabb_max[i] = std::max( aabb_max[i], (float)(s->x_last[i] + s->r * 1.1f) );
        aabb_min[i] = std::min( aabb_min[i], (float)(s->x_last[i] - s->r * 1.1f) );
    }
    int xmax = GridCoord( aabb_max.x() );
    int ymax = GridCoord( aabb_max.y() );
    int zmax = GridCoord( aabb_max.z() );
    int xmin = GridCoord( aabb_min.x() );
    int ymin = GridCoord( aabb_min.y() );
    int zmin = GridCoord( aabb_min.z() );

    std::vector<Sphere*> spheres;
    for (int i = xmin; i <= xmax; i++)
    {
        for (int j = ymin; j <= ymax; j++)
        {
            for (int k = zmin; k <= zmax; k++)
            {
                auto grid = _table.find( { i, j, k } );
                if (grid != _table.end())
                {
                    for (Sphere* sphere : grid->second)
                    {
                        if (sphere != s)
                        {
                            spheres.push_back( sphere );
                        }
                    }
                }
            }
        }
    }

    return spheres;
}

int SpatialHash::GridCoord( float p ) const
{
    return (int)(std::floor( p / _dx ) + 0.1f);
}

void SpatialHash::SetDx( float dx )
{
    _dx = dx;
}
}