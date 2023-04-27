#include "PBDTetraModel.h"
#include <chrono>
#include <unordered_set>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include "util/Intersection.h"
#include "input/Input.h"
#include "util/util.h"
#include "util/Camera.h"
#include "util/Shader.h"
#include "util/GlobalTimer.h"
#include "util/MathTypeConverter.h"
#include "model/HalfEdgeSurfaceTester.h"

PBD::PBDTetraModel::PBDTetraModel( const std::string& path, const std::string& surface, float density )
    :_density( density )
{
    _surface = std::make_unique<HalfEdgeMesh>( surface );
    _surface->Normalize();
    _surface->UpdateNormal();
    _surface->UpdateAttrBuffer();
    _surface->UpdatePosBuffer();

    _tet_mesh = std::make_unique<TetraMesh>();
    _tet_mesh->CreateFromSurface( path );
    //TetMeshPreprocess();
    InitParticles();
    InitConstraints();
    InitSurfaceMapping();
    est_velocities.resize( _tet_mesh->GetPointNum() );
    est_positions.resize( _tet_mesh->GetPointNum() );

    _cutting_manager = std::make_unique<FreeCuttingManager>();
    _cutting_manager->SetMesh( _surface );
    _cutting_manager->SetTetraModel( this );
    _cutting_manager->Enable();

    _scalpel = std::make_unique<Scalpel>();
    _hand = std::make_unique<Hand>();


    _stretch_vis_vao = std::make_unique<VertexArray>();
    _stretch_vis_vbo = std::make_unique<VertexBuffer>( nullptr, 0 );
    VertexBufferLayout layout;
    layout.Push( GL_FLOAT, 3, 0 );
    _stretch_vis_vbo->SetLayout( layout );
    _stretch_vis_vao->AddBuffer( *_stretch_vis_vbo );
}

void PBD::PBDTetraModel::PhysicalUpdate( float dt )
{
    //#define POINT_DAMPING
//#define TOTAL_DAMPING
//#define C_STRETCH
//#define C_VOLUME
//#define C_SHAPEMATCH
//#define C_COLLISION
#define C_FEM
    size_t particle_num = _position_list.size();

    //ESTIMATION

    for (size_t i = 0; i < particle_num; i++)
    {
        float w = 1.f / _mass_list[i];

        glm::vec3 ext_force( 0.0f );
        //ext_force += glm::vec3( 0, -1, 0 ) * 9.8f * _mass_list[i];

        est_velocities[i] = _velocity_list[i] + dt * w * ext_force;

        est_positions[i] = _position_list[i];
    }

    _external_forces_lock.lock();
    for (auto& pair : _external_forces)
    {
        est_velocities[pair.first] += (dt / _mass_list[pair.first]) * pair.second;
    }
    _external_forces.clear();
    _external_forces_lock.unlock();

    //DAMPING
#ifdef TOTAL_DAMPING
    glm::vec3 Xcm( 0.f );
    glm::vec3 Vcm( 0.f );
    float M_sum = 0.f;
    for (size_t i = 0; i < particle_num; i++)
    {
        Xcm += est_positions[i] * _mass_list[i];
        Vcm += est_velocities[i] * _mass_list[i];
        M_sum += _mass_list[i];
    }
    Xcm /= M_sum;
    Vcm /= M_sum;

    glm::vec3 L( 0.f );
    glm::mat3 I( 0.f );
    for (size_t i = 0; i < particle_num; i++)
    {
        glm::vec3 r = est_positions[i] - Xcm;
        L += glm::cross( r, est_velocities[i] * _mass_list[i] );
        glm::mat3 r_ = glm::mat3( { 0, r.z, -r.y }, { -r.z, 0, r.x }, { r.y, -r.x, 0 } );
        I += r_ * glm::transpose( r_ ) * _mass_list[i];
    }
    //angular velocity
    glm::vec3 w = glm::inverse( I ) * L;
    for (size_t i = 0; i < particle_num; i++)
    {
        glm::vec3 dv = Vcm + glm::cross( w, est_positions[i] - Xcm ) - est_velocities[i];
        est_velocities[i] += 0.1f * dv;
    }
#endif
#ifdef POINT_DAMPING
    for (size_t i = 0; i < particle_num; i++)
    {
        glm::vec3 n = est_velocities[i];
        float v = glm::length( n );
        float dv = -v * glm::min( 1.0f, 1.0f * dt / _mass_list[i] );
        est_velocities[i] += dv * n;
    }
#endif
    for (size_t i = 0; i < particle_num; i++)
    {
        est_positions[i] = _position_list[i] + dt * est_velocities[i];
    }
    for (int anchor : _anchors)
    {
        est_positions[anchor] = _rest_position[anchor];
    }
    for (unsigned anchor : _hold_vertex)
    {
        est_positions[anchor] = _position_list[anchor];
    }

    //GEN COLLISION CONSTRAINTS
    _collision_constraints.clear();
    for (size_t i = 0; i < particle_num; i++)
    {
        if (est_positions[i].y < -1)
        {
            glm::vec3 pc = glm::vec3( est_positions[i].x, -1, est_positions[i].z );
            //_collision_constraints.emplace_back( i, pc, glm::vec3( 0, 1, 0 ) );
        }
    }

    //SOLVE CONSTRAINTS
    for (int solver_it = 0; solver_it < 10; solver_it++)
    {
#ifdef C_COLLISION
        for (const auto& C : _collision_constraints)
        {
            float C_value = glm::dot( (est_positions[C._index] - C._pc), C._n );
            if (C_value >= 0)
            {
                continue;
            }
            glm::vec3 grad = C._n;
            glm::vec3 dP = -grad * C_value / glm::length2( grad );
            est_positions[C._index] += dP;
        }
#endif

#ifdef C_STRETCH
        for (int i = _stretch_constraints.size() - 1; i >= 0; i--)
        {
            StretchConstraint& C = _stretch_constraints[i];
            int i1 = C._indices[0];
            int i2 = C._indices[1];
            if (i1 == -1 || i2 == -1)
            {
                continue;
            }
            glm::vec3 p1 = est_positions[i1];
            glm::vec3 p2 = est_positions[i2];
            float w1 = 1.0f / _mass_list[i1];
            float w2 = 1.0f / _mass_list[i2];
            glm::vec3 n = glm::normalize( p1 - p2 );
            float s = (glm::distance( p1, p2 ) - C._d) / (w1 + w2);
            glm::vec3 dP1 = -w1 * s * n;
            glm::vec3 dP2 = w2 * s * n;
            est_positions[i1] += (float)(1 - glm::pow( 1 - _stretch_stiffness, solver_it + 1 )) * dP1;
            est_positions[i2] += (float)(1 - glm::pow( 1 - _stretch_stiffness, solver_it + 1 )) * dP2;
        }
#endif

#ifdef C_VOLUME
        for (const auto& C : _volume_constraints)
        {
            int i1 = C._indices[0];
            int i2 = C._indices[1];
            int i3 = C._indices[2];
            int i4 = C._indices[3];
            glm::vec3& p0 = est_positions[i1];
            glm::vec3& p1 = est_positions[i2];
            glm::vec3& p2 = est_positions[i3];
            glm::vec3& p3 = est_positions[i4];

            float C_value = std::abs( glm::dot( glm::cross( p1 - p0, p2 - p0 ), p3 - p0 ) ) / 6.0f - C._V;
            glm::vec3 grad1 = glm::cross( (p1 - p2), (p3 - p2) );
            glm::vec3 grad2 = glm::cross( (p2 - p0), (p3 - p0) );
            glm::vec3 grad3 = glm::cross( (p0 - p1), (p3 - p1) );
            glm::vec3 grad4 = glm::cross( (p1 - p0), (p2 - p0) );

            float lambda = glm::length2( grad1 ) / _mass_list[i1]
                + glm::length2( grad2 ) / _mass_list[i2]
                + glm::length2( grad3 ) / _mass_list[i3]
                + glm::length2( grad4 ) / _mass_list[i4];

            lambda = _volume_stiffness * C_value / lambda;

            glm::vec3 dP1 = -lambda * grad1 / _mass_list[i1];
            glm::vec3 dP2 = -lambda * grad2 / _mass_list[i2];
            glm::vec3 dP3 = -lambda * grad3 / _mass_list[i3];
            glm::vec3 dP4 = -lambda * grad4 / _mass_list[i4];

            est_positions[i1] += dP1;
            est_positions[i2] += dP2;
            est_positions[i3] += dP3;
            est_positions[i4] += dP4;
        }
#endif
#ifdef C_SHAPEMATCH
        {
            glm::vec3 Xcm( 0.f );
            float M = 0.f;
            size_t point_num = _tet_mesh->mPoints.size();
            for (size_t i = 0; i < point_num; i++)
            {
                Xcm += _mass_list[i] * est_positions[i];
                M += _mass_list[i];
            }
            Xcm /= M;

            glm::mat3 A( 0.f );
            for (size_t i = 0; i < point_num; i++)
            {
                glm::vec3 p = est_positions[i] - Xcm;
                glm::vec3 q = _rest_position[i] - _shape_match_constraint._Xcm;

                float w = _mass_list[i];
                p *= w;

                A[0][0] += p[0] * q[0]; A[1][0] += p[0] * q[1]; A[2][0] += p[0] * q[2];
                A[0][1] += p[1] * q[0]; A[1][1] += p[1] * q[1]; A[2][1] += p[1] * q[2];
                A[0][2] += p[2] * q[0]; A[1][2] += p[2] * q[1]; A[2][2] += p[2];
            }
            A = A * _shape_match_constraint._As;

            Eigen::Matrix3f eigen_A;
            for (int i = 0; i < 3; i++)
            {
                eigen_A( i, 0 ) = A[i][0];
                eigen_A( i, 1 ) = A[i][1];
                eigen_A( i, 2 ) = A[i][2];
            }

            Eigen::Matrix3f eigen_R;
            polarDecompositionStable( eigen_A, 0.00001, eigen_R );

            glm::mat3 R( 0.f );
            for (size_t i = 0; i < 3; i++)
            {
                R[i][0] = eigen_R( i, 0 );
                R[i][1] = eigen_R( i, 1 );
                R[i][2] = eigen_R( i, 2 );
            }
            for (size_t i = 0; i < point_num; i++)
            {
                glm::vec3 goal = Xcm + R * (_rest_position[i] - _shape_match_constraint._Xcm);
                glm::vec3 dP = (goal - est_positions[i]) * _shape_match_constraint._k;
                est_positions[i] += dP;
            }
        }
#endif
#ifdef C_FEM
        for (auto& c_fem : _fem_constraints)
        {
            float c = 0.f;

            int i0 = c_fem._indices[0];
            int i1 = c_fem._indices[1];
            int i2 = c_fem._indices[2];
            int i3 = c_fem._indices[3];

            glm::mat3 P;
            P[0] = est_positions[i1] - est_positions[i0];
            P[1] = est_positions[i2] - est_positions[i0];
            P[2] = est_positions[i3] - est_positions[i0];

            glm::mat3 F = P * c_fem._inv_rest;

            glm::mat4x3& g = c_fem._grad;
            g[1] = 2.f * F[0] * c_fem._inv_rest[0][0] +
                2.f * F[1] * c_fem._inv_rest[1][0] +
                2.f * F[2] * c_fem._inv_rest[2][0];

            g[2] = 2.f * F[0] * c_fem._inv_rest[0][1] +
                2.f * F[1] * c_fem._inv_rest[1][1] +
                2.f * F[2] * c_fem._inv_rest[2][1];


            g[3] = 2.f * F[0] * c_fem._inv_rest[0][2] +
                2.f * F[1] * c_fem._inv_rest[1][2] +
                2.f * F[2] * c_fem._inv_rest[2][2];

            c = glm::length2( F[0] ) + glm::length2( F[1] ) + glm::length2( F[2] ) - 3.f;

            //apply to element
            if (c != 0.f)
            {
                g[0] = glm::vec3( 0.f );
                g[0] = -g[1] - g[2] - g[3];
                float w = 0.f;
                for (int i = 0; i < 4; i++)
                {
                    w += glm::length2( g[i] ) / _mass_list[c_fem._indices[i]];
                }
                if (w != 0.f)
                {
                    float alpha = 0.f;
                    float dlambda = -c / (w + alpha);
                    for (int i = 0; i < 4; i++)
                    {
                        est_positions[c_fem._indices[i]] += g[i] * dlambda / _mass_list[c_fem._indices[i]];
                    }
                }
            }

            P[0] = est_positions[i1] - est_positions[i0];
            P[1] = est_positions[i2] - est_positions[i0];
            P[2] = est_positions[i3] - est_positions[i0];
            F = P * c_fem._inv_rest;
            glm::mat3 df;
            df[0] = glm::cross( F[1], F[2] );
            df[1] = glm::cross( F[2], F[0] );
            df[2] = glm::cross( F[0], F[1] );


            /*
            * function matij(a,anr, row, col) {
                return a[9*anr + 3 * col + row];
            }
            */
            g[1] += df[0] * c_fem._inv_rest[0][0];
            g[1] += df[1] * c_fem._inv_rest[1][0];
            g[1] += df[2] * c_fem._inv_rest[2][0];

            g[2] += df[0] * c_fem._inv_rest[0][1];
            g[2] += df[1] * c_fem._inv_rest[1][1];
            g[2] += df[2] * c_fem._inv_rest[2][1];

            g[3] += df[0] * c_fem._inv_rest[0][2];
            g[3] += df[1] * c_fem._inv_rest[1][2];
            g[3] += df[2] * c_fem._inv_rest[2][2];

            c = glm::determinant( F ) - 1.f;
            //apply to element
            if (c != 0.f)
            {
                g[0] = glm::vec3( 0.f );
                g[0] = -g[1] - g[2] - g[3];
                float w = 0.f;
                for (int i = 0; i < 4; i++)
                {
                    w += glm::length2( g[i] ) / _mass_list[c_fem._indices[i]];
                }
                if (w != 0.f)
                {
                    float alpha = 0.f;
                    float dlambda = -(c - alpha * c_fem._lambda) / (w + alpha);
                    c_fem._lambda += dlambda;
                    for (int i = 0; i < 4; i++)
                    {
                        est_positions[c_fem._indices[i]] += g[i] * dlambda / _mass_list[c_fem._indices[i]];
                    }
                }
            }
        }
#endif
    }

    for (int anchor : _anchors)
    {
        est_positions[anchor] = _rest_position[anchor];
    }
    for (unsigned anchor : _hold_vertex)
    {
        est_positions[anchor] = _position_list[anchor];
    }

    if (Input::IsKeyHeld( Input::Key::R ))
    {
        static float rad = 0.f;
        rad += 0.01f;
        if (rad < 3.14 * 1.5)
        {
            auto q = Eigen::Quaternionf( Eigen::AngleAxisf( 0.01f, Eigen::Vector3f( 1, 0, 0 ) ) );
            glm::mat3 m = ToGLM( q.toRotationMatrix() );
            for (int i = 0; i < particle_num; i++)
            {
                if (_rest_position[i].x > 0.3f)
                {
                    est_positions[i] = m * _position_list[i];
                }
            }
        }
        else
        {
            for (int i = 0; i < particle_num; i++)
            {
                if (_rest_position[i].x > 0.3f)
                {
                    est_positions[i] = _position_list[i];
                }
            }
        }
    }

    for (size_t i = 0; i < particle_num; i++)
    {
        _velocity_list[i] = (est_positions[i] - _position_list[i]) * (0.98f / dt);
        _position_list[i] = est_positions[i];
    }


    /////////////////////////////////////////////////////////////////////
    //Velocity update
    for (const auto& C : _collision_constraints)
    {
        int index = C._index;
    }
}

HalfEdgeMesh* PBD::PBDTetraModel::GetSurface()
{
    return _surface.get();
}

void PBD::PBDTetraModel::Update()
{
    _surface->Update();
    _tet_mesh->Update();

    IntersectionRec srec;
    int id = -1;

    glm::vec2 cursor = Input::GetMousePosition();
    cursor.y = Camera::current->_viewport_size.y - cursor.y;
    Ray ray = Camera::current->ScreenPointToRay( cursor );

    HalfEdgeSurfaceTester tester( _surface.get() );

    //PROCESS THE HAND MODEL
    //if (tester.RayIntersect( ray, &srec, 0.f, std::numeric_limits<float>::max(), &id ))
    //{
    //    if (Input::IsMouseButtonHeld( Input::MouseButton::Left ))
    //    {
    //        _scalpel->mTransform.SetPos( srec.p );

    //        if (_hold_vertex.empty())
    //        {
    //            _hand->mTransform.SetPos( srec.p + srec.normal * 0.1f );
    //        }
    //    }
    //    else
    //    {
    //        _scalpel->mTransform.SetPos( srec.p + srec.normal * 0.3f );
    //        _hand->mTransform.SetPos( srec.p + srec.normal * 0.2f );
    //    }
    //    _scalpel->mTransform.LookAt( srec.p - srec.normal );
    //    _hand->mTransform.LookAt( srec.p - srec.normal );
    //}
    //if (!_hold_vertex.empty())
    //{
    //    _hand->mTransform.SetPos( _tet_mesh->mPoints[_hold_vertex[0]] - _orinormal * 0.1f );
    //}

    if (_mode == 0)
    {
        _cutting_manager->Update();
        if (Input::IsMouseButtonHeld( Input::MouseButton::Left ))
        {
            auto [tri0, tri1] = _scalpel->GetSweepFace();
            for (auto& con : _stretch_constraints)
            {
                if (con._indices[0] == -1)
                {
                    continue;
                }
                glm::vec3 p0 = _tet_mesh->mPoints[con._indices[0]];
                glm::vec3 p1 = _tet_mesh->mPoints[con._indices[1]];
                glm::vec3 pos;
                glm::vec3 normal;
                if (LinesegTriIntersect( p0, p1, tri0.a, tri0.b, tri0.c, &pos, &normal ))
                {
                    con._indices[0] = -1;
                    con._indices[1] = -1;
                }
            }
            for (auto& con : _stretch_constraints)
            {
                if (con._indices[0] == -1)
                {
                    continue;
                }
                glm::vec3 p0 = _tet_mesh->mPoints[con._indices[0]];
                glm::vec3 p1 = _tet_mesh->mPoints[con._indices[1]];
                glm::vec3 pos;
                glm::vec3 normal;
                if (LinesegTriIntersect( p0, p1, tri1.a, tri1.b, tri1.c, &pos, &normal ))
                {
                    con._indices[0] = -1;
                    con._indices[1] = -1;
                }
            }
        }
    }

    _scalpel->Update();
    if (Input::IsKeyDown( Input::Key::P ))
    {
        _simulate = !_simulate;
    }
    if (Input::IsMouseButtonDown( Input::MouseButton::Left ))
    {
        glm::vec2 cursor = Input::GetMousePosition();
        cursor.y = Camera::current->_viewport_size.y - cursor.y;
        Ray ray = Camera::current->ScreenPointToRay( cursor );
        unsigned id = -1;
        bool intersect = false;

        IntersectionRec rec;
        rec.t = std::numeric_limits<float>::max();
        for (int i = 0, size = _tet_mesh->mBorderFaces.size(); i < size; ++i)
        {
            const glm::vec3& p0 = _position_list[_tet_mesh->mBorderFaces[i].a];
            const glm::vec3& p1 = _position_list[_tet_mesh->mBorderFaces[i].b];
            const glm::vec3& p2 = _position_list[_tet_mesh->mBorderFaces[i].c];

            if (RayTriIntersect( ray, p0, p1, p2, &rec, 0.f, rec.t ))
            {
                intersect = true;
                id = i;
            }
        }

        _hold_vertex.clear();
        if (intersect)
        {
            const glm::vec3& p0 = _position_list[_tet_mesh->mBorderFaces[id].a];
            const glm::vec3& p1 = _position_list[_tet_mesh->mBorderFaces[id].b];
            const glm::vec3& p2 = _position_list[_tet_mesh->mBorderFaces[id].c];
            //float d0 = glm::distance2( p0, rec.p );
            //float d1 = glm::distance2( p1, rec.p );
            //float d2 = glm::distance2( p2, rec.p );
            //if (d0 < d1 && d0 < d2)
            //{
            //    id = _tet_mesh->mBorderFaces[id].a;
            //}
            //else if (d1 < d0 && d1 < d2)
            //{
            //    id = _tet_mesh->mBorderFaces[id].b;
            //}
            //else
            //{
            //    id = _tet_mesh->mBorderFaces[id].c;
            //}

            _cursor_start_pos = Input::GetMousePosition();
            //_hold_vertex = std::vector<unsigned>{ id };
            _hold_vertex = std::vector<unsigned>{ _tet_mesh->mBorderFaces[id].a };
            //_oripos = std::vector<glm::vec3>{ _tet_mesh->mPoints[id] };
            _oripos = std::vector<glm::vec3>{ p0, p1, p2 };
            _orinormal = glm::normalize( glm::cross( p1 - p0, p2 - p0 ) );
        }
    }
    if (Input::IsMouseButtonReleased( Input::MouseButton::Left ))
    {
        _hold_vertex.clear();
    }
    if (_mode == 1)
    {
        if (Input::IsMouseButtonHeld( Input::MouseButton::Left ))
        {
            auto& cam_trans = Camera::current->mTransform;
            glm::vec2 delta_cursor = Input::GetMousePosition() - _cursor_start_pos;
            glm::vec3 move( -delta_cursor.x / Camera::current->_viewport_size.x, -delta_cursor.y / Camera::current->_viewport_size.y, 0.f );
            move = cam_trans.Left() * move.x + cam_trans.Up() * move.y;
            int i = 0;
            for (unsigned& v : _hold_vertex)
            {
                _position_list[v] = _oripos[i] + move * 2.f;
                i++;
            }
        }
    }


    if (Input::IsMouseButtonDown( Input::MouseButton::Middle ))
    {
        glm::vec2 cursor = Input::GetMousePosition();
        cursor.y = Camera::current->_viewport_size.y - cursor.y;
        Ray ray = Camera::current->ScreenPointToRay( cursor );
        unsigned id = -1;
        bool intersect = false;
        _intersect_solver->DispatchCompute( ray, id, intersect );
        if (intersect)
        {
            Triangle& t = _tet_mesh->mBorderFaces[id];
            glm::vec3& p0 = _tet_mesh->mPoints[t.a];
            glm::vec3& p1 = _tet_mesh->mPoints[t.b];
            glm::vec3& p2 = _tet_mesh->mPoints[t.c];
            _press_vertex = std::vector<unsigned>{ t.a, t.b, t.c };
            _press_force = glm::normalize( glm::cross( p1 - p0, p2 - p0 ) );
        }
    }
    if (Input::IsMouseButtonHeld( Input::MouseButton::Middle ))
    {
        for (unsigned i : _press_vertex)
        {
            _position_list[i] += 0.2f * _press_force;
        }
    }
    if (Input::IsMouseButtonReleased( Input::MouseButton::Middle ))
    {
        _press_vertex.clear();
    }

    int nb_iteration = 3;
    float dt = GlobalTimer::RecentAvgFrameTimeMs() / 1000.f / nb_iteration;
    dt = std::clamp( dt, 0.f, 0.1f );

    if (_simulate)
    {
        for (size_t i = 0; i < nb_iteration; i++)
        {
            PhysicalUpdate( 0.01 );
        }
        _external_forces_lock.lock();
        _external_forces.clear();
        _external_forces_lock.unlock();

        MapToSurface();
        //_surface->UpdateNormal();
        //_surface->UpdateAttrBuffer();
        _surface->UpdatePosBuffer();
    }

}

void PBD::PBDTetraModel::Draw()
{
    _tet_mesh->mPoints = _position_list;
    //_tet_mesh->UpdateBuffers();
    //_tet_mesh->Draw();
    _surface->Draw();

    if (_mode == 0)
    {
        //_scalpel->Draw();
    }
    if (_mode == 1)
    {
        //_hand->Draw();
    }
}

void PBD::PBDTetraModel::TetMeshPreprocess()
{
    std::vector<glm::vec3> extrude_dir( _tet_mesh->GetPointNum(), glm::vec3( 0.f ) );
    _tet_mesh->CalcBorderFaces();
    for (int f = 0; f < _tet_mesh->GetBorderFaceNum(); f++)
    {
        Triangle face = _tet_mesh->mBorderFaces[f];
        int ia = face.a;
        int ib = face.b;
        int ic = face.c;
        glm::vec3 va = _tet_mesh->mPoints[ia];
        glm::vec3 vb = _tet_mesh->mPoints[ib];
        glm::vec3 vc = _tet_mesh->mPoints[ic];
        glm::vec3 n = glm::cross( vb - va, vc - va );
        extrude_dir[ia] += n;
        extrude_dir[ib] += n;
        extrude_dir[ic] += n;
    }

    for (int i = 0; i < _tet_mesh->GetPointNum(); i++)
    {
        if (glm::length2( extrude_dir[i] ) != 0.f)
            _tet_mesh->mPoints[i] -= glm::normalize( extrude_dir[i] ) * 0.01f;
        _tet_mesh->mRestPos[i] = _tet_mesh->mPoints[i];
    }
    _tet_mesh->UpdateBuffers();
}

void PBD::PBDTetraModel::InitParticles()
{
    size_t point_num = _tet_mesh->mPoints.size();
    _position_list.reserve( point_num );
    _velocity_list.reserve( point_num );
    _mass_list.resize( point_num, 0.f );

    for (const glm::vec3& p : _tet_mesh->mPoints)
    {
        _position_list.push_back( p );
        _velocity_list.push_back( glm::vec3( 0.f ) );
    }
    _rest_position = _position_list;

    size_t tetra_count = _tet_mesh->mTetras.size();
    for (size_t i = 0; i < tetra_count; i++)
    {
        float volume = _tet_mesh->GetTetraVolume( i );
        float m_over_4 = volume * _density / 4.0f;
        _mass_list[_tet_mesh->mTetras[i].a] += m_over_4;
        _mass_list[_tet_mesh->mTetras[i].b] += m_over_4;
        _mass_list[_tet_mesh->mTetras[i].c] += m_over_4;
        _mass_list[_tet_mesh->mTetras[i].d] += m_over_4;
    }
}

void PBD::PBDTetraModel::InitConstraints()
{
    std::unordered_set<StretchConstraint, StretchConstraint::Hash, StretchConstraint::Pred> cst_set;

    size_t tetra_num = _tet_mesh->mTetras.size();
    for (size_t i = 0; i < tetra_num; i++)
    {
        const auto& tetra = _tet_mesh->mTetras[i];
        glm::vec3 points[4] = {
            _tet_mesh->mPoints[tetra[0]],
            _tet_mesh->mPoints[tetra[1]],
            _tet_mesh->mPoints[tetra[2]],
            _tet_mesh->mPoints[tetra[3]]
        };
        cst_set.insert( StretchConstraint( tetra[1], tetra[0], 0.1f, glm::distance( points[0], points[1] ) ) );
        cst_set.insert( StretchConstraint( tetra[2], tetra[0], 0.1f, glm::distance( points[0], points[2] ) ) );
        cst_set.insert( StretchConstraint( tetra[3], tetra[0], 0.1f, glm::distance( points[0], points[3] ) ) );
        cst_set.insert( StretchConstraint( tetra[1], tetra[2], 0.1f, glm::distance( points[1], points[2] ) ) );
        cst_set.insert( StretchConstraint( tetra[1], tetra[3], 0.1f, glm::distance( points[1], points[3] ) ) );
        cst_set.insert( StretchConstraint( tetra[2], tetra[3], 0.1f, glm::distance( points[2], points[3] ) ) );

        _volume_constraints.emplace_back( tetra[0], tetra[1], tetra[2], tetra[3], 0.1f, std::abs( _tet_mesh->GetTetraVolume( i ) ) );
    }

    _stretch_constraints.reserve( cst_set.size() );
    for (auto& sc : cst_set)
    {
        _stretch_constraints.emplace_back( sc );
    }
    std::cout << "stretch consts: " << _stretch_constraints.size() << std::endl;

    //SHAPE MATCH
    glm::vec3 Xcm( 0.f );
    float M = 0.f;
    size_t point_num = _tet_mesh->mPoints.size();
    for (size_t i = 0; i < point_num; i++)
    {
        Xcm += _mass_list[i] * _position_list[i];
        M += _mass_list[i];
    }
    Xcm /= M;

    glm::mat3 A( 0.f );
    for (size_t i = 0; i < point_num; i++)
    {
        glm::vec3 r = _position_list[i] - Xcm;
        float wi = _mass_list[i];
        float x2 = wi * r[0] * r[0];
        float y2 = wi * r[1] * r[1];
        float z2 = wi * r[2] * r[2];
        float xy = wi * r[0] * r[1];
        float xz = wi * r[0] * r[2];
        float yz = wi * r[1] * r[2];
        A[0][0] += x2; A[1][0] += xy; A[2][0] += xz;
        A[0][1] += xy; A[1][1] += y2; A[2][1] += yz;
        A[0][2] += xz; A[1][2] += yz; A[2][2] += z2;
    }
    A = glm::inverse( A );
    _shape_match_constraint = ShapeMatchConstraint( Xcm, A, 0.5f );

    //FEM
    _fem_constraints.clear();
    for (int i = 0; i < tetra_num; i++)
    {
        const auto& tetra = _tet_mesh->mTetras[i];
        int i0 = tetra.a;
        int i1 = tetra.b;
        int i2 = tetra.c;
        int i3 = tetra.d;
        glm::vec3 R0 = _tet_mesh->mPoints[i1] - _tet_mesh->mPoints[i0];
        glm::vec3 R1 = _tet_mesh->mPoints[i2] - _tet_mesh->mPoints[i0];
        glm::vec3 R2 = _tet_mesh->mPoints[i3] - _tet_mesh->mPoints[i0];
        glm::mat3 R( R0, R1, R2 );
        glm::mat3 invRest = glm::inverse( R );
        _fem_constraints.push_back( PBD::FEMConstraint( i0, i1, i2, i3, invRest ) );
    }


    for (size_t i = 0; i < point_num; i++)
    {
        if (_position_list[i].x < -0.3f)
        {
            _anchors.push_back( i );
        }
    }
}

void PBD::PBDTetraModel::InitSurfaceMapping( bool all )
{
    int outside_cnt = 0;
    int begin_num = 0;
    if (!all)
    {
        begin_num = _pos_mapping.size();
    }
    int size = _surface->_vertices.size();
    _pos_mapping.resize( size );

#pragma omp parallel for
    for (int i = begin_num; i < size; i++)
    {
        auto& p = _surface->_vertices[i];
        int tetra_i = -1;
        glm::vec4 p_bc;
        for (int i = 0; i < _tet_mesh->mTetras.size(); i++)
        {
            const auto& tet = _tet_mesh->mTetras[i];
            const glm::vec3& p0 = _tet_mesh->mRestPos[tet.a];
            const glm::vec3& p1 = _tet_mesh->mRestPos[tet.b];
            const glm::vec3& p2 = _tet_mesh->mRestPos[tet.c];
            const glm::vec3& p3 = _tet_mesh->mRestPos[tet.d];
            glm::vec4 bc = glm::TetraBarycentricPos( p0, p1, p2, p3, p.rest_pos );
            bool in_tetra = true;
            for (int j = 0; j < 4; j++)
            {
                if (bc[j] < 0 || bc[j] > 1)
                {
                    in_tetra = false;
                    break;
                }
            }
            if (in_tetra)
            {
                tetra_i = i;
                p_bc = bc;
                break;
            }
        }
        if (tetra_i == -1)
        {
            outside_cnt++;
            float min_dist = 99999.f;
            for (int i = 0; i < _tet_mesh->mTetras.size(); i++)
            {
                const auto& tet = _tet_mesh->mTetras[i];
                const glm::vec3& p0 = _tet_mesh->mRestPos[tet.a];
                const glm::vec3& p1 = _tet_mesh->mRestPos[tet.b];
                const glm::vec3& p2 = _tet_mesh->mRestPos[tet.c];
                const glm::vec3& p3 = _tet_mesh->mRestPos[tet.d];
                float d = glm::MinDistToTriangle( p.rest_pos, p0, p1, p2 );
                d = std::min( d, glm::MinDistToTriangle( p.rest_pos, p0, p2, p3 ) );
                d = std::min( d, glm::MinDistToTriangle( p.rest_pos, p0, p3, p1 ) );
                d = std::min( d, glm::MinDistToTriangle( p.rest_pos, p2, p1, p3 ) );

                if (d < min_dist)
                {
                    tetra_i = i;
                    min_dist = d;
                    glm::vec4 bc = glm::TetraBarycentricPos( p0, p1, p2, p3, p.rest_pos );
                    p_bc = bc;
                }
            }
        }
        _pos_mapping[i] = std::make_pair( tetra_i, p_bc );
    }
    std::cout << "outside: " << outside_cnt << std::endl;
}

void PBD::PBDTetraModel::SetCutMode()
{
    _mode = 0;
}

void PBD::PBDTetraModel::SetHandMode()
{
    _mode = 1;
}

void PBD::PBDTetraModel::SetStretchStiffness( float value )
{
    _stretch_stiffness = value;
}

void PBD::PBDTetraModel::SetVolumeStiffness( float value )
{
    _volume_stiffness = value;
}

const std::vector<glm::vec3>& PBD::PBDTetraModel::Points() const
{
    return _position_list;
}

const TetraMesh* PBD::PBDTetraModel::Mesh() const
{
    return _tet_mesh.get();
}

void PBD::PBDTetraModel::AddExternalForce( int i, glm::vec3 force )
{
    if (!_simulate)
        return;
    _external_forces_lock.lock();
    _external_forces.insert( std::make_pair( i, force ) );
    _external_forces_lock.unlock();
    //if (_simulate)
    //    _position_list[i] += force;
}

void PBD::PBDTetraModel::MapToSurface()
{
    int size = _pos_mapping.size();
    if (size != _surface->_vertices.size())
    {
        InitSurfaceMapping();
    }
#pragma omp parallel for
    for (int i = 0; i < size; i++)
    {
        int tet_i = _pos_mapping[i].first;
        glm::vec4 bc = _pos_mapping[i].second;
        const auto& tet = _tet_mesh->mTetras[tet_i];
        const glm::vec3& p0 = _position_list[tet.a];
        const glm::vec3& p1 = _position_list[tet.b];
        const glm::vec3& p2 = _position_list[tet.c];
        const glm::vec3& p3 = _position_list[tet.d];

        glm::vec3 pos = bc[0] * p0 + bc[1] * p1 + bc[2] * p2 + bc[3] * p3;
        _surface->_vertices[i].pos = pos;
    }
}

