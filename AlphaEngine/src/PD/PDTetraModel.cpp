#include "PDTetraModel.h"
#include <tinycolormap.hpp>
#include <fstream>
#include <omp.h>
#include <imgui/imgui.h>
#include "model/ModelLoader.h"
#include "model/HalfEdgeSurfaceTester.h"
#include "model/RigidSDF.h"
#include "util/Array3D.h"
#include "util/Camera.h"
#include "util/Intersection.h"
#include "util/util.h"
#include "input/Input.h"
#include "polar_decom/polar_decomposition_3x3.hpp"

PD::PDTetraModel::PDTetraModel( const std::string& sur_path, const std::string& coarse_path, float density, std::function<bool( glm::vec3 )> attach_filter )
    :_density( density ), _attach_filter( attach_filter )
{
    _mesh = std::make_unique<TetraMesh>();
    _mesh->CreateFromSurface( coarse_path );
    _surface = std::make_unique<CSurface>( sur_path );
    _simple_ball = std::make_unique<HalfEdgeMesh>( "res/models/ball960.obj" );
    _simple_ball->_material_main->SetDiffuseColor( 0.8f, 0.f, 0.f );
    _simple_ball->mTransform.SetScale( glm::vec3( 0.02f ) );
    _simple_cylin = std::make_unique<HalfEdgeMesh>( "res/models/cylinder.obj" );
    _simple_cylin->_material_main->SetDiffuseColor( 0.f, 0.f, 0.f );
    std::vector<glm::vec3> extrude_dir( _mesh->GetPointNum(), glm::vec3( 0.f ) );
    _mesh->CalcBorderFaces();
    for (int f = 0; f < _mesh->GetBorderFaceNum(); f++)
    {
        Triangle face = _mesh->mBorderFaces[f];
        int ia = face.a;
        int ib = face.b;
        int ic = face.c;
        glm::vec3 va = _mesh->mPoints[ia];
        glm::vec3 vb = _mesh->mPoints[ib];
        glm::vec3 vc = _mesh->mPoints[ic];
        glm::vec3 n = glm::cross( vb - va, vc - va );
        extrude_dir[ia] += n;
        extrude_dir[ib] += n;
        extrude_dir[ic] += n;
    }

    for (int i = 0; i < _mesh->GetPointNum(); i++)
    {
        if (glm::length2( extrude_dir[i] ) != 0.f)
        {
            _mesh->mPoints[i] += glm::normalize( extrude_dir[i] ) * 0.005f;
            _mesh->mRestPos[i] = _mesh->mPoints[i];

        }
    }
    _mesh->UpdateBuffers();

    Init();

    CreateSurfaceMapping();
}

void PD::PDTetraModel::Init()
{
    //Init Positions
    _current_pos.resize( 3, _mesh->GetPointNum() );
    _last_pos.resize( 3, _mesh->GetPointNum() );
    _rest_pos.resize( 3, _mesh->GetPointNum() );
    _mesh->mPoints = _mesh->mRestPos;
    for (int i = 0; i < _mesh->GetPointNum(); ++i)
    {
        _current_pos.col( i ) = Vector3( _mesh->mRestPos[i].x, _mesh->mRestPos[i].y, _mesh->mRestPos[i].z );
        _rest_pos.col( i ) = _current_pos.col( i );
    }

    //Init velocities & forces
    _current_vel.resize( 3, _mesh->GetPointNum() );
    _external_force.resize( 3, _mesh->GetPointNum() );
    _current_vel.setZero();
    _external_force.setZero();

    //Init Mass
    std::vector<float> mass_of_vertices( _mesh->GetPointNum() );
    std::fill( mass_of_vertices.begin(), mass_of_vertices.end(), 0.f );
    float m_tot = 0.f;
    for (int t = 0; t < _mesh->GetTetraNum(); ++t)
    {
        float v = _mesh->GetTetraVolume( t );
        float m = v * _density / 4.0f;
        mass_of_vertices[_mesh->mTetras[t].a] += m;
        mass_of_vertices[_mesh->mTetras[t].b] += m;
        mass_of_vertices[_mesh->mTetras[t].c] += m;
        mass_of_vertices[_mesh->mTetras[t].d] += m;
        m_tot += m * 4;
    }
    std::cout << "total mass: " << m_tot << std::endl;
    std::vector<Eigen::Triplet<Real, int>> m_triplets;
    std::vector<Eigen::Triplet<Real, int>> m_inv_triplets;
    for (int i = 0; i < _mesh->GetPointNum(); i++)
    {
        m_triplets.push_back( { i, i, mass_of_vertices[i] } );
        m_inv_triplets.push_back( { i, i, 1.f / mass_of_vertices[i] } );
    }
    _mass_matrix.resize( _mesh->GetPointNum(), _mesh->GetPointNum() );
    _mass_matrix_inv.resize( _mesh->GetPointNum(), _mesh->GetPointNum() );
    _mass_matrix.setFromTriplets( m_triplets.cbegin(), m_triplets.cend() );
    _mass_matrix_inv.setFromTriplets( m_inv_triplets.cbegin(), m_inv_triplets.cend() );

    //Init Constraints
    _constraints.clear();
    for (int t = 0; t < _mesh->GetTetraNum(); ++t)
    {
        std::vector<int> indices;
        indices.push_back( _mesh->mTetras[t].a );
        indices.push_back( _mesh->mTetras[t].b );
        indices.push_back( _mesh->mTetras[t].c );
        indices.push_back( _mesh->mTetras[t].d );
        _constraints.push_back( std::make_unique<TetraStrainConstraint<Real>>( indices, _stiffness, _current_pos ) );
    }
    for (int i = 0; i < _mesh->GetPointNum(); ++i)
    {
        if (_attach_filter( _mesh->mRestPos[i] ))
        {
            _constraints.push_back( std::make_unique<AttachConstraint<Real>>( i, _att_stiffness, _rest_pos.col( i ) ) );
        }
    }

    std::vector<Eigen::Triplet<Real, int>> triplets;
    int total_id = 0;
    for (auto& c : _constraints)
    {
        c->AddConstraint( triplets, total_id, 0.5f );
    }
    SparseMatrix A( total_id, _mesh->GetPointNum() );
    A.setFromTriplets( triplets.begin(), triplets.end() );
    _projections.setZero( 3, total_id );

    //Eigen::SparseMatrix<Real> temp_StAt;
    //triplets.clear();
    //temp_StAt.resize( total_id, _mesh->GetPointNum() );
    //total_id = 0;
    //for (auto& c : _constraints)
    //{
    //    c->AddConstraint( triplets, total_id, 1.0f );
    //}
    //temp_StAt.setFromTriplets( triplets.begin(), triplets.end() );

    _At = A.transpose();
    _N = _At * A + _mass_matrix / _timestep / _timestep;
    _llt.compute( _N );

    std::cout << "Tet Number: " << _mesh->GetTetraNum() << std::endl;
    std::cout << "Tet Vertices: " << _mesh->GetPointNum() << std::endl;
    std::cout << "ProjectVariable size: " << total_id << std::endl;
}

void PD::PDTetraModel::Update()
{
    _mesh->Update();
    if (_simulate)
    {
        _ext_forces.clear();

        if (Input::IsMouseButtonDown( Input::MouseButton::Left ))
        {
            glm::vec2 cursor = Input::GetMousePosition();
            cursor.y = Camera::current->_viewport_size.y - cursor.y;
            Ray ray = Camera::current->ScreenPointToRay( cursor );
            _hold_vertex = -1;
            bool intersect = false;
            int tid = -1;

            IntersectionRec rec;
            rec.t = std::numeric_limits<float>::max();
            for (int i = 0, size = _mesh->mBorderFaces.size(); i < size; ++i)
            {
                const glm::vec3& p0 = _mesh->mPoints[_mesh->mBorderFaces[i].a];
                const glm::vec3& p1 = _mesh->mPoints[_mesh->mBorderFaces[i].b];
                const glm::vec3& p2 = _mesh->mPoints[_mesh->mBorderFaces[i].c];

                if (RayTriIntersect( ray, p0, p1, p2, &rec, 0.f, rec.t ))
                {
                    intersect = true;
                    tid = i;
                }
            }

            if (intersect)
            {
                const glm::vec3& p0 = _mesh->mPoints[_mesh->mBorderFaces[tid].a];
                const glm::vec3& p1 = _mesh->mPoints[_mesh->mBorderFaces[tid].b];
                const glm::vec3& p2 = _mesh->mPoints[_mesh->mBorderFaces[tid].c];
                float d0 = glm::distance2( p0, rec.p );
                float d1 = glm::distance2( p1, rec.p );
                float d2 = glm::distance2( p2, rec.p );
                if (d0 < d1 && d0 < d2)
                {
                    _hold_vertex = _mesh->mBorderFaces[tid].a;
                }
                else if (d1 < d0 && d1 < d2)
                {
                    _hold_vertex = _mesh->mBorderFaces[tid].b;
                }
                else
                {
                    _hold_vertex = _mesh->mBorderFaces[tid].c;
                }

                _cursor_start_pos = Input::GetMousePosition();
                _oripos = _mesh->mPoints[_hold_vertex];
            }
        }
        if (Input::IsMouseButtonHeld( Input::MouseButton::Left ))
        {
            if (_hold_vertex != -1)
            {
                auto& cam_trans = Camera::current->mTransform;
                glm::vec2 delta = Input::GetMousePosition() - _cursor_start_pos;
                glm::vec3 delta3d = -delta.x * Camera::current->mTransform.Left() - delta.y * Camera::current->mTransform.Up();
                delta3d *= _force;
                //delta3d *= 0.1f;
                _ext_forces.insert( { _hold_vertex, Vector3( delta3d.x, delta3d.y, delta3d.z ) } );
                std::cout << "Fext" << _hold_vertex << ":" << delta3d.x << ',' << delta3d.y << ',' << delta3d.z << std::endl;
            }
        }
        if (Input::IsMouseButtonReleased( Input::MouseButton::Left ))
        {
            _hold_vertex = -1;
        }

        auto start_t = std::chrono::high_resolution_clock::now();

        for (size_t i = 0; i < 3; i++)
        {
            PhysicalUpdate();
        }

        if (Input::IsKeyDown( Input::Key::F ))
        {
            std::cout << "Phys=" << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_t).count() / 1000.0 << std::endl;
        }
        for (int i = 0; i < _mesh->GetPointNum(); i++)
        {
            _mesh->mPoints[i] = glm::vec3( _current_pos( 0, i ), _current_pos( 1, i ), _current_pos( 2, i ) );
        }
        _mesh->UpdateBuffers();

        start_t = std::chrono::high_resolution_clock::now();
        MapToSurface();
        if (Input::IsKeyDown( Input::Key::F ))
        {
            std::cout << "map=" << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_t).count() / 1000.0 << std::endl;
        }
        if (Input::IsKeyDown( Input::Key::E ))
        {
            if (_hold_vertex != -1)
            {
                auto pair = *_ext_forces.begin();
                _array_ext_forces.push_back( pair );
            }
        }

    }


    if (Input::IsKeyDown( Input::Key::O ))
        _show_tet = !_show_tet;
    if (Input::IsKeyDown( Input::Key::U ))
        _show_surface = !_show_surface;
    if (Input::IsKeyDown( Input::Key::P ))
        _simulate = !_simulate;
}

void PD::PDTetraModel::Draw()
{
    if (_show_tet)
    {
        _mesh->Draw();
    }

    if (_show_surface)
    {
        auto start_t = std::chrono::high_resolution_clock::now();

        _surface->Draw();
        if (Input::IsKeyDown( Input::Key::F ))
        {
            std::cout << "rendering=" << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_t).count() / 1000.0 << std::endl;
        }
    }

    for (auto& pair : _array_ext_forces)
    {
        glm::vec3 start_pos = ToGLM( _current_pos.col( pair.first ).eval() );
        glm::vec3 end_pos = ToGLM( (_current_pos.col( pair.first ) + pair.second.normalized() * 0.2f).eval() );
        //start_pos = start_pos + 0.05f * (end_pos - start_pos);
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

void PD::PDTetraModel::DrawGUI()
{
    ImGui::Begin( "tetra pd" );
    if (ImGui::Button( "Init" ))
    {
        Init();
    }
    ImGui::Checkbox( "simulate", &_simulate );
    ImGui::DragFloat( "stiffness", &_stiffness, 1.f, 0.f, 2000.f );
    ImGui::DragFloat( "force", &_force, 0.1f, 0.0f, 100.0f );
    if (ImGui::Button( "AddForce" ))
    {
        _array_ext_forces.push_back( { 0, Vector3( 0.f, 0.f, 0.f ) } );
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
    if (_hold_vertex != -1 && !_ext_forces.empty())
    {
        auto f = _ext_forces.begin()->second;
        ImGui::Text( "Dragging Vertex %d: %d, %d, %d", _hold_vertex, f[0], f[1], f[2] );
    }
    ImGui::End();
}

void PD::PDTetraModel::PhysicalUpdate()
{//External force
    _external_force.setZero();
    for (int i = 0; i < _mesh->GetPointNum(); ++i)
    {
        //_external_force.col( i ).y() -= 9.8f;
    }
    _external_force = (_mass_matrix * _external_force.transpose()).transpose();
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
        if (rad <= 3.1415f * 1.5)
        {
            auto q = Eigen::Quaternionf( Eigen::AngleAxisf( 0.01f, Eigen::Vector3f( 1, 0, 0 ) ) );
            Eigen::Matrix3f m = q.toRotationMatrix();
            for (int i = 0; i < _constraints.size(); i++)
            {
                if (typeid(*_constraints[i]) == typeid(AttachConstraint<Real>))
                {
                    AttachConstraint<Real>* c = dynamic_cast<AttachConstraint<Real>*>(_constraints[i].get());
                    int id = c->_indices[0];
                    if (_rest_pos.col( id ).x() > 0.3f)
                    {
                        auto p0 = c->_fixed_pos;
                        p0 = m * p0;
                        //p0 += Vector3( 0, -0.01f, 0 );
                        c->_fixed_pos = p0;
                    }
                }
            }
            rad += 0.01f;
        }
    }

    _last_pos = _current_pos;
    _inertia_y = _current_pos + _current_vel * _timestep;
    auto qn = _current_pos;
    Matrix3X sn = _inertia_y + (_timestep * _timestep) * (_mass_matrix_inv * _external_force.transpose()).transpose();

    Matrix3X qn1 = sn;

    sn = (_mass_matrix / (_timestep * _timestep) * sn.transpose()).transpose();

    //iteration
    for (int i = 0; i < 5; i++)
    {
#pragma omp parallel shared(qn1)
        {
#pragma omp for 
            for (int j = 0; j < _constraints.size(); j++)
            {
                _constraints[j]->Project( qn1, _projections );
            }
#pragma omp for
            for (int r = 0; r < 3; r++)
            {
                qn1.row( r ) = _llt.solve( _At * _projections.row( r ).transpose() + sn.row( r ).transpose() ).transpose();
            }
        }
    }
    _current_pos = qn1;
    _current_vel = (_current_pos - qn) * (0.3 / _timestep);
    CollisionDetection( _current_pos );
    //Matrix3X pene = CollisionDetection( _current_pos );
    //_current_pos -= pene;
}

PD::PDTetraModel::Matrix3X PD::PDTetraModel::CollisionDetection( const Matrix3X& x )
{
    std::vector<RigidSDF*> rigid_sdfs = Scene::active->GetAllChildOfType<RigidSDF>();
    for (RigidSDF* rigid_sdf : rigid_sdfs)
    {
        for (int i = 0; i < _mesh->GetPointNum(); i++)
        {
            glm::vec3 normal;
            float depth;
            glm::vec3 pos( _current_pos.col( i )(0), _current_pos.col( i )(1), _current_pos.col( i )(2) );
            if (rigid_sdf->CheckPoint( pos, &normal, &depth ))
            {
                Vector3 n( normal.x, normal.y, normal.z );
                //Vector3 dx = _current_pos.col( i ) - _last_pos.col( i );
                //Vector3 dx_n = n * dx.dot( n );
                //Vector3 dx_t = dx - dx_n;
                //float s = std::min( 1000 * depth, 1.0f );
                //Vector3 fric = -s * dx_t;

                Vector3 v = _current_vel.col( i );
                Vector3 vn = n * v.dot( n );
                Vector3 vt = v - vn;

                _current_pos.col( i ) += n * depth;
                _current_vel.col( i ) = -1.0 * vn + vt;
            }
        }
    }

    //for (int i = 0; i < _mesh->GetPointNum(); i++)
    //{
    //    Vector3 p = _current_pos.col( i );
    //    if (p.y() < -1.f)
    //    {
    //        Vector3 dx = _current_pos.col( i ) - _last_pos.col( i );
    //        Vector3 n( 0, 1, 0 );
    //        Vector3 v = _current_vel.col( i );
    //        Vector3 dxn = n * dx.dot( n );
    //        Vector3 dxt = dx - dxn;
    //        Vector3 vn = n * v.dot( n );
    //        Vector3 vt = v - vn;

    //        _current_vel.col( i ) = -vn + 0.1 * vt;
    //        _current_pos.col( i ).y() = -1.f;
    //        _current_pos.col( i ) -= 0.5f * dxt;
    //    }
    //}

    return Matrix3X();
}

void PD::PDTetraModel::CreateSurfaceMapping()
{
    _skininfos.resize( _surface->size_of_vertices() );

    int i = 0;
    for (auto hv = _surface->vertices_begin(); hv != _surface->vertices_end(); hv++, i++)
    {
        glm::vec3 p( hv->point().x(), hv->point().y(), hv->point().z() );
        for (int t = 0; t < _mesh->GetTetraNum(); t++)
        {
            glm::vec4 bc = glm::TetraBarycentricPos( _mesh->mPoints[_mesh->mTetras[t].a],
                _mesh->mPoints[_mesh->mTetras[t].b],
                _mesh->mPoints[_mesh->mTetras[t].c],
                _mesh->mPoints[_mesh->mTetras[t].d],
                p );
            if (bc[0] >= 0.f && bc[1] >= 0.f && bc[2] >= 0.f && bc[3] >= 0.f)
            {
                _skininfos[i].tid = t;
                _skininfos[i].bc = bc;
                break;
            }
        }
        if (_skininfos[i].tid == -1)
        {
            float min_dist = std::numeric_limits<float>::max();
            for (int t = 0; t < _mesh->GetTetraNum(); t++)
            {

                float d0 = glm::distance2( _mesh->mPoints[_mesh->mTetras[t].a], p );
                float d1 = glm::distance2( _mesh->mPoints[_mesh->mTetras[t].b], p );
                float d2 = glm::distance2( _mesh->mPoints[_mesh->mTetras[t].c], p );
                float d3 = glm::distance2( _mesh->mPoints[_mesh->mTetras[t].d], p );
                if (d0 < min_dist)
                {
                    _skininfos[i].tid = t;
                    min_dist = d0;
                }
                if (d1 < min_dist)
                {
                    _skininfos[i].tid = t;
                    min_dist = d1;
                }
                if (d2 < min_dist)
                {
                    _skininfos[i].tid = t;
                    min_dist = d2;
                }
                if (d3 < min_dist)
                {
                    _skininfos[i].tid = t;
                    min_dist = d3;
                }

            }
            glm::vec4 bc = glm::TetraBarycentricPos( _mesh->mPoints[_mesh->mTetras[_skininfos[i].tid].a],
                _mesh->mPoints[_mesh->mTetras[_skininfos[i].tid].b],
                _mesh->mPoints[_mesh->mTetras[_skininfos[i].tid].c],
                _mesh->mPoints[_mesh->mTetras[_skininfos[i].tid].d],
                p );
            _skininfos[i].bc = bc;
        }
    }
}

void PD::PDTetraModel::MapToSurface()
{
    int i = 0;
    for (auto hv = _surface->vertices_begin(); hv != _surface->vertices_end(); hv++, i++)
    {
        int tid = _skininfos[i].tid;
        glm::vec4 bc = _skininfos[i].bc;

        glm::vec3 pa = _mesh->mPoints[_mesh->mTetras[tid].a];
        glm::vec3 pb = _mesh->mPoints[_mesh->mTetras[tid].b];
        glm::vec3 pc = _mesh->mPoints[_mesh->mTetras[tid].c];
        glm::vec3 pd = _mesh->mPoints[_mesh->mTetras[tid].d];

        glm::vec3 result = pa * bc[0] + pb * bc[1] + pc * bc[2] + pd * bc[3];
        hv->point() = CSurface::Point( result.x, result.y, result.z );
    }

    //_surface->UpdateNormal( CSurface::NormalType::VertexNormal );
    _surface->UpdateBuffer( CSurface::BufferType::Position );
    //_surface->UpdateBuffer( CSurface::BufferType::Normal );
    //_surface->UpdateBuffer( CSurface::BufferType::Tangent );
}

