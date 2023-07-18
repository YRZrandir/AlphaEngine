#include "PDGPUMetaballModel.h"
#include <iomanip>
#include <imgui/imgui.h>
#include <tinycolormap.hpp>
#include <omp.h>
#include <cstdlib>
#include "cuda_gl_interop.h"
#include "CVT/WeightedCVT.h"
#include "input/Input.h"
#include "model/ModelLoader.h"
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

PD::PDGPUMetaballModel::PDGPUMetaballModel( const PDMetaballModelConfig& cfg, PDMetaballHalfEdgeMesh* mesh )
    :_cfg( cfg ), _surface( mesh )
{
    Instrumentor::Get().BeginSession( "PDGPUMetaballModel", "PDGPUMetaballModel.json" );
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
    for (int i = 0; i < _mesh->BallsNum(); i++)
    {
        _mesh->Ball( i ).x += _cfg._displacement;
        _mesh->Ball( i ).x0 = _mesh->Ball( i ).x;
    }

    UpdateSkinInfoBuffer();


    float m_max = 0.f;
    float m_total = 0.f;
    for (int i = 0; i < _mesh->BallsNum(); ++i)
    {
        auto& ball = _mesh->Ball( i );
        float r = ball.r;
        ball.m = 4.f / 3.f * 3.14f * r * r * r * cfg._density;
        m_max = std::max( m_max, ball.m );
        m_total += ball.m;
    }
    for (int i = 0; i < _mesh->BallsNum(); ++i)
    {
        _mesh->Ball( i ).m_rel = _mesh->Ball( i ).m / m_max;
    }

    std::cout << "total v = " << _surface->TotalVolume() << std::endl;
    std::cout << "total m = " << m_total << std::endl;
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

PD::PDGPUMetaballModel::~PDGPUMetaballModel()
{
    Instrumentor::Get().EndSession();
}

void PD::PDGPUMetaballModel::Start()
{

}

void PD::PDGPUMetaballModel::Init()
{
    int nb_points = _mesh->BallsNum();
    std::vector<Eigen::Triplet<Real, int>> m_triplets;
    std::vector<Eigen::Triplet<Real, int>> m_inv_triplets;
    _x.resize( 3, nb_points );
    _rest_pos.resize( 3, nb_points );
    _v.resize( 3, nb_points );
    _pene.resize( 3, nb_points );
    _f_ext.resize( 3, nb_points );
    _last_pos.resize( 3, nb_points );
    _last_pos1.resize( 3, nb_points );
    _momentum.resize( 3, nb_points );
    _v.setZero();
    _f_ext.setZero();
    std::vector<Real> host_m_vector;

    for (int i = 0; i < nb_points; ++i)
    {
        m_triplets.push_back( { i, i, _mesh->Ball( i ).m } );
        m_inv_triplets.push_back( { i, i, (Real)1.0 / _mesh->Ball( i ).m } );
        host_m_vector.push_back( _mesh->Ball( i ).m );
        _x.col( i ) = Vector3( _mesh->Ball( i ).x0.x, _mesh->Ball( i ).x0.y, _mesh->Ball( i ).x0.z );
        _rest_pos.col( i ) = _x.col( i );
        _last_pos.col( i ) = _x.col( i );
        _last_pos1.col( i ) = _x.col( i );
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
                _constraints.push_back( std::make_unique<PD::MeshlessStrainConstraint<Particle, Real>>( indices, k, _x, _mesh.get(), &_rest_pos ) );
            }
            else
            {
                _constraints.push_back( std::make_unique<PD::MeshlessStrainConstraint<Particle, Real>>( indices, _cfg._k_stiff, _x, _mesh.get(), &_rest_pos ) );
            }
        }
    }
    else
    {
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
            _constraints.push_back( std::make_unique<PD::EdgeConstraint<Real>>( pair.i0, pair.i1, _cfg._k_stiff, _x ) );
        }
        ComputeAinvForEdgeConsts();
    }
    if (_cfg._attach_filter != nullptr)
    {
        for (int i = 0; i < nb_points; ++i)
        {
            if (_cfg._attach_filter( _mesh->Ball( i ).x0 ))
            {
                _constraints.push_back( std::make_unique<AttachConstraint<Real>>( i, _cfg._k_attach, _rest_pos.col( i ) ) );
            }
        }
    }
    for (int i : _select_balls)
    {
        _constraints.push_back( std::make_unique<AttachConstraint<Real>>( i, _cfg._k_attach, _rest_pos.col( i ) ) );
    }
    _attached_balls = std::vector<int>( _select_balls.begin(), _select_balls.end() );
    _select_balls.clear();

    cudaMalloc( (void**)&_cudapd.q0, sizeof( vec3 ) * nb_points );
    cudaMemcpy( _cudapd.q0, _rest_pos.data(), sizeof( vec3 ) * nb_points, cudaMemcpyHostToDevice );

    cudaMalloc( (void**)&_cudapd.q, sizeof( vec3 ) * nb_points );
    cudaMemcpy( _cudapd.q, _x.data(), sizeof( vec3 ) * nb_points, cudaMemcpyHostToDevice );

    cudaMalloc( (void**)&_cudapd.v, sizeof( vec3 ) * nb_points );
    cudaMemcpy( _cudapd.v, _v.data(), sizeof( vec3 ) * nb_points, cudaMemcpyHostToDevice );

    cudaMalloc( (void**)&_cudapd.qlast, sizeof( vec3 ) * nb_points );
    cudaMemcpy( _cudapd.qlast, _x.data(), sizeof( vec3 ) * nb_points, cudaMemcpyHostToDevice );

    cudaMalloc( (void**)&_cudapd.qlast1, sizeof( vec3 ) * nb_points );
    cudaMemcpy( _cudapd.qlast1, _x.data(), sizeof( vec3 ) * nb_points, cudaMemcpyHostToDevice );

    cudaMalloc( (void**)&_cudapd.qlast2, sizeof( vec3 ) * nb_points );
    cudaMemcpy( _cudapd.qlast2, _x.data(), sizeof( vec3 ) * nb_points, cudaMemcpyHostToDevice );

    cudaMalloc( (void**)&_cudapd.f_ext, sizeof( vec3 ) * nb_points );
    cudaMemcpy( _cudapd.f_ext, _f_ext.data(), sizeof( vec3 ) * nb_points, cudaMemcpyHostToDevice );

    cudaMalloc( (void**)&_cudapd.momentum, sizeof( vec3 ) * nb_points );

    cudaMalloc( (void**)&_cudapd.m, sizeof( Real ) * nb_points );
    cudaMemcpy( _cudapd.m, host_m_vector.data(), sizeof( Real ) * nb_points, cudaMemcpyHostToDevice );

    VectorX host_qx = _x.row( 0 ).transpose();
    VectorX host_qy = _x.row( 1 ).transpose();
    VectorX host_qz = _x.row( 2 ).transpose();
    _d_q[0].UpdateBuffer( host_qx.rows(), host_qx.data() );
    _d_q[1].UpdateBuffer( host_qy.rows(), host_qy.data() );
    _d_q[2].UpdateBuffer( host_qz.rows(), host_qz.data() );

    _cudapd.qx = _d_q[0].Data();
    _cudapd.qy = _d_q[1].Data();
    _cudapd.qz = _d_q[2].Data();

    _cudapd.dt = _cfg._dt;
    _cudapd.nb_points = nb_points;

    std::vector<Eigen::Triplet<Real, int>> triplets;
    int total_id = 0;
    for (auto& c : _constraints)
    {
        c->AddConstraint( triplets, total_id, 0.0f );
    }
    _p.setZero( 3, total_id );

    _AS.resize( total_id, nb_points );
    _AS.setFromTriplets( triplets.cbegin(), triplets.cend() );

    triplets.clear();
    Eigen::SparseMatrix<Real> temp_StAt;
    temp_StAt.resize( total_id, nb_points );
    total_id = 0;
    for (const auto& c : _constraints)
    {
        c->AddConstraint( triplets, total_id, 1.0f );
    }
    temp_StAt.setFromTriplets( triplets.cbegin(), triplets.cend() );
    _StAt = temp_StAt.transpose();


    _CStAt.clear();
    _CStAtAS.clear();
    _CAS.clear();
    _StAtAS.resize( nb_points, nb_points );
    _StAtAS.setZero();
    for (const auto& c : _constraints)
    {
        auto A = c->GetA();
        auto S = c->GetS( nb_points );
        _StAtAS += c->_weight * S.transpose() * A.transpose() * A * S;
        _CAS.push_back( A * S );
        _CStAt.push_back( c->_weight * S.transpose() * A.transpose() );
        _CStAtAS.push_back( _CStAt.back() * _CAS.back() );
    }

    _N = _StAtAS + _mass_matrix / _cfg._dt / _cfg._dt;
    _llt.compute( _N );
    if (_llt.info() != Eigen::ComputationInfo::Success)
        std::cout << "ERROR: " << _llt.info() << std::endl;

    _g.resize( 3, nb_points );
    _J = _N;
    _newtonllt.compute( _J );

    cudaMalloc( (void**)&_cudapd.p, sizeof( vec3 ) * total_id );

    _StAt.makeCompressed();
    _d_At = CreateCUDASparseMatrix( _StAt );

    _LU = SparseMatrix( _N.cols(), _N.cols() );
    _Dinv = SparseMatrix( _N.cols(), _N.cols() );
    std::vector<Eigen::Triplet<Real, int>> LU_triplets;
    std::vector<Eigen::Triplet<Real, int>> Dinv_triplets;
    for (int k = 0; k < _N.outerSize(); ++k)
        for (SparseMatrix::InnerIterator it( _N, k ); it; ++it)
        {
            if (it.row() == it.col())
                Dinv_triplets.push_back( Eigen::Triplet<Real, int>( it.row(), it.col(), 1.f / it.value() ) );
            else
                LU_triplets.push_back( Eigen::Triplet<Real, int>( it.row(), it.col(), -it.value() ) );
        }
    _LU.setFromTriplets( LU_triplets.begin(), LU_triplets.end() );
    _Dinv.setFromTriplets( Dinv_triplets.begin(), Dinv_triplets.end() );

    _Dinv.makeCompressed();
    _Jacobi_Dinv = CreateCUDASparseMatrix( _Dinv );

    _B = _Dinv * _LU;
    _B.makeCompressed();
    _Jacobi_B = CreateCUDASparseMatrix( _B );

    for (int i = 0; i < 3; i++)
    {
        _d_proj_buf[i].UpdateBuffer( total_id, _p.data() );

        _d_rhvec_buf[i].UpdateBuffer( nb_points, _p.data() );
        _Jacobi_x_buf[i].UpdateBuffer( nb_points, _p.data() );
        _Jacobi_y_buf[i].UpdateBuffer( nb_points, _p.data() );
        _Jacobi_b_buf[i].UpdateBuffer( nb_points, _p.data() );
        _Jacobi_Dinvb_buf[i].UpdateBuffer( nb_points, _p.data() );
        _d_proj[i] = CreateCUDAVector( _d_proj_buf[i] );
        _d_rhvec[i] = CreateCUDAVector( _d_rhvec_buf[i] );
        _Jacobi_x[i] = CreateCUDAVector( _d_q[i] );
        _Jacobi_y[i] = CreateCUDAVector( _Jacobi_y_buf[i] );
        _Jacobi_Dinvb[i] = CreateCUDAVector( _Jacobi_Dinvb_buf[i] );
        _Jacobi_b[i] = CreateCUDAVector( _Jacobi_b_buf[i] );
    }
    _cudapd.projx = _d_proj_buf[0].Data();
    _cudapd.projy = _d_proj_buf[1].Data();
    _cudapd.projz = _d_proj_buf[2].Data();

    size_t bufsize = CUDASpmvBufferSize<Real>( _Jacobi_B, _Jacobi_x[0], _Jacobi_y[0] );
    _Spmv_buf[0].UpdateBuffer( bufsize, nullptr );
    _Spmv_buf[1].UpdateBuffer( bufsize, nullptr );
    _Spmv_buf[2].UpdateBuffer( bufsize, nullptr );

    for (int i = 0; i < _constraints.size(); i++)
    {
        if (typeid(*_constraints[i]) == typeid(PD::AttachConstraint<Real>))
        {
            PD::AttachConstraint<Real>* C = dynamic_cast<PD::AttachConstraint<Real>*>(_constraints[i].get());
            CudaAttachConst cudac;
            cudac.id = C->_indices[0];
            cudac.loc = C->_loc;
            cudac.p = make_vec3( (Real)C->_fixed_pos.x(), (Real)C->_fixed_pos.y(), (Real)C->_fixed_pos.z() );
            _host_attach_consts.push_back( cudac );
        }
        else if (typeid(*_constraints[i]) == typeid(PD::MeshlessStrainConstraint<Particle, Real>))
        {
            auto* C = dynamic_cast<PD::MeshlessStrainConstraint<Particle, Real>*>(_constraints[i].get());
            CudaMetaballConst c;
            c.id = C->_indices[0];
            c.loc = C->_loc;
            c.nei_base = (int)_host_metaball_neiinfos.size();
            c.nei_count = (int)C->_indices.size();
            c.weight = C->_weight;
            c.invA[0] = { C->_invA( 0, 0 ), C->_invA( 1, 0 ), C->_invA( 2, 0 ) };
            c.invA[1] = { C->_invA( 0, 1 ), C->_invA( 1, 1 ), C->_invA( 2, 1 ) };
            c.invA[2] = { C->_invA( 0, 2 ), C->_invA( 1, 2 ), C->_invA( 2, 2 ) };
            c.F[0] = { 0.f, 0.f, 0.f };
            c.F[1] = { 0.f, 0.f, 0.f };
            c.F[2] = { 0.f, 0.f, 0.f };
            _host_metaball_consts.push_back( c );
            for (int j = 0; j < C->_indices.size(); j++)
            {
                CudaMetaballConstNeiInfo info;
                info.id = C->_indices[j];
                info.w = C->_w[j];
                _host_metaball_neiinfos.push_back( info );
            }
        }
        else if (typeid(*_constraints[i]) == typeid(PD::EdgeConstraint<Real>))
        {
            auto* C = dynamic_cast<PD::EdgeConstraint<Real>*>(_constraints[i].get());
            CudaEdgeConst c;
            c.id0 = C->_indices[0];
            c.id1 = C->_indices[1];
            c.loc = C->_loc;
            c.restlen = C->_rest;
            c.weight = C->_weight;
            _host_edge_consts.push_back( c );
        }
    }

    _cuda_attach_consts.UpdateBuffer( _host_attach_consts.size(), _host_attach_consts.data() );
    _cudapd.attach_consts = _cuda_attach_consts.Data();
    _cudapd.nb_attach = _host_attach_consts.size();
    _cudapd.k_attach = _cfg._k_attach;

    _cuda_metaball_consts.UpdateBuffer( _host_metaball_consts.size(), _host_metaball_consts.data() );
    _cudapd.metaball_consts = _cuda_metaball_consts.Data();

    _cuda_metaball_neiinfos.UpdateBuffer( _host_metaball_neiinfos.size(), _host_metaball_neiinfos.data() );
    _cudapd.metaball_neiinfos = _cuda_metaball_neiinfos.Data();

    _cuda_edge_consts.UpdateBuffer( _host_edge_consts.size(), _host_edge_consts.data() );
    _cudapd.edge_consts = _cuda_edge_consts.Data();
    _cudapd.nb_edges = _host_edge_consts.size();

    _cuda_skinning_info.UpdateBuffer( _ball_skinning_infos.size(), (const CudaSkinningInfo*)(void*)_ball_skinning_infos.data() );
    _cudapd._skin_info = _cuda_skinning_info.Data();

    cudaGraphicsResource_t cuda_res[1];
    cudaGraphicsGLRegisterBuffer( &cuda_res[0], _skin_ball_buffer->GetID(), ::cudaGraphicsRegisterFlags::cudaGraphicsRegisterFlagsNone );
    cudaGraphicsMapResources( 1, cuda_res, 0 );
    size_t cuda_size;
    cudaGraphicsResourceGetMappedPointer( (void**)&_cudapd._skin_info, &cuda_size, cuda_res[0] );

    _cudapd.displacement = make_vec3( _cfg._displacement.x, _cfg._displacement.y, _cfg._displacement.z );
    std::cout << "ProjectVariable size: " << total_id << std::endl;
}

void PD::PDGPUMetaballModel::Update()
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
            _ext_forces.insert( { _hold_idx, ToEigen( delta3d ).cast<Real>() } );
        }
    }
    if (Input::IsMouseButtonReleased( Input::MouseButton::Left ))
    {
        _hold_idx = -1;
    }
    if (Input::IsKeyDown( Input::Key::E ))
    {
        if (!_ext_forces.empty())
        {
            auto pair = *_ext_forces.begin();
            _array_ext_forces.push_back( { pair.first, pair.second.cast<float>() } );
        }
    }
    if (Input::IsKeyDown( Input::Key::O ))
    {
        _show_balls = !_show_balls;
    }
    if (Input::IsKeyDown( Input::Key::I ))
    {
        _surface->mRenderConfig._draw_face = !_surface->mRenderConfig._draw_face;
    }

    for (int i = 0; i < _mesh->BallsNum(); ++i)
    {
        Vector3 p = _x.col( i );
        _mesh->Ball( i ).x = glm::vec3( p[0], p[1], p[2] );
    }
    if (_cfg._method == 2 || _cfg._const_type == 1)
    {

    }
}

void PD::PDGPUMetaballModel::Draw()
{
    if (_show_balls)
    {
        for (size_t i = 0; i < _mesh->BallsNum(); i++)
        {
            _mesh->Ball( i ).x = ToGLM( _x.col( i ) );
        }
        _mesh->Draw();
        //_line_segments->Draw();
    }

    if (_show_surface)
    {
        if (_surface->_gpu_skinning)
        {
            InstrumentationTimer timer( "Render" );
            if (_cfg._const_type == 1)
            {
                ComputeBallOrit2();
                UpdateSkinInfoBuffer();
            }
            _skin_vtx_buffer->BindBufferBase( 0 );
            _skin_ball_buffer->BindBufferBase( 1 );
            _surface->Draw();
        }
        else
        {
            cudaMemcpy( _ball_skinning_infos.data(), _cuda_skinning_info.Data(), _cuda_skinning_info.MemSize(), cudaMemcpyDeviceToHost );
            cudaDeviceSynchronize();
            for (int i = 0; i < _mesh->BallsNum(); i++)
            {
                _mesh->Ball( i ).x = ToGLM( _x.col( i ).eval() );
                _mesh->Ball( i ).R = ToEigen( glm::mat3( _ball_skinning_infos[i].R ) ).transpose().cast<Real>();
            }
            InstrumentationTimer timer( "Render" );
            MapSurface();
            _surface->Draw();
        }
        //_surface->_material_main->SetDiffuseColor( _color.x, _color.y, _color.z );
    }

    for (auto& pair : _array_ext_forces)
    {
        glm::vec3 start_pos = ToGLM( _x.col( pair.first ) );
        glm::vec3 end_pos = ToGLM( (_x.col( pair.first ).cast<float>() + pair.second.normalized() * 0.2f).eval() );
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

void PD::PDGPUMetaballModel::DrawGUI()
{
    ImGui::Begin( "metaball pd" );
    if (ImGui::Button( "Init" ))
    {
        Init();
    }
    ImGui::Checkbox( "simulate", &_simulate );
    ImGui::DragInt( "solve", &_cfg._nb_solve );
    ImGui::DragFloat( "rho", &_rho, 0.0001f, 0.0f, 1.0f, "%.4f" );
    ImGui::DragFloat( "gravity", &_cudapd.gravity, 0.1f, 0.0f, 9.8f );
    ImGui::DragFloat( "stiffness", &_cfg._k_stiff, 1.0f, 0.0f, 5000.0f );
    ImGui::Checkbox( "show balls", &_show_balls );
    ImGui::Checkbox( "show surface", &_surface->mRenderConfig._draw_face );
    ImGui::Checkbox( "show wireframe", &_surface->mRenderConfig._draw_line );
    ImGui::DragFloat( "force", &_cfg._force, 1.0f, 0.0f, 200.0f );
    if (ImGui::Button( "AddForce" ))
    {
        _array_ext_forces.push_back( { 0, Eigen::Vector3f::Zero() } );
    }

    int id_to_delete = -1;
    for (int i = 0; i < _array_ext_forces.size(); i++)
    {
        ImGui::InputInt( (std::string( "id" ) + std::to_string( i )).c_str(), &_array_ext_forces[i].first );
        ImGui::DragFloat3( (std::string( "F" ) + std::to_string( i )).c_str(), _array_ext_forces[i].second.data(), 1.0f, -200.f, 200.f );
        if (ImGui::Button( "X" ))
        {
            id_to_delete = i;
        }
    }
    if (id_to_delete != -1)
    {
        _array_ext_forces.erase( std::next( _array_ext_forces.begin(), id_to_delete ) );
    }

    if (ImGui::Button( "ClearForce" ))
    {
        _array_ext_forces.clear();
    }
    ImGui::End();
}

void PD::PDGPUMetaballModel::PhysicalUpdate()
{
    InstrumentationTimer timer( "PhysicalUpdate" );
    //External force
    for (int i = 0; i < _mesh->BallsNum(); ++i)
    {
        _f_ext.col( i ).y() -= 9.8f;
    }
    for (auto& pair : _ext_forces)
    {
        _f_ext.col( pair.first ) += pair.second;
    }
    for (auto& pair : _array_ext_forces)
    {
        _f_ext.col( pair.first ) += pair.second.cast<Real>();
    }

#pragma omp parallel for
    for (int c = 0; c < _mesh->BallsNum(); c++)
    {
        _momentum.col( c ) = _x.col( c ) + _v.col( c ) * _cfg._dt;
        _momentum.col( c ) += _cfg._dt * _cfg._dt * _f_ext.col( c );
        _last_pos.col( c ) = _x.col( c );
        _x.col( c ) = _momentum.col( c );
    }

    for (int i = 0; i < _cfg._nb_solve; i++)
    {
#pragma omp parallel
        {
#pragma omp for 
            for (int j = 0; j < _constraints.size(); j++)
            {
                _constraints[j]->Project( _x, _p );
            }
#pragma omp for
            for (int r = 0; r < 3; r++)
            {
                auto rh_vec = _StAt * _p.row( r ).transpose() + _mass_matrix / (_cfg._dt * _cfg._dt) * _momentum.row( r ).transpose();
                _x.row( r ) = _llt.solve( rh_vec ).transpose();
            }
        }
    }

#pragma omp parallel
    {
#pragma omp for
        for (int c = 0; c < _mesh->BallsNum(); c++)
        {
            _v.col( c ) = 1.0f / _cfg._dt * (_x.col( c ) - _last_pos.col( c ));
        }
    }

    _aabb.max_corner = glm::vec3( -FLT_MAX );
    _aabb.min_corner = glm::vec3( FLT_MAX );
    for (int i = 0; i < _mesh->BallsNum(); i++)
    {
        _aabb.Expand( glm::vec3( _x.col( i )(0), _x.col( i )(1), _x.col( i )(2) ) + glm::vec3( _mesh->Ball( i ).r ) );
        _aabb.Expand( glm::vec3( _x.col( i )(0), _x.col( i )(1), _x.col( i )(2) ) - glm::vec3( _mesh->Ball( i ).r ) );
    }

    _f_ext.setZero();

    UpdateSkinInfoBuffer();
}

void PD::PDGPUMetaballModel::CudaPhysicalUpdate()
{
    InstrumentationTimer timer( "PhysicalUpdate" );

    static int framecount = 0;
    framecount++;

    //External force
    if (Input::IsKeyHeld( Input::Key::R ))
    {
        static float rad = 0.f;
        if (rad <= 3.1415f * 1.5)
        {
            auto q = Eigen::Quaternion<Real>( Eigen::AngleAxis<Real>( 0.01f, Vector3( 0, 1, 0 ) ) );
            Matrix3 m = q.toRotationMatrix();
            for (int i = 0; i < _host_attach_consts.size(); i++)
            {
                int id = _host_attach_consts[i].id;
                if (_rest_pos.col( id ).y() < -0.3f)
                {
                    vec3 p0 = _host_attach_consts[i].p;
                    Vector3 p = m * Vector3( p0.x, p0.y, p0.z );
                    _host_attach_consts[i].p = make_vec3( p.x(), p.y(), p.z() );
                }
            }
            _cuda_attach_consts.UpdateBuffer( _host_attach_consts.size(), _host_attach_consts.data() );
            rad += 0.01f;
        }
    }
    for (auto& pair : _ext_forces)
    {
        _f_ext.col( pair.first ) += pair.second;
    }
    for (auto& pair : _array_ext_forces)
    {
        _f_ext.col( pair.first ) += pair.second.cast<Real>();
    }
    cudaMemcpy( _cudapd.f_ext, _f_ext.data(), _cudapd.nb_points * sizeof( Real ) * 3, cudaMemcpyHostToDevice );

    if (framecount == 100)
    {
#pragma omp parallel for
        for (int c = 0; c < _mesh->BallsNum(); c++)
        {
            _f_ext.col( c ) = Vector3( 0, 0, 0 );
            _f_ext.col( c ).y() -= 9.8;
            _momentum.col( c ) = _x.col( c ) + _v.col( c ) * (double)_cfg._dt;
            _momentum.col( c ) += (double)_cfg._dt * (double)_cfg._dt * _f_ext.col( c );
            _last_pos.col( c ) = _x.col( c );
            _x.col( c ) = _momentum.col( c );
        }

        SparseMatrix sqrtM = _mass_matrix.cwiseSqrt();
        auto calc_W = [&]( const Matrix3X& pos )
        {
            double W = 0.0;
            //#pragma omp parallel for
            for (int r = 0; r < 3; r++)
            {
                float w = (sqrtM * (pos - _momentum).transpose()).squaredNorm() / (2 * _cfg._dt * _cfg._dt);
                //#pragma omp atomic
                W += w;
            }
            //#pragma omp parallel for
            for (int j = 0; j < _constraints.size(); j++)
            {
                auto p = _constraints[j]->GetP( pos );
                float w = 0.5 * _constraints[j]->_weight * (_CAS[j] * pos - p).squaredNorm();
                //#pragma omp atomic
                W += w;
            }
            return W;
        };

        //        for (int i = 0; i < 100; i++)
        //        {
        //            double W = calc_W( xacc );
        //            Wmin = std::min( Wmin, W );
        //            std::cout << "Acc err" << i << " " << W << std::endl;
        //#pragma omp parallel for
        //            for (int r = 0; r < 3; r++)
        //            {
        //                _g.row( r ) = ((_mass_matrix / (_cfg._dt * _cfg._dt)) * xacc.row( r ).transpose() - (_mass_matrix / (_cfg._dt * _cfg._dt)) * _momentum.row( r ).transpose()).transpose();
        //            }
        //
        //            for (int j = 0; j < _constraints.size(); j++)
        //            {
        //                auto p = _constraints[j]->GetP( xacc );
        //                _g.row( 0 ) += (_CStAtAS[j] * xacc.row( 0 ).transpose() - _CStAt[j] * p.row( 0 ).transpose()).transpose();
        //                _g.row( 1 ) += (_CStAtAS[j] * xacc.row( 1 ).transpose() - _CStAt[j] * p.row( 1 ).transpose()).transpose();
        //                _g.row( 2 ) += (_CStAtAS[j] * xacc.row( 2 ).transpose() - _CStAt[j] * p.row( 2 ).transpose()).transpose();
        //            }
        //
        //#pragma omp parallel for
        //            for (int r = 0; r < 3; r++)
        //            {
        //                VectorX d = _newtonllt.solve( -_g.row( r ).transpose() );
        //                xacc.row( r ) += d.transpose();
        //            }
        //
        //        }
        //
        //        Matrix3X x2 = _momentum;
        //        std::cout << "newton" << std::endl;
        //        for (int i = 0; i < 20; i++)
        //        {
        //            std::cout << i << " " << (calc_W( x2 ) - Wmin) / (W0 - Wmin) << std::endl;
        //#pragma omp parallel for
        //            for (int r = 0; r < 3; r++)
        //            {
        //                _g.row( r ) = ((_mass_matrix / (_cfg._dt * _cfg._dt)) * x2.row( r ).transpose() - (_mass_matrix / (_cfg._dt * _cfg._dt)) * _momentum.row( r ).transpose()).transpose();
        //            }
        //            for (int j = 0; j < _constraints.size(); j++)
        //            {
        //                auto p = _constraints[j]->GetP( x2 );
        //                _g.row( 0 ) += (_CStAtAS[j] * x2.row( 0 ).transpose() - _CStAt[j] * p.row( 0 ).transpose()).transpose();
        //                _g.row( 1 ) += (_CStAtAS[j] * x2.row( 1 ).transpose() - _CStAt[j] * p.row( 1 ).transpose()).transpose();
        //                _g.row( 2 ) += (_CStAtAS[j] * x2.row( 2 ).transpose() - _CStAt[j] * p.row( 2 ).transpose()).transpose();
        //            }
        //#pragma omp parallel for
        //            for (int r = 0; r < 3; r++)
        //            {
        //                VectorX d = _newtonllt.solve( -_g.row( r ).transpose() );
        //                x2.row( r ) += d.transpose();
        //            }
        //        }

        std::cout << "acc" << std::endl;
        Matrix3X xacc = _momentum;
        for (int i = 0; i < 2000; i++)
        {
#pragma omp parallel for
            for (int j = 0; j < _constraints.size(); j++)
            {
                _constraints[j]->Project( xacc, _p );
            }
#pragma omp parallel for
            for (int r = 0; r < 3; r++)
            {
                VectorX rh_vec = _StAt * _p.row( r ).transpose() + _mass_matrix / ((double)_cfg._dt * (double)_cfg._dt) * _momentum.row( r ).transpose();
                xacc.row( r ) = _llt.solve( rh_vec ).transpose();
            }
        }

        double momentum_err = (_momentum - xacc).norm();

        std::cout << "PD" << std::endl;
        Matrix3X x3 = _momentum;
        auto start_time = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 2000; i++)
        {
            auto time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
            std::cout << i << "," << (x3 - xacc).norm() / momentum_err << "," << time << std::endl;
#pragma omp parallel for
            for (int j = 0; j < _constraints.size(); j++)
            {
                _constraints[j]->Project( x3, _p );
            }
#pragma omp parallel for
            for (int r = 0; r < 3; r++)
            {
                VectorX rh_vec = _StAt * _p.row( r ).transpose() + _mass_matrix / ((double)_cfg._dt * (double)_cfg._dt) * _momentum.row( r ).transpose();
                x3.row( r ) = _llt.solve( rh_vec ).transpose();
            }
        }

        //        {
        //            std::cout << "Cheby" << std::endl;
        //            Matrix3X x4 = _momentum;
        //            float omega = 1.f;
        //            _last_pos1 = x4;
        //            _last_pos2 = x4;
        //            for (int i = 0; i < 100; i++)
        //            {
        //                std::cout << i << "," << (x4 - xacc).norm() / momentum_err << std::endl;
        //                if (i == 5)
        //                    omega = 2.f / (2.f - _rho * _rho);
        //                else if (i > 5)
        //                    omega = 4.f / (4.f - _rho * _rho * omega);
        //#pragma omp parallel for
        //                for (int j = 0; j < _constraints.size(); j++)
        //                {
        //                    _constraints[j]->Project( x4, _p );
        //                }
        //#pragma omp parallel for
        //                for (int r = 0; r < 3; r++)
        //                {
        //                    VectorX b = _StAt * _p.row( r ).transpose() + _mass_matrix / (_cfg._dt * _cfg._dt) * _momentum.row( r ).transpose();
        //                    VectorX Dinvb = _Dinv * b;
        //                    for (int j = 0; j < 1; j++)
        //                    {
        //                        x4.row( r ) = (_B * x4.row( r ).transpose() + Dinvb).transpose();
        //                    }
        //                    x4.row( r ) = omega * (0.9 * (x4.row( r ) - _last_pos1.row( r )) + _last_pos1.row( r ) - _last_pos2.row( r )) + _last_pos2.row( r );
        //                    _last_pos2.row( r ) = _last_pos1.row( r );
        //                    _last_pos1.row( r ) = x4.row( r );
        //                }
        //            }
        //        }

        {
            std::cout << "GPU" << std::endl;
            int len = 128;
            dim3 blocksize( len, 1, 1 );
            dim3 gridsize( (_cudapd.nb_points + len - 1) / len, 1, 1 );
            dim3 gridsize_ball( (_host_metaball_consts.size() + len - 1) / len, 1, 1 );
            dim3 gridsize_attach( (_host_attach_consts.size() + len - 1) / len, 1, 1 );
            dim3 gridsize_edge( (_host_edge_consts.size() + len - 1) / len, 1, 1 );
            cudaMemcpy( _cudapd.v, _v.data(), _v.size() * sizeof( Real ), cudaMemcpyHostToDevice );
            PDPred( gridsize, blocksize, _cudapd );

            cudaMemcpy( _cudapd.q, _x.data(), _x.size() * sizeof( Real ), cudaMemcpyHostToDevice );
            VectorX q0 = _x.row( 0 ).transpose();
            VectorX q1 = _x.row( 1 ).transpose();
            VectorX q2 = _x.row( 2 ).transpose();
            cudaMemcpy( _cudapd.qx, q0.data(), q0.size() * sizeof( Real ), cudaMemcpyHostToDevice );
            cudaMemcpy( _cudapd.qy, q1.data(), q1.size() * sizeof( Real ), cudaMemcpyHostToDevice );
            cudaMemcpy( _cudapd.qz, q2.data(), q2.size() * sizeof( Real ), cudaMemcpyHostToDevice );

            double omega = 1.f;
            PDProcessPos( gridsize, blocksize, _cudapd );
            start_time = std::chrono::high_resolution_clock::now();
            long long total_time = 0;
            auto last_time = std::chrono::high_resolution_clock::now();
            for (int i = 0; i < 2000; i++)
            {
                cudaMemcpy( _x.data(), _cudapd.q, _x.size() * sizeof( Real ), cudaMemcpyDeviceToHost );
                cudaDeviceSynchronize();
                std::cout << i << "," << (_x - xacc).norm() / momentum_err << ", " << total_time / 1000 << std::endl;

                last_time = std::chrono::high_resolution_clock::now();
                if (i == 3)
                    omega = 2.0 / (2.0 - _rho * _rho);
                else if (i > 3)
                    omega = 4.0 / (4.0 - _rho * _rho * omega);
                if (_host_attach_consts.size() > 0)
                {
                    PDProjectAttachConstraint( gridsize_attach, blocksize, _cudapd );
                }
                if (_cfg._const_type == 0 && _host_metaball_consts.size() > 0)
                {
                    PDProjectMetaballConstraint( gridsize_ball, blocksize, _cudapd );
                }
                cudaDeviceSynchronize();
                total_time += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - last_time).count();


#pragma omp parallel for
                for (int j = 0; j < _constraints.size(); j++)
                {
                    _constraints[j]->Project( _x, _p );
                }
                cudaMemcpy( _cudapd.p, _p.data(), _p.size() * sizeof( Real ), cudaMemcpyHostToDevice );
                VectorX p0 = _p.row( 0 ).transpose();
                VectorX p1 = _p.row( 1 ).transpose();
                VectorX p2 = _p.row( 2 ).transpose();
                cudaMemcpy( _cudapd.projx, p0.data(), p0.size() * sizeof( Real ), cudaMemcpyHostToDevice );
                cudaMemcpy( _cudapd.projy, p1.data(), p1.size() * sizeof( Real ), cudaMemcpyHostToDevice );
                cudaMemcpy( _cudapd.projz, p2.data(), p2.size() * sizeof( Real ), cudaMemcpyHostToDevice );
                cudaDeviceSynchronize();

                last_time = std::chrono::high_resolution_clock::now();
                for (int r = 0; r < 3; r++)
                {
                    CUDASpmv<Real>( _d_At, _d_proj[r], _d_rhvec[r], _Spmv_buf[r].Data() );
                    PDComputeRhvec( gridsize, blocksize, _d_rhvec_buf[r].Data(), _Jacobi_b_buf[r].Data(), r, _cudapd );
                    CUDASpmv<Real>( _Jacobi_Dinv, _Jacobi_b[r], _Jacobi_Dinvb[r], _Spmv_buf[r].Data() );
                    for (int j = 0; j < 1; j++)
                    {
                        CUDASpmv<Real>( _Jacobi_B, _Jacobi_x[r], _Jacobi_y[r], _Spmv_buf[r].Data() );
                        CUDAvplusv( _Jacobi_y_buf[r], 1.0, _Jacobi_Dinvb_buf[r], 1.0, _d_q[r], gridsize, blocksize );
                    }
                    PDChebyshev( gridsize, blocksize, _cudapd, _d_q[r].Data(), r, omega );
                }
                cudaDeviceSynchronize();
                //#pragma omp parallel for
                //                for (int r = 0; r < 3; r++)
                //                {
                //                    VectorX rh_vec = _StAt * _p.row( r ).transpose() + _mass_matrix / (_cfg._dt * _cfg._dt) * _momentum.row( r ).transpose();
                //                    _x.row( r ) = _llt.solve( rh_vec ).transpose();
                //                }
                total_time += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - last_time).count();
            }

            _x = _last_pos;
            cudaMemcpy( _cudapd.q, _x.data(), _cudapd.nb_points * sizeof( Real ) * 3, cudaMemcpyHostToDevice );
        }
    }


    int len = 256;
    dim3 blocksize( len, 1, 1 );
    dim3 gridsize( (_cudapd.nb_points + len - 1) / len );
    dim3 gridsize_attach( (_host_attach_consts.size() + len - 1) / len );
    dim3 gridsize_edge( (_host_edge_consts.size() + len - 1) / len );
    PDPred( gridsize, blocksize, _cudapd );
    float omega = 1.f;
    PDProcessPos( gridsize, blocksize, _cudapd );
    for (int i = 0; i < _cfg._nb_solve; i++)
    {
        if (i == 5)
        {
            omega = 2.f / (2.f - _rho * _rho);
        }
        else if (i > 5)
        {
            omega = 4.f / (4.f - _rho * _rho * omega);
        }
        {
            InstrumentationTimer timer( "Local Solve" );
            if (_host_attach_consts.size() > 0)
            {
                PDProjectAttachConstraint( gridsize_attach, blocksize, _cudapd );
            }
            if (_cfg._const_type == 0 && _host_metaball_consts.size() > 0)
            {
                PDProjectMetaballConstraint( gridsize, blocksize, _cudapd );
            }
            if (_cfg._const_type == 1 && _host_edge_consts.size() > 0)
            {
                PDProjectEdgeConstraint( gridsize_edge, blocksize, _cudapd );
            }
        }
        for (int r = 0; r < 3; r++)
        {
            InstrumentationTimer timer( "Global Solve" );
            CUDASpmv<Real>( _d_At, _d_proj[r], _d_rhvec[r], _Spmv_buf[r].Data() );
            PDComputeRhvec( gridsize, blocksize, _d_rhvec_buf[r].Data(), _Jacobi_b_buf[r].Data(), r, _cudapd );
            CUDASpmv<Real>( _Jacobi_Dinv, _Jacobi_b[r], _Jacobi_Dinvb[r], _Spmv_buf[r].Data() );
            for (int j = 0; j < 5; j++)
            {
                CUDASpmv<Real>( _Jacobi_B, _Jacobi_x[r], _Jacobi_y[r], (void*)_Spmv_buf[r].Data() );
                CUDAvplusv( _Jacobi_y_buf[r], 1.0, _Jacobi_Dinvb_buf[r], 1.0, _d_q[r], gridsize, blocksize );
            }
            PDChebyshev( gridsize, blocksize, _cudapd, _d_q[r].Data(), r, omega );
        }
    }

    PDUpdateVel( gridsize, blocksize, _cudapd );
    cudaMemcpy( _x.data(), _cudapd.q, _cudapd.nb_points * sizeof( Real ) * 3, cudaMemcpyDeviceToHost );
    cudaMemcpy( _v.data(), _cudapd.v, _cudapd.nb_points * sizeof( Real ) * 3, cudaMemcpyDeviceToHost );
    cudaMemcpy( _last_pos.data(), _cudapd.qlast, _cudapd.nb_points * sizeof( Real ) * 3, cudaMemcpyDeviceToHost );
    if (_cfg._const_type == 0)
    {
        PDUpdateSkinningInfo( gridsize, blocksize, _cudapd );
    }
    cudaDeviceSynchronize();

    //_aabb.max_corner = glm::vec3( -FLT_MAX );
    //_aabb.min_corner = glm::vec3( FLT_MAX );
    //for (int i = 0; i < _mesh->BallsNum(); i++)
    //{
    //    _aabb.Expand( ToGLM( _x.col( i ).cast<float>() ) + glm::vec3( _mesh->Ball( i ).r ) );
    //    _aabb.Expand( ToGLM( _x.col( i ).cast<float>() ) - glm::vec3( _mesh->Ball( i ).r ) );
    //}
    _f_ext.setZero();
}

void PD::PDGPUMetaballModel::CollisionDetection()
{
    InstrumentationTimer timer( "CollisionDetection" );
    std::vector<RigidStatic*> rigid_bodys = Scene::active->GetAllChildOfType<RigidStatic>();
    std::vector<PD::PDGPUMetaballModel*> pd_models = Scene::active->GetAllChildOfType<PD::PDGPUMetaballModel>();
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
            if (!AABBIntersect( rigid->BVH().BoundingBox(), _aabb ))
                continue;

            HalfEdgeMesh& sur = rigid->Surface();
            auto pos = ToGLM( _x.col( i ) );
            auto last_pos = ToGLM( _last_pos.col( i ) );
            Vector3 dist( 0.f, 0.f, 0.f );
            for (int j = 0; j < sur.GetFaceNumber(); j++)
            {
                auto [ia, ib, ic] = sur.GetFaceIndices( j );
                glm::vec3 p0 = rigid->GetPos( ia );
                glm::vec3 p1 = rigid->GetPos( ib );
                glm::vec3 p2 = rigid->GetPos( ic );

                //glm::vec3 pc;
                //float depth;
                //if (BallTriIntersect( pos, pi.r, p0, p1, p2, &pc, &depth ))
                //{
                //    glm::vec3 newpos = pos + glm::normalize( pos - pc ) * depth;
                //    _current_pos.col( i ) = ToEigen( newpos );
                //    Vector3 v = _current_vel.col( i );
                //    Vector3 n = ToEigen( glm::normalize( glm::cross( p1 - p0, p2 - p0 ) ) );
                //    Vector3 vn = n * v.dot( n );
                //    Vector3 vt = v - vn;
                //    _current_vel.col( i ) = -vn + vt;
                //}

                MovingSphereTriIntersectInfo info;
                if (MovingSphereTriIntersect( last_pos, pi.r, pos - last_pos, p0, p1, p2, &info ))
                {
                    //pi.color = glm::vec3( 1, 0, 0 );
                    auto dx = pos - last_pos;
                    auto norm_dx = dx;
                    float lendx = glm::length( dx );
                    if (lendx > 1e-6f)
                        norm_dx = glm::normalize( norm_dx );

                    Vector3 n = ToEigen( info.nc ).cast<Real>();
                    Vector3 v = _v.col( i );
                    Vector3 newpos = ToEigen( last_pos + dx * (Real)info.t );
                    Vector3 pene = _x.col( i ) - newpos;
                    Vector3 pene_n = n * pene.dot( n );
                    Vector3 pene_t = pene - pene_n;
                    Vector3 dxn = n * ToEigen( dx ).cast<Real>().dot( n );
                    Vector3 dxt = ToEigen( dx ).cast<Real>() - dxn;

                    Vector3 vn = n * v.dot( n );
                    Vector3 vt = v - vn;
                    _v.col( i ) = -vn + vt;
                    _x.col( i ) = newpos + 0.01f * (-dxn + 0.5f * dxt) * (1.0f - info.t);
                    pos = ToGLM( _x.col( i ).eval() );
                }
            }


        }

        for (RigidBall* rigid : rigid_balls)
        {
            auto pos = ToGLM( _x.col( i ) );
            auto last_pos = ToGLM( _last_pos.col( i ) );
            auto info = BallBallIntersect( pos, pi.r, rigid->GetPos(), rigid->GetRadius() );
            if (info.has_value())
            {
                auto newpos = pos - (Real)info->d * glm::vec<3, Real>( info->c1toc2 );
                _x.col( i ) = ToEigen( newpos );
                Vector3 v = _v.col( i );
                Vector3 n = ToEigen( info->c1toc2 ).cast<Real>();
                Vector3 vn = n * v.dot( n );
                Vector3 vt = v - vn;
                _v.col( i ) = -vn + vt;
            }
        }

        for (RigidSDF* rigid_sdf : rigid_sdfs)
        {
            glm::vec3 normal;
            float depth;
            glm::vec3 pos( _x.col( i )(0), _x.col( i )(1), _x.col( i )(2) );

            if (rigid_sdf->CheckBall( pos, pi.r, &normal, &depth ))
            {
                Vector3 n( normal.x, normal.y, normal.z );
                Vector3 dx = _x.col( i ) - _last_pos.col( i );
                Vector3 dx_n = n * dx.dot( n );
                Vector3 dx_t = dx - dx_n;
                float s = std::min( 1000 * depth, 1.0f );
                Vector3 fric = -s * dx_t;

                Vector3 v = _v.col( i );
                Vector3 vn = n * v.dot( n );
                Vector3 vt = v - vn;

                _x.col( i ) += n * depth - s * dx_t;
                _v.col( i ) = -1.0 * vn + (1.f - s) * vt;
            }
        }

        //for (PD::PDGPUMetaballModel* model : pd_models)
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
        //                _current_pos.col( i ) = cp0 + 0.00f * (-dx0n + 0.5f * dx0t) * (1.0f - t);
        //                model->_current_pos.col( j ) -= pene1n + 0.5 * pene1t;
        //                model->_current_pos.col( j ) = cp1 + 0.00 * (-dx1n + 0.5f * dx1t) * (1.0f - t);

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

        glm::vec3 pos = ToGLM( _x.col( i ).eval() );
        if (pos.y < -2.f)
        {
            Vector3 n( 0.f, 1.f, 0.f );
            Vector3 dx = _x.col( i ) - _last_pos.col( i );
            Vector3 dxn = n * dx.dot( n );
            Vector3 dxt = dx - dxn;
            _x.col( i ) -= dxt * 0.9f;
            _x.col( i ).y() = -2.f;
            Vector3 v = _v.col( i );
            Vector3 vn = n * v.dot( n );
            Vector3 vt = v - vn;
            _v.col( i ) = -vn + 0.01 * vt;
        }
    }

    cudaMemcpy( _cudapd.q, _x.data(), _cudapd.nb_points * sizeof( float3 ), cudaMemcpyHostToDevice );
    cudaMemcpy( _cudapd.v, _v.data(), _cudapd.nb_points * sizeof( float3 ), cudaMemcpyHostToDevice );
}

void PD::PDGPUMetaballModel::PostPhysicalUpdate()
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

void PD::PDGPUMetaballModel::CreateSurfaceMapping()
{
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<float> avg_dist_to_neighbors( _mesh->BallsNum(), 0.f );
#pragma omp parallel for
    for (int i = 0; i < _mesh->BallsNum(); i++)
    {
        float dist = 0.f;
        for (int j : _mesh->Ball( i ).neighbors)
        {
            dist += glm::distance( _mesh->Ball( i ).x0, _mesh->Ball( j ).x0 );
        }
        dist /= _mesh->Ball( i ).neighbors.size();
        avg_dist_to_neighbors[i] = dist;
    }
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
            //float dot = glm::dot( _surface->_edges[_surface->_vertices[i].edge].normal, ball.x - _surface->_vertices[i].pos );
            float dist = glm::abs( glm::distance( ball.x0, _surface->GetRestPos( i ) ) );
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

void PD::PDGPUMetaballModel::UpdateSkinInfoBuffer()
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
            auto gu = _mesh->Ball( i ).gu;
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

void PD::PDGPUMetaballModel::MapSurface()
{
#pragma omp parallel for
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

    _surface->UpdatePosBuffer();
    //    _surface->UpdateNormal();
      //  _surface->UpdateAttrBuffer();
}

void PD::PDGPUMetaballModel::SampleFromVoxel( float steplen )
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

void PD::PDGPUMetaballModel::ComputeBallOrit()
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
        glm::mat3 A2 = ball.m * ball.r * ball.r * glm::toMat3( ball.q );
        for (int j : ball.neighbors)
        {
            const Particle& nei = _mesh->Ball( j );
            A1 += nei.m * glm::TensorProduct( nei.x, nei.x0 );
            A2 += nei.m * nei.r * nei.r * glm::toMat3( nei.q );
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

        ball.q = glm::normalize( glm::toQuat( R ) );
        auto eigenR = ToEigen( R );
        ball.R = eigenR.cast<Real>();
        ball.gu = eigenR.cast<Real>();
    }
}

void PD::PDGPUMetaballModel::ComputeBallOrit2()
{
#pragma omp parallel for
    for (int i = 0; i < _mesh->BallsNum(); i++)
    {
        Vector3 ui = _x.col( i ) - _rest_pos.col( i );
        Vector3 sx = Vector3::Zero();
        Vector3 sy = Vector3::Zero();
        Vector3 sz = Vector3::Zero();
        float wsum = 0.f;
        int cnt = 0;
        for (int j : _mesh->Ball( i ).neighbors)
        {
            Vector3 uj = _x.col( j ) - _rest_pos.col( j );
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

        _mesh->Ball( i ).R = (U * V.transpose()).cast<Real>();
        _mesh->Ball( i ).gu = F.cast<Real>();
    }

}

void PD::PDGPUMetaballModel::ComputeAinvForEdgeConsts()
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

void PD::PDGPUMetaballModel::ComputeBallOutsideVec()
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

float PD::PDGPUMetaballModel::Compare( const PDGPUMetaballModel* other )
{
    float dist = 0.f;
    if (_mesh->BallsNum() != other->_mesh->BallsNum())
    {
        return std::numeric_limits<float>::infinity();
    }
    return (_x - other->_x).norm();
}
