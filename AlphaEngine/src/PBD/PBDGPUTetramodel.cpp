#include "PBDGPUTetraModel.h"
#include <omp.h>
#include "util/GlobalTimer.h"
#include "util/Instrumentor.h"
#include "cuda_gl_interop.h"
#include "input/input.h"
#include <imgui/imgui.h>

namespace PBD
{
GPUTetraModel::GPUTetraModel( const std::string& path, const std::string& surface, float density )
    :_density( density )
{
    _surface = std::make_unique<HalfEdgeMesh>( surface );
    _surface->UpdateNormal();
    _surface->UpdateAttrBuffer();
    _surface->UpdatePosBuffer();

    _tet_mesh = std::unique_ptr<TetraMesh>();
    _tet_mesh->CreateFromSurface( path );

    //ExtrudeTetMesh();
    InitParticles();
    InitConstraints();
    InitCudaBuffer();
}

void GPUTetraModel::ExtrudeTetMesh()
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

void GPUTetraModel::InitParticles()
{
    size_t nb_points = _tet_mesh->mPoints.size();
    _x0.resize( 3, nb_points );
    _x.resize( 3, nb_points );
    _p.resize( 3, nb_points );
    _v.resize( 3, nb_points );
    _f.resize( 3, nb_points );
    _m.resize( nb_points );
    _invm.resize( nb_points );

#pragma omp parallel for
    for (int i = 0; i < nb_points; i++)
    {
        const auto& p = _tet_mesh->mPoints[i];
        _x0.col( i ) = Eigen::Vector3f( p.x, p.y, p.z );
        _x.col( i ) = _x0.col( i );
        _p.col( i ) = _x0.col( i );
        _v.col( i ) = Eigen::Vector3f( 0.f, 0.f, 0.f );
        _f.col( i ) = Eigen::Vector3f( 0.f, 0.f, 0.f );
        _m( i ) = 0.f;
        _invm( i ) = 0.f;
    }

    size_t nb_tet = _tet_mesh->mTetras.size();
    for (int i = 0; i < nb_tet; i++)
    {
        float volume = std::abs( _tet_mesh->GetTetraVolume( i ) );

        float m_over_4 = volume * _density / 4.0f;
        _m( _tet_mesh->mTetras[i].a ) += m_over_4;
        _m( _tet_mesh->mTetras[i].b ) += m_over_4;
        _m( _tet_mesh->mTetras[i].c ) += m_over_4;
        _m( _tet_mesh->mTetras[i].d ) += m_over_4;
    }

    for (int i = 0; i < nb_points; i++)
    {
        _invm( i ) = 1.f / _m( i );
    }
}

void GPUTetraModel::InitConstraints()
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
        cst_set.insert( StretchConstraint( tetra[1], tetra[0], 0.8f, glm::distance( points[0], points[1] ) ) );
        cst_set.insert( StretchConstraint( tetra[2], tetra[0], 0.8f, glm::distance( points[0], points[2] ) ) );
        cst_set.insert( StretchConstraint( tetra[3], tetra[0], 0.8f, glm::distance( points[0], points[3] ) ) );
        cst_set.insert( StretchConstraint( tetra[1], tetra[2], 0.8f, glm::distance( points[1], points[2] ) ) );
        cst_set.insert( StretchConstraint( tetra[1], tetra[3], 0.8f, glm::distance( points[1], points[3] ) ) );
        cst_set.insert( StretchConstraint( tetra[2], tetra[3], 0.8f, glm::distance( points[2], points[3] ) ) );

        _volume_constraints.emplace_back( tetra[0], tetra[1], tetra[2], tetra[3], 0.8f, std::abs( _tet_mesh->GetTetraVolume( i ) ) );
    }

    _stretch_constraints.reserve( cst_set.size() );
    for (auto& sc : cst_set)
    {
        _stretch_constraints.push_back( EdgeConstraint( sc._indices[0], sc._indices[1], sc._k, sc._d ) );
    }

    for (int i = 0; i < _tet_mesh->mPoints.size(); i++)
    {
        if (_x0( 1, i ) > 0.15f && _x0( 0, i ) < -0.2f)
        {
            _attach_constraints.push_back( PBD::AttachmentConstraint( i, glm::vec3( _x0( 0, i ), _x0( 1, i ), _x0( 2, i ) ) ) );
        }
    }
}

void GPUTetraModel::InitCudaBuffer()
{
    _sys.dt = 0.01;
    _sys.damp = 0.9f;

    _sys.nb_points = _tet_mesh->mPoints.size();
    _sys.x0.UpdateBuffer( _sys.nb_points * 3, _x0.data() );
    _sys.x.UpdateBuffer( _sys.nb_points * 3, _x.data() );
    _sys.p.UpdateBuffer( _sys.nb_points * 3, _p.data() );
    _sys.v.UpdateBuffer( _sys.nb_points * 3, _v.data() );
    _sys.m.UpdateBuffer( _sys.nb_points, _m.data() );
    _sys.dx.UpdateBuffer( _sys.nb_points * 3, _p.data() );
    _sys.n.UpdateBuffer( _sys.nb_points, nullptr );
    _sys.invm.UpdateBuffer( _sys.nb_points, _invm.data() );
    _sys.nb_stretch = _stretch_constraints.size();
    _sys.nb_attach = _attach_constraints.size();
    _sys.nb_volume = _volume_constraints.size();
    _sys.c_stretch.UpdateBuffer( _sys.nb_stretch, _stretch_constraints.data() );
    _sys.c_attach.UpdateBuffer( _sys.nb_attach, _attach_constraints.data() );
    _sys.c_volume.UpdateBuffer( _sys.nb_volume, _volume_constraints.data() );

    _sys.k_stretch = 0.5f;
    _sys.k_volume = 0.5f;
}

void GPUTetraModel::Update()
{
    if (Input::IsKeyDown( Input::Key::P ))
    {
        _simulate = !_simulate;
    }
    if (Input::IsKeyHeld( Input::Key::UP ))
    {
        _sys.k_volume = std::clamp( _sys.k_volume + 0.01f, 0.f, 1.f );
    }
    if (Input::IsKeyHeld( Input::Key::DOWN ))
    {
        _sys.k_volume = std::clamp( _sys.k_volume - 0.01f, 0.f, 1.f );
    }
    if (Input::IsKeyHeld( Input::Key::N ))
    {
        _sys.k_stretch = std::clamp( _sys.k_stretch + 0.01f, 0.f, 1.f );
    }
    if (Input::IsKeyHeld( Input::Key::M ))
    {
        _sys.k_stretch = std::clamp( _sys.k_stretch - 0.01f, 0.f, 1.f );
    }
    if (_simulate)
    {
        //InstrumentationTimer updatetimer( "update" );
        _sys.dt = GlobalTimer::RecentAvgFrameTimeMs() / 10000;
        _sys.dt = std::clamp( _sys.dt, 1e-6f, 0.03f );
        for (int i = 0; i < 10; i++)
        {
            InstrumentationTimer timer( "PhysicalUpdate" );
            PhysicalUpdate();
        }

        InstrumentationTimer timer( "Transfer" );
        cudaGraphicsResource_t res[1];
        cudaGraphicsGLRegisterBuffer( &res[0], _tet_mesh->mVBO->GetID(), cudaGraphicsRegisterFlagsWriteDiscard );
        cudaGraphicsMapResources( 1, res, 0 );
        void* ptr;
        size_t size;
        cudaGraphicsResourceGetMappedPointer( &ptr, &size, res[0] );
        cudaMemcpy( ptr, _sys.x.Data(), _sys.x.MemSize(), cudaMemcpyDeviceToDevice );
        cudaGraphicsUnmapResources( 1, res );
    }
}

void GPUTetraModel::PhysicalUpdate()
{
    int len = 1024;
    dim3 blocksize( len, 1, 1 );
    dim3 gridsize( (_sys.nb_points + len - 1) / len );

    PBDPredStep( blocksize, gridsize, _sys );
    cudaDeviceSynchronize();
    for (int i = 0; i < 3; i++)
    {
        _sys.nb_it = i + 1;
        dim3 gridsize2( (_sys.nb_stretch + len - 1) / len );
        PBDStretchConstraintStep( blocksize, gridsize2, _sys );
        cudaDeviceSynchronize();

        dim3 gridsize4( (_sys.nb_volume + len - 1) / len );
        PBDVolumeConstraintStep( blocksize, gridsize4, _sys );
        cudaDeviceSynchronize();

        dim3 gridsize3( (_sys.nb_attach + len - 1) / len );
        PBDAttachConstraintStep( blocksize, gridsize3, _sys );
        cudaDeviceSynchronize();
        PBDPostStep( blocksize, gridsize, _sys );
        cudaDeviceSynchronize();
    }
    PBDUpdateVel( blocksize, gridsize, _sys );
    cudaDeviceSynchronize();
}

void GPUTetraModel::Draw()
{
    //InstrumentationTimer timer( "Draw" );
    _tet_mesh->Draw();
}
}