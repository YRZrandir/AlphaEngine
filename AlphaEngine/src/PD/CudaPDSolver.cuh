#include "cuda_runtime.h"
#include "device_launch_parameters.h";
#include "util/CudaBuffer.h"
#include "util/CudaVecHelper.cuh"
#include "util/CudaMathTypes.cuh"

struct CudaAttachConst
{
    vec3 p;
    int id;
    int loc;
};

struct CudaMetaballConstNeiInfo
{
    Real w;
    int id;
};

struct CudaMetaballConst
{
    vec3 invA[3];
    vec3 F[3];
    Real weight;
    int id;
    int loc;
    int nei_base;
    int nei_count;
};

struct CudaEdgeConst
{
    Real weight;
    Real restlen;
    int id0;
    int id1;
    int loc;
};

struct CudaSkinningInfo
{
    float4 u;
    float4 xi;
    float4 gu[4];
    float4 R[4];

    CudaSkinningInfo()
    {
        u = make_float4( 0.f, 0.f, 0.f, 0.f );
        xi = make_float4( 0.f, 0.f, 0.f, 0.f );
        for (int i = 0; i < 4; i++)
        {
            gu[i] = make_float4( 0.f, 0.f, 0.f, 0.f );
            R[i] = make_float4( 0.f, 0.f, 0.f, 0.f );
        }
        gu[0].x = 1.f;
        gu[1].y = 1.f;
        gu[2].z = 1.f;
        gu[3].w = 1.f;
        R[0].x = 1.f;
        R[1].y = 1.f;
        R[2].z = 1.f;
        R[3].w = 1.f;
    }
};

struct CudaPDSystem
{
    Real dt;
    int nb_points;
    float gravity = 9.8f;

    Real* m;
    vec3* q0;
    vec3* q;
    Real* qx;
    Real* qy;
    Real* qz;
    vec3* qlast;
    vec3* qlast1;
    vec3* qlast2;
    vec3* v;
    vec3* momentum;
    vec3* f_ext;
    vec3* p;
    Real* projx;
    Real* projy;
    Real* projz;

    CudaAttachConst* attach_consts;
    int nb_attach;
    CudaMetaballConst* metaball_consts;
    CudaMetaballConstNeiInfo* metaball_neiinfos;
    CudaEdgeConst* edge_consts;
    int nb_edges;

    CudaSkinningInfo* _skin_info;
    vec3 displacement;

    Real k_stretch;
    Real k_attach;
};

void PDPred( dim3 gridsize, dim3 blocksize, CudaPDSystem sys );

void PDProcessPos( dim3 gridsize, dim3 blocksize, CudaPDSystem sys );

void PDComputeRhvec( dim3 gridsize, dim3 blocksize, Real* Atp, Real* result, int id, CudaPDSystem sys );

void PDProjectAttachConstraint( dim3 gridsize, dim3 blocksize, CudaPDSystem sys );

void PDProjectMetaballConstraint( dim3 gridsize, dim3 blocksize, CudaPDSystem sys );

void PDProjectEdgeConstraint( dim3 gridsize, dim3 blocksize, CudaPDSystem sys );

void PDChebyshev( dim3 gridsize, dim3 blocksize, CudaPDSystem sys, Real* v, int r, Real omega );

void PDUpdateVel( dim3 gridsize, dim3 blocksize, CudaPDSystem sys );

void PDUpdateSkinningInfo( dim3 gridsize, dim3 blocksize, CudaPDSystem sys );

void CUDAvplusv( CudaBuffer<float>& a, float sa, CudaBuffer<float>& b, float sb, CudaBuffer<float>& dst, dim3 gridsize, dim3 blocksize );
void CUDAvplusv( CudaBuffer<double>& a, double sa, CudaBuffer<double>& b, double sb, CudaBuffer<double>& dst, dim3 gridsize, dim3 blocksize );
