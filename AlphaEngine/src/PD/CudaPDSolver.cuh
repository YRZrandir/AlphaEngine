#include "cuda_runtime.h"
#include "device_launch_parameters.h";
#include "util/CudaBuffer.h"
#include "util/CudaVecHelper.cuh"

struct CudaAttachConst
{
    int id;
    int loc;
    float3 p;
};

struct CudaMetaballConstNeiInfo
{
    int id;
    float w;
};

struct CudaMetaballConst
{
    int id;
    int loc;
    int nei_base;
    int nei_count;
    float weight;
    float3 invA[3];
    float3 F[3];
};

struct CudaEdgeConst
{
    int id0;
    int id1;
    int loc;
    float weight;
    float restlen;
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
    float dt;
    int nb_points;
    float gravity = 9.8f;

    float* m;
    float3* q0;
    float3* q;
    float* qx;
    float* qy;
    float* qz;
    float3* qlast;
    float3* qlast1;
    float3* qlast2;
    float3* v;
    float3* momentum;
    float3* f_ext;
    float3* p;
    float* projx;
    float* projy;
    float* projz;

    CudaAttachConst* attach_consts;
    int nb_attach;
    CudaMetaballConst* metaball_consts;
    CudaMetaballConstNeiInfo* metaball_neiinfos;
    CudaEdgeConst* edge_consts;
    int nb_edges;

    CudaSkinningInfo* _skin_info;
    float3 displacement;

    float k_stretch;
    float k_attach;
};

void PDPred( dim3 gridsize, dim3 blocksize, CudaPDSystem sys );

void PDProcessPos( dim3 gridsize, dim3 blocksize, CudaPDSystem sys );

void PDComputeRhvec( dim3 gridsize, dim3 blocksize, float* Atp, float* result, int id, CudaPDSystem sys );

void PDProjectAttachConstraint( dim3 gridsize, dim3 blocksize, CudaPDSystem sys );

void PDProjectMetaballConstraint( dim3 gridsize, dim3 blocksize, CudaPDSystem sys );

void PDProjectEdgeConstraint( dim3 gridsize, dim3 blocksize, CudaPDSystem sys );

void PDChebyshev( dim3 gridsize, dim3 blocksize, CudaPDSystem sys, float* v, int r, float omega );

void PDUpdateVel( dim3 gridsize, dim3 blocksize, CudaPDSystem sys );

void PDUpdateSkinningInfo( dim3 gridsize, dim3 blocksize, CudaPDSystem sys );