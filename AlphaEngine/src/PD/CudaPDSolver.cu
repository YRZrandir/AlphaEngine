#include "CudaPDSolver.cuh"
#include "util/CudaVecHelper.cu"
#include "math_functions.h"

__global__ void cudaPDPred( CudaPDSystem sys );
__global__ void cudaPDProcessPos( CudaPDSystem sys );
__global__ void cudaPDComputeRhvec( Real* Atp, Real* result, int id, CudaPDSystem sys );
__global__ void cudaPDProjectAttachConstraint( CudaPDSystem sys );
__global__ void cudaPDProjectMetaballConstraint( CudaPDSystem sys );
__global__ void cudaPDProjectEdgeConstraint( CudaPDSystem sys );
__global__ void cudaPDChebyshev( CudaPDSystem sys, Real* v, int r, Real omega );
__global__ void cudaPDUpdateVel( CudaPDSystem sys );
__global__ void cudaPDUpdateSkinningInfo( CudaPDSystem sys );
__global__ void CUDAvplusv_impl( float* a, float sa, float* b, float sb, float* dst, int size );
__global__ void CUDAvplusv_impl( double* a, double sa, double* b, double sb, double* dst, int size );

void PDPred( dim3 gridsize, dim3 blocksize, CudaPDSystem sys )
{
    cudaPDPred << <gridsize, blocksize >> > (sys);
}

void PDProcessPos( dim3 gridsize, dim3 blocksize, CudaPDSystem sys )
{
    cudaPDProcessPos << <gridsize, blocksize >> > (sys);
}

void PDComputeRhvec( dim3 gridsize, dim3 blocksize, Real* Atp, Real* result, int id, CudaPDSystem sys )
{
    cudaPDComputeRhvec << <gridsize, blocksize >> > (Atp, result, id, sys);
}

void PDProjectAttachConstraint( dim3 gridsize, dim3 blocksize, CudaPDSystem sys )
{
    cudaPDProjectAttachConstraint << <gridsize, blocksize >> > (sys);
}

void PDProjectMetaballConstraint( dim3 gridsize, dim3 blocksize, CudaPDSystem sys )
{
    cudaPDProjectMetaballConstraint << <gridsize, blocksize >> > (sys);
}

void PDProjectEdgeConstraint( dim3 gridsize, dim3 blocksize, CudaPDSystem sys )
{
    cudaPDProjectEdgeConstraint << <gridsize, blocksize >> > (sys);
}

void PDChebyshev( dim3 gridsize, dim3 blocksize, CudaPDSystem sys, Real* v, int r, Real omega )
{
    cudaPDChebyshev << <gridsize, blocksize >> > (sys, v, r, omega);
}

void PDUpdateVel( dim3 gridsize, dim3 blocksize, CudaPDSystem sys )
{
    cudaPDUpdateVel << <gridsize, blocksize >> > (sys);
}

void PDUpdateSkinningInfo( dim3 gridsize, dim3 blocksize, CudaPDSystem sys )
{
    cudaPDUpdateSkinningInfo << <gridsize, blocksize >> > (sys);
}

void CUDAvplusv( CudaBuffer<float>& a, float sa, CudaBuffer<float>& b, float sb, CudaBuffer<float>& dst, dim3 gridsize, dim3 blocksize )
{
    if (!(a.Count() == b.Count() && a.Count() == dst.Count()))
    {
        __debugbreak();
    }
    CUDAvplusv_impl << <gridsize, blocksize >> > (a.Data(), sa, b.Data(), sb, dst.Data(), a.Count());
}

void CUDAvplusv( CudaBuffer<double>& a, double sa, CudaBuffer<double>& b, double sb, CudaBuffer<double>& dst, dim3 gridsize, dim3 blocksize )
{
    if (!(a.Count() == b.Count() && a.Count() == dst.Count()))
    {
        __debugbreak();
    }
    CUDAvplusv_impl << <gridsize, blocksize >> > (a.Data(), sa, b.Data(), sb, dst.Data(), a.Count());
}

__global__ void CUDAvplusv_impl( float* a, float sa, float* b, float sb, float* dst, int size )
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= size)
        return;
    dst[i] = a[i] * sa + b[i] * sb;
}

__global__ void CUDAvplusv_impl( double* a, double sa, double* b, double sb, double* dst, int size )
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= size)
        return;
    dst[i] = a[i] * sa + b[i] * sb;
}

__global__ void cudaPDPred( CudaPDSystem sys )
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= sys.nb_points)
        return;

    sys.f_ext[i] = make_vec3( 0.0, 0.0, 0.0 );
    sys.f_ext[i].y -= sys.gravity;
    sys.momentum[i] = sys.q[i] + sys.v[i] * sys.dt + sys.f_ext[i] * sys.dt * sys.dt;
    sys.qlast[i] = sys.q[i];
    sys.q[i] = sys.momentum[i];
    sys.qx[i] = sys.q[i].x;
    sys.qy[i] = sys.q[i].y;
    sys.qz[i] = sys.q[i].z;
    sys.f_ext[i] = make_vec3( 0.0, 0.0, 0.0 );
}

__global__ void cudaPDProcessPos( CudaPDSystem sys )
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= sys.nb_points)
        return;
    //sys.q[i] = make_vec3( sys.qx[i], sys.qy[i], sys.qz[i] );
    sys.qlast1[i] = sys.q[i];
    sys.qlast2[i] = sys.q[i];
}

__global__ void cudaPDComputeRhvec( Real* Atp, Real* result, int id, CudaPDSystem sys )
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= sys.nb_points)
        return;
    result[i] = Atp[i] + sys.m[i] / (sys.dt * sys.dt) * ((Real*)&sys.momentum[i])[id];
}

__global__ void cudaPDProjectAttachConstraint( CudaPDSystem sys )
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= sys.nb_attach)
        return;
    int loc = sys.attach_consts[i].loc;
    vec3 p = sys.attach_consts[i].p;

    sys.projx[loc] = p.x;
    sys.projy[loc] = p.y;
    sys.projz[loc] = p.z;
    sys.p[loc] = p;
}

__global__ void cudaPDProjectMetaballConstraint( CudaPDSystem sys )
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= sys.nb_points)
        return;

    CudaMetaballConst C = sys.metaball_consts[i];
    CudaMetaballConstNeiInfo* neighbors = &sys.metaball_neiinfos[C.nei_base];
    int id0 = neighbors[0].id;
    int loc = C.loc;

    //Compute F
    vec3 ui = sys.q[id0] - sys.q0[id0];
    vec3 sx = make_vec3( 0.0, 0.0, 0.0 );
    vec3 sy = make_vec3( 0.0, 0.0, 0.0 );
    vec3 sz = make_vec3( 0.0, 0.0, 0.0 );

    Real wsum = 0.f;

    for (int j = 1; j < C.nei_count; j++)
    {
        int neiid = neighbors[j].id;
        vec3 uj = sys.q[neiid] - sys.q0[neiid];
        vec3 xij = sys.q0[neiid] - sys.q0[id0];
        Real wij = neighbors[j].w;

        wsum += wij;
        sx += (uj.x - ui.x) * wij * xij;
        sy += (uj.y - ui.y) * wij * xij;
        sz += (uj.z - ui.z) * wij * xij;
    }

    sx /= wsum;
    sy /= wsum;
    sz /= wsum;

    vec3 dux = MulMv3x3( C.invA, sx );
    vec3 duy = MulMv3x3( C.invA, sy );
    vec3 duz = MulMv3x3( C.invA, sz );

    vec3 F[3];
    F[0] = dux;
    F[1] = duy;
    F[2] = duz;
    TransposeInplace3x3( F );
    F[0].x += 1.0;
    F[1].y += 1.0;
    F[2].z += 1.0;

    vec3 U[3];
    vec3 V[3];
    vec3 S[3];
    svd( &F[0].x, &U[0].x, &S[0].x, &V[0].x );

    TransposeInplace3x3( V );

    vec3 T[3];
    multAB( &U[0].x, &V[0].x, &T[0].x );

    sys.metaball_consts[i].F[0] = T[0];
    sys.metaball_consts[i].F[1] = T[1];
    sys.metaball_consts[i].F[2] = T[2];

    sys.p[loc] = make_vec3( 0, 0, 0 );
    for (int j = 1; j < C.nei_count; j++)
    {
        sys.p[loc + j] = MulMv3x3( T, sys.q0[neighbors[j].id] - sys.q0[id0] );
    }

    for (int j = 0; j < C.nei_count; j++)
    {
        sys.projx[loc + j] = sys.p[loc + j].x;
        sys.projy[loc + j] = sys.p[loc + j].y;
        sys.projz[loc + j] = sys.p[loc + j].z;
    }
}

__global__ void cudaPDProjectEdgeConstraint( CudaPDSystem sys )
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= sys.nb_edges)
        return;
    int id0 = sys.edge_consts[i].id0;
    int id1 = sys.edge_consts[i].id1;
    int loc = sys.edge_consts[i].loc;
    Real w = sys.edge_consts[i].weight;
    Real rest = sys.edge_consts[i].restlen;

    vec3 edge = sys.q[id1] - sys.q[id0];
    edge /= norm( edge );
    edge *= w;

    sys.projx[loc] = edge.x;
    sys.projy[loc] = edge.y;
    sys.projz[loc] = edge.z;
    //sys.p[loc] = edge;
}

__global__ void cudaPDChebyshev( CudaPDSystem sys, Real* v, int r, Real omega )
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= sys.nb_points)
        return;
    Real last = ((Real*)&sys.qlast1[i])[r];
    Real last1 = ((Real*)&sys.qlast2[i])[r];
    v[i] = omega * (0.9f * (v[i] - last) + last - last1) + last1;
    if (r == 0)
    {
        sys.qlast2[i].x = sys.qlast1[i].x;
        sys.qlast1[i].x = sys.qx[i];
        sys.qx[i] = v[i];
        sys.q[i].x = v[i];
    }
    else if (r == 1)
    {
        sys.qlast2[i].y = sys.qlast1[i].y;
        sys.qlast1[i].y = sys.qy[i];
        sys.qy[i] = v[i];
        sys.q[i].y = v[i];
    }
    else
    {
        sys.qlast2[i].z = sys.qlast1[i].z;
        sys.qlast1[i].z = sys.qz[i];
        sys.qz[i] = v[i];
        sys.q[i].z = v[i];
    }
}

__global__ void cudaPDUpdateVel( CudaPDSystem sys )
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= sys.nb_points)
        return;
    sys.q[i] = make_vec3( sys.qx[i], sys.qy[i], sys.qz[i] );
    sys.v[i] = (1.0 / sys.dt) * (sys.q[i] - sys.qlast[i]);
}

__global__ void cudaPDUpdateSkinningInfo( CudaPDSystem sys )
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= sys.nb_points)
        return;
    sys._skin_info[i].u = make_float4( sys.qx[i], sys.qy[i], sys.qz[i], 1.0f );
    sys._skin_info[i].xi = make_float4( sys.q0[i].x - sys.displacement.x, sys.q0[i].y - sys.displacement.y, sys.q0[i].z - sys.displacement.z, 1.0f );

    sys._skin_info[i].R[0] = make_float4( sys.metaball_consts[i].F[0].x, sys.metaball_consts[i].F[0].y, sys.metaball_consts[i].F[0].z, 0.0f );
    sys._skin_info[i].R[1] = make_float4( sys.metaball_consts[i].F[1].x, sys.metaball_consts[i].F[1].y, sys.metaball_consts[i].F[1].z, 0.0f );
    sys._skin_info[i].R[2] = make_float4( sys.metaball_consts[i].F[2].x, sys.metaball_consts[i].F[2].y, sys.metaball_consts[i].F[2].z, 0.0f );
    sys._skin_info[i].R[3] = make_float4( 0.f, 0.f, 0.f, 1.0f );
}