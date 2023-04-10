#include "CudaPbdSolver.cuh"
#include "util/CudaVecHelper.cu"

CudaPBDSystem CreateCudaPBDSystem( PBDSystem& sys )
{
    CudaPBDSystem cudasys;
    cudasys.dt = sys.dt;
    cudasys.damp = sys.damp;
    cudasys.nb_it = sys.nb_it;
    cudasys.nb_points = sys.nb_points;
    cudasys.x0 = (float3*)sys.x0.Data();
    cudasys.x = (float3*)sys.x.Data();
    cudasys.dx = (float3*)sys.dx.Data();
    cudasys.n = (int*)sys.n.Data();
    cudasys.p = (float3*)sys.p.Data();
    cudasys.v = (float3*)sys.v.Data();
    cudasys.m = sys.m.Data();
    cudasys.invm = sys.invm.Data();
    cudasys.c_attach = sys.c_attach.Data();
    cudasys.c_stretch = sys.c_stretch.Data();
    cudasys.c_volume = sys.c_volume.Data();
    cudasys.nb_attach = sys.nb_attach;
    cudasys.nb_stretch = sys.nb_stretch;
    cudasys.nb_volume = sys.nb_volume;
    cudasys.k_stretch = sys.k_stretch;
    cudasys.k_volume = sys.k_volume;
    return cudasys;
}

void PBDPredStep( dim3 blocksize, dim3 gridsize, PBDSystem& sys )
{
    CudaPBDSystem cudasys = CreateCudaPBDSystem( sys );
    cudaPBDPredStep << <gridsize, blocksize >> > (cudasys);
}

void PBDStretchConstraintStep( dim3 blocksize, dim3 gridsize, PBDSystem& sys )
{
    CudaPBDSystem cudasys = CreateCudaPBDSystem( sys );
    cudaPBDStretchConstraintStep << <gridsize, blocksize >> > (cudasys);
}

void PBDAttachConstraintStep( dim3 blocksize, dim3 gridsize, PBDSystem& sys )
{
    CudaPBDSystem cudasys = CreateCudaPBDSystem( sys );
    cudaPBDAttachConstraintStep << <gridsize, blocksize >> > (cudasys);
}

void PBDVolumeConstraintStep( dim3 blocksize, dim3 gridsize, PBDSystem& sys )
{
    CudaPBDSystem cudasys = CreateCudaPBDSystem( sys );
    cudaPBDVolumeConstraintStep << < gridsize, blocksize >> > (cudasys);
}

void PBDPostStep( dim3 blocksize, dim3 gridsize, PBDSystem& sys )
{
    CudaPBDSystem cudasys = CreateCudaPBDSystem( sys );
    cudaPBDPostStep << <gridsize, blocksize >> > (cudasys);
}

void PBDUpdateVel( dim3 blocksize, dim3 gridsize, PBDSystem& sys )
{
    CudaPBDSystem cudasys = CreateCudaPBDSystem( sys );
    cudaPBDUpdateVel << <gridsize, blocksize >> > (cudasys);
}

__global__ void cudaPBDPredStep( CudaPBDSystem sys )
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= sys.nb_points)
        return;
    //Prediction
    float3 g = make_float3( 0.f, -9.8f, 0.f );
    sys.v[i] += sys.dt * g;
    sys.p[i] = sys.x[i] + sys.dt * sys.v[i];
    sys.dx[i] = make_float3( 0, 0, 0 );
    sys.n[i] = 0;
}

__global__ void cudaPBDStretchConstraintStep( CudaPBDSystem sys )
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= sys.nb_stretch)
        return;
    int i0 = sys.c_stretch[i]._i0;
    int i1 = sys.c_stretch[i]._i1;
    float k = sys.c_stretch[i]._k;
    k = sys.k_stretch;
    float d = sys.c_stretch[i]._d;
    float3 diff = sys.p[i0] - sys.p[i1];
    float dist = norm( diff );
    float3 n = diff / dist;
    if (dist < 0.00001f)
    {
        n = make_float3( 0.f, 0.f, 0.f );
    }
    float s = (dist - d) / (sys.invm[i0] + sys.invm[i1]);
    //s = s * (1.f - powf( 1.f - k, sys.nb_it ));
    float3 dx0 = -sys.invm[i0] * s * k * n;
    float3 dx1 = sys.invm[i1] * s * k * n;

    atomicAdd( &sys.dx[i0].x, dx0.x );
    atomicAdd( &sys.dx[i0].y, dx0.y );
    atomicAdd( &sys.dx[i0].z, dx0.z );
    atomicAdd( &sys.dx[i1].x, dx1.x );
    atomicAdd( &sys.dx[i1].y, dx1.y );
    atomicAdd( &sys.dx[i1].z, dx1.z );
    atomicAdd( &sys.n[i0], 1 );
    atomicAdd( &sys.n[i1], 1 );
}

__global__ void cudaPBDAttachConstraintStep( CudaPBDSystem sys )
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= sys.nb_attach)
        return;

    PBD::AttachmentConstraint C = sys.c_attach[i];
    int id = C._index;

    float3 x0 = make_float3( C._p.x, C._p.y, C._p.z );
    sys.dx[id] += x0 - sys.p[id];
    sys.n[id] += 1;
}

__global__ void cudaPBDVolumeConstraintStep( CudaPBDSystem sys )
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= sys.nb_volume)
        return;

    int i0 = sys.c_volume[i]._indices[0];
    int i1 = sys.c_volume[i]._indices[1];
    int i2 = sys.c_volume[i]._indices[2];
    int i3 = sys.c_volume[i]._indices[3];
    const float3& p0 = sys.p[i0];
    const float3& p1 = sys.p[i1];
    const float3& p2 = sys.p[i2];
    const float3& p3 = sys.p[i3];

    float c_value = abs( dot( cross( p1 - p0, p2 - p0 ), p3 - p0 ) ) / 6.f - sys.c_volume[i]._V;

    float3 grad0 = cross( p1 - p2, p3 - p2 );
    float3 grad1 = cross( p2 - p0, p3 - p0 );
    float3 grad2 = cross( p0 - p1, p3 - p1 );
    float3 grad3 = cross( p1 - p0, p2 - p0 );

    float denorm = norm2( grad0 ) * sys.invm[i0]
        + norm2( grad1 ) * sys.invm[i1]
        + norm2( grad2 ) * sys.invm[i2]
        + norm2( grad3 ) * sys.invm[i3];

    float lambda = sys.k_volume * c_value / denorm;

    float3 dp0 = -lambda * sys.invm[i0] * grad0;
    float3 dp1 = -lambda * sys.invm[i1] * grad1;
    float3 dp2 = -lambda * sys.invm[i2] * grad2;
    float3 dp3 = -lambda * sys.invm[i3] * grad3;

    atomicFlt3Add( &(sys.dx[i0]), dp0 );
    atomicFlt3Add( &(sys.dx[i1]), dp1 );
    atomicFlt3Add( &(sys.dx[i2]), dp2 );
    atomicFlt3Add( &(sys.dx[i3]), dp3 );
    atomicAdd( &sys.n[i0], 1 );
    atomicAdd( &sys.n[i1], 1 );
    atomicAdd( &sys.n[i2], 1 );
    atomicAdd( &sys.n[i3], 1 );
}

__global__ void cudaPBDPostStep( CudaPBDSystem sys )
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= sys.nb_points)
        return;

    if (sys.n[i] != 0)
        sys.p[i] += sys.dx[i] * (1.2f / (float)sys.n[i]);
    sys.dx[i] = make_float3( 0.f, 0.f, 0.f );
    sys.n[i] = 0;
}

__global__ void cudaPBDUpdateVel( CudaPBDSystem sys )
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= sys.nb_points)
        return;

    sys.v[i] = (0.99f / sys.dt) * (sys.p[i] - sys.x[i]);
    sys.x[i] = sys.p[i];
}

__global__ void cudaRayIntersect( float3* x, int n, int* id, const float3& o, const float3& d, float r )
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n)
        return;

}