#include "cuda_runtime.h"
#include "device_launch_parameters.h";
#include "util/CudaBuffer.h"
#include "PBDConstraints.h"

struct CudaPBDSystem
{
    float dt;
    int nb_it;
    float damp;

    int nb_points;
    float3* x0;
    float3* x;
    float3* dx;
    float3* p;
    float3* v;
    float* m;
    float* invm;
    int* n;

    int nb_stretch;
    int nb_attach;
    int nb_volume;
    PBD::EdgeConstraint* c_stretch;
    PBD::AttachmentConstraint* c_attach;
    PBD::VolumeConservConstraint* c_volume;
    float k_stretch;
    float k_volume;
};

struct PBDSystem
{
    float dt;
    int nb_it;
    float damp;

    int nb_points;
    CudaBuffer<float> x0;
    CudaBuffer<float> x;
    CudaBuffer<float> p;
    CudaBuffer<float> v;
    CudaBuffer<float> m;
    CudaBuffer<float> invm;
    CudaBuffer<float> dx;
    CudaBuffer<int>   n;

    int nb_stretch;
    int nb_attach;
    int nb_volume;
    CudaBuffer<PBD::EdgeConstraint> c_stretch;
    CudaBuffer<PBD::AttachmentConstraint> c_attach;
    CudaBuffer<PBD::VolumeConservConstraint> c_volume;
    float k_stretch;
    float k_volume;
};


__global__ void cudaPBDPredStep( CudaPBDSystem sys );
__global__ void cudaPBDStretchConstraintStep( CudaPBDSystem sys );
__global__ void cudaPBDAttachConstraintStep( CudaPBDSystem sys );
__global__ void cudaPBDVolumeConstraintStep( CudaPBDSystem sys );
__global__ void cudaPBDPostStep( CudaPBDSystem sys );
__global__ void cudaPBDUpdateVel( CudaPBDSystem sys );
__global__ void cudaRayIntersect( float3* x, int n, int* id, const float3& o, const float3& d, float r );

CudaPBDSystem CreateCudaPBDSystem( PBDSystem& sys );
void PBDPredStep( dim3 blocksize, dim3 gridsize, PBDSystem& sys );
void PBDStretchConstraintStep( dim3 blocksize, dim3 gridsize, PBDSystem& sys );
void PBDAttachConstraintStep( dim3 blocksize, dim3 gridsize, PBDSystem& sys );
void PBDVolumeConstraintStep( dim3 blocksize, dim3 gridsize, PBDSystem& sys );
void PBDPostStep( dim3 blocksize, dim3 gridsize, PBDSystem& sys );
void PBDUpdateVel( dim3 blocksize, dim3 gridsize, PBDSystem& sys );