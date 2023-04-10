#pragma once
#include "cuda_runtime.h"
#include "device_launch_parameters.h";


__device__ float3 operator+( const float3& a, const float3& b );

__device__ float3 operator-( const float3& a, const float3& b );

__device__ float3& operator+= ( float3& a, const float3& b );

__device__ float3& operator-= ( float3& a, const float3& b );

__device__ float3 operator*( const float3& a, float s );

__device__ float3 operator*( float s, const float3& a );

__device__ float3 operator/( const float3& a, float d );

__device__ float3& operator/=( float3& a, float s );

__device__ float3& operator*=( float3& a, float s );

__device__ float norm( const float3& a );

__device__ float norm2( const float3& a );

__device__ float3 cross( const float3& a, const float3& b );

__device__ float dot( const float3& a, const float3& b );

__device__ void atomicFlt3Add( float3* dst, const float3& value );

//col major
__device__ float3 MulMv3x3( float3* m, const float3& v );

__device__ void TransposeInplace3x3( float3* m );

__device__ __forceinline__ float4 make_float4( float3 v3, float w );

#define qeal float

__device__ __forceinline__
qeal accurateSqrt( qeal x );

__device__ __forceinline__
void condSwap( bool c, qeal* X, qeal* Y );

__device__ __forceinline__
void condNegSwap( bool c, qeal* X, qeal* Y );

__device__ __forceinline__
void multAB( qeal* A, qeal* B, qeal* C );

__device__ __forceinline__
void multAtB( qeal* A, qeal* B, qeal* C );

__device__ __forceinline__
void quatToMat( qeal* mat, const qeal* qV );

__device__ __forceinline__
void approximateGivensQuaternion( qeal a11, qeal a12, qeal a22, qeal* ch, qeal* sh );

__device__ __forceinline__
void jacobiConjugation( const unsigned int x, const unsigned int y, const unsigned int z,
    qeal* s11,
    qeal* s21, qeal* s22,
    qeal* s31, qeal* s32, qeal* s33,
    qeal* qV );

__device__ __forceinline__
qeal dist2( qeal x, qeal y, qeal z );

// finds transformation that diagonalizes a symmetric matrix
__device__ __forceinline__
void jacobiEigenanlysis( // symmetric matrix
    qeal* s11,
    qeal* s21, qeal* s22,
    qeal* s31, qeal* s32, qeal* s33,
    // quaternion representation of V
    qeal* qV );

__device__ __forceinline__
void sortSingularValues(// matrix that we want to decompose
    qeal* A,
    // sort V simultaneously
    qeal* v );

__device__ __forceinline__
void QRGivensQuaternion( qeal a1, qeal a2, qeal* ch, qeal* sh );

__device__ __forceinline__
void QRDecomposition(// matrix that we want to decompose
    qeal* A, qeal* Q, qeal* R );

__device__ __forceinline__
void svd( qeal* A, qeal* U, qeal* S, qeal* V );

/// polar decomposition can be reconstructed trivially from SVD result
/// A = UP
__device__ __forceinline__
void pd( qeal* A,
    // output U
    qeal* U,
    // output P
    qeal* P );

__device__ __forceinline__
qeal dotVV( qeal* v1, qeal* v2 );


__device__ __forceinline__
qeal getMatrixDeterminant( qeal* mat );