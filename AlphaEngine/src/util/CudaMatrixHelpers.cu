#include "CudaMatrixHelpers.cuh"
#include <iostream>

namespace
{
cusparseHandle_t sCuSparseContext = nullptr;
void CheckCusparseInit()
{
    if (sCuSparseContext == nullptr)
    {
        cusparseCreate( &sCuSparseContext );
    }
}

}
__global__ void CUDAvplusv_impl( float* a, float sa, float* b, float sb, float* dst, int size );

cusparseSpMatDescr_t CreateCUDASparseMatrix( const Eigen::SparseMatrix<float>& m )
{
    CheckCusparseInit();
    if (!m.isCompressed())
    {
        std::cout << "ERROR: m has to be compressed." << std::endl;
        return nullptr;
    }

    int* dA_cscOffsets, * dA_rows;
    float* dA_values;

    int A_num_rows = m.rows();
    int A_num_cols = m.cols();
    int A_nnz = m.nonZeros();
    cudaMalloc( (void**)&dA_cscOffsets, (A_num_cols + 1) * sizeof( int ) );
    cudaMalloc( (void**)&dA_rows, A_nnz * sizeof( int ) );
    cudaMalloc( (void**)&dA_values, A_nnz * sizeof( float ) );

    cudaMemcpy( dA_cscOffsets, m.outerIndexPtr(), (A_num_cols + 1) * sizeof( int ), cudaMemcpyHostToDevice );
    cudaMemcpy( dA_rows, m.innerIndexPtr(), A_nnz * sizeof( int ), cudaMemcpyHostToDevice );
    cudaMemcpy( dA_values, m.valuePtr(), A_nnz * sizeof( float ), cudaMemcpyHostToDevice );

    cusparseSpMatDescr_t sparse_mat;
    cusparseStatus_t status = cusparseCreateCsc( &sparse_mat, A_num_rows, A_num_cols, A_nnz, dA_cscOffsets, dA_rows, dA_values,
        CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I, CUSPARSE_INDEX_BASE_ZERO, CUDA_R_32F );
    if (status != CUSPARSE_STATUS_SUCCESS)
    {
        std::cout << "ERROR: cusparseCreateCsc Failed." << std::endl;
    }
    return sparse_mat;
}

cusparseDnVecDescr_t CreateCUDAVector( const CudaBuffer<float>& vec )
{
    CheckCusparseInit();
    cusparseDnVecDescr_t cu_vec;
    cusparseCreateDnVec( &cu_vec, vec.Count(), (void*)vec.Data(), CUDA_R_32F );
    return cu_vec;
}

void SetCUDAVector( cusparseDnVecDescr_t cu_vec, const CudaBuffer<float>& mem )
{
    CheckCusparseInit();
    cusparseDnVecSetValues( cu_vec, (void*)mem.Data() );
}

void ReadCUDAVector( cusparseDnVecDescr_t cu_vec, CudaBuffer<float>& mem )
{
    CheckCusparseInit();
    cusparseDnVecGetValues( cu_vec, (void**)&mem.Data() );
}

size_t CUDASpmvBufferSize( cusparseSpMatDescr_t M, cusparseDnVecDescr_t x, cusparseDnVecDescr_t y )
{
    float alpha = 1.0f;
    float beta = 0.0f;
    size_t buffer_size = 0;
    cusparseSpMV_bufferSize( sCuSparseContext, CUSPARSE_OPERATION_NON_TRANSPOSE,
        &alpha, M, x, &beta, y, CUDA_R_32F,
        CUSPARSE_MV_ALG_DEFAULT, &buffer_size );
    return buffer_size;
}

void CUDASpmv( cusparseSpMatDescr_t M, cusparseDnVecDescr_t x, cusparseDnVecDescr_t y, void* buffer )
{
    float alpha = 1.0f;
    float beta = 0.0f;
    bool need_free = false;
    if (buffer == nullptr)
    {
        need_free = true;
        size_t buffer_size = 0;
        cusparseSpMV_bufferSize( sCuSparseContext, CUSPARSE_OPERATION_NON_TRANSPOSE,
            &alpha, M, x, &beta, y, CUDA_R_32F,
            CUSPARSE_MV_ALG_DEFAULT, &buffer_size );
        cudaMalloc( &buffer, buffer_size );
    }
    cusparseSpMV( sCuSparseContext, CUSPARSE_OPERATION_NON_TRANSPOSE,
        &alpha, M, x, &beta, y, CUDA_R_32F,
        CUSPARSE_MV_ALG_DEFAULT, buffer );
    if (need_free)
    {
        cudaFree( buffer );
    }
}

void CUDAvplusv( CudaBuffer<float>& a, float sa, CudaBuffer<float>& b, float sb, CudaBuffer<float>& dst, dim3 gridsize, dim3 blocksize )
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
