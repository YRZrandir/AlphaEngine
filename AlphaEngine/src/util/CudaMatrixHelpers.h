#include <concepts>
#include "cuda_runtime.h"
#include "device_launch_parameters.h";
#include "util/CudaBuffer.h"
#include "cusparse.h"
#include <Eigen/Eigen>
#include <Eigen/Sparse>

/*
cusparseXcsr2bsrNnz(cusparseHandle_t         handle,
                    cusparseDirection_t      dirA,
                    int                      m,
                    int                      n,
                    const cusparseMatDescr_t descrA,
                    const int*               csrSortedRowPtrA,
                    const int*               csrSortedColIndA,
                    int                      blockDim,
                    const cusparseMatDescr_t descrC,
                    int*                     bsrSortedRowPtrC,
                    int*                     nnzTotalDevHostPtr);
*/

extern cusparseHandle_t sCuSparseContext;
void CheckCusparseInit();

template <std::floating_point T>
cusparseSpMatDescr_t CreateCUDASparseMatrix( const Eigen::SparseMatrix<T>& m )
{
    constexpr cudaDataType_t datatype = std::conditional_t<
        std::is_same_v<T, float>,
        std::integral_constant<cudaDataType_t, CUDA_R_32F>,
        std::integral_constant<cudaDataType_t, CUDA_R_64F>>::value;

    CheckCusparseInit();
    if (!m.isCompressed())
    {
        std::cout << "ERROR: m has to be compressed." << std::endl;
        return nullptr;
    }

    int* dA_cscOffsets, * dA_rows;
    T* dA_values;

    int A_num_rows = m.rows();
    int A_num_cols = m.cols();
    int A_nnz = m.nonZeros();
    cudaMalloc( (void**)&dA_cscOffsets, (A_num_cols + 1) * sizeof( int ) );
    cudaMalloc( (void**)&dA_rows, A_nnz * sizeof( int ) );
    cudaMalloc( (void**)&dA_values, A_nnz * sizeof( T ) );

    cudaMemcpy( dA_cscOffsets, m.outerIndexPtr(), (A_num_cols + 1) * sizeof( int ), cudaMemcpyHostToDevice );
    cudaMemcpy( dA_rows, m.innerIndexPtr(), A_nnz * sizeof( int ), cudaMemcpyHostToDevice );
    cudaMemcpy( dA_values, m.valuePtr(), A_nnz * sizeof( T ), cudaMemcpyHostToDevice );
    cusparseSpMatDescr_t sparse_mat;
    cusparseStatus_t status = cusparseCreateCsc( &sparse_mat, A_num_rows, A_num_cols, A_nnz, dA_cscOffsets, dA_rows, dA_values,
        CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I, CUSPARSE_INDEX_BASE_ZERO, datatype );
    if (status != CUSPARSE_STATUS_SUCCESS)
    {
        std::cout << "ERROR: cusparseCreateCsc Failed." << std::endl;
    }
    return sparse_mat;
}

template <std::floating_point T>
cusparseDnVecDescr_t CreateCUDAVector( const CudaBuffer<T>& vec )
{
    constexpr cudaDataType_t datatype = std::conditional_t<
        std::is_same_v<T, float>,
        std::integral_constant<cudaDataType_t, CUDA_R_32F>,
        std::integral_constant<cudaDataType_t, CUDA_R_64F>>::value;

    CheckCusparseInit();
    cusparseDnVecDescr_t cu_vec;
    cusparseCreateDnVec( &cu_vec, vec.Count(), (void*)vec.Data(), datatype );
    return cu_vec;
}

template <std::floating_point T>
void SetCUDAVector( cusparseDnVecDescr_t cu_vec, const CudaBuffer<T>& mem )
{
    CheckCusparseInit();
    cusparseDnVecSetValues( cu_vec, (void*)mem.Data() );
}

template <std::floating_point T>
void ReadCUDAVector( cusparseDnVecDescr_t cu_vec, CudaBuffer<T>& mem )
{
    CheckCusparseInit();
    cusparseDnVecGetValues( cu_vec, (void**)&mem.Data() );
}

template <std::floating_point T>
size_t CUDASpmvBufferSize( cusparseSpMatDescr_t M, cusparseDnVecDescr_t x, cusparseDnVecDescr_t y )
{
    constexpr cudaDataType_t datatype = std::conditional_t<
        std::is_same_v<T, float>,
        std::integral_constant<cudaDataType_t, CUDA_R_32F>,
        std::integral_constant<cudaDataType_t, CUDA_R_64F>>::value;

    T alpha = 1.0;
    T beta = 0.0;
    size_t buffer_size = 0;
    cusparseSpMV_bufferSize( sCuSparseContext, CUSPARSE_OPERATION_NON_TRANSPOSE,
        &alpha, M, x, &beta, y, datatype,
        CUSPARSE_MV_ALG_DEFAULT, &buffer_size );
    return buffer_size;
}

template <std::floating_point T>
void CUDASpmv( cusparseSpMatDescr_t M, cusparseDnVecDescr_t x, cusparseDnVecDescr_t y, void* buffer = nullptr )
{
    constexpr cudaDataType_t datatype = std::conditional_t<
        std::is_same_v<T, float>,
        std::integral_constant<cudaDataType_t, CUDA_R_32F>,
        std::integral_constant<cudaDataType_t, CUDA_R_64F>>::value;

    T alpha = 1.0f;
    T beta = 0.0f;
    bool need_free = false;
    if (buffer == nullptr)
    {
        need_free = true;
        size_t buffer_size = 0;
        cusparseSpMV_bufferSize( sCuSparseContext, CUSPARSE_OPERATION_NON_TRANSPOSE,
            &alpha, M, x, &beta, y, datatype,
            CUSPARSE_MV_ALG_DEFAULT, &buffer_size );
        cudaMalloc( &buffer, buffer_size );
    }
    cusparseSpMV( sCuSparseContext, CUSPARSE_OPERATION_NON_TRANSPOSE,
        &alpha, M, x, &beta, y, datatype,
        CUSPARSE_MV_ALG_DEFAULT, buffer );
    if (need_free)
    {
        cudaFree( buffer );
    }
}


