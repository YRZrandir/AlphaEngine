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
class SparseBSR
{
public:



};

class SparseCSR
{

};

cusparseSpMatDescr_t CreateCUDASparseMatrix( const Eigen::SparseMatrix<float>& m );
cusparseDnVecDescr_t CreateCUDAVector( const CudaBuffer<float>& vec );
void SetCUDAVector( cusparseDnVecDescr_t cu_vec, const CudaBuffer<float>& mem );
void ReadCUDAVector( cusparseDnVecDescr_t cu_vec, CudaBuffer<float>& mem );
size_t CUDASpmvBufferSize( cusparseSpMatDescr_t M, cusparseDnVecDescr_t x, cusparseDnVecDescr_t y );
void CUDASpmv( cusparseSpMatDescr_t M, cusparseDnVecDescr_t x, cusparseDnVecDescr_t y, void* buffer = nullptr );
void CUDAvplusv( CudaBuffer<float>& a, float sa, CudaBuffer<float>& b, float sb, CudaBuffer<float>& dst, dim3 gridsize, dim3 blocksize );