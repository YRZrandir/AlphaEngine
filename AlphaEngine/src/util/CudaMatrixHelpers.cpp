#include "CudaMatrixHelpers.h"
#include <iostream>

cusparseHandle_t sCuSparseContext = nullptr;

void CheckCusparseInit()
{
    if (sCuSparseContext == nullptr)
    {
        cusparseCreate( &sCuSparseContext );
    }
}


