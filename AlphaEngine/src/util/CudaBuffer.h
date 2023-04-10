#pragma once
#include <iostream>
#include <vector>
#include "cuda_runtime.h"

template<typename T>
class CudaBuffer
{
public:
    CudaBuffer();
    CudaBuffer( size_t count );
    CudaBuffer( size_t count, const T* data );
    ~CudaBuffer();
    void UpdateBuffer( size_t count, const T* data );
    void ReadBuffer( T* out ) const;
    size_t Count() const;
    size_t MemSize() const;
    T* const& Data() const;
    T*& Data();

private:
    T* _data{ nullptr };
    size_t _count{ 0 };
};

template <typename T>
CudaBuffer<T>::CudaBuffer()
{
    cudaMalloc( reinterpret_cast<void**>(&_data), MemSize() );
}

template <typename T>
CudaBuffer<T>::CudaBuffer( size_t count )
    :_count( count )
{
    cudaMalloc( reinterpret_cast<void**>(&_data), MemSize() );
}

template<typename T>
CudaBuffer<T>::CudaBuffer( size_t count, const T* data )
    :_count( count )
{
    cudaMalloc( (void**)(&_data), MemSize() );
    cudaMemcpy( static_cast<void*>(_data), static_cast<const void*>(data), MemSize(), cudaMemcpyHostToDevice );
}

template <typename T>
CudaBuffer<T>::~CudaBuffer()
{
    if (_data)
        cudaFree( _data );
}

template<typename T>
void CudaBuffer<T>::UpdateBuffer( size_t count, const T* data )
{
    if (count > _count)
    {
        _count = count;
        cudaFree( static_cast<void*>(_data) );
        cudaMalloc( (void**)(&_data), MemSize() );
    }
    cudaMemcpy( static_cast<void*>(_data), static_cast<const void*>(data), MemSize(), cudaMemcpyHostToDevice );
}

template<typename T>
void CudaBuffer<T>::ReadBuffer( T* out ) const
{
    cudaMemcpy( static_cast<void*>(out), static_cast<const void*>(_data), MemSize(), cudaMemcpyDeviceToHost );
}

template<typename T>
size_t CudaBuffer<T>::Count() const
{
    return _count;
}

template<typename T>
size_t CudaBuffer<T>::MemSize() const
{
    return _count * sizeof( T );
}

template <typename T>
T* const& CudaBuffer<T>::Data() const
{
    return _data;
}

template <typename T>
T*& CudaBuffer<T>::Data()
{
    return _data;
}