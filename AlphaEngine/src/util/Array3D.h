#pragma once
#include <vector>

template <typename T>
class Array3D
{
public:
    Array3D();
    Array3D( int x, int y, int z );
    Array3D( int x, int y, int z, const T& v );
    Array3D( const Array3D<T>& rh );
    Array3D( Array3D<T>&& rh );
    Array3D<T>& operator=( const Array3D<T>& rh );
    Array3D<T>& operator=( Array3D<T>&& rh );
    T& operator()( int x, int y, int z );
    const T& operator()( int x, int y, int z ) const;
    const T* Data() const;
    void Clear();
    int DimX() const;
    int DimY() const;
    int DimZ() const;
private:
    std::vector<T> _data;
    int _dimx = 0;
    int _dimy = 0;
    int _dimz = 0;
};

template<typename T>
inline Array3D<T>::Array3D()
{

}

template<typename T>
inline Array3D<T>::Array3D( int x, int y, int z )
    :_dimx( x ), _dimy( y ), _dimz( z )
{
    _data = std::vector<T>( _dimx * _dimy * _dimz );
}

template <typename T>
inline Array3D<T>::Array3D( int x, int y, int z, const T& v )
    :_dimx( x ), _dimy( y ), _dimz( z )
{
    _data = std::vector<T>( _dimx * _dimy * _dimz, v );
}

template<typename T>
inline Array3D<T>::Array3D( const Array3D<T>& rh )
{
    _dimx = rh._dimx;
    _dimy = rh._dimy;
    _dimz = rh._dimz;
    _data = rh._data;
}

template<typename T>
inline Array3D<T>::Array3D( Array3D<T>&& rh )
{
    _dimx = rh._dimx;
    _dimy = rh._dimy;
    _dimz = rh._dimz;
    _data = std::move( rh._data );
}

template<typename T>
inline Array3D<T>& Array3D<T>::operator=( const Array3D<T>& rh )
{
    _dimx = rh._dimx;
    _dimy = rh._dimy;
    _dimz = rh._dimz;
    _data = rh._data;
    return *this;
}

template<typename T>
inline Array3D<T>& Array3D<T>::operator=( Array3D<T>&& rh )
{
    _dimx = rh._dimx;
    _dimy = rh._dimy;
    _dimz = rh._dimz;
    _data = std::move( rh._data );
    return *this;
}

template<typename T>
inline T& Array3D<T>::operator()( int x, int y, int z )
{
    return _data[x * _dimy * _dimz + y * _dimz + z];
}

template<typename T>
inline const T& Array3D<T>::operator()( int x, int y, int z ) const
{
    return _data[x * _dimy * _dimz + y * _dimz + z];
}

template<typename T>
inline const T* Array3D<T>::Data() const
{
    return _data.data();
}

template<typename T>
inline void Array3D<T>::Clear()
{
    _dimx = 0;
    _dimy = 0;
    _dimz = 0;
    _data.clear();
}

template<typename T>
inline int Array3D<T>::DimX() const
{
    return _dimx;
}

template<typename T>
inline int Array3D<T>::DimY() const
{
    return _dimy;
}

template<typename T>
inline int Array3D<T>::DimZ() const
{
    return _dimz;
}
