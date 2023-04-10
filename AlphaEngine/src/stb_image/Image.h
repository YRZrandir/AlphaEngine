#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <concepts>

template <typename T>
concept UnsignedIntegralExptBool = std::unsigned_integral<T> && !std::is_same_v<T, bool>;

template <typename T>
concept Numerical = UnsignedIntegralExptBool<T> || std::floating_point<T>;

template <Numerical T>
class Image
{
public:
    Image() = default;
    Image( unsigned width, unsigned height, unsigned n_channel )
        : _width( width ), _height( height ), _n_channel( n_channel )
    {
        _data.resize( _width * _height * _n_channel );
    }

    unsigned Width() const noexcept { return _width; }
    unsigned Height() const noexcept { return _height; }
    unsigned NChannel() const noexcept { return _n_channel; }
    /*
        if !discard, the original image will be clipped to target size.
        else, just resize the memory and the data is undefined.
    */
    void Resize( unsigned width, unsigned height, bool discard ) noexcept
    {
        if (!discard)
        {
            std::vector<T> data_new( width * height * _n_channel );
            unsigned width_cpy = std::min( _width, width );
            unsigned height_cpy = std::min( _height, height );
            for (unsigned y = 0; y < height_cpy; ++y)
            {
                for (unsigned x = 0; x < width_cpy; ++x)
                {
                    for (unsigned c = 0; c < _n_channel; ++c)
                    {
                        data_new[y * width_cpy * _n_channel + x * _n_channel + c] = _data[y * width_cpy * _n_channel + x * _n_channel + c];
                    }
                }
            }
            _data = std::move( data_new );
        }
        else
        {
            _data.resize( width * height * _n_channel );
        }

        _width = width;
        _height = height;
    }

    const T& Get( unsigned x, unsigned y, unsigned c ) const noexcept
    {
        return _data[y * _width * _n_channel + x * _n_channel + c];
    }

    T& Get( unsigned x, unsigned y, unsigned c ) noexcept
    {
        return _data[y * _width * _n_channel + x * _n_channel + c];
    }

    T* Data() noexcept { return _data.data(); }

    const T* Data() const noexcept { return _data.data(); }

    void SetData( size_t offset, size_t count, T* data )
    {
        std::copy( data, data + count, _data.data() + offset );
    }

protected:
    unsigned _n_channel{ 0 };
    unsigned _width{ 0 };
    unsigned _height{ 0 };
    std::vector<T> _data;
};


Image<unsigned char> LoadImageFile( const std::string& path );


template <UnsignedIntegralExptBool UnsignedType, std::floating_point FloatType>
Image<UnsignedType> Imagef2u( const Image<FloatType>& img )
{
    Image<UnsignedType> img_new( img.Width(), img.Height(), img.NChannel() );
    for (unsigned x = 0; x < img.Width(); ++x)
    {
        for (unsigned y = 0; y < img.Height(); ++y)
        {
            for (unsigned c = 0; c < img.NChannel(); ++c)
            {
                img_new.Get( x, y, c ) = static_cast<UnsignedType>(img.Get( x, y, c ) * 255);
            }
        }
    }

    return img_new;
}


template <std::floating_point FloatType, UnsignedIntegralExptBool UnsignedType>
Image<float> Imageu2f( const Image<UnsignedType>& img )
{
    Image<FloatType> img_new( img.Width(), img.Height(), img.NChannel() );
    for (unsigned x = 0; x < img.Width(); ++x)
    {
        for (unsigned y = 0; y < img.Height(); ++y)
        {
            for (unsigned c = 0; c < img.NChannel(); ++c)
            {
                img_new.Get( x, y, c ) = static_cast<FloatType>(img.Get( x, y, c )) / 255.0;
            }
        }
    }
}