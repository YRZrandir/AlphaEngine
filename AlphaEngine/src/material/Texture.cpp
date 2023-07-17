#include "Texture.h"
#include <memory>
#include <iostream>
#include "../stb_image/stb_image.h"

#include "../util/Shader.h"

Texture::Texture( const std::string& path, Types type )
    :_path( path ), _type( type )
{
    glCreateTextures( GL_TEXTURE_2D, 1, &_id );
    Bind();
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

    try
    {
        _img = LoadImageFile( path );
    }
    catch (std::exception)
    {
        _img = LoadImageFile( "res/img/default.jpg" );
    }

    UpdateData();
}

Texture::Texture( int width, int height, int channel_num, Types type )
    :_type( type ), _width( width ), _height( height ), _nb_channel( channel_num )
{
    _img = Image<unsigned char>( width, height, channel_num );
    glCreateTextures( GL_TEXTURE_2D, 1, &_id );
    Bind();
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

    UpdateData();
}

Texture::Texture( int width, int height, int channel_num, Types type, unsigned char* data )
    :_type( type ), _width( width ), _height( height ), _nb_channel( channel_num )
{
    _img = Image<unsigned char>( width, height, channel_num );
    _img.SetData( 0, width * height * channel_num, data );
    glCreateTextures( GL_TEXTURE_2D, 1, &_id );
    Bind();
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

    UpdateData();
}

Texture::Texture( int width, int height, int channel_num, Types type, float* data )
    :_type( type ), _width( width ), _height( height ), _nb_channel( channel_num )
{
    auto tmp = Image<float>( width, height, channel_num );
    tmp.SetData( 0, width * height * channel_num, data );
    _img = Imagef2u<unsigned char>( tmp );
    glCreateTextures( GL_TEXTURE_2D, 1, &_id );
    Bind();
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

    UpdateData();
}

Texture::Texture()
{
}

Texture::~Texture()
{
    glDeleteTextures( 1, &_id );
}

GLuint Texture::GetID() const noexcept
{
    return _id;
}

std::string Texture::GetPath() const noexcept
{
    return _path;
}

Texture::Types Texture::GetType() const noexcept
{
    return _type;
}

void Texture::Bind() const noexcept
{
    glBindTexture( GL_TEXTURE_2D, _id );
}

void Texture::Unbind() noexcept
{
    glBindTexture( GL_TEXTURE_2D, 0 );
}

void Texture::BindToUnit( unsigned int unit ) const noexcept
{
    //glActiveTexture( GL_TEXTURE0 + unit );
    //glBindTexture( GL_TEXTURE_2D, _id );
    glBindTextureUnit( unit, _id );
}

void Texture::SetShaderUniform( Shader& shader, const std::string& prefix, int unit ) const
{
    std::string name;
    switch (_type)
    {
    case Types::DIFFUSE:
        name = "diffuse_tex";
        break;
    case Types::SPECULAR:
        name = "specular_tex";
        break;
    case Types::NORMAL:
        name = "normal_tex";
        break;
    case Types::EMISSIVE:
        name = "emissive_tex";
        break;
    case Types::METALLIC:
        name = "metallic_tex";
        break;
    case Types::ROUGHNESS:
        name = "roughness_tex";
        break;
    default:
        break;
    }
    BindToUnit( unit );
    shader.setInt( prefix + "." + name, unit );
}

Image<unsigned char>& Texture::GetImage() noexcept
{
    return _img;
}

const Image<unsigned char>& Texture::GetImage() const noexcept
{
    return _img;
}

void Texture::UpdateData() noexcept
{
    Bind();
    GLenum format = GL_RGB;
    switch (_img.NChannel())
    {
    case 1:
        format = GL_RED;
        break;
    case 2:
        format = GL_RG;
        break;
    case 3:
        format = GL_RGB;
        break;
    case 4:
        format = GL_RGBA;
        break;
    }
    GLenum internal_format = _img.NChannel() == 3 ? GL_RGB : GL_RGBA;
    switch (_img.NChannel())
    {
    case 1:
        internal_format = GL_R8;
        break;
    case 2:
        internal_format = GL_RG8;
        break;
    case 3:
        internal_format = GL_RGB8;
        break;
    case 4:
        internal_format = GL_RGBA8;
        break;
    }
    if (_type == Types::DIFFUSE)
    {
        internal_format = _img.NChannel() == 3 ? GL_SRGB8 : GL_SRGB8_ALPHA8;
    }
    if (_type == Types::DEPTH)
    {
        internal_format = GL_DEPTH_COMPONENT;
    }
    glTexImage2D( GL_TEXTURE_2D, 0, internal_format, _img.Width(), _img.Height(), 0, format, GL_UNSIGNED_BYTE, _img.Data() );
}

int Texture::GetWidth() const
{
    return _width;
}

int Texture::GetHeight() const
{
    return _height;
}

int Texture::GetNbChannel() const
{
    return _nb_channel;
}

TextureDepth::TextureDepth( int width, int height )
{
    _width = width;
    _height = height;
    _nb_channel = 1;
    glCreateTextures( GL_TEXTURE_2D, 1, &_id );
    Bind();
    glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );

}

void TextureDepth::UpdateData() noexcept
{

}

TextureDepthStencil::TextureDepthStencil( int width, int height )
{
    _width = width;
    _height = height;
    _nb_channel = 1;
    glCreateTextures( GL_TEXTURE_2D, 1, &_id );
    Bind();
    glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH24_STENCIL8, width, height, 0, GL_DEPTH_STENCIL, GL_UNSIGNED_INT_24_8, nullptr );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
}

void TextureDepthStencil::UpdateData() noexcept
{
}

Texture2DArray::Texture2DArray( int width, int height, int nb_channel, int nb_layer, unsigned char* data )
    :Texture(), _nb_layer( nb_layer )
{
    _width = width;
    _height = height;
    _nb_channel = nb_channel;
    glCreateTextures( GL_TEXTURE_2D_ARRAY, 1, &_id );

    glBindTexture( GL_TEXTURE_2D_ARRAY, _id );
    GLenum internal_format = GL_RGB8;
    GLenum format = GL_RGB;
    switch (_nb_channel)
    {
    case 1:
        internal_format = GL_R8;
        format = GL_RED;
        break;
    case 2:
        internal_format = GL_RG8;
        format = GL_RG;
        break;
    case 3:
        internal_format = GL_RGB8;
        format = GL_RGB;
        break;
    case 4:
        internal_format = GL_RGBA8;
        format = GL_RGBA;
        break;
    }

    glTexImage3D( GL_TEXTURE_2D_ARRAY, 0, internal_format, _width, _height, _nb_layer, 0, format, GL_FLOAT, data );
    glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
}

Texture2DArray::Texture2DArray( int width, int height, int nb_channel, int nb_layer )
    :Texture(), _nb_layer( nb_layer )
{
    _width = width;
    _height = height;
    _nb_channel = nb_channel;
    glCreateTextures( GL_TEXTURE_2D_ARRAY, 1, &_id );

    glBindTexture( GL_TEXTURE_2D_ARRAY, _id );
    GLenum internal_format = GL_RGB8;
    GLenum format = GL_RGB;
    switch (_nb_channel)
    {
    case 1:
        internal_format = GL_DEPTH_COMPONENT24;
        format = GL_DEPTH_COMPONENT;
        break;
    case 2:
        internal_format = GL_RG8;
        format = GL_RG;
        break;
    case 3:
        internal_format = GL_RGB8;
        format = GL_RGB;
        break;
    case 4:
        internal_format = GL_RGBA8;
        format = GL_RGBA;
        break;
    }
    glTexImage3D( GL_TEXTURE_2D_ARRAY, 0, internal_format, _width, _height, _nb_layer, 0, format, GL_FLOAT, nullptr );
    glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
}

void Texture2DArray::UpdateData() noexcept
{
}

int Texture2DArray::GetNbLayer() const
{
    return _nb_layer;
}

void Texture2DArray::Bind() const noexcept
{
    glBindTexture( GL_TEXTURE_2D_ARRAY, _id );
}