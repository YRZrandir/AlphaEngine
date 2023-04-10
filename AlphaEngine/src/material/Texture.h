#pragma once
#include <string>
#include <glad/glad.h>
#include "../stb_image/Image.h"

class Shader;

class Texture
{
public:
    enum class Types { DIFFUSE, SPECULAR, NORMAL, EMISSIVE, ROUGHNESS, METALLIC, DEPTH, NONE };

public:
    Texture( const std::string& path, Types type );
    Texture( int width, int height, int channel_num, Types type );
    Texture( int width, int height, int channel_num, Types type, unsigned char* data );
    Texture( int width, int height, int channel_num, Types type, float* data );
    Texture( const Texture& rh ) = delete;
    Texture( Texture&& rh ) = delete;
    Texture& operator=( const Texture& rh ) = delete;
    Texture&& operator==( Texture&& rh ) = delete;
    ~Texture();

    Types			GetType() const noexcept;
    GLuint			GetID() const noexcept;
    std::string		GetPath() const noexcept;
    void			Bind() const noexcept;
    void			BindToUnit( unsigned int unit ) const noexcept;
    static void		Unbind() noexcept;
    void			SetShaderUniform( Shader& shader, const std::string& prefix, int unit ) const;
    Image<unsigned char>& GetImage() noexcept;
    const Image<unsigned char>& GetImage() const noexcept;
    void            UpdateData() noexcept;

protected:
    Texture();
    std::string		        _path;
    GLuint			        _id = 0;
    Types			        _type{ Types::NONE };
    Image<unsigned char>    _img;
};

class TextureDepth : public Texture
{
public:
    TextureDepth( int width, int height );
    void UpdateData() noexcept;
};

class TextureDepthStencil : public Texture
{
public:
    TextureDepthStencil( int width, int height );
    void UpdateData() noexcept;
};