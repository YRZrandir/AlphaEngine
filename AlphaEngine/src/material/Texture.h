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
    virtual ~Texture();

    virtual void LoadFromFile( const std::string& path );
    Types			GetType() const noexcept;
    GLuint			GetID() const noexcept;
    std::string		GetPath() const noexcept;
    virtual void	Bind() const noexcept;
    void			BindToUnit( unsigned int unit ) const noexcept;
    static void		Unbind() noexcept;
    void			SetShaderUniform( Shader& shader, const std::string& prefix, int unit ) const;
    Image<unsigned char>& GetImage() noexcept;
    const Image<unsigned char>& GetImage() const noexcept;
    virtual void    UpdateData() noexcept;
    int GetWidth() const;
    int GetHeight() const;
    int GetNbChannel() const;

protected:
    Texture();
    std::string		        _path;
    GLuint			        _id{ 0 };
    Types			        _type{ Types::NONE };
    Image<unsigned char>    _img;
    int _width{ 0 };
    int _height{ 0 };
    int _nb_channel{ 0 };
};

class TextureDepth : public Texture
{
public:
    TextureDepth( int width, int height );
    virtual void UpdateData() noexcept override;
    virtual void LoadFromFile( const std::string& path ) override {};
};

class TextureDepthStencil : public Texture
{
public:
    TextureDepthStencil( int width, int height );
    virtual void UpdateData() noexcept override;
    virtual void LoadFromFile( const std::string& path ) override {};
};

class Texture2DArray : public Texture
{
public:
    Texture2DArray( int width, int height, int nb_channel, int nb_layer, unsigned char* data );
    Texture2DArray( int width, int height, int nb_channel, int nb_layer );

    virtual void LoadFromFile( const std::string& path ) override {};
    virtual void Bind() const noexcept override;
    virtual void UpdateData() noexcept override;
    int GetNbLayer() const;

protected:
    int _nb_layer{ 0 };
};