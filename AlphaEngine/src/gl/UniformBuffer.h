#pragma once
#include <glad/glad.h>

class UniformBuffer
{
public:
    UniformBuffer();
    UniformBuffer( size_t size, const void* data, GLenum usage = GL_STATIC_DRAW );
    UniformBuffer( const UniformBuffer& ) = delete;
    UniformBuffer& operator=( const UniformBuffer& ) = delete;
    UniformBuffer( UniformBuffer&& ubo ) noexcept;
    UniformBuffer& operator=( UniformBuffer&& ubo ) noexcept;
    ~UniformBuffer();
    void Bind();
    void Unbind();
    void BindBase( unsigned int index );
    void SetData( size_t size, const void* data, GLenum usage = GL_STATIC_DRAW );
protected:
    unsigned int _id = 0;
};