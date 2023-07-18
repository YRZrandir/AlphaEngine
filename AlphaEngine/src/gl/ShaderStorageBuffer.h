#pragma once
#include <vector>
#include <glad/glad.h>

class ShaderStorageBuffer
{
private:
    GLuint mID;
    unsigned int mSize;

public:
    ShaderStorageBuffer( const void* data, unsigned int size, GLenum usage = GL_DYNAMIC_READ );
    ~ShaderStorageBuffer();
    ShaderStorageBuffer( const ShaderStorageBuffer& rh ) = delete;
    ShaderStorageBuffer( ShaderStorageBuffer&& rh ) noexcept;
    ShaderStorageBuffer& operator=( const ShaderStorageBuffer& rh ) = delete;
    ShaderStorageBuffer& operator=( ShaderStorageBuffer&& rh ) noexcept;

    unsigned int GetID() const;
    unsigned int GetSize() const;
    void Bind() const;
    void BindBufferBase( unsigned int index ) const;
    static void Unbind();
    void UpdateData( const void* data, unsigned int size, GLenum usage = GL_DYNAMIC_READ );
    void UpdateSubData( const void* data, unsigned int offset, unsigned int size );
    void CopyDataFrom( unsigned int srcId );
    void CopySubDataFrom( unsigned int srcId, unsigned int readOffset, unsigned int writeOffset, unsigned int size );
};

