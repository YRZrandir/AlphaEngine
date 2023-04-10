#include "ShaderStorageBuffer.h"

#include <iostream>

ShaderStorageBuffer::ShaderStorageBuffer( const void* data, unsigned int size, GLenum usage )
    :mSize( size )
{
    glCreateBuffers( 1, &mID );
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, mID );
    UpdateData( data, size, usage );
}

ShaderStorageBuffer::ShaderStorageBuffer( ShaderStorageBuffer&& ssbo ) noexcept
{
    mID = ssbo.mID;
    mSize = ssbo.mSize;
    ssbo.mID = 0;
    ssbo.mSize = 0;
}

ShaderStorageBuffer& ShaderStorageBuffer::operator=( ShaderStorageBuffer&& ssbo ) noexcept
{
    mID = ssbo.mID;
    mSize = ssbo.mSize;
    ssbo.mID = 0;
    ssbo.mSize = 0;
    return *this;
}

ShaderStorageBuffer::~ShaderStorageBuffer()
{
    glDeleteBuffers( 1, &mID );
}

unsigned int ShaderStorageBuffer::GetID() const
{
    return mID;
}

unsigned int ShaderStorageBuffer::GetSize() const
{
    return mSize;
}

void ShaderStorageBuffer::Bind()
{
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, mID );
}

void ShaderStorageBuffer::BindBufferBase( unsigned int index )
{
    glBindBufferBase( GL_SHADER_STORAGE_BUFFER, index, mID );
}

void ShaderStorageBuffer::Unbind()
{
    glBindBuffer( GL_SHADER_STORAGE_BUFFER, 0 );
}

void ShaderStorageBuffer::UpdateData( const void* data, unsigned int size, GLenum usage )
{
    mSize = size;
    glNamedBufferData( mID, size, data, usage );
}

void ShaderStorageBuffer::UpdateSubData( const void* data, unsigned int offset, unsigned int size )
{
    glNamedBufferSubData( mID, offset, size, data );
}

void ShaderStorageBuffer::CopyDataFrom( unsigned int id )
{
    unsigned int srcSize = 0;
    glGetNamedBufferParameteriv( id, GL_BUFFER_SIZE, reinterpret_cast<GLint*>(&srcSize) );
    glNamedBufferData( mID, srcSize, nullptr, GL_DYNAMIC_DRAW );
    glCopyNamedBufferSubData( id, mID, 0, 0, srcSize );
}

void ShaderStorageBuffer::CopySubDataFrom( unsigned int srcId, unsigned int readOffset, unsigned int writeOffset, unsigned int size )
{
    if ((size + writeOffset) <= mSize)
    {
        glCopyNamedBufferSubData( srcId, mID, readOffset, writeOffset, size );
    }
    else
    {
        std::cout << mID << "ShaderStorageBuffer::CopySubDataFrom() Failed" << std::endl;
    }
}
