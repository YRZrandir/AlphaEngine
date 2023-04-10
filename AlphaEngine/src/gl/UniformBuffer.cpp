#pragma once
#include "UniformBuffer.h"

UniformBuffer::UniformBuffer()
{
    glCreateBuffers( 1, &_id );
}

UniformBuffer::UniformBuffer( size_t size, const void* data, GLenum usage )
{
    glCreateBuffers( 1, &_id );
    glNamedBufferData( _id, size, data, usage );
}

UniformBuffer::UniformBuffer( UniformBuffer&& ubo ) noexcept
{
    _id = ubo._id;
    ubo._id = 0;
}

UniformBuffer& UniformBuffer::operator=( UniformBuffer&& ubo ) noexcept
{
    _id = ubo._id;
    ubo._id = 0;
    return *this;
}

UniformBuffer::~UniformBuffer()
{
    glDeleteBuffers( 1, &_id );
}

void UniformBuffer::Bind()
{
    glBindBuffer( GL_UNIFORM_BUFFER, _id );
}

void UniformBuffer::Unbind()
{
    glBindBuffer( GL_UNIFORM_BUFFER, 0 );
}

void UniformBuffer::BindBase( unsigned int index )
{
    glBindBufferBase( GL_UNIFORM_BUFFER, index, _id );
}

void UniformBuffer::SetData( size_t size, const void* data, GLenum usage )
{
    glNamedBufferData( _id, size, data, usage );
}
