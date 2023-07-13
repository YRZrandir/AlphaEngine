#pragma once
#include "VertexArray.h"
#include <vector>
#include "VertexBuffer.h"
#include "IndexBuffer.h"

VertexArray::VertexArray() : mAttribCount( 0 ), mOffset( 0 )
{
    glGenVertexArrays( 1, &mID );
    glBindVertexArray( mID );
}

VertexArray::~VertexArray()
{
    glDeleteVertexArrays( 1, &mID );
}

VertexArray::VertexArray( VertexArray&& vao ) noexcept
{
    mID = vao.mID;
    mOffset = vao.mOffset;
    mAttribCount = vao.mAttribCount;
    mVertexNumber = vao.mVertexNumber;
    vao.mID = 0;
    vao.mVertexNumber = 0;
}

VertexArray& VertexArray::operator=( VertexArray&& vao ) noexcept
{
    mID = vao.mID;
    mOffset = vao.mOffset;
    mAttribCount = vao.mAttribCount;
    mVertexNumber = vao.mVertexNumber;
    vao.mID = 0;
    vao.mVertexNumber = 0;
    return *this;
}

void VertexArray::AddBuffer( VertexBuffer& vbo )
{
    Bind();
    vbo.Bind();
    const VertexBufferLayout& layout = vbo.GetLayout();
    for (const LayoutElement& e : layout.GetElements())
    {
        glEnableVertexAttribArray( mAttribCount );
        if (e.type == GL_INT || e.type == GL_UNSIGNED_INT || e.type == GL_BYTE || e.type == GL_UNSIGNED_BYTE
            || e.type == GL_SHORT || e.type == GL_UNSIGNED_SHORT)
        {
            glVertexAttribIPointer( mAttribCount, e.count, e.type,
                layout.GetStride(), (const void*)(e.offset) );
        }
        else
        {
            glVertexAttribPointer( mAttribCount, e.count, e.type, e.normalized,
                layout.GetStride(), (const void*)(e.offset) );
        }
        mAttribCount++;
    }
    mVertexNumber = std::min( mVertexNumber, vbo.GetCount() );
}

void VertexArray::Bind() const
{
    glBindVertexArray( mID );
}

bool VertexArray::IsBinding() const
{
    return CurrentBinding() == mID;
}

void VertexArray::BindElementBuffer( const IndexBuffer& ibo ) const
{
    glVertexArrayElementBuffer( mID, ibo.GetID() );
}

unsigned int VertexArray::GetVertexNumber() const
{
    return mVertexNumber;
}

void VertexArray::Unbind()
{
    glBindVertexArray( 0 );
}

GLuint VertexArray::CurrentBinding()
{
    GLint result = 0;
    glGetIntegerv( GL_VERTEX_ARRAY_BINDING, &result );
    return result;
}
