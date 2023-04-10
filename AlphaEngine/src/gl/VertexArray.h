#pragma once
#include <glad/glad.h>

class VertexBufferLayout;
class VertexBuffer;
class IndexBuffer;

class VertexArray
{
private:
    unsigned int mID;
    unsigned int mOffset;
    unsigned int mAttribCount;

public:
    VertexArray();
    VertexArray( const VertexArray& rh ) = delete;
    VertexArray( VertexArray&& rh ) noexcept;
    VertexArray& operator=( const VertexArray& rh ) = delete;
    VertexArray& operator=( VertexArray&& rh ) noexcept;
    ~VertexArray();
    void AddBuffer( VertexBuffer& vbo );
    void Bind() const;
    bool IsBinding() const;
    void BindElementBuffer( const IndexBuffer& ibo ) const;
    static void Unbind();
    static GLuint CurrentBinding();
};