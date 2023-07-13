#pragma once
#include <mutex>
#include <vector>
#include <memory>
#include <glad/glad.h>

struct LayoutElement
{
    unsigned int type;
    size_t count;
    size_t offset;
    unsigned int normalized = GL_FALSE;
};


class VertexBufferLayout
{
private:
    size_t m_Stride;
    std::vector<LayoutElement> m_Elements;
    friend std::ostream& operator<<( std::ostream& os, const VertexBufferLayout& layout );
public:
    VertexBufferLayout();

    void Push( unsigned int type, size_t count, size_t offset, unsigned int normalized = GL_FALSE );

    inline size_t GetStride() const { return m_Stride; }

    inline void SetStride( size_t stride ) { m_Stride = stride; }

    inline const std::vector<LayoutElement>& GetElements() const { return m_Elements; }
};

std::ostream& operator<<( std::ostream& os, const VertexBufferLayout& layout );

class VertexBuffer
{
private:
    unsigned int _id;
    VertexBufferLayout _layout;
    size_t _size{ 0 };
    static std::mutex _gl_lock;

public:
    VertexBuffer( const void* data, unsigned int size, GLenum usage = GL_STATIC_DRAW );
    VertexBuffer( const VertexBuffer& rh ) = delete;
    VertexBuffer& operator=( const VertexBuffer& rh ) = delete;
    VertexBuffer& operator=( VertexBuffer&& rh ) noexcept;
    VertexBuffer( VertexBuffer&& rh ) noexcept;
    ~VertexBuffer();

    GLuint GetID() const;
    size_t GetSize() const;
    size_t GetCount() const;
    void Bind() const;
    bool IsBinding() const;
    void UpdateData( const void* data, unsigned int size, GLenum usage = GL_STATIC_DRAW );
    void UpdateSubData( const void* data, unsigned int offset, unsigned int size );
    std::vector<unsigned char> GetSubData( unsigned int offset, unsigned int size );
    void CopyDataFrom( const VertexBuffer& src );
    void CopyDataFrom( unsigned src );
    void CopySubDataFrom( const VertexBuffer& src, unsigned int readOffset, unsigned int writeOffset, unsigned int size );
    void CopySubDataFrom( unsigned src, unsigned int readOffset, unsigned int writeOffset, unsigned int size );
    std::unique_ptr<VertexBuffer> Copy() const;
    const VertexBufferLayout& GetLayout() const;
    void SetLayout( VertexBufferLayout layout );

    static void Unbind();
};