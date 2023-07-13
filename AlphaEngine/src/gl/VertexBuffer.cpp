#include "VertexBuffer.h"
#include "../util/util.h"

VertexBufferLayout::VertexBufferLayout() : m_Stride( 0 )
{
}

void VertexBufferLayout::Push( unsigned int type, size_t count, size_t offset, unsigned int normalized )
{
    m_Elements.push_back( { type, count, offset, normalized } );
    m_Stride += glutil::Type2Size( type ) * count;
}

std::ostream& operator<<( std::ostream& os, const VertexBufferLayout& layout )
{
    for (auto& element : layout.GetElements())
    {
        os << element.count << glutil::Type2String( element.type ) << ' ';
    }
    os << std::endl;
    return os;
}

std::mutex VertexBuffer::_gl_lock;

VertexBuffer::VertexBuffer( const void* data, unsigned int size, GLenum usage )
{
    std::lock_guard<std::mutex> lock( _gl_lock );
    glGenBuffers( 1, &_id );
    glBindBuffer( GL_ARRAY_BUFFER, _id );
    glBufferData( GL_ARRAY_BUFFER, size, data, usage );
    _size = size;
}

VertexBuffer& VertexBuffer::operator=( VertexBuffer&& rh ) noexcept
{
    _id = rh._id;
    _size = rh._size;
    rh._id = 0;
    rh._size = 0;
    return *this;
}

VertexBuffer::VertexBuffer( VertexBuffer&& rh ) noexcept
    :_id( rh._id ),
    _size( rh._size )
{
    rh._id = 0;
    rh._size = 0;
}


VertexBuffer::~VertexBuffer()
{
    glDeleteBuffers( 1, &_id );
}

GLuint VertexBuffer::GetID() const
{
    return _id;
}

size_t VertexBuffer::GetSize() const
{
    return _size;
}

size_t VertexBuffer::GetCount() const
{
    if (_size == 0)
        return 0;
    return _size / _layout.GetStride();
}

void VertexBuffer::Bind() const
{
    glBindBuffer( GL_ARRAY_BUFFER, _id );
}

bool VertexBuffer::IsBinding() const
{
    GLint id = 0;
    glGetIntegerv( GL_ARRAY_BUFFER_BINDING, &id );
    return id == _id;
}

void VertexBuffer::Unbind()
{
    glBindBuffer( GL_ARRAY_BUFFER, 0 );
}

void VertexBuffer::UpdateData( const void* data, unsigned int size, GLenum usage )
{
    std::lock_guard<std::mutex> lock( _gl_lock );
    Bind();
    glBufferData( GL_ARRAY_BUFFER, size, data, usage );
    _size = size;
}

void VertexBuffer::UpdateSubData( const void* data, unsigned int offset, unsigned int size )
{
    std::lock_guard<std::mutex> lock( _gl_lock );
    Bind();
    glBufferSubData( GL_ARRAY_BUFFER, offset, size, data );
}

std::vector<unsigned char> VertexBuffer::GetSubData( unsigned int offset, unsigned int size )
{
    std::vector<unsigned char> buf;
    buf.resize( size );
    glGetBufferSubData( GL_ARRAY_BUFFER, offset, size, const_cast<void*>(static_cast<const void*>(buf.data())) );
    return buf;
}

void VertexBuffer::CopyDataFrom( const VertexBuffer& src )
{
    CopyDataFrom( src.GetID() );
}

void VertexBuffer::CopyDataFrom( unsigned src )
{
    std::lock_guard<std::mutex> lock( _gl_lock );
    int size = 0;
    glGetNamedBufferParameteriv( src, GL_BUFFER_SIZE, &size );
    glNamedBufferData( _id, size, nullptr, GL_DYNAMIC_DRAW );
    glCopyNamedBufferSubData( src, _id, 0, 0, size );
    _size = size;
}

void VertexBuffer::CopySubDataFrom( const VertexBuffer& src, unsigned int readOffset, unsigned int writeOffset, unsigned int size )
{
    CopySubDataFrom( src.GetID(), readOffset, writeOffset, size );
}

void VertexBuffer::CopySubDataFrom( unsigned src, unsigned int readOffset, unsigned int writeOffset, unsigned int size )
{
    std::lock_guard<std::mutex> lock( _gl_lock );
    glCopyNamedBufferSubData( src, _id, readOffset, writeOffset, size );
}

std::unique_ptr<VertexBuffer> VertexBuffer::Copy() const
{
    int size = 0;
    glGetNamedBufferParameteriv( _id, GL_BUFFER_SIZE, &size );
    GLenum usage = GL_STATIC_DRAW;
    glGetNamedBufferParameteriv( _id, GL_BUFFER_USAGE, (GLint*)&usage );
    auto ret = std::make_unique<VertexBuffer>( nullptr, size, usage );
    ret->SetLayout( _layout );
    ret->CopyDataFrom( _id );
    return ret;
}

const VertexBufferLayout& VertexBuffer::GetLayout() const
{
    return _layout;
}

void VertexBuffer::SetLayout( VertexBufferLayout layout )
{
    _layout = std::move( layout );
}


