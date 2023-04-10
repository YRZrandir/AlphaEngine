#include "FrameBuffer.h"

FrameBuffer::FrameBuffer( int width, int height, bool has_color, bool has_depth, bool has_stencil )
    :_width( width ), _height( height ), _has_color( has_color ), _has_depth( has_depth ), _has_stencil( has_stencil )
{
    glGenFramebuffers( 1, &_id );
    Bind();

    if (_has_color)
    {
        _render_target = std::make_unique<Texture>( _width, _height, 3, Texture::Types::NONE );
        _render_target->BindToUnit( 0 );
        _render_target->Bind();
        glFramebufferTexture2D( GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, _render_target->GetID(), 0 );

    }
    else
    {
        //glNamedFramebufferDrawBuffer( GL_NONE, _id );
        //glNamedFramebufferReadBuffer( GL_NONE, _id );
        glDrawBuffer( GL_NONE );
        glReadBuffer( GL_NONE );
    }

    if (_has_depth && !_has_stencil)
    {
        _depth_target = std::make_unique<TextureDepth>( _width, _height );
        glFramebufferTexture2D( GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, _depth_target->GetID(), 0 );
    }
    else if (_has_stencil)
    {
        _ds_target = std::make_unique<TextureDepthStencil>( _width, _height );
        glFramebufferTexture2D( GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, _ds_target->GetID(), 0 );
    }
    //if (glCheckFramebufferStatus( GL_FRAMEBUFFER ) == GL_FRAMEBUFFER_COMPLETE)
    //{
    //    std::cout << "framebuffer complete!" << std::endl;
    //}
    Clear();
    Unbind();
}

FrameBuffer::~FrameBuffer()
{
    glDeleteFramebuffers( 1, &_id );
}

void FrameBuffer::Clear()
{
    if (_has_color)
    {
        glClear( GL_COLOR_BUFFER_BIT );
    }
    if (_has_depth)
    {
        glClear( GL_DEPTH_BUFFER_BIT );
    }
}

void FrameBuffer::Bind()
{
    glBindFramebuffer( GL_FRAMEBUFFER, _id );
}

void FrameBuffer::Unbind()
{
    glBindFramebuffer( GL_FRAMEBUFFER, 0 );
}

GLuint FrameBuffer::GetRenderTargetID() const
{
    return _render_target->GetID();
}

glm::vec3 FrameBuffer::ReadPixel( int x, int y )
{
    Bind();
    x %= _width;
    y %= _height;
    x = x > 0 ? x : x + _width;
    y = y > 0 ? y : y + _height;
    glm::vec3 pixel;
    glReadPixels( x, y, 1, 1, GL_RGB, GL_FLOAT, &pixel );
    Unbind();
    return pixel;
}

void FrameBuffer::ChangeSize( int width, int height )
{
    Bind();
    _width = width;
    _height = height;
    if (_has_color)
    {
        _render_target = std::make_unique<Texture>( _width, _height, 3, Texture::Types::NONE );
        _render_target->BindToUnit( 0 );
        _render_target->Bind();
    }

    if (_has_depth)
    {
        _depth_target = std::make_unique<TextureDepth>( _width, _height );
        glFramebufferTexture( GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, _depth_target->GetID(), 0 );
    }

    Unbind();
}
