#pragma once
#include <memory>
#include <glad/glad.h>
#include <glm/glm.hpp>

#include "material/Texture.h"

class FrameBuffer
{
public:
    FrameBuffer( int width, int height, bool has_color = true, bool has_depth = true, bool has_stencil = true );
    ~FrameBuffer();
    void Clear();
    void Bind();
    static void Unbind();
    GLuint GetID() const noexcept { return _id; };
    GLuint GetRenderTargetID() const;
    Texture* GetDepthTexture() { return _depth_target.get(); };
    Texture* GetRenderTexture() { return _render_target.get(); };
    Texture* GetDepthStencilTexture() { return _ds_target.get(); }
    glm::vec3 ReadPixel( int x, int y );
    void ChangeSize( int width, int height );
    bool HasColorBuffer() const noexcept { return _has_color; };
    bool HasDepthBuffer() const noexcept { return _has_depth; };
    bool HasStencilBuffer() const noexcept { return _has_stencil; };
    int Width() { return _width; }
    int Height() { return _height; }
private:
    GLuint _id;
    std::unique_ptr<Texture> _render_target;
    std::unique_ptr<Texture> _depth_target;
    std::unique_ptr<Texture> _ds_target;
    int _width;
    int _height;
    bool _has_color;
    bool _has_depth;
    bool _has_stencil;
};
