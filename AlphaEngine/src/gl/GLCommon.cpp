#include <glad/glad.h>
#include "GLCommon.h"

unsigned int ToGLBufferUsage( BufferUsage usage )
{
    switch (usage)
    {
    case BufferUsage::STATIC:
        return GL_STATIC_DRAW;
        break;
    case BufferUsage::DYNAMIC:
        return GL_DYNAMIC_DRAW;
        break;
    default:
        break;
    }
}