#pragma once
enum class BufferUsage
{
    STATIC,
    DYNAMIC
};

unsigned int ToGLBufferUsage( BufferUsage usage );
