#pragma once
#include "VertexArray.h"
#include "IndexBuffer.h"

class Renderer
{
public:
    void SetVAO( VertexArray* vao );
    void SetIBO( IndexBuffer* ibo );
    
private:
    VertexArray* _vao;
    IndexBuffer* _ibo;
};

