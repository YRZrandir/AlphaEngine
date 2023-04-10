#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <memory>
#include "../model/Transform.h"
#include "../util/SceneObject.h"
#include "../gl/VertexArray.h"
#include "../gl/VertexBuffer.h"
#include "../gl/IndexBuffer.h"
#include "../gl/ShaderStorageBuffer.h"

class Shader;
class Triangle;
class RenderConfig;
class Transform;

class Tetrahedron
{
public:
    int a;
    int b;
    int c;
    int d;

    //neighbors
    int na;
    int nb;
    int nc;
    int nd;

    int& operator[]( int i )
    {
        switch (i)
        {
        case 0:
            return a;
        case 1:
            return b;
        case 2:
            return c;
        case 3:
            return d;
        }
    }

    const int& operator[]( int i ) const
    {
        switch (i)
        {
        case 0:
            return a;
        case 1:
            return b;
        case 2:
            return c;
        case 3:
            return d;
        }
    }

    int& Neighbor( int i )
    {
        switch (i)
        {
        case 0:
            return na;
        case 1:
            return nb;
        case 2:
            return nc;
        case 3:
            return nd;
        }
    }

    const int& Neighbor( int i ) const
    {
        switch (i)
        {
        case 0:
            return na;
        case 1:
            return nb;
        case 2:
            return nc;
        case 3:
            return nd;
        }
    }
};

class TetraMesh : public SceneObject
{
public:
    TetraMesh();
    ~TetraMesh() = default;
    TetraMesh( const TetraMesh& rh ) = delete;
    TetraMesh( TetraMesh&& rh ) = delete;
    TetraMesh& operator=( const TetraMesh& rh ) = delete;
    TetraMesh& operator=( TetraMesh&& rh ) = delete;
    void CreateFromSurface( const std::string& path );
    void CreateFromSurface( const std::vector<float>& points, const std::vector<int>& triangles );

    unsigned int    GetTetraNum() const;
    unsigned int    GetFaceNum() const;
    unsigned int    GetBorderFaceNum() const;
    unsigned int    GetPointNum() const;
    float           GetTetraVolume( int index ) const;
    void SetShader( Shader* shader );

    void InitBuffers();
    void UpdateBuffers();
    void CalcBorderFaces();
    // 通过 SceneObject 继承
    virtual void Update() override;
    virtual void Draw() override;

public:
    std::vector<Tetrahedron>    mTetras;
    std::vector<Triangle>       mFaces;
    std::vector<int>            mFaceMarkers;
    std::vector<glm::vec3>      mPoints;
    std::vector<glm::vec3>      mColors;
    std::vector<glm::vec3>      mRestPos;
    std::vector<Triangle>       mBorderFaces;
    float mDrawDepthThreshold = 1.0f;

    std::unique_ptr<VertexBuffer>   mVBO;

private:
    Shader* mShader;
    std::unique_ptr<VertexArray>    mVAO;
    std::unique_ptr<VertexBuffer>   mColorVBO;
    std::unique_ptr<IndexBuffer>    mSurfaceIBO;
    std::unique_ptr<IndexBuffer>    mTotalIBO;
};

