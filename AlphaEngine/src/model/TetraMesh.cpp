#include "TetraMesh.h"
#include "input/Input.h"
#include "util/Shader.h"
#include "model/Triangle.h"
#include "util/Camera.h"
#include "tetgen.h"
#include "acceleration/AABB.h"
#include "model/HalfEdgeMesh.h"
#include "lighting/Light.h"

TetraMesh::TetraMesh()
{
    mVAO = std::make_unique<VertexArray>();
    mVBO = nullptr;
    mSurfaceIBO = nullptr;
    mTotalIBO = nullptr;
    mShader = Shader::Find( "tetra" );
}

void TetraMesh::CreateFromSurface( const std::string& path )
{
    auto surface = std::make_unique<HalfEdgeMesh>( path );
    std::vector<float> pts( surface->GetVertexNumber() * 3 );
    std::vector<int> triangles( surface->GetFaceNumber() * 3 );
    for (int i = 0; i < surface->GetVertexNumber(); ++i)
    {
        pts[i * 3 + 0] = surface->GetPosition( i ).x;
        pts[i * 3 + 1] = surface->GetPosition( i ).y;
        pts[i * 3 + 2] = surface->GetPosition( i ).z;
    }
    for (int i = 0; i < surface->GetFaceNumber(); ++i)
    {
        auto [i0, i1, i2] = surface->GetFaceIndices( i );
        triangles[i * 3 + 0] = i0;
        triangles[i * 3 + 1] = i1;
        triangles[i * 3 + 2] = i2;
    }

    CreateFromSurface( pts, triangles );
}

void TetraMesh::CreateFromSurface( const std::vector<float>& points, const std::vector<int>& triangles )
{
    mPoints.clear();
    mRestPos.clear();
    mTetras.clear();
    mBorderFaces.clear();
    mFaces.clear();
    mColors.clear();
    mFaceMarkers.clear();

    tetgenio in, out;
    std::vector<Tetrahedron> tetras;
    std::vector<Triangle> faces;
    std::vector<int> faceMarkers;
    std::vector<glm::vec3> tetraPoints;

    in.firstnumber = 0;
    in.numberofpoints = points.size() / 3;
    in.pointlist = new REAL[in.numberofpoints * 3];
    for (int i = 0; i < in.numberofpoints; i++)
    {
        in.pointlist[i * 3 + 0] = points[i * 3 + 0];
        in.pointlist[i * 3 + 1] = points[i * 3 + 1];
        in.pointlist[i * 3 + 2] = points[i * 3 + 2];
    }
    in.numberoffacets = triangles.size() / 3;
    in.facetlist = new tetgenio::facet[in.numberoffacets];
    in.facetmarkerlist = new int[in.numberoffacets];

    for (int i = 0; i < in.numberoffacets; i++)
    {
        tetgenio::facet* f = &in.facetlist[i];
        f->numberofpolygons = 1;
        f->polygonlist = new tetgenio::polygon[1];
        f->numberofholes = 0;
        f->holelist = nullptr;
        tetgenio::polygon* p = &f->polygonlist[0];
        p->numberofvertices = 3;
        p->vertexlist = new int[p->numberofvertices];
        p->vertexlist[0] = triangles[i * 3 + 0];
        p->vertexlist[1] = triangles[i * 3 + 1];
        p->vertexlist[2] = triangles[i * 3 + 2];
        in.facetmarkerlist[i] = 1;
    }

    tetgenbehavior behavior;
    behavior.plc = 1;
    behavior.facesout = 1;
    behavior.edgesout = 1;
    behavior.neighout = 2;
    behavior.minratio = 1.4;
    behavior.quality = 1;
    behavior.nobisect = 1;
    tetrahedralize( &behavior, &in, &out );

    int pointNum = out.numberofpoints;
    for (int i = 0; i < pointNum; i++)
    {
        mPoints.emplace_back( out.pointlist[i * 3], out.pointlist[i * 3 + 1], out.pointlist[i * 3 + 2] );
    }
    mRestPos = mPoints;
    mColors.resize( mPoints.size(), glm::vec3( 0.8, 1, 1 ) );

    int faceNum = out.numberoftrifaces;
    for (int i = 0; i < faceNum; i++)
    {
        mFaces.emplace_back( static_cast<unsigned int>(out.trifacelist[i * 3]),
            static_cast<unsigned int>(out.trifacelist[i * 3 + 2]),
            static_cast<unsigned int>(out.trifacelist[i * 3 + 1]) );
        mFaceMarkers.push_back( out.trifacemarkerlist[i] );
    }

    int tetNum = out.numberoftetrahedra;
    for (int i = 0; i < tetNum; i++)
    {
        Tetrahedron tet;
        for (int j = 0; j < 4; j++)
        {
            tet[j] = out.tetrahedronlist[i * 4 + j];
            tet.Neighbor( j ) = out.neighborlist[i * 4 + j];
        }
        mTetras.push_back( tet );
    }

    AABB aabb;
    for (auto& p : tetraPoints)
    {
        aabb.Expand( p );
    }

    mDrawDepthThreshold = aabb.GetCenter().x;
    CalcBorderFaces();
    InitBuffers();

    //std::ofstream ofs( "D:/tetra.obj" );
    //std::ofstream ofs2( "D:/tetra2.obj" );
    //for (int i = 0; i < pointNum; i++)
    //{
    //    ofs << "v " << mPoints[i].x << ' ' << mPoints[i].y << ' ' << mPoints[i].z << '\n';
    //    ofs2 << "v " << mPoints[i].x << ' ' << mPoints[i].y << ' ' << mPoints[i].z << '\n';
    //}
    //for (int i = 0; i < faceNum; i++)
    //{
    //    int ia = mFaces[i].a;
    //    int ib = mFaces[i].b;
    //    int ic = mFaces[i].c;
    //    if (glm::dot( mPoints[ia] - glm::vec3( 0, 0, -0.1 ), glm::vec3( 0.2, 0, 0.6 ) ) > 0 &&
    //        glm::dot( mPoints[ib] - glm::vec3( 0, 0, -0.1 ), glm::vec3( 0.2, 0, 0.6 ) ) > 0 &&
    //        glm::dot( mPoints[ic] - glm::vec3( 0, 0, -0.1 ), glm::vec3( 0.2, 0, 0.6 ) ) > 0)
    //    {
    //        ofs << "f " << mFaces[i].a + 1 << ' ' << mFaces[i].b + 1 << ' ' << mFaces[i].c + 1 << '\n';
    //    }
    //    else
    //    {
    //        ofs2 << "f " << mFaces[i].a + 1 << ' ' << mFaces[i].b + 1 << ' ' << mFaces[i].c + 1 << '\n';
    //    }
    //}
    //in.clean_memory();
    //out.clean_memory();
}

unsigned int TetraMesh::GetTetraNum() const
{
    return mTetras.size();
}

unsigned int TetraMesh::GetFaceNum() const
{
    return mFaces.size();
}

unsigned int TetraMesh::GetBorderFaceNum() const
{
    return mBorderFaces.size();
}

unsigned int TetraMesh::GetPointNum() const
{
    return mPoints.size();
}

float TetraMesh::GetTetraVolume( int index ) const
{
    const Tetrahedron& tetra = mTetras[index];
    const auto& p0 = mPoints[tetra.a];
    const auto& p1 = mPoints[tetra.b];
    const auto& p2 = mPoints[tetra.c];
    const auto& p3 = mPoints[tetra.d];
    return glm::dot( glm::cross( (p1 - p0), (p2 - p0) ), (p3 - p0) ) / 6.0f;
}

void TetraMesh::SetShader( Shader* shader )
{
    mShader = shader;
}

void TetraMesh::InitBuffers()
{
    mVBO = std::make_unique<VertexBuffer>( mPoints.data(), mPoints.size() * sizeof( glm::vec3 ) );
    mColorVBO = std::make_unique<VertexBuffer>( mColors.data(), mColors.size() * sizeof( mColors[0] ) );
    mSurfaceIBO = std::make_unique<IndexBuffer>( &(mBorderFaces[0].a), mBorderFaces.size() * 3 );
    mTotalIBO = std::make_unique<IndexBuffer>( &(mFaces[0].a), mFaces.size() * 3 );
    VertexBufferLayout layout_pos;
    layout_pos.Push( GL_FLOAT, 3, 0 );
    mVBO->SetLayout( layout_pos );
    mVAO->AddBuffer( *mVBO );
    VertexBufferLayout layout2;
    layout2.Push( GL_FLOAT, 3, 0 );
    mColorVBO->SetLayout( layout2 );
    mVAO->AddBuffer( *mColorVBO );
}

void TetraMesh::UpdateBuffers()
{
    mVBO->UpdateData( mPoints.data(), mPoints.size() * sizeof( glm::vec3 ) );
    mColorVBO->UpdateData( mColors.data(), mColors.size() * sizeof( mColors[0] ) );
}

void TetraMesh::CalcBorderFaces()
{
    mBorderFaces.clear();
    unsigned int faceNum = mFaces.size();
    for (size_t i = 0; i < faceNum; i++)
    {
        if (mFaceMarkers[i] == 1)
        {
            mBorderFaces.emplace_back( mFaces[i] );
        }
    }
}

void TetraMesh::Update()
{
    if (Input::IsKeyHeld( Input::Key::UP ))
    {
        mDrawDepthThreshold += 0.01f;
    }
    if (Input::IsKeyHeld( Input::Key::DOWN ))
    {
        mDrawDepthThreshold -= 0.01f;
    }
}

void TetraMesh::Draw()
{
    mVAO->Bind();
    mShader->use();
    mShader->setFloat( "uThreshold", 0.0f );

    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    glEnable( GL_POLYGON_OFFSET_FILL );
    glPolygonOffset( 1.0, 1.0 );
    mTotalIBO->Bind();
    mShader->setFloat( "uThreshold", -1.0 );
    mShader->setVec( "uColor", glm::vec3( 0.019608, 0.721569, 0.882353 ) );
    glDrawElements( GL_TRIANGLES, mTotalIBO->GetCount(), GL_UNSIGNED_INT, nullptr );
    glDisable( GL_POLYGON_OFFSET_FILL );
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    mShader->setVec( "uColor", glm::vec3( 0 ) );
    mShader->setFloat( "uThreshold", 3.0 );
    glDrawElements( GL_TRIANGLES, mTotalIBO->GetCount(), GL_UNSIGNED_INT, nullptr );
    mShader->setFloat( "uThreshold", 1.0 );
    glDrawElements( GL_TRIANGLES, mTotalIBO->GetCount(), GL_UNSIGNED_INT, nullptr );
    //mTotalIBO->Bind();
    //glDrawElements( GL_TRIANGLES, mSurfaceIBO->GetCount(), GL_UNSIGNED_INT, nullptr );
}
