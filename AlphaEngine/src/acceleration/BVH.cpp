#include "BVH.h"
#include <algorithm>
#include <numeric>

#include "../gl/VertexArray.h"
#include "../gl/VertexBuffer.h"
#include "../gl/IndexBuffer.h"
#include "../util/Shader.h"
#include "../util/Camera.h"
#include "../util/Intersection.h"
#include "../PBD/MetaballModel.h"

/* BVHNode */
const int BVHNode::THRESHOLD = 8;
bool BVHNode::sInit = false;
std::unique_ptr<VertexArray> BVHNode::sVAO = nullptr;
std::unique_ptr<VertexBuffer> BVHNode::sVBO = nullptr;
std::unique_ptr<IndexBuffer> BVHNode::sIBO = nullptr;
std::array<glm::vec3, 8> BVHNode::sVertexData =
{
    glm::vec3{0.5f, 0.5f, 0.5f}, glm::vec3{-0.5f, 0.5f, 0.5f}, glm::vec3{-0.5f, 0.5f, -0.5f}, glm::vec3{0.5f, 0.5f, -0.5f},
    glm::vec3{0.5f, -0.5f, 0.5f}, glm::vec3{-0.5f, -0.5f, 0.5f}, glm::vec3{-0.5f, -0.5f, -0.5f}, glm::vec3{0.5f, -0.5f, -0.5f}
};
std::array<unsigned int, 24> BVHNode::sIndexData =
{
    0, 1, 1, 2, 2, 3, 3, 0,
    0, 4, 1, 5, 2, 6, 3, 7,
    4, 5, 5, 6, 6, 7, 7, 4
};

void BVHNode::Draw( int level ) const
{
    if (!BVHNode::sInit)
    {
        BVHNode::Init();
    }
    if (level > 8)
    {
        return;
    }
    if (level <= 8)
    {
        auto shader = Shader::Find( "bvh" );
        shader->use();
        sVAO->Bind();
        glm::mat4 model = glm::translate( glm::mat4( 1.0f ), mBoundingBox.GetCenter() );
        model = glm::scale( model, mBoundingBox.max_corner - mBoundingBox.min_corner );
        shader->setMat( "uModelMat", model );
        shader->setMat( "uViewMat", Camera::current->GetViewMatrix() );
        shader->setMat( "uProjectionMat", Camera::current->GetProjectionMatrix() );
        shader->setVec( "uColor", mColor * ((float)level / 4.0f) );
        glDrawElements( GL_LINES, sIBO->GetCount(), GL_UNSIGNED_INT, nullptr );
    }
    if (mLeft)
    {
        mLeft->Draw( level + 1 );
    }
    if (mRight)
    {
        mRight->Draw( level + 1 );
    }
}

void BVHNode::Init()
{
    if (BVHNode::sInit)
    {
        return;
    }
    BVHNode::sInit = true;
    sVBO = std::make_unique<VertexBuffer>( sVertexData.data(), sVertexData.size() * sizeof( glm::vec3 ) );
    sIBO = std::make_unique<IndexBuffer>( sIndexData.data(), sIndexData.size() );
    VertexBufferLayout layout;
    layout.Push( GL_FLOAT, 3, 0 );
    sVBO->SetLayout( layout );
    sVAO = std::make_unique<VertexArray>();
    sVAO->AddBuffer( *sVBO );
    sVAO->BindElementBuffer( *sIBO );
}

void BVHNode::ResetColor() const
{
    mColor = glm::vec3( 0.97, 0.97, 0.58 );
    if (mLeft)
    {
        mLeft->ResetColor();
    }
    if (mRight)
    {
        mRight->ResetColor();
    }
}

bool BVHNode::IsPointInBVH( const glm::vec3& p ) const
{
    if (mBoundingBox.Intersect( p ))
    {
        bool left = false;
        bool right = false;
        if (mLeft)
        {
            left = mLeft->IsPointInBVH( p );
        }
        if (mRight)
        {
            right = mRight->IsPointInBVH( p );
        }
        if (!mLeft && !mRight)
        {
            return true;
        }
        return left || right;
    }
    return false;
}

AABB BVHNode::BoundingBox() const
{
    return mBoundingBox;
}

/* BVHTree */
void BVHTree::Update()
{
}

void BVHTree::Draw()
{
    _root->Draw( 0 );
}

AABB BVHTree::BoundingBox() const
{
    return _root->BoundingBox();
}

/* BVHTree_HalfEdgeMesh */
BVHTree_HalfEdgeMesh::BVHTree_HalfEdgeMesh( HalfEdgeMesh* mesh )
    :_mesh( mesh )
{
    if (!_mesh)
    {
        throw std::runtime_error( "BVHTree: No init mesh" );
    }
    std::vector<int> indices( _mesh->GetFaceNumber() );
    std::iota( indices.begin(), indices.end(), 0 );

    _root = std::make_unique<BVHNode_HalfEdgeMesh>( indices, _mesh );
}

void BVHTree_HalfEdgeMesh::Refit()
{
    _root->Refit();
}

bool BVHTree_HalfEdgeMesh::CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const
{
    rec->t = glm::distance( p0, p1 );
    return _root->CheckLineseg( p0, p1, rec, id );
}

/* BVHNode_HalfEdgeMesh */
BVHNode_HalfEdgeMesh::BVHNode_HalfEdgeMesh( std::vector<int> indices, HalfEdgeMesh* mesh )
    :_mesh( mesh )
{
    for (const auto& i : indices)
    {
        auto indices = mesh->GetFaceIndices( i );
        mBoundingBox.Expand( mesh->_vertices[std::get<0>( indices )].pos );
        mBoundingBox.Expand( mesh->_vertices[std::get<1>( indices )].pos );
        mBoundingBox.Expand( mesh->_vertices[std::get<2>( indices )].pos );
    }
    if (indices.size() <= BVHNode::THRESHOLD)
    {
        mIndices = indices;
        return;
    }

    unsigned int axis = mBoundingBox.LongestAxis();

    std::sort( indices.begin(), indices.end(), [&]( int left, int right )->bool {
        const auto& tri_left = mesh->_faces[left];
        const auto& tri_right = mesh->_faces[right];
        auto [ia, ib, ic] = mesh->GetFaceIndices( left );
        float l1 = mesh->_vertices[ia].pos[axis];
        float l2 = mesh->_vertices[ib].pos[axis];
        float l3 = mesh->_vertices[ic].pos[axis];
        auto [ia2, ib2, ic2] = mesh->GetFaceIndices( right );
        float r1 = mesh->_vertices[ia2].pos[axis];
        float r2 = mesh->_vertices[ib2].pos[axis];
        float r3 = mesh->_vertices[ic2].pos[axis];
        return (l1 + l2 + l3) < (r1 + r2 + r3);
        } );

    std::vector<int> mLeftIndices( indices.cbegin(), indices.cbegin() + indices.size() / 2 + 1 );
    std::vector<int> mRightIndices( indices.cbegin() + indices.size() / 2 + 1, indices.cend() );
    if (!mLeftIndices.empty())
    {
        mLeft = std::make_unique<BVHNode_HalfEdgeMesh>( mLeftIndices, mesh );
    }
    if (!mRightIndices.empty())
    {
        mRight = std::make_unique<BVHNode_HalfEdgeMesh>( mRightIndices, mesh );
    }
}

void BVHNode_HalfEdgeMesh::Refit()
{
    for (const auto& i : mIndices)
    {
        auto indices = _mesh->GetFaceIndices( i );
        mBoundingBox.max_corner = glm::vec3( -FLT_MAX );
        mBoundingBox.min_corner = glm::vec3( FLT_MAX );
        mBoundingBox.Expand( _mesh->_vertices[std::get<0>( indices )].pos );
        mBoundingBox.Expand( _mesh->_vertices[std::get<1>( indices )].pos );
        mBoundingBox.Expand( _mesh->_vertices[std::get<2>( indices )].pos );
    }
    if (mLeft)
        mLeft->Refit();
    if (mRight)
        mRight->Refit();
}

bool BVHNode_HalfEdgeMesh::CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const
{
    if (LinesegAABBIntersect( p0, p1, mBoundingBox ))
    {
        if (mLeft && mRight)
        {
            bool left = mLeft->CheckLineseg( p0, p1, rec, id );
            bool right = mRight->CheckLineseg( p0, p1, rec, id );
            return left || right;
        }
        else
        {
            glm::vec3 d = p1 - p0;
            float len = glm::length( d );
            Ray r( p0, d );
            bool intersect = false;
            for (auto& idx : mIndices)
            {
                auto [ia, ib, ic] = _mesh->GetFaceIndices( idx );
                if (RayTriIntersect( r, _mesh->GetPosition( ia ), _mesh->GetPosition( ib ), _mesh->GetPosition( ic ), rec, -FLT_MAX, rec->t ))
                {
                    intersect = true;
                    *id = idx;
                }

            }
            return intersect;
        }
    }
    return false;
}

bool BVHNode_HalfEdgeMesh::CheckBall( glm::vec3 c, float r, std::vector<CollisionInfo>* colliders ) const
{
    return false;
}

bool BVHNode_HalfEdgeMesh::CheckRay( glm::vec3 o, glm::vec3 d, CollisionInfo* info, float* t ) const
{
    return false;
}

/* BVHTree_Metaball */
BVHTree_Metaball::BVHTree_Metaball( PBD::MetaballModel* model )
    :_model( model )
{
    std::vector<int> indices( model->BallList().size() );
    std::iota( indices.begin(), indices.end(), 0 );

    _root = std::make_unique<BVHNode_Metaball>( indices, _model );
}

void BVHTree_Metaball::Refit()
{
    _root->Refit();
}

bool BVHTree_Metaball::CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const
{
    return false;
}

bool BVHTree_Metaball::CheckBall( glm::vec3 c, float r, std::vector<CollisionInfo>* infos ) const
{
    return _root->CheckBall( c, r, infos );
}

bool BVHTree_Metaball::CheckRay( glm::vec3 o, glm::vec3 d, CollisionInfo* info ) const
{
    float t = FLT_MAX;
    bool intersect = _root->CheckRay( o, d, info, &t );
    return intersect;
}

/* BVHNode_Metaball */
BVHNode_Metaball::BVHNode_Metaball( std::vector<int> indices, PBD::MetaballModel* model )
    :_model( model )
{
    for (int i : indices)
    {
        mBoundingBox.Expand( _model->Ball( i ).x_pred + glm::vec3( _model->Ball( i ).r ) );
        mBoundingBox.Expand( _model->Ball( i ).x_pred - glm::vec3( _model->Ball( i ).r ) );
    }
    if (indices.size() <= BVHNode::THRESHOLD)
    {
        mIndices = indices;
        return;
    }

    unsigned int axis = mBoundingBox.LongestAxis();

    std::sort( indices.begin(), indices.end(), [&]( int left, int right )->bool {
        float l = _model->Ball( left ).x_pred[axis] - _model->Ball( left ).r;
        float r = _model->Ball( right ).x_pred[axis] - _model->Ball( right ).r;
        return l < r;
        } );

    std::vector<int> mLeftIndices( indices.cbegin(), indices.cbegin() + indices.size() / 2 + 1 );
    std::vector<int> mRightIndices( indices.cbegin() + indices.size() / 2 + 1, indices.cend() );
    if (!mLeftIndices.empty())
    {
        mLeft = std::make_unique<BVHNode_Metaball>( mLeftIndices, _model );
    }
    if (!mRightIndices.empty())
    {
        mRight = std::make_unique<BVHNode_Metaball>( mRightIndices, _model );
    }

    if (mLeft && mRight)
    {
        mIndices.clear();
    }
    else if (mLeft || mRight)
    {
        std::cout << "error" << std::endl;
    }
}

void BVHNode_Metaball::Refit()
{
    if (mLeft)
        mLeft->Refit();
    if (mRight)
        mRight->Refit();

    mBoundingBox.min_corner = glm::vec3( FLT_MAX );
    mBoundingBox.max_corner = glm::vec3( -FLT_MAX );
    if (!mLeft && !mRight)
    {
        for (int i : mIndices)
        {
            mBoundingBox.Expand( _model->Ball( i ).x_pred + glm::vec3( _model->Ball( i ).r ) );
            mBoundingBox.Expand( _model->Ball( i ).x_pred - glm::vec3( _model->Ball( i ).r ) );
        }
    }
    else
    {
        mBoundingBox.Expand( mLeft->BoundingBox().min_corner );
        mBoundingBox.Expand( mLeft->BoundingBox().max_corner );
        mBoundingBox.Expand( mRight->BoundingBox().min_corner );
        mBoundingBox.Expand( mRight->BoundingBox().max_corner );
    }
}

bool BVHNode_Metaball::CheckLineseg( glm::vec3 p0, glm::vec3 p1, IntersectionRec* rec, int* id ) const
{
    return false;
}

bool BVHNode_Metaball::CheckBall( glm::vec3 c, float r, std::vector<CollisionInfo>* infos ) const
{
    if (BallAABBIntersect( c, r, mBoundingBox ))
    {
        if (mLeft && mRight)
        {
            bool left = mLeft->CheckBall( c, r, infos );
            bool right = mRight->CheckBall( c, r, infos );
            return left || right;
        }
        else if (!mLeft && !mRight)
        {
            bool intersect = false;
            for (int i : mIndices)
            {
                float dist = r + _model->Ball( i ).r;
                float dist2 = glm::distance( c, _model->Ball( i ).x_pred );
                float h = dist - dist2;

                if (dist2 < dist)
                {
                    intersect = true;
                    glm::vec3 n = glm::normalize( _model->Ball( i ).x_pred - c );
                    infos->push_back( CollisionInfo{ _model->Ball( i ).x_pred - n * (_model->Ball( i ).r - h * 0.5f), -n, h * 0.5f, i } );
                    //infos->push_back( CollisionInfo{ _model->Ball( i ).x_pred - n * (_model->Ball( i ).r), -n, h, i } );
                }
            }
            return intersect;
        }
    }
    return false;
}

bool BVHNode_Metaball::CheckRay( glm::vec3 o, glm::vec3 d, CollisionInfo* info, float* t ) const
{
    if (RayAABBIntersect( Ray( o, d ), mBoundingBox, *t, nullptr ))
    {
        if (mLeft && mRight)
        {
            bool left = mLeft->CheckRay( o, d, info, t );
            bool right = mRight->CheckRay( o, d, info, t );
            return left || right;
        }
        else
        {
            bool intersect = false;

            for (int i : mIndices)
            {
                IntersectionRec rec;
                bool rayball_intersect = RayBallIntersect( Ray( o, d ), _model->Ball( i ).x, _model->Ball( i ).r, &rec, 0.f, *t );
                if (rayball_intersect)
                {
                    info->id = i;
                    info->n = rec.normal;
                    info->p = rec.p;
                    info->d = 0.f;
                    *t = rec.t;
                    intersect = true;
                }
            }

            return intersect;
        }
    }
    return false;
}

