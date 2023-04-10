#include "FreeCuttingManager.h"
#include <optional>
#include "../free_cutting/CuttingFace.h"
#include "../gl/VertexArray.h"
#include "../gl/VertexBuffer.h"
#include "../input/Input.h"
#include "../model/Ray.h"
#include "../util/Camera.h"
#include "../util/Intersection.h"
#include "../util/Shader.h"
#include "../util/util.h"
#include "../model/HalfEdgeSurfaceTester.h"
#include "../PBD/MetaballModel.h"

#define SPLIT_WIDTH 0.02f
#define ANCHOR_THRESHOLD 0.2f
#define DEPTH 0.2f

void FreeCuttingManager::Enable()
{
    mActive = true;
}

void FreeCuttingManager::Disable()
{
    mActive = false;
}

void FreeCuttingManager::ProcessRay()
{
    IntersectionRec rec;
    rec.t = std::numeric_limits<float>::max();
    int id = 0;

    glm::vec2 cursor = Input::GetMousePosition();
    cursor.y = Camera::current->_viewport_size.y - cursor.y;
    Ray ray = Camera::current->ScreenPointToRay( cursor );

    HalfEdgeSurfaceTester tester( mMesh.get() );

    if (tester.RayIntersect( ray, &rec, 0.f, rec.t, &id ))
    {
        if (mCutIndices.empty() || glm::distance( mLastRec.p, rec.p ) > 0.01f)
        {
            mCutting = true;
            ProcessIntersection( rec, id );
            mCurve.AddPoint( rec.p );
            mCurve.UpdateGPUMem();
        }
    }
}

void FreeCuttingManager::ProcessIntersection( IntersectionRec rec, int face )
{
    if (mLastFace == -1)
    {
        mLastFace = face;
        mLastRec = rec;
    }

    if (std::find( std::cbegin( mActiveTriangles ), std::cend( mActiveTriangles ), face ) != std::cend( mActiveTriangles ) ||
        mActiveTriangles.empty())
    {
        mActiveTriangles.clear();
        int idx = SplitTriangleByPoint( rec.p, face );
        mSplitDir.push_back( glm::normalize( glm::cross( rec.normal, rec.p - mLastRec.p ) ) );
        mCutDir.push_back( -rec.normal );
        mCutIndices.push_back( idx );
    }
    else
    {
        //Get Cutting Plane
        CuttingTool plane = CuttingTool( (rec.p + mLastRec.p) / 2.0f, glm::distance( rec.p, mLastRec.p ), 0.1f );
        plane.mTransform.LookAt( plane.mTransform.GetPosition() - rec.normal, glm::cross( glm::normalize( rec.p - mLastRec.p ), rec.normal ) );
        plane.Update();
        std::unordered_set<int> neigh_of_hit_tri = mMesh->GetNeighborFacesByFace_edgeonly( face );
        std::optional<int> neigh;
        for (int at : mActiveTriangles)
        {
            if (neigh_of_hit_tri.find( at ) != neigh_of_hit_tri.end())
                neigh = at;
        }
        if (neigh.has_value())
        {
            auto& edge = mMesh->_edges[mMesh->GetPublicEdge( face, neigh.value() )];
            int v_start = edge.vtx;
            int v_end = mMesh->_edges[edge.next].vtx;
            mActiveTriangles.clear();
            int new_v_in_tri = SplitTriangleByPoint( rec.p, face );
            LinesegIntersectInfo info{};
            LinesegPlaneIntersect( mMesh->GetPosition( v_start ), mMesh->GetPosition( v_end ),
                plane.mTransform.Up(), plane.mTransform.GetPosition(), &info );

            int new_v_on_edge = 0;
            mSplitDir.push_back( glm::normalize( glm::cross( rec.normal, rec.p - mLastRec.p ) ) );
            mCutDir.push_back( -mMesh->GetEdgeNormal( v_start, v_end ) );
            mMesh->SplitEdge( v_start, v_end, info.pos, &new_v_on_edge );
            mCutIndices.push_back( new_v_on_edge );

            mSplitDir.push_back( glm::normalize( glm::cross( rec.normal, rec.p - mLastRec.p ) ) );
            mCutDir.push_back( -rec.normal );
            mCutIndices.push_back( new_v_in_tri );
        }
        else
        {
            int cur_face = face;
            std::unordered_set<int> processed_tri;
            processed_tri.insert( cur_face );
            processed_tri.insert( mActiveTriangles.begin(), mActiveTriangles.end() );
            std::unordered_set<EdgePlaneIntersectRec, EdgePlaneIntersectRec::Hash,
                EdgePlaneIntersectRec::Pred> plane_intersect_set;

            do
            {
                std::unordered_set<int> neighbors = mMesh->GetNeighborFacesByFace_edgeonly( cur_face );
                bool found = false;
                for (int neighbor : neighbors)
                {
                    if (processed_tri.find( neighbor ) != processed_tri.end())
                    {
                        continue;
                    }
                    auto [ia, ib, ic] = mMesh->GetFaceIndices( neighbor );
                    int ids[3] = { ia, ib, ic };
                    std::vector<EdgePlaneIntersectRec> recs;
                    for (size_t i = 0; i < 3; i++)
                    {
                        int i0 = ids[i];
                        int i1 = ids[(i + 1) % 3];
                        LinesegIntersectInfo info;
                        if (LinesegCuttoolIntersection( mMesh->GetPosition( i0 ), mMesh->GetPosition( i1 ), plane, &info ))
                        {
                            EdgePlaneIntersectRec edge_rec( i0, i1, info.pos, neighbor );
                            recs.push_back( edge_rec );
                        }
                    }
                    if (recs.size() > 0)
                    {
                        plane_intersect_set.insert( recs.begin(), recs.end() );
                        processed_tri.insert( neighbor );
                        cur_face = neighbor;
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    break;
                }
            } while (true);

            mActiveTriangles.clear();
            int end_idx = SplitTriangleByPoint( rec.p, face );

            glm::vec3 split_dir = glm::normalize( glm::cross( rec.normal, rec.p - mLastRec.p ) );
            std::list<int> index_list;
            for (auto& itst : plane_intersect_set)
            {
                int ivtx;
                mCutDir.push_back( -mMesh->GetEdgeNormal( itst.istart, itst.iend ) );
                mSplitDir.push_back( split_dir );
                mMesh->SplitEdge( itst.istart, itst.iend, itst.pos, &ivtx );
                index_list.push_back( ivtx );
            }
            std::list<int> reordered_list;
            reordered_list.push_front( end_idx );
            while (!index_list.empty())
            {
                bool found = false;
                for (auto it = index_list.begin(); it != index_list.end(); it++)
                {
                    if (mMesh->_edge_map.find( { *it, reordered_list.front() } ) != mMesh->_edge_map.end())
                    {
                        reordered_list.push_front( *it );
                        index_list.erase( it );
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    std::cout << "Not found";
                }
            }
            mSplitDir.push_back( split_dir );
            mCutDir.push_back( -rec.normal );
            for (auto idx : reordered_list)
            {
                mCutIndices.push_back( idx );
            }
        }
    }
    mMesh->UpdateAttrBuffer();
    mMesh->UpdatePosBuffer();
    mLastFace = face;
    mLastRec = rec;
}

int FreeCuttingManager::SplitTriangleByPoint( glm::vec3 point, int tri )
{
    int new_idx = 0;
    auto [ia, ib, ic] = mMesh->SplitTriangle( tri, point, &new_idx );
    mActiveTriangles.push_back( ia );
    mActiveTriangles.push_back( ib );
    mActiveTriangles.push_back( ic );
    return new_idx;
}

void FreeCuttingManager::CalcDistField()
{
    if (mCutIndices.empty())
    {
        return;
    }

    mDistanceField.push_back( 0.f );
    auto prev = mCutIndices.begin();
    auto it = std::next( prev );
    for (; it != mCutIndices.end(); it++, prev++)
    {
        float seg_len = glm::distance( mMesh->GetPosition( *it ), mMesh->GetPosition( *prev ) );
        mDistanceField.push_back( seg_len + mDistanceField.back() );
    }
    float total_length = mDistanceField.back();
    float half_total = total_length * 0.5f;
    for (auto& d : mDistanceField)
    {
        d = 1 - glm::abs( d - half_total ) / half_total;
        d = glm::sqrt( d );
    }
}

void FreeCuttingManager::SplitCutLine()
{
    if (mCutIndices.size() < 3)
    {
        return;
    }
    std::cout << "Start Spliting Cut Line" << std::endl;
    auto prev = mCutIndices.begin();
    auto it = std::next( prev );
    auto next = std::next( it );

    mCutTracePairs.push_back( { mCutIndices.front(), mCutIndices.front() } );
    std::pair<int, int> last_pair = { -1, -1 };
    while (next != mCutIndices.end())
    {
        int iprev;
        if (last_pair.second == -1)
        {
            iprev = *prev;
        }
        else
        {
            iprev = last_pair.second;
        }
        std::pair<int, int> pair = mMesh->SplitVertex( *it, iprev, *next );
        mCutTracePairs.push_back( pair );
        prev++;
        it++;
        next++;
        last_pair = pair;
    }
    mCutTracePairs.push_back( { mCutIndices.back(), mCutIndices.back() } );
    std::cout << "Spliting Cut Face Success";
}

void FreeCuttingManager::MoveCutTracePairs()
{
    auto dir_it = mSplitDir.begin();
    auto dist_it = mDistanceField.begin();
    for (auto it = mCutTracePairs.begin(); it != mCutTracePairs.end(); it++, dir_it++, dist_it++)
    {
        auto pair = *it;
        int p_upper = pair.first;
        int p_lower = pair.second;
        if (p_upper == p_lower)
        {
            continue;
        }
        mMesh->SetPosition( p_upper, mMesh->GetPosition( p_upper ) + SPLIT_WIDTH * (*dir_it) * (*dist_it) );
        mMesh->_vertices[p_upper].rest_pos = mMesh->GetPosition( p_upper );
        mMesh->SetPosition( p_lower, mMesh->GetPosition( p_lower ) - SPLIT_WIDTH * (*dir_it) * (*dist_it) );
        mMesh->_vertices[p_lower].rest_pos = mMesh->GetPosition( p_lower );
    }
}

void FreeCuttingManager::BuildCutFace()
{
#define XOFFSET 2.0f
    std::cout << "Start Building Face" << std::endl;
    auto it = mCutTracePairs.begin();
    auto next = std::next( it );
    auto dir_next = std::next( mCutDir.begin() );
    auto dist_next = std::next( mDistanceField.begin() );

    int istart = mCutTracePairs.front().first;
    int iend = mCutTracePairs.back().first;
    int new_vtx = mMesh->AddVertex( (mMesh->GetPosition( next->first ) + mMesh->GetPosition( next->second )) * 0.5f );
    mPostAdjust.push_back( { new_vtx, *dir_next * DEPTH * *dist_next } );
    mAnchors.insert( new_vtx );
    int newf1 = mMesh->AddTriangle( istart, new_vtx, next->first );
    int newf2 = mMesh->AddTriangle( istart, next->second, new_vtx );
    auto [newf1e0, newf1e1, newf1e2] = mMesh->GetFaceEdges( newf1 );
    mMesh->SetTexCoord( newf1e0, glm::vec2( XOFFSET + 0.0f, 0.0f ) );
    mMesh->SetTexCoord( newf1e1, glm::vec2( XOFFSET + *dist_next, *dist_next ) );
    mMesh->SetTexCoord( newf1e2, glm::vec2( XOFFSET + *dist_next, 0 ) );
    auto [newf2e0, newf2e1, newf2e2] = mMesh->GetFaceEdges( newf2 );
    mMesh->SetTexCoord( newf2e0, glm::vec2( XOFFSET + 0.0f, 0.0f ) );
    mMesh->SetTexCoord( newf2e1, glm::vec2( XOFFSET + *dist_next, 0 ) );
    mMesh->SetTexCoord( newf2e2, glm::vec2( XOFFSET + *dist_next, *dist_next ) );

    it++;
    next++;
    dir_next++;
    dist_next++;

    int last_vtx = new_vtx;
    auto stop_pos = std::prev( std::end( mCutTracePairs ) );
    while (next != stop_pos)
    {
        int new_vtx = mMesh->AddVertex( (mMesh->GetPosition( next->first ) + mMesh->GetPosition( next->second )) * 0.5f + *dir_next * DEPTH * *dist_next );
        int f0 = mMesh->AddTriangle( new_vtx, next->first, it->first );
        int f1 = mMesh->AddTriangle( new_vtx, it->first, last_vtx );
        int f2 = mMesh->AddTriangle( new_vtx, last_vtx, it->second );
        int f3 = mMesh->AddTriangle( new_vtx, it->second, next->second );
        mPostAdjust.push_back( { new_vtx, *dir_next * DEPTH * *dist_next } );
        mAnchors.insert( new_vtx );
        glm::vec2 t_upper0( XOFFSET + *std::prev( dist_next ), 0 );
        glm::vec2 t_upper1( XOFFSET + *dist_next, 0 );
        glm::vec2 t_lower0( XOFFSET + *std::prev( dist_next ), *dist_next );
        glm::vec2 t_lower1( XOFFSET + *dist_next, *dist_next );

        auto [f1e0, f1e1, f1e2] = mMesh->GetFaceEdges( f0 );
        mMesh->SetTexCoord( f1e0, t_lower1 );
        mMesh->SetTexCoord( f1e1, t_upper1 );
        mMesh->SetTexCoord( f1e2, t_upper0 );
        auto [f2e0, f2e1, f2e2] = mMesh->GetFaceEdges( f1 );
        mMesh->SetTexCoord( f2e0, t_lower1 );
        mMesh->SetTexCoord( f2e1, t_upper0 );
        mMesh->SetTexCoord( f2e2, t_lower0 );
        auto [f3e0, f3e1, f3e2] = mMesh->GetFaceEdges( f2 );
        mMesh->SetTexCoord( f3e0, t_lower1 );
        mMesh->SetTexCoord( f3e1, t_lower0 );
        mMesh->SetTexCoord( f3e2, t_upper0 );
        auto [f4e0, f4e1, f4e2] = mMesh->GetFaceEdges( f3 );
        mMesh->SetTexCoord( f4e0, t_lower1 );
        mMesh->SetTexCoord( f4e1, t_upper0 );
        mMesh->SetTexCoord( f4e2, t_upper1 );

        it++;
        next++;
        dir_next++;
        dist_next++;
        last_vtx = new_vtx;
    }

    mMesh->AddTriangle( iend, it->first, last_vtx );
    mMesh->AddTriangle( iend, last_vtx, it->second );
    std::cout << "Building Face Success" << std::endl;
#undef XOFFSET
}

void FreeCuttingManager::SetMesh( std::shared_ptr<HalfEdgeMesh> mesh )
{
    mMesh = mesh;
}

void FreeCuttingManager::SetMetaballModel( PBD::MetaballModel* model )
{
    mMetaballModel = model;
}

void FreeCuttingManager::SetTetraModel( PBD::PBDTetraModel* model )
{
    mTetModel = model;
}

void FreeCuttingManager::Update()
{
    if (!mActive)
    {
        return;
    }
    if (mCutting && Input::IsMouseButtonHeld( Input::MouseButton::Left ))
    {
        ProcessRay();
        mMesh->UpdatePosBuffer();
        mMesh->UpdateAttrBuffer();
        //mMetaballModel->CreateSurfaceMapping();
    }
    else if (!mCutting && Input::IsMouseButtonDown( Input::MouseButton::Left ))
    {
        //timer = clock::now();
        mCutIndices.clear();
        mDistanceField.clear();
        mCutTracePairs.clear();
        mSplitDir.clear();
        mCutDir.clear();
        mActiveTriangles.clear();
        mLastFace = -1;
        mCutting = true;
    }
    else if (mCutting && Input::IsMouseButtonReleased( Input::MouseButton::Left ))
    {
        mCurve.Clear();
        mCutting = false;
        CalcDistField();
        SplitCutLine();
        BuildCutFace();

        MoveCutTracePairs();
        /*std::vector<glm::vec3> oldPoints;
        for (auto& p : mMesh->_vertices)
        {
            oldPoints.push_back( p.rest_pos );
        }
        MoveCutTracePairs();
        std::vector<glm::vec3> newPoints;
        for (auto& p : mMesh->_vertices)
        {
            newPoints.push_back( p.rest_pos );
        }

        mLaplaceDeformer.SetMesh( mMesh );
        mLaplaceDeformer.SetNewPoints( newPoints );
        for (int i = 0, s = oldPoints.size(); i < s; i++)
        {
            mMesh->_vertices[i].pos = oldPoints[i];
            mMesh->_vertices[i].rest_pos = oldPoints[i];
        }

        for (int i = 0; i < mMesh->_vertices.size(); i++)
        {
            glm::vec3 p = mMesh->_vertices[i].rest_pos;
            bool anchor = true;
            for (auto& pair : mCutTracePairs)
            {
                glm::vec3 p1 = mMesh->_vertices[pair.first].rest_pos;
                glm::vec3 p2 = mMesh->_vertices[pair.second].rest_pos;
                float dist = glm::distance( p, p1 );
                if (dist < ANCHOR_THRESHOLD)
                {
                    anchor = false;
                    break;
                }
            }
            if (anchor)
            {
                mAnchors.insert( i );
            }
        }

        std::vector<int> anchors = std::vector<int>( mAnchors.begin(), mAnchors.end() );

        auto transformed_points = mLaplaceDeformer.Transform( anchors );
        for (int i = 0, size = transformed_points.size(); i < size; i++)
        {
            mMesh->_vertices[i].rest_pos = transformed_points[i];
            mMesh->_vertices[i].pos = transformed_points[i];
        }

        for (int anchor : mAnchors)
        {
            mMesh->_vertices[anchor].rest_pos = newPoints[anchor];
            mMesh->_vertices[anchor].pos = newPoints[anchor];
        }

        for (auto pair : mPostAdjust)
        {
            mMesh->_vertices[pair.first].rest_pos += pair.second;
            mMesh->_vertices[pair.first].pos += pair.second;
        }*/
        mMesh->UpdateNormal();
        mMesh->UpdateAttrBuffer();
        mMesh->UpdatePosBuffer();
        if (mMetaballModel)
        {
            mMetaballModel->CreateSurfaceMapping();
        }
        if (mTetModel)
        {
            mTetModel->InitSurfaceMapping( false );
        }
    }

}

void FreeCuttingManager::Draw()
{
    if (!mActive)
    {
        return;
    }
    mCurve.Draw();
}

SurfaceCurve::SurfaceCurve()
{
    mVAO = std::make_shared<VertexArray>();
    mVBO = std::make_shared<VertexBuffer>( nullptr, 0 );
    VertexBufferLayout layout;
    layout.Push( GL_FLOAT, 3, 0 );
    mVBO->SetLayout( layout );
    mVAO->AddBuffer( *mVBO );
}

void SurfaceCurve::AddPoint( glm::vec3 point )
{
    mPoints.push_back( point );
}

void SurfaceCurve::Clear()
{
    mPoints.clear();
}

void SurfaceCurve::UpdateGPUMem()
{
    mVBO->UpdateData( mPoints.data(), mPoints.size() * sizeof( glm::vec3 ) );
}

void SurfaceCurve::Update()
{

}

void SurfaceCurve::Draw()
{
    mVAO->Bind();
    auto shader = Shader::Find( "line" );
    shader->use();
    shader->setVec( "uColor", glm::vec3( 0.5, 0.5, 1.0 ) );
    shader->setMat( "uModelMat", mTransform.GetModelMat() );
    shader->setMat( "uViewMat", Camera::current->GetViewMatrix() );
    shader->setMat( "uProjectionMat", Camera::current->GetProjectionMatrix() );
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    glLineWidth( 5.0f );
    glDrawArrays( GL_LINE_STRIP, 0, mPoints.size() );
    glLineWidth( 1.0f );
    shader->setVec( "uColor", glm::vec3( 1.0, 0.0, 0.0 ) );
    glPointSize( 5.0f );
    glDrawArrays( GL_POINTS, 0, mPoints.size() );
    glPointSize( 1.0f );
    glDisable( GL_POLYGON_OFFSET_LINE );
}

void FreeCuttingTool::Update()
{
}

void FreeCuttingTool::Draw()
{
    mMesh->Draw();
}
