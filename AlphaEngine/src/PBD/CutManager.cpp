#include "CutManager.h"
#include "../util/Intersection.h"
#include "../input/Input.h"
#include "../util/util.h"
#include "../util/Fade.h"


CutManager::CutManager( HalfEdgeMesh* surface, PBD::ScalpelRect* scalpel )
    :_surface( surface ), _scalpel( scalpel )
{
    _surface_tester.SetSurface( surface );
}

HalfEdgeMesh* CutManager::GetSurface()
{
    return _surface;
}

PBD::ScalpelRect* CutManager::GetScalpel()
{
    return _scalpel;
}

void CutManager::CutMesh()
{
    if (!_surface || !_scalpel)
    {
        return;
    }
    if (!_scalpel->GetTotalRect().has_value())
    {
        return;
    }
    std::cout << "Start Cut Surface" << std::endl;
    Rect rect = _scalpel->GetTotalRect().value();
    glm::vec3 LU = rect.LeftUp();
    glm::vec3 RU = rect.RightUp();
    glm::vec3 LD = rect.LeftDown();
    glm::vec3 RD = rect.RightDown();

    std::unordered_set<IntersectInfo, IntersectInfo::Hash, IntersectInfo::Pred> intersect_infos;
    bool point_on_lineseg[4] = { false, false, false, false };
    for (size_t i = 0, s = _surface->GetFaceNumber(); i < s; i++)
    {
        auto& f = _surface->_faces[i];
        if (f.edge == -1)
        {
            continue;
        }
        auto [i0, i1, i2] = _surface->GetFaceIndices( i );
        glm::vec3 p0 = _surface->GetPosition( i0 );
        glm::vec3 p1 = _surface->GetPosition( i1 );
        glm::vec3 p2 = _surface->GetPosition( i2 );
        glm::vec3 pos;
        glm::vec3 normal;
        if (LinesegRectIntersect( p0, p1, rect, &pos ))
        {
            intersect_infos.insert( IntersectInfo( i0, i1, pos, IntersectInfo::Type::EdgeSplit ) );
        }
        if (LinesegRectIntersect( p1, p2, rect, &pos ))
        {
            intersect_infos.insert( IntersectInfo( i1, i2, pos, IntersectInfo::Type::EdgeSplit ) );
        }
        if (LinesegRectIntersect( p2, p0, rect, &pos ))
        {
            intersect_infos.insert( IntersectInfo( i2, i0, pos, IntersectInfo::Type::EdgeSplit ) );
        }
        if (LinesegTriIntersect( LD, LU, p0, p1, p2, &pos, &normal ))
        {
            intersect_infos.insert( IntersectInfo( i, i, pos, IntersectInfo::Type::TriSplit ) );
            point_on_lineseg[0] = true;
        }
        if (LinesegTriIntersect( RD, LD, p0, p1, p2, &pos, &normal ))
        {
            intersect_infos.insert( IntersectInfo( i, i, pos, IntersectInfo::Type::TriSplit ) );
            point_on_lineseg[1] = true;
        }
        if (LinesegTriIntersect( RU, RD, p0, p1, p2, &pos, &normal ))
        {
            intersect_infos.insert( IntersectInfo( i, i, pos, IntersectInfo::Type::TriSplit ) );
            point_on_lineseg[2] = true;
        }
        if (LinesegTriIntersect( LU, RU, p0, p1, p2, &pos, &normal ))
        {
            intersect_infos.insert( IntersectInfo( i, i, pos, IntersectInfo::Type::TriSplit ) );
            point_on_lineseg[3] = true;
        }
    }
    std::list<int> new_vertices;
    for (auto& info : intersect_infos)
    {
        if (info.type == IntersectInfo::Type::TriSplit)
        {
            int newv = -1;
            _surface->SplitTriangle( info.iv0, info.p, &newv );
            new_vertices.push_back( newv );
        }
    }

    for (auto& info : intersect_infos)
    {
        if (info.type == IntersectInfo::Type::EdgeSplit)
        {
            int newv = -1;
            _surface->SplitEdge( info.iv0, info.iv1, info.p, &newv );
            new_vertices.push_back( newv );
        }
    }


    //Get Trim Loops
    std::vector<std::list<int>> trim_loops;
    while (!new_vertices.empty())
    {
        std::list<int> trim_loop;
        trim_loop.push_back( new_vertices.front() );
        new_vertices.pop_front();
        bool found = true;

        while (found)
        {
            found = false;
            for (auto it = new_vertices.begin(); it != new_vertices.end();)
            {
                if (_surface->_edge_map.find( { *it, trim_loop.front() } ) != _surface->_edge_map.end())
                {
                    trim_loop.push_front( *it );
                    it = new_vertices.erase( it );
                    found = true;
                    break;
                }
                else if (_surface->_edge_map.find( { trim_loop.back(), *it } ) != _surface->_edge_map.end())
                {
                    trim_loop.push_back( *it );
                    it = new_vertices.erase( it );
                    found = true;
                    break;
                }
                else
                {
                    it++;
                }
            }
        }
        trim_loops.push_back( std::move( trim_loop ) );
    }

    std::vector<std::list<glm::vec3>> point_loops;
    for (const std::list<int>& trim_loop : trim_loops)
    {
        std::list<glm::vec3> points;
        for (int id : trim_loop)
        {
            points.push_back( _surface->GetPosition( id ) );
        }
        point_loops.push_back( std::move( points ) );
    }

    std::vector<std::list<int>> trim_loops_upper;
    std::vector<std::list<int>> trim_loops_lower;
    for (auto& trim_loop : trim_loops)
    {
        std::list<int> upper_list;
        std::list<int> lower_list;

        auto prev = trim_loop.begin();
        auto it = std::next( prev );
        if (it == trim_loop.end())
        {
            continue;
        }
        auto next = std::next( it );
        if (next == trim_loop.end())
        {
            continue;
        }
        bool upsidedown = glm::dot( rect.mTransform.Forward(), glm::cross( rect.Normal(), _surface->GetPosition( *it ) - _surface->GetPosition( *prev ) ) ) < 0;

        std::pair<int, int> last_pair = { -1, -1 };

        upper_list.push_back( *prev );
        lower_list.push_back( *prev );

        for (; next != trim_loop.end(); it++, next++)
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
            last_pair = _surface->SplitVertex( *it, iprev, *next );
            upper_list.push_back( last_pair.first );
            lower_list.push_back( last_pair.second );
        }
        upper_list.push_back( trim_loop.back() );
        lower_list.push_back( trim_loop.back() );

        if (upsidedown)
        {
            trim_loops_upper.push_back( lower_list );
            trim_loops_lower.push_back( upper_list );
        }
        else
        {
            trim_loops_upper.push_back( upper_list );
            trim_loops_lower.push_back( lower_list );
        }
    }

    std::vector<int> new_points_upper;
    std::vector<int> new_points_lower;
    for (auto& trim_loop : trim_loops_upper)
    {
        for (int i : trim_loop)
        {
            new_points_upper.push_back( i );
        }
    }
    for (auto& trim_loop : trim_loops_lower)
    {
        for (int i : trim_loop)
        {
            new_points_lower.push_back( i );
        }
    }

    //TEMP
    std::vector<glm::vec3> all_points;
    std::vector<glm::vec2> all_points2d;
    all_points.insert( all_points.end(), point_loops[0].begin(), point_loops[0].end() );
    //for (auto& point_loop : point_loops)
    //{
    //}
    int corner_count = 0;
    for (glm::vec3 corner : {LU, RU, RD, LD})
    {
        if (_surface_tester.PointIsInSurface( corner ))
        {
            all_points.push_back( corner );
            corner_count++;
        }
    }

    glm::vec3 u = glm::normalize( RU - LU );
    glm::vec3 v = glm::normalize( LD - LU );
    for (auto& p3d : all_points)
    {
        glm::vec2 p2d( glm::dot( u, p3d - LU ), glm::dot( v, p3d - LU ) );
        all_points2d.push_back( p2d );
    }

    std::vector<Triangle> out_triangles;
    std::vector<glm::vec2> out_points2d;

    Triangulation2d( all_points2d, &out_triangles, &out_points2d );
    std::vector<Triangle> out_triangles_lower = out_triangles;

    size_t idx_start = _surface->_vertices.size();
    for (auto& tri : out_triangles)
    {
        tri.a += idx_start;
        tri.b += idx_start;
        tri.c += idx_start;
    }
    for (int i = 0; i < out_points2d.size(); i++)
    {
        glm::vec2& p2d = out_points2d[i];
        glm::vec3 p3d = p2d.x * u + p2d.y * v + LU;
        for (int newv : new_points_upper)
        {
            if (glm::TwoPointsEqual( _surface->GetPosition( newv ), p3d, 1e-5f ))
            {
                for (auto& tri : out_triangles)
                {
                    for (int v = 0; v < 3; v++)
                    {
                        if (tri[v] == i + idx_start)
                        {
                            tri[v] = newv;
                        }
                    }
                }
            }
        }
        int v = _surface->AddVertex( p3d );
        _surface->_vertices[v].color = glm::vec3( 0, 1, 0 );
    }
    for (auto& tri : out_triangles)
    {
        int face = _surface->AddTriangle( tri.a, tri.b, tri.c );
        _surface->_vertices[tri.a].topo += 1;
        _surface->_vertices[tri.b].topo += 1;
        _surface->_vertices[tri.c].topo += 1;

        auto [e0, e1, e2] = _surface->GetFaceEdges( face );
        glm::vec2 tex0 = glm::vec2( glm::dot( u, _surface->GetPosition( tri.a ) - LU ) / rect.Width(),
            glm::dot( v, _surface->GetPosition( tri.a ) - LU ) / rect.Height() );
        glm::vec2 tex1 = glm::vec2( glm::dot( u, _surface->GetPosition( tri.b ) - LU ) / rect.Width(),
            glm::dot( v, _surface->GetPosition( tri.b ) - LU ) / rect.Height() );
        glm::vec2 tex2 = glm::vec2( glm::dot( u, _surface->GetPosition( tri.c ) - LU ) / rect.Width(),
            glm::dot( v, _surface->GetPosition( tri.c ) - LU ) / rect.Height() );

        _surface->SetTexCoord( e0, glm::vec2( 2.0, 0 ) + tex0 );
        _surface->SetTexCoord( e1, glm::vec2( 2.0, 0 ) + tex1 );
        _surface->SetTexCoord( e2, glm::vec2( 2.0, 0 ) + tex2 );
        _surface->SetNormal( e0, rect.Normal() );
        _surface->SetNormal( e1, rect.Normal() );
        _surface->SetNormal( e2, rect.Normal() );
        //std::cout << tri.a << ", " << tri.b << " , " << tri.c << "\n";
    }

    size_t idx_start2 = _surface->_vertices.size();
    for (auto& tri : out_triangles_lower)
    {
        tri.a += idx_start2;
        tri.b += idx_start2;
        tri.c += idx_start2;
    }
    for (int i = 0; i < out_points2d.size(); i++)
    {
        glm::vec2& p2d = out_points2d[i];
        glm::vec3 p3d = p2d.x * u + p2d.y * v + LU;
        for (int newv : new_points_lower)
        {
            if (glm::TwoPointsEqual( _surface->GetPosition( newv ), p3d, 1e-5f ))
            {
                for (auto& tri : out_triangles_lower)
                {
                    for (int v = 0; v < 3; v++)
                    {
                        if (tri[v] == i + idx_start2)
                        {
                            tri[v] = newv;
                        }
                    }
                }
            }
        }
        bool recalc_x0 = false;
        if (glm::TwoPointsEqual( p3d, LU ) || glm::TwoPointsEqual( p3d, LD ) ||
            glm::TwoPointsEqual( p3d, RU ) || glm::TwoPointsEqual( p3d, RD ))
        {
            recalc_x0 = true;
            for (auto& tri : out_triangles_lower)
            {
                for (int v = 0; v < 3; v++)
                {
                    if (tri[v] == i + idx_start2)
                    {
                        tri[v] = i + idx_start;
                        _surface->_vertices[i + idx_start].recal_x0 = recalc_x0;
                    }
                }
            }
        }
        int v = _surface->AddVertex( p3d );
        _surface->_vertices[v].color = glm::vec3( 0, 1, 0 );
    }

    for (auto& tri : out_triangles_lower)
    {
        int face = _surface->AddTriangle( tri.a, tri.c, tri.b );
        _surface->_vertices[tri.a].topo += -1;
        _surface->_vertices[tri.b].topo += -1;
        _surface->_vertices[tri.c].topo += -1;

        auto [e0, e1, e2] = _surface->GetFaceEdges( face );
        glm::vec2 tex0 = glm::vec2( glm::dot( u, _surface->GetPosition( tri.a ) - LU ) / rect.Width(),
            glm::dot( v, _surface->GetPosition( tri.a ) - LU ) / rect.Height() );
        glm::vec2 tex1 = glm::vec2( glm::dot( u, _surface->GetPosition( tri.b ) - LU ) / rect.Width(),
            glm::dot( v, _surface->GetPosition( tri.b ) - LU ) / rect.Height() );
        glm::vec2 tex2 = glm::vec2( glm::dot( u, _surface->GetPosition( tri.c ) - LU ) / rect.Width(),
            glm::dot( v, _surface->GetPosition( tri.c ) - LU ) / rect.Height() );
        _surface->SetTexCoord( e0, glm::vec2( 2.0, 0 ) + tex0 );
        _surface->SetTexCoord( e1, glm::vec2( 2.0, 0 ) + tex1 );
        _surface->SetTexCoord( e2, glm::vec2( 2.0, 0 ) + tex2 );
        _surface->SetNormal( e0, -rect.Normal() );
        _surface->SetNormal( e1, -rect.Normal() );
        _surface->SetNormal( e2, -rect.Normal() );
    }

    /* Decide Sharp Edge */
    for (auto it = new_points_upper.begin(), next = std::next( it ); next != new_points_upper.end(); it++, next++)
    {
        _surface->_edges[_surface->_edge_map[{*it, * next}]].sharp_edge = true;
        _surface->_edges[_surface->_edge_map[{*next, * it}]].sharp_edge = true;
    }
    for (auto it = new_points_lower.rbegin(), next = std::next( it ); next != new_points_lower.rend(); it++, next++)
    {
        _surface->_edges[_surface->_edge_map[{*it, * next}]].sharp_edge = true;
        _surface->_edges[_surface->_edge_map[{*next, * it}]].sharp_edge = true;
    }

    /* Show Upper and Lower points */
    //for (int i = 0; i < _surface->_vertices.size(); i++)
    //{
    //    if (_surface->_vertices[i].topo >= 1)
    //    {
    //        _surface->SetVtxColor( i, glm::vec3( 1, 0, 0 ) );
    //    }
    //    else if (_surface->_vertices[i].topo <= -1)
    //    {
    //        _surface->SetVtxColor( i, glm::vec3( 0, 1, 0 ) );
    //    }
    //}


    //for (int i = 0; i < _surface->GetVertexNumber(); i++)
    //{
    //    if (_surface->_vertices[i].edge == -1)
    //        continue;
    //    try
    //    {
    //        _surface->GetNeighborEdges( i );
    //    }
    //    catch (std::runtime_error e)
    //    {
    //        _surface->_vertices[i].color = glm::vec3( 1, 0, 0 );
    //    }
    //}

    std::cout << "Finish Cut Surface" << std::endl;
    return;
}
