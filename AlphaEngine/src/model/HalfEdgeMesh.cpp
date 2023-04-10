#include "HalfEdgeMesh.h"
#include <algorithm>
#include "acceleration/AABB.h"
#include "gl/ShaderStorageBuffer.h"
#include "input/Input.h"
#include "util/Intersection.h"
#include "util/Shader.h"
#include "util/Camera.h"
#include "util/util.h"
#include "material/Material.h"
#include "lighting/Light.h"
#include "HalfEdgeSurfaceTester.h"
#include "tinyobjloader/tiny_obj_loader.h"

HalfEdgeMesh::HalfEdgeMesh()
{
    InitOpenglObjects();
}

HalfEdgeMesh::HalfEdgeMesh( const std::string& path )
{
    InitOpenglObjects();
    LoadOBJ( path );
}

void HalfEdgeMesh::LoadOBJ( const std::string& path )
{
    _faces.clear();
    _edges.clear();
    _edge_map.clear();
    _vertices.clear();

    tinyobj::ObjReaderConfig reader_config;
    reader_config.triangulate = true;

    std::string dir = "";
    size_t pos = path.find_last_of( '/' );
    if (pos != std::string::npos)
    {
        dir = path.substr( 0, pos + 1 );
    }
    else if ((pos = path.find_last_of( '\\' )) != std::string::npos)
    {
        dir = path.substr( 0, pos + 1 );
    }
    else
    {
        dir = "./";
    }

    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile( path, reader_config )) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
        }
        return;
    }
    if (!reader.Warning().empty()) {
        std::cout << "TinyObjReader: " << reader.Warning();
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();
    if (shapes.empty())
    {
        return;
    }

    for (int i = 0; i < attrib.vertices.size() / 3; i++)
    {
        const tinyobj::real_t vx = attrib.vertices[i * 3 + 0];
        const tinyobj::real_t vy = attrib.vertices[i * 3 + 1];
        const tinyobj::real_t vz = attrib.vertices[i * 3 + 2];
        AddVertex( glm::vec3( vx, vy, vz ) );
    }

    auto& shape = shapes[0];
    for (int f = 0; f < shape.mesh.num_face_vertices.size(); f++)
    {
        // Loop over vertices in the face.
        const tinyobj::index_t idx0 = shape.mesh.indices[f * 3 + 0];
        const tinyobj::index_t idx1 = shape.mesh.indices[f * 3 + 1];
        const tinyobj::index_t idx2 = shape.mesh.indices[f * 3 + 2];

        const int iface = AddTriangle( idx0.vertex_index, idx1.vertex_index, idx2.vertex_index );
        auto [ea, eb, ec] = GetFaceEdges( iface );
        const int eidx[3] = { ea, eb, ec };

        for (int v = 0; v < 3; v++) {
            // access to vertex
            tinyobj::index_t idx = shape.mesh.indices[f * 3 + v];

            // Check if `normal_index` is zero or positive. negative = no normal data
            if (idx.normal_index >= 0)
            {
                tinyobj::real_t nx = attrib.normals[3 * idx.normal_index + 0];
                tinyobj::real_t ny = attrib.normals[3 * idx.normal_index + 1];
                tinyobj::real_t nz = attrib.normals[3 * idx.normal_index + 2];
                _edges[eidx[v]].normal = glm::vec3( nx, ny, nz );
            }

            // Check if `texcoord_index` is zero or positive. negative = no texcoord data
            if (idx.texcoord_index >= 0)
            {
                tinyobj::real_t tx = attrib.texcoords[2 * idx.texcoord_index + 0];
                tinyobj::real_t ty = attrib.texcoords[2 * idx.texcoord_index + 1];
                _edges[eidx[v]].texcoord = glm::vec2( tx, ty );
            }
        }
    }

    const static auto pt2vec3 = []( const tinyobj::real_t* p ) -> glm::vec3
    {
        return glm::vec3( p[0], p[1], p[2] );
    };
    const static auto pt2vec2 = []( const tinyobj::real_t* p ) -> glm::vec2
    {
        return glm::vec2( p[0], p[1] );
    };
    MaterialInfos main_info;
    if (!materials.empty())
    {
        const auto& main_mat = materials[0];
        main_info.alpha = main_mat.transmittance[0];
        main_info.diffuseColor = pt2vec3( main_mat.diffuse );
        main_info.specularColor = pt2vec3( main_mat.specular );
        main_info.diffuseTexPath = main_mat.diffuse_texname;
        main_info.specularTexPath = main_mat.specular_texname;
        main_info.normalTexPath = main_mat.bump_texname;
        main_info.roughnessTexPath = main_mat.roughness_texname;
        main_info.metallicTexPath = main_mat.metallic_texname;
        main_info.metallic = main_mat.metallic;
        main_info.roughness = main_mat.roughness;
        main_info.shininess = main_mat.shininess;
    }

    SetMainMaterial( std::make_shared<Material>( dir, "material", main_info ) );

    UpdateAttrBuffer();
    UpdatePosBuffer();
}

void HalfEdgeMesh::InitOpenglObjects()
{
    _vao = std::make_unique<VertexArray>();
    _point_vbo = std::make_unique<VertexBuffer>( nullptr, 0 );
    _attr_vbo = std::make_unique<VertexBuffer>( nullptr, 0 );

    VertexBufferLayout pos_layout;
    VertexBufferLayout attr_layout;
    pos_layout.Push( GL_FLOAT, 3, 0 );                      //pos
    attr_layout.Push( GL_FLOAT, 3, offsetof( VertexAttr, normal ) ); //normal
    attr_layout.Push( GL_FLOAT, 2, offsetof( VertexAttr, texcoord ) ); //texcoord
    attr_layout.Push( GL_FLOAT, 3, offsetof( VertexAttr, color ) ); //color
    attr_layout.Push( GL_FLOAT, 3, offsetof( VertexAttr, tangent ) ); //tangent
    attr_layout.Push( GL_INT, 1, offsetof( VertexAttr, index ) );

    _point_vbo->SetLayout( pos_layout );
    _attr_vbo->SetLayout( attr_layout );
    _vao->AddBuffer( *_point_vbo );
    _vao->AddBuffer( *_attr_vbo );
}

void HalfEdgeMesh::SetMainMaterial( const std::shared_ptr<Material>& mat )
{
    _material_main = mat;
}

void HalfEdgeMesh::SetSubMaterial( const std::shared_ptr<Material>& mat )
{
    _material_sub = mat;
    _material_sub->UpdateTextureUnits( _material_main->GetTextureUnitUsage() );
}

void HalfEdgeMesh::SetVtxNormal( unsigned vtx, const glm::vec3& normal )
{
    for (int e : GetNeighborEdges( vtx ))
    {
        _edges[e].normal = normal;
    }
}

void HalfEdgeMesh::SetVtxTexCoord( unsigned vtx, const glm::vec2& texcoord )
{
    for (int e : GetNeighborEdges( vtx ))
    {
        _edges[e].texcoord = texcoord;
    }
}

void HalfEdgeMesh::SetVtxTangent( unsigned vtx, const glm::vec3& tangent )
{
    for (int e : GetNeighborEdges( vtx ))
    {
        _edges[e].tangent = tangent;
    }
}

void HalfEdgeMesh::SetVtxColor( unsigned vtx, const glm::vec3& color )
{
    _vertices[vtx].color = color;
}

void HalfEdgeMesh::SetFaceColor( int face, const glm::vec3& color )
{
    auto [ia, ib, ic] = GetFaceIndices( face );
    _vertices[ia].color = color;
    _vertices[ib].color = color;
    _vertices[ic].color = color;
}

bool HalfEdgeMesh::CheckTexcoord() const
{
    int i = 0;
    for (const Edge& e : _edges)
    {
        if (e.texcoord.x > 1.f || e.texcoord.x < 0.f || e.texcoord.y > 1.f || e.texcoord.y < 0.f)
        {
            assert( false );
            return false;
        }
        i++;
    }
}

bool HalfEdgeMesh::CheckTexcoord( int f ) const
{
    int ie0 = _faces[f].edge;
    int ie1 = GetNextEdge( ie0 );
    int ie2 = GetNextEdge( ie1 );
    for (int i : {ie0, ie1, ie2})
    {
        const glm::vec2& texc = _edges[i].texcoord;
        if (texc.x > 1.f || texc.x < 0.f || texc.y > 1.f || texc.y < 0.f)
        {
            __debugbreak();
            return false;
        }
    }
}

int HalfEdgeMesh::GetNextEdge( int ie ) const
{
    return _edges[ie].next;
}

int HalfEdgeMesh::GetPrevEdge( int ie ) const
{
    return _edges[ie].prev;
}

int HalfEdgeMesh::GetOppoEdge( int ie ) const
{
    return _edges[ie].opposite;
}

std::tuple<int, int, int> HalfEdgeMesh::GetFaceIndices( int face ) const
{
    const Face& f = _faces[face];
    const Edge& e = _edges[f.edge];
    int ia = e.vtx;
    int ib = _edges[e.next].vtx;
    int ic = _edges[_edges[e.next].next].vtx;
    return std::tuple<int, int, int>( ia, ib, ic );
}

std::tuple<int, int, int> HalfEdgeMesh::GetFaceEdges( int face ) const
{
    int e0 = _faces[face].edge;
    int e1 = GetNextEdge( e0 );
    int e2 = GetNextEdge( e1 );
    return { (unsigned)e0, (unsigned)e1, (unsigned)e2 };
}

std::pair<int, int> HalfEdgeMesh::GetEdgePoints( int edge ) const
{
    int p0 = _edges[edge].vtx;
    int p1 = _edges[_edges[edge].next].vtx;
    return { p0, p1 };
}

std::vector<int> HalfEdgeMesh::GetNeighborFaces( int vertex ) const
{
    std::vector<int> edges = GetNeighborEdges( vertex );
    std::vector<int> faces;
    for (int i : edges)
    {
        faces.push_back( _edges[i].face );
    }
    return faces;
}

void HalfEdgeMesh::GetNeighborFaces( int vertex, std::vector<int>* faces ) const
{
    const auto& v = _vertices[vertex];
    int istart = v.edge;
    int ie = v.edge;
    do
    {
        faces->push_back( _edges[ie].face );
        ie = GetOppoEdge( GetPrevEdge( ie ) );
        if (ie == -1)
        {
            break;
        }
    } while (ie != istart);

    if (ie == istart)
    {
        return;
    }

    istart = GetOppoEdge( istart );
    if (istart == -1)
    {
        return;
    }
    istart = GetNextEdge( istart );
    ie = istart;
    do
    {
        faces->push_back( _edges[ie].face );

        ie = GetOppoEdge( ie );
        if (ie == -1)
        {
            break;
        }
        ie = GetNextEdge( ie );
    } while (ie != v.edge);
}

std::vector<int> HalfEdgeMesh::GetNeighborVertices( int vertex ) const
{
    std::vector<int> neighbors;
    int istart = GetNextEdge( _vertices[vertex].edge );
    int handle = istart;
    do
    {
        neighbors.push_back( _edges[handle].vtx );
        handle = GetOppoEdge( GetNextEdge( handle ) );
        if (handle == -1)
        {
            break;
        }
        handle = GetNextEdge( handle );
    } while (handle != istart);

    if (handle == istart)
    {
        return neighbors;
    }
    istart = GetOppoEdge( _vertices[vertex].edge );
    if (istart == -1)
    {
        return neighbors;
    }
    istart = GetPrevEdge( istart );
    handle = istart;
    do
    {
        neighbors.push_back( _edges[handle].vtx );
        handle = GetOppoEdge( GetPrevEdge( handle ) );
        if (handle == -1)
        {
            break;
        }
        handle = GetPrevEdge( handle );
    } while (handle != istart);
    return neighbors;
}

std::vector<int> HalfEdgeMesh::GetNeighborEdges( int vertex ) const
{
    const auto& v = _vertices[vertex];
    std::vector<int> edges;
    int istart = v.edge;
    int ie = v.edge;
    do
    {
        edges.push_back( ie );
        ie = GetOppoEdge( GetPrevEdge( ie ) );
        if (ie == -1)
        {
            break;
        }
    } while (ie != istart);

    if (ie == istart)
    {
        return edges;
    }

    istart = GetOppoEdge( istart );
    if (istart == -1)
    {
        return edges;
    }
    istart = GetNextEdge( istart );
    ie = istart;
    do
    {
        edges.push_back( ie );
        ie = GetOppoEdge( ie );
        if (ie == -1)
        {
            break;
        }
        ie = GetNextEdge( ie );
    } while (ie != v.edge);
    return edges;
}

void HalfEdgeMesh::GetNeighborEdges( int vertex, std::vector<int>* edges ) const
{
    const auto& v = _vertices[vertex];
    int istart = v.edge;
    int ie = v.edge;
    do
    {
        edges->push_back( ie );
        ie = GetOppoEdge( GetPrevEdge( ie ) );
        if (ie == -1)
        {
            break;
        }
    } while (ie != istart);

    if (ie == istart)
    {
        return;
    }

    istart = GetOppoEdge( istart );
    if (istart == -1)
    {
        return;
    }
    istart = GetNextEdge( istart );
    ie = istart;
    do
    {
        edges->push_back( ie );
        ie = GetOppoEdge( ie );
        if (ie == -1)
        {
            break;
        }
        ie = GetNextEdge( ie );
    } while (ie != v.edge);
}

std::unordered_set<int> HalfEdgeMesh::GetNeighborFacesByFace( int face ) const
{
    auto [ia, ib, ic] = GetFaceIndices( face );
    std::unordered_set<int> result;
    for (int i : {ia, ib, ic})
    {
        auto neighbor = GetNeighborFaces( i );
        result.insert( std::begin( neighbor ), std::end( neighbor ) );
    }
    result.erase( face );
    return result;
}

std::unordered_set<int> HalfEdgeMesh::GetNeighborFacesByFace_edgeonly( int face ) const
{
    std::unordered_set<int> result;
    auto [ia, ib, ic] = GetFaceEdges( face );
    int oa = GetOppoEdge( ia );
    int ob = GetOppoEdge( ib );
    int oc = GetOppoEdge( ic );
    if (oa != -1)
    {
        result.insert( _edges[oa].face );
    }
    if (ob != -1)
    {
        result.insert( _edges[ob].face );
    }
    if (oc != -1)
    {
        result.insert( _edges[oc].face );
    }
    return result;
}

int HalfEdgeMesh::GetPublicEdge( int t1, int t2 ) const
{
    auto [a0, b0, c0] = GetFaceIndices( t1 );
    auto [a1, b1, c1] = GetFaceIndices( t2 );
    int idx1[3] = { a0, b0, c0 };
    int idx2[3] = { a1, b1, c1 };
    for (size_t i = 0; i < 3; i++)
    {
        int istart = idx1[i];
        int iend = idx1[(i + 1) % 3];
        for (size_t j = 0; j < 3; j++)
        {
            int istart2 = idx2[j];
            int iend2 = idx2[(j + 1) % 3];
            if (istart2 == iend && iend2 == istart)
            {
                return _edge_map.at( { istart, iend } );
            }
        }
    }
    return -1;
}

glm::vec3 HalfEdgeMesh::GetFaceNormal( int face ) const
{
    auto [ia, ib, ic] = GetFaceIndices( face );
    glm::vec3 v0 = _vertices[ia].pos;
    glm::vec3 v1 = _vertices[ib].pos;
    glm::vec3 v2 = _vertices[ic].pos;
    return glm::normalize( glm::cross( v1 - v0, v2 - v0 ) );
}

glm::vec3 HalfEdgeMesh::GetEdgeNormal( int istart, int iend ) const
{
    auto it_e = _edge_map.find( { istart, iend } );
    auto it_oe = _edge_map.find( { iend, istart } );
    if (it_e != std::end( _edge_map ) && it_oe != std::end( _edge_map ))
    {
        const Edge& e = _edges.at( it_e->second );
        const Edge& oe = _edges.at( it_oe->second );
        int iface = e.face;
        int ioface = oe.face;
        return (GetFaceNormal( iface ) + GetFaceNormal( ioface )) * 0.5f;
    }
    if (it_e != std::end( _edge_map ))
    {
        const Edge& e = _edges[it_e->second];
        int iface = e.face;
        return GetFaceNormal( iface );
    }
    if (it_oe != std::end( _edge_map ))
    {
        const Edge& oe = _edges[it_oe->second];
        int ioface = oe.face;
        return GetFaceNormal( ioface );
    }
    return glm::vec3( 0.f );
}

glm::vec3 HalfEdgeMesh::DiffCoord( int handle ) const
{
    std::vector<int> neighbors = GetNeighborVertices( handle );
    glm::vec3 diff_coord = _vertices[handle].rest_pos;
    for (int v : neighbors)
    {
        diff_coord -= _vertices[v].rest_pos / (float)neighbors.size();
    }
    return diff_coord;
}

float HalfEdgeMesh::TotalVolume() const
{
    float v = 0.f;
    for (int i = 0; i < _faces.size(); i++)
    {
        auto [ia, ib, ic] = GetFaceIndices( i );
        glm::vec3 pa = _vertices[ia].pos;
        glm::vec3 pb = _vertices[ib].pos;
        glm::vec3 pc = _vertices[ic].pos;
        glm::vec3 p0 = glm::vec3( 0.f );
        v += glm::dot( glm::cross( (pa - p0), (pb - p0) ), (pc - p0) ) / 6.0f;
    }
    return glm::abs( v );
}

HalfEdgeMesh::VertexAttr HalfEdgeMesh::GetVertexAttr( unsigned vtx ) const
{
    std::vector<int> edges = GetNeighborEdges( vtx );
    VertexAttr attr{};
    if (edges.empty())
    {
        return attr;
    }
    const Edge& e = _edges[edges.front()];
    attr.normal = e.normal;
    attr.texcoord = e.texcoord;
    attr.color = _vertices[vtx].color;
    attr.tangent = e.tangent;
    return attr;
}

HalfEdgeMesh::VertexAttr HalfEdgeMesh::GetEdgeAttr( int f, int v ) const
{
    auto [ia, ib, ic] = GetFaceIndices( f );
    int ie0 = _faces[f].edge;
    if (_edges[ie0].vtx == v)
    {
        return GetEdgeAttr( ie0 );
    }
    int ie1 = GetNextEdge( ie0 );
    if (_edges[ie1].vtx == v)
    {
        return GetEdgeAttr( ie1 );
    }
    int ie2 = GetNextEdge( ie1 );
    if (_edges[ie2].vtx == v)
    {
        return GetEdgeAttr( ie2 );
    }
    return VertexAttr();
}

HalfEdgeMesh::VertexAttr HalfEdgeMesh::GetEdgeAttr( unsigned edge ) const
{
    VertexAttr attr;
    const Edge& e = _edges[edge];
    attr.normal = e.normal;
    attr.texcoord = e.texcoord;
    attr.tangent = e.tangent;
    attr.color = _vertices[e.vtx].color;
    return attr;
}

void HalfEdgeMesh::SetEdgeAttr( int edge, const HalfEdgeMesh::VertexAttr& attr )
{
    Edge& e = _edges[edge];
    e.normal = attr.normal;
    e.texcoord = attr.texcoord;
    _vertices[e.vtx].color = attr.color;
    e.tangent = attr.tangent;
}

glm::vec3 HalfEdgeMesh::BarycentricPosInTri( unsigned tri_id, glm::vec3 p ) const
{
    auto [ia, ib, ic] = GetFaceIndices( tri_id );
    glm::vec3 a = _vertices[ia].pos;
    glm::vec3 b = _vertices[ib].pos;
    glm::vec3 c = _vertices[ic].pos;
    return glm::BarycentricPos( a, b, c, p );
}

bool HalfEdgeMesh::PointIsInTriangle( glm::vec3 point, int triangle ) const
{
    glm::vec3 barycent_pos = BarycentricPosInTri( triangle, point );
    for (size_t i = 0; i < 3; i++)
    {
        if (barycent_pos[i] < 0.f || barycent_pos[i] > 1.f)
        {
            return false;
        }
    }
    return true;
}

void HalfEdgeMesh::PointPositionInTriangle( unsigned int tri_id, glm::vec3 p, int* pos_type, int* index ) const
{
    glm::vec3 barycent_pos = BarycentricPosInTri( tri_id, p );
    for (size_t i = 0; i < 3; i++)
    {
        if (glm::FloatEqual( barycent_pos[i], 1.0f ))
        {
            *pos_type = 2;
            *index = i;
            return;
        }
    }
    for (size_t i = 0; i < 3; i++)
    {
        if (glm::FloatEqual( barycent_pos[i], 0.0f ))
        {
            *pos_type = 1;
            *index = (i + 1) % 3;
            return;
        }
    }
    *pos_type = 0;
}

void HalfEdgeMesh::Normalize()
{
    AABB aabb;
    for (const auto& v : _vertices)
    {
        aabb.Expand( v.pos );
    }

    glm::vec3 c = aabb.GetCenter();
    float scale = 1.0f / (aabb.max_corner - aabb.min_corner)[aabb.LongestAxis()];
    for (auto& v : _vertices)
    {
        v.pos = (v.pos - c) * scale;
        v.rest_pos = v.pos;
    }
}

int HalfEdgeMesh::AddVertex( glm::vec3 point )
{
    Vertex v;
    v.pos = point;
    v.rest_pos = point;
    v.edge = -1;
    _vertices.push_back( v );
    return _vertices.size() - 1;
}

int HalfEdgeMesh::AddVertexBaryCentInterp( glm::vec3 point, unsigned faceId )
{
    auto [i0, i1, i2] = GetFaceIndices( faceId );
    glm::vec3 args = BarycentricPosInTri( faceId, point );
    float arg0 = args[0];
    float arg1 = args[1];
    float arg2 = args[2];
    glm::vec3 rest_pos = arg0 * _vertices[i0].rest_pos + arg1 * _vertices[i1].rest_pos + arg2 * _vertices[i2].rest_pos;
    _vertices.push_back( Vertex{ point, rest_pos, glm::vec3( 0.f ), -1 } );
    return _vertices.size() - 1;
}

int HalfEdgeMesh::AddEdge( int vstart, int vend )
{
    int index = _edges.size();
    int oppo = -1;
    if (_edge_map.find( std::make_pair( vend, vstart ) ) != _edge_map.end())
    {
        oppo = _edge_map[std::make_pair( vend, vstart )];
        _edges[oppo].opposite = index;
    }
    _edges.push_back( Edge( vstart, -1, -1, -1, oppo ) );
    return index;
}

int HalfEdgeMesh::RemoveEdge( int ie )
{
    Edge e = _edges[ie];
    Edge next = _edges[e.next];
    _edge_map.erase( std::make_pair( e.vtx, next.vtx ) );
    if (_edge_map.find( std::make_pair( next.vtx, e.vtx ) ) != std::end( _edge_map ))
    {
        _edges[_edge_map[std::make_pair( next.vtx, e.vtx )]].opposite = -1;
    }
    _vertices[e.vtx].edge = -1;
    return -1;
}

int HalfEdgeMesh::AddTriangle( unsigned v0, unsigned v1, unsigned v2 )
{
    int faceid = _faces.size();
    int iea = _edges.size();
    int ieb = iea + 1;
    int iec = ieb + 1;
    _faces.push_back( Face{ iea } );

    _edges.push_back( Edge( v0, faceid, ieb, iec, -1 ) );
    _edges.push_back( Edge( v1, faceid, iec, iea, -1 ) );
    _edges.push_back( Edge( v2, faceid, iea, ieb, -1 ) );
    _edge_map[std::make_pair( v0, v1 )] = iea;
    _edge_map[std::make_pair( v1, v2 )] = ieb;
    _edge_map[std::make_pair( v2, v0 )] = iec;

    _vertices[v0].edge = iea;
    _vertices[v1].edge = ieb;
    _vertices[v2].edge = iec;
    if (_edge_map.find( std::make_pair( v1, v0 ) ) != std::end( _edge_map ))
    {
        _edges[iea].opposite = _edge_map[std::make_pair( v1, v0 )];
        _edges[_edge_map[std::make_pair( v1, v0 )]].opposite = iea;
    }
    if (_edge_map.find( std::make_pair( v2, v1 ) ) != std::end( _edge_map ))
    {
        _edges[ieb].opposite = _edge_map[std::make_pair( v2, v1 )];
        _edges[_edge_map[std::make_pair( v2, v1 )]].opposite = ieb;
    }
    if (_edge_map.find( std::make_pair( v0, v2 ) ) != std::end( _edge_map ))
    {
        _edges[iec].opposite = _edge_map[std::make_pair( v0, v2 )];
        _edges[_edge_map[std::make_pair( v0, v2 )]].opposite = iec;
    }
    return faceid;
}

int HalfEdgeMesh::AddTriangleByEdge( int e0, int e1, int e2 )
{
    return -1;
}

void HalfEdgeMesh::RemoveTriangle( int face )
{
    _faces[face].edge = -1;
}

void HalfEdgeMesh::RemoveTriangleWithEdge( unsigned id )
{
    Face& face = _faces[id];
    auto [ia, ib, ic] = GetFaceIndices( id );

    int ie = face.edge;
    _edges[ie].face = -1;
    ie = GetNextEdge( ie );
    _edges[ie].face = -1;
    ie = GetNextEdge( ie );
    _edges[ie].face = -1;

    _edge_map.erase( std::make_pair( ia, ib ) );
    _edge_map.erase( std::make_pair( ib, ic ) );
    _edge_map.erase( std::make_pair( ic, ia ) );

    if (_edge_map.find( std::make_pair( ib, ia ) ) != std::end( _edge_map ))
    {
        _edges[_edge_map[std::make_pair( ib, ia )]].opposite = -1;
    }
    if (_edge_map.find( std::make_pair( ic, ib ) ) != std::end( _edge_map ))
    {
        _edges[_edge_map[std::make_pair( ic, ib )]].opposite = -1;
    }
    if (_edge_map.find( std::make_pair( ia, ic ) ) != std::end( _edge_map ))
    {
        _edges[_edge_map[std::make_pair( ia, ic )]].opposite = -1;
    }
    face.edge = -1;
}

std::tuple<int, int, int> HalfEdgeMesh::SplitTriangle( unsigned id, glm::vec3 point, int* newVtx )
{
    auto [iea, ieb, iec] = GetFaceEdges( id );
    auto [v0, v1, v2] = GetFaceIndices( id );

    glm::vec3 args = glm::BarycentricPos( _vertices[v0].pos, _vertices[v1].pos, _vertices[v2].pos, point );
    VertexAttr attr0 = GetEdgeAttr( iea );
    VertexAttr attr1 = GetEdgeAttr( ieb );
    VertexAttr attr2 = GetEdgeAttr( iec );
    VertexAttr attr = attr0 * args[0] + attr1 * args[1] + attr2 * args[2];

    int f0 = _faces.size();
    int f1 = f0 + 1;
    int f2 = f0 + 2;

    int vnew = AddVertex( point );
    _vertices[vnew].rest_pos = _vertices[v0].rest_pos * args[0] + _vertices[v1].rest_pos * args[1] + _vertices[v2].rest_pos * args[2];
    int ori_edge_num = _edges.size();
    int f0_vNewv0 = ori_edge_num;
    int f0_v0v1 = iea;
    int f0_v1vNew = ori_edge_num + 1;

    int f1_vNewv1 = ori_edge_num + 2;
    int f1_v1v2 = ieb;
    int f1_v2vNew = ori_edge_num + 3;

    int f2_vNewv2 = ori_edge_num + 4;
    int f2_v2v0 = iec;
    int f2_v0vNew = ori_edge_num + 5;

    _edges.push_back( Edge( vnew, f0, f0_v0v1, f0_v1vNew, f2_v0vNew ) );
    _edges.push_back( Edge( v1, f0, f0_vNewv0, f0_v0v1, f1_vNewv1 ) );
    _edges.push_back( Edge( vnew, f1, f1_v1v2, f1_v2vNew, f0_v1vNew ) );
    _edges.push_back( Edge( v2, f1, f1_vNewv1, f1_v1v2, f2_vNewv2 ) );
    _edges.push_back( Edge( vnew, f2, f2_v2v0, f2_v0vNew, f1_v2vNew ) );
    _edges.push_back( Edge( v0, f2, f2_vNewv2, f2_v2v0, f0_vNewv0 ) );
    _edge_map[{vnew, v0}] = f0_vNewv0;
    _edge_map[{v1, vnew}] = f0_v1vNew;
    _edge_map[{vnew, v1}] = f1_vNewv1;
    _edge_map[{v2, vnew}] = f1_v2vNew;
    _edge_map[{vnew, v2}] = f2_vNewv2;
    _edge_map[{v0, vnew}] = f2_v0vNew;

    _vertices[vnew].edge = f0_vNewv0;

    _faces.push_back( Face{ f0_vNewv0 } );
    _faces.push_back( Face{ f1_vNewv1 } );
    _faces.push_back( Face{ f2_vNewv2 } );

    _faces[id].edge = -1;
    _edges[f0_v0v1].face = f0;
    _edges[f0_v0v1].next = f0_v1vNew;
    _edges[f0_v0v1].prev = f0_vNewv0;

    _edges[f1_v1v2].face = f1;
    _edges[f1_v1v2].next = f1_v2vNew;
    _edges[f1_v1v2].prev = f1_vNewv1;

    _edges[f2_v2v0].face = f2;
    _edges[f2_v2v0].next = f2_v0vNew;
    _edges[f2_v2v0].prev = f2_vNewv2;

    SetEdgeAttr( f0_v1vNew, attr1 );
    SetEdgeAttr( f0_vNewv0, attr );
    SetEdgeAttr( f1_v2vNew, attr2 );
    SetEdgeAttr( f1_vNewv1, attr );
    SetEdgeAttr( f2_v0vNew, attr0 );
    SetEdgeAttr( f2_vNewv2, attr );

    if (newVtx)
    {
        *newVtx = vnew;
    }

    return { f0, f1, f2 };
}

std::vector<int> HalfEdgeMesh::SplitEdge( unsigned v0, unsigned v1, glm::vec3 point, int* newVtx )
{
    auto it = _edge_map.find( { v0, v1 } );
    auto it2 = _edge_map.find( { v1, v0 } );
    int fa, fb, fc, fd;
    int vnew = AddVertex( point );
    float d = 1 - glm::distance( point, _vertices[v0].pos ) / glm::distance( _vertices[v0].pos, _vertices[v1].pos );
    _vertices[vnew].rest_pos = _vertices[v0].rest_pos * d + _vertices[v1].rest_pos * (1 - d);

    *newVtx = vnew;
    if (it != std::end( _edge_map ) && it2 == std::end( _edge_map ))
    {
        int ie = it->second;

        VertexAttr v0attr = GetEdgeAttr( ie );
        VertexAttr v1attr = GetEdgeAttr( GetNextEdge( ie ) );
        VertexAttr vf0attr = GetEdgeAttr( GetPrevEdge( ie ) );

        VertexAttr attr = d * GetEdgeAttr( ie ) + (1 - d) * GetEdgeAttr( _vertices[v1].edge );

        int vf = _edges[GetPrevEdge( ie )].vtx;
        RemoveTriangleWithEdge( _edges[ie].face );
        fa = AddTriangle( vf, v0, vnew );
        fb = AddTriangle( vf, vnew, v1 );

        SetEdgeAttr( _edge_map.at( { vnew, v1 } ), attr );
        SetEdgeAttr( _edge_map.at( { v1, vf } ), v1attr );
        SetEdgeAttr( _edge_map.at( { vf, vnew } ), vf0attr );
        SetEdgeAttr( _edge_map.at( { vnew, vf } ), attr );
        SetEdgeAttr( _edge_map.at( { vf, v0 } ), vf0attr );
        SetEdgeAttr( _edge_map.at( { v0, vnew } ), v0attr );

        _edges[ie].face = -1;

        return std::vector<int>{ fa, fb };
    }
    if (it == std::end( _edge_map ) && it2 != std::end( _edge_map ))
    {
        int ie = it2->second;

        VertexAttr v0attr = GetEdgeAttr( GetNextEdge( ie ) );
        VertexAttr v1attr = GetEdgeAttr( ie );
        VertexAttr vf1attr = GetEdgeAttr( GetPrevEdge( ie ) );

        VertexAttr attr = d * GetEdgeAttr( _vertices[v0].edge ) + (1 - d) * GetEdgeAttr( ie );

        int f = _edges[ie].face;
        int vf = _edges[GetPrevEdge( ie )].vtx;
        RemoveTriangleWithEdge( f );
        fc = AddTriangle( vf, vnew, v0 );
        fd = AddTriangle( vf, v1, vnew );

        SetEdgeAttr( _edge_map.at( { vnew, v0 } ), attr );
        SetEdgeAttr( _edge_map.at( { vnew, vf } ), attr );
        SetEdgeAttr( _edge_map.at( { vf, v1 } ), vf1attr );
        SetEdgeAttr( _edge_map.at( { v1, vnew } ), v1attr );
        SetEdgeAttr( _edge_map.at( { v0, vf } ), v0attr );
        SetEdgeAttr( _edge_map.at( { vf, vnew } ), vf1attr );

        _edges[ie].face = -1;

        return std::vector<int>{ fc, fd };
    }
    if (it != std::end( _edge_map ) && it2 != std::end( _edge_map ))
    {
        int ie0 = it->second;
        int ie1 = it2->second;

        int iface0 = _edges[ie0].face;
        int iface1 = _edges[ie1].face;

        int ivf0 = _edges[GetPrevEdge( ie0 )].vtx;
        int ivf1 = _edges[GetPrevEdge( ie1 )].vtx;
        VertexAttr f0v0attr = GetEdgeAttr( iface0, v0 );
        VertexAttr f0v1attr = GetEdgeAttr( iface0, v1 );
        VertexAttr f1v0attr = GetEdgeAttr( iface1, v0 );
        VertexAttr f1v1attr = GetEdgeAttr( iface1, v1 );
        VertexAttr vf0attr = GetEdgeAttr( iface0, ivf0 );
        VertexAttr vf1attr = GetEdgeAttr( iface1, ivf1 );
        RemoveTriangleWithEdge( iface0 );
        RemoveTriangleWithEdge( iface1 );
        fa = AddTriangle( ivf0, v0, vnew );
        fb = AddTriangle( ivf0, vnew, v1 );
        fc = AddTriangle( ivf1, vnew, v0 );
        fd = AddTriangle( ivf1, v1, vnew );

        VertexAttr f0nvattr = d * f0v0attr + (1.f - d) * f0v1attr;
        VertexAttr f1nvattr = d * f1v0attr + (1.f - d) * f1v1attr;
        SetEdgeAttr( _edge_map.at( { vnew, v1 } ), f0nvattr );
        SetEdgeAttr( _edge_map.at( { v1, ivf0 } ), f0v1attr );
        SetEdgeAttr( _edge_map.at( { ivf0, vnew } ), vf0attr );
        SetEdgeAttr( _edge_map.at( { vnew, ivf0 } ), f0nvattr );
        SetEdgeAttr( _edge_map.at( { ivf0, v0 } ), vf0attr );
        SetEdgeAttr( _edge_map.at( { v0, vnew } ), f0v0attr );

        SetEdgeAttr( _edge_map.at( { vnew, v0 } ), f1nvattr );
        SetEdgeAttr( _edge_map.at( { v0, ivf1 } ), f1v0attr );
        SetEdgeAttr( _edge_map.at( { ivf1, vnew } ), vf1attr );
        SetEdgeAttr( _edge_map.at( { vnew, ivf1 } ), f1nvattr );
        SetEdgeAttr( _edge_map.at( { ivf1, v1 } ), vf1attr );
        SetEdgeAttr( _edge_map.at( { v1, vnew } ), f1v1attr );

        _edges[ie0].face = -1;
        _edges[ie1].face = -1;

        return std::vector<int>{ fa, fb, fc, fd };
    }
    assert( false );
    return std::vector<int>();
}

std::pair<int, int> HalfEdgeMesh::SplitVertex( unsigned vertex, unsigned prev, unsigned next )
{
    int prev_edge = -1;
    if (_edge_map.find( { vertex, prev } ) != _edge_map.end())
    {
        prev_edge = _edge_map.at( { vertex, prev } );
    }
    int next_edge = _edge_map.at( { vertex, next } );
    int prev_oppo = _edges[prev_edge].opposite;
    int next_oppo = _edges[next_edge].opposite;

    int pnew = AddVertexBaryCentInterp( _vertices[vertex].pos, _edges[prev_edge].face );
    int pold = vertex;
    std::vector<int> faces1;
    std::vector<int> faces2;

    int e = next_edge;
    while (e != prev_edge && e != -1)
    {
        faces1.push_back( _edges[e].face );
        e = _edges[_edges[e].prev].opposite;
    }
    if (e != prev_edge && prev_oppo != -1)
    {
        e = prev_oppo;
        while (e != -1 && e != next_edge)
        {
            faces1.push_back( _edges[e].face );
            e = GetOppoEdge( GetNextEdge( e ) );
        }
    }

    e = prev_edge;
    while (e != next_edge && e != -1)
    {
        faces2.push_back( _edges[e].face );
        e = _edges[_edges[e].prev].opposite;
    }
    if (e != next_edge)
    {
        e = GetOppoEdge( next_edge );
        while (e != -1 && e != prev_edge)
        {
            faces2.push_back( _edges[e].face );
            e = GetOppoEdge( GetNextEdge( e ) );
        }
    }

    for (int f : faces1)
    {
        auto [ia, ib, ic] = GetFaceEdges( f );
        int iedges[3] = { ia, ib, ic };
        for (int ie : iedges)
        {
            _edge_map.erase( GetEdgePoints( ie ) );
        }
        for (int ie : iedges)
        {
            Edge& e = _edges[ie];
            if (e.vtx == pold)
            {
                e.vtx = pnew;
                break;
            }
        }

        for (int ie : iedges)
        {
            _edge_map[GetEdgePoints( ie )] = ie;
        }
    }

    _vertices[pnew].edge = _edge_map.at( std::make_pair( pnew, next ) );
    _vertices[pold].edge = GetNextEdge( next_oppo );
    _edges[next_edge].opposite = -1;
    _edges[next_oppo].opposite = -1;
    if (prev_edge != -1)
        _edges[prev_edge].opposite = -1;
    if (prev_oppo != -1)
        _edges[prev_oppo].opposite = -1;

    return { pnew, pold };
}

std::pair<int, int> HalfEdgeMesh::CutTriangle( unsigned id, glm::vec3 start, glm::vec3 end,
    PtTriTopo start_topo, PtTriTopo end_topo, int start_index, int end_index )
{
    Face& face = _faces[id];
    auto [iv0, iv1, iv2] = GetFaceIndices( id );
    int vindices[3] = { iv0, iv1, iv2 };
    int v1 = -999;
    int v2 = -999;
    if (start_topo == PtTriTopo::INSIDE && end_topo == PtTriTopo::INSIDE)
    {
        auto [t0, t1, t2] = SplitTriangle( id, start, &v1 );
        for (int tri : {t0, t1, t2})
        {
            if (PointIsInTriangle( end, tri ))
            {
                SplitTriangle( tri, end, &v2 );
                break;
            }
        }
    }
    else if (start_topo == PtTriTopo::EDGE && end_topo == PtTriTopo::INSIDE)
    {
        unsigned edge_v0 = vindices[start_index];
        unsigned edge_v1 = vindices[(start_index + 1) % 3];
        SplitTriangle( id, end, &v1 );
        SplitEdge( edge_v0, edge_v1, start, &v2 );
    }
    else if (start_topo == PtTriTopo::INSIDE && end_topo == PtTriTopo::EDGE)
    {
        unsigned edge_v0 = vindices[end_index];
        unsigned edge_v1 = vindices[(end_index + 1) % 3];
        SplitTriangle( id, start, &v1 );
        SplitEdge( edge_v0, edge_v1, end, &v2 );
    }
    else if (start_topo == PtTriTopo::EDGE && end_topo == PtTriTopo::EDGE)
    {
        if (start_index != end_index)
        {
            unsigned edge_v0 = vindices[start_index];
            unsigned edge_v1 = vindices[(start_index + 1) % 3];
            unsigned edge_v2 = vindices[end_index];
            unsigned edge_v3 = vindices[(end_index + 1) % 3];
            SplitEdge( edge_v0, edge_v1, start, &v1 );
            SplitEdge( edge_v2, edge_v3, end, &v2 );
        }
        else
        {
            std::cout << "ERROR2" << std::endl;
        }
    }
    //else if (start_type == 2 && end_type == 0)
    //{
    //    v1 = curTri[start_index];
    //    SplitTriangle(id, end, &v2);
    //}
    //else if (start_type == 0 && end_type == 2)
    //{
    //    v1 = curTri[end_index];
    //    SplitTriangle(id, start, &v2);
    //}
    //else if (start_type == 2 && end_type == 1)
    //{
    //    unsigned edge_v0 = curTri[end_index];
    //    unsigned edge_v1 = curTri[(end_index + 1) % 3];
    //    v1 = curTri[start_index];
    //    SplitEdge(edge_v0, edge_v1, end, &v2);
    //}
    //else if (start_type == 1 && end_type == 2)
    //{
    //    unsigned edge_v0 = curTri[start_index];
    //    unsigned edge_v1 = curTri[(start_index + 1) % 3];
    //    v1 = curTri[end_index];
    //    SplitEdge(edge_v0, edge_v1, start, &v2);
    //}
    //else if (start_type == 2 && end_type == 2)
    //{
    //    v1 = curTri[start_index];
    //    v2 = curTri[end_index];
    //}
    else
    {
        assert( false );
    }
    return std::pair<int, int>( v1, v2 );
}

void HalfEdgeMesh::UpdateNormal()
{
    for (auto& e : _edges)
    {
        e.normal = glm::vec3( 0.f );
    }

    if (_use_face_normal)
    {
        for (int f = 0, s = _faces.size(); f < s; f++)
        {
            if (!TriangleIsValid( f ))
            {
                continue;
            }
            auto [ia, ib, ic] = GetFaceIndices( f );
            glm::vec3 normal = glm::normalize( glm::cross( _vertices[ib].pos - _vertices[ia].pos, _vertices[ic].pos - _vertices[ia].pos ) );
            auto [ea, eb, ec] = GetFaceEdges( f );
            _edges[ea].normal = normal;
            _edges[eb].normal = normal;
            _edges[ec].normal = normal;
        }
    }
    else
    {
        std::vector<int> neibuffer;
        std::vector<glm::vec3>  weighted_normals( _faces.size() );
        for (int f = 0, s = _faces.size(); f < s; f++)
        {
            if (!TriangleIsValid( f ))
            {
                continue;
            }
            auto [ia, ib, ic] = GetFaceIndices( f );
            glm::vec3 p0 = _vertices[ia].pos;
            glm::vec3 p1 = _vertices[ib].pos;
            glm::vec3 p2 = _vertices[ic].pos;
            glm::vec3 p0p1 = p1 - p0;
            glm::vec3 p0p2 = p2 - p0;
            glm::vec3 normal = glm::normalize( glm::cross( p0p1, p0p2 ) );
            float cos = p0p1.x * p0p2.x + p0p1.y * p0p2.y + p0p1.z * p0p2.z;
            cos = cos / (std::sqrtf( glm::length2( p0p1 ) * glm::length2( p0p2 ) ));
            float rad = std::acos( cos );
            weighted_normals[f] = rad * normal;
        }
        for (int v = 0, s = _vertices.size(); v < s; v++)
        {
            if (_vertices[v].edge == -1)
            {
                continue;
            }
            GetNeighborEdges( v, &neibuffer );
            glm::vec3 normal( 0.f );
            for (int e : neibuffer)
            {
                normal += weighted_normals[_edges[e].face];
            }

            for (int e : neibuffer)
            {
                _edges[e].normal = normal;
            }
            neibuffer.clear();
        }
        for (auto& e : _edges)
        {
            if (e.face == -1)
                continue;
            if (e.sharp_edge)
            {
                GetNeighborFaces( e.vtx, &neibuffer );
                for (int f : neibuffer)
                {
                    auto [ia, ib, ic] = GetFaceIndices( f );
                    glm::vec3 normal = glm::normalize( glm::cross( _vertices[ib].pos - _vertices[ia].pos, _vertices[ic].pos - _vertices[ia].pos ) );
                    auto [ea, eb, ec] = GetFaceEdges( f );
                    _edges[ea].normal = normal;
                    _edges[eb].normal = normal;
                    _edges[ec].normal = normal;
                }
                neibuffer.clear();
            }
            e.normal = glm::normalize( e.normal );
        }
    }

    /*for (int e = 0; e < _edges.size(); e++)
    {
        if (_edges[e].face == -1)
            continue;

        if (_edges[e].sharp_edge)
        {
            auto [ia, ib, ic] = GetFaceIndices( _edges[e].face );
            glm::vec3 normal = glm::cross( _vertices[ib].pos - _vertices[ia].pos, _vertices[ic].pos - _vertices[ia].pos );
            _edges[e].normal = glm::normalize( normal );
        }
        else
        {
            auto [ia, ib, ic] = GetFaceIndices( _edges[e].face );
            glm::vec3 normal = glm::cross( _vertices[ib].pos - _vertices[ia].pos, _vertices[ic].pos - _vertices[ia].pos );
            if (GetOppoEdge( e ) != -1 && _edges[GetOppoEdge( e )].face != -1)
            {
                auto [ia2, ib2, ic2] = GetFaceIndices( _edges[GetOppoEdge( e )].face );
                normal += glm::cross( _vertices[ib2].pos - _vertices[ia2].pos, _vertices[ic2].pos - _vertices[ia2].pos );
            }
            _edges[e].normal = glm::normalize( normal );
        }
    }*/


    //tan & bitan
    for (auto& e : _edges)
    {
        e.tangent = glm::vec3( 0.f );
    }
    for (int f = 0, s = _faces.size(); f < s; f++)
    {
        if (!TriangleIsValid( f ))
        {
            continue;
        }
        auto [ia, ib, ic] = GetFaceIndices( f );
        auto [ea, eb, ec] = GetFaceEdges( f );
        glm::vec3 edge1 = _vertices[ib].pos - _vertices[ia].pos;
        glm::vec3 edge2 = _vertices[ic].pos - _vertices[ia].pos;
        glm::vec2 duv1 = _edges[eb].texcoord - _edges[ea].texcoord;
        glm::vec2 duv2 = _edges[ec].texcoord - _edges[ea].texcoord;
        GLfloat p = 1.0f / (duv1.x * duv2.y - duv2.x * duv1.y);
        glm::vec3 tangent;
        tangent.x = p * (duv2.y * edge1.x - duv1.y * edge2.x);
        tangent.y = p * (duv2.y * edge1.y - duv1.y * edge2.y);
        tangent.z = p * (duv2.y * edge1.z - duv1.y * edge2.z);
        tangent = glm::normalize( tangent );

        _edges[ea].tangent = tangent;
        _edges[eb].tangent = tangent;
        _edges[ec].tangent = tangent;
    }
}

void HalfEdgeMesh::UpdateAttrBuffer()
{
    mRenderingVtxCount = 0;
    _attr_buffer.clear();
    _attr_buffer.reserve( _edges.size() );
    for (int i = 0, s = _faces.size(); i < s; i++)
    {
        if (!TriangleIsValid( i ))
        {
            continue;
        }
        const Face& f = _faces[i];
        const Edge& e1 = _edges[f.edge];
        const Edge& e2 = _edges[e1.next];
        const Edge& e3 = _edges[e2.next];
        _attr_buffer.push_back( VertexAttr{ e1.normal, e1.texcoord, _vertices[e1.vtx].color, e1.tangent, e1.vtx } );
        _attr_buffer.push_back( VertexAttr{ e2.normal, e2.texcoord, _vertices[e2.vtx].color, e2.tangent, e2.vtx } );
        _attr_buffer.push_back( VertexAttr{ e3.normal, e3.texcoord, _vertices[e3.vtx].color, e3.tangent, e3.vtx } );
        mRenderingVtxCount += 3;
    }
    _attr_vbo->UpdateData( (const void*)_attr_buffer.data(), sizeof( _attr_buffer[0] ) * _attr_buffer.size() );
}

void HalfEdgeMesh::UpdatePosBuffer()
{
    mRenderingVtxCount = 0;
    _point_buffer.clear();
    _point_buffer.reserve( _edges.size() );
    for (int i = 0, s = _faces.size(); i < s; i++)
    {
        if (!TriangleIsValid( i ))
        {
            continue;
        }
        auto [ia, ib, ic] = GetFaceIndices( i );
        if (_show_restpos)
        {
            _point_buffer.push_back( _vertices[ia].rest_pos );
            _point_buffer.push_back( _vertices[ib].rest_pos );
            _point_buffer.push_back( _vertices[ic].rest_pos );
        }
        else
        {
            _point_buffer.push_back( _vertices[ia].pos );
            _point_buffer.push_back( _vertices[ib].pos );
            _point_buffer.push_back( _vertices[ic].pos );
        }
        mRenderingVtxCount += 3;
    }
    _point_vbo->UpdateData( (const void*)_point_buffer.data(), sizeof( _point_buffer[0] ) * _point_buffer.size(), GL_DYNAMIC_DRAW );

}

void HalfEdgeMesh::Update()
{
    /*if (Input::IsMouseButtonDown( Input::MouseButton::Left ) && Input::IsKeyHeld( Input::Key::LEFT_SHIFT ))
    {
        HalfEdgeSurfaceTester tester( this );

        glm::vec2 cursor = Input::GetMousePosition();
        cursor.y = Camera::current->mViewportSize.y - cursor.y;
        Ray r = Camera::current->ScreenPointToRay( cursor );

        IntersectionRec rec;
        int id = -1;

        tester.RayIntersect( r, &rec, 0.f, FLT_MAX, &id );

        for (auto& v : _vertices)
        {
            v.color = glm::vec3( 1, 1, 1 );
        }

        auto neighbors = GetNeighborFacesByFace_edgeonly( id );
        std::cout << id << ", " << neighbors.size() << ":";
        for (auto i : neighbors)
        {
            std::cout << i << " ";
            auto [ia, ib, ic] = GetFaceIndices( i );
            _vertices[ia].color = glm::vec3( 1, 0, 0 );
            _vertices[ib].color = glm::vec3( 1, 0, 0 );
            _vertices[ic].color = glm::vec3( 1, 0, 0 );
        }
        std::cout << std::endl;
    }*/
}

void HalfEdgeMesh::Draw()
{
    _vao->Bind();
    static auto set_uniforms = [&]( Shader& shader, const Transform* transform, const Material* mat_main, const Material* mat_sub )
    {
        Light::SetAllLightUniforms( shader );
        mat_main->SetShaderUniforms( shader );
        if (mat_sub != nullptr)
        {
            mat_sub->SetShaderUniforms( shader );
        }
    };

    Shader* shader = nullptr;
    if (mRenderConfig._draw_line)
    {
        switch (mRenderConfig._line_mattype)
        {
        case RenderConfig::MatType::PlainColor:
            //glDepthMask( false );
            shader = Shader::Find( "model_plain_color" );
            break;
        case RenderConfig::MatType::VtxColor:
            //glDepthMask( false );
            shader = Shader::Find( "model_vtx_color" );
            break;
        case RenderConfig::MatType::Texture:
            shader = Shader::Find( "model" );
            break;
        default:
            break;
        }
        shader->use();
        set_uniforms( *shader, &mTransform, _material_main.get(), _material_sub.get() );
        glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
        glEnable( GL_POLYGON_OFFSET_LINE );
        glPolygonOffset( -0.5f, 0.0f );
        glDrawArrays( GL_TRIANGLES, 0, mRenderingVtxCount );
        glDepthMask( true );
        //glDisable( GL_POLYGON_OFFSET_LINE );
    }
    if (mRenderConfig._draw_points)
    {
        switch (mRenderConfig._point_mattype)
        {
        case RenderConfig::MatType::PlainColor:
            //glDepthMask( false );
            shader = Shader::Find( "model_plain_color" );
            break;
        case RenderConfig::MatType::VtxColor:
            //glDepthMask( false );
            shader = Shader::Find( "model_vtx_color" );
            break;
        case RenderConfig::MatType::Texture:
            shader = Shader::Find( _shader_name );
            break;
        default:
            break;
        }
        shader->use();
        set_uniforms( *shader, &mTransform, _material_main.get(), _material_sub.get() );
        glPolygonMode( GL_FRONT_AND_BACK, GL_POINT );
        glPointSize( 5.0f );
        glDrawArrays( GL_TRIANGLES, 0, mRenderingVtxCount );
        glDepthMask( true );
    }
    if (mRenderConfig._draw_face)
    {
        switch (mRenderConfig._face_mattype)
        {
        case RenderConfig::MatType::PlainColor:
            if (_material_main->mAlpha != 1.0f)
            {
                glDepthMask( false );
            }
            shader = Shader::Find( "model_plain_color" );
            break;
        case RenderConfig::MatType::VtxColor:
            if (_material_main->mAlpha != 1.0f)
            {
                glDepthMask( false );
            }
            shader = Shader::Find( "model_vtx_color" );
            break;
        case RenderConfig::MatType::Texture:
            if (_material_main->mAlpha != 1.0f)
            {
                glDisable( GL_DEPTH_TEST );
                glEnable( GL_CULL_FACE );
            }
            if (_no_normal)
            {
                shader = Shader::Find( "model_no_normal" );
            }
            else
            {
                shader = Shader::Find( _shader_name );
            }
            break;
        default:
            break;
        }
        shader->use();
        shader->setVec( "uBallCenter", mTransform.GetPosition() );
        set_uniforms( *shader, &mTransform, _material_main.get(), _material_sub.get() );
        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
        glDisable( GL_POLYGON_OFFSET_FILL );
        glDrawArrays( GL_TRIANGLES, 0, mRenderingVtxCount );
        glDepthMask( true );
        glEnable( GL_DEPTH_TEST );

    }
}

void HalfEdgeMesh::DrawShadowDepth()
{
    _vao->Bind();
    auto shader = Shader::Find( "depth" );
    shader->use();
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    glDrawArrays( GL_TRIANGLES, 0, mRenderingVtxCount );
}

HalfEdgeMesh::VertexAttr operator*( float s, const HalfEdgeMesh::VertexAttr& a )
{
    return HalfEdgeMesh::VertexAttr{ a.normal * s, a.texcoord * s, a.color * s, a.tangent * s };
}

MetaballHalfEdgeMesh::MetaballHalfEdgeMesh()
    :HalfEdgeMesh()
{
    mLayer = -1;
}

MetaballHalfEdgeMesh::MetaballHalfEdgeMesh( const std::string& path )
    :HalfEdgeMesh( path )
{
}

void MetaballHalfEdgeMesh::Draw()
{
    _vao->Bind();
    static auto set_uniforms = [&]( Shader& shader, const Transform* transform, const Material* mat_main, const Material* mat_sub )
    {
        mat_main->SetShaderUniforms( shader );
        if (mat_sub != nullptr)
        {
            mat_sub->SetShaderUniforms( shader );
        }
        shader.setFloat( "uAlpha", mat_main->mAlpha );
    };

    Shader* shader = nullptr;
    if (mRenderConfig._draw_line)
    {
        switch (mRenderConfig._line_mattype)
        {
        case RenderConfig::MatType::PlainColor:
            //glDepthMask( false );
            shader = Shader::Find( "model_plain_color_pbd" );
            break;
        case RenderConfig::MatType::VtxColor:
            //glDepthMask( false );
            shader = Shader::Find( "model_vtx_color_pbd" );
            break;
        case RenderConfig::MatType::Texture:
            shader = Shader::Find( "model_pbd" );
            break;
        default:
            break;
        }
        shader->use();
        set_uniforms( *shader, &mTransform, _material_main.get(), _material_sub.get() );
        glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
        glEnable( GL_POLYGON_OFFSET_LINE );
        glPolygonOffset( -0.5f, 0.0f );
        glDrawArrays( GL_TRIANGLES, 0, mRenderingVtxCount );
        glDepthMask( true );
        //glDisable( GL_POLYGON_OFFSET_LINE );
    }
    if (mRenderConfig._draw_points)
    {
        switch (mRenderConfig._point_mattype)
        {
        case RenderConfig::MatType::PlainColor:
            //glDepthMask( false );
            shader = Shader::Find( "model_plain_color_pbd" );
            break;
        case RenderConfig::MatType::VtxColor:
            //glDepthMask( false );
            shader = Shader::Find( "model_vtx_color_pbd" );
            break;
        case RenderConfig::MatType::Texture:
            shader = Shader::Find( "model_pbd" );
            break;
        default:
            break;
        }
        shader->use();
        set_uniforms( *shader, &mTransform, _material_main.get(), _material_sub.get() );
        glPolygonMode( GL_FRONT_AND_BACK, GL_POINT );
        glPointSize( 5.0f );
        glDrawArrays( GL_TRIANGLES, 0, mRenderingVtxCount );
        glDepthMask( true );
    }
    if (mRenderConfig._draw_face)
    {
        switch (mRenderConfig._face_mattype)
        {
        case RenderConfig::MatType::PlainColor:
            if (_material_main->mAlpha != 1.0f)
            {
                glDepthMask( false );
            }
            shader = Shader::Find( "model_plain_color_pbd" );
            break;
        case RenderConfig::MatType::VtxColor:
            if (_material_main->mAlpha != 1.0f)
            {
                glDepthMask( false );
            }
            shader = Shader::Find( "model_vtx_color_pbd" );
            break;
        case RenderConfig::MatType::Texture:
            shader = Shader::Find( "model_pbd" );
            break;
        default:
            break;
        }
        shader->use();
        set_uniforms( *shader, &mTransform, _material_main.get(), _material_sub.get() );

        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
        glDisable( GL_POLYGON_OFFSET_FILL );
        glDrawArrays( GL_TRIANGLES, 0, mRenderingVtxCount );
        glDepthMask( true );
    }
}

void MetaballHalfEdgeMesh::DrawShadowDepth()
{
    _vao->Bind();
    auto shader = Shader::Find( "depth" );
    shader->use();
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    glDrawArrays( GL_TRIANGLES, 0, mRenderingVtxCount );
}

PDMetaballHalfEdgeMesh::PDMetaballHalfEdgeMesh()
    : HalfEdgeMesh()
{
    mLayer = -1;
    _no_normal = true;
}

PDMetaballHalfEdgeMesh::PDMetaballHalfEdgeMesh( const std::string& path )
    : HalfEdgeMesh( path )
{
    _no_normal = true;
    mLayer = -1;
}

void PDMetaballHalfEdgeMesh::Draw()
{
    static auto set_uniforms = [&]( Shader& shader, const Transform* transform, const Material* mat_main )
    {
        mat_main->SetShaderUniforms( shader );
        shader.setFloat( "uAlpha", mat_main->mAlpha );
    };
    if (mRenderConfig._draw_face)
    {
        _vao->Bind();
        _material_main->mAlpha = 1.0f;
        Shader* shader = nullptr;
        if (_gpu_skinning)
        {
            shader = Shader::Find( "model_pd" );
        }
        else
        {
            shader = Shader::Find( "model" );
        }
        shader->use();
        set_uniforms( *shader, &mTransform, _material_main.get() );
        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
        glDisable( GL_POLYGON_OFFSET_FILL );
        glDepthMask( true );
        glDrawArrays( GL_TRIANGLES, 0, mRenderingVtxCount );
    }
    if (mRenderConfig._draw_line)
    {
        Shader* shader = nullptr;

        _material_main->SetDiffuseColor( 0.1, 0.1, 0.1 );
        _material_main->mAlpha = 0.3f;
        if (_gpu_skinning)
        {
            shader = Shader::Find( "model_pd" );
        }
        else
        {
            shader = Shader::Find( "model" );
        }
        shader->use();
        set_uniforms( *shader, &mTransform, _material_main.get() );
        glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
        glEnable( GL_POLYGON_OFFSET_LINE );
        glPolygonOffset( -0.5f, 0.0f );
        glDrawArrays( GL_TRIANGLES, 0, mRenderingVtxCount );
        glDepthMask( true );
        _material_main->SetDiffuseColor( 0.509804, 0.713726, 0.866667 );
        //glDisable( GL_POLYGON_OFFSET_LINE );
    }
    //shader->setInt( "temp_flag", 1 );
    //glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    //glLineWidth( 2 );
    //glDrawArrays( GL_TRIANGLES, 0, mRenderingVtxCount );
    //shader->setInt( "temp_flag", 0 );
    //glPointSize( 12 );
    //glPolygonMode( GL_FRONT_AND_BACK, GL_POINT );
    //glDrawArrays( GL_TRIANGLES, 0, mRenderingVtxCount );
    //glPointSize( 1 );
    //glLineWidth( 1 );
}

void PDMetaballHalfEdgeMesh::DrawShadowDepth()
{
    _vao->Bind();
    auto shader = Shader::Find( "depth_pd" );
    shader->use();
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    glDrawArrays( GL_TRIANGLES, 0, mRenderingVtxCount );
}

float MeshDistance( HalfEdgeMesh* mesh1, HalfEdgeMesh* mesh2 )
{
    double dist = 0.0;
    if (mesh1->GetVertexNumber() != mesh2->GetVertexNumber())
        return FLT_MAX;
    for (int i = 0; i < mesh1->GetVertexNumber(); i++)
    {
        glm::vec3 p1 = mesh1->GetPosition( i );
        p1.x += 1.5f;
        glm::vec3 p2 = mesh2->GetPosition( i );
        dist += glm::distance( p1, p2 );
    }
    dist /= mesh1->GetVertexNumber();
    return dist;
}