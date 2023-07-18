#pragma once
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <glm/glm.hpp>
#include <boost/container_hash/hash.hpp>
#include "util/SceneObject.h"
#include "acceleration/RayIntersectSolver.h"
#include "model/Ray.h"
#include "model/Triangle.h"
#include "gl/VertexArray.h"
#include "gl/VertexBuffer.h"
#include "gl/IndexBuffer.h"
#include "ECS/ECS.h"

class VertexArray;
class VertexBuffer;
class IndexBuffer;
class ShaderStorageBuffer;
class Material;

class HalfEdgeMesh :
    public SceneObject, public Component
{
public:
    class Vertex
    {
    public:
        glm::vec3 pos;
        glm::vec3 rest_pos;
        glm::vec3 color = glm::vec3( 0.8f );
        int edge = -1;
        int topo = 0;   //1 for above, -1 for beneath
        bool recal_x0 = false; // need to recalc rest pos
    };

    class Edge
    {
    public:
        Edge()
            :vtx( -1 ), face( -1 ), next( -1 ), prev( -1 ), opposite( -1 )
        {
        }
        Edge( int vtx_, int face_, int next_, int prev_, int opposite_ )
            :vtx( vtx_ ), face( face_ ), next( next_ ), prev( prev_ ), opposite( opposite_ )
        {
        }
        int vtx;
        int face;
        int next;
        int prev;
        int opposite;
        bool sharp_edge = false;        //if an edge is sharp edge, use face normal as its normal
        glm::vec3 normal = glm::vec3( 0.f );
        glm::vec2 texcoord = glm::vec2( 0.f );
        glm::vec3 tangent = glm::vec3( 0.f );
    };

    class EdgeHash
    {
    public:
        size_t operator()( const std::pair<int, int>& idxPair ) const
        {
            return boost::hash<std::pair<int, int>>()(idxPair);
            //return std::hash<int>()(idxPair.first) ^ std::hash<int>()(idxPair.second);
        }
    };

    class EdgePred
    {
    public:
        bool operator()( const std::pair<int, int>& left, const std::pair<int, int>& right ) const
        {
            return left.first == right.first && left.second == right.second;
        }
    };

    class Face
    {
    public:
        int edge = -1;
    };

    enum class PtTriTopo
    {
        INSIDE, EDGE
    };

    struct VertexAttr
    {
        glm::vec3 normal = glm::vec3( 0.f );
        glm::vec2 texcoord = glm::vec2( 0.f );
        glm::vec3 color = glm::vec3( 0.f );
        glm::vec3 tangent = glm::vec3( 0.f );
        int index = -1;
        VertexAttr operator*( float s )
        {
            return VertexAttr{ normal * s, texcoord * s, color * s, tangent * s, -1 };
        }

        VertexAttr operator+( const VertexAttr& attr )
        {
            return VertexAttr{ normal + attr.normal, texcoord + attr.texcoord, color + attr.color, tangent + attr.tangent, -1 };
        }
    };

    friend class HalfEdgeMeshRenderer;

    HalfEdgeMesh();
    HalfEdgeMesh( const std::string& path );
    void LoadOBJ( const std::string& path );

    void SetMainMaterial( const std::shared_ptr<Material>& mat );
    void SetSubMaterial( const std::shared_ptr<Material>& mat );

    inline bool         TriangleIsValid( int f ) const;
    inline size_t       GetVertexNumber() const;
    inline size_t       GetFaceNumber() const;
    inline glm::vec3	GetPosition( unsigned vtx ) const;
    inline glm::vec3    GetRestPos( unsigned vtx ) const;
    inline glm::vec3	GetNormal( unsigned edge ) const;
    inline glm::vec2	GetTexCoord( unsigned edge ) const;
    inline glm::vec3	GetTangent( unsigned edge ) const;
    VertexAttr          GetVertexAttr( unsigned vtx ) const;
    VertexAttr          GetEdgeAttr( int f, int v ) const;
    VertexAttr          GetEdgeAttr( unsigned edge ) const;
    inline void		    SetPosition( unsigned edge, const glm::vec3& value );
    inline void		    SetNormal( unsigned edge, const glm::vec3& value );
    inline void		    SetTexCoord( unsigned edge, const glm::vec2& value );
    inline void		    SetTangent( unsigned edge, const glm::vec3& value );
    void                SetEdgeAttr( int edge, const VertexAttr& attr );
    void                SetVtxNormal( unsigned vtx, const glm::vec3& normal );
    void                SetVtxTexCoord( unsigned vtx, const glm::vec2& texcoord );
    void                SetVtxTangent( unsigned vtx, const glm::vec3& tangent );
    void                SetVtxColor( unsigned vtx, const glm::vec3& color );
    void                SetFaceColor( int face, const glm::vec3& color );
    glm::vec3           DiffCoord( int handle ) const;
    float TotalVolume() const;

    bool CheckTexcoord() const;
    bool CheckTexcoord( int f ) const;

    int GetNextEdge( int ie ) const;
    int GetPrevEdge( int ie ) const;
    int GetOppoEdge( int ie ) const;
    std::tuple<int, int, int>   GetFaceIndices( int face ) const;
    std::tuple<int, int, int>   GetFaceEdges( int face ) const;
    std::pair<int, int>         GetEdgePoints( int edge ) const;
    std::vector<int>            GetNeighborFaces( int vertex ) const;
    void                        GetNeighborFaces( int vertex, std::vector<int>* faces ) const;
    std::vector<int>            GetNeighborVertices( int vertex ) const;
    std::vector<int>            GetNeighborEdges( int vertex ) const;
    void                        GetNeighborEdges( int vertex, std::vector<int>* edges ) const;
    std::unordered_set<int>     GetNeighborFacesByFace( int face ) const;
    std::unordered_set<int>     GetNeighborFacesByFace_edgeonly( int face ) const;
    int             GetPublicEdge( int t1, int t2 ) const;
    glm::vec3       GetFaceNormal( int face ) const;
    glm::vec3       GetEdgeNormal( int istart, int iend ) const;
    glm::vec3       BarycentricPosInTri( unsigned tri_id, glm::vec3 p ) const;
    bool            PointIsInTriangle( glm::vec3 point, int triangle ) const;
    void            PointPositionInTriangle( unsigned int tri_id, glm::vec3 p, int* pos_type, int* index ) const;

    void    Normalize();
    int     AddVertex( glm::vec3 point );
    int     AddVertexBaryCentInterp( glm::vec3 point, unsigned faceId );
    int     AddEdge( int vstart, int vend );
    int     RemoveEdge( int ie );
    int     AddTriangle( unsigned v0, unsigned v1, unsigned v2 );
    int     AddTriangleByEdge( int e0, int e1, int e2 );
    void    RemoveTriangle( int face );
    void    RemoveTriangleWithEdge( unsigned face );
    std::tuple<int, int, int> SplitTriangle( unsigned id, glm::vec3 point, int* newVtx );
    std::pair<int, int>     CutTriangle( unsigned id, glm::vec3 start, glm::vec3 end,
        PtTriTopo start_topo, PtTriTopo end_topo, int start_edge, int end_edge );
    std::vector<int>        SplitEdge( unsigned v0, unsigned v1, glm::vec3 point, int* newVtx );
    std::pair<int, int>     SplitVertex( unsigned vertex, unsigned prev, unsigned next );

    void UpdateNormal();
    void UpdateAttrBuffer();
    void UpdatePosBuffer();
    VertexArray& GetVAO();
    const VertexArray& GetVAO() const;

    virtual void Update() override;
    virtual void Draw() override;
    virtual void DrawShadowDepth() override;
    virtual void DrawGUI() override;

protected:
    void InitOpenglObjects();

public:
    std::vector<Vertex> _vertices;
    std::vector<Face>   _faces;
    std::vector<Edge>   _edges;
    std::unordered_map<std::pair<int, int>, int, EdgeHash, EdgePred> _edge_map;

    std::shared_ptr<Material> _material_main = nullptr;
    std::shared_ptr<Material> _material_sub = nullptr;

    bool _show_restpos = false;
    bool _use_face_normal = false;
    bool _no_normal = false;
    std::string _shader_name = "model";
protected:
    std::vector<glm::vec3>                  _point_buffer;
    std::vector<VertexAttr>                 _attr_buffer;
    std::unique_ptr<VertexArray>            _vao = nullptr;
    std::unique_ptr<VertexBuffer>           _point_vbo = nullptr;
    std::unique_ptr<VertexBuffer>           _attr_vbo = nullptr;
    int mRenderingVtxCount = 0;
};


bool HalfEdgeMesh::TriangleIsValid( int f ) const
{
    return _faces[f].edge != -1;
}

size_t HalfEdgeMesh::GetVertexNumber() const
{
    return _vertices.size();
}

size_t HalfEdgeMesh::GetFaceNumber() const
{
    return _faces.size();
}

glm::vec3 HalfEdgeMesh::GetPosition( unsigned vtx ) const
{
    return _vertices[vtx].pos;
}

glm::vec3 HalfEdgeMesh::GetRestPos( unsigned vtx ) const
{
    return _vertices[vtx].rest_pos;
}

glm::vec3 HalfEdgeMesh::GetNormal( unsigned edge ) const
{
    return _edges[edge].normal;
}

glm::vec2 HalfEdgeMesh::GetTexCoord( unsigned edge ) const
{
    return _edges[edge].texcoord;
}

glm::vec3 HalfEdgeMesh::GetTangent( unsigned edge ) const
{
    return _edges[edge].tangent;
}

void HalfEdgeMesh::SetPosition( unsigned vtx, const glm::vec3& value )
{
    _vertices[vtx].pos = value;
}

void HalfEdgeMesh::SetNormal( unsigned edge, const glm::vec3& value )
{
    _edges[edge].normal = value;
}

void HalfEdgeMesh::SetTexCoord( unsigned edge, const glm::vec2& value )
{
    _edges[edge].texcoord = value;
}

void HalfEdgeMesh::SetTangent( unsigned edge, const glm::vec3& value )
{
    _edges[edge].tangent = value;
}

HalfEdgeMesh::VertexAttr operator*( float s, const HalfEdgeMesh::VertexAttr& a );


class MetaballHalfEdgeMesh : public HalfEdgeMesh
{
public:
    MetaballHalfEdgeMesh();
    explicit MetaballHalfEdgeMesh( const std::string& path );
    virtual ~MetaballHalfEdgeMesh() = default;
    virtual void Draw() override;
    virtual void DrawShadowDepth() override;
};

class PDMetaballHalfEdgeMesh : public HalfEdgeMesh
{
public:
    PDMetaballHalfEdgeMesh();
    explicit PDMetaballHalfEdgeMesh( const std::string& path );
    virtual ~PDMetaballHalfEdgeMesh() = default;
    virtual void Draw() override;
    virtual void DrawShadowDepth() override;
    bool _gpu_skinning = true;
};

float MeshDistance( HalfEdgeMesh* mesh1, HalfEdgeMesh* mesh2 );