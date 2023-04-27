#pragma once
#include <array>
#include <memory>
#include <vector>
#include <fstream>
#include <iterator>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Kernel/global_functions.h>
#include "util/Shader.h"
#include "util/Camera.h"
#include "lighting/Light.h"
#include "gl/VertexArray.h"
#include "gl/VertexBuffer.h"
#include "util/SceneObject.h"
#include "util/MathTypeConverter.h"
#include "tinyobjloader/tiny_obj_loader.h"
#include "material/Material.h"

template <typename Refs, typename Tag, typename Traits>
class VertexBase : public CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, typename Traits::Point_3>
{
public:
    VertexBase() = default;
    explicit VertexBase( const typename Traits::Point_3& p ) : CGAL::HalfedgeDS_vertex_base<Refs, CGAL::Tag_true, typename Traits::Point_3>( p ) {}
public:
    typename Traits::Vector_3 color{ 1, 1, 1 };
    int index{ 0 };
};

template <typename Refs, typename Traits>
class HalfedgeBase : public CGAL::HalfedgeDS_halfedge_base<Refs>
{
public:
    Traits::Vector_3 normal{ 0, 1, 0 };
    Traits::Vector_3 tangent{ 0, 0, 0 };
    Traits::Vector_2 texcoord{ 0, 0 };
    bool sharp_edge{ false };
};

class ItemBase : public CGAL::Polyhedron_items_3
{
public:
    template<typename Refs, typename Traits>
    struct Vertex_wrapper
    {
        using Vertex = VertexBase<Refs, CGAL::Tag_true, Traits>;
    };

    template<typename Refs, typename Traits>
    struct Halfedge_wrapper
    {
        using Halfedge = HalfedgeBase<Refs, Traits>;
    };
};

template <typename HDS, typename Item, typename Kernel>
class PolyhedronObjBulider : public CGAL::Modifier_base<HDS>
{
public:
    PolyhedronObjBulider( const tinyobj::attrib_t& attrib, const tinyobj::shape_t& shape )
        : _attrib( attrib ), _shape( shape ) {}
    virtual void operator()( HDS& hds ) override;
private:
    const tinyobj::attrib_t& _attrib;
    const tinyobj::shape_t& _shape;
};

template<typename HDS, typename Item, typename Kernel>
inline void PolyhedronObjBulider<HDS, Item, Kernel>::operator()( HDS& hds )
{
    const auto& attrib = _attrib;
    const auto& shape = _shape;

    const std::vector<float>& vertices = attrib.vertices;
    const std::vector<float>& colors = attrib.colors;
    const std::vector<float>& normals = attrib.normals;
    const std::vector<float>& texcoords = attrib.texcoords;

    const bool verbose = true;
    CGAL::Polyhedron_incremental_builder_3<HDS> builder( hds, verbose );
    builder.begin_surface( vertices.size() / 3, shape.mesh.indices.size() / 3 );

    for (size_t i = 0, size = vertices.size(); i < size; i += 3)
    {
        typename HDS::Vertex_handle vh = builder.add_vertex( CGAL::Point_3<Kernel>( vertices[i], vertices[i + 1], vertices[i + 2] ) );
        vh->color = typename Kernel::Vector_3{ colors[i], colors[i + 1], colors[i + 2] };
        vh->index = i / 3;
    }

    for (size_t f = 0, size = shape.mesh.num_face_vertices.size(); f < size; ++f)
    {
        const tinyobj::index_t& i0 = shape.mesh.indices[f * 3 + 0];
        const tinyobj::index_t& i1 = shape.mesh.indices[f * 3 + 1];
        const tinyobj::index_t& i2 = shape.mesh.indices[f * 3 + 2];
        builder.begin_facet();
        builder.add_vertex_to_facet( i0.vertex_index );
        builder.add_vertex_to_facet( i1.vertex_index );
        builder.add_vertex_to_facet( i2.vertex_index );
        auto hh = builder.end_facet();

        for (int v = 0; v < 3; ++v)
        {
            tinyobj::index_t idx = shape.mesh.indices[f * 3 + v];

            int ni = idx.normal_index;
            if (ni >= 0)
                hh->normal = typename Kernel::Vector_3{ normals[3 * ni + 0], normals[3 * ni + 1], normals[3 * ni + 2] };

            int ti = idx.texcoord_index;
            if (ti >= 0)
                hh->texcoord = typename Kernel::Vector_2{ texcoords[2 * ti + 0], texcoords[2 * ti + 1] };

            hh = hh->next();
        }
    }

    builder.end_surface();
}

template <typename T>
concept ItemType = requires (T a) { std::is_base_of_v<ItemBase, T>; };


template <ItemType Item = ItemBase, typename Kernel = CGAL::Simple_cartesian<float>>
class CHalfEdgeMesh :
    public SceneObject, public CGAL::Polyhedron_3<Kernel, ItemBase>
{
public:
    using Polyhedron = CGAL::Polyhedron_3<Kernel, ItemBase>;
    using hHalfedge = Polyhedron::Halfedge_handle;
    using hVertex = Polyhedron::Vertex_handle;
    using hFacet = Polyhedron::Facet_handle;
    using Halfedge = Polyhedron::Halfedge;
    using Vertex = Polyhedron::Vertex;
    using Facet = Polyhedron::Facet;

    enum class BufferType { Position, Normal, Texcoord, Color, Tangent, Index, Count };
    enum class NormalType { FaceNormal, VertexNormal };

    CHalfEdgeMesh();
    explicit CHalfEdgeMesh( const std::string& path );
    virtual void Update() override;
    virtual void Draw() override;
    void UpdateBuffer();
    void UpdateBuffer( BufferType type );
    void UpdateNormal( NormalType type );
    CGAL::Vector_3<Kernel> FaceNormal( hFacet hf ) const;

protected:
    //materials
    std::unique_ptr<Material> _mat_main{ nullptr };

    //opengl objects
    std::unique_ptr<VertexArray> _vao{ nullptr };
    std::unique_ptr<VertexBuffer> _pos_vbo{ nullptr };
    std::unique_ptr<VertexBuffer> _normal_vbo{ nullptr };
    std::unique_ptr<VertexBuffer> _color_vbo{ nullptr };
    std::unique_ptr<VertexBuffer> _tangent_vbo{ nullptr };
    std::unique_ptr<VertexBuffer> _texcoord_vbo{ nullptr };
    std::unique_ptr<VertexBuffer> _index_vbo{ nullptr };
    std::vector<Eigen::Vector3<float>> _pos_buffer;
    std::vector<Eigen::Vector3<float>> _normal_buffer;
    std::vector<Eigen::Vector3<float>> _color_buffer;
    std::vector<Eigen::Vector3<float>> _tangent_buffer;
    std::vector<Eigen::Vector2<float>> _texcoord_buffer;
    std::vector<int> _index_buffer;
};

template <ItemType Item, typename Kernel>
CHalfEdgeMesh<Item, Kernel>::CHalfEdgeMesh()
{
    _vao = std::make_unique<VertexArray>();

    _pos_vbo = std::make_unique<VertexBuffer>( nullptr, 0 );
    _normal_vbo = std::make_unique<VertexBuffer>( nullptr, 0 );
    _color_vbo = std::make_unique<VertexBuffer>( nullptr, 0 );
    _tangent_vbo = std::make_unique<VertexBuffer>( nullptr, 0 );
    _texcoord_vbo = std::make_unique<VertexBuffer>( nullptr, 0 );
    _index_vbo = std::make_unique<VertexBuffer>( nullptr, 0 );

    VertexBufferLayout vec3_layout;
    vec3_layout.Push( GL_FLOAT, 3, 0 );
    VertexBufferLayout vec2_layout;
    vec2_layout.Push( GL_FLOAT, 2, 0 );
    VertexBufferLayout int_layout;
    int_layout.Push( GL_INT, 1, 0 );

    _pos_vbo->SetLayout( vec3_layout );
    _normal_vbo->SetLayout( vec3_layout );
    _color_vbo->SetLayout( vec3_layout );
    _tangent_vbo->SetLayout( vec3_layout );
    _texcoord_vbo->SetLayout( vec2_layout );
    _index_vbo->SetLayout( int_layout );

    _vao->AddBuffer( *_pos_vbo );
    _vao->AddBuffer( *_normal_vbo );
    _vao->AddBuffer( *_texcoord_vbo );
    _vao->AddBuffer( *_color_vbo );
    _vao->AddBuffer( *_tangent_vbo );
    _vao->AddBuffer( *_index_vbo );
}

template <ItemType Item, typename Kernel>
CHalfEdgeMesh<Item, Kernel>::CHalfEdgeMesh( const std::string& path )
    :CHalfEdgeMesh()
{
    std::string dir = "";
    size_t pos = path.find_last_of( '/' );
    if (pos != std::string::npos)
        dir = path.substr( 0, pos + 1 );
    else if ((pos = path.find_last_of( '\\' )) != std::string::npos)
        dir = path.substr( 0, pos + 1 );
    else
        dir = "./";

    tinyobj::ObjReaderConfig reader_config;
    reader_config.triangulate = true;

    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile( path, reader_config )) {
        if (!reader.Error().empty()) {
            std::cout << "TinyObjReader: " << reader.Error();
        }
    }
    if (!reader.Warning().empty()) {
        std::cout << "TinyObjReader: " << reader.Warning();
    }

    PolyhedronObjBulider<typename Polyhedron::HalfedgeDS, Item, Kernel> builder{ reader.GetAttrib(), reader.GetShapes()[0] };
    this->delegate( builder );

    const static auto pt2vec3 = []( const tinyobj::real_t* p ) { return glm::vec3( p[0], p[1], p[2] );  };
    const static auto pt2vec2 = []( const tinyobj::real_t* p ) { return glm::vec2( p[0], p[1] ); };
    auto& materials = reader.GetMaterials();
    MaterialInfos main_info;
    std::string main_name( "" );
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
        main_name = main_mat.name;
    }
    _mat_main = std::make_unique<Material>( dir, "material", main_info );

    UpdateNormal( NormalType::VertexNormal );
    UpdateBuffer();
}

template <ItemType Item, typename Kernel>
void CHalfEdgeMesh<Item, Kernel>::Update()
{
}

template <ItemType Item, typename Kernel>
void CHalfEdgeMesh<Item, Kernel>::Draw()
{
    //Maybe move all this to a Render class
    _vao->Bind();
    static auto set_uniforms = [&]( Shader& shader, const Transform* transform, const Material* mat_main )
    {
        mat_main->SetShaderUniforms( shader );
        shader.setFloat( "uAlpha", mat_main->mAlpha );
    };

    auto shader = Shader::Find( "model" );
    shader->use();
    set_uniforms( *shader, &mTransform, _mat_main.get() );
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    glDisable( GL_POLYGON_OFFSET_FILL );
    glDrawArrays( GL_TRIANGLES, 0, this->size_of_facets() * 3 );
    glDepthMask( true );
}

template <ItemType Item, typename Kernel>
void CHalfEdgeMesh<Item, Kernel>::UpdateBuffer()
{
    int num = this->size_of_facets() * 3;
    _pos_buffer.resize( num );
    _normal_buffer.resize( num );
    _color_buffer.resize( num );
    _tangent_buffer.resize( num );
    _texcoord_buffer.resize( num );
    _index_buffer.resize( num );

    int i = 0;
    for (auto it = this->facets_begin(); it != this->facets_end(); it++)
    {
        hHalfedge hh = it->halfedge();
        hVertex hv = hh->vertex();
        for (int v = 0; v < 3; v++)
        {
            _pos_buffer[i] = ToEigen( hv->point() );
            _color_buffer[i] = ToEigen( hv->color );
            _normal_buffer[i] = ToEigen( hh->normal );
            _tangent_buffer[i] = ToEigen( hh->tangent );
            _texcoord_buffer[i] = ToEigen( hh->texcoord );
            _index_buffer[i] = hv->index;
            i++;
            hh = hh->next();
            hv = hh->vertex();
        }
    }

    _pos_vbo->UpdateData( _pos_buffer.data(), _pos_buffer.size() * sizeof( _pos_buffer[0] ) );
    _normal_vbo->UpdateData( _normal_buffer.data(), _normal_buffer.size() * sizeof( _normal_buffer[0] ) );
    _color_vbo->UpdateData( _color_buffer.data(), _color_buffer.size() * sizeof( _color_buffer[0] ) );
    _tangent_vbo->UpdateData( _tangent_buffer.data(), _tangent_buffer.size() * sizeof( _tangent_buffer[0] ) );
    _texcoord_vbo->UpdateData( _texcoord_buffer.data(), _texcoord_buffer.size() * sizeof( _texcoord_buffer[0] ) );
    _index_vbo->UpdateData( _index_buffer.data(), _index_buffer.size() * sizeof( _index_buffer[0] ) );
}

template <ItemType Item, typename Kernel>
void CHalfEdgeMesh<Item, Kernel>::UpdateBuffer( BufferType type )
{
    int num = this->size_of_facets() * 3;
    int i = 0;
    switch (type)
    {
    case CHalfEdgeMesh::BufferType::Position:
    {
        _pos_buffer.resize( num );
        for (auto it = this->facets_begin(); it != this->facets_end(); it++)
        {
            hHalfedge hh = it->halfedge();
            for (int v = 0; v < 3; v++)
            {
                _pos_buffer[i] = ToEigen( hh->vertex()->point() );
                i++;
                hh = hh->next();
            }
        }
        _pos_vbo->UpdateData( _pos_buffer.data(), _pos_buffer.size() * sizeof( _pos_buffer[0] ) );
        break;
    }
    case CHalfEdgeMesh::BufferType::Normal:
    {
        _normal_buffer.resize( num );
        for (auto it = this->facets_begin(); it != this->facets_end(); it++)
        {
            hHalfedge hh = it->halfedge();
            for (int v = 0; v < 3; v++)
            {
                _normal_buffer[i] = ToEigen( hh->normal );
                i++;
                hh = hh->next();
            }
        }
        _normal_vbo->UpdateData( _normal_buffer.data(), _normal_buffer.size() * sizeof( _normal_buffer[0] ) );
        break;
    }
    case CHalfEdgeMesh::BufferType::Texcoord:
    {
        _texcoord_buffer.resize( num );
        for (auto it = this->facets_begin(); it != this->facets_end(); it++)
        {
            hHalfedge hh = it->halfedge();
            for (int v = 0; v < 3; v++)
            {
                _texcoord_buffer[i] = ToEigen( hh->texcoord );
                i++;
                hh = hh->next();
            }
        }
        _texcoord_vbo->UpdateData( _texcoord_buffer.data(), _texcoord_buffer.size() * sizeof( _texcoord_buffer[0] ) );
        break;
    }
    case CHalfEdgeMesh::BufferType::Color:
    {
        _color_buffer.resize( num );
        for (auto it = this->facets_begin(); it != this->facets_end(); it++)
        {
            hHalfedge hh = it->halfedge();
            for (int v = 0; v < 3; v++)
            {
                _color_buffer[i] = ToEigen( hh->vertex()->color );
                i++;
                hh = hh->next();
            }
        }
        _color_vbo->UpdateData( _color_buffer.data(), _color_buffer.size() * sizeof( _color_buffer[0] ) );
        break;
    }
    case CHalfEdgeMesh::BufferType::Tangent:
    {
        _tangent_buffer.resize( num );
        for (auto it = this->facets_begin(); it != this->facets_end(); it++)
        {
            hHalfedge hh = it->halfedge();
            for (int v = 0; v < 3; v++)
            {
                _tangent_buffer[i] = ToEigen( hh->tangent );
                i++;
                hh = hh->next();
            }
        }
        _tangent_vbo->UpdateData( _tangent_buffer.data(), _tangent_buffer.size() * sizeof( _tangent_buffer[0] ) );
        break;
    }
    case CHalfEdgeMesh::BufferType::Index:
    {
        _index_buffer.resize( num );
        for (auto it = this->facets_begin(); it != this->facets_end(); it++)
        {
            hHalfedge hh = it->halfedge();
            for (int v = 0; v < 3; v++)
            {
                _index_buffer[i] = hh->vertex()->index;
                i++;
                hh = hh->next();
            }
        }
        _index_vbo->UpdateData( _index_buffer.data(), _index_buffer.size() * sizeof( _index_buffer[0] ) );
        break;
    }
    case CHalfEdgeMesh::BufferType::Count:
        break;
    default:
        break;
    }

}

template <ItemType Item, typename Kernel>
void CHalfEdgeMesh<Item, Kernel>::UpdateNormal( NormalType type )
{

    if (type == NormalType::FaceNormal)
    {
        for (hFacet hf = this->facets_begin(); hf != this->facets_end(); hf++)
        {
            auto normal = FaceNormal( hf );
            hHalfedge hh = hf->halfedge();
            hh->normal = CGAL::Vector_3<Kernel>( normal.x(), normal.y(), normal.z() );
            hh->next()->normal = CGAL::Vector_3<Kernel>( normal.x(), normal.y(), normal.z() );
            hh->prev()->normal = CGAL::Vector_3<Kernel>( normal.x(), normal.y(), normal.z() );
        }
    }
    else
    {
        for (hVertex hv = this->vertices_begin(); hv != this->vertices_end(); hv++)
        {
            CGAL::Halfedge_around_target_circulator<Polyhedron> c( hv->halfedge(), *this );
            CGAL::Vector_3<Kernel> normal( 0, 0, 0 );
            do
            {
                auto p0 = (*c)->vertex()->point();
                auto p1 = (*c)->next()->vertex()->point();
                auto p2 = (*c)->prev()->vertex()->point();
                CGAL::Vector_3<Kernel> p0p1 = p1 - p0;
                CGAL::Vector_3<Kernel> p0p2 = p2 - p0;
                CGAL::Vector_3<Kernel> face_normal = CGAL::cross_product( p0p1, p0p2 );
                face_normal /= std::sqrtf( face_normal.squared_length() );
                float cos = p0p1.x() * p0p2.x() + p0p1.y() * p0p2.y() + p0p1.z() * p0p2.z();
                cos = cos / (std::sqrtf( p0p1.squared_length() * p0p2.squared_length() ));
                float rad = std::acos( cos );
                normal += face_normal * rad;
                c++;
            } while (*c != hv->halfedge());

            normal = normal / std::sqrt( normal.squared_length() );

            for (auto hh : CGAL::halfedges_around_target( hv, *this ))
            {
                hh->normal = normal;
            }
        }
    }
    UpdateBuffer( BufferType::Normal );

    for (hFacet hf = this->facets_begin(); hf != this->facets_end(); hf++)
    {
        hHalfedge hh0 = hf->halfedge();
        hHalfedge hh1 = hh0->next();
        hHalfedge hh2 = hh1->next();
        hVertex hv0 = hh0->vertex();
        hVertex hv1 = hh1->vertex();
        hVertex hv2 = hh2->vertex();

        auto temp1 = hv1->point() - hv0->point();
        auto temp2 = hv2->point() - hv0->point();
        CGAL::Vector_3<Kernel> edge1( temp1.x(), temp1.y(), temp1.z() );
        CGAL::Vector_3<Kernel> edge2( temp2.x(), temp2.y(), temp2.z() );
        CGAL::Vector_2<Kernel> duv1 = hh1->texcoord - hh0->texcoord;
        CGAL::Vector_2<Kernel> duv2 = hh2->texcoord - hh0->texcoord;
        float p = 1.0f / (duv1.x() * duv2.y() - duv2.x() * duv1.y());
        CGAL::Vector_3<Kernel> tangent(
            p * (duv2.y() * edge1.x() - duv1.y() * edge2.x()),
            p * (duv2.y() * edge1.y() - duv1.y() * edge2.y()),
            p * (duv2.y() * edge1.z() - duv1.y() * edge2.z())
        );
        tangent = tangent / std::sqrt( tangent.squared_length() );

        hh0->tangent = tangent;
        hh1->tangent = tangent;
        hh2->tangent = tangent;
    }
    UpdateBuffer( BufferType::Tangent );
}

template <ItemType Item, typename Kernel>
CGAL::Vector_3<Kernel> CHalfEdgeMesh<Item, Kernel>::FaceNormal( hFacet hf ) const
{
    hHalfedge hh = hf->halfedge();
    auto p0 = hh->vertex()->point();
    auto p1 = hh->next()->vertex()->point();
    auto p2 = hh->prev()->vertex()->point();
    return CGAL::unit_normal( p0, p1, p2 );
}

#define CGAL_GRAPH_TRAITS_INHERITANCE_TEMPLATE_PARAMS ItemType Item, typename Kernel
#define CGAL_GRAPH_TRAITS_INHERITANCE_CLASS_NAME CHalfEdgeMesh<Item, Kernel>
#define CGAL_GRAPH_TRAITS_INHERITANCE_BASE_CLASS_NAME typename CGAL_GRAPH_TRAITS_INHERITANCE_CLASS_NAME::Polyhedron
#include <CGAL/boost/graph/graph_traits_inheritance_macros.h>

template <typename Kernel = CGAL::Simple_cartesian<float>>
class CSurfaceMesh : public CGAL::Surface_mesh<typename Kernel::Point_3>
{
public:
    using CGALMeshType = CGAL::Surface_mesh<typename Kernel::Point_3>;

    CSurfaceMesh();
    CSurfaceMesh( const std::string& path );

    void LoadOBJ( const std::string& path );

protected:
    CGALMeshType::template Property_map<typename CGALMeshType::Vertex_index, typename Kernel::Vector_3> _colors;
    CGALMeshType::template Property_map<typename CGALMeshType::Halfedge_index, typename Kernel::Vector_3> _normals;
    CGALMeshType::template Property_map<typename CGALMeshType::Halfedge_index, typename Kernel::Vector_2> _texcoords;
    std::unique_ptr<Material> _material{ nullptr };
};

template <typename Kernel>
CSurfaceMesh<Kernel>::CSurfaceMesh()
{

}

template <typename Kernel>
CSurfaceMesh<Kernel>::CSurfaceMesh( const std::string& path )
{

}

template <typename Kernel>
void CSurfaceMesh<Kernel>::LoadOBJ( const std::string& path )
{
    std::string dir = "";
    size_t pos = path.find_last_of( '/' );
    if (pos != std::string::npos)
        dir = path.substr( 0, pos + 1 );
    else if ((pos = path.find_last_of( '\\' )) != std::string::npos)
        dir = path.substr( 0, pos + 1 );
    else
        dir = "./";

    tinyobj::ObjReaderConfig reader_config;
    reader_config.triangulate = true;

    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile( path, reader_config )) {
        if (!reader.Error().empty()) {
            std::cout << "TinyObjReader: " << reader.Error();
        }
    }
    if (!reader.Warning().empty()) {
        std::cout << "TinyObjReader: " << reader.Warning();
    }

    const static auto pt2vec3 = []( const tinyobj::real_t* p ) { return glm::vec3( p[0], p[1], p[2] );  };
    const static auto pt2vec2 = []( const tinyobj::real_t* p ) { return glm::vec2( p[0], p[1] ); };
    auto& materials = reader.GetMaterials();
    MaterialInfos main_info;
    std::string main_name( "" );
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
        main_name = main_mat.name;
    }
    _material = std::make_unique<Material>( dir, "material", main_info );

    const auto& attrib = reader.GetAttrib();
    const auto& shape = reader.GetShapes()[0];
    const std::vector<float>& vertices = attrib.vertices;
    const std::vector<float>& colors = attrib.colors;
    const std::vector<float>& normals = attrib.normals;
    const std::vector<float>& texcoords = attrib.texcoords;

    for (size_t v = 0; v < vertices.size() / 3; v++)
    {
        auto hv = add_vertex( Kernel::Point_3( vertices[v * 3 + 0], vertices[v * 3 + 1], vertices[v * 3 + 2] ) );
        _colors[hv] = Kernel::Vector_3( colors[v * 3 + 0], colors[v * 3 + 1], colors[v * 3 + 2] );
    }

    for (int f = 0, size = shape.mesh.num_face_vertices.size(); f < size; ++f)
    {
        const tinyobj::index_t& i0 = shape.mesh.indices[f * 3 + 0];
        const tinyobj::index_t& i1 = shape.mesh.indices[f * 3 + 1];
        const tinyobj::index_t& i2 = shape.mesh.indices[f * 3 + 2];

        auto hf = this->add_face( i0.vertex_index, i1.vertex_index, i2.vertex_index );

        CGAL::Halfedge_around_face_circulator<CSurfaceMesh<Kernel>> hbegin( hf, *this );
        _normals[hbegin] = normals[i0.normal_index];
        _texcoords[hbegin] = normals[i0.texcoord_index];
        hbegin++;
        _normals[hbegin] = normals[i1.normal_index];
        _texcoords[hbegin] = normals[i1.texcoord_index];
        hbegin++;
        _normals[hbegin] = normals[i2.normal_index];
        _texcoords[hbegin] = normals[i2.texcoord_index];
    }
}