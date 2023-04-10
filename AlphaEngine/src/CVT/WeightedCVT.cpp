#include "WeightedCVT.h"
#include <chrono>
#include <vector>
#include <string>
#include <fstream>
#include <geogram/basic/common.h>
#include <geogram/basic/logger.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/basic/stopwatch.h>
#include <geogram/basic/file_system.h>
#include <geogram/basic/process.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_topology.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/mesh/mesh_fill_holes.h>
#include <geogram/mesh/mesh_preprocessing.h>
#include <geogram/mesh/mesh_degree3_vertices.h>
#include <geogram/mesh/mesh_tetrahedralize.h>
#include <geogram/delaunay/delaunay.h>
#include <geogram/delaunay/periodic_delaunay_3d.h>
#include <geogram/voronoi/CVT.h>
#include <geogram/voronoi/RVD.h>
#include <geogram/voronoi/RVD_callback.h>
#include <geogram/voronoi/RVD_mesh_builder.h>
#include <geogram/voronoi/convex_cell.h>
#include <geogram/numerics/predicates.h>

using namespace std;

namespace {

using namespace GEO;

/**
 * \brief Removes zero area facets in a mesh
 * \param[in] M the input mesh
 */
void check_for_zero_area_facets( Mesh& M ) {
    GEO::vector<index_t> remove_f;
    vec3 q1( 0, 0, 0 );
    vec3 q2( 0, 0, 1 );
    vec3 q3( 0, 1, 0 );
    vec3 q4( 1, 0, 0 );
    for (index_t f = 0; f < M.facets.nb(); ++f) {
        index_t c = M.facets.corners_begin( f );
        index_t v1 = M.facet_corners.vertex( c );
        index_t v2 = M.facet_corners.vertex( c + 1 );
        index_t v3 = M.facet_corners.vertex( c + 2 );
        const vec3& p1 = Geom::mesh_vertex( M, v1 );
        const vec3& p2 = Geom::mesh_vertex( M, v2 );
        const vec3& p3 = Geom::mesh_vertex( M, v3 );

        // Colinearity is tested by using four coplanarity
        // tests with points q1,q2,q3,q4 that are
        // not coplanar.
        if (
            PCK::orient_3d( p1, p2, p3, q1 ) == 0.0 &&
            PCK::orient_3d( p1, p2, p3, q2 ) == 0.0 &&
            PCK::orient_3d( p1, p2, p3, q3 ) == 0.0 &&
            PCK::orient_3d( p1, p2, p3, q4 ) == 0.0
            ) {
            Logger::warn( "Validate" ) << "Found a zero-area facet"
                << std::endl;
            remove_f.resize( M.facets.nb(), 0 );
            remove_f[f] = 1;
        }
    }
    if (remove_f.size() != 0) {
        Logger::warn( "Validate" ) << "Removing zero-area facet(s)"
            << std::endl;
        M.facets.delete_elements( remove_f );
    }
}

/**
 * \brief The callback called for each RVD polyhedron. Constructs a
 *  mesh with the boundary of all cells.
 * \details Its member functions are called for each RVD polyhedron,
 *  i.e. the intersections between the volumetric mesh tetrahedra and
 *  the Voronoi cells. Based on set_simplify_xxx(), a smaller number of
 *  polyhedra can be generated.
 */
class SaveRVDCells : public RVDPolyhedronCallback {
public:

    /**
     * \brief SaveRVDCells constructor.
     * \param[out] output_mesh a reference to the generated mesh
     */
    SaveRVDCells( Mesh& output_mesh ) : output_mesh_( output_mesh ) {
        my_vertex_map_ = nullptr;

        // If set, then only one polyhedron per (connected component of) restricted Voronoi
        // cell is generated.
        set_simplify_internal_tet_facets( CmdLine::get_arg_bool( "RVD_cells:simplify_tets" ) );

        // If set, then only one polygon per Voronoi facet is generated. 
        set_simplify_voronoi_facets( CmdLine::get_arg_bool( "RVD_cells:simplify_voronoi" ) );

        // If set, then the intersection between a Voronoi cell and the boundary surface is
        // replaced with a single polygon whenever possible (i.e. when its topology is a
        // disk and when it has at least 3 corners).
        set_simplify_boundary_facets( CmdLine::get_arg_bool( "RVD_cells:simplify_boundary" ) );

        // If set, then the intersections are available as Mesh objects through the function
        // process_polyhedron_mesh(). Note that this is implied by simplify_voronoi_facets
        // or simplify_boundary.
        if (CmdLine::get_arg_double( "RVD_cells:shrink" ) != 0.0) {
            set_use_mesh( true );
        }
    }

    ~SaveRVDCells() override {
        delete my_vertex_map_;
        my_vertex_map_ = nullptr;
    }

    /**
     * \brief Called at the beginning of RVD traversal.
     */
    void begin() override {
        RVDPolyhedronCallback::begin();
        output_mesh_.clear();
        output_mesh_.vertices.set_dimension( 3 );
    }

    /**
     * \brief Called at the end of RVD traversal.
     */
    void end() override {
        RVDPolyhedronCallback::end();
        output_mesh_.facets.connect();
    }

    /**
     * \brief Called at the beginning of each RVD polyhedron.
     * \param[in] seed , tetrahedron the (seed,tetrahedron) pair that
     *  defines the RVD polyhedron, as the intersection between the Voronoi
     *  cell of the seed and the tetrahedron.
     */
    void begin_polyhedron( index_t seed, index_t tetrahedron ) override {
        geo_argused( tetrahedron );
        geo_argused( seed );

        //   The RVDVertexMap is used to map the symbolic representation of vertices
        // to indices. Here we reset indexing for each new cell, so that vertices shared
        // by the faces of two different cells will be duplicated. We do that because
        // we construct the boundary of the cells in a surfacic mesh (for visualization
        // purposes). Client code that has a data structure for polyhedral volumetric mesh
        // will not want to reset indexing (and will comment-out the following three lines).
        // It will also construct the RVDVertexMap in the constructor.

        delete my_vertex_map_;
        my_vertex_map_ = new RVDVertexMap;
        my_vertex_map_->set_first_vertex_index( output_mesh_.vertices.nb() );
    }

    /**
     * \brief Called at the beginning of each RVD polyhedron.
     * \param[in] facet_seed if the facet is on a Voronoi bisector,
     *  the index of the Voronoi seed on the other side of the bisector,
     *  else index_t(-1)
     * \param[in] facet_tet if the facet is on a tethedral facet, then
     *  the index of the tetrahedron on the other side, else index_t(-1)
     */
    void begin_facet( index_t facet_seed, index_t facet_tet ) override {
        geo_argused( facet_seed );
        geo_argused( facet_tet );
        current_facet_.resize( 0 );
    }

    void vertex(
        const double* geometry, const GEOGen::SymbolicVertex& symb
    ) override {
        // Find the index of the vertex associated with its symbolic representation.
        index_t vid = my_vertex_map_->find_or_create_vertex( seed(), symb );

        // If the vertex does not exist in the mesh, create it.
        if (vid >= output_mesh_.vertices.nb()) {
            output_mesh_.vertices.create_vertex( geometry );
        }

        // Memorize the current facet.
        current_facet_.push_back( vid );
    }

    void end_facet() override {
        // Create the facet from the memorized indices.
        index_t f = output_mesh_.facets.nb();
        output_mesh_.facets.create_polygon( current_facet_.size() );
        for (index_t i = 0; i < current_facet_.size(); ++i) {
            output_mesh_.facets.set_vertex( f, i, current_facet_[i] );
        }
    }

    void end_polyhedron() override {
        // Nothing to do.
    }

    void process_polyhedron_mesh() override {
        // This function is called for each cell if set_use_mesh(true) was called.
        // It is the case if simplify_voronoi_facets(true) or
        // simplify_boundary_facets(true) was called.
        //   Note1: most users will not need to overload this function (advanded use
        //   only).
        //   Note2: mesh_ is managed internally by RVDPolyhedronCallback class, as an
        // intermediary representation to store the cell before calling the callbacks.
        // It is distinct from the output_mesh_ constructed by the callbacks.

        //   The current cell represented by a Mesh can be 
        // filtered/modified/post-processed (member variable mesh_)
        // here, before calling base class's implementation.
        //   As an example, we shrink the cells. More drastic modifications/
        // transformations of the mesh can be done (see base class's implementation
        // in geogram/voronoi/RVD_polyhedron_callback.cpp).

        double shrink = CmdLine::get_arg_double( "RVD_cells:shrink" );
        if (shrink != 0.0 && mesh_.vertices.nb() != 0) {
            vec3 center( 0.0, 0.0, 0.0 );
            for (index_t v = 0; v < mesh_.vertices.nb(); ++v) {
                center += vec3( mesh_.vertices.point_ptr( v ) );
            }
            center = (1.0 / double( mesh_.vertices.nb() )) * center;
            for (index_t v = 0; v < mesh_.vertices.nb(); ++v) {
                vec3 p( mesh_.vertices.point_ptr( v ) );
                p = shrink * center + (1.0 - shrink) * p;
                mesh_.vertices.point_ptr( v )[0] = p.x;
                mesh_.vertices.point_ptr( v )[1] = p.y;
                mesh_.vertices.point_ptr( v )[2] = p.z;
            }
        }

        //  The default implementation simplifies Voronoi facets
        // and boundary mesh facets based on the boolean flags
        // defined by set_simplify_xxx(). Then it calls the callbacks
        // for each mesh facet.
        RVDPolyhedronCallback::process_polyhedron_mesh();

    }

private:
    GEO::vector<index_t> current_facet_;
    Mesh& output_mesh_;
    RVDVertexMap* my_vertex_map_;
};

void compute_RVD_cells( RestrictedVoronoiDiagram* RVD, Mesh& RVD_mesh ) {
    SaveRVDCells callback( RVD_mesh );
    RVD->for_each_polyhedron( callback );
}

}

WeightedSamplePointsResult WeightedSamplePoints( const std::string& mesh, int num, int lloyd, float sdf_step )
{
    using namespace GEO;

    static bool init = false;
    if (!init)
    {
        GEO::initialize();
        CmdLine::import_arg_group( "standard" );
        CmdLine::import_arg_group( "algo" );
        CmdLine::declare_arg( "volumetric", false, "volumetric or surfacic RVD" );
        CmdLine::declare_arg(
            "cell_borders", false, "generate only cell borders"
        );
        CmdLine::declare_arg(
            "integration_smplx", false,
            "in volumetric mode, generate integration simplices"
        );
        CmdLine::declare_arg( "RDT", false, "save RDT" );
        CmdLine::declare_arg( "RVD", true, "save RVD" );
        CmdLine::declare_arg( "RVD_cells", false, "use new API for computing RVD cells (implies volumetric)" );
        CmdLine::declare_arg_group( "RVD_cells", "RVD cells simplification flags" );
        CmdLine::declare_arg( "RVD_cells:simplify_tets", true, "Simplify tets intersections" );
        CmdLine::declare_arg( "RVD_cells:simplify_voronoi", true, "Simplify Voronoi facets" );
        CmdLine::declare_arg( "RVD_cells:simplify_boundary", false, "Simplify boundary facets" );
        CmdLine::declare_arg( "RVD_cells:shrink", 0.0, "Shrink factor for computed cells" );
        init = true;
    }

    WeightedSamplePointsResult result;
    try {

        Stopwatch Wtot( "Total time" );

        std::string mesh_filename = mesh;
        std::string output_filename = "";

        Mesh M_in;

        MeshIOFlags flags;
        flags.set_element( MESH_CELLS );
        if (!mesh_load( mesh_filename, M_in, flags )) {
            return result;
        }
        mesh_tetrahedralize( M_in, false, true );

        auto mesh2 = std::make_unique<HalfEdgeMesh>( mesh_filename );

        SDF sdf = CreateSDF( mesh_filename, sdf_step );

        WeightedCVT cvt( &M_in, mesh2.get(), &sdf );
        cvt.set_volumetric( true );
        cvt.delaunay()->set_stores_neighbors( true );
        cvt.delaunay()->set_keeps_infinite( true );
        cvt.compute_initial_sampling( num );

        cvt.WeightedLloyd( lloyd );

        result.border_flags.resize( cvt.delaunay()->nb_vertices() );
        result.sdf.resize( cvt.delaunay()->nb_vertices() );
        result.sdfgrad.resize( cvt.delaunay()->nb_vertices() );
        for (int i = 0; i < cvt.delaunay()->nb_vertices(); i++)
        {
            GEO::vector<GEO::index_t> cur_nei;
            cvt.delaunay()->get_neighbors_internal( i, cur_nei );
            result.nei_lists.push_back( std::vector<int>( cur_nei.begin(), cur_nei.end() ) );
            result.border_flags[i] = cvt._border_flags[i];
            float sdfv = sdf( cvt._pts[i].x, cvt._pts[i].y, cvt._pts[i].z );

            result.sdf[i] = std::abs( sdfv );
            Vec3f grad = sdf.Grad( cvt._pts[i].x, cvt._pts[i].y, cvt._pts[i].z );
            result.sdfgrad[i] = glm::vec3( grad[0], grad[1], grad[2] );
        }

        result.pts = cvt._pts;
        result.radius = cvt._radius;
    }
    catch (const std::exception& e) {
        std::cerr << "Received an exception: " << e.what() << std::endl;
        return result;
    }

    return result;
}

std::vector<float> WeightedCVT::_weights;
HalfEdgeMesh* WeightedCVT::_mesh2;
HalfEdgeSurfaceTester WeightedCVT::tester;
std::vector<std::mutex> WeightedCVT::_cell_mutex;
std::vector<int> WeightedCVT::_border_flags;
SDF* WeightedCVT::_sdf;

WeightedCVT::WeightedCVT( GEO::Mesh* mesh, HalfEdgeMesh* mesh2, SDF* sdf )
    :GEO::CentroidalVoronoiTesselation( mesh, 3, "tetgen" )
{
    RVD_ = new GEO::WeightedRVTImpl( delaunay_, mesh, mesh->vertices.point_ptr( 0 ), 3, mesh2 );
    WeightedCVT::_mesh2 = mesh2;
    _sdf = sdf;
    tester.SetSurface( mesh2 );
}

void WeightedCVT::WeightedLloyd( int nb_iter )
{
    int nb_points = int( points_.size() / dimension_ );
    std::vector<double> mg;
    std::vector<double> m;
    mg.assign( nb_points * dimension_, 0.0 );
    m.assign( nb_points, 0.0 );
    _weights.resize( nb_points );
    std::fill( _weights.begin(), _weights.end(), 0.f );

    _cell_mutex = std::vector<std::mutex>( nb_points );
    _border_flags.clear();
    _border_flags.resize( nb_points, 1 );
    RVD_->set_check_SR( false );

    delaunay_->set_vertices( nb_points, points_.data() );
    RVD_->compute_centroids( mg.data(), m.data() );
    _pts.clear();
    _radius.clear();
    int cur = 0;
    for (int j = 0; j < nb_points; j++) {
        _pts.push_back( glm::vec3( points_[cur], points_[cur + 1], points_[cur + 2] ) );
        _radius.push_back( std::pow( m[j] * 3 / 4 / 3.14, 1.0 / 3.0 ) );

        cur += dimension_;
    }

    cur_iter_ = 0;
    nb_iter_ = nb_iter;

    for (int i = 0; i < nb_iter; i++) {
        _weights.resize( nb_points );
        std::fill( _weights.begin(), _weights.end(), 0.f );
        _pts.clear();
        _radius.clear();
        _border_flags.clear();
        _border_flags.resize( nb_points, 1 );

        //UpdateWeights();
        mg.assign( nb_points * dimension_, 0.0 );
        m.assign( nb_points, 0.0 );
        delaunay_->set_vertices( nb_points, points_.data() );
        RVD_->compute_centroids( mg.data(), m.data() );
        cur = 0;
        for (int j = 0; j < nb_points; j++) {
            if (m[j] > 1e-30 && !point_is_locked( j )) {
                double s = 1.0 / _weights[j];
                for (int coord = 0; coord < dimension_; coord++) {
                    points_[cur + coord] = s * mg[cur + coord];
                }
            }

            glm::vec3 p = glm::vec3( points_[cur], points_[cur + 1], points_[cur + 2] );
            _pts.push_back( p );
            _radius.push_back( std::pow( m[j] * 3 / 4 / 3.14, 1.0 / 3.0 ) );
            cur += dimension_;
        }

        newiteration();
    }
}

void WeightedCVT::WritePoints( const std::string& path )
{
    std::ofstream ofs( path );
    int nb = points_.size() / 3;
    for (int i = 0; i < nb; i++)
    {
        ofs << points_[i * 3] << ' ' << points_[i * 3 + 1] << ' ' << points_[i * 3 + 2] << '\n';
    }
    ofs.flush();
    ofs.close();
}
