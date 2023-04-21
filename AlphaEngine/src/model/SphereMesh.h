#pragma once
#include <algorithm>
#include <numeric>
#include <omp.h>
#include <boost/container_hash/hash.hpp>
#include <glm/glm.hpp>
#include <tinycolormap.hpp>
#include <CGAL/Simple_cartesian.h>
#include "acceleration/AABB.h"
#include "CVT/WeightedCVT.h"
#include "input/Input.h"
#include "material/Material.h"
#include "model/GLLineSegment.h"
#include "model/HalfEdgeMesh.h"
#include "model/ModelLoader.h"
#include "util/Scene.h"
#include "util/SceneObject.h"
#include "util/util.h"

using Kernel = CGAL::Simple_cartesian<float>;
using CGALSphere = Kernel::Sphere_3;
using CGALPoint = Kernel::Point_3;
using CGALLine = Kernel::Line_3;
using CGALSeg = Kernel::Segment_3;

//TODO: Move this and all duplicate classes to somewhere else. Maybe replace with boost::hash.
class IndexPair
{
public:
    int i0;
    int i1;

    class Hash
    {
    public:
        unsigned operator()( const IndexPair& pair ) const
        {
            return pair.i0 > pair.i1 ?
                boost::hash<std::pair<int, int>>()({ pair.i0, pair.i1 }) :
                boost::hash<std::pair<int, int>>()({ pair.i1, pair.i0 });
        }
    };

    class Pred
    {
    public:
        bool operator()( const IndexPair& pair1, const IndexPair& pair2 ) const
        {
            return ((pair1.i0 == pair2.i0 && pair1.i1 == pair2.i1) ||
                (pair1.i0 == pair2.i1 && pair1.i1 == pair2.i0));
        }
    };
};

class SphereBase;

template <typename T>
concept SphereType = requires (T a) { std::is_base_of_v<SphereBase, T>; };

class SphereBase
{
public:
    class HashX
    {
    public:
        size_t operator()( const SphereBase& s ) const
        {
            return boost::hash<std::tuple<float, float, float>>()({ s.x.x, s.x.y, s.x.z });
        }
    };

    class HashX0
    {
    public:
        size_t operator()( const SphereBase& s ) const
        {
            return boost::hash<std::tuple<float, float, float>>()({ s.x0.x, s.x0.y, s.x0.z });
        }
    };

    class PredX
    {
    public:
        bool operator()( const SphereBase& s1, const SphereBase& s2 ) const
        {
            return s1.x == s2.x;
        }
    };

    class PredX0
    {
    public:
        bool operator()( const SphereBase& s1, const SphereBase& s2 ) const
        {
            return s1.x0 == s2.x0;
        }
    };

    bool deleted = false;
    glm::vec3 x = glm::vec3( 0.f );
    glm::vec3 x0 = glm::vec3( 0.f );
    glm::quat q = glm::quat( 0, 0, 0, 0 );
    glm::quat q0 = glm::quat( 0, 0, 0, 0 );
    float r = 0.f;
    float m = 0.f;
    glm::vec3 color = glm::vec3( 0.9f );
    std::vector<int> neighbors;
};

template <SphereType Sphere>
class SphereTreeNode
{
public:
    std::array<SphereTreeNode*, 8> children;
    Sphere metaball;

    SphereTreeNode( const std::vector<Sphere>& ball_list, int idx );
    ~SphereTreeNode();
    void GetLeafNodes( std::vector<Sphere>& balls );
};

template <SphereType Sphere>
SphereTreeNode<Sphere>::SphereTreeNode( const std::vector<Sphere>& ball_list, int idx )
{
    metaball = ball_list[idx];
    children.fill( nullptr );
    for (unsigned i = 1; i < 9; i++)
    {
        unsigned child_idx = idx * 8 + i;
        if (child_idx < ball_list.size() && ball_list[child_idx].r > 0)
        {
            children[i - 1] = new SphereTreeNode<Sphere>( ball_list, child_idx );
        }
    }
}

template <SphereType Sphere>
SphereTreeNode<Sphere>::~SphereTreeNode()
{
    for (int i = 0; i < 8; i++)
    {
        delete children[i];
    }
}

template <SphereType Sphere>
void SphereTreeNode<Sphere>::GetLeafNodes( std::vector<Sphere>& balls )
{
    bool have_child = false;
    for (int i = 0; i < 8; i++)
    {
        if (children[i])
        {
            have_child = true;
            children[i]->GetLeafNodes( balls );
        }
    }

    if (!have_child)
    {
        balls.push_back( metaball );
    }
}

template <SphereType Sphere>
class SphereMesh :
    public SceneObject
{
    /* Constructor & Assignment */
public:
    SphereMesh();
    SphereMesh( const std::vector<Sphere>& balls );
    SphereMesh( std::vector<Sphere>&& balls );
    SphereMesh( const std::vector<std::pair<glm::vec3, float>>& balls );
    ~SphereMesh() = default;
protected:
    SphereMesh( const SphereMesh<Sphere>& rh );
    SphereMesh( SphereMesh<Sphere>&& rh );
    SphereMesh<Sphere>& operator=( const SphereMesh<Sphere>& rh );
    SphereMesh<Sphere>& operator=( SphereMesh<Sphere>&& rh );

    /* Member Function */
public:
    // Inherited via SceneObject
    virtual void Update() override;
    virtual void Draw() override;

    Sphere& Ball( int id );
    const Sphere& Ball( int id ) const;
    void LoadFromSphereTreeFile( const std::string& path );
    void CreateFromSurfaceVoroOptimize( HalfEdgeMesh* mesh, std::string mesh_path, int nb_ball, int nb_lloyd, float step_len, int min_nb_neighbors = 8 );
    void CreateFromSurfaceUniform( HalfEdgeMesh* mesh, float steplen, int min_nb_neighbors = 8 );
    int BallsNum() const;
    void MakeConnections();
    void MakeConnections2();
    void ConnectionReduction();
    void Optimize();
    void UpdateConnectionsGLBuffer();
    void UpdateOritBuffer();
    void Clear();
    void AddBall( glm::vec3 x, float r );
    void DeleteCompleteInsideBalls();
    void DeleteMarkedBalls();
    template <SphereType Sphere2>
    float Distance( const SphereMesh<Sphere2>& other );
    Sphere& operator[]( unsigned int id );
    const Sphere& operator[]( unsigned int id ) const;

private:
    //virtual SceneObject* virtual_clone() const override;

private:
    std::vector<Sphere> _balls;
    std::unique_ptr<HalfEdgeMesh> _ballmesh;

    bool _show_edges = false;
    bool _show_ball = true;
    bool _show_orit = false;
    std::unique_ptr<GLLineSegment> _ball_edges = nullptr;
    std::unique_ptr<GLLineSegment> _ball_orit_lines = nullptr;
};

template<SphereType Sphere>
inline SphereMesh<Sphere>::SphereMesh()
    :_ballmesh( std::make_unique<HalfEdgeMesh>( "res/models/ball960.obj" ) )
{
    _ballmesh->Normalize();
    _ballmesh->UpdatePosBuffer();
    _ball_edges = std::make_unique<GLLineSegment>();
    _ball_orit_lines = std::make_unique<GLLineSegment>();
}

template<SphereType Sphere>
inline SphereMesh<Sphere>::SphereMesh( const std::vector<Sphere>& balls )
    : SphereMesh()
{
    _balls = balls;
    MakeConnections();
}

template<SphereType Sphere>
inline SphereMesh<Sphere>::SphereMesh( std::vector<Sphere>&& balls )
    : SphereMesh()
{
    _balls = std::move( balls );
    MakeConnections();
}

template<SphereType Sphere>
inline SphereMesh<Sphere>::SphereMesh( const std::vector<std::pair<glm::vec3, float>>& balls )
    :SphereMesh()
{
    _balls.clear();
    for (auto& pair : balls)
    {
        Sphere s;
        s.x = pair.first;
        s.x0 = pair.first;
        s.r = pair.second;
        _balls.push_back( s );
    }
    MakeConnections();
}

template<SphereType Sphere>
inline SphereMesh<Sphere>::SphereMesh( const SphereMesh<Sphere>& rh )
    : SphereMesh()
{
    _balls = rh._balls;
}

template<SphereType Sphere>
inline SphereMesh<Sphere>::SphereMesh( SphereMesh<Sphere>&& rh )
    :SphereMesh()
{
    _balls = std::move( rh._balls );
}

template<SphereType Sphere>
inline SphereMesh<Sphere>& SphereMesh<Sphere>::operator=( const SphereMesh<Sphere>& rh )
{
    _balls = rh._balls;
    return *this;
}

template<SphereType Sphere>
inline SphereMesh<Sphere>& SphereMesh<Sphere>::operator=( SphereMesh<Sphere>&& rh )
{
    _balls = rh._balls;
    return *this;
}

template<SphereType Sphere>
inline void SphereMesh<Sphere>::Update()
{
}

template<SphereType Sphere>
inline void SphereMesh<Sphere>::Draw()
{
    static bool temp2 = false;
    if (Input::IsKeyDown( Input::Key::H ))
        temp2 = !temp2;
    if (_show_ball)
    {
        //TODO: Add a instanced drawing class for this.
        for (int i = 0; i < _balls.size(); i++)
        {
            auto& ball = _balls[i];
            _ballmesh->mTransform.SetPos( ball.x );
            float r = ball.r;
            _ballmesh->mTransform.SetScale( glm::vec3( r * 2 ) );
            Scene::active->_transform_ubo_info.world_mat = _ballmesh->mTransform.GetModelMat();
            Scene::active->_transform_ubo->SetData( sizeof( Scene::active->_transform_ubo_info ), &Scene::active->_transform_ubo_info, GL_DYNAMIC_DRAW );
            auto mapcolor = tinycolormap::GetParulaColor( glm::pow( ball.m_rel, 1.0 / 3.0 ) );
            _ballmesh->_material_main->mDiffuseColor = ball.color;
            _ballmesh->Draw();
        }
    }

    if (_show_edges)
    {
        UpdateConnectionsGLBuffer();
        _ball_edges->Draw();
    }

    if (_show_orit)
    {
        UpdateOritBuffer();
        _ball_orit_lines->Draw();
    }
}

template<SphereType Sphere>
inline Sphere& SphereMesh<Sphere>::Ball( int id )
{
    return _balls[id];
}

template<SphereType Sphere>
inline const Sphere& SphereMesh<Sphere>::Ball( int id ) const
{
    return _balls[id];
}

template<SphereType Sphere>
inline void SphereMesh<Sphere>::LoadFromSphereTreeFile( const std::string& path )
{
    std::ifstream ifs( path, std::ios_base::in );
    std::string first_line;
    std::getline( ifs, first_line );
    std::stringstream ss( first_line );

    int depth = 0;
    int degree = 0;
    ss >> depth >> degree;

    if (depth <= 0 || degree <= 0)
    {
        throw std::runtime_error( "File format incorrect" );
    }

    int node_num = 0;
    for (size_t i = 0; i < depth; i++)
    {
        node_num += (int)glm::pow( degree, i );
    }

    std::vector<Sphere> metaballs;
    for (size_t i = 0; i < node_num; i++)
    {
        std::string line;
        std::getline( ifs, line );
        std::stringstream ss( line );
        float x, y, z, r;
        ss >> x >> y >> z >> r;
        Sphere s;
        s.x = glm::vec3( x, y, z );
        s.x0 = s.x;
        s.r = r;
        metaballs.push_back( s );
    }

    SphereTreeNode<Sphere> root( metaballs, 0 );
    metaballs.clear();
    root.GetLeafNodes( metaballs );

    std::unordered_set<Sphere, Sphere::HashX0, Sphere::PredX0> ballset;
    ballset.insert( metaballs.begin(), metaballs.end() );
    _balls = std::vector<Sphere>( ballset.begin(), ballset.end() );

    MakeConnections();
}

template<SphereType Sphere>
inline void SphereMesh<Sphere>::CreateFromSurfaceVoroOptimize( HalfEdgeMesh* mesh, std::string mesh_path, int nb_ball, int nb_lloyd, float steplen, int min_nb_neighbors )
{
    Clear();

    WeightedSamplePointsResult result = WeightedSamplePoints( mesh_path, nb_ball, nb_lloyd, steplen );

    std::vector<glm::vec3>& pts = result.pts;
    std::vector<float>& radius = result.radius;
    std::vector<std::vector<int>>& nei_lists = result.nei_lists;
    std::vector<int>& is_border = result.border_flags;
    std::vector<float>& sdf = result.sdf;
    std::vector<glm::vec3>& sdfgrad = result.sdfgrad;

    for (int i = 0; i < pts.size(); i++)
    {
        float r = radius[i];
        AddBall( pts[i], r );
        _balls.back().neighbors = nei_lists[i];
        _balls.back().isborder = is_border[i] == -1;
        _balls.back().sdf = sdf[i];
        _balls.back().outside = glm::normalize( sdfgrad[i] );
        _ball_orit_lines->AddPoint( pts[i], glm::vec3( 1, 0, 0 ) );
        _ball_orit_lines->AddPoint( pts[i] + Ball( BallsNum() - 1 ).outside * 0.1f, glm::vec3( 1.f ) );
    }
    _ball_orit_lines->UpdateMem();
    HalfEdgeSurfaceTester tester( mesh );
#pragma omp parallel for
    for (int i = 0; i < BallsNum(); i++)
    {
        auto& ball = Ball( i );
        int face_id = -1;
        float dist = tester.MinDistToSurface( ball.x, &face_id );
        bool in_surface = tester.PointIsInSurface( ball.x );
        if (face_id >= 0)
        {
            glm::vec3 normal = mesh->GetFaceNormal( face_id );
            if (in_surface)
            {
                if (ball.r > dist)
                    ball.x -= normal * (ball.r - dist);
            }
            else if (!in_surface)
            {
                ball.x -= normal * (ball.r + dist);
            }
            ball.x0 = ball.x;
        }
    }

    //_mesh->Optimize();
    //_mesh->MakeConnections2();

    ConnectionReduction();

    //Make sure every ball has a minimum number of neighbors.
    for (int i = 0; i < BallsNum(); i++)
    {
        std::vector<int>& neighbors = Ball( i ).neighbors;
        if (neighbors.size() >= min_nb_neighbors)
            continue;

        std::multimap<float, int> min_heap;
        for (int j = 0; j < BallsNum(); j++)
        {
            if (i == j)
                continue;
            if (std::find( std::begin( neighbors ), std::end( neighbors ), j ) != std::end( neighbors ))
                continue;

            float dist = glm::distance( Ball( i ).x0, Ball( j ).x0 );
            min_heap.insert( { dist, j } );
        }

        int counter = min_nb_neighbors - neighbors.size();
        for (auto& pair : min_heap)
        {
            if (counter <= 0)
                break;
            neighbors.push_back( pair.second );
            counter--;
        }
    }

    //make sure that every link is double sided
    for (int i = 0; i < BallsNum(); i++)
    {
        std::vector<int>& neilist_i = Ball( i ).neighbors;
        for (int nei : neilist_i)
        {
            std::vector<int>& neilist_nei = Ball( nei ).neighbors;
            if (std::find( neilist_nei.begin(), neilist_nei.end(), i ) == neilist_nei.end())
            {
                neilist_nei.push_back( i );
            }
        }
    }
}

template<SphereType Sphere>
inline void SphereMesh<Sphere>::CreateFromSurfaceUniform( HalfEdgeMesh* mesh, float steplen, int min_nb_neighbors )
{
    auto start_t = std::chrono::high_resolution_clock::now();
    AABB aabb;
    for (int i = 0; i < mesh->_vertices.size(); i++)
    {
        aabb.Expand( mesh->_vertices[i].pos );
    }

    Clear();

    HalfEdgeSurfaceTester tester( mesh );

    const int dimx = (aabb.max_corner.x - aabb.min_corner.x) / steplen + 1;
    const int dimy = (aabb.max_corner.y - aabb.min_corner.y) / steplen + 1;
    const int dimz = (aabb.max_corner.z - aabb.min_corner.z) / steplen + 1;

    int count = 0;
    for (int ix = 0; ix < dimx; ++ix)
    {
        for (int iy = 0; iy < dimy; ++iy)
        {
            for (int iz = 0; iz < dimz; ++iz)
            {
                const float px = aabb.min_corner.x + ix * steplen;
                const float py = aabb.min_corner.y + iy * steplen;
                const float pz = aabb.min_corner.z + iz * steplen;
                const glm::vec3 x( px, py, pz );
                if (tester.PointIsInSurface( x ))
                {
                    float r = glm::pow( 0.2387 * steplen * steplen * steplen, 1.0 / 3.0 );
                    AddBall( x, r );
                }
            }
        }
    }

    std::cout << "ParticleNum=" << BallsNum() << std::endl;

    for (int i = 0; i < BallsNum(); i++)
    {
        std::vector<int>& neighbors = _balls[i].neighbors;

        std::multimap<float, int> min_heap;
        for (int j = 0; j < BallsNum(); j++)
        {
            if (i == j)
                continue;
            if (std::find( std::begin( neighbors ), std::end( neighbors ), j ) != std::end( neighbors ))
                continue;

            float dist = glm::distance( _balls[i].x0, _balls[j].x0 );
            min_heap.insert( { dist, j } );
        }

        int counter = min_nb_neighbors - neighbors.size();
        for (auto& pair : min_heap)
        {
            if (counter <= 0)
                break;
            neighbors.push_back( pair.second );
            counter--;
        }
    }
}

template<SphereType Sphere>
inline int SphereMesh<Sphere>::BallsNum() const
{
    return _balls.size();
}

template<SphereType Sphere>
inline void SphereMesh<Sphere>::MakeConnections()
{
    static const int N = 128;
    static const int MIN_N = 8;

    std::unordered_set<IndexPair, IndexPair::Hash, IndexPair::Pred> idx_pairs;
    for (int i = 0, size = _balls.size(); i < size; i++)
    {
        Sphere& ball = _balls[i];
        if (ball.deleted)
            continue;

        std::multimap<float, int>  max_heap;

        ball.neighbors.clear();

        for (int j = 0; j < size; j++)
        {
            if (i == j)
                continue;

            const Sphere& ball2 = _balls[j];
            if (ball2.deleted)
                continue;

            float d = glm::distance( ball.x0, ball2.x0 );
            float diff = d - (ball.r + ball2.r);
            max_heap.insert( std::make_pair( diff, j ) );
        }

        int count = 0;
        for (auto& pair : max_heap)
        {
            if (count < MIN_N)
            {
                idx_pairs.insert( { i, pair.second } );
                count++;
            }
            else if (pair.first < 0 && count <= N)
            {
                idx_pairs.insert( { i, pair.second } );
                count++;
            }
            else
            {
                break;
            }
        }
    }

    for (const IndexPair& pair : idx_pairs)
    {
        _balls[pair.i0].neighbors.push_back( pair.i1 );
        _balls[pair.i1].neighbors.push_back( pair.i0 );
    }
}

template<SphereType Sphere>
inline void SphereMesh<Sphere>::MakeConnections2()
{
    static const int N = 32;
    static const int MIN_N = 8;

    std::vector<std::vector<int>> direct_neis;
    for (int i = 0, size = _balls.size(); i < size; i++)
    {
        Sphere& ball = _balls[i];
        if (ball.deleted)
            continue;

        std::vector<int> neighbors;

        for (int j = 0; j < size; j++)
        {
            const Sphere& ball2 = _balls[j];
            if (i == j || ball2.deleted)
                continue;

            if (glm::distance( ball2.x0, ball.x0 ) < (ball2.r + ball.r))
            {
                neighbors.push_back( j );
            }
        }

        direct_neis.push_back( neighbors );
    }

    std::unordered_set<IndexPair, IndexPair::Hash, IndexPair::Pred> idx_pairs;
    for (int i = 0, size = _balls.size(); i < size; i++)
    {
        Sphere& ball = _balls[i];
        if (ball.deleted)
            continue;

        std::multimap<float, int>  max_heap;

        ball.neighbors.clear();
        std::unordered_set<int> candidates;
        for (int j : direct_neis[i])
        {
            candidates.insert( j );
            for (int neiofj : direct_neis[j])
            {
                candidates.insert( neiofj );
            }
        }

        for (int j : candidates)
        {
            if (i == j)
                continue;

            const Sphere& ball2 = _balls[j];
            if (ball2.deleted)
                continue;

            float d = glm::distance( ball.x0, ball2.x0 );
            float diff = d - (ball.r + ball2.r);
            max_heap.insert( std::make_pair( diff, j ) );
        }

        int count = 0;
        for (auto& pair : max_heap)
        {
            if (count < MIN_N)
            {
                idx_pairs.insert( { i, pair.second } );
                count++;
            }
            else if (pair.first < 0 && count <= N)
            {
                idx_pairs.insert( { i, pair.second } );
                count++;
            }
            else
            {
                break;
            }
        }
    }

    for (const IndexPair& pair : idx_pairs)
    {
        _balls[pair.i0].neighbors.push_back( pair.i1 );
        _balls[pair.i1].neighbors.push_back( pair.i0 );
    }
}

template<SphereType Sphere>
inline void SphereMesh<Sphere>::ConnectionReduction()
{
#pragma omp parallel for
    for (int i = 0; i < BallsNum(); i++)
    {
        Sphere& ball = Ball( i );
        auto remove_it = std::remove_if( ball.neighbors.begin(), ball.neighbors.end(),
            [&]( int nei ) {
                float dist = glm::distance( ball.x0, Ball( nei ).x0 );
                return dist > ( ball.r + Ball( nei ).r ) * 2;
            } );
        ball.neighbors.erase( remove_it, ball.neighbors.end() );

        continue;
        int max_num = 6 + ball.m_rel * 8;
        while (ball.neighbors.size() > max_num)
        {
            int selected_nei = -1;
            float min_var = FLT_MAX;

            for (int j : ball.neighbors)
            {
                float var = 0.f;
                glm::vec3 mean( 0.f );
                for (int k : ball.neighbors)
                {
                    if (k == j)
                        continue;
                    glm::vec3 d = Ball( k ).x0 - ball.x0;
                    float dist = glm::length2( d );
                    d /= dist;
                    d *= Ball( k ).m_rel;
                    mean += d;
                }
                mean /= ball.neighbors.size() - 1;

                for (int k : ball.neighbors)
                {
                    if (k == j)
                        continue;
                    glm::vec3 d = Ball( k ).x0 - ball.x0;
                    float dist = glm::length2( d );
                    d /= dist;
                    d *= Ball( k ).m_rel;
                    var += glm::length2( d - mean );
                }

                var /= ball.neighbors.size() - 2;
                if (var < min_var)
                {
                    selected_nei = j;
                    min_var = var;
                }
            }

            ball.neighbors.erase( std::remove( ball.neighbors.begin(), ball.neighbors.end(), selected_nei ), ball.neighbors.end() );
        }
    }

    //HalfEdgeSurfaceTester tester( _coarse_surface.get() );
    //#pragma omp parallel for
    //    for (int i = 0; i < _mesh->BallsNum(); i++)
    //    {
    //        glm::vec3 xi = _mesh->Ball( i ).x0;
    //        auto& neis = _mesh->Ball( i ).neighbors;
    //        for (int j : neis)
    //        {
    //            glm::vec3 xj = _mesh->Ball( j ).x0;
    //            if (tester.LinesegIntersect( xi, xj ))
    //            {
    //                neis.erase( std::remove( neis.begin(), neis.end(), j ), neis.end() );
    //            }
    //        }
    //    }
}

template<SphereType Sphere>
inline void SphereMesh<Sphere>::Optimize()
{
    constexpr float THRESHOLD = 2.f;
    constexpr float MULTIPLIER = 1.1f;

    int ball_num = _balls.size();
    float avg_r = 0.f;
    for (int i = 0; i < ball_num; ++i)
    {
        avg_r += _balls[i].r;
    }
    avg_r /= _balls.size();

    float x = glm::cos( glm::radians( 60.f ) );
    float z = glm::sin( glm::radians( 60.f ) );
    float y = glm::sqrt( 1 - (z * 2.f / 3.f) * (z * 2.f / 3.f) );

    //std::array<glm::vec3, 13> directions = {
    //    glm::vec3( -1, 0, 0 ),
    //    glm::vec3( -x, 0, z ),
    //    glm::vec3( x, 0, z ),
    //    glm::vec3( 1, 0, 0 ),
    //    glm::vec3( x, 0, -z ),
    //    glm::vec3( -x, 0, -z ),
    //    glm::vec3( 0, y, z * 2.f / 3.f ),
    //    glm::vec3( x, y, -z / 3.f ),
    //    glm::vec3( -x, y, -z / 3.f ) ,
    //    glm::vec3( 0, -y, -z * 2.f / 3.f ),
    //    glm::vec3( x, -y, -z / 3.f ),
    //    glm::vec3( -x, -y, -z / 3.f ),
    //    glm::vec3( 0, 0, 0 )
    //};

    std::array<glm::vec3, 8> directions = {
        glm::normalize( glm::vec3( 1, 1, 1 ) ),
        glm::normalize( glm::vec3( 1, -1, 1 ) ),
        glm::normalize( glm::vec3( -1, 1, 1 ) ),
        glm::normalize( glm::vec3( -1, -1, 1 ) ),
        glm::normalize( glm::vec3( 1, 1, -1 ) ),
        glm::normalize( glm::vec3( 1, -1, -1 ) ),
        glm::normalize( glm::vec3( -1, 1, -1 ) ),
        glm::normalize( glm::vec3( -1, -1, -1 ) )
    };



    for (int i = 0; i < ball_num; ++i)
    {
        Sphere& ball = _balls[i];

        float avg_dist = 0.f;
        for (int j : ball.neighbors)
        {
            avg_dist += glm::length( _balls[j].x0 - ball.x0 );
        }
        avg_dist /= (ball.neighbors.size());

        glm::mat3 A;
        for (int j : ball.neighbors)
        {
            glm::vec3 xij = _balls[j].x0 - ball.x0;
            float r = glm::length( xij );
            float h = avg_dist * 3;
            float w = 0.f;
            if (r < h)
                w = 315.f * glm::pow( h * h - r * r, 3 ) / (64.f * glm::pi<float>() * glm::pow( h, 9 ));
            else
                w = 0.f;

            A += w * glm::TensorProduct( xij, xij );
        }

        float detA = glm::abs( glm::determinant( A ) );
        if (detA > 1.0f)
        {
            ball.color = glm::vec3( 0, 1, 1 );
            continue;
        }
        float newr = (ball.r / 2.f) * MULTIPLIER;
        for (glm::vec3 dir : directions)
        {
            Sphere s;
            s.x0 = ball.x0 + dir * newr;
            s.r = newr;
            s.x = s.x0;
            s.color = glm::vec3( 0, 1, 0 );

            _balls.push_back( s );
        }

        _balls[i].r = -1;
    }

    DeleteCompleteInsideBalls();

    std::cout << "After opt: " << _balls.size() << std::endl;
}

template<SphereType Sphere>
inline void SphereMesh<Sphere>::UpdateConnectionsGLBuffer()
{
    _ball_edges->Clear();
    for (auto& ball : _balls)
    {
        if (ball.deleted)
            continue;
        for (auto& i : ball.neighbors)
        {
            auto& ball2 = _balls[i];
            if (ball2.deleted)
                continue;
            _ball_edges->AddPoint( ball.x );
            _ball_edges->AddPoint( ball2.x );
        }
    }
    _ball_edges->UpdateMem();
}

template<SphereType Sphere>
inline void SphereMesh<Sphere>::UpdateOritBuffer()
{
    _ball_orit_lines->Clear();
    for (Sphere& ball : _balls)
    {
        if (ball.deleted)
            continue;
        glm::vec3 px = glm::normalize( glm::rotate( ball.q, glm::vec3( 1, 0, 0 ) ) );
        glm::vec3 py = glm::normalize( glm::rotate( ball.q, glm::vec3( 0, 1, 0 ) ) );
        glm::vec3 pz = glm::normalize( glm::rotate( ball.q, glm::vec3( 0, 0, 1 ) ) );
        _ball_orit_lines->AddPoint( ball.x );
        _ball_orit_lines->AddPoint( ball.x + px * 0.3f );
        _ball_orit_lines->AddPoint( ball.x, glm::vec3( 0, 1, 0 ) );
        _ball_orit_lines->AddPoint( ball.x + py * 0.3f, glm::vec3( 0, 1, 0 ) );
        _ball_orit_lines->AddPoint( ball.x, glm::vec3( 0, 0, 1 ) );
        _ball_orit_lines->AddPoint( ball.x + pz * 0.3f, glm::vec3( 0, 0, 1 ) );
    }
    _ball_orit_lines->UpdateMem();
}

template<SphereType Sphere>
inline void SphereMesh<Sphere>::Clear()
{
    _balls.clear();
    _ball_edges->Clear();
}

template<SphereType Sphere>
inline void SphereMesh<Sphere>::AddBall( glm::vec3 x, float r )
{
    Sphere s;
    s.x = x;
    s.x0 = x;
    s.r = r;
    _balls.push_back( s );
}

template<SphereType Sphere>
inline void SphereMesh<Sphere>::DeleteCompleteInsideBalls()
{
    for (int i = 0, size = _balls.size(); i < size; i++)
    {
        Sphere& balli = _balls[i];
        if (balli.r < 0)
            continue;
        for (int j = 0; j < size; j++)
        {
            if (i == j)
                continue;
            Sphere& ballj = _balls[j];
            if (ballj.r < 0)
            {
                continue;
            }
            if (glm::distance( balli.x0, ballj.x0 ) < ballj.r - balli.r)
            {
                balli.r = -1;
                break;
            }
        }
    }

    _balls.erase( std::remove_if( _balls.begin(), _balls.end(), []( Sphere& s ) { return s.r < 0; } ), _balls.end() );
}

template<SphereType Sphere>
inline void SphereMesh<Sphere>::DeleteMarkedBalls()
{
    _balls.erase( std::remove_if( _balls.begin(), _balls.end(), []( Sphere& s ) { return s.deleted; } ), _balls.end() );
}

template<SphereType Sphere>
template<SphereType Sphere2>
inline float SphereMesh<Sphere>::Distance( const SphereMesh<Sphere2>& other )
{
    if (_balls.size() != other._balls.size())
    {
        return std::numeric_limits<float>::infinity();
    }
    float dist = 0.f;
    for (int i = 0; i < _balls.size(); i++)
    {
        dist += glm::distance( _balls[i].x, other._balls[i].x );
    }
    dist /= _balls.size();
    return dist;
}

template <SphereType Sphere>
inline Sphere& SphereMesh<Sphere>::operator[]( unsigned int id )
{
    return _balls[id];
}

template <SphereType Sphere>
inline const Sphere& SphereMesh<Sphere>::operator[]( unsigned int id ) const
{
    return _balls[id];
}
