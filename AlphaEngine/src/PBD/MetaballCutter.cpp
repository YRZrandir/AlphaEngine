#include "MetaballCutter.h"
#include "MetaballTester.h"
#include "SphereTreeHelper.h"
#include "../input/Input.h"
#include "../model/Rect.h"
#include "../util/Camera.h"
#include "../util/GridDownSampler.h"

PBD::MetaballCutter::MetaballCutter( PBD::MetaballModel* model, PBD::ScalpelRect* scalpel )
    :_model( model ), _scalpel( scalpel )
{
}

void PBD::MetaballCutter::ProcessOneStepCutRect( Rect& rect )
{
    std::vector<int> touched_id = MetaballTester::BallsTouchedByRect( _model->BallList().begin(), _model->BallList().end(), rect );
    _removed_balls.insert( std::end( _removed_balls ), std::begin( touched_id ), std::end( touched_id ) );
    std::vector<PBD::MeshlessPoint> meshless_points;
    for (int id : touched_id)
    {
        std::vector<PBD::MeshlessPoint> temp = SamplePointsInBall( _model->Ball( id ).x, _model->Ball( id ).r );
        meshless_points.insert( std::end( meshless_points ), std::begin( temp ), std::end( temp ) );
        _model->Ball( id ).valid = false;
    }
    //_meshless_points.insert( std::end( _meshless_points ), std::begin( meshless_points ), std::end( meshless_points ) );
}

void PBD::MetaballCutter::ProcessTotalCutRect( const Rect& rect )
{
    std::cout << "Start Cut Metaball" << std::endl;
    std::vector<glm::vec3> ball_points = SamplePointsOnRemovedBallSurface();
    std::vector<glm::vec3> rect_points = SamplePointsOnRect( rect, 0.01f );
    auto point_not_in_any_ball = [&]( glm::vec3& p ) {
        return !std::any_of( _model->BallList().begin(), _model->BallList().end(),
            [&]( Metaball& ball ) {
                return glm::distance( p, ball.x ) < ball.r;
            } );
    };
    rect_points.erase( std::remove_if( rect_points.begin(), rect_points.end(), point_not_in_any_ball ), rect_points.end() );

    std::vector<glm::vec3> points_upper;
    std::vector<glm::vec3> points_lower;

    glm::vec3 n = rect.Normal();
    for (auto& p : ball_points)
    {
        if (glm::dot( p - rect.Center(), n ) > 0)
        {
            points_upper.push_back( p );
        }
        else
        {
            points_lower.push_back( p );
        }
    }

    for (auto& p : rect_points)
    {
        points_upper.push_back( p );
        points_lower.push_back( p );
    }

    std::vector<glm::vec3> new_points_upper = PointsToMetaball( points_upper, _model->BallList(), _removed_balls, rect, true );
    std::vector<glm::vec3> new_points_lower = PointsToMetaball( points_lower, _model->BallList(), _removed_balls, rect, false );

    float d_upper = 0.f;
    for (const auto& p : new_points_upper)
    {
        d_upper = glm::max( d_upper, 0.5f * glm::abs( glm::dot( rect.Normal(), p - rect.Center() ) ) );
    }
    float d_lower = 0.f;
    for (const auto& p : new_points_lower)
    {
        d_lower = glm::max( d_lower, 0.5f * glm::abs( glm::dot( rect.Normal(), p - rect.Center() ) ) );
    }

    GridDownSampler gds1( &new_points_upper, d_upper * 0.5f );
    new_points_upper = gds1.GetSamples();
    GridDownSampler gds2( &new_points_lower, d_lower * 0.5f );
    new_points_lower = gds2.GetSamples();

    std::vector<Metaball> new_balls_upper;
    for (auto& p : new_points_upper)
    {
        float min_dist = FLT_MAX;
        for (auto& up : points_upper)
        {
            min_dist = glm::min( min_dist, glm::distance( p, up ) );
        }
        for (auto& lp : points_lower)
        {
            min_dist = glm::min( min_dist, glm::distance( p, lp ) );
        }
        new_balls_upper.push_back( Metaball( p, min_dist ) );
    }

    std::vector<Metaball> new_balls_lower;
    for (auto& p : new_points_lower)
    {
        float min_dist = FLT_MAX;
        for (auto& up : points_upper)
        {
            min_dist = glm::min( min_dist, glm::distance( p, up ) );
        }
        for (auto& lp : points_lower)
        {
            min_dist = glm::min( min_dist, glm::distance( p, lp ) );
        }
        new_balls_lower.push_back( Metaball( p, min_dist ) );
    }

    HalfEdgeSurfaceTester tester;
    tester.SetSurface( &_model->Surface() );
    for (auto& ball : new_balls_upper)
    {
        for (int i : _removed_balls)
        {
            if (glm::distance( ball.x, _model->Ball( i ).x ) < _model->Ball( i ).r)
            {
                ball.x0 = _model->Ball( i ).x0 + glm::rotate( glm::inverse( _model->Ball( i ).q ), ball.x - _model->Ball( i ).x );
                break;
            }
        }
        //ball.r = glm::min( ball.r, tester.MinDistToSurface( ball.c ) );
    }
    for (auto& ball : new_balls_lower)
    {
        for (int i : _removed_balls)
        {
            if (glm::distance( ball.x, _model->Ball( i ).x ) < _model->Ball( i ).r)
            {
                ball.x0 = _model->Ball( i ).x0 + glm::rotate( glm::inverse( _model->Ball( i ).q ), ball.x - _model->Ball( i ).x );
                break;
            }
        }
        //ball.r = glm::min( ball.r, tester.MinDistToSurface( ball.c ) );
    }

    _model->BallList().insert( _model->BallList().end(), new_balls_upper.begin(), new_balls_upper.end() );
    _model->BallList().insert( _model->BallList().end(), new_balls_lower.begin(), new_balls_lower.end() );
    _model->CreateTopo( rect );

    _model->BallList().erase( std::remove_if( _model->BallList().begin(), _model->BallList().end(),
        []( PBD::Metaball& ball )->bool
        {
            return ball.neighbors.empty();
        } ), _model->BallList().end() );
    _model->CreateTopo( rect );

    _model->InitVertices();
    _model->InitConstraints();

    _removed_balls.clear();
    std::cout << "Finish Cut Metaball" << std::endl;
}

std::vector<PBD::MeshlessPoint> PBD::MetaballCutter::SamplePointsInBall( glm::vec3 c, float r, float step )
{
    AABB aabb;
    aabb.Expand( c + glm::vec3( r ) );
    aabb.Expand( c - glm::vec3( r ) );

    glm::vec3 start_pos = aabb.min_corner;
    glm::vec3 end_pos = aabb.max_corner;
    std::vector<PBD::MeshlessPoint> points;
    for (float x = start_pos.x; x <= end_pos.x; x += step)
    {
        for (float y = start_pos.y; y <= end_pos.y; y += step)
        {
            for (float z = start_pos.z; z <= end_pos.z; z += step)
            {
                glm::vec3 p( x, y, z );
                float d = glm::distance( c, p );

                if (d < r)
                {
                    points.push_back( PBD::MeshlessPoint( p, 0, -1 ) );
                    break;
                }
            }
        }
    }

    return points;
}

std::vector<glm::vec3> PBD::MetaballCutter::SamplePointsOnRemovedBallSurface()
{
    std::vector<glm::vec3> bound_points;
    for (int i : _removed_balls)
    {
        auto& ball = _model->Ball( i );
        int step_num = glm::round( 64.0f * ball.r );
        step_num = glm::clamp( step_num, 1, 64 );
        float step = 2.f * glm::pi<float>() / step_num;
        for (int j = 0; j < step_num; j++)
        {
            for (int k = 0; k < step_num; k++)
            {
                float x = glm::sin( j * step / 2.0f ) * glm::cos( k * step );
                float y = glm::sin( j * step / 2.0f ) * glm::sin( k * step );
                float z = glm::cos( j * step / 2.0f );
                glm::vec3 point = ball.x + glm::vec3( x, y, z ) * ball.r;
                bool in_any_ball = std::any_of( _removed_balls.begin(), _removed_balls.end(),
                    [&]( int otherball )
                    {
                        return otherball != i && glm::distance( point, _model->Ball( otherball ).x ) < _model->Ball( otherball ).r;
                    } );
                if (!in_any_ball)
                {
                    bound_points.push_back( point );
                }
            }
        }
    }
    return bound_points;
}

std::vector<glm::vec3> PBD::MetaballCutter::SamplePointsOnRect( const Rect& rect, float step )
{
    std::vector<glm::vec3> bound_points;
    glm::vec3 n = rect.Normal();
    glm::vec3 LU = rect.LeftUp();
    glm::vec3 LD = rect.LeftDown();
    glm::vec3 RD = rect.RightDown();
    glm::vec3 RU = rect.RightUp();
    glm::vec3 u = glm::normalize( RD - LD );
    glm::vec3 v = glm::normalize( LU - LD );

    for (float x = 0.f; x < rect.Width(); x += step)
    {
        for (float y = 0.f; y < rect.Height(); y += step)
        {
            bound_points.push_back( LD + x * u + y * v );
        }
    }
    return bound_points;
}
