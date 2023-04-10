#pragma once
#include <vector>
#include <iterator>
#include <glm/glm.hpp>
#include "MetaballModel.h"
#include "../model/Rect.h"
#include "../util/util.h"
#include "../util/Intersection.h"

//template <typename It>
//concept MetaballIt = std::input_iterator<It> && std::is_same_v<typename std::iterator_traits<It>::value_type, PBD::Metaball>;

class MetaballTester
{
public:
    template <typename It>
    static It PointInMetaball( It begin, It end, glm::vec3 p );

    template <typename It>
    static It LinesegIntersectMetaball( It begin, It end, glm::vec3 line0, glm::vec3 line1 );

    template <typename It>
    static std::vector<int> BallsTouchedByTriangle( It begin, It end, TriPoint triangle );

    template <typename It>
    static std::vector<int> BallsTouchedByLineSeg( It begin, It end, glm::vec3 p0, glm::vec3 p1 );

    template <typename It>
    static std::vector<int> BallsTouchedByRect( It begin, It end, const Rect& rect );

    template <typename It>
    static It NearestRayIntersection( It begin, It end, Ray ray, IntersectionRec* rec = nullptr, float t_min = 0.f, float t_max = FLT_MAX );

};

template <typename It>
It MetaballTester::PointInMetaball( It begin, It end, glm::vec3 p )
{
    for (; begin != end; ++begin)
    {
        PBD::Metaball& ball = *begin;
        if (!ball.valid)
        {
            continue;
        }
        if (glm::distance( ball.x, p ) < ball.r)
        {
            return begin;
        }
    }
    return end;
}

template <typename It>
It MetaballTester::LinesegIntersectMetaball( It begin, It end, glm::vec3 line0, glm::vec3 line1 )
{
    for (; begin != end; ++begin)
    {
        float dist = glm::MinDistToLineSeg( (*begin).c, line0, line1 );
        if (dist < (*begin).r)
        {
            return begin;
        }
    }
    return end;
}

template <typename It>
std::vector<int> MetaballTester::BallsTouchedByTriangle( It begin, It end, TriPoint triangle )
{
    std::vector<int> touched_ids;
    for (auto it = begin; it != end; ++it)
    {
        auto& ball = *it;
        if (!ball.valid)
        {
            continue;
        }
        float min_dist = glm::MinDistToTriangle( ball.c, triangle.a, triangle.b, triangle.c );
        if (min_dist < ball.r)
        {
            touched_ids.push_back( std::distance( begin, it ) );
        }
    }

    return touched_ids;
}

template <typename It>
It MetaballTester::NearestRayIntersection( It begin, It end, Ray ray, IntersectionRec* rec, float t_min, float t_max )
{
    IntersectionRec inner_rec;
    if (!rec)
    {
        rec = &inner_rec;
    }
    rec->t = t_max;
    auto intersect_ball_it = end;
    for (auto it = begin; it != end; ++it)
    {
        const auto& ball = *it;
        if (!ball.valid)
        {
            continue;
        }

        if (RayBallIntersect( ray, ball.x, ball.r, rec, t_min, rec->t ))
        {
            intersect_ball_it = it;
        }
    }

    return intersect_ball_it;
}

template <typename It>
std::vector<int> MetaballTester::BallsTouchedByLineSeg( It begin, It end, glm::vec3 p0, glm::vec3 p1 )
{
    std::vector<int> results;
    Ray ray( p0, glm::normalize( p1 - p0 ) );

    for (auto it = begin; it != end; ++it)
    {
        if (RayBallIntersect( ray, it->c, it->r, nullptr, 0.f, glm::distance( p1, p0 ) ))
        {
            results.push_back( std::distance( begin, it ) );
        }
    }

    return results;
}

template <typename It>
static std::vector<int> MetaballTester::BallsTouchedByRect( It begin, It end, const Rect& rect )
{
    std::vector<int> results;
    for (auto it = begin; it != end; ++it)
    {
        if (it->valid && BallRectIntersect( it->x, it->r, rect ))
        {
            results.push_back( std::distance( begin, it ) );
        }
    }
    return results;
}