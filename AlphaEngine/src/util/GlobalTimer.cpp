#include "GlobalTimer.h"
#include <chrono>
#include <numeric>

GlobalTimer::clock::duration GlobalTimer::_t_last_frame;
GlobalTimer::clock::time_point GlobalTimer::_t_total;
std::list<float> GlobalTimer::_recent_list;

void GlobalTimer::Start()
{
    _t_total = clock::now();
}

void GlobalTimer::Update()
{
    _t_last_frame = clock::now() - _t_total;
    _t_total = clock::now();

    _recent_list.push_back( LastFrameTimeMs() );
    if (_recent_list.size() > 128)
    {
        _recent_list.pop_front();
    }
}

float GlobalTimer::LastFrameTimeMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(_t_last_frame).count();
}

float GlobalTimer::FrameRate()
{
    return 1000.0f / LastFrameTimeMs();
}

float GlobalTimer::RecentAvgFrameTimeMs()
{
    if (_recent_list.size() == 0)
    {
        return 33.f;
    }
    return std::accumulate( _recent_list.begin(), _recent_list.end(), 0.f ) / _recent_list.size();
}
