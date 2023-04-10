#pragma once
#include <chrono>
#include <list> 

class GlobalTimer
{
public:
    static void Start();
    static void Update();
    static float LastFrameTimeMs();
    static float FrameRate();
    static float RecentAvgFrameTimeMs();

private:
    using clock = std::chrono::high_resolution_clock;
    static clock::time_point _t_total;
    static clock::duration _t_last_frame;
    static std::list<float> _recent_list;
};

