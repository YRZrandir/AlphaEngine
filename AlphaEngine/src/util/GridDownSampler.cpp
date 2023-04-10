#include "GridDownSampler.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

GridDownSampler::GridDownSampler( std::vector<glm::vec3>* points, float d )
    :_points( points ), _d( d )
{
    for (const auto& p : *_points)
    {
        _aabb.Expand( p );
    }

}

std::vector<glm::vec3> GridDownSampler::GetSamples()
{
    glm::vec3 size = _aabb.max_corner - _aabb.min_corner;
    long cnt_x = std::lroundf( size.x / _d );
    float dx = size.x / cnt_x;
    long cnt_y = std::lroundf( size.y / _d );
    float dy = size.y / cnt_y;
    long cnt_z = std::lroundf( size.z / _d );
    float dz = size.z / cnt_z;

    pcl::PointCloud<pcl::PointXYZ>::Ptr points( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_points( new pcl::PointCloud<pcl::PointXYZ>() );

    for (const auto& p : *_points)
    {
        points->push_back( pcl::PointXYZ( p.x, p.y, p.z ) );
    }

    pcl::VoxelGrid<pcl::PointXYZ> vgrid;
    vgrid.setInputCloud( points );
    vgrid.setLeafSize( dx, dy, dz );
    vgrid.filter( *new_points );

    std::vector<glm::vec3> results;
    for (auto& p : *new_points)
    {
        results.push_back( glm::vec3( p.x, p.y, p.z ) );
    }
    return results;
}
