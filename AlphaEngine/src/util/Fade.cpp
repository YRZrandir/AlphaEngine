#include "Fade.h"

void Triangulation2d( const std::vector<glm::vec2>& points, std::vector<Triangle>* out_triangles, std::vector<glm::vec2>* out_vertices )
{
    using namespace GEOM_FADE2D;
    std::vector<Point2> vInputPoints;
    for (const auto& p : points)
    {
        vInputPoints.push_back( Point2( p.x, p.y ) );
    }

    Fade_2D dt;
    dt.insert( vInputPoints );

    std::vector<Segment2> borders;
    for (int i = 0, s = points.size(); i < s; i++)
    {
        Segment2 seg( Point2( points[i].x, points[i].y ), Point2( points[(i + 1) % s].x, points[(i + 1) % s].y ) );
        borders.push_back( seg );
    }
    ConstraintGraph2* cg = dt.createConstraint( borders, CIS_CONSTRAINED_DELAUNAY );
    Zone2* zone_inside( dt.createZone( cg, ZL_INSIDE ) );

    FadeExport fadeExport;
    bool bCustomIndices( true );
    bool bClear( true );
    zone_inside->exportZone( fadeExport, true );
    //dt.exportTriangulation( fadeExport, true, true );
    for (int i = 0; i < fadeExport.numPoints; i++)
    {
        double x, y, z;
        fadeExport.getCoordinates( i, x, y );
        out_vertices->push_back( glm::vec2( x, y ) );
    }

    for (int i = 0; i < fadeExport.numTriangles; i++)
    {
        int ia, ib, ic;
        fadeExport.getCornerIndices( i, ia, ib, ic );
        out_triangles->push_back( Triangle( ia, ib, ic ) );
    }
}
