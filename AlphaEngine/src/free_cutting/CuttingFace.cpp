#include "CuttingFace.h"
#include "../util/util.h"

CuttingFace::CuttingFace( glm::vec3 a, glm::vec3 b, glm::vec3 c )
    :pa( a ), pb( b ), pc( c )
{
    normal = glm::normalize( glm::cross( b - a, c - a ) );
}

bool CuttingFace::TriangleIntersect( glm::vec3 a, glm::vec3 b, glm::vec3 c,
    std::pair<CuttingFace::IntersectInfo, CuttingFace::IntersectInfo>* result ) const
{
    CuttingFace meshFace( a, b, c );
    glm::vec3 ab_mesh_pos;
    bool ab_intersect_mesh = meshFace.LineIntersect( pa, pb, &ab_mesh_pos );
    glm::vec3 bc_mesh_pos;
    bool bc_intersect_mesh = meshFace.LineIntersect( pb, pc, &bc_mesh_pos );
    glm::vec3 ca_mesh_pos;
    bool ca_intersect_mesh = meshFace.LineIntersect( pc, pa, &ca_mesh_pos );
    int cut_mesh_cnt = 0;
    if (ab_intersect_mesh) cut_mesh_cnt++;
    if (bc_intersect_mesh) cut_mesh_cnt++;
    if (ca_intersect_mesh) cut_mesh_cnt++;

    glm::vec3 meshab_cutface_pos;
    bool meshab_intersect_cutface = LineIntersect( a, b, &meshab_cutface_pos );
    glm::vec3 meshbc_cutface_pos;
    bool meshbc_intersect_cutface = LineIntersect( b, c, &meshbc_cutface_pos );
    glm::vec3 meshca_cutface_pos;
    bool meshca_intersect_cutface = LineIntersect( c, a, &meshca_cutface_pos );
    int cut_cutface_cnt = 0;
    if (meshab_intersect_cutface) cut_cutface_cnt++;
    if (meshbc_intersect_cutface) cut_cutface_cnt++;
    if (meshca_intersect_cutface) cut_cutface_cnt++;

    bool intersect = false;
    glm::vec3 p0;
    glm::vec3 p1;
    IntersectInfo info0, info1;

    if (cut_mesh_cnt == 2 && cut_cutface_cnt == 0)
    {
        intersect = true;
        if (ab_intersect_mesh && bc_intersect_mesh)
        {
            p0 = ab_mesh_pos;
            p1 = bc_mesh_pos;
        }
        else if (bc_intersect_mesh && ca_intersect_mesh)
        {
            p0 = bc_mesh_pos;
            p1 = ca_mesh_pos;
        }
        else if (ca_intersect_mesh && ab_intersect_mesh)
        {
            p0 = ca_mesh_pos;
            p1 = ab_mesh_pos;
        }
    }
    else if (cut_mesh_cnt == 1 && cut_cutface_cnt == 1)
    {
        intersect = true;
        if (ab_intersect_mesh)
        {
            p0 = ab_mesh_pos;
        }
        else if (bc_intersect_mesh)
        {
            p0 = bc_mesh_pos;
        }
        else if (ca_intersect_mesh)
        {
            p0 = ca_mesh_pos;
        }

        info1.onedge = true;
        if (meshab_intersect_cutface)
        {
            p1 = meshab_cutface_pos;
            info1.edge = 0;
        }
        else if (meshbc_intersect_cutface)
        {
            p1 = meshbc_cutface_pos;
            info1.edge = 1;
        }
        else if (meshca_intersect_cutface)
        {
            p1 = meshca_cutface_pos;
            info1.edge = 2;
        }
    }
    else if (cut_mesh_cnt == 0 && cut_cutface_cnt == 2)
    {
        intersect = true;
        if (meshab_intersect_cutface && meshbc_intersect_cutface)
        {
            p0 = meshab_cutface_pos;
            p1 = meshbc_cutface_pos;
        }
        else if (meshbc_intersect_cutface && meshca_intersect_cutface)
        {
            p0 = meshbc_cutface_pos;
            p1 = meshca_cutface_pos;
        }
        else if (meshca_intersect_cutface && meshab_intersect_cutface)
        {
            p0 = meshca_cutface_pos;
            p1 = meshab_cutface_pos;
        }
    }
    else
    {
        intersect = false;
    }

    if (intersect)
    {
        info0.p = p0;
        info1.p = p1;
        glm::vec3 bcpos0 = BarycentricPos( a, b, c, p0 );
        glm::vec3 bcpos1 = BarycentricPos( a, b, c, p1 );

        for (size_t i = 0; i < 3; i++)
        {
            if (glm::FloatEqual( bcpos0[i], 0.0f ))
            {
                info0.onedge = true;
                info0.edge = (i + 1) % 3;
            }
        }

        for (size_t i = 0; i < 3; i++)
        {
            if (glm::FloatEqual( bcpos1[i], 0.0f ))
            {
                info1.onedge = true;
                info1.edge = (i + 1) % 3;
            }
        }
    }
    *result = { info0, info1 };
    return intersect;
}

bool CuttingFace::LineIntersect( glm::vec3 a, glm::vec3 b, glm::vec3* pos ) const
{
    glm::vec3 d = glm::normalize( b - a );
    float t_max = glm::distance( a, b );
    glm::vec3 e1, e2, s, s1, s2;
    float s2e2, s1s, s2d;
    glm::vec3 tbb;

    e1 = pb - pa;
    e2 = pc - pa;
    s = a - pa;
    s1 = glm::cross( d, e2 );
    s2 = glm::cross( s, e1 );

    s2e2 = glm::dot( s2, e2 );
    s1s = glm::dot( s1, s );
    s2d = glm::dot( s2, d );

    tbb = glm::vec3{ s2e2, s1s, s2d } / glm::dot( s1, e1 );
    if (tbb.x > 0.f && tbb.x < t_max &&
        tbb.y > 0.f &&
        tbb.z > 0.f &&
        (1 - tbb.y - tbb.z) > 0.f)
    {
        float t = tbb.x;
        *pos = a + t * d;
        return true;
    }
    return false;
}

glm::vec3 CuttingFace::BarycentricPos( glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 pos ) const
{
    float area_a = glm::length( glm::cross( b - pos, c - pos ) );
    float area_b = glm::length( glm::cross( a - pos, c - pos ) );
    float area_c = glm::length( glm::cross( a - pos, b - pos ) );
    float area = area_a + area_b + area_c;
    float arg0 = area_a / area;
    float arg1 = area_b / area;
    float arg2 = area_c / area;
    return glm::vec3( arg0, arg1, arg2 );
}
