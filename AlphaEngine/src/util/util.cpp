#include "util.h"
#include <algorithm>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
#include "../polar_decom/polar_decomposition_3x3.hpp"

std::string glutil::Type2String( GLuint type )
{
    /*
    GL_BYTE 0x1400
    GL_UNSIGNED_BYTE 0x1401
    GL_SHORT 0x1402
    GL_UNSIGNED_SHORT 0x1403
    GL_INT 0x1404
    GL_UNSIGNED_INT 0x1405
    GL_FLOAT 0x1406
    */
    switch (type)
    {
    case GL_BYTE:
        return "byte";
    case GL_UNSIGNED_BYTE:
        return "ubyte";
    case GL_SHORT:
        return "short";
    case GL_UNSIGNED_SHORT:
        return "ushort";
    case GL_INT:
        return "int";
    case GL_UNSIGNED_INT:
        return "uint";
    case GL_FLOAT:
        return "float";
    default:
        __debugbreak();
    }
    return "";
}

int glutil::Type2Size( GLuint type )
{
    switch (type)
    {
    case GL_BYTE:
        return sizeof( GLbyte );
    case GL_UNSIGNED_BYTE:
        return sizeof( GLubyte );
    case GL_SHORT:
        return sizeof( GLshort );
    case GL_UNSIGNED_SHORT:
        return sizeof( GLushort );
    case GL_INT:
        return sizeof( GLint );
    case GL_UNSIGNED_INT:
        return sizeof( GLuint );
    case GL_FLOAT:
        return sizeof( GLfloat );
    default:
        __debugbreak();
    }
    return 0;
}


const glm::vec3 glm::WHITE( 1.0f, 1.0f, 1.0f );
const glm::vec3 glm::LIGHTGREY( 0.8f, 0.8f, 0.8f );
const glm::vec3 glm::DARKGREY( 0.3f, 0.3f, 0.3f );
const glm::vec3 glm::BLACK( 0.0f, 0.0f, 0.0f );
const glm::vec3 glm::RED( 1.0f, 0.0f, 0.0f );
const glm::vec3 glm::GREEN( 0.0f, 1.0f, 0.0f );
const glm::vec3 glm::BLUE( 0.0f, 0.0f, 1.0f );
const glm::vec3 glm::YELLOW( 1.0f, 1.0f, 0.0f );
const glm::vec3 glm::GREENYELLOW( 0.68f, 1.0f, 0.18f );
const glm::vec3 glm::CADETBLUE( 0.37f, 0.62f, 0.63f );
const glm::vec3 glm::DARKGREEN( 0.56f, 0.74f, 0.56f );
const glm::vec3 glm::PINK( 1.0f, 0.71f, 0.76f );

double glm::TriangleSize( vec3 p0, vec3 p1, vec3 p2 )
{
    return glm::length( glm::cross( p1 - p0, p2 - p0 ) ) * 0.5;
}

double glm::TetrahedronVolume( vec3 p0, vec3 p1, vec3 p2, vec3 p3 )
{
    return glm::determinant( glm::mat3( p1 - p0, p2 - p0, p3 - p0 ) ) / 6.0;
}

bool glm::FloatEqual( float value1, float value2, float threshold )
{
    float diff = value1 - value2;
    return diff > -threshold && diff < threshold;
}

bool glm::TwoPointsEqual( const vec3& a, const vec3& b, float threshold )
{
    return FloatEqual( a.x, b.x, threshold ) && FloatEqual( a.y, b.y, threshold ) && FloatEqual( a.z, b.z, threshold );
}

glm::vec3 glm::BarycentricPos( glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 pos )
{
    glm::vec3 n = glm::normalize( glm::cross( b - a, c - a ) );
    glm::vec3 Spbc = glm::cross( b - pos, c - pos );
    glm::vec3 Spac = glm::cross( c - pos, a - pos );
    glm::vec3 Spab = glm::cross( a - pos, b - pos );
    float area_a = glm::length( Spbc ) * glm::sign( glm::dot( Spbc, n ) );
    float area_b = glm::length( Spac ) * glm::sign( glm::dot( Spac, n ) );
    float area_c = glm::length( Spab ) * glm::sign( glm::dot( Spab, n ) );
    float area = area_a + area_b + area_c;
    float arg0 = area_a / area;
    float arg1 = area_b / area;
    float arg2 = 1.f - arg0 - arg1;
    return glm::vec3( arg0, arg1, arg2 );
}

glm::vec4 glm::TetraBarycentricPos( glm::vec3 p0, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p )
{
    float a0 = SignedDistToPlane( p1, p2, p3, p ) / SignedDistToPlane( p1, p2, p3, p0 );
    float a1 = SignedDistToPlane( p0, p2, p3, p ) / SignedDistToPlane( p0, p2, p3, p1 );
    float a2 = SignedDistToPlane( p0, p1, p3, p ) / SignedDistToPlane( p0, p1, p3, p2 );
    float a3 = 1.f - a0 - a1 - a2;
    return glm::vec4( a0, a1, a2, a3 );
}

float glm::SignedDistToPlane( glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 p )
{
    glm::vec3 n = glm::normalize( glm::cross( b - a, c - a ) );
    float d = glm::dot( c - p, n );
    return d;
}

float glm::DistToLine( glm::vec3 p, glm::vec3 ori, glm::vec3 dir )
{
    if (glm::length2( dir ) <= FLT_EPSILON)
    {
        std::cout << "ERR: " << __FILE__ << ": " << __func__ << ": Argument 'glm::vec3 dir' must not be zero length" << std::endl;
#ifdef _DEBUG
        __debugbreak();
#else
        abort();
#endif
    }
    dir = glm::normalize( dir );
    return glm::abs( glm::dot( p - ori, dir ) );
}

float glm::MinDistToLineSeg( glm::vec3 p, glm::vec3 start, glm::vec3 end, int* topo, glm::vec3* projpos )
{
    const glm::vec3 dir = glm::normalize( end - start );
    const float dot = glm::dot( p - start, dir );
    if (dot >= 0.f && dot <= glm::length( end - start ))
    {
        if (topo)
            *topo = 1;
        if (projpos)
            *projpos = start + dir * dot;
        return glm::sqrt( glm::distance2( start, p ) - dot * dot );
    }

    const float dstart = glm::distance( p, start );
    const float dend = glm::distance( p, end );
    if (dstart < dend)
    {
        if (topo)
            *topo = 0;
        if (projpos)
            *projpos = start;
        return dstart;
    }
    else
    {
        if (topo)
            *topo = 2;
        if (projpos)
            *projpos = end;
        return dend;
    }
}

float glm::MinDistToTriangle( glm::vec3 p, glm::vec3 a, glm::vec3 b, glm::vec3 c, int* topo, glm::vec3* projpos )
{
    const glm::vec3 n = glm::normalize( glm::cross( b - a, c - a ) );
    const glm::vec3 proj = p - glm::dot( p - a, n ) * n;
    const glm::vec3 bcpos = BarycentricPos( a, b, c, proj );
    if (bcpos[0] >= 0.f && bcpos[0] <= 1.f &&
        bcpos[1] >= 0.f && bcpos[1] <= 1.f &&
        bcpos[2] >= 0.f && bcpos[2] <= 1.f)
    {
        if (topo)
            *topo = 6;
        if (projpos)
            *projpos = proj;
        return glm::distance( proj, p );
    }
    else
    {
        int topoab = -1;
        int topobc = -1;
        int topoca = -1;
        glm::vec3 pab, pbc, pca;
        const float dab = MinDistToLineSeg( p, a, b, &topoab, &pab );
        const float dbc = MinDistToLineSeg( p, b, c, &topobc, &pbc );
        const float dca = MinDistToLineSeg( p, c, a, &topoca, &pca );

        if (dab < dca && dab < dbc)
        {
            if (topo)
                *topo = topoab + 0;
            if (projpos)
                *projpos = pab;
            return dab;
        }
        else if (dbc < dab && dbc < dca)
        {
            if (topo)
                *topo = topobc + 2;
            if (projpos)
                *projpos = pbc;
            return dbc;
        }
        else
        {
            if (topo)
                *topo = (topoca + 4) % 6;
            if (projpos)
                *projpos = pca;
            return dca;
        }
    }
}

float glm::MinDistToRect( glm::vec3 p, const Rect& rect )
{
    glm::vec3 n = rect.Normal();
    glm::vec3 LU = rect.LeftUp();
    glm::vec3 LD = rect.LeftDown();
    glm::vec3 RD = rect.RightDown();
    glm::vec3 RU = rect.RightUp();
    glm::vec3 u = glm::normalize( RD - LD );
    glm::vec3 v = glm::normalize( LU - LD );
    glm::vec3 proj = p - glm::dot( p - rect.Center(), n ) * n;
    glm::vec2 uv( glm::dot( u, proj - LD ), glm::dot( v, proj - LD ) );
    if (uv.x >= 0 && uv.x <= rect.Width() && uv.y >= 0 && uv.y <= rect.Height())
    {
        return glm::distance( proj, p );
    }
    else
    {
        float dists[4] = {
            MinDistToLineSeg( p, LU, LD ), MinDistToLineSeg( p, LU, RU ),
            MinDistToLineSeg( p, RU, RD ), MinDistToLineSeg( p, RD, LD ) };
        return glm::min( glm::min( dists[0], dists[1] ), glm::min( dists[2], dists[3] ) );
    }
}

bool glm::IsRotationMatrix( glm::mat3& mat )
{
    float det = glm::determinant( mat );
    if (!glm::FloatEqual( det, 1.0f, 1e-5f ))
    {
        return false;
    }

    if (!glm::FloatEqual( glm::length( mat[0] ), 1.0f, 1e-5 ) ||
        !glm::FloatEqual( glm::length( mat[1] ), 1.0f, 1e-5 ) ||
        !glm::FloatEqual( glm::length( mat[2] ), 1.0f, 1e-5 ))
    {
        return false;
    }

    if (!glm::FloatEqual( glm::dot( mat[0], mat[1] ), 0.f, 1e-5 ) ||
        !glm::FloatEqual( glm::dot( mat[2], mat[1] ), 0.f, 1e-5 ) ||
        !glm::FloatEqual( glm::dot( mat[0], mat[2] ), 0.f, 1e-5 ))
    {
        return false;
    }

    return true;
}

glm::mat3 glm::TensorProduct( const glm::vec3& v, const glm::vec3& vT )
{
    glm::mat3 mat( 0.f );
    mat[0][0] += v[0] * vT[0]; mat[1][0] += v[0] * vT[1]; mat[2][0] += v[0] * vT[2];
    mat[0][1] += v[1] * vT[0]; mat[1][1] += v[1] * vT[1]; mat[2][1] += v[1] * vT[2];
    mat[0][2] += v[2] * vT[0]; mat[1][2] += v[2] * vT[1]; mat[2][2] += v[2] * vT[2];
    return mat;
}

glm::mat3 glm::SafeInverse( glm::mat3 M )
{
    Matrix3 eigenM;
    for (int i = 0; i < 3; i++)
    {
        eigenM( i, 0 ) = M[0][i];
        eigenM( i, 1 ) = M[1][i];
        eigenM( i, 2 ) = M[2][i];
    }

    auto eigenResult = EigenSafeInverse( eigenM );

    glm::mat3 result;
    for (int i = 0; i < 3; i++)
    {
        result[i][0] = eigenResult( 0, i );
        result[i][1] = eigenResult( 1, i );
        result[i][2] = eigenResult( 2, i );
    }
    return result;

}

float glm::FrobeniusNormSquare( glm::mat3 M )
{
    float result = 0.f;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result += M[i][j] * M[i][j];
        }
    }
    return result;
}

float glm::FrobeniusNorm( glm::mat3 M )
{
    return glm::sqrt( glm::FrobeniusNormSquare( M ) );
}

std::string RelPathToAbsPath( const std::string& rel_path )
{
    std::string abs_path;
    abs_path.resize( rel_path.size() );
#ifdef _WIN32
    _fullpath( abs_path.data(), rel_path.data(), rel_path.size() );
#else
    realpath( rel_path.data(), abs_path.data() );
#endif
    std::replace( abs_path.begin(), abs_path.end(), '\\', '/' );
    return abs_path;
}

