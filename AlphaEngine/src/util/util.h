#pragma once
#include <string>
#include <iostream>
#include <glad/glad.h>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <Eigen/Core>
#include "../model/Triangle.h"
#include "../model/Rect.h"
#include "Math.h"
#include "../polar_decom/polar_decomposition_3x3.hpp"

namespace glutil
{
std::string Type2String( GLuint type );
int Type2Size( GLuint type );
}


namespace glm
{
extern const glm::vec3 WHITE;
extern const glm::vec3 LIGHTGREY;
extern const glm::vec3 DARKGREY;
extern const glm::vec3 BLACK;
extern const glm::vec3 RED;
extern const glm::vec3 GREEN;
extern const glm::vec3 BLUE;
extern const glm::vec3 YELLOW;
extern const glm::vec3 GREENYELLOW;
extern const glm::vec3 CADETBLUE;
extern const glm::vec3 DARKGREEN;
extern const glm::vec3 PINK;

double TriangleSize( glm::vec3 p0, glm::vec3 p1, glm::vec3 p2 );

double TetrahedronVolume( glm::vec3 p0, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3 );

bool FloatEqual( float value1, float value2, float threshold = FLT_EPSILON * 2 );

bool TwoPointsEqual( const glm::vec3& a, const glm::vec3& b, float threshold = FLT_EPSILON );

glm::vec3 BarycentricPos( glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 pos );

glm::vec4 TetraBarycentricPos( glm::vec3 p0, glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p );

float	SignedDistToPlane( glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 p );

float	DistToLine( glm::vec3 p, glm::vec3 ori, glm::vec3 dir );

/* topo: 0 = start, 1 = middle, 2 = end*/
float	MinDistToLineSeg( glm::vec3 p, glm::vec3 start, glm::vec3 end, int* topo = nullptr, glm::vec3* projpos = nullptr );

/* topo: 0=a, 1=ab, 2=b, 3=bc, 4=c, 5=ca, 6=face*/
float   MinDistToTriangle( glm::vec3 p, glm::vec3 a, glm::vec3 b, glm::vec3 c, int* topo = nullptr, glm::vec3* projpos = nullptr );

float	MinDistToRect( glm::vec3 p, const Rect& rect );

bool	IsRotationMatrix( glm::mat3& mat );

glm::mat3 TensorProduct( const glm::vec3& v, const glm::vec3& vT );

glm::mat3 SafeInverse( glm::mat3 M );

float FrobeniusNormSquare( glm::mat3 M );

float FrobeniusNorm( glm::mat3 M );

template <typename T>
T TriangleInterpolation( vec3 p, vec3 a, vec3 b, vec3 c, const T& valueA, const T& valueB, const T& valueC )
{
    float area_a = glm::length( glm::cross( b - p, c - p ) );
    float area_b = glm::length( glm::cross( a - p, c - p ) );
    float area_c = glm::length( glm::cross( a - p, b - p ) );
    float area = area_a + area_b + area_c;
    return valueA * area_a / area + valueB * area_b / area + valueC * area_c / area;
}

//Assuming p is on the line ab
template <typename T>
T LinearInterpolation( glm::vec3 p, glm::vec3 a, glm::vec3 b, const T& value_a, const T& value_b )
{
    float t = glm::length( p - a ) / glm::length( b - a );
    return t * value_b + (1.f - t) * value_a;
}

template <typename T, int L, glm::qualifier Q>
bool AllFloatNumValid( const glm::vec<L, T, Q>& vec )
{
    for (int i = 0; i < L; i++)
    {
        if (std::isnan( vec[i] ) || std::isinf( vec[i] ))
            return false;
    }
    return true;
}

class Vec3Hash
{
public:
    size_t operator()( glm::vec3 vec ) const
    {
        return std::hash<float>()(vec.x) ^ std::hash<float>()(vec.y) ^ std::hash<float>()(vec.z);
    }
};
}

template <int size, typename T, glm::qualifier Q>
std::ostream& operator<<( std::ostream& os, const glm::vec<size, T, Q> vec )
{
    for (size_t i = 0; i < size; i++)
    {
        os << vec[i] << " ";
    }
    return os;
}

std::string RelPathToAbsPath( const std::string& relpath );

template <typename T>
Eigen::Matrix3<T> EigenSafeInverse( const Eigen::Matrix3<T>& m )
{
    Eigen::JacobiSVD<Eigen::Matrix3<T>, Eigen::ComputeFullU | Eigen::ComputeFullV> svd;
    svd.compute( m, Eigen::ComputeFullU | Eigen::ComputeFullV );
    Eigen::Vector3<T> Sv = svd.singularValues();
    Eigen::Matrix3<T> S;

    S.setZero();
    for (int i = 0; i < 3; i++)
    {
        if (std::abs( Sv( i ) ) > 1e-11)
        {
            S( i, i ) = 1.f / Sv( i );
        }
        else
        {
            S( i, i ) = 0.f;
        }
    }

    Eigen::Matrix3<T> V = svd.matrixV();
    Eigen::Matrix3<T> U = svd.matrixU();

    return V * S * U.transpose();
}

template <typename T>
void SVD( Eigen::Matrix3<T>* U, Eigen::Vector3<T>* S, Eigen::Matrix3<T>* V, const Eigen::Matrix3<T>& A )
{
    Eigen::JacobiSVD<Eigen::Matrix3<T>> svd;
    svd.compute( A, Eigen::ComputeFullU | Eigen::ComputeFullV );

    *U = svd.matrixU();
    *V = svd.matrixV();
    *S = svd.singularValues();

    float detU = U->determinant();
    float detV = V->determinant();

    if (detU < 0)
    {
        U->block<3, 1>( 0, 2 ) *= -1;
        (*S)[2] *= -1;
    }
    if (detV < 0)
    {
        V->block<3, 1>( 0, 2 ) *= -1;
        (*S)[2] *= -1;
    }
}

template<typename T>
void PolarDecomposition( const Eigen::Matrix3<T>& A, Eigen::Matrix3<T>* R, Eigen::Matrix3<T>* S )
{
    Polar_decomposition<true> pd;
    Mat3 m;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m( i, j ) = A( i, j );

    pd.compute( m );
    Mat3 R_ = pd.matrix_R();
    Mat3 S_ = pd.matrix_S();

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            (*R)(i, j) = R_( i, j );
            (*S)(i, j) = S_( i, j );
        }
    }
}

