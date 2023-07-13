#pragma once
#include <glm/glm.hpp>

namespace PBD
{
class StretchConstraint
{
public:
    struct Hash
    {
        size_t operator()( const StretchConstraint& sc ) const
        {
            return (size_t)sc._indices[0] + (size_t)sc._indices[1];
        }
    };
    struct Pred
    {
        bool operator()( const StretchConstraint& lhs, const StretchConstraint& rhs ) const
        {
            return lhs._indices[0] == rhs._indices[0] && lhs._indices[1] == rhs._indices[1]
                || lhs._indices[0] == rhs._indices[1] && lhs._indices[1] == rhs._indices[0];
        }
    };

    int _indices[2];
    float _k = 1.0f;
    float _d = 0.0f;

    StretchConstraint( int i1, int i2, float k, float d )
        :_d( d ), _k( k )
    {
        _indices[0] = i1;
        _indices[1] = i2;
    }
};

class EdgeConstraint
{
public:
    EdgeConstraint( int i0, int i1, float k, float d )
        :_i0( i0 ), _i1( i1 ), _k( k ), _d( d )
    {

    }

    int _i0;
    int _i1;
    float _k;
    float _d;
};

class VolumeConservConstraint
{
public:
    int _indices[4];
    float _k = 1.0f;
    float _V = 0.0f;

    VolumeConservConstraint( int i1, int i2, int i3, int i4, float k, float V )
        :_k( k ), _V( V )
    {
        _indices[0] = i1;
        _indices[1] = i2;
        _indices[2] = i3;
        _indices[3] = i4;
    }
};

class CollisionConstraint
{
public:
    int _index;
    glm::vec3 _pc;
    glm::vec3 _n;
    float _depth;
    float _m;   //the other particle's mass
    CollisionConstraint( int index, glm::vec3 pc, glm::vec3 n, float depth = 0.f, float m = FLT_MAX )
        :_index( index ), _pc( pc ), _n( n ), _depth( depth ), _m( m )
    {
    }
};

class ShapeMatchConstraint
{
public:
    float _k = 1.0f;
    glm::vec3 _Xcm;
    glm::mat3 _As;

    ShapeMatchConstraint() = default;

    ShapeMatchConstraint( glm::vec3 Xcm, glm::mat3 As, float k )
        :_Xcm( Xcm ), _As( As ), _k( k )
    {
    }
};

class AttachmentConstraint
{
public:
    int _index = -1;
    glm::vec3 _p;

    AttachmentConstraint( int index, glm::vec3 p )
        :_index( index ), _p( p )
    {
    }
};

class FEMConstraint
{
public:
    int _indices[4];
    glm::mat3 _inv_rest;
    glm::mat4x3 _grad;
    float _lambda = 0.f;
public:
    FEMConstraint( int i0, int i1, int i2, int i3, glm::mat3 inv_rest )
    {
        _indices[0] = i0;
        _indices[1] = i1;
        _indices[2] = i2;
        _indices[3] = i3;
        _inv_rest = inv_rest;
    }
};
}