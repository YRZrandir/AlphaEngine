#pragma once
#include "../util/Math.h"
#include "glm/glm.hpp"
#include "glm/ext/scalar_constants.hpp"

namespace PD
{

class PDMetaballModel;
class PDGPUMetaballModel;

class Constraint
{
public:
    Constraint( const std::vector<int>& indices, float weight )
        :_indices( indices ), _weight( weight )
    {

    }
    virtual void Project( const Matrix3X& pos, Matrix3X& proj ) const = 0;
    virtual void AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const = 0;

    std::vector<int> _indices;
    mutable int _loc{ -1 };

    float _weight;
};

class EdgeConstraint : public Constraint
{
public:
    EdgeConstraint( int i0, int i1, float weight, const Matrix3X& pos );
    virtual void AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const override;
    virtual void Project( const Matrix3X& pos, Matrix3X& proj ) const override;

    float _rest = 0.f;
};

class MeshlessStrainConstraint : public Constraint
{
public:
    MeshlessStrainConstraint( const std::vector<int>& indices, float weight, const Matrix3X& positions, PDMetaballModel* model );
    virtual void AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const override;
    virtual void Project( const Matrix3X& pos, Matrix3X& proj ) const override;

    Matrix3 _invA;
    PDMetaballModel* _model;
    float _avg_dist;
    std::vector<float> _w;
private:
    float ComputeW( float r, float h ) const
    {
        if (r < h)
            return 315.f * glm::pow( h * h - r * r, 3 ) / (64.f * glm::pi<float>() * glm::pow( h, 9 ));
        else
            return 0.0001f;
    }

    Matrix3 ComputeF( const Matrix3X& pos ) const;
};

class GPUMeshlessStrainConstraint : public Constraint
{
public:
    GPUMeshlessStrainConstraint( const std::vector<int>& indices, float weight, const Matrix3X& positions, PDGPUMetaballModel* model );
    virtual void AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const override;
    virtual void Project( const Matrix3X& pos, Matrix3X& proj ) const override;

    Matrix3 _invA;
    PDGPUMetaballModel* _model;
    float _avg_dist;
    std::vector<float> _w;
private:
    float ComputeW( float r, float h ) const
    {
        if (r < h)
            return 315.f * glm::pow( h * h - r * r, 3.f ) / (64.f * glm::pi<float>() * glm::pow( h, 9 ));
        else
            return 0.0001f;
    }

    Matrix3 ComputeF( const Matrix3X& pos ) const;
};

class AttachConstraint : public Constraint
{
public:
    AttachConstraint( int id, float weight, Vector3 fixed_pos )
        : Constraint( std::vector<int>{id}, weight ), _fixed_pos( fixed_pos )
    {};

    virtual void AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const override
    {
        _loc = total_id;
        triplets.push_back( SparseMatrixTriplet( total_id, _indices[0], _weight * 1 ) );
        total_id += 1;
    }

    virtual void Project( const Matrix3X& pos, Matrix3X& proj ) const override
    {
        proj.col( _loc ) = _weight * _fixed_pos;
    }

    Vector3 _fixed_pos;
};

class VolumeConstraint
{

};

class CollisionConstraint : public Constraint
{
public:
    CollisionConstraint( int id, float weight )
        : Constraint( std::vector<int>{id}, weight )
    {

    }

    void SetCollision( Vector3 n, Vector3 b, float depth )
    {
        _collide = true;
        _n = n;
        _b = b;
        _depth = depth;
    }

    void ClearCollision()
    {
        _collide = false;
    }

    void ProjectCollision( const Matrix3X& pos, Matrix3X& proj, const Matrix3X& last ) const
    {
        if (_collide)
        {
            int i = _indices[0];
            Vector3 movement = pos.col( i ) - last.col( i );
            Vector3 move_n = movement.dot( _n ) * _n;
            Vector3 move_t = movement - move_n;
            float damp = std::clamp( 2 * _depth, 0.f, 1.f );

            proj.col( _indices[0] ) = _b - damp * move_t;
        }
        else
        {
            proj.col( _indices[0] ) = pos.col( _indices[0] );
        }
    }

    virtual void Project( const Matrix3X& pos, Matrix3X& proj ) const override
    {
    }

    virtual void AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const override
    {
        _loc = total_id;
        triplets.push_back( SparseMatrixTriplet( total_id, _indices[0], _weight * 1 ) );
        total_id += 1;
    }

    void FilterVelocity( Matrix3X& velocity ) const
    {
        if (_collide)
        {
            Vector3 v = velocity.col( _indices[0] );
            Vector3 v_n = v.dot( _n ) * _n;
            Vector3 v_t = v - v_n;

            float damp = std::clamp( 1.f - 10 * _depth, 0.f, 1.f );

            velocity.col( _indices[0] ) = damp * v_t - damp * v_n;
        }
    }

public:
    bool _collide = false;
    Vector3 _n;
    Vector3 _b;
    float _depth;
};

class TetraStrainConstraint : public Constraint
{
public:
    TetraStrainConstraint( const std::vector<int>& indices, float weight, const Matrix3X& positions );

    virtual void AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const override;

    virtual void Project( const Matrix3X& pos, Matrix3X& proj ) const override;

    Matrix3 _rest;
};


}
