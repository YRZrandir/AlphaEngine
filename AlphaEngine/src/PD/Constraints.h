#pragma once
#include "util/Math.h"
#include "glm/glm.hpp"
#include "glm/ext/scalar_constants.hpp"
#include "model/SphereMesh.h"

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
    virtual SparseMatrix GetS( int nb_points ) const
    {
        std::vector<SparseMatrixTriplet> triplets;
        for (int nc = 0; nc < _indices.size(); nc++)
        {
            triplets.emplace_back( nc, _indices[nc], 1.0 );
        }
        SparseMatrix m( _indices.size(), nb_points );
        m.setFromTriplets( triplets.begin(), triplets.end() );
        return m;
    }
    virtual SparseMatrix GetA() const = 0;

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
    virtual SparseMatrix GetA() const override;

    float _rest = 0.f;
};

template <SphereType Sphere>
class MeshlessStrainConstraint : public Constraint
{
public:
    MeshlessStrainConstraint( const std::vector<int>& indices, float weight, const Matrix3X& positions, SphereMesh<Sphere>* sphere_mesh, Matrix3X* x0 );
    virtual void AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const override;
    virtual void Project( const Matrix3X& pos, Matrix3X& proj ) const override;
    virtual SparseMatrix GetA() const override;

    Matrix3 _invA;
    SphereMesh<Sphere>* _sphere_mesh;
    Matrix3X* _x0;
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

template <SphereType Sphere>
PD::MeshlessStrainConstraint<Sphere>::MeshlessStrainConstraint( const std::vector<int>& indices,
    float weight, const Matrix3X& positions, SphereMesh<Sphere>* sphere_mesh, Matrix3X* x0 )
    :Constraint( indices, weight ), _sphere_mesh( sphere_mesh ), _x0( x0 )
{
    Matrix3 A;
    A.setZero();

    _avg_dist = 0.f;
    for (int j : indices)
    {
        if (j == indices[0])
            continue;
        Vector3 d = positions.col( indices[0] ) - positions.col( j );
        _avg_dist += d.norm();
    }
    _avg_dist /= (indices.size() - 1);

    float wsum = 0.f;

    int cnt = 1;
    _w.resize( indices.size(), 0.f );
    for (int j : indices)
    {
        if (j == indices[0])
            continue;
        Vector3 xij = positions.col( j ) - positions.col( indices[0] );

        float wij = 0.f;
        float r = xij.norm();
        float h = _avg_dist * 3;
        if (r < h)
            wij = ComputeW( r, h ) * _sphere_mesh->Ball( j ).m;

        _w[cnt] = wij;
        wsum += wij;
        cnt++;
        A += wij * xij * xij.transpose();
    }
    A /= wsum;

    double detA = A.determinant();

    if (std::abs( detA ) < 1e-8f)
    {
        _invA = EigenSafeInverse( A );
    }
    else
    {
        _invA = A.inverse();
    }

    _weight *= std::sqrt( std::powf( _sphere_mesh->Ball( _indices[0] ).r, 3.f ) );
}

template <SphereType Sphere>
void PD::MeshlessStrainConstraint<Sphere>::AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const
{
    _loc = total_id;
    float v1 = ((float)(_indices.size() - 1) / _indices.size());
    float v2 = -(1.0f / _indices.size());

    for (int i = 0; i < _indices.size(); i++)
    {
        for (int j = 0; j < _indices.size(); j++)
        {
            float v = 0;
            if (i == j)
                v += 1;
            if (j == 0)
                v -= 1;
            triplets.push_back( SparseMatrixTriplet( total_id + i, _indices[j], _weight * v ) );
        }
    }
    total_id += _indices.size();
}

template <SphereType Sphere>
void PD::MeshlessStrainConstraint<Sphere>::Project( const Matrix3X& pos, Matrix3X& proj ) const
{
    Matrix3 F = ComputeF( pos );

    Sphere& pi = _sphere_mesh->Ball( _indices[0] );
    pi.gu = F;

    Matrix3 U, V;
    Vector3 S;
    SVD( &U, &S, &V, F );

    Matrix3 T = U * V.transpose();
    pi.R = T;

    proj.col( _loc ) = Vector3( 0.f, 0.f, 0.f );
    for (int j = 1; j < _indices.size(); j++)
    {
        auto x0ij = _x0->col( _indices[j] ) - _x0->col( _indices[0] );
        Vector3 after_rot = T * x0ij;
        proj.col( _loc + j ) = _weight * after_rot;
    }
}

template <SphereType Sphere>
Matrix3 PD::MeshlessStrainConstraint<Sphere>::ComputeF( const Matrix3X& pos ) const
{
    Vector3 ui = pos.col( _indices[0] ) - _x0->col( _indices[0] );

    Vector3 sx( 0.f, 0.f, 0.f );
    Vector3 sy( 0.f, 0.f, 0.f );
    Vector3 sz( 0.f, 0.f, 0.f );

    float wsum = 0.f;

    int cnt = 0;
    for (int j : _indices)
    {
        if (cnt == 0)
        {
            cnt++;
            continue;
        }
        Vector3 uj = pos.col( j ) - _x0->col( j );

        Vector3 xij = _x0->col( j ) - _x0->col( _indices[0] );

        float wij = _w[cnt];
        cnt++;

        wsum += wij;
        sx += (uj[0] - ui[0]) * xij * wij;
        sy += (uj[1] - ui[1]) * xij * wij;
        sz += (uj[2] - ui[2]) * xij * wij;
    }

    sx /= wsum;
    sy /= wsum;
    sz /= wsum;

    Vector3 dux = _invA * sx;
    Vector3 duy = _invA * sy;
    Vector3 duz = _invA * sz;

    Matrix3 F;
    F.col( 0 ) = dux;
    F.col( 1 ) = duy;
    F.col( 2 ) = duz;

    F.transposeInPlace();

    F( 0, 0 ) += 1.f;
    F( 1, 1 ) += 1.f;
    F( 2, 2 ) += 1.f;

    return F;
}

template <SphereType Sphere>
SparseMatrix PD::MeshlessStrainConstraint<Sphere>::GetA() const
{
    std::vector<SparseMatrixTriplet> triplets;
    SparseMatrix m( _indices.size(), _indices.size() );

    for (int i = 0; i < _indices.size(); i++)
    {
        for (int j = 0; j < _indices.size(); j++)
        {
            float v = 0;
            if (i == j)
                v += 1;
            if (j == 0)
                v -= 1;
            triplets.emplace_back( i, j, _weight * v );
        }
    }

    m.setFromTriplets( triplets.begin(), triplets.end() );
    return m;
}

class AttachConstraint : public Constraint
{
public:
    AttachConstraint( int id, float weight, Vector3 fixed_pos )
        : Constraint( std::vector<int>{id}, weight ), _fixed_pos( fixed_pos )
    {};

    virtual void AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const override;
    virtual void Project( const Matrix3X& pos, Matrix3X& proj ) const override;
    virtual SparseMatrix GetA() const override;

    Vector3 _fixed_pos;
};

class TetraStrainConstraint : public Constraint
{
public:
    TetraStrainConstraint( const std::vector<int>& indices, float weight, const Matrix3X& positions );

    virtual void AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const override;
    virtual void Project( const Matrix3X& pos, Matrix3X& proj ) const override;
    virtual SparseMatrix GetA() const override;

    Matrix3 _rest;
};


}
