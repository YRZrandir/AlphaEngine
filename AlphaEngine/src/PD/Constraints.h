#pragma once
#include <concepts>
#include <Eigen/Eigen>
#include "model/SphereMesh.h"

namespace PD
{

template <std::floating_point T>
class Constraint
{
public:
    Constraint( const std::vector<int>& indices, T weight )
        :_indices( indices ), _weight( weight )
    {

    }
    virtual void Project( const Eigen::Matrix3X<T>& pos, Eigen::Matrix3X<T>& proj ) const = 0;
    virtual void AddConstraint( std::vector<Eigen::Triplet<T, int>>& triplets, int& total_id, float w = 0.5 ) const = 0;
    virtual Eigen::SparseMatrix<T> GetS( int nb_points ) const
    {
        std::vector<Eigen::Triplet<T, int>> triplets;
        for (int nc = 0; nc < _indices.size(); nc++)
        {
            triplets.emplace_back( nc, _indices[nc], 1.0 );
        }
        Eigen::SparseMatrix<T> m( _indices.size(), nb_points );
        m.setFromTriplets( triplets.begin(), triplets.end() );
        return m;
    }
    virtual Eigen::SparseMatrix<T> GetA() const = 0;
    virtual Eigen::Matrix3X<T> GetP( const Eigen::Matrix3X<T>& pos ) const = 0;

    std::vector<int> _indices;
    mutable int _loc{ -1 };

    T _weight;
};

template <std::floating_point T>
class EdgeConstraint : public Constraint<T>
{
public:
    EdgeConstraint( int i0, int i1, float weight, const Eigen::Matrix3X<T>& pos );
    virtual void AddConstraint( std::vector<Eigen::Triplet<T, int>>& triplets, int& total_id, float w = 0.5 ) const override;
    virtual void Project( const Eigen::Matrix3X<T>& pos, Eigen::Matrix3X<T>& proj ) const override;
    virtual Eigen::SparseMatrix<T> GetA() const override;
    virtual Eigen::Matrix3X<T> GetP( const Eigen::Matrix3X<T>& pos ) const override;

    float _rest = 0.f;
};

template <std::floating_point T>
PD::EdgeConstraint<T>::EdgeConstraint( int i0, int i1, float weight, const Eigen::Matrix3X<T>& pos )
    :Constraint<T>( std::vector<int>{i0, i1}, weight )
{
    float len = (pos.col( i0 ) - pos.col( i1 )).norm();
    _rest = 1.f / len;
    this->_weight *= std::sqrt( len );
}

template <std::floating_point T>
void PD::EdgeConstraint<T>::AddConstraint( std::vector<Eigen::Triplet<T, int>>& triplets, int& total_id, float w ) const
{
    this->_loc = total_id;
    triplets.push_back( Eigen::Triplet<T, int>( this->_loc, this->_indices[0], -this->_weight * _rest ) );
    triplets.push_back( Eigen::Triplet<T, int>( this->_loc, this->_indices[1], this->_weight * _rest ) );
    total_id += 1;
}

template <std::floating_point T>
void PD::EdgeConstraint<T>::Project( const Eigen::Matrix3X<T>& pos, Eigen::Matrix3X<T>& proj ) const
{
    Eigen::Vector3<T> edge = pos.col( this->_indices[1] ) - pos.col( this->_indices[0] );
    edge.normalize();
    proj.col( this->_loc ) = this->_weight * edge;
}

template <std::floating_point T>
Eigen::SparseMatrix<T> PD::EdgeConstraint<T>::GetA() const
{
    std::vector<Eigen::Triplet<T, int>> triplets;
    Eigen::SparseMatrix<T> m( 1, this->_indices.size() );
    triplets.emplace_back( 0, 0, -this->_weight * _rest );
    triplets.emplace_back( 0, 1, this->_weight * _rest );
    m.setFromTriplets( triplets.begin(), triplets.end() );
    return m;
}

template <std::floating_point T>
Eigen::Matrix3X<T> PD::EdgeConstraint<T>::GetP( const Eigen::Matrix3X<T>& pos ) const
{
    Eigen::Vector3<T> edge = pos.col( this->_indices[1] ) - pos.col( this->_indices[0] );
    edge.normalize();
    return this->_weight * edge;
}

template <std::floating_point T>
class AttachConstraint : public Constraint<T>
{
public:
    AttachConstraint( int id, float weight, Eigen::Vector3<T> fixed_pos )
        : Constraint<T>( std::vector<int>{id}, weight ), _fixed_pos( fixed_pos )
    {};

    virtual void AddConstraint( std::vector<Eigen::Triplet<T, int>>& triplets, int& total_id, float w = 0.5 ) const override;
    virtual void Project( const Eigen::Matrix3X<T>& pos, Eigen::Matrix3X<T>& proj ) const override;
    virtual Eigen::SparseMatrix<T> GetA() const override;
    virtual Eigen::Matrix3X<T> GetP( const Eigen::Matrix3X<T>& pos ) const override;
    Eigen::Vector3<T> _fixed_pos;
};

template <std::floating_point T>
void PD::AttachConstraint<T>::AddConstraint( std::vector<Eigen::Triplet<T, int>>& triplets, int& total_id, float w ) const
{
    this->_loc = total_id;
    triplets.push_back( Eigen::Triplet<T, int>( total_id, this->_indices[0], std::pow( this->_weight, w ) ) );
    total_id += 1;
}

template <std::floating_point T>
void PD::AttachConstraint<T>::Project( const Eigen::Matrix3X<T>& pos, Eigen::Matrix3X<T>& proj ) const
{
    proj.col( this->_loc ) = _fixed_pos;
}

template <std::floating_point T>
Eigen::SparseMatrix<T> PD::AttachConstraint<T>::GetA() const
{
    std::vector<Eigen::Triplet<T, int>> triplets;
    Eigen::SparseMatrix<T> m( 1, 1 );
    triplets.emplace_back( 0, 0, 1 );
    m.setFromTriplets( triplets.begin(), triplets.end() );
    return m;
}

template <std::floating_point T>
Eigen::Matrix3X<T> PD::AttachConstraint<T>::GetP( const Eigen::Matrix3X<T>& pos ) const
{
    return _fixed_pos;
}

template <std::floating_point T>
class TetraStrainConstraint : public Constraint<T>
{
public:
    TetraStrainConstraint( const std::vector<int>& indices, T weight, const Eigen::Matrix3X<T>& positions );

    virtual void AddConstraint( std::vector<Eigen::Triplet<T, int>>& triplets, int& total_id, float w = 0.5 ) const override;
    virtual void Project( const Eigen::Matrix3X<T>& pos, Eigen::Matrix3X<T>& proj ) const override;
    virtual Eigen::SparseMatrix<T> GetA() const override;
    virtual Eigen::Matrix3X<T> GetP( const Eigen::Matrix3X<T>& pos ) const override;

    Eigen::Matrix3<T> _rest;
};

template <std::floating_point T>
PD::TetraStrainConstraint<T>::TetraStrainConstraint( const std::vector<int>& indices, T weight, const Eigen::Matrix3X<T>& positions )
    :Constraint<T>( indices, weight )
{
    Eigen::Matrix3<T> edges;
    for (int i = 0; i < 3; i++)
        edges.col( i ) = positions.col( indices[i + 1] ) - positions.col( indices[0] );
    _rest = edges.inverse();

    float v = edges.determinant() / 6.f;
    this->_weight *= std::sqrt( std::abs( v ) );
}

template <std::floating_point T>
void PD::TetraStrainConstraint<T>::AddConstraint( std::vector<Eigen::Triplet<T, int>>& triplets, int& total_id, float w ) const
{
    this->_loc = total_id;
    int n = 3;
    for (int i = 0; i < n; ++i) {
        triplets.emplace_back( this->_loc + i, this->_indices[0], -this->_weight * (_rest( 0, i ) + _rest( 1, i ) + _rest( 2, i )) );
        triplets.emplace_back( this->_loc + i, this->_indices[1], this->_weight * _rest( 0, i ) );
        triplets.emplace_back( this->_loc + i, this->_indices[2], this->_weight * _rest( 1, i ) );
        triplets.emplace_back( this->_loc + i, this->_indices[3], this->_weight * _rest( 2, i ) );
    }
    total_id += n;
}

template <std::floating_point T>
void PD::TetraStrainConstraint<T>::Project( const Eigen::Matrix3X<T>& pos, Eigen::Matrix3X<T>& proj ) const
{
    Eigen::Matrix3<T> edges;
    for (int i = 0; i < 3; ++i)
        edges.col( i ) = pos.col( this->_indices[i + 1] ) - pos.col( this->_indices[0] );
    Eigen::Matrix3<T> F = edges * _rest;
    Eigen::JacobiSVD<Eigen::Matrix3<T>> svd( F, Eigen::ComputeFullU | Eigen::ComputeFullV );
    Eigen::Vector3<T> S = svd.singularValues();
    S( 0 ) = std::clamp( S( 0 ), 1.f, 1.f );
    S( 1 ) = std::clamp( S( 1 ), 1.f, 1.f );
    S( 2 ) = std::clamp( S( 2 ), 1.f, 1.f );
    if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0f) S( 2 ) = -S( 2 );
    F = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
    proj.block<3, 3>( 0, this->_loc ) = this->_weight * F;
}

template <std::floating_point T>
Eigen::SparseMatrix<T> PD::TetraStrainConstraint<T>::GetA() const
{
    std::vector<Eigen::Triplet<T, int>> triplets;
    Eigen::SparseMatrix<T> m( 3, this->_indices.size() );
    for (int i = 0; i < 3; i++)
    {
        triplets.emplace_back( i, 0, -this->_weight * (_rest( 0, i ) + _rest( 1, i ) + _rest( 2, i )) );
        triplets.emplace_back( i, 1, this->_weight * _rest( 0, i ) );
        triplets.emplace_back( i, 2, this->_weight * _rest( 1, i ) );
        triplets.emplace_back( i, 3, this->_weight * _rest( 2, i ) );
    }
    m.setFromTriplets( triplets.begin(), triplets.end() );
    return m;
}

template <std::floating_point T>
Eigen::Matrix3X<T> PD::TetraStrainConstraint<T>::GetP( const Eigen::Matrix3X<T>& pos ) const
{
    Eigen::Matrix3<T> edges;
    for (int i = 0; i < 3; ++i)
        edges.col( i ) = pos.col( this->_indices[i + 1] ) - pos.col( this->_indices[0] );
    Eigen::Matrix3<T> F = edges * _rest;
    Eigen::JacobiSVD<Eigen::Matrix3<T>> svd( F, Eigen::ComputeFullU | Eigen::ComputeFullV );
    Eigen::Vector3<T> S = svd.singularValues();
    S( 0 ) = std::clamp( S( 0 ), 1.f, 1.f );
    S( 1 ) = std::clamp( S( 1 ), 1.f, 1.f );
    S( 2 ) = std::clamp( S( 2 ), 1.f, 1.f );
    if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0f) S( 2 ) = -S( 2 );
    F = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();

    return this->_weight * F;
}

template <SphereType Sphere, std::floating_point T>
class MeshlessStrainConstraint : public Constraint<T>
{
public:
    MeshlessStrainConstraint( const std::vector<int>& indices, T weight, const Eigen::Matrix3X<T>& positions, SphereMesh<Sphere>* sphere_mesh, Eigen::Matrix3X<T>* x0 );
    virtual void AddConstraint( std::vector<Eigen::Triplet<T, int>>& triplets, int& total_id, float w = 0.5 ) const override;
    virtual void Project( const Eigen::Matrix3X<T>& pos, Eigen::Matrix3X<T>& proj ) const override;
    virtual Eigen::SparseMatrix<T> GetA() const override;
    virtual Eigen::Matrix3X<T> GetP( const Eigen::Matrix3X<T>& pos ) const override;

    Eigen::Matrix3<T> _invA;
    SphereMesh<Sphere>* _sphere_mesh;
    Eigen::Matrix3X<T>* _x0;
    T _avg_dist;
    std::vector<T> _w;
    Eigen::MatrixX3<T> _edges0;
    T _wsum;
private:
    T ComputeW( T r, T h ) const
    {
        if (r < h)
            return 315.0 * std::pow( h * h - r * r, 3.0 ) / (64.0 * 3.1415926 * std::pow( h, 9.0 ));
        else
            return 0.0001;
    }

    Eigen::Matrix3<T> ComputeF( const Eigen::Matrix3X<T>& pos ) const;
};

template <SphereType Sphere, std::floating_point T>
PD::MeshlessStrainConstraint<Sphere, T>::MeshlessStrainConstraint( const std::vector<int>& indices,
    T weight, const Eigen::Matrix3X<T>& positions, SphereMesh<Sphere>* sphere_mesh, Eigen::Matrix3X<T>* x0 )
    :Constraint<T>( indices, weight ), _sphere_mesh( sphere_mesh ), _x0( x0 )
{
    Eigen::Matrix3<T> A;
    A.setZero();

    _avg_dist = 0.f;
    for (int j : indices)
    {
        if (j == indices[0])
            continue;
        Eigen::Vector3<T> d = positions.col( indices[0] ) - positions.col( j );
        _avg_dist += d.norm();
    }
    _avg_dist /= (indices.size() - 1);

    _wsum = 0.f;

    int cnt = 1;
    _w.resize( indices.size(), 0.f );
    for (int j : indices)
    {
        if (j == indices[0])
            continue;
        Eigen::Vector3<T> xij = positions.col( j ) - positions.col( indices[0] );

        T wij = 0.f;
        T r = xij.norm();
        T h = _avg_dist * 3;
        if (r < h)
            wij = ComputeW( r, h ) * _sphere_mesh->Ball( j ).m;

        _w[cnt] = wij;
        _wsum += wij;
        cnt++;
        A += wij * xij * xij.transpose();
    }
    A /= _wsum;

    T detA = A.determinant();

    if (std::abs( detA ) < 1e-8)
    {
        _invA = EigenSafeInverse( A );
    }
    else
    {
        _invA = A.inverse();
    }

    _edges0.resize( this->_indices.size() - 1, 3 );
    for (int i = 0; i < this->_indices.size() - 1; i++)
    {
        _edges0.row( i ) = (_x0->col( this->_indices[i + 1] ) - _x0->col( this->_indices[0] )).transpose();
    }
    _edges0 = _edges0 * _invA.transpose();
}

template <SphereType Sphere, std::floating_point T>
void PD::MeshlessStrainConstraint<Sphere, T>::AddConstraint( std::vector<Eigen::Triplet<T, int>>& triplets, int& total_id, float w ) const
{
    this->_loc = total_id;
    float v1 = ((float)(this->_indices.size() - 1) / this->_indices.size());
    float v2 = -(1.0f / this->_indices.size());

    for (int i = 0; i < this->_indices.size(); i++)
    {
        for (int j = 0; j < this->_indices.size(); j++)
        {
            float v = 0;
            if (i == j)
                v += 1;
            if (j == 0)
                v -= 1;
            triplets.push_back( Eigen::Triplet<T, int>( total_id + i, this->_indices[j], std::pow( this->_weight, w ) * v ) );
        }
    }
    total_id += this->_indices.size();
}

template <SphereType Sphere, std::floating_point T>
void PD::MeshlessStrainConstraint<Sphere, T>::Project( const Eigen::Matrix3X<T>& pos, Eigen::Matrix3X<T>& proj ) const
{
    Eigen::Matrix3<T> F = ComputeF( pos );

    Sphere& pi = _sphere_mesh->Ball( this->_indices[0] );
    pi.gu = F;

    Eigen::Matrix3<T> U, V;
    Eigen::Vector3<T> S;
    SVD( &U, &S, &V, F );

    Eigen::Matrix3<T> t = U * V.transpose();
    pi.R = t;

    proj.col( this->_loc ) = Eigen::Vector3<T>( 0.f, 0.f, 0.f );
    for (int j = 1; j < this->_indices.size(); j++)
    {
        proj.col( this->_loc + j ) = t * (_x0->col( this->_indices[j] ) - _x0->col( this->_indices[0] ));
    }
}

template <SphereType Sphere, std::floating_point T>
Eigen::Matrix3<T> PD::MeshlessStrainConstraint<Sphere, T>::ComputeF( const Eigen::Matrix3X<T>& pos ) const
{
    //Eigen::Vector3<T> ui = pos.col( this->_indices[0] ) - _x0->col( this->_indices[0] );
    //Eigen::Matrix3<T> S;
    //S.setZero();
    //for (int i = 1; i < this->_indices.size(); i++)
    //{
    //    int j = this->_indices[i];
    //    Eigen::Vector3<T> uj = pos.col( j ) - _x0->col( j );
    //    Eigen::Vector3<T> xij = _x0->col( j ) - _x0->col( this->_indices[0] );
    //    float wij = _w[i];
    //    S += wij * (uj - ui) * xij.transpose();
    //}
    //S /= _wsum;
    //Eigen::Matrix3<T> F = S * _invA.transpose() + Eigen::Matrix3<T>::Identity();

    Eigen::Matrix3X<T> edges( 3, this->_indices.size() - 1 );
    for (int i = 0; i < this->_indices.size() - 1; i++)
    {
        edges.col( i ) = _w[i + 1] * (pos.col( this->_indices[i + 1] ) - pos.col( this->_indices[0] ) - _x0->col( this->_indices[i + 1] ) + _x0->col( this->_indices[0] ));
    }
    Eigen::Matrix3<T> F = (edges * _edges0) / _wsum + Eigen::Matrix3<T>::Identity();

    return F;
}

template <SphereType Sphere, std::floating_point T>
Eigen::SparseMatrix<T> PD::MeshlessStrainConstraint<Sphere, T>::GetA() const
{
    std::vector<Eigen::Triplet<T, int>> triplets;
    Eigen::SparseMatrix<T> m( this->_indices.size(), this->_indices.size() );

    for (int i = 0; i < this->_indices.size(); i++)
    {
        for (int j = 0; j < this->_indices.size(); j++)
        {
            float v = 0;
            if (i == j)
                v += 1;
            if (j == 0)
                v -= 1;
            triplets.emplace_back( i, j, v );
        }
    }

    m.setFromTriplets( triplets.begin(), triplets.end() );
    return m;
}

template <SphereType Sphere, std::floating_point T>
Eigen::Matrix3X<T> PD::MeshlessStrainConstraint<Sphere, T>::GetP( const Eigen::Matrix3X<T>& pos ) const
{
    Eigen::Matrix3<T> F = ComputeF( pos );

    Sphere& pi = _sphere_mesh->Ball( this->_indices[0] );
    pi.gu = F;

    Eigen::Matrix3<T> U, V;
    Eigen::Vector3<T> S;
    SVD( &U, &S, &V, F );

    Eigen::Matrix3<T> t = U * V.transpose();
    pi.R = t;

    Eigen::Matrix3X<T> p;
    p.resize( 3, this->_indices.size() );

    p.col( 0 ) = Eigen::Vector3<T>( 0.f, 0.f, 0.f );
    for (int j = 1; j < this->_indices.size(); j++)
    {
        p.col( j ) = t * (_x0->col( this->_indices[j] ) - _x0->col( this->_indices[0] ));
    }

    return p;
}
}
