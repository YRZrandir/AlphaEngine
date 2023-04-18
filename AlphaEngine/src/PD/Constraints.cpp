#include "Constraints.h"
#include "PDGPUMetaballModel.h"

namespace PD
{
PD::TetraStrainConstraint::TetraStrainConstraint( const std::vector<int>& indices, float weight, const Matrix3X& positions )
    :Constraint( indices, weight )
{
    Matrix3 edges;
    for (int i = 0; i < 3; i++)
        edges.col( i ) = positions.col( indices[i + 1] ) - positions.col( indices[0] );
    _rest = edges.inverse();

    float v = edges.determinant() / 6.f;
    _weight *= std::sqrt( std::abs( v ) );
}

void PD::TetraStrainConstraint::AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const
{
    _loc = total_id;
    int n = 3;
    for (int i = 0; i < n; ++i) {
        triplets.emplace_back( _loc + i, _indices[0], -_weight * (_rest( 0, i ) + _rest( 1, i ) + _rest( 2, i )) );
        triplets.emplace_back( _loc + i, _indices[1], _weight * _rest( 0, i ) );
        triplets.emplace_back( _loc + i, _indices[2], _weight * _rest( 1, i ) );
        triplets.emplace_back( _loc + i, _indices[3], _weight * _rest( 2, i ) );
    }
    total_id += n;
}

void PD::TetraStrainConstraint::Project( const Matrix3X& pos, Matrix3X& proj ) const
{
    Matrix3 edges;
    for (int i = 0; i < 3; ++i)
        edges.col( i ) = pos.col( _indices[i + 1] ) - pos.col( _indices[0] );
    Matrix3 F = edges * _rest;
    Eigen::JacobiSVD<Matrix3> svd( F, Eigen::ComputeFullU | Eigen::ComputeFullV );
    Vector3 S = svd.singularValues();
    S( 0 ) = std::clamp( S( 0 ), 1.f, 1.f );
    S( 1 ) = std::clamp( S( 1 ), 1.f, 1.f );
    S( 2 ) = std::clamp( S( 2 ), 1.f, 1.f );
    if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0f) S( 2 ) = -S( 2 );
    F = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
    proj.block<3, 3>( 0, _loc ) = _weight * F;
}

SparseMatrix PD::TetraStrainConstraint::GetA() const
{
    std::vector<SparseMatrixTriplet> triplets;
    SparseMatrix m( 3, _indices.size() );
    for (int i = 0; i < 3; i++)
    {
        triplets.emplace_back( i, 0, -_weight * (_rest( 0, i ) + _rest( 1, i ) + _rest( 2, i )) );
        triplets.emplace_back( i, 1, _weight * _rest( 0, i ) );
        triplets.emplace_back( i, 2, _weight * _rest( 1, i ) );
        triplets.emplace_back( i, 3, _weight * _rest( 2, i ) );
    }
    m.setFromTriplets( triplets.begin(), triplets.end() );
    return m;
}

PD::EdgeConstraint::EdgeConstraint( int i0, int i1, float weight, const Matrix3X& pos )
    :Constraint( std::vector<int>{i0, i1}, weight )
{
    float len = (pos.col( i0 ) - pos.col( i1 )).norm();
    _rest = 1.f / len;
    _weight *= std::sqrt( len );
}

void PD::EdgeConstraint::AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const
{
    _loc = total_id;
    triplets.push_back( SparseMatrixTriplet( _loc, _indices[0], -_weight * _rest ) );
    triplets.push_back( SparseMatrixTriplet( _loc, _indices[1], _weight * _rest ) );
    total_id += 1;
}

void PD::EdgeConstraint::Project( const Matrix3X& pos, Matrix3X& proj ) const
{
    Vector3 edge = pos.col( _indices[1] ) - pos.col( _indices[0] );
    edge.normalize();
    proj.col( _loc ) = _weight * edge;
}

SparseMatrix PD::EdgeConstraint::GetA() const
{
    std::vector<SparseMatrixTriplet> triplets;
    SparseMatrix m( 1, _indices.size() );
    triplets.emplace_back( 0, 0, -_weight * _rest );
    triplets.emplace_back( 0, 1, _weight * _rest );
    m.setFromTriplets( triplets.begin(), triplets.end() );
    return m;
}

void PD::AttachConstraint::AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const
{
    _loc = total_id;
    triplets.push_back( SparseMatrixTriplet( total_id, _indices[0], _weight * 1 ) );
    total_id += 1;
}

void PD::AttachConstraint::Project( const Matrix3X& pos, Matrix3X& proj ) const
{
    proj.col( _loc ) = _weight * _fixed_pos;
}

SparseMatrix PD::AttachConstraint::GetA() const
{
    std::vector<SparseMatrixTriplet> triplets;
    SparseMatrix m( 1, 1 );
    triplets.emplace_back( 0, 0, _weight );
    m.setFromTriplets( triplets.begin(), triplets.end() );
    return m;
}
}
