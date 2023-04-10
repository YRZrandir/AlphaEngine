#include "Constraints.h"
#include "PDGPUMetaballModel.h"

PD::MeshlessStrainConstraint::MeshlessStrainConstraint( const std::vector<int>& indices,
    float weight, const Matrix3X& positions, PDMetaballModel* model )
    :Constraint( indices, weight ), _model( model )
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
            wij = ComputeW( r, h ) * _model->_mesh->Ball( j ).m;

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

    _weight *= std::sqrt( std::powf( _model->_mesh->Ball( _indices[0] ).r, 3.f ) );
}

void PD::MeshlessStrainConstraint::AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const
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
            //if (i == j)
            //    v = v1;
            //else
            //    v = v2;

            triplets.push_back( SparseMatrixTriplet( total_id + i, _indices[j], _weight * v ) );
        }
    }
    total_id += _indices.size();
}

void PD::MeshlessStrainConstraint::Project( const Matrix3X& pos, Matrix3X& proj ) const
{
    Matrix3 F = ComputeF( pos );

    PDMetaballModel::Particle& pi = _model->_mesh->Ball( _indices[0] );
    pi.gu = F;

    Matrix3 U, V;
    Vector3 S;
    SVD( &U, &S, &V, F );

    constexpr Real min = 1.0f;
    constexpr Real max = 1.0f;
    //Matrix3 Snew;
    //Snew << std::clamp( S( 0 ), min, max ), 0.0, 0.0,
    //    0.0, std::clamp( S( 1 ), min, max ), 0.0,
    //    0.0, 0.0, std::clamp( S( 2 ), min, max );

    Matrix3 T = U * V.transpose();
    pi.R = T;

    proj.col( _loc ) = Vector3( 0.f, 0.f, 0.f );
    for (int j = 1; j < _indices.size(); j++)
    {
        Vector3 x0ij = _model->_rest_pos.col( _indices[j] ) - _model->_rest_pos.col( _indices[0] );
        Vector3 after_rot = T * x0ij;
        proj.col( _loc + j ) = _weight * after_rot;
    }

    //float energy = (F - T).norm();
    //auto color = tinycolormap::GetHeatColor( std::log( energy * 2 + 1 ) );
    //_model->_mesh->Ball( _indices[0] ).color = glm::vec3( color.r(), color.g(), color.b() );
}

Matrix3 PD::MeshlessStrainConstraint::ComputeF( const Matrix3X& pos ) const
{
    Vector3 ui = pos.col( _indices[0] ) - _model->_rest_pos.col( _indices[0] );

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
        Vector3 uj = pos.col( j ) - _model->_rest_pos.col( j );

        Vector3 xij = _model->_rest_pos.col( j ) - _model->_rest_pos.col( _indices[0] );

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


PD::GPUMeshlessStrainConstraint::GPUMeshlessStrainConstraint( const std::vector<int>& indices,
    float weight, const Matrix3X& positions, PDGPUMetaballModel* model )
    :Constraint( indices, weight ), _model( model )
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
            wij = ComputeW( r, h ) * _model->_mesh->Ball( j ).m;

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

    _weight *= std::sqrt( std::powf( _model->_mesh->Ball( _indices[0] ).r, 3.f ) );
}

void PD::GPUMeshlessStrainConstraint::AddConstraint( std::vector<SparseMatrixTriplet>& triplets, int& total_id ) const
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
            //if (i == j)
            //    v = v1;
            //else
            //    v = v2;

            triplets.push_back( SparseMatrixTriplet( total_id + i, _indices[j], _weight * v ) );
        }
    }
    total_id += _indices.size();
}

void PD::GPUMeshlessStrainConstraint::Project( const Matrix3X& pos, Matrix3X& proj ) const
{
    Matrix3 F = ComputeF( pos );

    PDGPUMetaballModel::Particle& pi = _model->_mesh->Ball( _indices[0] );
    pi.gu = F;

    Matrix3 U, V;
    Vector3 S;
    SVD( &U, &S, &V, F );

    constexpr Real min = 1.0f;
    constexpr Real max = 1.0f;
    //Matrix3 Snew;
    //Snew << std::clamp( S( 0 ), min, max ), 0.0, 0.0,
    //    0.0, std::clamp( S( 1 ), min, max ), 0.0,
    //    0.0, 0.0, std::clamp( S( 2 ), min, max );

    Matrix3 T = U * V.transpose();
    pi.R = T;

    proj.col( _loc ) = Vector3( 0.f, 0.f, 0.f );
    for (int j = 1; j < _indices.size(); j++)
    {
        Vector3 x0ij = _model->_rest_pos.col( _indices[j] ) - _model->_rest_pos.col( _indices[0] );
        Vector3 after_rot = T * x0ij;
        proj.col( _loc + j ) = _weight * after_rot;
    }

    //float energy = (F - T).norm();
    //auto color = tinycolormap::GetHeatColor( std::log( energy * 2 + 1 ) );
    //_model->_mesh->Ball( _indices[0] ).color = glm::vec3( color.r(), color.g(), color.b() );
}

Matrix3 PD::GPUMeshlessStrainConstraint::ComputeF( const Matrix3X& pos ) const
{
    Vector3 ui = pos.col( _indices[0] ) - _model->_rest_pos.col( _indices[0] );

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
        Vector3 uj = pos.col( j ) - _model->_rest_pos.col( j );

        Vector3 xij = _model->_rest_pos.col( j ) - _model->_rest_pos.col( _indices[0] );

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
        triplets.push_back( SparseMatrixTriplet( _loc + i, _indices[0], -_weight * (_rest( 0, i ) + _rest( 1, i ) + _rest( 2, i )) ) );
        triplets.push_back( SparseMatrixTriplet( _loc + i, _indices[1], _weight * _rest( 0, i ) ) );
        triplets.push_back( SparseMatrixTriplet( _loc + i, _indices[2], _weight * _rest( 1, i ) ) );
        triplets.push_back( SparseMatrixTriplet( _loc + i, _indices[3], _weight * _rest( 2, i ) ) );
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
