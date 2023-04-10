#pragma once
#include "PDMetaballModel.h"

namespace PD
{
class PDMetaballModelFC
{
public:
    PDMetaballModelFC( PDMetaballModelConfig config, PDMetaballHalfEdgeMesh* surface );

protected:
    PDMetaballModelConfig _cfg;

    std::unique_ptr<SphereMesh<PDMetaballModel::Particle>> _mesh;
    Matrix3X _rest_pos;          //3*n
    bool _simulate = false;
    SparseMatrix _M; //3n*3n
    SparseMatrix _Minv; //3n*3n

    Matrix3X _x_last;;
    Matrix3X _x;       //3*n
    Matrix3X _v;       //3*n
    Matrix3X _pene;
    Matrix3X _friction;
    Matrix3X _momentum;         //3*n
    Matrix3X _fext;    //3*n
    Eigen::SimplicialLDLT<SparseMatrix> _llt;
    SparseMatrix _At;
    SparseMatrix _N;
    Matrix3X _projections;

    SparseMatrix _Dinv;
    SparseMatrix _LU;
    SparseMatrix _B;
};

}