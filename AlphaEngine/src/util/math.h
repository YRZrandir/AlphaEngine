#pragma once
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <CGAL/Vector_3.h>
#include <CGAL/Point_3.h>
#include <glm/glm.hpp>
#include "MathTypeConverter.h"

using Real = float;
using SparseMatrix = Eigen::SparseMatrix<Real>;
using SparseMatrixTriplet = Eigen::Triplet<Real, int>;

//Fix for Eigen3.3.9
namespace Eigen
{
#define EIGEN_MAKE_TYPEDEFS(Size, SizeSuffix)                     \
    /** \ingroup matrixtypedefs */                                    \
    /** \brief \cpp11 */                                              \
    template <typename Type>                                          \
    using Matrix##SizeSuffix = Matrix<Type, Size, Size>;              \
    /** \ingroup matrixtypedefs */                                    \
    /** \brief \cpp11 */                                              \
    template <typename Type>                                          \
    using Vector##SizeSuffix = Matrix<Type, Size, 1>;                 \
    /** \ingroup matrixtypedefs */                                    \
    /** \brief \cpp11 */                                              \
    template <typename Type>                                          \
    using RowVector##SizeSuffix = Matrix<Type, 1, Size>;
#define EIGEN_MAKE_FIXED_TYPEDEFS(Size)                           \
/** \ingroup matrixtypedefs */                                    \
/** \brief \cpp11 */                                              \
template <typename Type>                                          \
using Matrix##Size##X = Matrix<Type, Size, Dynamic>;              \
/** \ingroup matrixtypedefs */                                    \
/** \brief \cpp11 */                                              \
template <typename Type>                                          \
using Matrix##X##Size = Matrix<Type, Dynamic, Size>;
EIGEN_MAKE_TYPEDEFS( 2, 2 )
EIGEN_MAKE_TYPEDEFS( 3, 3 )
EIGEN_MAKE_TYPEDEFS( 4, 4 )
EIGEN_MAKE_TYPEDEFS( Dynamic, X )
EIGEN_MAKE_FIXED_TYPEDEFS( 2 )
EIGEN_MAKE_FIXED_TYPEDEFS( 3 )
EIGEN_MAKE_FIXED_TYPEDEFS( 4 )
}


using Vector2 = Eigen::Vector2<Real>;
using Vector3 = Eigen::Vector3<Real>;
using Vector4 = Eigen::Vector4<Real>;
using Matrix3 = Eigen::Matrix3<Real>;
using Matrix4 = Eigen::Matrix4<Real>;
using VectorX = Eigen::VectorX<Real>;
using MatrixX = Eigen::MatrixX<Real>;
using Matrix3X = Eigen::Matrix3X<Real>;
using MatrixX3 = Eigen::MatrixX3<Real>;
