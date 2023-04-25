#include "Constraints.h"

namespace PD
{
template<>
class EdgeConstraint<float>;
template <>
class EdgeConstraint<double>;
template <>
class AttachConstraint<float>;
template <>
class AttachConstraint<double>;
template <>
class TetraStrainConstraint<float>;
template <>
class TetraStrainConstraint<double>;
}
