#ifndef GEOMETRY_UTILS_ROTATIONN_H
#define GEOMETRY_UTILS_ROTATIONN_H

#include <ostream>
#include <Eigen/Core>
#include "VectorNBase.h"
#include "MatrixNxNBase.h"

namespace geometry_utils {

template <typename T, size_t N>
struct RotationNBase : MatrixNxNBase<T, N> {
  RotationNBase() : MatrixNxNBase<T, N>() { this->Eye(); }

  RotationNBase(const RotationNBase& in) : MatrixNxNBase<T, N>(in.data) {}
  RotationNBase(const boost::array<T, N* N>& in) : MatrixNxNBase<T, N>(in) {}
  RotationNBase(T (&in)[N * N]) : MatrixNxNBase<T, N>(in) {}
  RotationNBase(const Eigen::Matrix<T, N, N>& in) : MatrixNxNBase<T, N>(in) {}
  RotationNBase(const MatrixNxNBase<T, N>& in) : MatrixNxNBase<T, N>(in) {}

  virtual inline MatrixNxNBase<T, N> Inv() const {
    return this->Trans();
  }
};

template <size_t N>
inline RotationNBase<float, N> operator*(const float& lhs,
                                         const RotationNBase<float, N>& rhs) {
  return RotationNBase<float, N>(rhs * lhs);
}

template <size_t N>
inline RotationNBase<double, N> operator*(const double& lhs,
                                          const RotationNBase<double, N>& rhs) {
  return RotationNBase<double, N>(rhs * lhs);
}

template <typename T, size_t N>
inline RotationNBase<T, N> Inv(const RotationNBase<T, N>& m) {
  return m.Inv();
}

}

#endif
