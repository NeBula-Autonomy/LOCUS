#ifndef GEOMETRY_UTILS_MATRIXNXN_H
#define GEOMETRY_UTILS_MATRIXNXN_H

#include <ostream>
#include "VectorNBase.h"
#include "GeometryUtilsMath.h"
#include "MatrixNxMBase.h"

namespace geometry_utils {

template <typename T, size_t N>
struct MatrixNxNBase : MatrixNxMBase<T, N, N> {
  MatrixNxNBase() : MatrixNxMBase<T, N, N>() {}
  MatrixNxNBase(T val) : MatrixNxMBase<T, N, N>(val) {}
  MatrixNxNBase(const MatrixNxNBase& in) : MatrixNxMBase<T, N, N>(in.data) {}
  MatrixNxNBase(const boost::array<T, N* N>& in) : MatrixNxMBase<T, N, N>(in) {}
  MatrixNxNBase(T (&in)[N * N]) : MatrixNxMBase<T, N, N>(in) {}
  MatrixNxNBase(const Eigen::Matrix<T, N, N>& in)
      : MatrixNxMBase<T, N, N>(in) {}
  MatrixNxNBase(const MatrixNxMBase<T, N, N>& in)
      : MatrixNxMBase<T, N, N>(in) {}

  inline void Eye() {
    this->data.fill(0);
    for (size_t i = 0; i < this->nrows; i++)
      this->data[this->nrows * i + i] = 1;
  }

  virtual inline T Det() const {
    std::cerr << "MatrixNxMBase::det not implemented" << std::endl;
    return T();
  }

  virtual inline MatrixNxNBase<T, N> Inv() const {
    std::cerr << "MatrixNxMBase::inv not implemented" << std::endl;
    return MatrixNxNBase<T, N>();
  }

  static inline MatrixNxNBase<T, N> Diag(const VectorNBase<T, N>& in) {
    T d[N * N] = {0};
    for (size_t i = 0; i < N; i++)
      d[N * i + i] = in(i);
    return MatrixNxNBase<T, N>(d);
  }
};

template <size_t N>
inline MatrixNxNBase<float, N> operator*(const float& lhs,
                                         const MatrixNxNBase<float, N>& rhs) {
  return MatrixNxNBase<float, N>(rhs * lhs);
}

template <size_t N>
inline MatrixNxNBase<double, N> operator*(const double& lhs,
                                          const MatrixNxNBase<double, N>& rhs) {
  return MatrixNxNBase<double, N>(rhs * lhs);
}

template <typename T, size_t N>
inline MatrixNxNBase<T, N> Inv(const MatrixNxNBase<T, N>& m) {
  return m.Inv();
}

}

#endif
