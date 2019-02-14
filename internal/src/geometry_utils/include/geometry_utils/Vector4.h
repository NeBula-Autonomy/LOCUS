#ifndef GEOMETRY_UTILS_VECTOR4_H
#define GEOMETRY_UTILS_VECTOR4_H

#include "VectorNBase.h"

namespace geometry_utils {

template <typename T>
struct Vector4Base : VectorNBase<T, 4> {
  Vector4Base() : VectorNBase<T, 4>() {}
  Vector4Base(T val) : VectorNBase<T, 4>(val) {}
  Vector4Base(const Vector4Base& in) : VectorNBase<T, 4>(in.data) {}
  Vector4Base(const boost::array<T, 4>& in) : VectorNBase<T, 4>(in) {}
  Vector4Base(T (&in)[4]) : VectorNBase<T, 4>(in) {}
  Vector4Base(const Eigen::Matrix<T, 4, 1>& in) : VectorNBase<T, 4>(in) {}
  Vector4Base(const VectorNBase<T, 4>& in) : VectorNBase<T, 4>(in) {}

  Vector4Base(T v1, T v2, T v3, T v4) {
    this->data[0] = v1;
    this->data[1] = v2;
    this->data[2] = v3;
    this->data[3] = v4;
  }
};

inline Vector4Base<float> operator*(const float& lhs,
                                    const Vector4Base<float>& rhs) {
  return Vector4Base<float>(rhs * lhs);
}

inline Vector4Base<double> operator*(const double& lhs,
                                     const Vector4Base<double>& rhs) {
  return Vector4Base<double>(rhs * lhs);
}

typedef Vector4Base<float> Vector4f;
typedef Vector4Base<float> Vec4f;

typedef Vector4Base<double> Vector4d;
typedef Vector4Base<double> Vec4d;

typedef Vector4Base<double> Vector4;
typedef Vector4Base<double> Vec4;

}

#endif
