#ifndef GEOMETRY_UTILS_VECTOR3_H
#define GEOMETRY_UTILS_VECTOR3_H

#include "VectorNBase.h"

namespace geometry_utils {

template <typename T>
struct Vector3Base : VectorNBase<T, 3> {
  Vector3Base() : VectorNBase<T, 3>() {}
  Vector3Base(T val) : VectorNBase<T, 3>(val) {}
  Vector3Base(const Vector3Base& in) : VectorNBase<T, 3>(in.data) {}
  Vector3Base(const boost::array<T, 3>& in) : VectorNBase<T, 3>(in) {}
  Vector3Base(T (&in)[3]) : VectorNBase<T, 3>(in) {}
  Vector3Base(const Eigen::Matrix<T, 3, 1>& in) : VectorNBase<T, 3>(in) {}
  Vector3Base(const VectorNBase<T, 3>& in) : VectorNBase<T, 3>(in) {}

  Vector3Base(T v1, T v2, T v3) {
    this->data[0] = v1;
    this->data[1] = v2;
    this->data[2] = v3;
  }

  T X() const { return this->data[0]; }
  T Y() const { return this->data[1]; }
  T Z() const { return this->data[2]; }

  inline Vector3Base<T> Cross(const Vector3Base<T>& v) const {
    return Vector3Base<T>(-(*this)(2) * v(1) + (*this)(1) * v(2),
                          (*this)(2) * v(0) - (*this)(0) * v(2),
                          -(*this)(1) * v(0) + (*this)(0) * v(1));
  }
};

inline Vector3Base<float> operator*(const float& lhs,
                                    const Vector3Base<float>& rhs) {
  return Vector3Base<float>(rhs * lhs);
}

inline Vector3Base<double> operator*(const double& lhs,
                                     const Vector3Base<double>& rhs) {
  return Vector3Base<double>(rhs * lhs);
}

template <typename T>
inline VectorNBase<T, 3> Cross(const VectorNBase<T, 3>& v1,
                               const VectorNBase<T, 3>& v2) {
  return Vector3Base<T>(v1).Cross(v2);
}

typedef Vector3Base<float> Vector3f;
typedef Vector3Base<float> Vec3f;

typedef Vector3Base<double> Vector3d;
typedef Vector3Base<double> Vec3d;

typedef Vector3Base<double> Vector3;
typedef Vector3Base<double> Vec3;

}

#endif
