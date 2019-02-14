#ifndef GEOMETRY_UTILS_VECTOR2_H
#define GEOMETRY_UTILS_VECTOR2_H

#include "VectorNBase.h"

namespace geometry_utils {

template <typename T>
struct Vector2Base : VectorNBase<T, 2> {
  Vector2Base() : VectorNBase<T, 2>() {}
  Vector2Base(T val) : VectorNBase<T, 2>(val) {}
  Vector2Base(const Vector2Base& in) : VectorNBase<T, 2>(in.data) {}
  Vector2Base(const boost::array<T, 2>& in) : VectorNBase<T, 2>(in) {}
  Vector2Base(T (&in)[2]) : VectorNBase<T, 2>(in) {}
  Vector2Base(const Eigen::Matrix<T, 2, 1>& in) : VectorNBase<T, 2>(in) {}
  Vector2Base(const VectorNBase<T, 2>& in) : VectorNBase<T, 2>(in) {}

  Vector2Base(T v1, T v2) {
    this->data[0] = v1;
    this->data[1] = v2;
  }

  T X() const { return this->data[0]; }
  T Y() const { return this->data[1]; }
};

inline Vector2Base<float> operator*(const float& lhs,
                                    const Vector2Base<float>& rhs) {
  return Vector2Base<float>(rhs * lhs);
}

inline Vector2Base<double> operator*(const double& lhs,
                                     const Vector2Base<double>& rhs) {
  return Vector2Base<double>(rhs * lhs);
}

typedef Vector2Base<float> Vector2f;
typedef Vector2Base<float> Vec2f;

typedef Vector2Base<double> Vector2d;
typedef Vector2Base<double> Vec2d;

typedef Vector2Base<double> Vector2;
typedef Vector2Base<double> Vec2;

}

#endif
