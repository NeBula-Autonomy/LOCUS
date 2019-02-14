#ifndef GEOMETRY_UTILS_ROTATION2_H
#define GEOMETRY_UTILS_ROTATION2_H

#include <Eigen/Geometry>
#include "GeometryUtilsMath.h"
#include "RotationNBase.h"
#include "Matrix2x2.h"

namespace geometry_utils {

template <typename T>
struct Rotation2Base : RotationNBase<T, 2> {
  Rotation2Base() : RotationNBase<T, 2>() {}
  Rotation2Base(const Rotation2Base& in) : RotationNBase<T, 2>(in.data) {}
  Rotation2Base(const boost::array<T, 4>& in) : RotationNBase<T, 2>(in) {}
  Rotation2Base(T (&in)[2 * 2]) : RotationNBase<T, 2>(in) {}
  Rotation2Base(const Eigen::Matrix<T, 2, 2>& in) : RotationNBase<T, 2>(in) {}
  Rotation2Base(const Eigen::Rotation2D<T>& in)
      : RotationNBase<T, 2>(in.toRotationMatrix()) {}
  Rotation2Base(const RotationNBase<T, 2>& in) : RotationNBase<T, 2>(in) {}
  Rotation2Base(const Matrix2x2Base<T>& in) : RotationNBase<T, 2>(in) {}
  Rotation2Base(const MatrixNxMBase<T, 2, 2>& in) : RotationNBase<T, 2>(in) {}

  Rotation2Base(T val) { FromAngle(val); }

  Rotation2Base(T R11, T R12, T R21, T R22) {
    this->data[0] = R11;
    this->data[1] = R12;
    this->data[2] = R21;
    this->data[3] = R22;
  }

  virtual inline bool Equals(const Rotation2Base& that,
                             const T ptol = 1e-8) const {
    return Error(that) < ptol;
  }

  inline T Error(const Rotation2Base& r) const {
    return math::sin(Angle() - r.Angle());
  }

  inline T Angle() const {
    return math::atan2(this->data[2], this->data[0]);
  }

  inline void FromAngle(T val) {
    this->data[0] = math::cos(val);
    this->data[1] = -math::sin(val);
    this->data[2] = math::sin(val);
    this->data[3] = math::cos(val);
  }

  inline Eigen::Rotation2D<T> Eigen() {
    return Eigen::Rotation2D<T>(Angle());
  }
};

inline Rotation2Base<float> operator*(const float& lhs,
                                      const Rotation2Base<float>& rhs) {
  return Rotation2Base<float>(rhs.Scale(lhs));
}

inline Rotation2Base<double> operator*(const double& lhs,
                                       const Rotation2Base<double>& rhs) {
  return Rotation2Base<double>(rhs.Scale(lhs));
}

template <typename T>
inline Eigen::Rotation2D<T> Eigen(const Rotation2Base<T>& in) {
  return in.Eigen();
}

typedef Rotation2Base<float> Rotation2f;
typedef Rotation2Base<float> Rot2f;

typedef Rotation2Base<double> Rotation2d;
typedef Rotation2Base<double> Rot2d;

typedef Rotation2Base<double> Rotation2;
typedef Rotation2Base<double> Rot2;

}

#endif
