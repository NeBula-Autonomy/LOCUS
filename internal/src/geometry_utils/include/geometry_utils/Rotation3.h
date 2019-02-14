#ifndef GEOMETRY_UTILS_ROTATION3_H
#define GEOMETRY_UTILS_ROTATION3_H

#include <Eigen/Geometry>
#include "GeometryUtilsMath.h"
#include "RotationNBase.h"
#include "Vector3.h"
#include "Quaternion.h"
#include "Rotation2.h"
#include "Matrix2x2.h"

namespace geometry_utils {

template <typename T>
struct Rotation3Base : RotationNBase<T, 3> {
  Rotation3Base() : RotationNBase<T, 3>() {}
  Rotation3Base(const Rotation3Base& in) : RotationNBase<T, 3>(in.data) {}
  Rotation3Base(const boost::array<T, 9>& in) : RotationNBase<T, 3>(in) {}
  Rotation3Base(T (&in)[9]) : RotationNBase<T, 3>(in) {}
  Rotation3Base(const Eigen::Matrix<T, 3, 3>& in) : RotationNBase<T, 3>(in) {}
  Rotation3Base(const Eigen::AngleAxis<T>& in)
      : RotationNBase<T, 3>(in.toRotationMatrix()) {}
  Rotation3Base(const RotationNBase<T, 3>& in) : RotationNBase<T, 3>(in) {}
  Rotation3Base(const Matrix2x2Base<T>& in) : RotationNBase<T, 3>(in) {}
  Rotation3Base(const MatrixNxMBase<T, 3, 3>& in) : RotationNBase<T, 3>(in) {}

  Rotation3Base(T R11, T R12, T R13, T R21, T R22, T R23, T R31, T R32, T R33) {
    this->data[0] = R11;
    this->data[1] = R12;
    this->data[2] = R13;
    this->data[3] = R21;
    this->data[4] = R22;
    this->data[5] = R23;
    this->data[6] = R31;
    this->data[7] = R32;
    this->data[8] = R33;
  }

  Rotation3Base(const Vector3Base<T>& in) {
    FromEulerZYX(in(0), in(1), in(2));
  }

  Rotation3Base(T euler_x, T euler_y, T euler_z) {
    FromEulerZYX(euler_x, euler_y, euler_z);
  }

  Rotation3Base(const Rotation2Base<T>& rot) {
    this->Zeros();
    this->data[0] = rot(0);
    this->data[1] = rot(1);
    this->data[3] = rot(2);
    this->data[4] = rot(3);
    this->data[8] = static_cast<T>(1);
  }

  Rotation3Base(const QuaternionBase<T>& quat) {
    QuaternionBase<T> q(quat);
    if (math::fabs(1 - q.Norm()) > static_cast<T>(1e-6))
      q = q.Normalize();

    T a = q.W();
    T b = q.X();
    T c = q.Y();
    T d = q.Z();

    T a2 = a * a;
    T b2 = b * b;
    T c2 = c * c;
    T d2 = d * d;

    T ab = a * b;
    T ac = a * c;
    T ad = a * d;

    T bc = b * c;
    T bd = b * d;

    T cd = c * d;

    (*this)(0, 0) = a2 + b2 - c2 - d2;
    (*this)(0, 1) = static_cast<T>(2) * (bc - ad);
    (*this)(0, 2) = static_cast<T>(2) * (bd + ac);
    (*this)(1, 0) = static_cast<T>(2) * (bc + ad);
    (*this)(1, 1) = a2 - b2 + c2 - d2;
    (*this)(1, 2) = static_cast<T>(2) * (cd - ab);
    (*this)(2, 0) = static_cast<T>(2) * (bd - ac);
    (*this)(2, 1) = static_cast<T>(2) * (cd + ab);
    (*this)(2, 2) = a2 - b2 - c2 + d2;
  }

  virtual inline bool Equals(const Rotation3Base& that,
                             const T ptol = 1e-8) const {
    return Error(that) < ptol;
  }

  inline T Error(const Rotation3Base& r) const {
    return Norm(
        static_cast<T>(0.5) *
        Rotation3Base<T>(this->Trans() * r - r.Trans() * (*this)).Vee());
  }

  inline Vector3Base<T> Vee() const {
    return Vector3Base<T>((*this)(2, 1), (*this)(0, 2), (*this)(1, 0));
  }

  inline void FromEulerZYX(T euler_x, T euler_y, T euler_z) {
    T cph = math::cos(euler_x);
    T sph = math::sin(euler_x);

    T ct = math::cos(euler_y);
    T st = math::sin(euler_y);

    T cps = math::cos(euler_z);
    T sps = math::sin(euler_z);

    (*this)(0, 0) = ct * cps;
    (*this)(0, 1) = cps * st * sph - cph * sps;
    (*this)(0, 2) = cph * cps * st + sph * sps;

    (*this)(1, 0) = ct * sps;
    (*this)(1, 1) = cph * cps + st * sph * sps;
    (*this)(1, 2) = -cps * sph + cph * st * sps;

    (*this)(2, 0) = -st;
    (*this)(2, 1) = ct * sph;
    (*this)(2, 2) = ct * cph;
  }

  inline Vector3Base<T> GetEulerZYX() const {
    return ToEulerZYX();
  }

  inline Vector3Base<T> ToEulerZYX() const {
    T theta = -math::asin((*this)(2, 0));
    T phi = 0;
    T psi = 0;
    if (math::fabs(cos(theta)) > static_cast<T>(1e-6)) {
      phi = math::atan2((*this)(2, 1), (*this)(2, 2));
      psi = math::atan2((*this)(1, 0), (*this)(0, 0));
    }

    return Vector3Base<T>(phi, theta, psi);
  }

  inline T Roll() const {
    T theta = -math::asin((*this)(2, 0));
    return math::fabs(cos(theta)) > static_cast<T>(1e-6)
               ? math::atan2((*this)(2, 1), (*this)(2, 2))
               : 0;
  }

  inline T Pitch() const {
    return -math::asin((*this)(2, 0));
  }

  inline T Yaw() const {
    T theta = -math::asin((*this)(2, 0));
    return math::fabs(cos(theta)) > static_cast<T>(1e-6)
               ? math::atan2((*this)(1, 0), (*this)(0, 0))
               : 0;
  }
};

inline Rotation3Base<float> operator*(const float& lhs,
                                      const Rotation3Base<float>& rhs) {
  return Rotation3Base<float>(rhs.Scale(lhs));
}

inline Rotation3Base<double> operator*(const double& lhs,
                                       const Rotation3Base<double>& rhs) {
  return Rotation3Base<double>(rhs.Scale(lhs));
}

template <typename T>
inline Rotation3Base<T> Hat(const Vector3Base<T>& v) {
  return Rotation3Base<T>(0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0);
}

template <typename T>
inline Vector3Base<T> Vee(const MatrixNxMBase<T, 3, 3>& m) {
  return Rotation3Base<T>(m).Vee();
}

typedef Rotation3Base<float> Rotation3f;
typedef Rotation3Base<float> Rot3f;

typedef Rotation3Base<double> Rotation3d;
typedef Rotation3Base<double> Rot3d;

typedef Rotation3Base<double> Rotation3;
typedef Rotation3Base<double> Rot3;

}

#endif
