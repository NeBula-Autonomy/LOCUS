#ifndef GEOMETRY_UTILS_MATRIX3X3_H
#define GEOMETRY_UTILS_MATRIX3X3_H

#include "MatrixNxNBase.h"

namespace geometry_utils {

template <typename T>
struct Matrix3x3Base : MatrixNxNBase<T, 3> {
  Matrix3x3Base() : MatrixNxNBase<T, 3>() {}
  Matrix3x3Base(T val) : MatrixNxNBase<T, 3>(val) {}
  Matrix3x3Base(const Matrix3x3Base& in) : MatrixNxNBase<T, 3>(in.data) {}
  Matrix3x3Base(const boost::array<T, 9>& in) : MatrixNxNBase<T, 3>(in) {}
  Matrix3x3Base(T (&in)[9]) : MatrixNxNBase<T, 3>(in) {}
  Matrix3x3Base(const Eigen::Matrix<T, 3, 3>& in) : MatrixNxNBase<T, 3>(in) {}
  Matrix3x3Base(const MatrixNxNBase<T, 3>& in) : MatrixNxNBase<T, 3>(in) {}
  Matrix3x3Base(const MatrixNxMBase<T, 3, 3>& in) : MatrixNxNBase<T, 3>(in) {}

  Matrix3x3Base(T R11, T R12, T R13, T R21, T R22, T R23, T R31, T R32, T R33) {
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

  inline T Det() const {
    T a = this->data[0];
    T b = this->data[1];
    T c = this->data[2];
    T d = this->data[3];
    T e = this->data[4];
    T f = this->data[5];
    T g = this->data[6];
    T h = this->data[7];
    T i = this->data[8];
    return (-(c * e * g) + b * f * g + c * d * h - a * f * h - b * d * i +
            a * e * i);
  }

  virtual inline MatrixNxNBase<T, 3> Inv() const {
    if (math::fabs(Det()) < std::numeric_limits<T>::denorm_min())
      throw std::runtime_error("Matrix3x3Base: appears singular");

    T a = this->data[0];
    T b = this->data[1];
    T c = this->data[2];
    T d = this->data[3];
    T e = this->data[4];
    T f = this->data[5];
    T g = this->data[6];
    T h = this->data[7];
    T i = this->data[8];
    T tmp[9] = {(f * h - e * i) / (c * e * g - b * f * g - c * d * h +
                                   a * f * h + b * d * i - a * e * i),
                (c * h - b * i) / (-(c * e * g) + b * f * g + c * d * h -
                                   a * f * h - b * d * i + a * e * i),
                (c * e - b * f) / (c * e * g - b * f * g - c * d * h +
                                   a * f * h + b * d * i - a * e * i),
                (f * g - d * i) / (-(c * e * g) + b * f * g + c * d * h -
                                   a * f * h - b * d * i + a * e * i),
                (c * g - a * i) / (c * e * g - b * f * g - c * d * h +
                                   a * f * h + b * d * i - a * e * i),
                (c * d - a * f) / (-(c * e * g) + b * f * g + c * d * h -
                                   a * f * h - b * d * i + a * e * i),
                (e * g - d * h) / (c * e * g - b * f * g - c * d * h +
                                   a * f * h + b * d * i - a * e * i),
                (b * g - a * h) / (-(c * e * g) + b * f * g + c * d * h -
                                   a * f * h - b * d * i + a * e * i),
                (b * d - a * e) / (c * e * g - b * f * g - c * d * h +
                                   a * f * h + b * d * i - a * e * i)};
    return MatrixNxNBase<T, 3>(tmp);
  }
};

inline Matrix3x3Base<float> operator*(const float& lhs,
                                      const Matrix3x3Base<float>& rhs) {
  return Matrix3x3Base<float>(rhs.Scale(lhs));
}

inline Matrix3x3Base<double> operator*(const double& lhs,
                                       const Matrix3x3Base<double>& rhs) {
  return Matrix3x3Base<double>(rhs.Scale(lhs));
}

typedef Matrix3x3Base<float> Matrix3x3f;
typedef Matrix3x3Base<float> Mat33f;

typedef Matrix3x3Base<double> Matrix3x3d;
typedef Matrix3x3Base<double> Mat33d;

typedef Matrix3x3Base<double> Matrix3x3;
typedef Matrix3x3Base<double> Mat33;

}

#endif
