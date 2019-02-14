#ifndef GEOMETRY_UTILS_MATRIX4X4_H
#define GEOMETRY_UTILS_MATRIX4X4_H

#include "MatrixNxNBase.h"

namespace geometry_utils {

template <typename T>
struct Matrix4x4Base : MatrixNxNBase<T, 4> {
  Matrix4x4Base() : MatrixNxNBase<T, 4>() {}
  Matrix4x4Base(T val) : MatrixNxNBase<T, 4>(val) {}
  Matrix4x4Base(const Matrix4x4Base& in) : MatrixNxNBase<T, 4>(in.data) {}
  Matrix4x4Base(const boost::array<T, 16>& in) : MatrixNxNBase<T, 4>(in) {}
  Matrix4x4Base(T (&in)[16]) : MatrixNxNBase<T, 4>(in) {}
  Matrix4x4Base(const Eigen::Matrix<T, 4, 4>& in) : MatrixNxNBase<T, 4>(in) {}
  Matrix4x4Base(const MatrixNxNBase<T, 4>& in) : MatrixNxNBase<T, 4>(in) {}
  Matrix4x4Base(const MatrixNxMBase<T, 4, 4>& in) : MatrixNxNBase<T, 4>(in) {}

  Matrix4x4Base(T R11, T R12, T R13, T R14, T R21, T R22, T R23, T R24, T R31,
                T R32, T R33, T R34, T R41, T R42, T R43, T R44) {
    this->data[0] = R11;
    this->data[1] = R12;
    this->data[2] = R13;
    this->data[3] = R14;
    this->data[4] = R21;
    this->data[5] = R22;
    this->data[6] = R23;
    this->data[7] = R24;
    this->data[8] = R31;
    this->data[9] = R32;
    this->data[10] = R33;
    this->data[11] = R34;
    this->data[12] = R41;
    this->data[13] = R42;
    this->data[14] = R43;
    this->data[15] = R44;
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
    T j = this->data[9];
    T k = this->data[10];
    T l = this->data[11];
    T m = this->data[12];
    T n = this->data[13];
    T o = this->data[14];
    T p = this->data[15];

    return d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
           c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
           d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
           d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
           b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
           c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p;
  }

  virtual inline MatrixNxNBase<T, 4> Inv() const {
    if (math::fabs(Det()) < std::numeric_limits<T>::denorm_min())
      throw std::runtime_error("Matrix4x4Base: appears singular");

    T a = this->data[0];
    T b = this->data[1];
    T c = this->data[2];
    T d = this->data[3];
    T e = this->data[4];
    T f = this->data[5];
    T g = this->data[6];
    T h = this->data[7];
    T i = this->data[8];
    T j = this->data[9];
    T k = this->data[10];
    T l = this->data[11];
    T m = this->data[12];
    T n = this->data[13];
    T o = this->data[14];
    T p = this->data[15];

    T tmp[16] = {
        (-(h * k * n) + g * l * n + h * j * o - f * l * o - g * j * p +
         f * k * p) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p),
        (d * k * n - c * l * n - d * j * o + b * l * o + c * j * p -
         b * k * p) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p),
        (-(d * g * n) + c * h * n + d * f * o - b * h * o - c * f * p +
         b * g * p) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p),
        (d * g * j - c * h * j - d * f * k + b * h * k + c * f * l -
         b * g * l) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p),
        (h * k * m - g * l * m - h * i * o + e * l * o + g * i * p -
         e * k * p) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p),
        (-(d * k * m) + c * l * m + d * i * o - a * l * o - c * i * p +
         a * k * p) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p),
        (d * g * m - c * h * m - d * e * o + a * h * o + c * e * p -
         a * g * p) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p),
        (-(d * g * i) + c * h * i + d * e * k - a * h * k - c * e * l +
         a * g * l) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p),
        (-(h * j * m) + f * l * m + h * i * n - e * l * n - f * i * p +
         e * j * p) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p),
        (d * j * m - b * l * m - d * i * n + a * l * n + b * i * p -
         a * j * p) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p),
        (-(d * f * m) + b * h * m + d * e * n - a * h * n - b * e * p +
         a * f * p) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p),
        (d * f * i - b * h * i - d * e * j + a * h * j + b * e * l -
         a * f * l) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p),
        (g * j * m - f * k * m - g * i * n + e * k * n + f * i * o -
         e * j * o) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p),
        (-(c * j * m) + b * k * m + c * i * n - a * k * n - b * i * o +
         a * j * o) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p),
        (c * f * m - b * g * m - c * e * n + a * g * n + b * e * o -
         a * f * o) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p),
        (-(c * f * i) + b * g * i + c * e * j - a * g * j - b * e * k +
         a * f * k) /
            (d * g * j * m - c * h * j * m - d * f * k * m + b * h * k * m +
             c * f * l * m - b * g * l * m - d * g * i * n + c * h * i * n +
             d * e * k * n - a * h * k * n - c * e * l * n + a * g * l * n +
             d * f * i * o - b * h * i * o - d * e * j * o + a * h * j * o +
             b * e * l * o - a * f * l * o - c * f * i * p + b * g * i * p +
             c * e * j * p - a * g * j * p - b * e * k * p + a * f * k * p)};
    return MatrixNxNBase<T, 4>(tmp);
  }
};

inline Matrix4x4Base<float> operator*(const float& lhs,
                                      const Matrix4x4Base<float>& rhs) {
  return Matrix4x4Base<float>(rhs.Scale(lhs));
}

inline Matrix4x4Base<double> operator*(const double& lhs,
                                       const Matrix4x4Base<double>& rhs) {
  return Matrix4x4Base<double>(rhs.Scale(lhs));
}

typedef Matrix4x4Base<float> Matrix4x4f;
typedef Matrix4x4Base<float> Mat44f;

typedef Matrix4x4Base<double> Matrix4x4d;
typedef Matrix4x4Base<double> Mat44d;

typedef Matrix4x4Base<double> Matrix4x4;
typedef Matrix4x4Base<double> Mat44;

}

#endif
