#ifndef GEOMETRY_UTILS_VECTORN_H
#define GEOMETRY_UTILS_VECTORN_H

#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <Eigen/Core>
#include "GeometryUtilsMath.h"

namespace geometry_utils {

template <typename T, size_t N>
struct VectorNBase {
  typedef typename boost::shared_ptr<VectorNBase<T, N> > Ptr;
  typedef typename boost::shared_ptr<const VectorNBase<T, N> > ConstPtr;

  static const size_t length = N;

  boost::array<T, N> data;

  VectorNBase() { data.fill(0); }

  VectorNBase(T val) { data.fill(val); }

  VectorNBase(const VectorNBase& in) : data(in.data) {}

  VectorNBase(const boost::array<T, N>& in) : data(in) {}

  VectorNBase(T (&in)[N]) {
    for (size_t i = 0; i < N; i++)
      data[i] = in[i];
  }

  VectorNBase(const Eigen::Matrix<T, N, 1>& in) {
    for (size_t i = 0; i < N; i++)
      data[i] = in(i, 0);
  }

  inline T& operator()(unsigned int i) {
    return data[i];
  }

  inline const T& operator()(unsigned int i) const {
    return data[i];
  }

  inline T& Get(unsigned int i) {
    return data[i];
  }

  inline const T& Get(unsigned int i) const {
    return data[i];
  }

  inline VectorNBase& operator=(const VectorNBase& rhs) {
    if (this == &rhs)
      return *this;
    data = rhs.data;
    return *this;
  }

  inline VectorNBase operator*(T rhs) const {
    T d[N];
    for (size_t i = 0; i < N; i++)
      d[i] = data[i] * rhs;
    return VectorNBase<T, N>(d);
  }

  inline VectorNBase operator+(const VectorNBase& rhs) const {
    T d[N];
    for (size_t i = 0; i < N; i++)
      d[i] = data[i] + rhs.data[i];
    return VectorNBase<T, N>(d);
  }

  inline VectorNBase operator-() const {
    T d[N];
    for (size_t i = 0; i < N; i++)
      d[i] = -data[i];
    return VectorNBase<T, N>(d);
  }

  inline VectorNBase operator-(const VectorNBase& rhs) const {
    T d[N];
    for (size_t i = 0; i < N; i++)
      d[i] = data[i] - rhs.data[i];
    return VectorNBase<T, N>(d);
  }

  inline VectorNBase operator%(const VectorNBase& rhs) const {
    T d[N];
    for (size_t i = 0; i < N; i++)
      d[i] = data[i] * rhs.data[i];
    return VectorNBase<T, N>(d);
  }

  inline T operator^(const VectorNBase& rhs) const {
    T dot = 0;
    for (size_t i = 0; i < N; i++)
      dot += data[i] * rhs.data[i];
    return dot;
  }

  inline VectorNBase operator/(const VectorNBase& rhs) const {
    T d[N];
    for (size_t i = 0; i < N; i++)
      d[i] = data[i] / rhs.data[i];
    return VectorNBase<T, N>(d);
  }

  inline VectorNBase operator+=(const VectorNBase& rhs) {
    for (size_t i = 0; i < N; i++)
      data[i] += rhs.data[i];
    return *this;
  }

  inline VectorNBase operator-=(const VectorNBase& rhs) {
    for (size_t i = 0; i < N; i++)
      data[i] -= rhs.data[i];
    return *this;
  }

  inline VectorNBase operator%=(const VectorNBase& rhs) {
    for (size_t i = 0; i < N; i++)
      data[i] *= rhs.data[i];
    return *this;
  }

  inline VectorNBase operator/=(const VectorNBase& rhs) {
    for (size_t i = 0; i < N; i++)
      data[i] /= rhs.data[i];
    return *this;
  }

  inline VectorNBase operator*=(const T& rhs) {
    for (size_t i = 0; i < N; i++)
      data[i] *= rhs;
    return *this;
  }

  inline VectorNBase operator/=(const T& rhs) {
    for (size_t i = 0; i < N; i++)
      data[i] /= rhs;
    return *this;
  }

  inline bool operator==(const VectorNBase& that) const {
    return this->Equals(that);
  }

  inline bool operator!=(const VectorNBase& that) const {
    return !this->Equals(that);
  }

  inline bool Equals(const VectorNBase& that, const T ptol = 1e-8) const {
    return (*this - that).Norm() < ptol;
  }

  inline T Norm() const { return math::sqrt((*this) ^ (*this)); }

  inline VectorNBase Normalize() const { return (*this) / Norm(); }

  inline VectorNBase Abs() const {
    T d[N];
    for (size_t i = 0; i < N; i++)
      d[i] = std::abs(data[i]);
    return VectorNBase<T, N>(d);
  }

  inline void Ones() {
    data.fill(1);
  }

  inline void Zeros() {
    data.fill(0);
  }

  inline T Dot(const VectorNBase& v) const {
    return (*this) ^ v;
  }

  inline VectorNBase Scale(T s) const {
    return (*this) * s;
  }

  inline void Print(const std::string& prefix = std::string()) const {
    if (!prefix.empty())
      std::cout << prefix << std::endl;
    std::cout << (*this) << std::endl;
  }

  inline Eigen::Matrix<T, N, 1> Eigen() const {
    return Eigen::Matrix<T, N, 1>(data.data());
  }
};

template <typename T, size_t N>
inline VectorNBase<T, N> operator*(const T& lhs, const VectorNBase<T, N>& rhs) {
  return rhs * lhs;
}

template <typename T, size_t N>
inline std::ostream& operator<<(std::ostream& out, const VectorNBase<T, N>& m) {
  for (size_t i = 0; i < N - 1; i++)
    out << m.data[i] << " ";
  out << m.data[N - 1];
  return out;
}

template <typename T, size_t N>
inline Eigen::Matrix<T, N, 1> Eigen(const VectorNBase<T, N>& in) {
  return in.Eigen();
}

template <typename T, size_t N>
inline T Norm(const VectorNBase<T, N>& v) {
  return v.Norm();
}

template <typename T, size_t N>
inline T Dot(const VectorNBase<T, N>& v1, const VectorNBase<T, N>& v2) {
  return v1.Dot(v2);
}

}

#endif
