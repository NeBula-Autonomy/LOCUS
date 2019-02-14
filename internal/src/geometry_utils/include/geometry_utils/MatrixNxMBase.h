#ifndef GEOMETRY_UTILS_MATRIXNXM_H
#define GEOMETRY_UTILS_MATRIXNXM_H

#include <ostream>
#include "VectorNBase.h"
#include "GeometryUtilsMath.h"

namespace geometry_utils {

template <typename T, size_t N, size_t M>
struct MatrixNxMBase {
  typedef typename boost::shared_ptr<MatrixNxMBase<T, N, M> > Ptr;
  typedef typename boost::shared_ptr<const MatrixNxMBase<T, N, M> > ConstPtr;

  static const size_t size = N * M;
  static const size_t nrows = N;
  static const size_t ncols = M;

  boost::array<T, size> data;

  MatrixNxMBase() { data.fill(0); }

  MatrixNxMBase(T val) { data.fill(val); }

  MatrixNxMBase(const MatrixNxMBase& in) : data(in.data) {}

  MatrixNxMBase(const boost::array<T, size>& in) : data(in) {}

  MatrixNxMBase(T (&in)[size]) {
    for (unsigned int i = 0; i < size; i++)
      data[i] = in[i];
  }

  MatrixNxMBase(const Eigen::Matrix<T, N, M>& in) {
    for (size_t i = 0; i < nrows; i++)
      for (size_t j = 0; j < ncols; j++)
        data[ncols * i + j] = in(i, j);
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

  inline T& operator()(unsigned int i, unsigned int j) {
    return data[ncols * i + j];
  }

  inline const T& operator()(unsigned int i, unsigned int j) const {
    return data[ncols * i + j];
  }

  inline T& Get(unsigned int i, unsigned int j) {
    return data[ncols * i + j];
  }

  inline const T& Get(unsigned int i, unsigned int j) const {
    return data[ncols * i + j];
  }

  inline std::ostream& operator<<(std::ostream& out) {
    out << data;
    return out;
  }

  inline MatrixNxMBase& operator=(const MatrixNxMBase& rhs) {
    if (this == &rhs) return *this;
    data = rhs.data;
    return *this;
  }

  inline MatrixNxMBase<T, N, M> operator*(T rhs) const {
    T d[size];
    for (size_t i = 0; i < size; i++)
      d[i] = data[i] * rhs;
    return MatrixNxMBase<T, N, M>(d);
  }

  inline MatrixNxMBase<T, N, M> operator+(const MatrixNxMBase& rhs) const {
    T d[size];
    for (size_t i = 0; i < size; i++)
      d[i] = data[i] + rhs(i);
    return MatrixNxMBase<T, N, M>(d);
  }

  inline MatrixNxMBase<T, N, M> operator-() const {
    T d[size];
    for (size_t i = 0; i < size; i++)
      d[i] = -data[i];
    return MatrixNxMBase<T, N, M>(d);
  }

  inline MatrixNxMBase<T, N, M> operator-(const MatrixNxMBase& rhs) const {
    T d[size];
    for (size_t i = 0; i < size; i++)
      d[i] = data[i] - rhs(i);
    return MatrixNxMBase<T, N, M>(d);
  }

  inline MatrixNxMBase<T, N, N> operator*(const MatrixNxMBase<T, M, N>& rhs)
      const {
    T d[N * N];
    for (size_t i = 0; i < N; i++)
      for (size_t j = 0; j < N; j++)
        d[N * i + j] = this->Row(i) ^ rhs.Col(j);
    return MatrixNxMBase<T, N, N>(d);
  }

  inline VectorNBase<T, N> operator*(const VectorNBase<T, M>& rhs) const {
    T d[nrows];
    for (size_t i = 0; i < nrows; i++)
      d[i] = this->Row(i) ^ rhs;
    return VectorNBase<T, N>(d);
  }

  inline MatrixNxMBase<T, N, M> operator%(const MatrixNxMBase& rhs) const {
    T d[size];
    for (size_t i = 0; i < size; i++)
      d[i] = data[i] * rhs(i);
    return MatrixNxMBase<T, N, M>(d);
  }

  inline MatrixNxMBase<T, N, M> operator/(const MatrixNxMBase& rhs) const {
    T d[size];
    for (size_t i = 0; i < size; i++)
      d[i] = data[i] / rhs(i);
    return MatrixNxMBase<T, N, M>(d);
  }

  inline MatrixNxMBase<T, N, M> operator+=(const MatrixNxMBase& rhs) {
    for (size_t i = 0; i < size; i++)
      data[i] += rhs.data[i];
    return *this;
  }

  inline MatrixNxMBase<T, N, M> operator-=(const MatrixNxMBase& rhs) {
    for (size_t i = 0; i < size; i++)
      data[i] -= rhs.data[i];
    return *this;
  }

  inline MatrixNxMBase<T, N, M> operator%=(const MatrixNxMBase& rhs) {
    for (size_t i = 0; i < size; i++)
      data[i] *= rhs.data[i];
    return *this;
  }

  inline MatrixNxMBase<T, N, M> operator/=(const MatrixNxMBase& rhs) {
    for (size_t i = 0; i < size; i++)
      data[i] /= rhs.data[i];
    return *this;
  }

  inline MatrixNxMBase<T, M, N> Trans() const {
    T d[size];
    for (size_t i = 0; i < nrows; i++)
      for (size_t j = 0; j < ncols; j++)
        d[nrows * j + i] = data[ncols * i + j];
    return MatrixNxMBase<T, M, N>(d);
  }

  inline MatrixNxMBase<T, M, N> t() const {
    return this->Trans();
  }

  inline VectorNBase<T, M> Row(unsigned int r) const {
    T d[ncols];
    for (size_t i = 0; i < ncols; i++)
      d[i] = data[ncols * r + i];
    return VectorNBase<T, M>(d);
  }

  inline VectorNBase<T, N> Col(unsigned int c) const {
    T d[nrows];
    for (size_t i = 0; i < nrows; i++)
      d[i] = data[ncols * i + c];
    return VectorNBase<T, N>(d);
  }

  inline void Ones() { data.fill(1); }

  inline void Zeros() { data.fill(0); }

  inline MatrixNxMBase<T, N, M> Scale(T s) const {
    return (*this) * s;
  }

  inline void Print(const std::string& prefix = std::string()) const {
    if (!prefix.empty())
      std::cout << prefix << std::endl;
    std::cout << (*this) << std::endl;
  }

  inline Eigen::Matrix<T, N, M> Eigen() const {
    return Eigen::Matrix<T, M, N>(data.data()).transpose();
  }

  inline bool operator==(const MatrixNxMBase& that) const {
    return this->Equals(that);
  }

  inline bool operator!=(const MatrixNxMBase& that) const {
    return !this->Equals(that);
  }

  virtual inline bool Equals(const MatrixNxMBase& that,
                             const T ptol = 1e-8) const {
    return (*this - that).Norm() < ptol;
  }

  inline T Norm() const {
    return math::sqrt((this->Trans() * (*this)).Trace());
  }

  inline T Trace() {
    size_t count = nrows <= ncols ? nrows : ncols;
    T tr = 0;
    for (size_t i = 0; i < count; i++)
      tr += data[ncols * i + i];
    return tr;
  }

};

template <size_t N, size_t M>
inline MatrixNxMBase<float, N, M> operator*(
    const float& lhs, const MatrixNxMBase<float, N, M>& rhs) {
  return rhs * lhs;
}

template <size_t N, size_t M>
inline MatrixNxMBase<double, N, M> operator*(
    const double& lhs, const MatrixNxMBase<double, N, M>& rhs) {
  return rhs * lhs;
}

template <typename T, size_t N, size_t M>
inline std::ostream& operator<<(std::ostream& out,
                                const MatrixNxMBase<T, N, M>& m) {
  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < M; j++)
      out << m.data[M * i + j] << " ";
    out << std::endl;
  }
  return out;
}

template <typename T, size_t N, size_t M>
inline Eigen::Matrix<T, N, M> Eigen(const MatrixNxMBase<T, N, M>& in) {
  return in.Eigen();
}

template <typename T, size_t N, size_t M>
inline MatrixNxMBase<T, M, N> Trans(const MatrixNxMBase<T, N, M>& m) {
  return m.Trans();
}

template <typename T, size_t N, size_t M>
inline MatrixNxMBase<T, N, M> Outer(const VectorNBase<T, N>& v1,
                                    const VectorNBase<T, M>& v2) {
  T d[N * M];
  for (size_t i = 0; i < N; i++)
    for (size_t j = 0; j < M; j++)
      d[M * i + j] = v1(i) * v2(j);
  return MatrixNxMBase<T, N, M>(d);
}

}

#endif
