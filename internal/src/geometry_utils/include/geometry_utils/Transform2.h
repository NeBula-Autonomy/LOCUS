#ifndef GEOMETRY_UTILS_TRANSFORM2_H
#define GEOMETRY_UTILS_TRANSFORM2_H

#include "Vector2.h"
#include "Rotation2.h"

namespace geometry_utils {

template <typename T>
struct Transform2Base {
  typedef boost::shared_ptr<Transform2Base> Ptr;
  typedef boost::shared_ptr<const Transform2Base> ConstPtr;

  Vector2Base<T> translation;
  Rotation2Base<T> rotation;

  Transform2Base() {
    translation.Zeros();
    rotation.Eye();
  }

  Transform2Base(const Vector2Base<T>& translation_,
                 const Rotation2Base<T>& rotation_)
      : translation(translation_), rotation(rotation_) {}

  Transform2Base(const Transform2Base<T>& in)
      : translation(in.translation), rotation(in.rotation) {}

  Transform2Base(T x, T y, T th) : translation(x, y), rotation(th) {}

  Transform2Base& operator=(const Transform2Base& rhs) {
    if (this == &rhs)
      return *this;
    translation = rhs.translation;
    rotation = rhs.rotation;
    return *this;
  }

  Vector2Base<T> operator*(const Vector2Base<T>& p) const {
    return rotation * p + translation;
  }

  Transform2Base<T> operator+(const Transform2Base<T>& t) const {
    return Transform2Base<T>(translation + rotation * t.translation,
                             rotation * t.rotation);
  }

  bool operator==(const Transform2Base& that) const {
    return this->Equals(that);
  }

  bool operator!=(const Transform2Base& that) const {
    return !this->Equals(that);
  }

  bool Equals(const Transform2Base& that, const T ptol = 1e-5,
              const T rtol = 1e-5) const {
    return (translation.Equals(that.translation, ptol) &&
            rotation.Equals(that.rotation, rtol));
  }

  void Print(const std::string& prefix = std::string()) const {
    if (!prefix.empty())
      std::cout << prefix << std::endl;
    std::cout << (*this) << std::endl;
  }

  static Transform2Base Identity() {
    return Transform2Base();
  }
};

template <typename T>
std::ostream& operator<<(std::ostream& out, const Transform2Base<T>& m) {
  out << "Translation:" << std::endl << m.translation << std::endl;
  out << "Rotation:" << std::endl << m.rotation;
  return out;
}

template <typename T>
Transform2Base<T> PoseUpdate(const Transform2Base<T>& t1,
                             const Transform2Base<T>& t2) {
  return Transform2Base<T>(t1.translation + t1.rotation * t2.translation,
                           t1.rotation * t2.rotation);
}

template <typename T>
Transform2Base<T> PoseInverse(const Transform2Base<T>& t) {
  return Transform2Base<T>(-1.0 * t.rotation.Trans() * t.translation,
                           t.rotation.Trans());
}

template <typename T>
Transform2Base<T> PoseDelta(const Transform2Base<T>& t1,
                            const Transform2Base<T>& t2) {
  return Transform2Base<T>(
      t1.rotation.Trans() * (t2.translation - t1.translation),
      t1.rotation.Trans() * t2.rotation);
}

typedef Transform2Base<float> Transform2f;
typedef Transform2Base<double> Transform2d;
typedef Transform2d Transform2;
typedef Transform2 Tr2;

}

#endif
