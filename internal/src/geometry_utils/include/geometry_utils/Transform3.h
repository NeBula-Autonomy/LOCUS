#ifndef GEOMETRY_UTILS_TRANSFORM3_H
#define GEOMETRY_UTILS_TRANSFORM3_H

#include "Vector3.h"
#include "Rotation3.h"
#include "Transform2.h"

namespace geometry_utils {

template <typename T>
struct Transform3Base {
  typedef boost::shared_ptr<Transform3Base> Ptr;
  typedef boost::shared_ptr<const Transform3Base> ConstPtr;

  Vector3Base<T> translation;
  Rotation3Base<T> rotation;

  Transform3Base() {
    translation.Zeros();
    rotation.Eye();
  }

  Transform3Base(const Vector3Base<T>& translation_,
                 const Rotation3Base<T>& rotation_)
      : translation(translation_), rotation(rotation_) {}

  Transform3Base(const Transform3Base<T>& in)
      : translation(in.translation), rotation(in.rotation) {}

  Transform3Base(const Transform2Base<T>& in) {
    translation(0) = in.translation(0);
    translation(1) = in.translation(1);
    translation(2) = 0;
    rotation.Eye();
    for (unsigned int i = 0; i < 2; i++)
      for (unsigned int j = 0; j < 2; j++) rotation(i, j) = in.Rotation(i, j);
  }

  Transform3Base& operator=(const Transform3Base& rhs) {
    if (this == &rhs)
      return *this;
    translation = rhs.translation;
    rotation = rhs.rotation;
    return *this;
  }

  Vector3Base<T> operator*(const Vector3Base<T>& p) const {
    return rotation * p + translation;
  }

  Transform3Base<T> operator+(const Transform3Base<T>& t) const {
    return Transform3Base<T>(translation + rotation * t.translation,
                             rotation * t.rotation);
  }

  bool operator==(const Transform3Base& that) const {
    return this->Equals(that);
  }

  bool operator!=(const Transform3Base& that) const {
    return !this->Equals(that);
  }

  bool Equals(const Transform3Base& that, const T ptol = 1e-5,
              const T rtol = 1e-5) const {
    return (translation.Equals(that.translation, ptol) &&
            rotation.Equals(that.rotation, rtol));
  }

  void Print(const std::string& prefix = std::string()) const {
    if (!prefix.empty())
      std::cout << prefix << std::endl;
    std::cout << (*this) << std::endl;
  }

  static Transform3Base Identity() {
    return Transform3Base();
  }
};

template <typename T>
std::ostream& operator<<(std::ostream& out, const Transform3Base<T>& m) {
  out << "Translation:" << std::endl << m.translation << std::endl;
  out << "Rotation:" << std::endl << m.rotation;
  return out;
}

template <typename T>
Transform3Base<T> PoseUpdate(const Transform3Base<T>& t1,
                             const Transform3Base<T>& t2) {
  return Transform3Base<T>(t1.translation + t1.rotation * t2.translation,
                           t1.rotation * t2.rotation);
}

template <typename T>
Transform3Base<T> PoseInverse(const Transform3Base<T>& t) {
  return Transform3Base<T>(-1.0 * t.rotation.Trans() * t.translation,
                           t.rotation.Trans());
}

template <typename T>
Transform3Base<T> PoseDelta(const Transform3Base<T>& t1,
                            const Transform3Base<T>& t2) {
  return Transform3Base<T>(
      t1.rotation.Trans() * (t2.translation - t1.translation),
      t1.rotation.Trans() * t2.rotation);
}

typedef Transform3Base<float> Transform3f;
typedef Transform3Base<double> Transform3d;
typedef Transform3d Transform3;
typedef Transform3 Tr3;

}

#endif
