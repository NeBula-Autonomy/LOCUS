/*
  geometry_utils: Utility library to provide common geometry types and transformations
  Copyright (C) 2014  Nathan Michael
                2016  Erik Nelson

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include "Vector2.h"
#include "Vector3.h"
#include "Vector4.h"
#include "Quaternion.h"
#include "Matrix2x2.h"
#include "Matrix3x3.h"
#include "Matrix4x4.h"
#include "Rotation2.h"
#include "Rotation3.h"
#include "Transform2.h"
#include "Transform3.h"

namespace geometry_utils {
inline double Unroll(double x) {
  x = fmod(x, 2.0 * M_PI);
  if (x < 0)
    x += 2.0 * M_PI;
  return x;
}

inline double Normalize(double x) {
  x = fmod(x + M_PI, 2.0 * M_PI);
  if (x < 0)
    x += 2.0 * M_PI;
  return x - M_PI;
}

inline double S1Distance(double from, double to) {
  double result = Unroll(Unroll(to) - Unroll(from));
  if (result > M_PI)
    result = -(2.0 * M_PI - result);
  return Normalize(result);
}

inline double Rad2Deg(double angle) {
  return angle * 180.0 * M_1_PI;
}

inline double Deg2Rad(double angle) {
  return angle * M_PI / 180.0;
}

inline Vec3 Rad2Deg(const Vec3& angles) {
  return Vec3(Rad2Deg(angles(0)), Rad2Deg(angles(1)), Rad2Deg(angles(2)));
}

inline Vec3 Deg2Rad(const Vec3& angles) {
  return Vec3(Deg2Rad(angles(0)), Deg2Rad(angles(1)), Deg2Rad(angles(2)));
}

inline Vec3 RToZYX(const Rot3& rot) {
  return rot.GetEulerZYX();
}

inline Rot3 ZYXToR(const Vec3& angles) {
  return Rot3(angles);
}

inline Rot3 QuatToR(const Quat& quat) {
  return Rot3(quat);
}

inline Quat RToQuat(const Rot3& rot) {
  return Quat(Eigen::Quaterniond(rot.Eigen()));
}

inline double GetRoll(const Rot3& r) {
  return r.Roll();
}

inline double GetRoll(const Quat& q) {
  return Rot3(q).Roll();
}

inline double GetPitch(const Rot3& r) {
  return r.Pitch();
}

inline double GetPitch(const Quat& q) {
  return Rot3(q).Pitch();
}

inline double GetYaw(const Rot3& r) {
  return r.Yaw();
}

inline double GetYaw(const Quat& q) {
  return Rot3(q).Yaw();
}

inline double SO3Error(const Quat& q1, const Quat& q2) {
  return Rot3(q1).Error(Rot3(q2));
}

inline double SO3Error(const Rot3& r1, const Rot3& r2) {
  return r1.Error(r2);
}

inline Vec3 CartesianToSpherical(const Vec3& v) {
  double rho = v.Norm();
  return Vec3(rho, acos(v.Z() / rho), atan2(v.Y(), v.X()));
}

inline Vec3 SphericalToCartesian(const Vec3& v) {
  return Vec3(v(0) * sin(v(1)) * cos(v(2)), v(0) * sin(v(1)) * sin(v(2)),
              v(0) * cos(v(1)));
}

inline Vec3 NEDCartesian(const Vec3& v) {
  return Vec3(v(0), -v(1), -v(2));
}

}

#endif
