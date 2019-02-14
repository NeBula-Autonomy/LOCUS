/*
geometry_utils: Utility library to provide common geometry types and transformations
Copyright (C) 2013  Nathan Michael
              2015  Erik Nelson

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

#ifndef GEOMETRY_UTILS_ROS_H
#define GEOMETRY_UTILS_ROS_H

#include "GeometryUtils.h"

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>

namespace geometry_utils {
namespace ros {

inline Vec3 FromROS(const geometry_msgs::Point& p) {
  return Vec3(p.x, p.y, p.z);
}

inline Vec3 FromROS(const geometry_msgs::Point32& p) {
  return Vec3(p.x, p.y, p.z);
}

inline Vec3 FromROS(const geometry_msgs::Vector3& p) {
  return Vec3(p.x, p.y, p.z);
}

inline Quat FromROS(const geometry_msgs::Quaternion& msg) {
  return Quat(msg.w, msg.x, msg.y, msg.z);
}

inline Transform3 FromROS(const geometry_msgs::Pose& msg) {
  return Transform3(FromROS(msg.position), QuatToR(FromROS(msg.orientation)));
}

inline Transform3 FromROS(const geometry_msgs::Transform& msg) {
  return Transform3(FromROS(msg.translation), QuatToR(FromROS(msg.rotation)));
}

inline geometry_msgs::Point ToRosPoint(const Vec2& v) {
  geometry_msgs::Point msg;
  msg.x = v(0);
  msg.y = v(1);
  msg.z = 0.0;

  return msg;
}

inline geometry_msgs::Point ToRosPoint(const Vec3& v) {
  geometry_msgs::Point msg;
  msg.x = v(0);
  msg.y = v(1);
  msg.z = v(2);

  return msg;
}

inline geometry_msgs::Point32 ToRosPoint32(const Vec2& v) {
  geometry_msgs::Point32 msg;
  msg.x = v(0);
  msg.y = v(1);
  msg.z = 0.0f;

  return msg;
}

inline geometry_msgs::Point32 ToRosPoint32(const Vec3& v) {
  geometry_msgs::Point32 msg;
  msg.x = v(0);
  msg.y = v(1);
  msg.z = v(2);

  return msg;
}

inline geometry_msgs::Vector3 ToRosVec(const Vec2& v) {
  geometry_msgs::Vector3 msg;
  msg.x = v(0);
  msg.y = v(1);
  msg.z = 0.0;

  return msg;
}

inline geometry_msgs::Vector3 ToRosVec(const Vec3& v) {
  geometry_msgs::Vector3 msg;
  msg.x = v(0);
  msg.y = v(1);
  msg.z = v(2);

  return msg;
}

inline geometry_msgs::Quaternion ToRosQuat(const Quat& quat) {
  geometry_msgs::Quaternion msg;
  msg.w = quat.W();
  msg.x = quat.X();
  msg.y = quat.Y();
  msg.z = quat.Z();

  return msg;
}

inline geometry_msgs::Pose ToRosPose(const Transform2& trans) {
  geometry_msgs::Pose msg;
  msg.position = ToRosPoint(trans.translation);
  msg.orientation = ToRosQuat(RToQuat(Rot3(trans.rotation)));

  return msg;
}

inline geometry_msgs::Pose ToRosPose(const Transform3& trans) {
  geometry_msgs::Pose msg;
  msg.position = ToRosPoint(trans.translation);
  msg.orientation = ToRosQuat(RToQuat(trans.rotation));

  return msg;
}

inline geometry_msgs::Transform ToRosTransform(const Transform2& trans) {
  geometry_msgs::Transform msg;
  msg.translation = ToRosVec(trans.translation);
  msg.rotation = ToRosQuat(RToQuat(Rot3(trans.rotation)));

  return msg;
}

inline geometry_msgs::Transform ToRosTransform(const Transform3& trans) {
  geometry_msgs::Transform msg;
  msg.translation = ToRosVec(trans.translation);
  msg.rotation = ToRosQuat(RToQuat(trans.rotation));

  return msg;
}

inline geometry_msgs::Quaternion ToRosQuat(const Vec3& angles) {
  return ToRosQuat(RToQuat(ZYXToR(angles)));
}

inline Vec3 RosQuatToZYX(const geometry_msgs::Quaternion& msg) {
  return RToZYX(QuatToR(FromROS(msg)));
}

inline double GetRoll(const geometry_msgs::Quaternion& q) {
  return Rot3(FromROS(q)).Roll();
}

inline double GetPitch(const geometry_msgs::Quaternion& q) {
  return Rot3(FromROS(q)).Pitch();
}

inline double GetYaw(const geometry_msgs::Quaternion& q) {
  return Rot3(FromROS(q)).Yaw();
}

}
}
#endif
