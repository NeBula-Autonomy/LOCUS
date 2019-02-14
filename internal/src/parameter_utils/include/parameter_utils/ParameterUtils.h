/*
parameter_utils: Convenience utility functions for working with ROS parameters
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

#ifndef PARAMETER_UTILS_H
#define PARAMETER_UTILS_H

#include <cassert>
#include <ros/ros.h>

namespace ros {
namespace param {
inline bool get(const std::string& key, unsigned char& c) {
  int i;
  bool ret = ros::param::get(key, i);
  assert((i >= 0) && (i < 256));
  c = (unsigned char)i;
  return ret;
}

inline bool get(const std::string& key, unsigned int& d) {
  int i;
  bool ret = ros::param::get(key, i);
  assert(i >= 0);
  d = (unsigned int)i;
  return ret;
}

inline bool get(const std::string& key, float& f) {
  double d;
  bool ret = ros::param::get(key, d);
  f = (float)d;
  return ret;
}
}
}

namespace parameter_utils {
template <class M>
bool Get(const std::string& s, M& p) {
  std::string name = ros::this_node::getName();

  std::string r;
  if (!ros::param::search(s, r)) {
    ROS_ERROR("%s: Failed to search for parameter '%s'.", name.c_str(),
              s.c_str());
    return false;
  }

  if (!ros::param::has(r)) {
    ROS_ERROR("%s: Missing required parameter '%s'.", name.c_str(), s.c_str());
    return false;
  }

  if (!ros::param::get(r, p)) {
    ROS_ERROR("%s: Failed to get parameter '%s'.", name.c_str(), s.c_str());
    return false;
  }

  return true;
}

template <class M>
bool Get(const std::string& s, M& p, M def) {
  bool ret = true;

  std::string name = ros::this_node::getName();

  std::string r;
  if (!ros::param::search(s, r)) {
    ROS_DEBUG("%s: Failed to search for parameter '%s', using default.",
              name.c_str(), s.c_str());
    p = def;
    ret = false;
  }

  if (ret && !ros::param::has(r)) {
    ROS_DEBUG("%s: Missing required parameter '%s', using default.",
              name.c_str(), s.c_str());
    p = def;
    ret = false;
  }

  if (ret && !ros::param::get(r, p)) {
    ROS_DEBUG("%s: Failed to get parameter '%s', using default.", name.c_str(),
              s.c_str());
    p = def;
    ret = false;
  }

  return ret;
}
}
#endif
