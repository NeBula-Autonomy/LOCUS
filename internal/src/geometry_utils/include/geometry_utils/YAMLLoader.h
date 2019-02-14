/*
geometry_utils: Utility library to provide common geometry types and transformations
Copyright (C) 2013  Nathan Michael
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

#ifndef GEOMETRY_UTILS_YAML_LOADER_H
#define GEOMETRY_UTILS_YAML_LOADER_H

#ifdef __YAML_UTILS_H__
#error "YAMLUtils must be included last"
#endif

#include <yaml-cpp/yaml.h>
#include <geometry_utils/GeometryUtils.h>

namespace YAML {

template <>
struct convert<geometry_utils::Vector2> {
  static Node Encode(const geometry_utils::Vector2& rhs) {
    Node node;
    node.push_back(rhs[0]);
    node.push_back(rhs[1]);
    return node;
  }

  static bool Decode(const Node& node, geometry_utils::Vector2& rhs) {
    if (!node.IsSequence() || node.size() != 2)
      return false;

    rhs[0] = node[0].as<double>();
    rhs[1] = node[1].as<double>();
    return true;
  }
};

template <>
struct convert<geometry_utils::Vector3> {
  static Node Encode(const geometry_utils::Vector3& rhs) {
    Node node;
    node.push_back(rhs[0]);
    node.push_back(rhs[1]);
    node.push_back(rhs[2]);
    return node;
  }

  static bool Decode(const Node& node, geometry_utils::Vector3& rhs) {
    if (!node.IsSequence() || node.size() != 3)
      return false;

    rhs[0] = node[0].as<double>();
    rhs[1] = node[1].as<double>();
    rhs[2] = node[2].as<double>();
    return true;
  }
};

template <>
struct convert<geometry_utils::Vector4> {
  static Node Encode(const geometry_utils::Vector4& rhs) {
    Node node;
    node.push_back(rhs[0]);
    node.push_back(rhs[1]);
    node.push_back(rhs[2]);
    node.push_back(rhs[3]);
    return node;
  }

  static bool Decode(const Node& node, geometry_utils::Vector4& rhs) {
    if (!node.IsSequence() || node.size() != 4)
      return false;

    rhs[0] = node[0].as<double>();
    rhs[1] = node[1].as<double>();
    rhs[2] = node[2].as<double>();
    rhs[3] = node[3].as<double>();
    return true;
  }
};

}

#endif
