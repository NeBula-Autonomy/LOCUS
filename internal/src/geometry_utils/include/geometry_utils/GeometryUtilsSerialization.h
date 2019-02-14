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

#ifndef GEOMETRY_UTILS_SERIALIZATION_H
#define GEOMETRY_UTILS_SERIALIZATION_H

#include "GeometryUtils.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace boost {
namespace serialization {

  template<class Archive>
  void Serialize(Archive & ar, geometry_utils::Vector2& v,
                 const unsigned int version) {
    ar & v(0);
    ar & v(1);
  }

  template<class Archive>
  void Serialize(Archive & ar, geometry_utils::Vector3& v,
                 const unsigned int version) {
    ar & v(0);
    ar & v(1);
    ar & v(2);
  }

  template<class Archive>
  void Serialize(Archive & ar, geometry_utils::Vector4& v,
                 const unsigned int version) {
    ar & v(0);
    ar & v(1);
    ar & v(2);
    ar & v(3);
  }

  template<class Archive>
  void Serialize(Archive & ar, geometry_utils::Matrix3x3& m,
                 const unsigned int version) {
    for (unsigned int i = 0; i < 3; i++)
      for (unsigned int j = 0; j < 3; j++)
        ar & m(i, j);
  }

  template<class Archive>
  void Serialize(Archive & ar, geometry_utils::Transform& t,
                 const unsigned int version) {
    ar & t.translation;
    ar & t.rotation;
  }

}
}
#endif
