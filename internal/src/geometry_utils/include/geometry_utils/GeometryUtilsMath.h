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

#ifndef GEOMETRY_UTILS_MATH_H
#define GEOMETRY_UTILS_MATH_H

#include <cmath>

namespace geometry_utils {
namespace math {

  template<typename T> inline T cos(T in);
  template<> inline float cos(float in) { return ::cosf(in); }
  template<> inline double cos(double in) { return ::cos(in); }

  template<typename T> inline T acos(T in);
  template<> inline float acos(float in) { return ::acosf(in); }
  template<> inline double acos(double in) { return ::acos(in); }

  template<typename T> inline T sin(T in);
  template<> inline float sin(float in) { return ::sinf(in); }
  template<> inline double sin(double in) { return ::sin(in); }

  template<typename T> inline T asin(T in);
  template<> inline float asin(float in) { return ::asinf(in); }
  template<> inline double asin(double in) { return ::asin(in); }

  template<typename T> inline T tan(T in);
  template<> inline float tan(float in) { return ::tanf(in); }
  template<> inline double tan(double in) { return ::tan(in); }

  template<typename T> inline T atan(T in);
  template<> inline float atan(float in) { return ::atanf(in); }
  template<> inline double atan(double in) { return ::atan(in); }

  template<typename T> inline T fabs(T in);
  template<> inline float fabs(float in) { return ::fabsf(in); }
  template<> inline double fabs(double in) { return ::fabs(in); }

  template<typename T> inline T fmin(T v1, T v2);
  template<> inline float fmin(float v1, float v2) { return ::fminf(v1, v2); }
  template<> inline double fmin(double v1, double v2) { return ::fmin(v1, v2); }

  template<typename T> inline T fmax(T v1, T v2);
  template<> inline float fmax(float v1, float v2) { return ::fmaxf(v1, v2); }
  template<> inline double fmax(double v1, double v2) { return ::fmax(v1, v2); }

  template<typename T> inline T sqrt(T in);
  template<> inline float sqrt(float in) { return ::sqrtf(in); }
  template<> inline double sqrt(double in) { return ::sqrt(in); }

  template<typename T> inline T pow(T in, T exp);
  template<> inline float pow(float in, float exp) { return ::powf(in, exp); }
  template<> inline double pow(double in, double exp) { return ::pow(in, exp); }

  template<typename T> inline T atan2(T y, T x);
  template<> inline float atan2(float y, float x) { return ::atan2f(y, x); }
  template<> inline double atan2(double y, double x) { return ::atan2(y, x); }

  template<typename T> inline T hypot(T x, T y);
  template<> inline float hypot(float x, float y) { return ::hypotf(x, y); }
  template<> inline double hypot(double x, double y) { return ::hypot(x, y); }

}
}
#endif
