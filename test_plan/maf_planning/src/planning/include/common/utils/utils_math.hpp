#ifndef __UTILS__UTILS_MATH_HPP__
#define __UTILS__UTILS_MATH_HPP__

#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846 // pi
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923 // pi/2
#endif

#ifndef M_PI_4
#define M_PI_4 0.785398163397448309616 // pi/4
#endif

#ifndef M_1_PI
#define M_1_PI 0.318309886183790671538 // 1/pi
#endif

#ifndef M_2_PI
#define M_2_PI 0.636619772367581343076 // 2/pi
#endif

#ifndef DEG_PER_RAD
#define DEG_PER_RAD (180.0 / M_PI)
#endif

#ifndef DEG2RAD
#define DEG2RAD(x) ((x) / 180.0 * M_PI)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*180.0 / M_PI)
#endif

#define constrain_m(val, min_val, max_val)                                     \
  if ((val) < (min_val))                                                       \
    (val) = (min_val);                                                         \
  else if ((val) > (max_val))                                                  \
    (val) = (max_val);

#define constrain_abs_m(val, max_abs) constrain_m((val), -(max_abs), (max_abs))

template <class T> inline T constrain(T val, T min_val, T max_val) {
  return val > max_val ? max_val : (val < min_val ? min_val : val);
}

template <class T> inline T constrain_abs(T val, T max_abs) {
  return constrain(val, -max_abs, max_abs);
}

template <class T> inline T square(T x) { return x * x; }

template <class T> inline T square_sum(T x, T y) { return x * x + y * y; }

template <class T> inline T norm(T x, T y) { return sqrt(x * x + y * y); }

template <class T> inline T norm(T x, T y, T z) {
  return sqrt(x * x + y * y + z * z);
}

#endif
