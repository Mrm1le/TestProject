#ifndef MSQUARE_RULE_MATH
#define MSQUARE_RULE_MATH

#include <cmath>

namespace msquare {
inline double pow2(double a) { return a * a; }
inline bool inQuadrant1(double rad) { return (rad < M_PI_2 && rad > -1e-6); }
inline bool inQuadrant1Out(double rad) { return (rad < M_PI_2 && rad > -0.1); }

inline bool inQuadrant4(double rad) { return (rad < 0.0 && rad > -M_PI_2); }

/**
 * @brief solve quadratic equation with one unknown
 *
 * x = (-b +/- sqrt(b^2 - 4ac)) / (2a)
 *
 * @param a
 * @param b
 * @param c
 * @param x1
 * @param x2
 * @return bool
 */
inline bool solveQuadratic(double a, double b, double c, double &x1,
                           double &x2) {
  if (std::abs(a) < 1e-6) {
    if (std::abs(b) < 1e-6) {
      return false;
    }
    x1 = -c / b;
    x2 = -c / b;
    return true;
  }

  double disci = b * b - 4 * a * c;
  if (disci < 0) {
    return false;
  }

  double sqrt_discri = std::sqrt(disci);
  x1 = (-b + sqrt_discri) / (2 * a);
  x2 = (-b - sqrt_discri) / (2 * a);
  return true;
}

} // namespace msquare

#endif