#pragma once
#include "ceres/ceres.h"
#include "pnc/define/planner_constants.hpp"

namespace fast_math {
static constexpr double MathEpsilon = 1e-10;
/**
 * @brief Fast compute pow of 2 without using slow std::pow
 */
template <typename T> T integer_pow_2(const T &in) { return in * in; }

/**
 * @brief Fast compute pow of 3 without using slow std::pow
 */
template <typename T> T integer_pow_3(const T &in) { return in * in * in; }

/**
 * @brief Abs function with smooth derivatives
 *        In [0, 1] range, abs is steeper than x^2 and x^3
 */
template <typename T> T smooth_abs(const T &in) {
  return sqrt(in * in + MathEpsilon) - sqrt(MathEpsilon);
}

template <typename T> T math_sqrt(const T &in) { return sqrt(in); }

// Compute cos and sin simultaneously to save compute in ceres solver
inline void cos_sin_fun(const double &x, double &cosx, double &sinx) {
  cosx = std::cos(x);
  sinx = std::sin(x);
}

// We can reuse sinx and cosx to calculate their derivatives(Jet) to save
// compute.
template <typename Jet>
inline void cos_sin_fun(const Jet &x, Jet &cosx, Jet &sinx) {
  const double cosx_d = std::cos(x.a);
  const double sinx_d = std::sin(x.a);

  // Jet.a and Jet.v are used to compute value and derivatives
  // using chain rule we can get: cosx.v = - sinx * x.v
  cosx = {cosx_d, -sinx_d * x.v};
  sinx = {sinx_d, cosx_d * x.v};
}

// The derivative of sqrt at zero is inf, which makes derivative eval fail.
// Add an offset to get smooth sqrt.
template <typename T> T smooth_sqrt(const T &in) {
  return sqrt(in + MathEpsilon) - sqrt(MathEpsilon);
}

inline double get_val(const double &val) { return val; }

inline double
get_val(const ceres::Jet<double, path_planner::TOTAL_NUM_PARAMS> &val) {
  return val.a;
}

const int num_params_speed = speed_planner::TOTAL_NUM_PARAMS;
const int num_params_path = path_planner::TOTAL_NUM_PARAMS;
#if num_params_speed != num_params_path
inline double
get_val(const ceres::Jet<double, speed_planner::TOTAL_NUM_PARAMS> &val) {
  return val.a;
}
#endif

} // namespace fast_math
