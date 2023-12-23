#pragma once

#include "acc_types.hpp"
#include "ceres/ceres.h"
#include "pnc/define/speed_planner_interface.hpp"
#include "speed_planner_constants.hpp"
#include <unordered_map>

namespace speed_planner {

template <typename T> struct CommonTerms {
  double s = 0.0;
  T t;
  T v;
  T a;
  T jerk;
  double quad_weight;
};

} // namespace speed_planner
