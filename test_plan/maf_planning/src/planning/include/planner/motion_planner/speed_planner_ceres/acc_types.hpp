#pragma once
#include "speed_planner_constants.hpp"

namespace speed_planner {

enum class ACCRegionType {
  NONE = 0,
  DECEL_REGION,
  DEADBAND_REGION,
  ACCEL_REGION,
};

template <typename T> struct LonObjData {
  int id;
  T ds_follow_offseted = T(0);
  T dv_safe = T(MAX_VALID_SPEED_MPS);
  T dv_curve = T(MAX_VALID_SPEED_MPS);
  T dv_deadband = T(MAX_VALID_SPEED_MPS);
  T da_curve = T(0);
  T obj_v = T(MAX_VALID_SPEED_MPS);
  T obj_a = T(0);
  T obj_ds = T(0);
  T obj_headway = T(0);
};
} // namespace speed_planner
