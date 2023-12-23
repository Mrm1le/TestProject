#pragma once
#include <array>

namespace cp_path_planner {
static constexpr size_t QUADRATURE_ORDER = 5;
static constexpr size_t NUM_PATH_CONTROL_POINTS = 7;
static constexpr size_t NUM_PATH_SEGMENTS = NUM_PATH_CONTROL_POINTS - 1;

enum class PathPlannerResiduals {
  SOFT_BOUNDARY = 0,
  HARD_BOUNDARY,
  INIT_CURVATURE,
  // CURVATURE_LIMIT,
  LAT_ACCEL,
  LAT_JERK,
  HEADING_ALIGN,
  REF_CENTERING,
  PREV_PLAN_CENTERING,
  OBSTACLE,
  OBSTACLE_INFLATION,
  // FRONT_CORNER_SOFT_BOUNDARY,
  // FRONT_CORNER_HARD_BOUNDARY,
  COUNT,
};

// Offset0 is current ego state, not optimization param
enum class PathPlannerParams {
  OFFSET1 = 0,
  OFFSET2,
  OFFSET3,
  OFFSET4,
  OFFSET5,
  OFFSET6,
  COUNT,
};

static constexpr size_t TOTAL_NUM_RESIDUALS =
    static_cast<size_t>(PathPlannerResiduals::COUNT);
static constexpr size_t TOTAL_NUM_PARAMS =
    static_cast<size_t>(PathPlannerParams::COUNT);
using OptParamArray = std::array<double, TOTAL_NUM_PARAMS>;

// Currently number of params is the same with number of spline segments.
// This might change in the future.
static_assert(TOTAL_NUM_PARAMS == NUM_PATH_CONTROL_POINTS - 1,
              "Path planner params definition mismatch");

} // namespace path_planner

namespace cp_speed_planner {
  
static constexpr size_t NUM_SPEED_CONTROL_POINTS = 7;
static constexpr size_t NUM_SPEED_SEGMENTS = NUM_SPEED_CONTROL_POINTS - 1;

enum class SpeedPlannerResiduals {
  CURVATURE_SPEED_LIMIT = 0,
  ACCEL_LIMIT,
  ACCEL,
  JERK_LIMIT,
  JERK,
  STOP_POINT,
  RCC_POINT,
  ACC,
  MODEL_REFERENCE_SPEED,
  SAFETY,
  GMP_REFERENCE_SPEED,
  COUNT,
};

// Offset0 is current ego state, not optimization param
enum class SpeedPlannerParams {
  OFFSET1 = 0,
  OFFSET2,
  OFFSET3,
  OFFSET4,
  OFFSET5,
  OFFSET6,
  COUNT,
};

static constexpr size_t TOTAL_NUM_RESIDUALS =
    static_cast<size_t>(SpeedPlannerResiduals::COUNT);
static constexpr size_t TOTAL_NUM_PARAMS =
    static_cast<size_t>(SpeedPlannerParams::COUNT);
using OptParamArray = std::array<double, TOTAL_NUM_PARAMS>;

// Currently number of params is the same with number of spline segments.
// This might change in the future.
static_assert(TOTAL_NUM_PARAMS == NUM_SPEED_CONTROL_POINTS - 1,
              "Speed planner params definition mismatch");

} // namespace speed_planner
