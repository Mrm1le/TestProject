#pragma once

#include <vector>

// all public
// all static member files
// temporary usage only for hybrid astar 2.0 develop
// init function might be needed

namespace msquare {

namespace hybrid_a_star_2 {

class TmpGlobals {
public:
  static constexpr int CONFIG_VARIABLE_STEP_SIZE_STEER_OPTION_COUNT = 7;
  static constexpr float CONFIG_VARIABLE_STEP_SIZE_DISTANCE_STEP = 0.2f;
  static constexpr int CONFIG_VARIABLE_STEP_SIZE_DISTANCE_OPTION_COUNT = 3;
  static constexpr float CONFIG_VARIABLE_STEP_SIZE_DISTANCE_BIAS = 0.5f;

  static constexpr bool CONFIG_OBSTACLE_GRID_DEBUG = false;

  static constexpr float CONFIG_SIMPLE_RS_START_SEARCH_STEP = 0.01f;
  static constexpr float CONFIG_SIMPLE_RS_ARC_MAX_ARC_LENGTH = 1.0f;
  static constexpr float CONFIG_SIMPLE_RS_EULER_SPIRAL_STRAIGHT_DIRECTION_TH =
      2.0f;
  static constexpr float CONFIG_SIMPLE_RS_EULER_SPIRAL_MIN_STRAIGHT_LENGTH =
      1.0f;
  static constexpr float
      CONFIG_SIMPLE_RS_EULER_SPIRAL_REPLAN_IN_SLOT_MIN_STRAIGHT_LENGTH = 2.0f;

  static constexpr float CONFIG_MAX_PATH_LENGTH_PER_NODE = 0.1f;

  static constexpr float CONFIG_LAT_INFLATION_ADD_SLOT_ENTRY = 0.05f;
  static constexpr float CONFIG_LAT_INFLATION_ADD_OUT_SLOT = 0.08f;
  static constexpr float CONFIG_LAT_INFLATION_ADD_IN_SLOT_X_MIN = -1.0f;
  static constexpr float CONFIG_LAT_INFLATION_ADD_IN_SLOT_X_MAX = 3.0f;
  static constexpr float CONFIG_LAT_INFLATION_ADD_IN_SLOT_Y_TH = 0.3f;
  static constexpr float CONFIG_LAT_INFLATION_ADD_IN_SLOT_THETA_TH =
      0.17453f; // 10 deg
  static constexpr float CONFIG_LAT_INFLATION_ADD_SLOT_ENTRY_X_MIN = 3.0f;
  static constexpr float CONFIG_LAT_INFLATION_ADD_SLOT_ENTRY_X_MAX = 5.5f;
  static constexpr float CONFIG_LAT_INFLATION_ADD_SLOT_ENTRY_Y_TH = 1.0f;
  static constexpr float CONFIG_LAT_INFLATION_ADD_SLOT_ENTRY_THETA_TH =
      1.0472f; // 60 deg

  static void init() {}
};

} // namespace hybrid_a_star_2

} // namespace msquare
