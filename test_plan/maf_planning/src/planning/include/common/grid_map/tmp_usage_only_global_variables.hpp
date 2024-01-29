#pragma once

#include <vector>

// all public
// all static member files
// temporary usage only for hybrid astar 2.0 develop
// init function might be needed

namespace msquare {
namespace grid {

class TmpGlobals {
public:
  static constexpr int CONFIG_VARIABLE_STEP_SIZE_STEER_OPTION_COUNT = 7;
  static constexpr double CONFIG_VARIABLE_STEP_SIZE_DISTANCE_STEP = 0.2;
  static constexpr int CONFIG_VARIABLE_STEP_SIZE_DISTANCE_OPTION_COUNT = 3;
  static constexpr double CONFIG_VARIABLE_STEP_SIZE_DISTANCE_BIAS = 0.5;
  static constexpr double CONFIG_VARIABLE_STEP_SIZE_EULER_SPIRAL_MAX_SPEED =
      0.5;
  static constexpr double
      CONFIG_VARIABLE_STEP_SIZE_EULER_SPIRAL_MAX_STEER_RATE = 427.0;

  static constexpr bool CONFIG_OBSTACLE_GRID_DEBUG = false;

  enum class HeuristicMethod {
    NO = 0,
    EUCLIDEAN_DISTANCE = 1,
    MANHATTAN_DISTANCE = 2,
    EUCLIDEAN_DISTANCE_WITH_HEADING = 3,
    MANHATTAN_DISTANCE_WITH_HEADING = 4,
  };
  static constexpr HeuristicMethod CONFIG_HEURISTIC_METHOD =
      HeuristicMethod::MANHATTAN_DISTANCE;

  enum class OfflineHeuristicCostMethod {
    NO = 0,
    BOUNDARY = 1,
  };
  static constexpr OfflineHeuristicCostMethod
      CONFIG_OFFLINE_HEURISTIC_COST_METHOD =
          OfflineHeuristicCostMethod::BOUNDARY;
  static constexpr double CONFIG_OFFLINE_HEURISTIC_BOUNDARY_COST_WEIGHT = 10.0;

  enum class OfflineTrajCostMethod {
    NO = 0,
    BOUNDARY = 1,
  };
  static constexpr OfflineTrajCostMethod CONFIG_OFFLINE_TRAJ_COST_METHOD =
      OfflineTrajCostMethod::BOUNDARY;

  static constexpr double CONFIG_SIMPLE_RS_MAX_PATH_LENGTH_PER_NODE = 0.3;
  static constexpr double CONFIG_SIMPLE_RS_MIN_STRAIGHT_LENGTH = 1.0;
  static constexpr double CONFIG_SIMPLE_RS_MAX_CURVE_LENGTH = 5.0;
  static constexpr double CONFIG_SIMPLE_RS_START_SEARCH_STEP = 0.01;
  enum class SimpleRSCurveMode {
    ARC = 0,
    EULER_SPIRAL = 1,
  };
  static constexpr SimpleRSCurveMode CONFIG_SIMPLE_RS_CURVE_MODE =
      SimpleRSCurveMode::EULER_SPIRAL;
  static constexpr double CONFIG_SIMPLE_RS_EULER_SPIRAL_MAX_SPEED = 0.9;

  static constexpr double CONFIG_MAX_PATH_LENGTH_PER_NODE = 0.1;

  static void init() {}
};

} // namespace grid
} // namespace msquare
