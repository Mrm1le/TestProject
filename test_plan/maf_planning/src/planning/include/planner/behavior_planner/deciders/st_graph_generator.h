#ifndef MSQUARE_DECISION_PLANNING_PLANNER_ST_GRAPH_GENERATOR_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_ST_GRAPH_GENERATOR_H_

#include <limits>
#include <memory>
#include <string>
#include <unordered_set>

#include "common/obstacle.h"
#include "common/obstacle_decision_manager.h"
#include "common/obstacle_manager.h"
#include "common/path/path_data.h"
#include "common/scenario_facade_context.h"
#include "common/world_model.h"

namespace msquare {

typedef enum {
  SKETCHY_MODE = -0,
  ACCURATE_MODE = 1,
} STGraphConstructMode;

class StGraphGenerator {
private:
  using DoubleList = std::vector<double>;
  using Point2dMatrix = std::vector<std::vector<Point2D>>;
  using IntNonBlockObstacleUmap = std::unordered_map<int, NonBlockingObstacle>;
  using IntInSightStaticObstacleUmap =
      std::unordered_map<int, InSightStaticObstacle>;
  using IntInSightDynamicObstacleUmap =
      std::unordered_map<int, InSightDynamicObstacle>;

public:
  explicit StGraphGenerator(const std::shared_ptr<WorldModel> &world_model,
                            const std::shared_ptr<BaseLineInfo> &baseline_info,
                            SpeedBoundaryDeciderConfig config,
                            ScenarioFacadeContext *context);

  bool ComputeSTGraph();

  void compute_sl_polygon_sequence(ObstacleManager &obstacle_manager,
                                   ScenarioFacadeContext *context,
                                   std::shared_ptr<BaseLineInfo> baseline_info,
                                   StGraphData::DoublePair time_range,
                                   double time_gap);

  static bool compute_obs_sl_polygon_seq(
      Obstacle *obstacle, std::shared_ptr<BaseLineInfo> baseline_info,
      StGraphData::DoublePair time_range, double time_gap,
      bool enable_heuristic_search, std::string &failed_reason);

  static void generate_precise_sl_polygon(
      planning_math::Polygon2d &polygon,
      std::shared_ptr<FrenetCoordinateSystem> frenet_coord);

  static bool generate_rough_obs_sl_polygon(
      planning_math::Polygon2d &polygon, const Obstacle *obstacle,
      TrajectoryPoint point,
      std::shared_ptr<FrenetCoordinateSystem> frenet_coord, bool has_heuristic,
      double heuristic_s_begin, double heuristic_s_end);

  static std::pair<double, double> sl_polygon_path_check(
      const std::vector<planning_math::Polygon2d> &reference_line_segments,
      const planning_math::Polygon2d &polygon);

  bool enable_heuristic_search() {
    return config_.enable_heuristic_frenet_search;
  }

  bool exist_pedestrian_in_cross_walk(double &stop_distance);

  void clear();

  void update(const std::shared_ptr<WorldModel> &world_model,
              const std::shared_ptr<BaseLineInfo> &baseline_info,
              SpeedBoundaryDeciderConfig config,
              ScenarioFacadeContext *context);

private:
  void preliminary_screen_obstacles();

  void translate_scenario_decisions();

  void compute_st_boundaries();

  bool check_stop_for_pedestrian(const Obstacle &obstacle);

  void append_ignore_decision(ObstacleDecision *obstacle_decision) const;

  std::pair<double, double>
  sl_polygon_path_check(double left_border, double right_border,
                        const planning_math::Polygon2d &polygon);

  bool set_reference_line_border();

  void set_lane_changing_line_border(SLPoint &left_mid1, SLPoint &right_mid1,
                                     SLPoint &left_mid2, SLPoint &right_mid2);

  void generate_shifted_reference_line(
      PathData &reference_line,
      std::shared_ptr<FrenetCoordinateSystem> frenet_coord, double left_shift,
      double right_shift);

  void fill_lane_debug_info();

  void extract_info();

  void set_obstacle_new(const Obstacle *obstacle);

  void update_decision_info();

  void set_cross_lane_state();

private:
  std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<BaseLineInfo> baseline_info_;
  ScenarioFacadeContext *context_;

  //! considering s range
  StGraphData::DoublePair s_range_;
  //! considering time range
  StGraphData::DoublePair time_range_{};

  std::vector<SLPoint> left_border_, right_border_;
  std::vector<planning_math::Polygon2d> reference_line_segments_;

  bool is_in_lane_crossing_stage_;
  bool is_lane_interval_transformed_{false};

  SpeedBoundaryDeciderConfig config_;

  //! width of lane width
  static inline constexpr double LANE_WIDTH() { return 2.4; }
  //! consider s range, unit : m
  static inline constexpr double CONSIDER_RANGE() { return 90.0; }
  //! lateral consider range, unit : m
  static inline constexpr double LAT_CONSIDER_RANGE() { return 50.0; }
  //! velocity threshold of obstacle static, unit : m/s
  static inline constexpr double T_V0() { return 0.5; }
  //! rad ommit value, unit : rad
  static inline constexpr double PATH_TIME_GRAPH_OMMIT_RAD() { return 0.50; }

  double left_width_ = LANE_WIDTH() * 0.5;
  double right_width_ = LANE_WIDTH() * 0.5;

  static constexpr double kPedestrianStopTimeout = 4.0;
};

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_PLANNER_ST_GRAPH_GENERATOR_H_
