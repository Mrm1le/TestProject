#ifndef EXTRACT_PATH_BOUND_H
#define EXTRACT_PATH_BOUND_H

#pragma once

#include "common/scenario_facade_context.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

// constexpr double kFlagsObstacleLonStartBuffer = 0.5;
// constexpr double kFlagsObstacleLonEndBuffer = 2.0;
// constexpr double kFlagsObstacleLatBuffer = 0.3;
constexpr double kStaticObstacleLonStartBuffer = 0.5;
constexpr double kStaticObstacleLonEndBuffer = 0.4;
constexpr double kStaticObstacleLatBuffer = 0.4;
constexpr double kDynamicObstacleLonStartBuffer = 0.5;
constexpr double kDynamicObstacleLonEndBuffer = 0.5;
constexpr double kDynamicObstacleLatBuffer = 0.3;

constexpr double kObstacleLonStartOffset = 10.0;
constexpr double kObstacleLonEndOffset = 5.0;
constexpr double kStaticObstacleLatOffset = 0.8;
constexpr double kDynamicObstacleLatOffset = 0.9;
constexpr double KRoadSafetyBuffer = 0.2;
constexpr double KPhysicalRoadSafetyBuffer = 0.4;
constexpr double kPolygonErrorLatOffset = 0.2;
constexpr bool isSparseMode = true;

namespace msquare {
// AvoidObstacle contains: (id, avoid direction).
using AvoidObstacle = std::tuple<int, std::string>;
// PathBoundPoint contains: (s, l_min, l_max).
using PathBoundPoint = std::tuple<double, double, double>;
// PathBound contains a vector of PathBoundPoints.
using PathBound = std::vector<PathBoundPoint>;
// edge points.
using EdgePoints = std::vector<std::tuple<double, double>>; // @zyl

enum class LaneBorrowInfo : int {
  NO_BORROW = 0,
  LEFT_BORROW = 1,
  RIGHT_BORROW = -1,
  URGENT_LEFT_BORROW = 2,
  URGENT_RIGHT_BORROW = -2,
  LB_RIGHT_NOLANE = 4,
  RB_LEFT_NOLANE = -4,
  BOTH_BORROW = 8,
};

struct ObsBoundDeciderInfo {
  double init_lat_buffer_coef;
  double init_lon_start_buffer_coef;
  double init_lon_end_buffer_coef;
  double lat_atten_time;
  double lat_atten_coef;
  double lon_atten_time;
  double lon_atten_coef;
  double constrain_time_buffer;
  double desire_time_buffer;
  bool rear_constrain;
  bool is_positive{false};
  bool is_ignore{false};
};

struct DynamicObs {
  std::vector<std::tuple<EdgePoints, double, double>> dyn_edge_points;
  std::string lat_decider_source;
  std::string side_pass;
  std::string lon_decider;
  double start_time;
  double time_buffer;
  double obstacle_type_buffer;
  int id;
  int priority;
  bool is_trunk;
  ObsBoundDeciderInfo avd_decider;
};

struct BlockType {
  enum Type {
    UNKNOWN,
    NOBLOCK,
    MAP,
    MAP_MOTION_LEFT,
    MAP_MOTION_RIGHT,
    OBSTACLE,
    OBSTACLE_MOTION_LEFT,
    OBSTACLE_MOTION_RIGHT
  };
  Type type = Type::UNKNOWN;
  double static_block_s;
  std::map<int, std::tuple<bool, double>> obs_avd_time;

  void clear() {
    type = Type::NOBLOCK;
    static_block_s = 1.0e6;
    obs_avd_time.clear();
  }
};

class ExtractPathBound {
public:
  ExtractPathBound(ScenarioFacadeContext *context,
                   const std::shared_ptr<WorldModel> &world_model,
                   const LaneBorrowInfo lane_borrow_info,
                   const std::shared_ptr<BaseLineInfo> &baseline_info,
                   LateralBehaviorPlannerConfig config);
  virtual ~ExtractPathBound();

  bool process(double lb_width, double lb_length,
               const std::vector<AvoidObstacle> &obs_avd,
               const std::vector<AvoidObstacle> &obs_avd_prior,
               BlockType &block_type);

  void update_lon_info(std::vector<std::tuple<double, double>> &v_limit,
                       std::vector<std::tuple<double, double>> &a_limit,
                       std::vector<std::tuple<double, double>> &lat_offset) {
    v_limit.assign(v_limit_.begin(), v_limit_.end());
    a_limit.assign(a_limit_.begin(), a_limit_.end());
    lat_offset.assign(lat_offset_.begin(), lat_offset_.end());
  }

  const std::unordered_map<int, int> &get_obs_priority() {
    return obs_priority_;
  }

private:
  void update_delta_s();

  void get_refline_info(const int target_lane_id);

  void cal_inter_refline(const std::vector<RefPointFrenet> &cur_lane,
                         const std::vector<RefPointFrenet> &left_lane,
                         const std::vector<RefPointFrenet> &right_lane,
                         std::vector<path_planner::RefPointInfo> &refline_info);

  bool constructObsInterface(const std::vector<AvoidObstacle> &obs_avd,
                             std::vector<DynamicObs> &obstacle_list);

  bool GenerateRegularPathBound(const double lb_width, const double lb_length,
                                const std::vector<AvoidObstacle> &obs_avd,
                                const std::vector<AvoidObstacle> &obs_avd_prior,
                                BlockType &block_type);

  bool InitPathBoundary(PathBound *const path_bound,
                        PathBound *const des_bound);

  bool GetBoundaryFromLanesAndADC(const double lb_width, const double lb_length,
                                  double ADC_buffer, PathBound *const des_bound,
                                  PathBound *const path_bound,
                                  BlockType &block_type);

  bool GetBoundaryFromLanesAndADC(PathBound *const des_bound,
                                  PathBound *const path_bound,
                                  BlockType &block_type);

  bool GetBoundaryListFromReverseObstacles(
      const std::vector<AvoidObstacle> &obs_avd, PathBound &path_bound,
      PathBound &des_bound, BlockType &block_type);

  bool GetBoundaryListFromObstacles(const std::vector<AvoidObstacle> &obs_avd,
                                    const PathBound &path_bound,
                                    BlockType &block_type);

  bool
  GetBoundaryFromStaticObstacles(const std::vector<AvoidObstacle> &obs_avd,
                                 BlockType &block_type, PathBound &des_bound,
                                 PathBound &path_bound,
                                 std::vector<std::vector<int>> &obstacle_lists);

  bool GetBoundaryFromObstacles(
      const std::vector<std::tuple<int, std::string, EdgePoints>> &edge_list,
      const double static_block_s, PathBound &path_bound,
      std::unordered_set<int> &block_obs);

  void get_obstacle_edge(
      int time_index, const std::vector<AvoidObstacle> &dynamic_obs_avd,
      std::vector<std::tuple<int, std::string, EdgePoints>> &edge_list);

  void cal_bound_s(
      const std::vector<std::tuple<int, std::string, EdgePoints>> &edge_list,
      const double curr_s, const double last_s, const double next_s,
      std::multiset<double, std::greater<double>> &right_bounds,
      std::multiset<double> &left_bounds, std::vector<int> &obs_ids);

  void cal_corners(const std::vector<planning_math::Vec2d> &points,
                   std::string side_pass, double lat_buffer,
                   double start_buffer, double end_buffer,
                   EdgePoints &edge_points);

  double
  cal_bound_temp(double curr_s, double last_s, double next_s,
                 const std::vector<std::tuple<double, double>> &edge_points,
                 string side_pass);

  void check_dynamic_ignore(BlockType &block_type);

  bool check_motion_collide(std::string check_type,
                            std::vector<std::vector<int>> &obstacle_lists,
                            PathBound *const path_boundaries,
                            BlockType &block_type);

  double cal_motion_buffer(double delta_s, double dddl,
                           std::pair<double, double>, double &l, double &dl,
                           double &ddl);

  double GetBufferBetweenADCCenterAndEdge();

  bool UpdatePathBoundaryAndCenterLine(size_t idx, double left_bound,
                                       double right_bound,
                                       PathBound *const path_boundaries);

  bool UpdatePathBoundaryAndCenterLine(size_t idx, double left_bound,
                                       double right_bound,
                                       const PathBound path_boundaries);

  bool check_dynamic_cutin(const AvoidObstacle obs);

  bool GetSLPolygonSeq(const std::vector<AvoidObstacle> &obs_avd);

  void update_obstacle_priority();

  void GetSpeedLimit(const std::vector<AvoidObstacle> &obs_avd,
                     const PathBound &des_bound,
                     const PathBound &des_bound_reverse,
                     const PathBound &path_bound);

  void GetLatOffset(const PathBound &des_bound);

  void CheckStaticIgnore(BlockType &block_type);

  void distinguish_cutin(const Obstacle *ptr_obstacle,
                         std::string obs_side_pass);

  void check_side_collision(const std::vector<AvoidObstacle> &obs_avd,
                            const BlockType &block_type);

  void UpdateAvdInfo(const BlockType &block_type);

  int num_knots_ = 40;
  double delta_t_ = 0.2;
  double max_vel_ = 120;
  double max_acc_ = 2.0;
  double min_acc_ = -4.0;
  double max_jerk_ = 3.0;
  double obs_active_time_default_ = 4.0;
  double delta_s_;
  std::vector<double> adc_s_list_;
  std::vector<double> s_max_list_;

  double adc_s_;
  double adc_l_;
  double adc_dl_;
  double adc_ddl_;
  double adc_vel_;
  double adc_acc_;
  SLBoundary adc_sl_boundary_;

  LaneBorrowInfo lane_borrow_info_;
  std::vector<path_planner::RefPointInfo> refline_info_;
  std::shared_ptr<WorldModel> world_model_;
  ScenarioFacadeContext *context_;
  LateralBehaviorPlannerConfig config_;
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;
  std::unordered_map<int, std::vector<planning_math::Polygon2d>> sl_polygon_;
  std::unordered_map<int, int> obs_priority_;
  std::unordered_map<int, int> avd_loop_;
  std::unordered_set<int> obs_reverse_;
  std::vector<AvoidObstacle> static_obs_avd_;
  std::vector<AvoidObstacle> dynamic_obs_avd_;

  std::vector<std::tuple<double, double>> v_limit_;
  std::vector<std::tuple<double, double>> a_limit_;
  std::vector<std::tuple<double, double>> lat_offset_;
  std::shared_ptr<BaseLineInfo> baseline_info_;

  double kPolygonErrorLatOffset_;
  double kLatBufferComp_;
  double kLatBufferComp_atten_;
  double kLatBufferCompMax_v_;
};

} // namespace msquare

#endif