#pragma once

#include <fstream>
#include <vector>

#include "common/math/vec2d.h"
#include "common/sbp_rspath.h"

#include "../State.hpp"
#include "../openspace_footprint_model.h"
#include "../reeds_shepp_path.h"
#include "../sbp_obstacle_interface.h"
#include "../search_based_planner.h"
#include "boundary_cost.hpp"
#include "bucket_priority_queue.hpp"
#include "curve.hpp"
#include "heuristic_cost.hpp"
#include "multi_circle_footprint_model.h"
#include "obstacle_grid.h"
#include "obstacle_grid_test.h"
#include "resource_pool.hpp"
#include "search_node.hpp"
#include "state_expansion.hpp"
#include "state_map.hpp"
#include "transform.hpp"

namespace msquare {

namespace hybrid_a_star_2 {

inline float getRelativeX(const curve::State &point, const SearchNode &ref) {
  float dx = point.x - ref.x();
  float dy = point.y - ref.y();
  return dx * ref.cos_theta() + dy * ref.sin_theta();
}

inline float getRelativeX(const SearchNode &point, const SearchNode &ref) {
  return getRelativeX(point.state(), ref);
}

inline float getRelativeY(const curve::State &point, const SearchNode &ref) {
  float dx = point.x - ref.x();
  float dy = point.y - ref.y();
  return dx * -ref.sin_theta() + dy * ref.cos_theta();
}

inline float getRelativeY(const SearchNode &point, const SearchNode &ref) {
  return getRelativeY(point.state(), ref);
}

class HybridAstar2 : public SearchBasedPlanner {
public:
  std::fstream search_node_record;
  explicit HybridAstar2(const std::string &config_folder = std::string());
  ~HybridAstar2();

  void Update(const planning_math::Box2d map_boundary,
              const std::vector<planning_math::LineSegment2d> &map =
                  std::vector<planning_math::LineSegment2d>());
  bool Plan(const std::vector<SbpObstaclePtr> &obs_ptrs,
            parking::SearchProcessDebug *sp_debug = nullptr);
  SbpResult getResult();
  std::vector<Pose2D> getSearchPoints();

  enum class SlotType : std::size_t {
    UNKNOWN,
    VERTICAL,
    OBLIQUE,
    PARALLEL,
    VERTICAL_OUT,
    OBLIQUE_OUT,
    PARALLEL_OUT,
    TOTAL_COUNT,
  };
  void setSlotType(SlotType value);
  void setSlotTypeByPlanningCore(int value);

  static bool planningCoreSupported(int value);
#ifdef BUILD_IN_TEST_BAG_RECURRENT
  static void createHeuristic(HeuristicCost &heuristic_cost,
                              float min_turn_radius, float gear_switch_cost,
                              float x_range, float y_range, float extend_range,
                              float xy_resolution, float theta_resolution,
                              float xy_resolution_state,
                              float theta_resolution_state);
#endif // BUILD_IN_TEST_BAG_RECURRENT

private:
  void updateConfig();
  void updateStateMap(const SearchNode &start, const SearchNode &end);

  MultiModelMinDistances calcMinDistanceToObstacle(SearchNode &current_state);
  MultiModelMinDistances
  calcMinDistanceToObstacle(float x, float y, float cos_theta, float sin_theta);
  float getLatInflation(const SearchNode &current);
  bool checkCollisionUsingMultiCircleForLonInflation(
      bool forward, const curve::State &current, float lat_inflation,
      float lon_inflation, float lat_inflation_low);
  bool checkCollisionUsingMultiCircleForLatInflation(SearchNode &current,
                                                     float lat_inflation,
                                                     float lon_inflation,
                                                     float lat_inflation_low);
  bool checkCollisionUsingMultiCircle(SearchNode &current_state,
                                      float lat_inflation, float lon_inflation,
                                      float lat_inflation_low);
  bool checkCollisionUsingMultiCircle(SearchNode &current_state);
  bool checkCollisionUsingMultiCircle(float x, float y, float cos_theta,
                                      float sin_theta, float inflation,
                                      float inflation_low);

  const SearchNode *simpleRSAnalyticExpansion(const SearchNode &current,
                                              const SearchNode &end);
  const SearchNode *simpleRSAnalyticExpansionArc(const SearchNode &current,
                                                 const SearchNode &end);
  const SearchNode *
  simpleRSAnalyticExpansionEulerSpiral(const SearchNode &current,
                                       const SearchNode &end);
  const SearchNode *
  simpleRSAnalyticExpansionLineOnArc(const SearchNode &current,
                                     const SearchNode &end);
  const SearchNode *
  simpleRSAnalyticExpansionLineOnPoint(const SearchNode &current,
                                       const SearchNode &end);
  const SearchNode *simpleRSAnalyticExpansionDirect(const SearchNode &current,
                                                    const SearchNode &end);
  template <int NodeCount>
  const SearchNode *simpleRSAnalyticExpansionConstruct(
      std::array<const SearchNode *, NodeCount> nodes, float offset);

  void combineTrajectory(const SearchNode *current, const Transform &trans);
  bool isPlanInSlot(const SearchNode &start);
  bool isReplanInSlot(const SearchNode &start);

  float maxStraightDistance(const SearchNode &current, float max,
                            float resolution = 0.01f);
  bool checkCollisionAfterStraight(const SearchNode &current, float distance);

  static SlotType planningCoreToSlotType(int value);

  void setStartRange(double x_range, double y_range,
                     double theta_range) override;

  bool inRangeBasedEnd(const curve::State &current, const SearchNode &end) {
    return std::abs(getRelativeX(current, end)) < simple_rs_end_x_range_ &&
           std::abs(getRelativeY(current, end)) < simple_rs_end_y_range_ &&
           std::abs(planning_math::AngleDiff(end.theta(), current.theta)) <
               simple_rs_end_theta_range_;
  }

  bool inRangeBasedEnd(const SearchNode &current, const SearchNode &end) {
    return inRangeBasedEnd(current.state(), end);
  }

  bool isParkIn() {
    return slot_type_ == SlotType::VERTICAL ||
           slot_type_ == SlotType::OBLIQUE || slot_type_ == SlotType::PARALLEL;
  }

  bool isParkOut() {
    return slot_type_ == SlotType::VERTICAL_OUT ||
           slot_type_ == SlotType::OBLIQUE_OUT ||
           slot_type_ == SlotType::PARALLEL_OUT;
  }

  bool useHeuristicCostFromInside() {
    return slot_type_ == SlotType::VERTICAL || slot_type_ == SlotType::OBLIQUE;
  }

  void createHeuristicCostFromInside(
      int max_iter, float gear_switch_cost, float min_x, float max_x,
      float min_y, float max_y, const Pose2D &local_frame_pose,
      bool require_forward, const Transform &trans,
      parking::SearchProcessDebug *sp_debug = nullptr);

  VariableStepSizeStateExpansion state_expansion_;

  ResourcePool<SearchNode> nodes_pool_;
  // extra size for start / end node and nodes generated in last iter
  static constexpr int nodes_pool_extra_size_ = 100;
  ResourcePool<SearchNode> simple_rs_nodes_pool_;

  std::vector<SbpObstaclePtr> obs_ptrs_;

  // obstacle grid and multi-circle footprint model
  // are for hybrid astar 2.0 collision(min distance to obstacle) check
  ObstacleGrid obs_grid_;
  ObstacleGridTest obs_grid_test_;
  MultiCircleFootprintModel mc_footprint_model_;

  struct VehicleConfig {
    std::function<void(MultiCircleFootprintModel &)> mc_model_loader;
    float max_steer_rate;
    float min_turn_radius_buffer;
  };
  std::map<std::string, VehicleConfig> vehicle_to_config_;
  std::map<float, HeuristicCost> heuristic_cost_;
  HeuristicCost heuristic_cost_from_inside_;
  BoundaryCost boundary_cost_;

  template <typename NodeContainer> class KeyPriorityValueFunc {
  public:
    using ValueType = float;
    KeyPriorityValueFunc(const NodeContainer &data) : data_(data) {}
    ValueType operator()(int key) const {
      return data_[key].trajectory_cost() + data_[key].heuristic_cost();
    }

  private:
    const NodeContainer &data_;
  };

  using PriorityValueFunc = KeyPriorityValueFunc<ResourcePool<SearchNode>>;
  PriorityValueFunc pq_cost_func_;
  BucketPriorityQueue<int, PriorityValueFunc> pq_;

  int max_iter_base_;
  int max_iter_max_;
  float lon_inflation_;
  float lat_inflation_;
  float lat_inflation_low_;
  float vehicle_length_real_;
  int max_zigzag_allowd_;
  float min_turn_radius_;
  float max_steer_rate_;
  float simple_rs_min_spiral_scale_forward_;
  float simple_rs_min_spiral_scale_backward_;
  float spiral_straight_max_t_;
  float plan_in_slot_x_min_;
  float plan_in_slot_x_max_;
  float plan_in_slot_y_th_;
  float lat_inflation_add_parallel_long_reverse_;
  float lat_inflation_add_slot_entry_;
  float lat_inflation_add_out_slot_;
  float lat_inflation_add_in_slot_x_min_;
  float lat_inflation_add_in_slot_x_max_;
  float lat_inflation_add_in_slot_y_th_;
  float lat_inflation_add_in_slot_theta_th_;
  float lat_inflation_add_slot_entry_x_min_;
  float lat_inflation_add_slot_entry_x_max_;
  float lat_inflation_add_slot_entry_y_th_;
  float lat_inflation_add_slot_entry_theta_th_;
  float min_length_per_segment_;
  float min_length_per_output_point_;
  float simple_rs_end_offset_th_;
  float simple_rs_arc_max_arc_len_;
  float simple_rs_euler_spiral_max_arc_len_;
  float simple_rs_euler_spiral_min_straight_len_;
  float min_spiral_scale_close_gear_switch_;
  float check_collision_max_stride_;
  SearchNode::Config node_config_;

  template <typename NodeContainer> class KeyTrajectoryCostFunc {
  public:
    using ValueType = float;
    KeyTrajectoryCostFunc(const NodeContainer &data) : data_(data) {}
    ValueType operator()(const int key) const {
      return data_[key].trajectory_cost();
    }
    ValueType operator()(const SearchNode &node) const {
      return node.trajectory_cost();
    }

  private:
    const NodeContainer &data_;
  };

  using TrajectoryCostFunc = KeyTrajectoryCostFunc<ResourcePool<SearchNode>>;
  TrajectoryCostFunc state_map_cost_func_;
  StateMap<3, int, TrajectoryCostFunc> state_map_coarse_;
  StateMap<3, int, TrajectoryCostFunc> state_map_fine_end_;

  SlotType slot_type_;
  using SimpleRSMethod = decltype(&HybridAstar2::simpleRSAnalyticExpansionArc);
  std::vector<std::vector<SimpleRSMethod>> simple_rs_methods_for_slot_type_;
  ResourcePool<curve::Curve> simple_rs_curve_pool_;
  curve::Curve simple_rs_temp_curve_;

  bool enable_offline_boundary_cost_ = false;
  bool no_steer_change_gear_switch_ = false;

  float simple_rs_end_x_range_ = 0.0f;
  float simple_rs_end_y_range_ = 0.0f;
  float simple_rs_end_theta_range_ = 0.0f;
  bool simple_rs_range_based_end_ = false;

  bool config_valid_ = false;
};

} // namespace hybrid_a_star_2

} // namespace msquare
