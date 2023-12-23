#ifndef SEARCH_BASED_PLANNER_H
#define SEARCH_BASED_PLANNER_H

#include "common/math/math_utils.h"
#include "common/parking_planner_types.h"
#include "nlohmann/json.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/perpendicular_scenario_adapter.h"
#include "pnc/define/geometry.h"
#include "reeds_shepp_path.h"
#include "sbp_obstacle_interface.h"
#include <fstream>
#include <iostream>
#include <vector>

namespace msquare {

enum SbpStatus : int {
  SUCCESS = 0,
  INFEASIBLE = 1, //!< Infeasible
  TIMEOUT = 2,    //!< TimeOut
  EXCEPTION = 4,  //!< Exeception
  START_INFEASIBLE = 8,
  END_INFEASIBLE = 16,
};

struct SbpResult {
  SbpResult() = default;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<double> steer;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> accumulated_s;
  std::vector<double> wheel_base_offset;
  size_t num_segments;
  size_t iteration_times = 1000000;
  std::string debug_string;
  SbpStatus status;
  void isAligned() {
    unsigned long x_size = x.size();
    if (y.size() != x_size) {
      // throw std::logic_error("y of sbpresult is not aligned!");
    }
    if (phi.size() != x_size) {
      // throw std::logic_error("phi of sbpresult is not aligned!");
    }
    if (v.size() != x_size) {
      // throw std::logic_error("v of sbpresult is not aligned!");
    }
    if (a.size() != x_size) {
      // throw std::logic_error("a of sbpresult is not aligned!");
    }
    if (steer.size() != x_size) {
      // throw std::logic_error("steer of sbpresult is not aligned!");
    }
    if (accumulated_s.size() != x_size) {
      // throw std::logic_error("accumulated_s of sbpresult is not aligned!");
    }
    if (wheel_base_offset.size() != x_size) {
      // throw std::logic_error("wheel_base_offset of sbpresult is not
      // aligned!");
    }
  }
  void emplace_back(double x_arg, double y_arg, double phi_arg,
                    double steer_arg = 0, double v_arg = 0, double a_arg = 0,
                    double s_arg = 0, double wheel_base_offset_arg = 0) {
    x.push_back(x_arg);
    y.push_back(y_arg);
    phi.push_back(phi_arg);
    steer.push_back(steer_arg);
    v.push_back(v_arg);
    a.push_back(a_arg);
    accumulated_s.push_back(s_arg);
    wheel_base_offset.push_back(wheel_base_offset_arg);
  }

  void clear() {
    x.clear();
    y.clear();
    phi.clear();
    steer.clear();
    wheel_base_offset.clear();
    v.clear();
    a.clear();
    accumulated_s.clear();
  }
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SbpResult, x, y, phi, wheel_base_offset,
                                   debug_string)

inline SbpResult operator+(const SbpResult &lhs, const SbpResult &rhs) {
  SbpResult stitched_result(lhs);
  std::copy(rhs.x.begin(), rhs.x.end(), std::back_inserter(stitched_result.x));
  std::copy(rhs.y.begin(), rhs.y.end(), std::back_inserter(stitched_result.y));
  std::copy(rhs.phi.begin(), rhs.phi.end(),
            std::back_inserter(stitched_result.phi));
  std::copy(rhs.v.begin(), rhs.v.end(), std::back_inserter(stitched_result.v));
  std::copy(rhs.a.begin(), rhs.a.end(), std::back_inserter(stitched_result.a));
  std::copy(rhs.steer.begin(), rhs.steer.end(),
            std::back_inserter(stitched_result.steer));
  std::copy(rhs.wheel_base_offset.begin(), rhs.wheel_base_offset.end(),
            std::back_inserter(stitched_result.wheel_base_offset));

  std::vector<double> accumulated_s_biased = rhs.accumulated_s;
  double s_bias = 0;
  if (!stitched_result.accumulated_s.empty()) {
    s_bias = stitched_result.accumulated_s.back();
  }
  for (int i = 0; i < accumulated_s_biased.size(); ++i) {
    accumulated_s_biased.at(i) += s_bias;
  }
  std::copy(accumulated_s_biased.begin(), accumulated_s_biased.end(),
            std::back_inserter(stitched_result.accumulated_s));
  return stitched_result;
}

/**
 * @class SearchBasedPlannerInterface
 * @brief This abstract class defines an interface for Search Based planners
 */
class SearchBasedPlanner {
public:
  virtual ~SearchBasedPlanner() = default;
  /** @name Update SearchBasedPlanner*/

  /**
   * @brief Update and re-init SearchBasedPlanner.
   * @param goal_pose target pose
   * @param goal_v target velocity
   * @param map_boundary planning boundary
   * @param map obstacles in the current environment
   * @return no param
   */
  virtual void Update(const planning_math::Box2d map_boundary,
                      const std::vector<planning_math::LineSegment2d> &map =
                          std::vector<planning_math::LineSegment2d>()) = 0;

  /** @name Plan a trajectory */

  /**
   * @brief Plan a trajectory based on an initial reference plan.
   * @param obstacles in the current environment
   * @return \c true if planning was successful, \c false otherwise
   */
  virtual bool Plan(const std::vector<SbpObstaclePtr> &obs_ptrs,
                    parking::SearchProcessDebug *sp_debug = nullptr) = 0;

  /** @name Set the start pose */

  /**
   * @brief set ther planning start of the planner.
   * @param the starting pose
   */
  void setStartNode(const SearchNodePtr &node) { start_node_ = node; }
  /**
   * @brief set the planning target of the planner.
   * @param the starting pose
   */
  void setTargetNode(const SearchNodePtr &node) { end_node_ = node; }

  virtual void setStartRange(double x_range, double y_range,
                             double theta_range) {}
  virtual void setTargetRange(double x_range, double y_range,
                              double theta_range) {}

  /**
   * @brief get the planned trajectory
   * @param no param
   */

  virtual SbpResult getResult() = 0;

  /**
   * @brief for visualization,get all the searchpoints
   * @param no param
   */

  virtual std::vector<Pose2D> getSearchPoints() = 0;

  void get_global_env_points(std::vector<planning_math::Vec2d> &points) {
    perpendicular_scenario_adapter_.get_global_env_points(points);
  }

  std::unique_ptr<ReedShepp> reed_shepp_generator_;
  ReedSheppPath reeds_shepp_to_end_;

protected:
  double x_bound_;
  double y_bound_;
  double xy_grid_resolution_;
  double phi_grid_resolution_;
  double step_size_;
  double max_delta_angle_;
  double wheel_base_;
  double front_edge_to_rear_;
  double back_edge_to_rear_;
  double right_edge_to_center_;
  double left_edge_to_center_;
  double delta_t_;
  int next_node_num_;

  std::shared_ptr<SearchNode> start_node_;
  std::shared_ptr<SearchNode> end_node_;

  int verbose;
  int display_points;
  static const int NO_PRINT = 0;
  static const int BI_PRINT = 0;
  static const int FORWARD_PRINT = 0;
  static const int BACKWARD_PRINT = 0;
  static const int SHOW_CHANGE_NODES = 0;
  static const int SHOW_PATH = 0;

  static const int VERBOSE_DEBUG = 1;
  static const int BATCH_RESULTS = 2;

  // for visualization
  std::vector<Pose2D> searchPoints_;
  std::vector<std::vector<float>> searchEdges_;
  unsigned long iter;

  SbpResult result_;

  // to support any coordinate system.
  Pose2D local_frame_pose_;

  PerpendicularScenarioAdapter perpendicular_scenario_adapter_;
};

//! Abbrev. for shared instances of SearchBasedPlannerPtr or it's subclasses
typedef std::shared_ptr<SearchBasedPlanner> SearchBasedPlannerPtr;

} // namespace msquare

#endif // SEARCH_BASED_PLANNER_H
