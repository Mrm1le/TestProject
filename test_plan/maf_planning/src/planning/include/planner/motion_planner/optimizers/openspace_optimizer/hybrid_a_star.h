#pragma once

#include "Compare.hpp"
#include "State.hpp"
#include "common/math/vec2d.h"
#include "common/sbp_rspath.h"
#include "openspace_footprint_model.h"
#include "reeds_shepp_path.h"
#include "sbp_obstacle_interface.h"
#include "search_based_planner.h"
#include <fstream>
#include <vector>
namespace msquare {

class HybridAstar : public SearchBasedPlanner {
public:
  explicit HybridAstar(const int max_node_num);
  HybridAstar(const planning_math::Box2d map_boundary,
              const std::vector<planning_math::LineSegment2d> &map =
                  std::vector<planning_math::LineSegment2d>());
  ~HybridAstar();

  //<--------------------- Interface ------------------------>>//
  virtual void Update(const planning_math::Box2d map_boundary,
                      const std::vector<planning_math::LineSegment2d> &map =
                          std::vector<planning_math::LineSegment2d>());

  /**
   * @param: init_x initial x position, x > 0
   * @note: param init_v and target_v is not used yet
   */
  bool Plan(const std::vector<SbpObstaclePtr> &obs_ptrs,
            parking::SearchProcessDebug *sp_debug = nullptr);
  SbpResult getResult();

  bool AnalyticExpansion(std::shared_ptr<SearchNode> current_node,
                         std::vector<SbpObstaclePtr> obs_ptrs);
  bool IsNodeOutOfRange(std::shared_ptr<SearchNode> current_node);
  std::vector<std::shared_ptr<SearchNode>>
  getNextStates(const std::shared_ptr<SearchNode> &current);
  bool checkCollision(std::shared_ptr<SearchNode> current_state);
  bool checkCollisionReal(std::shared_ptr<SearchNode> current_state);
  double getObstacleCost(const SearchNodePtr &node,
                         const FootprintModelPtr &footprint_model);

  void combineTrajectory(SbpResult *result,
                         std::shared_ptr<SearchNode> current);

  // for visualziation
  std::vector<Pose2D> getSearchPoints();

private:
  double CalcSideDiff(const std::shared_ptr<SearchNode> &node);
  double calCircularRadius(std::shared_ptr<SearchNode> node_a,
                           std::shared_ptr<SearchNode> node_b);
  std::vector<double> wheel_base_offset_options_;
  double forward_distance_ = 0;
  double backward_distance_ = 0;

  // This value is used to set a threshold for distance of obstacle to car
  double safe_diff_distance_ = 3;

  std::vector<std::shared_ptr<SearchNode>> nodes_vec_;
  int nodes_size_;

  SbpRSPath sbp_rspath_;
  std::shared_ptr<ReedSheppPath> reeds_shepp_path_;
  FootprintModelPtr footprint_model_;
  FootprintModelPtr footprint_model_precise_;
  FootprintModelPtr footprint_model_real_;
  std::vector<SbpObstaclePtr> obs_ptrs_;
  SbpObstaclePtr obs_which_collision_;
};

} // namespace msquare
