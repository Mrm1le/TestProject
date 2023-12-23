#pragma once
#include "common/math/line_segment2d.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/State.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/reeds_shepp_path.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/sbp_obstacle_interface.h"

namespace msquare {

class SbpRSPath {
private:
  std::shared_ptr<ReedSheppPath> reeds_shepp_path_;
  std::vector<SearchNodePtr> nodes_;
  std::vector<SearchNodePtr> nodes_vec_;
  bool
  checkSingleNodeCollision(std::shared_ptr<SearchNode> current_state,
                           const std::vector<SbpObstaclePtr> obs_ptrs,
                           const FootprintModelPtr &footpint_model,
                           const FootprintModelPtr &footpint_model_precise);
  void calcTrajCost();
  void calcObstacleCost(const std::vector<SbpObstaclePtr> obs_ptrs,
                        const FootprintModelPtr &footprint_mode);

public:
  SbpRSPath();
  SbpRSPath(const std::shared_ptr<SearchNode> current_node,
            std::shared_ptr<ReedSheppPath> reeds_shepp_path);
  ~SbpRSPath();

  int max_nodes_num_ = 100;
  void update(const std::shared_ptr<SearchNode> current_node,
              std::shared_ptr<ReedSheppPath> reeds_shepp_path);
  std::vector<SearchNodePtr> getNodes() { return nodes_; }
  double getCost(const std::vector<SbpObstaclePtr> obs_ptrs,
                 const FootprintModelPtr &footprint_mode);
  bool checkCollision(const double step_size,
                      const std::vector<SbpObstaclePtr> obs_ptrs,
                      const FootprintModelPtr &footpint_model,
                      const FootprintModelPtr &footpint_model_precise);
  // [Hybrid Astar 2.0]a much simpler way to calculate cost, only use
  // trajc_cost, not obstacle needed
  double getCostWithoutObstacle();
};

} // namespace msquare
