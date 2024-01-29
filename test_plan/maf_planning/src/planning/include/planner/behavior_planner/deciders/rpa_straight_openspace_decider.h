#pragma once

#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "common/math/map_line.h"
#include "common/math/vec2d.h"
#include "common/parking_lot.h"
#include "common/parking_obstacle_manager.h"
#include "common/parking_world_model.h"
#include "planner/behavior_planner/deciders/openspace_decider_interface.h"
#include "planner/message_type.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config_utils.h"
#include <chrono>

namespace msquare {
namespace parking {

class RpaStraightOpenspaceDecider : public BaseOpenspaceDecider {
public:
  RpaStraightOpenspaceDecider(const std::shared_ptr<WorldModel> &world_model);
  ~RpaStraightOpenspaceDecider();

  virtual void set_init_and_target(const TrajectoryPoint &init_state,
                                   const TrajectoryPoint &target_state) {
    ego_state_ = init_state;
    target_state_ = target_state;
  }
  virtual TrajectoryPoint get_init() { return ego_state_; }
  virtual void requestSquareMapping();
  virtual bool isReverseSearchRequired() { return false; }
  virtual void process();
  virtual void reset() override;

private:
  void retriveData();
  void getOpenspaceBoundary();
  bool getObstacles();

  Pose2D local_frame_pose_;
  Pose2D local_init_pose_;
  Pose2D local_target_pose_;

  std::vector<planning_math::LineSegment2d> tlines_;
};

} // namespace parking
} // namespace msquare