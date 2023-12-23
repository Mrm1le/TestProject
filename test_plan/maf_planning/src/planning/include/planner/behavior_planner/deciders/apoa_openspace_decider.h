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

class ApoaOpenspaceDecider : public BaseOpenspaceDecider {
public:
  ApoaOpenspaceDecider(const std::shared_ptr<WorldModel> &world_model);
  ~ApoaOpenspaceDecider();

  void set_init_and_target(const TrajectoryPoint &init_state,
                           const TrajectoryPoint &target_state) {
    ego_state_ = init_state;
    target_state_ = target_state;
  }
  TrajectoryPoint get_init() { return ego_state_; }
  void requestSquareMapping();
  virtual bool isReverseSearchRequired() {
    if (CarParams::GetInstance()->car_config.apoa_config.use_legacy_parkout)
      return false;
    else {
      return true;
    }
  }
  void process();
  void reset() override;

private:
  void InitLocalFrame();
  void get_openspace_boundary();
  bool gather_map_info();
  void obstacle_filter();
  void modify_inflation_for_near_obs();
  void modify_inflation_for_obs_near_target();
  void get_static_obstacle();
  void modify_inflation_for_car();
  void ignore_obstacles_near_target();

  void get_freespace_lead();

  std::vector<planning_math::LineSegment2d> boundary_lines_;
  Pose2D local_frame_pose_;
  TrajectoryPoint ego_state_local_;
  TrajectoryPoint target_state_local_;

  std::shared_ptr<ParkingSlotInterface> parking_lot_;
  planning_math::Box2d lot_box_;
  planning_math::Box2d lot_box_on_left_;
  planning_math::Box2d lot_box_on_right_;
};

} // namespace parking
} // namespace msquare