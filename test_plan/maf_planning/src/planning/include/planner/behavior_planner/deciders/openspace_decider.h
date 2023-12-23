#ifndef MSQUARE_DECISION_PLANNING_PLANNER_OPENSPACE_DECIDER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_OPENSPACE_DECIDER_H_

#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "common/math/map_line.h"
#include "common/math/vec2d.h"
#include "common/parking_obstacle_manager.h"
#include "common/parking_world_model.h"
#include "planner/behavior_planner/deciders/openspace_decider_interface.h"
#include "planner/message_type.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config_utils.h"
#include <chrono>

namespace msquare {

namespace parking {

class OpenspaceDecider : public BaseOpenspaceDecider {
public:
  OpenspaceDecider(const std::shared_ptr<WorldModel> &world_model);
  ~OpenspaceDecider();

  // void get_map_boundary_teb();
  void requestSquareMapping();
  void process();

private:
  void obstacle_filter();
  void modify_inflation_for_near_obs();
  void modify_inflation_for_obs_near_target();
  void get_static_obstacle();
  void modify_inflation_for_car();
  void ignore_obstacle_box_near_target();
  // void add_freespace_obstacle();
  void get_freespace_lead();

  bool init_frame_AVP();
  bool gather_map_info_avp();

  Pose2D local_frame_pose_;
  TrajectoryPoint ego_state_local_;
  TrajectoryPoint target_state_local_;

  double x_bound_lower_;
  double x_bound_upper_;
  double y_bound_lower_;
  double y_bound_upper_;

  double default_swell_distance_;
  double default_step_size_;
};

} // namespace parking
} // namespace msquare

#endif
