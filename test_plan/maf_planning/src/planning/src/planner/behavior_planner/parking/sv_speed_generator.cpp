#include "planner/behavior_planner/parking/sv_speed_generator.h"
#include "common/config/vehicle_param.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/local_log.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"

namespace msquare {
namespace parking {

double SvSpeedGenerator::getSpeed(const planning_math::Box2d &slot_box,
                                  const Pose2D &ego_pose,
                                  const std::vector<float> &traj_curvature,
                                  const SvSpeedParam &sv_p, bool &is_in_slot) {
  double velocity =
      regulateSpeed(slot_box, ego_pose, traj_curvature, sv_p, is_in_slot);
  if (sv_p.is_reverse) {
    velocity = -velocity;
  }
  return velocity;
}
double SvSpeedGenerator::regulateSpeed(const planning_math::Box2d &slot_box,
                                       const Pose2D &ego_pose,
                                       const std::vector<float> &traj_curvature,
                                       const SvSpeedParam &sv_p,
                                       bool &is_in_slot) {
  double velocity = 0.0;
  if (sv_p.has_ever_been_inside_slot) {
    is_in_slot = true;
  }
  if (sv_p.force_stop) {
    return velocity;
  }

  if (sv_p.has_ever_been_inside_slot) {
    LOCAL_LOG(LOCAL_DEBUG, "has_ever_been_inside_slot");
    velocity = sv_p.max_v * sv_p.in_slot_coeff;
    is_in_slot = true;
    return velocity;
  }

  //  in slot
  if (!sv_p.no_slot) {
    planning_math::Vec2d ego_center(ego_pose.x, ego_pose.y);
    planning_math::Box2d ego_box(ego_center, ego_pose.theta, sv_p.length,
                                 sv_p.width_wo_rearview_mirror);
    bool is_overlap = ego_box.HasOverlap(slot_box);
    if (is_overlap) {
      velocity = sv_p.max_v * sv_p.in_slot_coeff;
      is_in_slot = true;
      return velocity;
    }
  }
  // reverse
  if (sv_p.is_reverse) {
    velocity = sv_p.max_v * sv_p.large_curv_coeff;
    is_in_slot = false;
    return velocity;
  }

  //  curvature
  if (!traj_curvature.empty()) {
    bool is_curv_limit = traj_curvature[0] > sv_p.curv_limit;
    if (!is_curv_limit && traj_curvature.size() > 1) {
      is_curv_limit = traj_curvature[1] > sv_p.curv_limit;
    }
    if (is_curv_limit) {
      velocity = sv_p.max_v * sv_p.large_curv_coeff;
      is_in_slot = false;
      return velocity;
    }
  }

  // out slot
  is_in_slot = false;
  velocity = sv_p.max_v * sv_p.out_slot_coeff;
  return velocity;
  ;
}

} // namespace parking
} // namespace msquare