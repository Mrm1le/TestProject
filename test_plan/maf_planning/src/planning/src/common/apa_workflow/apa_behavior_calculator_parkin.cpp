#include "common/apa_workflow/apa_behavior_calculator_parkin.h"

namespace msquare {
namespace parking {

using planning_math::Box2d;
using planning_math::Polygon2d;
using planning_math::Vec2d;

APABehaviorCalculatorParkIn::APABehaviorCalculatorParkIn(
    const std::shared_ptr<WorldModel> &world_model)
    : world_model_(world_model) {}

APABehaviorCalculatorParkIn::~APABehaviorCalculatorParkIn() {}

bool APABehaviorCalculatorParkIn::calculate_approaching_wheel_stop(
    double distance_to_end, bool openspace_is_running) {
  // ################## Rule Explanation ##################
  // [Default Value]
  //    ---> Rule: approaching_wheel_stop = false

  // [Base Scene] NonParallel R-gear Traj Following
  //    1. gear == REVERSE
  //    2. (AND) slot.type != PARALLEL
  //    3. (AND) openspace_statemachine == RUNNING
  //    4. (AND) non_pause

  // [Sub Scene 1] Has Wheel Stop
  //    ---> Rule : approaching_wheel_stop = distance_to_end < 1.0;

  // [Sub Scene 2] No Wheel Stop
  //    ---> Rule : approaching_wheel_stop = distance_to_end <
  //    0.5*vehicle_length
  //

  // ############## End of Rule Explanation ###############

  //[PlanningContext Input]
  const auto &g_planning_gear =
      PlanningContext::Instance()->planning_status().planning_result.gear;

  const auto &g_planning_status =
      PlanningContext::Instance()->planning_status();

  const auto &g_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;
  const auto &g_wheel_stop_info_available =
      PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .parking_slot_info.wheel_stop_info.available;

  //[Calculation]
  bool base_scene = g_planning_gear == GearState::REVERSE &&
                    g_slot_info.type.value != ParkingSlotType::PARALLEL &&
                    openspace_is_running && !world_model_->get_pause_status();

  bool result_approaching_wheel_stop = false;
  if (!base_scene) {
    return result_approaching_wheel_stop;
  }

  if (g_wheel_stop_info_available) {
    result_approaching_wheel_stop = distance_to_end < 1.0;
  } else {
    result_approaching_wheel_stop =
        distance_to_end < VehicleParam::Instance()->length / 2.0;
  }

  return result_approaching_wheel_stop;
}

bool APABehaviorCalculatorParkIn::calculate_blocked_base_scene(
    bool openspace_is_running) {
  // ################## Rule Explanation ##################

  // [Base Scene] Blocked
  //    1. non-pause
  //    2. planning_status.blocked_timeout
  //    3. (AND) planning_status.blocked
  //    4. (AND) openspace_statemachine == RUNNING
  // [PlanningContext Input]
  const auto &g_status_blocked_timeout =
      PlanningContext::Instance()->planning_status().blocked_timeout;

  const auto &g_status_blocked =
      PlanningContext::Instance()->planning_status().blocked;

  //[Calculation]
  bool base_scene = !world_model_->get_pause_status() &&
                    g_status_blocked_timeout && g_status_blocked &&
                    openspace_is_running;
  return base_scene;
}

bool APABehaviorCalculatorParkIn::calculate_hit_sth_base_scene(
    bool openspace_is_running) {
  // ################## Rule Explanation ##################

  // [Base Scene] Blocked
  //    1. non-pause
  //    2. hit unseen obstacle
  //    4. (AND) openspace_statemachine == RUNNING
  // [PlanningContext Input]

  const auto &g_planning_status =
      PlanningContext::Instance()->planning_status();

  //[Calculation]
  bool hit_something = (world_model_->get_collide_to_limiter_when_reverse() &&
                        !g_planning_status.stopping) ||
                       g_planning_status.collide_to_sth;

  bool base_scene = !world_model_->get_pause_status() && hit_something &&
                    openspace_is_running;
  return base_scene;
}

bool APABehaviorCalculatorParkIn::calculate_times_try_parkin(
    bool base_scene, bool is_at_last_segment,
    std::size_t *ptr_times_try_parking_in) {
  // ################## Rule Explanation ##################

  // [Base Scene] Blocked OR HitSth
  //    1. base_scene

  // [Sub Scene 1]
  //    1. has_moved
  //    2. is_at_last_segment
  //    3. is_ego_overlaps_lot
  //    ---> Rule : times_try_parking_in = g_times_try_parking_in +1

  // ############## End of Rule Explanation ###############

  // [PlanningContext Input]
  const auto &g_times_try_parking_in = *ptr_times_try_parking_in;

  const auto &g_has_moved =
      PlanningContext::Instance()->parking_behavior_planner_output().has_moved;

  const auto &g_parking_lot = PlanningContext::Instance()
                                  ->parking_behavior_planner_output()
                                  .parking_lot;

  const auto &ego_pose = world_model_->get_ego_state().ego_pose;

  //[Calculation]
  std::size_t result_times_try_parking_in = g_times_try_parking_in;

  if (!base_scene) {
    return false;
  }

  Box2d slot_box = g_parking_lot->getBox();
  bool is_ego_overlaps_lot = slot_box.IsPointIn(Vec2d(ego_pose.x, ego_pose.y));

  // sub scene 1
  if (g_has_moved && is_ego_overlaps_lot && is_at_last_segment) {
    result_times_try_parking_in += 1;
  }

  // output
  *ptr_times_try_parking_in = result_times_try_parking_in;
  return true;
}

bool APABehaviorCalculatorParkIn::calculate_blocked_by_obstacle_behind_in_slot(
    bool blocked_base_scene, const Pose2D &target_pose_lot) {
  // ################## Rule Explanation ##################
  // [Default Value]
  //    ---> Rule: is_blocked_by_obstacle_behind_in_slot = false

  // [Base Scene] Blocked
  //    1. planning_status.blocked_timeout
  //    2. (AND) planning_status.blocked
  //    3. (AND) openspace_statemachine == RUNNING
  //    4. (TODO: ) non_pause ?
  //    All these calculated in calculate_blocked_base_scene

  // ############## End of Rule Explanation ###############

  //[PlanningContext Input]
  const auto &ego_pose = world_model_->get_ego_state().ego_pose;

  //[Calculation]
  bool result_is_blocked_by_obstacle_behind_in_slot = false;

  if (!blocked_base_scene) {
    return false;
  }

  result_is_blocked_by_obstacle_behind_in_slot = isGroundlineBehindEgoUnsafe(
      world_model_->get_parking_ground_line_fusion(), ego_pose,
      target_pose_lot);

  return result_is_blocked_by_obstacle_behind_in_slot;
}

bool APABehaviorCalculatorParkIn::calculate_need_update_parking_slot_corners(
    const Pose2D &target_pose_lot) {
  // ################## Rule Explanation ##################
  // [Default Value]
  //    ---> Rule: if need stop update parking slot corners(default is true)

  // [Base Scene] need update
  //    1. pause state
  //    2. (OR) distance from ego_pose to target > 3.0m
  //    3. (OR) not at REVERSE gear
  //    4. (OR) parallel slot

  //

  // ############## End of Rule Explanation ###############
  const auto &g_planning_gear =
      PlanningContext::Instance()->planning_status().planning_result.gear;
  const auto &g_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;
  const auto &ego_pose = world_model_->get_ego_state().ego_pose;

  double distance_ego_to_target = std::hypot(target_pose_lot.x - ego_pose.x,
                                             target_pose_lot.y - ego_pose.y);

  bool base_scene = world_model_->get_pause_status() ||
                    distance_ego_to_target > 3.0 ||
                    g_planning_gear != GearState::REVERSE ||
                    g_slot_info.type.value == ParkingSlotType::PARALLEL;
  return base_scene;
}

bool APABehaviorCalculatorParkIn::calculate_need_check_mpc_collide(
    const bool need_update_slot_corners, bool is_at_last_segment) {
  // ################## Rule Explanation ##################
  // [Default Value]
  //    ---> Rule: if need stop update parking slot corners(default is true)

  // [Base Scene] Non-Parallel Last 3.0m
  //    1. not Need_lock_parking_lot
  //    2. (AND) has_moved
  //    3. (AND) is_at_last_segment

  //

  // ############## End of Rule Explanation ###############
  const auto &g_has_moved =
      PlanningContext::Instance()->parking_behavior_planner_output().has_moved;

  bool base_scene =
      !need_update_slot_corners && g_has_moved && is_at_last_segment;
  return base_scene;
}

bool APABehaviorCalculatorParkIn::calculate_need_dynamic_plan_adjust_tail(
    const bool need_check_mpc_collide) {
  // ################## Rule Explanation ##################
  // [Default Value]
  //    ---> Rule: if need stop update parking slot corners(default is true)

  // [Base Scene] Non-Parallel Last 3.0m
  //    1. need_check_mpc_collide
  //    2. (AND) vision_wheel_stop_available

  //

  // ############## End of Rule Explanation ###############
  const auto &g_is_wheel_stopper_vision_available =
      PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .parking_slot_info.wheel_stop_info.vision_wheel_stop_available;

  bool base_scene =
      need_check_mpc_collide && g_is_wheel_stopper_vision_available;
  return base_scene;
}

bool APABehaviorCalculatorParkIn::isPointBehindEgoUnsafe(
    const planning_math::Vec2d &point, const Pose2D &ego_pose,
    const Pose2D &target_pose) {
  double lon_inflation = CarParams::GetInstance()->lon_inflation();
  double lat_inflation = CarParams::GetInstance()->lat_inflation();
  // behind ego?
  planning_math::Vec2d point_in_ego_frame =
      planning_math::tf2d(ego_pose, point);
  if (point_in_ego_frame.y() <
          -(VehicleParam::Instance()->width + lat_inflation) ||
      point_in_ego_frame.y() >
          (VehicleParam::Instance()->width + lat_inflation) ||
      point_in_ego_frame.x() > -VehicleParam::Instance()->back_edge_to_center ||
      point_in_ego_frame.x() <
          -VehicleParam::Instance()->back_edge_to_center - 2 * lon_inflation)
    return false;

  return true;
}

bool APABehaviorCalculatorParkIn::isGroundlineBehindEgoUnsafe(
    const std::vector<GroundLine> &groundline, const Pose2D &ego_pose,
    const Pose2D &target_pose) {
  MSD_LOG(INFO, "[short_block] %s-%d", __FUNCTION__, __LINE__);
  for (auto &obs : groundline) {
    if (obs.id != PlanningContext::Instance()
                      ->parking_behavior_planner_output()
                      .parking_slot_info.id ||
        (int)obs.type < 100 || (int)obs.type > 150) {
      continue;
    }
    for (int i = 0; i < obs.pts.size(); i++) {
      planning_math::Vec2d point{obs.pts[i].x, obs.pts[i].y};
      if (isPointBehindEgoUnsafe(point, ego_pose, target_pose)) {
        MSD_LOG(INFO, "[short_block] %s-%d", __FUNCTION__, __LINE__);
        return true;
      }
    }
  }
  return false;
}

} // namespace parking
} // namespace msquare
