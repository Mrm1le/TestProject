#include "common/apa_workflow/apa_behavior_decider_parkin.h"
#include "common/planning_config.h"
#include "common/utils/timer.h"

namespace msquare {
namespace parking {

namespace {
constexpr double kDeadZonePathLength = 0.1;
constexpr double kRemainDistFromPauseReplan = 0.4;
}

using planning_math::Box2d;
using planning_math::Polygon2d;
using planning_math::Vec2d;

APABehaviorDeciderParkIn::APABehaviorDeciderParkIn(
    const std::shared_ptr<WorldModel> &world_model,
    const std::shared_ptr<ParkingLongitudinalBehaviorPlanner>
        &parking_longitudinal_behavior_planner)
    : world_model_(world_model), parking_longitudinal_behavior_planner_(
                                     parking_longitudinal_behavior_planner),
      dynamic_plan_mpc_block_timer_(1.0) {

  YAML::Node state_parking_config_node = YAML::LoadFile(
      PlanningConfig::Instance()->config_files().state_parking_config_file);
  state_parking_cfg_.apa_simple_mpc_entry_threshold_ =
      state_parking_config_node["apa_simple_mpc_entry_threshold"].as<double>();
  const double DYNAMIC_PLAN_MPC_BLOCK_DURATION =
      CarParams::GetInstance()
          ->car_config.parkin_decider_config.curve_join_block_duration;
  dynamic_plan_mpc_block_timer_.set_timer_duration(DYNAMIC_PLAN_MPC_BLOCK_DURATION);
}

APABehaviorDeciderParkIn::~APABehaviorDeciderParkIn() {}

APABehaviorDeciderParkIn::DeciderResult
APABehaviorDeciderParkIn::strategy_hit_wheelstop(bool openspace_is_running,
                                           bool vehicle_reached_no_wheelstop) {
  // ################## Strategy Explanation ##################
  // [Base Scene] NonParallel R-gear Traj Following HIT something
  //    1. gear == REVERSE
  //    2. (AND) slot.type != PARALLEL
  //    3. (AND) openspace_statemachine == RUNNING
  //    4. (AND) non_pause
  //    5. (AND) HIT SOMETHING
  //      5.1 collde_to_limiter_when_reverse && planning_status != stopping
  //      5.2 (OR) planning_status == collide_to_sth

  // [Sub Scene 1] Match Finish Condition
  //    1. reached target
  //    2. (OR) is WLC slot
  //    ---> Strategy : Finish

  // [Sub Scene 2] others
  //    ---> Strategy : Replan
  // ############## End of Strategy Explanation ###############

  //[PlanningContext Input]

  const auto &g_planning_gear =
      PlanningContext::Instance()->planning_status().planning_result.gear;

  const auto &g_planning_status =
      PlanningContext::Instance()->planning_status();

  const auto &g_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;

  const auto &g_is_fusion_wlc_property_valid =
      PlanningContext::Instance()
          ->planning_status()
          .wlc_info.is_fusion_wlc_property_valid;

  const auto &g_times_try_parking_in_about_hit_sth =
      PlanningContext::Instance()
          ->openspace_motion_planner_output()
          .times_try_parking_in_about_hit_sth;

  //[Strategy]
  DeciderResult result;
  result.strategy = APAStrategyType::NONE;

  bool hit_something = (world_model_->get_collide_to_limiter_when_reverse() &&
                        !g_planning_status.stopping) ||
                       g_planning_status.collide_to_sth;

  bool base_scene = g_planning_gear == GearState::REVERSE &&
                    g_slot_info.type.value != ParkingSlotType::PARALLEL &&
                    openspace_is_running && !world_model_->get_pause_status() &&
                    hit_something;

  if (!base_scene) {
    return result;
  }

  bool finish_condition =
      vehicle_reached_no_wheelstop || g_is_fusion_wlc_property_valid;

  if (finish_condition) {
    result.strategy = APAStrategyType::FINISH;
    result.reason = "finish because of hit wheel_stop (collide_to_sth)";

  } else {
    if (g_times_try_parking_in_about_hit_sth < 2) {
      result.strategy = APAStrategyType::REPLAN;
      result.reason = "replan cause hit wheel_stop" +
                      std::to_string(g_times_try_parking_in_about_hit_sth) +
                      "times(collide_sth), but "
                      "not reach target pose";
    } else {
      result.strategy = APAStrategyType::FINISH;
      result.reason = "finish because of hit wheel_stop (collide_to_sth), more than 2 times";
    }
  }

  // debug
  *PlanningContext::Instance()->mutable_planning_debug_info() +=
      "[sm-hit]times" + std::to_string(g_times_try_parking_in_about_hit_sth) +
      ",hit(" +
      std::to_string(world_model_->get_collide_to_limiter_when_reverse()) +
      "," + std::to_string(hit_something) + "), ";

  return result;
}

APABehaviorDeciderParkIn::DeciderResult
APABehaviorDeciderParkIn::strategy_wheelstop_in_parallel_slot(
    bool openspace_is_running) {
  // ################## Strategy Explanation ##################
  // [Base Scene] PARALLEL && Running
  //    1. no pause
  //    2. slot.type != PARALLEL
  //    3. (AND) openspace_statemachine == RUNNING

  // [Sub Scene 1] Wheelstop Detected by Collide
  //    1. get_collide_to_limiter_when_reverse
  //    ---> Strategy : Abandon
  // [Sub Scene 2] Wheelstop Detected by PSD
  //    1. slot_info.wheel_stop_info.available
  //    ---> Strategy : Abandon

  //[PlanningContext Input]
  const auto &g_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;

  //[Strategy]
  DeciderResult result;
  result.strategy = APAStrategyType::NONE;

  bool base_scene = !world_model_->get_pause_status() &&
                    g_slot_info.type.value == ParkingSlotType::PARALLEL &&
                    openspace_is_running;

  if (!base_scene) {
    return result;
  }

  bool wheelstop_detected_by_collide =
      world_model_->get_collide_to_limiter_when_reverse();
  bool wheelstop_detected_by_psd = g_slot_info.wheel_stop_info.available;

  if (wheelstop_detected_by_collide) {
    result.strategy = APAStrategyType::ABANDON;
    result.reason = "abandon because of detected wheelstop by collide in "
                    "parallel slot(get_collide_to_limiter_when_reverse) ";

  } else if (wheelstop_detected_by_psd) {
    result.strategy = APAStrategyType::ABANDON;
    result.reason =
        "abandon because of PSD detected wheelstop in parallel slot";
  }

  return result;
}

APABehaviorDeciderParkIn::DeciderResult
APABehaviorDeciderParkIn::strategy_blocked_and_reached(
    bool vehicle_reached_with_wheelstop) {
  // ################## Strategy Explanation ##################
  // [Base Scene] Blocked && Reached target
  //    1. not pause
  //    2. (AND) not finish
  //    3. (AND) blocked_timeout
  //    4. (AND) vehicle_reached_slot_with_wheelstop
  //    5. (AND) vehicle is static

  // [Sub Scene 1] Normal APA (WLC disabled_
  //    1. wlc_info.is_valid
  //    ---> Strategy : Abandon
  // [Sub Scene 2] (Other) WLC enabled
  //    ---> Strategy : WAIT_FOR_WLC_CONFIRM

  //[PlanningContext Input]
  const auto &g_wlc_info =
      PlanningContext::Instance()->planning_status().wlc_info;
  const auto &g_is_finish = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->is_finish;

  const auto &g_blocked_timeout =
      PlanningContext::Instance()->planning_status().blocked_timeout;

  //[Strategy]
  DeciderResult result;
  result.strategy = APAStrategyType::NONE;

  bool base_scene = !world_model_->get_pause_status() && !g_is_finish &&
                    g_blocked_timeout && vehicle_reached_with_wheelstop &&
                    world_model_->get_ego_state().is_static;

  if (!base_scene) {
    return result;
  }

  bool wlc_enabled = g_wlc_info.is_valid;

  if (!wlc_enabled) {
    result.strategy = APAStrategyType::FINISH;
    result.reason = "finish normally after blocked and reached target";

  } else {
    result.strategy = APAStrategyType::WAIT_FOR_WLC;
    result.reason = "WLC approached, wait for confirm before finish";
  }

  return result;
}

APABehaviorDeciderParkIn::DeciderResult
APABehaviorDeciderParkIn::strategy_tiny_perpendicular_and_oblique_slot(bool is_at_last_segment) {
  // ################## Strategy Explanation ##################
  // [Base Scene] (Perpendicular || Oblique) && next-to-car && last_segment
  //    1. not pause
  //    2. (AND) slot.type == PERPENDICULAR || OBLIQUE
  //    3. (AND) is_at_last_segment
  //    4. (AND)  slot is one_side_car || two_side_car

  // [Sub Scene 1] Tiny Slot
  //    1. (AND) ego_center inside slot polygon
  //    2. (AND) slot_width < threshold
  //    3. (AND) conditions 1-3 count in continuous 2 frames
  //    ---> Strategy : Abandon

  //[PlanningContext Input]
  const auto &g_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;

  //[Strategy]
  DeciderResult result;
  result.strategy = APAStrategyType::NONE;

  bool is_left_car = false;
  bool is_right_car = false;
  checkTwoSides(is_left_car, is_right_car);
  bool is_two_side_car = is_left_car && is_right_car;
  bool is_one_side_car =
      (is_left_car && !is_right_car) || (!is_left_car && is_right_car);

  bool base_scene = !world_model_->get_pause_status() &&
                    g_slot_info.type.value != ParkingSlotType::PARALLEL &&
                    is_at_last_segment && (is_two_side_car || is_one_side_car);

  if (!base_scene) {
    times_tiny_slot_overlap_ = 0;
    return result;
  }

  double MIN_SLOT_WIDTH_TH =
      is_one_side_car ? CarParams::GetInstance()
                            ->car_config.target_pose_config
                            .oneside_narrowest_width_advanced_abandon
                      : CarParams::GetInstance()
                            ->car_config.target_pose_config
                            .twosides_narrowest_width_advanced_abandon;
  bool is_overlap = false;
  double slot_width = MIN_SLOT_WIDTH_TH * 2.0;

  MSD_LOG(ERROR,
          "%s:min_slot_width:%.3f, is_two_side_car:%d, is_one_side_car:%d",
          __FUNCTION__, MIN_SLOT_WIDTH_TH, is_two_side_car, is_one_side_car);
  const auto &ego_pose = world_model_->get_ego_state().ego_pose;
  double center_offset = VehicleParam::Instance()->center_to_geometry_center;
  double xx = ego_pose.x + center_offset * cos(ego_pose.theta);
  double yy = ego_pose.y + center_offset * sin(ego_pose.theta);
  Vec2d ego_center = Vec2d(xx, yy);
  if (g_slot_info.corners.size() == 4) {
    auto &corners = g_slot_info.corners;
    slot_width = std::hypot(
        0.5 * (corners[0].x + corners[1].x - corners[2].x - corners[3].x),
        0.5 * (corners[0].y + corners[1].y - corners[2].y - corners[3].y));

    std::vector<Vec2d> points;
    for (auto &corner : corners) {
      points.emplace_back(corner.x, corner.y);
    }
    Polygon2d slot_polygon(points);
    if (slot_polygon.IsPointIn(ego_center)) {
      is_overlap = true;
    }
  }
  slot_width = std::max(slot_width, 0.0);

  if (slot_width < MIN_SLOT_WIDTH_TH && is_overlap) {
    times_tiny_slot_overlap_ += 1;
  } else {
    times_tiny_slot_overlap_ = 0;
  }
  if (times_tiny_slot_overlap_ > 1) {
    result.strategy = APAStrategyType::ABANDON;
    result.reason = "abandon as tiny perpendicular||oblique slot :" +
                    std::to_string(MIN_SLOT_WIDTH_TH);

    MSD_LOG(ERROR,
            "%s: abandon because slot width %.3f is smaller than "
            "min_slot_width %.3f, and ego center is in with planning "
            "slot in continuous two times",
            __FUNCTION__, slot_width, MIN_SLOT_WIDTH_TH);
  }

  return result;
}

bool APABehaviorDeciderParkIn::checkTwoSides(bool &is_left_car, bool &is_right_car) {
  auto &parking_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;
  auto &corners = parking_slot_info.corners;
  if (corners.size() != 4) {
    return false;
  }
  double fx = 0.5 * (corners[0].x + corners[3].x);
  double fy = 0.5 * (corners[0].y + corners[3].y);
  double bx = 0.5 * (corners[1].x + corners[2].x);
  double by = 0.5 * (corners[1].y + corners[2].y);
  double cx = 0.5 * (fx + bx);
  double cy = 0.5 * (fy + by);
  double ctheta = atan2(fy - by, fx - bx);
  double lot_length = std::hypot(fy - by, fx - bx);
  double lot_width =
      std::hypot(corners[0].x - corners[3].x, corners[0].y - corners[3].y);

  Box2d slot_box =
      Box2d(Vec2d(cx, cy), ctheta, 0.5 * lot_length, 0.5 * lot_width);
  Box2d slot_left(slot_box);
  Box2d slot_right(slot_box);

  Vec2d left_vector(corners[0].x - corners[3].x, corners[0].y - corners[3].y);
  slot_left.Shift(left_vector);
  slot_right.Shift(-1.0 * left_vector);

  for (auto &obj : world_model_->obstacle_manager().get_obstacles().Items()) {
    bool is_obstacle_vehicle = obj->Type() == ObjectType::COUPE ||
                               obj->Type() == ObjectType::BUS ||
                               obj->Type() == ObjectType::ENGINEER_TRUCK ||
                               obj->Type() == ObjectType::TRICYCLE;
    if (!is_obstacle_vehicle) {
      continue;
    }

    const Box2d &car_box = obj->PerceptionBoundingBox();

    if (!is_left_car) {
      is_left_car = slot_left.HasOverlap(car_box);
    }
    if (!is_right_car) {
      is_right_car = slot_right.HasOverlap(car_box);
    }
  }

  return true;
}

APABehaviorDeciderParkIn::DeciderResult
APABehaviorDeciderParkIn::strategy_dynamic_plan_adjust_tail(
    bool need_dynamic_plan_adjust_tail) {
  // ################## Strategy Explanation ##################
  // [Base Scene] Non-Parallel Last 3.0m
  //    1. not pause
  //    2. (AND) gear == REVERSE
  //    3. (AND) slot.type != PARALLEL
  //    2. (AND) distance_to_end <= 3.0
  //    3. (AND) has_moved
  //    4. (AND) is_at_last_segment
  //    5. (AND) wheelstop info vision available
  //    all these calculated in calculate_need_dynamic_plan_adjust_tail

  //    ---> Strategy: DYNAMIC_PLAN_ADJUST_TAIL(lenthen_or_shorten)

  //[Strategy]
  DeciderResult result;
  result.strategy = APAStrategyType::NONE;
  if (!need_dynamic_plan_adjust_tail) {
    return result;
  }

  result.strategy = APAStrategyType::DYNAMIC_PLAN_ADJUST_TAIL;
  result.reason = "vintage dynamic plan adjusting trajactory tail according to "
                  "wheel-stop position(lenthen_or_shorten)";

  return result;
}

APABehaviorDeciderParkIn::DeciderResult
APABehaviorDeciderParkIn::strategy_dynamic_plan_simple_mpc(
    double distance_to_end, bool is_at_last_segment,
    bool openspace_is_running) {

  //[fenix.refactor]
  //   当前的dynamic_planning_simple_mpc是否激活的逻辑非常混乱，同时存在一系列神奇的特性，
  //   为了无损还原，这里保留了原来的实现，不予解释。
  //  后续替换成干净的实现

  const auto &g_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;

  const auto &g_parking_lot = PlanningContext::Instance()
                                  ->parking_behavior_planner_output()
                                  .parking_lot;

  const auto &g_planning_gear =
      PlanningContext::Instance()->planning_status().planning_result.gear;

  const auto &g_has_moved =
      PlanningContext::Instance()->parking_behavior_planner_output().has_moved;

  const double dist_to_stop_curvejoin =
      CarParams::GetInstance()
          ->car_config.parkin_decider_config.dist_to_stop_curvejoin;

  if (!world_model_->get_pause_status() &&
      distance_to_end <= dist_to_stop_curvejoin &&
      g_planning_gear == GearState::REVERSE &&
      g_slot_info.type.value != ParkingSlotType::PARALLEL) {
    is_dynamic_planning_activated_ = false;
  }

  //[fenix.refactor] 是否真的执行dynamic planning
  //和is_dynamic_planning_activated 不完全对应
  bool do_dynamic_planning = false;

  if (openspace_is_running) {
    if (is_dynamic_planning_activated_ && !world_model_->get_pause_status()) {
      do_dynamic_planning = true;
    } else {
      // calculate centerline_follow_error
      double centerline_follow_error = std::numeric_limits<double>::infinity();

      const auto &ego_pose = world_model_->get_ego_state().ego_pose;
      std::vector<TrajectoryPoint> center_line_traj =
          g_parking_lot->getCenterLine(
              VehicleParam::Instance()->front_edge_to_center,
              VehicleParam::Instance()->back_edge_to_center,
              VehicleParam::Instance()->brake_distance_buffer,
              g_slot_info.wheel_stop_info.wheel_stop_depth);

      const Box2d &slot_box = PlanningContext::Instance()
                                  ->mutable_parking_behavior_planner_output()
                                  ->parking_lot->getBox();
      if (world_model_->get_ego_state().ego_box.HasOverlap(slot_box)) {
        double nearest_point_distance = std::numeric_limits<double>::max();
        for (auto &pt :
             PlanningContext::Instance()->openspace_decider_output().points) {
          nearest_point_distance =
              std::min(nearest_point_distance, slot_box.DistanceTo(pt));
        }
        auto nearest_pose_iter = std::min_element(
            center_line_traj.begin(), center_line_traj.end(),
            [ego_pose](const TrajectoryPoint &a, const TrajectoryPoint &b) {
              double dist_a = std::hypot(ego_pose.x - a.path_point.x,
                                         ego_pose.y - a.path_point.y);
              double dist_b = std::hypot(ego_pose.x - b.path_point.x,
                                         ego_pose.y - b.path_point.y);
              return dist_a < dist_b;
            });
        if (nearest_pose_iter != center_line_traj.end()) {
          centerline_follow_error =
              std::hypot(ego_pose.x - nearest_pose_iter->path_point.x,
                         ego_pose.y - nearest_pose_iter->path_point.y);
          MSD_LOG(WARN, "%s: centerline_follow_error = %f", __FUNCTION__,
                  centerline_follow_error);
        }
      }

      bool is_reached_slot_center_line =
          centerline_follow_error <
          state_parking_cfg_.apa_simple_mpc_entry_threshold_;

      if (g_has_moved && is_reached_slot_center_line && is_at_last_segment &&
          g_planning_gear == GearState::REVERSE) {
        is_dynamic_planning_activated_ = true;
      }
    }
  } else {
    is_dynamic_planning_activated_ = false;
  }

  //[Strategy]
  DeciderResult result;
  result.strategy = APAStrategyType::NONE;

  if (do_dynamic_planning) {
    result.strategy = APAStrategyType::DYNAMIC_PLAN_SIMPLE_MPC;
    result.reason = "dynamic plan when reached slot center line in last "
                    "segment (non-parallel)";
  }

  return result;
}

APABehaviorDeciderParkIn::DeciderResult
APABehaviorDeciderParkIn::strategy_dynamic_plan_simple_mpc_collide_timeout(
    bool dynamic_plan_simple_mpc_enabled,
    bool dynamic_plan_simple_mpc_traj_success,
    bool dynamic_plan_simple_mpc_traj_collide) {

  //[Fenix] this logic is really wierd, TODO: renew logics

  // ################## Strategy Explanation ##################
  // [Base Scene] dynamic_plan_simple_mpc enabled AND
  //    1. dynamic_plan_simple_mpc_enabled
  //    2. (AND) dynamic_plan_simple_mpc_traj_success  (generated traj)

  // [Sub Scene 1] collide timeout
  //    1. dynamic_plan_simple_mpc_collide (generated traj but collide)
  //    2. dynamic_plan_mpc_block_timer_ timeout
  //      Timer reset when :(Wierd logic)
  //        2.1 : dynamic_plan_simple_mpc_enabled
  //        2.2 : (AND) dynamic_plan_simple_mpc_traj_success
  //        2.3 : (AND) !dynamic_plan_simple_mpc_traj_collide

  //    ---> Strategy: REPLAN

  //[Strategy]
  DeciderResult result;
  result.strategy = APAStrategyType::NONE;

  bool base_scene =
      dynamic_plan_simple_mpc_enabled && dynamic_plan_simple_mpc_traj_success;

  if (!base_scene) {
    return result;
    // TODO: add timer reset ?
  }

  if (!dynamic_plan_simple_mpc_traj_collide) {
    dynamic_plan_mpc_block_timer_.reset();
  }

  if (dynamic_plan_mpc_block_timer_.is_timeout()) {
    result.strategy = APAStrategyType::REPLAN;
    result.reason = "replan for dynamic planning block timer";
  }

  return result;
}

APABehaviorDeciderParkIn::DeciderResult
APABehaviorDeciderParkIn::strategy_mpc_collide(bool need_check_mpc_collide) {
  // ################## Strategy Explanation ##################
  // [Base Scene] Non-Parallel Last 3.0m
  //    1. not pause
  //    2. (AND) gear == REVERSE
  //    3. (AND) slot.type != PARALLEL
  //    2. (AND) distance_to_end <= 3.0
  //    3. (AND) has_moved
  //    4. (AND) is_at_last_segment
  // all these calculated in calculate_need_check_mpc_collide

  // [Sub Scene 1]
  //    1. MPC traj collide
  //    ---> Strategy : Replan

  //[Strategy]
  DeciderResult result;
  result.strategy = APAStrategyType::NONE;

  const double MPC_BLOCK_DURATION =
      CarParams::GetInstance()
          ->car_config.parkin_decider_config.mpc_traj_block_duration;
  static Timer mpc_block_timer(MPC_BLOCK_DURATION);

  if (!need_check_mpc_collide) {
    // TOOD: add timer reset?
    return result;
  }

  is_dynamic_planning_activated_ = false;
  std::vector<Pose2D> end_curve;
  const Box2d &slot_box = PlanningContext::Instance()
                              ->mutable_parking_behavior_planner_output()
                              ->parking_lot->getBox();
  const Box2d parking_lot_box_bottom =
      Box2d(slot_box.center() - 0.5 * slot_box.length() *
                                    Vec2d::CreateUnitVec2d(slot_box.heading()),
            slot_box.heading(), slot_box.length(), slot_box.width() * 2.0);

  bool is_curve_collide = false;
  auto groundline_pts = world_model_->get_parking_ground_line_fusion();
  auto checker =
      parking_longitudinal_behavior_planner_->get_collision_checker();
  double obs_to_ego_square;
  double collision_threshold;
  double extra_thres;
  double ratio;
  bool use_secure;
  ObsPtsWithId obs_pts;
  for (const auto &obs : groundline_pts) {
    if (obs.id != PlanningContext::Instance()
                      ->parking_behavior_planner_output()
                      .parking_slot_info.id ||
        ((int)obs.type > 100 && (int)obs.type < 150)) {
      continue;
    }
    for (int i = 0; i < obs.pts.size(); i++) {
      if (slot_box.IsPointIn(Vec2d(obs.pts[i].x, obs.pts[i].y)) ||
          parking_lot_box_bottom.IsPointIn(Vec2d(obs.pts[i].x, obs.pts[i].y))) {
        continue;
      }
      planning_math::Vec2d obs_p(obs.pts[i].x, obs.pts[i].y, obs.id);
      collision_threshold = CarParams::GetInstance()->lat_inflation() * 0.5;
      extra_thres = std::min(CarParams::GetInstance()->lat_inflation() * 0.5,
                             CarParams::GetInstance()->lon_inflation_min - 0.1);
      obs_pts.emplace_back(collision_threshold, extra_thres,
                           CollisionCheckStatus(), obs_p, 1.0,
                           Pose2D(0.0, 0.0, 0.0, 0.0));
    }
  }
  const Pose2DTrajectory &mpc_traj =
      PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .mpc_trajectory;
  // obs_pts is obstacle points which ,
  const auto &ego_pose = world_model_->get_ego_state().ego_pose;
  FreespacePoint lead_point;
  std::string debug_str;
  std::vector<std::pair<double, double>> old_mpc_sl_points;
  checker.remainDisCheck(obs_pts, mpc_traj, end_curve, true, true, ego_pose,
                         &lead_point, &debug_str, &old_mpc_sl_points);
  for (const auto &obs_result : obs_pts) {
    const CollisionCheckStatus &result = obs_result.result;
    if (!result.is_collision || !result.is_valid) {
      continue;
    }
    double traj_length = PlanningContext::Instance()
                             ->longitudinal_behavior_planner_output()
                             .traj_length;
    if (result.s > 0.0 && result.s < std::min(2.0, traj_length) &&
        !world_model_->get_ego_state().is_static) {
      is_curve_collide = true;
      break;
    }
  }

  if (!is_curve_collide) {
    mpc_block_timer.reset();
  }

  if (mpc_block_timer.is_timeout()) {
    result.strategy = APAStrategyType::REPLAN;
    result.reason = "replan for potential block of mpc trajectory ";
  }

  return result;
}

APABehaviorDeciderParkIn::DeciderResult
APABehaviorDeciderParkIn::strategy_openspace_fallback(bool openspace_is_fallback) {
  // ################## Strategy Explanation ##################
  // [Base Scene] Openspace Statemachine Fallback
  //    1. Openspace state == OpenspaceStateEnum::FALLBACK
  //    ---> Strategy : Replan

  //[Strategy]
  DeciderResult result;
  result.strategy = APAStrategyType::NONE;
  if (openspace_is_fallback) {
    result.strategy = APAStrategyType::REPLAN;
    result.reason = "replan when openspace fallback";
  }

  return result;
}

APABehaviorDeciderParkIn::DeciderResult
APABehaviorDeciderParkIn::strategy_blocked(bool blocked_base_scene) {

  // ################## Strategy Explanation ##################
  // [Base Scene ] Blocked
  //    1. non-pause
  //    2. (AND) planning_status.blocked_timeout
  //    3. (AND) planning_status.blocked
  //    4. (AND) openspace_statemachine == RUNNING
  //    All these calculated in calculate_blocked_base_scene

  // [Sub Scene 1] Blocked By Obstacle Behind
  //    1. block_by_behind_obstacles (calculated by behavior)
  //    2. (AND) tried enough times inside parking-slot
  //      2.1 times_try_parking_in > 0 && lat && heading in threshold
  //      2.2 (OR) times_try_parking_in > 1
  //    ---> Strategy: FINISH when lat && heading in threshold (use 0.06rad
  //                    heading in 2.2)
  //    ---> Strategy: ABANDON when !(lat && heading in threshold )

  // [Sub Scene 2] Tried Too Many Times
  //    1. tried >3 times inside parking-slot
  //    ---> Strategy: FINISH when lat && heading in threshold
  //    ---> Strategy: ABANDON when !(lat && heading in threshold )

  // [Sub Scene 3] Not Paused Default
  //    1. has_paused
  //    2. not subscene 1&&2
  //    ---> Strategy: REPLAN

  // [Sub Scene 4] Paused running long enough
  //    1. has_running_enough_long
  //    ---> Strategy: FINISH

  //[PlanningContext Input]
  const auto &g_has_moved =
      PlanningContext::Instance()->parking_behavior_planner_output().has_moved;

  const auto &g_has_running_enough_long =
      PlanningContext::Instance()->planning_status().has_running_enough_long;

  const auto &g_has_paused =
      PlanningContext::Instance()->parking_behavior_planner_output().has_paused;

  const auto &ego_pose = world_model_->get_ego_state().ego_pose;

  const auto &g_times_try_parking_in = PlanningContext::Instance()
                                           ->openspace_motion_planner_output()
                                           .times_try_parking_in;

  const auto &g_block_by_behind_obstacles =
      PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .is_blocked_by_obstacle_behind_in_slot;

  //[Strategy]
  DeciderResult result;
  result.strategy = APAStrategyType::NONE;

  if (!blocked_base_scene) {
    return result;
  }

  Pose2D newest_target_pose_lot =
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->parking_lot->getParkingInPose(
              VehicleParam::Instance()->front_edge_to_center,
              VehicleParam::Instance()->back_edge_to_center,
              VehicleParam::Instance()->brake_distance_buffer,
              world_model_->get_ego_state().ego_pose,
              PlanningContext::Instance()
                  ->parking_behavior_planner_output()
                  .parking_slot_info.wheel_stop_info.wheel_stop_depth);

  const double lon_ending_thres =
      CarParams::GetInstance()->car_config.ending_check_config.lon_ending_thres;
  const double lat_ending_thres =
      CarParams::GetInstance()->car_config.ending_check_config.lat_ending_thres;
  const double angle_ending_thres =
      CarParams::GetInstance()
          ->car_config.ending_check_config.angle_ending_thres;
  const Pose2D POSE_THRES{lon_ending_thres, lat_ending_thres,
                          angle_ending_thres};
  bool is_lon_within_thres =
      isEgoLonInPlace(ego_pose, newest_target_pose_lot, POSE_THRES);
  bool is_lat_within_thres =
      isEgoLatInPlace(ego_pose, newest_target_pose_lot, POSE_THRES);
  bool is_heading_within_thres =
      isEgoHeadingInPlace(ego_pose, newest_target_pose_lot, POSE_THRES);

  const double angle_ending_thres_after_adjust =
      CarParams::GetInstance()
          ->car_config.ending_check_config.angle_ending_thres_after_adjust;
  const Pose2D POSE_THRES_SLACK{5 * lon_ending_thres, lat_ending_thres,
                                angle_ending_thres_after_adjust};
  bool is_heading_within_thres_slack =
      isEgoHeadingInPlace(ego_pose, newest_target_pose_lot, POSE_THRES_SLACK);

  bool is_supposed_to_finish1 =
      (g_times_try_parking_in > 1 && g_block_by_behind_obstacles) ||
      (g_times_try_parking_in > 0 && is_lat_within_thres &&
       is_heading_within_thres && g_block_by_behind_obstacles);

  bool is_supposed_to_finish2 = g_times_try_parking_in == 3;

  if (g_times_try_parking_in > 1) {
    is_heading_within_thres = is_heading_within_thres_slack;
  }

  if (is_supposed_to_finish1) {
    if (is_lat_within_thres && is_heading_within_thres) {
      result.strategy = APAStrategyType::FINISH;
      result.reason = "Finish when blocked by obstacle behind and heading&&lat "
                      "reached, times_try_parkng_in = " +
                      std::to_string(g_times_try_parking_in);
    } else {
      result.strategy = APAStrategyType::ABANDON;
      result.reason = "Abandon when blocked by obstacle behind heading&&lat "
                      "not reached, times_try_parkng_in = " +
                      std::to_string(g_times_try_parking_in);
    }
  } else if (is_supposed_to_finish2) {
    if (is_lat_within_thres && is_heading_within_thres) {
      result.strategy = APAStrategyType::FINISH;
      result.reason =
          "Finish when blocked and tried too many times and heading&&lat "
          "reached, times_try_parkng_in = " +
          std::to_string(g_times_try_parking_in);
    } else {
      result.strategy = APAStrategyType::ABANDON;
      result.reason =
          "Abandon when blocked and tried too many times heading&&lat "
          "not reached, times_try_parkng_in = " +
          std::to_string(g_times_try_parking_in);
    }
  } else {
    // set initial planning direction
    if (g_has_running_enough_long && g_has_moved || g_has_paused) {
      result.strategy = APAStrategyType::REPLAN;
      result.reason = "replan for block times_try_parking_in = " +
                      std::to_string(g_times_try_parking_in);
    } else if (g_has_running_enough_long) {
      result.strategy = APAStrategyType::ABANDON;
      result.reason = "finish(abandon) because of replan but not move";
    }
  }

  return result;
}

APABehaviorDeciderParkIn::DeciderResult
APABehaviorDeciderParkIn::strategy_traj_following_finish(
    bool openspace_is_finish, bool vehicle_reached_with_wheelstop) {
  // ################## Strategy Explanation ##################
  // [Base Scene] when traj following finish
  //    1. openspace_is_finish
  //    2. (AND) non_pause

  // [Sub Scene 1] Match Finish Condition
  //    1. reached target
  //    ---> Strategy : Finish

  // [Sub Scene 2] others
  //    ---> Strategy : Replan
  // ############## End of Strategy Explanation ###############

  //[PlanningContext Input]
  const auto &g_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;

  const auto &g_parking_lot = PlanningContext::Instance()
                                  ->parking_behavior_planner_output()
                                  .parking_lot;

  //[Strategy]
  DeciderResult result;
  result.strategy = APAStrategyType::NONE;

  const auto &ego_pose = world_model_->get_ego_state().ego_pose;

  bool base_scene = openspace_is_finish && !world_model_->get_pause_status();

  if (!base_scene) {
    return result;
  }

  if (vehicle_reached_with_wheelstop) {
    result.strategy = APAStrategyType::FINISH;
    result.reason = "finish because of reach target pose lot";

  } else {
    result.strategy = APAStrategyType::REPLAN;
    result.reason = "replan because of not reach target pose lot";
  }

  return result;
}

APABehaviorDeciderParkIn::DeciderResult
APABehaviorDeciderParkIn::strategy_first_reverse_gear_switch(
    bool is_at_last_segment) {
  // ################## Strategy Explanation ##################
  // [Base Scene] Non-Parallel first reverse gear switch
  //    1. gear_changing
  //    2. (AND) zigzag_num == 1
  //    3. (AND) non_at_last_segment
  //    4. (AND) non_parallel
  //    5. (AND) non_pause
  //    ---> Strategy: Replan

  //[PlanningContext Input]
  const auto &g_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;

  const auto &g_planning_status =
      PlanningContext::Instance()->planning_status();

  //[Strategy]
  DeciderResult result;
  result.strategy = APAStrategyType::NONE;

  bool base_scene = g_planning_status.planning_result.gear_changing &&
                    g_planning_status.zigzag_num == 1 && !is_at_last_segment &&
                    g_slot_info.type.value != ParkingSlotType::PARALLEL &&
                    !world_model_->get_pause_status();

  if (base_scene) {
    result.strategy = APAStrategyType::REPLAN;
    result.reason = "replan because of first reverse gear switch nonparallel";
  }

  return result;
}

APABehaviorDeciderParkIn::DeciderResult
APABehaviorDeciderParkIn::strategy_assumed_blocked() {
  // ################## Strategy Explanation ##################
  // [Base Scene] assumed blocked
  //    1. gear == REVERSE
  //    2. (AND) slot.type != PARALLEL
  //    3. (AND) world_model_->get_gear_report().gear_status.value != gear + 1
  //    4. (AND) zigzag_num == 1
  //    5. (AND) free_space.id > 0
  //    6. (AND) free_space.d_rel < 5.0
  //    7. (AND) non_pause
  //    ---> Strategy: Replan
  // ############## End of Strategy Explanation ###############

  //[PlanningContext Input]
  const auto &g_planning_gear =
      PlanningContext::Instance()->planning_status().planning_result.gear;

  const auto &g_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;

  const auto &g_planning_zigzag_num =
      PlanningContext::Instance()->planning_status().zigzag_num;

  const auto &g_free_space = PlanningContext::Instance()
                                 ->longitudinal_behavior_planner_output()
                                 .free_space;

  //[Strategy]
  DeciderResult result;
  result.strategy = APAStrategyType::NONE;
  bool base_scene = g_planning_gear == GearState::REVERSE &&
                    g_slot_info.type.value != ParkingSlotType::PARALLEL &&
                    ((int)world_model_->get_gear_report().gear_status.value) !=
                        (int(g_planning_gear) + 1) &&
                    g_planning_zigzag_num == 1 && g_free_space.id > 0 &&
                    g_free_space.d_rel < 5.0 &&
                    !world_model_->get_pause_status();

  if (base_scene) {
    result.strategy = APAStrategyType::REPLAN;
    result.reason = "replan because of assumed blocked";
  }

  return result;
}

void APABehaviorDeciderParkIn::reset_mpc_collide_timeout() {
  dynamic_plan_mpc_block_timer_.reset();
}

APABehaviorDeciderParkIn::DeciderResult
APABehaviorDeciderParkIn::strategy_during_pause(
    const bool is_pause_status, const double trajectory_length,
    const bool is_at_last_segment, const bool vehicle_reached_with_wheelstop,
    const bool has_moved, const bool need_pause,
    const LongitudinalBehaviorPlannerOutput::RemainDistInfo
        &remain_distance_info, std::string* ptr_debug_string) {
  // ################## Strategy Explanation ##################
  // [Base Scene ] Pause
  //    1. have paused

  // [Sub Scene 1] Blocked By Static Still Obstacle
  //    1. have long enough path
  //    2. (AND) static && block_by_behind_obstacles (calculated by behavior)
  //    ---> Strategy: REPLAN

  // [Sub Scene 2] Not Reached Target Pose At Last Segment
  //    1. not long enough path
  //    2. (AND) is_at_last_segment
  //    3. (AND) !reached_target
  //    ---> Strategy: REPLAN

  // debug info
  auto get_debug_string =
      [&](const APABehaviorDeciderParkIn::DeciderResult &result) {
        std::string debug_string;
        debug_string =
            "\n[sm]pause(status" + std::to_string(is_pause_status) +
            "|tl" + std::to_string(trajectory_length).substr(0, 5) +
            ",hasMov" + std::to_string(has_moved) +
            ",needPa" + std::to_string(need_pause) + ",isDyn" +
            std::to_string(remain_distance_info.is_traj_have_dynamic_obs_) + ",reach" +
            std::to_string(vehicle_reached_with_wheelstop) + "=act" +
            std::to_string((int)result.strategy);
        return debug_string;
      };

  //[Strategy]
  APABehaviorDeciderParkIn::DeciderResult result;
  result.strategy = APAStrategyType::NONE;

  if (!is_pause_status) {
    *ptr_debug_string = get_debug_string(result);
    return result;
  }

  // blocked by static obstacle
  if (trajectory_length > kDeadZonePathLength) {
    if (remain_distance_info.remaining_distance_ < kRemainDistFromPauseReplan &&
        !remain_distance_info.is_traj_have_dynamic_obs_ && has_moved) {
      result.strategy = APAStrategyType::REPLAN;
      result.reason = "replan because of static obs in path during pause";
    }
  } else { // not reached at last segment
    if (is_at_last_segment && !vehicle_reached_with_wheelstop) {
      result.strategy = APAStrategyType::REPLAN;
      result.reason = "replan because of no in target pose during pause";
    }
  }

  *ptr_debug_string = get_debug_string(result);
  return result;
}

} // namespace parking
} // namespace msquare
