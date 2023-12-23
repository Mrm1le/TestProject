#include "planner/behavior_planner/lateral_behavior_planner.h"

namespace msquare {

bool LateralBehaviorPlanner::update_planner_output() {
  auto &map_info = world_model_->get_map_info();
  auto &map_info_mgr = world_model_->get_map_info_manager();
  auto &lateral_output = context_->mutable_lateral_behavior_planner_output();
  auto &state_machine_output = context_->state_machine_output();

  auto &flane = virtual_lane_mgr_->get_fix_lane();
  auto &olane = virtual_lane_mgr_->get_origin_lane();
  auto &tlane = virtual_lane_mgr_->get_target_lane();
  auto &f_refline = virtual_lane_mgr_->get_fix_refline();
  auto &lateral_obstacle = world_model_->mutable_lateral_obstacle();
  auto planning_status = context_->mutable_planning_status();
  int scenario = state_machine_output.scenario;
  int state = state_machine_output.curr_state;
  auto &state_name = state_machine_output.state_name;
  int turn_light = state_machine_output.turn_light;

  bool lc_pause = state_machine_output.lc_pause;
  int lc_pause_id = state_machine_output.lc_pause_id;
  double tr_pause_l = state_machine_output.tr_pause_l;
  double tr_pause_s = state_machine_output.tr_pause_s;

  bool isRedLightStop =
      world_model_->get_traffic_light_decision()->get_stop_flag();

  update_planner_status();

  lateral_output.enable = true;
  lateral_output.track_id = ignore_track_id_;
  lateral_output.v_limit = 40.0 / 3.6;
  lateral_output.isRedLightStop = isRedLightStop;

  lateral_output.lat_offset = path_planner_->lat_offset();

  TrackedObject *lead_one = lateral_obstacle.leadone();

  lateral_output.fix_refline_index =
      f_refline.position() + map_info.current_lane_index();
  lateral_output.origin_refline_index =
      olane.position() + map_info.current_lane_index();
  lateral_output.target_refline_index =
      tlane.position() + map_info.current_lane_index();
  if (f_refline.position() == LEFT_POS ||
      f_refline.position() == LEFT_LEFT_POS) {
    lateral_output.which_lane = "left_line";
  } else if (f_refline.position() == RIGHT_POS) {
    lateral_output.which_lane = "right_line";
  } else {
    lateral_output.which_lane = "current_line";
  }

  int lc_request = state_machine_output.lc_request;

  if (lc_request == NO_CHANGE) {
    lateral_output.lc_request = "none";
  } else if (lc_request == LEFT_CHANGE) {
    lateral_output.lc_request = "left";
  } else {
    lateral_output.lc_request = "right";
  }

  if (state == ROAD_LC_LCHANGE) {
    lateral_output.lc_status = "left_lane_change";
  } else if (state == ROAD_LC_LBACK) {
    lateral_output.lc_status = "left_lane_change_back";
  } else if (state == ROAD_LC_RCHANGE) {
    lateral_output.lc_status = "right_lane_change";
  } else if (state == ROAD_LC_RBACK) {
    lateral_output.lc_status = "right_lane_change_back";
  } else if (state == ROAD_LC_LWAIT) {
    lateral_output.lc_status = "left_lane_change_wait";
  } else if (state == ROAD_LC_RWAIT) {
    lateral_output.lc_status = "right_lane_change_wait";
  } else {
    lateral_output.lc_status = "none";
  }

  lateral_output.int_cancel_reason_output =
      state_machine_output.int_cancel_reason_fsm;

  lateral_output.avd_info.clear();
  double ego_vel =
      world_model_->get_cart_ego_state_manager().get_cart_ego_state().ego_vel;
  // don't nudge any obstacles below 30kph
  const auto &planner_config =
      ConfigurationContext::Instance()->planner_config();
  bool cancle_avd_limit =
      planner_config.common_config.use_eftp ||
      planner_config.lateral_motion_planner_config.lat_safety_improved;
  if (ego_vel > 30.0 / 3.6 || cancle_avd_limit) {
    for (auto it = avd_info_.begin(); it != avd_info_.end(); ++it) {
      TrackedObject track{};
      // dont nudge any vrus
      if (lateral_obstacle.find_track(it->first, track)) {
        if (track.type > 10000 && !(cancle_avd_limit && track.type == 10002)) {
          continue;
        }
      } else {
        continue;
      }
      lateral_output.avd_info.push_back(it->second);
    }
  } else if (lateral_output.lc_status == "left_lane_change" ||
             lateral_output.lc_status == "right_lane_change") {
    if (lateral_obstacle.tleadone() != nullptr &&
        avd_info_.find(lateral_obstacle.tleadone()->track_id) !=
            avd_info_.end()) {
      lateral_output.avd_info.push_back(
          avd_info_[lateral_obstacle.tleadone()->track_id]);
    }
  }

  // don't nudge any obstacles when disabled
  if (!world_model_->enable_lateral_dogde()) {
    lateral_output.avd_info.clear();
  }

  lateral_output.scenario = scenario;
  lateral_output.flane_width = flane.width();
  if (!map_info.is_in_intersection()) {
    for (auto &p : map_info.current_refline_points()) {
      if (p.car_point.x >= 0 && p.in_intersection == false) {
        if (p.lane_width < 10.) {
          lateral_output.flane_width = p.lane_width;
        } else if (p.distance_to_left_lane_border < 10. &&
                   p.distance_to_right_lane_border < 10.) {
          lateral_output.flane_width =
              p.distance_to_left_lane_border + p.distance_to_right_lane_border;
        } else if (p.distance_to_left_lane_border < 10.) {
          lateral_output.flane_width = p.distance_to_left_lane_border * 2;
        } else if (p.distance_to_right_lane_border < 10.) {
          lateral_output.flane_width = p.distance_to_right_lane_border * 2;
        }
        if (lateral_output.flane_width > 4.5) {
          lateral_output.cur_wide_lane = true;
        } else {
          lateral_output.cur_wide_lane = false;
        }
        break;
      }
    }
  } else {
    lateral_output.cur_wide_lane = false;
  }

  lateral_output.path_points = f_refline.path_points();

  bool left_direct_exist =
      (map_info.left_lane_marks() & map_info.traffic_light_direction());

  bool right_direct_exist =
      (map_info.right_lane_marks() & map_info.traffic_light_direction());

  bool curr_direct_has_straight =
      (map_info.current_lane_marks() & Direction::GO_STRAIGHT);

  bool curr_direct_has_right =
      (map_info.current_lane_marks() & Direction::TURN_RIGHT);

  bool left_direct_has_straight =
      (map_info.left_lane_marks() & Direction::GO_STRAIGHT);

  bool is_right_turn =
      (map_info.traffic_light_direction() & Direction::TURN_RIGHT);

  if (map_info_mgr.llane_.exist() == false || left_direct_exist == false) {
    lateral_output.tleft_lane = true;
  } else {
    lateral_output.tleft_lane = false;
  }

  if (map_info.current_lane_index() == map_info.lanes_num() - 1 &&
      map_info.current_lane_index() - 1 >= 0) {
    lateral_output.rightest_lane = true;
  } else {
    lateral_output.rightest_lane = false;
  }

  if (std::fabs(map_info.dist_to_intsect() - DBL_MAX) > 1e-2) {
    lateral_output.dist_intersect = map_info.dist_to_intsect();
  } else {
    lateral_output.dist_intersect = 1000;
  }

  if (std::fabs(map_info.intsect_length() - DBL_MAX) > 1e-2) {
    lateral_output.intersect_length = map_info.intsect_length();
  } else {
    lateral_output.intersect_length = 1000;
  }

  if (std::fabs(map_info.lc_end_dis() - DBL_MAX) > 1e-2) {
    lateral_output.lc_end_dis = map_info.lc_end_dis();
  } else {
    lateral_output.lc_end_dis = 10000;
  }

  if (std::fabs(map_info.dis_to_ramp() - DBL_MAX) > 1e-2) {
    lateral_output.dis_to_ramp = map_info.dis_to_ramp();
  } else {
    lateral_output.dis_to_ramp = 10000;
  }

  lateral_output.premoving = path_planner_->premoving();
  lateral_output.lc_pause_id = lc_pause_id;
  lateral_output.lc_pause = lc_pause;
  lateral_output.tr_pause_l = tr_pause_l;
  lateral_output.tr_pause_s = tr_pause_s;
  lateral_output.l_dash_length = state_machine_output.l_dash_length;
  lateral_output.r_dash_length = state_machine_output.r_dash_length;
  lateral_output.must_change_lane = state_machine_output.must_change_lane;
  lateral_output.isOnHighway = map_info.is_on_highway();

  lateral_output.c_poly.assign(path_planner_->c_poly().begin(),
                               path_planner_->c_poly().end());

  lateral_output.d_poly.assign(path_planner_->d_poly().begin(),
                               path_planner_->d_poly().end());

  lateral_output.s_v_limit.clear();
  lateral_output.s_a_limit.clear();
  lateral_output.s_r_offset.clear();
  lateral_output.s_v_limit.assign(s_v_limit_.begin(), s_v_limit_.end());
  lateral_output.s_a_limit.assign(s_a_limit_.begin(), s_a_limit_.end());
  lateral_output.s_r_offset.assign(s_r_offset_.begin(), s_r_offset_.end());

  if (lead_one != nullptr) {
    lateral_output.lead_one_drel = lead_one->d_rel;
    lateral_output.lead_one_vrel = lead_one->v_rel;
  } else {
    lateral_output.lead_one_drel = 0.0;
    lateral_output.lead_one_vrel = 0.0;
  }

  lateral_output.state_name = state_name;

  lateral_output.scenario_name =
      (scenario == LOCATION_ROAD) ? "Road" : "Intersect";

  if (turn_light == 1) {
    lateral_output.turn_light = "Left";
  } else if (turn_light == 2) {
    lateral_output.turn_light = "Right";
  } else {
    lateral_output.turn_light = "None";
  }

  auto request_source = state_machine_output.lc_request_source;
  lateral_output.act_request_source = "none";
  if (request_source == INT_REQUEST) {
    lateral_output.lc_request_source = "int_request";
  } else if (request_source == MAP_REQUEST) {
    lateral_output.lc_request_source = "map_request";
  } else if (request_source == ACT_REQUEST) {
    lateral_output.lc_request_source = "act_request";
    lateral_output.act_request_source = state_machine_output.act_request_source;
  } else {
    lateral_output.lc_request_source = "none";
  }

  if (state_machine_output.lc_turn_light > 0) {
    lateral_output.turn_light_source = "lc_turn_light";
  } else {
    lateral_output.turn_light_source = "none";
  }

  if (nullptr != planning_status) {
    planning_status->planning_result.turn_signal_cmd.value = turn_light;
  }

  lateral_output.avd_car_past = avd_car_past_;
  lateral_output.avd_sp_car_past = avd_sp_car_past_;
  lateral_output.vel_sequence = vel_sequence_;

  lateral_output.ignore_change_false = ignore_change_false_;
  lateral_output.ignore_change_true = ignore_change_true_;

  lateral_output.l_poly = path_planner_->l_poly();
  lateral_output.r_poly = path_planner_->r_poly();

  if (!update_lateral_info()) {
    return false;
  }
  return true;
}

bool LateralBehaviorPlanner::update_lateral_info() {
  world_model_->mutable_map_info_manager().get_lane_change_point(world_model_);

  auto &map_info = world_model_->mutable_map_info_manager().get_map_info();

  auto planning_status = context_->mutable_planning_status();
  auto &lateral_output = context_->lateral_behavior_planner_output();

  auto lc_request = lateral_output.lc_request;
  auto lc_status = lateral_output.lc_status;

  auto &state_machine_output = context_->state_machine_output();

  int scenario = state_machine_output.scenario;
  int state = state_machine_output.curr_state;
  LaneStatus default_lane_status;

  if (state == ROAD_NONE) {
    planning_status->lane_status.status = LaneStatus::Status::LANE_KEEP;
  } else {
    if (lc_request != "none") {
      planning_status->lane_status.status = LaneStatus::Status::LANE_CHANGE;

      if (map_info.current_tasks().size() > 0 &&
          (map_info.current_tasks()[0] == -1 ||
           map_info.current_tasks()[0] == 1)) {
        planning_status->lane_status.change_lane.is_active_lane_change = false;
      } else {
        planning_status->lane_status.change_lane.is_active_lane_change = true;
      }
      if (lc_status == "none") {
        // lane change preparation stage
        planning_status->lane_status.change_lane.status =
            ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;
        if (lc_request == "left") {
          planning_status->lane_status.change_lane.direction = "left";
        } else if (lc_request == "right") {
          planning_status->lane_status.change_lane.direction = "right";
        }
      } else if (lc_status == "left_lane_change" ||
                 lc_status == "right_lane_change") {
        planning_status->lane_status.change_lane.status =
            ChangeLaneStatus::Status::IN_CHANGE_LANE;
        if (lc_status == "left_lane_change") {
          planning_status->lane_status.change_lane.direction = "left";
        } else if (lc_status == "right_lane_change") {
          planning_status->lane_status.change_lane.direction = "right";
        }
      } else if (lc_status == "left_lane_change_back" ||
                 lc_status == "right_lane_change_back") {
        planning_status->lane_status.change_lane.status =
            ChangeLaneStatus::Status::CHANGE_LANE_BACK;
        if (lc_status == "left_lane_change_back") {
          planning_status->lane_status.change_lane.direction = "left";
        } else if (lc_status == "right_lane_change_back") {
          planning_status->lane_status.change_lane.direction = "right";
        }
      } else if (lc_status == "left_lane_change_wait" ||
                 lc_status == "right_lane_change_wait") {
        planning_status->lane_status.change_lane.status =
            ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;
        if (lc_status == "left_lane_change_wait") {
          planning_status->lane_status.change_lane.direction = "left";
        } else if (lc_status == "right_lane_change_wait") {
          planning_status->lane_status.change_lane.direction = "right";
        }
      }
    }
  }

  // update target lane id
  int target_lane_id = 0;
  if (lateral_output.which_lane == "left_line") {
    target_lane_id = -1;
  } else if (lateral_output.which_lane == "right_line") {
    target_lane_id = 1;
  } else {
    target_lane_id = 0;
  }
  planning_status->lane_status.target_lane_id = target_lane_id;
  planning_status->lane_status.change_lane.path_id = target_lane_id;
  if (planning_status->lane_status.status == LaneStatus::Status::LANE_CHANGE &&
      (planning_status->lane_status.change_lane.status ==
           ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION ||
       planning_status->lane_status.change_lane.status ==
           ChangeLaneStatus::Status::CHANGE_LANE_BACK)) {
    auto &target_lane = virtual_lane_mgr_->mutable_target_lane();
    if (target_lane.has_master()) {
      planning_status->lane_status.change_lane.path_id =
          target_lane.master()->position();
    } else {
      if (lc_request == "left") {
        planning_status->lane_status.change_lane.path_id =
            clip(target_lane_id - 1, 1, -1);
      } else if (lc_request == "right") {
        planning_status->lane_status.change_lane.path_id =
            clip(target_lane_id + 1, 1, -1);
      }
    }
  }
  planning_status->lane_status.target_lane_lat_offset =
      lateral_output.lat_offset;

  auto baseline_info = world_model_->get_baseline_info(target_lane_id);

  // todo: add target baseline protection in lane change /lane borrow stage
  if (baseline_info == nullptr || !baseline_info->is_valid()) {
    MSD_LOG(ERROR, "arbitrator invalid target lane[%d]!", target_lane_id);
    return false;
  }

  return true;
}

void LateralBehaviorPlanner::update_planner_status() {
  auto &lateral_output = context_->mutable_lateral_behavior_planner_output();
  auto &state_machine_output = context_->state_machine_output();

  lateral_output.planner_scene = 0;
  lateral_output.planner_action = 0;
  lateral_output.planner_status = 0;

  switch (state_machine_output.curr_state) {
  case ROAD_NONE:
    lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
    break;

  case ROAD_LC_LWAIT:
    lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
    lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
    break;

  case ROAD_LC_RWAIT:
    lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
    lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
    break;

  case ROAD_LC_LCHANGE:
    lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
    lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
    break;

  case ROAD_LC_RCHANGE:
    lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
    lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
    break;

  case ROAD_LC_LBACK:
    lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
    lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
    break;

  case ROAD_LC_RBACK:
    lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
    lateral_output.planner_action = AlgorithmAction::LANE_CHANGE_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
    break;

  case ROAD_LB_LBORROW:
    lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
    lateral_output.planner_action = AlgorithmAction::LANE_BORROW_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_BORROWING;
    break;

  case ROAD_LB_RBORROW:
    lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
    lateral_output.planner_action = AlgorithmAction::LANE_BORROW_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_BORROWING;
    break;

  case ROAD_LB_LBACK:
    lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
    lateral_output.planner_action = AlgorithmAction::LANE_BORROW_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_BACK;
    break;

  case ROAD_LB_RBACK:
    lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
    lateral_output.planner_action = AlgorithmAction::LANE_BORROW_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_BACK;
    break;

  case ROAD_LB_LRETURN:
    lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
    lateral_output.planner_action = AlgorithmAction::LANE_BORROW_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_RETURN;
    break;

  case ROAD_LB_RRETURN:
    lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
    lateral_output.planner_action = AlgorithmAction::LANE_BORROW_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_RETURN;
    break;

  case ROAD_LB_LSUSPEND:
    lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
    lateral_output.planner_action = AlgorithmAction::LANE_BORROW_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_SUSPEND;
    break;

  case ROAD_LB_RSUSPEND:
    lateral_output.planner_scene = AlgorithmScene::NORMAL_ROAD;
    lateral_output.planner_action = AlgorithmAction::LANE_BORROW_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_BORROW_SUSPEND;
    break;

  case INTER_GS_NONE:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT;
    break;

  case INTER_GS_LC_LWAIT:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT |
                                    AlgorithmAction::LANE_CHANGE_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
    break;

  case INTER_GS_LC_RWAIT:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT |
                                    AlgorithmAction::LANE_CHANGE_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
    break;

  case INTER_GS_LC_LCHANGE:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT |
                                    AlgorithmAction::LANE_CHANGE_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
    break;

  case INTER_GS_LC_RCHANGE:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT |
                                    AlgorithmAction::LANE_CHANGE_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
    break;

  case INTER_GS_LC_LBACK:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT |
                                    AlgorithmAction::LANE_CHANGE_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
    break;

  case INTER_GS_LC_RBACK:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_GO_STRAIGHT |
                                    AlgorithmAction::LANE_CHANGE_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
    break;

  case INTER_TR_NONE:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT;
    break;

  case INTER_TR_LC_LWAIT:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
                                    AlgorithmAction::LANE_CHANGE_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
    break;

  case INTER_TR_LC_RWAIT:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
                                    AlgorithmAction::LANE_CHANGE_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
    break;

  case INTER_TR_LC_LCHANGE:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
                                    AlgorithmAction::LANE_CHANGE_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
    break;

  case INTER_TR_LC_RCHANGE:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
                                    AlgorithmAction::LANE_CHANGE_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
    break;

  case INTER_TR_LC_LBACK:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
                                    AlgorithmAction::LANE_CHANGE_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
    break;

  case INTER_TR_LC_RBACK:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_RIGHT |
                                    AlgorithmAction::LANE_CHANGE_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
    break;

  case INTER_TL_NONE:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT;
    break;

  case INTER_TL_LC_LWAIT:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
                                    AlgorithmAction::LANE_CHANGE_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
    break;

  case INTER_TL_LC_RWAIT:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
                                    AlgorithmAction::LANE_CHANGE_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_WAITING;
    break;

  case INTER_TL_LC_LCHANGE:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
                                    AlgorithmAction::LANE_CHANGE_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
    break;

  case INTER_TL_LC_RCHANGE:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
                                    AlgorithmAction::LANE_CHANGE_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGEING;
    break;

  case INTER_TL_LC_LBACK:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
                                    AlgorithmAction::LANE_CHANGE_LEFT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
    break;

  case INTER_TL_LC_RBACK:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_TURN_LEFT |
                                    AlgorithmAction::LANE_CHANGE_RIGHT;
    lateral_output.planner_status = AlgorithmStatus::LANE_CHANGE_BACK;
    break;

  case INTER_UT_NONE:
    lateral_output.planner_scene = AlgorithmScene::INTERSECT;
    lateral_output.planner_action = AlgorithmAction::INTERSECT_U_TURN;
    break;

  default:
    break;
  }
}

void print_avd_car_past(
    const std::array<std::vector<double>, 2> &avd_car_past) {
  int len = 0;
  char buff[1024];

  for (size_t i = 0; i < avd_car_past.size(); i++) {
    if (avd_car_past[i].size() == 0) {
      continue;
    }

    for (size_t j = 0; j < avd_car_past[i].size(); j++) {
      int temp = (int)avd_car_past[i][j];
      if (equal_zero(avd_car_past[i][j] - temp)) {
        len += sprintf(buff + len, "%d ", (int)avd_car_past[i][j]);
      } else {
        len += sprintf(buff + len, "%lf ", avd_car_past[i][j]);
      }
    }

    MSD_LOG(INFO, "avd_car_past[%lu] %s", i, buff);
    len = 0;
  }
}

void print_sp_car_past(
    const std::array<std::vector<double>, 2> &avd_sp_car_past) {
  int len = 0;
  char buff[1024];

  for (size_t i = 0; i < avd_sp_car_past.size(); i++) {
    if (avd_sp_car_past[i].size() == 0) {
      continue;
    }

    for (size_t j = 0; j < avd_sp_car_past[i].size(); j++) {
      int temp = (int)avd_sp_car_past[i][j];
      if (equal_zero(avd_sp_car_past[i][j] - temp)) {
        len += sprintf(buff + len, "%d ", (int)avd_sp_car_past[i][j]);
      } else {
        len += sprintf(buff + len, "%lf ", avd_sp_car_past[i][j]);
      }
    }

    MSD_LOG(INFO, "avd_sp_car_past[%lu] %s", i, buff);
    len = 0;
  }
}

void LateralBehaviorPlanner::print_planner_output() {
  auto &map_info_mgr = world_model_->get_map_info_manager();
  auto &lateral_output = context_->lateral_behavior_planner_output();
  auto &state_machine_output = context_->state_machine_output();

  if (map_info_mgr.mmp_update_ == true) {
    print_avd_car_past(avd_car_past_);
    print_sp_car_past(avd_sp_car_past_);

    MSD_LOG(INFO,
            "[LateralBehaviorPlanner] track_id[%d] lat_offset[%lf] lane[%s] "
            "request[%s] status[%s]",
            lateral_output.track_id, lateral_output.lat_offset,
            lateral_output.which_lane.c_str(),
            lateral_output.lc_request.c_str(),
            lateral_output.lc_status.c_str());
    int i = 0;
    for (auto &avd : lateral_output.avd_info) {
      MSD_LOG(DEBUG,
              "[LateralBehaviorPlanner] avd_info%d: id[%d] property[%s] "
              "ignore[%d] avd_direction[%s]",
              i, avd.id, avd.property.c_str(), avd.ignore,
              avd.avd_direction.c_str());

      i++;
    }

    MSD_LOG(INFO, "scenario[%s] status[%s] turn_light[%s]\n\n",
            lateral_output.scenario_name.c_str(),
            state_machine_output.state_name.c_str(),
            lateral_output.turn_light.c_str());
  }
}

} // namespace msquare
