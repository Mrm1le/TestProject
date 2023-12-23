#include <utility>

#include "common/config_context.h"
#include "planner/behavior_planner/deciders/lane_change_decider.h"
#include "planner/behavior_planner/general_motion_planner.h"
#include "planner/behavior_planner/lateral_behavior_planner.h"
#include "planner/scenarios/scenario_facade.h"

namespace msquare {

MSDStateMachine::MSDStateMachine(const std::shared_ptr<WorldModel> &world_model)
    : lateral_fsm_(fsm_context_), world_model_(world_model) {
  // -------------- obstacle decider -------------//

  ScenarioFacadeConfig scenario_config_odc =
      ConfigurationContext::Instance()->get_params().at(
          ScenarioFacadeConfig::ScenarioFacadeType::FixLaneCruise);
  TaskConfig config_odc =
      scenario_config_odc
          .tasks_config_map_[TaskConfig::TaskType::OBSTACLE_DECIDER];

  obstacle_decider_ = std::make_unique<ObstacleDecider>(config_odc);
  obstacle_decider_->init(world_model_);
  MSD_LOG(INFO, "ODC_Created!");
  MSD_LOG(INFO, "ODC_call_count: %d", obstacle_decider_->get_call_count());

  // -------------- obstacle decider -------------//
  // ---------------- gmp -------------- //
  ScenarioFacadeConfig scenario_config =
      ConfigurationContext::Instance()->get_params().at(
          ScenarioFacadeConfig::ScenarioFacadeType::FixLaneCruise);
  TaskConfig config =
      scenario_config
          .tasks_config_map_[TaskConfig::TaskType::GENERAL_MOTION_PLANNER];

  general_motion_planner_ = std::make_unique<GeneralMotionPlanner>(config);
  general_motion_planner_->init(world_model_);
  MSD_LOG(INFO, "GMP_Created!");
  MSD_LOG(INFO, "GMP_call_count: %d",
          general_motion_planner_->get_call_count());
  // ---------------- gmp -------------- //
  fsm_context_.state = ROAD_NONE;
  fsm_context_.external = false;
  fsm_context_.name = "ROAD_NONE";

  auto *context = new FsmUserContext;
  context->safety_dist = 0.0;
  context->state_machine = this;

  fsm_context_.user_data = static_cast<void *>(context);
  fsm_context_.world_model = world_model;

  auto &map_info_mgr = world_model_->mutable_map_info_manager();
  auto &lateral_obstacle = world_model_->mutable_lateral_obstacle();
  auto &ego_state =
      world_model_->get_cart_ego_state_manager().get_cart_ego_state();

  virtual_lane_mgr_ = std::make_shared<VirtualLaneManager>(
      map_info_mgr.clane_, map_info_mgr.llane_, map_info_mgr.rlane_,
      map_info_mgr.rrlane_, map_info_mgr.lllane_, map_info_mgr.c_raw_refline_,
      map_info_mgr.l_raw_refline_, map_info_mgr.r_raw_refline_,
      map_info_mgr.ll_raw_refline_, map_info_mgr.rr_raw_refline_);

  lane_tracks_mgr_ = std::make_shared<LaneTracksManager>(
      map_info_mgr, lateral_obstacle, *virtual_lane_mgr_);

  lc_req_mgr_ = std::make_shared<LCRequestManager>();
}

MSDStateMachine::~MSDStateMachine() {
  auto *context = static_cast<FsmUserContext *>(fsm_context_.user_data);

  delete context;
}

bool MSDStateMachine::update() {
  double start_time = MTIME()->timestamp().ms();
  // ------------- execute odc ----------- //
  ScenarioFacadeContext scenario_context;
  *(scenario_context.mutable_planning_status()) =
      PlanningContext::Instance()->planning_status();
  scenario_context.mutable_planning_status()->planning_success = false;
  scenario_context.mutable_lateral_behavior_planner_output() =
      PlanningContext::Instance()->lateral_behavior_planner_output();

  if (obstacle_decider_->execute(&scenario_context) ==
      TaskStatus::STATUS_SUCCESS) {
    MSD_LOG(INFO, "ODC Executed!");
    MSD_LOG(INFO, "ODC_call_count: %d", obstacle_decider_->get_call_count());

    PlanningContext::Instance()->mutable_planner_debug()->odc_output =
        scenario_context.planner_debug().odc_output;
    //---temp---//
    MSD_LOG(
        INFO, "ODC ddp_context[%d][%d][%d]",
        obstacle_decider_->get_call_count(),
        ddp::DdpContext::Instance()->obstacle_decider_output().obs_infos.size(),
        ddp::DdpContext::Instance()
            ->get_obstacle_decider_ddp_trajectory()
            .trajectory.size());
    //---temp---//
  }
  // ------------- end of execute odc ----------- //
  // ------- execute gmp ----------- //
  auto lc_request = lc_req_mgr_->request();
  bool valid_body = check_lc_body_valid(lc_request);
  bool is_lane_stable = is_lc_lane_stable(lc_request);
  bool has_olane = virtual_lane_mgr_->has_origin_lane();
  bool has_tlane = virtual_lane_mgr_->has_target_lane();

  bool is_solid_line = false;
  if (lc_request == LEFT_CHANGE) {
    is_solid_line =
        world_model_->get_mutable_map_info_manager().is_solid_line(0);
  } else if (lc_request == RIGHT_CHANGE) {
    is_solid_line =
        world_model_->get_mutable_map_info_manager().is_solid_line(1);
  }

  auto external_request = LC_Dir::NONE;

  if (lc_request == LEFT_CHANGE) {
    external_request = LC_Dir::LEFT;
  } else if (lc_request == RIGHT_CHANGE) {
    external_request = LC_Dir::RIGHT;
  }

  general_motion_planner_->get_external_request(
      external_request, valid_body, is_steer_over_limit_,
      is_lca_state_activated_, is_lane_stable, is_solid_line, has_olane,
      has_tlane, gmp_should_cancel_);
  if (general_motion_planner_->execute(&scenario_context) ==
      TaskStatus::STATUS_SUCCESS) {
    MSD_LOG(INFO, "GMP Executed!");

    PlanningContext::Instance()->mutable_planner_debug()->gmp_input =
        scenario_context.planner_debug().gmp_input;
    PlanningContext::Instance()->mutable_planner_debug()->gmp_input.valid_body =
        valid_body;
    PlanningContext::Instance()
        ->mutable_planner_debug()
        ->gmp_input.is_steer_over_limit = is_steer_over_limit_;
    PlanningContext::Instance()
        ->mutable_planner_debug()
        ->gmp_input.is_lca_state_activated = is_lca_state_activated_;
    PlanningContext::Instance()
        ->mutable_planner_debug()
        ->gmp_input.is_lane_stable = is_lane_stable;
    PlanningContext::Instance()
        ->mutable_planner_debug()
        ->gmp_input.is_solid_line = is_solid_line;
    PlanningContext::Instance()->mutable_planner_debug()->gmp_input.has_olane =
        has_olane;
    PlanningContext::Instance()->mutable_planner_debug()->gmp_input.has_tlane =
        has_tlane;
    PlanningContext::Instance()
        ->mutable_planner_debug()
        ->gmp_input.gmp_should_cancel = gmp_should_cancel_;
    PlanningContext::Instance()->mutable_planner_debug()->gmp_output =
        scenario_context.planner_debug().gmp_output;
    *PlanningContext::Instance()->mutable_general_motion_planner_output() =
        scenario_context.general_motion_planner_output();
  }
  // ------- end of execute gmp ----------- //
  auto &map_info = world_model_->get_map_info();
  auto &map_info_mgr = world_model_->get_map_info_manager();
  bool active = world_model_->get_vehicle_dbw_status();

  // update ego_state_queue_
  constexpr int kEgoStateQueueSize = 70;
  ego_state_queue_.emplace_back(
      world_model_->get_cart_ego_state_manager().get_cart_ego_state());
  if (ego_state_queue_.size() > kEgoStateQueueSize) {
    ego_state_queue_.erase(ego_state_queue_.begin());
  }

  auto &lateral_output =
      PlanningContext::Instance()->lateral_behavior_planner_output();
  auto planning_status = PlanningContext::Instance()->mutable_planning_status();

  if (!planning_status->planning_success) {
    reset_state_machine();
    lc_req_mgr_->finish_request();
  }
  planning_status->scheme_stage = SchemeStage::PRIMARY;
  planning_status->last_planning_success = planning_status->planning_success;
  planning_status->planning_success = false;

  scenario_ = LocationEnum::LOCATION_ROAD;

  lane_tracks_mgr_->update();

  if (map_info_mgr.mmp_update_) {
    if (!virtual_lane_mgr_->flane_update()) {
      virtual_lane_mgr_->update_fix_lane();
    }
    if (fsm_context_.state == ROAD_LC_LCHANGE) {
      virtual_lane_mgr_->update_lc_lanes(1, fsm_context_.state);
      if (virtual_lane_mgr_->has_target_lane()) {
        virtual_lane_mgr_->set_fix_lane(TARGET_LANE);
      }
    } else if (fsm_context_.state == ROAD_LC_LBACK) {
      virtual_lane_mgr_->update_lc_lanes(1, fsm_context_.state);
      if (virtual_lane_mgr_->has_origin_lane()) {
        virtual_lane_mgr_->set_fix_lane(ORIGIN_LANE);
      }
    } else if (fsm_context_.state == ROAD_LC_RCHANGE) {
      virtual_lane_mgr_->update_lc_lanes(2, fsm_context_.state);
      if (virtual_lane_mgr_->has_target_lane()) {
        virtual_lane_mgr_->set_fix_lane(TARGET_LANE);
      }
    } else if (fsm_context_.state == ROAD_LC_RBACK) {
      virtual_lane_mgr_->update_lc_lanes(2, fsm_context_.state);
      if (virtual_lane_mgr_->has_origin_lane()) {
        virtual_lane_mgr_->set_fix_lane(ORIGIN_LANE);
      }
    } else {
      virtual_lane_mgr_->update_fix_lane();
      if (lc_req_mgr_->request() == NO_CHANGE) {
        virtual_lane_mgr_->clear_lc_lanes();
      } else {
        if (lc_req_mgr_->request() == RIGHT_CHANGE) {
          if (virtual_lane_mgr_->has_origin_lane() ||
              virtual_lane_mgr_->has_target_lane())
            virtual_lane_mgr_->update_lc_lanes(2, fsm_context_.state);
        } else if (lc_req_mgr_->request() == LEFT_CHANGE) {
          if (virtual_lane_mgr_->has_origin_lane() ||
              virtual_lane_mgr_->has_target_lane())
            virtual_lane_mgr_->update_lc_lanes(1, fsm_context_.state);
        }
      }
    }
    use_backup_baseline_ = false;
    virtual_lane_mgr_->update_fix_lane_info();
    auto &f_refline = virtual_lane_mgr_->mutable_fix_lane();
    auto baseline_info = world_model_->get_baseline_info(f_refline.position());

    if (!baseline_info || !baseline_info->is_valid()) {
      std::shared_ptr<BaseLineInfo> backup_baseline;
      double min_adc_l = std::numeric_limits<double>::max();
      std::vector<int> lane_ids = {-1, 0, 1};
      for (int i : lane_ids) {
        auto temp_baseline = world_model_->get_baseline_info(i);
        if (temp_baseline && temp_baseline->is_valid()) {
          if (std::abs(temp_baseline->get_ego_state().ego_frenet.y) <
              min_adc_l) {
            backup_baseline = temp_baseline;
            min_adc_l = std::abs(temp_baseline->get_ego_state().ego_frenet.y);
          }
        }
      }
      if (!backup_baseline) {
        MSD_LOG(INFO, "MSDStateMachine[MAP_ERROR]: no backup baseline");
        return false;
      } else {
        MSD_LOG(INFO, "MSDStateMachine: has backup baseline %d",
                backup_baseline->lane_id());
        reset_state_machine();
        lc_req_mgr_->finish_request();
        use_backup_baseline_ = true;
        if (virtual_lane_mgr_->update_fix_lane(backup_baseline->lane_id())) {
          virtual_lane_mgr_->update_fix_lane_info();
          baseline_info = backup_baseline;
        } else {
          MSD_LOG(INFO,
                  "MSDStateMachine[MAP_ERROR]: set backup baseline failed");
          return false;
        }
      }
    } else {
      MSD_LOG(INFO, "MSDStateMachine: has baseline %d", f_refline.position());

      calc_baseline_overlap(baseline_info, planning_status);
    }
    auto origin_baseline_info = baseline_info;
    if (virtual_lane_mgr_->has_origin_lane()) {
      origin_baseline_info = world_model_->get_baseline_info(
          virtual_lane_mgr_->mutable_origin_lane().position());
    }

    world_model_->truncate_prediction_info(baseline_info);
    bool isRedLightStop =
        world_model_->get_traffic_light_decision()->get_stop_flag();
    auto &lateral_obstacle = world_model_->mutable_lateral_obstacle();
    (void)lateral_obstacle.update(baseline_info->get_ego_state(),
                                  world_model_->get_truncated_prediction_info(),
                                  virtual_lane_mgr_->get_fix_refline(),
                                  isRedLightStop);
    lane_tracks_mgr_->update_ego_state(baseline_info->get_ego_state());

    last_state_ = fsm_context_.state;

    bool enable_lc = active && (!world_model_->is_acc_mode() ||
                                !world_model_->last_is_acc_mode());

    bool enable_ilc = enable_lc && world_model_->enable_ilc();
    bool enable_alc = enable_lc && world_model_->enable_alc();
    bool enable_recommend_alc =
        enable_lc && world_model_->get_enable_recommend_alc();

    const auto &request_manager_input = lc_req_mgr_->update_input(
        enable_ilc, enable_alc, enable_recommend_alc, world_model_, *virtual_lane_mgr_,
        *lane_tracks_mgr_, fsm_context_.state);
    lc_req_mgr_->update(request_manager_input);
    auto request_manager_output =
        PlanningContext::Instance()->mutable_request_manager_outut();
    *request_manager_output = lc_req_mgr_->request_manager_output();

    CostTime cost_time =
        CostTime{"state_machine", MTIME()->timestamp().ms() - start_time};
    auto *planner_debug = PlanningContext::Instance()->mutable_planner_debug();
    planner_debug->cost_time.emplace_back(cost_time);

    if (enable_lc) {
      update_state_machine();
    } else {
      reset_state_machine();
      lc_req_mgr_->finish_request();
      post_process();
    }
    update_hfsm_debug();
    MSD_LOG(INFO, "not_suitable update hfsm condition [%d]",
            lateral_output.lane_change_condition);
    world_model_->set_last_acc_mode(world_model_->is_acc_mode());
    return true;
  }
  world_model_->set_last_acc_mode(world_model_->is_acc_mode());
  return false;
}

void MSDStateMachine::update_state_machine() {
  MLOG_PROFILING("update_state_machine");
  auto *user_context = static_cast<FsmUserContext *>(fsm_context_.user_data);

  if (user_context == nullptr) {
    return;
  }

  auto &ego_state =
      world_model_->get_cart_ego_state_manager().get_cart_ego_state();
  auto &map_info = world_model_->get_map_info();
  double v_ego = ego_state.ego_vel;

  double safety_dist_base = 2.0;
  if (ConfigurationContext::Instance()
          ->synthetic_config()
          .sensor_configuration != "lidar") {
    safety_dist_base = 4.0;
  }
  user_context->safety_dist = v_ego * v_ego * 0.01 + safety_dist_base;
  check_vision_lane_stable();
  if (fsm_context_.state >= ROAD_NONE && fsm_context_.state <= ROAD_LC_RBACK) {
    auto int_lc_ready_time =
        ConfigurationContext::Instance()
            ->planner_config()
            .lateral_behavior_planner_config.int_lc_ready_time;
    if (world_model_->get_driving_model_config() == RADICAL_DRIVING_MODEL) {
      int_lc_ready_time =
          ConfigurationContext::Instance()
              ->planner_config()
              .lateral_behavior_planner_config.int_lc_ready_time_3;
    }
    user_context->delay_time[INT_REQUEST] = int_lc_ready_time;
    user_context->delay_time[MAP_REQUEST] = 0.0;
    if (map_info.is_on_highway()) {
      user_context->delay_time[MAP_REQUEST] = 1.0;
    }
    user_context->delay_time[ACT_REQUEST] = 1.0;

    lateral_fsm_.update();
  } else {
    MSD_LOG(ERROR, "[update_state_machine] unexpected state[%d]",
            fsm_context_.state);
    change_state_external<RoadState::None>();
    virtual_lane_mgr_->set_fix_lane(CURRENT_LANE);
  }
}

void MSDStateMachine::reset_state_machine() {
  change_state_external<RoadState::None>();
  last_state_ = ROAD_NONE;

  virtual_lane_mgr_->clear_lc_lanes();
  virtual_lane_mgr_->set_fix_lane(CURRENT_LANE);

  clear_lc_variables();
  auto &lane_status =
      PlanningContext::Instance()->mutable_planning_status()->lane_status;
  LaneStatus reset_lanestatus;
  lane_status = reset_lanestatus;
}

bool MSDStateMachine::check_head_crosss(VirtualLaneManager &virtual_lane_mgr,
                                        int direction) {
  constexpr double KDisThrCross = 0.4;
  VirtualLane &tlane = virtual_lane_mgr.mutable_target_lane();
  auto &fix_refline = virtual_lane_mgr.mutable_fix_refline();
  auto &f_refline = virtual_lane_mgr_->mutable_fix_refline();
  auto baseline_info = world_model_->get_baseline_info(f_refline.position());
  if (baseline_info == nullptr || !baseline_info->is_valid()) {
    MSD_LOG(ERROR, "check_head_crosss baseline_info is nullptr");
    return false;
  }
  auto &ego_state = baseline_info->get_ego_state();
  double v_ego = ego_state.ego_vel;
  double length = ConfigurationContext::Instance()->get_vehicle_param().length;
  double pre_length = v_ego * 0.5;
  pre_length = std::min(pre_length, 1.5 * length);
  if (!tlane.has_master()) {
    MSD_LOG(ERROR, "check_head_crosss tlane is null");
    return false;
  }
  Lane *tlane_master = tlane.master();
  double dis = -100;
  double pre_dis = 100;
  double width = ConfigurationContext::Instance()->get_vehicle_param().width;
  if (direction == LEFT_CHANGE) {
    dis = tlane_master->dis_to_side_line(0.0, width * 0.5, 1, fix_refline);
    pre_dis =
        tlane_master->dis_to_side_line(pre_length, width * 0.5, 1, fix_refline);
  }
  if (direction == RIGHT_CHANGE) {
    dis = tlane_master->dis_to_side_line(0.0, -width * 0.5, 0, fix_refline);
    pre_dis = tlane_master->dis_to_side_line(pre_length, -width * 0.5, 0,
                                             fix_refline);
  }

  MSD_LOG(INFO, "check_head_crosss dis is :%.2f, pre_dis is %.2f", dis,
          pre_dis);

  if (dis > 0.0 && pre_dis > KDisThrCross) {
    return true;
  }
  return false;
}

void MSDStateMachine::clear_lc_variables() {
  lc_valid_cnt_ = 0;
  lc_pause_id_ = -1000;
  lc_valid_ = true;
  lc_should_back_ = false;
  lc_valid_back_ = true;
  must_change_lane_ = false;
  lc_back_reason_ = "none";
  lc_invalid_reason_ = "none";
  invalid_back_reason_ = "none";
  lc_invalid_track_.reset();
  lc_back_track_.reset();
  near_cars_target_.clear();
  near_cars_origin_.clear();
  start_move_dist_lane_ = 0;
  should_premove_ = false;
  should_suspend_ = false;
  lc_pause_ = false;
}

void MSDStateMachine::clear_lc_pause_variables() {
  lc_pause_id_ = -1000;
  lc_pause_ = false;
}

void MSDStateMachine::calc_baseline_overlap(
    const std::shared_ptr<BaseLineInfo> &baseline_info,
    const PlanningStatus *planning_status) {
  const auto lane_status = planning_status->lane_status;
  static auto last_lane_status = lane_status.status;
  const auto ego_vel =
      world_model_->get_cart_ego_state_manager().get_cart_ego_state().ego_vel;

  double min_interval = std::max(2.0, ego_vel * 0.125);
  double total_l = 0, average_l = 0;
  int count = 0;
  double jitter = 0.0;
  static std::vector<double> last_frenet_enu_x{}, last_frenet_enu_y{};

  // check only for cp
  if (world_model_->is_ddmap() && !world_model_->is_acc_mode()) {
    // check baseline valid
    if (baseline_info != nullptr && baseline_info->is_valid()) {
      auto frenet_enu_x = baseline_info->get_frenet_enu_x();
      auto frenet_enu_y = baseline_info->get_frenet_enu_y();
      auto frenet_coord = baseline_info->get_frenet_coord();
      // check frenet valid
      if (frenet_coord != nullptr) {
        const double ego_s =
            std::min(baseline_info->get_ego_state().ego_frenet.x,
                     frenet_coord->GetLength() - 1.0);
        // check max s to 8s distance
        double max_s = std::min(5.0 * std::max(ego_vel, 2.0) + ego_s,
                                frenet_coord->GetLength());
        // size check and lane_status not changed
        if (!last_frenet_enu_x.empty() &&
            last_frenet_enu_x.size() == last_frenet_enu_y.size() &&
            last_lane_status == lane_status.status) {
          double last_s = 0.0;
          for (size_t i = 0; i < last_frenet_enu_x.size(); i++) {
            Point2D frenet_point, carte_point;
            carte_point.x = last_frenet_enu_x[i];
            carte_point.y = last_frenet_enu_y[i];
            if (frenet_coord->CartCoord2FrenetCoord(
                    carte_point, frenet_point) == TRANSFORM_FAILED) {
              continue;
            }
            if (frenet_point.x < ego_s || frenet_point.x > max_s) {
              continue;
            }
            if (frenet_point.x - last_s < min_interval) {
              continue;
            }

            const double distance_decay_ratio_coeff = 0.25;
            double distance_decay_ratio =
                (frenet_point.x - ego_s) / (max_s - ego_s);
            distance_decay_ratio =
                (1.0 - distance_decay_ratio * distance_decay_ratio_coeff);
            total_l += std::abs(frenet_point.y) * distance_decay_ratio;
            count++;

            last_s = frenet_point.x;

            MSD_LOG(INFO, "OVERLAP_DEBUG: ego_s %f max_s %f s %f l %f decay %f",
                    ego_s, max_s, frenet_point.x, frenet_point.y,
                    distance_decay_ratio);
          }
          if (count > 0) {
            average_l = total_l / count;
            jitter = average_l;

            MSD_LOG(INFO, "OVERLAP_DEBUG: count: %d, average_l: %f", count,
                    average_l);
          }
        }
      }
      last_frenet_enu_x = frenet_enu_x;
      last_frenet_enu_y = frenet_enu_y;
    } else {
      last_frenet_enu_x.clear();
      last_frenet_enu_y.clear();
    }
  } else {
    last_frenet_enu_x.clear();
    last_frenet_enu_y.clear();
  }

  world_model_->set_baseline_jitter(jitter);
  MSD_LOG(INFO, "OVERLAP_DEBUG: jitter: %f", jitter);

  last_lane_status = lane_status.status;
}

bool MSDStateMachine::check_lc_valid(VirtualLaneManager &virtual_lane_mgr,
                                     int direction, int property) {
  auto lc_result = compute_lc_valid_info(virtual_lane_mgr, direction, property);
  return lc_valid_;
}

LaneChangeStageInfo
MSDStateMachine::compute_lc_valid_info(VirtualLaneManager &virtual_lane_mgr,
                                       int direction, int property) {
  mph_assert(direction == LEFT_CHANGE || direction == RIGHT_CHANGE);
  LaneChangeStageInfo result_info;
  lc_valid_ = true;
  lc_invalid_track_.reset();
  lc_pause_id_ = -1000;
  lc_pause_ = false;

  auto &map_info = world_model_->get_mutable_map_info();
  auto &lateral_obstacle = world_model_->mutable_lateral_obstacle();
  auto &flane = virtual_lane_mgr.get_fix_lane();
  auto &f_refline = virtual_lane_mgr.mutable_fix_refline();
  auto baseline_info = world_model_->get_baseline_info(f_refline.position());
  double lane_width = flane.width();
  double left_lane_width = 3.8;
  double right_lane_width = 3.8;
  double car_width = 2.2;
  bool enable_new_gap_logic =
      ConfigurationContext::Instance()
          ->planner_config()
          .lateral_behavior_planner_config.enable_new_gap_logic;
  MSD_LOG(INFO, "enable_new_gap_logic : %d", enable_new_gap_logic);

  if (baseline_info == nullptr || !baseline_info->is_valid()) {
    result_info.lc_valid = false;
    lc_valid_ = false;
    return result_info;
  }
  auto &ego_state = baseline_info->get_ego_state();

  LaneTracksManager lane_tracks_mgr(
      world_model_->get_mutable_map_info_manager(), lateral_obstacle,
      virtual_lane_mgr);
  lane_tracks_mgr.update_ego_state(ego_state);

  auto &lateral_output =
      PlanningContext::Instance()->lateral_behavior_planner_output();
  auto &vel_sequence = lateral_output.vel_sequence;

  result_info.gap_insertable = true;
  result_info.gap_approached = true;
  result_info.gap_valid = true;
  result_info.side_approach = false;
  result_info.should_premove = should_premove_;
  result_info.gap = {-10, -10};
  lc_invalid_track_.reset();

  double coefficient = FLAGS_planning_loop_rate / 25;

  if (!map_info.is_in_intersection() &&
      !map_info.left_refline_points().empty()) {
    for (auto &p : map_info.left_refline_points()) {
      if (p.car_point.x >= 0 && p.car_point.x <= 20 && !p.in_intersection) {
        left_lane_width = p.lane_width;
        if (left_lane_width < 100) {
          break;
        }
      }
    }
  }

  if (!map_info.is_in_intersection() &&
      !map_info.right_refline_points().empty()) {
    for (auto &p : map_info.right_refline_points()) {
      if (p.car_point.x >= 0 && p.car_point.x <= 20 && !p.in_intersection) {
        right_lane_width = p.lane_width;
        if (right_lane_width < 100) {
          break;
        }
      }
    }
  }

  if (right_lane_width >= 100.) {
    right_lane_width = 3.8;
  }
  if (left_lane_width >= 100) {
    left_lane_width = 3.8;
  }

  if (!lateral_obstacle.sensors_okay()) {
    if (lateral_obstacle.fvf_dead()) {
      result_info.lc_invalid_reason = "no front view";
      lc_invalid_reason_ = "no front view";
    } else if (lateral_obstacle.svf_dead()) {
      lc_invalid_reason_ = "no side view";
      result_info.lc_invalid_reason = "no side view";
    }

    result_info.gap_valid = false;
    result_info.gap_approached = false;
    result_info.gap_insertable = false;
    result_info.lc_valid = false;
    lc_valid_ = false;
    lc_valid_cnt_ = 0;
    return result_info;
  }

  double v_ego = ego_state.ego_vel;
  double l_ego = ego_state.ego_frenet.y;
  double safety_dist_base = 2.0;
  if (ConfigurationContext::Instance()
          ->synthetic_config()
          .sensor_configuration != "lidar") {
    safety_dist_base = 4.0;
  }

  auto &fix_refline = virtual_lane_mgr.mutable_fix_refline();
  auto &target_lane = virtual_lane_mgr.mutable_target_lane();
  auto &origin_lane = virtual_lane_mgr.mutable_origin_lane();
  ScenarioFacadeConfig scenario_config =
      ConfigurationContext::Instance()->get_params().at(
          ScenarioFacadeConfig::ScenarioFacadeType::ChangelanePreparation);
  TaskConfig config =
      scenario_config
          .tasks_config_map_[TaskConfig::TaskType::LANE_CHANGE_DECIDER];
  // LaneChangeDeciderConfig lane_change_decider_config;
  // config.set_lane_change_decider_config(lane_change_decider_config);
  auto lane_change_decider = std::make_unique<LaneChangeDecider>(config);

  lane_change_decider->init(world_model_);
  // gap selection
  const auto &planning_status = PlanningContext::Instance()->planning_status();
  bool is_pre_in_wait_stage =
      planning_status.lane_status.status == LaneStatus::Status::LANE_CHANGE &&
      planning_status.lane_status.change_lane.status ==
          ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;

  int origin_lane_id = 0;
  int target_lane_id = 0;
  if (origin_lane.has_master()) {
    origin_lane_id = origin_lane.master()->position();
  }
  if (target_lane.has_master()) {
    target_lane_id = target_lane.master()->position();
  }

  MSD_LOG(INFO, "gap origin_lane_id %d target_lane_id %d ", origin_lane_id,
          target_lane_id);
  ScenarioFacadeContext scenario_context;
  *(scenario_context.mutable_planning_status()) =
      PlanningContext::Instance()->planning_status();
  scenario_context.mutable_planning_status()->planning_success = false;
  scenario_context.mutable_planning_status()->lane_status.target_lane_id =
      origin_lane_id;
  scenario_context.mutable_lateral_behavior_planner_output() =
      PlanningContext::Instance()->lateral_behavior_planner_output();
  int lc_request = lc_req_mgr_->request();
  if (lc_request == LEFT_CHANGE || lc_request == RIGHT_CHANGE) {
    scenario_context.mutable_planning_status()->lane_status.status =
        LaneStatus::Status::LANE_CHANGE;
    scenario_context.mutable_planning_status()->lane_status.change_lane.status =
        ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;
    scenario_context.mutable_planning_status()
        ->lane_status.change_lane.path_id = target_lane_id;
    scenario_context.mutable_planning_status()
        ->lane_status.change_lane.origin_lane_leader_id = -4;
    scenario_context.mutable_planning_status()
        ->lane_status.change_lane.direction =
        direction == LEFT_CHANGE ? "left" : "right";
    scenario_context.mutable_planning_status()
        ->lane_status.change_lane.is_active_lane_change =
        lc_req_mgr_->request_source() != MAP_REQUEST;
    scenario_context.mutable_planning_status()
        ->lane_status.change_lane.lane_change_wait_time =
        !is_pre_in_wait_stage || planning_status.planning_loop <= 1
            ? 0.0
            : get_system_time() - entry_time();
  }

  scenario_context.mutable_state_machine_output().must_change_lane =
      must_change_lane_;

  near_cars_target_.clear();
  std::vector<TrackInfo> near_cars_target;

  std::vector<TrackedObject> temp_target_tracks;
  std::vector<TrackedObject> *side_target_tracks =
      lane_tracks_mgr.get_lane_tracks(property, TrackType::SIDE_TRACK);
  // for (auto tr : lateral_obstacle.side_tracks()) {
  //   if (std::find(real_on_target_obstacle.begin(),
  //                 real_on_target_obstacle.end(),
  //                 tr.track_id) != real_on_target_obstacle.end())
  //     temp_target_tracks.push_back(tr);
  // }
  if (side_target_tracks != nullptr)
    side_target_tracks = &temp_target_tracks;

  if (side_target_tracks == nullptr) {
    result_info.lc_invalid_reason = "no side view";
    result_info.gap_valid = false;
    result_info.gap_approached = false;
    result_info.gap_insertable = false;
    lc_valid_ = false;
    lc_invalid_reason_ = "no side view";
    result_info.lc_valid = false;
    return result_info;
  }

  std::vector<TrackedObject> *front_target_tracks =
      lane_tracks_mgr.get_lane_tracks(property, FRONT_TRACK);

  if (front_target_tracks == nullptr) {
    result_info.lc_invalid_reason = "no front view";
    result_info.gap_valid = false;
    result_info.gap_approached = false;
    result_info.gap_insertable = false;
    result_info.lc_invalid_reason = "no front view";
    result_info.lc_valid = false;
    lc_valid_ = false;
    lc_invalid_reason_ = "no front view";
    return result_info;
  }
  // temp_target_tracks.clear();
  // for (auto tr : lateral_obstacle.front_tracks()) {
  //   if (std::find(real_on_target_obstacle.begin(),
  //                 real_on_target_obstacle.end(),
  //                 tr.track_id) != real_on_target_obstacle.end())
  //     temp_target_tracks.push_back(tr);
  // }
  if (front_target_tracks != nullptr)
    front_target_tracks = &temp_target_tracks;

  auto target_baseline = world_model_->get_baseline_info(target_lane_id);
  if (!target_baseline || !target_baseline->is_valid()) {
    result_info.gap_valid = false;
    result_info.gap_approached = false;
    result_info.gap_insertable = false;
    result_info.lc_invalid_reason = "invalid target lane";
    result_info.lc_valid = false;
    lc_invalid_reason_ = "invalid target lane";
    lc_valid_ = false;
    return result_info;
  }

  if (world_model_->get_baseline_info(origin_lane_id)) {
    // ------------- gmp overwrite ----------- //
    const auto &gmp_decision_info =
        PlanningContext::Instance()->general_motion_planner_output();

    const bool is_in_lc = (gmp_decision_info.lc_action_state == 1) ||
                          (gmp_decision_info.lc_action_state == 0 &&
                           gmp_decision_info.lc_wait == 1) ||
                          (gmp_decision_info.lc_action_state == -1);
    const bool lc_can_go = gmp_decision_info.lc_action_state == 1;
    const bool lc_should_wait = gmp_decision_info.lc_action_state == 0 &&
                                gmp_decision_info.lc_wait == 1;
    const bool &gmp_decision_valid = gmp_decision_info.gmp_valid;
    const int &gmp_gap_front_id = gmp_decision_info.gap_info[0].id;
    const int &gmp_gap_rear_id = gmp_decision_info.gap_info[1].id;

    int gmp_motion_result = gmp_decision_info.motion_result;
    result_info.should_ready = is_in_lc && gmp_motion_result <= 4 &&
                               gmp_motion_result >= 1 && lc_should_wait;
    if (enable_new_gap_logic) {
      if (is_in_lc && gmp_decision_valid && lc_can_go) {
        std::pair<int, int> gmp_gap =
            make_pair(gmp_gap_front_id, gmp_gap_rear_id);
        result_info.gap = gmp_gap;
        lc_valid_ = true;
        result_info.gap_valid = true;
        result_info.gap_approached = true;
        result_info.gap_insertable = true;
        // result_info.lc_invalid_reason = "none";
        result_info.lc_valid = true;

        MSD_LOG(INFO, "GMP decision overwriting 1");
        PlanningContext::Instance()
            ->mutable_planning_status()
            ->lane_status.change_lane.target_gap_obs = gmp_gap;
        return result_info;
      } else if (is_in_lc && lc_should_wait) {
        std::pair<int, int> gmp_gap =
            make_pair(gmp_gap_front_id, gmp_gap_rear_id);
        result_info.gap = gmp_gap;
        lc_valid_ = false;
        lc_invalid_reason_ = "gmp should wait";
        result_info.gap_valid = false;
        result_info.gap_approached = false;
        result_info.gap_insertable = false;
        result_info.lc_invalid_reason = "gmp should wait";
        result_info.lc_valid = false;

        MSD_LOG(INFO, "GMP decision overwritting2");
        PlanningContext::Instance()
            ->mutable_planning_status()
            ->lane_status.change_lane.target_gap_obs = gmp_gap;
        return result_info;
      }
    }
    // -------------- gmp overwrite -----------//
    auto gap = lane_change_decider->get_target_gap();
    result_info.gap = gap;
    if (gap.first == -10 && gap.second == -10) {
      scenario_context.mutable_planning_status()
          ->lane_status.change_lane.target_gap_obs = gap;
      lc_valid_ = false;
      lc_invalid_reason_ = "no valid gap";
      result_info.gap_valid = false;
      result_info.gap_approached = false;
      result_info.gap_insertable = false;
      result_info.lc_invalid_reason = "no valid gap";
      result_info.lc_valid = false;
    }
    if (gap.first >= 0 || gap.second >= 0) {
      result_info.target_lane_traffic_flow =
          lane_change_decider->get_target_lane_traffic_flow();
      result_info.gap_valid = true;
      if ((lane_change_decider->get_cloest_front_id() != gap.first &&
           gap.first > 0) ||
          lane_change_decider->get_cloest_rear_id() != gap.second &&
              gap.second > 0) {
        MSD_LOG(INFO, "gap_not_match target [%d] [%d] cloest [%d] [%d]",
                gap.first, gap.second,
                lane_change_decider->get_cloest_front_id(),
                lane_change_decider->get_cloest_rear_id());
        result_info.lc_invalid_reason = "gap does not match";
        result_info.lc_valid = true;
        result_info.gap_insertable = false;
        return result_info;
      }
      // distance protection
      constexpr double KMinFollowDistance = 4.0;
      constexpr double KMinOvertakeDistance = 4.0;
      const std::array<double, 2> xp{8.3, 30.3};
      const std::array<double, 2> fp{4.0, 20.0};

      double follow_distance =
          std::max(KMinFollowDistance, interp(v_ego, xp, fp));
      double overtake_distance =
          std::max(KMinOvertakeDistance, interp(v_ego, xp, fp));
      if (lane_change_decider->get_cloest_front_dis() < follow_distance ||
          lane_change_decider->get_cloest_rear_dis() < overtake_distance) {
        result_info.lc_invalid_reason = "gap too close";
        result_info.lc_valid = true;
        result_info.gap_insertable = false;
        return result_info;
      }
      constexpr double KPredictLeadNTTC = 3.0;
      constexpr double KLonNTTC = 1.0;

      if (lateral_obstacle.leadone() != nullptr) {
        auto leadone_obj = lateral_obstacle.leadone();
        double v_leadone = leadone_obj->v;
        if (leadone_obj->v - v_ego < 0 &&
            ((v_ego - leadone_obj->v) * KPredictLeadNTTC +
                 KLonNTTC * leadone_obj->v >
             leadone_obj->d_rel)) {
          double rear_dis = lane_change_decider->get_cloest_rear_dis();
          TrackedObject rear_tr;
          if (lateral_obstacle.find_track(
                  lane_change_decider->get_cloest_rear_id(), rear_tr)) {
            double d_rel_lead =
                -rear_dis -
                ConfigurationContext::Instance()->get_vehicle_param().length -
                leadone_obj->d_rel;
            double d_rel_lead_pred =
                d_rel_lead + (rear_tr.v - leadone_obj->v) * KPredictLeadNTTC;
            if (KLonNTTC * leadone_obj->v - d_rel_lead_pred <
                KMinOvertakeDistance) {
              result_info.lc_invalid_reason = "gap too close";
              result_info.lc_valid = true;
              result_info.gap_insertable = false;
              return result_info;
            }
          }
        }
      }
      // ttc protection
      constexpr double KMinFollowTTC = 5.0;
      constexpr double KMinOvertakeTTC = 5.0;
      constexpr double KMinFollowNTTC = 4.0;
      constexpr double KMinOvertakeNTTC = 5.0;
      MSD_LOG(INFO, "COMPUTE_LC_VALID valid front ttc[%.2f]",
              lane_change_decider->get_cloest_front_ttc());
      MSD_LOG(INFO, "COMPUTE_LC_VALID valid rear ttc[%.2f]",
              lane_change_decider->get_cloest_rear_ttc());
      if (lane_change_decider->get_cloest_front_ttc() < KMinFollowTTC ||
          lane_change_decider->get_cloest_rear_ttc() < KMinOvertakeTTC ||
          lane_change_decider->get_cloest_front_nttc() < KMinFollowNTTC ||
          lane_change_decider->get_cloest_rear_nttc() < KMinOvertakeNTTC) {
        result_info.lc_invalid_reason = "gap narrow";
        result_info.lc_valid = true;
        result_info.gap_insertable = false;
        return result_info;
      }
      if (lane_change_decider->is_target_gap_frozen()) {
        result_info.lc_invalid_reason = "gap frozen";
        result_info.gap_approached = false;
        result_info.gap_insertable = false;
      }
    }
    auto prev_gap = PlanningContext::Instance()
                        ->planning_status()
                        .lane_status.change_lane.target_gap_obs;
    scenario_context.mutable_planning_status()
        ->lane_status.change_lane.target_gap_obs = gap;
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->lane_status.change_lane.target_gap_obs = gap;
    const auto &prev_follow_obstacles =
        PlanningContext::Instance()
            ->planning_status()
            .planning_result.lon_follow_obstacles;
    if (gap.first != PlanningContext::Instance()
                         ->planning_status()
                         .lane_status.change_lane.target_gap_obs.first &&
        gap.second != PlanningContext::Instance()
                          ->planning_status()
                          .lane_status.change_lane.target_gap_obs.second) {
      if (std::find(prev_follow_obstacles.begin(), prev_follow_obstacles.end(),
                    prev_gap.second) != prev_follow_obstacles.end()) {
        MSD_LOG(INFO, "**************** gap begin enable gap protection!");
        result_info.enable_gap_protection = true;
        result_info.gap_protection_counter = 5;
        result_info.gap_insertable = false;
        result_info.lc_invalid_reason = "gap protection";
      }
    }
    if (PlanningContext::Instance()
            ->mutable_planning_status()
            ->lane_status.change_lane.gap_protection_counter > 0) {
      MSD_LOG(INFO, "************* gap enable gap protection! counter %d",
              PlanningContext::Instance()
                  ->mutable_planning_status()
                  ->lane_status.change_lane.gap_protection_counter);
      result_info.gap_protection_counter =
          PlanningContext::Instance()
              ->mutable_planning_status()
              ->lane_status.change_lane.gap_protection_counter -
          1;
      result_info.enable_gap_protection = true;
      result_info.gap_insertable = false;
      result_info.lc_invalid_reason = "gap protection";
    } else {
      if (!result_info.enable_gap_protection) {
        result_info.gap_protection_counter = 0;
      }
    }
  } else {
    lc_valid_ = false;
    lc_invalid_reason_ = "invalid gap selection";
    scenario_context.mutable_planning_status()
        ->lane_status.change_lane.target_gap_obs = {-10, -10};
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->lane_status.change_lane.target_gap_obs = {-10, -10};
    result_info.gap_valid = false;
    result_info.gap_approached = false;
    result_info.gap_insertable = false;
    result_info.lc_invalid_reason = "invalid gap selection";
    return result_info;
  }
  double lc_valid_thre = 10.0 * coefficient;
  if (lc_valid_) {
    lc_valid_cnt_ += 1;

    if (lc_valid_cnt_ > lc_valid_thre) {
      lc_valid_cnt_ = 0;
    } else {
      lc_valid_ = false;
      lc_invalid_reason_ = "valid cnt below threshold";
    }
  } else {
    lc_valid_cnt_ = 0;
  }

  return result_info;
}

LaneChangeStageInfo
MSDStateMachine::decide_lc_valid_info(VirtualLaneManager &virtual_lane_mgr,
                                      int direction, int property) {
  auto raw_info = compute_lc_valid_info(virtual_lane_mgr, direction, property);
  auto &map_info = world_model_->get_mutable_map_info();
  double coefficient = FLAGS_planning_loop_rate / 25;
  int lc_valid_thre = static_cast<int>(10.0 * coefficient);
  auto &f_refline = virtual_lane_mgr.mutable_fix_refline();
  auto baseline_info = world_model_->get_baseline_info(f_refline.position());
  auto &ego_state = baseline_info->get_ego_state();
  double v_ego = ego_state.ego_vel;
  if (raw_info.enable_interactive_mode) {
    lc_valid_thre = static_cast<int>(5.0 * coefficient);
  }
  // update
  lc_invalid_reason_ = raw_info.lc_invalid_reason;
  return raw_info;
}

bool MSDStateMachine::check_lc_body_valid(int direction) {
  is_lca_state_activated_ = false;
  is_steer_over_limit_ = false;

  double KLimitedRad = ConfigurationContext::Instance()
          ->planner_config()
          .lateral_behavior_planner_config.angle_lc_limit / 180.0 * M_PI;
  auto ego_steer_manager = world_model_->get_cart_ego_state_manager();
  auto cart_ego = ego_steer_manager.get_cart_ego_state();
  if (std::fabs(cart_ego.ego_steer_angle) > KLimitedRad) {
    is_steer_over_limit_ = true;
    return false;
  }

  auto lca_state = world_model_->get_lca_status();
  if (direction == LEFT_CHANGE && lca_state.lca_left_activated) {
    is_lca_state_activated_ = true;
    return false;
  }

  if (direction == RIGHT_CHANGE && lca_state.lca_right_activated) {
    is_lca_state_activated_ = true;
    return false;
  }

  return true;
}

LaneChangeStageInfo
MSDStateMachine::compute_lc_back_info(VirtualLaneManager &virtual_lane_mgr,
                                      int direction) {
  LaneChangeStageInfo result;

  auto &map_info = world_model_->get_map_info();
  auto &lateral_obstacle = world_model_->mutable_lateral_obstacle();
  auto &f_refline = virtual_lane_mgr.mutable_fix_refline();
  auto baseline_info = world_model_->get_baseline_info(f_refline.position());
  if (baseline_info == nullptr || !baseline_info->is_valid()) {
    return result;
  }
  auto &ego_state = baseline_info->get_ego_state();
  auto &flane = virtual_lane_mgr.get_fix_lane();

  LaneTracksManager lane_tracks_mgr(
      world_model_->get_mutable_map_info_manager(), lateral_obstacle,
      virtual_lane_mgr);
  lane_tracks_mgr.update_ego_state(ego_state);

  double move_thre = 0.4;
  double lane_width = flane.width();
  double left_lane_width = 3.8;
  double right_lane_width = 3.8;
  double car_width = 2.2;
  double l_ego = ego_state.ego_frenet.y;

  auto &fix_refline = virtual_lane_mgr.get_fix_refline();
  auto &target_lane = virtual_lane_mgr.mutable_target_lane();
  auto &origin_lane = virtual_lane_mgr.mutable_origin_lane();
  lc_back_track_.reset();
  ScenarioFacadeConfig scenario_config =
      ConfigurationContext::Instance()->get_params().at(
          ScenarioFacadeConfig::ScenarioFacadeType::ChangelanePreparation);
  TaskConfig config =
      scenario_config
          .tasks_config_map_[TaskConfig::TaskType::LANE_CHANGE_DECIDER];

  auto lane_change_decider = std::make_unique<LaneChangeDecider>(config);

  lane_change_decider->init(world_model_);
  // gap selection
  const auto &planning_status = PlanningContext::Instance()->planning_status();
  bool is_pre_in_wait_stage =
      planning_status.lane_status.status == LaneStatus::Status::LANE_CHANGE &&
      planning_status.lane_status.change_lane.status ==
          ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;

  int origin_lane_id = 0;
  int target_lane_id = 0;
  if (origin_lane.has_master()) {
    origin_lane_id = origin_lane.master()->position();
  }
  if (target_lane.has_master()) {
    target_lane_id = target_lane.master()->position();
  }

  MSD_LOG(INFO, "gap origin_lane_id %d target_lane_id %d ", origin_lane_id,
          target_lane_id);
  ScenarioFacadeContext scenario_context;
  *(scenario_context.mutable_planning_status()) =
      PlanningContext::Instance()->planning_status();
  scenario_context.mutable_planning_status()->planning_success = false;
  scenario_context.mutable_planning_status()->lane_status.target_lane_id =
      origin_lane_id;
  scenario_context.mutable_lateral_behavior_planner_output() =
      PlanningContext::Instance()->lateral_behavior_planner_output();
  int lc_request = lc_req_mgr_->request();
  if (lc_request == LEFT_CHANGE || lc_request == RIGHT_CHANGE) {
    scenario_context.mutable_planning_status()->lane_status.status =
        LaneStatus::Status::LANE_CHANGE;
    scenario_context.mutable_planning_status()->lane_status.change_lane.status =
        ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;
    scenario_context.mutable_planning_status()
        ->lane_status.change_lane.path_id = target_lane_id;
    scenario_context.mutable_planning_status()
        ->lane_status.change_lane.origin_lane_leader_id = -4;
    scenario_context.mutable_planning_status()
        ->lane_status.change_lane.direction =
        direction == LEFT_CHANGE ? "left" : "right";
    scenario_context.mutable_planning_status()
        ->lane_status.change_lane.is_active_lane_change =
        lc_req_mgr_->request_source() != MAP_REQUEST;
    scenario_context.mutable_planning_status()
        ->lane_status.change_lane.lane_change_wait_time =
        !is_pre_in_wait_stage || planning_status.planning_loop <= 1
            ? 0.0
            : get_system_time() - entry_time();
  }

  scenario_context.mutable_state_machine_output().must_change_lane =
      must_change_lane_;
  bool enable_new_gap_logic =
      ConfigurationContext::Instance()
          ->planner_config()
          .lateral_behavior_planner_config.enable_new_gap_logic;
  MSD_LOG(INFO, "enable_new_gap_logic : %d", enable_new_gap_logic);
  double distance = virtual_lane_mgr_->dis_to_fixrefline();
  double lat_move_thr = 1.0;
  const double KbackTTC = 4.5;
  const double KbackNTTC = 4.0;
  const double KbackDis = 2.0;
  // -------------- gmp overwrite -----------------//
  const auto &gmp_decision_info =
      PlanningContext::Instance()->general_motion_planner_output();

  const bool is_in_lc = (gmp_decision_info.lc_action_state == 1) ||
                        (gmp_decision_info.lc_action_state == 0 &&
                         gmp_decision_info.lc_wait == 1) ||
                        (gmp_decision_info.lc_action_state == -1);
  const bool gmp_lc_should_back = gmp_decision_info.lc_action_state == -1;
  const bool &gmp_decision_valid = gmp_decision_info.gmp_valid;

  MSD_LOG(INFO, "GMP decision is_in_lc:%d", is_in_lc);

  if (enable_new_gap_logic) {
    if (is_in_lc && gmp_lc_should_back) {
      result.lc_back_reason = "gmp fail";
      result.lc_should_back = true;

      MSD_LOG(INFO, "GMP decision overwritting-1");
      return result;
    } else {
      result.lc_should_back = false;
      MSD_LOG(INFO, "GMP decision overwritting-2");
      return result;
    }
  }
  // -------------- gmp overwrite -----------------//
  if (world_model_->get_baseline_info(origin_lane_id) &&
      lane_change_decider->updata_real_obstacle_on_target(
          EXTERNAL, origin_lane_id, target_lane_id)) {
    MSD_LOG(INFO, "COMPUTE_LC_VALID front ttc[%.2f]",
            lane_change_decider->get_cloest_front_ttc());
    MSD_LOG(INFO, "COMPUTE_LC_VALID rear ttc[%.2f]",
            lane_change_decider->get_cloest_rear_ttc());
    MSD_LOG(INFO, "COMPUTE_LC_VALID front nttc[%.2f]",
            lane_change_decider->get_cloest_front_nttc());
    MSD_LOG(INFO, "COMPUTE_LC_VALID rear nttc[%.2f]",
            lane_change_decider->get_cloest_rear_nttc());

    if (fix_refline.position() != 0 ||
        ((direction == LEFT_CHANGE && std::fabs(start_move_dist_lane_) > 1e-5 &&
          distance < -left_lane_width / 2 - lat_move_thr + move_thre) ||
         (direction == RIGHT_CHANGE &&
          std::fabs(start_move_dist_lane_) > 1e-5 &&
          distance > right_lane_width / 2 + lat_move_thr - move_thre))) {
      if (lane_change_decider->get_cloest_rear_nttc() < KbackNTTC - 1) {
        result.lc_back_reason = "rear too close ttc";
        result.lc_should_back = true;
      }
      if (lane_change_decider->get_cloest_rear_ttc() < KbackTTC ||
          lane_change_decider->get_cloest_rear_nttc() < KbackNTTC) {
        result.lc_back_reason = "rear close ttc";
        result.lc_should_back = true;
      }
      if ((lane_change_decider->get_cloest_rear_dis() < KbackDis &&
           lane_change_decider->get_cloest_rear_dis() > 0 &&
           lane_change_decider->get_cloest_rear_dis_p1() > -KbackDis) ||
          (lane_change_decider->get_cloest_front_dis() < KbackDis &&
           lane_change_decider->get_cloest_front_dis() >= 0 &&
           lane_change_decider->get_cloest_front_dis_p1() < KbackDis)) {
        result.lc_back_reason = "rear close distance";
        result.lc_should_back = true;
      }
    }
  }

  if (!map_info.is_in_intersection() &&
      !map_info.left_refline_points().empty()) {
    for (auto &p : map_info.left_refline_points()) {
      if (p.car_point.x >= 0 && p.car_point.x <= 20 && !p.in_intersection) {
        left_lane_width = p.lane_width;
        if (left_lane_width < 100) {
          break;
        }
      }
    }
  }

  if (!map_info.is_in_intersection() &&
      !map_info.right_refline_points().empty()) {
    for (auto &p : map_info.right_refline_points()) {
      if (p.car_point.x >= 0 && p.car_point.x <= 20 && !p.in_intersection) {
        right_lane_width = p.lane_width;
        if (right_lane_width < 100) {
          break;
        }
      }
    }
  }

  if (!lateral_obstacle.sensors_okay() &&
      virtual_lane_mgr.is_on_lane(ORIGIN_LANE)) {
    if (lateral_obstacle.fvf_dead()) {
      result.lc_back_reason = "no front view";
    } else if (lateral_obstacle.svf_dead()) {
      result.lc_back_reason = "no side view";
    }
    result.lc_should_back = true;
    return result;
  }

  double mss = 0.0;
  double mss_t = 0.0;

  double v_ego = ego_state.ego_vel;
  double safety_dist_base = 2.0;
  if (ConfigurationContext::Instance()
          ->synthetic_config()
          .sensor_configuration != "lidar") {
    safety_dist_base = 4.0;
  }
  double safety_dist = v_ego * 0.2 + safety_dist_base;
  double dist_mline = virtual_lane_mgr.dist_mline(direction);
  double pause_ttc = 2.0;
  double pause_v_rel = 2.0;

  double t_reaction =
      (std::fabs(dist_mline - DBL_MAX) < 1e-2) ? 1.0 : 1.0 * dist_mline / 1.8;

  near_cars_target_.clear();

  std::vector<TrackInfo> near_cars_target;
  std::vector<TrackedObject> *side_target_tracks =
      lane_tracks_mgr.get_lane_tracks(TARGET_LANE, SIDE_TRACK);

  if ((direction == LEFT_CHANGE && std::fabs(start_move_dist_lane_) > 1e-5 &&
       distance > -left_lane_width / 2 - move_thre) ||
      (direction == RIGHT_CHANGE && std::fabs(start_move_dist_lane_) > 1e-5 &&
       distance < right_lane_width / 2 + move_thre) ||
      fix_refline.position() == 0) {
    if (result.lc_should_back && result.tr_pause_dv > pause_v_rel &&
        -result.tr_pause_s / result.tr_pause_dv < pause_ttc &&
        ((direction == LEFT_CHANGE && result.tr_pause_l - distance > 0.5) ||
         (direction == RIGHT_CHANGE && distance - result.tr_pause_l > 0.5))) {
      result.lc_pause = true;
    }
    result.lc_should_back = false;
    result.lc_back_reason = "exceed move_thre, do not back";
    MSD_LOG(INFO,
            "borrow first lc_pause:[%d], lc_pause_id[%d], "
            "result.tr_pause_dv[%f], pause_v_rel[%f] "
            "result.tr_pause_s / result.tr_pause_dv[%f], pause_ttc[%f], "
            "distance[%f], result.tr_pause_l[%f]",
            result.lc_pause, result.lc_pause_id, result.tr_pause_dv,
            pause_v_rel, (result.tr_pause_s / result.tr_pause_dv), pause_ttc,
            distance, result.tr_pause_l);
    return result;
  } else if (result.lc_should_back &&
             lc_req_mgr_->request_source() == MAP_REQUEST && v_ego < 2.) {
    if (direction == LEFT_CHANGE) {
      if (left_lane_width / 2 - l_ego - car_width / 2 > 3.) {
        result.lc_pause = true;
      }
      result.lc_should_back = false;
    } else if (direction == RIGHT_CHANGE) {
      if (right_lane_width / 2 + l_ego - car_width / 2 > 3.) {
        result.lc_pause = true;
      }
      result.lc_should_back = false;
    }
  }

  MSD_LOG(INFO,
          "borrow lc_pause:[%d], lc_pause_id[%d], result.lc_should_back[%d], "
          "result.tr_pause_dv[%f], pause_v_rel[%f] "
          "result.tr_pause_s / result.tr_pause_dv[%f], pause_ttc[%f], "
          "distance[%f], result.tr_pause_l[%f]",
          result.lc_pause, result.lc_pause_id, result.lc_should_back,
          result.tr_pause_dv, pause_v_rel,
          (result.tr_pause_s / result.tr_pause_dv), pause_ttc, distance,
          result.tr_pause_l);

  std::vector<TrackedObject> *front_target_tracks =
      lane_tracks_mgr.get_lane_tracks(TARGET_LANE, FRONT_TRACK);

  if (front_target_tracks != nullptr) {
    for (auto &tr : *front_target_tracks) {
      TrackInfo front_track(tr.track_id, tr.d_rel, tr.v_rel);
      near_cars_target_.push_back(front_track);
    }
    for (auto &tr : *front_target_tracks) {
      auto &x = tr.trajectory.x;
      auto &y = tr.trajectory.y;
      std::vector<double> ego_x;
      std::vector<double> ego_y;
      std::vector<double> ego_speed;
      std::vector<double> ego_yaw;
      double ego_fx = std::cos(ego_state.ego_pose_raw.theta);
      double ego_fy = std::sin(ego_state.ego_pose_raw.theta);
      double ego_lx = -ego_fy;
      double ego_ly = ego_fx;
      double theta = 0.0;
      double send = 0.0;
      double lend = 0.0;
      int end_idx = 0;
      if (tr.trajectory.intersection == 0) {
        for (int i = 0; i < x.size(); i++) {
          double dx = x[i] - ego_state.ego_pose_raw.x;
          double dy = y[i] - ego_state.ego_pose_raw.y;

          double rel_x = dx * ego_fx + dy * ego_fy;
          double rel_y = dx * ego_lx + dy * ego_ly;
          ego_x.push_back(rel_x);
          ego_y.push_back(rel_y);
        }
        int end_range = min((int)ego_x.size() - 1, 25);
        for (int i = end_range; i >= 0; i--) {
          if (ego_x[i] < 0 || ego_x[i] > 80) {
            continue;
          }

          end_idx = i;
          break;
        }
        f_refline.cartesian_frenet(ego_x[end_idx], ego_y[end_idx], send, lend,
                                   theta, false);
      }
      if (!result.lc_should_back &&
          ((direction == LEFT_CHANGE &&
            lend < car_width + 0.3 - lane_width / 2 + tr.width / 2) ||
           (direction == RIGHT_CHANGE &&
            lend > -(car_width + 0.3) + lane_width / 2 - tr.width / 2) ||
           tr.trajectory.intersection != 0)) {
        std::array<double, 2> a_dec_v{3.0, 2.0};
        std::array<double, 2> v_ego_bp{6, 20};
        double a_dflc = interp(v_ego, v_ego_bp, a_dec_v);

        auto driving_styleConfig =
            ConfigurationContext::Instance()->driving_styleConfig();
        auto lc_back_front_car_accel_ratio =
            driving_styleConfig.reader
                .get<std::vector<double>>("lc_back_front_car_accel_ratio")
                .at(int(driving_styleConfig.driving_style));
        MSD_LOG(INFO,
                "driving_style params,  lc_back_front_car_accel_ratio : %.2f",
                lc_back_front_car_accel_ratio);
        a_dflc *= lc_back_front_car_accel_ratio;

        auto lc_back_front_car_safety_dist_ratio =
            driving_styleConfig.reader
                .get<std::vector<double>>("lc_back_front_car_safety_dist_ratio")
                .at(int(driving_styleConfig.driving_style));
        MSD_LOG(INFO,
                "driving_style params,  lc_back_front_car_safety_dist_ratio "
                ": %.2f",
                lc_back_front_car_safety_dist_ratio);
        safety_dist *= lc_back_front_car_safety_dist_ratio;

        if (tr.v_rel < 0) {
          mss = tr.v_rel * tr.v_rel / (2 * a_dflc) + safety_dist;
        } else {
          mss = std::max(-tr.v_rel * tr.v_rel / 4 + safety_dist, 2.0);
        }

        double lat_condi = std::max((std::max(l_ego - tr.d_max_cpath - 1.6,
                                              tr.d_min_cpath - l_ego - 1.6)),
                                    0.0) /
                           (std::max(-tr.v_lat + 0.7, 0.3));
        if ((lat_condi < 1.5 && tr.d_rel < 1.0) ||
            (lat_condi <= 1.5 &&
             tr.d_rel < 0.8 * (mss - std::max(lat_condi * tr.v_rel, 0.0)))) {
          result.lc_should_back = true;
          result.lc_back_reason = "front view back";
          lc_back_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
          if (lc_req_mgr_->request_source() == MAP_REQUEST && v_ego < 2.) {
            result.lc_should_back = false;
          }
        }
      } else {
        break;
      }
    }
  }

  return result;
}

LaneChangeStageInfo
MSDStateMachine::decide_lc_back_info(VirtualLaneManager &virtual_lane_mgr,
                                     int direction) {
  auto lc_info = compute_lc_back_info(virtual_lane_mgr, direction);
  double coefficient = FLAGS_planning_loop_rate / 25;
  // int 2
  int lc_back_thre = static_cast<int>(5 * coefficient);
  if (lc_info.lc_should_back) {
    lc_back_cnt_ += 1;
    MSD_LOG(INFO, "COMPUTE_LC_VALID lc_back_cnt_[%d]", lc_back_cnt_);
    if (lc_back_cnt_ > lc_back_thre ||
        lc_info.lc_back_reason == "rear too close ttc") {
      // reset lc_back_cnt_ when choose back finally
    } else {
      lc_info.lc_should_back = false;
      lc_info.lc_back_reason = "but back cnt below threshold";
    }
  } else {
    lc_back_cnt_ = 0;
  }
  double lc_change_time = get_system_time() - entry_time();
  if (!ego_state_queue_.empty()) {
    double average_v = 0.0;
    double max_v = 0.0;
    for (const auto &ego_state : ego_state_queue_) {
      average_v += ego_state.ego_vel;
      max_v = std::max(max_v, ego_state.ego_vel);
    }
    average_v /= double(ego_state_queue_.size());
    MSD_LOG(INFO, "[decide_lc_back_info] change time %f average_v %f",
            lc_change_time, average_v);
    if (lc_change_time > 8.0 && std::abs(average_v) < 0.2 && max_v < 0.4) {
      lc_info.lc_should_reset = true;
      if (!lc_info.lc_should_back) {
        lc_back_reason_ = "lc_block";
      }
    }
  }
  lc_back_reason_ = lc_info.lc_back_reason;
  return lc_info;
}

bool MSDStateMachine::check_lc_finish(VirtualLaneManager &virtual_lane_mgr,
                                      RequestType direction) {
  auto &map_info = world_model_->get_mutable_map_info();

  mph_assert(virtual_lane_mgr.has_target_lane());

  auto &fix_refline = virtual_lane_mgr.mutable_fix_refline();

  auto &lateral_output =
      PlanningContext::Instance()->lateral_behavior_planner_output();
  auto baseline_info = world_model_->get_baseline_info(fix_refline.position());
  auto &ego_state = baseline_info->get_ego_state();

  if (fix_refline.path_points().empty()) {
    return false;
  }

  double dist_threshold = 0.3;
  double v_ego = ego_state.ego_vel;

  if (fix_refline.position() != 0) {
    return false;
  }

  int position = fix_refline.position();
  double target_lane_width = 3.8;
  auto target_baseline = world_model_->get_baseline_info(position);
  std::array<double, 3> angle_thre_v{0.72, 0.48, 0.12};
  std::array<double, 3> angle_thre_bp{1.0, 3.0, 5.0};
  double angle_threshold = interp(v_ego, angle_thre_bp, angle_thre_v);
  if (target_baseline == nullptr || !target_baseline->is_valid()) {
    return false;
  }
  if (map_info.current_tasks_id() == 0 &&
      lateral_output.lc_request_source == "map_request") {
    return target_baseline->is_adc_on_lane();
  } else {
    double s, l, theta;
    fix_refline.cartesian_frenet(0, 0, s, l, theta, true);

    if (!map_info.is_in_intersection()) {
      if (direction == RIGHT_CHANGE) {
        return (l <
                dist_threshold + std::min(lateral_output.lat_offset, 0.0)) &&
               (std::fabs(theta) < angle_threshold);
      } else if (direction == LEFT_CHANGE) {
        return (l >
                -dist_threshold + std::min(lateral_output.lat_offset, 0.0)) &&
               (std::fabs(theta) < angle_threshold);
      } else {
        MSD_LOG(ERROR, "[check_lc_finish] invalid direction[%d]", direction);
        return true;
      }
    }
    return false;
  }
}

bool MSDStateMachine::check_lc_back_finish(VirtualLaneManager &virtual_lane_mgr,
                                           RequestType direction) {
  mph_assert(virtual_lane_mgr.has_origin_lane());

  auto &fix_refline = virtual_lane_mgr.mutable_fix_refline();
  auto &flane = virtual_lane_mgr.get_fix_lane();
  double lane_width = flane.width();

  if (fix_refline.path_points().empty()) {
    return false;
  }

  double dist_thre =
      0.5 * (lane_width -
             ConfigurationContext::Instance()->get_vehicle_param().width) -
      0.1;
  double angle_thre = 0.01;

  if (fix_refline.position() != 0) {
    return false;
  }

  double s;
  double l = 0.0;
  double theta = 0.0;
  fix_refline.cartesian_frenet(0, 0, s, l, theta, true);

  if (direction == LEFT_CHANGE) {
    return l < dist_thre && std::fabs(theta) < angle_thre;
  } else if (direction == RIGHT_CHANGE) {
    return l > -dist_thre && std::fabs(theta) < angle_thre;
  } else {
    MSD_LOG(ERROR, "[check_lc_back_finish] invalid direction[%d]", direction);
  }

  return true;
}

void MSDStateMachine::post_process() {
  int state = fsm_context_.state;

  auto planning_status = PlanningContext::Instance()->mutable_planning_status();
  if (!planning_status->planning_success) {
    virtual_lane_mgr_->update_fix_lane_info();
  }

  if (get_dist_lane_) {
    start_move_dist_lane_ = virtual_lane_mgr_->dis_to_fixrefline();
    get_dist_lane_ = false;
  }

  turn_light_ = lc_req_mgr_->turn_light();

  if (!planning_status->planning_success) {
    scenario_process();
  } else {
    // todo(@zzd): copy chosen scenario context to planning context
    // copy result from optimal scenario context to planning context
    // *PlanningContext::Instance()->mutable_prev_planning_status() =
    //     PlanningContext::Instance()->planning_status();
    // PlanningContext::Instance()->mutable_lateral_behavior_planner_output()
    // =
    //     optimal_scenario_context_->lateral_behavior_planner_output();
    // *PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()
    // =
    //     optimal_scenario_context_->longitudinal_behavior_planner_output();
    // *PlanningContext::Instance()->mutable_longitudinal_motion_planner_output()
    // =
    //     optimal_scenario_context_->longitudinal_motion_planner_output();
    // *PlanningContext::Instance()->mutable_speed_limit() =
    //     optimal_scenario_context_->speed_limit();
    // *PlanningContext::Instance()->mutable_planning_status() =
    //     optimal_scenario_context_->planning_status();
    // PlanningContext::Instance()->mutable_obstacle_decision_manager() =
    //     optimal_scenario_context_->obstacle_decision_manager();
    should_premove_ =
        PlanningContext::Instance()->state_machine_output().should_premove;
    update_decision_result();
  }
}

void MSDStateMachine::scenario_process() {
  // todo(@zzd): move this code to preprocess every loop
  auto planning_status = PlanningContext::Instance()->mutable_planning_status();
  // planning_status->scheme_stage = SchemeStage::PRIMARY;

  ScenarioFacadeConfig scenario_config;
  // TODO(@zzd) construct specific scenario for post process
  scenario_config = ConfigurationContext::Instance()->get_params().at(
      ScenarioFacadeConfig::ScenarioFacadeType::FixLaneCruise);
  auto fix_lane_scenario_facade =
      std::make_shared<ScenarioFacade>(scenario_config);
  fix_lane_scenario_facade->reset(scenario_config);
  fix_lane_scenario_facade->init();
  auto scenario_context = fix_lane_scenario_facade->get_scenario_context();
  // todo(@zzd) construct obs_decision_manager in scenario context
  *scenario_context->mutable_planning_status() =
      PlanningContext::Instance()->planning_status();
  scenario_context->mutable_planning_status()->planning_success = false;
  scenario_context->mutable_lateral_behavior_planner_output() =
      PlanningContext::Instance()->lateral_behavior_planner_output();
  scenario_context->mutable_lateral_motion_planner_output() =
      PlanningContext::Instance()->lateral_motion_planner_output();
  *scenario_context->mutable_path_planner_input() =
      PlanningContext::Instance()->path_planner_input();
  *scenario_context->mutable_planner_debug() =
      PlanningContext::Instance()->planner_debug();
  *scenario_context->mutable_speed_planner_input() =
      PlanningContext::Instance()->speed_planner_input();
  *scenario_context->mutable_speed_planner_output() =
      PlanningContext::Instance()->speed_planner_output();
  *scenario_context->mutable_lon_decison_output() =
      PlanningContext::Instance()->lon_decison_output();
  update_state_machine_output(*virtual_lane_mgr_, scenario_context);

  (void)fix_lane_scenario_facade->process(world_model_);
  // need to make sure a valid output each loop
  // copy result from scenario context to planning context
  *PlanningContext::Instance()->mutable_prev_planning_status() =
      PlanningContext::Instance()->planning_status();
  PlanningContext::Instance()->mutable_lateral_behavior_planner_output() =
      scenario_context->lateral_behavior_planner_output();
  PlanningContext::Instance()->mutable_lateral_motion_planner_output() =
      scenario_context->lateral_motion_planner_output();
  *PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output() =
      scenario_context->longitudinal_behavior_planner_output();
  *PlanningContext::Instance()->mutable_longitudinal_motion_planner_output() =
      scenario_context->longitudinal_motion_planner_output();
  *PlanningContext::Instance()->mutable_speed_limit() =
      scenario_context->speed_limit();
  *PlanningContext::Instance()->mutable_planning_status() =
      scenario_context->planning_status();
  PlanningContext::Instance()->mutable_obstacle_decision_manager() =
      scenario_context->obstacle_decision_manager();
  PlanningContext::Instance()->mutable_state_machine_output() =
      scenario_context->state_machine_output();
  *PlanningContext::Instance()->mutable_path_planner_input() =
      scenario_context->path_planner_input();
  *PlanningContext::Instance()->mutable_planner_debug() =
      scenario_context->planner_debug();
  *PlanningContext::Instance()->mutable_speed_planner_input() =
      scenario_context->speed_planner_input();
  *PlanningContext::Instance()->mutable_speed_planner_output() =
      scenario_context->speed_planner_output();
  *PlanningContext::Instance()->mutable_lon_decison_output() =
      scenario_context->lon_decison_output();
  update_decision_result();
}

void MSDStateMachine::update_decision_result() {
  PlanningStatus *planning_status =
      PlanningContext::Instance()->mutable_planning_status();
  PlanningResult &planning_result = planning_status->planning_result;
  const auto &obstacle_decision_manager =
      PlanningContext::Instance()->obstacle_decision_manager();
  auto &pre_yield_history = planning_status->pre_planning_result.yield_history;
  // set longitudinal & lateral decision info
  planning_result.lon_follow_obstacles.clear();
  planning_result.lon_overtake_obstacles.clear();
  planning_result.lat_nudge_obstacles.clear();
  planning_result.yield_history.clear();
  std::unordered_map<int, int> lon_decision_obs;
  lon_decision_obs.clear();
  auto target_baseline = world_model_->get_baseline_info(
      planning_status->lane_status.target_lane_id);
  if (target_baseline && target_baseline->is_valid()) {
    for (auto obstacle :
         target_baseline->mutable_obstacle_manager().get_obstacles().Items()) {
      auto ptr_obstacle_decision =
          obstacle_decision_manager.find_obstacle_decision(obstacle->Id());
      if (ptr_obstacle_decision->HasLongitudinalDecision()) {
        auto lon_decision = ptr_obstacle_decision->LongitudinalDecision();
        if (lon_decision.has_follow() || lon_decision.has_stop() ||
            lon_decision.has_yield()) {
          planning_result.lon_follow_obstacles.push_back(obstacle->Id());
          int origin_id = inverse_hash_prediction_id(obstacle->Id());
          if (lon_decision_obs.find(origin_id) == lon_decision_obs.end()) {
            lon_decision_obs.insert(std::make_pair(origin_id, 1));
            if (pre_yield_history.empty() ||
                pre_yield_history.find(origin_id) == pre_yield_history.end()) {
              planning_result.yield_history.insert(
                  std::make_pair(origin_id, 1));
            } else {
              planning_result.yield_history.insert(std::make_pair(
                  origin_id, std::min(100, pre_yield_history[origin_id] + 1)));
            }
          }
        } else if (lon_decision.has_overtake()) {
          planning_result.lon_overtake_obstacles.push_back(obstacle->Id());
        }
      } else if (ptr_obstacle_decision->HasLateralDecision()) {
        auto lat_decision = ptr_obstacle_decision->LateralDecision();
        if (lat_decision.has_nudge()) {
          planning_result.lat_nudge_obstacles.push_back(obstacle->Id());
        }
      }
    }
  }
  for (auto &obs : pre_yield_history) {
    if (lon_decision_obs.find(obs.first) == lon_decision_obs.end()) {
      obs.second--;
    }
    if (obs.second > 0) {
      planning_result.yield_history.insert(
          std::make_pair(obs.first, obs.second));
    }
  }

  // set trajectory matched scenario type
  auto lane_status = planning_status->lane_status;
  std::string first_stage, second_stage;
  if (lane_status.status == LaneStatus::LANE_KEEP) {
    first_stage = "lane_keep";
  } else if (lane_status.status == LaneStatus::LANE_CHANGE) {
    first_stage = "lane_change";
    auto lane_change_status = lane_status.change_lane;
    if (lane_change_status.status == ChangeLaneStatus::IN_CHANGE_LANE) {
      second_stage = "ongoing";
    } else if (lane_change_status.status ==
               ChangeLaneStatus::CHANGE_LANE_FAILED) {
      second_stage = "failed";
    } else if (lane_change_status.status ==
               ChangeLaneStatus::CHANGE_LANE_FINISHED) {
      second_stage = "finished";
    }
    second_stage = second_stage + lane_change_status.direction;
  }

  planning_result.matched_scenario_type = first_stage + second_stage;
  planning_result.turn_signal_cmd.value = turn_light_;
}

void MSDStateMachine::update_state_machine_output(
    const VirtualLaneManager &virtual_lane_mgr,
    const std::shared_ptr<ScenarioFacadeContext> &context) {
  // auto &state_machine_output =E
  //     PlanningContext::Instance()->mutable_state_machine_output();
  auto &state_machine_output = context->mutable_state_machine_output();
  state_machine_output.scenario = scenario_;
  state_machine_output.int_cancel_reason_fsm = int_cancel_reason_;
  state_machine_output.curr_state = fsm_context_.state;
  state_machine_output.state_name = fsm_context_.name;
  state_machine_output.must_change_lane = must_change_lane_;
  state_machine_output.turn_light =
      turn_light_; //  target state related & real-time
  state_machine_output.should_premove =
      should_premove_; // check_lc_valid updated & real-time
  state_machine_output.should_suspend =
      should_suspend_; // not target state related & sequential
  state_machine_output.lc_pause =
      lc_pause_; // check_lc_valid updated & real-time
  state_machine_output.lc_pause_id =
      lc_pause_id_; // check_lc_valid updated & real-time
  state_machine_output.tr_pause_l =
      tr_pause_l_; // check_lc_back updated & real-time
  state_machine_output.tr_pause_s =
      tr_pause_s_; // check_lc_back updated & real-time
  state_machine_output.lc_back_reason = lc_back_reason_;
  state_machine_output.lc_invalid_reason = lc_invalid_reason_;
  state_machine_output.state_change_reason = state_change_reason_;

  state_machine_output.lc_request = lc_req_mgr_->request();
  state_machine_output.lc_request_source = lc_req_mgr_->request_source();
  state_machine_output.act_request_source = lc_req_mgr_->act_request_source();
  state_machine_output.lc_turn_light = lc_req_mgr_->turn_light();

  virtual_lane_mgr.save_context(state_machine_output.virtual_lane_mgr_context);
}

void MSDStateMachine::generate_state_machine_output(
    int target_state, std::string target_state_name,
    const VirtualLaneManager &virtual_lane_mgr,
    const LaneChangeStageInfo &lc_info,
    const std::shared_ptr<ScenarioFacadeContext> &context) {
  auto &state_machine_output = context->mutable_state_machine_output();

  context->mutable_planning_status()->lane_status.change_lane.target_gap_obs =
      lc_info.gap;
  context->mutable_planning_status()
      ->lane_status.change_lane.target_lane_traffic_flow =
      lc_info.target_lane_traffic_flow;
  context->mutable_planning_status()
      ->lane_status.change_lane.enable_gap_protection =
      lc_info.enable_gap_protection;
  context->mutable_planning_status()
      ->lane_status.change_lane.gap_protection_counter =
      lc_info.gap_protection_counter;
  context->mutable_planning_status()
      ->lane_status.change_lane.enable_interactive_mode =
      lc_info.enable_interactive_mode;

  state_machine_output.scenario = scenario_;
  state_machine_output.curr_state = target_state;
  state_machine_output.state_name = std::move(target_state_name);
  state_machine_output.lc_back_reason = lc_info.lc_back_reason;
  state_machine_output.lc_invalid_reason = lc_info.lc_invalid_reason;
  state_machine_output.must_change_lane = must_change_lane_;

  state_machine_output.turn_light = lc_req_mgr_->turn_light();

  state_machine_output.should_premove = lc_info.should_premove;
  state_machine_output.should_suspend = should_suspend_;
  state_machine_output.lc_pause = lc_info.lc_pause;
  state_machine_output.lc_pause_id = lc_info.lc_pause_id;
  state_machine_output.tr_pause_l = lc_info.tr_pause_l;
  state_machine_output.tr_pause_s = lc_info.tr_pause_s;
  state_machine_output.lc_request = lc_req_mgr_->request();
  state_machine_output.lc_request_source = lc_req_mgr_->request_source();
  state_machine_output.act_request_source = lc_req_mgr_->act_request_source();
  state_machine_output.lc_turn_light = lc_req_mgr_->turn_light();

  virtual_lane_mgr.save_context(state_machine_output.virtual_lane_mgr_context);
}
void MSDStateMachine::update_hfsm_debug() {
  auto planner_debug = PlanningContext::Instance()->mutable_planner_debug();
  // update o/t/flane
  planner_debug->lat_dec_info.tlane_id = 100;
  planner_debug->lat_dec_info.olane_id = 100;
  planner_debug->lat_dec_info.flane_id = 100;
  if (virtual_lane_mgr_->has_target_lane()) {
    auto tlane = virtual_lane_mgr_->get_target_lane().master();
    if (tlane != nullptr) {
      planner_debug->lat_dec_info.tlane_id = tlane->position();
    }
  }
  if (virtual_lane_mgr_->has_origin_lane()) {
    auto olane = virtual_lane_mgr_->get_origin_lane().master();
    if (olane != nullptr) {
      planner_debug->lat_dec_info.olane_id = olane->position();
    }
  }
  auto flane = virtual_lane_mgr_->get_fix_lane().master();
  if (flane != nullptr) {
    planner_debug->lat_dec_info.flane_id = flane->position();
  }
  auto lc_request = lc_req_mgr_->request();
  bool is_solid_line = false;
  if (lc_request == LEFT_CHANGE) {
    is_solid_line =
        world_model_->get_mutable_map_info_manager().is_solid_line(0);
  } else {
    is_solid_line =
        world_model_->get_mutable_map_info_manager().is_solid_line(1);
  }
  planner_debug->lat_dec_info.is_target_in_solid = is_solid_line;
}
void MSDStateMachine::check_vision_lane_stable() {
  auto map_info = world_model_->get_map_info();
  map_info.lane_.size();
  if ((fsm_context_.state >= ROAD_LC_LCHANGE) &&
      (fsm_context_.state <= ROAD_LC_RBACK)) {
    // push stable while LC process and back
    if (llane_stable_list_.size() >= 10) {
      llane_stable_list_.erase(llane_stable_list_.begin());
      llane_stable_list_.emplace_back(int(0));
    } else {
      llane_stable_list_.emplace_back(int(0));
    }
    if (rlane_stable_list_.size() >= 10) {
      rlane_stable_list_.erase(rlane_stable_list_.begin());
      rlane_stable_list_.emplace_back(int(0));
    } else {
      rlane_stable_list_.emplace_back(int(0));
    }
  } else {
    bool left_stable = false;
    bool right_stable = false;
    for (auto &lane : map_info.lane_) {
      if (lane.relative_id == -1 && lane.reference_line.available) {
        left_stable = true;
      }
      if (lane.relative_id == 1 && lane.reference_line.available) {
        right_stable = true;
      }
    }
    if (llane_stable_list_.size() >= 10) {
      llane_stable_list_.erase(llane_stable_list_.begin());
      if (left_stable) {
        llane_stable_list_.emplace_back(int(0));
      } else {
        llane_stable_list_.emplace_back(int(1));
      }
    } else {
      if (left_stable) {
        llane_stable_list_.emplace_back(int(0));
      } else {
        llane_stable_list_.emplace_back(int(1));
      }
    }
    if (rlane_stable_list_.size() >= 10) {
      rlane_stable_list_.erase(rlane_stable_list_.begin());
      if (right_stable) {
        rlane_stable_list_.emplace_back(int(0));
      } else {
        rlane_stable_list_.emplace_back(int(1));
      }
    } else {
      if (right_stable) {
        rlane_stable_list_.emplace_back(int(0));
      } else {
        rlane_stable_list_.emplace_back(int(1));
      }
    }
  }
  left_lane_stable_cnt_ = 0;
  auto planner_debug = PlanningContext::Instance()->mutable_planner_debug();
  for (auto &i : llane_stable_list_) {
    left_lane_stable_cnt_ += i;
  }
  planner_debug->lat_dec_info.left_lane_stable_cnt = left_lane_stable_cnt_;
  right_lane_stable_cnt_ = 0;
  for (auto &i : rlane_stable_list_) {
    right_lane_stable_cnt_ += i;
  }
  planner_debug->lat_dec_info.right_lane_stable_cnt = right_lane_stable_cnt_;
}

bool MSDStateMachine::is_lc_lane_stable(int direction) const {
  if (direction == LEFT_CHANGE) {
    return left_lane_stable_cnt_ <= 0;
  } else {
    return right_lane_stable_cnt_ <= 0;
  }
}

} // namespace msquare
