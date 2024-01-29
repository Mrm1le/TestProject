#include "planner/behavior_planner/lateral_behavior_road_state.h"
#include "planner/arbitrators/arbitrator_cost_strategy.h"
#include "planner/arbitrators/merit_based_arbitrator.h"
#include "planner/arbitrators/priority_based_arbitrator.h"
#include "planner/arbitrators/sequence_arbitrator.h"
#include "planner/arbitrators/single_scenario_arbitrator.h"
#include "planner/behavior_planner/lateral_behavior_planner.h"
#include "planner/behavior_planner/lateral_behavior_state_machine.h"
#include "planner/scenarios/scenario_factory.h"

namespace msquare {

void RoadState::None::callback(Control &control, FsmContext &context) {

  FsmUserContext *user_context =
      static_cast<FsmUserContext *>(context.user_data);

  if (user_context == nullptr || user_context->state_machine == nullptr) {
    return;
  }

  MSDStateMachine *state_machine = user_context->state_machine;
  state_machine->set_entry_time(context.entry_time);
  state_machine->set_gmp_should_cancel(false);
  auto &state_machine_output_mutable =
      PlanningContext::Instance()->mutable_state_machine_output();
  state_machine_output_mutable.lane_change_condition =
      LaneChangeCondition ::DEFAULT_LCC;
  auto &int_cancel_reason = state_machine->int_cancel_reason();
  int_cancel_reason = NO_CANCEL;
  // auto &virtual_lane_mgr = *state_machine->virtual_lane_mgr();
  static VirtualLaneManager virtual_lane_mgr(
      *state_machine->virtual_lane_mgr());
  virtual_lane_mgr.update(*state_machine->virtual_lane_mgr());
  auto &lc_req_mgr = *state_machine->lc_request_manager();
  int lc_request = lc_req_mgr.request();
  int lc_source = lc_req_mgr.request_source();
  double lc_tstart = lc_req_mgr.get_req_tstart(lc_source);
  double delay_time = user_context->delay_time[lc_source];
  double initial_protect_time = 0.2;
  double curr_time = get_system_time();
  // if target lane invalid over 2.0, clear lc request
  double KLaneProtectTime =
      ConfigurationContext::Instance()
          ->planner_config()
          .lateral_behavior_planner_config.time_lc_wait_limit;
  constexpr double KKeepToWaitDelayTime = 0.8;
  constexpr double KReadyMaxTime = 0.5;
  if (lc_request == NO_CHANGE)
    int_cancel_reason = lc_req_mgr.get_int_request_cancel_reason();
  state_machine_output_mutable.int_cancel_reason_fsm = int_cancel_reason;
  auto current_baseline = context.world_model->get_baseline_info(
      virtual_lane_mgr.get_fix_refline().position());
  const auto &map_info = context.world_model->get_map_info();
  bool must_change_lane = true;
  bool curr_direct_exist =
      ((map_info.current_lane_marks() & map_info.traffic_light_direction()) ||
       map_info.traffic_light_direction() == Direction::UNKNOWN ||
       (map_info.current_lane_marks() == Direction::UNKNOWN &&
        map_info.current_lane_type() == LaneType::NORMAL));
  // if (current_baseline && !current_baseline->is_target_lane()) {
  //   must_change_lane = true;
  // }
  if ((lc_source == MAP_REQUEST &&
       (!curr_direct_exist || map_info.is_on_highway() ||
        map_info.lanes_merge_type() == MergeType::LANE_END ||
        (map_info.merge_split_point_data().size() > 0 &&
         !map_info.merge_split_point_data()[0].is_continue &&
         map_info.merge_split_point_data()[0].distance <
             map_info.dist_to_intsect()))) ||
      lc_source == INT_REQUEST) {
    must_change_lane = true;
  }
  // request must be executed
  // if (context.world_model->get_map_info().is_on_highway()) {
  //   must_change_lane = true;
  // }
  state_machine->set_must_change_lane(must_change_lane);

  // construct lane keep scenario
  std::shared_ptr<Arbitrator> lane_keep_scenario_arbitrator;
  static VirtualLaneManager virtual_lane_mgr_lane_keep(virtual_lane_mgr);
  virtual_lane_mgr_lane_keep.update(virtual_lane_mgr);
  auto lane_keep_context = context;
  lane_keep_context.name = type2name<RoadState::None>::name;
  lane_keep_context.state = type2int<RoadState::None>::value;
  std::shared_ptr<SingleScenarioArbitrator>
      normal_lane_keep_scenario_arbitrator =
          std::make_shared<SingleScenarioArbitrator>(context,
                                                     "none_lane_keep_scenario");
  normal_lane_keep_scenario_arbitrator->init_from_type(
      ScenarioFacadeConfig::ScenarioFacadeType::FixLaneCruise);
  LaneChangeStageInfo lc_info;
  state_machine->generate_state_machine_output(
      lane_keep_context.state, lane_keep_context.name,
      virtual_lane_mgr_lane_keep, lc_info,
      normal_lane_keep_scenario_arbitrator->get_mutable_scenario_facade()
          .get_scenario_context());
  // normal_lane_keep_scenario_arbitrator->set_cost_function(ArbitratorCostFunc::)

  bool merge_or_split_scenario = false;
  const auto &MergeInfo = map_info.next_merge_info();
  if (MergeInfo.orientation.value != Orientation::UNKNOWN &&
      !MergeInfo.is_continue) {
    merge_or_split_scenario = true;
  }
  merge_or_split_scenario = false;
  if (merge_or_split_scenario) {
    std::shared_ptr<SingleScenarioArbitrator>
        conservative_lane_keep_scenario_arbitrator =
            std::make_shared<SingleScenarioArbitrator>(
                context, "conservative_none_lane_keep_scenario");
    ScenarioFacadeConfig scenario_config =
        ConfigurationContext::Instance()->get_params().at(
            ScenarioFacadeConfig::ScenarioFacadeType::FixLaneCruise);
    scenario_config
        .tasks_config_map_[TaskConfig::TaskType::LONGITUDINAL_BEHAVIOR_PLANNER]
        .set_longitudinal_behavior_planner_config(
            ConfigurationContext::Instance()
                ->get_conservative_tasks_params()
                .at(TaskConfig::TaskType::LONGITUDINAL_BEHAVIOR_PLANNER)
                .longitudinal_behavior_planner_config());
    conservative_lane_keep_scenario_arbitrator->init(scenario_config);
    std::shared_ptr<SingleScenarioArbitrator>
        aggressive_lane_keep_scenario_arbitrator =
            std::make_shared<SingleScenarioArbitrator>(
                context, "aggressvie_none_lane_keep_scenario");
    scenario_config
        .tasks_config_map_[TaskConfig::TaskType::LONGITUDINAL_BEHAVIOR_PLANNER]
        .set_longitudinal_behavior_planner_config(
            ConfigurationContext::Instance()
                ->get_aggressive_tasks_params()
                .at(TaskConfig::TaskType::LONGITUDINAL_BEHAVIOR_PLANNER)
                .longitudinal_behavior_planner_config());
    aggressive_lane_keep_scenario_arbitrator->init(scenario_config);
    std::vector<std::shared_ptr<Arbitrator>> sub_arbitrators;
    sub_arbitrators.push_back(aggressive_lane_keep_scenario_arbitrator);
    sub_arbitrators.push_back(conservative_lane_keep_scenario_arbitrator);
    auto merit_arbitrator = std::make_shared<MeritArbitrator>(
        context, "none_lane_keep_aggressive_or_conservative_arbitrator");
    merit_arbitrator->set_sub_arbitrators(sub_arbitrators);
    ArbitratorCostStrategy::set_arbitrator_cost_func(
        merit_arbitrator, ArbitratorCostType::UNPROTECTED_MANEUVER);
    lane_keep_scenario_arbitrator = merit_arbitrator;
  } else {
    lane_keep_scenario_arbitrator = normal_lane_keep_scenario_arbitrator;
  }

  std::shared_ptr<SingleScenarioArbitrator> lc_wait_scenario_arbitrator,
      lc_change_scenario_arbitrator;

  if ((lc_request == LEFT_CHANGE || lc_request == RIGHT_CHANGE)) {
    static VirtualLaneManager virtual_lane_mgr_lc(virtual_lane_mgr);
    virtual_lane_mgr_lc.update(virtual_lane_mgr);
    virtual_lane_mgr_lc.clear_lc_lanes();
    if (virtual_lane_mgr_lc.assign_lc_lanes(lc_request)) {
      static VirtualLaneManager virtual_lane_mgr_lc_change(virtual_lane_mgr_lc);
      virtual_lane_mgr_lc_change.update(virtual_lane_mgr_lc);
      static VirtualLaneManager virtual_lane_mgr_lc_wait(virtual_lane_mgr_lc);
      virtual_lane_mgr_lc_wait.update(virtual_lane_mgr_lc);
      virtual_lane_mgr_lc_change.set_fix_lane(TARGET_LANE);
      virtual_lane_mgr_lc_change.update_fix_lane_info();
      if (virtual_lane_mgr_lc_wait.has_origin_lane()) {
        virtual_lane_mgr_lc_wait.set_fix_lane(ORIGIN_LANE);
      } else {
        virtual_lane_mgr_lc_wait.set_fix_lane(CURRENT_LANE);
      }
      virtual_lane_mgr_lc_wait.update_fix_lane_info();
      int origin_lane_id =
          virtual_lane_mgr_lc_wait.mutable_fix_refline().position();
      int target_lane_id =
          virtual_lane_mgr_lc_change.mutable_fix_refline().position();

      auto lane_change_info = state_machine->decide_lc_valid_info(
          virtual_lane_mgr_lc, lc_request, TARGET_LANE);
      // construct lc change scenario
      auto lc_change_context = context;
      auto lc_wait_context = context;
      if (lc_request == LEFT_CHANGE) {
        lc_change_context.name = type2name<RoadState::LC::LChange>::name;
        lc_change_context.state = type2int<RoadState::LC::LChange>::value;
        lc_wait_context.name = type2name<RoadState::LC::LWait>::name;
        lc_wait_context.state = type2int<RoadState::LC::LWait>::value;
      } else {
        lc_change_context.name = type2name<RoadState::LC::RChange>::name;
        lc_change_context.state = type2int<RoadState::LC::RChange>::value;
        lc_wait_context.name = type2name<RoadState::LC::RWait>::name;
        lc_wait_context.state = type2int<RoadState::LC::RWait>::value;
      }
      lc_change_scenario_arbitrator =
          std::make_shared<SingleScenarioArbitrator>(context,
                                                     "none_lc_change_scenario");
      lc_change_scenario_arbitrator->init_from_type(
          ScenarioFacadeConfig::ScenarioFacadeType::ChangeLaneCruise);
      // to do: set right lc change related info in scenario context
      state_machine->generate_state_machine_output(
          lc_change_context.state, lc_change_context.name,
          virtual_lane_mgr_lc_change, lane_change_info,
          lc_change_scenario_arbitrator->get_mutable_scenario_facade()
              .get_scenario_context());
      auto &f_refline = virtual_lane_mgr_lc_change.get_fix_refline();
      if (f_refline.position() == LEFT_POS ||
          f_refline.position() == LEFT_LEFT_POS) {
        MSD_LOG(INFO, "arbitrator left_line");
      } else if (f_refline.position() == RIGHT_POS) {
        MSD_LOG(INFO, "arbitrator right_line");
      } else {
        MSD_LOG(INFO, "arbitrator current_line %d", f_refline.position());
      }
      // construct lc wait scenario
      lc_wait_scenario_arbitrator = std::make_shared<SingleScenarioArbitrator>(
          context, "none_lc_wait_scenario");
      lc_wait_scenario_arbitrator->init_from_type(
          ScenarioFacadeConfig::ScenarioFacadeType::ChangelanePreparation);
      // to do: set right lc wait related info in scenario context
      state_machine->generate_state_machine_output(
          lc_wait_context.state, lc_wait_context.name, virtual_lane_mgr_lc_wait,
          lane_change_info,
          lc_wait_scenario_arbitrator->get_mutable_scenario_facade()
              .get_scenario_context());
      bool valid_body = state_machine->check_lc_body_valid(lc_request);
      if (must_change_lane) {
        // satisfy condition
        // 0 left 1 right
        bool is_solid_line = false;
        if (lc_request == LEFT_CHANGE)
          is_solid_line = state_machine->world_model()
                              ->get_mutable_map_info_manager()
                              .is_solid_line(0);
        else
          is_solid_line = state_machine->world_model()
                              ->get_mutable_map_info_manager()
                              .is_solid_line(1);
        bool is_lane_stable = false;
        is_lane_stable = state_machine->is_lc_lane_stable(lc_request);
        if ((lane_change_info.gap_insertable ||
             lane_change_info.should_ready) &&
            !is_solid_line && valid_body && is_lane_stable) {
          state_machine_output_mutable.lane_change_condition =
              LaneChangeCondition ::SUITABLE_LCC;
          state_machine_output_mutable.lane_change_condition =
              LaneChangeCondition ::SUITABLE_LCC;
          bool prohibit_process_lc =
              lane_change_info.should_ready && !lane_change_info.gap_insertable;
          // ready time
          if (!(curr_time > lc_tstart + KReadyMaxTime) && prohibit_process_lc) {
            state_machine_output_mutable.int_cancel_reason_fsm = NO_CANCEL;
            state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_wait);
            state_machine->set_lc_invalid_reason("lc_ready");
            state_machine->set_lc_state("lc ready");
            state_machine->set_state_change_reason(
                "lc satisfy condition but time not suitable");
            // lane_keep_scenario_arbitrator->get_mutable_scenario_facade().output_planning_context();
            state_machine->post_process();
            return;
          }
          if (!(curr_time > lc_tstart + delay_time)) {
            state_machine_output_mutable.int_cancel_reason_fsm = NO_CANCEL;
            state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_wait);
            state_machine->set_lc_invalid_reason("lc_ready");
            state_machine->set_lc_state("lc ready");
            state_machine->set_state_change_reason(
                "lc satisfy condition but time not suitable");
            // lane_keep_scenario_arbitrator->get_mutable_scenario_facade().output_planning_context();
            state_machine->post_process();
            return;
          }
          // ready time over : go to lanechange
          if (lc_request == LEFT_CHANGE) {
            change_state<RoadState::LC::LChange>(control, context);
            state_machine->set_lc_state("left_change:must_change_lane");
            state_machine->set_state_change_reason("lc left request");
          } else {
            change_state<RoadState::LC::RChange>(control, context);
            state_machine->set_lc_state("right_change:must_change_lane");
            state_machine->set_state_change_reason("lc right request");
          }
          state_machine->set_get_dist_lane(true);
          state_machine->clear_lc_valid_cnt();
          state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_change);
          lc_change_scenario_arbitrator->arbitrate();
          lc_change_scenario_arbitrator->get_mutable_scenario_facade()
              .output_planning_context();
          state_machine->post_process();
          return;
        } else if (lane_change_info.gap_valid || is_solid_line || !valid_body ||
                   !is_lane_stable) {
          // have gap but not satisfy condition
          // go to lc_wait
          if (is_solid_line) {
            state_machine_output_mutable.lane_change_condition =
                LaneChangeCondition ::SOLID_LINE_LCC;
          } else if (!valid_body) {
            state_machine_output_mutable.lane_change_condition =
                LaneChangeCondition ::VEHICLE_UNSUITABLE_LCC;
          } else if (!is_lane_stable) {
            state_machine_output_mutable.lane_change_condition =
                LaneChangeCondition ::ENV_UNSUITABLE_LCC;
          } else {
            state_machine_output_mutable.lane_change_condition =
                LaneChangeCondition ::UNINSERTABLE_LCC;
          }
          if (lc_request == LEFT_CHANGE) {
            change_state<RoadState::LC::LWait>(control, context);
            state_machine->set_lc_state("left_change wait:must_change_lane");
            state_machine->set_state_change_reason("lc left request and have "
                                                   "valid gap or solid line or "
                                                   "steer limit");
          } else {
            change_state<RoadState::LC::RWait>(control, context);
            state_machine->set_lc_state("right_change wait:must_change_lane");
            state_machine->set_state_change_reason("lc right request and have "
                                                   "valid gap or solid line or "
                                                   "steer limit");
          }
          state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_wait);
          lc_wait_scenario_arbitrator->arbitrate();
          lc_wait_scenario_arbitrator->get_mutable_scenario_facade()
              .output_planning_context();
          state_machine->post_process();
          return;
        } else {
          // no valid gap and must change lane, go to backup decelarating mode
          // or request for rerouting
          // L for demo
          if (!(curr_time > lc_tstart + initial_protect_time)) {
            state_machine_output_mutable.int_cancel_reason_fsm = NO_CANCEL;
            state_machine_output_mutable.lane_change_condition =
                LaneChangeCondition::SUITABLE_LCC;
            state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_wait);
            state_machine->set_lc_invalid_reason("lc_ready_protect");
            state_machine->set_lc_state("lc ready");
            state_machine->set_state_change_reason(
                "lc do not satisfy condition but during initial protect time");
            // lane_keep_scenario_arbitrator->get_mutable_scenario_facade().output_planning_context();
            state_machine->post_process();
            return;
          }
          MSD_LOG(INFO, "arbitrator no valid gap for a lane change that must "
                        "be executed");
          state_machine_output_mutable.lane_change_condition =
              LaneChangeCondition ::ENV_UNSUITABLE_LCC;
          if (!context.world_model->get_baseline_info(origin_lane_id) ||
              !context.world_model->get_baseline_info(origin_lane_id)
                   ->is_valid()) {
            state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lane_keep);
            state_machine->set_lc_invalid_reason("lc_change_or_wait_not_valid");
            state_machine->set_lc_state("lane follow");
            state_machine->set_state_change_reason(
                "lc left request but have no baseline info");
            // lane_keep_scenario_arbitrator->get_mutable_scenario_facade().output_planning_context();
            state_machine->post_process();
            return;
          }
          // now we go to lc wait temporarily
          if (lc_request == LEFT_CHANGE) {
            change_state<RoadState::LC::LWait>(control, context);
            state_machine->set_lc_state("left_change wait:must_change_lane");
            state_machine->set_state_change_reason(
                "lc left request but have no valid gap");
          } else {
            change_state<RoadState::LC::RWait>(control, context);
            state_machine->set_lc_state("right_change wait:must_change_lane");
            state_machine->set_state_change_reason(
                "lc left request but have no valid gap");
          }
          state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_wait);
          lc_wait_scenario_arbitrator->arbitrate();
          lc_wait_scenario_arbitrator->get_mutable_scenario_facade()
              .output_planning_context();
          state_machine->post_process();
          return;
        }
      } else {
        // go to lk or lc_wait/lc_change
        // disable active none -> change
        if (lane_change_info.gap_insertable || lane_change_info.gap_valid) {
          // go to lk or lc_wait(approach or indicate intention)
          std::vector<std::shared_ptr<Arbitrator>> sub_arbitrators;
          sub_arbitrators.push_back(lane_keep_scenario_arbitrator);
          sub_arbitrators.push_back(lc_wait_scenario_arbitrator);
          std::shared_ptr<MeritArbitrator> merit_arbitrator =
              std::make_shared<MeritArbitrator>(
                  context, "none_lc_wait_or_keep_arbitrator");
          merit_arbitrator->set_sub_arbitrators(sub_arbitrators);
          if (lc_req_mgr.act_request_source() == "avoid_int_cutin_obj") {
            ArbitratorCostStrategy::set_arbitrator_cost_func(
                merit_arbitrator,
                ArbitratorCostType::NONESSENTIAL_LANECHANGE_MANEUVER);
          } else {
            ArbitratorCostStrategy::set_arbitrator_cost_func(
                merit_arbitrator, ArbitratorCostType::NORMAL);
          }
          merit_arbitrator->arbitrate();
          auto best_solution = merit_arbitrator->get_best_solution();
          mph_assert(best_solution != nullptr);
          if (curr_time - context.entry_time > KKeepToWaitDelayTime &&
              (best_solution && best_solution->get_optimal_scenario_context()
                                        ->state_machine_output()
                                        .curr_state == lc_wait_context.state)) {
            if (lc_request == LEFT_CHANGE) {
              change_state<RoadState::LC::LWait>(control, context);
              state_machine->set_lc_state("left_change wait");
              state_machine->set_state_change_reason(
                  "lc arbitrate lc wait better");
            } else {
              change_state<RoadState::LC::RWait>(control, context);
              state_machine->set_lc_state("right_change wait");
              state_machine->set_state_change_reason(
                  "lc arbitrate lc wait better");
            }
            state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_wait);
            lc_wait_scenario_arbitrator->get_mutable_scenario_facade()
                .output_planning_context();
            state_machine->post_process();
            return;
          } else {
            state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lane_keep);
            state_machine->set_lc_invalid_reason("lc_change_not_optimal");
            ScenarioFacade::output_planning_context(
                *best_solution->get_optimal_scenario_context());
            // lane_keep_scenario_arbitrator->get_mutable_scenario_facade().output_planning_context();
            state_machine->post_process();
            return;
          }
        } else {
          // go to lk
          state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lane_keep);
          lane_keep_scenario_arbitrator->arbitrate();
          // state_machine->set_lc_invalid_reason("lc_change_not_valid");
          ScenarioFacade::output_planning_context(
              *lane_keep_scenario_arbitrator->get_best_solution()
                   ->get_optimal_scenario_context());
          // lane_keep_scenario_arbitrator->get_mutable_scenario_facade().output_planning_context();
          state_machine->post_process();
          return;
        }
      }
    } else {
      // go to lk
      state_machine->set_lc_invalid_reason("lc_change_assign_lc_lanes_failed");
      state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lane_keep);
      state_machine_output_mutable.lane_change_condition =
          LaneChangeCondition ::ENV_UNSUITABLE_LCC;
      if (curr_time > lc_tstart + KLaneProtectTime) {
        lc_req_mgr.finish_request();
        state_machine_output_mutable.lane_change_condition =
            LaneChangeCondition ::DEFAULT_LCC;
        MSD_LOG(INFO, "not_suitable change to defalut");
        lane_keep_scenario_arbitrator->arbitrate();
        ScenarioFacade::output_planning_context(
            *lane_keep_scenario_arbitrator->get_best_solution()
                 ->get_optimal_scenario_context());
        state_machine_output_mutable.int_cancel_reason_fsm = TIMEOUT_LC;
        // lane_keep_scenario_arbitrator->get_mutable_scenario_facade().output_planning_context();
        state_machine->post_process();
        return;
      }
      lane_keep_scenario_arbitrator->arbitrate();
      ScenarioFacade::output_planning_context(
          *lane_keep_scenario_arbitrator->get_best_solution()
               ->get_optimal_scenario_context());
      state_machine_output_mutable.int_cancel_reason_fsm = ENV_ERROR;
      // lane_keep_scenario_arbitrator->get_mutable_scenario_facade().output_planning_context();
      state_machine->post_process();
      return;
    }
  } else {
    virtual_lane_mgr.clear_lc_lanes();
    virtual_lane_mgr.update_fix_lane();

    // no lc request, check traj safety
    virtual_lane_mgr.update_fix_lane_info();
    state_machine->update_virtual_lane_mgr(virtual_lane_mgr);
    LaneChangeStageInfo lc_info;
    state_machine->generate_state_machine_output(
        lane_keep_context.state, lane_keep_context.name, virtual_lane_mgr,
        lc_info,
        normal_lane_keep_scenario_arbitrator->get_mutable_scenario_facade()
            .get_scenario_context());
    ArbitratorCostStrategy::set_arbitrator_cost_func(
        normal_lane_keep_scenario_arbitrator, ArbitratorCostType::NORMAL);
    normal_lane_keep_scenario_arbitrator->arbitrate();
    ScenarioFacade::output_planning_context(
        *normal_lane_keep_scenario_arbitrator->get_best_solution()
             ->get_optimal_scenario_context());
    state_machine->post_process();
    return;
  }

  // choose optimal scenario and output
  virtual_lane_mgr.update_fix_lane_info();
  state_machine->update_virtual_lane_mgr(virtual_lane_mgr);
  state_machine->post_process();
}

void RoadState::LC::process_wait(Control &control, FsmContext &context) {
  int state = context.state;

  FsmUserContext *user_context =
      static_cast<FsmUserContext *>(context.user_data);

  if (user_context == nullptr || user_context->state_machine == nullptr) {
    return;
  }

  MSDStateMachine *state_machine = user_context->state_machine;
  state_machine->set_gmp_should_cancel(false);
  state_machine->set_entry_time(context.entry_time);
  auto &state_machine_output_mutable =
      PlanningContext::Instance()->mutable_state_machine_output();
  auto &int_cancel_reason = state_machine->int_cancel_reason();
  int_cancel_reason = NO_CANCEL;
  state_machine_output_mutable.int_cancel_reason_fsm = int_cancel_reason;
  // auto &virtual_lane_mgr = *state_machine->virtual_lane_mgr();
  static VirtualLaneManager virtual_lane_mgr(
      *state_machine->virtual_lane_mgr());
  virtual_lane_mgr.update(*state_machine->virtual_lane_mgr());
  auto &lc_req_mgr = *state_machine->lc_request_manager();

  int lc_request = lc_req_mgr.request();
  int lc_source = lc_req_mgr.request_source();
  double lc_tstart = lc_req_mgr.get_req_tstart(lc_source);
  double delay_time = user_context->delay_time[lc_source];
  if (lc_req_mgr.act_request_source() == "avoid_int_cutin_obj") {
    delay_time = 0.0;
  }
  constexpr double KWaitToKeepDelayTime = 5.0;
  double KWaitToKeepDelayTimeHard =
      ConfigurationContext::Instance()
          ->planner_config()
          .lateral_behavior_planner_config.time_lc_wait_limit;
  double wait_to_keep_protect_time = KWaitToKeepDelayTime;
  double wait_to_cancel_protect_time = KWaitToKeepDelayTimeHard;
  if (lc_req_mgr.act_request_source() == "avoid_int_cutin_obj") {
    wait_to_keep_protect_time = 1.0;
  }
  double curr_time = get_system_time();
  MSD_LOG(INFO, "arbitrator delay time %f", delay_time);

  auto current_baseline = context.world_model->get_baseline_info(
      virtual_lane_mgr.get_fix_refline().position());
  auto &map_info = context.world_model->get_map_info();
  const auto &refline_condition = context.world_model->get_refline_condition();
  bool must_change_lane = true;
  bool curr_direct_exist =
      ((map_info.current_lane_marks() & map_info.traffic_light_direction()) ||
       map_info.traffic_light_direction() == Direction::UNKNOWN ||
       (map_info.current_lane_marks() == Direction::UNKNOWN &&
        map_info.current_lane_type() == LaneType::NORMAL));
  // if (current_baseline && !current_baseline->is_target_lane()) {
  //   must_change_lane = true;
  // }
  if ((lc_source == MAP_REQUEST &&
       (!curr_direct_exist || map_info.is_on_highway() ||
        map_info.lanes_merge_type() == MergeType::LANE_END ||
        (map_info.merge_split_point_data().size() > 0 &&
         !map_info.merge_split_point_data()[0].is_continue &&
         map_info.merge_split_point_data()[0].distance <
             map_info.dist_to_intsect()))) ||
      lc_source == INT_REQUEST) {
    must_change_lane = true;
  }
  state_machine->set_must_change_lane(must_change_lane);
  MSD_LOG(INFO, "zzzz wait: must change lane %d", must_change_lane);
  // construct lane keep scenario
  // call state_machine->clear_lc_variables() when choose lane keep or pass back
  // the lc variable in chosen scenario
  static VirtualLaneManager virtual_lane_mgr_lane_keep(virtual_lane_mgr);
  virtual_lane_mgr_lane_keep.update(virtual_lane_mgr);
  virtual_lane_mgr_lane_keep.clear_lc_lanes();
  virtual_lane_mgr_lane_keep.set_fix_lane(CURRENT_LANE);
  virtual_lane_mgr_lane_keep.update_fix_lane_info();
  auto lane_keep_context = context;
  lane_keep_context.name = type2name<RoadState::None>::name;
  lane_keep_context.state = type2int<RoadState::None>::value;
  std::shared_ptr<SingleScenarioArbitrator> lane_keep_scenario_arbitrator =
      std::make_shared<SingleScenarioArbitrator>(context,
                                                 "wait_lane_keep_scenario");
  lane_keep_scenario_arbitrator->init_from_type(
      ScenarioFacadeConfig::ScenarioFacadeType::FixLaneCruise);
  LaneChangeStageInfo lc_info;
  state_machine->generate_state_machine_output(
      lane_keep_context.state, lane_keep_context.name,
      virtual_lane_mgr_lane_keep, lc_info,
      lane_keep_scenario_arbitrator->get_mutable_scenario_facade()
          .get_scenario_context());
  std::shared_ptr<SingleScenarioArbitrator> lc_wait_scenario_arbitrator,
      lc_change_scenario_arbitrator;

  std::string lc_state = "";
  if ((state == ROAD_LC_LWAIT && lc_request == RIGHT_CHANGE) ||
      (state == ROAD_LC_RWAIT && lc_request == LEFT_CHANGE)) {
    lc_state = "reversed_lc";
  } else if ((state == ROAD_LC_LWAIT && lc_request == LEFT_CHANGE) ||
             (state == ROAD_LC_RWAIT && lc_request == RIGHT_CHANGE)) {
    lc_state = "forward_lc";
  }
  if (lc_request == NO_CHANGE ||
      (state_machine->world_model()->is_ddmap() &&
       (refline_condition.reflinecondition !=
            pass_intersection_planner::ReflineCondition::NO_INTERSECTION ||
        curr_time - state_machine->entry_time() >
            wait_to_cancel_protect_time))) {
    change_state<RoadState::None>(control, context);
    if (lc_request == NO_CHANGE)
      int_cancel_reason = lc_req_mgr.get_int_request_cancel_reason();
    else if (curr_time - state_machine->entry_time() >
             wait_to_cancel_protect_time)
      int_cancel_reason = TIMEOUT_LC;
    else
      int_cancel_reason = ENV_ERROR;
    if (int_cancel_reason == TIMEOUT_LC &&
        !lc_req_mgr.is_int_exist_dash_line()) {
      int_cancel_reason = SOLID_LC;
    }
    state_machine_output_mutable.int_cancel_reason_fsm = int_cancel_reason;
    lc_req_mgr.finish_request();
    state_machine->set_lc_state("lane follow");
    state_machine->set_state_change_reason("do not need lc");
    virtual_lane_mgr.clear_lc_lanes();
    state_machine->clear_lc_variables();
    virtual_lane_mgr.set_fix_lane(CURRENT_LANE);
    state_machine->update_virtual_lane_mgr(virtual_lane_mgr);
    state_machine->post_process();
    return;
  } else if (lc_state == "reversed_lc" || lc_state == "forward_lc") {
    static VirtualLaneManager virtual_lane_mgr_lc(virtual_lane_mgr);
    virtual_lane_mgr_lc.update(virtual_lane_mgr);
    if (lc_state == "reversed_lc") {
      virtual_lane_mgr_lc.clear_lc_lanes(); // ---1
      state_machine->clear_lc_variables();  // --- ???
    } else {
      virtual_lane_mgr_lc.update_lc_lanes(lc_request, state); // ---1
    }
    // the logic is same as None
    if ((lc_state == "reversed_lc" &&
         virtual_lane_mgr_lc.assign_lc_lanes(lc_request)) ||
        (lc_state == "forward_lc" && virtual_lane_mgr_lc.has_target_lane() &&
         virtual_lane_mgr_lc.has_origin_lane())) { // ---2
      static VirtualLaneManager virtual_lane_mgr_lc_change(virtual_lane_mgr_lc);
      virtual_lane_mgr_lc_change.update(virtual_lane_mgr_lc);
      static VirtualLaneManager virtual_lane_mgr_lc_wait(virtual_lane_mgr_lc);
      virtual_lane_mgr_lc_wait.update(virtual_lane_mgr_lc);
      virtual_lane_mgr_lc_change.set_fix_lane(TARGET_LANE);
      virtual_lane_mgr_lc_change.update_fix_lane_info();

      // virtual_lane_mgr_lc_wait.set_fix_lane(ORIGIN_LANE); // ---3
      if (lc_state == "reversed_lc" ||
          virtual_lane_mgr_lc_wait.has_origin_lane()) {
        virtual_lane_mgr_lc_wait.set_fix_lane(ORIGIN_LANE);
      } else {
        virtual_lane_mgr_lc_wait.update_fix_lane();
      }

      virtual_lane_mgr_lc_wait.update_fix_lane_info();

      auto lane_change_info = state_machine->decide_lc_valid_info(
          virtual_lane_mgr_lc, lc_request, TARGET_LANE);

      // construct lc change scenario
      auto lc_change_context = context;
      auto lc_wait_context = context;
      if (lc_request == LEFT_CHANGE) {
        lc_change_context.name = type2name<RoadState::LC::LChange>::name;
        lc_change_context.state = type2int<RoadState::LC::LChange>::value;
        lc_wait_context.name = type2name<RoadState::LC::LWait>::name;
        lc_wait_context.state = type2int<RoadState::LC::LWait>::value;
      } else {
        lc_change_context.name = type2name<RoadState::LC::RChange>::name;
        lc_change_context.state = type2int<RoadState::LC::RChange>::value;
        lc_wait_context.name = type2name<RoadState::LC::RWait>::name;
        lc_wait_context.state = type2int<RoadState::LC::RWait>::value;
      }
      lc_change_scenario_arbitrator =
          std::make_shared<SingleScenarioArbitrator>(context,
                                                     "wait_lc_change_scenario");
      lc_change_scenario_arbitrator->init_from_type(
          ScenarioFacadeConfig::ScenarioFacadeType::ChangeLaneCruise);
      // to do: set right lc change related info in scenario context
      state_machine->generate_state_machine_output(
          lc_change_context.state, lc_change_context.name,
          virtual_lane_mgr_lc_change, lane_change_info,
          lc_change_scenario_arbitrator->get_mutable_scenario_facade()
              .get_scenario_context());

      // construct lc wait scenario
      lc_wait_scenario_arbitrator = std::make_shared<SingleScenarioArbitrator>(
          context, "wait_lc_wait_scenario");
      lc_wait_scenario_arbitrator->init_from_type(
          ScenarioFacadeConfig::ScenarioFacadeType::ChangelanePreparation);
      // to do: set right lc wait related info in scenario context
      state_machine->generate_state_machine_output(
          lc_wait_context.state, lc_wait_context.name, virtual_lane_mgr_lc_wait,
          lane_change_info,
          lc_wait_scenario_arbitrator->get_mutable_scenario_facade()
              .get_scenario_context());

      if (must_change_lane) {
        // satisfy condition
        // 0 left 1 right
        bool is_solid_line = false;
        if (lc_request == LEFT_CHANGE)
          is_solid_line = state_machine->world_model()
                              ->get_mutable_map_info_manager()
                              .is_solid_line(0);
        else
          is_solid_line = state_machine->world_model()
                              ->get_mutable_map_info_manager()
                              .is_solid_line(1);
        bool valid_body = state_machine->check_lc_body_valid(lc_request);
        bool is_lane_stable = false;
        is_lane_stable = state_machine->is_lc_lane_stable(lc_request);
        // go to lc_wait or lc_change
        if (curr_time > lc_tstart + delay_time &&
            lane_change_info.gap_insertable && valid_body && is_lane_stable &&
            !is_solid_line) {
          // go to lc_change > lc_wait
          state_machine_output_mutable.lane_change_condition =
              LaneChangeCondition ::SUITABLE_LCC;
          std::shared_ptr<PriorityArbitrator> priority_arbitrator =
              std::make_shared<PriorityArbitrator>(
                  context, "wait_lane_change_or_wait_arbitrator");

          priority_arbitrator->add_sub_arbitrators(
              lc_change_scenario_arbitrator);
          priority_arbitrator->add_sub_arbitrators(lc_wait_scenario_arbitrator);
          ArbitratorCostStrategy::set_arbitrator_cost_func(
              priority_arbitrator, ArbitratorCostType::NORMAL);
          priority_arbitrator->arbitrate();
          auto best_solution = priority_arbitrator->get_best_solution();
          mph_assert(best_solution != nullptr);
          if (best_solution && best_solution->get_optimal_scenario_context()
                                       ->state_machine_output()
                                       .curr_state == lc_change_context.state) {
            if (lc_change_context.state ==
                type2int<RoadState::LC::RChange>::value) {
              change_state<RoadState::LC::RChange>(control, context);
              state_machine->set_lc_state("right lanechange:must_change_lane");
              state_machine->set_state_change_reason("arbitrator is ok");
            } else {
              change_state<RoadState::LC::LChange>(control, context);
              state_machine->set_lc_state("left lanechange:must_change_lane");
              state_machine->set_state_change_reason("arbitrator is ok");
            }
            state_machine_output_mutable.int_cancel_reason_fsm = NO_CANCEL;
            state_machine->clear_lc_valid_cnt();
            state_machine->set_get_dist_lane(true);
            state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_change);
            lc_change_scenario_arbitrator->get_mutable_scenario_facade()
                .output_planning_context();
            state_machine->post_process();
            return;
          } else {
            // choose to go to lc_wait
            state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_wait);
            lc_wait_scenario_arbitrator->get_mutable_scenario_facade()
                .output_planning_context();
            state_machine->post_process();
            return;
          }
        } else {
          if (lane_change_info.gap_insertable == false) {
            state_machine_output_mutable.lane_change_condition =
                LaneChangeCondition::UNINSERTABLE_LCC;
          } else if (is_solid_line) {
            state_machine_output_mutable.lane_change_condition =
                LaneChangeCondition::SOLID_LINE_LCC;
          } else if (!valid_body) {
            state_machine_output_mutable.lane_change_condition =
                LaneChangeCondition::VEHICLE_UNSUITABLE_LCC;
          } else if (!is_lane_stable) {
            state_machine_output_mutable.lane_change_condition =
                LaneChangeCondition::ENV_UNSUITABLE_LCC;
          } else {
            state_machine_output_mutable.lane_change_condition =
                LaneChangeCondition::UNINSERTABLE_LCC;
          }
          state_machine->set_state_change_reason(
              "time or gap not suitable or steer limit");
          state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_wait);
          lc_wait_scenario_arbitrator->arbitrate();
          lc_wait_scenario_arbitrator->get_mutable_scenario_facade()
              .output_planning_context();

          state_machine->post_process();
          return;
        }
      } else {
        // go to lk or lc_wait/lc_change
        if (curr_time > lc_tstart + delay_time &&
            lane_change_info.gap_insertable) {
          // go to lc_wait or lc_change
          std::vector<std::shared_ptr<Arbitrator>> sub_arbitrators;
          sub_arbitrators.push_back(lane_keep_scenario_arbitrator);
          sub_arbitrators.push_back(lc_wait_scenario_arbitrator);
          sub_arbitrators.push_back(lc_change_scenario_arbitrator);
          std::shared_ptr<MeritArbitrator> merit_arbitrator =
              std::make_shared<MeritArbitrator>(
                  context, "wait_lane_change_or_keep_arbitrator");
          merit_arbitrator->set_sub_arbitrators(sub_arbitrators);
          if (lc_req_mgr.act_request_source() == "avoid_int_cutin_obj") {
            ArbitratorCostStrategy::set_arbitrator_cost_func(
                merit_arbitrator,
                ArbitratorCostType::NONESSENTIAL_LANECHANGE_MANEUVER);
          } else {
            ArbitratorCostStrategy::set_arbitrator_cost_func(
                merit_arbitrator, ArbitratorCostType::NORMAL);
          }
          merit_arbitrator->arbitrate();
          auto best_solution = merit_arbitrator->get_best_solution();
          mph_assert(best_solution != nullptr);
          MSD_LOG(INFO, "arbitrator active cost change %f wait %f keep %f",
                  lc_change_scenario_arbitrator->cost(),
                  lc_wait_scenario_arbitrator->cost(),
                  lane_keep_scenario_arbitrator->cost());
          if ((best_solution &&
               best_solution->get_optimal_scenario_context()
                       ->state_machine_output()
                       .curr_state == lc_change_context.state) ||
              (lc_change_scenario_arbitrator->cost() <
                   lane_keep_scenario_arbitrator->cost() &&
               lc_change_scenario_arbitrator->cost() <
                   std::numeric_limits<double>::infinity())) {
            // lc_change is optimal
            if (lc_request == LEFT_CHANGE) {
              change_state<RoadState::LC::LChange>(control, context);
              state_machine->set_lc_state("left lanechange");
              state_machine->set_state_change_reason("arbitrator is lc");
            } else {
              change_state<RoadState::LC::RChange>(control, context);
              state_machine->set_lc_state("right lanechange");
              state_machine->set_state_change_reason("arbitrator is lc");
            }
            state_machine->clear_lc_valid_cnt();
            state_machine->set_get_dist_lane(true);
            state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_change);
            lc_change_scenario_arbitrator->get_mutable_scenario_facade()
                .output_planning_context();
            state_machine->post_process();
            return;
          } else if (curr_time - context.entry_time <
                         wait_to_keep_protect_time ||
                     (best_solution &&
                      best_solution->get_optimal_scenario_context()
                              ->state_machine_output()
                              .curr_state == lc_wait_context.state)) {
            state_machine->set_lc_invalid_reason(
                "active_lc_change_not_optimal_currently");
            state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_wait);
            lc_wait_scenario_arbitrator->get_mutable_scenario_facade()
                .output_planning_context();
            state_machine->post_process();
            return;
          } else {
            // lane keep is optimal
            change_state<RoadState::None>(control, context);
            state_machine->set_lc_state("lane follow");
            state_machine->set_state_change_reason(
                "arbitrate lanefollow better");
            state_machine->set_lc_invalid_reason("lc_change_not_optimal");
            state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lane_keep);
            lane_keep_scenario_arbitrator->get_mutable_scenario_facade()
                .output_planning_context();
            state_machine->post_process();
            return;
          }
        } else if (lane_change_info.gap_valid) {
          // go to lk or lc_wait(approach or indicate intention)
          std::vector<std::shared_ptr<Arbitrator>> sub_arbitrators;
          sub_arbitrators.push_back(lane_keep_scenario_arbitrator);
          sub_arbitrators.push_back(lc_wait_scenario_arbitrator);
          std::shared_ptr<MeritArbitrator> merit_arbitrator =
              std::make_shared<MeritArbitrator>(
                  context, "wait_lc_wait_or_keep_arbitrator");
          merit_arbitrator->set_sub_arbitrators(sub_arbitrators);
          if (lc_req_mgr.act_request_source() == "avoid_int_cutin_obj") {
            ArbitratorCostStrategy::set_arbitrator_cost_func(
                merit_arbitrator,
                ArbitratorCostType::NONESSENTIAL_LANECHANGE_MANEUVER);
          } else {
            ArbitratorCostStrategy::set_arbitrator_cost_func(
                merit_arbitrator, ArbitratorCostType::NORMAL);
          }
          merit_arbitrator->arbitrate();
          auto best_solution = merit_arbitrator->get_best_solution();
          mph_assert(best_solution != nullptr);
          if (curr_time - context.entry_time < wait_to_keep_protect_time ||
              (best_solution && best_solution->get_optimal_scenario_context()
                                        ->state_machine_output()
                                        .curr_state == lc_wait_context.state)) {
            state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_wait);
            lc_wait_scenario_arbitrator->get_mutable_scenario_facade()
                .output_planning_context();
            state_machine->post_process();
            return;
          } else {
            if (lc_state == "forward_lc") {
              change_state<RoadState::None>(control, context);
              state_machine->set_lc_state("lane follow");
              state_machine->set_state_change_reason(
                  "arbitrate lane follow better, forward lc");
            }
            state_machine->set_lc_invalid_reason("lc_change_not_optimal");
            state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lane_keep);
            lane_keep_scenario_arbitrator->get_mutable_scenario_facade()
                .output_planning_context();
            state_machine->post_process();
            return;
          }
        } else {
          // go to lk
          change_state<RoadState::None>(control, context);
          state_machine->set_lc_state("lane follow");
          state_machine->set_state_change_reason("gap not valid");
          // state_machine->set_lc_invalid_reason("lc_change_not_valid");
          state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lane_keep);
          lane_keep_scenario_arbitrator->arbitrate();
          lane_keep_scenario_arbitrator->get_mutable_scenario_facade()
              .output_planning_context();
          state_machine->post_process();
          return;
        }
      }
    } else {
      // go to lk
      change_state<RoadState::None>(control, context);
      state_machine->set_lc_state("lane follow");
      state_machine->set_state_change_reason("no lc lane");
      state_machine->set_lc_invalid_reason("lc_change_request_canceled");
      state_machine_output_mutable.int_cancel_reason_fsm = ENV_ERROR;
      state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lane_keep);
      lane_keep_scenario_arbitrator->arbitrate();
      lane_keep_scenario_arbitrator->get_mutable_scenario_facade()
          .output_planning_context();
      state_machine->post_process();
      return;
    }
  } else {
    change_state<RoadState::None>(control, context);
    state_machine->set_lc_state("lane follow");
    state_machine->set_state_change_reason("no lc request");
    virtual_lane_mgr.clear_lc_lanes();
    state_machine->clear_lc_variables();
    virtual_lane_mgr.set_fix_lane(CURRENT_LANE);
  }

  state_machine->update_virtual_lane_mgr(virtual_lane_mgr);
  state_machine->post_process();
}

void RoadState::LC::process_change(Control &control, FsmContext &context) {
  int state = context.state;

  FsmUserContext *user_context =
      static_cast<FsmUserContext *>(context.user_data);

  if (user_context == nullptr || user_context->state_machine == nullptr) {
    return;
  }
  auto &state_machine_output_mutable =
      PlanningContext::Instance()->mutable_state_machine_output();
  MSDStateMachine *state_machine = user_context->state_machine;
  state_machine->set_entry_time(context.entry_time);
  state_machine->set_gmp_should_cancel(false);
  auto &int_cancel_reason = state_machine->int_cancel_reason();
  int_cancel_reason = NO_CANCEL;
  state_machine_output_mutable.int_cancel_reason_fsm = int_cancel_reason;
  state_machine_output_mutable.lane_change_condition =
      LaneChangeCondition::SUITABLE_LCC;
  // auto &virtual_lane_mgr = *state_machine->virtual_lane_mgr();
  static VirtualLaneManager virtual_lane_mgr(
      *state_machine->virtual_lane_mgr());
  virtual_lane_mgr.update(*state_machine->virtual_lane_mgr());
  auto &lc_req_mgr = *state_machine->lc_request_manager();
  auto &map_info = context.world_model->get_map_info();

  int lc_request = lc_req_mgr.request();
  int lc_source = lc_req_mgr.request_source();
  auto direction = (state == ROAD_LC_LCHANGE) ? LEFT_CHANGE : RIGHT_CHANGE;

  virtual_lane_mgr.update_lc_lanes(direction, state);
  constexpr int kMaxInvalidLaneChangeCount = 3;

  if (virtual_lane_mgr.has_target_lane()) {
    if (state_machine->check_lc_finish(virtual_lane_mgr, direction)) {
      change_state<RoadState::None>(control, context);
      state_machine->set_lc_state("lane follow");
      state_machine->set_state_change_reason("lanechange is over");
      lc_req_mgr.finish_request();
      virtual_lane_mgr.clear_lc_lanes();
      state_machine->clear_lc_variables();
      if (lc_source == ACT_REQUEST) {
        state_machine->set_active_request_finished(true);
        state_machine->set_active_request_finished_dir(direction);
        state_machine->set_map_request_finished(false);
        MSD_LOG(INFO, "ALC_DEBUG set_active_request_finished");
      } else if (lc_source == MAP_REQUEST) {
        state_machine->set_map_request_finished(true);
        state_machine->set_active_request_finished(false);
        MSD_LOG(INFO, "MLC_DEBUG set_map_request_finished");
      }
      virtual_lane_mgr.set_fix_lane(CURRENT_LANE);
    } else {
      // construct lc_back scenario
      static VirtualLaneManager virtual_lane_mgr_lc_back(virtual_lane_mgr);
      virtual_lane_mgr_lc_back.update(virtual_lane_mgr);
      if (virtual_lane_mgr_lc_back.has_origin_lane()) {
        virtual_lane_mgr_lc_back.set_fix_lane(ORIGIN_LANE);
        virtual_lane_mgr_lc_back.update_fix_lane_info();
      }
      auto lc_back_context = context;
      auto lc_change_context = context;
      if (state == ROAD_LC_LCHANGE) {
        lc_back_context.name = type2name<RoadState::LC::LBack>::name;
        lc_back_context.state = type2int<RoadState::LC::LBack>::value;
        lc_change_context.name = type2name<RoadState::LC::LChange>::name;
        lc_change_context.state = type2int<RoadState::LC::LChange>::value;
      } else {
        lc_back_context.name = type2name<RoadState::LC::RBack>::name;
        lc_back_context.state = type2int<RoadState::LC::RBack>::value;
        lc_change_context.name = type2name<RoadState::LC::RChange>::name;
        lc_change_context.state = type2int<RoadState::LC::RChange>::value;
      }
      std::shared_ptr<SingleScenarioArbitrator> lc_back_scenario_arbitrator =
          std::make_shared<SingleScenarioArbitrator>(context,
                                                     "change_lc_back_scenario");
      lc_back_scenario_arbitrator->init_from_type(
          ScenarioFacadeConfig::ScenarioFacadeType::ChangeLaneCruise);

      // construct lc_change scenario(same direction)
      std::shared_ptr<SingleScenarioArbitrator> lc_change_scenario_arbitrator;
      static VirtualLaneManager virtual_lane_mgr_lc_change(virtual_lane_mgr);
      virtual_lane_mgr_lc_change.update(virtual_lane_mgr);
      virtual_lane_mgr_lc_change.set_fix_lane(TARGET_LANE);
      virtual_lane_mgr_lc_change.update_fix_lane_info();
      lc_change_scenario_arbitrator =
          std::make_shared<SingleScenarioArbitrator>(
              context, "change_lc_change_scenario");
      lc_change_scenario_arbitrator->init_from_type(
          ScenarioFacadeConfig::ScenarioFacadeType::ChangeLaneCruise);
      LaneChangeStageInfo lc_info;
      // use gap selected in wait stage
      lc_info.gap = PlanningContext::Instance()
                        ->planning_status()
                        .lane_status.change_lane.target_gap_obs;
      lc_info.enable_interactive_mode |=
          PlanningContext::Instance()
              ->planning_status()
              .lane_status.change_lane.enable_interactive_mode;
      MSD_LOG(INFO,
              "arbitrator gap [%d %d] enable_interactive_mode %d lane_id %d",
              lc_info.gap.first, lc_info.gap.second,
              lc_info.enable_interactive_mode,
              virtual_lane_mgr_lc_change.get_fix_refline().position());
      state_machine->generate_state_machine_output(
          lc_change_context.state, lc_change_context.name,
          virtual_lane_mgr_lc_change, lc_info,
          lc_change_scenario_arbitrator->get_mutable_scenario_facade()
              .get_scenario_context());

      bool is_consistent_request =
          (state == ROAD_LC_LCHANGE && lc_request == LEFT_CHANGE) ||
          (state == ROAD_LC_RCHANGE && lc_request == RIGHT_CHANGE);
      MSD_LOG(INFO,
              "arbitrator is_consistent_request %d  lc_request %d "
              "is_on_target_lane %d",
              is_consistent_request, lc_request,
              virtual_lane_mgr.is_on_lane(TARGET_LANE));
      if (is_consistent_request || (lc_request == NO_CHANGE &&
                                    virtual_lane_mgr.is_on_lane(TARGET_LANE))) {
        if (virtual_lane_mgr.has_origin_lane()) {
          MSD_LOG(INFO, "arbitrator has_origin_lane");
          auto lc_back_info =
              state_machine->decide_lc_back_info(virtual_lane_mgr, direction);
          state_machine->generate_state_machine_output(
              lc_back_context.state, lc_back_context.name,
              virtual_lane_mgr_lc_back, lc_back_info,
              lc_back_scenario_arbitrator->get_mutable_scenario_facade()
                  .get_scenario_context());
          if (lc_back_info.lc_should_back) {
            // go to lc_back > ?(suspend evadeobject)
            MSD_LOG(INFO, "arbitrator lc should back");
            state_machine_output_mutable.lane_change_condition =
                LaneChangeCondition::UNINSERTABLE_LCC;
            if (state == ROAD_LC_LCHANGE) {
              change_state<RoadState::LC::LBack>(control, context);
              state_machine->set_lc_state("left lanechange back");
              state_machine->set_state_change_reason(
                  "lc should back :decide_lc_back");
            } else {
              change_state<RoadState::LC::RBack>(control, context);
              state_machine->set_lc_state("right lanechange back");
              state_machine->set_state_change_reason(
                  "lc should back :decide_lc_back");
            }
            state_machine->clear_lc_back_cnt();
            state_machine->set_lc_back_reason(lc_back_info.lc_back_reason);
            state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_back);
            lc_back_scenario_arbitrator->arbitrate();
            lc_back_scenario_arbitrator->get_mutable_scenario_facade()
                .output_planning_context();
            state_machine->post_process();
            return;
          } else if (lc_back_info.lc_should_reset) {
            state_machine_output_mutable.lane_change_condition =
                LaneChangeCondition::UNINSERTABLE_LCC;
            MSD_LOG(INFO, "reset blocked lane change");
            change_state<RoadState::None>(control, context);
            state_machine->set_lc_state("lane follow");
            state_machine->set_state_change_reason(
                "lc should reset :decide_lc_back");
            lc_req_mgr.finish_request();
            virtual_lane_mgr.set_fix_lane(CURRENT_LANE);
          } else {
            // go to lc_change > lc_back
            lc_back_info.gap = PlanningContext::Instance()
                                   ->planning_status()
                                   .lane_status.change_lane.target_gap_obs;
            lc_back_info.enable_interactive_mode |=
                PlanningContext::Instance()
                    ->planning_status()
                    .lane_status.change_lane.enable_interactive_mode;
            state_machine->generate_state_machine_output(
                lc_change_context.state, lc_change_context.name,
                virtual_lane_mgr_lc_change, lc_back_info,
                lc_change_scenario_arbitrator->get_mutable_scenario_facade()
                    .get_scenario_context());
            std::shared_ptr<PriorityArbitrator> priority_arbitrator =
                std::make_shared<PriorityArbitrator>(
                    context, "change_lane_change_or_back_arbitrator");

            priority_arbitrator->add_sub_arbitrators(
                lc_change_scenario_arbitrator);
            priority_arbitrator->add_sub_arbitrators(
                lc_back_scenario_arbitrator);
            ArbitratorCostStrategy::set_arbitrator_cost_func(
                priority_arbitrator, ArbitratorCostType::NORMAL);
            priority_arbitrator->arbitrate();
            auto best_solution = priority_arbitrator->get_best_solution();
            mph_assert(best_solution != nullptr);
            // choose to keep lane change stage
            state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_change);
            lc_change_scenario_arbitrator->get_mutable_scenario_facade()
                .output_planning_context();
            state_machine->post_process();
            return;
          }
        } else {
          MSD_LOG(INFO, "arbitrator not has_origin_lane");
          // keep lc_change
          state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_change);
          lc_change_scenario_arbitrator->arbitrate();
          lc_change_scenario_arbitrator->get_mutable_scenario_facade()
              .output_planning_context();
          state_machine->post_process();
          return;
        }
      } else if (lc_request == NO_CHANGE &&
                 ((lc_source == INT_REQUEST &&
                   !state_machine->check_head_crosss(virtual_lane_mgr,
                                                     direction)) ||
                  (lc_source != INT_REQUEST &&
                   !virtual_lane_mgr.is_on_lane(TARGET_LANE)))) {
        if (lc_request == NO_CHANGE)
          int_cancel_reason = lc_req_mgr.get_int_request_cancel_reason();
        state_machine->set_state_change_reason(
            "lc request cancel and not cross lane");
        state_machine_output_mutable.lane_change_condition =
            LaneChangeCondition::DEFAULT_LCC;
        MSD_LOG(INFO, "not_suitable change to defalut");
        state_machine_output_mutable.int_cancel_reason_fsm = int_cancel_reason;
        MSD_LOG(INFO, "arbitrator NO_CHANGE & TARGET_LANE");
        // MSD_LOG(INFO, "lc_source:
        // %d,%d,%d",lc_source,state_machine->check_head_crosss(virtual_lane_mgr,
        //                                              direction),virtual_lane_mgr.is_on_lane(TARGET_LANE));
        // invalid input go to None
        change_state<RoadState::None>(control, context);
        int_cancel_reason = lc_req_mgr.get_int_request_cancel_reason();
        state_machine_output_mutable.int_cancel_reason_fsm = int_cancel_reason;
        state_machine->set_lc_state("lane follow");
        state_machine->set_gmp_should_cancel(true);
        state_machine->set_state_change_reason(
            "request disappear and not on target lane");
        virtual_lane_mgr.clear_lc_lanes();
        state_machine->clear_lc_variables();
        virtual_lane_mgr.set_fix_lane(CURRENT_LANE);
      } else if ((state == ROAD_LC_LCHANGE && lc_request == RIGHT_CHANGE) ||
                 (state == ROAD_LC_RCHANGE && lc_request == LEFT_CHANGE)) {
        MSD_LOG(INFO, "arbitrator reverse change");
        virtual_lane_mgr.clear_lc_lanes();
        state_machine->clear_lc_variables();

        if (virtual_lane_mgr.assign_lc_lanes(lc_request)) {
          if (state_machine->check_lc_valid(virtual_lane_mgr, lc_request,
                                            TARGET_LANE)) {
            if (state == ROAD_LC_LCHANGE) {
              change_state<RoadState::LC::RChange>(control, context);
              state_machine->set_lc_state("right lanechange");
              state_machine->set_state_change_reason(
                  "reverse request and lc valid, left to right");
            } else {
              change_state<RoadState::LC::LChange>(control, context);
              state_machine->set_lc_state("left lanechange");
              state_machine->set_state_change_reason(
                  "reverse request and lc valid, right to left");
            }
            state_machine->set_get_dist_lane(true);
            virtual_lane_mgr.set_fix_lane(TARGET_LANE);
          } else {
            if (state == ROAD_LC_LCHANGE) {
              change_state<RoadState::LC::RWait>(control, context);
              state_machine->set_lc_state("right lanechange wait");
              state_machine->set_state_change_reason(
                  "reverse request but lc not valid, left to right");
            } else {
              change_state<RoadState::LC::LWait>(control, context);
              state_machine->set_lc_state("left lanechange wait");
              state_machine->set_state_change_reason(
                  "reverse request but lc not valid, right to left");
            }
            virtual_lane_mgr.set_fix_lane(ORIGIN_LANE);
          }
        } else {
          change_state<RoadState::None>(control, context);
          state_machine->set_lc_state("lane follow");
          state_machine->set_state_change_reason(
              "reverse request but not assign lc lanes");
          virtual_lane_mgr.set_fix_lane(CURRENT_LANE);
        }
      }
    }
  } else if (virtual_lane_mgr.has_origin_lane()) {
    MSD_LOG(INFO, "arbitrator has_origin_lane not target_lane");
    state_machine_output_mutable.lane_change_condition =
        LaneChangeCondition::ENV_UNSUITABLE_LCC;
    if (state == ROAD_LC_LCHANGE) {
      change_state<RoadState::LC::LBack>(control, context);
      state_machine->set_lc_state("left back lanechange");
      state_machine->set_state_change_reason(
          "target lane lost and have origin lane");
    } else {
      change_state<RoadState::LC::RBack>(control, context);
      state_machine->set_lc_state("right back lanechange");
      state_machine->set_state_change_reason(
          "target lane lost and have origin lane");
    }
    state_machine->set_lc_back_reason("no_target_lane");
    state_machine->clear_lc_back_cnt();
    virtual_lane_mgr.set_fix_lane(ORIGIN_LANE);
  } else if (virtual_lane_mgr.lc_lanes_lost()) {
    MSD_LOG(INFO, "arbitrator not has_origin_lane not target_lane");
    lc_req_mgr.finish_request();
    state_machine_output_mutable.lane_change_condition =
        LaneChangeCondition::ENV_UNSUITABLE_LCC;
    change_state<RoadState::None>(control, context);
    state_machine->set_lc_state("lane follow");
    state_machine->set_state_change_reason("target lane and origin lane lost");
    virtual_lane_mgr.set_fix_lane(CURRENT_LANE);
  }

  state_machine->update_virtual_lane_mgr(virtual_lane_mgr);
  state_machine->post_process();
}

void RoadState::LC::process_back(Control &control, FsmContext &context) {
  int state = context.state;

  FsmUserContext *user_context =
      static_cast<FsmUserContext *>(context.user_data);

  if (user_context == nullptr || user_context->state_machine == nullptr) {
    return;
  }
  auto &state_machine_output_mutable =
      PlanningContext::Instance()->mutable_state_machine_output();
  MSDStateMachine *state_machine = user_context->state_machine;
  state_machine->set_entry_time(context.entry_time);
  state_machine->set_gmp_should_cancel(false);

  // auto &virtual_lane_mgr = *state_machine->virtual_lane_mgr();
  static VirtualLaneManager virtual_lane_mgr(
      *state_machine->virtual_lane_mgr());
  virtual_lane_mgr.update(*state_machine->virtual_lane_mgr());
  auto &lc_req_mgr = *state_machine->lc_request_manager();

  int lc_request = lc_req_mgr.request();
  auto direction = (state == ROAD_LC_LBACK) ? LEFT_CHANGE : RIGHT_CHANGE;

  virtual_lane_mgr.update_lc_lanes(direction, state);

  if (virtual_lane_mgr.has_origin_lane()) {
    if (state_machine->check_lc_back_finish(virtual_lane_mgr, direction)) {
      change_state<RoadState::None>(control, context);
      lc_req_mgr.finish_request();
      state_machine->set_lc_state("lane follow");
      state_machine->set_state_change_reason("back finish");
      state_machine->clear_lc_variables();
      virtual_lane_mgr.set_fix_lane(CURRENT_LANE);
    } else {
      if ((state == ROAD_LC_LBACK && lc_request == LEFT_CHANGE) ||
          (state == ROAD_LC_RBACK && lc_request == RIGHT_CHANGE)) {
        // construct lc back & lc change scenario
        auto lc_back_context = context;
        auto lc_change_context = context;
        if (state == ROAD_LC_LBACK) {
          lc_back_context.name = type2name<RoadState::LC::LBack>::name;
          lc_back_context.state = type2int<RoadState::LC::LBack>::value;
        } else {
          lc_back_context.name = type2name<RoadState::LC::RBack>::name;
          lc_back_context.state = type2int<RoadState::LC::RBack>::value;
        }
        std::shared_ptr<SingleScenarioArbitrator> lc_back_scenario_arbitrator =
            std::make_shared<SingleScenarioArbitrator>(context,
                                                       "back_lc_back_scenario");
        lc_back_scenario_arbitrator->init_from_type(
            ScenarioFacadeConfig::ScenarioFacadeType::ChangeLaneCruise);
        auto lane_change_info = state_machine->decide_lc_valid_info(
            virtual_lane_mgr, lc_request, TARGET_LANE);
        static VirtualLaneManager virtual_lane_mgr_lc_back(virtual_lane_mgr);
        virtual_lane_mgr_lc_back.update(virtual_lane_mgr);
        virtual_lane_mgr_lc_back.set_fix_lane(ORIGIN_LANE);
        virtual_lane_mgr_lc_back.update_fix_lane_info();
        state_machine->generate_state_machine_output(
            lc_back_context.state, lc_back_context.name,
            virtual_lane_mgr_lc_back, lane_change_info,
            lc_back_scenario_arbitrator->get_mutable_scenario_facade()
                .get_scenario_context());
        lc_back_scenario_arbitrator->arbitrate();
        state_machine->update_virtual_lane_mgr(virtual_lane_mgr_lc_back);
        lc_back_scenario_arbitrator->get_mutable_scenario_facade()
            .output_planning_context();
        state_machine->post_process();
        return;
      } else {
        change_state<RoadState::None>(control, context);
        lc_req_mgr.finish_request();
        state_machine->set_lc_state("lane follow");
        state_machine->set_state_change_reason(
            "have origin lane but have no request");
        virtual_lane_mgr.clear_lc_lanes();
        state_machine->clear_lc_variables();
        state_machine->clear_lc_pause_variables();
        virtual_lane_mgr.set_fix_lane(CURRENT_LANE);
      }
    }
  } else {
    change_state<RoadState::None>(control, context);
    lc_req_mgr.finish_request();
    state_machine->set_lc_state("lane follow");
    state_machine->set_state_change_reason("have no o/t lane");
    state_machine->clear_lc_variables();
    virtual_lane_mgr.set_fix_lane(CURRENT_LANE);
  }

  state_machine->update_virtual_lane_mgr(virtual_lane_mgr);
  state_machine->post_process();
}

void RoadState::LC::LWait::callback(Control &control, FsmContext &context) {
  process_wait(control, context);
}

void RoadState::LC::RWait::callback(Control &control, FsmContext &context) {
  process_wait(control, context);
}

void RoadState::LC::LChange::callback(Control &control, FsmContext &context) {
  process_change(control, context);
}

void RoadState::LC::RChange::callback(Control &control, FsmContext &context) {
  process_change(control, context);
}

void RoadState::LC::LBack::callback(Control &control, FsmContext &context) {
  process_back(control, context);
}

void RoadState::LC::RBack::callback(Control &control, FsmContext &context) {
  process_back(control, context);
}

void RoadState::LB::process_borrow(const Control &control,
                                   const FsmContext &context) {}

void RoadState::LB::process_back(const Control &control,
                                 const FsmContext &context) {}

void RoadState::LB::process_return(const Control &control,
                                   const FsmContext &context) {}

void RoadState::LB::process_suspend(const Control &control,
                                    const FsmContext &context) {}

void RoadState::LB::LBorrow::callback(Control &control, FsmContext &context) {
  process_borrow(control, context);
}

void RoadState::LB::RBorrow::callback(Control &control, FsmContext &context) {
  process_borrow(control, context);
}

void RoadState::LB::LBack::callback(Control &control, FsmContext &context) {
  process_back(control, context);
}

void RoadState::LB::RBack::callback(Control &control, FsmContext &context) {
  process_back(control, context);
}

void RoadState::LB::LReturn::callback(Control &control, FsmContext &context) {
  process_return(control, context);
}

void RoadState::LB::RReturn::callback(Control &control, FsmContext &context) {
  process_return(control, context);
}

void RoadState::LB::LSuspend::callback(Control &control, FsmContext &context) {
  process_suspend(control, context);
}

void RoadState::LB::RSuspend::callback(Control &control, FsmContext &context) {
  process_suspend(control, context);
}

} // namespace msquare
