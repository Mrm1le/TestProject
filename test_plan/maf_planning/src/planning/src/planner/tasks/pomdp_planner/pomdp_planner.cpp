#include "planner/tasks/pomdp_planner/pomdp_planner.h"
#include "common/config_context.h"
#include "despot/core/pomdp_world.h"
#include "planner/behavior_planner/deciders/prediction_reviser.h"
#include "planner/behavior_planner/deciders/st_graph_generator.h"

using namespace despot;

namespace msquare {

bool LCMDPPlanner::RunStep(Solver *solver, World *world, Logger *logger) {
  logger->CheckTargetTime();

  double step_start_t = get_time_second();

  double start_t = get_time_second();
  ACT_TYPE action = solver->Search().action;
  double end_t = get_time_second();
  double search_time = (end_t - start_t);
  logi << "[RunStep] Time spent in " << typeid(*solver).name()
       << "::Search(): " << search_time << endl;

  OBS_TYPE obs;
  start_t = get_time_second();
  bool terminal = world->ExecuteAction(action, obs);
  end_t = get_time_second();
  double execute_time = (end_t - start_t);
  logi << "[RunStep] Time spent in ExecuteAction(): " << execute_time << endl;

  start_t = get_time_second();
  auto &input_state = static_cast<LCAgentState &>(*world->GetCurrentState());
  // std::cout << "before belief update ego x " << input_state.ego_pos.x << "y
  // " << input_state.ego_pos.y << std::endl;
  ((LCAgentBelief *)solver->belief())->Update(*world->GetCurrentState());
  vector<despot::State *> particles =
      ((LCAgentBelief *)solver->belief())->Sample(1);
  // std::cout << "sample x : " << ((LCAgentState*)(particles[0]))->ego_pos.x<<
  // " y: " << ((LCAgentState*)(particles[0]))->ego_pos.y << std::endl;
  auto future_traj = solver->future();
  // std::cout << "future traj size: " << future_traj.Size() << std::endl;
  // std::cout << future_traj << std::endl;
  solver->BeliefUpdate(action, obs);
  end_t = get_time_second();
  double update_time = (end_t - start_t);
  logi << "[RunStep] Time spent in Update(): " << update_time << endl;

  return logger->SummarizeStep(step_++, round_, terminal, action, obs,
                               step_start_t);
}

LCMDPPlanner::LCMDPPlanner(const TaskConfig &config) : Task(config) {
  // mph_assert(config.has_lane_change_pomdp_planner_config());
  // initial_rival_aggressiveness_ =
  //     config.lane_change_pomdp_planner_config().initial_rival_aggressiveness;
  // time_horizon_ = config.lane_change_pomdp_planner_config().time_horizon;
  // time_step_ = config.lane_change_pomdp_planner_config().time_step;
  // MSD_LOG(INFO, "[POMDP_PLANNER] config rival aggressiveness %f time horizon
  // %f time step %f", initial_rival_aggressiveness_, time_horizon_,
  // time_step_);
}

void LCMDPPlanner::init(std::shared_ptr<WorldModel> world_model) {
  Task::init(world_model);
}

void LCMDPPlanner::set_npc_car(std::shared_ptr<BaseLineInfo> origin_lane,
                               std::shared_ptr<BaseLineInfo> target_lane,
                               std::shared_ptr<BaseLineInfo> current_lane,
                               LCEnv *model) {
  // std::vector<const Obstacle *> npc_cars;
  auto &obstacle_manager = current_lane->mutable_obstacle_manager();
  const auto &planning_status = context_->planning_status();
  const auto &gap_info = planning_status.lane_status.change_lane.target_gap_obs;
  auto direction = planning_status.lane_status.change_lane.direction;
  double target_lane_rival__end_s =
      obstacle_manager.find_obstacle(target_lane_rival_id_)
          ->PerceptionSLBoundary()
          .end_s;
  bool is_in_lc_wait{false};
  if (planning_status.lane_status.status == LaneStatus::LANE_CHANGE) {
    if (planning_status.lane_status.change_lane.status ==
        ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION) {
      is_in_lc_wait = true;
    }
  }
  if (is_in_lc_wait) {
    auto npc_obstacle = obstacle_manager.find_obstacle(gap_info.first);
    if (npc_obstacle && !npc_obstacle->IsFrenetInvalid()) {
      // npc_cars.push_back(npc_obstacle);
      if (std::find(related_obstacles_.begin(), related_obstacles_.end(),
                    npc_obstacle->Id()) == related_obstacles_.end()) {
        related_obstacles_.emplace_back(npc_obstacle->Id());
        if (!npc_obstacle->has_sl_polygon_seq() &&
            !npc_obstacle->is_sl_polygon_seq_invalid()) {
          std::string fail_reason{};
          StGraphGenerator::compute_obs_sl_polygon_seq(
              npc_obstacle, current_lane, {0.0, 8.0}, 0.2, true, fail_reason);
        }
        if (npc_obstacle->has_sl_polygon_seq()) {
          model->FeedNPCPredictionWithDecision(npc_obstacle, "follow");
        }
      }
    }
  } else {
    std::vector<int> target_lane_npc_car_ids;
    auto target_lane_leader_car =
        find_leader_car_in_target_lane(target_lane, target_lane_npc_car_ids);
    if (target_lane_leader_car) {
      auto npc_obstacle =
          obstacle_manager.find_obstacle(target_lane_leader_car->Id());
      if (npc_obstacle && !npc_obstacle->IsFrenetInvalid()) {
        // npc_cars.push_back(npc_obstacle);
        if (std::find(related_obstacles_.begin(), related_obstacles_.end(),
                      npc_obstacle->Id()) == related_obstacles_.end()) {
          related_obstacles_.emplace_back(npc_obstacle->Id());
          if (!npc_obstacle->has_sl_polygon_seq() &&
              !npc_obstacle->is_sl_polygon_seq_invalid()) {
            std::string fail_reason{};
            StGraphGenerator::compute_obs_sl_polygon_seq(
                npc_obstacle, current_lane, {0.0, 8.0}, 0.2, true, fail_reason);
          }
          if (npc_obstacle->has_sl_polygon_seq()) {
            model->FeedNPCPredictionWithDecision(npc_obstacle, "follow");
          }
        }
      }
    }
    for (auto target_lane_npc_car_id : target_lane_npc_car_ids) {
      auto npc_obstacle =
          obstacle_manager.find_obstacle(target_lane_npc_car_id);
      if (npc_obstacle && !npc_obstacle->IsFrenetInvalid()) {
        // npc_cars.push_back(npc_obstacle);
        if (std::find(related_obstacles_.begin(), related_obstacles_.end(),
                      npc_obstacle->Id()) == related_obstacles_.end()) {
          related_obstacles_.emplace_back(npc_obstacle->Id());
          if (!npc_obstacle->has_sl_polygon_seq() &&
              !npc_obstacle->is_sl_polygon_seq_invalid()) {
            std::string fail_reason{};
            StGraphGenerator::compute_obs_sl_polygon_seq(
                npc_obstacle, current_lane, {0.0, 8.0}, 0.2, true, fail_reason);
          }
          if (npc_obstacle->has_sl_polygon_seq()) {
            model->FeedNPCPredictionWithDecision(npc_obstacle, "none");
          }
        }
      }
    }
  }

  auto origin_lane_leader_car = find_leader_car_in_origin_lane(origin_lane);
  if (origin_lane_leader_car) {
    auto npc_obstacle =
        obstacle_manager.find_obstacle(origin_lane_leader_car->Id());
    if (npc_obstacle && !npc_obstacle->IsFrenetInvalid()) {
      // npc_cars.push_back(npc_obstacle);
      if (std::find(related_obstacles_.begin(), related_obstacles_.end(),
                    npc_obstacle->Id()) == related_obstacles_.end()) {
        related_obstacles_.emplace_back(npc_obstacle->Id());
        if (!npc_obstacle->has_sl_polygon_seq() &&
            !npc_obstacle->is_sl_polygon_seq_invalid()) {
          std::string fail_reason{};
          StGraphGenerator::compute_obs_sl_polygon_seq(
              npc_obstacle, current_lane, {0.0, 8.0}, 0.2, true, fail_reason);
        }
        model->FeedNPCPredictionWithDecision(npc_obstacle, "none");
      }
    }
  }
  auto origin_lane_rival_car = find_rival_car_in_origin_lane(origin_lane);
  if (origin_lane_rival_car) {
    auto npc_obstacle =
        obstacle_manager.find_obstacle(origin_lane_rival_car->Id());
    if (npc_obstacle && !npc_obstacle->IsFrenetInvalid()) {
      if (std::find(related_obstacles_.begin(), related_obstacles_.end(),
                    npc_obstacle->Id()) == related_obstacles_.end()) {
        related_obstacles_.emplace_back(npc_obstacle->Id());
        if (!npc_obstacle->has_sl_polygon_seq() &&
            !npc_obstacle->is_sl_polygon_seq_invalid()) {
          std::string fail_reason{};
          StGraphGenerator::compute_obs_sl_polygon_seq(
              npc_obstacle, current_lane, {0.0, 8.0}, 0.2, true, fail_reason);
        }
        // model->FeedNPCPredictionWithDecision(npc_obstacle, "none");
      }
    }
  }
}

const Obstacle *LCMDPPlanner::find_rival_car_in_target_lane(
    std::shared_ptr<BaseLineInfo> target_lane) {
  const Obstacle *rear_car{nullptr};
  auto &obstacle_manager = target_lane->mutable_obstacle_manager();
  bool is_in_lc_wait{false};
  const auto &planning_status = context_->planning_status();
  const auto &gap_info = planning_status.lane_status.change_lane.target_gap_obs;
  if (planning_status.lane_status.status == LaneStatus::LANE_CHANGE) {
    if (planning_status.lane_status.change_lane.status ==
        ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION) {
      is_in_lc_wait = true;
    }
  }
  if (is_in_lc_wait) {
    auto rear_car = obstacle_manager.find_obstacle(gap_info.second);
    if (rear_car && !rear_car->IsFrenetInvalid()) {
      MSD_LOG(INFO, "[POMDP_PLANNER] find_rival_car_in_target_lane id: %d",
              rear_car->Id());
      return rear_car;
    } else {
      MSD_LOG(INFO, "[POMDP_PLANNER] not find_rival_car_in_target_lane id: %d",
              gap_info.second);
    }
  }
  auto adc_sl = target_lane->get_adc_sl_boundary();
  double adc_v = target_lane->get_ego_state().ego_vel;
  auto &target_lane_path_data = target_lane->get_path_data();
  PathPoint adc_point;
  adc_point.x = target_lane->get_ego_state().ego_pose.x;
  adc_point.y = target_lane->get_ego_state().ego_pose.y;
  double adc_path_s =
      target_lane_path_data.discretized_path().QueryMatchedS(adc_point);
  auto lane_border = target_lane_path_data.getWidth(adc_path_s);
  double nearest_rear_car_dis = std::numeric_limits<double>::lowest();
  for (const auto &obstacle : obstacle_manager.get_obstacles().Items()) {
    // if (!obstacle->has_sl_polygon_seq()) {
    //   continue;
    // }
    auto obstacle_sl = obstacle->PerceptionSLBoundary();
    constexpr double kOvertakeLateralBuffer = 0.7;
    if (obstacle_sl.end_s < adc_sl.end_s &&
        obstacle_sl.end_s - adc_sl.start_s > nearest_rear_car_dis) {
      double overlap_dis = std::min(obstacle_sl.end_l, lane_border.first) -
                           std::max(obstacle_sl.start_l, lane_border.second);
      if (overlap_dis / (obstacle_sl.end_l - obstacle_sl.start_l + 1.e-3) >
          0.2) {
        nearest_rear_car_dis = obstacle_sl.end_s - adc_sl.start_s;
        MSD_LOG(
            INFO,
            "[POMDP_PLANNER] find_rival_car_in_target_lane id: %d distance: %f",
            obstacle->Id(), nearest_rear_car_dis);
        rear_car = obstacle;
      }
    }
  }

  // judge if it's a strong interactive scenario
  bool is_current_in_interactive_scenario{false};
  bool is_current_in_weak_interactive_scenario{false};
  if (rear_car && !rear_car->IsFrenetInvalid()) {
    double follow_dis = adc_sl.start_s - rear_car->PerceptionSLBoundary().end_s;
    const double kTTC = 0.8;
    const double kMaxTTC = 1.2;
    double obs_frenet_v =
        rear_car->speed() * std::cos(rear_car->Yaw_relative_frenet());
    double ref_follow_dis = (2 * obs_frenet_v - adc_v) * kTTC;
    double max_ref_follow_dis = (2 * obs_frenet_v - adc_v) * kMaxTTC;
    MSD_LOG(INFO, "[POMDP_PLANNER] follow dis %f ref_follow_dis %f", follow_dis,
            ref_follow_dis);
    if (follow_dis < ref_follow_dis) {
      is_current_in_interactive_scenario = true;
    } else if (follow_dis < max_ref_follow_dis) {
      is_current_in_weak_interactive_scenario = true;
    }
  }
  MSD_LOG(INFO, "[POMDP_PLANNER] previous mode %d current mode %d",
          context_->planning_status()
              .lane_status.change_lane.enable_interactive_mode,
          is_current_in_interactive_scenario);
  if (is_current_in_interactive_scenario) {
    context_->mutable_planning_status()
        ->lane_status.change_lane.enable_interactive_mode = true;
    return rear_car;
  } else if (context_->planning_status()
                 .lane_status.change_lane.enable_interactive_mode &&
             is_current_in_weak_interactive_scenario) {
    return rear_car;
  } else {
    return nullptr;
  }
  return rear_car;
}

const Obstacle *LCMDPPlanner::find_leader_car_in_target_lane(
    std::shared_ptr<BaseLineInfo> target_lane, std::vector<int> &npc_cars) {
  const Obstacle *leader_car{nullptr};
  auto &obstacle_manager = target_lane->mutable_obstacle_manager();
  auto &obstacle_decision_manager =
      context_->mutable_obstacle_decision_manager();
  const auto &gap_info =
      context_->planning_status().lane_status.change_lane.target_gap_obs;
  auto adc_sl = target_lane->get_adc_sl_boundary();
  auto &target_lane_path_data = target_lane->get_path_data();
  PathPoint adc_point;
  adc_point.x = target_lane->get_ego_state().ego_pose.x;
  adc_point.y = target_lane->get_ego_state().ego_pose.y;
  double adc_path_s =
      target_lane_path_data.discretized_path().QueryMatchedS(adc_point);
  auto lane_border = target_lane_path_data.getWidth(adc_path_s);
  double nearest_front_car_dis = std::numeric_limits<double>::max();
  double target_lane_rival_end_s =
      obstacle_manager.find_obstacle(target_lane_rival_id_)
          ->PerceptionSLBoundary()
          .end_s;
  std::vector<int> cur_npc_cars;
  double min_npc_s = 30.0;
  for (const auto &obstacle : obstacle_manager.get_obstacles().Items()) {
    // if (!obstacle->has_sl_polygon_seq()) {
    //   continue;
    // }
    // auto ptr_obstacle_decision =
    // obstacle_decision_manager.find_obstacle_decision(obstacle->Id()); if
    // (ptr_obstacle_decision->HasLateralDecision() &&
    // ptr_obstacle_decision->LateralDecision().has_nudge() &&
    //     ptr_obstacle_decision->LateralDecision().nudge().is_longitunidal_ignored)
    //     {
    //   continue;
    // }
    auto obstacle_sl = obstacle->PerceptionSLBoundary();
    double obs_overlap_dis = std::min(obstacle_sl.end_l, lane_border.first) -
                             std::max(obstacle_sl.start_l, lane_border.second);
    double lateral_space_for_adc =
        std::max(std::max(obstacle_sl.start_l - lane_border.second, 0.0),
                 std::max(lane_border.first - obstacle_sl.end_l, 0.0));
    double adc_overlap_dis = std::min(adc_sl.end_l, lane_border.first) -
                             std::max(adc_sl.start_l, lane_border.second);
    double lateral_space_for_obs =
        std::max(std::max(adc_sl.start_l - lane_border.second, 0.0),
                 std::max(lane_border.first - adc_sl.end_l, 0.0));
    if (obstacle_sl.start_s > adc_sl.end_s &&
        obstacle_sl.start_s - adc_sl.end_s < nearest_front_car_dis) {
      if (lateral_space_for_adc < obstacle_sl.end_l - obstacle_sl.start_l &&
          obs_overlap_dis / (obstacle_sl.end_l - obstacle_sl.start_l + 1.e-3) >
              0.5) {
        nearest_front_car_dis = obstacle_sl.start_s - adc_sl.end_s;
        MSD_LOG(INFO,
                "[POMDP_PLANNER] find_leader_car_in_target_lane id: %d "
                "distance: %f",
                obstacle->Id(), nearest_front_car_dis);
        leader_car = obstacle;
        min_npc_s = std::min(min_npc_s, obstacle_sl.start_s);
      }
    } else if (obstacle_sl.end_s > target_lane_rival_end_s &&
               obs_overlap_dis /
                       (obstacle_sl.end_l - obstacle_sl.start_l + 1.e-3) >
                   0.2) {
      MSD_LOG(INFO,
              "[POMDP_PLANNER] find_npc_car_in_target_lane id: %d distance: %f",
              obstacle->Id(), obstacle_sl.start_s - adc_sl.end_s);
      cur_npc_cars.emplace_back(obstacle->Id());
    }
  }
  for (auto npc_id : cur_npc_cars) {
    auto npc_obs = obstacle_manager.find_obstacle(npc_id);
    if (npc_obs &&
        npc_obs->PerceptionSLBoundary().start_s < min_npc_s - 1.e-2) {
      npc_cars.emplace_back(npc_id);
    }
  }
  return leader_car;
}

const Obstacle *LCMDPPlanner::find_leader_car_in_origin_lane(
    std::shared_ptr<BaseLineInfo> origin_lane) {
  const Obstacle *leader_car{nullptr};
  auto &obstacle_manager = origin_lane->mutable_obstacle_manager();
  auto &obstacle_decision_manager =
      context_->mutable_obstacle_decision_manager();
  auto adc_sl = origin_lane->get_adc_sl_boundary();
  auto &origin_lane_path_data = origin_lane->get_path_data();
  PathPoint adc_point;
  adc_point.x = origin_lane->get_ego_state().ego_pose.x;
  adc_point.y = origin_lane->get_ego_state().ego_pose.y;
  double adc_path_s =
      origin_lane_path_data.discretized_path().QueryMatchedS(adc_point);
  auto lane_border = origin_lane_path_data.getWidth(adc_path_s);
  double nearest_front_car_dis = std::numeric_limits<double>::max();
  for (const auto &obstacle : obstacle_manager.get_obstacles().Items()) {
    // if (!obstacle->has_sl_polygon_seq()) {
    //   continue;
    // }
    // auto ptr_obstacle_decision =
    // obstacle_decision_manager.find_obstacle_decision(obstacle->Id()); if
    // (ptr_obstacle_decision->HasLateralDecision() &&
    // ptr_obstacle_decision->LateralDecision().has_nudge() &&
    //     ptr_obstacle_decision->LateralDecision().nudge().is_longitunidal_ignored)
    //     {
    //   continue;
    // }
    auto obstacle_sl = obstacle->PerceptionSLBoundary();
    if (obstacle_sl.end_s > adc_sl.end_s &&
        obstacle_sl.start_s - adc_sl.end_s < nearest_front_car_dis) {
      double overlap_dis = std::min(obstacle_sl.end_l, lane_border.first) -
                           std::max(obstacle_sl.start_l, lane_border.second);
      if (overlap_dis / (obstacle_sl.end_l - obstacle_sl.start_l + 1.e-3) >
          0.2) {
        nearest_front_car_dis = obstacle_sl.start_s - adc_sl.end_s;
        MSD_LOG(INFO,
                "[POMDP_PLANNER] find_leader_car_in_origin_lane id: %d "
                "distance: %f",
                obstacle->Id(), nearest_front_car_dis);
        leader_car = obstacle;
      }
    }
  }
  return leader_car;
}

const Obstacle *LCMDPPlanner::find_rival_car_in_origin_lane(
    std::shared_ptr<BaseLineInfo> origin_lane) {
  const Obstacle *rear_car{nullptr};
  auto &obstacle_manager = origin_lane->mutable_obstacle_manager();
  auto adc_sl = origin_lane->get_adc_sl_boundary();
  auto &origin_lane_path_data = origin_lane->get_path_data();
  PathPoint adc_point;
  adc_point.x = origin_lane->get_ego_state().ego_pose.x;
  adc_point.y = origin_lane->get_ego_state().ego_pose.y;
  double adc_path_s =
      origin_lane_path_data.discretized_path().QueryMatchedS(adc_point);
  auto lane_border = origin_lane_path_data.getWidth(adc_path_s);
  double nearest_rear_car_dis = std::numeric_limits<double>::lowest();
  for (const auto &obstacle : obstacle_manager.get_obstacles().Items()) {
    // if (!obstacle->has_sl_polygon_seq()) {
    //   continue;
    // }
    auto obstacle_sl = obstacle->PerceptionSLBoundary();
    constexpr double kOvertakeLateralBuffer = 0.7;
    if (obstacle_sl.end_s < adc_sl.end_s &&
        obstacle_sl.end_s - adc_sl.start_s > nearest_rear_car_dis) {
      double overlap_dis = std::min(obstacle_sl.end_l, lane_border.first) -
                           std::max(obstacle_sl.start_l, lane_border.second);
      if (overlap_dis / (obstacle_sl.end_l - obstacle_sl.start_l + 1.e-3) >
          0.2) {
        nearest_rear_car_dis = obstacle_sl.end_s - adc_sl.start_s;
        MSD_LOG(
            INFO,
            "[POMDP_PLANNER] find_rival_car_in_origin_lane id: %d distance: %f",
            obstacle->Id(), nearest_rear_car_dis);
        rear_car = obstacle;
      }
    }
  }
  return rear_car;
}

void LCMDPPlanner::reset(const TaskConfig &config) {
  Task::reset(config);
  despot::Planner::reset();

  npc_ids_.clear();
  related_obstacles_.clear();

  initial_rival_aggressiveness_ =
      config.lane_change_pomdp_planner_config().initial_rival_aggressiveness;
  time_horizon_ = config.lane_change_pomdp_planner_config().time_horizon;
  time_step_ = config.lane_change_pomdp_planner_config().time_step;
}

void LCMDPPlanner::unset() {
  despot::Planner::unset();

  npc_ids_.clear();
  related_obstacles_.clear();
}

TaskStatus LCMDPPlanner::execute(ScenarioFacadeContext *context) {
  MLOG_PROFILING(name_.c_str());
  if (Task::execute(context) != TaskStatus::STATUS_SUCCESS) {
    return TaskStatus::STATUS_FAILED;
  }
  if (world_model_ == nullptr) {
    MSD_LOG(INFO, "world model is none!");
    return TaskStatus::STATUS_FAILED;
  }
  if (!baseline_info_ || !baseline_info_->is_valid()) {
    MSD_LOG(INFO, "baseline info is invalid!");
    return TaskStatus::STATUS_FAILED;
  }
  if (context_->planning_status().planning_success) {
    return TaskStatus::STATUS_SUCCESS_BREAK;
  }
  if (!ConfigurationContext::Instance()
           ->synthetic_config()
           .enable_interactive_lane_change) {
    return TaskStatus::STATUS_SUCCESS;
  }
  if (calculate()) {
    return TaskStatus::STATUS_SUCCESS;
  } else {
    return TaskStatus::STATUS_FAILED;
  }
}

bool LCMDPPlanner::calculate() {
  std::array<double, 5> lat_range_info{1.9, -1.9, 0.2, 1.9, -5.7};
  std::array<double, 3> lon_range_info{0.0, 150.0, 1.0};
  // todo : instantiate model
  const auto &planning_status = context_->planning_status();
  const auto &state_machine_out = context_->state_machine_output();
  auto direction = planning_status.lane_status.change_lane.direction;
  current_lane_id_ = planning_status.lane_status.target_lane_id;
  if (planning_status.lane_status.status == LaneStatus::LANE_CHANGE) {
    if (planning_status.lane_status.change_lane.status ==
        ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION) {
      origin_lane_id_ = planning_status.lane_status.target_lane_id;
      target_lane_id_ = planning_status.lane_status.change_lane.path_id;
      MSD_LOG(INFO, "[POMDP_PLANNER] lc wait state");
      return true;
    } else if (planning_status.lane_status.change_lane.status ==
               ChangeLaneStatus::Status::IN_CHANGE_LANE) {
      target_lane_id_ = planning_status.lane_status.target_lane_id;
      origin_lane_id_ =
          direction == "left" ? current_lane_id_ + 1 : current_lane_id_ - 1;
      MSD_LOG(INFO, "[POMDP_PLANNER] lc change state direction %s",
              direction.c_str());
    } else {
      origin_lane_id_ = planning_status.lane_status.target_lane_id;
      target_lane_id_ = planning_status.lane_status.change_lane.path_id;
      return true;
    }
  } else {
    return true;
  }
  bool is_pre_lc_change =
      PlanningContext::Instance()->planning_status().lane_status.status ==
          LaneStatus::LANE_CHANGE &&
      PlanningContext::Instance()
              ->planning_status()
              .lane_status.change_lane.status ==
          ChangeLaneStatus::Status::IN_CHANGE_LANE;
  bool disable_lc_for_invalid_solution = false;
  if (!is_pre_lc_change &&
      context_->planning_status()
          .lane_status.change_lane.enable_interactive_mode) {
    disable_lc_for_invalid_solution = true;
  }

  dis_to_change_point_ = std::numeric_limits<double>::max();
  const auto &map_info = world_model_->get_map_info();
  lc_wait_time_ = planning_status.lane_status.change_lane.lane_change_wait_time;
  // bool is_in_dash_line = false;
  if (direction == "left") {
    if (planning_status.lane_status.change_lane.path_id == 0) {
      dis_to_change_point_ = map_info.get_distance_to_dash_line("right");
    } else {
      dis_to_change_point_ = map_info.get_distance_to_dash_line("left");
    }
  } else if (context_->planning_status().lane_status.change_lane.direction ==
             "right") {
    if (planning_status.lane_status.change_lane.path_id == 0) {
      dis_to_change_point_ = map_info.get_distance_to_dash_line("left");
    } else {
      dis_to_change_point_ = map_info.get_distance_to_dash_line("right");
    }
  }

  dis_to_change_point_ = std::min(dis_to_change_point_, map_info.lc_end_dis());
  if (dis_to_change_point_ < 0.0) {
    dis_to_change_point_ = map_info.dist_to_intsect();
  }

  v_limit_ = planning_status.v_limit;
  map_v_limit_ = map_info.v_cruise();
  auto origin_baseline = world_model_->get_baseline_info(origin_lane_id_);
  auto target_baseline = world_model_->get_baseline_info(target_lane_id_);
  auto curr_baseline = world_model_->get_baseline_info(current_lane_id_);
  if (origin_baseline == nullptr || !origin_baseline->is_valid()) {
    MSD_LOG(INFO, "[POMDP_PLANNER] no origin lane %d", origin_lane_id_);
    return true;
  }
  if (target_baseline == nullptr || !target_baseline->is_valid()) {
    MSD_LOG(INFO, "[POMDP_PLANNER] no target lane %d", target_lane_id_);
    return true;
  }

  auto target_lane_adc_sl = target_baseline->get_adc_sl_boundary();
  auto &target_lane_path_data = target_baseline->get_path_data();
  PathPoint adc_point;
  adc_point.x = target_baseline->get_ego_state().ego_pose.x;
  adc_point.y = target_baseline->get_ego_state().ego_pose.y;
  double target_lane_adc_path_s =
      target_lane_path_data.discretized_path().QueryMatchedS(adc_point);
  auto target_lane_border =
      target_lane_path_data.getWidth(target_lane_adc_path_s);
  double target_lane_width =
      target_lane_border.first - target_lane_border.second;

  auto origin_lane_adc_sl = origin_baseline->get_adc_sl_boundary();
  auto &origin_lane_path_data = origin_baseline->get_path_data();
  adc_point.x = origin_baseline->get_ego_state().ego_pose.x;
  adc_point.y = origin_baseline->get_ego_state().ego_pose.y;
  double origin_lane_adc_path_s =
      origin_lane_path_data.discretized_path().QueryMatchedS(adc_point);
  auto origin_lane_border =
      origin_lane_path_data.getWidth(origin_lane_adc_path_s);
  double origin_lane_width =
      origin_lane_border.first - origin_lane_border.second;
  if (direction == "left") {
    lat_range_info[0] = target_lane_width + origin_lane_width / 2.0;
    lat_range_info[1] = origin_lane_width / 2.0;
    lat_range_info[3] = target_lane_width + origin_lane_width / 2.0;
    lat_range_info[4] = -origin_lane_width / 2.0;
    if (curr_baseline == target_baseline) {
      lat_range_info[0] -= (target_lane_width / 2.0 + origin_lane_width / 2.0);
      lat_range_info[1] -= (target_lane_width / 2.0 + origin_lane_width / 2.0);
      lat_range_info[3] -= (target_lane_width / 2.0 + origin_lane_width / 2.0);
      lat_range_info[4] -= (target_lane_width / 2.0 + origin_lane_width / 2.0);
    }
  } else {
    lat_range_info[0] = -origin_lane_width / 2.0;
    lat_range_info[1] = -origin_lane_width / 2.0 - target_lane_width;
    lat_range_info[3] = origin_lane_width / 2.0;
    lat_range_info[4] = -origin_lane_width / 2.0 - target_lane_width;
    if (curr_baseline == target_baseline) {
      lat_range_info[0] += (target_lane_width / 2.0 + origin_lane_width / 2.0);
      lat_range_info[1] += (target_lane_width / 2.0 + origin_lane_width / 2.0);
      lat_range_info[3] += (target_lane_width / 2.0 + origin_lane_width / 2.0);
      lat_range_info[4] += (target_lane_width / 2.0 + origin_lane_width / 2.0);
    }
  }
  MSD_LOG(
      INFO,
      "[POMDP_PLANNER] lat range [%f %f %f %f %f], lon_range_info [%f %f %f]",
      lat_range_info[0], lat_range_info[1], lat_range_info[2],
      lat_range_info[3], lat_range_info[4], lon_range_info[0],
      lon_range_info[1], lon_range_info[2]);
  lat_range_info_ = lat_range_info;
  lon_range_info_ = lon_range_info;
  // InitializeDefaultParameters();
  // auto model = new LCEnv(lat_range_info, lon_range_info);

  string solver_type = ChooseSolver();
  bool search_solver;
  int num_runs = 1;
  string world_type = "pomdp";
  string belief_type = "DEFAULT";
  int time_limit = -1;
  int argc{0};
  char *argv[0];

  option::Option *options =
      InitializeParamers(argc, argv, solver_type, search_solver, num_runs,
                         world_type, belief_type, time_limit);

  auto model = static_cast<LCEnv *>(InitializeModel(options));
  // MSD_LOG(INFO, "[POMDP_PLANNER] debug model delta_t %f agg %f zero %d",
  // model->delta_t_, model->rival_aggressiveness_,
  // model->translate_ego_lat_vel(0.0));

  // todo: feed ego info & rival info & map info & npc_info
  auto &obstacle_manager = curr_baseline->mutable_obstacle_manager();
  model->FeedEgoInfo(curr_baseline);
  auto rival_obstacle = find_rival_car_in_target_lane(target_baseline);
  if (rival_obstacle && rival_obstacle->Trajectory().size() > 1) {
    if (std::find(related_obstacles_.begin(), related_obstacles_.end(),
                  rival_obstacle->Id()) == related_obstacles_.end()) {
      related_obstacles_.emplace_back(rival_obstacle->Id());
    }
    auto current_lane_rival_obstacle =
        obstacle_manager.find_obstacle(rival_obstacle->Id());
    if (current_lane_rival_obstacle &&
        !current_lane_rival_obstacle->IsFrenetInvalid()) {
      model->FeedRivalInfo(current_lane_rival_obstacle);
      MSD_LOG(INFO,
              "[POMDP_PLANNER] find target lane rival obstacle %d lane id %d "
              "polygon %d",
              current_lane_rival_obstacle->Id(), curr_baseline->lane_id(),
              current_lane_rival_obstacle->has_sl_polygon_seq());
      if (!current_lane_rival_obstacle->has_sl_polygon_seq() &&
          !current_lane_rival_obstacle->is_sl_polygon_seq_invalid()) {
        std::string fail_reason{};
        StGraphGenerator::compute_obs_sl_polygon_seq(
            current_lane_rival_obstacle, curr_baseline, {0.0, 8.0}, 0.2, true,
            fail_reason);
      }
      if (!current_lane_rival_obstacle->has_sl_polygon_seq()) {
        MSD_LOG(INFO,
                "[POMDP_PLANNER] rival obstacle %d sl_polygon_seq constructed "
                "failed",
                current_lane_rival_obstacle->Id());
        return true;
      }
    } else {
      MSD_LOG(INFO, "[POMDP_PLANNER] invalid target lane rival obstacle");
      context_->mutable_planning_status()
          ->lane_status.change_lane.enable_lane_change_traj_checker = false;
      return true;
    }
  } else {
    MSD_LOG(INFO, "[POMDP_PLANNER] no target lane rival obstacle");
    context_->mutable_planning_status()
        ->lane_status.change_lane.enable_lane_change_traj_checker = false;
    return true;
  }
  target_lane_rival_id_ = rival_obstacle->Id();

  // evaluate whether last pomdp planner output is reusable
  constexpr int kMaxReuseCounter = 1;
  if (PlanningContext::Instance()
          ->mutable_pomdp_planner_output()
          .rival_obstacle_id == rival_obstacle->Id()) {
    if (PlanningContext::Instance()
            ->mutable_pomdp_planner_output()
            .prediction_revised &&
        PlanningContext::Instance()
                ->mutable_pomdp_planner_output()
                .last_result_time_counter < kMaxReuseCounter) {
      PredictionReviser prediction_reviser(world_model_, baseline_info_,
                                           context_);
      auto current_lane_rival_obstacle =
          obstacle_manager.find_obstacle(rival_obstacle->Id());
      const auto &obs_traj = rival_obstacle->Trajectory();
      double origin_average_rival_vel =
          (obs_traj.back().path_point.s - obs_traj.front().path_point.s) /
          (obs_traj.back().relative_time - obs_traj.front().relative_time);
      const auto &rival_speed_data = PlanningContext::Instance()
                                         ->mutable_pomdp_planner_output()
                                         .rival_speed_data;
      double new_average_rival_vel =
          (rival_speed_data.back().s - rival_speed_data.front().s) /
          (1.e-2 + rival_speed_data.back().t - rival_speed_data.front().t);
      if (new_average_rival_vel <= origin_average_rival_vel - 0.2 &&
          new_average_rival_vel / (1.e-2 + origin_average_rival_vel) < 0.95) {
        prediction_reviser.revise_prediction_trajectory(
            current_lane_rival_obstacle, PlanningContext::Instance()
                                             ->mutable_pomdp_planner_output()
                                             .rival_speed_data);
      }
      context_->mutable_pomdp_planner_output() =
          PlanningContext::Instance()->pomdp_planner_output();
      context_->mutable_pomdp_planner_output().last_result_time_counter++;
      MSD_LOG(INFO, "[POMDP_PLANNER] reuse last loop revision");
      return true;
    } else {
      context_->mutable_pomdp_planner_output().prediction_revised = false;
      context_->mutable_pomdp_planner_output().last_result_time_counter = 0;
      context_->mutable_pomdp_planner_output().valid_lane_change_solution =
          false;
      context_->mutable_planning_status()
          ->lane_status.change_lane.enable_lane_change_traj_checker = false;
    }
  } else {
    context_->mutable_pomdp_planner_output().prediction_revised = false;
    context_->mutable_pomdp_planner_output().last_result_time_counter = 0;
    context_->mutable_pomdp_planner_output().valid_lane_change_solution = false;
    context_->mutable_planning_status()
        ->lane_status.change_lane.enable_lane_change_traj_checker = false;
  }
  context_->mutable_pomdp_planner_output().ego_speed_data.clear();
  context_->mutable_pomdp_planner_output().rival_speed_data.clear();
  context_->mutable_pomdp_planner_output().rival_traj.clear();
  context_->mutable_pomdp_planner_output().ego_traj.clear();
  // auto debug_obs = obstacle_manager.find_obstacle(rival_obstacle->Id());
  // MSD_LOG(INFO, "[POMDP_PLANNER] debug find target lane rival obstacle %d
  // lane id %d", current_lane_rival_obstacle->Id(), curr_baseline->lane_id());
  // model->FeedNPCPrediction(find_npc_car(origin_baseline, target_baseline,
  // curr_baseline));
  set_npc_car(origin_baseline, target_baseline, curr_baseline, model);
  model->FeedMapInfo(map_v_limit_, origin_baseline, target_baseline);

  clock_t main_clock_start = clock();
  // World *world = InitializeWorld(world_type, model, options);
  // mph_assert(world != NULL);
  auto current_state = model->CreateStartState();
  Belief *belief = model->InitialBelief(current_state, belief_type);
  mph_assert(belief != NULL);

  ScenarioLowerBound *lower_bound =
      model->CreateScenarioLowerBound("DEFAULT", "DEFAULT");
  ScenarioUpperBound *upper_bound =
      model->CreateScenarioUpperBound("DEFAULT", "DEFAULT");

  // Solver *solver = new DESPOT(model, lower_bound, upper_bound);
  Solver *solver = InitializeSolver(model, belief, solver_type, options);

  // Logger *logger = NULL;
  //   InitializeLogger(logger, options, model, belief, solver, num_runs,
  // 		  main_clock_start, world, world_type, time_limit, solver_type);

  // Reset belief and solver
  double start_t = get_time_second();
  // delete solver->belief();
  double end_t = get_time_second();

  solver->belief(belief);
  // logger->belief(belief);
  // DisplayParameters(options, model);

  start_t = get_time_second();
  ACT_TYPE action = solver->Search().action;
  end_t = get_time_second();
  double search_time = (end_t - start_t);
  MSD_LOG(INFO, "[POMDP_PLANNER] search time %f", search_time);

  // todo: execute action to get next state & send to rival implict state
  // estimation
  double reward;
  OBS_TYPE obs;
  History future_traj = solver->future();
  if (future_traj.Size() < int(time_horizon_ / time_step_) - 1) {
    MSD_LOG(INFO, "[POMDP_PLANNER] future traj size %d", future_traj.Size());
    // return false;
  }
  SpeedPoint ego_sp, rival_sp;
  auto &lc_state = static_cast<const LCAgentState &>(*current_state);
  ego_sp.s = lc_state.ego_pos.x;
  ego_sp.v = lc_state.ego_lon_vel;
  ego_sp.a = lc_state.ego_lon_acc;
  ego_sp.t = lc_state.time;
  rival_sp.s = lc_state.rival_pos.x;
  rival_sp.v = lc_state.rival_lon_vel;
  rival_sp.a = lc_state.rival_lon_acc;
  rival_sp.t = lc_state.time;
  // MSD_LOG(INFO, "[POMDP_PLANNER] future traj size %d", future_traj.Size());
  SpeedData ego_speed_data, rival_speed_data;
  std::vector<std::pair<double, FrenetState>> ego_traj, rival_traj;
  std::pair<double, FrenetState> ego_traj_point, rival_traj_point;
  ego_traj_point.first = lc_state.time;
  ego_traj_point.second.s = lc_state.ego_pos.x;
  ego_traj_point.second.ds = lc_state.ego_lon_vel;
  ego_traj_point.second.dds = lc_state.ego_lon_acc;
  ego_traj_point.second.r = lc_state.ego_pos.y;
  ego_traj_point.second.dr = lc_state.ego_lat_vel;
  ego_traj_point.second.ddr = 0.0;
  rival_traj_point.first = lc_state.time;
  rival_traj_point.second.s = lc_state.rival_pos.x;
  rival_traj_point.second.ds = lc_state.rival_lon_vel;
  rival_traj_point.second.dds = lc_state.rival_lon_acc;
  rival_traj_point.second.r = lc_state.rival_pos.y;
  rival_traj_point.second.dr = 0.0;
  rival_traj_point.second.ddr = 0.0;

  ego_speed_data.push_back(ego_sp);
  rival_speed_data.push_back(rival_sp);
  ego_traj.push_back(ego_traj_point);
  rival_traj.push_back(rival_traj_point);
  bool is_ego_in_target_lane{false};
  bool is_ego_become_rival_leader{false};
  const auto &obs_traj = rival_obstacle->Trajectory();
  double origin_average_rival_vel =
      (obs_traj.back().path_point.s - obs_traj.front().path_point.s) /
      (obs_traj.back().relative_time - obs_traj.front().relative_time);
  for (int i = 0; i < future_traj.Size(); i++) {
    auto cur_action = future_traj.Action(i);
    // model->PrintAction(cur_action);
    auto cur_agent_action = model->translate_action(cur_action);
    model->Step(*current_state, 0, cur_action, reward, obs);
    ego_sp.s = lc_state.ego_pos.x;
    ego_sp.v = lc_state.ego_lon_vel;
    ego_sp.a = lc_state.ego_lon_acc;
    ego_sp.t = lc_state.time;
    rival_sp.s = lc_state.rival_pos.x;
    rival_sp.v = lc_state.rival_lon_vel;
    rival_sp.a = lc_state.rival_lon_acc;
    rival_sp.t = lc_state.time;
    ego_speed_data.push_back(ego_sp);
    rival_speed_data.push_back(rival_sp);
    ego_traj_point.first = lc_state.time;
    ego_traj_point.second.s = lc_state.ego_pos.x;
    ego_traj_point.second.ds = lc_state.ego_lon_vel;
    ego_traj_point.second.dds = lc_state.ego_lon_acc;
    ego_traj_point.second.r = lc_state.ego_pos.y;
    ego_traj_point.second.dr = lc_state.ego_lat_vel;
    ego_traj_point.second.ddr = 0.0;
    rival_traj_point.first = lc_state.time;
    rival_traj_point.second.s = lc_state.rival_pos.x;
    rival_traj_point.second.ds = lc_state.rival_lon_vel;
    rival_traj_point.second.dds = lc_state.rival_lon_acc;
    rival_traj_point.second.r = lc_state.rival_pos.y;
    rival_traj_point.second.dr = 0.0;
    rival_traj_point.second.ddr = 0.0;
    ego_traj.push_back(ego_traj_point);
    rival_traj.push_back(rival_traj_point);
    // judge if ego in target lane
    double ego_overlap_dis =
        std::min(lc_state.ego_pos.y + model->ego_width_ / 2.0,
                 lat_range_info[0]) -
        std::max(lc_state.ego_pos.y - model->ego_width_ / 2.0,
                 lat_range_info[1]);
    if (ego_overlap_dis / model->ego_width_ > 0.7) {
      is_ego_in_target_lane = true;
    } else {
      is_ego_in_target_lane = false;
    }
    double lateral_space =
        std::max(std::max(lc_state.ego_pos.y - model->ego_width_ / 2.0 -
                              lat_range_info[1],
                          0.0),
                 std::max(lat_range_info[0] - lc_state.ego_pos.y +
                              model->ego_width_ / 2.0,
                          0.0));
    if (model->rival_width_ < lateral_space + 0.1 &&
        lc_state.ego_pos.x - model->ego_length_ / 2.0 >
            lc_state.rival_pos.x + model->rival_length_ / 2.0) {
      is_ego_become_rival_leader = true;
    } else {
      is_ego_become_rival_leader = false;
    }
    // judge if ego right in front of rival

    MSD_LOG(INFO,
            "[POMDP_PLANNER] time %f ego pos [%f %f] lon acc %f lon_vel %f ego "
            "lat vel %f",
            lc_state.time, lc_state.ego_pos.x, lc_state.ego_pos.y,
            lc_state.ego_lon_acc, lc_state.ego_lon_vel, lc_state.ego_lat_vel);
    MSD_LOG(INFO,
            "[POMDP_PLANNER] rival pos [%f %f] lon acc %f lon_vel %f rival lon "
            "acc %f",
            lc_state.rival_pos.x, lc_state.rival_pos.y, lc_state.rival_lon_acc,
            lc_state.rival_lon_vel, lc_state.rival_lon_acc);
    MSD_LOG(INFO,
            "[POMDP_PLANNER] ego lateral space %f is_ego_in_target_lane %d "
            "is_ego_become_rival_leader %d",
            lateral_space, is_ego_in_target_lane, is_ego_become_rival_leader);
  }

  // evaluate the output
  // judge if ego car invade both target lane and rival obstacle lateral space
  bool valid_lane_change_solution{true};
  bool is_current_in_wait_stage{false};
  const auto &last_planning_status =
      PlanningContext::Instance()->planning_status();
  if (last_planning_status.lane_status.status == LaneStatus::LANE_KEEP ||
      (last_planning_status.lane_status.status == LaneStatus::LANE_CHANGE &&
       last_planning_status.lane_status.change_lane.status ==
           ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION)) {
    context_->mutable_planning_status()
        ->lane_status.change_lane.enable_lane_change_traj_checker = true;
    is_current_in_wait_stage = true;
  } else {
    context_->mutable_planning_status()
        ->lane_status.change_lane.enable_lane_change_traj_checker = false;
  }
  if (is_ego_become_rival_leader && is_ego_in_target_lane) {
    MSD_LOG(INFO, "[POMDP_PLANNER] has a chance to get into target lane");
  } else {
    MSD_LOG(INFO,
            "[POMDP_PLANNER] it's better to keep waiting "
            "is_ego_become_rival_leader %d is_ego_in_target_lane %d",
            is_ego_become_rival_leader, is_ego_in_target_lane);
    if (is_current_in_wait_stage) {
      return false;
    }
    if (disable_lc_for_invalid_solution) {
      return false;
    }
    valid_lane_change_solution = false;
    // return false;
  }
  // evaluate the output
  // judge if rival obstacle prediction should be revised
  double new_average_rival_vel =
      (rival_speed_data.back().s - rival_speed_data.front().s) /
      (1.e-2 + rival_speed_data.back().t - rival_speed_data.front().t);
  MSD_LOG(INFO, "[POMDP_PLANNER] rival average speed change from %f to %f",
          origin_average_rival_vel, new_average_rival_vel);
  if (!valid_lane_change_solution ||
      (new_average_rival_vel >= origin_average_rival_vel - 0.2 ||
       new_average_rival_vel / (1.e-2 + origin_average_rival_vel) > 0.95)) {
    MSD_LOG(INFO, "[POMDP_PLANNER] no need to revise rival prediction");
    if (!valid_lane_change_solution) {
      // todo: add lc puase decision or add checkmode
      context_->mutable_planning_status()
          ->lane_status.change_lane.enable_lane_change_traj_checker = true;
    }
  } else {
    // revise rival prediction
    PredictionReviser prediction_reviser(world_model_, baseline_info_,
                                         context_);
    auto current_lane_rival_obstacle =
        obstacle_manager.find_obstacle(rival_obstacle->Id());
    prediction_reviser.revise_prediction_trajectory(current_lane_rival_obstacle,
                                                    rival_speed_data);
    context_->mutable_pomdp_planner_output().prediction_revised = true;
    context_->mutable_pomdp_planner_output().last_result_time_counter = 0;
  }

  // output result to context
  context_->mutable_pomdp_planner_output().valid_lane_change_solution =
      valid_lane_change_solution;
  context_->mutable_pomdp_planner_output().ego_speed_data = ego_speed_data;
  context_->mutable_pomdp_planner_output().rival_obstacle_id =
      rival_obstacle->Id();
  context_->mutable_pomdp_planner_output().rival_speed_data = rival_speed_data;

  // revise rival prediction
  // PredictionReviser preidction_reviser(world_model_, baseline_info_,
  // context_); auto current_lane_rival_obstacle =
  // obstacle_manager.find_obstacle(rival_obstacle->Id());
  // preidction_reviser.revise_prediction_trajectory(current_lane_rival_obstacle,
  // rival_speed_data);

  // model->Step(*current_state, 0, action, reward, obs);
  model->Free(current_state);

  // int argc{0};
  // char* argv[0];
  // RunEvaluation(argc, argv);

  // todo: post process for ego & rival traj & action

  return true;
}

} // namespace msquare
