#include "planner/scenarios/scenario_facade.h"
#include "planner/tasks/task_factory.h"

namespace msquare {

ScenarioFacade::ScenarioFacade(const ScenarioFacadeConfig &config)
    : config_(config) {
  // name_ =
  //     ScenarioFacadeNames[static_cast<int>(config.scenario_facade_type_) -
  //     1];
  // std::unordered_map<TaskConfig::TaskType, const TaskConfig *,
  // std::hash<int>>
  //     config_map;
  // for (const auto &task_config : config_.tasks_config_vector_) {
  //   config_map[task_config.task_type()] = &task_config;
  // }
  // for (int i = 0; i < config_.tasks_config_vector_.size(); ++i) {
  //   auto task_type = config_.tasks_config_vector_[i].task_type();
  //   mph_assert(config_map.find(task_type) != config_map.end());
  //   auto iter = tasks_.find(task_type);
  //   // same task executed more than once uses single instance
  //   if (iter == tasks_.end()) {
  //     auto ptr = TaskFactory::CreateTask(*config_map[task_type]);
  //     task_list_.push_back(ptr.get());
  //     tasks_[task_type] = std::move(ptr);
  //   } else {
  //     task_list_.push_back(iter->second.get());
  //   }
  // }
}

Task *ScenarioFacade::FindTask(TaskConfig::TaskType task_type) const {
  auto iter = tasks_.find(task_type);
  if (iter == tasks_.end()) {
    return nullptr;
  } else {
    return iter->second.get();
  }
}

void ScenarioFacade::init() {
  scenario_facade_context_ = std::make_shared<ScenarioFacadeContext>();
  return;
}

bool ScenarioFacade::process(std::shared_ptr<WorldModel> world_model) {
  MLOG_PROFILING("ScenarioFacade");
  mph_assert(nullptr != world_model);
  world_model_ = world_model;

  for (auto task_iter = tasks_.begin(); task_iter != tasks_.end();
       task_iter++) {
    // MSD_LOG(INFO, "init task[%s]", task_iter->second->name().c_str());
    task_iter->second->init(world_model_);
  }
  double start = MTIME()->timestamp().ms();
  auto *planner_debug = scenario_facade_context_->mutable_planner_debug();

  for (auto *task : task_list_) {
    // run one time planner to update vars when CH2 acc mode
    if (!world_model_->get_vehicle_dbw_status() || world_model->is_acc_mode()) {
      if (std::strcmp(task->name().c_str(), "LateralMotionPlanner") == 0) {
        MSD_LOG(ERROR, "[lat_reset] task lat_reset");
        task->lat_reset();
      }
    }
    if (world_model->is_acc_mode() && world_model->last_is_acc_mode()) {
      if (std::strcmp(task->name().c_str(), "LateralBehaviorPlanner") == 0 ||
          std::strcmp(task->name().c_str(), "LateralMotionPlanner") == 0) {
        scenario_facade_context_->mutable_lateral_motion_planner_output()
            .clear();
        scenario_facade_context_->mutable_lateral_behavior_planner_output()
            .clear();
        *scenario_facade_context_->mutable_path_planner_input() = {};
        if (std::strcmp(task->name().c_str(), "LateralMotionPlanner") == 0) {
          task->lat_reset();
        }
        continue;
      }
    }

    MSD_LOG(INFO, "DEBUG_CJ6 task_name:%s", task->name().c_str());

    double start1 = MTIME()->timestamp().ms();
    auto ret = task->execute(scenario_facade_context_.get());
    if (ret == TaskStatus::STATUS_FAILED) {
      MSD_LOG(INFO, "failed to run task[%s] ", task->name().c_str());
      double end1 = MTIME()->timestamp().ms();
      double time_cost1 = end1 - start1;
      MSD_LOG(ERROR, "time_cost, scenario tasks[%s] cost time: %f",
              task->name().c_str(), time_cost1);
      CostTime task_cost = CostTime{task->name(), time_cost1};
      planner_debug->cost_time.emplace_back(task_cost);
      return false;
    } else if (ret == TaskStatus::STATUS_SUCCESS_BREAK) {
      double end1 = MTIME()->timestamp().ms();
      double time_cost1 = end1 - start1;
      MSD_LOG(ERROR, "time_cost, scenario tasks[%s] cost time: %f",
              task->name().c_str(), time_cost1);
      CostTime task_cost = CostTime{task->name(), time_cost1};
      planner_debug->cost_time.emplace_back(task_cost);
      break;
    }
    double end1 = MTIME()->timestamp().ms();
    double time_cost1 = end1 - start1;
    MSD_LOG(ERROR, "time_cost, scenario tasks[%s] cost time: %f",
            task->name().c_str(), time_cost1);
    CostTime task_cost = CostTime{task->name(), time_cost1};
    planner_debug->cost_time.emplace_back(task_cost);
    if (world_model->is_acc_mode() == true) {
      if (std::strcmp(task->name().c_str(), "LongitudinalMotionPlanner") == 0) {
        break;
      }
    }
  }
  double end = MTIME()->timestamp().ms();
  double time_cost = end - start;
  MSD_LOG(ERROR, "time_cost, all scenario tasks cost time: %f", time_cost);

  CostTime tasks_cost = CostTime{"tasks", time_cost};
  planner_debug->cost_time.emplace_back(tasks_cost);

  post_process();
  return true;
}

void ScenarioFacade::reset(const ScenarioFacadeConfig &config) {
  scenario_facade_context_ = nullptr;
  task_list_.clear();
  tasks_.clear();
  world_model_ = nullptr;
  baseline_info_ = nullptr;

  config_ = config;
  name_ =
      ScenarioFacadeNames[static_cast<int>(config.scenario_facade_type_) - 1];
  std::unordered_map<TaskConfig::TaskType, const TaskConfig *, std::hash<int>>
      config_map;
  for (const auto &task_config : config_.tasks_config_vector_) {
    config_map[task_config.task_type()] = &task_config;
  }
  for (int i = 0; i < config_.tasks_config_vector_.size(); ++i) {
    auto task_type = config_.tasks_config_vector_[i].task_type();
    mph_assert(config_map.find(task_type) != config_map.end());
    auto iter = tasks_.find(task_type);
    // same task executed more than once uses single instance
    if (iter == tasks_.end()) {
      auto ptr = TaskFactory::CreateTask(*config_map[task_type]);
      task_list_.push_back(ptr.get());
      tasks_[task_type] = ptr;
    } else {
      task_list_.push_back(iter->second.get());
    }
  }
}

void ScenarioFacade::unset() {
  scenario_facade_context_ = nullptr;
  task_list_.clear();
  tasks_.clear();
  world_model_ = nullptr;
  baseline_info_ = nullptr;
}

void ScenarioFacade::post_process() {
  const auto &lane_status =
      scenario_facade_context_->planning_status().lane_status;
  if (lane_status.status != LaneStatus::LANE_CHANGE) {
    scenario_facade_context_->mutable_planning_status()
        ->lane_status.change_lane.target_gap_obs = {-10, -10};
  }
  auto &planning_status = scenario_facade_context_->planning_status();
  int target_lane_id = planning_status.lane_status.target_lane_id;
  baseline_info_ = world_model_->get_baseline_info(target_lane_id);
  if (baseline_info_ == nullptr || !baseline_info_->is_valid()) {
    MSD_LOG(ERROR, "scenario invalid target lane[%d]!", target_lane_id);
    return;
  }

  if (world_model_->is_acc_mode()) {
    auto &planning_result =
        scenario_facade_context_->mutable_planning_status()->planning_result;
    auto frenet_coord = baseline_info_->get_frenet_coord();
    planning_result.traj_pose_array.clear();
    planning_result.curv_rate.clear();

    for (const auto &vel_point : planning_result.traj_vel_array) {
      maf_planning::PathPoint pose{};
      FrenetState frenet_point{};
      CartesianState cart_point{};
      frenet_point.s =
          vel_point.distance +
          baseline_info_->get_ego_state().planning_init_point.path_point.s;
      // vel_point.distance + baseline_info_->get_ego_state().ego_frenet.x;
      if (frenet_coord->FrenetState2CartState(frenet_point, cart_point) ==
          TRANSFORM_FAILED) {
        break;
      }

      pose.position_enu.x = cart_point.x;
      pose.position_enu.y = cart_point.y;
      pose.heading_yaw = cart_point.yaw;
      pose.curvature = cart_point.curvature;
      planning_result.traj_pose_array.emplace_back(pose);
      planning_result.curv_rate.push_back(0.0);
    }
    scenario_facade_context_->mutable_planning_status()->planning_success =
        true;
  }

  auto stitched_trajectory = TrajectoryStitcher::TransformToPublishedTraj(
      baseline_info_->get_ego_state().stitch_trajectory);
  MSD_LOG(INFO, "stitched_trajectory size is %d",
          stitched_trajectory.traj_pose_array.size());
  if (stitched_trajectory.traj_pose_array.size() > 1 &&
      planning_status.use_stitching_trajectory) {
    MSD_LOG(INFO,
            "stitched_trajectory size is %d, use_stitching_trajectory is %d",
            stitched_trajectory.traj_pose_array.size(),
            planning_status.use_stitching_trajectory);
    auto &planning_result =
        scenario_facade_context_->mutable_planning_status()->planning_result;
    planning_result.traj_pose_array.insert(
        planning_result.traj_pose_array.begin(),
        stitched_trajectory.traj_pose_array.begin(),
        stitched_trajectory.traj_pose_array.end() - 1);
    planning_result.traj_vel_array.insert(
        planning_result.traj_vel_array.begin(),
        stitched_trajectory.traj_vel_array.begin(),
        stitched_trajectory.traj_vel_array.end() - 1);
    planning_result.traj_acceleration.insert(
        planning_result.traj_acceleration.begin(),
        stitched_trajectory.traj_acceleration.begin(),
        stitched_trajectory.traj_acceleration.end() - 1);
  }
  // debug for follow obs
  const auto &obstacle_decision_manager =
      scenario_facade_context_->obstacle_decision_manager();
  for (auto obstacle :
       baseline_info_->mutable_obstacle_manager().get_obstacles().Items()) {
    auto ptr_obstacle_decision =
        obstacle_decision_manager.find_obstacle_decision(obstacle->Id());
    if (ptr_obstacle_decision->HasLongitudinalDecision()) {
      auto lon_decision = ptr_obstacle_decision->LongitudinalDecision();
      if (lon_decision.has_follow() || lon_decision.has_stop() ||
          lon_decision.has_yield()) {
        MSD_LOG(INFO, "arbitrator scenario[%s] follow[%d]", name_.c_str(),
                obstacle->Id());
      } else if (lon_decision.has_overtake()) {
        MSD_LOG(INFO, "arbitrator scenario[%s] overtake[%d]", name_.c_str(),
                obstacle->Id());
      }
    }
    if (ptr_obstacle_decision->HasLateralDecision()) {
      auto lat_decision = ptr_obstacle_decision->LateralDecision();
      if (lat_decision.has_nudge()) {
        MSD_LOG(INFO, "arbitrator scenario[%s] nudge[%d]", name_.c_str(),
                obstacle->Id());
      }
    }
  }
}

void ScenarioFacade::output_planning_context() {
  *PlanningContext::Instance()->mutable_prev_planning_status() =
      PlanningContext::Instance()->planning_status();
  PlanningContext::Instance()->mutable_lateral_behavior_planner_output() =
      scenario_facade_context_->lateral_behavior_planner_output();
  PlanningContext::Instance()->mutable_lateral_motion_planner_output() =
      scenario_facade_context_->lateral_motion_planner_output();
  *PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output() =
      scenario_facade_context_->longitudinal_behavior_planner_output();
  *PlanningContext::Instance()->mutable_longitudinal_motion_planner_output() =
      scenario_facade_context_->longitudinal_motion_planner_output();
  *PlanningContext::Instance()->mutable_speed_limit() =
      scenario_facade_context_->speed_limit();
  *PlanningContext::Instance()->mutable_planning_status() =
      scenario_facade_context_->planning_status();
  PlanningContext::Instance()->mutable_obstacle_decision_manager() =
      scenario_facade_context_->obstacle_decision_manager();
  // PlanningContext::Instance()->mutable_state_machine_output() =
  //     scenario_facade_context_->state_machine_output();
  *PlanningContext::Instance()->mutable_path_planner_input() =
      scenario_facade_context_->path_planner_input();
  *PlanningContext::Instance()->mutable_planner_debug() =
      scenario_facade_context_->planner_debug();
  *PlanningContext::Instance()->mutable_speed_planner_input() =
      scenario_facade_context_->speed_planner_input();
  *PlanningContext::Instance()->mutable_speed_planner_output() =
      scenario_facade_context_->speed_planner_output();
  *PlanningContext::Instance()->mutable_lon_decison_output() =
      scenario_facade_context_->lon_decison_output();
}

void ScenarioFacade::output_planning_context(
    const ScenarioFacadeContext &context) {
  *PlanningContext::Instance()->mutable_prev_planning_status() =
      PlanningContext::Instance()->planning_status();
  PlanningContext::Instance()->mutable_lateral_behavior_planner_output() =
      context.lateral_behavior_planner_output();
  PlanningContext::Instance()->mutable_lateral_motion_planner_output() =
      context.lateral_motion_planner_output();
  *PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output() =
      context.longitudinal_behavior_planner_output();
  *PlanningContext::Instance()->mutable_longitudinal_motion_planner_output() =
      context.longitudinal_motion_planner_output();
  *PlanningContext::Instance()->mutable_speed_limit() = context.speed_limit();
  *PlanningContext::Instance()->mutable_planning_status() =
      context.planning_status();
  PlanningContext::Instance()->mutable_obstacle_decision_manager() =
      context.obstacle_decision_manager();
  // PlanningContext::Instance()->mutable_state_machine_output() =
  //     context.state_machine_output();
  *PlanningContext::Instance()->mutable_path_planner_input() =
      context.path_planner_input();
  *PlanningContext::Instance()->mutable_planner_debug() =
      context.planner_debug();
  *PlanningContext::Instance()->mutable_speed_planner_input() =
      context.speed_planner_input();
  *PlanningContext::Instance()->mutable_speed_planner_output() =
      context.speed_planner_output();
  *PlanningContext::Instance()->mutable_lon_decison_output() =
      context.lon_decison_output();
}

} // namespace msquare
