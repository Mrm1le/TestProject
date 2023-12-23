#include "planner/behavior_planner/lateral_behavior_planner.h"
#include <string>
#include <utility>

namespace msquare {

LateralBehaviorPlanner::LateralBehaviorPlanner(const TaskConfig &config)
    : Task(config) {
  vel_sequence_.push_back(0.0);
  // mph_assert(config.has_lateral_behavior_planner_config());
}

LateralBehaviorPlanner::~LateralBehaviorPlanner() {}

void LateralBehaviorPlanner::init(std::shared_ptr<WorldModel> world_model) {
  Task::init(world_model);
}

void LateralBehaviorPlanner::reset(const TaskConfig &config) {
  Task::reset(config);

  no_sp_car_ = true;

  is_ncar_ = false;
  t_avd_car_ = 3.0;
  t_avd_sp_car_ = 3.0;
  final_y_rel_ = 10;

  ncar_change_ = 0;
  flag_avd_ = 0;
  avd_back_cnt_ = 0;
  avd_leadone_ = 0;
  pre_leadone_id_ = 0;

  ignore_track_id_ = -10000;

  vel_sequence_.clear();
  avd_info_.clear();
  avd_obstacles.clear();
  avd_obstacle_prior_.clear();

  ignore_change_false_.clear();
  ignore_change_true_.clear();

  s_v_limit_.clear();
  s_a_limit_.clear();
  s_r_offset_.clear();

  for (auto &element : avd_car_past_) {
    element.clear();
  }
  for (auto &element : avd_sp_car_past_) {
    element.clear();
  }

  virtual_lane_mgr_ = nullptr;
  path_planner_ = nullptr;

  vel_sequence_.push_back(0.0);
}

void LateralBehaviorPlanner::unset() {
  vel_sequence_.clear();
  avd_info_.clear();
  avd_obstacles.clear();
  avd_obstacle_prior_.clear();

  ignore_change_false_.clear();
  ignore_change_true_.clear();

  s_v_limit_.clear();
  s_a_limit_.clear();
  s_r_offset_.clear();

  for (auto &element : avd_car_past_) {
    element.clear();
  }
  for (auto &element : avd_sp_car_past_) {
    element.clear();
  }

  virtual_lane_mgr_ = nullptr;
  path_planner_ = nullptr;
}

TaskStatus LateralBehaviorPlanner::execute(ScenarioFacadeContext *context) {
  MLOG_PROFILING(name_.c_str());
  if (Task::execute(context) != TaskStatus::STATUS_SUCCESS) {
    return TaskStatus::STATUS_FAILED;
  }

  if (virtual_lane_mgr_ == nullptr) {
    auto &map_info_mgr = world_model_->mutable_map_info_manager();
    virtual_lane_mgr_ = std::make_unique<VirtualLaneManager>(
        map_info_mgr.clane_, map_info_mgr.llane_, map_info_mgr.rlane_,
        map_info_mgr.rrlane_, map_info_mgr.lllane_, map_info_mgr.c_raw_refline_,
        map_info_mgr.l_raw_refline_, map_info_mgr.r_raw_refline_,
        map_info_mgr.ll_raw_refline_, map_info_mgr.rr_raw_refline_);
  }

  auto &state_machine_output = context_->state_machine_output();
  virtual_lane_mgr_->restore_context(
      state_machine_output.virtual_lane_mgr_context);
  baseline_info_ = world_model_->get_baseline_info(
      virtual_lane_mgr_->mutable_fix_refline().position());
  if (!baseline_info_ || !baseline_info_->is_valid()) {
    MSD_LOG(INFO, "LateralBehaviorPlanner baseline %d invalid",
            virtual_lane_mgr_->mutable_fix_refline().position());
    return TaskStatus::STATUS_FAILED;
  }

  if (path_planner_ == nullptr) {
    path_planner_ =
        std::make_unique<PathPlanner>(world_model_, *virtual_lane_mgr_);
  }

  if (calculate()) {
    return TaskStatus::STATUS_SUCCESS;
  } else {
    return TaskStatus::STATUS_FAILED;
  }
}

bool LateralBehaviorPlanner::calculate() {
  if (!update(world_model_->get_vehicle_dbw_status())) {
    return false;
  }

  return true;
}

bool LateralBehaviorPlanner::update(bool active) {
  auto &map_info_mgr = world_model_->get_map_info_manager();

  if (map_info_mgr.mmp_update_ == false) {
    return false;
  }

  restore_context();

  auto &state_machine_output = context_->state_machine_output();

  update_avoid_cars();

  update_path_planner();

  update_vel_sequence();

  update_ignore_track_id();

  update_avd_info();

  update_avdobstacles_info();

  update_obstacle_time_interval();

  ignore_cutin_avd_obstacles();

  if (!update_planner_output()) {
    return false;
  }

  print_planner_output();

  save_context();

  return true;
}

void LateralBehaviorPlanner::update_path_planner() {
  auto &state_machine_output = context_->state_machine_output();

  int state = state_machine_output.curr_state;
  bool should_premove = state_machine_output.should_premove;
  bool should_suspend = state_machine_output.should_suspend;

  path_planner_->update(state, flag_avd_, should_premove, should_suspend,
                        avd_car_past_, avd_sp_car_past_);

  avd_car_past_ = path_planner_->avd_car_past();
}

void LateralBehaviorPlanner::update_vel_sequence() {
  auto &lateral_output = context_->lateral_behavior_planner_output();
  auto &ego_state = baseline_info_->get_ego_state();

  auto &planning_result = context_->planning_status().planning_result;

  auto &v_array = planning_result.traj_vel_array;

  double time_diff = MTIME()->timestamp().sec() - planning_result.timestamp_sec;

  // vel_sequence_ set to ego_vel if hdmap valid is false
  if (v_array.size() > 0 && time_diff < 1.0 && lateral_output.enable) {
    vel_sequence_.resize(11);

    vel_sequence_[0] = v_array[0].target_velocity;

    for (int i = 0; i < 5; i++) {
      vel_sequence_[1 + i * 2] = v_array[(i * 2 + 1) * 4].target_velocity;
      vel_sequence_[1 + i * 2 + 1] = v_array[(i * 2 + 2) * 4].target_velocity;
    }
  } else {
    vel_sequence_.resize(1);
    vel_sequence_[0] = ego_state.ego_vel;
  }
}

void LateralBehaviorPlanner::update_ignore_track_id() {
  double ego_s = DBL_MAX;
  double ego_l = DBL_MAX;
  double theta = DBL_MAX;

  bool calc_ego_dist = false;
  double coefficient = FLAGS_planning_loop_rate / 50;

  auto &state_machine_output = context_->state_machine_output();

  int lc_request = state_machine_output.lc_request;
  auto &tlane = virtual_lane_mgr_->mutable_target_lane();
  auto &f_refline = virtual_lane_mgr_->mutable_fix_refline();
  auto &lateral_obstacle = world_model_->mutable_lateral_obstacle();
  auto &ego_state = baseline_info_->get_ego_state();
}

void LateralBehaviorPlanner::restore_context() {
  auto &planner_context =
      context_->lateral_behavior_planner_output().planner_context;

  no_sp_car_ = planner_context.no_sp_car;
  t_avd_sp_car_ = planner_context.t_avd_sp_car;
  final_y_rel_ = planner_context.final_y_rel;
  ncar_change_ = planner_context.ncar_change;
  flag_avd_ = planner_context.flag_avd;
  avd_back_cnt_ = planner_context.avd_back_cnt;
  avd_leadone_ = planner_context.avd_leadone;
  pre_leadone_id_ = planner_context.pre_leadone_id;
  ignore_track_id_ = planner_context.ignore_track_id;
  vel_sequence_ = planner_context.vel_sequence;
  ignore_change_false_ = planner_context.ignore_change_false;
  ignore_change_true_ = planner_context.ignore_change_true;
  avd_car_past_ = planner_context.avd_car_past;
  avd_sp_car_past_ = planner_context.avd_sp_car_past;

  path_planner_->restore_context(planner_context.path_planner);
}

void LateralBehaviorPlanner::save_context() const {
  auto &planner_context =
      context_->mutable_lateral_behavior_planner_output().planner_context;

  planner_context.no_sp_car = no_sp_car_;
  planner_context.t_avd_sp_car = t_avd_sp_car_;
  planner_context.final_y_rel = final_y_rel_;
  planner_context.ncar_change = ncar_change_;
  planner_context.flag_avd = flag_avd_;
  planner_context.avd_back_cnt = avd_back_cnt_;
  planner_context.avd_leadone = avd_leadone_;
  planner_context.pre_leadone_id = pre_leadone_id_;
  planner_context.ignore_track_id = ignore_track_id_;
  planner_context.vel_sequence = vel_sequence_;
  planner_context.ignore_change_false = ignore_change_false_;
  planner_context.ignore_change_true = ignore_change_true_;
  planner_context.avd_car_past = avd_car_past_;
  planner_context.avd_sp_car_past = avd_sp_car_past_;

  path_planner_->save_context(planner_context.path_planner);
}

} // namespace msquare
