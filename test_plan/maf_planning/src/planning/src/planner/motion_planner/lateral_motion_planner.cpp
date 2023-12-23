#include "planner/motion_planner/lateral_motion_planner.h"
#include "common/config_context.h"
#include "common/math/math_utils.h"
#include "common/obstacle_manager.h"
#include "common/utils/utils_math.hpp"
#include "planner/behavior_planner/lateral_behavior_state.h"
#include "planner/motion_planner/lateral_motion_planner_preprocessor.h"
#include "planning/common/common.h"

namespace msquare {

LateralMotionPlanner::LateralMotionPlanner(const TaskConfig &config)
    : Task(config) {
  // mph_assert(config.has_lateral_motion_planner_config());
}

void LateralMotionPlanner::reset(const TaskConfig &config) {
  Task::reset(config);
  frenet_coord_ = nullptr;
}

void LateralMotionPlanner::lat_reset() {
  path_planner_.reset();
  MSD_LOG(ERROR, "[lat_reset] dbw_status = %d  and lat reset",
          world_model_->get_vehicle_dbw_status());
  MSD_LOG(ERROR, "[lat_reset] last_active_avd_obs_id_count_ = %d",
          path_planner_.get_last_active_avd_obs_id_count());
}

void LateralMotionPlanner::unset() { frenet_coord_ = nullptr; }

TaskStatus LateralMotionPlanner::execute(ScenarioFacadeContext *context) {
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

  frenet_coord_ = baseline_info_->get_frenet_coord();
  MSD_LOG(INFO, "[lat plan length] frenet_length = %.3f",
          frenet_coord_->GetLength());
  if (calculate()) {
    return TaskStatus::STATUS_SUCCESS;
  } else {
    return TaskStatus::STATUS_FAILED;
  }
}

void LateralMotionPlanner::init(std::shared_ptr<WorldModel> world_model) {
  Task::init(world_model);
}

bool LateralMotionPlanner::calculate() {
  PlanningStatus *planning_status = context_->mutable_planning_status();
  PlanningResult &planning_result = planning_status->planning_result;

  double adc_vel;
  std::array<double, 4> init_state;
  get_init_car_state(init_state, &adc_vel);

  // compute planner input
  LateralMotionPlannerPreprocessor lat_motion_planner_preprocessor;
  lat_motion_planner_preprocessor.init(context_, world_model_, baseline_info_);
  lat_motion_planner_preprocessor.process();

  double start_time = MTIME()->timestamp().sec();
  const auto &path_planner_input = context_->path_planner_input();
  path_planner_.update_input(path_planner_input, true);

  const auto &planner_debug = context_->mutable_planner_debug();
  planner_debug->lat_solver_report = path_planner_.get_solver_report();

  planner_debug->intelligent_avd_info.count_active_avd =
      path_planner_.get_count_active_avd();
  planner_debug->intelligent_avd_info.last_active_avd_state =
      path_planner_.get_last_active_avd_state();
  planner_debug->intelligent_avd_info.last_active_avd_obs_id =
      path_planner_.get_last_active_avd_obs_id();
  planner_debug->intelligent_avd_info.last_active_avd_obs_id_count =
      path_planner_.get_last_active_avd_obs_id_count();
  planner_debug->intelligent_avd_info.lc_end_clear_count =
      path_planner_.get_lc_end_clear_count();

  auto convert_to_pathpoint =
      [&](const path_planner::PathPlannerPoint &path_planner_output) {
        maf_planning::PathPoint path_point;
        path_point.curvature = path_planner_output.curvature;
        path_point.path_follow_strength = 0.0;
        path_point.position_enu.x = path_planner_output.x;
        path_point.position_enu.y = path_planner_output.y;
        path_point.heading_yaw =
            std::atan2(path_planner_output.dy_ds, path_planner_output.dx_ds);
        return path_point;
      };

  planning_result.curv_rate.clear();
  planning_result.traj_pose_array.clear();
  const auto &planning_init_point =
      baseline_info_->get_ego_state().planning_init_point.path_point;
  planning_result.curv_rate.emplace_back(planning_init_point.kappa);

  maf_planning::PathPoint planning_init_point_result;
  planning_init_point_result.curvature = planning_init_point.kappa;
  planning_init_point_result.heading_yaw = planning_init_point.theta;
  planning_init_point_result.path_follow_strength = 0.0;
  planning_init_point_result.position_enu.x = planning_init_point.x;
  planning_init_point_result.position_enu.y = planning_init_point.y;
  planning_result.traj_pose_array.emplace_back(
      std::move(planning_init_point_result));
  const auto &traj_vel_array = planning_result.traj_vel_array;
  double start_s = baseline_info_->get_ego_state().planning_start_state.s;

  path_planner::PathPlannerPoint path_planner_output;
  path_planner_.mutable_prev_output()->path_planner_output.clear();

  double max_s = std::fmin(
      path_planner_input.path_segments.back().end_segment_control_point_s,
      frenet_coord_->GetLength());

  double deta_s = std::max(adc_vel * 0.025, 0.1);
  const double min_length_from_last_path = 1.0;
  double sample_s = start_s + deta_s;
  while (sample_s < max_s) {
    path_planner_.get_output_at_s(&path_planner_input, path_planner_output,
                                  sample_s);
    if (sample_s <
        std::fmax(start_s + adc_vel * 1.0, min_length_from_last_path)) {
      path_planner_.mutable_prev_output()->path_planner_output.push_back(
          path_planner_output);
    }

    auto point_at_sample_s = convert_to_pathpoint(path_planner_output);
    planning_result.curv_rate.emplace_back(point_at_sample_s.curvature);
    planning_result.traj_pose_array.emplace_back(std::move(point_at_sample_s));
    sample_s += deta_s;
  }

  auto &lat_motion_output = context_->mutable_lateral_motion_planner_output();
  lat_motion_output.prev_path_planner_output = path_planner_.get_prev_output();

  const auto &lc_end_point_fren = path_planner_.get_lc_end_point();
  if (lc_end_point_fren.x > 0) {
    Point2D cart, fren;
    fren.x = std::min(lc_end_point_fren.x, frenet_coord_->GetLength());
    fren.y = lc_end_point_fren.y;
    (void)frenet_coord_->FrenetCoord2CartCoord(fren, cart);
    lat_motion_output.prev_lc_end_point = cart;
  }

  path_planner::PathPlannerOutput output;
  path_planner_.get_output(output);
  const auto avd_result_info = output.avd_result_info;
  if (avd_result_info.avd_direction == "left") {
    planning_result.lane_avd_in_lane_first_dir = 0;
  } else if (avd_result_info.avd_direction == "right") {
    planning_result.lane_avd_in_lane_first_dir = 1;
  } else {
    planning_result.lane_avd_in_lane_first_dir = -1;
  }
  planning_result.lane_avd_in_lane_first_id = avd_result_info.object_id;
  planning_result.lane_avd_in_lane_first_type = avd_result_info.type;
  planning_result.ego_faster_truck = (int)avd_result_info.ego_faster_truck;
  planning_result.overlap_lane = (int)avd_result_info.overlap_lane;

  planner_debug->intelligent_avd_info.ego_faster_truck =
        planning_result.ego_faster_truck;
  planner_debug->intelligent_avd_info.overlap_lane =
        planning_result.overlap_lane;
  double end_time = MTIME()->timestamp().sec();
  MSD_LOG(INFO, "planner_cost time new_path_planner_cost time : %.9f",
          end_time - start_time);

  lat_motion_output.path_planner_debug_info = output.debug_info_output;
  lat_motion_output.path_planner_output = output;

  lat_motion_output.pre_dodge_info = path_planner_.get_dodge_info();
  lat_motion_output.pre_dlp_info = path_planner_.get_dlp_info();

  return true;
}

void LateralMotionPlanner::get_init_car_state(std::array<double, 4> &init_state,
                                              double *adc_vel) {
  if (adc_vel == nullptr) {
    return;
  }

  const auto &init_point = baseline_info_->get_ego_state_manager()
                               .get_ego_state()
                               .planning_start_state;

  *adc_vel =
      std::max(baseline_info_->get_ego_state().planning_init_point.v, 0.1);
  init_state[0] = init_point.s;
  init_state[1] = init_point.r;
  init_state[2] = init_point.dr_ds;
  init_state[3] = init_point.ddr_dsds;
}

} // namespace msquare
