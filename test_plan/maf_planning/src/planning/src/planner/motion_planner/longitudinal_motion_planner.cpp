#include "planner/motion_planner/longitudinal_motion_planner.h"
#include "common/config_context.h"
#include "planner/motion_planner/common/speed_profile_generator.h"
#include "planner/motion_planner/speed_planner_ceres/speed_planner_preprocessor.h"
#include "planning/common/common.h"
#include "planning/common/logging.h"

namespace msquare {

void LongitudinalMotionPlanner::reset(const TaskConfig &config) {
  Task::reset(config);
}

void LongitudinalMotionPlanner::unset() {}

TaskStatus LongitudinalMotionPlanner::execute(ScenarioFacadeContext *context) {
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
  if (calculate()) {
    context->mutable_planning_status()->planning_success = true;
    context->mutable_planning_status()->backup_consecutive_loops = 0;
    return TaskStatus::STATUS_SUCCESS_BREAK;
  } else {
    return TaskStatus::STATUS_FAILED;
  }
}

void LongitudinalMotionPlanner::init(std::shared_ptr<WorldModel> world_model) {
  Task::init(world_model);
}

LongitudinalMotionPlanner::LongitudinalMotionPlanner(const TaskConfig &config)
    : Task(config) {}

bool LongitudinalMotionPlanner::calculate() {
  auto start_time = MTIME()->timestamp();
  LongitudinalPlannerPreprocessor longitudinal_planner_preprocessor;
  longitudinal_planner_preprocessor.init(context_, world_model_,
                                         baseline_info_);
  longitudinal_planner_preprocessor.process();

  auto speed_planner_output = context_->mutable_speed_planner_output();
  const auto &speed_planner_input = context_->speed_planner_input();
  speed_planner_.update_input(speed_planner_input, true);
  speed_planner_.get_output(*speed_planner_output);
  speed_planner_.get_opt_params(
      context_->mutable_speed_planner_input()->last_opt_params);

  const auto &planner_debug = context_->mutable_planner_debug();
  planner_debug->lon_solver_report = speed_planner_.get_solver_report();
  auto input_ptr = context_->mutable_speed_planner_input();
  speed_planner_.update_CIPV_info(input_ptr);
  bool enable_new_speed_planner =
      ConfigurationContext::Instance()
          ->planner_config()
          .longitudinal_motion_planner_config.enable_new_speed_planner;
  MSD_LOG(INFO, "enable_new_speed_planner : %d", enable_new_speed_planner);
  // output
  if (enable_new_speed_planner) {
    const auto &planning_init_state = speed_planner_input.planning_init_state;
    auto &planning_result =
        context_->mutable_planning_status()->planning_result;
    planning_result.pnc_stop = speed_planner_input.stop_point_info.hold_flag;
    planning_result.traj_vel_array.clear();
    planning_result.traj_acceleration.clear();
    double init_point_relative_time =
        baseline_info_->get_ego_state().planning_init_point.relative_time;

    SpeedPoint sp{};
    maf_planning::VelocityPoint point{};
    point.target_velocity = planning_init_state.v;
    planning_result.traj_vel_array.push_back(point);
    planning_result.traj_acceleration.push_back(planning_init_state.a);

    const int lon_traj_output_size = 201;
    const double delta_time = 0.025;
    for (const auto &speed_point : speed_planner_output->path_planner_output) {
      double t = planning_result.traj_vel_array.size() * delta_time;
      if (speed_point.t > t) {
        int insert_size =
            static_cast<int>((speed_point.t - t) / delta_time) + 1;
        for (int i = 0; i < insert_size; i++) {
          t = planning_result.traj_vel_array.size() * delta_time;
          const auto &last_point = planning_result.traj_vel_array.back();
          const auto &last_a = planning_result.traj_acceleration.back();
          double weight = (t - last_point.relative_time) /
                          (speed_point.t - last_point.relative_time);
          sp.a = (1 - weight) * last_a + weight * speed_point.a;
          point.target_velocity = (1 - weight) * last_point.target_velocity +
                                  weight * speed_point.v;
          point.relative_time =
              planning_result.traj_vel_array.size() * delta_time;
          point.distance = (1 - weight) * last_point.distance +
                           weight * (speed_point.s - planning_init_state.s);

          if (planning_result.traj_vel_array.size() < lon_traj_output_size) {
            planning_result.traj_vel_array.push_back(point);
            planning_result.traj_acceleration.push_back(sp.a);
          } else {
            break;
          }
        }
      }
    }

    for (int i = 0; i < std::min(planning_result.traj_vel_array.size(),
                                 planning_result.traj_acceleration.size());
         i++) {
      auto &velocity_point = planning_result.traj_vel_array[i];
      auto &accel = planning_result.traj_acceleration[i];
      if (velocity_point.target_velocity < 0.0) {
        velocity_point.relative_time = 1.0e8;
        velocity_point.target_velocity = 0.0;
        accel = 0.0;
      }
      velocity_point.relative_time += init_point_relative_time;
      if (std::fabs(velocity_point.relative_time - 0.1) < delta_time) {
        planning_result.v_target = velocity_point.target_velocity;
        planning_result.a_target = accel;
      }
    }

    double new_speed_planner_time_consume =
        (MTIME()->timestamp() - start_time).sec();
    MSD_LOG(INFO, "new_speed_planner_time_consume: %.9f",
            new_speed_planner_time_consume);

    if (planning_result.traj_vel_array.empty() ||
        planning_result.traj_acceleration.empty()) {
      MSD_LOG(ERROR,
              "longitudinal_motion_planner failed! traj_vel_array size: %d, "
              "traj_acceleration size: %d",
              planning_result.traj_vel_array.size(),
              planning_result.traj_acceleration.size());
      return false;
    }
    const auto &traj_vel_array = planning_result.traj_vel_array;
    auto curv_rate_old = planning_result.curv_rate;
    auto traj_pose_array_old = planning_result.traj_pose_array;
    auto &curv_rate = planning_result.curv_rate;
    auto &traj_pose_array = planning_result.traj_pose_array;
    curv_rate.clear();
    traj_pose_array.clear();
    const auto &traj_pose_frenet = planning_result.traj_pose_frenet;
    const int length =
        std::min(traj_pose_array_old.size(),
                 std::min(traj_pose_frenet.size(), curv_rate_old.size()));
    auto interp_lat_result = [&](double s) {
      for (int i = 0; i < length; ++i) {
        if (traj_pose_frenet[i].x > s) {
          if (i == 0) {
            curv_rate.push_back(curv_rate_old.front());
            traj_pose_array.push_back(traj_pose_array_old.front());
          } else {
            double rate = (s - traj_pose_frenet[i - 1].x) /
                          (traj_pose_frenet[i].x - traj_pose_frenet[i - 1].x);
            double curv = curv_rate_old[i - 1] +
                          rate * (curv_rate_old[i] - curv_rate_old[i - 1]);
            maf_planning::PathPoint traj_pose;
            traj_pose.position_enu.x =
                traj_pose_array_old[i - 1].position_enu.x +
                rate * (traj_pose_array_old[i].position_enu.x -
                        traj_pose_array_old[i - 1].position_enu.x);
            traj_pose.position_enu.y =
                traj_pose_array_old[i - 1].position_enu.y +
                rate * (traj_pose_array_old[i].position_enu.y -
                        traj_pose_array_old[i - 1].position_enu.y);
            traj_pose.curvature = traj_pose_array_old[i - 1].curvature +
                                  rate * (traj_pose_array_old[i].curvature -
                                          traj_pose_array_old[i - 1].curvature);
            traj_pose.heading_yaw = planning_math::interpolate_angle(
                traj_pose_frenet[i - 1].x,
                traj_pose_array_old[i - 1].heading_yaw, traj_pose_frenet[i].x,
                traj_pose_array_old[i].heading_yaw, s);
            traj_pose.path_follow_strength = 0.0;
            curv_rate.push_back(curv);
            traj_pose_array.push_back(traj_pose);
          }
          break;
        } else if (i == traj_pose_frenet.size() - 1) {
          curv_rate.push_back(curv_rate_old.back());
          traj_pose_array.push_back(traj_pose_array_old.back());
        }
      }
    };

    for (int i = 0; i < traj_vel_array.size(); ++i) {
      double start_s = baseline_info_->get_ego_state().planning_start_state.s;
      double s = traj_vel_array[i].distance + start_s;
      interp_lat_result(s);
    }

    if (traj_vel_array.back().distance < 8.0) {
      for (double extend_s = 0.0;
           extend_s < 8.0 - traj_vel_array.back().distance; extend_s += 0.1) {
        double start_s = baseline_info_->get_ego_state().planning_start_state.s;
        double s = traj_vel_array.back().distance + start_s + extend_s;
        interp_lat_result(s);
      }
    }
  }

  return true;
}

} // namespace msquare
