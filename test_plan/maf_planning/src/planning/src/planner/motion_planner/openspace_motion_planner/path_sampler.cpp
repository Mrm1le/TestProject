#include "planner/motion_planner/openspace_motion_planner/path_sampler.h"
#include "common/planning_context.h"
#include <algorithm>
// #include "planning_config.h"
#include "planning/common/common.h"

namespace msquare {

using namespace parking;

PathSampler::PathSampler(const std::shared_ptr<WorldModel> &world_model)
    : MotionPlanner(world_model), fsm_ctx_(), zigzag_pathsampler_() {
  fsm_ = std::make_unique<path_sampler_statemachine::MPeerRoot>(
      fsm_ctx_ /* , HfsmContextLogger::Instance() */);
}

PathSampler::~PathSampler() {}

void PathSampler::set_path(const std::vector<TrajectoryPoint> &points) {
  MSD_LOG(INFO, "PathSampler::set_path\n");
  fsm_ctx_.path = ZigzagPath(points);
  fsm_ctx_.path_iter = fsm_ctx_.path.get_points().begin();
  zigzag_pathsampler_.reset_stage_idx();

  // publish path
  OpenSpacePath *openspace_path =
      PlanningContext::Instance()->mutable_open_space_path();
  openspace_path->path_poses.clear();
  openspace_path->path_poses.reserve(fsm_ctx_.path.get_points().size());
  openspace_path->path_poses.clear();
  // MSD_LOG(INFO, "----8<----Result of zigzag path----8<----\n");
  // MSD_LOG(INFO, "|index | direction | vel | acc | x | y | theta | time | s
  // |\n");
  int index = 0;
  for (auto iter = fsm_ctx_.path.get_points().begin();
       iter != fsm_ctx_.path.get_points().end(); ++iter) {
    PathPose path_pose;
    // auto q = tf::createQuaternionFromRPY(0, 0, iter->path_point.theta);
    using namespace Eigen;
    // Roll pitch and yaw in Radians
    Quaternionf q;
    q = AngleAxisf(0, Vector3f::UnitX()) * AngleAxisf(0, Vector3f::UnitY()) *
        AngleAxisf(iter->path_point.theta, Vector3f::UnitZ());
    path_pose.orient.x = q.x();
    path_pose.orient.y = q.y();
    path_pose.orient.z = q.z();
    path_pose.orient.w = q.w();
    path_pose.pos.x = iter->path_point.x;
    path_pose.pos.y = iter->path_point.y;
    path_pose.pos.z = iter->path_point.z;
    openspace_path->path_poses.push_back(path_pose);
    // MSD_LOG(INFO, "%d, %d, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", index,
    // iter->direction,
    //   iter->v, iter->a, iter->path_point.x, iter->path_point.y,
    //   iter->path_point.theta, iter->relative_time, iter->path_point.s);
    ++index;
  }

  fsm_ = std::make_unique<path_sampler_statemachine::MPeerRoot>(
      fsm_ctx_ /* , HfsmContextLogger::Instance() */);
}

bool PathSampler::calculate() {
  using namespace path_sampler_statemachine;

  if (PlanningContext::Instance()->open_space_path().isNew()) {
    set_path(PlanningContext::Instance()->mutable_open_space_path()->unstash());
    PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()->path_segment_updated = true;
  }
  MSD_LOG(INFO, "PathSampler::calculate\n");
  PlanningStatus *planning_status =
      PlanningContext::Instance()->mutable_planning_status();
  PlanningResult &planning_result = planning_status->planning_result;
  planning_result.traj_pose_array.clear();
  planning_result.traj_vel_array.clear();
  planning_result.traj_acceleration.clear();
  planning_result.traj_curvature.clear();
  planning_result.traj_curvature_radius.clear();
  planning_result.traj_relative_time.clear();
  planning_result.traj_relative_s.clear();

  if (planning_status->scenario.status_type == StatusType::WAIT ||
      planning_status->scenario.status_type ==
          StatusType::RPA_STRAIGHT_STANDBY) {
    planning_status->zigzag_num = 0;
  }

  if (world_model_->get_ego_state().is_static &&
      planning_status->scenario.status_type == StatusType::WAIT) {
    planning_result.gear = GearState::PARK;
  }
  if (world_model_->get_ego_state().is_static &&
      planning_status->scenario.status_type == StatusType::RPA_STRAIGHT_STANDBY) {
    planning_result.gear = GearState::PARK;
  }
  
  if (fsm_ctx_.path.get_points().empty()) {
    // not trajectory available but in APA task
    if (planning_status->scenario.status_type == StatusType::APA ||
        planning_status->scenario.next_status_type == StatusType::APA) {
      planning_result.gear = GearState::NONE;
      MSD_LOG(WARN, "%s: %d\n", __FUNCTION__, __LINE__);
    }
    if (!PlanningContext::Instance()
             ->parking_behavior_planner_output()
             .has_planned) {
      planning_result.gear = GearState::NONE;
      MSD_LOG(WARN, "%s: %d\n", __FUNCTION__, __LINE__);
    }
    return false;
  }

  fsm_->update();
  fsm_ctx_.is_vehicle_static = world_model_->get_ego_state().is_static;
  fsm_ctx_.gear.value = world_model_->get_gear_report().gear_status.value;

  if (fsm_->isActive<Follow>()) {
    MSD_LOG(DEBUG, "at stage %d",
            int(fsm_ctx_.path.get_stage_idx(fsm_ctx_.path_iter)));

    /******** update path iter in fsm context ********/

    auto ego_pose = world_model_->get_ego_state().ego_pose;
    auto ego_point = PathPoint(ego_pose.x, ego_pose.y, 0, ego_pose.theta);

    auto min_dis_iter = fsm_ctx_.path_iter;
    double min_dis = std::numeric_limits<double>::max();
    // to avoid mismatch of nearest point, only search on current stage
    auto search_upper_bound = fsm_ctx_.path.get_stage_upper(fsm_ctx_.path_iter);

    // TODO: deal with duplicated position

    for (auto iter = fsm_ctx_.path_iter; iter != search_upper_bound; ++iter) {
      double tmp_x = iter->path_point.x;
      double tmp_y = iter->path_point.y;
      double tmp_dis = std::hypot(tmp_x - ego_point.x, tmp_y - ego_point.y);
      if (tmp_dis < min_dis) {
        min_dis = tmp_dis;
        min_dis_iter = iter;
      }
    }
    MSD_LOG(INFO, "%s: min_dis: %f", __FUNCTION__, min_dis);
    MSD_LOG(INFO, "%s: ego pose: [%f,%f], nearest traj point [%f, %f]",
            __FUNCTION__, ego_pose.x, ego_pose.y, min_dis_iter->path_point.x,
            min_dis_iter->path_point.y);

    MSD_LOG(INFO, "%s: nearest traj point [x:%f, y:%f, vel:%f, acc:%f]",
            __FUNCTION__, min_dis_iter->path_point.x,
            min_dis_iter->path_point.y, min_dis_iter->v, min_dis_iter->a);

    bool is_gear_matching = fsm_->isActive<Follow::Reverse>() &&
                                planning_result.gear == GearState::REVERSE ||
                            fsm_->isActive<Follow::Drive>() &&
                                planning_result.gear == GearState::DRIVE;
    // enforce the upper_bound reaching condition
    double min_finish_len = CarParams::GetInstance()->car_config.common_config.min_finish_len;
    bool can_skip_to_upper_bound = (min_dis_iter + 2 == search_upper_bound);
    if (can_skip_to_upper_bound && is_gear_matching &&
        PlanningContext::Instance()
                ->longitudinal_behavior_planner_output()
                .traj_length < min_finish_len) {
      fsm_ctx_.path_iter = min_dis_iter + 1;
      PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()
          ->path_segment_updated = true;
    } else {
      fsm_ctx_.path_iter = min_dis_iter;
    }
    MSD_LOG(INFO, "%s: index: %d", __FUNCTION__,
            std::distance(fsm_ctx_.path.get_points().begin(), min_dis_iter));
    MSD_LOG(
        INFO, "%s: stage_end_index: %d", __FUNCTION__,
        std::distance(fsm_ctx_.path.get_points().begin(), search_upper_bound));
  }

  planning_result.gear_changing = !fsm_->isActive<Follow::Reverse>() &&
                                  !fsm_->isActive<Follow::Drive>() &&
                                  !fsm_->isActive<Finish>();
  // sample path of current stage
  if (!world_model_->get_pause_status() &&
      world_model_->get_planning_request().cmd.value != ParkingCommand::STOP) {
    if (fsm_->isActive<Follow::Reverse>()) {
      if (planning_result.gear != GearState::REVERSE) {
        planning_status->zigzag_num++;
      }
      planning_result.gear = GearState::REVERSE;
    } else if (fsm_->isActive<Follow::Drive>()) {
      if (planning_result.gear != GearState::DRIVE &&
          planning_status->zigzag_num > 0) {
        planning_status->zigzag_num++;
      }
      planning_result.gear = GearState::DRIVE;
    }
  }

  PlanningContext::Instance()
      ->mutable_longitudinal_behavior_planner_output()
      ->is_at_last_segment = is_at_last_segment();
  
  // debug
  *PlanningContext::Instance()->mutable_planning_debug_info() +=
      "[sampler]ps_sm" + std::to_string((int)this->getCurrentState()) +
      ",result.gear" + std::to_string((int)planning_result.gear) + ", " +
      get_segment_str();

  std::vector<DirTrajectoryPoint> result_traj =
      zigzag_pathsampler_.sample(fsm_ctx_.path, fsm_ctx_.path_iter);
  for (auto iter = result_traj.begin(); iter != result_traj.end(); ++iter) {
    MSD_LOG(DEBUG,
            "published traj point:[x: %4.2f, y: %4.2f, theta: %4.2f, "
            "direction: %d, v: %4.2f, a: %4.2f, steer: %4.2f \n",
            iter->path_point.x, iter->path_point.y, iter->path_point.theta,
            iter->direction, iter->v, iter->a, iter->steer);
    Pose2D pose;
    pose.x = iter->path_point.x;
    pose.y = iter->path_point.y;
    pose.theta = iter->path_point.theta;
    planning_result.traj_pose_array.push_back(pose);
    planning_result.traj_vel_array.push_back(iter->v);
    planning_result.traj_acceleration.push_back(iter->a);
    planning_result.traj_curvature.push_back(iter->path_point.kappa);
    planning_result.traj_curvature_radius.push_back(iter->path_point.rho);
    planning_result.traj_relative_time.push_back(iter->relative_time);
    planning_result.traj_relative_s.push_back(iter->path_point.s);
  }
  return true;
}

bool PathSampler::calculate_sop() {
  using namespace path_sampler_statemachine;

  if (PlanningContext::Instance()->open_space_path().isNew()) {
    MSD_LOG(ERROR, "update path segment: receive new path");
    PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()->path_segment_updated = true;
    set_path(PlanningContext::Instance()->mutable_open_space_path()->unstash());
    PlanningContext::Instance()->mutable_planning_status()->plan_control_interface.is_extended = true;
    PlanningResult &tmp_planning_result =
        PlanningContext::Instance()->mutable_planning_status()->planning_result;
    tmp_planning_result.second_traj_not_replan_scope
                = std::make_pair(0,0);
    tmp_planning_result.pre_segment_index = 0;
    tmp_planning_result.real_gear_change_flag = false;
    // tmp_planning_result.smart_change_direction_flag = false;
    tmp_planning_result.is_use_reach_the_end_flag = false;
    PlanningContext::Instance()->mutable_planning_status()
        ->plan_control_interface.is_use_traj_s_and_v = false;
    PlanningContext::Instance()->mutable_planning_status()
        ->plan_control_interface.second_remain_traj = 0.0;
    PlanningContext::Instance()->mutable_planning_status()
        ->plan_control_interface.second_traj_target_v = 0.0;
    tmp_planning_result.pre_real_gear = 0;
    tmp_planning_result.pre_wheel_velocity = 0;
    tmp_planning_result.is_finished_flag = false;
    smart_change_direction_flag = false;
    switch_traj_flag = false;
    mock_is_static_flag_ = false;
    not_use_reached_info = false;
  }
  MSD_LOG(INFO, "PathSampler::calculate\n");
  PlanningStatus *planning_status =
      PlanningContext::Instance()->mutable_planning_status();
  PlanningResult &planning_result = planning_status->planning_result;
  zigzag_pathsampler_.set_extend_length(0.1);
  planning_result.traj_pose_array.clear();
  planning_result.second_traj_pose_array.clear();
  planning_result.second_traj_curvature.clear();
  planning_result.second_traj_vel_array.clear();
  planning_result.traj_vel_array.clear();
  planning_result.traj_acceleration.clear();
  planning_result.traj_curvature.clear();
  planning_result.traj_curvature_radius.clear();
  planning_result.traj_relative_time.clear();
  planning_result.traj_relative_s.clear();

  if (planning_status->scenario.status_type == StatusType::WAIT) {
    planning_status->zigzag_num = 0;
  }

  if (world_model_->get_ego_state().is_static &&
      planning_status->scenario.status_type == StatusType::WAIT) {
    MSD_LOG(ERROR, "Wait set gear to Pakring");
    planning_result.gear = GearState::PARK;
  }

  if (fsm_ctx_.path.get_points().empty()) {
    // not trajectory available but in APA task
    if (planning_status->scenario.status_type == StatusType::APA ||
        planning_status->scenario.next_status_type == StatusType::APA) {
      planning_result.gear = GearState::NONE;
      MSD_LOG(WARN, "%s: %d\n", __FUNCTION__, __LINE__);
    }
    if (!PlanningContext::Instance()
             ->parking_behavior_planner_output()
             .has_planned) {
      planning_result.gear = GearState::NONE;
      MSD_LOG(WARN, "%s: %d\n", __FUNCTION__, __LINE__);
    }
    return false;
  }
  fsm_->update();
  if (mock_is_static_flag_) {
    MSD_LOG(ERROR," mock is_static flag");
    fsm_ctx_.is_vehicle_static = true;
  } else {
    fsm_ctx_.is_vehicle_static = world_model_->get_ego_state().is_static;
  }
  MSD_LOG(ERROR," mock is_static flag %d", mock_is_static_flag_);
  fsm_ctx_.gear.value = world_model_->get_gear_report().gear_status.value;
  bool is_change_traj = false;
  // std::cout << "************************************ SAtart" << std::endl;
  if (fsm_->isActive<Follow>()) {
    MSD_LOG(DEBUG, "at stage %d",
            int(fsm_ctx_.path.get_stage_idx(fsm_ctx_.path_iter)));

    /******** update path iter in fsm context ********/
    // std::cout << "the stage is: " 
    //           << int(fsm_ctx_.path.get_stage_idx(fsm_ctx_.path_iter))
    //           << std::endl;
    MSD_LOG(ERROR, "the stage is: %d",  int(fsm_ctx_.path.get_stage_idx(fsm_ctx_.path_iter)));
    MSD_LOG(ERROR, "sum stage is %d, ", int(fsm_ctx_.path.get_num_stages()));

    auto ego_pose = world_model_->get_ego_state().ego_pose;
    auto ego_point = PathPoint(ego_pose.x, ego_pose.y, 0, ego_pose.theta);

    auto min_dis_iter = fsm_ctx_.path_iter;
    double min_dis = std::numeric_limits<double>::max();
    // to avoid mismatch of nearest point, only search on current stage
    auto search_upper_bound = fsm_ctx_.path.get_stage_upper(fsm_ctx_.path_iter);

    // TODO: deal with duplicated position

    for (auto iter = fsm_ctx_.path_iter; iter != search_upper_bound; ++iter) {
      double tmp_x = iter->path_point.x;
      double tmp_y = iter->path_point.y;
      if (std::isnan(tmp_x) || std::isnan(tmp_y)) {
        MSD_LOG(ERROR,"the x or y is nan");
        continue;
      }
      double tmp_dis = std::hypot(tmp_x - ego_point.x, tmp_y - ego_point.y);
      if (tmp_dis < min_dis) {
        min_dis = tmp_dis;
        min_dis_iter = iter;
      }
    }
    MSD_LOG(INFO, "%s: min_dis: %f", __FUNCTION__, min_dis);
    MSD_LOG(INFO, "%s: ego pose: [%f,%f], nearest traj point [%f, %f]",
            __FUNCTION__, ego_pose.x, ego_pose.y, min_dis_iter->path_point.x,
            min_dis_iter->path_point.y);

    MSD_LOG(INFO, "%s: nearest traj point [x:%f, y:%f, vel:%f, acc:%f]",
            __FUNCTION__, min_dis_iter->path_point.x,
            min_dis_iter->path_point.y, min_dis_iter->v, min_dis_iter->a);

    bool is_gear_matching = fsm_->isActive<Follow::Reverse>() &&
                                planning_result.gear == GearState::REVERSE ||
                            fsm_->isActive<Follow::Drive>() &&
                                planning_result.gear == GearState::DRIVE;
    // enforce the upper_bound reaching condition
    // std::cout << " the Reverse" << fsm_->isActive<Follow::Reverse>()
    //           << " the Drive" << fsm_->isActive<Follow::Drive>()
    //           << std::endl;
    MSD_LOG(ERROR, "cur path point index is: %d, the end point index is: %d",
        std::distance(fsm_ctx_.path.get_points().begin(), min_dis_iter),
        std::distance(fsm_ctx_.path.get_points().begin(), search_upper_bound));
    // std::cout << " index is: " << std::distance(fsm_ctx_.path.get_points().begin(), min_dis_iter);
    // std::cout << " stage end index is:" << std::distance(fsm_ctx_.path.get_points().begin(), search_upper_bound);
    bool can_skip_to_upper_bound = (min_dis_iter + 2 == search_upper_bound);
    // std::cout << "is gear matching:" << is_gear_matching
    //           << " can_skip_to_upper_bound " << can_skip_to_upper_bound
    //           << " trajectory length: "
    //           << PlanningContext::Instance()
    //             ->longitudinal_behavior_planner_output()
    //             .traj_length
    //           << std::endl;
    MSD_LOG(ERROR, "is gear matching: %d . can_skip_to_upper_bound  %d . trajectory length: %f",
      is_gear_matching, can_skip_to_upper_bound, 
      PlanningContext::Instance()->longitudinal_behavior_planner_output().traj_length);
    bool reach_trajectory_end = 
        PlanningContext::Instance()->planning_status().is_reached_end;
    if (reach_trajectory_end && !not_use_reached_info) {
      planning_result.is_use_reach_the_end_flag = true;
    }
    MSD_LOG(ERROR, "the control reached info is: %d, not_use_reached_info is: %d",
        reach_trajectory_end, not_use_reached_info);
    // std::cout << "the control reached info is:" << reach_trajectory_end << std::endl;
    if (can_skip_to_upper_bound
        && is_gear_matching && PlanningContext::Instance()
                ->longitudinal_behavior_planner_output()
                .traj_length < 0.1) {
      fsm_ctx_.path_iter = min_dis_iter + 1;
      MSD_LOG(ERROR, "update path segment sucess");
      is_change_traj = true;
      PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()
          ->path_segment_updated = true;
    } else {
      fsm_ctx_.path_iter = min_dis_iter;
      is_change_traj = false;
    }

    int current_segment_0 = int(fsm_ctx_.path.get_stage_idx(fsm_ctx_.path_iter));
    MSD_LOG(ERROR,"current_segment_0 is: %d, last_segment_ %d", current_segment_0, last_segment_);
    static int once_use;
    MSD_LOG(ERROR,"is_use_reach_the_end_flag is: %d, is_extended %d traj_length %f, once_use %d",
      planning_result.is_use_reach_the_end_flag, planning_status->plan_control_interface.is_extended,
      PlanningContext::Instance()
              ->longitudinal_behavior_planner_output()
              .traj_length, once_use);
    if (current_segment_0 == last_segment_) {
      MSD_LOG(ERROR, "update path in curr segment");
      if (planning_result.is_use_reach_the_end_flag &&
          planning_status->plan_control_interface.is_extended &&
          !switch_traj_flag) {
        smart_change_direction_flag = true;
      }
    }
    MSD_LOG(ERROR, "smart_change_direction_flag is %d, switch_traj_flag %d",
        smart_change_direction_flag, switch_traj_flag);
    bool is_gear_match_velocity = false;
    int cur_real_gear = int(world_model_->get_gear_report().gear_status.value);
    if (planning_result.pre_real_gear != cur_real_gear) {
      if ((planning_result.pre_real_gear == 2 && cur_real_gear == 4) ||
          (planning_result.pre_real_gear == 4 && cur_real_gear == 2)) {
        planning_result.real_gear_change_flag = true;
        switch_traj_flag = false;
        MSD_LOG(ERROR, "set gear change to true");
      }
    }
    double curr_gear_v =  VehicleParam::Instance()->wheel_rolling_radius *
        (world_model_->get_wheel_speed_report().rear_left +
         world_model_->get_wheel_speed_report().rear_right +
         world_model_->get_wheel_speed_report().front_right +
         world_model_->get_wheel_speed_report().front_left ) / 4;
    MSD_LOG(ERROR, "curr gear is %d, the pre real gear is %d, real_gear_change_flag %d, and curr velocity is:%f, pre velocity is:%f",
        int(fsm_ctx_.gear.value),planning_result.pre_real_gear,
        planning_result.real_gear_change_flag, curr_gear_v,
        planning_result.pre_wheel_velocity);
    planning_result.pre_real_gear = int(fsm_ctx_.gear.value);
    if (planning_result.real_gear_change_flag
        && (std::fabs(curr_gear_v) < 0.05
        || planning_result.pre_wheel_velocity * curr_gear_v <= 0)) {
      MSD_LOG(ERROR,"the gear is match to the velocity");
      is_gear_match_velocity = true;
    }
    planning_result.pre_wheel_velocity = curr_gear_v;
    if (smart_change_direction_flag && is_gear_match_velocity) {
      // bool is_init_gear_info = fsm_ctx_.path_iter == search_upper_bound - 1;
      // fsm_ctx_.path_iter = search_upper_bound - 1;
      // smart_change_direction_flag = false;
      switch_traj_flag = true;
      mock_is_static_flag_ = true;
      not_use_reached_info = true;
      MSD_LOG(ERROR,"the gear is match the gear velocity, ready change next trajectory");
      // if (!world_model_->get_pause_status()) {
      //   if (fsm_ctx_.path_iter->direction == -1 ){
      //     if (planning_result.gear != GearState::REVERSE) {
      //       planning_status->zigzag_num++;
      //     }
      //     planning_result.gear = GearState::REVERSE;
      //   } else if(fsm_ctx_.path_iter->direction == 1 ) {
      //     if (planning_result.gear != GearState::DRIVE) {
      //       planning_status->zigzag_num++;
      //     }
      //     planning_result.gear = GearState::REVERSE;
      //   }
      // }
    }
    int current_segment_1 = int(fsm_ctx_.path.get_stage_idx(fsm_ctx_.path_iter));
    if(current_segment_1 != last_segment_){
      once_use = false;
      switch_traj_flag = true;
      planning_result.real_gear_change_flag = false;
      smart_change_direction_flag = false;
      MSD_LOG(ERROR, "update path segment 2");
      PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()->path_segment_updated = true;
      last_segment_ = current_segment_1;
      planning_status->plan_control_interface.is_use_traj_s_and_v = false;
      planning_status->plan_control_interface.second_remain_traj = 0.0;
      planning_status->plan_control_interface.second_traj_target_v = 0.0;
      planning_result.is_use_reach_the_end_flag = false;
      mock_is_static_flag_ = false;
      not_use_reached_info = false;
    }

    int planning_gear = 0;
    if (int(fsm_ctx_.gear.value) == 0) {
      planning_gear = 5;
    }
    if (int(fsm_ctx_.gear.value) == 1) {
      planning_gear = 0;
    }
    if (int(fsm_ctx_.gear.value) == 2) {
      planning_gear = 1;
    }
    if (int(fsm_ctx_.gear.value) == 3) {
      planning_gear = 2;
    }
    if (int(fsm_ctx_.gear.value) == 4) {
      planning_gear = 3;
    }
    if (int(fsm_ctx_.gear.value) == 5) {
      planning_gear = 4;
    }
    // if (!is_gear_matching) {
    //   once_use = 0;
    // }
    // std::cout << "cur gear is:" << planning_gear
    //           << " expect gear is:" << int(planning_result.gear)
    //           << std::endl;
    MSD_LOG(ERROR, "cur gear is: %d. expect gear is: %d", planning_gear, int(planning_result.gear));
    // std::cout << " index is: " << std::distance(fsm_ctx_.path.get_points().begin(), min_dis_iter);
    // std::cout << " stage end index is:" << std::distance(fsm_ctx_.path.get_points().begin(), search_upper_bound);
    MSD_LOG(INFO, "%s: index: %d", __FUNCTION__,
            std::distance(fsm_ctx_.path.get_points().begin(), min_dis_iter));
    MSD_LOG(
        INFO, "%s: stage_end_index: %d", __FUNCTION__,
        std::distance(fsm_ctx_.path.get_points().begin(), search_upper_bound));
  }

  MSD_LOG(ERROR, "The extend flag is %d",planning_status->plan_control_interface.is_extended);
  // std::cout << "The extend flag is: " << planning_status->plan_control_interface.is_extended << std::endl;

  planning_result.gear_changing = !fsm_->isActive<Follow::Reverse>() &&
                                  !fsm_->isActive<Follow::Drive>() &&
                                  !fsm_->isActive<Finish>();
  // sample path of current stage
  if (!world_model_->get_pause_status()) {
    if (fsm_->isActive<Follow::Reverse>()) {
      if (planning_result.gear != GearState::REVERSE) {
        planning_status->zigzag_num++;
      }
      planning_result.gear = GearState::REVERSE;
    } else if (fsm_->isActive<Follow::Drive>()) {
      if (planning_result.gear != GearState::DRIVE &&
          planning_status->zigzag_num > 0) {
        planning_status->zigzag_num++;
      }
      planning_result.gear = GearState::DRIVE;
    }
  }

  std::vector<DirTrajectoryPoint> result_traj =
      zigzag_pathsampler_.sample_sop(fsm_ctx_.path, fsm_ctx_.path_iter);
  std::vector<DirTrajectoryPoint> second_result_traj =
      zigzag_pathsampler_.get_second_traj(fsm_ctx_.path, fsm_ctx_.path_iter);
  // std::cout << "first traj size is:" << result_traj.size()
  //           << " second traj size is: " << second_result_traj.size()
  //           << " sum point size : " << fsm_ctx_.path.get_points().size()
  //           << " the segment size is:" << fsm_ctx_.path.get_num_stages()
  //           << std::endl;
  for (auto iter = result_traj.begin(); iter != result_traj.end(); ++iter) {
    // std::cout << "the direction is:" << iter->direction << std::endl;
    MSD_LOG(DEBUG,
            "published traj point:[x: %4.2f, y: %4.2f, theta: %4.2f, "
            "direction: %d, v: %4.2f, a: %4.2f, steer: %4.2f \n",
            iter->path_point.x, iter->path_point.y, iter->path_point.theta,
            iter->direction, iter->v, iter->a, iter->steer);
    Pose2D pose;
    pose.x = iter->path_point.x;
    pose.y = iter->path_point.y;
    pose.theta = iter->path_point.theta;
    planning_result.traj_pose_array.push_back(pose);
    planning_result.traj_vel_array.push_back(iter->v);
    planning_result.traj_acceleration.push_back(iter->a);
    planning_result.traj_curvature.push_back(iter->path_point.kappa);
    planning_result.traj_curvature_radius.push_back(iter->path_point.rho);
    planning_result.traj_relative_time.push_back(iter->relative_time);
    planning_result.traj_relative_s.push_back(iter->path_point.s);
  }
  for (auto iter = second_result_traj.begin(); iter != second_result_traj.end(); ++iter) {
    // std::cout << "the direction is:" << iter->direction << std::endl;
    MSD_LOG(DEBUG,
            "published traj point:[x: %4.2f, y: %4.2f, theta: %4.2f, "
            "direction: %d, v: %4.2f, a: %4.2f, steer: %4.2f \n",
            iter->path_point.x, iter->path_point.y, iter->path_point.theta,
            iter->direction, iter->v, iter->a, iter->steer);
    Pose2D pose;
    pose.x = iter->path_point.x;
    pose.y = iter->path_point.y;
    pose.theta = iter->path_point.theta;
    planning_result.second_traj_pose_array.push_back(pose);
    planning_result.second_traj_vel_array.push_back(iter->v);
    // planning_result.traj_acceleration.push_back(iter->a);
    planning_result.second_traj_curvature.push_back(iter->path_point.kappa);
    // planning_result.traj_curvature_radius.push_back(iter->path_point.rho);
    // planning_result.traj_relative_time.push_back(iter->relative_time);
    // planning_result.traj_relative_s.push_back(iter->path_point.s);
  }

  MSD_LOG(ERROR, "pre segment index is: %d", planning_result.pre_segment_index);
  MSD_LOG(ERROR, "second traj first is: %f, second is:%f",
          planning_result.second_traj_not_replan_scope.first,
          planning_result.second_traj_not_replan_scope.second);
  int current_segment_2 = int(fsm_ctx_.path.get_stage_idx(fsm_ctx_.path_iter));
  if (planning_result.pre_segment_index + 1 == current_segment_2) {
    // std::cout << "Finish Switch to next trajectory" << std::endl;
    MSD_LOG(ERROR, "Finish Switch to next trajectory");
    planning_status->plan_control_interface.is_extended
      = true;
    double second_traj_remain_s = planning_math::getRemainDistance(
        planning_result.traj_pose_array,
        planning_result.traj_vel_array,
        world_model_->get_ego_state().ego_pose);
    planning_result.is_finish_switch_next_traj_flag = true;
    second_traj_remain_s = second_traj_remain_s + 0.1;

    if (second_traj_remain_s > 0.6) {
      planning_result.second_traj_not_replan_scope
          = std::make_pair(second_traj_remain_s - 0.4 - 0.1,
          second_traj_remain_s);
    }
  }
  planning_result.pre_segment_index = current_segment_2;
  planning_status->plan_control_interface.gear_change_index
      = result_traj.size();
  planning_status->plan_control_interface.first_gear 
      = int(planning_result.gear);
  if (planning_result.gear == GearState::DRIVE) {
    planning_status->plan_control_interface.first_gear = 4;
  }
  if (planning_result.gear == GearState::REVERSE) {
    planning_status->plan_control_interface.first_gear = 2;
  }
  if (planning_result.gear == GearState::PARK) {
    planning_status->plan_control_interface.first_gear = 1;
  }
  if (planning_result.gear == GearState::NONE) {
    planning_status->plan_control_interface.first_gear = 0;
  }
  unsigned int second_gear = 0;
  const bool is_at_last_segment_temp = is_at_last_segment();
  PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()->is_at_last_segment = is_at_last_segment_temp;

  if (is_at_last_segment_temp) {
    second_gear = int(GearState::PARK);
        planning_status->plan_control_interface.is_extended
          = false;
    MSD_LOG(ERROR, "is last traj not allow extend");
  } else {
    if (planning_result.gear == GearState::DRIVE) {
      second_gear = int(GearState::REVERSE);
    }
    if (planning_result.gear == GearState::REVERSE) {
      second_gear = int(GearState::DRIVE);
    }
  }
  planning_status->plan_control_interface.second_gear 
      = second_gear;
  if (second_gear == 3) {
    planning_status->plan_control_interface.second_gear = 4;
  }
  if (second_gear == 1) {
    planning_status->plan_control_interface.second_gear = 2;
  }
  if (second_gear == 0) {
    planning_status->plan_control_interface.second_gear = 1;
  }
  if (second_gear == 5) {
    planning_status->plan_control_interface.second_gear = 0;
  }
  auto getTrajLength = [](std::vector<DirTrajectoryPoint> traj, bool is_first_traj){
    double length = 0.0;
    if (is_first_traj) {
      if (traj.size() < 12) {
        return length;
      }
    }
    if (traj.size() < 2) {
      return length;
    }
    int size = traj.size() - 1;
    if (is_first_traj) {
      size = traj.size() - 10;
    }
    for (int i = 0; i < size; i++) {
      length += std::hypot(traj[i + 1].path_point.x - traj[i].path_point.x,
                traj[i + 1].path_point.y - traj[i].path_point.y);
    }
    return length;
  };
  if (getTrajLength(result_traj, true) < 1.0 ||
      getTrajLength(second_result_traj, false) < 1.0) {
    planning_status->plan_control_interface.is_extended
        = false;
  MSD_LOG(ERROR, "the traj is too short. not extend first %f, second %f",
          getTrajLength(result_traj, true), getTrajLength(second_result_traj, false));
  }
  // if (result_traj.size() >= 3) {
  //   for (const auto& obs : world_model_->get_parking_ground_line_fusion()) {
  //     if () {

  //     }
  //   }
  // }

  if (is_at_last_segment_temp) {
    planning_status->plan_control_interface.is_use_traj_s_and_v = false;
    planning_status->plan_control_interface.second_remain_traj = 0.0;
    planning_status->plan_control_interface.second_traj_target_v = 0.0;
  }

  return true;
}

path_sampler_statemachine::SamplerFsmStateEnum PathSampler::getCurrentState() {
  using namespace path_sampler_statemachine;

  if (fsm_) {
    if (fsm_->isActive<Init>()) {
      return path_sampler_statemachine::SamplerFsmStateEnum::INIT;
    } else if (fsm_->isActive<Follow::Drive>()) {
      return path_sampler_statemachine::SamplerFsmStateEnum::DRIVE;
    } else if (fsm_->isActive<Follow::Reverse>()) {
      return path_sampler_statemachine::SamplerFsmStateEnum::REVERSE;
    } else if (fsm_->isActive<Follow::Drive2Reverse>()) {
      return path_sampler_statemachine::SamplerFsmStateEnum::DRIVE2REVERSE;
    } else if (fsm_->isActive<Follow::Reverse2Drive>()) {
      return path_sampler_statemachine::SamplerFsmStateEnum::REVERSE2DRIVE;
    } else if (fsm_->isActive<Finish>()) {
      return path_sampler_statemachine::SamplerFsmStateEnum::FINISH;
    }
  }
  // invalid call
  return path_sampler_statemachine::SamplerFsmStateEnum::INVALID;
}

bool PathSampler::is_arrived() const {
  return fsm_ctx_.path_iter == fsm_ctx_.path.get_points().end() - 1;
}

bool PathSampler::is_arrived(const Pose2D &ego_pose,
                             const double ending_distance) const {
  if (is_at_last_segment()) {
    const DirTrajectoryPoint end_point = fsm_ctx_.path.get_points().back();
    double dis2end = std::hypot(end_point.path_point.x - ego_pose.x,
                                end_point.path_point.y - ego_pose.y);
    return dis2end < ending_distance;
  }

  return false;
}

bool PathSampler::is_at_last_segment() const {
  return !fsm_ctx_.path.get_points().empty() &&
         (fsm_ctx_.path.get_stage_idx(fsm_ctx_.path_iter) + 1 ==
          fsm_ctx_.path.get_num_stages());
}

std::string PathSampler::get_segment_str() const {
  std::string str =
      ", sampler_seg(" +
      std::to_string(fsm_ctx_.path.get_stage_idx(fsm_ctx_.path_iter) + 1) +
      ", " + std::to_string(fsm_ctx_.path.get_num_stages()) + ") ";
  return str;
}

} // namespace msquare
