#include "common/ego_state_manager.h"
#include "common/config/dp_st_speed_config.h"
#include "common/config_context.h"
#include "common/world_model.h"
#include "planning/common/common.h"

namespace msquare {

std::size_t CartEgoStateManager::query_lower_bound_point(
    const std::vector<maf_planning::VelocityPoint> &vel_array,
    double rel_time) {
  if (rel_time >= vel_array.back().relative_time) {
    return vel_array.size() - 1;
  }

  auto func = [](const maf_planning::VelocityPoint &point, double rel_time) {
    return point.relative_time < rel_time;
  };

  auto it_lower =
      std::lower_bound(vel_array.begin(), vel_array.end(), rel_time, func);
  return std::distance(vel_array.begin(), it_lower);
}

std::size_t CartEgoStateManager::query_nearst_point_with_buffer(
    const std::vector<maf_planning::PathPoint> &pose_array, double x, double y,
    double buffer) const {
  double dist_min = std::numeric_limits<double>::max();
  std::uint32_t index_min = 0;

  for (std::uint32_t i = 0; i < pose_array.size(); i++) {
    const auto &pos = pose_array[i].position_enu;
    double dist =
        std::sqrt((x - pos.x) * (x - pos.x) + (y - pos.y) * (y - pos.y));
    if (dist < dist_min + buffer) {
      dist_min = dist;
      index_min = i;
    }
  }

  return index_min;
}

Point2D CartEgoStateManager::compute_position_projection(
    const double x, const double y, const maf_planning::PathPoint &p,
    double p_s) {
  double rel_x = x - p.position_enu.x;
  double rel_y = y - p.position_enu.y;
  double yaw = p.heading_yaw;
  Point2D frenet_sd;
  double n_x = cos(yaw);
  double n_y = sin(yaw);
  frenet_sd.x = p_s + rel_x * n_x + rel_y * n_y;
  frenet_sd.y = rel_x * n_y - rel_y * n_x;
  return frenet_sd;
}

void CartEgoStateManager::compute_stitch_trajectory(bool dbw_status,
                                                    bool throttle_override) {
  VehicleState vehicle_state;
  vehicle_state.throttle_override = throttle_override;
  vehicle_state.x = cart_ego_state_.ego_pose.x;
  vehicle_state.y = cart_ego_state_.ego_pose.y;
  vehicle_state.yaw = cart_ego_state_.ego_pose.theta;
  vehicle_state.linear_acceleration = cart_ego_state_.ego_acc;
  vehicle_state.linear_velocity = cart_ego_state_.ego_vel;
  vehicle_state.kappa =
      tan(cart_ego_state_.ego_steer_angle /
          ConfigurationContext::Instance()->get_vehicle_param().steer_ratio) /
      ConfigurationContext::Instance()->get_vehicle_param().wheel_base;
  // TODO(@zzd) get speed direction
  vehicle_state.heading = cart_ego_state_.ego_pose.theta;
  // TODO(@zzd) get dbw status from worldmodel
  auto *planning_status =
      PlanningContext::Instance()->mutable_planning_status();
  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  auto &pre_planning_result = planning_status->planning_result;
  pre_planning_result.lon_error = 0.0;
  pre_planning_result.lat_error = 0.0;
  cart_ego_state_.flag_is_replan = false;
  // hack for cruise overshoot and override
  // when function triggered there is no need to replan from
  // current state in case driver acceleration is inherited
  static int active_cnt = 0;
  const int active_cnt_thres = 3;
  if (!dbw_status || throttle_override) {
    active_cnt = 0;
  } else {
    active_cnt = std::min(active_cnt + 1, 100);
  }
  if (planning_status->planning_loop == 0) {
    MSD_LOG(WARN, "before first planning loop, init starting");
    cart_ego_state_.stitch_trajectory =
        TrajectoryStitcher::ComputeReinitStitchingTrajectory(
            planning_cycle_time, vehicle_state);
    cart_ego_state_.real_init_point = cart_ego_state_.stitch_trajectory.back();
    cart_ego_state_.planning_init_point = cart_ego_state_.real_init_point;
    if (active_cnt < active_cnt_thres) {
      cart_ego_state_.real_init_point.a = 0;
      cart_ego_state_.planning_init_point.a = 0;
    }
    cart_ego_state_.flag_is_replan = true;
    return;
  }

  if (!planning_status->planning_success) {
    MSD_LOG(WARN, "last loop failed, init starting");
    cart_ego_state_.stitch_trajectory =
        TrajectoryStitcher::ComputeReinitStitchingTrajectory(
            planning_cycle_time, vehicle_state);
    cart_ego_state_.real_init_point = cart_ego_state_.stitch_trajectory.back();
    cart_ego_state_.planning_init_point = cart_ego_state_.real_init_point;
    if (active_cnt < active_cnt_thres) {
      cart_ego_state_.real_init_point.a = 0;
      cart_ego_state_.planning_init_point.a = 0;
    }
    cart_ego_state_.flag_is_replan = true;
    return;
  }

  vehicle_state.driving_mode =
      dbw_status ? DrivingMode::AUTO : DrivingMode::MANUAL;
  double last_pnc_time_consumption = planning_status->time_consumption;
  double rel_time = (planning_status->planning_result.next_timestamp_sec -
                     pre_planning_result.timestamp_sec);
  // TODO(@zzd): use absolute time in trajectory header and
  // ComputeStitchingTrajectory func call
  MSD_LOG(INFO, "last pnc time cost: %f rel_time: %f",
          last_pnc_time_consumption, rel_time);
  vehicle_state.timestamp = rel_time;
  bool replan_by_offset = true;
  std::string replan_reason{""};
  constexpr double kPreservedPointsNum = 5;
  constexpr bool isAlwaysReplan = false; // true only for open-loop simulation
  if (isAlwaysReplan) {
    cart_ego_state_.stitch_trajectory =
        TrajectoryStitcher::ComputeReinitStitchingTrajectory(0.0,
                                                             vehicle_state);
    cart_ego_state_.flag_is_replan = true;
  } else {
    cart_ego_state_.stitch_trajectory =
        TrajectoryStitcher::ComputeStitchingTrajectory(
            vehicle_state, rel_time, planning_cycle_time, kPreservedPointsNum,
            replan_by_offset, &pre_planning_result, &replan_reason);
    PlanningContext::Instance()
        ->mutable_planner_debug()
        ->replan_reason.reason_str = replan_reason;
    cart_ego_state_.flag_is_replan = replan_reason == "" ? false : true;
  }
  cart_ego_state_.planning_init_point =
      cart_ego_state_.stitch_trajectory.back();
  cart_ego_state_.real_init_point =
      TrajectoryStitcher::ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                           vehicle_state)
          .back();
  if (active_cnt < active_cnt_thres) {
    cart_ego_state_.real_init_point.a = 0;
    cart_ego_state_.planning_init_point.a = 0;
  }
}

void EgoStateManager::get_lat_replan_state(
    FrenetState &lateral_planning_start_state) const {
  CartesianState cs0_rel;
  FrenetState start_state_rel;
  // TRANSFORM_STATUS trans_flag_rel = TRANSFORM_SUCCESS;

  cs0_rel.x = ego_state_.ego_pose.x;
  cs0_rel.y = ego_state_.ego_pose.y;
  cs0_rel.yaw = ego_state_.ego_pose.theta;
  cs0_rel.speed = std::max(0.0, ego_state_.ego_vel);
  cs0_rel.acceleration = ego_state_.ego_acc;
  cs0_rel.curvature =
      tan(ego_state_.ego_steer_angle /
          ConfigurationContext::Instance()->get_vehicle_param().steer_ratio) /
      ConfigurationContext::Instance()->get_vehicle_param().wheel_base;
  // trans_flag_rel =
  //     frenet_coord_->CartState2FrenetState(cs0_rel,
  //                                          start_state_rel);
  (void)frenet_coord_->CartState2FrenetState(cs0_rel, start_state_rel);
  lateral_planning_start_state = start_state_rel;

  lateral_planning_start_state.ds = ego_state_.ego_vel;
  lateral_planning_start_state.dds = ego_state_.ego_acc;
}

void CartEgoStateManager::update_transform() {
  Eigen::Vector4d q;
  q.x() = cart_ego_state_.ego_enu.orientation.x;
  q.y() = cart_ego_state_.ego_enu.orientation.y;
  q.z() = cart_ego_state_.ego_enu.orientation.z;
  q.w() = cart_ego_state_.ego_enu.orientation.w;
  Eigen::Vector3d v;
  v.x() = cart_ego_state_.ego_enu.position.x;
  v.y() = cart_ego_state_.ego_enu.position.y;
  // v.z() = cart_ego_state_.ego_enu.position.z;
  v.z() = 0.0; // TODO: set to zero because when we use transfrom, z is always
               // set to zero

  car2enu_ = Transform(q, v);
  enu2car_ = Transform(q, v).inverse();
}

} // namespace msquare
