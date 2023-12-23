#include "common/trajectory/trajectory_stitcher.h"
#include "common/math/math_utils.h"
#include "common/planning_context.h"
#include "common/vehicle_model/vehicle_model.h"
#include "planning/common/common.h"
#include <algorithm>

namespace msquare {

TrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
    const VehicleState &vehicle_state) {
  TrajectoryPoint point;
  point.path_point.s = 0.0;
  point.path_point.x = vehicle_state.x;
  point.path_point.y = vehicle_state.y;
  point.path_point.z = vehicle_state.z;
  point.path_point.theta = vehicle_state.heading;
  point.path_point.kappa = vehicle_state.kappa;
  point.v = vehicle_state.linear_velocity;
  point.a = vehicle_state.linear_acceleration;
  point.relative_time = 0.0;
  return point; // parasoft-suppress AUTOSAR-A8_5_0 "f-drop"
}

std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const double planning_cycle_time, const VehicleState &vehicle_state) {
  TrajectoryPoint reinit_point;
  // do not predict for sim
  bool is_simulation = false;
  auto env = std::getenv("RealitySimulation");
  if (env != nullptr) {
    if (std::strcmp(env, "simulation") == 0) {
      is_simulation = true;
    }
  }
  static constexpr double kEpsilon_v = 0.1;
  static constexpr double kEpsilon_a = 0.4;
  if ((std::abs(vehicle_state.linear_velocity) < kEpsilon_v &&
       std::abs(vehicle_state.linear_acceleration) < kEpsilon_a) ||
      is_simulation) {
    reinit_point = ComputeTrajectoryPointFromVehicleState(vehicle_state);
  } else {
    VehicleState predicted_vehicle_state;
    predicted_vehicle_state =
        common::VehicleModel::Predict(planning_cycle_time, vehicle_state);
    reinit_point =
        ComputeTrajectoryPointFromVehicleState(predicted_vehicle_state);
    reinit_point.relative_time += planning_cycle_time;
  }

  return std::vector<TrajectoryPoint>(1, reinit_point);
}

// only used in navigation mode
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,
    DiscretizedTrajectory *prev_trajectory) {
  if (!prev_trajectory) {
    return;
  }

  // R^-1
  double cos_theta = std::cos(theta_diff);
  double sin_theta = -std::sin(theta_diff);

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

  std::for_each(
      prev_trajectory->begin(), prev_trajectory->end(),
      [&cos_theta, &sin_theta, &tx, &ty, &theta_diff](TrajectoryPoint &p) {
        auto x = p.path_point.x;
        auto y = p.path_point.y;
        auto theta = p.path_point.theta;

        auto x_new = cos_theta * x - sin_theta * y + tx;
        auto y_new = sin_theta * x + cos_theta * y + ty;
        auto theta_new = planning_math::NormalizeAngle(theta - theta_diff);

        p.path_point.x = x_new;
        p.path_point.y = y_new;
        p.path_point.theta = theta_new;
      });
}

DiscretizedTrajectory TrajectoryStitcher::ConstructTrajFromPlanningResult(
    const PlanningResult &planning_result) {
  const auto &traj_pose_array = planning_result.traj_pose_array;
  const auto &traj_vel_array = planning_result.traj_vel_array;
  const auto &traj_acc_array = planning_result.traj_acceleration;
  DiscretizedTrajectory points;
  mph_assert(traj_pose_array.size() == traj_vel_array.size());
  for (size_t i = 0; i < traj_pose_array.size(); ++i) {
    TrajectoryPoint point;
    point.path_point.s = traj_vel_array[i].distance;
    point.path_point.x = traj_pose_array[i].position_enu.x;
    point.path_point.y = traj_pose_array[i].position_enu.y;
    // point.path_point.z = traj_pose_array[i].position_enu.z;
    point.path_point.theta = traj_pose_array[i].heading_yaw;
    point.path_point.kappa = traj_pose_array[i].curvature;
    point.v = traj_vel_array[i].target_velocity;
    point.a = traj_acc_array[i];
    point.relative_time = traj_vel_array[i].relative_time;
    points.emplace_back(point);
  }
  // TODO(@zzd) use mtime API
  // points.set_header_time(planning_result.update_time.toSec());

  return points;
}

PublishedTrajectory TrajectoryStitcher::TransformToPublishedTraj(
    std::vector<TrajectoryPoint> trajectory) {
  PublishedTrajectory published_traj;
  for (auto point : trajectory) {
    maf_planning::PathPoint pose_point;
    maf_planning::VelocityPoint vel_point;
    pose_point.position_enu.x = point.path_point.x;
    pose_point.position_enu.y = point.path_point.y;
    pose_point.heading_yaw = point.path_point.theta;
    pose_point.curvature = point.path_point.kappa;
    vel_point.target_velocity = point.v;
    vel_point.relative_time = point.relative_time;
    vel_point.distance = point.path_point.s;
    published_traj.traj_pose_array.emplace_back(pose_point);
    published_traj.traj_vel_array.emplace_back(vel_point);
    published_traj.traj_acceleration.emplace_back(point.a);
  }
  return published_traj;
}

/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const VehicleState &vehicle_state, const double current_timestamp,
    const double planning_cycle_time, const size_t preserved_points_num,
    const bool replan_by_offset, PlanningResult *prev_planning_result,
    std::string *replan_reason) {

  DiscretizedTrajectory prev_trajectory(*prev_planning_result);
  if (!prev_trajectory.NumOfPoints()) {
    *replan_reason = "replan for no previous trajectory.";
    MSD_LOG(WARN, "trajectory stitcher: replan for no previous trajectory.");
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  if (vehicle_state.driving_mode != DrivingMode::AUTO) {
    *replan_reason = "replan for manual mode.";
    MSD_LOG(WARN, "trajectory stitcher: replan for manual mode.");
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  if (std::abs(vehicle_state.linear_velocity - prev_planning_result->v_target) >
          FLAGS_replan_velocity_threshold &&
      !vehicle_state.throttle_override) {
    MSD_LOG(
        WARN,
        "trajectory stitcher: vehicle-vel control-error is bigger than: %f ",
        FLAGS_replan_velocity_threshold);
    *replan_reason = "replan for empty previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  size_t prev_trajectory_size = prev_trajectory.size();

  if (prev_trajectory_size == 0) {
    MSD_LOG(WARN,
            "trajectory stitcher: Projected trajectory at time [%f] size is "
            "zero! Previous planning not exist or failed. Use "
            "origin car status instead.",
            prev_trajectory.header_time());
    *replan_reason = "replan for empty previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  const double veh_rel_time = current_timestamp - prev_trajectory.header_time();

  size_t time_matched_index =
      prev_trajectory.QueryLowerBoundPoint(veh_rel_time);

  if (time_matched_index == 0 &&
      veh_rel_time + 1.e-2 <
          prev_trajectory.StartPoint().relative_time - planning_cycle_time) {
    MSD_LOG(WARN, "trajectory stitcher: current time smaller than the previous "
                  "trajectory's first time");
    *replan_reason = "replan for current time smaller than the previous "
                     "trajectory's first time.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  if (time_matched_index + 1 >= prev_trajectory_size) {
    MSD_LOG(WARN, "trajectory stitcher: current time beyond the previous "
                  "trajectory's last time");
    *replan_reason =
        "replan for current time beyond the previous trajectory's last time";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  // auto time_matched_point = prev_trajectory.TrajectoryPointAt(
  //     static_cast<uint32_t>(time_matched_index));
  auto time_matched_point = prev_trajectory.Evaluate(veh_rel_time);

  MSD_LOG(WARN,
          "DEBUG_CJ13 pre_x: %.2f, pre_y: %.2f, time_point_x: %.2f, "
          "time_point_y: %.2f,  ego.x : %.2f, ego.y : %.2f, ego_v : %.2f",
          prev_trajectory.front().path_point.x,
          prev_trajectory.front().path_point.y, time_matched_point.path_point.x,
          time_matched_point.path_point.y, vehicle_state.x, vehicle_state.y,
          vehicle_state.linear_velocity);

  size_t position_matched_index = prev_trajectory.QueryNearestPointWithBuffer(
      {vehicle_state.x, vehicle_state.y}, 1.0e-6);
  auto position_matched_point = prev_trajectory.TrajectoryPointAt(
      static_cast<uint32_t>(position_matched_index));

  PlanningContext::Instance()
      ->mutable_planner_debug()
      ->replan_reason.time_and_pos_matched_point_dist = std::hypot(
      time_matched_point.path_point.x - position_matched_point.path_point.x,
      time_matched_point.path_point.y - position_matched_point.path_point.y);
  PlanningContext::Instance()
      ->mutable_planner_debug()
      ->replan_reason.position_matched_index = position_matched_index;
  PlanningContext::Instance()
      ->mutable_planner_debug()
      ->replan_reason.time_matched_index = time_matched_index;

  auto frenet_sd = ComputePositionProjection(
      vehicle_state.x, vehicle_state.y,
      prev_trajectory.TrajectoryPointAt(
          static_cast<uint32_t>(position_matched_index)));
  auto position_project_matched_index =
      prev_trajectory.QueryNearestPointWithBuffer(frenet_sd.first, 1.0e-6);
  auto position_project_matched_point = prev_trajectory.TrajectoryPointAt(
      static_cast<uint32_t>(position_project_matched_index));

  double forward_rel_time = std::max(veh_rel_time, planning_cycle_time);

  size_t forward_time_index =
      prev_trajectory.QueryLowerBoundPoint(forward_rel_time);
  // auto forward_time_trajectory_point =
  // prev_trajectory.at(forward_time_index);
  auto forward_time_trajectory_point =
      prev_trajectory.Evaluate(forward_rel_time);

  if (replan_by_offset) {
    auto lon_diff = time_matched_point.path_point.s - frenet_sd.first;
    auto lat_diff = frenet_sd.second;
    auto lon_time_diff = time_matched_point.relative_time -
                         position_project_matched_point.relative_time;
    prev_planning_result->lon_error = lon_diff;
    prev_planning_result->lat_error = lat_diff;

    MSD_LOG(WARN,
            "trajectory stitcher: Control lateral diff: %f, longitudinal diff: "
            "%f longitudinal time diff: %f",
            lat_diff, lon_diff, lon_time_diff);

    if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) {
      std::string msg(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lat_diff = " +
          std::to_string(lat_diff));
      MSD_LOG(WARN, "trajectory stitcher: %s", msg.c_str());
      *replan_reason = msg;
      auto reinit_traj =
          ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
      if (FLAGS_enable_s_update_mechanism) {
        reinit_traj.at(0).v = forward_time_trajectory_point.v;
        reinit_traj.at(0).a = forward_time_trajectory_point.a;
      }
      return reinit_traj;
    }

    double replan_longitudinal_distance_threshold =
        interp(vehicle_state.linear_velocity,
               FLAGS_replan_longitudinal_distance_threshold_speed,
               FLAGS_replan_longitudinal_distance_threshold_value);

    bool can_replan = true;
    const double PLAN_ACCEL_THR = -0.5;
    const double EGO_VELOCITY_THR = 1.0;
    // if (forward_time_trajectory_point.a < PLAN_ACCEL_THR &&
    //     vehicle_state.linear_velocity > EGO_VELOCITY_THR) {
    //   can_replan = false;
    //   MSD_LOG(WARN, "trajectory stitcher: braking, ego_accel is smaller than
    //   "
    //                 "plan_accel, do not replan");
    // }

    MSD_LOG(WARN,
            "forward_time_trajectory_point.a: %.2f, "
            "vehicle_state.linear_acceleration: %.2f, "
            "vehicle_state.linear_velocity: %.2f, can_replan : %d",
            forward_time_trajectory_point.a, vehicle_state.linear_acceleration,
            vehicle_state.linear_velocity, can_replan);

    if (can_replan &&
            std::fabs(lon_diff) > replan_longitudinal_distance_threshold ||
        vehicle_state.throttle_override) {
      std::string msg(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lon_diff = " +
          std::to_string(lon_diff) +
          " lon_time_diff = " + std::to_string(lon_time_diff));
      if (vehicle_state.throttle_override) {
        msg = "Replan is triggered because of throttle_override";
      }
      MSD_LOG(WARN, "trajectory stitcher: %s", msg.c_str());
      *replan_reason = msg;
      auto replan_traj =
          ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
      auto lon_replan_init_point = replan_traj.back();

      size_t lon_replan_position_matched_index =
          prev_trajectory.QueryNearestPointWithBuffer(
              {lon_replan_init_point.path_point.x,
               lon_replan_init_point.path_point.y},
              1.0e-6);
      auto lon_replan_position_matched_point =
          prev_trajectory.TrajectoryPointAt(
              static_cast<uint32_t>(lon_replan_position_matched_index));
      lon_replan_position_matched_point.relative_time =
          lon_replan_init_point.relative_time;
      lon_replan_position_matched_point.v = lon_replan_init_point.v;
      lon_replan_position_matched_point.a = lon_replan_init_point.a;

      auto position_projection_point =
          ComputePositionProjection(lon_replan_init_point.path_point.x,
                                    lon_replan_init_point.path_point.y,
                                    lon_replan_position_matched_point);

      std::vector<TrajectoryPoint> reinit_traj;
      if (std::abs(lon_replan_position_matched_point.path_point.s -
                   position_projection_point.first) >
          FLAGS_min_replan_longitudinal_distance_threshold) {
        reinit_traj = replan_traj;
      } else {
        reinit_traj =
            std::vector<TrajectoryPoint>(1, lon_replan_position_matched_point);
      }

      MSD_LOG(WARN, "Replan lon_replan_position_matched_index = %d",
              lon_replan_position_matched_index);

      if (FLAGS_enable_s_update_mechanism && !vehicle_state.throttle_override) {
        reinit_traj.at(0).v = forward_time_trajectory_point.v;
        reinit_traj.at(0).a = forward_time_trajectory_point.a;
      }
      return reinit_traj;
    }

  } else {
    MSD_LOG(WARN, "trajectory stitcher: replan according to certain amount of "
                  "lat and lon offset is "
                  "disabled");
  }

  MSD_LOG(WARN, "trajectory stitcher: Position matched index: %d",
          position_matched_index);
  MSD_LOG(WARN, "trajectory stitcher: Time matched index: %d",
          time_matched_index);

  std::vector<TrajectoryPoint> stitching_trajectory =
      std::vector<TrajectoryPoint>(1, forward_time_trajectory_point);
  constexpr double delta_t = 0.025;
  for (int i = 0; i < 5; i++) {
    double traj_t = forward_rel_time - delta_t * (i + 1);
    if (traj_t < prev_trajectory.front().relative_time) {
      break;
    }
    MSD_LOG(INFO, "traj_t: %f", traj_t);
    auto traj_point = prev_trajectory.Evaluate(traj_t);
    stitching_trajectory.insert(stitching_trajectory.begin(), traj_point);
  }

  // auto matched_index = std::min(time_matched_index, position_matched_index);

  // std::vector<TrajectoryPoint> stitching_trajectory(
  //     prev_trajectory.begin() +
  //         std::max(0, static_cast<int>(matched_index -
  //         preserved_points_num)),
  //     prev_trajectory.begin() + forward_time_index + 1);
  // MSD_LOG(WARN, "trajectory stitcher: stitching_trajectory size: %d",
  // stitching_trajectory.size());

  const double zero_s = stitching_trajectory.back().path_point.s;
  for (auto &tp : stitching_trajectory) {
    tp.relative_time =
        tp.relative_time + prev_trajectory.header_time() - current_timestamp;
    tp.path_point.s = tp.path_point.s - zero_s;
    MSD_LOG(INFO, "relative_time: %f", tp.relative_time);
  }
  return stitching_trajectory;
}

std::pair<double, double>
TrajectoryStitcher::ComputePositionProjection(const double x, const double y,
                                              const TrajectoryPoint &p) {
  planning_math::Vec2d v(x - p.path_point.x, y - p.path_point.y);
  planning_math::Vec2d n(std::cos(p.path_point.theta),
                         std::sin(p.path_point.theta));

  std::pair<double, double> frenet_sd;
  frenet_sd.first = v.InnerProd(n) + p.path_point.s;
  frenet_sd.second = v.CrossProd(n);
  return frenet_sd;
}

} // namespace msquare