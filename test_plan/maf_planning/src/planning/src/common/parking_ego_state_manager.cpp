#include "common/parking_ego_state_manager.h"
#include "common/config/vehicle_param.h"
#include "common/math/math_utils.h"
#include "common/parking_world_model.h"
#include "planning/common/common.h"

namespace msquare {

namespace parking {

std::size_t
EgoStateManager::query_lower_bound_point(const std::vector<float> &time_array,
                                         double rel_time) {
  if (rel_time >= time_array.back()) {
    return (int)time_array.size() - 1;
  }

  auto func = [](float tp, double rel_time) { return tp < rel_time; };

  auto it_lower =
      std::lower_bound(time_array.begin(), time_array.end(), rel_time, func);
  return std::distance(time_array.begin(), it_lower);
}

std::size_t EgoStateManager::query_nearst_point_with_buffer(
    const std::vector<Pose2DType> &pose_array, double x, double y,
    double buffer) const {
  double dist_min = std::numeric_limits<double>::max();
  std::uint32_t index_min = 0;

  for (std::uint32_t i = 0; i < pose_array.size(); i++) {
    double dist = std::sqrt((x - pose_array[i].x) * (x - pose_array[i].x) +
                            (y - pose_array[i].y) * (y - pose_array[i].y));
    if (dist < dist_min + buffer) {
      dist_min = dist;
      index_min = i;
    }
  }

  return index_min;
}

std::size_t EgoStateManager::query_nearst_point_with_buffer(
    const std::vector<PathPoint> &pose_array, double x, double y,
    double buffer) const {
  double dist_min = std::numeric_limits<double>::max();
  std::uint32_t index_min = 0;

  for (std::uint32_t i = 0; i < pose_array.size(); i++) {
    double dist = std::sqrt((x - pose_array[i].x) * (x - pose_array[i].x) +
                            (y - pose_array[i].y) * (y - pose_array[i].y));
    if (dist < dist_min + buffer) {
      dist_min = dist;
      index_min = i;
    }
  }

  return index_min;
}

Point2D EgoStateManager::compute_position_projection(const double x,
                                                     const double y,
                                                     const Pose2D p,
                                                     double p_s) {
  double rel_x = x - p.x;
  double rel_y = y - p.y;
  double yaw = p.theta;
  Point2D frenet_sd;
  double n_x = cos(yaw);
  double n_y = sin(yaw);
  frenet_sd.x = p_s + rel_x * n_x + rel_y * n_y;
  frenet_sd.y = rel_x * n_y - rel_y * n_x;
  return frenet_sd;
}

void EgoStateManager::lateral_start_state_to_frenet_start_state(
    const std::shared_ptr<WorldModel> &world_model,
    const EgoState &lateral_planning_start_state,
    EgoState &frenet_lateral_planning_start_state) const {
  frenet_lateral_planning_start_state = lateral_planning_start_state;
  frenet_lateral_planning_start_state.ego_frenet.x =
      lateral_planning_start_state.ego_frenet.x;
  frenet_lateral_planning_start_state.ego_frenet.y = 0.0;
  Point2D cart_pt;
  auto frenet_ptr = world_model->get_frenet_coord();
  if (frenet_ptr == nullptr)
    return;
  (void)frenet_ptr->FrenetCoord2CartCoord(
      frenet_lateral_planning_start_state.ego_frenet, cart_pt);
  frenet_lateral_planning_start_state.ego_carte = cart_pt;
  frenet_lateral_planning_start_state.ego_pose.x = cart_pt.x;
  frenet_lateral_planning_start_state.ego_pose.y = cart_pt.y;
  frenet_lateral_planning_start_state.ego_pose.theta =
      frenet_ptr->GetRefCurveHeading(
          frenet_lateral_planning_start_state.ego_frenet.x);
}

void EgoStateManager::set_planning_start_state_parking(
    const std::shared_ptr<WorldModel> &world_model) {
  ego_state_.flag_is_replan = true;
  const double vel_deviation_threshold = 1.0;

  // the same with urban
  start_from_current_vehicle_state(world_model);
  ego_state_.real_init_point = ego_state_.planning_init_point;

  auto *planning_status =
      PlanningContext::Instance()->mutable_planning_status();
  if (planning_status->planning_loop == 0) {
    MSD_LOG(INFO, "before first planning loop, init starting");
    return;
  }

  if (!planning_status->planning_success) {
    MSD_LOG(INFO, "last loop failed, init starting");
    return;
  }

  PlanningResult &planning_result = planning_status->planning_result;

  // for TBDEBUG
  // planning_result.stop_flag = traffic_light_decision_->get_stop_flag();
  // planning_result.is_passed_stop_line =
  //     traffic_light_decision_->get_is_passed_stop_line();
  // planning_result.dist_to_stop = traffic_light_decision_->get_stop_point();

  if (planning_result.traj_pose_array.empty()) {
    MSD_LOG(INFO, "init starting");
    return;
  }

  // FOR GUYUNFENG AND TBDEBUG
  if (fabs(planning_result.v_target - ego_state_.ego_vel) >
      vel_deviation_threshold) {
    MSD_LOG(INFO, "vehicle-vel control-error is bigger than: %f",
            vel_deviation_threshold);
    return;
  }

  // calculate time interval between last published planning msg and now
  // double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;

  double duration = MTIME()->timestamp().sec() - planning_result.time;
  double rel_time = duration;

  MSD_LOG(INFO, "rel_time: %f", rel_time);

  std::size_t position_matched_index = query_nearst_point_with_buffer(
      planning_result.traj_pose_array, ego_state_.ego_pose.x,
      ego_state_.ego_pose.y, 1.0e-6);
  auto position_mathced_point =
      planning_result.traj_pose_array[position_matched_index];
  Point2D frenet_point_position, cart_point_position;
  cart_point_position.x = position_mathced_point.x;
  cart_point_position.y = position_mathced_point.y;
  (void)world_model->get_frenet_coord()->CartCoord2FrenetCoord(
      cart_point_position, frenet_point_position);

  // need modify for parking
  auto frenet_sd = compute_position_projection(
      ego_state_.ego_pose.x, ego_state_.ego_pose.y,
      planning_result.traj_pose_array[position_matched_index],
      frenet_point_position.x);
  auto lon_diff = frenet_sd.x;
  auto lat_diff = frenet_sd.y;

  planning_result.lon_error = lon_diff;
  planning_result.lat_error = lat_diff;
  // bool flag_s_replan = false;
  // if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) {

  //   MSD_LOG(INFO, "the distance between matched point and actual position is
  //   too "
  //            "large. Replan is triggered. lat_diff: %f",
  //            lat_diff);
  //   flag_s_replan = true;
  //   if (!dp_st_config::FLAGS_enable_s_update_mechanism) {
  //     return;
  //   }
  // }

  // if (std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {
  //   MSD_LOG(INFO, "the distance between matched point and actual position is
  //   too "
  //            "large. Replan is triggered. lon_diff: %f",
  //            lon_diff);
  //   flag_s_replan = true;
  //   if (!dp_st_config::FLAGS_enable_s_update_mechanism) {
  //     return;
  //   }
  // }

  // CartesianState cur_cart_state;
  // cur_cart_state.x = ego_state_.ego_pose.x;
  // cur_cart_state.y = ego_state_.ego_pose.y;
  // cur_cart_state.speed = ego_state_.ego_vel;
  // cur_cart_state.curvature = 0.0;
  // cur_cart_state.yaw = ego_state_.ego_pose.theta;
  // cur_cart_state.acceleration = ego_state_.ego_acc;

  //     // using current car state
  // ego_state_.flag_is_replan = false;
  // ego_state_.planning_init_point.path_point.s = frenet_sd.x;
  // ego_state_.planning_init_point.path_point.x = ego_state_.ego_pose.x;
  // ego_state_.planning_init_point.path_point.y = ego_state_.ego_pose.y;
  // ego_state_.planning_init_point.path_point.theta =
  // ego_state_.ego_pose.theta; auto flag =
  // world_model->get_frenet_coord()->CartState2FrenetState(
  //     cur_cart_state, ego_state_.planning_start_state);

  // ego_state_.planning_start_state.ds = std::max(0.0, ego_state_.ego_vel);
  // ego_state_.planning_start_state.dr_ds = 0.000001;
  // ego_state_.planning_start_state.dr = 0.000001;
  // ego_state_.planning_start_state.dds = 0.00001;
  // ego_state_.planning_start_state.ddr_dsds = 0.000001;
  // ego_state_.planning_start_state.ddr = 0.000001;

  // ego_state_.planning_init_point.v = std::max(0.0, ego_state_.ego_vel);
  // ego_state_.planning_init_point.a =
  //     (ego_state_.planning_init_point.v <= 0.000000001 && ego_state_.ego_acc
  //     < 0.0)
  //         ? 0.00001
  //         : ego_state_.ego_acc;
  // ego_state_.planning_init_point.relative_time = 0.0;
  // ego_state_.planning_init_point.path_point.kappa = 0.0;
  // ego_state_.planning_init_point.path_point.dkappa = 0.0;
  // ego_state_.planning_init_point.path_point.ddkappa = 0.0;
  // ego_state_.mpc_vehicle_state[0] = ego_state_.planning_init_point.v;
  // ego_state_.mpc_vehicle_state[1] = ego_state_.planning_init_point.a;
  // ego_state_.mpc_vehicle_state[2] =
  //     ego_state_.planning_init_point.path_point.theta -
  //     world_model->get_frenet_coord()->GetRefCurveHeading(
  //         ego_state_.planning_init_point.path_point.s);
  return;
}

void EgoStateManager::start_from_current_vehicle_state(
    const std::shared_ptr<WorldModel> &world_model) {
  CartesianState cs0;
  // FrenetState fs0;
  cs0.x = ego_state_.ego_pose.x;
  cs0.y = ego_state_.ego_pose.y;
  cs0.yaw = ego_state_.ego_pose.theta;
  cs0.speed = std::max(0.0, ego_state_.ego_vel);
  cs0.acceleration = ego_state_.ego_acc;
  cs0.curvature = 0;
  TRANSFORM_STATUS flag;
  flag = world_model->get_frenet_coord()->CartState2FrenetState(
      cs0, ego_state_.planning_start_state);
  ego_state_.planning_start_state.ds = std::max(0.0, cs0.speed);
  ego_state_.planning_start_state.dr_ds = 0.000001;
  ego_state_.planning_start_state.dr = 0.000001;
  ego_state_.planning_start_state.dds = 0.00001;
  ego_state_.planning_start_state.ddr_dsds = 0.000001;
  ego_state_.planning_start_state.ddr = 0.000001;
  // set planning_start_state_ without considering transformation
  if (flag == TRANSFORM_FAILED) {
    ego_state_.planning_start_state.s = ego_state_.ego_frenet.x;
    ego_state_.planning_start_state.r = ego_state_.ego_frenet.y;
    ego_state_.planning_start_state.ds = std::max(0.0, ego_state_.ego_vel);
    ego_state_.planning_start_state.dr_ds = 0.000001;
    ego_state_.planning_start_state.dr = 0.000001;
    ego_state_.planning_start_state.dds = 0.00001;
    ego_state_.planning_start_state.ddr_dsds = 0.000001;
    ego_state_.planning_start_state.ddr = 0.000001;
  }
  ego_state_.planning_init_point.v =
      std::max(0.0, ego_state_.ego_vel);                 // ego_vel_
  ego_state_.planning_init_point.a = ego_state_.ego_acc; // ego_acc_
  ego_state_.planning_init_point.relative_time = 0.0;
  ego_state_.planning_init_point.path_point.s =
      ego_state_.planning_start_state.s;
  ego_state_.planning_init_point.path_point.x = ego_state_.ego_pose.x;
  ego_state_.planning_init_point.path_point.y = ego_state_.ego_pose.y;
  ego_state_.planning_init_point.path_point.theta = ego_state_.ego_pose.theta;
  ego_state_.planning_init_point.path_point.kappa = 0.0;
  ego_state_.planning_init_point.path_point.dkappa = 0.0;
  ego_state_.planning_init_point.path_point.ddkappa = 0.0;
  ego_state_.mpc_vehicle_state[0] = ego_state_.planning_init_point.v;
  ego_state_.mpc_vehicle_state[1] = ego_state_.planning_init_point.a;
  ego_state_.mpc_vehicle_state[2] =
      ego_state_.planning_init_point.path_point.theta -
      world_model->get_frenet_coord()->GetRefCurveHeading(
          ego_state_.planning_init_point.path_point.s);
}

// void EgoStateManager::get_lat_planning_start_state(
//     const std::shared_ptr<WorldModel> &world_model,
//     FrenetState &lateral_planning_start_state) const {
//   CartesianState cs0;
//   FrenetState start_state;
//   // TRANSFORM_STATUS trans_flag = TRANSFORM_SUCCESS;

//   get_lat_replan_state(world_model, lateral_planning_start_state);

//   const PlanningStatus &planning_status =
//       PlanningContext::Instance()->planning_status();
//   const PlanningResult planning_result = planning_status.planning_result;

//   std::size_t position_matched_index = query_nearst_point_with_buffer(
//       planning_result.traj_pose_array, ego_state_.ego_pose.x,
//       ego_state_.ego_pose.y, 1.0e-6);
//   if (!(world_model->get_optimal_info().lc_status.find("lane_change") !=
//   std::string::npos && ego_state_.is_static)){ // for accident car
//     if (planning_result.traj_pose_array.size() > 0) {
//       cs0.x = planning_result.traj_pose_array[position_matched_index].x;
//       cs0.y = planning_result.traj_pose_array[position_matched_index].y;
//       cs0.yaw =
//       planning_result.traj_pose_array[position_matched_index].theta;
//       cs0.speed = std::max(0.0, ego_state_.ego_vel);
//       cs0.acceleration = ego_state_.ego_acc;
//       cs0.curvature = planning_result.traj_curvature[position_matched_index];
//       // trans_flag =
//       //     world_model->get_frenet_coord()->CartState2FrenetState(cs0,
//       start_state);
//       world_model->get_frenet_coord()->CartState2FrenetState(cs0,
//       start_state);
//       // todo : tune buffer
//       if (fabs(start_state.s - lateral_planning_start_state.s) < 1. &&
//           fabs(start_state.r - lateral_planning_start_state.r) < 0.5) {
//         lateral_planning_start_state = start_state;
//       }
//     }
//   }
// }

void EgoStateManager::get_parking_lat_planning_start_state(
    const std::shared_ptr<WorldModel> &world_model,
    FrenetState &lateral_planning_start_state) const {
  FrenetState init_point;
  CartesianState cs0_rel;

  // std::cout << "frenet: " << world_model->get_ego_state().ego_frenet.x << "
  // " << world_model->get_ego_state().ego_frenet.y << std::endl;

  auto frenet_state = world_model->get_ego_state().ego_frenet;

  const PlanningStatus &planning_status =
      PlanningContext::Instance()->planning_status();
  const PlanningResult planning_result = planning_status.planning_result;
  // world_model_->get_lat_planning_start_state(init_point);

  cs0_rel.x = world_model->get_ego_state().ego_pose.x;
  cs0_rel.y = world_model->get_ego_state().ego_pose.y;
  cs0_rel.yaw = world_model->get_ego_state().ego_pose.theta;
  cs0_rel.speed = std::max(0.0, world_model->get_ego_state().ego_vel);
  cs0_rel.acceleration = world_model->get_ego_state().ego_acc;
  cs0_rel.curvature = tan(world_model->get_ego_state().ego_steer_angle /
                          VehicleParam::Instance()->steer_ratio) /
                      VehicleParam::Instance()->wheel_base;
  (void)world_model->get_frenet_coord()->CartState2FrenetState(cs0_rel,
                                                               init_point);
  // std::cout << "srbdebug: " << planning_result.frenet_pwj_traj.size() << "  "
  // << planning_result.cart_pwj_traj.size() << std::endl;
  if (planning_result.frenet_pwj_traj.size() == 0 ||
      planning_result.cart_pwj_traj.size() == 0) {
    // std::cout << "srbdebug: if" << std::endl;
    lateral_planning_start_state = init_point;
  } else {
    // find current reference point in last pathpoints
    // std::cout << "srbdebug: else" << std::endl;
    std::size_t index = query_nearst_point_with_buffer(
        planning_result.cart_pwj_traj, world_model->get_ego_state().ego_pose.x,
        world_model->get_ego_state().ego_pose.y, 1e-9);
    PathPoint ego_state = planning_result.cart_pwj_traj[index];
    // std::cout << "srbdebug_initstate: " << index << " " << ego_state.x << " "
    // << ego_state.y << "  " << ego_state.s << "  " << ego_state.l <<
    // std::endl;
    FrenetState init_point_last;
    get_init_state_last_pathpoints(world_model, init_point_last, ego_state);
    if (std::abs(init_point_last.s - frenet_state.x) > 2.0 ||
        std::abs(init_point_last.r) > 2.0) {
      lateral_planning_start_state = init_point;
    } else
      lateral_planning_start_state = init_point_last;
  }
}

void EgoStateManager::get_cart_planning_start_state(
    const std::shared_ptr<WorldModel> &world_model,
    EgoState &lateral_planning_start_state,
    EgoState &frenet_lateral_planning_start_state) const {
  const double replan_distance = 0.2;
  const double look_ahead_time = 0.1;
  const double distance_ahead =
      0.0; // std::max(0.3, ego_state_.ego_vel) * look_ahead_time;
  const PlanningStatus &planning_status =
      PlanningContext::Instance()->planning_status();
  const PlanningResult planning_result = planning_status.planning_result;

  if (planning_result.path_points_real.empty()) {
    lateral_planning_start_state = world_model->get_ego_state();
    lateral_start_state_to_frenet_start_state(
        world_model, lateral_planning_start_state,
        frenet_lateral_planning_start_state);
    // std::cout << "srbdebug_start: ego" << std::endl;
  } else {
    //    std::cout << "srbdebug_start: traj" << std::endl;
    double ego_theta = world_model->get_ego_state().ego_pose.theta;
    double ego_vel = world_model->get_ego_state().ego_vel;
    double ego_x = world_model->get_ego_state().ego_pose.x +
                   ego_vel * look_ahead_time * std::cos(ego_theta);
    double ego_y = world_model->get_ego_state().ego_pose.y +
                   ego_vel * look_ahead_time * std::sin(ego_theta);
    auto iter =
        std::min_element(planning_result.path_points_real.begin(),
                         planning_result.path_points_real.end(),
                         [ego_x, ego_y](PathPoint a, PathPoint b) -> bool {
                           return std::hypot(a.x - ego_x, a.y - ego_y) <
                                  std::hypot(b.x - ego_x, b.y - ego_y);
                         });

    if (iter == planning_result.path_points_real.end()) {
      lateral_planning_start_state = world_model->get_ego_state();
      lateral_start_state_to_frenet_start_state(
          world_model, lateral_planning_start_state,
          frenet_lateral_planning_start_state);
      return;
    }
    double min_distance = std::hypot(iter->x - ego_x, iter->y - ego_y);
    if (min_distance > replan_distance ||
        std::abs(iter->theta - ego_theta) > 0.2) {
      lateral_planning_start_state = world_model->get_ego_state();
      lateral_start_state_to_frenet_start_state(
          world_model, lateral_planning_start_state,
          frenet_lateral_planning_start_state);
      return;
    }
    // std::cout << "srbdebug_init: " << min_distance << "  " << (iter -
    // planning_result.path_points_real.begin()) << " " << std::abs(iter->theta
    // - ego_theta) << std::endl;
    lateral_planning_start_state.ego_pose.x = iter->x;
    lateral_planning_start_state.ego_pose.y = iter->y;
    lateral_planning_start_state.ego_pose.theta = iter->theta;
    lateral_start_state_to_frenet_start_state(
        world_model, lateral_planning_start_state,
        frenet_lateral_planning_start_state);
    return;

    auto last_iter = iter;
    Pose2D project_last;
    double last_distance = 1e19;
    if (last_iter != planning_result.path_points_real.begin() + 1) {
      last_iter = iter - 1;
      project_last = planning_math::calc_projection_point(
          Pose2D(last_iter->x, last_iter->y, last_iter->theta),
          Pose2D(iter->x, iter->y, iter->theta),
          Pose2D(ego_x, ego_y, ego_theta));
      last_distance =
          std::hypot(project_last.x - ego_x, project_last.y - ego_y);
    }
    auto next_iter = iter;
    Pose2D project_next;
    double next_distance = 1e19;
    if (next_iter != planning_result.path_points_real.end() - 1) {
      next_iter = iter + 1;
      project_next = planning_math::calc_projection_point(
          Pose2D(next_iter->x, next_iter->y, next_iter->theta),
          Pose2D(iter->x, iter->y, iter->theta),
          Pose2D(ego_x, ego_y, ego_theta));
      next_distance =
          std::hypot(project_next.x - ego_x, project_next.y - ego_y);
    }
    Pose2D nearest_pose;
    double nearest_pose_s_last_traj = 0.0;
    auto start_iter = iter;
    if (last_distance < next_distance) {
      nearest_pose = project_last;
      nearest_pose_s_last_traj = std::hypot(nearest_pose.x - last_iter->x,
                                            nearest_pose.y - last_iter->y) +
                                 last_iter->s;
      start_iter = last_iter;
    } else {
      nearest_pose = project_next;
      nearest_pose_s_last_traj =
          std::hypot(nearest_pose.x - iter->x, nearest_pose.y - iter->y) +
          iter->s;
      start_iter = iter;
    }

    for (auto itr = start_iter; itr != planning_result.path_points_real.end();
         itr = itr + 1) {
      if ((itr->s - nearest_pose_s_last_traj) > distance_ahead) {
        // std::cout << "srbdebug_init: " << int(itr - start_iter) << std::endl;
        // calculate projection
        // double delta_s = distance_ahead - ((itr - 1)->s -
        // nearest_pose_s_last_traj); double diff_s = itr->s - (itr - 1)->s;
        lateral_planning_start_state.ego_pose.x = itr->x;
        lateral_planning_start_state.ego_pose.y = itr->y;
        lateral_planning_start_state.ego_pose.theta = itr->theta;
        // std::cout << "srbdebug_init: " << itr->x << "  " << itr->y << "  " <<
        // itr->theta << std::endl;
        return;
      }
    }
    lateral_planning_start_state = world_model->get_ego_state();
  }
}

void EgoStateManager::get_init_state_last_pathpoints(
    const std::shared_ptr<WorldModel> &world_model, FrenetState &init_point,
    const PathPoint &ego_state) const {
  std::size_t index_ref = 0;

  const PlanningStatus &planning_status =
      PlanningContext::Instance()->planning_status();
  const PlanningResult planning_result = planning_status.planning_result;
  // world_model_->get_lat_planning_start_state(init_point);
  // if(planning_result.frenet_pwj_traj.size() == 0 ||
  // planning_result.cart_pwj_traj.size() == 0){

  for (int i = 0; i < (int)planning_result.frenet_pwj_traj.size() - 1; i++) {

    if (i == 0 && ego_state.s <= planning_result.frenet_pwj_traj[i].s) {
      index_ref = 0;
      break;
    }
    if (ego_state.s < planning_result.frenet_pwj_traj[i].s) {
      index_ref = i - 1;
      break;
    }
  }

  // std::cout << "srbdebug-pwjpoint: " << index_ref << std::endl;

  Point2D cart_pp(ego_state.x, ego_state.y);
  Point2D frenet_pp;

  (void)world_model->get_frenet_coord()->CartCoord2FrenetCoord(cart_pp,
                                                               frenet_pp);

  // set l, dl, ddl with last solution
  double t = (ego_state.s - planning_result.frenet_pwj_traj[index_ref].s) /
             (planning_result.frenet_pwj_traj[index_ref + 1].s -
              planning_result.frenet_pwj_traj[index_ref].s);
  init_point.s = frenet_pp.x;
  init_point.r = frenet_pp.y;
  init_point.dr = planning_result.frenet_pwj_traj[index_ref].dl +
                  (planning_result.frenet_pwj_traj[index_ref + 1].dl -
                   planning_result.frenet_pwj_traj[index_ref].dl) *
                      t;
  init_point.ddr = planning_result.frenet_pwj_traj[index_ref].ddl +
                   (planning_result.frenet_pwj_traj[index_ref + 1].ddl -
                    planning_result.frenet_pwj_traj[index_ref].ddl) *
                       t;
}

void EgoStateManager::get_lat_replan_state(
    const std::shared_ptr<WorldModel> &world_model,
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
      tan(ego_state_.ego_steer_angle / VehicleParam::Instance()->steer_ratio) /
      VehicleParam::Instance()->wheel_base;
  // trans_flag_rel =
  //     world_model->get_frenet_coord()->CartState2FrenetState(cs0_rel,
  //     start_state_rel);
  (void)world_model->get_frenet_coord()->CartState2FrenetState(cs0_rel,
                                                               start_state_rel);
  lateral_planning_start_state = start_state_rel;
}

void EgoStateManager::update_transform() {
  Eigen::Vector4d q;
  q.x() = ego_state_.ego_enu.orientation.x;
  q.y() = ego_state_.ego_enu.orientation.y;
  q.z() = ego_state_.ego_enu.orientation.z;
  q.w() = ego_state_.ego_enu.orientation.w;
  // MSD_LOG(INFO, "cjtest_q x:%lf,y:%lf,z:%lf,w:%lf",q.x(),q.y(),q.z(),q.w());
  Eigen::Vector3d v;
  v.x() = ego_state_.ego_enu.position.x;
  v.y() = ego_state_.ego_enu.position.y;
  v.z() = ego_state_.ego_enu.position.z;

  // MSD_LOG(INFO, "cjtest_v x:%lf,y:%lf,z:%lf",v.x(),v.y(),v.z());
  car2enu_ = Transform(q, v);
  enu2car_ = Transform(q, v).inverse();
}

void EgoStateManager::set_ego_pose(const Pose2D &pose) {
  ego_state_.ego_pose = pose;

  // ego_state_.ego_pose.x -=
  //     std::cos(ego_state_.ego_pose.theta) *
  //     VehicleParam::Instance()->front_edge_to_center;
  // ego_state_.ego_pose.y -=
  //     std::sin(ego_state_.ego_pose.theta) *
  //     VehicleParam::Instance()->front_edge_to_center;

  // ego_state_.ego_vel = vel;

  // generate ego box
  planning_math::Vec2d vec_to_center(
      (VehicleParam::Instance()->front_edge_to_center -
       VehicleParam::Instance()->back_edge_to_center) /
          2.0,
      (VehicleParam::Instance()->left_edge_to_center -
       VehicleParam::Instance()->right_edge_to_center) /
          2.0);

  planning_math::Vec2d position(ego_state_.ego_pose.x, ego_state_.ego_pose.y);
  planning_math::Vec2d center(position +
                              vec_to_center.rotate(ego_state_.ego_pose.theta));

  ego_state_.ego_box = planning_math::Box2d(center, ego_state_.ego_pose.theta,
                                            VehicleParam::Instance()->length,
                                            VehicleParam::Instance()->width);
  // ego_state_.is_static = std::abs(ego_state_.ego_vel) <
  //                        VehicleParam::Instance()->max_abs_speed_when_stopped;
}

} // namespace parking

} // namespace msquare
