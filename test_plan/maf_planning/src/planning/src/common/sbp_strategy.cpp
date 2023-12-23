#include "common/sbp_strategy.h"
#include "common/priority_obs.h"
#include "common/sbp_comfort_gear_zone.h"
#include "common/sbp_map_line.h"
#include "common/sbp_obstacle_box.h"
#include "common/sbp_obstacle_line.h"
#include "common/sbp_obstacle_point.h"
#include "common/utils/trajectory_point_utils.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/openspace_optimizer.h"


namespace msquare {
namespace parking {

using namespace planning_math;

bool GetTemporalProfile(SbpResult *result) {
  double partition_init_v_ = 0.5;
  double partition_init_a_ = 1.0;
  std::vector<SbpResult> partitioned_results;
  if (!TrajectoryPartition(*result, &partitioned_results, partition_init_v_,
                           partition_init_a_)) {
    // std::cout << "TrajectoryPartition fail" << std::endl;
    return false;
  }
  SbpResult stitched_result;
  stitched_result.status = result->status;
  stitched_result.iteration_times = result->iteration_times;
  stitched_result.debug_string = result->debug_string;
  stitched_result.num_segments = partitioned_results.size();

  for (const auto &result : partitioned_results) {
    stitched_result = stitched_result + result;
  }
  stitched_result.isAligned();
  *result = stitched_result;
  return true;
}

bool TrajectoryPartition(const SbpResult &result,
                         std::vector<SbpResult> *partitioned_result,
                         double init_v, double init_a) {
  auto x = result.x;
  auto y = result.y;
  auto phi = result.phi;
  if (x.size() != y.size() || x.size() != phi.size()) {
    // std::cout << "states sizes are not equal when do trajectory partitioning
    // of "
    //           "Hybrid A Star result" << std::endl;
    return false;
  }

  size_t horizon = x.size();
  // deprecate duplicated points
  for (size_t i = 0; i + 1 < horizon;) {
    if (std::abs(x[i] - x[i + 1]) > 1e-6 || std::abs(y[i] - y[i + 1]) > 1e-6 ||
        std::abs(phi[i] - phi[i + 1]) > 1e-6) {
      ++i;
    } else {
      x.erase(x.begin() + i);
      y.erase(y.begin() + i);
      phi.erase(phi.begin() + i);
      horizon -= 1;
    }
  }
  partitioned_result->clear();
  partitioned_result->emplace_back();
  auto *current_traj = &(partitioned_result->back());
  double heading_angle = phi.front();
  const planning_math::Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]);
  double tracking_angle = init_tracking_vector.Angle();

  std::vector<int> partition_gears;
  bool current_gear =
      std::abs(planning_math::NormalizeAngle(tracking_angle - heading_angle)) <
      (M_PI_2);
  partition_gears.push_back(current_gear ? 1 : -1);
  for (size_t i = 0; i + 1 < horizon; ++i) {
    heading_angle = phi[i];
    const planning_math::Vec2d tracking_vector(x[i + 1] - x[i],
                                               y[i + 1] - y[i]);
    tracking_angle = tracking_vector.Angle();
    bool gear = std::abs(planning_math::NormalizeAngle(
                    tracking_angle - heading_angle)) < (M_PI_2);
    if (gear != current_gear) {
      current_traj->x.push_back(x[i]);
      current_traj->y.push_back(y[i]);
      current_traj->phi.push_back(phi[i]);
      current_traj->wheel_base_offset.push_back(result.wheel_base_offset[i]);
      partitioned_result->emplace_back();
      // prepare for next partition
      current_traj = &(partitioned_result->back());
      current_gear = gear;
      partition_gears.push_back(gear ? 1 : -1);
    }
    current_traj->x.push_back(x[i]);
    current_traj->y.push_back(y[i]);
    current_traj->phi.push_back(phi[i]);
    current_traj->wheel_base_offset.push_back(result.wheel_base_offset[i]);
  }
  current_traj->x.push_back(x.back());
  current_traj->y.push_back(y.back());
  current_traj->phi.push_back(phi.back());
  current_traj->wheel_base_offset.push_back(result.wheel_base_offset.back());

  // Retrieve v, a and steer from path
  for (auto &result : *partitioned_result) {
    if (!GenerateSpeedAcceleration(&result)) {
      // std::cout << "GenerateSpeedAcceleration fail" <<std::endl;
      return false;
    }
  }
  // modify init v and a for each partition
  for (size_t i = 0; i < partitioned_result->size(); ++i) {
    double init_v_i = init_v * partition_gears.at(i);
    double init_a_i = init_a * partition_gears.at(i);
    auto &result = partitioned_result->at(i);
    result.v.front() = init_v_i;
    result.a.front() = init_a_i;
  }

  return true;
}

bool GenerateSpeedAcceleration(SbpResult *result) {
  double delta_t_ = 1.0;
  double wheel_base_ = VehicleParam::Instance()->wheel_base;
  double step_size_ = 0.5;
  // Sanity Check
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    // std::cout << "GenerateSpeedAcceleration fail, size(x) is " <<
    // result->x.size() << std::endl;
    return false;
  }
  const size_t x_size = result->x.size();

  // load velocity from position
  if (!result->v.empty() || !result->a.empty() || !result->steer.empty()) {
    // throw std::logic_error(
    //     "HybridAstar::GenerateSpeedAcceleration sanity check failed");
  }
  for (size_t i = 0; i + 1 < x_size; ++i) {
    double discrete_v = (((result->x[i + 1] - result->x[i]) / delta_t_) *
                         std::cos(result->phi[i])) +
                        (((result->y[i + 1] - result->y[i]) / delta_t_) *
                         std::sin(result->phi[i]));
    result->v.push_back(discrete_v);
  }
  result->v.push_back(result->v.back());

  // load acceleration from velocity
  for (size_t i = 0; i + 1 < x_size; ++i) {
    const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
    result->a.push_back(discrete_a);
  }
  result->a.push_back(result->a.back());

  // load steering from phi
  if (msquare::CarParams::GetInstance()->car_config.lon_config.use_sop_algorithm) {
    if (!(result->steer.size() >= 2 && result->x.size() == result->steer.size())) {
      for (size_t i = 0; i + 1 < x_size; ++i) {
        double discrete_steer =
            planning_math::NormalizeAngle((result->phi[i + 1] - result->phi[i])) *
            wheel_base_ /
            std::hypot(result->x[i + 1] - result->x[i],
                      result->y[i + 1] - result->y[i]);
        if (result->v[i] > 0.0) {
          discrete_steer = std::atan(discrete_steer);
        } else {
          discrete_steer = std::atan(-discrete_steer);
        }
        result->steer.push_back(discrete_steer);
      }
      result->steer.push_back(result->steer.back());
    }    
  } else {
    for (size_t i = 0; i + 1 < x_size; ++i) {
      double discrete_steer =
          planning_math::NormalizeAngle((result->phi[i + 1] - result->phi[i])) *
          wheel_base_ /
          std::hypot(result->x[i + 1] - result->x[i],
                    result->y[i + 1] - result->y[i]);
      if (result->v[i] > 0.0) {
        discrete_steer = std::atan(discrete_steer);
      } else {
        discrete_steer = std::atan(-discrete_steer);
      }
      result->steer.push_back(discrete_steer);
    }
    result->steer.push_back(result->steer.back());
  }

  // generate s
  if (!result->accumulated_s.empty()) {
    // throw std::logic_error(
    //     "HybridAstar::GetTemporalProfile sanity check failed");
  }
  result->accumulated_s.push_back(0);
  for (size_t i = 1; i < x_size; ++i) {
    result->accumulated_s.push_back(
        result->accumulated_s[i - 1] +
        std::hypot(result->x[i] - result->x[i - 1],
                   result->y[i] - result->y[i - 1]));
  }

  result->isAligned();

  return true;
}

std::vector<TrajectoryPoint>
planWithStrategy(const SearchBasedPlannerPtr sbp, const StrategyParams *cfg,
                 const OpenspaceDeciderOutput &input) {
  // std::cout << "planWithStrategy started..." << std::endl;

  std::vector<TrajectoryPoint> traj;
  SbpResult sbp_result;
  if (genOpenspacePath(
          sbp, extractSearchProblem(StrategyParams::GetInstance(), input),
          sbp_result)) {
    (void)assembleSearchProblem(cfg, sbp_result);
    if (!GetTemporalProfile(&sbp_result)) {
      // std::cout << "GetSpeedProfile from Hybrid Astar path fails" <<
      // std::endl;
    }
    traj = convertSbpResult2Traj(sbp_result);
    regulateVelocity(cfg, traj);
    // std::cout << "planWithStrategy succeed!" << std::endl;
  } else {
    // std::cout << "planWithStrategy failed!" << std::endl;
  }
  return traj;
}

OpenspaceDeciderOutput
extractSearchProblem(const StrategyParams *cfg,
                     const OpenspaceDeciderOutput &input_) {
  OpenspaceDeciderOutput partial_problem = genPartialProblem(cfg, input_);
  if (cfg->enable_revserse_search_) {
    partial_problem = genReverseProblem(partial_problem);
  }
  return partial_problem;
}

OpenspaceDeciderOutput genPartialProblem(const StrategyParams *cfg,
                                         const OpenspaceDeciderOutput &input_) {
  if (std::fabs(cfg->default_check_endsegent_len_) < 1e-8) {
    return input_;
  }

  double check_endsegent_len = std::abs(cfg->default_check_endsegent_len_);
  if (cfg->enable_endsegment_len_correction_) {
    auto &init_point = input_.init_state.path_point;
    auto &target_point = input_.target_state.path_point;
    double denominator =
        std::cos(init_point.theta) * std::sin(target_point.theta) -
        std::sin(init_point.theta) * std::cos(target_point.theta);
    if (std::abs(denominator) > 1e-6) {
      double length_a =
          ((target_point.x - init_point.x) * std::sin(target_point.theta) -
           (target_point.y - init_point.y) * std::cos(target_point.theta)) /
          denominator;
      double length_b =
          ((target_point.x - init_point.x) * std::sin(init_point.theta) -
           (target_point.y - init_point.y) * std::cos(init_point.theta)) /
          denominator;
      if (length_a * length_b < 0) {
        double diff_theta = planning_math::NormalizeAngle(target_point.theta -
                                                          init_point.theta);
        double turn_radius = std::min(std::abs(length_a), std::abs(length_b)) *
                             std::abs(std::tan(diff_theta / 2));
        if (turn_radius >= CarParams::GetInstance()->min_turn_radius) {
          check_endsegent_len =
              std::max(std::min(length_b -
                                    CarParams::GetInstance()->min_turn_radius /
                                        std::abs(std::tan(diff_theta / 2)) -
                                    0.1,
                                check_endsegent_len),
                       0.0);
        }
      }
    }
  }

  check_endsegent_len = cfg->default_check_endsegent_len_;
  Pose2D ideal_mid_pose;
  int ideal_mid_direction = check_endsegent_len >= 0 ? 1 : -1;
  ideal_mid_pose.x =
      input_.target_state.path_point.x -
      std::cos(input_.target_state.path_point.theta) * check_endsegent_len;
  ideal_mid_pose.y =
      input_.target_state.path_point.y -
      std::sin(input_.target_state.path_point.theta) * check_endsegent_len;
  ideal_mid_pose.theta = input_.target_state.path_point.theta;
  TrajectoryPoint ideal_mid_state(ideal_mid_pose.x, ideal_mid_pose.y, 0.0,
                                  ideal_mid_pose.theta, 0,
                                  cfg->default_v_ * ideal_mid_direction, 0.0);
  OpenspaceDeciderOutput mid_input(input_);
  mid_input.target_state = ideal_mid_state;

  return mid_input;
}

OpenspaceDeciderOutput genReverseProblem(const OpenspaceDeciderOutput &input_) {
  OpenspaceDeciderOutput output(input_);
  auto &init_state = output.init_state;
  auto &target_state = output.target_state;
  std::swap(init_state, target_state);
  init_state.v = -init_state.v;
  target_state.v = -target_state.v;
  return output;
}

bool assembleSearchProblem(const StrategyParams *cfg, SbpResult &sbp_result) {
  if (cfg->enable_revserse_search_) {
    std::reverse(sbp_result.x.begin(), sbp_result.x.end());
    std::reverse(sbp_result.y.begin(), sbp_result.y.end());
    std::reverse(sbp_result.phi.begin(), sbp_result.phi.end());
    if (msquare::CarParams::GetInstance()->car_config.lon_config.use_sop_algorithm) {
      std::reverse(sbp_result.steer.begin(), sbp_result.steer.end());
    }
  }
  extendSbpResult(sbp_result, cfg->default_check_endsegent_len_,
                  cfg->default_dt_ * cfg->default_v_);

  return true;
}

void regulateVelocity(const StrategyParams *cfg,
                      std::vector<TrajectoryPoint> &traj) {
  const double LOW_SPEED_VELOCITY = 0.21;
  if (traj.empty()) {
    return;
  }

  modify_trajpoints_v(
      traj, TrajectoryOptimizerConfig::GetInstance()->param_max_speed_forward,
      TrajectoryOptimizerConfig::GetInstance()->param_max_speed_reverse);

  size_t traj_size = traj.size();

  apply_trajpoint_s(traj); // will be used to decide low_speed_start_idx
  double low_speed_length = cfg->low_speed_endsegment_len_;
  double total_length = traj.back().path_point.s;
  size_t low_speed_start_idx = 0;
  for (; low_speed_start_idx < traj_size; low_speed_start_idx++) {
    if (total_length - traj.at(low_speed_start_idx).path_point.s <=
        low_speed_length)
      break;
  }
  // refine ending segment size
  size_t end_seg_size = 0;
  int direction = 0;
  auto iter = traj.rbegin();
  for (; iter != traj.rend(); ++iter) {
    if (std::fabs(direction) < 1e-8 && std::fabs(iter->v) > 1e-5) {
      direction = iter->v > 0 ? 1 : -1;
      break;
    }
  }
  for (; iter != traj.rend(); ++iter) {
    if (direction * iter->v > 0) {
      end_seg_size++;
    } else {
      break;
    }
  }

  size_t end_seg_start_idx = traj_size - end_seg_size;
  low_speed_start_idx = std::max(end_seg_start_idx, low_speed_start_idx);
  for (size_t i = low_speed_start_idx; i < traj_size; ++i) {
    double &v_i = traj.at(i).v;
    v_i = std::min(std::max(-LOW_SPEED_VELOCITY, v_i), LOW_SPEED_VELOCITY);
  }
  // smooth end segment speed
  smooth_v(traj, cfg->default_dt_, cfg->default_a_, end_seg_start_idx,
           traj_size, true);
}

bool judgeHeadTowards(double start_theta, double end_theta) {
  double s_n_t = NormalizeAngle(start_theta);
  double e_n_t = NormalizeAngle(end_theta);
  double delta_t = e_n_t - s_n_t;
  if (delta_t > 0 && delta_t <= M_PI) {
    return false;
  } else {
    return true;
  }
}

planning_math::Box2d setGearComfortZone(const Pose2D &start_pose,
                                        bool forward) {
  const double half_parking_space_width = 1.35;
  const double half_parking_space_length = 2.85;
  const double comfort_zone_width = 5.0;
  const double comfort_zone_length = 7.0 + half_parking_space_length;
  Box2d gear_comfort_zone;
  if (forward) {
    Pose2D temp_c((comfort_zone_width - half_parking_space_width) / 2.0,
                  (half_parking_space_length + comfort_zone_length) / 2.0, 0.0);
    Pose2D zone_center = tf2d_inv(start_pose, temp_c);
    gear_comfort_zone = Box2d({zone_center.x, zone_center.y}, zone_center.theta,
                              half_parking_space_width + comfort_zone_width,
                              comfort_zone_length + half_parking_space_length);
  } else {
    Pose2D temp_c(-(comfort_zone_width - half_parking_space_width) / 2.0,
                  (half_parking_space_length + comfort_zone_length) / 2.0,
                  M_PI);
    Pose2D zone_center = tf2d_inv(start_pose, temp_c);
    gear_comfort_zone = Box2d({zone_center.x, zone_center.y}, zone_center.theta,
                              half_parking_space_width + comfort_zone_width,
                              comfort_zone_length + half_parking_space_length);
  }
  return gear_comfort_zone;
}

// std::vector<TrajectoryPoint> getStraightTraj(const Pose2D &ego_pose,
//                                              const Pose2D &target_pose,
//                                              SbpResult &sbp_result) {
//   using namespace planning_math;
//   const double STEP_SIZE = 0.25;
//   const double FORWARD_VELOCITY = 1.0;
//   const double BACKWARD_VELOCITY = 0.75;

//   // project ego point to target_pose
//   Pose2D ego_pose_in_target = tf2d(target_pose, ego_pose);
//   Pose2D start_pose_in_target(ego_pose_in_target.x, 0, 0);
//   Pose2D start_pose = tf2d_inv(target_pose, start_pose_in_target);
//   Vec2d move(target_pose.x - ego_pose.x, target_pose.y - ego_pose.y);
//   double default_velocity =
//       Vec2d::CreateUnitVec2d(ego_pose.theta).InnerProd(move) >= 0
//           ? FORWARD_VELOCITY
//           : -BACKWARD_VELOCITY;

//   Vec2d start_point(start_pose.x, start_pose.y);
//   Vec2d target_point(target_pose.x, target_pose.y);
//   // generate revese line to ensure ending point gap
//   LineSegment2d center_line(target_point, start_point);
//   std::vector<Vec2d> points;
//   sbp_result.x.clear();
//   sbp_result.y.clear();
//   sbp_result.phi.clear();
//   sbp_result.v.clear();
//   for (double s = 0; s < center_line.length(); s += STEP_SIZE) {
//     points.push_back(center_line.getPoint(s));
//   }
//   points.push_back(center_line.getPoint(center_line.length()));

//   std::reverse(points.begin(), points.end());

//   std::vector<TrajectoryPoint> straight_traj;
//   for (const Vec2d &point : points) {
//     TrajectoryPoint tp;
//     tp.path_point.theta = target_pose.theta;
//     tp.path_point.x = point.x();
//     tp.path_point.y = point.y();
//     tp.v = default_velocity;
//     straight_traj.push_back(tp);

//     sbp_result.x.push_back(point.x());
//     sbp_result.y.push_back(point.y());
//     sbp_result.phi.push_back(target_pose.theta);
//     sbp_result.v.push_back(default_velocity);
//   }
//   sbp_result.a = std::vector<double>(sbp_result.x.size(), 0);
//   sbp_result.steer = std::vector<double>(sbp_result.x.size(), 0);
//   sbp_result.accumulated_s = std::vector<double>(sbp_result.x.size(), 0);
//   sbp_result.wheel_base_offset = std::vector<double>(sbp_result.x.size(), 0);
//   return straight_traj;
// }

bool genOpenspacePath(const SearchBasedPlannerPtr sbp,
                      const OpenspaceDeciderOutput &input,
                      SbpResult &sbp_result, SearchProcessDebug *sp_debug) {
  auto init_state = input.init_state;
  auto target_state = input.target_state;

  std::vector<SbpObstaclePtr> sbp_obs_ptrs;
  // sbp_obs_ptrs.push_back(std::make_shared<GridObsManager>(input));

  sbp->setStartNode(std::make_shared<SearchNode>(
      init_state.path_point.x, init_state.path_point.y,
      init_state.path_point.theta, init_state.v));
  sbp->setTargetNode(std::make_shared<SearchNode>(
      target_state.path_point.x, target_state.path_point.y,
      target_state.path_point.theta, target_state.v));
  sbp->setStartRange(init_state.sigma_x,
                     init_state.sigma_y,
                     init_state.sigma_yaw);
  sbp->setTargetRange(target_state.sigma_x,
                      target_state.sigma_y,
                      target_state.sigma_yaw);
  sbp_obs_ptrs.push_back(
      std::make_shared<SbpObstacleLine>(input.obstacle_lines));
  sbp_obs_ptrs.push_back(std::make_shared<SbpObstaclePoint>(input.points));

  // TODO: Jinwei: map lines as low in parallel, and ignored otherwise
  if (input.lines.size() > 0) {
    SbpObstaclePtr map_lines = std::make_shared<SbpObstacleLine>(input.lines);
    map_lines->setHeightType(ObstacleHeightType::MAP_LINE);
    sbp_obs_ptrs.push_back(map_lines);
  }

  // sbp_obs_ptrs.push_back(std::make_shared<SbpMapLine>(input_.lines));
  // for (const planning_math::Box2d &box : input_.obstacle_boxs) {
  //   if(input_.map_boundary.HasOverlap(box))
  //   {
  //     sbp_obs_ptrs.push_back(std::make_shared<SbpObstacleBox>(box));
  //   }
  // }
  // TODO@huangzhengming: not implemented
  // sbp_obs_ptrs.push_back(std::make_shared<SbpMapLine>(input_.obstacle_boxs));

  // bool forward = judgeHeadTowards(init_state.path_point.theta,
  //                                 target_state.path_point.theta);
  // Pose2D start_pose(init_state.path_point.x, init_state.path_point.y,
  //                   NormalizeAngle(init_state.path_point.theta - M_PI_2));
  // planning_math::Box2d gear_comfort_zone =
  //     setGearComfortZone(start_pose, forward);
  // sbp_obs_ptrs.push_back(
  //     std::make_shared<SbpComfortGearZone>(gear_comfort_zone));

  if (sbp->Plan(sbp_obs_ptrs, sp_debug)) {
    sbp_result = sbp->getResult();
    if (sbp_result.wheel_base_offset.empty()) {
      sbp_result.wheel_base_offset =
          std::vector<double>(sbp_result.x.size(), 0);
    }
  } else {
    sbp_result.status = sbp->getResult().status;
  }
  return !sbp_result.x.empty();
}

void getObsTangentLine(planning_math::LineSegment2d line,
                       planning_math::Box2d parking_space_box,
                       const OpenspaceDeciderOutput &osd) {
  const double DEFAULT_ROAD_WIDTH = 5.0;

  double min_y = DEFAULT_ROAD_WIDTH / 2.0;
}

void extendSbpResult(SbpResult &sbp_result, double s_total, double ds) {
  if (sbp_result.x.empty()) {
    return;
  }
  Pose2D mid_pose{sbp_result.x.back(), sbp_result.y.back(),
                  sbp_result.phi.back()};
  double diff_cos = std::cos(mid_pose.theta);
  double diff_sin = std::sin(mid_pose.theta);
  if (s_total < 0) {
    diff_cos = -diff_cos;
    diff_sin = -diff_sin;
  }
  double abs_s_total = std::abs(s_total);
  for (double s = ds; s < abs_s_total; s += ds) {
    Pose2D pp(mid_pose.x + s * diff_cos, mid_pose.y + s * diff_sin,
              mid_pose.theta);
    sbp_result.x.push_back(pp.x);
    sbp_result.y.push_back(pp.y);
    sbp_result.phi.push_back(pp.theta);
  }
  Pose2D pp(mid_pose.x + abs_s_total * diff_cos,
            mid_pose.y + abs_s_total * diff_sin, mid_pose.theta);
  sbp_result.x.push_back(pp.x);
  sbp_result.y.push_back(pp.y);
  sbp_result.phi.push_back(pp.theta);
}

} // namespace parking
} // namespace msquare