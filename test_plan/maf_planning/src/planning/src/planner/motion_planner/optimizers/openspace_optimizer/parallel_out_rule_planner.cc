#include "planner/motion_planner/optimizers/openspace_optimizer/parallel_out_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/reeds_shepp_path.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/rule_math.h"

namespace msquare {

ParallelOutRulePlanner::ParallelOutRulePlanner(
    const parking::OpenspaceDeciderOutput &odo) {
  LOCAL_LOG(LOCAL_INFO, ""); // empty line
  LOCAL_LOG(LOCAL_INFO, ""); // empty line
  LOCAL_LOG(LOCAL_INFO, "***************************************");
  LOCAL_LOG(LOCAL_INFO, "************ ENTER PARALLEL PLAN ******");
  LOCAL_LOG(LOCAL_INFO, "***************************************");

  fillPara(odo, para_);
  initScenario(odo);
}

void ParallelOutRulePlanner::fillPara(
    const parking::OpenspaceDeciderOutput &odo, clothoid::Parameter &para) {
  para_.width = VehicleParam::Instance()->width_wo_rearview_mirror;
  para_.length = VehicleParam::Instance()->length;
  para_.width_with_rearview = VehicleParam::Instance()->width;
  para_.obs_inflation =
      std::max(HybridAstarConfig::GetInstance()->inflation_for_points_, 0.0);
  para_.lat = std::max(0.0, CarParams::GetInstance()->lat_inflation());
  para_.lon = std::max(0.0, CarParams::GetInstance()->lon_inflation());
  para_.min_radius = CarParams::GetInstance()->min_turn_radius;
  para_.max_steering_angle = CarParams::GetInstance()->max_steer_angle;
  para_.step = msquare::HybridAstarConfig::GetInstance()->step_size;
  para_.wheel_radius = VehicleParam::Instance()->wheel_rolling_radius;
  para_.wheel_base = VehicleParam::Instance()->wheel_base;
  para_.steering_speed = 270;
  para_.steering_angle_ratio = CarParams::GetInstance()->steer_ratio;
  para_.init_steer_angle = std::min(300.0, para_.max_steering_angle);
  para_.back_light_len =
      CarParams::GetInstance()->car_config.car_only_config.back_light_len;
  para_.back_light_height =
      CarParams::GetInstance()->car_config.car_only_config.back_light_height;
  radiu_used_ = para_.min_radius;
  para_.front_corner_width = VehicleParam::Instance()->bumper_length / 2.0;
  para_.front_corner_length = VehicleParam::Instance()->front_edge_to_center -
                              VehicleParam::Instance()->light_to_front_edge;

  /* */
  para_.straight_lat = 0.2;
  para_.straight_lon = 0.3;
  para_.side_safe_distance = 0.3;

  /* define by general */
  para_.max_alpha =
      1 / para_.min_radius /
      (para_.turning_speed * (para_.max_steering_angle / para_.steering_speed));
  para_.radical_alpha = 1 / para_.min_radius /
                        (para_.radical_turning_speed *
                         (para_.max_steering_angle / para_.steering_speed));

  para_.front_to_rear = VehicleParam::Instance()->front_edge_to_center;
  para_.back_to_rear = VehicleParam::Instance()->back_edge_to_center;
  para_.rear_to_center =
      0.5 * std::abs(para_.front_to_rear - para_.back_to_rear);
  para_.virtual_wall_interval = 0.1;
  para_.invalid_lat_offset = 0.6;

  impl_.half_width = 0.5 * para_.width;
  impl_.last_v = odo.apa_meta_state.is_valid ? odo.apa_meta_state.last_v
                                             : odo.target_state.v;
  impl_.block_direc =
      std::abs(impl_.last_v) > 1e-6 ? (0 < impl_.last_v ? 1 : -1) : 0;
  impl_.safe_radius =
      para_.wheel_base / (std::tan(para_.init_steer_angle /
                                   para_.steering_angle_ratio * M_PI / 180.0));
  impl_.min_block_theta = para_.min_block_len / para_.min_radius;
  impl_.tline_height_thres = 0.0;

  LOCAL_LOG(LOCAL_INFO, "length:%.3f, width:%.3f", para_.length, para_.width);
  LOCAL_LOG(LOCAL_INFO, "alpha:%f min radius:%.3f block direc:%d",
            para_.max_alpha, para_.min_radius, impl_.block_direc);
  LOCAL_LOG(LOCAL_INFO, "lat:%.3f, lon:%.3f, fr:%.3f, br:%.3f, rr:%.3f",
            para_.lat, para_.lon, para_.front_to_rear, para_.back_to_rear,
            para_.rear_to_center);

  csg_.getRawShape(Pose2D(0.0, 0.0, 0.0), impl_.zero_6_corners, 0.0, 0.0);
  impl_.c4_add_theta =
      std::atan2(impl_.zero_6_corners.at(4).x(),
                 para_.min_radius - impl_.zero_6_corners.at(4).y());
  impl_.c4_to_center =
      std::hypot(impl_.zero_6_corners.at(4).x(),
                 para_.min_radius - impl_.zero_6_corners.at(4).y());
  LOCAL_LOG(LOCAL_INFO, "c4_add_theta:%.3f, c4_to_center:%.3f",
            impl_.c4_add_theta, impl_.c4_to_center);
}

void ParallelOutRulePlanner::initScenario(
    const parking::OpenspaceDeciderOutput &input) {
  const parking::APAMetaState &apa_meta_state = input.apa_meta_state;
  LOCAL_LOG(LOCAL_INFO, "apa_meta_state.is_valid = %d",
            apa_meta_state.is_valid);

  // get local frame
  local_frame_pose_ =
      apa_meta_state.is_valid
          ? Pose2D(apa_meta_state.meta_pose_x, apa_meta_state.meta_pose_y,
                   apa_meta_state.meta_pose_theta)
          : Pose2D(input.init_state.path_point.x, input.init_state.path_point.y,
                   input.init_state.path_point.theta);

  // get local poses
  init_pose_ = planning_math::tf2d(local_frame_pose_,
                                   Pose2D(input.target_state.path_point.x,
                                          input.target_state.path_point.y,
                                          input.target_state.path_point.theta));
  target_pose_ = planning_math::tf2d(local_frame_pose_,
                                     Pose2D(input.init_state.path_point.x,
                                            input.init_state.path_point.y,
                                            input.init_state.path_point.theta));
  LOCAL_LOG(LOCAL_INFO, "target_pose_ x:%.6f, y:%.6f, z:%.6f", target_pose_.x,
            target_pose_.y, target_pose_.theta);
  LOCAL_LOG(LOCAL_INFO, "init_pose_ x:%.6f, y:%.6f, z:%.6f", init_pose_.x,
            init_pose_.y, init_pose_.theta);
  LOCAL_LOG(LOCAL_INFO, "local_frame_pose_ x:%.6f, y:%.6f, z:%.6f",
            local_frame_pose_.x, local_frame_pose_.y, local_frame_pose_.theta);

  msquare::planning_math::Vec2d local_center = msquare::planning_math::tf2d(
      local_frame_pose_, input.map_boundary.center());
  impl_.is_on_left = (local_center.y() < 0.0 ? true : false);
  double parallel_direc = apa_meta_state.is_valid
                              ? apa_meta_state.parallel_direc
                              : input.target_state.steer;
  if (std::abs(parallel_direc - 1.0) < 1e-6) {
    park_half_ = true;
    impl_.is_on_left = false;
  } else if (std::abs(parallel_direc + 1.0) < 1e-6) {
    park_half_ = true;
    impl_.is_on_left = true;
  } else {
    park_half_ = false;
  }

  LOCAL_LOG(LOCAL_INFO, "is_on_left:%d, park_half_:%d", impl_.is_on_left,
            park_half_);

  auto local_slot_left_bound = planning_math::tf2d(
      local_frame_pose_, input.T_lines.road_lower_left_bound);

  auto local_slot_right_bound =
      planning_math::tf2d(local_frame_pose_, input.T_lines.slot_right_bound);

  std::vector<planning_math::LineSegment2d> obs_lines_;
  std::vector<planning_math::Vec2d> obs_pts_;

  std::vector<planning_math::LineSegment2d> step_lines_;
  std::vector<planning_math::Vec2d> step_pts_;
  if (impl_.is_on_left) {
    init_pose_ = localMirrorX(init_pose_);

    target_pose_ = localMirrorX(target_pose_);

    impl_.left_corner = localMirrorX(local_slot_right_bound.end());
    impl_.right_corner = localMirrorX(local_slot_left_bound.end());

    for (auto line : input.obstacle_lines) {
      auto local_line = planning_math::tf2d(local_frame_pose_, line);
      obs_lines_.push_back(localMirrorX(local_line));
    }

    // get points
    for (auto line : obs_lines_) {
      obs_pts_.push_back(line.start());
      obs_pts_.push_back(line.end());
    }
    for (auto point : input.points) {
      obs_pts_.emplace_back(
          localMirrorX(planning_math::tf2d(local_frame_pose_, point)));
    }

    for (auto line : input.lines) {
      auto local_line = planning_math::tf2d(local_frame_pose_, line);
      step_lines_.push_back(localMirrorX(local_line));
    }
    for (auto obstacle : step_lines_) {
      step_pts_.push_back(obstacle.start());
      step_pts_.push_back(obstacle.end());
    }
    for (auto point : input.step_points) {
      step_pts_.emplace_back(
          localMirrorX(planning_math::tf2d(local_frame_pose_, point)));
    }
  } else {
    impl_.left_corner = local_slot_left_bound.end();
    impl_.right_corner = local_slot_right_bound.end();

    for (auto line : input.obstacle_lines) {
      auto local_line = planning_math::tf2d(local_frame_pose_, line);
      obs_lines_.push_back(local_line);
    }

    // get points
    for (auto obstacle : obs_lines_) {
      obs_pts_.push_back(obstacle.start());
      obs_pts_.push_back(obstacle.end());
    }
    for (auto point : input.points) {
      obs_pts_.emplace_back(planning_math::tf2d(local_frame_pose_, point));
    }

    for (auto line : input.lines) {
      auto local_line = planning_math::tf2d(local_frame_pose_, line);
      step_lines_.push_back(local_line);
    }
    for (auto obstacle : step_lines_) {
      step_pts_.push_back(obstacle.start());
      step_pts_.push_back(obstacle.end());
    }
    for (auto point : input.step_points) {
      step_pts_.emplace_back(planning_math::tf2d(local_frame_pose_, point));
    }
  }
  // add virtual bottom line
  planning_math::LineSegment2d bottom_line = planning_math::LineSegment2d(
      planning_math::Vec2d(impl_.left_corner.x(), -impl_.half_width - 1.0),
      planning_math::Vec2d(impl_.right_corner.x(), -impl_.half_width - 1.0));
  obs_lines_.push_back(bottom_line);

  refactorRightCorner(obs_pts_, step_pts_);
  impl_.slot_height = std::max(-impl_.half_width, impl_.right_corner.y());
  impl_.is_tline_lower = impl_.slot_height < impl_.tline_height_thres;

  LOCAL_LOG(LOCAL_INFO, "is left:%d, slot_height: %.3f", impl_.is_on_left,
            impl_.slot_height);
  auto local_upper_line =
      planning_math::tf2d(local_frame_pose_, input.T_lines.road_upper_bound);
  impl_.upper_height = local_upper_line.min_y();
  csg_ = clothoid::CollisionShapeGenerator(para_);
  checker_ = clothoid::CollisionChecker(csg_);
  checker_.setData(obs_lines_, obs_pts_, step_lines_, step_pts_);
}

bool ParallelOutRulePlanner::Plan(const std::vector<SbpObstaclePtr> &obs_ptrs,
                                  parking::SearchProcessDebug *sp_debug) {
  // park_half_ = true;
  if (!park_half_) {
    LOCAL_LOG(LOCAL_INFO, "failed as not in park half");
    return false;
  }

  if (!checkStartPose()) {
    return false;
  }

  InSlotPathSegemnts path_segments;

  if (!planParkoutHalf(path_segments)) {
    std::vector<std::pair<double, std::vector<double>>> backdis_radius_kvs =
        getBackdisRadiusKv();
    bool is_plan_diff_radius_success = false;
    for (auto p : backdis_radius_kvs) {
      if (planDiffRadius(p.first, p.second, path_segments)) {
        is_plan_diff_radius_success = true;
        break;
      }
    }
    if (!is_plan_diff_radius_success) {
      LOCAL_LOG(LOCAL_INFO, "failed to plan an valid path");
      return false;
    }
  }
  if (path_segments.key_pose.size() < 2) {
    LOCAL_LOG(LOCAL_INFO,
              "return as key pose size:%lu, which is smaller than 2",
              path_segments.key_pose.size());
  }
  generatePath(radiu_used_, path_segments);
  return true;
}
bool ParallelOutRulePlanner::isEscapableForLowerTline(const Pose2D &key_pose,
                                                      Pose2D &escape_pose,
                                                      bool &valid,
                                                      const bool is_init) {
  // quadrant limit
  if (!inQuadrant1Out(key_pose.theta)) {
    LOCAL_LOG(LOCAL_INFO, "not in quadrant 1");
    valid = false;
    return false;
  }
  if (!isParkoutForLowerTline(key_pose, para_.min_radius, is_init, escape_pose,
                              valid)) {
    return false;
  }
  LOCAL_LOG(LOCAL_INFO, "isEscapable !");
  return true;
}
bool ParallelOutRulePlanner::isEscapableStraight(const Pose2D &key_pose,
                                                 Pose2D &escape_pose,
                                                 bool &valid,
                                                 const bool is_init) {
  // quadrant limit
  if (!inQuadrant1Out(key_pose.theta)) {
    LOCAL_LOG(LOCAL_INFO, "not in quadrant 1");
    valid = false;
    return false;
  }
  if (key_pose.theta < 0.1) {
    LOCAL_LOG(LOCAL_INFO, "keypose theta is too small");
    valid = false;
    return false;
  }

  double forward_dis = checker_.moveForward(key_pose, para_.lat, para_.lon,
                                            clothoid::ShapeType::OCTAGON);
  LOCAL_LOG(LOCAL_INFO, "forward dis:%.3f", forward_dis);

  if (forward_dis < para_.min_block_len) {
    LOCAL_LOG(LOCAL_INFO, "straight path is tiny");
    valid = false;
    return false;
  }

  std::vector<planning_math::Vec2d> polygon_points;
  csg_.getRawShape(key_pose, polygon_points, 0.0, 0.0);
  double diff_height = impl_.slot_height - polygon_points[4].y();
  LOCAL_LOG(LOCAL_INFO, "diff_height:%.3f", diff_height);
  if (diff_height < 0) {
    LOCAL_LOG(LOCAL_INFO, "diff_height < 0");
    valid = false;
    return false;
  }

  double needed_dis = diff_height / std::sin(key_pose.theta);
  LOCAL_LOG(LOCAL_INFO, "needed_dis:%.3f", needed_dis);

  if (needed_dis > forward_dis) {
    LOCAL_LOG(LOCAL_INFO, "needed_dis > forward_dis");
    valid = false;
    return false;
  }

  if (needed_dis < para_.min_block_len && forward_dis < para_.min_block_len) {
    valid = false;
    LOCAL_LOG(LOCAL_INFO,
              "return as needed_dis is smaller than min_block_len %.3f",
              para_.min_block_len);
    return false;
  }

  if (needed_dis < para_.min_block_len &&
      forward_dis > para_.min_block_len - 1e-6) {
    needed_dis = forward_dis;
  }
  if (needed_dis < para_.min_block_len &&
      forward_dis > 2 * para_.min_block_len) {
    needed_dis = 2 * para_.min_block_len;
  }

  if (needed_dis > 3.0) {
    LOCAL_LOG(LOCAL_INFO, "finally needed_dis:%.3f is larger than 3 m",
              needed_dis);
    valid = false;
    return false;
  }
  LOCAL_LOG(LOCAL_INFO, "finally needed_dis:%.3f", needed_dis);
  escape_pose.theta = key_pose.theta;
  escape_pose.x = key_pose.x + needed_dis * cos(key_pose.theta);
  escape_pose.y = key_pose.y + needed_dis * sin(key_pose.theta);
  LOCAL_LOG(LOCAL_INFO, "isEscapable by straight !");
  valid = true;
  return true;
}

bool ParallelOutRulePlanner::isEscapable(const Pose2D &key_pose,
                                         Pose2D &escape_pose, bool &valid,
                                         const bool is_init) {
  LOCAL_LOG(LOCAL_INFO, "is_init:%d", is_init);
  if (impl_.is_tline_lower) {
    return isEscapableForLowerTline(key_pose, escape_pose, valid, is_init);
  }
  // quadrant limit
  if (!inQuadrant1Out(key_pose.theta)) {
    LOCAL_LOG(LOCAL_DEBUG, "not in quadrant 1, key_pose.theta = %f",
              key_pose.theta);
    valid = false;
    return false;
  }

  if (!isParkoutSuccess(key_pose, para_.min_radius, is_init, escape_pose,
                        valid)) {
    return false;
  }
  LOCAL_LOG(LOCAL_INFO, "isEscapable !");
  return true;
}

bool ParallelOutRulePlanner::planParkoutHalf(
    InSlotPathSegemnts &path_segments) {
  auto &key_pose_list = path_segments.key_pose;
  key_pose_list.push_back(target_pose_);
  Pose2D temp_pose;
  bool valid = false; // if no escable, the temp_pose
  if (impl_.block_direc < 1e-6 &&
      isEscapable(target_pose_, temp_pose, valid, true)) {
    LOCAL_LOG(LOCAL_INFO, "plan successfully: just turn");
    key_pose_list.push_back(temp_pose);
    return true;
  }

  // back lon  which is larger than rotate lon
  double escape_lon = std::min(para_.lon + 0.05, 0.4);
  double escape_lat = std::min(para_.lat + 0.02, 0.19);
  bool is_init = false;
  bool has_backed = false;

  Pose2D turning_pose;
  bool is_backward_success =
      getBackwardPose(target_pose_, para_.min_radius, escape_lat, escape_lon,
                      is_init, turning_pose);
  if (!is_backward_success || impl_.block_direc < -1e-6) {
    turning_pose = target_pose_;
    is_init = true;
  } else {
    key_pose_list.push_back(turning_pose);
    has_backed = true;
  }

  if (impl_.block_direc > 1e-6 && !has_backed) {
    if (!getRightBackwardPose(target_pose_, para_.min_radius, is_init,
                              turning_pose)) {
      LOCAL_LOG(LOCAL_DEBUG, "forward block and cant move back");
      return false;
    } else {
      key_pose_list.push_back(turning_pose);
      has_backed = true;
      is_init = false;
    }
  }

  LOCAL_LOG(LOCAL_INFO, "back dis:%.6f", back_max);

  int i = 0;
  while (
      !isEscapable(turning_pose, temp_pose, valid, i == 0 ? is_init : false) &&
      valid && i++ < 5) {
    Pose2D next_pose;
    if (!getLeftForwardPose(turning_pose, para_.min_radius, is_init,
                            next_pose)) {
      LOCAL_LOG(LOCAL_INFO, "getLeftForwardPose rotate theta is too small");
      valid = false;
      break;
    }
    key_pose_list.push_back(next_pose);
    Pose2D end_pose;
    if (!getRightBackwardPose(next_pose, para_.min_radius, false, end_pose)) {
      LOCAL_LOG(LOCAL_INFO, "getRightBackwardPose rotate theta is too small");
      valid = false;
      break;
    }
    key_pose_list.push_back(end_pose);
    turning_pose = end_pose;
  }

  LOCAL_LOG(LOCAL_INFO, "rotate time i:%d", i);
  if (i > 5) {
    return false;
  }

  if (!valid &&
      !isEscapableStraight(key_pose_list.back(), temp_pose, valid, is_init)) {
    return false;
  }

  key_pose_list.push_back(temp_pose);
  LOCAL_LOG(LOCAL_INFO, "plan successfully");
  return true;
}

bool ParallelOutRulePlanner::planDiffRadius(
    const double backward_max, const std::vector<double> &radiu_list,
    InSlotPathSegemnts &path_segments) {
  auto &key_pose_list = path_segments.key_pose;
  key_pose_list.clear();
  key_pose_list.push_back(target_pose_);
  bool valid = false;
  double real_back_max = checker_.moveBackward(
      target_pose_, para_.lat, para_.lon, clothoid::ShapeType::OCTAGON);
  double back_real = std::max(0.0, real_back_max - 1e-3);
  double back_max = std::min(back_real, backward_max);
  if (impl_.block_direc < -1e-6) { // block_direc, 0: 前后都能走, 1: 前面不能走,
    back_max = 0.0;                // -1: 后面不能走, 只限制第一把
  }
  double sin_ego = std::sin(target_pose_.theta);
  double cos_ego = std::cos(target_pose_.theta);
  Pose2D turning_pose(target_pose_.x - back_max * cos_ego,
                      target_pose_.y - back_max * sin_ego, target_pose_.theta);
  bool is_init = false;
  bool has_backed = false;
  if (back_max < para_.min_block_len || impl_.block_direc < -1e-6) {
    turning_pose = target_pose_;
    is_init = true;
  } else {
    key_pose_list.push_back(turning_pose);
    has_backed = true;
  }
  if (impl_.block_direc > 1e-6 && !has_backed) {
    LOCAL_LOG(LOCAL_DEBUG, "forward block and cant move back");
    return false;
  }

  for (int i = 0; i < radiu_list.size(); i++) {
    Pose2D next_pose;
    double min_block_theta = para_.min_block_len / radiu_list[i];
    if (impl_.is_tline_lower) {
      if (isParkoutForLowerTline(turning_pose, radiu_list[i], is_init,
                                 next_pose, valid)) {
        if (std::abs(next_pose.theta - turning_pose.theta) >= min_block_theta) {
          key_pose_list.push_back(next_pose);
          radiu_used_ = radiu_list[i];
          return true;
        }
      }
    }
    if (isParkoutSuccess(turning_pose, radiu_list[i], is_init, next_pose,
                         valid)) {
      if (std::abs(next_pose.theta - turning_pose.theta) >= min_block_theta) {
        key_pose_list.push_back(next_pose);
        radiu_used_ = radiu_list[i];
        return true;
      }
    }
  }
  return false;
}

bool ParallelOutRulePlanner::isParkoutSuccess(const Pose2D &start_pose,
                                              const double radiu_rotate,
                                              const bool is_init,
                                              Pose2D &escape_pose,
                                              bool &valid) {
  Pose2D next_pose;
  next_pose =
      checker_.rotateMaxPose(start_pose, 1, 1, clothoid::ShapeType::OCTAGON,
                             radiu_rotate, is_init, para_.lat, para_.lon);

  // cal some params
  double c4_add_theta =
      std::atan2(impl_.zero_6_corners.at(4).x(),
                 radiu_rotate - impl_.zero_6_corners.at(4).y());
  double c4_to_center =
      std::hypot(impl_.zero_6_corners.at(4).x(),
                 radiu_rotate - impl_.zero_6_corners.at(4).y());
  double min_block_theta = para_.min_block_len / radiu_rotate;

  //  right corner point
  double ry = impl_.slot_height;
  double rx = impl_.right_corner.x();
  double max_theta = std::max(0.0, next_pose.theta - start_pose.theta);
  max_theta = std::min(M_PI_4, max_theta);

  // get c1 center
  double c1x = start_pose.x - 1.0 * radiu_rotate * sin(start_pose.theta);
  double c1y = start_pose.y + 1.0 * radiu_rotate * cos(start_pose.theta);
  if (c1y - ry < 0) {
    LOCAL_LOG(LOCAL_DEBUG, "return as right corner is too high");
    valid = false;
    return false;
  }

  double dis_to_key = std::hypot(rx - c1x, ry - c1y);
  std::vector<planning_math::Vec2d> obs_poses_in_distance;
  planning_math::Vec2d obs_poses_center(rx, ry);
  double obs_distance = para_.side_safe_distance;
  // to get obs points in safe ditance
  for (planning_math::Vec2d obs_pose : checker_.obs_pts_) {
    double obs_pose_dis = obs_poses_center.DistanceTo(obs_pose);
    if (obs_pose_dis < obs_distance) {
      obs_poses_in_distance.push_back(obs_pose);
    }
  }
  // to deal obs wall
  for (planning_math::LineSegment2d obs_line : checker_.obs_lines_) {
    planning_math::Vec2d obs_line_start = obs_line.start();
    planning_math::Vec2d obs_line_end = obs_line.end();
    double obs_line_start_dis = obs_poses_center.DistanceTo(obs_line_start);
    double obs_line_end_dis = obs_poses_center.DistanceTo(obs_line_end);
    if (obs_line_start_dis < obs_distance || obs_line_end_dis < obs_distance) {
      obs_poses_in_distance.push_back(obs_poses_center);
    }
  }
  if (obs_poses_in_distance.empty()) {
    dis_to_key += obs_distance;
  } else {
    for (planning_math::Vec2d obs_pose_in_dis : obs_poses_in_distance) {
      dis_to_key =
          std::hypot(c1x - obs_pose_in_dis.x(), c1y - obs_pose_in_dis.y());
    }
  }

  double right_corner_theta = std::atan2(rx - c1x, c1y - ry);
  double needed_theta = right_corner_theta - c4_add_theta -
                        start_pose.theta; // parkout needed theta
  // double right_to_center = std::hypot(rx - c1x, ry - c1y);
  double right_to_center = dis_to_key;
  LOCAL_LOG(LOCAL_INFO,
            "right_corner_theta:%.6f c4_add_theta:%.6f ego theta:%.6f",
            right_corner_theta, c4_add_theta, next_pose.theta);
  LOCAL_LOG(LOCAL_INFO, "max_theta:%.6f needed_theta:%.6f ", max_theta,
            needed_theta);
  LOCAL_LOG(LOCAL_INFO, "right_to_center:%.6f c4_to_center:%.6f ",
            right_to_center, c4_to_center);
  if (needed_theta < 0.0) {
    LOCAL_LOG(LOCAL_INFO, "return as theta is negative");
    valid = false;
    return false;
  }

  if (max_theta < needed_theta) {
    escape_pose = next_pose;
    valid = true;
    LOCAL_LOG(LOCAL_INFO, "return as theta is not enough");
    return false;
  }

  if (right_to_center - c4_to_center < para_.side_safe_distance) {
    double t_theta = start_pose.theta + needed_theta;
    escape_pose.theta = t_theta;
    escape_pose.x = c1x + radiu_rotate * sin(t_theta);
    escape_pose.y = c1y - radiu_rotate * cos(t_theta);
    valid = true;
    LOCAL_LOG(LOCAL_INFO,
              "return as right_to_center - impl_.c4_to_center < "
              "side_safe_distance, which is %.3f",
              para_.side_safe_distance);
    return false;
  }

  if (needed_theta < min_block_theta && max_theta < min_block_theta) {
    valid = false;
    LOCAL_LOG(LOCAL_INFO,
              "return as needed_theta is smaller than min_block_theta %.3f",
              min_block_theta);
    return false;
  }

  if (needed_theta < min_block_theta && max_theta > 2 * min_block_theta) {
    LOCAL_LOG(LOCAL_INFO, "use min_block_theta*2");
    needed_theta = 2 * min_block_theta;
  }

  if (needed_theta < min_block_theta && max_theta > min_block_theta - 1e-6) {
    LOCAL_LOG(LOCAL_INFO, "use min_block_theta");
    needed_theta = min_block_theta;
  }

  valid = true;
  double t_theta = start_pose.theta + needed_theta;
  escape_pose.theta = t_theta;
  escape_pose.x = c1x + radiu_rotate * sin(t_theta);
  escape_pose.y = c1y - radiu_rotate * cos(t_theta);
  LOCAL_LOG(LOCAL_INFO, "isParkoutSuccess !");
  return true;
}

bool ParallelOutRulePlanner::isParkoutForLowerTline(const Pose2D &start_pose,
                                                    const double radiu_rotate,
                                                    const bool is_init,
                                                    Pose2D &escape_pose,
                                                    bool &valid) {
  Pose2D next_pose;
  next_pose =
      checker_.rotateMaxPose(start_pose, 1, 1, clothoid::ShapeType::OCTAGON,
                             radiu_rotate, is_init, para_.lat, para_.lon);

  // cal some params
  double c4_add_theta =
      std::atan2(impl_.zero_6_corners.at(4).x(),
                 radiu_rotate - impl_.zero_6_corners.at(4).y());
  double c4_to_center =
      std::hypot(impl_.zero_6_corners.at(4).x(),
                 radiu_rotate - impl_.zero_6_corners.at(4).y());
  double min_block_theta = para_.min_block_len / radiu_rotate;

  //  right corner point
  double ry = impl_.slot_height;
  double rx = impl_.right_corner.x();
  double max_theta = std::max(0.0, next_pose.theta - start_pose.theta);
  max_theta = std::min(M_PI_2, max_theta);

  std::vector<planning_math::Vec2d> polygon_points;
  csg_.getRawShape(start_pose, polygon_points, 0.0, 0.0);
  if (polygon_points.at(4).y() > impl_.half_width) {
    escape_pose = start_pose;
    valid = true;
    LOCAL_LOG(LOCAL_INFO, "current pose is Escapable");
    return true;
  }

  csg_.getRawShape(next_pose, polygon_points, 0.0, 0.0);
  if (polygon_points.at(4).y() < impl_.half_width) {
    escape_pose = next_pose;
    valid = true;
    LOCAL_LOG(LOCAL_INFO, "return as no needed theta");
    return false;
  }

  // get c1 center
  double c1x = start_pose.x - 1.0 * radiu_rotate * sin(start_pose.theta);
  double c1y = start_pose.y + 1.0 * radiu_rotate * cos(start_pose.theta);
  double height_diff = c1y - impl_.half_width;
  if (height_diff < 0) {
    LOCAL_LOG(LOCAL_INFO, "return as right corner is too high");
    valid = false;
    return false;
  }

  double target_c4_theta = std::acos(height_diff / c4_to_center);
  double needed_theta = target_c4_theta - c4_add_theta - start_pose.theta;
  LOCAL_LOG(LOCAL_INFO,
            "isParkoutForLowerTline max_theta:%.6f needed_theta:%.6f ",
            max_theta, needed_theta);

  if (needed_theta < 0.0) {
    LOCAL_LOG(LOCAL_INFO, "return as theta is negative");
    valid = false;
    return false;
  }

  if (max_theta < needed_theta) {
    escape_pose = next_pose;
    valid = true;
    LOCAL_LOG(LOCAL_INFO, "return as theta is not enough");
    return false;
  }

  if (needed_theta < min_block_theta && max_theta < min_block_theta) {
    valid = false;
    LOCAL_LOG(LOCAL_INFO,
              "return as needed_theta is smaller than min_block_theta %.3f",
              min_block_theta);
    return false;
  }

  if (needed_theta < min_block_theta && max_theta > 2 * min_block_theta) {
    LOCAL_LOG(LOCAL_INFO, "use min_block_theta * 2");
    needed_theta = 2 * min_block_theta;
  }
  if (needed_theta < min_block_theta && max_theta > min_block_theta - 1e-6) {
    LOCAL_LOG(LOCAL_INFO, "use min_block_theta");
    needed_theta = min_block_theta;
  }
  valid = true;
  double t_theta = start_pose.theta + needed_theta;
  escape_pose.theta = t_theta;
  escape_pose.x = c1x + radiu_rotate * sin(t_theta);
  escape_pose.y = c1y - radiu_rotate * cos(t_theta);
  LOCAL_LOG(LOCAL_INFO, "isParkoutForLowerTline !");
  return true;
}

void ParallelOutRulePlanner::generatePath(const double radiu_used,
                                          InSlotPathSegemnts &path_segments) {
  LOCAL_LOG(LOCAL_INFO, "radiu_used %.3f", radiu_used);
  auto local_rule_path =
      path_segments.interpolatePath(para_.step, para_.step / radiu_used);
  std::vector<Pose2D> global_rule_path;
  double path_pose_size = local_rule_path.size();
  global_rule_path.reserve(path_pose_size);

  if (impl_.is_on_left) {
    for (int i = 0; i < path_pose_size; ++i) {
      global_rule_path.push_back(planning_math::tf2d_inv(
          local_frame_pose_, localMirrorX(local_rule_path.at(i))));
    }
  } else {
    for (int i = 0; i < path_pose_size; ++i) {
      global_rule_path.push_back(
          planning_math::tf2d_inv(local_frame_pose_, local_rule_path.at(i)));
    }
  }

  result_.clear();
  for (auto pt : global_rule_path) {
    result_.x.push_back(pt.x);
    result_.y.push_back(pt.y);
    result_.phi.push_back(pt.theta);
  }
}

bool ParallelOutRulePlanner::isInSlot(const Pose2D &pose) {
  if (pose.y > impl_.slot_height) {
    return false;
  }
  return true;
  std::vector<planning_math::Vec2d> corners(6);
  csg_.getRawShape(pose, corners, para_.lat, para_.lon);
  if (corners.at(4).y() > impl_.slot_height) {
    return false;
  }

  return true;
}

void ParallelOutRulePlanner::Update(
    const planning_math::Box2d map_boundary,
    const std::vector<planning_math::LineSegment2d> &map) {}

SbpResult ParallelOutRulePlanner::getResult() { return result_; }
std::vector<Pose2D> ParallelOutRulePlanner::getSearchPoints() {
  return key_points_;
}

void ParallelOutRulePlanner::refactorRightCorner(
    std::vector<planning_math::Vec2d> &obs_pts,
    std::vector<planning_math::Vec2d> &step_pts) {
  double nx = impl_.right_corner.x() + 1.0;
  double ny = std::max(impl_.right_corner.y() - para_.width, -impl_.half_width);
  LOCAL_LOG(LOCAL_INFO, "before refactor right corner x: %.3f y: %.3f", nx, ny);

  double min_y = 0.0;
  double max_y = 1.5 * para_.width;
  double min_x = para_.front_to_rear;
  double max_x = std::max(nx + 1.0, para_.front_to_rear + 3.0);

  double px, py;
  for (auto &p : obs_pts) {
    px = p.x();
    py = p.y();
    if (px < min_x || px > max_x || py < min_y || py > max_y) {
      continue;
    }
    if (px < nx) {
      nx = px;
    }
    if (py > ny) {
      ny = py;
    }
  }
  for (auto &p : step_pts) {
    px = p.x();
    py = p.y();
    if (px < min_x || px > max_x || py < min_y || py > max_y) {
      continue;
    }
    if (px < nx) {
      nx = px;
    }
    if (py > ny) {
      ny = py;
    }
  }

  impl_.right_corner.set_x(nx);
  impl_.right_corner.set_y(ny);
  LOCAL_LOG(LOCAL_INFO, "after refactor right corner x: %.3f y: %.3f", nx, ny);
}

bool ParallelOutRulePlanner::checkStartPose() {
  if (!checker_.checkTerminalPose(target_pose_)) {
    LOCAL_LOG(LOCAL_INFO, "start pose is not valid.");
    result_.status = SbpStatus::START_INFEASIBLE;
    return false;
  }

  return true;
}

bool ParallelOutRulePlanner::getLeftForwardPose(const Pose2D &start_pose,
                                                const double radiu_rotate,
                                                const bool is_init,
                                                Pose2D &next_pose) {
  double theta_epsilon = para_.min_block_len / radiu_rotate - 1e-6;
  next_pose =
      checker_.rotateMaxPose(start_pose, 1, 1, clothoid::ShapeType::OCTAGON,
                             radiu_rotate, false, para_.lat, para_.lon);
  if (!getRealRotatePose(start_pose, radiu_rotate, next_pose)) {
    return false;
  }
  Pose2D end_pose =
      checker_.rotateMaxPose(next_pose, -1, -1, clothoid::ShapeType::OCTAGON,
                             radiu_rotate, false, para_.lat, para_.lon);
  double deta_theta = std::abs(next_pose.theta - start_pose.theta);
  if (deta_theta < theta_epsilon) {
    return false;
  }
  double rotate_theta = std::abs(end_pose.theta - next_pose.theta);
  if (rotate_theta < theta_epsilon) {
    double start_pose_center_x =
        start_pose.x - radiu_rotate * std::sin(start_pose.theta);
    double start_pose_center_y =
        start_pose.y + radiu_rotate * std::cos(start_pose.theta);
    double step_theta = deta_theta - theta_epsilon;
    int i = 0;
    while (i < 5) {
      double alpha = start_pose.theta + step_theta;
      next_pose.x = start_pose_center_x + radiu_rotate * std::sin(alpha);
      next_pose.y = start_pose_center_y - radiu_rotate * std::cos(alpha);
      next_pose.theta = alpha;
      end_pose = checker_.rotateMaxPose(
          next_pose, -1, -1, clothoid::ShapeType::OCTAGON, radiu_rotate, false,
          para_.lat, para_.lon);
      if (std::abs(end_pose.theta - next_pose.theta) > theta_epsilon) {
        return true;
      }
      i += 1;
      step_theta -= i * theta_epsilon;
      if (step_theta < theta_epsilon) {
        return false;
      }
    }
    return false;
  }
  return true;
}

bool ParallelOutRulePlanner::getRealRotatePose(const Pose2D &start_pose,
                                               const double radiu_rotate,
                                               Pose2D &next_pose) {
  double theta_epsilon = para_.min_block_len / radiu_rotate - 1e-6;
  double rot_center_x =
      start_pose.x - radiu_rotate * std::sin(start_pose.theta);
  double rot_center_y =
      start_pose.y + radiu_rotate * std::cos(start_pose.theta);
  planning_math::Vec2d rot_center(rot_center_x, rot_center_y);
  std::vector<planning_math::Vec2d> next_pose_corners;
  csg_.getRawShape(next_pose, next_pose_corners, 0.0, 0.0);
  double slot_right_height = impl_.right_corner.y();
  if (slot_right_height < 0.0) {
    slot_right_height = impl_.half_width;
  }
  if (next_pose_corners[5].y() > slot_right_height) {
    double l2 = rot_center.DistanceTo(next_pose_corners[5]);
    planning_math::Vec2d next_pose_vec2d(next_pose.x, next_pose.y);
    double l3 = next_pose_vec2d.DistanceTo(next_pose_corners[5]);
    double cos_constant_theta =
        (radiu_rotate * radiu_rotate + l2 * l2 - l3 * l3) /
        (2 * radiu_rotate * l2);
    double constant_theta = std::acos(cos_constant_theta);
    double cos_alpha = (rot_center_y - slot_right_height) / l2;
    double alpha = std::acos(cos_alpha);
    double deta_theta = alpha - start_pose.theta - constant_theta;
    double real_theta = start_pose.theta + deta_theta;
    double theta_max = next_pose.theta;

    if (deta_theta > theta_epsilon) {
      next_pose.x = rot_center_x + radiu_rotate * std::sin(real_theta);
      next_pose.y = rot_center_y - radiu_rotate * std::cos(real_theta);
      next_pose.theta = real_theta;
    } else {
      if (theta_max > 2 * theta_epsilon) {
        real_theta = start_pose.theta + 2 * theta_epsilon;
        next_pose.x = rot_center_x + radiu_rotate * std::sin(real_theta);
        next_pose.y = rot_center_y - radiu_rotate * std::cos(real_theta);
        next_pose.theta = real_theta;
      } else if (theta_max > theta_epsilon) {
        real_theta = start_pose.theta + theta_epsilon;
        next_pose.x = rot_center_x + radiu_rotate * std::sin(real_theta);
        next_pose.y = rot_center_y - radiu_rotate * std::cos(real_theta);
        next_pose.theta = real_theta;
      } else {
        LOCAL_LOG(LOCAL_INFO, "getRealRotatePose rotate theta is too small");
        return false;
      }
    }
  }
  return true;
}

bool ParallelOutRulePlanner::getRightBackwardPose(const Pose2D &start_pose,
                                                  const double radiu_rotate,
                                                  const bool is_init,
                                                  Pose2D &next_pose) {
  double theta_epsilon = para_.min_block_len / radiu_rotate - 1e-6;
  next_pose =
      checker_.rotateMaxPose(start_pose, -1, -1, clothoid::ShapeType::OCTAGON,
                             radiu_rotate, false, para_.lat, para_.lon);
  Pose2D end_pose =
      checker_.rotateMaxPose(next_pose, 1, 1, clothoid::ShapeType::OCTAGON,
                             radiu_rotate, false, para_.lat, para_.lon);
  double deta_theta = std::abs(next_pose.theta - start_pose.theta);
  if (deta_theta < theta_epsilon) {
    return false;
  }
  double rotate_theta = end_pose.theta - next_pose.theta;
  if (rotate_theta < theta_epsilon) {
    double start_pose_center_x =
        start_pose.x + radiu_rotate * std::sin(start_pose.theta);
    double start_pose_center_y =
        start_pose.y - radiu_rotate * std::cos(start_pose.theta);
    double step_theta = deta_theta - theta_epsilon;
    int i = 0;
    while (i < 5) {
      double alpha = start_pose.theta + step_theta;
      next_pose.x = start_pose_center_x - radiu_rotate * std::sin(alpha);
      next_pose.y = start_pose_center_y + radiu_rotate * std::cos(alpha);
      next_pose.theta = alpha;
      end_pose =
          checker_.rotateMaxPose(next_pose, 1, 1, clothoid::ShapeType::OCTAGON,
                                 radiu_rotate, false, para_.lat, para_.lon);
      if (end_pose.theta - next_pose.theta > theta_epsilon) {
        return true;
      }
      i += 1;
      step_theta -= i * theta_epsilon;
      if (step_theta < theta_epsilon) {
        return false;
      }
    }
    return false;
  }
  return true;
}

bool ParallelOutRulePlanner::getBackwardPose(const Pose2D &start_pose,
                                             const double radiu_rotate,
                                             const double lat, const double lon,
                                             const bool is_init,
                                             Pose2D &next_pose) {
  double theta_epsilon = para_.min_block_len / radiu_rotate - 1e-6;
  double real_back_max =
      checker_.moveBackward(start_pose, lat, lon, clothoid::ShapeType::OCTAGON);
  double back_real = std::max(0.0, real_back_max - 1e-3);
  double back_max = std::min(back_real, 3.0);
  if (back_max < para_.min_block_len) {
    return false;
  }
  double sin_ego = std::sin(start_pose.theta);
  double cos_ego = std::cos(start_pose.theta);
  next_pose.x = start_pose.x - back_max * cos_ego;
  next_pose.y = start_pose.y - back_max * sin_ego;
  next_pose.theta = start_pose.theta;
  Pose2D end_pose =
      checker_.rotateMaxPose(next_pose, 1, 1, clothoid::ShapeType::OCTAGON,
                             radiu_rotate, false, para_.lat, para_.lon);
  double rotate_theta = end_pose.theta - next_pose.theta;
  if (rotate_theta < theta_epsilon) {
    int i = 1;
    while (i < 6) {
      double len = back_max - para_.min_block_len * i;
      if (len < para_.min_block_len) {
        return false;
      }
      next_pose.x = start_pose.x - len * cos_ego;
      next_pose.y = start_pose.y - len * sin_ego;
      next_pose.theta = start_pose.theta;
      end_pose =
          checker_.rotateMaxPose(next_pose, 1, 1, clothoid::ShapeType::OCTAGON,
                                 radiu_rotate, false, para_.lat, para_.lon);
      if (end_pose.theta - next_pose.theta > theta_epsilon) {
        return true;
      }
      i += 1;
    }
    return false;
  }
  return true;
}

std::vector<std::pair<double, std::vector<double>>>
ParallelOutRulePlanner::getBackdisRadiusKv() {
  double back_max_1 = 3.0;
  std::vector<double> radiu_list_1 = {
      2.0 * para_.min_radius, 3.0 * para_.min_radius, 4.0 * para_.min_radius,
      5.0 * para_.min_radius};
  std::pair<double, std::vector<double>> p1(back_max_1, radiu_list_1);

  double back_max_2 = 5.0;
  std::vector<double> radiu_list_2 = {
      5.0 * para_.min_radius, 6.0 * para_.min_radius, 8.0 * para_.min_radius,
      10.0 * para_.min_radius};
  std::pair<double, std::vector<double>> p2(back_max_2, radiu_list_2);

  std::vector<std::pair<double, std::vector<double>>> backdis_radius_kvs;
  backdis_radius_kvs.push_back(p1);
  backdis_radius_kvs.push_back(p2);
  return backdis_radius_kvs;
}

} // namespace msquare