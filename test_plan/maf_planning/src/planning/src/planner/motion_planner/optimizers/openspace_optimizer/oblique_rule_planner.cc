#include "planner/motion_planner/optimizers/openspace_optimizer/oblique_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/rule_planner_util.h"

namespace msquare {

ObliqueRulePlanner::ObliqueRulePlanner(
    const parking::OpenspaceDeciderOutput &odo) {
  inflation_for_points_ = std::max(
      HybridAstarConfig::GetInstance()->inflation_for_points_ + 1e-6, 1e-6);
  half_width = VehicleParam::Instance()->width_wo_rearview_mirror / 2 +
               CarParams::GetInstance()->lat_inflation();
  front_to_rear = VehicleParam::Instance()->front_edge_to_center +
                  CarParams::GetInstance()->lat_inflation();
  back_to_rear = VehicleParam::Instance()->back_edge_to_center +
                 CarParams::GetInstance()->lat_inflation();
  veh_min_r = CarParams::GetInstance()->min_turn_radius;
  front_corner_width = VehicleParam::Instance()->bumper_length / 2.0;
  front_corner_length = VehicleParam::Instance()->front_edge_to_center -
                        VehicleParam::Instance()->light_to_front_edge;

  extractKeyInfo(odo);
  if (!isInnerSideVehicle(odo)) {
    std::reverse(buffer_zigzag_pairs_.begin(), buffer_zigzag_pairs_.end());
  }

  l_1 = veh_min_r - half_width;
  l_2 = std::hypot(veh_min_r + half_width, front_corner_length);
  gamma_4 = atan2(front_to_rear, veh_min_r + half_width);

  slot_width_ = floor((P_1_.x() - P_0_.x()) * 10) / 10;
  if (slot_width_ < 2.5) {
    slot_width_ = 2.5;
  } else if (slot_width_ > 2.8) {
    slot_width_ = 2.8;
  }

  for (double sw_x = sweet_x_min_; sw_x < sweet_x_max_; sw_x += 0.5) {
    for (double sw_y = sweet_y_max_; sw_y > sweet_y_min_; sw_y -= 0.2) {
      Pose2D sw_pose(sw_x, sw_y, 0);
      if (checkSinglePose(sw_pose, true)) {
        sweet_spot_.emplace_back(sw_x, sw_y, 0);
      }
    }
  }
}

bool ObliqueRulePlanner::checkSinglePose(const Pose2D &pose, bool use_lon) {
  static planning_math::Polygon2d ego_polygon;
  static std::vector<planning_math::Vec2d> ego_corners(6);
  static std::vector<planning_math::LineSegment2d> lines(6);

  double extra_lon_inflation =
      use_lon ? std::max(CarParams::GetInstance()->lon_inflation() -
                             CarParams::GetInstance()->lat_inflation(),
                         0.0)
              : -CarParams::GetInstance()->lat_inflation();
  double extra_lat_infaltion =
      use_lon ? 0.0 : -CarParams::GetInstance()->lat_inflation();
  getEPCorners(pose, ego_corners, lines, extra_lon_inflation,
               extra_lat_infaltion);

  ego_polygon.update(ego_corners);
  for (auto &line_segment : obstacles) {
    if (ego_polygon.HasOverlap(line_segment)) {
      return false;
    }
  }

  for (auto &p : points_of_obstacles) {
    if (ego_polygon.IsPointIn(p)) {
      return false;
    }
  }

  return true;
}

SbpResult ObliqueRulePlanner::getResult() { return result_; }
std::vector<Pose2D> ObliqueRulePlanner::getSearchPoints() {
  return key_points_;
}

void ObliqueRulePlanner::Update(
    const planning_math::Box2d map_boundary,
    const std::vector<planning_math::LineSegment2d> &map) {}

bool ObliqueRulePlanner::isMidPlan() {
  const double MAX_HEADING_DIFF = M_PI_2 - 0.3;
  // const double MIN_HEADING_DIFF = -0.1;
  double heading_diff = target_pose.theta - init_pose.theta;
  if (/* MIN_HEADING_DIFF < heading_diff && */ heading_diff <
      MAX_HEADING_DIFF) {
    return true;
  } else {
    return false;
  }
}

Pose2D ObliqueRulePlanner::calcExpectedPoseBoth(const Pose2D &start_pose,
                                                int steer_direction,
                                                int travel_direction) {
  if (steer_direction == -1 && travel_direction == -1) {
    return calcExpectedPoseReverse(start_pose, -1, -1, veh_min_r);
  } else if (steer_direction == 1 && travel_direction == 1) {
    return calcExpectedPose(start_pose, 1, 1);
  } else {
    // invalid argument
  }
  return Pose2D{};
}

bool ObliqueRulePlanner::planMidway(const Pose2D &mid_pose, bool is_reverse) {
  key_points_.clear();
  result_.clear();
  key_points_.push_back(mid_pose);
  Pose2D turning_pose(mid_pose);
  int i = 0;
  int steer_direction = 1;
  int travel_direction = 1;
  if (is_reverse) {
    steer_direction = -steer_direction;
    travel_direction = -travel_direction;
  }
  while (i++ < 8) {
    Pose2D next_pose =
        calcNextPose(obstacles, points_of_obstacles, init_pose, turning_pose,
                     steer_direction, steer_direction, veh_min_r);
    auto expected_pose =
        calcExpectedPoseBoth(turning_pose, steer_direction, steer_direction);
    if (travel_direction < 0) {
      auto limited_expected_pose_front =
          calcNextPose(obstacles, points_of_obstacles, init_pose, expected_pose,
                       1, -1, veh_min_r);
      if (next_pose.theta > expected_pose.theta &&
          expected_pose.theta > turning_pose.theta &&
          limited_expected_pose_front.theta < target_pose.theta) {
        key_points_.push_back(expected_pose);
        double middle_y =
            expected_pose.y + veh_min_r * cos(expected_pose.theta);
        if (middle_y < target_pose.y) {
          return false;
        }
        key_points_.emplace_back(target_pose.x, middle_y, target_pose.theta);
        key_points_.push_back(target_pose);
        break;
      } else if (next_pose.theta > target_pose.theta &&
                 next_pose.x < target_pose.x) {
        double bigger_radius = (target_pose.x - next_pose.x) /
                               (1 - cos(next_pose.theta - target_pose.theta));
        if (fabs(bigger_radius) >= veh_min_r) {
          limited_expected_pose_front =
              calcNextPose(obstacles, points_of_obstacles, init_pose, next_pose,
                           1, -1, bigger_radius);
          if (limited_expected_pose_front.theta < target_pose.theta) {
            key_points_.push_back(next_pose);
            double middle_y =
                next_pose.y -
                bigger_radius * sin(next_pose.theta - target_pose.theta);
            if (middle_y < target_pose.y) {
              return false;
            }
            key_points_.emplace_back(target_pose.x, middle_y,
                                     target_pose.theta);
            key_points_.push_back(target_pose);
            break;
          }
        }
      } else if (next_pose.theta > target_pose.theta &&
                 next_pose.x > target_pose.x &&
                 turning_pose.theta < target_pose.theta) {
        double denominator = 5.0001;
        int i = 1;
        for (; i <= denominator; ++i) {
          double pose_theta = (i / denominator) * turning_pose.theta +
                              (1 - i / denominator) * target_pose.theta;
          double x_O = turning_pose.x + veh_min_r * sin(turning_pose.theta);
          double y_O = turning_pose.y - veh_min_r * cos(turning_pose.theta);
          planning_math::Vec2d center(x_O, y_O);

          double pose_x = x_O - veh_min_r * sin(pose_theta);
          double pose_y = y_O + veh_min_r * cos(pose_theta);
          Pose2D loose_pose(pose_x, pose_y, pose_theta);
          double turning_radius =
              (pose_x - target_pose.x) / (1 - sin(pose_theta));
          if (turning_radius < veh_min_r) {
            continue;
          }
          auto new_limit_pose =
              calcNextPose(obstacles, points_of_obstacles, init_pose,
                           loose_pose, -1, -1, turning_radius);
          if (new_limit_pose.theta < target_pose.theta) {
            continue;
          }

          double middle_y = pose_y - turning_radius * cos(pose_theta);
          if (middle_y < target_pose.y) {
            continue;
          }
          key_points_.emplace_back(loose_pose);
          key_points_.emplace_back(target_pose.x, middle_y, target_pose.theta);
          key_points_.push_back(target_pose);
          break;
        }
        if (i <= denominator) {
          break;
        }
      }
    } else {
      auto limited_expected_pose_back =
          calcNextPose(obstacles, points_of_obstacles, init_pose, expected_pose,
                       -1, -1, veh_min_r);
      if (next_pose.theta > expected_pose.theta &&
          expected_pose.theta > turning_pose.theta + 0.1 / veh_min_r &&
          limited_expected_pose_back.theta > target_pose.theta) {
        key_points_.push_back(expected_pose);
        double middle_y =
            expected_pose.y - veh_min_r * cos(expected_pose.theta);
        if (middle_y < target_pose.y) {
          return false;
        }
        key_points_.emplace_back(target_pose.x, middle_y, target_pose.theta);
        key_points_.push_back(target_pose);
        break;
      } else if (next_pose.theta > expected_pose.theta &&
                 next_pose.theta < target_pose.theta) {
        double denominator = 5.0001;
        int i = 1;
        for (; i <= denominator; ++i) {
          double pose_theta = (i / denominator) * turning_pose.theta +
                              (1 - i / denominator) * expected_pose.theta;
          double x_O = next_pose.x - veh_min_r * sin(next_pose.theta);
          double y_O = next_pose.y + veh_min_r * cos(next_pose.theta);
          planning_math::Vec2d center(x_O, y_O);

          double pose_x = x_O + veh_min_r * sin(pose_theta);
          double pose_y = y_O - veh_min_r * cos(pose_theta);
          Pose2D interrupt_pose(pose_x, pose_y, pose_theta);
          double turning_radius =
              (pose_x - target_pose.x) / (1 - sin(pose_theta));
          if (turning_radius < veh_min_r) {
            continue;
          }
          auto new_limit_pose =
              calcNextPose(obstacles, points_of_obstacles, init_pose,
                           interrupt_pose, -1, -1, turning_radius);
          if (new_limit_pose.theta < target_pose.theta) {
            continue;
          }

          double middle_y = pose_y - turning_radius * cos(pose_theta);
          if (middle_y < target_pose.y) {
            continue;
          }
          key_points_.emplace_back(interrupt_pose);
          key_points_.emplace_back(target_pose.x, middle_y, target_pose.theta);
          key_points_.push_back(target_pose);
          break;
        }
        if (i <= denominator) {
          break;
        }
      }
    }
    if (next_pose.y < target_pose.y ||
        next_pose.theta > target_pose.theta + M_PI_4 ||
        next_pose.theta < init_pose.theta - M_PI_4) {
      return false;
    }
    key_points_.push_back(next_pose);
    turning_pose = next_pose;
    steer_direction = -steer_direction;
    travel_direction = -travel_direction;
  }
  return i < 9;
}

bool ObliqueRulePlanner::planRestartWay(const Pose2D &mid_pose) {
  key_points_.clear();
  result_.clear();
  // get zero heading pose
  Pose2D right_front_limit_pose = calcNextPose(
      obstacles, points_of_obstacles, init_pose, mid_pose, -1, 1, veh_min_r);
  if (right_front_limit_pose.theta > target_pose.theta - M_PI_2) {
    return false;
  }
  Pose2D restart_pose(mid_pose.x + veh_min_r * sin(mid_pose.theta),
                      mid_pose.y + veh_min_r * (1 - cos(mid_pose.theta)),
                      target_pose.theta - M_PI_2);
  Pose2D original_init_pose = mid_pose;
  init_pose = restart_pose;
  points_of_obstacles.push_back(P_1_);
  for (auto buffer_pair : buffer_zigzag_pairs_) {
    if (PlanWithBuffer(buffer_pair.first, buffer_pair.second - 1)) {
      key_points_.insert(key_points_.begin(), original_init_pose);
      return true;
    }
  }
  return false;
}

bool ObliqueRulePlanner::Plan(const std::vector<SbpObstaclePtr> &obs_ptrs,
                              parking::SearchProcessDebug *sp_debug) {
  if (fabs(init_pose.x - target_pose.x) < in_slot_x_ &&
      init_pose.y - target_pose.y < front_to_rear &&
      fabs(init_pose.theta - target_pose.theta) < in_slot_theta_) {
    if (init_pose.y - target_pose.y > 2.0 && init_vel_ < 0) {
      std::vector<Pose2D> path_to_fill{init_pose};
      if (PlanInSlotSub_1(path_to_fill) &&
          isRSPathSafe(obstacles, points_of_obstacles, init_pose,
                       path_to_fill)) {
        key_points_ = path_to_fill;
        extendKeyPointsToResult(
            msquare::HybridAstarConfig::GetInstance()->step_size);
        return true;
      }
    }
  }

  if ((fabs(init_pose.x - target_pose.x) < in_slot_x_ &&
       init_pose.y - target_pose.y < front_to_rear &&
       fabs(init_pose.theta - target_pose.theta) < in_slot_theta_) ||
      (fabs(init_pose.x - target_pose.x) < in_slot_x_ * 2 &&
       init_pose.y - target_pose.y < front_to_rear / 2)) {
    if (PlanInSlot()) {
      return true;
    }
  }

  if (is_on_left) {
    auto local_right_road_bound = MirrorInjection(planning_math::tf2d(
        local_frame_pose_, t_lines_.road_lower_right_bound));
    auto local_right_slot_bound = MirrorInjection(
        planning_math::tf2d(local_frame_pose_, t_lines_.slot_right_bound));
    obstacles.push_back(local_right_road_bound);
    obstacles.push_back(local_right_slot_bound);
    points_of_obstacles.push_back(local_right_road_bound.start());
    points_of_obstacles.push_back(local_right_road_bound.end());
    points_of_obstacles.push_back(local_right_slot_bound.start());
    points_of_obstacles.push_back(local_right_slot_bound.end());
  } else {
    auto local_left_road_bound =
        planning_math::tf2d(local_frame_pose_, t_lines_.road_lower_left_bound);
    auto local_left_slot_bound =
        planning_math::tf2d(local_frame_pose_, t_lines_.slot_left_bound);
    obstacles.push_back(local_left_road_bound);
    obstacles.push_back(local_left_slot_bound);
    points_of_obstacles.push_back(local_left_road_bound.start());
    points_of_obstacles.push_back(local_left_road_bound.end());
    points_of_obstacles.push_back(local_left_slot_bound.start());
    points_of_obstacles.push_back(local_left_slot_bound.end());
  }

  if (std::abs(init_vel_) > 1e-6) {
    if (!planMidway(init_pose, init_vel_ < 0) &&
        !planMidway2(init_pose, init_vel_ < 0)) {
      return false;
    }
    extendKeyPointsToResult(
        msquare::HybridAstarConfig::GetInstance()->step_size);
    return true;
  }

  points_of_obstacles.push_back(P_1_);
  for (auto buffer_pair : buffer_zigzag_pairs_) {
    if (PlanWithBuffer(buffer_pair.first, buffer_pair.second)) {
      extendKeyPointsToResult(
          msquare::HybridAstarConfig::GetInstance()->step_size);
      return true;
    }
  }
  return false;
}

bool ObliqueRulePlanner::planMidway2(const Pose2D &mid_pose, bool is_reverse) {
  double back_dist = calBackStraightDist(mid_pose);
  Pose2D back_pose = mid_pose;
  if (back_dist > 0.1) {
    double back_step = std::min(0.5, std::max(0.1, back_dist / 10));
    int i = 1;
    while (back_pose.x > target_pose.x - half_width &&
           back_step * i < back_dist) {
      back_pose.x = mid_pose.x + i * back_step * cos(mid_pose.theta + M_PI);
      back_pose.y = mid_pose.y + i * back_step * sin(mid_pose.theta + M_PI);
      back_pose.theta = mid_pose.theta;
      if (planMidway(back_pose, is_reverse)) {
        key_points_.insert(key_points_.begin(), mid_pose);
        return true;
      }
      i++;
    }
  }

  double forward_dist = calForwardStraightDist(mid_pose);
  Pose2D forward_pose = mid_pose;
  if (forward_dist > 0.1) {
    double forward_step = std::min(0.5, std::max(0.1, forward_dist / 10));
    int i = 1;
    while (forward_pose.x < target_pose.x + half_width &&
           forward_step * i < forward_dist) {
      forward_pose.x = mid_pose.x + i * forward_step * cos(mid_pose.theta);
      forward_pose.y = mid_pose.y + i * forward_step * sin(mid_pose.theta);
      forward_pose.theta = mid_pose.theta;
      if (planMidway(forward_pose, is_reverse)) {
        key_points_.insert(key_points_.begin(), mid_pose);
        return true;
      }
      i++;
    }
  }
  return false;
}

bool ObliqueRulePlanner::PlanInSlot() {
  if (init_pose.x < target_pose.x && init_pose.theta < target_pose.theta) {
    std::vector<Pose2D> path_to_fill{init_pose};
    if (PlanInSlotSub0(path_to_fill)) {
      std::vector<Pose2D> path_to_fill_without_end = path_to_fill;
      if (path_to_fill_without_end.size() >
          2) { // ignore last straigth line safe check
        path_to_fill_without_end.pop_back();
      }
      if (isRSPathSafe(obstacles, points_of_obstacles, init_pose,
                       path_to_fill_without_end)) {
        key_points_ = path_to_fill;
        extendKeyPointsToResult(
            msquare::HybridAstarConfig::GetInstance()->step_size);
        return true;
      }
    }
  }

  if (init_pose.x > target_pose.x && init_pose.theta > target_pose.theta) {
    auto general_start_pose = MirrorInjection(init_pose);
    std::vector<Pose2D> path_to_fill{general_start_pose};
    if (PlanInSlotSub0(path_to_fill)) {
      for (int i = 0; i < path_to_fill.size(); ++i) {
        path_to_fill[i] = MirrorInjection(path_to_fill[i]);
      }
      std::vector<Pose2D> path_to_fill_without_end = path_to_fill;
      if (path_to_fill_without_end.size() >
          2) { // ignore last straigth line safe check
        path_to_fill_without_end.pop_back();
      }
      if (isRSPathSafe(obstacles, points_of_obstacles, init_pose,
                       path_to_fill_without_end)) {
        key_points_ = path_to_fill;
        extendKeyPointsToResult(
            msquare::HybridAstarConfig::GetInstance()->step_size);
        return true;
      }
    }
  }

  key_points_.push_back(init_pose);

  Pose2D side_para_pose = init_pose;
  if (init_pose.theta < target_pose.theta) {
    side_para_pose.x += veh_min_r * (1 - sin(init_pose.theta));
    side_para_pose.y += veh_min_r * cos(init_pose.theta);
    side_para_pose.theta = target_pose.theta;
  } else {
    side_para_pose.x -=
        veh_min_r * (1 - cos(init_pose.theta - target_pose.theta));
    side_para_pose.y += veh_min_r * sin(init_pose.theta - target_pose.theta);
    side_para_pose.theta = target_pose.theta;
  }

  key_points_.push_back(side_para_pose);
  double dist = std::hypot(side_para_pose.x - init_pose.x,
                           side_para_pose.y - init_pose.y);
  if (dist < 0.1) {
    side_para_pose.y += 0.11;
    key_points_.push_back(side_para_pose);
  }

  Pose2D method_input = side_para_pose.x < target_pose.x
                            ? side_para_pose
                            : MirrorInjection(side_para_pose);
  std::vector<Pose2D> path_to_fill;
  for (double ratio = 1.0; ratio < 4.01; ratio += 0.5) {
    path_to_fill = {method_input};
    PlanInSlotSub1(path_to_fill, ratio);
    std::vector<Pose2D> path_to_fill_without_end = path_to_fill;
    if (path_to_fill_without_end.size() >
        2) { // ignore last straigth line safe check
      path_to_fill_without_end.pop_back();
    }
    if (side_para_pose.x < target_pose.x &&
        isRSPathSafe(obstacles, points_of_obstacles, init_pose,
                     path_to_fill_without_end)) {
      key_points_.insert(key_points_.end(), path_to_fill.begin(),
                         path_to_fill.end());
      extendKeyPointsToResult(
          msquare::HybridAstarConfig::GetInstance()->step_size);
      return true;
    } else if (side_para_pose.x > target_pose.x) {
      for (int i = 0; i < path_to_fill.size(); ++i) {
        path_to_fill[i] = MirrorInjection(path_to_fill[i]);
      }
      path_to_fill_without_end = path_to_fill;
      if (path_to_fill_without_end.size() >
          2) { // ignore last straigth line safe check
        path_to_fill_without_end.pop_back();
      }
      if (isRSPathSafe(obstacles, points_of_obstacles, init_pose,
                       path_to_fill_without_end)) {
        key_points_.insert(key_points_.end(), path_to_fill.begin(),
                           path_to_fill.end());
        extendKeyPointsToResult(
            msquare::HybridAstarConfig::GetInstance()->step_size);
        return true;
      }
    }
  }

  for (double ratio = 1.0; ratio < 4.01; ratio += 0.5) {
    path_to_fill = {method_input};
    PlanInSlotSub2(path_to_fill, ratio);
    std::vector<Pose2D> path_to_fill_without_end = path_to_fill;
    if (path_to_fill_without_end.size() >
        2) { // ignore last straigth line safe check
      path_to_fill_without_end.pop_back();
    }
    if (side_para_pose.x < target_pose.x &&
        isRSPathSafe(obstacles, points_of_obstacles, init_pose,
                     path_to_fill_without_end)) {
      key_points_.insert(key_points_.end(), path_to_fill.begin(),
                         path_to_fill.end());
      extendKeyPointsToResult(
          msquare::HybridAstarConfig::GetInstance()->step_size);
      return true;
    } else if (side_para_pose.x > target_pose.x) {
      for (int i = 0; i < path_to_fill.size(); ++i) {
        path_to_fill[i] = MirrorInjection(path_to_fill[i]);
      }
      path_to_fill_without_end = path_to_fill;
      if (path_to_fill_without_end.size() >
          2) { // ignore last straigth line safe check
        path_to_fill_without_end.pop_back();
      }
      if (isRSPathSafe(obstacles, points_of_obstacles, init_pose,
                       path_to_fill)) {
        key_points_.insert(key_points_.end(), path_to_fill.begin(),
                           path_to_fill.end());
        extendKeyPointsToResult(
            msquare::HybridAstarConfig::GetInstance()->step_size);
        return true;
      }
    }
  }
  key_points_.clear();
  return false;
}

bool ObliqueRulePlanner::PlanInSlotSub_1(std::vector<Pose2D> &path_to_fill) {
  Pose2D start_local = planning_math::tf2d(target_pose, path_to_fill.back());
  Pose2D target_local{0.0, 0.0, 0.0};
  double dx = start_local.x - target_local.x;
  double dy = start_local.y - target_local.y;
  double dtheta =
      planning_math::NormalizeAngle(start_local.theta - target_local.theta);
  std::vector<Pose2D> key_points{start_local};
  bool pre_requirement = dx > 0.0;
  bool success{false};
  const double kMathEpsilon = 1e-3;
  if (pre_requirement) {
    if (std::abs(1 - std::cos(dtheta)) < kMathEpsilon) {
      if (std::abs(dy) < kMathEpsilon) {
        key_points.emplace_back(target_local);
        success = true;
      }
    } else {
      double r = dy / (1 - std::cos(dtheta));
      if ((std::abs(r) > veh_min_r) &&
          (start_local.x - r * std::sin(dtheta) > target_local.x) &&
          (r * std::sin(dtheta) > 0.0)) {
        Pose2D via_point{start_local.x - r * sin(dtheta), target_local.y,
                         target_local.theta};
        key_points.emplace_back(via_point);
        key_points.emplace_back(target_local);
        success = true;
      }
    }
    if (!success) {
      std::vector<double> sgn_list{1.0, -1.0};
      for (auto sgn : sgn_list) {
        double x_o1 =
            start_local.x + sgn * veh_min_r * std::sin(start_local.theta);
        double y_o1 =
            start_local.y - sgn * veh_min_r * std::cos(start_local.theta);
        double y_o2 = target_local.y + sgn * veh_min_r;
        double delta_y = y_o1 - y_o2;
        if (std::abs(delta_y) < 2 * veh_min_r) {
          double theta1 =
              (std::acos(std::abs(delta_y) / (2 * veh_min_r))) * sgn;
          double x2 = x_o1 - sgn * veh_min_r * std::sin(theta1) -
                      sgn * veh_min_r * std::sin(theta1);
          if ((x2 > target_local.x) &&
              (x_o1 - sgn * veh_min_r * std::sin(theta1) > x2) &&
              (x_o1 - sgn * veh_min_r * std::sin(theta1) < start_local.x)) {
            key_points.emplace_back(
                Pose2D{x_o1 - sgn * veh_min_r * std::sin(theta1),
                       y_o1 + sgn * veh_min_r * std::cos(theta1), theta1});
            key_points.emplace_back(
                Pose2D{x2, target_local.y, target_local.theta});
            key_points.emplace_back(target_local);
            success = true;
            break;
          }
        }
      }
    }
  }
  for (int i = 1; i < key_points.size(); i++) {
    path_to_fill.push_back(planning_math::tf2d_inv(target_pose, key_points[i]));
  }
  return success;
}

bool ObliqueRulePlanner::PlanInSlotSub0(std::vector<Pose2D> &path_to_fill) {
  // one curve
  double delta_x = fabs(target_pose.x - path_to_fill.back().x);
  if (fabs(1 - sin(path_to_fill.back().theta)) < 1e-6) {
    return false;
  }
  double curve_radius = delta_x / (1 - sin(path_to_fill.back().theta));

  if (curve_radius < veh_min_r) {
    return false;
  }
  double highest_y =
      path_to_fill.back().y + curve_radius * cos(path_to_fill.back().theta);
  double dist = std::hypot(path_to_fill.back().x - target_pose.x,
                           path_to_fill.back().y - highest_y);
  path_to_fill.emplace_back(target_pose.x, highest_y, target_pose.theta);
  if (dist < 0.1) {
    path_to_fill.emplace_back(target_pose.x, highest_y + 0.1,
                              target_pose.theta);
  }
  path_to_fill.push_back(target_pose);
  return true;
}

void ObliqueRulePlanner::PlanInSlotSub1(std::vector<Pose2D> &path_to_fill,
                                        double ratio) {
  // CCS
  Pose2D side_pose = path_to_fill.back();
  double delta_x = fabs(target_pose.x - side_pose.x);
  double delta_y = fabs(target_pose.y + in_slot_higher_dist_ - side_pose.y);
  double minimum_r =
      (std::pow(delta_x, 2) + std::pow(delta_y, 2)) / (2 * delta_x);
  double radius = minimum_r < ratio * veh_min_r ? ratio * veh_min_r : minimum_r;

  double mid_x = (side_pose.x + target_pose.x) / 2;
  double half_dy =
      sqrt(std::pow(radius, 2) - std::pow(radius - delta_x / 2, 2));
  double mid_y = side_pose.y + half_dy;
  double single_curve_theta = acos((radius - delta_x / 2) / radius);

  path_to_fill.emplace_back(mid_x, mid_y, side_pose.theta - single_curve_theta);
  path_to_fill.emplace_back(target_pose.x, side_pose.y + 2 * half_dy,
                            target_pose.theta);
  path_to_fill.push_back(target_pose);
}

void ObliqueRulePlanner::PlanInSlotSub2(std::vector<Pose2D> &path_to_fill,
                                        double ratio) {
  // CCCCS or CSCCCCS
  Pose2D side_pose = path_to_fill.back();
  double start_y = side_pose.y;
  if (side_pose.y < target_pose.y + in_slot_higher_dist_) {
    start_y = target_pose.y + in_slot_higher_dist_;
    path_to_fill.emplace_back(side_pose.x, start_y, side_pose.theta);
  }

  double delta_x = fabs(target_pose.x - side_pose.x) / 2;
  double dist_y = sqrt(std::pow(ratio * veh_min_r, 2) -
                       std::pow(ratio * veh_min_r - delta_x / 2, 2));
  double single_curve_theta =
      acos((veh_min_r * ratio - delta_x / 2) / veh_min_r / ratio);
  path_to_fill.emplace_back(side_pose.x + delta_x / 2, start_y + dist_y,
                            side_pose.theta - single_curve_theta);
  path_to_fill.emplace_back(side_pose.x + delta_x, start_y + 2 * dist_y,
                            side_pose.theta);
  path_to_fill.emplace_back(target_pose.x - delta_x / 2, start_y + dist_y,
                            side_pose.theta + single_curve_theta);
  path_to_fill.emplace_back(target_pose.x, start_y, target_pose.theta);
  path_to_fill.push_back(target_pose);
}

bool ObliqueRulePlanner::PlanWithBuffer(double safe_buffer, int max_zigzag) {
  key_points_.clear();
  result_.clear();
  // all key_points need to checkCollision such as init_pose, turning_pose

  std::vector<Pose2D> head{init_pose};
  if (startWithStraightLine(head.back(), safe_buffer, max_zigzag)) {
    return true;
  } else {
    return startWithSweetPose(safe_buffer, max_zigzag);
  }
}

bool ObliqueRulePlanner::startWithStraightLine(const Pose2D &start_pose,
                                               double safe_buffer,
                                               int max_zigzag) {
  if (fabs(tan(start_pose.theta)) > 1e3) {
    return false;
  }
  key_points_.clear();
  key_points_.push_back(start_pose);
  planning_math::Vec2d center;
  Pose2D turning_pose = calcTurningPose(start_pose, safe_buffer, center);
  if (std::isnan(turning_pose.x) || std::isnan(turning_pose.y) ||
      std::isnan(turning_pose.theta)) {
    return false;
  }

  if (planOneShot(start_pose)) {
    return true;
  }

  if (!checkStraightLine(obstacles, points_of_obstacles, start_pose,
                         turning_pose)) {
    return false;
  }
  key_points_.push_back(turning_pose);

  if (calcCurvePath(turning_pose, max_zigzag)) {
    return true;
  }
  return false;
}

bool ObliqueRulePlanner::startWithSweetPose(double safe_buffer,
                                            int max_zigzag) {
  double ratio = 2.0;
  bool flag = false;
  Pose2D turning_pose;
  planning_math::Vec2d center;
  for (; ratio > 0.99; ratio -= 0.1) {
    for (auto sw_pose : sweet_spot_) {
      Pose2D slot_open_frame_ = Pose2D(0, slot_open_height_, slot_angle_);
      Pose2D actual_pose = planning_math::tf2d_inv(slot_open_frame_, sw_pose);

      key_points_.clear();
      key_points_.push_back(init_pose);

      if (calcAdjustPoses(actual_pose, ratio * veh_min_r)) {
        flag = true;
        break;
      }
    }
    if (flag) {
      break;
    }
  }
  if (!flag) {
    return false;
  }

  std::vector<Pose2D> planning_head = key_points_;
  if (startWithStraightLine(key_points_.back(), safe_buffer, max_zigzag)) {
    key_points_.insert(key_points_.begin(), planning_head.begin(),
                       planning_head.end());
    return true;
  }
  return false;
}

bool ObliqueRulePlanner::planOneShot(const Pose2D &start_pose) {
  double A = tan(start_pose.theta);
  double B = -1;
  double C = start_pose.y - tan(start_pose.theta) * start_pose.x;

  Pose2D added_pose(target_pose.x, target_pose.y + oneshot_higher_dist_,
                    target_pose.theta);
  double dist = fabs(A * added_pose.x + B * added_pose.y + C) /
                    sqrt(std::pow(A, 2) + std::pow(B, 2)) +
                veh_min_r * sin(start_pose.theta);
  if (dist >= veh_min_r) {
    double center_x = target_pose.x + veh_min_r;
    double center_y = tan(start_pose.theta) * (center_x - start_pose.x) +
                      start_pose.y - veh_min_r / cos(start_pose.theta);
    double stop_x = center_x - veh_min_r * sin(start_pose.theta);
    double stop_y = center_y + veh_min_r * cos(start_pose.theta);
    // collision check
    Pose2D stop_pose(stop_x, stop_y, start_pose.theta);
    if (!checkStraightLine(obstacles, points_of_obstacles, start_pose,
                           stop_pose)) {
      return false;
    }
    Pose2D back_limit_pose =
        calcNextPose(obstacles, points_of_obstacles, init_pose, stop_pose, -1,
                     -1, veh_min_r);
    if (back_limit_pose.theta < target_pose.theta) {
      return false;
    }
    key_points_.emplace_back(stop_x, stop_y, start_pose.theta);
    key_points_.emplace_back(target_pose.x, center_y, target_pose.theta);
    key_points_.push_back(target_pose);
    return true;
  } else {
    Pose2D center_right(target_pose.x + veh_min_r, added_pose.y,
                        target_pose.theta);
    C += veh_min_r / cos(init_pose.theta);
    double dist_2 = fabs(A * center_right.x + B * center_right.y + C) /
                    sqrt(std::pow(A, 2) + std::pow(B, 2));
    if (dist_2 > 2 * veh_min_r) {
      return false;
    }
    double more_theta = acos(dist_2 / 2 / veh_min_r);
    double gear_pose_theta = start_pose.theta + more_theta;
    Pose2D center_left(center_right.x - 2 * veh_min_r * sin(gear_pose_theta),
                       center_right.y + 2 * veh_min_r * cos(gear_pose_theta),
                       gear_pose_theta);
    Pose2D steer_pose(center_left.x + veh_min_r * sin(start_pose.theta),
                      center_left.y - veh_min_r * cos(start_pose.theta),
                      start_pose.theta);
    if (!checkStraightLine(obstacles, points_of_obstacles, start_pose,
                           steer_pose)) {
      return false;
    }
    Pose2D gear_pose(center_left.x + veh_min_r * sin(gear_pose_theta),
                     center_left.y - veh_min_r * cos(gear_pose_theta),
                     gear_pose_theta);
    Pose2D front_limit_pose = calcNextPose(
        obstacles, points_of_obstacles, init_pose, steer_pose, 1, 1, veh_min_r);
    if (front_limit_pose.theta < gear_pose_theta) {
      return false;
    }
    Pose2D back_limit_pose =
        calcNextPose(obstacles, points_of_obstacles, init_pose, gear_pose, -1,
                     -1, veh_min_r);
    if (back_limit_pose.theta < target_pose.theta) {
      return false;
    }
    key_points_.push_back(steer_pose);
    key_points_.push_back(gear_pose);
    key_points_.push_back(added_pose);
    key_points_.push_back(target_pose);
    return true;
  }
}

bool ObliqueRulePlanner::calcCurvePath(const Pose2D &start_pose,
                                       int max_zigzag) {
  int i = 0;
  Pose2D turning_pose = start_pose;
  while (true) {
    Pose2D next_pose = calcNextPose(obstacles, points_of_obstacles, init_pose,
                                    turning_pose, -1, -1, veh_min_r);
    auto expected_pose_reverse =
        calcExpectedPoseReverse(turning_pose, -1, -1, veh_min_r);
    auto limited_expected_pose_front =
        calcNextPose(obstacles, points_of_obstacles, init_pose,
                     expected_pose_reverse, 1, -1, veh_min_r);
    if (next_pose.theta > expected_pose_reverse.theta &&
        expected_pose_reverse.theta > turning_pose.theta &&
        limited_expected_pose_front.theta < target_pose.theta) {
      key_points_.push_back(expected_pose_reverse);
      double middle_y = expected_pose_reverse.y +
                        veh_min_r * cos(expected_pose_reverse.theta);
      if (middle_y < target_pose.y) {
        return false;
      }
      key_points_.emplace_back(target_pose.x, middle_y, target_pose.theta);
      key_points_.push_back(target_pose);
      break;
    } else if (next_pose.theta > target_pose.theta &&
               next_pose.x < target_pose.x) {
      double bigger_radius = (target_pose.x - next_pose.x) /
                             (1 - cos(next_pose.theta - target_pose.theta));
      if (fabs(bigger_radius) >= veh_min_r) {
        limited_expected_pose_front =
            calcNextPose(obstacles, points_of_obstacles, init_pose, next_pose,
                         1, -1, bigger_radius);
        if (limited_expected_pose_front.theta < target_pose.theta) {
          key_points_.push_back(next_pose);
          double middle_y =
              next_pose.y -
              bigger_radius * sin(next_pose.theta - target_pose.theta);
          if (middle_y < target_pose.y) {
            return false;
          }
          key_points_.emplace_back(target_pose.x, middle_y, target_pose.theta);
          key_points_.push_back(target_pose);
          break;
        }
      }
    } else if (next_pose.theta > target_pose.theta &&
               next_pose.x > target_pose.x &&
               turning_pose.theta < target_pose.theta) {
      double denominator = 5.0001;
      int i = 1;
      for (; i <= denominator; ++i) {
        double pose_theta = (i / denominator) * turning_pose.theta +
                            (1 - i / denominator) * target_pose.theta;
        double x_O = turning_pose.x + veh_min_r * sin(turning_pose.theta);
        double y_O = turning_pose.y - veh_min_r * cos(turning_pose.theta);
        planning_math::Vec2d center(x_O, y_O);

        double pose_x = x_O - veh_min_r * sin(pose_theta);
        double pose_y = y_O + veh_min_r * cos(pose_theta);
        Pose2D loose_pose(pose_x, pose_y, pose_theta);
        double turning_radius =
            (pose_x - target_pose.x) / (1 - sin(pose_theta));
        if (turning_radius < veh_min_r) {
          continue;
        }
        auto new_limit_pose =
            calcNextPose(obstacles, points_of_obstacles, init_pose, loose_pose,
                         -1, -1, turning_radius);
        if (new_limit_pose.theta < target_pose.theta) {
          continue;
        }

        double middle_y = pose_y - turning_radius * cos(pose_theta);
        if (middle_y < target_pose.y) {
          continue;
        }
        key_points_.emplace_back(loose_pose);
        key_points_.emplace_back(target_pose.x, middle_y, target_pose.theta);
        key_points_.push_back(target_pose);
        break;
      }
      if (i <= denominator) {
        break;
      }
    }
    if (next_pose.y < target_pose.y ||
        next_pose.theta > target_pose.theta + M_PI_4 ||
        next_pose.theta < init_pose.theta - M_PI_4) {
      return false;
    }
    key_points_.push_back(next_pose);
    turning_pose = calcNextPose(obstacles, points_of_obstacles, init_pose,
                                next_pose, 1, 1, veh_min_r);
    auto expected_pose = calcExpectedPose(next_pose, 1, 1);
    auto limited_expected_pose_back =
        calcNextPose(obstacles, points_of_obstacles, init_pose, expected_pose,
                     -1, -1, veh_min_r);
    if (turning_pose.theta > expected_pose.theta &&
        expected_pose.theta > next_pose.theta + 0.1 / veh_min_r &&
        limited_expected_pose_back.theta > target_pose.theta) {
      key_points_.push_back(expected_pose);
      double middle_y = expected_pose.y - veh_min_r * cos(expected_pose.theta);
      if (middle_y < target_pose.y) {
        return false;
      }
      key_points_.emplace_back(target_pose.x, middle_y, target_pose.theta);
      key_points_.push_back(target_pose);
      break;
    } else if (turning_pose.theta > expected_pose.theta &&
               turning_pose.theta < target_pose.theta) {
      double denominator = 5.0001;
      int i = 1;
      for (; i <= denominator; ++i) {
        double pose_theta = (i / denominator) * turning_pose.theta +
                            (1 - i / denominator) * expected_pose.theta;
        double x_O = next_pose.x - veh_min_r * sin(next_pose.theta);
        double y_O = next_pose.y + veh_min_r * cos(next_pose.theta);
        planning_math::Vec2d center(x_O, y_O);

        double pose_x = x_O + veh_min_r * sin(pose_theta);
        double pose_y = y_O - veh_min_r * cos(pose_theta);
        Pose2D interrupt_pose(pose_x, pose_y, pose_theta);
        double turning_radius =
            (pose_x - target_pose.x) / (1 - sin(pose_theta));
        if (turning_radius < veh_min_r) {
          continue;
        }
        auto new_limit_pose =
            calcNextPose(obstacles, points_of_obstacles, init_pose,
                         interrupt_pose, -1, -1, turning_radius);
        if (new_limit_pose.theta < target_pose.theta) {
          continue;
        }

        double middle_y = pose_y - turning_radius * cos(pose_theta);
        if (middle_y < target_pose.y) {
          continue;
        }
        key_points_.emplace_back(interrupt_pose);
        key_points_.emplace_back(target_pose.x, middle_y, target_pose.theta);
        key_points_.push_back(target_pose);
        break;
      }
      if (i <= denominator) {
        break;
      }
    }
    if (turning_pose.y < target_pose.y ||
        turning_pose.theta > target_pose.theta + M_PI_4 ||
        turning_pose.theta < init_pose.theta - M_PI_4) {
      return false;
    }
    key_points_.push_back(turning_pose);
    ++i;
    if (i > max_zigzag) {
      break;
    }
  }
  // return true;
  return i <= max_zigzag;
}

void ObliqueRulePlanner::extractKeyInfo(
    const parking::OpenspaceDeciderOutput &input) {
  // reserve original tlines
  t_lines_ = input.T_lines;

  // get local frame
  local_frame_pose_ =
      Pose2D(input.init_state.path_point.x, input.init_state.path_point.y,
             input.init_state.path_point.theta - M_PI_2);

  // get local poses
  init_pose = planning_math::tf2d(local_frame_pose_,
                                  Pose2D(input.target_state.path_point.x,
                                         input.target_state.path_point.y,
                                         input.target_state.path_point.theta));
  init_vel_ = -input.target_state.v;
  target_pose = planning_math::tf2d(local_frame_pose_,
                                    Pose2D(input.init_state.path_point.x,
                                           input.init_state.path_point.y,
                                           input.init_state.path_point.theta));

  planning_math::Vec2d init_vec(cos(init_pose.theta), sin(init_pose.theta));
  planning_math::Vec2d i2t_vec(target_pose.x - init_pose.x,
                               target_pose.y - init_pose.y);
  is_on_left = init_vec.CrossProd(i2t_vec) > 0;
  planning_math::Polygon2d ego_polygon;
  std::vector<planning_math::Vec2d> ego_corners(6);
  std::vector<planning_math::LineSegment2d> lines(6);

  getEPCorners(target_pose, ego_corners, lines);
  planning_math::Polygon2d temp_polygon;
  temp_polygon.update(ego_corners);

  auto local_slot_left_bound = planning_math::tf2d(
      local_frame_pose_, input.T_lines.road_lower_left_bound);
  slot_angle_ = atan2(
      local_slot_left_bound.end().y() - local_slot_left_bound.start().y(),
      local_slot_left_bound.end().x() - local_slot_left_bound.start().x());
  auto local_slot_right_bound =
      planning_math::tf2d(local_frame_pose_, input.T_lines.slot_right_bound);

  if (is_on_left) {
    slot_angle_ *= -1.0;
    init_pose = MirrorInjection(init_pose);
    P_0_ = local_slot_right_bound.end();
    P_0_ = MirrorInjection(P_0_);
    P_1_ = local_slot_left_bound.end();
    P_1_ = MirrorInjection(P_1_);

    for (auto line : input.obstacle_lines) {
      auto local_line = planning_math::tf2d(local_frame_pose_, line);
      obstacles.push_back(MirrorInjection(local_line));
    }

    // get points
    for (auto obstacle : obstacles) {
      points_of_obstacles.push_back(obstacle.start());
      points_of_obstacles.push_back(obstacle.end());
    }
    for (auto point : input.points) {
      if (temp_polygon.IsPointIn(
              planning_math::tf2d(local_frame_pose_, point))) {
        continue;
      }
      addInflationPoints(
          points_of_obstacles,
          MirrorInjection(planning_math::tf2d(local_frame_pose_, point)),
          inflation_for_points_);
    }
  } else {
    P_0_ = local_slot_left_bound.end();
    P_1_ = local_slot_right_bound.end();

    for (auto line : input.obstacle_lines) {
      auto local_line = planning_math::tf2d(local_frame_pose_, line);
      obstacles.push_back(local_line);
    }

    // get points
    for (auto obstacle : obstacles) {
      points_of_obstacles.push_back(obstacle.start());
      points_of_obstacles.push_back(obstacle.end());
    }
    for (auto point : input.points) {
      if (temp_polygon.IsPointIn(
              planning_math::tf2d(local_frame_pose_, point))) {
        continue;
      }
      addInflationPoints(points_of_obstacles,
                         planning_math::tf2d(local_frame_pose_, point),
                         inflation_for_points_);
    }
  }
  planning_math::LineSegment2d oppo_line(
      planning_math::Vec2d(-11 * sin(slot_angle_) - 10 * cos(slot_angle_),
                           11 * cos(slot_angle_) - 10 * sin(slot_angle_)),
      planning_math::Vec2d(-11 * sin(slot_angle_) + 10 * cos(slot_angle_),
                           11 * cos(slot_angle_) + 10 * sin(slot_angle_)));
  obstacles.push_back(oppo_line);

  double target_vertical_interval =
      std::hypot(half_width, front_to_rear) *
      sin(M_PI - slot_angle_ - atan2(front_to_rear, half_width));
  slot_open_height_ = target_vertical_interval / cos(slot_angle_);
}

bool ObliqueRulePlanner::isInnerSideVehicle(
    const parking::OpenspaceDeciderOutput &input) {
  std::vector<planning_math::Box2d> local_obs_boxes;
  if (is_on_left) {
    for (auto obs_box : input.obstacle_boxs) {
      local_obs_boxes.push_back(
          MirrorInjection(planning_math::tf2d(local_frame_pose_, obs_box)));
    }
  } else {
    for (auto obs_box : input.obstacle_boxs) {
      local_obs_boxes.push_back(
          planning_math::tf2d(local_frame_pose_, obs_box));
    }
  }

  double xx = target_pose.x +
              msquare::VehicleParam::Instance()->center_to_geometry_center *
                  cos(target_pose.theta);
  double yy = target_pose.y +
              msquare::VehicleParam::Instance()->center_to_geometry_center *
                  sin(target_pose.theta);
  planning_math::Box2d inner_box(
      planning_math::Vec2d(xx + msquare::VehicleParam::Instance()->width,
                           yy + msquare::VehicleParam::Instance()->width *
                                    tan(slot_angle_)),
      target_pose.theta, msquare::VehicleParam::Instance()->length,
      msquare::VehicleParam::Instance()->width);
  for (auto local_obs_box : local_obs_boxes) {
    if (inner_box.IsPointIn(local_obs_box.center())) {
      return true;
    }
  }
  return false;
}

bool ObliqueRulePlanner::calcAdjustPoses(const Pose2D &end_pose,
                                         double radius) {
  Pose2D init_local = planning_math::tf2d(end_pose, init_pose);
  Pose2D start_local = Pose2D(-init_local.x, -init_local.y, init_local.theta);
  Pose2D target_local{0.0, 0.0, 0.0};
  double dx = start_local.x - target_local.x;
  double dy = start_local.y - target_local.y;
  double dtheta = start_local.theta - target_local.theta;
  std::vector<Pose2D> key_points;
  // key_points.emplace_back(start_local);

  bool pre_requirement = dx > 0.0;
  bool success{false};
  const double kMathEpsilon = 1e-3;
  if (pre_requirement) {
    if (std::abs(1 - std::cos(dtheta)) < kMathEpsilon) {
      if (std::abs(dy) < kMathEpsilon) {
        key_points.emplace_back(target_local);
        success = true;
      }
    } else {
      double r = dy / (1 - std::cos(dtheta));
      if ((std::abs(r) > radius) &&
          (start_local.x - r * std::sin(dtheta) > target_local.x) &&
          (r * std::sin(dtheta) > 0.0)) {
        Pose2D via_point{start_local.x - r * sin(dtheta), target_local.y,
                         target_local.theta};
        key_points.emplace_back(via_point);
        success = true;
      }
    }
    if (!success) {
      std::vector<double> sgn_list{1.0, -1.0};
      for (auto sgn : sgn_list) {
        double x_o1 =
            start_local.x + sgn * radius * std::sin(start_local.theta);
        double y_o1 =
            start_local.y - sgn * radius * std::cos(start_local.theta);
        double y_o2 = target_local.y + sgn * radius;
        double delta_y = y_o1 - y_o2;
        if (std::abs(delta_y) < 2 * radius) {
          double theta1 = (std::acos(std::abs(delta_y) / (2 * radius))) * sgn;
          double x2 = x_o1 - sgn * radius * std::sin(theta1) -
                      sgn * radius * std::sin(theta1);
          if ((x2 > target_local.x) &&
              (x_o1 - sgn * radius * std::sin(theta1) < start_local.x)) {
            key_points.emplace_back(
                Pose2D{x_o1 - sgn * radius * std::sin(theta1),
                       y_o1 + sgn * radius * std::cos(theta1), theta1});
            key_points.emplace_back(
                Pose2D{x2, target_local.y, target_local.theta});
            success = true;
            break;
          }
        }
      }
    }
  }
  if (key_points.empty()) {
    return false;
  }

  std::vector<Pose2D> global_points;
  for (int i = 0; i < key_points.size(); i++) {
    global_points.push_back(planning_math::tf2d_inv(
        end_pose,
        Pose2D(-key_points[i].x, -key_points[i].y, key_points[i].theta)));
  }

  Pose2D last_pose = init_pose;
  if (global_points.size() == 1) {
    planning_math::Vec2d vec_former_latter(global_points[0].x - last_pose.x,
                                           global_points[0].y - last_pose.y);
    double vec_heading = vec_former_latter.Angle();
    double turning_radius =
        sqrt(std::pow(vec_former_latter.Length(), 2) / 2 /
             (1 - cos(last_pose.theta - global_points[0].theta)));
    if (turning_radius < radius) {
      return false;
    }
    int to_left = global_points[0].theta - last_pose.theta > 0 ? 1 : -1;
    auto limit_pose = calcNextPose(obstacles, points_of_obstacles, init_pose,
                                   last_pose, to_left, 1, turning_radius);
    if ((limit_pose.theta - global_points[0].theta) * to_left < 0) {
      return false;
    }
  } else { // size == 2
    for (int i = 0; i < global_points.size(); i++) {
      auto next_pose = global_points[i];
      int to_left = next_pose.theta - last_pose.theta > 0 ? 1 : -1;
      auto limit_pose = calcNextPose(obstacles, points_of_obstacles, init_pose,
                                     last_pose, to_left, 1, radius);
      if ((limit_pose.theta - next_pose.theta) * to_left < 0) {
        return false;
      }
      last_pose = next_pose;
    }
  }
  key_points_.insert(key_points_.end(), global_points.begin(),
                     global_points.end());

  return success;
}

Pose2D ObliqueRulePlanner::calcTurningPose(const Pose2D &start_pose,
                                           double safe_buffer,
                                           planning_math::Vec2d &arc_center) {
  double xs = start_pose.x, ys = start_pose.y;
  double heading = start_pose.theta; // relative angle

  double mid_const =
      ys - veh_min_r / cos(heading) - tan(heading) * xs - P_1_.y();

  double turing_radius = l_1 - safe_buffer;
  double A = 1 + std::pow(tan(heading), 2);
  double B = 2 * (tan(heading) * mid_const - P_1_.x());
  double C = std::pow(mid_const, 2) + std::pow(P_1_.x(), 2) -
             std::pow(turing_radius, 2);

  // when slot on left or up this needs to change
  double x_O1 = (-B + sqrt(std::pow(B, 2) - 4 * A * C)) / (2 * A);
  double y_O1 =
      tan(heading) * x_O1 + ys - xs * tan(heading) - veh_min_r / cos(heading);
  arc_center.set_point(x_O1, y_O1);
  double target_x = x_O1 - veh_min_r * sin(heading);
  double target_y = tan(heading) * (target_x - xs) + ys;

  return Pose2D(target_x, target_y, heading);
}

void ObliqueRulePlanner::extendKeyPointsToResult(double step_size) {
  auto local_rule_path =
      InterpolatePath(2 * step_size, key_points_); // hack@lizhiqiang1

  // judge if needs mirror-injection
  if (is_on_left) {
    for (int i = 0; i < key_points_.size(); ++i) {
      key_points_.at(i) = MirrorInjection(key_points_.at(i));
    }
    for (int i = 0; i < local_rule_path.size(); ++i) {
      local_rule_path.at(i) = MirrorInjection(local_rule_path.at(i));
    }
  }

  std::vector<Pose2D> global_key_points, global_rule_path;
  for (auto key_point : key_points_) {
    global_key_points.push_back(
        planning_math::tf2d_inv(local_frame_pose_, key_point));
  }
  for (auto path_point : local_rule_path) {
    global_rule_path.push_back(
        planning_math::tf2d_inv(local_frame_pose_, path_point));
  }
  key_points_ = global_key_points;
  result_ = convertToSbpResult(global_rule_path);
  std::reverse(result_.x.begin(), result_.x.end());
  std::reverse(result_.y.begin(), result_.y.end());
  std::reverse(result_.phi.begin(), result_.phi.end());
  std::reverse(result_.wheel_base_offset.begin(),
               result_.wheel_base_offset.end());
}

Pose2D ObliqueRulePlanner::calcExpectedPose(const Pose2D &start_pose,
                                            int steer_direction,
                                            int travel_direction) {
  // get center
  double x_O =
      start_pose.x - steer_direction * veh_min_r * sin(start_pose.theta);
  double y_O =
      start_pose.y + steer_direction * veh_min_r * cos(start_pose.theta);
  planning_math::Vec2d center(x_O, y_O);

  double x_O2 = target_pose.x + veh_min_r;
  double dx = x_O2 - x_O;
  if (fabs(dx) > (2 * veh_min_r)) {
    return Pose2D(0, 0, 1e19);
  }
  double next_theta = asin(dx / (2 * veh_min_r));

  double next_x = x_O + steer_direction * veh_min_r * sin(next_theta);
  double next_y = y_O - steer_direction * veh_min_r * cos(next_theta);
  return Pose2D(next_x, next_y, next_theta);
}

Pose2D ObliqueRulePlanner::calcExpectedPoseReverse(const Pose2D &start_pose,
                                                   int steer_direction,
                                                   int travel_direction,
                                                   double radius) {
  // get center
  double x_O = start_pose.x - steer_direction * radius * sin(start_pose.theta);
  double y_O = start_pose.y + steer_direction * radius * cos(start_pose.theta);
  planning_math::Vec2d center(x_O, y_O);

  double x_O2 = target_pose.x - radius;
  double dx = x_O - x_O2;
  if (fabs(dx) > (2 * radius)) {
    return Pose2D(0, 0, 1e19);
  }
  double next_theta = acos(dx / (2 * radius)) + M_PI_2;

  double next_x = x_O + steer_direction * radius * sin(next_theta);
  double next_y = y_O - steer_direction * radius * cos(next_theta);
  return Pose2D(next_x, next_y, next_theta);
}

double ObliqueRulePlanner::calBackStraightDist(const Pose2D &start_pose) {
  double back_length = 10.0;
  double max_y = half_width;
  double min_y = -half_width;
  double max_x = -back_to_rear - CarParams::GetInstance()->lon_inflation() +
                 CarParams::GetInstance()->lat_inflation();

  for (auto &p : points_of_obstacles) {
    planning_math::Vec2d p_transformed = planning_math::tf2d(start_pose, p);
    if (p_transformed.x() > max_x || p_transformed.y() > max_y ||
        p_transformed.y() < min_y) {
      continue;
    }
    back_length = std::min(back_length, max_x - p_transformed.x());
  }

  Pose2D local_start_pose = planning_math::tf2d(start_pose, start_pose);
  std::vector<planning_math::Vec2d> ego_corners;
  std::vector<planning_math::LineSegment2d> ego_lines;
  getEPRectCorners(local_start_pose, ego_corners, ego_lines);
  planning_math::LineSegment2d back_edge = ego_lines.at(0);

  for (auto &line : obstacles) {
    planning_math::LineSegment2d l_transformed =
        planning_math::tf2d(start_pose, line);
    if (l_transformed.HasIntersect(back_edge)) {
      return 0;
    }
    if (l_transformed.max_y() < back_edge.min_y() ||
        l_transformed.min_y() > back_edge.max_y() ||
        l_transformed.min_x() > back_edge.max_x()) {
      continue;
    }
    if (l_transformed.min_y() >= back_edge.min_y() &&
        l_transformed.max_y() <= back_edge.max_y()) {
      back_length =
          std::min(back_length, back_edge.min_x() - l_transformed.max_x());
    } else if (l_transformed.min_y() < back_edge.min_y() &&
               l_transformed.max_y() > back_edge.max_y()) {
      double lambda_1 = (back_edge.max_y() - l_transformed.start().y()) /
                        (l_transformed.end().y() - l_transformed.start().y());
      double x_1 = lambda_1 * l_transformed.end().x() +
                   (1 - lambda_1) * l_transformed.start().x();
      double lambda_2 = (back_edge.min_y() - l_transformed.start().y()) /
                        (l_transformed.end().y() - l_transformed.start().y());
      double x_2 = lambda_2 * l_transformed.end().x() +
                   (1 - lambda_1) * l_transformed.start().x();
      back_length =
          std::min(back_length, back_edge.min_x() - std::max(x_1, x_2));
    } else if (l_transformed.start().y() >= back_edge.min_y() &&
               l_transformed.start().y() <= back_edge.max_y()) {
      double axis_y = l_transformed.end().y() > back_edge.max_y()
                          ? back_edge.max_y()
                          : back_edge.min_y();
      double lambda = (axis_y - l_transformed.start().y()) /
                      (l_transformed.end().y() - l_transformed.start().y());
      double axis_x = lambda * l_transformed.end().x() +
                      (1 - lambda) * l_transformed.start().x();
      if (axis_x < back_edge.min_x()) {
        back_length = std::min(back_length,
                               back_edge.min_x() -
                                   std::max(axis_x, l_transformed.start().x()));
      }
    } else {
      // if (l_transformed.end().y() >= back_edge.min_y() &&
      //   l_transformed.end().y() <= back_edge.max_y())
      double axis_y = l_transformed.start().y() > back_edge.max_y()
                          ? back_edge.max_y()
                          : back_edge.min_y();
      double lambda = (axis_y - l_transformed.start().y()) /
                      (l_transformed.end().y() - l_transformed.start().y());
      double axis_x = lambda * l_transformed.end().x() +
                      (1 - lambda) * l_transformed.start().x();
      if (axis_x < back_edge.min_x()) {
        back_length = std::min(back_length,
                               back_edge.min_x() -
                                   std::max(axis_x, l_transformed.end().x()));
      }
    }
    back_length = std::max(0.0, back_length);
  }

  return back_length;
}

double ObliqueRulePlanner::calForwardStraightDist(const Pose2D &start_pose) {
  double forward_length = 10.0;
  double max_y = half_width;
  double min_y = -half_width;
  double max_x = front_to_rear + CarParams::GetInstance()->lon_inflation() -
                 CarParams::GetInstance()->lat_inflation();

  for (auto &p : points_of_obstacles) {
    planning_math::Vec2d p_transformed = planning_math::tf2d(start_pose, p);
    if (p_transformed.x() < max_x || p_transformed.y() > max_y ||
        p_transformed.y() < min_y) {
      continue;
    }
    forward_length = std::min(forward_length, p_transformed.x() - max_x);
  }

  Pose2D local_start_pose = planning_math::tf2d(start_pose, start_pose);
  std::vector<planning_math::Vec2d> ego_corners;
  std::vector<planning_math::LineSegment2d> ego_lines;
  getEPRectCorners(local_start_pose, ego_corners, ego_lines);
  planning_math::LineSegment2d front_edge = ego_lines.at(2);

  for (auto &line : obstacles) {
    planning_math::LineSegment2d l_transformed =
        planning_math::tf2d(start_pose, line);
    if (l_transformed.HasIntersect(front_edge)) {
      return 0;
    }
    if (l_transformed.max_y() < front_edge.min_y() ||
        l_transformed.min_y() > front_edge.max_y() ||
        l_transformed.max_x() < front_edge.min_x()) {
      continue;
    }
    if (l_transformed.min_y() >= front_edge.min_y() &&
        l_transformed.max_y() <= front_edge.max_y()) {
      forward_length =
          std::min(forward_length, l_transformed.min_x() - front_edge.max_x());
    } else if (l_transformed.min_y() < front_edge.min_y() &&
               l_transformed.max_y() > front_edge.max_y()) {
      double lambda_1 = (front_edge.max_y() - l_transformed.start().y()) /
                        (l_transformed.end().y() - l_transformed.start().y());
      double x_1 = lambda_1 * l_transformed.end().x() +
                   (1 - lambda_1) * l_transformed.start().x();
      double lambda_2 = (front_edge.min_y() - l_transformed.start().y()) /
                        (l_transformed.end().y() - l_transformed.start().y());
      double x_2 = lambda_2 * l_transformed.end().x() +
                   (1 - lambda_1) * l_transformed.start().x();
      forward_length =
          std::min(forward_length, std::min(x_1, x_2) - front_edge.max_x());
    } else if (l_transformed.start().y() >= front_edge.min_y() &&
               l_transformed.start().y() <= front_edge.max_y()) {
      double axis_y = l_transformed.end().y() > front_edge.max_y()
                          ? front_edge.max_y()
                          : front_edge.min_y();
      double lambda = (axis_y - l_transformed.start().y()) /
                      (l_transformed.end().y() - l_transformed.start().y());
      double axis_x = lambda * l_transformed.end().x() +
                      (1 - lambda) * l_transformed.start().x();
      if (axis_x > front_edge.max_x()) {
        forward_length = std::min(forward_length,
                                  std::min(axis_x, l_transformed.start().x()) -
                                      front_edge.max_x());
      }
    } else {
      // if (l_transformed.end().y() >= front_edge.min_y() &&
      //   l_transformed.end().y() <= front_edge.max_y())
      double axis_y = l_transformed.start().y() > front_edge.max_y()
                          ? front_edge.max_y()
                          : front_edge.min_y();
      double lambda = (axis_y - l_transformed.start().y()) /
                      (l_transformed.end().y() - l_transformed.start().y());
      double axis_x = lambda * l_transformed.end().x() +
                      (1 - lambda) * l_transformed.start().x();
      if (axis_x > front_edge.max_x()) {
        forward_length =
            std::min(forward_length, std::min(axis_x, l_transformed.end().x()) -
                                         front_edge.max_x());
      }
    }
    forward_length = std::max(0.0, forward_length);
  }

  return forward_length;
}

void ObliqueRulePlanner::getEPRectCorners(
    const Pose2D &pose, std::vector<planning_math::Vec2d> &corners,
    std::vector<planning_math::LineSegment2d> &lines) {
  // all six corners
  corners.clear();
  lines.clear();

  double edge_width = half_width;
  double back_edge_length = -VehicleParam::Instance()->back_edge_to_center -
                            CarParams::GetInstance()->lon_inflation();
  double front_edge_length = VehicleParam::Instance()->front_edge_to_center +
                             CarParams::GetInstance()->lon_inflation();

  corners.emplace_back(pose.x - edge_width * sin(pose.theta) +
                           back_edge_length * cos(pose.theta),
                       pose.y + edge_width * cos(pose.theta) +
                           back_edge_length * sin(pose.theta));
  corners.emplace_back(pose.x + edge_width * sin(pose.theta) +
                           back_edge_length * cos(pose.theta),
                       pose.y - edge_width * cos(pose.theta) +
                           back_edge_length * sin(pose.theta));
  corners.emplace_back(pose.x + edge_width * sin(pose.theta) +
                           front_edge_length * cos(pose.theta),
                       pose.y - edge_width * cos(pose.theta) +
                           front_edge_length * sin(pose.theta));
  corners.emplace_back(pose.x - edge_width * sin(pose.theta) +
                           front_edge_length * cos(pose.theta),
                       pose.y + edge_width * cos(pose.theta) +
                           front_edge_length * sin(pose.theta));
  for (int i = 0; i < corners.size(); ++i) {
    lines.emplace_back(corners.at(i), corners.at((i + 1) % (corners.size())));
  }
}

} // namespace msquare
