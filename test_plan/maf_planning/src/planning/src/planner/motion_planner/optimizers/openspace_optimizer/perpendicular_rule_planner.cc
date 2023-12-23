#include "planner/motion_planner/optimizers/openspace_optimizer/perpendicular_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/rule_planner_util.h"

#define MAX_MOVE_DISTANCE 0.6

namespace msquare {

PerpendicularRulePlanner::PerpendicularRulePlanner(
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

  msquare::OdoAdaptor odo_adapter(odo);
  const auto &new_odo = odo_adapter.generateNewOdo();
  move_ditsance_ = odo_adapter.getMoveDistance();

  // move_ditsance_ = move_ditsance_;
  up_bound_move_distance_ = odo_adapter.getUpBoundMoveDistance();

  extractKeyInfo(new_odo);

  // run env_generator
  perpendicular_scenario_adapter_.init(P_0_, P_1_, points_of_obstacles,
                                       init_pose, local_frame_pose_,
                                       is_on_left);
  perpendicular_scenario_adapter_.process();

  if (isInnerSideVehicle(new_odo)) {
    for (auto &pair : buffer_zigzag_pairs_) {
      pair.first -= 0.25;
    }
  }

  // std::cout << "channel width: "
  //           << perpendicular_scenario_adapter_.get_channel_width() <<
  //           std::endl;
  // std::cout << "slot width: "
  //           << perpendicular_scenario_adapter_.get_slot_width() << std::endl;
  // std::cout << "inner_obs_height: "
  //           << perpendicular_scenario_adapter_.get_inner_obs_height()
  //           << std::endl;
  // std::cout << "outside_obs_height: "
  //           << perpendicular_scenario_adapter_.get_outside_obs_height()
  //           << std::endl;
  // std::cout << "inner_space_width: "
  //           << perpendicular_scenario_adapter_.get_inner_space_width()
  //           << std::endl;
  // std::cout << "outside_space_width: "
  //           << perpendicular_scenario_adapter_.get_outside_space_width()
  //           << std::endl;

  l_1 = veh_min_r - half_width;
  l_2 = std::hypot(veh_min_r + half_width, front_corner_length);
  gamma_4 = atan2(front_to_rear, veh_min_r + half_width);

  slot_width_ = floor((P_1_.x() - P_0_.x() + 0.1) * 10) / 10;
  slot_width_ = 7.5 - perpendicular_scenario_adapter_.get_channel_width();
  if (slot_width_ < 2.5) {
    slot_width_ = 2.5;
  } else if (slot_width_ > 3.0) {
    slot_width_ = 3.0;
  }

  generateSweetSpots();

  safe_sweet_poses_.clear();
  for (auto sw_pose : sweet_spot_[slot_width_]) {
    if (checkSinglePose(sw_pose, true)) {
      safe_sweet_poses_.push_back(sw_pose);
    }
  }
}

bool PerpendicularRulePlanner::checkSinglePose(const Pose2D &pose,
                                               bool use_lon) {
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

SbpResult PerpendicularRulePlanner::getResult() { return result_; }
std::vector<Pose2D> PerpendicularRulePlanner::getSearchPoints() {
  return key_points_;
}

void PerpendicularRulePlanner::Update(
    const planning_math::Box2d map_boundary,
    const std::vector<planning_math::LineSegment2d> &map) {}

bool PerpendicularRulePlanner::isMidPlan() {
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

Pose2D PerpendicularRulePlanner::calcExpectedPoseBoth(const Pose2D &start_pose,
                                                      int steer_direction,
                                                      int travel_direction) {
  if (steer_direction == -1 && travel_direction == -1) {
    return calcExpectedPoseReverse(start_pose, -1, -1, veh_min_r);
  } else if (steer_direction == 1 && travel_direction == 1) {
    return calcExpectedPose(start_pose, 1, 1);
  }

  return Pose2D(0.0, 0.0, 0.0);
}

bool PerpendicularRulePlanner::planMidway(const Pose2D &mid_pose,
                                          bool is_reverse) {
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
        int i = 0;
        for (; i <= denominator; ++i) {
          double pose_theta = (1 - i / denominator) * turning_pose.theta +
                              (i / denominator) * target_pose.theta;
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
          if (middle_y < target_pose.y || middle_y > upper_height_) {
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
                 next_pose.theta > turning_pose.theta &&
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
          if (middle_y < target_pose.y || middle_y > upper_height_) {
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
    if (next_pose.y > upper_height_ || next_pose.y < target_pose.y ||
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

bool PerpendicularRulePlanner::planRestartWay(const Pose2D &mid_pose) {
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

bool PerpendicularRulePlanner::Plan(const std::vector<SbpObstaclePtr> &obs_ptrs,
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

  double move_down_dist = 0.5;
  if (is_on_left) {
    auto local_right_road_bound = MirrorInjection(planning_math::tf2d(
        local_frame_pose_, t_lines_.road_lower_right_bound));
    auto local_right_slot_bound = MirrorInjection(
        planning_math::tf2d(local_frame_pose_, t_lines_.slot_right_bound));
    if (isNarrowScenario()) {
      obstacles.emplace_back(
          planning_math::Vec2d(local_right_road_bound.start().x(),
                               local_right_road_bound.start().y() -
                                   move_down_dist),
          planning_math::Vec2d(local_right_road_bound.end().x(),
                               local_right_road_bound.end().y() -
                                   move_down_dist));
      obstacles.emplace_back(
          planning_math::Vec2d(local_right_slot_bound.start().x(),
                               local_right_slot_bound.start().y() -
                                   move_down_dist),
          planning_math::Vec2d(local_right_slot_bound.end().x(),
                               local_right_slot_bound.end().y()));
    } else {
      obstacles.push_back(local_right_road_bound);
      obstacles.push_back(local_right_slot_bound);
    }
  } else {
    auto local_left_road_bound =
        planning_math::tf2d(local_frame_pose_, t_lines_.road_lower_left_bound);
    auto local_left_slot_bound =
        planning_math::tf2d(local_frame_pose_, t_lines_.slot_left_bound);
    if (isNarrowScenario()) {
      obstacles.emplace_back(
          planning_math::Vec2d(local_left_road_bound.start().x(),
                               local_left_road_bound.start().y() -
                                   move_down_dist),
          planning_math::Vec2d(local_left_road_bound.end().x(),
                               local_left_road_bound.end().y() -
                                   move_down_dist));
      obstacles.emplace_back(
          planning_math::Vec2d(local_left_slot_bound.start().x(),
                               local_left_slot_bound.start().y() -
                                   move_down_dist),
          planning_math::Vec2d(local_left_slot_bound.end().x(),
                               local_left_slot_bound.end().y()));
    } else {
      obstacles.push_back(local_left_road_bound);
      obstacles.push_back(local_left_slot_bound);
    }
  }

  if (std::abs(init_vel_) > 1e-6) {
    if (!planMidway(init_pose, init_vel_ < 0) &&
        !planMidway2(init_pose, init_vel_ < 0)) {
      if (init_vel_ > 0 && planMidway3(init_pose)) {
        extendKeyPointsToResult(
            msquare::HybridAstarConfig::GetInstance()->step_size);
        return true;
      } else {
        return false;
      }
    }
    extendKeyPointsToResult(
        msquare::HybridAstarConfig::GetInstance()->step_size);
    return true;
  }

  // 初次规划
  for (auto buffer_pair : buffer_zigzag_pairs_) {
    if (std::abs(target_vel_ - 2) < 1e-6) {
      if (PlanWithBufferReverse(buffer_pair.first, buffer_pair.second)) {
        extendKeyPointsToResult(
            msquare::HybridAstarConfig::GetInstance()->step_size);
        return true;
      }
    } else {
      if (PlanWithBuffer(buffer_pair.first, buffer_pair.second)) {
        extendKeyPointsToResult(
            msquare::HybridAstarConfig::GetInstance()->step_size);
        return true;
      }
    }
  }

  return false;
}

bool PerpendicularRulePlanner::planMidway2(const Pose2D &mid_pose,
                                           bool is_reverse) {
  if (is_reverse) {
    if (planMidwayBackward(mid_pose, is_reverse)) {
      return true;
    } else {
      return planMidwayForward(mid_pose, is_reverse);
    }
  } else {
    if (planMidwayForward(mid_pose, is_reverse)) {
      return true;
    } else {
      return planMidwayBackward(mid_pose, is_reverse);
    }
  }
}

bool PerpendicularRulePlanner::planMidwayBackward(const Pose2D &mid_pose,
                                                  bool is_reverse) {
  key_points_.clear();
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
  return false;
}

bool PerpendicularRulePlanner::planMidwayForward(const Pose2D &mid_pose,
                                                 bool is_reverse) {
  key_points_.clear();
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

bool PerpendicularRulePlanner::planMidway3(const Pose2D &mid_pose) {
  key_points_.clear();
  double inner_useful_dist = calInnerSideDist(mid_pose);
  double inner_buffer = 0.05;

  double O_x = mid_pose.x + veh_min_r * sin(mid_pose.theta);
  double O_y = mid_pose.y - veh_min_r * cos(mid_pose.theta);
  double single_curve_theta =
      acos(1 - (inner_useful_dist - inner_buffer) / 2 / veh_min_r);
  double mid_s_theta = init_pose.theta - single_curve_theta;
  Pose2D mid_s_pose{O_x - veh_min_r * sin(mid_s_theta),
                    O_y + veh_min_r * cos(mid_s_theta), mid_s_theta};

  Pose2D first_limit_pose = calcNextPose(obstacles, points_of_obstacles,
                                         init_pose, mid_pose, -1, 1, veh_min_r);
  if (first_limit_pose.theta > mid_s_theta) {
    return false;
  }

  double O2_x = O_x - 2 * veh_min_r * sin(mid_s_theta);
  double O2_y = O_y + 2 * veh_min_r * cos(mid_s_theta);
  double parallel_theta = mid_pose.theta;
  Pose2D parallel_pose{O2_x + veh_min_r * sin(parallel_theta),
                       O2_y - veh_min_r * cos(parallel_theta), parallel_theta};
  Pose2D second_limit_pose = calcNextPose(obstacles, points_of_obstacles,
                                          init_pose, mid_pose, 1, 1, veh_min_r);
  if (second_limit_pose.theta < parallel_theta) {
    return false;
  }

  planning_math::Vec2d center;
  Pose2D turning_pose = calcTurningPose(parallel_pose, inner_buffer, center);
  if (std::isnan(turning_pose.x) || std::isnan(turning_pose.y) ||
      std::isnan(turning_pose.theta)) {
    return false;
  }

  if (!checkStraightLine(obstacles, points_of_obstacles, init_pose,
                         turning_pose)) {
    return false;
  }
  key_points_.push_back(mid_pose);
  key_points_.push_back(mid_s_pose);
  key_points_.push_back(parallel_pose);
  key_points_.push_back(turning_pose);
  double max_zigzag = 4;
  if (calcCurvePath(turning_pose, max_zigzag)) {
    return true;
  }
  return false;
}

bool PerpendicularRulePlanner::PlanWithBuffer2(double safe_buffer,
                                               int max_zigzag) {
  if (PlanWithBuffer2FromInitPose(safe_buffer, max_zigzag)) {
    return true;
  } else {
    return PlanWithBuffer2FromInitPose(safe_buffer, max_zigzag);
  }
}

bool PerpendicularRulePlanner::PlanWithBuffer2FromInitPose(double safe_buffer,
                                                           int max_zigzag) {
  key_points_.clear();
  result_.clear();
  double l_0 = std::hypot(veh_min_r + half_width, front_to_rear);
  double O_y = upper_height_ - l_0;
  double O_x = P_1_.x() + sqrt(std::pow(veh_min_r - half_width, 2) -
                               std::pow(P_1_.y() - O_y, 2));

  double O2_y = init_pose.y + veh_min_r;
  double O2_x =
      O_x - sqrt(std::pow(2 * veh_min_r, 2) - std::pow(O_y - O2_y, 2));

  Pose2D start_left_turn_pose{O2_x, init_pose.y, 0};
  double end_theta = atan2(O_x - O2_x, O2_y - O_y);
  Pose2D gear_change_pose{(O_x + O2_x) / 2, (O_y + O2_y) / 2, end_theta};
  planning_math::Vec2d center;
  Pose2D turning_pose = calcTurningPose(gear_change_pose, safe_buffer, center);
  if (std::isnan(turning_pose.x) || std::isnan(turning_pose.y) ||
      std::isnan(turning_pose.theta)) {
    return false;
  }

  if (!checkStraightLine(obstacles, points_of_obstacles, init_pose,
                         turning_pose)) {
    return false;
  }
  key_points_.push_back(init_pose);
  key_points_.push_back(start_left_turn_pose);
  key_points_.push_back(gear_change_pose);
  key_points_.push_back(turning_pose);

  if (calcCurvePath(turning_pose, max_zigzag)) {
    return true;
  }
  return false;
}

bool PerpendicularRulePlanner::PlanWithBuffer2FromLowerPose(double safe_buffer,
                                                            int max_zigzag) {
  key_points_.clear();
  result_.clear();
  double l_0 = std::hypot(veh_min_r + half_width, front_to_rear);
  double O_y = upper_height_ - l_0;
  double O_x = P_1_.x() + sqrt(std::pow(veh_min_r - half_width, 2) -
                               std::pow(P_1_.y() - O_y, 2));

  double O2_y = P_1_.y() + 1.3 + veh_min_r;
  double O2_x =
      O_x - sqrt(std::pow(2 * veh_min_r, 2) - std::pow(O_y - O2_y, 2));

  Pose2D start_left_turn_pose{O2_x, O2_y - veh_min_r, 0};
  double end_theta = atan2(O_x - O2_x, O2_y - O_y);
  Pose2D gear_change_pose{(O_x + O2_x) / 2, (O_y + O2_y) / 2, end_theta};

  bool flag = false;
  for (double ratio = 2.0; ratio > 0.99; ratio -= 0.1) {
    if (calcAdjustPoses(start_left_turn_pose, ratio * veh_min_r)) {
      flag = true;
      break;
    }
  }
  if (!flag)
    return false;

  planning_math::Vec2d center;
  Pose2D turning_pose = calcTurningPose(gear_change_pose, safe_buffer, center);
  if (std::isnan(turning_pose.x) || std::isnan(turning_pose.y) ||
      std::isnan(turning_pose.theta)) {
    return false;
  }

  if (!checkStraightLine(obstacles, points_of_obstacles, init_pose,
                         turning_pose)) {
    return false;
  }
  key_points_.push_back(start_left_turn_pose);
  key_points_.push_back(gear_change_pose);
  key_points_.push_back(turning_pose);

  Pose2D rb_limit_pose = calcNextPose(obstacles, points_of_obstacles, init_pose,
                                      turning_pose, -1, -1, veh_min_r);
  double front_dist = calForwardStraightDist(rb_limit_pose);
  Pose2D second_fs_pose{rb_limit_pose.x + front_dist * cos(rb_limit_pose.theta),
                        rb_limit_pose.y + front_dist * sin(rb_limit_pose.theta),
                        rb_limit_pose.theta};
  key_points_.push_back(rb_limit_pose);
  key_points_.push_back(second_fs_pose);

  if (calcCurvePath(second_fs_pose, max_zigzag)) {
    return true;
  }

  return false;
}

void PerpendicularRulePlanner::constructNewTlines() {
  obstacles.pop_back();
  obstacles.pop_back();
  obstacles.emplace_back(
      planning_math::Vec2d(target_pose.x - 3 * half_width, target_pose.y + 1.0),
      planning_math::Vec2d(target_pose.x - 1.5 * half_width,
                           target_pose.y + 1.0));

  std::vector<planning_math::Vec2d> inner_points;
  for (const auto &point : points_of_obstacles) {
    if (point.x() > target_pose.x &&
        point.x() < target_pose.x + 3 * half_width &&
        point.y() < init_pose.y - half_width &&
        point.y() > VehicleParam::Instance()->length / 2 -
                        VehicleParam::Instance()->back_edge_to_center) {
      inner_points.push_back(point);
    }
  }
  double min_x = target_pose.x + 3 * half_width;
  double max_y = init_pose.y - half_width;
  for (const auto &point : inner_points) {
    min_x = std::min(point.x(), min_x);
    max_y = std::max(point.y(), max_y);
  }
  P_1_.set_point(min_x, max_y);
}

bool PerpendicularRulePlanner::PlanInSlot() {
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
  if (side_para_pose.y > target_pose.y + in_slot_higher_dist_) {
    path_to_fill = {method_input};
    if (PlanInSlotSub0_5(path_to_fill)) {
      std::vector<Pose2D> path_to_fill_without_end = path_to_fill;
      if (path_to_fill_without_end.size() >
          2) { // ignore last straight line safe check
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
            2) { // ignore last straight line safe check
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
  }

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
          2) { // ignore last straight line safe check
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
          2) { // ignore last straight line safe check
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
  key_points_.clear();
  return false;
}

bool PerpendicularRulePlanner::PlanInSlotSub_1(
    std::vector<Pose2D> &path_to_fill) {
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

bool PerpendicularRulePlanner::PlanInSlotSub0_5(
    std::vector<Pose2D> &path_to_fill) {
  Pose2D side_pose = path_to_fill.back();
  double delta_x = fabs(target_pose.x - side_pose.x);
  double delta_y = fabs(side_pose.y - (target_pose.y + in_slot_higher_dist_));
  double max_r = (std::pow(delta_x, 2) + std::pow(delta_y, 2)) / (4 * delta_x);

  if (max_r < veh_min_r) {
    return false;
  }
  double mid_x = (side_pose.x + target_pose.x) / 2;
  double mid_y = side_pose.y - delta_y / 2;
  double single_curve_theta = acos((max_r - delta_x / 2) / max_r);
  path_to_fill.emplace_back(mid_x, mid_y, side_pose.theta + single_curve_theta);
  path_to_fill.emplace_back(target_pose.x, target_pose.y + in_slot_higher_dist_,
                            target_pose.theta);
  path_to_fill.push_back(target_pose);
  return true;
}

bool PerpendicularRulePlanner::PlanInSlotSub0(
    std::vector<Pose2D> &path_to_fill) {
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

void PerpendicularRulePlanner::PlanInSlotSub1(std::vector<Pose2D> &path_to_fill,
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

void PerpendicularRulePlanner::PlanInSlotSub2(std::vector<Pose2D> &path_to_fill,
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

bool PerpendicularRulePlanner::PlanWithBuffer(double safe_buffer,
                                              int max_zigzag) {
  key_points_.clear();
  result_.clear();
  // all key_points need to checkCollision such as init_pose, turning_pose

  // if (init_pose.theta > 0) {
  //   if (startWithStraightLine(safe_buffer, max_zigzag)) {
  //     return true;
  //   } else {
  //     return startWithSweetPose(safe_buffer, max_zigzag);
  //   }
  // } else {
  //   if (startWithSweetPose(safe_buffer, max_zigzag)) {
  //     return true;
  //   } else {
  //     key_points_.clear();
  //     return startWithStraightLine(safe_buffer, max_zigzag);
  //   }
  // }

  if (isTwoSideIsEmptyScenario()) {
    if (startWithStraightLine(safe_buffer, max_zigzag)) {
      return true;
    } else {
      return startWithSweetPose(safe_buffer, max_zigzag);
    }
  } else {
    if (startWithSweetPose(safe_buffer, max_zigzag)) {
      return true;
    } else {
      key_points_.clear();
      return startWithStraightLine(safe_buffer, max_zigzag);
    }
  }
}

// 行进间停车
bool PerpendicularRulePlanner::PlanWithBufferReverse(double safe_buffer,
                                                     int max_zigzag) {
  key_points_.clear();
  result_.clear();
  // a ll key_points need to checkCollision such as init_pose, turning_pose
  // std::cout<< init_pose.x <<"," << init_pose.y << "," << init_pose.theta <<
  // std::endl; printf("init pose theta: %f.\r\n", init_pose.theta); if
  // (init_pose.x >= 4.5 && init_pose.y >= 8 - 2.283
  // && init_pose.y <= 9.5 - 2.283)
  // 暂时不做区域判断
  if (true) {
    planning_math::Vec2d center;
    Pose2D plan_pose = init_pose;
    bool is_vehicle_zheng =
        init_pose.theta >= -0.025 && init_pose.theta <= 0.025;

    if (!is_vehicle_zheng) {
      Pose2D judge_pose;
      if (init_pose.theta > 0) {
        judge_pose = calcNextPose(obstacles, points_of_obstacles, init_pose,
                                  init_pose, 1, -1, veh_min_r);
        // printf("x: %f, y: %f, temp_pose.theta: %f\r\n", judge_pose.x,
        // judge_pose.y, judge_pose.theta);
        if (judge_pose.theta <= 0) {
          Pose2D temp_pose(init_pose.x - veh_min_r * sin(init_pose.theta),
                           init_pose.y - veh_min_r * (1 - cos(init_pose.theta)),
                           0);
          key_points_.push_back(temp_pose);
          plan_pose = temp_pose;
        }
      } else {
        judge_pose = calcNextPose(obstacles, points_of_obstacles, init_pose,
                                  init_pose, -1, -1, veh_min_r);
        if (judge_pose.theta >= 0) {
          Pose2D temp_pose(init_pose.x + veh_min_r * sin(init_pose.theta),
                           init_pose.y + veh_min_r * (1 - cos(init_pose.theta)),
                           0);
          key_points_.push_back(temp_pose);
          plan_pose = temp_pose;
        }
      }
    }

    Pose2D turning_pose = calcTurningPose(plan_pose, safe_buffer, center);
    if (std::isnan(turning_pose.x) || std::isnan(turning_pose.y) ||
        std::isnan(turning_pose.theta)) {
      return false;
    }

    if (turning_pose.x > plan_pose.x) {
      // return false;
      // printf("turning_pose is to the right of the plan_pose.\r\n");
      auto mid_pose = calcNextPose(obstacles, points_of_obstacles, plan_pose,
                                   plan_pose, -1, -1, veh_min_r);
      // 向右打满倒车
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
          if (planMidway(forward_pose, true)) {
            key_points_.insert(key_points_.begin(), mid_pose);
            key_points_.insert(key_points_.begin(), plan_pose);
            if (!is_vehicle_zheng)
              key_points_.insert(key_points_.begin(), init_pose);
            return true;
          }
          i++;
        }
      }
      return false;
    } else {
      key_points_.insert(key_points_.begin(), plan_pose);
      if (!is_vehicle_zheng)
        key_points_.insert(key_points_.begin(), init_pose);

      if (planOneShot(plan_pose))
        return true;

      if (!checkStraightLine(obstacles, points_of_obstacles, plan_pose,
                             turning_pose))
        return false;

      key_points_.push_back(turning_pose);

      if (calcCurvePath(turning_pose, max_zigzag))
        return true;

      return false;
    }
  } else
    return false;
}

bool PerpendicularRulePlanner::startWithStraightLine(double safe_buffer,
                                                     int max_zigzag) {
  key_points_.push_back(init_pose);
  planning_math::Vec2d center;
  Pose2D turning_pose = calcTurningPose(init_pose, safe_buffer, center);
  if (std::isnan(turning_pose.x) || std::isnan(turning_pose.y) ||
      std::isnan(turning_pose.theta)) {
    return false;
  }

  // if (P_1_.y() < VehicleParam::Instance()->length / 2 - back_to_rear &&
  //     planOneShot(init_pose)) {
  //   return true;
  // }
  if (planOneShot(init_pose)) {
    return true;
  }

  if (!checkStraightLine(obstacles, points_of_obstacles, init_pose,
                         turning_pose)) {
    return false;
  }
  key_points_.push_back(turning_pose);

  if (calcCurvePath(turning_pose, max_zigzag)) {
    return true;
  }
  return false;
}

bool PerpendicularRulePlanner::startWithSweetPose(double safe_buffer,
                                                  int max_zigzag) {
  double ratio = 2.0;
  bool flag = false;
  Pose2D turning_pose;
  planning_math::Vec2d center;
  for (; ratio > 0.99; ratio -= 0.1) {
    for (auto sw_pose : safe_sweet_poses_) {
      std::vector<Pose2D> head_kps;
      if (calcSCSAdjustPoses(init_pose, sw_pose, head_kps)) {
        key_points_ = head_kps;
        turning_pose = calcTurningPose(head_kps.back(), safe_buffer, center);
        if (std::isnan(turning_pose.x) || std::isnan(turning_pose.y) ||
            std::isnan(turning_pose.theta)) {
          continue;
        }
        if (!checkStraightLine(obstacles, points_of_obstacles, head_kps.back(),
                               turning_pose)) {
          continue;
        }
        flag = true;
        break;
      }

      key_points_.clear();
      key_points_.push_back(init_pose);

      if (calcAdjustPoses(sw_pose, ratio * veh_min_r)) {
        turning_pose = calcTurningPose(sw_pose, safe_buffer, center);
        if (std::isnan(turning_pose.x) || std::isnan(turning_pose.y) ||
            std::isnan(turning_pose.theta)) {
          continue;
        }
        if (!checkStraightLine(obstacles, points_of_obstacles,
                               key_points_.back(), turning_pose)) {
          continue;
        }
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

  key_points_.push_back(turning_pose);
  if (calcCurvePath(turning_pose, max_zigzag)) {
    return true;
  }
  return false;
}

bool PerpendicularRulePlanner::calcSCSAdjustPoses(
    const Pose2D &start_pose, const Pose2D &end_pose,
    std::vector<Pose2D> &head_kps) {
  head_kps.clear();
  if (start_pose.theta >= end_pose.theta) {
    return false;
  }

  double x0 = start_pose.x - veh_min_r * sin(start_pose.theta);
  double y0 = start_pose.y + veh_min_r * cos(start_pose.theta);

  double x1 = end_pose.x - veh_min_r * sin(end_pose.theta);
  double y1 = end_pose.y + veh_min_r * cos(end_pose.theta);

  double A0 = tan(start_pose.theta), A1 = tan(end_pose.theta);
  double B0 = -1, B1 = -1;
  double C0 = y0 - tan(start_pose.theta) * x0;
  double C1 = y1 - tan(end_pose.theta) * x1;

  if (fabs(A0 * B1 - A1 * B0) < 1e-3) {
    return false;
  }

  double center_x = (C1 * B0 - C0 * B1) / (A0 * B1 - A1 * B0);
  double center_y = (C1 * A0 - C0 * A1) / (A1 * B0 - A0 * B1);

  double steer_x = center_x + veh_min_r * sin(start_pose.theta);
  double steer_y = center_y - veh_min_r * cos(start_pose.theta);
  Pose2D steer_pose{steer_x, steer_y, start_pose.theta};
  if (steer_x < target_pose.x - 1.0 ||
      (steer_x < start_pose.x &&
       (std::hypot(steer_x - start_pose.x, steer_y - start_pose.y) < 0.1)) ||
      !checkStraightLine(obstacles, points_of_obstacles, start_pose,
                         steer_pose)) {
    return false;
  }

  double turned_x = center_x + veh_min_r * sin(end_pose.theta);
  double turned_y = center_y - veh_min_r * cos(end_pose.theta);
  Pose2D re_straight_pose{turned_x, turned_y, end_pose.theta};
  Pose2D front_limit_pose = calcNextPose(
      obstacles, points_of_obstacles, init_pose, steer_pose, 1, 1, veh_min_r);
  if (front_limit_pose.theta < re_straight_pose.theta ||
      steer_x > target_pose.x + 5.0) {
    return false;
  }

  head_kps.push_back(start_pose);
  head_kps.push_back(steer_pose);
  head_kps.push_back(re_straight_pose);

  return true;
}

bool PerpendicularRulePlanner::planOneShot(const Pose2D &start_pose) {
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

bool PerpendicularRulePlanner::calcCurvePath(const Pose2D &start_pose,
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
      if (middle_y < target_pose.y + 0.5) {
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
          if (middle_y < target_pose.y + 0.5 &&
              isNarrowChannelWidthScenario()) {
            double O_x = turning_pose.x + veh_min_r * sin(turning_pose.theta);
            double O_y = turning_pose.y - veh_min_r * cos(turning_pose.theta);
            key_points_.pop_back();
            key_points_.emplace_back(O_x - veh_min_r, O_y, target_pose.theta);
            key_points_.emplace_back(O_x - veh_min_r, target_pose.y,
                                     target_pose.theta);
            break;
          } else if (middle_y > target_pose.y + 0.5) {
            key_points_.emplace_back(target_pose.x, middle_y,
                                     target_pose.theta);
            key_points_.push_back(target_pose);
            break;
          }
        }
      }
    } else if (next_pose.theta > target_pose.theta &&
               next_pose.x > target_pose.x &&
               turning_pose.theta < target_pose.theta) {
      double denominator = 5.0001;
      int i = 0;
      for (; i <= denominator; ++i) {
        double pose_theta = (1 - i / denominator) * turning_pose.theta +
                            (i / denominator) * target_pose.theta;
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
        if (middle_y < target_pose.y + 0.5 || middle_y > upper_height_) {
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
    if (next_pose.y > upper_height_ || next_pose.y < target_pose.y ||
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
        if (middle_y < target_pose.y + 0.5 || middle_y > upper_height_) {
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
    if (turning_pose.y > upper_height_ || turning_pose.y < target_pose.y ||
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
  return i <= max_zigzag;
}

void PerpendicularRulePlanner::extractKeyInfo(
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
  planning_math::LineSegment2d oppo_line(
      planning_math::Vec2d(-10, 11 + up_bound_move_distance_),
      planning_math::Vec2d(10, 11 + up_bound_move_distance_));
  target_vel_ = input.init_state.v;

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
  auto local_slot_right_bound =
      planning_math::tf2d(local_frame_pose_, input.T_lines.slot_right_bound);

  if (is_on_left) {
    init_pose = MirrorInjection(init_pose);
    P_0_ = local_slot_right_bound.end();
    P_0_ = MirrorInjection(P_0_);
    P_1_ = local_slot_left_bound.end();
    P_1_ = MirrorInjection(P_1_);

    for (auto line : input.obstacle_lines) {
      auto local_line = planning_math::tf2d(local_frame_pose_, line);
      obstacles.push_back(MirrorInjection(local_line));
    }
    obstacles.push_back(oppo_line);

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
    obstacles.push_back(oppo_line);

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

  auto local_upper_line =
      planning_math::tf2d(local_frame_pose_, input.T_lines.road_upper_bound);
  upper_height_ = local_upper_line.min_y();
}

bool PerpendicularRulePlanner::isNarrowScenario() {
  double roi_min_x = -0.5 * slot_width_;
  double roi_max_x = 1.5 * slot_width_;

  // get upper height
  double upper_y = init_pose.y + 10;
  double lower_y = target_pose.y + front_to_rear - 2.0;
  for (const auto &point : points_of_obstacles) {
    if (point.x() > roi_min_x && point.x() < roi_max_x) {
      if (point.y() > init_pose.y) {
        upper_y = std::min(upper_y, point.y());
      } else {
        lower_y = std::max(lower_y, point.y());
      }
    }
  }

  return upper_y - lower_y < 5.0;
}

bool PerpendicularRulePlanner::isTwoSideIsEmptyScenario() {
  double inner_space_width =
      perpendicular_scenario_adapter_.get_inner_space_width();
  double outside_space_width =
      perpendicular_scenario_adapter_.get_outside_space_width();
  double half_slot_and_car_width =
      0.5 * slot_width_ + msquare::VehicleParam::Instance()->width * 0.5;
  if (inner_space_width > half_slot_and_car_width &&
      outside_space_width > half_slot_and_car_width) {
    return true;
  }
  return false;
}

bool PerpendicularRulePlanner::isInnerSideVehicle(
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

  planning_math::Box2d inner_box(
      planning_math::Vec2d(target_pose.x +
                               msquare::VehicleParam::Instance()->width,
                           target_pose.y),
      target_pose.theta, msquare::VehicleParam::Instance()->length,
      msquare::VehicleParam::Instance()->width);
  for (auto local_obs_box : local_obs_boxes) {
    if (inner_box.IsPointIn(local_obs_box.center())) {
      return true;
    }
  }
  return false;
}

bool PerpendicularRulePlanner::isNarrowChannelWidthScenario() {
  constexpr double NARROW_CHANNEL_WIDTH_TH = 5.0; // m
  return perpendicular_scenario_adapter_.get_channel_width() <
         NARROW_CHANNEL_WIDTH_TH;
}
bool PerpendicularRulePlanner::tooNearUpperWall(
    const Pose2D &turning_pose, const planning_math::Vec2d &center) {
  double left_front_corner_y = turning_pose.y +
                               front_corner_length * sin(turning_pose.theta) +
                               half_width * cos(turning_pose.theta);
  if (left_front_corner_y > upper_height_) {
    return true;
  }

  if (turning_pose.theta - gamma_4 < 0) {
    return center.y() + l_2 > upper_height_;
  }

  return false;
}

bool PerpendicularRulePlanner::calcAdjustPoses(const Pose2D &end_pose,
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
      if ((std::abs(r) > veh_min_r) &&
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

Pose2D
PerpendicularRulePlanner::calcTurningPose(const Pose2D &start_pose,
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
  // fixing an old bug
  double target_x = x_O1 - veh_min_r * sin(heading);
  double target_y = tan(heading) * (target_x - xs) + ys;

  return Pose2D(target_x, target_y, heading);
}

void PerpendicularRulePlanner::extendKeyPointsToResult(double step_size) {
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

Pose2D PerpendicularRulePlanner::calcExpectedPose(const Pose2D &start_pose,
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

Pose2D PerpendicularRulePlanner::calcExpectedPoseReverse(
    const Pose2D &start_pose, int steer_direction, int travel_direction,
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

double PerpendicularRulePlanner::calBackStraightDist(const Pose2D &start_pose) {
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
      if (x_1 > back_edge.min_x() && x_2 > back_edge.min_x()) {
        continue;
      }
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

double
PerpendicularRulePlanner::calForwardStraightDist(const Pose2D &start_pose) {
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
      if (x_1 < front_edge.max_x() && x_2 < front_edge.max_x()) {
        continue;
      }
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

double PerpendicularRulePlanner::calInnerSideDist(const Pose2D &start_pose) {
  Pose2D init_frame{start_pose.x, start_pose.y, start_pose.theta - M_PI_2};
  std::vector<planning_math::Vec2d> local_points;
  for (const auto &point : points_of_obstacles) {
    local_points.push_back(planning_math::tf2d(init_frame, point));
  }

  double min_dist = 1.0;
  for (const auto &local_pt : local_points) {
    if (local_pt.y() > 0 && local_pt.y() < front_to_rear && local_pt.x() > 0) {
      min_dist = std::min(min_dist, local_pt.x() - half_width);
    }
  }

  return min_dist;
}

void PerpendicularRulePlanner::getEPRectCorners(
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

void PerpendicularRulePlanner::generateSweetSpots() {
  double init_y = 6.0 + move_ditsance_;
  double step = 0.2;
  sweet_spot_[2.5] = {
      Pose2D(2.6, init_y + 3 * step, 0.175),
      Pose2D(3.0, init_y + 3 * step, 0.175),
      Pose2D(3.6, init_y + 3 * step, 0.175),
      Pose2D(4.0, init_y + 3 * step, 0.175),
      Pose2D(2.0, init_y + 2 * step, 0.175),
      Pose2D(2.6, init_y + 2 * step, 0.175),
      Pose2D(3.0, init_y + 2 * step, 0.175),
      Pose2D(3.6, init_y + 2 * step, 0.175),
      Pose2D(4.0, init_y + 2 * step, 0.175),
      Pose2D(4.6, init_y + 3 * step, 0.175),
      Pose2D(4.0, init_y + 4 * step, 0.175),
      Pose2D(4.6, init_y + 4 * step, 0.175),
      Pose2D(2.0, init_y + 1 * step, 0.175),
      Pose2D(2.6, init_y + 1 * step, 0.175),
      Pose2D(3.0, init_y + 1 * step, 0.175),
      Pose2D(3.6, init_y + 1 * step, 0.175),
      Pose2D(4.0, init_y + 1 * step, 0.175),
      Pose2D(4.6, init_y + 2 * step, 0.175),
      Pose2D(5.0, init_y + 3 * step, 0.175),
      Pose2D(5.0, init_y + 2 * step, 0.175),
      Pose2D(5.0, init_y + 4 * step, 0.175),
      Pose2D(2.0, init_y + 0 * step, 0.175),
      Pose2D(2.6, init_y + 0 * step, 0.175),
      Pose2D(3.0, init_y + 0 * step, 0.175),
      Pose2D(2.0, init_y + -1 * step, 0.175),
      Pose2D(5.6, init_y + 3 * step, 0.175),
      Pose2D(5.6, init_y + 4 * step, 0.175),
      Pose2D(5.6, init_y + 2 * step, 0.175),
      Pose2D(5.6, init_y + 5 * step, 0.175),
      Pose2D(6.0, init_y + 3 * step, 0.175),
      Pose2D(6.0, init_y + 4 * step, 0.175),
      Pose2D(6.0, init_y + 5 * step, 0.175),
      Pose2D(6.0, init_y + 6 * step, 0.175),
      Pose2D(6.6, init_y + 3 * step, 0.175),
      Pose2D(6.6, init_y + 4 * step, 0.175),
      Pose2D(6.6, init_y + 5 * step, 0.175),
      Pose2D(6.6, init_y + 6 * step, 0.175),
      Pose2D(7.0, init_y + 3 * step, 0.175),
      Pose2D(7.0, init_y + 4 * step, 0.175),
      Pose2D(7.0, init_y + 5 * step, 0.175),
      Pose2D(7.0, init_y + 6 * step, 0.175),
      Pose2D(7.6, init_y + 4 * step, 0.175),
      Pose2D(7.6, init_y + 5 * step, 0.175),
      Pose2D(7.6, init_y + 6 * step, 0.175),
      Pose2D(7.6, init_y + 7 * step, 0.175),
      Pose2D(8.0, init_y + 4 * step, 0.175),
      Pose2D(8.0, init_y + 5 * step, 0.175),
      Pose2D(8.0, init_y + 6 * step, 0.175),
      Pose2D(8.0, init_y + 7 * step, 0.175),
  };

  sweet_spot_[2.6] = {Pose2D(3.0, init_y + 3 * step, 0.175),
                      Pose2D(3.6, init_y + 3 * step, 0.175),
                      Pose2D(4.0, init_y + 3 * step, 0.175),
                      Pose2D(2.0, init_y + 2 * step, 0.175),
                      Pose2D(2.6, init_y + 2 * step, 0.175),
                      Pose2D(3.0, init_y + 2 * step, 0.175),
                      Pose2D(3.6, init_y + 2 * step, 0.175),
                      Pose2D(4.0, init_y + 2 * step, 0.175),
                      Pose2D(4.6, init_y + 3 * step, 0.175),
                      Pose2D(4.6, init_y + 4 * step, 0.175),
                      Pose2D(2.0, init_y + 1 * step, 0.175),
                      Pose2D(2.6, init_y + 1 * step, 0.175),
                      Pose2D(3.0, init_y + 1 * step, 0.175),
                      Pose2D(3.6, init_y + 1 * step, 0.175),
                      Pose2D(4.0, init_y + 1 * step, 0.175),
                      Pose2D(4.6, init_y + 2 * step, 0.175),
                      Pose2D(5.0, init_y + 3 * step, 0.175),
                      Pose2D(5.0, init_y + 2 * step, 0.175),
                      Pose2D(5.0, init_y + 4 * step, 0.175),
                      Pose2D(2.0, init_y + 0 * step, 0.175),
                      Pose2D(2.6, init_y + 0 * step, 0.175),
                      Pose2D(5.6, init_y + 3 * step, 0.175),
                      Pose2D(5.6, init_y + 4 * step, 0.175),
                      Pose2D(5.6, init_y + 5 * step, 0.175),
                      Pose2D(6.0, init_y + 3 * step, 0.175),
                      Pose2D(6.0, init_y + 4 * step, 0.175),
                      Pose2D(6.0, init_y + 5 * step, 0.175),
                      Pose2D(6.6, init_y + 4 * step, 0.175),
                      Pose2D(6.6, init_y + 5 * step, 0.175),
                      Pose2D(7.0, init_y + 4 * step, 0.175),
                      Pose2D(7.0, init_y + 5 * step, 0.175),
                      Pose2D(7.0, init_y + 6 * step, 0.175),
                      Pose2D(7.6, init_y + 5 * step, 0.175),
                      Pose2D(7.6, init_y + 6 * step, 0.175),
                      Pose2D(8.0, init_y + 5 * step, 0.175),
                      Pose2D(8.0, init_y + 6 * step, 0.175)};

  sweet_spot_[2.7] = {Pose2D(2.6, init_y + 2 * step, 0.175),
                      Pose2D(3.0, init_y + 2 * step, 0.175),
                      Pose2D(3.6, init_y + 2 * step, 0.175),
                      Pose2D(4.0, init_y + 2 * step, 0.175),
                      Pose2D(2.0, init_y + 2 * step, 0.175),
                      Pose2D(2.6, init_y + 2 * step, 0.175),
                      Pose2D(3.0, init_y + 2 * step, 0.175),
                      Pose2D(3.6, init_y + 2 * step, 0.175),
                      Pose2D(4.6, init_y + 2 * step, 0.175),
                      Pose2D(3.6, init_y + 3 * step, 0.175),
                      Pose2D(4.0, init_y + 3 * step, 0.175),
                      Pose2D(4.6, init_y + 3 * step, 0.175),
                      Pose2D(5.0, init_y + 3 * step, 0.175),
                      Pose2D(2.0, init_y + 0 * step, 0.175),
                      Pose2D(5.6, init_y + 3 * step, 0.175),
                      Pose2D(6.0, init_y + 3 * step, 0.175),
                      Pose2D(5.0, init_y + 4 * step, 0.175),
                      Pose2D(5.6, init_y + 4 * step, 0.175),
                      Pose2D(6.0, init_y + 4 * step, 0.175),
                      Pose2D(6.0, init_y + 5 * step, 0.175),
                      Pose2D(6.6, init_y + 4 * step, 0.175),
                      Pose2D(6.6, init_y + 5 * step, 0.175),
                      Pose2D(7.0, init_y + 4 * step, 0.175),
                      Pose2D(7.0, init_y + 5 * step, 0.175),
                      Pose2D(7.6, init_y + 5 * step, 0.175),
                      Pose2D(7.6, init_y + 6 * step, 0.175),
                      Pose2D(8.0, init_y + 5 * step, 0.175),
                      Pose2D(8.0, init_y + 6 * step, 0.175)};

  sweet_spot_[2.8] = {Pose2D(3.0, init_y + 2 * step, 0.175),
                      Pose2D(3.6, init_y + 2 * step, 0.175),
                      Pose2D(4.0, init_y + 2 * step, 0.175),
                      Pose2D(2.0, init_y + 1 * step, 0.175),
                      Pose2D(2.6, init_y + 1 * step, 0.175),
                      Pose2D(3.0, init_y + 1 * step, 0.175),
                      Pose2D(3.6, init_y + 1 * step, 0.175),
                      Pose2D(4.6, init_y + 2 * step, 0.175),
                      Pose2D(4.6, init_y + 3 * step, 0.175),
                      Pose2D(5.0, init_y + 2 * step, 0.175),
                      Pose2D(5.0, init_y + 3 * step, 0.175),
                      Pose2D(2.0, init_y + 0 * step, 0.175),
                      Pose2D(2.6, init_y + 0 * step, 0.175),
                      Pose2D(5.6, init_y + 3 * step, 0.175),
                      Pose2D(5.6, init_y + 4 * step, 0.175),
                      Pose2D(6.0, init_y + 3 * step, 0.175),
                      Pose2D(6.0, init_y + 4 * step, 0.175),
                      Pose2D(6.6, init_y + 4 * step, 0.175),
                      Pose2D(7.0, init_y + 4 * step, 0.175),
                      Pose2D(7.0, init_y + 5 * step, 0.175),
                      Pose2D(7.6, init_y + 5 * step, 0.175),
                      Pose2D(8.0, init_y + 5 * step, 0.175)};

  sweet_spot_[2.9] = {Pose2D(3.6, init_y + 2 * step, 0.175),
                      Pose2D(4.0, init_y + 2 * step, 0.175),
                      Pose2D(2.6, init_y + 1 * step, 0.175),
                      Pose2D(3.0, init_y + 1 * step, 0.175),
                      Pose2D(3.6, init_y + 1 * step, 0.175),
                      Pose2D(4.6, init_y + 2 * step, 0.175),
                      Pose2D(4.6, init_y + 3 * step, 0.175),
                      Pose2D(5.0, init_y + 2 * step, 0.175),
                      Pose2D(5.0, init_y + 3 * step, 0.175),
                      Pose2D(2.0, init_y + 0 * step, 0.175),
                      Pose2D(2.6, init_y + 0 * step, 0.175),
                      Pose2D(5.6, init_y + 3 * step, 0.175),
                      Pose2D(6.0, init_y + 3 * step, 0.175),
                      Pose2D(6.0, init_y + 4 * step, 0.175),
                      Pose2D(6.6, init_y + 4 * step, 0.175),
                      Pose2D(7.0, init_y + 4 * step, 0.175),
                      Pose2D(7.6, init_y + 5 * step, 0.175),
                      Pose2D(8.0, init_y + 5 * step, 0.175)};

  sweet_spot_[3.0] = {Pose2D(4.0, init_y + 2 * step, 0.175),
                      Pose2D(3.0, init_y + 1 * step, 0.175),
                      Pose2D(3.6, init_y + 1 * step, 0.175),
                      Pose2D(4.0, init_y + 1 * step, 0.175),
                      Pose2D(4.6, init_y + 2 * step, 0.175),
                      Pose2D(5.0, init_y + 2 * step, 0.175),
                      Pose2D(2.0, init_y + 0 * step, 0.175),
                      Pose2D(2.6, init_y + 0 * step, 0.175),
                      Pose2D(3.0, init_y + 0 * step, 0.175),
                      Pose2D(5.6, init_y + 3 * step, 0.175),
                      Pose2D(6.0, init_y + 3 * step, 0.175),
                      Pose2D(6.6, init_y + 3.5 * step, 0.175),
                      Pose2D(7.0, init_y + 4 * step, 0.175),
                      Pose2D(7.6, init_y + 4.5 * step, 0.175),
                      Pose2D(8.0, init_y + 4.5 * step, 0.175)};
}

parking::OpenspaceDeciderOutput OdoAdaptor::generateNewOdo() {

  auto movePoint = [](const planning_math::Vec2d &init_point,
                      const double moveDis, const double moveTheta) {
    planning_math::Vec2d new_point;
    new_point.set_x(init_point.x() + moveDis * cos(moveTheta));
    new_point.set_y(init_point.y() + moveDis * sin(moveTheta));
    return new_point;
  };

  // auto coutSegmentLineInfo = [](const planning_math::LineSegment2d &line,
  //                               std::string line_name) {
  //   std::cout << "====" << line_name << " " << line.start().x() << " "
  //             << line.start().y() << " " << line.end().x() << " "
  //             << line.end().y() << std::endl;
  // };

  // auto coutOdoTLineInfo = [&coutSegmentLineInfo](
  //                             const parking::TshapedAreaLines &t_line) {
  //   coutSegmentLineInfo(t_line.road_lower_left_bound,
  //   "road_lower_left_bound"); coutSegmentLineInfo(t_line.slot_left_bound, "
  //   slot_left_bound"); coutSegmentLineInfo(t_line.slot_right_bound, "
  //   slot_right_bound"); coutSegmentLineInfo(t_line.road_lower_right_bound,
  //                       "road_lower_right_bound");
  //   coutSegmentLineInfo(t_line.road_upper_bound, "road_upper_bound ");
  // };

  parking::OpenspaceDeciderOutput odo;
  const auto &road_line = odo_.T_lines.road_lower_right_bound;
  // coutOdoTLineInfo(odo_.T_lines);
  // std::cout << "pt is:" << road_line.start().x() << " "
  //           << road_line.start().y() << std::endl;
  const auto &t_lines = odo_.T_lines;
  const auto &obs_points = odo_.points;
  std::vector<planning_math::Vec2d> slot_point_frame_obs_pts;
  // std::cout << "slot frame is:" << right_boundary.end().x()
  //           << " " << right_boundary.end().y()
  //           << " " << right_boundary.heading() << std::endl;
  double right_max_obs_y = 0;
  auto init_slot_right_bound = odo_.T_lines.slot_right_bound;
  auto init_slot_left_bound = odo_.T_lines.slot_left_bound;
  auto init_road_upper_bound = odo_.T_lines.road_upper_bound;
  auto init_road_lower_left_bound = odo_.T_lines.road_lower_left_bound;
  auto init_road_lower_right_bound = odo_.T_lines.road_lower_right_bound;

  Pose2D local_frame_pose_ =
      Pose2D(odo_.init_state.path_point.x, odo_.init_state.path_point.y,
             odo_.init_state.path_point.theta - M_PI_2);

  // get local poses
  Pose2D init_pose = planning_math::tf2d(
      local_frame_pose_,
      Pose2D(odo_.target_state.path_point.x, odo_.target_state.path_point.y,
             odo_.target_state.path_point.theta));
  Pose2D target_pose = planning_math::tf2d(
      local_frame_pose_,
      Pose2D(odo_.init_state.path_point.x, odo_.init_state.path_point.y,
             odo_.init_state.path_point.theta));
  Pose2D ego_pose(odo_.target_state.path_point.x,
                  odo_.target_state.path_point.y,
                  odo_.target_state.path_point.theta);

  planning_math::Vec2d init_vec(cos(init_pose.theta), sin(init_pose.theta));
  planning_math::Vec2d i2t_vec(target_pose.x - init_pose.x,
                               target_pose.y - init_pose.y);
  bool is_on_left = init_vec.CrossProd(i2t_vec) > 0;

  // std::cout << "is_on_left:" << is_on_left << std::endl;

  const auto &pillar_scene = identifySpecialPillarScene(is_on_left);

  // std::cout << "the pillar scene is:" << int(pillar_scene) << std::endl;

  if (pillar_scene == PillarInvadeType::NO_PILLAR_INVADE) {
    move_distance_ = 0.0;
    up_bound_move_distance_ = 0.0;
    return odo_;
  }

  if (pillar_scene == PillarInvadeType::OUT_SIDE_PILLAR_INVADE_ONLY) {
    handleOutSidePillar();
    return odo_;
  }

  Pose2D P_1;
  if (is_on_left) {
    const auto &left_boundary = t_lines.slot_left_bound;
    P_1.x = left_boundary.start().x();
    P_1.y = left_boundary.start().y();
    P_1.theta = left_boundary.heading() + M_PI_2;
  } else {
    const auto &right_boundary = t_lines.slot_right_bound;
    P_1.x = right_boundary.end().x();
    P_1.y = right_boundary.end().y();
    P_1.theta = right_boundary.heading() - M_PI_2;
  }

  // std::cout << "P_1:" << P_1.x << " " << P_1.y << std::endl;

  // std::cout << "========the ego pose is:" << ego_pose.x
  //           << "  " << ego_pose.y << std::endl;
  const auto &ego_pose_in_frame =
      planning_math::tf2d(P_1, planning_math::Vec2d(ego_pose.x, ego_pose.y));
  // std::cout << "the ego pose in P_1 frame is:" << ego_pose_in_frame.x()
  //           << "  " << ego_pose_in_frame.y() << std::endl;

  double x_max_limit = 4.5;
  for (const auto &obs_pt : obs_points) {
    // std::cout << "the slot_pt_frame_obs_pt:" << obs_pt.x()
    //           << " " << obs_pt.y() << std::endl;
    const auto &slot_pt_frame_obs_pt = planning_math::tf2d(P_1, obs_pt);
    // std::cout << "the slot_pt_frame_obs_pt:" << slot_pt_frame_obs_pt.x()
    //           << " " << slot_pt_frame_obs_pt.y() << std::endl;
    slot_point_frame_obs_pts.emplace_back(std::move(slot_pt_frame_obs_pt));
    if (slot_pt_frame_obs_pt.x() > 0 && slot_pt_frame_obs_pt.y() > 0 &&
        slot_pt_frame_obs_pt.x() < x_max_limit &&
        slot_pt_frame_obs_pt.y() < ego_pose_in_frame.y()) {
      right_max_obs_y = std::max(right_max_obs_y, slot_pt_frame_obs_pt.y());
    }
  }

  double left_max_obs_y = 0;
  for (const auto &obs_pt : obs_points) {
    // std::cout << "the slot_pt_frame_obs_pt:" << obs_pt.x()
    //           << " " << obs_pt.y() << std::endl;
    const auto &slot_pt_frame_obs_pt = planning_math::tf2d(P_1, obs_pt);
    // std::cout << "the slot_pt_frame_obs_pt:" << slot_pt_frame_obs_pt.x()
    //           << " " << slot_pt_frame_obs_pt.y() << std::endl;
    slot_point_frame_obs_pts.emplace_back(std::move(slot_pt_frame_obs_pt));
    if (slot_pt_frame_obs_pt.x() < 0 && slot_pt_frame_obs_pt.y() > 0 &&
        slot_pt_frame_obs_pt.x() > -x_max_limit &&
        slot_pt_frame_obs_pt.y() < ego_pose_in_frame.y()) {
      left_max_obs_y = std::max(left_max_obs_y, slot_pt_frame_obs_pt.y());
    }
  }

  // std::cout << " the left move is:" << left_max_obs_y << std::endl;
  // std::cout << " the right move is:" << right_max_obs_y << std::endl;

  double move_distance = 0.0;
  double max_obs_y = 0.0;
  if (is_on_left) {
    max_obs_y = left_max_obs_y;
  } else {
    max_obs_y = right_max_obs_y;
  }
  double move_theta = 0.0;
  double p_0_move_dis = 0.0;
  planning_math::Vec2d new_boundary_pt(0, max_obs_y);
  const auto &new_pt = planning_math::tf2d_inv(P_1, new_boundary_pt);
  // std::cout << "the new pt is:" << new_pt.x() << " " << new_pt.y() <<
  // std::endl;

  // move_distance = right_max_obs_y;
  move_distance = max_obs_y;

  double offset = 1.5;
  if (is_on_left) {
    // double move_distance = right_max_obs_y;
    move_theta = t_lines.slot_left_bound.heading() + M_PI;
    planning_math::LineSegment2d new_slot_left_bound(
        new_pt, t_lines.slot_right_bound.end());

    const auto &right_bound_in_P_1_frame =
        planning_math::tf2d(P_1, odo_.T_lines.slot_right_bound.end());
    if (right_bound_in_P_1_frame.y() > max_obs_y) {
      p_0_move_dis = 0.0;
    } else {
      p_0_move_dis = max_obs_y - right_bound_in_P_1_frame.y();
    }

    const auto &P_1_in_target_frame = planning_math::tf2d(
        local_frame_pose_, planning_math::Vec2d(new_pt.x(), new_pt.y()));
    double left_move_distance = P_1_in_target_frame.y() - 6 + offset;
    if (left_move_distance < 0) {
      move_distance_ = 0.0;
      up_bound_move_distance_ = 0.0;
      return odo_;
    }

    // std::cout << "the p_0_move_dis:" << p_0_move_dis << std::endl;
    odo_.T_lines.slot_left_bound = new_slot_left_bound;

    odo_.T_lines.road_lower_left_bound = planning_math::LineSegment2d(
        movePoint(init_road_lower_left_bound.start(), move_distance,
                  move_theta),
        new_slot_left_bound.start());

    odo_.T_lines.slot_right_bound = planning_math::LineSegment2d(
        odo_.T_lines.slot_right_bound.start(),
        movePoint(init_slot_right_bound.end(), p_0_move_dis, move_theta));

    odo_.T_lines.road_lower_right_bound = planning_math::LineSegment2d(
        odo_.T_lines.slot_right_bound.end(),
        movePoint(init_road_lower_right_bound.end(), p_0_move_dis, move_theta));

    odo_.T_lines.road_upper_bound = planning_math::LineSegment2d(
        movePoint(init_road_upper_bound.start(), left_move_distance,
                  move_theta),
        movePoint(init_road_upper_bound.end(), left_move_distance, move_theta));

    up_bound_move_distance_ = left_move_distance;

  } else {

    move_theta = t_lines.slot_left_bound.heading() + M_PI;

    const auto &left_bound_in_P_1_frame =
        planning_math::tf2d(P_1, odo_.T_lines.slot_left_bound.start());
    // std::cout << "the left_bound_in_P_1_frame" << left_bound_in_P_1_frame.x()
    // << " "
    //           << left_bound_in_P_1_frame.y() << std::endl;
    if (left_bound_in_P_1_frame.y() > right_max_obs_y) {
      p_0_move_dis = 0.0;
    } else {
      p_0_move_dis = right_max_obs_y - left_bound_in_P_1_frame.y();
    }

    const auto &P_1_in_target_frame = planning_math::tf2d(
        local_frame_pose_, planning_math::Vec2d(new_pt.x(), new_pt.y()));
    double right_move_distance = P_1_in_target_frame.y() - 6 + offset;
    if (right_move_distance < 0) {
      move_distance_ = 0.0;
      up_bound_move_distance_ = 0.0;
      return odo_;
    }

    planning_math::LineSegment2d new_slot_right_bound(
        t_lines.slot_right_bound.start(), new_pt);

    odo_.T_lines.road_lower_left_bound = planning_math::LineSegment2d(
        movePoint(init_road_lower_left_bound.start(), p_0_move_dis, move_theta),
        movePoint(init_road_lower_left_bound.end(), p_0_move_dis, move_theta));

    odo_.T_lines.slot_left_bound = planning_math::LineSegment2d(
        odo_.T_lines.road_lower_left_bound.end(), init_slot_left_bound.end());

    odo_.T_lines.slot_right_bound = new_slot_right_bound;

    odo_.T_lines.road_lower_right_bound = planning_math::LineSegment2d(
        odo_.T_lines.slot_right_bound.end(),
        movePoint(init_road_lower_right_bound.end(), move_distance,
                  move_theta));

    odo_.T_lines.road_upper_bound = planning_math::LineSegment2d(
        movePoint(init_road_upper_bound.start(), right_move_distance,
                  move_theta),
        movePoint(init_road_upper_bound.end(), right_move_distance,
                  move_theta));

    up_bound_move_distance_ = right_move_distance;
  }
  const auto &P_1_in_target_frame = planning_math::tf2d(
      local_frame_pose_, planning_math::Vec2d(new_pt.x(), new_pt.y()));
  // std::cout << "the target pos is:" << local_frame_pose_.x
  //           << " " << local_frame_pose_.y << std::endl;
  // std::cout << "the P_1_in_target_frame:" << P_1_in_target_frame.x()
  //           << " " << P_1_in_target_frame.y() << std::endl;
  move_distance_ = P_1_in_target_frame.y() - 6 + offset;
  // std::cout << "------------> move distance is:" <<  move_distance_ <<
  // std::endl; std::cout << "------------> up bound distance is:" <<
  // up_bound_move_distance_ << std::endl; coutOdoTLineInfo(odo_.T_lines);
  return odo_;
}

OdoAdaptor::PillarInvadeType
OdoAdaptor::identifySpecialPillarScene(bool is_on_left) {
  Pose2D local_frame_pose_ =
      Pose2D(odo_.init_state.path_point.x, odo_.init_state.path_point.y,
             odo_.init_state.path_point.theta - M_PI_2);
  Pose2D init_pose = planning_math::tf2d(
      local_frame_pose_,
      Pose2D(odo_.target_state.path_point.x, odo_.target_state.path_point.y,
             odo_.target_state.path_point.theta));
  double x_limit = 6.0;
  const auto &obs_points = odo_.points;
  bool is_left_has_pillar = false;
  bool is_right_has_pillar = false;
  for (const auto &obs_pt : obs_points) {
    const auto &slot_pt_frame_obs_pt =
        planning_math::tf2d(local_frame_pose_, obs_pt);
    if (slot_pt_frame_obs_pt.x() < 0 && slot_pt_frame_obs_pt.y() > 4.5 &&
        slot_pt_frame_obs_pt.x() > -x_limit &&
        slot_pt_frame_obs_pt.y() < init_pose.y) {
      // std::cout << "----------------> Left Pillar" << std::endl;
      is_left_has_pillar = true;
      break;
    }
  }

  for (const auto &obs_pt : obs_points) {
    const auto &slot_pt_frame_obs_pt =
        planning_math::tf2d(local_frame_pose_, obs_pt);
    if (slot_pt_frame_obs_pt.x() > 0 && slot_pt_frame_obs_pt.y() > 4.5 &&
        slot_pt_frame_obs_pt.x() < x_limit &&
        slot_pt_frame_obs_pt.y() < init_pose.y) {
      // std::cout << "---------------->Right Pillar" << std::endl;
      is_right_has_pillar = true;
      break;
    }
  }

  PillarInvadeType pillar_type = PillarInvadeType::NO_PILLAR_INVADE;

  if (is_left_has_pillar && is_right_has_pillar) {
    pillar_type = PillarInvadeType::OUT_AND_IN_SIDE_PILLAR_INVADE;
  }

  if (is_on_left && is_left_has_pillar && !is_right_has_pillar) {
    pillar_type = PillarInvadeType::IN_SIDE_PILLAR_INVADE_ONLY;
  }

  if (!is_on_left && is_right_has_pillar && !is_left_has_pillar) {
    pillar_type = PillarInvadeType::IN_SIDE_PILLAR_INVADE_ONLY;
  }

  if (is_on_left && !is_left_has_pillar && is_right_has_pillar) {
    pillar_type = PillarInvadeType::OUT_SIDE_PILLAR_INVADE_ONLY;
  }

  if (!is_on_left && is_left_has_pillar && !is_right_has_pillar) {
    pillar_type = PillarInvadeType::OUT_SIDE_PILLAR_INVADE_ONLY;
  }

  return pillar_type;
}

void OdoAdaptor::handleOutSidePillar() { return; }
} // namespace msquare