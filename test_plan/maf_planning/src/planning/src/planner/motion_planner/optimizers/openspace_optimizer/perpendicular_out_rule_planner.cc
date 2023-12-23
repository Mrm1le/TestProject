#include "planner/motion_planner/optimizers/openspace_optimizer/perpendicular_out_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/rule_planner_util.h"

namespace msquare {

PerpendicularOutRulePlanner::PerpendicularOutRulePlanner(
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

  l_1 = veh_min_r - half_width;
  l_2 = std::hypot(veh_min_r + half_width, front_corner_length);
  gamma_4 = atan2(front_to_rear, veh_min_r + half_width);

  slot_width_ = floor((P_1_.x() - P_0_.x()) * 10) / 10;
  if (slot_width_ < 2.5) {
    slot_width_ = 2.5;
  } else if (slot_width_ > 2.8) {
    slot_width_ = 2.8;
  }

  sweet_spot_[2.5] = {
      Pose2D(2.0, 6.0, 0.175), Pose2D(2.0, 6.2, 0.175), Pose2D(2.0, 6.4, 0.175),
      Pose2D(2.0, 6.6, 0.175), Pose2D(2.6, 6.2, 0.175), Pose2D(2.6, 6.4, 0.175),
      Pose2D(2.6, 6.6, 0.175), Pose2D(3.0, 6.2, 0.175), Pose2D(3.0, 6.4, 0.175),
      Pose2D(3.0, 6.6, 0.175), Pose2D(3.0, 6.8, 0.175), Pose2D(3.6, 6.4, 0.175),
      Pose2D(3.6, 6.6, 0.175), Pose2D(3.6, 6.8, 0.175), Pose2D(4.0, 6.4, 0.175),
      Pose2D(4.0, 6.6, 0.175), Pose2D(4.0, 6.8, 0.175), Pose2D(4.6, 6.6, 0.175),
      Pose2D(4.6, 6.8, 0.175), Pose2D(4.6, 7.0, 0.175), Pose2D(5.0, 6.6, 0.175),
      Pose2D(5.0, 6.8, 0.175), Pose2D(5.0, 7.0, 0.175), Pose2D(5.6, 6.6, 0.175),
      Pose2D(5.5, 6.8, 0.175), Pose2D(5.6, 7.0, 0.175), Pose2D(5.6, 7.2, 0.175),
      Pose2D(6.0, 6.8, 0.175), Pose2D(6.0, 7.0, 0.175), Pose2D(6.0, 7.2, 0.175),
      Pose2D(6.6, 7.0, 0.175), Pose2D(6.6, 7.2, 0.175), Pose2D(7.0, 7.0, 0.175),
      Pose2D(7.0, 7.2, 0.175), Pose2D(7.5, 7.0, 0.175), Pose2D(7.5, 7.2, 0.175),
      Pose2D(7.5, 7.4, 0.175), Pose2D(8.0, 7.2, 0.175)};

  sweet_spot_[2.6] = {
      Pose2D(2.0, 6.0, 0.175), Pose2D(2.0, 6.2, 0.175), Pose2D(2.0, 6.4, 0.175),
      Pose2D(2.0, 6.6, 0.175), Pose2D(2.6, 5.8, 0.175), Pose2D(2.6, 6.0, 0.175),
      Pose2D(2.6, 6.4, 0.175), Pose2D(2.6, 6.6, 0.175), Pose2D(3.0, 6.0, 0.175),
      Pose2D(3.0, 6.2, 0.175), Pose2D(3.0, 6.4, 0.175), Pose2D(3.0, 6.6, 0.175),
      Pose2D(3.0, 6.8, 0.175), Pose2D(3.6, 6.0, 0.175), Pose2D(3.6, 6.2, 0.175),
      Pose2D(3.6, 6.6, 0.175), Pose2D(3.6, 6.8, 0.175), Pose2D(4.0, 6.2, 0.175),
      Pose2D(4.0, 6.4, 0.175), Pose2D(4.0, 6.6, 0.175), Pose2D(4.0, 6.8, 0.175),
      Pose2D(4.0, 7.0, 0.175), Pose2D(4.6, 6.2, 0.175), Pose2D(4.6, 6.4, 0.175),
      Pose2D(4.6, 6.8, 0.175), Pose2D(4.6, 7.0, 0.175), Pose2D(5.0, 6.2, 0.175),
      Pose2D(5.0, 6.4, 0.175), Pose2D(5.0, 6.6, 0.175), Pose2D(5.0, 6.8, 0.175),
      Pose2D(5.0, 7.0, 0.175), Pose2D(5.6, 6.6, 0.175), Pose2D(5.5, 6.8, 0.175),
      Pose2D(5.6, 7.0, 0.175), Pose2D(6.0, 6.4, 0.175), Pose2D(6.0, 6.6, 0.175),
      Pose2D(6.0, 7.0, 0.175), Pose2D(6.0, 7.2, 0.175), Pose2D(6.6, 6.8, 0.175),
      Pose2D(6.6, 7.0, 0.175), Pose2D(6.6, 7.2, 0.175), Pose2D(7.0, 6.6, 0.175),
      Pose2D(7.0, 6.8, 0.175), Pose2D(7.0, 7.4, 0.175), Pose2D(7.5, 7.0, 0.175),
      Pose2D(7.5, 7.2, 0.175), Pose2D(7.5, 7.4, 0.175), Pose2D(8.0, 6.8, 0.175),
      Pose2D(8.0, 7.0, 0.175), Pose2D(8.0, 7.4, 0.175)};

  sweet_spot_[2.7] = {Pose2D(2.0, 6.0, 0.175), Pose2D(2.0, 6.2, 0.175),
                      Pose2D(2.0, 6.4, 0.175), Pose2D(2.0, 6.6, 0.175),
                      Pose2D(2.6, 5.8, 0.175), Pose2D(2.6, 6.0, 0.175),
                      Pose2D(2.6, 6.2, 0.175), Pose2D(2.6, 6.4, 0.175),
                      Pose2D(2.6, 6.6, 0.175), Pose2D(3.0, 5.8, 0.175),
                      Pose2D(3.0, 6.2, 0.175), Pose2D(3.0, 6.4, 0.175),
                      Pose2D(3.0, 6.6, 0.175), Pose2D(3.0, 6.8, 0.175),
                      Pose2D(3.6, 6.0, 0.175), Pose2D(3.6, 6.2, 0.175),
                      Pose2D(3.6, 6.4, 0.175), Pose2D(3.6, 6.6, 0.175),
                      Pose2D(3.6, 6.8, 0.175), Pose2D(4.0, 6.0, 0.175),
                      Pose2D(4.0, 6.4, 0.175), Pose2D(4.0, 6.6, 0.175),
                      Pose2D(4.0, 6.8, 0.175), Pose2D(4.6, 6.0, 0.175),
                      Pose2D(4.6, 6.2, 0.175), Pose2D(4.6, 6.4, 0.175),
                      Pose2D(4.6, 6.6, 0.175), Pose2D(4.6, 6.8, 0.175),
                      Pose2D(4.6, 7.0, 0.175), Pose2D(5.0, 6.2, 0.175),
                      Pose2D(5.0, 6.4, 0.175), Pose2D(5.0, 6.6, 0.175),
                      Pose2D(5.0, 6.8, 0.175), Pose2D(5.0, 7.0, 0.175),
                      Pose2D(5.6, 6.2, 0.175), Pose2D(5.6, 6.4, 0.175),
                      Pose2D(5.6, 6.6, 0.175), Pose2D(5.5, 6.8, 0.175),
                      Pose2D(5.6, 7.0, 0.175), Pose2D(5.6, 7.2, 0.175),
                      Pose2D(6.0, 6.4, 0.175), Pose2D(6.0, 6.6, 0.175),
                      Pose2D(6.0, 6.8, 0.175), Pose2D(6.0, 7.0, 0.175),
                      Pose2D(6.0, 7.2, 0.175), Pose2D(6.6, 6.8, 0.175),
                      Pose2D(6.6, 7.0, 0.175), Pose2D(6.6, 7.2, 0.175),
                      Pose2D(7.0, 6.6, 0.175), Pose2D(7.0, 6.8, 0.175),
                      Pose2D(7.0, 7.0, 0.175), Pose2D(7.0, 7.2, 0.175),
                      Pose2D(7.5, 7.0, 0.175), Pose2D(7.5, 7.2, 0.175),
                      Pose2D(7.5, 7.4, 0.175), Pose2D(8.0, 6.6, 0.175),
                      Pose2D(8.0, 6.8, 0.175), Pose2D(8.0, 7.0, 0.175),
                      Pose2D(8.0, 7.2, 0.175), Pose2D(8.0, 7.4, 0.175)};

  sweet_spot_[2.8] = {
      Pose2D(2.0, 5.6, 0.175), Pose2D(2.0, 5.8, 0.175), Pose2D(2.0, 6.0, 0.175),
      Pose2D(2.0, 6.2, 0.175), Pose2D(2.0, 6.4, 0.175), Pose2D(2.0, 6.6, 0.175),
      Pose2D(2.6, 5.8, 0.175), Pose2D(2.6, 6.0, 0.175), Pose2D(2.6, 6.2, 0.175),
      Pose2D(2.6, 6.4, 0.175), Pose2D(2.6, 6.6, 0.175), Pose2D(3.0, 6.0, 0.175),
      Pose2D(3.0, 6.2, 0.175), Pose2D(3.0, 6.4, 0.175), Pose2D(3.0, 6.6, 0.175),
      Pose2D(3.0, 6.8, 0.175), Pose2D(3.6, 6.0, 0.175), Pose2D(3.6, 6.2, 0.175),
      Pose2D(3.6, 6.4, 0.175), Pose2D(3.6, 6.6, 0.175), Pose2D(3.6, 6.8, 0.175),
      Pose2D(4.0, 6.2, 0.175), Pose2D(4.0, 6.4, 0.175), Pose2D(4.0, 6.6, 0.175),
      Pose2D(4.0, 6.8, 0.175), Pose2D(4.6, 6.0, 0.175), Pose2D(4.6, 6.2, 0.175),
      Pose2D(4.6, 6.4, 0.175), Pose2D(4.6, 6.6, 0.175), Pose2D(4.6, 6.8, 0.175),
      Pose2D(4.6, 7.0, 0.175), Pose2D(5.0, 6.0, 0.175), Pose2D(5.0, 6.4, 0.175),
      Pose2D(5.0, 6.6, 0.175), Pose2D(5.0, 6.8, 0.175), Pose2D(5.0, 7.0, 0.175),
      Pose2D(5.6, 6.2, 0.175), Pose2D(5.6, 6.4, 0.175), Pose2D(5.6, 6.6, 0.175),
      Pose2D(5.5, 6.8, 0.175), Pose2D(5.6, 7.0, 0.175), Pose2D(5.6, 7.2, 0.175),
      Pose2D(6.0, 6.4, 0.175), Pose2D(6.0, 6.6, 0.175), Pose2D(6.0, 6.8, 0.175),
      Pose2D(6.0, 7.0, 0.175), Pose2D(6.0, 7.2, 0.175), Pose2D(6.6, 6.4, 0.175),
      Pose2D(6.6, 6.6, 0.175), Pose2D(6.6, 6.8, 0.175), Pose2D(6.6, 7.0, 0.175),
      Pose2D(6.6, 7.2, 0.175), Pose2D(7.0, 6.6, 0.175), Pose2D(7.0, 6.8, 0.175),
      Pose2D(7.0, 7.0, 0.175), Pose2D(7.0, 7.2, 0.175), Pose2D(7.5, 6.8, 0.175),
      Pose2D(7.5, 7.2, 0.175), Pose2D(7.5, 7.4, 0.175), Pose2D(8.0, 6.6, 0.175),
      Pose2D(8.0, 6.8, 0.175), Pose2D(8.0, 7.0, 0.175), Pose2D(8.0, 7.2, 0.175),
      Pose2D(8.0, 7.4, 0.175)};
}

SbpResult PerpendicularOutRulePlanner::getResult() { return result_; }
std::vector<Pose2D> PerpendicularOutRulePlanner::getSearchPoints() {
  return key_points_;
}

void PerpendicularOutRulePlanner::Update(
    const planning_math::Box2d map_boundary,
    const std::vector<planning_math::LineSegment2d> &map) {}

bool PerpendicularOutRulePlanner::Plan(
    const std::vector<SbpObstaclePtr> &obs_ptrs,
    parking::SearchProcessDebug *sp_debug) {
  if (fabs(init_pose.x - target_pose.x) < 0.2 &&
      fabs(init_pose.theta - target_pose.theta) < 0.01 &&
      init_pose.y - target_pose.y > 1e-3) {
    std::vector<Pose2D> path_to_fill{init_pose};
    if (PlanDirectOut(path_to_fill) &&
        isRSPathSafe(obstacles, points_of_obstacles, init_pose, path_to_fill)) {
      std::reverse(path_to_fill.begin(), path_to_fill.end());
      key_points_ = path_to_fill;
      extendKeyPointsToResult(
          msquare::HybridAstarConfig::GetInstance()->step_size);
      return true;
    }
  }

  if (abs(init_pose.x - target_pose.x) < 0.2 &&
      fabs(init_pose.theta - target_pose.theta) < 0.01 &&
      init_pose.y - target_pose.y < 0) {
    if (!checkStraightLine(obstacles, points_of_obstacles, init_pose,
                           target_pose)) {
      return false;
    }
    key_points_ = {init_pose, target_pose};
    extendKeyPointsToResult(
        msquare::HybridAstarConfig::GetInstance()->step_size);
    return true;
  }

  if (std::abs(init_vel_) > 1e-6) {
    // not support midplan temporatily
    return false;
  }

  for (auto buffer_pair : buffer_zigzag_pairs_) {
    if (PlanWithBuffer(buffer_pair.first, buffer_pair.second)) {
      extendKeyPointsToResult(
          msquare::HybridAstarConfig::GetInstance()->step_size);
      return true;
    }
  }
  return false;
}

bool PerpendicularOutRulePlanner::PlanDirectOut(
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
  std::reverse(path_to_fill.begin(), path_to_fill.end());
  return success;
}

bool PerpendicularOutRulePlanner::PlanWithBuffer(double safe_buffer,
                                                 int max_zigzag) {
  key_points_.clear();
  result_.clear();

  std::vector<Pose2D> planning_head = {init_pose};
  // not support target which is too far
  if (planning_head.empty()) {
    return false;
  }
  bool succeed_flag = false;

  if (planning_head.back().theta > 0) {
    if (startWithStraightLine(planning_head.back(), safe_buffer, max_zigzag)) {
      succeed_flag = true;
    } else if (startWithSweetPose(planning_head.back(), safe_buffer,
                                  max_zigzag)) {
      succeed_flag = true;
    }
  } else {
    if (startWithSweetPose(planning_head.back(), safe_buffer, max_zigzag)) {
      succeed_flag = true;
    } else if (startWithStraightLine(planning_head.back(), safe_buffer,
                                     max_zigzag)) {
      succeed_flag = true;
    }
  }

  key_points_.insert(key_points_.begin(), planning_head.begin(),
                     planning_head.end());
  return succeed_flag;
}

bool PerpendicularOutRulePlanner::startWithStraightLine(
    const Pose2D &start_pose, double safe_buffer, int max_zigzag) {
  key_points_.clear();
  key_points_.push_back(start_pose);
  planning_math::Vec2d center;
  Pose2D turning_pose = calcTurningPose(start_pose, safe_buffer, center);
  if (std::isnan(turning_pose.x) || std::isnan(turning_pose.y) ||
      std::isnan(turning_pose.theta)) {
    return false;
  }

  if (tooNearUpperWall(turning_pose, center) ||
      !checkStraightLine(obstacles, points_of_obstacles, start_pose,
                         turning_pose)) {
    return false;
  }
  key_points_.push_back(turning_pose);

  if (calcCurvePath(turning_pose, max_zigzag)) {
    return true;
  }
  return false;
}

bool PerpendicularOutRulePlanner::startWithSweetPose(const Pose2D &start_pose,
                                                     double safe_buffer,
                                                     int max_zigzag) {
  double ratio = 2.0;
  bool flag = false;
  Pose2D turning_pose;
  planning_math::Vec2d center;
  for (; ratio > 0.99; ratio -= 0.1) {
    for (auto sw_pose : sweet_spot_[slot_width_]) {
      key_points_.clear();
      key_points_.push_back(start_pose);

      if (calcAdjustPoses(start_pose, sw_pose, ratio * veh_min_r)) {
        turning_pose = calcTurningPose(sw_pose, safe_buffer, center);
        if (std::isnan(turning_pose.x) || std::isnan(turning_pose.y) ||
            std::isnan(turning_pose.theta)) {
          continue;
        }
        if (!checkStraightLine(obstacles, points_of_obstacles,
                               key_points_.back(), turning_pose)) {
          continue;
        }
        key_points_.push_back(turning_pose);
        if (!calcCurvePath(turning_pose, max_zigzag)) {
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
  return flag;
}

bool PerpendicularOutRulePlanner::calcCurvePath(const Pose2D &start_pose,
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
        expected_pose.theta > next_pose.theta &&
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

void PerpendicularOutRulePlanner::extractKeyInfo(
    const parking::OpenspaceDeciderOutput &input) {
  // get local frame
  local_frame_pose_ =
      Pose2D(input.init_state.path_point.x, input.init_state.path_point.y,
             input.init_state.path_point.theta - M_PI_2);

  // get local poses
  init_pose = planning_math::tf2d(local_frame_pose_,
                                  Pose2D(input.target_state.path_point.x,
                                         input.target_state.path_point.y,
                                         input.target_state.path_point.theta));
  init_vel_ = -input.init_state.v;
  target_pose = planning_math::tf2d(local_frame_pose_,
                                    Pose2D(input.init_state.path_point.x,
                                           input.init_state.path_point.y,
                                           input.init_state.path_point.theta));
  planning_math::LineSegment2d oppo_line(planning_math::Vec2d(-10, 11),
                                         planning_math::Vec2d(10, 11));

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

    auto local_left_road_bound = MirrorInjection(planning_math::tf2d(
        local_frame_pose_, input.T_lines.road_lower_left_bound));
    auto local_left_slot_bound = MirrorInjection(
        planning_math::tf2d(local_frame_pose_, input.T_lines.slot_left_bound));
    auto local_right_road_bound = MirrorInjection(planning_math::tf2d(
        local_frame_pose_, input.T_lines.road_lower_right_bound));
    auto local_right_slot_bound = MirrorInjection(
        planning_math::tf2d(local_frame_pose_, input.T_lines.slot_right_bound));
    obstacles.push_back(local_left_road_bound);
    obstacles.push_back(local_left_slot_bound);
    obstacles.push_back(local_right_road_bound);
    obstacles.push_back(local_right_slot_bound);

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

    auto local_left_road_bound = planning_math::tf2d(
        local_frame_pose_, input.T_lines.road_lower_left_bound);
    auto local_left_slot_bound =
        planning_math::tf2d(local_frame_pose_, input.T_lines.slot_left_bound);
    auto local_right_road_bound = planning_math::tf2d(
        local_frame_pose_, input.T_lines.road_lower_right_bound);
    auto local_right_slot_bound =
        planning_math::tf2d(local_frame_pose_, input.T_lines.slot_right_bound);
    obstacles.push_back(local_left_road_bound);
    obstacles.push_back(local_left_slot_bound);
    obstacles.push_back(local_right_road_bound);
    obstacles.push_back(local_right_slot_bound);

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

bool PerpendicularOutRulePlanner::tooNearUpperWall(
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

bool PerpendicularOutRulePlanner::calcAdjustPoses(const Pose2D &start_pose,
                                                  const Pose2D &end_pose,
                                                  double radius) {
  Pose2D init_local = planning_math::tf2d(end_pose, start_pose);
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

  Pose2D last_pose = start_pose;
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
PerpendicularOutRulePlanner::calcTurningPose(const Pose2D &start_pose,
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
  double target_x = x_O1 - turing_radius * sin(heading);
  double target_y = tan(heading) * (target_x - xs) + ys;

  return Pose2D(target_x, target_y, heading);
}

void PerpendicularOutRulePlanner::extendKeyPointsToResult(double step_size) {
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

Pose2D PerpendicularOutRulePlanner::calcExpectedPose(const Pose2D &start_pose,
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

Pose2D PerpendicularOutRulePlanner::calcExpectedPoseReverse(
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

} // namespace msquare
