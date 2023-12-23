#include "planner/motion_planner/optimizers/openspace_optimizer/parallel_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/reeds_shepp_path.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/rule_math.h"

namespace msquare {

void ParallelRulePlanner::fillPara(const parking::OpenspaceDeciderOutput &odo,
                                   clothoid::Parameter &para) {
  para_.min_block_len = CarParams::GetInstance()->car_config.common_config.min_block_len;
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
  para_.back_light_len = CarParams::GetInstance()->car_config.car_only_config.back_light_len;
  para_.back_light_height = CarParams::GetInstance()->car_config.car_only_config.back_light_height;
  para_.front_corner_width = VehicleParam::Instance()->bumper_length / 2.0;
  para_.front_corner_length = VehicleParam::Instance()->front_edge_to_center -
                              VehicleParam::Instance()->light_to_front_edge;

  para_.check_empty_length = CarParams::GetInstance()->car_config.parallel_config.check_empty_length;
  para_.is_min_r_priority =
      CarParams::GetInstance()->car_config.parallel_config.is_min_r_priority;
  /* */
  para_.straight_lat = 0.2;
  para_.straight_lon = 0.3;

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
  impl_.last_v = odo.target_state.v;
  impl_.block_direc =
      std::abs(impl_.last_v) > 1e-6 ? (0 < impl_.last_v ? 1 : -1) : 0;
  impl_.safe_radius =
      para_.wheel_base / (std::tan(para_.init_steer_angle /
                                   para_.steering_angle_ratio * M_PI / 180.0));

  impl_.rs = {para_.min_radius, 1.414 * para_.min_radius,
              2.0 * para_.min_radius, 3.0 * para_.min_radius};
  impl_.rs_size = impl_.rs.size();

  LOCAL_LOG(LOCAL_INFO, "alpha:%f min radius:%.3f block direc:%d",
            para_.max_alpha, para_.min_radius, impl_.block_direc);
  LOCAL_LOG(LOCAL_INFO, "lat:%.3f, lon:%.3f, fr:%.3f, br:%.3f, rr:%.3f",
            para_.lat, para_.lon, para_.front_to_rear, para_.back_to_rear,
            para_.rear_to_center);
  LOCAL_LOG(LOCAL_INFO, "safe radius:%.3f", impl_.safe_radius);
}

ParallelRulePlanner::ParallelRulePlanner(
    const parking::OpenspaceDeciderOutput &odo) {
  LOCAL_LOG(LOCAL_INFO, ""); // empty line
  LOCAL_LOG(LOCAL_INFO, ""); // empty line
  LOCAL_LOG(LOCAL_INFO, "***************************************");
  LOCAL_LOG(LOCAL_INFO, "************ ENTER PARALLEL PLAN ******");
  LOCAL_LOG(LOCAL_INFO, "***************************************");

  fillPara(odo, para_);
  initScenario(odo);

  clo_solver_ = clothoid::ClothoidSolver(para_.max_alpha);
}

/**
spiral csc path with fi curvature at front path
**/
bool ParallelRulePlanner::spiralCscPath(const Pose2D &escape_pose,
                                        const Pose2D &start_pose,
                                        OutSlotPathSegments &path_segments,
                                        OutSlotAdjustPath &last_segments,
                                        double r2, int block_direc) {
  //  add virtual point
  Pose2D virtual_start_pose = start_pose;
  clothoid::ClothoidCurve virtual_clo_curve, virtual_clo_curve_moved;
  if (r2 < impl_.safe_radius) {
    LOCAL_LOG(LOCAL_INFO, "use virtual start pose");
    clothoid::ClothoidPoint clo_p_s, clo_p_e;
    clo_solver_.getPointByRadius(impl_.safe_radius, clo_p_s);
    clo_solver_.getPointByRadius(r2, clo_p_e);
    clothoid::ClothoidCurve clo_ps_cut_raw, clo_ps_cut_origin;
    clo_solver_.getCurveByS(clo_p_s.s, clo_p_e.s, para_.step, clo_ps_cut_raw);
    clothoid::transformS4(clo_ps_cut_raw, clo_ps_cut_origin);

    clothoid::ClothoidCurve clo_ps_cut_rotated;
    double rotated_theta = start_pose.theta - clo_ps_cut_origin.back().theta;
    clothoid::transform2D(clo_ps_cut_origin, clo_ps_cut_rotated, 0.0, 0.0,
                          rotated_theta);

    double delta_x = start_pose.x - clo_ps_cut_rotated.back().x;
    double delta_y = start_pose.y - clo_ps_cut_rotated.back().y;
    clothoid::transform2D(clo_ps_cut_rotated, virtual_clo_curve, delta_x,
                          delta_y, 0.0);

    virtual_start_pose = virtual_clo_curve.front();
  }
  double start_x = virtual_start_pose.x;
  double start_y = virtual_start_pose.y;
  double start_theta = virtual_start_pose.theta;
  double real_start_theta = start_pose.theta;

  double r1 = para_.min_radius;

  double cx, cy, mu1, mu2;
  double r1_large, r2_large;
  clo_solver_.getMuOffset(para_.min_radius, cx, cy, mu1);
  r1_large = std::hypot(cx, cy);
  clo_solver_.getMuOffset(r2, cx, cy, mu2);
  r2_large = std::hypot(cx, cy);

  // start start
  Pose2D next_pose = checker_.rotateMaxPose(
      escape_pose, clothoid::RotateType::LEFT_FORWARD,
      clothoid::ShapeType::OCTAGON, r1, false, para_.lat, para_.lon);
  double max_theta = next_pose.theta - escape_pose.theta;
  // LOCAL_LOG(LOCAL_INFO,"max_theta:%f", max_theta);

  // get circle1 center
  double c2_offset = r2; // r2_large * std::cos(mu2); // clothoid or not
  double cx1, cy1;
  cx1 = escape_pose.x + r1 * cos(escape_pose.theta + M_PI_2);
  cy1 = escape_pose.y + r1 * sin(escape_pose.theta + M_PI_2);
  clothoid::Circle circle1(cx1, cy1, r1);
  clothoid::Circle circle1_large(cx, cy, r1_large);

  double current_length =
      std::sqrt(pow2(r1_large + r2_large) -
                pow2(r1_large * std::cos(mu1) + r2_large * std::cos(mu2)));
  double least_length = r1_large * std::sin(mu1) + r2_large * std::sin(mu2);

  double add_length_to_least =
      std::max(para_.s_straight - current_length + least_length,
               current_length - least_length);
  double center_distance =
      std::sqrt(pow2(add_length_to_least + current_length) +
                pow2(r1_large * std::cos(mu1) + r2_large * std::cos(mu2)));
  LOCAL_LOG(LOCAL_INFO,
            "add_length_to_least:%.3f, add length to center distance:%.3f",
            add_length_to_least, center_distance - r1_large - r2_large);

  // calculate forward straight length acaccording to tangent circle
  double bx, by;
  bx = start_x + c2_offset * std::cos(start_theta - M_PI_2) - cx1;
  by = start_y + c2_offset * std::sin(start_theta - M_PI_2) - cy1;

  double aa, bb, cc;
  aa = 1.0;
  bb =
      2 * bx * std::cos(real_start_theta) + 2 * by * std::sin(real_start_theta);
  cc = bx * bx + by * by - pow2(center_distance);

  double l1, l2;
  bool is_solved = solveQuadratic(aa, bb, cc, l1, l2);

  if (!is_solved) {
    return false;
  }

  double forward_length = std::max(l1, l2);

  double c2x, c2y;
  c2x = start_x + forward_length * std::cos(real_start_theta) +
        c2_offset * std::cos(start_theta - M_PI_2);
  c2y = start_y + forward_length * std::sin(real_start_theta) +
        c2_offset * std::sin(start_theta - M_PI_2);

  clothoid::Circle circle2(c2x, c2y, r2);
  clothoid::Circle circle2_large(c2x, c2y, r2_large);

  // validatity
  if (c2y > cy1 || cx1 > c2x || escape_pose.x > c2x) {
    return false;
  }

  // add by xhy
  std::vector<planning_math::Vec2d> obs_poses_in_distance;
  planning_math::Vec2d obs_poses_center(impl_.right_corner.x(), impl_.right_corner.y());
  double obs_distance = std::min(para_.side_safe_distance, corner_safe_distance_);
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
    double dis_to_key =
        std::hypot(c2x - (impl_.right_corner.x() - obs_distance), c2y - (impl_.right_corner.y() - obs_distance));
    double safe_dis = r2 - dis_to_key - impl_.half_width;
    if (safe_dis < obs_distance) {
      LOCAL_LOG(LOCAL_DEBUG, "c2 safe distance: %f which is samller than %f",
                safe_dis, obs_distance);
      return false;
    }
  } else {
    for (planning_math::Vec2d obs_pose_in_dis : obs_poses_in_distance) {
      double dis_to_key =
          std::hypot(c2x - obs_pose_in_dis.x(), c2y - obs_pose_in_dis.y());
      double safe_dis = r2 - dis_to_key - impl_.half_width;
      if (safe_dis < obs_distance) {
        LOCAL_LOG(LOCAL_DEBUG, "c2 safe distance: %f which is samller than %f",
                  safe_dis, obs_distance);
        return false;
      }
    }
  }
  
  //   LOCAL_LOG(LOCAL_INFO,"forward length: %f \tl1:%f \tl2:%f",
  //   forward_length, l1, l2); LOCAL_LOG(LOCAL_INFO,"cx:5%f \t cy:%f", c2x,
  //   c2y);

  double theta1_1 = escape_pose.theta;
  double theta1_2 = std::atan2(c2x - cx1, cy1 - c2y);

  // c1-c2
  double theta3 =
      std::acos((r1_large * std::cos(mu1) + r2_large * std::cos(mu2)) /
                (center_distance));

  //  get clothoid curve
  clothoid::ClothoidPoint clo_p1, clo_p;
  clo_solver_.getPointByRadius(r1, clo_p1);
  clo_solver_.getPointByRadius(r2, clo_p);
  clothoid::ClothoidCurve clo_ps1, clo_ps;
  clo_solver_.getCurveByS(clo_p1.s, para_.step, clo_ps1);
  clo_solver_.getCurveByS(clo_p.s, para_.step, clo_ps);

  if (theta1_2 - theta1_1 - clo_p1.theta - theta3 < 0.0) {
    LOCAL_LOG(LOCAL_DEBUG, "not enough to make a circle");
    return false;
  }
  Pose2D escape_end_pose = checker_.rotateTheta(
      escape_pose, 1, 1, r1, theta1_2 - theta1_1 - clo_p1.theta - theta3);
  // theat1, move_theta, cx, cy,
  double theta_step = para_.step / r1;
  double delta_theta = escape_end_pose.theta - escape_pose.theta;
  if (delta_theta > max_theta) {
    LOCAL_LOG(LOCAL_DEBUG, "c1 rotate theta %.3f is less than needed: %.3f",
              max_theta, delta_theta);
    return false;
  }

  clothoid::CircularCurve cir1 =
      clothoid::linspaceCCW(escape_pose, delta_theta, theta_step, circle1);

  // mirror II
  clothoid::ClothoidCurve c1, c2, c3, c4;
  clothoid::transformS2(clo_ps1, c2);
  clothoid::transform2D(c2, c4, escape_end_pose.x, escape_end_pose.y,
                        escape_end_pose.theta + clo_p1.theta);

  // circle 2:
  double move_theta_s = theta1_2 - clo_p.theta - theta3;
  double move_theta_e = start_theta;

  if (move_theta_s < move_theta_e) {
    LOCAL_LOG(LOCAL_DEBUG,
              "not enough to make a circle 2, theta_s: %f theta_e: %f",
              move_theta_s, move_theta_e);
    return false;
  }

  double s12 = 0.0;
  s12 = std::sqrt(pow2(center_distance) -
                  pow2(r1_large * std::cos(mu1) + r2_large * std::cos(mu2))) -
        (r1_large * std::sin(mu1) + r2_large * std::sin(mu2));

  clothoid::StraightPoint sp1;
  sp1.x = c4.back().x + s12 * std::cos(c4.back().theta);
  sp1.y = c4.back().y + s12 * std::sin(c4.back().theta);
  sp1.theta = c4.back().theta;

  clothoid::StraightCurve straight_path =
      clothoid::linspace(c4.back(), sp1, para_.step);

  // mirror type III
  clothoid::ClothoidCurve cm2;
  clothoid::transformS3(clo_ps, c1);
  clothoid::transform2D(c1, cm2, sp1.x, sp1.y,
                        escape_end_pose.theta + clo_p1.theta);

  clothoid::CircularCurve cir5 = clothoid::linspaceCW(
      M_PI_2 - move_theta_s, M_PI_2 - move_theta_e, para_.step / r2, circle2);

  double accum_error =
      std::hypot(cm2.back().x - cir5.front().x, cm2.back().y - cir5.front().y);
  if (accum_error > para_.max_accum_error) {
    LOCAL_LOG(LOCAL_DEBUG, "accum error:%f which is larger than %f",
              accum_error, para_.max_accum_error);
    return false;
  }

  //  straight
  double move_x = cir5.back().x - start_x;
  double move_y = cir5.back().y - start_y;
  bool is_forward =
      move_x * std::cos(start_theta) + move_y * std::sin(start_theta) > 0;
  double remain_s =
      std::hypot(cir5.back().x - start_x, cir5.back().y - start_y);
  if (cir5.back().x > 20) {
    LOCAL_LOG(LOCAL_DEBUG, "move forward too much:%f", cir5.back().x);
    return false;
  }
  if (block_direc > 1e-6 && is_forward) {
    LOCAL_LOG(LOCAL_DEBUG, "move forward is blocked");
    return false;
  }

  if (!is_forward && remain_s < para_.s_first_straight && block_direc > 1e-6) {
    LOCAL_LOG(LOCAL_DEBUG, "backward length is too short than %.3f m:%.3f",
              para_.s_first_straight, remain_s);
    return false;
  }

  double move_length =
      is_forward
          ? checker_.moveForward(start_pose, para_.straight_lat,
                                 para_.straight_lon, clothoid::ShapeType::RAW)
          : checker_.moveBackward(start_pose, para_.straight_lat,
                                  para_.straight_lon, clothoid::ShapeType::RAW);

  if (move_length < remain_s) {
    LOCAL_LOG(LOCAL_DEBUG, "remain s:%f move length:%f", remain_s, move_length);
    return false;
  }

  clothoid::StraightCurve str7;
  bool has_last = !last_segments.final_path.empty();

  /**
   *                    forward        backward
   * -------------------------------
   *                    longer
   * -----------------------------------------
   *  last forward      plus < thres        < thres - longer
   *  last backward     b < thres - longer
   */

  if (has_last && (last_segments.is_forward != is_forward) &&
      last_segments.remain_s < para_.min_block_len) {
    LOCAL_LOG(LOCAL_DEBUG, "last segment with tiny");
    return false;
  }

  if (is_forward && remain_s < para_.min_block_len &&
      !(has_last && last_segments.is_forward &&
        last_segments.remain_s + remain_s > para_.min_block_len)) {
    LOCAL_LOG(LOCAL_DEBUG, "remain_s:%f which is less than %f", remain_s,
              para_.min_block_len);
    if (move_length < 3.0 * para_.min_block_len + remain_s) {
      return false;
    }

    clothoid::StraightCurve str7_1 = clothoid::linspace(
        start_pose, remain_s + 2.0 * para_.min_block_len, para_.step);
    clothoid::StraightCurve str7_2 = clothoid::linspace(
        str7_1.back(), -2.0 * para_.min_block_len, para_.step);
    str7.insert(str7.end(), str7_1.begin(), str7_1.end());
    str7.insert(str7.end(), str7_2.begin(), str7_2.end());

  } else if ((para_.is_min_r_priority && !is_forward && (r2 > para_.min_radius+1e-3) &&
             remain_s < para_.s_first_straight)  ||
             (!para_.is_min_r_priority && !is_forward && remain_s < para_.s_first_straight)) { 
    LOCAL_LOG(LOCAL_DEBUG, "backward length is too short than %.3f m:%.3f",
              para_.s_first_straight, remain_s);
    double add_straight = para_.s_first_straight - remain_s;
    double forward_length =
        checker_.moveForward(start_pose, para_.straight_lat, para_.straight_lon,
                             clothoid::ShapeType::RAW);
    if (add_straight > forward_length) {
      LOCAL_LOG(LOCAL_DEBUG,
                "forward_length %.3f is too short than add_straight %.3f",
                forward_length, add_straight);
      return false;
    }

    if (has_last && !last_segments.is_forward &&
        (last_segments.remain_s < para_.min_block_len ||
         add_straight < para_.min_block_len)) {
      LOCAL_LOG(LOCAL_DEBUG, "has last and last remain is tiny:%.3f",
                last_segments.remain_s);
      return false;
    }
    if (has_last && last_segments.is_forward &&
        last_segments.remain_s + add_straight < para_.min_block_len) {
      return false;
    }
    if (!has_last && add_straight < para_.min_block_len) {
      return false;
    }
    clothoid::StraightCurve str7_1 =
        clothoid::linspace(start_pose, add_straight, para_.step);
    clothoid::StraightCurve str7_2 =
        clothoid::linspace(str7_1.back(), -add_straight - remain_s, para_.step);
    str7.insert(str7.end(), str7_1.begin(), str7_1.end());
    str7.insert(str7.end(), str7_2.begin(), str7_2.end());
  } else {
    str7 = clothoid::linspace(start_pose, (is_forward ? remain_s : -remain_s),
                              para_.step);
  }

  std::reverse(str7.begin(), str7.end());
  double virtual_x = 0;
  double virtual_y = 0;
  if (!str7.empty()) {
    virtual_x = str7.front().x - str7.back().x;
    virtual_y = str7.front().y - str7.back().y;
  }
  clothoid::transform2D(virtual_clo_curve, virtual_clo_curve_moved, virtual_x,
                        virtual_y, 0.0);

  path_segments.remain_s = remain_s;
  path_segments.is_forward = is_forward;
  path_segments.radius2 = r2;
  // path_segments.segments.push_back(last_segments.final_path);
  path_segments.segments.push_back(cir1);
  path_segments.segments.push_back(clothoid::clo2cir(c4));
  path_segments.segments.push_back(straight_path);
  path_segments.segments.push_back(clothoid::clo2cir(cm2));
  path_segments.segments.push_back(cir5);
  path_segments.segments.push_back(clothoid::clo2cir(virtual_clo_curve_moved));
  path_segments.segments.push_back(str7);
  // validate path

  for (unsigned int seg = 1; seg < path_segments.segments.size() - 1; seg++) {
    if (!checker_.checkBatchPose(path_segments.segments[seg],
                                 para_.straight_lat, para_.straight_lon,
                                 clothoid::ShapeType::OCTAGON)) {
      LOCAL_LOG(LOCAL_DEBUG, "with collision");
      return false;
    }
  }

path_segments.score =
      (path_segments.is_forward ? (path_segments.remain_s) : 0.0) -
      (para_.is_min_r_priority ? (-std::sqrt(path_segments.radius2))
                                : std::sqrt(path_segments.radius2));

  return true;
}

bool ParallelRulePlanner::adjustOutSlotCPPCs(
    const Pose2D &ego_pose, const OutSlotAdjustPath &last_segments,
    std::vector<OutSlotAdjustPath> &path_segments) {
  std::vector<double> rs = {1.414 * para_.min_radius, 1.414 * para_.min_radius,
                            para_.min_radius, 1.414 * para_.min_radius};
  std::vector<double> move_height = {0.05, 0.10, 0.15, 0.15};
  for (unsigned int i = 0; i < rs.size(); i++) {
    OutSlotAdjustPath path_segment;
    if (!adjustOutSlotCPPC(ego_pose, last_segments, path_segment, rs[i],
                           move_height[i])) {
      continue;
    }
    path_segments.push_back(path_segment);
  }
  return true;
}

bool ParallelRulePlanner::adjustOutSlotCPPC(
    const Pose2D &ego_pose, const OutSlotAdjustPath &last_segments,
    OutSlotAdjustPath &path_segments, double r, double move_height) {
  double radius = r;
  double radical_alpha = para_.radical_alpha;
  LOCAL_LOG(LOCAL_INFO, "radical alpha:%.4f", radical_alpha);
  clothoid::ClothoidSolver clo_solver(radical_alpha);

  clothoid::ClothoidPoint clo_p;
  clo_solver.getPointByRadius(radius, clo_p);
  clothoid::ClothoidCurve clo_curve;
  clo_solver.getCurveByS(clo_p.s, para_.step, clo_curve);

  double move_theta = std::acos((radius - move_height) / radius);
  LOCAL_LOG(LOCAL_INFO, "move theta:%.4f, spiral theta:%.4f", move_theta,
            clo_p.theta);
  if (move_theta < 1e-3 || move_theta < clo_p.theta) {
    return false;
  }
  LOCAL_LOG(LOCAL_INFO, "ego_pose: x:%.3f\ty:%.3f\ttheta:%.3f", ego_pose.x,
            ego_pose.y, ego_pose.theta);

  // adjust small theta
  Pose2D low_pose = ego_pose;
  if (std::abs(ego_pose.theta) > 1e-6) {
    LOCAL_LOG(LOCAL_DEBUG, "ego pose theta is too larger");
    return false;
  }
  low_pose.theta = 0.0;

  clothoid::Circle circle(low_pose.x - radius * std::sin(low_pose.theta),
                          low_pose.y + radius * std::cos(low_pose.theta),
                          radius); // left turn
  clothoid::CircularCurve cir1 =
      clothoid::linspaceCCW(low_pose, move_theta, para_.step / radius, circle);

  clothoid::ClothoidCurve clo_temp, clo2;
  clothoid::StraightCurve cir2, clo3, clo4;
  clothoid::transformS2(clo_curve, clo_temp);
  clothoid::transform2D(clo_temp, clo2, cir1.back().x, cir1.back().y,
                        cir1.back().theta + clo_p.theta);

  Pose2D rotate_pose = clo2.back();
  cir2 = clothoid::clo2cir(clo2);

  clothoid::rotationalSymmetry(rotate_pose.x, rotate_pose.y, cir2, clo3);
  clothoid::rotationalSymmetry(rotate_pose.x, rotate_pose.y, cir1, clo4);

  std::reverse(cir1.begin(), cir1.end());
  std::reverse(cir2.begin(), cir2.end());
  std::reverse(clo3.begin(), clo3.end());
  std::reverse(clo4.begin(), clo4.end());

  path_segments.segments.push_back(clo4);
  path_segments.segments.push_back(clo3);
  path_segments.segments.push_back(cir2);
  path_segments.segments.push_back(cir1);

  path_segments.is_forward = true;
  path_segments.remain_s = radius * move_theta;
  if (path_segments.remain_s < para_.min_block_len) {
    LOCAL_LOG(LOCAL_DEBUG, "tiny seg");
    return false;
  }

  // up-down
  double y_multi2 = low_pose.y * 2;
  double y_thres_range = 0.7 + impl_.half_width;
  if (low_pose.y > 3.0 && low_pose.y - impl_.slot_height > y_thres_range) {
    for (auto &seg : path_segments.segments) {
      for (auto &p : seg) {
        p.theta = -p.theta;
        p.y = y_multi2 - p.y;
      }
    }
  }

  if (!path_segments.accumPath(para_.max_accum_error)) {
    LOCAL_LOG(LOCAL_DEBUG, "failed to accum path");
    return false;
  }

  if (!checker_.checkBatchPoseExceptEnd(path_segments.final_path, para_.lat,
                                        para_.straight_lon,
                                        clothoid::ShapeType::ROTATE)) {
    LOCAL_LOG(LOCAL_DEBUG, "failed to check collision");
    return false;
  }

  // combine
  if (last_segments.is_valid) {
    if (last_segments.remain_s < para_.min_block_len &&
        last_segments.is_forward == (!path_segments.is_forward)) {
      LOCAL_LOG(LOCAL_DEBUG, "last seg tiny path");
      return false;
    }
    path_segments.segments.insert(path_segments.segments.end(),
                                  last_segments.segments.begin(),
                                  last_segments.segments.end());
    if (last_segments.is_forward == path_segments.is_forward) {
      path_segments.remain_s += last_segments.remain_s;
    }
  }

  if (!path_segments.accumPath(para_.max_accum_error)) {
    LOCAL_LOG(LOCAL_DEBUG, "accum path failed");
    return false;
  }

  return true;
}

bool ParallelRulePlanner::adjustOutslotLargeC(
    const Pose2D &ego_pose, std::vector<OutSlotAdjustPath> &path_segments) {
  double target_height = impl_.slot_height + 1.8;
  double theta_abs = std::abs(ego_pose.theta);
  if (theta_abs < 1e-3 || theta_abs > M_PI * 0.45) {
    return false;
  }

  double theta_cos = std::cos(theta_abs);
  double theta_sin = std::sin(theta_abs);

  double x_abs = ego_pose.x;
  double y_offset = std::abs(ego_pose.y - target_height);
  double y_abs = target_height - y_offset;

  double target_r = y_offset / (1 - theta_cos);
  LOCAL_LOG(LOCAL_INFO, "target_r:%.3f", target_r);

  if (target_r < para_.min_radius) {
    LOCAL_LOG(LOCAL_DEBUG,
              "target radius is smaller than min radius,  target r:%.3f",
              target_r);
    return false;
  }
  double target_x = x_abs + target_r * theta_sin;
  double target_y = y_abs - target_r * theta_cos;

  clothoid::Circle circle(target_x, target_y, target_r);
  clothoid::CircularCurve cir = clothoid::linspaceCW(
      M_PI_2 - theta_abs, M_PI_2, para_.step / target_r, circle);

  if (cir.back().x > 10.0) {
    return false;
  }

  OutSlotAdjustPath seg;
  seg.is_forward = true;

  //  mirror
  if (ego_pose.theta < 0 && ego_pose.y > target_height) {
    for (auto &p : cir) {
      p.y = 2 * target_height - p.y;
      p.theta = -p.theta;
    }
  }
  if (ego_pose.theta > 0 && ego_pose.y > target_height) {
    for (auto &p : cir) {
      p.y = 2 * ego_pose.x - p.x;
      p.x = 2 * ego_pose.x - p.x;
    }
    seg.is_forward = false;
  }
  if (ego_pose.theta < 0 && ego_pose.y < target_height) {
    for (auto &p : cir) {
      p.y = 2 * ego_pose.x - p.x;
      p.theta = -p.theta;
    }
    seg.is_forward = false;
  }

  std::reverse(cir.begin(), cir.end());
  seg.segments.push_back(cir);
  seg.remain_s = target_r * theta_abs;
  if (!seg.accumPath(para_.max_accum_error)) {
    return false;
  }
  if (!checker_.checkBatchPoseExceptEnd(seg.final_path, para_.straight_lat,
                                        para_.straight_lon,
                                        clothoid::ShapeType::ROTATE)) {
    LOCAL_LOG(LOCAL_DEBUG, "failed to check collision");
    return false;
  }
  seg.score = 0;
  path_segments.push_back(seg);

  return true;
}

bool ParallelRulePlanner::adjustOutslotSPCP(
    const Pose2D &ego_pose, std::vector<OutSlotAdjustPath> &path_segments) {
  double theta = std::abs(ego_pose.theta);
  Pose2D start_pose(ego_pose.x, ego_pose.y, theta);
  if (theta > M_PI_2 || theta < 1e-6) {
    return false;
  }
  double sin_theta, cos_theta, tan_theta;
  sin_theta = std::sin(theta);
  cos_theta = std::cos(theta);
  tan_theta = std::tan(theta);
  double r1 = para_.min_radius;
  double theta_step = para_.step / r1;

  clothoid::ClothoidSolver clo_solver(para_.max_alpha);
  clothoid::ClothoidPoint clo_p;
  clothoid::ClothoidCurve clo_s1, clo_s2, clo_s3, clo_s4;
  clo_solver.getPointByRadius(r1, clo_p);
  clo_solver.getCurveByS(clo_p.s, para_.step, clo_s1);
  clothoid::transformS2(clo_s1, clo_s2);
  clothoid::transformS3(clo_s1, clo_s3);
  clothoid::transformS4(clo_s1, clo_s4);

  double one_theta = clo_p.theta;
  double two_theta = 2.0 * clo_p.theta;

  std::vector<OutSlotAdjustPath> temp_segments;
  if (theta <= one_theta) {
    clothoid::ClothoidCurve clo_add;
    clothoid::ClothoidPoint clo_theta;
    clo_solver.getPointByTheta(theta, clo_theta);
    clothoid::ClothoidCurve clo_s1_theta, clo_s4_theta;
    clo_solver.getCurveByS(clo_theta.s, para_.step, clo_s1_theta);
    clothoid::transformS4(clo_s1_theta, clo_s4_theta);

    // backward
    double delta_x = start_pose.x - clo_s1_theta.back().x;
    double delta_y = start_pose.y - clo_s1_theta.back().y;
    clothoid::transform2D(clo_s1_theta, clo_add, delta_x, delta_y, 0.0);

    OutSlotAdjustPath osap1;
    osap1.is_forward = false;
    osap1.remain_s = clo_s1_theta.back().s;

    osap1.segments.push_back(clothoid::clo2cir(clo_add));
    temp_segments.push_back(osap1);

    // forward
    clothoid::transform2D(clo_s4_theta, clo_add, start_pose.x, start_pose.y,
                          0.0);
    std::reverse(clo_add.begin(), clo_add.end());
    OutSlotAdjustPath osap2;
    osap2.is_forward = true;
    osap2.remain_s = clo_s1_theta.back().s;
    osap2.segments.push_back(clothoid::clo2cir(clo_add));
    temp_segments.push_back(osap2);
  } else if (theta >= two_theta) {
    // forward
    clothoid::ClothoidCurve p1, p2;
    clothoid::transform2D(clo_s3, p1, start_pose.x, start_pose.y, theta);

    double cir_theta = theta - two_theta;
    clothoid::ClothoidPoint p1_back = p1.back();
    clothoid::Circle circle(p1_back.x + r1 * std::sin(p1_back.theta),
                            p1_back.y - r1 * std::cos(p1_back.theta), r1);
    clothoid::CircularCurve circular_curve = clothoid::linspaceCW(
        M_PI_2 - p1_back.theta, M_PI_2 - one_theta, theta_step, circle);

    Pose2D cir_back = circular_curve.back();
    clothoid::transform2D(clo_s4, p2, cir_back.x, cir_back.y,
                          cir_back.theta - one_theta);

    std::reverse(p1.begin(), p1.end());
    std::reverse(circular_curve.begin(), circular_curve.end());
    std::reverse(p2.begin(), p2.end());

    OutSlotAdjustPath forward_segments;
    forward_segments.is_forward = true;
    forward_segments.remain_s = clo_p.s * 2 + r1 * cir_theta;
    forward_segments.segments.push_back(clothoid::clo2cir(p2));
    forward_segments.segments.push_back(circular_curve);
    forward_segments.segments.push_back(clothoid::clo2cir(p1));
    temp_segments.push_back(forward_segments);

    // backward
    clothoid::ClothoidCurve bp1_rotate, bp1, bp2;
    clothoid::rotateCurve(clo_s2, bp1_rotate, theta);
    double delta_x = start_pose.x - bp1_rotate.back().x;
    double delta_y = start_pose.y - bp1_rotate.back().y;
    clothoid::transform2D(bp1_rotate, bp1, delta_x, delta_y, 0);
    clothoid::ClothoidPoint bp1_back = bp1.front();
    clothoid::Circle bcircle(bp1_back.x - r1 * std::sin(bp1_back.theta),
                             bp1_back.y + r1 * std::cos(bp1_back.theta), r1);
    clothoid::CircularCurve bcircular_curve = clothoid::linspaceCWRight(
        M_PI_2 - bp1_back.theta, M_PI_2 - bp1_back.theta + cir_theta,
        theta_step, bcircle);
    Pose2D bcir_back = bcircular_curve.back();

    delta_x = bcir_back.x - clo_s1.back().x;
    delta_y = bcir_back.y - clo_s1.back().y;
    clothoid::transform2D(clo_s1, bp2, delta_x, delta_y, 0);

    std::reverse(bcircular_curve.begin(), bcircular_curve.end());

    // LOCAL_LOG(LOCAL_INFO,"bp2 end x:%.3f, y:%.3f, theta:%.3f",bp2.back().x,
    // bp2.back().y, bp2.back().theta );
    // LOCAL_LOG(LOCAL_INFO,"bcircular_curve.front x:%.3f, y:%.3f,
    // theta:%.3f",bcircular_curve.front().x, bcircular_curve.front().y,
    // bcircular_curve.front().theta);
    // LOCAL_LOG(LOCAL_INFO,"bcircular_curve.back:%.3f, y:%.3f,
    // theta:%.3f",bcircular_curve.back().x, bcircular_curve.back().y,
    // bcircular_curve.back().theta); LOCAL_LOG(LOCAL_INFO,"bp1 front x:%.3f,
    // y:%.3f, theta:%.3f",bp1.front().x, bp1.front().y, bp1.front().theta);
    // LOCAL_LOG(LOCAL_INFO,"ego_pose x:%.3f, y:%.3f, theta:%.3f",ego_pose.x,
    // ego_pose.y, ego_pose.theta);

    OutSlotAdjustPath backward_segments;
    backward_segments.is_forward = false;
    backward_segments.remain_s = clo_p.s * 2 + r1 * cir_theta;
    backward_segments.segments.push_back(clothoid::clo2cir(bp2));

    backward_segments.segments.push_back(bcircular_curve);
    backward_segments.segments.push_back(clothoid::clo2cir(bp1));

    temp_segments.push_back(backward_segments);
  } else {
    double left_cx, left_cy, right_cx, right_cy;
    left_cx = start_pose.x - r1 * sin_theta;
    left_cy = start_pose.y + r1 * cos_theta;
    clothoid::Circle left_circle(left_cx, left_cy, r1);

    right_cx = start_pose.x + r1 * sin_theta;
    right_cy = start_pose.y - r1 * cos_theta;
    clothoid::Circle right_circle(right_cx, right_cy, r1);

    clothoid::CircularCurve cir_add;
    clothoid::ClothoidCurve clo_add;
    // backward
    cir_add = clothoid::linspaceCCW(start_pose, clo_p.theta - theta, theta_step,
                                    left_circle);
    double delta_x = cir_add.back().x - clo_s1.back().x;
    double delta_y = cir_add.back().y - clo_s1.back().y;
    clothoid::transform2D(clo_s1, clo_add, delta_x, delta_y, 0.0);

    std::reverse(cir_add.begin(), cir_add.end());

    OutSlotAdjustPath osap1;
    osap1.is_forward = false;
    osap1.remain_s = clo_s1.back().s + r1 * (theta - clo_p.theta);
    osap1.segments.push_back(clothoid::clo2cir(clo_add));
    osap1.segments.push_back(cir_add);
    temp_segments.push_back(osap1);

    // forward
    cir_add = clothoid::linspaceCW(M_PI_2 - theta, M_PI_2 - clo_p.theta,
                                   theta_step, right_circle);
    clothoid::transform2D(clo_s4, clo_add, cir_add.back().x, cir_add.back().y,
                          0.0);
    OutSlotAdjustPath osap2;
    osap2.is_forward = true;
    osap1.remain_s = clo_s1.back().s + r1 * (theta - clo_p.theta);
    std::reverse(clo_add.begin(), clo_add.end());
    std::reverse(cir_add.begin(), cir_add.end());

    osap2.segments.push_back(clothoid::clo2cir(clo_add));
    osap2.segments.push_back(cir_add);
    temp_segments.push_back(osap2);
  }

  // mirror
  if (ego_pose.theta < 0) {
    double y_multi2 = 2 * ego_pose.y;
    for (auto &seg : temp_segments) {
      for (auto &curve : seg.segments) {
        for (auto &p : curve) {
          p.theta = -p.theta;
          p.y = y_multi2 - p.y;
        }
      }
    }
  }

  double passage_min = impl_.slot_height;
  double ego_safe_height = passage_min + 1.80;
  double ego_min_height = passage_min + 1.7;
  double ego_max_height = passage_min + 2.5;
  double max_move_s = 5.0;

  // get result
  std::vector<OutSlotAdjustPath> spcp_segments;
  for (auto &seg : temp_segments) {
    if (!seg.accumPath(para_.max_accum_error)) {
      LOCAL_LOG(LOCAL_DEBUG, "accum error");
      continue;
    }
    std::vector<Pose2D> final_path;
    seg.getPath(final_path);
    if (ego_pose.y > ego_min_height) {
      if (ego_pose.y > ego_max_height && ego_pose.theta > 0.0 &&
          seg.is_forward) {
        continue;
      }
      if (ego_pose.y > ego_max_height && ego_pose.theta < 0.0 &&
          !seg.is_forward) {
        continue;
      }
      Pose2D front_pose = final_path.front();
      if (front_pose.y > ego_min_height && front_pose.y < ego_max_height) {
        if (!checker_.checkBatchPoseExceptEnd(
                seg.final_path, para_.straight_lat, para_.straight_lon,
                clothoid::ShapeType::ROTATE)) {
          spcp_segments.push_back(seg);
        }
      }

      // straight length
      double move_straight = 0.0;
      if (ego_pose.theta < 0.0) {
        move_straight =
            checker_.moveForward(ego_pose, para_.straight_lat,
                                 para_.straight_lon, clothoid::ShapeType::RAW);
      } else {
        move_straight =
            checker_.moveBackward(ego_pose, para_.straight_lat,
                                  para_.straight_lon, clothoid::ShapeType::RAW);
      }
      double real_move_straight = std::min(move_straight, max_move_s);
      double real_safe_straight = std::min(
          (ego_pose.y - ego_min_height) / sin_theta, real_move_straight);

      double opti_straight = (ego_pose.y - ego_safe_height) / sin_theta;

      std::vector<double> straight_offsets =
          clothoid::linspace(0, real_safe_straight, 0.5);
      if (opti_straight > 0 && opti_straight < real_safe_straight) {
        straight_offsets.insert(straight_offsets.begin(), opti_straight);
      }

      for (auto s : straight_offsets) {

        double y_dis = s * sin_theta;
        double x_dis_abs = s * cos_theta;
        double x_dis = ego_pose.theta < 0.0 ? x_dis_abs : -x_dis_abs;

        clothoid::StraightPoint s1(ego_pose.x + x_dis, ego_pose.y - y_dis,
                                   ego_pose.theta);
        clothoid::StraightCurve s_curve =
            clothoid::linspace(s1, ego_pose, para_.step);
        std::vector<Pose2D> curve_path;
        clothoid::transform2D(final_path, curve_path, x_dis, -y_dis);
        if (!checker_.checkBatchPose(curve_path, para_.straight_lat,
                                     para_.straight_lon,
                                     clothoid::ShapeType::ROTATE)) {
          continue;
        }
        OutSlotAdjustPath s_seg;
        s_seg.is_forward = seg.is_forward;
        s_seg.remain_s = seg.remain_s + std::hypot(x_dis, y_dis);
        s_seg.segments.push_back(curve_path);
        s_seg.segments.push_back(s_curve);
        spcp_segments.push_back(s_seg);
      }
    } else if (ego_pose.y < ego_max_height) {
      if (ego_pose.y < ego_min_height && ego_pose.theta > 0.0 &&
          !seg.is_forward) {
        continue;
      }
      if (ego_pose.y < ego_min_height && ego_pose.theta < 0.0 &&
          seg.is_forward) {
        continue;
      }
      Pose2D front_pose = final_path.front();
      if (front_pose.y > ego_min_height && front_pose.y < ego_max_height) {
        if (!checker_.checkBatchPoseExceptEnd(
                seg.final_path, para_.straight_lat, para_.straight_lon,
                clothoid::ShapeType::ROTATE)) {
          spcp_segments.push_back(seg);
        }
      }

      // straight length
      double move_straight = 0.0;
      if (ego_pose.theta < 0.0) {
        move_straight =
            checker_.moveBackward(ego_pose, para_.straight_lat,
                                  para_.straight_lon, clothoid::ShapeType::RAW);
      } else {
        move_straight =
            checker_.moveForward(ego_pose, para_.straight_lat,
                                 para_.straight_lon, clothoid::ShapeType::RAW);
      }
      double real_move_straight = std::min(move_straight, max_move_s);
      double real_safe_straight = std::min(
          (ego_max_height - ego_pose.y) / sin_theta, real_move_straight);

      double opti_straight = (ego_safe_height - ego_pose.y) / sin_theta;

      std::vector<double> straight_offsets =
          clothoid::linspace(0, real_safe_straight, 0.5);
      if (opti_straight > 0 && opti_straight < real_safe_straight) {
        straight_offsets.insert(straight_offsets.begin(), opti_straight);
      } // end straight
      for (auto s : straight_offsets) {

        double y_dis = s * sin_theta;
        double x_dis_abs = s * cos_theta;
        double x_dis = ego_pose.theta < 0.0 ? -x_dis_abs : x_dis_abs;

        clothoid::StraightPoint s1(ego_pose.x + x_dis, ego_pose.y + y_dis,
                                   ego_pose.theta);
        clothoid::StraightCurve s_curve =
            clothoid::linspace(s1, ego_pose, para_.step);
        std::vector<Pose2D> curve_path;
        clothoid::transform2D(final_path, curve_path, x_dis, y_dis);
        if (!checker_.checkBatchPose(curve_path, para_.lat, para_.straight_lon,
                                     clothoid::ShapeType::ROTATE)) {
          continue;
        }
        OutSlotAdjustPath s_seg;
        s_seg.is_forward = seg.is_forward;
        s_seg.remain_s = seg.remain_s + std::hypot(x_dis, y_dis);
        s_seg.segments.push_back(curve_path);
        s_seg.segments.push_back(s_curve);
        spcp_segments.push_back(s_seg);
      }
    }

    path_segments = spcp_segments;
  }
  return true;
}

bool ParallelRulePlanner::adjustOutslotCP(
    const Pose2D &ego_pose, std::vector<OutSlotAdjustPath> &path_segments) {
  double theta = std::abs(ego_pose.theta);
  LOCAL_LOG(LOCAL_INFO, "real theta: %f", ego_pose.theta);

  Pose2D start_pose(ego_pose.x, ego_pose.y, theta);
  if (theta < -M_PI_2 || theta > M_PI_2) {
    return false;
  }

  if (theta < 1e-6) {
    return false;
  }

  double sin_theta, cos_theta;
  sin_theta = std::sin(theta);
  cos_theta = std::cos(theta);
  double r1 = para_.min_radius;
  double theta_step = para_.step / r1;

  clothoid::ClothoidSolver clo_solver(para_.max_alpha);
  clothoid::ClothoidPoint clo_p;
  clothoid::ClothoidCurve clo_s1, clo_s4;
  clo_solver.getPointByRadius(r1, clo_p);
  clo_solver.getCurveByS(clo_p.s, para_.step, clo_s1);
  clothoid::transformS4(clo_s1, clo_s4);

  double left_cx, left_cy, right_cx, right_cy;
  left_cx = start_pose.x - r1 * sin_theta;
  left_cy = start_pose.y + r1 * cos_theta;
  clothoid::Circle left_circle(left_cx, left_cy, r1);

  right_cx = start_pose.x + r1 * sin_theta;
  right_cy = start_pose.y - r1 * cos_theta;
  clothoid::Circle right_circle(right_cx, right_cy, r1);

  std::vector<OutSlotAdjustPath> temp_segments;
  if (theta > clo_p.theta) {
    clothoid::CircularCurve cir_add;
    clothoid::ClothoidCurve clo_add;
    // backward
    cir_add = clothoid::linspaceCCW(start_pose, clo_p.theta - theta, theta_step,
                                    left_circle);
    double delta_x = cir_add.back().x - clo_s1.back().x;
    double delta_y = cir_add.back().y - clo_s1.back().y;
    clothoid::transform2D(clo_s1, clo_add, delta_x, delta_y, 0.0);

    std::reverse(cir_add.begin(), cir_add.end());

    OutSlotAdjustPath osap1;
    osap1.is_forward = false;
    osap1.remain_s = clo_s1.back().s + r1 * (theta - clo_p.theta);
    osap1.segments.push_back(clothoid::clo2cir(clo_add));
    osap1.segments.push_back(cir_add);
    temp_segments.push_back(osap1);

    // forward
    cir_add = clothoid::linspaceCW(M_PI_2 - theta, M_PI_2 - clo_p.theta,
                                   theta_step, right_circle);
    clothoid::transform2D(clo_s4, clo_add, cir_add.back().x, cir_add.back().y,
                          0.0);
    OutSlotAdjustPath osap2;
    osap2.is_forward = true;
    osap1.remain_s = clo_s1.back().s + r1 * (theta - clo_p.theta);
    std::reverse(clo_add.begin(), clo_add.end());
    std::reverse(cir_add.begin(), cir_add.end());

    osap2.segments.push_back(clothoid::clo2cir(clo_add));
    osap2.segments.push_back(cir_add);
    temp_segments.push_back(osap2);

  } else {
    clothoid::ClothoidCurve clo_add;
    clothoid::ClothoidPoint clo_theta;
    clo_solver.getPointByTheta(theta, clo_theta);
    clothoid::ClothoidCurve clo_s1_theta, clo_s4_theta;
    clo_solver.getCurveByS(clo_theta.s, para_.step, clo_s1_theta);
    clothoid::transformS4(clo_s1_theta, clo_s4_theta);

    // backward
    double delta_x = start_pose.x - clo_s1_theta.back().x;
    double delta_y = start_pose.y - clo_s1_theta.back().y;
    clothoid::transform2D(clo_s1_theta, clo_add, delta_x, delta_y, 0.0);

    OutSlotAdjustPath osap1;
    osap1.is_forward = false;
    osap1.remain_s = clo_s1_theta.back().s;

    osap1.segments.push_back(clothoid::clo2cir(clo_add));
    temp_segments.push_back(osap1);

    // forward
    clothoid::transform2D(clo_s4_theta, clo_add, start_pose.x, start_pose.y,
                          0.0);
    std::reverse(clo_add.begin(), clo_add.end());
    OutSlotAdjustPath osap2;
    osap2.is_forward = true;
    osap2.remain_s = clo_s1_theta.back().s;
    osap2.segments.push_back(clothoid::clo2cir(clo_add));
    temp_segments.push_back(osap2);
  }

  // mirror
  if (ego_pose.theta < 0) {
    double y_multi2 = 2 * ego_pose.y;
    for (auto &seg : temp_segments) {
      for (auto &curve : seg.segments) {
        for (auto &p : curve) {
          p.theta = -p.theta;
          p.y = y_multi2 - p.y;
        }
      }
    }
  }

  // get result
  for (auto &seg : temp_segments) {
    if (!seg.accumPath(para_.max_accum_error)) {
      continue;
    }
    if (!checker_.checkBatchPoseExceptEnd(seg.final_path, para_.lat,
                                          para_.straight_lon,
                                          clothoid::ShapeType::ROTATE)) {
      continue;
    }
    seg.score = std::abs(seg.final_path.front().y - 3.2);
    path_segments.push_back(seg);
  }

  std::sort(path_segments.begin(), path_segments.end(),
            std::less<OutSlotAdjustPath>());

  return true;
}
bool ParallelRulePlanner::planOneTry(const Pose2D &target_pose,
                                      const Pose2D &start_pose,
                                      int block_direc) {
  if(!is_front_empty_){
    LOCAL_LOG(LOCAL_DEBUG, "return as not is_front_empty_");
    return false;
  }  
  // forward distance
  double escape_lon = std::min(para_.lon + 0.10, 0.4);
  double escape_lat = std::min(para_.lat + 0.05, 0.19);
  double real_forward_max = checker_.moveForward(
      target_pose, escape_lat, escape_lon, clothoid::ShapeType::RAW);
  
  double min_straight = 0.3-1e-6;
  double max_straight = std::min(0.5,real_forward_max);
  double step_straight = 0.1;

  // clothoid path
  clothoid::ClothoidPoint clo_p_s;
  clo_solver_.getPointByRadius(para_.min_radius, clo_p_s);
  clothoid::ClothoidCurve clo_ps_cut_raw;
  clo_solver_.getCurveByS(clo_p_s.s, para_.step, clo_ps_cut_raw);
  clothoid::StraightCurve clo_ps_cir = clo2cir(clo_ps_cut_raw);

  // try different move forward straight
  double cur_straight = max_straight;
  std::vector<Pose2D> local_straight_path;
  Pose2D escape_pose;
  clothoid::StraightCurve clo_curve_cur;

  while(cur_straight > min_straight){
    local_straight_path.clear();
    local_straight_path = clothoid::linspace(target_pose, cur_straight, para_.step);
    cur_straight -= step_straight;
    escape_pose = local_straight_path.back();

    clo_curve_cur.clear();
    clothoid::transform2D(clo_ps_cir, clo_curve_cur, escape_pose.x, escape_pose.y);

    escape_pose = clo_curve_cur.back();
    // validate the possibility with one turn to escape out
    Pose2D escape_rotate_pose =checker_.rotateMaxPose(escape_pose, clothoid::RotateType::LEFT_FORWARD,
                              clothoid::ShapeType::OCTAGON, para_.min_radius,
                              false, para_.lat, para_.lon);
    std::vector<planning_math::Vec2d> corners_temp;
    csg_.getRawShape(escape_rotate_pose, corners_temp, 0.0, 0.0);
    double theta_diff = escape_rotate_pose.theta - escape_pose.theta;
    if ( theta_diff < M_PI_2 && corners_temp[5].y() < impl_.right_corner.y()) {
      continue;
    }

    OutSlotPathSegments opti_path_segments;
    if (!planOutslot(escape_pose, start_pose, opti_path_segments,
                    block_direc)) {
      LOCAL_LOG(LOCAL_DEBUG, "failed to plan out slot");
      continue;
    }

    std::vector<Pose2D> local_outslot_path;
    if (!opti_path_segments.getPath(local_outslot_path)) {
      LOCAL_LOG(LOCAL_DEBUG, "plan outslot is empty");
      continue;
    }
    local_straight_path.insert(local_straight_path.end(),clo_curve_cur.begin(), clo_curve_cur.end());
    local_straight_path.insert(local_straight_path.end(), local_outslot_path.begin(),
                            local_outslot_path.end());

    generatePath(local_straight_path);
    return true;
  }
  return false;
}
bool ParallelRulePlanner::planOutslot(const Pose2D &escape_pose,
                                      const Pose2D &start_pose,
                                      OutSlotPathSegments &opti_path_segments,
                                      int block_direc) {
  std::vector<OutSlotPathSegments> path_segments_list;
  LOCAL_LOG(LOCAL_DEBUG, "enter spiral path");
  for (unsigned int i = 0; i < impl_.rs_size; i++) {
    OutSlotPathSegments path_seg;
    OutSlotAdjustPath empty_last_path;
    if (!spiralCscPath(escape_pose, start_pose, path_seg, empty_last_path,
                       impl_.rs[i], block_direc)) {
      LOCAL_LOG(LOCAL_DEBUG, "failed to use r = %.2f", impl_.rs[i]);
      continue;
    }
    if (!path_seg.accumPath(para_.max_accum_error)) {
      continue;
    }
    path_segments_list.push_back(path_seg);
  }

  if (!path_segments_list.empty()) {
    LOCAL_LOG(LOCAL_INFO, "not need adjust");
    std::sort(path_segments_list.begin(), path_segments_list.end(),
              std::less<OutSlotPathSegments>());
    opti_path_segments = path_segments_list.front();
    return true;
  }

  LOCAL_LOG(LOCAL_DEBUG, "use CP path");

  // adjust theta
  std::vector<OutSlotAdjustPath> cp_segments;
  adjustOutslotCP(start_pose, cp_segments);

  for (auto &seg : cp_segments) {
    LOCAL_LOG(LOCAL_INFO, "use adjust theta");
    if (seg.final_path.empty()) {
      continue;
    }
    if ((seg.is_forward && block_direc == 1) ||
        (!seg.is_forward && block_direc == -1)) {
      LOCAL_LOG(LOCAL_DEBUG,
                "direction is block, is forward: %d, block direc: %d",
                int(seg.is_forward), block_direc);
      continue;
    }
    Pose2D new_start_pose = seg.final_path.front();
    path_segments_list.clear();

    for (unsigned int i = 0; i < impl_.rs_size; i++) {
      OutSlotPathSegments path_seg;
      if (!spiralCscPath(escape_pose, new_start_pose, path_seg, seg,
                         impl_.rs[i], 0)) {
        LOCAL_LOG(LOCAL_DEBUG, "failed to use r = %.2f", impl_.rs[i]);
        continue;
      }
      if (!path_seg.accumPath(para_.max_accum_error)) {
        continue;
      }
      path_segments_list.push_back(path_seg);
    }

    if (path_segments_list.empty()) {
      continue;
    }
    std::sort(path_segments_list.begin(), path_segments_list.end(),
              std::less<OutSlotPathSegments>());
    opti_path_segments = path_segments_list.front();

    std::vector<Pose2D> last_seg_path;
    if (!seg.getPath(last_seg_path)) {
      continue;
    }
    opti_path_segments.final_path.insert(opti_path_segments.final_path.end(),
                                         last_seg_path.begin(),
                                         last_seg_path.end());
    return true;
  }

  LOCAL_LOG(LOCAL_DEBUG, "use SPCP path");
  std::vector<OutSlotAdjustPath> spcp_segments;
  adjustOutslotSPCP(init_pose_, spcp_segments);
  for (auto &spcp_seg : spcp_segments) {
    if ((spcp_seg.is_forward && block_direc == 1) ||
        (!spcp_seg.is_forward && block_direc == -1)) {
      LOCAL_LOG(LOCAL_DEBUG,
                "direction is block, is forward: %d, block direc: %d",
                int(spcp_seg.is_forward), block_direc);
      continue;
    }

    std::vector<Pose2D> spcp_path;
    if (!spcp_seg.getPath(spcp_path)) {
      LOCAL_LOG(LOCAL_DEBUG, "plan spcp is empty");
      return false;
    }
    if (spcp_path.empty()) {
      continue;
    }

    Pose2D new_start_pose = spcp_path.front();
    path_segments_list.clear();

    for (unsigned int i = 0; i < impl_.rs_size; i++) {
      OutSlotPathSegments path_seg;
      if (!spiralCscPath(escape_pose, new_start_pose, path_seg, spcp_seg,
                         impl_.rs[i], 0)) {
        LOCAL_LOG(LOCAL_DEBUG, "failed to use r = %.2f", impl_.rs[i]);
        continue;
      }
      if (!path_seg.accumPath(para_.max_accum_error)) {
        continue;
      }
      path_segments_list.push_back(path_seg);
    }

    if (path_segments_list.empty()) {
      continue;
    }

    std::sort(path_segments_list.begin(), path_segments_list.end(),
              std::less<OutSlotPathSegments>());
    opti_path_segments = path_segments_list.front();
    opti_path_segments.final_path.insert(opti_path_segments.final_path.end(),
                                         spcp_path.begin(), spcp_path.end());
    return true;
  }

  LOCAL_LOG(LOCAL_DEBUG, "need LaegeC path");
  std::vector<OutSlotAdjustPath> large_c_segments;
  adjustOutslotLargeC(start_pose, large_c_segments);
  for (auto &seg : large_c_segments) {
    LOCAL_LOG(LOCAL_INFO, "use adjust height");
    if (seg.final_path.empty()) {
      continue;
    }
    if ((seg.is_forward && block_direc == 1) ||
        (!seg.is_forward && block_direc == -1)) {
      LOCAL_LOG(LOCAL_DEBUG,
                "direction is block, is forward: %d, block direc: %d",
                int(seg.is_forward), block_direc);
      continue;
    }
    Pose2D new_start_pose = seg.final_path.front();
    path_segments_list.clear();

    for (unsigned int i = 0; i < impl_.rs_size; i++) {
      OutSlotPathSegments path_seg;
      if (!spiralCscPath(escape_pose, new_start_pose, path_seg, seg,
                         impl_.rs[i], 0)) {
        LOCAL_LOG(LOCAL_DEBUG, "failed to use r = %.2f", impl_.rs[i]);
        continue;
      }
      if (!path_seg.accumPath(para_.max_accum_error)) {
        continue;
      }
      path_segments_list.push_back(path_seg);
    }

    if (path_segments_list.empty()) {
      continue;
    }
    std::sort(path_segments_list.begin(), path_segments_list.end(),
              std::less<OutSlotPathSegments>());
    opti_path_segments = path_segments_list.front();

    std::vector<Pose2D> last_seg_path;
    if (!seg.getPath(last_seg_path)) {
      continue;
    }
    opti_path_segments.final_path.insert(opti_path_segments.final_path.end(),
                                         last_seg_path.begin(),
                                         last_seg_path.end());
    return true;
  }

  // sdjust lat
  LOCAL_LOG(LOCAL_DEBUG, "use CPPC path");
  cp_segments.emplace_back(OutSlotAdjustPath());
  cp_segments[0].final_path.push_back(start_pose);
  std::vector<std::vector<Pose2D>> cppc_path_list;

  for (auto &seg : cp_segments) {
    if (seg.final_path.empty()) {
      continue;
    }
    if (cp_segments[0].final_path.size() > 1 &&
        ((seg.is_forward && block_direc == 1) ||
         (!seg.is_forward && block_direc == -1))) {
      LOCAL_LOG(LOCAL_DEBUG,
                "direction is block, is forward: %d, block direc: %d",
                int(seg.is_forward), block_direc);
      continue;
    }
    std::vector<OutSlotAdjustPath> cppc_segments;
    if (!adjustOutSlotCPPCs(seg.final_path.front(), seg, cppc_segments)) {
      LOCAL_LOG(LOCAL_DEBUG, "adjust out slot lat failed");
      continue;
    }
    for (auto &lat_seg : cppc_segments) {
      std::vector<Pose2D> last_lat_path;
      if (!lat_seg.getPath(last_lat_path)) {
        LOCAL_LOG(LOCAL_DEBUG, "get path failed");
        continue;
      }
      if (cp_segments[0].final_path.size() == 1 &&
          ((lat_seg.is_forward && block_direc == 1) ||
           (!lat_seg.is_forward && block_direc == -1))) {
        LOCAL_LOG(LOCAL_DEBUG,
                  "direction is block, is forward: %d, block direc: %d",
                  int(lat_seg.is_forward), block_direc);
        continue;
      }
      cppc_path_list.push_back(last_lat_path);
    }
  }
  for (auto &cppc_path : cppc_path_list) {
    path_segments_list.clear();
    for (unsigned int i = 0; i < impl_.rs_size; i++) {
      OutSlotPathSegments path_seg;
      OutSlotAdjustPath empty_last_path;
      if (!spiralCscPath(escape_pose, cppc_path.front(), path_seg,
                         empty_last_path, impl_.rs[i], 0)) {
        LOCAL_LOG(LOCAL_DEBUG, "failed to use r = %.2f", impl_.rs[i]);
        continue;
      }
      if (!path_seg.accumPath(para_.max_accum_error)) {
        continue;
      }
      path_segments_list.push_back(path_seg);
    }

    if (!path_segments_list.empty()) {
      std::sort(path_segments_list.begin(), path_segments_list.end(),
                std::less<OutSlotPathSegments>());
      opti_path_segments = path_segments_list.front();
      opti_path_segments.final_path.insert(opti_path_segments.final_path.end(),
                                           cppc_path.begin(), cppc_path.end());
      return true;
    }
  }
  return false;
}
bool ParallelRulePlanner::findEscapePoseV2(Pose2D &escape_pose,
                                         InSlotPathSegemnts &path_segments){
                                          // min safe distance
  planning_math::Vec2d rot_center(0.0, para_.min_radius);
  Pose2D ego_origin(0.0, 0.0, 0.0);
  std::vector<planning_math::Vec2d> corners, corners_temp;
  csg_.getRawShape(ego_origin, corners, 0.0, 0.0);
  double corner_obs_distance = std::min(para_.side_safe_distance, corner_safe_distance_);
  double l1 = rot_center.DistanceTo(corners[4]);
  double l2 = rot_center.DistanceTo(corners[5]);
  double l1_theta =
      std::atan2(corners[4].x(), para_.min_radius - corners[4].y());
  double l2_theta =
      std::atan2(corners[5].x(), para_.min_radius - corners[5].y());
  bool is_l2_l1 = l2 > l1;
  double min_r_dis = std::max(l1, l2) + corner_obs_distance;
  double min_step_size = std::max(para_.min_block_len, para_.lon - 0.2); // 最小弧长
  double theta_epsilon = min_step_size / para_.min_radius - 1e-6; // 最小转角

  // back lon  which is larger than rotate lon
  double escape_lon = std::min(para_.lon + 0.10, 0.4);
  double escape_lat = std::min(para_.lat + 0.05, 0.19);

  // back_max_pose
  double back_max_limit = 3.0;
  double real_back_max = checker_.moveBackward(
      target_pose_, escape_lat, escape_lon, clothoid::ShapeType::RAW);
  double slot_back_dis =
      std::max(-impl_.left_corner.x() - csg_.back_to_rear_ - para_.lon, 0.0);
  if (impl_.left_corner.y() < -csg_.half_width_) {
    slot_back_dis = back_max_limit;
  }
  double back_real = std::max(0.0, std::min(slot_back_dis, real_back_max));
  double back_max = std::min(back_real, back_max_limit);
  Pose2D back_max_pose(target_pose_.x - back_max, target_pose_.y,
                       target_pose_.theta);
  Pose2D back_max_rotate_pose =
      checker_.rotateMaxPose(back_max_pose, clothoid::RotateType::LEFT_FORWARD,
                             clothoid::ShapeType::OCTAGON, para_.min_radius,
                             false, para_.lat, para_.lon);
  double real_dis = std::hypot(impl_.right_corner.x() - back_max_pose.x,
                               para_.min_radius - impl_.right_corner.y());
  csg_.getRawShape(back_max_rotate_pose, corners_temp, 0.0, 0.0);

  double real_forward_length = checker_.moveForward(
          target_pose_, escape_lat, escape_lon, clothoid::ShapeType::RAW);
  if (real_dis > min_r_dis && corners_temp[5].y() > impl_.right_corner.y()) {
    if (back_max < para_.min_block_len) {
      double need_forward_length = 2 * para_.min_block_len;
      if (real_forward_length > 2 * para_.min_block_len) {
        escape_pose = back_max_pose;
        path_segments.key_pose.push_back(target_pose_);
        path_segments.key_pose.emplace_back(target_pose_.x +
                                                need_forward_length,
                                            target_pose_.y, target_pose_.theta);
        path_segments.key_pose.push_back(escape_pose);
        return true;
      }
    } else {
      LOCAL_LOG(LOCAL_INFO, "just straight");
      escape_pose = back_max_pose;
      path_segments.key_pose.push_back(target_pose_);
      path_segments.key_pose.push_back(escape_pose);
      return true;
    }
  }

  if (real_forward_length < para_.min_block_len && back_max < para_.min_block_len) {
    LOCAL_LOG(LOCAL_INFO, "slot have no space to move straight to target pose");
    return false;
  }

  std::vector<Pose2D> local_key_poses;
  local_key_poses.push_back(target_pose_);
  bool is_escapable = false;
  Pose2D first_pose = back_max_pose;
  Pose2D front_pose = target_pose_;
  if (back_max < para_.min_block_len) {
    if (real_forward_length > 2 * para_.min_block_len) {
      front_pose.x = target_pose_.x + 2 * para_.min_block_len;
    } else {
      front_pose.x = target_pose_.x + real_forward_length;
    }
    local_key_poses.push_back(front_pose);
  }
  local_key_poses.push_back(first_pose);
  for (int i = 0;i < 4;i++) {
    is_escapable = isEscapable(first_pose, 1, 1, false, corner_obs_distance);
    if (is_escapable) {
      escape_pose = first_pose;
      break;
    }
    Pose2D next_pose;
    if (!getLeftForwardPose(first_pose, min_step_size, theta_epsilon, next_pose)) {
      break;
    }
    Pose2D end_pose;
    if (!getRightBackwardPose(next_pose, min_step_size, theta_epsilon, end_pose)) {
      break;
    }
    local_key_poses.push_back(next_pose);
    local_key_poses.push_back(end_pose);
    first_pose = end_pose;
  }

  if (!is_escapable) {
    return false;
  }
  
  path_segments.key_pose.insert(path_segments.key_pose.end(),
                                local_key_poses.begin(),
                                local_key_poses.end());
  return true;
}

bool ParallelRulePlanner::getLeftForwardPose(const Pose2D &start_pose, const double min_step_size, 
                                             const double theta_epsilon, Pose2D &next_pose) {
  next_pose = checker_.rotateMaxPose(start_pose, 1, 1, clothoid::ShapeType::OCTAGON, 
                                     para_.min_radius, false, para_.lat, para_.lon);
  if (!getRealRotatePose(start_pose, min_step_size, theta_epsilon, next_pose)) {
    return false;
  }
  Pose2D end_pose = checker_.rotateMaxPose(next_pose, -1, -1, clothoid::ShapeType::OCTAGON, 
                                           para_.min_radius, false, para_.lat, para_.lon);
  double deta_theta = std::abs(next_pose.theta - start_pose.theta);
  if (deta_theta < theta_epsilon) {
    return false;
  }
  double rotate_theta = std::abs(end_pose.theta - next_pose.theta);
  if (rotate_theta < theta_epsilon) {
    double start_pose_center_x = start_pose.x - para_.min_radius * std::sin(start_pose.theta);
    double start_pose_center_y = start_pose.y + para_.min_radius * std::cos(start_pose.theta);
    double step_theta = deta_theta - theta_epsilon;
    int i = 0;
    while (i < 5) {
      double alpha = start_pose.theta + step_theta;
      next_pose.x = start_pose_center_x + para_.min_radius * std::sin(alpha);
      next_pose.y = start_pose_center_y - para_.min_radius * std::cos(alpha);
      next_pose.theta = alpha;
      end_pose = checker_.rotateMaxPose(next_pose, -1, -1, clothoid::ShapeType::OCTAGON, 
                                        para_.min_radius, false, para_.lat, para_.lon);
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

bool ParallelRulePlanner::getRightBackwardPose(const Pose2D &start_pose, const double min_step_size, 
                                               const double theta_epsilon, Pose2D &next_pose) {
  next_pose = checker_.rotateMaxPose(start_pose, -1, -1, clothoid::ShapeType::OCTAGON, 
                                            para_.min_radius, false, para_.lat, para_.lon);
  Pose2D end_pose = checker_.rotateMaxPose(next_pose, 1, 1, clothoid::ShapeType::OCTAGON, 
                                           para_.min_radius, false, para_.lat, para_.lon);
  double deta_theta = std::abs(next_pose.theta - start_pose.theta);
  if (deta_theta < theta_epsilon) {
    return false;
  }
  double rotate_theta = end_pose.theta - next_pose.theta;
  if (rotate_theta < theta_epsilon) {
    double start_pose_center_x = start_pose.x + para_.min_radius * std::sin(start_pose.theta);
    double start_pose_center_y = start_pose.y - para_.min_radius * std::cos(start_pose.theta);
    double step_theta = deta_theta - theta_epsilon;
    int i = 0;
    while (i < 5) {
      double alpha = start_pose.theta + step_theta;
      next_pose.x = start_pose_center_x - para_.min_radius * std::sin(alpha);
      next_pose.y = start_pose_center_y + para_.min_radius * std::cos(alpha);
      next_pose.theta = alpha;
      end_pose = checker_.rotateMaxPose(next_pose, 1, 1, clothoid::ShapeType::OCTAGON, 
                                        para_.min_radius, false, para_.lat, para_.lon);
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

bool ParallelRulePlanner::getRealRotatePose(const Pose2D &start_pose, const double min_step_size, 
                                            const double theta_epsilon, Pose2D &next_pose) {
  double rot_center_x = start_pose.x - para_.min_radius * std::sin(start_pose.theta);
  double rot_center_y = start_pose.y + para_.min_radius * std::cos(start_pose.theta);
  planning_math::Vec2d rot_center(rot_center_x, rot_center_y);
  std::vector<planning_math::Vec2d> next_pose_corners;
  csg_.getRawShape(next_pose, next_pose_corners, 0.0, 0.0);
  if (next_pose_corners[5].y() > impl_.right_corner.y()) {
    double l2 = rot_center.DistanceTo(next_pose_corners[5]);
    planning_math::Vec2d next_pose_vec2d(next_pose.x, next_pose.y);
    double l3 = next_pose_vec2d.DistanceTo(next_pose_corners[5]);
    double cos_constant_theta = (para_.min_radius * para_.min_radius + l2 * l2 - l3 * l3) / (2 * para_.min_radius * l2);
    double constant_theta = std::acos(cos_constant_theta);
    double cos_alpha = (rot_center_y - impl_.right_corner.y()) / l2;
    double alpha = std::acos(cos_alpha);
    double deta_theta = alpha - start_pose.theta - constant_theta;
    double real_theta = start_pose.theta + deta_theta;
    double theta_max = next_pose.theta;

    if (deta_theta > theta_epsilon) {
      next_pose.x = rot_center_x + para_.min_radius * std::sin(real_theta);
      next_pose.y = rot_center_y - para_.min_radius * std::cos(real_theta);
      next_pose.theta = real_theta;
    } else {
      if (theta_max > 2 * theta_epsilon) {
        real_theta = start_pose.theta + 2 * theta_epsilon;
        next_pose.x = rot_center_x + para_.min_radius * std::sin(real_theta);
        next_pose.y = rot_center_y - para_.min_radius * std::cos(real_theta);
        next_pose.theta = real_theta;
      } else if (theta_max > theta_epsilon) {
        real_theta = start_pose.theta + theta_epsilon;
        next_pose.x = rot_center_x + para_.min_radius * std::sin(real_theta);
        next_pose.y = rot_center_y - para_.min_radius * std::cos(real_theta);
        next_pose.theta = real_theta;
      } else {
        LOCAL_LOG(LOCAL_INFO, "getRealRotatePose rotate theta is too small");
        return false;
      }
    }
  }
  return true;
}

bool ParallelRulePlanner::isEscapable(const Pose2D &start_pose, const int steer_dirct, 
                                      const int travel_direct, const bool is_init,
                                      double obs_distance) {
  // to get rotate centor, vehicle corners and min safe radius
  double rot_center_x = start_pose.x - para_.min_radius * std::sin(start_pose.theta);
  double rot_center_y = start_pose.y + para_.min_radius * std::cos(start_pose.theta);
  planning_math::Vec2d rot_center(rot_center_x, rot_center_y);
  std::vector<planning_math::Vec2d> corners;
  csg_.getRawShape(start_pose, corners, 0.0, 0.0);
  double l1 = rot_center.DistanceTo(corners[4]);
  double l2 = rot_center.DistanceTo(corners[5]);
  double min_r_dis = std::max(l1, l2) + obs_distance;

  // to get remotest rotate pose
  Pose2D next_pose = checker_.rotateMaxPose(start_pose, steer_dirct, travel_direct, 
                                            clothoid::ShapeType::OCTAGON, para_.min_radius, 
                                            is_init, para_.lat, para_.lon);

  double deta_theta = next_pose.theta - start_pose.theta;
  std::vector<planning_math::Vec2d> rotate_corners;
  csg_.getRawShape(next_pose, rotate_corners, 0.0, 0.0);

  if (rotate_corners[5].y() > impl_.right_corner.y() || deta_theta > M_PI_2) {
    std::vector<planning_math::Vec2d> obs_poses_in_distance;
    planning_math::Vec2d obs_poses_center(impl_.right_corner.x(), impl_.right_corner.y());

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

    // to judge if rotate path safe
    if (obs_poses_in_distance.empty()) {
      double obs_dis_to_corner = rot_center.DistanceTo(obs_poses_center);
      if (obs_dis_to_corner < (min_r_dis - obs_distance)) {
        return false;
      } else {
        return true;
      }
    }
    for (planning_math::Vec2d obs_pose_in_dis : obs_poses_in_distance) {
      double obs_dis_to_rotate_center = rot_center.DistanceTo(obs_pose_in_dis);
      if (obs_dis_to_rotate_center < min_r_dis) {
        return false;
      }
    }
  } else {
    return false;
  }

  return true;
}

bool ParallelRulePlanner::findEscapePose(Pose2D &escape_pose,
                                         InSlotPathSegemnts &path_segments) {
  // min safe distance
  planning_math::Vec2d rot_center(0.0, para_.min_radius);
  Pose2D ego_origin(0.0, 0.0, 0.0);
  std::vector<planning_math::Vec2d> corners, corners_temp;
  csg_.getRawShape(ego_origin, corners, 0.0, 0.0);
  double l1 = rot_center.DistanceTo(corners[4]);
  double l2 = rot_center.DistanceTo(corners[5]);
  double l1_theta =
      std::atan2(corners[4].x(), para_.min_radius - corners[4].y());
  double l2_theta =
      std::atan2(corners[5].x(), para_.min_radius - corners[5].y());
  bool is_l2_l1 = l2 > l1;

  // add by xhy: to calc min_r_dis
  std::vector<double> obs_poses_distance;
  planning_math::Vec2d obs_poses_center(impl_.right_corner.x(), impl_.right_corner.y());
  double obs_distance = para_.side_safe_distance;
  double min_r_dis = std::max(l1, l2) + para_.side_safe_distance;
  // to get obs points in safe ditance
  for (planning_math::Vec2d obs_pose : checker_.obs_pts_) {
    double obs_pose_dis = obs_poses_center.DistanceTo(obs_pose);
    if (obs_pose_dis < obs_distance) {
      obs_poses_distance.push_back(obs_pose_dis);
    }
  }
  // to deal obs wall
  for (planning_math::LineSegment2d obs_line : checker_.obs_lines_) {
    planning_math::Vec2d obs_line_start = obs_line.start();
    planning_math::Vec2d obs_line_end = obs_line.end();
    double obs_line_start_dis = obs_poses_center.DistanceTo(obs_line_start);
    double obs_line_end_dis = obs_poses_center.DistanceTo(obs_line_end);
    if (obs_line_start_dis < obs_distance || obs_line_end_dis < obs_distance) {
      obs_poses_distance.push_back(0.0);
    }
  }
  if (obs_poses_distance.empty()) {
    min_r_dis -= obs_distance;
  } else {
    double min_dis = 10000.0;
    for (double obs_pose_distance : obs_poses_distance) {
      if (obs_pose_distance < min_dis) {
        min_dis = obs_pose_distance;
      }
    }
    min_r_dis -= min_dis;
  }

  // back lon  which is larger than rotate lon
  double escape_lon = std::min(para_.lon + 0.05, 0.3);
  double escape_lat = std::min(para_.lat + 0.05, 0.19);
  double max_lat_error = 0.1;
  double min_lat_error = -0.05;

  // back_max_pose
  double back_max_limit = 3.0;
  double real_back_max = checker_.moveBackward(
      target_pose_, escape_lat, escape_lon, clothoid::ShapeType::RAW);
  double slot_back_dis =
      std::max(-impl_.left_corner.x() - csg_.back_to_rear_ - para_.lon, 0.0);
  if (impl_.left_corner.y() < -csg_.half_width_) {
    slot_back_dis = back_max_limit;
  }
  double back_real = std::max(0.0, std::min(slot_back_dis, real_back_max));
  double back_max = std::min(back_real, back_max_limit);
  Pose2D back_max_pose(target_pose_.x - back_max, target_pose_.y,
                       target_pose_.theta);
  Pose2D back_max_rotate_pose =
      checker_.rotateMaxPose(back_max_pose, clothoid::RotateType::LEFT_FORWARD,
                             clothoid::ShapeType::OCTAGON, para_.min_radius,
                             false, para_.lat, para_.lon);
  double real_dis = std::hypot(impl_.right_corner.x() - back_max_pose.x,
                               para_.min_radius - impl_.right_corner.y());
  csg_.getRawShape(back_max_rotate_pose, corners_temp, 0.0, 0.0);

  if (real_dis > min_r_dis && corners_temp[5].y() > impl_.right_corner.y()) {
    if (back_max < para_.min_block_len) {
      double real_forward_length = checker_.moveForward(
          target_pose_, escape_lat, escape_lon, clothoid::ShapeType::RAW);
      double need_forward_length = 2 * para_.min_block_len;
      if (real_forward_length > 2 * para_.min_block_len) {
        escape_pose = back_max_pose;
        path_segments.key_pose.push_back(target_pose_);
        path_segments.key_pose.emplace_back(target_pose_.x +
                                                need_forward_length,
                                            target_pose_.y, target_pose_.theta);
        path_segments.key_pose.push_back(escape_pose);
        return true;
      }

    } else {
      LOCAL_LOG(LOCAL_INFO, "just straight");
      escape_pose = back_max_pose;
      path_segments.key_pose.push_back(target_pose_);
      path_segments.key_pose.push_back(escape_pose);
      return true;
    }
  }

  double target_offset = std::numeric_limits<double>::max();
  Pose2D new_target = target_pose_;
  double height_y = std::max(impl_.right_corner.y(), 0.0);
  // lat toreance error is alrger when slot is narrow
  Pose2D back_pose, corner_pose, pose1;
  // constant theta
  csg_.getRawShape(ego_origin, corners_temp, escape_lat, escape_lon);
  planning_math::Vec2d pb = corners_temp[2];
  csg_.getWheelBaseShape(ego_origin, corners_temp, escape_lat, escape_lon);
  planning_math::Vec2d pc = corners_temp[1];

  double a = pb.DistanceTo(pc);
  double b = rot_center.DistanceTo(pb);
  double c = rot_center.DistanceTo(pc);
  double phi = std::acos((a * a + b * b - c * c) / (2 * a * b));
  double the1 = M_PI_2 - std::abs((pc - pb).Angle());
  LOCAL_LOG(LOCAL_DEBUG, "phi:%f \t the1:%f", phi, the1);
  LOCAL_LOG(LOCAL_DEBUG, "c:%f \t r+width:%f", c,
            para_.min_radius + para_.width / 2.0);

  double add_theta;

  // constant

  int try_times = 0;
  double min_target_offset = std::numeric_limits<double>::max();
  Pose2D min_offset_pose;
  InSlotPathSegemnts min_offset_path_segments;
  bool is_narrow_slot = true;
  while (try_times < para_.escape_sdjust_time) {
    if (impl_.right_corner.x() - impl_.left_corner.x() >= 1.1 + para_.length) {
      is_narrow_slot = false;
      if (target_offset <= max_lat_error && target_offset >= min_lat_error) {
        break;
      }
    }
    try_times++;
    double coeff = target_offset > 0 ? 1.2 : -1.2;
    if (try_times > 1) {
      new_target.y -= coeff * std::min(std::abs(target_offset), 0.15);
    }
    LOCAL_LOG(LOCAL_INFO, "try times:%d with new target-y:%f", try_times,
              new_target.y);
    planning_math::Vec2d new_rot_center(0.0, para_.min_radius + new_target.y);

    // get x1
    if (height_y > new_rot_center.y()) {
      LOCAL_LOG(LOCAL_DEBUG, "right corner height %.3f is too high than %.3f",
                  height_y, new_rot_center.y());
      if (!is_narrow_slot) {
        return false;
      } else {
        continue;
      }
    }

    double delta_x = std::sqrt(min_r_dis * min_r_dis -
                               msquare::pow2(new_rot_center.y() - height_y));
    pose1 = Pose2D(impl_.right_corner.x() - delta_x, new_target.y,
                   new_target.theta);

    //  get x2
    double corner_theta = std::atan2(delta_x, new_rot_center.y() - height_y);
    double move_theta = corner_theta - (is_l2_l1 ? l2_theta : l1_theta);
    double x2 = pose1.x + para_.min_radius * std::sin(move_theta);
    double y2 = pose1.y + para_.min_radius * (1 - std::cos(move_theta));
    corner_pose = Pose2D(x2, y2, move_theta);
    back_pose =
        checker_.rotateMaxPose(corner_pose, clothoid::RotateType::LEFT_BCAKWARD,
                               clothoid::ShapeType::OCTAGON, para_.min_radius,
                               false, escape_lat, escape_lon);
    if (back_pose.theta > corner_pose.theta) {
      LOCAL_LOG(LOCAL_DEBUG, "not valid theta");
      if (!is_narrow_slot) {
        return false;
      } else {
        continue;
      }
    }
    if (back_pose.theta < 0.0 && try_times == 1) {
      back_pose = checker_.rotateTheta(corner_pose, 1, -1, para_.min_radius,
                                       corner_pose.theta);
      LOCAL_LOG(LOCAL_DEBUG, "use the init pose");
    }

    if (back_pose.theta < 0.0 && try_times > 1) {
      LOCAL_LOG(LOCAL_DEBUG, "not valid theta 2");
      if (!is_narrow_slot) {
        return false;
      } else {
        continue;
      }
    }

    if (!inslotCSPath(path_segments, back_pose, false, 0, false)) {
      LOCAL_LOG(LOCAL_DEBUG, "plan fail");
      if (!is_narrow_slot) {
        return false;
      } else {
        continue;
      }
    }
    if (path_segments.key_pose.empty()) {
      if (!is_narrow_slot) {
        return false;
      } else {
        continue;
      }
    }
    target_offset = path_segments.key_pose.front().y;
    LOCAL_LOG(LOCAL_INFO, "offset:%f ", target_offset);
    if (std::abs(target_offset) < min_target_offset) {
      min_target_offset = std::abs(target_offset);
      min_offset_pose = back_pose;
      min_offset_path_segments.reset();
      min_offset_path_segments = path_segments;
    }
  }
  target_offset = min_target_offset;
  back_pose = min_offset_pose;
  path_segments.reset();
  path_segments = min_offset_path_segments;
  if (target_offset > max_lat_error || target_offset < min_lat_error) {
    return false;
  }

  escape_pose = back_pose;

  csg_.getRotateShape(back_pose, corners, escape_lat, escape_lon);
  double back_dis =
      std::min(1.0, std::max(corners[3].x() - impl_.left_corner.x() - 0.1,
                             0.0)); // there is 0.1 remain margin
  if (back_dis < 0.05) { // if adjust dis is small, then no need to try it
    return true;
  }

  /* adjust back distance*/
  LOCAL_LOG(LOCAL_INFO, "adjust back dis:%f", back_dis);
  // back_pose.x -=back_dis;

  back_pose.x -= back_dis;

  InSlotPathSegemnts path_segments_candidate;

  csg_.getWheelBaseShape(back_pose, corners_temp, escape_lat, escape_lon);
  planning_math::Vec2d rotate_corner = corners_temp[1];
  double ra = rotate_corner.DistanceTo(impl_.right_corner);
  double rb = c;
  double rc = min_r_dis;
  double r_theta1 =
      (rc > ra + rb) ? 0
                     : std::acos((ra * ra + rb * rb - rc * rc) / (2 * ra * rb));
  double r_theta2 = std::atan2(impl_.right_corner.y() - rotate_corner.y(),
                               impl_.right_corner.x() - rotate_corner.x());
  double r_add_theta = (rot_center - pc).Angle();
  double r_theta = r_theta1 + r_theta2 - r_add_theta;

  double center_theta = std::atan2(0 - pc.y(), 0 - pc.x());
  double ll = std::hypot(pc.y(), pc.x());
  Pose2D n_pose(rotate_corner.x() + ll * std::cos(center_theta + r_theta),
                rotate_corner.y() + ll * std::sin(center_theta + r_theta),
                r_theta);
  // n_pose = back_pose;
  if (!checker_.checkSinglePose(n_pose, para_.lat, para_.lon,
                                clothoid::ShapeType::ROTATE) ||
      !checker_.checkSinglePoseWheelBase(n_pose, para_.lat, para_.lon,
                                         clothoid::ShapeType::WHEEL_BASE)) {
    LOCAL_LOG(LOCAL_DEBUG, "failed to check new back pose rotate");
    return true;
  }
  csg_.getRotateShape(n_pose, corners, escape_lat, escape_lon);
  csg_.getRotateShape(escape_pose, corners, escape_lat, escape_lon);

  LOCAL_LOG(LOCAL_INFO,
            "r_theta1:%.3f  r_theta2a:%.3f r_add_theta:%.3f center theta:%.3f",
            r_theta1, r_theta2, r_add_theta, center_theta);
  LOCAL_LOG(LOCAL_INFO, "init theta:%.3f  after theta:%.3f", back_pose.theta,
            r_theta);
  if (back_pose.theta < r_theta) {
    LOCAL_LOG(LOCAL_DEBUG, "failed to calculate theta diff");
    return true;
  }

  double add_forward_length = checker_.moveForward(
      n_pose, para_.lat, escape_lon, clothoid::ShapeType::RAW);
  double add_forward = 0.5 * std::min(1.0, add_forward_length);
  add_forward = 0.0;
  // Pose2D str_pose(n_pose.x + add_forward * std::cos(n_pose.theta), n_pose.y +
  // add_forward * std::sin(n_pose.theta), n_pose.theta );
  path_segments_candidate.reset();
  if (!inslotCSPath(path_segments_candidate, n_pose, false, 0, true)) {
    LOCAL_LOG(LOCAL_DEBUG, "new pose plan fail");
    return true;
  }
  double new_target_offset = path_segments_candidate.key_pose.front().y;
  if (new_target_offset > max_lat_error || new_target_offset < min_lat_error) {
    LOCAL_LOG(LOCAL_DEBUG, "new_target_offset is not valid:%.3f",
              new_target_offset);
    return true;
  }

  LOCAL_LOG(LOCAL_DEBUG, "use new pose, before lat:%.3f  current lat:%.3f",
            target_offset, new_target_offset);

  path_segments = path_segments_candidate;

  escape_pose = n_pose;
  return true;
}

bool ParallelRulePlanner::Plan(const std::vector<SbpObstaclePtr> &obs_ptrs,
                               parking::SearchProcessDebug *sp_debug) {
  if (!checkStartEndPose()) {
    LOCAL_LOG(LOCAL_INFO, "failed to check start or end pose");
    return false;
  }
  LOCAL_LOG(LOCAL_INFO, "start x:%.3f, y:%.3f, theta:%.3f", init_pose_.x,
            init_pose_.y, init_pose_.theta);
  bool is_in_slot = isInSlot(init_pose_);
  std::vector<Pose2D> local_escape_path;

  if (is_in_slot && planInSlot(local_escape_path)) {
    if (local_escape_path.front().y > para_.invalid_lat_offset) {
      LOCAL_LOG(LOCAL_DEBUG,
                "failed to plan in slot for the lat offset is too large:%.3f",
                local_escape_path.back().y);

    } else {
      LOCAL_LOG(LOCAL_INFO, "success to plan in slot");
      generatePath(local_escape_path);
      return true;
    }
  }

  if (is_in_slot && !isInHalfSlot(init_pose_)) {
    LOCAL_LOG(LOCAL_DEBUG, "complete in slot and plan fail");
    return false;
  }

  int block_direc = is_in_slot
                    ? impl_.block_direc
                    : 0; // consider block direction when car is in slot
  OutSlotPathSegments outslot_path_segments;
  
  if(planOneTry(target_pose_,init_pose_,block_direc)){
    LOCAL_LOG(LOCAL_DEBUG, "successful to plan a path in one try");
    return true;
  }

  Pose2D escape_pose;
  InSlotPathSegemnts escape_path_segments;
  if (!findEscapePoseV2(escape_pose, escape_path_segments)) {
    if (!findEscapePose(escape_pose, escape_path_segments)) {
      LOCAL_LOG(LOCAL_DEBUG, "failed to find escape pose");
      return false;
    }
  }

  if (!planOutslot(escape_pose, init_pose_, outslot_path_segments,
                   block_direc)) {
    LOCAL_LOG(LOCAL_DEBUG, "failed to plan out slot");
    return false;
  }

  std::vector<Pose2D> local_outslot_path;
  if (!outslot_path_segments.getPath(local_outslot_path)) {
    LOCAL_LOG(LOCAL_DEBUG, "plan outslot is empty");
    return false;
  }

  local_escape_path = escape_path_segments.interpolatePath(
      para_.step, para_.step / para_.min_radius);
  local_escape_path.insert(local_escape_path.end(), local_outslot_path.begin(),
                           local_outslot_path.end());

  generatePath(local_escape_path);
  LOCAL_LOG(LOCAL_INFO, "successful to plan a path");
  return true;
}

void ParallelRulePlanner::generatePath(const std::vector<Pose2D> &local_path) {
  std::vector<Pose2D> global_path;
  if (impl_.is_on_left) {
    for (auto &p : local_path) {
      global_path.push_back(
          planning_math::tf2d_inv(local_frame_pose_, localMirrorX(p)));
    }
  } else {
    for (auto &p : local_path) {
      global_path.push_back(planning_math::tf2d_inv(local_frame_pose_, p));
    }
  }
  for (auto p : global_path) {
    result_.x.push_back(p.x);
    result_.y.push_back(p.y);
    result_.phi.push_back(p.theta);
  }
}

bool ParallelRulePlanner::planInSlot(std::vector<Pose2D> &in_slot_path) {
  InSlotPathSegemnts path_segments;
  if (!inslotCSPath(path_segments, init_pose_, true, impl_.block_direc, true)) {
    return false;
  }

  in_slot_path.clear();
  in_slot_path =
      path_segments.interpolatePath(para_.step, para_.step / para_.min_radius);
  return true;
}

bool ParallelRulePlanner::inslotCSPath(InSlotPathSegemnts &path_segments,
                                       const Pose2D &pose, bool is_init,
                                       int block_direc, bool is_final) {
  if (planInSlotCurve(path_segments, pose, is_init, block_direc, is_final)) {
    return true;
  }
  LOCAL_LOG(LOCAL_DEBUG, "curve fail");

  if (planInSlotKeepStraight(path_segments, pose, is_init, block_direc)) {
    return true;
  }
  LOCAL_LOG(LOCAL_DEBUG, "straight fail");

  return false;
}

bool ParallelRulePlanner::planInSlotKeepStraight(
    InSlotPathSegemnts &path_segments, const Pose2D &start_pose, bool is_init,
    int block_direc) {
  double x_diff = std::abs(start_pose.x - target_pose_.x);
  if (std::abs(start_pose.theta) > 0.03 || x_diff > 3.5 || x_diff < para_.min_block_len) {
    return false;
  }

  bool forward = start_pose.x < target_pose_.x;
  if (forward && block_direc == 1) {
    return false;
  }
  if (!forward && block_direc == -1) {
    return false;
  }
  double max_forward_dis =
      forward ? checker_.moveForward(start_pose, para_.lat, para_.lon,
                                     clothoid::ShapeType::RAW)
              : checker_.moveBackward(start_pose, para_.lat, para_.lon,
                                      clothoid::ShapeType::RAW);

  double straight_dis = x_diff / std::cos(start_pose.theta);
  if (straight_dis > max_forward_dis || straight_dis < para_.min_block_len) {
    return false;
  }

  if (!forward) {
    straight_dis = -straight_dis;
  }

  Pose2D new_target_pose_ = Pose2D(
      start_pose.x + straight_dis * cos(start_pose.theta),
      start_pose.y + straight_dis * sin(start_pose.theta), start_pose.theta);

  path_segments.reset();
  path_segments.key_pose.push_back(new_target_pose_);
  path_segments.key_pose.push_back(start_pose);

  return true;
}

bool ParallelRulePlanner::planInSlotCurve(InSlotPathSegemnts &path_segments,
                                          const Pose2D &pose, bool is_init_pose,
                                          int block_direc, bool is_final) {
  Pose2D start_pose = pose;
  double min_step_size = std::max(para_.min_block_len, para_.lon - 0.2); // 最小弧长
  double theta_epsilon = min_step_size / para_.min_radius - 1e-6; // 最小转角
  using SinglePath = std::pair<std::vector<Pose2D>, double>;
  std::vector<SinglePath> candidates;

  double penalty_times = is_init_pose ? 0.08 : 0;

  Pose2D turning_pose, next_pose;
  next_pose = start_pose;
  double forward_dis_all = checker_.moveForward(next_pose, para_.lat, para_.lon,
                                                clothoid::ShapeType::RAW); // max move forward distance in slot
  double backward_dis_all = checker_.moveBackward(
      next_pose, para_.lat, para_.lon, clothoid::ShapeType::RAW);
  bool is_success = false;
  double start_theta = next_pose.theta;
  if (start_theta > 0) {
    double max_forward_dis = forward_dis_all;

    std::vector<double> forward_dis_list;
    forward_dis_list.push_back(0.0);

    if (max_forward_dis > para_.min_block_len) {
      max_forward_dis = std::max(para_.min_block_len, std::min(1.5, max_forward_dis - 1e-6));
      int step_num = std::floor(max_forward_dis / 0.2);
      double step_dis = max_forward_dis / (step_num + 1);
      if (step_num > 0) {
        forward_dis_list.push_back(0.2);
      } else {
        forward_dis_list.push_back(max_forward_dis);
      }
      for (int m = 2; m <= step_num + 1; m++) {
        forward_dis_list.push_back(m * step_dis);
      }
    }

    for (double dis : forward_dis_list) {
      if (is_success) {
        break;
      }
      // LOCAL_LOG(LOCAL_INFO,"try forward dis:%.3f",dis );
      bool no_straight = false;
      Pose2D temp_next_pose;
      if (dis < 1e-6) {
        no_straight = true;
        temp_next_pose = start_pose;
      } else {
        if (block_direc == 1) { // block_direc, 0: 前后都能走, 1: 前面不能走, -1: 后面不能走, 只限制第一把
          continue;
        }
        temp_next_pose = Pose2D(start_pose.x + dis * cos(start_pose.theta),
                                start_pose.y + dis * sin(start_pose.theta),
                                start_pose.theta);
      }

      int j = 0;
      int direc_base = -1;
      while (j++ < 2 && !is_success) {
        std::vector<Pose2D> local_key_points;

        direc_base = -direc_base;
        int direc = direc_base;
        int i = 0;
        std::vector<Pose2D> seg_points;
        next_pose = temp_next_pose;
        while (next_pose.theta > 0.0 && i++ < 7 && isInSlot(next_pose)) {
          direc *= -1;
          if (no_straight && i == 1 && direc == block_direc) {
            break;
          }
          turning_pose = next_pose;
          seg_points.push_back(turning_pose);
          bool is_current_init = (is_init_pose && no_straight && i == 1);
          next_pose = checker_.rotateMaxPose(
              turning_pose, -direc, direc, clothoid::ShapeType::OCTAGON,
              para_.min_radius, is_current_init, para_.lat, para_.lon);
          if (std::abs(next_pose.theta - turning_pose.theta) < theta_epsilon) {
            // LOCAL_LOG(LOCAL_INFO,"theta is tiny: %.3f
            // %.3f",std::abs(next_pose.theta - turning_pose.theta),
            // theta_epsilon );
            break;
          }
        }

        if (next_pose.theta > 0.0) {
          // LOCAL_LOG(LOCAL_INFO,"try times:%d", i);
          continue;
        }
        next_pose =
            checker_.rotateTheta(turning_pose, -direc, direc, para_.min_radius,
                                 std::abs(turning_pose.theta));
        if (std::abs(turning_pose.theta) < theta_epsilon) {
          if (direc > 0 && next_pose.x > -para_.min_block_len) {
            continue;
          }
          if (direc < 0 && next_pose.x < para_.min_block_len) {
            continue;
          }
        }
        if (dis > 1e-6) {
          local_key_points.push_back(start_pose);
        }
        local_key_points.insert(local_key_points.end(), seg_points.begin(),
                                seg_points.end());
        local_key_points.push_back(next_pose);
        double offset_y = local_key_points.back().y;
        double score =
            (offset_y < 0 ? 2 : 1.0) * std::abs(offset_y); // less is better
        score += (no_straight ? i : i + 1) *
                 penalty_times; // penalty on reverse times
        candidates.emplace_back(local_key_points, score);
        // if(std::abs(offset_y) < para_.in_slot_curve_thres){
        //     is_success = true;
        //     LOCAL_LOG(LOCAL_DEBUG,"stop ahead for the offset is tiny:%.3f,
        //     score:%.3f", offset_y, score); break;
        // }
      } // end while
    }
    if (!is_success) {

      double max_forward_dis = backward_dis_all;

      std::vector<double> forward_dis_list;
      forward_dis_list.push_back(0.0);

      if (max_forward_dis > para_.min_block_len) {
        max_forward_dis = std::max(para_.min_block_len, std::min(1.5, max_forward_dis - 1e-6));
        int step_num = std::floor(max_forward_dis / 0.2);
        double step_dis = max_forward_dis / (step_num + 1);
        if (step_num > 0) {
          forward_dis_list.push_back(-0.2);
        } else {
          forward_dis_list.push_back(-max_forward_dis);
        }
        for (int m = 2; m <= step_num + 1; m++) {
          forward_dis_list.push_back(-m * step_dis);
        }
      }

      for (double dis : forward_dis_list) {
        // LOCAL_LOG(LOCAL_INFO,"try backward dis:%.3f",dis );
        if (is_success) {
          break;
        }
        bool no_straight = false;
        Pose2D temp_next_pose;
        if (std::abs(dis) < 1e-6) {
          no_straight = true;
          temp_next_pose = start_pose;
        } else {
          if (block_direc == -1) {
            continue;
          }
          temp_next_pose = Pose2D(start_pose.x + dis * cos(start_pose.theta),
                                  start_pose.y + dis * sin(start_pose.theta),
                                  start_pose.theta);
        }

        int j = 0;
        int direc_base = -1;
        while (j++ < 2 && (!is_success)) {
          std::vector<Pose2D> local_key_points;
          std::vector<Pose2D> seg_points;

          direc_base = -direc_base;
          int direc = direc_base;
          int i = 0;
          next_pose = temp_next_pose;
          while (next_pose.theta > 0.0 && i++ < 7 && isInSlot(next_pose)) {
            direc *= -1;
            if (no_straight && i == 1 && direc == block_direc) {
              break;
            }
            turning_pose = next_pose;
            seg_points.push_back(turning_pose);
            bool is_current_init = (is_init_pose && no_straight && i == 1);
            next_pose = checker_.rotateMaxPose(
                turning_pose, -direc, direc, clothoid::ShapeType::OCTAGON,
                para_.min_radius, is_current_init, para_.lat, para_.lon);

            if (std::abs(next_pose.theta - turning_pose.theta) <
                theta_epsilon) {
              break;
            }
          }
          if (next_pose.theta > 0.0) {
            continue;
          }
          next_pose = checker_.rotateTheta(turning_pose, -direc, direc,
                                           para_.min_radius,
                                           std::abs(turning_pose.theta));
          if (std::abs(turning_pose.theta) < theta_epsilon) {
            if (direc > 0 && next_pose.x > -para_.min_block_len) {
              continue;
            }
            if (direc < 0 && next_pose.x < para_.min_block_len) {
              continue;
            }
          }
          if (std::abs(dis) > 1e-6) {
            local_key_points.push_back(start_pose);
          }
          local_key_points.insert(local_key_points.end(), seg_points.begin(),
                                  seg_points.end());
          local_key_points.push_back(next_pose);
          double offset_y = local_key_points.back().y;
          double score =
              (offset_y < 0 ? 2 : 1.0) * std::abs(offset_y); // less is better
          score += (no_straight ? i : i + 1) * penalty_times;
          candidates.emplace_back(local_key_points, score);
          // if(std::abs(offset_y) < para_.in_slot_curve_thres){
          //     is_success = true;
          //     LOCAL_LOG(LOCAL_DEBUG,"stop ahead for the offset is tiny:%.3f",
          //     offset_y); break;
          // }

        } // end while
      }
    }
  } else if (start_theta < 0.0) {
    double max_forward_dis = backward_dis_all;
    std::vector<double> forward_dis_list;
    forward_dis_list.push_back(0.0);

    if (max_forward_dis > para_.min_block_len) {
      max_forward_dis = std::max(para_.min_block_len, std::min(1.5, max_forward_dis - 1e-6));
      int step_num = std::floor(max_forward_dis / 0.2);
      double step_dis = max_forward_dis / (step_num + 1);
      if (step_num > 0) {
        forward_dis_list.push_back(-0.2);
      } else {
        forward_dis_list.push_back(-max_forward_dis);
      }
      for (int m = 2; m <= step_num + 1; m++) {
        forward_dis_list.push_back(-m * step_dis);
      }
    }

    for (double dis : forward_dis_list) {
      LOCAL_LOG(LOCAL_INFO, "-theta try forward dis:%.3f", dis);
      if (is_success) {
        break;
      }
      bool no_straight = false;
      Pose2D temp_next_pose;
      if (std::abs(dis) < 1e-6) {
        no_straight = true;
        temp_next_pose = start_pose;
      } else {
        if (block_direc == -1) {
          continue;
        }
        temp_next_pose = Pose2D(start_pose.x + dis * cos(start_pose.theta),
                                start_pose.y + dis * sin(start_pose.theta),
                                start_pose.theta);
      }

      int j = 0;
      int direc_base = -1;
      while (j++ < 2 && (!is_success)) {
        std::vector<Pose2D> local_key_points;
        std::vector<Pose2D> seg_points;
        direc_base = -direc_base;
        int direc = direc_base;
        int i = 0;
        next_pose = temp_next_pose;
        while (next_pose.theta < 0.0 && i++ < 7 && isInSlot(next_pose)) {
          direc *= -1;
          if (i == 1 && direc == block_direc) {
            break;
          }
          turning_pose = next_pose;
          seg_points.push_back(turning_pose);
          bool is_current_init = (is_init_pose && no_straight && i == 1);
          next_pose = checker_.rotateMaxPose(
              turning_pose, direc, direc, clothoid::ShapeType::OCTAGON,
              para_.min_radius, is_current_init, para_.lat, para_.lon);
          if (std::abs(next_pose.theta - turning_pose.theta) < theta_epsilon) {
            break;
          }
        }
        if (next_pose.theta < 0.0) {
          continue;
        }
        next_pose =
            checker_.rotateTheta(turning_pose, direc, direc, para_.min_radius,
                                 std::abs(turning_pose.theta));
        if (std::abs(turning_pose.theta) < theta_epsilon) {
          if (direc > 0 && next_pose.x > -para_.min_block_len) {
            continue;
          }
          if (direc < 0 && next_pose.x < para_.min_block_len) {
            continue;
          }
        }

        if (abs(dis) > 1e-6) {
          local_key_points.push_back(start_pose);
        }
        local_key_points.insert(local_key_points.end(), seg_points.begin(),
                                seg_points.end());
        local_key_points.push_back(next_pose);
        double offset_y = local_key_points.back().y;
        double score =
            (offset_y < 0 ? 2 : 1.0) * std::abs(offset_y); // less is better
        score += (no_straight ? i : i + 1) * penalty_times;
        candidates.emplace_back(local_key_points, score);
        // if(std::abs(offset_y) < para_.in_slot_curve_thres){
        //     is_success = true;
        //     LOCAL_LOG(LOCAL_DEBUG,"stop ahead for the offset is tiny:%.3f",
        //     offset_y); break;
        // }

      } // end while

    } // end for dis
  }   // end else

  double max_offset_desired = getMaxOffsetCover(target_pose_, min_step_size, theta_epsilon);
  if (!is_success && candidates.empty()) {
    // add by xhy
    if (is_final) {
      std::vector<Pose2D> local_key_points;
      local_key_points.push_back(start_pose);
      if (planInSlotHorizon(local_key_points, start_pose, min_step_size, theta_epsilon, max_offset_desired, is_init_pose, block_direc)) {
        if (local_key_points.size() == 1) {
          LOCAL_LOG(LOCAL_INFO, "candidates empty");
          return false;
        }
        double offset_y = local_key_points.back().y;
        double score = (offset_y < 0 ? 2 : 1.0) * std::abs(offset_y); 
        candidates.emplace_back(local_key_points, score);
      } else{
        LOCAL_LOG(LOCAL_INFO, "candidates empty");
        return false;
      }
    } else {
      LOCAL_LOG(LOCAL_INFO, "candidates empty");
      return false;
    }
    // LOCAL_LOG(LOCAL_INFO, "candidates empty");
    // return false;
  }

  std::sort(candidates.begin(), candidates.end(),
            [](SinglePath a, SinglePath b) { return a.second < b.second; });

  std::vector<Pose2D> local_key_points = candidates.front().first;

  // add by xhy
  if (is_final) {
    Pose2D start_offset_pose = local_key_points.back();
    planInSlotHorizon(local_key_points, start_offset_pose, min_step_size, theta_epsilon, max_offset_desired, is_init_pose, block_direc);
  }

  next_pose = local_key_points.back();
  double straight_thres = para_.min_block_len;
  if (std::abs(next_pose.x) > straight_thres - 1e-6) {
    Pose2D new_target(target_pose_.x, next_pose.y, next_pose.theta);
    local_key_points.push_back(new_target);
  }

  std::reverse(local_key_points.begin(), local_key_points.end());
  path_segments.reset();
  path_segments.key_pose.insert(path_segments.key_pose.end(),
                                local_key_points.begin(),
                                local_key_points.end());

  return true;
}

double ParallelRulePlanner::getMaxOffsetCover(const Pose2D &pose, double min_step_size, 
                                              double theta_epsilon) {
  double max_len = std::abs(impl_.left_corner.x()) + std::abs(impl_.right_corner.x()) - para_.length;
  if (max_len < (2 * min_step_size)) {
    return 0.0;
  }
  if (max_len >= (2 * para_.min_radius)) {
    return 2 * para_.min_radius;
  }
  double sin_alpha = max_len / (2 * para_.min_radius);
  double max_ratate_alpha = std::asin(sin_alpha);
  if (max_ratate_alpha < theta_epsilon) {
    return 0.0;
  }
  double max_offset = 2 * para_.min_radius * (1 - std::cos(max_ratate_alpha));
  return max_offset;
}

bool ParallelRulePlanner::planInSlotHorizon(std::vector<Pose2D> &local_planning_key_points, 
                                          const Pose2D &pose, double min_step_size, 
                                          double theta_epsilon, double max_offset_desired,
                                          bool is_init, int block_direc) {
  Pose2D start_pose = pose;
  Pose2D end_pose = start_pose;
  Pose2D next_pose = start_pose;
  int direc = 1;
  using PointsAndOffset = std::pair<std::vector<Pose2D>, double>;
  std::vector<PointsAndOffset> points_and_offset;
  double max_arc_length = std::abs(start_pose.theta) * para_.min_radius;
  if (std::abs(start_pose.theta) <= theta_epsilon || max_arc_length <= min_step_size) {
    if (planInSlotOffset(local_planning_key_points, start_pose, min_step_size, theta_epsilon, max_offset_desired, is_init, block_direc)) {
      return true;
    } else {
      return false;
    }
  }

  if (start_pose.theta > 0.0) {
    if ((is_init && block_direc != 1) || !is_init) {
      next_pose = checker_.rotateMaxPose(start_pose, -direc, direc, // 右，前
                                        clothoid::ShapeType::OCTAGON,
                                        para_.min_radius, is_init, para_.lat, para_.lon);
      if (next_pose.theta < 0.0) {
        end_pose = Pose2D(start_pose.x + para_.min_radius * sin(start_pose.theta),
                          start_pose.y + para_.min_radius * (1 - cos(start_pose.theta)),
                          0.0);
        std::vector<Pose2D> tmp_points;
        tmp_points.push_back(end_pose);
        if (planInSlotOffset(tmp_points, end_pose, min_step_size, theta_epsilon, max_offset_desired, is_init, block_direc)) {
          Pose2D tmp_back_point = tmp_points.back();
          double end_offset = tmp_back_point.y;
          points_and_offset.emplace_back(tmp_points, end_offset);
        }
      }
    }

    if ((is_init && block_direc != -1) || !is_init) {
      next_pose = checker_.rotateMaxPose(start_pose, direc, -direc, // 左，后
                                        clothoid::ShapeType::OCTAGON,
                                        para_.min_radius, is_init, para_.lat, para_.lon);
      if (next_pose.theta < 0.0) {
        end_pose = Pose2D(start_pose.x - para_.min_radius * sin(start_pose.theta),
                          start_pose.y - para_.min_radius * (1 - cos(start_pose.theta)),
                          0.0);
        std::vector<Pose2D> tmp_points;
        tmp_points.push_back(end_pose);
        if (planInSlotOffset(tmp_points, end_pose, min_step_size, theta_epsilon, max_offset_desired, is_init, block_direc)) {
          Pose2D tmp_back_point = tmp_points.back();
          double end_offset = tmp_back_point.y;
          points_and_offset.emplace_back(tmp_points, end_offset);
        }
      }
    }
  } else {
    if ((is_init && block_direc != 1) || !is_init) {
      next_pose = checker_.rotateMaxPose(start_pose, direc, direc, // 左，前
                                        clothoid::ShapeType::OCTAGON,
                                        para_.min_radius, is_init, para_.lat, para_.lon);
      if (next_pose.theta > 0.0) {
        end_pose = Pose2D(start_pose.x + para_.min_radius * sin(std::abs(start_pose.theta)),
                          start_pose.y - para_.min_radius * (1 - cos(std::abs(start_pose.theta))),
                          0.0);
        std::vector<Pose2D> tmp_points;
        tmp_points.push_back(end_pose);
        if (planInSlotOffset(tmp_points, end_pose, min_step_size, theta_epsilon, max_offset_desired, is_init, block_direc)) {
          Pose2D tmp_back_point = tmp_points.back();
          double end_offset = tmp_back_point.y;
          points_and_offset.emplace_back(tmp_points, end_offset);
        }
      }
    }

    if ((is_init && block_direc != -1) || !is_init) {
      next_pose = checker_.rotateMaxPose(start_pose, -direc, -direc, // 右，后
                                        clothoid::ShapeType::OCTAGON,
                                        para_.min_radius, is_init, para_.lat, para_.lon);
      if (next_pose.theta > 0.0) {
        end_pose = Pose2D(start_pose.x - para_.min_radius * sin(std::abs(start_pose.theta)),
                          start_pose.y + para_.min_radius * (1 - cos(std::abs(start_pose.theta))),
                          0.0);
        std::vector<Pose2D> tmp_points;
        tmp_points.push_back(end_pose);
        if (planInSlotOffset(tmp_points, end_pose, min_step_size, theta_epsilon, max_offset_desired, is_init, block_direc)) {
          Pose2D tmp_back_point = tmp_points.back();
          double end_offset = tmp_back_point.y;
          points_and_offset.emplace_back(tmp_points, end_offset);
        }
      }
    }
  }

  if (points_and_offset.empty()) {
    LOCAL_LOG(LOCAL_INFO, "planInSlotHorizon points_and_offset empty");
    return false;
  }

  std::sort(points_and_offset.begin(), points_and_offset.end(), 
            [](PointsAndOffset a, PointsAndOffset b) {return a.second < b.second;});
  std::vector<Pose2D> local_offset_key_points = points_and_offset.front().first;
  local_planning_key_points.insert(local_planning_key_points.end(),
                                    local_offset_key_points.begin(),
                                    local_offset_key_points.end());
  return true;
}

bool ParallelRulePlanner::planInSlotOffset(std::vector<Pose2D> &local_planning_key_points, 
                                          const Pose2D &pose, double min_step_size, 
                                          double theta_epsilon, double max_offset_desired,
                                          bool is_init, int block_direc) {
  Pose2D end_pose = pose; // to select true end point
  double end_offset = end_pose.y;
  double target_offset = end_offset;
  double end_offset_limit = 0.05; // m
  double offset_step = 0.005; // m
  using PointsAndOffset = std::pair<std::vector<Pose2D>, double>;
  std::vector<PointsAndOffset> points_and_offset;
  using PointAndDirect = std::pair<Pose2D, int>;
  std::vector<PointAndDirect> point_and_direct;
  Pose2D start_pose = end_pose;
  Pose2D next_pose = start_pose;
  Pose2D turning_pose = start_pose;
  int direc = 1;
  int path_block_direct = 0;
  double added_dis = 0.0;
  double straight_dis_limit = 5.0; // m 
  double lon_left_dis = lon_left_dis_; // m

  if (end_offset < end_offset_limit) {
    return true;
  }

  // to calc max rotate radius
  // if (end_offset > max_offset_desired) {
  //   target_offset = max_offset_desired;
  // }
  double offset_to_staight_len = end_offset / straight_dis_limit;
  double straight_len_to_offset = straight_dis_limit / end_offset;
  double sin_rotate_angle = 2 / (offset_to_staight_len + straight_len_to_offset);
  double max_rotate_radiu = straight_dis_limit / (2 * sin_rotate_angle);
  std::vector<double> rotate_radiu_list;
  rotate_radiu_list.push_back(para_.min_radius);
  if (max_rotate_radiu > para_.min_radius) {
    double radiu_step = (max_rotate_radiu - para_.min_radius) / 5.0;
    for (double radiu_tmp = radiu_step + para_.min_radius;radiu_tmp <= max_rotate_radiu;radiu_tmp += radiu_step) {
      rotate_radiu_list.push_back(radiu_tmp);
    }
  }

  std::vector<double> offset_dis_list;
  int step_num = std::floor(target_offset / offset_step);
  for (int i = 0; i < step_num; i++) {
    double offset_dis = target_offset - i * offset_step;
    if (offset_dis < 0.01) {
      break;
    }
    offset_dis_list.push_back(offset_dis);
  }

  getStartPoseAndDirect(local_planning_key_points, min_step_size, start_pose, path_block_direct, added_dis);
  bool is_forward_point = false;
  double forward_dis = checker_.moveForward(start_pose, para_.lat, para_.lon,
                                            clothoid::ShapeType::RAW); // max move forward distance in slot
  double max_forward_dis = std::max(0.0, std::min(forward_dis - lon_left_dis, straight_dis_limit));
  if (max_forward_dis + added_dis > min_step_size && path_block_direct != 1) {
    Pose2D start_pose_forward = Pose2D(start_pose.x + max_forward_dis, start_pose.y, start_pose.theta);
    point_and_direct.emplace_back(start_pose_forward, -direc);
    is_forward_point = true;
  } else {
    if (path_block_direct != -1) {
      point_and_direct.emplace_back(start_pose, -direc);
    }
  }

  bool is_backward_point = false;
  double backward_dis = checker_.moveBackward(start_pose, para_.lat, para_.lon, 
                                              clothoid::ShapeType::RAW); // max move backward distance in slot
  double max_backward_dis = std::max(0.0, std::min(backward_dis - lon_left_dis, straight_dis_limit));
  if (max_backward_dis + added_dis > min_step_size && path_block_direct != -1) {
    Pose2D start_pose_backward = Pose2D(start_pose.x - max_backward_dis, start_pose.y, start_pose.theta);
    point_and_direct.emplace_back(start_pose_backward, direc);
    is_backward_point = true;
  } else {
    if (path_block_direct != 1) {
      point_and_direct.emplace_back(start_pose, direc);
    }
  }
  
  for (PointAndDirect point_direct_memeber : point_and_direct) {
    Pose2D straight_pose = point_direct_memeber.first;
    int member_direct = point_direct_memeber.second;
    double forward_dis_max = checker_.moveForward(straight_pose, para_.lat, para_.lon,
                                                  clothoid::ShapeType::RAW); 
    double forward_dis_all = std::min(forward_dis_max, straight_dis_limit);
    double backward_dis_max = checker_.moveBackward(straight_pose, para_.lat, para_.lon, 
                                                    clothoid::ShapeType::RAW); 
    double backward_dis_all = std::min(backward_dis_max, straight_dis_limit);

    bool is_forward_success = false;
    bool is_backward_success = false;
    for (double offset_dis : offset_dis_list) {
      for (double rotate_radiu : rotate_radiu_list) {
        theta_epsilon = min_step_size / rotate_radiu - 1e-6; // 最小转角
        double cos_alpha = 1 - (offset_dis / (2.0 * rotate_radiu));
        double rotate_alpha = std::acos(cos_alpha);
        double to_cover_offset_length = 2.0 * rotate_radiu * std::sin(rotate_alpha);
        if (rotate_alpha < theta_epsilon) {
          continue;
        }

        if (member_direct == 1) {
          if ((is_init && block_direc != 1) || !is_init) {
            if (!is_forward_success) {
              if (forward_dis_all >= to_cover_offset_length) {
                next_pose = checker_.rotateMaxPose(
                        straight_pose, -direc, direc, clothoid::ShapeType::OCTAGON,
                        rotate_radiu, is_init, para_.lat, para_.lon); // direc, 1: 左, -1: 右; direc, 1: 前, -1: 后; 
                if (std::abs(next_pose.theta) > rotate_alpha) {
                  turning_pose = Pose2D(straight_pose.x + to_cover_offset_length / 2.0,
                                        straight_pose.y - offset_dis / 2.0,
                                        -rotate_alpha);
                  next_pose = checker_.rotateMaxPose(
                        turning_pose, direc, direc, clothoid::ShapeType::OCTAGON,
                        rotate_radiu, is_init, para_.lat, para_.lon);
                  if (next_pose.theta > 0) {
                    next_pose = Pose2D(turning_pose.x + to_cover_offset_length / 2.0,
                                      turning_pose.y - offset_dis / 2.0,
                                      0.0);
                    std::vector<Pose2D> tmp_points;
                    if (is_backward_point) {
                      tmp_points.push_back(straight_pose);
                    }
                    tmp_points.push_back(turning_pose);
                    tmp_points.push_back(next_pose);
                    points_and_offset.emplace_back(tmp_points, offset_dis);
                    is_forward_success = true;
                  }
                }
              }
            }
          }
        }

        if (member_direct == -1) {
          if ((is_init && block_direc != -1) || !is_init) {
            if (!is_backward_success) {
              if (backward_dis_all >= to_cover_offset_length) {
                next_pose = checker_.rotateMaxPose(
                        straight_pose, -direc, -direc, clothoid::ShapeType::OCTAGON,
                        rotate_radiu, is_init, para_.lat, para_.lon); // direc, 1: 左, -1: 右; direc, 1: 前, -1: 后; 
                if (next_pose.theta > rotate_alpha) {
                  turning_pose = Pose2D(straight_pose.x - to_cover_offset_length / 2.0,
                                        straight_pose.y - offset_dis / 2.0,
                                        rotate_alpha);
                  next_pose = checker_.rotateMaxPose(
                        turning_pose, direc, -direc, clothoid::ShapeType::OCTAGON,
                        rotate_radiu, is_init, para_.lat, para_.lon);
                  if (next_pose.theta < 0) {
                    next_pose = Pose2D(turning_pose.x - to_cover_offset_length / 2.0,
                                        turning_pose.y - offset_dis / 2.0,
                                        0.0);
                    std::vector<Pose2D> tmp_points;
                    if (is_forward_point) {
                      tmp_points.push_back(straight_pose);
                    }
                    tmp_points.push_back(turning_pose);
                    tmp_points.push_back(next_pose);
                    points_and_offset.emplace_back(tmp_points, offset_dis);
                    is_backward_success = true;
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  if (points_and_offset.empty()) {
    LOCAL_LOG(LOCAL_INFO, "planInSlotOffset points_and_offset empty");
    return false;
  }

  std::sort(points_and_offset.begin(), points_and_offset.end(), 
            [](PointsAndOffset a, PointsAndOffset b) {return a.second > b.second;});
  std::vector<Pose2D> local_offset_key_points = points_and_offset.front().first;
  local_planning_key_points.insert(local_planning_key_points.end(),
                                    local_offset_key_points.begin(),
                                    local_offset_key_points.end());
  return true;
}

void ParallelRulePlanner::getStartPoseAndDirect(const std::vector<Pose2D> &local_planning_key_points, 
                                                const double min_step_size, const Pose2D &pose, 
                                                int &path_block_direc, double &added_dis) {
  int len_key_points = local_planning_key_points.size();
  int block_direct = 0; // 0:都可以, 1:不能向前, -1:不能向后
  double pose_dis = 0.0;

  if (len_key_points == 2) {
    Pose2D first_pose = local_planning_key_points.front();
    Pose2D second_pose = local_planning_key_points.back();
    double tmp_dis = calcPose2DDistance(first_pose, second_pose);
    if (tmp_dis > min_step_size) {
      block_direct = 0;
      pose_dis = 0.0;
    } else {
      block_direct = second_pose.x - first_pose.x > 0? -1 : 1;
      pose_dis = tmp_dis;
    }
  } 

  if (len_key_points > 2) {
    int turn_pose_index = 0;
    bool has_turn_pose = false;
    for (int i = len_key_points - 1;i > 1;i--) {
      Pose2D first_pose = local_planning_key_points[i];
      Pose2D second_pose = local_planning_key_points[i - 1];
      Pose2D third_pose = local_planning_key_points[i - 2];
      int direct_1 = second_pose.x - first_pose.x >= 0 ? 1 : -1;
      int direct_2 = third_pose.x - second_pose.x >= 0 ? 1 : -1;
      if (direct_1 * direct_2 < 0) {
        turn_pose_index = i - 1;
        has_turn_pose = true;
        break;
      }
    }

    if (has_turn_pose) {
      Pose2D first_pose = local_planning_key_points[turn_pose_index];
      Pose2D second_pose = local_planning_key_points.back();  
      double tmp_dis = calcPose2DDistance(first_pose, second_pose);
      
      if (tmp_dis > min_step_size) {
        block_direct = 0;
        pose_dis = 0.0;
      } else {
        block_direct = second_pose.x - first_pose.x > 0? -1 : 1;
        pose_dis = tmp_dis;
      }
    } else {
      Pose2D first_pose = local_planning_key_points.front();
      Pose2D second_pose = local_planning_key_points.back();
      double tmp_dis = calcPose2DDistance(first_pose, second_pose);
      if (tmp_dis > min_step_size) {
        block_direct = 0;
        pose_dis = 0.0;
      } else {
        block_direct = second_pose.x - first_pose.x > 0? -1 : 1;
        pose_dis = tmp_dis;
      }
    }
  }

  path_block_direc = block_direct;
  added_dis = pose_dis;
}

double ParallelRulePlanner::calcPose2DDistance(const Pose2D &pose1, const Pose2D &pose2) {
  double deta_x = pose1.x - pose2.x;
  double deta_y = pose1.y - pose2.y;
  double dis = std::sqrt(deta_x * deta_x + deta_y * deta_y);
  return dis;
}

bool ParallelRulePlanner::isInHalfSlot(const Pose2D &pose) {
  std::vector<planning_math::Vec2d> corners(6);

  csg_.getRawShape(pose, corners, para_.lat, para_.lon);
  if (pose.y < impl_.slot_height && corners.at(5).y() > impl_.slot_height) {
    return true;
  }

  return false;
}

bool ParallelRulePlanner::isInSlot(const Pose2D &pose) {
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

void ParallelRulePlanner::Update(
    const planning_math::Box2d map_boundary,
    const std::vector<planning_math::LineSegment2d> &map) {}

SbpResult ParallelRulePlanner::getResult() { return result_; }

void ParallelRulePlanner::initScenario(
    const parking::OpenspaceDeciderOutput &input) {
  // get local frame
  local_frame_pose_ =
      Pose2D(input.init_state.path_point.x, input.init_state.path_point.y,
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

  msquare::planning_math::Vec2d local_center = msquare::planning_math::tf2d(
      local_frame_pose_, input.map_boundary.center());
  impl_.is_on_left = (local_center.y() < 0.0 ? true : false);

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
  }

  addSideVirtualWall(obs_pts_);
  refactorRightCorner(obs_pts_);

  impl_.slot_height = std::max(impl_.right_corner.y(), impl_.half_width);

  auto local_upper_line =
      planning_math::tf2d(local_frame_pose_, input.T_lines.road_upper_bound);
  impl_.upper_height = local_upper_line.min_y();
  LOCAL_LOG(LOCAL_INFO, "slot height:%.3f", impl_.slot_height);

  csg_ = clothoid::CollisionShapeGenerator(para_);
  checker_ = clothoid::CollisionChecker(csg_);
  checker_.setData(obs_lines_, obs_pts_, step_lines_, step_pts_);
}

void ParallelRulePlanner::refactorRightCorner(
    std::vector<planning_math::Vec2d> &obs_pts) {
  double nx = impl_.right_corner.x();
  double ny = impl_.right_corner.y();
  LOCAL_LOG(LOCAL_INFO, "before refactor right corner x: %.3f y: %.3f", nx, ny);

  // check if is_front_empty_
  is_front_empty_ = true;
  double min_y2 = -0.5 * para_.width;
  double max_y2 = 1.5 * para_.width;
  double min_x2 = para_.front_to_rear;
  double max_x2 = para_.front_to_rear + para_.check_empty_length;
  double px, py;
  for (auto &p : obs_pts) {
    px = p.x();
    py = p.y();
    if (px < min_x2 || px > max_x2 || py < min_y2 || py > max_y2) {
      continue;
    }

    is_front_empty_ = false;
    break;
  }

  double min_y = 0.0;
  double max_y = 1.5 * para_.width;
  double min_x = para_.front_to_rear;
  double max_x = std::max(nx + 1.0, para_.front_to_rear + 3.0);
  if(is_front_empty_ && max_x2 > max_x){
    max_x = max_x2;
    nx = max_x;
    ny = min_y;
  }

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

  LOCAL_LOG(LOCAL_INFO, "after refactor right corner x: %.3f y: %.3f", nx, ny);

  impl_.right_corner.set_x(nx);
  impl_.right_corner.set_y(ny);
}

bool ParallelRulePlanner::checkStartEndPose() {
  if (!checker_.checkTerminalPose(init_pose_)) {
    result_.status = SbpStatus::START_INFEASIBLE;
    return false;
  }
  if (!checker_.checkTerminalPose(target_pose_)) {
    result_.status = SbpStatus::END_INFEASIBLE;
    return false;
  }

  return true;
}

void ParallelRulePlanner::addSideVirtualWall(
    std::vector<planning_math::Vec2d> &obs_pts) {
  double upper_offset = 1.0;
  double real_width = para_.width + upper_offset;
  double real_length = para_.length;
  planning_math::Box2d ego_box(
      {para_.rear_to_center, target_pose_.y + upper_offset / 2.0},
      target_pose_.theta, real_length, real_width);
  planning_math::Vec2d left_shift(-real_length, 0);
  planning_math::Vec2d right_shift(real_length, 0);

  planning_math::Box2d left_box(ego_box);
  left_box.Shift(left_shift);
  planning_math::Box2d right_box(ego_box);
  right_box.Shift(right_shift);

  planning_math::Vec2d left_key, right_key;
  bool left_is_empty = true;
  bool right_is_empty = true;
  for (auto &op : obs_pts) {
    if (left_box.IsPointIn(op)) {
      if (left_is_empty) {
        left_is_empty = false;
        left_key = op;
      } else {
        if (op.x() > left_key.x()) {
          left_key = op;
        }
      }
    } // end left

    if (right_box.IsPointIn(op)) {
      if (right_is_empty) {
        right_is_empty = false;
        right_key = op;
      } else {
        if (op.x() < right_key.x()) {
          right_key = op;
        }
      }
    } // end right
  }   // end for

  if (!right_is_empty && right_key.y() > 0.0) {
    int steps = std::floor(right_key.y() / para_.virtual_wall_interval) + 1;
    double step_length = right_key.y() / steps;
    for (int i = 0; i <= steps; i++) {
      obs_pts.emplace_back(right_key.x(), i * step_length);
    }
  }
  if (!left_is_empty && left_key.y() > 0.0) {
    int steps = std::floor(left_key.y() / para_.virtual_wall_interval) + 1;
    double step_length = left_key.y() / steps;
    for (int i = 0; i <= steps; i++) {
      obs_pts.emplace_back(left_key.x(), i * step_length);
    }
  }
}

void ParallelRulePlanner::getGlobalPath(std::vector<Pose2D> &global_path,
                                        const std::vector<Pose2D> &local_path,
                                        const Pose2D &local_frame) {
  for (auto &p : local_path) {
    global_path.push_back(planning_math::tf2d_inv(local_frame, p));
  } // end for
}

std::vector<Pose2D> ParallelRulePlanner::getSearchPoints() {
  return key_points_;
}

} // namespace msquare