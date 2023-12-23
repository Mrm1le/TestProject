
#include "planner/behavior_planner/general_motion_planner_preprocessor.h"

using namespace gmp_interface;
using namespace msquare::planning_math;

namespace msquare {

void GeneralMotionPlannerPreprocessor::process_time_manager() {
  if (call_count_ == 1) {
    gmp_prec_created_time_ = MTIME()->timestamp().sec();
  }
  t_buffer_.front() = t_buffer_.back();
  t_buffer_.back() = MTIME()->timestamp().sec();

  double t_delta_tmp = t_buffer_.back() - t_buffer_.front();
  if (abs(t_delta_tmp - 0.1) > 0.2) {
    t_delta_ = 0.1;
  } else {
    t_delta_ = t_delta_tmp;
  }
  MSD_LOG(INFO, "GMP_t_delta: %.3f s", t_delta_);
}

void GeneralMotionPlannerPreprocessor::generate_time_array() {
  for (int i = 0; i < time_array_index_max_; ++i) {
    time_array_.emplace_back(0.1 * (i + 1));
  }
}

double
GeneralMotionPlannerPreprocessor::cal_refline_y(const vector<double> &c_poly,
                                                const double &x_max, double x,
                                                int dev_order) {
  x = min(x, x_max);
  x = max(x, 0.0);
  double c3 = c_poly[3];
  double c4 = c_poly[4];
  double c5 = c_poly[5];
  double y = 0.0;

  if (!(dev_order == 1 || dev_order == 2 || dev_order == 0)) {
    return y;
  }

  switch (dev_order) {
  case 0:
    y = c3 * pow(x, 3) + c4 * pow(x, 4) + c5 * pow(x, 5);
    break;
  case 1:
    y = 3 * c3 * x * x + 4 * c4 * pow(x, 3) + 5 * c5 * pow(x, 4);
    break;
  case 2:
    y = 6 * c3 * x + 12 * c4 * pow(x, 2) + 20 * c5 * pow(x, 3);
    break;
  default:
    break;
  }
  return y;
}

vector<double>
GeneralMotionPlannerPreprocessor::cal_lc_refline_quinyic(double s, double l) {
  double x_end = s;
  double y_end = l;
  Eigen::MatrixXd Y(3, 1);
  Eigen::MatrixXd X(3, 3);

  vector<double> c_poly(6, 0.0);

  Y << y_end, 0.0, 0.0;

  X << pow(x_end, 3), pow(x_end, 4), pow(x_end, 5), 3 * x_end * x_end,
      4 * pow(x_end, 3), 5 * pow(x_end, 4), 6 * x_end, 12 * pow(x_end, 2),
      20 * pow(x_end, 3);

  Eigen::MatrixXd Cs(3, 1);
  Cs = X.inverse() * Y;

  c_poly[3] = Cs(0, 0);
  c_poly[4] = Cs(1, 0);
  c_poly[5] = Cs(2, 0);

  return c_poly;
}

void GeneralMotionPlannerPreprocessor::extend_frenet(
    const Point2D &frenet, gmp_interface::Point2d &local, double &heading) {
  using gmp_interface::Point2d;

  double veh_s0 = baseline_info_->get_ego_state().planning_start_state.s;
  double veh_heading =
      general_motion_planner_input_->ego_state_cart.ego_pose.theta;
  double refline_end_s = frenet_coord_->GetLength();
  double refline_start_s = 0.1;
  double refline_end_l = 0.0;
  double refline_s_max = refline_end_s - veh_s0;
  double s_extend = frenet.x - refline_end_s;
  double l_extend = frenet.y;

  double refline_end_heading = 0.0;
  double refline_start_heading = 0.0;

  MSD_LOG(INFO, "GMP Frenet end: heading: %.4f, len: %.2f", refline_end_heading,
          refline_s_max);

  Point2D refline_end_frenet{refline_end_s, refline_end_l};
  Point2D refline_start_frenet{refline_start_s, refline_end_l};

  Point2D refline_end_cart{0.0, 0.0};
  Point2D refline_start_cart{0.0, 0.0};

  if (frenet.x < refline_start_s) {
    if (frenet_coord_->FrenetCoord2CartCoord(
            refline_start_frenet, refline_start_cart) == TRANSFORM_SUCCESS) {

      refline_start_heading =
          frenet_coord_->GetRefCurveHeading(refline_start_frenet.x) -
          veh_heading;
      Point2d refline_start_env{refline_start_cart.x, refline_start_cart.y};
      Point2d refline_start_local = env2local_pos(refline_start_env);

      s_extend = frenet.x - refline_start_s; // negative
      l_extend = frenet.y;

      local.y = refline_start_local.y + s_extend * sin(refline_start_heading) +
                l_extend * cos(refline_start_heading);
      local.x = refline_start_local.x + s_extend * cos(refline_start_heading) +
                l_extend * sin(refline_start_heading);
    } else {
      return;
    }

  } else {
    if (frenet_coord_->FrenetCoord2CartCoord(
            refline_end_frenet, refline_end_cart) == TRANSFORM_SUCCESS) {
      refline_end_heading =
          frenet_coord_->GetRefCurveHeading(refline_end_frenet.x) - veh_heading;

      Point2d refline_end_env{refline_end_cart.x, refline_end_cart.y};
      Point2d refline_end_local = env2local_pos(refline_end_env);

      local.y = refline_end_local.y + s_extend * sin(refline_end_heading) +
                l_extend * cos(refline_end_heading);
      local.x = refline_end_local.x + s_extend * cos(refline_end_heading) -
                l_extend * sin(refline_end_heading);
    } else {
      return;
    }
  }
}

void GeneralMotionPlannerPreprocessor::update_target_point(
    gmp_interface::BehaviorCandidate &behavior_candidate) {
  const double veh_set_speed =
      general_motion_planner_input_->ego_state_cart.ego_v_cruise;
  const double veh_speed = general_motion_planner_input_->ego_state.ego_vel;
  vector<double> target_speeds = {veh_set_speed,
                                  veh_speed,
                                  veh_set_speed,
                                  veh_speed,
                                  min(veh_set_speed + 2.70, veh_speed + 3.0),
                                  target_speed_lc_wait_};
  const double target_speed = target_speeds[behavior_candidate.behavior_index];

  if (behavior_candidate.behavior_type == LC_Dir::NONE) {
    // lk
    update_target_point_LH(behavior_candidate, target_speed);
  } else if (behavior_candidate.behavior_index ==
             static_cast<int>(lc_t_total_candidates_.size()) - 1) {
    // lc_wait
    update_target_point_LH(behavior_candidate, target_speed);
  } else {
    // lc proceed
    update_target_point_LC(behavior_candidate, target_speed);
  }
}

void GeneralMotionPlannerPreprocessor::update_target_point_LH(
    gmp_interface::BehaviorCandidate &behavior_candidate,
    const double &target_speed) {
  using gmp_interface::Point2d;
  using gmp_interface::TargetPoint;
  double veh_speed = general_motion_planner_input_->ego_state.ego_vel; // m/s
  double veh_heading =
      general_motion_planner_input_->ego_state_cart.ego_pose.theta;
  double veh_set_speed =
      general_motion_planner_input_->ego_state_cart.ego_v_cruise;

  double detaT = std::max(abs(ego_l0_) / 0.3, 2.0);
  double lh_near_s = saturation(detaT * 20, 25.0, 60.0);
  double veh_s0 = baseline_info_->get_ego_state().planning_start_state.s;
  double refline_s_max = frenet_coord_->GetLength() - veh_s0;
  double pp_end_s = 250.0;
  vector<double> ss{0.5 * lh_near_s, lh_near_s, refline_s_max, pp_end_s};

  auto c_poly = cal_lc_refline_quinyic(lh_near_s, -ego_l0_);
  vector<pair<double, double>> key_points{};
  for (auto s : ss) {
    key_points.emplace_back(s, cal_refline_y(c_poly, lh_near_s, s, 0));
  }
  sort(key_points.begin(), key_points.end(),
       [](const auto &a, const auto &b) { return a.first < b.first; });

  behavior_candidate.target_points.clear();
  double temp = 0.0;
  for (const auto &point : key_points) {
    if (abs(point.first - temp) <= 5) {
      continue;
    }
    temp = point.first;
    Point2D frenet_point{point.first + veh_s0, point.second + ego_l0_};
    Point2D cart_point{0.0, 0.0};
    double frenet_heading = 0.0;
    double refline_heading = cal_refline_y(c_poly, lh_near_s, point.first, 1);
    Point2d local_point{0.0, 0.0};

    if (frenet_coord_->FrenetCoord2CartCoord(frenet_point, cart_point) ==
        TRANSFORM_SUCCESS) {
      frenet_heading =
          frenet_coord_->GetRefCurveHeading(frenet_point.x) - veh_heading;
      Point2d lh_end_env{cart_point.x, cart_point.y};
      local_point = env2local_pos(lh_end_env);
      MSD_LOG(INFO, "Behavior candidate Heading %.4f, %.4f, %.4f",
              frenet_heading, veh_heading, refline_heading);
      MSD_LOG(INFO, "Behavior candidate targetpoint frenet succeed!");
    } else {
      extend_frenet(frenet_point, local_point, frenet_heading);
      MSD_LOG(INFO, "Behavior candidate Heading %.4f, %.4f, %.4f",
              frenet_heading, veh_heading, refline_heading);
      MSD_LOG(INFO, "Behavior candidate targetpoint extend succeed!");
    }
    TargetPoint target_point;
    target_point.x = local_point.x;
    target_point.y = local_point.y;
    target_point.heading = refline_heading + tan(frenet_heading);
    target_point.vel = target_speed;
    target_point.behavior_type = behavior_candidate.behavior_type;
    MSD_LOG(INFO, "Behavior candidate %.2f %.2f m %.1f", target_point.x,
            target_point.y, target_point.vel);
    behavior_candidate.target_points.push_back(target_point);
  }
}

void GeneralMotionPlannerPreprocessor::update_target_point_LC(
    gmp_interface::BehaviorCandidate &behavior_candidate,
    const double &target_speed) {
  using gmp_interface::Point2d;
  using gmp_interface::TargetPoint;
  vector<TargetPoint> target_points_LC(4, target_point_default_);
  auto lc_dir = behavior_candidate.behavior_type;
  double lc_dist_total = behavior_candidate.behavior_info.lc_dist_total;
  double lc_dist_pass = behavior_candidate.behavior_info.lc_dist_pass;
  behavior_candidate.target_points.clear();

  if (lc_dir == LC_Dir::NONE) {
    behavior_candidate.target_points = target_points_LC;
    return;
  }

  double veh_speed = general_motion_planner_input_->ego_state.ego_vel;
  double veh_heading =
      general_motion_planner_input_->ego_state_cart.ego_pose.theta; // global
  double veh_set_speed =
      general_motion_planner_input_->ego_state_cart.ego_v_cruise;
  double veh_s0 = baseline_info_->get_ego_state().planning_start_state.s;
  // ---- seg_endpoint_s, veh pos = 0m. all endpoint_s will be pushed into ss

  double lc_near_s = 40;
  double refline_s_max = frenet_coord_->GetLength() - veh_s0;
  double pp_end_s = 250.0;

  vector<double> ss{lc_near_s + lc_dist_pass, lc_dist_total,
                    refline_s_max + lc_dist_pass, pp_end_s + lc_dist_pass};

  MSD_LOG(INFO, "Behavior candidate[%.2fs] %.2f %.2f m,%.2f %.2f m",
          MTIME()->timestamp().sec() - gmp_prec_created_time_,
          lc_near_s + lc_dist_pass, lc_dist_total, refline_s_max + lc_dist_pass,
          pp_end_s + lc_dist_pass);

  if (lc_dist_total - lc_dist_pass < 10.0) {
    behavior_candidate.target_points = target_points_LC;
    return;
  }

  double lc_end_l = (lc_dir == LC_Dir::LEFT) ? lane_width_ : -lane_width_;
  auto c_poly = cal_lc_refline_quinyic(lc_dist_total, lc_end_l);
  double refline_bias = lane_cross_ ? lc_end_l : 0;

  vector<pair<double, double>> key_points{};
  for (double s : ss) {
    key_points.emplace_back(s - lc_dist_pass,
                            cal_refline_y(c_poly, lc_dist_total, s, 0));
  }

  sort(key_points.begin(), key_points.end(),
       [](const auto &a, const auto &b) { return a.first < b.first; });
  int i = 0;
  double temp = 0.0;
  for (const auto &point : key_points) {
    if (abs(point.first - temp) <= 5) {
      continue;
    }
    temp = point.first;
    Point2D lc_end_frenet{point.first + veh_s0, point.second - refline_bias};
    Point2D lc_end_cart{0.0, 0.0};
    double frenet_heading = 0.0;
    double lc_refline_heading =
        cal_refline_y(c_poly, lc_dist_total, point.first + lc_dist_pass, 1);
    Point2d lc_end_local{0.0, 0.0};

    if (frenet_coord_->FrenetCoord2CartCoord(lc_end_frenet, lc_end_cart) ==
        TRANSFORM_SUCCESS) {

      frenet_heading =
          frenet_coord_->GetRefCurveHeading(lc_end_frenet.x) - veh_heading;
      Point2d lc_end_env{lc_end_cart.x, lc_end_cart.y};
      lc_end_local = env2local_pos(lc_end_env);
      MSD_LOG(INFO, "LALALA targetpoint frenet succeed!");

    } else {
      extend_frenet(lc_end_frenet, lc_end_local, frenet_heading);
      MSD_LOG(INFO, "LALALA targetpoint extend succeed!");
    }
    target_points_LC[i].x = lc_end_local.x;
    target_points_LC[i].y = lc_end_local.y;
    target_points_LC[i].heading = lc_refline_heading + tan(frenet_heading);
    target_points_LC[i].vel = target_speed;
    target_points_LC[i].behavior_type = lc_dir;
    MSD_LOG(INFO, "Behavior candidate[%.2fs] %.2f %.2f m",
            MTIME()->timestamp().sec() - gmp_prec_created_time_,
            target_points_LC[i].x, target_points_LC[i].y);
    i++;
  }
  behavior_candidate.target_points.clear();
  behavior_candidate.target_points = target_points_LC;
}

void GeneralMotionPlannerPreprocessor::set_target_point_speed(
    vector<gmp_interface::TargetPoint> &target_points, const double &tp_speed) {
  if (target_points.empty()) {
    return;
  }
  for (auto &target_point : target_points) {
    target_point.vel = tp_speed;
  }
}

void GeneralMotionPlannerPreprocessor::set_target_point_type(
    vector<gmp_interface::TargetPoint> &target_points, const int &tp_type) {
  if (target_points.empty()) {
    return;
  }
  for (auto &target_point : target_points) {
    target_point.behavior_type = tp_type;
  }
}

void GeneralMotionPlannerPreprocessor::generate_target_points(void) {
  using gmp_interface::TargetPoint;

  general_motion_planner_output_->target_points.clear();
  general_motion_planner_input_->behavior_candidates.clear();
  generate_behavior_candidates();
  for (const auto &behavior_candidate : behavior_candidates_) {
    general_motion_planner_output_->target_points.emplace_back(
        behavior_candidate.target_points);
  }
  general_motion_planner_input_->behavior_candidates = behavior_candidates_;
}

void GeneralMotionPlannerPreprocessor::generate_behavior_candidates(void) {
  behavior_candidates_.clear();
  const int behavior_index_max =
      (lc_state_ == LC_Dir::NONE)
          ? 2
          : static_cast<int>(lc_t_total_candidates_.size());
  for (int i = 0; i < behavior_index_max; ++i) {
    gmp_interface::BehaviorCandidate behavior_candidate;
    behavior_candidate.behavior_index = i;
    behavior_candidate.behavior_type = lc_state_;
    generate_behavior_info(behavior_candidate);
    update_equivalent_behavior_info(behavior_candidate);
    update_target_point(behavior_candidate);
    MSD_LOG(INFO, "GMP lc_to_static_obj behavior[%.2f]: cand_num[%d]",
            MTIME()->timestamp().sec() - gmp_prec_created_time_,
            behavior_candidates_.size(),
            behavior_candidate.behavior_info.lc_time_total);
    behavior_candidates_.emplace_back(behavior_candidate);
  }

  return;
}

void GeneralMotionPlannerPreprocessor::generate_behavior_info(
    gmp_interface::BehaviorCandidate &behavior_candidate) {
  double lc_time_coef = 1.0;
  if (ego_speed_lc_trigger_ <= 10.0) {
    lc_time_coef = 0.75;
  } else if (ego_speed_lc_trigger_ >= 25.0) {
    lc_time_coef = 1.0;
  } else {
    lc_time_coef =
        (1.0 - 0.75) / (25.0 - 10.0) * (ego_speed_lc_trigger_ - 10.0) + 0.75;
  }

  double lc_close2cipv_coef = 1.0;
  double lc_proceed_time = general_motion_planner_output_->lc_in_proceed_time;
  if (lc_proceed_time > 0.0) {
    lc_close2cipv_coef = general_motion_planner_input_->gmp_suggest_lc_time /
                         lc_t_total_candidates_.front();
    lc_close2cipv_coef = saturation(lc_close2cipv_coef, 0.60, 1.0);
  }

  const int index = behavior_candidate.behavior_index;
  double lc_time_total = 0.0;

  if (general_motion_planner_output_->lc_to_static_obj_time > 0.01) {
    lc_time_total = general_motion_planner_output_->lc_to_static_obj_time;
  } else {
    lc_time_total =
        lc_t_total_candidates_[index] * lc_time_coef * 0.9 * lc_close2cipv_coef;
    MSD_LOG(INFO,
            "GMP lc_to_static_obj lc_time_total[%.2f]: "
            "cand_num[%d],%.2f,%.2f",
            MTIME()->timestamp().sec() - gmp_prec_created_time_,
            behavior_candidates_.size(), lc_time_total,
            general_motion_planner_output_->lc_to_static_obj_time);
  }
  double lc_dist_total = lc_time_total * ego_speed_lc_trigger_;
  double lc_time_pass = 0.0;
  double lc_dist_pass = 0.0;
  double lc_time_remain = lc_time_total;
  double lc_dist_remain = lc_dist_total;

  DrivingTaskInfo behavior_info{
      lc_time_pass,         lc_time_remain, lc_dist_remain,
      lc_dist_pass,         lc_time_total,  lc_dist_total,
      lc_latdist_abs_past_, lc_wait_time_,  lane_cross_};

  behavior_candidate.behavior_info = behavior_info;
  general_motion_planner_input_->one_lc_time = lc_time_total;

  return;
}

void GeneralMotionPlannerPreprocessor::update_equivalent_behavior_info(
    gmp_interface::BehaviorCandidate &behavior_candidate) {

  double lat_dist_pass_abs = lc_latdist_abs_past_;
  auto task_type = behavior_candidate.behavior_type;
  int chosen_behavior_index =
      general_motion_planner_output_->chosen_behavior_candidate_index;
  bool lc_in_waiting =
      (general_motion_planner_output_->lc_in_proceed_time <= 0.0) ? 1 : 0;

  if (task_type == LC_Dir::NONE) {
    return;
  }
  bool lc_triggered = lc_request_buffer_.front() == LC_Dir::NONE &&
                      lc_request_buffer_.back() != LC_Dir::NONE;

  MSD_LOG(INFO, "GMP chosen behavior at[%.2f] is [%d]",
          MTIME()->timestamp().sec() - gmp_prec_created_time_,
          chosen_behavior_index);

  if (!lc_triggered &&
      behavior_candidate.behavior_index == chosen_behavior_index &&
      behavior_candidate.behavior_index != lc_t_total_candidates_.size() - 1) {
    // for the chosen lc candidate, use real task_info
    behavior_candidate.behavior_info = lc_task_info_;
    return;
  }

  double lc_end_s = behavior_candidate.behavior_info.lc_dist_total;
  double lc_end_l = (task_type == LC_Dir::LEFT) ? lane_width_ : -lane_width_;
  vector<double> c_poly = behavior_candidate.c_poly;
  if (c_poly.empty()) {
    c_poly = cal_lc_refline_quinyic(lc_end_s, lc_end_l);
  }

  double left = 0.0;
  double right = lc_end_s;
  double mid = 0.0;
  double lat_dist_pass_abs_mid = 0.0;
  double dist_error_abs = abs(lat_dist_pass_abs_mid - lat_dist_pass_abs);
  double error_thres = 0.1;

  if (lat_dist_pass_abs >= abs(lc_end_l)) {
    mid = lc_end_s;
  } else if (lat_dist_pass_abs <= 0.4 || lc_in_waiting) {
    mid = 0.0;
  } else {
    int ii = 0;
    while (dist_error_abs > error_thres && left <= right) {
      MSD_LOG(INFO, "GMP bsearch[%.1f][%d]",
              MTIME()->timestamp().sec() - gmp_prec_created_time_, ii);
      mid = 0.5 * (left + right);
      lat_dist_pass_abs_mid = abs(cal_refline_y(c_poly, lc_end_s, mid, 0));
      dist_error_abs = abs(lat_dist_pass_abs_mid - lat_dist_pass_abs);
      if (dist_error_abs <= error_thres) {
        break;
      }
      if (lat_dist_pass_abs_mid < lat_dist_pass_abs) {
        left = mid;
      } else {
        right = mid;
      }
      ii++;
    }
  }
  if (mid <= 0.01) {
    behavior_candidate.behavior_info.lc_dist_pass = lc_task_info_.lc_dist_pass;
    behavior_candidate.behavior_info.lc_time_pass = lc_task_info_.lc_time_pass;
  } else {
    behavior_candidate.behavior_info.lc_dist_pass = mid;
    behavior_candidate.behavior_info.lc_time_pass =
        (mid / behavior_candidate.behavior_info.lc_dist_total) *
        behavior_candidate.behavior_info.lc_time_total;
  }
  behavior_candidate.behavior_info.lc_dist_remain =
      behavior_candidate.behavior_info.lc_dist_total -
      behavior_candidate.behavior_info.lc_dist_pass;
  behavior_candidate.behavior_info.lc_time_remain =
      behavior_candidate.behavior_info.lc_time_total -
      behavior_candidate.behavior_info.lc_time_pass;
  behavior_candidate.behavior_info.lc_latdist_abs_past = lat_dist_pass_abs_mid;
  behavior_candidate.behavior_info.lc_time_wait = lc_wait_time_;
  behavior_candidate.behavior_info.lane_cross = lane_cross_;

  return;
}

void GeneralMotionPlannerPreprocessor::is_lane_crossed() {
  if (!lane_cross_ && abs(ego_l0_buffer_[2] - ego_l0_buffer_[1]) > 1.5) {
    lane_cross_ = true;
  } else if (lane_cross_ && lc_state_ == LC_Dir::NONE) {
    lane_cross_ = false;
  }

  MSD_LOG(INFO, "GMP_lane_crossed: %.2fs, %d",
          MTIME()->timestamp().sec() - gmp_prec_created_time_, lane_cross_);
}

void GeneralMotionPlannerPreprocessor::process() {
  update_call_count();
  process_time_manager();

  double total_time_offset =
      ConfigurationContext::Instance()
          ->planner_config()
          .lateral_behavior_planner_config.lc_total_time_offset;

  double total_time_offset_1 =
      ConfigurationContext::Instance()
          ->planner_config()
          .lateral_behavior_planner_config.lc_total_time_offset_1;

  double total_time_offset_2 =
      ConfigurationContext::Instance()
          ->planner_config()
          .lateral_behavior_planner_config.lc_total_time_offset_2;

  double total_time_offset_3 =
      ConfigurationContext::Instance()
          ->planner_config()
          .lateral_behavior_planner_config.lc_total_time_offset_3;

  lc_t_total_candidates_ = {7.0, 7.0, 10.0, 10.0, 7.0, 7.0};
  if (world_model_->get_driving_model_config() == SAFE_DRIVING_MODEL) {
    total_time_offset = total_time_offset_1;
  } else if (world_model_->get_driving_model_config() == STEADY_DRIVING_MODEL) {
    total_time_offset = total_time_offset_2;
  } else if (world_model_->get_driving_model_config() ==
             RADICAL_DRIVING_MODEL) {
    total_time_offset = total_time_offset_3;
  }

  for (int i = 0; i < static_cast<int>(lc_t_total_candidates_.size()); i++) {
    lc_t_total_candidates_[i] += total_time_offset;
  }

  if (time_array_.empty()) {
    generate_time_array();
  }

  frenet_coord_ = baseline_info_->get_frenet_coord();
  enu2car_ = world_model_->get_cart_ego_state_manager().get_enu2car();

  update_gmp_input_ego_state(); // egostate, lc_state_ (highlevel_guidance)

  update_lane_info();

  update_gmp_input_task_info();

  gmp_obj_infos_ = generate_gmp_object_info();

  update_concerned_objs();

  update_gmp_input_obj();

  generate_target_points();

  update_gmp_output();
  last_lc_to_static_obj_time_ =
      general_motion_planner_output_->lc_to_static_obj_time;
  general_motion_planner_input_->total_time_offset = total_time_offset;
  general_motion_planner_input_->driving_model_config =
      world_model_->get_driving_model_config();
  general_motion_planner_input_->lc_duration = lc_t_total_candidates_[5];
}

void GeneralMotionPlannerPreprocessor::update_gmp_input_task_info() {
  using gmp_interface::DrivingTaskInfo;

  DrivingTaskInfo task_info{
      lc_time_pass_,        lc_time_remain_, lc_dist_remain_,
      lc_dist_pass_,        lc_time_total_,  lc_dist_total_,
      lc_latdist_abs_past_, lc_wait_time_,   lane_cross_};
  bool lc_triggered = lc_request_buffer_.front() == LC_Dir::NONE &&
                      lc_request_buffer_.back() != LC_Dir::NONE;

  if (lc_state_ != LC_Dir::NONE && !lc_triggered) {
    MSD_LOG(INFO,
            "GMP lc_to_static_obj task info[%.2fs], %.1fm, %.1fm, %.1fm,%.1f, "
            "%.1f, %.1f",
            MTIME()->timestamp().sec() - gmp_prec_created_time_,
            lc_dist_remain_, lc_dist_pass_, lc_dist_total_, lc_time_pass_,
            lc_time_remain_, lc_time_total_);
  }

  general_motion_planner_input_->lc_task_info = task_info;
  lc_task_info_ = task_info;
}

double GeneralMotionPlannerPreprocessor::smooth_headaway(
    const double &headaway_current, const double &headaway_target) {
  double smooth_coef = 0.05;
  double error = headaway_target - headaway_current;
  double headaway_filtered = headaway_target;
  if (error <= 0.09) {
    return headaway_filtered;
  } else {
    headaway_filtered = headaway_current + smooth_coef * error;
  }
  return headaway_filtered;
}

void GeneralMotionPlannerPreprocessor::update_concerned_objs() {
  using gmp_interface::ObjPredSlice;
  using gmp_interface::ObsInfo;
  using gmp_interface::Point2d;

  double lateral_interval = 0.0;
  concerned_obj_infos_.clear();

  vector<pair<double, double>> concern_zone_lat(3, make_pair(0.0, 0.0));
  vector<pair<double, double>> concern_zone_longi(3, make_pair(0.0, 0.0));
  const double &ego_speed =
      general_motion_planner_input_->ego_state_cart.ego_vel;

  Point2d ego_pos_cart{
      general_motion_planner_input_->ego_state_cart.ego_pose.x,
      general_motion_planner_input_->ego_state_cart.ego_pose.y};

  const double &ego_s0 = get_ego_sl0(ego_pos_cart).x;

  double lc_latdist_past = min(lc_latdist_abs_past_, 3.5);
  double lc_lat_dist_thre = 3.2; // threshold to release the objs behind
  double lc_front_lat_thres = 0.5 * real_lane_width_ + 4.7;
  double lc_rear_lat_thres = 5.7;

  if (lc_state_ == LC_Dir::NONE) {
    concern_zone_lat[0] = make_pair(-2.5, 2.5);
    concern_zone_longi[0] = make_pair(2.5, 130.0);
  } else if (lc_state_ == LC_Dir::RIGHT) { // lc_state_ is Task_Type
    if (lc_latdist_past <= lc_lat_dist_thre) {
      concern_zone_lat[0] =
          make_pair(-lc_front_lat_thres + lc_latdist_abs_past_,
                    -1.6 + lc_latdist_abs_past_);
      concern_zone_lat[1] = make_pair(-1.6 + lc_latdist_abs_past_, 1.8);
      concern_zone_lat[2] = make_pair(-lc_rear_lat_thres + lc_latdist_abs_past_,
                                      -1.6 + lc_latdist_abs_past_);
    } else {
      concern_zone_lat[0] =
          make_pair(-lc_front_lat_thres + lc_latdist_abs_past_, -1.2);
      concern_zone_lat[1] = make_pair(-1.2, 1.8);
      concern_zone_lat[2] =
          make_pair(-lc_rear_lat_thres + lc_latdist_abs_past_, -1.2);
    }
    concern_zone_longi[0] = make_pair(-10.0, 130.0);
    concern_zone_longi[1] = make_pair(2.0, 130.0);
    concern_zone_longi[2] = make_pair(-65.0, -10.0);
  } else if (lc_state_ == LC_Dir::LEFT) {
    if (lc_latdist_past <= lc_lat_dist_thre) {
      concern_zone_lat[0] =
          make_pair(1.6 - lc_latdist_abs_past_,
                    lc_front_lat_thres - lc_latdist_abs_past_);
      concern_zone_lat[1] = make_pair(-1.8, 1.6 - lc_latdist_abs_past_);
      concern_zone_lat[2] = make_pair(1.6 - lc_latdist_abs_past_,
                                      lc_rear_lat_thres - lc_latdist_abs_past_);
    } else {
      concern_zone_lat[0] =
          make_pair(1.2, lc_front_lat_thres - lc_latdist_abs_past_);
      concern_zone_lat[1] = make_pair(-1.8, 1.2);
      concern_zone_lat[2] =
          make_pair(1.2, lc_rear_lat_thres - lc_latdist_abs_past_);
    }
    concern_zone_longi[0] = make_pair(-10.0, 130.0);
    concern_zone_longi[1] = make_pair(2.0, 130.0);
    concern_zone_longi[2] = make_pair(-65.0, -10.0);
  } else {
    return;
  }

  double obj_l = 0.0;
  double obj_s = 0.0;

  double obj_pred_l = 0.0;
  double obj_pred_s = 0.0;
  int t_check_pred = 14; // check 1.5s

  double sm_lon_min = 1.5;
  if (lc_state_ != LC_Dir::NONE) {
    sm_lon_min = 10.0;
  }

  for (auto &obj : gmp_obj_infos_) {
    double obj_len_half = 0.5 * obj.obj_size.length;
    double heading_obj = obj.obj_state_init.obj_state_env.heading;
    Point2d obj_frontcenter_cart{obj.obj_state_init.obj_state_env.pos.x +
                                     obj_len_half * cos(heading_obj),
                                 obj.obj_state_init.obj_state_env.pos.y +
                                     obj_len_half *
                                         sin(heading_obj)}; // front center
    Point2d obj_pos_cart{obj.obj_state_init.obj_state_env.pos.x,
                         obj.obj_state_init.obj_state_env.pos.y};

    obj_l = get_ego_sl0(obj_frontcenter_cart).y - ego_l0_;
    obj_l = (obj_l < -80.0) ? obj.obj_state_init.obj_state_local.pos.y : obj_l;

    obj_s = obj.obj_state_init.obj_state_local.pos.x; // TODO: this is
                                                      // temporary!

    double pred_decel = 0.0;
    if (obj_timer_cache_[obj.id] < 0.5 && obj_s < ego_s0) {
      pred_decel = -0.3;
    }

    double sm_lon =
        obj.obj_size.length + max(sm_lon_min, headaway_smoothed_ * ego_speed);

    if (lc_state_ != LC_Dir::NONE) {
      sm_lon = min(sm_lon, 25.0);
    }
    double sm_lat = 0.5 * obj.obj_size.width + 1.4;

    SafetyMargin safety_margin_ini{sm_lon, sm_lat};
    bool obj_init_in_zone =
        is_in_zone(obj_l, obj_s, concern_zone_lat, concern_zone_longi);

    Point2d obj_pos_pred_cart{
        obj.pred_trajectory[t_check_pred].obj_state_env.pos.x,
        obj.pred_trajectory[t_check_pred].obj_state_env.pos.y};

    obj_pred_l = get_ego_sl0(obj_pos_pred_cart).y - ego_l0_;
    obj_pred_s = obj.pred_trajectory[t_check_pred]
                     .obj_state_local.pos.x; // TODO: this is temporary!
    bool obj_pred_in_zone = is_in_zone(obj_pred_l, obj_pred_s, concern_zone_lat,
                                       concern_zone_longi);

    bool rear_obj_in_zone = false;
    if (lc_state_ == LC_Dir::NONE) {
      obj_pred_in_zone = obj_pred_in_zone && obj_s >= 2;
      rear_obj_in_zone = false;
    } else {
      obj_pred_in_zone = obj_pred_in_zone && obj_s >= 2;
      rear_obj_in_zone =
          is_in_target_lane(obj_pred_l, obj_pred_s, concern_zone_lat,
                            concern_zone_longi) &&
          obj_s < -3.0;
    }

    MSD_LOG(
        INFO,
        "GMP input obj GMP pred inzone[%.2fs](%.1f,%.1f)m,(%.1f,%.1f)m: %d %d",
        MTIME()->timestamp().sec() - gmp_prec_created_time_, obj_l, obj_s,
        obj_pred_l, obj_pred_s, obj_init_in_zone, obj_pred_in_zone);

    if (obj_init_in_zone || obj_pred_in_zone || rear_obj_in_zone) {
      if (obj_timer_cache_.find(obj.id) != obj_timer_cache_.end()) {
        obj_timer_cache_[obj.id] += t_delta_;
        obj_timer_cache_[obj.id] = min(obj_timer_cache_[obj.id], 10.0);
      } else {
        obj_timer_cache_[obj.id] = t_delta_;
      }
      obj.noticed_timer = obj_timer_cache_[obj.id];

      // extend prediction trajectory to 10s
      ObjPredSlice obj_pred_single_slice_cache = obj.obj_state_init;
      Point2D obj_base_sl{0.0, 0.0};
      for (int i = 0; i < time_array_.size(); ++i) {
        const double &time = time_array_[i];
        // time-varying safety margin
        SafetyMargin safety_margin_t = safety_margin_ini;
        if (time <= 1.5) {
          safety_margin_t.longitu = obj.obj_size.length + 1.5;
        } else if (time <= 3.5) {
          double temp = obj.obj_size.length +
                        min((headaway_ - dheadaway_), 0.7) * ego_speed;
          safety_margin_t.longitu = min(temp, safety_margin_ini.longitu);
        } else {
          double temp =
              obj.obj_size.length + (headaway_ - dheadaway_) * ego_speed;
          safety_margin_t.longitu = min(temp, safety_margin_ini.longitu);
        }
        // end of time-varying safety margin
        if (time <= pred_valid_time_) {
          obj_pred_single_slice_cache = obj.pred_trajectory[i];
          obj_base_sl = get_obj_base_sl(obj_pred_single_slice_cache);
          double obj_l_temp = obj_base_sl.y;
          if (abs(obj_l_temp) <= lane_width_ / 2.0) {
            obj.pred_trajectory[i].lane_assignment = 1; // current lane
          } else if (obj_l_temp > lane_width_ / 2.0 &&
                     obj_l_temp <= 1.5 * lane_width_ + 0.7) {
            obj.pred_trajectory[i].lane_assignment = 2; // left lane
          } else if (obj_l_temp < -lane_width_ / 2.0 &&
                     obj_l_temp >= -1.5 * lane_width_ - 0.7) {
            obj.pred_trajectory[i].lane_assignment = 3; // right lane
          } else {
            obj.pred_trajectory[i].lane_assignment = -1; // faraway lane
          }
          obj_pred_single_slice_cache = obj.pred_trajectory[i];
        } else {
          obj.pred_trajectory[i] = get_predslice_at_time(
              time, obj_base_sl, obj_pred_single_slice_cache, pred_decel);
          obj.pred_trajectory[i].safety_margin = safety_margin_t;

          if (obj.pred_trajectory[i].lane_assignment == -1 &&
              real_lane_width_ > 3.8) {
            double sm_lat_extend = 0.0;
            if (time <= 5.0) {
              sm_lat_extend = min(4.2 - safety_margin_t.lateral,
                                  0.5 * (real_lane_width_ - 3.8));
            } else {
              sm_lat_extend = min(4.2 - safety_margin_t.lateral,
                                  0.5 * (real_lane_width_ - 2.8));
            }
            sm_lat_extend = max(0.0, sm_lat_extend);
            safety_margin_t.lateral += sm_lat_extend;
          }
          obj.pred_trajectory[i].safety_margin = safety_margin_t;
        }
      }

      // end of predict trajectory extension
      concerned_obj_infos_.emplace_back(obj);
    } else {
      if (obj_timer_cache_.find(obj.id) != obj_timer_cache_.end()) {
        obj_timer_cache_.erase(obj.id);
      }
      obj.noticed_timer = 0;
    }
  }
  auto cmp = [](const ObsInfo &a, const ObsInfo &b) {
    return abs(a.obj_state_init.obj_state_local.pos.x) <
           abs(b.obj_state_init.obj_state_local.pos.x);
  };

  sort(concerned_obj_infos_.begin(), concerned_obj_infos_.end(), cmp);
}

bool GeneralMotionPlannerPreprocessor::is_in_zone(
    const double &x, const double &y,
    const vector<pair<double, double>> &zone_lat,
    const vector<pair<double, double>> &zone_lon) {

  for (int i = 0; i < zone_lat.size(); ++i) {
    if (x >= zone_lat[i].first && x <= zone_lat[i].second &&
        y >= zone_lon[i].first && y <= zone_lon[i].second) {
      return true;
    }
  }
  return false;
}

bool GeneralMotionPlannerPreprocessor::is_in_target_lane(
    const double &x, const double &y,
    const vector<pair<double, double>> &zone_lat,
    const vector<pair<double, double>> &zone_lon) {
  if (x >= zone_lat[0].first && x <= zone_lat[0].second &&
      y >= zone_lon[0].first && y <= zone_lon[0].second) {
    return true;
  } else if (x >= zone_lat[2].first && x <= zone_lat[2].second &&
             y >= zone_lon[2].first && y <= zone_lon[2].second) {
    return true;
  }

  return false;
}

void GeneralMotionPlannerPreprocessor::update_gmp_output() {
  general_motion_planner_output_->t_array = time_array_;
}

void GeneralMotionPlannerPreprocessor::update_gmp_input_obj() {
  general_motion_planner_input_->obj_infos = concerned_obj_infos_;
}

void GeneralMotionPlannerPreprocessor::update_lane_info() {
  bool is_left_line_solid =
      world_model_->get_map_info_manager().is_solid_line(0);
  bool is_right_line_solid =
      world_model_->get_map_info_manager().is_solid_line(1);

  general_motion_planner_input_->is_left_solid = is_left_line_solid;
  general_motion_planner_input_->is_right_solid = is_right_line_solid;
  auto &map_info = world_model_->get_mutable_map_info();

  if (lc_state_ == LC_Dir::LEFT && !lane_cross_) {
    if (!map_info.is_in_intersection() &&
        !map_info.current_refline_points().empty()) {
      for (auto &p : map_info.left_refline_points()) {
        if (p.car_point.x >= 0 && p.car_point.x <= 20 && !p.in_intersection) {
          real_lane_width_ = p.lane_width;
          if (real_lane_width_ < 100) {
            break;
          }
        }
      }
    }
  } else if (lc_state_ == LC_Dir::RIGHT && !lane_cross_) {
    if (!map_info.is_in_intersection() &&
        !map_info.right_refline_points().empty()) {
      for (auto &p : map_info.right_refline_points()) {
        if (p.car_point.x >= 0 && p.car_point.x <= 20 && !p.in_intersection) {
          real_lane_width_ = p.lane_width;
          if (real_lane_width_ < 100) {
            break;
          }
        }
      }
    }
  } else {
    real_lane_width_ = 3.8;
  }

  real_lane_width_ = max(3.8, real_lane_width_);
  real_lane_width_ = min(6.8, real_lane_width_);

  MSD_LOG(INFO, "laneline[%.1f]:%.2f, %.2f",
          MTIME()->timestamp().sec() - gmp_prec_created_time_,
          is_left_line_solid, is_right_line_solid);
};

void GeneralMotionPlannerPreprocessor::update_gmp_input_ego_state() {
  using gmp_interface::Point2d;
  const auto &ego_state_cart = baseline_info_->get_ego_state();
  const double &veh_set_speed = world_model_->get_map_info().v_cruise();
  MSD_LOG(INFO, "cruise_speed_cmp[%.1f]:%.2f, %.2f",
          MTIME()->timestamp().sec() - gmp_prec_created_time_, veh_set_speed,
          ego_state_cart.ego_v_cruise);

  general_motion_planner_input_->ego_state_cart.ego_vel =
      ego_state_cart.ego_vel;

  general_motion_planner_input_->ego_state_cart.ego_steer_angle =
      ego_state_cart.ego_steer_angle;

  general_motion_planner_input_->ego_state_cart.ego_v_cruise = veh_set_speed;

  general_motion_planner_input_->ego_state_cart.ego_acc =
      ego_state_cart.ego_acc;

  general_motion_planner_input_->ego_state_cart.ego_pose.x =
      ego_state_cart.ego_pose.x;

  general_motion_planner_input_->ego_state_cart.ego_pose.y =
      ego_state_cart.ego_pose.y;

  general_motion_planner_input_->ego_state_cart.ego_pose.theta =
      ego_state_cart.ego_pose.theta;

  Point2d ego_pos_cart{
      general_motion_planner_input_->ego_state_cart.ego_pose.x,
      general_motion_planner_input_->ego_state_cart.ego_pose.y};

  ego_l0_ = get_ego_sl0(ego_pos_cart).y;

  for (int i = 0; i < ego_l0_buffer_.size() - 1; i++) {
    ego_l0_buffer_[i] = ego_l0_buffer_[i + 1];
  }
  ego_l0_buffer_.back() = ego_l0_;

  MSD_LOG(INFO, "ego_l0_buffer: %.2f %.2f %.2f %d", ego_l0_buffer_[0],
          ego_l0_buffer_[1], ego_l0_buffer_[2], call_count_);

  is_lane_crossed();

  update_lc_state(); // here, the lc_total info is updated, dheadaway updated

  update_lw_state();

  cal_lc_pass_info(); // must under update_lc_state()

  update_dheadaway();

  update_smoothed_headaway();

  Point2d ego_pos_local = env2local_pos(ego_pos_cart);

  general_motion_planner_input_->ego_state =
      general_motion_planner_input_->ego_state_cart;
  general_motion_planner_input_->ego_state.ego_pose.x = ego_pos_local.x;
  general_motion_planner_input_->ego_state.ego_pose.y = ego_pos_local.y;
  general_motion_planner_input_->ego_state.ego_pose.theta = 0.0;
  general_motion_planner_input_->highlevel_guidance = lc_state_;

  general_motion_planner_input_->headaway = headaway_;
  general_motion_planner_input_->dheadaway = dheadaway_;
  general_motion_planner_input_->headaway_smoothed = headaway_smoothed_;

  target_speed_lc_wait_max_ = ConfigurationContext::Instance()
                                  ->planner_config()
                                  .lateral_behavior_planner_config.max_v_at_lc;

  general_motion_planner_input_->target_speed_lc_wait_max =
      target_speed_lc_wait_max_;
}

void GeneralMotionPlannerPreprocessor::update_smoothed_headaway(void) {
  if (lc_state_ != LC_Dir::NONE) {
    headaway_smoothed_ = headaway_ - dheadaway_;
  } else {
    headaway_smoothed_ = smooth_headaway(headaway_smoothed_, headaway_);
  }
  MSD_LOG(INFO, "GMP smooth headaway[%.1f][%.2fs]",
          MTIME()->timestamp().sec() - gmp_prec_created_time_,
          headaway_smoothed_);
  return;
}

void GeneralMotionPlannerPreprocessor::update_lc_clear(void) {
  MSD_LOG(INFO, "GMP LC clear: judge started...");
  for (int i = 0; i < lc_back_buffer_.size() - 1; ++i) {
    lc_back_buffer_[i] = lc_back_buffer_[i + 1];
  }
  lc_back_buffer_.back() = general_motion_planner_output_->lc_action_state;

  if (lc_state_ == LC_Dir::NONE) {
    lc_clear_ = false;
    MSD_LOG(INFO, "GMP LC clear: NONE!");
  } else if ((lc_request_ == LC_Dir::NONE &&
              (!lane_cross_ || gmp_should_cancel_)) &&
             !lc_clear_) {
    lc_clear_ = true;
    MSD_LOG(INFO, "GMP LC clear: external clear!");
  } else if ((lc_wait_time_ >= 11.0) || (lc_trigger_time_ >= 19.5)) {
    lc_clear_ = true;
    MSD_LOG(INFO, "GMP LC clear: time overdue!");
  } else if ((!lane_cross_ && lc_back_buffer_[2] == -1 &&
              lc_back_buffer_[1] == -1 && lc_back_buffer_[0] == -1) &&
             !lc_clear_) {
    lc_clear_ = true;
    MSD_LOG(INFO, "GMP LC clear: lc back!");
  }
}

void GeneralMotionPlannerPreprocessor::update_lc_state(void) {
  int motion_result = 0;
  if (!general_motion_planner_output_->trajectories_gmp.empty()) {
    motion_result =
        general_motion_planner_output_->trajectories_gmp.front().motion_type;
  }

  motion_result_buffer_.front() = motion_result_buffer_.back();
  motion_result_buffer_.back() = motion_result;

  lc_request_buffer_.front() = lc_request_buffer_.back();
  lc_request_buffer_.back() = lc_request_;

  bool lc_triggered = lc_request_buffer_.front() == LC_Dir::NONE &&
                      lc_request_buffer_.back() != LC_Dir::NONE;

  if (lc_triggered) {
    lc_state_ = lc_request_buffer_.back(); // TODO: This is temporary
    ego_speed_lc_trigger_ = general_motion_planner_input_->ego_state.ego_vel;
  }
  cal_lc_total_info();

  bool ego_centered = abs(ego_l0_buffer_.back()) < 0.3;
  bool lc_overtime = lc_time_pass_ > 10;
  bool lc_overdist = lc_dist_remain_ <= 10;

  update_lc_clear();

  bool lc_should_finish = lc_state_ != LC_Dir::NONE &&
                          ((lane_cross_ && (lc_overtime || ego_centered)) ||
                           lc_overdist || lc_clear_);

  MSD_LOG(INFO, "GMP lc_should_finish: %d,%d,%d,%d,%d,%d,%d,%d,%d",
          lc_should_finish, gmp_should_cancel_, lc_state_, lane_cross_,
          lc_overtime, ego_centered, lc_overdist, lc_clear_, lc_request_);

  if (lc_should_finish) {
    lc_state_ = LC_Dir::NONE;
    lane_cross_ = false;
    retreat_count_ = 0;
    retreat_count_max_ = 1;
  }
}

void GeneralMotionPlannerPreprocessor::update_lw_state(void) {
  if (lc_state_ != LC_Dir::NONE) {
    if (((motion_result_buffer_.front() >= 1 &&
          motion_result_buffer_.front() <= 3 &&
          motion_result_buffer_.back() == 5) ||
         (motion_result_buffer_.front() >= 1 &&
          motion_result_buffer_.front() <= 3 &&
          motion_result_buffer_.back() == 0) ||
         (motion_result_buffer_.front() == 0 &&
          motion_result_buffer_.back() == 5) ||
         (motion_result_buffer_.front() == 0 &&
          motion_result_buffer_.back() == 0)) &&
        !lane_cross_) {
      lc_wait_ = true;
      update_retreat_target_speed();
    } else if (motion_result_buffer_.front() >= 1 &&
               motion_result_buffer_.front() <= 3 &&
               motion_result_buffer_.back() == 5 && lane_cross_) {
      update_pos_lc_target_speed();

    } else if (motion_result_buffer_.back() <= 3 &&
               motion_result_buffer_.back() >= 1 &&
               (motion_result_buffer_.front() == 5 ||
                motion_result_buffer_.front() == 0)) {
      lc_wait_ = false;
    } else if (lc_wait_ &&
               retreat_count_max_ <
                   general_motion_planner_output_->lc_wait_speed_adjust_count &&
               motion_result_buffer_.back() == 5) {
      retreat_count_max_ =
          general_motion_planner_output_->lc_wait_speed_adjust_count;
      update_retreat_target_speed();
    }
  } else {
    lc_wait_ = false;
    target_speed_lc_wait_ =
        general_motion_planner_input_->ego_state_cart.ego_vel;
  }

  retreat_count_max_ =
      general_motion_planner_output_->lc_wait_speed_adjust_count;

  MSD_LOG(INFO, "GMP retreat2[%.1f][%d] %d %d retreat_count %d %d",
          MTIME()->timestamp().sec() - gmp_prec_created_time_, lc_wait_,
          motion_result_buffer_.front(), motion_result_buffer_.back(),
          retreat_count_max_, retreat_count_);
}

void GeneralMotionPlannerPreprocessor::update_retreat_target_speed(void) {
  const double &ego_speed = general_motion_planner_input_->ego_state.ego_vel;
  const bool &gmp_wait = general_motion_planner_output_->lc_action_state == 0 &&
                         general_motion_planner_output_->lc_wait == 1;

  if ((!is_lane_stable_ || is_solid_line_ || is_steer_over_limit_) &&
      gmp_wait) {
    target_speed_lc_wait_ = ego_speed;
    return;
  }

  if (retreat_count_ >= retreat_count_max_) {
    return;
  }

  const double &set_speed =
      general_motion_planner_input_->ego_state_cart.ego_v_cruise;

  int retreat_speed_advice =
      general_motion_planner_output_->lc_wait_speed_adjust_advice;

  double min_vel = ego_speed_lc_trigger_ - 4.0;
  const double min_vel_retreat = 5.0;
  if (retreat_speed_advice == 2) {
    min_vel = (ego_speed_lc_trigger_ < min_vel_retreat)
                  ? ego_speed_lc_trigger_
                  : std::max(ego_speed_lc_trigger_ - 4.0, min_vel_retreat);
  }

  if (retreat_speed_advice == 1) { // accelerate
    target_speed_lc_wait_ =
        std::min(ego_speed + set_speed_delta_, set_speed + 2.70);
    target_speed_lc_wait_ =
        std::min(target_speed_lc_wait_, target_speed_lc_wait_max_);
  } else if (retreat_speed_advice == 2 ||
             retreat_speed_advice == 3) {                 // decelerate
    target_speed_lc_wait_ = ego_speed - set_speed_delta_; // default 4m/s
  } else {
    target_speed_lc_wait_ = set_speed;
  }
  target_speed_lc_wait_ = std::max(target_speed_lc_wait_, min_vel);
  MSD_LOG(INFO, "GMP retreat count[%.2f] %d,%.2f",
          MTIME()->timestamp().sec() - gmp_prec_created_time_, retreat_count_,
          target_speed_lc_wait_);
  retreat_count_++;

  general_motion_planner_output_->lc_wait_speed_advice = target_speed_lc_wait_;
}

void GeneralMotionPlannerPreprocessor::update_pos_lc_target_speed(void) {

  const double &ego_speed = general_motion_planner_input_->ego_state.ego_vel;

  const double &set_speed =
      general_motion_planner_input_->ego_state_cart.ego_v_cruise;

  int retreat_speed_advice =
      general_motion_planner_output_->lc_wait_speed_adjust_advice;

  const double min_vel_retreat = 15.0;
  double min_vel = (ego_speed_lc_trigger_ < min_vel_retreat)
                       ? ego_speed_lc_trigger_
                       : std::max(ego_speed_lc_trigger_ - 4.0, min_vel_retreat);

  if (retreat_speed_advice == 1) { // accelerate
    target_speed_lc_wait_ =
        std::min(ego_speed + set_speed_delta_, set_speed + 2.70);
    target_speed_lc_wait_ =
        std::min(target_speed_lc_wait_, target_speed_lc_wait_max_);
  } else if (retreat_speed_advice == 2) {                 // decelerate
    target_speed_lc_wait_ = ego_speed - set_speed_delta_; // default 4m/s
  } else {
    target_speed_lc_wait_ = set_speed;
  }
  target_speed_lc_wait_ = std::max(target_speed_lc_wait_, min_vel);

  MSD_LOG(INFO, "GMP pos lc targetspeed[%.2f] %.1fm/s",
          MTIME()->timestamp().sec() - gmp_prec_created_time_,
          target_speed_lc_wait_);

  general_motion_planner_output_->lc_wait_speed_advice = target_speed_lc_wait_;
}

void GeneralMotionPlannerPreprocessor::cal_lc_total_info(void) {
  if (lc_state_ == LC_Dir::NONE) {
    return;
  }

  bool lc_triggered = lc_request_buffer_.front() == LC_Dir::NONE &&
                      lc_request_buffer_.back() != LC_Dir::NONE;

  if (behavior_candidates_.empty() || lc_triggered) {
    double lc_time_total =
        lc_time_total_; // TODO: set lc_time to vary with ego_vel
    lc_time_total_ = lc_time_total;
    lc_dist_total_ = ego_speed_lc_trigger_ * lc_time_total;
    MSD_LOG(INFO,
            "GMP lc_to_static_obj LC_total_info 1  : %.2f s, %.2f, %.2f s",
            lc_time_total_, lc_dist_total_,
            MTIME()->timestamp().sec() - gmp_prec_created_time_);
  } else {
    if (general_motion_planner_output_->lc_to_static_obj_time > 0.01) {
      lc_time_total_ = general_motion_planner_output_->lc_to_static_obj_time;
      lc_dist_total_ = lc_time_total_ * ego_speed_lc_trigger_;
    } else {
      int index =
          general_motion_planner_output_->chosen_behavior_candidate_index;
      index = saturation(index, 0, int(behavior_candidates_.size() - 1));
      lc_time_total_ = behavior_candidates_[index].behavior_info.lc_time_total;
      lc_dist_total_ = behavior_candidates_[index].behavior_info.lc_dist_total;
    }

    MSD_LOG(INFO, "GMP lc_to_static_obj LC_total_info 2 : %.2f s, %.2f, %.2f s",
            lc_time_total_, lc_dist_total_,
            MTIME()->timestamp().sec() - gmp_prec_created_time_);
  }
  MSD_LOG(INFO, "GMP lc_to_static_obj LC_total_info  : %.2f s, %.2f, %.2f s",
          lc_time_total_, lc_dist_total_,
          MTIME()->timestamp().sec() - gmp_prec_created_time_);
}

void GeneralMotionPlannerPreprocessor::cal_lc_pass_info(void) {
  MSD_LOG(INFO,
          "GMP lc_to_static_obj LC_pass_info_pass before  : %.2f s, %.2f, "
          "%d,%.2f, %d, %.2f s",
          lc_time_pass_, lc_dist_pass_, lc_state_, lc_wait_time_, lc_wait_,
          MTIME()->timestamp().sec() - gmp_prec_created_time_);

  const double t_confirm = 0.4;
  double ego_vel = general_motion_planner_input_->ego_state_cart.ego_vel;

  if (!lc_wait_ && lc_state_ == LC_Dir::NONE) {
    lc_wait_time_ = 0.0;
  } else if (lc_wait_) {
    lc_wait_time_ += t_delta_;
  }

  if (lc_state_ == LC_Dir::NONE) {
    lc_latdist_abs_past_ = 0.0;
    lc_time_remain_ = lc_time_total_;
    lc_time_pass_ = 0.0;
    lc_dist_pass_ = 0.0;
    lc_dist_remain_ = lc_time_remain_ * ego_vel;
    lc_trigger_time_ = 0.0;
    return;
  } else if (lc_state_ != LC_Dir::NONE && lc_wait_ &&
             lc_time_pass_ <= 0.5 + t_confirm) {
    lc_trigger_time_ += t_delta_;
    lc_latdist_abs_past_ = std::abs(ego_l0_);
    lc_time_remain_ = lc_time_total_;
    lc_time_pass_ = 0.0;
    lc_dist_pass_ = 0.0;
    lc_dist_remain_ = lc_time_remain_ * ego_vel;
    return;
  }
  // ----------- update lc time and distance info ----------//

  bool lc_triggered = lc_request_buffer_.front() == LC_Dir::NONE &&
                      lc_request_buffer_.back() != LC_Dir::NONE;
  if (!behavior_candidates_.empty() && !lc_triggered &&
      lc_state_ != LC_Dir::NONE) {
    // update pass_time and pass_dist based on the chosen lc candidate
    int index = general_motion_planner_output_->chosen_behavior_candidate_index;
    index = saturation(index, 0, int(behavior_candidates_.size() - 1));
    lc_dist_pass_ = behavior_candidates_[index].behavior_info.lc_dist_pass;
    lc_time_pass_ = behavior_candidates_[index].behavior_info.lc_time_pass;
  }

  lc_trigger_time_ += t_delta_;
  lc_time_pass_ += t_delta_;
  lc_dist_pass_ += t_delta_ * ego_vel;

  lc_dist_remain_ = std::max(lc_dist_total_ - lc_dist_pass_, 0.0);
  lc_time_remain_ = std::max(lc_time_total_ - lc_time_pass_, 0.0);

  if (!lane_cross_) {
    lc_latdist_abs_past_ = abs(ego_l0_);
  } else {
    lc_latdist_abs_past_ = lane_width_ - abs(ego_l0_);
  }

  MSD_LOG(INFO,
          "GMP lc_to_static_obj LC_pass_info[%.2f]: %.1fs %.1fs %.1fs, "
          "%.1f,%.1f,%.1f,%d %d",
          MTIME()->timestamp().sec() - gmp_prec_created_time_, lc_trigger_time_,
          lc_time_pass_, lc_time_remain_, lc_dist_pass_, lc_dist_remain_,
          lc_wait_time_, lc_state_, lc_wait_);
}

void GeneralMotionPlannerPreprocessor::update_dheadaway(void) {
  double lc_proceed_time = general_motion_planner_output_->lc_in_proceed_time;
  if (lc_proceed_time >= 1.6) {
    dheadaway_ = dheadaway_shrink_;
  } else {
    dheadaway_ = dheadaway_origin_;
  }
  return;
}

vector<gmp_interface::ObsInfo>
GeneralMotionPlannerPreprocessor::generate_gmp_object_info() {
  using gmp_interface::ObjPredSlice;
  using gmp_interface::ObsInfo;
  std::vector<ObsInfo> obs_list{};
  unordered_set<int> obj_id_cache;
  const auto &obstacle_manager_ = baseline_info_->obstacle_manager();
  const auto &all_obstacles = obstacle_manager_.get_obstacles();
  const double &ego_speed =
      general_motion_planner_input_->ego_state_cart.ego_vel;

  double planning_init_state_s =
      baseline_info_->get_planning_start_point().path_point.s;

  double sm_lon_min = 1.5;
  if (lc_state_ != LC_Dir::NONE) {
    sm_lon_min = 10.0;
  }

  for (const auto &ptr_obj : all_obstacles.Items()) {
    ObsInfo obj_info{};
    obj_info.id = ptr_obj->Id();
    obj_info.obj_size.length = ptr_obj->PerceptionBoundingBox().length();
    obj_info.obj_size.width = ptr_obj->PerceptionBoundingBox().width();
    // update headaway according to driving task: when lc, shrink
    // TODO: smooth headaway transition when lc switched to lh

    // end of headaway shrinking
    double sm_lon = obj_info.obj_size.length +
                    max(sm_lon_min, headaway_smoothed_ * ego_speed);
    if (lc_state_ != LC_Dir::NONE) {
      sm_lon = min(sm_lon, 25.0);
    }
    double sm_lat = 0.5 * obj_info.obj_size.width + 1.4;

    SafetyMargin safety_margin_ini{sm_lon, sm_lat};

    // MSD_LOG(INFO,"GMP SM [ego_speed: %.2f m/s]: %.2f, %.2f",
    //         ego_speed, safety_margin_ini.longitu,safety_margin_ini.lateral);
    vector<ObjPredSlice> obj_pred_slices{};
    obj_info.obj_state_init =
        get_predslice_at_time(0.0, ptr_obj, planning_init_state_s,
                              baseline_info_->get_frenet_coord());
    obj_info.obj_state_init.safety_margin = safety_margin_ini;

    for (const double &time : time_array_) {
      SafetyMargin safety_margin_t = safety_margin_ini;

      if (time <= 1.5) {
        safety_margin_t.longitu = obj_info.obj_size.length + 1.5;
      } else if (time <= 3.5) {
        double temp = obj_info.obj_size.length +
                      min((headaway_ - dheadaway_), 0.7) * ego_speed;
        safety_margin_t.longitu = min(temp, safety_margin_ini.longitu);
      } else {
        double temp =
            obj_info.obj_size.length + (headaway_ - dheadaway_) * ego_speed;
        safety_margin_t.longitu = min(temp, safety_margin_ini.longitu);
      }

      ObjPredSlice obj_pred_single_slice =
          get_predslice_at_time(time, ptr_obj, planning_init_state_s,
                                baseline_info_->get_frenet_coord());

      obj_pred_single_slice.safety_margin = safety_margin_t;
      obj_info.pred_trajectory.push_back(obj_pred_single_slice);
    }

    obs_list.emplace_back(obj_info);
    obj_id_cache.insert(obj_info.id);
  }
  // maintain obj_timer_cache:
  vector<int> obj_to_delete;
  for (auto &k : obj_timer_cache_) {
    MSD_LOG(INFO, "GMP_timer[%d] %.2f", k.first, k.second);
    if (obj_id_cache.find(k.first) == obj_id_cache.end()) {
      obj_to_delete.emplace_back(k.first);
    }
  }
  if (!obj_to_delete.empty()) {
    for (const auto &id : obj_to_delete) {
      MSD_LOG(INFO, "GMP_timer erasing [%d]", id);
      obj_timer_cache_.erase(id);
    }
  }
  return obs_list;
}

gmp_interface::ObjPredSlice
GeneralMotionPlannerPreprocessor::get_predslice_at_time(
    const double &time, const Point2D &obj_sl,
    const gmp_interface::ObjPredSlice &obj_info_base,
    const double &pred_decel) {
  using gmp_interface::Point2d;
  ObjPredSlice obj_pred_slice{};

  double detaT = time - pred_valid_time_;

  double obj_vel0 = obj_info_base.obj_state_local.vel;
  double obj_s0 = obj_sl.x;
  double obj_l0 = obj_sl.y;

  double obj_l = obj_l0;
  double obj_s = obj_vel0 * detaT + obj_s0 + 1 / 2 * pred_decel * detaT * detaT;
  double obj_global_x = obj_info_base.obj_state_env.pos.x;
  double obj_global_y = obj_info_base.obj_state_env.pos.y;

  Point2D obj_global_pos{0.0, 0.0};
  Point2D obj_frenet_pos{obj_s, obj_l};

  double obj_yaw = 0.0;
  Point2d obj_local_pos{0.0, 0.0};

  const auto &ego_state = general_motion_planner_input_->ego_state_cart;
  double ego_heading = ego_state.ego_pose.theta;
  int state_temp = 0;
  if (frenet_coord_->FrenetCoord2CartCoord(obj_frenet_pos, obj_global_pos) ==
      TRANSFORM_SUCCESS) {
    Point2d obj_env_pos{obj_global_pos.x, obj_global_pos.y};
    obj_local_pos = env2local_pos(obj_env_pos);

    obj_yaw = frenet_coord_->GetRefCurveHeading(obj_frenet_pos.x) - ego_heading;
  } else {
    // TODO: the obj_pos in global coord is not updated here.
    extend_frenet(obj_frenet_pos, obj_local_pos, obj_yaw);
    state_temp = 1;
  }

  // if (obj_l < 1.0 && obj_l > 0.0)
  //   MSD_LOG(INFO,"Obj_base s[%d]: %.2f, l: %.2f %.2f %.2f, %.2f", state_temp,
  //                   obj_s, obj_l,obj_vel0 * detaT,obj_local_pos.x,obj_yaw);

  obj_pred_slice.obj_state_local.vel = obj_vel0;
  obj_pred_slice.obj_state_local.s = obj_s;
  obj_pred_slice.obj_state_local.pos.x = obj_local_pos.x;
  obj_pred_slice.obj_state_local.pos.y = obj_local_pos.y;
  obj_pred_slice.obj_state_local.heading = obj_yaw;

  obj_pred_slice.obj_state_env.vel = obj_vel0;
  obj_pred_slice.obj_state_env.s = obj_s;
  obj_pred_slice.obj_state_env.pos.x = obj_global_pos.x;
  obj_pred_slice.obj_state_env.pos.y = obj_global_pos.y;
  obj_pred_slice.obj_state_env.heading = obj_yaw + ego_heading;

  if (abs(obj_l) <= lane_width_ / 2.0) {
    obj_pred_slice.lane_assignment = 1; // current lane
  } else if (obj_l > lane_width_ / 2.0 && obj_l <= lane_width_ * 1.5 + 0.7) {
    obj_pred_slice.lane_assignment = 2; // left lane
  } else if (obj_l < -lane_width_ / 2.0 && obj_l >= -lane_width_ * 1.5 - 0.7) {
    obj_pred_slice.lane_assignment = 3; // right lane
  } else {
    obj_pred_slice.lane_assignment = -1; // faraway lane
  }

  // SafetyMargin safety_margin{20.0, 2.0};
  obj_pred_slice.safety_margin = obj_info_base.safety_margin;

  return obj_pred_slice;
}

Point2D GeneralMotionPlannerPreprocessor::get_obj_base_sl(
    const ObjPredSlice &obj_info_base) {
  using gmp_interface::Point2d;
  Point2D obj_base_sl{0.0, 0.0};
  double refline_end_s = frenet_coord_->GetLength();

  double obj_global_x = obj_info_base.obj_state_env.pos.x;
  double obj_global_y = obj_info_base.obj_state_env.pos.y;
  double obj_local_x = obj_info_base.obj_state_local.pos.x;
  double obj_local_y = obj_info_base.obj_state_local.pos.y;
  double obj_s0 = obj_info_base.obj_state_local.s;
  double obj_ds = obj_s0 - refline_end_s;
  double veh_heading =
      general_motion_planner_input_->ego_state_cart.ego_pose.theta;

  Point2D obj_global_pos{obj_global_x, obj_global_y};
  Point2D obj_frenet_pos{0.0, 0.0};
  double obj_yaw = 0.0;

  if (frenet_coord_->CartCoord2FrenetCoord(obj_global_pos, obj_frenet_pos) ==
      TRANSFORM_SUCCESS) {
    // obj_l0 = obj_frenet_pos.y;
    obj_base_sl.x = obj_frenet_pos.x;
    obj_base_sl.y = obj_frenet_pos.y;
    // MSD_LOG(INFO, "obj_base_sl: %.2f, %.2f",obj_base_sl.x, obj_base_sl.y);

  } else {
    Point2D refline_end_frenet{refline_end_s, 0.0};
    Point2D refline_start_frenet{0.1, 0.0};
    Point2D refline_end_global{0.0, 0.0};
    Point2D refline_start_global{0.0, 0.0};
    bool trans_state1 = frenet_coord_->FrenetCoord2CartCoord(
        refline_end_frenet, refline_end_global);
    bool trans_state2 = frenet_coord_->FrenetCoord2CartCoord(
        refline_start_frenet, refline_start_global);

    Point2d refline_end_env{refline_end_global.x, refline_end_global.y};
    Point2d refline_end_local = env2local_pos(refline_end_env);

    Point2d refline_start_env{refline_start_global.x, refline_start_global.y};
    Point2d refline_start_local = env2local_pos(refline_start_env);

    double obj2start_point = sqrt(pow(obj_local_y - refline_start_local.y, 2) +
                                  pow(obj_local_x - refline_start_local.x, 2));
    double obj2end_point = sqrt(pow(obj_local_y - refline_end_local.y, 2) +
                                pow(obj_local_x - refline_end_local.x, 2));

    if (obj2start_point < obj2end_point) {
      double refline_start_heading =
          frenet_coord_->GetRefCurveHeading(refline_start_frenet.x) -
          veh_heading;
      double d_x = obj_local_x - refline_start_local.x;
      double d_y = obj_local_y - refline_start_local.y;
      double obj2start_dist = obj2start_point;
      double obj2start_ang = 0.0;
      if (d_x == 0) {
        obj2start_ang = (d_y >= 0) ? (-3.14159 / 2) : (3.14159 / 2);
      } else {
        obj2start_ang = atan(d_y / d_x);
      }
      obj_base_sl.x =
          0.1 - cos(obj2start_ang - refline_start_heading) * obj2start_dist;
      obj_base_sl.y =
          -sin(obj2start_ang - refline_start_heading) * obj2start_dist;

    } else {
      double refline_end_heading =
          frenet_coord_->GetRefCurveHeading(refline_end_frenet.x) - veh_heading;
      double d_x = obj_local_x - refline_start_local.x;
      double d_y = obj_local_y - refline_start_local.y;
      double obj2end_dist = obj2end_point;
      double obj2end_ang = 0.0;
      if (d_x == 0) {
        obj2end_ang = (d_y >= 0) ? (3.1416 / 2) : (-3.1416 / 2);
      } else {
        obj2end_ang = atan(d_y / d_x);
      }
      obj_base_sl.x =
          refline_end_s + cos(obj2end_ang - refline_end_heading) * obj2end_dist;
      obj_base_sl.y = sin(obj2end_ang - refline_end_heading) * obj2end_dist;
    }
  }
  return obj_base_sl;
}

gmp_interface::ObjPredSlice
GeneralMotionPlannerPreprocessor::get_predslice_at_time(
    const double &time, const Obstacle *obstacle, const double &ego_s,
    std::shared_ptr<FrenetCoordinateSystem> frenet_coord) {
  using gmp_interface::ObjPredSlice;
  ObjPredSlice obj_pred_slice{};
  const auto &traj_point = obstacle->GetPointAtTime(time);
  const auto &ego_state = general_motion_planner_input_->ego_state_cart;

  ObjState obj_state_env{};
  ObjState obj_state_local{};

  obj_state_env.pos.x = traj_point.path_point.x;
  obj_state_env.pos.y = traj_point.path_point.y;
  obj_state_env.s = traj_point.path_point.s;
  obj_state_env.vel = traj_point.v;
  obj_state_env.accel = traj_point.a;
  obj_state_env.heading = traj_point.path_point.theta;

  obj_state_local.pos = env2local_pos(obj_state_env.pos);
  obj_state_local.s = traj_point.path_point.s;
  obj_state_local.vel = traj_point.v;
  obj_state_local.accel = traj_point.a;
  obj_state_local.heading =
      traj_point.path_point.theta - ego_state.ego_pose.theta;

  obj_pred_slice.obj_state_env = obj_state_env;
  obj_pred_slice.obj_state_local = obj_state_local;

  SafetyMargin safety_margin{20.0, 2.0};
  obj_pred_slice.safety_margin = safety_margin;

  return obj_pred_slice;
}

Point2D GeneralMotionPlannerPreprocessor::get_ego_sl0(
    gmp_interface::Point2d ego_pose0) {
  Point2D frenet_p{-1, -100};
  Point2D carte_p{ego_pose0.x, ego_pose0.y};
  double veh_s0 = baseline_info_->get_ego_state().planning_start_state.s;

  if (baseline_info_->get_frenet_coord()->CartCoord2FrenetCoord(
          carte_p, frenet_p) == TRANSFORM_SUCCESS) {
    return frenet_p;
  }
  return frenet_p;
}

gmp_interface::Point2d GeneralMotionPlannerPreprocessor::env2local_pos(
    gmp_interface::Point2d point_env) {
  using gmp_interface::Point2d;
  auto &enu2car = enu2car_;
  Eigen::Vector3d car_point, enu_point;

  enu_point.x() = point_env.x;
  enu_point.y() = point_env.y;
  enu_point.z() = 0;

  car_point = enu2car * enu_point;

  Point2d point_local{};
  point_local.x = car_point.x();
  point_local.y = car_point.y();

  return point_local;
}

} // namespace msquare