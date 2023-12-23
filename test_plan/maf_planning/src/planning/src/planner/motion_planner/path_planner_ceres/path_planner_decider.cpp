#include "path_planner_decider.hpp"
#include "common/math/box2d.h"
#include "linear_interpolation_utility.hpp"

namespace path_planner {

void PathPlannerDecider::update_input(const PathPlannerInput &input) {
  input_ = input;
  half_self_width_ = input.vehicle_param.width / 2.;
  half_self_length_ = input.vehicle_param.length / 2.;
  pre_dodge_info_ = input.dodge_info;
  
  Index<NUM_PATH_CONTROL_POINTS> index;
  while (index.advance()) {
    const auto &map_info = input_.path_segments[index.segment_index]
                               .quad_point_info[index.quad_index]
                               .refline_info;
    auto &decision_info = decision_info_[index.segment_index][index.quad_index];
    decision_info.ref_offset = 0.;
    decision_info.left_activation_dist =
        std::max(map_info.left_lane_border - LANE_LINE_ACTIVATION_DIST_M, 0.1);
    decision_info.right_activation_dist = std::min(
        -map_info.right_lane_border + LANE_LINE_ACTIVATION_DIST_M, -0.1);
    decision_info.left_map_hard_dist =
        map_info.left_road_border - half_self_width_;
    decision_info.right_map_hard_dist =
        -map_info.right_road_border + half_self_width_;
    decision_info.left_map_hard_desire = decision_info.left_map_hard_dist;
    decision_info.right_map_hard_desire = decision_info.right_map_hard_dist;

    decision_info.l_min = std::numeric_limits<double>::lowest();
    decision_info.l_max = std::numeric_limits<double>::max();
    decision_info.left_obs_constrain = std::numeric_limits<double>::max();
    decision_info.right_obs_constrain = std::numeric_limits<double>::lowest();
    decision_info.left_obs_desire = std::numeric_limits<double>::max();
    decision_info.right_obs_desire = std::numeric_limits<double>::lowest();
    decision_info.left_obs_inflation = std::numeric_limits<double>::max();
    decision_info.right_obs_inflation = std::numeric_limits<double>::lowest();
  }

  if (input_.lc_decider_info.lc_status ==
      path_planner::LaneChangeInfo::LaneKeep) {
    lc_end_point_fren_.x = -1;

    Index<NUM_PATH_CONTROL_POINTS> index;
    while (index.advance()) {
      const auto &sample_info = input_.path_segments[index.segment_index]
                                    .quad_point_info[index.quad_index];
      auto &decision_info =
          decision_info_[index.segment_index][index.quad_index];
      decision_info.left_obs_inflation =
          std::max(sample_info.refline_info.left_lane_border -
                       half_self_width_ - LANE_BORDER_AVD_DIST_M,
                   0.1);
      decision_info.right_obs_inflation =
          std::min(-sample_info.refline_info.right_lane_border +
                       half_self_width_ + LANE_BORDER_AVD_DIST_M,
                   -0.1);
    }

    if (input_.ddp_info.valid) {
      update_key_params("ddp");
      update_offset_list_and_active_from_ddp();
    } else {
      update_key_params();
      init_offset_list();

      ActiveAvoid active_avoid(input_);
      active_avoid.offset_active(decision_info_, active_target_l_,
                                 pre_dodge_info_);

    }

    update_road_border_bound();

    update_obstacle_bound();

    update_desire_bound();

    smooth_bound();

    slash_activitation_dist();
  } else {
    update_key_params();

    sample_quintic_poly();

    update_road_border_bound();

    update_obstacle_bound();

    slash_activitation_dist();
  }

  Index<NUM_PATH_CONTROL_POINTS> index_debug;
  while (index_debug.advance()) {
    const auto &map_info = input_.path_segments[index.segment_index]
                               .quad_point_info[index.quad_index]
                               .refline_info;
    auto &decision_info =
        decision_info_[index_debug.segment_index][index_debug.quad_index];
    auto &lane_activation_dist =
        lane_activation_dis_[index_debug.segment_index][index_debug.quad_index];
    auto &obj_activation_dist =
        obj_activation_dis_[index_debug.segment_index][index_debug.quad_index];

    lane_activation_dist.first =
        map_info.left_lane_border - decision_info.left_activation_dist;
    lane_activation_dist.second =
        map_info.right_lane_border + decision_info.right_activation_dist;

    double left_obs_desire = std::min(decision_info.left_obs_inflation,
                                      decision_info.left_obs_desire);
    double right_obs_desire = std::max(decision_info.right_obs_inflation,
                                       decision_info.right_obs_desire);
    obj_activation_dist.first = map_info.left_lane_border - left_obs_desire;
    obj_activation_dist.second = map_info.right_lane_border + right_obs_desire;
  }

  // Index<NUM_PATH_CONTROL_POINTS> index_p;
  // while (index_p.advance()) {
  //   const auto &map_info = input_.path_segments[index_p.segment_index]
  //                              .quad_point_info[index_p.quad_index]
  //                              .refline_info;
  //   auto &decision_info =
  //       decision_info_[index_p.segment_index][index_p.quad_index];
  //   std::cout << "final t : " << map_info.time
  //             << " offset : " << decision_info.ref_offset
  //             << " l : " << decision_info.left_activation_dist
  //             << " r : " << decision_info.right_activation_dist
  //             << " obc l : " <<
  //             std::min(decision_info.left_obs_constrain, 10.)
  //             << " r : " << std::min(decision_info.right_obs_constrain, 10.)
  //             << " obd l : " << std::min(decision_info.left_obs_desire, 10.)
  //             << " r :" << std::max(decision_info.right_obs_desire, -10.)
  //             << " obf l : " <<
  //             std::min(decision_info.left_obs_inflation, 10.)
  //             << " obf r : " << std::max(decision_info.right_obs_inflation,
  //             -10.) << std::endl;
  // }
}

void PathPlannerDecider::update_key_params(std::string mode) {
  if (mode == "ddp") {
    key_params_.pred_vilad_time_max = PredViladTimeMaxVSVelDDP;
    key_params_.pred_vilad_time_min = PredViladTimeMinVSVelDDP;
  } else {
    key_params_.pred_vilad_time_max = PredViladTimeMaxVSVel;
    key_params_.pred_vilad_time_min = PredViladTimeMinVSVel;
    if (input_.lat_safety_improved) {
      key_params_.pred_vilad_time_min.second += 1.0;
    }
  }
}

void PathPlannerDecider::update_offset_list_and_active_from_ddp() {
  Index<NUM_PATH_CONTROL_POINTS> index;
  while (index.advance()) {
    if (index.segment_index >= input_.ddp_info.ddp_path.size() ||
        index.quad_index >=
            input_.ddp_info.ddp_path[index.segment_index].size())
      continue;
    auto &decision_info = decision_info_[index.segment_index][index.quad_index];
    // decision_info.ref_offset =
    //     input_.ddp_info.ddp_path[index.segment_index][index.quad_index].y;
    const auto &map_info = input_.path_segments[index.segment_index]
                               .quad_point_info[index.quad_index]
                               .refline_info;
    double lane_left_border_limit =
        std::max(map_info.left_lane_border - half_self_width_ - 0.1, 0.1);
    double lane_right_border_limit =
        std::min(-map_info.right_lane_border + half_self_width_ + 0.1, -0.1);
    decision_info.ref_offset = std::min(
        std::max(
            input_.ddp_info.ddp_path[index.segment_index][index.quad_index].y,
            lane_right_border_limit),
        lane_left_border_limit);
    decision_info.left_activation_dist =
        std::max(decision_info.left_activation_dist, decision_info.ref_offset);
    decision_info.right_activation_dist =
        std::min(decision_info.right_activation_dist, decision_info.ref_offset);
  }
}

void PathPlannerDecider::init_offset_list() {
  double max_ref_jerk_v = MAX_REF_JERK_V;
  double max_ref_jerk = input_.intersec_info.keep_in_intersection
                            ? MAX_REF_JERK_INTERSECTION
                            : MAX_REF_JERK;
  double min_ref_jerk_v =
      std::min(input_.min_ref_jerk.first, max_ref_jerk_v - 0.1);
  double min_ref_jerk = std::min(input_.min_ref_jerk.second, max_ref_jerk);
  double ref_jerk =
      max_ref_jerk + (min_ref_jerk - max_ref_jerk) /
                         (min_ref_jerk_v - max_ref_jerk_v) *
                         (input_.planning_init_state.v - max_ref_jerk_v);
  ref_jerk = std::min(std::max(ref_jerk, min_ref_jerk), max_ref_jerk);
  double end_s = 0;
  msquare::planning_math::QuinticPoly1d quintic_poly;
  while (ref_jerk < max_ref_jerk + 1.0e-6) {
    bool overshot = false;
    bool cross_border = false;
    double dddl =
        ref_jerk / std::pow(std::max(input_.planning_init_state.v, 5.5), 3);
    if ((input_.planning_init_state.l > 0 &&
         ActiveAvoid::optimize_quintic_poly(input_, 0, -dddl, "right",
                                            quintic_poly, end_s)) ||
        (input_.planning_init_state.l < 0 &&
         ActiveAvoid::optimize_quintic_poly(input_, 0, dddl, "left",
                                            quintic_poly, end_s))) {
      Index<NUM_PATH_CONTROL_POINTS> index;
      while (index.advance()) {
        const auto &sample_s = input_.path_segments[index.segment_index]
                                   .quad_point_info[index.quad_index]
                                   .sample_s -
                               input_.planning_init_state.s;
        const auto &map_info = input_.path_segments[index.segment_index]
                                   .quad_point_info[index.quad_index]
                                   .refline_info;
        if (sample_s < end_s && map_info.time > 0.5) {
          double quintic_l = quintic_poly.Evaluate(0, sample_s);
          if ((input_.planning_init_state.l < 0 &&
               (quintic_l > 0.1 ||
                quintic_l < input_.planning_init_state.l - 0.1)) ||
              (input_.planning_init_state.l > 0 &&
               (quintic_l < -0.1 ||
                quintic_l > input_.planning_init_state.l + 0.1))) {
            overshot = true;
          }
          if (quintic_l > map_info.left_lane_border - half_self_width_ -
                              LANE_BORDER_AVD_DIST_M ||
              quintic_l < -map_info.right_lane_border + half_self_width_ +
                              LANE_BORDER_AVD_DIST_M) {
            cross_border = true;
          }
        }
      }
      if (!overshot && !cross_border)
        break;
    }
    ref_jerk += 0.1;
  }

  if (end_s > 0.1) {
    Index<NUM_PATH_CONTROL_POINTS> index;
    while (index.advance()) {
      const auto &sample_s = input_.path_segments[index.segment_index]
                                 .quad_point_info[index.quad_index]
                                 .sample_s -
                             input_.planning_init_state.s;
      if (sample_s < end_s) {
        auto &decision_info =
            decision_info_[index.segment_index][index.quad_index];
        decision_info.ref_offset = quintic_poly.Evaluate(0, sample_s);
      }
    }
  }
}

void PathPlannerDecider::update_road_border_bound() {
  // scenario distinguish
  bool is_merge = current_lane_end_distinguish();

  double last_bound_left = 1.0e6;
  double last_bound_right = -1.0e6;
  double last_time = 0.;
  Index<NUM_PATH_CONTROL_POINTS> index;
  while (index.advance()) {
    const auto &sample_info = input_.path_segments[index.segment_index]
                                  .quad_point_info[index.quad_index];
    const auto &map_info = sample_info.refline_info;
    auto &decision_info = decision_info_[index.segment_index][index.quad_index];

    double delta_time = map_info.time - last_time;
    double decay_dist =
        std::abs(delta_time) < 5.0 ? std::max(delta_time * 0.2, 0.) : 0.;
    if (map_info.left_road_border_type == 1) {
      double hard_buffer = cal_hard_road_border_buffer(sample_info, "left");
      double soft_buffer = cal_soft_road_border_buffer(sample_info, "left",
                                                       is_merge, hard_buffer);
      decision_info.left_map_hard_dist =
          map_info.left_road_border - hard_buffer;
      decision_info.left_map_hard_desire =
          std::min(map_info.left_road_border - soft_buffer,
                   last_bound_left + decay_dist);
    }

    if (map_info.right_road_border_type == 1) {
      double hard_buffer = cal_hard_road_border_buffer(sample_info, "right");
      double soft_buffer = cal_soft_road_border_buffer(sample_info, "right",
                                                       is_merge, hard_buffer);
      decision_info.right_map_hard_dist =
          -map_info.right_road_border + hard_buffer;
      decision_info.right_map_hard_desire =
          std::max(-map_info.right_road_border + soft_buffer,
                   last_bound_right - decay_dist);
    }
    last_bound_left = decision_info.left_map_hard_desire;
    last_bound_right = decision_info.right_map_hard_desire;
    last_time = map_info.time;
  }
}

void PathPlannerDecider::sample_quintic_poly() {
  double s_pred = input_.lc_decider_info.t_pred * input_.planning_init_state.v;
  double l_pred =
      input_.planning_init_state.l + input_.planning_init_state.dl * s_pred;

  double end_vel =
      input_.path_segments.back().quad_point_info.back().speed_plan.v;
  double v_average = (input_.planning_init_state.v + end_vel) / 2.;

  double desire_end_s = std::abs(input_.planning_init_state.l) /
                        LAT_VEL_DESIRE * std::max(v_average, 3.0);
  double prev_lc_end_s = input_.lc_decider_info.prev_lc_end_point_fren.x -
                         input_.planning_init_state.s;
  if (prev_lc_end_s > 0) {
    desire_end_s = prev_lc_end_s;
  }
  if (input_.lc_decider_info.origin_lane_front_obs_id != -1000) {
    update_end_s_for_collision_avoidance(s_pred, l_pred, desire_end_s);
  }

  double sample_deta_s = std::max(v_average, 3.0) * LCQuinticPolySampleT;

  int cost_min_index = -1;
  double path_cost_min = std::numeric_limits<double>::max();
  std::vector<std::pair<msquare::planning_math::QuinticPoly1d *, double>>
      quintic_poly_vector;
  double path_cost_fix_end = -1;
  ;
  int cost_index_fix_end = -1;
  double acc_approximate_fix_end = 0;
  double acc_approximate_min_cost = 0;
  bool fix_lc_end{false};

  if (input_.planning_init_state.l > 0 &&
      std::fabs(input_.planning_init_state.l) <
          input_.planning_init_state.sample_info.refline_info
              .left_lane_border) {
    fix_lc_end = true;
  } else if (input_.planning_init_state.l < 0 &&
             std::fabs(input_.planning_init_state.l) <
                 input_.planning_init_state.sample_info.refline_info
                     .right_lane_border) {
    fix_lc_end = true;
  }

  for (int i = -3; i <= 3; ++i) {
    double end_s = desire_end_s + sample_deta_s * i;
    if (end_s < 1.0)
      continue;

    msquare::planning_math::QuinticPoly1d quintic_poly(
        l_pred, input_.planning_init_state.dl, input_.planning_init_state.ddl,
        0., 0., 0., end_s - s_pred);

    // calculate path cost
    double path_cost = 0.;
    double path_cost_end = 0.;
    double path_cost_l = 0.;
    double path_cost_dl = 0.;
    double path_cost_ddl = 0.;
    double path_cost_acc = 0.;
    double path_cost_overshot = 0.;

    // end s cost
    double end_t_error =
        (end_s - desire_end_s) / std::max(input_.planning_init_state.v, 0.1);
    double end_t_weight = 100.;
    path_cost_end = end_t_error * end_t_error * end_t_weight;

    int count = 0;
    double acc_approximate_max = 0.;
    Index<NUM_PATH_CONTROL_POINTS> index;
    while (index.advance()) {
      double sample_s = input_.path_segments[index.segment_index]
                            .quad_point_info[index.quad_index]
                            .sample_s -
                        input_.planning_init_state.s - s_pred;
      if (sample_s < 0)
        continue;
      if (sample_s > end_s - s_pred)
        break;
      double v = std::max(3.0, input_.path_segments[index.segment_index]
                                   .quad_point_info[index.quad_index]
                                   .speed_plan.v);
      double l = quintic_poly.Evaluate(0, sample_s);
      double l_weight = std::abs(l) < 0.3 ? 0.1 : 1.;
      path_cost_l += l * l * l_weight;

      double dl = quintic_poly.Evaluate(1, sample_s);
      double dl_weight = 1. * std::pow(v, 2);
      path_cost_dl += dl * dl * dl_weight;

      double ddl = quintic_poly.Evaluate(2, sample_s);
      double ddl_weight = 1 * std::pow(v, 4);
      path_cost_ddl += ddl * ddl * ddl_weight;

      // approximate value of lateral acc cost
      double curv = input_.path_segments[index.segment_index]
                        .quad_point_info[index.quad_index]
                        .refline_info.curvature;
      double acc_approximate = (curv + ddl) * v * v;
      double acc_weight = std::abs(acc_approximate) > 3.0 ? 10 : 1.;
      path_cost_acc += acc_approximate * acc_approximate * acc_weight;
      acc_approximate_max =
          std::fabs(acc_approximate) > std::fabs(acc_approximate_max)
              ? acc_approximate
              : acc_approximate_max;

      // overshot cost
      if ((input_.planning_init_state.l > 0.2 && l < -0.1) ||
          (input_.planning_init_state.l < -0.2 && l > 0.1)) {
        double overshot_weight = 100.;
        path_cost_overshot += l * l * overshot_weight;
      }

      ++count;
    }
    if (count == 0)
      continue;
    path_cost = path_cost_end + path_cost_l + path_cost_dl + path_cost_ddl +
                path_cost_acc + path_cost_overshot;

    quintic_poly_vector.emplace_back(std::make_pair(&quintic_poly, end_s));
    if (i == 0) {
      path_cost_fix_end = path_cost;
      cost_index_fix_end = (int)quintic_poly_vector.size() - 1;
      acc_approximate_fix_end = acc_approximate_max;
    }
    if (path_cost_min > path_cost) {
      path_cost_min = path_cost;
      cost_min_index = (int)quintic_poly_vector.size() - 1;
    }
  }

  if ((fix_lc_end || std::fabs(acc_approximate_fix_end) < 3.5) &&
      cost_index_fix_end > 0 && cost_index_fix_end != cost_min_index) {
    cost_min_index = cost_index_fix_end;
  }

  Index<NUM_PATH_CONTROL_POINTS> index;
  while (index.advance()) {
    double l = 0;
    if (cost_min_index != -1 && !quintic_poly_vector.empty()) {
      double sample_s = input_.path_segments[index.segment_index]
                            .quad_point_info[index.quad_index]
                            .sample_s -
                        input_.planning_init_state.s - s_pred;
      if (sample_s < 0) {
        l = l_pred + sample_s * input_.planning_init_state.dl;
      } else if (sample_s >
                 quintic_poly_vector[cost_min_index].second - s_pred) {
        l = 0.;
      } else {
        l = quintic_poly_vector[cost_min_index].first->Evaluate(0, sample_s);
      }
    }
    decision_info_[index.segment_index][index.quad_index].ref_offset = l;
    decision_info_[index.segment_index][index.quad_index].left_activation_dist =
        l + 0.2;
    decision_info_[index.segment_index][index.quad_index]
        .right_activation_dist = l - 0.2;
  }

  if (cost_min_index != -1 && !quintic_poly_vector.empty()) {
    double end_s_res = quintic_poly_vector[cost_min_index].second +
                       input_.planning_init_state.s;
    lc_end_point_fren_.x = end_s_res;
    lc_end_point_fren_.y = 0;
  }
}

void PathPlannerDecider::update_end_s_for_collision_avoidance(
    const double s_pred, const double l_pred, double &desire_end_s) {
  ObsInfo front_obs_from_curr_lane;
  for (const auto &obs : input_.obs_list) {
    if (obs.id == input_.lc_decider_info.origin_lane_front_obs_id) {
      front_obs_from_curr_lane = obs;
    }
  }

  bool collision = true;
  while (collision && desire_end_s > s_pred + 0.5) {
    double max_jerk = 0;
    double max_dddl = 0;
    msquare::planning_math::QuinticPoly1d quintic_poly(
        l_pred, input_.planning_init_state.dl, input_.planning_init_state.ddl,
        0., 0., 0., desire_end_s);
    Index<NUM_PATH_CONTROL_POINTS> index;
    while (index.advance()) {
      const auto &sample_info = input_.path_segments[index.segment_index]
                                    .quad_point_info[index.quad_index];
      double sample_s = sample_info.sample_s - input_.planning_init_state.s;
      if (sample_s < 0)
        continue;
      if (sample_s > desire_end_s)
        break;

      double l = quintic_poly.Evaluate(0, sample_s);
      double dl = quintic_poly.Evaluate(1, sample_s);
      double ddl = quintic_poly.Evaluate(2, sample_s);
      double dddl = quintic_poly.Evaluate(3, sample_s);
      double jerk =
          std::pow(std::max(sample_info.speed_plan.v, 3.), 3) * dddl +
          2 * sample_info.speed_plan.v * sample_info.speed_plan.a * ddl;
      max_jerk = std::max(max_jerk, std::abs(jerk));
      max_dddl = std::max(max_dddl, std::abs(dddl));

      if (index.segment_index < front_obs_from_curr_lane.polygon_list.size() &&
          index.quad_index <
              front_obs_from_curr_lane.polygon_list[index.segment_index]
                  .size()) {
        const auto &pred =
            front_obs_from_curr_lane
                .polygon_list[index.segment_index][index.quad_index];
        if (pred.polygon_fren.size() > 1) {
          using namespace msquare::planning_math;
          Vec2d self_postion(sample_info.sample_s, l);
          Box2d self_box(self_postion, std::atan2(dl, 1.),
                         input_.vehicle_param.length,
                         input_.vehicle_param.width);
          self_box.LongitudinalExtend(4.0);
          self_box.LateralExtend(1.0);
          for (int i = 0; i < pred.polygon_fren.size() - 1; i++) {
            Vec2d start(pred.polygon_fren[i].x, pred.polygon_fren[i].y);
            Vec2d end(pred.polygon_fren[i + 1].x, pred.polygon_fren[i + 1].y);
            LineSegment2d obs_edge(start, end);

            std::string nudge_side = "none";
            if (front_obs_from_curr_lane.nudge_side ==
                ObsInfo::NudgeType::LEFT_NUDGE) {
              nudge_side = "left";
            } else if (front_obs_from_curr_lane.nudge_side ==
                       ObsInfo::NudgeType::RIGHT_NUDGE) {
              nudge_side = "right";
            }

            if (self_box.isCorrectLateralSide(obs_edge, nudge_side)) {
              double dist = self_box.DistanceTo(obs_edge);
              if (dist < 0.01) {
                collision = true;
              }
            } else {
              collision = true;
            }
          }
        }
      }
    }

    if (max_jerk > 2.0 || max_dddl > 0.05)
      break;
    desire_end_s -= std::max(input_.planning_init_state.v * 0.3, 0.4);
  }
}

void PathPlannerDecider::update_obstacle_bound() {
  left_aim_ = {};
  right_aim_ = {};

  double vel_lat_init =
      input_.planning_init_state.dl * input_.planning_init_state.v;
  double rear_trigger_lon_dist = 0.;
  double intersec_lat_comp = 0.;
  if (input_.intersec_info.is_in_intersection && input_.lat_safety_improved) {
    rear_trigger_lon_dist = half_self_length_;
    intersec_lat_comp = 0.2;
  }

  for (const auto &obs : input_.obs_list) {
    if (input_.lc_decider_info.lc_status !=
            path_planner::LaneChangeInfo::LaneKeep &&
        obs.id != input_.lc_decider_info.origin_lane_front_obs_id) {
      continue;
    }

    // set lat expansion buffer
    bool is_overtake = false;
    bool is_VRU = false;
    double press_lane_border_buffer = 0.;
    if (input_.lat_safety_improved) {
      is_overtake = judge_overtake(obs);
      is_VRU = (obs.type == ObsInfo::Type::PEDESTRIAN ||
                obs.type == ObsInfo::Type::OFO);
      press_lane_border_buffer =
          compute_press_lane_border_buffer(obs.distance_to_lane_line_history);
    }

    double base_buffer = is_VRU ? AVD_WIDTH_DESIRE_VRU : AVD_WIDTH_DESIRE;
    double desire_buffer =
        half_self_width_ + base_buffer + intersec_lat_comp +
        press_lane_border_buffer +
        input_.planning_init_state.v * AVD_WIDTH_DESIRE_INCREASE_RATE_VS_VEL;

    // limit avd buffer for rear obstacles
    double max_l = std::numeric_limits<double>::max();
    double min_l = std::numeric_limits<double>::lowest();
    const auto init_polygon = obs.polygon_init.polygon_fren;
    if (!init_polygon.empty() &&
        input_.planning_init_state.s + half_self_length_ >
            init_polygon.back().x + rear_trigger_lon_dist) {
      if (obs.nudge_side == ObsInfo::NudgeType::LEFT_NUDGE)
        max_l = input_.planning_init_state.l - desire_buffer;
      else
        min_l = input_.planning_init_state.l + desire_buffer;
    }

    // set lon expansion buffer
    double deta_v = input_.planning_init_state.v - obs.polygon_init.v_frenet;
    double lon_buffer_base_ori =
        LON_INFLATION_BASE + std::min(std::max(0., deta_v * 0.5), 1.);
    double lon_buffer_base_safe =
        LON_INFLATION_BASE +
        std::min(10., std::max(0., input_.planning_init_state.v * 0.2 +
                                       deta_v * 0.5));

    // cal avd passage when lon overlap
    double pred_valid_time = prediction_valid_time(obs.polygon_init.v);
    const double temp_lead_one_buffer = 2.0;
    if (obs.id == input_.lc_decider_info.origin_lane_front_obs_id) {
      pred_valid_time += temp_lead_one_buffer;
    } else {
      const double press_lane_border_time_buffer = 1.0;
      if (is_overtake &&
          press_lane_border_buffer > PRESS_LANE_AVD_BUFFER_MAX / 2.0) {
        pred_valid_time += press_lane_border_time_buffer;
      }
    }

    Index<NUM_PATH_CONTROL_POINTS> index;
    while (index.advance()) {
      if (obs.polygon_list.size() <= index.segment_index ||
          obs.polygon_list[index.segment_index].size() <= index.quad_index)
        break;
      const auto &pred =
          obs.polygon_list[index.segment_index][index.quad_index];
      if (pred.polygon_fren.empty())
        continue;

      const auto &quad_point_info = input_.path_segments[index.segment_index]
                                        .quad_point_info[index.quad_index];
      if (quad_point_info.refline_info.time > pred_valid_time)
        break;

      double obs_l = 0;
      double lon_buffer = input_.lat_safety_improved ? 0. : lon_buffer_base_ori;
      bool overlap = cal_ref_offset(index.segment_index, index.quad_index,
                                    pred.polygon_fren, obs.nudge_side, max_l,
                                    min_l, lon_buffer, obs_l);

      bool infl_overlap = false;
      if (!overlap && input_.lat_safety_improved) {
        infl_overlap = cal_ref_offset(index.segment_index, index.quad_index,
                                      pred.polygon_fren, obs.nudge_side, max_l,
                                      min_l, lon_buffer_base_safe, obs_l);
        if (infl_overlap && deta_v < -0.1 * input_.planning_init_state.v &&
            input_.planning_init_state.s + half_self_length_ <
                init_polygon.front().x) {
          if (obs.nudge_side == ObsInfo::NudgeType::LEFT_NUDGE) {
            obs_l =
                std::min(obs_l, input_.planning_init_state.l - desire_buffer);
          } else {
            obs_l =
                std::max(obs_l, input_.planning_init_state.l + desire_buffer);
          }
        }
      }
      if (overlap || infl_overlap) {
        auto &sample_decision =
            decision_info_[index.segment_index][index.quad_index];
        const double &sample_time = quad_point_info.refline_info.time;
        if (obs.nudge_side == ObsInfo::NudgeType::LEFT_NUDGE) {
          if (obs_l + half_self_width_ > sample_decision.l_min) {
            sample_decision.l_min = obs_l + half_self_width_;
            sample_decision.right_obs_desire = obs_l + desire_buffer;
            sample_decision.right_obs_constrain =
                sample_decision.l_min + AVD_WIDTH_CONSTRAIN;
          }

          double desire_l = obs_l + desire_buffer;
          double deta_l = desire_l - input_.planning_init_state.l;
          double a = (deta_l - vel_lat_init * sample_time) * 2 /
                     std::pow(sample_time, 2);
          if (left_aim_.empty() ||
              (a > left_aim_.a &&
               desire_l > sample_decision.right_obs_inflation)) {
            left_aim_.id = obs.id;
            left_aim_.type = int(obs.type);
            left_aim_.avd_dir = "left";
            left_aim_.a = a;
            left_aim_.max_l =
                (is_overtake && input_.lat_safety_improved) ? max_l : obs_l;
            left_aim_.min_l = min_l;
            left_aim_.polygon = pred.polygon_fren;
            left_aim_.desire_buffer = desire_buffer;
            left_aim_.lane_border_base_l = sample_decision.right_obs_inflation;
            left_aim_.distance_to_lane_line_history =
                obs.distance_to_lane_line_history;
          }
        } else {
          if (obs_l - half_self_width_ < sample_decision.l_max) {
            sample_decision.l_max = obs_l - half_self_width_;
            sample_decision.left_obs_desire = obs_l - desire_buffer;
            sample_decision.left_obs_constrain =
                sample_decision.l_max - AVD_WIDTH_CONSTRAIN;
          }

          double desire_l = obs_l - desire_buffer;
          double deta_l = desire_l - input_.planning_init_state.l;
          double a = (deta_l - vel_lat_init * sample_time) * 2 /
                     std::pow(sample_time, 2);
          if (right_aim_.empty() ||
              (a < right_aim_.a &&
               desire_l < sample_decision.left_obs_inflation)) {
            right_aim_.id = obs.id;
            right_aim_.type = int(obs.type);
            right_aim_.avd_dir = "right";
            right_aim_.a = a;
            right_aim_.max_l = max_l;
            right_aim_.min_l =
                (is_overtake && input_.lat_safety_improved) ? min_l : obs_l;
            right_aim_.polygon = pred.polygon_fren;
            right_aim_.desire_buffer = desire_buffer;
            right_aim_.lane_border_base_l = sample_decision.left_obs_inflation;
            right_aim_.distance_to_lane_line_history =
                obs.distance_to_lane_line_history;
          }
        }
      }
    }
  }
}

void PathPlannerDecider::update_desire_bound() {
  const auto &left_polygon = left_aim_.polygon;
  const auto &right_polygon = right_aim_.polygon;

  double dddl =
      compute_obstacle_inflation_jerk(left_aim_.distance_to_lane_line_history) /
      std::pow(std::max(input_.planning_init_state.v, 0.1), 3);
  if (!left_polygon.empty()) {
    for (int i = 0; i < left_polygon.size(); ++i) {
      double desire_l = std::min(left_polygon[i].y, left_aim_.max_l) +
                        left_aim_.desire_buffer;
      if (i == 0 || left_polygon[i].y > left_polygon[i - 1].y) {
        update_tail_inflation(left_polygon[i].x, desire_l,
                              left_aim_.lane_border_base_l, dddl, "right");
      }

      if (i == left_polygon.size() - 1 ||
          left_polygon[i].y > left_polygon[i + 1].y) {
        bool right_avd_concerned =
            !right_polygon.empty() &&
            left_polygon[i].x < right_aim_.min_y_point().x &&
            left_polygon[i].y > right_aim_.min_y_point().y;
        update_head_inflation(left_polygon[i].x, desire_l,
                              left_aim_.lane_border_base_l, -dddl,
                              right_avd_concerned, "right");
      }
    }
  }

  dddl = compute_obstacle_inflation_jerk(
             right_aim_.distance_to_lane_line_history) /
         std::pow(std::max(input_.planning_init_state.v, 0.1), 3);
  if (!right_polygon.empty()) {
    for (int i = 0; i < right_polygon.size(); ++i) {
      double desire_l = std::max(right_polygon[i].y, right_aim_.min_l) -
                        right_aim_.desire_buffer;
      if (i == 0 || right_polygon[i].y < right_polygon[i - 1].y) {
        update_tail_inflation(right_polygon[i].x, desire_l,
                              right_aim_.lane_border_base_l, -dddl, "left");
      }

      if (i == right_polygon.size() - 1 ||
          right_polygon[i].y < right_polygon[i + 1].y) {
        bool left_avd_concerned =
            !left_polygon.empty() &&
            right_polygon[i].x < left_aim_.max_y_point().x &&
            right_polygon[i].y < left_aim_.max_y_point().y;
        update_head_inflation(right_polygon[i].x, desire_l,
                              right_aim_.lane_border_base_l, dddl,
                              left_avd_concerned, "left");
      }
    }
  }
}

void PathPlannerDecider::update_tail_inflation(const double &tail_s,
                                               const double &desire_l,
                                               const double &lane_border_base_l,
                                               const double &dddl,
                                               const std::string &direction) {
  if ((direction == "right" && desire_l > 0) ||
      (direction == "left" && desire_l < 0)) {
    msquare::planning_math::QuinticPoly1d quintic_poly(0., desire_l, dddl);
    double s_length = quintic_poly.get_end_s();
    update_desire_bound_quintic_poly(&quintic_poly, tail_s - s_length, tail_s,
                                     direction);
  } else if ((direction == "right" && desire_l > lane_border_base_l) ||
             (direction == "left" && desire_l < lane_border_base_l)) {
    msquare::planning_math::QuinticPoly1d quintic_poly(lane_border_base_l,
                                                       desire_l, dddl);
    double s_length = quintic_poly.get_end_s();
    update_desire_bound_quintic_poly(&quintic_poly, tail_s - s_length, tail_s,
                                     direction);
  }
}

void PathPlannerDecider::update_head_inflation(
    const double &head_s, const double &desire_l,
    const double &lane_border_base_l, const double &dddl,
    const bool &reverse_avd_concerned, const std::string &direction) {
  if ((direction == "right" && desire_l > 0) ||
      (direction == "left" && desire_l < 0)) {
    if (reverse_avd_concerned) {
      msquare::planning_math::QuinticPoly1d quintic_poly(desire_l, 0, dddl);
      double s_length = quintic_poly.get_end_s();
      update_desire_bound_quintic_poly(&quintic_poly, head_s, head_s + s_length,
                                       direction);
    } else {
      update_desire_bound_const_l(head_s, desire_l, direction);
    }
  } else if (reverse_avd_concerned) {
    // msquare::planning_math::QuinticPoly1d quintic_poly(
    //     desire_l, left_aim_.lane_border_base_l, dddl);
    // double s_length = quintic_poly.get_end_s();
    // update_desire_bound_quintic_poly(&quintic_poly, head_s, head_s +
    // s_length,
    //                                  "right");
    // hotfix:
    if ((direction == "right" && desire_l > lane_border_base_l) ||
        (direction == "left" && desire_l < lane_border_base_l)) {
      msquare::planning_math::QuinticPoly1d quintic_poly(
          desire_l, lane_border_base_l, dddl);
      double s_length = quintic_poly.get_end_s();
      update_desire_bound_quintic_poly(&quintic_poly, head_s, head_s + s_length,
                                       direction);
    }
  }
}

double PathPlannerDecider::compute_obstacle_inflation_jerk(
    const std::vector<double> &distance_to_lane_line_history) {
  double obstacle_inflation_jerk{};
  int history_position_size =
      std::min(int(distance_to_lane_line_history.size()),
               int(OBSTACLE_HISTORY_INFO_SIZE));

  if (distance_to_lane_line_history.size() < OBSTACLE_HISTORY_INFO_SIZE) {
    obstacle_inflation_jerk =
        INFLATION_JERK_MAX - (INFLATION_JERK_MAX - INFLATION_JERK_MIN) /
                                 OBSTACLE_HISTORY_INFO_SIZE *
                                 history_position_size;
  } else {
    double avarge_distance_to_lane_line = 0.0;
    for (const auto &distance : distance_to_lane_line_history) {
      avarge_distance_to_lane_line += distance;
    }
    avarge_distance_to_lane_line /= distance_to_lane_line_history.size();
    avarge_distance_to_lane_line =
        std::fmax(-OBSTACLE_CONFIDENCE,
                  std::fmin(OBSTACLE_CONFIDENCE, avarge_distance_to_lane_line));

    obstacle_inflation_jerk =
        INFLATION_JERK_MIN +
        (INFLATION_JERK_MAX - INFLATION_JERK_MIN) / (2 * OBSTACLE_CONFIDENCE) *
            (avarge_distance_to_lane_line - (-OBSTACLE_CONFIDENCE));
  }

  obstacle_inflation_jerk =
      std::fmin(std::fmax(obstacle_inflation_jerk, INFLATION_JERK_MIN),
                INFLATION_JERK_MAX);

  return obstacle_inflation_jerk;
}

void PathPlannerDecider::update_desire_bound_quintic_poly(
    const msquare::planning_math::QuinticPoly1d *quintic_poly,
    const double &s_start, const double &s_end, const std::string &direction) {
  Index<NUM_PATH_CONTROL_POINTS> index;
  while (index.advance()) {
    const auto &quad_point_info = input_.path_segments[index.segment_index]
                                      .quad_point_info[index.quad_index];
    double sample_s = quad_point_info.sample_s - s_start;
    if (sample_s >= 0 && sample_s <= s_end - s_start) {
      double bound = quintic_poly->Evaluate(0, sample_s);
      auto &decision_info =
          decision_info_[index.segment_index][index.quad_index];
      if (direction == "right") {
        decision_info.right_obs_inflation =
            std::min(std::max(decision_info.right_obs_inflation, bound),
                     quad_point_info.refline_info.left_lane_border -
                         half_self_width_ - LANE_BORDER_AVD_DIST_M);
      } else if (direction == "left") {
        decision_info.left_obs_inflation =
            std::max(std::min(decision_info.left_obs_inflation, bound),
                     -quad_point_info.refline_info.right_lane_border +
                         half_self_width_ + LANE_BORDER_AVD_DIST_M);
      }
    } else if (sample_s > s_end) {
      break;
    }
  }
}

void PathPlannerDecider::update_desire_bound_const_l(
    const double &s_start, const double &const_l,
    const std::string &direction) {
  Index<NUM_PATH_CONTROL_POINTS> index;
  while (index.advance()) {
    const auto &quad_point_info = input_.path_segments[index.segment_index]
                                      .quad_point_info[index.quad_index];
    double sample_s = quad_point_info.sample_s;
    if (sample_s >= s_start) {
      auto &decision_info =
          decision_info_[index.segment_index][index.quad_index];
      if (direction == "right") {
        decision_info.right_obs_inflation =
            std::min(std::max(decision_info.right_obs_inflation, const_l),
                     quad_point_info.refline_info.left_lane_border -
                         half_self_width_ - LANE_BORDER_AVD_DIST_M);
      } else if (direction == "left") {
        decision_info.left_obs_inflation =
            std::max(std::min(decision_info.left_obs_inflation, const_l),
                     -quad_point_info.refline_info.right_lane_border +
                         half_self_width_ + LANE_BORDER_AVD_DIST_M);
      }
    }
  }
}

void PathPlannerDecider::smooth_bound() {
  double left_target_l =
      input_.planning_init_state.sample_info.refline_info.left_lane_border -
      half_self_width_ - LANE_BORDER_AVD_DIST_M;
  double right_target_l =
      -input_.planning_init_state.sample_info.refline_info.right_lane_border +
      half_self_width_ + LANE_BORDER_AVD_DIST_M;

  const double jerk = input_.lat_safety_improved ? 1.5 : 1.0;
  double dddl = jerk / std::pow(std::max(input_.planning_init_state.v, 1.0), 3);
  // left smoothing
  double end_s_right = 0.;
  bool get_opt_quintic_poly_right = false;
  msquare::planning_math::QuinticPoly1d quintic_poly_right;
  if (input_.planning_init_state.l < left_target_l) {
    get_opt_quintic_poly_right = ActiveAvoid::optimize_quintic_poly(
        input_, left_target_l, dddl, "left", quintic_poly_right, end_s_right);
  }
  // right smoothing
  double end_s_left = 0.;
  bool get_opt_quintic_poly_left = false;
  msquare::planning_math::QuinticPoly1d quintic_poly_left;
  if (input_.planning_init_state.l > right_target_l) {
    get_opt_quintic_poly_left = ActiveAvoid::optimize_quintic_poly(
        input_, right_target_l, -dddl, "right", quintic_poly_left, end_s_left);
  }

  Index<NUM_PATH_CONTROL_POINTS> index;
  while (index.advance()) {
    auto &decision_info = decision_info_[index.segment_index][index.quad_index];
    const auto &quad_point_info = input_.path_segments[index.segment_index]
                                      .quad_point_info[index.quad_index];
    double sample_s = quad_point_info.sample_s - input_.planning_init_state.s;
    double max_l = std::min(quad_point_info.refline_info.left_lane_border -
                                half_self_width_ - 0.3,
                            decision_info.left_map_hard_desire);
    double l_right = max_l;
    if (sample_s < end_s_right && get_opt_quintic_poly_right)
      l_right = std::min(quintic_poly_right.Evaluate(0, sample_s), max_l);
    decision_info.right_obs_constrain =
        std::min(decision_info.right_obs_constrain, l_right);
    decision_info.right_obs_desire =
        std::min(decision_info.right_obs_desire, l_right);
    decision_info.right_obs_inflation =
        std::min(decision_info.right_obs_inflation, l_right);

    double min_l = std::max(-quad_point_info.refline_info.right_lane_border +
                                half_self_width_ + 0.3,
                            decision_info.right_map_hard_desire);
    double l_left = min_l;
    if (sample_s < end_s_left && get_opt_quintic_poly_left)
      l_left = std::max(quintic_poly_left.Evaluate(0, sample_s), min_l);
    decision_info.left_obs_constrain =
        std::max(decision_info.left_obs_constrain, l_left);
    decision_info.left_obs_desire =
        std::max(decision_info.left_obs_desire, l_left);
    decision_info.left_obs_inflation =
        std::max(decision_info.left_obs_inflation, l_left);
  }
}

void PathPlannerDecider::slash_activitation_dist() {
  Index<NUM_PATH_CONTROL_POINTS> index;
  while (index.advance()) {
    auto &decision_info = decision_info_[index.segment_index][index.quad_index];
    double left_obs_avd = std::min(decision_info.left_obs_inflation,
                                   decision_info.left_obs_desire);
    double right_obs_avd = std::max(decision_info.right_obs_inflation,
                                    decision_info.right_obs_desire);
    decision_info.left_activation_dist =
        std::max(decision_info.left_activation_dist, right_obs_avd);
    decision_info.right_activation_dist =
        std::min(decision_info.right_activation_dist, left_obs_avd);
    if (left_obs_avd > right_obs_avd) {
      decision_info.ref_offset = std::min(
          left_obs_avd, std::max(decision_info.ref_offset, right_obs_avd));
    } else {
      decision_info.ref_offset = (left_obs_avd + right_obs_avd) / 2.0;
    }

    if (decision_info.left_map_hard_desire >
        decision_info.right_map_hard_desire) {
      decision_info.left_activation_dist =
          std::min(decision_info.left_map_hard_desire,
                   std::max(decision_info.left_activation_dist,
                            decision_info.right_map_hard_desire));
      decision_info.right_activation_dist =
          std::max(decision_info.right_map_hard_desire,
                   std::min(decision_info.right_activation_dist,
                            decision_info.left_map_hard_desire));
      decision_info.ref_offset =
          std::min(decision_info.left_map_hard_desire,
                   std::max(decision_info.right_map_hard_desire,
                            decision_info.ref_offset));
    } else {
      decision_info.ref_offset = (decision_info.right_map_hard_desire +
                                  decision_info.left_map_hard_desire) /
                                 2.;
      decision_info.left_activation_dist = decision_info.ref_offset;
      decision_info.right_activation_dist = decision_info.ref_offset;
      decision_info.left_map_hard_desire = decision_info.ref_offset;
      decision_info.right_map_hard_desire = decision_info.ref_offset;
    }
  }
}

double PathPlannerDecider::compute_press_lane_border_buffer(
    const std::vector<double> &distance_to_lane_line_history) {
  const std::vector<double> PRESS_LANE_COUNT{
      0., static_cast<double>(OBSTACLE_HISTORY_INFO_SIZE)};
  std::vector<double> PRESS_LANE_AVD_BUFFER{0., PRESS_LANE_AVD_BUFFER_MAX};
  double press_count = 0;
  for (const auto &dist : distance_to_lane_line_history) {
    if (dist < 0) {
      press_count++;
    }
  }
  return msquare::planning_math::LinearInterpation::interpolation(
      press_count, PRESS_LANE_COUNT, PRESS_LANE_AVD_BUFFER);
}

bool PathPlannerDecider::judge_overtake(const ObsInfo &obstacle) {
  if (obstacle.polygon_init.polygon_fren.empty()) {
    return false;
  } else if (input_.planning_init_state.s + half_self_length_ <
             obstacle.polygon_init.polygon_fren.back().x) {
    double max_lon_time =
        input_.path_segments.back().quad_point_info.back().refline_info.time;
    int seg_max_index =
        std::min(input_.path_segments.size(), obstacle.polygon_list.size());
    bool end_point_judge = false;
    for (int seg_idx = seg_max_index - 1; seg_idx >= 0; seg_idx--) {
      int quad_max_index =
          std::min(obstacle.polygon_list[seg_idx].size(),
                   input_.path_segments[seg_idx].quad_point_info.size());
      for (int quad_idx = quad_max_index - 1; quad_idx >= 0; quad_idx--) {
        double t = input_.path_segments[seg_idx]
                       .quad_point_info[quad_idx]
                       .refline_info.time;
        if (t < max_lon_time - 0.1) {
          double self_front_s =
              input_.path_segments[seg_idx].quad_point_info[quad_idx].sample_s +
              half_self_length_;
          double obstacle_front_s =
              obstacle.polygon_list[seg_idx][quad_idx].polygon_fren.back().x;
          if (self_front_s > obstacle_front_s + 0.5) {
            return true;
          } else {
            end_point_judge = true;
            break;
          }
        }

        if (end_point_judge)
          break;
      }
    }
  }

  return false;
}

bool PathPlannerDecider::cal_ref_offset(
    const size_t &segment_index, const size_t &quad_index,
    const std::vector<Point2d> &polygon, const ObsInfo::NudgeType &nudge_side,
    const double &max_l, const double &min_l, const double &lon_buffer_base,
    double &ref_offset) {
  using namespace msquare::planning_math;
  double sample_s =
      input_.path_segments[segment_index].quad_point_info[quad_index].sample_s;
  double last_s = sample_s;
  double next_s = sample_s;

  const double &sample_time = input_.path_segments[segment_index]
                                  .quad_point_info[quad_index]
                                  .refline_info.time;
  const double &delta_s =
      polygon.front().x - input_.planning_init_state.s - half_self_length_;
  double time = std::min(sample_time, delta_s / input_.planning_init_state.v);
  double lon_buffer =
      lon_buffer_base - lon_buffer_base / LON_INFLATION_TIME_LIMIT * time;
  lon_buffer = std::max(lon_buffer, 0.);

  if (quad_index > 0) {
    last_s = input_.path_segments[segment_index]
                 .quad_point_info[quad_index - 1]
                 .sample_s;
  } else if (segment_index > 0) {
    last_s = input_.path_segments[segment_index - 1]
                 .quad_point_info[QUADRATURE_ORDER - 1]
                 .sample_s;
  }

  if (quad_index < QUADRATURE_ORDER - 1) {
    next_s = input_.path_segments[segment_index]
                 .quad_point_info[quad_index + 1]
                 .sample_s;
  } else if (segment_index < NUM_PATH_SEGMENTS - 1) {
    next_s =
        input_.path_segments[segment_index + 1].quad_point_info[0].sample_s;
  }

  double front_s = std::max(next_s, sample_s + half_self_length_ + lon_buffer);
  double rear_s = std::min(last_s, sample_s - half_self_length_);

  InterpolationData<Point2d> interpolate_data;
  for (const auto &point : polygon) {
    interpolate_data.emplace_back(point.x, point);
  }

  std::vector<double> s_list{rear_s, sample_s, front_s};
  std::multiset<double> l_list;
  for (const auto &s : s_list) {
    if (s > polygon.front().x && s < polygon.back().x) {
      auto interpolate_point =
          LinearInterpation::interpolate<Point2d>(interpolate_data, s);
      l_list.insert(interpolate_point.y);
    }
  }

  for (const auto &p : polygon) {
    if (p.x > rear_s && p.x < front_s)
      l_list.insert(p.y);
  }

  if (l_list.empty()) {
    return false;
  } else {
    if (nudge_side == ObsInfo::NudgeType::LEFT_NUDGE) {
      auto it = l_list.end();
      ref_offset = *(--it);
      ref_offset = std::min(max_l, ref_offset);
    } else {
      ref_offset = *l_list.begin();
      ref_offset = std::max(min_l, ref_offset);
    }
    return true;
  }
}

double PathPlannerDecider::prediction_valid_time(const double &v) {
  if (v < key_params_.pred_vilad_time_max.first) {
    return key_params_.pred_vilad_time_max.second;
  } else if (v > key_params_.pred_vilad_time_min.first) {
    return key_params_.pred_vilad_time_min.second;
  } else {
    double k = (key_params_.pred_vilad_time_min.second -
                key_params_.pred_vilad_time_max.second) /
               (key_params_.pred_vilad_time_min.first -
                key_params_.pred_vilad_time_max.first);
    return key_params_.pred_vilad_time_max.second +
           (v - key_params_.pred_vilad_time_max.first) * k;
  }
}

bool PathPlannerDecider::current_lane_end_distinguish() {
  int count_road_border = 0;
  const double scenario_hard_buffer_base = 0.4;
  const double decay_rate = 0.05;
  Index<NUM_PATH_CONTROL_POINTS> index;
  while (index.advance()) {
    const auto &sample_info = input_.path_segments[index.segment_index]
                                  .quad_point_info[index.quad_index];
    if (sample_info.refline_info.time > 3)
      break;

    double scenario_hard_buffer =
        scenario_hard_buffer_base - sample_info.refline_info.time * decay_rate;
    const auto &map_info = sample_info.refline_info;
    if (map_info.left_lane_border + map_info.right_road_border <
            2 * half_self_width_ + scenario_hard_buffer ||
        map_info.right_lane_border + map_info.left_road_border <
            2 * half_self_width_ + scenario_hard_buffer)
      count_road_border++;
    if (count_road_border > 2) {
      return true;
    }
  }
  return false;
}

double PathPlannerDecider::cal_soft_road_border_buffer(
    const PathSampleInfo &sample_info, const std::string &direction,
    const bool is_merge, const double hard_buffer) {
  const auto &map_info = sample_info.refline_info;
  double soft_base_buffer = ROAD_BORDER_AVD_DIST_SOFT + half_self_width_;

  const double max_speed_buffer = 0.5;
  const double lat_acc = 0.8;
  double v = sample_info.speed_plan.v;
  double vel_lat_init = 0;
  if ((input_.planning_init_state.dl > 0 && direction == "left") ||
      (input_.planning_init_state.dl < 0 && direction == "right")) {
    vel_lat_init = input_.planning_init_state.dl * v;
  }
  double speed_buffer =
      std::min(v * ROAD_BORDER_AVD_SPEED_COEF +
                   std::max(vel_lat_init * vel_lat_init / lat_acc / 2., 0.),
               max_speed_buffer);

  double curv_buffer = 0;
  const double narrow_lane_width = 3.4;
  double current_lane_width =
      map_info.left_lane_border + map_info.right_lane_border;
  double max_curv_buffer = (current_lane_width - narrow_lane_width) * 0.5;
  max_curv_buffer = std::max(std::min(max_curv_buffer, 0.3), 0.);
  if ((direction == "left" && map_info.curvature < 0.) ||
      (direction == "right" && map_info.curvature > 0.)) {
    curv_buffer = std::fmin(v * v * std::fabs(map_info.curvature) *
                                ROAD_BORDER_AVD_LAT_ACCEL_COEF,
                            max_curv_buffer);
  }

  double decay_buffer = 0;
  const double decay_rate = 0.05;
  if (sample_info.sample_s - input_.planning_init_state.s > 10.)
    decay_buffer = map_info.time * decay_rate;

  double soft_buffer =
      soft_base_buffer + curv_buffer + speed_buffer - decay_buffer;

  const double min_to_lane_dist = 0.3;
  if (!is_merge && direction == "left") {
    soft_buffer =
        std::max(std::min(soft_buffer, map_info.left_road_border +
                                           map_info.right_lane_border -
                                           half_self_width_ - min_to_lane_dist),
                 hard_buffer);
  } else if (!is_merge && direction == "right") {
    soft_buffer =
        std::max(std::min(soft_buffer, map_info.right_road_border +
                                           map_info.left_lane_border -
                                           half_self_width_ - min_to_lane_dist),
                 hard_buffer);
  }

  return soft_buffer;
}

double PathPlannerDecider::cal_hard_road_border_buffer(
    const PathSampleInfo &sample_info, const std::string &direction) {
  double hard_base_buffer = ROAD_BORDER_AVD_DIST_HARD + half_self_width_;

  const double max_speed_buffer = 0.3;
  const double lat_acc = 1.0;
  double v = sample_info.speed_plan.v;
  double vel_lat_init = 0;
  if ((input_.planning_init_state.dl > 0 && direction == "left") ||
      (input_.planning_init_state.dl < 0 && direction == "right")) {
    vel_lat_init = input_.planning_init_state.dl * v;
  }
  double speed_buffer =
      std::min(v * ROAD_BORDER_AVD_SPEED_COEF / 2.0 +
                   std::max(vel_lat_init * vel_lat_init / lat_acc / 2., 0.),
               max_speed_buffer);

  double decay_buffer = 0;
  const double decay_rate = 0.025;
  if (sample_info.sample_s - input_.planning_init_state.s > 10.)
    decay_buffer = sample_info.refline_info.time * decay_rate;

  return (hard_base_buffer + speed_buffer - decay_buffer);
}

} // namespace path_planner
