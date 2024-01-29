#include "lateral_active_avoid.hpp"

namespace path_planner {

ActiveAvoid::ActiveAvoid(const PathPlannerInput &input) {
  input_ = input;
  half_self_width_ = input_.vehicle_param.width / 2.;
  half_self_length_ = input_.vehicle_param.length / 2.;
}

void ActiveAvoid::select_trucks(std::vector<ObsInfo> &front_trucks,
                                std::vector<ObsInfo> &overlap_trucks) {
  // std::cout << "input_.obs_list.size(): " << input_.obs_list.size()
  //           << std::endl;
  // for (auto truck: input_.obs_list) {
  //   std::cout << truck.id << " ";
  // }
  // std::cout << std::endl;
  for (const auto &obs : input_.obs_list) {
    if (std::fabs(obs.polygon_init.heading - input_.ego_theta) > 1.57) {
      // std::cout << "continue obs " << obs.id << std::endl;
      continue;
    }
    if (!obs.polygon_init.polygon_fren.empty() &&
        (std::fabs(obs.polygon_init.polygon_fren.back().x -
                   obs.polygon_init.polygon_fren.front().x) < TruckLengthMin)) {
      continue;
    }
    if ((obs.type == ObsInfo::Type::TRANSPORT_TRUNK ||
         obs.type == ObsInfo::Type::BUS) &&
        !obs.polygon_init.polygon_fren.empty()) {
      // truck tail > ego head && truck_tail - ego_head <= 100
      if (obs.polygon_init.polygon_fren.front().x >
              input_.planning_init_state.s + half_self_length_ &&
          obs.polygon_init.polygon_fren.front().x -
                  input_.planning_init_state.s - half_self_length_ <=
              100.0) {
        front_trucks.emplace_back(obs);
        // truck tail <= ego head && truck head > ego tail
      } else if (obs.polygon_init.polygon_fren.back().x >
                 input_.planning_init_state.s - half_self_length_) {
        overlap_trucks.emplace_back(obs);
        front_trucks.emplace_back(obs);
      }
    }
  }

  sort(front_trucks.begin(), front_trucks.end(),
       [](const ObsInfo &obs1, const ObsInfo &obs2) {
         return obs1.polygon_init.s < obs2.polygon_init.s;
       });
}

void ActiveAvoid::update_decision_info(DecisionInfo &decision_info,
                                       const double &target_l,
                                       const double &target_s,
                                       const std::string &avd_direction) {
  msquare::planning_math::QuinticPoly1d quintic_poly;
  double end_s = 0.;
  const double jerk = 1.;
  double dddl = jerk / std::pow(std::max(input_.planning_init_state.v, 3.0), 3);
  if (input_.planning_init_state.l < target_l) {
    bool left_quintic_poly = optimize_quintic_poly(
        input_, target_l, dddl, "left", quintic_poly, end_s, target_s);
  } else {
    bool right_quintic_poly = optimize_quintic_poly(
        input_, target_l, -dddl, "right", quintic_poly, end_s, target_s);
  }

  Index<NUM_PATH_CONTROL_POINTS> index;
  while (index.advance()) {
    const auto &sample_info = input_.path_segments[index.segment_index]
                                  .quad_point_info[index.quad_index];
    auto &sample_decision_info =
        decision_info[index.segment_index][index.quad_index];
    double sample_s_relative =
        sample_info.sample_s - input_.planning_init_state.s;

    if (sample_s_relative > end_s) {
      sample_decision_info.ref_offset = target_l;
    } else {
      sample_decision_info.ref_offset =
          quintic_poly.Evaluate(0, sample_s_relative);
    }

    double map_limit_max_l =
        std::max(0., std::min(sample_info.refline_info.left_road_border -
                                  half_self_width_ - OffsetRoadBoundLimit,
                              sample_info.refline_info.left_lane_border -
                                  half_self_width_ - OffsetLaneBoundLimit));
    double map_limit_min_l =
        -std::max(0., std::min(sample_info.refline_info.right_lane_border -
                                   half_self_width_ - OffsetLaneBoundLimit,
                               sample_info.refline_info.right_road_border -
                                   half_self_width_ - OffsetRoadBoundLimit));
    sample_decision_info.ref_offset =
        std::max(std::min(sample_decision_info.ref_offset, map_limit_max_l),
                 map_limit_min_l);

    if (avd_direction == "left" && end_s > 0.1) {
      sample_decision_info.left_activation_dist =
          std::max(sample_decision_info.left_activation_dist,
                   std::min(sample_info.refline_info.left_lane_border -
                                half_self_width_ - 0.2,
                            sample_decision_info.ref_offset + 0.1));
      sample_decision_info.right_activation_dist =
          std::max(sample_decision_info.right_activation_dist,
                   std::min(target_l, sample_decision_info.ref_offset));
    } else if (avd_direction == "right" && end_s > 0.1) {
      sample_decision_info.right_activation_dist =
          std::min(sample_decision_info.right_activation_dist,
                   std::max(-sample_info.refline_info.right_lane_border +
                                half_self_width_ + 0.2,
                            sample_decision_info.ref_offset - 0.1));
      sample_decision_info.left_activation_dist =
          std::min(sample_decision_info.left_activation_dist,
                   std::max(target_l, sample_decision_info.ref_offset + 0.1));
    }
  }
}

void ActiveAvoid::update_decision_info_lateral_keeping(
    DecisionInfo &decision_info) {
  Index<NUM_PATH_CONTROL_POINTS> index;
  while (index.advance()) {
    const auto &sample_info = input_.path_segments[index.segment_index]
                                  .quad_point_info[index.quad_index];
    auto &sample_decision_info =
        decision_info[index.segment_index][index.quad_index];
    sample_decision_info.ref_offset = input_.planning_init_state.l;

    if (input_.planning_init_state.l > 0) {
      sample_decision_info.left_activation_dist =
          std::max(sample_decision_info.left_activation_dist,
                   std::min(sample_info.refline_info.left_lane_border -
                                half_self_width_ - 0.2,
                            sample_decision_info.ref_offset + 0.1));
      sample_decision_info.right_activation_dist =
          std::max(sample_decision_info.right_activation_dist,
                   sample_decision_info.ref_offset - 0.1);
    } else {
      sample_decision_info.right_activation_dist =
          std::min(sample_decision_info.right_activation_dist,
                   std::max(-sample_info.refline_info.right_lane_border +
                                half_self_width_ + 0.2,
                            sample_decision_info.ref_offset - 0.1));
      sample_decision_info.left_activation_dist =
          std::min(sample_decision_info.left_activation_dist,
                   sample_decision_info.ref_offset + 0.1);
    }
  }
}

bool ActiveAvoid::is_parallel_driving(const ObsInfo &obs) {
  int seg_max_index =
      std::min(input_.path_segments.size(), obs.polygon_list.size());
  bool end_point_judge = false;
  for (int seg_idx = seg_max_index - 1; seg_idx >= 0; seg_idx--) {
    int quad_max_index =
        std::min(obs.polygon_list[seg_idx].size(),
                 input_.path_segments[seg_idx].quad_point_info.size());
    for (int quad_idx = quad_max_index - 1; quad_idx >= 0; quad_idx--) {
      double t = input_.path_segments[seg_idx]
                     .quad_point_info[quad_idx]
                     .refline_info.time;
      if (t < 4.99 && t > 3) {
        end_point_judge = true;
        double self_s =
            input_.path_segments[seg_idx].quad_point_info[quad_idx].sample_s;
        double lon_dis =
            obs.polygon_list[seg_idx][quad_idx].polygon_fren.front().x - self_s;
        double obs_length = obs.polygon_init.polygon_fren.back().x -
                            obs.polygon_init.polygon_fren.front().x;
        if (lon_dis > obs_length / 5. * t - obs_length) {
          return false;
        }
        break;
      }
    }
    if (end_point_judge)
      break;
  }
  return true;
}

bool ActiveAvoid::front_truck_avd_target(
    const std::vector<ObsInfo> &front_trucks, double &target_s,
    double &target_l, std::string &avd_direction,
    std::unordered_map<int, double> &active_target_l) {
  for (const auto &near_truck : front_trucks) {
    // lon overlap
    double target_s_tmp = 1.0e6 + 1;
    double target_l_tmp = 0.;
    Index<NUM_PATH_CONTROL_POINTS> index;
    while (index.advance()) {
      const auto &sample_info = input_.path_segments[index.segment_index]
                                    .quad_point_info[index.quad_index];
      if (near_truck.polygon_list.size() > index.segment_index &&
          near_truck.polygon_list[index.segment_index].size() >
              index.quad_index) {
        double self_head = sample_info.sample_s + half_self_length_;
        double self_tail = sample_info.sample_s - half_self_length_;
        const auto &obs_pred_poly =
            near_truck.polygon_list[index.segment_index][index.quad_index]
                .polygon_fren;
        if (!obs_pred_poly.empty() && self_head > obs_pred_poly.front().x) {
          target_s_tmp =
              std::max(sample_info.sample_s - input_.planning_init_state.s,
                       input_.planning_init_state.v * 1.0);
          double max_l = -10.;
          double min_l = 10.;
          for (const auto p : near_truck.polygon_init.polygon_fren) {
            max_l = std::max(max_l, p.y);
            min_l = std::min(min_l, p.y);
          }
          if (near_truck.nudge_side == ObsInfo::NudgeType::LEFT_NUDGE) {
            double map_limit_l = std::max(
                0., std::min(sample_info.refline_info.left_lane_border -
                                 half_self_width_ - OffsetLaneBoundLimit,
                             sample_info.refline_info.left_road_border -
                                 half_self_width_ - OffsetRoadBoundLimit));
            double avd_max = std::min(TruckOffsetRef, map_limit_l);
            double offset_increase_rate =
                TruckOffsetRef / (StartOffsetLatDis - SaturationOffsetLatDis);
            target_l_tmp = std::min(
                std::max(0., (StartOffsetLatDis + max_l + half_self_width_) *
                                 offset_increase_rate),
                avd_max);
          } else {
            double map_limit_l = std::max(
                0., std::min(sample_info.refline_info.right_lane_border -
                                 half_self_width_ - OffsetLaneBoundLimit,
                             sample_info.refline_info.right_road_border -
                                 half_self_width_ - OffsetRoadBoundLimit));
            double avd_max = std::min(TruckOffsetRef, map_limit_l);
            double offset_increase_rate =
                TruckOffsetRef / (StartOffsetLatDis - SaturationOffsetLatDis);
            target_l_tmp = -std::min(
                std::max(0., (StartOffsetLatDis - min_l + half_self_width_) *
                                 offset_increase_rate),
                avd_max);
          }
          break;
        }
      }
    }

    if (target_s_tmp < 1.0e6) {
      if (target_s < 1.0e6 && target_l_tmp * target_l < 0) {
        target_s = 1.0e6 + 1.0;
        target_l = 0.0;
        avd_direction = "none";
        break;
      } else if (target_s < 1.0e6 &&
                 ((target_l_tmp > 0 && target_l_tmp > target_l + 0.1) ||
                  (target_l_tmp < 0 && target_l_tmp < target_l - 0.1))) {
        target_s = target_s_tmp;
        target_l = target_l_tmp;
        active_target_l[near_truck.id] = target_l_tmp;
        break;
      } else if (target_s > 1.0e6) {
        if (target_l_tmp > 1e-6) {
          target_s = target_s_tmp;
          target_l = target_l_tmp;
          avd_direction = "left";
          active_target_l[near_truck.id] = target_l;
        } else if (target_l_tmp < -1e-6) {
          target_s = target_s_tmp;
          target_l = target_l_tmp;
          avd_direction = "right";
          active_target_l[near_truck.id] = target_l;
        } else {
          target_s = target_s_tmp;
          target_l = 0.0;
          avd_direction = "none";
          active_target_l[near_truck.id] = target_l;
        }
      }
    }
  }
  if (target_s < 1.0e6) {
    return true;
  } else {
    active_target_l.clear();
    // std::cout << "front_truck_avd_target failed !!!!!" << std::endl;
    return false;
  }
}

bool ActiveAvoid::truck_avd_invalid(const std::vector<ObsInfo> &front_trucks) {
  // scenario 3.2 双侧大车纵向距离小于5+30m，不避让
  if (front_trucks.size() >= 2) {
    auto truck1 = front_trucks.at(0);
    ObsInfo truck2;
    bool avd_diff_exist = false;
    for (int i = 0; i < front_trucks.size(); i++) {
      if (truck1.nudge_side != front_trucks.at(i).nudge_side) {
        truck2 = front_trucks.at(i);
        avd_diff_exist = true;
        break;
      }
    }
    if (!avd_diff_exist)
      // std::cout << "only one side trucks" << std::endl;
      return false;

    if (truck1.nudge_side != truck2.nudge_side) {
      if (!truck1.polygon_init.polygon_fren.empty() &&
          !truck2.polygon_init.polygon_fren.empty()) {
        if ((truck1.polygon_init.polygon_fren.back().x <
                 truck2.polygon_init.polygon_fren.back().x &&
             std::abs(truck1.polygon_init.polygon_fren.back().x -
                      truck2.polygon_init.polygon_fren.front().x) < 35) ||
            (truck2.polygon_init.polygon_fren.back().x <
                 truck1.polygon_init.polygon_fren.back().x &&
             std::abs(truck2.polygon_init.polygon_fren.back().x -
                      truck1.polygon_init.polygon_fren.front().x) < 35)) {
          // std::cout << "two near trucks long dist < 35m" << std::endl;
          return true;
        }
      }
    }
  }

  return false;
}

bool ActiveAvoid::truck_avd_should_end(
    const std::vector<ObsInfo> &front_trucks,
    const Intelligent_Dodge_Info &pre_dodge_info) {
  // curvature limit
  if (std::abs(input_.planning_init_state.sample_info.refline_info.curvature) >
      EndoffsetCurvature) {
    return true;
  }
  // 1.scenario 3.3 双侧大车：躲避中自车头到第二辆车TTC<=3s(TBD)，闪躲结束
  if (front_trucks.size() >= 2) {
    auto truck1 = front_trucks.at(0);
    for (size_t i = 1; i < front_trucks.size(); ++i) {
      auto truck2 = front_trucks.at(i);
      if (truck1.nudge_side != truck2.nudge_side) {
        if (!truck2.polygon_init.polygon_fren.empty()) {
          double ttc2truck = -1.;
          ttc2truck =
              (truck2.polygon_init.polygon_fren.front().x -
               (input_.planning_init_state.s + half_self_length_)) /
              (std::abs(input_.planning_init_state.v - truck2.polygon_init.v) +
               1e-6);
          // std::cout << "ttc2truck: " << ttc2truck << std::endl;
          if (ttc2truck < 3.0 && ttc2truck > 0.0) {
            return true;
          }
          double dist2truck1 =
              truck1.polygon_init.polygon_fren.front().x -
              (input_.planning_init_state.s + half_self_length_);
          double dist2truck2 =
              truck2.polygon_init.polygon_fren.front().x -
              (input_.planning_init_state.s + half_self_length_);
          if (dist2truck1 < 0 || dist2truck2 < 0.0) {
            return true;
          }
        }
      }
    }
  }

  // scenario: ego surpass truck, should fulfill control time
  if (front_trucks.empty() && pre_dodge_info.elapsed_count >= CONTROL_TIME) {
    return true;
  }

  // 2. 闪躲过程中距离对侧的车道线/路沿距离 <= 0.2/0.5m
  Index<NUM_PATH_CONTROL_POINTS> index_avd_ing;
  while (index_avd_ing.advance()) {
    const auto &sample_info = input_.path_segments[index_avd_ing.segment_index]
                                  .quad_point_info[index_avd_ing.quad_index];
    // if (sample_info.refline_info.right_lane_border - half_self_width_ -
    // AVD_LB_LIMIT <
    //         0. ||
    //     sample_info.refline_info.left_lane_border - half_self_width_ -
    //     AVD_LB_LIMIT <
    //         0.) {
    //   return true;
    // }
    if (sample_info.refline_info.right_road_border - half_self_width_ -
                AVD_RB_LIMIT <
            0. ||
        sample_info.refline_info.left_road_border - half_self_width_ -
                AVD_RB_LIMIT <
            0.) {
      return true;
    }
  }

  // scenario: lat dist > 2.8m
  bool lat_dist_flag = false;
  for (const ObsInfo &obs : front_trucks) {
    double max_l = -10.;
    double min_l = 10.;
    for (const auto p : obs.polygon_init.polygon_fren) {
      max_l = std::max(max_l, p.y);
      min_l = std::min(min_l, p.y);
    }
    if (obs.nudge_side == ObsInfo::NudgeType::LEFT_NUDGE) {
      double lat_dist = input_.planning_init_state.l - max_l - half_self_width_;
      if (lat_dist > 0.0 && lat_dist < 2.8) {
        lat_dist_flag = true;
        break;
      }
    } else {
      double lat_dist = min_l - input_.planning_init_state.l - half_self_width_;
      if (lat_dist > 0.0 && lat_dist < 2.8) {
        lat_dist_flag = true;
        break;
      }
    }
  }
  if (!lat_dist_flag && pre_dodge_info.elapsed_count >= CONTROL_TIME)
    return true;

  return false;
}

bool ActiveAvoid::lateral_keeping(const std::vector<ObsInfo> &overlap_obs) {
  bool lateral_keep = false;
  for (const auto &obs : overlap_obs) {
    double max_l = -10.;
    double min_l = 10.;
    for (const auto p : obs.polygon_init.polygon_fren) {
      max_l = std::max(max_l, p.y);
      min_l = std::min(min_l, p.y);
    }
    if (obs.nudge_side == ObsInfo::NudgeType::LEFT_NUDGE &&
        -max_l - half_self_width_ < StartOffsetLatDis) {
      if (input_.planning_init_state.l > 0) {
        lateral_keep = true;
      } else {
        return false;
      }
    } else if (obs.nudge_side == ObsInfo::NudgeType::RIGHT_NUDGE &&
               min_l - half_self_width_ < StartOffsetLatDis) {
      if (input_.planning_init_state.l < 0) {
        lateral_keep = true;
      } else {
        return false;
      }
    }
  }
  return lateral_keep;
}

bool ActiveAvoid::optimize_quintic_poly(
    const PathPlannerInput &input, const double &target_l,
    const double &dddl_ref, const std::string direction,
    msquare::planning_math::QuinticPoly1d &quintic_poly_opt, double &end_s_opt,
    double end_s_ref) {
  bool get_opt_quintic_poly = false;
  msquare::planning_math::QuinticPoly1d quintic_poly_base(
      input.planning_init_state.l, target_l, dddl_ref);
  double end_s_base = quintic_poly_base.get_end_s();
  double step_s = std::fmax(input.planning_init_state.v, 3.0) * 0.4;

  double cost_min = std::numeric_limits<double>::max();
  double end_s = end_s_base - step_s * 10;
  for (; end_s < end_s_base + step_s * 10; end_s += step_s) {
    if (end_s < 0)
      continue;
    msquare::planning_math::QuinticPoly1d quintic_poly(
        input.planning_init_state.l, input.planning_init_state.dl,
        input.planning_init_state.ddl, target_l, 0., 0., end_s);
    double cost = 0.;
    int count = 0;
    Index<NUM_PATH_CONTROL_POINTS> index;
    while (index.advance()) {
      const auto &sample_s = input.path_segments[index.segment_index]
                                 .quad_point_info[index.quad_index]
                                 .sample_s -
                             input.planning_init_state.s;
      if (sample_s > end_s)
        break;

      double end_s_scale = 10.;
      cost += std::abs(end_s - end_s_ref) /
              std::max(input.planning_init_state.v, 1.0) * end_s_scale;

      double dddl = quintic_poly.Evaluate(3, sample_s);
      double jerk =
          std::pow(std::max(input.planning_init_state.v, 5.5), 3) * dddl;
      double jerk_scale = std::abs(dddl) < std::abs(dddl_ref) ? 0.1 : 100.;
      cost += jerk * jerk * jerk_scale;

      double acc = std::pow(input.planning_init_state.v, 2) *
                   quintic_poly.Evaluate(2, sample_s);
      double acc_scale = std::abs(acc) < 3.0 ? 0.1 : 50.;
      cost += acc * acc * acc_scale;

      double l = quintic_poly.Evaluate(0, sample_s);
      double overshot_scale = 10.;
      if (direction == "left") {
        if (l > target_l + 0.1) {
          cost += overshot_scale * (l - target_l - 0.1);
        }
        if (l < input.planning_init_state.l - 0.1) {
          cost += overshot_scale * (input.planning_init_state.l - 0.1 - l);
        }
      } else if (direction == "right") {
        if (l < target_l - 0.1) {
          cost += overshot_scale * (target_l - 0.1 - l);
        }
        if (l > input.planning_init_state.l + 0.1) {
          cost += overshot_scale * (l - input.planning_init_state.l - 0.1);
        }
      }
      ++count;
    }
    if (cost < cost_min && count > 0) {
      cost_min = cost;
      quintic_poly_opt = quintic_poly;
      end_s_opt = end_s;
      get_opt_quintic_poly = true;
    }
  }
  return get_opt_quintic_poly;
}

void ActiveAvoid::offset_active(
    DecisionInfo &decision_info,
    std::unordered_map<int, double> &active_target_l,
    Intelligent_Dodge_Info &pre_dodge_info) {
  // std::cout << std::endl;
  // std::cout << "_____________LAT_ACTIVE_AVOID_____________" << std::endl;
  // select front trucks
  std::vector<ObsInfo> front_trucks;
  std::vector<ObsInfo> overlap_trucks;
  select_trucks(front_trucks, overlap_trucks);
  // std::cout << "front_trucks.size():" << front_trucks.size() << std::endl;
  // for (auto truck : front_trucks) {
  //   std::cout << truck.id << " ";
  // }
  // clear last target l
  active_target_l.clear();
  // std::cout << "front_trucks.size():" << front_trucks.size() << std::endl;
  double target_s = 1.0e6 + 1;
  double target_l = 0.;
  std::string avd_direction = "none";

  size_t elapsed_count = pre_dodge_info.elapsed_count;
  State state = pre_dodge_info.state;

  switch (state) {
  case State::Idle:
    if (truck_avd_should_trigger(front_trucks, pre_dodge_info)) {
      if (front_truck_avd_target(front_trucks, target_s, target_l,
                                 avd_direction, active_target_l)) {
        state = State::Control;
        elapsed_count = 0;
        pre_dodge_info.end_debounce_count = 0;
        update_decision_info(decision_info, target_l, target_s, avd_direction);
      } else if (lateral_keeping(overlap_trucks)) {
        update_decision_info_lateral_keeping(decision_info);
      }
    }
    break;

  case State::CoolingDown:
    // CoolingDown over
    if (elapsed_count >= COOLING_TIME) {
      if (truck_avd_should_trigger(front_trucks, pre_dodge_info)) {
        if (front_truck_avd_target(front_trucks, target_s, target_l,
                                   avd_direction, active_target_l)) {
          state = State::Control;
          elapsed_count = 0;
          pre_dodge_info.end_debounce_count = 0;
          update_decision_info(decision_info, target_l, target_s,
                               avd_direction);
        } else {
          state = State::Idle;
          elapsed_count = 0;
          if (lateral_keeping(overlap_trucks)) {
            update_decision_info_lateral_keeping(decision_info);
          }
        }
      } else {
        state = State::Idle;
        elapsed_count = 0;
      }
    } else {
      // continue cool down
    }
    break;

  case State::Control:
    // Control over
    if (truck_avd_should_end(front_trucks, pre_dodge_info)) {
      if (pre_dodge_info.end_debounce_count < END_DEBOUNCE_TIME) {
        pre_dodge_info.end_debounce_count++;
        pre_dodge_info.end_debounce_count =
            std::min(pre_dodge_info.end_debounce_count, END_DEBOUNCE_TIME);
        target_l = pre_dodge_info.dodge_l;
        avd_direction = pre_dodge_info.avd_direction;
        update_decision_info(decision_info, target_l, AVD_PLAN_LEN,
                             avd_direction);
      } else {
        // turn to CoolingDown
        state = State::CoolingDown;
        elapsed_count = 0;
        pre_dodge_info.end_debounce_count = 0;
      }
    } else {
      if (front_truck_avd_target(front_trucks, target_s, target_l,
                                 avd_direction, active_target_l)) {
        if (avd_direction == "none") {
          avd_direction = pre_dodge_info.avd_direction;
        }
        pre_dodge_info.end_debounce_count = 0;
        update_decision_info(decision_info, target_l, target_s, avd_direction);
      } else {
        if (pre_dodge_info.elapsed_count < CONTROL_TIME) {
          // still Control
          pre_dodge_info.end_debounce_count = 0;
          target_l = pre_dodge_info.dodge_l;
          avd_direction = pre_dodge_info.avd_direction;
          update_decision_info(decision_info, target_l, AVD_PLAN_LEN,
                               avd_direction);
        } else if (pre_dodge_info.end_debounce_count < END_DEBOUNCE_TIME) {
          pre_dodge_info.end_debounce_count++;
          pre_dodge_info.end_debounce_count =
              std::min(pre_dodge_info.end_debounce_count, END_DEBOUNCE_TIME);
          target_l = pre_dodge_info.dodge_l;
          avd_direction = pre_dodge_info.avd_direction;
          update_decision_info(decision_info, target_l, AVD_PLAN_LEN,
                               avd_direction);
        } else {
          // turn to CoolingDown
          pre_dodge_info.end_debounce_count = 0;
          state = State::CoolingDown;
          elapsed_count = 0;
          pre_dodge_info.end_debounce_count = 0;
        }
      }
    }
    break;
  }

  int max_avd_object_id = -1;
  double max_avd_range = 0.0;
  for (const auto &t : active_target_l) {
    if (std::abs(t.second) > std::abs(max_avd_range)) {
      max_avd_range = t.second;
      max_avd_object_id = t.first;
    }
  }
  pre_dodge_info.state = state;
  pre_dodge_info.dodge_truck_id = max_avd_object_id;
  pre_dodge_info.dodge_l = target_l;
  pre_dodge_info.avd_direction = avd_direction;
  elapsed_count++;
  elapsed_count = std::min(elapsed_count, MAX_TIME);
  pre_dodge_info.elapsed_count = elapsed_count;
  pre_dodge_info.lc_count++;
  pre_dodge_info.lc_count = std::min(pre_dodge_info.lc_count, LANE_CHANGE_TIME);
  pre_dodge_info.cp_count++;
  pre_dodge_info.cp_count = std::min(pre_dodge_info.cp_count, CP_TIME);
}

bool ActiveAvoid::truck_avd_should_trigger(
    const std::vector<ObsInfo> &front_trucks,
    const Intelligent_Dodge_Info &pre_dodge_info) {
  if (front_trucks.empty()) {
    return false;
  }
  if (pre_dodge_info.lc_count < LANE_CHANGE_TIME) {
    // std::cout << "not trigger because of lane change cout: "
    //           << pre_dodge_info.lc_count << std::endl;
    return false;
  }
  // check cp status
  if (pre_dodge_info.cp_count < CP_TIME) {
    // std::cout << "not trigger because of cp count: " <<
    // pre_dodge_info.cp_count
    //           << std::endl;
    return false;
  }
  // init speed < 30 kph
  if (input_.planning_init_state.v < 30.0 / 3.6) {
    // std::cout << "not trigger because of low velocity..." << std::endl;
    return false;
  }
  // double side trucks long dist < 35m
  if (truck_avd_invalid(front_trucks)) {
    // std::cout << "truck_avd_invalid = 1" << std::endl;
    return false;
  }

  // scenario: lat dist > 2m
  bool lat_dist_flag = false;
  for (const ObsInfo &obs : front_trucks) {
    double max_l = -10.;
    double min_l = 10.;
    for (const auto p : obs.polygon_init.polygon_fren) {
      max_l = std::max(max_l, p.y);
      min_l = std::min(min_l, p.y);
    }
    if (obs.nudge_side == ObsInfo::NudgeType::LEFT_NUDGE) {
      double lat_dist = input_.planning_init_state.l - max_l - half_self_width_;
      if (lat_dist < 2.0) {
        lat_dist_flag = true;
        break;
      }
    } else {
      double lat_dist = min_l - input_.planning_init_state.l - half_self_width_;
      if (lat_dist < 2.0) {
        lat_dist_flag = true;
        break;
      }
    }
  }
  if (!lat_dist_flag) {
    return false;
  }

  // scenario: radius < 250m
  if (std::abs(input_.planning_init_state.sample_info.refline_info.curvature) >
      StartoffsetCurvature) {
    return false;
  }

  return true;
}
} // namespace path_planner