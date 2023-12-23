#include "planner/behavior_planner/deciders/lane_keep_overtake_decider.h"
#include "linear_interpolation_utility.hpp"
#include "planner/behavior_planner/deciders/st_graph_generator.h"

namespace msquare {
static constexpr double Epsilon = 1.0e-6;
inline double interpolate(const double x0, const double y0, const double x1,
                          const double y1, const double x) {
  if (x < x0 || x1 - x0 < Epsilon) {
    return y0;
  } else if (x > x1) {
    return y1;
  } else {
    return (y1 - y0) / (x1 - x0) * (x - x0) + y0;
  }
}

LaneKeepOvertakeDecider::LaneKeepOvertakeDecider(
    const std::shared_ptr<WorldModel> &world_model,
    const std::shared_ptr<BaseLineInfo> &baseline_info)
    : world_model_(world_model), baseline_info_(baseline_info) {}

bool LaneKeepOvertakeDecider::process(
    const LateralBehaviorPlannerConfig &behavior_config,
    const std::map<int, AvdMsg> &avd_info, const int target_lane_id,
    const std::map<int, double> &last_overtake_obstacles_map) {
  overtake_obstacles_map_ = last_overtake_obstacles_map;

  get_init_state();

  select_block_obstacles(avd_info, behavior_config);

  // judge whether overtake
  double vel_cruise = world_model_->get_map_info().v_cruise();
  (void)init_referece_path_boundary(vel_cruise);

  get_map_boundary(target_lane_id);

  bool has_static_obstacles_overtake = get_static_obstacles_boundary(avd_info);
  if (has_static_obstacles_overtake) {
    for (auto iter = pre_overtake_obstacles_map_.begin();
         iter != pre_overtake_obstacles_map_.end();) {
      if (static_obstacles_.find(iter->first) == static_obstacles_.end()) {
        pre_overtake_obstacles_map_.erase(iter++);
      } else {
        iter++;
      }
    }
  } else if (!get_dynamic_obstacles_boundary(avd_info)) {
    pre_overtake_obstacles_map_.clear();
    overtake_obstacles_map_.clear();
    return false;
  }

  update_overtake_obstacles();

  double avd_buffer = self_half_width_ + path_planner::AVD_WIDTH_DESIRE;
  if (overtake_obstacles_map_.empty()) {
    return false;
  } else if (has_static_obstacles_overtake) {
    std::string avd_direction = "none";
    for (const auto &pre_obj : pre_overtake_obstacles_map_) {
      if (overtake_obstacles_map_.find(pre_obj.first) ==
              overtake_obstacles_map_.end() ||
          avd_info.find(pre_obj.first) == avd_info.end()) {
        continue;
      }
      if (overtake_obstacles_map_[pre_obj.first] > WAIT_TIME &&
          (avd_direction == "none" ||
           avd_info.at(pre_obj.first).avd_direction == avd_direction)) {
        avd_direction = avd_info.at(pre_obj.first).avd_direction;
        update_lon_info(pre_obj.first, avd_direction, vel_cruise, avd_buffer);
      } else {
        break;
      }
    }
  } else if (overtake_obstacles_map_.size() == 1) {
    for (const auto &pre_obj : pre_overtake_obstacles_map_) {
      if (overtake_obstacles_map_.find(pre_obj.first) !=
              overtake_obstacles_map_.end() &&
          avd_info.find(pre_obj.first) != avd_info.end() &&
          overtake_obstacles_map_[pre_obj.first] > WAIT_TIME) {
        update_dynamic_lon_info(
            pre_obj, avd_info.at(pre_obj.first).avd_direction, avd_buffer);
      }
    }
  }

  return true;
}

void LaneKeepOvertakeDecider::get_init_state() {
  const auto &ego_state =
      baseline_info_->get_ego_state_manager().get_ego_state();
  const auto &planning_init_point = ego_state.planning_init_point.path_point;
  const auto &planning_start_frenet_state = ego_state.planning_start_state;

  init_state_.s = planning_init_point.s;
  init_state_.ds = ego_state.ego_vel;
  init_state_.dds = ego_state.ego_acc;
  init_state_.r = planning_start_frenet_state.r;
  init_state_.dr_ds = planning_start_frenet_state.dr_ds;
  init_state_.ddr_dsds = planning_start_frenet_state.ddr_dsds;
  self_half_length_ =
      ConfigurationContext::Instance()->get_vehicle_param().length / 2.0;
  self_half_width_ =
      ConfigurationContext::Instance()->get_vehicle_param().width / 2.0;
}

void LaneKeepOvertakeDecider::select_block_obstacles(
    const std::map<int, AvdMsg> &avd_info,
    const LateralBehaviorPlannerConfig &behavior_config) {
  pre_overtake_obstacles_map_.clear();
  std::vector<std::pair<int, planning_math::Polygon2d>> pre_ot_obss_vector;
  const double block_buffer = 1.8;
  const double relative_block_buffer = 1.7;
  double left_range =
      std::max(init_state_.r + relative_block_buffer, block_buffer);
  double right_range =
      std::min(init_state_.r - relative_block_buffer, -block_buffer);

  auto &obstacle_manager = baseline_info_->mutable_obstacle_manager();
  for (const auto &obj : avd_info) {
    auto ptr_obstacle = obstacle_manager.find_obstacle(obj.first);
    if (ptr_obstacle == nullptr) {
      continue;
    }

    if (!ptr_obstacle->has_sl_polygon_seq()) {
      StGraphData::DoublePair time_range{0.0,
                                         FLAGS_speed_lon_decision_time_horizon};
      bool enable_heuristic = behavior_config.enable_heuristic_frenet_search;

      std::string fail_reason{};
      if (!StGraphGenerator::compute_obs_sl_polygon_seq(
              ptr_obstacle, baseline_info_, time_range, TIME_GAP,
              enable_heuristic, fail_reason)) {
        MSD_LOG(WARN, "obtacle [%d] polygon construction failed",
                ptr_obstacle->Id());
        continue;
      }
    }

    PolygonWithT polygon_with_t;
    if (ptr_obstacle->sl_polygon_seq().EvaluateByTime(0., &polygon_with_t)) {
      const auto &init_polygon = polygon_with_t.second;
      if (init_polygon.points().empty() ||
          init_polygon.max_x() < init_state_.s + self_half_length_) {
        continue;
      }
      if ((obj.second.avd_direction == "left" &&
           init_polygon.max_y() > right_range) ||
          (obj.second.avd_direction == "right" &&
           init_polygon.min_y() < left_range)) {
        pre_ot_obss_vector.emplace_back(obj.first, init_polygon);
      }
    }
  }

  sort(pre_ot_obss_vector.begin(), pre_ot_obss_vector.end(),
       [](const std::pair<int, planning_math::Polygon2d> &x,
          const std::pair<int, planning_math::Polygon2d> &y) {
         return x.second.min_x() < y.second.min_x();
       });
  for (const auto pre_obs : pre_ot_obss_vector) {
    pre_overtake_obstacles_map_[pre_obs.first] = pre_obs.second;
  }
}

bool LaneKeepOvertakeDecider::init_referece_path_boundary(
    const double vel_cruise) {
  v_limit_.clear();
  lat_offset_.clear();
  sl_boundary_.clear();

  v_limit_.reserve(NUM_SAMPLE_POINTS);
  lat_offset_.reserve(NUM_SAMPLE_POINTS);
  sl_boundary_.reserve(NUM_SAMPLE_POINTS);

  double total_length = 1000.;
  if (vel_cruise > init_state_.ds + ACC_LIMIT * TIME_DECIDER) {
    total_length = init_state_.ds * TIME_DECIDER +
                   0.5 * ACC_LIMIT * TIME_DECIDER * TIME_DECIDER;
  } else {
    double acc_time = (vel_cruise - init_state_.ds) / ACC_LIMIT;
    total_length = (vel_cruise - init_state_.ds) / 2.0 * acc_time +
                   vel_cruise * (TIME_DECIDER - acc_time);
  }
  total_length =
      std::min(total_length,
               baseline_info_->get_frenet_coord()->GetLength() - init_state_.s);
  delta_s_ =
      std::min(total_length / static_cast<double>(NUM_SAMPLE_POINTS), 5.0);
  for (int i = 0; i < NUM_SAMPLE_POINTS; i++) {
    double s = init_state_.s + delta_s_ * i;
    v_limit_.emplace_back(s, 40.);
    lat_offset_.emplace_back(s, init_state_.r);
    sl_boundary_.emplace_back(SampleBound{s, 10.0, -10.0, -1000, -1000});
  }

  return true;
}

void LaneKeepOvertakeDecider::get_map_boundary(const int target_lane_id) {
  static std::vector<RefPointFrenet> cur_lane, left_lane, right_lane;
  cur_lane.clear();
  left_lane.clear();
  right_lane.clear();
  world_model_->get_map_lateral_info(cur_lane, left_lane, right_lane,
                                     target_lane_id);

  planning_math::InterpolationData<RefPointFrenet>
      current_lane_map_info_pair_list;
  current_lane_map_info_pair_list.reserve(cur_lane.size());
  for (const auto &p : cur_lane) {
    current_lane_map_info_pair_list.emplace_back(p.s, p);
  }

  for (int i = 0; i < NUM_SAMPLE_POINTS; i++) {
    RefPointFrenet curr_lane_info =
        planning_math::LinearInterpation::interpolate<RefPointFrenet>(
            current_lane_map_info_pair_list, sl_boundary_[i].s);
    double left = std::min(curr_lane_info.left_lane_border -
                               path_planner::LANE_BORDER_AVD_DIST_M,
                           curr_lane_info.left_road_border -
                               path_planner::ROAD_BORDER_AVD_DIST_SOFT) -
                  self_half_width_;
    double right = std::max(-curr_lane_info.right_lane_border +
                                path_planner::LANE_BORDER_AVD_DIST_M,
                            -curr_lane_info.right_road_border +
                                path_planner::ROAD_BORDER_AVD_DIST_SOFT) +
                   self_half_width_;
    if (left < sl_boundary_[i].left) {
      sl_boundary_[i].left = left;
      sl_boundary_[i].left_id = -100;
    }
    if (right > sl_boundary_[i].right) {
      sl_boundary_[i].right = right;
      sl_boundary_[i].right_id = -100;
    }

    if (left > right) {
      std::get<1>(lat_offset_[i]) =
          clip(std::get<1>(lat_offset_[i]), left, right);
    } else {
      std::get<1>(lat_offset_[i]) = 0.;
    }
  }
}

bool LaneKeepOvertakeDecider::get_static_obstacles_boundary(
    const std::map<int, AvdMsg> &avd_info) {
  static_obstacles_.clear();
  ObsDirectedPolyline avd_polyline_info;
  const auto &obstacle_manager = baseline_info_->obstacle_manager();
  double avd_buffer_base = self_half_width_ + path_planner::AVD_WIDTH_DESIRE;
  for (const auto &obj : avd_info) {
    const auto ptr_obstacle = obstacle_manager.find_obstacle(obj.first);
    if (ptr_obstacle != nullptr &&
        (ptr_obstacle->IsStatic() || ptr_obstacle->speed() < 0.2) &&
        ptr_obstacle->has_sl_polygon_seq()) {
      static_obstacles_.insert(obj.first);

      PolygonWithT polygon_with_t;
      double avd_buffer = avd_buffer_base;
      if (overtake_obstacles_map_.find(obj.first) !=
              overtake_obstacles_map_.end() &&
          overtake_obstacles_map_[obj.first] > WAIT_TIME) {
        avd_buffer = avd_buffer_base - DELAY_LATERAL_BUFFER;
      }
      if (ptr_obstacle->sl_polygon_seq().EvaluateByTime(0., &polygon_with_t)) {
        avd_polyline_info[obj.first] = DirectedPolyline{
            obj.second.avd_direction, avd_buffer,
            get_polygon_edge_with_decision(polygon_with_t.second.points(),
                                           obj.second.avd_direction)};
      }
    }
  }

  update_obstacle_boundary(avd_polyline_info, sl_boundary_);

  double block_s = sl_boundary_.back().s + Epsilon;
  for (const auto &bound : sl_boundary_) {
    if (bound.left < bound.right) {
      block_s = std::min(bound.s, block_s);
    }
  }

  if (block_s < sl_boundary_.back().s) {
    for (const auto &obj : avd_polyline_info) {
      const auto &polyline = obj.second.points;
      if (pre_overtake_obstacles_map_.find(obj.first) !=
              pre_overtake_obstacles_map_.end() &&
          polyline.back().x() > block_s - 6.0) {
        pre_overtake_obstacles_map_.erase(obj.first);
      }
    }
  }

  for (const auto &obj : pre_overtake_obstacles_map_) {
    if (static_obstacles_.find(obj.first) != static_obstacles_.end()) {
      return true;
    }
  }
  return false;
}

bool LaneKeepOvertakeDecider::get_dynamic_obstacles_boundary(
    const std::map<int, AvdMsg> &avd_info) {
  int nearest_dynamic_id = -1000;
  for (const auto &obj : pre_overtake_obstacles_map_) {
    if (static_obstacles_.find(obj.first) == static_obstacles_.end()) {
      nearest_dynamic_id = obj.first;
      break;
    }
  }
  if (nearest_dynamic_id == -1000) {
    return false;
  }

  const auto &obstacle_manager = baseline_info_->obstacle_manager();
  const auto ptr_obstacle = obstacle_manager.find_obstacle(nearest_dynamic_id);
  if (ptr_obstacle == nullptr || !ptr_obstacle->has_sl_polygon_seq() ||
      ptr_obstacle->sl_polygon_seq().back().first < 4.0) {
    return false;
  }

  if (avd_info.find(ptr_obstacle->Id()) == avd_info.end()) {
    return false;
  }
  std::string avd_direction = avd_info.at(ptr_obstacle->Id()).avd_direction;
  std::vector<TimedDirectedPolyline> avd_polyline_info;
  double avd_buffer_base =
      self_half_width_ + path_planner::AVD_WIDTH_DESIRE +
      AVD_WIDTH_DESIRE_INCREASE_RATE_VS_VEL_BEHAVIOR * ptr_obstacle->speed();
  if (overtake_obstacles_map_.find(nearest_dynamic_id) !=
          overtake_obstacles_map_.end() &&
      overtake_obstacles_map_[nearest_dynamic_id] > WAIT_TIME) {
    avd_buffer_base -= DELAY_LATERAL_BUFFER;
  }

  for (double time = 0.; time < FLAGS_speed_lon_decision_time_horizon;
       time += TIME_GAP) {
    ObsDirectedPolyline directed_polyline;
    PolygonWithT polygon_with_t;
    double avd_buffer = avd_buffer_base - time * AVD_WIDTH_DESIRE_DECAY_VS_TIME;

    if (ptr_obstacle->sl_polygon_seq().EvaluateByTime(time, &polygon_with_t)) {
      directed_polyline[nearest_dynamic_id] =
          DirectedPolyline{avd_direction, avd_buffer,
                           get_polygon_edge_with_decision(
                               polygon_with_t.second.points(), avd_direction)};
    }
    avd_polyline_info.emplace_back(time, directed_polyline);
  }

  std::vector<std::vector<SampleBound>> t_sl_boundary(avd_polyline_info.size(),
                                                      sl_boundary_);
  for (int i = 0; i < avd_polyline_info.size(); i++) {
    update_obstacle_boundary(avd_polyline_info[i].second, t_sl_boundary[i]);
    for (const auto &bound : t_sl_boundary[i]) {
      if (bound.left < bound.right) {
        return false;
      }
    }
  }

  return true;
} // namespace msquare

void LaneKeepOvertakeDecider::update_overtake_obstacles() {
  for (auto iter = overtake_obstacles_map_.begin();
       iter != overtake_obstacles_map_.end();) {
    if (pre_overtake_obstacles_map_.find(iter->first) ==
        pre_overtake_obstacles_map_.end()) {
      overtake_obstacles_map_.erase(iter++);
    } else {
      overtake_obstacles_map_[iter->first] += 0.1;
      iter++;
    }
  }

  for (const auto &pre_ot_obj : pre_overtake_obstacles_map_) {
    if (overtake_obstacles_map_.find(pre_ot_obj.first) ==
        overtake_obstacles_map_.end()) {
      overtake_obstacles_map_[pre_ot_obj.first] = 0.1;
    }
  }
}

std::vector<planning_math::Vec2d>
LaneKeepOvertakeDecider::get_polygon_edge_with_decision(
    const std::vector<planning_math::Vec2d> &points_ori,
    const std::string &avd_direction) {
  std::vector<planning_math::Vec2d> output_polygon_edge;
  auto comp1 = [](const planning_math::Vec2d &point1,
                  const planning_math::Vec2d &point2) {
    return point1.x() < point2.x();
  };

  auto points = points_ori;
  sort(points.begin(), points.end(), comp1);
  planning_math::LineSegment2d line(points.front(), points.back());
  output_polygon_edge.emplace_back(points.front());
  output_polygon_edge.emplace_back(points.back());

  for (const auto &point : points) {
    const double prod =
        planning_math::CrossProd(line.start(), line.end(), point);
    if (std::abs(prod) > Epsilon) {
      if (avd_direction == "left" && prod > 0.0) {
        output_polygon_edge.emplace_back(point);
      } else if (avd_direction == "right" && prod < 0.0) {
        output_polygon_edge.emplace_back(point);
      }
    }
  }

  sort(output_polygon_edge.begin(), output_polygon_edge.end(), comp1);
  return output_polygon_edge;
}

void LaneKeepOvertakeDecider::update_obstacle_boundary(
    const ObsDirectedPolyline &avd_polyline_info,
    std::vector<SampleBound> &sl_boundary) {
  for (int i = 1; i < sl_boundary.size(); i++) {
    for (const auto &obj : avd_polyline_info) {
      const auto &polyline = obj.second.points;
      const auto &avd_direction = obj.second.direction;
      const double avd_buffer = obj.second.avd_buffer;
      if (polyline.empty()) {
        continue;
      }

      double front_s = polyline.front().x() - self_half_length_ + 2.0;
      double rear_s = polyline.back().x() + self_half_length_;
      if (sl_boundary[i].s > front_s &&
          sl_boundary[i].s < polyline.front().x()) {
        update_boundary(i, polyline.front().y(), obj.first, avd_direction,
                        avd_buffer, sl_boundary);
        continue;
      } else if (sl_boundary[i].s > polyline.back().x()) {
        update_boundary(i, polyline.back().y(), obj.first, avd_direction,
                        avd_buffer, sl_boundary);
        continue;
      } else if (sl_boundary[i].s > rear_s) {
        update_boundary(i, polyline.back().y(), obj.first, avd_direction,
                        avd_buffer, sl_boundary);
        break;
      } else {
        for (int j = 1; j < polyline.size(); ++j) {
          if (polyline[j - 1].x() < sl_boundary[i].s &&
              polyline[j].x() > sl_boundary[i].s) {
            double l =
                interpolate(polyline[j - 1].x(), polyline[j - 1].y(),
                            polyline[j].x(), polyline[j].y(), sl_boundary[i].s);
            update_boundary(i, l, obj.first, avd_direction, avd_buffer,
                            sl_boundary);
            break;
          }
        }
      }
    }
  }

  for (const auto &obj : avd_polyline_info) {
    const auto &polyline = obj.second.points;
    const auto &avd_direction = obj.second.direction;
    const double avd_buffer = obj.second.avd_buffer;
    if (polyline.empty()) {
      continue;
    }

    int start_index = std::max(
        static_cast<int>(std::floor(
            (polyline.front().x() - self_half_length_ - init_state_.s) /
            delta_s_)),
        1);
    int end_index =
        std::min(static_cast<int>(std::ceil(
                     (polyline.back().x() + self_half_length_ - init_state_.s) /
                     delta_s_)),
                 NUM_SAMPLE_POINTS);
    for (int i = 0; i < polyline.size(); ++i) {
      double front_s = polyline[i].x() - self_half_length_;
      double rear_s = polyline[i].x() + self_half_length_;
      for (int j = start_index; j < NUM_SAMPLE_POINTS && j < end_index; ++j) {
        if ((sl_boundary[j].s > front_s &&
             sl_boundary[j].s < polyline[i].x()) ||
            (sl_boundary[j].s > polyline[i].x() && sl_boundary[j].s < rear_s)) {
          update_boundary(j, polyline[i].y(), obj.first, avd_direction,
                          avd_buffer, sl_boundary);
        }
      }
    }
  }
}

void LaneKeepOvertakeDecider::update_boundary(
    const int index, const double l, const int id,
    const std::string &avd_direction, const double avd_buffer,
    std::vector<SampleBound> &sl_boundary) {
  if (avd_direction == "left" && sl_boundary[index].right < l + avd_buffer) {
    sl_boundary[index].right = l + avd_buffer;
    sl_boundary[index].right_id = id;
  } else if (avd_direction == "right" &&
             sl_boundary[index].left > l - avd_buffer) {
    sl_boundary[index].left = l - avd_buffer;
    sl_boundary[index].left_id = id;
  }
}

void LaneKeepOvertakeDecider::update_lon_info(const int id,
                                              const std::string avd_direction,
                                              const double v_cruise,
                                              const double avd_buffer) {
  ActiveAvdDeciderinfo target;
  const auto &polygon = pre_overtake_obstacles_map_[id];
  if ((avd_direction == "left" && polygon.max_y() + avd_buffer < 0.) ||
      (avd_direction == "right" && polygon.min_y() - avd_buffer > 0)) {
    return;
  }

  const double min_vel = 5.0;
  const double jerk_limit = 1.0;

  target.id = id;
  target.start_s = pre_overtake_obstacles_map_[id].min_x() - self_half_length_;
  target.end_s = pre_overtake_obstacles_map_[id].max_x() + self_half_length_;
  double dddl = jerk_limit / std::pow(std::max(init_state_.ds, min_vel), 3);
  if (avd_direction == "left") {
    target.target_l = polygon.max_y() + avd_buffer;
  } else if (avd_direction == "right") {
    target.target_l = polygon.min_y() - avd_buffer;
  }

  planning_math::QuinticPoly1d quintic_poly(0., target.target_l, dddl);
  double s_length = quintic_poly.get_end_s();
  for (auto &bound : sl_boundary_) {
    double sample_s = bound.s - init_state_.s;
    double l = 0;
    if (sample_s <= target.start_s - init_state_.s - s_length) {
      continue;
    } else if (sample_s > target.start_s - init_state_.s - s_length &&
               sample_s <= target.start_s - init_state_.s) {
      l = quintic_poly.Evaluate(0, sample_s);
    } else {
      l = target.target_l;
      if (sample_s < target.end_s - init_state_.s) {
        target.target_vel =
            std::min(target.target_vel,
                     interpolate(SpaceSpeedLow.first, SpaceSpeedLow.second,
                                 SpaceSpeedHigh.first, SpaceSpeedHigh.second,
                                 bound.left - bound.right));
      }
    }
    if (avd_direction == "left") {
      bound.right = clip(l, bound.left, bound.right);
    } else if (avd_direction == "right") {
      bound.left = clip(l, bound.left, bound.right);
    }
  }

  smooth_static_boundary();

  double dec = std::min((init_state_.ds * init_state_.ds -
                         target.target_vel * target.target_vel) /
                            (target.start_s - init_state_.s) / 2.0,
                        DEC_FOR_V_LIMIT);
  double v_limit_last = init_state_.ds;
  for (int i = 1; i < NUM_SAMPLE_POINTS; i++) {
    std::get<1>(lat_offset_[i]) =
        clip(init_state_.r, sl_boundary_[i].left, sl_boundary_[i].right);

    if (std::get<0>(v_limit_[i]) < target.end_s) {
      if (init_state_.ds <= target.target_vel ||
          v_limit_last <= target.target_vel + 0.1) {
        std::get<1>(v_limit_[i]) = target.target_vel;
      } else {
        std::get<1>(v_limit_[i]) = std::sqrt(
            std::max(v_limit_last * v_limit_last - 2. * dec * delta_s_,
                     target.target_vel * target.target_vel));
      }
    } else {
      std::get<1>(v_limit_[i]) = std::min(
          std::sqrt(v_limit_last * v_limit_last + 2. * ACC_LIMIT * delta_s_),
          v_cruise);
    }
    v_limit_last = std::get<1>(v_limit_[i]);
  }
}

void LaneKeepOvertakeDecider::update_dynamic_lon_info(
    const std::pair<int, planning_math::Polygon2d> obj,
    const std::string &avd_direction, const double avd_buffer) {
  const auto &obstacle_manager = baseline_info_->obstacle_manager();
  const auto ptr_obstacle = obstacle_manager.find_obstacle(obj.first);
  if (ptr_obstacle == nullptr) {
    return;
  }

  const double static_desire_dist = 10.0;
  const double obj_desire_time = 2.0;
  const double self_desire_time = 2.0;
  double obj_vel = ptr_obstacle->speed();
  double desire_dist = obj_vel * obj_desire_time +
                       init_state_.ds * self_desire_time + static_desire_dist;
  if (obj.second.min_x() - init_state_.s - self_half_length_ < desire_dist) {
    const double jerk = 1.5;
    double dddl = jerk / std::pow(std::max(init_state_.ds, 3.0), 3);
    double target_l = init_state_.r;

    double end_s = 0.;
    bool get_opt_quintic_poly = false;
    msquare::planning_math::QuinticPoly1d quintic_poly;
    if (avd_direction == "left") {
      target_l = std::min(sl_boundary_.front().left,
                          obj.second.max_y() + avd_buffer + self_half_width_);
      if (init_state_.r < target_l) {
        get_opt_quintic_poly = get_const_dddl_quintic_poly(
            target_l, dddl, avd_direction, quintic_poly, end_s);
      }
    } else if (avd_direction == "right") {
      target_l = std::max(sl_boundary_.front().right,
                          obj.second.min_y() - avd_buffer - self_half_width_);
      if (init_state_.r > target_l) {
        get_opt_quintic_poly = get_const_dddl_quintic_poly(
            target_l, -dddl, avd_direction, quintic_poly, end_s);
      }
    }

    for (auto &bound : sl_boundary_) {
      double sample_s = bound.s - init_state_.s;
      double l = 0;
      if (sample_s <= end_s) {
        l = quintic_poly.Evaluate(0, sample_s);
      } else {
        l = target_l;
      }
      if (avd_direction == "left") {
        bound.right = clip(l, bound.left, bound.right);
      } else if (avd_direction == "right") {
        bound.left = clip(l, bound.left, bound.right);
      }
    }

    double v_frenet =
        std::max(0., obj_vel * std::cos(ptr_obstacle->Yaw_relative_frenet()));
    double target_vel = v_frenet + 5.;
    double v_limit_last = init_state_.ds;
    for (int i = 1; i < NUM_SAMPLE_POINTS; i++) {
      std::get<1>(lat_offset_[i]) =
          clip(init_state_.r, sl_boundary_[i].left, sl_boundary_[i].right);

      if (init_state_.ds <= target_vel || v_limit_last <= target_vel + 0.1) {
        std::get<1>(v_limit_[i]) = target_vel;
      } else {
        std::get<1>(v_limit_[i]) = std::sqrt(std::max(
            v_limit_last * v_limit_last - 2. * DEC_FOR_V_LIMIT * delta_s_,
            target_vel * target_vel));
      }
      v_limit_last = std::get<1>(v_limit_[i]);
    }
  }
}

bool LaneKeepOvertakeDecider::get_const_dddl_quintic_poly(
    const double target_l, const double dddl_ref, const std::string &direction,
    planning_math::QuinticPoly1d &quintic_poly_opt, double &end_s_opt) {
  bool get_opt_quintic_poly = false;
  planning_math::QuinticPoly1d quintic_poly_base(init_state_.r, target_l,
                                                 dddl_ref);
  double end_s_base = quintic_poly_base.get_end_s();
  double step_s = std::fmax(init_state_.ds, 3.0) * 0.4;

  double cost_min = std::numeric_limits<double>::max();
  double end_s = end_s_base - step_s * 10;
  for (; end_s < end_s_base + step_s * 10; end_s += step_s) {
    if (end_s < 0)
      continue;
    planning_math::QuinticPoly1d quintic_poly(init_state_.r, init_state_.dr_ds,
                                              init_state_.ddr_dsds, target_l,
                                              0., 0., end_s);
    double cost = 0.;
    int count = 0;
    for (const auto bound : sl_boundary_) {
      double s = bound.s - init_state_.s;
      if (s > end_s) {
        break;
      }
      double end_s_scale = 10.;
      cost += end_s / std::max(init_state_.ds, 1.0) * end_s_scale;

      double dddl = quintic_poly.Evaluate(3, s);
      double jerk = std::pow(std::max(init_state_.ds, 5.0), 3) * dddl;
      double jerk_scale = std::abs(dddl) < std::abs(dddl_ref) ? 0.1 : 100.;
      cost += jerk * jerk * jerk_scale;

      double l = quintic_poly.Evaluate(0, s);
      if (direction == "left" && l > target_l + 0.1) {
        double overshot_scale = 10.;
        cost += overshot_scale * (l - target_l - 0.1);
      }
      if (direction == "right" && l < target_l - 0.1) {
        double overshot_scale = 10.;
        cost += overshot_scale * (target_l - 0.1 - l);
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

void LaneKeepOvertakeDecider::smooth_static_boundary() {
  double left_target_l = sl_boundary_.front().left;
  double right_target_l = sl_boundary_.front().right;

  const double jerk = 2.0;
  double dddl = jerk / std::pow(std::max(init_state_.ds, 1.0), 3);
  // left smoothing
  double end_s_right = 0.;
  bool get_opt_quintic_poly_right = false;
  planning_math::QuinticPoly1d quintic_poly_right;
  if (init_state_.r < left_target_l) {
    get_opt_quintic_poly_right = get_const_dddl_quintic_poly(
        left_target_l, dddl, "left", quintic_poly_right, end_s_right);
  }
  // right smoothing
  double end_s_left = 0.;
  bool get_opt_quintic_poly_left = false;
  planning_math::QuinticPoly1d quintic_poly_left;
  if (init_state_.r > right_target_l) {
    get_opt_quintic_poly_left = get_const_dddl_quintic_poly(
        left_target_l, -dddl, "right", quintic_poly_left, end_s_left);
  }

  for (auto &bound : sl_boundary_) {
    double sample_s = bound.s - init_state_.s;

    if (get_opt_quintic_poly_right) {
      double l = bound.left;
      if (sample_s <= end_s_right) {
        l = std::min(quintic_poly_right.Evaluate(0, sample_s), l);
      }
      bound.right = std::min(bound.right, l);
    }

    if (get_opt_quintic_poly_left) {
      double l = bound.right;
      if (sample_s <= end_s_left) {
        l = std::max(quintic_poly_left.Evaluate(0, sample_s), l);
      }
      bound.left = std::max(bound.left, l);
    }
  }
}

} // namespace msquare