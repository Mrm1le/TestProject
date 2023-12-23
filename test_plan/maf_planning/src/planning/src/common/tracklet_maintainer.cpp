#define _USE_MATH_DEFINES
#include "common/tracklet_maintainer.h"
#include "common/planning_context.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace msquare {

TrackletSequentialState *LifecycleDict::get(int uid) {
  auto iter = data_dict_.find(uid);
  if (iter != data_dict_.end()) {
    return &(iter->second);
  } else {
    // if not exist, create
    TrackletSequentialState state;
    data_dict_.insert(std::make_pair(uid, state));
  }

  return &(iter->second);
}

bool LifecycleDict::set(int uid, const TrackletSequentialState &state) {
  auto iter = data_dict_.find(uid);
  if (iter == data_dict_.end()) {
    data_dict_.insert(std::make_pair(uid, state));
  } else {
    iter->second = state;
  }

  return false;
}

void LifecycleDict::mark_dirty(int uid) { dirty_set_.insert(uid); }

void LifecycleDict::remove_clean() {
  if (dirty_set_.empty()) {
    data_dict_.clear();
    return;
  }

  std::map<int, TrackletSequentialState> new_dict;
  for (auto id : dirty_set_) {
    auto iter = data_dict_.find(id);
    if (iter != data_dict_.end()) {
      new_dict.insert(std::make_pair(id, iter->second));
    }
  }

  data_dict_ = new_dict;
  dirty_set_.clear();
}

SimpleRefLine::SimpleRefLine() { update_ = false; }

void SimpleRefLine::update_pathpoints(
    const std::vector<PathPoint> &path_points) {
  if (path_points.size() < 2) {
    update_ = false;
    path_points_.clear();
    return;
  }

  path_points_ = path_points;
  update_ = true;
}

void SimpleRefLine::cartesian_frenet(double x, double y, double &s, double &l,
                                     double &v_s, double &v_l, double &theta,
                                     bool get_theta, const double *v,
                                     const double *yaw) {
  if (path_points_.size() < 2) {
    return;
  }

  calc_cartesian_frenet(path_points_, x, y, s, l, v_s, v_l, theta, get_theta, v,
                        yaw);
}

void SimpleRefLine::frenet_cartesian(double s, double l, double &x, double &y) {
  calc_frenet_cartesian(path_points_, s, l, x, y);
}

TrackletMaintainer::TrackletMaintainer() {
  s0_ = 0;
  l0_ = 0;
  theta_ego_ = 0;
  vl_ego_ = 0;
  vs_ego_ = 0;
}

TrackletMaintainer::~TrackletMaintainer() {
  for (auto iter = object_map_.begin(); iter != object_map_.end(); ++iter) {
    delete iter->second;
  }

  object_map_.clear();
}

void TrackletMaintainer::apply_update(
    const EgoState &ego_state, const std::vector<PredictionObject> &predictions,
    const RefLine &f_refline, std::vector<TrackedObject> &tracked_objects,
    LeadCars &lead_cars, bool isRedLightStop) {
  ego_state_ = ego_state;
  LeadCars leadcars;
  std::vector<TrackedObject *> objects;

  recv_prediction_objects(predictions, objects);

  auto &lateral_output =
      PlanningContext::Instance()->lateral_behavior_planner_output();
  auto &state_machine_output =
      PlanningContext::Instance()->state_machine_output();

  calc(objects, f_refline.path_points(), lateral_output.scenario,
       lateral_output.flane_width, lateral_output.lat_offset,
       lateral_output.tleft_lane, lateral_output.rightest_lane,
       lateral_output.dist_intersect, lateral_output.intersect_length, leadcars,
       isRedLightStop, lateral_output.isOnHighway, lateral_output.d_poly,
       lateral_output.c_poly, lateral_output.l_dash_length,
       lateral_output.r_dash_length);

  set_default_value(objects);

  tracked_objects.resize(objects.size());

  double curr_time = PlanningContext::Instance()
                         ->planning_status()
                         .planning_result.next_timestamp_sec;

  for (size_t i = 0; i < objects.size(); i++) {
    tracked_objects[i] = *objects[i];

    objects[i]->last_recv_time = curr_time;
  }

  lead_cars = leadcars;
}

void TrackletMaintainer::recv_prediction_objects(
    const std::vector<PredictionObject> &predictions,
    std::vector<TrackedObject *> &objects) {

  double pi = std::atan(1.0) * 4;
  std::map<int, TrackedObject *> new_map;

  double ego_fx = std::cos(ego_state_.ego_pose_raw.theta);
  double ego_fy = std::sin(ego_state_.ego_pose_raw.theta);
  double ego_lx = -ego_fy;
  double ego_ly = ego_fx;

  for (auto &p : predictions) {
    if (p.type == 0) {
      continue;
    }

    double dx = p.position_x - ego_state_.ego_pose_raw.x;
    double dy = p.position_y - ego_state_.ego_pose_raw.y;

    double rel_x = dx * ego_fx + dy * ego_fy;
    double rel_y = dx * ego_lx + dy * ego_ly;

    if (rel_x < -50 || rel_x > 120 || p.trajectory_array.size() == 0) {
      continue;
    }

    TrackedObject *origin = nullptr;

    auto iter = object_map_.find(p.id);

    if (iter != object_map_.end()) {
      origin = iter->second;
      object_map_.erase(iter);
      new_map.insert(std::make_pair(origin->track_id, origin));
    } else {
      origin = new TrackedObject();
      origin->track_id = p.id;
      new_map.insert(std::make_pair(origin->track_id, origin));
    }

    origin->timestamp = get_system_time();
    origin->type = p.type;

    double speed_yaw = p.yaw;
    if (p.trajectory_array[0].trajectory.size() > 0) {
      speed_yaw = p.trajectory_array[0].trajectory[0].yaw;
    }
    double abs_vx = p.speed * std::cos(speed_yaw);
    double abs_vy = p.speed * std::sin(speed_yaw);
    double rot_vx = abs_vx * ego_fx + abs_vy * ego_fy;

    double abs_ax = p.acc * std::cos(speed_yaw);
    double abs_ay = p.acc * std::sin(speed_yaw);
    double rot_ax = abs_ax * ego_fx + abs_ay * ego_fy;

    double theta = p.yaw - ego_state_.ego_pose_raw.theta;
    if (theta > pi) {
      theta -= 2 * pi;
    } else if (theta < -pi) {
      theta += 2 * pi;
    }

    origin->length = p.length;
    origin->width = p.width;

    origin->center_x = rel_x;
    origin->center_y = rel_y;
    origin->theta = theta;
    origin->y_rel_ori = rel_y;

    origin->a = p.acc;
    origin->v = p.speed;
    origin->v_lead = rot_vx;

    origin->a_lead = rot_ax;

    origin->oncoming = (origin->v_lead < -3.9);

    int idx = 0;
    for (auto &tr : p.trajectory_array) {
      if (tr.trajectory.empty()) {
        idx++;
        continue;
      }
      TrackedObject *object = nullptr;
      int track_id = hash_prediction_id(origin->track_id, idx);

      iter = object_map_.find(track_id);
      if (iter != object_map_.end()) {
        object = iter->second;
        object_map_.erase(iter);

        if (object != origin) {
          *object = *origin;
          object->track_id = track_id;
        }
        new_map.insert(std::make_pair(track_id, object));
      } else {
        iter = new_map.find(track_id);
        if (iter != new_map.end()) {
          object = iter->second;

          if (object != origin) {
            *object = *origin;
            object->track_id = track_id;
          }
        } else {
          object = new TrackedObject(*origin);
          object->track_id = track_id;
          new_map.insert(std::make_pair(track_id, object));
        }
      }

      dx = tr.trajectory[0].x - ego_state_.ego_pose_raw.x;
      dy = tr.trajectory[0].y - ego_state_.ego_pose_raw.y;
      rel_x = dx * ego_fx + dy * ego_fy;
      rel_y = dx * ego_lx + dy * ego_ly;
      object->center_x = rel_x;
      object->center_y = rel_y;

      object->prediction.prob = tr.prob;
      object->prediction.interval = tr.prediction_interval;
      object->prediction.num_of_points = tr.num_of_points;
      // object->prediction.const_vel_prob = tr.const_vel_prob;
      // object->prediction.const_acc_prob = tr.const_acc_prob;
      // object->prediction.still_prob = tr.still_prob;
      // object->prediction.coord_turn_prob = tr.coord_turn_prob;

      size_t size = tr.trajectory.size();

      object->trajectory.x.resize(size);
      object->trajectory.y.resize(size);
      object->trajectory.yaw.resize(size);
      object->trajectory.speed.resize(size);

      object->trajectory.std_dev_x.resize(size);
      object->trajectory.std_dev_y.resize(size);
      // object->trajectory.std_dev_yaw.resize(size);
      // object->trajectory.std_dev_speed.resize(size);

      // object->trajectory.relative_ego_x.resize(size);
      // object->trajectory.relative_ego_y.resize(size);
      object->trajectory.relative_ego_yaw.resize(size);
      // object->trajectory.relative_ego_speed.resize(size);

      // object->trajectory.relative_ego_std_dev_x.resize(size);
      // object->trajectory.relative_ego_std_dev_y.resize(size);
      // object->trajectory.relative_ego_std_dev_yaw.resize(size);
      // object->trajectory.relative_ego_std_dev_speed.resize(size);

      for (size_t i = 0; i < size; i++) {
        object->trajectory.x[i] = tr.trajectory[i].x;
        object->trajectory.y[i] = tr.trajectory[i].y;
        object->trajectory.yaw[i] = tr.trajectory[i].yaw;
        object->trajectory.speed[i] = tr.trajectory[i].speed;

        object->trajectory.std_dev_x[i] = tr.trajectory[i].std_dev_x;
        object->trajectory.std_dev_y[i] = tr.trajectory[i].std_dev_y;
        // object->trajectory.std_dev_yaw[i] = tr.trajectory[i].std_dev_yaw;
        // object->trajectory.std_dev_speed[i] = tr.trajectory[i].std_dev_speed;

        // object->trajectory.relative_ego_x[i] =
        // tr.trajectory[i].relative_ego_x; object->trajectory.relative_ego_y[i]
        // = tr.trajectory[i].relative_ego_y;
        object->trajectory.relative_ego_yaw[i] =
            tr.trajectory[i].relative_ego_yaw;
        // object->trajectory.relative_ego_speed[i] =
        //     tr.trajectory[i].relative_ego_speed;

        // object->trajectory.relative_ego_std_dev_x[i] =
        //     tr.trajectory[i].relative_ego_std_dev_x;
        // object->trajectory.relative_ego_std_dev_y[i] =
        //     tr.trajectory[i].relative_ego_std_dev_y;
        // object->trajectory.relative_ego_std_dev_yaw[i] =
        //     tr.trajectory[i].relative_ego_std_dev_yaw;
        // object->trajectory.relative_ego_std_dev_speed[i] =
        //     tr.trajectory[i].relative_ego_std_dev_speed;
      }

      objects.push_back(object);
      idx++;
    }
  }

  for (auto iter = object_map_.begin(); iter != object_map_.end(); ++iter) {
    delete iter->second;
  }

  object_map_.clear();
  object_map_ = new_map;
}

void TrackletMaintainer::calc(
    std::vector<TrackedObject *> &tracked_objects,
    const std::vector<PathPoint> &path_points, int scenario, double lane_width,
    double lat_offset, bool tleft_lane, bool rightest_lane,
    double dist_intersect, double intersect_length, LeadCars &lead_cars,
    bool isRedLightStop, bool isOnHighway, std::vector<double> d_poly,
    std::vector<double> c_poly, double l_dash_length, double r_dash_length) {

  seq_state_.remove_clean();
  simple_refline_.update_pathpoints(path_points);

  double v_ego = ego_state_.ego_vel;

  if (simple_refline_.has_update()) {
    double yaw = 0;
    simple_refline_.cartesian_frenet(0, 0, s0_, l0_, vs_ego_, vl_ego_,
                                     theta_ego_, true, &v_ego, &yaw);

    for (auto item : tracked_objects) {
      item->v_ego = v_ego;
      item->v_rel = item->v_lead - v_ego;

      double d_poly_offset = lat_offset;

      fill_info_with_refline(*item, simple_refline_, d_poly_offset);
      (void)is_potential_lead_one(*item, v_ego);
    }
  }

  select_lead_cars(tracked_objects, lead_cars);

  for (auto tr : tracked_objects) {
    tr->is_avd_car = is_potential_avoiding_car(
        *tr, lead_cars.lead_one, v_ego, lane_width, scenario, tleft_lane,
        rightest_lane, dist_intersect, intersect_length, isRedLightStop);
  }

  (void)is_leadone_potential_avoiding_car(lead_cars.lead_one, scenario,
                                          lane_width, rightest_lane,
                                          dist_intersect, isRedLightStop);
}

void TrackletMaintainer::fill_info_with_refline(TrackedObject &item,
                                                SimpleRefLine &simple_refline,
                                                double lat_offset) {
  double half_length = item.length * 0.5;

  double half_width;
  if (item.type > 10000) {
    half_width = std::max(0.5, std::min(2.0, item.width * 0.5));
  } else if (item.type > 0) {
    half_width = std::max(1.0, std::min(2.0, item.width * 0.5));
  } else {
    half_width = item.width * 0.5;
  }

  double speed_yaw;
  if (item.trajectory.yaw.size() > 0) {
    speed_yaw = item.trajectory.yaw[0] - ego_state_.ego_pose_raw.theta;
  } else {
    speed_yaw = item.theta;
  }

  double s = 0.0;
  double l = 0.0;
  double theta = 0.0;
  double v_s = 0.0;
  double v_l = 0.0;
  simple_refline.cartesian_frenet(item.center_x, item.center_y, s, l, v_s, v_l,
                                  theta, true, &item.v, &speed_yaw);

  item.vs_rel = v_s - vs_ego_;
  item.v_lead = v_s;
  item.v_rel = v_s - vs_ego_;
  item.s = s;
  item.l = l;
  item.l0 = l0_;
  item.c0 = l0_;
  item.a_lead = item.a;
  item.v_lat = (l > 0) ? v_l : -v_l;
  item.v_lat_self = item.v_lat;
  item.vy_rel = v_l;

  std::array<int, 2> sgn_list{1, -1};
  std::vector<double> bbox_s;
  std::vector<double> bbox_l;
  std::vector<double> bbox_l_pos;

  for (auto sgn_length : sgn_list) {
    for (auto sgn_width : sgn_list) {
      double _s = item.s +
                  sgn_length * std::cos(item.theta - theta) * half_length -
                  sgn_width * std::sin(item.theta - theta) * item.width * 0.5;

      if ((item.s - s0_) * (_s - s0_) <= 0) {
        half_width = item.width * 0.5;
        break;
      }
    }
  }

  for (auto sgn_length : sgn_list) {
    for (auto sgn_width : sgn_list) {
      double _s = item.s +
                  sgn_length * std::cos(item.theta - theta) * half_length -
                  sgn_width * std::sin(item.theta - theta) * half_width;

      double _l = item.l +
                  sgn_length * std::sin(item.theta - theta) * half_length +
                  sgn_width * std::cos(item.theta - theta) * half_width;

      bbox_s.push_back(_s);
      bbox_l.push_back(_l);

      if (_s >= s0_) {
        bbox_l_pos.push_back(_l);
      }
    }
  }

  if (bbox_s.empty()) {
    item.d_rel = 0;
  } else {
    if (item.s > s0_) {
      double min_s = *std::min_element(bbox_s.begin(), bbox_s.end());
      item.d_rel = (min_s > s0_) ? (min_s - s0_) : 0;
    } else {
      double max_s = *std::max_element(bbox_s.begin(), bbox_s.end());
      item.d_rel = (max_s < s0_) ? (max_s - s0_) : 0;
    }
  }

  std::vector<double> bbox_l_self;
  std::vector<double> bbox_l_self_pos;

  for (auto value : bbox_l) {
    bbox_l_self.push_back(value - l0_);
  }

  for (auto value : bbox_l_pos) {
    bbox_l_self_pos.push_back(value - l0_);
  }

  double min_l =
      (!bbox_l.empty()) ? *std::min_element(bbox_l.begin(), bbox_l.end()) : 0;
  double max_l =
      (!bbox_l.empty()) ? *std::max_element(bbox_l.begin(), bbox_l.end()) : 0;
  double min_l_pos =
      (!bbox_l_pos.empty())
          ? *std::min_element(bbox_l_pos.begin(), bbox_l_pos.end())
          : 0;
  double max_l_pos =
      (!bbox_l_pos.empty())
          ? *std::max_element(bbox_l_pos.begin(), bbox_l_pos.end())
          : 0;
  double min_l_self =
      (!bbox_l_self.empty())
          ? *std::min_element(bbox_l_self.begin(), bbox_l_self.end())
          : 0;
  double max_l_self =
      (!bbox_l_self.empty())
          ? *std::max_element(bbox_l_self.begin(), bbox_l_self.end())
          : 0;
  double min_l_self_pos =
      (!bbox_l_self_pos.empty())
          ? *std::min_element(bbox_l_self_pos.begin(), bbox_l_self_pos.end())
          : 0;
  double max_l_self_pos =
      (!bbox_l_self_pos.empty())
          ? *std::max_element(bbox_l_self_pos.begin(), bbox_l_self_pos.end())
          : 0;

  if (item.l >= lat_offset) {
    item.d_path = ((item.l - lat_offset) * (min_l - lat_offset) > 0)
                      ? (min_l - lat_offset)
                      : 0;

    if (!bbox_l_pos.empty()) {
      item.d_path_pos = ((item.l - lat_offset) * (min_l_pos - lat_offset) > 0)
                            ? (min_l_pos - lat_offset)
                            : 0;
    } else {
      item.d_path_pos = 100;
    }
  } else {
    item.d_path = ((item.l - lat_offset) * (max_l - lat_offset) > 0)
                      ? (max_l - lat_offset)
                      : 0;

    if (!bbox_l_pos.empty()) {
      item.d_path_pos = ((item.l - lat_offset) * (max_l_pos - lat_offset) > 0)
                            ? (max_l_pos - lat_offset)
                            : 0;
    } else {
      item.d_path_pos = 100;
    }
  }

  if (item.l >= l0_) {
    item.d_path_self = ((item.l - l0_) * min_l_self > 0) ? min_l_self : 0;

    if (!bbox_l_pos.empty()) {
      item.d_path_self_pos =
          ((item.l - l0_) * min_l_self_pos > 0) ? min_l_self_pos : 0;
    } else {
      item.d_path_self_pos = 100;
    }
  } else {
    item.d_path_self = ((item.l - l0_) * max_l_self > 0) ? max_l_self : 0;

    if (!bbox_l_pos.empty()) {
      item.d_path_self_pos =
          ((item.l - l0_) * max_l_self_pos > 0) ? max_l_self_pos : 0;
    } else {
      item.d_path_self_pos = 100;
    }
  }

  item.d_center_cpath = item.l;
  item.d_max_cpath = max_l;
  item.d_min_cpath = min_l;

  item.s_center = item.s;

  auto it = std::find(bbox_l.begin(), bbox_l.end(), max_l);
  size_t index = (size_t)(it - bbox_l.begin());

  if (index < bbox_s.size()) {
    item.s_max = std::max(bbox_s[index], 0.0);
  }

  it = std::find(bbox_l.begin(), bbox_l.end(), min_l);
  index = (size_t)(it - bbox_l.begin());

  if (index < bbox_s.size()) {
    item.s_min = std::max(bbox_s[index], 0.0);
  }

  item.y_rel = item.y_rel_ori - item.l + item.d_path;
  item.d_path = std::fabs(item.d_path);
  item.d_path_self = std::fabs(item.d_path_self);
  item.d_path_pos = std::fabs(item.d_path_pos);
  item.d_path_self_pos = std::fabs(item.d_path_self_pos);
}

bool TrackletMaintainer::is_potential_lead_one(TrackedObject &item,
                                               double v_ego) {
  std::array<double, 5> xp1{1.5, 5.0, 10.0, 40.0, 60.0 + v_ego / 1.2};
  std::array<double, 5> fp1{0.8, 1.0, 0.8, 0.7, 0.0};
  double t_lookahead = interp(item.d_rel, xp1, fp1);

  std::array<double, 3> xp2{1.5, 5.0, 10.0};
  std::array<double, 3> fp2{-0.85, -0.9, -0.85};
  double max_d_offset = interp(item.d_rel, xp2, fp2);

  if (std::fabs(item.d_path_pos) < 3.0 && item.v_lat < -0.2) {
    item.lat_coeff = std::min(item.lat_coeff * 1.035, 1.5);
  } else {
    item.lat_coeff = std::max(item.lat_coeff * 0.9, 1.0);
  }

  double lat_corr =
      std::max(max_d_offset, std::min(0.0, t_lookahead * item.v_lat)) *
      item.lat_coeff;

  if (item.oncoming && item.d_rel >= 0) {
    lat_corr = 0.0;
  }

  double lead_d_path_thr;
  std::array<double, 5> xp3{0, 5, 40, 80, 120};
  std::array<double, 5> fp3{1.28, 1.28, 1.21, 1.19, 1.1};
  double result = interp(item.d_rel, xp3, fp3);

  double d_path = std::max(item.d_path_pos + lat_corr, 0.0);
  if (item.type < 10000 && item.v_lead > 0.2) {
    lead_d_path_thr = result;
  } else if (item.type < 10000) {
    lead_d_path_thr = result - 0.25;
  } else {
    lead_d_path_thr = result - 0.2;
  }

  if (item.oncoming && item.d_rel > 50) {
    lead_d_path_thr = 0.3;
  }

  item.leadone_confidence_cnt = 0.0;
  item.is_lead = d_path < lead_d_path_thr && item.d_rel >= 0.0;

  return item.is_lead;
}

bool TrackletMaintainer::is_potential_lead_two(TrackedObject &item,
                                               const TrackedObject *lead_one) {
  if (lead_one == nullptr) {
    return false;
  }

  bool is_diff_d_rel = (std::fabs(item.d_rel - lead_one->d_rel) > 5.0 ||
                        lead_one->type == 20001 || lead_one->type == 10001);
  bool is_diff_v_rel = (std::fabs(item.v_rel - lead_one->v_rel) > 1.0);
  bool is_diff_y_rel = (std::fabs(item.y_rel - lead_one->y_rel) > 1.0);

  item.leadtwo_confidence_cnt = 0.0;
  return (is_diff_d_rel || is_diff_v_rel || is_diff_y_rel) && item.d_rel > 0.0;
}

bool TrackletMaintainer::is_potential_temp_lead_one(TrackedObject &item,
                                                    double v_ego,
                                                    bool refline_update) {
  std::array<double, 5> xp1{1.5, 5.0, 10.0, 40.0, 60.0 + v_ego / 1.2};
  std::array<double, 5> fp1{0.8, 1.0, 0.8, 0.7, 0.0};
  double t_lookahead = interp(item.d_rel, xp1, fp1);

  std::array<double, 3> xp2{1.5, 5.0, 10.0};
  std::array<double, 3> fp2{-0.85, -0.9, -0.85};
  double max_d_offset = interp(item.d_rel, xp2, fp2);

  double lat_corr =
      std::max(max_d_offset, std::min(0.0, t_lookahead * item.v_lat_self));
  if (item.oncoming && item.d_rel >= 0) {
    lat_corr = 0.0;
  }

  double d_path_self = std::max(item.d_path_self_pos + lat_corr, 0.0);

  double lead_d_path_thr;
  std::array<double, 5> xp3{0, 5, 40, 80, 120};
  std::array<double, 5> fp3{1.28, 1.28, 1.21, 1.19, 1.1};
  double result = interp(item.d_rel, xp3, fp3);

  if (item.type < 10000 && item.v_lead > 0.2) {
    lead_d_path_thr = result - 0.15;
  } else if (item.type < 10000) {
    lead_d_path_thr = result - 0.25;
  } else {
    lead_d_path_thr = result - 0.2;
  }

  if (item.oncoming && item.d_rel > 50) {
    lead_d_path_thr = 0.3;
  }

  bool is_get_threshold = (d_path_self < lead_d_path_thr);
  bool is_in_range = false;
  if (refline_update) {
    is_in_range =
        ((item.l > item.l0 && item.l < 0) || (item.l < item.l0 && item.l > 0));
  }

  item.tleadone_confidence_cnt = 0.0;
  item.is_temp_lead = (is_get_threshold || is_in_range) && item.d_rel >= 0.0;

  return item.is_temp_lead;
}

bool TrackletMaintainer::is_potential_temp_lead_two(
    TrackedObject &item, const TrackedObject *temp_lead_one) {
  if (temp_lead_one == nullptr) {
    return false;
  }

  bool is_diff_d_rel =
      (std::fabs(item.d_rel - temp_lead_one->d_rel) > 5.0 ||
       temp_lead_one->type == 20001 || temp_lead_one->type == 10001);
  bool is_diff_v_rel = (std::fabs(item.v_rel - temp_lead_one->v_rel) > 1.0);
  bool is_diff_y_rel = (std::fabs(item.y_rel - temp_lead_one->y_rel) > 1.0);

  item.tleadtwo_confidence_cnt = 0.0;
  return (is_diff_d_rel || is_diff_v_rel || is_diff_y_rel) && item.d_rel > 0.0;
}

bool TrackletMaintainer::is_potential_avoiding_car(
    TrackedObject &item, const TrackedObject *lead_one, double v_ego,
    double lane_width, int scenario, bool tleft_lane, bool rightest_lane,
    double dist_intersect, double intersect_length, bool isRedLightStop) {
  item.is_ncar = false;
  double ego_car_width = 2.2;
  double lat_safety_buffer = 0.2;
  double l_ego = ego_state_.ego_frenet.y;
  double dist_limit;

  std::array<double, 3> xp{20, 40, 60};
  std::array<double, 3> fp{0.3, 0.12, 0.09};
  double near_car_d_lane_thr = interp(item.d_rel, xp, fp);

  bool is_not_full_in_road = (std::fabs(item.y_rel) > 0.0);
  bool is_in_range = (item.d_rel < 20.0 && item.v_rel < 1.0);
  bool is_about_to_enter_range =
      (item.d_rel < std::min(std::fabs(15.0 * item.v_rel), 60.0) &&
       item.v_rel < -2.5);
  bool cross_solid_line = false;

  if (is_not_full_in_road && (is_in_range || is_about_to_enter_range)) {
    if (std::fabs(item.d_min_cpath - DBL_MAX) > 1.0 &&
        std::fabs(item.d_max_cpath - DBL_MAX) > 1.0) {
      if (item.type != 20001) {
        dist_limit = lane_width * 0.5 + near_car_d_lane_thr;
      } else {
        dist_limit = lane_width * 0.5 - 0.2;
      }
      bool is_same_side = ((item.d_min_cpath > 0 && item.d_max_cpath > 0) ||
                           (item.d_min_cpath <= 0 && item.d_max_cpath <= 0));

      bool is_need_avoid =
          (item.d_max_cpath < 0 && std::fabs(item.d_max_cpath) < dist_limit) ||
          (item.d_min_cpath > 0 && item.d_min_cpath < dist_limit);

      bool can_avoid =
          (item.d_min_cpath >
           std::min(((ego_car_width + lat_safety_buffer) - lane_width / 2),
                    0.9)) ||
          (item.d_max_cpath <
           std::max((lane_width / 2 - (ego_car_width + lat_safety_buffer)),
                    -0.9));

      if (std::fabs(dist_intersect - 1000) < 1e-5 && lane_width > 5. &&
          lead_one != nullptr) {
        can_avoid = item.d_min_cpath > 0.9 || item.d_max_cpath < -0.5;
      }

      cross_solid_line =
          (((item.d_min_cpath <=
             std::min(((ego_car_width + lat_safety_buffer) - lane_width / 2),
                      0.9)) &&
            ((item.d_min_cpath >= 0 && !rightest_lane && item.type < 10000) ||
             (item.d_max_cpath > 0 && item.type == 20001))) ||
           ((item.d_max_cpath >=
             std::max((lane_width / 2 - (ego_car_width + lat_safety_buffer)),
                      -0.9)) &&
            ((item.d_max_cpath <= 0 && !tleft_lane && item.type < 10000) ||
             (item.d_min_cpath < 0 && item.type == 20001)))) &&
          lead_one != nullptr && item.track_id == lead_one->track_id &&
          item.v_lead < 0.3 && abs(item.v_lat) < 0.2 &&
          dist_intersect - item.d_rel >= -5 &&
          dist_intersect - item.d_rel < 50.0 &&
          (!isRedLightStop || item.type == 20001);

      item.is_ncar =
          (is_same_side && is_need_avoid && can_avoid) ||
          (rightest_lane && item.d_max_cpath < 0 &&
           std::fabs(item.d_max_cpath) < dist_limit && item.v_lead < 0.5) ||
          cross_solid_line;
    }
  }

  double ncar_count;
  if (item.type < 10000) {
    if (item.d_rel >= 20) {
      std::array<double, 2> xp2{-7.5, -2.5};
      std::array<double, 2> fp2{5, 20};
      ncar_count = interp(item.v_rel, xp2, fp2);
    } else {
      std::array<double, 4> xp2{-5, -2.499, 0, 1};
      std::array<double, 4> fp2{2, 3, 4, 20};
      std::array<double, 4> xp3{0, 2, 5, 10};
      std::array<double, 4> fp3{4, 3, 2, 0};

      ncar_count = interp(item.v_rel, xp2, fp2) + interp(item.v_ego, xp3, fp3);
    }
  } else if (item.type != 20001) {
    std::array<double, 2> xp2{20, 40};
    std::array<double, 2> fp2{5, 10};
    ncar_count = interp(item.d_rel, xp2, fp2);
  } else {
    ncar_count = 1.0;
  }
  if (cross_solid_line) {
    std::array<double, 4> xp4{0, 0.4, 0.8};
    std::array<double, 4> fp4{30, 20, 5};
    ncar_count += interp(
        std::min(std::fabs(item.d_min_cpath), std::fabs(item.d_max_cpath)), xp4,
        fp4);
  }

  double curr_time = PlanningContext::Instance()
                         ->planning_status()
                         .planning_result.next_timestamp_sec;
  double gap = (std::fabs(item.last_recv_time) < 1e-5)
                   ? 0.1
                   : (curr_time - item.last_recv_time);
  int count = (int)((gap + 0.01) / 0.1);

  if (item.is_ncar) {
    if (item.trajectory.intersection == 0 ||
        (item.type < 10000 && std::fabs(item.v_lead) < 0.5 &&
         item.v_lat > -0.2 && item.v_lat < 0.3) ||
        (item.type > 10000 && (std::fabs(item.v_lat) < 0.3)) || rightest_lane) {
      if ((item.v_lat > -0.5 && item.v_lat < 0.3 && item.type < 10000) ||
          (std::fabs(item.v_lat) < 0.3 && item.type > 10000)) {
        item.ncar_count = std::min(item.ncar_count + gap, 100 * 0.1);
      }
    } else {
      item.ncar_count = std::max(
          item.ncar_count -
              5 * (int)(std::fabs(item.v_lat) / 0.3 + 0.5) * count * 0.1,
          0.0);
    }

    if (item.ncar_count < ncar_count * 0.1 &&
        item.timestamp > item.close_time + 2.5) {
      item.is_ncar = false;
      return false;
    } else if (item.ncar_count >= ncar_count * 0.1) {
      if (item.ncar_count_in == false) {
        item.ncar_count = 100 * 0.1;
      }

      item.close_time = item.timestamp;
      return true;
    }
  } else {
    item.ncar_count = std::max(item.ncar_count - 2 * count * 0.1, 0.0);
    if (item.v_rel > 1.5) {
      item.ncar_count = std::max(item.ncar_count - 5 * count * 0.1, 0.0);
    }

    if (item.trajectory.intersection > 0) {
      item.ncar_count = std::max(item.ncar_count - 10 * count * 0.1, 0.0);
    }

    if (item.d_max_cpath > 0 && item.d_min_cpath < 0) {
      item.ncar_count = 0;
    }

    if (item.ncar_count > 60 * 0.1) {
      item.ncar_count_in = true;
      return true;
    } else {
      item.ncar_count = 0;
      item.ncar_count_in = false;
      return false;
    }
  }

  return false;
}

bool TrackletMaintainer::is_leadone_potential_avoiding_car(
    TrackedObject *lead_one, int scenario, double lane_width,
    bool rightest_lane, double dist_intersect, bool isRedLightStop) {
  if (lead_one == nullptr) {
    return false;
  }

  bool is_in_range = (lead_one->d_rel < 20.0 && lead_one->v_rel < 1.0);
  bool is_about_to_enter_range =
      (lead_one->d_rel < std::min(std::fabs(15.0 * lead_one->v_rel), 60.0)) &&
      lead_one->v_rel < -2.5;

  if (is_in_range || is_about_to_enter_range) {
    double ego_car_width = 2.2;
    double near_end_pos = 0.5 * lane_width - 0.7 * (lane_width - ego_car_width);
    double far_end_pos = 0.5 * lane_width + 0.2;

    bool is_in_avoid_range_by_nearest_point =
        lead_one->d_path >= near_end_pos && lead_one->d_path < far_end_pos;

    bool is_in_avoid_range_by_nearest_line_in_left =
        lead_one->d_min_cpath >= near_end_pos &&
        lead_one->d_min_cpath < far_end_pos &&
        lead_one->d_max_cpath >= near_end_pos;

    bool is_in_avoid_range_by_nearest_line_in_right =
        lead_one->d_max_cpath > -far_end_pos &&
        lead_one->d_max_cpath <= -near_end_pos &&
        lead_one->d_min_cpath <= -near_end_pos;

    lead_one->is_avd_car =
        (lead_one->is_avd_car) &&
        (is_in_avoid_range_by_nearest_point ||
         is_in_avoid_range_by_nearest_line_in_left ||
         is_in_avoid_range_by_nearest_line_in_right || rightest_lane ||
         (dist_intersect - lead_one->d_rel < 50 &&
          dist_intersect - lead_one->d_rel >= -5 &&
          (!isRedLightStop || lead_one->type == 20001)));
  }

  return lead_one->is_avd_car;
}

void TrackletMaintainer::select_lead_cars(
    const std::vector<TrackedObject *> &tracked_objects, LeadCars &lead_cars) {

  TrackedObject *lead_one = nullptr;
  TrackedObject *lead_two = nullptr;
  TrackedObject *temp_lead_one = nullptr;
  TrackedObject *temp_lead_two = nullptr;

  double v_ego = ego_state_.ego_vel;

  for (auto tr : tracked_objects) {
    if (tr->is_lead == false) {
      continue;
    }

    if (lead_one == nullptr || tr->d_rel < lead_one->d_rel) {
      lead_one = tr;
    }
  }

  for (auto tr : tracked_objects) {
    if (tr->is_lead == false || lead_one == tr ||
        is_potential_lead_two(*tr, lead_one) == false) {
      continue;
    }

    if (lead_two == nullptr || tr->d_rel < lead_two->d_rel) {
      lead_two = tr;
    }
  }

  for (auto tr : tracked_objects) {
    if (tr->is_lead == true ||
        is_potential_temp_lead_one(*tr, v_ego, simple_refline_.has_update()) ==
            false) {
      continue;
    }

    if (temp_lead_one == nullptr || tr->d_rel < temp_lead_one->d_rel) {
      temp_lead_one = tr;
    }
  }

  for (auto tr : tracked_objects) {
    if (tr->is_lead == true || temp_lead_one == tr ||
        is_potential_temp_lead_one(*tr, v_ego, simple_refline_.has_update()) ==
            false ||
        is_potential_temp_lead_two(*tr, temp_lead_one) == false) {
      continue;
    }

    if (temp_lead_two == nullptr || tr->d_rel < temp_lead_two->d_rel) {
      temp_lead_two = tr;
    }
  }

  lead_cars.lead_one = lead_one;
  lead_cars.lead_two = lead_two;
  lead_cars.temp_lead_one = temp_lead_one;
  lead_cars.temp_lead_two = temp_lead_two;
}

void TrackletMaintainer::set_default_value(
    const std::vector<TrackedObject *> &tracked_objects) {

  for (auto tr : tracked_objects) {
    if (std::fabs(tr->d_center_cpath - DBL_MAX) < 1.0) {
      tr->d_center_cpath = 100;
    }

    if (std::fabs(tr->d_max_cpath - DBL_MAX) < 1.0) {
      tr->d_max_cpath = 100;
    }

    if (std::fabs(tr->d_min_cpath - DBL_MAX) < 1.0) {
      tr->d_min_cpath = 100;
    }

    if (std::fabs(tr->y_rel_ori - DBL_MAX) < 1.0) {
      tr->y_center_rel = tr->y_rel;
    } else {
      tr->y_center_rel = tr->y_rel_ori;
    }

    if (std::fabs(tr->timestamp - DBL_MAX) < 1.0) {
      tr->timestamp = 0;
    }

    if (std::fabs(tr->length - DBL_MAX) < 1.0) {
      tr->length = 5.0;
    }

    if (std::fabs(tr->width - DBL_MAX) < 1.0) {
      tr->width = 2.2;
    }

    if (std::fabs(tr->theta - DBL_MAX) < 1.0) {
      tr->theta = 0.0;
    }
  }
}

} // namespace msquare
