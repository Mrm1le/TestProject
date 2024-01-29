#include "planner/behavior_planner/parking/speed_margin_limiter.h"
#include "common/config/vehicle_param.h"
#include "common/math/math_utils.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/local_log.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include <algorithm>

namespace msquare {
namespace parking {

double SpeedMarginLimiter::getTotals() {
  if (vec_sv_.empty()) {
    return 0.0;
  }
  return vec_sv_.begin()->first;
}

bool SpeedMarginLimiter::filterSV() {
  vec_sv_.clear();
  if (traj_.size() < 2) {
    return false;
  }
  limitVByObstacle(vec_sv_);
  if (is_debug_) {
    sv_list_debug_array_.push_back(vec_sv_);
  }

  limitVByCurvature(vec_sv_);
  if (is_debug_) {
    // sv_list_debug_array_.push_back(vec_sv_);
  }

  LOCAL_LOG(LOCAL_DEBUG, "filterByMinV:");
  change_index_ = filterByMinV(para_.window_s_size, vec_sv_);
  if (is_debug_) {
    // push back filtered vec_sv by min v
    // sv_list_debug_array_.push_back(vec_sv_);
  }

  LOCAL_LOG(LOCAL_DEBUG, "filterByMinS:");
  bool filter_res = filterByMinS(vec_sv_, change_index_);
  if (is_debug_) {
    // push back filtered vec_sv by min s
    sv_list_debug_array_.push_back(vec_sv_);
  }

  filterByRemainS(vec_sv_);
  if (is_debug_) {
    // push back filtered vec_sv by min s
    sv_list_debug_array_.push_back(vec_sv_);
  }
  // limitVAtStart(vec_sv_);
  if (is_debug_) {
    // push back filtered vec_sv by min s
    // sv_list_debug_array_.push_back(vec_sv_);
  }

  adaptDynamicPlanning(vec_sv_);
  if (is_debug_) {
    // push back filtered vec_sv by adaptDynamicPlanning
    sv_list_debug_array_.push_back(vec_sv_);
  }

  return true;
}

bool SpeedMarginLimiter::limitVAtStart(VecSV &vec_sv) {
  if (vec_sv.size() < 2) {
    return true;
  }
  VecSV::iterator last_iter = vec_sv.begin();
  VecSV::iterator cur_iter = last_iter + 1;
  if (last_iter->first < para_.control_take_over_remain_s) {
    return true;
  }

  double eps = 1e-6;

  // flat scenario
  while (cur_iter != vec_sv.end() &&
         std::abs(cur_iter->second - last_iter->second) < eps) {
    cur_iter++;
  }
  if (cur_iter == vec_sv.end() || last_iter->second < eps ||
      cur_iter->second < eps) {
    // this should wont run here
    return false;
  }
  double flat_elapse_time =
      (last_iter->first - (cur_iter + 1)->first) / last_iter->second;
  if (flat_elapse_time > para_.min_duration_filter_time) {
    return true;
  }

  // in principle, there is no up scenario
  if (cur_iter->second - last_iter->second > eps) {
    LOCAL_LOG(LOCAL_DEBUG, "up scenario:");
    return true;
  }

  // down scenario
  flat_elapse_time = (last_iter->first - cur_iter->first) / cur_iter->second;
  while (flat_elapse_time < para_.min_duration_filter_time) {
    // can not find a flat, quit early
    if (std::abs(cur_iter->second - para_.remain_s_min_velocity) < 1e-3) {
      break;
    }
    if (cur_iter->second < para_.remain_s_min_velocity) {
      if (cur_iter != last_iter) {
        cur_iter--;
      }
      break;
    }
    cur_iter++;
    if (cur_iter == vec_sv.end()) {
      break;
    }
    flat_elapse_time = (last_iter->first - cur_iter->first) / cur_iter->second;
  }
  if (cur_iter == vec_sv.end()) {
    // this should wont run here
    return true;
  }

  // reset v
  while (last_iter != cur_iter) {
    last_iter->second = cur_iter->second;
    last_iter++;
  }

  return true;
}

bool SpeedMarginLimiter::filterByRemainS(VecSV &vec_sv) {
  if (vec_sv.size() < 2) {
    return true;
  }
  VecSV::iterator last_iter = vec_sv.begin();
  VecSV::iterator cur_iter = last_iter + 1;

  double eps = 1e-6;
  // remove accelaration segment when s < para_.const_speed_min_s
  while (cur_iter != vec_sv.end()) {
    if (cur_iter->first > para_.const_speed_min_s) {
      last_iter++;
      cur_iter++;
      continue;
    }

    // down scenario, break
    if (last_iter->second - cur_iter->second > eps) {
      LOCAL_LOG(LOCAL_DEBUG, "down scenario:");
      break;
    }

    // up scenario, find last flat
    if (cur_iter->second - last_iter->second > eps) {
      LOCAL_LOG(LOCAL_DEBUG, "up scenario:");
      while (last_iter != vec_sv.begin() &&
             cur_iter->second - last_iter->second > eps) {
        last_iter--;
        cur_iter--;
      }
      break;
    }
    /* flat scenario */
    // break;
    LOCAL_LOG(LOCAL_DEBUG, "flat scenario:");
    VecSV::iterator cur_back_iter = cur_iter;
    double flat_min_s = para_.min_duration_filter_time * cur_iter->second;
    while (last_iter != vec_sv.begin() &&
           std::abs(cur_iter->second - last_iter->second) < eps &&
           last_iter->first - cur_back_iter->first < flat_min_s) {
      last_iter--;
      cur_iter--;
    }
    if (last_iter == vec_sv.begin()) {
      break;
    }

    //  flat is enough
    if (std::abs(cur_iter->second - last_iter->second) < eps &&
        last_iter->first - cur_back_iter->first > flat_min_s) {
      LOCAL_LOG(LOCAL_DEBUG, "flat is enough");
      break;
    }

    // flat is not enough
    // before flat is down
    LOCAL_LOG(LOCAL_DEBUG, "flat is not enough");
    if (last_iter->second > cur_iter->second) {
      last_iter = cur_iter;
      cur_iter = last_iter - 1;
      break;
    }
    // flat is not enough
    // before flat is up, find the last flat
    last_iter--;
    cur_iter--;
    while (last_iter != vec_sv.begin() &&
           std::abs(cur_iter->second - last_iter->second) > eps) {
      last_iter--;
      cur_iter--;
    }
    break;
  }

  while (cur_iter != vec_sv.end()) {
    if (cur_iter->second > last_iter->second) {
      cur_iter->second = last_iter->second;
    }
    last_iter++;
    cur_iter++;
  }

  cur_iter = vec_sv.begin();
  while (cur_iter != vec_sv.end()) {
    if (cur_iter->first > para_.const_speed_min_s) {
      cur_iter++;
      continue;
    }
    double v_remain_s = getVByRemains(cur_iter->first);
    cur_iter->second = std::min(v_remain_s, cur_iter->second);
    cur_iter++;
  }

  //  add anchor
  if (para_.control_take_over_remain_s < para_.lower_const_speed_min_s) {
    return false;
  }
  if (para_.lower_const_speed_min_s < para_.traj_end_speed_limit_s) {
    return false;
  }

  // return true;

  std::vector<double> ss = {// para_.control_take_over_remain_s,
                            // para_.lower_const_speed_min_s,
                            para_.traj_end_speed_limit_s};

  VecSV anchor;
  double ep = 1e-4;
  for (int i = 0; i < ss.size(); i++) {
    anchor.emplace_back(ss[i] + ep, getVByRemains(ss[i] + ep));
    // anchor.emplace_back(ss[i], getVByRemains(ss[i]));
    anchor.emplace_back(ss[i] - ep, getVByRemains(ss[i] - ep));
  }

  for (int i = 0; i < anchor.size(); i++) {
    last_iter = vec_sv.begin();
    cur_iter = last_iter + 1;
    while (cur_iter != vec_sv.end()) {
      if (cur_iter->first > anchor[i].first) {
        last_iter++;
        cur_iter++;
        continue;
      }
      VecSV::iterator pose_iter =
          last_iter->first < anchor[i].first ? last_iter : cur_iter;

      vec_sv.insert(pose_iter, anchor[i]);
      break;
    }
  }

  // remove accelaration segment when s < para_.const_speed_min_s

  return true;
}

bool SpeedMarginLimiter::limitVByCurvature(VecSV &vec_sv) {
  if (curvatures_.size() != vec_sv.size()) {
    // MSD_LOG(ERROR, "limitVByCurvature:curvatures_.size()=%d,
    // vec_sv.size()=%d",curvatures_.size(), vec_sv.size());
    return false;
  }
  if (curvatures_.size() < 2) {
    return true;
  }
  std::vector<double>::const_iterator iter_last = curvatures_.begin();
  std::vector<double>::const_iterator iter_cur = curvatures_.begin();
  iter_cur++;
  for (unsigned int index = 1; index < curvatures_.size(); index++) {
    double delta_cur = std::abs(curvatures_[index] - curvatures_[index - 1]);

    if (delta_cur > para_.curvature_change_threshold) {
      vec_sv[index].second =
          std::min(vec_sv[index].second, para_.curvature_limit_v);
    }
  }
  return true;
}

bool SpeedMarginLimiter::limitVByObstacle(VecSV &vec_sv_raw) {
  double filter_square_dis = 2 * para_.length * para_.length;
  std::vector<Pose2D>::const_iterator iter = traj_.begin();
  double s = 0.0;
  bool is_init = true;
  for (auto &p : traj_) {
    double ego_sin = std::sin(p.theta);
    double ego_cos = std::cos(p.theta);
    planning_math::Vec2d center_line_start(
        p.x + (para_.is_reverse ? para_.back_p1 * ego_cos
                                : para_.front_p1 * ego_cos),
        p.y + (para_.is_reverse ? para_.back_p1 * ego_sin
                                : para_.front_p1 * ego_sin));
    planning_math::Vec2d center_line_end(
        p.x + (para_.is_reverse ? para_.back_p2 * ego_cos
                                : para_.front_p2 * ego_cos),
        p.y + (para_.is_reverse ? para_.back_p2 * ego_sin
                                : para_.front_p2 * ego_sin));

    planning_math::Box2d ego_box(
        planning_math::LineSegment2d(center_line_start, center_line_end),
        para_.width_wo_rearview_mirror + 2 * para_.side_add);
    if (is_debug_) {
      std::vector<msquare::planning_math::Vec2d> corners =
          ego_box.GetAllCorners();
      traj_debug_.push_back(corners);
    }
    planning_math::Box2d ego_box_large(
        planning_math::LineSegment2d(center_line_start, center_line_end),
        para_.width +
            2 * std::max(para_.obstacle_consider_max,
                         para_.mirror_extrude_length + para_.side_add));
    planning_math::Vec2d mirror_center_line_start(
        p.x + para_.mirror_p1 * ego_cos, p.y + para_.mirror_p1 * ego_sin);
    planning_math::Vec2d mirror_center_line_end(
        p.x + para_.mirror_p2 * ego_cos, p.y + para_.mirror_p2 * ego_sin);
    planning_math::Box2d mirror_ego_box(
        planning_math::LineSegment2d(mirror_center_line_start,
                                     mirror_center_line_end),
        para_.width + 2 * para_.side_add);
    double min_dis = 10;
    // for (auto &box: para_.obs_boxes) {
    //     if(min_dis < 1e-6){break;}
    //     if(!ego_box_large.HasOverlap(box)){
    //     continue;
    //     }
    //     if(mirror_ego_box.HasOverlap(box)){
    //         min_dis = 0.0;
    //         break;
    //     }
    //     double dis = ego_box.DistanceTo(box);
    //     if(dis < min_dis){
    //         min_dis = dis;
    //     }
    // }
    // MSD_LOG(ERROR, "limitVByCurvature: box obstacle min_dis = %.3f",
    // min_dis);
    for (auto &obs_p : para_.obs_pts) {
      if (min_dis < 1e-6) {
        break;
      }
      double dis_to_center =
          planning_math::sum_square(obs_p.x() - p.x, obs_p.y() - p.y);
      if (dis_to_center > filter_square_dis) {
        continue;
      }
      if (!ego_box_large.IsPointIn(obs_p)) {
        continue;
      }
      if (mirror_ego_box.IsPointIn(obs_p)) {
        min_dis = 0.0;
        break;
      }
      if (ego_box.IsPointIn(obs_p)) {
        min_dis = 0.0;
        break;
      }
      double dis = ego_box.DistanceTo(obs_p);
      if (dis < min_dis) {
        min_dis = dis;
      }
    }

    for (auto &line : para_.obs_lines) {
      if (min_dis < 1e-6) {
        break;
      }
      if (!ego_box_large.HasOverlap(line)) {
        continue;
      }
      if (mirror_ego_box.HasOverlap(line)) {
        min_dis = 0.0;
        break;
      }
      double dis = ego_box.DistanceTo(line);
      if (dis < min_dis) {
        min_dis = dis;
      }
    }

    s += std::sqrt(planning_math::sum_square(p.x - iter->x, p.y - iter->y));
    if (is_init) {
      is_init = false;
    } else {
      iter++;
    }
    min_dis = std::max(0.0, min_dis);

    double v_limit = min_dis * para_.v_slop + para_.obstacle_consider_min_v;
    v_limit = std::min(v_limit, para_.max_v);
    vec_sv_raw.emplace_back(s, v_limit);
  }

  // convert length-v to remain_s-v
  double all_length = s;
  for (auto &side_dis : vec_sv_raw) {
    side_dis.first = std::max(0.0, all_length - side_dis.first);
  }

  return true;
}

bool SpeedMarginLimiter::getVConsiderLast(double s, double last_v, double &v,
                                          SpeedRes &res) {

  bool is_valid = getV(s, v, res);
  return is_valid;
  if (!is_valid || s > para_.forbid_acc_max_s) {
    return is_valid;
  }
  if (!para_.is_reverse || !para_.is_apa_not_parallel ||
      !para_.is_last_segment) {
    return is_valid;
  }

  if (!para_.is_dynamic_planning) {
    return is_valid;
  }
  if (last_v < para_.control_take_over_speed) {
    return is_valid;
  }

  if (s < 3 * para_.traj_end_speed_limit_s) {
    return is_valid;
  }

  double abs_last_v = std::abs(last_v);
  if (v > abs_last_v) {
    v = last_v;
    return is_valid;
  }

  double delta_v = para_.forbid_acc_delta_v;
  if (last_v - delta_v > v) {
    v = last_v - delta_v;
  }

  return true;
}

bool SpeedMarginLimiter::adaptDynamicPlanning(VecSV &vec_sv) {
  if (!para_.is_last_segment) {
    return limitVAtStart(vec_sv_);
  }
  if (!para_.is_dynamic_planning && !para_.update_wheel_stop) {
    return limitVAtStart(vec_sv_);
  }

  if (vec_sv.size() < 2) {
    return false;
  }

  double delta_s = std::max(para_.last_delta_s, 0.0);

  VecSV::iterator last_iter = vec_sv.begin();
  // forbidden acc
  while (last_iter != vec_sv.end()) {
    last_iter->second = std::min(last_iter->second, para_.last_v);
    last_iter++;
  }

  double eps = 1e-3;
  if (para_.last_v < para_.remain_s_min_velocity + eps) {
    return false;
  }

  // equal
  if (std::abs(para_.last_v - last_iter->second) < eps) {
    double travel_s = para_.last_res.travel_s + delta_s + last_iter->first;
    vec_sv.insert(vec_sv.begin(), {travel_s, para_.last_v});
    return true;
  }

  last_iter = vec_sv.begin();
  VecSV::iterator cur_iter = last_iter;
  double traveled_s = para_.last_res.travel_s;
  double cur_needed_s = para_.min_duration_filter_time * para_.last_v;
  cur_needed_s -= traveled_s;
  cur_needed_s = std::max(0.0, cur_needed_s);
  while (cur_iter != vec_sv.end()) {
    //  too close to target point
    if (cur_iter->first < para_.traj_end_speed_limit_s) {
      return false;
    }

    double last_to_cur_s = last_iter->first - cur_iter->first + delta_s;

    /* equal */
    if (std::abs(cur_iter->second - para_.last_v) < eps) {
      if (last_to_cur_s < cur_needed_s) {
        //  s is not enough
        cur_iter++;
        continue;
      }

      // rewrite velocity
      while (cur_iter >= last_iter) {
        cur_iter->second = para_.last_v;
        cur_iter--;
      }

      vec_sv.insert(vec_sv.begin(),
                    {last_iter->first + traveled_s + delta_s, para_.last_v});
      break;
    }
    /* not equal, then velocity is smaller */
    double temp_v = cur_iter->second;
    double last_v = para_.last_v;
    double last_v_square = para_.last_v * para_.last_v;

    double flat1_s = para_.last_res.type == DEC ? 0.0 : cur_needed_s;
    double dec_s = (para_.last_v * para_.last_v - temp_v * temp_v) /
                   (2 * para_.remain_s_planning_deceleration);
    double duration_time = para_.min_duration_filter_time;
    if (temp_v < para_.remain_s_min_velocity + eps) {
      duration_time = 0.5 * para_.min_duration_filter_time;
    }
    double flat2_s = duration_time * temp_v;
    double all_needed_s = flat1_s + dec_s + flat2_s;
    if (last_to_cur_s < all_needed_s) {
      //  s is not enough
      cur_iter++;
      continue;
    }

    double dec_end_s = flat1_s + dec_s;
    double dec_begin_s = flat1_s;

    // rewrite velocity
    while (cur_iter >= last_iter) {
      double last_to_temp_s = last_iter->first - cur_iter->first + delta_s;
      if (last_to_temp_s > dec_end_s) {
        cur_iter->second = temp_v;
      } else if (last_to_temp_s > dec_begin_s) {
        double dec_temp_s = last_to_temp_s - dec_begin_s;
        cur_iter->second =
            std::sqrt(last_v_square -
                      2 * para_.remain_s_planning_deceleration * dec_temp_s);
      } else {
        cur_iter->second = last_v;
      }
      cur_iter--;
    }

    if (para_.last_res.type == CONST_SPEED) {
      VecSV temp_vec = {{last_iter->first + traveled_s + delta_s, para_.last_v},
                        {last_iter->first + delta_s, para_.last_v}};
      vec_sv.insert(vec_sv.begin(), temp_vec.begin(), temp_vec.end());
    } else {
      vec_sv.insert(vec_sv.begin(), {last_iter->first + delta_s, para_.last_v});
    }

    break;
  }

  if (cur_iter == vec_sv.end()) {
    return false;
  }

  return true;
}

bool SpeedMarginLimiter::getV(double s, double &v, SpeedRes &res) {
  res.type = SpeedType::CONST_SPEED;
  res.travel_s = 0.0;
  if (vec_sv_.empty()) {
    v = 0.0;
    return false;
  }
  if (s < para_.traj_end_speed_limit_s) {
    v = 0.0;
    return true;
  }

  double v_remain_s = para_.max_v;
  if (s < para_.lower_const_speed_min_s) {
    v_remain_s = getVByRemains(s);
  }

  if (s > vec_sv_.front().first - 1e-3) {
    v = std::min(vec_sv_.front().second, v_remain_s);
    return true;
  }

  if (s < 1e-3) {
    v = std::min(vec_sv_.back().second, v_remain_s);
    return true;
  }

  // the first which is smaller than s
  // vec_sv_: larger s ----> smaller s
  unsigned int left = 0;
  unsigned int right = vec_sv_.size() - 1;
  unsigned int mid;
  while (left < right) {
    mid = int(0.5 * (left + right));
    if (vec_sv_[mid].first > s) {
      left = mid + 1;
    } else {
      right = mid;
    }
  }

  if (left == 0) {
    v = std::min(vec_sv_.back().second, v_remain_s);
    return true;
  }

  double eps = 1e-3;
  // linear interpolation
  double s1 = vec_sv_[left - 1].first;
  double s2 = vec_sv_[left].first;
  double delta_s = std::max(0.0, s1 - s2);
  if (delta_s < 1e-3) {
    v = vec_sv_[left].second;
  } else {
    double ratio = std::max(0.0, s - s2) / delta_s;
    v = ratio * vec_sv_[left - 1].second + (1 - ratio) * vec_sv_[left].second;
  }

  res.type = v > vec_sv_[left - 1].second ? SpeedType::ACC : DEC;
  if (std::abs(vec_sv_[left].second - vec_sv_[left - 1].second) < eps &&
      std::abs(v - vec_sv_[left - 1].second) < eps) {
    res.type = SpeedType::CONST_SPEED;
  }

  while (left > 1 &&
         std::abs(vec_sv_[left - 2].second - vec_sv_[left - 1].second) < eps) {
    left--;
  }
  res.travel_s = (res.type == SpeedType::CONST_SPEED
                      ? std::abs(vec_sv_[left - 1].first - s)
                      : 0.0);

  return true;
}

bool SpeedMarginLimiter::filterByMinS(
    VecSV &vec_sv, const std::vector<unsigned int> &change_index) {
  if (change_index.empty() || vec_sv.empty()) {
    return false;
  }
  if (change_index.front() != 0) {
    return false;
  }

  std::vector<VelocityStep> v_step_list;
  VecSV::iterator sv_iter = vec_sv.begin();
  std::vector<unsigned int>::const_iterator iter_begin = change_index.begin();
  std::vector<unsigned int>::const_iterator iter_end = change_index.begin();
  unsigned int end_index = vec_sv.size() - 1;
  while (iter_begin != change_index.end()) {

    iter_end = iter_begin + 1;
    if (iter_end == change_index.end()) {
      iter_end--;
    }
    unsigned int i0 = *iter_begin;
    unsigned int i1 = *iter_begin;
    if (*iter_end > 0) {
      i1 = std::max(i1, *iter_end - 1);
    }
    if (iter_begin + 1 == change_index.end() && *iter_end != end_index) {
      i1 = std::max(i1, end_index);
    }
    v_step_list.emplace_back(i0, i1, (sv_iter + i0)->first,
                             (sv_iter + i1)->first, (sv_iter + i0)->second,
                             (sv_iter + i0)->first - (sv_iter + i1)->first);
    iter_begin++;
  }

  unsigned int cur_i = 0;
  unsigned int next_i = 1;
  double eps = 1e-4;

  while (next_i < v_step_list.size()) {
    double current_s_end = v_step_list[cur_i].s1;
    double current_v = v_step_list[cur_i].v;
    double next_s_begin = v_step_list[next_i].s0;
    double next_s_end = v_step_list[next_i].s1;
    double next_v = v_step_list[next_i].v;

    double delta_v = next_v - current_v;

    // same v
    if (std::abs(delta_v) < eps) {
      unsigned int seg_i0 = v_step_list[cur_i].i1;
      unsigned int seg_i1 = v_step_list[next_i].i0;
      if (seg_i0 == seg_i1) {
        cur_i = next_i;
        next_i++;
        continue;
      }

      for (unsigned int seg_i = seg_i0 + 1; seg_i < seg_i1; seg_i++) {
        vec_sv[seg_i].second = current_v;
      }

      cur_i = next_i;
      next_i++;
      continue;
    }

    // different v
    double a =
        delta_v > 0.0 ? para_.acc : -para_.remain_s_planning_deceleration;
    double delta_s = (next_v * next_v - current_v * current_v) / (2 * a);
    double flat_begin_s = current_s_end - delta_s;
    double flat_delta_s = flat_begin_s - next_s_end;
    // else: fill velocity in between s
    // std::cout<<"a:"<<a<<std::endl;
    // std::cout<<"nv:"<<next_v<<std::endl;
    // std::cout<<"current_v:"<<current_v<<std::endl;
    // std::cout<<"delta_s:"<<delta_s<<std::endl;
    // std::cout<<"flat_delta_s:"<<flat_delta_s<<std::endl;
    // std::cout<<"current_s_end:"<<current_s_end<<std::endl;
    // std::cout<<"next_s_end:"<<next_s_end<<std::endl;
    if (flat_delta_s < para_.min_duration_filter_time * next_v) {
      if ((next_i + 1) < v_step_list.size()) {
        next_i++;
        continue;
      }
    }

    // flat_delta_s is large enough
    unsigned int seg_i0 = v_step_list[cur_i].i1;
    unsigned int seg_i1 = v_step_list[next_i].i1;
    for (unsigned int seg_i = seg_i0 + 1; seg_i < seg_i1; seg_i++) {
      if (vec_sv[seg_i].first < flat_begin_s) {
        vec_sv[seg_i].second = next_v;
        continue;
      }
      double slop_delta_s = current_s_end - vec_sv[seg_i].first;
      double temp_0 = 2 * a * slop_delta_s + current_v * current_v;
      double temp_1 = std::max(0.0, temp_0);
      vec_sv[seg_i].second = std::sqrt(temp_1);
    }
    cur_i = next_i;
    next_i++;

    // return true;
  }

  return true;
}

std::vector<unsigned int> SpeedMarginLimiter::filterByMinV(double window_size,
                                                           VecSV &vec_sv) {
  std::vector<unsigned int> change_index;
  VecSV::iterator iter_f = vec_sv.begin();
  VecSV::iterator iter_b = vec_sv.begin();
  VecSV::iterator iter_min = vec_sv.begin();
  VecSV::iterator last_iter_min = vec_sv.end();
  while (iter_f != vec_sv.end()) {
    if (iter_min < iter_f) {
      if (iter_b < iter_f) {
        iter_b = iter_f + 1;
      }
      iter_min = std::min_element(iter_f, iter_b,
                                  [](const SV_PAIR &sv0, const SV_PAIR &sv1) {
                                    return sv0.second < sv1.second;
                                  });
    }

    while (iter_b != vec_sv.end() &&
           std::abs(iter_b->first - iter_f->first) < window_size) {
      // find the min v
      if (iter_b->second < iter_min->second) {
        iter_min = iter_b;
      }
      iter_b++;
    }
    if (iter_min != last_iter_min) {
      if (last_iter_min == vec_sv.end() ||
          std::abs(iter_min->second - last_iter_min->second) > 1e-3 ||
          (iter_min + 1) == vec_sv.end()) {
        last_iter_min = iter_min;
        change_index.push_back((unsigned int)(iter_f - vec_sv.begin()));
      }
    }
    iter_f->second = iter_min->second;
    iter_f++;
  }

  return change_index;
} // end filterByMinV

double SpeedMarginLimiter::getVByRemains(double remain_s) {
  double s1 = para_.const_speed_min_s;
  double s3 = para_.lower_const_speed_min_s;
  s3 = std::max(s3, para_.traj_end_speed_limit_s);
  double v_target = para_.max_v;
  if (remain_s > s1) {
    v_target = para_.max_v;
  } else if (remain_s > para_.control_take_over_remain_s) {
    double delta_s = remain_s - para_.control_take_over_remain_s;
    v_target = std::sqrt(para_.control_take_over_speed *
                             para_.control_take_over_speed +
                         2 * para_.remain_s_planning_deceleration * delta_s);
  } else if (remain_s > s3) {
    double delta_s = remain_s - s3;
    v_target =
        std::sqrt(para_.remain_s_min_velocity * para_.remain_s_min_velocity +
                  2 * para_.control_take_over_acc * delta_s);
    ;
  } else if (remain_s > para_.traj_end_speed_limit_s) {
    double delta_s = remain_s - para_.traj_end_speed_limit_s;
    v_target = para_.remain_s_min_velocity;
  } else {
    v_target = 0.0;
  }

  v_target = std::min(para_.max_v, v_target);
  return v_target;
}

void SpeedMarginPara::init(
    bool _is_reverse, bool _is_last_segment,
    const std::vector<planning_math::Box2d> _obs_boxes,
    const std::vector<planning_math::Vec2d> _obs_pts,
    const std::vector<planning_math::LineSegment2d> _obs_lines) {
  is_last_segment = _is_last_segment;
  is_reverse = _is_reverse;
  obs_boxes = _obs_boxes;
  obs_pts = _obs_pts;
  obs_lines = _obs_lines;

  double front_edge_to_mirror =
      msquare::CarParams::GetInstance()
          ->car_config.car_only_config.front_edge_to_mirror;
  double mirror_length = msquare::CarParams::GetInstance()
                             ->car_config.car_only_config.mirror_length;
  double back_edge_to_center =
      msquare::VehicleParam::Instance()->back_edge_to_center;
  double front_edge_to_center =
      msquare::VehicleParam::Instance()->front_edge_to_center;
  obstacle_consider_max = msquare::CarParams::GetInstance()
                              ->car_config.lon_config.obstacle_consider_max;
  double obstacle_consider_min =
      msquare::CarParams::GetInstance()
          ->car_config.lon_config.obstacle_consider_min;
  width_wo_rearview_mirror =
      msquare::VehicleParam::Instance()->width_wo_rearview_mirror;
  width = msquare::VehicleParam::Instance()->width;
  length = msquare::VehicleParam::Instance()->length;
  obstacle_consider_extend_length =
      msquare::CarParams::GetInstance()
          ->car_config.lon_config.obstacle_consider_extend_length;
  side_add = std::max(obstacle_consider_min, 0.0);

  back_p1 = -back_edge_to_center;
  back_p2 = front_edge_to_center + 2 * side_add;
  front_p1 = -back_edge_to_center;
  front_p2 = front_edge_to_center;
  mirror_p1 = front_edge_to_center - front_edge_to_mirror;
  mirror_p2 = front_edge_to_center - front_edge_to_mirror + mirror_length;
  mirror_extrude_length = 0.5 * (width - width_wo_rearview_mirror);

  obstacle_consider_min_v = msquare::CarParams::GetInstance()
                                ->car_config.lon_config.obstacle_consider_min_v;
  max_v = is_reverse ? msquare::TrajectoryOptimizerConfig::GetInstance()
                           ->param_max_speed_reverse
                     : msquare::TrajectoryOptimizerConfig::GetInstance()
                           ->param_max_speed_forward;
  obstacle_consider_max = msquare::CarParams::GetInstance()
                              ->car_config.lon_config.obstacle_consider_max;
  v_slop = (max_v - obstacle_consider_min_v) /
           (std::max(obstacle_consider_max - obstacle_consider_min, 0.1));

  window_s_size =
      msquare::CarParams::GetInstance()->car_config.lon_config.window_s_size;
  acc = msquare::CarParams::GetInstance()->car_config.lon_config.acc;
  dec = msquare::CarParams::GetInstance()->car_config.lon_config.dec;
  limit_dec =
      msquare::CarParams::GetInstance()->car_config.lon_config.limit_dec;
  dt = 1.0 / msquare::CarParams::GetInstance()->car_config.lon_config.rate;
  forbid_acc_delta_v = limit_dec * dt;
  forbid_acc_max_s =
      msquare::CarParams::GetInstance()->car_config.lon_config.forbid_acc_max_s;

  min_duration_filter_time =
      msquare::CarParams::GetInstance()
          ->car_config.lon_config.min_duration_filter_time;
  curvature_change_threshold =
      msquare::CarParams::GetInstance()
          ->car_config.lon_config.curvature_change_threshold;
  curvature_limit_v = msquare::CarParams::GetInstance()
                          ->car_config.lon_config.curvature_limit_v;

  traj_end_speed_limit_s = msquare::CarParams::GetInstance()
                               ->car_config.lon_config.traj_end_speed_limit_s;
  control_take_over_remain_s =
      msquare::CarParams::GetInstance()
          ->car_config.lon_config.control_take_over_remain_s;
  control_take_over_speed = msquare::CarParams::GetInstance()
                                ->car_config.lon_config.control_take_over_speed;
  control_take_over_acc = msquare::CarParams::GetInstance()
                              ->car_config.lon_config.control_take_over_acc;
  remain_s_min_velocity =
      msquare::CarParams::GetInstance()->car_config.lon_config.min_velocity;
  remain_s_planning_deceleration =
      msquare::CarParams::GetInstance()
          ->car_config.lon_config.planning_deceleration;
  if (is_last_segment) {
    remain_s_planning_deceleration =
        msquare::CarParams::GetInstance()
            ->car_config.lon_config.planning_deceleration_last;
  }
  const_speed_min_s = std::abs(max_v * max_v - control_take_over_speed *
                                                   control_take_over_speed) /
                          (2 * remain_s_planning_deceleration) +
                      control_take_over_remain_s;
  lower_const_speed_min_s = control_take_over_remain_s -
                            (control_take_over_speed * control_take_over_speed -
                             remain_s_min_velocity * remain_s_min_velocity) /
                                (2 * control_take_over_acc);
}

bool SpeedMarginLimiter::getSegmentV(std::vector<double> &vs) {
  for (auto &seg : vec_sv_) {
    vs.push_back(seg.second);
  }
  return true;
}

void SpeedMarginLimiter::getSerializeString(std::string &debug_string) {
  SpeedMarginDebug debug;
  debug.is_reverse = para_.is_reverse;
  debug.is_last_segment = para_.is_last_segment;
  debug.update_wheel_stop = para_.update_wheel_stop;
  debug.is_dynamic_planning = para_.is_dynamic_planning;
  debug.is_apa_not_parallel = para_.is_apa_not_parallel;
  debug.speed_type = para_.last_res.type;
  debug.travel_s = para_.last_res.travel_s;
  debug.last_v = para_.last_v;
  debug.last_delta_s = para_.last_delta_s;
  debug.vec_sv = vec_sv_;
  nlohmann::json debug_result = debug;
  debug_string = debug_result.dump();
}

SpeedMarginDebug
SpeedMarginLimiter::generateFromString(const std::string &info) {
  nlohmann::json input_json = nlohmann::json::parse(info);
  SpeedMarginDebug debug = input_json;
  return debug;
}

bool SpeedMarginLimiter::getVecSVSegment(VecSV &vec_sv, double s) {
  vec_sv.clear();
  if (vec_sv_.size() < 2) {
    vec_sv = vec_sv_;
    return true;
  }
  if (s < vec_sv_.back().first) {
    vec_sv.push_back(vec_sv_.back());
    return true;
  }

  double total_length = vec_sv_[0].first;
  if (s > total_length - 1e-3) {
    vec_sv.emplace_back(s, vec_sv_.front().second);
    vec_sv.insert(vec_sv.end(), vec_sv_.begin(), vec_sv_.end());
    return true;
  }

  unsigned int left = 0;
  unsigned int right = vec_sv_.size() - 1;
  unsigned int mid;
  while (left < right) {
    mid = int(0.5 * (left + right));
    if (vec_sv_[mid].first > s) {
      left = mid + 1;
    } else {
      right = mid;
    }
  }

  if (left == 0) {
    vec_sv.push_back(vec_sv_.back());
    return true;
  }

  // linear interpolation
  double s1 = vec_sv_[left - 1].first;
  double s2 = vec_sv_[left].first;
  double delta_s = std::max(0.0, s1 - s2);
  double v = 0.0;
  if (delta_s < 1e-3) {
    v = vec_sv_[left].second;
  } else {
    double ratio = std::max(0.0, s - s2) / delta_s;
    v = ratio * vec_sv_[left - 1].second + (1 - ratio) * vec_sv_[left].second;
  }
  vec_sv.emplace_back(s, v);
  vec_sv.insert(vec_sv.end(), vec_sv_.begin() + left, vec_sv_.end());
  return true;
}

bool SpeedMarginLimiter::getSTByS(VecST &vec_st, double delta_t, double s) {
  VecSV vec_sv;
  getVecSVSegment(vec_sv, s);
  vec_st.clear();
  if (vec_sv.size() < 2) {
    return false;
  }

  double total_length = vec_sv[0].first;

  if (total_length < para_.traj_end_speed_limit_s) {
    //  return as traj is too short
    return false;
  }

  // precomputed t, v, a at  knot points
  int sv_nums = vec_sv.size();
  double init_v = vec_sv[0].second;
  std::vector<double> ts;
  std::vector<double> ss;
  std::vector<double> vs;
  std::vector<double> as;
  ss.resize(sv_nums, 0.0);
  ts.resize(sv_nums, 0.0);
  vs.resize(sv_nums, 0.0);
  as.resize(sv_nums, 0.0);
  vs[0] = init_v;
  double eps = 1e-3;
  for (int i = 1; i < sv_nums; i++) {
    ss[i] = total_length - vec_sv.at(i).first;
    double ds = vec_sv.at(i - 1).first - vec_sv.at(i).first;
    double dv = vec_sv.at(i).second - vec_sv.at(i - 1).second;
    double avg_v = 0.5 * (vec_sv.at(i - 1).second + vec_sv.at(i).second);

    if (std::abs(avg_v) < eps) {
      // TODO: when v is tiny
      ts[i] = ts[i - 1] + 1.0;
      vs[i] = vec_sv.at(i).second;
      as[i] = 0.0;
      continue;
    }

    double dt = ds / avg_v;
    ts[i] = ts[i - 1] + dt;
    vs[i] = vec_sv.at(i).second;
    as[i] = dv / (std::max(dt, eps));
  }

  double cur_t = 0.0;
  double final_t = ts.back();
  int last_i = 0;
  int cur_i = 1;
  vec_st.emplace_back(cur_t, 0.0, vec_sv[0].second, 0.0);
  VecSV::const_iterator last_iter = vec_sv.begin();
  VecSV::const_iterator cur_iter = last_iter + 1;

  while (cur_t < final_t) {
    cur_t += delta_t;
    if (cur_t > final_t) {
      vec_st.emplace_back(cur_t, ss[cur_i], vs[cur_i], as[cur_i]);
      break;
    }
    // find cur_t in {ts[last_i], ts[cur_i]}
    while (cur_t > ts[cur_i]) {
      cur_i++;
      last_i++;
    }

    double ai = as[cur_i];
    double vi = vs[last_i] + ai * (cur_t - ts[last_i]);
    double si = ss[last_i] + 0.5 * (vs[last_i] + vi) * (cur_t - ts[last_i]);
    vec_st.emplace_back(cur_t, si, vi, ai);
  }

  return true;
}

bool SpeedMarginLimiter::getSVByT(VecST &vec_st, double delta_t) {
  vec_st.clear();
  if (vec_sv_.size() < 2) {
    return false;
  }

  double total_length = vec_sv_[0].first;

  if (total_length < para_.traj_end_speed_limit_s) {
    //  return as traj is too short
    return false;
  }

  // precomputed t, v, a at  knot points
  int sv_nums = vec_sv_.size();
  double init_v = vec_sv_[0].second;
  std::vector<double> ts;
  std::vector<double> ss;
  std::vector<double> vs;
  std::vector<double> as;
  ss.resize(sv_nums, 0.0);
  ts.resize(sv_nums, 0.0);
  vs.resize(sv_nums, 0.0);
  as.resize(sv_nums, 0.0);
  vs[0] = init_v;
  double eps = 1e-3;
  for (int i = 1; i < sv_nums; i++) {
    ss[i] = total_length - vec_sv_.at(i).first;
    double ds = vec_sv_.at(i - 1).first - vec_sv_.at(i).first;
    double dv = vec_sv_.at(i).second - vec_sv_.at(i - 1).second;
    double avg_v = 0.5 * (vec_sv_.at(i - 1).second + vec_sv_.at(i).second);

    if (std::abs(avg_v) < eps) {
      // TODO: when v is tiny
      ts[i] = ts[i - 1] + 1.0;
      vs[i] = vec_sv_.at(i).second;
      as[i] = 0.0;
      continue;
    }

    double dt = ds / avg_v;
    ts[i] = ts[i - 1] + dt;
    vs[i] = vec_sv_.at(i).second;
    as[i] = dv / (std::max(dt, eps));
  }

  double cur_t = 0.0;
  double final_t = ts.back();
  int last_i = 0;
  int cur_i = 1;
  vec_st.emplace_back(cur_t, 0.0, vec_sv_[0].second, 0.0);
  VecSV::const_iterator last_iter = vec_sv_.begin();
  VecSV::const_iterator cur_iter = last_iter + 1;

  while (cur_t < final_t) {
    cur_t += delta_t;
    if (cur_t > final_t) {
      vec_st.emplace_back(cur_t, ss[cur_i], vs[cur_i], as[cur_i]);
      break;
    }
    // find cur_t in {ts[last_i], ts[cur_i]}
    while (cur_t > ts[cur_i]) {
      cur_i++;
      last_i++;
    }

    double ai = as[cur_i];
    double vi = vs[last_i] + ai * (cur_t - ts[last_i]);
    double si = ss[last_i] + 0.5 * (vs[last_i] + vi) * (cur_t - ts[last_i]);
    vec_st.emplace_back(cur_t, si, vi, ai);
  }

  return true;
}

} // namespace parking
} // namespace msquare