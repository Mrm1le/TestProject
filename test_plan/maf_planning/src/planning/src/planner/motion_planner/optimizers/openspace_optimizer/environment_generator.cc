#include "planner/motion_planner/optimizers/openspace_optimizer/environment_generator.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/rule_planner_util.h"
#include <algorithm>
#include <iostream>

namespace msquare {
EnvironmentGenerator::EnvironmentGenerator() { deinit(); }

EnvironmentGenerator::~EnvironmentGenerator() {}

bool EnvironmentGenerator::init(
    planning_math::Vec2d &p_left, planning_math::Vec2d &p_right,
    std::vector<planning_math::Vec2d> &points_of_obstacles, Pose2D &init_pose,
    Pose2D &local_frame_pose, bool &is_on_left) {
  if (param_.virtual_slot_ids.empty()) {
    // std::cout << "[EnvironmentGenerator::init]: virtual_slot_ids is empty."
    //           << std::endl;
    return false;
  }

  if (param_.slot_width <= 0) {
    // std::cout << "[EnvironmentGenerator::init]: invalid slot_width: "
    //           << param_.slot_width << std::endl;
    return false;
  }

  for (auto &id : param_.virtual_slot_ids) {
    double x = param_.slot_width * id;
    center_x_of_virtual_top_slots_.push_back(x);
  }

  for (auto &id : param_.virtual_slot_ids) {
    double x = param_.slot_width * id;
    center_x_of_virtual_bottom_slots_.push_back(x);
  }

  for (auto &id : param_.virtual_slot_ids) {
    double x = param_.slot_width * id - param_.slot_width / 2;
    threshold_x_of_virtual_top_slots_.push_back(x);
  }
  int id = param_.virtual_slot_ids.at(param_.virtual_slot_ids.size() - 1);
  threshold_x_of_virtual_top_slots_.push_back(param_.slot_width * id +
                                              param_.slot_width / 2);

  for (auto &id : param_.virtual_slot_ids) {
    double x = param_.slot_width * id - param_.slot_width / 2;
    threshold_x_of_virtual_bottom_slots_.push_back(x);
  }
  threshold_x_of_virtual_bottom_slots_.push_back(param_.slot_width * id +
                                                 param_.slot_width / 2);

  for (auto &id : param_.virtual_slot_ids) {
    std::vector<planning_math::Vec2d> points;
    points_in_virtural_top_slots_[id] = points;
  }

  for (auto &id : param_.virtual_slot_ids) {
    std::vector<planning_math::Vec2d> points;
    points_in_virtural_bottom_slots_[id] = points;
  }

  left_boundary_ = std::min(threshold_x_of_virtual_top_slots_.at(0),
                            threshold_x_of_virtual_bottom_slots_.at(0));
  right_boundary_ =
      std::max(threshold_x_of_virtual_top_slots_.at(
                   threshold_x_of_virtual_top_slots_.size() - 1),
               threshold_x_of_virtual_bottom_slots_.at(
                   threshold_x_of_virtual_bottom_slots_.size() - 1));

  max_slot_y_ = std::max(p_left.y(), p_right.y());
  base_y_ = std::max(init_pose.y, max_slot_y_ + 2.0);

  top_boundary_ = std::max(base_y_ + 8, top_boundary_);
  bottom_boundary_ = 0.0;

  set_obstacle_points(points_of_obstacles);

  local_frame_pose_ = local_frame_pose;
  is_on_left_ = is_on_left;

  inited_ = true;
  // std::cout << "[EnvironmentGenerator::init] success!" << std::endl;
  return true;
}

bool EnvironmentGenerator::get_virtual_slot_index(int &index, double x,
                                                  std::vector<double> vec) {
  if (vec.size() < 2) {
    return false;
  }
  if (x<vec.at(0) | x> vec.at(vec.size() - 1)) {
    return false;
  }
  for (int i = 0; i < vec.size() - 1; ++i) {
    if (x < vec.at(i + 1)) {
      index = i;
      return true;
    }
  }
  return false;
}

void EnvironmentGenerator::set_obstacle_points(
    std::vector<planning_math::Vec2d> &points_of_obstacles) {
  for (auto &p : points_of_obstacles) {
    if (p.x() < left_boundary_ | p.x() > right_boundary_ |
        p.y() < bottom_boundary_ | p.y() > top_boundary_) {
      continue;
    }
    int index = -1;
    if (p.y() > base_y_) {
      bool re = get_virtual_slot_index(index, p.x(),
                                       threshold_x_of_virtual_top_slots_);
      if (false == re) {
        continue;
      }
      int id = param_.virtual_slot_ids.at(index);
      points_in_virtural_top_slots_[id].push_back(p);

    } else {
      bool re = get_virtual_slot_index(index, p.x(),
                                       threshold_x_of_virtual_bottom_slots_);
      if (false == re) {
        continue;
      }
      int id = param_.virtual_slot_ids.at(index);
      points_in_virtural_bottom_slots_[id].push_back(p);
    }
  }
}

bool EnvironmentGenerator::deinit() {
  inited_ = false;

  left_boundary_ = -3.75;
  right_boundary_ = 8.75;
  top_boundary_ = 14;
  bottom_boundary_ = 0;
  base_y_ = 6;
  max_slot_y_ = 0.0;
  is_on_left_ = false;

  center_x_of_virtual_top_slots_.clear();
  center_x_of_virtual_bottom_slots_.clear();

  threshold_x_of_virtual_top_slots_.clear();
  threshold_x_of_virtual_bottom_slots_.clear();

  points_in_virtural_top_slots_.clear();
  points_in_virtural_bottom_slots_.clear();

  env_.reset();
  return true;
}

bool EnvironmentGenerator::get_min_x(
    double &x, std::vector<planning_math::Vec2d> &points) {
  if (points.empty()) {
    return false;
  }
  for (auto &p : points) {
    if (p.x() < x) {
      x = p.x();
    }
  }
  return true;
}

bool EnvironmentGenerator::get_max_x(
    double &x, std::vector<planning_math::Vec2d> &points) {
  if (points.empty()) {
    return false;
  }
  for (auto &p : points) {
    if (p.x() > x) {
      x = p.x();
    }
  }
  return true;
}

bool EnvironmentGenerator::get_min_y(
    double &y, std::vector<planning_math::Vec2d> &points) {
  if (points.empty()) {
    return false;
  }
  for (auto &p : points) {
    if (p.y() < y) {
      y = p.y();
    }
  }
  return true;
}

bool EnvironmentGenerator::get_max_y(
    double &y, std::vector<planning_math::Vec2d> &points) {
  if (points.empty()) {
    return false;
  }
  for (auto &p : points) {
    if (p.y() > y) {
      y = p.y();
    }
  }
  return true;
}

void EnvironmentGenerator::get_left_right(
    double &left, double &right, double &center_x,
    std::vector<planning_math::Vec2d> &points, double top, double bottom) {
  for (auto &p : points) {
    if (p.y() >= top | p.y() <= bottom) {
      continue;
    }
    if (p.x() < center_x & p.x() > left) {
      left = p.x();
    }
    if (p.x() >= center_x & p.x() < right) {
      right = p.x();
    }
  }
}
void EnvironmentGenerator::get_min_y1_y2(
    double &y1, double &y2, double &center_x,
    std::vector<planning_math::Vec2d> &points) {
  for (auto &p : points) {
    if (p.x() < center_x & p.y() < y1) {
      y1 = p.y();
    }
    if (p.x() >= center_x & p.y() < y2) {
      y2 = p.y();
    }
  }
}
void EnvironmentGenerator::get_max_y1_y2(
    double &y1, double &y2, double &center_x,
    std::vector<planning_math::Vec2d> &points) {
  for (auto &p : points) {
    if (p.x() < center_x & p.y() > y1) {
      y1 = p.y();
    }
    if (p.x() >= center_x & p.y() > y2) {
      y2 = p.y();
    }
  }
}

void EnvironmentGenerator::set_top_environment() {
  std::vector<double> vx;
  std::vector<double> vy;
  for (int i = 0; i < param_.virtual_slot_ids.size(); ++i) {
    int slot_id = param_.virtual_slot_ids[i];
    double center_x = param_.slot_width * slot_id;
    double left = center_x - param_.slot_width / 2;
    double right = center_x + param_.slot_width / 2;
    get_left_right(left, right, center_x,
                   points_in_virtural_top_slots_[slot_id]);

    double min_y1 = top_boundary_;
    double min_y2 = top_boundary_;
    get_min_y1_y2(min_y1, min_y2, center_x,
                  points_in_virtural_top_slots_[slot_id]);

    double min_x = left;
    double max_x = right;
    if (i > 0 && points_in_virtural_top_slots_[slot_id].size() > 0) {
      get_min_x(min_x, points_in_virtural_top_slots_[slot_id]);
    } else {
      min_x = center_x - param_.slot_width / 2;
    }
    if (i < param_.virtual_slot_ids.size() - 1 &&
        points_in_virtural_top_slots_[slot_id].size() > 0) {
      get_max_x(max_x, points_in_virtural_top_slots_[slot_id]);
    } else {
      max_x = center_x + param_.slot_width / 2;
    }

    double x0 = min_x;
    double y0 = min_y1;
    double x1 = left;
    double y1 = min_y1;
    double x2 = left;
    double y2 = top_boundary_;
    double x3 = right;
    double y3 = top_boundary_;
    double x4 = right;
    double y4 = min_y2;
    double x5 = max_x;
    double y5 = min_y2;

    vx.push_back(x0);
    vx.push_back(x1);
    vx.push_back(x2);
    vx.push_back(x3);
    vx.push_back(x4);
    vx.push_back(x5);

    vy.push_back(y0);
    vy.push_back(y1);
    vy.push_back(y2);
    vy.push_back(y3);
    vy.push_back(y4);
    vy.push_back(y5);
  }

  // refine
  for (int i = 0; i < param_.virtual_slot_ids.size() - 1; ++i) {
    if (vy.at(6 * i + 5) > vy.at(6 * i + 6)) {
      vx.at(6 * i + 5) = vx.at(6 * i + 6);
    } else {
      vx.at(6 * i + 6) = vx.at(6 * i + 5);
    }
  }

  for (int i = 0; i < vx.size(); ++i) {
    env_.top_points.push_back(planning_math::Vec2d(vx.at(i), vy.at(i)));
  }
}

void EnvironmentGenerator::set_bottom_environment() {
  std::vector<double> vx;
  std::vector<double> vy;
  for (int i = 0; i < param_.virtual_slot_ids.size(); ++i) {
    int slot_id = param_.virtual_slot_ids[i];
    double center_x = param_.slot_width * slot_id;
    double left = center_x - param_.slot_width / 2;
    double right = center_x + param_.slot_width / 2;
    get_left_right(left, right, center_x,
                   points_in_virtural_bottom_slots_[slot_id]);

    double max_y1 = bottom_boundary_;
    double max_y2 = bottom_boundary_;
    get_max_y1_y2(max_y1, max_y2, center_x,
                  points_in_virtural_bottom_slots_[slot_id]);

    double min_x = left;
    double max_x = right;
    if (i > 0 & points_in_virtural_bottom_slots_[slot_id].size() > 0) {
      get_min_x(min_x, points_in_virtural_bottom_slots_[slot_id]);
    } else {
      min_x = center_x - param_.slot_width / 2;
    }
    if (i<param_.virtual_slot_ids.size() - 1 &
          points_in_virtural_bottom_slots_[slot_id].size()> 0) {
      get_max_x(max_x, points_in_virtural_bottom_slots_[slot_id]);
    } else {
      max_x = center_x + param_.slot_width / 2;
    }

    double x0 = min_x;
    double y0 = max_y1;
    double x1 = left;
    double y1 = max_y1;
    double x2 = left;
    double y2 = bottom_boundary_;
    double x3 = right;
    double y3 = bottom_boundary_;
    double x4 = right;
    double y4 = max_y2;
    double x5 = max_x;
    double y5 = max_y2;

    vx.push_back(x0);
    vx.push_back(x1);
    vx.push_back(x2);
    vx.push_back(x3);
    vx.push_back(x4);
    vx.push_back(x5);

    vy.push_back(y0);
    vy.push_back(y1);
    vy.push_back(y2);
    vy.push_back(y3);
    vy.push_back(y4);
    vy.push_back(y5);
  }

  // refine
  for (int i = 0; i < param_.virtual_slot_ids.size() - 1; ++i) {
    if (vy.at(6 * i + 5) < vy.at(6 * i + 6)) {
      vx.at(6 * i + 5) = vx.at(6 * i + 6);
    } else {
      vx.at(6 * i + 6) = vx.at(6 * i + 5);
    }
  }

  for (int i = 0; i < vx.size(); ++i) {
    env_.bottom_points.push_back(planning_math::Vec2d(vx.at(i), vy.at(i)));
  }
}

bool EnvironmentGenerator::set_channel_of_env() {
  if (env_.top_points.size() != 6 * param_.virtual_slot_ids.size()) {
    // std::cout << "wrong number of top points in environment!" << std::endl;
    return false;
  }
  if (env_.bottom_points.size() != 6 * param_.virtual_slot_ids.size()) {
    // std::cout << "wrong number of bottom points in environment!" <<
    // std::endl;
    return false;
  }

  double channel0_bottom_left = 0.0;
  double channel0_bottom_right = 0.0;
  for (int i = 0; i < param_.virtual_slot_ids.size(); ++i) {
    int slot_id = param_.virtual_slot_ids.at(i);
    double center_x = param_.slot_width * slot_id;
    double top_left = threshold_x_of_virtual_top_slots_.at(0);
    double top_right = threshold_x_of_virtual_top_slots_.at(
        threshold_x_of_virtual_top_slots_.size() - 1);
    get_left_right(top_left, top_right, center_x, env_.top_points,
                   top_boundary_ - 1e-9, -1e10);

    double bottom_left = threshold_x_of_virtual_bottom_slots_.at(0);
    double bottom_right = threshold_x_of_virtual_bottom_slots_.at(
        threshold_x_of_virtual_bottom_slots_.size() - 1);
    get_left_right(bottom_left, bottom_right, center_x, env_.bottom_points,
                   1e10, bottom_boundary_ + 1e-9);

    double top = top_boundary_;
    double bottom = bottom_boundary_;
    std::vector<planning_math::Vec2d> tmp_top_points(
        env_.top_points.begin() + 6 * i, env_.top_points.begin() + 6 * i + 6);
    std::vector<planning_math::Vec2d> tmp_bot_points(
        env_.bottom_points.begin() + 6 * i,
        env_.bottom_points.begin() + 6 * i + 6);
    get_min_y(top, tmp_top_points);
    get_max_y(bottom, tmp_bot_points);

    Channel c;
    c.x1 = top_left;
    c.x2 = top_right;
    c.dx12 = top_right - top_left;
    c.x3 = bottom_left;
    c.x4 = bottom_right;
    c.dx34 = bottom_right - bottom_left;
    c.y1 = top;
    c.y2 = bottom;
    c.dy12 = top - bottom;
    env_.channels[slot_id] = c;

    if (slot_id == 0) {
      channel0_bottom_left = bottom_left;
      channel0_bottom_right = bottom_right;
    }
  }

  double top = top_boundary_;
  double bottom = bottom_boundary_;
  get_min_y(top, env_.top_points);
  get_max_y(bottom, env_.bottom_points);

  std::vector<planning_math::Vec2d> left_slot_bottom_points{};
  std::vector<planning_math::Vec2d> right_slot_bottom_points{};
  for (const auto &point : env_.bottom_points) {
    if (point.x() < 0.0 && point.x() > -1.5 * 2.5) { // 1.5 * slot width
      left_slot_bottom_points.push_back(point);
    }
    if (point.x() > 0.0 && point.x() < 1.5 * 2.5) {
      right_slot_bottom_points.push_back(point);
    }
  }

  double left_slot_obs_height = 0.0;
  double right_slot_obs_height = 0.0;
  get_max_y(left_slot_obs_height, left_slot_bottom_points);
  get_max_y(right_slot_obs_height, right_slot_bottom_points);

  double none_left_top_boundary = top_boundary_;
  // int virtual_index = -2;
  // int obstacle_index = -1;
  for (int i = 0; i < param_.virtual_slot_ids.size(); ++i) {
    int slot_id = param_.virtual_slot_ids.at(i);
    // if(slot_id > 2) {
    //   continue;
    // }
    const auto &top_obs = points_in_virtural_top_slots_[slot_id];
    for (int j = 0; j < top_obs.size(); j++) {
      const auto &pt = top_obs[j];
      if (pt.x() < p_left_.x()) {
        continue;
      }
      if (none_left_top_boundary > pt.y()) {
        // virtual_index = i;
        // obstacle_index = j;
        none_left_top_boundary = pt.y();
      }
    }
  }

  // if(is_on_left_) {
  //   const auto& pt =
  //   points_in_virtural_top_slots_[param_.virtual_slot_ids.at(virtual_index)][obstacle_index];
  //   planning_math::Vec2d new_p = MirrorInjection(pt);
  //   const auto global_pt = planning_math::tf2d_inv(local_frame_pose_, new_p);
  //   std::cout << "----> top obs PT:" << global_pt.x() << "  " <<
  //   global_pt.y() << std::endl;
  // }
  // std::vector<planning_math::Vec2d> none_left_top_points{};
  // for (const auto &point : env_.top_points) {
  //   if (point.x() > p_left_.x()) {
  //     planning_math::Vec2d new_p = MirrorInjection(point);
  //     const auto global_pt = planning_math::tf2d_inv(local_frame_pose_,
  //     new_p); std::cout << "----> P_0 " << point.x() << "  " << point.y() <<
  //     std::endl; std::cout << "----> P_1 " << global_pt.x() << "  " <<
  //     global_pt.y() << std::endl; none_left_top_points.push_back(point);
  //   }
  // }
  // get_min_y(none_left_top_boundary, none_left_top_points);

  double none_left_bottom_boundary = bottom_boundary_;
  // virtual_index = -2;
  // obstacle_index = -1;
  for (int i = 0; i < param_.virtual_slot_ids.size(); ++i) {
    int slot_id = param_.virtual_slot_ids.at(i);
    // if(slot_id > 2) {
    //   continue;
    // }
    const auto &bottom_obs = points_in_virtural_bottom_slots_[slot_id];
    for (int j = 0; j < bottom_obs.size(); j++) {
      const auto &pt = bottom_obs[j];
      if (pt.x() < p_left_.x()) {
        continue;
      }
      if (pt.y() > none_left_bottom_boundary) {
        // virtual_index = i;
        // obstacle_index = j;
        none_left_bottom_boundary = pt.y();
      }
    }
  }
  // if(is_on_left_) {
  //   const auto& pt =
  //   points_in_virtural_bottom_slots_[param_.virtual_slot_ids.at(virtual_index)][obstacle_index];
  //   std::cout << "the virtual_index" << virtual_index << " j " <<
  //   obstacle_index << std::endl; planning_math::Vec2d new_p =
  //   MirrorInjection(pt); const auto global_pt =
  //   planning_math::tf2d_inv(local_frame_pose_, new_p); std::cout << "---->
  //   bottom obs PT:" << global_pt.x() << "  " << global_pt.y() << std::endl;
  // }
  // std::vector<planning_math::Vec2d> none_left_bottom_points{};
  // for (const auto &point : env_.bottom_points) {
  //   if (point.x() > p_left_.x()) {
  //     planning_math::Vec2d new_p = MirrorInjection(point);
  //     const auto global_pt = planning_math::tf2d_inv(local_frame_pose_,
  //     new_p);
  //     // std::cout << "----> P" << global_pt.x() << "  " << global_pt.y() <<
  //     std::endl; none_left_bottom_points.push_back(point);
  //   }
  // }
  // get_max_y(none_left_bottom_boundary, none_left_bottom_points);

  // std::cout << "the max slot y:" << max_slot_y_
  //           << " bottom is:" << bottom
  //           << " the no left bottom is:" << none_left_bottom_boundary
  //           << " the no left top is:" << none_left_top_boundary
  //           << std::endl;
  env_.channel_width =
      none_left_top_boundary - std::max(none_left_bottom_boundary, max_slot_y_);
  env_.slot_width = env_.channels[0].dx34;
  env_.left_slot_width = std::max(-channel0_bottom_left, 0.0);
  env_.right_slot_width = std::max(channel0_bottom_right, 0.0);
  env_.left_slot_obs_height = left_slot_obs_height;
  env_.right_slot_obs_height = right_slot_obs_height;
  env_.inited = true;
  return true;
}

void EnvironmentGenerator::get_local_env_points(
    std::vector<planning_math::Vec2d> &points) {
  for (auto &p : env_.top_points) {
    points.push_back(p);
  }

  for (auto it = env_.bottom_points.rbegin(); it != env_.bottom_points.rend();
       ++it) {
    points.push_back(*it);
  }
}

void EnvironmentGenerator::get_global_env_points(
    std::vector<planning_math::Vec2d> &points) {
  for (auto &p : env_.top_points) {
    if (is_on_left_) {
      planning_math::Vec2d new_p = MirrorInjection(p);
      points.push_back(planning_math::tf2d_inv(local_frame_pose_, new_p));
    } else {
      points.push_back(planning_math::tf2d_inv(local_frame_pose_, p));
    }
  }

  for (auto it = env_.bottom_points.rbegin(); it != env_.bottom_points.rend();
       ++it) {
    if (is_on_left_) {
      planning_math::Vec2d new_p = MirrorInjection(*it);
      points.push_back(planning_math::tf2d_inv(local_frame_pose_, new_p));
    } else {
      points.push_back(planning_math::tf2d_inv(local_frame_pose_, *it));
    }
  }
}

Environment EnvironmentGenerator::get_environment() { return env_; }

bool EnvironmentGenerator::process() {
  env_.reset();
  set_top_environment();
  set_bottom_environment();
  set_channel_of_env();
  // std::cout << "[EnvironmentGenerator::process] success!" << std::endl;
  return true;
}

} // namespace msquare