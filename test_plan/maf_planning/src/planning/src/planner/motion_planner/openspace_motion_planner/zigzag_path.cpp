#include "planner/motion_planner/openspace_motion_planner/zigzag_path.h"
#include "common/math/math_utils.h"
#include <cmath>

namespace msquare {

ZigzagPath::ZigzagPath() : points_() {}

ZigzagPath::ZigzagPath(size_t size) : points_(size) { init(); }

ZigzagPath::ZigzagPath(const std::vector<DirTrajectoryPoint> &points)
    : points_(points) {
  if (!points.empty()) {
    init();
  }
}

ZigzagPath::ZigzagPath(const std::vector<TrajectoryPoint> &points) {
  if (!points.empty()) {
    points_.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      points_.push_back(DirTrajectoryPoint(points[i]));
    }
    init();
  }
}

void ZigzagPath::init() {
  gen_directions();
  gen_stages();
}

void ZigzagPath::reset() {
  points_.clear();
  stages_info_.clear();
}

void ZigzagPath::gen_directions() {
  if (points_.empty()) {
    // throw(std::logic_error("gen_directions No points to be operated."));
  }
  // find the first point with none-zero velocity to determine direction
  auto fisrt_vel_iter = points_.begin();
  for (; fisrt_vel_iter != points_.end(); ++fisrt_vel_iter) {
    if (std::fabs(fisrt_vel_iter->v) > 1e-5)
      break;
  }
  if (fisrt_vel_iter == points_.end()) {
    // throw(std::invalid_argument("All velocity of trajectory is 0!"));
  }
  // apply direction to the first point
  points_.begin()->direction = fisrt_vel_iter->v > 0 ? 1 : -1;

  // gen directions
  for (std::size_t i = 1; i != points_.size(); ++i) {
    if (std::fabs(points_[i].v) > 1e-5) {
      points_[i].direction = points_[i].v > 0 ? 1 : -1;
    } else {
      points_[i].direction = points_[i - 1].direction;

      // toggle direction if moving direction differs to last direction
      double x_diff_position =
          points_[i].path_point.x - points_[i - 1].path_point.x;
      double y_diff_position =
          points_[i].path_point.y - points_[i - 1].path_point.y;

      int direction_factor = points_[i - 1].direction;
      double x_direction_vector =
          cos(points_[i - 1].path_point.theta) * direction_factor;
      double y_direction_vector =
          sin(points_[i - 1].path_point.theta) * direction_factor;

      if (planning_math::InnerProd(x_diff_position, y_diff_position,
                                   x_direction_vector,
                                   y_direction_vector) < 0) {
        points_[i].direction *= -1;
      }
      points_[i].v = points_[i].direction;
    }
  }
}

void ZigzagPath::gen_stages() {
  if (points_.empty()) {
    // throw(std::logic_error("gen_stages No points to be operated."));
  }
  std::vector<std::vector<DirTrajectoryPoint>::iterator> stages_start_iter;
  stages_start_iter.push_back(points_.begin());
  int current_direction = points_.begin()->direction;
  for (auto iter = points_.begin() + 1; iter != points_.end(); ++iter) {
    if (iter->direction != current_direction) {
      current_direction = iter->direction;
      stages_start_iter.push_back(iter);
    }
  }
  stages_start_iter.push_back(points_.end()); // ensure the end

  auto stages_start_iter_last = stages_start_iter.end() - 1;
  for (auto iter = stages_start_iter.begin(); iter != stages_start_iter_last;
       ++iter) {
    stages_info_.emplace_back(*iter, *(iter + 1));
  }
}

size_t ZigzagPath::get_stage_idx(
    std::vector<DirTrajectoryPoint>::const_iterator iter) const {
  if (points_.empty()) {
    // throw(std::logic_error("get_stage_idx No points to be operated."));
  }
  auto stage_iter = stages_info_.begin();
  for (; stage_iter != stages_info_.end(); ++stage_iter) {
    if (stage_iter->is_contains(iter)) {
      return std::distance(stages_info_.begin(), stage_iter);
    }
  }
  // throw(std::invalid_argument("Iterator in none of all stages."));

  return -1;
}

std::vector<DirTrajectoryPoint>::const_iterator ZigzagPath::get_stage_lower(
    std::vector<DirTrajectoryPoint>::const_iterator iter) const {
  if (points_.empty()) {
    // throw(std::logic_error("get_stage_lower No points to be operated."));
  }
  auto stage_iter = stages_info_.begin();
  for (; stage_iter != stages_info_.end(); ++stage_iter) {
    if (stage_iter->is_contains(iter))
      return stage_iter->get_lower();
  }
  // throw(std::invalid_argument("Iterator in none of all stages."));

  return stages_info_.begin()->get_lower();
}

std::vector<DirTrajectoryPoint>::const_iterator ZigzagPath::get_stage_upper(
    std::vector<DirTrajectoryPoint>::const_iterator iter) const {
  if (points_.empty()) {
    // throw(std::logic_error("get_stage_upper No points to be operated."));
  }
  auto stage_iter = stages_info_.begin();
  for (; stage_iter != stages_info_.end(); ++stage_iter) {
    if (stage_iter->is_contains(iter))
      return stage_iter->get_upper();
  }
  // throw(std::invalid_argument("Iterator in none of all stages."));

  return stages_info_.begin()->get_upper();
}

std::vector<DirTrajectoryPoint> 
  ZigzagPath::get_segment_traj(size_t seg_index) const {
  std::vector<DirTrajectoryPoint>  result_traj;
  result_traj.clear();
  if (seg_index >= stages_info_.size()) {
    return result_traj;
  }
  result_traj = std::vector<DirTrajectoryPoint>(
    (stages_info_[seg_index]).get_lower(),
    (stages_info_[seg_index]).get_upper());
  return result_traj;
}

ZigzagPath::StageInfo::StageInfo(
    std::vector<DirTrajectoryPoint>::const_iterator start,
    std::vector<DirTrajectoryPoint>::const_iterator end)
    : start_(start), end_(end) {
  len_ = std::distance(start_, end_);
  if (len_ < 0) {
    // throw(std::invalid_argument("Wrong sequence of args."));
  }
}

bool ZigzagPath::StageInfo::is_contains(
    std::vector<DirTrajectoryPoint>::const_iterator iter) const {
  if (std::abs(std::distance(start_, iter)) +
              std::abs(std::distance(iter, end_)) ==
          len_ &&
      iter != end_) {
    return true;
  } else {
    return false;
  }
}

} // namespace msquare