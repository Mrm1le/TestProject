#include "planner/motion_planner/openspace_motion_planner/zigzag_pathsampler.h"
#include "common/config/vehicle_param.h"
#include "common/planning_context.h"
#include "common/math/math_utils.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"

namespace msquare {

using namespace parking;

ZigzagPathSampler::ZigzagPathSampler()
    : DELTA_T_(0.5), TIME_RANGE_(5.5),
      MIN_VALID_V_(VehicleParam::Instance()->velocity_deadzone * 1.5),
      DIST_FIRST_EXTEND_(0.05), stage_idx_(-1) {}

std::vector<DirTrajectoryPoint> &ZigzagPathSampler::sample(
    ZigzagPath &path, std::vector<DirTrajectoryPoint>::const_iterator iter) {
  size_t input_stage_idx = path.get_stage_idx(iter);
  if (input_stage_idx != stage_idx_) {
    stage_idx_ = input_stage_idx;
    update_points(path, iter);
    special_slot_type_ = PlanningContext::Instance()
                              ->parking_behavior_planner_output()
                              .parking_slot_info.special_slot_type;
    if (PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->is_narrow_channel || special_slot_type_ >= 1) {
      extend_points_curv();
    } else {
      extend_points();
    }
  }
  return extended_stage_points_;
}

std::vector<DirTrajectoryPoint> &ZigzagPathSampler::sample_sop(
    ZigzagPath &path, std::vector<DirTrajectoryPoint>::const_iterator iter) {
  size_t input_stage_idx = path.get_stage_idx(iter);
  double extend_length = 0.1;
  if (input_stage_idx != stage_idx_) {
    stage_idx_ = input_stage_idx;
    update_points(path, iter);
    special_slot_type_ = PlanningContext::Instance()
                              ->parking_behavior_planner_output()
                              .parking_slot_info.special_slot_type;
    if (PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->is_narrow_channel || special_slot_type_ == 1) {
      extend_points_curv();
    } else {
      extend_points(input_stage_idx == 0);
    }
  }
  return extended_stage_points_;
}

std::vector<DirTrajectoryPoint> ZigzagPathSampler::get_second_traj(
    ZigzagPath &path, std::vector<DirTrajectoryPoint>::const_iterator iter) {
  // std::vector<DirTrajectoryPoint> traj;
  // traj.clear();
  size_t input_stage_idx = path.get_stage_idx(iter);
  if (input_stage_idx == path.get_num_stages() - 1) {
    // hack 2m trajectory
    std::vector<DirTrajectoryPoint> trajectory;
    if (path.get_num_stages() >= 1) {
      std::vector<DirTrajectoryPoint> cur_trajectory =
          std::vector<DirTrajectoryPoint>(path.get_stage_lower(iter),
                                          path.get_stage_upper(iter));
      if(cur_trajectory.size() <= 1) {
        return trajectory;
      }
      const DirTrajectoryPoint &back_traj_point = cur_trajectory.back();
      double assume_v = std::max(std::abs(back_traj_point.v), MIN_VALID_V_);
      double extend_length = 2.0;
      double time_range = extend_length / assume_v;
      double dist_first_extend = DIST_FIRST_EXTEND_;
      if (time_range < DELTA_T_) {
        dist_first_extend = extend_length;
      }
      int direction_factor = back_traj_point.direction;
      double extend_direction_cos =
          cos(back_traj_point.path_point.theta) * direction_factor;
      double extend_direction_sin =
          sin(back_traj_point.path_point.theta) * direction_factor;
      for (double t = 0; t < time_range; t += DELTA_T_) {
        double s = t * assume_v;
        if (std::fabs(t) < 1e-5) {
          s = dist_first_extend;
        }
        DirTrajectoryPoint new_traj_point(back_traj_point);
        PathPoint &new_path_point = new_traj_point.path_point;
        new_path_point.x = back_traj_point.path_point.x + s * extend_direction_cos;
        new_path_point.y = back_traj_point.path_point.y + s * extend_direction_sin;
        new_path_point.kappa = 0.0;

        new_path_point.s = back_traj_point.path_point.s + s;

        new_traj_point.v = 0;
        new_traj_point.a = 0;
        new_traj_point.relative_time = back_traj_point.relative_time + t;
        trajectory.emplace_back(new_traj_point);
        // extended_stage_points_.insert(extended_stage_points_.end(), new_traj_point);
      }
    }
    return trajectory;
  }
  std::vector<DirTrajectoryPoint> final_path = path.get_segment_traj(input_stage_idx + 1);
  DirTrajectoryPoint &front_traj_point = final_path.front();
  double extend_length = 0.1;
  double assume_v = std::max(std::abs(front_traj_point.v), MIN_VALID_V_);
  double dist_first_extend = DIST_FIRST_EXTEND_;
  double time_range = extend_length / assume_v;
  if (time_range < DELTA_T_) {
    dist_first_extend = extend_length;
  }
  int direction_factor = front_traj_point.direction;
  double extend_direction_cos =
      cos(front_traj_point.path_point.theta) * direction_factor;
  double extend_direction_sin =
      sin(front_traj_point.path_point.theta) * direction_factor;
  for (double t = 0; t < time_range; t += DELTA_T_) {
    double s = t * assume_v;
    if (std::fabs(t) < 1e-5) {
      s = dist_first_extend;
    }
    DirTrajectoryPoint new_traj_point(front_traj_point);
    PathPoint &new_path_point = new_traj_point.path_point;
    new_path_point.x = front_traj_point.path_point.x - s * extend_direction_cos;
    new_path_point.y = front_traj_point.path_point.y - s * extend_direction_sin;
    new_path_point.kappa = 0.0;

    new_path_point.s = front_traj_point.path_point.s - s;

    new_traj_point.v = assume_v * direction_factor;
    new_traj_point.a = 0;
    new_traj_point.relative_time = front_traj_point.relative_time - t;
    final_path.insert(final_path.begin(), new_traj_point);
  }

  return final_path;
}

void ZigzagPathSampler::reset_stage_idx() {
  stage_idx_ = -1;
  return;
}

void ZigzagPathSampler::update_points(
    ZigzagPath &path, std::vector<DirTrajectoryPoint>::const_iterator iter) {
  stage_points_ = std::vector<DirTrajectoryPoint>(path.get_stage_lower(iter),
                                                  path.get_stage_upper(iter));
  extended_stage_points_ = stage_points_;
}

// TODO: Jinwei: following previous area based impl, forward with left steer and
// backward with right steer is positive curvature
double ZigzagPathSampler::get_back_curvature() {
  if (stage_points_.size() < 2) {
    return 0.0;
  }

  int stage_pts_size = stage_points_.size();
  PathPoint pt_0 = stage_points_[stage_pts_size - 1].path_point;
  PathPoint pt_1 = stage_points_[stage_pts_size - 2].path_point;
  double dx = pt_0.x - pt_1.x;
  double dy = pt_0.y - pt_1.y;
  return planning_math::AngleDiff(pt_1.theta, pt_0.theta) /
         std::sqrt(dx * dx + dy * dy);
}

void ZigzagPathSampler::extend_points(bool is_first_segment) {
  DirTrajectoryPoint &front_traj_point = stage_points_.front();
  DirTrajectoryPoint &back_traj_point = stage_points_.back();
  // MSD_LOG(ERROR, "The init trajectory last point is x: %f, y: %f",
  //           back_traj_point.path_point.x, back_traj_point.path_point.x);
  // std::cout << "The init trajectory last point is x: " << back_traj_point.path_point.x
  //           << " y: " << back_traj_point.path_point.y << std::endl;

  // extend front points
  double assume_v = std::max(std::abs(front_traj_point.v), MIN_VALID_V_);
  int direction_factor = front_traj_point.direction;
  double extend_direction_cos =
      cos(front_traj_point.path_point.theta) * direction_factor;
  double extend_direction_sin =
      sin(front_traj_point.path_point.theta) * direction_factor;
  if (!is_first_segment) {
    for (double t = 0; t < 0.05; t += DELTA_T_) {
      double s = t * assume_v;
      if (std::fabs(t) < 1e-5) {
        s = DIST_FIRST_EXTEND_;
      }
      DirTrajectoryPoint new_traj_point(front_traj_point);
      PathPoint &new_path_point = new_traj_point.path_point;
      new_path_point.x = front_traj_point.path_point.x - s * extend_direction_cos;
      new_path_point.y = front_traj_point.path_point.y - s * extend_direction_sin;
      new_path_point.kappa = 0.0;

      new_path_point.s = front_traj_point.path_point.s - s;

      new_traj_point.v = assume_v * direction_factor;
      new_traj_point.a = 0;
      new_traj_point.relative_time = front_traj_point.relative_time - t;
      extended_stage_points_.insert(extended_stage_points_.begin(),
                                    new_traj_point);
    }
  }
  // extend back points
  assume_v = std::max(std::abs(back_traj_point.v), MIN_VALID_V_);
  direction_factor = back_traj_point.direction;
  extend_direction_cos =
      cos(back_traj_point.path_point.theta) * direction_factor;
  extend_direction_sin =
      sin(back_traj_point.path_point.theta) * direction_factor;
  for (double t = 0; t < TIME_RANGE_; t += DELTA_T_) {
    double s = t * assume_v;
    if (std::fabs(t) < 1e-5) {
      s = DIST_FIRST_EXTEND_;
    }
    DirTrajectoryPoint new_traj_point(back_traj_point);
    PathPoint &new_path_point = new_traj_point.path_point;
    new_path_point.x = back_traj_point.path_point.x + s * extend_direction_cos;
    new_path_point.y = back_traj_point.path_point.y + s * extend_direction_sin;
    new_path_point.kappa = 0.0;

    new_path_point.s = back_traj_point.path_point.s + s;

    new_traj_point.v = 0;
    new_traj_point.a = 0;
    new_traj_point.relative_time = back_traj_point.relative_time + t;
    extended_stage_points_.insert(extended_stage_points_.end(), new_traj_point);
  }
}

void ZigzagPathSampler::extend_points() {
  DirTrajectoryPoint &front_traj_point = stage_points_.front();
  DirTrajectoryPoint &back_traj_point = stage_points_.back();

  // extend front points
  double assume_v = std::max(std::abs(front_traj_point.v), MIN_VALID_V_);
  int direction_factor = front_traj_point.direction;
  double extend_direction_cos =
      cos(front_traj_point.path_point.theta) * direction_factor;
  double extend_direction_sin =
      sin(front_traj_point.path_point.theta) * direction_factor;
  for (double t = 0; t < TIME_RANGE_; t += DELTA_T_) {
    double s = t * assume_v;
    if (std::fabs(t) < 1e-5) {
      s = DIST_FIRST_EXTEND_;
    }
    DirTrajectoryPoint new_traj_point(front_traj_point);
    PathPoint &new_path_point = new_traj_point.path_point;
    new_path_point.x = front_traj_point.path_point.x - s * extend_direction_cos;
    new_path_point.y = front_traj_point.path_point.y - s * extend_direction_sin;

    new_path_point.s = front_traj_point.path_point.s - s;

    new_traj_point.v = assume_v * direction_factor;
    new_traj_point.a = 0;
    new_traj_point.relative_time = front_traj_point.relative_time - t;
    extended_stage_points_.insert(extended_stage_points_.begin(),
                                  new_traj_point);
  }
  // extend back points
  assume_v = std::max(std::abs(back_traj_point.v), MIN_VALID_V_);
  direction_factor = back_traj_point.direction;
  extend_direction_cos =
      cos(back_traj_point.path_point.theta) * direction_factor;
  extend_direction_sin =
      sin(back_traj_point.path_point.theta) * direction_factor;
  for (double t = 0; t < TIME_RANGE_; t += DELTA_T_) {
    double s = t * assume_v;
    if (std::fabs(t) < 1e-5) {
      s = DIST_FIRST_EXTEND_;
    }
    DirTrajectoryPoint new_traj_point(back_traj_point);
    PathPoint &new_path_point = new_traj_point.path_point;
    new_path_point.x = back_traj_point.path_point.x + s * extend_direction_cos;
    new_path_point.y = back_traj_point.path_point.y + s * extend_direction_sin;

    new_path_point.s = back_traj_point.path_point.s + s;

    new_traj_point.v = 0;
    new_traj_point.a = 0;
    new_traj_point.relative_time = back_traj_point.relative_time + t;
    extended_stage_points_.insert(extended_stage_points_.end(), new_traj_point);
  }
}

void ZigzagPathSampler::extend_points_curv() {
  DirTrajectoryPoint &front_traj_point = stage_points_.front();
  DirTrajectoryPoint &back_traj_point = stage_points_.back();

  // extend front points
  double assume_v = std::max(std::abs(front_traj_point.v), MIN_VALID_V_);
  int direction_factor = front_traj_point.direction;
  double extend_direction_cos =
      cos(front_traj_point.path_point.theta) * direction_factor;
  double extend_direction_sin =
      sin(front_traj_point.path_point.theta) * direction_factor;
  for (double t = 0; t < TIME_RANGE_; t += DELTA_T_) {
    double s = t * assume_v;
    if (std::fabs(t) < 1e-5) {
      s = DIST_FIRST_EXTEND_;
    }
    DirTrajectoryPoint new_traj_point(front_traj_point);
    PathPoint &new_path_point = new_traj_point.path_point;
    new_path_point.x = front_traj_point.path_point.x - s * extend_direction_cos;
    new_path_point.y = front_traj_point.path_point.y - s * extend_direction_sin;

    new_path_point.s = front_traj_point.path_point.s - s;

    new_traj_point.v = assume_v * direction_factor;
    new_traj_point.a = 0;
    new_traj_point.relative_time = front_traj_point.relative_time - t;
    extended_stage_points_.insert(extended_stage_points_.begin(),
                                  new_traj_point);
  }
  
  // extend back points
  assume_v = std::max(std::abs(back_traj_point.v), MIN_VALID_V_);
  direction_factor = back_traj_point.direction;
  double back_points_curvature = get_back_curvature();
  if (fabs(back_points_curvature) < 0.01) {
    extend_direction_cos =
        cos(back_traj_point.path_point.theta) * direction_factor;
    extend_direction_sin =
        sin(back_traj_point.path_point.theta) * direction_factor;
    for (double t = 0; t < TIME_RANGE_; t += DELTA_T_) {
      double s = t * assume_v;
      if (std::fabs(t) < 1e-5) {
        s = DIST_FIRST_EXTEND_;
      }
      DirTrajectoryPoint new_traj_point(back_traj_point);
      PathPoint &new_path_point = new_traj_point.path_point;
      new_path_point.x =
          back_traj_point.path_point.x + s * extend_direction_cos;
      new_path_point.y =
          back_traj_point.path_point.y + s * extend_direction_sin;

      new_path_point.s = back_traj_point.path_point.s + s;

      new_traj_point.v = 0;
      new_traj_point.a = 0;
      new_traj_point.relative_time = back_traj_point.relative_time + t;
      extended_stage_points_.insert(extended_stage_points_.end(),
                                    new_traj_point);
    }
  } else {
    double radius = 1.0 / fabs(back_points_curvature);
    int clock_side = back_points_curvature > 0 ? 1 : -1;
    double O_x = back_traj_point.path_point.x -
                 direction_factor * clock_side * radius * sin(back_traj_point.path_point.theta);
    double O_y = back_traj_point.path_point.y +
                 direction_factor * clock_side * radius * cos(back_traj_point.path_point.theta);
    for (double t = 0; t < TIME_RANGE_; t += DELTA_T_) {
      double s = t * assume_v;
      if (std::fabs(t) < 1e-5) {
        s = DIST_FIRST_EXTEND_;
      }
      DirTrajectoryPoint new_traj_point(back_traj_point);
      new_traj_point.path_point.theta =
          back_traj_point.path_point.theta +
          clock_side * s / radius;
      new_traj_point.path_point.x =
          O_x + direction_factor * clock_side * radius * sin(new_traj_point.path_point.theta);
      new_traj_point.path_point.y =
          O_y - direction_factor * clock_side * radius * cos(new_traj_point.path_point.theta);
      new_traj_point.path_point.s = back_traj_point.path_point.s + s;

      new_traj_point.v = 0;
      new_traj_point.a = 0;
      new_traj_point.relative_time = back_traj_point.relative_time + t;
      extended_stage_points_.insert(extended_stage_points_.end(),
                                    new_traj_point);
    }
  }
}

} // namespace msquare
