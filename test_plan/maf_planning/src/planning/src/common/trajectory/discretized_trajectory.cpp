#include "common/trajectory/discretized_trajectory.h"
#include "common/math/linear_interpolation.h"
#include "planning/common/common.h"

namespace msquare {

DiscretizedTrajectory::DiscretizedTrajectory(
    const std::vector<TrajectoryPoint> &trajectory_points)
    : std::vector<TrajectoryPoint>(trajectory_points) {
  mph_assert(!trajectory_points.empty());
}

DiscretizedTrajectory::DiscretizedTrajectory(const PlanningResult &msg) {
  // header_time_ = msg.update_time.toSec();
  const auto &traj_path_array = msg.traj_pose_array;
  const auto &traj_vel_array = msg.traj_vel_array;
  const auto &traj_acc_array = msg.traj_acceleration;
  const auto &curv_rate = msg.curv_rate;
  size_t min_size =
      std::min(traj_path_array.size(),
               std::min(traj_vel_array.size(),
                        std::min(traj_acc_array.size(), curv_rate.size())));
  for (size_t i = 0; i < min_size; ++i) {
    TrajectoryPoint point;
    point.a = traj_acc_array[i];
    point.v = traj_vel_array[i].target_velocity;
    point.relative_time = traj_vel_array[i].relative_time;
    point.path_point.x = traj_path_array[i].position_enu.x;
    point.path_point.y = traj_path_array[i].position_enu.y;
    point.path_point.theta = traj_path_array[i].heading_yaw;
    point.path_point.s = traj_vel_array[i].distance;
    point.path_point.kappa = traj_path_array[i].curvature;
    point.path_point.dkappa = curv_rate[i];
    push_back(point);
  }
}

void DiscretizedTrajectory::CompensateTrajectory(
    std::vector<TrajectoryPoint> &trajectory_points, double max_time) {
  if (trajectory_points.back().relative_time >= max_time) {
    return;
  } else {
    double cur_time = trajectory_points.back().relative_time;
    constexpr double kTimeResolution = 0.2;
    while (cur_time < max_time) {
      cur_time = std::min(max_time, cur_time + kTimeResolution);
      TrajectoryPoint cur_point = trajectory_points.back();
      cur_point.relative_time = cur_time;
      cur_point.path_point.x +=
          cur_point.v * std::cos(cur_point.velocity_direction) *
          (cur_time - trajectory_points.back().relative_time);
      cur_point.path_point.y +=
          cur_point.v * std::sin(cur_point.velocity_direction) *
          (cur_time - trajectory_points.back().relative_time);
      cur_point.path_point.s += std::hypot(
          cur_point.path_point.x - trajectory_points.back().path_point.x,
          cur_point.path_point.y - trajectory_points.back().path_point.y);
      trajectory_points.emplace_back(cur_point);
    }
    return;
  }
}

TrajectoryPoint
DiscretizedTrajectory::Evaluate(const double relative_time) const {
  auto comp = [](const TrajectoryPoint &p, const double relative_time) {
    return p.relative_time < relative_time;
  };

  auto it_lower = std::lower_bound(begin(), end(), relative_time, comp);

  if (it_lower == begin()) {
    return front();
  } else if (it_lower == end()) {
    MSD_LOG(INFO, "When evaluate trajectory, relative_time(%lf) is too large",
            relative_time);
    return back();
  }
  return planning_math::InterpolateUsingLinearApproximation(
      *(it_lower - 1), *it_lower, relative_time);
}

size_t DiscretizedTrajectory::QueryLowerBoundPoint(const double relative_time,
                                                   const double epsilon) const {
  mph_assert(!empty());

  if (relative_time >= back().relative_time) {
    return (int)size() - 1;
  }
  auto func = [&epsilon](const TrajectoryPoint &tp,
                         const double relative_time) {
    return tp.relative_time + epsilon < relative_time;
  };
  auto it_lower = std::lower_bound(begin(), end(), relative_time, func);
  return std::distance(begin(), it_lower);
}

size_t DiscretizedTrajectory::QueryNearestPoint(
    const planning_math::Vec2d &position) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (size_t i = 0; i < size(); ++i) {
    const planning_math::Vec2d curr_point(data()[i].path_point.x,
                                          data()[i].path_point.y);

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

size_t DiscretizedTrajectory::QueryNearestPointWithBuffer(
    const planning_math::Vec2d &position, const double buffer) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (size_t i = 0; i < size(); ++i) {
    const planning_math::Vec2d curr_point(data()[i].path_point.x,
                                          data()[i].path_point.y);

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min + buffer) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

size_t
DiscretizedTrajectory::QueryNearestPointWithBuffer(double s,
                                                   const double buffer) const {
  double dist_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (size_t i = 0; i < size(); ++i) {

    const double dist = std::fabs(s - data()[i].path_point.s);
    if (dist < dist_min + buffer) {
      dist_min = dist;
      index_min = i;
    }
  }
  return index_min;
}

void DiscretizedTrajectory::AppendTrajectoryPoint(
    const TrajectoryPoint &trajectory_point) {
  if (!empty()) {
    mph_assert(trajectory_point.relative_time > back().relative_time);
  }
  push_back(trajectory_point);
}

const TrajectoryPoint &
DiscretizedTrajectory::TrajectoryPointAt(const size_t index) const {
  mph_assert(index < NumOfPoints());
  return data()[index];
}

TrajectoryPoint DiscretizedTrajectory::StartPoint() const {
  mph_assert(!empty());
  return front();
}

double DiscretizedTrajectory::GetTemporalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().relative_time - front().relative_time;
}

double DiscretizedTrajectory::GetSpatialLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().path_point.s - front().path_point.s;
}

} // namespace msquare
