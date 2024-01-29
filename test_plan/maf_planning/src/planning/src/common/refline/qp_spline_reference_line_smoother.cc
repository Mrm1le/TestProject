#include "common/refline/qp_spline_reference_line_smoother.h"
#include "common/math/curve_math.h"
#include "common/math/vec2d.h"
#include "common/utils/util.h"
#include <algorithm>
#include <sys/time.h>
#include <time.h>
#include <utility>

namespace msquare {

QpSplineReferenceLineSmoother::QpSplineReferenceLineSmoother(
    const ReferenceLineSmootherConfig &config)
    : ReferenceLineSmoother(config) {}

void QpSplineReferenceLineSmoother::Clear() { t_knots_.clear(); }

bool QpSplineReferenceLineSmoother::Smooth() {
  Clear();
  const double kEpsilon = 1e-6;
  if (!Sampling()) {
    // std::cout << "Fail to sample reference line smoother points!" <<
    // std::endl;
    return false;
  }

  if (!AddConstraint()) {
    // std::cout << "Add constraint for spline smoother failed" << std::endl;
    return false;
  }

  if (!AddKernel()) {
    // std::cout << "Add kernel for spline smoother failed." << std::endl;
    return false;
  }

  if (!Solve()) {
    // std::cout << "Solve spline smoother problem failed" << std::endl;
    return false;
  }

  vx_.clear();
  vy_.clear();

  return true;
}

void QpSplineReferenceLineSmoother::GetRefPoint(
    std::vector<double> &vx, std::vector<double> &vy,
    std::vector<SmoothPoint> &smooth_points_out) {
  vx = vx_;
  vy = vy_;
  smooth_points_out = smooth_points_out_;
}

void QpSplineReferenceLineSmoother::GetSolveStatus(int &solve_status) {
  solve_status = solve_status_;
}

bool QpSplineReferenceLineSmoother::Sampling() {
  const double length =
      anchor_points_.back().path_point.s - anchor_points_.front().path_point.s;
  uint32_t num_spline =
      std::max(1u, static_cast<uint32_t>(
                       length / config_.qp_spline().max_spline_length() + 0.5));
  for (std::uint32_t i = 0; i <= num_spline; ++i) {
    t_knots_.push_back(i * 1.0);
  }
  // normalize point xy
  ref_x_ = anchor_points_.front().path_point.x;
  ref_y_ = anchor_points_.front().path_point.y;
  return true;
}

bool QpSplineReferenceLineSmoother::AddConstraint() {
  // Add x, y boundary constraint
  static std::vector<double> headings;
  static std::vector<double> longitudinal_bound;
  static std::vector<double> lateral_bound;
  static std::vector<planning_math::Vec2d> xy_points;
  headings.clear();
  longitudinal_bound.clear();
  lateral_bound.clear();
  xy_points.clear();
  for (const auto &point : anchor_points_) {
    const auto &path_point = point.path_point;
    headings.push_back(path_point.theta);
    longitudinal_bound.push_back(point.longitudinal_bound);
    lateral_bound.push_back(point.lateral_bound);
    xy_points.emplace_back(path_point.x - ref_x_, path_point.y - ref_y_);
  }
  const double scale = (anchor_points_.back().path_point.s -
                        anchor_points_.front().path_point.s) /
                       (t_knots_.back() - t_knots_.front());
  static std::vector<double> evaluated_t;
  evaluated_t.clear();
  for (const auto &point : anchor_points_) {
    evaluated_t.emplace_back(point.path_point.s / scale);
  }

  return true;
}

bool QpSplineReferenceLineSmoother::AddKernel() { return true; }

bool QpSplineReferenceLineSmoother::Solve() { return true; }

void QpSplineReferenceLineSmoother::SetAnchorPoints(
    const std::vector<AnchorPoint> &anchor_points) {
  // CHECK_GE(anchor_points.size(), 2);
  anchor_points_ = anchor_points;
}

} // namespace msquare
