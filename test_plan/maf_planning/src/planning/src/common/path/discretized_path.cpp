#include "common/path/discretized_path.h"

#include "common/math/line_segment2d.h"
#include "common/math/linear_interpolation.h"
#include "mph_assert.h"
#include <algorithm>

namespace msquare {

DiscretizedPath::DiscretizedPath(const std::vector<PathPoint> &path_points)
    : std::vector<PathPoint>(path_points) {}

double DiscretizedPath::Length() const {
  if (empty()) {
    return 0.0;
  }
  return back().s - front().s;
}

PathPoint DiscretizedPath::Evaluate(const double path_s) const {
  mph_assert(!empty());
  auto it_lower = QueryLowerBound(path_s);
  if (it_lower == begin()) {
    return front();
  }
  if (it_lower == end()) {
    return back();
  }
  return planning_math::InterpolateUsingLinearApproximation(*(it_lower - 1),
                                                            *it_lower, path_s);
}

double DiscretizedPath::QueryMatchedS(const PathPoint &path_point) const {
  if (size() == 1) {
    return back().s;
  }
  auto matched_iter = std::min_element(
      begin(), end(), [&](const PathPoint &p1, const PathPoint &p2) {
        return std::hypot(p1.x - path_point.x, p1.y - path_point.y) <
               std::hypot(p2.x - path_point.x, p2.y - path_point.y);
      });
  planning_math::LineSegment2d matched_seg;
  double begin_s, end_s;
  if (matched_iter == end() - 1) {
    matched_seg = planning_math::LineSegment2d(
        planning_math::Vec2d((matched_iter - 1)->x, (matched_iter - 1)->y),
        planning_math::Vec2d(matched_iter->x, matched_iter->y));
    begin_s = (matched_iter - 1)->s;
    end_s = matched_iter->s;
  } else if (matched_iter == begin()) {
    matched_seg = planning_math::LineSegment2d(
        planning_math::Vec2d(matched_iter->x, matched_iter->y),
        planning_math::Vec2d((matched_iter + 1)->x, (matched_iter + 1)->y));
    begin_s = matched_iter->s;
    end_s = (matched_iter + 1)->s;
  } else {
    if (std::hypot((matched_iter - 1)->x - path_point.x,
                   (matched_iter - 1)->y - path_point.y) <
        std::hypot((matched_iter + 1)->x - path_point.x,
                   (matched_iter + 1)->y - path_point.y)) {
      matched_seg = planning_math::LineSegment2d(
          planning_math::Vec2d((matched_iter - 1)->x, (matched_iter - 1)->y),
          planning_math::Vec2d(matched_iter->x, matched_iter->y));
      begin_s = (matched_iter - 1)->s;
      end_s = matched_iter->s;
    } else {
      matched_seg = planning_math::LineSegment2d(
          planning_math::Vec2d(matched_iter->x, matched_iter->y),
          planning_math::Vec2d((matched_iter + 1)->x, (matched_iter + 1)->y));
      begin_s = matched_iter->s;
      end_s = (matched_iter + 1)->s;
    }
  }
  planning_math::Vec2d point(path_point.x, path_point.y);
  return std::min(std::max(0.0, matched_seg.ProjectOntoUnit(point)) + begin_s,
                  end_s);
}

std::vector<PathPoint>::const_iterator
DiscretizedPath::QueryLowerBound(const double path_s) const {
  auto func = [](const PathPoint &tp, const double path_s) {
    return tp.s < path_s;
  };
  return std::lower_bound(begin(), end(), path_s, func);
}

PathPoint DiscretizedPath::EvaluateReverse(const double path_s) const {
  mph_assert(!empty());
  auto it_upper = QueryUpperBound(path_s);
  if (it_upper == begin()) {
    return front();
  }
  if (it_upper == end()) {
    return back();
  }
  return planning_math::InterpolateUsingLinearApproximation(*(it_upper - 1),
                                                            *it_upper, path_s);
}

std::vector<PathPoint>::const_iterator
DiscretizedPath::QueryUpperBound(const double path_s) const {
  auto func = [](const double path_s, const PathPoint &tp) {
    return tp.s < path_s;
  };
  return std::upper_bound(begin(), end(), path_s, func);
}

} // namespace msquare
