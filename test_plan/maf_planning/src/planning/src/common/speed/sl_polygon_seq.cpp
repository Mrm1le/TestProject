#include "math.h"
#include "mph_assert.h"
#include <algorithm>
#include <utility>

#include "common/math/linear_interpolation.h"
#include "common/speed/sl_polygon_seq.h"

namespace msquare {

planning_math::IntervalMethodSolution<double> SLPolygonSeq::interval_methods_;

SLPolygonSeq::SLPolygonSeq(std::vector<PolygonWithT> sl_polygon_points)
    : std::vector<PolygonWithT>(std::move(sl_polygon_points)) {
  std::sort(begin(), end(), [](const PolygonWithT &p1, const PolygonWithT &p2) {
    return p1.first < p2.first;
  });
}

void SLPolygonSeq::set_invalid_time_sections(
    const std::vector<std::pair<double, double>> &secs) {
  invalid_time_sections_ = secs;
}

bool SLPolygonSeq::EvaluateByTime(const double t,
                                  PolygonWithT *const polygon_t) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().first < t + 1.0e-6 && t - 1.0e-6 < back().first)) {
    return false;
  }
  if (interval_methods_.is_in_intervals(invalid_time_sections_, t)) {
    return false;
  }

  std::vector<PolygonWithT>::const_iterator it_lower;
  if (is_uniform_time_step_) {
    int lower_index = round((t - front().first) / time_step_);
    lower_index = max(0, min(lower_index, int(size() - 1)));
    it_lower = begin();
    it_lower = it_lower + lower_index;
  } else {
    auto comp = [](const PolygonWithT &sp, const double t) {
      return sp.first < t;
    };
    it_lower = std::lower_bound(begin(), end(), t, comp);
  }
  if (it_lower == end()) {
    *polygon_t = back();
  } else if (it_lower == begin()) {
    *polygon_t = front();
  } else {
    const auto &p0 = *(it_lower - 1);
    const auto &p1 = *it_lower;
    double t0 = p0.first;
    double t1 = p1.first;
    std::pair<double, double> time_range{t0, t1};
    if (!interval_methods_.intersect(invalid_time_sections_, time_range)
             .empty()) {
      if (std::fabs(t - t0) < std::fabs(t - t1)) {
        *polygon_t = p0;
        return true;
      } else {
        *polygon_t = p1;
        return true;
      }
    }

    if (std::fabs(t - t0) < 1.e-2) {
      *polygon_t = p0;
      return true;
    } else if (std::fabs(t - t1) < 1.e-2) {
      *polygon_t = p1;
      return true;
    }

    PolygonWithT res;
    res.first = t;
    const auto &p0_points = p0.second.points();
    const auto &p1_points = p1.second.points();
    // mph_assert(p0_points.size() == p1_points.size());
    if (p0_points.size() != p1_points.size()) {
      *polygon_t = t < (t0 + t1) / 2.0 ? p0 : p1;
      return true;
    }
    std::vector<planning_math::Vec2d> p_points;
    for (size_t i = 0; i < p0_points.size(); ++i) {
      planning_math::Vec2d vec_point;
      vec_point.set_x(
          planning_math::lerp(p0_points[i].x(), t0, p1_points[i].x(), t1, t));
      vec_point.set_y(
          planning_math::lerp(p0_points[i].y(), t0, p1_points[i].y(), t1, t));
      p_points.push_back(vec_point);
    }
    planning_math::Polygon2d convex_polygon;
    if (planning_math::Polygon2d::ComputeConvexHull(p_points,
                                                    &convex_polygon)) {
      res.second = convex_polygon;
      *polygon_t = res;
    } else {
      *polygon_t = t < (t0 + t1) / 2.0 ? p0 : p1;
    }
  }
  return true;
}

void SLPolygonSeq::SetTimeStep(double time_step) {
  if (time_step > 0 && time_step < (back().first - front().first)) {
    is_uniform_time_step_ = true;
    time_step_ = time_step;
  }
}

double SLPolygonSeq::TotalTime() const {
  if (empty()) {
    return 0.0;
  }
  return back().first - front().first;
}

} // namespace msquare
