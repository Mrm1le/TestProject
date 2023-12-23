#include "common/math/circle2d.h"

#include "mph_assert.h"
#include <algorithm>
#include <cmath>
#include <utility>

#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"

namespace msquare {
namespace planning_math {

Circle2d::Circle2d(const Vec2d &center, const double radius)
    : center_(center), radius_(radius), diameter_(radius * 2.0) {
  mph_assert(radius > -kMathEpsilon);
}

Circle2d::Circle2d(const LineSegment2d &diameter)
    : center_(diameter.center()), radius_(diameter.length() / 2.0),
      diameter_(diameter.length()) {
  mph_assert(radius_ > -kMathEpsilon);
}

bool Circle2d::IsPointIn(const Vec2d &point) const {
  return point.DistanceTo(center_) <= radius_ + kMathEpsilon;
}

bool Circle2d::IsPointOnBoundary(const Vec2d &point) const {
  return std::abs(point.DistanceTo(center_) - radius_) <= kMathEpsilon;
}

double Circle2d::DistanceTo(const Vec2d &point) const {
  return std::max(point.DistanceTo(center_) - radius_, 0.0);
}

bool Circle2d::HasOverlap(const LineSegment2d &line_segment) const {
  if (line_segment.length() <= kMathEpsilon) {
    return IsPointIn(line_segment.start());
  }
  return DistanceTo(line_segment) <= kMathEpsilon;
}

double Circle2d::DistanceTo(const LineSegment2d &line_segment) const {
  if (line_segment.length() <= kMathEpsilon) {
    return DistanceTo(line_segment.start());
  }
  return std::max(line_segment.DistanceTo(center_) - radius_, 0.0);
}

double Circle2d::DistanceTo(const Box2d &box) const {
  return std::max(box.DistanceTo(center_) - radius_, 0.0);
}

bool Circle2d::HasOverlap(const Box2d &box) const {
  return DistanceTo(box) <= kMathEpsilon;
}

double Circle2d::DistanceTo(const Polygon2d &polygon) const {
  return std::max(polygon.DistanceTo(center_) - radius_, 0.0);
}

bool Circle2d::HasOverlap(const Polygon2d &polygon) const {
  return DistanceTo(polygon) <= kMathEpsilon;
}

double Circle2d::DistanceTo(const Circle2d &circle) const {
  return std::max(
      circle.center().DistanceTo(center_) - radius_ - circle.radius(), 0.0);
}

bool Circle2d::HasOverlap(const Circle2d &circle) const {
  return DistanceTo(circle) <= kMathEpsilon;
}

void Circle2d::Shift(const Vec2d &shift_vec) { center_ += shift_vec; }

bool operator==(const Circle2d &lhs, const Circle2d &rhs) {
  return std::fabs(lhs.center_x() - rhs.center_x()) < 1e-8 &&
         std::fabs(lhs.center_y() - rhs.center_y()) < 1e-8 &&
         std::fabs(lhs.radius() - rhs.radius()) < 1e-8;
}

} // namespace planning_math
} // namespace msquare
