#include "common/math/line2d.h"

#include "common/math/math_utils.h"

#include <iostream>
namespace msquare {
namespace planning_math {

Line2d::~Line2d() {}

Line2d::Line2d() {
  point_ = Vec2d(0, 0);
  unit_direction_ = Vec2d(1, 0);
}

Line2d::Line2d(const Vec2d &start, const Vec2d &end) : point_(start) {
  const double dx = end.x() - start.x();
  const double dy = end.y() - start.y();
  double length = hypot(dx, dy);
  unit_direction_ =
      (length <= kMathEpsilon ? Vec2d(0, 0) : Vec2d(dx / length, dy / length));
  heading_ = unit_direction_.Angle();
}

Line2d::Line2d(const LineSegment2d &line_segment)
    : point_(line_segment.start()) {
  const double dx = line_segment.end().x() - line_segment.start().x();
  const double dy = line_segment.end().y() - line_segment.start().y();
  double length = line_segment.length();
  unit_direction_ =
      (length <= kMathEpsilon ? Vec2d(0, 0) : Vec2d(dx / length, dy / length));
  heading_ = unit_direction_.Angle();
}

Line2d::Line2d(const Vec2d &start, const double &direction) : point_(start) {
  unit_direction_ = Vec2d(cos(direction), sin(direction));
  heading_ = unit_direction_.Angle();
}

double Line2d::DistanceTo(const Vec2d &point) const {
  return std::abs(unit_direction_.CrossProd(point - point_));
}

double Line2d::DistanceTo(const Vec2d &point, Vec2d *const nearest_pt) const {
  mph_assert(nearest_pt != nullptr);
  const double proj = unit_direction_.InnerProd(point - point_);
  *nearest_pt = point_ + unit_direction_ * proj;
  return std::abs(unit_direction_.CrossProd(point - point_));
}

bool Line2d::IsPointIn(const Vec2d &point) const {
  const double prod = unit_direction_.CrossProd(point - point_);
  return !(std::abs(prod) > kMathEpsilon);
}

double Line2d::ProjectOntoUnit(const LineSegment2d &line_segment) const {
  const double prod = unit_direction_.InnerProd(line_segment.unit_direction());
  return std::abs(prod * line_segment.length());
}

bool Line2d::HasIntersect(const Line2d &other_line) const {
  const double prod = unit_direction_.CrossProd(other_line.unit_direction());
  return (std::abs(prod * prod) >
          kMathEpsilon); // To keep precision, we take square of prod
}

bool Line2d::HasIntersect(const LineSegment2d &line_segment) const {
  Line2d line = Line2d(line_segment);
  Vec2d intersection;
  if (GetIntersect(line, &intersection)) {
    return ((intersection - line_segment.start())
                .InnerProd(intersection - line_segment.end()) < 0);
  }
  return false;
}

bool Line2d::GetIntersect(const Line2d &other_line, Vec2d *const point) const {
  mph_assert(point != nullptr);
  if (!HasIntersect(other_line)) {
    return false;
  }
  const Vec2d vec = other_line.point() - point_;
  const double length = other_line.unit_direction().CrossProd(vec) /
                        other_line.unit_direction().CrossProd(unit_direction_);
  *point = point_ + unit_direction_ * length;
  return true;
}

bool Line2d::GetIntersect(const LineSegment2d &line_segment,
                          Vec2d *const point) const {
  mph_assert(point != nullptr);
  if (!HasIntersect(line_segment)) {
    return false;
  }
  Line2d line = Line2d(line_segment);
  return GetIntersect(line, point);
}

// return distance with perpendicular foot point.
double Line2d::GetPerpendicularFoot(const Vec2d &point,
                                    Vec2d *const foot_point) const {
  mph_assert(foot_point != nullptr);

  const double x0 = point.x() - point_.x();
  const double y0 = point.y() - point_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  *foot_point = point_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

bool Line2d::HasBisector(const Line2d &other_line) const {
  const double inner_prod =
      unit_direction_.InnerProd(other_line.unit_direction());
  return (HasIntersect(other_line) || inner_prod > 0.0);
}

bool Line2d::GetBisector(const Line2d &other_line,
                         Line2d *const bisector) const {
  mph_assert(bisector != nullptr);
  if (!HasBisector(other_line)) {
    return false;
  }

  Vec2d direction = unit_direction_ + other_line.unit_direction();
  Vec2d point;
  if (!GetIntersect(other_line, &point)) {
    point = (point_ + other_line.point()) / 2;
  }
  *bisector = Line2d(point, point + direction);
  return true;
}

} // namespace planning_math
} // namespace msquare
