#include "common/math/line_segment2d.h"

#include "mph_assert.h"
#include <algorithm>
#include <cmath>
#include <utility>

#include "common/math/math_utils.h"

namespace msquare {
namespace planning_math {
namespace {

bool IsWithin(double val, double bound1, double bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }
  return val >= bound1 - kMathEpsilon && val <= bound2 + kMathEpsilon;
}

} // namespace

LineSegment2d::~LineSegment2d() {}

LineSegment2d::LineSegment2d() {
  unit_direction_ = Vec2d(1, 0);
  initMaxMin();
}

LineSegment2d::LineSegment2d(const Vec2d &start, const Vec2d &end)
    : start_(start), end_(end) {
  const double dx = end_.x() - start_.x();
  const double dy = end_.y() - start_.y();
  length_ = hypot(dx, dy);
  unit_direction_ =
      (length_ <= kMathEpsilon ? Vec2d(0, 0)
                               : Vec2d(dx / length_, dy / length_));
  initMaxMin();
}

double LineSegment2d::heading() const {
  if (heading_generated_) {
    return heading_;
  }

  heading_ = unit_direction_.Angle();
  heading_generated_ = true;
  return heading_;
}

double LineSegment2d::length() const { return length_; }

double LineSegment2d::length_sqr() const { return length_ * length_; }

double LineSegment2d::DistanceTo(const Vec2d &point) const {
  if (length_ <= kMathEpsilon) {
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return hypot(x0, y0);
  }
  if (proj >= length_) {
    return point.DistanceTo(end_);
  }
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceTo(const Vec2d &point,
                                 Vec2d *const nearest_pt) const {
  mph_assert(nearest_pt != nullptr);
  if (length_ <= kMathEpsilon) {
    *nearest_pt = start_;
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj < 0.0) {
    *nearest_pt = start_;
    return hypot(x0, y0);
  }
  if (proj > length_) {
    *nearest_pt = end_;
    return point.DistanceTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceSquareTo(const Vec2d &point) const {
  if (length_ <= kMathEpsilon) {
    return point.DistanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return Square(x0) + Square(y0);
  }
  if (proj >= length_) {
    return point.DistanceSquareTo(end_);
  }
  return Square(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceSquareTo(const Vec2d &point,
                                       Vec2d *const nearest_pt) const {
  mph_assert(nearest_pt != nullptr);
  if (length_ <= kMathEpsilon) {
    *nearest_pt = start_;
    return point.DistanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    *nearest_pt = start_;
    return Square(x0) + Square(y0);
  }
  if (proj >= length_) {
    *nearest_pt = end_;
    return point.DistanceSquareTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return Square(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

bool LineSegment2d::IsPointIn(const Vec2d &point) const {
  if (length_ <= kMathEpsilon) {
    return std::abs(point.x() - start_.x()) <= kMathEpsilon &&
           std::abs(point.y() - start_.y()) <= kMathEpsilon;
  }
  const double prod = CrossProd(point, start_, end_);
  if (std::abs(prod) > kMathEpsilon) {
    return false;
  }
  return IsWithin(point.x(), start_.x(), end_.x()) &&
         IsWithin(point.y(), start_.y(), end_.y());
}

double LineSegment2d::ProjectOntoUnit(const Vec2d &point) const {
  return unit_direction_.InnerProd(point - start_);
}

double LineSegment2d::ProductOntoUnit(const Vec2d &point) const {
  return unit_direction_.CrossProd(point - start_);
}

bool LineSegment2d::HasIntersect(const LineSegment2d &other_segment) const {
  if (std::max(other_segment.start().x(), other_segment.end().x()) < min_x_) {
    return false;
  }
  if (std::max(other_segment.start().y(), other_segment.end().y()) < min_y_) {
    return false;
  }
  if (std::min(other_segment.start().x(), other_segment.end().x()) > max_x_) {
    return false;
  }
  if (std::min(other_segment.start().y(), other_segment.end().y()) > max_y_) {
    return false;
  }
  Vec2d point;
  return GetIntersect(other_segment, &point);
}

bool LineSegment2d::GetIntersect(const LineSegment2d &other_segment,
                                 Vec2d *const point) const {
  mph_assert(point != nullptr);
  if (IsPointIn(other_segment.start())) {
    *point = other_segment.start();
    return true;
  }
  if (IsPointIn(other_segment.end())) {
    *point = other_segment.end();
    return true;
  }
  if (other_segment.IsPointIn(start_)) {
    *point = start_;
    return true;
  }
  if (other_segment.IsPointIn(end_)) {
    *point = end_;
    return true;
  }
  if (length_ <= kMathEpsilon || other_segment.length() <= kMathEpsilon) {
    return false;
  }
  const double cc1 = CrossProd(start_, end_, other_segment.start());
  const double cc2 = CrossProd(start_, end_, other_segment.end());
  if (cc1 * cc2 >= -kMathEpsilon) {
    return false;
  }
  const double cc3 =
      CrossProd(other_segment.start(), other_segment.end(), start_);
  const double cc4 =
      CrossProd(other_segment.start(), other_segment.end(), end_);
  if (cc3 * cc4 >= -kMathEpsilon) {
    return false;
  }
  const double ratio = cc4 / (cc4 - cc3);
  *point = Vec2d(start_.x() * ratio + end_.x() * (1.0 - ratio),
                 start_.y() * ratio + end_.y() * (1.0 - ratio));
  return true;
}

// return distance with perpendicular foot point.
double LineSegment2d::GetPerpendicularFoot(const Vec2d &point,
                                           Vec2d *const foot_point) const {
  mph_assert(foot_point != nullptr);
  if (length_ <= kMathEpsilon) {
    *foot_point = start_;
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  *foot_point = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

void LineSegment2d::initMaxMin() {
  min_x_ = std::min(start_.x(), end_.x());
  max_x_ = std::max(start_.x(), end_.x());
  min_y_ = std::min(start_.y(), end_.y());
  max_y_ = std::max(start_.y(), end_.y());
}

Vec2d LineSegment2d::getPoint(double s) const {
  double ratio = s / length_;
  return start_ * (1 - ratio) + end_ * ratio;
}

} // namespace planning_math
} // namespace msquare
