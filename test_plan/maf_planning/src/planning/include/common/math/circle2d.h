

/**
 * @file
 * @brief The class of Circle2d.
 */

#ifndef MODULES_PLANNING_MATH_CIRCLE2D_H_
#define MODULES_PLANNING_MATH_CIRCLE2D_H_

#include <limits>
#include <string>
#include <vector>

#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"

namespace msquare {
namespace planning_math {

/**
 * @class Circle2d
 * @brief Circle in 2-D.
 */

class Circle2d {
public:
  Circle2d() = default;
  /**
   * @brief Constructor which takes the center and radius.
   * @param center The center of the circle.
   * @param radius The length between the center and the perimeter
   */
  Circle2d(const Vec2d &center, const double radius);

  /**
   * @brief Constructor which takes the diameter.
   * @param diameter The line segment from the one side of the circle to another
   * through its center.
   */
  Circle2d(const LineSegment2d &diameter);

  /**
   * @brief Getter of the center of the circle
   * @return The center of the circle
   */
  const Vec2d &center() const { return center_; }

  /**
   * @brief Getter of the x-coordinate of the center of the circle
   * @return The x-coordinate of the center of the circle
   */
  double center_x() const { return center_.x(); }

  /**
   * @brief Getter of the y-coordinate of the center of the circle
   * @return The y-coordinate of the center of the circle
   */
  double center_y() const { return center_.y(); }

  /**
   * @brief Getter of the radius
   * @return The length of the radius
   */
  double radius() const { return radius_; }

  /**
   * @brief Getter of the diameter
   * @return The length of the diameter
   */
  double diameter() const { return diameter_; }

  /**
   * @brief Getter of the area of the circle
   * @return The product of M_PI and the square of its radius
   */
  double area() const { return M_PI * radius_ * radius_; }

  /**
   * @brief Tests points for membership in the circle
   * @param point A point that we wish to test for membership in the circle
   * @return True iff the point is contained in the circle
   */
  bool IsPointIn(const Vec2d &point) const;

  /**
   * @brief Tests points for membership in the boundary of the circle
   * @param point A point that we wish to test for membership in the boundary
   * @return True iff the point is a boundary point of the circle
   */
  bool IsPointOnBoundary(const Vec2d &point) const;

  /**
   * @brief Determines the distance between the circle and a given point
   * @param point The point whose distance to the circle we wish to compute
   * @return A distance
   */
  double DistanceTo(const Vec2d &point) const;

  /**
   * @brief Determines the distance between the circle and a given line segment
   * @param line_segment The line segment whose distance to the circle we
   * compute
   * @return A distance
   */
  double DistanceTo(const LineSegment2d &line_segment) const;

  /**
   * @brief Determines the distance between the circle and a given box
   * @param box The box whose distance to this circle we want to compute
   * @return A distance
   */
  double DistanceTo(const Box2d &box) const;

  /**
   * @brief Determines the distance between the circle and a given polygon
   * @param polygon The polygon whose distance to this circle we want to compute
   * @return A distance
   */
  double DistanceTo(const Polygon2d &polygon) const;

  /**
   * @brief Determines the distance between two circles
   * @param circle The circle whose distance to this circle we want to compute
   * @return A distance
   */
  double DistanceTo(const Circle2d &circle) const;

  /**
   * @brief Determines whether this circle overlaps a given line segment
   * @param line_segment The line-segment
   * @return True if they overlap
   */
  bool HasOverlap(const LineSegment2d &line_segment) const;

  /**
   * @brief Determines whether this circle overlaps a given box
   * @param box The box
   * @return True if they overlap
   */
  bool HasOverlap(const Box2d &box) const;

  /**
   * @brief Determines whether this circle overlaps a given polygon
   * @param polygon The polygon
   * @return True if they overlap
   */
  bool HasOverlap(const Polygon2d &polygon) const;

  /**
   * @brief Determines whether these two circles overlap
   * @param circle The circle
   * @return True if they overlap
   */
  bool HasOverlap(const Circle2d &circle) const;

  /**
   * @brief Shifts this circle by a given vector
   * @param shift_vec The vector determining the shift
   */
  void Shift(const Vec2d &shift_vec);

  /**
   * @brief Gets a human-readable description of the circle
   * @return A debug-string
   */
  std::string DebugString() const;

private:
  Vec2d center_;
  double radius_ = 0.0;
  double diameter_ = 0.0;
};

} // namespace planning_math
} // namespace msquare

#endif /* MODULES_PLANNING_MATH_CIRCLE2D_H_ */
