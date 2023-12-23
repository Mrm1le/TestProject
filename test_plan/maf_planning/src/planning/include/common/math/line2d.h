#ifndef MODULES_PLANNING_MATH_LINE2D_H_
#define MODULES_PLANNING_MATH_LINE2D_H_

#include "common/math/line_segment2d.h"
#include "common/math/vec2d.h"
#include "mph_assert.h"

namespace msquare {
namespace planning_math {

/**
 * @class Line
 * @brief Line in 2-D with direction.
 */
class Line2d {
public:
  /**
   * @brief Empty constructor.
   */
  Line2d();

  /**
   * @brief Constructor with start point and end point.
   * @param start The start point on the line.
   * @param end The end point on the line.
   */
  Line2d(const Vec2d &start, const Vec2d &end);

  /**
   * @brief Constructor with line_segment.
   * @param line_segment The line_segment on the line.
   */
  Line2d(const LineSegment2d &line_segment);

  /**
   * @brief Constructor with a point and a direction.
   * @param start The start point on the line.
   * @param direction The direction of the line (angle in radian).
   */
  Line2d(const Vec2d &start, const double &direction);

  ~Line2d();

  /**
   * @brief Get one point on the line.
   * @return One point on the line.
   */
  const Vec2d &point() const { return point_; }

  /**
   * @brief Get the unit direction of the line.
   * @return The unit direction of the line.
   */
  const Vec2d &unit_direction() const { return unit_direction_; }

  /**
   * @brief Get the heading of the line.
   * @return The heading, which is the angle between unit direction and x-axis.
   */
  double heading() const { return heading_; }

  /**
   * @brief Get the cosine of the heading.
   * @return The cosine of the heading.
   */
  double cos_heading() const { return unit_direction_.x(); }

  /**
   * @brief Get the sine of the heading.
   * @return The sine of the heading.
   */
  double sin_heading() const { return unit_direction_.y(); }

  /**
   * @brief Compute the shortest distance from a point on the line
   *        to a point in 2-D.
   * @param point The point to compute the distance to.
   * @return The shortest distance from points on the line to point.
   */
  double DistanceTo(const Vec2d &point) const;

  /**
   * @brief Compute the shortest distance from a point on the line
   *        to a point in 2-D, and get the nearest point on the line.
   * @param point The point to compute the distance to.
   * @param nearest_pt The nearest point on the line
   *        to the input point.
   * @return The shortest distance from points on the line
   *         to the input point.
   */
  double DistanceTo(const Vec2d &point, Vec2d *const nearest_pt) const;

  /**
   * @brief Check if a point is on the line.
   * @param point The point to check if it is on the line.
   * @return Whether the input point is on the line or not.
   */
  bool IsPointIn(const Vec2d &point) const;

  /**
   * @brief Check if the line has an intersect
   *        with another line in 2-D.
   * @param other_line The line to check if it has an intersect.
   * @return Whether the line has an intersect
   *         with the input other_line.
   */
  bool HasIntersect(const Line2d &other_line) const;

  /**
   * @brief Check if the line has an intersect
   *        with another line in 2-D.
   * @param line_segment The line segment to check if it has an intersect.
   * @return Whether the line has an intersect
   *         with the input line_segment.
   */
  bool HasIntersect(const LineSegment2d &line_segment) const;

  /**
   * @brief Compute the intersect with another line in 2-D if any.
   * @param other_line The line to compute the intersect.
   * @param point the computed intersect between the line and
   *        the input other_line.
   * @return Whether the line has an intersect
   *         with the input other_line.
   */
  bool GetIntersect(const Line2d &other_line, Vec2d *const point) const;

  /**
   * @brief Compute the intersect with another line in 2-D if any.
   * @param line_segment The line segment to compute the intersect.
   * @param point the computed intersect between the line and
   *        the input other_line.
   * @return Whether the line has an intersect
   *         with the input line_segment.
   */
  bool GetIntersect(const LineSegment2d &line_segment,
                    Vec2d *const point) const;

  /**
   * @brief Compute the projection of a line segment onto the line.
   * @param line_segment The line segment to compute the projection onto the
   * line.
   * @return The projection of the line segment onto the unit direction.
   */
  double ProjectOntoUnit(const LineSegment2d &line_segment) const;

  /**
   * @brief Compute perpendicular foot of a point in 2-D on the line.
   * @param point The point to compute the perpendicular foot from.
   * @param foot_point The computed perpendicular foot from the input point to
   *        the line.
   * @return The distance from the input point to the perpendicular foot.
   */
  double GetPerpendicularFoot(const Vec2d &point,
                              Vec2d *const foot_point) const;

  /**
   * @brief Check if the line has an bisector
   *        with another line in 2-D.
   * @param other_line The line to check if it has an bisector.
   * @return Whether the line has an bisector
   *         with the input other_line.
   */
  bool HasBisector(const Line2d &other_line) const;

  /**
   * @brief Compute the bisector with another line in 2-D if any.
   * @param other_line The line to compute the bisector.
   * @param bisector the computed bisector between the line and
   *        the input other_line.
   * @return Whether the line has an bisector
   *         with the input other_line.
   */
  bool GetBisector(const Line2d &other_line, Line2d *const bisector) const;

private:
  Vec2d point_;
  Vec2d unit_direction_;
  double heading_ = 0.0;
};

} // namespace planning_math
} // namespace msquare

#endif /* MODULES_PLANNING_MATH_LINE2D_H_ */