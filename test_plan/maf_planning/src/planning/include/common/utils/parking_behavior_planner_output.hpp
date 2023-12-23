#pragma once

#include "common/math/vec2d.h"


namespace msquare {
namespace parking {
double calculateMinDistOnLine(const planning_math::Vec2d &line_pt1,
                              const planning_math::Vec2d &line_pt2,
                              const planning_math::Vec2d &other_point,
                              const double op_theta) {
  // set cross point to other_point dist as L
  // cross point x = xo + L*cos(op_theta), y = yo + L*sin(op_theta)
  // besides cross point is on line
  // so, x = k * x1 + (1-k) * x2, y = k * y1 + (1-k) * y2

  if (fabs(cos(op_theta) * (line_pt2.y() - line_pt1.y()) -
           sin(op_theta) * (line_pt2.x() - line_pt1.x())) < 1e-3) {
    return std::numeric_limits<double>::max();
  }

  double numerator =
      (line_pt1.x() - other_point.x()) * (line_pt1.y() - line_pt2.y()) -
      (line_pt1.y() - other_point.y()) * (line_pt1.x() - line_pt2.x());
  double denominator = cos(op_theta) * (line_pt1.y() - line_pt2.y()) -
                       sin(op_theta) * (line_pt1.x() - line_pt2.x());

  return fabs(numerator / denominator);
}
} //namespace parking
} //namespace msquare