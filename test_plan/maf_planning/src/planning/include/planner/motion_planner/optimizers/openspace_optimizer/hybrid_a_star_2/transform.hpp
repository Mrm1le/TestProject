#pragma once

#include <cmath>

#include "common/math/math_utils.h"
#include "common/parking_planner_types.h"
#include "common/utils/geometry.h"

namespace msquare {

namespace hybrid_a_star_2 {

class Transform {
public:
  Transform() {}

  Transform(double x, double y, double theta)
      : x_(x), y_(y), theta_(theta), cos_theta_(std::cos(theta)),
        sin_theta_(std::sin(theta)) {}

  void toSelf(double &x, double &y, double &theta) const {
    double dx = x - x_;
    double dy = y - y_;
    x = dx * cos_theta_ + dy * sin_theta_;
    y = dx * -sin_theta_ + dy * cos_theta_;
    theta = planning_math::AngleDiff(theta_, theta);
  }

  void toSelf(SearchNode &node) const {
    double x = node.x();
    double y = node.y();
    double theta = node.theta();
    toSelf(x, y, theta);
    node.setPose(x, y, theta);
  }

  void toSelf(planning_math::Vec2d &point) const {
    double x = point.x();
    double y = point.y();
    double theta = 0.0;
    toSelf(x, y, theta);
    point.set_point(x, y);
  }

  void toSelf(Pose2D &pose) const { toSelf(pose.x, pose.y, pose.theta); }

  void fromSelf(double &x, double &y, double &theta) const {
    double new_x = x * cos_theta_ + y * -sin_theta_ + x_;
    double new_y = x * sin_theta_ + y * cos_theta_ + y_;
    x = new_x;
    y = new_y;
    theta = planning_math::NormalizeAngle(theta + theta_);
  }

  void fromSelf(SearchNode &node) const {
    double x = node.x();
    double y = node.y();
    double theta = node.theta();
    fromSelf(x, y, theta);
    node.setPose(x, y, theta);
  }

  void fromSelf(parking::SearchDebugNode &node) const {
    fromSelf(node.x, node.y, node.theta);
  }

private:
  double x_;
  double y_;
  double theta_;
  double cos_theta_;
  double sin_theta_;
};

} // namespace hybrid_a_star_2

} // namespace msquare
