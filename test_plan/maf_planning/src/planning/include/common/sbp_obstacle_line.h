/*
 * @Author: your name
 * @Date: 2021-10-15 16:56:04
 * @LastEditTime: 2021-10-25 14:43:24
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath:
 * /maf_planning/home/ros/catkin_ws/.back_up/maf_planning_004/src/planning/include/common/sbp_obstacle_line.h
 */
#pragma once
#include "common/math/line_segment2d.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/sbp_obstacle_interface.h"
#include "sbp_obstacle_util.h"

#include <limits>

namespace msquare {

class SbpObstacleLine : public SbpObstacleInterface {
private:
  std::vector<planning_math::LineSegment2d> lines_;

public:
  SbpObstacleLine(const std::vector<planning_math::LineSegment2d> &lines);
  ~SbpObstacleLine();
  virtual double getCost(const SearchNodePtr &node,
                         const FootprintModelPtr &footpint_model) {
    return 0;
  }
  virtual bool checkCollision(const SearchNodePtr &node,
                              const FootprintModelPtr &footpint_model) {
    const SearchNodePtr &previous_node = node->previous;
    if (previous_node &&
        footpint_model->checkTraceOverlap(
            Pose2D(previous_node->x, previous_node->y, previous_node->theta),
            Pose2D(node->x, node->y, node->theta), lines_)) {
      return true;
    }
    return footpint_model->checkOverlap(Pose2D(node->x, node->y, node->theta),
                                        lines_);
  }
  virtual double getDistance(const planning_math::Vec2d &point) {
    double min_dist = std::numeric_limits<double>::infinity();
    for (const planning_math::LineSegment2d &line : lines_) {
      min_dist = std::min(min_dist, line.DistanceTo(point));
    }
    return min_dist;
  }
  virtual std::vector<planning_math::Vec2d>
  getNearestPoints(const planning_math::LineSegment2d &ego_centerline) {
    std::vector<planning_math::Vec2d> nearest_pts{};
    for (auto line : lines_) {
      nearest_pts.push_back(getSingleNearestPoint(line, ego_centerline));
    }
    return nearest_pts;
  }
  virtual std::vector<planning_math::Vec2d>
  getDiscretePoints(double step) const {
    std::vector<planning_math::Vec2d> results;
    for (const auto &line : lines_) {
      double length = line.length();
      int count = std::ceil(length / step) + 1;
      for (int i = 0; i < count; i++) {
        results.push_back(line.getPoint(std::min(i * step, length)));
      }
    }
    return results;
  };
};

inline SbpObstacleLine::SbpObstacleLine(
    const std::vector<planning_math::LineSegment2d> &lines)
    : lines_(lines) {}

inline SbpObstacleLine::~SbpObstacleLine() {}

} // namespace msquare