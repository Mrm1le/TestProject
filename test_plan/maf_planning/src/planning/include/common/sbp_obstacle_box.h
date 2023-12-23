/*
 * @Author: your name
 * @Date: 2021-10-15 16:56:04
 * @LastEditTime: 2021-11-03 16:21:22
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath:
 * /maf_planning/home/ros/catkin_ws/.back_up/maf_planning_004/src/planning/include/common/sbp_obstacle_box.h
 */
#pragma once
#include "common/math/line_segment2d.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/sbp_obstacle_interface.h"
#include "sbp_obstacle_util.h"

namespace msquare {

class SbpObstacleBox : public SbpObstacleInterface {
private:
  planning_math::Box2d box_;
  std::vector<planning_math::Box2d> boxes_;

public:
  SbpObstacleBox(planning_math::Box2d box);
  SbpObstacleBox(const std::vector<planning_math::Box2d> &boxes);
  ~SbpObstacleBox();
  virtual double getCost(const SearchNodePtr &node,
                         const FootprintModelPtr &footpint_model) {
    return 0;
  }
  virtual bool checkCollision(const SearchNodePtr &node,
                              const FootprintModelPtr &footpint_model) {
    if (boxes_.empty()) {
      return footpint_model->checkOverlap(Pose2D(node->x, node->y, node->theta),
                                          box_);
    } else {
      // throw std::runtime_error("checkOverlap(Pose2D(node->x, node->y, "
      //                          "node->theta), boxes_) not implemented");
      // return footpint_model->checkOverlap(Pose2D(node->x, node->y,
      // node->theta), boxes_);
    }

    return true;
  }
  virtual double getDistance(const planning_math::Vec2d &point) {
    if (boxes_.empty()) {
      return box_.DistanceTo(point);
    } else {
      double min_dist = std::numeric_limits<double>::infinity();
      for (const planning_math::Box2d &box : boxes_) {
        min_dist = std::min(min_dist, box.DistanceTo(point));
      }
      return min_dist;
    }

    return 1e19;
  }

  virtual std::vector<planning_math::Vec2d>
  getNearestPoints(const planning_math::LineSegment2d &ego_centerline) {
    std::vector<planning_math::Vec2d> nearest_pts{};
    if (boxes_.empty()) {
      auto lines = box_.GetAllEdges();
      for (auto line : lines) {
        nearest_pts.push_back(getSingleNearestPoint(line, ego_centerline));
      }
    } else {
      for (auto box : boxes_) {
        auto lines = box.GetAllEdges();
        for (auto line : lines) {
          nearest_pts.push_back(getSingleNearestPoint(line, ego_centerline));
        }
      }
    }
    return nearest_pts;
  }
};

inline SbpObstacleBox::SbpObstacleBox(planning_math::Box2d box) : box_(box) {}

inline SbpObstacleBox::SbpObstacleBox(
    const std::vector<planning_math::Box2d> &boxes)
    : boxes_(boxes) {}

inline SbpObstacleBox::~SbpObstacleBox() {}

} // namespace msquare