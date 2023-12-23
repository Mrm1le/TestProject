#ifndef COLLISION_SHAPE_H
#define COLLISION_SHAPE_H

#include "common/math/polygon2d.h"
#include "common/utils/geometry.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/clothoid_parameter.h"

namespace clothoid {
using namespace msquare;

enum class ShapeType { RAW = 0, WHEEL_BASE, RECT, ROTATE, OCTAGON};
class CollisionShapeGenerator {
public:
  CollisionShapeGenerator() : para_(Parameter()) { init(); }
  CollisionShapeGenerator(const Parameter &para) : para_(para) { init(); }
  const Parameter getPara() { return para_; }
  void getCollisionShape(const Pose2D &ego_pose,
                         std::vector<planning_math::Vec2d> &polygon_points,
                         ShapeType type, double lat, double lon) const;
  void getRawShape(const Pose2D &ego_pose,
                   std::vector<planning_math::Vec2d> &polygon_points,
                   double lat, double lon) const;
  void getRectShape(const Pose2D &ego_pose,
                    std::vector<planning_math::Vec2d> &polygon_points,
                    double lat, double lon) const;
  void getWheelBaseShape(const Pose2D &ego_pose,
                         std::vector<planning_math::Vec2d> &polygon_points,
                         double lat, double lon) const;
  void getOctagonShape(const Pose2D &ego_pose,
                         std::vector<planning_math::Vec2d> &polygon_points,
                         double lat, double lon) const;
  /**
   * @brief Get the Rotate Shape object
   *  8-edges with chamfer
   *
   * @param ego_pose
   * @param polygon_points
   * @param lat
   * @param lon
   */
  void getRotateShape(const Pose2D &ego_pose,
                      std::vector<planning_math::Vec2d> &polygon_points,
                      double lat, double lon) const;

  void formatLines(const std::vector<planning_math::Vec2d> &pts,
                   std::vector<planning_math::LineSegment2d> &lines);

  double width_;
  double half_width_;
  double length_;
  double front_to_rear_;
  double back_to_rear_;

private:
  void init();
  Parameter para_;
};

} // namespace clothoid

#endif