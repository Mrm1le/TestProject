#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include "common/math/polygon2d.h"
#include "common/utils/geometry.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/clothoid_parameter.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/collision_shape.h"

namespace clothoid {

class CollisionChecker {
public:
  CollisionChecker() : csg_(CollisionShapeGenerator()), para_(csg_.getPara()) {}
  CollisionChecker(CollisionShapeGenerator &csg)
      : csg_(csg), para_(csg_.getPara()) {}
  ~CollisionChecker() {}

  /**
   * @brief Get the Next Pose when rotate with theta (theta > 0.0), not consider
   * collision
   *
   * @param start_pose
   * @param steer_direction
   * @param travel_direction
   * @param radius
   * @param theta
   * @return Pose2D
   */
  Pose2D rotateTheta(const Pose2D &start_pose, int steer_direction,
                     int travel_direction, double radius, double theta);

  /**
   * @brief get pose when rotating furthest from start_pose without collision
   *
   * @param start_pose
   * @param rotate_type
   * @param type
   * @param radius
   * @param is_init
   * @param lat
   * @param lon
   * @return Pose2D
   */
  Pose2D rotateMaxPose(const Pose2D &start_pose, RotateType rotate_type,
                       ShapeType type, double radius, bool is_init, double lat,
                       double lon);
  Pose2D rotateMaxPose(const Pose2D &start_pose, int steer_direction,
                       int travel_direction, ShapeType type, double radius,
                       bool is_init, double lat, double lon);

  /**
   * @brief
   *
   * @param arc_center
   * @param line
   * @param point
   * @return std::vector<double>
   */
  std::vector<double> rotateMaxTheta(const planning_math::Vec2d &arc_center,
                                     const planning_math::LineSegment2d &line,
                                     const planning_math::Vec2d &point);
  bool checkSinglePose(const Pose2D &pose, double lat, double lon,
                       ShapeType type);
  /**
   * @brief check terminus pose, start and end
   *
   * @param pose
   * @return true
   * @return false
   */
  bool checkTerminalPose(const Pose2D &pose);
  bool checkSinglePoseWheelBase(const Pose2D &pose, double lat, double lon,
                                ShapeType type);

  bool checkBatchPose(const std::vector<Pose2D> &pose, double lat, double lon,
                      ShapeType type);
  bool checkBatchPoseExceptStart(const std::vector<Pose2D> &pose, double lat,
                                 double lon, ShapeType type);
  bool checkBatchPoseExceptEnd(const std::vector<Pose2D> &pose, double lat,
                               double lon, ShapeType type);

  double moveForward(const Pose2D &pose, double lat, double lon,
                     ShapeType type);
  double moveBackward(const Pose2D &pose, double lat, double lon,
                      ShapeType type);

  void setData(std::vector<msquare::planning_math::LineSegment2d> &obs_lines,
               std::vector<msquare::planning_math::Vec2d> &obs_pts,
               std::vector<msquare::planning_math::LineSegment2d> &step_lines,
               std::vector<msquare::planning_math::Vec2d> &step_pts) {
    obs_lines_ = obs_lines;
    obs_pts_ = obs_pts;
    step_lines_ = step_lines;
    step_pts_ = step_pts;
  }

  std::vector<msquare::planning_math::LineSegment2d> obs_lines_;
  std::vector<msquare::planning_math::Vec2d> obs_pts_;

  std::vector<msquare::planning_math::LineSegment2d> step_lines_;
  std::vector<msquare::planning_math::Vec2d> step_pts_;

private:
  CollisionShapeGenerator csg_;
  Parameter para_;
  msquare::planning_math::Polygon2d ego_polygon_;
};

} // namespace clothoid

#endif