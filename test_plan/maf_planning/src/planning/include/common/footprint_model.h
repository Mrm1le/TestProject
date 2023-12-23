#pragma once

#include "common/math/map_line.h"
#include "common/math/math_utils.h"
#include "pnc/define/geometry.h"
#include <memory>
#include <unordered_set>

namespace msquare {

/**
 * @class FootprintModelBase
 * @brief Abstract class that defines the interface for robot footprint/contour
 * models
 *
 * The robot model class is currently used in optimization only, since
 * taking the navigation stack footprint into account might be
 * inefficient. The footprint is only used for checking feasibility.
 */
class FootprintModelBase {
public:
  /**
   * @brief Default constructor of the abstract obstacle class
   */
  FootprintModelBase() {}

  /**
   * @brief Virtual destructor.
   */
  virtual ~FootprintModelBase() {}

  double max_x() const { return max_x_; }
  double min_x() const { return min_x_; }
  double max_y() const { return max_y_; }
  double min_y() const { return min_y_; }

  virtual void updatePose(const Pose2D &current_pose) = 0;

  /**
   * @brief Calculate the distance between the robot and an obstacle
   * @param current_pose Current robot pose
   * @param obstacle Pointer to the obstacle
   * @return Euclidean distance to the robot
   */
  virtual double calculateDistance(const Pose2D &current_pose,
                                   const ObstacleLine *obstacle) = 0;

  // virtual double calculateDistance(const Pose2D& current_pose, const
  // planning_math::LineSegment2d& line) = 0;

  // virtual double calculateDistance(const Pose2D& current_pose, const
  // planning_math::Box2d& box) = 0;

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const planning_math::LineSegment2d &obstacle,
                            bool is_virtual = false) = 0;

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const planning_math::Box2d &box) = 0;
  virtual bool checkOverlap(const Pose2D &current_pose,
                            const planning_math::Vec2d &obstacle,
                            bool is_virtual = false) = 0;

  virtual bool
  checkOverlap(const Pose2D &current_pose,
               const std::vector<planning_math::LineSegment2d> &obstacles,
               bool is_virtual = false) = 0;
  virtual bool checkOverlap(const Pose2D &current_pose,
                            const std::vector<planning_math::Vec2d> &obstacles,
                            bool is_virtual = false) = 0;
  virtual bool checkTraceOverlap(
      const Pose2D &current_pose, const Pose2D &next_pose,
      const std::vector<planning_math::LineSegment2d> &obstacles) = 0;

protected:
  double max_x_ = std::numeric_limits<double>::lowest();
  double min_x_ = std::numeric_limits<double>::max();
  double max_y_ = std::numeric_limits<double>::lowest();
  double min_y_ = std::numeric_limits<double>::max();
};

//! Abbrev. for shared obstacle pointers
typedef std::shared_ptr<FootprintModelBase> FootprintModelPtr;
//! Abbrev. for shared obstacle const pointers
typedef std::shared_ptr<const FootprintModelBase> FootprintModelConstPtr;

} // namespace msquare
