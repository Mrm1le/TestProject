#pragma once
#include "common/ego_model_manager.h"
#include "common/footprint_model.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"

namespace msquare {

class BoxFootprintModel : public FootprintModelBase {
  struct GeometryCache {
    Pose2D pose;
    planning_math::Box2d box;
    planning_math::Box2d box_shrinked;
  };

private:
  const VehicleParam *vehicle_param_;
  // planning_math::Box2d box_real_;
  // planning_math::Box2d box_;
  // planning_math::Box2d box_shrinked_;
  double lat_inflation_;
  double shrink_ratio_for_lines_;
  double inflation_for_points_;
  GeometryCache geometry_cache_;
  planning_math::Vec2d center_;

public:
  BoxFootprintModel(const VehicleParam *vehicle_param, const double = 0,
                    const double = 0);
  ~BoxFootprintModel();

  virtual double calculateDistance(const Pose2D &current_pose,
                                   const ObstacleLine *obstacle);

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const planning_math::LineSegment2d &obstacle,
                            bool is_virtual = false);

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const planning_math::Vec2d &obstacle,
                            bool is_virtual = false);

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const planning_math::Box2d &box);

  virtual bool
  checkOverlap(const Pose2D &current_pose,
               const std::vector<planning_math::LineSegment2d> &obstacles,
               bool is_virtual = false);

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const std::vector<planning_math::Vec2d> &obstacles,
                            bool is_virtual = false);

  virtual bool
  checkTraceOverlap(const Pose2D &current_pose, const Pose2D &next_pose,
                    const std::vector<planning_math::LineSegment2d> &obstacles);

  virtual void updatePose(const Pose2D &current_pose);

private:
  void initMaxMmin();
  planning_math::Vec2d getCenter(const Pose2D &current_pose) const {
    double center_to_geometry_center =
        vehicle_param_->center_to_geometry_center;
    double xx =
        current_pose.x + center_to_geometry_center * cos(current_pose.theta);
    double yy =
        current_pose.y + center_to_geometry_center * sin(current_pose.theta);
    return planning_math::Vec2d(xx, yy);
  }
  // private:
  //   planning_math::Box2d getBoxReal(const Pose2D& current_pose) const;
  //   planning_math::Box2d getBox(const Pose2D& current_pose) const;
  //   planning_math::Box2d getBoxShrinked(const Pose2D& current_pose) const;
};

class PolygonFootprintModel : public FootprintModelBase {
  struct GeometryCache {
    Pose2D pose;
    planning_math::Polygon2d polygon;
    planning_math::Box2d box_shrinked;
  };

private:
  const VehicleParam *vehicle_param_;
  double lat_inflation_;
  double shrink_ratio_for_lines_;
  double inflation_for_points_;
  EgoModelType ego_model_type_;
  EgoModelManager ego_model_manager_;
  GeometryCache geometry_cache_;

public:
  PolygonFootprintModel(const VehicleParam *vehicle_param, EgoModelType type,
                        const double lat_inflation = 0,
                        const double shrink_ratio_for_lines = 0);
  ~PolygonFootprintModel();

  /**
   * @brief Calculate the distance between the robot and an obstacle
   * @param current_pose Current robot pose
   * @param obstacle Pointer to the obstacle
   * @return Euclidean distance to the robot
   */
  virtual double calculateDistance(const Pose2D &current_pose,
                                   const ObstacleLine *obstacle);

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const planning_math::LineSegment2d &obstacle,
                            bool is_virtual = false);

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const planning_math::Vec2d &obstacle,
                            bool is_virtual = false);

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const planning_math::Box2d &box);

  virtual bool
  checkOverlap(const Pose2D &current_pose,
               const std::vector<planning_math::LineSegment2d> &obstacles,
               bool is_virtual = false);

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const std::vector<planning_math::Vec2d> &obstacles,
                            bool is_virtual = false);

  virtual bool
  checkTraceOverlap(const Pose2D &current_pose, const Pose2D &next_pose,
                    const std::vector<planning_math::LineSegment2d> &obstacles);
  virtual void updatePose(const Pose2D &current_pose);

private:
  void initMaxMmin();
  planning_math::Vec2d getCenter(const Pose2D &current_pose) const {
    double center_to_geometry_center =
        vehicle_param_->center_to_geometry_center;
    double xx =
        current_pose.x + center_to_geometry_center * cos(current_pose.theta);
    double yy =
        current_pose.y + center_to_geometry_center * sin(current_pose.theta);
    return planning_math::Vec2d(xx, yy);
  }
};

class CircleFootprintModel : public FootprintModelBase {
private:
  const VehicleParam *vehicle_param_;
  double inflation_;
  double shrink_ratio_;
  double inflation_for_points_;
  planning_math::Vec2d center_;
  Pose2D cache_pose_;

public:
  CircleFootprintModel(const VehicleParam *vehicle_param, double inflation = 0,
                       double shrink_ratio = 0);
  ~CircleFootprintModel();

  virtual double calculateDistance(const Pose2D &current_pose,
                                   const ObstacleLine *obstacle);

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const planning_math::LineSegment2d &obstacle,
                            bool is_virtual = false);

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const planning_math::Vec2d &obstacle,
                            bool is_virtual = false);
  virtual bool checkOverlap(const Pose2D &current_pose,
                            const planning_math::Box2d &box);

  virtual bool
  checkOverlap(const Pose2D &current_pose,
               const std::vector<planning_math::LineSegment2d> &obstacles,
               bool is_virtual = false);

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const std::vector<planning_math::Vec2d> &obstacles,
                            bool is_virtual = false);

  virtual bool
  checkTraceOverlap(const Pose2D &current_pose, const Pose2D &next_pose,
                    const std::vector<planning_math::LineSegment2d> &obstacles);
  virtual void updatePose(const Pose2D &current_pose);

private:
  void initMaxMmin(const Pose2D &current_pose);
  planning_math::Vec2d getCenter(const Pose2D &current_pose) const {
    double center_to_geometry_center = vehicle_param_->front_edge_to_center -
                                       vehicle_param_->front_edge_to_mirror;
    double xx =
        current_pose.x + center_to_geometry_center * cos(current_pose.theta);
    double yy =
        current_pose.y + center_to_geometry_center * sin(current_pose.theta);
    return planning_math::Vec2d(xx, yy);
  }
};

class CompositeFootprintModel : public FootprintModelBase {
private:
  std::vector<FootprintModelPtr> models_;
  void initMaxMmin();

public:
  // TODO@huangzhengming: modify implementation of polygon footpring model and
  // replance with FootprintModelConstPtr
  CompositeFootprintModel(const std::vector<FootprintModelPtr> &models);
  ~CompositeFootprintModel();

  virtual void updatePose(const Pose2D &current_pose);

  virtual double calculateDistance(const Pose2D &current_pose,
                                   const ObstacleLine *obstacle);

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const planning_math::LineSegment2d &obstacle,
                            bool is_virtual = false);

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const planning_math::Vec2d &obstacle,
                            bool is_virtual = false);

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const planning_math::Box2d &box);

  virtual bool
  checkOverlap(const Pose2D &current_pose,
               const std::vector<planning_math::LineSegment2d> &obstacles,
               bool is_virtual = false);

  virtual bool checkOverlap(const Pose2D &current_pose,
                            const std::vector<planning_math::Vec2d> &obstacles,
                            bool is_virtual = false);

  virtual bool
  checkTraceOverlap(const Pose2D &current_pose, const Pose2D &next_pose,
                    const std::vector<planning_math::LineSegment2d> &obstacles);
};

} // namespace msquare
