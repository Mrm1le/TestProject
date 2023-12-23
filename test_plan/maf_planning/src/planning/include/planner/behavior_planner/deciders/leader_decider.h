#ifndef MSQUARE_DECISION_PLANNING_PLANNER_LEADER_DECIDER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_LEADER_DECIDER_H_

#include "common/parking_obstacle_manager.h"
#include "common/parking_world_model.h"
#include "common/math/linear_interpolation.h"
// #include "common/ego_model_manager.h"
#include "common/parking_slot_interface.h"
#include "common/math/linear_interpolation.h"
#include "planner/behavior_planner/deciders/collision_checker.h"

namespace msquare {

namespace parking {

class LeaderDecider {
public:
  LeaderDecider(const std::shared_ptr<WorldModel> &world_model);

  bool execute();
  bool check_traj(std::vector<Pose2D> &trajectory);
  std::vector<int> get_static_obstacle_beside_poi();

private:
  double calc_lat_offset(std::vector<PathPoint> path_points_real,
                         const double s);
  
  bool clip_traj();

  bool clip_traj_sop();

  bool check_bounding_box_collision(const planning_math::Box2d &obstacle_box);
  
  CollisionCheckStatus
  calc_bounding_box_collision_dist(const planning_math::Box2d &obstacle_box,
                                   const Obstacle *obstacle = nullptr);
  
  CollisionCheckStatus calc_polygon_collision_dist(
      const planning_math::Box2d &obstacle_box,
      const planning_math::Polygon2d &fillet_cutting_polygon);
  
  planning_math::Polygon2d
  generate_fillet_cutting_polygon(const planning_math::Box2d &obstacle_box,
                                  double fillet_cutting_length);
  void construct_exclusve_box();

  void calculatePathS(std::vector<Pose2D> *const points);

  void calculatePathKappa(std::vector<Pose2D> *const points);

  Pose2D GetInterpolateByLinearApproximation(const Pose2D &base,
                                             const Pose2D &p,
                                             const double s_tmp) const;

  std::vector<Pose2D>
  InterpolatePathPoints(const std::vector<Pose2D> &raw_points,
                        const double step);

private:
  std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;
  std::vector<const Obstacle *> obstacles_;
  std::shared_ptr<ParkingSlotInterface> parking_lot_;
  CollisionChecker collision_checker_;
  // EgoModelManager ego_model_;
};

} // namespace parking

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_PLANNER_LEADER_DECIDER_H_
