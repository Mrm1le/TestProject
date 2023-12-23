#ifndef MSQUARE_DECISION_PLANNING_PLANNER_BEHAVIOR_PLANNER_PARKING_LONGITUDINAL_BEHAVIOR_PLANNER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_BEHAVIOR_PLANNER_PARKING_LONGITUDINAL_BEHAVIOR_PLANNER_H_

#include "common/ego_model_manager.h"
#include "common/obstacle_manager.h"
#include "common/parking_world_model.h"
#include "planner/behavior_planner/deciders/collision_checker.h"
#include "planner/behavior_planner/deciders/leader_decider.h"
#include "planner/behavior_planner/deciders/remain_dist_decider.h"
#include "planner/behavior_planner/parking_behavior_planner.h"
#include <algorithm>
#include <iostream>

namespace msquare {

namespace parking {

class ParkingLongitudinalBehaviorPlanner : public BehaviorPlanner {
public:
  explicit ParkingLongitudinalBehaviorPlanner(
      const std::shared_ptr<WorldModel> &world_model);
  virtual ~ParkingLongitudinalBehaviorPlanner();

  virtual bool calculate();

  const CollisionChecker &get_collision_checker() const {
    return collision_checker_;
  }

  CollisionCheckStatus
  check_fs_point_collision(const std::vector<Pose2D> trajectory,
                           const planning_math::Vec2d &fs_point);

  static bool compute_lead_obs_info(
      const ParkingSlotInfo &park_info,
      LongitudinalBehaviorPlannerOutput::RemainDistInfo *ptr_remain_dist_info, 
      int* ptr_is_need_pause);

private:
  bool compute(LeaderPair &lead_cars);
  bool compute(FreespacePoint &lead_point,
               std::vector<std::pair<double, double>>
                   *const ptr_old_mpc_sl_points = nullptr);
  bool compute(FreespaceLine &lead_line);
  bool fill_multidirectional_cars_decision(double v_ego);
  bool compute_multidirectional_cars_frenet(
      std::vector<MultiDirectionalCars> &multidirectional_cars);
  bool compute_multidirectional_cars_ego(
      std::vector<MultiDirectionalCars> &multidirectional_cars);
  bool compute_multidirectional_human(
      std::vector<MultiDirectionalHuman> &multidirectional_human);
  bool compute_intention_status_obstacles(
      std::vector<IntentionStatusObstacles> &intention_status_obstacles);
  bool compute_prediction_obstacles(std::vector<int> &prediction_obstacles);

  double calc_lat_offset(std::vector<PathPoint> path_points_real,
                         const double s);
  CollisionCheckStatus
  check_fs_point_collision(const planning_math::Vec2d &fs_point,
                           double ratio = 0.5);
  bool is_side_pass_obj(const int id);
  CollisionCheckStatus
  check_fs_line_collision(const planning_math::LineSegment2d &fs_line,
                          const GroundLineType fusion_type);
  bool MakeSpecialCase(const double side_safe_threshold,
                       FreespacePoint *const ptr_lead_point);

  void freespace_point_debug(
      const msquare::parking::FreespacePoint &lead_point_grid,
      const msquare::parking::FreespacePoint &lead_point_polygon,
      std::vector<std::vector<std::pair<double, double>>> &vec_sl_points,
      const bool remain_decider_status);

  RemainDistDecider::OutHoleParam* const mutable_out_hole_param() {
    return &out_hole_param_;
  }

private:
  std::vector<const Obstacle *> obstacles_;
  std::vector<const Obstacle *> points_;
  std::vector<const Obstacle *> lines_;
  std::vector<const Obstacle *> pillars_;
  std::vector<const Obstacle *> road_borders_;
  std::vector<const Obstacle *> gates_;
  state_t vehicle_state_;
  BehaviorType behavior_type_;

  std::unordered_map<int, ObstacleType> obstacle_initial_state_map_;
  std::unordered_map<int, ObjectDecisionType> longitudinal_decisions_;
  planning_math::Box2d exclusive_box_;
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;
  CollisionChecker collision_checker_;

  // remain dist decider
  RemainDistDecider::OutHoleParam out_hole_param_;

};

} // namespace parking

} // namespace msquare
#endif
