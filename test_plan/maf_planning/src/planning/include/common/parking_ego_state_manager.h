#pragma once

#include "common/config/vehicle_param.h"
#include "common/math/box2d.h"
#include "common/math/transform.h"
#include "common/math/vec2d.h"
#include "common/planning_context.h"
#include "pnc/define/geometry.h"

namespace msquare {

namespace parking {

class WorldModel;
using State = std::array<double, 3>;
struct EgoState {
  Pose2D ego_pose;
  planning_math::Box2d ego_box;
  Pose3D ego_enu;
  double ego_acc;
  Point2D ego_carte;
  Point2D ego_frenet;
  double ego_vel;
  double ego_steer_angle;
  bool is_static;

  bool flag_is_replan;
  State mpc_vehicle_state;
  TrajectoryPoint real_init_point;
  TrajectoryPoint planning_init_point;
  FrenetState planning_start_state;
  EgoState()
      : ego_vel(0.0), ego_steer_angle(0.0),
        ego_box(planning_math::Box2d(planning_math::Vec2d(0, 0), 0.0,
                                     VehicleParam::Instance()->length,
                                     VehicleParam::Instance()->width)) {}
};
class EgoStateManager {
public:
  // void
  // get_lat_planning_start_state(const std::shared_ptr<WorldModel>
  // &world_model,
  //                              FrenetState &lateral_planning_start_state)
  //                              const;
  void get_parking_lat_planning_start_state(
      const std::shared_ptr<WorldModel> &world_model,
      FrenetState &lateral_planning_start_state) const;
  void get_cart_planning_start_state(
      const std::shared_ptr<WorldModel> &world_model,
      EgoState &lateral_planning_start_state,
      EgoState &frenet_lateral_planning_start_state) const;
  void
  get_init_state_last_pathpoints(const std::shared_ptr<WorldModel> &world_model,
                                 FrenetState &init_point,
                                 const PathPoint &ego_state) const;

  void get_lat_replan_state(const std::shared_ptr<WorldModel> &world_model,
                            FrenetState &lateral_planning_start_state) const;
  void set_ego_carte(const Point2D &ego_carte) {
    ego_state_.ego_carte = ego_carte;
  }
  void start_from_current_vehicle_state(
      const std::shared_ptr<WorldModel> &world_model);
  // void set_planning_start_state(const std::shared_ptr<WorldModel>
  // &world_model);
  void set_planning_start_state_parking(
      const std::shared_ptr<WorldModel> &world_model);
  const EgoState &get_ego_state() const { return ego_state_; }
  EgoState &get_mutable_ego_state() { return ego_state_; }
  const EgoState &get_ego_state_planning() const { return ego_state_planning_; }
  EgoState &get_mutable_ego_state_planning() { return ego_state_planning_; }
  bool get_flag_is_replan() const { return ego_state_.flag_is_replan; }
  void set_ego_pose(const Pose2D &pose);
  void set_ego_enu(const Pose3D &enu) {
    ego_state_.ego_enu = enu;
    ego_state_.ego_enu.position.x +=
        std::cos(ego_state_.ego_pose.theta) *
        VehicleParam::Instance()->front_edge_to_center;
    ego_state_.ego_enu.position.y +=
        std::sin(ego_state_.ego_pose.theta) *
        VehicleParam::Instance()->front_edge_to_center;
    update_transform();
  }
  void set_ego_steer_angle(double steer_angle) {
    ego_state_.ego_steer_angle = steer_angle;
  }
  void set_ego_acc(float ego_acc) { ego_state_.ego_acc = ego_acc; }
  void set_ego_vel(float vel) { ego_state_.ego_vel = vel; }
  void set_ego_static_status(bool is_static) {
    ego_state_.is_static = is_static;
  }
  void set_ego_state_planning() { ego_state_planning_ = ego_state_; }

  const Transform &get_car2enu() const { return car2enu_; }
  const Transform &get_enu2car() const { return enu2car_; }

  bool get_matched_point_info(Pose2D &match_point, double &curvature) const {
    const auto &traj_pose_array = PlanningContext::Instance()
                                      ->planning_status()
                                      .planning_result.traj_pose_array;
    const auto &traj_curvature_array = PlanningContext::Instance()
                                           ->planning_status()
                                           .planning_result.traj_curvature;

    if (!traj_pose_array.empty() && !traj_curvature_array.empty()) {
      std::size_t position_matched_index =
          query_nearst_point_with_buffer(traj_pose_array, ego_state_.ego_pose.x,
                                         ego_state_.ego_pose.y, 1.0e-6);
      match_point = traj_pose_array[position_matched_index];
      curvature = traj_curvature_array[position_matched_index];
      return true;
    }
    return false;
  }

private:
  void update_transform();
  using Pose2DType = Pose2D;
  std::size_t query_lower_bound_point(const std::vector<float> &time_array,
                                      double rel_time);
  std::size_t
  query_nearst_point_with_buffer(const std::vector<Pose2DType> &pose_array,
                                 double x, double y, double buffer) const;
  std::size_t
  query_nearst_point_with_buffer(const std::vector<PathPoint> &pose_array,
                                 double x, double y, double buffer) const;
  Point2D compute_position_projection(const double x, const double y,
                                      const Pose2D p, double p_s);

  void lateral_start_state_to_frenet_start_state(
      const std::shared_ptr<WorldModel> &world_model,
      const EgoState &lateral_planning_start_state,
      EgoState &frenet_lateral_planning_start_state) const;

  EgoState ego_state_;
  EgoState ego_state_planning_;
  Transform car2enu_;
  Transform enu2car_;
};

} // namespace parking

} // namespace msquare