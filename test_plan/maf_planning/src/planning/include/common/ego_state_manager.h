#ifndef EGO_STATE_H
#define EGO_STATE_H

#include "common/math/transform.h"
#include "common/planning_context.h"
#include "common/trajectory/trajectory_stitcher.h"

namespace msquare {

class WorldModel;

using State = std::array<double, 3>;
struct CartEgoState {
  Pose2D ego_pose;
  Pose2D ego_pose_raw;
  Pose3D ego_enu;
  double ego_acc;
  Point2D ego_carte;

  double ego_vel;
  double ego_vx_mps;
  double ego_vy_mps;
  double ego_steer_angle;
  double ego_v_cruise;

  bool flag_is_replan;
  uint8_t ego_blinker;
  TrajectoryPoint real_init_point;
  TrajectoryPoint planning_init_point;
  std::vector<TrajectoryPoint> stitch_trajectory;
};

struct EgoState {
  Pose2D ego_pose;
  Pose2D ego_pose_raw;
  Pose3D ego_enu;
  double ego_acc;
  Point2D ego_carte;
  Point2D ego_frenet;
  Point2D ego_frenet_raw;
  double ego_vel;
  double ego_steer_angle;
  double ego_v_cruise;

  bool flag_is_replan;
  uint8_t ego_blinker;
  State mpc_vehicle_state;
  TrajectoryPoint real_init_point;
  TrajectoryPoint planning_init_point;
  std::vector<TrajectoryPoint> stitch_trajectory;
  FrenetState planning_start_state;
};

class CartEgoStateManager {
public:
  void set_ego_carte(const Point2D &ego_carte) {
    cart_ego_state_.ego_carte = ego_carte;
  }
  void compute_stitch_trajectory(bool dbw_status, bool throttle_override);
  const CartEgoState &get_cart_ego_state() const { return cart_ego_state_; }
  CartEgoState &get_mutable_cart_ego_state() { return cart_ego_state_; }
  bool get_flag_is_replan() const { return cart_ego_state_.flag_is_replan; }
  void set_ego_pose_and_vel(
      const maf_vehicle_status::VehicleStatus &vehicle_status) {
    cart_ego_state_.ego_pose.x = vehicle_status.location.location_enu.x;
    cart_ego_state_.ego_pose.y = vehicle_status.location.location_enu.y;
    cart_ego_state_.ego_pose.theta =
        vehicle_status.heading_yaw.heading_yaw_data.value_rad;

    cart_ego_state_.ego_pose_raw = cart_ego_state_.ego_pose;
    cart_ego_state_.ego_pose.x -=
        std::cos(cart_ego_state_.ego_pose.theta) *
        ConfigurationContext::Instance()->get_vehicle_param().length / 2.0;
    cart_ego_state_.ego_pose.y -=
        std::sin(cart_ego_state_.ego_pose.theta) *
        ConfigurationContext::Instance()->get_vehicle_param().length / 2.0;
    cart_ego_state_.ego_vel =
        vehicle_status.velocity.heading_velocity.value_mps;
    cart_ego_state_.ego_vx_mps =
        vehicle_status.velocity.heading_velocity.vx_mps;
    cart_ego_state_.ego_vy_mps =
        vehicle_status.velocity.heading_velocity.vy_mps;
    cart_ego_state_.ego_v_cruise =
        vehicle_status.velocity.cruise_velocity.value_mps;
  }
  void set_ego_enu(const maf_vehicle_status::VehicleStatus &vehicle_status) {
    const auto &location_enu = vehicle_status.location.location_enu;
    cart_ego_state_.ego_enu.position = {location_enu.x, location_enu.y,
                                        location_enu.z};
    cart_ego_state_.ego_enu.orientation = {
        location_enu.orientation.x, location_enu.orientation.y,
        location_enu.orientation.z, location_enu.orientation.w};
    update_transform();
  }
  void
  set_ego_steer_angle(const maf_vehicle_status::VehicleStatus &vehicle_status,
                      const double steer_angle_offset_deg) {
    static constexpr double deg2rad = M_PI / 180.0;
    const double epsilon = 1.0e-6;
    double angle_offset =
        ConfigurationContext::Instance()->get_vehicle_param().angle_offset;
    if (std::abs(steer_angle_offset_deg) > epsilon) {
      angle_offset = steer_angle_offset_deg;
    }
    cart_ego_state_.ego_steer_angle =
        vehicle_status.steering_wheel.steering_wheel_data.steering_wheel_rad -
        angle_offset * deg2rad;
  }
  void set_ego_acc(const maf_vehicle_status::VehicleStatus &vehicle_status) {
    cart_ego_state_.ego_acc =
        vehicle_status.brake_info.brake_info_data.acceleration_on_vehicle_wheel;
  }
  void
  set_ego_blinker(const maf_vehicle_status::VehicleStatus &vehicle_status) {
    cart_ego_state_.ego_blinker =
        vehicle_status.vehicle_light.vehicle_light_data.turn_signal.value;
  }
  void set_ego_blinker(const maf_vehicle_status::VehicleLight &vehicle_light) {
    cart_ego_state_.ego_blinker =
        vehicle_light.vehicle_light_data.turn_signal.value;
  }
  const Transform &get_car2enu() const { return car2enu_; }
  const Transform &get_enu2car() const { return enu2car_; }

public:
  std::size_t query_nearst_point_with_buffer(
      const std::vector<maf_planning::PathPoint> &pose_array, double x,
      double y, double buffer) const;

private:
  void update_transform();
  std::size_t query_lower_bound_point(
      const std::vector<maf_planning::VelocityPoint> &vel_array,
      double rel_time);

  Point2D compute_position_projection(const double x, const double y,
                                      const maf_planning::PathPoint &p,
                                      double p_s);

  CartEgoState cart_ego_state_;
  Transform car2enu_;
  Transform enu2car_;
};

class EgoStateManager : public CartEgoStateManager {
public:
  const EgoState &get_ego_state() const { return ego_state_; }
  EgoState &get_mutable_ego_state() { return ego_state_; }
  // void get_lat_planning_start_state(FrenetState
  // &lateral_planning_start_state) const;
  void get_lat_replan_state(FrenetState &lateral_planning_start_state) const;
  void set_frenet_coord_system(
      std::shared_ptr<FrenetCoordinateSystem> frenet_coord) {
    frenet_coord_ = frenet_coord;
  }

private:
  EgoState ego_state_;
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;
};

} // namespace msquare

#endif
