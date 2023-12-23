#pragma once

// fake constant vehicle param
namespace msquare
{

class CarParams {
  CarParams();
public:
  ~CarParams();

  static CarParams *GetInstance();

  static CarParams *instance_;
  double lon_inflation_ = 0.4;
  double lat_inflation_ = 0.22;
  double lat_inflation() const { return lat_inflation_; }
};

class VehicleParam {
  VehicleParam();
  static VehicleParam *instance_;
public:
  ~VehicleParam();
  static VehicleParam *Instance();
  double front_edge_to_center = 3.89;
  double back_edge_to_center = 1.043;
  double left_edge_to_center = 1.055;
  double right_edge_to_center = 1.055;
  double center_to_geometry_center = 1.465;
  double bumper_length = 1.19;       // manually measured, not precise value
  double light_to_front_edge = 0.37; // manually measured, not precise value
  double front_edge_to_mirror = 1.8; // manually measured, not precise value

  double length = 4.98;
  double width = 2.11;
  double width_wo_rearview_mirror = 1.89;
  double height = 1.48;
  double min_turn_radius = 5.05386147161;
  double max_acceleration = 2.5;
  double max_deceleration = -6.0;
  double max_steer_angle = 8.20304748437;
  double max_steer_angle_rate = 8.55211;
  double min_steer_angle_rate = 0;
  double steer_ratio = 16;
  double wheel_base = 2.8448;
  double wheel_rolling_radius = 0.335;
  double max_abs_speed_when_stopped = 0.2;

  double brake_deadzone = 15.5;
  double throttle_deadzone = 18.0;
  double velocity_deadzone;

  double max_front_wheel_angle = 0.56;
  double brake_distance_buffer = 0.4;

  double limiter_detect_linear_acceleration_z_delta = 0;
  double limiter_detect_linear_acceleration_x_positive = 0;
  double limiter_detect_collision_duration_time = 0;
};

} // namespace msquare
