
#pragma once

#include "utils/macro.h"
#include "nlohmann/json.hpp"
#include <cassert>
#include <yaml-cpp/yaml.h>

namespace worldmodel_pec {

class VehicleParam {
  DECLARE_SINGLETON(VehicleParam);

public:
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
  // imu
  std::vector<double> r_s2b{0, 0, -M_PI_2};

  std::string car_type = "none";
  std::string architecture = "none";

  bool loadFile(std::string file);

  NLOHMANN_DEFINE_TYPE_INTRUSIVE(
      VehicleParam, car_type, front_edge_to_center, back_edge_to_center,
      left_edge_to_center, right_edge_to_center, center_to_geometry_center,
      bumper_length, light_to_front_edge, front_edge_to_mirror,

      length, width, width_wo_rearview_mirror, height, min_turn_radius,
      max_acceleration, max_deceleration, max_steer_angle, max_steer_angle_rate,
      min_steer_angle_rate, steer_ratio, wheel_base, wheel_rolling_radius,
      max_abs_speed_when_stopped,

      brake_deadzone, throttle_deadzone, velocity_deadzone,

      max_front_wheel_angle, brake_distance_buffer,

      limiter_detect_linear_acceleration_z_delta,
      limiter_detect_linear_acceleration_x_positive,
      limiter_detect_collision_duration_time, r_s2b)
};

struct vehicle_param {
  double front_edge_to_center = 3.89;
  double back_edge_to_center = 1.043;
  double left_edge_to_center = 1.055;
  double right_edge_to_center = 1.055;

  double from_front_bumper_to_front_axle = 0.967;
  double rear_bumper_to_rear_axle = 1.108;

  double length = 4.98;
  double width = 2.11;
  double height = 1.48;
  double min_turn_radius = 5.05386147161;
  double max_acceleration = 2.5;
  double max_deceleration = -6.0;
  double max_steer_angle = 8.20304748437;
  double max_steer_angle_rate = 8.55211;
  double min_steer_angle_rate = 0;
  double steer_ratio = 14.8;
  double wheel_base = 2.8448;
  double front_wheel_rolling_radius = 0.335;
  double rear_wheel_rolling_radius = 0.335;
  double max_abs_speed_when_stopped = 0.2;

  double brake_deadzone = 15.5;
  double throttle_deadzone = 18.0;

  double max_front_wheel_angle = 0.56;

  double angle_offset = 0.0;
  // imu
  std::vector<double> r_s2b{0, 0, -M_PI_2};

  std::string car_type = "none";
  std::string architecture = "none";
};

}