

#include "slot_release_vehicle_param.h"
#include <unistd.h>

namespace worldmodel_pec {

bool VehicleParam::loadFile(std::string file) {
  YAML::Node yaml_node = YAML::LoadFile(file);

  // Read vehicle params from calib
  length = yaml_node["param"]["vehicle_length"].as<double>();
  width = yaml_node["param"]["vehicle_width_with_rearview_mirror"].as<double>();
  width_wo_rearview_mirror = yaml_node["param"]["vehicle_width"].as<double>();
  height = yaml_node["param"]["vehicle_height"].as<double>();
  back_edge_to_center =
      yaml_node["param"]["distance_from_rear_bumper_to_rear_axle"].as<double>();
  front_edge_to_center = length - back_edge_to_center;
  max_steer_angle =
      yaml_node["param"]["steering_angle_max"].as<double>() * M_PI / 180;
  wheel_base = yaml_node["param"]["wheel_base_distance"].as<double>();
  steer_ratio = yaml_node["param"]["steering_angle_ratio"].as<double>();
  wheel_rolling_radius =
      yaml_node["param"]["wheel_radius_front_left"].as<double>();
  car_type = yaml_node["type"].as<std::string>();
  if (yaml_node["architecture"]) {
    architecture = yaml_node["architecture"].as<std::string>();
  }

  // TODO: Move params below to control params.
  max_acceleration = 2.0;
  max_deceleration = -6.0;
  // max_steer_angle = yaml_node["CAR_PARAMS"]["max_steer_angle"].as<float>();
  max_steer_angle_rate = 8.55211;
  // steer_ratio = yaml_node["CAR_PARAMS"]["steer_ratio"].as<float>();
  // wheel_base = yaml_node["CAR_PARAMS"]["wheel_base"].as<float>();
  // wheel_rolling_radius =
  // yaml_node["CAR_PARAMS"]["wheel_rolling_radius"].as<float>();
  max_abs_speed_when_stopped = 0.01;
  brake_deadzone = 15.5;
  throttle_deadzone = 18.0;
  velocity_deadzone = 0.1;
  brake_distance_buffer = 0.1;

  limiter_detect_linear_acceleration_z_delta = 0.5;
  limiter_detect_linear_acceleration_x_positive = 0.5;
  limiter_detect_collision_duration_time = 3.0;

  // derived params
  max_front_wheel_angle = max_steer_angle / steer_ratio;
  // back_edge_to_center = length - front_edge_to_center;
  left_edge_to_center = right_edge_to_center = width / 2.0;
  min_turn_radius = wheel_base / tan(max_front_wheel_angle);
  center_to_geometry_center = front_edge_to_center - length / 2.0;
  return true;
}

}