#include "common/config/vehicle_param.h"
#include "common/math/box2d.h"
#include "common/math/vec2d.h"
// #include "planning/common/common.h"
#include <unistd.h>

namespace msquare {

namespace {
constexpr double kOctagonContourPointNumber = 8;
constexpr double kHexadecagonContourPointNumber = 16;
} // namespace

bool VehicleParam::loadFile(std::string file) {
  YAML::Node yaml_node = YAML::LoadFile(file);
  if (yaml_node["spare_tire"]) {
    spare_tire_.loadFromYaml(yaml_node["spare_tire"]);
  }
  if (yaml_node["Octagon_contour"]) {
    geometry_contour_.loadFromYaml(yaml_node);
  }

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

  // remedy  spare_tire
  if (spare_tire_.exist_spare_tire) {
    length += spare_tire_.spare_tire_protrusion_length;
    back_edge_to_center += spare_tire_.spare_tire_protrusion_length;
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
  // min_turn_radius = wheel_base / tan(max_front_wheel_angle);
  min_turn_radius = 5.9;
  center_to_geometry_center = front_edge_to_center - length / 2.0;

  return true;
}

planning_math::Box2d VehicleParam::getBox(double x, double y,
                                          double yaw) const {
  // generate ego box
  planning_math::Vec2d vec_to_center(
      (front_edge_to_center - back_edge_to_center) / 2.0,
      (left_edge_to_center - right_edge_to_center) / 2.0);

  planning_math::Vec2d position(x, y);
  planning_math::Vec2d center(position + vec_to_center.rotate(yaw));

  return planning_math::Box2d(center, yaw, length, width);
}

bool VehicleParam::SpareTire::loadFromYaml(const YAML::Node &spare_tire_node) {
  if (spare_tire_node["exist_spare_tire"]) {
    exist_spare_tire = spare_tire_node["exist_spare_tire"].as<bool>(false);
  }
  if (spare_tire_node["spare_tire_radius"]) {
    spare_tire_radius = spare_tire_node["spare_tire_radius"].as<double>();
  }
  if (spare_tire_node["spare_tire_thickness"]) {
    spare_tire_thickness = spare_tire_node["spare_tire_thickness"].as<double>();
  }
  if (spare_tire_node["spare_tire_protrusion_length"]) {
    spare_tire_protrusion_length =
        spare_tire_node["spare_tire_protrusion_length"].as<double>();
  }

  YAML::Node spare_tire_level_arm_node;
  if (spare_tire_node["spare_tire_level_arm"]) {
    spare_tire_level_arm_node = spare_tire_node["spare_tire_level_arm"];
  }
  if (spare_tire_level_arm_node.IsSequence() &&
      spare_tire_level_arm_node.size() == 3) {
    spare_tire_level_arm[0] = spare_tire_level_arm_node[0].as<double>();
    spare_tire_level_arm[1] = spare_tire_level_arm_node[1].as<double>();
    spare_tire_level_arm[2] = spare_tire_level_arm_node[2].as<double>();
  }

  YAML::Node spare_tire_r_s2b_node;
  if (spare_tire_node["spare_tire_r_s2b"]) {
    spare_tire_r_s2b_node = spare_tire_node["spare_tire_r_s2b"];
  }
  if (spare_tire_r_s2b_node.IsSequence() && spare_tire_r_s2b_node.size() == 3) {
    spare_tire_r_s2b[0] = spare_tire_r_s2b_node[0].as<double>();
    spare_tire_r_s2b[1] = spare_tire_r_s2b_node[1].as<double>();
    spare_tire_r_s2b[2] = spare_tire_r_s2b_node[2].as<double>();
  }

  return true;
}

bool VehicleParam::GeometryContour::loadFromYaml(const YAML::Node &yaml_node) {
  auto constructContour =
      [](const YAML::Node &node,
         std::vector<planning_math::Vec2d> *const ptr_contour_point) {
        ptr_contour_point->clear();
        double point_size = node.size();
        ptr_contour_point->reserve(point_size);

        auto iter = node.begin();
        while (iter != node.end()) {
          ptr_contour_point->emplace_back((*iter)[0].as<double>() * 0.001,
                                          (*iter)[1].as<double>() * 0.001);
          iter++;
        }
      };

  is_using_geometry_contour_ = false;
  if (!(yaml_node["Octagon_contour"] && yaml_node["Hexadecagon_contour"])) {
    return false;
  }
  YAML::Node contour_node;
  contour_node = yaml_node["Octagon_contour"];
  if (contour_node.size() == 8) {
    constructContour(contour_node, &octagon_contour_point_);
  } else {
    return false;
  }
  contour_node = yaml_node["Hexadecagon_contour"];
  if (contour_node.size() == 16) {
    constructContour(contour_node, &hexadecagon_contour_point_);
  } else {
    return false;
  }
  is_using_geometry_contour_ = true;
  return true;
}

} // namespace msquare
