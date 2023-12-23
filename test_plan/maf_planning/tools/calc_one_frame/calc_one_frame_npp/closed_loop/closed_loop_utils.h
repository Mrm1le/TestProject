#pragma once

#include "common.h"
#include "maf_interface.h"

namespace msquare {
namespace utils {

struct ClosedLoopVehicleParam {
  double vehicle_length;
  double distance_from_rear_bumper_to_rear_axle;
  double steering_angle_ratio;
  double wheel_base_distance;
  double wheel_radius_front_left;
  double wheel_radius_front_right;
  double wheel_radius_rear_left;
  double wheel_radius_rear_right;
};

} // namespace utils
} // namespace msquare