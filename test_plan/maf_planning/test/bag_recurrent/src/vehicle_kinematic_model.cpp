#include "vehicle_kinematic_model.h"
#include "common/math/math_utils.h"
#include <cmath>

namespace msquare {

VehicleKinematicModel::VehicleKinematicModel(double wheel_base,
                                             double wheel_base_offset)
    : wheel_base_(wheel_base), wheel_base_offset_(wheel_base_offset) {}

VehicleKinematicModel::~VehicleKinematicModel() {}

VehicleState VehicleKinematicModel::getNextState(VehicleState current_state,
                                                 double steer,
                                                 double travel_dist) {
  if (std::abs(steer) < 1e-6) {
    steer = steer > 0 ? 1e-6 : -1e-6;
  }
  double x = current_state.x;
  double y = current_state.y;
  double theta = current_state.heading;
  double radius = (wheel_base_ + wheel_base_offset_) / std::tan(steer);
  double beta = travel_dist / radius;
  double next_theta = planning_math::NormalizeAngle(theta + beta);
  double next_x = x + radius * (std::cos(theta) * std::sin(beta) -
                                std::sin(theta) * (1 - std::cos(beta)));
  double next_y = y + radius * (std::sin(theta) * std::sin(beta) +
                                std::cos(theta) * (1 - std::cos(beta)));
  // transform the state w.radius.t virtual wheel base back
  next_x += wheel_base_offset_ * std::cos(next_theta);
  next_y += wheel_base_offset_ * std::sin(next_theta);
  int gear = 0;
  if (travel_dist >= 0) {
    gear = 1;
  } else {
    gear = -1;
  }
  return VehicleState{next_x, next_y, next_theta, steer, gear};
}

} // namespace msquare
