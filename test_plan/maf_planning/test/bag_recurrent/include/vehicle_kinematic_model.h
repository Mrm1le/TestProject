#pragma once

namespace msquare {

struct VehicleState {
  double x;
  double y;
  double heading;
  double steer;
  int gear;
};

class VehicleKinematicModel {
private:
  double wheel_base_;
  double wheel_base_offset_;

public:
  VehicleKinematicModel(double wheel_base, double wheel_base_offset);
  ~VehicleKinematicModel();
  VehicleState getNextState(VehicleState current_state, double steer,
                            double travel_dist);
};

} // namespace msquare
