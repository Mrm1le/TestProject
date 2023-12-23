#include "closed_loop/closed_loop_utils.h"
#include "maf_interface.h"
#include <cmath>
#include <maf_interface/maf_endpoint.h>
#include <maf_interface/maf_planning.h>

namespace msquare {
namespace control {
inline void calc_steer_ratio(const double next_curvature,
                             const utils::ClosedLoopVehicleParam &param,
                             maf_endpoint::ChassisReport *chassis) {
  auto wheel_base_distance = param.wheel_base_distance;
  auto steer_angle_ratio = param.steering_angle_ratio;
  auto current_angle =
      std::atan(wheel_base_distance * next_curvature) * steer_angle_ratio;
  chassis->steering_report.steering_report_data.steering_cmd = current_angle;
  chassis->steering_report.steering_report_data.steering_wheel_angle_report =
      current_angle;
  chassis->steering_report.steering_report_data.steering_wheel_torque_report =
      current_angle;
  chassis->steering_report.steering_report_data.steering_report = current_angle;
  chassis->steering_report.available =
      maf_endpoint::SteeringReport::STEERING_REPORT_DATA;
};
} // namespace control
} // namespace msquare