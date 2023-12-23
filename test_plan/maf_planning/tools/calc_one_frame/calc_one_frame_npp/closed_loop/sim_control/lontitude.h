#include "maf_interface.h"
#include <maf_interface/maf_endpoint.h>
namespace msquare {
namespace control {

const double MAX_ACC = 3.0;
const double MIN_ACC = -5.0;

inline void calc_throttle_and_pedal(const double next_acc,
                                    maf_endpoint::ChassisReport *chassis) {
  if (next_acc > 0) {
    chassis->throttle_report.throttle_report_data.throttle_output =
        std::min(1.0, next_acc / MAX_ACC);
    chassis->brake_report.brake_report_data.brake_output = 0;
  } else {
    chassis->throttle_report.throttle_report_data.throttle_output = 0;
    chassis->brake_report.brake_report_data.brake_output =
        std::min(next_acc / MIN_ACC, 1.0);
  }
  chassis->brake_report.available =
      maf_endpoint::BrakeReport::BRAKE_REPORT_DATA;
};
} // namespace control
} // namespace msquare