#include "common/search_based_planning_utils.h"
#include "nlohmann/json.hpp"

namespace msquare {
namespace parking {
using namespace msquare::planning_math;
using namespace maf_planning;

void convert_to_maf(const msquare::SbpResult &sbp_result,
                    maf_planning::Result &msg_result) {
  if (sbp_result.x.empty()) {
    msg_result.available |= maf_planning::Result::FAILURE_REASON;
    return;
  }
  msg_result.available |= maf_planning::Result::ITERATION_TIMES;
  msg_result.available |= maf_planning::Result::TRAJECTORY;
  msg_result.available |= maf_planning::Result::NUM_SEGMENTS;
  msg_result.iteration_times = sbp_result.iteration_times;
  maf_planning::SBPVehicleState state;
  for (size_t i = 0; i < sbp_result.x.size(); ++i) {
    state.pose.x = sbp_result.x.at(i);
    state.pose.y = sbp_result.y.at(i);
    state.pose.theta = sbp_result.phi.at(i);
    if (sbp_result.wheel_base_offset.at(i) < 0) {
      state.steer_mode.value = maf_planning::SteerModeEnum::TWIST;
    } else if (sbp_result.wheel_base_offset.at(i) > 0) {
      state.steer_mode.value = maf_planning::SteerModeEnum::TRANSLATION;
    } else {
      state.steer_mode.value = maf_planning::SteerModeEnum::NORMAL;
    }
    // if (sbp_result.v.at(i) > 0) {
    //   state.gear = 1;
    // } else if (sbp_result.v.at(i) < 0) {
    //   state.gear = -1;
    // } else {
    //   state.gear = 0;
    // }
    // state.steer = sbp_result.steer.at(i);
    msg_result.trajectory.push_back(state);
  }
  msg_result.num_segments = sbp_result.num_segments;
  msg_result.failure_reason.value = (uint8_t)sbp_result.status;
}

void convert_from_maf(const maf_planning::Result &msg_result,
                      SbpResult &sbp_result) {
  // fill in these items:
  // uint8_t available;
  // int64_t iteration_times;
  // std::vector<SBPVehicleState> trajectory;
  // uint8_t num_segments;
  // FailureReasonEnum failure_reason;
  sbp_result.iteration_times = msg_result.iteration_times;
  for (const maf_planning::SBPVehicleState &vs : msg_result.trajectory) {
    sbp_result.x.push_back(vs.pose.x);
    sbp_result.y.push_back(vs.pose.y);
    sbp_result.phi.push_back(vs.pose.theta);
    // sbp_result.v.push_back(vs.gear);
    // sbp_result.steer.push_back(vs.steer);
    sbp_result.wheel_base_offset.push_back((int)vs.steer_mode.value);
  }
  sbp_result.num_segments = msg_result.num_segments;
  sbp_result.status =
      static_cast<SbpStatus>((int)msg_result.failure_reason.value);
}
} // namespace parking
} // namespace msquare