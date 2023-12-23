
#include "planning/common/inner_snapshot.h"
#include "mph_assert.h"
#include "planning/common/logging.h"

namespace msd_planning {

PlanInnerSnapshot *PlanInnerSnapshot::GetInst() {
  static PlanInnerSnapshot inst_;

  return &inst_;
}

PlanInnerSnapshot::PlanInnerSnapshot() { snap_mode_ = M_DISABLE; }

PlanInnerSnapshot::DataCache *PlanInnerSnapshot::get_mutable_data() {
  return &data_cache_;
}

uint64_t PlanInnerSnapshot::get_data_size() const {
  return sizeof(data_cache_);
}

PlanInnerSnapshot::SnapMode PlanInnerSnapshot::get_snap_mode() const {
  return snap_mode_;
}

void PlanInnerSnapshot::set_snap_mode(SnapMode mode) {
  if (snap_mode_ != mode) {
    snap_mode_ = mode;
  }
}

void PlanInnerSnapshot::set_speed_opt_param(
    const speed_planner::OptParamArray &param) {
  for (uint32_t i = 0; i < param.size(); i++) {
    data_cache_.speed_opt_param[i] = param[i];
  }
}

void PlanInnerSnapshot::restore_speed_opt_param(
    speed_planner::OptParamArray &param) {
  for (uint32_t i = 0; i < param.size(); i++) {
    param[i] = data_cache_.speed_opt_param[i];
  }
}

void PlanInnerSnapshot::set_path_opt_param(
    const path_planner::OptParamArray &param) {
  for (uint32_t i = 0; i < param.size(); i++) {
    data_cache_.path_opt_param[i] = param[i];
  }
}

void PlanInnerSnapshot::restore_path_opt_param(
    path_planner::OptParamArray &param) {
  for (uint32_t i = 0; i < param.size(); i++) {
    param[i] = data_cache_.path_opt_param[i];
  }
}

} // namespace msd_planning