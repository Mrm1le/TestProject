
#ifndef INNER_SANPSHOT_
#define INNER_SANPSHOT_

#include <sstream>
#include <stdint.h>

#include "mlog_core/mlog_data_stream.h"
#include "pnc/define/path_planner_interface.hpp"
#include "pnc/define/planner_constants.hpp"
#include "pnc/define/speed_planner_interface.hpp"

namespace msd_planning {

template <typename T> std::string ShowOptParam(const T &param) {
  mlog::MLogDataStream ss;
  ss << "<";

  for (uint32_t i = 0; i < param.size(); i++) {
    ss << param[i] << ", ";
  }

  ss << ">\n";

  return ss.str();
}

class PlanInnerSnapshot {
public:
  enum SnapMode { M_DISABLE = 0, M_REALCAR, M_PLAYBACK };

public:
  struct DataCache {
    double real_start_runonce_sec = 0.0;
    double real_intersection_time = 0.0;
    uint32_t plan_out_seq = 0;

    // speed planner checkpoint
    int32_t speed_real_num_iterations;
    double speed_opt_param[speed_planner::TOTAL_NUM_PARAMS];

    // path planner checkpoint
    int32_t path_real_num_iterations;
    double path_opt_param[path_planner::TOTAL_NUM_PARAMS];
  };

public:
  static PlanInnerSnapshot *GetInst();

  DataCache *get_mutable_data();

  uint64_t get_data_size() const;

  SnapMode get_snap_mode() const;

  void set_snap_mode(SnapMode mode);

  void set_speed_opt_param(const speed_planner::OptParamArray &param);

  void restore_speed_opt_param(speed_planner::OptParamArray &param);

  void set_path_opt_param(const path_planner::OptParamArray &param);

  void restore_path_opt_param(path_planner::OptParamArray &param);

private:
  PlanInnerSnapshot();

private:
  DataCache data_cache_;
  SnapMode snap_mode_ = M_DISABLE;
};

} // namespace msd_planning

#endif
