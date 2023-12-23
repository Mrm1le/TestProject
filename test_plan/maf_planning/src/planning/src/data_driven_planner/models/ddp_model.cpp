#ifdef USE_TORCH

#include "data_driven_planner/models/ddp_model.h"

#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/mdebug_context.h"
#include "common/planning_fail_tracer.h"
#include <iostream>

#include "data_driven_planner/common/base64.h"
#include "data_driven_planner/common/ddp_context.h"
#include "data_driven_planner/common/ddp_utils.h"
#include "data_driven_planner/common/planning_result_manager.h"
#include "data_driven_planner/models/ego_pose_manager.h"
#include "data_driven_planner/models/fusion_object_manager.h"
// #include "data_driven_planner/models/hdmap_adapter.h"
// #include "data_driven_planner/models/traffic_light_info_manager.h"
#include "mjson/mjson.hpp"

namespace msquare {
namespace ddp {

static inline void save_binary_input(std::string bin_file, const float *data,
                                     int size) {
  // 静态检查不准有ofstream
  // std::ofstream outfile(bin_file, std::ios::binary);
  // outfile.write(reinterpret_cast<const char *>(data), size*4);
  // outfile.close();
}

DdpModel::DdpModel(const std::shared_ptr<DdpConfigBuilder> &config_builder) {}

bool DdpModel::run(double timestamp, DdpTrajectorys &ddp_trajectorys,
                   std::string &ddp_model_input, bool is_replan) {
  return true;
}

} // namespace ddp
} // namespace msquare

#endif
