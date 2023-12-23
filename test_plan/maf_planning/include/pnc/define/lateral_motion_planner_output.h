#ifndef COMMON_LATERAL_MOTION_PLANNER_OUTPUT_
#define COMMON_LATERAL_MOTION_PLANNER_OUTPUT_

#include <array>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include "path_planner_interface.hpp"
#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace msquare {

struct LateralMotionPlannerOutput {
  Point2D prev_lc_end_point;
  std::unordered_map<int, std::vector<double>>
      lat_avoid_obstacle_history_info_map{};
  std::vector<path_planner::SampleDebugInfo> path_planner_debug_info{};
  path_planner::PathPlannerOutput prev_path_planner_output{};
  path_planner::PathPlannerOutput path_planner_output{};
  path_planner::Intelligent_Dodge_Info pre_dodge_info{};
  path_planner::DLPInfo pre_dlp_info{};


  void clear() {
    prev_lc_end_point = {};
    lat_avoid_obstacle_history_info_map = {};
    prev_path_planner_output.path_planner_output.clear();
    path_planner_output = {};
    pre_dodge_info = {};
    pre_dlp_info = {};
  }
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LateralMotionPlannerOutput,
                                   path_planner_debug_info, path_planner_output,
                                   pre_dodge_info, pre_dlp_info)

} // namespace msquare

#endif
