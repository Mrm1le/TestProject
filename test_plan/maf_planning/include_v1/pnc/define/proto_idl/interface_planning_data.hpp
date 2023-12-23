#ifndef CP_INTERFACE_PLANNING_DATA_H
#define CP_INTERFACE_PLANNING_DATA_H


#include "interface_planner_debug.hpp"
#include "interface_request_manager.hpp"
#include "interface_speed_planner.hpp"
#include "interface_path_planner.hpp"
#include "interface_others.hpp"
#include "interface_pass_intersection_input.hpp"
namespace cp_path_planner {

struct PlanningData {
  cp_path_planner::PathPlannerInput path_planner_input;
  cp_path_planner::LateralMotionPlannerOutput path_planner_output;

  uint64_t time_stamp_ns = 0;
  cp::RequestManagerInput request_manager_input;
  cp::RequestManagerOutput request_manager_output;
  cp_speed_planner::LonDecisionOutput lon_decison_output;
  cp::PlannerDebug planner_json_debug;
  std::vector<cp::DdpTrajectory> ddp_trajectory_info;
  cp_pass_intersection_planner::PassIntersectionPlannerInput
      pass_intersection_planner_input;
  cp_speed_planner::SpeedPlannerInput speed_planner_input;
  cp_path_planner::PathPlannerOutput speed_planner_output;
  std::string debug_json_cp;

  std::string ddp_model_input;
  bool is_ddmap;
  bool pnc_start;
  bool pnc_stop;
  double baseline_jitter;
  double v_curv;
  double a_curv;
  double max_curv;
  double max_curv_distance;
  double headway;
  bool use_eftp;
  bool lateral_use_eftp;
  bool b_dagger_longitudinal;
  bool b_dagger_lateral;
  bool use_ddld;
  uint8_t intersection_status;
};

}  // namespace path_planner

#endif