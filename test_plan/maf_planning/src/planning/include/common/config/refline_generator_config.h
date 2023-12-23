#ifndef MODULES_PLANNING_REFLINE_GENERATOR_CONFIG_H_
#define MODULES_PLANNING_REFLINE_GENERATOR_CONFIG_H_

#include "common/utils/macro.h"
#include <cassert>
#include <yaml-cpp/yaml.h>

namespace msquare {

class ReflineGeneratorConfig {
  DECLARE_SINGLETON(ReflineGeneratorConfig);

public:
  int core_type;
  bool enable_follow_target_pt;
  double shrink_ratio;
  double step_size;
  double min_turning_radius;
  double vehicle_length;
  double vehicle_width;
  double center_to_geometry_center;

  double line_repulsion_weight;
  double line_repulsion_max_distance;
  double line_repulsion_2_weight;
  double line_repulsion_2_attenuation;

  double direction_field_intensity;

  double stop_gradient_value;
  int back_extend_points_num;
  double thin_out_angle_accuracy;
  double thin_out_max_length;
  double max_length;
  double min_length;
  double lane_select_score;

  double max_look_ahead_distance;
  double min_look_ahead_distance;
  double look_ahead_time;

  double max_leader_distance;
  double min_leader_distance;
  double max_yaw_diff;
  double max_angle_diff;
  double use_agnle_diff_ratio;
  double max_l_tolerance;

  double muti_leader_discard_time;
  int max_historical_info_length;
  double traj_select_s_tolerance;
  double traj_select_l_tolerance;
  double traj_select_yaw_tolerance;
  double refline_real_length;

  double quit_cp_lat_dis_in_inter;
  double quit_cp_lat_dis_refline;
  double quit_cp_consider_refline_length;
  bool loadFile(std::string file);

  bool use_quintic;
  int farthest_index;
};

} // namespace msquare

#endif