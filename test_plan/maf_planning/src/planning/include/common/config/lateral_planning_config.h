#pragma once

#include "common/utils/macro.h"
#include <cmath>
#include <yaml-cpp/yaml.h>

namespace msquare {
class ParkingLateralBehaviorPlannerConfig {
  DECLARE_SINGLETON(ParkingLateralBehaviorPlannerConfig);

public:
  bool using_teb_g2o_;
  bool using_virtual_box_complex_;

  int kPathN_default_;
  double FRENET_CHECK_S_START_;
  double FRENET_CHECK_S_END_;

  // OBS_SIDEPASS_PARAM:
  double Duration_hold_time_delault_;
  double sidepass_buffer_;
  double road_middle_width_delault_;
  double sidepass_buffer_upper_bound_default_;
  double opposite_car_pass_buffer_factor_;
  double ped_road_middle_width_;
  double ped_sidepass_buffer_upper_bound_;
  double car_hysteresis_offset_;
  double ped_hysteresis_offset_;
  double sidepass_buffer_default_;

  // STRATEGY1_PARAM:
  double PASS_BUFFER_;
  double DIST_SQUARE_THRE_;

  // BRANCH_AND_BOUND_PARAM:
  double alpha_;
  double alpha_f2_;
  double virtual_car_block_cost_;
  double opposite_car_block_cost_;
  double freespace_block_cost_;
  double couple_block_cost_;
  double ped_block_cost_;
  double influence_obs_end_s_addition_factor_;
  double influence_obs_end_s_addition_offset_;
  double f1_molecular_;
  double f1_denominator_plus1_;
  double f1_denominator_exp_factor_;
  double f2_delta_l_harmony_offset_;
  double f2_molecular_;
  double f2_denominator_plus1_;
  double f2_denominator_exp_factor_;
  double f2_denominator_exp_offset_;

  std::vector<double> LATERAL_PASSABLE_INTERP_S_;
  std::vector<double> LATERAL_PASSABLE_INTERP_L_;

  bool loadFile(std::string file);
};

class TebOpenspaceDeciderConfig {
  DECLARE_SINGLETON(TebOpenspaceDeciderConfig);

public:
  // OBSTACLE_AVOID_PARAM:
  double default_swell_distance_;
  double default_step_size_;
  double min_obstacle_dist_;
  double min_static_obstacle_dist_default_;
  double physics_border_offset_default_;
  double virtual_border_offset_default_;
  double pillar_offset_default_;
  double couple_offset_default_;
  double ped_offset_default_;
  double freespace_offset_default_;
  double lead_offset_default_;
  double l_range_hack_;
  double min_obstacle_dist_addition_;
  double min_obstacle_dist_factor_;
  double min_obstacle_dist_max_;
  double min_obstacle_dist_min_;
  double min_obstacle_dist_entrance_;
  double couple_offset_factor_;
  double couple_offset_min_;
  double min_static_obstacle_dist_min_;
  double min_static_obstacle_dist_offset_;
  double min_virtual_obstacle_dist_check_upper_bound_;
  double min_free_obstacle_dist_check_upper_bound_;
  double min_obstacle_dist_check_upper_bound_;
  double min_fix_obstacle_dist_check_lower_bound_;
  double couple_offset_max_;
  double couple_offset_attenuation_;
  double pillar_offset_min_;
  double pillar_offset_max_;
  double pillar_offset_attenuation_;
  double pillar_offset_factor_;
  double pillar_offset_offset_;
  double pillar_offset_l_range_min_;
  double lead_offset_max_;
  double lead_offset_min_;
  double lead_offset_l_range_offset_;
  double couple_offset_interf_addition_;

  // TRAGECTORY_GENERATOR_PARAM:
  double default_distance_;
  double end_s_add_default_;
  double off_track_l_limit_;
  double end_s_add_off_track_;
  double distance_length_addition_default_;
  double distance_length_left_left_addition_;
  double turn_start_curv_limit_;
  double turn_end_curv_limit_;
  double end_s_cutoff_offset_;
  double length_upper_check_offset_;
  double goal_generate_after_turn_offset_;
  double target_effective_check_s_step_;
  double target_effective_max_curv_;
  double target_effective_count_;
  double target_effective_end_s_offset_;
  double target_effective_box_collision_limit_;

  bool loadFile(std::string file);
};

class ViaPointDeciderConfig {
  DECLARE_SINGLETON(ViaPointDeciderConfig);

public:
  // VIA_WEIGHT_PARAM:
  std::vector<double> VIA_WEIGHT_LANE_;
  std::vector<double> VIA_WEIGHT_LANE_INTERSECTION_;
  std::vector<double> VIA_WEIGHT_DISTANCE_;
  std::vector<double> VIA_WEIGHT_EGO_;
  double WEIGHT_VIA_BASE_INTERSECTION_;
  double WEIGHT_VIA_BASE_LANE_;
  double interf_dist_limit_;
  double off_track_l_limit_;
  double clip_off_track_l_min_;
  double clip_off_track_l_max_;
  double weight_factor_;
  double via_traj_lookahead_time_;
  double distance_ahead_min_;
  double front_via_limit_range_;

  // VIA_ADD_PARAM:
  double curve_theta_check_limit_;
  double long_lane_min_;
  double double_lane_border_distance_;
  double avg_curvature_limit_;
  double straight_lane_length_before_limit_;
  double straight_lane_length_after_limit_;
  double road_width_middle_min_;
  double road_width_middle_max_;
  double turn_width_diff_limit_;
  double double_lane_length_middle_max_;
  double double_lane_length_middle_min_;
  double curve_lane_point_curv_max_;
  double curve_lane_point_curv_mmin_;
  double ref_trajectory_info_via_delta_s_filter_;
  double epsilon_;
  double check_turn_delta_s_;
  int check_turn_pt_count_;
  int check_turn_pt_invalid_limit_;

  bool loadFile(std::string file);
};

class ApfDeciderConfig {
  DECLARE_SINGLETON(ApfDeciderConfig);

public:
  // for apf settings
  bool use_apf_refline_ = false;
  bool move_apf_traj_ = true;
  double move_apf_traj_ds_ = 1.4;
  bool use_teb_;

  // for store
  bool is_store_;
  std::string store_path_;
  bool clear_files_;
  bool store_old_traj_;
  int store_freq_div_;

  // for apf trajectory settings
  double gradient_descent_distance_;
  int max_trajectory_points_num_;
  int min_trajectory_points_num_;
  int back_points_num_;
  int min_straight_trajectory_points_num_;
  int min_turning_trajectory_points_num_;
  double relieve_fixed_gradient_dtheta_;
  double apf_replan_threshold_;
  double line_segment_min_size_;

  // for calc road direction
  double road_direction_circle_radius_wall_;
  double road_direction_circle_radius_lot_;
  double road_direction_wall_considered_length_;
  // for ego vehicle bbox
  double vehicle_length_added_;
  double vehicle_length_;
  double vehicle_width_;
  double min_turning_radius_;
  double wheelbase_;
  double max_dtheta_;

  // for attached gradient
  double forward_gradient_intensity_;
  double turning_gradient_intensity_;
  double turning_gradient_dtheta_ratio_;
  double stop_gradient_value_;

  // for wall repulision gradient
  bool use_map_potential_field_;
  double wall_repulsion_weight_;
  double wall_repulsion_max_distance_;
  double wall_repulsion_2_weight_;
  double wall_repulsion_2_attenuation_;

  // for pillar repulision gradient
  double pillar_repulsion_weight_;
  double pillar_repulsion_max_distance_;
  double pillar_repulsion_2_weight_;
  double pillar_repulsion_2_attenuation_;

  // parking lot
  bool is_parking_lot_from_file_;
  std::string parking_lot_file_;
  double parking_lot_circle_radius_;
  double parking_lot_circle_offset_;
  double parking_lot_z_offset_;
  double praking_lot_z_deviation_;
  double parking_lot_repulsion_weight_;
  double parking_lot_repulsion_max_distance_;
  double parking_lot_repulsion_2_weight_;
  double parking_lot_repulsion_2_attenuation_;

  // ground points
  bool use_ground_point_potential_field_;
  double ground_point_repulsion_weight_;
  double ground_point_repulsion_max_distance_;
  double ground_point_repulsion_2_weight_;
  double ground_point_repulsion_2_attenuation_;

  // for teb settings
  double teb_replan_start_dis_threshold_;
  double teb_replan_start_theta_threshold_;
  double teb_replan_end_dis_threshold_;
  double teb_replan_end_theta_threshold_;
  bool teb_use_parking_lot_;

  // for check reasonable
  double min_reasonable_s_;

  bool loadFile(std::string file);
};

} // namespace msquare
