#include <common/config/lateral_planning_config.h>

namespace msquare {
bool ParkingLateralBehaviorPlannerConfig::loadFile(std::string file) {
  YAML::Node yaml_node = YAML::LoadFile(file);
  using_teb_g2o_ = yaml_node["using_teb_g2o"].as<bool>();
  using_virtual_box_complex_ =
      yaml_node["using_virtual_box_complex"].as<bool>();

  kPathN_default_ = yaml_node["kPathN_default"].as<int>();
  FRENET_CHECK_S_START_ = yaml_node["FRENET_CHECK_S_START"].as<double>();
  FRENET_CHECK_S_END_ = yaml_node["FRENET_CHECK_S_END"].as<double>();

  // OBS_SIDEPASS_PARAM:
  Duration_hold_time_delault_ =
      yaml_node["OBS_SIDEPASS_PARAM"]["Duration_hold_time_delault"]
          .as<double>();
  sidepass_buffer_ =
      yaml_node["OBS_SIDEPASS_PARAM"]["sidepass_buffer"].as<double>();
  road_middle_width_delault_ =
      yaml_node["OBS_SIDEPASS_PARAM"]["road_middle_width_delault"].as<double>();
  sidepass_buffer_upper_bound_default_ =
      yaml_node["OBS_SIDEPASS_PARAM"]["sidepass_buffer_upper_bound_default"]
          .as<double>();
  opposite_car_pass_buffer_factor_ =
      yaml_node["OBS_SIDEPASS_PARAM"]["opposite_car_pass_buffer_factor"]
          .as<double>();
  ped_road_middle_width_ =
      yaml_node["OBS_SIDEPASS_PARAM"]["ped_road_middle_width"].as<double>();
  ped_sidepass_buffer_upper_bound_ =
      yaml_node["OBS_SIDEPASS_PARAM"]["ped_sidepass_buffer_upper_bound"]
          .as<double>();
  car_hysteresis_offset_ =
      yaml_node["OBS_SIDEPASS_PARAM"]["car_hysteresis_offset"].as<double>();
  ped_hysteresis_offset_ =
      yaml_node["OBS_SIDEPASS_PARAM"]["ped_hysteresis_offset"].as<double>();
  sidepass_buffer_default_ =
      yaml_node["OBS_SIDEPASS_PARAM"]["sidepass_buffer_default"].as<double>();

  // STRATEGY1_PARAM:
  PASS_BUFFER_ = yaml_node["STRATEGY1_PARAM"]["PASS_BUFFER"].as<double>();
  DIST_SQUARE_THRE_ =
      yaml_node["STRATEGY1_PARAM"]["DIST_SQUARE_THRE"].as<double>();

  // BRANCH_AND_BOUND_PARAM:
  alpha_ = yaml_node["BRANCH_AND_BOUND_PARAM"]["alpha"].as<double>();
  alpha_f2_ = yaml_node["BRANCH_AND_BOUND_PARAM"]["alpha_f2"].as<double>();
  virtual_car_block_cost_ =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["virtual_car_block_cost"]
          .as<double>();
  ;
  opposite_car_block_cost_ =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["opposite_car_block_cost"]
          .as<double>();
  freespace_block_cost_ =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["freespace_block_cost"].as<double>();
  couple_block_cost_ =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["couple_block_cost"].as<double>();
  ped_block_cost_ =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["ped_block_cost"].as<double>();
  influence_obs_end_s_addition_factor_ =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["influence_obs_end_s_addition_factor"]
          .as<double>();
  influence_obs_end_s_addition_offset_ =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["influence_obs_end_s_addition_offset"]
          .as<double>();
  f1_molecular_ =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["f1_molecular"].as<double>();
  f1_denominator_plus1_ =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["f1_denominator_plus1"].as<double>();
  f1_denominator_exp_factor_ =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["f1_denominator_exp_factor"]
          .as<double>();
  f2_delta_l_harmony_offset_ =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["f2_delta_l_harmony_offset"]
          .as<double>();
  f2_molecular_ =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["f2_molecular"].as<double>();
  f2_denominator_plus1_ =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["f2_denominator_plus1"].as<double>();
  f2_denominator_exp_factor_ =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["f2_denominator_exp_factor"]
          .as<double>();
  f2_denominator_exp_offset_ =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["f2_denominator_exp_offset"]
          .as<double>();

  // LATERAL_PASSABLE_INTERP_S_ =
  // yaml_node["BRANCH_AND_BOUND_PARAM"][""].as<double>();
  // LATERAL_PASSABLE_INTERP_L_ =
  // yaml_node["BRANCH_AND_BOUND_PARAM"][""].as<double>();
  YAML::Node LATERAL_PASSABLE_INTERP_S =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["LATERAL_PASSABLE_INTERP_S"];
  YAML::iterator iter = LATERAL_PASSABLE_INTERP_S.begin();
  while (iter != LATERAL_PASSABLE_INTERP_S.end()) {
    LATERAL_PASSABLE_INTERP_S_.emplace_back((*iter).as<double>());
    ++iter;
  }
  YAML::Node LATERAL_PASSABLE_INTERP_L =
      yaml_node["BRANCH_AND_BOUND_PARAM"]["LATERAL_PASSABLE_INTERP_L"];
  YAML::iterator iter_L = LATERAL_PASSABLE_INTERP_L.begin();
  while (iter_L != LATERAL_PASSABLE_INTERP_L.end()) {
    LATERAL_PASSABLE_INTERP_L_.emplace_back((*iter_L).as<double>());
    ++iter_L;
  }
  return true;
}

bool TebOpenspaceDeciderConfig::loadFile(std::string file) {
  YAML::Node yaml_node = YAML::LoadFile(file);
  // OBSTACLE_AVOID_PARAM:
  default_swell_distance_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["default_swell_distance"].as<double>();
  default_step_size_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["default_step_size"].as<double>();
  min_obstacle_dist_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["min_obstacle_dist"].as<double>();
  min_static_obstacle_dist_default_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["min_static_obstacle_dist_default"]
          .as<double>();
  physics_border_offset_default_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["physics_border_offset_default"]
          .as<double>();
  virtual_border_offset_default_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["virtual_border_offset_default"]
          .as<double>();
  pillar_offset_default_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["pillar_offset_default"].as<double>();
  couple_offset_default_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["couple_offset_default"].as<double>();
  ped_offset_default_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["ped_offset_default"].as<double>();
  freespace_offset_default_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["freespace_offset_default"]
          .as<double>();
  lead_offset_default_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["lead_offset_default"].as<double>();
  l_range_hack_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["l_range_hack"].as<double>();
  min_obstacle_dist_addition_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["min_obstacle_dist_addition"]
          .as<double>();
  min_obstacle_dist_factor_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["min_obstacle_dist_factor"]
          .as<double>();
  min_obstacle_dist_max_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["min_obstacle_dist_max"].as<double>();
  min_obstacle_dist_min_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["min_obstacle_dist_min"].as<double>();
  min_obstacle_dist_entrance_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["min_obstacle_dist_entrance"]
          .as<double>();
  couple_offset_factor_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["couple_offset_factor"].as<double>();
  couple_offset_min_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["couple_offset_min"].as<double>();
  min_static_obstacle_dist_min_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["min_static_obstacle_dist_min"]
          .as<double>();
  min_static_obstacle_dist_offset_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["min_static_obstacle_dist_offset"]
          .as<double>();
  min_virtual_obstacle_dist_check_upper_bound_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]
               ["min_virtual_obstacle_dist_check_upper_bound"]
                   .as<double>();
  min_free_obstacle_dist_check_upper_bound_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]
               ["min_free_obstacle_dist_check_upper_bound"]
                   .as<double>();
  min_obstacle_dist_check_upper_bound_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["min_obstacle_dist_check_upper_bound"]
          .as<double>();
  min_fix_obstacle_dist_check_lower_bound_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]
               ["min_fix_obstacle_dist_check_lower_bound"]
                   .as<double>();
  couple_offset_max_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["couple_offset_max"].as<double>();
  couple_offset_attenuation_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["couple_offset_attenuation"]
          .as<double>();
  pillar_offset_min_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["pillar_offset_min"].as<double>();
  pillar_offset_max_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["pillar_offset_max"].as<double>();
  pillar_offset_attenuation_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["pillar_offset_attenuation"]
          .as<double>();
  pillar_offset_factor_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["pillar_offset_factor"].as<double>();
  pillar_offset_offset_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["pillar_offset_offset"].as<double>();
  pillar_offset_l_range_min_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["pillar_offset_l_range_min"]
          .as<double>();
  lead_offset_max_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["lead_offset_max"].as<double>();
  lead_offset_min_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["lead_offset_min"].as<double>();
  lead_offset_l_range_offset_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["lead_offset_l_range_offset"]
          .as<double>();
  couple_offset_interf_addition_ =
      yaml_node["OBSTACLE_AVOID_PARAM"]["couple_offset_interf_addition"]
          .as<double>();

  // TRAGECTORY_GENERATOR_PARAM:
  default_distance_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]["default_distance"].as<double>();
  end_s_add_default_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]["end_s_add_default"].as<double>();
  off_track_l_limit_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]["off_track_l_limit"].as<double>();
  end_s_add_off_track_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]["end_s_add_off_track"]
          .as<double>();
  distance_length_addition_default_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]
               ["distance_length_addition_default"]
                   .as<double>();
  distance_length_left_left_addition_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]
               ["distance_length_left_left_addition"]
                   .as<double>();
  turn_start_curv_limit_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]["turn_start_curv_limit"]
          .as<double>();
  turn_end_curv_limit_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]["turn_end_curv_limit"]
          .as<double>();
  end_s_cutoff_offset_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]["end_s_cutoff_offset"]
          .as<double>();
  length_upper_check_offset_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]["length_upper_check_offset"]
          .as<double>();
  goal_generate_after_turn_offset_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]["goal_generate_after_turn_offset"]
          .as<double>();
  target_effective_check_s_step_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]["target_effective_check_s_step"]
          .as<double>();
  target_effective_max_curv_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]["target_effective_max_curv"]
          .as<double>();
  target_effective_count_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]["target_effective_count"]
          .as<double>();
  target_effective_end_s_offset_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]["target_effective_end_s_offset"]
          .as<double>();
  target_effective_box_collision_limit_ =
      yaml_node["TRAGECTORY_GENERATOR_PARAM"]
               ["target_effective_box_collision_limit"]
                   .as<double>();
  return true;
}

bool ViaPointDeciderConfig::loadFile(std::string file) {
  YAML::Node yaml_node = YAML::LoadFile(file);
  // VIA_WEIGHT_PARAM:
  YAML::Node VIA_WEIGHT_LANE = yaml_node["VIA_WEIGHT_PARAM"]["VIA_WEIGHT_LANE"];
  YAML::iterator iter = VIA_WEIGHT_LANE.begin();
  while (iter != VIA_WEIGHT_LANE.end()) {
    VIA_WEIGHT_LANE_.emplace_back((*iter).as<double>());
    iter++;
  }
  YAML::Node VIA_WEIGHT_LANE_INTERSECTION =
      yaml_node["VIA_WEIGHT_PARAM"]["VIA_WEIGHT_LANE_INTERSECTION"];
  iter = VIA_WEIGHT_LANE_INTERSECTION.begin();
  while (iter != VIA_WEIGHT_LANE_INTERSECTION.end()) {
    VIA_WEIGHT_LANE_INTERSECTION_.emplace_back((*iter).as<double>());
    iter++;
  }
  YAML::Node VIA_WEIGHT_DISTANCE =
      yaml_node["VIA_WEIGHT_PARAM"]["VIA_WEIGHT_DISTANCE"];
  iter = VIA_WEIGHT_DISTANCE.begin();
  while (iter != VIA_WEIGHT_DISTANCE.end()) {
    VIA_WEIGHT_DISTANCE_.emplace_back((*iter).as<double>());
    iter++;
  }
  YAML::Node VIA_WEIGHT_EGO = yaml_node["VIA_WEIGHT_PARAM"]["VIA_WEIGHT_EGO"];
  iter = VIA_WEIGHT_EGO.begin();
  while (iter != VIA_WEIGHT_EGO.end()) {
    VIA_WEIGHT_EGO_.emplace_back((*iter).as<double>());
    iter++;
  }
  WEIGHT_VIA_BASE_INTERSECTION_ =
      yaml_node["VIA_WEIGHT_PARAM"]["WEIGHT_VIA_BASE_INTERSECTION"]
          .as<double>();
  WEIGHT_VIA_BASE_LANE_ =
      yaml_node["VIA_WEIGHT_PARAM"]["WEIGHT_VIA_BASE_LANE"].as<double>();
  interf_dist_limit_ =
      yaml_node["VIA_WEIGHT_PARAM"]["interf_dist_limit"].as<double>();
  off_track_l_limit_ =
      yaml_node["VIA_WEIGHT_PARAM"]["off_track_l_limit"].as<double>();
  clip_off_track_l_min_ =
      yaml_node["VIA_WEIGHT_PARAM"]["clip_off_track_l_min"].as<double>();
  clip_off_track_l_max_ =
      yaml_node["VIA_WEIGHT_PARAM"]["clip_off_track_l_max"].as<double>();
  weight_factor_ = yaml_node["VIA_WEIGHT_PARAM"]["weight_factor"].as<double>();
  via_traj_lookahead_time_ =
      yaml_node["VIA_WEIGHT_PARAM"]["via_traj_lookahead_time"].as<double>();
  distance_ahead_min_ =
      yaml_node["VIA_WEIGHT_PARAM"]["distance_ahead_min"].as<double>();
  front_via_limit_range_ =
      yaml_node["VIA_WEIGHT_PARAM"]["front_via_limit_range"].as<double>();

  // VIA_ADD_PARAM:
  curve_theta_check_limit_ =
      yaml_node["VIA_ADD_PARAM"]["curve_theta_check_limit"].as<double>();
  long_lane_min_ = yaml_node["VIA_ADD_PARAM"]["long_lane_min"].as<double>();
  double_lane_border_distance_ =
      yaml_node["VIA_ADD_PARAM"]["double_lane_border_distance"].as<double>();
  avg_curvature_limit_ =
      yaml_node["VIA_ADD_PARAM"]["avg_curvature_limit"].as<double>();
  straight_lane_length_before_limit_ =
      yaml_node["VIA_ADD_PARAM"]["straight_lane_length_before_limit"]
          .as<double>();
  straight_lane_length_after_limit_ =
      yaml_node["VIA_ADD_PARAM"]["straight_lane_length_after_limit"]
          .as<double>();
  road_width_middle_min_ =
      yaml_node["VIA_ADD_PARAM"]["road_width_middle_min"].as<double>();
  road_width_middle_max_ =
      yaml_node["VIA_ADD_PARAM"]["road_width_middle_max"].as<double>();
  turn_width_diff_limit_ =
      yaml_node["VIA_ADD_PARAM"]["turn_width_diff_limit"].as<double>();
  double_lane_length_middle_max_ =
      yaml_node["VIA_ADD_PARAM"]["double_lane_length_middle_max"].as<double>();
  double_lane_length_middle_min_ =
      yaml_node["VIA_ADD_PARAM"]["double_lane_length_middle_min"].as<double>();
  curve_lane_point_curv_max_ =
      yaml_node["VIA_ADD_PARAM"]["curve_lane_point_curv_max"].as<double>();
  curve_lane_point_curv_mmin_ =
      yaml_node["VIA_ADD_PARAM"]["curve_lane_point_curv_mmin"].as<double>();
  ref_trajectory_info_via_delta_s_filter_ =
      yaml_node["VIA_ADD_PARAM"]["ref_trajectory_info_via_delta_s_filter"]
          .as<double>();
  epsilon_ = yaml_node["VIA_ADD_PARAM"]["epsilon"].as<double>();
  check_turn_delta_s_ =
      yaml_node["VIA_ADD_PARAM"]["check_turn_delta_s"].as<double>();
  check_turn_pt_count_ =
      yaml_node["VIA_ADD_PARAM"]["check_turn_pt_count"].as<double>();
  check_turn_pt_invalid_limit_ =
      yaml_node["VIA_ADD_PARAM"]["check_turn_pt_invalid_limit"].as<double>();
  return true;
}

bool ApfDeciderConfig::loadFile(std::string file) {
  YAML::Node yaml_node = YAML::LoadFile(file);

  use_apf_refline_ = yaml_node["use_apf_refline"].as<bool>();
  move_apf_traj_ = yaml_node["move_apf_traj"].as<bool>();
  move_apf_traj_ds_ = yaml_node["move_apf_traj_ds"].as<double>();
  use_teb_ = yaml_node["use_teb"].as<bool>();

  is_store_ = yaml_node["is_store"].as<bool>();
  store_path_ =
      "/home/ros/Downloads/" + yaml_node["store_path"].as<std::string>();
  clear_files_ = yaml_node["clear_files"].as<bool>();
  store_old_traj_ = yaml_node["store_old_traj"].as<bool>();
  store_freq_div_ = yaml_node["store_freq_div"].as<int>();

  gradient_descent_distance_ =
      yaml_node["gradient_descent_distance"].as<double>();
  max_trajectory_points_num_ = yaml_node["max_trajectory_points_num"].as<int>();
  min_trajectory_points_num_ = yaml_node["min_trajectory_points_num"].as<int>();
  back_points_num_ = yaml_node["back_points_num"].as<int>();
  min_straight_trajectory_points_num_ =
      yaml_node["min_straight_trajectory_points_num"].as<int>();
  min_turning_trajectory_points_num_ =
      yaml_node["min_turning_trajectory_points_num"].as<int>();
  apf_replan_threshold_ = yaml_node["apf_replan_threshold"].as<double>();
  line_segment_min_size_ = yaml_node["line_segment_min_size"].as<double>();

  road_direction_circle_radius_wall_ =
      yaml_node["road_direction_circle_radius_wall"].as<double>();
  road_direction_circle_radius_lot_ =
      yaml_node["road_direction_circle_radius_lot"].as<double>();
  road_direction_wall_considered_length_ =
      yaml_node["road_direction_wall_considered_length"].as<double>();

  vehicle_length_added_ = yaml_node["vehicle_length_added"].as<double>();
  vehicle_length_ = yaml_node["vehicle_length"].as<double>();
  vehicle_width_ = yaml_node["vehicle_width"].as<double>();
  min_turning_radius_ = yaml_node["min_turning_radius"].as<double>();
  wheelbase_ = yaml_node["wheelbase"].as<double>();
  max_dtheta_ = gradient_descent_distance_ / min_turning_radius_;

  forward_gradient_intensity_ =
      yaml_node["forward_gradient_intensity"].as<double>();
  turning_gradient_intensity_ =
      yaml_node["turning_gradient_intensity"].as<double>();
  turning_gradient_dtheta_ratio_ =
      yaml_node["turning_gradient_dtheta_ratio"].as<double>();
  relieve_fixed_gradient_dtheta_ =
      yaml_node["relieve_fixed_gradient_dtheta"].as<double>();
  stop_gradient_value_ = yaml_node["stop_gradient_value"].as<double>();

  use_map_potential_field_ = yaml_node["use_map_potential_field"].as<bool>();
  wall_repulsion_weight_ = yaml_node["wall_repulsion_weight"].as<double>();
  wall_repulsion_max_distance_ =
      yaml_node["wall_repulsion_max_distance"].as<double>();
  wall_repulsion_2_weight_ = yaml_node["wall_repulsion_2_weight"].as<double>();
  wall_repulsion_2_attenuation_ =
      yaml_node["wall_repulsion_2_attenuation"].as<double>();

  pillar_repulsion_weight_ = yaml_node["pillar_repulsion_weight"].as<double>();
  pillar_repulsion_max_distance_ =
      yaml_node["pillar_repulsion_max_distance"].as<double>();
  pillar_repulsion_2_weight_ =
      yaml_node["pillar_repulsion_2_weight"].as<double>();
  pillar_repulsion_2_attenuation_ =
      yaml_node["pillar_repulsion_2_attenuation"].as<double>();

  is_parking_lot_from_file_ = yaml_node["is_parking_lot_from_file"].as<bool>();
  parking_lot_file_ = yaml_node["parking_lot_file"].as<std::string>();
  parking_lot_circle_radius_ =
      yaml_node["parking_lot_circle_radius"].as<double>();
  parking_lot_circle_offset_ =
      yaml_node["parking_lot_circle_offset"].as<double>();
  parking_lot_z_offset_ = yaml_node["parking_lot_z_offset"].as<double>();
  praking_lot_z_deviation_ = yaml_node["praking_lot_z_deviation"].as<double>();

  parking_lot_repulsion_weight_ =
      yaml_node["parking_lot_repulsion_weight"].as<double>();
  parking_lot_repulsion_max_distance_ =
      yaml_node["parking_lot_repulsion_max_distance"].as<double>();
  parking_lot_repulsion_2_weight_ =
      yaml_node["parking_lot_repulsion_2_weight"].as<double>();
  parking_lot_repulsion_2_attenuation_ =
      yaml_node["parking_lot_repulsion_2_attenuation"].as<double>();

  use_ground_point_potential_field_ =
      yaml_node["use_ground_point_potential_field"].as<bool>();
  ground_point_repulsion_weight_ =
      yaml_node["ground_point_repulsion_weight"].as<double>();
  ground_point_repulsion_max_distance_ =
      yaml_node["ground_point_repulsion_max_distance"].as<double>();
  ground_point_repulsion_2_weight_ =
      yaml_node["ground_point_repulsion_2_weight"].as<double>();
  ground_point_repulsion_2_attenuation_ =
      yaml_node["ground_point_repulsion_2_attenuation"].as<double>();

  teb_replan_start_dis_threshold_ =
      yaml_node["teb_replan_start_dis_threshold"].as<double>();
  teb_replan_start_theta_threshold_ =
      yaml_node["teb_replan_start_theta_threshold"].as<double>() * M_PI_2;
  teb_replan_end_dis_threshold_ =
      yaml_node["teb_replan_end_dis_threshold"].as<double>();
  teb_replan_end_theta_threshold_ =
      yaml_node["teb_replan_end_theta_threshold"].as<double>() * M_PI_2;
  teb_use_parking_lot_ = yaml_node["teb_use_parking_lot"].as<bool>();

  min_reasonable_s_ = yaml_node["min_reasonable_s"].as<double>();
  return true;
}

}; // namespace msquare