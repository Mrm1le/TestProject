#include "common/config/refline_generator_config.h"
#include "planning/common/common.h"
#include <unistd.h>

namespace msquare {

bool ReflineGeneratorConfig::loadFile(std::string file) {
  YAML::Node yaml_node = YAML::LoadFile(file);
  core_type = yaml_node["core_type"].as<int>();
  enable_follow_target_pt = yaml_node["enable_follow_target_pt"].as<bool>();
  shrink_ratio = yaml_node["VEHICLE_MODEL"]["shrink_ratio"].as<double>();
  step_size = yaml_node["VEHICLE_MODEL"]["step_size"].as<double>();
  min_turning_radius =
      yaml_node["VEHICLE_MODEL"]["min_turning_radius"].as<double>();
  vehicle_length = yaml_node["VEHICLE_MODEL"]["vehicle_length"].as<double>();
  vehicle_width = yaml_node["VEHICLE_MODEL"]["vehicle_width"].as<double>();
  center_to_geometry_center =
      yaml_node["VEHICLE_MODEL"]["center_to_geometry_center"].as<double>();

  line_repulsion_weight =
      yaml_node["LINE_OBSTACLE"]["repulsion_weight"].as<double>();
  line_repulsion_max_distance =
      yaml_node["LINE_OBSTACLE"]["repulsion_max_distance"].as<double>();
  line_repulsion_2_weight =
      yaml_node["LINE_OBSTACLE"]["repulsion_2_weight"].as<double>();
  line_repulsion_2_attenuation =
      yaml_node["LINE_OBSTACLE"]["repulsion_2_attenuation"].as<double>();

  direction_field_intensity =
      yaml_node["DIRECTION_FIELD"]["intensity"].as<double>();

  stop_gradient_value =
      yaml_node["STRATEGY"]["stop_gradient_value"].as<double>();
  back_extend_points_num =
      yaml_node["STRATEGY"]["back_extend_points_num"].as<int>();
  thin_out_angle_accuracy =
      yaml_node["STRATEGY"]["thin_out_angle_accuracy"].as<double>();
  thin_out_max_length =
      yaml_node["STRATEGY"]["thin_out_max_length"].as<double>();
  max_length = yaml_node["STRATEGY"]["max_length"].as<double>();
  min_length = yaml_node["STRATEGY"]["min_length"].as<double>();
  lane_select_score = yaml_node["STRATEGY"]["lane_select_score"].as<double>();

  max_look_ahead_distance =
      yaml_node["ROAD_REF_SELECT"]["max_look_ahead_distance"].as<double>();
  min_look_ahead_distance =
      yaml_node["ROAD_REF_SELECT"]["min_look_ahead_distance"].as<double>();
  look_ahead_time =
      yaml_node["ROAD_REF_SELECT"]["look_ahead_time"].as<double>();

  max_leader_distance =
      yaml_node["LEADER_SELECT"]["max_leader_distance"].as<double>();
  min_leader_distance =
      yaml_node["LEADER_SELECT"]["min_leader_distance"].as<double>();
  max_yaw_diff = yaml_node["LEADER_SELECT"]["max_yaw_diff"].as<double>();
  max_angle_diff = yaml_node["LEADER_SELECT"]["max_angle_diff"].as<double>();
  use_agnle_diff_ratio =
      yaml_node["LEADER_SELECT"]["use_agnle_diff_ratio"].as<double>();
  max_l_tolerance = yaml_node["LEADER_SELECT"]["max_l_tolerance"].as<double>();

  muti_leader_discard_time =
      yaml_node["MUTI_LEADER_HISTORICAL"]["discard_time"].as<double>();
  max_historical_info_length =
      yaml_node["MUTI_LEADER_HISTORICAL"]["max_historical_info_length"]
          .as<int>();
  traj_select_s_tolerance =
      yaml_node["MUTI_LEADER_HISTORICAL"]["traj_select_s_tolerance"]
          .as<double>();
  traj_select_l_tolerance =
      yaml_node["MUTI_LEADER_HISTORICAL"]["traj_select_l_tolerance"]
          .as<double>();
  traj_select_yaw_tolerance =
      yaml_node["MUTI_LEADER_HISTORICAL"]["traj_select_yaw_tolerance"]
          .as<double>();
  refline_real_length =
      yaml_node["MUTI_LEADER_HISTORICAL"]["refline_real_length"].as<double>();

  quit_cp_lat_dis_in_inter =
      yaml_node["QUIT_CP"]["quit_cp_lat_dis_in_inter"].as<double>();
  quit_cp_lat_dis_refline =
      yaml_node["QUIT_CP"]["quit_cp_lat_dis_refline"].as<double>();
  quit_cp_consider_refline_length =
      yaml_node["QUIT_CP"]["quit_cp_consider_refline_length"].as<double>();

  use_quintic = yaml_node["QUINTIC"]["use_quintic"].as<bool>();
  farthest_index = yaml_node["QUINTIC"]["farthest_index"].as<int>();

  return true;
}

} // namespace msquare
