#pragma once
#include "nlohmann/json.hpp"
#include <mutex>
#include <yaml-cpp/yaml.h>

namespace worldmodel_pec {

struct SlotReleaseParams {

public:
  SlotReleaseParams();
  virtual ~SlotReleaseParams();
  double compensate_time;
  double max_compensate_dist;
  bool enable_oblique_slot;
  double vertical_passage_width;
  double vertical_passage_width_hys_scope;
  double vertical_slot_width_hys_scope;
  double parallel_passage_width;
  double parallel_passage_width_hys_scope;
  double parallel_slot_width_hys_scope;
  double oblique_passage_width;
  double oblique_passage_width_hys_scope;
  double oblique_slot_width_hys_scope;
  double slot_extend_depth_ratio;
  double slot_in_extend_depth;
  double ego_side_obstacel_distance_low;
  double ego_side_obstacel_distance_high;
  double ego_side_obs_dis_side_slot_low;
  double ego_side_obs_dis_side_slot_high;

  double vertical_slot_width;
  // parallel slot parameter
  double min_width_parallel;
  double min_length_parallel;
  double min_length_parallel_step_bottom;

  // parkable area parameter
  double parkable_area_x;
  double parkable_area_y;
  double uss_friendly_area_y_min;
  double uss_friendly_area_y_max;
  double uss_friendly_area_y_max_parallel;
  double forward_pass_dist;
  double backward_pass_dist;

  // other parameter
  double groundline_save_area_extend_width;
  double valid_passage_length;
  double velocity_limit;
  double pitch_limit;
  double yaw_limit;
  double space_yam_max_limit;

  NLOHMANN_DEFINE_TYPE_INTRUSIVE(SlotReleaseParams,
                                 compensate_time,
                                 max_compensate_dist, 
                                 enable_oblique_slot,
                                 vertical_passage_width,
                                 vertical_passage_width_hys_scope,
                                 vertical_slot_width_hys_scope,
                                 parallel_passage_width,
                                 parallel_passage_width_hys_scope,
                                 parallel_slot_width_hys_scope,
                                 oblique_passage_width,
                                 oblique_passage_width_hys_scope,
                                 oblique_slot_width_hys_scope,
                                 slot_extend_depth_ratio,
                                 slot_in_extend_depth,
                                 ego_side_obstacel_distance_low,
                                 ego_side_obstacel_distance_high,
                                 ego_side_obs_dis_side_slot_low,
                                 ego_side_obs_dis_side_slot_high,
                                 vertical_slot_width,
                                 min_width_parallel,
                                 min_length_parallel,
                                 min_length_parallel_step_bottom,
                                 parkable_area_x,
                                 parkable_area_y,
                                 uss_friendly_area_y_min,
                                 uss_friendly_area_y_max,
                                 uss_friendly_area_y_max_parallel,
                                 forward_pass_dist,
                                 backward_pass_dist,
                                 groundline_save_area_extend_width,
                                 valid_passage_length,
                                 velocity_limit,
                                 pitch_limit,
                                 yaw_limit,
                                 space_yam_max_limit);
};

class CarParams {
public:
  ~CarParams();

  static CarParams *GetInstance();

  bool loadFile(const std::string file_name);
  bool loadFile4Plan(const std::string file_name);
  bool loadFileSlotRelease(const std::string config_file_dir);

  bool setLonInflation(double inflation);
  const double lon_inflation() const { return lon_inflation_; }
  const double get_max_steer_angle() const { return max_steer_angle; }
  bool setLatInflation(double inflation);
  const double lat_inflation() const { return lat_inflation_; }
  bool shrinkLatInflation(double inflation) {
    return setLatInflation(std::min(inflation, lat_inflation_));
  }
  bool shrinkLonInflation(double inflation) {
    return setLonInflation(std::min(inflation, lon_inflation_));
  }
  void setMaxSteer(double max_steer);
  void setMaxSteerRate(double max_steer_rate);
  void setMaxSteerRear(double max_steer_rear);
  void setMaxSteerRateRear(double max_steer_rate_rear);

  // TODO: replace with get() and set() functions
  YAML::Node yaml_node_;

  double vehicle_length_real;
  double vehicle_width_real;
  double vehicle_front_edge_to_rear;
  double vehicle_back_edge_to_center;
  double vehicle_width_wo_rearview_mirror;
  double vehicle_length;
  double vehicle_width;
  double vehicle_height;
  double max_acceleration;
  double min_acceleration;
  double max_steer_angle;
  double max_steer_angle_rate;
  double max_steer_angle_rear;
  double max_steer_angle_rate_rear;
  double steer_ratio;
  double wheel_base;
  double wheel_rolling_radius;
  double max_abs_speed_when_stopped;
  double brake_deadzone;
  double throttle_deadzone;
  double lon_inflation_min;
  double lon_inflation_max;
  double lat_inflation_min;
  double lat_inflation_max;
  double inflation_rearview_mirror;
  double shrink_ratio_for_lines_;
  double shrink_ratio_for_lines_min_;
  bool enable_multiple_steer_modes;

  // derivated params
  double max_delta_angle;
  double max_delta_angle_rate;
  double max_delta_angle_rear;
  double max_delta_angle_rate_rear;
  double min_turn_radius;

  bool is_loaded;
  SlotReleaseParams slot_release_config_;

  NLOHMANN_DEFINE_TYPE_INTRUSIVE(
      CarParams, vehicle_length_real, vehicle_width_real,vehicle_front_edge_to_rear,
      vehicle_back_edge_to_center, vehicle_width_wo_rearview_mirror, vehicle_length, vehicle_width,
      vehicle_height, max_acceleration, min_acceleration, max_steer_angle,
      max_steer_angle_rate, max_steer_angle_rear, max_steer_angle_rate_rear,
      steer_ratio, wheel_base, wheel_rolling_radius, max_abs_speed_when_stopped,
      brake_deadzone, throttle_deadzone, lon_inflation_min, lon_inflation_max,
      lat_inflation_min, lat_inflation_max, inflation_rearview_mirror,
      shrink_ratio_for_lines_, shrink_ratio_for_lines_min_,
      enable_multiple_steer_modes,slot_release_config_,

      max_delta_angle, max_delta_angle_rate, max_delta_angle_rear,
      max_delta_angle_rate_rear, min_turn_radius, lon_inflation_,
      lat_inflation_)

private:
  CarParams();
  void updateDerived();
  static CarParams *instance_;
  double lon_inflation_;
  double lat_inflation_;
};

} // namespace worldmodel_pec
