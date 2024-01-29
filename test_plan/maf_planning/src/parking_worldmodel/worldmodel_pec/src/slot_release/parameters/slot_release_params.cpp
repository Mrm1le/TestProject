#include "slot_release_params.h"
#include "slot_release_vehicle_param.h"
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>

namespace worldmodel_pec {

SlotReleaseParams::SlotReleaseParams()
    : compensate_time(0), max_compensate_dist(0), enable_oblique_slot(false),
      vertical_passage_width(4.6), vertical_passage_width_hys_scope(0.29),
      vertical_slot_width_hys_scope(0.2), parallel_passage_width(3.7),
      parallel_passage_width_hys_scope(0.3), parallel_slot_width_hys_scope(0.2),
      oblique_passage_width(4.1), oblique_passage_width_hys_scope(0.3),
      oblique_slot_width_hys_scope(0.2), slot_extend_depth_ratio(0.667),
      slot_in_extend_depth(1.2), ego_side_obstacel_distance_low(1.2),
      ego_side_obstacel_distance_high(1.4), ego_side_obs_dis_side_slot_low(1.3),
      ego_side_obs_dis_side_slot_high(1.8), vertical_slot_width(2.5),
      min_width_parallel(5.2), min_length_parallel(2.1),
      min_length_parallel_step_bottom(1.9), parkable_area_x(10),
      parkable_area_y(3.5), uss_friendly_area_y_min(0.95),
      uss_friendly_area_y_max(5.5), uss_friendly_area_y_max_parallel(3.35),
      forward_pass_dist(-1.0), backward_pass_dist(0.0),
      groundline_save_area_extend_width(0.4), valid_passage_length(2.5),
      velocity_limit(15.0 / 3.6), pitch_limit(5.0 * M_PI / 180),
      yaw_limit(10.0 * M_PI / 180.0), space_yam_max_limit(15.0 * M_PI / 180.0) {
}

SlotReleaseParams::~SlotReleaseParams() {}

CarParams::CarParams() : is_loaded(false) {}
CarParams::~CarParams() {}
CarParams *CarParams::GetInstance() {
  if (instance_ == nullptr) {
    instance_ = new CarParams;
  }
  return instance_;
}

bool CarParams::loadFile(const std::string file_name) {
  try {
    yaml_node_.reset(YAML::LoadFile(file_name));

    YAML::Node spare_tire_node;
    double spare_tire_protrusion_length = 0.0;
    bool exist_spare_tire = false;
    if (yaml_node_["spare_tire"]) {
      spare_tire_node = yaml_node_["spare_tire"];
    }
    if (spare_tire_node["exist_spare_tire"]) {
      exist_spare_tire = spare_tire_node["exist_spare_tire"].as<bool>(false);
    }
    if (spare_tire_node["spare_tire_protrusion_length"]) {
      spare_tire_protrusion_length =
          spare_tire_node["spare_tire_protrusion_length"].as<double>();
    }

    vehicle_length_real = yaml_node_["param"]["vehicle_length"].as<double>();
    if (exist_spare_tire) {
      vehicle_length_real += spare_tire_protrusion_length;
    }
    vehicle_width_real =
        yaml_node_["param"]["vehicle_width_with_rearview_mirror"].as<double>();
    vehicle_width_wo_rearview_mirror =
        yaml_node_["param"]["vehicle_width"].as<double>();
    vehicle_back_edge_to_center =
        yaml_node_["param"]["distance_from_rear_bumper_to_rear_axle"]
            .as<double>();
    vehicle_front_edge_to_rear =
        vehicle_length_real - vehicle_back_edge_to_center;
    vehicle_height = yaml_node_["param"]["vehicle_height"].as<float>();
    steer_ratio = yaml_node_["param"]["steering_angle_ratio"].as<float>();
    wheel_base = yaml_node_["param"]["wheel_base_distance"].as<float>();
    wheel_rolling_radius =
        yaml_node_["param"]["wheel_radius_front_left"].as<float>();
    max_steer_angle = yaml_node_["param"]["steering_angle_max"].as<float>();
    double steering_angle_normalize_ratio = 0.907297;
    if (yaml_node_["param"]["steering_angle_normalize_ratio"]) {
      steering_angle_normalize_ratio =
          yaml_node_["param"]["steering_angle_normalize_ratio"].as<float>();
    }
    if (steering_angle_normalize_ratio < 0.7 ||
        steering_angle_normalize_ratio > 1.0 + 1e-6) {
      steering_angle_normalize_ratio = 0.907297;
    }

    max_steer_angle *= steering_angle_normalize_ratio; // 460 / 507
    max_abs_speed_when_stopped = 0.01;
    brake_deadzone = 15.5;
    throttle_deadzone = 18.0;
    lat_inflation_ = lon_inflation_ = lat_inflation_max = lon_inflation_max = 0;

    // std::cout<<"max steer angle ratio:"<<max_steer_angle<<std::endl;
    // std::cout<<"steering_angle_normalize_ratio:"<<steering_angle_normalize_ratio<<std::endl;

    updateDerived();
  } catch (const std::exception &e) {
    // std::cerr << "[Error]CarParams: " << e.what() << '\n';
    return false;
  }
  return true;
}

bool CarParams::loadFile4Plan(const std::string file_name) {
  try {
    yaml_node_.reset(YAML::LoadFile(file_name));

    lon_inflation_ = lon_inflation_max =
        yaml_node_["CAR_PARAMS"]["lon_inflation_max"].as<float>();
    lon_inflation_min =
        yaml_node_["CAR_PARAMS"]["lon_inflation_min"].as<float>();
    lat_inflation_ = lat_inflation_max =
        yaml_node_["CAR_PARAMS"]["lat_inflation_max"].as<float>();
    lat_inflation_min =
        yaml_node_["CAR_PARAMS"]["lat_inflation_min"].as<float>();
    inflation_rearview_mirror =
        yaml_node_["CAR_PARAMS"]["inflation_rearview_mirror"].as<float>();
    shrink_ratio_for_lines_ =
        yaml_node_["CAR_PARAMS"]["shrink_ratio_for_lines"].as<double>();
    shrink_ratio_for_lines_min_ =
        yaml_node_["CAR_PARAMS"]["shrink_ratio_for_lines_min"].as<double>();
    max_acceleration = yaml_node_["CAR_PARAMS"]["max_acceleration"].as<float>();
    min_acceleration = yaml_node_["CAR_PARAMS"]["min_acceleration"].as<float>();
    // max_steer_angle =
    // yaml_node_["CAR_PARAMS"]["max_steer_angle"].as<float>();
    max_steer_angle_rate =
        yaml_node_["CAR_PARAMS"]["max_steer_angle_rate"].as<float>();
    max_steer_angle_rear =
        yaml_node_["CAR_PARAMS"]["max_steer_angle_rear"].as<float>();
    max_steer_angle_rate_rear =
        yaml_node_["CAR_PARAMS"]["max_steer_angle_rate_rear"].as<float>();
    enable_multiple_steer_modes =
        yaml_node_["CAR_PARAMS"]["enable_multiple_steer_modes"].as<bool>();

    updateDerived();

    is_loaded = true;
  } catch (const std::exception &e) {
    // std::cerr << "[Error]CarParams: " << e.what() << '\n';
    return false;
  }
  return true;
}

bool CarParams::setLatInflation(double inflation) {
  lat_inflation_ =
      std::min(std::max(inflation, lat_inflation_min), lat_inflation_max);
  updateDerived();
  if (std::fabs(lat_inflation_ - inflation) > 1e-5) {
    return false;
  } else {
    return true;
  }
}

bool CarParams::setLonInflation(double inflation) {
  lon_inflation_ =
      std::min(std::max(inflation, lon_inflation_min), lon_inflation_max);
  updateDerived();
  if (std::fabs(lon_inflation_ - inflation) > 1e-5) {
    return false;
  } else {
    return true;
  }
}

void CarParams::setMaxSteer(double max_steer) {
  max_steer_angle = max_steer;
  updateDerived();
}

void CarParams::setMaxSteerRate(double max_steer_rate) {
  max_steer_angle_rate = max_steer_rate;
  updateDerived();
}
void CarParams::setMaxSteerRear(double max_steer_rear) {
  max_steer_angle_rear = max_steer_rear;
  updateDerived();
}

void CarParams::setMaxSteerRateRear(double max_steer_rate_rear) {
  max_steer_angle_rate_rear = max_steer_rate_rear;
  updateDerived();
}

void CarParams::updateDerived() {
  vehicle_width = vehicle_width_wo_rearview_mirror + 2 * lat_inflation_;
  vehicle_length = vehicle_length_real + 2 * lon_inflation_;

  max_delta_angle = max_steer_angle / steer_ratio;
  max_delta_angle_rate = max_steer_angle_rate / steer_ratio;
  max_delta_angle_rear = max_steer_angle_rear / steer_ratio;
  max_delta_angle_rate_rear = max_steer_angle_rate_rear / steer_ratio;
  min_turn_radius = wheel_base / fmax(tan(max_delta_angle / 180 * M_PI), 1e-6);
}

bool CarParams::loadFileSlotRelease(const std::string config_file_dir) {
  std::string config_file_name = "slot_release_devcar.json";
  const auto &car_type = VehicleParam::Instance()->car_type;
  const auto &architecture = VehicleParam::Instance()->architecture;

  if (car_type == "L7") {
    // including: Lacar, Lamce
    config_file_name = "slot_release_config_lacar.json";
  } else if (car_type == "LS7" || car_type == "LC6") {
    // including Lbcar, Lccar
    config_file_name = "slot_release_config_lbcar.json";
  } else if (car_type == "SG") {
    // lianhuashan, including: SG, UXE
    config_file_name = "slot_release_config_sg.json";
  } else if (car_type == "UXE") {
    // lianhuashan, including: SG, UXE
    config_file_name = "slot_release_config_uxe.json";
  } else if (!std::strncmp(car_type.c_str(), "V71", 3) ||
             !std::strncmp(car_type.c_str(), "C03", 3)) {
    // wlingshan, includeing: V71, C03
    config_file_name = "slot_release_config_wls.json";
  } else if (car_type == "A02") {
    // baiyunshan, includeing: A02
    config_file_name = "slot_release_config_ptcar.json";
  } else {
    // use car_config_devcar.json for default
    config_file_name = "slot_release_config_devcar.json";
  }

  std::ifstream fjson(config_file_dir + "/parking/slot_release_configs/" +
                      config_file_name);
  std::string json_str((std::istreambuf_iterator<char>(fjson)),
                       std::istreambuf_iterator<char>());
  nlohmann::json input_json = nlohmann::json::parse(json_str);
  slot_release_config_ = input_json;
  return true;
}

// Init static member before use
CarParams *CarParams::instance_ = nullptr;

} /* namespace worldmodel_pec */
