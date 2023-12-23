#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "common/config/vehicle_param.h"
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>

namespace msquare {
HybridAstarConfig::HybridAstarConfig() : is_loaded(false) {}
HybridAstarConfig::~HybridAstarConfig() {}
HybridAstarConfig *HybridAstarConfig::GetInstance() {
  if (instance_ == nullptr) {
    instance_ = new HybridAstarConfig;
  }
  return instance_;
}

bool HybridAstarConfig::loadFile(const std::string file_name) {
  try {
    yaml_node_.reset(YAML::LoadFile(file_name));
    use_t_line_ = yaml_node_["HYBRID_ASTAR_PARAMS"]["use_t_line"].as<bool>();
    inflation_for_points_ =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["inflation_for_points"].as<double>();
    max_zigzag_allowd =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["max_zigzag_allowd"].as<int>();
    xy_grid_resolution =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["xy_grid_resolution"].as<double>();
    phi_grid_resolution =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["phi_grid_resolution"].as<double>();
    grid_dijkstra_xy_resolution =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["grid_dijkstra_xy_resolution"]
            .as<double>();
    pixel_resolution =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["pixel_resolution"].as<double>();

    // penalty
    traj_forward_penalty =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["traj_forward_penalty"].as<double>();
    traj_back_penalty =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["traj_back_penalty"].as<double>();
    traj_gear_switch_penalty =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["traj_gear_switch_penalty"]
            .as<double>();
    traj_steer_penalty =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["traj_steer_penalty"].as<double>();
    traj_steer_change_penalty =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["traj_steer_change_penalty"]
            .as<double>();
    traj_sides_diff_penalty =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["traj_sides_diff_penalty"]
            .as<double>();

    // state update
    step_size = yaml_node_["HYBRID_ASTAR_PARAMS"]["step_size"].as<double>();
    next_node_num =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["next_node_num"].as<int>();
    step_direction =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["step_direction"].as<int>();
    delta_t = yaml_node_["HYBRID_ASTAR_PARAMS"]["delta_t"].as<double>();
    holonomic_with_obs_heuristic =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["holonomic_with_obs_heuristic"]
            .as<double>();
    non_holonomic_without_obs_heuristic =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["non_holonomic_without_obs_heuristic"]
            .as<double>();
    enable_delta_cost =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["enable_delta_cost"].as<bool>();
    enable_analytic_expansion =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["enable_analytic_expansion"]
            .as<bool>();
    max_analytic_expansion_length =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["max_analytic_expansion_length"]
            .as<double>();
    analytic_expansion_end_size_threshold =
        yaml_node_["HYBRID_ASTAR_PARAMS"]
                  ["analytic_expansion_end_size_threshold"]
                      .as<double>();
    force_analytic_expansion_end_direction =
        yaml_node_["HYBRID_ASTAR_PARAMS"]
                  ["force_analytic_expansion_end_direction"]
                      .as<int>();

    // visualize
    verbose = yaml_node_["HYBRID_ASTAR_PARAMS"]["verbose"].as<int>();
    display_points =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["display_points"].as<int>();

    // algorithm
    planning_core =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["planning_core"].as<int>();

    max_iter = max_iter_base =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["max_iter_base"].as<unsigned>();
    max_iter_max =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["max_iter_max"].as<unsigned>();

    footprint_model_ =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["footprint_model"].as<int>();
    footprint_model_precise_ =
        yaml_node_["HYBRID_ASTAR_PARAMS"]["footprint_model_precise"].as<int>();

    is_loaded = true;
  } catch (const std::exception &e) {
    // std::cerr << "[Error]HybridAstarConfig: " << e.what() << '\n';
    // exit(-1);
    return false;
  }
  return true;
}

// Init static member before use
HybridAstarConfig *HybridAstarConfig::instance_ = nullptr;

TrajectoryOptimizerConfig::TrajectoryOptimizerConfig() : is_loaded(false) {}
TrajectoryOptimizerConfig::~TrajectoryOptimizerConfig() {}
TrajectoryOptimizerConfig *TrajectoryOptimizerConfig::GetInstance() {
  if (instance_ == nullptr) {
    instance_ = new TrajectoryOptimizerConfig;
  }
  return instance_;
}

bool TrajectoryOptimizerConfig::loadFile(const std::string file_name) {
  try {
    yaml_node_.reset(YAML::LoadFile(file_name));
    OBCA_running = yaml_node_["OBCA_PARAMS"]["OBCA_running"].as<bool>();
    param_enable_check_parallel_trajectory =
        yaml_node_["OBCA_PARAMS"]["param_enable_check_parallel_trajectory"]
            .as<bool>();
    param_FLAGS_use_iterative_anchoring_smoother =
        yaml_node_["OBCA_PARAMS"]
                  ["param_FLAGS_use_iterative_anchoring_smoother"]
                      .as<bool>();
    param_FLAGS_use_dual_variable_warm_start =
        yaml_node_["OBCA_PARAMS"]["param_FLAGS_use_dual_variable_warm_start"]
            .as<bool>();
    param_FLAGS_enable_smoother_failsafe =
        yaml_node_["OBCA_PARAMS"]["param_FLAGS_enable_smoother_failsafe"]
            .as<bool>();

    param_is_near_destination_threshold =
        yaml_node_["OBCA_PARAMS"]["param_is_near_destination_threshold"]
            .as<double>();
    param_delta_t = yaml_node_["OBCA_PARAMS"]["param_delta_t"].as<double>();

    param_cost_end_state =
        yaml_node_["OBCA_PARAMS"]["param_cost_end_state"].as<double>(); // 5
    param_cost_xy =
        yaml_node_["OBCA_PARAMS"]["param_cost_xy"].as<double>(); // 1
    param_cost_theta =
        yaml_node_["OBCA_PARAMS"]["param_cost_theta"].as<double>(); // 1
    param_cost_speed =
        yaml_node_["OBCA_PARAMS"]["param_cost_speed"].as<double>(); // 1
    param_cost_steer =
        yaml_node_["OBCA_PARAMS"]["param_cost_steer"].as<double>(); // 1
    param_cost_acc =
        yaml_node_["OBCA_PARAMS"]["param_cost_acc"].as<double>(); // 1
    param_cost_steerrate =
        yaml_node_["OBCA_PARAMS"]["param_cost_steerrate"].as<double>(); // 1
    param_cost_jerk =
        yaml_node_["OBCA_PARAMS"]["param_cost_jerk"].as<double>(); // 2
    param_cost_stitching_steer =
        yaml_node_["OBCA_PARAMS"]["param_cost_stitching_steer"]
            .as<double>(); // 3
    param_cost_first_order_time =
        yaml_node_["OBCA_PARAMS"]["param_cost_first_order_time"]
            .as<double>(); // 1
    param_cost_second_order_time =
        yaml_node_["OBCA_PARAMS"]["param_cost_second_order_time"]
            .as<double>(); // 1
    param_cost_stitching_a =
        yaml_node_["OBCA_PARAMS"]["param_cost_stitching_a"].as<double>(); // 3
    param_min_safe_dist =
        yaml_node_["OBCA_PARAMS"]["param_min_safe_dist"].as<double>(); // 0.05
    param_max_steer_angle =
        yaml_node_["OBCA_PARAMS"]["param_max_steer_angle"].as<double>(); // 0.6
    param_max_speed_forward =
        yaml_node_["OBCA_PARAMS"]["param_max_speed_forward"].as<double>(); // 5
    param_max_speed_reverse =
        yaml_node_["OBCA_PARAMS"]["param_max_speed_reverse"].as<double>(); // 5
    param_max_acc_forward =
        yaml_node_["OBCA_PARAMS"]["param_max_acc_forward"].as<double>(); // 1.5
    param_max_acc_reverse =
        yaml_node_["OBCA_PARAMS"]["param_max_acc_reverse"].as<double>(); // 1.5
    param_min_time_sample_scaling =
        yaml_node_["OBCA_PARAMS"]["param_min_time_sample_scaling"]
            .as<double>(); // 0.5
    param_max_time_sample_scaling =
        yaml_node_["OBCA_PARAMS"]["param_max_time_sample_scaling"]
            .as<double>(); // 0.5
    param_warm_start_weight =
        yaml_node_["OBCA_PARAMS"]["param_warm_start_weight"].as<double>(); // 2
    param_max_steer_rate =
        yaml_node_["OBCA_PARAMS"]["param_max_steer_rate"].as<double>(); // 2
    param_if_use_fix_time =
        yaml_node_["OBCA_PARAMS"]["param_if_use_fix_time"].as<bool>(); // true
    param_constraint_check =
        yaml_node_["OBCA_PARAMS"]["param_constraint_check"].as<bool>(); // false
    param_jacobian_ad =
        yaml_node_["OBCA_PARAMS"]["param_jacobian_ad"].as<bool>(); // false

    param_info_level =
        yaml_node_["OBCA_PARAMS"]["param_info_level"].as<int>();          // 0
    param_max_itr = yaml_node_["OBCA_PARAMS"]["param_max_itr"].as<int>(); // 30
    param_lin_torl =
        yaml_node_["OBCA_PARAMS"]["param_lin_torl"].as<double>(); // 1
    param_mumps_mem =
        yaml_node_["OBCA_PARAMS"]["param_mumps_mem"].as<int>(); // 10000
    param_mu_init =
        yaml_node_["OBCA_PARAMS"]["param_mu_init"].as<double>(); // 1
    param_conv_tol =
        yaml_node_["OBCA_PARAMS"]["param_conv_tol"].as<double>(); // 1e6
    param_constraints_tol =
        yaml_node_["OBCA_PARAMS"]["param_constraints_tol"].as<double>(); // 3
    param_accet_objchange_tol =
        yaml_node_["OBCA_PARAMS"]["param_accet_objchange_tol"]
            .as<double>(); // 5
    param_acceptable_compl_inf_tol =
        yaml_node_["OBCA_PARAMS"]["param_acceptable_compl_inf_tol"]
            .as<double>(); // 1e6
    param_dual_inf_tol =
        yaml_node_["OBCA_PARAMS"]["param_dual_inf_tol"].as<double>(); // 1e7
    param_acceptable_tol =
        yaml_node_["OBCA_PARAMS"]["param_acceptable_tol"].as<double>(); // 1e7
    param_acceptable_iter =
        yaml_node_["OBCA_PARAMS"]["param_acceptable_iter"].as<int>(); // 3
    param_warmstart_acceptable_iter =
        yaml_node_["OBCA_PARAMS"]["param_warmstart_acceptable_iter"]
            .as<int>(); // 5
    param_linear_solver = yaml_node_["OBCA_PARAMS"]["param_linear_solver"]
                              .as<std::string>(); // "ma86"

    is_loaded = true;
  } catch (const std::exception &e) {
    // std::cerr << "[Error]TrajectoryOptimizerConfig: " << e.what() << '\n';
    // exit(-1);
    return false;
  }
  return true;
}

// Init static member before use
TrajectoryOptimizerConfig *TrajectoryOptimizerConfig::instance_ = nullptr;

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
    if(yaml_node_["spare_tire"]){
        spare_tire_node = yaml_node_["spare_tire"];
    }
    if(spare_tire_node["exist_spare_tire"] ){
        exist_spare_tire = spare_tire_node["exist_spare_tire"].as<bool>(false);
    }
    if(spare_tire_node["spare_tire_protrusion_length"]){
        spare_tire_protrusion_length = spare_tire_node["spare_tire_protrusion_length"].as<double>();
    }

    type = yaml_node_["type"].as<std::string>();

    vehicle_length_real = yaml_node_["param"]["vehicle_length"].as<double>();
    if(exist_spare_tire){
        vehicle_length_real += spare_tire_protrusion_length;
    }
    vehicle_width_real =
        yaml_node_["param"]["vehicle_width_with_rearview_mirror"].as<double>();
    vehicle_width_wo_rearview_mirror =
        yaml_node_["param"]["vehicle_width"].as<double>();
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

void CarParams::setLatInflationForce(double inflation, double deta) {
    lat_inflation_ = inflation;
    lat_inflation_min += deta;
    updateDerived();
    is_inflation_param_adjusted = true;
    return;
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
void CarParams::setLonInflationMin(double inflation) {
  lon_inflation_min = std::min(lon_inflation_min, inflation);
}

void CarParams::setLonInflationForce(double inflation, double deta) {
    lon_inflation_ = inflation;
    lon_inflation_min += deta;
    updateDerived();
    is_inflation_param_adjusted = true;
    return;
}

void CarParams::resetInflationAdjustFlag() {
    is_inflation_param_adjusted = false;
    return;
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
  min_turn_radius = 5.9;
}

bool CarParams::loadFile4Car(const std::string config_file_dir) {
  std::string config_file_name = "car_config_devcar.json";
  const auto &car_type = VehicleParam::Instance()->car_type;
  const auto &architecture = VehicleParam::Instance()->architecture;

  if (car_type == "L7") {
    // including: Lacar, Lamce
    config_file_name = "car_config_lacar.json";
  } else if (car_type == "LS7") {
    // tianmashan: Lbcar
    config_file_name = "car_config_lbcar.json";
  } else if (car_type == "LC6") {
    // tianmashan: Lccar
    config_file_name = "car_config_lccar.json";
  } else if (car_type == "SG") {
    // lianhuashan, including: SG
    config_file_name = "car_config_sg.json";
  } else if (car_type == "UXE") {
    // lianhuashan, including: UXE
    config_file_name = "car_config_uxe.json";
  } else if (!std::strncmp(car_type.c_str(), "V71", 3) ||
             !std::strncmp(car_type.c_str(), "C03", 3)) {
    // wlingshan, includeing: V71, C03
    config_file_name = "car_config_wls.json";
  } else if (car_type == "A02") {
    // baiyunshan, includeing: A02
    config_file_name = "car_config_ptcar.json";
  } else {
    // use car_config_devcar.json for default
    config_file_name = "car_config_devcar.json";
  }

  std::ifstream fjson(config_file_dir + "/parking/car_configs/" +
                      config_file_name);
  std::string json_str((std::istreambuf_iterator<char>(fjson)),
                       std::istreambuf_iterator<char>());
  nlohmann::json input_json = nlohmann::json::parse(json_str);
  car_config = input_json;


  return true;
}

// Init static member before use
CarParams *CarParams::instance_ = nullptr;

StrategyParams::StrategyParams() : is_loaded(false) {}
StrategyParams::~StrategyParams() {}
StrategyParams *StrategyParams::GetInstance() {
  if (instance_ == nullptr) {
    instance_ = new StrategyParams;
  }
  return instance_;
}

bool StrategyParams::loadFile(const std::string file_name) {
  try {
    yaml_node_.reset(YAML::LoadFile(file_name));

    reset();

    is_loaded = true;
  } catch (const std::exception &e) {
    // std::cerr << "[Error]StrategyParams: " << e.what() << '\n';
    return false;
  }
  return true;
}

void StrategyParams::reset() {
  default_v_ = yaml_node_["STRATEGY_PARAM"]["default_v"].as<double>();
  default_a_ = yaml_node_["STRATEGY_PARAM"]["default_a"].as<double>();
  default_dt_ = yaml_node_["STRATEGY_PARAM"]["default_dt"].as<double>();
  bcheckendsegment =
      yaml_node_["STRATEGY_PARAM"]["bcheckendsegment"].as<bool>();
  check_endsegment_max_tan_tolerance_ =
      yaml_node_["STRATEGY_PARAM"]["check_endsegment_max_tan_tolerance"]
          .as<double>();
  default_check_endsegent_len_ =
      yaml_node_["STRATEGY_PARAM"]["default_check_endsegent_len"].as<double>();
  enable_endsegment_len_correction_ =
      yaml_node_["STRATEGY_PARAM"]["enable_endsegment_len_correction"]
          .as<bool>();
  time_out_ = 0.001; // TODO: complete
  low_speed_endsegment_len_ =
      yaml_node_["STRATEGY_PARAM"]["low_speed_endsegment_len"].as<double>();
  enable_revserse_search_ =
      yaml_node_["STRATEGY_PARAM"]["enable_revserse_search"].as<bool>();
  enable_smooth_ = yaml_node_["STRATEGY_PARAM"]["enable_smooth"].as<bool>();

  force_terminate = false;
}

void StrategyParams::setForceTerminate(bool f) {
  std::lock_guard<std::mutex> lk(mtx_);
  force_terminate = f;
}

bool StrategyParams::getForceTerminate() {
  std::lock_guard<std::mutex> lk(mtx_);
  return force_terminate;
}

// Init static member before use
StrategyParams *StrategyParams::instance_ = nullptr;
} // namespace msquare
