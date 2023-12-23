#pragma once

#include "common/config/refline_generator_config.h"
#include "common/config/scenario_facade_config.h"
#include "common/config/vehicle_param.h"
#include "common/utils/macro.h"
#include "mjson/mjson.hpp"
#include "nlohmann/json.hpp"
#include "planning/common/common.h"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <iterator>
#include <unistd.h>
#include <unordered_map>

namespace msquare {

using ScenarioConfigMap =
    std::unordered_map<ScenarioFacadeConfig::ScenarioFacadeType,
                       ScenarioFacadeConfig, std::hash<int>>;
const std::unordered_map<std::string, TaskConfig::TaskType> string_to_tasktype =
    {{"lateral_behavior_planner",
      TaskConfig::TaskType::LATERAL_BEHAVIOR_PLANNER},
     {"path_decider", TaskConfig::TaskType::PATH_DECIDER},
     {"speed_boundary_decider", TaskConfig::TaskType::SPEED_BOUNDARY_DECIDER},
     {"longitudinal_behavior_planner",
      TaskConfig::TaskType::LONGITUDINAL_BEHAVIOR_PLANNER},
     {"longitudinal_motion_planner",
      TaskConfig::TaskType::LONGITUDINAL_MOTION_PLANNER},
     {"lateral_motion_planner", TaskConfig::TaskType::LATERAL_MOTION_PLANNER},
     {"backup_path_decider", TaskConfig::TaskType::BACKUP_PATH_DECIDER},
     {"lane_change_decider", TaskConfig::TaskType::LANE_CHANGE_DECIDER},
     {"obstacle_decider", TaskConfig::TaskType::OBSTACLE_DECIDER},
     {"general_motion_planner", TaskConfig::TaskType::GENERAL_MOTION_PLANNER}};

struct SyntheticConfiguration {
  std::string scene_type;
  std::string sensor_configuration;
  std::string cpu_configuration;
  bool enable_interactive_lane_change{false};
};

enum DrivingStyle {
  DRIVING_STYLE_AGGRESSIVE = 0,
  DRIVING_STYLE_NORMAL = 1,
  DRIVING_STYLE_CONSERVATIVE = 2,
};

enum LaneChangingStyle {
  LANECHANGING_STYLE_AGGRESSIVE = 0,
  LANECHANGING_STYLE_NORMAL = 1,
  LANECHANGING_STYLE_CONSERVATIVE = 2,
};

struct DrivingStyleConfig {
  bool is_config_load{false};
  DrivingStyle driving_style{};
  std::string config_file_dir{};
  mjson::Reader reader{};
};

struct CommonConfiguration {
  bool enable_json{true};
  bool cp_enable_json{false};
  bool use_eftp{false};
  bool lateral_use_eftp{false};
  bool b_dagger_longitudinal{false};
  bool b_dagger_lateral{false};
  bool disable_refline_smoother{false};
};

struct LateralBehaviorPlannerConfiguration {
  bool enable_new_gap_logic{false};
  bool enable_alc{false};
  bool enable_recommend_alc{false};
  double max_v_at_lc{33.612};
  double st_speed_limit{34.72};
  double angle_lc_limit{15.0};
  double time_lc_wait_limit{10.0};
  double lc_total_time_offset{0.0};
  double lc_total_time_offset_1{0.0};
  double lc_total_time_offset_2{0.0};
  double lc_total_time_offset_3{0.0};
  double int_lc_ready_time{0.2};
  double int_lc_ready_time_3{0.0};
};

struct SpeedBoundaryDeciderConfiguration {
  std::vector<double> curv_limit_lat_acc_velocity_bp{};
  std::vector<double> curv_limit_lat_acc_value{};
  std::vector<double> curv_limit_lat_acc_value_1{};
  std::vector<double> curv_limit_lat_acc_value_2{};
  std::vector<double> curv_limit_lat_acc_value_3{};
};

struct LongitudinalMotionPlannerConfiguration {
  bool enable_new_speed_planner{true};
  bool use_prediction{false};

  bool acc_overtake_assist_switch{false};

  bool enable_modify_stop_point{true};
  bool enable_dynamic_stop_offset{false};
  double kick_start_scale{1.0};
  double acc_decel_region_scale{10.0};
  double velocity_threshold_use_const_jerk_predict_traj{5.0};

  double min_headway{0.5};
  double headway_adjust_rate{8.0};
  double lane_change_headway_rate{2.0};

  double stop_target_accel{0.5};
  double stop_target_accel_entry_velocity{1.0};
  double stop_target_accel_entry_distance{-1.5};
  double stop_ref_jerk{1.0};

  double couple_stop_offset{4.5};
  double transport_trunk_stop_offset{4.5};
  double vru_stop_offset{4.5};
  double eftp_stop_offset{3.5};
  double modify_stop_point_accel_thr{1.0};
  bool enable_ACC_comfot_cost{true};

  double ego_box_expansion_length{0.3};
  double ego_box_expansion_width{0.3};
  double model_traj_modify_time_s{2.0};

  bool is_reduce_stop_offset{false};

  std::vector<double> a_min_value_pt{-5.0, -5.0};
  std::vector<double> j_min_value_pt{-5.0, -5.0};

  double cone_bucket_warning_speed_thr_1{22.23};
  double cone_bucket_warning_speed_thr_2{16.67};
  double cone_bucket_warning_ttc_thr_1{3.0};
  double cone_bucket_warning_ttc_thr_2{2.0};
  double cone_bucket_warning_ttc_thr_3{1.6};
  bool enable_env_speed_limit{false};
  bool use_lidar_rb{false};
  bool enable_cutin_threshold_adjust{false};
  double lon_cutin_threshold = 0.2;
  bool enable_acc_takeover{true};
  double acc_takeover_ttc_thr{1.0};
  double cp_acc_takeover_ttc_thr{1.0};
  double acc_takeover_dec_thr{-3.0};
  double acc_takeover_min_cooldown_time{5.0};
  double acc_takeover_max_run_time{3.0};
  int cipv_lost_prohibit_acc_counter_thr = 1;
  int cipv_lost_warning_counter_thr = 2;
  int cipv_lost_quit_cp_counter_thr = 3;
  double cipv_lost_prohibit_acc_duration = 0.5;
  double cipv_lost_min_cooldown_time = 1.0;
  double cipv_lost_quit_cp_delay_time = 3.0;
};

struct LateralMotionPlannerConfiguration {
  bool lat_safety_improved{false};
  double max_curv_rate_ratio{5.0};
  double min_ref_jerk{0.5};
  double min_ref_jerk_velocity{5.5};
  double ref_jerk_auto_start_ratio{0.5};
  double ref_jerk_auto_start_time_limit{2.0};
};

struct PlannerConfig {
  CommonConfiguration common_config;
  LateralBehaviorPlannerConfiguration lateral_behavior_planner_config;
  SpeedBoundaryDeciderConfiguration speed_boundary_decider_config;
  LongitudinalMotionPlannerConfiguration longitudinal_motion_planner_config;
  LateralMotionPlannerConfiguration lateral_motion_planner_config;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CommonConfiguration, enable_json,
                                   cp_enable_json, use_eftp, lateral_use_eftp,
                                   b_dagger_longitudinal, b_dagger_lateral,
                                   disable_refline_smoother)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    LateralBehaviorPlannerConfiguration, enable_new_gap_logic, enable_alc,
    enable_recommend_alc, max_v_at_lc, st_speed_limit, angle_lc_limit,
    time_lc_wait_limit, lc_total_time_offset, lc_total_time_offset_1,
    lc_total_time_offset_2, lc_total_time_offset_3, int_lc_ready_time,
    int_lc_ready_time_3)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SpeedBoundaryDeciderConfiguration,
                                   curv_limit_lat_acc_velocity_bp,
                                   curv_limit_lat_acc_value,
                                   curv_limit_lat_acc_value_1,
                                   curv_limit_lat_acc_value_2,
                                   curv_limit_lat_acc_value_3)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    LongitudinalMotionPlannerConfiguration, enable_new_speed_planner,
    use_prediction, acc_overtake_assist_switch, enable_modify_stop_point,
    enable_dynamic_stop_offset, kick_start_scale, acc_decel_region_scale,
    velocity_threshold_use_const_jerk_predict_traj, min_headway,
    headway_adjust_rate, lane_change_headway_rate, stop_target_accel,
    stop_target_accel_entry_velocity, stop_target_accel_entry_distance,
    stop_ref_jerk, couple_stop_offset, transport_trunk_stop_offset,
    vru_stop_offset, eftp_stop_offset, modify_stop_point_accel_thr,
    enable_ACC_comfot_cost, ego_box_expansion_length, ego_box_expansion_width,
    model_traj_modify_time_s, is_reduce_stop_offset, a_min_value_pt,
    j_min_value_pt, cone_bucket_warning_speed_thr_1,
    cone_bucket_warning_speed_thr_2, cone_bucket_warning_ttc_thr_1,
    cone_bucket_warning_ttc_thr_2, cone_bucket_warning_ttc_thr_3,
    enable_env_speed_limit, use_lidar_rb, enable_cutin_threshold_adjust,
    lon_cutin_threshold, enable_acc_takeover, acc_takeover_ttc_thr,
    cp_acc_takeover_ttc_thr, acc_takeover_dec_thr,
    acc_takeover_min_cooldown_time, acc_takeover_max_run_time,
    cipv_lost_prohibit_acc_counter_thr, cipv_lost_warning_counter_thr,
    cipv_lost_quit_cp_counter_thr, cipv_lost_prohibit_acc_duration,
    cipv_lost_min_cooldown_time, cipv_lost_quit_cp_delay_time)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LateralMotionPlannerConfiguration,
                                   lat_safety_improved, max_curv_rate_ratio,
                                   min_ref_jerk, min_ref_jerk_velocity,
                                   ref_jerk_auto_start_ratio,
                                   ref_jerk_auto_start_time_limit)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PlannerConfig, common_config,
                                   lateral_behavior_planner_config,
                                   speed_boundary_decider_config,
                                   longitudinal_motion_planner_config,
                                   lateral_motion_planner_config)

class ConfigurationContext {
private:
  // this is a singleton class
  DECLARE_SINGLETON(ConfigurationContext);

public:
  SceneType scene_type() { return scene_type_; }

  SyntheticConfiguration synthetic_config() { return synthetic_conf_; }

  DrivingStyleConfig driving_styleConfig() { return DrivingStyleConfig_; }

  const PlannerConfig &planner_config() { return planner_config_; }

  bool
  reset_synthetic_config(SyntheticConfiguration synthetic_config,
                         std::string config_file_dir,
                         DrivingStyle driving_style = DRIVING_STYLE_NORMAL) {
    if (synthetic_config.scene_type != synthetic_conf_.scene_type ||
        DrivingStyleConfig_.driving_style != driving_style) {
      load_driving_style_config(config_file_dir, driving_style);
      MSD_LOG(INFO, "ConfigContext: reset scene_type %s",
              synthetic_config.scene_type.c_str());
      std::string target_scene_config_dir;
      if (synthetic_config.scene_type == "highway") {
        scene_type_ = SceneType::HIGHWAY;
        target_scene_config_dir =
            config_file_dir + "/scenario_configs_json" + "/highway";
      } else if (synthetic_config.scene_type == "urban") {
        scene_type_ = SceneType::URBAN;
        target_scene_config_dir =
            config_file_dir + "/scenario_configs_json" + "/urban";
      } else {
        scene_type_ = SceneType::HIGHWAY;
        target_scene_config_dir =
            config_file_dir + "/scenario_configs_json" + "/highway";
        MSD_LOG(
            ERROR,
            "ConfigContext: invalid scene type, set to highway for default!");
        // return false;
      }
      default_scenario_facade_configs_.clear();
      default_tasks_config_map_.clear();
      aggressive_tasks_config_map_.clear();
      conservative_tasks_config_map_.clear();
      load_scene_params_from_json(target_scene_config_dir);
      synthetic_conf_ = synthetic_config;
      return true;
    } else {
      synthetic_conf_ = synthetic_config;
      return false;
    }
  }

  void load_vehicle_param() {
    char *calib_dir_env = std::getenv("CALIB_DIR");
    if (calib_dir_env == nullptr) {
      MSD_LOG(INFO, "ConfigContext: env CALIB_DIR not set!");
      return;
    }
    std::string calib_dir(calib_dir_env);
    std::string vehicle_param_path = calib_dir + "/vehicle.yaml";
    MSD_LOG(INFO, "ConfigContext: CALIB_DIR %s yaml dir %s", calib_dir.c_str(),
            vehicle_param_path.c_str());
    if (access(vehicle_param_path.c_str(), F_OK) == -1) {
      MSD_LOG(INFO, "ConfigContext: vehicle.yaml not exist!");
      return;
    }
    YAML::Node yaml_config = YAML::LoadFile(vehicle_param_path);
    // double vehicle_length =
    // yaml_config["param"]["vehicle_length"].as<double>(); MSD_LOG(INFO,
    // "ConfigContext: vehicle length %f", vehicle_length);

    vehicle_param_.length = yaml_config["param"]["vehicle_length"].as<double>();
    vehicle_param_.width = yaml_config["param"]["vehicle_width"].as<double>();
    vehicle_param_.height = yaml_config["param"]["vehicle_height"].as<double>();
    vehicle_param_.from_front_bumper_to_front_axle =
        yaml_config["param"]["distance_from_front_bumper_to_front_axle"]
            .as<double>();
    vehicle_param_.rear_bumper_to_rear_axle =
        yaml_config["param"]["distance_from_rear_bumper_to_rear_axle"]
            .as<double>();
    vehicle_param_.front_edge_to_center =
        yaml_config["param"]["vehicle_length"].as<double>() -
        yaml_config["param"]["distance_from_rear_bumper_to_rear_axle"]
            .as<double>();
    vehicle_param_.back_edge_to_center =
        yaml_config["param"]["distance_from_rear_bumper_to_rear_axle"]
            .as<double>();
    vehicle_param_.left_edge_to_center = vehicle_param_.width / 2.0;
    vehicle_param_.right_edge_to_center = vehicle_param_.width / 2.0;
    vehicle_param_.max_steer_angle =
        yaml_config["param"]["steering_angle_max"].as<double>() * M_PI / 180.0;
    vehicle_param_.wheel_base =
        yaml_config["param"]["wheel_base_distance"].as<double>();
    vehicle_param_.steer_ratio =
        yaml_config["param"]["steering_angle_ratio"].as<double>();
    vehicle_param_.front_wheel_rolling_radius =
        yaml_config["param"]["wheel_radius_front_left"].as<double>();
    vehicle_param_.rear_wheel_rolling_radius =
        yaml_config["param"]["wheel_radius_rear_left"].as<double>();
    vehicle_param_.car_type = yaml_config["type"].as<std::string>();
    vehicle_param_.architecture = yaml_config["architecture"].as<std::string>();
  }

  void load_angle_offset() {
    char *calib_dir_env = std::getenv("CALIB_DIR");
    if (calib_dir_env == nullptr) {
      MSD_LOG(INFO, "ConfigContext: env CALIB_DIR not set!");
      return;
    }
    std::string calib_dir(calib_dir_env);
    std::string params_config_path = calib_dir + "/params_config.yaml";
    MSD_LOG(INFO, "ConfigContext: CALIB_DIR %s yaml dir %s", calib_dir.c_str(),
            params_config_path.c_str());
    if (access(params_config_path.c_str(), F_OK) == -1) {
      MSD_LOG(INFO, "ConfigContext: params_config.yaml not exist!");
      return;
    }
    YAML::Node yaml_config = YAML::LoadFile(params_config_path);

    vehicle_param_.angle_offset =
        yaml_config["ControlsdParams"]["ANGLE_OFFSET_DEFAULT"].as<double>();
  }

  // load all scenarios configuration for target scene
  void
  load_params_from_json(std::string config_file_dir,
                        DrivingStyle driving_style = DRIVING_STYLE_NORMAL) {
    load_vehicle_param();
    load_angle_offset();
    // judge urban or highway scene
    load_driving_style_config(config_file_dir, driving_style);

    load_planner_config(config_file_dir);

    TasksConfigVector tasks_configs_set;
    std::ifstream fjson(config_file_dir + "/scenario_configs_json/scene.json");
    std::string json_str((std::istreambuf_iterator<char>(fjson)),
                         std::istreambuf_iterator<char>());
    mjson::Reader reader(json_str);
    auto scene_type = reader.get<mjson::Json>("scene_type").string_value();
    synthetic_conf_.scene_type = scene_type;
    synthetic_conf_.sensor_configuration =
        reader.get<mjson::Json>("sensor_configuration").string_value();
    synthetic_conf_.cpu_configuration =
        reader.get<mjson::Json>("cpu_configuration").string_value();
    synthetic_conf_.enable_interactive_lane_change =
        reader.get<mjson::Json>("enable_interactive_lane_change").bool_value();
    MSD_LOG(INFO, "ConfigContext: sensor_configuration %s",
            synthetic_conf_.sensor_configuration.c_str());
    std::string target_scene_config_dir;
    MSD_LOG(INFO, "ConfigContext: scene_type %s", scene_type.c_str());
    if (scene_type == "highway") {
      scene_type_ = SceneType::HIGHWAY;
      target_scene_config_dir =
          config_file_dir + "/scenario_configs_json/highway";
    } else if (scene_type == "urban") {
      scene_type_ = SceneType::URBAN;
      target_scene_config_dir =
          config_file_dir + "/scenario_configs_json/urban";
    } else {
      scene_type_ = SceneType::HIGHWAY;
      target_scene_config_dir = config_file_dir + "/highway";
      MSD_LOG(ERROR,
              "ConfigContext: invalid scene type, set to highway for default!");
      // return false;
    }
    load_scene_params_from_json(target_scene_config_dir);
  }

  // load all scenarios configuration
  void load_scene_params_from_json(std::string target_scene_config_dir) {
    std::string default_task_config_path =
        target_scene_config_dir + "/default_tasks.json";
    load_task_config_from_json(default_task_config_path,
                               default_tasks_config_map_);
    std::string aggressive_task_config_path =
        target_scene_config_dir + "/aggressive_tasks.json";
    load_task_config_from_json(aggressive_task_config_path,
                               aggressive_tasks_config_map_);
    std::string conservative_task_config_path =
        target_scene_config_dir + "/conservative_tasks.json";
    load_task_config_from_json(conservative_task_config_path,
                               conservative_tasks_config_map_);

    std::string scenario_config_path =
        target_scene_config_dir + "/lane_follow_scenario.json";
    load_scenario_params_from_json(scenario_config_path);

    scenario_config_path =
        target_scene_config_dir + "/lane_change_preparation_scenario.json";
    load_scenario_params_from_json(scenario_config_path);

    scenario_config_path =
        target_scene_config_dir + "/lane_change_execution_scenario.json";
    load_scenario_params_from_json(scenario_config_path);

    std::string refline_generator_config_path =
        target_scene_config_dir + "/../refline_generator_config.yaml";
    MSD_LOG(INFO, "file_path: %s", refline_generator_config_path.c_str());
    (void)ReflineGeneratorConfig::Instance()->loadFile(
        refline_generator_config_path);
  }

  void set_params(ScenarioConfigMap scenario_facade_configs) {
    default_scenario_facade_configs_ = scenario_facade_configs;
  }

  void
  set_scenario_params(ScenarioFacadeConfig::ScenarioFacadeType scenario_type,
                      ScenarioFacadeConfig scenario_config) {
    if (default_scenario_facade_configs_.find(scenario_type) ==
        default_scenario_facade_configs_.end()) {
      default_scenario_facade_configs_.insert({scenario_type, scenario_config});
    } else {
      default_scenario_facade_configs_[scenario_type] = scenario_config;
    }
  }

  const ScenarioConfigMap &get_params() {
    return default_scenario_facade_configs_;
  }

  const vehicle_param &get_vehicle_param() { return vehicle_param_; }

  const TasksConfigMap &get_default_tasks_params() {
    return default_tasks_config_map_;
  }

  const TasksConfigMap &get_aggressive_tasks_params() {
    return aggressive_tasks_config_map_;
  }

  const TasksConfigMap &get_conservative_tasks_params() {
    return conservative_tasks_config_map_;
  }

  static bool
  rewirte_task_config_in_scenario(ScenarioFacadeConfig &scenario_config,
                                  TaskConfig &task_config) {
    if (scenario_config.tasks_config_map_.find(task_config.task_type()) ==
        scenario_config.tasks_config_map_.end()) {
      MSD_LOG(INFO, "rewrite task[%s] config in scenario[%s] config failed",
              TaskTypeNames[task_config.task_type() - 1].c_str(),
              ScenarioFacadeNames[scenario_config.scenario_facade_type_ - 1]
                  .c_str());
      return false;
    }
    scenario_config.tasks_config_map_[task_config.task_type()] = task_config;
    return true;
  }

  void write_params(std::string path) {}

  void load_task_config_from_json(std::string config_path,
                                  TasksConfigMap &tasks_configs_map) {
    TasksConfigVector tasks_configs_set;
    std::ifstream fjson(config_path);
    std::string json_str((std::istreambuf_iterator<char>(fjson)),
                         std::istreambuf_iterator<char>());
    mjson::Reader reader(json_str);
    auto task_types_params =
        reader.get<mjson::Json>("task_types").array_value();
    for (auto task_type_name : task_types_params) {
      TaskConfig task_config;
      auto task_type = string_to_tasktype.at(task_type_name.string_value());

      update_task_config_from_json(task_type, reader, task_config);

      tasks_configs_map[task_config.task_type()] = task_config;
      tasks_configs_set.push_back(task_config);
    }
  }

  void load_planner_config(const std::string &config_file_dir) {
    std::string config_file_name = "planner_config.json";
    const auto &car_type =
        ConfigurationContext::Instance()->get_vehicle_param().car_type;
    const auto &architecture =
        ConfigurationContext::Instance()->get_vehicle_param().architecture;

    if (car_type == "L7") {
      // including: Lacar, Lamce,
      if (architecture == "L1.0") {
        // Lacar
        config_file_name = "planner_config_lacar.json";
      } else if (architecture == "L2.1") {
        // Lamce
        config_file_name = "planner_config_lamce.json";
      } else {
        // unknown architecture, use Lacar for default
        config_file_name = "planner_config_lacar.json";
      }
    } else if (car_type == "LS7") {
      // including Lbcar
      config_file_name = "planner_config_lbcar.json";
    } else if (car_type == "LC6") {
      // including Lccar
      config_file_name = "planner_config_lccar.json";
    } else if (car_type == "SG" || car_type == "UXE") {
      // lianhuashan, including: SG, UXE
      config_file_name = "planner_config_lhs.json";
    } else {
      // use planner_config.json for default
      config_file_name = "planner_config.json";
    }
    MSD_LOG(ERROR,
            "load_planner_config: car_type: %s, architecture: %s, "
            "config_file_name: %s",
            car_type.c_str(), architecture.c_str(), config_file_name.c_str());

    std::ifstream fjson(config_file_dir +
                        "/scenario_configs_json/planner_config/" +
                        config_file_name);
    std::string json_str((std::istreambuf_iterator<char>(fjson)),
                         std::istreambuf_iterator<char>());
    nlohmann::json input_json = nlohmann::json::parse(json_str);
    planner_config_ = input_json;
  }

private:
  void update_task_config_from_json(TaskConfig::TaskType task_type,
                                    mjson::Reader &reader,
                                    TaskConfig &task_config) {
    switch (task_type) {
    case TaskConfig::TaskType::LATERAL_BEHAVIOR_PLANNER: {
      LateralBehaviorPlannerConfig lateral_behavior_planner_config;
      lateral_behavior_planner_config.enable_heuristic_frenet_search =
          reader.get<mjson::Json>("lateral_behavior_planner_config")
              .object_value()["enable_heuristic_frenet_search"]
              .bool_value();
      lateral_behavior_planner_config.kStartLonComp_nolidar =
          reader.get<mjson::Json>("lateral_behavior_planner_config")
              .object_value()["kStartLonComp_nolidar"]
              .number_value();
      lateral_behavior_planner_config.kLatComp_nolidar =
          reader.get<mjson::Json>("lateral_behavior_planner_config")
              .object_value()["kLatComp_nolidar"]
              .number_value();
      lateral_behavior_planner_config.kPolygonLatError_nolidar =
          reader.get<mjson::Json>("lateral_behavior_planner_config")
              .object_value()["kPolygonLatError_nolidar"]
              .number_value();
      lateral_behavior_planner_config.kLatBufferComp_noLidar =
          reader.get<mjson::Json>("lateral_behavior_planner_config")
              .object_value()["kLatBufferComp_noLidar"]
              .number_value();
      lateral_behavior_planner_config.kLatBufferComp_noLidar_atten =
          reader.get<mjson::Json>("lateral_behavior_planner_config")
              .object_value()["kLatBufferComp_noLidar_atten"]
              .number_value();
      lateral_behavior_planner_config.kLatBufferCompMax_v =
          reader.get<mjson::Json>("lateral_behavior_planner_config")
              .object_value()["kLatBufferCompMax_v"]
              .number_value();
      task_config.set_lateral_behavior_planner_config(
          lateral_behavior_planner_config);
    } break;

    case TaskConfig::TaskType::PATH_DECIDER: {
      PathDeciderConfig path_decider_config;
      task_config.set_path_decider_config(path_decider_config);
    } break;

    case TaskConfig::TaskType::OBSTACLE_DECIDER: {
      ObstacleDeciderConfig obstacle_decider_config;
      task_config.set_obstacle_decider_config(obstacle_decider_config);
    } break;

    case TaskConfig::TaskType::GENERAL_MOTION_PLANNER: {
      GeneralMotionPlannerConfig general_motion_planner_config;
      task_config.set_general_motion_planner_config(
          general_motion_planner_config);
    } break;

    case TaskConfig::TaskType::SPEED_BOUNDARY_DECIDER: {
      SpeedBoundaryDeciderConfig speed_boundary_decider_config;
      speed_boundary_decider_config.enable_heuristic_frenet_search =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["enable_heuristic_frenet_search"]
              .bool_value();
      speed_boundary_decider_config.enable_crosswalk_pedestrian_stop =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["enable_crosswalk_pedestrian_stop"]
              .bool_value();
      speed_boundary_decider_config.lane_width_buff_sensor =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["lane_width_buff_sensor"]
              .number_value();
      speed_boundary_decider_config.max_cruise_speed =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["max_cruise_speed"]
              .number_value();
      speed_boundary_decider_config.max_cruise_acceleration =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["max_cruise_acceleration"]
              .number_value();
      speed_boundary_decider_config.lane_change_prebrake_dis =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["lane_change_prebrake_dis"]
              .number_value();
      speed_boundary_decider_config.lane_change_prebrake_vlim =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["lane_change_prebrake_vlim"]
              .number_value();
      speed_boundary_decider_config.lane_change_prebrake_ratio =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["lane_change_prebrake_ratio"]
              .number_value();
      speed_boundary_decider_config.lane_change_prebrake_sensor_ratio =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["lane_change_prebrake_sensor_ratio"]
              .number_value();
      speed_boundary_decider_config.enable_prebrake_for_close_obs =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["enable_prebrake_for_close_obs"]
              .bool_value();
      speed_boundary_decider_config.enable_prebrake_for_intersection =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["enable_prebrake_for_intersection"]
              .bool_value();
      speed_boundary_decider_config.enable_prebrake_for_curvature =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["enable_prebrake_for_curvature"]
              .bool_value();
      speed_boundary_decider_config.enable_prebrake_for_small_curvature =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["enable_prebrake_for_small_curvature"]
              .bool_value();
      speed_boundary_decider_config.enable_prebrake_for_lane_change =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["enable_prebrake_for_lane_change"]
              .bool_value();
      speed_boundary_decider_config.enable_prebrake_for_ttc =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["enable_prebrake_for_ttc"]
              .bool_value();
      speed_boundary_decider_config.enable_prebrake_for_cutin_score =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["enable_prebrake_for_cutin_score"]
              .bool_value();
      speed_boundary_decider_config.enable_prebrake_for_nbo =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["enable_prebrake_for_nbo"]
              .bool_value();
      speed_boundary_decider_config.enable_prebrake_for_uncertainty =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["enable_prebrake_for_uncertainty"]
              .bool_value();
      speed_boundary_decider_config.enable_prebrake_for_latBP =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["enable_prebrake_for_latBP"]
              .bool_value();
      speed_boundary_decider_config.enable_prebrake_for_POI =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["enable_prebrake_for_POI"]
              .bool_value();
      speed_boundary_decider_config.min_curvature =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["min_curvature"]
              .number_value();
      speed_boundary_decider_config.max_centrifugal_acceleration =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["max_centrifugal_acceleration"]
              .number_value();
      speed_boundary_decider_config.ds_step_curvature =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["ds_step_curvature"]
              .number_value();
      speed_boundary_decider_config.ds_length_curvature =
          reader.get<mjson::Json>("speed_boundary_decider_config")
              .object_value()["ds_length_curvature"]
              .number_value();

      speed_boundary_decider_config.driving_style_params.a_max_ratio_vector =
          DrivingStyleConfig_.reader.get<std::vector<double>>(
              "speed_decider.a_max_ratio");

      speed_boundary_decider_config.driving_style_params.a_max_ratio =
          speed_boundary_decider_config.driving_style_params
              .a_max_ratio_vector[int(DrivingStyleConfig_.driving_style)];

      task_config.set_speed_boundary_decider_config(
          speed_boundary_decider_config);
    } break;

    case TaskConfig::TaskType::LONGITUDINAL_BEHAVIOR_PLANNER: {
      LongitudinalBehaviourPlannerConfig longitudinal_behavior_planner_config;
      longitudinal_behavior_planner_config.time_horizon =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["time_horizon"]
              .number_value();
      longitudinal_behavior_planner_config.max_cruise_speed =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["max_cruise_speed"]
              .number_value();
      longitudinal_behavior_planner_config.kAccCrossMax_GS =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kAccCrossMax_GS"]
              .number_value();
      longitudinal_behavior_planner_config.kAccCrossMin_GS =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kAccCrossMin_GS"]
              .number_value();
      longitudinal_behavior_planner_config.kJerkCrossMax_GS =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kJerkCrossMax_GS"]
              .number_value();
      longitudinal_behavior_planner_config.kJerkCrossMin_GS =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kJerkCrossMin_GS"]
              .number_value();

      longitudinal_behavior_planner_config.kAccCrossMax_TL =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kAccCrossMax_TL"]
              .number_value();
      longitudinal_behavior_planner_config.kAccCrossMin_TL =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kAccCrossMin_TL"]
              .number_value();
      longitudinal_behavior_planner_config.kJerkCrossMax_TL =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kJerkCrossMax_TL"]
              .number_value();
      longitudinal_behavior_planner_config.kJerkCrossMin_TL =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kJerkCrossMin_TL"]
              .number_value();

      longitudinal_behavior_planner_config.kAccCrossMax_TR =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kAccCrossMax_TR"]
              .number_value();
      longitudinal_behavior_planner_config.kAccCrossMin_TR =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kAccCrossMin_TR"]
              .number_value();
      longitudinal_behavior_planner_config.kJerkCrossMax_TR =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kJerkCrossMax_TR"]
              .number_value();
      longitudinal_behavior_planner_config.kJerkCrossMin_TR =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kJerkCrossMin_TR"]
              .number_value();

      longitudinal_behavior_planner_config.kAccCrossMax_UT =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kAccCrossMax_UT"]
              .number_value();
      longitudinal_behavior_planner_config.kAccCrossMin_UT =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kAccCrossMin_UT"]
              .number_value();
      longitudinal_behavior_planner_config.kJerkCrossMax_UT =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kJerkCrossMax_UT"]
              .number_value();
      longitudinal_behavior_planner_config.kJerkCrossMin_UT =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kJerkCrossMin_UT"]
              .number_value();

      longitudinal_behavior_planner_config.kRefDis2Cross =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kRefDis2Cross"]
              .number_value();
      longitudinal_behavior_planner_config.kDeltaDis2Cross =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDeltaDis2Cross"]
              .number_value();

      longitudinal_behavior_planner_config.kW_v =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kW_v"]
              .number_value();
      longitudinal_behavior_planner_config.kW_acc =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kW_acc"]
              .number_value();
      longitudinal_behavior_planner_config.kW_jerk =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kW_jerk"]
              .number_value();
      longitudinal_behavior_planner_config.kW_s =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kW_s"]
              .number_value();
      longitudinal_behavior_planner_config.kW_v_dv =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kW_v_dv"]
              .number_value();
      longitudinal_behavior_planner_config.kW_acc_dv =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kW_acc_dv"]
              .number_value();
      longitudinal_behavior_planner_config.kW_jerk_pow =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kW_jerk_pow"]
              .number_value();
      longitudinal_behavior_planner_config.kW_s_TTC =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kW_s_TTC"]
              .number_value();
      longitudinal_behavior_planner_config.kW_s_ds =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kW_s_ds"]
              .number_value();
      longitudinal_behavior_planner_config.kW_s_pow =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kW_s_pow"]
              .number_value();

      longitudinal_behavior_planner_config.kDecay_vr0 =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_vr0"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_vr1 =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_vr1"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_acc0 =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_acc0"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_acc1 =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_acc1"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_jerk0 =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_jerk0"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_jerk1 =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_jerk1"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_ds0 =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_ds0"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_ds1 =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_ds1"]
              .number_value();

      longitudinal_behavior_planner_config.kDecay_dt_GS =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_dt_GS"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_dt_TR =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_dt_TR"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_dt_TL =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_dt_TL"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_dt_UT =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_dt_UT"]
              .number_value();

      longitudinal_behavior_planner_config.kDecay_dis_base_GS =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_dis_base_GS"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_dis_base_TR =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_dis_base_TR"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_dis_base_TL =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_dis_base_TL"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_dis_base_UT =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_dis_base_UT"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_dis_buff_GS =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_dis_buff_GS"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_dis_buff_TR =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_dis_buff_TR"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_dis_buff_TL =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_dis_buff_TL"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_dis_buff_UT =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_dis_buff_UT"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_dis_ratio =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_dis_ratio"]
              .number_value();

      longitudinal_behavior_planner_config.kDecay_GSvsGS =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_GSvsGS"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_GSvsTR =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_GSvsTR"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_GSvsTL =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_GSvsTL"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_GSvsUT =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_GSvsUT"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_GSvsFM =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_GSvsFM"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_TRvsGS =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_TRvsGS"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_TRvsTR =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_TRvsTR"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_TRvsTL =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_TRvsTL"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_TRvsUT =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_TRvsUT"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_TRvsFM =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_TRvsFM"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_TLvsGS =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_TLvsGS"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_TLvsTR =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_TLvsTR"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_TLvsTL =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_TLvsTL"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_TLvsUT =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_TLvsUT"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_TLvsFM =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_TLvsFM"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_UTvsGS =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_UTvsGS"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_UTvsTR =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_UTvsTR"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_UTvsTL =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_UTvsTL"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_UTvsUT =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_UTvsUT"]
              .number_value();
      longitudinal_behavior_planner_config.kDecay_UTvsFM =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_UTvsFM"]
              .number_value();

      longitudinal_behavior_planner_config.kDecay_IniPos =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDecay_IniPos"]
              .number_value();

      longitudinal_behavior_planner_config.kDsRoad_GS =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDsRoad_GS"]
              .number_value();
      longitudinal_behavior_planner_config.kDsRoad_TL =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDsRoad_TL"]
              .number_value();
      longitudinal_behavior_planner_config.kDsRoad_TR =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDsRoad_TR"]
              .number_value();
      longitudinal_behavior_planner_config.kDsRoad_UT =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDsRoad_UT"]
              .number_value();

      longitudinal_behavior_planner_config.kDsPed =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDsPed"]
              .number_value();
      longitudinal_behavior_planner_config.kDsOfo =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDsOfo"]
              .number_value();
      longitudinal_behavior_planner_config.kDsTruck =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kDsTruck"]
              .number_value();

      longitudinal_behavior_planner_config.kTTC_Lon =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kTTC_Lon"]
              .number_value();
      longitudinal_behavior_planner_config.kTTC_Buff_Sensor =
          reader.get<mjson::Json>("longitudinal_behavior_planner_config")
              .object_value()["kTTC_Buff_Sensor"]
              .number_value();

      longitudinal_behavior_planner_config.driving_style_params.d_stop_vector =
          DrivingStyleConfig_.reader.get<std::vector<double>>(
              "lon_behavior.d_stop");

      longitudinal_behavior_planner_config.driving_style_params.d_stop =
          longitudinal_behavior_planner_config.driving_style_params
              .d_stop_vector[int(DrivingStyleConfig_.driving_style)];

      longitudinal_behavior_planner_config.driving_style_params
          .weight_acc_ratio_vector =
          DrivingStyleConfig_.reader.get<std::vector<double>>(
              "lon_behavior.weight_acc_ratio");

      longitudinal_behavior_planner_config.driving_style_params
          .weight_acc_ratio =
          longitudinal_behavior_planner_config.driving_style_params
              .weight_acc_ratio_vector[int(DrivingStyleConfig_.driving_style)];

      longitudinal_behavior_planner_config.driving_style_params
          .weight_jerk_ratio_vector =
          DrivingStyleConfig_.reader.get<std::vector<double>>(
              "lon_behavior.weight_jerk_ratio");

      longitudinal_behavior_planner_config.driving_style_params
          .weight_jerk_ratio =
          longitudinal_behavior_planner_config.driving_style_params
              .weight_jerk_ratio_vector[int(DrivingStyleConfig_.driving_style)];

      task_config.set_longitudinal_behavior_planner_config(
          longitudinal_behavior_planner_config);
    } break;

    case TaskConfig::TaskType::LONGITUDINAL_MOTION_PLANNER: {
      LongitudinalMotionPlannerConfig longitudinal_motion_planner_config;
      task_config.set_longitudinal_motion_planner_config(
          longitudinal_motion_planner_config);
    } break;

    case TaskConfig::TaskType::LATERAL_MOTION_PLANNER: {
      LateralMotionPlannerConfig lateral_motion_planner_config;
      lateral_motion_planner_config.kStartLonComp_nolidar =
          reader.get<mjson::Json>("lateral_motion_planner_config")
              .object_value()["kStartLonComp_nolidar"]
              .number_value();
      lateral_motion_planner_config.kLatComp_nolidar =
          reader.get<mjson::Json>("lateral_motion_planner_config")
              .object_value()["kLatComp_nolidar"]
              .number_value();
      lateral_motion_planner_config.kPolygonLatError_nolidar =
          reader.get<mjson::Json>("lateral_motion_planner_config")
              .object_value()["kPolygonLatError_nolidar"]
              .number_value();
      lateral_motion_planner_config.enable_offset_smooth =
          reader.get<mjson::Json>("lateral_motion_planner_config")
              .object_value()["enable_offset_smooth"]
              .bool_value();
      lateral_motion_planner_config.kLatBufferCompStatic_Highway =
          reader.get<mjson::Json>("lateral_motion_planner_config")
              .object_value()["kLatBufferCompStatic_Highway"]
              .number_value();
      lateral_motion_planner_config.kMaxStartBuffer_LowSpeed =
          reader.get<mjson::Json>("lateral_motion_planner_config")
              .object_value()["kMaxStartBuffer_LowSpeed"]
              .number_value();
      lateral_motion_planner_config.driving_style_params.l_weight_ratio_vector =
          DrivingStyleConfig_.reader.get<std::vector<double>>(
              "lat_motion.l_weight_ratio");
      lateral_motion_planner_config.driving_style_params.l_weight_ratio =
          lateral_motion_planner_config.driving_style_params
              .l_weight_ratio_vector[int(DrivingStyleConfig_.driving_style)];
      task_config.set_lateral_motion_planner_config(
          lateral_motion_planner_config);
    } break;

    case TaskConfig::TaskType::BACKUP_PATH_DECIDER: {
      BackupPathDeciderConfig backup_path_decider_config;
      task_config.set_backup_path_decider_config(backup_path_decider_config);
    } break;

    case TaskConfig::TaskType::LANE_CHANGE_DECIDER: {
      LaneChangeDeciderConfig lane_change_decider_config;
      lane_change_decider_config.execute_in_lane_change_scenario =
          reader.get<mjson::Json>("lane_change_decider_config")
              .object_value()["execute_in_lane_change_scenario"]
              .bool_value();
      lane_change_decider_config.execute_in_lane_merge_scenario =
          reader.get<mjson::Json>("lane_change_decider_config")
              .object_value()["execute_in_lane_merge_scenario"]
              .bool_value();
      task_config.set_lane_change_decider_config(lane_change_decider_config);
    } break;
    default:
      MSD_LOG(INFO, "ConfigContext: debug invalid task type");
      break;
    }
  }

  // load single scenario configuration from json
  void load_scenario_params_from_json(std::string config_path) {
    std::ifstream fjson(config_path);
    std::string json_str((std::istreambuf_iterator<char>(fjson)),
                         std::istreambuf_iterator<char>());
    mjson::Reader reader(json_str);
    // todo: load params from config files
    auto scenario_type =
        reader.get<mjson::Json>("scenario_type").string_value();
    ScenarioFacadeConfig scenario_config;
    if (scenario_type == "LANE_FOLLOW") {
      FixLaneCruiseScenarioFacadeConfig fix_lane_scenario_config;
      scenario_config.fixlane_cruise_scenario_config_ =
          fix_lane_scenario_config;
      scenario_config.scenario_facade_type_ =
          ScenarioFacadeConfig::ScenarioFacadeType::FixLaneCruise;
    } else if (scenario_type == "LANE_CHANGE_PREPARATION") {
      ChangelanePreparationScenarioFacadeConfig change_lane_preparation_config;
      scenario_config.change_lane_preparation_scenario_config_ =
          change_lane_preparation_config;
      scenario_config.scenario_facade_type_ =
          ScenarioFacadeConfig::ScenarioFacadeType::ChangelanePreparation;
    } else if (scenario_type == "LANE_CHANGE_EXECUTION") {
      ChangeLaneCruiseScenarioFacadeConfig change_lane_cruise_config;
      scenario_config.change_lane_cruise_scenario_config_ =
          change_lane_cruise_config;
      scenario_config.scenario_facade_type_ =
          ScenarioFacadeConfig::ScenarioFacadeType::ChangeLaneCruise;
    }

    TasksConfigVector tasks_configs;
    std::unordered_map<TaskConfig::TaskType, TaskConfig, std::hash<int>>
        tasks_config_map;
    auto task_types_params =
        reader.get<mjson::Json>("task_types").array_value();
    auto task_list_params = reader.get<mjson::Json>("task_list").array_value();
    MSD_LOG(INFO, "ConfigContext[json]: debug tasks size %d %d",
            task_list_params.size(), task_types_params.size());

    for (auto task_type_name : task_types_params) {
      TaskConfig task_config;
      auto task_type = string_to_tasktype.at(task_type_name.string_value());

      update_task_config_from_json(task_type, reader, task_config);

      tasks_config_map[task_config.task_type()] = task_config;
    }
    for (auto task_type_name : task_list_params) {
      auto task_type = string_to_tasktype.at(task_type_name.string_value());
      tasks_configs.push_back(tasks_config_map[task_type]);
    }
    scenario_config.tasks_config_vector_ = tasks_configs;
    scenario_config.tasks_config_map_ = tasks_config_map;
    default_scenario_facade_configs_[scenario_config.scenario_facade_type_] =
        scenario_config;
  }

  void load_driving_style_config(std::string config_file_dir,
                                 DrivingStyle driving_style) {
    DrivingStyleConfig_.config_file_dir = config_file_dir;
    DrivingStyleConfig_.driving_style = driving_style;
    if (!DrivingStyleConfig_.is_config_load) {
      std::ifstream fjson(
          config_file_dir +
          "/scenario_configs_json/driving_style/driving_style.json");
      std::string json_str((std::istreambuf_iterator<char>(fjson)),
                           std::istreambuf_iterator<char>());
      DrivingStyleConfig_.reader = mjson::Reader(json_str);
      DrivingStyleConfig_.is_config_load = true;
    }
  }

private:
  ScenarioConfigMap default_scenario_facade_configs_;
  TasksConfigMap default_tasks_config_map_;
  TasksConfigMap aggressive_tasks_config_map_;
  TasksConfigMap conservative_tasks_config_map_;
  SceneType scene_type_;
  SyntheticConfiguration synthetic_conf_;
  vehicle_param vehicle_param_;
  DrivingStyleConfig DrivingStyleConfig_{};
  PlannerConfig planner_config_{};
};

} // namespace msquare
