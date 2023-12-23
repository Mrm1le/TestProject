#pragma once

#include <iostream>
#include <set>
#include <vector>

#include "planning/common/logging.h"

namespace msquare {
namespace ddp {

using Json = nlohmann::json;

struct Config;

class DdpConfigBuilder {
public:
  explicit DdpConfigBuilder(const Json &json) : json_(json) {}

  template <typename T> T cast() const {
    static_assert(std::is_base_of<Config, T>::value,
                  "cast only works on derived Config");
    T target;
    target.init(json_);
    return target;
  }

private:
  const Json json_;
};

template <typename T> T read_json_key(const Json &json, const char *key) {
  if (json.find(key) != json.end()) {
    return json[key];
  } else {
    return {};
  }
}

template <typename T>
T read_json_key(const Json &json, const char *key, T default_value) {
  if (json.find(key) != json.end()) {
    return json[key];
  } else {
    return default_value;
  }
}

template <typename T>
T read_json_keys(const Json &json, const std::vector<std::string> &keys) {
  if (keys.empty())
    return {};
  Json json_new = json;
  for (int i = 0; i < (int)keys.size() - 1; i++) {
    if (json_new.find(keys[i]) != json_new.end()) {
      json_new = json_new[keys[i]];
    } else
      return {};
  }
  if (json_new.find(keys.back()) != json_new.end())
    return json_new[keys.back()];
  else
    return {};
}

template <typename T>
T read_json_keys(const Json &json, const std::vector<std::string> &keys,
                 T default_value) {
  if (keys.empty())
    return default_value;
  Json json_new = json;
  for (int i = 0; i < (int)keys.size() - 1; i++) {
    if (json_new.find(keys[i]) != json_new.end()) {
      json_new = json_new[keys[i]];
    } else
      return default_value;
  }
  if (json_new.find(keys.back()) != json_new.end())
    return json_new[keys.back()];
  else
    return default_value;
}

template <typename T>
void read_json_vec(const Json &json, const std::string &key,
                   std::vector<T> &vec,
                   const std::vector<T> &default_vec = {}) {
  if (json.find(key) != json.end()) {
    for (size_t i = 0; i < json[key].size(); i++) {
      vec.push_back(json[key][i]);
    }
    return;
  }
  vec = default_vec;
}

struct Config {
  virtual ~Config() = default;
  virtual void init(const Json &json) = 0;
};

/***************************************************************************************/
struct DdpConfig : public Config {
  void init(const Json &json) override {
    enable_raw_ddp = read_json_key<bool>(json, "enable_raw_ddp");
    enable_dagger = read_json_key<bool>(json, "enable_dagger");
    use_ddp_model_in_planning =
        read_json_key<bool>(json, "use_ddp_model_in_planning", false);
    MSD_LOG(INFO, "[DdpConfig::init] b_dagger: %d", enable_dagger);
  }
  bool enable_raw_ddp = false;
  bool enable_dagger = false;
  bool use_ddp_model_in_planning = false;
};

struct TrafficLightDeciderConfig : public DdpConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
  }
};

struct ModelRequestConfig : public DdpConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
    lane_change_request_threshold =
        read_json_key<double>(json, "lane_change_request_threshold");
    lane_change_cancel_threshold =
        read_json_key<double>(json, "lane_change_cancel_threshold");
  }

  double lane_change_request_threshold{0.0};
  double lane_change_cancel_threshold{0.0};
};

struct ObstacleDeciderConfig : public DdpConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
    bound_return_speed = read_json_key<double>(json, "bound_return_speed");
    solid_line_bound_buffer =
        read_json_key<double>(json, "solid_line_bound_buffer");
    dash_line_bound_buffer =
        read_json_key<double>(json, "dash_line_bound_buffer");
    physical_road_bound_buffer =
        read_json_key<double>(json, "physical_road_bound_buffer");
  }

  double bound_return_speed = 0.5;
  double physical_road_bound_buffer = 0.5;
  double solid_line_bound_buffer = 0.5;
  double dash_line_bound_buffer = 0.3;
  std::string decision_traj_lat = "refline";
  std::string decision_traj_lon = "traj";
  double min_gain_vel{1.0}; // 基于速度调整weight的速度区间下限
  double max_map_gain_vel{10.0}; // 基于速度调整weight的速度区间上限
};

struct LongitudinalDeciderV3Config : public DdpConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
    velocity_upper_bound = read_json_key<double>(json, "velocity_upper_bound");
    velocity_upper_bound_scale_rate =
        read_json_key<double>(json, "velocity_upper_bound_scale_rate");
    acceleration_upper_bound =
        read_json_key<double>(json, "acceleration_upper_bound");
  }

  int lon_num_step = 20;
  double delta_time = 0.2;
  double lon_speed_decision_horizon = 100.0;
  double lane_width = 2.4;
  double lane_width_large = 3.4;
  double kDistanceSafe = 2.0;
  double enable_prediction_time = 4.0;
  double velocity_upper_bound = 27.777778;
  double velocity_upper_bound_scale_rate = 1.0;
  double acceleration_upper_bound = 1.0;
};

struct ScenarioDisplayStateConfig : public DdpConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
    ready_remain_time = read_json_keys<int>(
        json, std::vector<std::string>{"display_state", "ready_remain_time"});
    wait_remain_time = read_json_keys<int>(
        json, std::vector<std::string>{"display_state", "wait_remain_time"});
    int_rqt_cnt_trsh = read_json_keys<int>(
        json,
        std::vector<std::string>{"int_request", "request_count_threshold"});
    finish_remain_time = read_json_keys<int>(
        json, std::vector<std::string>{"display_state", "finish_remain_time"});
    enalbe_display_function = read_json_keys<bool>(
        json,
        std::vector<std::string>{"display_state", "enable_display_function"});
    int_vel_limit = read_json_keys<double>(
        json, std::vector<std::string>{"int_request", "int_vel_limit"});
    enable_int_request_function = read_json_keys<bool>(
        json,
        std::vector<std::string>{"int_request", "enable_int_request_function"});
    enable_hnp_function = read_json_key<bool>(json, "enable_hnp_functions");
  }

  int ready_remain_time = 2;
  int wait_remain_time = 100;
  int finish_remain_time = 1;
  double int_vel_limit = 0.0;
  double Kkph2m_s = 3.6;
  int int_rqt_cnt_trsh = 2;
  bool enable_hnp_function = false;
  bool enalbe_display_function = false;
  bool enable_int_request_function = false;
};

struct LongitudinalOptimizerV3Config : public DdpConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
  }

  std::string optimization_type = "refine";
  uint max_iteration_num = 30000;

  double acc_stop = -4.0;
};

struct LateralDeciderConfig : public DdpConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
    lateral_reference_enable_max_time =
        read_json_key<double>(json, "lateral_reference_enable_max_time");
  }

  double lateral_reference_enable_max_time = -1.0;
  double min_init_velocity = 0.1;
  double max_lateral_acc = 3.0;
  double max_delta_rate = 0.4;
};

struct LateralOptimizerConfig : public DdpConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
    lateral_endpoint_reference_l_range =
        read_json_key<double>(json, "lateral_endpoint_reference_l_range");
    lat_optimization_weight_dx =
        read_json_key<double>(json, "lateral_optimization_weight_dx");
    lat_optimization_weight_ddx =
        read_json_key<double>(json, "lateral_optimization_weight_ddx");
    lat_optimization_weight_dddx =
        read_json_key<double>(json, "lateral_optimization_weight_dddx");
    lat_optimization_weight_xref =
        read_json_key<double>(json, "lateral_optimization_weight_xref");
    lat_optimization_weight_slack =
        read_json_key<double>(json, "lateral_optimization_weight_slack");
  }

  int lat_num_step{30};
  uint max_iteration_num{10000};
  double lat_optimization_weight_dx{0.0};
  double lat_optimization_weight_ddx{0.0};
  double lat_optimization_weight_dddx{0.0};
  double lat_optimization_weight_xref{0.0};
  double lat_optimization_weight_slack{0.0};
  double min_gain_vel{1.0};
  double lateral_endpoint_reference_l_range{0.0};
};
using LateralOptimizerV2Config = LateralOptimizerConfig;

struct ResultTrajectoryGeneratorConfig : public DdpConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
  }
  double planning_result_delta_time = 0.025;
  double min_path_length = 10.0;
};

struct DdpObstacleManagerConfig : public DdpConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
  }
  double frenet_obstacle_range_s_min = -30.0;
  double frenet_obstacle_range_s_max = 100.0;
  double frenet_obstacle_range_l_min = -50.0;
  double frenet_obstacle_range_l_max = 50.0;
};

struct DdpMapInfoManagerConfig : public DdpConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
  }
};

struct DdpBlockDetectorConfig : public DdpConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
  }
};

struct DdpTaskPipelineConfig : public DdpConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
  }
};

struct DdpTaskPipelineNormalConfig : public DdpTaskPipelineConfig {
  void init(const Json &json) override {
    DdpTaskPipelineConfig::init(json);
    /* read config from json */
    pipeline_version =
        read_json_key<std::string>(json, "pipeline_version", "v1");
  }

  std::string pipeline_version = "v1";
};

struct DdpEvaluatorConfig : public DdpConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
  }

  double kFailedPenalty = -1;
  double kCollisionPenalty = -1;
  double kSafeTimeRear = 0;
  double kSafeTimeFront = 0;
  double kSafeDistanceSide = 0;
  double kMaxSafeDistanceFront = 10;
  double kMaxSafeDistanceRear = 5;
  double kMarginCoefficient = 0;
  double kCarePredictionTime = 0;
};

struct DdpKeepEvaluatorConfig : public DdpEvaluatorConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
    kSafeTimeRear = 0.5;
    kSafeTimeFront = 0.5;
    kSafeDistanceSide = 0.0;
    kMarginCoefficient = 0.9;
    kCarePredictionTime = 1.0;
  }
};

struct DdpNormalChangeEvaluatorConfig : public DdpEvaluatorConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
    kSafeTimeRear = 0.8;
    kSafeTimeFront = 0.8;
    kSafeDistanceSide = 0.5;
    kMarginCoefficient = 0.9;
    kCarePredictionTime = 3.0;
  }
};

struct DdpUrgentChangeEvaluatorConfig : public DdpEvaluatorConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
    kSafeTimeRear = 0.6;
    kSafeTimeFront = 0.6;
    kSafeDistanceSide = 0.5;
    kMarginCoefficient = 0.9;
    kCarePredictionTime = 2.5;
  }
};

struct DdpModelConfig : public DdpConfig {
  void init(const Json &json) override {
    DdpConfig::init(json);
    /* read config from json */
    torch_use_cuda = read_json_key<bool>(json, "torch_use_cuda", true);
    torch_gpu_index = read_json_key<int>(json, "torch_gpu_index", 0);
    torch_model_cpu_path =
        read_json_key<std::string>(json, "torch_model_cpu_path");
    torch_model_gpu_path =
        read_json_key<std::string>(json, "torch_model_gpu_path");
    replanning_with_navi =
        read_json_key<bool>(json, "replanning_with_navi", false);
    // inferjam model path
    model_path_inferjam =
        read_json_key<std::string>(json, "model_path_inferjam");
    // inferjam model input blob names
    feature_obj_blob_name = read_json_key<std::string>(
        json, "feature_obj_blob_name", "feature_obj");
    mask_obj_blob_name =
        read_json_key<std::string>(json, "mask_obj_blob_name", "mask_obj");
    style_feature_blob_name = read_json_key<std::string>(
        json, "style_feature_blob_name", "style_feature");
    feature_static_blob_name = read_json_key<std::string>(
        json, "feature_static_blob_name", "feature_static");
    mask_static_blob_name = read_json_key<std::string>(
        json, "mask_static_blob_name", "mask_static");
    traffic_light_status_blob_name = read_json_key<std::string>(
        json, "traffic_light_status_blob_name", "traffic_light_status");
    traffic_light_extra_input_blob_name =
        read_json_key<std::string>(json, "traffic_light_extra_input_blob_name",
                                   "traffic_light_extra_input");
    speed_limit_blob_name = read_json_key<std::string>(
        json, "speed_limit_blob_name", "speed_limit");
    rotate_matrix_blob_name = read_json_key<std::string>(
        json, "rotate_matrix_blob_name", "rotate_matrix");
    // inferjam model output blob names
    plan_traj_blob_name =
        read_json_key<std::string>(json, "plan_traj_blob_name", "plan_traj");
    lane_logits_blob_name = read_json_key<std::string>(
        json, "lane_logits_blob_name", "lane_logits");
    pred_traj_blob_name =
        read_json_key<std::string>(json, "pred_traj_blob_name", "pred_traj");
    // inferjam model params
    num_node = read_json_key<int>(json, "num_node", 16);
    num_agent = read_json_key<int>(json, "num_agent", 100);
    num_obj_base_feature = read_json_key<int>(json, "num_obj_base_feature", 11);
    num_obj_one_hot_feature =
        read_json_key<int>(json, "num_obj_one_hot_feature", 20);
    num_map_polyline = read_json_key<int>(json, "num_map_polyline", 80);
    num_map_base_feature = read_json_key<int>(json, "num_map_base_feature", 6);
    num_map_one_hot_feature =
        read_json_key<int>(json, "num_map_one_hot_feature", 20);
    Tp = read_json_key<int>(json, "Tp", 10);
    Mp = read_json_key<int>(json, "Mp", 10);
    Tf = read_json_key<int>(json, "Tf", 20);

    // debug switch
    dump_image = read_json_key<bool>(json, "dump_image", false);
    dump_image_dir = read_json_key<std::string>(json, "dump_image_dir", "");
    dump_tensor = read_json_key<bool>(json, "dump_tensor", false);
    dump_tensor_dir = read_json_key<std::string>(json, "dump_tensor_dir", "");
    load_tensor_from_file =
        read_json_key<bool>(json, "load_tensor_from_file", false);
    tensor_json_dir = read_json_key<std::string>(json, "tensor_json_dir", "");
    dump_mdebug = read_json_key<bool>(json, "dump_mdebug", false);

    dump_bin = read_json_key<bool>(json, "dump_bin", false);
    dump_bin_timestamp =
        read_json_key<std::string>(json, "dump_bin_timestamp", "");
    dump_bin_input_dir =
        read_json_key<std::string>(json, "dump_bin_input_dir", "");
    dump_bin_output_dir =
        read_json_key<std::string>(json, "dump_bin_output_dir", "");
    save_model_input_mode =
        read_json_key<std::string>(json, "save_model_input_mode", "");
  }

  // 模型配置
  std::string torch_model_cpu_path; // torch cpu模型配置相关路径
  std::string torch_model_gpu_path; // torch gpu模型配置相关路径
  // inferjam model path
  std::string model_path_inferjam;
  // model input blob names
  std::string feature_obj_blob_name;
  std::string mask_obj_blob_name;
  std::string style_feature_blob_name;
  std::string feature_static_blob_name;
  std::string mask_static_blob_name;
  std::string traffic_light_status_blob_name;
  std::string traffic_light_extra_input_blob_name;
  std::string speed_limit_blob_name;
  std::string rotate_matrix_blob_name;
  // model output blob names
  std::string plan_traj_blob_name;
  std::string lane_logits_blob_name;
  std::string pred_traj_blob_name;

  // 模型结构相关参数
  int num_node = 16;
  int num_agent = 100;
  int num_obj_base_feature = 11;
  int num_obj_one_hot_feature = 20;
  int num_map_polyline = 80;
  int num_map_base_feature = 6;
  int num_map_one_hot_feature = 20;
  int Tp = 10;
  int Mp = 10; // 输出轨迹条数
  int Tf = 20; // 输出轨迹点数
  int time_step_interval = 2;
  int object_least_past_step = 1;
  double map_search_range = 200;
  bool replanning_with_navi = false; // 模型的当前位置是否根据navi信息做重规划

  // 模型inference使用的计算资源
  bool torch_use_cuda = true; // inference时是否使用gpu
  int torch_gpu_index = 0;    // inference使用的gpu的id号

  // 模型debug相关配置
  bool dump_image = false;    // 是否将输入tensor dump成图像
  std::string dump_image_dir; // dump图像的路径
  int map_shape = 256;        // dump图像的大小
  double mpg = 0.5; // dump图像时每个像素对应的实际物理距离
  bool dump_tensor = false;           // 是否将tensor dump出来
  std::string dump_tensor_dir;        // dump tensor的保存路径
  bool load_tensor_from_file = false; // 模型的tensor是否从文件读取
  std::string tensor_json_dir; // 从文件load tensor时，tensor对应json文件的路径
  bool dump_mdebug = false; // 是否将输入tensor dump成mdebug消息

  bool dump_bin = false; // 是否将inferjam推理时的输入和输出存储成bin文件
  std::string dump_bin_timestamp;    // dump inferjam输入和输出的时间戳
  std::string dump_bin_input_dir;    // dump inferjam输入的存储路径
  std::string dump_bin_output_dir;   // dump inferjam输出的存储路径
  std::string save_model_input_mode; // 保存模型输入的方式 simplify vs
                                     // completely vs both
};

} // namespace ddp
} // namespace msquare
