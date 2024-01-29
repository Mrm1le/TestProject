#ifdef USE_INFERJAM

#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "data_driven_planner/models/ddp_model.h"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#ifdef ENABLE_OPENCV
#include "opencv2/opencv.hpp"
#endif

#include "common/planning_fail_tracer.h"
#include "data_driven_planner/common/base64.h"
#include "data_driven_planner/common/ddp_context.h"
#include "data_driven_planner/common/ddp_utils.h"
#include "data_driven_planner/common/planning_result_manager.h"
#include "data_driven_planner/models/ego_pose_manager.h"
#include "data_driven_planner/models/fusion_object_manager.h"
#include "data_driven_planner/models/vision_lane_manager.h"
#include "mlog_core/mlog_data_stream.h"
// #include "data_driven_planner/models/hdmap_adapter.h"
// #include "data_driven_planner/models/traffic_light_info_manager.h"

#include "mjson/mjson.hpp"

#define ENSURE(status)                                                         \
  if ((status) != JamStatus::JAM_OK) {                                         \
    MSD_LOG(ERROR, "'%s' failed, pls check\n", #status);                       \
    std::abort();                                                              \
  }

namespace msquare {
namespace ddp {

static inline void save_binary_input(std::string bin_file, const float *data,
                                     int size) {
  // 静态代码检查不准有ofstream，要用的时候自己打开注释吧
  // std::ofstream outfile(bin_file, std::ios::binary);
  // outfile.write(reinterpret_cast<const char *>(data), size*4);
  // outfile.close();
}

static inline std::string read_binary_file(const std::string &file_path) {
  std::fstream stream(file_path, std::ios::in);
  mlog::MLogDataStream ss;
  ss << stream.rdbuf();
  auto result = ss.str();
  return result;
}

template <typename Arr, typename Ptr>
void DdpModel::emplace_back_model_input_completely(const int array_size,
                                                   Arr &arr, const Ptr ptr) {
  for (int i = 0; i < array_size; i++) {
    arr.emplace_back(ptr[i]);
  }
}

template <typename Arr, typename Ptr>
void DdpModel::emplace_back_model_input_simplify(const int num_outside,
                                                 const int num_medium,
                                                 const int num_inner,
                                                 const int num_feature,
                                                 Arr &arr_sim, const Ptr ptr) {
  int N = config_.num_node;
  for (int i = 0; i < num_outside; i++) {
    for (int j = 0; j < num_medium; j++) {
      for (int k = 0; k < num_inner; k++) {
        arr_sim.emplace_back(ptr[i * N * num_feature + j * num_feature + k]);
      }
    }
  }
}

void DdpModel::set_ddp_model_input_info_default() {
  ddp_model_input_info_.num_agent_actual = 0;
  ddp_model_input_info_.num_map_polyline_actual = 0;
  ddp_model_input_info_.fusion_object_info_ts = 0;
  ddp_model_input_info_.vision_lane_ts = 0;
  ddp_model_input_info_.lane_proposals_ts = 0;
  ddp_model_input_info_.fusion_obj_id_ts_info.clear();
}

void DdpModel::get_ddp_model_input(
    double timestamp, const EgoPose *ego_pose_plan_ptr,
    const float *feature_obj_ptr, const float *mask_obj_ptr,
    const float *feature_static_ptr, const float *mask_static_ptr,
    const float *traffic_light_status_ptr, const float *traffic_light_extra_ptr,
    const float *speed_limit_ptr, const float *style_feature_ptr,
    const float *rotate_matrix_ptr, std::string &ddp_model_input) {
  int B = 1;
  int A = config_.num_agent;
  int M = config_.num_map_polyline;
  int N = config_.num_node;
  int num_obj_feature =
      config_.num_obj_base_feature + config_.num_obj_one_hot_feature;
  int num_map_feature =
      config_.num_map_base_feature + config_.num_map_one_hot_feature;

  auto ego_pose_plan_position =
      mjson::Json::object{{"x", mjson::Json(ego_pose_plan_ptr->position.x)},
                          {"y", mjson::Json(ego_pose_plan_ptr->position.y)},
                          {"z", mjson::Json(ego_pose_plan_ptr->position.z)}};
  auto ego_pose_plan_velocity =
      mjson::Json::object{{"x", mjson::Json(ego_pose_plan_ptr->velocity.x)},
                          {"y", mjson::Json(ego_pose_plan_ptr->velocity.y)},
                          {"z", mjson::Json(ego_pose_plan_ptr->velocity.z)}};
  auto ego_pose_plan = mjson::Json::object{
      {"timestamp", mjson::Json(ego_pose_plan_ptr->timestamp)},
      {"v", mjson::Json(ego_pose_plan_ptr->v)},
      {"a", mjson::Json(ego_pose_plan_ptr->a)},
      {"heading_angle", mjson::Json(ego_pose_plan_ptr->heading_angle)},
      {"position", mjson::Json(ego_pose_plan_position)},
      {"velocity", mjson::Json(ego_pose_plan_velocity)}};
  mjson::Json::array speed_limit_arr{};
  emplace_back_model_input_completely(B * 1 * 1 *
                                          config_.num_obj_one_hot_feature,
                                      speed_limit_arr, speed_limit_ptr);
  mjson::Json::object obj_id_ts_ts_arr{};
  for (auto &fusion_obj_id_ts_ts :
       ddp_model_input_info_.fusion_obj_id_ts_info) {
    std::string obj_id = std::to_string(fusion_obj_id_ts_ts.first);
    mjson::Json::object obj_ts_ts_arr{};
    for (auto &fusion_obj_ts_ts : fusion_obj_id_ts_ts.second) {
      std::string ts = std::to_string(fusion_obj_ts_ts.first);
      mjson::Json::array obj_ts_arr{};
      for (auto &fusion_obj_ts : fusion_obj_ts_ts.second) {
        obj_ts_arr.emplace_back(fusion_obj_ts);
      }
      obj_ts_ts_arr[ts] = obj_ts_arr;
    }
    obj_id_ts_ts_arr[obj_id] = obj_ts_ts_arr;
  }
  mjson::Json input_value(mjson::Json::object{
      // necessary info
      {"time_stamp", mjson::Json(timestamp)},
      {"ego_pose_plan", mjson::Json(ego_pose_plan)},
      {"fusion_object_info_ts",
       mjson::Json(ddp_model_input_info_.fusion_object_info_ts)},
      {"vision_lane_ts", mjson::Json(ddp_model_input_info_.vision_lane_ts)},
      {"lane_proposals_ts",
       mjson::Json(ddp_model_input_info_.lane_proposals_ts)},
      {"speed_limit", mjson::Json(speed_limit_arr)},
      {"style_feature", mjson::Json(style_feature_ptr[0])},
      {"fusion_ts_dict", mjson::Json(obj_id_ts_ts_arr)},
      // completely info
      {"feature_obj", mjson::Json(mjson::Json::array{})}, // completely
      {"mask_obj", mjson::Json(mjson::Json::array{})},
      {"feature_static", mjson::Json(mjson::Json::array{})},
      {"mask_static", mjson::Json(mjson::Json::array{})},
      // simplify info
      {"feature_obj_sim", mjson::Json(mjson::Json::array{})}, // simplify
      {"mask_obj_sim", mjson::Json(mjson::Json::array{})},
      {"feature_static_sim", mjson::Json(mjson::Json::array{})},
      {"mask_static_sim", mjson::Json(mjson::Json::array{})},
      {"num_agent_actual", mjson::Json(ddp_model_input_info_.num_agent_actual)},
      {"num_map_polyline_actual",
       mjson::Json(ddp_model_input_info_.num_map_polyline_actual)},
      {"save_mode", mjson::Json(config_.save_model_input_mode)},
      // not save info
      // { "traffic_light_status", mjson::Json( mjson::Json::array{} )},
      // { "traffic_light_extra_input", mjson::Json( mjson::Json::array{} )},
      // both info
      {"rotate_matrix", mjson::Json(mjson::Json::array{})}});

  auto &feature_obj_arr =
      input_value["feature_obj"].array_value(); // completely
  auto &mask_obj_arr = input_value["mask_obj"].array_value();
  auto &feature_static_arr = input_value["feature_static"].array_value();
  auto &mask_static_arr = input_value["mask_static"].array_value();
  auto &feature_obj_arr_sim =
      input_value["feature_obj_sim"].array_value(); // simplify
  auto &mask_obj_arr_sim = input_value["mask_obj_sim"].array_value();
  auto &feature_static_arr_sim =
      input_value["feature_static_sim"].array_value();
  auto &mask_static_arr_sim = input_value["mask_static_sim"].array_value();
  // auto & traffic_light_status_arr =
  // input_value["traffic_light_status"].array_value(); auto &
  // traffic_light_extra_input_arr =
  // input_value["traffic_light_extra_input"].array_value();
  auto &rotate_matrix_arr = input_value["rotate_matrix"].array_value();

  if (config_.save_model_input_mode == "completely" ||
      config_.save_model_input_mode == "both") {
    emplace_back_model_input_completely(B * A * N * num_obj_feature,
                                        feature_obj_arr, feature_obj_ptr);
    emplace_back_model_input_completely(B * A * N * 1, mask_obj_arr,
                                        mask_obj_ptr);
    emplace_back_model_input_completely(B * M * N * num_map_feature,
                                        feature_static_arr, feature_static_ptr);
    emplace_back_model_input_completely(B * M * N * 1, mask_static_arr,
                                        mask_static_ptr);
    emplace_back_model_input_completely(B * 1 * 2 * 2, rotate_matrix_arr,
                                        rotate_matrix_ptr);
  }
  if (config_.save_model_input_mode == "simplify" ||
      config_.save_model_input_mode == "both") {
    // only save valid info
    emplace_back_model_input_simplify(ddp_model_input_info_.num_agent_actual,
                                      config_.Tp, config_.num_obj_base_feature,
                                      num_obj_feature, feature_obj_arr_sim,
                                      feature_obj_ptr);
    emplace_back_model_input_simplify(ddp_model_input_info_.num_agent_actual,
                                      config_.Tp, 1, 1, mask_obj_arr_sim,
                                      mask_obj_ptr);
    emplace_back_model_input_simplify(
        ddp_model_input_info_.num_map_polyline_actual, N,
        config_.num_map_base_feature, num_map_feature, feature_static_arr_sim,
        feature_static_ptr);
    emplace_back_model_input_simplify(
        ddp_model_input_info_.num_map_polyline_actual, N, 1, 1,
        mask_static_arr_sim, mask_static_ptr);
    emplace_back_model_input_completely(B * 1 * 2 * 2, rotate_matrix_arr,
                                        rotate_matrix_ptr);
  }

  ddp_model_input = input_value.dump();
}

void DdpModel::save_model_input(double timestamp, const float *feature_obj_ptr,
                                const float *mask_obj_ptr,
                                const float *feature_static_ptr,
                                const float *mask_static_ptr,
                                const float *traffic_light_status_ptr,
                                const float *traffic_light_extra_ptr,
                                const float *rotate_matrix_ptr) {
  std::string ts_str = std::to_string(timestamp);
  // std::cout << "----- fusion ts = " << ts_str << ", " <<
  // config_.dump_bin_timestamp << ", " << config_.dump_bin_input_dir <<
  // std::endl;
  if (ts_str == config_.dump_bin_timestamp) {
    std::string save_dir = config_.dump_bin_input_dir;
    int B = 1;
    int A = config_.num_agent;
    int M = config_.num_map_polyline;
    int N = config_.num_node;
    int num_obj_feature =
        config_.num_obj_base_feature + config_.num_obj_one_hot_feature;
    int num_map_feature =
        config_.num_map_base_feature + config_.num_map_one_hot_feature;
    save_binary_input(save_dir + "/feature_obj.bin", feature_obj_ptr,
                      B * A * N * num_obj_feature);
    save_binary_input(save_dir + "/mask_obj.bin", mask_obj_ptr, B * A * N * 1);
    save_binary_input(save_dir + "/feature_static.bin", feature_static_ptr,
                      B * M * N * num_map_feature);
    save_binary_input(save_dir + "/mask_static.bin", mask_static_ptr,
                      B * M * N * 1);
    save_binary_input(save_dir + "/traffic_light_status.bin",
                      traffic_light_status_ptr, B * 1 * N * 10);
    save_binary_input(save_dir + "/traffic_light_extra_input.bin",
                      traffic_light_extra_ptr, B * 1 * N * 2);
    save_binary_input(save_dir + "/rotate_matrix.bin", rotate_matrix_ptr,
                      B * 1 * 2 * 2);
  }
}

void DdpModel::save_model_output(double timestamp, const float *lane_logits_ptr,
                                 const float *plan_trajs_ptr,
                                 const float *pred_traj_ptr) {
  std::string ts_str = std::to_string(timestamp);
  if (ts_str == config_.dump_bin_timestamp) {
    std::string save_dir = config_.dump_bin_output_dir;
    save_binary_input(save_dir + "/plan_traj.bin", plan_trajs_ptr,
                      1 * config_.Mp * config_.Tf * 2);
    save_binary_input(save_dir + "/lane_logits.bin", lane_logits_ptr, 1 * 10);
    save_binary_input(save_dir + "/pred_traj.bin", pred_traj_ptr,
                      1 * 99 * config_.Tf * 2);
  }
}

DdpModel::DdpModel(const std::shared_ptr<DdpConfigBuilder> &config_builder) {
  config_ = config_builder->cast<DdpModelConfig>();
  auto env = std::getenv("RealitySimulation");
  if (env != nullptr) {
    if (std::strcmp(env, "simulation") == 0) {
      load_model();
      has_load_model_completed_ = true;
    }
  }
}

std::string DdpModel::get_model_platform() {
  std::string model_platform = "x86_cpu";

#if defined(ENABLE_X86_GPU_PLATFORM)
  model_platform = "onnx";
#endif

#if defined(ENABLE_XAVIER_MODEL)
  model_platform = "xavier";
#elif defined(ENABLE_XAVIER_SAFETY_MODEL)
  model_platform = "xavier_safety";
#endif

#ifdef ENABLE_MDC_810_PLATFORM
  model_platform = "mdc_810";
#elif defined(ENABLE_MDC_610_LHS_PLATFORM)
  model_platform = "mdc_610_lhs";
#elif defined(ENABLE_MDC_610_WLS_PLATFORM)
  model_platform = "mdc_610_wls";
#elif defined(ENABLE_ORIN_PLATFORM)
  model_platform = "orin";
#endif
  // use onnx model for sim and offline
  if (nullptr != std::getenv("RealitySimulation")) {
    if ("simulation" ==
        std::string(std::getenv(     // parasoft-suppress AUTOSAR-A5_3_2-a
            "RealitySimulation"))) { // parasoft-suppress AUTOSAR-A5_3_2-a
      model_platform = "onnx";
    }
  }
  auto env = std::getenv("WHICH_CAR");
  if (env != nullptr) {
    if (std::strcmp(env, "MKZ_SIM") == 0) {
      model_platform = "onnx";
    }
  }
  return model_platform;
}

void DdpModel::load_model() {
#if defined(ENABLE_MDC_610_WLS_PLATFORM)
  auto init_settings = R"({"acl":{"init_hal_buff":false}})";
  (void)infer_jam_init(init_settings);
#else
  (void)infer_jam_init("{}"); // parasoft-suppress AUTOSAR-A8_5_0 "interface"
#endif

  auto network_config = NetworkConfig::make("graph");
  std::string model_root = "";
  if (std::getenv("APA_PLANNING_RESOURCE_PATH") != nullptr) {
    model_root = std::string(std::getenv( // parasoft-suppress AUTOSAR-A5_3_2-a
                     "APA_PLANNING_RESOURCE_PATH")) + // parasoft-suppress
                                                      // AUTOSAR-A5_3_2-a
                 config_.model_path_inferjam;
  }
  MSD_LOG(INFO, "model_root_path: %s", model_root.c_str());
  std::string model_platform = get_model_platform();
  MSD_LOG(INFO, "model path: %s",
          (model_root + "/" + model_platform + "/model.bin").c_str());

  network_config->set("infer_jam.json",
                      read_binary_file(model_root + "/infer_jam.json"));
  network_config->set(
      "ddp/model.json",
      read_binary_file(model_root + "/" + model_platform + "/model.json"));
  network_config->set(
      "ddp/model.bin",
      read_binary_file(model_root + "/" + model_platform + "/model.bin"));

  ENSURE(NetManager::make(network_config, net_manager_));

  std::vector<NetDescriptor> descriptors;
  ENSURE(net_manager_->get_descriptors(descriptors));

  std::vector<infer_jam::DeviceInfo> device_infos;
  infer_jam::DeviceInfo device_info{};
  device_info.device_type = DeviceType::GPU;

#if defined(ENABLE_X86_CPU_PLATFORM)
  device_info.device_type = DeviceType::CPU;
#endif

  // assign cuda_index currently
  device_info.device_id = 0;
  device_infos.push_back(std::move(device_info));
  for (auto descriptor : descriptors) {
    descriptor.device_infos = device_infos;
    (void)net_manager_->update_descriptor(descriptor);
  }

  ENSURE(net_manager_->make_plan(plan_));

  // get input_feature starting address and initialize input_feature memory
  std::shared_ptr<NetBlob> feature_obj_blob{};
  ENSURE(plan_->get_net_blob(config_.feature_obj_blob_name, feature_obj_blob));
  void *feature_obj_data{};
  ENSURE(feature_obj_blob->data(DeviceType::CPU, feature_obj_data));
  memset(feature_obj_data, 0, feature_obj_blob->bytes());

  std::shared_ptr<NetBlob> mask_obj_blob{};
  ENSURE(plan_->get_net_blob(config_.mask_obj_blob_name, mask_obj_blob));
  void *mask_obj_data{};
  ENSURE(mask_obj_blob->data(DeviceType::CPU, mask_obj_data));
  memset(mask_obj_data, 0, mask_obj_blob->bytes());

  std::shared_ptr<NetBlob> style_feature_blob{};
  ENSURE(
      plan_->get_net_blob(config_.style_feature_blob_name, style_feature_blob));
  void *style_feature_data{};
  ENSURE(style_feature_blob->data(DeviceType::CPU, style_feature_data));
  memset(style_feature_data, 0, style_feature_blob->bytes());

  std::shared_ptr<NetBlob> feature_static_blob{};
  ENSURE(plan_->get_net_blob(config_.feature_static_blob_name,
                             feature_static_blob));
  void *feature_static_data{};
  ENSURE(feature_static_blob->data(DeviceType::CPU, feature_static_data));
  memset(feature_static_data, 0, feature_static_blob->bytes());

  std::shared_ptr<NetBlob> mask_static_blob{};
  ENSURE(plan_->get_net_blob(config_.mask_static_blob_name, mask_static_blob));
  void *mask_static_data{};
  ENSURE(mask_static_blob->data(DeviceType::CPU, mask_static_data));
  memset(mask_static_data, 0, mask_static_blob->bytes());

  std::shared_ptr<NetBlob> traffic_light_status_blob{};
  ENSURE(plan_->get_net_blob(config_.traffic_light_status_blob_name,
                             traffic_light_status_blob));
  void *traffic_light_status_data{};
  ENSURE(traffic_light_status_blob->data(DeviceType::CPU,
                                         traffic_light_status_data));
  memset(traffic_light_status_data, 0, traffic_light_status_blob->bytes());

  std::shared_ptr<NetBlob> traffic_light_extra_blob{};
  ENSURE(plan_->get_net_blob(config_.traffic_light_extra_input_blob_name,
                             traffic_light_extra_blob));
  void *traffic_light_extra_data{};
  ENSURE(traffic_light_extra_blob->data(DeviceType::CPU,
                                        traffic_light_extra_data));
  memset(traffic_light_extra_data, 0, traffic_light_extra_blob->bytes());

  std::shared_ptr<NetBlob> speed_limit_blob{};
  ENSURE(plan_->get_net_blob(config_.speed_limit_blob_name, speed_limit_blob));
  void *speed_limit_data{};
  ENSURE(speed_limit_blob->data(DeviceType::CPU, speed_limit_data));
  memset(speed_limit_data, 0, speed_limit_blob->bytes());

  std::shared_ptr<NetBlob> rotate_matrix_blob{};
  ENSURE(
      plan_->get_net_blob(config_.rotate_matrix_blob_name, rotate_matrix_blob));
  void *rotate_matrix_data{};
  ENSURE(rotate_matrix_blob->data(DeviceType::CPU, rotate_matrix_data));
  memset(rotate_matrix_data, 0, rotate_matrix_blob->bytes());

  // inference model
  (void)plan_->run();

  // get inference results
  std::shared_ptr<NetBlob> plan_traj_blob{};
  ENSURE(plan_->get_net_blob(config_.plan_traj_blob_name, plan_traj_blob));

  std::shared_ptr<NetBlob> lane_logits_blob{};
  ENSURE(plan_->get_net_blob(config_.lane_logits_blob_name, lane_logits_blob));

  std::shared_ptr<NetBlob> pred_traj_blob{};
  ENSURE(plan_->get_net_blob(config_.pred_traj_blob_name, pred_traj_blob));
}

bool DdpModel::run_model(double timestamp, DdpTrajectorys &ddp_trajectorys,
                         bool is_replan, std::string &ddp_model_input) {
  const float *plan_traj_ptr;
  const float *lane_logits_ptr;
  const float *pred_traj_ptr;

  // set default value for ddp_model_input_info_
  set_ddp_model_input_info_default();

  EgoPose ego_pose;
  auto ok = get_ego_pose(timestamp, ego_pose, is_replan);
  if (!ok) {
    PLANNING_FAIL_TRACE("failed to get_ego_pose");
    MSD_LOG(ERROR, "run_model: failed to get_ego_pose!");
    return false;
  }

  auto all_time_start = MTIME()->timestamp().ms();
  // get input_feature starting address
  // feature_obj
  std::shared_ptr<NetBlob> feature_obj_blob{};
  ENSURE(plan_->get_net_blob(config_.feature_obj_blob_name, feature_obj_blob));
  void *feature_obj_data{};
  ENSURE(feature_obj_blob->data(DeviceType::CPU, feature_obj_data));
  float *feature_obj_ptr = static_cast<float *>(feature_obj_data);
  // mask_obj
  std::shared_ptr<NetBlob> mask_obj_blob{};
  ENSURE(plan_->get_net_blob(config_.mask_obj_blob_name, mask_obj_blob));
  void *mask_obj_data{};
  ENSURE(mask_obj_blob->data(DeviceType::CPU, mask_obj_data));
  float *mask_obj_ptr = static_cast<float *>(mask_obj_data);
  // style_feature
  std::shared_ptr<NetBlob> style_feature_blob{};
  ENSURE(
      plan_->get_net_blob(config_.style_feature_blob_name, style_feature_blob));
  void *style_feature_data{};
  ENSURE(style_feature_blob->data(DeviceType::CPU, style_feature_data));
  float *style_feature_ptr = static_cast<float *>(style_feature_data);
  // feature_static
  std::shared_ptr<NetBlob> feature_static_blob{};
  ENSURE(plan_->get_net_blob(config_.feature_static_blob_name,
                             feature_static_blob));
  void *feature_static_data{};
  ENSURE(feature_static_blob->data(DeviceType::CPU, feature_static_data));
  float *feature_static_ptr = static_cast<float *>(feature_static_data);
  // mask_static
  std::shared_ptr<NetBlob> mask_static_blob{};
  ENSURE(plan_->get_net_blob(config_.mask_static_blob_name, mask_static_blob));
  void *mask_static_data{};
  ENSURE(mask_static_blob->data(DeviceType::CPU, mask_static_data));
  float *mask_static_ptr = static_cast<float *>(mask_static_data);
  // traffic_light_status
  std::shared_ptr<NetBlob> traffic_light_status_blob{};
  ENSURE(plan_->get_net_blob(config_.traffic_light_status_blob_name,
                             traffic_light_status_blob));
  void *traffic_light_status_data{};
  ENSURE(traffic_light_status_blob->data(DeviceType::CPU,
                                         traffic_light_status_data));
  float *traffic_light_status_ptr =
      static_cast<float *>(traffic_light_status_data);
  // traffic_light_extra
  std::shared_ptr<NetBlob> traffic_light_extra_blob{};
  ENSURE(plan_->get_net_blob(config_.traffic_light_extra_input_blob_name,
                             traffic_light_extra_blob));
  void *traffic_light_extra_data{};
  ENSURE(traffic_light_extra_blob->data(DeviceType::CPU,
                                        traffic_light_extra_data));
  float *traffic_light_extra_ptr =
      static_cast<float *>(traffic_light_extra_data);
  // speed limit
  std::shared_ptr<NetBlob> speed_limit_blob{};
  ENSURE(plan_->get_net_blob(config_.speed_limit_blob_name, speed_limit_blob));
  void *speed_limit_data{};
  ENSURE(speed_limit_blob->data(DeviceType::CPU, speed_limit_data));
  float *speed_limit_ptr = static_cast<float *>(speed_limit_data);
  // rotate_matrix
  std::shared_ptr<NetBlob> rotate_matrix_blob{};
  ENSURE(
      plan_->get_net_blob(config_.rotate_matrix_blob_name, rotate_matrix_blob));
  void *rotate_matrix_data{};
  ENSURE(rotate_matrix_blob->data(DeviceType::CPU, rotate_matrix_data));
  float *rotate_matrix_ptr = static_cast<float *>(rotate_matrix_data);

  // initialize input_feature memory
  memset(feature_obj_ptr, 0, feature_obj_blob->bytes());
  memset(mask_obj_ptr, 0, mask_obj_blob->bytes());
  memset(style_feature_ptr, 0, style_feature_blob->bytes());
  memset(feature_static_ptr, 0, feature_static_blob->bytes());
  memset(mask_static_ptr, 0, mask_static_blob->bytes());
  memset(traffic_light_status_ptr, 0, traffic_light_status_blob->bytes());
  memset(traffic_light_extra_ptr, 0, traffic_light_extra_blob->bytes());
  memset(rotate_matrix_ptr, 0, rotate_matrix_blob->bytes());
  memset(speed_limit_ptr, 0, speed_limit_blob->bytes());

  // fill input_feature
  ddp_trajectorys.clear();
  get_object_input_tensor(timestamp, ego_pose, feature_obj_ptr, mask_obj_ptr,
                          rotate_matrix_ptr);
  get_map_input_tensor(timestamp, ego_pose, feature_static_ptr, mask_static_ptr,
                       ddp_trajectorys);
  get_traffic_light_tensor(ego_pose, traffic_light_status_ptr,
                           traffic_light_extra_ptr);
  int navi_ttc_gear =
      DdpContext::Instance()->ego_pose_manager()->get_navi_ttc_gear();
  MSD_LOG(INFO, "navi_ttc_gear: %d", navi_ttc_gear);
  if (navi_ttc_gear == 0) {
    MSD_LOG(INFO, "navi_ttc_gear_style: 1");
    style_feature_ptr[0] = 1.f;
  } else if (navi_ttc_gear == 1) {
    MSD_LOG(INFO, "navi_ttc_gear_style: 2");
    style_feature_ptr[0] = 1.3f;
  } else if (navi_ttc_gear == 2) {
    MSD_LOG(INFO, "navi_ttc_gear_style: 3");
    style_feature_ptr[0] = 1.8f;
  } else if (navi_ttc_gear == 3) {
    MSD_LOG(INFO, "navi_ttc_gear_style: 4");
    style_feature_ptr[0] = 2.3f;
  } else if (navi_ttc_gear == 4) {
    MSD_LOG(INFO, "navi_ttc_gear_style: 5");
    style_feature_ptr[0] = 3.f;
  }

  float ego_v = std::sqrt(std::pow(ego_pose.velocity.x, 2) +
                          std::pow(ego_pose.velocity.y, 2)) *
                3.6f;
  float cruise_velocity =
      DdpContext::Instance()->ego_pose_manager()->get_ego_cruise_velocity();
  int ego_speed_idx = static_cast<int>(std::round(ego_v / 10.f));
  int speed_idx = static_cast<int>(std::round(cruise_velocity / 10.f));
  if (ego_v < 40.f) {
    speed_idx = std::min(ego_speed_idx + 1, speed_idx);
  }
  speed_idx = std::max(speed_idx, 4);
  speed_limit_ptr[speed_idx] = 1.f;

  auto complete_input_feature_time = MTIME()->timestamp().ms();
  MSD_LOG(ERROR, "[ddp_model_inferjam] prepare input feature time: %f",
          complete_input_feature_time - all_time_start);

  // save model input bin for debug
  MSD_LOG(INFO, "===== timestamp: %s", std::to_string(timestamp).c_str());

  // get ddp_model_input for offline debug
  get_ddp_model_input(
      timestamp, &ego_pose, feature_obj_ptr, mask_obj_ptr, feature_static_ptr,
      mask_static_ptr, traffic_light_status_ptr, traffic_light_extra_ptr,
      speed_limit_ptr, style_feature_ptr, rotate_matrix_ptr, ddp_model_input);

  if (config_.dump_bin) {
    save_model_input(timestamp, feature_obj_ptr, mask_obj_ptr,
                     feature_static_ptr, mask_static_ptr,
                     traffic_light_status_ptr, traffic_light_extra_ptr,
                     rotate_matrix_ptr);
  }

  // inference model
  auto inference_model_time_start = MTIME()->timestamp().ms();
  (void)plan_->run();
  auto inference_model_time_end = MTIME()->timestamp().ms();
  MSD_LOG(ERROR, "[ddp_model_inferjam] inference model time: %f",
          inference_model_time_end - inference_model_time_start);

  // get inference results
  // plan_traj
  std::shared_ptr<NetBlob> plan_traj_blob{};
  ENSURE(plan_->get_net_blob(config_.plan_traj_blob_name, plan_traj_blob));
  void *plan_traj_data{};
  ENSURE(plan_traj_blob->data(DeviceType::CPU, plan_traj_data));
  plan_traj_ptr = static_cast<const float *>(plan_traj_data);

  // lane_logits
  std::shared_ptr<NetBlob> lane_logits_blob{};
  ENSURE(plan_->get_net_blob(config_.lane_logits_blob_name, lane_logits_blob));
  void *lane_logits_data{};
  ENSURE(lane_logits_blob->data(DeviceType::CPU, lane_logits_data));
  lane_logits_ptr = static_cast<const float *>(lane_logits_data);

  // pred_traj
  std::shared_ptr<NetBlob> pred_traj_blob{};
  ENSURE(plan_->get_net_blob(config_.pred_traj_blob_name, pred_traj_blob));
  void *pred_traj_data{};
  ENSURE(pred_traj_blob->data(DeviceType::CPU, pred_traj_data));
  pred_traj_ptr = static_cast<const float *>(pred_traj_data);

  // process output
  process_output(ego_pose, lane_logits_ptr, plan_traj_ptr, ddp_trajectorys);

  auto all_time_end = MTIME()->timestamp().ms();
  MSD_LOG(ERROR, "[ddp_model_inferjam] process model output time: %f",
          all_time_end - inference_model_time_end);
  MSD_LOG(ERROR, "[ddp_model_inferjam] inferjam all time cost: %f",
          all_time_end - all_time_start);

  // save model output bin for debug
  if (config_.dump_bin) {
    save_model_output(timestamp, lane_logits_ptr, plan_traj_ptr, pred_traj_ptr);
  }

  if (config_.dump_image) {
    draw_debug_image(timestamp);
  }
  return true;
}

bool DdpModel::run(double timestamp, DdpTrajectorys &ddp_trajectorys,
                   std::string &ddp_model_input, bool is_replan) {
  if (!has_load_model_completed_) {
    load_model();
    has_load_model_completed_ = true;
    PLANNING_FAIL_TRACE("load model");
    return false;
  } else {
    return run_model(timestamp, ddp_trajectorys, is_replan, ddp_model_input);
  }
}

void DdpModel::get_object_input_tensor(double timestamp,
                                       const EgoPose &ego_pose,
                                       float *feature_obj_ptr,
                                       float *mask_obj_ptr,
                                       float *rotate_matrix_ptr) {
  // step 1) config
  int dim_A = config_.num_agent;
  int dim_N = config_.num_node;
  int dim_F = config_.num_obj_base_feature + config_.num_obj_one_hot_feature;
  int time_step_interval = config_.time_step_interval;
  double time_step_delta = 0.1;

  // step 2.1) ego feature
  for (int i = 0; i < config_.Tp; i++) {
    feature_obj_ptr[i * dim_F + static_cast<int>(ObjFeatMeaningType::VX)] =
        ego_pose.velocity.x;
    feature_obj_ptr[i * dim_F + static_cast<int>(ObjFeatMeaningType::VY)] =
        ego_pose.velocity.y;
    feature_obj_ptr[i * dim_F + static_cast<int>(ObjFeatMeaningType::TYPE)] =
        0.f;
    feature_obj_ptr[i * dim_F + static_cast<int>(ObjFeatMeaningType::THETA)] =
        ego_pose.heading_angle;
    // add ego feature type one_hot
    feature_obj_ptr[i * dim_F + config_.num_obj_base_feature] = 1.f;
    mask_obj_ptr[i] = 1.f;
  }
  // step 2.2) set rotate_matrix
  msquare::planning_math::get_rotate_matrix(ego_pose.heading_angle,
                                            rotate_matrix_ptr);

  // step 3) object feature and mask
  // step 3.1) get object info
  std::vector<double> timestamp_list;
  for (int i = 0; i < config_.Tp + 1; i++) {
    timestamp_list.push_back(timestamp -
                             time_step_delta * i * time_step_interval);
  }
  FusionObjectInfo fusion_object_info;
  ddp_model_input_info_.fusion_object_info_ts =
      DdpContext::Instance()
          ->fusion_object_manager()
          ->get_last_fusion_timestamp();
  (void)DdpContext::Instance()
      ->fusion_object_manager()
      ->get_nearest_fusion_object_info(
          ego_pose, dim_A, timestamp_list, &fusion_object_info,
          ddp_model_input_info_.fusion_obj_id_ts_info);

  // step 3.2) dump data
  int cur_object_feature_idx = 1;
  for (auto it = fusion_object_info.begin();
       it != fusion_object_info.end() and cur_object_feature_idx < dim_A;
       ++it, ++cur_object_feature_idx) {
    auto &fusion_object_history = it->second;
    float obj_type = parse_object_type(fusion_object_history.back().type);
    int n = 0;
    for (auto it = fusion_object_history.begin();
         (it + 1) < fusion_object_history.end() and n < config_.Tp; it++) {
      feature_obj_ptr[(cur_object_feature_idx * dim_N + n) * dim_F +
                      static_cast<int>(ObjFeatMeaningType::X_START)] =
          (it + 1)->position.x - ego_pose.position.x;
      feature_obj_ptr[(cur_object_feature_idx * dim_N + n) * dim_F +
                      static_cast<int>(ObjFeatMeaningType::Y_START)] =
          (it + 1)->position.y - ego_pose.position.y;
      feature_obj_ptr[(cur_object_feature_idx * dim_N + n) * dim_F +
                      static_cast<int>(ObjFeatMeaningType::X_END)] =
          it->position.x - ego_pose.position.x;
      feature_obj_ptr[(cur_object_feature_idx * dim_N + n) * dim_F +
                      static_cast<int>(ObjFeatMeaningType::Y_END)] =
          it->position.y - ego_pose.position.y;
      feature_obj_ptr[(cur_object_feature_idx * dim_N + n) * dim_F +
                      static_cast<int>(ObjFeatMeaningType::VX)] =
          it->velocity.x;
      feature_obj_ptr[(cur_object_feature_idx * dim_N + n) * dim_F +
                      static_cast<int>(ObjFeatMeaningType::VY)] =
          it->velocity.y;
      feature_obj_ptr[(cur_object_feature_idx * dim_N + n) * dim_F +
                      static_cast<int>(ObjFeatMeaningType::YAW)] =
          msquare::planning_math::NormalizeAngle(it->heading_angle -
                                                 ego_pose.heading_angle);
      feature_obj_ptr[(cur_object_feature_idx * dim_N + n) * dim_F +
                      static_cast<int>(ObjFeatMeaningType::LENGTH)] =
          std::max(it->shape.length, 1.0);
      feature_obj_ptr[(cur_object_feature_idx * dim_N + n) * dim_F +
                      static_cast<int>(ObjFeatMeaningType::WIDTH)] =
          std::max(it->shape.width, 1.0);
      feature_obj_ptr[(cur_object_feature_idx * dim_N + n) * dim_F +
                      static_cast<int>(ObjFeatMeaningType::TYPE)] = obj_type;
      feature_obj_ptr[(cur_object_feature_idx * dim_N + n) * dim_F +
                      static_cast<int>(ObjFeatMeaningType::THETA)] =
          it->heading_angle;
      // add obj feature type one_hot
      int obj_type_idx = static_cast<int>(obj_type);
      feature_obj_ptr[(cur_object_feature_idx * dim_N + n) * dim_F +
                      config_.num_obj_base_feature + obj_type_idx] = 1.f;
      mask_obj_ptr[cur_object_feature_idx * dim_N + n] = 1.f;
      n++;
    }
  }
  ddp_model_input_info_.num_agent_actual = cur_object_feature_idx;
}

void DdpModel::get_map_input_tensor(double timestamp, const EgoPose &ego_pose,
                                    float *feature_static_ptr,
                                    float *mask_static_ptr,
                                    DdpTrajectorys &ddp_trajectorys) {
  int dim_P = config_.num_map_polyline;
  int dim_N = config_.num_node;
  int dim_F = config_.num_map_base_feature + config_.num_map_one_hot_feature;

  // get lanes
  std::vector<NodeLane> lanes;
  (void)DdpContext::Instance()->vision_lane_manager()->get_vision_lane(
      timestamp, ego_pose.position, lanes,
      ddp_model_input_info_.vision_lane_ts);

  // lane proposal
  std::vector<NodeLane> lane_proposals;
  (void)DdpContext::Instance()->vision_lane_manager()->get_lane_proposal(
      timestamp, ego_pose.position, lane_proposals,
      ddp_model_input_info_.lane_proposals_ts);
  lanes.insert(lanes.begin(), lane_proposals.begin(), lane_proposals.end());

  int cur_lane_feature_idx = 0;
  ddp_trajectorys.clear();
  for (auto it = lanes.begin(); it != lanes.end(); ++it) {
    auto &nodes = it->nodes;

    if (it->id == "proposal") {
      DdpTrajectory ddp_trajectory;
      for (int n = 0; n < dim_N && n < static_cast<int>(nodes.size()); ++n) {
        ddp_trajectory.track_ids.push_back(nodes[n].id);
      }
      ddp_trajectorys.emplace_back(ddp_trajectory);
    }

    for (int n = 0; n < dim_N and n < static_cast<int>(nodes.size()); n++) {
      auto &node = nodes[n];
      feature_static_ptr[(cur_lane_feature_idx * dim_N + n) * dim_F +
                         static_cast<int>(StaticFeatMeaning::X_START)] =
          node.start_point.x - ego_pose.position.x;
      feature_static_ptr[(cur_lane_feature_idx * dim_N + n) * dim_F +
                         static_cast<int>(StaticFeatMeaning::Y_START)] =
          node.start_point.y - ego_pose.position.y;
      feature_static_ptr[(cur_lane_feature_idx * dim_N + n) * dim_F +
                         static_cast<int>(StaticFeatMeaning::X_END)] =
          node.end_point.x - ego_pose.position.x;
      feature_static_ptr[(cur_lane_feature_idx * dim_N + n) * dim_F +
                         static_cast<int>(StaticFeatMeaning::Y_END)] =
          node.end_point.y - ego_pose.position.y;
      feature_static_ptr[(cur_lane_feature_idx * dim_N + n) * dim_F +
                         static_cast<int>(StaticFeatMeaning::TYPE)] =
          (float)node.type;
      feature_static_ptr[(cur_lane_feature_idx * dim_N + n) * dim_F +
                         static_cast<int>(StaticFeatMeaning::ON_ROUTE_DIST)] =
          it->on_route_dis;
      // add static feature type one hot
      int static_type_idx = static_cast<int>(node.type);
      feature_static_ptr[(cur_lane_feature_idx * dim_N + n) * dim_F +
                         config_.num_map_base_feature + static_type_idx] = 1.f;
      mask_static_ptr[cur_lane_feature_idx * dim_N + n] = 1.f;
    }

    cur_lane_feature_idx++;
    if (cur_lane_feature_idx >= dim_P) {
      break;
    }
  }
  ddp_model_input_info_.num_map_polyline_actual = cur_lane_feature_idx;
}

void DdpModel::get_traffic_light_tensor(const EgoPose &ego_pose,
                                        float *traffic_light_status_ptr,
                                        float *traffic_light_extra_ptr) {
  constexpr int num_traffic_light_status = 10;
  auto dim_N = config_.num_node;

  for (int i = 0; i < dim_N; ++i) {
    int traffic_light_idx = static_cast<int>(6);
    traffic_light_status_ptr[i * num_traffic_light_status + traffic_light_idx] =
        1.f;
    traffic_light_extra_ptr[i * 2] = i + 1.f;
    traffic_light_extra_ptr[i * 2 + 1] = i + 1.f;
  }
}

void DdpModel::process_output(const EgoPose &ego_pose,
                              const float *lane_logits_ptr,
                              const float *plan_trajs_ptr,
                              DdpTrajectorys &ddp_trajectorys) {
  double time_step_delta = 0.1;
  for (int i = 0;
       i < config_.Mp && i < static_cast<int>(ddp_trajectorys.size()); i++) {
    auto &ddp_trajectory = ddp_trajectorys[i];
    ddp_trajectory.type = DDP_LANE_CHANGE;
    ddp_trajectory.logit = lane_logits_ptr[i];

    TrajectoryPoint start_pt;
    start_pt.x = ego_pose.position.x;
    start_pt.y = ego_pose.position.y;
    start_pt.t = 0;
    start_pt.v = ego_pose.v;
    start_pt.a = ego_pose.a;
    start_pt.heading_angle = ego_pose.heading_angle;
    ddp_trajectory.trajectory.push_back(start_pt);

    for (int j = 0; j < config_.Tf; j++) {
      TrajectoryPoint pt;
      pt.x = ego_pose.position.x + plan_trajs_ptr[(i * config_.Tf + j) * 2 + 0];
      pt.y = ego_pose.position.y + plan_trajs_ptr[(i * config_.Tf + j) * 2 + 1];
      pt.t = time_step_delta * (j + 1) * config_.time_step_interval;

      auto &last_pt = ddp_trajectory.trajectory.back();
      auto delta =
          msquare::planning_math::Vec2d(pt.x - last_pt.x, pt.y - last_pt.y);
      auto length = delta.Length();
      pt.v = length / (time_step_delta * config_.time_step_interval);
      if (length > 0.1) {
        pt.heading_angle = delta.Angle();
      } else {
        pt.heading_angle = last_pt.heading_angle;
      }
      pt.a =
          (pt.v - last_pt.v) / (time_step_delta * config_.time_step_interval);
      ddp_trajectory.trajectory.push_back(pt);
    }
  }
}

bool DdpModel::get_ego_pose(double timestamp, EgoPose &ego_pose,
                            bool is_replan) {
  EgoPose ego_pose_navi;
  auto ok = DdpContext::Instance()->ego_pose_manager()->get_ego_pose(
      timestamp, ego_pose_navi);
  if (config_.replanning_with_navi || is_replan) {
    if (!ok) {
      return false;
    }
    ego_pose = ego_pose_navi;
  } else {
    EgoPose ego_pose_traj;
    auto traj_ok = DdpContext::Instance()
                       ->planning_result_manager()
                       ->get_planning_ego_pose(timestamp, &ego_pose_traj);
    if (traj_ok) {
      ego_pose = ego_pose_traj;
      ego_pose.position.z = ego_pose_navi.position.z;
    } else {
      if (!ok) {
        return false;
      }
      ego_pose = ego_pose_navi;
    }
  }

  return true;
}

float DdpModel::parse_object_type(const ObjectType type) {
  if (type == ObjectType::COUPE or type == ObjectType::TRANSPORT_TRUCK or
      type == ObjectType::BUS or type == ObjectType::ENGINEER_TRUCK) {
    return (float)VectorType::CAR;
  } else if (type == ObjectType::OFO or type == ObjectType::TRICYCLE) {
    return (float)VectorType::OFO;
  } else if (type == ObjectType::PEDESTRIAN) {
    return (float)VectorType::HUMAN;
  } else if (type == ObjectType::CONE_BUCKET) {
    return (float)VectorType::CONE;
  }

  return (float)VectorType::CAR;
}

void DdpModel::draw_debug_image(double timestamp) {
#ifdef ENABLE_OPENCV
  // get input_feature starting address
  // feature_obj
  std::shared_ptr<NetBlob> feature_obj_blob{};
  ENSURE(plan_->get_net_blob(config_.feature_obj_blob_name, feature_obj_blob));
  void *feature_obj_data{};
  ENSURE(feature_obj_blob->data(DeviceType::CPU, feature_obj_data));
  float *feature_obj_ptr = static_cast<float *>(feature_obj_data);
  // mask_obj
  std::shared_ptr<NetBlob> mask_obj_blob{};
  ENSURE(plan_->get_net_blob(config_.mask_obj_blob_name, mask_obj_blob));
  void *mask_obj_data{};
  ENSURE(mask_obj_blob->data(DeviceType::CPU, mask_obj_data));
  float *mask_obj_ptr = static_cast<float *>(mask_obj_data);
  // feature_static
  std::shared_ptr<NetBlob> feature_static_blob{};
  ENSURE(plan_->get_net_blob(config_.feature_static_blob_name,
                             feature_static_blob));
  void *feature_static_data{};
  ENSURE(feature_static_blob->data(DeviceType::CPU, feature_static_data));
  float *feature_static_ptr = static_cast<float *>(feature_static_data);
  // mask_static
  std::shared_ptr<NetBlob> mask_static_blob{};
  ENSURE(plan_->get_net_blob(config_.mask_static_blob_name, mask_static_blob));
  void *mask_static_data{};
  ENSURE(mask_static_blob->data(DeviceType::CPU, mask_static_data));
  float *mask_static_ptr = static_cast<float *>(mask_static_data);

  // get inference results
  // plan_traj
  std::shared_ptr<NetBlob> plan_traj_blob{};
  ENSURE(plan_->get_net_blob(config_.plan_traj_blob_name, plan_traj_blob));
  void *plan_traj_data{};
  ENSURE(plan_traj_blob->data(DeviceType::CPU, plan_traj_data));
  float *plan_traj_ptr = static_cast<float *>(plan_traj_data);

  // lane_logits
  std::shared_ptr<NetBlob> lane_logits_blob{};
  ENSURE(plan_->get_net_blob(config_.lane_logits_blob_name, lane_logits_blob));
  void *lane_logits_data{};
  ENSURE(lane_logits_blob->data(DeviceType::CPU, lane_logits_data));
  float *lane_logits_ptr = static_cast<float *>(lane_logits_data);

  int mp = config_.Mp;
  int dim_P = config_.num_map_polyline;
  int dim_N = config_.num_node;
  int dim_F = config_.num_map_base_feature + config_.num_map_one_hot_feature;
  int dim_A = config_.num_agent;
  int dim_obj_F =
      config_.num_obj_base_feature + config_.num_obj_one_hot_feature;
  int dim_Tf = config_.Tf;

  int selected_lane_index = -1;
  float max_lane_logit = -1e10;
  for (int lane_index = 0; lane_index < mp; lane_index++) {
    if (mask_static_ptr[lane_index * dim_N] > 0.f &&
        std::fabs(feature_static_ptr[lane_index * dim_N * dim_F + 4] - 8) <
            1e-3) {
      if (lane_logits_ptr[lane_index] > max_lane_logit) {
        max_lane_logit = lane_logits_ptr[lane_index];
        selected_lane_index = lane_index;
      }
    }
  }

  cv::Mat debug_im = cv::Mat(config_.map_shape, config_.map_shape, CV_8UC3,
                             cv::Scalar(0, 0, 0));
  debug_im.convertTo(debug_im, CV_32F, 255.0);
  int ms = config_.map_shape;
  cv::resize(debug_im, debug_im, cv::Size(ms * 4, ms * 4));
  float rs = 1.f / config_.mpg * 4.f;
  int mhs = ms / 2 * 4;

  // static
  for (int a = 0; a < dim_P; a++) {
    for (int n = 0; n < dim_N; n++) {
      if (mask_static_ptr[a * dim_N + n] > 0.f) {
        float x = feature_static_ptr[a * dim_N * dim_F + n * dim_F];
        float y = feature_static_ptr[a * dim_N * dim_F + n * dim_F + 1];
        cv::Point2i p0(static_cast<int>(x * rs + mhs),
                       static_cast<int>(-y * rs + mhs));
        x = feature_static_ptr[a * dim_N * dim_F + n * dim_F + 2];
        y = feature_static_ptr[a * dim_N * dim_F + n * dim_F + 3];
        cv::Point2i p1(static_cast<int>(x * rs + mhs),
                       static_cast<int>(-y * rs + mhs));
        // int type = input_static[0][a][n][4].item<int>();
        int type_int = feature_static_ptr[a * dim_N * dim_F + n * dim_F + 4];
        VectorType type = (VectorType)type_int;
        cv::Scalar color;
        int width = 1;
        if (type > VectorType::LANE_PROPOSAL_BUS) {
          color = cv::Scalar(192, 192, 192);
        } else if ((type == VectorType::ROUTE_ON_ROUTE) ||
                   (type == VectorType::ROUTE_OFF_ROUTE) ||
                   (type == VectorType::ROUTE_BUS)) {
          color = cv::Scalar(0, 255, 255);
        } else if ((type == VectorType::LANE_PROPOSAL_ON_ROUTE) ||
                   (type == VectorType::LANE_PROPOSAL_OFF_ROUTE) ||
                   (type == VectorType::LANE_PROPOSAL_BUS)) {
          width = 2;
          if (a == selected_lane_index) {
            color = cv::Scalar(255, 0, 255);
          } else {
            color = cv::Scalar(255, 255, 0);
          }
        }
        cv::line(debug_im, p0, p1, color, width, CV_AA);
        cv::circle(debug_im, p1, 3, color, -1);
      }
    }
  }

  // obj
  for (int a = 0; a < dim_A; a++) {
    for (int n = 0; n < dim_N; n++) {
      if (mask_obj_ptr[a * dim_N + n] > 0.f) {
        float x = feature_obj_ptr[a * dim_N * dim_obj_F + n * dim_obj_F + 0];
        float y = feature_obj_ptr[a * dim_N * dim_obj_F + n * dim_obj_F + 1];
        cv::Point2i p0(static_cast<int>(x * rs + mhs),
                       static_cast<int>(-y * rs + mhs));
        x = feature_obj_ptr[a * dim_N * dim_obj_F + n * dim_obj_F + 2];
        y = feature_obj_ptr[a * dim_N * dim_obj_F + n * dim_obj_F + 3];
        cv::Point2i p1(static_cast<int>(x * rs + mhs),
                       static_cast<int>(-y * rs + mhs));
        // int type = input_obj[0][a][n][9].item<int>();
        if (n == 0) {
          float length =
              feature_obj_ptr[a * dim_N * dim_obj_F + n * dim_obj_F + 7];
          float width =
              feature_obj_ptr[a * dim_N * dim_obj_F + n * dim_obj_F + 8];
          float theta =
              feature_obj_ptr[a * dim_N * dim_obj_F + n * dim_obj_F + 10];
          float x_start =
              feature_obj_ptr[a * dim_N * dim_obj_F + n * dim_obj_F + 2];
          float y_start =
              feature_obj_ptr[a * dim_N * dim_obj_F + n * dim_obj_F + 3];
          cv::Scalar color_obj = cv::Scalar(255, 0, 0);
          if (a == 0) {
            length = 4.2f;
            width = 1.8f;
            color_obj = cv::Scalar(0, 0, 255);
          }
          float cos_y = std::cos(theta);
          float sin_y = std::sin(theta);
          float hl_x = cos_y * length / 2 + (-sin_y) * width / 2 + x_start;
          float hl_y = sin_y * length / 2 + (cos_y)*width / 2 + y_start;
          cv::Point2i p_hl(static_cast<int>(hl_x * rs + mhs),
                           static_cast<int>(-hl_y * rs + mhs));

          float hr_x = cos_y * length / 2 + (-sin_y) * (-width) / 2 + x_start;
          float hr_y = sin_y * length / 2 + (cos_y) * (-width) / 2 + y_start;
          cv::Point2i p_hr(static_cast<int>(hr_x * rs + mhs),
                           static_cast<int>(-hr_y * rs + mhs));

          float rr_x =
              cos_y * (-length) / 2 + (-sin_y) * (-width) / 2 + x_start;
          float rr_y = sin_y * (-length) / 2 + (cos_y) * (-width) / 2 + y_start;
          cv::Point2i p_rr(static_cast<int>(rr_x * rs + mhs),
                           static_cast<int>(-rr_y * rs + mhs));

          float rl_x = cos_y * (-length) / 2 + (-sin_y) * (width) / 2 + x_start;
          float rl_y = sin_y * (-length) / 2 + (cos_y) * (width) / 2 + y_start;
          cv::Point2i p_rl(static_cast<int>(rl_x * rs + mhs),
                           static_cast<int>(-rl_y * rs + mhs));

          cv::line(debug_im, p_hl, p_hr, color_obj, 2);
          cv::line(debug_im, p_hr, p_rr, color_obj, 2);
          cv::line(debug_im, p_rr, p_rl, color_obj, 2);
          cv::line(debug_im, p_hl, p_rl, color_obj, 2);
        }
        int type_int =
            feature_obj_ptr[a * dim_N * dim_obj_F + n * dim_obj_F + 9];
        VectorType type = (VectorType)type_int;
        cv::Scalar color;
        if (type == VectorType::OFO)
          color = cv::Scalar(0, 255, 0);
        else if (type == VectorType::HUMAN)
          color = cv::Scalar(0, 0, 255);
        else if (type == VectorType::CONE)
          color = cv::Scalar(255, 255, 255);
        else
          color = cv::Scalar(255, 0, 0);
        cv::arrowedLine(debug_im, p0, p1, color, 2);
        cv::circle(debug_im, p1, 3, color, -1);
      }
    }
  }

  // output
  for (int m = 0; m < mp; ++m) {
    if (mask_static_ptr[m * dim_N] < 1.f ||
        std::fabs(feature_static_ptr[m * dim_N * dim_F + 4] - 8) > 1e-3) {
      continue;
    }
    for (int t = 0; t < dim_Tf; t++) {
      double x = plan_traj_ptr[m * dim_Tf * 2 + t * 2 + 0];
      double y = plan_traj_ptr[m * dim_Tf * 2 + t * 2 + 1];
      cv::Point2i p0(static_cast<int>(x * rs + mhs),
                     static_cast<int>(-y * rs + mhs));
      if (m == selected_lane_index) {
        cv::circle(debug_im, p0, 3, {0, 255, 0}, -1);
      } else {
        cv::circle(debug_im, p0, 3, {255, 255, 0}, -1);
      }
    }
  }

  // ego speed, yaw
  constexpr int BUFFER_SIZE = 128;
  char sbuffer[BUFFER_SIZE];
  snprintf(sbuffer, BUFFER_SIZE, "%f, %f, %f", feature_obj_ptr[4],
           feature_obj_ptr[5], feature_obj_ptr[10]);
  cv::putText(debug_im, sbuffer, cv::Point2i(100, 100),
              cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, 8, 0);
  cv::putText(debug_im, std::to_string(timestamp), cv::Point2i(100, 150),
              cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, 8, 0);

  std::string ts_str = std::to_string(timestamp);
  cv::imwrite(config_.dump_image_dir + std::to_string(timestamp) + ".jpg",
              debug_im);
#endif
}

} // namespace ddp
} // namespace msquare

#endif
