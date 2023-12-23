#pragma once

#include <string>
#include <vector>

namespace msquare {

const std::string TOPIC_MODULE_STATUS = "/msd/function_module_status";
const std::string MSIM_TOPIC_MODULE_STATUS = "/msim/msd/function_module_status";
const std::string TOPIC_CHASSIS_REPORT = "/vehicle/chassis_report";
const std::string MSIM_TOPIC_CHASSIS_REPORT = "/msim/vehicle/chassis_report";
const std::string TOPIC_WHEEL_REPORT = "/vehicle/wheel_report";
const std::string TOPIC_BODY_REPORT = "/vehicle/body_report";
const std::string TOPIC_TRAFFIC_LIGHT = "/worldmodel/traffic_light";
const std::string TOPIC_EGO_POSE = "/mla/egopose";
const std::string MSIM_TOPIC_EGO_POSE = "/msim/mla/egopose";
const std::string TOPIC_FUSION_OBJECTS = "/perception/fusion/object";
const std::string TOPIC_FUSION_GROUNDLINE = "/perception/fusion/ground_line";
const std::string TOPIC_PREDICTION_INFO = "/msd/prediction/prediction_result";
const std::string TOPIC_CP_PLANNING_CONTROL_CMD_REQUEST =
    "/system_manager/cp_planning/control_cmd_request";
const std::string TOPIC_PLANNING_CONTROL_CMD_REQUEST =
    "/system_manager/planning/control_cmd_request";
const std::string TOPIC_CP_PLANNING_REQUEST =
    "/system_manager/cp_planning/request";
const std::string TOPIC_PLANNING_REQUEST = "/system_manager/planning/request";
const std::string TOPIC_PLANNING_FREQ_CONTROL =
    "/msd/planning_frequency_control";
const std::string TOPIC_TRANSFORM = "/tf";
const std::string MVIZ_TOPIC_EGO_CAR_GT = "/parking_gt/egopose";

const std::string TOPIC_PLANNING_OUTPUT = "/msd/planning/plan";
const std::string MSIM_TOPIC_PLANNING_OUTPUT = "/msim/msd/planning/plan";
const std::string TOPIC_PLANNING_INFO = "/planning/info";
const std::string MSIM_TOPIC_PLANNING_INFO = "/msim/np_planning/info";
const std::string TOPIC_MDEBUG = "/cp_mdebug";
const std::string TOPIC_CONTROL_COMMAND = "/msd/endpoint/control_command";

const std::string TOPIC_SENSOR_IMU = "/sensor/imu";
const std::string TOPIC_MPC_TRAJECTORY = "/msd/control/mpc_traj";
const std::string TOPIC_PERCEPTION_VISION_LANE = "/perception/vision/lane";
const std::string TOPIC_WORLDMODEL_PROCESSED_MAP = "/worldmodel/processed_map";
const std::string TOPIC_WORLDMODEL_OBJECTS = "/worldmodel/objects";
const std::string TOPIC_PERCEPTION_RADAR_FRONT =
    "/perception/radar/result_front";
const std::string TOPIC_WORLDMODEL_SCENE_OBJECT = "/worldmodel/scene_object";
const std::string TOPIC_FUSION_GROUNDLINE_USS =
    "/perception/fusion/ground_line_uss";
const std::string TOPIC_FUSION_DISTANCE_USS = "/perception/fusion/distance_uss";
const std::string TOPIC_SBP_RESULT = "/msd/sbp_result";
const std::string TOPIC_PLANNING_RESET_REQUEST =
    "/system_manager/cp_planning/reset_request";
const std::string TOPIC_SYSTEM_MANAGER_WORLDMODEL_REQUEST_HIGHWAY =
    "/system_manager/worldmodel/request/highway";
const std::string TOPIC_SYSTEM_MANAGER_WORLDMODEL_CONTROL_CMD_REQUEST_HIGHWAY =
    "/system_manager/worldmodel/control_cmd_request/highway";
const std::string TOPIC_SYSTEM_MANAGER_WORLDMODEL_RESET_REQUEST =
    "/system_manager/worldmodel/reset_request";
const std::string TOPIC_NODE_STATUS_PLANNING = "/node_status/planning"; // pub
const std::string TOPIC_PLANNING_CP = "/msd/planning/cp";               // pub
const std::string TOPIC_PATH_PLANNER_INPUT = "/msd/path_planner_input"; // pub
const std::string TOPIC_CLA_EVENT_FILTER = "/msquare/cla_event_filter"; // pub
const std::string TOPIC_SBP_REQUEST = "/msd/sbp_request";               // pub
const std::string TOPIC_SYSTEM_MANAGER_CP_PLANNING_CONTROL_CMD_RESPONSE =
    "/system_manager/cp_planning/control_cmd_response"; // pub
const std::string TOPIC_SYSTEM_MANAGER_CP_PLANNING_RESPONSE =
    "/system_manager/cp_planning/response"; // pub
const std::string TOPIC_SYSTEM_MANAGER_CP_PLANNING_RESET_RESPONSE =
    "/system_manager/cp_planning/reset_response"; // pub
const std::string TOPIC_NODE_STATUS_WORLDMODEL_HIGHWAY =
    "/node_status/worldmodel/highway"; // pub
const std::string TOPIC_SYSTEM_MANAGER_WORLDMODEL_RESPONSE_HIGHWAY =
    "/system_manager/worldmodel/response/highway"; // pub
const std::string TOPIC_SYSTEM_MANAGER_WORLDMODEL_CONTROL_CMD_RESPONSE_HIGHWAY =
    "/system_manager/worldmodel/control_cmd_response/highway"; // pub
const std::string TOPIC_SYSTEM_MANAGER_WORLDMODEL_RESET_RESPONSE =
    "/system_manager/worldmodel/reset_response"; // pub

// for APA
const std::string TOPIC_WORLDMODEL_PARKING_SLOT =
    "/worldmodel/parking_slot_info";
const std::string TOPIC_PERCEPTION_FUSION_OBJ_PARKING_ENV =
    "/perception/fusion/object_parking_environment";
const std::string TOPIC_SYSTEM_MANAGER_INFO_PARKING =
    "/system_manager/info/parking";
const std::string TOPIC_PERCEPTION_FUSION_PARKING_SLOT =
    "/perception/fusion/parking_slot";
const std::string TOPIC_SYSTEM_MANAGER_WORLDMODEL_CONTROL_CMD_REQUEST_PARKING =
    "/system_manager/worldmodel/control_cmd_request/parking";
const std::string TOPIC_SYSTEM_MANAGER_WORLDMODEL_RESPONSE_PARKING =
    "/system_manager/worldmodel/response/parking"; // pub
const std::string TOPIC_NODE_STATUS_WORLDMODEL_PARKING =
    "/node_status/worldmodel/parking"; // pub

const std::string FPP_MODULE_STATUS_MSG_ON_AUTO = "fpp module status on auto";
const std::string FPP_MODULE_STATUS_MSG_NOT_ON_AUTO =
    "fpp module status not on auto";

constexpr int RETURN_CODE_SUCCESS = 0;
constexpr int RETURN_CODE_FAILED = 255;
constexpr int64_t MICRO_NS = 1000;
constexpr int64_t MILLI_NS = 1000 * MICRO_NS;
constexpr int64_t SECOND_NS = 1000 * MILLI_NS;
constexpr int64_t SECOND_MS = 1000;
constexpr int64_t SECOND_US = 1000 * SECOND_MS;
constexpr int64_t MILLI_US = 1000;
constexpr int64_t FRAME_TIME_SPAN_NS = 100 * MILLI_NS;
constexpr int MAX_FRAME_COUNT = 1000;
constexpr int WARM_UP_FRAME_COUNT = 30;
constexpr int DEFAULT_DEBUG_FRAME = -1;
constexpr int FRAME_DURATION = 100;
constexpr int64_t FRAME_TIME_ADJUSTMENT_NS = MICRO_NS;
constexpr int MAX_FRAME_COUNT_NOT_ON_AUTO = 20;
constexpr int FPP_DEFAULT_SWITCH_FRAME = 15;
constexpr double kMaxEgoposeFuzzyMatchingDiffForLanePerceptionMeters = 50;
const uint64_t kEgoposeCountPerFrame = 5;

constexpr double kAccInformationDefaultCruiseSpeed = 120.0;
constexpr double kAccInformationDefaultTimeDistance = 2.0;

constexpr double kDefaultSpeedLimit = 120.0;

const std::string PLAN_FILE_SUFFIX = ".plan";
const std::string FRAME_FILE_SUFFIX = ".frame";
const std::string MFBAG_SUFFIX = ".mfbag";

inline bool is_mfbag(const std::string &bag_path) {
  const auto pos = bag_path.find(MFBAG_SUFFIX);
  if (pos == std::string::npos) {
    return false;
  }
  return bag_path.substr(pos) == MFBAG_SUFFIX;
}

const std::string DEFAULT_NPP_RESOURCE_DIR =
    "/opt/maf_planning/resource/config";

inline const std::string &get_planning_resource_dir() {
  static std::string planning_resource_dir = []() {
    const std::string NPP_RESOURCE_PATH_ENV_NAME = "NPP_RESOURCE_PATH";

    char *env = getenv(NPP_RESOURCE_PATH_ENV_NAME.c_str());
    if (env != nullptr) {
      return std::string{env};
    } else {
      return DEFAULT_NPP_RESOURCE_DIR;
    }
  }();
  return planning_resource_dir;
}

inline const std::string &get_planning_config_dir() {
  return get_planning_resource_dir();
}

inline bool failed(int rc) { return rc != RETURN_CODE_SUCCESS; }

inline std::string
make_plan_result_file_path(const std::string &frame_file_path) {
  return frame_file_path + PLAN_FILE_SUFFIX;
}

inline std::string make_frame_file_path(const std::string &frame_file_path,
                                        int index) {
  return frame_file_path + "." + std::to_string(index) + FRAME_FILE_SUFFIX;
}

inline double interpolate(double t_start, double y1, double t_end, double y2,
                          double t_calc) {
  if (t_end == t_start) {
    // printf("interpolate error, t_start=t_end\n");
    return y2;
  } else {
    auto ratio = (t_end - t_calc) / (t_end - t_start);
    auto y = ratio * y1 + (1 - ratio) * y2;
    return y;
  }
}

struct NormalPoint3D {
  double x;
  double y;
  double z;
};

struct NormalPoint4D {
  double w;
  double x;
  double y;
  double z;
};

struct PosGlobal {
  double latitude;  //!< \unit{°} \value_min{-90°} \value_max{-90°} (.8)
  double longitude; //!< \unit{°} \value_min{-180°} \value_max{-180°} (.8)
  double altitude;  //!< \unit{m} (.3)
};

} // namespace msquare