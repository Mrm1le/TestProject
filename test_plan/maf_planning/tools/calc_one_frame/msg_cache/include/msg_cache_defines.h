#pragma once

#include <string>

namespace msc {

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
const std::string MSIM_TOPIC_CP_PLANNING_REQUEST =
    "/msim/system_manager/cp_planning/request";
const std::string TOPIC_PLANNING_REQUEST = "/system_manager/planning/request";
const std::string TOPIC_PLANNING_FREQ_CONTROL =
    "/msd/planning_frequency_control";
const std::string TOPIC_TRANSFORM = "/tf";
const std::string MVIZ_TOPIC_EGO_CAR_GT = "/parking_gt/egopose";

const std::string TOPIC_PLANNING_OUTPUT = "/msd/planning/plan";
const std::string MSIM_TOPIC_PLANNING_OUTPUT = "/msim/msd/planning/plan";
const std::string TOPIC_PLANNING_INFO = "/planning/info";
const std::string MSIM_TOPIC_PLANNING_INFO = "/msim/np_planning/info";
const std::string TOPIC_MDEBUG = "/mdebug";
const std::string TOPIC_CONTROL_COMMAND = "/msd/endpoint/control_command";
const std::string TOPIC_PLANNING_EXTRA_DATA = "/cp_planning/extra_data";

const std::string TOPIC_SENSOR_IMU = "/sensor/imu";
const std::string TOPIC_MPC_TRAJECTORY = "/msd/control/mpc_traj";
const std::string TOPIC_PERCEPTION_VISION_LANE = "/perception/vision/lane";
const std::string TOPIC_PERCEPTION_VISION_LANDMARK =
    "/perception/vision/landmark";
const std::string TOPIC_PERCEPTION_LIDAR_ROAD_EDGE =
    "/perception/lidar/road_edge";
const std::string TOPIC_WORLDMODEL_PROCESSED_MAP = "/worldmodel/processed_map";
const std::string TOPIC_WORLDMODEL_OBJECTS = "/worldmodel/objects";
const std::string TOPIC_PERCEPTION_RADAR_FRONT =
    "/perception/radar/result_front";
const std::string TOPIC_WORLDMODEL_SCENE_OBJECT = "/worldmodel/scene_object";
const std::string MSIM_TOPIC_WORLDMODEL_SCENE_OBJECT =
    "/msim/worldmodel/scene_object";
const std::string TOPIC_FUSION_GROUNDLINE_USS =
    "/perception/fusion/ground_line_uss";
const std::string TOPIC_FUSION_DISTANCE_USS = "/perception/fusion/distance_uss";
const std::string TOPIC_FUSION_DISTANCE_USS_RAW =
    "/sensor/ultrasonic/upa_distance";
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
const std::string MSIM_TOPIC_NODE_STATUS_PLANNING =
    "/msim/node_status/planning"; // pub
const std::string TOPIC_NODE_STATUS_APA_PLANNING =
    "/node_status/apa_planning"; // pub
const std::string MSIM_TOPIC_NODE_STATUS_APA_PLANNING =
    "/msim/node_status/apa_planning";                                   // pub
const std::string TOPIC_PLANNING_CP = "/msd/planning/cp";               // pub
const std::string TOPIC_PATH_PLANNER_INPUT = "/msd/path_planner_input"; // pub
const std::string TOPIC_CLA_EVENT_FILTER = "/msquare/cla_event_filter"; // pub
const std::string TOPIC_SBP_REQUEST = "/msd/sbp_request";               // pub
const std::string MSIM_TOPIC_SBP_REQUEST = "/msim/msd/sbp_request";
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
const std::string MSIM_TOPIC_WORLDMODEL_PARKING_SLOT =
    "/msim/worldmodel/parking_slot_info";
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

constexpr int64_t MICRO_NS = 1000;
} // namespace msc
