#include "LDPPlanningExec/executable.h"
#include "ap.hpp"
#include "pnc.h"

#include "mjson/mjson.hpp"
#include "mlog_core/mlog.h"
#include "mlog_publisher/publisher.h"
#include "mtime_core/mtime.h"
#include "pnc/ldp_planning_engine_interface.h"
#include "timeline/timeline_mfr.h"

using namespace msd_planning;
using namespace msquare;

std::string read_file(const std::string &path) {
  std::ifstream fjson(path);
  std::string json_str((std::istreambuf_iterator<char>(fjson)),
                       std::istreambuf_iterator<char>());
  return json_str;
}

mlog::MLogLevel get_mlog_level(std::string level) {
  std::transform(level.begin(), level.end(), level.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (level == "debug") {
    return mlog::MLogLevel::MLOG_LEVEL_DEBUG;
  } else if (level == "info") {
    return mlog::MLogLevel::MLOG_LEVEL_INFO;
  } else if (level == "warn") {
    return mlog::MLogLevel::MLOG_LEVEL_WARN;
  } else if (level == "error") {
    return mlog::MLogLevel::MLOG_LEVEL_ERROR;
  } else if (level == "fatal") {
    return mlog::MLogLevel::MLOG_LEVEL_FATAL;
  } else {
    // use default level
    return mlog::MLogLevel::MLOG_LEVEL_INFO;
  }
}

int main(int argc, char **argv) {
  ap::APSpinner::init();

  ap::APSubscriber<ap_maf_std::header_service_interface::proxy::
                       std_HeaderServiceInterfaceProxy>
      sub_mff_ldp_planning_request_by_mff(
          "LDPPlanningExec/LDPPlanningRoot/LDPPlanningSubstd_Header",
          "std_Header/mff_ldp_planning_request/MFF");
  ap::APSubscriber<
      ap_maf_framework_status::module_status_service_interface::proxy::
          framework_status_ModuleStatusServiceInterfaceProxy>
      sub_msd_function_module_status_by_vehicle_service(
          "LDPPlanningExec/LDPPlanningRoot/"
          "LDPPlanningSubframework_status_ModuleStatus",
          "framework_status_ModuleStatus/msd_function_module_status/"
          "VehicleService");
  ap::APSubscriber<
      ap_maf_perception_interface::road_line_perception_service_interface::
          proxy::perception_interface_RoadLinePerceptionServiceInterfaceProxy>
      sub_perception_vision_lane_by_camera_detection(
          "LDPPlanningExec/LDPPlanningRoot/"
          "LDPPlanningSubperception_interface_RoadLinePerception",
          "perception_interface_RoadLinePerception/perception_vision_lane/"
          "CameraDetection");
  ap::APSubscriber<ap_maf_endpoint::body_report_service_interface::proxy::
                       endpoint_BodyReportServiceInterfaceProxy>
      sub_vehicle_body_report_by_vehicle_service(
          "LDPPlanningExec/LDPPlanningRoot/LDPPlanningSubendpoint_BodyReport",
          "endpoint_BodyReport/vehicle_body_report/VehicleService");
  ap::APSubscriber<ap_maf_endpoint::chassis_report_service_interface::proxy::
                       endpoint_ChassisReportServiceInterfaceProxy>
      sub_vehicle_chassis_report_by_vehicle_service(
          "LDPPlanningExec/LDPPlanningRoot/"
          "LDPPlanningSubendpoint_ChassisReport",
          "endpoint_ChassisReport/vehicle_chassis_report/VehicleService");
  ap::APSubscriber<ap_maf_endpoint::wheel_report_service_interface::proxy::
                       endpoint_WheelReportServiceInterfaceProxy>
      sub_vehicle_wheel_report_by_vehicle_service(
          "LDPPlanningExec/LDPPlanningRoot/LDPPlanningSubendpoint_WheelReport",
          "endpoint_WheelReport/vehicle_wheel_report/VehicleService");
  ap::APPublisher<ap_maf_std::header_service_interface::skeleton::
                      std_HeaderServiceInterfaceSkeleton>
      pub_mff_ldp_planning_response(
          "std_Header/mff_ldp_planning_response/LDPPlanning");
  ap::APPublisher<ap_maf_planning::planning_service_interface::skeleton::
                      planning_PlanningServiceInterfaceSkeleton>
      pub_msd_planning_ldp_plan(
          "planning_Planning/msd_planning_ldp_plan/LDPPlanning");
  ap::APPublisher<
      ap_maf_framework_status::nodes_status_service_interface::skeleton::
          framework_status_NodesStatusServiceInterfaceSkeleton>
      pub_node_status_ldp_planning(
          "framework_status_NodesStatus/node_status_ldp_planning/LDPPlanning");
  ap::APPublisher<ap_maf_std::header_service_interface::skeleton::
                      std_HeaderServiceInterfaceSkeleton>
      pub_planning_ldp_status("std_Header/planning_ldp_status/LDPPlanning");

  // yaml file don't support env parse, so use relative path directly
  std::string config_file = std::string(getenv("EXEC_PATH")) +
                            "/../resource/config/ldp_planning.ap.mdc.json";
  std::string mtaskflow_config_file =
      std::string(getenv("EXEC_PATH")) +
      "/../resource/config/mtaskflow_config.json";
  std::string config_file_dir =
      std::string(getenv("EXEC_PATH")) + "/../resource/config/";
  std::string config_data = read_file(config_file).c_str();

  std::shared_ptr<LdpPlanningEngineInterface> planning_engine;
  bool start_machine = false;
  bool is_paused = true;

  // init_config_data
  char *log_level = std::getenv("MSQUARE_PNC_LOG_LEVEL");
  auto config_reader = mjson::Reader(config_data.c_str());
  config_reader.expand_environment_varialbe();
  if (log_level) {
    std::string log_level_lower(log_level);
    transform(log_level_lower.begin(), log_level_lower.end(),
              log_level_lower.begin(), tolower);
    auto logger_reader =
        mjson::Reader(config_reader.get<mjson::Json>("logger"));
    logger_reader.override_value("level", log_level_lower);
    config_reader.override_value("logger", logger_reader.raw());
    config_data = config_reader.raw().dump().c_str();
  }

  start_machine = config_reader.get<bool>("start_machine", false, false);

  // init mlog and mtime
  mlog::PublisherSettings publisher_settings;
  void *node_handle = malloc(1);
  publisher_settings.handle = node_handle;
  publisher_settings.config_file = config_file.c_str();
  std::shared_ptr<mlog::MLogPublisher> publisher =
      mlog::MLogPublisher::make(publisher_settings);

  mlog::MLogEndpointCallback planning_callback =
      [publisher](const std::vector<mlog::MLogEntry> &entries) -> void {
    if (publisher != nullptr) {
      publisher->publish(entries);
    } else {
      fprintf(stderr, "create MLogPublisher failed.\n");
    }
  };

  // MSDPlanning_set_log_config(config_data, node_handle);
  // if (planning_callback) {
  //   MSDPlanning_set_log_callback(planning_callback);
  // }
  // MSDPlanning_init_log_manager();

  MSDPlanningTimeConfig planning_time_config{};
  planning_time_config.basic_config = config_data;
  mtime::MTimeConfig time_config{};
  time_config.timeline_type = mtime::MTIMELINE_TYPE_mtime_customize_timeline;
  planning_time_config.time_config = time_config;
  mtime::MTimeClockFeeder feeder;
  MSDPlanning_set_time_config(planning_time_config, feeder);

  // init LdpPlanningEngine
  MSDLdpPlanningConfig planning_config{};
  planning_config.enable_timer_tick = true;
  planning_config.mtaskflow_config_file = mtaskflow_config_file.c_str();
  planning_engine = LdpPlanningEngineInterface::make(planning_config);

  planning_engine->init();

  if (start_machine) {
    planning_engine->start();
  }

  sub_mff_ldp_planning_request_by_mff.setCallback(
      [&](const ap_maf_std::Header *src) {
        maf_std::Header dst{};
        FROM_AP(src, dst);
        planning_engine->feed_mff_planning_request(dst.frame_id);

        maf_std::Header maf_response{};
        maf_response.stamp = MTIME()->timestamp().ns();

        auto response_json = mjson::Json(mjson::Json::object());
        response_json["request_seq"] = mjson::Json(dst.seq);
        response_json["success"] = mjson::Json(true);
        maf_response.frame_id = response_json.dump();

        auto to_ap_dst = pub_mff_ldp_planning_response.allocateEmptyData();
        TO_AP(maf_response, to_ap_dst);
        pub_mff_ldp_planning_response.publish(std::move(to_ap_dst));

        // check ldp_w_status
        if (dst.frame_id != "") {
          auto request_reader = mjson::Reader(dst.frame_id);

          auto ldp_string =
              request_reader.get<std::string>("ldp_w_status", false, "");
          if (ldp_string != "") {
            auto ldp_reader = mjson::Reader(ldp_string);
            int status = ldp_reader.get<int>("status", false, 0);

            // 0: off 4: error
            if (status == 0 || status == 4) {
              is_paused = true;
              planning_engine->stop();
            } else {
              is_paused = false;
              planning_engine->start();
            }
          }
        }
      });
  sub_msd_function_module_status_by_vehicle_service.setCallback(
      [&](const ap_maf_framework_status::ModuleStatus *src) {
        if (is_paused) {
          return;
        }
        maf_framework_status::ModuleStatus dst{};
        FROM_AP(src, dst);
        planning_engine->feed_module_status(dst);
      });
  sub_perception_vision_lane_by_camera_detection.setCallback(
      [&](const ap_maf_perception_interface::RoadLinePerception *src) {
        if (is_paused) {
          return;
        }
        maf_perception_interface::RoadLinePerception dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_perception_interface::RoadLinePerception>(
                std::move(dst));
        planning_engine->feed_perception_vision_lane(dst_ptr);
      });
  sub_vehicle_body_report_by_vehicle_service.setCallback(
      [&](const ap_maf_endpoint::BodyReport *src) {
        if (is_paused) {
          return;
        }
        maf_endpoint::BodyReport dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_endpoint::BodyReport>(std::move(dst));
        planning_engine->feed_body_report(dst_ptr);
      });
  sub_vehicle_chassis_report_by_vehicle_service.setCallback(
      [&](const ap_maf_endpoint::ChassisReport *src) {
        if (is_paused) {
          return;
        }
        maf_endpoint::ChassisReport dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_endpoint::ChassisReport>(std::move(dst));
        planning_engine->feed_chassis_report(dst_ptr);
      });
  sub_vehicle_wheel_report_by_vehicle_service.setCallback(
      [&](const ap_maf_endpoint::WheelReport *src) {
        if (is_paused) {
          return;
        }
        maf_endpoint::WheelReport dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_endpoint::WheelReport>(std::move(dst));
        planning_engine->feed_wheel_report(dst_ptr);
      });

  // set callback for pub topics
  planning_engine->set_callback(
      [&pub_msd_planning_ldp_plan](
          const MSDPlanningOutputMeta &meta,
          const maf_planning::Planning &planning_result,
          const std::string &trigger_msg_id) {
        if (meta.succeed) {
          auto to_ap_dst = pub_msd_planning_ldp_plan.allocateEmptyData();
          TO_AP(planning_result, to_ap_dst);
          pub_msd_planning_ldp_plan.publish(std::move(to_ap_dst));
        }
      });

  planning_engine->set_callback([&pub_planning_ldp_status](
                                    const std::string &ldp_output) {
    maf_std::Header maf_ldp_status{};
    maf_ldp_status.stamp = MTIME()->timestamp().ns();
    maf_ldp_status.frame_id = ldp_output;

    auto ap_planning_ldp_status = pub_planning_ldp_status.allocateEmptyData();
    TO_AP(maf_ldp_status, ap_planning_ldp_status);
    pub_planning_ldp_status.publish(std::move(ap_planning_ldp_status));
  });

  planning_engine->set_callback(
      [&pub_node_status_ldp_planning](
          const maf_framework_status::NodeStatus &planning_node_status) {
        maf_framework_status::NodesStatus planning_nodes_status;
        planning_nodes_status.header.stamp = MTIME()->timestamp().ns();
        planning_nodes_status.nodes_status.push_back(planning_node_status);

        auto ap_planning_node_status =
            pub_node_status_ldp_planning.allocateEmptyData();
        TO_AP(planning_nodes_status, ap_planning_node_status);
        pub_node_status_ldp_planning.publish(
            std::move(ap_planning_node_status));
      });

  // Start algorithm handle if required

  ap::registerSignalHandler();
  ap::getSpinner().startPubSub();
  ap::getSpinner().spin();
  ap::APSpinner::exit();

  // Stop algorithm handle if required
}
