#pragma once
#include <endpoint_mfrmsgs/endpoint_mfrmsgs.h>
#include <glog/logging.h>
#include <maf_interface/maf_endpoint.h>
#include <maf_interface/maf_framework_status.h>
#include <maf_interface/maf_hdmap.h>
#include <maf_interface/maf_perception_interface.h>
#include <maf_interface/maf_planning.h>
#include <maf_interface/maf_std.h>
#include <maf_interface/maf_system_manager.h>
#include <maf_interface/maf_worldmodel.h>
#include <maf_message_cache.h>
#include <mfr/core/node.h>
#include <mfr/core/publisher.h>
#include <msg_cache_defines.h>
#include <perception_interface_mfrmsgs/perception_interface_mfrmsgs.h>
#include <planning_mfrmsgs/planning_mfrmsgs.h>
#include <std_mfrmsgs/std_mfrmsgs.h>
#include <system_manager_mfrmsgs/system_manager_mfrmsgs.h>
#include <worldmodel_mfrmsgs/worldmodel_mfrmsgs.h>

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <map>
#include <thread>

// #include "core/common/utils.h"
// #include "core/lib/npp_interface.h"
// #include "core/proto/ego_prediction_config.pb.h"
#include "endpoint_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "framework_status_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "gps_imu_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "hdmap_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "mfr/mfr.h"
#include "mjson/mjson.hpp"
#include "mla_localization_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "mlog_core/mlog.h"
#include "mlog_publisher/publisher.h"
#include "mtaskflow/mtaskflow.hpp"
#include "mtime_core/mtime.h"
#include "perception_interface_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "planning_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "sensor_interface_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "std_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "std_mfrmsgs/std_mfrmsgs.h"
#include "timeline/timeline_mfr.h"
#include "worldmodel_mfrmsgs/mfr_cpp_struct_convert.hpp"
namespace fpp {
template <typename MFRType> struct MapMafMfr;

template <> struct MapMafMfr<maf_planning::Planning> {
  typedef mmessage::planning_mfrmsgs::MFRMessagePlanning MfrType;
};
template <> struct MapMafMfr<maf_perception_interface::RoadLinePerception> {
  typedef mmessage::perception_interface_mfrmsgs::MFRMessageRoadLinePerception
      MfrType;
};
template <>
struct MapMafMfr<maf_perception_interface::PerceptionFusionObjectResult> {
  typedef mmessage::perception_interface_mfrmsgs::
      MFRMessagePerceptionFusionObjectResult MfrType;
};

template <> struct MapMafMfr<maf_endpoint::BodyReport> {
  typedef mmessage::endpoint_mfrmsgs::MFRMessageBodyReport MfrType;
};

template <> struct MapMafMfr<maf_endpoint::ChassisReport> {
  typedef mmessage::endpoint_mfrmsgs::MFRMessageChassisReport MfrType;
};

template <> struct MapMafMfr<maf_mla_localization::MLALocalization> {
  typedef mmessage::mla_localization_mfrmsgs::MFRMessageMLALocalization MfrType;
};

template <> struct MapMafMfr<maf_framework_status::ModuleStatus> {
  typedef mmessage::framework_status_mfrmsgs::MFRMessageModuleStatus MfrType;
};
template <> struct MapMafMfr<maf_system_manager::SysPlanningRequest> {
  typedef mmessage::system_manager_mfrmsgs::MFRMessageSysPlanningRequest
      MfrType;
};

template <> struct MapMafMfr<maf_hdmap::LocalMap> {
  typedef mmessage::hdmap_mfrmsgs::MFRMessageLocalMap MfrType;
};
template <> struct MapMafMfr<maf_std::Header> {
  typedef mmessage::std_mfrmsgs::MFRMessageHeader MfrType;
};
template <> struct MapMafMfr<maf_endpoint::ControlCommand> {
  typedef mmessage::endpoint_mfrmsgs::MFRMessageControlCommand MfrType;
};
template <> struct MapMafMfr<maf_endpoint::WheelReport> {
  typedef mmessage::endpoint_mfrmsgs::MFRMessageWheelReport MfrType;
};

template <> struct MapMafMfr<maf_perception_interface::TrafficLightPerception> {
  typedef mmessage::perception_interface_mfrmsgs::
      MFRMessageTrafficLightPerception MfrType;
};
template <> struct MapMafMfr<maf_worldmodel::PredictionResult> {
  typedef mmessage::worldmodel_mfrmsgs::MFRMessagePredictionResult MfrType;
};
template <> struct MapMafMfr<maf_planning::MpcTrajectoryResult> {
  typedef mmessage::planning_mfrmsgs::MFRMessageMpcTrajectoryResult MfrType;
};
template <> struct MapMafMfr<maf_system_manager::SysMapManagerRequest> {
  typedef mmessage::system_manager_mfrmsgs::MFRMessageSysMapManagerRequest
      MfrType;
};
template <>
struct MapMafMfr<maf_perception_interface::FusionParkingSlotResult> {
  typedef mmessage::perception_interface_mfrmsgs::
      MFRMessageFusionParkingSlotResult MfrType;
};
template <> struct MapMafMfr<maf_sensor_interface::UltrasonicUpaReport> {
  typedef mmessage::sensor_interface_mfrmsgs::MFRMessageUltrasonicUpaReport
      MfrType;
};

template <> struct MapMafMfr<maf_framework_status::NodesStatus> {
  typedef mmessage::framework_status_mfrmsgs::MFRMessageNodesStatus MfrType;
};
template <> struct MapMafMfr<maf_worldmodel::SceneObjects> {
  typedef mmessage::worldmodel_mfrmsgs::MFRMessageSceneObjects MfrType;
};
template <> struct MapMafMfr<maf_worldmodel::FusionAPA> {
  typedef mmessage::worldmodel_mfrmsgs::MFRMessageFusionAPA MfrType;
};

class FPPNode : public mfr::MFRNode {
public:
  bool on_init(mfr::MFRNodeHandle *node_handle) override {
    auto &communication_manager = node_handle->communication_manager();
    using namespace msc;

    publishers_[TOPIC_PLANNING_OUTPUT] =
        communication_manager
            .advertise<MapMafMfr<maf_planning::Planning>::MfrType>(
                mfr::
                MFRPublisherConfig{topic_name : TOPIC_PLANNING_OUTPUT.c_str()});

    publishers_[TOPIC_PLANNING_INFO] =
        communication_manager.advertise<MapMafMfr<maf_std::Header>::MfrType>(
            mfr::MFRPublisherConfig{topic_name : TOPIC_PLANNING_INFO.c_str()});

    publishers_[TOPIC_MDEBUG] =
        communication_manager.advertise<MapMafMfr<maf_std::Header>::MfrType>(
            mfr::MFRPublisherConfig{topic_name : TOPIC_MDEBUG.c_str()});

    publishers_[TOPIC_PLANNING_EXTRA_DATA] =
        communication_manager.advertise<MapMafMfr<maf_std::Header>::MfrType>(
            mfr::
            MFRPublisherConfig{topic_name : TOPIC_PLANNING_EXTRA_DATA.c_str()});

    publishers_[TOPIC_WHEEL_REPORT] =
        communication_manager
            .advertise<MapMafMfr<maf_endpoint::WheelReport>::MfrType>(
                mfr::
                MFRPublisherConfig{topic_name : TOPIC_WHEEL_REPORT.c_str()});

    publishers_[TOPIC_TRAFFIC_LIGHT] = communication_manager.advertise<
        MapMafMfr<maf_perception_interface::TrafficLightPerception>::MfrType>(
        mfr::MFRPublisherConfig{topic_name : TOPIC_TRAFFIC_LIGHT.c_str()});
    publishers_[TOPIC_PREDICTION_INFO] =
        communication_manager
            .advertise<MapMafMfr<maf_worldmodel::PredictionResult>::MfrType>(
                mfr::
                MFRPublisherConfig{topic_name : TOPIC_PREDICTION_INFO.c_str()});
    publishers_[TOPIC_MPC_TRAJECTORY] =
        communication_manager
            .advertise<MapMafMfr<maf_planning::MpcTrajectoryResult>::MfrType>(
                mfr::
                MFRPublisherConfig{topic_name : TOPIC_MPC_TRAJECTORY.c_str()});
    publishers_[TOPIC_CONTROL_COMMAND] =
        communication_manager
            .advertise<MapMafMfr<maf_endpoint::ControlCommand>::MfrType>(
                mfr::
                MFRPublisherConfig{topic_name : TOPIC_CONTROL_COMMAND.c_str()});

    publishers_[TOPIC_PERCEPTION_LIDAR_ROAD_EDGE] =
        communication_manager.advertise<
            MapMafMfr<maf_perception_interface::RoadLinePerception>::MfrType>(
            mfr::MFRPublisherConfig{
              topic_name : TOPIC_PERCEPTION_LIDAR_ROAD_EDGE.c_str()
            });
    publishers_[MSIM_TOPIC_PLANNING_OUTPUT] =
        communication_manager
            .advertise<MapMafMfr<maf_planning::Planning>::MfrType>(
                mfr::MFRPublisherConfig{
                  topic_name : MSIM_TOPIC_PLANNING_OUTPUT.c_str()
                });
    publishers_[TOPIC_MODULE_STATUS] =
        communication_manager
            .advertise<MapMafMfr<maf_framework_status::ModuleStatus>::MfrType>(
                mfr::
                MFRPublisherConfig{topic_name : TOPIC_MODULE_STATUS.c_str()});
    publishers_[MSIM_TOPIC_MODULE_STATUS] =
        communication_manager
            .advertise<MapMafMfr<maf_framework_status::ModuleStatus>::MfrType>(
                mfr::MFRPublisherConfig{
                  topic_name : MSIM_TOPIC_MODULE_STATUS.c_str()
                });
    publishers_[TOPIC_CHASSIS_REPORT] =
        communication_manager
            .advertise<MapMafMfr<maf_endpoint::ChassisReport>::MfrType>(
                mfr::
                MFRPublisherConfig{topic_name : TOPIC_CHASSIS_REPORT.c_str()});
    publishers_[MSIM_TOPIC_CHASSIS_REPORT] =
        communication_manager
            .advertise<MapMafMfr<maf_endpoint::ChassisReport>::MfrType>(
                mfr::MFRPublisherConfig{
                  topic_name : MSIM_TOPIC_CHASSIS_REPORT.c_str()
                });
    publishers_[TOPIC_BODY_REPORT] =
        communication_manager
            .advertise<MapMafMfr<maf_endpoint::BodyReport>::MfrType>(
                mfr::
                MFRPublisherConfig{topic_name : TOPIC_BODY_REPORT.c_str()});
    publishers_[TOPIC_FUSION_OBJECTS] =
        communication_manager.advertise<MapMafMfr<
            maf_perception_interface::PerceptionFusionObjectResult>::MfrType>(
            mfr::MFRPublisherConfig{topic_name : TOPIC_FUSION_OBJECTS.c_str()});
    publishers_[MSIM_TOPIC_PLANNING_INFO] =
        communication_manager.advertise<MapMafMfr<maf_std::Header>::MfrType>(
            mfr::
            MFRPublisherConfig{topic_name : MSIM_TOPIC_PLANNING_INFO.c_str()});
    // publishers_[MSIM_TOPIC_MDEBUG] =
    //     communication_manager.advertise<MapMafMfr<maf_std::Header>::MfrType>(
    //         mfr::MFRPublisherConfig{topic_name : MSIM_TOPIC_MDEBUG.c_str()});
    publishers_[TOPIC_PERCEPTION_VISION_LANE] = communication_manager.advertise<
        MapMafMfr<maf_perception_interface::RoadLinePerception>::MfrType>(
        mfr::
        MFRPublisherConfig{topic_name : TOPIC_PERCEPTION_VISION_LANE.c_str()});
    publishers_[TOPIC_PERCEPTION_VISION_LANDMARK] =
        communication_manager.advertise<
            MapMafMfr<maf_perception_interface::RoadLinePerception>::MfrType>(
            mfr::MFRPublisherConfig{
              topic_name : TOPIC_PERCEPTION_VISION_LANDMARK.c_str()
            });
    publishers_[MSIM_TOPIC_EGO_POSE] = communication_manager.advertise<
        MapMafMfr<maf_mla_localization::MLALocalization>::MfrType>(
        mfr::MFRPublisherConfig{topic_name : MSIM_TOPIC_EGO_POSE.c_str()});
    publishers_[MVIZ_TOPIC_EGO_CAR_GT] = communication_manager.advertise<
        MapMafMfr<maf_mla_localization::MLALocalization>::MfrType>(
        mfr::MFRPublisherConfig{topic_name : MVIZ_TOPIC_EGO_CAR_GT.c_str()});
    publishers_[TOPIC_EGO_POSE] = communication_manager.advertise<
        MapMafMfr<maf_mla_localization::MLALocalization>::MfrType>(
        mfr::MFRPublisherConfig{topic_name : TOPIC_EGO_POSE.c_str()});
    publishers_[TOPIC_MODULE_STATUS] =
        communication_manager
            .advertise<MapMafMfr<maf_framework_status::ModuleStatus>::MfrType>(
                mfr::
                MFRPublisherConfig{topic_name : TOPIC_MODULE_STATUS.c_str()});
    publishers_[TOPIC_CHASSIS_REPORT] =
        communication_manager
            .advertise<MapMafMfr<maf_endpoint::ChassisReport>::MfrType>(
                mfr::
                MFRPublisherConfig{topic_name : TOPIC_CHASSIS_REPORT.c_str()});
    publishers_[TOPIC_BODY_REPORT] =
        communication_manager
            .advertise<MapMafMfr<maf_endpoint::BodyReport>::MfrType>(
                mfr::
                MFRPublisherConfig{topic_name : TOPIC_BODY_REPORT.c_str()});
    publishers_[TOPIC_FUSION_OBJECTS] =
        communication_manager.advertise<MapMafMfr<
            maf_perception_interface::PerceptionFusionObjectResult>::MfrType>(
            mfr::MFRPublisherConfig{topic_name : TOPIC_FUSION_OBJECTS.c_str()});
    publishers_[TOPIC_PERCEPTION_VISION_LANE] = communication_manager.advertise<
        MapMafMfr<maf_perception_interface::RoadLinePerception>::MfrType>(
        mfr::
        MFRPublisherConfig{topic_name : TOPIC_PERCEPTION_VISION_LANE.c_str()});
    publishers_[TOPIC_PERCEPTION_VISION_LANDMARK] =
        communication_manager.advertise<
            MapMafMfr<maf_perception_interface::RoadLinePerception>::MfrType>(
            mfr::MFRPublisherConfig{
              topic_name : TOPIC_PERCEPTION_VISION_LANDMARK.c_str()
            });
    publishers_[TOPIC_PERCEPTION_FUSION_OBJ_PARKING_ENV] =
        communication_manager.advertise<MapMafMfr<
            maf_perception_interface::PerceptionFusionObjectResult>::MfrType>(
            mfr::MFRPublisherConfig{
              topic_name : TOPIC_PERCEPTION_FUSION_OBJ_PARKING_ENV.c_str()
            });
    publishers_[TOPIC_FUSION_DISTANCE_USS] = communication_manager.advertise<
        MapMafMfr<maf_sensor_interface::UltrasonicUpaReport>::MfrType>(
        mfr::
        MFRPublisherConfig{topic_name : TOPIC_FUSION_DISTANCE_USS.c_str()});
    publishers_[TOPIC_FUSION_DISTANCE_USS_RAW] =
        communication_manager.advertise<
            MapMafMfr<maf_sensor_interface::UltrasonicUpaReport>::MfrType>(
            mfr::MFRPublisherConfig{
              topic_name : TOPIC_FUSION_DISTANCE_USS_RAW.c_str()
            });
    publishers_[TOPIC_PERCEPTION_FUSION_PARKING_SLOT] =
        communication_manager.advertise<MapMafMfr<
            maf_perception_interface::FusionParkingSlotResult>::MfrType>(
            mfr::MFRPublisherConfig{
              topic_name : TOPIC_PERCEPTION_FUSION_PARKING_SLOT.c_str()
            });
    publishers_[TOPIC_NODE_STATUS_PLANNING] =
        communication_manager
            .advertise<MapMafMfr<maf_framework_status::NodesStatus>::MfrType>(
                mfr::MFRPublisherConfig{
                  topic_name : TOPIC_NODE_STATUS_PLANNING.c_str()
                });
    publishers_[MSIM_TOPIC_WORLDMODEL_PARKING_SLOT] =
        communication_manager
            .advertise<MapMafMfr<maf_worldmodel::FusionAPA>::MfrType>(
                mfr::MFRPublisherConfig{
                  topic_name : MSIM_TOPIC_WORLDMODEL_PARKING_SLOT.c_str()
                });
    publishers_[TOPIC_PLANNING_REQUEST] =
        communication_manager
            .advertise<MapMafMfr<maf_system_manager::SysPlanningRequest>::MfrType>(
                mfr::MFRPublisherConfig{
                  topic_name : TOPIC_PLANNING_REQUEST.c_str()
                });


    return true;
  }
  void on_finish() override {}
  void on_running(const mfr::MFRNodeRunningInfo &info) override {}
  template <typename MafType>
  void publish(const std::string &topic, const MafType &t) {
    typedef typename MapMafMfr<MafType>::MfrType MfrMsg;
    MfrMsg p;
    mfr_cpp_struct_convert::to_mfr(t, p);
    if (publishers_.find(topic) == publishers_.end()) {
      LOG(FATAL) << "topic " << topic << " not registered";
    }
    publishers_[topic]->publish(p);
  }

private:
  std::map<std::string, mfr::MFRPublisher *> publishers_;
};

class MFRPublisher;

class MFRPublisher {
public:
  MFRPublisher() : machine(mfr::MFRNodeMachine::instance()) {
    fpp_node = mmemory::MFMakeShared<FPPNode>();
    mfr::MFRNodeFactory::instance().register_by_name(
        "fpp_node_type", [&]() { return fpp_node; });

    std::srand(std::time(0));
    auto rand = std::to_string(std::rand());
    mfr::MFRMachineConfig machine_config;
    auto fpp_machine_name = "fpp_machine_" + rand;
    machine_config.machine_name = fpp_machine_name.c_str();
    machine_config.machine_url = "mfrrpc://127.0.0.1:11300";
    machine_config.machine_param_yaml = machine_param_yaml();

    mfr::MFRNodeConfig node_config;
    auto fpp_node_name = "fpp_node_" + rand;
    node_config.node_name = fpp_node_name.c_str();
    node_config.node_type = "fpp_node_type";
    node_config.node_param_yaml = param_yaml();

    machine.register_node(node_config);
    machine.init(machine_config); // TODO
    machine.run();
    // thr = std::thread(&MFRPublisher::pub_async, this);
  }
  ~MFRPublisher() {
    running = false;

    machine.stop();
    machine.join();
  }
  template <class T> void publish_topic(const std::string &topic, const T &t) {
    fpp_node->publish(topic, t);
  }
  void pub_async_NONONO() {
    while (running) {
      std::queue<maf_mla_localization::MLALocalization> q2;
      {
        std::unique_lock<std::mutex> lck(m);
        while (q.empty() && running)
          cv.wait(lck);
        q2 = q;
        while (!q.empty())
          q.pop();
        lck.unlock();
      }
      int interval = 10 * (1 - (int)(q2.size() / 5) * 0.2);
      LOG(INFO) << "q2 q1 size " << q2.size() << " " << q.size() << " interval "
                << interval << std::endl;
      while (!q2.empty()) {
        LOG(INFO) << "pop" << std::endl;
        publish_topic(msc::TOPIC_EGO_POSE, q2.front());
        q2.pop();
        std::this_thread::sleep_for(std::chrono::milliseconds(interval));
      }
    }
  }
  void publish_frame(msc::MafMessageCache &msg_cache, int frame) {
    for_each(
        msg_cache,
        [this, frame](auto &msgs, const std::string &topic) {
          //  LOG(INFO) << "frame " << frame << " topic " << topic;
          for (auto &msg : msgs.frame(frame)) {
            publish_topic(topic, msg.msg());
          }
        },
        true);
  }
  template <typename Func>
  void for_each(msc::MafMessageCache &msg_cache, Func callback,
                bool is_closed_loop) {
    using namespace msc;
    callback(msg_cache.new_planning_msgs(), TOPIC_PLANNING_OUTPUT);
    callback(msg_cache.new_planning_info_msgs(), TOPIC_PLANNING_INFO);
    // callback(msg_cache.new_mdebug_msgs(), TOPIC_MDEBUG);
    callback(msg_cache.new_planning_extra_data_msgs(),
             TOPIC_PLANNING_EXTRA_DATA);
    // callback(msg_cache.origin_local_map_msgs(), TOPIC_HDMAP_LOCALMAP_ONLINE);
    // callback(msg_cache.origin_local_map_msgs(), TOPIC_HDMAP_LOCALMAP);
    // callback(msg_cache.origin_hdmap_version_msgs(), TOPIC_HDMAP_VERSION);
    callback(msg_cache.origin_wheel_report_msgs(), TOPIC_WHEEL_REPORT);
    callback(msg_cache.origin_traffic_light_perception_msgs(),
             TOPIC_TRAFFIC_LIGHT);
    callback(msg_cache.origin_prediction_result_msgs(), TOPIC_PREDICTION_INFO);
    callback(msg_cache.origin_mpc_trajectory_msgs(), TOPIC_MPC_TRAJECTORY);
    callback(msg_cache.new_control_command_msgs(), TOPIC_CONTROL_COMMAND);
    // callback(msg_cache.origin_control_command_msgs(),
    //          MSIM_TOPIC_CONTROL_COMMAND);
    // callback(msg_cache.origin_sys_map_manager_request_msgs(),
    //          TOPIC_MAP_MANAGER_REQUEST);
    // callback(msg_cache.origin_maf_mff_info_msgs(), TOPIC_MAF_MFF_INFO);

    callback(msg_cache.origin_perception_lidar_road_edge_msgs(),
             TOPIC_PERCEPTION_LIDAR_ROAD_EDGE);

    // apa
    callback(msg_cache.origin_perception_fusion_obj_parking_env_msgs(),
             TOPIC_PERCEPTION_FUSION_OBJ_PARKING_ENV);
    callback(msg_cache.origin_fusion_parking_slot_msgs(),
             TOPIC_PERCEPTION_FUSION_PARKING_SLOT);
    callback(msg_cache.origin_ultrasonic_upa_report_msgs(),
             TOPIC_FUSION_DISTANCE_USS);
    callback(msg_cache.origin_ultrasonic_upa_report_raw_msgs(),
             TOPIC_FUSION_DISTANCE_USS_RAW);

    if (is_closed_loop) {
      callback(msg_cache.origin_planning_msgs(), MSIM_TOPIC_PLANNING_OUTPUT);
      callback(msg_cache.new_module_status_msgs(), TOPIC_MODULE_STATUS);
      callback(msg_cache.origin_module_status_msgs(), MSIM_TOPIC_MODULE_STATUS);
      callback(msg_cache.new_chassis_report_msgs(), TOPIC_CHASSIS_REPORT);
      callback(msg_cache.origin_chassis_report_msgs(),
               MSIM_TOPIC_CHASSIS_REPORT);
      callback(msg_cache.new_body_report_msgs(), TOPIC_BODY_REPORT);
      callback(msg_cache.new_perception_fusion_object_result_msgs(),
               TOPIC_FUSION_OBJECTS);
      callback(msg_cache.origin_planning_info_msgs(), MSIM_TOPIC_PLANNING_INFO);
      // callback(msg_cache.origin_mdebug_msgs(), MSIM_TOPIC_MDEBUG);
      callback(msg_cache.new_perception_vision_lane_msgs(),
               TOPIC_PERCEPTION_VISION_LANE);
      callback(msg_cache.new_perception_vision_landmark_msgs(),
               TOPIC_PERCEPTION_VISION_LANDMARK);
      callback(msg_cache.origin_mla_localization_msgs(), MSIM_TOPIC_EGO_POSE);
      callback(msg_cache.origin_mla_localization_msgs(), MVIZ_TOPIC_EGO_CAR_GT);
      callback(msg_cache.new_mla_localization_msgs(), TOPIC_EGO_POSE);
      // apa
      callback(msg_cache.origin_sys_planning_request_msgs(),
               TOPIC_PLANNING_REQUEST);
      callback(msg_cache.origin_node_status_apa_planning_msgs(),
               TOPIC_NODE_STATUS_APA_PLANNING);
      callback(msg_cache.origin_node_status_planning_msgs(),
               TOPIC_NODE_STATUS_PLANNING);
      callback(msg_cache.origin_worldmodel_fusion_apa_msgs(),
               MSIM_TOPIC_WORLDMODEL_PARKING_SLOT);
      callback(msg_cache.origin_world_model_scene_objects_msgs(),
               MSIM_TOPIC_WORLDMODEL_SCENE_OBJECT);
    } else {
      callback(msg_cache.origin_module_status_msgs(), TOPIC_MODULE_STATUS);
      callback(msg_cache.origin_chassis_report_msgs(), TOPIC_CHASSIS_REPORT);
      callback(msg_cache.origin_body_report_msgs(), TOPIC_BODY_REPORT);
      callback(msg_cache.origin_perception_fusion_object_result_msgs(),
               TOPIC_FUSION_OBJECTS);
      callback(msg_cache.origin_perception_vision_lane_msgs(),
               TOPIC_PERCEPTION_VISION_LANE);
      callback(msg_cache.origin_perception_vision_landmark_msgs(),
               TOPIC_PERCEPTION_VISION_LANDMARK);
      callback(msg_cache.origin_mla_localization_msgs(), TOPIC_EGO_POSE);
      // apa
      callback(msg_cache.new_sys_planning_request_msgs(),
               TOPIC_PLANNING_REQUEST);
      callback(msg_cache.new_node_status_apa_planning_msgs(),
               TOPIC_NODE_STATUS_APA_PLANNING);
      callback(msg_cache.new_node_status_planning_msgs(),
               TOPIC_NODE_STATUS_PLANNING);
      callback(msg_cache.new_worldmodel_fusion_apa_msgs(),
               MSIM_TOPIC_WORLDMODEL_PARKING_SLOT);
      callback(msg_cache.new_world_model_scene_objects_msgs(),
               MSIM_TOPIC_WORLDMODEL_SCENE_OBJECT);
    }
  }

private:
  bool running = true;
  mfr::MFRNodeMachine &machine;
  mmemory::MFSharedPtr<FPPNode> fpp_node;
  const char *param_yaml() {
    std::stringstream ss;
    ss << "node_config:\n";
    ss << "  memory:\n";
    ss << "    total_virtual_memory_size_MB: 0\n";
    ss << "    total_shared_memory_size_MB: 0\n";
    return ss.str().c_str();
  }
  const char *machine_param_yaml() {
    std::stringstream mss;
    mss << "log:\n";
    mss << "  level: info\n";
    mss << "  enable_stderr: true\n";
    mss << "  export_frequence: 0\n";
    return mss.str().c_str();
  }

  std::queue<maf_mla_localization::MLALocalization> q;
  std::mutex m;
  std::condition_variable cv;
};
} // namespace fpp