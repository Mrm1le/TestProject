#pragma once

#include "common/search_based_planning_task.hpp"
#include "maf_message_cache.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/search_based_planner.h"
#include "src/highway_worldmodel/include/worldmodel/fusion_task.hpp"
#include "src/planning/include/common/parking_planner_types.h"
#include "src/planning/include/common/planning_task_interface.h"
#include "worldmodel/ddmap_generator_task.hpp"
#include "worldmodel/fusion_task.hpp"
#include <maf_interface/maf_perception_interface.h>
#include "worldmodel_flowchart.h"
namespace msquare {

class PlanningTaskOnRunning {
public:
  PlanningTaskOnRunning(bool is_apa);
  ~PlanningTaskOnRunning();

  void set_planning_config();
  void init();

  maf::StatusManager *status_manager() { return status_manager_.get(); }

  msquare::PlanningTaskInterface *mutable_planning_task() {
    return planning_task_.get();
  }
  msquare::PlanningTaskInterface *mutable_parking_planning_task() {
    return parking_planning_task_.get();
  }
  msd_worldmodel::worldmodel_v1::FusionTask *mutable_fusion_object_task() {
    return fusion_object_task_.get();
  }
  msd_worldmodel::worldmodel_v1::DdmapGeneratorTask *
  mutable_ddmap_generator_task() {
    return ddmap_generator_task_.get();
  }
  msquare::parking::SBPlanningTask *mutable_sbp_planning_task() {
    return sbp_planning_task_.get();
  }
  ::parking::SetPSDFusionTask *mutable_pec_task() { return pec_task_.get(); }

  void set_callback(
      std::function<void(const maf_framework_status::NodeStatus &)> callback);
  maf_framework_status::NodeStatus get_status();

  void set_current_frame_planning_start_time(int64_t start_time_ns) const;

  void feed_scenario(const bool is_parking);

  template <typename MsgFrame>
  void feed_planning_request(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      if (maf_system_manager::SystemCmdTypeEnum::
              PLANNING_HIGHWAY_FUNCTION_MODE == msg.msg().cmd.value) {
        LOG(INFO) << "skip planning_request: "
                  << get_planning_request_name(msg.msg());
        continue;
      }
      LOG(INFO) << "publish planning_request: "
                << get_planning_request_name(msg.msg());
      planning_request_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame> void feed_module_status(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      module_status_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame> void feed_chassis(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      chassis_report_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame> void feed_body(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      LOG(INFO) << "lever_state: "
                << to_string(msg.msg()
                                 .vehicle_light_report.vehicle_light_report_data
                                 .lever_state);
      body_report_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame> void feed_wheel(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      wheel_report_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame> void feed_tfl(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      traffic_light_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame> void feed_egopose(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      ego_pose_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame> void feed_fusion(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      fusion_objects_publisher_->publish(msg.msg_wrap_copy());
    }
  }

  template <typename MsgFrame>
  void feed_fusion_ground_line(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      fusion_ground_lines_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame> void feed_prediction(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      prediction_info_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame> void feed_mla_imu(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      imu_report_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame> void feed_mpc_trajectory(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      mpc_trajectory_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame>
  void feed_road_line_perception(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      perception_vision_lane_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame>
  void feed_lidar_road_edge_perception(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      perception_lidar_road_edge_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame>
  void feed_perception_fusion_object(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      perception_fusion_object_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame>
  void feed_radar_perception_result(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      perception_radar_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame>
  void feed_world_model_scene_objects(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      worldmodel_scene_objects_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame>
  void feed_fusion_uss_ground_lines( // feed in PARKING Mode
      const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      fusion_uss_ground_lines_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame>
  void feed_fusion_uss( // feed in PARKING Mode
      const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      fusion_uss_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame>
  void feed_planning_control_cmd(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      planning_control_cmd_request_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame>
  void feed_planning_reset_request( // no feed in PARKING Mode
      const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      planning_reset_request_publisher_->publish(msg.msg_wrap());

      try {
        const auto &request = msg.msg();
        mjson::Reader reset_json(request.frame_id);
        std::string reset =
            reset_json.get<std::string>("reset", false, "False");

        if (reset == "True") {
          if (ddmap_generator_task_ != nullptr)
            ddmap_generator_task_->reset();
        }
      } catch (const mjson::Exception &e) {
        LOG(ERROR) << "json exception: " << e.what();
      }
    }
  }

  template <typename MsgFrame> void feed_sbp_problem(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      sbp_problem_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame> void feed_sbp_result(const MsgFrame &msgs) {
    for (auto &msg : msgs) {
      sbp_result_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame>
  void feed_fusion_parking_slot(const MsgFrame &msgs) {
    LOG(ERROR) << "feed_fusion_parking_slot 1 ";
    for (auto &msg : msgs) {
      LOG(ERROR) << "feed_fusion_parking_slot 2 ";
      fusion_parking_slot_publisher_->publish(msg.msg_wrap());
    }
  }

  template <typename MsgFrame>
  void feed_world_model_parking_slots(const MsgFrame &msgs) {
    LOG(ERROR) << "feed_fusion_parking_slot 1 ";
    for (auto &msg : msgs) {
      LOG(ERROR) << "feed_fusion_parking_slot 2 ";
      worldmodel_parking_slots_publisher_->publish(msg.msg_wrap());
    }
  }

  auto *mutable_worldmodel_map_receiver() {
    return worldmodel_map_receiver_.get();
  }
  auto *mutable_worldmodel_objects_receiver() {
    return worldmodel_objects_receiver_.get();
  }
  auto *mutable_worldmodel_parking_slot_info_receiver() {
    return worldmodel_parking_slot_info_receiver_.get();
  }

private:
  std::string to_string(const maf_endpoint::LeverStatus &v) {
    switch (v.value) {
    case maf_endpoint::LeverStatus::LEVER_STATE_OFF:
      return "LEVER_STATE_OFF";
    case maf_endpoint::LeverStatus::LEVER_STATE_LEFT:
      return "LEVER_STATE_OFF";
    case maf_endpoint::LeverStatus::LEVER_STATE_RIGHT:
      return "LEVER_STATE_RIGHT";
    case maf_endpoint::LeverStatus::LEVER_STATE_LEFT_INVALID:
      return "LEVER_STATE_LEFT_INVALID";
    case maf_endpoint::LeverStatus::LEVER_STATE_RIGHT_INVALID:
      return "LEVER_STATE_RIGHT_INVALID";
    }
    return "INVALID";
  }

  std::string to_string(const maf_system_manager::FunctionModeEnum &v) {
    switch (v.value) {
    case maf_system_manager::FunctionModeEnum::ACC:
      return "PLANNING_HIGHWAY_FUNCTION_MODE:ACC";
    case maf_system_manager::FunctionModeEnum::CP:
      return "PLANNING_HIGHWAY_FUNCTION_MODE:PILOT";
    case maf_system_manager::FunctionModeEnum::HNP:
      return "PLANNING_HIGHWAY_FUNCTION_MODE:HNP";
    default:
      return "PLANNING_HIGHWAY_FUNCTION_MODE:invalid function mode";
    }
  }

  std::string to_string(const maf_system_manager::DrivingStyleEnum &v) {
    switch (v.value) {
    case maf_system_manager::DrivingStyleEnum::AGGRESIVE:
      return "PLANNING_HIGHWAY_DRIVING_STYLE:DRIVING_STYLE_AGGRESSIVE";
    case maf_system_manager::DrivingStyleEnum::NORMAL:
      return "PLANNING_HIGHWAY_DRIVING_STYLE:DRIVING_STYLE_NORMAL";
    case maf_system_manager::DrivingStyleEnum::CONSERVATIVE:
      return "PLANNING_HIGHWAY_DRIVING_STYLE:DRIVING_STYLE_CONSERVATIVE";
    case 3:
      return "PLANNING_HIGHWAY_DRIVING_STYLE:LANECHANGING_STYLE_AGGRESSIVE";
    case 4:
      return "PLANNING_HIGHWAY_DRIVING_STYLE:LANECHANGING_STYLE_NORMAL";
    case 5:
      return "PLANNING_HIGHWAY_DRIVING_STYLE:LANECHANGING_STYLE_CONSERVATIVE";
    default:
      return "PLANNING_HIGHWAY_DRIVING_STYLE:invalid driving mode";
    }
  }

  std::string to_string(const maf_system_manager::LaneChangeCmd &v) {
    if (v.type.value == maf_system_manager::LaneChangeTypeEnum::INTERACTIVE) {
      switch (v.direction.value) {
      case maf_system_manager::LaneChangeDirectionEnum::LEFT:
        return "PLANNING_HIGHWAY_LANE_CHANGE:INTERACTIVE:LEFT";
      case maf_system_manager::LaneChangeDirectionEnum::RIGHT:
        return "PLANNING_HIGHWAY_LANE_CHANGE:INTERACTIVE:RIGHT";
      default:
        return "PLANNING_HIGHWAY_LANE_CHANGE:INTERACTIVE:LEVER_STATE_OFF";
      }
    } else if (v.type.value ==
               maf_system_manager::LaneChangeTypeEnum::FORBIDDEN) {
      return "PLANNING_HIGHWAY_LANE_CHANGE:FORBIDDEN";
    } else if (v.type.value ==
               maf_system_manager::LaneChangeTypeEnum::REMOVE_FORBIDDEN) {
      return "PLANNING_HIGHWAY_LANE_CHANGE:REMOVE_FORBIDDEN";
    } else if (v.type.value ==
               maf_system_manager::LaneChangeTypeEnum::REMOVE_INTERACTIVE) {
      return "PLANNING_HIGHWAY_LANE_CHANGE:REMOVE_INTERACTIVE";
    } else {
      return "PLANNING_HIGHWAY_LANE_CHANGE";
    }
  }

  std::string to_string(const maf_system_manager::StartStopCmdEnum &v) {
    if (v.value == maf_system_manager::StartStopCmdEnum::START) {
      return "PLANNING_HIGHWAY_START_STOP:START";
    } else if (v.value == maf_system_manager::StartStopCmdEnum::STOP) {
      return "PLANNING_HIGHWAY_START_STOP:STOP";
    } else {
      return "PLANNING_HIGHWAY_START_STOP";
    }
  }

  std::string get_planning_request_name(
      const maf_system_manager::SysPlanningRequest &request) {
    switch (request.cmd.value) {
    case maf_system_manager::SystemCmdTypeEnum::PLANNING_HIGHWAY_FUNCTION_MODE:
      return to_string(request.highway_info.function_mode);
    case maf_system_manager::SystemCmdTypeEnum::PLANNING_HIGHWAY_NAVI_SETTINGS:
      return "PLANNING_HIGHWAY_NAVI_SETTINGS";
    case maf_system_manager::SystemCmdTypeEnum::PLANNING_HIGHWAY_DRIVING_STYLE:
      return to_string(request.highway_info.driving_style);
    case maf_system_manager::SystemCmdTypeEnum::PLANNING_HIGHWAY_LANE_CHANGE:
      return to_string(request.highway_info.lane_change_cmd);
    case maf_system_manager::SystemCmdTypeEnum::PLANNING_HIGHWAY_START_STOP:
      return to_string(request.highway_info.start_stop_cmd);
    default:
      return "PLANNING_UNKNOWN_REQUEST";
    }
  }

  void init_planning_task();
  void init_ddmap_generator_task();
  void init_fusion_object_task();

  MSDPlanningConfig planning_config_{};
  uint8_t running_mode_{maf_system_manager::RunningModeEnum::MANUAL_DRIVING};

#define DECLARE_RESOURCE(type, name)                                           \
  std::shared_ptr<mtaskflow::FlowResource<type>> name##_resource_;
#define DECLARE_PUBLISHER(type, name)                                          \
  mtaskflow::FlowPublisher<type> name##_publisher_;
#define DECLARE_RESOURCE_AND_PUBLISHER(type, name)                             \
  DECLARE_RESOURCE(type, name)                                                 \
  DECLARE_PUBLISHER(type, name)

  DECLARE_RESOURCE_AND_PUBLISHER(uint64_t, tick)
  DECLARE_RESOURCE_AND_PUBLISHER(maf_framework_status::ModuleStatus,
                                 module_status)
  DECLARE_RESOURCE_AND_PUBLISHER(std::shared_ptr<maf_endpoint::ChassisReport>,
                                 chassis_report)
  DECLARE_RESOURCE_AND_PUBLISHER(std::shared_ptr<maf_endpoint::WheelReport>,
                                 wheel_report)
  DECLARE_RESOURCE_AND_PUBLISHER(std::shared_ptr<maf_endpoint::BodyReport>,
                                 body_report)
  DECLARE_RESOURCE_AND_PUBLISHER(std::shared_ptr<maf_gps_imu::MLAImu>,
                                 imu_report)
  DECLARE_RESOURCE_AND_PUBLISHER(std::shared_ptr<maf_worldmodel::ProcessedMap>,
                                 worldmodel_map)
  DECLARE_RESOURCE_AND_PUBLISHER(
      std::shared_ptr<maf_worldmodel::ObjectsInterface>, worldmodel_objects)
  DECLARE_RESOURCE_AND_PUBLISHER(std::shared_ptr<maf_worldmodel::FusionAPA>,
                                 worldmodel_parking_slots)
  DECLARE_RESOURCE_AND_PUBLISHER(std::shared_ptr<maf_worldmodel::SceneObjects>,
                                 worldmodel_scene_objects)
  DECLARE_RESOURCE_AND_PUBLISHER(
      std::shared_ptr<maf_perception_interface::PerceptionFusionObjectResult>,
      fusion_objects)
  DECLARE_RESOURCE_AND_PUBLISHER(
      std::shared_ptr<maf_perception_interface::FusionGroundLineResult>,
      fusion_ground_lines)
  DECLARE_RESOURCE_AND_PUBLISHER(
      std::shared_ptr<maf_perception_interface::FusionGroundLineResult>,
      fusion_uss_ground_lines)
  DECLARE_RESOURCE_AND_PUBLISHER(
      std::shared_ptr<maf_sensor_interface::UltrasonicUpaReport>, fusion_uss)
  DECLARE_RESOURCE_AND_PUBLISHER(
      std::shared_ptr<maf_perception_interface::TrafficLightPerception>,
      traffic_light)
  DECLARE_RESOURCE_AND_PUBLISHER(
      std::shared_ptr<maf_mla_localization::MLALocalization>, ego_pose)
  DECLARE_RESOURCE_AND_PUBLISHER(
      std::shared_ptr<maf_worldmodel::PredictionResult>, prediction_info)
  DECLARE_RESOURCE_AND_PUBLISHER(
      std::shared_ptr<maf_planning::MpcTrajectoryResult>, mpc_trajectory)
  DECLARE_RESOURCE_AND_PUBLISHER(maf_system_manager::ModuleControlCmdRequest,
                                 planning_control_cmd_request)
  DECLARE_RESOURCE_AND_PUBLISHER(maf_system_manager::SysPlanningRequest,
                                 planning_request)
  DECLARE_RESOURCE_AND_PUBLISHER(maf_std::Header, planning_reset_request)
  DECLARE_RESOURCE_AND_PUBLISHER(uint64_t, sequence)
  DECLARE_RESOURCE_AND_PUBLISHER(
      std::shared_ptr<maf_perception_interface::RoadLinePerception>,
      perception_vision_lane)
  DECLARE_RESOURCE_AND_PUBLISHER(
      std::shared_ptr<maf_perception_interface::RoadLinePerception>,
      perception_lidar_road_edge)
  DECLARE_RESOURCE_AND_PUBLISHER(
      std::shared_ptr<maf_perception_interface::PerceptionFusionObjectResult>,
      perception_fusion_object)
  DECLARE_RESOURCE_AND_PUBLISHER(
      std::shared_ptr<maf_perception_interface::RadarPerceptionResult>,
      perception_radar)
  DECLARE_RESOURCE_AND_PUBLISHER(std::shared_ptr<maf_std::Header>, mff_info)
  DECLARE_RESOURCE_AND_PUBLISHER(
      std::shared_ptr<maf_perception_interface::FusionParkingSlotResult>,
      fusion_parking_slot)
  DECLARE_RESOURCE_AND_PUBLISHER(msquare::parking::OpenspaceDeciderOutput,
                                 sbp_problem)
  DECLARE_RESOURCE_AND_PUBLISHER(msquare::SbpResult, sbp_result)
  DECLARE_RESOURCE_AND_PUBLISHER(std::string, sbp_debug)
  DECLARE_RESOURCE_AND_PUBLISHER(
      std::shared_ptr<maf_endpoint::WirelessChargerReport>,
      wireless_charger_report)

#undef DECLARE_RESOURCE_AND_PUBLISHER
#undef DECLARE_PUBLISHER
#undef DECLARE_RESOURCE

  bool is_apa_{false};
  mtaskflow::FlowReceiver<std::shared_ptr<maf_worldmodel::ProcessedMap>>
      worldmodel_map_receiver_{};
  mtaskflow::FlowReceiver<std::shared_ptr<maf_worldmodel::ObjectsInterface>>
      worldmodel_objects_receiver_{};
  mtaskflow::FlowReceiver<std::shared_ptr<maf_worldmodel::FusionAPA>>
      worldmodel_parking_slot_info_receiver_{};

  std::shared_ptr<maf::StatusManager> status_manager_;

  std::shared_ptr<PlanningTaskInterface> planning_task_{};
  std::shared_ptr<PlanningTaskInterface> parking_planning_task_{};
  std::shared_ptr<msquare::parking::SBPlanningTask> sbp_planning_task_{};
  std::shared_ptr<::parking::SetPSDFusionTask> pec_task_{};
  std::shared_ptr<msd_worldmodel::worldmodel_v1::DdmapGeneratorTask>
      ddmap_generator_task_{};
  std::shared_ptr<msd_worldmodel::worldmodel_v1::FusionTask>
      fusion_object_task_{};
};

} // namespace msquare
