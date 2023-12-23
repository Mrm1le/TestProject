#pragma once

// #include "common/pnc_trigger.h"
// #include "common/collision_check.h"
#include "common/math/math_utils.h"
#include "common/apa_workflow/apa_state_machine.h"
#include "common/parking_world_model.h"
#include "common/search_based_planning_utils.h"
#include "common/static_check.h"
#include "common/utils/polyfit.h"
#include "mtaskflow/mtaskflow.hpp"
#include "common/planning_task_interface.h"
#include "rapidjson/document.h" // rapidjson's DOM-style API
#include <iostream>

using namespace msd_planning;
using namespace maf_framework_status;
using namespace maf_worldmodel;
using namespace maf_perception_interface;
using namespace maf_mla_localization;
using namespace maf_vehicle_status;
using namespace maf_endpoint;

namespace msquare {
namespace parking {

class PlanningTask : public PlanningTaskInterface {
public:
  PlanningTask(
      MSDPlanningConfig planning_config, bool enable_timer_tick,
      std::shared_ptr<maf::StatusManager> monitor,
      mtaskflow::FlowReceiver<uint64_t> tick_receiver,
      mtaskflow::FlowReceiver<ModuleStatus> module_status_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<maf_endpoint::ChassisReport>>
          chassis_report_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<maf_endpoint::WheelReport>>
          wheel_report_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<maf_endpoint::BodyReport>>
          body_report_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<maf_gps_imu::MLAImu>>
          imu_report_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<maf_worldmodel::ProcessedMap>>
          worldmodel_map_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<maf_worldmodel::ObjectsInterface>>
          worldmodel_objects_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<maf_worldmodel::FusionAPA>>
          worldmodel_parking_slots_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<maf_worldmodel::SceneObjects>>
          worldmodel_scene_objects_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<
          maf_perception_interface::PerceptionFusionObjectResult>>
          fusion_objects_receiver,
      mtaskflow::FlowReceiver<
          std::shared_ptr<maf_perception_interface::FusionGroundLineResult>>
          fusion_groundlines_receiver,
      mtaskflow::FlowReceiver<
          std::shared_ptr<maf_perception_interface::FusionGroundLineResult>>
          fusion_uss_groundlines_receiver,
      mtaskflow::FlowReceiver<
          std::shared_ptr<maf_sensor_interface::UltrasonicUpaReport>>
          fusion_uss_receiver,
      mtaskflow::FlowReceiver<
          std::shared_ptr<maf_perception_interface::TrafficLightPerception>>
          traffic_light_receiver,
      mtaskflow::FlowReceiver<
          std::shared_ptr<maf_mla_localization::MLALocalization>>
          ego_pose_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<maf_worldmodel::PredictionResult>>
          prediction_result_receiver,
      mtaskflow::FlowReceiver<
          std::shared_ptr<maf_planning::MpcTrajectoryResult>>
          mpc_trajectory_receiver,
      mtaskflow::FlowReceiver<maf_system_manager::ModuleControlCmdRequest>
          planning_control_cmd_request_receiver,
      mtaskflow::FlowReceiver<maf_system_manager::SysPlanningRequest>
          planning_request_receiver,
      mtaskflow::FlowPublisher<parking::OpenspaceDeciderOutput>
          sbp_problem_publisher,
      mtaskflow::FlowReceiver<SbpResult> sbp_result_receiver,
      mtaskflow::FlowReceiver<std::string> sbp_debug_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<maf_endpoint::WirelessChargerReport>>
          wireless_charger_report_recv)
      : enable_timer_tick_(enable_timer_tick), monitor_(monitor),
        tick_receiver_(tick_receiver),
        module_status_receiver_(module_status_receiver),
        chassis_report_receiver_(chassis_report_receiver),
        wheel_report_receiver_(wheel_report_receiver),
        body_report_receiver_(body_report_receiver),
        imu_report_receiver_(imu_report_receiver),
        worldmodel_map_receiver_(worldmodel_map_receiver),
        worldmodel_objects_receiver_(worldmodel_objects_receiver),
        worldmodel_parking_slots_receiver_(worldmodel_parking_slots_receiver),
        worldmodel_scene_objects_receiver_(worldmodel_scene_objects_receiver),
        fusion_objects_receiver_(fusion_objects_receiver),
        fusion_groundlines_receiver_(fusion_groundlines_receiver),
        fusion_uss_groundlines_receiver_(fusion_uss_groundlines_receiver),
        fusion_uss_receiver_(fusion_uss_receiver),
        traffic_light_receiver_(traffic_light_receiver),
        ego_pose_receiver_(ego_pose_receiver),
        prediction_result_receiver_(prediction_result_receiver),
        mpc_trajectory_receiver_(mpc_trajectory_receiver),
        planning_control_cmd_request_receiver_(
            planning_control_cmd_request_receiver),
        planning_request_receiver_(planning_request_receiver),
        sbp_problem_publisher_(sbp_problem_publisher),
        sbp_result_receiver_(sbp_result_receiver),
        sbp_debug_receiver_(sbp_debug_receiver),
        wireless_charger_report_recv_(wireless_charger_report_recv) {
    module_control_cmd_request_.module_control_cmd.value =
        maf_system_manager::ModuleControlCmdEnum::PAUSE;

    world_model_ = std::make_shared<WorldModel>();
    (void)world_model_->init(planning_config.scene_type);

    apa_state_machine_ = std::make_shared<ApaStateMachine>(world_model_);

    cruise_velocity_ = planning_config.cruise_velocity;
    is_in_simulation_ = is_in_simulation();
    if (is_in_simulation_) {
      _runner = std::bind(&PlanningTask::_run_in_simulation, this);
    } else {
      _runner = std::bind(&PlanningTask::_run_in_reality, this);
    }
  }

  maf_planning::SBPRequest
  gen_sbp_request(const OpenspaceDeciderOutput &problem,
                  std::string problem_id);
  void
  fill_openspace_motion_planner_output(const OpenspaceDeciderOutput &problem,
                                       OpenspaceMotionPlannerOutput *ompo);

  void on_running();

  void set_callback(MSDPlanningOutputCallback callback) {
    planning_output_callback_ = callback;
  }

  void set_callback(MSDPlanningLdpOutputCallback callback) {
    // not used
  }

  void set_sync_callback(NPPHeaderLoggerCallback callback) {
    publish_simulation_sync_ = callback;
  }

  void set_callback(MSDPlanningTriggerCallback callback) {
    planning_trigger_callback_ = callback;
  }

  void
  set_callback(std::function<void(const maf_planning::SBPRequest &sbp_request)>
                   &callback) {
    sbp_request_cb_ = callback;
  }

  void set_callback(MSDPlanningInfoCallback callback) {
    planning_info_callback_ = callback;
  }

  void set_callback(MSDMdebugCallback callback) {
    // not used
  }

  void reset() {
    reset_ = true;
    feed_stop_request();
  }

private:
  bool run_once();

  void clear_all_recievers() {
    module_status_receiver_->clear();
    chassis_report_receiver_->clear();
    wheel_report_receiver_->clear();
    body_report_receiver_->clear();
    imu_report_receiver_->clear();
    worldmodel_map_receiver_->clear();
    worldmodel_objects_receiver_->clear();
    worldmodel_parking_slots_receiver_->clear();
    worldmodel_scene_objects_receiver_->clear();
    fusion_objects_receiver_->clear();
    fusion_groundlines_receiver_->clear();
    fusion_uss_groundlines_receiver_->clear();
    fusion_uss_receiver_->clear();
    traffic_light_receiver_->clear();
    ego_pose_receiver_->clear();
    prediction_result_receiver_->clear();
    mpc_trajectory_receiver_->clear();
    planning_request_receiver_->clear();
    sbp_result_receiver_->clear();
    sbp_debug_receiver_->clear();
    wireless_charger_report_recv_->clear();
    planning_seq_ = 0;
  }

  void feed_stop_request() {
    MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
    PlanningRequest planning_request;
    planning_request.cmd.value = ParkingCommand::STOP;
    planning_request.id = 0;
    planning_request.wireless_charge = false;
    planning_request.park_out_direction.value =
        ParkOutDirectionType::AUTO_SELECT;
    world_model_->feed_planning_request(planning_request);
    world_model_->feed_pause_status(false);
    world_model_->feed_force_p_gear(false);
    world_model_->feed_hard_brake(false);
  }

  void update_planning_request(double current_time) {
    MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
    while (!planning_request_receiver_->empty()) {
      MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
      maf_system_manager::SysPlanningRequest request{};
      auto ret = planning_request_receiver_->pop_oldest(request);
      if (!ret) {
        continue;
      }
      MSD_LOG(ERROR, "no_react request.cmd.value = %d", (int)request.cmd.value);
      MSD_LOG(ERROR, "no_react request.parking_info.mode.value = %d",
              (int)request.parking_info.mode.value);
      MSD_LOG(ERROR, "no_react request.parking_info.id = %d",
              (int)request.parking_info.id);
      if (request.cmd.value ==
          maf_system_manager::SystemCmdTypeEnum::PLANNING_CHANGE_MODE) {
        MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
        SceneType scene_type;
        switch (request.parking_info.mode.value) {
        case maf_system_manager::SysPlanningModeTypeForParking::PLANNING_SVP:
          scene_type = SceneType::PARKING_SVP;
          break;
        case maf_system_manager::SysPlanningModeTypeForParking::PLANNING_LVP:
          scene_type = SceneType::PARKING_LVP;
          break;
        case maf_system_manager::SysPlanningModeTypeForParking::PLANNING_APA:
        default:
          scene_type = SceneType::PARKING_APA;
          break;
        }
        world_model_->change_scene_type(scene_type);
        PlanningContext::Instance()
            ->mutable_parking_behavior_planner_output()
            ->has_planned = false;
        PlanningContext::Instance()
            ->mutable_planning_status()
            ->task_status.status = TaskStatusType::RUNNING;
      } else if (request.cmd.value ==
                 maf_system_manager::SystemCmdTypeEnum::PLANNING_PAUSE) {
        world_model_->feed_pause_status(true);
      } else if (request.cmd.value == 655383) {
        world_model_->feed_force_p_gear(true);
        world_model_->feed_hard_brake(true);
      } else if (request.cmd.value ==
                 maf_system_manager::SystemCmdTypeEnum::PLANNING_CONTINUE) {
        world_model_->feed_pause_status(false);
        world_model_->feed_force_p_gear(false);
        world_model_->feed_hard_brake(false);
      } else if (request.cmd.value == maf_system_manager::SystemCmdTypeEnum::
                                          PLANNING_EMERGENCY_PAUSE) {
        world_model_->feed_pause_status(true);
        world_model_->feed_hard_brake(true);
        world_model_->feed_force_p_gear(false);
      } else {
        MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
        PlanningRequest planning_request;
        switch (request.cmd.value) {
        case maf_system_manager::SystemCmdTypeEnum::PLANNING_RANDOM_PARKIN:
          planning_request.cmd.value = ParkingCommand::RANDOM_PARKIN;
          world_model_->feed_pause_status(false);
          world_model_->feed_force_p_gear(false);
          world_model_->feed_hard_brake(false);
          reschedule("mode_running");
          break;
        case maf_system_manager::SystemCmdTypeEnum::PLANNING_DESIGNATE_PARKIN:
          planning_request.cmd.value = ParkingCommand::DESIGNATE_PARKIN;
          MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
          world_model_->feed_pause_status(false);
          world_model_->feed_force_p_gear(false);
          world_model_->feed_hard_brake(false);
          reschedule("mode_running");
          break;
        case maf_system_manager::SystemCmdTypeEnum::PLANNING_PARKOUT:
          planning_request.cmd.value = ParkingCommand::PARK_OUT;
          world_model_->feed_pause_status(false);
          world_model_->feed_force_p_gear(false);
          world_model_->feed_hard_brake(false);
          reschedule("mode_running");
          break;
        case maf_system_manager::SystemCmdTypeEnum::PLANNING_EXIT:
          planning_request.cmd.value = ParkingCommand::EXIT;
          world_model_->feed_pause_status(false);
          world_model_->feed_force_p_gear(false);
          world_model_->feed_hard_brake(false);
          reschedule("mode_idle");
          break;
        case maf_system_manager::SystemCmdTypeEnum::PLANNING_STOP:
          planning_request.cmd.value = ParkingCommand::STOP;
          world_model_->feed_pause_status(false);
          world_model_->feed_force_p_gear(false);
          world_model_->feed_hard_brake(false);
          reschedule("mode_running");
          break;
        case maf_system_manager::SystemCmdTypeEnum::PLANNING_EMERGENCY_STOP:
          planning_request.cmd.value = ParkingCommand::STOP;
          world_model_->feed_pause_status(false);
          world_model_->feed_force_p_gear(false);
          world_model_->feed_hard_brake(true);
          reschedule("mode_running");
          break;
        case maf_system_manager::SystemCmdTypeEnum::PLANNING_PARKOUT_STANDBY:
          planning_request.cmd.value = ParkingCommand::PARK_OUT_STANDBY;
          world_model_->feed_pause_status(false);
          world_model_->feed_force_p_gear(false);
          world_model_->feed_hard_brake(false);
          reschedule("mode_running");
          break;
        case 655381: // rpa straight standby
          planning_request.cmd.value = ParkingCommand::RPA_STRAIGHT_STANDBY;
          world_model_->feed_pause_status(false);
          PlanningContext::Instance()
              ->mutable_planning_status()
              ->task_status.status = TaskStatusType::RUNNING;
          reschedule("mode_running");
          break;
        case 655379: // rpa forward
          planning_request.cmd.value = ParkingCommand::RPA_STRAIGHT;
          planning_request.rpa_straight_direction.value =
              RpaStraightDirectionType::RPA_STRAIGHT_FORWARD;
          reschedule("mode_running");
          world_model_->feed_pause_status(false);
          break;
        case 655380: // rpa backward
          planning_request.cmd.value = ParkingCommand::RPA_STRAIGHT;
          planning_request.rpa_straight_direction.value =
              RpaStraightDirectionType::RPA_STRAIGHT_BACKWARD;
          reschedule("mode_running");
          world_model_->feed_pause_status(false);
          break;
        default:
          planning_request.cmd.value = ParkingCommand::NONE;
          reschedule("mode_idle");
          break;
        }
        planning_request.id = request.parking_info.id;
        planning_request.wireless_charge = request.parking_info.wireless_charge;
        planning_request.park_out_direction.value =
            request.parking_info.parking_out_direction.value;
        // switch (request.parking_info.parking_out_direction.value) {
        // case maf_system_manager::ParkOutDirectionType::
        //     FRONT_SIDE_LEFT_DIRECTION_TYPE:
        //   planning_request.park_out_direction.value =
        //       ParkOutDirectionType::FRONT_LEFT;
        //   break;
        // case maf_system_manager::ParkOutDirectionType::
        //     FRONT_SIDE_RIGHT_DIRECTION_TYPE:
        //   planning_request.park_out_direction.value =
        //       ParkOutDirectionType::FRONT_RIGHT;
        //   break;
        // case maf_system_manager::ParkOutDirectionType::
        //     LEFT_SIDE_FRONT_DIRECTION_TYPE:
        //   planning_request.park_out_direction.value =
        //       ParkOutDirectionType::LEFT_FRONT;
        //   break;
        // case maf_system_manager::ParkOutDirectionType::
        //     RIGHT_SIDE_FRONT_DIRECTION_TYPE:
        //   planning_request.park_out_direction.value =
        //       ParkOutDirectionType::RIGHT_FRONT;
        //   break;
        // case maf_system_manager::ParkOutDirectionType::
        //     FRONT_SIDE_FRONT_DIRECTION_TYPE:
        //   planning_request.park_out_direction.value =
        //       ParkOutDirectionType::FRONT;
        //   break;
        // case maf_system_manager::ParkOutDirectionType::
        //     REAR_SIDE_REAR_DIRECTION_TYPE:
        //   planning_request.park_out_direction.value =
        //       ParkOutDirectionType::REAR;
        //   break;
        // case maf_system_manager::ParkOutDirectionType::
        //     AUTO_SELECT_DIRECTION_TYPE:
        // default:
        //   planning_request.park_out_direction.value =
        //       ParkOutDirectionType::AUTO_SELECT;
        //   break;
        // }
        const uint32_t IN_VALID_CMD_REQUEST = maf_system_manager::
            SystemCmdTypeEnum::PLANNING_HIGHWAY_FUNCTION_MODE;
        if (request.cmd.value == IN_VALID_CMD_REQUEST) {
          reschedule("mode_idle");
          MSD_LOG(ERROR, "%s::%s: %d\n", __FILE__, __FUNCTION__, __LINE__);
          continue;
        }
        MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
        world_model_->feed_planning_request(planning_request);
      }
    }
  }

  void update_chassis_report(double current_time) {
    while (!chassis_report_receiver_->empty()) {
      std::shared_ptr<maf_endpoint::ChassisReport> received_chassis_report{};
      auto ret = chassis_report_receiver_->pop_oldest(received_chassis_report);
      if (!ret) {
        continue;
      }

      if (received_chassis_report->steering_report.available &
          SteeringReport::STEERING_REPORT_DATA) {
        auto &steering_wheel = vehicle_status_.steering_wheel;
        steering_wheel.available |= SteeringWheel::STEERING_WHEEL_DATA;
        steering_wheel.steering_wheel_data.steering_wheel_rad =
            received_chassis_report->steering_report.steering_report_data
                .steering_wheel_angle_report;
        last_feed_time_[FEED_EGO_STEER_ANGLE] = current_time;
      }

      if (received_chassis_report->brake_report.available &
          BrakeReport::BRAKE_REPORT_DATA) {
        auto &brake = vehicle_status_.brake;
        brake.available |= Brake::BRAKE_DATA;
        brake.brake_data.override =
            received_chassis_report->brake_report.brake_report_data.override;
      }

      if (received_chassis_report->brake_info_report.available &
          BrakeInfoReport::BRAKE_INFO_REPORT_DATA) {
        auto &brake_info = vehicle_status_.brake_info;
        brake_info.available |= BrakeInfo::BRAKE_INFO_DATA;
        brake_info.brake_info_data.acceleration_on_vehicle_wheel =
            received_chassis_report->brake_info_report.brake_info_report_data
                .accleration_on_wheel;
        brake_info.brake_info_data.stationary =
            received_chassis_report->brake_info_report.brake_info_report_data
                .stationary;
        last_feed_time_[FEED_EGO_ACC] = current_time;
      }

      if (received_chassis_report->gear_report.available &
          GearReport::GEAR_REPORT_DATA) {
        vehicle_status_.gear.available |= maf_vehicle_status::Gear::GEAR_DATA;
        vehicle_status_.gear.gear_data.gear_status.value =
            received_chassis_report->gear_report.gear_report_data.current_state
                .value;
        last_feed_time_[FEED_GEAR_REPORT] = current_time;
      }
    }
  }

  void update_wheel_report(double current_time) {
    // bool cruise_control_set_increase = false;
    // bool cruise_control_set_decrease = false;

    while (!wheel_report_receiver_->empty()) {
      std::shared_ptr<maf_endpoint::WheelReport> received_wheel_report{};
      auto ret = wheel_report_receiver_->pop_oldest(received_wheel_report);
      if (!ret) {
        continue;
      }

      if (received_wheel_report->wheel_speed_report.available &
          WheelSpeedReport::WHEEL_SPEED_REPORT_DATA) {
        auto &wheel_velocity = vehicle_status_.wheel_velocity;
        auto &wheel_speed_report_data =
            received_wheel_report->wheel_speed_report.wheel_speed_report_data;
        wheel_velocity.available |= WheelVelocity::WHEEL_VELOCITY4D;

        wheel_velocity.wheel_velocity4d.front_left =
            wheel_speed_report_data.front_left;
        wheel_velocity.wheel_velocity4d.front_right =
            wheel_speed_report_data.front_right;
        wheel_velocity.wheel_velocity4d.rear_left =
            wheel_speed_report_data.rear_left;
        wheel_velocity.wheel_velocity4d.rear_right =
            wheel_speed_report_data.rear_right;

        last_feed_time_[FEED_WHEEL_SPEED_REPORT] = current_time;
      }
      if (received_wheel_report->wheel_position_report.available &
          WheelPositionReport::WHEEL_POSITION_REPORT_DATA) {
        auto &wheel_position_report_data =
            received_wheel_report->wheel_position_report
                .wheel_position_report_data;

        static_check_.UpdateWheelPositionData(
            wheel_position_report_data.front_left,
            wheel_position_report_data.front_right,
            wheel_position_report_data.rear_left,
            wheel_position_report_data.rear_right);
      }
    }
  }

  void update_body_report(double current_time) {
    bool cruise_control_set_increase = false;
    bool cruise_control_set_decrease = false;

    while (!body_report_receiver_->empty()) {
      std::shared_ptr<maf_endpoint::BodyReport> received_body_report{};
      auto ret = body_report_receiver_->pop_oldest(received_body_report);
      if (!ret) {
        continue;
      }

      if (received_body_report->vehicle_light_report.available &
          VehicleLightReport::VEHICLE_LIGHT_REPORT_DATA) {
        vehicle_status_.vehicle_light.available |=
            VehicleLight::VEHICLE_LIGHT_DATA;
        auto &vehicle_light_data =
            vehicle_status_.vehicle_light.vehicle_light_data;
        auto &vehicle_light_report_data =
            received_body_report->vehicle_light_report
                .vehicle_light_report_data;
        vehicle_light_data.turn_signal.value =
            vehicle_light_report_data.lever_state.value;
        // }
        last_feed_time_[FEED_MISC_REPORT] = current_time;
      }

      // if (received_body_report->botton_event_report.available &
      //     BottonEventReport::BOTTON_EVENT_REPORT_DATA) {
      //   const auto button_events =
      //       received_body_report->botton_event_report.botton_event_report_data
      //           .botton_event_array;
      //   ;
      //   auto set_increase = std::find_if(
      //       button_events.begin(), button_events.end(),
      //       [](const BottonEvent &data) {
      //         return data.value == BottonEvent::CRUISE_CONTROL_SET_INCREASE;
      //       });
      //   if (set_increase != button_events.end()) {
      //     cruise_control_set_increase = true;
      //   }
      //   auto set_decrease = std::find_if(
      //       button_events.begin(), button_events.end(),
      //       [](const BottonEvent &data) {
      //         return data.value == BottonEvent::CRUISE_CONTROL_SET_DECREASE;
      //       });
      //   if (set_decrease != button_events.end()) {
      //     cruise_control_set_decrease = true;
      //   }
      // }
    }

    // update_cruise_velocity(cruise_control_set_increase,
    // cruise_control_set_decrease);

    vehicle_status_.velocity.available |=
        maf_vehicle_status::Velocity::CRUISE_VELOCITY;
    vehicle_status_.velocity.cruise_velocity.value_mps = cruise_velocity_ / 3.6;
  }

  void update_imu_report(double current_time) {
    imu_report_receiver_->clear();
    last_feed_time_[FEED_SENSOR_IMU] = current_time;
  }

  void update_cruise_velocity(bool set_increase, bool set_decrease) {
    double max_cruise_velocity = 120.0;
    double cruise_step = 2.0;

    if (set_increase && !lase_cruise_control_set_increase_) {
      cruise_velocity_ += cruise_step;
    }
    if (set_decrease && !last_cruise_control_set_decrease_) {
      cruise_velocity_ -= cruise_step;
    }

    cruise_velocity_ = clip(cruise_velocity_, max_cruise_velocity, 0.0);

    lase_cruise_control_set_increase_ = set_increase;
    last_cruise_control_set_decrease_ = set_decrease;
  }

  void update_ego_pose(double current_time) {
    uint8_t type = 0;
    bool status = false;
    while (!ego_pose_receiver_->empty()) {
      std::shared_ptr<maf_mla_localization::MLALocalization> mla_localization{};
      auto ret = ego_pose_receiver_->pop_oldest(mla_localization);
      if (!ret) {
        continue;
      }
      localization_seq_ = mla_localization->header.seq;
      if (mla_localization->velocity.available & MLAVelocity::MLA_VEL_LOCAL) {
        vehicle_status_.velocity.available |=
            maf_vehicle_status::Velocity::HEADING_VELOCITY;
        vehicle_status_.velocity.heading_velocity.value_mps =
            std::hypot(mla_localization->velocity.velocity_local.vx,
                       mla_localization->velocity.velocity_local.vy);

        ego_pose_timestamp_us_ = mla_localization->meta.timestamp_us;
        last_feed_time_[FEED_EGO_VEL] = current_time;
      }

      if ((mla_localization->orientation.available &
           MLAOrientation::MLA_EULER_LOCAL) &&
          (mla_localization->position.available &
           MLAPosition::MLA_POSITION_LOCAL)) {
        vehicle_status_.location.available |= Location::LOCATION_ENU;

        type = mla_localization->status.status_info.type;
        auto &location_enu = vehicle_status_.location.location_enu;

        vehicle_status_.heading_yaw.heading_yaw_data.value_rad =
            mla_localization->orientation.euler_local.yaw;

        location_enu.x = mla_localization->position.position_local.x;
        location_enu.y = mla_localization->position.position_local.y;
        location_enu.z = mla_localization->position.position_local.z;

        location_enu.orientation.x =
            mla_localization->orientation.quaternion_local.x;
        location_enu.orientation.y =
            mla_localization->orientation.quaternion_local.y;
        location_enu.orientation.z =
            mla_localization->orientation.quaternion_local.z;
        location_enu.orientation.w =
            mla_localization->orientation.quaternion_local.w;

        ego_pose_timestamp_us_ = mla_localization->meta.timestamp_us;
        last_feed_time_[FEED_EGO_ENU] = current_time;
        MSD_LOG(INFO, "ENU_DEBUG,Time:%3.5f", MTIME()->timestamp().sec());
        MSD_LOG(INFO, "ENU_DEBUG,enu.x:%3.2f,enu.y:%3.2f,enu.z:%3.2f",
                location_enu.x, location_enu.y, location_enu.z);
        MSD_LOG(
            INFO,
            "ENU_DEBUG,orien.x:%3.3f,orien.y:%3.3f,orien.z:%3.3f,orien.w:%3.3f",
            location_enu.orientation.x, location_enu.orientation.y,
            location_enu.orientation.z, location_enu.orientation.w);
        MSD_LOG(INFO, "ENU_DEBUG,enu.yaw:%3.5f",
                mla_localization->orientation.euler_local.yaw);
      }
    }

    if ((vehicle_status_.location.available & Location::LOCATION_ENU)) {
      // localization
      ::Quaternion orientation{
          vehicle_status_.location.location_enu.orientation.x,
          vehicle_status_.location.location_enu.orientation.y,
          vehicle_status_.location.location_enu.orientation.z,
          vehicle_status_.location.location_enu.orientation.w};

      Point3D position{vehicle_status_.location.location_enu.x,
                       vehicle_status_.location.location_enu.y,
                       vehicle_status_.location.location_enu.z};

      Pose3D enu{position, orientation};

      double yaw = std::atan2(
          2 * (orientation.w * orientation.z + orientation.x * orientation.y),
          1 - 2 * (orientation.y * orientation.y +
                   orientation.z * orientation.z));

      Pose2D pose{position.x, position.y, yaw};

      //!< type: current egopose mode
      //!< 0 - error type, egopose invalid
      //!< 1 - egomotion mode, hdmap not available
      //!< 2 - localization mode, hdmap available
      if (world_model_->is_parking_apa() ||
          world_model_->is_parking_lvp() && type != 0) {
        status = true;
      } else if ((false && world_model_->is_parking_lvp() ||
                  world_model_->is_parking_svp()) &&
                 type == 2) {
        status = true;
      }
      world_model_->feed_localization_status(status);
      world_model_->feed_ego_pose(pose);
      world_model_->feed_ego_enu(enu);

      last_feed_time_[FEED_EGO_ENU] = current_time;
    }
  }

  void update_vehicle_status(double current_time) {

    update_chassis_report(current_time);

    update_wheel_report(current_time);

    update_body_report(current_time);

    update_ego_pose(current_time);

    if (vehicle_status_.wheel_velocity.available &
        maf_vehicle_status::WheelVelocity::WHEEL_VELOCITY4D) {
      auto &wheel_velocity4d = vehicle_status_.wheel_velocity.wheel_velocity4d;
      float velocity_mps =
          0.339 *
          (wheel_velocity4d.front_left + wheel_velocity4d.front_right +
           wheel_velocity4d.rear_left + wheel_velocity4d.rear_right) /
          4.0;
      // collision_check_.update_velocity(velocity_mps);
    }

    if (/*vehicle_status_.location.available & Location::LOCATION_ENU &&
        vehicle_status_.velocity.available &
            maf_vehicle_status::Velocity::HEADING_VELOCITY && */
        vehicle_status_.steering_wheel.available &
            SteeringWheel::STEERING_WHEEL_DATA &&
        vehicle_status_.brake_info.available & BrakeInfo::BRAKE_INFO_DATA &&
        vehicle_status_.brake.available & Brake::BRAKE_DATA &&
        vehicle_status_.wheel_velocity.available &
            maf_vehicle_status::WheelVelocity::WHEEL_VELOCITY4D &&
        vehicle_status_.vehicle_light.available &
            VehicleLight::VEHICLE_LIGHT_DATA &&
        vehicle_status_.gear.available & maf_vehicle_status::Gear::GEAR_DATA &&
        vehicle_status_.velocity.available &
            maf_vehicle_status::Velocity::CRUISE_VELOCITY) {
      // world_model_->feed_vehicle_status(vehicle_status_);
      // vehicle status
      world_model_->feed_gear_report(vehicle_status_.gear.gear_data);
      world_model_->feed_brake_override(
          vehicle_status_.brake.brake_data.override);
      world_model_->feed_vehicle_light_report(
          vehicle_status_.vehicle_light.vehicle_light_data);
      bool stationary = false;
      if (VehicleParam::Instance()->car_type == "L7") {
        stationary = vehicle_status_.brake_info.brake_info_data.stationary;
      } else {
        stationary = static_check_.get_static_status();
      }
      world_model_->feed_wheel_speed_report(
          vehicle_status_.wheel_velocity.wheel_velocity4d, stationary);
      world_model_->feed_ego_steer_angle(
          vehicle_status_.steering_wheel.steering_wheel_data
              .steering_wheel_rad);
      world_model_->feed_ego_acc(vehicle_status_.brake_info.brake_info_data
                                     .acceleration_on_vehicle_wheel);
      last_feed_time_[FEED_EGO_VEL] = current_time;
    }
  }

  void fill_refline_points(
      const std::vector<maf_worldmodel::ReferenceLinePoint> &input,
      std::vector<RefLinePoint> &refline_points) {
    refline_points.clear();
    refline_points.resize(input.size());
    for (size_t i = 0; i < input.size(); i++) {
      refline_points[i].x = input[i].enu_point.x;
      refline_points[i].y = input[i].enu_point.y;
      refline_points[i].left_lane_border_distance =
          input[i].distance_to_left_lane_border;
      refline_points[i].right_lane_border_distance =
          input[i].distance_to_right_lane_border;
      refline_points[i].left_road_border_distance =
          input[i].distance_to_left_road_border;
      refline_points[i].right_road_border_distance =
          input[i].distance_to_right_road_border;
      refline_points[i].left_obstacle_distance =
          input[i].distance_to_left_obstacle;
      refline_points[i].right_obstacle_distance =
          input[i].distance_to_right_obstacle;
      refline_points[i].lane_width = input[i].lane_width;
      refline_points[i].track_id = std::to_string(input[i].track_id);
    }
  }

  static inline void fill_point(const mjson::Reader &j, const std::string key,
                                Point3D &value) {
    auto obj = mjson::Reader(j.get<mjson::Json>(key, false, mjson::Json()));

    value.x = obj.get<double>("x", false, 0.0);
    value.y = obj.get<double>("y", false, 0.0);
    value.z = obj.get<double>("z", false, 0.0);
  }

  static inline void fill_point(maf_perception_interface::Point3f input,
                                Point3D &value) {
    value.x = input.x;
    value.y = input.y;
    value.z = input.z;
  }

  static inline void fill_points(const mjson::Reader &j, const std::string key,
                                 std::vector<Point3D> &values) {
    values.clear();
    auto obj = j.get<mjson::Json::array>(key, false, mjson::Json::array());
    for (const auto &json_point : obj) {
      auto json_point_reader = mjson::Reader(json_point);
      Point3D value;

      value.x = json_point_reader.get<double>("x", false, 0.0);
      value.y = json_point_reader.get<double>("y", false, 0.0);
      value.z = json_point_reader.get<double>("z", false, 0.0);

      values.push_back(value);
    }
  }

  static inline void
  fill_points(std::vector<maf_perception_interface::Point3f> inputs,
              std::vector<Point3D> &values) {
    values.clear();
    for (const auto &input : inputs) {
      Point3D value;

      value.x = input.x;
      value.y = input.y;
      value.z = input.z;

      values.push_back(value);
    }
  }

  static inline void fill_points_local_points_fusion(
      maf_perception_interface::FusionGroundLineData inputs,
      std::vector<Point3D> &values) {
    values.clear();
    for (size_t i = 0; i < inputs.local_points_fusion.size(); i++) {
      Point3D value;

      value.x = inputs.local_points_fusion[i].x;
      value.y = inputs.local_points_fusion[i].y;
      value.z = inputs.local_points_fusion[i].z;

      values.push_back(value);
    }
  }

  void fill_square_mapping_result(
      SquareMapResponse &square_mapping_result,
      const std::vector<maf_worldmodel::ScenePolygonObject> &polygon_objects) {
    square_mapping_result.road_borders.clear();
    square_mapping_result.obstacles.pillar.clear();
    square_mapping_result.obstacles.unknown.clear();
    square_mapping_result.obstacles.fire_exit_door.clear();
    if (polygon_objects.empty())
      return;
    SquareObstacles square_obstacles;
    for (auto &polygon_object : polygon_objects) {
      if (polygon_object.object_type.value ==
          maf_worldmodel::SceneObjectType::MAP_FIRE_EXIT_DOOR) {
        ObstacleItem obstacle_item;
        obstacle_item.id = polygon_object.object_id;
        fill_points(polygon_object.object_polygon.points, obstacle_item.pts);
        square_obstacles.fire_exit_door.emplace_back(obstacle_item);
      } else if (polygon_object.object_type.value ==
                 maf_worldmodel::SceneObjectType::MAP_PILLAR) {
        ObstacleItem obstacle_item;
        obstacle_item.id = polygon_object.object_id;
        fill_points(polygon_object.object_polygon.points, obstacle_item.pts);
        if (obstacle_item.pts.size() == 5) {
          obstacle_item.pts.pop_back();
        }
        square_obstacles.pillar.emplace_back(obstacle_item);
      } else if (polygon_object.object_type.value ==
                 maf_worldmodel::SceneObjectType::MAP_TOLL_GATE) {
        ObstacleItem obstacle_item;
        obstacle_item.id = polygon_object.object_id;
        fill_points(polygon_object.object_polygon.points, obstacle_item.pts);
        obstacle_item.pts.pop_back();
        square_obstacles.unknown.emplace_back(obstacle_item);
      } else if (polygon_object.object_type.value ==
                 maf_worldmodel::SceneObjectType::UNKNOWN) {
        // Note: LaneLineType::UNKNOWN now belongs to
        // SceneObjectType::MAP_BORDER_DASH_LINE
        SquareRoadBorder square_road_border;
        square_road_border.id = polygon_object.object_id;
        fill_points(polygon_object.object_polygon.points,
                    square_road_border.pts);
        square_road_border.type = LaneLineType::PHYSICAL;
        square_mapping_result.road_borders.emplace_back(square_road_border);
      } else if (polygon_object.object_type.value ==
                 maf_worldmodel::SceneObjectType::MAP_BORDER_SOLID_LINE) {
        SquareRoadBorder square_road_border;
        square_road_border.id = polygon_object.object_id;
        fill_points(polygon_object.object_polygon.points,
                    square_road_border.pts);
        square_road_border.type = LaneLineType::SOLID_LINE;
        square_mapping_result.road_borders.emplace_back(square_road_border);
      } else if (polygon_object.object_type.value ==
                 maf_worldmodel::SceneObjectType::MAP_BORDER_DASH_LINE) {
        SquareRoadBorder square_road_border;
        square_road_border.id = polygon_object.object_id;
        fill_points(polygon_object.object_polygon.points,
                    square_road_border.pts);
        square_road_border.type = LaneLineType::DASH_LINE;
        square_mapping_result.road_borders.emplace_back(square_road_border);
      } else if (polygon_object.object_type.value ==
                 maf_worldmodel::SceneObjectType::MAP_BORDER_PHYSICAL) {
        SquareRoadBorder square_road_border;
        square_road_border.id = polygon_object.object_id;
        fill_points(polygon_object.object_polygon.points,
                    square_road_border.pts);
        square_road_border.type = LaneLineType::PHYSICAL;
        square_mapping_result.road_borders.emplace_back(square_road_border);
      }
    }
    square_mapping_result.obstacles = square_obstacles;
  }

  void fill_target_poi_info(
      const maf_worldmodel::ProcessedMapData processed_map_data,
      AimedPoiInfo &aimed_poi_info) {
    if (processed_map_data.available & ProcessedMapData::TARGET_POTISION) {
      if (processed_map_data.target_position.parking_target_position.available &
          ParkingTargetPosition::TARGET_MAP_POI) {
        auto &target_map_poi = processed_map_data.target_position
                                   .parking_target_position.target_map_poi;
        aimed_poi_info.id = target_map_poi.id;
        switch (target_map_poi.type.value) {
        case MapPOIType::UNKNOWN:
          aimed_poi_info.type = "UNKNOWN";
          break;
        case MapPOIType::PARKING_LOT:
          aimed_poi_info.type = "PARKING_LOT";
          break;
        case MapPOIType::HUMAN_ACCESS:
          aimed_poi_info.type = "HUMAN_ACCESS";
          break;
        case MapPOIType::DESTINATION:
          aimed_poi_info.type = "DESTINATION";
          break;
        case MapPOIType::PARKING:
          aimed_poi_info.type = "PARKING";
          break;
        case MapPOIType::BARRIER_GAP:
          aimed_poi_info.type = "BARRIER_GAP";
          break;
        case MapPOIType::FACILITY_ENTRANCE:
          aimed_poi_info.type = "FACILITY_ENTRANCE";
          break;
        case MapPOIType::FACILITY_EXIT:
          aimed_poi_info.type = "FACILITY_EXIT";
          break;
        case MapPOIType::FACILITY_EXIT_AND_ENTRANCE:
          aimed_poi_info.type = "FACILITY_EXIT_AND_ENTRANCE";
          break;
        case MapPOIType::BUS_STOP:
          aimed_poi_info.type = "BUS_STOP";
          break;
        case MapPOIType::GARAGE_ENTRANCE:
          aimed_poi_info.type = "GARAGE_ENTRANCE";
          break;
        case MapPOIType::GARAGE_EXIT:
          aimed_poi_info.type = "GARAGE_EXIT";
          break;
        case MapPOIType::SPEED_BUMP:
          aimed_poi_info.type = "SPEED_BUMP";
          break;
        case MapPOIType::CROSS_WALK:
          aimed_poi_info.type = "CROSS_WALK";
          break;
        case MapPOIType::DASHED_SEGMENT:
          aimed_poi_info.type = "DASHED_SEGMENT";
          break;
        case MapPOIType::CENTRAL_CIRCLE:
          aimed_poi_info.type = "CENTRAL_CIRCLE";
          break;
        case MapPOIType::NO_PARKING_ZONE:
          aimed_poi_info.type = "NO_PARKING_ZONE";
          break;
        case MapPOIType::ROAD_MERGE:
          aimed_poi_info.type = "ROAD_MERGE";
          break;
        case MapPOIType::ROAD_SPLIT:
          aimed_poi_info.type = "ROAD_SPLIT";
          break;
        default:
          aimed_poi_info.type = "UNKNOWN";
        };
        fill_points(target_map_poi.position.points, aimed_poi_info.corners);
        aimed_poi_info.rerouting_status = 1;
      }
    }
  }

  void fill_map_info(const maf_worldmodel::ProcessedMapData &processed_map_data,
                     MapInfo &map_info) {
    if (processed_map_data.available & ProcessedMapData::LANE_STRATEGY) {

      auto &current_lane_change_available_interval =
          processed_map_data.lane_strategy
              .current_lane_change_available_interval;
      map_info.first_task_ranges.resize(
          current_lane_change_available_interval.size());
      for (size_t i = 0; i < current_lane_change_available_interval.size();
           i++) {
        map_info.first_task_ranges[i].begin_pos =
            current_lane_change_available_interval[i].begin;
        map_info.first_task_ranges[i].end_pos =
            current_lane_change_available_interval[i].end;
      }
      map_info.current_tasks.clear();
      for (auto state :
           processed_map_data.lane_strategy.strategy_start_with_current_lane) {
        if (state.value == LaneChangeState::TURN_LEFT) {
          map_info.current_tasks.push_back(-1);
        } else if (state.value == LaneChangeState::TURN_RIGHT) {
          map_info.current_tasks.push_back(1);
        } else {
          mph_assert(false);
        }
      }
    }

    if (processed_map_data.available & ProcessedMapData::SELF_POSITION) {
      map_info.is_in_intersection =
          processed_map_data.self_position.in_intersection;

      map_info.is_on_ramp = processed_map_data.self_position.on_ramp;

      map_info.current_parking_lot_id =
          processed_map_data.self_position.current_parking_slot_id;
    }

    if (processed_map_data.available & ProcessedMapData::INTERSECTION) {
      if (processed_map_data.intersections.size() > 0) {
        map_info.road_type =
            RoadType(processed_map_data.intersections[0].direction.value);
        map_info.distance_to_crossing =
            processed_map_data.intersections[0]
                .distance_to_stop_line.distance_to_stop_line;
        map_info.distance_to_stop_line =
            processed_map_data.intersections[0]
                .distance_to_stop_line.distance_to_stop_line;
      }
    }

    if (processed_map_data.available & ProcessedMapData::LANE) {
      auto &map_info_lanes_info = map_info.lanes_info_marks;
      map_info_lanes_info.resize(processed_map_data.lanes.size());

      for (auto &lane : processed_map_data.lanes) {
        map_info_lanes_info[lane.relative_id -
                            processed_map_data.lanes[0].relative_id] =
            RoadType(lane.lane_marks.value);

        if (lane.relative_id == 0) {
          fill_refline_points(lane.reference_line.reference_line_points,
                              map_info.current_refline_points);

          map_info.left_boundary_info.resize(
              lane.left_lane_boundary.segments.size());
          for (size_t i = 0; i < lane.left_lane_boundary.segments.size(); i++) {
            map_info.left_boundary_info[i].length =
                lane.left_lane_boundary.segments[i].length;
            map_info.left_boundary_info[i].type =
                lane.left_lane_boundary.segments[i].type.value.value;
          }
          map_info.right_boundary_info.resize(
              lane.right_lane_boundary.segments.size());
          for (size_t i = 0; i < lane.right_lane_boundary.segments.size();
               i++) {
            map_info.right_boundary_info[i].length =
                lane.right_lane_boundary.segments[i].length;
            map_info.right_boundary_info[i].type =
                lane.right_lane_boundary.segments[i].type.value.value;
          }
          map_info.next_merge_type = lane.merge_point.type.value;
        } else if (lane.relative_id == -1) {
          fill_refline_points(lane.reference_line.reference_line_points,
                              map_info.left_refline_points);
        } else if (lane.relative_id == 1) {
          fill_refline_points(lane.reference_line.reference_line_points,
                              map_info.right_refline_points);
        }
      }
      map_info.current_lane_index =
          (processed_map_data.lanes.size() > 0)
              ? 0 - processed_map_data.lanes[0].relative_id
              : 0;
    }

    // map_info.traffic_light.patterns.resize(
    //     map_planning->traffic_light.patterns.size());
    // for (size_t i = 0; i < map_planning->traffic_light.patterns.size();
    // i++) {
    //   map_info.traffic_light.patterns[i] =
    //       Pattern(map_planning->traffic_light.patterns[i]);
    // }
    // map_info.traffic_light.direction =
    //     RoadType(map_planning->traffic_light.direction);
    if (processed_map_data.available & ProcessedMapData::TARGET_POTISION) {
      if (processed_map_data.target_position.parking_target_position.available &
          ParkingTargetPosition::TARGET_MAP_POI) {
        auto &target_map_poi = processed_map_data.target_position
                                   .parking_target_position.target_map_poi;
        map_info.distance_to_aimed_poi = target_map_poi.distance;
      }
    }
    map_info.square_mapping_result = on_path_polygon_objects_;
  }

  void fill_fusion_object_info(
      const std::vector<maf_perception_interface::PerceptionFusionObjectData>
          &input,
      std::vector<FusionObject> &ret) {
    ret.clear();
    ret.resize(input.size());
    for (size_t i = 0; i < input.size(); ++i) {
      ret[i].track_id = input[i].track_id;
      ret[i].type = static_cast<FusionObjectType>(
          from_msd_fusion_type(input[i].type_info));
      ret[i].is_static = input[i].is_static;
      ret[i].shape.height = input[i].shape.height;
      ret[i].shape.length = input[i].shape.length;
      ret[i].shape.width = input[i].shape.width;
      ret[i].position.x = input[i].position.x;
      ret[i].position.y = input[i].position.y;
      ret[i].position.z = input[i].position.z;
      ret[i].velocity.x = input[i].velocity.x;
      ret[i].velocity.y = input[i].velocity.y;
      ret[i].velocity.z = input[i].velocity.z;
      ret[i].relative_position.x = input[i].relative_position.x;
      ret[i].relative_position.y = input[i].relative_position.y;
      ret[i].relative_velocity.x = input[i].velocity_relative_to_ground.x;
      ret[i].relative_velocity.y = input[i].velocity_relative_to_ground.y;
      ret[i].heading_yaw = input[i].heading_yaw;
      ret[i].relative_heading_yaw = input[i].relative_heading_yaw;
    }
  }

  // void fill_fusion_object_info(
  //     const std::vector<ObjectInterface> &input,
  //     std::vector<FusionObject> &ret) {
  //   ret.clear();
  //   ret.resize(input.size());
  //   for (size_t i = 0; i < input.size(); ++i) {
  //     const auto &object_fusion_data =
  //     input[i].object_fusion_result.object_fusion_data; ret[i].track_id =
  //     object_fusion_data.track_id; ret[i].type =
  //     static_cast<FusionObjectType>(from_msd_fusion_type(object_fusion_data.type_info));
  //     ret[i].shape.height = object_fusion_data.shape.height;
  //     ret[i].shape.length = object_fusion_data.shape.length;
  //     ret[i].shape.width = object_fusion_data.shape.width;
  //     ret[i].position.x = object_fusion_data.position.x;
  //     ret[i].position.y = object_fusion_data.position.y;
  //     ret[i].position.z = object_fusion_data.position.z;
  //     ret[i].velocity.x = object_fusion_data.velocity.x;
  //     ret[i].velocity.y = object_fusion_data.velocity.y;
  //     ret[i].velocity.z = object_fusion_data.velocity.z;
  //     ret[i].relative_position.x = object_fusion_data.relative_position.x;
  //     ret[i].relative_position.y = object_fusion_data.relative_position.y;
  //     ret[i].relative_velocity.x =
  //     object_fusion_data.velocity_relative_to_ground.x;
  //     ret[i].relative_velocity.y =
  //     object_fusion_data.velocity_relative_to_ground.y; ret[i].heading_yaw =
  //     object_fusion_data.heading_yaw; ret[i].relative_heading_yaw   =
  //     object_fusion_data.relative_heading_yaw;
  //   }
  // }

  void fill_ground_line_info(
      const std::vector<maf_perception_interface::FusionGroundLineData> &input,
      std::vector<msquare::GroundLine> &ret) {
    ret.clear();
    ret.resize(input.size());
    // MSD_LOG(WARN, "fill_ground_line_info size = %d", input.size());
    for (size_t i = 0; i < input.size(); ++i) {
      ret[i].id = input[i].track_id;
      ret[i].type = static_cast<GroundLineType>(input[i].type.value);
      fill_points_local_points_fusion(input[i], ret[i].pts);
    }
  }

  void fill_ground_line_info(
      const std::vector<maf_perception_interface::FusionGroundLineData> &input,
      std::vector<FusionFreespacePoint> &ret) {
    ret.clear();
    for (size_t i = 0; i < input.size(); ++i) {
      for (size_t j = 0; j < input[i].local_points_fusion.size(); j++) {
        FusionFreespacePoint fs_point;
        fs_point.position.x = input[i].local_points_fusion[j].x;
        fs_point.position.y = input[i].local_points_fusion[j].y;
        fs_point.position.z = input[i].local_points_fusion[j].z;
        fs_point.confidence = input[i].local_points_fusion_confidence[j];
        fs_point.type = FusionFreespacePointType::PFUSION_FREESPACE_UNKNOWN;
        ret.emplace_back(fs_point);
      }
    }
  }

  void
  fill_parking_lot_info(const FusionAPA &input_fusion_apa,
                        std::vector<msquare::ParkingLotDetectionInfo> &ret) {
    auto &input = input_fusion_apa.parking_slots;
    auto &info_list = input_fusion_apa.reserved_info;
    size_t info_size = info_list.size();
    ret.clear();
    ret.resize(input.size());
    for (size_t i = 0; i < input.size(); ++i) {
      msquare::ParkingLotDetectionInfo parking_lot_info;
      parking_lot_info.corners.resize(
          input[i].parking_slot.local_points_fusion.size());
      for (size_t j = 0; j < input[i].parking_slot.local_points_fusion.size();
           ++j) {
        msquare::ParkingLotDetectionInfo::CornerPoint corner;
        corner.position.x = input[i].parking_slot.local_points_fusion[j].x;
        corner.position.y = input[i].parking_slot.local_points_fusion[j].y;
        corner.position.z = input[i].parking_slot.local_points_fusion[j].z;
        if (j < input[i].parking_slot.local_points_fusion_confidence.size()) {
          corner.confidence =
              input[i].parking_slot.local_points_fusion_confidence[j];
        } else {
          corner.confidence = 1.0;
          MSD_LOG(ERROR,
                  "fill_parking_lot_info: local_points_fusion_confidence size: "
                  "%d does not match local_points_fusion size: %d",
                  input[i].parking_slot.local_points_fusion_confidence.size(),
                  input[i].parking_slot.local_points_fusion.size());
        }
        if (j < input[i].parking_slot.local_points_fusion_type.size()) {
          corner.is_visible =
              (input[i].parking_slot.local_points_fusion_type[j].value ==
               ParkingSlotPointTypeEnum::PARKING_SLOT_POINT_TYPE_REAL);
        } else {
          corner.is_visible = true;
          MSD_LOG(ERROR,
                  "fill_parking_lot_info: local_points_fusion_type size: %d "
                  "does not match local_points_fusion size: %d",
                  input[i].parking_slot.local_points_fusion_type.size(),
                  input[i].parking_slot.local_points_fusion.size());
        }
        parking_lot_info.corners[j] = corner;
      }
      parking_lot_info.is_good = true;
      parking_lot_info.is_empty = (input[i].apa_info.status == APAInfo::VACANT);
      parking_lot_info.is_on_map_list = (input[i].apa_info.map_id > 0);
      parking_lot_info.id = parking_lot_info.is_on_map_list
                                ? input[i].apa_info.map_id
                                : input[i].parking_slot.track_id;
      parking_lot_info.is_car_in =
          parking_lot_info.is_on_map_list
              ? (input_fusion_apa.ego_parking_slot_map_id ==
                 parking_lot_info.id)
              : (input_fusion_apa.ego_parking_slot_track_id ==
                 parking_lot_info.id);
      auto wheel_stop_points = input[i].parking_slot.local_wheel_stop_points;
      parking_lot_info.wheel_stop.available = (wheel_stop_points.size() == 2);
      if (parking_lot_info.wheel_stop.available) {
        parking_lot_info.wheel_stop.point1.x = wheel_stop_points[0].x;
        parking_lot_info.wheel_stop.point1.y = wheel_stop_points[0].y;
        parking_lot_info.wheel_stop.point1.z = wheel_stop_points[0].z;
        parking_lot_info.wheel_stop.point2.x = wheel_stop_points[1].x;
        parking_lot_info.wheel_stop.point2.y = wheel_stop_points[1].y;
        parking_lot_info.wheel_stop.point2.z = wheel_stop_points[1].z;
      }

      const auto& parking_slot_infos = input[i].parking_slot;
      if(parking_slot_infos.available) {
        bool source_from_uss =
          parking_slot_infos.available & parking_slot_infos.FUSION_PARKING_SLOT_SOURCE_ULTRASONIC;
        bool source_from_vision =
          parking_slot_infos.available & parking_slot_infos.FUSION_PARKING_SLOT_SOURCE_VISON;
        parking_lot_info.is_space_slot = source_from_uss && !source_from_vision;
      }

      if (i >= info_size) {
        ret[i] = parking_lot_info;
        continue;
      }
      // fill virtual corner
      msquare::ParkingLotDetectionInfo::VirtualCorner virtual_corner;
      std::string info_str = info_list[i];
      std::string error = "";
      auto slot_type_mjson = mjson::Json::parse(info_str, error);
      if (error != "") {
        // MSD_LOG( INFO,"[fill_parking_lot_info] parse reserved_info error
        // %s\n", error.c_str());
        ret[i] = parking_lot_info;
        continue;
      }
      auto slot_type_reader = mjson::Reader(info_str);

      if (false == slot_type_mjson.has_key("slot_type")) {
        // MSD_LOG(INFO, "[fill_parking_lot_info] reserved info(%s) has no
        // slot_type key\n", reserved_info.c_str());
        ret[i] = parking_lot_info;
        continue;
      }
      int slot_type = slot_type_reader.get<int>("slot_type");

      if (false == slot_type_mjson.has_key("virtual_point_index")) {
        // MSD_LOG(INFO, "[fill_parking_lot_info] reserved info(%s) has no
        // virtual_point_index key\n", reserved_info.c_str());
        ret[i] = parking_lot_info;
        continue;
      }
      int virtual_point_index =
          slot_type_reader.get<int>("virtual_point_index");

      double vx, vy, vz;
      if (false == slot_type_mjson.has_key("virual_point_x")) {
        // MSD_LOG(INFO,"[fill_parking_lot_info] reserved info(%s) has no
        // virual_point_x key\n", reserved_info.c_str());
        ret[i] = parking_lot_info;
        continue;
      }
      vx = slot_type_reader.get<double>("virual_point_x");

      if (false == slot_type_mjson.has_key("virual_point_y")) {
        // MSD_LOG(INFO,"[fill_parking_lot_info] reserved info(%s) has no
        // virual_point_y key\n", reserved_info.c_str());
        ret[i] = parking_lot_info;
        continue;
      }
      vy = slot_type_reader.get<double>("virual_point_y");

      if (false == slot_type_mjson.has_key("virual_point_z")) {
        // MSD_LOG(INFO,"[fill_parking_lot_info] reserved info(%s) has no
        // virual_point_z key\n", reserved_info.c_str());
        ret[i] = parking_lot_info;
        continue;
      }
      vz = slot_type_reader.get<double>("virual_point_z");
      virtual_corner.slot_type = slot_type;
      virtual_corner.index = virtual_point_index;
      virtual_corner.p.x = vx;
      virtual_corner.p.y = vy;
      virtual_corner.p.z = vz;

      parking_lot_info.virtual_corner = virtual_corner;
      parking_lot_info.charge_property = input[i].parking_slot.charge_property;
      ret[i] = parking_lot_info;
    }
  }

  void update_mpc_trajectory(double current_time) {
    if (!mpc_trajectory_receiver_->empty()) {
      std::shared_ptr<maf_planning::MpcTrajectoryResult> mpc_trajectory{};
      auto ret =
          mpc_trajectory_receiver_->fetch_newest_and_clear(mpc_trajectory);
      if (ret && mpc_trajectory->mpc_trajectory.available &
                     maf_planning::MpcTrajectory::PATH_POINTS) {
        bool use_sop_algotithm = 
            msquare::CarParams::GetInstance()->car_config.lon_config.use_sop_algorithm;
        std::vector<PathPoint> mpc_traj;
        auto &path_points = mpc_trajectory->mpc_trajectory.path_points;
        uint64_t ignore_point_count = 1;
        if (use_sop_algotithm) {
          ignore_point_count = 2;
        }
        for (uint64_t i = 0; i + ignore_point_count < path_points.size(); i++) {
          PathPoint mpc_point;
          mpc_point.x = path_points[i].position_enu.x;
          mpc_point.y = path_points[i].position_enu.y;
          mpc_point.theta = path_points[i].heading_yaw;
          mpc_traj.push_back(mpc_point);
        }
        world_model_->feed_mpc_traj(mpc_traj);
        last_feed_time_[FEED_CONTROL_MPC] = current_time;
        if (!path_points.empty()) {
          auto &control_feedback = path_points.back();
          bool collide_to_limiter_when_reverse =
              control_feedback.position_enu.x > 0.0;
          world_model_->feed_collide_to_limiter_when_reverse(
              collide_to_limiter_when_reverse);
          bool steering_reset = control_feedback.position_enu.y > 0.0;
          bool blocked_by_uss = fabs(control_feedback.heading_yaw - 1.0) < 1e-3;
          bool blocked_by_ground =
              fabs(control_feedback.heading_yaw - 2.0) < 1e-3;
          PlanningContext::Instance()
              ->mutable_planning_status()
              ->control_block_feedback = blocked_by_uss || blocked_by_ground;
          MSD_LOG(ERROR, "[%s] blocked_by_uss:%d, blocked_by_ground:%d", __FUNCTION__, blocked_by_uss, blocked_by_ground);    
          world_model_->feed_steering_reset(steering_reset);
          if (path_points.size() >= 2 && use_sop_algotithm) {
            const auto &last_second_pt = path_points[path_points.size() - 2];
            if (std::fabs(last_second_pt.heading_yaw - 1.0) < 1e-3) {
              PlanningContext::Instance()
                ->mutable_planning_status()
                ->is_reached_end = true;
              MSD_LOG(ERROR, "----------------> is reached end");
              // std::cout << "----------------> is reached end" << std::endl;
            } else {
              PlanningContext::Instance()
                ->mutable_planning_status()
                ->is_reached_end = false;            
            }
          }
        } else {
          PlanningContext::Instance()
              ->mutable_planning_status()
              ->control_block_feedback = false;
        }
      }
    }
    if (PlanningContext::Instance()->planning_status().task_status.task ==
            StatusType::WAIT ||
        PlanningContext::Instance()->planning_status().task_status.task ==
            StatusType::RPA_STRAIGHT_STANDBY) {
      last_feed_time_[FEED_CONTROL_MPC] = current_time;
    }
  }

  void clear_idle_receiver() {
    if (!worldmodel_objects_receiver_->empty()) {
      std::shared_ptr<maf_worldmodel::ObjectsInterface> msg{};
      (void)worldmodel_objects_receiver_->fetch_newest_and_clear(msg);
    }
    if (!traffic_light_receiver_->empty()) {
      std::shared_ptr<maf_perception_interface::TrafficLightPerception> msg{};
      (void)traffic_light_receiver_->fetch_newest_and_clear(msg);
    }
    if (!prediction_result_receiver_->empty()) {
      std::shared_ptr<maf_worldmodel::PredictionResult> msg{};
      (void)prediction_result_receiver_->fetch_newest_and_clear(msg);
    }
  }

  void update_wlc_info() {
    if (!wireless_charger_report_recv_->empty()) {
      std::shared_ptr<maf_endpoint::WirelessChargerReport>
        wireless_charger_report{};
      auto ret = wireless_charger_report_recv_->fetch_newest_and_clear(wireless_charger_report);
      if (ret) {
        if (wireless_charger_report->available & maf_endpoint::WirelessChargerReport::WIRELESS_CHARGER_REPORT_DATA) {
          world_model_->feed_wireless_charger_report_data(wireless_charger_report->wireless_charger_report_data);
        }
      }
    }
  }

  void update_worldmodel_info(double current_time);

  void update_world_model(double current_time) {
    MLOG_PROFILING("update_world_model");

    if (!module_status_receiver_->empty()) {
      ModuleStatus module_status{};
      auto ret = module_status_receiver_->fetch_newest_and_clear(module_status);
      if (ret) {
        bool status{true};
        if (ModuleStatus::STOP == module_status.status ||
            ModuleStatus::RUNNING_TAKING_OVER == module_status.detail_status) {
          status = false;
        }

        auto env = std::getenv("WHICH_CAR");
        if (env != nullptr) {
          std::string which_car(env);
          MSD_LOG(INFO, "WHICH_CAR: %s", which_car.c_str());
          if (std::strcmp(env, "MKZ_SIM") == 0) {
            status = true;
          }
        }
        world_model_->feed_vehicle_dbw_status(status);
        last_feed_time_[FEED_VEHICLE_DBW_STATUS] = current_time;
      }
    }

    if (!reset_) {
      update_planning_request(current_time);
    }

    update_vehicle_status(current_time);

    update_imu_report(current_time);

    update_worldmodel_info(current_time);

    update_mpc_trajectory(current_time);

    update_wlc_info();

    clear_idle_receiver();
  }

  bool can_run(double current_time, std::string &warn_msg,
               std::string &error_msg) {
    auto to_string = [](FeedType feed_type) -> const char * {
      switch (feed_type) {
      case FEED_VEHICLE_DBW_STATUS:
        return "vehicle_dbw_status";
      case FEED_EGO_VEL:
        return "ego_pose_and_vel";
      case FEED_EGO_STEER_ANGLE:
        return "ego_steer_angle";
      case FEED_EGO_ENU:
        return "ego_enu";
      case FEED_WHEEL_SPEED_REPORT:
        return "wheel_speed_report";
      case FEED_GEAR_REPORT:
        return "gear_report";
      case FEED_EGO_ACC:
        return "ego_acc";
      case FEED_MISC_REPORT:
        return "misc_report";
      // case FEED_MAP_INFO:
      //   return "map_info";
      case FEED_FUSION_INFO:
        return "fusion_info";
      case FEED_FUSION_APA:
        return "fusion_apa";
      case FEED_FUSION_GROUNDLINE:
        return "fusion_groundline";
      case FEED_FUSION_USS:
        return "fusion_uss";
      case FEED_SCENE_OBJECT:
        return "scene_object";
      case FEED_SENSOR_IMU:
        return "sensor_imu";
      case FEED_CONTROL_MPC:
        return "control_mpc";
      // case FEED_PREDICTION_INFO:
      //   return "prediction_info";
      // case FEED_FUSION_LANES_INFO:
      //   return "fusion_lanes_info";
      default:
        return "unknown type";
      };
    };

    static const double kCheckTimeDiffWarn = 0.5;
    static const double kCheckTimeDiffError = 1.0;

    warn_msg.clear();
    error_msg.clear();

    bool res = true;
    for (int i = 0; i < FEED_TYPE_MAX; ++i) {
      const char *feed_type_str = to_string(static_cast<FeedType>(i));
      if ((world_model_->is_parking_apa() ||
          world_model_->is_parking_lvp()) && i == FEED_SCENE_OBJECT) {
        // Do not have map info in APA mode
        continue;
      }

      if (false && world_model_->is_parking_lvp() && i == FEED_SCENE_OBJECT) {
        // Do not have scene object info in LVP mode
        continue;
      }

      if (last_feed_time_[i] > 0.0) {
        if (current_time - last_feed_time_[i] > kCheckTimeDiffWarn &&
            current_time - last_feed_time_[i] <= kCheckTimeDiffError) {
          MSD_LOG(ERROR, "(%s)feed delay warn: %d, %s", __FUNCTION__, i,
                  feed_type_str);
          warn_msg += std::string(feed_type_str) + "; ";
        }

        if (current_time - last_feed_time_[i] > kCheckTimeDiffError) {
          MSD_LOG(ERROR, "(%s)feed delay error: %d, %s", __FUNCTION__, i,
                  feed_type_str);
          error_msg += std::string(feed_type_str) + "; ";
          res = false;
        }
      } else {
        MSD_LOG(ERROR, "(%s)no feed: %d, %s", __FUNCTION__, i, feed_type_str);
        error_msg += std::string(feed_type_str) + "; ";
        res = false;
      }
    }

    return res;
  }

private:
  enum FeedType {
    FEED_VEHICLE_DBW_STATUS = 0,
    FEED_EGO_VEL,
    FEED_EGO_STEER_ANGLE,
    FEED_EGO_ENU,
    FEED_WHEEL_SPEED_REPORT,
    FEED_GEAR_REPORT,
    FEED_EGO_ACC,
    FEED_MISC_REPORT,
    // FEED_MAP_INFO,
    FEED_FUSION_INFO,
    FEED_FUSION_APA,
    FEED_FUSION_GROUNDLINE,
    FEED_FUSION_USS,
    FEED_SCENE_OBJECT,
    FEED_SENSOR_IMU,
    FEED_CONTROL_MPC,
    // FEED_PREDICTION_INFO,
    // FEED_FUSION_LANES_INFO,
    FEED_TYPE_MAX,
  };

  maf_planning::Planning generate_planning_output();
  maf_planning::Planning generate_planning_output_sop();

  maf_std::Header generate_planning_info(uint64_t tick_count) {
    maf_std::Header planning_info{};
    auto info = mjson::Json(mjson::Json::object());
    auto trajectory_info = mjson::Json(mjson::Json::object());
    const auto &planning_result =
        PlanningContext::Instance()->planning_status().planning_result;
    // Trajector Info
    auto polynomial_curve = mjson::Json::array(5, mjson::Json(0.0));
    double x_start = 0.0;
    double x_end = 0.0;
    if (!planning_result.traj_pose_array.empty()) {
      std::vector<double> poly_coeff;
      std::vector<Pose2D> traj;
      // std::vector<Pose2D> mpc_traj;
      // auto &origin_mpc_traj = world_model_->get_mpc_trajectory();
      traj.resize(planning_result.traj_pose_array.size());
      // mpc_traj.resize(origin_mpc_traj.size());
      Pose2D ego_pose = world_model_->get_ego_state().ego_pose;
      std::transform(planning_result.traj_pose_array.begin(),
                     planning_result.traj_pose_array.end(), traj.begin(),
                     [&ego_pose](Pose2D pose) {
                       return planning_math::tf2d(ego_pose, pose);
                     });
      std::vector<Pose2D> filter_traj;
      std::vector<double> filter_vel;
      int sgn = planning_result.gear != GearState::REVERSE ? -1 : 1;
      double x_last = std::numeric_limits<double>::infinity() * sgn;
      double step_size = 0.15;
      for (int i = 0; i < traj.size(); i++) {
        if (-sgn * (traj[i].x - x_last) > step_size) {
          filter_traj.emplace_back(traj[i]);
          filter_vel.emplace_back(planning_result.traj_vel_array[i]);
          x_last = traj[i].x;
        }
      }
      for (int i = (int)filter_vel.size() - 2; i >= 0; i--) {
        if (std::abs(filter_vel[i]) > 1e-5) {
          if (planning_result.gear != GearState::REVERSE) {
            x_end = filter_traj[i + 1].x +
                    (VehicleParam::Instance()->front_edge_to_center -
                     VehicleParam::Instance()->wheel_base) *
                        0.9 * std::cos(filter_traj[i + 1].theta);
          } else {
            x_end = filter_traj[i + 1].x -
                    VehicleParam::Instance()->back_edge_to_center * 0.9 *
                        std::cos(filter_traj[i + 1].theta);
          }
          break;
        }
      }
      for (int i = (int)planning_result.traj_vel_array.size() - 2; i >= 0;
           i--) {
        if (std::abs(planning_result.traj_vel_array[i]) > 1e-5) {
          if (planning_result.gear != GearState::REVERSE) {
            x_end =
                std::min(traj[i + 1].x +
                             (VehicleParam::Instance()->front_edge_to_center -
                              VehicleParam::Instance()->wheel_base) *
                                 0.9 * std::cos(traj[i + 1].theta),
                         x_end);
          } else {
            x_end = std::max(traj[i + 1].x -
                                 VehicleParam::Instance()->back_edge_to_center *
                                     0.9 * std::cos(traj[i + 1].theta),
                             x_end);
          }
          break;
        }
      }
      // if (!mpc_traj.empty()) {
      //   for (auto &mpc_point : origin_mpc_traj) {
      //     Pose2D pose;
      //     pose.x = mpc_point.x;
      //     pose.y = mpc_point.y;
      //     pose.theta = mpc_point.theta;
      //     mpc_traj.emplace_back(planning_math::tf2d(ego_pose, pose));
      //   }
      //   for (int i = 0; i < traj.size(); i++) {
      //     if (std::abs(traj[i].x) > std::abs(mpc_traj.back().x) &&
      //         traj[i].x * mpc_traj.back().x >= 0.0) {
      //       if (i > 0) {
      //         traj.erase(traj.begin(), traj.begin() + i - 1);
      //       }
      //       break;
      //     }
      //   }
      //   traj.insert(traj.begin(), mpc_traj.begin(), mpc_traj.end());
      // }
      if (Polyfit(filter_traj, 5, poly_coeff)) {
        poly_coeff[0] = 0.0;
        polynomial_curve.clear();
        for (auto coeff : poly_coeff) {
          polynomial_curve.emplace_back(mjson::Json(coeff));
        }
      }
    }
    trajectory_info["polynomial_curve"] = mjson::Json(polynomial_curve);
    trajectory_info["start_point_x"] = mjson::Json(x_start);
    trajectory_info["end_point_x"] = mjson::Json(x_end);
    trajectory_info["remaining_distance"] =
        mjson::Json(PlanningContext::Instance()
                        ->longitudinal_behavior_planner_output()
                        .traj_length);
    info["trajectory_info"] = trajectory_info;

    auto leader_info = mjson::Json(mjson::Json::object());
    auto &lead_one = PlanningContext::Instance()
                         ->longitudinal_behavior_planner_output()
                         .lead_cars.first;

    if (PlanningContext::Instance()->planning_status().blocked &&
        (lead_one.id <= 0 ||
         lead_one.d_rel > CarParams::GetInstance()->lon_inflation() + 0.1)) {
      leader_info["id"] = mjson::Json(1);
      leader_info["is_static"] = mjson::Json(true);
      leader_info["d_rel"] =
          mjson::Json(CarParams::GetInstance()->lon_inflation());
      leader_info["type"] = mjson::Json((int)ObjectType::FREESPACE);
      
    } else {
      const auto& remain_dist_info = PlanningContext::Instance()
                        ->longitudinal_behavior_planner_output().remain_dist_info_;

      leader_info["id"] = mjson::Json(remain_dist_info.id_);
      if(msquare::CarParams::GetInstance()->car_config.lon_config.keep_people_dynamic){
        leader_info["is_static"] =
            ((int)remain_dist_info.type_ == (int)ObjectType::PEDESTRIAN)
                ? mjson::Json(false)
                : mjson::Json((int)remain_dist_info.is_static_);
      }else{
        leader_info["is_static"] = mjson::Json(remain_dist_info.is_static_);
      }
      leader_info["d_rel"] = mjson::Json(remain_dist_info.d_rel_);
      leader_info["type"] = mjson::Json((int)remain_dist_info.type_);
      MSD_LOG(ERROR, "lead_one.d_rel: %f", lead_one.d_rel);
    }
    if (PlanningContext::Instance()
            ->longitudinal_behavior_planner_output()
            .is_need_pause_)
      MSD_LOG(ERROR, "block is_need_pause");

    leader_info["is_need_pause"] =
        mjson::Json(PlanningContext::Instance()
                        ->longitudinal_behavior_planner_output()
                        .is_need_pause_);

    // debug info
    PlanningContext::Instance()
                        ->mutable_longitudinal_behavior_planner_output()->
                        remain_dist_info_.is_need_pause_ = PlanningContext::Instance()
                        ->longitudinal_behavior_planner_output().is_need_pause_;

    leader_info["is_blocked_by_obstacle_behind_in_slot"] =
        mjson::Json(PlanningContext::Instance()
                        ->longitudinal_behavior_planner_output()
                        .is_blocked_by_obstacle_behind_in_slot);
    leader_info["planning_debug_info"] =
        mjson::Json(PlanningContext::Instance()->planning_debug_info());

    // add multi circle ft model
    const auto& mc_model = PlanningContext::Instance()->lon_mc_footprint_model();
    int circle_num = mc_model->circles_vehicle_.size();
    auto mc_model_json = mjson::Json::array(3 * circle_num, mjson::Json(0.0));
    if (circle_num > 0) {
      //auto polynomial_curve = mjson::Json::array(5, mjson::Json(0.0));
      mc_model_json.clear();
      for (const auto& circle : mc_model->circles_vehicle_) {
        mc_model_json.emplace_back(circle.center_.x());
        mc_model_json.emplace_back(circle.center_.y());
        mc_model_json.emplace_back(circle.radius_);
      }
      leader_info["mc_footprint_model"] = mjson::Json(mc_model_json);
    }

    // add
    const auto& full_body_model_contour = *PlanningContext::Instance()->mutable_full_body_model_contour();
    auto model_json = mjson::Json::array(2 * full_body_model_contour.size(), 
        mjson::Json(0.0));
    if (full_body_model_contour.size() > 0) {
      model_json.clear();
      for (const auto& pt : full_body_model_contour) {
        model_json.emplace_back(pt.x());
        model_json.emplace_back(pt.y());
      }
      leader_info["full_body_model_contour"] = mjson::Json(model_json);
    }

    const auto &vec_extra_obstacle_points =
        *PlanningContext::Instance()->mutable_vec_extra_obstacle_points();
    if (!vec_extra_obstacle_points.empty()) {
      // 1. add extra pilliar points
      const auto &extra_pilliar_points = vec_extra_obstacle_points.at(0);
      auto model_json = mjson::Json::array(2 * extra_pilliar_points.size(), 
        mjson::Json(0.0));
      model_json.clear();
      for (const auto& pt : extra_pilliar_points) {
        model_json.emplace_back(pt.x());
        model_json.emplace_back(pt.y());
      }
      leader_info["extra_obstacle_point"] = mjson::Json(model_json);
    }

    info["cipv_info"] = leader_info;
    info["available_park_out_direction"] =
        mjson::Json(PlanningContext::Instance()
                        ->mutable_parking_behavior_planner_output()
                        ->parking_slot_info.available_park_out_type.value);
    auto wlc_info = mjson::Json(mjson::Json::object());
    int approaching = 0;
    if (PlanningContext::Instance()->planning_status().wlc_info.is_approaching) {
      approaching = 1;
      if (PlanningContext::Instance()->planning_status().wlc_info.is_approached) {
        approaching = 2;
      }
    }

    info["get_take_over_point"] = mjson::Json(PlanningContext::Instance()
          ->parking_behavior_planner_output().reached_parkout_takeover_point);

    wlc_info["approaching"] = mjson::Json(approaching);
    info["wlc_info"] = wlc_info;
    info["recommend_park_out_direction"] =
        mjson::Json(PlanningContext::Instance()
                        ->mutable_parking_behavior_planner_output()
                        ->parking_slot_info.recommend_park_out_type.value);
    info["is_direction_limit"] =
        mjson::Json(PlanningContext::Instance()
                        ->mutable_parking_behavior_planner_output()
                        ->parking_slot_info.is_direction_limit);

    auto rpa_info = mjson::Json(mjson::Json::object());
    rpa_info["forward_distance"] =
        mjson::Json(PlanningContext::Instance()
                        ->mutable_parking_behavior_planner_output()
                        ->rpa_forward_dis);
    rpa_info["backward_distance"] =
        mjson::Json(PlanningContext::Instance()
                        ->mutable_parking_behavior_planner_output()
                        ->rpa_backward_dis);
    info["rpa_info"] = rpa_info;
#ifdef RUN_IN_FPP
    //encode and publish planning_context debug data to /planning/info in FPP mode
    info["parking_planning_context"] = PlanningContext::Instance()->encoder_planning_context();
#endif
    planning_info.stamp = MTIME()->timestamp().ns();
    planning_info.seq = tick_count;
    planning_info.frame_id = info.dump();

    return planning_info;
  }

  bool is_in_simulation();
  void _run_in_simulation();
  void _run_in_reality();

private:
  // std::shared_ptr<ModuleMonitor> monitor_;
  std::shared_ptr<maf::StatusManager> monitor_;
  std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<ApaStateMachine> apa_state_machine_;
  ModuleStatus planning_module_status_;
  maf_vehicle_status::VehicleStatus vehicle_status_{};
  // CollisionCheck collision_check_;
  StaticCheck static_check_;
  SquareMapResponse on_path_polygon_objects_;
  maf_system_manager::ModuleControlCmdRequest module_control_cmd_request_{};

  double last_feed_time_[FEED_TYPE_MAX]{};

  mtaskflow::FlowReceiver<uint64_t> tick_receiver_;
  mtaskflow::FlowReceiver<ModuleStatus> module_status_receiver_;
  mtaskflow::FlowReceiver<std::shared_ptr<maf_endpoint::ChassisReport>>
      chassis_report_receiver_;
  mtaskflow::FlowReceiver<std::shared_ptr<maf_endpoint::WheelReport>>
      wheel_report_receiver_;
  mtaskflow::FlowReceiver<std::shared_ptr<maf_endpoint::BodyReport>>
      body_report_receiver_;
  mtaskflow::FlowReceiver<std::shared_ptr<maf_gps_imu::MLAImu>>
      imu_report_receiver_;
  mtaskflow::FlowReceiver<std::shared_ptr<maf_worldmodel::ProcessedMap>>
      worldmodel_map_receiver_;
  mtaskflow::FlowReceiver<std::shared_ptr<maf_worldmodel::ObjectsInterface>>
      worldmodel_objects_receiver_;
  mtaskflow::FlowReceiver<std::shared_ptr<maf_worldmodel::FusionAPA>>
      worldmodel_parking_slots_receiver_;
  mtaskflow::FlowReceiver<std::shared_ptr<maf_worldmodel::SceneObjects>>
      worldmodel_scene_objects_receiver_;
  mtaskflow::FlowReceiver<
      std::shared_ptr<maf_perception_interface::PerceptionFusionObjectResult>>
      fusion_objects_receiver_;
  mtaskflow::FlowReceiver<
      std::shared_ptr<maf_perception_interface::FusionGroundLineResult>>
      fusion_groundlines_receiver_;
  mtaskflow::FlowReceiver<
      std::shared_ptr<maf_perception_interface::FusionGroundLineResult>>
      fusion_uss_groundlines_receiver_;
  mtaskflow::FlowReceiver<
      std::shared_ptr<maf_sensor_interface::UltrasonicUpaReport>>
      fusion_uss_receiver_;
  mtaskflow::FlowReceiver<
      std::shared_ptr<maf_perception_interface::TrafficLightPerception>>
      traffic_light_receiver_;
  mtaskflow::FlowReceiver<
      std::shared_ptr<maf_mla_localization::MLALocalization>>
      ego_pose_receiver_;
  mtaskflow::FlowReceiver<std::shared_ptr<maf_worldmodel::PredictionResult>>
      prediction_result_receiver_;
  mtaskflow::FlowReceiver<std::shared_ptr<maf_planning::MpcTrajectoryResult>>
      mpc_trajectory_receiver_;
  mtaskflow::FlowReceiver<maf_system_manager::ModuleControlCmdRequest>
      planning_control_cmd_request_receiver_;
  mtaskflow::FlowReceiver<maf_system_manager::SysPlanningRequest>
      planning_request_receiver_;
  mtaskflow::FlowReceiver<SbpResult> sbp_result_receiver_;
  mtaskflow::FlowReceiver<std::string> sbp_debug_receiver_;
  mtaskflow::FlowPublisher<parking::OpenspaceDeciderOutput>
      sbp_problem_publisher_;
  mtaskflow::FlowReceiver<std::shared_ptr<maf_endpoint::WirelessChargerReport>>
      wireless_charger_report_recv_;

  MSDPlanningOutputCallback planning_output_callback_{};
  MSDPlanningTriggerCallback planning_trigger_callback_{};
  MSDPlanningInfoCallback planning_info_callback_{};

  std::function<void(const maf_planning::SBPRequest &sbp_request)>
      sbp_request_cb_{};

  bool enable_timer_tick_{true};
  bool reset_{false};
  double ego_pose_timestamp_us_ = 0.0;

  bool lase_cruise_control_set_increase_ = false;
  bool last_cruise_control_set_decrease_ = false;
  double cruise_velocity_ = 120.0; // km/h
  std::string sbp_debug_info_{};   // km/h

  // Add for SLP
  std::function<void()> _runner{};
  uint64_t planning_seq_{0};
  uint64_t localization_seq_{0};
  bool is_in_simulation_ = false;
  NPPHeaderLoggerCallback publish_simulation_sync_{};
};

} // namespace parking
} // namespace msquare
