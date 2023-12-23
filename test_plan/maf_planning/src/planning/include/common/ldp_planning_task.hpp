#include "common/config/vehicle_param.h"
#include "common/ldp_result.h"
#include "common/planning_context.h"
#include "common/utils/polyfit.h"
#include "ldp_config_context.h"
#include "mtaskflow/mtaskflow.hpp"
#include "planner/message_type.h"
#include "planning_task_interface.h"
#include "pnc/ldp_planning_engine_interface.h"
#include <iostream>

using namespace msd_planning;
using namespace maf_framework_status;
using namespace maf_worldmodel;
using namespace maf_perception_interface;
using namespace maf_mla_localization;
using namespace maf_vehicle_status;
using namespace maf_endpoint;

static const double kCheckTimeDiffWarn = 0.5;
static const double kCheckTimeDiffError = 1.0;

namespace msquare {

class LdpPlanningTask : public PlanningTaskInterface {
public:
  LdpPlanningTask(
      MSDLdpPlanningConfig planning_config, bool enable_timer_tick,
      std::shared_ptr<maf::StatusManager> monitor,
      mtaskflow::FlowReceiver<uint64_t> tick_receiver,
      mtaskflow::FlowReceiver<ModuleStatus> module_status_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<maf_endpoint::ChassisReport>>
          chassis_report_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<maf_endpoint::WheelReport>>
          wheel_report_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<maf_endpoint::BodyReport>>
          body_report_receiver,
      mtaskflow::FlowReceiver<
          std::shared_ptr<maf_perception_interface::RoadLinePerception>>
          perception_vision_lane_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<
          maf_perception_interface::PerceptionFusionObjectResult>>
          perception_fusion_receiver,
      mtaskflow::FlowReceiver<
          std::shared_ptr<maf_perception_interface::PerceptionFusionAEBResult>>
          perception_fusion_aeb_receiver,
      mtaskflow::FlowReceiver<std::string> mff_planning_request_receiver,
      mtaskflow::FlowReceiver<std::string> planning_ess_status_receiver)
      : enable_timer_tick_(enable_timer_tick), monitor_(monitor),
        tick_receiver_(tick_receiver),
        module_status_receiver_(module_status_receiver),
        chassis_report_receiver_(chassis_report_receiver),
        wheel_report_receiver_(wheel_report_receiver),
        body_report_receiver_(body_report_receiver),
        perception_vision_lane_receiver_(perception_vision_lane_receiver),
        perception_fusion_receiver_(perception_fusion_receiver),
        perception_fusion_aeb_receiver_(perception_fusion_aeb_receiver),
        mff_planning_request_receiver_(mff_planning_request_receiver),
        planning_ess_status_receiver_(planning_ess_status_receiver) {
    ConfigurationContext::Instance()->load_vehicle_param();

    auto config_file_dir = PlanningContext::Instance()->get_config_file_dir();
    LdpConfigurationContext::Instance()->load_ldp_planner_config(
        config_file_dir);
    ldp_planner_config_ =
        LdpConfigurationContext::Instance()->ldp_planner_config();
  }

  void on_running() {
    return;
    static uint64_t tick_count = 0;
    if (enable_timer_tick_) {
      tick_count++;
    } else if (!tick_receiver_->empty()) {
      uint64_t tick{};
      auto ret = tick_receiver_->fetch_newest_and_clear(tick);
      if (ret) {
        tick_count = tick;
      }
    } else {
      return; // no tick, no running.
    }

    // resource mode, sync by vision lane
    if (perception_vision_lane_receiver_->empty()) {
      return;
    }

    maf_system_manager::RunningModeEnum mode;
    mode.value = maf_system_manager::RunningModeEnum::PILOT;
    monitor_->set_node_status_running_mode(mode);

    double start_time = MTIME()->timestamp().ms();
    MSD_LOG(WARN, "(%s)tick_count: %lu", __FUNCTION__, tick_count);

    MSD_LOG(WARN, "VERSION: 2021-07-17");

    bool succeed = run_once();

    auto &ldp_result = PlanningContext::Instance()->mutable_ldp_result();
    auto &elk_result = PlanningContext::Instance()->mutable_elk_result();
    if (reset_) {
      ldp_result.ldw_info.reset_flag = true;
      ldp_result.ldw_info.trigger_time = 0;
      ldp_result.ldw_info.ldw_status = LdwInfo::LDW_STATUS_NONE;
      ldp_result.ldw_info.last_warning = LdwInfo::LDW_STATUS_NONE;

      ldp_result.lka_info.reset_flag = true;
      ldp_result.lka_info.trigger_time = 0;
      ldp_result.lka_info.lka_status = LkaInfo::LKA_STATUS_NONE;
      ldp_result.lka_info.last_warning = LkaInfo::LKA_STATUS_NONE;

      elk_result.elk_info.reset_flag = true;
      elk_result.elk_info.trigger_time = 0;
      elk_result.elk_info.lka_status = LkaInfo::LKA_STATUS_NONE;
      elk_result.elk_info.last_warning = LkaInfo::LKA_STATUS_NONE;

      reset_ = false;
      MSD_LOG(WARN, "reset ldp planning");
    }

    if (nullptr != planning_output_callback_) {
      if ((ldp_state_machine_info_.ldp_state_machine_status ==
               LdpStateMachineInfo::LDP_STATE_MACHINE_ACTIVE ||
           elk_result.elk_info.lka_status != LkaInfo::LKA_STATUS_NONE) &&
          succeed) {
        auto maf_planning_output = generate_planning_output();

        planning_output_callback_({tick_count, succeed}, maf_planning_output,
                                  "");
      }
    }

    if (nullptr != planning_ldp_output_callback_ && succeed) {
      auto ldp_output = generate_ldp_output();
      planning_ldp_output_callback_(ldp_output);
    }

    double end_time = MTIME()->timestamp().ms();
    MSD_LOG(INFO, "time_cost, total time: %f", end_time - start_time);
  }

  void set_callback(MSDPlanningOutputCallback callback) {
    planning_output_callback_ = callback;
  }

  void set_sync_callback(NPPHeaderLoggerCallback callback) {
    // not used
  }

  void set_callback(MSDPlanningLdpOutputCallback callback) {
    planning_ldp_output_callback_ = callback;
  }

  void set_callback(MSDPlanningTriggerCallback callback) {
    // not used
  }

  void
  set_callback(std::function<void(const maf_planning::SBPRequest &sbp_request)>
                   &callback) {
    // not used
  }

  void set_callback(MSDPlanningInfoCallback callback) {
    // not used
  }

  void set_callback(MSDMdebugCallback callback) {
    // not used
  }

  std::string generate_ldp_output() {
    const auto &ldp_result = PlanningContext::Instance()->ldp_result();
    const auto &elk_result = PlanningContext::Instance()->elk_result();

    // calc lateral velocity
    double ldp_dlc = ldp_result.lka_info.ldp_dlc;
    if (elk_re_enable_ || elk_ot_enable_ || elk_oc_enable_) {
      ldp_dlc = elk_result.elk_info.ldp_dlc;
    }
    double delta_ldp_dlc = ldp_dlc - last_ldp_dlc_;
    if (std::abs(delta_ldp_dlc) > 0.001) {
      lateral_velocity_ = delta_ldp_dlc / 0.1;
    }
    last_ldp_dlc_ = ldp_dlc;

    std::string ldp_output_string{};
    auto result = mjson::Json(mjson::Json::object());
    result["timestamp"] = mjson::Json(double(MTIME()->timestamp().us()));
    result["ldw_tlc"] = mjson::Json(ldp_result.ldw_info.ldw_tlc);
    result["ldp_dlc"] = mjson::Json(ldp_result.lka_info.ldp_dlc);
    result["ldp_tlc"] = mjson::Json(ldp_result.lka_info.ldp_tlc);
    result["machine_status"] =
        mjson::Json((int)ldp_state_machine_info_.ldp_state_machine_status);
    result["ldw_only"] = mjson::Json(ldw_only_);
    result["ldp_reset_flag"] = mjson::Json(ldp_result.lka_info.reset_flag);
    result["ldw_reset_flag"] = mjson::Json(ldp_result.ldw_info.reset_flag);
    result["ldp_lane_cross"] = mjson::Json(ldp_lane_cross_);
    result["left_track_id"] = mjson::Json(left_track_id_);
    result["right_track_id"] = mjson::Json(right_track_id_);
    result["is_left_virtual_lane"] = mjson::Json(is_left_virtual_lane_);
    result["is_right_virtual_lane"] = mjson::Json(is_right_virtual_lane_);
    result["left_light_activated"] = mjson::Json(left_light_activated_);
    result["right_light_activated"] = mjson::Json(right_light_activated_);
    result["left_lane_length_unsuppressed_count"] =
        mjson::Json(left_lane_length_unsuppressed_count_);
    result["right_lane_length_unsuppressed_count"] =
        mjson::Json(right_lane_length_unsuppressed_count_);
    result["is_left_lane_length_suppressed"] =
        mjson::Json(is_left_lane_length_suppressed_);
    result["is_right_lane_length_suppressed"] =
        mjson::Json(is_right_lane_length_suppressed_);
    result["lane_width_suppressed_count"] =
        mjson::Json(lane_width_suppressed_count_);
    result["lane_width_unsuppressed_count"] =
        mjson::Json(lane_width_unsuppressed_count_);
    result["lane_width_suppressed"] = mjson::Json(is_lane_width_suppressed_);
    result["lateral_velocity"] = mjson::Json(lateral_velocity_);
    result["is_left_road_edge"] = mjson::Json(is_left_road_edge_);
    result["is_right_road_edge"] = mjson::Json(is_right_road_edge_);
    result["is_left_solid_line"] = mjson::Json(is_left_solid_line_);
    result["is_right_solid_line"] = mjson::Json(is_right_solid_line_);
    result["elk_status"] = mjson::Json((int)(elk_result.elk_info.lka_status));
    int elk_re_status = (int)((!elk_absm_status_ && elk_re_enable_)
                                  ? elk_result.elk_info.lka_status
                                  : LkaInfo::LKA_STATUS_NONE);
    result["elk_re_status"] = mjson::Json(elk_re_status);
    int elk_ot_status = (int)((!elk_absm_status_ && elk_ot_enable_)
                                  ? elk_result.elk_info.lka_status
                                  : LkaInfo::LKA_STATUS_NONE);
    result["elk_ot_status"] = mjson::Json(elk_ot_status);
    int elk_oc_status = (int)((!elk_absm_status_ && elk_oc_enable_)
                                  ? elk_result.elk_info.lka_status
                                  : LkaInfo::LKA_STATUS_NONE);
    result["elk_oc_status"] = mjson::Json(elk_oc_status);
    result["absm_status"] = mjson::Json(elk_absm_status_);
    result["ldw_status"] = mjson::Json((int)ldp_result.ldw_info.ldw_status);
    result["ldp_status"] =
        mjson::Json((int)((elk_absm_status_ || elk_re_status || elk_ot_status ||
                           elk_oc_status || ldw_only_)
                              ? LkaInfo::LKA_STATUS_NONE
                              : ldp_result.lka_info.lka_status));
    result["elk_re_machine_status"] =
        mjson::Json((int)elk_state_machine_info_.re_status);
    result["elk_ot_machine_status"] =
        mjson::Json((int)elk_state_machine_info_.ot_status);
    result["elk_oc_machine_status"] =
        mjson::Json((int)elk_state_machine_info_.oc_status);
    result["elk_absm_machine_status"] =
        mjson::Json((int)elk_state_machine_info_.absm_status);
    result["elk_re_enable"] = mjson::Json((int)elk_re_enable_);
    result["elk_ot_enable"] = mjson::Json((int)elk_ot_enable_);
    result["elk_oc_enable"] = mjson::Json((int)elk_oc_enable_);
    result["elk_absm_enable"] = mjson::Json((int)elk_absm_enable_);
    result["elk_dlc"] = mjson::Json(elk_result.elk_info.ldp_dlc);
    result["elk_tlc"] = mjson::Json(elk_result.elk_info.ldp_tlc);
    result["elk_reset_flag"] = mjson::Json(elk_result.elk_info.reset_flag);
    result["elk_lane_cross"] = mjson::Json(elk_lane_cross_);
    result["curvature"] = mjson::Json(2.0 * ldp_result.lka_info.d_poly_lka[2]);
    mjson::Json::array left_elk_oncoming_targets{};
    mjson::Json::array left_elk_overtaking_targets{};
    mjson::Json::array right_elk_oncoming_targets{};
    mjson::Json::array right_elk_overtaking_targets{};
    for (const auto &target : left_oncoming_targets_) {
      left_elk_oncoming_targets.emplace_back(mjson::Json(target.track_id));
    }
    for (const auto &target : left_overtaking_targets_) {
      left_elk_overtaking_targets.emplace_back(mjson::Json(target.track_id));
    }
    for (const auto &target : right_oncoming_targets_) {
      right_elk_oncoming_targets.emplace_back(mjson::Json(target.track_id));
    }
    for (const auto &target : right_overtaking_targets_) {
      right_elk_overtaking_targets.emplace_back(mjson::Json(target.track_id));
    }
    result["left_elk_oncoming_targets"] =
        mjson::Json(left_elk_oncoming_targets);
    result["left_elk_overtaking_targets"] =
        mjson::Json(left_elk_overtaking_targets);
    result["right_elk_oncoming_targets"] =
        mjson::Json(right_elk_oncoming_targets);
    result["right_elk_overtaking_targets"] =
        mjson::Json(right_elk_overtaking_targets);

    result["left_bsd_activated"] = mjson::Json(left_bsd_activated_);
    result["right_bsd_activated"] = mjson::Json(right_bsd_activated_);
    result["left_lca_activated"] = mjson::Json(left_lca_activated_);
    result["right_lca_activated"] = mjson::Json(right_lca_activated_);

    result["left_bsd_activated_count"] = mjson::Json(left_bsd_activated_count_);
    result["right_bsd_activated_count"] =
        mjson::Json(right_bsd_activated_count_);
    result["left_lca_activated_count"] = mjson::Json(left_lca_activated_count_);
    result["right_lca_activated_count"] =
        mjson::Json(right_lca_activated_count_);

    int vision_lane_header_latency_ms = 0;
    int vision_lane_sensor_latency_ms = 0;
    vision_lane_header_latency_ms =
        MTIME()->timestamp().ms() - last_vision_lane_header_timestamp_ms_;
    vision_lane_sensor_latency_ms =
        MTIME()->timestamp().ms() - last_vision_lane_sensor_timestamp_ms_;
    result["vision_lane_header_latency_ms"] =
        mjson::Json(vision_lane_header_latency_ms);
    result["vision_lane_sensor_latency_ms"] =
        mjson::Json(vision_lane_sensor_latency_ms);

    ldp_output_string = result.dump();
    return ldp_output_string;
  }

  void reset() { reset_ = true; }

private:
  bool run_once() {
    MLOG_PROFILING("run_once");

    // update world module from msg
    double current_time = MTIME()->timestamp().sec();
    update_world_model(current_time);

    std::string warn_msg, error_msg;
    if (!can_run(current_time, warn_msg, error_msg)) {
      MSD_LOG(ERROR, "(%s)can not run", __FUNCTION__);

      error_msg = "ldp planning can not run, missing inputs: " + error_msg;
      (void)monitor_->try_change_status(node_status::Status::RUNNING_ERROR);
      monitor_->set_node_status_message(error_msg);
      reset();
    } else {
      if (warn_msg.size() > 0) {
        warn_msg = "ldp planning running warnning, inputs delay: " + warn_msg;
        (void)monitor_->try_change_status(node_status::Status::RUNNING_WARNING);
        monitor_->set_node_status_message(warn_msg);
      } else {
        (void)monitor_->try_change_status(node_status::Status::RUNNING);
        monitor_->set_node_status_message("ldp planning running");
      }
    }

    if (!ldp_process()) {
      reset();
    }

    // reset when all functions are disabled, may happen when ess only or before
    // pending
    if (ldp_state_machine_info_.ldp_state_machine_status ==
            LdpStateMachineInfo::LDP_STATE_MACHINE_OFF &&
        elk_state_machine_info_.oc_status ==
            ElkStateMachineInfo::ELK_STATE_MACHINE_DISABLE &&
        elk_state_machine_info_.ot_status ==
            ElkStateMachineInfo::ELK_STATE_MACHINE_DISABLE &&
        elk_state_machine_info_.re_status ==
            ElkStateMachineInfo::ELK_STATE_MACHINE_DISABLE &&
        elk_state_machine_info_.absm_status ==
            ElkStateMachineInfo::ELK_STATE_MACHINE_DISABLE) {
      reset();
    }

    // do not trig all when ess trigged
    if (ess_trigged_) {
      reset();
    }

    return true;
  }
  // CJ:update steer angle,wheel acc, gear
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

  // CJ:update wheel speed
  void update_wheel_report(double current_time) {
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
    }
  }
  // CJ:update button and light
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
        // use turn_signal_type, is real turn light report, blinking when active
        vehicle_light_data.turn_signal.value =
            vehicle_light_report_data.turn_signal_type.value;
        last_feed_time_[FEED_MISC_REPORT] = current_time;
      }

      // if (received_body_report->botton_event_report.available &
      //     BottonEventReport::BOTTON_EVENT_REPORT_DATA) {
      //   const auto button_events =
      //       received_body_report->botton_event_report.botton_event_report_data
      //           .botton_event_array;

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
  }

  void update_vehicle_status(
      double current_time) { // CJ:update steer angle,wheel acc, gear
    update_chassis_report(current_time);
    // CJ:update wheel speed
    update_wheel_report(current_time);
    // CJ:update button and light
    update_body_report(current_time);
  }

  void update_perception_vision_lane(double current_time) {
    while (!perception_vision_lane_receiver_->empty()) {
      std::shared_ptr<maf_perception_interface::RoadLinePerception>
          perception_vision_lane_ptr{};
      auto ret = perception_vision_lane_receiver_->pop_oldest(
          perception_vision_lane_ptr);
      if (!ret) {
        continue;
      }

      perception_vision_lane_ = *perception_vision_lane_ptr;
      last_feed_time_[FEED_VISION_LANE] = current_time;
    }
    MSD_LOG(INFO, "CJ_DEBUG_LDW,from_meta_to_planning_ms: %d",
            (MTIME()->timestamp().us() -
             perception_vision_lane_.meta.sensor_timestamp_us) /
                1000);

    last_vision_lane_header_timestamp_ms_ =
        perception_vision_lane_.header.stamp / 1e6;
    last_vision_lane_sensor_timestamp_ms_ =
        perception_vision_lane_.meta.sensor_timestamp_us / 1e3;
  }

  void update_perception_fusion(double current_time) {
    while (!perception_fusion_receiver_->empty()) {
      std::shared_ptr<maf_perception_interface::PerceptionFusionObjectResult>
          perception_fusion_ptr{};
      auto ret = perception_fusion_receiver_->pop_oldest(perception_fusion_ptr);
      if (!ret) {
        continue;
      }

      perception_fusion_ = *perception_fusion_ptr;
      last_feed_time_[FEED_FUSION] = current_time;
    }
  }

  void update_perception_fusion_aeb(double current_time) {
    while (!perception_fusion_aeb_receiver_->empty()) {
      std::shared_ptr<maf_perception_interface::PerceptionFusionAEBResult>
          perception_fusion_aeb_ptr{};
      auto ret = perception_fusion_aeb_receiver_->pop_oldest(
          perception_fusion_aeb_ptr);
      if (!ret) {
        continue;
      }
      perception_fusion_aeb_ = *perception_fusion_aeb_ptr;
    }

    const auto &reserved_infos =
        perception_fusion_aeb_.perception_fusion_aeb_objects.reserved_infos;
    if (reserved_infos.size() > 14) {
      const auto &bsd_info = reserved_infos[12];
      const auto &lca_info = reserved_infos[14];

      left_bsd_activated_ = false;
      right_bsd_activated_ = false;
      if (bsd_info == "1") {
        left_bsd_activated_ = true;
      } else if (bsd_info == "2") {
        right_bsd_activated_ = true;
      } else if (bsd_info == "3") {
        left_bsd_activated_ = true;
        right_bsd_activated_ = true;
      }

      left_lca_activated_ = false;
      right_lca_activated_ = false;
      if (lca_info == "1") {
        left_lca_activated_ = true;
      } else if (lca_info == "2") {
        right_lca_activated_ = true;
      } else if (lca_info == "3") {
        left_lca_activated_ = true;
        right_lca_activated_ = true;
      }
    }
  }

  void update_world_model(double current_time) {
    MLOG_PROFILING("update_world_model");

    while (!mff_planning_request_receiver_->empty()) {
      std::string request{};
      auto ret = mff_planning_request_receiver_->pop_oldest(request);
      if (!ret) {
        continue;
      }
      MSD_LOG(WARN, "receive mff_planning_request_receiver: %s",
              request.c_str());

      if (request != "") {
        auto request_reader = mjson::Reader(request);

        auto ldp_string =
            request_reader.get<std::string>("ldp_w_status", false, "");
        if (ldp_string != "") {
          auto ldp_reader = mjson::Reader(ldp_string);
          int status = ldp_reader.get<int>("status", false, 0);
          int sensitive_degree =
              ldp_reader.get<int>("sensitive_degree", false, 0);
          ldw_only_ = ldp_reader.get<int>("ldw_only", false, 0);

          ldp_state_machine_info_.ldp_state_machine_status =
              LdpStateMachineInfo::LdpStateMachineStatus(status);
          ldp_sensitive_info_.ldp_sensitive_degree =
              LdpSensitiveInfo::LdpSensitiveDegree(sensitive_degree);
        }

        auto elk_string =
            request_reader.get<std::string>("elk_status", false, "");
        if (elk_string != "") {
          auto elk_reader = mjson::Reader(elk_string);
          int re_status = elk_reader.get<int>("re_status", false, 0);
          int ot_status = elk_reader.get<int>("ot_status", false, 0);
          int oc_status = elk_reader.get<int>("oc_status", false, 0);

          elk_state_machine_info_.re_status =
              ElkStateMachineInfo::ElkStateMachineStatus(re_status);
          elk_state_machine_info_.ot_status =
              ElkStateMachineInfo::ElkStateMachineStatus(ot_status);
          elk_state_machine_info_.oc_status =
              ElkStateMachineInfo::ElkStateMachineStatus(oc_status);
        }

        auto absm_string =
            request_reader.get<std::string>("absm_status", false, "");
        if (absm_string != "") {
          auto absm_reader = mjson::Reader(absm_string);
          int absm_status = absm_reader.get<int>("status", false, 0);

          elk_state_machine_info_.absm_status =
              ElkStateMachineInfo::ElkStateMachineStatus(absm_status);
        }
      }
    }

    while (!planning_ess_status_receiver_->empty()) {
      std::string request{};
      auto ret = planning_ess_status_receiver_->pop_oldest(request);
      if (!ret) {
        continue;
      }

      if (request != "") {
        auto request_reader = mjson::Reader(request);

        int ess_status = request_reader.get<int>("ess_status", false, 0);

        if (ess_status) {
          ess_trigged_ = true;
        } else {
          ess_trigged_ = false;
        }

        last_ess_feed_time_ = current_time;
      }
      if (current_time - last_ess_feed_time_ > kCheckTimeDiffError) {
        ess_trigged_ = false;
      }
    }

    if (!module_status_receiver_->empty()) {
      bool status{true};
      ModuleStatus module_status{};
      auto ret = module_status_receiver_->fetch_newest_and_clear(module_status);
      if (ret) {
        if (ModuleStatus::STOP == module_status.status ||
            ModuleStatus::RUNNING_TAKING_OVER == module_status.detail_status) {
          status = false;
        }

        auto env = std::getenv("WHICH_CAR");
        MSD_LOG(INFO, "WHICH_CAR: %s", env);
        if (env != nullptr && std::strcmp(env, "MKZ_SIM") == 0) {
          status = true;
        }

        last_feed_time_[FEED_VEHICLE_DBW_STATUS] = current_time;
      }
    }

    update_vehicle_status(current_time);

    update_perception_vision_lane(current_time);

    update_perception_fusion(current_time);

    update_perception_fusion_aeb(current_time);
  }

  bool can_run(double current_time, std::string &warn_msg,
               std::string &error_msg) {
    auto to_string = [](FeedType feed_type) -> const char * {
      switch (feed_type) {
      case FEED_VEHICLE_DBW_STATUS:
        return "vehicle_dbw_status";
      case FEED_EGO_STEER_ANGLE:
        return "ego_steer_angle";
      case FEED_WHEEL_SPEED_REPORT:
        return "wheel_speed_report";
      case FEED_EGO_ACC:
        return "ego_acc";
      case FEED_MISC_REPORT:
        return "misc_report";
      case FEED_GEAR_REPORT:
        return "gear_report";
      case FEED_VISION_LANE:
        return "vision_lane";
      case FEED_FUSION:
        return "fusion object";
      default:
        return "unknown type";
      };
    };

    warn_msg.clear();
    error_msg.clear();

    bool res = true;
    for (int i = 0; i < FEED_TYPE_MAX; ++i) {
      const char *feed_type_str = to_string(static_cast<FeedType>(i));

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

  maf_planning::Planning generate_planning_output() {
    maf_planning::Planning msg{};

    auto timestamp = MTIME()->timestamp();

    msg.header.stamp = timestamp.ns();
    msg.header.frame_id = "enu";
    msg.meta.timestamp_us = timestamp.us();
    msg.meta.plan_timestamp_us = timestamp.us();

    auto &ldp_result = PlanningContext::Instance()->ldp_result();
    auto &elk_result = PlanningContext::Instance()->elk_result();
    msg.trajectory.available |= maf_planning::Trajectory::POLYNOMIAL_CURVE;
    msg.trajectory.polynomial_curve.polynomial = ldp_result.lka_info.d_poly_lka;

    // move center line by offset for comfort
    if (ldp_result.lka_info.lka_status == LkaInfo::LKA_STATUS_LEFT_TRIGGERED ||
        elk_result.elk_info.lka_status == LkaInfo::LKA_STATUS_LEFT_TRIGGERED) {
      msg.trajectory.polynomial_curve.polynomial[0] +=
          ldp_result.lka_info.center_offset;
    } else if (ldp_result.lka_info.lka_status ==
                   LkaInfo::LKA_STATUS_RIGHT_TRIGGERED ||
               elk_result.elk_info.lka_status ==
                   LkaInfo::LKA_STATUS_RIGHT_TRIGGERED) {
      msg.trajectory.polynomial_curve.polynomial[0] -=
          ldp_result.lka_info.center_offset;
    }

    return msg;
  }

  // CJ:ldp main algorithm including ldw & lka
  uint64_t last_vision_stamp;
  uint64_t last_vision_header;
  uint64_t last_log_time;

  bool ldp_process() {
    double ego_vel =
        (vehicle_status_.wheel_velocity.wheel_velocity4d.rear_left +
         vehicle_status_.wheel_velocity.wheel_velocity4d.rear_right) /
        2 *
        ConfigurationContext::Instance()
            ->get_vehicle_param()
            .rear_wheel_rolling_radius;

    double rear_axle_to_head =
        ConfigurationContext::Instance()->get_vehicle_param().length -
        ConfigurationContext::Instance()
            ->get_vehicle_param()
            .rear_bumper_to_rear_axle;

    auto const &perception_vision_lane = perception_vision_lane_;

    auto &ldp_result =
        msquare::PlanningContext::Instance()->mutable_ldp_result();
    auto &elk_result =
        msquare::PlanningContext::Instance()->mutable_elk_result();

    auto const &lines = perception_vision_lane.lane_perception;
    auto const &roadedges = perception_vision_lane.road_edge_perception;
    std::vector<double> poly_coef_l{}, poly_coef_r{};
    std::vector<double> roadedge_poly_coef_l{}, roadedge_poly_coef_r{};
    static std::vector<Point2D> traj_l{}, traj_r{};
    traj_l.clear();
    traj_r.clear();
    unsigned int sensitivity = ldp_sensitive_info_.ldp_sensitive_degree;

    double left_lane_start_x = DBL_MAX;
    double left_lane_end_x = DBL_MAX;
    double right_lane_start_x = DBL_MAX;
    double right_lane_end_x = DBL_MAX;
    double lane_width = STD_LANE_WIDTH;

    is_left_virtual_lane_ = true;
    is_right_virtual_lane_ = true;
    is_left_road_edge_ = false;
    is_right_road_edge_ = false;
    is_left_solid_line_ = false;
    is_right_solid_line_ = false;
    left_track_id_ = -1;
    right_track_id_ = -1;

    uint64_t vision_stamp_gap =
        perception_vision_lane.meta.sensor_timestamp_us - last_vision_stamp;
    uint64_t vision_header_gap =
        perception_vision_lane.header.stamp - last_vision_header;
    MSD_LOG(INFO, "CJ_DEBUG_LDW,Vision_meta_stamp:%ld",
            perception_vision_lane.meta.sensor_timestamp_us);
    MSD_LOG(INFO, "CJ_DEBUG_LDW,Vision_stamp_gap_ms:%d",
            vision_stamp_gap / 1000);
    MSD_LOG(INFO, "CJ_DEBUG_LDW,Vision_header_gap_ms:%d",
            vision_header_gap / 1000000);
    MSD_LOG(INFO, "CJ_DEBUG_LDW,from_Vision_stamp_to_header_ms: %d",
            (perception_vision_lane.header.stamp / 1000 -
             perception_vision_lane.meta.sensor_timestamp_us) /
                1000);
    MSD_LOG(INFO, "CJ_DEBUG_LDW,from_Vision_stamp_to_log_stamp: %d",
            (MTIME()->timestamp().us() -
             perception_vision_lane.meta.sensor_timestamp_us) /
                1000);
    last_vision_stamp = perception_vision_lane.meta.sensor_timestamp_us;
    last_vision_header = perception_vision_lane.header.stamp;

    for (int i = 0; i < lines.lanes.size(); i++) {
      const auto &line = lines.lanes[i];
      // check left line
      if (line.is_failed_3d == false && line.is_centerline == false &&
          line.index == -1 &&
          line.camera_source.value ==
              CameraSourceEnum::CAMERA_SOURCE_FRONT_MID &&
          line.points_3d_x.size() > 3) {
        for (int i = 0; i < line.points_3d_x.size(); ++i) {
          traj_l.emplace_back(line.points_3d_x[i], line.points_3d_y[i]);
        }

        left_track_id_ = line.track_id;
        left_lane_start_x = line.points_3d_x.front() - rear_axle_to_head;
        left_lane_end_x = line.points_3d_x.back() - rear_axle_to_head;
        if (lines.reserved_infos.size() > i) {
          std::string err{};
          auto reserved_infos_json =
              mjson::Json::parse(lines.reserved_infos[i], err);
          if (err.empty() && reserved_infos_json.has_key("is_virtual_lane")) {
            if (reserved_infos_json["is_virtual_lane"].bool_value() == true) {
              is_left_virtual_lane_ = true;
            } else {
              is_left_virtual_lane_ = false;
            }
          } else {
            is_left_virtual_lane_ = false;
          }
        } else {
          is_left_virtual_lane_ = false;
        }

        if (!is_left_virtual_lane_) {
          if (line.lane_type.value == LaneTypeEnum::LANE_TYPE_SOLID ||
              line.lane_type.value == LaneTypeEnum::LANE_TYPE_DOUBLE_SOLID) {
            is_left_solid_line_ = true;
          }
        }
      }

      // check right lane
      if (line.is_failed_3d == false && line.is_centerline == false &&
          line.index == 1 &&
          line.camera_source.value ==
              CameraSourceEnum::CAMERA_SOURCE_FRONT_MID &&
          line.points_3d_x.size() > 3) {
        for (int i = 0; i < line.points_3d_x.size(); ++i) {
          traj_r.emplace_back(line.points_3d_x[i], line.points_3d_y[i]);
        }

        right_track_id_ = line.track_id;
        right_lane_start_x = line.points_3d_x.front() - rear_axle_to_head;
        right_lane_end_x = line.points_3d_x.back() - rear_axle_to_head;
        is_right_virtual_lane_ = false;

        if (lines.reserved_infos.size() > i) {
          std::string err{};
          auto reserved_infos_json =
              mjson::Json::parse(lines.reserved_infos[i], err);
          if (err.empty() && reserved_infos_json.has_key("is_virtual_lane")) {
            if (reserved_infos_json["is_virtual_lane"].bool_value() == true) {
              is_right_virtual_lane_ = true;
            } else {
              is_right_virtual_lane_ = false;
            }
          } else {
            is_right_virtual_lane_ = false;
          }
        } else {
          is_right_virtual_lane_ = false;
        }

        if (!is_right_virtual_lane_) {
          if (line.lane_type.value == LaneTypeEnum::LANE_TYPE_SOLID ||
              line.lane_type.value == LaneTypeEnum::LANE_TYPE_DOUBLE_SOLID) {
            is_right_solid_line_ = true;
          }
        }
      }
    }

    if (traj_l.size() < 3 || !Polyfit(traj_l, 3, poly_coef_l) ||
        poly_coef_l.size() != 4) {
      MSD_LOG(WARN, "left road line polyfit error");
      is_left_virtual_lane_ = true;
    }
    if (traj_r.size() < 3 || !Polyfit(traj_r, 3, poly_coef_r) ||
        poly_coef_r.size() != 4) {
      MSD_LOG(WARN, "left road line polyfit error");
      is_right_virtual_lane_ = true;
    }

    // filter elk targets
    left_oncoming_targets_.clear();
    left_overtaking_targets_.clear();
    right_oncoming_targets_.clear();
    right_overtaking_targets_.clear();
    for (const auto &target :
         perception_fusion_.perception_fusion_objects_data) {
      double d_rel = calc_d_rel(target);
      // left lane targets
      if (!is_left_virtual_lane_) {
        double dy = target.relative_position.y -
                    calc_line_y(poly_coef_l, target.relative_position.x);
        if (dy > 0 && dy < near_lane_overlap_thres_) {

          MSD_LOG(
              INFO,
              "ELK_DEBUG: left lane cars, track id: %d, x: %f, y: %f, dy: %f",
              target.track_id, target.relative_position.x,
              target.relative_position.y, dy);
          filter_elk_target(target, d_rel, ego_vel, left_oncoming_targets_,
                            left_overtaking_targets_);
        }
      }

      // right lane cars
      if (!is_right_virtual_lane_) {
        double dy = target.relative_position.y -
                    calc_line_y(poly_coef_r, target.relative_position.x);
        if (dy < 0 && dy > -near_lane_overlap_thres_) {

          MSD_LOG(
              INFO,
              "ELK_DEBUG: right lane cars, track id: %d, x: %f, y: %f, dy: %f",
              target.track_id, target.relative_position.x,
              target.relative_position.y, dy);
          filter_elk_target(target, d_rel, ego_vel, right_oncoming_targets_,
                            right_overtaking_targets_);
        }
      }
    }

    // ignore dash line when only elk road edge can trigger
    is_left_dash_line_ignored_ = false;
    is_right_dash_line_ignored_ = false;
    if (!is_left_virtual_lane_ && !is_left_solid_line_ &&
        ldp_state_machine_info_.ldp_state_machine_status !=
            LdpStateMachineInfo::LdpStateMachineStatus::
                LDP_STATE_MACHINE_ACTIVE &&
        !left_bsd_activated_count_ && !left_lca_activated_count_ &&
        left_oncoming_targets_.empty()) {
      is_left_virtual_lane_ = true;
      is_left_solid_line_ = false;
      is_left_dash_line_ignored_ = true;
    }
    if (!is_right_virtual_lane_ && !is_right_solid_line_ &&
        ldp_state_machine_info_.ldp_state_machine_status !=
            LdpStateMachineInfo::LdpStateMachineStatus::
                LDP_STATE_MACHINE_ACTIVE &&
        !right_bsd_activated_count_ && !right_lca_activated_count_ &&
        right_oncoming_targets_.empty()) {
      is_right_virtual_lane_ = true;
      is_right_solid_line_ = false;
      is_right_dash_line_ignored_ = true;
    }

    for (const auto &roadedge : roadedges.road_edges) {
      // check left road edges
      if (roadedge.is_failed_3d == false && roadedge.index == -1 &&
          roadedge.camera_source.value ==
              CameraSourceEnum::CAMERA_SOURCE_FRONT_MID &&
          roadedge.points_3d_x.size() > 3) {
        if (is_left_virtual_lane_) {
          MSD_LOG(INFO, "use left road edge");
          is_left_virtual_lane_ = false;
          left_track_id_ = roadedge.track_id;
          left_lane_start_x = roadedge.points_3d_x.front() - rear_axle_to_head;
          left_lane_end_x = roadedge.points_3d_x.back() - rear_axle_to_head;

          traj_l.clear();
          for (int i = 0; i < roadedge.points_3d_x.size(); ++i) {
            traj_l.emplace_back(roadedge.points_3d_x[i],
                                roadedge.points_3d_y[i]);
          }

          poly_coef_l.clear();
          if (traj_l.size() < 3 || !Polyfit(traj_l, 3, poly_coef_l) ||
              poly_coef_l.size() != 4) {
            MSD_LOG(WARN, "left road edge polyfit error");
            is_left_virtual_lane_ = true;
          }

          if (!is_left_virtual_lane_) {
            is_left_road_edge_ = true;
          }
        }
      }

      // check right road edges
      if (roadedge.is_failed_3d == false && roadedge.index == 1 &&
          roadedge.camera_source.value ==
              CameraSourceEnum::CAMERA_SOURCE_FRONT_MID &&
          roadedge.points_3d_x.size() > 3) {
        if (is_right_virtual_lane_) {
          MSD_LOG(INFO, "use right road edge");
          is_right_virtual_lane_ = false;
          right_track_id_ = roadedge.track_id;
          right_lane_start_x = roadedge.points_3d_x.front() - rear_axle_to_head;
          right_lane_end_x = roadedge.points_3d_x.back() - rear_axle_to_head;

          traj_r.clear();
          for (int i = 0; i < roadedge.points_3d_x.size(); ++i) {
            traj_r.emplace_back(roadedge.points_3d_x[i],
                                roadedge.points_3d_y[i]);
          }

          poly_coef_r.clear();
          if (traj_r.size() < 3 || !Polyfit(traj_r, 3, poly_coef_r) ||
              poly_coef_r.size() != 4) {
            MSD_LOG(WARN, "right road edge polyfit error");
            is_right_virtual_lane_ = true;
          }

          if (!is_right_virtual_lane_) {
            is_right_road_edge_ = true;
          }
        }
      }
    }

    // check lane
    ldp_lane_cross_ = false;
    elk_lane_cross_ = false;
    if (ldp_result.lka_info.lka_status != LkaInfo::LKA_STATUS_NONE) {
      if (ldp_result.lka_info.lka_left_id != left_track_id_ &&
          ldp_result.lka_info.lka_status ==
              LkaInfo::LKA_STATUS_LEFT_TRIGGERED) {
        ldp_result.lka_info.lka_status = LkaInfo::LKA_STATUS_NONE;
        ldp_result.lka_info.trigger_time = 0;
        ldp_lane_cross_ = true;
        MSD_LOG(INFO, "CJ_DEBUG_LKA1,Left_Cross or missing!");
        MSD_LOG(INFO, "CJ_DEBUG_LKA1,LKA_Status:%d,lka_id:%d,line_id:%d",
                ldp_result.lka_info.lka_status, ldp_result.lka_info.lka_left_id,
                left_track_id_);
      }

      if (ldp_result.lka_info.lka_right_id != right_track_id_ &&
          ldp_result.lka_info.lka_status ==
              LkaInfo::LKA_STATUS_RIGHT_TRIGGERED) {
        ldp_result.lka_info.lka_status = LkaInfo::LKA_STATUS_NONE;
        ldp_result.lka_info.trigger_time = 0;
        ldp_lane_cross_ = true;
        MSD_LOG(INFO, "CJ_DEBUG_LKA1,Right_Cross or missing!");
        MSD_LOG(INFO, "CJ_DEBUG_LKA1,LKA_Status:%d,lka_id:%d,line_id:%d",
                ldp_result.lka_info.lka_status,
                ldp_result.lka_info.lka_right_id, right_track_id_);
      }
    }
    ldp_result.lka_info.lka_left_id = left_track_id_;
    ldp_result.lka_info.lka_right_id = right_track_id_;

    if (elk_result.elk_info.lka_status != LkaInfo::LKA_STATUS_NONE) {
      if (elk_result.elk_info.lka_left_id != left_track_id_ &&
          elk_result.elk_info.lka_status ==
              LkaInfo::LKA_STATUS_LEFT_TRIGGERED) {
        elk_result.elk_info.lka_status = LkaInfo::LKA_STATUS_NONE;
        elk_result.elk_info.trigger_time = 0;
        elk_lane_cross_ = true;
        MSD_LOG(INFO, "G_DEBUG_ELK,Left_Cross or missing!");
        MSD_LOG(INFO, "G_DEBUG_ELK,ELK_Status:%d,elk_id:%d,line_id:%d",
                elk_result.elk_info.lka_status, elk_result.elk_info.lka_left_id,
                left_track_id_);
      }

      if (elk_result.elk_info.lka_right_id != right_track_id_ &&
          elk_result.elk_info.lka_status ==
              LkaInfo::LKA_STATUS_RIGHT_TRIGGERED) {
        elk_result.elk_info.lka_status = LkaInfo::LKA_STATUS_NONE;
        elk_result.elk_info.trigger_time = 0;
        elk_lane_cross_ = true;
        MSD_LOG(INFO, "G_DEBUG_ELK,Right_Cross or missing!");
        MSD_LOG(INFO, "G_DEBUG_ELK,ELK_Status:%d,elk_id:%d,line_id:%d",
                elk_result.elk_info.lka_status,
                elk_result.elk_info.lka_right_id, right_track_id_);
      }
    }
    elk_result.elk_info.lka_left_id = left_track_id_;
    elk_result.elk_info.lka_right_id = right_track_id_;

    // both exist, check if ignore lane width
    if (!is_left_virtual_lane_ && !is_right_virtual_lane_) {
      lane_width = poly_coef_l[0] - poly_coef_r[0];
      if (lane_width > IGNORE_LANE_WIDTH) {
        if (std::abs(poly_coef_l[0]) < std::abs(poly_coef_r[0])) {
          is_right_virtual_lane_ = true;
        } else {
          is_left_virtual_lane_ = true;
        }
        MSD_LOG(INFO, "G_DEBUG, ignore lane width!");
      }
    }

    if (is_left_virtual_lane_ && is_right_virtual_lane_) {
      // both missing
      return false;
    } else if (!is_left_virtual_lane_ && !is_right_virtual_lane_) {
      // both exist, do notion
    } else if (!is_left_virtual_lane_) {
      // left exist
      poly_coef_r = poly_coef_l;
      poly_coef_r[0] = poly_coef_l[0] - STD_LANE_WIDTH;
    } else {
      // right exist
      poly_coef_l = poly_coef_r;
      poly_coef_l[0] = poly_coef_r[0] + STD_LANE_WIDTH;
    }

    lane_width = poly_coef_l[0] - poly_coef_r[0];

    // calc d_poly_lka
    if (lane_width > MAX_LANE_WIDTH) {
      if (std::abs(poly_coef_l[0]) < std::abs(poly_coef_r[0])) {
        ldp_result.lka_info.d_poly_lka = poly_coef_l;
        ldp_result.lka_info.d_poly_lka[0] -= MAX_LANE_WIDTH / 2.0;
      } else {
        ldp_result.lka_info.d_poly_lka = poly_coef_r;
        ldp_result.lka_info.d_poly_lka[0] += MAX_LANE_WIDTH / 2.0;
      }
    } else {
      for (int i = 0; i < poly_coef_l.size(); ++i) {
        ldp_result.lka_info.d_poly_lka[i] =
            (poly_coef_l[i] + poly_coef_r[i]) / 2;
      }
    }
    MSD_LOG(INFO, "CJ_DEBUG_LKA,C0:%f,C1:%f", ldp_result.lka_info.d_poly_lka[0],
            ldp_result.lka_info.d_poly_lka[1]); // CJ:wheel position in car
                                                // coord 0:FL,1:FR,2:RL,3:RR

    // calc wheels position
    vector<Point2D> wheels(4);
    wheels[0].x =
        ConfigurationContext::Instance()->get_vehicle_param().wheel_base;
    wheels[0].y =
        ConfigurationContext::Instance()->get_vehicle_param().width / 2;
    wheels[1].x =
        ConfigurationContext::Instance()->get_vehicle_param().wheel_base;
    wheels[1].y =
        -ConfigurationContext::Instance()->get_vehicle_param().width / 2;
    wheels[2].x = 0;
    wheels[2].y =
        ConfigurationContext::Instance()->get_vehicle_param().width / 2;
    wheels[3].x = 0;
    wheels[3].y =
        -ConfigurationContext::Instance()->get_vehicle_param().width / 2;

    vector<double> wheel_dist2line_l, wheel_dist2line_r;
    double relative_theta_l = -std::atan(poly_coef_l[1]);
    double relative_theta_r = -std::atan(poly_coef_r[1]);
    double left_min_dist = 10, right_min_dist = 10;
    MSD_LOG(INFO, "CJ_DEBUG_LKA:theta_l:%f,theta_r:%f", relative_theta_l,
            relative_theta_r);
    // CJ:find nearest point of each line
    for (auto p : wheels) {
      double left_line_y = calc_line_y(poly_coef_l, p.x);
      double right_line_y = calc_line_y(poly_coef_r, p.x);

      if (std::fabs(left_min_dist) > std::fabs(left_line_y - p.y)) {
        left_min_dist = left_line_y - p.y;
      }
      if (std::fabs(right_min_dist) > std::fabs(right_line_y - p.y)) {
        right_min_dist = right_line_y - p.y;
      }
      wheel_dist2line_l.push_back(p.y - left_line_y);
      wheel_dist2line_r.push_back(p.y - right_line_y);
      /*LDP test log*/
      MSD_LOG(INFO, "CJ_DEBUG_LDW1");
      MSD_LOG(INFO, "CJ_DEBUG_LDW1,dist2left:%.2f", wheel_dist2line_l.back());
      MSD_LOG(INFO, "CJ_DEBUG_LDW1,dist2right:%.2f", wheel_dist2line_r.back());
    }

    int ldw_flag = 0, last_ldw_flag = 0;
    if (std::fabs(left_min_dist) < std::fabs(right_min_dist)) {
      if (ldp_result.ldw_info.last_warning == LdwInfo::LDW_STATUS_NONE) {
        last_ldw_flag = 0;
      } else if (ldp_result.ldw_info.last_warning ==
                 LdwInfo::LDW_STATUS_LEFT_TRIGGERED) {
        last_ldw_flag = 1;
      } else {
        last_ldw_flag = 2;
      }
      if (ldp_result.ldw_info.ldw_status == LdwInfo::LDW_STATUS_NONE) {
        ldw_flag = 0;
      } else if (ldp_result.ldw_info.ldw_status ==
                 LdwInfo::LDW_STATUS_LEFT_TRIGGERED) {
        ldw_flag = 1;
      } else {
        ldw_flag = 2;
      }
      MSD_LOG(INFO, "CJ_DEBUG_LDW,Main Left,Left ID:%d,Right ID: %d",
              left_track_id_, right_track_id_);
      if (ldw_warning_decide(ego_vel, relative_theta_l, wheel_dist2line_l,
                             left_min_dist, ldp_result, last_ldw_flag, ldw_flag,
                             sensitivity,
                             std::max(poly_coef_l[2] + poly_coef_r[2], 0.0),
                             lane_width) == true) {
        ldp_result.ldw_info.ldw_status = LdwInfo::LDW_STATUS_LEFT_TRIGGERED;
      }
    } else { // CJ: mirror in lateral direction to unify sign in both sides
      vector<double> wheel_dist2line_r_mirror(4);
      for (int i = 0; i < 4; ++i) {
        wheel_dist2line_r_mirror[i] = -wheel_dist2line_r[i];
      }
      if (ldp_result.ldw_info.last_warning == LdwInfo::LDW_STATUS_NONE) {
        last_ldw_flag = 0;
      } else if (ldp_result.ldw_info.last_warning ==
                 LdwInfo::LDW_STATUS_LEFT_TRIGGERED) {
        last_ldw_flag = 2;
      } else {
        last_ldw_flag = 1;
      }

      if (ldp_result.ldw_info.ldw_status == LdwInfo::LDW_STATUS_NONE) {
        ldw_flag = 0;
      } else if (ldp_result.ldw_info.ldw_status ==
                 LdwInfo::LDW_STATUS_LEFT_TRIGGERED) {
        ldw_flag = 2;
      } else {
        ldw_flag = 1;
      }
      MSD_LOG(INFO, "CJ_DEBUG_LDW,Main Right, Left ID:%d,Right ID: %d",
              left_track_id_, right_track_id_);
      if (ldw_warning_decide(ego_vel, -relative_theta_r,
                             wheel_dist2line_r_mirror, -right_min_dist,
                             ldp_result, last_ldw_flag, ldw_flag, sensitivity,
                             std::max(-(poly_coef_l[2] + poly_coef_r[2]), 0.0),
                             lane_width) == true) {
        ldp_result.ldw_info.ldw_status = LdwInfo::LDW_STATUS_RIGHT_TRIGGERED;
      }
    }
    if (ldp_result.ldw_info.ldw_status != LdwInfo::LDW_STATUS_NONE) {
      ldp_result.ldw_info.last_warning = ldp_result.ldw_info.ldw_status;
      ldp_result.ldw_info.trigger_time += 100;
    }

    MSD_LOG(INFO, "CJ_DEBUG_LDW,State: %d", ldp_result.ldw_info.ldw_status);
    MSD_LOG(INFO, "CJ_DEBUG_LDW,DUR: %d", ldp_result.ldw_info.trigger_time);

    // CJ: LKA process
    int last_lka_flag = 0;
    double C0 = ldp_result.lka_info.d_poly_lka[0];
    if (std::fabs(left_min_dist) < std::fabs(right_min_dist)) {
      if (ldp_result.lka_info.last_warning == LkaInfo::LKA_STATUS_NONE) {
        last_lka_flag = 0;
      } else if (ldp_result.lka_info.last_warning ==
                 LkaInfo::LKA_STATUS_LEFT_TRIGGERED) {
        last_lka_flag = 1; // CJ:main side
      } else {
        last_lka_flag = 2; // CJ:other side
      }
      if (lka_warning_decide(ego_vel, relative_theta_l, wheel_dist2line_l,
                             left_min_dist, ldp_result.lka_info, last_lka_flag,
                             std::max(poly_coef_l[2] + poly_coef_r[2], 0.0),
                             lane_width, C0,
                             ldp_planner_config_.lka_config) == true) {
        ldp_result.lka_info.lka_status = LkaInfo::LKA_STATUS_LEFT_TRIGGERED;
      }
      MSD_LOG(INFO, "L_DEBUG,Left state: %d", ldp_result.lka_info.lka_status);
    } else { // CJ: mirror in lateral direction to unify sign in both sides
      vector<double> wheel_dist2line_r_mirror(4);
      for (int i = 0; i < 4; ++i) {
        wheel_dist2line_r_mirror[i] = -wheel_dist2line_r[i];
        ;
      }
      if (ldp_result.lka_info.last_warning == LkaInfo::LKA_STATUS_NONE) {
        last_lka_flag = 0;
      } else if (ldp_result.lka_info.last_warning ==
                 LkaInfo::LKA_STATUS_LEFT_TRIGGERED) {
        last_lka_flag = 2;
      } else {
        last_lka_flag = 1;
      }

      if (lka_warning_decide(
              ego_vel, -relative_theta_r, wheel_dist2line_r_mirror,
              -right_min_dist, ldp_result.lka_info, last_lka_flag,
              std::max(-(poly_coef_l[2] + poly_coef_r[2]), 0.0), lane_width, C0,
              ldp_planner_config_.lka_config) == true) {
        ldp_result.lka_info.lka_status = LkaInfo::LKA_STATUS_RIGHT_TRIGGERED;
      }
      MSD_LOG(INFO, "L_DEBUG,Right state: %d", ldp_result.lka_info.lka_status);
    }
    if (ldp_result.lka_info.lka_status != LkaInfo::LKA_STATUS_NONE) {
      ldp_result.lka_info.last_warning = ldp_result.lka_info.lka_status;
      ldp_result.lka_info.trigger_time += 100;
    }

    // ELK process
    auto elk_config = ldp_planner_config_.elk_config;
    if (elk_absm_enable_) {
      elk_config = ldp_planner_config_.absm_config;
    }

    int last_elk_flag = 0;
    if (std::fabs(left_min_dist) < std::fabs(right_min_dist)) {
      if (elk_result.elk_info.last_warning == LkaInfo::LKA_STATUS_NONE) {
        last_elk_flag = 0;
      } else if (elk_result.elk_info.last_warning ==
                 LkaInfo::LKA_STATUS_LEFT_TRIGGERED) {
        last_elk_flag = 1; // CJ:main side
      } else {
        last_elk_flag = 2; // CJ:other side
      }
      if (lka_warning_decide(ego_vel, relative_theta_l, wheel_dist2line_l,
                             left_min_dist, elk_result.elk_info, last_elk_flag,
                             std::max(poly_coef_l[2] + poly_coef_r[2], 0.0),
                             lane_width, C0, elk_config) == true) {
        elk_result.elk_info.lka_status = LkaInfo::LKA_STATUS_LEFT_TRIGGERED;
      }
      MSD_LOG(INFO, "L_DEBUG, ELK Left state: %d",
              elk_result.elk_info.lka_status);
    } else { // CJ: mirror in lateral direction to unify sign in both sides
      vector<double> wheel_dist2line_r_mirror(4);
      for (int i = 0; i < 4; ++i) {
        wheel_dist2line_r_mirror[i] = -wheel_dist2line_r[i];
        ;
      }
      if (elk_result.elk_info.last_warning == LkaInfo::LKA_STATUS_NONE) {
        last_elk_flag = 0;
      } else if (elk_result.elk_info.last_warning ==
                 LkaInfo::LKA_STATUS_LEFT_TRIGGERED) {
        last_elk_flag = 2;
      } else {
        last_elk_flag = 1;
      }

      if (lka_warning_decide(ego_vel, -relative_theta_r,
                             wheel_dist2line_r_mirror, -right_min_dist,
                             elk_result.elk_info, last_elk_flag,
                             std::max(-(poly_coef_l[2] + poly_coef_r[2]), 0.0),
                             lane_width, C0, elk_config) == true) {
        elk_result.elk_info.lka_status = LkaInfo::LKA_STATUS_RIGHT_TRIGGERED;
      }
      MSD_LOG(INFO, "L_DEBUG, ELK Right state: %d",
              elk_result.elk_info.lka_status);
    }
    if (elk_result.elk_info.lka_status != LkaInfo::LKA_STATUS_NONE) {
      elk_result.elk_info.last_warning = elk_result.elk_info.lka_status;
      elk_result.elk_info.trigger_time += 100;
    }

    // turn light suppressed
    left_light_activated_ = false;
    right_light_activated_ = false;
    static int left_light_activated_count = 0;
    static int right_light_activated_count = 0;
    const int light_activated_count = 30; // 10hz, 3s

    if (vehicle_status_.vehicle_light.vehicle_light_data.turn_signal.value ==
        TurnSignalType::LEFT) {
      left_light_activated_count = light_activated_count;
    }
    if (vehicle_status_.vehicle_light.vehicle_light_data.turn_signal.value ==
        TurnSignalType::RIGHT) {
      right_light_activated_count = light_activated_count;
    }
    if (left_light_activated_count > 0) {
      left_light_activated_ = true;
    }
    if (right_light_activated_count > 0) {
      right_light_activated_ = true;
    }
    left_light_activated_count = std::max(left_light_activated_count - 1, 0);
    right_light_activated_count = std::max(right_light_activated_count - 1, 0);

    // lane length suppressed
    double lane_length_suppressed_thres = std::max(15.0, 1.0 * ego_vel);

    if (!is_left_virtual_lane_) {
      // suppressed when lane too short or far away
      if (left_lane_end_x < lane_length_suppressed_thres ||
          left_lane_start_x > lane_start_length_thres) {
        is_left_lane_length_suppressed_ = true;
        left_lane_length_unsuppressed_count_ =
            lane_length_unsuppressed_count_thres;
      }
    } else {
      // suppressed when lane not exist
      if (!is_left_dash_line_ignored_) {
        is_left_lane_length_suppressed_ = true;
        left_lane_length_unsuppressed_count_ =
            lane_length_unsuppressed_count_thres;
      }
    }
    if (is_left_lane_length_suppressed_) {
      // check lane > 20m for 3s
      if (left_lane_end_x > lane_end_length_unsuppressed_thres &&
          left_lane_start_x < lane_start_length_thres) {
        left_lane_length_unsuppressed_count_ =
            std::max(left_lane_length_unsuppressed_count_ - 1, 0);
      } else {
        left_lane_length_unsuppressed_count_ =
            lane_length_unsuppressed_count_thres;
      }
    }
    if (left_lane_length_unsuppressed_count_ == 0) {
      is_left_lane_length_suppressed_ = false;
    }

    // right lane
    if (!is_right_virtual_lane_) {
      if (right_lane_end_x < lane_length_suppressed_thres ||
          right_lane_start_x > lane_start_length_thres) {
        is_right_lane_length_suppressed_ = true;
        right_lane_length_unsuppressed_count_ =
            lane_length_unsuppressed_count_thres;
      }
    } else {
      // suppressed when lane not exist
      if (!is_right_dash_line_ignored_) {
        is_right_lane_length_suppressed_ = true;
        right_lane_length_unsuppressed_count_ =
            lane_length_unsuppressed_count_thres;
      }
    }
    if (is_right_lane_length_suppressed_) {
      if (right_lane_end_x > lane_end_length_unsuppressed_thres &&
          right_lane_start_x < lane_start_length_thres) {
        right_lane_length_unsuppressed_count_ =
            std::max(right_lane_length_unsuppressed_count_ - 1, 0);
      } else {
        right_lane_length_unsuppressed_count_ =
            lane_length_unsuppressed_count_thres;
      }
    }
    if (right_lane_length_unsuppressed_count_ == 0) {
      is_right_lane_length_suppressed_ = false;
    }

    // lane width suppressed
    if (!is_left_virtual_lane_ && !is_right_virtual_lane_) {
      if (lane_width < narrow_lane_width_suppressed_thres ||
          lane_width > wide_lane_width_suppressed_thres) {
        lane_width_suppressed_count_ =
            std::max(lane_width_suppressed_count_ - 1, 0);
      } else {
        lane_width_suppressed_count_ = lane_width_suppressed_count_thres;
      }
      if (lane_width_suppressed_count_ == 0) {
        is_lane_width_suppressed_ = true;
        lane_width_unsuppressed_count_ = lane_width_unsuppressed_count_thres;
      }
      if (is_lane_width_suppressed_) {
        if (lane_width < wide_lane_width_unsuppressed_thres &&
            lane_width > narrow_lane_width_unsuppressed_thres) {
          lane_width_unsuppressed_count_ =
              std::max(lane_width_unsuppressed_count_ - 1, 0);
        } else {
          lane_width_unsuppressed_count_ = lane_width_unsuppressed_count_thres;
        }

        if (lane_width_unsuppressed_count_ == 0) {
          is_lane_width_suppressed_ = false;
        }
      }
    } else {
      lane_width_suppressed_count_ = lane_width_suppressed_count_thres;
      lane_width_unsuppressed_count_ = 0;
      is_lane_width_suppressed_ = false;
    }

    // supress triggers for lane not exit, lane length and width
    bool left_lane_suppressed = is_left_virtual_lane_ ||
                                is_left_lane_length_suppressed_ ||
                                is_lane_width_suppressed_;
    bool right_lane_suppressed = is_right_virtual_lane_ ||
                                 is_right_lane_length_suppressed_ ||
                                 is_lane_width_suppressed_;
    // supress ldw trigger
    if (((left_lane_suppressed || left_light_activated_) &&
         ldp_result.ldw_info.ldw_status ==
             LdwInfo::LDW_STATUS_LEFT_TRIGGERED) ||
        ((right_lane_suppressed || right_light_activated_) &&
         ldp_result.ldw_info.ldw_status ==
             LdwInfo::LDW_STATUS_RIGHT_TRIGGERED)) {
      ldp_result.ldw_info.ldw_status = LdwInfo::LDW_STATUS_NONE;
      ldp_result.ldw_info.last_warning = LdwInfo::LDW_STATUS_NONE;
      ldp_result.ldw_info.trigger_time = 0;
      ldp_result.ldw_info.reset_flag = true;
    }
    // supress lka trigger
    if (((left_lane_suppressed || left_light_activated_) &&
         ldp_result.lka_info.lka_status ==
             LkaInfo::LKA_STATUS_LEFT_TRIGGERED) ||
        ((right_lane_suppressed || right_light_activated_) &&
         ldp_result.lka_info.lka_status ==
             LkaInfo::LKA_STATUS_RIGHT_TRIGGERED)) {
      ldp_result.lka_info.lka_status = LkaInfo::LKA_STATUS_NONE;
      ldp_result.lka_info.last_warning = LkaInfo::LKA_STATUS_NONE;
      ldp_result.lka_info.trigger_time = 0;
      ldp_result.lka_info.reset_flag = true;
      ldp_result.lka_info.straight_time = 0;
    }

    // supress all ldp triggers when not active
    if (ldp_state_machine_info_.ldp_state_machine_status !=
        LdpStateMachineInfo::LdpStateMachineStatus::LDP_STATE_MACHINE_ACTIVE) {
      ldp_result.ldw_info.ldw_status = LdwInfo::LDW_STATUS_NONE;
      ldp_result.ldw_info.last_warning = LdwInfo::LDW_STATUS_NONE;
      ldp_result.ldw_info.trigger_time = 0;
      ldp_result.ldw_info.reset_flag = true;

      ldp_result.lka_info.lka_status = LkaInfo::LKA_STATUS_NONE;
      ldp_result.lka_info.last_warning = LkaInfo::LKA_STATUS_NONE;
      ldp_result.lka_info.trigger_time = 0;
      ldp_result.lka_info.reset_flag = true;
      ldp_result.lka_info.straight_time = 0;
    }

    // supress elk trigger for lane not exit, vehcile light, lane length and
    // width
    if ((left_lane_suppressed && elk_result.elk_info.lka_status ==
                                     LkaInfo::LKA_STATUS_LEFT_TRIGGERED) ||
        (right_lane_suppressed && elk_result.elk_info.lka_status ==
                                      LkaInfo::LKA_STATUS_RIGHT_TRIGGERED)) {
      elk_result.elk_info.lka_status = LkaInfo::LKA_STATUS_NONE;
      elk_result.elk_info.last_warning = LkaInfo::LKA_STATUS_NONE;
      elk_result.elk_info.trigger_time = 0;
      elk_result.elk_info.reset_flag = true;
      elk_result.elk_info.straight_time = 0;
    }

    elk_re_enable_ = (elk_state_machine_info_.re_status ==
                      ElkStateMachineInfo::ELK_STATE_MACHINE_ENABLE);
    elk_ot_enable_ = (elk_state_machine_info_.ot_status ==
                      ElkStateMachineInfo::ELK_STATE_MACHINE_ENABLE);
    elk_oc_enable_ = (elk_state_machine_info_.oc_status ==
                      ElkStateMachineInfo::ELK_STATE_MACHINE_ENABLE);
    elk_absm_enable_ = (elk_state_machine_info_.absm_status ==
                        ElkStateMachineInfo::ELK_STATE_MACHINE_ENABLE);
    // enable elk ot for absm
    if (elk_absm_enable_) {
      elk_ot_enable_ = true;
    }

    // disable elk re when line not exist or dash line or light triggered
    if (elk_result.elk_info.lka_status == LkaInfo::LKA_STATUS_LEFT_TRIGGERED) {
      if (!is_left_virtual_lane_ &&
          (is_left_solid_line_ || is_left_road_edge_) &&
          !left_light_activated_) {
      } else {
        elk_re_enable_ = false;
      }
    }
    if (elk_result.elk_info.lka_status == LkaInfo::LKA_STATUS_RIGHT_TRIGGERED) {
      if (!is_right_virtual_lane_ &&
          (is_right_solid_line_ || is_right_road_edge_) &&
          !right_light_activated_) {
      } else {
        elk_re_enable_ = false;
      }
    }

    // disable elk oc when oc object not exist
    if ((elk_result.elk_info.lka_status == LkaInfo::LKA_STATUS_LEFT_TRIGGERED &&
         left_oncoming_targets_.empty()) ||
        (elk_result.elk_info.lka_status ==
             LkaInfo::LKA_STATUS_RIGHT_TRIGGERED &&
         right_oncoming_targets_.empty())) {
      elk_oc_enable_ = false;
    }

    // add debounce for bsd/lca
    if (left_bsd_activated_) {
      left_bsd_activated_count_ = left_bsd_activated_count_thres_;
    }
    if (right_bsd_activated_) {
      right_bsd_activated_count_ = right_bsd_activated_count_thres_;
    }
    if (left_lca_activated_) {
      left_lca_activated_count_ = left_lca_activated_count_thres_;
    }
    if (right_lca_activated_) {
      right_lca_activated_count_ = right_lca_activated_count_thres_;
    }

    // disable elk ot when bsd and lca not activated
    if ((elk_result.elk_info.lka_status == LkaInfo::LKA_STATUS_LEFT_TRIGGERED &&
         (!left_bsd_activated_count_ && !left_lca_activated_count_)) ||
        (elk_result.elk_info.lka_status ==
             LkaInfo::LKA_STATUS_RIGHT_TRIGGERED &&
         (!right_bsd_activated_count_ && !right_lca_activated_count_))) {
      elk_ot_enable_ = false;
    }

    // supress elk trigger when all elk enables are false
    if (!elk_re_enable_ && !elk_oc_enable_ && !elk_ot_enable_) {
      elk_result.elk_info.lka_status = LkaInfo::LKA_STATUS_NONE;
      elk_result.elk_info.last_warning = LkaInfo::LKA_STATUS_NONE;
      elk_result.elk_info.trigger_time = 0;
      elk_result.elk_info.reset_flag = true;
      elk_result.elk_info.straight_time = 0;
    }

    elk_absm_status_ = (elk_absm_enable_ && elk_ot_enable_)
                           ? elk_result.elk_info.lka_status
                           : LkaInfo::LKA_STATUS_NONE;

    left_bsd_activated_count_ = std::max(left_bsd_activated_count_ - 1, 0);
    right_bsd_activated_count_ = std::max(right_bsd_activated_count_ - 1, 0);
    left_lca_activated_count_ = std::max(left_lca_activated_count_ - 1, 0);
    right_lca_activated_count_ = std::max(right_lca_activated_count_ - 1, 0);

    MSD_LOG(INFO, "CJ_DEBUG_LKA,MFF: %d",
            ldp_state_machine_info_.ldp_state_machine_status);
    MSD_LOG(INFO, "CJ_DEBUG_LKA,State: %d", ldp_result.lka_info.lka_status);
    MSD_LOG(INFO, "CJ_DEBUG_LKA,DUR: %d", ldp_result.lka_info.trigger_time);
    MSD_LOG(INFO, "CJ_DEBUG_LKA,Left_id:%d", ldp_result.lka_info.lka_left_id);
    MSD_LOG(INFO, "CJ_DEBUG_LKA,Right_id:%d", ldp_result.lka_info.lka_right_id);
    MSD_LOG(INFO, "CJ_DEBUG_LKA,Straight: %d",
            ldp_result.lka_info.straight_time);
    MSD_LOG(INFO, "CJ_DEBUG_LKA,reset:%d", ldp_result.lka_info.reset_flag);

    return true;
  }

  bool ldw_warning_decide(const double &ego_vel, const double &relative_theta,
                          const vector<double> &wheel_dist2line,
                          double min_dist, LdpResult &ldp_result,
                          int last_ldw_flag, int ldw_flag,
                          const unsigned int sensitivity, const double curve,
                          const double lane_width) {
    bool ldw_warning = false;
    double ldw_tlc = 10;                    // CJ:time to cross lane
    double ldw_dlc = 10;                    // CJ:distance to cross lane
    bool trigger_flag = false;              // CJ: warning start flag
    bool stop_flag = false;                 // CJ: warning stop flag
    double K_ldw_inner_reset_dlc_thr = 0.3; // CJ:m, when ego car comes back
                                            // crossing this line, next warning
                                            // is available again
    double K_ldw_inner_dlc_thr = 0.2;  // CJ:m, when ego car departs crossing
                                       // this line, warning would be triggered
    double K_ldw_inner_tlc_thr = 0.45; // CJ:s, when ego car's tlc is lower than
                                       // this threshold, warning would be
                                       // triggered
    double K_ldw_inner_earliest_dlc = 0.5; // CJ:m, when warning is triggered by
                                           // tlc, distance to lane must be
                                           // smaller than this
    double K_ldw_inner_warning_end_thr = 0.3;
    // CJ:m, when ego car come back awary from line by this distance,
    // warning would be stopped
    double K_ldw_outer_latest_dlc = -0.3; // CJ:m, when ego car has crossed lane
                                          // by this distance, warning would not
                                          // be triggered
    double K_ldw_outer_reset_dlc_thr = 0.3;
    // CJ:m, when ego car continue crossing lane by this distance, next
    // warning in new lane is available
    double K_ldw_outer_warning_end_thr = 0.3; // CJ:m, when ego car continue
                                              // crossing lane by this distance,
                                              // warning would be stopped
    double K_ldw_curv_delay_tlc_max = 0.15;
    // CJ:s, larggest tlc delay in curve road
    double K_ldw_curv_delay_dlc_max = 0.075;
    // CJ:m, larggest dlc delay in curve road
    double K_ldw_curv_delay_tlc_rate = 30; // CJ:s*m, delta_tlc/delta_curvature
    double K_ldw_curv_delay_dlc_rate = 15; // CJ:m^2, dleta_dlc/delta_curvature
    double K_ldw_warning_time_min = 900;   // CJ:ms, minimum warning time of LDW
    double K_ldw_warning_time_max = 1000;  // CJ:ms, maximum warning time of LDW

    double dist_to_cross = 65535.0; // CJ:dist to move befor cross
    double max_y = -100, min_y = 100;

    if (sensitivity == LdpSensitiveInfo::LDP_SENSITIVE_DEGREE_MIDDLE) {
      K_ldw_inner_dlc_thr = 0.3;
      K_ldw_inner_tlc_thr = 0.7;
    } else if (sensitivity == LdpSensitiveInfo::LDP_SENSITIVE_DEGREE_HIGH) {
      K_ldw_inner_dlc_thr = 0.4;
      K_ldw_inner_tlc_thr = 0.95;
    }
    // lower ldw thres when ldp is activated
    if (!ldw_only_) {
      K_ldw_inner_dlc_thr -= 0.1;
      K_ldw_inner_tlc_thr -= 0.25;
    }
    K_ldw_inner_dlc_thr -=
        std::fmin(curve * K_ldw_curv_delay_dlc_rate, K_ldw_curv_delay_dlc_max);
    K_ldw_inner_tlc_thr -=
        std::fmin(curve * K_ldw_curv_delay_tlc_rate, K_ldw_curv_delay_tlc_max);
    K_ldw_inner_dlc_thr -= std::fmax(STD_LANE_WIDTH - lane_width, 0) / 2;
    K_ldw_inner_dlc_thr = std::fmax(K_ldw_inner_dlc_thr, 0.1);
    K_ldw_inner_tlc_thr = std::fmax(K_ldw_inner_tlc_thr, 0.2);
    K_ldw_inner_earliest_dlc -= std::fmax(STD_LANE_WIDTH - lane_width, 0) / 2;

    for (auto dist : wheel_dist2line) {
      if (dist > max_y) {
        max_y = dist;
      }
      if (dist < min_y) {
        min_y = dist;
      }
    }

    ldw_dlc = -max_y; // CJ:set dlc sign, not crossed->positive
                      // crossed->negative
    // check if ldw_dlc is decreasing
    if (ldw_dlc < K_ldw_outer_latest_dlc ||
        ldw_dlc > ldp_result.ldw_info.ldw_dlc) {
      ldp_result.ldw_info.ldw_dlc_dec_time = 0;
    } else {
      ldp_result.ldw_info.ldw_dlc_dec_time += 100;
    }
    MSD_LOG(INFO, "G_DEBUG, ldw_dlc_dec_time: %d",
            ldp_result.ldw_info.ldw_dlc_dec_time);
    ldp_result.ldw_info.ldw_dlc = ldw_dlc;
    if (relative_theta >
        0.1 / std::fmax(ego_vel, 22.2)) // CJ:ego car is driving toward line
    {
      if (ldw_dlc < 0) // CJ:has already crossed line
      {
        ldw_tlc = -1;
      } else // CJ:is about to cross
      {
        dist_to_cross = ldw_dlc / std::tan(relative_theta);
        ldw_tlc = dist_to_cross / std::max(ego_vel, 0.01);
      }
    } else {
      ldw_tlc = 10; // CJ:default value
    }
    ldp_result.ldw_info.ldw_tlc = ldw_tlc;
    MSD_LOG(INFO, "CJ_DEBUG_LDW,ldw_dlc: %f,ldw_tlc: %f", ldw_dlc, ldw_tlc);
    MSD_LOG(INFO, "CJ_DEBUG_LDW,Log_Time_gap_ms: %d",
            (MTIME()->timestamp().us() - last_log_time) / 1000);
    last_log_time = MTIME()->timestamp().us();
    const long ldw_dlc_dec_time_thres = 300; // 0.3s
    trigger_flag =
        ((ldw_dlc < K_ldw_inner_dlc_thr ||
          ldw_tlc < K_ldw_inner_tlc_thr) // CJ:dlc,tlc threshold
         &&
         (ldw_dlc < K_ldw_inner_earliest_dlc &&
          ldw_dlc > K_ldw_outer_latest_dlc) // CJ:within larggest warning region
         && (relative_theta > 0.1 / std::fmax(ego_vel, 22.2) ||
             ldp_result.ldw_info.ldw_dlc_dec_time >= ldw_dlc_dec_time_thres));

    if (ldp_result.ldw_info.reset_flag == false) {
      if ((last_ldw_flag != 2 &&
           ldw_dlc > K_ldw_inner_reset_dlc_thr) // CJ:LDW not triggered or LDW
                                                // triggered and back to center
          || (last_ldw_flag == 2 &&
              ldw_dlc > K_ldw_outer_reset_dlc_thr) // CJ:LDW triggered and
                                                   // moving to neighbour lane
                                                   // center
      ) {
        ldp_result.ldw_info.reset_flag = true;
      }
    } else {
      if (ldp_result.ldw_info.ldw_status != LdwInfo::LDW_STATUS_NONE) {
        ldp_result.ldw_info.reset_flag = false;
      }
    }

    stop_flag =
        (((ldw_flag == 2 &&
           -min_y > K_ldw_outer_warning_end_thr // CJ: has changed lane
           ) ||
          (ldw_flag == 1 &&
           ldw_dlc > K_ldw_inner_warning_end_thr // CJ: back to center
           ) ||
          ldp_result.ldw_info.trigger_time >
              K_ldw_warning_time_max // CJ: warning time long enough
          ) &&
         ldp_result.ldw_info.trigger_time >
             K_ldw_warning_time_min // CJ: warning time shouldn't be too short
        );

    if (ldp_result.ldw_info.reset_flag == true && trigger_flag == true &&
        ldp_result.ldw_info.trigger_time == 0 && stop_flag == false) {
      ldw_warning = true;
    } else if (ldp_result.ldw_info.ldw_status != LdwInfo::LDW_STATUS_NONE &&
               stop_flag == true) {
      ldp_result.ldw_info.ldw_status = LdwInfo::LDW_STATUS_NONE;
      ldp_result.ldw_info.trigger_time = 0;
      ldw_warning = false;
      MSD_LOG(INFO, "CJ_DEBUG_LDW,State: %d", ldp_result.ldw_info.ldw_status);
    }
    return ldw_warning;
  }

  bool lka_warning_decide(const double ego_vel, const double relative_theta,
                          const vector<double> &wheel_dist2line,
                          const double min_dist, LkaInfo &lka_info,
                          const int last_lka_flag, const double curve,
                          const double lane_width, const double C0,
                          const LkaConfig &lka_config) {
    bool lka_warning = false;
    double ldp_tlc = 10;                    // CJ:time to cross lane
    double ldp_dlc = 10;                    // CJ:distance to cross lane
    bool trigger_flag = false;              // CJ: warning start flag
    bool stop_flag = false;                 // CJ: warning stop flag
    double K_lka_inner_reset_dlc_thr = 0.5; // CJ:m, when ego car comes back
                                            // crossing this line, next warning
                                            // is available again
    double K_lka_inner_dlc_thr =
        lka_config.K_lka_inner_dlc_thr; // CJ:m, when ego car departs crossing
                                        // this line, warning would be triggered
    double K_lka_inner_dlc_thr_min = 0.2;
    double K_lka_inner_tlc_thr = 1.0; // CJ:s, when ego car's tlc is lower than
                                      // this threshold, warning would be
                                      // triggered
    double K_lka_inner_tlc_line_thr = lka_config.K_lka_inner_tlc_line_thr;
    double K_lka_inner_earliest_dlc = 0.8; // CJ:m, when warning is triggered by
                                           // tlc, distance to lane must be
                                           // smaller than this
    double K_lka_inner_warning_end_thr =
        0.3; // CJ:m, when ego car come back awary from line by this distance,
             // warning would be stopped
    double K_lka_outer_latest_dlc = -0.2; // CJ:m, when ego car has crossed lane
                                          // by this distance, warning would not
                                          // be triggered
    double K_lka_outer_reset_dlc_thr =
        0.5; // CJ:m, when ego car continue crossing lane by this distance, next
             // warning in new lane is available
    double K_lka_outer_warning_end_thr = 0.3; // CJ:m, when ego car continue
                                              // crossing lane by this distance,
                                              // warning would be stopped
    double K_lka_curv_delay_tlc_max =
        0.3; // CJ:s, larggest tlc delay in curve road
    double K_lka_curv_delay_dlc_max =
        0.15; // CJ:m, larggest dlc delay in curve road
    double K_lka_curv_delay_tlc_rate = 30; // CJ:s*m, delta_tlc/delta_curvature
    double K_lka_curv_delay_dlc_rate = 15; // CJ:m^2, dleta_dlc/delta_curvature
    double K_lka_warning_time_min = 1000;  // CJ:ms, minimum warning time of LDW
    double K_lka_warning_time_max = 3000;  // CJ:ms, maximum warning time of LDW

    double dist_to_cross = 65535.0; // CJ:dist to move befor cross
    double max_y = -100, min_y = 100;
    const double K_straight_lat_err = 0.5;
    const double K_straight_yaw = 0.07;

    MSD_LOG(ERROR,
            "lka_warning_decide: K_lka_inner_dlc_thr: %f, "
            "K_lka_inner_tlc_line_thr: %f",
            K_lka_inner_dlc_thr, K_lka_inner_tlc_line_thr);

    K_lka_inner_dlc_thr -=
        std::fmin(curve * K_lka_curv_delay_dlc_rate, K_lka_curv_delay_dlc_max);
    K_lka_inner_tlc_thr -=
        std::fmin(curve * K_lka_curv_delay_tlc_rate, K_lka_curv_delay_tlc_max);
    K_lka_inner_dlc_thr -= std::fmax(STD_LANE_WIDTH - lane_width, 0) / 2;
    K_lka_inner_dlc_thr =
        std::fmax(K_lka_inner_dlc_thr, K_lka_inner_dlc_thr_min);
    K_lka_inner_earliest_dlc -= std::fmax(STD_LANE_WIDTH - lane_width, 0) / 2;
    lka_info.center_offset =
        std::fmax(0.15 - std::fmax(STD_LANE_WIDTH - lane_width, 0) / 2, 0.0);
    MSD_LOG(ERROR, "L_DEBUG,curve: %f", curve);
    MSD_LOG(INFO, "L_DEBUG,lane_width: %f", lane_width);
    for (auto dist : wheel_dist2line) {
      if (dist > max_y) {
        max_y = dist;
      }
      if (dist < min_y) {
        min_y = dist;
      }
    }

    ldp_dlc = -max_y; // CJ:set dlc sign, not crossed->positive
                      // crossed->negative
    // check if ldp_dlc is decreasing
    if (ldp_dlc < K_lka_outer_latest_dlc || ldp_dlc > lka_info.ldp_dlc) {
      lka_info.ldp_dlc_dec_time = 0;
    } else {
      lka_info.ldp_dlc_dec_time += 100;
    }
    MSD_LOG(INFO, "G_DEBUG, ldp_dlc_dec_time: %d", lka_info.ldp_dlc_dec_time);
    lka_info.ldp_dlc = ldp_dlc;
    if (relative_theta >
        0.1 / std::fmax(ego_vel, 22.2)) // CJ:ego car is driving toward line
    {
      if (ldp_dlc < K_lka_inner_tlc_line_thr) // CJ:has already crossed line
      {
        ldp_tlc = -1;
      } else // CJ:is about to cross
      {
        dist_to_cross =
            (ldp_dlc - K_lka_inner_tlc_line_thr) / std::tan(relative_theta);
        ldp_tlc = dist_to_cross / std::max(ego_vel, 0.01);
      }
    } else {
      ldp_tlc = 10; // CJ:default value
    }
    lka_info.ldp_tlc = ldp_tlc;
    MSD_LOG(INFO, "L_DEBUG,theta: %f", relative_theta);
    MSD_LOG(INFO, "L_DEBUG,ldp_dlc: %f,ldp_tlc: %f", ldp_dlc, ldp_tlc);
    MSD_LOG(INFO, "L_DEBUG,K_lka_inner_dlc_thr: %f", K_lka_inner_dlc_thr);
    MSD_LOG(INFO, "L_DEBUG,K_lka_inner_tlc_thr: %f", K_lka_inner_tlc_thr);
    MSD_LOG(INFO, "L_DEBUG,K_lka_inner_earliest_dlc: %f",
            K_lka_inner_earliest_dlc);
    MSD_LOG(INFO, "L_DEBUG,K_lka_outer_latest_dlc: %f", K_lka_outer_latest_dlc);

    const long ldp_dlc_dec_time_thres = 500; // 0.5s
    trigger_flag =
        ((ldp_dlc < K_lka_inner_dlc_thr ||
          ldp_tlc < K_lka_inner_tlc_thr) // CJ:dlc,tlc threshold
         &&
         (ldp_dlc < K_lka_inner_earliest_dlc &&
          ldp_dlc > K_lka_outer_latest_dlc) // CJ:within larggest warning region
         && ((relative_theta > 0.1 / std::fmax(ego_vel, 22.2) &&
              ldp_dlc > K_lka_inner_tlc_line_thr) ||
             lka_info.ldp_dlc_dec_time >
                 ldp_dlc_dec_time_thres)); // CJ:moving towards main line

    const long ldp_dlc_not_dec_time_thres = 200; // 0.2s
    const long straight_time_thres = 300;        // 0.3s
    if (lka_info.reset_flag == false) {
      if (last_lka_flag == 0) {
        if (std::fabs(C0) < K_straight_lat_err &&
            std::fabs(relative_theta) < K_straight_yaw) {
          lka_info.reset_flag = true;
        }
      } else {
        MSD_LOG(INFO, "L_DEBUG,C0: %f,trigger_time: %ld", C0,
                lka_info.trigger_time);
        if (ldp_dlc > K_lka_outer_latest_dlc &&
            lka_info.ldp_dlc_dec_time <= ldp_dlc_not_dec_time_thres && // 0.2s
            relative_theta < 0.3 / std::fmax(ego_vel, 22.2)) {
          lka_info.straight_time += 100; // ms
          MSD_LOG(INFO, "L_DEBUG,straight_time: %d", lka_info.straight_time);
          // reset when coming back after 0.3s
          if (lka_info.straight_time >= straight_time_thres) {
            lka_info.reset_flag = true;
          }
        } else {
          lka_info.straight_time = 0;
        }
      }
    } else {
      lka_info.straight_time = 0;
      if (lka_info.lka_status != LkaInfo::LKA_STATUS_NONE) {
        lka_info.reset_flag = false;
      }
    }

    MSD_LOG(INFO, "G_DEBUG,reset_flag: %d", lka_info.reset_flag);
    if (lka_info.reset_flag == true && trigger_flag == true &&
        lka_info.trigger_time == 0) {
      lka_warning = true;
    } else if (lka_info.lka_status != LkaInfo::LKA_STATUS_NONE &&
               lka_info.reset_flag == true) {
      lka_info.lka_status = LkaInfo::LKA_STATUS_NONE;
      lka_info.trigger_time = 0;
      lka_warning = false;
    } else if (lka_info.trigger_time > 4000) {
      lka_info.lka_status = LkaInfo::LKA_STATUS_NONE;
      lka_info.trigger_time = 0;
      lka_warning = false;
    }
    return lka_warning;
  }

  double calc_line_y(const std::vector<double> &coef, double x) {
    double y = 0;

    for (int i = (int)coef.size() - 1; i >= 0; --i) {
      y = y * x + coef[i];
    }
    return y;
  }

  // d_rel: == 0, overlap, < 0, back to ego head, > 0, head to ego back
  double calc_d_rel(const PerceptionFusionObjectData &target) {
    const double rear_axle_to_head =
        ConfigurationContext::Instance()->get_vehicle_param().length -
        ConfigurationContext::Instance()
            ->get_vehicle_param()
            .rear_bumper_to_rear_axle;
    double d_rel = 0.0;
    double front_distance = target.relative_position.x -
                            target.shape.length / 2.0 - rear_axle_to_head;
    double back_distance = target.relative_position.x +
                           target.shape.length / 2.0 +
                           ConfigurationContext::Instance()
                               ->get_vehicle_param()
                               .rear_bumper_to_rear_axle;
    if (front_distance > 0) {
      d_rel = front_distance;
    } else if (back_distance < 0) {
      d_rel = back_distance;
    }

    return d_rel;
  }

  void filter_elk_target(const PerceptionFusionObjectData &target,
                         const double d_rel, const double ego_vel,
                         std::vector<ElkTarget> &oncoming_targets,
                         std::vector<ElkTarget> &overtaking_targets) {

    double target_vel = target.relative_velocity.x + ego_vel;
    // check oncoming targets
    double oncoming_ttc = 10.0;
    if (d_rel > 0 && target.relative_velocity.x < 0.0 &&
        target_vel < -oncoming_target_speed_thres_) {
      oncoming_ttc = -1.0 * d_rel / target.relative_velocity.x;
    }
    MSD_LOG(INFO, "ELK_DEBUG: d_rel: %f, target_vel: %f", d_rel, target_vel);
    if (target_vel < -oncoming_target_speed_thres_) {
      if (d_rel > 0 && d_rel < oncoming_target_distance_thres_ ||
          oncoming_ttc < oncoming_target_ttc_thres_) {
        ElkTarget elk_target{};
        elk_target.track_id = target.track_id;
        elk_target.oncomine_ttc = oncoming_ttc;
        elk_target.relative_position = target.relative_position;
        elk_target.relative_velocity = target.relative_velocity;
        oncoming_targets.push_back(elk_target);
        MSD_LOG(INFO,
                "ELK_DEBUG: oncoming cars, track id: %d, x: %f, y: %f, "
                "d_rel: %f, oncoming_ttc: %f",
                target.track_id, target.relative_position.x,
                target.relative_position.y, d_rel, oncoming_ttc);
      }
    }

    // check overtaking targets
    double overtaking_ttc = 10.0;
    if (d_rel < 0 && target.relative_velocity.x > 0 &&
        target_vel > overtaking_target_speed_thres_) {
      overtaking_ttc = -1.0 * d_rel / target.relative_velocity.x;
    }
    if (target_vel > overtaking_target_speed_thres_) {
      if (std::fabs(d_rel) < std::numeric_limits<float>::epsilon() ||
          (d_rel < 0 && d_rel > -overtaking_target_distance_thres_ &&
           overtaking_ttc < overtaking_target_ttc_thres_)) {
        ElkTarget elk_target{};
        elk_target.track_id = target.track_id;
        elk_target.overtaking_ttc = overtaking_ttc;
        elk_target.relative_position = target.relative_position;
        elk_target.relative_velocity = target.relative_velocity;
        overtaking_targets.push_back(elk_target);
        MSD_LOG(INFO,
                "ELK_DEBUG: overtaking cars, track id: %d, x: %f, y: %f, "
                "d_rel: %f, overtaking_ttc: %f",
                target.track_id, target.relative_position.x,
                target.relative_position.y, d_rel, overtaking_ttc);
      }
    }
  }

private:
  enum FeedType {
    FEED_VEHICLE_DBW_STATUS = 0,
    FEED_EGO_STEER_ANGLE,
    FEED_WHEEL_SPEED_REPORT,
    FEED_EGO_ACC,
    FEED_MISC_REPORT,
    FEED_GEAR_REPORT,
    FEED_VISION_LANE,
    FEED_FUSION,
    FEED_TYPE_MAX,
  };

private:
  // std::shared_ptr<ModuleMonitor> monitor_;
  std::shared_ptr<maf::StatusManager> monitor_;
  maf_vehicle_status::VehicleStatus vehicle_status_{};
  maf_perception_interface::RoadLinePerception perception_vision_lane_{};
  maf_perception_interface::PerceptionFusionObjectResult perception_fusion_{};
  maf_perception_interface::PerceptionFusionAEBResult perception_fusion_aeb_{};

  double last_feed_time_[FEED_TYPE_MAX]{};

  mtaskflow::FlowReceiver<uint64_t> tick_receiver_;
  mtaskflow::FlowReceiver<ModuleStatus> module_status_receiver_;
  mtaskflow::FlowReceiver<std::shared_ptr<maf_endpoint::ChassisReport>>
      chassis_report_receiver_;
  mtaskflow::FlowReceiver<std::shared_ptr<maf_endpoint::WheelReport>>
      wheel_report_receiver_;
  mtaskflow::FlowReceiver<std::shared_ptr<maf_endpoint::BodyReport>>
      body_report_receiver_;
  mtaskflow::FlowReceiver<
      std::shared_ptr<maf_perception_interface::RoadLinePerception>>
      perception_vision_lane_receiver_;
  mtaskflow::FlowReceiver<
      std::shared_ptr<maf_perception_interface::PerceptionFusionObjectResult>>
      perception_fusion_receiver_;
  mtaskflow::FlowReceiver<
      std::shared_ptr<maf_perception_interface::PerceptionFusionAEBResult>>
      perception_fusion_aeb_receiver_;
  mtaskflow::FlowReceiver<std::string> mff_planning_request_receiver_;
  mtaskflow::FlowReceiver<std::string> planning_ess_status_receiver_;

  MSDPlanningOutputCallback planning_output_callback_{};
  MSDPlanningLdpOutputCallback planning_ldp_output_callback_{};

  bool enable_timer_tick_{true};
  bool reset_{false};

  bool ess_trigged_{false};
  double last_ess_feed_time_{0.0};

  LdpStateMachineInfo ldp_state_machine_info_{};
  LdpSensitiveInfo ldp_sensitive_info_{};
  int ldw_only_ = 0;

  double lateral_velocity_ = 0.0;
  double last_ldp_dlc_ = 0.0;

  // elk
  ElkStateMachineInfo elk_state_machine_info_{};
  std::vector<ElkTarget> left_oncoming_targets_{};
  std::vector<ElkTarget> left_overtaking_targets_{};
  std::vector<ElkTarget> right_oncoming_targets_{};
  std::vector<ElkTarget> right_overtaking_targets_{};
  bool left_bsd_activated_ = false;
  bool right_bsd_activated_ = false;
  bool left_lca_activated_ = false;
  bool right_lca_activated_ = false;
  int left_bsd_activated_count_ = 0;
  int right_bsd_activated_count_ = 0;
  int left_lca_activated_count_ = 0;
  int right_lca_activated_count_ = 0;
  int left_bsd_activated_count_thres_ = 5;  // 0.5s, 10hz
  int right_bsd_activated_count_thres_ = 5; // 0.5s, 10hz
  int left_lca_activated_count_thres_ = 5;  // 0.5s, 10hz
  int right_lca_activated_count_thres_ = 5; // 0.5s, 10hz

  const double near_lane_overlap_thres_ = 2.5;
  const double oncoming_target_speed_thres_ = 30.0 / 3.6;
  const double oncoming_target_distance_thres_ = 60.0;
  const double oncoming_target_ttc_thres_ = 3.0;
  const double overtaking_target_speed_thres_ = 30.0 / 3.6;
  const double overtaking_target_distance_thres_ = 60.0;
  const double overtaking_target_ttc_thres_ = 3.0;
  bool elk_re_enable_ = false;
  bool elk_ot_enable_ = false;
  bool elk_oc_enable_ = false;
  bool elk_absm_enable_ = false;
  int elk_absm_status_ = 0;

  // lane related
  int left_track_id_ = -1;
  int right_track_id_ = -1;
  bool is_left_virtual_lane_ = true;
  bool is_right_virtual_lane_ = true;
  bool is_left_road_edge_ = false;
  bool is_right_road_edge_ = false;
  bool is_left_solid_line_ = false;
  bool is_right_solid_line_ = false;
  bool is_left_dash_line_ignored_ = false;
  bool is_right_dash_line_ignored_ = false;

  bool left_light_activated_ = false;
  bool right_light_activated_ = false;

  bool ldp_lane_cross_ = false;
  bool elk_lane_cross_ = false;

  // suppressed related
  const double lane_start_length_thres = 15.0;
  const double lane_end_length_unsuppressed_thres = 20.0;
  const int lane_length_unsuppressed_count_thres = 30; // 3s, 10hz

  const double narrow_lane_width_suppressed_thres = 2.5;
  const double narrow_lane_width_unsuppressed_thres = 2.6;
  const double wide_lane_width_suppressed_thres = 5.5;
  const double wide_lane_width_unsuppressed_thres = 5.2;
  const int lane_width_suppressed_count_thres = 30;   // 3s, 10hz
  const int lane_width_unsuppressed_count_thres = 10; // 1s, 10hz

  int left_lane_length_unsuppressed_count_ = 0;
  int right_lane_length_unsuppressed_count_ = 0;
  bool is_left_lane_length_suppressed_ = false;
  bool is_right_lane_length_suppressed_ = false;

  int lane_width_suppressed_count_ = lane_width_suppressed_count_thres;
  int lane_width_unsuppressed_count_ = 0;
  bool is_lane_width_suppressed_ = false;

  const double STD_LANE_WIDTH = 3.5;
  const double MAX_LANE_WIDTH = 4.5;
  const double IGNORE_LANE_WIDTH = 7.0;

  // time info
  uint64_t last_vision_lane_header_timestamp_ms_ = 0;
  uint64_t last_vision_lane_sensor_timestamp_ms_ = 0;

  LdpPlannerConfig ldp_planner_config_{};
};

} // namespace msquare
