#pragma once
#include <iostream>
#include <maf_interface/maf_perception_interface.h>
#include <string>

#include "common/config_context.h"
#include "common/hmi/hmi_manager.h"
#include "common/scenario_manager.h"
#include "common/world_model.h"
#include "data_driven_planner/common/ddp_context.h"
#include "data_driven_planner/common/ddp_debug_logger.h"
#include "data_driven_planner/common/ddp_utils.h"
#include "data_driven_planner/models/ego_pose_manager.h"
#include "data_driven_planner/models/fusion_object_manager.h"
#include "mdiag/mdiag.h"
#include "mlog_core/mlog.h"
#include "mlog_msg_id/mlog_msg_id.hpp"
#include "mtaskflow/mtaskflow.hpp"
#include "planner/message_type.h"
#include "planner/motion_planner/speed_planner_ceres/speed_planner_constants.hpp"
#include "planning/common/inner_snapshot.h"
#include "planning_task_interface.h"

#ifdef ENABLE_XAVIER_SAFETY_MODEL
#include <cuda_runtime.h>
#endif

using namespace msd_planning;
using namespace maf_framework_status;
using namespace maf_worldmodel;
using namespace maf_perception_interface;
using namespace maf_mla_localization;
using namespace maf_vehicle_status;
using namespace maf_endpoint;

namespace msquare {

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
      mtaskflow::FlowReceiver<maf_std::Header> planning_reset_request_receiver,
      mtaskflow::FlowReceiver<
          std::shared_ptr<maf_perception_interface::RoadLinePerception>>
          perception_vision_lane_receiver,
      mtaskflow::FlowReceiver<
          std::shared_ptr<maf_perception_interface::RoadLinePerception>>
          perception_lidar_road_edge_receiver,
      mtaskflow::FlowReceiver<
          std::shared_ptr<maf_perception_interface::RadarPerceptionResult>>
          perception_radar_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<maf_std::Header>>
          mff_info_receiver,
      mtaskflow::FlowReceiver<std::shared_ptr<
          maf_perception_interface::PerceptionFusionObjectResult>>
          perception_fusion_object_receiver)
      : enable_timer_tick_(enable_timer_tick), monitor_(monitor),
        tick_receiver_(tick_receiver),
        module_status_receiver_(module_status_receiver),
        chassis_report_receiver_(chassis_report_receiver),
        wheel_report_receiver_(wheel_report_receiver),
        body_report_receiver_(body_report_receiver),
        imu_report_receiver_(imu_report_receiver),
        worldmodel_map_receiver_(worldmodel_map_receiver),
        worldmodel_objects_receiver_(worldmodel_objects_receiver),
        traffic_light_receiver_(traffic_light_receiver),
        ego_pose_receiver_(ego_pose_receiver),
        prediction_result_receiver_(prediction_result_receiver),
        mpc_trajectory_receiver_(mpc_trajectory_receiver),
        planning_control_cmd_request_receiver_(
            planning_control_cmd_request_receiver),
        planning_request_receiver_(planning_request_receiver),
        planning_reset_request_receiver_(planning_reset_request_receiver),
        perception_vision_lane_receiver_(perception_vision_lane_receiver),
        perception_lidar_road_edge_receiver_(
            perception_lidar_road_edge_receiver),
        perception_radar_receiver_(perception_radar_receiver),
        mff_info_receiver_(mff_info_receiver),
        perception_fusion_object_receiver_(perception_fusion_object_receiver),
        worldmodel_objects_buffer_(3) {
    module_control_cmd_request_.module_control_cmd.value =
        maf_system_manager::ModuleControlCmdEnum::PAUSE;

    world_model_ = std::make_shared<WorldModel>();
    (void)world_model_->init(planning_config.scene_type);
    world_model_->set_acc_mode(planning_config.is_acc_mode);
    world_model_->set_ddmap(true); // ddmap always set to true
    world_model_->set_driving_style(
        DrivingStyle(planning_config.driving_style));
    world_model_->set_ilc_limit_velocity(planning_config.ilc_limit_velocity);
    world_model_->set_ilc_limit_time(planning_config.ilc_limit_time);
    // FOR HARZ DEMO / by jojo.feng
    if (ConfigurationContext::Instance()->get_vehicle_param().car_type ==
        "S450") {
      world_model_->set_ilc_limit_time(15.0);
    }
    scenario_manager_ = std::make_shared<ScenarioManager>(world_model_);

    enable_ilc_ = planning_config.enable_ilc;
    world_model_->set_enable_alc(planning_config.enable_alc);

    cruise_velocity_ = planning_config.cruise_velocity;
    underclocking_enum_ = planning_config.underclocking_enum;
    underclocking_ = underclocking_enum_;
    is_in_simulation_ = is_in_simulation();
    if (is_in_simulation_) {
      _runner = std::bind(&PlanningTask::_run_in_simulation, this);
    } else {
      _runner = std::bind(&PlanningTask::_run_in_reality, this);
    }

    hmi_manager_ = std::make_unique<HmiManager>();
  }

  void on_load() {
    MSD_LOG(INFO, "on_loading");

    MD_REG_MODULE(CP_PLANNING_NODE);
    MD_REG_TASK(CP_APA_PLANNING);
    MD_REG_TASK(CP_APA_PLANNING1);

#ifdef ENABLE_XAVIER_SAFETY_MODEL
    int device_count = 0;
    cudaGetDeviceCount(&device_count);
#endif
  }

  bool is_in_simulation() {
    bool is_in_simulation = false;
    const char *sim = std::getenv("RealitySimulation");
    if (sim != nullptr && std::strcmp(sim, "simulation") == 0) {
      is_in_simulation = true;
    } else {
      is_in_simulation = false;
    }
    return is_in_simulation;
  }

  void _run_in_simulation() {
    _run_in_reality();
    if (module_control_cmd_request_.running_mode.value !=
        maf_system_manager::RunningModeEnum::PILOT) {
      return;
    }
    std::string ss = "";
    ss.append("input seqs, cp planning: ")
        .append(std::to_string(planning_seq_))
        .append(", localization: ")
        .append(std::to_string(localization_seq_));
    MSD_LOG(INFO, "%s", ss.c_str());
    if (publish_simulation_sync_ != nullptr) {
      publish_simulation_sync_(std::string("PLANNING END: ") + ss);
    }
  }

  void on_running() { _runner(); }

  void _run_in_reality() {
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

    if (!planning_control_cmd_request_receiver_->empty()) {
      auto ret = planning_control_cmd_request_receiver_->fetch_newest_and_clear(
          module_control_cmd_request_);
      if (!ret) {
        return;
      }
    }

    if (module_control_cmd_request_.running_mode.value !=
            maf_system_manager::RunningModeEnum::PILOT ||
        module_control_cmd_request_.module_control_cmd.value ==
            maf_system_manager::ModuleControlCmdEnum::PAUSE) {
      clear_all_recievers();
      reset();
      return;
    }

    maf_system_manager::RunningModeEnum mode;
    mode.value = maf_system_manager::RunningModeEnum::PILOT;
    monitor_->set_node_status_running_mode(mode);

    if (reset_) {
      reset_finished_ = false;
    }
    world_model_->set_reset(reset_);
    world_model_->set_apf_lat_far_flag(false);
    world_model_->set_use_eftp(ConfigurationContext::Instance()
                                   ->planner_config()
                                   .common_config.use_eftp);
    world_model_->set_lateral_use_eftp(ConfigurationContext::Instance()
                                           ->planner_config()
                                           .common_config.lateral_use_eftp);

    static bool alc_switch_inited = false;
    static bool last_enable_alc = false;
    const bool enable_alc = ConfigurationContext::Instance()
                                ->planner_config()
                                .lateral_behavior_planner_config.enable_alc;
    if (!alc_switch_inited || last_enable_alc != enable_alc) {
      world_model_->set_enable_alc(enable_alc);
      last_enable_alc = enable_alc;
      alc_switch_inited = true;
    }
    const bool enable_cutin_threshold =
        ConfigurationContext::Instance()
            ->planner_config()
            .longitudinal_motion_planner_config.enable_cutin_threshold_adjust;
    world_model_->set_enable_cutin_threshold(enable_cutin_threshold);

    const double lon_cutin_threshold =
        ConfigurationContext::Instance()
            ->planner_config()
            .longitudinal_motion_planner_config.lon_cutin_threshold;
    world_model_->set_lon_cutin_threshold(lon_cutin_threshold);

    const bool enable_recomand_alc =
        ConfigurationContext::Instance()
            ->planner_config()
            .lateral_behavior_planner_config.enable_recommend_alc;
    if (enable_recomand_alc) {
      world_model_->set_enable_recommend_alc(true);
      world_model_->set_enable_alc(false);
    } else {
      world_model_->set_enable_recommend_alc(false);
    }

    MSD_LOG(INFO,
            "refline_generator cond judge quit cp: reset 0 each frame [%d]",
            world_model_->get_apf_lat_far_flag());
    MSD_LOG(WARN, "(%s)tick_count: %lu", __FUNCTION__, tick_count);

    log_tips_info();

    auto inner_snapshot = PlanInnerSnapshot::GetInst();
    auto inner_ctx = inner_snapshot->get_mutable_data();

    inner_ctx->real_start_runonce_sec = MTIME()->timestamp().sec();

    // restore mfr snapshot checkpoint for timestamp synchronizer
    auto syner = get_chkpoint_syner();
    if (syner) {
      inner_snapshot->set_snap_mode(PlanInnerSnapshot::M_REALCAR);
      /*
      if (syner->IsPlayBackMode()) {
        syner->RestoreCheckPoint((uint8_t *)inner_ctx, sizeof(*inner_ctx));
        inner_snapshot->set_snap_mode(PlanInnerSnapshot::M_PLAYBACK);
      } else if (syner->IsRealCarMode()) {
        inner_snapshot->set_snap_mode(PlanInnerSnapshot::M_REALCAR);
      }*/
    }

    bool succeed = run_once(inner_ctx->real_start_runonce_sec);

    if (nullptr != planning_output_callback_) {
      const auto &map_info = world_model_->get_map_info();
      const auto &planning_status =
          PlanningContext::Instance()->planning_status();

      auto maf_planning_output = generate_planning_output();

      // add nan check for protection
      for (const auto &path_point : maf_planning_output.trajectory.path) {
        if (std::isnan(path_point.position_enu.x) ||
            std::isnan(path_point.position_enu.y)) {
          succeed = false;
        }
      }
      for (const auto &velocity_point :
           maf_planning_output.trajectory.velocity.vel_points) {
        if (std::isnan(velocity_point.target_velocity)) {
          succeed = false;
        }
      }

      if (map_info.is_in_map_area() || world_model_->is_acc_mode()) {
        msd_planning::MSDPlanning_record_relation(
            last_feed_msg_id_[FEED_PREDICTION_INFO],
            mlog::MLOG_msg_id(maf_planning_output.header.stamp,
                              "/msd/planning/plan"));
        msd_planning::MSDPlanning_record_timestamp(
            mlog::MLOG_msg_id(maf_planning_output.header.stamp,
                              "/msd/planning/plan"),
            "planning/send", 0);
        maf_planning_output.header.seq = inner_ctx->plan_out_seq;
        if (is_in_simulation_ && succeed) {
          maf_planning_output.header.seq = ++planning_seq_;
        }
        planning_output_callback_({tick_count, succeed}, maf_planning_output,
                                  planning_status.trigger_msg_id);

        if (succeed) {
          auto &planning_result =
              msquare::ddp::DdpContext::Instance()->mutable_planning_result();
          update_planning_result(maf_planning_output, planning_result);
          // msquare::ddp::DdpContext::Instance()
          //     ->planning_result_manager()
          //     ->add_planning_result(planning_result);
        }

        // use plannint timestamp
        PlanningContext::Instance()
            ->mutable_path_planner_input()
            ->header.stamp = maf_planning_output.header.stamp;
        PlanningContext::Instance()
            ->mutable_path_planner_input()
            ->header.frame_id = maf_planning_output.header.frame_id;
        // TODO
        // local_lat_motion_callback_(
        //     PlanningContext::Instance()->path_planner_input());

        if (nullptr != planning_info_callback_ && succeed) {
          auto maf_planning_info = generate_planning_info(tick_count);

          planning_info_callback_(maf_planning_info);
        }

        if (nullptr != mdebug_callback_ && !is_hnp_mode_) {
          if (MdebugContext::Instance()->json_raw_buffer().size() > 0) {
            MdebugContext::Instance()->json_raw_buffer().pop_last(']');
            if (MdebugContext::Instance()->json_raw_buffer().last() != '[') {
              MdebugContext::Instance()->json_raw_buffer().append_raw(",");
            }
          } else {
            MdebugContext::Instance()->json_raw_buffer().append_raw("[");
          }

          MdebugContext::Instance()->json_raw_buffer().append_raw(
              "{"
              "\"camera_source\": 0,"
              "\"category\": \"np_planning\","
              "\"coordinate\": 2,"
              "\"type\": 100,"
              "\"data\":");

          const char *ddp_debug_json =
              MdebugContext::Instance()->json_dict_buffer().c_str();
          size_t ddp_debug_len =
              MdebugContext::Instance()->json_dict_buffer().size();
          MdebugContext::Instance()->json_raw_buffer().append_raw(
              ddp_debug_json, ddp_debug_len);
          MdebugContext::Instance()->json_raw_buffer().append_raw("}]");

          std::string mdebug_json(
              MdebugContext::Instance()->json_raw_buffer().c_str(),
              MdebugContext::Instance()->json_raw_buffer().size());
          mdebug_callback_(std::move(mdebug_json));
          MdebugContext::Instance()->reset();
        } else {
          MdebugContext::Instance()->reset();
        }
      }
    }

    if (syner && syner->IsRealCarMode()) {
      syner->SetCheckPoint((uint8_t *)inner_ctx, sizeof(*inner_ctx));
    }

    inner_ctx->plan_out_seq += 1;
  }

  void set_sync_callback(NPPHeaderLoggerCallback callback) {
    publish_simulation_sync_ = callback;
  }

  void set_callback(MSDPlanningOutputCallback callback) {
    planning_output_callback_ = callback;
  }

  void set_callback(MSDPlanningLdpOutputCallback callback) {
    // not used
  }

  void set_callback(MSDPlanningTriggerCallback callback) {
    planning_trigger_callback_ = callback;
  }

  void
  set_callback(std::function<void(const maf_planning::SBPRequest &sbp_request)>
                   &callback) {
    // TODO@all: wether use or not
  }

  void set_callback(MSDPlanningInfoCallback callback) {
    planning_info_callback_ = callback;
  }

  void set_callback(MSDMdebugCallback callback) { mdebug_callback_ = callback; }

  void reset() { reset_ = true; }

private:
  void log_tips_info() {
    MSD_LOG(WARN, "VERSION: %s",
            PlanningContext::Instance()->get_version().c_str());
    MSD_LOG(WARN, "is_acc_mode: %d", world_model_->is_acc_mode());
    MSD_LOG(WARN, "is_ddmap: %d", world_model_->is_ddmap());
    MSD_LOG(WARN, "dbw_status: %d", world_model_->get_vehicle_dbw_status());
    MSD_LOG(WARN, "sensor config: %s",
            ConfigurationContext::Instance()
                ->synthetic_config()
                .sensor_configuration.c_str());
    switch (world_model_->get_driving_style()) {
    case DRIVING_STYLE_AGGRESSIVE:
      MSD_LOG(WARN, "driving_style: AGGRESSIVE");
      break;
    case DRIVING_STYLE_NORMAL:
      MSD_LOG(WARN, "driving_style: NORMAL");
      break;
    case DRIVING_STYLE_CONSERVATIVE:
      MSD_LOG(WARN, "driving_style: CONSERVATIVE");
      break;
    default:
      break;
    }
    switch (world_model_->get_lane_changing_style()) {
    case LANECHANGING_STYLE_AGGRESSIVE:
      MSD_LOG(WARN, "lane_changing_style: AGGRESSIVE");
      break;
    case LANECHANGING_STYLE_NORMAL:
      MSD_LOG(WARN, "lane_changing_style: NORMAL");
      break;
    case LANECHANGING_STYLE_CONSERVATIVE:
      MSD_LOG(WARN, "lane_changing_style: CONSERVATIVE");
      break;
    default:
      break;
    }
  }

  bool run_once(double real_start_run_sec) {
    MLOG_PROFILING("run_once");
    double start_time = MTIME()->timestamp().ms();
    auto pre_planning_status =
        PlanningContext::Instance()->mutable_prev_planning_status();
    *pre_planning_status = PlanningContext::Instance()->planning_status();
    auto *planning_status =
        PlanningContext::Instance()->mutable_planning_status();
    planning_status->pre_planning_result = planning_status->planning_result;
    nlohmann::json extra_json;
    planning_status->planning_result.extra_json_raw = extra_json;
    planning_status->planning_result.debug_json =
        mjson::Json(mjson::Json::object());
    auto *planner_debug = PlanningContext::Instance()->mutable_planner_debug();

    double start_timestamp_sec = real_start_run_sec;
    double start_timestamp_sys_sec = MTIME()->timestamp().sec();

    PlanningContext::Instance()
        ->mutable_planning_status()
        ->planning_result.next_timestamp_sec = start_timestamp_sec;

    uint64_t start_timestamp_ns =
        static_cast<uint64_t>(start_timestamp_sec * 1000000000.0);
    auto trigger_msg_id = mlog::MLOG_msg_id(start_timestamp_ns, kTriggerTag);
    msd_planning::MSDPlanning_record_timestamp(
        trigger_msg_id, MSDPlanning_msg_tag(kPlanTag), 0);
    msd_planning::MSDPlanning_record_timestamp(
        trigger_msg_id, "pnc_plan_timestamp-run-start", start_timestamp_ns);

    // update world module from msg
    double current_time = start_timestamp_sec;
    double before_update_world_model_time = MTIME()->timestamp().ms();

    update_world_model(current_time);

    double after_update_world_model_time = MTIME()->timestamp().ms();

    MSD_LOG(ERROR, "time_cost, update_world_model time: %f",
            after_update_world_model_time - before_update_world_model_time);
    CostTime task_cost =
        CostTime{"update_world_model", after_update_world_model_time -
                                           before_update_world_model_time};
    planner_debug->cost_time.emplace_back(task_cost);

    std::string warn_msg{}, error_msg{};
    if (!can_run(current_time, warn_msg, error_msg)) {
      MSD_LOG(ERROR, "(%s)can not run", __FUNCTION__);

      if (is_hnp_mode_ && error_msg == "") {
        (void)monitor_->try_change_status(node_status::Status::STOP);
        monitor_->set_node_status_message(
            "planning execute skipped. mode: HNP");
      } else {
        error_msg = "planning can not run, missing inputs: " + error_msg;
        (void)monitor_->try_change_status(node_status::Status::RUNNING_ERROR);
        monitor_->set_node_status_message(error_msg);
      }

      return false;
    } else {
      if (warn_msg.size() > 0) {
        warn_msg = "planning running warnning, inputs delay: " + warn_msg;
        (void)monitor_->try_change_status(node_status::Status::RUNNING_WARNING);
        monitor_->set_node_status_message(warn_msg);
      }
    }

    double before_world_model_update_time = MTIME()->timestamp().ms();

    MD_CP_BEGIN(CP_APA_PLANNING);
    bool ret = world_model_->update();
    MD_CP_END(CP_APA_PLANNING);
    if (!ret) {
      MSD_LOG(ERROR, "(%s)world model update error", __FUNCTION__);

      if (!world_model_->is_acc_mode()) {
        std::string msg = "planning world model update error.";
        (void)monitor_->try_change_status(node_status::Status::RUNNING_ERROR);
        monitor_->set_node_status_message(msg);
      }

      return false;
    }

    double after_world_model_update_time = MTIME()->timestamp().ms();

    MSD_LOG(ERROR, "time_cost, world_model_update time: %f",
            after_world_model_update_time - before_world_model_update_time);
    CostTime task_cost1 =
        CostTime{"world_model_update", after_world_model_update_time -
                                           before_world_model_update_time};
    planner_debug->cost_time.emplace_back(task_cost1);

    planning_status->trigger_msg_id = trigger_msg_id;

    if (scenario_manager_->execute_planning()) {
      if (reset_ && !reset_finished_) {
        reset_ = false;
        reset_finished_ = true;
        MSD_LOG(ERROR, "highway planning task, reset finished!");
      }
      planning_status->planning_success = true;

#ifndef __QNX__
      scenario_manager_->log_debug_info();
#endif

      const auto current_status = monitor_->get_status();
      if (current_status.status !=
          static_cast<uint16_t>(node_status::Status::RUNNING_WARNING)) {
        std::string msg = "planning execute succeed.";
        std::string function_mode_msg = "";
        if (!is_hnp_mode_) {
          if (world_model_->is_acc_mode()) {
            function_mode_msg = " mode: ACC";
          } else {
            function_mode_msg = " mode: PILOT";
          }
          if (world_model_->is_mrc_inlane_brake()) {
            function_mode_msg = " mode: MRC";
          }
          (void)monitor_->try_change_status(node_status::Status::RUNNING);

          if (ego_position_jerk_large_) {
            msg = "ego pose jerk large.";
            (void)monitor_->try_change_status(
                node_status::Status::RUNNING_WARNING);
          }
        } else {
          function_mode_msg = " mode: HNP";
          (void)monitor_->try_change_status(node_status::Status::STOP);
        }
        msg += function_mode_msg;
        monitor_->set_node_status_message(msg);
      }
    } else {
      planning_status->planning_success = false;
#ifndef __QNX__
      scenario_manager_->log_debug_info();
#endif

      MSD_LOG(ERROR, "(%s)execute planning error", __FUNCTION__);

      std::string msg = "planning execute failed.";
      (void)monitor_->try_change_status(node_status::Status::RUNNING_ERROR);
      monitor_->set_node_status_message(msg);
    }

    auto end_timestamp_sec = MTIME()->timestamp().sec();

    auto planning_staus =
        PlanningContext::Instance()->mutable_planning_status();
    planning_status->time_consumption =
        end_timestamp_sec - start_timestamp_sys_sec;
    planning_status->planning_result.timestamp_sec =
        planning_status->planning_result.next_timestamp_sec;

    double end_time = MTIME()->timestamp().ms();
    MSD_LOG(ERROR, "time_cost, total time: %f", end_time - start_time);
    CostTime task_cost2 = CostTime{"total", end_time - start_time};
    planner_debug->cost_time.emplace_back(task_cost2);

    return planning_status->planning_success;
  }

  void clear_all_recievers() {
    module_status_receiver_->clear();
    chassis_report_receiver_->clear();
    wheel_report_receiver_->clear();
    body_report_receiver_->clear();
    imu_report_receiver_->clear();
    worldmodel_map_receiver_->clear();
    worldmodel_objects_receiver_->clear();
    traffic_light_receiver_->clear();
    ego_pose_receiver_->clear();
    prediction_result_receiver_->clear();
    mpc_trajectory_receiver_->clear();
    planning_control_cmd_request_receiver_->clear();
    planning_request_receiver_->clear();
    planning_reset_request_receiver_->clear();

    // clear last feed time
    for (int i = 0; i < FEED_TYPE_MAX; ++i) {
      last_feed_time_[i] = 0.0;
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

      if (received_chassis_report->throttle_report.available &
          ThrottleReport::THROTTLE_REPORT_DATA) {
        vehicle_status_.throttle.available |=
            maf_vehicle_status::Throttle::THROTTLE_DATA;
        vehicle_status_.throttle.throttle_data.override =
            received_chassis_report->throttle_report.throttle_report_data
                .override;
      } else {
        vehicle_status_.throttle.available = 0;
        vehicle_status_.throttle.throttle_data.override = 0;
      }

      if (received_chassis_report->epb_report.available &
          EpbReport::EPB_REPORT_DATA) {
        vehicle_status_.epb_info.available |=
            maf_vehicle_status::EpbInfo::EPB_DATA;
        vehicle_status_.epb_info.epb_data.auto_hold_status =
            received_chassis_report->epb_report.epb_report_data.auto_hold_state
                .value;
      } else {
        vehicle_status_.epb_info.available = 0;
        vehicle_status_.epb_info.epb_data.auto_hold_status = 0;
      }
    }
  }

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

      // for epcar, use front wheel angle instead of steering_wheel_angle
      const auto &car_type =
          ConfigurationContext::Instance()->get_vehicle_param().car_type;
      if (car_type == "L7" || car_type == "LS7") {
        if (received_wheel_report->wheel_angle_report.available &
            WheelAngleReport::WHEEL_ANGLE_REPORT_DATA) {
          static constexpr double deg2rad = M_PI / 180.0;
          auto &steering_wheel = vehicle_status_.steering_wheel;
          steering_wheel.available |= SteeringWheel::STEERING_WHEEL_DATA;
          double steering_wheel_angle_report =
              deg2rad *
              (received_wheel_report->wheel_angle_report.wheel_angle_report_data
                   .front_left +
               received_wheel_report->wheel_angle_report.wheel_angle_report_data
                   .front_right) /
              2 *
              ConfigurationContext::Instance()->get_vehicle_param().steer_ratio;
          steering_wheel.steering_wheel_data.steering_wheel_rad =
              steering_wheel_angle_report;

          last_feed_time_[FEED_EGO_STEER_ANGLE] = current_time;
        }
      }
    }
  }

  void update_body_report(double current_time) {
    // bool cruise_control_set_increase = false;
    // bool cruise_control_set_decrease = false;

    static int turn_signal_type_activated_count = 0;
    const int turn_signal_type_activated_count_thres = 10; // 10hz, 1s
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
        if (enable_ilc_) {
          auto &vehicle_light_data =
              vehicle_status_.vehicle_light.vehicle_light_data;
          auto &vehicle_light_report_data =
              received_body_report->vehicle_light_report
                  .vehicle_light_report_data;
          vehicle_light_data.turn_signal.value =
              vehicle_light_report_data.lever_state.value;
          // use lever state for ilc in simluiation
          auto env = std::getenv("RealitySimulation");
          if (env != nullptr) {
            if (std::strcmp(env, "simulation") == 0 &&
                vehicle_light_data.turn_signal.value != TurnSignalType::NONE) {
              world_model_->set_ego_blinker(vehicle_status_.vehicle_light);
            }
          }
          // blinking for epcar
          static int valid_turn_signal_type = TurnSignalType::NONE;
          if (vehicle_light_report_data.turn_signal_type.value !=
              TurnSignalType::NONE) {
            turn_signal_type_activated_count =
                turn_signal_type_activated_count_thres;
            valid_turn_signal_type =
                vehicle_light_report_data.turn_signal_type.value;
          }
          if (turn_signal_type_activated_count > 0) {
            vehicle_light_data.turn_signal_type.value = valid_turn_signal_type;
          } else {
            vehicle_light_data.turn_signal_type.value = TurnSignalType::NONE;
          }
        }
        last_feed_time_[FEED_MISC_REPORT] = current_time;
      }
    }
    turn_signal_type_activated_count =
        std::max(turn_signal_type_activated_count - 1, 0);

    vehicle_status_.velocity.available |=
        maf_vehicle_status::Velocity::CRUISE_VELOCITY;
    vehicle_status_.velocity.cruise_velocity.value_mps = cruise_velocity_ / 3.6;

    // auto ego_pose_manager = ddp::DdpContext::Instance()->ego_pose_manager();
    // if (ego_pose_manager != nullptr) {
    //   ego_pose_manager->feed_ego_cruise_velocity(cruise_velocity_);
    //   ego_pose_manager->feed_navi_ttc_gear(navi_ttc_gear_);
    // }
  }

  void update_ego_pose(double current_time) {
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
        vehicle_status_.velocity.heading_velocity.vx_mps =
            mla_localization->velocity.velocity_local.vx;
        vehicle_status_.velocity.heading_velocity.vy_mps =
            mla_localization->velocity.velocity_local.vy;

        ego_pose_timestamp_us_ = mla_localization->meta.timestamp_us;
        last_feed_time_[FEED_EGO_VEL] = current_time;

        // auto ego_pose_manager =
        // ddp::DdpContext::Instance()->ego_pose_manager(); if (ego_pose_manager
        // != nullptr) {
        //   ego_pose_manager->feed_ego_velocity(ego_pose_timestamp_us_ / 1e6,
        //                                       mla_localization->velocity);
        // }
      }

      if ((mla_localization->orientation.available &
           MLAOrientation::MLA_EULER_LOCAL) &&
          (mla_localization->position.available &
           MLAPosition::MLA_POSITION_LOCAL)) {
        vehicle_status_.location.available |= Location::LOCATION_ENU;
        auto &location_enu = vehicle_status_.location.location_enu;

        vehicle_status_.heading_yaw.heading_yaw_data.value_rad =
            mla_localization->orientation.euler_local.yaw;

        location_enu.x = mla_localization->position.position_local.x;
        location_enu.y = mla_localization->position.position_local.y;
        location_enu.z = mla_localization->position.position_local.z;

        // convert from rear axle to head
        location_enu.x +=
            std::cos(vehicle_status_.heading_yaw.heading_yaw_data.value_rad) *
            (ConfigurationContext::Instance()->get_vehicle_param().length -
             ConfigurationContext::Instance()
                 ->get_vehicle_param()
                 .rear_bumper_to_rear_axle);
        location_enu.y +=
            std::sin(vehicle_status_.heading_yaw.heading_yaw_data.value_rad) *
            (ConfigurationContext::Instance()->get_vehicle_param().length -
             ConfigurationContext::Instance()
                 ->get_vehicle_param()
                 .rear_bumper_to_rear_axle);

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
        MSD_LOG(INFO,
                "ENU_DEBUG,orien.x:%3.3f,orien.y:%3.3f,orien.z:%3.3f,orien.w:"
                "%3.3f",
                location_enu.orientation.x, location_enu.orientation.y,
                location_enu.orientation.z, location_enu.orientation.w);
        MSD_LOG(INFO, "ENU_DEBUG,enu.yaw:%3.5f",
                mla_localization->orientation.euler_local.yaw);

        // auto ego_pose_manager =
        // ddp::DdpContext::Instance()->ego_pose_manager(); if (ego_pose_manager
        // != nullptr) {
        //   ego_pose_manager->feed_ego_position(
        //       ego_pose_timestamp_us_ / 1e6, location_enu,
        //       mla_localization->orientation.euler_local.yaw);
        // }

        // check position jerk
        ego_position_jerk_large_ = false;
        const double ego_position_jerk_thres = 1.0;
        if (std::abs(last_position_.position_local.x) > 0.0) {
          double dx = mla_localization->position.position_local.x -
                      last_position_.position_local.x;
          double dy = mla_localization->position.position_local.y -
                      last_position_.position_local.y;
          double ds = std::hypot(dx, dy);
          if (ds > ego_position_jerk_thres) {
            ego_position_jerk_large_ = true;
            MSD_LOG(ERROR, "ego position jerk large, delta: %f", ds);
          }
        }
        last_position_ = mla_localization->position;
      }
    }
  }

  void update_vehicle_status(double current_time) {

    update_chassis_report(current_time);

    update_wheel_report(current_time);

    update_body_report(current_time);

    update_ego_pose(current_time);

    if (vehicle_status_.location.available & Location::LOCATION_ENU &&
        vehicle_status_.velocity.available &
            maf_vehicle_status::Velocity::HEADING_VELOCITY &&
        vehicle_status_.steering_wheel.available &
            SteeringWheel::STEERING_WHEEL_DATA &&
        vehicle_status_.brake_info.available & BrakeInfo::BRAKE_INFO_DATA &&
        vehicle_status_.wheel_velocity.available &
            WheelVelocity::WHEEL_VELOCITY4D &&
        vehicle_status_.vehicle_light.available &
            VehicleLight::VEHICLE_LIGHT_DATA &&
        vehicle_status_.gear.available & maf_vehicle_status::Gear::GEAR_DATA &&
        vehicle_status_.velocity.available &
            maf_vehicle_status::Velocity::CRUISE_VELOCITY) {
      world_model_->feed_vehicle_status(vehicle_status_,
                                        steer_angle_offset_deg_);

      double ego_pose_delay = current_time - ego_pose_timestamp_us_ / 1.0e+6;
      MSD_LOG(WARN, "[ego_pose_delay_time] ego_pose_delay %f ", ego_pose_delay);
    } else {
      MSD_LOG(WARN, "ENU_DEBUG, Veh_Status NOT Updated!");
    }
  }

  void update_imu_report(double current_time) {
    while (!imu_report_receiver_->empty()) {
      std::shared_ptr<maf_gps_imu::MLAImu> received_imu_report{};
      auto ret = imu_report_receiver_->pop_oldest(received_imu_report);
      if (ret && received_imu_report->info.available &
                     maf_gps_imu::MLAImuInfo::MLA_IMU_BASIC) {
        double yawrate = received_imu_report->info.imu_data_basic.gyroz;

        world_model_->set_yawrate(yawrate);
        // last_feed_time_[FEED_SENSOR_IMU] = current_time;
      }
    }
  }

  void update_mpc_trajectory(double current_time) {
    if (!mpc_trajectory_receiver_->empty()) {
      std::shared_ptr<maf_planning::MpcTrajectoryResult> mpc_trajectory{};
      auto ret =
          mpc_trajectory_receiver_->fetch_newest_and_clear(mpc_trajectory);
      if (!ret || mpc_trajectory == nullptr) {
        return;
      }
      if (mpc_trajectory->header.frame_id == "" ||
          mpc_trajectory->header.frame_id == "car") {
        return;
      }
      try {
        mjson::Reader mpc_extra_reader(mpc_trajectory->header.frame_id);
        steer_angle_offset_deg_ =
            mpc_extra_reader.get<double>("steer_angle_offset_deg", false, 0.0);
      } catch (mjson::Exception &e) {
        MSD_LOG(WARN, "mpc json parse error!");
        MSD_LOG(WARN, e.what());
      }
    }
  }

  void fill_prediction_trajectory_point(
      const std::vector<maf_worldmodel::PredictionTrajectoryPoint> &input,
      std::vector<msquare::PredictionTrajectoryPoint> &output) {
    output.resize(input.size());

    for (size_t i = 0; i < input.size(); i++) {
      output[i].x = input[i].position.x;
      output[i].y = input[i].position.y;
      output[i].yaw = input[i].yaw;
      output[i].speed = input[i].velocity;
      output[i].theta = input[i].theta;
      output[i].prob = input[i].confidence;

      output[i].std_dev_x = input[i].covariance_xy.x00;
      output[i].std_dev_y = input[i].covariance_xy.x11;
      // output[i].std_dev_yaw = input[i].std_dev_yaw;
      // output[i].std_dev_speed = input[i].std_dev_velocity;

      // output[i].relative_ego_x = input[i].relative_position.x;
      // output[i].relative_ego_y = input[i].relative_position.y;
      output[i].relative_ego_yaw =
          input[i].yaw - vehicle_status_.heading_yaw.heading_yaw_data.value_rad;
      // output[i].relative_ego_speed = input[i].relative_velocity;

      // output[i].relative_ego_std_dev_x =
      // input[i].std_dev_relative_position.x;
      // output[i].relative_ego_std_dev_y =
      // input[i].std_dev_relative_position.y;
      // output[i].relative_ego_std_dev_yaw = input[i].std_dev_relative_yaw;
      // output[i].relative_ego_std_dev_speed =
      // input[i].std_dev_relative_velocity;
    }
  }

  void fill_prediction_trajectory(
      const std::vector<maf_worldmodel::PredictionTrajectory> &input,
      std::vector<msquare::PredictionTrajectory> &output) {

    output.resize(input.size());
    for (size_t i = 0; i < input.size(); i++) {
      output[i].prob = input[i].confidence;

      switch (input[i].intention.value) {
      case IntentionType::INVALID:
        output[i].intention = "invalid";
        break;
      case IntentionType::FREE_MOVE:
        output[i].intention = "free_move";
        break;
      case IntentionType::KEEP_LANE:
        output[i].intention = "lane_keep";
        break;
      case IntentionType::LEFT_CHANGE_LANE_OUT:
        output[i].intention = "left_change_lane_out";
        break;
      case IntentionType::LEFT_CHANGE_LANE_IN:
        output[i].intention = "left_change_lane_in";
        break;
      case IntentionType::RIGHT_CHANGE_LANE_OUT:
        output[i].intention = "right_change_lane_out";
        break;
      case IntentionType::RIGHT_CHANGE_LANE_IN:
        output[i].intention = "right_change_lane_in";
        break;
      case IntentionType::GO_STRAIGHT:
        output[i].intention = "go_straight";
        break;
      case IntentionType::TURN_LEFT:
        output[i].intention = "turn_left";
        break;
      case IntentionType::TURN_RIGHT:
        output[i].intention = "turn_right";
        break;
      case IntentionType::U_TURN:
        output[i].intention = "u_turn";
        break;
      default:
        output[i].intention = "";
        break;
      }

      auto object_reader = mjson::Reader(input[i].extra_json);
      output[i].source = object_reader.get<std::string>("source", false);

      std::string sigma =
          object_reader.get<std::string>("b_valid_sigma", false);
      output[i].b_valid_sigma = (atof(sigma.c_str()) > 0.0);

      std::string minor_modal =
          object_reader.get<std::string>("b_minor_modal", false);
      output[i].b_minor_modal = (atof(minor_modal.c_str()) > 0.0);

      output[i].prediction_interval = input[i].prediction_interval;
      output[i].num_of_points = input[i].trajectory_points.size();

      // output[i].const_vel_prob = input[i].status.const_velocity_confidence;
      // output[i].const_acc_prob =
      // input[i].status.const_acceleration_confidence; output[i].still_prob =
      // input[i].status.still_confidence; output[i].coord_turn_prob =
      // input[i].status.turn_confidence;

      fill_prediction_trajectory_point(input[i].trajectory_points,
                                       output[i].trajectory);
    }
  }

  void fill_prediction_object_info(
      const std::shared_ptr<maf_worldmodel::PredictionResult>
          &prediction_result,
      const std::shared_ptr<maf_worldmodel::ObjectsInterface>
          &objects_interface,
      std::vector<PredictionObject> &output) {

    const auto &input = prediction_result->object_prediction_data;
    const auto &perception_object = objects_interface->object_interface;

    std::unordered_map<uint64_t, ObjectInterface> perception_object_map{};

    for (auto &object : perception_object) {
      if (object.is_ignore) {
        continue;
      }
      perception_object_map[object.object_fusion_data.track_id] = object;
    }

    output.clear();
    for (size_t i = 0; i < input.size(); i++) {
      auto perception_object_iterator =
          perception_object_map.find(input[i].object_id);
      if (perception_object_iterator == perception_object_map.end()) {
        continue;
      }

      const auto &object_fusion_data =
          perception_object_iterator->second.object_fusion_data;
      const auto &object_prediction_data = input[i];
      auto object_fusion_type =
          from_msd_fusion_type(object_fusion_data.type_info);

      PredictionObject cur_prediction_info;
      cur_prediction_info.id = object_fusion_data.track_id;
      cur_prediction_info.type = object_fusion_type;

      cur_prediction_info.position_x = object_fusion_data.position.x;
      cur_prediction_info.position_y = object_fusion_data.position.y;

      cur_prediction_info.length = object_fusion_data.shape.length;
      cur_prediction_info.width = object_fusion_data.shape.width;

      cur_prediction_info.speed = std::hypot(object_fusion_data.velocity.x,
                                             object_fusion_data.velocity.y);
      cur_prediction_info.yaw = object_fusion_data.heading_yaw;
      cur_prediction_info.acc =
          object_fusion_data.acceleration_relative_to_ground.x;

      cur_prediction_info.bottom_polygon_points.resize(
          object_fusion_data.polygon_bottom.points.size());
      for (size_t j = 0; j < object_fusion_data.polygon_bottom.points.size();
           j++) {
        cur_prediction_info.bottom_polygon_points[j].x =
            double(object_fusion_data.polygon_bottom.points[j].x);
        cur_prediction_info.bottom_polygon_points[j].y =
            double(object_fusion_data.polygon_bottom.points[j].y);
        cur_prediction_info.bottom_polygon_points[j].z =
            double(object_fusion_data.polygon_bottom.points[j].z);
      }

      cur_prediction_info.top_polygon_points.resize(
          object_fusion_data.polygon_top.points.size());
      for (size_t j = 0; j < object_fusion_data.polygon_top.points.size();
           j++) {
        cur_prediction_info.top_polygon_points[j].x =
            double(object_fusion_data.polygon_top.points[j].x);
        cur_prediction_info.top_polygon_points[j].y =
            double(object_fusion_data.polygon_top.points[j].y);
        cur_prediction_info.top_polygon_points[j].z =
            double(object_fusion_data.polygon_top.points[j].z);
      }

      cur_prediction_info.is_ego_lane_overlap = false;
      if (object_fusion_data.vehicle_state_info.ego_lane_relation_info
              .ego_lane_relation.value ==
          VehicleEgoLaneRelationEnum::VEHICLE_EGO_LANE_RELATION_OVERLAP) {
        cur_prediction_info.is_ego_lane_overlap = true;
      }
      int debug_id = object_fusion_data.track_id;
      int debug_overlap = object_fusion_data.vehicle_state_info
                              .ego_lane_relation_info.ego_lane_relation.value;
      MSD_LOG(INFO, "DEBUG_CJ831:ID:%d,Overlap:%d", debug_id, debug_overlap);

      cur_prediction_info.timestamp_us =
          objects_interface->meta.sensor_timestamp_us;

      auto object_reader = mjson::Reader(object_prediction_data.extra_json);

      std::string freemove =
          object_reader.get<std::string>("b_backup_freemove", false);
      cur_prediction_info.b_backup_freemove = (atof(freemove.c_str()) > 0.0);

      std::string cutin_str = object_reader.get<std::string>("b_cutin", false);
      cur_prediction_info.is_cutin = (atof(cutin_str.c_str()) > 0.0);

      std::string score = object_reader.get<std::string>("cutin_score", false);
      cur_prediction_info.cutin_score = atof(score.c_str());

      fill_prediction_trajectory(object_prediction_data.trajectories,
                                 cur_prediction_info.trajectory_array);

      // ignore abnormal prediction
      if (object_prediction_data.trajectories.size() == 0 ||
          object_prediction_data.trajectories[0].trajectory_points.size() ==
              0) {
        MSD_LOG(ERROR, "abnormal prediction: %d", cur_prediction_info.id);
        continue;
      }

      output.emplace_back(cur_prediction_info);
    }

    // map fusion to prediction for static obstacles
    for (const auto &object : perception_object) {
      const auto &object_fusion_data = object.object_fusion_data;
      auto object_fusion_type =
          from_msd_fusion_type(object_fusion_data.type_info);

      if (object_fusion_type != MSD_OBJECT_TYPE_CONE_BUCKET) {
        MSD_LOG(INFO, "DBCBLXPT filter non cb i pt2");
        continue;
      }

      MSD_LOG(INFO, "map fusion to prediction, id: %d",
              object_fusion_data.track_id);

      PredictionObject cur_prediction_info;
      cur_prediction_info.id = object_fusion_data.track_id;
      cur_prediction_info.type = object_fusion_type;

      cur_prediction_info.position_x = object_fusion_data.position.x;
      cur_prediction_info.position_y = object_fusion_data.position.y;

      cur_prediction_info.length = object_fusion_data.shape.length;
      cur_prediction_info.width = object_fusion_data.shape.width;

      cur_prediction_info.speed = std::hypot(object_fusion_data.velocity.x,
                                             object_fusion_data.velocity.y);
      cur_prediction_info.yaw = object_fusion_data.heading_yaw;
      cur_prediction_info.acc =
          object_fusion_data.acceleration_relative_to_ground.x;

      cur_prediction_info.bottom_polygon_points.resize(
          object_fusion_data.polygon_bottom.points.size());
      for (size_t j = 0; j < object_fusion_data.polygon_bottom.points.size();
           j++) {
        cur_prediction_info.bottom_polygon_points[j].x =
            double(object_fusion_data.polygon_bottom.points[j].x);
        cur_prediction_info.bottom_polygon_points[j].y =
            double(object_fusion_data.polygon_bottom.points[j].y);
        cur_prediction_info.bottom_polygon_points[j].z =
            double(object_fusion_data.polygon_bottom.points[j].z);
      }

      cur_prediction_info.top_polygon_points.resize(
          object_fusion_data.polygon_top.points.size());
      for (size_t j = 0; j < object_fusion_data.polygon_top.points.size();
           j++) {
        cur_prediction_info.top_polygon_points[j].x =
            double(object_fusion_data.polygon_top.points[j].x);
        cur_prediction_info.top_polygon_points[j].y =
            double(object_fusion_data.polygon_top.points[j].y);
        cur_prediction_info.top_polygon_points[j].z =
            double(object_fusion_data.polygon_top.points[j].z);
      }

      PredictionTrajectoryPoint p;
      p.x = cur_prediction_info.position_x;
      p.y = cur_prediction_info.position_y;
      p.yaw = cur_prediction_info.yaw;
      p.speed = cur_prediction_info.speed;
      // p.relative_ego_x = p.x -
      //                   vehicle_status_.location.location_enu.x;
      // p.relative_ego_y = p.y -
      //                   vehicle_status_.location.location_enu.y;
      p.relative_ego_yaw =
          p.yaw - vehicle_status_.heading_yaw.heading_yaw_data.value_rad;
      // p.relative_ego_speed =
      //     p.speed - vehicle_status_.velocity.heading_velocity.value_mps;;
      static PredictionTrajectory traj;
      traj.trajectory.resize(41);
      for (int i = 0; i < 41; i++) {
        traj.trajectory[i] = p;
      }
      cur_prediction_info.trajectory_array.push_back(traj);
      output.emplace_back(cur_prediction_info);
    }
  }

  void fill_fusion_object_info(
      const std::vector<ObjectInterface> &object_interface,
      std::vector<maf_perception_interface::PerceptionFusionObjectData>
          &output) {
    output.clear();
    for (size_t i = 0; i < object_interface.size(); i++) {
      if (object_interface[i].is_ignore) {
        continue;
      }
      output.emplace_back(object_interface[i].object_fusion_data);
    }
  }

  void update_worldmodel_info(double current_time) {
    // fetch worldmodel_map
    maf_worldmodel::ProcessedMapData processed_map_data{};
    static uint64_t processed_map_data_time_stamp_us = 0;
    while (!worldmodel_map_receiver_->empty()) {
      std::shared_ptr<maf_worldmodel::ProcessedMap> received_worldmodel_map{};
      auto ret = worldmodel_map_receiver_->pop_oldest(received_worldmodel_map);
      if (!ret || received_worldmodel_map->header.frame_id.empty()) {
        continue;
      }

      if (received_worldmodel_map->processed_map_data.available &
          ProcessedMapData::LANE) {
        processed_map_data.available |= ProcessedMapData::LANE;
        processed_map_data.lanes =
            received_worldmodel_map->processed_map_data.lanes;
      }

      if (received_worldmodel_map->processed_map_data.available &
          ProcessedMapData::MAP_POI_INFO) {
        processed_map_data.available |= ProcessedMapData::MAP_POI_INFO;
        processed_map_data.map_poi_info =
            received_worldmodel_map->processed_map_data.map_poi_info;
      }

      if (received_worldmodel_map->processed_map_data.available &
          ProcessedMapData::INTERSECTION) {
        processed_map_data.available |= ProcessedMapData::INTERSECTION;
        processed_map_data.intersections =
            received_worldmodel_map->processed_map_data.intersections;
      }

      if (received_worldmodel_map->processed_map_data.available &
          ProcessedMapData::SELF_POSITION) {
        processed_map_data.available |= ProcessedMapData::SELF_POSITION;
        processed_map_data.self_position =
            received_worldmodel_map->processed_map_data.self_position;
      }

      if (received_worldmodel_map->processed_map_data.available &
          ProcessedMapData::LANE_STRATEGY) {
        processed_map_data.available |= ProcessedMapData::LANE_STRATEGY;
        processed_map_data.lane_strategy =
            received_worldmodel_map->processed_map_data.lane_strategy;
      }

      if (received_worldmodel_map->processed_map_data.available &
          ProcessedMapData::LANE_MERGING_SPLITTING_POINT) {
        processed_map_data.available |=
            ProcessedMapData::LANE_MERGING_SPLITTING_POINT;
        processed_map_data.lane_merging_splitting_points =
            received_worldmodel_map->processed_map_data
                .lane_merging_splitting_points;
      }

      if (received_worldmodel_map->processed_map_data.available &
          ProcessedMapData::EXTRA_INFO) {
        processed_map_data.available |= ProcessedMapData::EXTRA_INFO;
        processed_map_data.extra_info =
            received_worldmodel_map->processed_map_data.extra_info;
      }

      processed_map_data_time_stamp_us =
          received_worldmodel_map->meta.egopose_timestamp_us;
    }

    if ((processed_map_data.available & ProcessedMapData::LANE) &&
        (processed_map_data.available & ProcessedMapData::MAP_POI_INFO) &&
        (processed_map_data.available & ProcessedMapData::INTERSECTION) &&
        (processed_map_data.available & ProcessedMapData::SELF_POSITION) &&
        (processed_map_data.available & ProcessedMapData::LANE_STRATEGY) &&
        (processed_map_data.available &
         ProcessedMapData::LANE_MERGING_SPLITTING_POINT)) {
      // only feed map info when hdmap is valid
      MSDMapInfo msd_map_info(processed_map_data);
      world_model_->feed_map_info(msd_map_info);
      last_feed_time_[FEED_MAP_INFO] = current_time;

      double processed_map_delay =
          current_time - processed_map_data_time_stamp_us / 1.0e+6;
      MSD_LOG(WARN, "[processed_map_delay_time] processed_map_delay %f ",
              processed_map_delay);
    }

    while (!worldmodel_objects_receiver_->empty()) {
      std::shared_ptr<maf_worldmodel::ObjectsInterface> perception_objects{};
      auto ret = worldmodel_objects_receiver_->pop_oldest(perception_objects);
      if (!ret) {
        continue;
      }

      static std::vector<maf_perception_interface::PerceptionFusionObjectData>
          fusion_info{};
      fill_fusion_object_info(perception_objects->object_interface,
                              fusion_info);
      world_model_->feed_fusion_info(fusion_info);

      (void)worldmodel_objects_buffer_.push(perception_objects);
      last_feed_time_[FEED_FUSION_INFO] = current_time;

      // std::shared_ptr<ddp::FusionObjectManager> fusion_object_manager =
      //     ddp::DdpContext::Instance()->fusion_object_manager();
      // if (fusion_object_manager != nullptr) {
      //   static maf_perception_interface::PerceptionFusionObjectResult
      //       fusion_object_result;
      //   fusion_object_result.header = perception_objects->header;
      //   fusion_object_result.perception_fusion_objects_data = fusion_info;
      //   fusion_object_result.meta.sensor_timestamp_us =
      //       perception_objects->meta.sensor_timestamp_us;

      //   fusion_object_manager->feed_fusion_object_info(fusion_object_result);
      // }

      double perception_objects_delay =
          current_time - perception_objects->meta.sensor_timestamp_us / 1.0e+6;
      MSD_LOG(WARN,
              "[perception_objects_delay_time] perception_objects_delay %f ",
              perception_objects_delay);
    }

    while (!traffic_light_receiver_->empty()) {
      std::shared_ptr<maf_perception_interface::TrafficLightPerception>
          traffic_light{};
      auto ret = traffic_light_receiver_->pop_oldest(traffic_light);
      if (ret &&
          (traffic_light->available &
           TrafficLightPerception::VISION_PERCEPTION_TRAFFIC_LIGHT_RESULT) &&
          (traffic_light->traffic_light_perception_result.traffic_light_decision
               .size() > 0)) {
        world_model_->feed_traffic_light_info(
            current_time, traffic_light->traffic_light_perception_result
                              .traffic_light_decision);

        double traffic_light_delay =
            current_time - traffic_light->meta.sensor_timestamp_us / 1.0e+6;
        MSD_LOG(WARN, "[traffic_light_delay_time] traffic_light_delay %f ",
                traffic_light_delay);
      }
    }
  }

  void update_prediction_info(double current_time) {
    std::shared_ptr<maf_worldmodel::PredictionResult> prediction_result{};

    while (!prediction_result_receiver_->empty()) {
      auto ret = prediction_result_receiver_->fetch_newest_and_clear(
          prediction_result);
      if (!ret) {
        continue;
      }
    }

    // feed prediction info when hdmap valid is true
    if (prediction_result != nullptr && worldmodel_objects_buffer_.size() > 0) {
      int index = -1;
      for (int i = 0; i < worldmodel_objects_buffer_.size(); i++) {
        if (worldmodel_objects_buffer_.at(i)->meta.sensor_timestamp_us ==
            prediction_result->meta.sensor_timestamp_us) {
          index = i;
          MSD_LOG(WARN,
                  "update_prediction_info, sensor_timestamp matched! index = "
                  "%d, size = %d",
                  index, worldmodel_objects_buffer_.size());
          break;
        }
      }
      // sensor_timestamp not matched, use latest
      if (index < 0) {
        index = worldmodel_objects_buffer_.size() - 1;
        MSD_LOG(WARN,
                "update_prediction_info, sensor_timestamp not matched! index "
                "= %d",
                index);
      }

      last_feed_msg_id_[FEED_PREDICTION_INFO] =
          mlog::MLOG_msg_id(prediction_result->header.stamp, "/msd/prediction");
      msd_planning::MSDPlanning_record_timestamp(
          last_feed_msg_id_[FEED_PREDICTION_INFO], "planning/recv", 0);
      static std::vector<PredictionObject> prediction_object{};
      fill_prediction_object_info(prediction_result,
                                  worldmodel_objects_buffer_.at(index),
                                  prediction_object);
      world_model_->feed_prediction_info(prediction_object);
      last_feed_time_[FEED_PREDICTION_INFO] = current_time;
    }
  }

  void update_planning_request(double current_time) {
    while (!planning_request_receiver_->empty()) {
      maf_system_manager::SysPlanningRequest request{};
      auto ret = planning_request_receiver_->pop_oldest(request);
      if (!ret) {
        continue;
      }

      auto set_underclocking = [this]() {
        this->underclocking_ = underclocking_enum_;
      };
      auto reset_underclocking = [this]() {
        this->underclocking_ = 0U;
        this->underclocking_cnt_ = 0U;
      };

      switch (request.cmd.value) {
      case maf_system_manager::SystemCmdTypeEnum::
          PLANNING_HIGHWAY_FUNCTION_MODE:
        switch (request.highway_info.function_mode.value) {
        case maf_system_manager::FunctionModeEnum::ACC:
          world_model_->set_acc_mode(true);
          world_model_->set_ddmap(true);
          MSD_LOG(WARN, "planning_request, function mode: set to acc mode");
          reset_underclocking();
          is_hnp_mode_ = false;
          break;
        case maf_system_manager::FunctionModeEnum::CP:
          world_model_->set_acc_mode(false);
          world_model_->set_ddmap(true);
          MSD_LOG(WARN, "planning_request, function mode: set to pilot mode");
          reset_underclocking();
          is_hnp_mode_ = false;
          break;
        case maf_system_manager::FunctionModeEnum::HNP:
          world_model_->set_acc_mode(false);
          world_model_->set_ddmap(true); // ddmap always set to true
          MSD_LOG(WARN, "planning_request, function mode: set to hnp mode");
          set_underclocking();
          is_hnp_mode_ = true;
          break;
        default:
          MSD_LOG(ERROR,
                  "planning_request, function mode: invalid function mode: %d",
                  request.highway_info.function_mode.value);
          set_underclocking();
          is_hnp_mode_ = true;
          break;
        }
        break;
      case maf_system_manager::SystemCmdTypeEnum::
          PLANNING_HIGHWAY_NAVI_SETTINGS:
        cruise_velocity_ = request.highway_info.navi_settings.navi_max_speed;
        MSD_LOG(WARN, "planning_request, speed: set to %f kph",
                request.highway_info.navi_settings.navi_max_speed);

        if (request.highway_info.navi_settings.navi_time_distance.value >= 0 &&
            request.highway_info.navi_settings.navi_time_distance.value <= 4) {
          navi_ttc_gear_ =
              request.highway_info.navi_settings.navi_time_distance.value;
          world_model_->set_navi_ttc_gear(
              request.highway_info.navi_settings.navi_time_distance.value);
        }
        break;
      case maf_system_manager::SystemCmdTypeEnum::
          PLANNING_HIGHWAY_DRIVING_STYLE:
        switch (request.highway_info.driving_style.value) {
        case maf_system_manager::DrivingStyleEnum::AGGRESIVE:
          world_model_->set_driving_style(DRIVING_STYLE_AGGRESSIVE);
          break;
        case maf_system_manager::DrivingStyleEnum::NORMAL:
          world_model_->set_driving_style(DRIVING_STYLE_NORMAL);
          break;
        case maf_system_manager::DrivingStyleEnum::CONSERVATIVE:
          world_model_->set_driving_style(DRIVING_STYLE_CONSERVATIVE);
          break;
        case 3:
          world_model_->set_lane_changing_style(LANECHANGING_STYLE_AGGRESSIVE);
          break;
        case 4:
          world_model_->set_lane_changing_style(LANECHANGING_STYLE_NORMAL);
          break;
        case 5:
          world_model_->set_lane_changing_style(
              LANECHANGING_STYLE_CONSERVATIVE);
          break;
        default:
          MSD_LOG(WARN,
                  "planning_request, driving mode: invalid driving mode: %d",
                  request.highway_info.driving_style.value);
          break;
        }
        break;
      case maf_system_manager::SystemCmdTypeEnum::PLANNING_HIGHWAY_LANE_CHANGE:
        if (request.highway_info.lane_change_cmd.type.value ==
            maf_system_manager::LaneChangeTypeEnum::INTERACTIVE) {
          switch (request.highway_info.lane_change_cmd.direction.value) {
          case maf_system_manager::LaneChangeDirectionEnum::LEFT:
            vehicle_light_.vehicle_light_data.turn_signal.value =
                maf_endpoint::LeverStatus::LEVER_STATE_LEFT;
            break;
          case maf_system_manager::LaneChangeDirectionEnum::RIGHT:
            vehicle_light_.vehicle_light_data.turn_signal.value =
                maf_endpoint::LeverStatus::LEVER_STATE_RIGHT;
            break;
          default:
            vehicle_light_.vehicle_light_data.turn_signal.value =
                maf_endpoint::LeverStatus::LEVER_STATE_OFF;
            break;
          }
        } else if (request.highway_info.lane_change_cmd.type.value ==
                   maf_system_manager::LaneChangeTypeEnum::FORBIDDEN) {
          enable_ilc_ = false;
        } else if (request.highway_info.lane_change_cmd.type.value ==
                   maf_system_manager::LaneChangeTypeEnum::REMOVE_FORBIDDEN) {
          enable_ilc_ = true;
        } else if (request.highway_info.lane_change_cmd.type.value ==
                   maf_system_manager::LaneChangeTypeEnum::REMOVE_INTERACTIVE) {
          // ignore lc cancel for simulation
          auto env = std::getenv("RealitySimulation");
          if (env != nullptr) {
            std::string relality_simulation(env);
            MSD_LOG(INFO, "relality_simulation: %s",
                    relality_simulation.c_str());
            if (std::strcmp(env, "simulation") == 0) {
              continue;
            }
          }
          vehicle_light_.vehicle_light_data.turn_signal.value =
              maf_endpoint::LeverStatus::LEVER_STATE_OFF;
        } else if (request.highway_info.lane_change_cmd.type.value == 8) {
          // enable alc
          const bool enable_alc =
              ConfigurationContext::Instance()
                  ->planner_config()
                  .lateral_behavior_planner_config.enable_alc;
          world_model_->set_enable_alc(enable_alc);
        } else if (request.highway_info.lane_change_cmd.type.value == 9) {
          // disable alc
          world_model_->set_enable_alc(false);
        } else if (request.highway_info.lane_change_cmd.type.value == 10) {
          // enable avd
          world_model_->set_enable_lateral_dogde(true);
        } else if (request.highway_info.lane_change_cmd.type.value == 11) {
          // disable avd
          world_model_->set_enable_lateral_dogde(false);
        }
        if (!enable_ilc_) {
          vehicle_light_.vehicle_light_data.turn_signal.value =
              maf_endpoint::LeverStatus::LEVER_STATE_OFF;
        }
        if (request.highway_info.lane_change_cmd.type.value >=
                maf_system_manager::LaneChangeTypeEnum::FORBIDDEN &&
            request.highway_info.lane_change_cmd.type.value <=
                maf_system_manager::LaneChangeTypeEnum::REMOVE_INTERACTIVE) {
          world_model_->set_ego_blinker(vehicle_light_);
        }

        break;
      case maf_system_manager::SystemCmdTypeEnum::PLANNING_HIGHWAY_START_STOP:
        if (request.highway_info.start_stop_cmd.value ==
            maf_system_manager::StartStopCmdEnum::START) {
          world_model_->set_force_stop(false);
        } else if (request.highway_info.start_stop_cmd.value ==
                   maf_system_manager::StartStopCmdEnum::STOP) {
          world_model_->set_force_stop(true);
        }

        break;
      case 0x000a1000: // PLANNING_MRC_INLANE_BRAKE
        world_model_->set_mrc_inlane_brake(true);
        break;
      default:
        break;
      }
    }

    auto env = std::getenv("DISABLE_ILC");
    if (env != nullptr && std::strcmp(env, "TRUE") == 0) {
      enable_ilc_ = false;
    }
    MSD_LOG(INFO, "enable ilc: %d\n", enable_ilc_);
    world_model_->set_enable_ilc(enable_ilc_);

    // clear ilc when not active or diabled
    bool active = world_model_->get_vehicle_dbw_status();
    if (!active || !enable_ilc_) {
      vehicle_light_.vehicle_light_data.turn_signal.value =
          maf_endpoint::LeverStatus::LEVER_STATE_OFF;
      world_model_->set_ego_blinker(vehicle_light_);
    }

    // clear force stop when taking over or ego_vel > 1m/s
    double ego_vel =
        world_model_->get_cart_ego_state_manager().get_cart_ego_state().ego_vel;
    static bool last_active = false;
    if ((last_active && !active) || (ego_vel > 1.0)) {
      world_model_->set_force_stop(false);
    }
    last_active = active;
  }

  void update_planning_reset_request(double current_time) {
    while (!planning_reset_request_receiver_->empty()) {
      maf_std::Header request{};
      auto ret =
          planning_reset_request_receiver_->fetch_newest_and_clear(request);
      if (!ret) {
        continue;
      }

      std::string reset = "False";
      std::string which_car = "";
      std::string enable_snapshot = "False";

      try {
        mjson::Reader reset_json(request.frame_id);
        reset = reset_json.get<std::string>("reset", false, "False");
        which_car = reset_json.get<std::string>("WHICH_CAR", false, "");
        planning_seq_ = 0;
        localization_seq_ = 0;
        enable_snapshot =
            reset_json.get<std::string>("enable_snapshot", false, "False");
      } catch (mjson::Exception &e) {
      }

      if (reset == "True") {
        reset_ = true;
      }

      MSD_LOG(INFO, "planning_reset: %s, %s, %s\n", reset.c_str(),
              which_car.c_str(), enable_snapshot.c_str());
    }
  }

  void update_perception_vision_lane(double current_time) {
    while (!perception_vision_lane_receiver_->empty()) {
      std::shared_ptr<maf_perception_interface::RoadLinePerception>
          perception_vision_lane_ptr{};
      auto ret = perception_vision_lane_receiver_->fetch_newest_and_clear(
          perception_vision_lane_ptr);
      if (!ret) {
        continue;
      }
      perception_vision_lane_ = *perception_vision_lane_ptr;
      last_feed_time_[FEED_VISION_LANE] = current_time;

      world_model_->feed_perception_vision_lane(perception_vision_lane_);

      // auto vision_lane_manager =
      //     ddp::DdpContext::Instance()->vision_lane_manager();
      // if (vision_lane_manager != nullptr) {
      //   vision_lane_manager->feed_vision_lane(
      //       perception_vision_lane_.meta.sensor_timestamp_us / 1e6,
      //       perception_vision_lane_);
      // }
    }
  }

  void update_perception_lidar_road_edge(double current_time) {
    while (!perception_lidar_road_edge_receiver_->empty()) {
      std::shared_ptr<maf_perception_interface::RoadLinePerception>
          perception_lidar_road_edge_ptr;
      auto ret = perception_lidar_road_edge_receiver_->fetch_newest_and_clear(
          perception_lidar_road_edge_ptr);
      if (!ret) {
        continue;
      }
      perception_lidar_road_edge_ = *(perception_lidar_road_edge_ptr);
      // last_feed_time_[FEED_LIDAR_ROAD_EDGE] = current_time;

      world_model_->feed_perception_lidar_road_edge(
          perception_lidar_road_edge_);
    }
  }

  void update_perception_radar(double current_time) {
    while (!perception_radar_receiver_->empty()) {
      std::shared_ptr<maf_perception_interface::RadarPerceptionResult>
          perception_radar_ptr{};
      auto ret = perception_radar_receiver_->fetch_newest_and_clear(
          perception_radar_ptr);
      if (!ret) {
        continue;
      }
      perception_radar_ = *perception_radar_ptr;
      // last_feed_time_[FEED_RADAR] = current_time;

      world_model_->feed_perception_radar(perception_radar_);
    }
  }

  void update_mff_info(double current_time) {
    while (!mff_info_receiver_->empty()) {
      std::shared_ptr<maf_std::Header> mff_info{};
      auto ret = mff_info_receiver_->pop_oldest(mff_info);
      if (!ret) {
        continue;
      }

      LcaStatus lca_status{};
      int driving_model;

      try {
        mjson::Reader mff_info_reader(mff_info->frame_id);
        auto highway_info =
            mff_info_reader.get<std::string>("highway_info", false, "");
        if (highway_info == "") {
          continue;
        }
        mjson::Reader highway_info_reader(highway_info);
        lca_status.lca_left_activated =
            highway_info_reader.get<bool>("LCA_left_warning", false, false);
        lca_status.lca_right_activated =
            highway_info_reader.get<bool>("LCA_right_warning", false, false);
        driving_model = mff_info_reader.get<int>("drive_style", false, 0);
      } catch (mjson::Exception &e) {
        MSD_LOG(WARN, "mff info parse error!");
        MSD_LOG(WARN, e.what());
      }

      DrivingModelConfig driving_model_config;
      if (driving_model == 1) {
        driving_model_config = DrivingModelConfig::SAFE_DRIVING_MODEL;
      } else if (driving_model == 2) {
        driving_model_config = DrivingModelConfig::STEADY_DRIVING_MODEL;
      } else if (driving_model == 3) {
        driving_model_config = DrivingModelConfig::RADICAL_DRIVING_MODEL;
      } else if (driving_model == 4) {
        driving_model_config = DrivingModelConfig::SELF_LEARNING_MODEL;
      } else {
        driving_model_config = DrivingModelConfig::NO_REQUEST_DrivingModel;
      }

      world_model_->feed_lca_status(lca_status);
      world_model_->feed_driving_model_config(driving_model_config);
    }
  }

  void update_perception_fusion_object(double current_time) {
    while (!perception_fusion_object_receiver_->empty()) {
      MSD_LOG(INFO, "perception_fusion_object_receiver is not empty");
      std::shared_ptr<maf_perception_interface::PerceptionFusionObjectResult>
          perception_fusion_object_ptr;
      auto ret = perception_fusion_object_receiver_->fetch_newest_and_clear(
          perception_fusion_object_ptr);
      if (!ret) {
        continue;
      }

      perception_fusion_object_ = *(perception_fusion_object_ptr);
      world_model_->feed_perception_fusion_object(perception_fusion_object_);
    }
  }

  void update_world_model(double current_time) {
    MLOG_PROFILING("update_world_model");

    update_planning_reset_request(current_time);

    if (!module_status_receiver_->empty()) {
      ModuleStatus module_status{};
      auto ret = module_status_receiver_->fetch_newest_and_clear(module_status);
      if (ret) {
        bool status{true};
        if (ModuleStatus::STOP == module_status.status ||
            ModuleStatus::RUNNING_TAKING_OVER == module_status.detail_status) {
          status = false;
        }

        // NOTE: 这是段历史代码，用于仿真进入自动的，对于正常仿真并没有作用。
        // 注释掉它是因为FPP仿真会走到分支里面去，相当于进默认进自动了，影响规划结果
        // auto env = std::getenv("WHICH_CAR");
        // if (env != nullptr) {
        //   std::string which_car(env);
        //   MSD_LOG(INFO, "WHICH_CAR: %s", which_car.c_str());
        //   if (std::strcmp(env, "MKZ_SIM") == 0) {
        //     MSD_LOG(INFO, "FPP: update_world_model:MKZ_SIM: status=%d",
        //     (int)status); status = true;
        //   }
        // }

        // set dbw to false when reset or hnp mode
        if (reset_ || is_hnp_mode_) {
          status = false;
        }

        if (!status) {
          world_model_->set_mrc_inlane_brake(false);
        }

        world_model_->feed_vehicle_dbw_status(status);
        last_feed_time_[FEED_VEHICLE_DBW_STATUS] = current_time;
      }
    }

    update_mpc_trajectory(current_time);

    update_vehicle_status(current_time);

    update_imu_report(current_time);

    update_perception_vision_lane(current_time);

    update_perception_lidar_road_edge(current_time);

    update_worldmodel_info(current_time);

    update_prediction_info(current_time);

    update_planning_request(current_time);

    update_perception_radar(current_time);

    update_mff_info(current_time);

    update_perception_fusion_object(current_time);
  }

  bool can_run(double current_time, std::string &warn_msg,
               std::string &error_msg) {
    auto to_string = [](FeedType feed_type) -> const char * {
      switch (feed_type) {
      case FEED_VEHICLE_DBW_STATUS:
        return "vehicle_dbw_status";
      case FEED_EGO_VEL:
        return "ego_vel";
      case FEED_EGO_STEER_ANGLE:
        return "ego_steer_angle";
      case FEED_EGO_ENU:
        return "ego_enu";
      case FEED_WHEEL_SPEED_REPORT:
        return "wheel_speed_report";
      case FEED_EGO_ACC:
        return "ego_acc";
      case FEED_MISC_REPORT:
        return "misc_report";
      case FEED_GEAR_REPORT:
        return "gear_report";
      case FEED_MAP_INFO:
        return "map_info";
      case FEED_FUSION_INFO:
        return "fusion_info";
      case FEED_PREDICTION_INFO:
        return "prediction_info";
      case FEED_VISION_LANE:
        return "vision_lane";
      // case FEED_LIDAR_ROAD_EDGE:
      //   return "lidar_road_edge";
      default:
        return "unknown type";
      };
    };

    static const double kCheckTimeDiffWarn = 0.5;
    static const double kCheckTimeDiffError = 1.0;

    warn_msg.clear();
    error_msg.clear();

    bool res = true;

    if (underclocking_cnt_ == underclocking_) {
      for (int i = 0; i < FEED_TYPE_MAX; ++i) {
        const char *feed_type_str = to_string(static_cast<FeedType>(i));

        if (last_feed_time_[i] > 0.0) {
          if (world_model_->is_ddmap() && i == FEED_MAP_INFO) {
            // use history results when map info is delayed
            continue;
          }
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
    } else {
      res = false;
    }

    MSD_LOG(WARN, "(%s)underclocking: (%d/%d)", __FUNCTION__,
            underclocking_cnt_, underclocking_);
    if (underclocking_ > 0) {
      underclocking_cnt_ %= underclocking_;
      ++underclocking_cnt_;
    }

    return res;
  }

  maf_planning::Planning generate_planning_output() {
    // set path_planner_input to planning output
    scenario_manager_->output_lat_dec_info();

    const auto &planning_status =
        PlanningContext::Instance()->planning_status();
    const auto &lateral_output =
        PlanningContext::Instance()->lateral_behavior_planner_output();

    double start_time = MTIME()->timestamp().ms();
    maf_planning::Planning msg{};
    const auto &planning_result = planning_status.planning_result;
    const auto &change_lane_status = planning_status.lane_status.change_lane;

#if !defined(WITHOUT_CPP_LOG)
    nlohmann::json extra_json_raw = planning_result.extra_json_raw;
    if (ConfigurationContext::Instance()
            ->planner_config()
            .common_config.enable_json) {
      extra_json_raw["path_planner_input"] =
          PlanningContext::Instance()->path_planner_input();
      extra_json_raw["path_planner_output"] =
          PlanningContext::Instance()->lateral_motion_planner_output();
      // extra_json_raw["pass_intersection_planner_input"] =
      //     PlanningContext::Instance()->pass_intersection_planner_input();
      extra_json_raw["speed_planner_input"] =
          PlanningContext::Instance()->speed_planner_input();
      extra_json_raw["speed_planner_output"] =
          PlanningContext::Instance()->speed_planner_output();
      extra_json_raw["request_manager_input"] =
          PlanningContext::Instance()->request_manager_input();
      extra_json_raw["request_manager_output"] =
          PlanningContext::Instance()->request_manager_output();
      extra_json_raw["lon_decison_output"] =
          PlanningContext::Instance()->lon_decison_output();
      extra_json_raw["planner_json_debug"] =
          PlanningContext::Instance()->planner_debug();
      extra_json_raw["ddp_trajectory_info"] = ddp::DdpContext::Instance()
                                                  ->model_result_manager()
                                                  ->get_ddp_trajectory_info();
      extra_json_raw["debug_json"] =
          planning_status.planning_result.debug_json.dump();
    }
#endif
#if !defined(WITHOUT_CPP_LOG)
    extra_json_raw["ddp_model_input"] =
        PlanningContext::Instance()->ddp_model_input().c_str();

    extra_json_raw["is_ddmap"] = (world_model_->is_ddmap());
    extra_json_raw["pnc_start"] = PlanningContext::Instance()
                                      ->speed_planner_input()
                                      .stop_point_info.should_start;
    extra_json_raw["pnc_stop"] = PlanningContext::Instance()
                                     ->speed_planner_input()
                                     .stop_point_info.standstill;
    extra_json_raw["baseline_jitter"] = world_model_->get_baseline_jitter();
    extra_json_raw["v_curv"] = world_model_->get_v_curv();
    extra_json_raw["a_curv"] = world_model_->get_a_curv();
    extra_json_raw["max_curv"] = world_model_->get_max_curv();
    extra_json_raw["max_curv_distance"] = world_model_->get_max_curv_distance();

    extra_json_raw["headway"] =
        speed_planner::NORMAL_HEADWAY_TABLE[world_model_->get_navi_ttc_gear()];

    extra_json_raw["use_eftp"] = world_model_->use_eftp();
    extra_json_raw["lateral_use_eftp"] = world_model_->lateral_use_eftp();
    extra_json_raw["b_dagger_longitudinal"] =
        ConfigurationContext::Instance()
            ->planner_config()
            .common_config.b_dagger_longitudinal;
    extra_json_raw["b_dagger_lateral"] = ConfigurationContext::Instance()
                                             ->planner_config()
                                             .common_config.b_dagger_lateral;
#endif

    // calc cipv desired hw and real hw
    const auto &speed_planner_input =
        PlanningContext::Instance()->speed_planner_input();
    bool is_cipv_exist = false;
    double desired_hw = -1.0;
    double real_hw = -1.0;
    double cipv_dist = -1.0;
    double cipv_v = -1.0;
    for (const auto &obs : speed_planner_input.lon_obs) {
      if (speed_planner_input.cipv_info.cipv_id == obs.id) {
        is_cipv_exist = true;
        desired_hw = obs.polygon_init.desired_headway;
        cipv_dist =
            obs.polygon_init.rel_s -
            ConfigurationContext::Instance()->get_vehicle_param().length / 2;
        cipv_v = obs.polygon_init.v;
        break;
      }
    }
    if (is_cipv_exist) {
      if (cipv_v < 3.0) {
        real_hw = -1.0;
      } else if (cipv_dist > 4.0 && cipv_v > 0.0) {
        real_hw = std::min((cipv_dist - 4.0) / std::max(cipv_v, 1.0), 5.0);
      }
    } else {
      desired_hw = -1.0;
      real_hw = -1.0;
    }
#if !defined(WITHOUT_CPP_LOG)
    extra_json_raw["desired_headway"] = desired_hw;
    extra_json_raw["real_headway"] = real_hw;
#endif

    CostTime cost_time =
        CostTime{"json", MTIME()->timestamp().ms() - start_time};
    auto *planner_debug = PlanningContext::Instance()->mutable_planner_debug();
    planner_debug->cost_time.clear();
    planner_debug->cost_time.emplace_back(cost_time);

    auto timestamp = MTIME()->timestamp("publish");
    msd_planning::MSDPlanning_record_timestamp(planning_status.trigger_msg_id,
                                               "pnc_plan_timestamp-publish",
                                               timestamp.ns());
    msg.header.stamp = timestamp.ns();
    msg.header.frame_id = "enu";
    msg.meta.timestamp_us = ego_pose_timestamp_us_;
    msg.meta.plan_timestamp_us =
        static_cast<uint64_t>(planning_result.timestamp_sec * 1000000.0);
    msg.trajectory.available |= maf_planning::Trajectory::PATH;
    msg.trajectory.path.resize(planning_result.traj_pose_array.size());
    for (size_t i = 0; i < msg.trajectory.path.size(); i++) {
      msg.trajectory.path[i].position_enu.x =
          planning_result.traj_pose_array[i].position_enu.x;
      msg.trajectory.path[i].position_enu.y =
          planning_result.traj_pose_array[i].position_enu.y;

      msg.trajectory.path[i].curvature =
          planning_result.traj_pose_array[i].curvature;
      msg.trajectory.path[i].heading_yaw =
          planning_result.traj_pose_array[i].heading_yaw;
    }

    msg.trajectory.available |= maf_planning::Trajectory::VELOCITY;
    msg.trajectory.velocity.available |= maf_planning::Velocity::VEL_POINTS;
    msg.trajectory.velocity.vel_points.resize(
        planning_result.traj_vel_array.size());
    for (size_t i = 0; i < msg.trajectory.velocity.vel_points.size(); i++) {
      msg.trajectory.velocity.vel_points[i].distance =
          planning_result.traj_vel_array[i].distance;
      msg.trajectory.velocity.vel_points[i].relative_time =
          planning_result.traj_vel_array[i].relative_time;
      msg.trajectory.velocity.vel_points[i].target_velocity =
          planning_result.traj_vel_array[i].target_velocity;
    }

    msg.trajectory.available |= maf_planning::Trajectory::ACCELERATION;
    msg.trajectory.acceleration.available |=
        maf_planning::Acceleration::ACC_POINTS;
    msg.trajectory.acceleration.acc_points.resize(
        planning_result.traj_acceleration.size());
    for (size_t i = 0; i < planning_result.traj_acceleration.size(); i++) {
      msg.trajectory.acceleration.acc_points[i].acc =
          planning_result.traj_acceleration[i];
      msg.trajectory.acceleration.acc_points[i].jerk = 0.0;
    }
    for (size_t i = 0; i < std::min(planning_result.jerk_output.size(),
                                    planning_result.traj_acceleration.size());
         i++) {
      msg.trajectory.acceleration.acc_points[i].jerk =
          planning_result.jerk_output[i];
    }

    msg.extra.available |= maf_planning::PlanExtra::VERSION;
    msg.extra.version = PlanningContext::Instance()->get_version();

    double start_time2 = MTIME()->timestamp().ms();

#if !defined(WITHOUT_CPP_LOG)
    msg.extra.available |= maf_planning::PlanExtra::JSON;
    msg.extra.json = extra_json_raw.dump();
#endif

    CostTime cost_time2 =
        CostTime{"json_dump", MTIME()->timestamp().ms() - start_time2};
    planner_debug->cost_time.emplace_back(cost_time2);

    msg.trajectory.velocity.available |=
        maf_planning::Velocity::CRUISE_VELOCITY;
    msg.trajectory.velocity.cruise_velocity = cruise_velocity_ / 3.6;

    msg.turn_signal_command.available |=
        maf_planning::TurnSignalCommand::TURN_SIGNAL_DATA;
    msg.turn_signal_command.turn_signal_data.value =
        planning_result.turn_signal_cmd.value;

    msg.gear_command.available |= maf_planning::GearCommand::GEAR_DATA;
    msg.gear_command.gear_data.value = planning_result.gear_cmd.value;

    double ego_vel =
        world_model_->get_cart_ego_state_manager().get_cart_ego_state().ego_vel;
    if (world_model_->is_mrc_inlane_brake()) {
      if (ego_vel < 0.1) {
        msg.gear_command.gear_data.value = maf_planning::Gear::PARK;
      }
      msg.turn_signal_command.turn_signal_data.value =
          maf_planning::TurnSignal::EMERGENCY_FLASHER;
    }

    msg.plan_status.available |= maf_planning::PlanStatus::ALGORITHM_STATUS;
    msg.plan_status.algorithm_status.scene = lateral_output.planner_scene;
    msg.plan_status.algorithm_status.action = lateral_output.planner_action;
    msg.plan_status.algorithm_status.action_status =
        lateral_output.planner_status;
    // lc_request_source  0:none; 1:int; 2:map; 3:act
    int request_source = 0;
    if (lateral_output.lc_request_source == "none") {
      request_source = 0;
    } else if (lateral_output.lc_request_source == "int_request") {
      request_source = 1;
    } else if (lateral_output.lc_request_source == "map_request") {
      request_source = 2;
    } else if (lateral_output.lc_request_source == "act_request") {
      request_source = 3;
    }

    msg.plan_status.algorithm_status.source = request_source;

    // set to inersect for cp intersect refline generator
    if ((world_model_->get_refline_condition().reflinecondition ==
             pass_intersection_planner::ReflineCondition::
                 FIRST_HALF_INTERSECTION ||
         world_model_->get_refline_condition().reflinecondition ==
             pass_intersection_planner::ReflineCondition::IN_INTERSECTION ||
         world_model_->get_refline_condition().reflinecondition ==
             pass_intersection_planner::ReflineCondition::
                 LAST_HALF_INTERSECTION) &&
        world_model_->is_ddmap()) {
      msg.plan_status.algorithm_status.scene =
          maf_planning::PlanAlgorithmStatus::INTERSECT;
    }

    msg.plan_status.hdmap_valid = true;

    // msg.trajectory.available |= maf_planning::Trajectory::POLYNOMIAL_CURVE;
    // msg.trajectory.polynomial_curve.polynomial = lateral_output.d_poly;

    msg.comfort.available |= maf_planning::Comfort::MANEUVER_GEAR;
    msg.comfort.maneuver_gear.value = maf_planning::ManeuverGear::NORMAL;

    msg.decision.available |= maf_planning::Decision::LON_DECISION;
    msg.decision.lon_decision.follow_obstacles.assign(
        planning_result.lon_follow_obstacles.begin(),
        planning_result.lon_follow_obstacles.end());

    msg.decision.lon_decision.overtake_obstacles.assign(
        planning_result.lon_overtake_obstacles.begin(),
        planning_result.lon_overtake_obstacles.end());

    msg.decision.lon_decision.lon_nudge_obstacles.assign(
        planning_result.id_nudge.begin(), planning_result.id_nudge.end());

    msg.decision.lon_decision.lc_gap_info.clear();
    msg.decision.lon_decision.lc_gap_info.push_back(
        change_lane_status.target_gap_obs.first);
    msg.decision.lon_decision.lc_gap_info.push_back(
        change_lane_status.target_gap_obs.second);

    msg.decision.lon_decision.id_yield_closest = planning_result.id_yield;
    msg.decision.lon_decision.id_overtake_closest = planning_result.id_overtake;
    msg.decision.lon_decision.tag_yield = planning_result.tag_yield;
    msg.decision.lon_decision.N_hard = planning_result.N_hard;
    msg.decision.lon_decision.dt_yield = planning_result.num_yield * 0.2;
    msg.decision.lon_decision.dt_set = planning_result.t_set;
    msg.decision.lon_decision.ds_set = planning_result.ds_set;
    msg.decision.lon_decision.ds_yield = planning_result.dsl_yield;
    msg.decision.lon_decision.ds_overtake = planning_result.dsl_overtake;
    msg.decision.lon_decision.v_set = planning_result.v_set;
    msg.decision.lon_decision.v_yield_closest = planning_result.vl_yield;
    msg.decision.lon_decision.v_overtake_closest = planning_result.v_overtake;
    msg.decision.lon_decision.v_lim = planning_result.v_limit;
    msg.decision.lon_decision.v_lim_map = planning_result.v_limit_map;
    msg.decision.lon_decision.a_set = planning_result.a_set;
    msg.decision.lon_decision.dis_close = planning_result.dis_close;

    msg.decision.lon_decision.ds_lon.clear();
    msg.decision.lon_decision.ds_lon.push_back(planning_result.ds_lon_front);
    msg.decision.lon_decision.ds_lon.push_back(planning_result.ds_lon_rear);

    msg.decision.lon_decision.ttc_lon.clear();
    msg.decision.lon_decision.ttc_lon.push_back(planning_result.ttc_lon_front);
    msg.decision.lon_decision.ttc_lon.push_back(planning_result.ttc_lon_rear);

    msg.decision.lon_decision.dt_lon.clear();
    msg.decision.lon_decision.dt_lon.push_back(planning_result.dt_lon_front);
    msg.decision.lon_decision.dt_lon.push_back(planning_result.dt_lon_rear);

    msg.decision.lon_decision.s_max.assign(planning_result.ds_yield_seq.begin(),
                                           planning_result.ds_yield_seq.end());

    msg.decision.lon_decision.d_safe.assign(planning_result.d_safe_seq.begin(),
                                            planning_result.d_safe_seq.end());

    msg.decision.lon_decision.ds_flag.assign(
        planning_result.ds_flag_seq.begin(), planning_result.ds_flag_seq.end());

    msg.decision.lon_decision.v_ref.assign(planning_result.v_ref_seq.begin(),
                                           planning_result.v_ref_seq.end());

    msg.decision.lon_decision.weights.assign(planning_result.lon_weights,
                                             planning_result.lon_weights + 4);

    // hack for control, reduce throttle brake switch
    msg.decision.lon_decision.enable_prebrake =
        PlanningContext::Instance()
            ->speed_planner_input()
            .cipv_info.is_need_accurate_control;

    msg.decision.lon_decision.enable_preacc =
        PlanningContext::Instance()
            ->speed_planner_input()
            .cipv_info.is_need_soft_control;
    // planning_status.pre_action.enable_preacc;
    msg.decision.lon_decision.need_aeb = planning_result.NeedAEB;

    msg.decision.lon_decision.pnc_start = planning_result.pnc_start;
    msg.decision.lon_decision.pnc_stop = planning_result.pnc_stop;

    msg.decision.available |= maf_planning::Decision::LAT_DECISION;
    msg.decision.lat_decision.nudge_obstacles.assign(
        planning_result.lat_nudge_obstacles.begin(),
        planning_result.lat_nudge_obstacles.end());

    msg.decision.lat_decision.fix_refline_index =
        lateral_output.fix_refline_index;
    msg.decision.lat_decision.origin_refline_index =
        lateral_output.origin_refline_index;
    msg.decision.lat_decision.target_refline_index =
        lateral_output.target_refline_index;

    msg.decision.lat_decision.avd_info.resize(lateral_output.avd_info.size());
    for (size_t i = 0; i < msg.decision.lat_decision.avd_info.size(); i++) {
      msg.decision.lat_decision.avd_info[i].id = lateral_output.avd_info[i].id;
      msg.decision.lat_decision.avd_info[i].property =
          lateral_output.avd_info[i].property;
      msg.decision.lat_decision.avd_info[i].ignore =
          lateral_output.avd_info[i].ignore;
      msg.decision.lat_decision.avd_info[i].avd_direction =
          lateral_output.avd_info[i].avd_direction;
      msg.decision.lat_decision.avd_info[i].avd_priority =
          lateral_output.avd_info[i].avd_priority;
      msg.decision.lat_decision.avd_info[i].blocked_time_begin =
          lateral_output.avd_info[i].blocked_time_begin;
      msg.decision.lat_decision.avd_info[i].blocked_time_end =
          lateral_output.avd_info[i].blocked_time_end;
    }
    const LateralObstacle &lateral_obstacle =
        world_model_->mutable_lateral_obstacle();
    // fengxiaotong debug: display gap begin s
    msg.decision.lat_decision.avd_car_past_fisrt.clear();
    msg.decision.lat_decision.avd_car_past_second.clear();
    for (const auto tr : lateral_obstacle.front_tracks()) {
      if (tr.track_id == change_lane_status.target_gap_obs.first) {
        msg.decision.lat_decision.avd_car_past_fisrt.push_back(tr.d_rel);
      }
      if (tr.track_id == change_lane_status.target_gap_obs.second) {
        msg.decision.lat_decision.avd_car_past_second.push_back(tr.d_rel);
      }
    }
    for (const auto tr : lateral_obstacle.side_tracks()) {
      if (tr.track_id == change_lane_status.target_gap_obs.first) {
        msg.decision.lat_decision.avd_car_past_fisrt.push_back(tr.d_rel);
      }
      if (tr.track_id == change_lane_status.target_gap_obs.second) {
        msg.decision.lat_decision.avd_car_past_second.push_back(tr.d_rel);
      }
    }
    // hack: use lc_back_id to show lc_state
    if (lateral_output.lc_status == "left_lane_change_wait" ||
        lateral_output.lc_status == "right_lane_change_wait") {
      msg.decision.lat_decision.lc_back_id = int(0);
    } else if (lateral_output.lc_status == "left_lane_change" ||
               lateral_output.lc_status == "right_lane_change") {
      msg.decision.lat_decision.lc_back_id = int(1);
    } else if (lateral_output.lc_status == "left_lane_change_back" ||
               lateral_output.lc_status == "right_lane_change_back") {
      msg.decision.lat_decision.lc_back_id = int(2);
    } else
      msg.decision.lat_decision.lc_back_id = int(-1);

    return msg;
  }

  void update_planning_result(const maf_planning::Planning &maf_planning_output,
                              msquare::ddp::PlanningResult &planning_result) {
    planning_result.timestamp =
        maf_planning_output.meta.plan_timestamp_us / 1.0e6;
    planning_result.traj_points.clear();
    int traj_size = std::min(
        std::min(maf_planning_output.trajectory.velocity.vel_points.size(),
                 maf_planning_output.trajectory.path.size()),
        maf_planning_output.trajectory.acceleration.acc_points.size());
    msquare::ddp::TrajectoryPoint point;
    bool match = false;
    for (int i = 0; i < traj_size; i++) {
      point.t =
          maf_planning_output.trajectory.velocity.vel_points[i].relative_time;
      point.v =
          maf_planning_output.trajectory.velocity.vel_points[i].target_velocity;
      point.a = maf_planning_output.trajectory.acceleration.acc_points[i].acc;
      point.x = maf_planning_output.trajectory.path[i].position_enu.x;
      point.y = maf_planning_output.trajectory.path[i].position_enu.y;
      point.heading_angle = maf_planning_output.trajectory.path[i].heading_yaw;
      planning_result.traj_points.push_back(point);
      const auto &ddp_traj = ddp::DdpContext::Instance()
                                 ->model_result_manager()
                                 ->get_current_lane_ddp_trajectory();
      if (std::fabs(point.t) < 0.01 && ddp_traj.trajectory.size() > 0 &&
          !match) {
        const auto &ddp_start_point = ddp_traj.trajectory[0];
        double dx = ddp_start_point.x - point.x;
        double dy = ddp_start_point.y - point.y;
        double ds = std::hypot(dx, dy);
        MSD_LOG(INFO, "update_planning_result: ds: %f", ds);
        match = true;
      }
    }
    if (!match) {
      MSD_LOG(INFO, "update_planning_result: not matched!");
    }
  }

  path_planner::PathPlannerInput generate_local_lat_planning_info_output() {
    return PlanningContext::Instance()->path_planner_input();
  }

  maf_std::Header generate_planning_info(uint64_t tick_count) {
    maf_std::Header planning_info{};
    auto info = mjson::Json(mjson::Json::object());
    const auto &planning_result =
        PlanningContext::Instance()->planning_status().planning_result;
    const auto &lateral_output =
        PlanningContext::Instance()->lateral_behavior_planner_output();
    const auto &state_machine_output =
        PlanningContext::Instance()->state_machine_output();
    const auto &map_info = world_model_->get_map_info();
    auto state_machine = scenario_manager_->get_state_machine();
    const auto &lc_back_track = state_machine->lc_back_track();
    auto &lateral_obstacle = world_model_->mutable_lateral_obstacle();
    double ego_vel =
        world_model_->get_cart_ego_state_manager().get_cart_ego_state().ego_vel;

    // lateral_dodge_info
    if (world_model_->is_acc_mode() ||
        !world_model_->get_vehicle_dbw_status()) {
      auto &planning_result_tmp = PlanningContext::Instance()
                                      ->mutable_planning_status()
                                      ->planning_result;
      planning_result_tmp.lane_avd_in_lane_first_dir = -1;
      planning_result_tmp.lane_avd_in_lane_first_id = -1;
      planning_result_tmp.lane_avd_in_lane_first_type = 0;
      planning_result_tmp.ego_faster_truck = 0;
      planning_result_tmp.overlap_lane = 0;
    }

    MSD_LOG(INFO, "zyl_truck dir = %d",
            planning_result.lane_avd_in_lane_first_dir);
    MSD_LOG(INFO, "zyl_truck DLP[%d %d] = %d", planning_result.ego_faster_truck,
            planning_result.overlap_lane);
    if (planning_result.lane_avd_in_lane_first_dir == 0 ||
        planning_result.lane_avd_in_lane_first_dir == 1) {
      auto avd_in_lane_first_info = mjson::Json(mjson::Json::object());
      // 0: left, 1: right
      int lane_avd_in_lane_first_dir =
          planning_result.lane_avd_in_lane_first_dir;
      int lane_avd_in_lane_first_id = planning_result.lane_avd_in_lane_first_id;
      // 0: vru, 1: vehicle
      int lane_avd_in_lane_first_type =
          planning_result.lane_avd_in_lane_first_type;
      avd_in_lane_first_info["direction"] =
          mjson::Json(lane_avd_in_lane_first_dir);
      avd_in_lane_first_info["object_id"] =
          mjson::Json(inverse_hash_prediction_id(lane_avd_in_lane_first_id));
      avd_in_lane_first_info["object_type"] =
          mjson::Json(lane_avd_in_lane_first_type);
      info["lateral_dodge_info"] = avd_in_lane_first_info;

      if (planning_result.ego_faster_truck == 1 &&
          planning_result.overlap_lane == 1) {
        MSD_LOG(INFO, "zyl_truck print to topic");
        auto object_overlap_slower = mjson::Json(mjson::Json::object());
        object_overlap_slower["truck_overlap_slower"] = mjson::Json(1);
        info["lateral_dodge_DLP"] = object_overlap_slower;
      }
    }

    bool has_leadone = false;
    double leadone_vel = DBL_MAX;
    double leadone_a = DBL_MAX;
    double leadone_relative_distance = DBL_MAX;
    // cutin and leadone info
    auto &lon_follow_obstacles = planning_result.lon_follow_obstacles;
    if ((lateral_output.lc_request == "none") &&
        (lon_follow_obstacles.size() > 0)) {
      auto target_baseline =
          world_model_->get_baseline_info(PlanningContext::Instance()
                                              ->planning_status()
                                              .lane_status.target_lane_id);
      if (target_baseline != nullptr) {
        auto &obstacles = target_baseline->obstacle_manager().get_obstacles();
        auto &ego_state = target_baseline->get_ego_state();
        double half_width =
            ConfigurationContext::Instance()->get_vehicle_param().width / 2 -
            0.1;
        double closest_cutin_s = DBL_MAX;
        double cloest_lead_s = DBL_MAX;
        std::vector<int> cutin_id{};
        for (auto id : lon_follow_obstacles) {
          int object_id = inverse_hash_prediction_id(id);
          const Obstacle *obstacle = obstacles.Find(object_id);
          if (obstacle != nullptr) {
            const SLBoundary &obs_sl = obstacle->PerceptionSLBoundary();
            double lat_speed =
                obstacle->speed() * std::sin(obstacle->Yaw_relative_frenet());
            // 0: vru, 1: vehicle
            int object_type = 0;
            if (obstacle->Type() == ObjectType::PEDESTRIAN ||
                obstacle->Type() == ObjectType::OFO) {
              object_type = 0;
            } else {
              object_type = 1;
            }

            // cuntin info
            static constexpr double theta_merge = 3.14 / 9.0;
            // lateral not overlap and has lat speed
            if ((obs_sl.start_l > ego_state.ego_frenet.y + half_width &&
                 lat_speed < -0.2) ||
                (obs_sl.end_l < ego_state.ego_frenet.y - half_width &&
                 lat_speed > 0.2)) {
              // yaw is statisfied and choose lon closest cutin car
              if (std::fabs(obstacle->Yaw_relative_frenet()) < theta_merge &&
                  obs_sl.start_s < closest_cutin_s) {
                closest_cutin_s = obs_sl.start_s;
                cutin_id.push_back(object_id);

                auto cutin_info = mjson::Json(mjson::Json::object());
                cutin_info["object_id"] = mjson::Json(object_id);
                cutin_info["object_type"] = mjson::Json(object_type);
                info["cutin_info"] = cutin_info;
              }
            }
          }
        }
        for (auto id : lon_follow_obstacles) {
          int object_id = inverse_hash_prediction_id(id);
          const Obstacle *obstacle = obstacles.Find(object_id);
          if (obstacle != nullptr) {
            const SLBoundary &obs_sl = obstacle->PerceptionSLBoundary();
            // 0: vru, 1: vehicle
            int object_type = 0;
            if (obstacle->Type() == ObjectType::PEDESTRIAN ||
                obstacle->Type() == ObjectType::OFO) {
              object_type = 0;
            } else {
              object_type = 1;
            }

            // leadone info
            static int last_object_id = -10;
            static double lead_time = 0.0;
            // choose lon closest not cutin car with overlap
            if (obs_sl.start_s < cloest_lead_s &&
                ((obs_sl.start_l < ego_state.ego_frenet.y + half_width) &&
                 (obs_sl.end_l > ego_state.ego_frenet.y - half_width)) &&
                std::find(cutin_id.begin(), cutin_id.end(), object_id) ==
                    cutin_id.end()) {
              cloest_lead_s = obs_sl.start_s;
              if (last_object_id == object_id) {
                lead_time += 0.1;
              } else {
                lead_time = 0.0;
              }
              last_object_id = object_id;

              auto leadone_info = mjson::Json(mjson::Json::object());
              leadone_info["object_id"] = mjson::Json(object_id);
              leadone_info["object_type"] = mjson::Json(object_type);
              leadone_info["duration"] = mjson::Json((int)lead_time);
              info["leadone_info"] = leadone_info;

              // for imcu
              auto imcu_obj_cmd_info = mjson::Json(mjson::Json::object());
              int obj_type = 0;
              if (obstacle->Type() == ObjectType::COUPE) {
                obj_type = 1;
              } else if (obstacle->Type() == ObjectType::TRANSPORT_TRUNK ||
                         obstacle->Type() == ObjectType::BUS ||
                         obstacle->Type() == ObjectType::ENGINEER_TRUCK) {
                obj_type = 2;
              } else if (obstacle->Type() == ObjectType::OFO) {
                obj_type = 4;
              }
              static int obj_rolling_counter = 0;
              if (last_object_id == object_id) {
                obj_rolling_counter += 1;
              } else {
                obj_rolling_counter = 0;
              }
              double obj_longitude_relative_speed = obstacle->speed() - ego_vel;
              double obj_longitude_relative_distance =
                  obs_sl.start_s - ego_state.ego_frenet.x -
                  ConfigurationContext::Instance()->get_vehicle_param().length /
                      2;
              double obj_lattitude_relative_distance =
                  (obs_sl.start_l + obs_sl.end_l) / 2.0 -
                  ego_state.ego_frenet.y;
              imcu_obj_cmd_info["obj_id"] = mjson::Json(object_id);
              imcu_obj_cmd_info["obj_type"] = mjson::Json(obj_type);
              imcu_obj_cmd_info["obj_rolling_counter"] =
                  mjson::Json(obj_rolling_counter);
              imcu_obj_cmd_info["obj_longitude_relative_speed"] =
                  mjson::Json(obj_longitude_relative_speed);
              imcu_obj_cmd_info["obj_longitude_relative_distance"] =
                  mjson::Json(obj_longitude_relative_distance);
              imcu_obj_cmd_info["obj_lattitude_relative_distance"] =
                  mjson::Json(obj_lattitude_relative_distance);
              imcu_obj_cmd_info["obj_exsist_probability"] = mjson::Json(1.0);
              imcu_obj_cmd_info["obs_exsist_probability"] = mjson::Json(0.0);
              info["imcu_obj_cmd"] = imcu_obj_cmd_info;

              has_leadone = true;
              leadone_vel = obstacle->speed();
              leadone_a = obstacle->acceleration();
              leadone_relative_distance = obj_longitude_relative_distance;
            }
          }
        }
      }
    }

    auto apf_attract_type = world_model_->get_refline_attract_type();
    if (apf_attract_type.attract_type ==
        pass_intersection_planner::ApfAttractType::Traj) {
      int follow_flag = 1;
      auto pi_follow_car = mjson::Json(mjson::Json::object());
      pi_follow_car["follow_flag"] = mjson::Json(follow_flag);
      info["pass_intersection_info"] = pi_follow_car;
    }
    auto apf_lat_far = world_model_->get_apf_lat_far_flag(); // true, false
    if (apf_lat_far) {
      int pi_lat_far_flag = 1;
      auto pi_lat_far = mjson::Json(mjson::Json::object());
      pi_lat_far["pi_lat_far_flag"] = mjson::Json(pi_lat_far_flag);
      info["pass_intersection_info"] = pi_lat_far;
      MSD_LOG(
          INFO,
          "refline_generator cond judge quit cp: PI lat far and quit cp !!!");
    }

    // lane_change_info
    static mjson::Json last_lane_change_info{};
    static int last_status = -1;
    static int last_fail_reason = -1;
    static int fail_reason_dur_count = 0;
    int status = -1;
    int new_fail_reason = -1; // hack for mff :3 times
    // to match L state machine
    if (lateral_output.lc_request == "left" ||
        lateral_output.lc_request == "right" ||
        (lateral_output.planner_action &
         (AlgorithmAction::LANE_CHANGE_LEFT |
          AlgorithmAction::LANE_CHANGE_RIGHT))) {
      //  5 ready
      status = 5;
      switch (lateral_output.planner_status) {
      case AlgorithmStatus::LANE_CHANGE_WAITING:
        status = 0;
        break;
      case AlgorithmStatus::LANE_CHANGEING:
        status = 1;
        break;
      case AlgorithmStatus::LANE_CHANGE_BACK:
        status = 2;
        break;
      }
    }
    // L fsm
    // 0: wait(all unsuitable), 1: changing, 2: back, 3: cancel, 4: done , 5
    // ready
    if (status >= 0) {
      // 0: map, 1: act, 2: int
      int type = 0;
      if (lateral_output.lc_request_source == "map_request") {
        type = 0;
      } else if (lateral_output.lc_request_source == "act_request") {
        type = 1;
      } else if (lateral_output.lc_request_source == "int_request") {
        type = 2;
      }

      // 0: left, 1: right
      int direction = (lateral_output.lc_request == "left") ? 0 : 1;

      // 0: in ramp, 1: out ramp, 2: slow car, 3: avd merge, 4: avd bus lane,
      // 5: lane end, 6: cone bucket, 7: navi
      int trigger_reason = 7;
      if (lateral_output.lc_request_source == "act_request") {
        if (lateral_output.act_request_source == "act") {
          int track_id = -1;

          TrackedObject alc_car{};
          if (lateral_obstacle.find_track(track_id, alc_car)) {
            int type = alc_car.type;
            if (type == MSD_OBJECT_TYPE_CONE_BUCKET) {
              trigger_reason = 6;
            } else {
              trigger_reason = 2;
            }
          } else {
            trigger_reason = 2;
          }
        } else if (lateral_output.act_request_source.find("avd") !=
                   string::npos) {
          trigger_reason = 3;
        }
      } else if (lateral_output.lc_request_source == "map_request") {
        if (map_info.is_on_highway()) {
          if (map_info.dis_to_ramp() < 1000.0 &&
              std::abs(map_info.lc_map_decision() == 1)) {
            if (direction == 0) {
              trigger_reason = 1;
            } else {
              trigger_reason = 0;
            }
          } else if (map_info.lanes_merge_type() == MergeType::LANE_END) {
            trigger_reason = 5;
          }
        }
      }

      // 0: unsuitable(object or env), 1: solid line 2: timeout
      int fail_reason = -1;
      int condition = 0;
      int condition_detail = 0;
      int object_id = 0;
      // 0: vru, 1: vehicle
      int object_type = 0;
      if (lateral_output.planner_status == AlgorithmStatus::LANE_CHANGE_BACK) {
        object_id = lc_back_track.track_id;
        TrackedObject fail_car{};
        if (lateral_obstacle.find_track(object_id, fail_car)) {
          int type = fail_car.type;
          if (type == MSD_OBJECT_TYPE_PEDESTRIAN ||
              type == MSD_OBJECT_TYPE_OFO) {
            object_type = 0;
          } else {
            object_type = 1;
          }
        }
      }

      if (state_machine_output.int_cancel_reason_fsm == SOLID_LC)
        fail_reason = 1;
      if (state_machine_output.int_cancel_reason_fsm == TIMEOUT_LC)
        fail_reason = 2;
      if (state_machine_output.int_cancel_reason_fsm == ENV_ERROR ||
          state_machine_output.int_cancel_reason_fsm == UNSUITABLE_VEL)
        fail_reason = 0;

      // 0: left, 1: right
      // condition 0 : nodisplay 1 :satisfied left  2: notsatisfied left  3:
      // satisfied right  4: notsatisfied right
      if (status == 0 || status == 2 || fail_reason >= 0) {
        condition = (direction == 0) ? 2 : 4;
      } else {
        if (fail_reason < 0)
          condition = (direction == 0) ? 1 : 3;
      }
      // 0: left, 1: right
      if (direction == 0) {
        condition_detail = std::max(
            0, int(state_machine_output.lane_change_condition) * 2 - 1);
      } else {
        condition_detail = int(state_machine_output.lane_change_condition) * 2;
      }
      if (status == 5 && (condition == 2 || condition == 4))
        status = 0;
      auto lane_change_info = mjson::Json(mjson::Json::object());
      lane_change_info["type"] = mjson::Json(type);
      lane_change_info["status"] = mjson::Json(status);
      lane_change_info["direction"] = mjson::Json(direction);
      lane_change_info["condition"] = mjson::Json(condition);
      lane_change_info["condition_detail"] = mjson::Json(condition_detail);
      if (lateral_output.lc_request_source != "int_request") {
        lane_change_info["trigger_reason"] = mjson::Json(trigger_reason);
      }
      if (lateral_output.planner_status == AlgorithmStatus::LANE_CHANGE_BACK &&
          fail_reason >= 0) {
        new_fail_reason = fail_reason;
        lane_change_info["fail_reason"] = mjson::Json(fail_reason);
        lane_change_info["object_id"] =
            mjson::Json(inverse_hash_prediction_id(object_id));
        lane_change_info["object_type"] = mjson::Json(object_type);
      }
      if (fail_reason == 2) {
        lane_change_info["status"] = 3;
        new_fail_reason = fail_reason;
        lane_change_info["fail_reason"] = mjson::Json(fail_reason);
      }
      info["lane_change_info"] = lane_change_info;
      last_lane_change_info = lane_change_info;
      last_status = status;
    } else {
      auto lane_change_info = mjson::Json(mjson::Json::object());
      if (last_status == 1) {
        // processing 2 none
        int fail_reason = -1;
        if (state_machine_output.int_cancel_reason_fsm == SOLID_LC)
          fail_reason = 1;
        if (state_machine_output.int_cancel_reason_fsm == TIMEOUT_LC)
          fail_reason = 2;
        if (state_machine_output.int_cancel_reason_fsm == ENV_ERROR ||
            state_machine_output.int_cancel_reason_fsm == UNSUITABLE_VEL)
          fail_reason = 0;
        // 4 done
        if (fail_reason < 0)
          lane_change_info["status"] = 4;
        else
          lane_change_info["status"] = 3;
        if (fail_reason >= 0) {
          new_fail_reason = fail_reason;
          lane_change_info["fail_reason"] = mjson::Json(fail_reason);
        }
        info["lane_change_info"] = lane_change_info;
      } else if (last_status == 2 || last_status == 0) {
        // back 2 none or waitting 2 none
        // 3 cancel
        int fail_reason = -1;
        if (state_machine_output.int_cancel_reason_fsm == SOLID_LC)
          fail_reason = 1;
        if (state_machine_output.int_cancel_reason_fsm == TIMEOUT_LC)
          fail_reason = 2;
        if (state_machine_output.int_cancel_reason_fsm == ENV_ERROR ||
            state_machine_output.int_cancel_reason_fsm == UNSUITABLE_VEL)
          fail_reason = 0;
        if (fail_reason >= 0) {
          new_fail_reason = fail_reason;
          lane_change_info["fail_reason"] = mjson::Json(fail_reason);
        } else
          lane_change_info["status"] = 3;
        info["lane_change_info"] = lane_change_info;
      }
      last_lane_change_info = lane_change_info;
      last_status = -1;
    }

    // alc recommend dir
    const auto &request_manager_output =
        PlanningContext::Instance()->request_manager_output();
    info["alc_recommend_dir"] =
        mjson::Json(request_manager_output.alc_recommend_dir);

    // hack for mff
    if (new_fail_reason != last_fail_reason && new_fail_reason >= 0) {
      // get new valid fail reason
      fail_reason_dur_count = 0;
      last_fail_reason = new_fail_reason;
    } else {
      // no new valid fail reason
      if (fail_reason_dur_count < 2) {
        auto lane_change_info = mjson::Json(mjson::Json::object());
        lane_change_info["fail_reason"] = mjson::Json(last_fail_reason);
        info["lane_change_info"] = lane_change_info;
        fail_reason_dur_count++;
      } else {
        last_fail_reason = -1;
      }
    }

    // trajectory_info
    auto polynomial_curve = mjson::Json::array(5, mjson::Json(0.0));
    double x_start = 0.0;
    double x_end = 0.0;

    // fit coeffs from traj
    if (!planning_result.traj_pose_array.empty()) {
      static std::vector<double> poly_coeff;
      static std::vector<Pose2D> traj;
      poly_coeff.clear();
      traj.resize(planning_result.traj_pose_array.size());
      Pose2D ego_pose = world_model_->get_cart_ego_state_manager()
                            .get_cart_ego_state()
                            .ego_pose;
      ego_pose.x =
          ego_pose.x -
          (ConfigurationContext::Instance()->get_vehicle_param().length / 2 -
           ConfigurationContext::Instance()
               ->get_vehicle_param()
               .rear_bumper_to_rear_axle) *
              cos(ego_pose.theta);
      ego_pose.y =
          ego_pose.y -
          (ConfigurationContext::Instance()->get_vehicle_param().length / 2 -
           ConfigurationContext::Instance()
               ->get_vehicle_param()
               .rear_bumper_to_rear_axle) *
              sin(ego_pose.theta);
      std::transform(planning_result.traj_pose_array.begin(),
                     planning_result.traj_pose_array.end(), traj.begin(),
                     [&ego_pose](maf_planning::PathPoint path_point) {
                       Pose2D pose;
                       pose.x = path_point.position_enu.x;
                       pose.y = path_point.position_enu.y;
                       pose.theta = path_point.heading_yaw;
                       return planning_math::tf2d(ego_pose, pose);
                     });
      x_end = traj.back().x;
      if (Polyfit(traj, 3, poly_coeff)) {
        poly_coeff[0] = 0.0;
        // use 3rd fit and add two zero for 5rd
        poly_coeff.push_back(0.0);
        poly_coeff.push_back(0.0);
        polynomial_curve.clear();
        for (auto coeff : poly_coeff) {
          polynomial_curve.emplace_back(mjson::Json(coeff));
        }
      }
    }

    auto trajectory_info = mjson::Json(mjson::Json::object());
    trajectory_info["polynomial_curve"] = mjson::Json(polynomial_curve);
    trajectory_info["start_point_x"] = mjson::Json(x_start);
    trajectory_info["end_point_x"] = mjson::Json(x_end);
    info["trajectory_info"] = trajectory_info;

    const auto &speed_planner_input =
        PlanningContext::Instance()->speed_planner_input();
    const bool hold_flag = speed_planner_input.stop_point_info.hold_flag;
    const bool ready_go = speed_planner_input.stop_point_info.ready_go;

    // follow_stop_info
    auto follow_stop_info = mjson::Json(mjson::Json::object());
    follow_stop_info["is_stop"] =
        mjson::Json(speed_planner_input.stop_point_info.hold_flag);
    follow_stop_info["cipv_lost"] =
        mjson::Json(speed_planner_input.stop_point_info.last_CIPV_lost);
    info["follow_stop_info"] = follow_stop_info;

    // follow_start_info
    auto follow_start_info = mjson::Json(mjson::Json::object());
    follow_start_info["is_start"] = mjson::Json(!hold_flag);
    follow_start_info["ready_to_go"] = mjson::Json(!hold_flag || ready_go);
    info["follow_start_info"] = follow_start_info;

    // lane_change_point_info
    static constexpr double kResponseOffset = 300.0;
    std::array<double, 3> xp{40.0 / 3.6, 80.0 / 3.6, 120.0 / 3.6};
    std::array<double, 3> fp{300.0, 500.0, 800.0};
    double v_limit = PlanningContext::Instance()->planning_status().v_limit;
    double adaptor_interval = interp(v_limit, xp, fp);

    mjson::Json::array off_ramp_json_array;
    if ((lateral_output.planner_action & AlgorithmAction::LANE_CHANGE_RIGHT) &&
        (lateral_output.lc_request_source == "map_request") &&
        (map_info.lc_map_decision() > 0)) {
      for (int i = map_info.lc_map_decision(); i > 0; i--) {
        double map_response_dist = kResponseOffset + adaptor_interval * i;

        double start_dist =
            std::max(map_info.lc_end_dis() - map_response_dist, 0.0);
        double end_dist = map_info.lc_end_dis();
        int relative_id = i - map_info.lc_map_decision();

        mjson::Json::object off_ramp_info;
        off_ramp_info["earliest_lane_change_point_x"] = mjson::Json(start_dist);
        off_ramp_info["latest_lane_change_point_x"] = mjson::Json(end_dist);
        off_ramp_info["relative_lane_id"] = mjson::Json(relative_id);
        off_ramp_json_array.push_back(mjson::Json(off_ramp_info));
      }
      info["lane_change_point_info"] = off_ramp_json_array;
    }

    mjson::Json::object curv_info;
    curv_info["v_curv"] = mjson::Json(world_model_->get_v_curv());
    curv_info["a_curv"] = mjson::Json(world_model_->get_a_curv());
    curv_info["max_curv"] = mjson::Json(world_model_->get_max_curv());
    curv_info["max_curv_distance"] =
        mjson::Json(world_model_->get_max_curv_distance());
    info["curv_info"] = curv_info;

    bool brake_to_stop = false;
    bool brake_prefer = false;
    if (world_model_->is_acc_mode() && has_leadone) {
      double desired_stance =
          leadone_vel *
              speed_planner::NORMAL_HEADWAY_TABLE[world_model_
                                                      ->get_navi_ttc_gear()] +
          4.0;
      if (ego_vel < 0.7 && leadone_vel < 0.4 &&
          (leadone_relative_distance - desired_stance) < 0.8) {
        brake_to_stop = true;
      }

      if (brake_to_stop ||
          (ego_vel < 5.5 && leadone_vel < 5.0 && leadone_a < 0.0 &&
           (leadone_relative_distance - desired_stance) < 0.0)) {
        brake_prefer = true;
      }
    }

    mjson::Json::object brake_info;
    brake_info["brake_to_stop"] = mjson::Json(brake_to_stop);
    brake_info["brake_prefer"] = mjson::Json(brake_prefer);
    info["brake_info"] = brake_info;

    int cone_bucket_warn_level =
        speed_planner_input.cone_bucket_info.warning_level;
    mjson::Json::object cone_bucket_stop_info;
    cone_bucket_stop_info["cone_bucket_warn_level"] =
        mjson::Json(cone_bucket_warn_level);
    info["cone_bucket_stop_info"] = cone_bucket_stop_info;

    mjson::Json::object longitudinal_planning_info;
    double plan_acc = 0.;
    if (planning_result.traj_acceleration.size() >= 2) {
      plan_acc = planning_result.traj_acceleration[1];
    }
    longitudinal_planning_info["plan_acc"] = mjson::Json(plan_acc);
    info["longitudinal_planning_info"] = longitudinal_planning_info;

    auto baselin_info =
        world_model_->get_baseline_info(PlanningContext::Instance()
                                            ->planning_status()
                                            .lane_status.target_lane_id);

    hmi_manager_->Update(world_model_, baselin_info);
    bool need_acc_takeover =
        speed_planner_input.acc_takeover_info.need_acc_takeover;
    mjson::Json::object acc_safety_info;
    acc_safety_info["need_takeover"] = mjson::Json(need_acc_takeover);
    info["acc_safety_info"] = acc_safety_info;

    // cipv_lost
    mjson::Json::object cipv_lost_info;
    mjson::Json::object fault_diagnosis_info;
    if (speed_planner_input.cipv_lost_info.warning_level == 2) {
      fault_diagnosis_info["has_fault"] = mjson::Json(true);
      fault_diagnosis_info["fault_type"] = mjson::Json(1);
      cipv_lost_info["warning_level"] = mjson::Json(2);
    } else if (speed_planner_input.cipv_lost_info.warning_level == 1) {
      cipv_lost_info["warning_level"] = mjson::Json(1);
    } else {
      cipv_lost_info["warning_level"] = mjson::Json(0);
    }
    info["fault_diagnosis_info"] = fault_diagnosis_info;
    info["cipv_lost_info"] = cipv_lost_info;

    planning_info.stamp = MTIME()->timestamp().ns();
    planning_info.seq = tick_count;
    planning_info.frame_id = info.dump();

    return planning_info;
  }

private:
  enum FeedType {
    FEED_VEHICLE_DBW_STATUS = 0,
    FEED_EGO_VEL,
    FEED_EGO_STEER_ANGLE,
    FEED_EGO_ENU,
    FEED_WHEEL_SPEED_REPORT,
    FEED_EGO_ACC,
    FEED_MISC_REPORT,
    FEED_GEAR_REPORT,
    FEED_MAP_INFO,
    FEED_FUSION_INFO,
    FEED_PREDICTION_INFO,
    FEED_VISION_LANE,
    // FEED_LIDAR_ROAD_EDGE,
    // FEED_RADAR,
    FEED_TYPE_MAX,
  };

private:
  // std::shared_ptr<ModuleMonitor> monitor_;
  std::shared_ptr<maf::StatusManager> monitor_;

  std::shared_ptr<WorldModel> world_model_{};
  std::unique_ptr<HmiManager> hmi_manager_;
  std::shared_ptr<ScenarioManager> scenario_manager_;
  ModuleStatus planning_module_status_;
  maf_vehicle_status::VehicleStatus vehicle_status_{};
  maf_vehicle_status::VehicleLight vehicle_light_{};
  maf_system_manager::ModuleControlCmdRequest module_control_cmd_request_{};
  maf_perception_interface::RoadLinePerception perception_vision_lane_{};
  maf_perception_interface::RoadLinePerception perception_lidar_road_edge_{};
  maf_perception_interface::PerceptionFusionObjectResult
      perception_fusion_object_{};
  maf_perception_interface::RadarPerceptionResult perception_radar_{};

  double last_feed_time_[FEED_TYPE_MAX]{};
  std::string last_feed_msg_id_[FEED_TYPE_MAX]{};

  CircleObjectBuffer<std::shared_ptr<maf_worldmodel::ObjectsInterface>>
      worldmodel_objects_buffer_;

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
  mtaskflow::FlowReceiver<
      std::shared_ptr<maf_perception_interface::PerceptionFusionObjectResult>>
      perception_fusion_object_receiver_;
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
  mtaskflow::FlowReceiver<maf_std::Header> planning_reset_request_receiver_;
  mtaskflow::FlowReceiver<
      std::shared_ptr<maf_perception_interface::RoadLinePerception>>
      perception_vision_lane_receiver_;
  mtaskflow::FlowReceiver<
      std::shared_ptr<maf_perception_interface::RoadLinePerception>>
      perception_lidar_road_edge_receiver_;
  mtaskflow::FlowReceiver<
      std::shared_ptr<maf_perception_interface::RadarPerceptionResult>>
      perception_radar_receiver_;
  mtaskflow::FlowReceiver<std::shared_ptr<maf_std::Header>> mff_info_receiver_;

  MSDPlanningOutputCallback planning_output_callback_{};
  MSDPlanningTriggerCallback planning_trigger_callback_{};
  MSDPlanningInfoCallback planning_info_callback_{};
  MSDMdebugCallback mdebug_callback_{};
  NPPHeaderLoggerCallback publish_simulation_sync_{};
  std::function<void()> _runner{};

  bool enable_timer_tick_{true};
  bool reset_{false};
  bool reset_finished_{true};
  uint64_t ego_pose_timestamp_us_ = 0;
  bool ego_position_jerk_large_ = false;
  maf_mla_localization::MLAPosition last_position_{};
  bool is_in_simulation_ = false;

  bool enable_ilc_ = true;
  double cruise_velocity_ = 120.0; // km/h
  uint8_t navi_ttc_gear_ = 2;
  double steer_angle_offset_deg_ = 0.0;
  uint32_t underclocking_enum_{10U};
  uint32_t underclocking_{0U};
  uint32_t underclocking_cnt_{0U};
  uint64_t planning_seq_{0};
  uint64_t localization_seq_{0};

  bool is_hnp_mode_ = false;
  int last_cone_bucket_warn_level_ = 0;
  int cone_bucket_ttc_cnt_ = 0;
};

} // namespace msquare
