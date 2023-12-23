#include "common/apa_workflow/parking_planning_task.hpp"
#include "common/planning_task.hpp"
#include "common/utils/trajectory_point_utils.h"
#include "common/utils/parking_behavior_planner_output.hpp"
#include "common/sbp_strategy.h"
#include "common/search_based_planning_utils.h"
#include "mlog_core/mlog.h"
#include "mlog_msg_id/mlog_msg_id.hpp"
#include "nlohmann/json.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "common/apa_workflow/apa_state_machine_logger.hpp"
#include "planner/behavior_planner/parking/speed_margin_limiter.h"


#ifdef __x86_64__
#include <sys/time.h>
static double now_ms()
{
  struct timeval tv;
  struct timezone tz;
  if(gettimeofday(&tv,&tz) ==0)
  {
    return (double)tv.tv_sec*1000.0 + ((double)tv.tv_usec)*1.0e-3;
  }
  else
  {
    return 0;
  }
}
#endif

namespace msquare {
namespace parking {

bool PlanningTask::is_in_simulation() {
  bool is_in_simulation = false;
  const char *sim = std::getenv("RealitySimulation");
  if (sim != nullptr && std::strcmp(sim, "simulation") == 0) {
    is_in_simulation = true;
  } else {
    is_in_simulation = false;
  }
  return is_in_simulation;
}

void PlanningTask::on_running() {
  _runner();
}

void PlanningTask::_run_in_simulation() {
#ifdef __x86_64__
  double begin_time = now_ms();
#endif
  _run_in_reality();
#ifdef __x86_64__
  double end_time = now_ms();
  MSD_LOG(ERROR, "ParkingPlanningTask run_once cost: %f\n", (end_time-begin_time));
#endif
  if (module_control_cmd_request_.running_mode.value !=
          maf_system_manager::RunningModeEnum::PARKING) {
    MSD_LOG(INFO, "not run parking planning");
    return;
  }
  std::string ss = "";
  ss.append("input seqs, parking planning: ")
      .append(std::to_string(planning_seq_))
      .append(", localization: ")
      .append(std::to_string(localization_seq_));
  MSD_LOG(INFO, "%s", ss.c_str());
  if (publish_simulation_sync_ != nullptr) {
    publish_simulation_sync_(std::string("PLANNING END: ") + ss);
  }
}

void PlanningTask::_run_in_reality() {
  // clear status info in each loop
  *PlanningContext::Instance()->mutable_planning_debug_info() = "";
  *PlanningContext::Instance()->mutable_is_planner_update_plan_path() = false;

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
          maf_system_manager::RunningModeEnum::PARKING ||
      module_control_cmd_request_.module_control_cmd.value ==
          maf_system_manager::ModuleControlCmdEnum::PAUSE) {
    MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
    clear_all_recievers();
    reset();
    return;
  }

  maf_system_manager::RunningModeEnum mode;
  mode.value = maf_system_manager::RunningModeEnum::PARKING;
  monitor_->set_node_status_running_mode(mode);

  auto &scenario = PlanningContext::Instance()->planning_status().scenario;
  if (scenario.status_type == StatusType::WAIT &&
      scenario.next_status_type == StatusType::WAIT) {
    reset_ = false;
  }

  const int SBP_REQUEST_CALLBACK_PERIOD = 10;
  auto &odo = PlanningContext::Instance()->openspace_decider_output();
  OpenspaceDeciderOutput search_problem;
  if (odo.is_problem_ready) {
    search_problem = extractSearchProblem(StrategyParams::GetInstance(), odo);
    PlanningContext::Instance()
        ->mutable_openspace_decider_output()
        ->is_problem_ready = false;
    sbp_problem_publisher_->publish(search_problem);
    // also send request for debug
    if (sbp_request_cb_) {
      sbp_request_cb_(gen_sbp_request(search_problem, "search_problem"));
      // debug info
      *PlanningContext::Instance()->mutable_planning_debug_info() +=
          "[pp]path planning request! ";
      (*PlanningContext::Instance()->mutable_sbp_request_count())++;
    }
    sbp_debug_info_ += "sbp request publish! ";
  }

  bool is_odo_valid = std::abs(odo.map_boundary.length()) > 1e-6 &&
                      std::abs(odo.map_boundary.width()) > 1e-6;
  if (is_odo_valid && scenario.status_type == StatusType::APA &&
      tick_count % SBP_REQUEST_CALLBACK_PERIOD == 0) {
    OpenspaceDeciderOutput odo_modify(odo);
    const maf_vehicle_status::GearData &gear_data =
        world_model_->get_gear_report();
    odo_modify.init_state.v =
        gear_data.gear_status.value == maf_vehicle_status::GearType::REVERSE
            ? -1
            : 1;
    search_problem =
        extractSearchProblem(StrategyParams::GetInstance(), odo_modify);
    if (sbp_request_cb_) {
      sbp_request_cb_(gen_sbp_request(search_problem, "debug"));
    }
  }
  fill_openspace_motion_planner_output(
      odo,
      PlanningContext::Instance()->mutable_openspace_motion_planner_output());
  
  WlcInfo &wlc_info_ref = PlanningContext::Instance()->mutable_planning_status()->wlc_info;
  auto &wireless_charger_report_data = world_model_->get_wireless_charger_report_data();
  wlc_info_ref.x_offset = (double)wireless_charger_report_data.offset_x / 1000.0;
  wlc_info_ref.y_offset = (double)wireless_charger_report_data.offset_y / 1000.0;
  wlc_info_ref.is_valid = wireless_charger_report_data.detected_state.value
    == maf_endpoint::WrlsChrgDetResult::WIRELESS_CHARGER_DETECED_RESULT_DEFAULT;
  wlc_info_ref.is_valid = wlc_info_ref.is_valid || wireless_charger_report_data.detected_state.value
    == maf_endpoint::WrlsChrgDetResult::WIRELESS_CHARGER_DETECED_RESULT_OK;
  wlc_info_ref.is_valid = wlc_info_ref.is_valid || wireless_charger_report_data.detected_state.value
    == maf_endpoint::WrlsChrgDetResult::WIRELESS_CHARGER_DETECED_RESULT_OFFSET_X_NOT_OK;
  wlc_info_ref.is_valid = wlc_info_ref.is_valid || wireless_charger_report_data.detected_state.value
    == maf_endpoint::WrlsChrgDetResult::WIRELESS_CHARGER_DETECED_RESULT_OFFSET_Y_NOT_OK;
  wlc_info_ref.is_valid = wlc_info_ref.is_valid || wireless_charger_report_data.detected_state.value
    == maf_endpoint::WrlsChrgDetResult::WIRELESS_CHARGER_DETECED_RESULT_OFFSET_XY_NOT_OK;
  const std::vector<ParkingLotDetectionInfo> &parking_lots_detection_fusion_results =
      world_model_->get_parking_map_info().parking_lots_detection_fusion_results;
  
  MSD_LOG(ERROR, "[wlc.if@%s,%d]!parking_lots_detection_fusion_results.empty()(%d) AND !PlanningContext::Instance()\
    ->mutable_parking_behavior_planner_output()->is_request_to_ego_slot(%d)",
    __FUNCTION__,
    __LINE__,
    (int)!parking_lots_detection_fusion_results.empty(),
    (int)!PlanningContext::Instance()->mutable_parking_behavior_planner_output()->is_request_to_ego_slot
    );
  if (!parking_lots_detection_fusion_results.empty()) {
    MSD_LOG(ERROR, "[wlc.enter_if@%s,%d]", __FUNCTION__, __LINE__);
    int target_id = world_model_->get_planning_request().id;
    // Select parking spot from accurate search result
    for (auto &parking_lot : parking_lots_detection_fusion_results) {
      if (parking_lot.id == target_id) {
        // printf("wlc charge_property updated for slot: %d\n", target_id);
        wlc_info_ref.is_fusion_wlc_property_valid = parking_lot.charge_property != 0;
        MSD_LOG(ERROR, "[wlc.operator@%s,%d] wlc_info_ref.is_fusion_wlc_property_valid(%d)\
        = parking_lot.charge_property(%d) != 0;",
        __FUNCTION__, __LINE__,
        (int)wlc_info_ref.is_fusion_wlc_property_valid, (int)parking_lot.charge_property);
        wlc_info_ref.is_fusion_wlc_property_valid = parking_lot.charge_property != 0;
      } else {
        // printf("wlc charge_property: %d, %d\n", parking_lot.id, target_id);
      }
    }
  } else {
    MSD_LOG(ERROR, "[wlc.exit_if@%s,%d]", __FUNCTION__, __LINE__);
  }
  if (!PlanningContext::Instance()->mutable_parking_behavior_planner_output()->is_request_to_ego_slot) {
    wlc_info_ref.is_valid = wlc_info_ref.is_valid && wlc_info_ref.is_fusion_wlc_property_valid;
  }
  MSD_LOG(INFO, "%s: wlc_info_ref.is_valid = %d\n", __FUNCTION__, wlc_info_ref.is_valid);
  
  double start_time = MTIME()->timestamp().ms();
  *PlanningContext::Instance()->mutable_planning_start_time_ms() = start_time;
  MSD_LOG(INFO, "parking (%s)tick_count: %lu", __FUNCTION__, tick_count);

  bool succeed = run_once();

  if (nullptr != planning_output_callback_) {
    const auto &planning_status =
        PlanningContext::Instance()->planning_status();

    maf_planning::Planning maf_planning_output;
    if (msquare::CarParams::GetInstance()->car_config.lon_config.use_sop_algorithm) {
      maf_planning_output = generate_planning_output_sop();
    } else {
      maf_planning_output = generate_planning_output();
    }
    if (is_in_simulation_ && succeed) {
      maf_planning_output.header.seq = ++planning_seq_;
    }
    planning_output_callback_({tick_count, succeed}, maf_planning_output,
                              planning_status.trigger_msg_id);
  }

  if (nullptr != planning_info_callback_) {
    auto maf_planning_info = generate_planning_info(tick_count);

    planning_info_callback_(maf_planning_info);
  }

  double end_time = MTIME()->timestamp().ms();
  MSD_LOG(ERROR, "time_cost, total time: %f", end_time - start_time);
}

maf_planning::SBPRequest
PlanningTask::gen_sbp_request(const OpenspaceDeciderOutput &problem,
                              std::string problem_id) {
  maf_planning::SBPRequest request;
  auto timestamp = MTIME()->timestamp("publish");
  request.header.stamp = timestamp.ns();
  request.header.frame_id = "map";
  request.meta.timestamp_us = ego_pose_timestamp_us_;

  request.task_info.available |= maf_planning::TaskInfo::REQUEST;
  request.task_info.request.value = maf_planning::RequestEnum::START_TASK;
  request.task_info.available |= maf_planning::TaskInfo::GOAL_ID;
  request.task_info.goal_id.id = problem_id;
  request.task_config.available |= maf_planning::TaskConfig::CONFIG;
  {
    auto &problem_config = request.task_config.problem_config;

    problem_config.available |= maf_planning::ProblemConfig::PARAMS_STRING;
    nlohmann::json param_json_obj;
    param_json_obj["HybridAstarConfig"] = *HybridAstarConfig::GetInstance();
    param_json_obj["VehicleParam"] = *VehicleParam::Instance();
    param_json_obj["CarParams"] = *CarParams::GetInstance();
    param_json_obj["StrategyParams"] = *StrategyParams::GetInstance();
    param_json_obj["TrajectoryOptimizerConfig"] =
        *TrajectoryOptimizerConfig::GetInstance();
    param_json_obj["OpenspaceDeciderOutput"] = problem;
    param_json_obj["ParkoutImprove1121"] = "ParkoutImprove1130";
    problem_config.params_string = param_json_obj.dump();
  }
  return request;
}

bool PlanningTask::run_once() {
  std::string now_time_str = "Now: " + std::to_string(MTIME()->timestamp().sec()).substr(6, 7);
  *PlanningContext::Instance()->mutable_now_time_seq_str() =
      std::to_string(MTIME()->timestamp().sec()).substr(6, 7);
  *PlanningContext::Instance()->mutable_planning_debug_info() += now_time_str;
  *PlanningContext::Instance()->mutable_planning_debug_info() += ", sbp_req" +
  std::to_string(PlanningContext::Instance()->sbp_request_count()) + ", ";
  MSD_LOG(ERROR, "%s\n", now_time_str.c_str());

  MLOG_PROFILING("run_once");
  // auto pre_planning_status =
  //     msquare::PlanningContext::Instance()->mutable_prev_planning_status();
  // *pre_planning_status =
  //     msquare::PlanningContext::Instance()->planning_status();
  auto *planning_status =
      PlanningContext::Instance()->mutable_planning_status();
  // planning_status->pre_planning_result = planning_status->planning_result;
  auto start_timestamp = MTIME()->timestamp("run-start");
  // msquare::PlanningContext::Instance()
  //     ->mutable_planning_status()
  //     ->planning_result.next_timestamp = start_timestamp;

  auto trigger_msg_id = mlog::MLOG_msg_id(start_timestamp.ns(), kTriggerTag);
  msd_planning::MSDPlanning_record_timestamp(trigger_msg_id,
                                             MSDPlanning_msg_tag(kPlanTag), 0);
  msd_planning::MSDPlanning_record_timestamp(
      trigger_msg_id, "pnc_plan_timestamp-run-start", start_timestamp.ns());

  // update world module from msg
  double current_time = start_timestamp.sec();
  update_world_model(current_time);

  std::string warn_msg{}, error_msg{};
  if (!can_run(current_time, warn_msg, error_msg)) {
    MSD_LOG(ERROR, "(%s)can not run", __FUNCTION__);

    error_msg = "planning can not run, missing inputs: " + error_msg;
    // monitor_->update_node_status(NodeStatusEnum::RUNNING_ERROR, error_msg);
    // monitor_->update_module_status(ModuleStatus::RUNNING_ERROR,
    //                                ModuleStatus::RUNNING_MISSING_INPUTS,
    //                                status_msg);

    // return false;
  } else {
    if (warn_msg.size() > 0) {
      warn_msg = "planning running warnning, inputs delay: " + warn_msg;
      //   monitor_->update_node_status(NodeStatusEnum::RUNNING_WARNING,
      //   warn_msg);
      // } else {
      //   monitor_->update_node_status(NodeStatusEnum::RUNNING, "");
    }
  }

  double before_world_model_update_time = MTIME()->timestamp().ms();

  MD_CP_BEGIN(CP_APA_PLANNING1);
  bool ret = world_model_->update();
  MD_CP_END(CP_APA_PLANNING1);
  if (!ret) {
    MSD_LOG(ERROR, "(%s)world model update error", __FUNCTION__);

    error_msg = "planning world model update error." + error_msg;
    // monitor_->update_node_status(NodeStatusEnum::RUNNING_ERROR, error_msg);
    // monitor_->update_module_status(ModuleStatus::RUNNING_ERROR,
    //                                ModuleStatus::RUNNING_OUT_OF_MAP,
    //                                status_msg);

    // return false;
  }

  planning_status->trigger_msg_id = trigger_msg_id;

  if (apa_state_machine_->update()) {
    planning_status->planning_success = true;

#ifndef __QNX__
    ApaStateMachineLogger::log_debug_info();
#endif

    error_msg = "planning execute succeed." + error_msg;
    // monitor_->update_node_status(NodeStatusEnum::RUNNING, error_msg);
    // monitor_->update_module_status(ModuleStatus::RUNNING, ModuleStatus::NONE,
    //                                status_msg);
  } else {
    planning_status->planning_success = false;

    MSD_LOG(ERROR, "(%s)execute planning error", __FUNCTION__);

    error_msg = "planning execute failed." + error_msg;
    // monitor_->update_node_status(NodeStatusEnum::RUNNING_ERROR, error_msg);
    // monitor_->update_module_status(ModuleStatus::RUNNING_ERROR,
    //                                ModuleStatus::NONE, status_msg);
  }

  uint16_t task_type;
  auto &task_status =
      PlanningContext::Instance()->planning_status().task_status;

  switch (task_status.task) {
  case StatusType::SEARCH:
  case StatusType::AVP:
    task_type = (uint16_t)node_status::TaskType::NODE_PLANNING_TASK_APA_SEARCH;
    break;
  case StatusType::APA:
    task_type = (uint16_t)node_status::TaskType::NODE_PLANNING_TASK_APA_APA;
    break;
  case StatusType::APOA:
    task_type = (uint16_t)node_status::TaskType::NODE_PLANNING_TASK_APA_APOA;
    break;
  case StatusType::WAIT:
    task_type = (uint16_t)node_status::TaskType::NODE_PLANNING_TASK_APA_WAIT;
    break;
  case StatusType::RPA_STRAIGHT_STANDBY: // send wait when rpa straight standby
    task_type = (uint16_t)node_status::TaskType::NODE_PLANNING_TASK_APA_WAIT;
    break;
  case StatusType::RPA_STRAIGHT:
    task_type = 4100; // NODE_PLANNING_TASK_RPA_STRAIGHT
    break;
  default:
    task_type = (uint16_t)node_status::TaskType::NODE_PLANNING_TASK_APA_WAIT;
    break;
  }

  if (!world_model_->get_pause_status()) {
    switch (task_status.status) {
    case TaskStatusType::SUCCEEDED:
      if ((world_model_->get_planning_request().cmd.value ==
              ParkingCommand::STOP ||
          world_model_->get_steering_reset()) &&
          world_model_->get_gear_report().gear_status.value ==
              maf_vehicle_status::GearType::PARK) {
        (void)monitor_->try_change_status(
            node_status::Status::COMPLETE_SUCCEEDED);
      } else if (task_status.task == StatusType::RPA_STRAIGHT_STANDBY) {
        (void)monitor_->try_change_status(
            node_status::Status(9)); // RPA_STRAIGHT_SUCCESSED
      } else {
        (void)monitor_->try_change_status(node_status::Status::RUNNING);
      }
      monitor_->set_node_status_message(error_msg);
      monitor_->set_node_status_task_type(task_type);
      break;
    case TaskStatusType::FAILED:
      (void)monitor_->try_change_status(node_status::Status::COMPLETE_FAILED);
      monitor_->set_node_status_message(error_msg);
      monitor_->set_node_status_task_type(task_type);
      break;
    case TaskStatusType::RUNNING:
    default:
      (void)monitor_->try_change_status(node_status::Status::RUNNING);
      monitor_->set_node_status_message(error_msg);
      monitor_->set_node_status_task_type(task_type);
      break;
    }

    if (task_status.status == TaskStatusType::FAILED) {
      switch (task_status.failure_reason) {
      case FailureReason::PLANNING_FAILED:
        monitor_->set_node_status_detail_status(
            (uint16_t)node_status::DetailStatus::PLANNING_PLAN_FAILED);
        break;
      case FailureReason::BARRIER_IN_PARKING_SLOT:
        monitor_->set_node_status_detail_status(
            (uint16_t)
                node_status::DetailStatus::PLANNING_BARRIER_IN_PARKING_SLOT);
        break;
      case FailureReason::ADJUEST_TOO_MANY:
        monitor_->set_node_status_detail_status(
            (uint16_t)node_status::DetailStatus::PLANNING_ADJUEST_TOO_MANY);
        break;
      case FailureReason::START_INFEASIBLE:
        monitor_->set_node_status_detail_status(0X1008);
        break;
      case FailureReason::RPA_COMPLETE_FAILED:
        monitor_->set_node_status_detail_status(520);
        break;
      case FailureReason::UNKNOW_FAILED:
      default:
        monitor_->set_node_status_detail_status(
            (uint16_t)node_status::DetailStatus::PLANNING_UNKNOW_FAILED);
        break;
      }
    } else if (PlanningContext::Instance()
                 ->parking_behavior_planner_output()
                 .is_move_ready &&
             (task_status.task != StatusType::WAIT &&
              task_status.task != StatusType::RPA_STRAIGHT_STANDBY)) {
    monitor_->set_node_status_detail_status(
        (uint16_t)node_status::DetailStatus::PLANNING_OK);
    } else {
      monitor_->set_node_status_detail_status(0);
    }
  } else {
    (void)monitor_->try_change_status(node_status::Status::RUNNING);
    monitor_->set_node_status_message(error_msg);
    monitor_->set_node_status_task_type(task_type);
  }

  auto end_timestamp = MTIME()->timestamp();

  // planning_status->time_consumption = (end_timestamp -
  // start_timestamp).sec(); planning_status->planning_result.timestamp =
  //     planning_status->planning_result.next_timestamp;

  return planning_status->planning_success;
}

void PlanningTask::fill_openspace_motion_planner_output(
    const OpenspaceDeciderOutput &problem, OpenspaceMotionPlannerOutput *ompo) {
  ompo->is_fail = false;
  ompo->is_plan_ready = false;
  while (!sbp_result_receiver_->empty()) {
    sbp_debug_info_ += "sbp result received! ";
    SbpResult sbp_result{};
    auto ret = sbp_result_receiver_->pop_oldest(sbp_result);
    if (!ret) {
      continue;
    }
    bool sbp_available = (!sbp_result.x.empty());
    ompo->status =
        (OpenspaceMotionPlannerOutput::PlannerStatus)(int)sbp_result.status;

    nlohmann::json debug_json_obj =
        nlohmann::json::parse(sbp_result.debug_string);
    ompo->planner_calc_duration = debug_json_obj["calc_duration"];
    if (!sbp_available) {
      ompo->is_fail = true;
      MSD_LOG(WARN, "sbp result is not available: %d!");
    } else {
      ompo->is_fail = false;
      ompo->is_plan_ready = true;
      (void)assembleSearchProblem(StrategyParams::GetInstance(), sbp_result);
      if (!GetTemporalProfile(&sbp_result)) {
        // std::cout << "GetSpeedProfile from Hybrid Astar path fails"
        //           << std::endl;
      }
      ompo->traj = convertSbpResult2Traj(sbp_result);
      regulateVelocity(StrategyParams::GetInstance(), ompo->traj);
      *PlanningContext::Instance()->mutable_is_planner_update_plan_path() = true;
    }
  }
}

void PlanningTask::update_worldmodel_info(double current_time) {
  // fetch worldmodel_map
  maf_worldmodel::ProcessedMapData processed_map_data{};
  while (!worldmodel_map_receiver_->empty()) {
    std::shared_ptr<maf_worldmodel::ProcessedMap> received_worldmodel_map{};
    auto ret = worldmodel_map_receiver_->pop_oldest(received_worldmodel_map);
    if (!ret) {
      continue;
    }

    //   if (received_worldmodel_map->processed_map_data.available &
    //       ProcessedMapData::TARGET_POTISION) {
    //     processed_map_data.available |= ProcessedMapData::TARGET_POTISION;
    //     processed_map_data.target_position =
    //         received_worldmodel_map->processed_map_data.target_position;
    //   }

    //   if (received_worldmodel_map->processed_map_data.available &
    //       ProcessedMapData::LANE) {
    //     processed_map_data.available |= ProcessedMapData::LANE;
    //     processed_map_data.lanes =
    //         received_worldmodel_map->processed_map_data.lanes;
    //   }

    //   if (received_worldmodel_map->processed_map_data.available &
    //       ProcessedMapData::MAP_POI_INFO) {
    //     processed_map_data.available |= ProcessedMapData::MAP_POI_INFO;
    //     processed_map_data.map_poi_info =
    //         received_worldmodel_map->processed_map_data.map_poi_info;
    //   }

    //   if (received_worldmodel_map->processed_map_data.available &
    //       ProcessedMapData::INTERSECTION) {
    //     processed_map_data.available |= ProcessedMapData::INTERSECTION;
    //     processed_map_data.intersections =
    //         received_worldmodel_map->processed_map_data.intersections;
    //   }

    //   if (received_worldmodel_map->processed_map_data.available &
    //       ProcessedMapData::SELF_POSITION) {
    //     processed_map_data.available |= ProcessedMapData::SELF_POSITION;
    //     processed_map_data.self_position =
    //         received_worldmodel_map->processed_map_data.self_position;
    //   }

    //   if (received_worldmodel_map->processed_map_data.available &
    //       ProcessedMapData::LANE_MERGING_SPLITTING_POINT) {
    //     processed_map_data.available |=
    //         ProcessedMapData::LANE_MERGING_SPLITTING_POINT;
    //     processed_map_data.lane_merging_splitting_points =
    //         received_worldmodel_map->processed_map_data
    //             .lane_merging_splitting_points;
    //   }

    //   if (received_worldmodel_map->processed_map_data.available &
    //       ProcessedMapData::EXTRA_INFO) {
    //     processed_map_data.available |= ProcessedMapData::EXTRA_INFO;
    //     processed_map_data.extra_info =
    //         received_worldmodel_map->processed_map_data.extra_info;
    //   }
  }

  maf_worldmodel::SceneObjects scene_objects{};
  while (!worldmodel_scene_objects_receiver_->empty()) {
    std::shared_ptr<maf_worldmodel::SceneObjects> received_scene_objects{};
    auto ret =
        worldmodel_scene_objects_receiver_->pop_oldest(received_scene_objects);
    if (!ret) {
      continue;
    }
    if (received_scene_objects->available &
        SceneObjects::ON_PATH_POLYGON_OBJECTS) {
      scene_objects.available |= SceneObjects::ON_PATH_POLYGON_OBJECTS;
      scene_objects.on_path_polygon_objects =
          received_scene_objects->on_path_polygon_objects;
    }
    if (received_scene_objects->available &
        SceneObjects::SURROUND_POLYGON_OBJECTS) {
      scene_objects.available |= SceneObjects::SURROUND_POLYGON_OBJECTS;
      scene_objects.surround_polygon_objects =
          received_scene_objects->surround_polygon_objects;
    }
  }

  maf_worldmodel::FusionAPA fusion_apa{};
  while (!worldmodel_parking_slots_receiver_->empty()) {
    std::shared_ptr<maf_worldmodel::FusionAPA> received_fusion_apa{};
    auto ret =
        worldmodel_parking_slots_receiver_->pop_oldest(received_fusion_apa);
    if (!ret) {
      continue;
    }

    if (received_fusion_apa->available & FusionAPA::PARKING_SLOTS) {
      fusion_apa.available |= FusionAPA::PARKING_SLOTS;
      fusion_apa.parking_slots = received_fusion_apa->parking_slots;
    }

    if (received_fusion_apa->available & FusionAPA::EGO_PARKING_SLOT_TRACK_ID) {
      fusion_apa.available |= FusionAPA::EGO_PARKING_SLOT_TRACK_ID;
      fusion_apa.ego_parking_slot_track_id =
          received_fusion_apa->ego_parking_slot_track_id;
    }

    if (received_fusion_apa->available & FusionAPA::EGO_PARKING_SLOT_MAP_ID) {
      fusion_apa.available |= FusionAPA::EGO_PARKING_SLOT_MAP_ID;
      fusion_apa.ego_parking_slot_map_id =
          received_fusion_apa->ego_parking_slot_map_id;
    }

    if (received_fusion_apa->available &
        FusionAPA::SUGGESTED_PARKING_SLOT_TRACK_ID) {
      fusion_apa.available |= FusionAPA::SUGGESTED_PARKING_SLOT_TRACK_ID;
      fusion_apa.suggested_parking_slot_track_id =
          received_fusion_apa->suggested_parking_slot_track_id;
    }

    if (received_fusion_apa->available &
        FusionAPA::SUGGESTED_PARKING_SLOT_MAP_ID) {
      fusion_apa.available |= FusionAPA::SUGGESTED_PARKING_SLOT_MAP_ID;
      fusion_apa.suggested_parking_slot_map_id =
          received_fusion_apa->suggested_parking_slot_map_id;
    }

    if (received_fusion_apa->available & FusionAPA::RESERVED_INFO) {
      fusion_apa.available |= FusionAPA::RESERVED_INFO;
      fusion_apa.reserved_info = received_fusion_apa->reserved_info;
    }
  }

  maf_perception_interface::FusionGroundLineResult fusion_ground_lines{};
  if (!fusion_groundlines_receiver_->empty()) {
    std::shared_ptr<maf_perception_interface::FusionGroundLineResult>
        received_fusion_ground_lines;
    auto ret = fusion_groundlines_receiver_->fetch_newest_and_clear(
        received_fusion_ground_lines);
    if (ret) {
      fusion_ground_lines.header = received_fusion_ground_lines->header;
      fusion_ground_lines.meta = received_fusion_ground_lines->meta;
      fusion_ground_lines.ground_line =
          received_fusion_ground_lines->ground_line;
    }
  }

  maf_perception_interface::FusionGroundLineResult fusion_uss_ground_lines{};
  if (!fusion_uss_groundlines_receiver_->empty()) {
    std::shared_ptr<maf_perception_interface::FusionGroundLineResult>
        received_uss_fusion_ground_lines{};
    auto ret = fusion_uss_groundlines_receiver_->fetch_newest_and_clear(
        received_uss_fusion_ground_lines);
    if (ret) {
      fusion_uss_ground_lines.header = received_uss_fusion_ground_lines->header;
      fusion_uss_ground_lines.meta = received_uss_fusion_ground_lines->meta;
      fusion_uss_ground_lines.ground_line =
          received_uss_fusion_ground_lines->ground_line;
    }
  }

  maf_sensor_interface::UltrasonicUpaReport fusion_uss{};
  if (!fusion_uss_receiver_->empty()) {
    std::shared_ptr<maf_sensor_interface::UltrasonicUpaReport>
        received_fusion_uss{};
    auto ret =
        fusion_uss_receiver_->fetch_newest_and_clear(received_fusion_uss);
    if (ret) {
      fusion_uss.header = received_fusion_uss->header;
      fusion_uss.uss_upa_distance = received_fusion_uss->uss_upa_distance;
    }
  }

  if (scene_objects.available & SceneObjects::ON_PATH_POLYGON_OBJECTS) {
    fill_square_mapping_result(on_path_polygon_objects_,
                               scene_objects.on_path_polygon_objects);
    last_feed_time_[FEED_SCENE_OBJECT] = current_time;
  }
  if (scene_objects.available & SceneObjects::SURROUND_POLYGON_OBJECTS) {
    SquareMapResponse surround_polygon_objects;
    fill_square_mapping_result(surround_polygon_objects,
                               scene_objects.surround_polygon_objects);
    world_model_->feed_square_map_response(surround_polygon_objects);
    last_feed_time_[FEED_SCENE_OBJECT] = current_time;
  }

  if (processed_map_data.available & ProcessedMapData::TARGET_POTISION) {
    if (processed_map_data.target_position.parking_target_position.available &
        ParkingTargetPosition::TARGET_MAP_POI) {
      AimedPoiInfo aimed_poi_info;
      fill_target_poi_info(processed_map_data, aimed_poi_info);
      world_model_->feed_aimed_poi_info(aimed_poi_info);
    }
  }

  // if ((processed_map_data.available & ProcessedMapData::LANE) &&
  //     (processed_map_data.available & ProcessedMapData::MAP_POI_INFO) &&
  //     (processed_map_data.available & ProcessedMapData::INTERSECTION) &&
  //     (processed_map_data.available & ProcessedMapData::SELF_POSITION) &&
  //     (processed_map_data.available & ProcessedMapData::LANE_STRATEGY) &&
  //     (total_map_data.available &
  //      ProcessedMapData::LANE_MERGING_SPLITTING_POINT) &&
  //     (msd_worldmodel.extra.available &
  //      (WorldModelExtra::JSON | WorldModelExtra::VERSION))) {
  //   // only feed map info when hdmap is valid
  //   MapInfo map_info;
  //   fill_map_info(processed_map_data, map_info);
  //   world_model_->feed_map_info(map_info);
  //   last_feed_time_[FEED_MAP_INFO] = current_time;
  // }

  if ((fusion_apa.available & FusionAPA::PARKING_SLOTS) &&
      (fusion_apa.available & FusionAPA::EGO_PARKING_SLOT_TRACK_ID) &&
      (fusion_apa.available & FusionAPA::EGO_PARKING_SLOT_MAP_ID)) {

    std::vector<ParkingLotDetectionInfo> parking_lots_detection_fusion_results;
    fill_parking_lot_info(fusion_apa, parking_lots_detection_fusion_results);

    world_model_->feed_parking_lots_detection_fusion_results(
        parking_lots_detection_fusion_results);

    last_feed_time_[FEED_FUSION_APA] = current_time;
  }

  if (fusion_ground_lines.ground_line.available &
      GroundLinePerception::VISION_PERCEPTION_GROUND_LINE_RESULT) {
    std::vector<GroundLine> ground_line_fusion_array;
    std::vector<FusionFreespacePoint> freespace_array;
    fill_ground_line_info(fusion_ground_lines.ground_line.ground_line_data,
                          ground_line_fusion_array);
    fill_ground_line_info(fusion_ground_lines.ground_line.ground_line_data,
                          freespace_array);
    // world_model_->feed_parking_ground_line_fusion(ground_line_fusion_array);
    world_model_->feed_parking_fusion_freespace(freespace_array);
    last_feed_time_[FEED_FUSION_GROUNDLINE] = current_time;
  }

  if (fusion_uss_ground_lines.ground_line.available &
      GroundLinePerception::VISION_PERCEPTION_GROUND_LINE_RESULT) {
    std::vector<GroundLine> uss_ground_line_fusion_array;
    fill_ground_line_info(fusion_uss_ground_lines.ground_line.ground_line_data,
                          uss_ground_line_fusion_array);
    world_model_->feed_parking_ground_line_fusion(uss_ground_line_fusion_array);
    last_feed_time_[FEED_FUSION_GROUNDLINE] = current_time;
  }

  maf_perception_interface::PerceptionFusionObjectResult fusion_objects{};
  if (!fusion_objects_receiver_->empty()) {
    std::shared_ptr<maf_perception_interface::PerceptionFusionObjectResult>
        received_fusion_objects{};
    auto ret = fusion_objects_receiver_->fetch_newest_and_clear(
        received_fusion_objects);
    if (ret) {
      fusion_objects.header = received_fusion_objects->header;
      fusion_objects.meta = received_fusion_objects->meta;
      fusion_objects.perception_fusion_objects_data =
          received_fusion_objects->perception_fusion_objects_data;
      fusion_objects.reserved_infos = received_fusion_objects->reserved_infos;
      std::vector<FusionObject> fusion_array;
      fill_fusion_object_info(fusion_objects.perception_fusion_objects_data,
                              fusion_array);
      world_model_->feed_parking_fusion_car(fusion_array);
      last_feed_time_[FEED_FUSION_INFO] = current_time;
    }
  }

  // if (msd_worldmodel.environment_data.fusion_apa.available &
  //     FusionAPA::STATIC_OBJECTS) {
  //   std::vector<FusionObject> static_fusion_array;
  //   fill_fusion_object_info(
  //       msd_worldmodel.environment_data.fusion_apa.static_objects
  //           .perception_fusion_objects_data,
  //       static_fusion_array);
  //   world_model_->feed_parking_static_fusion_car(static_fusion_array);
  // }

  std::vector<UssResult> uss_results;
  auto &sonar = fusion_uss.uss_upa_distance.sonar;
  for (size_t i = 0; i < sonar.size(); i++) {
    UssResult uss_result;
    uss_result.type = (UssType)i;
    uss_result.range = sonar[i].available ? sonar[i].upadistance : FLT_MAX;
    uss_results.push_back(uss_result);
  }
  world_model_->feed_parking_uss_results(uss_results);
  last_feed_time_[FEED_FUSION_USS] = current_time;
}

// seralize LongitudinalBehaviorPlannerOutput
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Leader, id, d_rel, d_path)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(FreespacePoint, id, d_rel, d_path)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LongitudinalBehaviorPlannerOutput, lead_cars,
                                   free_space)
maf_planning::Planning PlanningTask::generate_planning_output() {
  const auto &planning_status = PlanningContext::Instance()->planning_status();

  maf_planning::Planning msg{};
  const auto &planning_result = planning_status.planning_result;

  auto timestamp = MTIME()->timestamp("publish");
  msd_planning::MSDPlanning_record_timestamp(planning_status.trigger_msg_id,
                                             "pnc_plan_timestamp-publish",
                                             timestamp.ns());
  msg.header.stamp = timestamp.ns();
  msg.header.frame_id = "map";
  msg.meta.timestamp_us = ego_pose_timestamp_us_;
  msg.meta.plan_timestamp_us = timestamp.us(); // TODO: add plan ts @tianbo

  msg.trajectory.available |= maf_planning::Trajectory::PATH;
  msg.trajectory.path.resize(planning_result.traj_pose_array.size());
  for (size_t i = 0; i < msg.trajectory.path.size(); i++) {
    msg.trajectory.path[i].position_enu.x =
        planning_result.traj_pose_array[i].x;
    msg.trajectory.path[i].position_enu.y =
        planning_result.traj_pose_array[i].y;

    msg.trajectory.path[i].heading_yaw =
        planning_result.traj_pose_array[i].theta;
  }

  msg.trajectory.available |= maf_planning::Trajectory::VELOCITY;
  msg.trajectory.velocity.available |= maf_planning::Velocity::TARGET_VALUE;
  msg.trajectory.velocity.target_value = planning_result.v_target;

  msg.trajectory.available |= maf_planning::Trajectory::ACCELERATION;
  msg.trajectory.acceleration.available |=
      maf_planning::Acceleration::RANGE_LIMIT;
  msg.trajectory.acceleration.range_limit.min = planning_result.a_target_min;
  msg.trajectory.acceleration.range_limit.max = planning_result.a_target_max;
  msg.trajectory.velocity.available |= maf_planning::Velocity::CRUISE_VELOCITY;
  msg.trajectory.velocity.cruise_velocity = cruise_velocity_ / 3.6;

  msg.turn_signal_command.available |=
      maf_planning::TurnSignalCommand::TURN_SIGNAL_DATA;
  msg.turn_signal_command.turn_signal_data.value =
      PlanningContext::Instance()->turn_signal_cmd().value;

  msg.gear_command.available |= maf_planning::GearCommand::GEAR_DATA;
  if (planning_result.gear == GearState::NONE) {
    msg.gear_command.gear_data.value = maf_planning::Gear::NONE;
  } else {
    msg.gear_command.gear_data.value = (unsigned)planning_result.gear + 1;
  }

  msg.plan_status.available |= maf_planning::PlanStatus::ALGORITHM_STATUS;
  msg.plan_status.algorithm_status.scene |=
      maf_planning::PlanAlgorithmStatus::PARKING;

  if (planning_result.is_apa) {
    if (planning_status.scenario.status_type == StatusType::APA ||
        planning_status.scenario.status_type == StatusType::APOA) {
      msg.plan_status.algorithm_status.action |=
          maf_planning::PlanAlgorithmStatus::LANE_KEEP;
      msg.plan_status.algorithm_status.action_status =
          maf_planning::PlanAlgorithmStatus::LANE_KEEPING;
    } else if (planning_status.is_pullover) {
      msg.plan_status.algorithm_status.action |=
          maf_planning::PlanAlgorithmStatus::PULL_OVER;
    } else {
      msg.plan_status.algorithm_status.action |=
          maf_planning::PlanAlgorithmStatus::OPEN_SPACE_PASS;
    }
  } else {
    msg.plan_status.algorithm_status.action |=
        maf_planning::PlanAlgorithmStatus::LANE_KEEP;
    if (PlanningContext::Instance()->has_scene(
            planning_result.scene_avp, ParkingSceneType::SCENE_SIDEPASS)) {
      msg.plan_status.algorithm_status.action_status =
          maf_planning::PlanAlgorithmStatus::SIDE_PASS;
    } else {
      msg.plan_status.algorithm_status.action_status =
          maf_planning::PlanAlgorithmStatus::LANE_KEEPING;
    }
  }

  switch (planning_status.scenario.status_type) {
  case StatusType::WAIT:
    msg.plan_status.algorithm_status.function |=
        maf_planning::PlanAlgorithmStatus::WAIT;
    break;
  case StatusType::SEARCH:
    msg.plan_status.algorithm_status.function |=
        maf_planning::PlanAlgorithmStatus::SEARCH;
    break;
  case StatusType::AVP:
    msg.plan_status.algorithm_status.function |=
        maf_planning::PlanAlgorithmStatus::SEARCH;
    break;
  case StatusType::APA:
    msg.plan_status.algorithm_status.function |=
        maf_planning::PlanAlgorithmStatus::APA;
    break;
  case StatusType::APOA:
    msg.plan_status.algorithm_status.function |=
        maf_planning::PlanAlgorithmStatus::APOA;
    break;
  case StatusType::RPA_STRAIGHT_STANDBY:
    msg.plan_status.algorithm_status.function |= 32; // RPA_STRAIGHT_STANDBY
    break;
  case StatusType::RPA_STRAIGHT:
    msg.plan_status.algorithm_status.function |= 64; // RPA_STRAIGHT
    break;
  default:
    msg.plan_status.algorithm_status.function |=
        maf_planning::PlanAlgorithmStatus::WAIT;
    break;
  }

  msg.extra.available |= maf_planning::PlanExtra::VERSION;
  msg.extra.version = PlanningContext::Instance()->get_version();

  msg.extra.available |= maf_planning::PlanExtra::JSON;

  auto extra_json = mjson::Json(mjson::Json::object());
  switch (planning_status.task_status.task) {
  case StatusType::WAIT:
    extra_json["task"] = "WAIT";
    break;
  case StatusType::SEARCH:
    extra_json["task"] = "SEARCH";
    break;
  case StatusType::AVP:
    extra_json["task"] = "CRUISE";
    break;
  case StatusType::APA:
    extra_json["task"] = "APA";
    break;
  case StatusType::APOA:
    extra_json["task"] = "APOA";
    break;
  case StatusType::RPA_STRAIGHT_STANDBY:
    extra_json["task"] = "RPA_STRAIGHT_STANDBY";
    break;
  case StatusType::RPA_STRAIGHT:
    extra_json["task"] = "RPA_STRAIGHT";
    break;
  default:
    extra_json["task"] = "WAIT";
    break;
  }

  switch (planning_status.task_status.status) {
  case TaskStatusType::RUNNING:
    msg.plan_status.algorithm_status.function_status |=
        maf_planning::PlanAlgorithmStatus::RUNNING;
    extra_json["status"] = "RUNNING";
    break;
  case TaskStatusType::PAUSED:
    msg.plan_status.algorithm_status.function_status |=
        maf_planning::PlanAlgorithmStatus::PAUSED;
    extra_json["status"] = "PAUSED";
    break;
  case TaskStatusType::SUCCEEDED:
    msg.plan_status.algorithm_status.function_status |=
        maf_planning::PlanAlgorithmStatus::SUCCEEDED;
    extra_json["status"] = "SUCCEEDED";
    break;
  case TaskStatusType::FAILED:
    msg.plan_status.algorithm_status.function_status |=
        maf_planning::PlanAlgorithmStatus::FAILED;
    extra_json["status"] = "FAILED";
    break;
  default:
    msg.plan_status.algorithm_status.function_status |=
        maf_planning::PlanAlgorithmStatus::RUNNING;
    extra_json["task"] = "RUNNING";
    break;
  }

  if (world_model_->get_pause_status()) {
    msg.plan_status.algorithm_status.function_status |=
        maf_planning::PlanAlgorithmStatus::PAUSED;
    extra_json["status"] = "PAUSED";
  }

  auto &parking_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;
  if (parking_slot_info.corners.size() == 4) {
    mjson::Json::array parking_slot;
    for (auto &corner : parking_slot_info.corners) {
      auto corner_json = mjson::Json(mjson::Json::object());
      corner_json["x"] = corner.x;
      corner_json["y"] = corner.y;
      parking_slot.push_back(corner_json);
    }
    extra_json["parking_slot"] = parking_slot;
  }

  double parking_slot_iou = 0.0;
  if (parking_slot_info.original_corners.size() == 4) {
    std::vector<planning_math::Vec2d> origin_corners;
    for (const auto& corner : parking_slot_info.original_corners) {
      origin_corners.emplace_back(corner.x, corner.y);
    }
    planning_math::Polygon2d overlap_polygon;
    planning_math::Polygon2d ego_polygon(world_model_->get_ego_state().ego_box);
    planning_math::Polygon2d slot_polygon(origin_corners);
    (void)ego_polygon.ComputeOverlap(slot_polygon, &overlap_polygon);
    parking_slot_iou = overlap_polygon.area() / ego_polygon.area();
  }

  extra_json["parking_slot_iou"] =
      parking_slot_info.type.value == ParkingSlotType::PARALLEL
          ? -parking_slot_iou
          : parking_slot_iou;
  extra_json["is_path_new"] =
      PlanningContext::Instance()->open_space_path().isNew();
  int plan_pause_flag = world_model_->get_pause_status() ? 1 : 0;
  extra_json["plan_pause_flag"] = plan_pause_flag;
  extra_json["is_hard_brake"] = world_model_->get_hard_brake();
  extra_json["is_comfortable_brake"] = world_model_->get_pause_status();
  extra_json["slot_type"] = int(parking_slot_info.type.value) + 1;
  extra_json["is_last_path"] = PlanningContext::Instance()
                                   ->parking_behavior_planner_output()
                                   .is_last_path;

  // calculate dist to PSD opening and bottom line
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->dist_to_wheel_stop = 2.0;
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->min_dist_to_opening = -1;
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->min_dist_to_bottom = 10;
  std::vector<planning_math::Vec2d> origin_corners;
  if (parking_slot_info.corners.size() == 4) {
    for (const auto &corner : parking_slot_info.original_corners) {
      origin_corners.emplace_back(corner.x, corner.y);
    }
    Pose2D ego_pose = world_model_->get_ego_state().ego_pose;
    planning_math::Vec2d ego_pt(ego_pose.x, ego_pose.y);

    if (parking_slot_info.type.value != ParkingSlotType::PARALLEL &&
        parking_slot_info.original_corners.size() == 4) {
      planning_math::Polygon2d slot_polygon(origin_corners);
      // set default value
      if (!slot_polygon.IsPointIn(ego_pt)) {
        PlanningContext::Instance()
            ->mutable_parking_behavior_planner_output()
            ->min_dist_to_opening = -1;
        PlanningContext::Instance()
            ->mutable_parking_behavior_planner_output()
            ->min_dist_to_bottom = 10;
      } else {
        double dx = VehicleParam::Instance()->width_wo_rearview_mirror / 2 *
                    sin(ego_pose.theta);
        double dy = -VehicleParam::Instance()->width_wo_rearview_mirror / 2 *
                    cos(ego_pose.theta);
        planning_math::Vec2d left_rear_wheel(ego_pose.x - dx, ego_pose.y - dy);
        planning_math::Vec2d right_rear_wheel(ego_pose.x + dx, ego_pose.y + dy);

        // opening
        planning_math::Vec2d p_0 = origin_corners[0];
        planning_math::Vec2d p_3 = origin_corners[3];
        planning_math::Vec2d vec_p0_p3(p_3.x() - p_0.x(), p_3.y() - p_0.y());
        planning_math::Vec2d vec_p0_lw(left_rear_wheel.x() - p_0.x(),
                                       left_rear_wheel.y() - p_0.y());
        int direc = vec_p0_p3.CrossProd(vec_p0_lw) > 0 ? -1 : 1;
        double dist_lw_opening =
            direc *
            calculateMinDistOnLine(p_0, p_3, left_rear_wheel, ego_pose.theta);
        planning_math::Vec2d vec_p0_rw(right_rear_wheel.x() - p_0.x(),
                                       right_rear_wheel.y() - p_0.y());
        direc = vec_p0_p3.CrossProd(vec_p0_rw) > 0 ? -1 : 1;
        double dist_rw_opening =
            direc *
            calculateMinDistOnLine(p_0, p_3, right_rear_wheel, ego_pose.theta);

        double minor_dist_opening = std::min(dist_lw_opening, dist_rw_opening);
        PlanningContext::Instance()
            ->mutable_parking_behavior_planner_output()
            ->min_dist_to_opening =
            minor_dist_opening < 0 ? -1 : minor_dist_opening;

        // bottom
        planning_math::Vec2d p_1 = origin_corners[1];
        planning_math::Vec2d p_2 = origin_corners[2];
        planning_math::Vec2d vec_p1_p2(p_2.x() - p_1.x(), p_2.y() - p_1.y());
        planning_math::Vec2d vec_p1_lw(left_rear_wheel.x() - p_1.x(),
                                       left_rear_wheel.y() - p_1.y());
        direc = vec_p1_p2.CrossProd(vec_p1_lw) > 0 ? 1 : -1;
        double dist_lw_bottom =
            direc *
            calculateMinDistOnLine(p_1, p_2, left_rear_wheel, ego_pose.theta);
        planning_math::Vec2d vec_p1_rw(right_rear_wheel.x() - p_1.x(),
                                       right_rear_wheel.y() - p_1.y());
        direc = vec_p1_p2.CrossProd(vec_p1_rw) > 0 ? 1 : -1;
        double dist_rw_bottom =
            direc *
            calculateMinDistOnLine(p_1, p_2, right_rear_wheel, ego_pose.theta);

        double minor_dist_bottom = std::min(dist_lw_bottom, dist_rw_bottom);
        PlanningContext::Instance()
            ->mutable_parking_behavior_planner_output()
            ->min_dist_to_bottom = minor_dist_bottom;

        // stopper
        if (parking_slot_info.wheel_stop_info.vision_wheel_stop_available) {
          planning_math::Vec2d p_w1(
              parking_slot_info.wheel_stop_info.vision_point1.x,
              parking_slot_info.wheel_stop_info.vision_point1.y);
          planning_math::Vec2d p_w2(
              parking_slot_info.wheel_stop_info.vision_point2.x,
              parking_slot_info.wheel_stop_info.vision_point2.y);
          planning_math::Vec2d vec_pw1_pw2(p_w2.x() - p_w1.x(),
                                           p_w2.y() - p_w1.y());
          planning_math::Vec2d vec_pw1_lw(left_rear_wheel.x() - p_w1.x(),
                                          left_rear_wheel.y() - p_w1.y());
          direc = vec_pw1_pw2.CrossProd(vec_pw1_lw) > 0 ? 1 : -1;
          double dist_lw_stopper =
              direc * calculateMinDistOnLine(p_w1, p_w2, left_rear_wheel,
                                             ego_pose.theta);
          planning_math::Vec2d vec_pw1_rw(right_rear_wheel.x() - p_w1.x(),
                                          right_rear_wheel.y() - p_w1.y());
          direc = vec_pw1_pw2.CrossProd(vec_pw1_rw) > 0 ? 1 : -1;
          double dist_rw_stopper =
              direc * calculateMinDistOnLine(p_w1, p_w2, right_rear_wheel,
                                             ego_pose.theta);

          double minor_dist_stopper =
              std::min(dist_lw_stopper, dist_rw_stopper);
          PlanningContext::Instance()
              ->mutable_parking_behavior_planner_output()
              ->dist_to_wheel_stop = minor_dist_stopper;
        }
      }
    }
  }

  extra_json["dist_to_stopper"] = PlanningContext::Instance()
                                      ->parking_behavior_planner_output()
                                      .dist_to_wheel_stop;
  extra_json["min_dist_to_opening"] = PlanningContext::Instance()
                                      ->parking_behavior_planner_output()
                                      .min_dist_to_opening;
  extra_json["min_dist_to_bottom"] = PlanningContext::Instance()
                                      ->parking_behavior_planner_output()
                                      .min_dist_to_bottom;

  int special_slot_type = 0;
  if(parking_slot_info.type.value == ParkingSlotType::PARALLEL) {
    if(std::abs(parking_slot_iou) > 0.1) {
      if(parking_slot_info.special_slot_type == 1) {
        special_slot_type = 1;
      }
      if(parking_slot_info.special_slot_type == 2) {
        special_slot_type = 2;
      }
    }
  }
  if (PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->is_narrow_channel) {
    special_slot_type = 1;
  }
  extra_json["special_slot_type"] = special_slot_type;
  extra_json["lon_inflation"] = CarParams::GetInstance()->lon_inflation();


  // auto &t_lines =
  //     PlanningContext::Instance()->openspace_decider_output().T_lines;
  // std::vector<planning_math::LineSegment2d> lines;
  // lines.emplace_back(t_lines.road_lower_left_bound);
  // lines.emplace_back(t_lines.road_lower_right_bound);
  // lines.emplace_back(t_lines.road_upper_bound);
  // lines.emplace_back(t_lines.slot_left_bound);
  // lines.emplace_back(t_lines.slot_right_bound);

  // mjson::Json::array lines_json;
  // for (auto &line : lines) {
  //   mjson::Json::array line_json;
  //   auto start = mjson::Json(mjson::Json::object());
  //   auto end = mjson::Json(mjson::Json::object());
  //   start["x"] = line.start().x();
  //   start["y"] = line.start().y();
  //   end["x"] = line.end().x();
  //   end["y"] = line.end().y();
  //   line_json.push_back(start);
  //   line_json.push_back(end);
  //   lines_json.push_back(mjson::Json(line_json));
  // }
  // extra_json["lines"] = lines_json;

  // nlohmann::json lon_bpo_nlohmann_json =
  //     PlanningContext::Instance()
  //         ->longitudinal_behavior_planner_output(); // automatic
  // std::string error;
  // mjson::Json lon_bpo_mjson =
  //     mjson::Json::parse(lon_bpo_nlohmann_json.dump(), error);
  // if (!error.empty()) {
  //   // std::cout << "mjson::Json::parse(lon_bpo_nlohmann_json, error) failed:
  //   "
  //   //           << error << std::endl;
  // }
  // extra_json["longitudinal_behavior_planner_output"] = lon_bpo_mjson;

  // remaining distance
  ParkingLongitudinalBehaviorPlanner::compute_lead_obs_info(
      parking_slot_info,
      &PlanningContext::Instance()
           ->mutable_longitudinal_behavior_planner_output()
           ->remain_dist_info_,
      &PlanningContext::Instance()
           ->mutable_longitudinal_behavior_planner_output()
           ->is_need_pause_);

  auto &fs_point = PlanningContext::Instance()
                       ->longitudinal_behavior_planner_output()
                       .free_space;

  int perpendicular_bottom_line_type = 0;
  if (VehicleParam::Instance()->car_type == "C03") {
    if(parking_slot_info.type.value == ParkingSlotType::PERPENDICULAR) {
      if (parking_slot_info.bottom_line_type == 1 && parking_slot_iou > 0.25) {
        perpendicular_bottom_line_type = 1;
      }
    }
  }
  extra_json["bottom_line_type"] = perpendicular_bottom_line_type;

  extra_json["lead_free_space_type"] = (int)fs_point.type;

   // when planning, remaining_distance is set to 0
  const ParkingBehaviorPlannerOutput &parking_behavior_planner_output =
      PlanningContext::Instance()->parking_behavior_planner_output();
  const auto& sv_config2 = msquare::CarParams::GetInstance()->car_config.sv_config;
  if (!parking_behavior_planner_output.is_move_ready && sv_config2.use_sv_speed_generator) {
    PlanningContext::Instance()
      ->mutable_longitudinal_behavior_planner_output()
      ->remain_dist_info_.remaining_distance_ = 0.0;
  }

  extra_json["remaining_distance"] = mjson::Json(PlanningContext::Instance()
      ->mutable_longitudinal_behavior_planner_output()
      ->remain_dist_info_.remaining_distance_);

  extra_json["has_wheel_stop"] =
      mjson::Json(parking_slot_info.wheel_stop_info.available);
  extra_json["approaching_wheel_stop"] =
      mjson::Json(PlanningContext::Instance()
                      ->parking_behavior_planner_output()
                      .approaching_wheel_stop);
  extra_json["planner_clac_duration"] =
      mjson::Json(PlanningContext::Instance()
                      ->openspace_motion_planner_output()
                      .planner_calc_duration);

  auto vec_sl_points = PlanningContext::Instance()->vec_sl_points();
  mjson::Json::array v_arr;
  if (!vec_sl_points.empty() &&
      !vec_sl_points[0].empty()) {
      int i = 0;
      for (const auto& vec : vec_sl_points) {
        mjson::Json::array arr;
        for (const auto& p : vec) {
          // limit mpc trajectory sl's length to 1.0m
          if (i == 0 && p.first > 1.0)
            break;
          auto obj = mjson::Json(mjson::Json::object());
          obj["s"] = p.first;
          obj["l"] = p.second;
          arr.push_back(obj);
        }
        i++;
        v_arr.emplace_back(arr);
      }
      extra_json["vec_sl_points"] = v_arr;
  }

  double not_use_comfortable_min_s =
      msquare::CarParams::GetInstance()
          ->car_config.sv_config.not_use_comfortable_min_s;
  if (not_use_comfortable_min_s > 0.0 &&
      PlanningContext::Instance()
              ->mutable_longitudinal_behavior_planner_output()
              ->remain_dist_info_.remaining_distance_ <
          not_use_comfortable_min_s) {
      extra_json["is_comfortable_brake"] = false;
  }

  msg.extra.json = extra_json.dump();

  nlohmann::json plan_strategy_name_json;
  plan_strategy_name_json["behavior"] =
      PlanningContext::Instance()->parking_behavior_planner_output().behavior;

  std::string statemachine_str = PlanningContext::Instance()
                                     ->mutable_planning_status()
                                     ->statemachine_stringstream.str();
  PlanningContext::Instance()
      ->mutable_planning_status()
      ->statemachine_stringstream.reset(); // clear
  if (statemachine_str.find("enter") != std::string::npos) {
    plan_strategy_name_json["statemachine_stringstream"] = statemachine_str;
  }

  while (!sbp_debug_receiver_->empty()) {
    std::string sbp_debug{};
    auto ret = sbp_debug_receiver_->pop_oldest(sbp_debug);
    if (!ret) {
      continue;
    }
    sbp_debug_info_ += sbp_debug + ". ";
  }
  plan_strategy_name_json["sbp_debug"] = sbp_debug_info_;
  sbp_debug_info_ = "";
  msg.meta.plan_strategy_name = plan_strategy_name_json.dump();

  return msg;
}

maf_planning::Planning PlanningTask::generate_planning_output_sop() {
  const auto &planning_status = PlanningContext::Instance()->planning_status();

  maf_planning::Planning msg{};
  const auto &planning_result = planning_status.planning_result;
  auto planning_control_interface = planning_status.plan_control_interface;

  auto timestamp = MTIME()->timestamp("publish");
  msd_planning::MSDPlanning_record_timestamp(planning_status.trigger_msg_id,
                                             "pnc_plan_timestamp-publish",
                                             timestamp.ns());
  msg.header.stamp = timestamp.ns();
  msg.header.frame_id = "map";
  msg.meta.timestamp_us = ego_pose_timestamp_us_;
  msg.meta.plan_timestamp_us = timestamp.us(); // TODO: add plan ts @tianbo
  PlanningContext::Instance()->mutable_planning_status()
      ->planning_result.pre_planning_time = msg.meta.plan_timestamp_us;

  msg.trajectory.available |= maf_planning::Trajectory::PATH;
  const auto& second_trajectory = planning_result.second_traj_pose_array;
  const auto& first_trajectory = planning_result.traj_pose_array;
  const auto& first_curvature = planning_result.traj_curvature;
  const auto& second_traj_curvature = planning_result.second_traj_curvature;
  MSD_LOG(ERROR, "the first trajectory path size is: %d, the curvature size is: %d", 
      first_trajectory.size(), first_curvature.size());
  if (first_trajectory.size() >= 10 && second_trajectory.size() >= 0) {
    msg.trajectory.path.resize(first_trajectory.size() +
        second_trajectory.size() - 10);
    // std::cout << "the first size  is: " << first_trajectory.size()
    //           << "  second_trajectory size is: " << second_trajectory.size()
    //           << " ve size is: " << planning_result.traj_vel_array.size()
    //           << std::endl;
    // if (first_trajectory.size() > 0 && second_trajectory.size() > 0) {
    //   std::cout << "the first traj final pt is:" << first_trajectory.back().x
    //             << " y:" << first_trajectory.back().y
    //             << std::endl;
    //   std::cout << "the second traj final pt is:" << second_trajectory.back().x
    //             << " y:" << second_trajectory.back().y
    //             << std::endl;
    // }
    for (size_t i = 0; i < first_trajectory.size(); i++) {
      msg.trajectory.path[i].position_enu.x =
          first_trajectory[i].x;
      msg.trajectory.path[i].position_enu.y =
          first_trajectory[i].y;

      msg.trajectory.path[i].heading_yaw =
          first_trajectory[i].theta;
      if (i < first_curvature.size()) {
        msg.trajectory.path[i].curvature = first_curvature[i];
      }
      if (i >= planning_result.traj_vel_array.size()) {
        continue;
      }
      if (std::fabs(planning_result.traj_vel_array[i]) < 0.01) {
        break;
      }
    }
    planning_control_interface.gear_change_index = first_trajectory.size() - 10;
    if (msg.trajectory.path.size() > 0) {
      MLOG_ERROR("the last point is x: %f, y %f",
          msg.trajectory.path.back().position_enu.x,
          msg.trajectory.path.back().position_enu.y);
    }
    for (size_t i = first_trajectory.size() - 10; i < msg.trajectory.path.size(); i++) {
      msg.trajectory.path[i].position_enu.x =
          second_trajectory[i - first_trajectory.size() + 10].x;
      msg.trajectory.path[i].position_enu.y =
          second_trajectory[i - first_trajectory.size() + 10].y;

      msg.trajectory.path[i].heading_yaw =
          second_trajectory[i - first_trajectory.size() + 10].theta;
      if (i - first_trajectory.size() + 10 < second_traj_curvature.size()) {
        msg.trajectory.path[i].curvature = second_traj_curvature[i - first_trajectory.size() + 10];
      }
    }
  }
  msg.trajectory.available |= maf_planning::Trajectory::VELOCITY;
  msg.trajectory.velocity.available |= maf_planning::Velocity::TARGET_VALUE;
  msg.trajectory.velocity.target_value = planning_result.v_target;
  msg.trajectory.velocity.available |= maf_planning::Velocity::VEL_POINTS;
  const auto pwj_trajectory = planning_result.pwj_trajectory;
  for (const auto& traj_pt : pwj_trajectory) {
    maf_planning::VelocityPoint vel_pt;
    maf_planning::AccelerationPoint accel_pt;
    vel_pt.relative_time = traj_pt.relative_time;
    vel_pt.distance = traj_pt.path_point.s;
    vel_pt.target_velocity = traj_pt.v;
    accel_pt.acc = traj_pt.a;
    msg.trajectory.velocity.vel_points.emplace_back(vel_pt);
    msg.trajectory.acceleration.acc_points.emplace_back(accel_pt);
  }


  msg.trajectory.available |= maf_planning::Trajectory::ACCELERATION;
  msg.trajectory.acceleration.available |=
      maf_planning::Acceleration::RANGE_LIMIT;
  msg.trajectory.acceleration.available |=
      maf_planning::Acceleration::ACC_POINTS;
  msg.trajectory.acceleration.range_limit.min = planning_result.a_target_min;
  msg.trajectory.acceleration.range_limit.max = planning_result.a_target_max;
  msg.trajectory.velocity.available |= maf_planning::Velocity::CRUISE_VELOCITY;
  msg.trajectory.velocity.cruise_velocity = cruise_velocity_ / 3.6;

  msg.turn_signal_command.available |=
      maf_planning::TurnSignalCommand::TURN_SIGNAL_DATA;
  msg.turn_signal_command.turn_signal_data.value =
      PlanningContext::Instance()->turn_signal_cmd().value;

  msg.gear_command.available |= maf_planning::GearCommand::GEAR_DATA;
  if (planning_result.gear == GearState::NONE) {
    msg.gear_command.gear_data.value = maf_planning::Gear::NONE;
  } else {
    msg.gear_command.gear_data.value = (unsigned)planning_result.gear + 1;
  }

  msg.plan_status.available |= maf_planning::PlanStatus::ALGORITHM_STATUS;
  msg.plan_status.algorithm_status.scene |=
      maf_planning::PlanAlgorithmStatus::PARKING;

  if (planning_result.is_apa) {
    if (planning_status.scenario.status_type == StatusType::APA ||
        planning_status.scenario.status_type == StatusType::APOA) {
      msg.plan_status.algorithm_status.action |=
          maf_planning::PlanAlgorithmStatus::LANE_KEEP;
      msg.plan_status.algorithm_status.action_status =
          maf_planning::PlanAlgorithmStatus::LANE_KEEPING;
    } else if (planning_status.is_pullover) {
      msg.plan_status.algorithm_status.action |=
          maf_planning::PlanAlgorithmStatus::PULL_OVER;
    } else {
      msg.plan_status.algorithm_status.action |=
          maf_planning::PlanAlgorithmStatus::OPEN_SPACE_PASS;
    }
  } else {
    msg.plan_status.algorithm_status.action |=
        maf_planning::PlanAlgorithmStatus::LANE_KEEP;
    if (PlanningContext::Instance()->has_scene(
            planning_result.scene_avp, ParkingSceneType::SCENE_SIDEPASS)) {
      msg.plan_status.algorithm_status.action_status =
          maf_planning::PlanAlgorithmStatus::SIDE_PASS;
    } else {
      msg.plan_status.algorithm_status.action_status =
          maf_planning::PlanAlgorithmStatus::LANE_KEEPING;
    }
  }

  switch (planning_status.scenario.status_type) {
  case StatusType::WAIT:
    msg.plan_status.algorithm_status.function |=
        maf_planning::PlanAlgorithmStatus::WAIT;
    break;
  case StatusType::SEARCH:
    msg.plan_status.algorithm_status.function |=
        maf_planning::PlanAlgorithmStatus::SEARCH;
    break;
  case StatusType::AVP:
    msg.plan_status.algorithm_status.function |=
        maf_planning::PlanAlgorithmStatus::SEARCH;
    break;
  case StatusType::APA:
    msg.plan_status.algorithm_status.function |=
        maf_planning::PlanAlgorithmStatus::APA;
    break;
  case StatusType::APOA:
    msg.plan_status.algorithm_status.function |=
        maf_planning::PlanAlgorithmStatus::APOA;
    break;
  case StatusType::RPA_STRAIGHT_STANDBY:
    msg.plan_status.algorithm_status.function |= 32; // RPA_STRAIGHT_STANDBY
    break;
  case StatusType::RPA_STRAIGHT:
    msg.plan_status.algorithm_status.function |= 64; // RPA_STRAIGHT
    break;
  default:
    msg.plan_status.algorithm_status.function |=
        maf_planning::PlanAlgorithmStatus::WAIT;
    break;
  }

  msg.extra.available |= maf_planning::PlanExtra::VERSION;
  msg.extra.version = PlanningContext::Instance()->get_version();

  msg.extra.available |= maf_planning::PlanExtra::JSON;

  auto extra_json = mjson::Json(mjson::Json::object());
  switch (planning_status.task_status.task) {
  case StatusType::WAIT:
    extra_json["task"] = "WAIT";
    break;
  case StatusType::SEARCH:
    extra_json["task"] = "SEARCH";
    break;
  case StatusType::AVP:
    extra_json["task"] = "CRUISE";
    break;
  case StatusType::APA:
    extra_json["task"] = "APA";
    break;
  case StatusType::APOA:
    extra_json["task"] = "APOA";
    break;
  default:
    extra_json["task"] = "WAIT";
    break;
  }

  switch (planning_status.task_status.status) {
  case TaskStatusType::RUNNING:
    msg.plan_status.algorithm_status.function_status |=
        maf_planning::PlanAlgorithmStatus::RUNNING;
    extra_json["status"] = "RUNNING";
    break;
  case TaskStatusType::PAUSED:
    msg.plan_status.algorithm_status.function_status |=
        maf_planning::PlanAlgorithmStatus::PAUSED;
    extra_json["status"] = "PAUSED";
    break;
  case TaskStatusType::SUCCEEDED:
    msg.plan_status.algorithm_status.function_status |=
        maf_planning::PlanAlgorithmStatus::SUCCEEDED;
    extra_json["status"] = "SUCCEEDED";
    break;
  case TaskStatusType::FAILED:
    msg.plan_status.algorithm_status.function_status |=
        maf_planning::PlanAlgorithmStatus::FAILED;
    extra_json["status"] = "FAILED";
    break;
  default:
    msg.plan_status.algorithm_status.function_status |=
        maf_planning::PlanAlgorithmStatus::RUNNING;
    extra_json["task"] = "RUNNING";
    break;
  }

  if (world_model_->get_pause_status()) {
    msg.plan_status.algorithm_status.function_status |=
        maf_planning::PlanAlgorithmStatus::PAUSED;
    extra_json["status"] = "PAUSED";
  }

  auto &parking_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;
  if (parking_slot_info.corners.size() == 4) {
    mjson::Json::array parking_slot;
    for (auto &corner : parking_slot_info.corners) {
      auto corner_json = mjson::Json(mjson::Json::object());
      corner_json["x"] = corner.x;
      corner_json["y"] = corner.y;
      parking_slot.push_back(corner_json);
    }
    extra_json["parking_slot"] = parking_slot;
  }

  double parking_slot_iou = 0.0;
  if (parking_slot_info.original_corners.size() == 4) {
    std::vector<planning_math::Vec2d> origin_corners;
    for (const auto& corner : parking_slot_info.original_corners) {
      origin_corners.emplace_back(corner.x, corner.y);
    }
    planning_math::Polygon2d overlap_polygon;
    planning_math::Polygon2d ego_polygon(world_model_->get_ego_state().ego_box);
    planning_math::Polygon2d slot_polygon(origin_corners);
    (void)ego_polygon.ComputeOverlap(slot_polygon, &overlap_polygon);
    parking_slot_iou = overlap_polygon.area() / ego_polygon.area();
  }

  extra_json["parking_slot_iou"] =
      parking_slot_info.type.value == ParkingSlotType::PARALLEL
          ? -parking_slot_iou
          : parking_slot_iou;
  extra_json["is_path_new"] =
      PlanningContext::Instance()->open_space_path().isNew();
  int plan_pause_flag = world_model_->get_pause_status() ? 1 : 0;
  extra_json["plan_pause_flag"] = plan_pause_flag;
  extra_json["is_hard_brake"] = world_model_->get_hard_brake();
  extra_json["is_comfortable_brake"] = world_model_->get_pause_status();
  extra_json["slot_type"] = int(parking_slot_info.type.value) + 1;
  extra_json["is_last_path"] = PlanningContext::Instance()
                                   ->parking_behavior_planner_output()
                                   .is_last_path;

  // calculate dist to PSD opening and bottom line
    PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->dist_to_wheel_stop = 2.0;
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->min_dist_to_opening = -1;
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->min_dist_to_bottom = 10;
  std::vector<planning_math::Vec2d> origin_corners;
  if (parking_slot_info.corners.size() == 4) {
    for (const auto &corner : parking_slot_info.original_corners) {
      origin_corners.emplace_back(corner.x, corner.y);
    }
    Pose2D ego_pose = world_model_->get_ego_state().ego_pose;
    planning_math::Vec2d ego_pt(ego_pose.x, ego_pose.y);

    if (parking_slot_info.type.value != ParkingSlotType::PARALLEL &&
        parking_slot_info.original_corners.size() == 4) {
      planning_math::Polygon2d slot_polygon(origin_corners);
      // set default value
      if (!slot_polygon.IsPointIn(ego_pt)) {
        PlanningContext::Instance()
            ->mutable_parking_behavior_planner_output()
            ->min_dist_to_opening = -1;
        PlanningContext::Instance()
            ->mutable_parking_behavior_planner_output()
            ->min_dist_to_bottom = 10;
      } else {
        double dx = VehicleParam::Instance()->width_wo_rearview_mirror / 2 *
                    sin(ego_pose.theta);
        double dy = -VehicleParam::Instance()->width_wo_rearview_mirror / 2 *
                    cos(ego_pose.theta);
        planning_math::Vec2d left_rear_wheel(ego_pose.x - dx, ego_pose.y - dy);
        planning_math::Vec2d right_rear_wheel(ego_pose.x + dx, ego_pose.y + dy);

        // opening
        planning_math::Vec2d p_0 = origin_corners[0];
        planning_math::Vec2d p_3 = origin_corners[3];
        planning_math::Vec2d vec_p0_p3(p_3.x() - p_0.x(), p_3.y() - p_0.y());
        planning_math::Vec2d vec_p0_lw(left_rear_wheel.x() - p_0.x(),
                                       left_rear_wheel.y() - p_0.y());
        int direc = vec_p0_p3.CrossProd(vec_p0_lw) > 0 ? -1 : 1;
        double dist_lw_opening =
            direc *
            calculateMinDistOnLine(p_0, p_3, left_rear_wheel, ego_pose.theta);
        planning_math::Vec2d vec_p0_rw(right_rear_wheel.x() - p_0.x(),
                                       right_rear_wheel.y() - p_0.y());
        direc = vec_p0_p3.CrossProd(vec_p0_rw) > 0 ? -1 : 1;
        double dist_rw_opening =
            direc *
            calculateMinDistOnLine(p_0, p_3, right_rear_wheel, ego_pose.theta);

        double minor_dist_opening = std::min(dist_lw_opening, dist_rw_opening);
        PlanningContext::Instance()
            ->mutable_parking_behavior_planner_output()
            ->min_dist_to_opening =
            minor_dist_opening < 0 ? -1 : minor_dist_opening;

        // bottom
        planning_math::Vec2d p_1 = origin_corners[1];
        planning_math::Vec2d p_2 = origin_corners[2];
        planning_math::Vec2d vec_p1_p2(p_2.x() - p_1.x(), p_2.y() - p_1.y());
        planning_math::Vec2d vec_p1_lw(left_rear_wheel.x() - p_1.x(),
                                       left_rear_wheel.y() - p_1.y());
        direc = vec_p1_p2.CrossProd(vec_p1_lw) > 0 ? 1 : -1;
        double dist_lw_bottom =
            direc *
            calculateMinDistOnLine(p_1, p_2, left_rear_wheel, ego_pose.theta);
        planning_math::Vec2d vec_p1_rw(right_rear_wheel.x() - p_1.x(),
                                       right_rear_wheel.y() - p_1.y());
        direc = vec_p1_p2.CrossProd(vec_p1_rw) > 0 ? 1 : -1;
        double dist_rw_bottom =
            direc *
            calculateMinDistOnLine(p_1, p_2, right_rear_wheel, ego_pose.theta);

        double minor_dist_bottom = std::min(dist_lw_bottom, dist_rw_bottom);
        PlanningContext::Instance()
            ->mutable_parking_behavior_planner_output()
            ->min_dist_to_bottom = minor_dist_bottom;

        // stopper
        if (parking_slot_info.wheel_stop_info.vision_wheel_stop_available) {
          planning_math::Vec2d p_w1(
              parking_slot_info.wheel_stop_info.vision_point1.x,
              parking_slot_info.wheel_stop_info.vision_point1.y);
          planning_math::Vec2d p_w2(
              parking_slot_info.wheel_stop_info.vision_point2.x,
              parking_slot_info.wheel_stop_info.vision_point2.y);
          planning_math::Vec2d vec_pw1_pw2(p_w2.x() - p_w1.x(),
                                           p_w2.y() - p_w1.y());
          planning_math::Vec2d vec_pw1_lw(left_rear_wheel.x() - p_w1.x(),
                                          left_rear_wheel.y() - p_w1.y());
          direc = vec_pw1_pw2.CrossProd(vec_pw1_lw) > 0 ? 1 : -1;
          double dist_lw_stopper =
              direc * calculateMinDistOnLine(p_w1, p_w2, left_rear_wheel,
                                             ego_pose.theta);
          planning_math::Vec2d vec_pw1_rw(right_rear_wheel.x() - p_w1.x(),
                                          right_rear_wheel.y() - p_w1.y());
          direc = vec_pw1_pw2.CrossProd(vec_pw1_rw) > 0 ? 1 : -1;
          double dist_rw_stopper =
              direc * calculateMinDistOnLine(p_w1, p_w2, right_rear_wheel,
                                             ego_pose.theta);

          double minor_dist_stopper =
              std::min(dist_lw_stopper, dist_rw_stopper);
          PlanningContext::Instance()
              ->mutable_parking_behavior_planner_output()
              ->dist_to_wheel_stop = minor_dist_stopper;
        }
      }
    }
  }

  extra_json["dist_to_stopper"] = PlanningContext::Instance()
                                      ->parking_behavior_planner_output()
                                      .dist_to_wheel_stop;
  extra_json["min_dist_to_opening"] = PlanningContext::Instance()
                                      ->parking_behavior_planner_output()
                                      .min_dist_to_opening;
  extra_json["min_dist_to_bottom"] = PlanningContext::Instance()
                                      ->parking_behavior_planner_output()
                                      .min_dist_to_bottom;

  int special_slot_type = 0;
  if(parking_slot_info.type.value == ParkingSlotType::PARALLEL) {
    if(std::abs(parking_slot_iou) > 0.1) {
      if(parking_slot_info.special_slot_type == 1) {
        special_slot_type = 1;
      }
      if(parking_slot_info.special_slot_type == 2) {
        special_slot_type = 2;
      }
    }
  }
  if (PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->is_narrow_channel) {
    special_slot_type = 1;
  }
  extra_json["special_slot_type"] = special_slot_type;
  extra_json["lon_inflation"] = CarParams::GetInstance()->lon_inflation();

  // remaining distance
  ParkingLongitudinalBehaviorPlanner::compute_lead_obs_info(
      parking_slot_info,
      &PlanningContext::Instance()
           ->mutable_longitudinal_behavior_planner_output()
           ->remain_dist_info_,
      &PlanningContext::Instance()
           ->mutable_longitudinal_behavior_planner_output()
           ->is_need_pause_);

  auto &fs_point = PlanningContext::Instance()
                       ->longitudinal_behavior_planner_output()
                       .free_space;


  int perpendicular_bottom_line_type = 0;
  if (VehicleParam::Instance()->car_type == "C03") {
    if(parking_slot_info.type.value == ParkingSlotType::PERPENDICULAR) {
      if (parking_slot_info.bottom_line_type == 1 && parking_slot_iou > 0.25) {
        perpendicular_bottom_line_type = 1;
      }
    }
  }
  extra_json["bottom_line_type"] = perpendicular_bottom_line_type;

  extra_json["lead_free_space_type"] = (int)fs_point.type;

  const ParkingBehaviorPlannerOutput &parking_behavior_planner_output =
      PlanningContext::Instance()->parking_behavior_planner_output();
  const auto& sv_config2 = msquare::CarParams::GetInstance()->car_config.sv_config;
  if (!parking_behavior_planner_output.is_move_ready && sv_config2.use_sv_speed_generator) {
    PlanningContext::Instance()
      ->mutable_longitudinal_behavior_planner_output()
      ->remain_dist_info_.remaining_distance_ = 0.0;
  }

  extra_json["remaining_distance"] = mjson::Json(PlanningContext::Instance()
      ->mutable_longitudinal_behavior_planner_output()
      ->remain_dist_info_.remaining_distance_);
  const auto& planner_output = PlanningContext::Instance()->longitudinal_behavior_planner_output();
  extra_json["traj_length"] = mjson::Json(planner_output.traj_length);

  extra_json["has_wheel_stop"] =
      mjson::Json(parking_slot_info.wheel_stop_info.available);
  extra_json["approaching_wheel_stop"] =
      mjson::Json(PlanningContext::Instance()
                      ->parking_behavior_planner_output()
                      .approaching_wheel_stop);
  extra_json["planner_clac_duration"] =
      mjson::Json(PlanningContext::Instance()
                      ->openspace_motion_planner_output()
                      .planner_calc_duration);
  
  planning_control_interface.remain_traj = 
    PlanningContext::Instance()
                        ->longitudinal_behavior_planner_output()
                        .traj_length;
  // if (PlanningContext::Instance()->parking_behavior_planner_output().is_last_path
  //     && parking_slot_info.type.value == ParkingSlotType::PERPENDICULAR
  //     && planning_control_interface.is_static
  //     && planning_control_interface.remain_traj < 0.2) {
  //     MSD_LOG(ERROR, "advance to the end. set the remain to 0");
  //     planning_control_interface.remain_traj = 0.0;
  // }
  if (parking_slot_info.type.value == ParkingSlotType::PERPENDICULAR) {
    planning_control_interface.remain_traj = 
        std::min(planning_control_interface.remain_traj, 
        PlanningContext::Instance()
                        ->parking_behavior_planner_output()
                        .min_remain_distance);
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->min_remain_distance = 999.0;
    MSD_LOG(ERROR, "advance to the end. set the remain to 0"); 
  }

  MSD_LOG(ERROR, "set to control obs remain distance %f",
      PlanningContext::Instance()
      ->mutable_longitudinal_behavior_planner_output()
      ->remain_dist_info_.remaining_distance_ );
  
  if (PlanningContext::Instance()
      ->mutable_longitudinal_behavior_planner_output()
      ->remain_dist_info_.remaining_distance_ + CarParams::GetInstance()->lon_inflation_min < 0.5) {
    MSD_LOG(ERROR, "near to obs not extend");
    planning_control_interface.is_extended = false;
  }

  if (!planning_control_interface.is_extended) {
      planning_control_interface.is_use_traj_s_and_v = false;
      planning_control_interface.second_remain_traj = 0.0;
      planning_control_interface.second_traj_target_v = 0.0;
  }

  auto getTargetV = [&planning_result, this](double remain_s) {
    /** get second trajectory speed by margin speed limit*/
    bool is_reverse = (planning_result.gear == GearState::REVERSE);
    if (planning_result.second_traj_pose_array.size() < 1) {
      return 0.0;
    }
    std::vector<planning_math::Box2d> obs_boxes;
    std::vector<planning_math::Vec2d> obs_pts;
    for(const auto& box_obs:world_model_->obstacle_manager().get_obstacles().Items()){
      obs_boxes.emplace_back(box_obs->PerceptionBoundingBox());
    }
    for(const auto& p_obs:world_model_->obstacle_manager().get_points().Items()){
      obs_pts.emplace_back(p_obs->PerceptionBoundingBox().center());
    }
    auto line_obs = world_model_->obstacle_manager().get_lines().Items();
    auto & pillars = world_model_->obstacle_manager().get_pillars().Items();
    auto & road_borders = world_model_->obstacle_manager().get_road_borders().Items();
    auto & gates = world_model_->obstacle_manager().get_gates().Items();
    line_obs.insert(line_obs.end(), pillars.begin(), pillars.end());
    line_obs.insert(line_obs.end(), road_borders.begin(),road_borders.end());
    line_obs.insert(line_obs.end(), gates.begin(), gates.end());
    std::vector<msquare::planning_math::LineSegment2d> line_obs_final;
    for(const auto& l_obs:line_obs){
      line_obs_final.emplace_back(l_obs->PerceptionLine());
    }

    SpeedMarginPara speed_margin_para;
    speed_margin_para.init(is_reverse, false, obs_boxes, obs_pts, line_obs_final);
    MSD_LOG(ERROR, "the traj size is: %d, the curvature is:%d",
            planning_result.second_traj_pose_array.size(),
            planning_result.second_traj_curvature.size());
    SpeedMarginLimiter speed_margin_limiter(
      speed_margin_para,
      planning_result.second_traj_pose_array,
      planning_result.second_traj_curvature, false);
    speed_margin_limiter.filterSV();
    double target_v = 0;
    msquare::parking::SpeedRes temp_res;
    speed_margin_limiter.getV(remain_s, target_v, temp_res);
    MSD_LOG(ERROR, "the target v is: %f", target_v);
    return target_v;
    // speed_margin_limiter.getSegmentV(planning_result.second_traj_vel_array);
  };

  // double remain_length =  PlanningContext::Instance()
  //                         ->longitudinal_behavior_planner_output()
  //                         .traj_length;
  MSD_LOG(ERROR, "The extend is: %d, is_use_s_and_v: %d, remain dis is: %f",
      planning_control_interface.is_extended, planning_control_interface.is_use_traj_s_and_v,
      planning_control_interface.remain_traj);
  if (planning_control_interface.is_extended
      && !planning_control_interface.is_use_traj_s_and_v
      && planning_control_interface.remain_traj < 0.25) {
    if (planning_result.second_traj_pose_array.size() >= 2) {
      std::vector<Pose2D> traj_pose_array_ = planning_result.second_traj_pose_array;
      std::vector<float> traj_vel_array_ = planning_result.second_traj_vel_array;
      double second_traj_remain_s = planning_math::getRemainDistance(
          traj_pose_array_,traj_vel_array_, world_model_->get_ego_state().ego_pose);
      planning_control_interface.is_use_traj_s_and_v = true;
      planning_control_interface.second_remain_traj = second_traj_remain_s;
      planning_control_interface.second_traj_target_v =
          getTargetV(second_traj_remain_s);
      MSD_LOG(ERROR, "the remain s is: %f, the target v is:%f, the flag is:%d",
              second_traj_remain_s, planning_control_interface.second_traj_target_v,
              1);
    }
  }

  MSD_LOG(ERROR, "Current Gear is: %d", int(planning_result.gear));
  if (planning_result.is_finished_flag || planning_result.gear == GearState::PARK) {
    MSD_LOG(ERROR, "finished set is_extend to true");
    planning_control_interface.is_extended = false;
    planning_control_interface.first_gear = 1;
  }

  // if (PlanningContext::Instance()
  //         ->parking_behavior_planner_output()
  //         .parking_lot != nullptr) {
  //   Box2d slot_box = PlanningContext::Instance()
  //                         ->mutable_parking_behavior_planner_output()
  //                         ->parking_lot->getBox();
  //   const auto& ego_pose = world_model_->get_ego_state().ego_pose;
  //   bool is_ego_overlaps_lot =
  //       slot_box.IsPointIn(Vec2d(ego_pose.x, ego_pose.y));
  //   if (is_ego_overlaps_lot) {
  //     MSD_LOG(ERROR, "car is in slot not extend");
  //     planning_control_interface.is_extended = false;
  //   }
  // }
  if (planning_control_interface.is_extended == true) {
    planning_control_interface.extend_type = 0;
  } else {
    bool is_narrow_parallel_slot = false;
    if (is_narrow_parallel_slot) {
      planning_control_interface.extend_type = 1;
    } else {
      planning_control_interface.extend_type = 2;
    }
  }

  auto planning_control_interface_json = mjson::Json(mjson::Json::object());
  planning_control_interface_json["is_extended"] = 
      planning_control_interface.is_extended;
  planning_control_interface_json["extend_type"] = 
      planning_control_interface.extend_type;
  planning_control_interface_json["remain_traj"] = 
      planning_control_interface.remain_traj;
  planning_control_interface_json["second_remain_traj"] =
      planning_control_interface.second_remain_traj;
  planning_control_interface_json["second_traj_target_v"] =
      planning_control_interface.second_traj_target_v;
  planning_control_interface_json["is_use_traj_s_and_v"] =
      planning_control_interface.is_use_traj_s_and_v;
  planning_control_interface_json["first_gear"] =
      planning_control_interface.first_gear;
  planning_control_interface_json["second_gear"] = 
      planning_control_interface.second_gear;
  planning_control_interface_json["gear_change_index"] = 
      planning_control_interface.gear_change_index;
  extra_json["plan_param"] = planning_control_interface_json;
  MSD_LOG(ERROR, "is_extended:%d : extend_type: %d,remain_traj: %f; second_remain_traj: %f, second_traj_target_v:%f,is_use_traj_s_and_v: %d, first_gear: %d, second_gear %d, gear_change_index %d",
    planning_control_interface.is_extended,
    planning_control_interface.extend_type,
    planning_control_interface.remain_traj,
    planning_control_interface.second_remain_traj,
    planning_control_interface.second_traj_target_v,
    planning_control_interface.is_use_traj_s_and_v,
    planning_control_interface.first_gear,
    planning_control_interface.second_gear,
    planning_control_interface.gear_change_index);
  const std::string& speed_margin_debug_string 
      = PlanningContext::Instance()->longitudinal_behavior_planner_output().speed_margin_debug;
  extra_json["speed_margin_debug"] = speed_margin_debug_string;
  PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()->speed_margin_debug = "";


  auto vec_sl_points = PlanningContext::Instance()->vec_sl_points();
  mjson::Json::array v_arr;
  if (!vec_sl_points.empty() &&
      !vec_sl_points[0].empty()) {
      for (const auto& vec : vec_sl_points) {
        mjson::Json::array arr;
        for (const auto& p : vec) {
          auto obj = mjson::Json(mjson::Json::object());
          obj["s"] = p.first;
          obj["l"] = p.second;
          arr.push_back(obj);
        }
        v_arr.emplace_back(arr);
      }
      extra_json["vec_sl_points"] = v_arr;
  }

  double not_use_comfortable_min_s =
      msquare::CarParams::GetInstance()
          ->car_config.sv_config.not_use_comfortable_min_s;
  if (not_use_comfortable_min_s > 0.0 &&
      PlanningContext::Instance()
              ->mutable_longitudinal_behavior_planner_output()
              ->remain_dist_info_.remaining_distance_ <
          not_use_comfortable_min_s) {
      extra_json["is_comfortable_brake"] = false;
  }

  msg.extra.json = extra_json.dump();
  // std::cout << "The extend flag is: " << planning_control_interface.is_extended << std::endl;

  nlohmann::json plan_strategy_name_json;
  plan_strategy_name_json["behavior"] =
      PlanningContext::Instance()->parking_behavior_planner_output().behavior;

  std::string statemachine_str = PlanningContext::Instance()
                                     ->mutable_planning_status()
                                     ->statemachine_stringstream.str();
  PlanningContext::Instance()
      ->mutable_planning_status()
      ->statemachine_stringstream.reset(); // clear
  if (statemachine_str.find("enter") != std::string::npos) {
    plan_strategy_name_json["statemachine_stringstream"] = statemachine_str;
  }

  while (!sbp_debug_receiver_->empty()) {
    std::string sbp_debug{};
    auto ret = sbp_debug_receiver_->pop_oldest(sbp_debug);
    if (!ret) {
      continue;
    }
    sbp_debug_info_ += sbp_debug + ". ";
  }
  plan_strategy_name_json["sbp_debug"] = sbp_debug_info_;
  *PlanningContext::Instance()->mutable_planning_debug_info() += "\n[task info]" + sbp_debug_info_;
  sbp_debug_info_ = "";
  msg.meta.plan_strategy_name = plan_strategy_name_json.dump();

  return msg;
}

} // namespace parking

} // namespace msquare
