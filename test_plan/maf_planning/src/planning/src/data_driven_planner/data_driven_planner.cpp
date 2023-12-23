#include "data_driven_planner/data_driven_planner.h"
#include "common/planning_fail_tracer.h"
#include "data_driven_planner/common/ddp_config.h"
#include "data_driven_planner/common/ddp_debug_logger.h"
#include "data_driven_planner/common/model_result_manager.h"
// #include "data_driven_planner/common/obstacle_manager.h"
#include "data_driven_planner/common/planning_result_manager.h"
#include "data_driven_planner/models/ego_pose_manager.h"
#include "data_driven_planner/models/fusion_object_manager.h"
#include "data_driven_planner/models/vision_lane_manager.h"
// #include "data_driven_planner/models/hdmap_adapter.h"
#include "mjson/mjson.hpp"

namespace msquare {
namespace ddp {

DataDrivenPlanner::DataDrivenPlanner(
    const std::shared_ptr<WorldModel> &world_model)
    : world_model_(world_model) {
  // config
  auto config_file_dir =
      msquare::PlanningContext::Instance()->get_config_file_dir();
  auto ddp_config_json_file =
      config_file_dir + "ddp_configs/ddp_x86_config.json";
  //  auto hnp_function_config_json_file =
  //      config_file_dir + "function_configs/hnp_function_config.json";

#ifdef ENABLE_MDC_810_PLATFORM
  ddp_config_json_file =
      config_file_dir + "/ddp_configs/ddp_mdc_810_config.json";
#elif defined(ENABLE_MDC_610_LHS_PLATFORM) ||                                  \
    defined(ENABLE_MDC_610_WLS_PLATFORM)
  ddp_config_json_file =
      config_file_dir + "/ddp_configs/ddp_mdc_610_config.json";
#elif defined(ENABLE_XAVIER_MODEL)
  ddp_config_json_file =
      config_file_dir + "/ddp_configs/ddp_xavier_config.json";
#elif defined(ENABLE_XAVIER_SAFETY_MODEL)
  ddp_config_json_file =
      config_file_dir + "/ddp_configs/ddp_xavier_config.json";
#endif
  MSD_LOG(INFO, "[DataDrivenPlanner::init] ddp_config_json_file: %s",
          ddp_config_json_file.c_str());
  Json ddp_config_json;
  std::ifstream fin(ddp_config_json_file);
  fin >> ddp_config_json;
  fin.close();

  auto config_builder = std::make_shared<DdpConfigBuilder>(ddp_config_json);
  DdpContext::Instance()->set_config_builder(config_builder);
  config_ = config_builder->cast<DdpConfig>();

  //  MSD_LOG(INFO, "[DataDrivenPlanner::init] hnp_function_config_json_file:
  //  %s",
  //          hnp_function_config_json_file.c_str());
  //  Json hnp_function_config_json;
  //  std::ifstream hnp_fin(hnp_function_config_json_file);
  //  hnp_fin >> hnp_function_config_json;
  //  hnp_fin.close();

  //  auto hnp_function_config_builder =
  //      std::make_shared<DdpConfigBuilder>(hnp_function_config_json);
  //  DdpContext::Instance()->set_hnp_config_builder(hnp_function_config_builder);

  // set global flag
  DdpContext::Instance()->mutable_b_dagger() = config_.enable_dagger;
  MSD_LOG(INFO, "[DataDrivenPlanner::init] b_dagger: %d",
          DdpContext::Instance()->b_dagger());

  // init hdmap adaptor
  // HDmapAdapter::Instance()->init(world_model_);

  // init ddp managers
  ddp_model_ = std::make_shared<DdpModel>(config_builder);
  model_result_manager_ = std::make_shared<ModelResultManager>(
      world_model_, ddp_model_, config_.use_ddp_model_in_planning);
  DdpContext::Instance()->set_model_result_manager(model_result_manager_);

  // ego_state_ = std::make_shared<EgoState>(world_model_);
  // DdpContext::Instance()->set_ego_state(ego_state_);

  // virtual_lane_manager_ = std::make_shared<VirtualLaneManager>(world_model);
  // DdpContext::Instance()->set_virtual_lane_manager(virtual_lane_manager_);

  // obstacle_manager_ =
  //     std::make_shared<ObstacleManager>(config_builder, world_model);
  // DdpContext::Instance()->set_obstacle_manager(obstacle_manager_);

  // traffic_light_decider_ = std::make_shared<TrafficLightDecider>(
  //     config_builder, world_model, virtual_lane_manager_);
  // DdpContext::Instance()->set_traffic_light_decider(traffic_light_decider_);

  fusion_object_manager_ = std::make_shared<FusionObjectManager>();
  DdpContext::Instance()->set_fusion_object_manager(fusion_object_manager_);

  // traffic_light_info_manager_ =
  //     std::make_shared<TrafficLightInfoManager>(world_model_);
  // DdpContext::Instance()->set_traffic_light_info_manager(
  //     traffic_light_info_manager_);

  // reference_path_manager_ = std::make_shared<ReferencePathManager>();
  // DdpContext::Instance()->set_reference_path_manager(reference_path_manager_);

  planning_result_manager_ = std::make_shared<PlanningResultManager>();
  DdpContext::Instance()->set_planning_result_manager(planning_result_manager_);

  ego_pose_manager_ = std::make_shared<EgoPoseManager>();
  DdpContext::Instance()->set_ego_pose_manager(ego_pose_manager_);

  vision_lane_manager_ = std::make_shared<VisionLaneManager>();
  DdpContext::Instance()->set_vision_lane_manager(vision_lane_manager_);

  // init state machine
  // scenario_state_machine_ =
  //     std::make_shared<ScenarioStateMachine>(config_builder, world_model);
  // scenario_state_machine_->init();
  // DdpContext::Instance()->set_scenario_state_machine(scenario_state_machine_);
  // DdpContext::Instance()->set_vehicle_param(
  //     ConfigurationContext::Instance()->get_vehicle_param());

  // scenario_state_display_ =
  //     std::make_shared<ScenarioStateDisplay>(hnp_function_config_builder);
  // DdpContext::Instance()->set_scenario_state_display(scenario_state_display_);
}

bool DataDrivenPlanner::run() {
  // Step 1) clear info
  MSD_LOG(INFO, "[data_driven_planner] entering run");

  // auto config_file_dir =
  // msquare::PlanningContext::Instance()->get_config_file_dir(); auto
  // ddp_config_json_file = config_file_dir + "/ddp_configs/ddp_config.json";
  // MSD_LOG(INFO, "[DataDrivenPlanner::run] ddp_config_json_file: %s",
  // ddp_config_json_file.c_str());

  // auto config_builder = DdpContext::Instance()->config_builder();
  // config_ = config_builder->cast<DdpConfig>();
  // MSD_LOG(INFO, "[DataDrivenPlanner::run] b_dagger: %d",
  // DdpContext::Instance()->status_info().b_dagger);
  // DdpContext::Instance()->mutable_status_info().b_dagger =
  // config_.enable_dagger;
  MSD_LOG(INFO, "[DataDrivenPlanner::run] b_dagger: %d",
          DdpContext::Instance()->b_dagger());

  auto start_time = MTIME()->timestamp().ms();
  msquare::PlanningStatus *pnc_status =
      msquare::PlanningContext::Instance()->mutable_planning_status();
  msquare::PlanningResult &pnc_result = pnc_status->planning_result;
  DdpContext::Instance()->clear();
  DdpContext::Instance()->mutable_status_info().planning_loop += 1;
  DdpContext::Instance()->mutable_status_info().error_info = "DDP_OK";

  bool dbw_status = DdpContext::Instance()->status_info().vehicle_dbw_status;
  auto &mutable_fault_diagnosis_result =
      DdpContext::Instance()->mutable_fault_diagnosis_result();
  auto &fault_diagnosis_result =
      DdpContext::Instance()->fault_diagnosis_result();
  mutable_fault_diagnosis_result.pre_able_to_auto =
      dbw_status ? fault_diagnosis_result.able_to_auto : false;
  mutable_fault_diagnosis_result.able_to_auto = true;
  mutable_fault_diagnosis_result.fault_type = -1;

  // Step 2) update planning world
  model_result_manager_->update();
  // ego_state_->update();
  // if (!virtual_lane_manager_->update()) {
  //   DdpContext::Instance()->mutable_planning_success() = false;
  //   PLANNING_FAIL_TRACE("virtual_lane_manager update failed");
  //   return false;
  // }
  // obstacle_manager_->update();
  // // HACK (@Haowen) obstacle_manager_->update_virtual_obstacles() depends on
  // // virtual_lane_manager_->update()
  // obstacle_manager_->update_virtual_obstacles();
  // // HACK (@Haowen) reference_path_manager_->update() depends on
  // // update_virtual_obstacles();
  // reference_path_manager_->update();
  // // HACK (@Haowen) this operation depends on obstacle_manager_->update() and
  // // reference_path_manager_->update()
  // obstacle_manager_->assign_obstacles_to_lanes();

  // traffic_light_decider_->update();

  // auto updated_time = MTIME()->timestamp().ms();
  // MSD_LOG(INFO, "[data_driven_planner] update time:%f",
  //         updated_time - start_time);

  // // Step 3) update state machine
  // scenario_state_machine_->update();
  // auto state_machine_time = MTIME()->timestamp().ms();
  // MSD_LOG(INFO, "[data_driven_planner] state machine update time:%f",
  //         state_machine_time - updated_time);

  // // Step 4) copy planning result
  // auto &ddp_result = DdpContext::Instance()->mutable_planning_result();
  // // auto extra_json = mjson::Json(mjson::Json::object());
  // auto ddp_disabled = disable_data_driven_planning(ddp_result);

  // // Step 5) check vehicle state is steady to getin automode
  // auto c_lane =
  // reference_path_manager_->get_reference_path_by_current_lane(); if
  // (!fault_diagnosis_result.pre_able_to_auto && c_lane != nullptr) {
  //   double half_ego_width = DdpContext::Instance()->vehicle_param().width /
  //   2; auto ego_l = c_lane->get_frenet_ego_state().l(); auto ego_s =
  //   c_lane->get_frenet_ego_state().s(); ReferencePathPoint refpath_pt; auto
  //   success = c_lane->get_reference_point_by_lon(ego_s, refpath_pt); double
  //   min_dis_to_road_border = 10.0; double min_dis_to_lane_border = 10.0; if
  //   (success) {
  //     min_dis_to_road_border =
  //         ego_l < 0 ? ego_l + refpath_pt.distance_to_right_road_border
  //                   : refpath_pt.distance_to_left_road_border - ego_l;
  //     min_dis_to_lane_border =
  //         ego_l < 0 ? ego_l + refpath_pt.distance_to_right_lane_border
  //                   : refpath_pt.distance_to_left_lane_border - ego_l;
  //   }
  //   MSD_LOG(INFO,
  //           "[fault_diagnosis_result] dbw false, r_road_border: %f, "
  //           "l_road_border: %f, r_lane_border: %f, l_lane_border: %f",
  //           refpath_pt.distance_to_right_road_border,
  //           refpath_pt.distance_to_left_road_border,
  //           refpath_pt.distance_to_right_lane_border,
  //           refpath_pt.distance_to_left_lane_border);
  //   MSD_LOG(INFO,
  //           "[fault_diagnosis_result] dbw false, ego_l: %f, ego_s: %f, "
  //           "min_dis_to_road_border: %f, min_dis_to_lane_border: %f",
  //           ego_l, ego_s, min_dis_to_road_border, min_dis_to_lane_border);
  //   if (std::min(min_dis_to_road_border, min_dis_to_lane_border) <
  //       half_ego_width) {
  //     mutable_fault_diagnosis_result.able_to_auto = false;
  //     if (min_dis_to_road_border < half_ego_width) {
  //       MSD_LOG(INFO,
  //               "[fault_diagnosis_result] In start process: vehicle is close
  //               " "to road border");
  //       mutable_fault_diagnosis_result.fault_type =
  //           FaultDiagnosisType::BORDER_PRESSING_DRIVING;
  //     } else {
  //       MSD_LOG(INFO,
  //               "[fault_diagnosis_result] In start process: vehicle is close
  //               " "to lane border");
  //       mutable_fault_diagnosis_result.fault_type =
  //           FaultDiagnosisType::LINE_PRESSING_DRIVING;
  //     }
  //   }
  // }
  // bool able_to_auto = fault_diagnosis_result.able_to_auto;
  // MSD_LOG(INFO,
  //         "[fault_diagnosis_result] able_to_auto: %d, pre_able_to_auto: %d",
  //         able_to_auto, fault_diagnosis_result.pre_able_to_auto);
  // if (!able_to_auto) {
  //   DdpContext::Instance()->mutable_planning_success() = false;
  //   MSD_LOG(INFO,
  //           "[data_driven_planner] In start process: has fault diagnosis");
  //   return false;
  // }

  // // Step 6) check proposal match
  // bool has_valid_proposal{false};
  // const auto virtual_lanes =
  //     DdpContext::Instance()->virtual_lane_manager()->get_virtual_lanes();
  // for (size_t i = 0; i < virtual_lanes.size(); ++i) {
  //   if (virtual_lanes[i]->lc_proposal_id() > 0) {
  //     has_valid_proposal = true;
  //     break;
  //   }
  // }
  // if (ddp_disabled) {
  //   // extra_json["source"] = mjson::Json("PNC");
  //   ddp_result.extra_json_raw["source"] = mjson::Json("PNC");
  //   ddp_result.extra_json_raw["ddp_pipeline_failed"] = mjson::Json(true);
  //   ddp_result.extra_json_raw["npp_proposal_match_info"] =
  //       mjson::Json(has_valid_proposal);
  //   pnc_result.extra_json = ddp_result.extra_json_raw.dump();
  //   DdpContext::Instance()->set_last_ddp_planning_result(nullptr);
  //   MSD_LOG(ERROR, "[data_driven_planner] data_driven_planner disabled");
  // } else {
  //   // set ddp source flag
  //   set_planning_result(ddp_result, pnc_result);
  //   ddp_result.extra_json_raw["source"] = mjson::Json("DDP");
  //   ddp_result.extra_json_raw["ddp_pipeline_failed"] = mjson::Json(false);
  //   ddp_result.extra_json_raw["npp_proposal_match_info"] =
  //       mjson::Json(has_valid_proposal);
  //   pnc_result.extra_json = ddp_result.extra_json_raw.dump();
  //   DdpContext::Instance()->set_last_ddp_planning_result(
  //       std::make_shared<PlanningResult>(ddp_result));
  //   DdpContext::Instance()->planning_result_manager()->add_planning_result(
  //       ddp_result);
  //   MSD_LOG(INFO, "[data_driven_planner] data_driven_planner run success");
  // }

  // auto exit_time = MTIME()->timestamp().ms();
  // MSD_LOG(INFO, "[data_driven_planner] copy_result time:%f",
  //         exit_time - state_machine_time);
  // return (not ddp_disabled);
  return true;
}

// bool DataDrivenPlanner::disable_data_driven_planning(
//     const PlanningResult &ddp_result) {
//   bool b_dagger = DdpContext::Instance()->b_dagger();
//   MSD_LOG(INFO,
//           "[DataDrivenPlanner::disable_data_driven_planning] b_dagger: %d",
//           b_dagger);

//   // ddp planning failed
//   bool b_ddp_planning_failed = !DdpContext::Instance()->planning_success();

//   // final disable flag
//   bool b_disable_ddp = b_ddp_planning_failed;

//   // log disable info
//   if (b_disable_ddp) {
//     DdpContext::Instance()->mutable_status_info().error_info =
//         "DDP_DISABLE|fail|," + std::to_string(b_ddp_planning_failed) +
//         DdpContext::Instance()->status_info().error_info;
//     MSD_LOG(ERROR,
//             "[data_driven_planner] xuhaowen debug: DDP_DISABLE|fail|%d|%s",
//             b_ddp_planning_failed,
//             DdpContext::Instance()->status_info().error_info.c_str());
//   }
//   return b_disable_ddp;
// }

// void DataDrivenPlanner::set_planning_result(
//     const PlanningResult &ddp_result, msquare::PlanningResult &pnc_result) {
//   const auto &traj_points = ddp_result.traj_points;

//   clear_planning_result(pnc_result);
//   if (ddp_result.turn_signal == NO_CHANGE) {
//     pnc_result.turn_signal_cmd.value = maf_planning::TurnSignal::NONE;
//   } else if (ddp_result.turn_signal == LEFT_CHANGE) {
//     pnc_result.turn_signal_cmd.value = maf_planning::TurnSignal::LEFT;
//   } else {
//     pnc_result.turn_signal_cmd.value = maf_planning::TurnSignal::RIGHT;
//   }
//   MSD_LOG(INFO, "[DataDrivenPlanner::set_planning_result] turn_signal: %d",
//           (int)pnc_result.turn_signal_cmd.value);

//   // add stitcher trajectory
//   auto stitched_trajectory = TrajectoryStitcher::TransformToPublishedTraj(
//       DdpContext::Instance()->ego_state()->stitching_trajectory());
//   if (stitched_trajectory.traj_pose_array.size() > 1) {
//     pnc_result.traj_pose_array.insert(
//         pnc_result.traj_pose_array.begin(),
//         stitched_trajectory.traj_pose_array.begin(),
//         stitched_trajectory.traj_pose_array.end() - 1);
//     pnc_result.traj_vel_array.insert(
//         pnc_result.traj_vel_array.begin(),
//         stitched_trajectory.traj_vel_array.begin(),
//         stitched_trajectory.traj_vel_array.end() - 1);
//     pnc_result.traj_acceleration.insert(
//         pnc_result.traj_acceleration.begin(),
//         stitched_trajectory.traj_acceleration.begin(),
//         stitched_trajectory.traj_acceleration.end() - 1);
//   }

//   for (size_t i = 0; i < traj_points.size(); i++) {
//     // longitudinal_planning
//     const auto &pt = traj_points[i];
//     maf_planning::VelocityPoint point;
//     point.target_velocity = pt.v;
//     point.relative_time = pt.t;
//     point.distance = pt.s;
//     pnc_result.traj_vel_array.push_back(std::move(point));
//     pnc_result.traj_acceleration.push_back(pt.a);
//     // lateral_planning
//     maf_planning::PathPoint pose;
//     pose.position_enu.x = pt.x;
//     pose.position_enu.y = pt.y;
//     pose.heading_yaw = pt.heading_angle;
//     pose.curvature = pt.curvature;
//     pnc_result.traj_pose_array.push_back(std::move(pose));
//   }

//   for (size_t i = 0; i < pnc_result.traj_pose_array.size(); i++) {
//     pnc_result.traj_vel_array[i].distance = 0.0;
//     if (i > 0) {
//       pnc_result.traj_vel_array[i].distance =
//           pnc_result.traj_vel_array[i - 1].distance +
//           std::hypot(pnc_result.traj_pose_array[i].position_enu.x -
//                          pnc_result.traj_pose_array[i - 1].position_enu.x,
//                      pnc_result.traj_pose_array[i].position_enu.y -
//                          pnc_result.traj_pose_array[i - 1].position_enu.y);
//     }
//   }
// }

// void DataDrivenPlanner::clear_planning_result(
//     msquare::PlanningResult &pnc_result) {
//   pnc_result.traj_vel_array.clear();
//   pnc_result.traj_acceleration.clear();
//   pnc_result.traj_pose_array.clear();
// }

} // namespace ddp
} // namespace msquare
