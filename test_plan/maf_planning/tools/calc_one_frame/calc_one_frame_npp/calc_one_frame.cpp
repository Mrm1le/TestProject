#include "calc_one_frame.h"
#include "interactive/interactive.h"

#include <maf_interface/maf_framework_status.h>
#include <maf_interface/maf_hdmap.h>
#include <maf_interface/maf_planning.h>
#include <maf_interface/maf_std.h>
#include <maf_interface/maf_system_manager.h>
#include <signal.h>
#include <stdio.h>

#include <atomic>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <chrono>
#include <cstdint>
#include <exception>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "absl/strings/str_format.h"
#include "closed_loop/closed_loop_tool.h"
#include "closed_loop/closed_loop_utils.h"
#include "common.h"
#include "maf_message_cache.h"
#include "planning_task_on_running.h"

namespace msquare {

static bool IS_APA_STARTED = false;

template <typename MsgFrames>
void copy_origin_to_new_frame(const uint32_t frame_id,
                              const MsgFrames &origin_msgs,
                              MsgFrames *new_msgs) {
  if (frame_id < new_msgs->frame_count()) {
    LOG(ERROR) << "frame_id(" << frame_id << ") < frame_count("
               << new_msgs->frame_count() << ")";
    return;
  }
  new_msgs->add_frame(frame_id);
  for (const auto &msg : origin_msgs.frame(frame_id)) {
    auto new_msg = new_msgs->add_message_to_last_frame();
    *new_msg->mutable_msg() = msg.msg();
    new_msg->set_timestamp(msg.timestamp());
    new_msg->set_frame_id(msg.frame_id());
  }
}

void add_empty_control_msg_to_new_frame(const uint32_t frame_id,
                                        msc::MafMessageCache *msg_cache) {
  for (uint32_t i = msg_cache->new_control_command_msgs().frame_count();
       i <= frame_id; i++) {
    msg_cache->mutable_new_control_command_msgs()->add_frame(i);
    for (const auto &egopose_msg :
         msg_cache->origin_mla_localization_msgs().frame(i)) {
      auto *control_msg = msg_cache->mutable_new_control_command_msgs()
                              ->add_message_to_last_frame();

      control_msg->set_frame_id(i);
      control_msg->set_timestamp(egopose_msg.timestamp());
      control_msg->mutable_msg()->wire_type_command.available = 0;
      control_msg->mutable_msg()->brake_command.available = 0;
      control_msg->mutable_msg()->gear_command.available = 0;
      control_msg->mutable_msg()->steering_command.available = 0;
      control_msg->mutable_msg()->throttle_command.available = 0;
      control_msg->mutable_msg()->aeb_command.available = 0;
      control_msg->mutable_msg()->vehicle_light_command.available = 0;
      control_msg->mutable_msg()->window_command.available = 0;
      control_msg->mutable_msg()->auxiliary_command.available = 0;
      control_msg->mutable_msg()->extra.available = 0;
      // auto info = mjson::Json(mjson::Json::object());
      // control_msg->mutable_msg()->extra.json = info.dump();
      control_msg->mutable_msg()->extra.json =
          "{\"intersection_direction\":0.0,\"distance_to_stop_line\":1000.0,\"enter_intersection_flag\"\
          :0.0,\"lane_change_direction\":0.0,\"lane_change_status\":0.0,\"car_type\":\"\"\
          ,\"type\":\"\",\"architecture\":\"\",\"circle_time\":20.9985,\"state_time\":0.0,\"\
          cond_1\":1.0,\"cond_2\":0.0,\"cond_3\":0.0,\"cond_4\":0.0,\"cond_5\":1.0,\"cond_6\"\
          :1.0,\"stopper_limit_timer\":0.0,\"current_acc_req\":-0.274,\"throttle_fb\":-0.274,\"\
          vel_wheel\":0.0,\"cloud_simulation_flag\":1.0,\"steering_angle_bias0_deg\":0.0,\"\
          curvature\":0.0,\"curv_factor\":0.2839,\"ref_curvature\":0.0,\"transport_delay_sec\"\
          :0.0,\"vel_ref\":0.0,\"acc_ref\":0.0,\"vel_bx\":0.0,\"vel_by\":0.0,\"vel_ego\":0.0,\"\
          vel_ego_real\":0.0,\"acc_ego\":0.0,\"acc_ego_real\":0.0,\"acc_wheel\":0.0,\"acc_measure_vel\"\
          :-0.0678,\"ang_vel_iz\":0.0,\"slope_acc\":0.0,\"steering_angle\":-1.8185,\"steering_angle_deg\"\
          :-104.1929,\"lon_enable\":0.0,\"lat_enable\":0.0,\"ldp_activated\":0.0,\"throttle_override_flag\"\
          :0.0,\"brake_override_flag\":0.0,\"eps_hand_success_timer\":0.0,\"pasuse_lat_ctrl\"\
          :0.0,\"pasuse_lon_ctrl\":0.0,\"measures_yaw\":6.267,\"controller_status\":0.0,\"\
          car_model_type\":8.0,\"intersection_flag\":2.0,\"auto_status_timer\":0.0,\"ctrl_function_mode\"\
          :3.0,\"lat_situation\":0.0,\"unp_scenario\":0.0,\"steering_angle_deg_p2p\":168.3932,\"\
          ref_generate_time\":0.0,\"condition_2\":0.0,\"condition_5\":0.0,\"condition_8\"\
          :0.0,\"replanning_flag_keep\":0.0,\"brake_counter\":0.0,\"dist_to_stopper\":2.0,\"\
          stopper_dist\":2.0,\"update_para_time\":0.0,\"update_time\":0.0,\"apa_finish_\"\
          :1.0,\"apa_finish_steer_final_value\":0.5236,\"raw_steering_angle_cmd\":0.5236,\"\
          steering_angle_rate_cmd\":0.0,\"dt1\":0.0,\"dt2\":0.0,\"sign\":0.0,\"chassis_break_stationary\"\
          :1.0,\"stoper_flag\":0.0,\"flag_break_stoper\":1.0,\"dist_break_stoper\":0.0,\"\
          pos_stoper_x\":0.0,\"pos_stoper_y\":0.0,\"heading_stoper\":0.0,\"dist_vel_disable\"\
          :0.18,\"reverse_flag\":0.0,\"remain_s_uss\":2.0,\"remain_s\":0.0,\"remain_s_plan\"\
          :0.0813,\"remain_s_plan_rt\":0.1348,\"replan_flag\":0.0,\"uss_replan_flag\":0.0,\"\
          vel_tgt_plan\":0.0,\"vel_measure\":0.0,\"vel_filtered\":0.0002,\"slip_time\":2.0,\"\
          current_status\":2.0,\"ibs_acc_mode\":2.0,\"standstill_type\":0.0,\"pos_err\":0.0,\"\
          pos_fdbk_out\":0.0,\"pos_raw_out\":0.0,\"pos_out\":0.0,\"raw_vel_cmd\":0.0579,\"\
          vel_cmd\":0.11,\"vel_err\":0.0491,\"vel_fdbk_out\":0.0749,\"vel_ffwd_out\":-0.0056,\"\
          steer_ffwd_out\":0.0286,\"vel_dist_out\":0.0,\"vel_raw_out\":0.0978,\"vel_out\"\
          :-0.28,\"dist_enter_flag\":0.0,\"dist_comp_acc\":0.0,\"vel_acc_out\":-0.28,\"parking_slot_iou\"\
          :-0.9993,\"is_path_new\":1.0,\"slot_type\":2.0,\"plan_pause_flag\":0.0,\"lat_err\"\
          :-0.0317,\"phi_err\":0.005,\"steering_angle_cmd\":0.5236,\"enter_detection_region\"\
          :0.0,\"perception_has_stopper\":0.0,\"impact_flag\":0.0,\"safe_stop_acc\":-0.2,\"\
          decel_flag\":0.0,\"osqp_solver_flag\":0.0,\"ctrl_osqp_mpc\":-0.1227,\"ctrl_osqp_mpc_dot\"\
          :0.0084,\"mpc_run_time_ms\":0.0,\"time_vec_mpc\":\"0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2.0,2.1,2.2,2.3,2.4,2.5\"\
          ,\"dx_ref_vec_mpc\":\"-0.1,-0.12,-0.15,-0.17,-0.2,-0.22,-0.25,-0.27,-0.3,-0.32,-0.35,-0.37,-0.4,-0.42,-0.45,-0.47,-0.5,-0.52,-0.55,-0.57,-0.6,-0.62,-0.65,-0.67,-0.7,-0.72\"\
          ,\"dy_ref_vec_mpc\":\"-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.03,-0.04,-0.04,-0.04,-0.04\"\
          ,\"vel_ref_vec_mpc\":\"-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25,-0.25\"\
          ,\"dphi_ref_vec_mpc\":\"0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005\"\
          ,\"dx_vec_mpc\":\"0.0,-0.03,-0.05,-0.07,-0.1,-0.12,-0.15,-0.17,-0.2,-0.22,-0.25,-0.27,-0.3,-0.32,-0.35,-0.37,-0.4,-0.42,-0.45,-0.47,-0.5,-0.52,-0.55,-0.57,-0.6,-0.62\"\
          ,\"dy_vec_mpc\":\"0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.01,-0.01\"\
          ,\"dphi_vec_mpc\":\"0.0,0.001,0.002,0.003,0.003,0.004,0.005,0.006,0.007,0.007,0.008,0.009,0.009,0.01,0.011,0.011,0.012,0.012,0.013,0.013,0.014,0.014,0.015,0.016,0.016,0.017\"\
          ,\"delta_angle_vec_mpc\":\"-0.1237,-0.1227,-0.121,-0.1188,-0.116,-0.113,-0.1097,-0.1063,-0.1028,-0.0994,-0.0961,-0.0929,-0.0899,-0.0872,-0.0847,-0.0826,-0.0807,-0.0791,-0.0778,-0.0768,-0.0761,-0.0756,-0.0753,-0.0752,-0.0751,-0.0751\"\
          ,\"delta_rate_vec_mpc\":\"0.01,0.017,0.023,0.027,0.031,0.033,0.034,0.035,0.034,0.033,0.032,0.03,0.027,0.025,0.022,0.019,0.016,0.013,0.01,0.007,0.005,0.003,0.001,0.0,0.0,0.0\"\
          ,\"q0\":30.0,\"q1\":30.0,\"r\":0.35,\"q0_base\":30.0,\"q1_base\":30.0,\"r_base\"\
          :0.35,\"mpc_curv_factor\":0.2839,\"mpc_init_dx\":0.0,\"mpc_init_dy\":0.0,\"mpc_init_dphi\"\
          :0.0,\"mpc_init_delta\":-0.1237,\"wheel_angle_min\":-0.6018,\"wheel_angle_max\"\
          :0.6018,\"wheel_angle_rate_limit\":0.514,\"s_proj\":0.0687,\"s_proj_pred\":0.1688,\"\
          ego_local_x\":-7.1525,\"ego_local_y\":-1.7304,\"path_size\":25.0,\"match_index\"\
          :13.0,\"pred_match_point_index\":14.0,\"startup_settle_timer\":10.0,\"start_flag\"\
          :1.0,\"startup_timer\":0.0,\"ibs_hold_flag\":1.0,\"is_last_path\":1.0,\"special_slot_type\"\
          :1.0,\"bottom_line_type\":0.0,\"vel_ctrl_KP_term\":0.0,\"vel_ctrl_KI_term\":0.0,\"\
          print_json_time\":0.0,\"ctrl_loop_time\":0.0,\"ctrl_interface_time\":0.0,\"final_accelaration\"\
          :-0.28,\"override_timer\":0.0,\"throttle\":0.0,\"brake\":-0.28,\"acc_mode_neg\"\
          :2.0,\"final_steer_angle_cmd\":-5.8196,\"brake_output\":-0.274,\"throttle_output\"\
          :-0.274}";
    }
  }
}

void set_new_frame_module_status_by_use_origin_bag(
    const uint32_t frame_id, msc::MafMessageCache *msg_cache) {
  copy_origin_to_new_frame(frame_id, msg_cache->origin_module_status_msgs(),
                           msg_cache->mutable_new_module_status_msgs());

  for (auto &module_status :
       msg_cache->mutable_new_module_status_msgs()->mutable_frame(frame_id)) {
    const auto frame_time_span = msg_cache->get_frame_time_span(frame_id);

    module_status.set_timestamp(frame_time_span.start_time_ns());
    const uint64_t timestamp = frame_time_span.start_time_ns();
    module_status.mutable_msg()->header.stamp = timestamp;
    module_status.mutable_msg()->timestamp_us = timestamp / MICRO_NS;
    module_status.mutable_msg()->module_type.value =
        maf_framework_status::ModuleType::ENDPOINT;
    module_status.mutable_msg()->status =
        maf_framework_status::ModuleStatus::STOP;
    module_status.mutable_msg()->detail_status =
        maf_framework_status::ModuleStatus::RUNNING_UNKNOWN_ERROR;
    module_status.mutable_msg()->latency_ms = 0;
    module_status.mutable_msg()->message = FPP_MODULE_STATUS_MSG_NOT_ON_AUTO;
  }
}

void set_new_frame_msg_by_use_origin_bag(const uint32_t frame_id,
                                         msc::MafMessageCache *msg_cache) {
  copy_origin_to_new_frame(frame_id, msg_cache->origin_mla_localization_msgs(),
                           msg_cache->mutable_new_mla_localization_msgs());

  add_empty_control_msg_to_new_frame(frame_id, msg_cache);

  copy_origin_to_new_frame(frame_id, msg_cache->origin_chassis_report_msgs(),
                           msg_cache->mutable_new_chassis_report_msgs());
  copy_origin_to_new_frame(frame_id, msg_cache->origin_wheel_report_msgs(),
                           msg_cache->mutable_new_wheel_report_msgs());
  copy_origin_to_new_frame(frame_id, msg_cache->origin_body_report_msgs(),
                           msg_cache->mutable_new_body_report_msgs());
  copy_origin_to_new_frame(
      frame_id, msg_cache->origin_perception_fusion_object_result_msgs(),
      msg_cache->mutable_new_perception_fusion_object_result_msgs());
  copy_origin_to_new_frame(
      frame_id, msg_cache->origin_perception_vision_lane_msgs(),
      msg_cache->mutable_new_perception_vision_lane_msgs());

  set_new_frame_module_status_by_use_origin_bag(frame_id, msg_cache);
}

void set_new_egopose_frame_msg_by_use_perfect_dynamics(
    const bool is_apa, const uint32_t frame_id,
    const ClosedLoopTool &closed_loop_tool, msc::MafMessageCache *msg_cache) {
  const auto frame_time_span = msg_cache->get_frame_time_span(frame_id);
  const auto &last_planning_msg =
      msg_cache->new_planning_msgs().messages().back().msg();
  const auto &origin_msgs = msg_cache->origin_mla_localization_msgs();
  auto *new_msgs = msg_cache->mutable_new_mla_localization_msgs();
  if (is_apa) {
    closed_loop_tool.gen_apa_perfect_dynamics_egopose_msgs(
        frame_id, frame_time_span, last_planning_msg, origin_msgs, new_msgs);
  } else {
    closed_loop_tool.gen_perfect_dynamics_egopose_msgs(
        frame_id, frame_time_span, last_planning_msg, origin_msgs, new_msgs);
  }
}

void set_new_control_frame_msg_by_use_perfect_dynamics(
    const uint32_t frame_id, const ClosedLoopTool &closed_loop_tool,
    msc::MafMessageCache *msg_cache) {
  const auto &last_planning_msg =
      msg_cache->new_planning_msgs().messages().back().msg();
  auto egopose_frame = msg_cache->new_mla_localization_msgs().frame(frame_id);
  auto *new_msgs = msg_cache->mutable_new_control_command_msgs();
  closed_loop_tool.gen_perfect_dynamics_control_msgs(
      frame_id, last_planning_msg, egopose_frame, new_msgs);
}

void set_new_chassis_frame_msg_by_use_perfect_dynamics(
    const uint32_t frame_id, const ClosedLoopTool &closed_loop_tool,
    msc::MafMessageCache *msg_cache) {
  const auto &last_planning_msg =
      msg_cache->new_planning_msgs().messages().back().msg();
  const auto &new_egopose_frame =
      msg_cache->new_mla_localization_msgs().frame(frame_id);
  auto *new_msgs = msg_cache->mutable_new_chassis_report_msgs();
  closed_loop_tool.gen_perfect_dynamics_chassis_msgs(
      frame_id, last_planning_msg, new_egopose_frame, new_msgs);
}

void set_new_wheel_frame_msg_by_use_perfect_dynamics(
    const uint32_t frame_id, const ClosedLoopTool &closed_loop_tool,
    msc::MafMessageCache *msg_cache) {
  const auto &new_egopose_frame =
      msg_cache->new_mla_localization_msgs().frame(frame_id);
  auto *new_msgs = msg_cache->mutable_new_wheel_report_msgs();
  closed_loop_tool.gen_perfect_dynamics_wheel_msgs(frame_id, new_egopose_frame,
                                                   new_msgs);
}

void set_new_body_frame_msg_by_use_perfect_dynamics(
    const uint32_t frame_id, const ClosedLoopTool &closed_loop_tool,
    msc::MafMessageCache *msg_cache) {
  const auto frame_time_span = msg_cache->get_frame_time_span(frame_id);
  const auto &last_planning =
      msg_cache->new_planning_msgs().messages().back().msg();
  auto *new_msgs = msg_cache->mutable_new_body_report_msgs();
  closed_loop_tool.gen_perfect_dynamics_body_msgs(frame_id, frame_time_span,
                                                  last_planning, new_msgs);
}

void set_new_fusion_frame_msg_by_use_perfect_dynamics(
    const uint32_t frame_id, const ClosedLoopTool &closed_loop_tool,
    msc::MafMessageCache *msg_cache) {
  copy_origin_to_new_frame(
      frame_id, msg_cache->origin_perception_fusion_object_result_msgs(),
      msg_cache->mutable_new_perception_fusion_object_result_msgs());

  const auto &new_egopose_msgs =
      msg_cache->new_mla_localization_msgs().messages();
  auto fusion_frame =
      msg_cache->mutable_new_perception_fusion_object_result_msgs()
          ->mutable_frame(frame_id);
  closed_loop_tool.set_perfect_dynamics_fusion_msgs(new_egopose_msgs,
                                                    &fusion_frame);
}

bool set_new_lane_frame_msg_by_use_perfect_dynamics(
    const bool is_ddld, const uint32_t frame_id,
    const ClosedLoopTool &closed_loop_tool, msc::MafMessageCache *msg_cache) {
  const auto &origin_msgs = msg_cache->origin_mla_localization_msgs();
  const auto &new_egopose_msgs =
      msg_cache->new_mla_localization_msgs().messages();
  auto *new_lane_msgs = msg_cache->mutable_new_perception_vision_lane_msgs();
  const auto &ori_lane_msgs = msg_cache->origin_perception_vision_lane_msgs();
  if (!closed_loop_tool.set_perfect_dynamics_lane_msgs_by_pos(
          frame_id, origin_msgs, new_egopose_msgs, ori_lane_msgs,
          new_lane_msgs)) {
    return false;
  }
  return true;
}

void add_module_status_msg_to_new_frame(const uint32_t frame_id,
                                        msc::MafMessageCache *msg_cache) {
  for (uint32_t i = msg_cache->new_module_status_msgs().frame_count();
       i <= frame_id; i++) {
    msg_cache->mutable_new_module_status_msgs()->add_frame(i);
    auto *module_status = msg_cache->mutable_new_module_status_msgs()
                              ->add_message_to_last_frame();
    module_status->set_frame_id(i);
    const auto frame_time_span = msg_cache->get_frame_time_span(frame_id);
    module_status->set_timestamp(frame_time_span.start_time_ns());
    const uint64_t timestamp = frame_time_span.start_time_ns();
    module_status->mutable_msg()->header.stamp = timestamp;
    module_status->mutable_msg()->timestamp_us = timestamp / MICRO_NS;
    module_status->mutable_msg()->module_type.value =
        maf_framework_status::ModuleType::ENDPOINT;
    module_status->mutable_msg()->status =
        maf_framework_status::ModuleStatus::RUNNING;
    module_status->mutable_msg()->detail_status =
        maf_framework_status::ModuleStatus::NONE;
    module_status->mutable_msg()->latency_ms = 0;
    module_status->mutable_msg()->message = FPP_MODULE_STATUS_MSG_ON_AUTO;
  }
}

bool set_new_frame_msg_by_use_perfect_dynamics(
    const GeneratePlanningResultOptions &options, const uint32_t frame_id,
    const ClosedLoopTool &closed_loop_tool, msc::MafMessageCache *msg_cache) {
  set_new_egopose_frame_msg_by_use_perfect_dynamics(
      options.is_apa, frame_id, closed_loop_tool, msg_cache);
  set_new_control_frame_msg_by_use_perfect_dynamics(frame_id, closed_loop_tool,
                                                    msg_cache);
  set_new_chassis_frame_msg_by_use_perfect_dynamics(frame_id, closed_loop_tool,
                                                    msg_cache);
  set_new_wheel_frame_msg_by_use_perfect_dynamics(frame_id, closed_loop_tool,
                                                  msg_cache);
  set_new_body_frame_msg_by_use_perfect_dynamics(frame_id, closed_loop_tool,
                                                 msg_cache);
  set_new_fusion_frame_msg_by_use_perfect_dynamics(frame_id, closed_loop_tool,
                                                   msg_cache);
  if (!set_new_lane_frame_msg_by_use_perfect_dynamics(
          options.is_ddld, frame_id, closed_loop_tool, msg_cache)) {
    return false;
  }
  add_module_status_msg_to_new_frame(frame_id, msg_cache);
  return true;
}

void process_ignore_env_car(const uint32_t frame_id,
                            msc::MafMessageCache *msg_cache) {
  auto fusion_frame =
      msg_cache->mutable_new_perception_fusion_object_result_msgs()
          ->mutable_frame(frame_id);
  for (auto &fusion_msg : fusion_frame) {
    fusion_msg.mutable_msg()->perception_fusion_objects_data.clear();
    fusion_msg.mutable_msg()->reserved_infos.clear();
  }

  auto prediction_frame =
      msg_cache->mutable_origin_prediction_result_msgs()->mutable_frame(
          frame_id);
  for (auto &prediction_msg : prediction_frame) {
    prediction_msg.mutable_msg()->object_prediction_data.clear();
  }
}

bool set_new_frame_msg(const GeneratePlanningResultOptions &options,
                       const bool on_auto, const uint32_t frame_id,
                       const ClosedLoopTool &closed_loop_tool,
                       msc::MafMessageCache *msg_cache) {
  bool ret = true;
  if (on_auto) {
    ret = set_new_frame_msg_by_use_perfect_dynamics(
        options, frame_id, closed_loop_tool, msg_cache);
  } else {
    set_new_frame_msg_by_use_origin_bag(frame_id, msg_cache);
    ret = true;
  }

  if (options.ignore_env_car) {
    process_ignore_env_car(frame_id, msg_cache);
  }

  return ret;
}

template <typename MsgFrame> uint8_t get_running_mode(const MsgFrame &msgs) {
  for (auto &msg : msgs) {
    return msg.msg().running_mode.value;
  }
  return maf_system_manager::RunningModeEnum::MANUAL_DRIVING;
}

std::string get_running_mode_name(const uint8_t running_mode) {
  switch (running_mode) {
  case maf_system_manager::RunningModeEnum::PILOT:
    return "HIGHWAY";
  case maf_system_manager::RunningModeEnum::PARKING:
    return "PARKING";
  case maf_system_manager::RunningModeEnum::MANUAL_DRIVING:
    return "MANUAL_DRIVING";
  case maf_system_manager::RunningModeEnum::PRODUCTION_LINE:
    return "PRODUCTION_LINE";
  case maf_system_manager::RunningModeEnum::MEB:
    return "MEB";
  case maf_system_manager::RunningModeEnum::RADS:
    return "RADS";
  }
  return "UNKNOWN RUNNING_MODE";
}

void closed_loop_feed_one_frame(
    const GeneratePlanningResultOptions &options, const uint32_t frame_id,
    const msc::MafMessageCache &msg_cache,
    PlanningTaskOnRunning *planning_task_on_running) {
  LOG(INFO) << "frame: <" << frame_id
            << "> -------------------------------------------";

  auto egopose_frame = msg_cache.new_mla_localization_msgs().frame(frame_id);
  planning_task_on_running->feed_egopose(egopose_frame);
  LOG(INFO) << "egopose msgs count: " << egopose_frame.size();

  auto fusion_frame =
      msg_cache.new_perception_fusion_object_result_msgs().frame(frame_id);
  planning_task_on_running->feed_fusion(fusion_frame);
  LOG(INFO) << "fusion msgs count: " << fusion_frame.size();

  auto roal_line_perception_frame =
      msg_cache.new_perception_vision_lane_msgs().frame(frame_id);
  planning_task_on_running->feed_road_line_perception(
      roal_line_perception_frame);
  LOG(INFO) << "roal_line_perception msgs count: "
            << roal_line_perception_frame.size();

  auto perception_fusion_object_fame =
      msg_cache.origin_perception_fusion_object_result_msgs().frame(frame_id);
  planning_task_on_running->feed_perception_fusion_object(
      perception_fusion_object_fame);
  LOG(INFO) << "perception_fusion_object msgs count: "
            << perception_fusion_object_fame.size();

  uint8_t running_mode = maf_system_manager::RunningModeEnum::MANUAL_DRIVING;
  if (options.is_apa) {
    running_mode = maf_system_manager::RunningModeEnum::PARKING;
  } else {
    auto control_cmd_frame =
        msg_cache.origin_module_control_cmd_request_msgs().frame(frame_id);
    uint8_t running_mode = get_running_mode(control_cmd_frame);
    planning_task_on_running->feed_planning_control_cmd(control_cmd_frame);
    LOG(INFO) << "control_cmd msgs count: " << control_cmd_frame.size();
  }
  LOG(INFO) << "running_mode: " << get_running_mode_name(running_mode);

  auto mla_imu_frame = msg_cache.origin_mla_imu_msgs().frame(frame_id);
  planning_task_on_running->feed_mla_imu(mla_imu_frame);
  LOG(INFO) << "mla_imu msgs count: " << mla_imu_frame.size();

  auto mpc_trajectory_frame =
      msg_cache.origin_mpc_trajectory_msgs().frame(frame_id);
  planning_task_on_running->feed_mpc_trajectory(mpc_trajectory_frame);
  LOG(INFO) << "mpc_trajectory msgs count: " << mpc_trajectory_frame.size();

  auto radar_perception_frame =
      msg_cache.origin_radar_perception_result_msgs().frame(frame_id);
  planning_task_on_running->feed_radar_perception_result(
      radar_perception_frame);
  LOG(INFO) << "radar_perception msgs count: " << radar_perception_frame.size();

  auto planning_request_frame =
      msg_cache.origin_sys_planning_request_msgs().frame(frame_id);
  planning_task_on_running->feed_planning_request(planning_request_frame);
  LOG(INFO) << "planning_request msgs count: " << planning_request_frame.size();

  if (running_mode == maf_system_manager::RunningModeEnum::PARKING) {
    auto world_model_scene_objects_frame =
        msg_cache.origin_world_model_scene_objects_msgs().frame(frame_id);
    planning_task_on_running->feed_world_model_scene_objects(
        world_model_scene_objects_frame);
    LOG(INFO) << "world_model_scene_objects msgs count: "
              << world_model_scene_objects_frame.size();

    auto fusion_uss_ground_lines_frame =
        msg_cache.origin_fusion_uss_ground_line_result_msgs().frame(frame_id);
    planning_task_on_running->feed_fusion_uss_ground_lines(
        fusion_uss_ground_lines_frame);
    LOG(INFO) << "fusion_uss_ground_lines msgs count: "
              << fusion_uss_ground_lines_frame.size();

    auto fusion_uss_frame =
        msg_cache.origin_ultrasonic_upa_report_msgs().frame(frame_id);
    planning_task_on_running->feed_fusion_uss(fusion_uss_frame);
    LOG(INFO) << "fusion_uss msgs count: " << fusion_uss_frame.size();

    auto fusion_parking_slot_frame =
        msg_cache.origin_fusion_parking_slot_msgs().frame(frame_id);
    planning_task_on_running->feed_fusion_parking_slot(
        fusion_parking_slot_frame);
    LOG(INFO) << "fusion_parking_slot msgs count: "
              << fusion_parking_slot_frame.size();
  } else {
    auto planning_reset_request_frame =
        msg_cache.origin_planning_reset_request_msgs().frame(frame_id);
    planning_task_on_running->feed_planning_reset_request(
        planning_reset_request_frame);
    LOG(INFO) << "planning_reset_request msgs count: "
              << planning_reset_request_frame.size();
  }

  auto chassis_frame = msg_cache.new_chassis_report_msgs().frame(frame_id);
  planning_task_on_running->feed_chassis(chassis_frame);
  LOG(INFO) << "chassis msgs count: " << chassis_frame.size();

  auto body_frame = msg_cache.new_body_report_msgs().frame(frame_id);
  planning_task_on_running->feed_body(body_frame);
  LOG(INFO) << "body msgs count: " << body_frame.size();

  auto wheel_frame = msg_cache.new_wheel_report_msgs().frame(frame_id);
  planning_task_on_running->feed_wheel(wheel_frame);
  LOG(INFO) << "wheel msgs count: " << wheel_frame.size();

  auto module_status_frame = msg_cache.new_module_status_msgs().frame(frame_id);
  planning_task_on_running->feed_module_status(module_status_frame);
  LOG(INFO) << "module_status msgs count: " << module_status_frame.size();

  auto traffic_light_frame =
      msg_cache.origin_traffic_light_perception_msgs().frame(frame_id);
  planning_task_on_running->feed_tfl(traffic_light_frame);
  LOG(INFO) << "traffic_light msgs count: " << traffic_light_frame.size();

  auto fusion_ground_line_frame =
      msg_cache.origin_fusion_ground_line_result_msgs().frame(frame_id);
  planning_task_on_running->feed_fusion_ground_line(fusion_ground_line_frame);
  LOG(INFO) << "fusion_ground_line msgs count: "
            << fusion_ground_line_frame.size();

  auto prediction_frame =
      msg_cache.origin_prediction_result_msgs().frame(frame_id);
  planning_task_on_running->feed_prediction(prediction_frame);
  LOG(INFO) << "prediction msgs count: " << prediction_frame.size();
}

bool check_on_auto(bool on_auto, uint32_t frame_id, int start_frame,
                   msc::MafMessageCache *msg_cache) {
  if (on_auto) {
    return true;
  }
  if (static_cast<int>(frame_id) <= start_frame || frame_id == 0) {
    return false;
  }
  // 只有当 start_frame 或者之后的 frame 产生了输出轨迹, 才能进入自动
  if (msg_cache->new_planning_msgs().frame_count() != frame_id) {
    return false;
  }
  if (msg_cache->new_planning_msgs().frame(frame_id - 1).empty()) {
    return false;
  }
  LOG(INFO) << "on auto frame: " << frame_id;
  return true;
}

void set_failed_to_enter_auto(const uint32_t start_frame,
                              msc::MafMessageCache *msg_cache) {
  for (uint32_t i = start_frame; i < msg_cache->origin_frame_count(); i++) {
    set_new_frame_msg_by_use_origin_bag(i, msg_cache);
  }
  const decltype(msg_cache->origin_mla_localization_msgs().frame(
      0))::MessageWrapper *last_egopose = nullptr;
  for (uint32_t i = 0; i < start_frame; i++) {
    const auto &frame = msg_cache->origin_mla_localization_msgs().frame(i);
    if (frame.empty()) {
      continue;
    }
    last_egopose = &(*(frame.end() - 1));
  }
  if (last_egopose != nullptr) {
    for (uint32_t i = start_frame; i < msg_cache->origin_frame_count(); i++) {
      auto frame =
          msg_cache->mutable_new_mla_localization_msgs()->mutable_frame(i);
      for (auto &msg : frame) {
        const uint64_t timestamp_ns = msg.msg().header.stamp;
        (*msg.mutable_msg()) = last_egopose->msg();
        msg.mutable_msg()->header.stamp = timestamp_ns;
        msg.set_timestamp(timestamp_ns);
      }
    }
  }
}

void set_exit_auto(const uint32_t start_frame,
                   msc::MafMessageCache *msg_cache) {
  for (uint32_t i = start_frame; i < msg_cache->origin_frame_count(); i++) {
    set_new_frame_msg_by_use_origin_bag(i, msg_cache);
  }
  const decltype(msg_cache->new_mla_localization_msgs().frame(
      0))::MessageWrapper *last_egopose = nullptr;
  for (uint32_t i = 0; i < start_frame; i++) {
    const auto &frame = msg_cache->new_mla_localization_msgs().frame(i);
    if (frame.empty()) {
      continue;
    }
    last_egopose = &(*(frame.end() - 1));
  }
  if (last_egopose != nullptr) {
    for (uint32_t i = start_frame; i < msg_cache->origin_frame_count(); i++) {
      auto frame =
          msg_cache->mutable_new_mla_localization_msgs()->mutable_frame(i);
      for (auto &msg : frame) {
        const uint64_t timestamp_ns = msg.msg().header.stamp;
        (*msg.mutable_msg()) = last_egopose->msg();
        msg.mutable_msg()->header.stamp = timestamp_ns;
        msg.set_timestamp(timestamp_ns);
      }
    }
  }
}

FppTaskResult check_planning_fail(const GeneratePlanningResultOptions &options,
                                  const bool on_auto,
                                  const uint32_t current_frame_id,
                                  msc::MafMessageCache *msg_cache) {
  if (on_auto) {
    if (static_cast<int>(current_frame_id -
                         msg_cache->new_planning_msgs().frame_count()) >
        MAX_FRAME_COUNT_NOT_ON_AUTO) {
      LOG(ERROR) << "no plan over " << (MAX_FRAME_COUNT_NOT_ON_AUTO / 10)
                 << " seconds";
      set_exit_auto(current_frame_id, msg_cache);
      return FppTaskResult::fail("no plan over 2 seconds.");
    }
  } else {
    if (static_cast<int>(current_frame_id) - options.switch_frame >
        MAX_FRAME_COUNT_NOT_ON_AUTO) {
      LOG(ERROR) << "failed to enter auto diriving over "
                 << (MAX_FRAME_COUNT_NOT_ON_AUTO / 10) << " seconds";
      set_failed_to_enter_auto(current_frame_id, msg_cache);
      return FppTaskResult::fail("failed to enter auto diriving.");
    }
  }
  return FppTaskResult::success("");
}

void save_processed_map(const uint32_t frame_id,
                        PlanningTaskOnRunning *planning_task_on_running,
                        msc::MafMessageCache *msg_cache) {
  LOG(INFO) << "save processed map";
  auto *worldmodel_map_receiver =
      planning_task_on_running->mutable_worldmodel_map_receiver();
  msg_cache->mutable_new_processed_map_msgs()->add_frame(frame_id);
  while (!worldmodel_map_receiver->empty()) {
    std::shared_ptr<maf_worldmodel::ProcessedMap> processed_map;
    bool ret = worldmodel_map_receiver->pop_oldest(processed_map);
    if (!ret) {
      break;
    }

    auto *new_processed_map_msg = msg_cache->mutable_new_processed_map_msgs()
                                      ->add_message_to_last_frame();
    new_processed_map_msg->set_frame_id(frame_id);
    new_processed_map_msg->set_timestamp(processed_map->header.stamp);
    *(new_processed_map_msg->mutable_msg()) = *processed_map;
  }
}

void save_fusion_objects(const uint32_t frame_id,
                         PlanningTaskOnRunning *planning_task_on_running,
                         msc::MafMessageCache *msg_cache) {

  LOG(INFO) << "save fusion objects";
  auto *worldmodel_objects_receiver =
      planning_task_on_running->mutable_worldmodel_objects_receiver();
  msg_cache->mutable_new_objects_interface_msgs()->add_frame(frame_id);
  while (!worldmodel_objects_receiver->empty()) {
    std::shared_ptr<maf_worldmodel::ObjectsInterface> object;
    bool ret = worldmodel_objects_receiver->pop_oldest(object);
    if (!ret) {
      break;
    }

    auto *new_objects_interface_msg =
        msg_cache->mutable_new_objects_interface_msgs()
            ->add_message_to_last_frame();
    new_objects_interface_msg->set_frame_id(frame_id);
    new_objects_interface_msg->set_timestamp(object->header.stamp);
    *(new_objects_interface_msg->mutable_msg()) = *object;
  }

  auto *worldmodel_parking_slot_info_receiver =
      planning_task_on_running->mutable_worldmodel_parking_slot_info_receiver();
  msg_cache->mutable_new_worldmodel_fusion_apa_msgs()->add_frame(frame_id);
  while (!worldmodel_parking_slot_info_receiver->empty()) {
    std::shared_ptr<maf_worldmodel::FusionAPA> object;
    bool ret = worldmodel_parking_slot_info_receiver->pop_oldest(object);
    if (!ret) {
      break;
    }

    auto *new_objects_interface_msg =
        msg_cache->mutable_new_worldmodel_fusion_apa_msgs()
            ->add_message_to_last_frame();
    new_objects_interface_msg->set_frame_id(frame_id);
    new_objects_interface_msg->set_timestamp(object->header.stamp);
    *(new_objects_interface_msg->mutable_msg()) = *object;
  }
}

FppTaskResult run_closed_loop(const GeneratePlanningResultOptions &options,
                              isim::Interactive *interactive,
                              std::atomic_int *input_frame_count,
                              msc::MafMessageCache *msg_cache,
                              PlanningTaskOnRunning *planning_task_on_running) {
  LOG(INFO) << "run closed loop";
  bool on_auto = false;
  ClosedLoopTool closed_loop_tool;
  LOG(INFO) << "start_frame:" << options.switch_frame;
  for (uint32_t i = 0; i < msg_cache->origin_frame_count(); i++) {
    if (interactive)
      interactive->frame_callback(msg_cache, i);
    on_auto = check_on_auto(on_auto, i, options.switch_frame, msg_cache);
    if (options.is_apa) {
      on_auto = on_auto && IS_APA_STARTED && options.is_closed_loop;
    } else {
      const auto res = check_planning_fail(options, on_auto, i, msg_cache);
      if (res.result_type != FppTaskResultType::FPP_SUCCESS) {
        return res;
      }
    }

    if (!set_new_frame_msg(options, on_auto, i, closed_loop_tool, msg_cache)) {
      LOG(WARNING) << "end play bag early";
      break;
    }

    closed_loop_feed_one_frame(options, i, *msg_cache,
                               planning_task_on_running);

    const auto frame_time_span = msg_cache->get_frame_time_span(i);
    planning_task_on_running->set_current_frame_planning_start_time(
        frame_time_span.end_time_ns() - FRAME_TIME_ADJUSTMENT_NS);

    if (static_cast<int>(i) == options.debug_frame) {
      raise(SIGSTOP);
      // in debug mode, process will stop at the frame to be
      // debuged, and wait for gdb command `continue' to resume
    }

    input_frame_count->store(static_cast<int>(i + 1));
    if (!options.is_apa) {
      // cp
      planning_task_on_running->mutable_ddmap_generator_task()->on_running();
      planning_task_on_running->mutable_fusion_object_task()->on_running();
      planning_task_on_running->mutable_planning_task()->on_running();
    } else {
      // apa
      planning_task_on_running->mutable_fusion_object_task()->on_running();
      planning_task_on_running->mutable_pec_task()->on_running();
      planning_task_on_running->mutable_parking_planning_task()->on_running();
      planning_task_on_running->mutable_sbp_planning_task()->on_running();
    }
    save_fusion_objects(i, planning_task_on_running, msg_cache);
    save_processed_map(i, planning_task_on_running, msg_cache);
    if (options.pause_frame >= 0 && i >= options.pause_frame) {
      fprintf(
          stderr,
          "-----> Paused at the end of frame %d/%d, Press Enter to Continue \n",
          (int)i, (int)msg_cache->origin_frame_count());
      getchar();
    }
  }
  return FppTaskResult::success("success");
}

FppTaskResult run_open_loop(const GeneratePlanningResultOptions &options,
                            std::atomic_bool *is_planning_generated,
                            std::atomic_int *input_frame_count,
                            msc::MafMessageCache *msg_cache,
                            PlanningTaskOnRunning *planning_task_on_running) {
  LOG(ERROR) << "run open loop is not supported";
  std::abort();
}

void set_planning_output_callback(
    const std::atomic_int &input_frame_count,
    std::atomic_bool *is_planning_generated, msc::MafMessageCache *msg_cache,
    PlanningTaskOnRunning *planning_task_on_running) {
  auto cb_func = [&input_frame_count, is_planning_generated,
                  msg_cache](const MSDPlanningOutputMeta &meta,
                             const maf_planning::Planning &planning_result,
                             const std::string &trigger_msg_id) {
    if (meta.succeed) {
      int frame_idx = input_frame_count.load() - 1;
      if (frame_idx <
          static_cast<int>(msg_cache->new_planning_msgs().frame_count())) {
        LOG(ERROR) << "frame_idx error";
        std::abort();
      }

      msg_cache->mutable_new_planning_msgs()->add_frame(frame_idx);
      auto *new_planning_msg =
          msg_cache->mutable_new_planning_msgs()->add_message_to_last_frame();
      new_planning_msg->set_frame_id(frame_idx);
      new_planning_msg->set_timestamp(planning_result.header.stamp);
      *(new_planning_msg->mutable_msg()) = planning_result;

      is_planning_generated->store(true);

      std::string error_info;
      auto extra_json =
          mjson::Json::parse(planning_result.extra.json, error_info);
      auto apa_status = extra_json["status"].string_value();
      auto apa_task = extra_json["task"].string_value();
      if (!IS_APA_STARTED) {
        if (apa_status == "RUNNING" && apa_task == "APA") {
          IS_APA_STARTED = true;
        }
      }

      LOG(ERROR) << "hujy status:" << apa_status << " task:" << apa_task
                 << " traj:" << planning_result.trajectory.path.size()
                 << " target_value:"
                 << planning_result.trajectory.velocity.target_value
                 << " IS_APA_STARTED:" << IS_APA_STARTED;

      static int32_t last_apa_plan_trajectory_len_ = 0;
      if (last_apa_plan_trajectory_len_ > 0 &&
          planning_result.trajectory.path.size() == 0) {
        const static std::string TASK_APA = "\"task\" : \"APA\"";
        const static std::string STATUS_RUNNING = "\"status\" : \"RUNNING\"";
        if ((planning_result.extra.available & planning_result.extra.JSON) &&
            (planning_result.extra.json.find(TASK_APA) != std::string::npos) &&
            (planning_result.extra.json.find(STATUS_RUNNING) !=
             std::string::npos)) {
          maf_framework_status::NodesStatus node_status;
          node_status.nodes_status.resize(1);
          node_status.header.stamp = MTIME()->timestamp().ns();
          node_status.nodes_status[0].timestamp_us = MTIME()->timestamp().us();
          static uint16_t APA_PLANNING_NO_TRAJECTORY = 0xB301;
          node_status.nodes_status[0].detail_status =
              static_cast<uint16_t>(APA_PLANNING_NO_TRAJECTORY);
          node_status.nodes_status[0].status =
              static_cast<uint16_t>(node_status::Status::RUNNING_ERROR);

          msg_cache->mutable_new_node_status_apa_planning_msgs()->add_frame(
              frame_idx);
          auto *new_msg = msg_cache->mutable_new_node_status_apa_planning_msgs()
                              ->add_message_to_last_frame();
          new_msg->set_frame_id(frame_idx);
          new_msg->set_timestamp(planning_result.header.stamp);
          *(new_msg->mutable_msg()) = node_status;
        }
      }
      last_apa_plan_trajectory_len_ = planning_result.trajectory.path.size();
    } else {
      is_planning_generated->store(false);
    }
  };

  std::function<void(const maf_planning::SBPRequest &sbp_request)>
      sbp_request_cb_func = [&input_frame_count, msg_cache](
                                const maf_planning::SBPRequest &sbp_request) {
        LOG(ERROR) << "SBPRequest cb";
        int frame_idx = input_frame_count.load() - 1;
        if (frame_idx >=
            static_cast<int>(msg_cache->new_sbp_request_msgs().frame_count())) {
          msg_cache->mutable_new_sbp_request_msgs()->add_frame(frame_idx);
        }

        auto *new_sbp_request_msg = msg_cache->mutable_new_sbp_request_msgs()
                                        ->add_message_to_last_frame();
        new_sbp_request_msg->set_frame_id(frame_idx);
        new_sbp_request_msg->set_timestamp(sbp_request.header.stamp);
        *(new_sbp_request_msg->mutable_msg()) = sbp_request;
      };

  if (planning_task_on_running->mutable_planning_task()) {
    planning_task_on_running->mutable_planning_task()->set_callback(cb_func);
  }
  if (planning_task_on_running->mutable_parking_planning_task()) {
    planning_task_on_running->mutable_parking_planning_task()->set_callback(
        cb_func);
    planning_task_on_running->mutable_parking_planning_task()->set_callback(
        sbp_request_cb_func);
  }
}

void set_planning_info_output_callback(
    const std::atomic_int &input_frame_count, msc::MafMessageCache *msg_cache,
    PlanningTaskOnRunning *planning_task_on_running) {
  auto cb_func = [&input_frame_count,
                  msg_cache](const maf_std::Header &planning_info) {
    int frame_idx = input_frame_count.load() - 1;
    if (frame_idx <
        static_cast<int>(msg_cache->new_planning_info_msgs().frame_count())) {
      LOG(ERROR) << "frame_idx error";
      std::abort();
    }

    msg_cache->mutable_new_planning_info_msgs()->add_frame(frame_idx);
    auto *new_planning_info_msg = msg_cache->mutable_new_planning_info_msgs()
                                      ->add_message_to_last_frame();
    new_planning_info_msg->set_frame_id(frame_idx);
    new_planning_info_msg->set_timestamp(planning_info.stamp);
    *(new_planning_info_msg->mutable_msg()) = planning_info;
  };

  std::function<void(
      const maf_framework_status::NodeStatus &planning_node_status)>
      nodestatus_cb_func =
          [](const maf_framework_status::NodeStatus &planning_node_status)
      -> void { // not work now
    LOG(ERROR) << "planning_node_status";
  };

  if (planning_task_on_running->mutable_planning_task()) {
    planning_task_on_running->mutable_planning_task()->set_callback(cb_func);
  }
  if (planning_task_on_running->mutable_parking_planning_task()) {
    planning_task_on_running->mutable_parking_planning_task()->set_callback(
        cb_func);
    planning_task_on_running->status_manager()->set_notifier_callback(
        nodestatus_cb_func); // not work now
  }
}

void print_options(const GeneratePlanningResultOptions &options) {
  LOG(INFO) << "max_frame_count: " << options.max_frame_count;
  LOG(INFO) << "is_pnp: " << options.is_pnp;
  LOG(INFO) << "is_apa: " << options.is_apa;
  LOG(INFO) << "debug_frame: " << options.debug_frame;
  LOG(INFO) << "is_closed_loop: " << options.is_closed_loop;
  LOG(INFO) << "start_frame: " << options.switch_frame;
  LOG(INFO) << "start_output_frame: " << options.start_output_frame;
  LOG(INFO) << "speed_limit: " << options.speed_limit;
  LOG(INFO) << "navi_time_distance: " << options.navi_time_distance;
  LOG(INFO) << "ignore_env_car: " << options.ignore_env_car;
  LOG(INFO) << "is_ddld: " << options.is_ddld;
  LOG(INFO) << "pause_frame: " << options.pause_frame;
}

void read_bag(const std::string &input_msg_bag_path,
              msc::MafMessageCache *msg_cache) {
  LOG(INFO) << "begin read bag: " << input_msg_bag_path;
  msg_cache->load_bag(input_msg_bag_path, is_mfbag(input_msg_bag_path));
  msg_cache->segement_msgs();
  LOG(INFO) << "end read bag: " << input_msg_bag_path;
}

void set_output_callbacks(const std::atomic_int &input_frame_count,
                          std::atomic_bool *is_planning_generated,
                          msc::MafMessageCache *msg_cache,
                          PlanningTaskOnRunning *planning_task_on_running) {
  set_planning_output_callback(input_frame_count, is_planning_generated,
                               msg_cache, planning_task_on_running);
  set_planning_info_output_callback(input_frame_count, msg_cache,
                                    planning_task_on_running);
}

float get_default_cruise_velocity(const msc::MafMessageCache &msg_cache) {
  for (const auto &msg : msg_cache.origin_planning_msgs().messages()) {
    float cruise_velocity = msg.msg().trajectory.velocity.cruise_velocity * 3.6;
    if (cruise_velocity > 0) {
      LOG(INFO) << "get cruise_velocity: " << cruise_velocity;
      return cruise_velocity;
    }
  }
  LOG(INFO) << "use default cruise_velocity: " << kDefaultSpeedLimit;
  return kDefaultSpeedLimit;
}

float get_chassis_report_speed_limit(const msc::MafMessageCache &msg_cache) {
  float max_speed = 0.0;
  for (auto &msg : msg_cache.origin_chassis_report_msgs().messages()) {
    float vehicle_speed = msg.msg()
                              .throttle_info_report.throttle_info_report_data
                              .vehicle_speed_average *
                          3.6;
    if (max_speed < vehicle_speed) {
      max_speed = vehicle_speed;
    }
  }
  if (max_speed > 0) {
    LOG(INFO) << "get max_speed: " << max_speed;
    return max_speed;
  }
  LOG(INFO) << "use default speed_limit: " << kDefaultSpeedLimit;
  return kDefaultSpeedLimit;
}

void set_planning_speed_limit(const GeneratePlanningResultOptions &options,
                              const msc::MafMessageCache &msg_cache,
                              PlanningTaskOnRunning *planning_task_on_running) {
  float speed_limit = kDefaultSpeedLimit;
  if (options.speed_limit > 0) {
    speed_limit = options.speed_limit;
  } else if (options.speed_limit == 0) {
    // 仿真从原 bag 中读取cruise_velocity (该速度一般为司机进入智驾时收到设置
    // 的车辆最大限速, 由一个config文件传入) 作为限速 (默认值)
    speed_limit = get_default_cruise_velocity(msg_cache);
  } else {
    // 仿真遍历原bag中的自车速度, 将最大速度作为限速 (一般用于专家场景)
    speed_limit = get_chassis_report_speed_limit(msg_cache);
  }

  LOG(INFO) << "speed_limit: " << speed_limit;
  if (speed_limit > 0) {
    LOG(INFO) << "set planning speed limit: " << speed_limit
              << ", navi_time_distance: " << options.navi_time_distance;

    std::array<
        msc::MessageWrapper<maf_system_manager::SysPlanningRequest, false>, 1>
        planning_requests{};
    planning_requests[0].mutable_msg()->cmd.value =
        maf_system_manager::SystemCmdTypeEnum::PLANNING_HIGHWAY_NAVI_SETTINGS;
    planning_requests[0]
        .mutable_msg()
        ->highway_info.navi_settings.navi_max_speed = speed_limit;
    planning_requests[0]
        .mutable_msg()
        ->highway_info.navi_settings.navi_time_distance.value =
        options.navi_time_distance;
    planning_task_on_running->feed_planning_request(planning_requests);
  }
}

void write_bag(const std::string &input_msg_bag_path,
               const std::string &planning_result_bag_path,
               const GeneratePlanningResultOptions &options,
               const msc::MafMessageCache &msg_cache) {
  LOG(INFO) << "begin write bag: " << planning_result_bag_path;
  if (is_mfbag(input_msg_bag_path)) {
    write_mfbag(planning_result_bag_path, msg_cache, options.is_closed_loop);
  } else {
    write_rosbag(planning_result_bag_path, msg_cache, options.is_closed_loop,
                 input_msg_bag_path, {});
  }
  LOG(INFO) << "end write bag";
}

static std::shared_ptr<isim::Interactive> interactive;
void handle_int(int sig) { interactive.reset(); }
FppTaskResult
generate_planning_result(const std::string &input_msg_bag_path,
                         const std::string &planning_result_bag_path,
                         const GeneratePlanningResultOptions &options) {
  try {
    print_options(options);
    PlanningTaskOnRunning planning_task_on_running(options.is_apa);
    planning_task_on_running.init();
    if (options.interactive) {
      signal(SIGINT, handle_int);
      interactive = std::make_shared<isim::Interactive>(
          options.interactive, options.start_time * 10, options.end_time * 10, &planning_task_on_running);
    }

    msc::MafMessageCache msg_cache{FRAME_TIME_SPAN_NS};
    read_bag(input_msg_bag_path, &msg_cache);

    std::atomic_bool is_planning_generated{false};
    std::atomic_int input_frame_count{0};
    set_output_callbacks(input_frame_count, &is_planning_generated, &msg_cache,
                         &planning_task_on_running);

    set_planning_speed_limit(options, msg_cache, &planning_task_on_running);
    planning_task_on_running.feed_scenario(options.is_pnp || options.is_apa);
    FppTaskResult res = FppTaskResult::success("success");
    res = run_closed_loop(options, interactive.get(), &input_frame_count,
                          &msg_cache, &planning_task_on_running);

    write_bag(input_msg_bag_path, planning_result_bag_path, options, msg_cache);
    return res;
  } catch (const std::exception &ex) {
    std::stringstream ss;
    ss << "error: " << ex.what();
    LOG(ERROR) << ss.str();
    return FppTaskResult::error(ss.str());
  } catch (...) {
    LOG(ERROR) << "unknown error";
    return FppTaskResult::error("unknown error");
  }
}

} // namespace msquare
