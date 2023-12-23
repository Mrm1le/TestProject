#pragma once
#include <cmath>
#include <cstdlib>
#include <glog/logging.h>
#include <limits>
#include <maf_interface/maf_endpoint.h>
#include <maf_interface/maf_mla_localization.h>
#include <maf_interface/maf_perception_interface.h>
#include <maf_interface/maf_planning.h>
#include <memory>
#include <stdexcept>

#include "closed_loop/closed_loop_utils.h"
#include "common.h"
#include "common/math/line_segment2d.h"
#include "common/math/vec2d.h"
#include "geometry_tools/geometry_utils.h"
#include "maf_interface.h"
#include "maf_message_cache.h"
#include "planner/behavior_planner/deciders/apf_decider.h"
#include "pnc/define/geometry.h"
#include "sim_control/latitude.h"
#include "sim_control/lontitude.h"
#include "src/planning/include/common/utils/cartesian_coordinate_system.h"
#include "src/planning/include/common/utils/utils_math.hpp"
#include "yaml-cpp/yaml.h"

namespace msquare {
class ClosedLoopTool {
public:
  ClosedLoopTool() { load_vehicle_param(); }
  ~ClosedLoopTool() = default;

  template <typename EgoposeMsgFrames>
  void gen_perfect_dynamics_egopose_msgs(
      const uint32_t frame_id, const msc::FrameTimeSpan &frame_time_span,
      const maf_planning::Planning &last_planning_msg,
      const EgoposeMsgFrames &origin_msgs, EgoposeMsgFrames *new_msgs) const {
    const uint64_t last_planning_start_time_ns =
        (last_planning_msg.meta.plan_timestamp_us * MICRO_NS) +
        static_cast<uint64_t>(
            last_planning_msg.trajectory.velocity.vel_points.front()
                .relative_time *
            SECOND_NS);
    const uint64_t last_planning_end_time_ns =
        (last_planning_msg.meta.plan_timestamp_us * MICRO_NS) +
        static_cast<uint64_t>(
            last_planning_msg.trajectory.velocity.vel_points.back()
                .relative_time *
            SECOND_NS);
    const uint64_t egopose_start_timestamp_ns =
        std::max(frame_time_span.start_time_ns(), last_planning_start_time_ns);
    const uint64_t egopose_end_timestamp_ns =
        std::min(frame_time_span.end_time_ns(), last_planning_end_time_ns);
    new_msgs->add_frame(frame_id);
    if (egopose_start_timestamp_ns >= egopose_end_timestamp_ns) {
      LOG(WARNING) << "egopose start time >= end time";
    } else {
      const uint64_t kEgoposeTimestampIntervalNs =
          (egopose_end_timestamp_ns - egopose_start_timestamp_ns) /
          kEgoposeCountPerFrame;

      uint32_t header_seq_begin =
          new_msgs->messages().back().msg().header.seq + 1;
      int search_reference_egopose_heuristic_frame_id = -1;
      for (uint32_t i = 0; i < kEgoposeCountPerFrame; i++) {
        auto *loc_wrap = new_msgs->add_message_to_last_frame();
        loc_wrap->set_frame_id(frame_id);
        const uint64_t timestamp_ns =
            egopose_start_timestamp_ns + kEgoposeTimestampIntervalNs * (i + 1);
        loc_wrap->set_timestamp(timestamp_ns);

        auto *loc = loc_wrap->mutable_msg();
        set_egopose_all_field_unavailable(loc);
        loc->header.stamp = timestamp_ns;
        loc->header.seq = header_seq_begin + i;
        loc->meta.timestamp_us = timestamp_ns / MICRO_NS;

        loc->status.status_info.quality = 0;
        loc->status.status_info.extended = 0xffff;
        loc->status.status_info.type = 2;
        loc->status.status_info.common = 4;
        loc->status.available =
            maf_mla_localization::MLAStatus::MLA_STATUS_INFO;

        double ego_yaw = interpolate_mla(last_planning_msg, loc);
        const auto &reference_egopose = get_reference_egopose(
            search_reference_egopose_heuristic_frame_id, origin_msgs, *loc);
        search_reference_egopose_heuristic_frame_id =
            reference_egopose.frame_id();
        loc->transform.transform_llh_to_boot =
            reference_egopose.msg().transform.transform_llh_to_boot;
        loc->transform.available = reference_egopose.msg().transform.available;
        loc->position.position_local.z =
            reference_egopose.msg().position.position_local.z;
        set_quaternion_and_euler(ego_yaw, loc);
        set_position_global(loc);

        loc->angular_velocity.available = 0;
        loc->position_std.available = 0;
        loc->velocity_std.available = 0;
        loc->angular_velocity_std.available = 0;
        loc->orientation_std.available = 0;
        loc->acceleration_std.available = 0;
        loc->pose_detail.available = 0;
        loc->road_lane_id_info = {0, 0, 0, 0, 0};
        loc->reserved.available = 0;
      }
    }
    if (new_msgs->frame_count() != frame_id + 1) {
      LOG(ERROR) << "new_msgs->frame_count() != frame_id + 1";
      std::abort();
    }
  };

  static std::vector<planning_math::LineSegment2d>
  get_path_segments(const maf_planning::Planning &planning_msg) {
    std::vector<planning_math::LineSegment2d> segments{};
    const auto &path = planning_msg.trajectory.path;
    segments.reserve(path.size());
    for (uint32_t i = 0; i < planning_msg.trajectory.path.size() - 1; i++) {
      planning_math::Vec2d segment_start{path[i].position_enu.x,
                                         path[i].position_enu.y};
      planning_math::Vec2d segment_end{path[i + 1].position_enu.x,
                                       path[i + 1].position_enu.y};
      segments.emplace_back(segment_start, segment_end);
    }
    return segments;
  }

  static std::vector<planning_math::LineSegment2d>::const_iterator
  get_nearest_path_segments(
      const planning_math::Vec2d point,
      const std::vector<planning_math::LineSegment2d> &segments) {
    std::vector<planning_math::LineSegment2d>::const_iterator nearest_segment =
        segments.cbegin();
    double nearest_distance = std::numeric_limits<double>::max();
    for (auto it = segments.cbegin(); it != segments.cend(); ++it) {
      const double distance = it->DistanceTo(point);
      if (nearest_distance > distance) {
        nearest_distance = distance;
        nearest_segment = it;
      }
    }
    return nearest_segment;
  }

  struct ApaGeneratedEgopose {
    double x;
    double y;
    double yaw;
    uint64_t timestamp_ns;

    ApaGeneratedEgopose(const double x, const double y, const double yaw,
                        const uint64_t timestamp_ns)
        : x{x}, y{y}, yaw{yaw}, timestamp_ns{timestamp_ns} {}
  };

  template <typename EgoposeMsgFrames>
  void gen_apa_perfect_dynamics_egopose_msgs(
      const uint32_t frame_id, const msc::FrameTimeSpan &frame_time_span,
      const maf_planning::Planning &last_planning_msg,
      const EgoposeMsgFrames &origin_msgs, EgoposeMsgFrames *new_msgs) const {
    if (new_msgs->messages().empty()) {
      LOG(ERROR) << "no new egopose msg";
      throw std::runtime_error(
          "Failed to call gen_apa_perfect_dynamics_egopose_msgs, no new "
          "egopose msg");
    }

    new_msgs->add_frame(frame_id);
    const auto &last_egopose = new_msgs->messages().back().msg();
    const uint64_t egopose_interval_ns =
        (frame_time_span.end_time_ns() - frame_time_span.start_time_ns()) /
        kEgoposeCountPerFrame;
    if (last_planning_msg.trajectory.path.empty()) {
      LOG(INFO) << "planning path is empty";
      uint32_t header_seq_begin =
          new_msgs->messages().back().msg().header.seq + 1;
      for (uint32_t i = 0; i < kEgoposeCountPerFrame; i++) {
        auto *loc_wrap = new_msgs->add_message_to_last_frame();
        loc_wrap->set_frame_id(frame_id);
        const uint64_t timestamp_ns =
            frame_time_span.start_time_ns() + (egopose_interval_ns * (i + 1));
        loc_wrap->set_timestamp(timestamp_ns);
        auto *loc = loc_wrap->mutable_msg();
        *loc = last_egopose;
        loc->header.stamp = timestamp_ns;
        loc->header.seq = header_seq_begin;
        header_seq_begin++;
        loc->meta.timestamp_us = timestamp_ns / MICRO_NS;
      }

      return;
    }

    auto path_segments = get_path_segments(last_planning_msg);
    if (path_segments.empty()) {
      LOG(ERROR) << "path segments is empty";
      throw std::runtime_error(
          "Failed to call gen_apa_perfect_dynamics_egopose_msgs, path segments "
          "is empty");
    }

    const planning_math::Vec2d planning_init_point{
        last_egopose.position.position_local.x,
        last_egopose.position.position_local.y};
    auto current_segment =
        get_nearest_path_segments(planning_init_point, path_segments);
    const double egopose_interval_s =
        static_cast<double>(egopose_interval_ns) / SECOND_NS;
    const double velocity = last_planning_msg.trajectory.velocity.target_value;
    const double move_distance = egopose_interval_s * std::abs(velocity);

    std::vector<ApaGeneratedEgopose> generated_egoposes{};
    generated_egoposes.reserve(kEgoposeCountPerFrame);
    auto current_point = planning_init_point;
    for (uint32_t i = 0; i < kEgoposeCountPerFrame; i++) {
      double distance_to_segment_end =
          current_segment->length() -
          current_segment->ProjectOntoUnit(current_point);
      double distance_left = move_distance;
      while (distance_left > distance_to_segment_end) {
        ++current_segment;
        if (current_segment >= path_segments.end()) {
          break;
        }
        distance_left -= distance_to_segment_end;
        distance_to_segment_end = current_segment->length();
      }
      planning_math::Vec2d next_point{};
      if (current_segment >= path_segments.end()) {
        LOG(WARNING) << "reach path end";
        next_point = path_segments.back().end();
        current_segment = path_segments.cend() - 1;
      } else {
        next_point = current_segment->getPoint(
            current_segment->length() -
            (distance_to_segment_end - distance_left));
      }

      double yaw = last_egopose.orientation.euler_local.yaw;
      if (planning_math::Vec2d::CreateUnitVec2d(yaw).InnerProd(
              current_segment->unit_direction()) > 0) {
        yaw = current_segment->heading();
      } else {
        yaw = planning_math::LineSegment2d(current_segment->end(),
                                           current_segment->start())
                  .heading();
      }

      generated_egoposes.emplace_back(next_point.x(), next_point.y(), yaw,
                                      frame_time_span.start_time_ns() +
                                          (egopose_interval_ns * (i + 1)));
      current_point = next_point;
    }

    uint32_t header_seq_begin =
        new_msgs->messages().back().msg().header.seq + 1;
    for (const auto egopose : generated_egoposes) {
      auto *loc_wrap = new_msgs->add_message_to_last_frame();
      loc_wrap->set_frame_id(frame_id);
      loc_wrap->set_timestamp(egopose.timestamp_ns);
      auto *loc = loc_wrap->mutable_msg();
      set_egopose_all_field_unavailable(loc);
      loc->header.stamp = egopose.timestamp_ns;
      loc->header.seq = header_seq_begin;
      header_seq_begin++;
      loc->meta.timestamp_us = egopose.timestamp_ns / MICRO_NS;

      loc->position.position_local.x = egopose.x;
      loc->position.position_local.y = egopose.y;
      loc->position.position_local.z = last_egopose.position.position_local.z;
      loc->position.available |=
          maf_mla_localization::MLAPosition::MLA_POSITION_LOCAL;

      loc->velocity.velocity_local.vx = std::cos(egopose.yaw) * velocity;
      loc->velocity.velocity_local.vy = std::sin(egopose.yaw) * velocity;
      loc->velocity.velocity_local.vz = 0;
      loc->velocity.available |=
          maf_mla_localization::MLAVelocity::MLA_VEL_LOCAL;

      loc->orientation.euler_local.yaw = egopose.yaw;
      loc->orientation.quaternion_local =
          geometry::Euler2Quaternion(loc->orientation.euler_local);
      loc->orientation.available =
          maf_mla_localization::MLAOrientation::MLA_EULER_LOCAL |
          maf_mla_localization::MLAOrientation::MLA_QUATERNION_LOCAL;
    }
  }

  template <typename EgoposeMsgFrame, typename ControlMsgFrames>
  static void gen_perfect_dynamics_control_msgs(
      const uint32_t frame_id, const maf_planning::Planning &last_planning_msg,
      const EgoposeMsgFrame &egopose_frame, ControlMsgFrames *new_msgs) {
    new_msgs->add_frame(frame_id);
    for (const auto &egopose : egopose_frame) {
      auto *control_msg = new_msgs->add_message_to_last_frame();

      control_msg->set_frame_id(egopose.frame_id());
      control_msg->set_timestamp(egopose.timestamp());
      control_msg->mutable_msg()->wire_type_command.available = 0;
      control_msg->mutable_msg()->brake_command.available = 0;
      control_msg->mutable_msg()->gear_command.available = 0;
      control_msg->mutable_msg()->steering_command.available = 0;
      control_msg->mutable_msg()->throttle_command.available = 0;
      control_msg->mutable_msg()->aeb_command.available = 0;
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

      control_msg->mutable_msg()->vehicle_light_command.available = 1;
      control_msg->mutable_msg()
          ->vehicle_light_command.vehicle_light_command_data.turn_signal.value =
          last_planning_msg.turn_signal_command.turn_signal_data.value;
      control_msg->mutable_msg()
          ->vehicle_light_command.vehicle_light_command_data.timestamp_us =
          egopose.msg().meta.timestamp_us;
    }
  }

  template <typename BodyReportMsgFrames>
  static void
  gen_perfect_dynamics_body_msgs(const uint32_t frame_id,
                                 const msc::FrameTimeSpan &frame_time_span,
                                 const maf_planning::Planning &last_planning,
                                 BodyReportMsgFrames *new_msgs) {
    new_msgs->add_frame(frame_id);
    auto *body_wrap = new_msgs->add_message_to_last_frame();
    const uint64_t timestamp_ns = frame_time_span.start_time_ns();
    body_wrap->set_frame_id(frame_id);
    body_wrap->set_timestamp(timestamp_ns);
    auto *body = body_wrap->mutable_msg();
    body->meta.timestamp_us = timestamp_ns / MICRO_NS;

    body->vehicle_light_report.vehicle_light_report_data.turn_signal_type
        .value = last_planning.turn_signal_command.turn_signal_data.value;
    body->vehicle_light_report.vehicle_light_report_data.valid = true;
    body->vehicle_light_report.vehicle_light_report_data.timestamp_us =
        body->meta.timestamp_us;
    body->vehicle_light_report.available =
        maf_endpoint::VehicleLightReport::VEHICLE_LIGHT_REPORT_DATA;

    body->button_report.available = 0;
    body->door_report.available = 0;
    body->tire_report.available = 0;
    body->window_report.available = 0;
    body->wiper_report.available = 0;
    body->energy_remaining_report.available = 0;
    body->vehicle_alarm_report.available = 0;
    body->vehicle_extra_report.available = 0;
    body->human_protected_device_report.available = 0;
    body->auxiliary_report.available = 0;
    body->suspension_report.available = 0;
  };

  template <typename EgoposeMsgs, typename FusionMsgFrames>
  static void set_perfect_dynamics_fusion_msgs(const EgoposeMsgs &egopose_msgs,
                                               FusionMsgFrames *fusion_msgs) {
    for (auto &fusion : *fusion_msgs) {
      auto egopose = gen_fusion_reference_egopose(
          fusion.msg().meta.sensor_timestamp_us, egopose_msgs);
      if (egopose == nullptr) {
        LOG(ERROR) << "egopose == nullptr";
        continue;
      }
      process_fusion(*egopose, fusion.mutable_msg());
    }
  };

  static bool is_origin_egopose_perception_lane_valid(
      const maf_mla_localization::MLALocalization &origin_egopose,
      const maf_mla_localization::MLALocalization &new_egopose) {
    const double diff_square =
        square_sum(origin_egopose.position.position_local.x -
                       new_egopose.position.position_local.x,
                   origin_egopose.position.position_local.y -
                       new_egopose.position.position_local.y);
    return diff_square < (kMaxEgoposeFuzzyMatchingDiffForLanePerceptionMeters *
                          kMaxEgoposeFuzzyMatchingDiffForLanePerceptionMeters);
  }

  template <typename EgoposeMsgFrames, typename EgoposeMsgs,
            typename RoadLinePerceptionMsgFrames>
  static bool set_perfect_dynamics_lane_msgs_by_pos(
      const uint32_t frame_id, const EgoposeMsgFrames &ori_egopose_msgs,
      const EgoposeMsgs &new_egopose_msgs,
      const RoadLinePerceptionMsgFrames &ori_lane_msgs,
      RoadLinePerceptionMsgFrames *lane_msgs) {
    if (lane_msgs->messages().empty()) {
      return true;
    }
    size_t new_frame_begin = lane_msgs->mutable_messages()->size();
    uint32_t header_seq_begin =
        lane_msgs->messages().back().msg().header.seq + 1;
    int cnt = 0;
    lane_msgs->add_frame(frame_id);
    for (auto &lane : ori_lane_msgs.frame(frame_id)) {
      auto *lane_wrap = lane_msgs->add_message_to_last_frame();
      lane_wrap->set_frame_id(frame_id);
      lane_wrap->set_timestamp(lane.timestamp());

      auto *new_lane = lane_wrap->mutable_msg();
      new_lane->meta.sensor_timestamp_us = lane.msg().meta.sensor_timestamp_us;
      new_lane->header.stamp = lane.msg().header.stamp;
      new_lane->header.seq = header_seq_begin + cnt;
      cnt++;
      new_lane->lane_perception.available =
          lane.msg().lane_perception.available;
      new_lane->road_edge_perception.available =
          lane.msg().road_edge_perception.available;

      const auto &new_egopose = gen_fusion_reference_egopose(
          lane_wrap->msg().meta.sensor_timestamp_us, new_egopose_msgs);
      int search_reference_egopose_heuristic_frame_id = -1;
      const auto &reference_egopose =
          get_reference_egopose(search_reference_egopose_heuristic_frame_id,
                                ori_egopose_msgs, *new_egopose);

      if (!is_origin_egopose_perception_lane_valid(reference_egopose.msg(),
                                                   *new_egopose)) {
        LOG(WARNING) << "no valid perception_lane, frame: " << frame_id;
        return false;
      }

      const typename RoadLinePerceptionMsgFrames::MessageWrapperType
          *nearest_origin_lane = nullptr;

      size_t begin_index = 0;
      size_t end_index = ori_lane_msgs.message_count() - 1;
      while (begin_index != end_index) {
        double begin_diff =
            std::fabs(ori_lane_msgs.messages()[begin_index]
                              .msg()
                              .meta.sensor_timestamp_us /
                          1e6 -
                      reference_egopose.msg().meta.timestamp_us / 1e6);
        double end_diff = std::fabs(
            ori_lane_msgs.messages()[end_index].msg().meta.sensor_timestamp_us /
                1e6 -
            reference_egopose.msg().meta.timestamp_us / 1e6);
        if (begin_diff > end_diff) {
          begin_index = std::ceil((double)(begin_index + end_index) / 2.0);
        } else {
          end_index = std::floor((double)(begin_index + end_index) / 2.0);
        }
      }
      nearest_origin_lane = &(ori_lane_msgs.messages()[begin_index]);

      if (nearest_origin_lane != nullptr)
        process_vision_lane_by_pos(reference_egopose.msg(), *new_egopose,
                                   nearest_origin_lane->msg(),
                                   lane_wrap->mutable_msg());
    }
    if (lane_msgs->frame_count() != frame_id + 1) {
      LOG(ERROR) << "new_msgs->frame_count() != frame_id + 1";
      std::abort();
    }
    return true;
  }

  template <typename EgoposeMsgFrame, typename ChassisMsgFrames>
  void gen_perfect_dynamics_chassis_msgs(
      const uint32_t frame_id, const maf_planning::Planning &last_planning_msg,
      const EgoposeMsgFrame &egopose_frame, ChassisMsgFrames *new_msgs) const {
    new_msgs->add_frame(frame_id);
    for (const auto &egopose_msg : egopose_frame) {
      auto *chassis_wrap = new_msgs->add_message_to_last_frame();
      const uint64_t timestamp_ns = egopose_msg.timestamp();
      chassis_wrap->set_frame_id(frame_id);
      chassis_wrap->set_timestamp(timestamp_ns);

      auto *chassis = chassis_wrap->mutable_msg();
      chassis->header.stamp = timestamp_ns;
      chassis->meta.timestamp_us = timestamp_ns / MICRO_NS;

      const auto &velocity = egopose_msg.msg().velocity.velocity_local;
      const double velocity_abs = std::hypot(velocity.vx, velocity.vy);
      chassis->throttle_info_report.throttle_info_report_data
          .vehicle_speed_average = velocity_abs;
      chassis->throttle_info_report.available =
          maf_endpoint::ThrottleInfoReport::THROTTLE_INFO_REPORT_DATA;

      const double VEHCILE_VELOCITY_IS_STATIC_TH = 0.001;
      const int ASSUME_VEHICLE_IS_STATIC_COUNT_TH = 10;

      static int assume_vechile_is_static_count = 0;
      if (velocity_abs < VEHCILE_VELOCITY_IS_STATIC_TH) {
        if(assume_vechile_is_static_count < ASSUME_VEHICLE_IS_STATIC_COUNT_TH){
          assume_vechile_is_static_count++;
        }
      } else {
        assume_vechile_is_static_count = 0;
      }
      chassis->brake_info_report.brake_info_report_data.stationary =
          assume_vechile_is_static_count >= ASSUME_VEHICLE_IS_STATIC_COUNT_TH;
      const auto &acceleration =
          egopose_msg.msg().acceleration.acceleration_local;
      chassis->brake_info_report.brake_info_report_data.accleration_on_wheel =
          ((acceleration.ax * velocity.vx + acceleration.ay * velocity.vy) >= 0
               ? 1
               : -1) *
          std::hypot(acceleration.ax, acceleration.ay);
      chassis->brake_info_report.available =
          maf_endpoint::BrakeInfoReport::BRAKE_INFO_REPORT_DATA;

      // 用未来100ms的曲率，一个点25ms，第4个点
      const uint32_t fourth_point_idx = 3;
      if (last_planning_msg.trajectory.path.size() > fourth_point_idx) {
        control::calc_steer_ratio(
            last_planning_msg.trajectory.path[fourth_point_idx].curvature,
            vehicle_param_, chassis);
      }
      chassis->steering_report.steering_report_data.override = false;

      control::calc_throttle_and_pedal(
          chassis->brake_info_report.brake_info_report_data
              .accleration_on_wheel,
          chassis);
      chassis->throttle_report.throttle_report_data.override = false;
      chassis->throttle_report.available =
          maf_endpoint::ThrottleReport::THROTTLE_REPORT_DATA;

      chassis->gear_report.gear_report_data.current_state.value =
          maf_endpoint::Gear::DRIVE;
      if (last_planning_msg.gear_command.available &
          maf_planning::GearCommand::GEAR_DATA != 0) {
        chassis->gear_report.gear_report_data.current_state.value =
            last_planning_msg.gear_command.gear_data.value;
      }

      chassis->gear_report.available =
          maf_endpoint::GearReport::GEAR_REPORT_DATA;

      chassis->epb_report.available = 0;
    }
  };

  template <typename EgoposeMsgFrame, typename WheelMsgFrames>
  void gen_perfect_dynamics_wheel_msgs(const uint32_t frame_id,
                                       const EgoposeMsgFrame &egopose_frame,
                                       WheelMsgFrames *new_msgs) const {
    new_msgs->add_frame(frame_id);
    for (const auto &egopose_msg : egopose_frame) {
      auto *wheel_wrap = new_msgs->add_message_to_last_frame();
      const uint64_t timestamp_ns = egopose_msg.timestamp();
      wheel_wrap->set_frame_id(frame_id);
      wheel_wrap->set_timestamp(timestamp_ns);

      auto *wheel = wheel_wrap->mutable_msg();
      wheel->header.stamp = timestamp_ns;
      wheel->meta.timestamp_us = timestamp_ns / MICRO_NS;

      const double vx = egopose_msg.msg().velocity.velocity_local.vx;
      const double vy = egopose_msg.msg().velocity.velocity_local.vy;
      const float aver_speed = std::hypot(vx, vy);
      wheel->wheel_speed_report.wheel_speed_report_data.front_left_mps =
          aver_speed;
      wheel->wheel_speed_report.wheel_speed_report_data.front_left =
          aver_speed / vehicle_param_.wheel_radius_front_left;
      wheel->wheel_speed_report.wheel_speed_report_data.front_right_mps =
          aver_speed;
      wheel->wheel_speed_report.wheel_speed_report_data.front_right =
          aver_speed / vehicle_param_.wheel_radius_front_right;
      wheel->wheel_speed_report.wheel_speed_report_data.rear_left_mps =
          aver_speed;
      wheel->wheel_speed_report.wheel_speed_report_data.rear_left =
          aver_speed / vehicle_param_.wheel_radius_rear_left;
      wheel->wheel_speed_report.wheel_speed_report_data.rear_right_mps =
          aver_speed;
      wheel->wheel_speed_report.wheel_speed_report_data.rear_right =
          aver_speed / vehicle_param_.wheel_radius_rear_right;
      wheel->wheel_speed_report.wheel_speed_report_data.timestamp_us =
          wheel->meta.timestamp_us;
      wheel->wheel_speed_report.available =
          maf_endpoint::WheelSpeedReport::WHEEL_SPEED_REPORT_DATA;

      wheel->wheel_position_report.available = 0;
      wheel->wheel_angle_report.available = 0;
      wheel->vehicle_imu_report.available = 0;
    }
  }

private:
  static std::string get_vehical_param_dir() {
    auto car_lib_dir = std::getenv("CALIB_DIR");
    if (car_lib_dir && *car_lib_dir) {
      return car_lib_dir;
    } else {
      auto which_car_ = std::getenv("WHICH_CAR");
      std::string carlib_dir = std::getenv("CAM_CALIB_DIR");
      return carlib_dir + "/" + which_car_;
    }
  }

  void load_vehicle_param() {
    std::string vehicle_param_path = get_vehical_param_dir() + "/vehicle.yaml";
    LOG(INFO) << "load_vehicle_param:  " << vehicle_param_path;

    YAML::Node yaml_config = YAML::LoadFile(vehicle_param_path);

    vehicle_param_.vehicle_length =
        yaml_config["param"]["vehicle_length"].as<double>();
    vehicle_param_.distance_from_rear_bumper_to_rear_axle =
        yaml_config["param"]["distance_from_rear_bumper_to_rear_axle"]
            .as<double>();
    vehicle_param_.steering_angle_ratio =
        yaml_config["param"]["steering_angle_ratio"].as<double>();
    vehicle_param_.wheel_base_distance =
        yaml_config["param"]["wheel_base_distance"].as<double>();

    vehicle_param_.wheel_radius_front_left =
        yaml_config["param"]["wheel_radius_front_left"].as<double>();
    vehicle_param_.wheel_radius_front_right =
        yaml_config["param"]["wheel_radius_front_right"].as<double>();
    vehicle_param_.wheel_radius_rear_left =
        yaml_config["param"]["wheel_radius_rear_left"].as<double>();
    vehicle_param_.wheel_radius_rear_right =
        yaml_config["param"]["wheel_radius_rear_right"].as<double>();
  }

  static void set_egopose_all_field_unavailable(
      maf_mla_localization::MLALocalization *loc) {
    loc->position.available = 0;
    loc->velocity.available = 0;
    loc->angular_velocity.available = 0;
    loc->orientation.available = 0;
    loc->acceleration.available = 0;
    loc->position_std.available = 0;
    loc->velocity_std.available = 0;
    loc->angular_velocity_std.available = 0;
    loc->orientation_std.available = 0;
    loc->acceleration_std.available = 0;
    loc->status.available = 0;
    loc->transform.available = 0;
    loc->pose_detail.available = 0;
    loc->reserved.available = 0;
  }

  double interpolate_mla(const maf_planning::Planning &last_planning_msg,
                         maf_mla_localization::MLALocalization *loc) const {
    if (last_planning_msg.trajectory.path.empty()) {
      LOG(ERROR) << "last_planning_msg.trajectory.path is empty";
      std::abort();
    }
    if (last_planning_msg.trajectory.velocity.vel_points.empty()) {
      LOG(ERROR) << "last_planning_msg.trajectory.velocity.vel_points is empty";
      std::abort();
    }
    if (last_planning_msg.trajectory.acceleration.acc_points.empty()) {
      LOG(ERROR)
          << "last_planning_msg.trajectory.acceleration.acc_points is empty";
      std::abort();
    }

    const double egopose_timestamp_us = loc->meta.timestamp_us;
    const double last_planning_start_time_us =
        static_cast<double>(last_planning_msg.meta.plan_timestamp_us) +
        (last_planning_msg.trajectory.velocity.vel_points.front()
             .relative_time *
         static_cast<double>(SECOND_US));
    const double kPlanningPointIntervalUs = 25 * MILLI_US;
    uint32_t left_nearest_planning_point_idx = static_cast<uint32_t>(
        (egopose_timestamp_us - last_planning_start_time_us) /
        kPlanningPointIntervalUs);
    uint32_t right_nearest_planning_point_idx =
        left_nearest_planning_point_idx + 1;

    if (left_nearest_planning_point_idx >=
        last_planning_msg.trajectory.path.size()) {
      LOG(WARNING) << "left_nearest_planning_point_idx:"
                   << left_nearest_planning_point_idx
                   << " >= "
                      "last_planning_msg.trajectory.path.size():"
                   << last_planning_msg.trajectory.path.size();
      left_nearest_planning_point_idx =
          last_planning_msg.trajectory.path.size() - 1;
    }
    if (right_nearest_planning_point_idx >=
        last_planning_msg.trajectory.path.size()) {
      LOG(WARNING) << "right_nearest_planning_point_idx:"
                   << right_nearest_planning_point_idx
                   << " >= "
                      "last_planning_msg.trajectory.path.size():"
                   << last_planning_msg.trajectory.path.size();
      right_nearest_planning_point_idx =
          last_planning_msg.trajectory.path.size() - 1;
    }

    const double left_nearest_planning_point_time_us =
        last_planning_start_time_us +
        left_nearest_planning_point_idx * kPlanningPointIntervalUs;
    const double right_nearest_planning_point_time_us =
        left_nearest_planning_point_time_us + kPlanningPointIntervalUs;

    double yaw_left =
        last_planning_msg.trajectory.path[left_nearest_planning_point_idx]
            .heading_yaw;
    double yaw_right =
        last_planning_msg.trajectory.path[right_nearest_planning_point_idx]
            .heading_yaw;
    // yaw角取值范围为[-PI, PI], 需处理PI到-PI的跳变
    if (yaw_left > yaw_right && yaw_left - yaw_right > PI) {
      yaw_right += 2 * PI;
    } else if (yaw_right > yaw_left && yaw_right - yaw_left > PI) {
      yaw_left += 2 * PI;
    }
    double ego_yaw = interpolate(left_nearest_planning_point_time_us, yaw_left,
                                 right_nearest_planning_point_time_us,
                                 yaw_right, egopose_timestamp_us);
    while (ego_yaw > PI) {
      ego_yaw -= (PI * 2);
    }

    loc->position.position_local.x = interpolate(
        left_nearest_planning_point_time_us,
        last_planning_msg.trajectory.path[left_nearest_planning_point_idx]
                .position_enu.x -
            std::cos(ego_yaw) *
                (vehicle_param_.vehicle_length / 2 -
                 vehicle_param_.distance_from_rear_bumper_to_rear_axle),
        right_nearest_planning_point_time_us,
        last_planning_msg.trajectory.path[right_nearest_planning_point_idx]
                .position_enu.x -
            std::cos(ego_yaw) *
                (vehicle_param_.vehicle_length / 2 -
                 vehicle_param_.distance_from_rear_bumper_to_rear_axle),
        egopose_timestamp_us);
    loc->position.position_local.y = interpolate(
        left_nearest_planning_point_time_us,
        last_planning_msg.trajectory.path[left_nearest_planning_point_idx]
                .position_enu.y -
            std::sin(ego_yaw) *
                (vehicle_param_.vehicle_length / 2 -
                 vehicle_param_.distance_from_rear_bumper_to_rear_axle),
        right_nearest_planning_point_time_us,
        last_planning_msg.trajectory.path[right_nearest_planning_point_idx]
                .position_enu.y -
            std::sin(ego_yaw) *
                (vehicle_param_.vehicle_length / 2 -
                 vehicle_param_.distance_from_rear_bumper_to_rear_axle),
        egopose_timestamp_us);
    loc->position.available =
        maf_mla_localization::MLAPosition::MLA_POSITION_LOCAL;

    double velocity =
        interpolate(left_nearest_planning_point_time_us,
                    last_planning_msg.trajectory.velocity
                        .vel_points[left_nearest_planning_point_idx]
                        .target_velocity,
                    right_nearest_planning_point_time_us,
                    last_planning_msg.trajectory.velocity
                        .vel_points[right_nearest_planning_point_idx]
                        .target_velocity,
                    egopose_timestamp_us);

    loc->velocity.velocity_local.vx = std::cos(ego_yaw) * velocity;
    loc->velocity.velocity_local.vy = std::sin(ego_yaw) * velocity;
    loc->velocity.velocity_local.vz = 0;
    loc->velocity.available = maf_mla_localization::MLAVelocity::MLA_VEL_LOCAL;

    double acceleration =
        interpolate(left_nearest_planning_point_time_us,
                    last_planning_msg.trajectory.acceleration
                        .acc_points[left_nearest_planning_point_idx]
                        .acc,
                    right_nearest_planning_point_time_us,
                    last_planning_msg.trajectory.acceleration
                        .acc_points[right_nearest_planning_point_idx]
                        .acc,
                    egopose_timestamp_us);

    loc->acceleration.acceleration_local.ax = std::cos(ego_yaw) * acceleration;
    loc->acceleration.acceleration_local.ay = std::sin(ego_yaw) * acceleration;
    loc->velocity.velocity_local.vz = 0;
    loc->acceleration.available =
        maf_mla_localization::MLAAcceleration::MLA_ACC_LOCAL;

    return ego_yaw;
  }

  static void interpolate_fusion_reference_egopose(
      const maf_mla_localization::MLALocalization &left,
      const maf_mla_localization::MLALocalization &right,
      const uint64_t timestamp_us, maf_mla_localization::MLALocalization *loc) {
    set_egopose_all_field_unavailable(loc);
    loc->header.stamp = timestamp_us * MICRO_NS;
    loc->meta.timestamp_us = timestamp_us;

    double yaw_left = left.orientation.euler_local.yaw;
    double yaw_right = right.orientation.euler_local.yaw;
    // yaw角取值范围为[-PI, PI], 需处理PI到-PI的跳变
    if (yaw_left > yaw_right && yaw_left - yaw_right > PI) {
      yaw_right += 2 * PI;
    } else if (yaw_right > yaw_left && yaw_right - yaw_left > PI) {
      yaw_left += 2 * PI;
    }
    const uint64_t left_timestamp_us = left.meta.timestamp_us;
    const uint64_t right_timestamp_us = right.meta.timestamp_us;
    double ego_yaw = interpolate(left_timestamp_us, yaw_left,
                                 right_timestamp_us, yaw_right, timestamp_us);
    while (ego_yaw > PI) {
      ego_yaw -= (PI * 2);
    }

    loc->position.position_local.x = interpolate(
        left_timestamp_us, left.position.position_local.x, right_timestamp_us,
        right.position.position_local.x, timestamp_us);
    loc->position.position_local.y = interpolate(
        left_timestamp_us, left.position.position_local.y, right_timestamp_us,
        right.position.position_local.y, timestamp_us);
    loc->position.available =
        maf_mla_localization::MLAPosition::MLA_POSITION_LOCAL;

    set_quaternion_and_euler(ego_yaw, loc);
  }

  static void
  set_quaternion_and_euler(const double ego_yaw,
                           maf_mla_localization::MLALocalization *loc) {
    Eigen::Quaterniond q{};
    q.x() = loc->transform.transform_llh_to_boot.transform_q.x;
    q.y() = loc->transform.transform_llh_to_boot.transform_q.y;
    q.z() = loc->transform.transform_llh_to_boot.transform_q.z;
    q.w() = loc->transform.transform_llh_to_boot.transform_q.w;
    Eigen::Matrix3d r = geometry::quaternion_to_rotation_matrix(q);
    const Eigen::Matrix3d &transform_q_t = r.transpose();
    geometry::compute_quaternion_and_euler(ego_yaw, transform_q_t, loc);
  }

  static void set_position_global(maf_mla_localization::MLALocalization *loc) {
    NormalPoint3D boot_point{loc->position.position_local.x,
                             loc->position.position_local.y,
                             loc->position.position_local.z};
    NormalPoint4D q{};
    NormalPoint3D t{};
    PosGlobal center_point{};
    q.w = loc->transform.transform_llh_to_boot.transform_q.w;
    q.x = loc->transform.transform_llh_to_boot.transform_q.x;
    q.y = loc->transform.transform_llh_to_boot.transform_q.y;
    q.z = loc->transform.transform_llh_to_boot.transform_q.z;
    t.x = loc->transform.transform_llh_to_boot.transform_t.x;
    t.y = loc->transform.transform_llh_to_boot.transform_t.y;
    t.z = loc->transform.transform_llh_to_boot.transform_t.z;
    center_point.altitude =
        loc->transform.transform_llh_to_boot.transform_center.altitude;
    center_point.latitude =
        loc->transform.transform_llh_to_boot.transform_center.latitude;
    center_point.longitude =
        loc->transform.transform_llh_to_boot.transform_center.longitude;
    auto gps_point = geometry::boot_2_global(boot_point, q, t, center_point);
    loc->position.position_global.longitude = gps_point.x;
    loc->position.position_global.latitude = gps_point.y;
    loc->position.position_global.altitude = 0;
    loc->position.available |=
        maf_mla_localization::MLAPosition::MLA_POSITION_GLOBAL;
  }

  template <typename EgoposeMsgFrames>
  static const typename EgoposeMsgFrames::MessageWrapperType &
  get_reference_egopose(const int frame_id, const EgoposeMsgFrames &origin_msgs,
                        const maf_mla_localization::MLALocalization &loc) {
    if (frame_id < 0) {
      uint32_t nearest_frame_id = 0;
      double min_distance_square = std::numeric_limits<double>::max();

      // coarse-grained search
      for (uint32_t i = 0; i < origin_msgs.frame_count(); i++) {
        const auto &frame = origin_msgs.frame(i);
        if (frame.empty()) {
          continue;
        }

        for (const auto &msg : frame) {
          double distance_square =
              square_sum(frame.begin()->msg().position.position_local.x -
                             loc.position.position_local.x,
                         frame.begin()->msg().position.position_local.y -
                             loc.position.position_local.y);
          if (distance_square < min_distance_square) {
            min_distance_square = distance_square;
            nearest_frame_id = i;
          }
          break;
        }
      }
      return get_reference_egopose(nearest_frame_id, origin_msgs, loc);
    } else {
      double min_distance_square = std::numeric_limits<double>::max();
      const typename EgoposeMsgFrames::MessageWrapperType
          *nearest_origin_egopose = nullptr;
      const uint32_t search_start_frame_id =
          frame_id > 0 ? frame_id - 1 : frame_id;
      const uint32_t search_end_frame_id =
          frame_id + 1 < static_cast<int>(origin_msgs.frame_count())
              ? frame_id + 1
              : frame_id;

      // fine-grained search
      for (uint32_t i = search_start_frame_id; i <= search_end_frame_id; i++) {
        const auto &frame = origin_msgs.frame(i);
        for (const auto &msg : frame) {
          double distance_square =
              square_sum(msg.msg().position.position_local.x -
                             loc.position.position_local.x,
                         msg.msg().position.position_local.y -
                             loc.position.position_local.y);
          if (distance_square < min_distance_square) {
            min_distance_square = distance_square;
            nearest_origin_egopose = &msg;
          }
        }
      }
      if (nearest_origin_egopose != nullptr) {
        return *nearest_origin_egopose;
      }
    }
    LOG(ERROR) << "no egopose";
    std::abort();
  }

  template <typename EgoposeMsgs>
  static std::unique_ptr<maf_mla_localization::MLALocalization>
  gen_fusion_reference_egopose(const uint64_t timestamp_us,
                               const EgoposeMsgs &egopose_msgs) {
    if (egopose_msgs.empty()) {
      LOG(ERROR) << "non egopose msgs";
      return nullptr;
    }
    auto egopose = std::make_unique<maf_mla_localization::MLALocalization>();
    if (egopose_msgs.size() == 1) {
      *egopose = egopose_msgs.back().msg();
      LOG(WARNING) << "egopose_msgs.size() == 1, time diff: "
                   << egopose->meta.timestamp_us - timestamp_us << " us";
      return egopose;
    }
    if (egopose_msgs.back().msg().meta.timestamp_us < timestamp_us) {
      for (auto it = egopose_msgs.crbegin(); it < egopose_msgs.crend(); ++it) {
        *egopose = it->msg();
        LOG(WARNING) << "egopose msg meta.timestamp_us < "
                        "timestamp_us, time diff: "
                     << egopose->meta.timestamp_us - timestamp_us << " us";
        return egopose;
      }
      LOG(ERROR) << "no available egopose msgs";
      return nullptr;
    }
    if (egopose_msgs.front().msg().meta.timestamp_us > timestamp_us) {
      for (auto it = egopose_msgs.cbegin(); it < egopose_msgs.cend(); ++it) {
        *egopose = it->msg();
        LOG(WARNING) << "egopose msg meta.timestamp_us > "
                        "timestamp_us, time diff: "
                     << egopose->meta.timestamp_us - timestamp_us << " us";
        return egopose;
      }
      LOG(ERROR) << "no available egopose msgs";
      return nullptr;
    }

    const auto *egopose_left = &(egopose_msgs.back().msg());
    const auto *egopose_right = &(egopose_msgs.back().msg());
    for (auto it = egopose_msgs.crbegin(); it < egopose_msgs.crend(); ++it) {
      egopose_right = &(it->msg());
      for (auto it2 = it + 1; it2 < egopose_msgs.crend(); ++it2) {
        egopose_left = &(it2->msg());
        break;
      }
      if (egopose_left->meta.timestamp_us <= timestamp_us) {
        break;
      }
    }

    if (egopose_left->meta.timestamp_us > egopose_right->meta.timestamp_us) {
      LOG(WARNING) << "egopose disordered";
      *egopose = *(egopose_left);
      return egopose;
    }
    interpolate_fusion_reference_egopose(*egopose_left, *egopose_right,
                                         timestamp_us, egopose.get());

    return egopose;
  }

  static void process_fusion(
      const maf_mla_localization::MLALocalization &egopose,
      maf_perception_interface::PerceptionFusionObjectResult *fusion) {
    float theta_ego = 0.0;
    auto car2enu_matrix = geometry::car_to_enu_transform2(egopose, theta_ego);
    for (size_t i = 0; i < fusion->perception_fusion_objects_data.size(); i++) {
      auto position_ego = geometry::transform_enu2car(
          Eigen::Vector2d(fusion->perception_fusion_objects_data[i].position.x,
                          fusion->perception_fusion_objects_data[i].position.y),
          car2enu_matrix);
      fusion->perception_fusion_objects_data[i].relative_heading_yaw =
          fusion->perception_fusion_objects_data[i].heading_yaw - theta_ego;
      if (fusion->perception_fusion_objects_data[i].relative_heading_yaw >=
          2 * M_PI) {
        fusion->perception_fusion_objects_data[i].relative_heading_yaw -=
            2 * M_PI;
      } else if (fusion->perception_fusion_objects_data[i]
                     .relative_heading_yaw <= -2 * M_PI) {
        fusion->perception_fusion_objects_data[i].relative_heading_yaw +=
            2 * M_PI;
      }
      fusion->perception_fusion_objects_data[i].relative_position.x =
          position_ego.x();
      fusion->perception_fusion_objects_data[i].relative_position.y =
          position_ego.y();
    }
  };

  static void process_vision_lane_by_pos(
      const maf_mla_localization::MLALocalization &ori_egopose_msg,
      const maf_mla_localization::MLALocalization &new_egopose_msg,
      const maf_perception_interface::RoadLinePerception &ref_vision_lane,
      maf_perception_interface::RoadLinePerception *vision_lane) {
    float theta_ego_sim = 0.0;
    auto car2enu_matrix_sim =
        geometry::car_to_enu_transform2(new_egopose_msg, theta_ego_sim);
    float theta_ego_ori = 0.0;
    auto car2enu_matrix_ori =
        geometry::car_to_enu_transform2(ori_egopose_msg, theta_ego_ori);

    auto lanes_data = ref_vision_lane.lane_perception.lanes;
    // std::cout<<"lanes_data.size"<<lanes_data.size()<<std::endl;
    for (auto &temp_lane : lanes_data) {
      if (temp_lane.camera_source.value ==
              maf_perception_interface::CameraSourceEnum::
                  CAMERA_SOURCE_FRONT_WIDE ||
          temp_lane.is_failed_3d) {
        continue;
      }
      // lane_type need to consider!!!!
      auto &lane_points_3d_x = temp_lane.points_3d_x;
      auto &lane_points_3d_y = temp_lane.points_3d_y;
      auto lane_points_size =
          std::min(lane_points_3d_x.size(), lane_points_3d_y.size());
      // std::cout<<"lane_points_size:"<<lane_points_size<<std::endl;
      for (size_t i = 0; i < lane_points_size; ++i) {
        auto position_enu = geometry::transform_car2enu(
            Eigen::Vector2d(lane_points_3d_x[i], lane_points_3d_y[i]),
            car2enu_matrix_ori);
        auto position_ego =
            geometry::transform_enu2car(position_enu, car2enu_matrix_sim);
        lane_points_3d_x[i] = position_ego.x();
        lane_points_3d_y[i] = position_ego.y();
      }
      vision_lane->lane_perception.lanes.push_back(temp_lane);
    }
    // push_back roadedge!!!!
    auto roadedges_data = ref_vision_lane.road_edge_perception.road_edges;
    for (auto &temp_roadedge : roadedges_data) {
      if (temp_roadedge.camera_source.value ==
              maf_perception_interface::CameraSourceEnum::
                  CAMERA_SOURCE_FRONT_WIDE ||
          temp_roadedge.is_failed_3d) {
        continue;
      }
      auto &lane_points_3d_x = temp_roadedge.points_3d_x;
      auto &lane_points_3d_y = temp_roadedge.points_3d_y;
      auto lane_points_size =
          std::min(lane_points_3d_x.size(), lane_points_3d_y.size());
      for (size_t i = 0; i < lane_points_size; ++i) {
        auto position_enu = geometry::transform_car2enu(
            Eigen::Vector2d(lane_points_3d_x[i], lane_points_3d_y[i]),
            car2enu_matrix_ori);
        auto position_ego =
            geometry::transform_enu2car(position_enu, car2enu_matrix_sim);
        lane_points_3d_x[i] = position_ego.x();
        lane_points_3d_y[i] = position_ego.y();
      }
      vision_lane->road_edge_perception.road_edges.push_back(temp_roadedge);
    }
  };

private:
  utils::ClosedLoopVehicleParam vehicle_param_{};
};
} // namespace msquare
