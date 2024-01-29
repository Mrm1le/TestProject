#include "common/config/vehicle_param.h"
#include "common/ldp_result.h"
#include "common/math/cartesian_quintic_poly_1d.h"
#include "common/planning_context.h"
#include "common/utils/polyfit.h"
#include "ess_config_context.h"
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

namespace msquare {

class EssPlanningTask : public PlanningTaskInterface {
public:
  EssPlanningTask(
      MSDLdpPlanningConfig planning_config, bool enable_timer_tick,
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
      mtaskflow::FlowReceiver<
          std::shared_ptr<maf_planning::MpcTrajectoryResult>>
          mpc_trajectory_receiver,
      mtaskflow::FlowPublisher<std::string> planning_ess_status_publisher)
      : enable_timer_tick_(enable_timer_tick), tick_receiver_(tick_receiver),
        module_status_receiver_(module_status_receiver),
        chassis_report_receiver_(chassis_report_receiver),
        wheel_report_receiver_(wheel_report_receiver),
        body_report_receiver_(body_report_receiver),
        perception_vision_lane_receiver_(perception_vision_lane_receiver),
        perception_fusion_receiver_(perception_fusion_receiver),
        perception_fusion_aeb_receiver_(perception_fusion_aeb_receiver),
        mff_planning_request_receiver_(mff_planning_request_receiver),
        mpc_trajectory_receiver_(mpc_trajectory_receiver),
        planning_ess_status_publisher_(planning_ess_status_publisher) {
    ConfigurationContext::Instance()->load_vehicle_param();

    auto config_file_dir = PlanningContext::Instance()->get_config_file_dir();
    EssConfigurationContext::Instance()->load_ess_planner_config(
        config_file_dir);
    ess_planner_config_ =
        EssConfigurationContext::Instance()->ess_planner_config();
  }

  void on_running() {
    return;
    static uint64_t tick_count = 0;
    if (enable_timer_tick_) {
      tick_count++;
    } else if (!tick_receiver_->empty()) {
      uint64_t temp{0};
      auto ret = tick_receiver_->fetch_newest_and_clear(temp);
      if (ret) {
        tick_count = temp;
      }
    } else {
      return; // no tick, no running.
    }

    double start_time = MTIME()->timestamp().ms();
    MSD_LOG(WARN, "(%s)tick_count: %lu", __FUNCTION__, tick_count);

    MSD_LOG(WARN, "VERSION: 2021-07-17");

    bool succeed = run_once();

    if (reset_) {
      ess_result_.ess_info.ess_status = EssInfo::ESS_STATUS_NONE;
      ess_result_.ess_info.last_ess_status = EssInfo::ESS_STATUS_NONE;
      ess_result_.ess_info.aeb_object_track_id = 0;
      ess_result_.ess_info.d_poly.clear();

      reset_ = false;
      MSD_LOG(WARN, "reset ess planning");
    }

    if (nullptr != planning_output_callback_) {
      // only send once when ess trigged
      if (ess_result_.ess_info.last_ess_status == EssInfo::ESS_STATUS_NONE &&
          ess_result_.ess_info.ess_status != EssInfo::ESS_STATUS_NONE) {
        auto maf_planning_output = generate_planning_output();

        planning_output_callback_({tick_count, succeed}, maf_planning_output,
                                  "");
        MSD_LOG(INFO, "planning_output: publish once");
      }
    }

    std::string ess_output{};
    if (succeed) {
      ess_output = generate_ess_output();
    }
    if (nullptr != planning_ess_output_callback_ && succeed) {
      planning_ess_output_callback_(ess_output);
    }
    if (nullptr != planning_ess_status_publisher_ && succeed) {
      planning_ess_status_publisher_->publish(ess_output);
    }

    ess_result_.ess_info.last_ess_status = ess_result_.ess_info.ess_status;
    double end_time = MTIME()->timestamp().ms();
    MSD_LOG(INFO, "time_cost, total time: %f", end_time - start_time);
  }

  void set_sync_callback(NPPHeaderLoggerCallback callback) {
    // not used
  }

  void set_callback(MSDPlanningOutputCallback callback) {
    planning_output_callback_ = callback;
  }

  void set_callback(MSDPlanningLdpOutputCallback callback) {
    planning_ess_output_callback_ = callback;
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

  std::string generate_ess_output() {
    std::string ess_output_string{};
    auto result = mjson::Json(mjson::Json::object());

    result["is_aeb_object_existed"] = mjson::Json(is_aeb_object_existed_);
    result["is_left_solid_line"] = mjson::Json(is_left_solid_line_);
    result["is_right_solid_line"] = mjson::Json(is_right_solid_line_);
    result["is_left_near_road_edge"] = mjson::Json(is_left_near_road_edge_);
    result["is_right_near_road_edge"] = mjson::Json(is_right_near_road_edge_);
    result["is_left_near_object"] = mjson::Json(is_left_near_object_);
    result["is_right_near_object"] = mjson::Json(is_right_near_object_);
    mjson::Json::array left_front_objects_array{};
    mjson::Json::array left_back_objects_array{};
    mjson::Json::array right_front_objects_array{};
    mjson::Json::array right_back_objects_array{};
    for (const auto &object : left_front_objects_) {
      left_front_objects_array.emplace_back(mjson::Json(object.track_id));
    }
    for (const auto &object : left_back_objects_) {
      left_back_objects_array.emplace_back(mjson::Json(object.track_id));
    }
    for (const auto &object : right_front_objects_) {
      right_front_objects_array.emplace_back(mjson::Json(object.track_id));
    }
    for (const auto &object : right_back_objects_) {
      right_back_objects_array.emplace_back(mjson::Json(object.track_id));
    }
    result["left_front_objects"] = mjson::Json(left_front_objects_array);
    result["left_back_objects"] = mjson::Json(left_back_objects_array);
    result["right_front_objects"] = mjson::Json(right_front_objects_array);
    result["right_back_objects"] = mjson::Json(right_back_objects_array);

    result["end_x"] = mjson::Json(ess_result_.ess_info.end_x);

    result["ess_machine_status"] =
        mjson::Json(ess_state_machine_info_.ess_status);
    result["ess_driver_intention"] =
        mjson::Json(ess_result_.ess_info.ess_driver_intention);
    result["ess_status"] = mjson::Json(ess_result_.ess_info.ess_status);
    result["last_ess_status"] =
        mjson::Json(ess_result_.ess_info.last_ess_status);

    ess_output_string = result.dump();
    return ess_output_string;
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

      reset();
    }

    // reset when function is disabled
    if (ess_state_machine_info_.ess_status ==
        EssStateMachineInfo::ESS_STATE_MACHINE_DISABLE) {
      reset();
    }

    if (steering_wheel_data_list_.empty()) {
      reset();
      return false;
    }

    if (!ess_process()) {
      reset();
    }

    auto end_timestamp = MTIME()->timestamp();

    return true;
  }

  void update_chassis_report(double current_time) {
    static constexpr double deg2rad = M_PI / 180.0;

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
        steering_wheel.steering_wheel_data.timestamp_us =
            received_chassis_report->meta.timestamp_us;
        steering_wheel.steering_wheel_data.steering_wheel_rad =
            received_chassis_report->steering_report.steering_report_data
                .steering_wheel_angle_report -
            steer_angle_offset_deg_ * deg2rad;
        steering_wheel.steering_wheel_data.steering_wheel_torque =
            received_chassis_report->steering_report.steering_report_data
                .steering_wheel_torque_report;
        last_feed_time_[FEED_EGO_STEER_ANGLE] = current_time;

        steering_wheel_data_list_.push_back(steering_wheel.steering_wheel_data);
        if (steering_wheel_data_list_.size() > 100) {
          steering_wheel_data_list_.pop_front();
        }
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
  }

  void update_mpc_trajectory(double current_time) {
    if (!mpc_trajectory_receiver_->empty()) {
      std::shared_ptr<maf_planning::MpcTrajectoryResult> mpc_trajectory{
          nullptr};
      auto ret =
          mpc_trajectory_receiver_->fetch_newest_and_clear(mpc_trajectory);
      if (!ret) {
        return;
      }

      if (mpc_trajectory == nullptr) {
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
        MSD_LOG(INFO, "steer_angle_offset_deg: %f", steer_angle_offset_deg_);
      } catch (mjson::Exception &e) {
        MSD_LOG(WARN, "mpc json parse error!");
        MSD_LOG(WARN, e.what());
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

        auto ess_string =
            request_reader.get<std::string>("ess_status", false, "");
        if (ess_string != "") {
          auto ess_reader = mjson::Reader(ess_string);
          int ess_status = ess_reader.get<int>("status", false, 0);

          ess_state_machine_info_.ess_status =
              EssStateMachineInfo::EssStateMachineStatus(ess_status);
        }
      }
    }

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

    update_mpc_trajectory(current_time);
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

    static const double kCheckTimeDiffWarn = 0.5;
    static const double kCheckTimeDiffError = 1.0;

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

    msg.trajectory.available |= maf_planning::Trajectory::POLYNOMIAL_CURVE;
    msg.trajectory.polynomial_curve.polynomial = ess_result_.ess_info.d_poly;

    return msg;
  }

  double calc_line_y(const std::vector<double> &coef, double x) {
    double y = 0;

    for (int i = (int)coef.size() - 1; i >= 0; --i) {
      y = y * x + coef[i];
    }
    return y;
  }

  void check_lanes_and_road_edges() {
    auto const &lines = perception_vision_lane_.lane_perception;
    auto const &roadedges = perception_vision_lane_.road_edge_perception;

    is_left_solid_line_ = false;
    is_right_solid_line_ = false;
    bool is_left_virtual_lane = true;
    bool is_right_virtual_lane = true;
    for (int i = 0; i < lines.lanes.size(); i++) {
      const auto &line = lines.lanes[i];
      // check left line
      if (line.is_failed_3d == false && line.is_centerline == false &&
          line.index == -1 &&
          line.camera_source.value ==
              CameraSourceEnum::CAMERA_SOURCE_FRONT_MID &&
          line.points_3d_x.size() > 3) {
        if (lines.reserved_infos.size() > i) {
          std::string err{};
          auto reserved_infos_json =
              mjson::Json::parse(lines.reserved_infos[i], err);
          if (err.empty() && reserved_infos_json.has_key("is_virtual_lane")) {
            if (reserved_infos_json["is_virtual_lane"].bool_value() == true) {
              is_left_virtual_lane = true;
            } else {
              is_left_virtual_lane = false;
            }
          } else {
            is_left_virtual_lane = false;
          }
        } else {
          is_left_virtual_lane = false;
        }

        if (!is_left_virtual_lane) {
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
        if (lines.reserved_infos.size() > i) {
          std::string err{};
          auto reserved_infos_json =
              mjson::Json::parse(lines.reserved_infos[i], err);
          if (err.empty() && reserved_infos_json.has_key("is_virtual_lane")) {
            if (reserved_infos_json["is_virtual_lane"].bool_value() == true) {
              is_right_virtual_lane = true;
            } else {
              is_right_virtual_lane = false;
            }
          } else {
            is_right_virtual_lane = false;
          }
        } else {
          is_right_virtual_lane = false;
        }

        if (!is_right_virtual_lane) {
          if (line.lane_type.value == LaneTypeEnum::LANE_TYPE_SOLID ||
              line.lane_type.value == LaneTypeEnum::LANE_TYPE_DOUBLE_SOLID) {
            is_right_solid_line_ = true;
          }
        }
      }
    }

    is_left_near_road_edge_ = false;
    is_right_near_road_edge_ = false;
    double half_ego_width =
        ConfigurationContext::Instance()->get_vehicle_param().width / 2;
    for (const auto &roadedge : roadedges.road_edges) {
      // check left road edges
      if (roadedge.is_failed_3d == false && roadedge.index == -1 &&
          roadedge.camera_source.value ==
              CameraSourceEnum::CAMERA_SOURCE_FRONT_MID &&
          roadedge.points_3d_x.size() > 3 &&
          roadedge.coefficient_bv.size() > 0) {
        // check intercept
        if (std::abs(roadedge.coefficient_bv[0]) <
            half_ego_width + ess_planner_config_.near_road_edge_thres) {
          is_left_near_road_edge_ = true;
          MSD_LOG(INFO, "left_near_road_edge: track id: %d, intercept: %f",
                  roadedge.track_id, roadedge.coefficient_bv[0]);
        }

        // check aeb object
        std::vector<double> coeffs{};
        coeffs.assign(roadedge.coefficient_bv.begin(),
                      roadedge.coefficient_bv.end());
        if (is_aeb_object_existed_) {
          double road_edge_y =
              calc_line_y(coeffs, aeb_object_.relative_position.x);
          double dy = road_edge_y - aeb_object_.relative_position.y;
          if (std::abs(dy) <
              half_ego_width + ess_planner_config_.near_road_edge_thres) {
            is_left_near_road_edge_ = true;
            MSD_LOG(INFO,
                    "left_near_road_edge: track id: %d, road_edge_y: %f, "
                    "aeb_object_x: %f, aeb_object_y: %f",
                    roadedge.track_id, road_edge_y,
                    aeb_object_.relative_position.x,
                    aeb_object_.relative_position.y);
          }
        }
      }

      // check right road edges
      if (roadedge.is_failed_3d == false && roadedge.index == 1 &&
          roadedge.camera_source.value ==
              CameraSourceEnum::CAMERA_SOURCE_FRONT_MID &&
          roadedge.points_3d_x.size() > 3 &&
          roadedge.coefficient_bv.size() > 0) {
        // check intercept
        if (std::abs(roadedge.coefficient_bv[0]) <
            half_ego_width + ess_planner_config_.near_road_edge_thres) {
          is_right_near_road_edge_ = true;
          MSD_LOG(INFO, "right_near_road_edge: track iD; %d, intercept: %f",
                  roadedge.track_id, roadedge.coefficient_bv[0]);
        }

        // check aeb object
        std::vector<double> coeffs{};
        coeffs.assign(roadedge.coefficient_bv.begin(),
                      roadedge.coefficient_bv.end());
        if (is_aeb_object_existed_) {
          double road_edge_y =
              calc_line_y(coeffs, aeb_object_.relative_position.x);
          double dy = road_edge_y - aeb_object_.relative_position.y;
          if (std::abs(dy) <
              half_ego_width + ess_planner_config_.near_road_edge_thres) {
            is_right_near_road_edge_ = true;
            MSD_LOG(INFO,
                    "right_near_road_edge: track id: %d, road_edge_y: %f, "
                    "aeb_object_x: %f, aeb_object_y: %f",
                    roadedge.track_id, road_edge_y,
                    aeb_object_.relative_position.x,
                    aeb_object_.relative_position.y);
          }
        }
      }
    }
  }

  void check_aeb_objects() {
    is_aeb_object_existed_ = false;
    is_fcw_trigged_ = false;

    double ttc = 10.0;
    if (perception_fusion_aeb_.perception_fusion_aeb_objects
            .perception_fusion_aeb_objects_data.size() > 0) {
      aeb_object_ = perception_fusion_aeb_.perception_fusion_aeb_objects
                        .perception_fusion_aeb_objects_data[0];
      aeb_object_ttc_ = perception_fusion_aeb_.min_ttc;
      is_aeb_object_existed_ = true;
      if (perception_fusion_aeb_.perception_fusion_aeb_state.value ==
          maf_perception_interface::PerceptionFusionAEBStateEnum::
              AEB_STATE_FCW) {
        fcw_trigger_count_ = ess_planner_config_.fcw_count_thres;
      } else if (perception_fusion_aeb_.perception_fusion_aeb_state.value ==
                 maf_perception_interface::PerceptionFusionAEBStateEnum::
                     AEB_STATE_AEB) {
        fcw_trigger_count_ = 0;
      }

      // check ttc
      ttc = aeb_object_.relative_velocity.x < 0.0
                ? (aeb_object_.relative_position.x -
                   aeb_object_.shape.length / 2.0 - rear_axle_to_head_) /
                      std::abs(aeb_object_.relative_velocity.x)
                : 10.0;
      MSD_LOG(INFO, "check_aeb_objects: track id: %d, aeb ttc: %f, ttc: %f",
              aeb_object_.track_id, perception_fusion_aeb_.min_ttc, ttc);
    }

    // add count check
    if (fcw_trigger_count_ > 0) {
      is_fcw_trigged_ = true;
    }
    fcw_trigger_count_ = std::max(fcw_trigger_count_ - 1, 0);
  }

  void check_front_near_object(
      const std::vector<maf_perception_interface::PerceptionFusionObjectData>
          &front_objects,
      bool is_left) {
    const double ignore_vx_thres = -3.0;
    for (const auto &object : front_objects) {
      // check reverse object
      if (object.velocity_relative_to_ground.x < ignore_vx_thres) {
        is_left ? is_left_near_object_ = true : is_right_near_object_ = true;
        MSD_LOG(INFO,
                "check_front_near_object: reverse object: %d, velocity: %f",
                object.track_id, object.velocity_relative_to_ground.x);
        continue;
      }

      // check near distance
      double safty_distance = 1.0 * ego_vel_;
      double relative_distance = object.relative_position.x -
                                 object.shape.length / 2.0 - rear_axle_to_head_;
      if (relative_distance < safty_distance) {
        is_left ? is_left_near_object_ = true : is_right_near_object_ = true;
        MSD_LOG(INFO,
                "check_front_near_object: near distance object: %d, "
                "relative_distance: %f",
                object.track_id, relative_distance);
        continue;
      }

      // check ttc
      double ttc = object.relative_velocity.x < 0.0
                       ? (object.relative_position.x -
                          object.shape.length / 2.0 - rear_axle_to_head_) /
                             std::abs(object.relative_velocity.x)
                       : 10.0;
      if (ttc < ess_planner_config_.near_forward_object_ttc) {
        is_left ? is_left_near_object_ = true : is_right_near_object_ = true;
        MSD_LOG(INFO,
                "check_front_near_object: near ttc object: %d, "
                "ttc: %f",
                object.track_id, ttc);
        continue;
      }
    }
  }

  void check_back_near_object(
      const std::vector<maf_perception_interface::PerceptionFusionObjectData>
          &back_objects,
      bool is_left) {
    for (const auto &object : back_objects) {
      // check near distance
      double safty_distance = 1.0 * ego_vel_;
      double relative_distance = object.relative_position.x +
                                 object.shape.length / 2.0 + rear_axle_to_back_;
      if (relative_distance > -safty_distance) {
        is_left ? is_left_near_object_ = true : is_right_near_object_ = true;
        MSD_LOG(INFO,
                "check_back_near_object: near distance object: %d, "
                "relative_distance: %f",
                object.track_id, relative_distance);
        continue;
      }

      // check ttc
      double ttc = object.relative_velocity.x > 0.0
                       ? (object.relative_position.x +
                          object.shape.length / 2.0 + rear_axle_to_back_) /
                             object.relative_velocity.x
                       : 10.0;
      if (ttc < ess_planner_config_.near_backward_object_ttc) {
        is_left ? is_left_near_object_ = true : is_right_near_object_ = true;
        MSD_LOG(INFO,
                "check_back_near_object: near ttc object: %d, "
                "ttc: %f",
                object.track_id, ttc);
        continue;
      }
    }
  }

  void check_fusion_objects() {
    is_left_near_object_ = false;
    is_right_near_object_ = false;
    left_front_objects_.clear();
    left_back_objects_.clear();
    right_front_objects_.clear();
    right_back_objects_.clear();
    double front_object_distance =
        ess_planner_config_.near_forward_object_distance;
    if (is_aeb_object_existed_) {
      front_object_distance = std::max(
          front_object_distance, aeb_object_.relative_position.x +
                                     aeb_object_.shape.length / 2.0 + 5.0);
    }
    for (const auto &object :
         perception_fusion_.perception_fusion_objects_data) {
      // check lateral thres
      if (std::abs(object.relative_position.y) - object.shape.width / 2.0 <
          ess_planner_config_.near_object_lateral_thres) {
        if (is_aeb_object_existed_ && object.track_id == aeb_object_.track_id) {
          continue;
        }
        // front thres
        if (object.relative_position.x + object.shape.length / 2.0 >
                -rear_axle_to_back_ &&
            object.relative_position.x - object.shape.length / 2.0 <
                front_object_distance) {
          if (object.relative_position.y > 0.0) {
            left_front_objects_.push_back(object);
          } else {
            right_front_objects_.push_back(object);
          }
          // back thres
        } else if (object.relative_position.x + object.shape.length / 2.0 <
                       -rear_axle_to_back_ &&
                   object.relative_position.x + object.shape.length / 2.0 >
                       ess_planner_config_.near_backward_object_distance) {
          if (object.relative_position.y > 0.0) {
            left_back_objects_.push_back(object);
          } else {
            right_back_objects_.push_back(object);
          }
        }
      }
    }

    check_front_near_object(left_front_objects_, true);
    check_front_near_object(right_front_objects_, false);
    check_back_near_object(left_back_objects_, true);
    check_back_near_object(right_back_objects_, false);
  }

  void check_driver_intention() {
    ess_result_.ess_info.ess_driver_intention =
        EssInfo::ESS_DRIVER_INTENTION_NONE;
    // not trigged
    if (ess_result_.ess_info.ess_status == EssInfo::ESS_STATUS_NONE) {
      if (is_aeb_object_existed_) {
        double delta_torque =
            steering_wheel_data_list_.back().steering_wheel_torque -
            steering_wheel_data_list_.front().steering_wheel_torque;
        if (steering_wheel_data_list_.back().steering_wheel_torque >
                ess_planner_config_.driver_intention_torque &&
            delta_torque > ess_planner_config_.driver_intention_delta_torque) {
          ess_result_.ess_info.ess_driver_intention =
              EssInfo::ESS_DRIVER_INTENTION_LEFT;
          MSD_LOG(INFO,
                  "check_driver_intention: left! torque: %f, delta torque: %f",
                  steering_wheel_data_list_.back().steering_wheel_torque,
                  delta_torque);
        } else if (steering_wheel_data_list_.back().steering_wheel_torque <
                       -ess_planner_config_.driver_intention_torque &&
                   delta_torque <
                       -ess_planner_config_.driver_intention_delta_torque) {
          ess_result_.ess_info.ess_driver_intention =
              EssInfo::ESS_DRIVER_INTENTION_RIGHT;
          MSD_LOG(INFO,
                  "check_driver_intention: right! torque: %f, delta torque: %f",
                  steering_wheel_data_list_.back().steering_wheel_torque,
                  delta_torque);
        }
      }
      // left trigged, check right override
    } else if (ess_result_.ess_info.ess_status ==
               EssInfo::ESS_STATUS_LEFT_TRIGGERED) {
      if (steering_wheel_data_list_.back().steering_wheel_torque <
          -ess_planner_config_.driver_override_torque) {
        ess_result_.ess_info.ess_driver_intention =
            EssInfo::ESS_DRIVER_INTENTION_RIGHT_OVERRIDE;
      }
      // right trigged, check left override
    } else if (ess_result_.ess_info.ess_status ==
               EssInfo::ESS_STATUS_RIGHT_TRIGGERED) {
      if (steering_wheel_data_list_.back().steering_wheel_torque >
          ess_planner_config_.driver_override_torque) {
        ess_result_.ess_info.ess_driver_intention =
            EssInfo::ESS_DRIVER_INTENTION_LEFT_OVERRIDE;
      }
    }
  }

  bool check_driver_avoidance(bool is_left) {
    if (!is_aeb_object_existed_) {
      return true;
    }

    const auto &steering_wheel_rad =
        vehicle_status_.steering_wheel.steering_wheel_data.steering_wheel_rad;
    if (ess_result_.ess_info.ess_driver_intention ==
                EssInfo::ESS_DRIVER_INTENTION_LEFT &&
            steering_wheel_rad < 0.0 ||
        ess_result_.ess_info.ess_driver_intention ==
                EssInfo::ESS_DRIVER_INTENTION_RIGHT &&
            steering_wheel_rad > 0.0) {
      return false;
    }

    double remaining_distance =
        aeb_object_.relative_position.x - aeb_object_.shape.length / 2.0 -
        rear_axle_to_head_ -
        ess_planner_config_.driver_longitudinal_safety_thres;

    double steering_radius =
        ConfigurationContext::Instance()->get_vehicle_param().wheel_base /
        std::max(std::abs(steering_wheel_rad), 0.001) *
        ConfigurationContext::Instance()->get_vehicle_param().steer_ratio;

    if (remaining_distance > steering_radius) {
      return true;
    }

    double dl = steering_radius - std::sqrt(std::pow(steering_radius, 2) -
                                            std::pow(remaining_distance, 2));

    double half_ego_width =
        ConfigurationContext::Instance()->get_vehicle_param().width / 2;
    double lateral_min_distance =
        is_left ? dl - aeb_object_.relative_position.y -
                      aeb_object_.shape.width / 2.0 - half_ego_width
                : dl + aeb_object_.relative_position.y -
                      aeb_object_.shape.width / 2.0 - half_ego_width;
    MSD_LOG(INFO,
            "check_driver_avoidance: remaining_distance: %f, steering_radius: "
            "%f, dl: %f, lateral_min_distance: %f",
            remaining_distance, steering_radius, dl, lateral_min_distance);

    if (lateral_min_distance >
        ess_planner_config_.driver_lateral_safety_thres) {
      return true;
    }

    return false;
  }

  bool generate_ess_path(std::vector<double> &coeff) {
    coeff.clear();

    if (!is_aeb_object_existed_ ||
        ess_result_.ess_info.ess_status == EssInfo::ESS_STATUS_NONE) {
      MSD_LOG(INFO, "generate_ess_path: no aeb target or not trigged");
      return false;
    }

    // start point
    double start_x = 0.0;
    double start_y = 0.0;
    double start_yaw = 0.0;
    double start_curvature = 0.0;

    double start_v = ego_vel_;

    // end point
    double end_yaw = 0.0;
    double end_curvature = 0.0;

    double longitudinal_safety_buffer =
        ess_planner_config_.planning_longitudinal_safety_thres;
    double end_x = aeb_object_.relative_position.x -
                   aeb_object_.shape.length / 2.0 - longitudinal_safety_buffer;
    double end_y = aeb_object_.relative_position.y;
    double lateral_safety_buffer =
        (aeb_object_.shape.width / 2.0 +
         ConfigurationContext::Instance()->get_vehicle_param().width / 2 +
         ess_planner_config_.planning_lateral_safety_thres);
    if (ess_result_.ess_info.ess_status == EssInfo::ESS_STATUS_LEFT_TRIGGERED) {
      end_y += lateral_safety_buffer;
    } else if (ess_result_.ess_info.ess_status ==
               EssInfo::ESS_STATUS_RIGHT_TRIGGERED) {
      end_y -= lateral_safety_buffer;
    }

    msquare::planning_math::CartesianQuinticPoly1d cartesian_quintic_poly1d(
        {start_x, start_y, start_yaw, start_curvature},
        {end_x, end_y, end_yaw, end_curvature}, start_v);
    MSD_LOG(INFO,
            "generate_ess_path: start_x: %f, start_y: %f, end_x: %f, end_y: %f",
            start_x, start_y, end_x, end_y);

    // sample pts
    std::vector<Point2D> cartesian_quintic_poly1d_pts{};
    cartesian_quintic_poly1d_pts.clear();
    double max_jerk = 0.0;
    double max_acc = 0.0;
    for (double percent = 0.0; percent < 1.001; percent += 0.02) {
      auto sample_point_quintic_poly =
          cartesian_quintic_poly1d.Evaluate(percent);
      Point2D ref_pt;
      ref_pt.x = sample_point_quintic_poly.x;
      ref_pt.y = sample_point_quintic_poly.y;
      max_jerk =
          std::fmax(std::fabs(sample_point_quintic_poly.lat_jerk), max_jerk);
      max_acc =
          std::fmax(std::fabs(sample_point_quintic_poly.lat_acc), max_acc);
      cartesian_quintic_poly1d_pts.push_back(ref_pt);
    }

    // check lateral acc
    if (max_acc > ess_planner_config_.planning_max_lateral_acc_thres) {
      MSD_LOG(INFO, "generate_ess_path: laterl acc too large, max_acc: %f",
              max_acc);
      // return false;
    }

    // polyfit
    if (cartesian_quintic_poly1d_pts.size() < 3 ||
        !Polyfit(cartesian_quintic_poly1d_pts, 5, coeff) || coeff.size() != 6) {
      MSD_LOG(INFO, "generate_ess_path: polyfit failed");
      return false;
    }
    ess_result_.ess_info.end_x = cartesian_quintic_poly1d_pts.back().x;

    double starting_polyfit_error =
        cartesian_quintic_poly1d_pts.front().y -
        calc_line_y(coeff, cartesian_quintic_poly1d_pts.front().x);
    double mid_polyfit_error =
        cartesian_quintic_poly1d_pts[cartesian_quintic_poly1d_pts.size() / 2]
            .y -
        calc_line_y(
            coeff,
            cartesian_quintic_poly1d_pts[cartesian_quintic_poly1d_pts.size() /
                                         2]
                .x);
    double ending_polyfit_error =
        cartesian_quintic_poly1d_pts.back().y -
        calc_line_y(coeff, cartesian_quintic_poly1d_pts.back().x);

    MSD_LOG(INFO,
            "generate_ess_path: polyfit error: front: %f, mid: %f end: %f, max "
            "jerk: "
            "%f, max acc: %f",
            starting_polyfit_error, mid_polyfit_error, ending_polyfit_error,
            max_jerk, max_acc);

    return true;
  }

  bool check_exit() {
    if (ess_result_.ess_info.ess_status == EssInfo::ESS_STATUS_NONE) {
      MSD_LOG(INFO, "check_exit: not triggered");
      return true;
    }

    // check max trigger time
    double current_time_sec = MTIME()->timestamp().sec();
    if (current_time_sec - ess_result_.ess_info.trigger_timestamp_start_sec >
        ess_planner_config_.planning_max_trigger_time_sec) {
      MSD_LOG(INFO, "check_exit: max trigger time reached");
      return true;
    }

    int aeb_object_index = -1;
    for (int i = 0;
         i < perception_fusion_.perception_fusion_objects_data.size(); i++) {
      if (perception_fusion_.perception_fusion_objects_data[i].track_id ==
          ess_result_.ess_info.aeb_object_track_id) {
        aeb_object_index = i;
      }
    }
    if (aeb_object_index < 0) {
      MSD_LOG(INFO, "check_exit: aeb object not existed in fusion object");
      return true;
    }

    const auto &aeb_object =
        perception_fusion_.perception_fusion_objects_data[aeb_object_index];

    double remaining_distance =
        aeb_object.relative_position.x - aeb_object.shape.length / 2.0 -
        rear_axle_to_head_ + ess_planner_config_.planning_exit_exceed_distance;
    if (remaining_distance < 0) {
      MSD_LOG(INFO, "check_exit: remaining distance short");
      return true;
    }

    return false;
  }

  void check_ess_status() {
    is_left_trigger_suppressed_ = false;
    is_right_trigger_suppressed_ = false;
    // not trigged, check trigger
    if (ess_result_.ess_info.ess_status == EssInfo::ESS_STATUS_NONE) {
      ess_result_.ess_info.d_poly.clear();
      ess_result_.ess_info.end_x = -1;
      ess_result_.ess_info.aeb_object_track_id = 0;

      // check machine status
      if (ess_state_machine_info_.ess_status ==
          EssStateMachineInfo::ESS_STATE_MACHINE_DISABLE) {
        is_left_trigger_suppressed_ = true;
        is_right_trigger_suppressed_ = true;
        MSD_LOG(INFO, "check_ess_status: suppressed by mff");
      }

      // check line/road_edge/object
      if (is_left_solid_line_ || is_left_near_road_edge_ ||
          is_left_near_object_) {
        is_left_trigger_suppressed_ = true;
        MSD_LOG(INFO,
                "check_ess_status: left suppressed by line/road_edge/object");
      }
      if (is_right_solid_line_ || is_right_near_road_edge_ ||
          is_right_near_object_) {
        is_right_trigger_suppressed_ = true;
        MSD_LOG(INFO,
                "check_ess_status: right suppressed by line/road_edge/object");
      }

      // check cooling
      double cooling_time = MTIME()->timestamp().sec() -
                            ess_result_.ess_info.trigger_timestamp_sec;
      if (cooling_time < ess_planner_config_.planning_cooling_time_sec) {
        is_left_trigger_suppressed_ = true;
        is_right_trigger_suppressed_ = true;
        MSD_LOG(INFO,
                "check_ess_status: suppressed by cooling not finished, cooling "
                "time: %f",
                cooling_time);
      }

      if (is_fcw_trigged_) {
        // check ttc
        if (aeb_object_ttc_ < ess_planner_config_.planning_min_ttc_thres) {
          is_left_trigger_suppressed_ = true;
          is_right_trigger_suppressed_ = true;
          MSD_LOG(INFO,
                  "check_ess_status: suppressed by ttc too small, ttc: %f",
                  aeb_object_ttc_);
        }

        // check driver avoidance
        if (ess_result_.ess_info.ess_driver_intention ==
                EssInfo::ESS_DRIVER_INTENTION_LEFT &&
            check_driver_avoidance(true)) {
          is_left_trigger_suppressed_ = true;
          MSD_LOG(INFO,
                  "check_ess_status: left suppressed by driver avoidance");
        }
        if (ess_result_.ess_info.ess_driver_intention ==
                EssInfo::ESS_DRIVER_INTENTION_RIGHT &&
            check_driver_avoidance(false)) {
          is_right_trigger_suppressed_ = true;
          MSD_LOG(INFO,
                  "check_ess_status: right suppressed by driver avoidance");
        }

        // left trigger check
        if (ess_result_.ess_info.ess_driver_intention ==
                EssInfo::ESS_DRIVER_INTENTION_LEFT &&
            !is_left_trigger_suppressed_) {
          ess_result_.ess_info.ess_status = EssInfo::ESS_STATUS_LEFT_TRIGGERED;

          // right trigger check
        } else if (ess_result_.ess_info.ess_driver_intention ==
                       EssInfo::ESS_DRIVER_INTENTION_RIGHT &&
                   !is_right_trigger_suppressed_) {
          ess_result_.ess_info.ess_status = EssInfo::ESS_STATUS_RIGHT_TRIGGERED;
        }

        // generate ess path
        if (ess_result_.ess_info.ess_status != EssInfo::ESS_STATUS_NONE) {
          if (!generate_ess_path(ess_result_.ess_info.d_poly)) {
            ess_result_.ess_info.ess_status = EssInfo::ESS_STATUS_NONE;
            MSD_LOG(INFO, "generate_ess_path failed!");
          }

          ess_result_.ess_info.trigger_timestamp_start_sec =
              MTIME()->timestamp().sec();
          ess_result_.ess_info.aeb_object_track_id = aeb_object_.track_id;
        }
      }
      // left triggered, check exit
    } else if (ess_result_.ess_info.ess_status ==
               EssInfo::ESS_STATUS_LEFT_TRIGGERED) {
      // check override
      if (ess_result_.ess_info.ess_driver_intention ==
          EssInfo::ESS_DRIVER_INTENTION_RIGHT_OVERRIDE) {
        ess_result_.ess_info.ess_status = EssInfo::ESS_STATUS_NONE;
        MSD_LOG(INFO, "check_ess_status: right override");
      }
      // right triggered, check exit
    } else if (ess_result_.ess_info.ess_status ==
               EssInfo::ESS_STATUS_RIGHT_TRIGGERED) {
      // check override
      if (ess_result_.ess_info.ess_driver_intention ==
          EssInfo::ESS_DRIVER_INTENTION_LEFT_OVERRIDE) {
        ess_result_.ess_info.ess_status = EssInfo::ESS_STATUS_NONE;
        MSD_LOG(INFO, "check_ess_status: left override");
      }
    }

    if (ess_result_.ess_info.ess_status != EssInfo::ESS_STATUS_NONE) {
      double current_time_sec = MTIME()->timestamp().sec();
      ess_result_.ess_info.trigger_timestamp_sec = current_time_sec;

      // check exit
      if (check_exit()) {
        ess_result_.ess_info.ess_status = EssInfo::ESS_STATUS_NONE;
        MSD_LOG(INFO, "check_ess_status: check exit success");
      }
    }
  }

  bool ess_process() {
    ego_vel_ = (vehicle_status_.wheel_velocity.wheel_velocity4d.rear_left +
                vehicle_status_.wheel_velocity.wheel_velocity4d.rear_right) /
               2 *
               ConfigurationContext::Instance()
                   ->get_vehicle_param()
                   .rear_wheel_rolling_radius;

    rear_axle_to_head_ =
        ConfigurationContext::Instance()->get_vehicle_param().length -
        ConfigurationContext::Instance()
            ->get_vehicle_param()
            .rear_bumper_to_rear_axle;
    rear_axle_to_back_ = ConfigurationContext::Instance()
                             ->get_vehicle_param()
                             .rear_bumper_to_rear_axle;

    // reload config when decel below 40kph
    static double last_ego_vel = ego_vel_;
    if (last_ego_vel > 40.0 / 3.6 && ego_vel_ < 40.0 / 3.6) {
      auto config_file_dir = PlanningContext::Instance()->get_config_file_dir();
      EssConfigurationContext::Instance()->load_ess_planner_config(
          config_file_dir);
      ess_planner_config_ =
          EssConfigurationContext::Instance()->ess_planner_config();
    }
    last_ego_vel = ego_vel_;

    check_aeb_objects();

    check_fusion_objects();

    check_lanes_and_road_edges();

    check_driver_intention();

    check_ess_status();

    return true;
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
  maf_vehicle_status::VehicleStatus vehicle_status_{};
  maf_perception_interface::RoadLinePerception perception_vision_lane_{};
  maf_perception_interface::PerceptionFusionObjectResult perception_fusion_{};
  maf_perception_interface::PerceptionFusionAEBResult perception_fusion_aeb_{};
  std::deque<maf_vehicle_status::SteeringWheelData> steering_wheel_data_list_{};

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
  mtaskflow::FlowReceiver<std::shared_ptr<maf_planning::MpcTrajectoryResult>>
      mpc_trajectory_receiver_;
  mtaskflow::FlowPublisher<std::string> planning_ess_status_publisher_;

  MSDPlanningOutputCallback planning_output_callback_{};
  MSDPlanningLdpOutputCallback planning_ess_output_callback_{};

  double steer_angle_offset_deg_ = 0.0;

  bool enable_timer_tick_{true};
  bool reset_{false};

  double ego_vel_ = 0.0;

  EssStateMachineInfo ess_state_machine_info_{};
  EssResult ess_result_{};

  // aeb object
  bool is_aeb_object_existed_ = false;
  bool is_fcw_trigged_ = false;
  int fcw_trigger_count_ = 0;
  maf_perception_interface::PerceptionFusionAEBObjectData aeb_object_{};
  double aeb_object_ttc_ = 0.0;
  bool is_left_near_object = false;
  bool is_right_near_object = false;

  // fusion object
  bool is_left_near_object_ = false;
  bool is_right_near_object_ = false;
  std::vector<maf_perception_interface::PerceptionFusionObjectData>
      left_front_objects_{};
  std::vector<maf_perception_interface::PerceptionFusionObjectData>
      right_front_objects_{};
  std::vector<maf_perception_interface::PerceptionFusionObjectData>
      left_back_objects_{};
  std::vector<maf_perception_interface::PerceptionFusionObjectData>
      right_back_objects_{};

  // lane
  bool is_left_solid_line_ = false;
  bool is_right_solid_line_ = false;
  bool is_left_near_road_edge_ = false;
  bool is_right_near_road_edge_ = false;

  const double STD_LANE_WIDTH = 3.5;
  const double MAX_LANE_WIDTH = 4.5;
  const double IGNORE_LANE_WIDTH = 7.0;

  // trigger
  bool is_left_trigger_suppressed_ = false;
  bool is_right_trigger_suppressed_ = false;

  EssPlannerConfig ess_planner_config_{};
  double rear_axle_to_head_ = 0.0;
  double rear_axle_to_back_ = 0.0;
};

} // namespace msquare
