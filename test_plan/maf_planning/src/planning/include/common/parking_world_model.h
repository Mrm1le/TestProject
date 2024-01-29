#pragma once

#include <memory>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <vector>

#include "common/math/line_segmentxd.h"
#include "common/parking_ego_state_manager.h"
#include "common/parking_map_info_manager.h"
#include "common/planning_context.h"
#include "planner/behavior_planner/deciders/ground_line_decider.h"
#include "planner/message_type.h"
#include "planner/planning_config.h"
#include "pnc.h"

// TODO: solve loop include
#include "common/parking_obstacle_manager.h"
// #include "pnc/define/parking_vehicle_report.h"
#include "planner/behavior_planner/parking_refline_manager.h"

namespace msquare {

namespace parking {

class MessageManager;
// class ObstacleManager;

// enum class SceneType : unsigned char {
//   // URBAN = 0,
//   // PARKING,
//   X_PLUS = 0,
//   PRO,
//   APA,
//   REMOTE_CONTROL,
// };

class WorldModel : public std::enable_shared_from_this<WorldModel> {
public:
  WorldModel();
  ~WorldModel();

  bool init(SceneType scene_type); // init after construction

  bool update(); // update infos each frame

  bool update_parking();

  // void get_map_lateral_info_parking();

  // bool cal_intersection(const double s);

  // bool cal_carriage_way(const double s);

  // bool cal_via_point(const double s);

  // void set_via_weight(const double via_weight_base, const int id);

  // double cal_via_weight(const double s);

  // double cal_theta(const double s);

  // int cal_lane_id(const double s);

  std::string get_neareat_refline_points();

  void reset_square_map_response();

  // void update_and_filter_freespace(
  //     const std::vector<planning_math::LineSegment2d> &map_boundarys);

  void update_map_boundary();

  void update_groundline();

  // bool freespace_filter(const planning_math::LineSegment2d &line,
  //                       const std::string type,
  //                       const SLBoundary *line_sl = nullptr);

  /**
   * @brief get the cartesian coordinate of aimed poi's projection on frenet.
   * Only aimed poi in forward is valid.
   */
  bool get_aimed_poi_projection_on_route(Pose2D *pose, double lat_offset = 0);

private:
  void update_scenes();

  void update_members();

  bool extend_traj(std::vector<Pose2D> &traj_points);

  bool construct_obstacle_manager_parking();

public:
  // get
  const ObstacleManager &obstacle_manager() const { return *obstacle_manager_; }

  ObstacleManager &mutable_obstacle_manager() const {
    return *obstacle_manager_;
  }

  const FrenetState &get_planning_start_state() const {
    return ego_state_manager_.get_ego_state().planning_start_state;
  }

  const State &get_mpc_vehicle_state() const {
    return ego_state_manager_.get_ego_state().mpc_vehicle_state;
  }

  bool is_replan() const { return ego_state_manager_.get_flag_is_replan(); }

  const EgoStateManager &get_ego_state_manager() const {
    return ego_state_manager_;
  }

  const EgoState &get_ego_state() const {
    return ego_state_manager_.get_ego_state();
  }

  const EgoState &get_ego_state_planning() const {
    return ego_state_manager_.get_ego_state_planning();
  }

  const MapInfo &get_map_info() const {
    return map_info_manager_.get_map_info();
  }

  const SLBoundary &get_adc_sl_boundary() const { return adc_sl_boundary_; }

  MapInfo &get_mutable_map_info() {
    return map_info_manager_.get_mutable_map_info();
  }

  int get_ignore_obs_id() const { return ignore_obs_id_; }

  bool get_vehicle_dbw_status() const { return vehicle_dbw_status_; }

  bool get_localization_status() const { return localization_status_; }

  bool get_vechicle_set_status() const { return vehicle_set_status_; }

  bool get_dbw_status() const { return dbw_status_; }

  bool get_pretreatment_status() const { return pretreatment_status_; }

  bool is_rear_obstacles_considered() const { return is_rear_obs_considered_; }

  bool get_cross_lane_enable() const { return cross_lane_enable_; }

  std::string get_cross_lane_direction() const { return cross_lane_direction_; }

  const maf_vehicle_status::VehicleLightData &get_vehicle_light_report() const {
    return vehicle_light_report_;
  }

  const maf_vehicle_status::GearData &get_gear_report() const {
    return gear_report_;
  }

  const maf_endpoint::WirelessChargerReportData &
  get_wireless_charger_report_data() const {
    return wireless_charger_report_data_;
  }

  const maf_vehicle_status::WheelVelocity4d &get_wheel_speed_report() const {
    return wheel_speed_report_;
  }
  const ParkingMapInfo &get_parking_map_info() const {
    return parking_map_info_;
  }
  // const ParkingVisionInfo &get_parking_vision_info() const {
  //   return parking_vision_info_;
  // }
  // const FusionInfo &get_parking_fusion_vision_info() const {
  //   return parking_fusion_vision_info_;
  // }
  const SquareMapResponse &get_square_map_response() const {
    return square_map_response_;
  }

  const SquareMapResponse &get_square_map_response2() const {
    return square_map_response2_;
  }

  const PlanningRequest &get_planning_request() const {
    return planning_request_;
  }

  bool clear_planning_request() {
    planning_request_.cmd.value = ParkingCommand::NONE;
    return true;
  }

  void set_control_test_flag(bool flag) { control_test_flag_ = flag; }

  const bool get_control_test_flag() { return control_test_flag_; }

  // const RefLineInfoFrenet &get_refline_info_frenet() const {
  //   return ref_trajectory_info_;
  // }

  // const RefLineInfoFrenet &get_refline_info_frenet_last() const {
  //   return ref_trajectory_info_last_;
  // }

  const std::vector<PathPoint> &get_mpc_trajectory() const { return mpc_traj_; }

  // const std::vector<PathPoint> &get_mpc_trajectory_undiluted() const {
  //   return mpc_traj_undiluted_;
  // }
  double get_current_msg_time() const { return current_msg_time_; }

  std::shared_ptr<FrenetCoordinateSystem> get_frenet_coord() {
    return frenet_coord_;
  }

  std::shared_ptr<FrenetCoordinateSystem> get_planning_frenet_coord() {
    return frenet_coord_planning_;
  }

  double get_distance_to_poi() const {
    if (is_parking_svp()) {
      if (parking_map_info_.aimed_poi_info.type == "PARKING_LOT" &&
          PlanningContext::Instance()
              ->parking_behavior_planner_output()
              .parking_slot_info.park_in_point.available)
        return PlanningContext::Instance()
            ->parking_behavior_planner_output()
            .parking_slot_info.park_in_point.distance;
      else if (parking_map_info_.aimed_poi_info.type == "HUMAN_ACCESS")
        return map_info_manager_.get_map_info().distance_to_aimed_poi;
      else
        return 1e19;
    } else if (false && is_parking_lvp()) {
      if (PlanningContext::Instance()
              ->parking_behavior_planner_output()
              .parking_slot_info.park_in_point.available) {
        return PlanningContext::Instance()
            ->parking_behavior_planner_output()
            .parking_slot_info.park_in_point.distance;
      } else
        return 1e19;
    } else
      return 1e19;
  }

  double get_poi_s() const {
    return get_ego_state().ego_frenet.x + get_map_info().distance_to_aimed_poi;
  }

  double get_distance_to_refline_end() const {
    if (frenet_coord_ != nullptr)
      return frenet_coord_->GetSlength() -
             ego_state_manager_.get_ego_state().ego_frenet.x;
    else
      return 1e19;
  }

  const SquareMapResponse &get_square_map_response_avp() const {
    return map_info_manager_.get_map_info().square_mapping_result;
  }

  std::vector<planning_math::LineSegment2d> get_square_map__border_avp() {
    std::vector<planning_math::LineSegment2d> border_vec;
    for (size_t i = 0; i < map_info_manager_.get_map_info()
                               .square_mapping_result.road_borders.size();
         i++) {
      for (size_t j = 1; j < map_info_manager_.get_map_info()
                                 .square_mapping_result.road_borders[i]
                                 .pts.size();
           j++) {
        auto pt1 = map_info_manager_.get_map_info()
                       .square_mapping_result.road_borders[i]
                       .pts[j - 1];
        auto pt2 = map_info_manager_.get_map_info()
                       .square_mapping_result.road_borders[i]
                       .pts[j];
        planning_math::LineSegment2d line_cur(
            planning_math::Vec2d(pt1.x, pt1.y),
            planning_math::Vec2d(pt2.x, pt2.y));
        border_vec.push_back(line_cur);
      }
    }
    return border_vec;
  }

  bool get_collide_to_limiter_when_reverse() {
    return collide_to_limiter_when_reverse_;
  }

  bool get_steering_reset() { return steering_reset_; }

  // double get_control_lat_error() { return control_lat_error_; }

  // double get_control_phi_error() { return control_phi_error_; }

  const std::vector<UssResult> &get_parking_uss_results() const {
    return parking_fusion_vision_info_.uss_results;
  }

  const std::vector<GroundLine> &get_parking_ground_line_fusion() const {
    return parking_fusion_vision_info_.ground_line_fusion;
  }

  bool is_parking_svp() const { return scene_type_ == SceneType::PARKING_SVP; }

  bool is_parking_lvp() const { return scene_type_ == SceneType::PARKING_LVP; }

  bool is_parking_apa() const { return scene_type_ == SceneType::PARKING_APA; }

  bool get_pause_status() const { return pause_; }
  bool get_hard_brake() const { return is_hard_brake_; }
  bool get_brake_override() const { return override_; }
  bool get_force_p_gear() const { return force_p_gear_; }

  bool is_simulation() const { return is_simulation_; }

  RefLineManager *get_refline_manager() { return refline_manager_; };

  void feed_pause_status(bool pause) { pause_ = pause; }

private:
  void change_scene_type(const SceneType &scene_type) {
    scene_type_ = scene_type;
  }

  void feed_map_info(const MapInfo &map_info) {
    map_info_manager_.set_map_info(map_info);
  }

  void feed_vehicle_dbw_status(bool flag) {
    vehicle_dbw_status_last_ = vehicle_dbw_status_;
    vehicle_dbw_status_ = flag;
    if (!vehicle_dbw_status_last_ && vehicle_dbw_status_)
      vehicle_set_status_ = true;
    else
      vehicle_set_status_ = false;
  }

  void feed_localization_status(bool flag) { localization_status_ = flag; }

  void feed_ego_pose(const Pose2D &pose) {
    ego_state_manager_.set_ego_pose(pose);
  }

  void feed_ego_enu(const Pose3D &enu) { ego_state_manager_.set_ego_enu(enu); }

  void feed_ego_steer_angle(double steer_angle) {
    ego_state_manager_.set_ego_steer_angle(steer_angle);
  }

  void feed_ego_acc(float ego_acc) { ego_state_manager_.set_ego_acc(ego_acc); }

  void feed_vehicle_light_report(
      const maf_vehicle_status::VehicleLightData &vehicle_light_report) {
    vehicle_light_report_ = vehicle_light_report;
  }
  void feed_gear_report(const maf_vehicle_status::GearData &gear_report) {
    gear_report_ = gear_report;
  }
  void feed_wireless_charger_report_data(
      const maf_endpoint::WirelessChargerReportData &data) {
    wireless_charger_report_data_ = data;
  }
  void feed_wheel_speed_report(
      const maf_vehicle_status::WheelVelocity4d &wheel_speed_report,
      bool static_status) {
    wheel_speed_report_ = wheel_speed_report;
    float vel =
        0.339 *
        (wheel_speed_report_.front_left + wheel_speed_report_.front_right +
         wheel_speed_report_.rear_left + wheel_speed_report_.rear_right) /
        4.0;
    ego_state_manager_.set_ego_vel(vel);
    ego_state_manager_.set_ego_static_status(static_status);
  }

  void feed_aimed_poi_info(const AimedPoiInfo &aimed_poi_info) {
    parking_map_info_.aimed_poi_info = aimed_poi_info;
  }
  // void feed_parking_out_info(const ParkingOutInfo &parking_out_info) {
  //   parking_map_info_.parking_out_info = parking_out_info;
  // }
  // void
  // feed_fuzzy_search_results(const std::vector<FuzzySearchResult> &results) {
  //   parking_map_info_.fuzzy_search_results = results;
  // }
  // void feed_accurate_search_results(
  //     const std::vector<AccurateSearchResult> &results) {
  //   parking_map_info_.accurate_search_results = results;
  // }

  void feed_parking_lots_detection_fusion_results(
      const std::vector<ParkingLotDetectionInfo> &results) {
    parking_map_info_.parking_lots_detection_fusion_results = results;
  }

  // void feed_parking_human(const std::vector<VisionHuman> &vision_human) {
  //   use_car_preception_result = true;
  //   parking_vision_info_.human_array = vision_human;
  // }
  // void feed_parking_car(const std::vector<VisionCar> &vision_car) {
  //   use_car_preception_result = true;
  //   parking_vision_info_.car_array = vision_car;
  // }
  // void feed_parking_freespace(const VisionFreeSpace &vision_free_space) {
  //   use_car_preception_result = true;
  //   parking_vision_info_.free_space = vision_free_space;
  // }

  // void feed_parking_flc_vision(const ParkingVisionInfo &parking_vision_info)
  // {
  //   parking_flc_vision_info_ = parking_vision_info;
  // }

  void
  feed_parking_fusion_human(const std::vector<FusionObject> &vision_human) {
    // use_fusion_result_ = true;
    parking_fusion_vision_info_.human_array = vision_human;
  }
  void feed_parking_fusion_car(const std::vector<FusionObject> &vision_car) {
    // use_fusion_result_ = true;
    parking_fusion_vision_info_.car_array = vision_car;
  }
  void
  feed_parking_static_fusion_car(const std::vector<FusionObject> &vision_car) {
    parking_fusion_vision_info_.static_car_array = vision_car;
  }
  void feed_parking_fusion_freespace(
      const std::vector<FusionFreespacePoint> &vision_free_space) {
    // use_fusion_result_ = true;
    parking_fusion_vision_info_.free_space = vision_free_space;
  }
  void
  feed_parking_uss_fusion(const std::vector<FusionFreespacePoint> &uss_fusion) {
    parking_fusion_vision_info_.uss_fusion = uss_fusion;
  }
  void feed_parking_uss_results(const std::vector<UssResult> &uss_results) {
    parking_fusion_vision_info_.uss_results = uss_results;
  }
  void feed_parking_ground_line_fusion(
      const std::vector<GroundLine> &ground_line_fusion) {
    parking_fusion_vision_info_.ground_line_fusion = ground_line_fusion;
  }
  // void feed_parking_fusion_fcp_human(const std::vector<VisionHuman>
  // &vision_human) {
  //   parking_fusion_fcp_vision_info_.human_array = vision_human;
  // }
  // void feed_parking_fusion_fcp_car(const std::vector<VisionCar> &vision_car)
  // {
  //   parking_fusion_fcp_vision_info_.car_array = vision_car;
  // }
  // void feed_parking_fusion_fcp_freespace(const VisionFreeSpace
  // &vision_free_space) {
  //   parking_fusion_fcp_vision_info_.free_space = vision_free_space;
  // }

  void feed_planning_request(const PlanningRequest &request) {
    planning_request_ = request;

    // ParkingUiResponse *response =
    //     PlanningContext::Instance()->mutable_parking_ui_response();

    // response->send_flag = true;
    // response->time_stamp = request.time_stamp;
  }

  void feed_square_map_response(const SquareMapResponse &square_map_response) {
    square_map_response_ = square_map_response;
  };

  void feed_square_map_response2(const SquareMapResponse &square_map_response) {
    square_map_response2_ = square_map_response;
  };

  void feed_mpc_traj(const std::vector<PathPoint> &mpc_traj_reltaive) {
    // transform from car coords to enu coors
    double ego_x = ego_state_manager_.get_ego_state().ego_pose.x;
    double ego_y = ego_state_manager_.get_ego_state().ego_pose.y;
    double ego_theta = ego_state_manager_.get_ego_state().ego_pose.theta;

    mpc_traj_.clear();
    mpc_traj_.emplace_back(ego_x, ego_y, 0.0, ego_theta, 0.0);
    for (auto point_relative : mpc_traj_reltaive) {
      PathPoint point;
      point.x =
          ego_x +
          cos(ego_theta) * (point_relative.x +
                            VehicleParam::Instance()->front_edge_to_center) -
          sin(ego_theta) * (point_relative.y);
      point.y =
          ego_y +
          sin(ego_theta) * (point_relative.x +
                            VehicleParam::Instance()->front_edge_to_center) +
          cos(ego_theta) * (point_relative.y);
      point.theta = ego_theta + point_relative.theta;
      mpc_traj_.push_back(point);
    }

    // std::vector<PathPoint> mpc_traj_undiluted;
    // for (auto point_relative : mpc_traj_reltaive_undiluted) {
    //   PathPoint point;
    //   point.x =
    //       ego_x +
    //       cos(ego_theta) * (point_relative.x +
    //                         VehicleParam::Instance()->front_edge_to_center) -
    //       sin(ego_theta) * (point_relative.y);
    //   point.y =
    //       ego_y +
    //       sin(ego_theta) * (point_relative.x +
    //                         VehicleParam::Instance()->front_edge_to_center) +
    //       cos(ego_theta) * (point_relative.y);
    //   point.theta = ego_theta + point_relative.theta;
    //   mpc_traj_undiluted.push_back(point);
    // }
    // mpc_traj_undiluted_ = mpc_traj_undiluted;

    // control_lat_error_ = lat_error;
    // control_phi_error_ = phi_error;
  };

  void feed_current_msg_time(double current_msg_time) {
    current_msg_time_ = current_msg_time;
  }

  void
  feed_collide_to_limiter_when_reverse(bool collide_to_limiter_when_reverse) {
    collide_to_limiter_when_reverse_ = collide_to_limiter_when_reverse;
  }

  void feed_steering_reset(bool steering_reset) {
    steering_reset_ = steering_reset;
  }

  void feed_hard_brake(bool is_hard_brake) { is_hard_brake_ = is_hard_brake; }

  void feed_brake_override(bool override) { override_ = override; }

  void feed_force_p_gear(bool force_p_gear) { force_p_gear_ = force_p_gear; }

private:
  std::unique_ptr<ObstacleManager> obstacle_manager_;
  RefLineManager *refline_manager_;

  SceneType scene_type_{SceneType::PARKING_APA};
  EgoStateManager ego_state_manager_;
  MapInfoManager map_info_manager_;

  std::unique_ptr<GroundLineDecider> ground_line_decider_;

  FrenetCoordinateSystemParameters frenet_parameters_;

  bool vehicle_dbw_status_{false};
  bool dbw_status_{false};
  bool localization_status_{false};
  bool pretreatment_status_{true};
  int ignore_obs_id_ = -10000;
  bool is_rear_obs_considered_ = false;
  bool cross_lane_enable_ = false;
  std::string cross_lane_direction_;
  // bool use_fusion_result_ = false;
  // bool use_car_preception_result = false;
  bool vehicle_set_status_{false};
  bool vehicle_dbw_status_last_{false};
  bool pause_{false};
  bool force_p_gear_{false};
  bool is_hard_brake_{false};
  bool override_{false};

  bool control_test_flag_{false};

  bool is_simulation_{false};

  // ego car state
  // frenet coords based on map refline
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;
  // frenet coords based on planning trajectory
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_planning_;
  std::shared_ptr<Obstacle> obs_ptr_ = nullptr;
  SLBoundary adc_sl_boundary_;

  // MPC
  double process_calculate_time_;
  bool planner_success_;
  int64_t planning_loops_;

  maf_vehicle_status::VehicleLightData vehicle_light_report_;
  maf_vehicle_status::GearData gear_report_;
  maf_vehicle_status::WheelVelocity4d wheel_speed_report_;
  maf_endpoint::WirelessChargerReportData wireless_charger_report_data_;
  ParkingMapInfo parking_map_info_;
  // ParkingVisionInfo parking_vision_info_;
  // ParkingVisionInfo parking_flc_vision_info_;
  FusionInfo parking_fusion_vision_info_;
  // ParkingVisionInfo parking_fusion_fcp_vision_info_;
  PlanningRequest planning_request_;
  SquareMapResponse square_map_response_;
  SquareMapResponse square_map_response2_;
  std::vector<PathPoint> mpc_traj_;
  // std::vector<PathPoint> mpc_traj_undiluted_;
  bool collide_to_limiter_when_reverse_{false};
  bool steering_reset_{false};
  // double control_lat_error_;
  // double control_phi_error_;

  // current imprecise msg time updated each frame
  double current_msg_time_{0.0};

  // tmp use
  // RefLineInfoFrenet ref_trajectory_info_;
  // RefLineInfoFrenet ref_trajectory_info_last_;
  planning_math::LineSegmentXd base_lines_;

  friend class PlanningTask;
};

} // namespace parking

} // namespace msquare
