#ifndef MSQUARE_DECISION_PLANNING_COMMON_WORLD_MODEL_H_
#define MSQUARE_DECISION_PLANNING_COMMON_WORLD_MODEL_H_

#include "common/config_context.h"
#include "planning/common/common.h"
#include <cstdint>
#include <maf_interface/maf_perception_interface.h>
#include <memory>
#include <mlog_core/mlog.h>
#include <string>
#include <typeinfo>
#include <unordered_map>

#include "common/baseline_info.h"
#include "common/ego_state_manager.h"
#include "common/map_info_manager.h"
#include "common/obstacle_manager.h"
#include "common/path/discretized_path.h"
#include "common/planning_common_type.h"
#include "common/planning_context.h"
#include "common/refline_generator.h"
#include "common/traffic_light_decision.h"
#include "data_driven_planner/common/basic_types.h"
#include "planner/message_type.h"
#include "planner/planning_config.h"
#include "pnc.h"
#include "pnc/define/pass_intersection_input_interface.hpp"

using namespace maf_vehicle_status;
using namespace pass_intersection_planner;

namespace msquare {

class MessageManager;
class ObstacleManager;
class TrafficLightDecision;
class LateralObstacle;

using namespace msd_planning;

struct TrafficLight {
  double header = 0.0;
  bool valid = false;
  std::vector<maf_perception_interface::TrafficLightDecision> info;
};

enum class VirtualObstacleId : int {
  TRAFFIC_LIGHT = -2,
  LANE_CHANGE_FULL_LINE = -3,
  FORCE_STOP = -4,
};

struct ObstacleRawData {
  bool has_prediction{false};
  bool has_fusion{false};
  maf_perception_interface::PerceptionFusionObjectData *fusion_info;
  PredictionObject *prediction_info;
};

class WorldModel : public std::enable_shared_from_this<WorldModel> {
public:
  WorldModel();
  ~WorldModel();
  MapInfoManager &mutable_map_info_manager() { return map_info_manager_; }

  LateralObstacle &mutable_lateral_obstacle() { return *lateral_obstacle_; }

  bool init(SceneType scene_type); // init after construction

  bool update(); // update infos each frame

  bool is_replan() const { return ego_state_manager_.get_flag_is_replan(); }

  const CartEgoStateManager &get_cart_ego_state_manager() const {
    return ego_state_manager_;
  }

  const TrafficLight &get_traffic_light_info() { return traffic_light_info_; }

  const std::shared_ptr<TrafficLightDecision> &
  get_traffic_light_decision() const {
    return traffic_light_decision_;
  }

  const std::shared_ptr<ReflineGenerator> &get_refline_generator() const {
    return refline_generator_;
  }

  const MapInfoManager &get_map_info_manager() const {
    return map_info_manager_;
  }

  MapInfoManager &get_mutable_map_info_manager() { return map_info_manager_; }

  const MSDMapInfo &get_map_info() const {
    return map_info_manager_.get_map_info();
  }

  MSDMapInfo &get_mutable_map_info() {
    return map_info_manager_.get_mutable_map_info();
  }

  std::shared_ptr<BaseLineInfo> get_baseline_info(
      int lane_id) { // CJ:In ACC mode baseline needs to be updated each cycle
    if (baseline_infos_.find(lane_id) != baseline_infos_.end() &&
        baseline_infos_[lane_id]->is_update()) {
      return baseline_infos_[lane_id];
    } else {
      construct_baseline_info(lane_id);
      return get_baseline_info(lane_id);
    }
    // construct_baseline_info(lane_id);
    return baseline_infos_[lane_id];
  }

  int get_ignore_obs_id() const { return ignore_obs_id_; }
  bool get_vehicle_dbw_status() const { return vehicle_dbw_status_; }
  bool get_last_vehicle_dbw_status() const { return last_vehicle_dbw_status_; }
  bool get_dbw_status() const { return dbw_status_; }
  double get_enter_auto_drive_time() const { return enter_auto_drive_time_; }
  bool get_pretreatment_status() const { return pretreatment_status_; }

  bool is_urban_scene() const { return SceneType::URBAN == scene_type_; }
  bool is_parking_scene() const {
    return SceneType::PARKING_SVP == scene_type_ ||
           SceneType::PARKING_LVP == scene_type_ ||
           SceneType::PARKING_APA == scene_type_;
  }

  const maf_vehicle_status::VehicleStatus &get_vehicle_status() const {
    return vehicle_status_;
  }
  const maf_vehicle_status::VehicleLightData &get_vehicle_light_report() const {
    return vehicle_light_report_;
  }
  const maf_vehicle_status::GearData &get_gear_report() const {
    return gear_report_;
  }
  const maf_vehicle_status::WheelVelocity &get_wheel_speed_report() const {
    return wheel_speed_report_;
  }
  const ParkingMapInfo &get_parking_map_info() const {
    return parking_map_info_;
  }
  const ParkingVisionInfo &get_parking_vision_info() const {
    return parking_vision_info_;
  }
  const SquareMapResponse &get_square_map_response() const {
    return square_map_response_;
  }

  const maf_perception_interface::RoadLinePerception &
  get_perception_vision_lane() const {
    return perception_vision_lane_;
  }

  const maf_perception_interface::RoadLinePerception &
  get_perception_lidar_road_edges() const {
    return perception_lidar_road_edge_;
  }

  const maf_perception_interface::RadarPerceptionResult &
  get_perception_radar() const {
    return perception_radar_;
  }

  const maf_perception_interface::PerceptionFusionObjectResult &
  get_perception_fusion_object() const {
    return perception_fusion_object_;
  }

  bool get_cipv_fn() const { return cipv_fn_; }

  int get_cipv_fn_tid() const { return cipv_fn_tid_; }

public:
  void get_current_lane_ref(int target_lane_id,
                            std::vector<Point2D> &current_lane_ref);
  void get_map_lateral_info(std::vector<RefPointFrenet> &cur_lane,
                            std::vector<RefPointFrenet> &left_lane,
                            std::vector<RefPointFrenet> &right_lane,
                            int target_lane_id);
  void
  trans_lane_points(const std::vector<ReferenceLinePointDerived> &lane_points,
                    std::vector<RefPointFrenet> &refline,
                    std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
                    bool smoother_comp);

  void trans_lane_points_from_baseline(std::vector<RefPointFrenet> &refline,
                                       std::shared_ptr<BaseLineInfo> baseline,
                                       bool smoother_comp);

  std::vector<PredictionObject> &get_prediction_info() {
    return prediction_info_;
  }

  std::vector<PredictionObject> &get_truncated_prediction_info() {
    return truncated_prediction_info_;
  }

  const std::vector<maf_perception_interface::PerceptionFusionObjectData> &
  get_fusion_info() {
    return fusion_info_;
  }

  std::vector<PredictionObject> &get_full_prediction_info() {
    return full_prediction_info_;
  }

  void construct_baseline_info(int relative_lane_id);

  void truncate_prediction_info(std::shared_ptr<BaseLineInfo> baseline_info);

  bool is_acc_mode() { return is_acc_mode_; }
  bool last_is_acc_mode() { return last_is_acc_mode_; }
  void set_acc_mode(bool is_acc_mode) { is_acc_mode_ = is_acc_mode; }
  void set_last_acc_mode(bool is_acc_mode) { last_is_acc_mode_ = is_acc_mode; }

  bool is_ddmap() { return is_ddmap_; }
  void set_ddmap(bool is_ddmap) { is_ddmap_ = is_ddmap; }

  bool use_eftp() { return use_eftp_; }
  void set_use_eftp(bool use_eftp) { use_eftp_ = use_eftp; }

  bool lateral_use_eftp() { return lateral_use_eftp_; }
  void set_lateral_use_eftp(bool lateral_use_eftp) {
    lateral_use_eftp_ = lateral_use_eftp;
  }

  bool is_ddmap_intersection() { return is_ddmap_intersection_; }
  void set_ddmap_intersection(bool is_ddmap_intersection) {
    is_ddmap_intersection_ = is_ddmap_intersection;
  }

  double get_baseline_jitter() { return baseline_overlap_; }

  void set_baseline_jitter(double jitter) { baseline_overlap_ = jitter; }

  void set_reset(bool reset) { reset_ = reset; }
  bool is_reset() { return reset_; }

  void set_v_curv(double v_curv) { v_curv_ = v_curv; }
  double get_v_curv() { return v_curv_; }

  void set_a_curv(double a_curv) { a_curv_ = a_curv; }
  double get_a_curv() { return a_curv_; }

  void set_max_curv(double curv) { max_curv_ = curv; }
  double get_max_curv() { return max_curv_; }

  void set_max_curv_distance(double distance) { max_curv_distance_ = distance; }
  double get_max_curv_distance() { return max_curv_distance_; }

  void set_current_last_car_point_x(double current_last_car_point_x) {
    current_last_car_point_x_ = current_last_car_point_x;
  }

  void set_current_last_enu_point(const Point2D &current_last_enu_point) {
    current_last_enu_point_ = current_last_enu_point;
  }

  void set_ilc_limit_velocity(double limit_v) { ilc_limit_velocity_ = limit_v; }
  double get_ilc_limit_velocity() const { return ilc_limit_velocity_; }

  void set_ilc_limit_time(double limit_t) { ilc_limit_time_ = limit_t; }
  double get_ilc_limit_time() const { return ilc_limit_time_; }

  double get_current_last_car_point_x() { return current_last_car_point_x_; }

  const Point2D &get_current_last_enu_point() {
    return current_last_enu_point_;
  }

  // void set_refline_attract_type(ApfAttractType attract_type) {
  //   attract_type_ = attract_type;
  // }
  void set_refline_attract_type(uint8_t attract_type) {
    attract_type_.attract_type = attract_type;
  }
  // void set_refline_condition(ReflineCondition condition) {
  //   condition_ = condition;
  // }
  void set_refline_condition(uint8_t condition) {
    condition_.reflinecondition = condition;
  }
  // void set_refline_last_attract_type(ApfAttractType last_attract_type) {
  //   last_attract_type_ = last_attract_type;
  // }
  void set_refline_last_attract_type(uint8_t last_attract_type) {
    last_attract_type_.attract_type = last_attract_type;
  }
  // void set_refline_last_condition(ReflineCondition last_condition) {
  //   last_condition_ = last_condition;
  // }
  void set_refline_last_condition(uint8_t last_condition) {
    last_condition_.reflinecondition = last_condition;
  }
  void set_apf_lat_far_flag(bool apf_lat_far_flag) {
    apf_lat_far_flag_ = apf_lat_far_flag;
  }

  void replace_refline_points(
      std::vector<Pose2D> refline_result, double cur_lane_left_border,
      double cur_lane_right_border,
      std::vector<ReferenceLinePointDerived> &refline_points);

  pass_intersection_planner::ApfAttractType get_refline_attract_type() {
    return attract_type_;
  }
  pass_intersection_planner::ReflineCondition get_refline_condition() {
    return condition_;
  }
  pass_intersection_planner::ApfAttractType get_refline_last_attract_type() {
    return last_attract_type_;
  }
  pass_intersection_planner::ReflineCondition get_refline_last_condition() {
    return last_condition_;
  }
  bool get_apf_lat_far_flag() { return apf_lat_far_flag_; }

  DrivingStyle get_driving_style() { return driving_style_; }
  void set_driving_style(DrivingStyle driving_style) {
    driving_style_ = driving_style;
  }

  LaneChangingStyle get_lane_changing_style() { return lane_changing_style_; }
  void set_lane_changing_style(LaneChangingStyle lane_changing_style) {
    lane_changing_style_ = lane_changing_style;
  }

  void set_enable_ilc(bool enable_ilc) { enable_ilc_ = enable_ilc; }

  bool enable_ilc() { return enable_ilc_; }

  void set_enable_alc(bool enable_alc) { enable_alc_ = enable_alc; }

  bool enable_alc() { return enable_alc_; }

  void set_enable_recommend_alc(bool enable_recommand_alc) {
    enable_recommend_alc_ = enable_recommand_alc;
  }

  bool get_enable_recommend_alc() { return enable_recommend_alc_; }

  void set_enable_cutin_threshold(bool enable_cutin_threshold_adjust) {
    enable_cutin_threshold_adjust_ = enable_cutin_threshold_adjust;
  }

  bool get_enable_cutin_threshold() { return enable_cutin_threshold_adjust_; }

  void set_lon_cutin_threshold(double lon_cutin_threshold) {
    lon_cutin_threshold_ = lon_cutin_threshold;
  }

  double get_lon_cutin_threshold() { return lon_cutin_threshold_; }

  void set_enable_lateral_dogde(bool enable_lateral_dogde) {
    enable_lateral_dogde_ = enable_lateral_dogde;
  }

  bool enable_lateral_dogde() { return enable_lateral_dogde_; }

  void set_force_stop(bool is_force_stop) { is_force_stop_ = is_force_stop; };

  bool is_force_stop() { return is_force_stop_; };

  void set_mrc_inlane_brake(bool is_mrc_inlane_brake) {
    is_mrc_inlane_brake_ = is_mrc_inlane_brake;
  };

  bool is_mrc_inlane_brake() { return is_mrc_inlane_brake_; };

  void set_ego_blinker(const maf_vehicle_status::VehicleLight &vehicle_light) {
    ego_state_manager_.set_ego_blinker(vehicle_light);
  }

  void set_navi_ttc_gear(uint8_t navi_ttc_gear) {
    navi_ttc_gear_ = navi_ttc_gear;
  }
  void set_yawrate(double yawrate) { yawrate_ = yawrate; }
  double get_yawrate() { return yawrate_; }

  uint8_t get_navi_ttc_gear() { return navi_ttc_gear_; }

  bool get_throttle_override() { return throttle_override_; }

  LcaStatus get_lca_status() { return lca_status_; }
  DrivingModelConfig get_driving_model_config() {
    return driving_model_config_;
  }

private:
  void update_members();
  bool construct_baseline_priority();
  void map_fusion_prediction_info();
  void generate_prediction_delay_time();
  void transform_fusion_to_prediction();
  bool construct_baseline_infos();

  void feed_map_info(const MSDMapInfo &map_info);
  void feed_vehicle_dbw_status(bool flag) {
    last_vehicle_dbw_status_ = vehicle_dbw_status_;
    vehicle_dbw_status_ = flag;
    static double start_auto_drive_time = MTIME()->timestamp().sec();
    if (!last_vehicle_dbw_status_ && vehicle_dbw_status_) {
      start_auto_drive_time = MTIME()->timestamp().sec();
    }

    if (vehicle_dbw_status_) {
      enter_auto_drive_time_ =
          MTIME()->timestamp().sec() - start_auto_drive_time;
    } else {
      enter_auto_drive_time_ = 0.0;
    }
  }

  void
  feed_vehicle_status(const maf_vehicle_status::VehicleStatus &vehicle_status,
                      const double steer_angle_offset_deg) {
    ego_state_manager_.set_ego_pose_and_vel(vehicle_status);
    ego_state_manager_.set_ego_steer_angle(vehicle_status,
                                           steer_angle_offset_deg);
    ego_state_manager_.set_ego_enu(vehicle_status);
    ego_state_manager_.set_ego_acc(vehicle_status);
    wheel_speed_report_ = vehicle_status.wheel_velocity;
    vehicle_light_report_ = vehicle_status.vehicle_light.vehicle_light_data;
    vehicle_status_ = vehicle_status;
    throttle_override_ = vehicle_status.throttle.throttle_data.override;
  }

  void feed_fusion_info(
      const std::vector<maf_perception_interface::PerceptionFusionObjectData>
          &fusion_polygon_array) {
    fusion_info_ = fusion_polygon_array;
  }

  void feed_prediction_info(
      const std::vector<PredictionObject> &prediction_object_array) {
    prediction_info_ = prediction_object_array;

    lateral_obstacle_->set_prediction_update(true);
  }

  void
  feed_perception_vision_lane(const maf_perception_interface::RoadLinePerception
                                  &perception_vision_lane) {
    perception_vision_lane_ = perception_vision_lane;
  }

  void feed_perception_lidar_road_edge(
      const maf_perception_interface::RoadLinePerception
          &perception_lidar_road_edge) {
    perception_lidar_road_edge_ = perception_lidar_road_edge;
  }

  void feed_perception_radar(
      const maf_perception_interface::RadarPerceptionResult &perception_radar) {
    perception_radar_ = perception_radar;
  }

  void feed_lca_status(const LcaStatus lca_status) { lca_status_ = lca_status; }
  void
  feed_driving_model_config(const DrivingModelConfig driving_model_config) {
    driving_model_config_ = driving_model_config;
  }

  void feed_perception_fusion_object(
      const maf_perception_interface::PerceptionFusionObjectResult
          perception_fusion_object) {
    perception_fusion_object_ = perception_fusion_object;
    for (const auto &info : perception_fusion_object_.reserved_infos) {
      if (info.compare(0, 14, "cipv_fn_status") == 0) {
        if (stoi(&info[info.length() - 1]) == 1) {
          cipv_fn_ = true;
        } else {
          cipv_fn_ = false;
        }
      }

      if (info.size() >= 13 && info.compare(0, 11, "cipv_fn_tid") == 0) {
        cipv_fn_tid_ = stoi(info.substr(12));
      }
    }
  }
    
  void feed_traffic_light_info(
      double time,
      const std::vector<maf_perception_interface::TrafficLightDecision>
          &fusion) {
    traffic_light_info_.valid = true;
    traffic_light_info_.header = time;
    traffic_light_info_.info = fusion;
  }

  void feed_gear_report(const maf_vehicle_status::GearData &gear_report) {
    gear_report_ = gear_report;
  }

  void feed_aimed_poi_info(const AimedPoiInfo &aimed_poi_info) {
    parking_map_info_.aimed_poi_info = aimed_poi_info;
  }
  void feed_parking_out_info(const ParkingOutInfo &parking_out_info) {
    parking_map_info_.parking_out_info = parking_out_info;
  }
  void
  feed_fuzzy_search_results(const std::vector<FuzzySearchResult> &results) {
    parking_map_info_.fuzzy_search_results = results;
  }
  void feed_accurate_search_results(
      const std::vector<AccurateSearchResult> &results) {
    parking_map_info_.accurate_search_results = results;
  }

  void feed_parking_human(const std::vector<VisionHuman> &vision_human) {
    parking_vision_info_.human_array = vision_human;
  }
  void feed_parking_car(const std::vector<VisionCar> &vision_car) {
    parking_vision_info_.car_array = vision_car;
  }
  void feed_parking_freespace(const VisionFreeSpace &vision_free_space) {
    parking_vision_info_.free_space = vision_free_space;
  }

  // void feed_parking_ui_request(const ParkingUiRequest &request);
  void feed_square_map_response(const SquareMapResponse &square_map_response) {
    square_map_response_ = square_map_response;
  }

private:
  std::unique_ptr<LateralObstacle> lateral_obstacle_;
  std::shared_ptr<TrafficLightDecision> traffic_light_decision_;

  SceneType scene_type_ = SceneType::URBAN;
  DrivingModelConfig driving_model_config_ =
      DrivingModelConfig::NO_REQUEST_DrivingModel;
  CartEgoStateManager ego_state_manager_;
  std::vector<maf_perception_interface::PerceptionFusionObjectData>
      fusion_info_;
  std::vector<PredictionObject> prediction_info_;
  std::vector<PredictionObject> truncated_prediction_info_;
  std::vector<PredictionObject> full_prediction_info_;
  maf_perception_interface::RoadLinePerception perception_vision_lane_;
  maf_perception_interface::RoadLinePerception perception_lidar_road_edge_;
  maf_perception_interface::RadarPerceptionResult perception_radar_;
  LcaStatus lca_status_;
  maf_perception_interface::PerceptionFusionObjectResult
      perception_fusion_object_;
  TrafficLight traffic_light_info_;
  MapInfoManager map_info_manager_;
  std::unordered_map<int, std::array<std::vector<RefPointFrenet>, 3>>
      map_info_frenet_;

  std::shared_ptr<ReflineGenerator> refline_generator_;

  // baseline info
  std::unordered_map<int, std::shared_ptr<BaseLineInfo>> baseline_infos_;
  FrenetCoordinateSystemParameters frenet_parameters_;

  bool vehicle_dbw_status_{false};
  double enter_auto_drive_time_{0.0};
  bool last_vehicle_dbw_status_{false};
  bool dbw_status_{false};
  bool pretreatment_status_{true};
  int ignore_obs_id_ = -10000;
  double ilc_limit_velocity_ = 0.0;
  double ilc_limit_time_ = 10.0;

  // MPC
  double process_calculate_time_;
  bool planner_success_;
  int64_t planning_loops_;

  // drving mode and style
  bool is_acc_mode_{false};
  bool last_is_acc_mode_{false};
  bool is_ddmap_{false};
  bool is_ddmap_intersection_{false};
  bool use_eftp_{false};
  bool lateral_use_eftp_{false};
  double baseline_overlap_{1.0};
  bool reset_{false};
  double v_curv_{33.3};
  double a_curv_{0.0};
  double max_curv_{0.0};
  double max_curv_distance_{0.0};
  DrivingStyle driving_style_{DRIVING_STYLE_NORMAL};
  LaneChangingStyle lane_changing_style_{LANECHANGING_STYLE_NORMAL};
  uint8_t navi_ttc_gear_{maf_system_manager::NaviTimeDistance::NORMAL};

  // interection refline generator
  pass_intersection_planner::ReflineCondition condition_;
  pass_intersection_planner::ReflineCondition last_condition_;
  pass_intersection_planner::ApfAttractType attract_type_;
  pass_intersection_planner::ApfAttractType last_attract_type_;
  bool apf_lat_far_flag_{false};
  double current_last_car_point_x_;
  Point2D current_last_enu_point_;

  bool enable_ilc_{true};
  bool enable_alc_{true};
  bool enable_recommend_alc_{false};
  bool enable_lateral_dogde_{true};
  double lon_cutin_threshold_{0.2};
  bool enable_cutin_threshold_adjust_{false};

  bool is_force_stop_{false};
  bool is_mrc_inlane_brake_{false};
  double yawrate_{0.0};

  bool throttle_override_ = false;

  maf_vehicle_status::VehicleStatus vehicle_status_;
  maf_vehicle_status::VehicleLightData vehicle_light_report_;
  maf_vehicle_status::GearData gear_report_;
  maf_vehicle_status::WheelVelocity wheel_speed_report_;
  ParkingMapInfo parking_map_info_;
  ParkingVisionInfo parking_vision_info_;
  // ParkingUiRequest parking_ui_request_;
  SquareMapResponse square_map_response_;

  friend class PlanningTask;

  bool cipv_fn_ = false;
  int cipv_fn_tid_ = -1;
};

} // namespace msquare

#endif
