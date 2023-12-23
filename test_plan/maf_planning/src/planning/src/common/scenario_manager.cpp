#include "common/scenario_manager.h"
#include "planner/tasks/task_factory.h"
#include "planning/common/common.h"

#include "common/config_context.h"
#include "common/vehicle_model/vehicle_model.h"
#include "planner/arbitrators/arbitrator_cost_strategy.h"
#include "planner/scenarios/scenario_factory.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace msquare {

ScenarioManager::ScenarioManager(const std::shared_ptr<WorldModel> &world_model)
    : world_model_(world_model) {

  // ConfigurationContext::Instance()->load_params();
  auto config_file_dir = PlanningContext::Instance()->get_config_file_dir();

  ConfigurationContext::Instance()->load_params_from_json(config_file_dir);
  TaskFactory::Init(
      ConfigurationContext::Instance()->get_default_tasks_params());
  ScenarioFactory::Init(ConfigurationContext::Instance()->get_params());
  ArbitratorCostStrategy::init();
  (void)common::VehicleModel::LoadVehicleModelConfig();
  state_machine_ = std::make_shared<MSDStateMachine>(world_model_);

  data_driven_planner_ =
      std::make_unique<msquare::ddp::DataDrivenPlanner>(world_model_);
}

ScenarioManager::~ScenarioManager() {}

bool ScenarioManager::execute_planning() {
  // TODO: get synthetic configuration from msd_worldmodel and reset
  // ConfigurationContext
  // auto scenario_config_file_dir = SCENARIO_CONFIG_JSON_FILE_DIR;
  auto config_file_dir = PlanningContext::Instance()->get_config_file_dir();
  SyntheticConfiguration current_conf =
      ConfigurationContext::Instance()->synthetic_config();
  current_conf.scene_type = world_model_->get_map_info().scene_type_;
  if (world_model_->is_ddmap()) {
    current_conf.scene_type = "highway";
  }
  if (ConfigurationContext::Instance()->reset_synthetic_config(
          current_conf, config_file_dir, world_model_->get_driving_style())) {
    TaskFactory::Init(
        ConfigurationContext::Instance()->get_default_tasks_params());
    ScenarioFactory::Init(ConfigurationContext::Instance()->get_params());
  }

  MSD_LOG(INFO, "dbw : %d, last_dbw : %d", world_model_->get_dbw_status(),
          world_model_->get_last_vehicle_dbw_status());
  // load planner config only at the moment of entry auto_drive
  if (world_model_->get_vehicle_dbw_status() &&
      !world_model_->get_last_vehicle_dbw_status()) {
    MSD_LOG(INFO, "reload planner config");
    ConfigurationContext::Instance()->load_planner_config(config_file_dir);
  }

  ScenarioStatus status = check_scenario_status();

  if (status == ScenarioStatus::NOT_READY) {
    MSD_LOG(INFO, "scenario not ready");
    return false;
  } else if (status == ScenarioStatus::PRETREATMENT_FAIL) {
    MSD_LOG(INFO, "pretreatment fail");
    return false;
  }

  // do not use eftp traj when lc trigged or in intersection
  const auto &lateral_output =
      PlanningContext::Instance()->lateral_behavior_planner_output();
  const int in_intersection_count_thres = 6; // delay 0.5s
  static int in_intersection_count = in_intersection_count_thres;
  bool is_in_intersection = false;
  if (world_model_->get_refline_condition().reflinecondition ==
          pass_intersection_planner::ReflineCondition::
              FIRST_HALF_INTERSECTION ||
      world_model_->get_refline_condition().reflinecondition ==
          pass_intersection_planner::ReflineCondition::IN_INTERSECTION ||
      world_model_->get_refline_condition().reflinecondition ==
          pass_intersection_planner::ReflineCondition::LAST_HALF_INTERSECTION) {
    in_intersection_count = in_intersection_count_thres;
  } else {
    in_intersection_count = std::max(in_intersection_count - 1, 0);
  }
  if (in_intersection_count > 0) {
    is_in_intersection = true;
  } else {
    is_in_intersection = false;
  }
  if (lateral_output.lc_request != "none" || is_in_intersection ||
      world_model_->is_mrc_inlane_brake()) {
    world_model_->set_use_eftp(false);
  }

  double start_time = MTIME()->timestamp().ms();

  if (ConfigurationContext::Instance()
          ->planner_config()
          .common_config.use_eftp) {
    (void)data_driven_planner_->run();
  } else {
    auto &ddp_trajectory_info = ddp::DdpContext::Instance()
                                    ->model_result_manager()
                                    ->get_mutable_ddp_trajectory_info();
    ddp_trajectory_info.clear();
    auto &ddp_traj = ddp::DdpContext::Instance()
                         ->model_result_manager()
                         ->get_mutable_current_lane_ddp_trajectory();
    ddp_traj.trajectory.clear();
  }

  double end_time = MTIME()->timestamp().ms();
  MSD_LOG(ERROR, "time_cost, ddp time: %f", end_time - start_time);
  CostTime task_cost2 = CostTime{"ddp", end_time - start_time};
  auto *planner_debug = PlanningContext::Instance()->mutable_planner_debug();
  planner_debug->cost_time.emplace_back(task_cost2);

  const auto &ddp_traj = ddp::DdpContext::Instance()
                             ->model_result_manager()
                             ->get_current_lane_ddp_trajectory();
  if (ddp_traj.trajectory.empty()) {
    world_model_->set_use_eftp(false);
  }

  return state_machine_->update() &&
         PlanningContext::Instance()->planning_status().planning_success;
}

template <typename T>
std::string vector_to_string(const std::vector<T> &source) {
  std::string dest = "[";

  if (source.size() > 0) {
    if (typeid(T).hash_code() == typeid(std::string).hash_code()) {
      dest += "\"";
      dest += source[0];
      dest += "\"";

      for (size_t i = 1; i < source.size(); i++) {
        dest += " \"";
        dest += source[i];
        dest += "\"";
      }
    } else {
      dest += std::to_string(source[0]);
      for (size_t i = 1; i < source.size(); i++) {
        dest += " " + std::to_string(source[i]);
      }
    }
  }

  dest += "]";
  return dest;
}

void ScenarioManager::log_debug_info() {
  MLOG_PROFILING("log_debug_info");
  const PlanningStatus &planning_status =
      PlanningContext::Instance()->planning_status();
  const PlanningResult &planning_result = planning_status.planning_result;

  auto &lateral_output =
      PlanningContext::Instance()->lateral_behavior_planner_output();

  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, lat_error,
                    planning_result.lat_error);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, lon_error,
                    planning_result.lon_error);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, flag_closelead,
                    planning_result.flag_closelead);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, flag_invalid,
                    planning_result.flag_invalid);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, flag_type0,
                    planning_result.flag_type0);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, flag_softbrake,
                    planning_result.flag_softbrake);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, flag_fastcutin,
                    planning_result.flag_fastcutin);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, v_set, planning_result.v_set);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, ds_set,
                    planning_result.ds_set);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, t_set, planning_result.t_set);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, v_limit,
                    planning_result.v_limit);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, v_limit_map,
                    planning_result.v_limit_map);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, v_limit_map_current,
                    planning_result.v_limit_map_current);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, num_yield,
                    planning_result.num_yield);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, vl_yield,
                    planning_result.vl_yield);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, dsl_yield,
                    planning_result.dsl_yield);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, type_yield,
                    planning_result.type_yield);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, id_yield,
                    planning_result.id_yield);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, tag_yield,
                    planning_result.tag_yield);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, type_merge,
                    planning_result.type_merge);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, dis2cross,
                    planning_result.dis2cross);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, dsl_overtake,
                    planning_result.dsl_overtake);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, type_overtake,
                    planning_result.type_overtake);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, id_overtake,
                    planning_result.id_overtake);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, dis2merge,
                    planning_result.dis2merge);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, cutin_score,
                    planning_result.cutin_score);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, probability,
                    planning_result.probability);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, ds_lon_front,
                    planning_result.ds_lon_front);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, ttc_lon_front,
                    planning_result.ttc_lon_front);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, dt_lon_front,
                    planning_result.dt_lon_front);
  MSD_WRN_TAG_STR(longitudinal_planning_debug, id_nudge,
                  vector_to_string(planning_result.id_nudge).c_str());

  mjson::Json::array json_array;
  for (auto &item : lateral_output.avd_info) {
    mjson::Json::object avd_info;
    avd_info["id"] = mjson::Json(item.id);
    avd_info["property"] = mjson::Json(item.property);
    avd_info["ignore"] = mjson::Json(item.ignore);
    avd_info["avd_direction"] = mjson::Json(item.avd_direction);
    json_array.push_back(mjson::Json(avd_info));
  }
  mjson::Json avd_info_json(json_array);

  MSD_WRN_TAG_STR(longitudinal_planning_debug, avd_info,
                  avd_info_json.dump().c_str());

  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, lead_one_drel,
                    lateral_output.lead_one_drel);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, lead_one_vrel,
                    lateral_output.lead_one_vrel);

  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, v_target,
                    planning_result.v_target);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, a_target,
                    planning_result.a_target);
  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, dis_to_stop,
                    planning_result.dist_to_stop);

  std::vector<int> traffic_light_decision{planning_result.traffic_light_state,
                                          planning_result.stop_flag,
                                          planning_result.is_passed_stop_line};

  MSD_WRN_TAG_STR(longitudinal_planning_debug, traffic_light_decision,
                  vector_to_string(traffic_light_decision).c_str());

  MSD_WRN_TAG_STR(
      longitudinal_planning_debug, lon_follow_obstacles,
      vector_to_string(planning_result.lon_follow_obstacles).c_str());

  MSD_WRN_TAG_STR(
      longitudinal_planning_debug, lon_overtake_obstacles,
      vector_to_string(planning_result.lon_overtake_obstacles).c_str());

  MSD_WRN_TAG_STR(
      longitudinal_planning_debug, lat_nudge_obstacles,
      vector_to_string(planning_result.lat_nudge_obstacles).c_str());

  MSD_WRN_TAG_STR(longitudinal_planning_debug, acc_output,
                  vector_to_string(planning_result.acc_output).c_str());

  MSD_WRN_TAG_STR(longitudinal_planning_debug, jerk_output,
                  vector_to_string(planning_result.jerk_output).c_str());

  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, min_acc,
                    planning_result.min_acc);

  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, max_acc,
                    planning_result.max_acc);

  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, min_jerk,
                    planning_result.min_jerk);

  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, max_jerk,
                    planning_result.max_jerk);

  MSD_WRN_TAG_VALUE(longitudinal_planning_debug, lat_plan_status,
                    planning_result.lat_plan_status);

  MSD_WRN_TAG_STR(longitudinal_planning_debug, id_clear,
                  vector_to_string(planning_result.id_clear).c_str());

  std::string plan_msg;
  create_lateral_decision_info_msg(plan_msg);
  MSD_WRN_TAG_STR(lateral_decision_info, json_str, plan_msg.c_str());

  std::string pld_msg;
  create_lateral_decision_debug_msg(pld_msg);
  MSD_WRN_TAG_STR(lateral_decision_debug, json_str, pld_msg.c_str());

  std::string fusion_msg;
  create_prediction_object_info_msg(fusion_msg);

  MSD_WRN_TAG_STR(prediction_object_info, json_str, fusion_msg.c_str());

  std::string mmp_msg;
  create_map_planning_info_msg(mmp_msg);

  MSD_WRN_TAG_STR(map_planning_info, json_str, mmp_msg.c_str());

  std::string acc_refline_msg;
  create_acc_refline_msg(acc_refline_msg);

  MSD_WRN_TAG_STR(refline_acc_info, json_str, acc_refline_msg.c_str());
}

ScenarioManager::ScenarioStatus ScenarioManager::check_scenario_status() {
  if (!world_model_->get_pretreatment_status()) {
    return ScenarioStatus::PRETREATMENT_FAIL;
  }

  if (!world_model_->get_dbw_status()) {
    MSD_LOG(WARN, "DBW_Disable");
  }

  return ScenarioStatus::READY;
}

void update_json_object_for_car_cnt(int track_id, const CarCount &car_cnt,
                                    rapidjson::Value &dest,
                                    rapidjson::Document::AllocatorType &alloc) {
  dest.AddMember("id", rapidjson::Value().SetInt(track_id), alloc);
  dest.AddMember("neg", rapidjson::Value().SetInt(car_cnt.neg), alloc);
  dest.AddMember("pos", rapidjson::Value().SetInt(car_cnt.pos), alloc);
}

void update_json_object_for_track_info(
    const TrackInfo &source, rapidjson::Value &dest,
    rapidjson::Document::AllocatorType &alloc) {
  dest.AddMember("trackId", rapidjson::Value().SetInt(source.track_id), alloc);
  dest.AddMember("dRel", rapidjson::Value().SetDouble(source.d_rel), alloc);
  dest.AddMember("vRel", rapidjson::Value().SetDouble(source.v_rel), alloc);
}

void update_json_object_for_fusion(const TrackedObject &source,
                                   rapidjson::Value &dest,
                                   rapidjson::Document::AllocatorType &alloc) {
  dest.AddMember("trackId", rapidjson::Value().SetInt(source.track_id), alloc);
  dest.AddMember("type", rapidjson::Value().SetInt(source.type), alloc);
  dest.AddMember("dRel", rapidjson::Value().SetDouble(source.d_rel), alloc);
  dest.AddMember("vRel", rapidjson::Value().SetDouble(source.v_rel), alloc);
  dest.AddMember("yRel", rapidjson::Value().SetDouble(source.y_rel), alloc);
  dest.AddMember("vEgo", rapidjson::Value().SetDouble(source.v_ego), alloc);
  dest.AddMember("vLead", rapidjson::Value().SetDouble(source.v_lead), alloc);
  dest.AddMember("vLat", rapidjson::Value().SetDouble(source.v_lat), alloc);
  dest.AddMember("dPath", rapidjson::Value().SetDouble(source.d_path), alloc);
  dest.AddMember("dPathPos", rapidjson::Value().SetDouble(source.d_path_pos),
                 alloc);
  dest.AddMember("dPathSelf", rapidjson::Value().SetDouble(source.d_path_self),
                 alloc);
  dest.AddMember("dPathSelfPos",
                 rapidjson::Value().SetDouble(source.d_path_self_pos), alloc);
  dest.AddMember("dMaxCPath", rapidjson::Value().SetDouble(source.d_max_cpath),
                 alloc);
  dest.AddMember("dMinCPath", rapidjson::Value().SetDouble(source.d_min_cpath),
                 alloc);
  dest.AddMember("isNCar", rapidjson::Value().SetBool(source.is_ncar), alloc);
  dest.AddMember("ncarCnt", rapidjson::Value().SetDouble(source.ncar_count),
                 alloc);
  dest.AddMember("intersection",
                 rapidjson::Value().SetDouble(source.trajectory.intersection),
                 alloc);
  dest.AddMember("is_lead", rapidjson::Value().SetBool(source.is_lead), alloc);
  dest.AddMember("leadone_confidence_cnt",
                 rapidjson::Value().SetDouble(source.leadone_confidence_cnt),
                 alloc);
  dest.AddMember("leadtwo_confidence_cnt",
                 rapidjson::Value().SetDouble(source.leadtwo_confidence_cnt),
                 alloc);
  dest.AddMember("length", rapidjson::Value().SetDouble(source.length), alloc);
  dest.AddMember("width", rapidjson::Value().SetDouble(source.width), alloc);
  dest.AddMember("theta", rapidjson::Value().SetDouble(source.theta), alloc);
  dest.AddMember("speed_yaw", rapidjson::Value().SetDouble(source.speed_yaw),
                 alloc);
  dest.AddMember("vy_rel", rapidjson::Value().SetDouble(source.vy_rel), alloc);
  dest.AddMember("center_x", rapidjson::Value().SetDouble(source.center_x),
                 alloc);
  dest.AddMember("center_y", rapidjson::Value().SetDouble(source.center_y),
                 alloc);
}

void update_json_object_for_boundary_segment(
    const maf_worldmodel::LaneBoundarySegment &source, rapidjson::Value &dest,
    rapidjson::Document::AllocatorType &alloc) {
  dest.AddMember("length", rapidjson::Value().SetDouble(source.length), alloc);
  dest.AddMember(
      "type",
      rapidjson::Value().SetInt(static_cast<int>(source.type.value.value)),
      alloc);
}

void update_json_object_for_polyline(
    const LaneBoundaryPolyline &source, rapidjson::Value &dest,
    rapidjson::Document::AllocatorType &alloc) {
  dest.AddMember("track_id", rapidjson::Value().SetInt(source.track_id), alloc);

  if (!source.poly_coefficient.empty()) {
    rapidjson::Value coefficient_array(rapidjson::kArrayType);
    for (auto item : source.poly_coefficient) {
      coefficient_array.PushBack(rapidjson::Value().SetDouble(item), alloc);
    }
    dest.AddMember("coefficient", coefficient_array, alloc);
  } else {
    dest.AddMember("coefficient", rapidjson::Value().SetNull(), alloc);
  }

  dest.AddMember("begin",
                 rapidjson::Value().SetDouble(source.available_interval.begin),
                 alloc);
  dest.AddMember("end",
                 rapidjson::Value().SetDouble(source.available_interval.end),
                 alloc);
}

void update_json_object_for_lane_boundary(
    const maf_worldmodel::LaneBoundary &source, rapidjson::Value &dest,
    rapidjson::Document::AllocatorType &alloc) {
  if (!source.segments.empty()) {
    rapidjson::Value json_array(rapidjson::kArrayType);
    for (auto &segment : source.segments) {
      rapidjson::Value segment_json(rapidjson::kObjectType);
      update_json_object_for_boundary_segment(segment, segment_json, alloc);
      json_array.PushBack(segment_json, alloc);
    }

    dest.AddMember("segment", json_array, alloc);
  } else {
    dest.AddMember("segment", rapidjson::Value().SetNull(), alloc);
  }

  rapidjson::Value polyline_json(rapidjson::kObjectType);
  update_json_object_for_polyline(source.polyline, polyline_json, alloc);
  dest.AddMember("polylinel", polyline_json, alloc);
}

void update_json_object_for_refline_point(
    const ReferenceLinePointDerived &source, rapidjson::Value &dest,
    rapidjson::Document::AllocatorType &alloc) {
  constexpr double kMaxDistance = 10000.0;

  dest.AddMember("car_point_x",
                 rapidjson::Value().SetDouble(source.car_point.x), alloc);
  dest.AddMember("car_point_y",
                 rapidjson::Value().SetDouble(source.car_point.y), alloc);
  dest.AddMember("car_point_z",
                 rapidjson::Value().SetDouble(source.car_point.z), alloc);

  dest.AddMember("enu_point_x",
                 rapidjson::Value().SetDouble(source.enu_point.x), alloc);
  dest.AddMember("enu_point_y",
                 rapidjson::Value().SetDouble(source.enu_point.y), alloc);
  dest.AddMember("enu_point_z",
                 rapidjson::Value().SetDouble(source.enu_point.z), alloc);

  if (source.curvature >= -0.05 && source.curvature <= 0.05) {
    dest.AddMember("curvature", rapidjson::Value().SetDouble(source.curvature),
                   alloc);
  } else {
    dest.AddMember("curvature", rapidjson::Value().SetNull(), alloc);
  }

  if (source.distance_to_left_road_border < kMaxDistance) {
    dest.AddMember(
        "distance_left_road_border",
        rapidjson::Value().SetDouble(source.distance_to_left_road_border),
        alloc);
  } else {
    dest.AddMember("distance_left_road_border",
                   rapidjson::Value().SetDouble(kMaxDistance), alloc);
  }

  if (source.distance_to_right_road_border < kMaxDistance) {
    dest.AddMember(
        "distance_right_road_border",
        rapidjson::Value().SetDouble(source.distance_to_right_road_border),
        alloc);
  } else {
    dest.AddMember("distance_right_road_border",
                   rapidjson::Value().SetDouble(kMaxDistance), alloc);
  }

  if (source.distance_to_left_lane_border < kMaxDistance) {
    dest.AddMember(
        "distance_left_lane_border",
        rapidjson::Value().SetDouble(source.distance_to_left_lane_border),
        alloc);
  } else {
    dest.AddMember("distance_left_lane_border",
                   rapidjson::Value().SetDouble(kMaxDistance), alloc);
  }

  if (source.distance_to_right_lane_border < kMaxDistance) {
    dest.AddMember(
        "distance_right_lane_border",
        rapidjson::Value().SetDouble(source.distance_to_right_lane_border),
        alloc);
  } else {
    dest.AddMember("distance_right_lane_border",
                   rapidjson::Value().SetDouble(kMaxDistance), alloc);
  }

  if (source.distance_to_left_obstacle < kMaxDistance) {
    dest.AddMember(
        "distance_left_obstacle",
        rapidjson::Value().SetDouble(source.distance_to_left_obstacle), alloc);
  } else {
    dest.AddMember("distance_left_obstacle",
                   rapidjson::Value().SetDouble(kMaxDistance), alloc);
  }

  if (source.distance_to_right_obstacle < kMaxDistance) {
    dest.AddMember(
        "distance_right_obstacle",
        rapidjson::Value().SetDouble(source.distance_to_right_obstacle), alloc);
  } else {
    dest.AddMember("distance_right_obstacle",
                   rapidjson::Value().SetDouble(kMaxDistance), alloc);
  }

  if (source.lane_width < kMaxDistance) {
    dest.AddMember("lane_width",
                   rapidjson::Value().SetDouble(source.lane_width), alloc);
  } else {
    dest.AddMember("lane_width", rapidjson::Value().SetDouble(kMaxDistance),
                   alloc);
  }

  dest.AddMember("max_velocity",
                 rapidjson::Value().SetDouble(source.max_velocity), alloc);
}

void update_json_object_for_refline_segment(
    const ReferenceLineSegment &source, rapidjson::Value &dest,
    rapidjson::Document::AllocatorType &alloc) {
  dest.AddMember("begin_index", rapidjson::Value().SetInt(source.begin_index),
                 alloc);
  dest.AddMember("end_index", rapidjson::Value().SetInt(source.end_index),
                 alloc);
  dest.AddMember(
      "direction",
      rapidjson::Value().SetInt(static_cast<int>(source.direction.value)),
      alloc);
  dest.AddMember(
      "lane_type",
      rapidjson::Value().SetInt(static_cast<int>(source.lane_type.value)),
      alloc);
  dest.AddMember("in_intersection",
                 rapidjson::Value().SetBool(source.is_in_intersection), alloc);
  dest.AddMember("in_route", rapidjson::Value().SetBool(source.is_in_route),
                 alloc);
}

void update_json_object_for_reference_line(
    const std::vector<ReferenceLinePointDerived> reference_line_points,
    const maf_worldmodel::LaneData &source, rapidjson::Value &dest,
    rapidjson::Document::AllocatorType &alloc, int32_t relative_id,
    const std::shared_ptr<WorldModel> &world_model) {

  if (relative_id != 0) {
    return;
  }

  auto *planning_status =
      PlanningContext::Instance()->mutable_planning_status();
  auto x = mjson::Json::array(200, mjson::Json(0.0));
  auto y = mjson::Json::array(200, mjson::Json(0.0));
  auto z = mjson::Json::array(200, mjson::Json(0.0));
  x.clear();
  y.clear();
  z.clear();
  if (!reference_line_points.empty()) {
    rapidjson::Value json_array(rapidjson::kArrayType);
    for (auto &refline_point : reference_line_points) {
      double curr_point_x = refline_point.car_point.x;

      x.emplace_back(mjson::Json(refline_point.enu_point.x));
      y.emplace_back(mjson::Json(refline_point.enu_point.y));
      z.emplace_back(mjson::Json(refline_point.enu_point.z));

      rapidjson::Value point_json(rapidjson::kObjectType);
      update_json_object_for_refline_point(refline_point, point_json, alloc);
      json_array.PushBack(point_json, alloc);
    }

    auto refline_xy_json = mjson::Json(mjson::Json::object());
    refline_xy_json["x"] = mjson::Json(x);
    refline_xy_json["y"] = mjson::Json(y);
    refline_xy_json["z"] = mjson::Json(z);

    planning_status->planning_result.debug_json["refline_xy"] =
        mjson::Json(refline_xy_json);

    dest.AddMember("refline_points", json_array, alloc);
  } else {
    dest.AddMember("refline_points", rapidjson::Value().SetNull(), alloc);
  }

  if (!source.reference_line.reference_line_segments.empty()) {
    rapidjson::Value json_array(rapidjson::kArrayType);
    for (auto &refline_segment :
         source.reference_line.reference_line_segments) {
      rapidjson::Value segment_json(rapidjson::kObjectType);
      update_json_object_for_refline_segment(refline_segment, segment_json,
                                             alloc);
      json_array.PushBack(segment_json, alloc);
    }

    dest.AddMember("refline_segments", json_array, alloc);
  } else {
    dest.AddMember("refline_segments", rapidjson::Value().SetNull(), alloc);
  }
}

void update_json_object_for_lane_data(
    const MSDMapInfo &map_info, const maf_worldmodel::LaneData &source,
    rapidjson::Value &dest, rapidjson::Document::AllocatorType &alloc,
    const std::shared_ptr<WorldModel> &world_model) {
  dest.AddMember("relative_id", rapidjson::Value().SetInt(source.relative_id),
                 alloc);
  dest.AddMember(
      "lane_mark",
      rapidjson::Value().SetInt(static_cast<int>(source.lane_marks.value)),
      alloc);
  dest.AddMember(
      "lane_type",
      rapidjson::Value().SetInt(static_cast<int>(source.lane_type.value)),
      alloc);

  if (source.left_lane_boundary.existence) {
    rapidjson::Value boundary_json(rapidjson::kObjectType);
    update_json_object_for_lane_boundary(source.left_lane_boundary,
                                         boundary_json, alloc);
    dest.AddMember("left_lane_boundary", boundary_json, alloc);
  } else {
    dest.AddMember("left_lane_boundary", rapidjson::Value().SetNull(), alloc);
  }

  if (source.right_lane_boundary.existence) {
    rapidjson::Value boundary_json(rapidjson::kObjectType);
    update_json_object_for_lane_boundary(source.right_lane_boundary,
                                         boundary_json, alloc);
    dest.AddMember("right_lane_boundary", boundary_json, alloc);
  } else {
    dest.AddMember("right_lane_boundary", rapidjson::Value().SetNull(), alloc);
  }
  switch (source.relative_id) {
  case -1:
    update_json_object_for_reference_line(map_info.left_refline_points(),
                                          source, dest, alloc,
                                          source.relative_id, world_model);
    break;
  case 0:
    update_json_object_for_reference_line(map_info.current_refline_points(),
                                          source, dest, alloc,
                                          source.relative_id, world_model);
    break;
  case 1:
    update_json_object_for_reference_line(map_info.right_refline_points(),
                                          source, dest, alloc,
                                          source.relative_id, world_model);
    break;
  default:
    break;
  }
}

void ScenarioManager::output_lat_dec_info() {
  auto *planner_debug = PlanningContext::Instance()->mutable_planner_debug();
  auto &lateral_output =
      PlanningContext::Instance()->lateral_behavior_planner_output();

  planner_debug->lat_dec_info.lc_back_reason = state_machine_->lc_back_reason();
  planner_debug->lat_dec_info.lc_invalid_reason =
      state_machine_->lc_invalid_reason();
  planner_debug->lat_dec_info.lc_request = lateral_output.lc_request;
  planner_debug->lat_dec_info.lc_request_source =
      lateral_output.lc_request_source;
  planner_debug->lat_dec_info.state_change_reason =
      state_machine_->state_change_reason();
}

void ScenarioManager::create_lateral_decision_info_msg(std::string &msg) {
  auto &lateral_output =
      PlanningContext::Instance()->lateral_behavior_planner_output();
  auto &change_lane_status =
      PlanningContext::Instance()->planning_status().lane_status.change_lane;
  auto virtual_lane_mgr = state_machine_->virtual_lane_mgr();
  auto &f_refline = virtual_lane_mgr->mutable_fix_refline();
  auto &ego_state =
      world_model_->get_baseline_info(f_refline.position())->get_ego_state();

  namespace rp = rapidjson;
  rp::Document json_doc;
  json_doc.SetObject();
  auto &alloc = json_doc.GetAllocator();

  json_doc.AddMember("distIntersect",
                     rp::Value().SetDouble(lateral_output.dist_intersect),
                     alloc);
  json_doc.AddMember("fixRefline",
                     rp::Value().SetString(lateral_output.which_lane.c_str(),
                                           lateral_output.which_lane.size()),
                     alloc);
  json_doc.AddMember("ignoreTrackId",
                     rp::Value().SetInt(lateral_output.track_id), alloc);
  json_doc.AddMember(
      "isRedLightStop",
      rp::Value().SetBool(
          world_model_->get_traffic_light_decision()->get_stop_flag()),
      alloc);
  json_doc.AddMember("laneWidth",
                     rp::Value().SetDouble(lateral_output.flane_width), alloc);
  json_doc.AddMember("latOffset",
                     rp::Value().SetDouble(lateral_output.lat_offset), alloc);
  json_doc.AddMember(
      "lcBack", rp::Value().SetBool(state_machine_->lc_should_back()), alloc);
  json_doc.AddMember("lcBackCnt",
                     rp::Value().SetInt(state_machine_->lc_back_cnt()), alloc);
  json_doc.AddMember(
      "lcBackReason",
      rp::Value().SetString(state_machine_->lc_back_reason().c_str(),
                            state_machine_->lc_back_reason().length()),
      alloc);
  json_doc.AddMember(
      "lcInvalidBackReason",
      rp::Value().SetString(state_machine_->invalid_back_reason().c_str(),
                            state_machine_->invalid_back_reason().length()),
      alloc);
  json_doc.AddMember(
      "lcInvalidReason",
      rp::Value().SetString(state_machine_->lc_invalid_reason().c_str(),
                            state_machine_->lc_invalid_reason().length()),
      alloc);
  json_doc.AddMember("lcRequest",
                     rp::Value().SetString(lateral_output.lc_request.c_str(),
                                           lateral_output.lc_request.length()),
                     alloc);
  json_doc.AddMember(
      "lcRequestSource",
      rp::Value().SetString(lateral_output.lc_request_source.c_str(),
                            lateral_output.lc_request_source.length()),
      alloc);
  json_doc.AddMember(
      "actRequestSource",
      rp::Value().SetString(lateral_output.act_request_source.c_str(),
                            lateral_output.act_request_source.length()),
      alloc);
  json_doc.AddMember("lcStatus",
                     rp::Value().SetString(lateral_output.lc_status.c_str(),
                                           lateral_output.lc_status.length()),
                     alloc);
  json_doc.AddMember(
      "lcBetterSource",
      rp::Value().SetString(lateral_output.lc_better_request_source.c_str(),
                            lateral_output.lc_better_request_source.length()),
      alloc);
  json_doc.AddMember("HFSMState",
                     rp::Value().SetString(state_machine_->lc_state().c_str(),
                                           state_machine_->lc_state().length()),
                     alloc);
  json_doc.AddMember(
      "HFSMStateChangeSource",
      rp::Value().SetString(state_machine_->state_change_reason().c_str(),
                            state_machine_->state_change_reason().length()),
      alloc);

  rp::Value target_gap_obs_json(rp::kArrayType);
  target_gap_obs_json.PushBack(
      rp::Value().SetInt(change_lane_status.target_gap_obs.first), alloc);
  target_gap_obs_json.PushBack(
      rp::Value().SetInt(change_lane_status.target_gap_obs.second), alloc);
  json_doc.AddMember("lcTargetGapObs", target_gap_obs_json, alloc);

  json_doc.AddMember("lcValid", rp::Value().SetBool(state_machine_->lc_valid()),
                     alloc);
  json_doc.AddMember("lcValidBack",
                     rp::Value().SetBool(state_machine_->lc_valid_back()),
                     alloc);
  json_doc.AddMember("lcValidCnt",
                     rp::Value().SetInt(state_machine_->lc_valid_cnt()), alloc);
  json_doc.AddMember("lEgo", rp::Value().SetDouble(ego_state.ego_frenet.y),
                     alloc);
  json_doc.AddMember(
      "scenario",
      rp::Value().SetString(lateral_output.scenario_name.c_str(),
                            lateral_output.scenario_name.length()),
      alloc);
  json_doc.AddMember(
      "status",
      rp::Value().SetString(state_machine_->state_name().c_str(),
                            state_machine_->state_name().length()),
      alloc);
  json_doc.AddMember("turnLight",
                     rp::Value().SetString(lateral_output.turn_light.c_str(),
                                           lateral_output.turn_light.length()),
                     alloc);
  json_doc.AddMember(
      "turnLightSource",
      rp::Value().SetString(lateral_output.turn_light_source.c_str(),
                            lateral_output.turn_light_source.length()),
      alloc);

  if (lateral_output.avd_car_past.size() > 0) {
    rp::Value json_array(rp::kArrayType);
    for (size_t i = 0; i < lateral_output.avd_car_past.size(); i++) {
      rp::Value car_past_json(rp::kArrayType);
      for (auto item : lateral_output.avd_car_past[i]) {
        car_past_json.PushBack(rp::Value().SetDouble(item), alloc);
      }

      if (car_past_json.Size() > 0) {
        json_array.PushBack(car_past_json, alloc);
      }
    }

    if (json_array.Size() > 0) {
      json_doc.AddMember("vec.avdCarPast", json_array, alloc);
    } else {
      json_doc.AddMember("vec.avdCarPast", rp::Value().SetString("none"),
                         alloc);
    }
  } else {
    json_doc.AddMember("vec.avdCarPast", rp::Value().SetString("none"), alloc);
  }

  if (lateral_output.avd_sp_car_past.size() > 0) {
    rp::Value json_array(rp::kArrayType);
    for (size_t i = 0; i < lateral_output.avd_sp_car_past.size(); i++) {
      rp::Value car_past_json(rp::kArrayType);
      for (auto item : lateral_output.avd_sp_car_past[i]) {
        car_past_json.PushBack(rp::Value().SetDouble(item), alloc);
      }

      if (car_past_json.Size() > 0) {
        json_array.PushBack(car_past_json, alloc);
      }
    }

    if (json_array.Size() > 0) {
      json_doc.AddMember("vec.avdSPCarPast", json_array, alloc);
    } else {
      json_doc.AddMember("vec.avdSPCarPast", rp::Value().SetString("none"),
                         alloc);
    }
  } else {
    json_doc.AddMember("vec.avdSPCarPast", rp::Value().SetString("none"),
                       alloc);
  }

  if (lateral_output.avd_info.size() > 0) {
    rp::Value json_array(rp::kArrayType);
    for (auto &item : lateral_output.avd_info) {
      rp::Value avd_msg(rp::kObjectType);
      avd_msg.AddMember("id", rp::Value().SetInt(item.id), alloc);
      avd_msg.AddMember(
          "property",
          rp::Value().SetString(item.property.c_str(), item.property.length()),
          alloc);
      avd_msg.AddMember("ignore", rp::Value().SetBool(item.ignore), alloc);
      avd_msg.AddMember("avd_direction",
                        rp::Value().SetString(item.avd_direction.c_str(),
                                              item.avd_direction.length()),
                        alloc);
      avd_msg.AddMember("avd_priority", rp::Value().SetInt(item.avd_priority),
                        alloc);
      avd_msg.AddMember("blocked_time_begin",
                        rp::Value().SetDouble(item.blocked_time_begin), alloc);
      avd_msg.AddMember("blocked_time_end",
                        rp::Value().SetDouble(item.blocked_time_end), alloc);
      json_array.PushBack(avd_msg, alloc);
    }

    json_doc.AddMember("vec.avoidInfo", json_array, alloc);
  } else {
    json_doc.AddMember("vec.avoidInfo", rp::Value().SetString("none"), alloc);
  }

  if (lateral_output.c_poly.size() > 0) {
    rp::Value json_array(rp::kArrayType);
    for (auto item : lateral_output.c_poly) {
      json_array.PushBack(rp::Value().SetDouble(item), alloc);
    }
    json_doc.AddMember("vec.cPoly", json_array, alloc);
  } else {
    json_doc.AddMember("vec.cPoly", rp::Value().SetString("none"), alloc);
  }

  if (lateral_output.d_poly.size() > 0) {
    rp::Value json_array(rp::kArrayType);
    for (auto item : lateral_output.d_poly) {
      json_array.PushBack(rp::Value().SetDouble(item), alloc);
    }
    json_doc.AddMember("vec.dPoly", json_array, alloc);
  } else {
    json_doc.AddMember("vec.dPoly", rp::Value().SetString("none"), alloc);
  }

  if (lateral_output.ignore_change_false.size() > 0) {
    rp::Value json_array(rp::kArrayType);
    for (auto item : lateral_output.ignore_change_false) {
      json_array.PushBack(rp::Value().SetDouble(item), alloc);
    }
    json_doc.AddMember("vec.ignoreChangeFalse", json_array, alloc);
  } else {
    json_doc.AddMember("vec.ignoreChangeFalse", rp::Value().SetString("none"),
                       alloc);
  }

  if (lateral_output.ignore_change_true.size() > 0) {
    rp::Value json_array(rp::kArrayType);
    for (auto item : lateral_output.ignore_change_true) {
      json_array.PushBack(rp::Value().SetDouble(item), alloc);
    }
    json_doc.AddMember("vec.ignoreChangeTrue", json_array, alloc);
  } else {
    json_doc.AddMember("vec.ignoreChangeTrue", rp::Value().SetString("none"),
                       alloc);
  }

  rp::Value json_lpoly(rp::kArrayType);
  for (auto item : lateral_output.l_poly) {
    json_lpoly.PushBack(rp::Value().SetDouble(item), alloc);
  }
  json_doc.AddMember("vec.leftPoly", json_lpoly, alloc);

  rp::Value json_rpoly(rp::kArrayType);
  for (auto item : lateral_output.r_poly) {
    json_rpoly.PushBack(rp::Value().SetDouble(item), alloc);
  }
  json_doc.AddMember("vec.rightPoly", json_rpoly, alloc);

  rp::StringBuffer buffer;
  rp::Writer<rp::StringBuffer> writer(buffer);
  (void)json_doc.Accept(writer);
  msg = std::string(buffer.GetString(), buffer.GetSize());
}

void ScenarioManager::create_lateral_decision_debug_msg(std::string &msg) {
  auto &lateral_output =
      PlanningContext::Instance()->lateral_behavior_planner_output();

  auto virtual_lane_mgr = state_machine_->virtual_lane_mgr();
  auto &olane = virtual_lane_mgr->get_origin_lane();
  auto &tlane = virtual_lane_mgr->get_target_lane();
  auto &flane = virtual_lane_mgr->get_fix_lane();
  auto &lateral_obstacle = world_model_->mutable_lateral_obstacle();

  namespace rp = rapidjson;
  rp::Document json_doc;
  json_doc.SetObject();
  auto &alloc = json_doc.GetAllocator();

  json_doc.AddMember("mmp.intersectLength",
                     rp::Value().SetDouble(lateral_output.intersect_length),
                     alloc);
  json_doc.AddMember("mmp.hasOlane",
                     rp::Value().SetBool(virtual_lane_mgr->has_origin_lane()),
                     alloc);
  json_doc.AddMember("mmp.hasTlane",
                     rp::Value().SetBool(virtual_lane_mgr->has_target_lane()),
                     alloc);

  int olane_position = -99;
  int tlane_position = -99;
  if (virtual_lane_mgr->has_origin_lane()) {
    olane_position = virtual_lane_mgr->mutable_origin_lane().position();
  }
  if (virtual_lane_mgr->has_target_lane()) {
    tlane_position = virtual_lane_mgr->mutable_target_lane().position();
  }
  // int olane_left_id = olane.rids()[0];
  // if (olane_left_id == kInvalidRelativeId) {
  //   olane_left_id = -99;
  // }

  // int olane_right_id = olane.rids()[1];
  // if (olane_right_id == kInvalidRelativeId) {
  //   olane_right_id = -99;
  // }

  // int tlane_left_id = tlane.rids()[0];
  // if (tlane_left_id == kInvalidRelativeId) {
  //   tlane_left_id = -99;
  // }

  // int tlane_right_id = tlane.rids()[1];
  // if (tlane_right_id == kInvalidRelativeId) {
  //   tlane_right_id = -99;
  // }

  // int flane_left_id = flane.rids()[0];
  // if (flane_left_id == kInvalidRelativeId) {
  //   flane_left_id = -99;
  // }

  // int flane_right_id = flane.rids()[1];
  // if (flane_right_id == kInvalidRelativeId) {
  //   flane_right_id = -99;
  // }

  json_doc.AddMember("mmp.olanePosition", rp::Value().SetInt(olane_position),
                     alloc);
  json_doc.AddMember("mmp.tlanePosition", rp::Value().SetInt(tlane_position),
                     alloc);
  // json_doc.AddMember("mmp.tlaneLeftId", rp::Value().SetInt(tlane_left_id),
  // alloc); json_doc.AddMember("mmp.tlaneRightId",
  // rp::Value().SetInt(tlane_right_id), alloc);
  // json_doc.AddMember("mmp.flaneLeftId", rp::Value().SetInt(flane_left_id),
  // alloc); json_doc.AddMember("mmp.flaneRightId",
  // rp::Value().SetInt(flane_right_id), alloc);
  json_doc.AddMember("mmp.tLeftLane",
                     rp::Value().SetBool(lateral_output.tleft_lane), alloc);
  json_doc.AddMember("mmp.rightestLane",
                     rp::Value().SetBool(lateral_output.rightest_lane), alloc);

  json_doc.AddMember("sensor.fvfDead",
                     rp::Value().SetBool(lateral_obstacle.fvf_dead()), alloc);
  json_doc.AddMember("sensor.svfDead",
                     rp::Value().SetBool(lateral_obstacle.svf_dead()), alloc);

  auto &lc_back_track = state_machine_->lc_back_track();
  if (lc_back_track.track_id != -10000) {
    rp::Value json(rp::kObjectType);
    update_json_object_for_track_info(lc_back_track, json, alloc);
    json_doc.AddMember("lc.BackTrack", json, alloc);
  } else {
    json_doc.AddMember("lc.BackTrack", rp::Value().SetString("none"), alloc);
  }

  auto &lc_invalid_track = state_machine_->lc_invalid_track();
  if (lc_invalid_track.track_id != -10000) {
    rp::Value json(rp::kObjectType);
    update_json_object_for_track_info(lc_invalid_track, json, alloc);
    json_doc.AddMember("lc.InvalidTrack", json, alloc);
  } else {
    json_doc.AddMember("lc.InvalidTrack", rp::Value().SetString("none"), alloc);
  }

  auto &near_cars_target = state_machine_->near_cars_target();
  if (near_cars_target.size() > 0) {
    rp::Value json_array(rp::kArrayType);
    for (auto &car : near_cars_target) {
      rp::Value json(rp::kObjectType);
      update_json_object_for_track_info(car, json, alloc);
      json_array.PushBack(json, alloc);
    }
    json_doc.AddMember("lc.nearCarsTarget", json_array, alloc);
  } else {
    json_doc.AddMember("lc.nearCarsTarget", rp::Value().SetString("none"),
                       alloc);
  }

  auto &near_cars_origin = state_machine_->near_cars_origin();
  if (near_cars_origin.size() > 0) {
    rp::Value json_array(rp::kArrayType);
    for (auto &car : near_cars_origin) {
      rp::Value json(rp::kObjectType);
      update_json_object_for_track_info(car, json, alloc);
      json_array.PushBack(json, alloc);
    }
    json_doc.AddMember("lc.nearCarsOrigin", json_array, alloc);
  } else {
    json_doc.AddMember("lc.nearCarsOrigin", rp::Value().SetString("none"),
                       alloc);
  }

  rp::StringBuffer buffer;
  rp::Writer<rp::StringBuffer> writer(buffer);
  (void)json_doc.Accept(writer);
  msg = std::string(buffer.GetString(), buffer.GetSize());
}

void ScenarioManager::create_prediction_object_info_msg(std::string &msg) {
  namespace rp = rapidjson;
  rp::Document json_doc;
  json_doc.SetObject();
  auto &alloc = json_doc.GetAllocator();
  auto &lateral_obstacle = world_model_->mutable_lateral_obstacle();

  auto &front_tracks = lateral_obstacle.front_tracks();
  if (front_tracks.size() > 0) {
    rp::Value front_tracks_json(rp::kArrayType);
    for (auto &source : front_tracks) {
      rp::Value json(rp::kObjectType);
      update_json_object_for_fusion(source, json, alloc);
      front_tracks_json.PushBack(json, alloc);
    }
    json_doc.AddMember("viewFront", front_tracks_json, alloc);
  } else {
    json_doc.AddMember("viewFront", rp::Value().SetString("none"), alloc);
  }

  auto &side_tracks = lateral_obstacle.side_tracks();
  if (side_tracks.size() > 0) {
    rp::Value side_tracks_json(rp::kArrayType);
    for (auto &source : side_tracks) {
      rp::Value json(rp::kObjectType);
      update_json_object_for_fusion(source, json, alloc);
      side_tracks_json.PushBack(json, alloc);
    }
    json_doc.AddMember("viewSide", side_tracks_json, alloc);
  } else {
    json_doc.AddMember("viewSide", rp::Value().SetString("none"), alloc);
  }

  auto lead_one = lateral_obstacle.leadone();
  if (lead_one != nullptr) {
    rp::Value json(rp::kObjectType);
    update_json_object_for_fusion(*lead_one, json, alloc);
    json_doc.AddMember("leadOne", json, alloc);
  } else {
    json_doc.AddMember("leadOne", rp::Value().SetString("none"), alloc);
  }

  auto lead_two = lateral_obstacle.leadtwo();
  if (lead_two != nullptr) {
    rp::Value json(rp::kObjectType);
    update_json_object_for_fusion(*lead_two, json, alloc);
    json_doc.AddMember("leadTwo", json, alloc);
  } else {
    json_doc.AddMember("leadTwo", rp::Value().SetString("none"), alloc);
  }

  auto temp_leadone = lateral_obstacle.tleadone();
  if (temp_leadone != nullptr) {
    rp::Value json(rp::kObjectType);
    update_json_object_for_fusion(*temp_leadone, json, alloc);
    json_doc.AddMember("tempLeadOne", json, alloc);
  } else {
    json_doc.AddMember("tempLeadOne", rp::Value().SetString("none"), alloc);
  }

  auto temp_leadtwo = lateral_obstacle.tleadtwo();
  if (temp_leadtwo != nullptr) {
    rp::Value json(rp::kObjectType);
    update_json_object_for_fusion(*temp_leadtwo, json, alloc);
    json_doc.AddMember("tempLeadTwo", json, alloc);
  } else {
    json_doc.AddMember("tempLeadTwo", rp::Value().SetString("none"), alloc);
  }

  rp::StringBuffer buffer;
  rp::Writer<rp::StringBuffer> writer(buffer);
  (void)json_doc.Accept(writer);
  msg = std::string(buffer.GetString(), buffer.GetSize());
}

void ScenarioManager::create_map_planning_info_msg(std::string &msg) {
  auto tic = MTIME()->timestamp().sec();
  namespace rp = rapidjson;
  rp::Document json_doc;
  json_doc.SetObject();
  auto &alloc = json_doc.GetAllocator();
  auto &map_info = world_model_->get_map_info();
  auto current_lane_dis_to_merge =
      map_info.distance_to_lanes_merge(map_info.current_lane_index());
  auto current_lane_merge_type =
      map_info.lanes_merge_type(map_info.current_lane_index());
  auto right_lane_dis_to_merge =
      map_info.current_lane_index() < map_info.lanes_num() - 1
          ? map_info.distance_to_lanes_merge(map_info.current_lane_index() + 1)
          : DBL_MAX;
  auto right_lane_merge_type =
      map_info.current_lane_index() < map_info.lanes_num() - 1
          ? map_info.lanes_merge_type(map_info.current_lane_index() + 1)
          : MergeType::UNKNOWN;

  json_doc.AddMember("hasTask", rp::Value().SetBool(map_info.has_task()),
                     alloc);
  json_doc.AddMember("currLaneIndex",
                     rp::Value().SetInt(map_info.current_lane_index()), alloc);
  json_doc.AddMember("currTaskIndex",
                     rp::Value().SetInt(map_info.current_tasks_id()), alloc);
  json_doc.AddMember("leftTaskIndex",
                     rp::Value().SetInt(map_info.left_lane_tasks_id()), alloc);
  json_doc.AddMember("rightTaskIndex",
                     rp::Value().SetInt(map_info.right_lane_tasks_id()), alloc);
  json_doc.AddMember("currIntsectTask",
                     rp::Value().SetInt(map_info.curr_intsect_task()), alloc);
  json_doc.AddMember("lastIntsectTask",
                     rp::Value().SetInt(map_info.last_intsect_task()), alloc);
  json_doc.AddMember("lcMapDecision",
                     rp::Value().SetInt(map_info.lc_map_decision()), alloc);
  json_doc.AddMember("lcStartDistance",
                     rp::Value().SetDouble(map_info.lc_start_dis()), alloc);
  json_doc.AddMember("lcEndDistance",
                     rp::Value().SetDouble(map_info.lc_end_dis()), alloc);
  json_doc.AddMember("globalEndDistance",
                     rp::Value().SetDouble(map_info.global_end_dis()), alloc);
  json_doc.AddMember("distIntersect",
                     rp::Value().SetDouble(map_info.dist_to_intsect()), alloc);
  json_doc.AddMember("distLastIntersect",
                     rp::Value().SetDouble(map_info.dist_to_last_intsect()),
                     alloc);
  json_doc.AddMember("distStopLine",
                     rp::Value().SetDouble(map_info.distance_to_stop_line()),
                     alloc);
  json_doc.AddMember("intsectLength",
                     rp::Value().SetDouble(map_info.intsect_length()), alloc);
  json_doc.AddMember("trafficLightDirection",
                     rp::Value().SetInt(map_info.traffic_light_direction()),
                     alloc);
  json_doc.AddMember("inMapArea",
                     rp::Value().SetBool(map_info.is_in_map_area()), alloc);
  json_doc.AddMember("inIntersection",
                     rp::Value().SetBool(map_info.is_in_intersection()), alloc);
  json_doc.AddMember("lanesNum", rp::Value().SetInt(map_info.lanes_num()),
                     alloc);
  json_doc.AddMember("currentLaneDisToMerge",
                     rp::Value().SetDouble(current_lane_dis_to_merge), alloc);
  json_doc.AddMember(
      "currentLaneMergeType",
      rp::Value().SetInt(static_cast<int>(current_lane_dis_to_merge)), alloc);
  json_doc.AddMember("rightLaneDisToMerge",
                     rp::Value().SetDouble(right_lane_dis_to_merge), alloc);
  json_doc.AddMember(
      "rightLaneMergeType",
      rp::Value().SetInt(static_cast<int>(right_lane_merge_type)), alloc);

  if (map_info.lanes_num() > 0) {
    rp::Value lanes_data_json(rp::kArrayType);
    for (auto &lane_data : map_info.lane_) {
      if ((lane_data.relative_id < -1) || (lane_data.relative_id > 1)) {
        // only update clr lane log
        continue;
      }
      rp::Value lane_json(rp::kObjectType);
      update_json_object_for_lane_data(map_info, lane_data, lane_json, alloc,
                                       world_model_);
      lanes_data_json.PushBack(lane_json, alloc);
    }

    json_doc.AddMember("vecLanesData", lanes_data_json, alloc);
  } else {
    json_doc.AddMember("vecLanesData", rp::Value().SetNull(), alloc);
  }

  rp::StringBuffer buffer;
  rp::Writer<rp::StringBuffer> writer(buffer);
  (void)json_doc.Accept(writer);
  msg = std::string(buffer.GetString(), buffer.GetSize());
  auto toc = MTIME()->timestamp().sec();
  auto duration = toc - tic;
  MSD_LOG(INFO, "jsondbg: %u", duration);
}

void ScenarioManager::create_acc_refline_msg(std::string &msg) {
  auto tic = MTIME()->timestamp().sec();
  namespace rp = rapidjson;
  rp::Document json_doc;
  json_doc.SetObject();
  auto &alloc = json_doc.GetAllocator();

  const auto &acc_refline =
      world_model_->get_baseline_info(0)->get_acc_refline();

  const auto &acc_leftborder =
      world_model_->get_baseline_info(0)->get_acc_leftborder();

  const auto &acc_rightborder =
      world_model_->get_baseline_info(0)->get_acc_rightborder();

  if (acc_refline.size()) {
    rp::Value json_array_x(rp::kArrayType);
    rp::Value json_array_y(rp::kArrayType);
    for (auto &car_point : acc_refline) {
      json_array_x.PushBack(rp::Value().SetDouble(car_point.x()), alloc);
      json_array_y.PushBack(rp::Value().SetDouble(car_point.y()), alloc);
    }
    json_doc.AddMember("acc_refline_x", json_array_x, alloc);
    json_doc.AddMember("acc_refline_y", json_array_y, alloc);
  } else {
    json_doc.AddMember("acc_refline_x", rp::Value().SetString("none"), alloc);
    json_doc.AddMember("acc_refline_y", rp::Value().SetString("none"), alloc);
  }

  if (acc_leftborder.size()) {
    rp::Value json_array_x(rp::kArrayType);
    rp::Value json_array_y(rp::kArrayType);
    for (auto &car_point : acc_leftborder) {
      json_array_x.PushBack(rp::Value().SetDouble(car_point.x()), alloc);
      json_array_y.PushBack(rp::Value().SetDouble(car_point.y()), alloc);
    }
    json_doc.AddMember("acc_refline_left_border_x", json_array_x, alloc);
    json_doc.AddMember("acc_refline_left_border_y", json_array_y, alloc);
  } else {
    json_doc.AddMember("acc_refline_left_border_x",
                       rp::Value().SetString("none"), alloc);
    json_doc.AddMember("acc_refline_left_border_y",
                       rp::Value().SetString("none"), alloc);
  }

  if (acc_rightborder.size()) {
    rp::Value json_array_x(rp::kArrayType);
    rp::Value json_array_y(rp::kArrayType);
    for (auto &car_point : acc_rightborder) {
      json_array_x.PushBack(rp::Value().SetDouble(car_point.x()), alloc);
      json_array_y.PushBack(rp::Value().SetDouble(car_point.y()), alloc);
    }
    json_doc.AddMember("acc_refline_right_border_x", json_array_x, alloc);
    json_doc.AddMember("acc_refline_right_border_y", json_array_y, alloc);
  } else {
    json_doc.AddMember("acc_refline_right_border_x",
                       rp::Value().SetString("none"), alloc);
    json_doc.AddMember("acc_refline_right_border_y",
                       rp::Value().SetString("none"), alloc);
  }

  rp::StringBuffer buffer;
  rp::Writer<rp::StringBuffer> writer(buffer);
  (void)json_doc.Accept(writer);
  msg = std::string(buffer.GetString(), buffer.GetSize());
  auto toc = MTIME()->timestamp().sec();
  auto duration = toc - tic;
  MSD_LOG(INFO, "jsondbg: %u", duration);
}

} // namespace msquare
