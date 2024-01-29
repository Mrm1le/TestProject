#include "planner/behavior_planner/deciders/st_graph_generator.h"
#include "common/config_context.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/math/transform.h"
#include "common/obstacle_process_utils.h"
#include "common/world_model.h"
#include "data_driven_planner/common/ddp_context.h"
#include "math.h"
#include "planner/behavior_planner/deciders/st_boundary_mapper.h"
#include "planner/motion_planner/common/speed_profile_generator.h"
#include "planner/motion_planner/common/window_smoother.h"
#include "planning/common/common.h"
#include "planning/common/logging.h"
#include <cassert>
#include <mlog_core/mlog.h>
#include <string>

#define st_graph_construct_mode msquare::STGraphConstructMode::SKETCHY_MODE

namespace msquare {

constexpr double StGraphGenerator::kPedestrianStopTimeout;

StGraphGenerator::StGraphGenerator(
    const std::shared_ptr<WorldModel> &world_model,
    const std::shared_ptr<BaseLineInfo> &baseline_info,
    SpeedBoundaryDeciderConfig config, ScenarioFacadeContext *context) {
  update(world_model, baseline_info, config, context);
}

void StGraphGenerator::clear() {
  world_model_ = nullptr;
  baseline_info_ = nullptr;
  context_ = nullptr;

  left_border_.clear();
  right_border_.clear();
  reference_line_segments_.clear();

  is_in_lane_crossing_stage_ = false;
  is_lane_interval_transformed_ = false;

  left_width_ = LANE_WIDTH() * 0.5;
  right_width_ = LANE_WIDTH() * 0.5;
}

void StGraphGenerator::update(
    const std::shared_ptr<WorldModel> &world_model,
    const std::shared_ptr<BaseLineInfo> &baseline_info,
    SpeedBoundaryDeciderConfig config, ScenarioFacadeContext *context) {
  clear();

  world_model_ = world_model;
  baseline_info_ = baseline_info;
  config_ = config;
  context_ = context;
  s_range_.first = 0.0;
  s_range_.second = FLAGS_speed_lon_decision_horizon;
  time_range_.first = 0.0;
  time_range_.second = FLAGS_speed_lon_decision_time_horizon;
}

bool StGraphGenerator::ComputeSTGraph() {
  const auto &planning_status = context_->planning_status();

  const auto &frenet_coord = baseline_info_->get_frenet_coord();
  const double ego_vel = baseline_info_->get_ego_state().ego_vel;
  const auto &ego_sl = baseline_info_->get_adc_sl_boundary();
  auto total_start = MTIME()->timestamp();

  // customize ego lane width according to current scenario
  set_cross_lane_state();

  static PathData reference_line;
  generate_shifted_reference_line(reference_line, frenet_coord, left_width_,
                                  -right_width_);
  // generate_road_boundary_obstacles();
  preliminary_screen_obstacles();

  double start = MTIME()->timestamp().sec();

  // construct st boundary according to shifted lane width
  MSD_LOG(INFO, "DEBUG_CJ6 scheme_stage:%d", planning_status.scheme_stage);
  if (planning_status.scheme_stage == SchemeStage::PRIMARY) {
    auto plan_start = MTIME()->timestamp();
    if (st_graph_construct_mode == STGraphConstructMode::ACCURATE_MODE) {
      STBoundaryMapper boundary_mapper(
          baseline_info_->get_frenet_coord(), reference_line,
          baseline_info_->get_planning_start_point(),
          std::fmin(dp_st_config::FLAGS_speed_lon_decision_horizon,
                    reference_line.discretized_path().Length()),
          dp_st_config::FLAGS_speed_lon_decision_time_horizon);
      if (!boundary_mapper.ComputeSTBoundary(
              &baseline_info_->mutable_obstacle_manager(),
              &context_->mutable_obstacle_decision_manager())) {
        MSD_LOG(INFO, "debug: mapping obstacle failed.");
        return false;
      }
    } else if (st_graph_construct_mode == STGraphConstructMode::SKETCHY_MODE) {
      compute_sl_polygon_sequence(baseline_info_->mutable_obstacle_manager(),
                                  context_, baseline_info_, time_range_,
                                  FLAGS_speed_lon_decision_time_resolution);
    }
    compute_st_boundaries();

    auto process_end = MTIME()->timestamp();
    auto process_span = (process_end - plan_start).sec();
    // MSD_LOG(INFO, "debug: st graph mapper cost time: %f s", process_span);
    double end = MTIME()->timestamp().sec();
    double time_cost = end - start;
    if (time_cost > 10.e-4) {
      MSD_LOG(INFO,
              "--------******* st graph mapper cost time tasks cost time "
              "%.9f------",
              time_cost);
    }
  }

  translate_scenario_decisions();
  extract_info();

  auto total_end = MTIME()->timestamp();
  auto total_span = (total_end - total_start).sec();
  MSD_LOG(INFO, "debug: st graph generator cost time: %f s", total_span);
  return true;
}

void StGraphGenerator::fill_lane_debug_info() {
  // longitudinal debug info
  // fill ego virtual lane info
  auto *planner_debug = context_->mutable_planner_debug();
  Eigen::Vector3d car_point, enu_point;
  auto &enu2car = world_model_->get_cart_ego_state_manager().get_enu2car();
  const auto &frenet_coord = baseline_info_->get_frenet_coord();

  Point2D frenet_p, carte_p;
  if (!world_model_->use_eftp()) {
    planner_debug->lon_debug_info.left_virtual_line.clear();
    planner_debug->lon_debug_info.right_virtual_line.clear();
    PointXY left_virtual_point{0.0, 0.0}, right_virtual_point{0.0, 0.0};
    for (int i = 0; i < left_border_.size(); ++i) {
      frenet_p.x = left_border_[i].s;
      frenet_p.y = left_border_[i].l;
      MSD_LOG(INFO, "DEBUG_CJ_JSON:lb[%d]:[%f,%f]", i, frenet_p.x, frenet_p.y);
      if (frenet_coord->FrenetCoord2CartCoord(frenet_p, carte_p) ==
          TRANSFORM_SUCCESS) {
        enu_point.x() = carte_p.x;
        enu_point.y() = carte_p.y;
        enu_point.z() = 0;
        car_point = enu2car * enu_point;
        left_virtual_point.x = car_point.x();
        left_virtual_point.y = car_point.y();
      } else {
        // use last i value (i > 0) or default value (i == 0)
      }
      planner_debug->lon_debug_info.left_virtual_line.push_back(
          left_virtual_point);
      frenet_p.x = right_border_[i].s;
      frenet_p.y = right_border_[i].l;
      MSD_LOG(INFO, "DEBUG_CJ_JSON:rb[%d]:[%f,%f]", i, frenet_p.x, frenet_p.y);
      if (frenet_coord->FrenetCoord2CartCoord(frenet_p, carte_p) ==
          TRANSFORM_SUCCESS) {
        enu_point.x() = carte_p.x;
        enu_point.y() = carte_p.y;
        enu_point.z() = 0;
        car_point = enu2car * enu_point;
        right_virtual_point.x = car_point.x();
        right_virtual_point.y = car_point.y();
      } else {
        // use last i value (i > 0) or default value (i == 0)
      }
      planner_debug->lon_debug_info.right_virtual_line.push_back(
          right_virtual_point);
    }
  }

  // fill vision line info
  planner_debug->lon_debug_info.left_line.clear();
  planner_debug->lon_debug_info.right_line.clear();
  auto ref_line = world_model_->get_map_info().current_refline_points();
  Point2D left_frenet_p{0.0, 0.0}, right_frenet_p{0.0, 0.0},
      left_carte_p{0.0, 0.0}, right_carte_p{0.0, 0.0};
  PointXY left_line_point{0.0, 0.0}, right_line_point{0.0, 0.0};
  MSD_LOG(INFO, "DEBUG_CJ_JSON:ref_line_size:%d", ref_line.size());
  if (ref_line.size() > 0) {
    for (int i = 0; i < ref_line.size(); ++i) {
      carte_p.x = ref_line[i].enu_point.x;
      carte_p.y = ref_line[i].enu_point.y;
      MSD_LOG(INFO, "DEBUG_CJ_JSON:ref_line[%d],cx:%f,cy:%f", i, carte_p.x,
              carte_p.y);
      if (frenet_coord->CartCoord2FrenetCoord(carte_p, frenet_p) ==
          TRANSFORM_SUCCESS) {
        left_frenet_p.x = frenet_p.x;
        right_frenet_p.x = frenet_p.x;
        left_frenet_p.y = frenet_p.y + ref_line[i].distance_to_left_lane_border;
        right_frenet_p.y =
            frenet_p.y - ref_line[i].distance_to_right_lane_border;
        MSD_LOG(INFO,
                "DEBUG_CJ_JSON:ref_line[%d],fx:%.2f,fy:%.2f,ly:%.2f,ry:%.2f", i,
                frenet_p.x, frenet_p.y, left_frenet_p.y, right_frenet_p.y);
      } else {
        // use last i value (i > 0) or default value (i == 0)
        planner_debug->lon_debug_info.left_line.push_back(left_line_point);
        planner_debug->lon_debug_info.right_line.push_back(right_line_point);
        MSD_LOG(INFO, "DEBUG_CJ_JSON:ref_line[%d],failed", i);
      }
      if (frenet_coord->FrenetCoord2CartCoord(left_frenet_p, left_carte_p) ==
          TRANSFORM_SUCCESS) {
        enu_point.x() = left_carte_p.x;
        enu_point.y() = left_carte_p.y;
        enu_point.z() = 0;
        car_point = enu2car * enu_point;
        left_line_point.x = car_point.x();
        left_line_point.y = car_point.y();
        MSD_LOG(INFO, "DEBUG_CJ_JSON:left_line[%d],success,llx:%f,lly:%f", i,
                car_point.x(), car_point.y());
      } else {
        // use last i value (i > 0) or default value (i == 0)
      }
      planner_debug->lon_debug_info.left_line.push_back(left_line_point);
      if (frenet_coord->FrenetCoord2CartCoord(right_frenet_p, right_carte_p) ==
          TRANSFORM_SUCCESS) {
        enu_point.x() = right_carte_p.x;
        enu_point.y() = right_carte_p.y;
        enu_point.z() = 0;
        car_point = enu2car * enu_point;
        right_line_point.x = car_point.x();
        right_line_point.y = car_point.y();
      } else {
        // use last i value (i > 0) or default value (i == 0)
      }
      planner_debug->lon_debug_info.right_line.push_back(right_line_point);
    }
  }
  planner_debug->lon_debug_info.object_list.clear();
  MSD_LOG(INFO, "DEBUG_CJ_JSON:ll_size:%d,rl_size:%d,lvl_size:%d,rvl_size:%d",
          planner_debug->lon_debug_info.left_line.size(),
          planner_debug->lon_debug_info.right_line.size(),
          planner_debug->lon_debug_info.left_virtual_line.size(),
          planner_debug->lon_debug_info.right_virtual_line.size());
  if (planner_debug->lon_debug_info.left_line.size() > 0) {
    MSD_LOG(INFO, "DEBUG_CJ_JSON:ll[0]:[%f,%f]",
            planner_debug->lon_debug_info.left_line[0].x,
            planner_debug->lon_debug_info.left_line[0].y);
  }
}

void StGraphGenerator::extract_info() {
  MLOG_PROFILING("extract_info");
  auto &obstacle_decision_manager =
      context_->mutable_obstacle_decision_manager();
  const auto normal_obstacles_items =
      baseline_info_->obstacle_manager().get_obstacles().Items();
  const auto virtual_obstacles_items =
      context_->get_virtual_obstacles().Items();
  auto obstacles_items = normal_obstacles_items;
  obstacles_items.insert(obstacles_items.end(), virtual_obstacles_items.begin(),
                         virtual_obstacles_items.end());

  fill_lane_debug_info();

  auto &obj_mem_info_map = context_->mutable_planner_debug()->obj_mem_info_map;
  for (auto iter = obj_mem_info_map.begin(); iter != obj_mem_info_map.end();
       ++iter) {
    iter->second.exist = false;
  }

  auto &lon_obstacle_decision_info_map =
      context_->mutable_lon_decison_output()->obstacle_decision_info_map;
  MSD_LOG(INFO, "DEBUG_CJ_Deci:obj_size:%d",
          lon_obstacle_decision_info_map.size());
  for (auto &lon_decision : lon_obstacle_decision_info_map) {
    lon_decision.second.exist = false;
    MSD_LOG(INFO, "DEBUG_CJ_Deci:ID:%d", lon_decision.first);
  }

  for (const auto &obstacle : obstacles_items) {
    // ignore obstacle with lateral nudge decision
    MSD_LOG(INFO, "IGNORE:P0:ID:%d", obstacle->Id());
    auto ptr_obstacle_decision =
        obstacle_decision_manager.find_obstacle_decision(obstacle->Id());
    MSD_LOG(INFO, "DEBUG_CJ831:extract_info:ID:%d,Overlap:%d", obstacle->Id(),
            obstacle->IsEgoLaneOverlap());
    set_obstacle_new(obstacle);

    if (ptr_obstacle_decision->HasLateralDecision()) {
      auto lat_decision = ptr_obstacle_decision->LateralDecision();
      if (lat_decision.has_nudge() &&
          lat_decision.nudge().is_longitunidal_ignored) {
        MSD_LOG(INFO, "IGNORE:P4:ID:%d", obstacle->Id());
        continue;
      }
    }
    // ignore obstacle with longitudinal ignore decision
    if (ptr_obstacle_decision->HasLongitudinalDecision()) {
      auto lon_decision = ptr_obstacle_decision->LongitudinalDecision();
      if (lon_decision.has_ignore() || lon_decision.has_overtake() ||
          lon_decision.has_follow()) {
        MSD_LOG(INFO, "IGNORE:P5:ID:%d", obstacle->Id());
        continue;
      }
    }
  }
  for (auto iter = obj_mem_info_map.begin(); iter != obj_mem_info_map.end();) {
    if (iter->second.exist == false) {
      obj_mem_info_map.erase(iter++);
    } else {
      MSD_LOG(INFO, "DEBUG_CJ_Vy:ID:%d", iter->first);
      ++iter;
    }
  }
  MSD_LOG(INFO, "DEBUG_CJ_Vy3:obj_size:%d", obj_mem_info_map.size());

  update_decision_info();
}

std::pair<double, double> StGraphGenerator::sl_polygon_path_check(
    double left_border, double right_border,
    const planning_math::Polygon2d &polygon) {
  planning_math::Vec2d left_begin_point(s_range_.first, left_border);
  planning_math::Vec2d left_end_point(s_range_.second, left_border);
  planning_math::Vec2d right_begin_point(s_range_.first, right_border);
  planning_math::Vec2d right_end_point(s_range_.second, right_border);

  if (std::abs(left_border - right_border) < planning_math::kMathEpsilon) {
    if (left_border > right_border) {
      left_border += planning_math::kMathEpsilon;
      right_border -= planning_math::kMathEpsilon;
    } else {
      left_border -= planning_math::kMathEpsilon;
      right_border += planning_math::kMathEpsilon;
    }
  }

  double s_lower = std::numeric_limits<double>::max();
  double s_upper = 0.0;

  static std::vector<planning_math::Vec2d> reference_line_vertexes;
  reference_line_vertexes.clear();
  reference_line_vertexes.push_back(left_begin_point);
  reference_line_vertexes.push_back(left_end_point);
  reference_line_vertexes.push_back(right_end_point);
  reference_line_vertexes.push_back(right_begin_point);
  static planning_math::Polygon2d reference_line;
  reference_line.update(reference_line_vertexes);
  static planning_math::Polygon2d overlap_polygon;
  overlap_polygon.clear();
  if (polygon.ComputeOverlap(reference_line, &overlap_polygon)) {
    for (auto point : overlap_polygon.GetAllVertices()) {
      s_lower = min(s_lower, point.x());
      s_upper = max(s_upper, point.x());
    }
    return {s_lower, s_upper};
  } else {
    return {s_lower, s_upper};
  }
}

std::pair<double, double> StGraphGenerator::sl_polygon_path_check(
    const std::vector<planning_math::Polygon2d> &reference_line_segments,
    const planning_math::Polygon2d &polygon) {
  double s_lower = std::numeric_limits<double>::max();
  double s_upper = 0.0;

  for (size_t i = 0; i < reference_line_segments.size(); ++i) {
    if (polygon.min_x() > reference_line_segments[i].max_x() ||
        polygon.max_x() < reference_line_segments[i].min_x()) {
      continue;
    }
    if (polygon.min_y() > reference_line_segments[i].max_y() ||
        polygon.max_y() < reference_line_segments[i].min_y()) {
      continue;
    }
    static planning_math::Polygon2d overlap_polygon;
    overlap_polygon.clear();
    if (polygon.ComputeOverlap(reference_line_segments[i], &overlap_polygon)) {
      for (auto point : overlap_polygon.GetAllVertices()) {
        s_lower = min(s_lower, point.x());
        s_upper = max(s_upper, point.x());
      }
    }
  }

  return {s_lower, s_upper};
}

void StGraphGenerator::set_lane_changing_line_border(SLPoint &left_mid1,
                                                     SLPoint &right_mid1,
                                                     SLPoint &left_mid2,
                                                     SLPoint &right_mid2) {
  const double ego_s = baseline_info_->get_ego_state().ego_frenet.x;
  const auto &ego_sl = baseline_info_->get_adc_sl_boundary();
  auto lane_status = context_->mutable_planning_status()->lane_status;
  const double ego_vel = baseline_info_->get_ego_state().ego_vel;
  const double ego_theta_refline =
      baseline_info_->get_ego_state().mpc_vehicle_state[2];
  constexpr double kLcDuration = 3.0;
  constexpr double kLcDistance = 20.0;
  left_mid1.s =
      ego_s + ConfigurationContext::Instance()->get_vehicle_param().length;
  left_mid1.l = std::min(left_width_, ego_sl.end_l + 0.5 * LANE_WIDTH());
  right_mid1.s = left_mid1.s;
  right_mid1.l = std::max(-right_width_, ego_sl.start_l - 0.5 * LANE_WIDTH());
  if (lane_status.change_lane.direction == "right") {
    double dw_left = left_width_ - 0.5 * LANE_WIDTH();
    left_mid2.s =
        left_mid1.s +
        std::min(std::max(ego_vel * kLcDuration, kLcDistance),
                 dw_left / (std::fabs(std::tan(ego_theta_refline)) + 0.01));
    left_mid2.l = 0.5 * LANE_WIDTH();
    right_mid2.s = left_mid2.s;
    right_mid2.l = -right_width_;
  } else if (lane_status.change_lane.direction == "left") {
    double dw_right = right_width_ - 0.5 * LANE_WIDTH();
    right_mid2.s =
        right_mid1.s +
        std::min(std::max(ego_vel * kLcDuration, kLcDistance),
                 dw_right / (std::fabs(std::tan(ego_theta_refline)) + 0.01));
    right_mid2.l = -0.5 * LANE_WIDTH();
    left_mid2.s = right_mid2.s;
    left_mid2.l = left_width_;
  }
  left_border_.push_back(left_mid1);
  left_border_.push_back(left_mid2);
  right_border_.push_back(right_mid1);
  right_border_.push_back(right_mid2);
}

bool StGraphGenerator::set_reference_line_border() {
  left_border_.clear();
  right_border_.clear();
  reference_line_segments_.clear();
  const double ego_s = baseline_info_->get_ego_state().ego_frenet.x;
  const double ego_theta_refline =
      baseline_info_->get_ego_state().mpc_vehicle_state[2];
  const double ego_r = baseline_info_->get_ego_state().ego_frenet.y;
  const double ego_vel = baseline_info_->get_ego_state().ego_vel;
  const auto &ego_pose = baseline_info_->get_ego_state().ego_pose;
  const auto &ego_sl = baseline_info_->get_adc_sl_boundary();

  constexpr double kLcDuration = 3.0;
  constexpr double kLcDistance = 20.0;
  constexpr double kLbDuration1 = 2.0;
  constexpr double kLbDuration2 = 4.0;
  // CJ:kLbMinDis1 should not be too small to ensure smooth pass
  constexpr double kLbMinDis1 = 40.0;
  // CJ:kLbMaxDis1 should not be too large to prevent false brake
  constexpr double kLbMaxDis1 = 60.0;
  // CJ:kLbMinDis2 should not be too small to ensure safety
  constexpr double kLbMinDis2 = 60.0;
  // CJ:kLbMaxDis2 should not be too large to prevent false brake
  constexpr double kLbMaxDis2 = 80.0;

  double kdl_buffer = 0.2;
  if (world_model_->get_enable_cutin_threshold()) {
    if (ego_vel < 10 / 3.6) {
      kdl_buffer = world_model_->get_lon_cutin_threshold();
    }
  }
  MSD_LOG(INFO,
          "kdl_buffer = %.2f, world_model_->get_enable_cutin_threshold() = %d",
          kdl_buffer, world_model_->get_enable_cutin_threshold());

  SLPoint left_begin, left_end, right_begin, right_end;
  SLPoint left_mid1, right_mid1, left_mid2, right_mid2, left_mid3, right_mid3;

  ////////CJ:ACC mode
  if (world_model_->is_acc_mode() == true) {
    left_border_.clear();
    right_border_.clear();
    auto left_border =
        world_model_->get_baseline_info(0)->get_path_data().LeftBorder();
    auto right_border =
        world_model_->get_baseline_info(0)->get_path_data().RightBorder();
    if (left_border.size() < 3 || right_border.size() < 3) {
      return false;
    }
    for (int i = 0; i < left_border.size(); ++i) {
      left_border_.push_back(left_border[i]);
    }
    for (int i = 0; i < right_border.size(); ++i) {
      right_border_.push_back(right_border[i]);
    }
    return true;
  }

  const auto &lat_result_enu =
      context_->planning_status().planning_result.traj_pose_array;
  auto &traj_pose_frenet =
      context_->mutable_planning_status()->planning_result.traj_pose_frenet;
  traj_pose_frenet.clear();
  const auto &frenet_coord = baseline_info_->get_frenet_coord();
  Point2D frenet_p, carte_p;
  for (int i = 0; i < lat_result_enu.size(); ++i) {
    carte_p.x = lat_result_enu[i].position_enu.x;
    carte_p.y = lat_result_enu[i].position_enu.y;
    if (frenet_coord->CartCoord2FrenetCoord(carte_p, frenet_p) ==
        TRANSFORM_SUCCESS) {
      traj_pose_frenet.push_back(frenet_p);
    } else {
      break;
    }
  }

  auto lane_status = context_->mutable_planning_status()->lane_status;
  if (traj_pose_frenet.size() == 0) {
    left_begin.s = 0.0;
    left_begin.l = ego_sl.end_l + kdl_buffer;
    right_begin.s = 0.0;
    right_begin.l = ego_sl.start_l - kdl_buffer;
    left_border_.push_back(left_begin);
    right_border_.push_back(right_begin);

    MSD_LOG(INFO, "debug polygon: left_width %f, right_width %f", left_width_,
            right_width_);
    MSD_LOG(INFO,
            "debug polygon: ego s %f, start_l %f, end_l %f, ego cart %f %f",
            ego_s, ego_sl.start_l, ego_sl.end_l, ego_pose.x, ego_pose.y);

    if (lane_status.status == LaneStatus::Status::LANE_CHANGE &&
        lane_status.change_lane.status ==
            ChangeLaneStatus::Status::IN_CHANGE_LANE) {
      set_lane_changing_line_border(left_mid1, right_mid1, left_mid2,
                                    right_mid2);
    } else {
      left_mid1.s = ego_s;
      left_mid1.l = left_begin.l;
      right_mid1.s = left_mid1.s;
      right_mid1.l = right_begin.l;
      left_border_.push_back(left_mid1);
      right_border_.push_back(right_mid1);
      double ds_mid1 =
          std::min(kLbMaxDis1, std::max(ego_vel * kLbDuration1, kLbMinDis1));
      double ds_mid2 =
          std::min(kLbMaxDis2, std::max(ego_vel * kLbDuration2, kLbMinDis2));
      left_mid2.s = ego_s + ds_mid1;
      left_mid2.l = ego_r > 0.0 ? left_width_ : 0.5 * LANE_WIDTH();
      right_mid2.s = left_mid2.s;
      right_mid2.l = ego_r > 0.0 ? -0.5 * LANE_WIDTH() : -right_width_;
      left_mid3.s = ego_s + ds_mid2;
      left_mid3.l = 0.5 * LANE_WIDTH();
      right_mid3.s = left_mid3.s;
      right_mid3.l = -0.5 * LANE_WIDTH();
      if (world_model_->is_ddmap()) {
        left_mid2.l += 0.2;
        right_mid2.l -= 0.2;
        left_mid3.l -= 0.3;
        right_mid3.l += 0.3;
      }
      left_border_.push_back(left_mid2);
      right_border_.push_back(right_mid2);
      left_border_.push_back(left_mid3);
      right_border_.push_back(right_mid3);
    }

    left_end.s = ego_s + s_range_.second;
    left_end.l = 0.5 * LANE_WIDTH();
    right_end.s = ego_s + s_range_.second;
    right_end.l = -0.5 * LANE_WIDTH();
    if (world_model_->is_ddmap()) {
      left_end.l -= 0.6;
      right_end.l += 0.6;
    }

    left_border_.push_back(left_end);
    right_border_.push_back(right_end);
  } else {
    // use lat planning path at center line of ego path
    double half_width =
        ConfigurationContext::Instance()->get_vehicle_param().width / 2;
    double ds_mid1 =
        std::min(kLbMaxDis1, std::max(ego_vel * kLbDuration1, kLbMinDis1));
    double ds_mid2 =
        std::min(kLbMaxDis2, std::max(ego_vel * kLbDuration2, kLbMinDis2));
    const std::vector<double> virtual_lane_s{
        0.0, ego_s, ego_s + ds_mid1, ego_s + ds_mid2, ego_s + s_range_.second};
    const std::vector<double> virtual_lane_wid{
        half_width + kdl_buffer, half_width + kdl_buffer,
        0.5 * LANE_WIDTH() + 0.2, 0.5 * LANE_WIDTH() - 0.3,
        0.5 * LANE_WIDTH() - 0.6};
    for (int i = 0; i < virtual_lane_s.size(); ++i) {
      if (i > traj_pose_frenet.size() - 1) {
        continue;
      }
      if (virtual_lane_s[i] < traj_pose_frenet[i].x) {
        SLPoint point;
        point.s = virtual_lane_s[i];
        point.l = virtual_lane_wid[i];
        left_border_.push_back(point);
        point.l *= -1.0;
        right_border_.push_back(point);
      }
    }

    if (lane_status.status == LaneStatus::Status::LANE_CHANGE &&
        lane_status.change_lane.status ==
            ChangeLaneStatus::Status::IN_CHANGE_LANE) {
      set_lane_changing_line_border(left_mid1, right_mid1, left_mid2,
                                    right_mid2);
    } else {
      for (int i = 0; i < traj_pose_frenet.size(); ++i) {
        SLPoint point;
        double s = traj_pose_frenet[i].x;
        auto it = std::find_if(virtual_lane_s.begin(), virtual_lane_s.end(),
                               [s](const double &val) { return val > s; });
        point.s = s;
        if (it == virtual_lane_s.begin()) {
          point.l = traj_pose_frenet[i].y + virtual_lane_wid[0];
          left_border_.push_back(point);
          point.l = traj_pose_frenet[i].y - virtual_lane_wid[0];
          right_border_.push_back(point);
        } else if (it == virtual_lane_s.end()) {
          point.l = traj_pose_frenet[i].y + virtual_lane_wid.back();
          left_border_.push_back(point);
          point.l = traj_pose_frenet[i].y - virtual_lane_wid.back();
          right_border_.push_back(point);
        }
        if (it != virtual_lane_s.end()) {
          double rate = (s - *(it - 1)) / (*it - *(it - 1));
          double width = (virtual_lane_wid[it - virtual_lane_s.begin()] -
                          virtual_lane_wid[it - virtual_lane_s.begin() - 1]) *
                             rate +
                         virtual_lane_wid[it - virtual_lane_s.begin() - 1];
          point.l = traj_pose_frenet[i].y + width;
          left_border_.push_back(point);
          point.l = traj_pose_frenet[i].y - width;
          right_border_.push_back(point);
        }
      }
    }

    for (int i = 0; i < virtual_lane_s.size(); ++i) {
      if (virtual_lane_s[i] > traj_pose_frenet.back().x) {
        SLPoint point;
        point.s = virtual_lane_s[i];
        point.l = virtual_lane_wid[i];
        left_border_.push_back(point);
        point.l *= -1.0;
        right_border_.push_back(point);
      }
    }
  }
  // generate reference line polygon segments in frenet coordinate system
  // sorted s lists merge
  PolygonalLine left_line(left_border_);
  PolygonalLine right_line(right_border_);

  double cur_s = left_line.front().s;
  double next_s;
  double cur_left_l, cur_right_l, next_left_l, next_right_l;
  if (!left_line.EvaluateByS(cur_s, &cur_left_l) ||
      !right_line.EvaluateByS(cur_s, &cur_right_l)) {
    MSD_LOG(ERROR, "invalid reference line border");
  }

  left_border_.clear();
  right_border_.clear();
  SLPoint left_point(cur_s, cur_left_l), right_point(cur_s, cur_right_l);
  left_border_.push_back(left_point);
  right_border_.push_back(right_point);

  MSD_LOG(INFO, "reference line segments size: %d", int(left_line.size()) - 1);
  for (size_t s = cur_s; s < left_line.back().s; s += 2) {
    next_s = s;
    if (!left_line.EvaluateByS(next_s, &next_left_l) ||
        !right_line.EvaluateByS(next_s, &next_right_l)) {
      continue;
    }
    planning_math::Vec2d left_begin_point(cur_s, cur_left_l);
    planning_math::Vec2d left_end_point(next_s, next_left_l);
    planning_math::Vec2d right_begin_point(cur_s, cur_right_l);
    planning_math::Vec2d right_end_point(next_s, next_right_l);
    static std::vector<planning_math::Vec2d> reference_line_vertexes;
    reference_line_vertexes.clear();
    reference_line_vertexes.push_back(left_begin_point);
    reference_line_vertexes.push_back(left_end_point);
    reference_line_vertexes.push_back(right_end_point);
    reference_line_vertexes.push_back(right_begin_point);
    MSD_LOG(INFO, "ST_DEBUG1 cur_s=%.2f  cur_left_l=%.2f  cur_right_l=%.2f",
            cur_s, cur_left_l, cur_right_l);
    static planning_math::Polygon2d reference_line_segment;
    reference_line_segment.clear();
    reference_line_segment.update(reference_line_vertexes);
    reference_line_segments_.emplace_back(reference_line_segment);
    cur_s = next_s;
    cur_left_l = next_left_l;
    cur_right_l = next_right_l;
    left_border_.push_back(SLPoint(next_s, next_left_l));
    right_border_.push_back(SLPoint(next_s, next_right_l));
  }
  MSD_LOG(INFO, "ST_DEBUG1 cur_s=%.2f  cur_left_l=%.2f  cur_right_l=%.2f",
          cur_s, cur_left_l, cur_right_l);
  double half_width =
      ConfigurationContext::Instance()->get_vehicle_param().width / 2;
  return true;
}

void StGraphGenerator::compute_st_boundaries() {
  MLOG_PROFILING("compute_st_boundaries");
  auto &obstacle_manager = baseline_info_->mutable_obstacle_manager();
  auto adc_sl_boundary = baseline_info_->get_adc_sl_boundary();
  auto &obstacle_decision_manager =
      context_->mutable_obstacle_decision_manager();
  auto &pedestrian_stop_timer =
      context_->mutable_planning_status()->crosswalk.stop_times;
  const double ego_s = baseline_info_->get_ego_state().ego_frenet.x;
  constexpr double kPedestrianCompensateBuffer = 1.5;
  constexpr double kMinPedestrianStopTime = 1.0;
  const double dimension =
      static_cast<int>((time_range_.second - time_range_.first) /
                       FLAGS_speed_lon_decision_time_resolution) +
      1;
  const auto normal_obstacles_items =
      baseline_info_->obstacle_manager().get_obstacles().Items();
  const auto virtual_obstacles_items =
      context_->get_virtual_obstacles().Items();
  auto obstacles_items = normal_obstacles_items;

  obstacles_items.insert(obstacles_items.end(), virtual_obstacles_items.begin(),
                         virtual_obstacles_items.end());

  for (const auto &obstacle : obstacles_items) {
    double compensate_avoid_buffer = 0.0;
    if (pedestrian_stop_timer.find(obstacle->Id()) !=
        pedestrian_stop_timer.end()) {
      double current_time_seconds = MTIME()->timestamp().sec();
      double continued_avoid_duration =
          current_time_seconds - pedestrian_stop_timer[obstacle->Id()];

      if (continued_avoid_duration >= kPedestrianStopTimeout) {
        compensate_avoid_buffer = 0.0;
      } else {
        compensate_avoid_buffer =
            kPedestrianCompensateBuffer *
            std::max(0.0,
                     1 - clip(continued_avoid_duration - kMinPedestrianStopTime,
                              kPedestrianStopTimeout, 0.0) /
                             kPedestrianStopTimeout);
      }
    }
    auto perception_sl_boundary = obstacle->PerceptionSLBoundary();
    const auto &sl_polygon_seq = obstacle->sl_polygon_seq();
    if (sl_polygon_seq.empty()) {
      MSD_LOG(INFO, "DEBUG_CJ8:ID:%d,polygon_empty CONTINUE", obstacle->Id());
      continue;
    }
    static std::vector<STPoint> lower_points, upper_points;
    lower_points.clear();
    upper_points.clear();
    Obstacle *ptr_obstacle = obstacle_manager.find_obstacle(obstacle->Id());
    if (ptr_obstacle == nullptr) {
      ptr_obstacle = context_->find_virtual_obstacle(obstacle->Id());
    }
    auto ptr_obstacle_decision =
        obstacle_decision_manager.find_obstacle_decision(ptr_obstacle->Id());

    static std::vector<planning_math::Polygon2d> reference_line_segments;
    reference_line_segments.clear();
    double cur_s, next_s;
    double cur_left_l, cur_right_l, next_left_l, next_right_l;

    cur_s = left_border_[0].s;
    cur_left_l = left_border_[0].l + obstacle->avdDisBuffer();
    cur_right_l = right_border_[0].l - obstacle->avdDisBuffer();

    double left_compensate_avoid_buffer{0.0},
        right_compensate_avoid_buffer{0.0};

    if (obstacle->PerceptionSLBoundary().end_l < adc_sl_boundary.start_l) {
      right_compensate_avoid_buffer = -compensate_avoid_buffer;
    } else if (obstacle->PerceptionSLBoundary().start_l >
               adc_sl_boundary.end_l) {
      left_compensate_avoid_buffer = compensate_avoid_buffer;
    } else {
      left_compensate_avoid_buffer = compensate_avoid_buffer;
      right_compensate_avoid_buffer = -compensate_avoid_buffer;
    }
    cur_left_l += left_compensate_avoid_buffer;
    cur_right_l += right_compensate_avoid_buffer;

    MSD_LOG(INFO, "ST_DEBUG2 ID:%d,ped_buf=%.2f  obs_buf=%.2f", obstacle->Id(),
            compensate_avoid_buffer, obstacle->avdDisBuffer());

    for (size_t i = 1; i < left_border_.size(); ++i) {
      next_s = left_border_[i].s;
      next_left_l = left_border_[i].l + obstacle->avdDisBuffer() +
                    left_compensate_avoid_buffer;
      next_right_l = right_border_[i].l - obstacle->avdDisBuffer() +
                     right_compensate_avoid_buffer;
      MSD_LOG(INFO, "ST_DEBUG2 ID:%d,cur_s=%.2f left_l=%.2f right_l=%.2f",
              obstacle->Id(), cur_s, left_border_[i].l, right_border_[i].l);
      MSD_LOG(INFO,
              "ST_DEBUG2 ID:%d,cur_s=%.2f avd_l=%.2f l_com_l=%.2f r_com_l=%.2f",
              obstacle->Id(), cur_s, obstacle->avdDisBuffer(),
              left_compensate_avoid_buffer, right_compensate_avoid_buffer);
      planning_math::Vec2d left_begin_point(cur_s, cur_left_l);
      planning_math::Vec2d left_end_point(next_s, next_left_l);
      planning_math::Vec2d right_begin_point(cur_s, cur_right_l);
      planning_math::Vec2d right_end_point(next_s, next_right_l);

      static std::vector<planning_math::Vec2d> reference_line_vertexes;
      reference_line_vertexes.clear();
      reference_line_vertexes.push_back(left_begin_point);
      reference_line_vertexes.push_back(left_end_point);
      reference_line_vertexes.push_back(right_end_point);
      reference_line_vertexes.push_back(right_begin_point);

      MSD_LOG(INFO,
              "ST_DEBUG2 ID:%d,cur_s=%.2f cur_left_l=%.2f cur_right_l=%.2f",
              obstacle->Id(), cur_s, cur_left_l, cur_right_l);
      static planning_math::Polygon2d reference_line_segment;
      reference_line_segment.clear();
      reference_line_segment.update(reference_line_vertexes);
      reference_line_segments.emplace_back(reference_line_segment);
      cur_s = next_s;
      cur_left_l = next_left_l;
      cur_right_l = next_right_l;
    }
    MSD_LOG(INFO, "ST_DEBUG2 ID:%d,cur_s=%.2f cur_left_l=%.2f cur_right_l=%.2f",
            obstacle->Id(), cur_s, cur_left_l, cur_right_l);

    if (st_graph_construct_mode == STGraphConstructMode::SKETCHY_MODE) {
      if (obstacle->IsStatic()) {
        double s_lower = std::numeric_limits<double>::infinity();
        double s_upper = 0.0;
        static PolygonWithT cur_sl_polygon_t;
        cur_sl_polygon_t.second.clear();
        if (!sl_polygon_seq.EvaluateByTime(0.0, &cur_sl_polygon_t)) {
          MSD_LOG(INFO, "DEBUG_CJ8:ID:%d,Static,CONTINUE");
          continue;
        }
        static planning_math::Polygon2d cur_sl_polygon;
        cur_sl_polygon = cur_sl_polygon_t.second;
        auto s_range =
            sl_polygon_path_check(reference_line_segments, cur_sl_polygon);
        s_lower = s_range.first;
        s_upper = s_range.second;
        MSD_LOG(INFO,
                "DEBUG_CJ881:ID:%d,x_min:%.2f,x_max:%.2f,y_min:%.2f,y_"
                "max:%.2f",
                obstacle->Id(), cur_sl_polygon.min_x(), cur_sl_polygon.max_x(),
                cur_sl_polygon.min_y(), cur_sl_polygon.max_y());
        MSD_LOG(INFO, "DEBUG_CJ8:ID:%d,Static,s_lower:%3.1f,s_upper:%3.1f",
                obstacle->Id(), s_lower, s_upper);
        if (s_lower > ego_s + s_range_.second || s_upper < s_range_.first ||
            s_lower >= s_upper) {
          MSD_LOG(INFO, "DEBUG_CJ8:ID:%d,Static,OUT OF s_range",
                  obstacle->Id());
          continue;
        }
        lower_points.push_back(STPoint(s_lower, 0.0));
        upper_points.push_back(STPoint(s_upper, 0.0));
        lower_points.push_back(STPoint(s_lower, time_range_.second));
        upper_points.push_back(STPoint(s_upper, time_range_.second));
        auto st_boundary =
            STBoundary::CreateInstance(lower_points, upper_points);
        ptr_obstacle->set_path_st_boundary(st_boundary);
      } else {
        static std::vector<std::pair<double, double>> invalid_time_sections;
        invalid_time_sections.clear();
        std::pair<double, double> invalid_time_section{8.0, 0.0};
        bool has_invalid_time_sec{false};
        for (size_t i = 0; i < dimension; ++i) {
          double relative_time = i * FLAGS_speed_lon_decision_time_resolution;
          double s_lower = std::numeric_limits<double>::infinity();
          double s_upper = 0.0;
          static PolygonWithT cur_sl_polygon_t;
          cur_sl_polygon_t.second.clear();
          if (!sl_polygon_seq.EvaluateByTime(relative_time,
                                             &cur_sl_polygon_t)) {
            if (!has_invalid_time_sec) {
              has_invalid_time_sec = true;
              if (lower_points.empty()) {
                invalid_time_section.first = relative_time;
              } else {
                invalid_time_section.first =
                    (lower_points.back().t() + relative_time) / 2.0;
              }
            }
            invalid_time_section.second =
                std::max(invalid_time_section.second, relative_time);
            if (int(i) == int(dimension) - 1) {
              invalid_time_sections.emplace_back(invalid_time_section);
            }
            MSD_LOG(INFO,
                    "DEBUG_CJ8:ID:%d,CONTINUE,rel_time:%f:", obstacle->Id(),
                    relative_time);
            continue;
          }
          static planning_math::Polygon2d cur_sl_polygon;
          cur_sl_polygon = cur_sl_polygon_t.second;
          auto s_range =
              sl_polygon_path_check(reference_line_segments, cur_sl_polygon);
          s_lower = s_range.first;
          s_upper = s_range.second;

          MSD_LOG(INFO,
                  "DEBUG_CJ881:ID:%d,i:%d,x_min:%.2f,x_max:%.2f,y_min:%.2f,y_"
                  "max:%.2f",
                  obstacle->Id(), i, cur_sl_polygon.min_x(),
                  cur_sl_polygon.max_x(), cur_sl_polygon.min_y(),
                  cur_sl_polygon.max_y());
          auto traj_point0 = obstacle->GetPointAtTime(0.0);
          double Fusion_Yaw = obstacle->PerceptionInfo().heading_yaw;
          double Pred_Yaw = traj_point0.path_point.theta;
          MSD_LOG(INFO, "DEBUG_CJ881:ID:%d,i:%d,Fusion_Yaw:%f,Pred_Yaw:%f",
                  obstacle->Id(), i, Fusion_Yaw, Pred_Yaw);
          MSD_LOG(INFO,
                  "DEBUG_CJ81:ID:%d, "
                  "i:%d,seg_end:x_min:%2.2f,x_max:%2.2f,y_min:%2.2f,"
                  "y_max:%2.2f",
                  obstacle->Id(), i, reference_line_segments.back().min_x(),
                  reference_line_segments.back().max_x(),
                  reference_line_segments.back().min_y(),
                  reference_line_segments.back().max_y());

          MSD_LOG(INFO, "DEBUG_CJ81:ID:%d,i:%d,s_lower:%3.1f,s_upper:%3.1f",
                  obstacle->Id(), i, std::fmin(s_lower, 500), s_upper);

          if (s_lower > s_range_.second || s_upper < s_range_.first ||
              s_lower >= s_upper) {
            if (!has_invalid_time_sec) {
              has_invalid_time_sec = true;
              if (lower_points.empty()) {
                invalid_time_section.first = relative_time;
              } else {
                invalid_time_section.first =
                    (lower_points.back().t() + relative_time) / 2.0;
              }
            }
            invalid_time_section.second =
                std::max(invalid_time_section.second, relative_time);
            if (int(i) == int(dimension) - 1) {
              invalid_time_sections.emplace_back(invalid_time_section);
            }
            MSD_LOG(INFO, "DEBUG_CJ81:ID:%d,i:%d,OUT OF s_range,rel_time:%2.2f",
                    obstacle->Id(), i, relative_time);
            continue;
          }
          if (has_invalid_time_sec) {
            has_invalid_time_sec = false;
            invalid_time_section.second =
                std::max(invalid_time_section.second,
                         (invalid_time_section.second + relative_time) / 2.0);
            invalid_time_sections.emplace_back(invalid_time_section);
            invalid_time_section = {8.0, 0.0};
          }
          lower_points.push_back(STPoint(s_lower, relative_time));
          upper_points.push_back(STPoint(s_upper, relative_time));
        }
        if (lower_points.size() > 1 && upper_points.size() > 1) {
          auto st_boundary =
              STBoundary::CreateInstance(lower_points, upper_points);
          st_boundary.set_invalid_time_sections(invalid_time_sections);
          ptr_obstacle->set_path_st_boundary(st_boundary);
        }
      }
    } else if (st_graph_construct_mode == STGraphConstructMode::ACCURATE_MODE) {
      auto st_boundary = obstacle->path_st_boundary();
      if (st_boundary.upper_points().size() > 1) {
        auto shifted_st_boundary = st_boundary.ShiftByS(
            ego_s +
            ConfigurationContext::Instance()->get_vehicle_param().length / 2.0);
        ptr_obstacle->set_path_st_boundary(shifted_st_boundary);
      }
    }
  }
}

void StGraphGenerator::set_obstacle_new(const Obstacle *obstacle) {
  const auto &st_boundary = obstacle->path_st_boundary();
  auto perception_sl_boundary = obstacle->PerceptionSLBoundary();
  auto sl_polygon_seq = obstacle->sl_polygon_seq();
  auto &obstacle_manager = baseline_info_->mutable_obstacle_manager();
  auto &obstacle_decision_manager =
      context_->mutable_obstacle_decision_manager();
  auto ptr_obstacle_decision =
      obstacle_decision_manager.find_obstacle_decision(obstacle->Id());
  auto &lon_obstacle_decision_info_map =
      context_->mutable_lon_decison_output()->obstacle_decision_info_map;
  const auto &frenet_coord = baseline_info_->get_frenet_coord();
  const double ego_vel = baseline_info_->get_ego_state().ego_vel;
  double mtime = MTIME()->timestamp().sec() * 100;
  mtime = (unsigned long)mtime % 100000;
  mtime /= 100;
  MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, ==================", obstacle->Id());
  MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, Time:%2f", obstacle->Id(), mtime);
  // CJ: construct new objs in lon_decision_log
  if (lon_obstacle_decision_info_map.find(obstacle->Id()) ==
      lon_obstacle_decision_info_map.end()) {
    auto &lon_decision = lon_obstacle_decision_info_map[obstacle->Id()];
    lon_decision.is_follow = false;
    lon_decision.exist = true;
    lon_decision.cut_in_flag = false;
    lon_decision.cut_in_count = 0;
    lon_decision.crossing = false;
    lon_decision.on_heading = false;
    lon_decision.in_lane = obstacle->IsEgoLaneOverlap();
    lon_decision.collision = false;
    lon_decision.collision_time = -1.0;
    lon_decision.lat_dist_head = 99;
    lon_decision.obj_vy_dy = 0;
    lon_decision.follow_time_count = 0;
    lon_decision.ignore_time_count = 100;
    lon_decision.overlap = 0;
    lon_decision.type = (int)ObjectType::NOT_KNOW;
    lon_decision.virtual_overlap = -100;
    MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, Build New Obj!", obstacle->Id());
  } else {
    auto &lon_decision = lon_obstacle_decision_info_map[obstacle->Id()];
    lon_decision.is_follow = false;
    lon_decision.exist = true;
    lon_decision.cut_in_flag = false;
    // lon_decision.cut_in_count;
    lon_decision.crossing = false;
    lon_decision.on_heading = false;
    lon_decision.in_lane = obstacle->IsEgoLaneOverlap();
    lon_decision.collision = false;
    lon_decision.collision_time = -1.0;
    lon_decision.overlap = 0;
    // lon_decision.lat_dist_head;
    // lon_decision.obj_vy_d;
    // lon_decision.follow_time_count;
    // lon_decision.ignore_time_count;
    MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, Exist Old Obj!", obstacle->Id());
  }
  MSD_LOG(INFO, "DEBUG_CJ_Vy2:ID:%d, obj_size:%d", obstacle->Id(),
          lon_obstacle_decision_info_map.size());
  auto &lon_decision_info = lon_obstacle_decision_info_map[obstacle->Id()];

  auto &obj_mem_info_map = context_->mutable_planner_debug()->obj_mem_info_map;
  if (obj_mem_info_map.find(obstacle->Id()) == obj_mem_info_map.end()) {
    obj_mem_info_map[obstacle->Id()].lat_dist_head = 99;
    obj_mem_info_map[obstacle->Id()].obj_vy_dy = 0;
    obj_mem_info_map[obstacle->Id()].exist = true;
    obj_mem_info_map[obstacle->Id()].collision = false;
    obj_mem_info_map[obstacle->Id()].collision_time = -1.0;
    obj_mem_info_map[obstacle->Id()].in_lane = false;
    obj_mem_info_map[obstacle->Id()].follow_k1 = false;
    obj_mem_info_map[obstacle->Id()].follow_count = 0;
    obj_mem_info_map[obstacle->Id()].cut_in_k1 = false;
    obj_mem_info_map[obstacle->Id()].cut_in_count = 0;
    obj_mem_info_map[obstacle->Id()].rule_base_cutin = false;
    obj_mem_info_map[obstacle->Id()].ftp_cutin = false;
    obj_mem_info_map[obstacle->Id()].lat_cutin = false;
    obj_mem_info_map[obstacle->Id()].overlap = 0;
    obj_mem_info_map[obstacle->Id()].type = (int)ObjectType::NOT_KNOW;
    obj_mem_info_map[obstacle->Id()].virtual_overlap = -100;
  } else {
    obj_mem_info_map[obstacle->Id()].exist = true;
  }

  if (sl_polygon_seq.empty()) {
    auto ptr_obstacle_decision =
        obstacle_decision_manager.find_obstacle_decision(obstacle->Id());
    if (ptr_obstacle_decision) {
      append_ignore_decision(ptr_obstacle_decision);
      MSD_LOG(INFO, "DEBUG_CJ81:polygen_empty ID:%d", obstacle->Id());
    }
    LOG_LON_DECISION_INFO(obstacle->Id(), "ignore", "polygon_seq is empty");
    return;
  }

  // CJ: basic motion info calculatte
  bool is_follow = false;
  const EgoState &ego_state = baseline_info_->get_ego_state();

  double ego_theta = ego_state.ego_pose.theta -
                     frenet_coord->GetRefCurveHeading(ego_state.ego_frenet.x);
  const double on_heading_yaw_thr_deg = 160.0;
  double theta = planning_math::NormalizeAngle(obstacle->Yaw_relative_frenet());
  bool crossing = false;
  double obj_vx = obstacle->speed() * std::cos(theta);
  double obj_vy = obstacle->speed() * std::sin(theta);
  double long_ttc_tail = 99; // ttc between ego head and obj tail
  double long_ttc_head = 99; // ttc between ego head and obj head
  double rel_vx = ego_state.ego_vel - obj_vx;
  double ego_s = ego_state.ego_frenet.x;
  double ego_l = ego_state.ego_frenet.y;
  const double half_length =
      ConfigurationContext::Instance()->get_vehicle_param().length / 2.0;
  const double half_width =
      ConfigurationContext::Instance()->get_vehicle_param().width / 2.0;
  double lat_ttc_head = 99, lat_ttc_tail = 99;
  const double K_LongLatTTCDiff_Thr_s = 1.0;
  const double K_LongLatTTCDiffVRU_Thr_s = 3.0;
  const double K_LatLongTTCDiff_Thr_s = 2.0;
  const double K_CrossVy_Thr_mps = 3.0;
  const double K_CrossLongTTC_Thr_s = 2.0;
  const double K_CrossLongTTCVRU_Thr_s = 3.0;
  const double K_CrossEgoV_Thr = 3.0;
  double lat_dist_head = 99, lat_dist_tail = 99, lat_dist_head_ego = 99;
  auto traj_point_0 = obstacle->GetPointAtTime(0.0);
  auto polygon_0 = obstacle->GetPolygonAtPoint(traj_point_0);
  double min_l = 10000, max_l = -99;
  double min_s = 10000, max_s = -99;
  Point2D obj_frenet_p{0.0, 0.0}, obj_carte_p{0.0, 0.0};
  MSD_LOG(INFO, "DEBUG_CJ_Yaw:ID:%d,fusion_yaw:%.2f,pred_yaw:%.2f",
          obstacle->Id(), obstacle->PerceptionInfo().heading_yaw * 57.3,
          traj_point_0.path_point.theta * 57.3);
  // CJ: calculate ttc vy etc.
  for (auto carte_point : polygon_0.points()) {
    obj_carte_p.x = carte_point.x();
    obj_carte_p.y = carte_point.y();
    if (frenet_coord->CartCoord2FrenetCoord(obj_carte_p, obj_frenet_p) ==
        TRANSFORM_SUCCESS) {
      MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, obj_corner:[%.2f,%.2f]", obstacle->Id(),
              obj_frenet_p.x, obj_frenet_p.y);
      if (min_l > obj_frenet_p.y) {
        min_l = obj_frenet_p.y;
      }
      if (max_l < obj_frenet_p.y) {
        max_l = obj_frenet_p.y;
      }
      if (min_s > obj_frenet_p.x) {
        min_s = obj_frenet_p.x;
      }
      if (max_s < obj_frenet_p.x) {
        max_s = obj_frenet_p.x;
      }
    } else {
      min_l = obstacle->R_frenet();
      max_l = obstacle->R_frenet();
      min_s = obstacle->S_frenet();
      max_s = obstacle->S_frenet();
    }
  }
  if (rel_vx > 0.1 && min_s - ego_state.ego_frenet.x - half_length > 0.1) {
    long_ttc_tail = (min_s - ego_state.ego_frenet.x - half_length) / rel_vx;
  }
  if (rel_vx > 0.1 && max_s - ego_state.ego_frenet.x - half_length > 0.1) {
    long_ttc_head = (max_s - ego_state.ego_frenet.x - half_length) / rel_vx;
  }
  if (obj_vy < -0.1) {
    lat_dist_head = min_l;
    lat_dist_head_ego = min_l - ego_l;
    lat_dist_tail = max_l - ego_l;
  } else if (obj_vy > 0.1) {
    lat_dist_head = max_l;
    lat_dist_head_ego = max_l - ego_l;
    lat_dist_tail = min_l - ego_l;
  } else {
    lat_dist_head = std::fabs(min_l) < std::fabs(max_l) ? min_l : max_l;
    lat_dist_head_ego = lat_dist_head - ego_l;
    lat_dist_tail = std::fabs(min_l) > std::fabs(max_l) ? min_l : max_l - ego_l;
  }
  double refline_cur = frenet_coord->GetRefCurveCurvature(max_s);
  MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, refline_cur:%f", obstacle->Id(),
          refline_cur);
  MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, miny:%.2f, maxy:%.2f, ego_l:%.2f",
          obstacle->Id(), min_l, max_l, ego_l);
  // double min_fre_l = std::fabs(min_l) < std::fabs(max_l) ? min_l : max_l;
  const double centering_t_s = 1.0;
  double dist_ratio = (max_s - ego_s) / (ego_vel * centering_t_s);
  dist_ratio = std::fmax(dist_ratio, 0.0);
  dist_ratio = std::fmin(dist_ratio, 1.0);
  lat_dist_head =
      lat_dist_head * dist_ratio + (1 - dist_ratio) * lat_dist_head_ego;
  MSD_LOG(INFO,
          "DEBUG_CJ884 ID:%d, obj_vy:%.2f, lat_dist_head:%.2f, "
          "lat_dist_head_ego:%.2f",
          obstacle->Id(), obj_vy, lat_dist_head, lat_dist_head_ego);
  double overlap_ref = 0, overlap_ego = 0, overlap = 0;
  std::vector<double> overlap_ego_p, overlap_ref_p;
  overlap_ego_p.clear();
  overlap_ego_p.push_back(min_l);
  overlap_ego_p.push_back(max_l);
  overlap_ego_p.push_back(ego_l - half_width);
  overlap_ego_p.push_back(ego_l + half_width);
  std::sort(overlap_ego_p.begin(), overlap_ego_p.end());

  if (overlap_ego_p.back() - overlap_ego_p.front() >=
      max_l - min_l + half_width * 2) {
    overlap_ego = 0;
  } else if (overlap_ego_p.back() - overlap_ego_p.front() <
             half_width * 2 + 0.01) {
    overlap_ego = std::min(overlap_ego_p[2] - overlap_ego_p[0],
                           overlap_ego_p[3] - overlap_ego_p[1]);
  } else {
    overlap_ego = overlap_ego_p[2] - overlap_ego_p[1];
  }
  overlap_ref_p.clear();
  overlap_ref_p.push_back(min_l);
  overlap_ref_p.push_back(max_l);
  overlap_ref_p.push_back(-half_width);
  overlap_ref_p.push_back(half_width);
  std::sort(overlap_ref_p.begin(), overlap_ref_p.end());
  if (overlap_ref_p.back() - overlap_ref_p.front() >=
      max_l - min_l + half_width * 2) {
    overlap_ref = 0;
  } else if (overlap_ref_p.back() - overlap_ref_p.front() <
             half_width * 2 + 0.01) {
    overlap_ref = std::min(overlap_ref_p[2] - overlap_ref_p[0],
                           overlap_ref_p[3] - overlap_ref_p[1]);
  } else {
    overlap_ref = overlap_ref_p[2] - overlap_ref_p[1];
  }
  overlap = dist_ratio * overlap_ref + (1 - dist_ratio) * overlap_ego;
  lon_decision_info.overlap = overlap;
  // Calculate virtual_overlap: overlap of obj and virtual lane
  PolygonalLine left_border(left_border_);
  PolygonalLine right_border(right_border_);
  double left_border_l = 1.0;
  left_border.EvaluateByS(obstacle->S_frenet(), &left_border_l);
  double right_border_l = -1.0;
  right_border.EvaluateByS(obstacle->S_frenet(), &right_border_l);
  double virtual_overlap = -100;
  MSD_LOG(INFO, "ENV:ID:%d,obj_s:%.2f,lb:%.2f,rb:%.2f", obstacle->Id(),
          obstacle->S_frenet(), left_border_l, right_border_l);
  std::vector<double> virtual_overlap_p;
  virtual_overlap_p.clear();
  virtual_overlap_p.push_back(min_l);
  virtual_overlap_p.push_back(max_l);
  virtual_overlap_p.push_back(left_border_l);
  virtual_overlap_p.push_back(right_border_l);
  std::sort(virtual_overlap_p.begin(), virtual_overlap_p.end());

  if (virtual_overlap_p.back() - virtual_overlap_p.front() >=
      max_l - min_l + left_border_l - right_border_l) {
    if (max_l < right_border_l) {
      virtual_overlap = max_l - right_border_l;
    } else if (min_l > left_border_l) {
      virtual_overlap = left_border_l - min_l;
    } else {
      virtual_overlap = 0;
    }
  } else if (virtual_overlap_p.back() - virtual_overlap_p.front() <
             left_border_l - right_border_l + 0.01) {
    virtual_overlap = std::min(virtual_overlap_p[2] - virtual_overlap_p[0],
                               virtual_overlap_p[3] - virtual_overlap_p[1]);
  } else {
    virtual_overlap = virtual_overlap_p[2] - virtual_overlap_p[1];
  }
  lon_decision_info.virtual_overlap = virtual_overlap;
  const double vy_fil_fac = 0.2;
  double obj_vy_dy = 0;
  double last_lat_dist_head = lat_dist_head;
  const double cycle_s = 0.1;
  if (std::fabs(lon_decision_info.lat_dist_head) < 98) {
    last_lat_dist_head = lon_decision_info.lat_dist_head;
    double last_vy_dy = lon_decision_info.obj_vy_dy;
    obj_vy_dy = (lat_dist_head - last_lat_dist_head) / cycle_s;
    obj_vy_dy = obj_vy_dy * vy_fil_fac + last_vy_dy * (1 - vy_fil_fac);
    lon_decision_info.obj_vy_dy = obj_vy_dy;
    lon_decision_info.lat_dist_head = lat_dist_head;
    MSD_LOG(INFO, "DEBUG_CJ_Vy:Old ID:%d,Vy:%.2f,last_dy:%.2f,dy:%.2f",
            obstacle->Id(), obj_vy_dy, last_lat_dist_head, lat_dist_head);
  } else {
    MSD_LOG(INFO, "DEBUG_CJ_Vy:New ID:%d,Vy:%.2f", obstacle->Id(), obj_vy);
    lon_decision_info.obj_vy_dy = obj_vy_dy;
    lon_decision_info.lat_dist_head = lat_dist_head;
  }
  // obj_vy = obj_vy_dy;
  obj_vy_dy = obj_vy;
  if (obj_vy * lat_dist_head < 0) {
    lat_ttc_head = std::fmax((std::fabs(lat_dist_head) - half_width), 0) /
                   std::fmax(0.1, std::fabs(obj_vy));
  } else {
    lat_ttc_head = 0;
  }
  if (obj_vy * lat_dist_tail < 0) {
    lat_ttc_tail = (std::fabs(lat_dist_tail) + half_width) /
                   std::fmax(0.1, std::fabs(obj_vy));
  } else {
    lat_ttc_tail = 0;
  }
  // CJ: decide crosssing atttribute
  // CJ:there is K_LongLatTTCDiff_Thr_s time left for ego car to get to
  // the cross point after obj left ego lane
  // 1.obj will cross ego lane before ego get to collision point long_ttc >
  // lat_ttc 2.ego will pass collision point before obj get into ego lane
  // lat_ttc > long_ttc
  bool is_VRU = obstacle->Type() == ObjectType::PEDESTRIAN ||
                obstacle->Type() == ObjectType::OFO ||
                obstacle->Type() == ObjectType::TRICYCLE;
  if (((long_ttc_tail - lat_ttc_tail > K_LongLatTTCDiff_Thr_s && !is_VRU ||
        long_ttc_tail - lat_ttc_tail > K_LongLatTTCDiffVRU_Thr_s && is_VRU) &&
       std::fabs(obj_vy) > K_CrossVy_Thr_mps && ego_vel > K_CrossEgoV_Thr) ||
      ((lat_ttc_head - long_ttc_head > K_LatLongTTCDiff_Thr_s && !is_VRU ||
        lat_ttc_head - long_ttc_head > K_CrossLongTTCVRU_Thr_s && is_VRU) &&
       long_ttc_head < K_CrossLongTTC_Thr_s)) {
    crossing = true;
  }
  lon_decision_info.crossing = crossing;
  MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, crossing:%d, obj_vx:%.2f, obj_vy:%.2f",
          obstacle->Id(), crossing, obj_vx, obj_vy);
  MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, long_ttc_head:%.2f,long_ttc_tail:%.2f",
          obstacle->Id(), long_ttc_head, long_ttc_tail);
  MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, lat_ttc_head:%.2f, lat_ttc_tail:%.2f",
          obstacle->Id(), lat_ttc_head, lat_ttc_tail);
  MSD_LOG(INFO,
          "DEBUG_CJ884 ID:%d, min_s:%.2f, ego_frent_x:%.2f, half_len:%.2f, "
          "rel_vel:%.2f",
          obstacle->Id(), min_s, ego_state.ego_frenet.x, half_length, rel_vx);

  // CJ: decide CutIn attribute
  bool cut_in_flag = false;
  const double K_CutInVyThr1 = 0.3;
  const double K_CutInDyTrh1 = half_width + 2.0;
  const double K_CutInLatTTCThr = 1.5;
  const double K_CutInHeadwayRange_s = 3.6;
  const double K_CutInDistMin_m = 30;
  const double K_CutInDistMax_m = 60;
  const double K_LargeYawThr_rad = 5 / 57.3;
  const double K_LargeYawLatBuff_m = 0.5;
  const double K_LargeYawVyThr_mps = 0.17;
  const double K_CutInVyThr_mps = 0.1;
  const double K_CutInSpeedLimit = 0.5;
  auto ref_line = world_model_->get_map_info().current_refline_points();
  double lane_width = 3.75;
  if (ref_line.size() > 0) {
    lane_width = std::fmin(ref_line[0].lane_width, 3.75);
  }
  MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, Lane_Width:%.2f,ego_width:%.2f",
          obstacle->Id(), lane_width, half_width * 2);
  double CutInDistRange = ego_state.ego_vel * K_CutInHeadwayRange_s;
  CutInDistRange = std::fmax(CutInDistRange, K_CutInDistMin_m);
  CutInDistRange = std::fmin(CutInDistRange, K_CutInDistMax_m);
  MSD_LOG(INFO,
          "DEBUG_CJ884 ID:%d, objs_s:%.2f,ego_s:%.2f,s_lim:%.2f,is_cutin:%d",
          obstacle->Id(), max_s, ego_s, CutInDistRange, obstacle->IsCutin());

  bool long_dis_obj = long_ttc_head * rel_vx > 80.0;
  // CJ: 1.ego car is not stopped
  // 2.obj is getting closer in lat
  // 3.obj is ahead of ego
  // 4.obj is not too far(far obj doesn't need quick cut-in detection)
  // 5.lat ttc is small & obj vy is obvious & lat dist is short
  // 6.low speed large yaw obj
  const double ego_stop_spd_mps = 0.5;
  const double K_FusionDisableCurThr = 0.0015;
  bool rule_base_cutin =
      obj_vy * lat_dist_head < 0 && max_s - ego_s > half_length &&
      max_s - ego_s < CutInDistRange && obstacle->speed() > K_CutInSpeedLimit &&
      std::fabs(refline_cur) < K_FusionDisableCurThr &&
      ((lat_ttc_head < K_CutInLatTTCThr &&
        std::fabs(lat_dist_head) < K_CutInDyTrh1 &&
        std::fabs(obj_vy) > K_CutInVyThr_mps) ||
       (theta * lat_dist_head < 0 && std::fabs(theta) > K_LargeYawThr_rad &&
        std::fabs(lat_dist_head) < lane_width / 2 + K_LargeYawLatBuff_m &&
        std::fabs(obj_vy) > K_LargeYawVyThr_mps));

  const double ftp_cutin_deactivation_speed = 100 / 3.6;

  if ((ftp_cutin_deactivation_speed > ego_vel) &&
      (ego_vel > ego_stop_spd_mps) &&
      (obstacle->IsCutin() || rule_base_cutin)) {
    cut_in_flag = true;
    lon_decision_info.cut_in_flag = cut_in_flag;
    lon_decision_info.cut_in_count++;
  } else if (ego_vel > ftp_cutin_deactivation_speed && rule_base_cutin) {
    cut_in_flag = true;
    lon_decision_info.cut_in_flag = cut_in_flag;
    lon_decision_info.cut_in_count++;
  } else {
    lon_decision_info.cut_in_flag = cut_in_flag;
    lon_decision_info.cut_in_count = 0;
  }
  MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, cut_in_flag:%d, cut_in_count:%d",
          obstacle->Id(), cut_in_flag, lon_decision_info.cut_in_count);

  // CJ: decide on headdinng attribute
  double ref_line_end = world_model_->get_current_last_car_point_x();
  const double K_refline_available_hw = 1.5;
  const double K_refline_available_dist = 30.0;
  double refline_dist_thr =
      std::fmin(K_refline_available_hw * ego_vel, K_refline_available_dist);
  MSD_LOG(INFO, "DEBUG_CJ912: ref_line_end:%f", ref_line_end);
  // CJ: Find out on_heading objs
  bool on_heading = false;
  const double K_InLaneOnHeadingLDEndOffset_m = 2.0;
  if (std::fabs(theta) > on_heading_yaw_thr_deg / 180.0 * M_PI &&
      obstacle->speed() > 3) {
    // confirmed in lane obj would not be set as on_heading
    if (!(std::fabs(lat_dist_head + lat_dist_tail) < lane_width &&
          ref_line_end > max_s + K_InLaneOnHeadingLDEndOffset_m)) {
      on_heading = true;
    }
  }
  lon_decision_info.on_heading = on_heading;
  MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, on_heading:%d, delta_yaw:%.2f, spd:%.2f",
          obstacle->Id(), on_heading, theta * 57.3, obstacle->speed());
  MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, ego_yaw:%.2f, ego_fre_yaw:%.2f",
          obstacle->Id(), ego_state.ego_pose.theta * 57.3, ego_theta * 57.3);

  // CJ: decide time range of prediction
  double CIPV_check_time_range = 2.5;
  if (long_dis_obj) {
    CIPV_check_time_range = 0.25;
  }
  if (obstacle->IsEgoLaneOverlap()) {
    CIPV_check_time_range = 5.0;
  }
  if (ego_vel < ego_stop_spd_mps || on_heading == true) {
    CIPV_check_time_range = FLAGS_speed_lon_decision_time_resolution * 1.5;
  }
  const double Lat_TTC_Ignore_Thr = 2.0;
  if (std::fabs(lat_dist_head) > lane_width) {
    CIPV_check_time_range = 2.0;
  }
  const int CIPV_check_cycle =
      int(CIPV_check_time_range / FLAGS_speed_lon_decision_time_resolution);
  MSD_LOG(INFO, "DEBUG_CJ881:Obj_ID:%d,Type:%d,TimeRange:%d,InLane:%d",
          obstacle->Id(), obstacle->Type(), CIPV_check_cycle,
          obstacle->IsEgoLaneOverlap());

  // CJ: decide follow decision
  double collision_time = -1.0;
  for (size_t i = 0; i < CIPV_check_cycle; ++i) {
    double relative_time = i * FLAGS_speed_lon_decision_time_resolution;
    static PolygonWithT cur_polygon;
    cur_polygon.second.clear();
    MSD_LOG(INFO, "DEBUG_CJ889:i:%d,Obj_ID:%d", i, obstacle->Id());
    if (!sl_polygon_seq.EvaluateByTime(relative_time, &cur_polygon)) {
      MSD_LOG(INFO, "DEBUG_CJ889:i:%d,Obj_ID:%d,EvaluateByTime Failed", i,
              obstacle->Id());
      if (CIPV_check_cycle == 1) {
        LOG_LON_DECISION_INFO(obstacle->Id(), "ignore",
                              "ignore because polygon get failed");
      }
      continue;
    }

    double s_lower = std::numeric_limits<double>::infinity();
    double s_upper = 0.0;

    bool is_obs_collide_with_path = false;
    if (st_boundary.GetBoundarySRange(relative_time, &s_upper, &s_lower)) {
      is_obs_collide_with_path = true;
    }
    MSD_LOG(INFO, "DEBUG_CJ889:Obj_ID:%d,Collision:%d,rel_time:%.2f",
            obstacle->Id(), is_obs_collide_with_path, relative_time);

    if (is_obs_collide_with_path) {
      MSD_LOG(INFO,
              "DEBUG_CJ889:Obj_ID:%d,s_upper:%.1f,s_lower:%.1f, "
              "s_max:%.1f,s_min:%.1f",
              obstacle->Id(), s_upper - ego_s - half_length,
              s_lower - ego_s - half_length, max_s - ego_s - half_length,
              min_s - ego_s - half_length);
    }

    // obstacle not ignored
    static PolygonWithT cur_polygon_0;
    double obj_head = -1;
    cur_polygon_0.second.clear();
    if (!sl_polygon_seq.EvaluateByTime(0.0, &cur_polygon_0)) {
      MSD_LOG(INFO, "DEBUG_CJ889:Obj_ID:%d,EvaluateByTime Failed at 0",
              obstacle->Id());
      if (CIPV_check_cycle == 1) {
        LOG_LON_DECISION_INFO(obstacle->Id(), "ignore",
                              "ignore because polygon get failed");
      }
      continue;
    }
    obj_head = cur_polygon_0.second.max_x();
    MSD_LOG(INFO,
            "DEBUG_CJ890:Obj_ID:%d, "
            "obj_s:%.1f,ego_s:%.1f,cls:%d,oh:%d,crs:%d,cut:%d,cutcnt:%d",
            obstacle->Id(), obj_head, ego_s + half_length,
            is_obs_collide_with_path, on_heading, crossing, cut_in_flag,
            lon_decision_info.cut_in_count);
    if (is_obs_collide_with_path == true) {
      lon_decision_info.collision = true;
      if (collision_time < 0) {
        collision_time = relative_time;
      } else {
        collision_time = std::fmin(collision_time, relative_time);
      }
      lon_decision_info.collision_time = collision_time;
    }
    if (obj_head >= ego_s + half_length + 0.5 && !crossing &&
        (is_obs_collide_with_path || lon_decision_info.cut_in_count >= 3)) {
      is_follow = true;
    }
  }

  // CJ: sustain follow decision for 3 cycles
  lon_decision_info.is_follow = is_follow;
  if (is_follow == true) {
    lon_decision_info.follow_time_count++;
    lon_decision_info.ignore_time_count = 0;
  } else {
    lon_decision_info.ignore_time_count++;
    if (lon_decision_info.ignore_time_count >= 3) {
      lon_decision_info.follow_time_count = 0;
    }
  }
  // CJ:in LC process keep origin lane follow decision for some time
  const auto &lc_status = context_->lateral_behavior_planner_output().lc_status;
  bool is_lane_change =
      lc_status == "left_lane_change" || lc_status == "right_lane_change";
  auto tmp_leadone = world_model_->mutable_lateral_obstacle().tleadone();
  auto &enu2car = world_model_->get_cart_ego_state_manager().get_enu2car();
  auto &car2enu = world_model_->get_cart_ego_state_manager().get_car2enu();
  Eigen::Vector3d car_point, enu_point;
  MSD_LOG(INFO, "LON_LC:ID:%d ===============", obstacle->Id());
  MSD_LOG(INFO, "LON_LC:ID:%d Time:%.2f", obstacle->Id(), mtime);
  MSD_LOG(INFO, "LON_LC:ID:%d obj_r:%.2f", obstacle->Id(),
          obstacle->R_frenet());
  MSD_LOG(INFO, "LON_LC:ID:%d lwidth:%.2f,vwidth:%.2f", obstacle->Id(),
          lane_width, half_width * 2);
  if (is_lane_change) {
    if (tmp_leadone != nullptr && obstacle->Id() == tmp_leadone->track_id &&
        obstacle->Type() != ObjectType::CONE_BUCKET) {
      double in_lane_offset;
      if (lc_status == "left_lane_change") {
        in_lane_offset = obstacle->R_frenet() + lane_width;
      } else {
        in_lane_offset = obstacle->R_frenet() - lane_width;
      }
      if (lon_decision_info.lc_follow_id <= 0 &&
          std::fabs(in_lane_offset) < lane_width / 2 - half_width - 0.1) {
        lon_decision_info.lc_cipv_in_lane = true;
      }
      lon_decision_info.lc_follow_id = tmp_leadone->track_id;
    }
    if (lon_decision_info.lc_follow_id == obstacle->Id()) {
      lon_decision_info.lc_follow_count++;
    } else {
      lon_decision_info.lc_follow_count = 0;
    }
    double min_y = 100;
    for (auto carte_point : polygon_0.points()) {
      enu_point.x() = carte_point.x();
      enu_point.y() = carte_point.y();
      enu_point.z() = 0;
      car_point = enu2car * enu_point;
      if (std::fabs(car_point.y()) < min_y) {
        min_y = std::fabs(car_point.y());
      }
    }
    MSD_LOG(INFO, "LON_LC:ID:%d min_y:%.2f", obstacle->Id(), min_y);
    if (lon_decision_info.lc_follow_count > 0) {
      if (lon_decision_info.lc_follow_count <= 20) {
        ObjectDecisionType follow_decision;
        auto mutable_follow_decision = follow_decision.mutable_follow();
        obstacle_decision_manager.add_longitudinal_decision(
            "st_graph_generator", obstacle->Id(), follow_decision);
        LOG_LON_DECISION_INFO(obstacle->Id(), "follow", "");
        MSD_LOG(INFO, "LON_LC:ID:%d LC CIPV KEEP", obstacle->Id());
        lon_decision_info.is_follow = true;
        lon_decision_info.ignore_time_count = 0;
      } else if (min_y > half_width + 0.3 &&
                 lon_decision_info.lc_cipv_in_lane) {
        auto ptr_obstacle_decision =
            obstacle_decision_manager.find_obstacle_decision(obstacle->Id());
        if (ptr_obstacle_decision) {
          append_ignore_decision(ptr_obstacle_decision);
          LOG_LON_DECISION_INFO(obstacle->Id(), "ignore", "LC follow time out");
          MSD_LOG(INFO, "LON_LC:ID:%d LC CIPV IGNORE", obstacle->Id());
        }
        lon_decision_info.is_follow = false;
        lon_decision_info.follow_time_count = 0;
        lon_decision_info.ignore_time_count = 10;
      }

      // if ego is in target lane, AND is_follow == true
      if (std::fabs(ego_l) < std::fabs(lane_width / 2.0) && is_follow == true) {
        ObjectDecisionType follow_decision;
        auto mutable_follow_decision = follow_decision.mutable_follow();
        obstacle_decision_manager.add_longitudinal_decision(
            "st_graph_generator", obstacle->Id(), follow_decision);
        LOG_LON_DECISION_INFO(obstacle->Id(), "follow", "");
        MSD_LOG(INFO, "LON_LC_2:ID:%d LC CIPV reselect", obstacle->Id());
        lon_decision_info.is_follow = true;
        lon_decision_info.ignore_time_count = 0;
      }
    }
  } else {
    lon_decision_info.lc_follow_id = -1;
  }
  MSD_LOG(INFO, "LON_LC:ID:%d LC_CIPV:%d", obstacle->Id(),
          lon_decision_info.lc_follow_id);
  MSD_LOG(INFO, "LON_LC:ID:%d CIPV_InLane:%d", obstacle->Id(),
          lon_decision_info.lc_cipv_in_lane);
  MSD_LOG(INFO, "LON_LC:ID:%d LC_Cnt:%d", obstacle->Id(),
          lon_decision_info.lc_follow_count);
  MSD_LOG(INFO, "LON_LC:ID:%d is_lane_change:%d", obstacle->Id(),
          is_lane_change);
  MSD_LOG(INFO, "LON_LC:ID:%d tmp_leadone:%d", obstacle->Id(),
          tmp_leadone != nullptr);
  //
  MSD_LOG(INFO, "DEBUG_CJ889 Follow:%d,cnt:%d, ID:%d", is_follow,
          lon_decision_info.follow_time_count, obstacle->Id());

  const auto &ddp_traj = ddp::DdpContext::Instance()
                             ->get_obstacle_decider_ddp_trajectory()
                             .trajectory;

  const double K_cone_bucket_ignore_dist_thr = 80.0;
  const bool is_cone_bucket = obstacle->Type() == ObjectType::CONE_BUCKET;
  const double front_edge_to_center = ConfigurationContext::Instance()
                                          ->get_vehicle_param()
                                          .front_edge_to_center;
  const double back_edge_to_center =
      ConfigurationContext::Instance()->get_vehicle_param().back_edge_to_center;
  const double back_axle_to_geometry_center =
      0.5 * (front_edge_to_center - back_edge_to_center);
  const double obs_rel_s =
      perception_sl_boundary.start_s - ego_s + back_axle_to_geometry_center;

  if (lon_decision_info.ignore_time_count < 3) {
    if (!world_model_->use_eftp() &&
        !(is_cone_bucket && obs_rel_s > K_cone_bucket_ignore_dist_thr)) {
      ObjectDecisionType follow_decision;
      auto mutable_follow_decision = follow_decision.mutable_follow();
      (void)obstacle_decision_manager.add_longitudinal_decision(
          "st_graph_generator", obstacle->Id(), follow_decision);
      LOG_LON_DECISION_INFO(obstacle->Id(), "follow", "rule base follow");
      MSD_LOG(INFO, "DEBUG_CJ889 Follow ID:%d, Time:%.2f", obstacle->Id(),
              mtime);
    }
    // LOG_LON_DECISION_INFO(obstacle->Id(), "follow", "rule base follow");
    // MSD_LOG(INFO, "DEBUG_CJ889 Follow ID:%d, Time:%.2f", obstacle->Id(),
    // mtime);
  }

  // CJ: decide merge attribute
  auto right_refline = world_model_->get_map_info().right_refline_points();
  auto obj_decision =
      obstacle_decision_manager.find_obstacle_decision(obstacle->Id());
  bool is_merge_flag = false;
  const double K_NormalWidthBias_m = 1.0;
  const double K_NormalWidthMax_m = 5.0;
  const int K_odd_lane_cnt_thr = 5;
  if (obj_decision->LongitudinalDecision().has_follow()) {
    if (obstacle->R_frenet() - ego_l < -lane_width) {
      double right_lane_width = -1.0;
      int odd_lane_cnt = 0;
      if (right_refline.size() > 2) {
        right_lane_width = right_refline[0].distance_to_left_lane_border +
                           right_refline[0].distance_to_right_lane_border;
        for (int i = 1; i < right_refline.size(); ++i) {
          double cur_width = right_refline[i].distance_to_left_lane_border +
                             right_refline[i].distance_to_right_lane_border;
          if (std::fabs(cur_width - right_lane_width) > K_NormalWidthBias_m ||
              cur_width > K_NormalWidthMax_m) {
            ++odd_lane_cnt;
          }
          if (odd_lane_cnt > K_odd_lane_cnt_thr) {
            break;
          }
        }
        if (odd_lane_cnt > K_odd_lane_cnt_thr) {
          is_merge_flag = true;
        }
      } else {
        is_merge_flag = true;
      }
    }
  }
  MSD_LOG(INFO, "DEBUG_CJ_909: ID:%d,=========================================",
          obstacle->Id());
  MSD_LOG(INFO, "DEBUG_CJ_909: ID:%d, size:%d", obstacle->Id(),
          right_refline.size());
  for (int i = 0; i < right_refline.size(); i += 5) {
    MSD_LOG(INFO, "DEBUG_CJ_909: ID:%d, x:%.1f,d2lb:%.1f, d2rb:%.1f, wid:%.1f",
            obstacle->Id(), right_refline[i].car_point.x,
            right_refline[i].distance_to_left_lane_border,
            right_refline[i].distance_to_right_lane_border,
            right_refline[i].lane_width);
  }
  MSD_LOG(INFO, "DEBUG_CJ_909: ID:%d,*****************************************",
          obstacle->Id());
  MSD_LOG(INFO, "DEBUG_CJ_909: ID:%d, size:%d", obstacle->Id(),
          ref_line.size());
  for (int i = 0; i < ref_line.size(); i += 5) {
    MSD_LOG(INFO, "DEBUG_CJ_909: ID:%d, x:%.1f,d2lb:%.1f, d2rb:%.1f, wid:%.1f",
            obstacle->Id(), ref_line[i].car_point.x,
            ref_line[i].distance_to_left_lane_border,
            ref_line[i].distance_to_right_lane_border, ref_line[i].lane_width);
  }
  MSD_LOG(INFO, "DEBUG_CJ_909: ID:%d, Merge:%d", obstacle->Id(), is_merge_flag);
  // CJ: fill info in log struct
  lon_decision_info.rule_base_cutin = rule_base_cutin;
  lon_decision_info.ftp_cutin = obstacle->IsCutin();
  lon_decision_info.lat_cutin = obstacle->IsDistinguishCutin();
  lon_decision_info.is_merge_flag = is_merge_flag;
  lon_decision_info.overlap = overlap;
  lon_decision_info.type = (int)obstacle->Type();
  MSD_LOG(INFO, "DEBUG_CJ_911:------------------------------------------");
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,Time:%.2f", obstacle->Id(), mtime);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,is_follow:        %d", obstacle->Id(),
          lon_decision_info.is_follow);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,exist:            %d", obstacle->Id(),
          lon_decision_info.exist);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,cut_in_flag:      %d", obstacle->Id(),
          lon_decision_info.cut_in_flag);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,cut_in_count:     %d", obstacle->Id(),
          lon_decision_info.cut_in_count);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,crossing:         %d", obstacle->Id(),
          lon_decision_info.crossing);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,on_heading:       %d", obstacle->Id(),
          lon_decision_info.on_heading);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,in_lane:          %d", obstacle->Id(),
          lon_decision_info.in_lane);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,collision:        %d", obstacle->Id(),
          lon_decision_info.collision);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,collision_time:   %f", obstacle->Id(),
          lon_decision_info.collision_time);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,lat_dist_head:    %.2f", obstacle->Id(),
          lon_decision_info.lat_dist_head);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,obj_vy_dy:        %.2f", obstacle->Id(),
          lon_decision_info.obj_vy_dy);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,follow_time_count:%d", obstacle->Id(),
          lon_decision_info.follow_time_count);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,ignore_time_count:%d", obstacle->Id(),
          lon_decision_info.ignore_time_count);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,rule_base_cutin:  %d", obstacle->Id(),
          lon_decision_info.rule_base_cutin);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,ftp_cutin:        %d", obstacle->Id(),
          lon_decision_info.ftp_cutin);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,is_merge_flag:    %d", obstacle->Id(),
          lon_decision_info.is_merge_flag);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,overlap:          %.2f", obstacle->Id(),
          lon_decision_info.overlap);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,type:             %d", obstacle->Id(),
          lon_decision_info.type);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,virtual_overlap:  %.2f", obstacle->Id(),
          lon_decision_info.virtual_overlap);
  // longitudinal debug info
  // fill obj_mem_info_map
  obj_mem_info_map[obstacle->Id()].lat_dist_head =
      lon_decision_info.lat_dist_head;
  obj_mem_info_map[obstacle->Id()].obj_vy_dy = lon_decision_info.obj_vy_dy;
  obj_mem_info_map[obstacle->Id()].collision = lon_decision_info.collision;
  obj_mem_info_map[obstacle->Id()].collision_time =
      lon_decision_info.collision_time;
  obj_mem_info_map[obstacle->Id()].in_lane = lon_decision_info.in_lane;
  obj_mem_info_map[obstacle->Id()].follow_k1 = lon_decision_info.is_follow;
  obj_mem_info_map[obstacle->Id()].follow_count =
      lon_decision_info.follow_time_count;
  obj_mem_info_map[obstacle->Id()].cut_in_k1 = lon_decision_info.cut_in_flag;
  obj_mem_info_map[obstacle->Id()].cut_in_count =
      lon_decision_info.cut_in_count;
  obj_mem_info_map[obstacle->Id()].rule_base_cutin = rule_base_cutin;
  obj_mem_info_map[obstacle->Id()].ftp_cutin = obstacle->IsCutin();
  obj_mem_info_map[obstacle->Id()].lat_cutin = obstacle->IsDistinguishCutin();
  obj_mem_info_map[obstacle->Id()].is_merge_flag = is_merge_flag;
  obj_mem_info_map[obstacle->Id()].overlap = overlap;
  obj_mem_info_map[obstacle->Id()].type = (int)obstacle->Type();
  obj_mem_info_map[obstacle->Id()].virtual_overlap = virtual_overlap;
  MSD_LOG(INFO, "DEBUG_CJ_911:----------------------");
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,lat_dist_head:%.2f", obstacle->Id(),
          obj_mem_info_map[obstacle->Id()].lat_dist_head);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,obj_vy_dy:    %.2f", obstacle->Id(),
          obj_mem_info_map[obstacle->Id()].obj_vy_dy);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,exist:        %d", obstacle->Id(),
          obj_mem_info_map[obstacle->Id()].exist);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,collision:    %d", obstacle->Id(),
          obj_mem_info_map[obstacle->Id()].collision);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,collision_t:  %f", obstacle->Id(),
          obj_mem_info_map[obstacle->Id()].collision_time);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,in_lane:      %d", obstacle->Id(),
          obj_mem_info_map[obstacle->Id()].in_lane);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,follow_k1:    %d", obstacle->Id(),
          obj_mem_info_map[obstacle->Id()].follow_k1);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,follow_count: %d", obstacle->Id(),
          obj_mem_info_map[obstacle->Id()].follow_count);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,cut_in_k1:    %d", obstacle->Id(),
          obj_mem_info_map[obstacle->Id()].cut_in_k1);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,cut_in_count: %d", obstacle->Id(),
          obj_mem_info_map[obstacle->Id()].cut_in_count);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,rule_cutin:   %d", obstacle->Id(),
          obj_mem_info_map[obstacle->Id()].rule_base_cutin);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,FTP_cutin:    %d", obstacle->Id(),
          obj_mem_info_map[obstacle->Id()].ftp_cutin);
  MSD_LOG(INFO, "DEBUG_CJ_911: ID:%d,overlap:      %.2f", obstacle->Id(),
          obj_mem_info_map[obstacle->Id()].overlap);

  // fill objects info for plot
  auto *planner_debug = context_->mutable_planner_debug();
  Point2D frenet_p, carte_p;
  PointXY box_point{0.0, 0.0};
  ObjectBox obj_box;
  LonObject debug_obj;
  TrajectoryPoint traj_point;

  auto &object_list = planner_debug->lon_debug_info.object_list;
  for (int i = 0; i < CIPV_check_cycle; ++i) {
    double relative_time = i * FLAGS_speed_lon_decision_time_resolution;
    traj_point = obstacle->GetPointAtTime(relative_time);
    auto obs_polygon = obstacle->GetPolygonAtPoint(traj_point);
    obj_box.box.clear();
    Point2D obj_frenet_p{0.0, 0.0}, obj_carte_p{0.0, 0.0};
    for (auto carte_point : obs_polygon.points()) {
      enu_point.x() = carte_point.x();
      enu_point.y() = carte_point.y();
      enu_point.z() = 0.0;
      car_point = enu2car * enu_point;
      box_point.x = car_point.x();
      box_point.y = car_point.y();
      obj_box.box.push_back(box_point);
      obj_carte_p.x = carte_point.x();
      obj_carte_p.y = carte_point.y();
      if (frenet_coord->CartCoord2FrenetCoord(obj_carte_p, obj_frenet_p) ==
          TRANSFORM_SUCCESS) {
        MSD_LOG(INFO, "ST_DEBUG3:ID:%d,i:%d,fx:%.2f,fy:%.2f", obstacle->Id(), i,
                obj_frenet_p.x, obj_frenet_p.y);
      }
    }
    debug_obj.BoxList.push_back(obj_box);
  }
  debug_obj.id = obstacle->Id();
  debug_obj.CrossLine = obstacle->IsEgoLaneOverlap();
  debug_obj.type = int(obstacle->Type());
  object_list[obstacle->Id()] = debug_obj;
}

void StGraphGenerator::update_decision_info() {
  const auto &obstacle_manager = baseline_info_->obstacle_manager();
  auto &obstacle_decision_manager =
      context_->mutable_obstacle_decision_manager();
  auto &lon_obstacle_decision_info_map =
      context_->mutable_lon_decison_output()->obstacle_decision_info_map;

  std::vector<int> erase_obstacle_id{};
  for (const auto &lon_decision : lon_obstacle_decision_info_map) {
    if (lon_decision.second.exist == false) {
      erase_obstacle_id.push_back(lon_decision.first);
    }
  }
  for (const auto &id : erase_obstacle_id) {
    lon_obstacle_decision_info_map.erase(id);
  }
}

void StGraphGenerator::append_ignore_decision(
    ObstacleDecision *obstacle_decision) const {
  ObjectDecisionType ignore_decision;
  ignore_decision.mutable_ignore();
  if (!obstacle_decision->HasLongitudinalDecision()) {
    obstacle_decision->ReplaceLongitudinalDecision("st_graph_generator",
                                                   ignore_decision);
  }
  if (!obstacle_decision->HasLateralDecision()) {
    obstacle_decision->AddLateralDecision("st_graph_generator",
                                          ignore_decision);
  }
}

void StGraphGenerator::translate_scenario_decisions() {
  auto &obstacle_manager = baseline_info_->mutable_obstacle_manager();
  auto &obstacle_decision_manager =
      context_->mutable_obstacle_decision_manager();
  auto adc_sl = baseline_info_->get_adc_sl_boundary();
  const double ego_s = baseline_info_->get_ego_state().ego_frenet.x;
  const double ego_vel = baseline_info_->get_ego_state().ego_vel;
  const auto &ego_sl = baseline_info_->get_adc_sl_boundary();

  if (is_in_lane_crossing_stage_) {
    const auto &lane_status = context_->planning_status().lane_status;
    std::string moving_direction;
    if (lane_status.status == LaneStatus::LANE_CHANGE) {
      moving_direction = lane_status.change_lane.direction;
    } else {
      moving_direction = "none";
    }

    // find rear car in target lane
    if (moving_direction != "none") {
      for (const auto &obstacle : obstacle_manager.get_obstacles().Items()) {
        auto obstacle_sl = obstacle->PerceptionSLBoundary();
        constexpr double kOvertakeLateralBuffer = 0.7;
        if (obstacle_sl.end_s < adc_sl.start_s) {
          if ((moving_direction == "left" &&
               obstacle_sl.start_l + kOvertakeLateralBuffer > ego_sl.end_l &&
               obstacle_sl.start_l < left_width_) ||
              (moving_direction == "right" &&
               obstacle_sl.end_l > -right_width_ &&
               obstacle_sl.end_l - kOvertakeLateralBuffer < ego_sl.start_l)) {
            ObjectDecisionType overtake_decision;
            auto mutable_overtake_decision =
                overtake_decision.mutable_overtake();
            mutable_overtake_decision->start_time = 0.0;
            mutable_overtake_decision->time_buffer =
                time_range_.second - time_range_.first;
            (void)obstacle_decision_manager.add_longitudinal_decision(
                "st_graph_generator", obstacle->Id(), overtake_decision);
            LOG_LON_DECISION_INFO(obstacle->Id(), "overtake",
                                  "rear car in target lane");
          } else if ((moving_direction == "left" &&
                      obstacle_sl.start_l < left_width_) ||
                     (moving_direction == "right" &&
                      obstacle_sl.end_l > -right_width_)) {
            auto ptr_obstacle_decision =
                obstacle_decision_manager.find_obstacle_decision(
                    obstacle->Id());
            if (ptr_obstacle_decision) {
              append_ignore_decision(ptr_obstacle_decision);
              LOG_LON_DECISION_INFO(obstacle->Id(), "ignore",
                                    "rear car, non-intervention object");
              MSD_LOG(INFO, "IGNORE:P1:ID:%d", obstacle->Id());
            }
          }
        }
      }
    }
  }

  for (const auto &obstacle : obstacle_manager.get_obstacles().Items()) {
    auto ptr_obstacle_decision =
        obstacle_decision_manager.find_obstacle_decision(obstacle->Id());
    const auto &boundary = obstacle->path_st_boundary();
    if (boundary.IsEmpty() || boundary.max_s() < 0.0 ||
        boundary.max_t() < 0.0 || boundary.min_t() >= time_range_.second) {
      // append_ignore_decision(ptr_obstacle_decision);
      // LOG_LON_DECISION_INFO(obstacle->Id(), "ignore",
      //                       "st boundart is over limit");
      MSD_LOG(INFO, "IGNORE:P2:ID:%d", obstacle->Id());
      continue;
    }
    if (ptr_obstacle_decision->HasLongitudinalDecision()) {
      // append_ignore_decision(ptr_obstacle_decision);
      continue;
    }
    // for Virtual obstacle, skip if center point NOT "on lane"
    if (obstacle->IsVirtual()) {
      const auto &obstacle_sl = obstacle->PerceptionSLBoundary();
      if (obstacle_sl.start_l > left_width_ ||
          obstacle_sl.end_l < -right_width_) {
        continue;
      }
    }
    if (check_stop_for_pedestrian(*obstacle)) {
      MSD_LOG(INFO, "debug stop for pedestrian[%d]", obstacle->Id());
      ObjectDecisionType follow_decision;
      auto mutable_follow_decision = follow_decision.mutable_follow();
      mutable_follow_decision->start_time = 0.0;
      mutable_follow_decision->time_buffer =
          time_range_.second - time_range_.first;
      (void)obstacle_decision_manager.add_longitudinal_decision(
          "st_graph_generator", obstacle->Id(), follow_decision);
      LOG_LON_DECISION_INFO(obstacle->Id(), "follow", "follow for pedestrian");
      // clear lateral decision for pedestrian
      // ptr_obstacle_decision->ClearLateralDecision();
      if (ptr_obstacle_decision->MutableLateralDecision().has_nudge()) {
        ptr_obstacle_decision->MutableLateralDecision()
            .mutable_nudge()
            ->is_longitunidal_ignored = false;
      }
    }
  }

  // ignore obstacles with unachievable cut in intention
  constexpr double kCutinTimeBuffer = 0.5;
  double dis_to_merge = world_model_->get_map_info().distance_to_next_merge();
  bool is_in_intersection = world_model_->get_map_info().is_in_intersection();
  bool have_right_of_way = true;
  if (dis_to_merge < 30.0 || is_in_intersection || is_in_lane_crossing_stage_) {
    have_right_of_way = false;
  }
  MSD_LOG(
      INFO,
      "close_to_merge %d is_in_intersection %d is_in_lane_crossing_stage_ %d",
      dis_to_merge < 30.0, is_in_intersection, is_in_lane_crossing_stage_);
  if (have_right_of_way) {
    constexpr double kMaxYieldAcc = -3.0;
    constexpr double kCheckTimeStep = 0.2;
    auto brake_speed_data = SpeedProfileGenerator::GenerateStopProfile(
        ego_vel, baseline_info_->get_ego_state().ego_acc, kMaxYieldAcc);

    for (const auto &obstacle : obstacle_manager.get_obstacles().Items()) {
      auto ptr_obstacle_decision =
          obstacle_decision_manager.find_obstacle_decision(obstacle->Id());
      const auto &st_boundary = obstacle->path_st_boundary();
      if (st_boundary.IsEmpty()) {
        continue;
      }
      if (ptr_obstacle_decision->HasLongitudinalDecision()) {
        // append_ignore_decision(ptr_obstacle_decision);
        continue;
      }
      bool is_cutin_from_behind = false;
      if (obstacle->PerceptionSLBoundary().end_s < ego_s) {
        double cut_in_timestamp;
        if (!st_boundary.GetTimestampBeginWithS(
                ego_s + ConfigurationContext::Instance()
                                ->get_vehicle_param()
                                .length /
                            2.0,
                cut_in_timestamp)) {
          cut_in_timestamp = std::numeric_limits<double>::max();
        }
        if (cut_in_timestamp >= kCutinTimeBuffer &&
            st_boundary.max_s() > ego_s - ConfigurationContext::Instance()
                                                  ->get_vehicle_param()
                                                  .length /
                                              2.0) {
          is_cutin_from_behind = true;
          MSD_LOG(INFO, "debug obstacle[%d] cut in from behind",
                  obstacle->Id());
        }
      }

      bool is_unavoidable_with_obs = false;
      double s_upper, s_lower;

      for (double t = 0.0; t < time_range_.second; t += kCheckTimeStep) {
        SpeedPoint sp;
        if (!brake_speed_data.EvaluateByTime(t, &sp)) {
          continue;
        }
        if (!st_boundary.GetBoundarySRange(t, &s_upper, &s_lower)) {
          continue;
        }
        double front_s =
            sp.s +
            ConfigurationContext::Instance()->get_vehicle_param().length / 2.0 +
            ego_s;
        if (front_s >= s_lower && front_s <= s_upper) {
          MSD_LOG(INFO, "debug collide with obstacle[%d] in time[%f]",
                  obstacle->Id(), t);
          is_unavoidable_with_obs = true;
          break;
        }
      }
      if (is_unavoidable_with_obs && is_cutin_from_behind) {
        MSD_LOG(INFO, "debug ignore obstacle[%d] for unachievable cut in.",
                obstacle->Id());
        // append_ignore_decision(ptr_obstacle_decision);
        // LOG_LON_DECISION_INFO(obstacle->Id(), "ignore",
        //                       "is_unavoidable_with_obs & cutin_from_behind");
        MSD_LOG(INFO, "IGNORE:P3:ID:%d", obstacle->Id());
      }
    }
  }
}

bool StGraphGenerator::check_stop_for_pedestrian(const Obstacle &obstacle) {
  if (obstacle.Type() != ObjectType::PEDESTRIAN &&
      obstacle.Type() != ObjectType::OFO) {
    return false;
  }

  const auto &last_lon_follow_obs =
      context_->planning_status().planning_result.lon_follow_obstacles;
  if (std::find(last_lon_follow_obs.begin(), last_lon_follow_obs.end(),
                obstacle.Id()) == last_lon_follow_obs.end()) {
    return false;
  }
  const auto &obstacle_sl_boundary = obstacle.PerceptionSLBoundary();
  const auto &adc_sl_boundary = baseline_info_->get_adc_sl_boundary();
  const double ego_s = baseline_info_->get_ego_state().ego_frenet.x;
  if (obstacle_sl_boundary.start_s < adc_sl_boundary.end_s) {
    return false;
  }
  auto &pedestrian_stop_timer =
      context_->mutable_planning_status()->crosswalk.stop_times;
  // update stop timestamp on static pedestrian for watch timer
  // check on stop timer for static pedestrians
  static constexpr double kSDistanceStartTimer = 10.0;
  static constexpr double kSDistanceStartTimerForIntersection = 15.0;
  static constexpr double kMaxStopSpeed = 1.0;
  double dis_to_intersect = world_model_->get_map_info().dist_to_intsect();
  bool is_crosswalk_pedestrian =
      dis_to_intersect > 0.0 &&
      obstacle.path_st_boundary().min_s() - adc_sl_boundary.end_s >
          dis_to_intersect &&
      obstacle.path_st_boundary().min_s() - adc_sl_boundary.end_s <
          kSDistanceStartTimerForIntersection;
  MSD_LOG(INFO,
          "pedestrian speed %f min s %f ego_s %f dis_to_intersect %f "
          "is_crosswalk_pedestrian %d",
          obstacle.speed(), obstacle.path_st_boundary().min_s(), ego_s,
          dis_to_intersect, is_crosswalk_pedestrian);
  if (obstacle.path_st_boundary().min_s() - adc_sl_boundary.end_s <
          kSDistanceStartTimer ||
      is_crosswalk_pedestrian) {
    constexpr double kMinPedestrianStopTimeout = 2.0;
    double max_stop_time = is_crosswalk_pedestrian ? kPedestrianStopTimeout
                                                   : kMinPedestrianStopTimeout;
    if (obstacle.speed() > kMaxStopSpeed) {
      pedestrian_stop_timer.erase(obstacle.Id());
    } else {
      double current_time_seconds = MTIME()->timestamp().sec();
      if (pedestrian_stop_timer.find(obstacle.Id()) ==
          pedestrian_stop_timer.end()) {
        // add timestamp
        pedestrian_stop_timer[obstacle.Id()] = current_time_seconds;
        MSD_LOG(INFO, "pedestrian add timestamp: obstacle_id[%d] timestamp[%f]",
                obstacle.Id(), current_time_seconds);
        return true;
      } else {
        // check timeout
        double stop_timer =
            current_time_seconds - pedestrian_stop_timer[obstacle.Id()];
        MSD_LOG(INFO, "pedestrian stop_timer: obstacle_id[%d] stop_timer[%f]",
                obstacle.Id(), stop_timer);
        if (stop_timer >= max_stop_time) {
          pedestrian_stop_timer.erase(obstacle.Id());
          return false;
        }
        return true;
      }
    }
  }

  return false;
}

void StGraphGenerator::generate_shifted_reference_line(
    PathData &reference_line,
    std::shared_ptr<FrenetCoordinateSystem> frenet_coord, double left_shift,
    double right_shift) {
  reference_line.Clear();
  static std::vector<PathPoint> path_points;
  path_points.clear();
  constexpr double min_path_len = 20.0;         // m
  constexpr double max_path_len = 100.0;        // m
  constexpr double min_look_forward_time = 2.0; // s
  auto ego_state = baseline_info_->get_ego_state();
  double path_len =
      ego_state.ego_vel * min_look_forward_time +
      0.5 * ego_state.ego_acc * std::pow(min_look_forward_time, 2);
  path_len = std::min(max_path_len, std::max(min_path_len, path_len));
  constexpr double KDenseSampleUnit = 0.5;
  constexpr double KSparseSmapleUnit = 2.0;
  double reference_line_len = frenet_coord->GetLength();
  double start_plan_point_project_s =
      ego_state.planning_init_point.path_point.s;
  double lane_shift = (left_shift + right_shift) / 2.0;
  double lane_width = abs(left_shift - right_shift);
  MSD_LOG(INFO, "DEBUG_CJ6 lane_shift:%3.2f", lane_shift);

  for (double s = start_plan_point_project_s; s < reference_line_len;
       s += ((s < path_len) ? KDenseSampleUnit : KSparseSmapleUnit)) {
    static PathPoint path_point;
    auto pt = frenet_coord->GetRefCurvePoint(s);
    double theta = frenet_coord->GetRefCurveHeading(s);
    static Point2D fre_point, car_point;
    fre_point.x = s;
    fre_point.y = lane_shift;
    (void)frenet_coord->FrenetCoord2CartCoord(fre_point, car_point);
    path_point.x = car_point.x;
    path_point.y = car_point.y;
    path_point.z = 0.0;
    path_point.theta = theta;
    path_point.x_derivative = 0.0;
    path_point.y_derivative = 0.0;
    path_point.kappa = frenet_coord->GetRefCurveCurvature(s);
    path_point.dkappa = frenet_coord->GetRefCurveDCurvature(s);
    path_point.ddkappa = 0.0;
    path_point.s = s;
    path_points.emplace_back(path_point);
  }
  (void)set_reference_line_border();
  static DiscretizedPath path;
  path.resize(path_points.size());
  for (int i = 0; i < path_points.size(); i++) {
    path[i] = path_points[i];
  }
  static PolygonalLine left_border_line;
  left_border_line.resize(left_border_.size());
  for (int i = 0; i < left_border_.size(); i++) {
    left_border_line[i] = left_border_[i];
  }
  static PolygonalLine right_border_line;
  right_border_line.resize(right_border_.size());
  for (int i = 0; i < right_border_.size(); i++) {
    right_border_line[i] = right_border_[i];
  }
  (void)reference_line.SetDiscretizedPath(path);
  (void)reference_line.SetPathBorder(left_border_line, right_border_line);
}

void StGraphGenerator::preliminary_screen_obstacles() {
  MLOG_PROFILING("preliminary_screen_obstacles");
  ObjectDecisionType ignore_decision;
  ignore_decision.mutable_ignore();
  auto &obstacle_manager = baseline_info_->mutable_obstacle_manager();
  auto &obstacle_decision_manager =
      context_->mutable_obstacle_decision_manager();
  const auto &frenet_coord = baseline_info_->get_frenet_coord();
  const auto &map_info = world_model_->get_map_info();
  const auto &ego_pose = baseline_info_->get_ego_state().ego_pose;
  constexpr double kIgnoreLateralBuffer = 0.5;
  const double ego_s = baseline_info_->get_ego_state().ego_frenet.x;
  const double ego_r = baseline_info_->get_ego_state().ego_frenet.y;
  const double ego_vel = baseline_info_->get_ego_state().ego_vel;
  const double dis_limit =
      std::max(CONSIDER_RANGE(), std::min(FLAGS_speed_lon_decision_horizon,
                                          ego_vel * ego_vel / 3.0));

  const double K_cone_bucket_ignore_dist_thr = 80.0;
  const double front_edge_to_center = ConfigurationContext::Instance()
                                          ->get_vehicle_param()
                                          .front_edge_to_center;
  const double back_edge_to_center =
      ConfigurationContext::Instance()->get_vehicle_param().back_edge_to_center;
  const double back_axle_to_geometry_center =
      0.5 * (front_edge_to_center - back_edge_to_center);

  int lc_pause_id = context_->lateral_behavior_planner_output().lc_pause_id;

  auto &pedestrian_stop_timer =
      context_->mutable_planning_status()->crosswalk.stop_times;
  static std::vector<int> erased_pedestrians;
  erased_pedestrians.clear();
  // clear untracked pedestrian
  for (auto &pedestrian : pedestrian_stop_timer) {
    auto tracked_ped = obstacle_manager.find_obstacle(pedestrian.first);
    if (tracked_ped == nullptr) {
      erased_pedestrians.push_back(pedestrian.first);
    } else if (tracked_ped->Type() != ObjectType::PEDESTRIAN &&
               tracked_ped->Type() != ObjectType::OFO) {
      erased_pedestrians.push_back(pedestrian.first);
    }
  }
  for (auto ped : erased_pedestrians) {
    pedestrian_stop_timer.erase(ped);
  }
  auto &crosswalk_pedestrian_stop_timer =
      context_->mutable_planning_status()->crosswalk.stopline_times;
  erased_pedestrians.clear();
  for (auto &pedestrian : crosswalk_pedestrian_stop_timer) {
    auto tracked_ped = obstacle_manager.find_obstacle(pedestrian.first);
    if (tracked_ped == nullptr) {
      erased_pedestrians.push_back(pedestrian.first);
    } else if (tracked_ped->Type() != ObjectType::PEDESTRIAN &&
               tracked_ped->Type() != ObjectType::OFO) {
      erased_pedestrians.push_back(pedestrian.first);
    }
  }
  for (auto ped : erased_pedestrians) {
    crosswalk_pedestrian_stop_timer.erase(ped);
  }

  for (const auto &obstacle :
       baseline_info_->obstacle_manager().get_obstacles().Items()) {
    auto ptr_obstacle_decision =
        obstacle_decision_manager.find_obstacle_decision(obstacle->Id());

    bool is_pause_obs = obstacle->Id() == lc_pause_id;

    if (obstacle->IsFrenetInvalid()) {
      ptr_obstacle_decision->ReplaceLongitudinalDecision("st_graph_generator",
                                                         ignore_decision);
      // obstacle_decision_manager.add_longitudinal_decision(
      //     "st_graph_generator", obstacle->Id(), ignore_decision);
      LOG_LON_DECISION_INFO(obstacle->Id(), "ignore", "because frenet invalid");
      (void)obstacle_decision_manager.add_lateral_decision(
          "st_graph_generator", obstacle->Id(), ignore_decision);
      continue;
    }
    static SLBoundary perception_sl_boundary;
    perception_sl_boundary = obstacle->PerceptionSLBoundary();
    auto perception_point = obstacle->PerceptionBoundingBox().center();
    double distance_to_adc = std::hypot(ego_pose.x - perception_point.x(),
                                        ego_pose.y - perception_point.y());
    double yaw_relative = std::atan2(perception_point.y() - ego_pose.y,
                                     perception_point.x() - ego_pose.x);
    const auto &perception_box = obstacle->PerceptionBoundingBox();
    const double ego_x = baseline_info_->get_ego_state().ego_carte.x;
    const double ego_y = baseline_info_->get_ego_state().ego_carte.y;
    const double ego_enu_x = baseline_info_->get_ego_state().ego_enu.position.x;
    const double ego_enu_y = baseline_info_->get_ego_state().ego_enu.position.y;

    // ignore cone_bucket away 60m
    if (obstacle->Type() == ObjectType::CONE_BUCKET &&
        perception_sl_boundary.start_s - ego_s + back_axle_to_geometry_center >
            K_cone_bucket_ignore_dist_thr) {
      ptr_obstacle_decision->ReplaceLongitudinalDecision("st_graph_generator",
                                                         ignore_decision);
      std::string cone_bucket_info =
          "rel_s: " + std::to_string(perception_sl_boundary.start_s - ego_s +
                                     back_axle_to_geometry_center);

      LOG_LON_DECISION_INFO(obstacle->Id(), "ignore", cone_bucket_info);
      // (void)obstacle_decision_manager.add_lateral_decision(
      //     "st_graph_generator", obstacle->Id(), ignore_decision);
    }

    // 1. ignore obstacles far away from ego car
    if (distance_to_adc >= dis_limit) {
      // obstacle_decision_manager.add_longitudinal_decision(
      //     "st_graph_generator", obstacle->Id(), ignore_decision);
      (void)obstacle_decision_manager.add_lateral_decision(
          "st_graph_generator", obstacle->Id(), ignore_decision);
      // continue;
    }
    double theta_refline =
        (perception_sl_boundary.start_l + perception_sl_boundary.end_l >
         2.0 * ego_r)
            ? std::atan2(perception_sl_boundary.end_l - ego_r,
                         perception_sl_boundary.end_s - ego_s)
            : std::atan2(perception_sl_boundary.start_l - ego_r,
                         perception_sl_boundary.end_s - ego_s);
    // 2. ignore obstacle driving backward in lane keep stage
    double cone_thld = (map_info.distance_to_crossing() < 10.0 &&
                        map_info.road_type() == Direction::TURN_RIGHT &&
                        perception_sl_boundary.end_l < ego_r - 1.0)
                           ? M_PI / 6.0
                           : M_PI / 3.0;
    bool isInCone =
        (obstacle->speed() >= T_V0() &&
         std::fabs(planning_math::WrapAngle(theta_refline) - M_PI) <
             cone_thld) ||
        (obstacle->speed() < T_V0() &&
         std::fabs(planning_math::WrapAngle(theta_refline) - M_PI) <
             PATH_TIME_GRAPH_OMMIT_RAD() &&
         distance_to_adc >
             ConfigurationContext::Instance()->get_vehicle_param().length);
    if ((!is_lane_interval_transformed_ &&
         baseline_info_->get_right_of_way_status() ==
             BaseLineInfo::PROTECTED) ||
        std::fabs(ego_r) < kIgnoreLateralBuffer) {
      if (isInCone && !is_pause_obs) {
        // obstacle_decision_manager.add_longitudinal_decision(
        //     "st_graph_generator", obstacle->Id(), ignore_decision);
        (void)obstacle_decision_manager.add_lateral_decision(
            "st_graph_generator", obstacle->Id(), ignore_decision);
        // continue;
      }
    }

    Point2D cart_point, fren_point;
    cart_point.x = perception_point.x();
    cart_point.y = perception_point.y();
    fren_point.x = obstacle->S_frenet();
    fren_point.y = obstacle->R_frenet();

    // 3. ignore obstacle right behind ego car at initial time point
    // bool isBehindEgo = perception_sl_boundary.end_s < ego_s &&
    //                    std::fabs(fren_point.y - ego_r) < 0.5;
    auto adc_sl_boundary = baseline_info_->get_adc_sl_boundary();
    double lateral_overlap_length = 0.0;
    if (perception_sl_boundary.end_l > adc_sl_boundary.start_l &&
        perception_sl_boundary.start_l < adc_sl_boundary.end_l) {
      lateral_overlap_length =
          std::min(perception_sl_boundary.end_l, adc_sl_boundary.end_l) -
          std::max(adc_sl_boundary.start_l, perception_sl_boundary.start_l);
    } else if (perception_sl_boundary.end_l < adc_sl_boundary.start_l) {
      lateral_overlap_length =
          perception_sl_boundary.end_l - adc_sl_boundary.start_l;
    } else if (perception_sl_boundary.start_l > adc_sl_boundary.end_l) {
      lateral_overlap_length =
          adc_sl_boundary.end_l - perception_sl_boundary.start_l;
    }
    constexpr double kMinLateralOverlapLength = 0.0;
    bool isBehindEgo = perception_sl_boundary.end_s < ego_s &&
                       lateral_overlap_length > kMinLateralOverlapLength;
    if (isBehindEgo) {
      ptr_obstacle_decision->ReplaceLongitudinalDecision("st_graph_generator",
                                                         ignore_decision);
      // obstacle_decision_manager.add_longitudinal_decision(
      //     "st_graph_generator", obstacle->Id(), ignore_decision);
      LOG_LON_DECISION_INFO(obstacle->Id(), "ignore", "ignore_behind_ego");
    }

    // ignore behind car when frenet is a circle
    if (perception_sl_boundary.start_s < ego_s &&
        perception_sl_boundary.end_s > ego_s &&
        (perception_sl_boundary.end_s - perception_sl_boundary.start_s) >
            20.0 &&
        lateral_overlap_length > kMinLateralOverlapLength) {
      ptr_obstacle_decision->ReplaceLongitudinalDecision("st_graph_generator",
                                                         ignore_decision);
      // obstacle_decision_manager.add_longitudinal_decision(
      //     "st_graph_generator", obstacle->Id(), ignore_decision);
      LOG_LON_DECISION_INFO(obstacle->Id(), "ignore",
                            "ignore behind car when frenet is a circle");
      (void)obstacle_decision_manager.add_lateral_decision(
          "st_graph_generator", obstacle->Id(), ignore_decision);
      continue;
    }

    // 4. ignore obstacles far away from ego car in lateral direction
    if (std::fabs(fren_point.y) >= LAT_CONSIDER_RANGE()) {
      ptr_obstacle_decision->ReplaceLongitudinalDecision("st_graph_generator",
                                                         ignore_decision);
      // obstacle_decision_manager.add_longitudinal_decision(
      //     "st_graph_generator", obstacle->Id(), ignore_decision);
      LOG_LON_DECISION_INFO(
          obstacle->Id(), "ignore",
          "ignore obstacles far away from ego car in lateral direction");
      (void)obstacle_decision_manager.add_lateral_decision(
          "st_graph_generator", obstacle->Id(), ignore_decision);
      continue;
    }

    // update obstacle first time state
    auto traj_point = obstacle->GetPointAtTime(0.0);
    auto traj_point_mid =
        obstacle->GetPointAtTime(FLAGS_speed_lon_decision_time_horizon / 2.0);
    double theta_ref = frenet_coord->GetRefCurveHeading(fren_point.x);
    if (isBehindEgo) {
      continue;
    }
    // update obstacle avoidance distance buffer
    double x_error = obstacle->GetPointAtTime(0.0).path_point.x - ego_pose.x;
    double y_error = obstacle->GetPointAtTime(0.0).path_point.y - ego_pose.y;
    double v_obs_lat =
        obstacle->speed() * std::sin(obstacle->Yaw_relative_frenet());
    // > 0 when lat towards ego car
    if (obstacle->R_frenet() > ego_r) {
      v_obs_lat *= -1.0;
    }
    double width_modify_init_pos =
        std::min(0.1, 2.0 / sqrt(0.1 + pow(x_error, 2.0) + pow(y_error, 2.0)));
    double width_modify_dyn_obs = std::min(
        0.3, 0.1 * std::fabs(obstacle->speed()) + 0.3 * std::fabs(v_obs_lat));
    double width_modify_obs_type =
        (obstacle->Type() == ObjectType::PEDESTRIAN ||
         obstacle->Type() == ObjectType::OFO)
            ? 0.25
            : 0.0;
    double width_modify_ego_lane_overlap = 0.0;
    if (ConfigurationContext::Instance()
            ->synthetic_config()
            .sensor_configuration != "lidar") {
      width_modify_obs_type = (obstacle->Type() == ObjectType::TRANSPORT_TRUNK)
                                  ? -0.2
                                  : width_modify_obs_type;

      if (obstacle->IsEgoLaneOverlap() &&
          (obstacle->Type() == ObjectType::TRANSPORT_TRUNK ||
           obstacle->Type() == ObjectType::BUS)) {
        width_modify_ego_lane_overlap = 0.1;
        MSD_LOG(
            INFO,
            "EgoLaneOverlap, id: %d, type: %d, lane_overlap:%d, v_obs_lat: %f",
            obstacle->Id(), obstacle->Type(), obstacle->IsEgoLaneOverlap(),
            v_obs_lat);
      }
    }
    double width_modify_truck_back_mirror = 0.0;
    if (ConfigurationContext::Instance()
            ->synthetic_config()
            .sensor_configuration != "lidar") {
      if (obstacle->Type() == ObjectType::TRANSPORT_TRUNK) {
        width_modify_truck_back_mirror = -0.2;
      }
    }
    double width_modify_sensor = config_.lane_width_buff_sensor;
    double width_modify = width_modify_init_pos + width_modify_dyn_obs +
                          width_modify_obs_type +
                          width_modify_ego_lane_overlap - width_modify_sensor;
    width_modify += is_in_lane_crossing_stage_ ? 0.1 : 0.0;
    width_modify = std::max(width_modify, -0.1);
    Obstacle *ptr_obstacle = obstacle_manager.find_obstacle(obstacle->Id());
    width_modify = width_modify_ego_lane_overlap + width_modify_obs_type;
    ptr_obstacle->SetAvdDisBUffer(width_modify);
  }
}

void StGraphGenerator::compute_sl_polygon_sequence(
    ObstacleManager &obstacle_manager, ScenarioFacadeContext *context,
    std::shared_ptr<BaseLineInfo> baseline_info,
    StGraphData::DoublePair time_range, double time_gap) {
  MLOG_PROFILING("compute_sl_polygon_sequence");
  // auto &obstacle_manager = world_model_->mutable_obstacle_manager();
  auto frenet_coord = baseline_info->get_frenet_coord();
  auto ego_cart_pose = baseline_info->get_ego_state().ego_pose;
  if (frenet_coord == nullptr) {
    return;
  }
  const auto normal_obstacles_items = obstacle_manager.get_obstacles().Items();
  const auto virtual_obstacles_items = context->get_virtual_obstacles().Items();
  static std::vector<const msquare::Obstacle *> obstacles_items;
  obstacles_items = normal_obstacles_items;
  obstacles_items.insert(obstacles_items.end(), virtual_obstacles_items.begin(),
                         virtual_obstacles_items.end());
  for (const auto &obstacle : obstacles_items) {
    Obstacle *ptr_obstacle = obstacle_manager.find_obstacle(obstacle->Id());
    if (ptr_obstacle == nullptr) {
      ptr_obstacle = context->find_virtual_obstacle(obstacle->Id());
    }
    auto ptr_obstacle_decision =
        context->mutable_obstacle_decision_manager().find_obstacle_decision(
            obstacle->Id());
    mph_assert(ptr_obstacle != nullptr);
    // std::vector<PolygonWithT> sl_polygon_points;
    bool is_lon_ignore = false;
    if (ptr_obstacle_decision->HasLongitudinalDecision()) {
      if (ptr_obstacle_decision->LongitudinalDecision().has_ignore()) {
        is_lon_ignore = true;
      }
    }
    bool is_lat_ignore = false;
    if (ptr_obstacle_decision->HasLateralDecision()) {
      if (ptr_obstacle_decision->LateralDecision().has_ignore()) {
        is_lat_ignore = true;
      }
    }
    if (is_lon_ignore && is_lat_ignore) {
      continue;
    }

    if (!obstacle->sl_polygon_seq().empty() || obstacle->has_sl_polygon_seq()) {
      continue;
    }

    std::string failed_reson = "";
    bool compute_sucess = compute_obs_sl_polygon_seq(
        ptr_obstacle, baseline_info, time_range, time_gap,
        config_.enable_heuristic_frenet_search, failed_reson);
    if (!compute_sucess) {
      LOG_LON_DECISION_INFO(ptr_obstacle->Id(), "ignore", failed_reson);
    }
  }
}

bool StGraphGenerator::compute_obs_sl_polygon_seq(
    Obstacle *obstacle, std::shared_ptr<BaseLineInfo> baseline_info,
    StGraphData::DoublePair time_range, double time_gap,
    bool enable_heuristic_search, std::string &failed_reson) {
  if (obstacle->IsFrenetInvalid()) {
    failed_reson = "will be ignored, frenet is invalid";
    return false;
  }
  static SLPolygonSeq sl_polygon_seq;
  sl_polygon_seq.reserve(50);
  sl_polygon_seq.unset();
  auto frenet_coord = baseline_info->get_frenet_coord();
  auto ego_cart_pose = baseline_info->get_ego_state().ego_pose;
  if (obstacle->IsStatic() ||
      std::fabs(obstacle->Trajectory().back().path_point.s -
                obstacle->Trajectory().front().path_point.s) < 1.e-2) {
    // auto perception_box = obstacle->PerceptionBoundingBox();
    static PolygonWithT p0_point, p1_point;
    p0_point.second.clear();
    p1_point.second.clear();
    p0_point.first = time_range.first;
    p1_point.first = time_range.second;
    static std::vector<planning_math::Vec2d> cart_vertexes, fren_vertexes;
    cart_vertexes.clear();
    fren_vertexes.clear();
    // cart_vertexes = perception_box.GetAllCorners();
    auto perception_polygon = obstacle->PerceptionPolygon();
    bool is_vertexes_valid = true;

    for (auto cart_vertex : perception_polygon.points()) {
      Point2D cart_point, fren_point;
      cart_point.x = cart_vertex.x();
      cart_point.y = cart_vertex.y();

      if (frenet_coord->CartCoord2FrenetCoord(cart_point, fren_point) ==
              TRANSFORM_FAILED ||
          std::isnan(fren_point.x) || std::isnan(fren_point.y)) {
        is_vertexes_valid = false;
        obstacle->SetSLPolygonSequenceInvalid(true);
        failed_reson = "will be ignored, frenet convert is failed";
        return false;
      }
      planning_math::Vec2d fren_vertex(fren_point.x, fren_point.y);
      fren_vertexes.push_back(fren_vertex);
      MSD_LOG(INFO, "ST_DEBUG3:ID:%d,fx:%.2f,fy:%.2f", obstacle->Id(),
              fren_point.x, fren_point.y);
    }
    if (!is_vertexes_valid) {
      obstacle->SetSLPolygonSequenceInvalid(true);
      failed_reson = "will be ignored, frenet convert is failed";
      return false;
    }
    static planning_math::Polygon2d convex_polygon;
    convex_polygon.clear();
    if (!planning_math::Polygon2d::ComputeConvexHull(fren_vertexes,
                                                     &convex_polygon)) {
      obstacle->SetSLPolygonSequenceInvalid(true);
      failed_reson = "will be ignored, mpute convexhull is failed";
      return false;
    }
    generate_precise_sl_polygon(convex_polygon, frenet_coord);
    p0_point.second = convex_polygon;
    p1_point.second = convex_polygon;
    sl_polygon_seq.push_back(p0_point);
    sl_polygon_seq.push_back(p1_point);
    obstacle->SetSLPolygonSequence(sl_polygon_seq);
    if (obstacle->Id() == -3) {
      for (auto p : convex_polygon.points()) {
        MSD_LOG(INFO, "virtual p0 x %f y %f", p.x(), p.y());
      }
    }
    return true;
  } else {
    int dimension =
        static_cast<int>((time_range.second - time_range.first) / time_gap);
    double last_cal_dis = 0.0;
    // double last_cal_frenet_s_min = 0.0;
    // double last_cal_frenet_s_max = 0.0;
    static std::vector<std::pair<double, double>> invalid_time_sections;
    invalid_time_sections.clear();
    std::pair<double, double> invalid_time_section{8.0, 0.0};
    bool has_invalid_time_sec{false};
    constexpr double kDefaultCurvatureRadius = 5.0;
    double min_obs_check_length =
        std::min(2.0, std::min(obstacle->PerceptionBoundingBox().width(),
                               obstacle->PerceptionBoundingBox().length()));
    constexpr double kMaxHeuristicDis = 5.0;
    double obs_size = std::max(obstacle->PerceptionBoundingBox().width(),
                               obstacle->PerceptionBoundingBox().length());
    auto init_traj_point = obstacle->GetPointAtTime(0.0);
    bool rough_mode =
        std::hypot(init_traj_point.path_point.x - ego_cart_pose.x,
                   init_traj_point.path_point.y - ego_cart_pose.y) > 30.0;
    MSD_LOG(INFO, "ST_DEBUG3:ID:%d,rough_mode:%d", obstacle->Id(), rough_mode);
    for (size_t i = 0; i < dimension; ++i) {
      double t = i * time_gap + time_range.first;
      auto traj_point = obstacle->GetPointAtTime(t);

      if (traj_point.path_point.s - last_cal_dis < min_obs_check_length &&
          i != 0 && i != dimension - 1) {
        continue;
      }
      // auto moving_box = obstacle->GetBoundingBox(traj_point);
      static planning_math::Polygon2d moving_polygon;
      moving_polygon.clear();
      bool has_heuristics = false;
      double heuristic_s_begin, heuristic_s_end;

      if (enable_heuristic_search && !sl_polygon_seq.empty()) {
        auto &last_polygon = sl_polygon_seq.back().second;
        double curvature = max(
            std::abs(frenet_coord->GetRefCurveCurvature(
                clip(last_polygon.min_x(), frenet_coord->GetLength(), 0.0))),
            std::abs(frenet_coord->GetRefCurveCurvature(
                clip(last_polygon.max_x(), frenet_coord->GetLength(), 0.0))));
        double cur_radius = curvature > 0.0
                                ? 1.0 / curvature
                                : std::numeric_limits<double>::infinity();
        double euler_dis = std::abs(traj_point.path_point.s - last_cal_dis);
        double last_frenet_l = std::max(std::abs(last_polygon.min_y()),
                                        std::abs(last_polygon.max_y()));
        if (cur_radius > std::max(kDefaultCurvatureRadius,
                                  2 * (last_frenet_l + euler_dis + obs_size)) &&
            std::abs(traj_point.path_point.s - last_cal_dis) <
                kMaxHeuristicDis) {

          double theta =
              std::asin((euler_dis + obs_size) / (cur_radius - last_frenet_l));
          double search_buffer1 = theta * cur_radius;
          double search_buffer = std::max(
              std::abs(traj_point.path_point.s - last_cal_dis), search_buffer1);
          has_heuristics = true;
          heuristic_s_begin =
              std::max(0.0, last_polygon.min_x() - search_buffer);
          heuristic_s_end = std::max(0.0, last_polygon.max_x() + search_buffer);
        }
      }
      if (rough_mode) {
        if (!generate_rough_obs_sl_polygon(
                moving_polygon, obstacle, traj_point, frenet_coord,
                has_heuristics, heuristic_s_begin, heuristic_s_end)) {
          if (!has_invalid_time_sec) {
            has_invalid_time_sec = true;

            if (sl_polygon_seq.empty()) {
              invalid_time_section.first = t;
            } else {
              invalid_time_section.first =
                  (sl_polygon_seq.back().first + t) / 2.0;
            }
          }
          invalid_time_section.second =
              std::max(invalid_time_section.second, t);
          if (i == dimension - 1) {
            invalid_time_sections.emplace_back(invalid_time_section);
          }
          continue;
        } else {
          PolygonWithT p_point;
          p_point.first = t;
          p_point.second = moving_polygon;
          sl_polygon_seq.push_back(p_point);

          if (has_invalid_time_sec) {
            has_invalid_time_sec = false;
            invalid_time_section.second =
                std::max(invalid_time_section.second,
                         (invalid_time_section.second + t) / 2.0);
            invalid_time_sections.emplace_back(invalid_time_section);
            invalid_time_section = {8.0, 0.0};
          }
          continue;
        }
      } else {
        moving_polygon = obstacle->GetPolygonAtPoint(traj_point);
      }
      PolygonWithT p_point;
      p_point.first = t;
      static std::vector<planning_math::Vec2d> cart_vertexes, fren_vertexes;
      cart_vertexes.clear();
      fren_vertexes.clear();
      // cart_vertexes = moving_box.GetAllCorners();
      bool is_vertexes_valid = true;
      for (auto cart_vertex : moving_polygon.points()) {
        Point2D cart_point, fren_point;
        cart_point.x = cart_vertex.x();
        cart_point.y = cart_vertex.y();
        if (frenet_coord->CartCoord2FrenetCoord(
                cart_point, fren_point, has_heuristics, heuristic_s_begin,
                heuristic_s_end) == TRANSFORM_FAILED ||
            std::isnan(fren_point.x) || std::isnan(fren_point.y)) {
          is_vertexes_valid = false;
          break;
        }
        planning_math::Vec2d fren_vertex(fren_point.x, fren_point.y);
        fren_vertexes.push_back(fren_vertex);
        MSD_LOG(INFO, "ST_DEBUG3:ID:%d,i:%d,fcx:%.2f,fcy:%.2f", obstacle->Id(),
                fren_point.x, fren_point.y);
      }

      if (!is_vertexes_valid) {
        if (!has_invalid_time_sec) {
          has_invalid_time_sec = true;
          if (sl_polygon_seq.empty()) {
            invalid_time_section.first = t;
          } else {
            invalid_time_section.first =
                (sl_polygon_seq.back().first + t) / 2.0;
          }
        }
        invalid_time_section.second = std::max(invalid_time_section.second, t);
        if (i == dimension - 1) {
          invalid_time_sections.emplace_back(invalid_time_section);
        }
        continue;
      }
      static planning_math::Polygon2d convex_polygon;
      convex_polygon.clear();
      if (!planning_math::Polygon2d::ComputeConvexHull(fren_vertexes,
                                                       &convex_polygon)) {
        if (!has_invalid_time_sec) {
          has_invalid_time_sec = true;
          if (sl_polygon_seq.empty()) {
            invalid_time_section.first = t;
          } else {
            invalid_time_section.first =
                (sl_polygon_seq.back().first + t) / 2.0;
          }
        }
        invalid_time_section.second = std::max(invalid_time_section.second, t);
        if (i == dimension - 1) {
          invalid_time_sections.emplace_back(invalid_time_section);
        }
        continue;
      }
      generate_precise_sl_polygon(convex_polygon, frenet_coord);
      p_point.second = convex_polygon;

      if (has_invalid_time_sec) {
        has_invalid_time_sec = false;
        invalid_time_section.second =
            std::max(invalid_time_section.second,
                     (invalid_time_section.second + t) / 2.0);
        invalid_time_sections.emplace_back(invalid_time_section);
        invalid_time_section = {8.0, 0.0};
      }
      sl_polygon_seq.push_back(p_point);
      last_cal_dis = traj_point.path_point.s;
      // last_cal_frenet_s_min = convex_polygon.min_x();
      // last_cal_frenet_s_max = convex_polygon.max_x();
    }
    sl_polygon_seq.set_invalid_time_sections(invalid_time_sections);
    if (static_cast<int>(sl_polygon_seq.size()) > 1) {
      // sl_polygon_seq.SetTimeStep(time_gap);
    }
    if (sl_polygon_seq.size() >= 2) {
      obstacle->SetSLPolygonSequence(sl_polygon_seq);
      return true;
    } else {
      failed_reson = "will be ignored, compute sl_polygon is failed";
      obstacle->SetSLPolygonSequenceInvalid(true);
      return false;
    }
  }
}

void StGraphGenerator::set_cross_lane_state() {
  constexpr double adjacent_lane_width = 3.4;
  double lane_expansion_width =
      ConfigurationContext::Instance()->get_vehicle_param().width / 2.0 +
      adjacent_lane_width / 2.0 + 0.1;
  constexpr double dl_buffer = 0.2;
  left_width_ = LANE_WIDTH() * 0.5;
  right_width_ = LANE_WIDTH() * 0.5;
  is_lane_interval_transformed_ = false;

  auto lane_status = context_->mutable_planning_status()->lane_status;
  const auto &ego_sl = baseline_info_->get_adc_sl_boundary();

  MSD_LOG(INFO, "DEBUG_CJ6 lane_status:%d", lane_status.status);
  MSD_LOG(INFO, "DEBUG_CJ6 ego_start_l:%3.2f,ego_end_l:%3.2f", ego_sl.start_l,
          ego_sl.end_l);
  is_in_lane_crossing_stage_ =
      (lane_status.status == LaneStatus::LANE_CHANGE &&
       lane_status.change_lane.status == ChangeLaneStatus::IN_CHANGE_LANE);

  if (lane_status.status == LaneStatus::Status::LANE_KEEP) {
    if (ego_sl.start_l < -right_width_) {
      right_width_ = std::fabs(ego_sl.start_l) + dl_buffer;
      left_width_ =
          std::max(0.0, std::min(left_width_, ego_sl.end_l + dl_buffer));
    }
    if (ego_sl.end_l > left_width_) {
      left_width_ = std::fabs(ego_sl.end_l) + dl_buffer;
      right_width_ =
          std::max(0.0, std::min(right_width_, -ego_sl.start_l + dl_buffer));
    }
    return;
  } else if (lane_status.status == LaneStatus::Status::LANE_BORROW) {
    // TODO(@zzd) need revise
    auto &lateral_output = context_->lateral_behavior_planner_output();
    auto lat_offset = lateral_output.lat_offset;

    if (ego_sl.start_l < -right_width_) {
      right_width_ = std::fabs(ego_sl.start_l) + dl_buffer;
      left_width_ = std::min(left_width_, ego_sl.end_l + dl_buffer);
    }
    if (ego_sl.end_l > left_width_) {
      left_width_ = std::fabs(ego_sl.end_l) + dl_buffer;
      right_width_ = std::min(right_width_, -ego_sl.start_l + dl_buffer);
    }
    return;
  } else if (lane_status.status == LaneStatus::Status::LANE_CHANGE) {
    if (ego_sl.start_l < -right_width_) {
      right_width_ = std::fabs(ego_sl.start_l) + dl_buffer;
    }
    if (ego_sl.end_l > left_width_) {
      left_width_ = std::fabs(ego_sl.end_l) + dl_buffer;
    }
    if (lane_status.change_lane.status ==
        ChangeLaneStatus::Status::IN_CHANGE_LANE) {
      is_lane_interval_transformed_ = true;
    }
    return;
  }
}

void StGraphGenerator::generate_precise_sl_polygon(
    planning_math::Polygon2d &polygon,
    std::shared_ptr<FrenetCoordinateSystem> frenet_coord) {
  static planning_math::Polygon2d result_polygon;
  result_polygon.clear();
  double max_curvature = 0.0;
  double min_l = std::numeric_limits<double>::max();
  constexpr int kMaxInterpolateNums = 5;
  constexpr double kMinCurvatureRadius = 5.0;
  // constexpr double kMaxDeltaCurvature = 0.2;
  constexpr double kDeltaRadian = 0.4;
  constexpr double kLRange = 5.0;
  auto origin_points = polygon.GetAllVertices();
  mph_assert(origin_points.size() > 2);
  static std::vector<double> curvatures;
  curvatures.clear();
  for (auto point : origin_points) {
    if (point.x() < 0.0 || point.x() > frenet_coord->GetLength()) {
      return;
    }
    double curvature = frenet_coord->GetRefCurveCurvature(point.x());
    curvatures.push_back(curvature);
    max_curvature = max(max_curvature, curvature);
    min_l = min(min_l, std::abs(point.y()));
  }
  double cur_radius = max_curvature > 0.0
                          ? 1.0 / max_curvature
                          : std::numeric_limits<double>::infinity();
  if (cur_radius > kMinCurvatureRadius) {
    return;
  }
  if (min_l > kLRange) {
    return;
  }
  static std::vector<planning_math::Vec2d> new_points;
  new_points.clear();
  static std::vector<planning_math::Vec2d> origin_cart_points;
  origin_cart_points.clear();
  for (auto point : origin_points) {
    Point2D fren_point, cart_point;
    fren_point.x = point.x();
    fren_point.y = point.y();
    if (frenet_coord->FrenetCoord2CartCoord(fren_point, cart_point) ==
        TRANSFORM_FAILED) {
      return;
    }
    origin_cart_points.push_back({cart_point.x, cart_point.y});
  }
  for (size_t i = 1; i <= origin_points.size(); ++i) {
    size_t index_begin = (i - 1) % origin_points.size();
    size_t index_end = i % origin_points.size();
    auto begin_point = origin_points[index_begin];
    auto end_point = origin_points[index_end];
    double s_diff = std::abs(end_point.x() - begin_point.x());
    double ref_curvature = max(curvatures[index_begin], curvatures[index_end]);
    double delta_s = max(0.5, kDeltaRadian / ref_curvature);
    int inter_num =
        min(kMaxInterpolateNums, max(2, int(ceil(s_diff / delta_s))));
    for (size_t j = 1; j <= inter_num; ++j) {
      Point2D cur_cart_p, cur_fren_p;
      if (j == inter_num) {
        new_points.emplace_back(end_point);
        continue;
      }
      cur_cart_p.x =
          planning_math::lerp(origin_cart_points[index_begin].x(), 0,
                              origin_cart_points[index_end].x(), inter_num, j);
      cur_cart_p.y =
          planning_math::lerp(origin_cart_points[index_begin].y(), 0,
                              origin_cart_points[index_end].y(), inter_num, j);
      if (frenet_coord->CartCoord2FrenetCoord(cur_cart_p, cur_fren_p) ==
          TRANSFORM_FAILED) {
        return;
      }
      mph_assert(!std::isnan(cur_fren_p.x) && !std::isnan(cur_fren_p.y));
      planning_math::Vec2d cur_point(cur_fren_p.x, cur_fren_p.y);
      new_points.emplace_back(cur_point);
    }
  }

  if (new_points.size() <= 2) {
    return;
  }

  if (!planning_math::Polygon2d::ComputeConvexHull(new_points,
                                                   &result_polygon)) {
    return;
  } else {
    polygon = result_polygon;
  }
}

bool StGraphGenerator::generate_rough_obs_sl_polygon(
    planning_math::Polygon2d &polygon, const Obstacle *obstacle,
    TrajectoryPoint point, std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
    bool has_heuristics, double heuristic_s_begin, double heuristic_s_end) {
  Point2D fren_point, cart_point;
  cart_point.x = point.path_point.x;
  cart_point.y = point.path_point.y;
  if (frenet_coord->CartCoord2FrenetCoord(
          cart_point, fren_point, has_heuristics, heuristic_s_begin,
          heuristic_s_end) == TRANSFORM_FAILED ||
      fren_point.x > frenet_coord->GetLength() || std::isnan(fren_point.x) ||
      std::isnan(fren_point.y)) {
    return false;
  }
  mph_assert(!std::isnan(fren_point.x) && !std::isnan(fren_point.y));
  double obs_length = obstacle->PerceptionBoundingBox().length();
  double obs_width = obstacle->PerceptionBoundingBox().width();
  double theta_ref = frenet_coord->GetRefCurveHeading(fren_point.x);
  double obs_yaw = point.path_point.theta;
  static std::vector<Point2D> corners;
  corners.clear();
  corners.push_back(Point2D(fren_point.x - (obs_length / 2.0),
                            fren_point.y - (obs_width / 2.0)));
  corners.push_back(Point2D(fren_point.x + (obs_length / 2.0),
                            fren_point.y - (obs_width / 2.0)));
  corners.push_back(Point2D(fren_point.x + (obs_length / 2.0),
                            fren_point.y + (obs_width / 2.0)));
  corners.push_back(Point2D(fren_point.x - (obs_length / 2.0),
                            fren_point.y + (obs_width / 2.0)));
  double rotate_angle = obs_yaw - theta_ref, s = 0.0, l = 0.0;
  static std::vector<planning_math::Vec2d> rough_points;
  rough_points.clear();
  for (Point2D &p : corners) {
    s = fren_point.x + (p.x - fren_point.x) * std::cos(rotate_angle) -
        (p.y - fren_point.y) * std::sin(rotate_angle);
    l = fren_point.y + (p.x - fren_point.x) * std::sin(rotate_angle) +
        (p.y - fren_point.y) * std::cos(rotate_angle);
    rough_points.push_back(planning_math::Vec2d(s, l));
  }
  if (!planning_math::Polygon2d::ComputeConvexHull(rough_points, &polygon)) {
    return false;
  }
  return true;
}

} // namespace msquare
