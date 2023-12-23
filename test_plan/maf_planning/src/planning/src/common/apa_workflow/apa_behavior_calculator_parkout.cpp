#include "common/apa_workflow/apa_behavior_calculator_parkout.h"

namespace msquare {
namespace parking {

using planning_math::Vec2d;
using planning_math::Box2d;
using planning_math::Polygon2d;


APABehaviorCalculatorParkOut::APABehaviorCalculatorParkOut(
    const std::shared_ptr<WorldModel> &world_model)
    : world_model_(world_model) {}

APABehaviorCalculatorParkOut::~APABehaviorCalculatorParkOut() {}

bool APABehaviorCalculatorParkOut::area_has_obstacle(
    const planning_math::Box2d &area) {
  // Check if area has obstacle
  auto &obstacles = world_model_->obstacle_manager().get_obstacles().Items();
  for (const auto &obstacle : obstacles) {
    if (area.HasOverlap(obstacle->PerceptionBoundingBox())) {
      return true;
    }
  }

  auto &points = world_model_->obstacle_manager().get_points().Items();
  for (const auto &point : points) {
    planning_math::Vec2d p(point->point().x, point->point().y, point->Id());
    if (area.IsPointIn(p)) {

      return true;
    }
  }
  return false;
}

bool APABehaviorCalculatorParkOut::calculate_reached_takeover_point(
    bool is_at_last_segment, bool openspace_is_running) {
  // ################## Rule Explanation ##################
  // [Conditions] 
  //    1. reached take_over point before  (g_reached_takeover_point == true)
  //    2. (OR) takeover conditions matched
  //      2.1. is_at_last_segment
  //      2.2. (AND) openspace_is_running
  //      2.3. (AND) gear correct
  //            3.1 gear == REVERSE when parkout REAR
  //            3.2 (OR) gear == DRIVE when parkout front
  //      2.4. (AND) yaw not too far fron target 
  //              yaw - target.yaw < 45deg 
  //      2.5  (AND) leave starting pose far enough: 
  //              yaw - init.yaw > 16deg  OR xy - init.xy > 2.5m
  //      2.6. (AND) check-area has no obstacle


  // ############## End of Rule Explanation ###############

  //[PlanningContext Input]
  const auto &g_planning_gear =
      PlanningContext::Instance()->planning_status().planning_result.gear;

  const auto &g_parkout_type = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info.park_out_type.value;

  const Pose2D &g_park_out_target = PlanningContext::Instance()
                                       ->parking_behavior_planner_output()
                                       .parking_slot_info.park_out_point.point;

  const Pose2D &g_park_out_init = PlanningContext::Instance()
                                       ->parking_behavior_planner_output()
                                       .parking_slot_info.park_out_init_pose;

  const bool g_reached_takeover_point = PlanningContext::Instance()
                                            ->parking_behavior_planner_output()
                                            .reached_parkout_takeover_point;

  const auto &ego_pose = world_model_->get_ego_state().ego_pose;
  //[Calculation]
  // 1. reached take_over point before
  if(g_reached_takeover_point){
    return true;
  }
  // 2.1 && 2.2 is directly input from function params
  // 2.3 gear correct
  const bool parkout_is_rear =
      g_parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_REAR ||
      g_parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_REAR_LEFT ||
      g_parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_REAR_RIGHT;

  bool gear_is_correct;
  if (parkout_is_rear) {
    gear_is_correct = g_planning_gear == GearState::REVERSE;
  } else {
    gear_is_correct = g_planning_gear == GearState::DRIVE;
  }

  // 2.4 yaw not too far fron target
  const double angle_diff_target_th = 45.0 / 180.0 * M_PI;
  double angle_diff_from_target =
      planning_math::AngleDiff(ego_pose.theta, g_park_out_target.theta);
  bool angle_diff_from_target_small =
      std::abs(angle_diff_from_target) < angle_diff_target_th;

  // 2.5 leave starting pose far enough
  const double angle_diff_init_th = 16.0 / 180.0 * M_PI;
  const double xy_diff_init_th = 2.5; // meters
  double angle_diff_from_init =
      planning_math::AngleDiff(ego_pose.theta, g_park_out_init.theta);
  double xy_diff_from_init = std::hypot(
      ego_pose.x - g_park_out_init.x, ego_pose.y - g_park_out_init.y);
  bool leave_init_pose_far_enough =
      std::abs(angle_diff_from_init) > angle_diff_init_th ||
      xy_diff_from_init > xy_diff_init_th;

  // prevent heavy calculations if previous conditions not satisfied
  if (!(is_at_last_segment && gear_is_correct && openspace_is_running &&
      angle_diff_from_target_small && leave_init_pose_far_enough)) {
    return false;
  }

  //construct check-area
  const double check_length = 2.5;
  const double parkout_takeover_point_lat_inflation = 0.5;
  const double width = CarParams::GetInstance()->vehicle_width_real + parkout_takeover_point_lat_inflation;

  //"back_edge_to_center" is actually rear_bumper_to_rear_axle 
  //
  //   in src/planning/src/common/config/vehicle_param.cpp Function VehicleParam::loadFile
  const double front_to_rear = VehicleParam::Instance()->front_edge_to_center;
  const double back_to_rear = VehicleParam::Instance()->back_edge_to_center;
  
  planning_math::Vec2d ego_position{ego_pose.x, ego_pose.y};
  planning_math::Vec2d heading_vector =
      planning_math::Vec2d::CreateUnitVec2d(ego_pose.theta);

  planning_math::LineSegment2d ego_center_axis = planning_math::LineSegment2d(
      ego_position - back_to_rear* heading_vector, ego_position + front_to_rear* heading_vector);
  planning_math::Box2d check_area = planning_math::Box2d(ego_center_axis, width);

  if (parkout_is_rear) {
    check_area.Shift(-check_length  * heading_vector);
  } else {
    check_area.Shift(check_length * heading_vector);
  }

  //check if area has obstacle
  bool obstacle_inside_check_area = area_has_obstacle(check_area);

  return !obstacle_inside_check_area;
}

bool APABehaviorCalculatorParkOut::check_parkout_direction(uint32_t direction) {

  const auto &g_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;

  const auto &g_slot_type = g_slot_info.type.value;

  const auto &g_wheel_stop_info = g_slot_info.wheel_stop_info;

  const double length = CarParams::GetInstance()->vehicle_length_real;
  const double width = CarParams::GetInstance()->vehicle_width_real;
  const double front_to_rear = VehicleParam::Instance()->front_edge_to_center;
  const double back_to_rear = VehicleParam::Instance()->back_edge_to_center;
  
  const Pose2D ego_pose = world_model_->get_ego_state().ego_pose;
  
  planning_math::Vec2d ego_position{ego_pose.x, ego_pose.y};
  planning_math::Vec2d heading_vector =
      planning_math::Vec2d::CreateUnitVec2d(ego_pose.theta);
  
  planning_math::LineSegment2d ego_center_axis = planning_math::LineSegment2d(
      ego_position - back_to_rear* heading_vector, ego_position + front_to_rear* heading_vector);
  planning_math::Box2d check_area = planning_math::Box2d(ego_center_axis, width);

  const double parallel_shift_dis = 1.2;
  const double parallel_front_shift_dis = 1.8;
  const double perpendicular_shift_dis = 3.7;
  const double oblique_shift_dis = 3.0;

  const double perpendicular_oblique_shift_dis =
      g_slot_type == ParkingSlotType::OBLIQUE ? oblique_shift_dis
                                              : perpendicular_shift_dis;
  switch (direction) {
  // perpendicular or oblique
  case ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_LEFT:
  case ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_RIGHT:
  case ParkOutType::PERPENDICULAR_OBLIQUE_FRONT:
    check_area.Shift(perpendicular_oblique_shift_dis * heading_vector);
    break;
  case ParkOutType::PERPENDICULAR_OBLIQUE_REAR_LEFT:
  case ParkOutType::PERPENDICULAR_OBLIQUE_REAR_RIGHT:
  case ParkOutType::PERPENDICULAR_OBLIQUE_REAR:
    check_area.Shift(-perpendicular_oblique_shift_dis * heading_vector);
    break;
  //parallel
  case ParkOutType::PARALLEL_LEFT:
    check_area.Shift(parallel_shift_dis * heading_vector.rotate(M_PI_2));
    break;
  case ParkOutType::PARALLEL_LEFT_FRONT:
    check_area.Shift(parallel_front_shift_dis * heading_vector.rotate(M_PI_2));
    break;
  case ParkOutType::PARALLEL_RIGHT:
    check_area.Shift(parallel_shift_dis * heading_vector.rotate(-M_PI_2));
    break;
  case ParkOutType::PARALLEL_RIGHT_FRONT:
    check_area.Shift(parallel_front_shift_dis * heading_vector.rotate(-M_PI_2));
    break;

  default:
    break;
  }

  //Check Collide with wheel-stop
  if (g_wheel_stop_info.vision_wheel_stop_available) {
    planning_math::Vec2d wheelstop_point1(g_wheel_stop_info.vision_point1.x,
                                          g_wheel_stop_info.vision_point1.y);
    planning_math::Vec2d wheelstop_point2(g_wheel_stop_info.vision_point2.x,
                                          g_wheel_stop_info.vision_point2.y);
    planning_math::LineSegment2d wheel_stop_line(wheelstop_point1,
                                                 wheelstop_point2);

    if (check_area.HasOverlap(wheel_stop_line)) {
      MSD_LOG(ERROR,
              "[check_parkout_direction] collision with wheel_stop_line");
      return false;
    }
  }   

  auto &obstacles = world_model_->obstacle_manager().get_obstacles().Items();
  for (const auto &obstacle : obstacles) {
    if (msquare::CarParams::GetInstance()->car_config.apoa_config.is_direct_dynamic_person_ignore) {
      if (obstacle->Type() == ObjectType::PEDESTRIAN) {
        MSD_LOG(ERROR,"[check_parkout_direction] PEDESTRIAN continue");
        continue;
      }
    }
    if (check_area.HasOverlap(obstacle->PerceptionBoundingBox())) {
      MSD_LOG(ERROR, "[check_parkout_direction] collision with PerceptionBoundingBox");  
      return false;
    }
  }

  for (auto &obs : world_model_->get_parking_ground_line_fusion()) {
    if (obs.id != g_slot_info.id) {
      continue;
    }
    for (int i = 0; i < obs.pts.size(); i++) {
      planning_math::Vec2d uss_point{obs.pts[i].x, obs.pts[i].y};
      if (check_area.IsPointIn(uss_point)) {
        MSD_LOG(ERROR, "[check_parkout_direction] collision with uss_point");  
        return false;
      }
    }
    if (g_slot_type != ParkingSlotType::PARALLEL) {
      continue;
    }
    if (!(obs.type == GroundLineType::GROUND_LINE_USS_TYPE_UNKNOWN ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_WALL ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_PILLAR ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_FENCE ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_STEP ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_SPECIAL ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_VEHICLE ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_ONLY_USS_UNKNOWN)) {
      continue;
    }
    if (obs.pts.size() % 2 != 0) {
      continue;
    }

    for (unsigned int i = 0; i < obs.pts.size(); i = i + 2) {
      planning_math::LineSegment2d uss_ground_line{
          planning_math::Vec2d{obs.pts[i].x, obs.pts[i].y},
          planning_math::Vec2d{obs.pts[i + 1].x, obs.pts[i + 1].y}};
      if (check_area.HasOverlap(uss_ground_line)) {
        MSD_LOG(ERROR, "[check_parkout_direction] collision with uss_ground_line");  
        return false;
      }
    }
  }


  return true;
}

bool APABehaviorCalculatorParkOut::calc_parkout_target_perpendicular_oblique(
    Pose2D &out_target_pose, Pose2D &out_error_tolerence, bool oblique) {

  const auto &parking_slot_info = PlanningContext::Instance()
                                      ->parking_behavior_planner_output()
                                      .parking_slot_info;
  const Pose2D ego_pose = world_model_->get_ego_state().ego_pose;
  const auto &parkout_type = parking_slot_info.park_out_type.value;

  const bool use_legacy_parkout =
      CarParams::GetInstance()->car_config.apoa_config.use_legacy_parkout;

  if (parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_FRONT ||
      parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_REAR) {
    // independent approach for front/rear straight parkout
    const double PERPENDICULAR_OFFSET = use_legacy_parkout ? 3.7 : 5.0;
    const double OBLIQUE_OFFSET = use_legacy_parkout ? 3.7 : 5.0;

    const double straight_offset = oblique ? PERPENDICULAR_OFFSET : OBLIQUE_OFFSET;
    const planning_math::Vec2d ego_vec = Vec2d::CreateUnitVec2d(ego_pose.theta);
    double dir;
    if (parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_FRONT) {
      dir = 1.0;
    } else {
      dir = -1.0;
    }
    planning_math::Vec2d ego_position(ego_pose.x, ego_pose.y);
    planning_math::Vec2d target_xy = ego_position + dir * ego_vec * straight_offset;
    
    out_target_pose.x = target_xy.x();
    out_target_pose.y = target_xy.y();
    out_target_pose.theta = ego_pose.theta; 
    out_error_tolerence.x = 0;
    out_error_tolerence.y = 0;
    out_error_tolerence.theta = 0;
    return true;
  }

  planning_math::Vec2d corner0{parking_slot_info.original_corners[0].x, parking_slot_info.original_corners[0].y};
  planning_math::Vec2d corner1{parking_slot_info.original_corners[1].x, parking_slot_info.original_corners[1].y};
  planning_math::Vec2d corner2{parking_slot_info.original_corners[2].x, parking_slot_info.original_corners[2].y};
  planning_math::Vec2d corner3{parking_slot_info.original_corners[3].x, parking_slot_info.original_corners[3].y};
  // virtual corner replacing real corner in rectangle oblique
  if (oblique && parking_slot_info.virtual_corner.slot_type == 4) {
    planning_math::Vec2d virtual_corner{parking_slot_info.virtual_corner.p.x, parking_slot_info.virtual_corner.p.y};
    if (parking_slot_info.virtual_corner.index == 0) {
      corner0 = virtual_corner;
    } else if (parking_slot_info.virtual_corner.index == 3) {
      corner3 = virtual_corner;
    }
  }

  const planning_math::Vec2d corner0_in_ego = planning_math::tf2d(ego_pose, corner0);
  const planning_math::Vec2d corner1_in_ego = planning_math::tf2d(ego_pose, corner1);
  const planning_math::Vec2d corner2_in_ego = planning_math::tf2d(ego_pose, corner2);
  const planning_math::Vec2d corner3_in_ego = planning_math::tf2d(ego_pose, corner3);

  const bool parkout_is_front_left =
      parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_LEFT;

  const bool parkout_is_front_right =
      parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_RIGHT;

  const bool parkout_is_rear_left =
      parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_REAR_LEFT;

  const bool parkout_is_rear_right =
      parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_REAR_RIGHT;

  const bool parkout_is_front = parkout_is_front_left || parkout_is_front_right;
  
  const bool parkout_is_left = parkout_is_front_left || parkout_is_rear_left;

  // Step1. Find out reference edge (the edge where ego vehicle parkout through)
  //
  const bool edge_03_is_in_ego_front =
      corner0_in_ego.x() + corner3_in_ego.x() > corner1_in_ego.x() + corner2_in_ego.x();

  planning_math::Vec2d ref_edge_c0;
  planning_math::Vec2d ref_edge_c1;
  planning_math::Vec2d ref_edge_c0_in_ego;
  planning_math::Vec2d ref_edge_c1_in_ego;
  if (edge_03_is_in_ego_front == parkout_is_front) {
    //  parkout throuth 0-3 edge
    ref_edge_c0 = corner0;
    ref_edge_c1 = corner3;
    ref_edge_c0_in_ego = corner0_in_ego;
    ref_edge_c1_in_ego = corner3_in_ego;
  } else {
    //  parkout throuth 1-2 edge
    ref_edge_c0 = corner1;
    ref_edge_c1 = corner2;
    ref_edge_c0_in_ego = corner1_in_ego;
    ref_edge_c1_in_ego = corner2_in_ego;
  }

  // Step2. Find out anchor point and unitvec_lon
  planning_math::Vec2d anchor_point;
  planning_math::Vec2d unitvec_lon;
  planning_math::Vec2d unitvec_lat;

  if (parkout_is_left == ref_edge_c0_in_ego.y() > ref_edge_c1_in_ego.y()) {
    // parkout_is_left  && ref_edge_c0 is the left point
    //    OR parkout_is_right && ref_edge_c1 is the left point
    anchor_point = ref_edge_c0;
    unitvec_lon = ref_edge_c0 - ref_edge_c1;
    unitvec_lon.Normalize();
  } else {
    anchor_point = ref_edge_c1;
    unitvec_lon = ref_edge_c1 - ref_edge_c0;
    unitvec_lon.Normalize();
  }

  // Step3. find out unitvec_lon, and Calculate offset
  const double FRONT_LEFT_LON_MIN = 0.0;
  const double FRONT_LEFT_LON_MAX = 5.5;
  const double FRONT_LEFT_LAT_MIN = 3.0;
  const double FRONT_LEFT_LAT_MAX = 3.5;

  const double FRONT_RIGHT_LON_MIN = 0.0;
  const double FRONT_RIGHT_LON_MAX = 5.5;
  const double FRONT_RIGHT_LAT_MIN = 0.7;
  const double FRONT_RIGHT_LAT_MAX = 2.0;

  const double REAR_LEFT_LON_MIN = 0.0;
  const double REAR_LEFT_LON_MAX = 5.5;
  const double REAR_LEFT_LAT_MIN = 3.0;
  const double REAR_LEFT_LAT_MAX = 3.5;

  const double REAR_RIGHT_LON_MIN = 0.0;
  const double REAR_RIGHT_LON_MAX = 5.5;
  const double REAR_RIGHT_LAT_MIN = 0.7;
  const double REAR_RIGHT_LAT_MAX = 2.0;

  double lat_min = 0; 
  double lat_max = 0;
  double lon_min = 0;
  double lon_max = 0;

  if (parkout_is_front_left) {
    unitvec_lat = unitvec_lon.rotate(-M_PI_2);

    lat_min = FRONT_LEFT_LAT_MIN;
    lat_max = FRONT_LEFT_LAT_MAX;
    lon_min = FRONT_LEFT_LON_MIN;
    lon_max = FRONT_LEFT_LON_MAX;
  } else if (parkout_is_front_right) {
    unitvec_lat = unitvec_lon.rotate(M_PI_2);

    lat_min = FRONT_RIGHT_LAT_MIN;
    lat_max = FRONT_RIGHT_LAT_MAX;
    lon_min = FRONT_RIGHT_LON_MIN;
    lon_max = FRONT_RIGHT_LON_MAX;
  } else if (parkout_is_rear_left) {
    unitvec_lat = unitvec_lon.rotate(M_PI_2);

    lat_min = REAR_LEFT_LAT_MIN;
    lat_max = REAR_LEFT_LAT_MAX;
    lon_min = REAR_LEFT_LON_MIN;
    lon_max = REAR_LEFT_LON_MAX;
  } else if (parkout_is_rear_right) {
    unitvec_lat = unitvec_lon.rotate(-M_PI_2);

    lat_min = REAR_RIGHT_LAT_MIN;
    lat_max = REAR_RIGHT_LAT_MAX;
    lon_min = REAR_RIGHT_LON_MIN;
    lon_max = REAR_RIGHT_LON_MAX;
  }

  //"back_edge_to_center" is actually rear_bumper_to_rear_axle 
  //   in src/planning/src/common/config/vehicle_param.cpp Function VehicleParam::loadFile
  const double rear_bumper_to_rear_axle = VehicleParam::Instance()->back_edge_to_center;
  const double vehicle_width = VehicleParam::Instance()->width;
  const double front_bumper_to_rear_axle = VehicleParam::Instance()->length - rear_bumper_to_rear_axle;

  double bumper_to_axle_offset = parkout_is_front ? rear_bumper_to_rear_axle : front_bumper_to_rear_axle;

  double offset_lat = 0.5 * (lat_min + lat_max) + 0.5 * vehicle_width;
  double offset_lon = 0.5 * (lon_min + lon_max) + bumper_to_axle_offset;

  double target_yaw = parkout_is_front
                          ? std::atan2(unitvec_lon.y(), unitvec_lon.x())
                          : std::atan2(-unitvec_lon.y(), -unitvec_lon.x());

  planning_math::Vec2d target_xy =
      anchor_point + offset_lon * unitvec_lon + offset_lat * unitvec_lat;

  double error_tolerence_x = 0.5 * (lon_max - lon_min);
  double error_tolerence_y = 0.5 * (lat_max - lat_min);
  double error_tolerence_yaw = 5.0 * M_PI / 180.0;

  // output result
  out_target_pose.x = target_xy.x();
  out_target_pose.y = target_xy.y();
  out_target_pose.theta = target_yaw;
  out_error_tolerence.x = error_tolerence_x;
  out_error_tolerence.y = error_tolerence_y;
  out_error_tolerence.theta = error_tolerence_yaw;

  return true;
}

bool APABehaviorCalculatorParkOut::calc_parkout_target_parallel(
    Pose2D &target_pose, Pose2D &error_tolerence) {
  const auto &parking_slot_info = PlanningContext::Instance()
                                      ->parking_behavior_planner_output()
                                      .parking_slot_info;
  const Pose2D ego_pose = world_model_->get_ego_state().ego_pose;
  const auto& parkout_type = parking_slot_info.park_out_type.value;

  const planning_math::Vec2d corner0{parking_slot_info.original_corners[0].x, parking_slot_info.original_corners[0].y};
  const planning_math::Vec2d corner1{parking_slot_info.original_corners[1].x, parking_slot_info.original_corners[1].y};
  const planning_math::Vec2d corner2{parking_slot_info.original_corners[2].x, parking_slot_info.original_corners[2].y};
  const planning_math::Vec2d corner3{parking_slot_info.original_corners[3].x, parking_slot_info.original_corners[3].y};

  const planning_math::Vec2d corner0_in_ego = planning_math::tf2d(ego_pose, corner0);
  const planning_math::Vec2d corner1_in_ego = planning_math::tf2d(ego_pose, corner1);
  const planning_math::Vec2d corner2_in_ego = planning_math::tf2d(ego_pose, corner2);
  const planning_math::Vec2d corner3_in_ego = planning_math::tf2d(ego_pose, corner3);

  //Step1. Find out reference edge (the edge where ego vehicle parkout through)
  //       Anchor point (front point of reference edge)
  //       and parkout unitvec lon & lat
  //       
  bool edge_03_is_in_ego_left = corner0_in_ego.y() + corner3_in_ego.y() > corner1_in_ego.y() + corner2_in_ego.y();
    bool parkout_is_parallel_left = parkout_type == ParkOutType::PARALLEL_LEFT_FRONT ||
                                    parkout_type == ParkOutType::PARALLEL_LEFT;
  planning_math::Vec2d anchor_point;
  planning_math::Vec2d unitvec_lon;
  planning_math::Vec2d unitvec_lat;
  if (edge_03_is_in_ego_left == parkout_is_parallel_left) {
    //  parkout throuth 0-3 edge
    //  anchor_point = 0 or 3  depens on which one is in ego front
    if (corner0_in_ego.x() > corner3_in_ego.x()) {
      anchor_point = corner0;
      unitvec_lon = (corner0 - corner3);
      unitvec_lon.Normalize();
    } else {
      anchor_point = corner3;
      unitvec_lon = (corner3 - corner0);
      unitvec_lon.Normalize();
    }
  } else {
    //  parkout throuth 1-2 edge
    //  anchor_point = 1 or 2  depens on which one is in ego front
    if (corner1_in_ego.x() > corner2_in_ego.x()) {
      anchor_point = corner1;
      unitvec_lon = (corner1 - corner2);
      unitvec_lon.Normalize();
    } else {
      anchor_point = corner2;
      unitvec_lon = (corner2 - corner1);
      unitvec_lon.Normalize();
    }
  }
  if (parkout_is_parallel_left) {
    unitvec_lat = unitvec_lon.rotate(M_PI_2);
  } else {
    unitvec_lat = unitvec_lon.rotate(-M_PI_2);
  }

  //Step2. Calculate target pose  
  //  in parallel parkout , target poses are all front 
  const double PARALLEL_OFFSET_LON_MIN = 0.0;
  const double PARALLEL_OFFSET_LON_MAX = 5.0;
  const double PARALLEL_OFFSET_LAT_MIN = 0.7;
  const double PARALLEL_OFFSET_LAT_MAX = 1.2;

  //"back_edge_to_center" is actually rear_bumper_to_rear_axle 
  //   in src/planning/src/common/config/vehicle_param.cpp Function VehicleParam::loadFile
  const double rear_bumper_to_rear_axle = VehicleParam::Instance()->back_edge_to_center;  
  
  const double vehicle_width = VehicleParam::Instance()->width;
  const double offset_lon = 0.5*(PARALLEL_OFFSET_LON_MIN + PARALLEL_OFFSET_LON_MAX) + rear_bumper_to_rear_axle;
  const double offset_lat = 0.5*(PARALLEL_OFFSET_LAT_MIN + PARALLEL_OFFSET_LAT_MAX) + 0.5*vehicle_width;

  double target_yaw = std::atan2(unitvec_lon.y(), unitvec_lon.x());
  planning_math::Vec2d target_xy =
      anchor_point + offset_lon * unitvec_lon + offset_lat * unitvec_lat;

  double error_tolerence_x = 0.5 * (PARALLEL_OFFSET_LON_MAX - PARALLEL_OFFSET_LON_MIN);
  double error_tolerence_y = 0.5 * (PARALLEL_OFFSET_LAT_MAX - PARALLEL_OFFSET_LAT_MIN);
  double error_tolerence_yaw = 5.0 * M_PI / 180.0;
  
  target_pose.x = target_xy.x();
  target_pose.y = target_xy.y();
  target_pose.theta = target_yaw;
  error_tolerence.x = error_tolerence_x;
  error_tolerence.y = error_tolerence_y;
  error_tolerence.theta = error_tolerence_yaw;  
  return true;
}

bool APABehaviorCalculatorParkOut::calculate_parkout_target(
    Pose2D &out_target_pose, Pose2D &out_error_tolerence) {

  const auto &parking_slot_info = PlanningContext::Instance()
                                      ->parking_behavior_planner_output()
                                      .parking_slot_info;
  const auto &slot_type = parking_slot_info.type.value;
  const auto &parkout_type = parking_slot_info.park_out_type.value;
  bool is_valid_direction =
      parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_FRONT ||
      parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_REAR ||
      parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_LEFT ||
      parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_RIGHT ||
      parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_REAR_LEFT ||
      parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_REAR_RIGHT ||
      parkout_type == ParkOutType::PARALLEL_LEFT ||
      parkout_type == ParkOutType::PARALLEL_RIGHT ||
      parkout_type == ParkOutType::PARALLEL_LEFT_FRONT ||
      parkout_type == ParkOutType::PARALLEL_RIGHT_FRONT;
  if (!is_valid_direction) {
    MSD_LOG(ERROR, "[calculate_parkout_target] invalid parkout type ");
    return false;
  }

  Pose2D target_pose;
  Pose2D error_tolerence;

  bool success = false;
  if (slot_type == ParkingSlotType::PERPENDICULAR) {
    // oblique == false means perpendicular
    success = calc_parkout_target_perpendicular_oblique(target_pose, error_tolerence, false);
  } else if (slot_type == ParkingSlotType::PARALLEL) {
    success = calc_parkout_target_parallel(target_pose, error_tolerence);
  } else if (slot_type == ParkingSlotType::OBLIQUE) {
    // oblique == true means oblique
    success = calc_parkout_target_perpendicular_oblique(target_pose, error_tolerence, true);
  } else {
    MSD_LOG(ERROR, "[calculate_parkout_target] unexpected slot type");
  }

  if (!success) {
    MSD_LOG(ERROR, "[calculate_parkout_target] failed to get parkout point");
    return false;
  }

  out_target_pose = target_pose;
  out_error_tolerence = error_tolerence;
  return true;
}

uint32_t APABehaviorCalculatorParkOut::calculate_recommended_parkout_direction(
    uint32_t available_directions) {

  const auto &g_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;

  const auto &g_slot_type = g_slot_info.type.value;

  const auto &g_last_parkin_data = PlanningContext::Instance()
                                       ->parking_behavior_planner_output()
                                       .last_parkin_data;

  // 1. Supress available parkout when invalid basic conditions unstasfied

  //    1.1. no available ---> invalid
  if (available_directions == ParkOutType::INVALID ||
      available_directions == ParkOutType::CALC_INVALID){
    return ParkOutType::CALC_INVALID;
  }

  //2.  calculate recommended pose
  const Pose2D ego_pose = world_model_->get_ego_state().ego_pose;

  planning_math::Vec2d center(0.0,0.0);
  std::vector<planning_math::Vec2d> corner_pts;
  std::vector<Point3D> corners_p3d;
  double last_theta = 0.0;

  if (msquare::CarParams::GetInstance()->car_config.apoa_config.use_last_parkin_data) {
    corner_pts = g_last_parkin_data.corner_pts;
    
    for(auto &corner_p:corner_pts){
      center += corner_p;
      Point3D p(corner_p.x(), corner_p.y(), 0.0);
      corners_p3d.push_back(p);
    }
    center /= 4.0;
    last_theta = g_last_parkin_data.last_theta;
  } else {
    corners_p3d = g_slot_info.corners;
    for(auto &corner_p:corners_p3d){
      planning_math::Vec2d p(corner_p.x, corner_p.y);
      center += p;
      corner_pts.push_back(p);
    }
    center /= 4.0;
    last_theta = ego_pose.theta;
  }
  MSD_LOG(INFO,"[CheckRecommendParkoutDirection] last_theta: %f", last_theta);

  planning_math::Vec2d left_vector = (corner_pts[0]+corner_pts[1] - corner_pts[2] - corner_pts[3]) / 2;
  planning_math::Vec2d up_vector = (corner_pts[0]+corner_pts[3] - corner_pts[2] - corner_pts[1]) / 2;
  planning_math::Vec2d up_unit = up_vector;
  up_unit.Normalize();
  left_vector.Normalize();

  double slot_width = left_vector.Length();
  double slot_height = up_vector.Length();
  double car_length = CarParams::GetInstance()->vehicle_length_real;
  double car_width = CarParams::GetInstance()->vehicle_width_real;
  double front_to_rear = VehicleParam::Instance()->front_edge_to_center;
  double back_to_rear = VehicleParam::Instance()->back_edge_to_center;
  double rear_to_center = 0.5 * std::abs(front_to_rear - back_to_rear);

  planning_math::Vec2d ego_position(ego_pose.x, ego_pose.y);
  planning_math::Vec2d heading_vector = planning_math::Vec2d::CreateUnitVec2d(ego_pose.theta);
  planning_math::Vec2d last_heading_vector =
      planning_math::Vec2d::CreateUnitVec2d(last_theta);
  MSD_LOG(INFO,"[CheckRecommendParkoutDirection] up_unit x:%f y:%f heading_vector x:%f y:%f", up_unit.x(), up_unit.y(), heading_vector.x(), heading_vector.y());
  MSD_LOG(INFO,"[CheckRecommendParkoutDirection] left_vector x:%f y:%f", left_vector.x(), left_vector.y());
  MSD_LOG(INFO,"[CheckRecommendParkoutDirection] last heading_vector x:%f y:%f", last_heading_vector.x(), last_heading_vector.y());

  uint32_t recommend_parkout_direction = ParkOutType::INVALID;
  std::vector<uint32_t> prioritized_directions;
  if (g_slot_type == ParkingSlotType::PARALLEL) {
    // park in from left
    if (left_vector.InnerProd(last_heading_vector) < 0.0) {
      prioritized_directions.push_back(ParkOutType::PARALLEL_LEFT_FRONT);
      prioritized_directions.push_back(ParkOutType::PARALLEL_LEFT);
      prioritized_directions.push_back(ParkOutType::PARALLEL_RIGHT_FRONT);
      prioritized_directions.push_back(ParkOutType::PARALLEL_RIGHT);
    } else {
      prioritized_directions.push_back(ParkOutType::PARALLEL_RIGHT_FRONT);
      prioritized_directions.push_back(ParkOutType::PARALLEL_RIGHT);
      prioritized_directions.push_back(ParkOutType::PARALLEL_LEFT_FRONT);
      prioritized_directions.push_back(ParkOutType::PARALLEL_LEFT);
    }
  } else if (g_slot_type == ParkingSlotType::PERPENDICULAR ||
             g_slot_type == ParkingSlotType::OBLIQUE) {
    if (up_unit.InnerProd(last_heading_vector) < 0.0) {
      prioritized_directions.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_REAR_RIGHT);
      prioritized_directions.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_REAR_LEFT);
      prioritized_directions.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_REAR);
      prioritized_directions.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_RIGHT);
      prioritized_directions.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_LEFT);
      prioritized_directions.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_FRONT);
    } else {
      prioritized_directions.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_REAR_RIGHT);
      prioritized_directions.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_REAR_LEFT);
      prioritized_directions.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_REAR);
      prioritized_directions.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_RIGHT);
      prioritized_directions.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_LEFT);
      prioritized_directions.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_FRONT);
    }
  }

  uint32_t recommended_direction = 0;
  // recommend the first available direction
  for(std::size_t i=0; i<prioritized_directions.size(); i++){
    if(available_directions & prioritized_directions[i]){
      recommended_direction = prioritized_directions[i];
      break;
    }
  }
  if (recommended_direction == 0) {
    recommended_direction = ParkOutType::CALC_INVALID;
  }
  
  return recommended_direction;
}

uint32_t
APABehaviorCalculatorParkOut::calculate_available_parkout_directions(
  const ParkoutSceneType &parkout_scene_res) {
  const auto &g_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;

  const auto &g_slot_type = g_slot_info.type.value;

  const auto &g_last_parkin_data = PlanningContext::Instance()
                                       ->parking_behavior_planner_output()
                                       .last_parkin_data;

  const bool use_legacy_parkout =
      CarParams::GetInstance()->car_config.apoa_config.use_legacy_parkout;

  // 1. Supress available parkout when invalid basic conditions unstasfied
  //    1.1. unexpected slot data ----> invalid
  if( g_slot_info.corners.size() != 4){
    MSD_LOG(ERROR, "[calc_available_parkout_direction] return CALC_INVALID "
                     " as corners.size() != 4");
    return ParkOutType::CALC_INVALID;
  }
  //    1.2. use_last_parkin_data ON: no last_parkin_data   ---> invalid
  if (msquare::CarParams::GetInstance()
          ->car_config.apoa_config.use_last_parkin_data) {

    if (!g_last_parkin_data.is_valid) {
      MSD_LOG(ERROR, "[calc_available_parkout_direction] return CALC_INVALID "
                     " as last_parkin_data is invalid");
      return ParkOutType::CALC_INVALID;
    }
  }
  //    1.3 support_perpendicular OFF && PARALEL
  if (!msquare::CarParams::GetInstance()
           ->car_config.apoa_config.support_perpendicular &&
      g_slot_type != ParkingSlotType::PARALLEL) {
    MSD_LOG(ERROR, "[calc_available_parkout_direction] return CALC_INVALID "
                   " as support_perpendicular is off and slot is parallel");
    return ParkOutType::CALC_INVALID;
  }

  // 2. is_direct_limit ON: direct limit reached ---> invalid
  if (msquare::CarParams::GetInstance()
          ->car_config.apoa_config.is_direct_limit) {

    if (!parkout_scene_res.is_front_collision ||
        parkout_scene_res.is_both_side_collision) {
      MSD_LOG(ERROR, "[calc_available_parkout_direction] CALC_INVALID as "
                     "isObsDirectLimit");
      return ParkOutType::CALC_INVALID;
    }
    if (is_slot_too_narrow_for_parkout(g_slot_info)) {
      MSD_LOG(ERROR, "[calc_available_parkout_direction] CALC_INVALID as "
                     "isSlotTooNarrowForParkout");
      return ParkOutType::CALC_INVALID;
    }
  }

  // 3. check for plannable directions using simple collision check
  uint32_t available_direction = 0;
  std::vector<uint32_t> open_direction;
  if (use_legacy_parkout) {
    if (g_slot_type == ParkingSlotType::PERPENDICULAR ||
        g_slot_type == ParkingSlotType::OBLIQUE) {
      open_direction.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_FRONT);
      open_direction.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_REAR);
    } else if (g_slot_type == ParkingSlotType::PARALLEL) {
      open_direction.push_back(ParkOutType::PARALLEL_LEFT);
      open_direction.push_back(ParkOutType::PARALLEL_RIGHT);
    }
  } else {
    if (g_slot_type == ParkingSlotType::PERPENDICULAR) {
      open_direction.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_FRONT);
      open_direction.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_REAR);
      open_direction.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_LEFT);
      open_direction.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_RIGHT);
      open_direction.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_REAR_LEFT);
      open_direction.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_REAR_RIGHT);
    } else if (g_slot_type == ParkingSlotType::PARALLEL) {
      open_direction.push_back(ParkOutType::PARALLEL_LEFT);
      open_direction.push_back(ParkOutType::PARALLEL_RIGHT);
      open_direction.push_back(ParkOutType::PARALLEL_LEFT_FRONT);
      open_direction.push_back(ParkOutType::PARALLEL_RIGHT_FRONT);
    } else if (g_slot_type == ParkingSlotType::OBLIQUE) {
      open_direction.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_FRONT);
      open_direction.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_REAR);
      open_direction.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_LEFT);
      open_direction.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_RIGHT);
      open_direction.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_REAR_LEFT);
      open_direction.push_back(ParkOutType::PERPENDICULAR_OBLIQUE_REAR_RIGHT);
    }
  }

  for (auto direction : open_direction) {
    if (check_parkout_direction(direction)) {
      available_direction |= direction;
    }
  }
  if (available_direction == 0) {
    available_direction = ParkOutType::CALC_INVALID;
  }
  return available_direction;
}

bool APABehaviorCalculatorParkOut::is_slot_too_narrow_for_parkout(
    const ParkingSlotInfo &parking_slot_info) {
  if (parking_slot_info.type.value != ParkingSlotType::PARALLEL) {
    return false;
  }
  double x_max_left = -10000.0;
  double x_min_right = 10000.0;
  if (!calc_two_size_length(parking_slot_info, x_min_right, x_max_left)) {
    return true;
  }

  double deta_slot = x_min_right - x_max_left;
  double slot_len_limit = VehicleParam::Instance()->length + 1.3;

  MSD_LOG(ERROR,"[%s] cal slot length: %f", __FUNCTION__, deta_slot);
  MSD_LOG(ERROR,"[%s] cal slot_len_limit: %f", __FUNCTION__, slot_len_limit);

  if ((x_min_right - x_max_left) < slot_len_limit) {
    MSD_LOG(ERROR,"[%s] is slot too narrow", __FUNCTION__);
    return true;
  }
  return false;
}

ParkoutSceneType APABehaviorCalculatorParkOut::calc_parkout_scene_type(
    const ParkingSlotInfo &parking_slot_info) {
  ParkoutSceneType parkout_scene_res;
  parkout_scene_res.is_front_collision = true;
  parkout_scene_res.is_both_side_collision = false;
  if (parking_slot_info.type.value != ParkingSlotType::PARALLEL) {
    return parkout_scene_res;
  }
  double car_length = CarParams::GetInstance()->vehicle_length_real;
  double car_width = CarParams::GetInstance()->vehicle_width_wo_rearview_mirror;
  double front_to_rear = VehicleParam::Instance()->front_edge_to_center;
  double back_to_rear = VehicleParam::Instance()->back_edge_to_center;
  double rear_to_center = 0.5 * std::abs(front_to_rear - back_to_rear);
  double front_shift_dis = 4.5;
  double side_check_region = 2.0;
  double side_shift_dis = (car_width + side_check_region) / 2.0;

  auto &corners = parking_slot_info.corners;
  double fx = 0.5 * (corners[0].x + corners[3].x);
  double fy = 0.5 * (corners[0].y + corners[3].y);
  double bx = 0.5 * (corners[1].x + corners[2].x);
  double by = 0.5 * (corners[1].y + corners[2].y);
  double cx = 0.5 * (fx + bx);
  double cy = 0.5 * (fy + by);
  double ctheta = atan2(fy - by, fx - bx);
  double ltheta = atan2(corners[0].y - corners[3].y, corners[0].x - corners[3].x);
  double slot_center_theta = ctheta - M_PI_2;
  if (slot_center_theta < -M_PI) {
    slot_center_theta += 2 * M_PI;
  }
  if (slot_center_theta > M_PI) {
    slot_center_theta -= 2 * M_PI;
  }
  Pose2D slot_center(cx, cy, slot_center_theta);

  Pose2D ego_pose = world_model_->get_ego_state().ego_pose;
  planning_math::Vec2d ego_position(ego_pose.x, ego_pose.y);
  planning_math::Vec2d ego_heading_vector = planning_math::Vec2d::CreateUnitVec2d(ego_pose.theta);
  planning_math::Vec2d heading_vector = planning_math::Vec2d::CreateUnitVec2d(ltheta);
  double inner_prod = heading_vector.InnerProd(ego_heading_vector);
  if (inner_prod < 0) {
    heading_vector = (-1) * heading_vector;
  }
  planning_math::Vec2d side_vector = heading_vector.rotate(M_PI_2);
  planning_math::Vec2d side_vector_reverse = (-1) * side_vector;

  planning_math::LineSegment2d center_axis = planning_math::LineSegment2d(
      ego_position - back_to_rear * heading_vector,
      ego_position + front_to_rear * heading_vector);
  planning_math::Box2d check_box = planning_math::Box2d(center_axis, car_width);
  planning_math::Box2d check_box_side = planning_math::Box2d(center_axis, side_check_region);
  planning_math::Box2d check_box_side_reverse = planning_math::Box2d(center_axis, side_check_region);

  check_box.Shift(front_shift_dis * heading_vector);
  check_box_side.Shift(side_shift_dis * side_vector);
  check_box_side_reverse.Shift(side_shift_dis * side_vector_reverse);
  planning_math::Vec2d ego_pose_2d{ego_pose.x, ego_pose.y};
  planning_math::Vec2d ego_pose_local = tf2d(slot_center, ego_pose_2d);
  double obs_bottom_limit = ego_pose_local.y() - car_width / 2.0 + 0.3;
  MSD_LOG(ERROR,"[%s] obs_bottom_limit: %f", __FUNCTION__, obs_bottom_limit);
  MSD_LOG(ERROR,"[%s] slot_center_theta: %f", __FUNCTION__, slot_center_theta);
  MSD_LOG(ERROR,"[%s] car_width: %f", __FUNCTION__, car_width);

  bool is_collision = false;
  bool is_collision_side = false;
  bool is_collision_side_reverse = false;
  bool is_collision_side_both = false;
  auto &obstacles = world_model_->obstacle_manager().get_obstacles().Items();
  for (const auto &obstacle : obstacles) {
    if (obstacle->IsStatic() == 1 && obstacle->Type() != ObjectType::PEDESTRIAN) {
      if (!is_collision) {
        if (check_box.HasOverlap(obstacle->PerceptionBoundingBox())) {
          is_collision = true;
          MSD_LOG(ERROR,"[%s] front collide with PerceptionBoundingBox", __FUNCTION__);
        }
      }
      if (!is_collision_side) {
        if (check_box_side.HasOverlap(obstacle->PerceptionBoundingBox())) {
          is_collision_side = true;
          MSD_LOG(ERROR,"[%s] side collide with PerceptionBoundingBox", __FUNCTION__);
        }
      }
      if (!is_collision_side_reverse) {
        if (check_box_side_reverse.HasOverlap(obstacle->PerceptionBoundingBox())) {
          is_collision_side_reverse = true;
          MSD_LOG(ERROR,"[%s] side reverse collide with PerceptionBoundingBox", __FUNCTION__);
        }
      }
    }
  }

  if (is_collision_side && is_collision_side_reverse) {
    MSD_LOG(ERROR,"[%s] side both collide with PerceptionBoundingBox", __FUNCTION__);
    is_collision_side_both = true;
  }

  for (auto &obs : world_model_->get_parking_ground_line_fusion()) {
    if(is_collision && is_collision_side_both){
      break;
    }

    if (obs.id != parking_slot_info.id) {
      continue;
    }

    for (int i = 0; i < obs.pts.size(); i++) {
      planning_math::Vec2d uss_point{obs.pts[i].x, obs.pts[i].y};
      if (!is_collision) {
        planning_math::Vec2d uss_point_local = tf2d(slot_center, uss_point);
        if (uss_point_local.y() >= obs_bottom_limit) {
          if (check_box.IsPointIn(uss_point)) {
            MSD_LOG(ERROR,"[%s] front collide with uss points", __FUNCTION__);
            is_collision = true;
          }
        }
      }
      if (!is_collision_side) {
        if (check_box_side.IsPointIn(uss_point)) {
          MSD_LOG(ERROR,"[%s] side collide with uss points", __FUNCTION__);
          is_collision_side = true;
        }
      }
      if (!is_collision_side_reverse) {
        if (check_box_side_reverse.IsPointIn(uss_point)) {
          MSD_LOG(ERROR,"[%s] side reverse collide with uss points", __FUNCTION__);
          is_collision_side_reverse = true;
        }
      }
    }
    if (is_collision_side && is_collision_side_reverse) {
      is_collision_side_both = true;
    }
    if(is_collision && is_collision_side_both){
      break;
    }

    if (!(obs.type == GroundLineType::GROUND_LINE_USS_TYPE_UNKNOWN ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_WALL ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_PILLAR ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_FENCE ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_STEP ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_SPECIAL ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_VEHICLE ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_ONLY_USS_UNKNOWN)) {
      continue;
    }
    if (obs.pts.size() % 2 != 0) {
      continue;
    }

    for (unsigned int i = 0; i < obs.pts.size(); i = i + 2) {
      planning_math::LineSegment2d uss_ground_line{
          planning_math::Vec2d{obs.pts[i].x, obs.pts[i].y},
          planning_math::Vec2d{obs.pts[i + 1].x, obs.pts[i + 1].y}};
      if (!is_collision_side) {
        if (check_box_side.HasOverlap(uss_ground_line)) {
          MSD_LOG(ERROR,"[%s] side collide with obstacle line", __FUNCTION__);
          is_collision_side = true;
        }
      }
      if (!is_collision_side_reverse) {
        if (check_box_side_reverse.HasOverlap(uss_ground_line)) {
          MSD_LOG(ERROR,"[%s] side reverse collide with obstacle line", __FUNCTION__);
          is_collision_side_reverse = true;
        }
      }
    }
  }

  MSD_LOG(ERROR,"[%s] is_collision: %d", __FUNCTION__, is_collision);
  MSD_LOG(ERROR,"[%s] is_collision_side: %d", __FUNCTION__, is_collision_side);
  MSD_LOG(ERROR,"[%s] is_collision_side_reverse: %d", __FUNCTION__, is_collision_side_reverse);

  if (is_collision_side && is_collision_side_reverse) {
    MSD_LOG(ERROR,"[%s] side both collide with ground_line_fusion", __FUNCTION__);
    is_collision_side_both = true;
  }
  if (!is_collision) { // front 4.5m have obs, do not release parkout direct
    MSD_LOG(ERROR,"[%s] return as front have no obs", __FUNCTION__);
    parkout_scene_res.is_front_collision = false;
  }
  if (is_collision_side_both) {
    MSD_LOG(ERROR,"[%s] return as side both have obs", __FUNCTION__);
    parkout_scene_res.is_both_side_collision = true;
  }
  return parkout_scene_res;
}

bool APABehaviorCalculatorParkOut::calc_two_size_length(
    const ParkingSlotInfo &parking_slot_info, double &x_min_right,
    double &x_max_left) {
  auto &corners = parking_slot_info.corners;
  if (corners.size() != 4) {
    return false;
  }
  double fx = 0.5 * (corners[0].x + corners[3].x);
  double fy = 0.5 * (corners[0].y + corners[3].y);
  double bx = 0.5 * (corners[1].x + corners[2].x);
  double by = 0.5 * (corners[1].y + corners[2].y);
  double cx = 0.5 * (fx + bx);
  double cy = 0.5 * (fy + by);
  double ctheta = atan2(fy - by, fx - bx);
  double lot_length = std::hypot(fy - by, fx - bx);
  double lot_width =
      std::hypot(corners[0].x - corners[3].x, corners[0].y - corners[3].y);

  planning_math::Box2d slot_box = planning_math::Box2d(
      planning_math::Vec2d(cx, cy),
      ctheta, lot_length, lot_width);
  planning_math::Box2d slot_left(slot_box);
  planning_math::Box2d slot_right(slot_box);

  planning_math::Vec2d left_vector(corners[0].x - corners[3].x, corners[0].y - corners[3].y);
  slot_left.Shift(left_vector);
  slot_right.Shift(-1.0 * left_vector);
  double slot_center_theta = ctheta - M_PI_2;
  if (slot_center_theta < -M_PI) {
    slot_center_theta += 2 * M_PI;
  }
  if (slot_center_theta > M_PI) {
    slot_center_theta -= 2 * M_PI;
  }
  Pose2D slot_center(cx, cy, slot_center_theta);
  planning_math::Vec2d slot_bottom_middle_point{bx, by};
  planning_math::Vec2d slot_bottom_middle_point_local = tf2d(slot_center, slot_bottom_middle_point);
  Pose2D ego_pose = world_model_->get_ego_state().ego_pose;
  double car_width = CarParams::GetInstance()->vehicle_width_wo_rearview_mirror;
  planning_math::Vec2d ego_pose_2d{ego_pose.x, ego_pose.y};
  planning_math::Vec2d ego_pose_local = tf2d(slot_center, ego_pose_2d);

  double obs_bottom_limit = ego_pose_local.y() - car_width / 2.0 + 0.3;
  // double obs_bottom_limit = slot_bottom_middle_point_local.y() + 0.25;
  MSD_LOG(ERROR,"[%s] obs_bottom_limit: %f", __FUNCTION__, obs_bottom_limit);
  MSD_LOG(ERROR,"[%s] slot_center_theta: %f", __FUNCTION__, slot_center_theta);
  MSD_LOG(ERROR,"[%s] car_width: %f", __FUNCTION__, car_width);

  for (auto &obs : world_model_->get_parking_ground_line_fusion()) {
    if ((obs.type < GroundLineType::GROUND_LINE_USS_TYPE_POINT && 
        obs.type >= GroundLineType::GROUND_LINE_USS_TYPE_UNKNOWN) ||
        obs.id != parking_slot_info.id) {
      continue;
    }
    if (obs.type == GroundLineType::GROUND_LINE_TYPE_USS_REALTIME_OBSTACLE) {
      continue;
    }
    for (int i = 0; i < obs.pts.size(); i++) {
      planning_math::Vec2d uss_point{obs.pts[i].x, obs.pts[i].y};
      planning_math::Vec2d uss_point_local = tf2d(slot_center, uss_point);
      if (slot_left.IsPointIn(uss_point)) {
        if (uss_point_local.x() > x_max_left && uss_point_local.y() >= obs_bottom_limit) {
          x_max_left = uss_point_local.x();
        }
      } 
      if (slot_right.IsPointIn(uss_point)) {
        if (uss_point_local.x() < x_min_right && uss_point_local.y() >= obs_bottom_limit) {
          x_min_right = uss_point_local.x();
        }
      }
    }
  }
  return true;
}


} // namespace parking
} // namespace msquare
