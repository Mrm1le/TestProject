#include <cstdint>

#include "common/math/math_utils.h"
#include "common/parking_slot_manager.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planning/common/common.h"

namespace msquare {

namespace parking {

namespace {
const double kNarrowestLegalSlotWidth =
    CarParams::GetInstance()
        ->car_config.target_pose_config
        .twosides_narrowest_width_advanced_abandon +
    0.03;
const double kMirrorToPillarDist =
    CarParams::GetInstance()
        ->car_config.target_pose_config
        .oneside_dist_mirror_to_pillar; // client's request. TODO(shaw)
} // namespace

ParkingSlotManager::ParkingSlotManager() { world_model_ = nullptr; }

ParkingSlotManager::ParkingSlotManager(
    const std::shared_ptr<WorldModel> &world_model) {
  world_model_ = world_model;
  const CarParams *car_params = CarParams::GetInstance();
  perpendicular_parking_slot_width_ = VehicleParam::Instance()->width + 0.6;
  parallel_parking_slot_width_ = car_params->vehicle_length_real + 0.7;
  default_distance_apa_ = car_params->wheel_base /
                              std::tan(car_params->get_max_steer_angle() /
                                       car_params->steer_ratio / 180.0 * M_PI) +
                          HybridAstarConfig::GetInstance()->step_size;
  apf_decider_ = std::make_shared<ApfDecider>(world_model_);
  apf_decider_->set_traj_tag("normal");
  body_tire_offset_ =
      CarParams::GetInstance()->car_config.car_only_config.body_tire_offset;
  body_tire_offset_space_slot_ =
      CarParams::GetInstance()
          ->car_config.car_only_config.body_tire_offset_space_slot;
}

bool ParkingSlotManager::UpdateTargetParkingSlotInfo() {
  if (world_model_ == nullptr) {
    return false;
  }
  auto &parking_slot_info = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->parking_slot_info;
  if (world_model_->get_parking_map_info().aimed_poi_info.type ==
      "PARKING_LOT") {
    parking_slot_info.id =
        world_model_->get_parking_map_info().aimed_poi_info.id;
    parking_slot_info.corners =
        world_model_->get_parking_map_info().aimed_poi_info.corners;
    parking_slot_info.wheel_stop_info.available = false;
    return OptimizeParkingSlotCorners(parking_slot_info);
  }
  return false;
}

bool ParkingSlotManager::UpdateEgoParkingSlotInfo() {
  if (world_model_ == nullptr) {
    return false;
  }
  const ParkingMapInfo &parking_map_info = world_model_->get_parking_map_info();
  std::vector<ParkingLotDetectionInfo> parking_lots_detection_fusion_results =
      parking_map_info.parking_lots_detection_fusion_results;
  for (auto &parking_lot : parking_lots_detection_fusion_results) {
    if (parking_lot.is_car_in) {
      return UpdateParkingSlotInfo(parking_lot.id);
    }
  }

  return UpdateParkingSlotInfo(
      world_model_->get_map_info().current_parking_lot_id);
}

bool ParkingSlotManager::UpdateParkingSlotInfo(
    const ParkingLotDetectionInfo &parking_lot_info) {
  auto &parking_slot_info = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->parking_slot_info;
  parking_slot_info.id = parking_lot_info.id;
  std::vector<Point3D> corners;
  for (ParkingLotDetectionInfo::CornerPoint corner : parking_lot_info.corners) {
    corners.emplace_back(corner.position);
  }
  parking_slot_info.corners = corners;
  parking_slot_info.wheel_stop_info.vision_wheel_stop_available =
      parking_lot_info.wheel_stop.available;
  parking_slot_info.wheel_stop_info.available =
      parking_lot_info.wheel_stop.available;
  parking_slot_info.wheel_stop_info.vision_point1 =
      parking_lot_info.wheel_stop.point1;
  parking_slot_info.wheel_stop_info.vision_point2 =
      parking_lot_info.wheel_stop.point2;
  parking_slot_info.virtual_corner.slot_type =
      parking_lot_info.virtual_corner.slot_type;
  parking_slot_info.virtual_corner.index =
      parking_lot_info.virtual_corner.index;
  parking_slot_info.virtual_corner.p = parking_lot_info.virtual_corner.p;

  return OptimizeParkingSlotCorners(parking_slot_info);
}

bool ParkingSlotManager::UpdateParkingSlotInfo(const int id) {
  MSD_LOG(ERROR, "no_react UpdateParkingSlotInfo run ...");
  if (world_model_ == nullptr) {
    MSD_LOG(ERROR,
            "no_react UpdateParkingSlotInfo failed for null world model");
    return false;
  }
  const ParkingMapInfo &parking_map_info = world_model_->get_parking_map_info();
  std::vector<ParkingLotDetectionInfo> parking_lots_detection_fusion_results =
      parking_map_info.parking_lots_detection_fusion_results;
  if (parking_lots_detection_fusion_results.size() > 0) {
    // Select parking spot from accurate search result
    for (auto &parking_lot : parking_lots_detection_fusion_results) {
      if (parking_lot.id == id) {
        return UpdateParkingSlotInfo(parking_lot);
      }
    }
  }

  auto &parking_slot_info = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->parking_slot_info;
  if (parking_slot_info.id == id) {
    return OptimizeParkingSlotCorners(parking_slot_info);
  }
  MSD_LOG(ERROR, "no_react UpdateParkingSlotInfo failed!");
  return false;
}

bool ParkingSlotManager::UpdateWheelStopPoint(const bool collision) {
  auto &parking_slot_info = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->parking_slot_info;
  if (world_model_ == nullptr) {
    return false;
  }
  const double kWheelStopBuffer =
      CarParams::GetInstance()
          ->car_config.target_pose_config.rear_axis_to_stopper;
  if (collision) {
    parking_slot_info.wheel_stop_info.collision_wheel_stop_available = true;
    auto &ego_pose = world_model_->get_ego_state().ego_pose;
    auto wheel_point1 = Point3D(
        ego_pose.x +
            std::sin(ego_pose.theta) *
                CarParams::GetInstance()->vehicle_width_wo_rearview_mirror,
        ego_pose.y -
            std::cos(ego_pose.theta) *
                CarParams::GetInstance()->vehicle_width_wo_rearview_mirror,
        0.0);
    auto wheel_point2 = Point3D(
        ego_pose.x -
            std::sin(ego_pose.theta) *
                CarParams::GetInstance()->vehicle_width_wo_rearview_mirror,
        ego_pose.y +
            std::cos(ego_pose.theta) *
                CarParams::GetInstance()->vehicle_width_wo_rearview_mirror,
        0.0);
    parking_slot_info.wheel_stop_info.collision_point1 = wheel_point1;
    parking_slot_info.wheel_stop_info.collision_point2 = wheel_point2;
  }
  if (parking_slot_info.wheel_stop_info.collision_wheel_stop_available) {
    auto &corners = parking_slot_info.corners;

    if (corners.size() != 4) {
      parking_slot_info.wheel_stop_info.collision_wheel_stop_available = false;
      return false;
    }
    parking_slot_info.wheel_stop_info.available = true;
    planning_math::Vec2d left_front{corners[0].x, corners[0].y};
    planning_math::Vec2d left_rear{corners[1].x, corners[1].y};
    planning_math::Vec2d right_rear{corners[2].x, corners[2].y};
    planning_math::Vec2d right_front{corners[3].x, corners[3].y};

    planning_math::Vec2d wheel_stop;
    planning_math::LineSegment2d long_line_seg;
    if (parking_slot_info.type.value != ParkingSlotType::PARALLEL) {
      long_line_seg = LineSegment2d((left_rear + right_rear) / 2.0,
                                    (left_front + right_front) / 2.0);
    } else {
      auto &ego_pose = world_model_->get_ego_state().ego_pose;
      planning_math::Vec2d ego_direction =
          planning_math::Vec2d::CreateUnitVec2d(ego_pose.theta);
      if (ego_direction.InnerProd(left_front - right_front) > 0.0) {
        long_line_seg = LineSegment2d((right_front + right_rear) / 2.0,
                                      (left_front + left_rear) / 2.0);
      } else {
        long_line_seg = LineSegment2d((left_front + left_rear) / 2.0,
                                      (right_front + right_rear) / 2.0);
      }
    }
    planning_math::Line2d long_line{long_line_seg};
    planning_math::Vec2d point1{
        parking_slot_info.wheel_stop_info.collision_point1.x,
        parking_slot_info.wheel_stop_info.collision_point1.y};
    planning_math::Vec2d point2{
        parking_slot_info.wheel_stop_info.collision_point2.x,
        parking_slot_info.wheel_stop_info.collision_point2.y};
    if (long_line.unit_direction().InnerProd(point1 - point2) < 0.0) {
      long_line.GetPerpendicularFoot(point1, &wheel_stop);
    } else {
      long_line.GetPerpendicularFoot(point2, &wheel_stop);
    }
    double wheel_stop_depth =
        wheel_stop.DistanceTo(long_line_seg.end()) + kWheelStopBuffer;
    if (parking_slot_info.wheel_stop_info.vision_wheel_stop_available) {
      parking_slot_info.wheel_stop_info.wheel_stop_depth = std::min(
          wheel_stop_depth, parking_slot_info.wheel_stop_info.wheel_stop_depth);
    } else {
      parking_slot_info.wheel_stop_info.wheel_stop_depth = wheel_stop_depth;
    }
  }

  return true;
}

bool ParkingSlotManager::ClearParkingSlotInfo() {
  auto &parking_slot_info = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->parking_slot_info;
  ParkingSlotInfo tmp_info{};
  parking_slot_info = tmp_info;
  return true;
}

bool ParkingSlotManager::ClearParkingSlotKeyPoint() {
  auto &parking_slot_info = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->parking_slot_info;
  parking_slot_info.park_in_point = KeyPoint();
  parking_slot_info.park_out_point = KeyPoint();
  parking_slot_info.wheel_stop_info.collision_wheel_stop_available = false;
  return true;
}

bool ParkingSlotManager::OptimizeParkingSlotCorners(
    ParkingSlotInfo &parking_slot_info) {
  auto &corners = parking_slot_info.corners;
  parking_slot_info.original_corners = corners;
  parking_slot_info.left_empty = true;
  parking_slot_info.right_empty = true;
  parking_slot_info.nearby_obstacle_id.clear();

  if (corners.size() != 4 || world_model_ == nullptr) {
    return false;
  }
  parking_slot_info.type.value =
      GetParkingSlotType(parking_slot_info.virtual_corner.slot_type);
  double parking_slot_width;

  planning_math::Vec2d left_front{corners[0].x, corners[0].y};
  planning_math::Vec2d left_rear{corners[1].x, corners[1].y};
  planning_math::Vec2d right_rear{corners[2].x, corners[2].y};
  planning_math::Vec2d right_front{corners[3].x, corners[3].y};
  planning_math::Vec2d front_center, rear_center;
  planning_math::Line2d bisector;
  MSD_LOG(ERROR, "[OptimizeParkingSlotCorners] slot type:%u",
          parking_slot_info.type.value);
  MSD_LOG(ERROR, "[OptimizeParkingSlotCorners] fusion slot type:%d",
          parking_slot_info.virtual_corner.slot_type);

  MSD_LOG(ERROR, "[OptimizeParkingSlotCorners] slot type:%u",
          parking_slot_info.type.value);
  MSD_LOG(ERROR, "[OptimizeParkingSlotCorners] fusion slot type:%d",
          parking_slot_info.virtual_corner.slot_type);

  if (parking_slot_info.type.value != ParkingSlotType::PARALLEL) {
    parking_slot_width = perpendicular_parking_slot_width_;
    planning_math::Line2d left_line{left_rear, left_front};
    planning_math::Line2d right_line{right_rear, right_front};
    if (!left_line.GetBisector(right_line, &bisector)) {
      return false;
    }

    planning_math::Vec2d bisector_unit = bisector.unit_direction();
    if (parking_slot_info.type.value == ParkingSlotType::OBLIQUE) {
      front_center = (left_front + right_front) / 2.0;
      planning_math::Vec2d front_direc = left_front - right_front;
      front_direc.Normalize();
      double slot_cos = std::abs(bisector_unit.InnerProd(front_direc));
      double slot_sin = std::abs(bisector_unit.CrossProd(front_direc));
      double move_depth = 1.5;
      if (slot_sin > 0.34) { // sin(pi/9) = 0.3420
        move_depth = std::min(VehicleParam::Instance()->bumper_length / 2.0 *
                                  slot_cos / slot_sin,
                              move_depth);
      }
      // const ParkingSlotInfo::VirtualCorner& virtual_corner =
      // parking_slot_info.virtual_corner; if(virtual_corner.slot_type != 4){
      //   front_center -= bisector_unit*move_depth;
      // }
      // MSD_LOG(INFO, "[OptimizeParkingSlotCorners] oblique type:%d",
      // virtual_corner.slot_type);
      front_center -= bisector_unit * move_depth;
      MSD_LOG(INFO,
              "[OptimizeParkingSlotCorners] slot_sin:%.4f slot_cos:%.4f "
              "move_depth:%.4f",
              slot_sin, slot_cos, move_depth);
    } else {
      if (bisector_unit.InnerProd(left_front - right_front) > 0.0) {
        bisector.GetPerpendicularFoot(right_front, &front_center);
      } else {
        bisector.GetPerpendicularFoot(left_front, &front_center);
      }
    }
    rear_center =
        front_center -
        bisector_unit * (CarParams::GetInstance()->vehicle_length_real + 0.4);
  } else {
    parking_slot_width = parallel_parking_slot_width_;
    front_center = (left_front + right_front) / 2.0;
    planning_math::Vec2d direction_vector =
        (right_front - left_front).rotate(M_PI_2);
    direction_vector.Normalize();
    if (direction_vector.InnerProd(left_front - left_rear) < 0.0) {
      direction_vector.SelfRotate(M_PI);
    }
    double direction = direction_vector.Angle();
    bisector = planning_math::Line2d(front_center, direction);
    front_center -= direction_vector * 0.05;
    rear_center =
        front_center -
        direction_vector *
            (CarParams::GetInstance()->vehicle_width_wo_rearview_mirror +
             2 * parallel_parking_slot_offset_);
  }

  planning_math::LineSegment2d center_line{rear_center, front_center};
  parking_slot_info.center_line = center_line;

  planning_math::Vec2d left_shift_vec =
      bisector.unit_direction().rotate(M_PI_2);
  planning_math::LineSegment2d left_center_line{
      rear_center + left_shift_vec * parking_slot_width / 2.0,
      front_center + left_shift_vec * parking_slot_width / 2.0};
  planning_math::LineSegment2d right_center_line{
      rear_center - left_shift_vec * parking_slot_width / 2.0,
      front_center - left_shift_vec * parking_slot_width / 2.0};

  // TODO: @tianbo refine left & right box
  planning_math::Box2d left_box{left_center_line, parking_slot_width};
  planning_math::Box2d right_box{right_center_line, parking_slot_width};

  if (parking_slot_info.type.value != ParkingSlotType::PARALLEL) {
    const double kSlotComp = 1.5;
    double box_length =
        std::min(VehicleParam::Instance()->length, center_line.length()) * 0.8;
    left_box.SetLength(box_length + kSlotComp);
    right_box.SetLength(box_length + kSlotComp);
    left_box.Shift(bisector.unit_direction() *
                   (center_line.length() - box_length + kSlotComp) / 2.0);
    right_box.Shift(bisector.unit_direction() *
                    (center_line.length() - box_length + kSlotComp) / 2.0);
  } else {
    double box_length =
        std::min(VehicleParam::Instance()->width, center_line.length()) * 0.9;
    left_box.SetLength(box_length);
    right_box.SetLength(box_length);
    left_box.Shift(bisector.unit_direction() *
                   (center_line.length() - box_length) / 2.0);
    right_box.Shift(bisector.unit_direction() *
                    (center_line.length() - box_length) / 2.0);
  }

  const ObstacleManager &obstacle_manager = world_model_->obstacle_manager();

  double left_distance = std::numeric_limits<double>::max();
  double right_distance = std::numeric_limits<double>::max();
  double slot_width_origin =
      center_line.DistanceTo((left_rear + left_front) / 2.0) +
      center_line.DistanceTo((right_rear + right_front) / 2.0);

  // static obstacle
  // for (auto &obs : obstacle_manager.get_static_obstacles().Items()) {
  //   if (obs->PerceptionBoundingBox().HasOverlap(left_box)) {
  //     parking_slot_info.left_empty = false;
  //     parking_slot_info.nearby_obstacle_id.emplace_back(obs->Id());
  //     left_distance = std::min(
  //         obs->PerceptionBoundingBox().DistanceTo(center_line),
  //         left_distance);
  //   } else if (obs->PerceptionBoundingBox().HasOverlap(right_box)) {
  //     parking_slot_info.right_empty = false;
  //     parking_slot_info.nearby_obstacle_id.emplace_back(obs->Id());
  //     right_distance = std::min(
  //         obs->PerceptionBoundingBox().DistanceTo(center_line),
  //         right_distance);
  //   }
  // }

  // groundline point
  // double left_threshold =
  //     std::min(left_distance, std::max(parking_slot_width - right_distance,
  //                                      parking_slot_width / 2.0));

  // double right_threshold =
  //     std::min(right_distance, std::max(parking_slot_width - left_distance,
  //                                       parking_slot_width / 2.0));

  // for (auto &obs : obstacle_manager.get_points().Items()) {
  //   if (left_box.IsPointIn(obs->PerceptionBoundingBox().center())) {
  //     left_distance = std::min(slot_width_origin / 2.0, left_distance);
  //     right_distance = std::min(slot_width_origin / 2.0, right_distance);
  //     parking_slot_info.left_empty = false;
  //     // break;
  //   } else if (right_box.IsPointIn(obs->PerceptionBoundingBox().center())) {
  //     left_distance = std::min(slot_width_origin / 2.0, left_distance);
  //     right_distance = std::min(slot_width_origin / 2.0, right_distance);
  //     parking_slot_info.right_empty = false;
  //     // break;
  //   }
  // }
  std::pair<SlotClosestSideType, SlotClosestSideType> slot_two_sides_type{
      SlotClosestSideType::NONE_OBSTACLE, SlotClosestSideType::NONE_OBSTACLE};

  for (auto &obs : world_model_->get_parking_ground_line_fusion()) {
    if (obs.type <= GroundLineType::GROUND_LINE_USS_TYPE_POINT ||
        obs.type > GroundLineType::GROUND_LINE_USS_TYPE_SIDE_POINT_SPECIAL ||
        obs.id != parking_slot_info.id) {
      continue;
    }
    for (int i = 0; i < obs.pts.size(); i++) {
      planning_math::Vec2d uss_point{obs.pts[i].x, obs.pts[i].y};

      if (left_box.IsPointIn(uss_point)) {
        parking_slot_info.left_empty = false;
        double distance_to_obstacle = bisector.DistanceTo(uss_point);
        if (distance_to_obstacle < left_distance) {
          left_distance = distance_to_obstacle;
          if (obs.type == GroundLineType::GROUND_LINE_USS_TYPE_SIDE_POINT_STEP)
            slot_two_sides_type.first = SlotClosestSideType::SIDE_SLOT;
          else if (obs.type ==
                   GroundLineType::GROUND_LINE_USS_TYPE_SIDE_POINT_VEHICLE)
            slot_two_sides_type.first = SlotClosestSideType::SIDE_CAR_OBSTACLE;
          else
            slot_two_sides_type.first = SlotClosestSideType::SIDE_HIGH_OBSTACLE;
        }
      } else if (right_box.IsPointIn(uss_point)) {
        parking_slot_info.right_empty = false;
        double distance_to_obstacle = bisector.DistanceTo(uss_point);
        if (distance_to_obstacle < right_distance) {
          right_distance = distance_to_obstacle;
          if (obs.type == GroundLineType::GROUND_LINE_USS_TYPE_SIDE_POINT_STEP)
            slot_two_sides_type.second = SlotClosestSideType::SIDE_SLOT;
          else if (obs.type ==
                   GroundLineType::GROUND_LINE_USS_TYPE_SIDE_POINT_VEHICLE)
            slot_two_sides_type.second = SlotClosestSideType::SIDE_CAR_OBSTACLE;
          else
            slot_two_sides_type.second =
                SlotClosestSideType::SIDE_HIGH_OBSTACLE;
        }
      }
    }
  }

  double max_left_distance = std::min(left_distance, parking_slot_width);
  double max_right_distance = std::min(right_distance, parking_slot_width);

  double left_obstacle_to_origin_slot_center = left_distance;
  double right_obstacle_to_origin_slot_center = right_distance;
  left_distance =
      std::min(left_distance, std::max(parking_slot_width - right_distance,
                                       parking_slot_width / 2.0));

  right_distance =
      std::min(right_distance, std::max(parking_slot_width - left_distance,
                                        parking_slot_width / 2.0));

  double parking_slot_depth = center_line.length();
  planning_math::LineSegment2d rear_line_seg = planning_math::LineSegment2d(
      rear_center + left_shift_vec * left_distance,
      rear_center - left_shift_vec * right_distance);

  planning_math::LineSegment2d front_line_seg = planning_math::LineSegment2d(
      front_center + left_shift_vec * left_distance,
      front_center - left_shift_vec * right_distance);

  planning_math::Line2d front_line =
      planning_math::Line2d(front_center + left_shift_vec * left_distance,
                            front_center - left_shift_vec * right_distance);

  // for (auto &obs : obstacle_manager.get_static_obstacles().Items()) {
  //   if (obs->PerceptionBoundingBox().HasOverlap(rear_line_seg)) {
  //     parking_slot_depth =
  //         std::min(obs->PerceptionBoundingBox().DistanceTo(front_line_seg),
  //                  parking_slot_depth);
  //   }
  // }

  planning_math::Box2d rear_box{rear_line_seg, parking_slot_depth};
  const double kSafetyThreshold = 0.2;

  // uss groundlines
  double parking_slot_bottom_line = center_line.length();
  bool enable_deviation = false;
  for (auto &obs : world_model_->get_parking_ground_line_fusion()) {
    if (!(obs.type == GroundLineType::GROUND_LINE_USS_TYPE_UNKNOWN ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_WALL ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_PILLAR ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_FENCE ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_STEP ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_SPECIAL ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_VEHICLE ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_ONLY_USS_UNKNOWN) ||
        obs.id != parking_slot_info.id) {
      continue;
    }

    if (parking_slot_info.type.value == ParkingSlotType::PARALLEL) {
      if (obs.pts.size() % 2 != 0) {
        continue;
      }

      double safety_threshold = kSafetyThreshold;
      if (obs.type == GroundLineType::GROUND_LINE_USS_TYPE_STEP) {
        if (parking_slot_info.is_space_slot) {
          safety_threshold = 0.05 - body_tire_offset_space_slot_;
        } else {
          safety_threshold = 0.05 - body_tire_offset_;
        }
      }
      for (int i = 0; i < obs.pts.size(); i = i + 2) {
        planning_math::LineSegment2d uss_ground_line{
            planning_math::Vec2d{obs.pts[i].x, obs.pts[i].y},
            planning_math::Vec2d{obs.pts[i + 1].x, obs.pts[i + 1].y}};
        if (rear_box.HasOverlap(uss_ground_line) &&
            !front_line.HasIntersect(uss_ground_line)) {
          enable_deviation = true;
          parking_slot_bottom_line = std::min(
              front_line.DistanceTo(uss_ground_line.start()) - safety_threshold,
              std::min(front_line.DistanceTo(uss_ground_line.end()) -
                           safety_threshold,
                       parking_slot_bottom_line));
        }
      }
    } else if (parking_slot_info.type.value == ParkingSlotType::PERPENDICULAR &&
               !parking_slot_info.is_space_slot) { // lined perpendicular slot
      enable_deviation = false;
    } else { // except parallel slot
      double safety_threshold = kSafetyThreshold;
      if (obs.type == GroundLineType::GROUND_LINE_USS_TYPE_STEP) {
        safety_threshold = 0.05;
      }
      for (int i = 0; i < obs.pts.size(); i++) {
        planning_math::Vec2d obs_p{obs.pts[i].x, obs.pts[i].y};
        if (!rear_box.IsPointIn(obs_p)) {
          continue;
        }
        enable_deviation = true;
        parking_slot_bottom_line =
            std::min(front_line.DistanceTo(obs_p) - safety_threshold,
                     parking_slot_bottom_line);
      } // end obs loop
    }   // end except parallel slot
  }
  if (!enable_deviation) {
    parking_slot_bottom_line = center_line.length();
  }

  parking_slot_depth =
      std::min(parking_slot_bottom_line,
               std::max(parking_slot_depth, center_line.length() / 2.0));

  double rear_distance = center_line.length() - parking_slot_depth;

  if (parking_slot_info.type.value != ParkingSlotType::PARALLEL) {
    center_line = planning_math::LineSegment2d(
        rear_center + left_shift_vec * (left_distance - right_distance) / 2.0 +
            bisector.unit_direction() * rear_distance,
        front_center + left_shift_vec * (left_distance - right_distance) / 2.0);
  } else {
    center_line = planning_math::LineSegment2d(
        rear_center + left_shift_vec * (left_distance - right_distance) / 2.0 +
            bisector.unit_direction() * rear_distance,
        front_center + left_shift_vec * (left_distance - right_distance) / 2.0 +
            bisector.unit_direction() * rear_distance);
  }

  std::vector<double> slot_dis = {left_distance, right_distance,
                                  max_left_distance, max_right_distance};
  bool is_adjusted =
      refactorCenterLines(center_line, parking_slot_info, slot_dis);
  double optimized_slot_width = (is_adjusted && slot_dis.size() == 4)
                                    ? slot_dis[0] + slot_dis[1]
                                    : left_distance + right_distance;
  planning_math::Box2d optimized_slot_box{
      center_line,
      std::min(optimized_slot_width,
               parking_slot_width / parking_slot_inflation_ratio_)};

  auto optimized_corners = optimized_slot_box.GetAllCorners();
  optimized_corners.emplace_back(optimized_corners.front());
  optimized_corners.erase(optimized_corners.begin());

  for (int i = 0; i < optimized_corners.size(); i++) {
    // std::cout << "corner: x " << corners[i].x << " y " << corners[i].y
    //           << std::endl;
    corners[i].x = optimized_corners[i].x();
    corners[i].y = optimized_corners[i].y();
    // std::cout << "optimized_corners: x " << corners[i].x << " y "
    //           << corners[i].y << std::endl;
  }

  if (parking_slot_info.wheel_stop_info.vision_wheel_stop_available) {
    planning_math::Vec2d wheel_stop;
    planning_math::LineSegment2d long_line_seg;
    if (parking_slot_info.type.value != ParkingSlotType::PARALLEL) {
      long_line_seg =
          LineSegment2d((optimized_corners[1] + optimized_corners[2]) / 2.0,
                        (optimized_corners[0] + optimized_corners[3]) / 2.0);
    } else {
      auto &ego_pose = world_model_->get_ego_state().ego_pose;
      planning_math::Vec2d ego_direction =
          planning_math::Vec2d::CreateUnitVec2d(ego_pose.theta);
      if (ego_direction.InnerProd(optimized_corners[0] - optimized_corners[3]) >
          0.0) {
        long_line_seg =
            LineSegment2d((optimized_corners[2] + optimized_corners[3]) / 2.0,
                          (optimized_corners[0] + optimized_corners[1]) / 2.0);
      } else {
        long_line_seg =
            LineSegment2d((optimized_corners[0] + optimized_corners[1]) / 2.0,
                          (optimized_corners[2] + optimized_corners[3]) / 2.0);
      }
    }
    planning_math::Line2d long_line{long_line_seg};
    planning_math::Vec2d point1{
        parking_slot_info.wheel_stop_info.vision_point1.x,
        parking_slot_info.wheel_stop_info.vision_point1.y};
    planning_math::Vec2d point2{
        parking_slot_info.wheel_stop_info.vision_point1.x,
        parking_slot_info.wheel_stop_info.vision_point1.y};
    if (long_line.unit_direction().InnerProd(point1 - point2) > 0.0) {
      long_line.GetPerpendicularFoot(point1, &wheel_stop);
    } else {
      long_line.GetPerpendicularFoot(point2, &wheel_stop);
    }
    parking_slot_info.wheel_stop_info.wheel_stop_depth =
        wheel_stop.DistanceTo(long_line_seg.end());
  } else {
    parking_slot_info.wheel_stop_info.wheel_stop_depth = -1.0;
  }
  updateSlotByVirtualCorner(parking_slot_info);
  checkTwoSidesVehicle(parking_slot_info);
  checkPerpendicularBottomWall(parking_slot_info);

  // only use in non_parallel slot and non_wlc slot
  if (!PlanningContext::Instance()->planning_status().wlc_info.is_valid) {
    updateSlotBySingleSideObstacle(
        bisector, slot_two_sides_type, left_obstacle_to_origin_slot_center,
        right_obstacle_to_origin_slot_center, &parking_slot_info);
  }

  // debug info
  double slot_width = std::hypot(
      parking_slot_info.corners[0].x - parking_slot_info.corners[3].x,
      parking_slot_info.corners[0].y - parking_slot_info.corners[3].y);
  *PlanningContext::Instance()->mutable_planning_debug_info() +=
      "\n[su](type" + std::to_string(parking_slot_info.type.value) + ",wlc" +
      std::to_string(
          PlanningContext::Instance()->planning_status().wlc_info.is_valid) +
      ",spa" + std::to_string(parking_slot_info.is_space_slot) +
      "),slot_width" + std::to_string(slot_width).substr(0, 5) + "-" +
      std::to_string(slot_width - VehicleParam::Instance()->width)
          .substr(0, 5) +
      "m";

  return true;
}

bool ParkingSlotManager::updateSlotBySingleSideObstacle(
    const planning_math::Line2d &bisector,
    const std::pair<SlotClosestSideType, SlotClosestSideType>
        &slot_two_sides_type,
    const double left_obstacle_to_origin_slot_center,
    const double right_obstacle_to_origin_slot_center,
    ParkingSlotInfo *const ptr_parking_slot_info) {
  // only run one-side su logic in perpendicular slot
  if (ptr_parking_slot_info->type.value == ParkingSlotType::PARALLEL)
    return true;

  const double kMirrorToObstacleCarDist =
      CarParams::GetInstance()
          ->car_config.target_pose_config.oneside_dist_mirror_to_obstacle_car;

  bool is_single_side_slot = false;
  // if one side's obstacle dist < (slot smallest allowed width / 2)
  planning_math::Vec2d left_shift_vector =
      bisector.unit_direction().rotate(M_PI_2);
  planning_math::Vec2d right_shift_vector =
      bisector.unit_direction().rotate(-M_PI_2);

  // if slot's right side is step.
  double slot_width = ptr_parking_slot_info->slot_width();
  if (slot_two_sides_type.second != SlotClosestSideType::NONE_OBSTACLE) {
    double new_right_step_slot_width = 0.0;
    if (slot_two_sides_type.second == SlotClosestSideType::SIDE_SLOT) {
      new_right_step_slot_width = kNarrowestLegalSlotWidth;
    } else if (slot_two_sides_type.second ==
               SlotClosestSideType::SIDE_HIGH_OBSTACLE) {
      new_right_step_slot_width =
          VehicleParam::Instance()->width + kMirrorToPillarDist * 2.0;
    } else if (slot_two_sides_type.second ==
               SlotClosestSideType::SIDE_CAR_OBSTACLE) {
      new_right_step_slot_width =
          VehicleParam::Instance()->width + kMirrorToObstacleCarDist * 2.0;
    }
    if (ptr_parking_slot_info->left_empty == true &&
        right_obstacle_to_origin_slot_center < slot_width / 2) {
      is_single_side_slot = true;
      planning_math::Vec2d shift_vector =
          left_shift_vector * max(new_right_step_slot_width,
                                  2 * right_obstacle_to_origin_slot_center);
      ptr_parking_slot_info->corners[0].x =
          ptr_parking_slot_info->corners[3].x + shift_vector.x();
      ptr_parking_slot_info->corners[0].y =
          ptr_parking_slot_info->corners[3].y + shift_vector.y();
      ptr_parking_slot_info->corners[1].x =
          ptr_parking_slot_info->corners[2].x + shift_vector.x();
      ptr_parking_slot_info->corners[1].y =
          ptr_parking_slot_info->corners[2].y + shift_vector.y();
    }
  }
  // if slot's left side is step.
  if (slot_two_sides_type.first != SlotClosestSideType::NONE_OBSTACLE) {
    double new_left_step_slot_width = 0.0;
    if (slot_two_sides_type.first == SlotClosestSideType::SIDE_SLOT) {
      new_left_step_slot_width = kNarrowestLegalSlotWidth;
    } else if (slot_two_sides_type.first ==
               SlotClosestSideType::SIDE_HIGH_OBSTACLE) {
      new_left_step_slot_width =
          VehicleParam::Instance()->width + kMirrorToPillarDist * 2.0;
    } else if (slot_two_sides_type.first ==
               SlotClosestSideType::SIDE_CAR_OBSTACLE) {
      new_left_step_slot_width =
          VehicleParam::Instance()->width + kMirrorToObstacleCarDist * 2.0;
    }
    if (ptr_parking_slot_info->right_empty == true &&
        left_obstacle_to_origin_slot_center < slot_width / 2) {
      is_single_side_slot = true;
      planning_math::Vec2d shift_vector =
          right_shift_vector * max(new_left_step_slot_width,
                                   2 * left_obstacle_to_origin_slot_center);
      ptr_parking_slot_info->corners[3].x =
          ptr_parking_slot_info->corners[0].x + shift_vector.x();
      ptr_parking_slot_info->corners[3].y =
          ptr_parking_slot_info->corners[0].y + shift_vector.y();
      ptr_parking_slot_info->corners[2].x =
          ptr_parking_slot_info->corners[1].x + shift_vector.x();
      ptr_parking_slot_info->corners[2].y =
          ptr_parking_slot_info->corners[1].y + shift_vector.y();
    }
  }
  if (is_single_side_slot)
    *PlanningContext::Instance()->mutable_planning_debug_info() +=
        ", [su]singleSide,";

  return true;
}

void ParkingSlotManager::checkTwoSidesVehicle(
    ParkingSlotInfo &parking_slot_info) {
  auto &corners = parking_slot_info.corners;
  if (corners.size() != 4) {
    return;
  }
  if (parking_slot_info.type.value != ParkingSlotType::PARALLEL) {
    return;
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
      planning_math::Vec2d(cx, cy), ctheta, lot_length, lot_width);
  planning_math::Box2d slot_left(slot_box);
  planning_math::Box2d slot_right(slot_box);

  planning_math::Vec2d left_vector(corners[0].x - corners[3].x,
                                   corners[0].y - corners[3].y);
  slot_left.Shift(left_vector);
  slot_right.Shift(-1.0 * left_vector);
  Pose2D slot_center(cx, cy, ctheta - M_PI_2);
  planning_math::Vec2d slot_bottom_middle_point{bx, by};
  planning_math::Vec2d slot_bottom_middle_point_local =
      tf2d(slot_center, slot_bottom_middle_point);
  double obs_bottom_limit = slot_bottom_middle_point_local.y() + 0.2;

  double x_max_left = -10000.0;
  double x_min_right = 10000.0;
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
        if (uss_point_local.x() > x_max_left &&
            uss_point_local.y() >= obs_bottom_limit) {
          x_max_left = uss_point_local.x();
        }
      }
      if (slot_right.IsPointIn(uss_point)) {
        if (uss_point_local.x() < x_min_right &&
            uss_point_local.y() >= obs_bottom_limit) {
          x_min_right = uss_point_local.x();
        }
      }
    }
  }

  double slot_len_limit = VehicleParam::Instance()->length + 2.0;
  double slot_len_limit_1 = VehicleParam::Instance()->length + 1.1;
  double slot_len_limit_2 = VehicleParam::Instance()->length + 0.9;
  double slot_len_real = x_min_right - x_max_left;
  parking_slot_info.special_slot_type = 0;
  parking_slot_info.slot_length_type = 0;
  if (slot_len_real >= slot_len_limit_1 && slot_len_real < slot_len_limit) {
    parking_slot_info.slot_length_type = 1;
  }
  if (slot_len_real >= slot_len_limit_2 && slot_len_real < slot_len_limit_1) {
    parking_slot_info.slot_length_type = 2;
  }
  if (slot_len_real < slot_len_limit_2) {
    parking_slot_info.slot_length_type = 3;
  }

  switch (parking_slot_info.slot_length_type) {
  case 1:
    parking_slot_info.special_slot_type = 1;
    break;
  case 2:
    parking_slot_info.special_slot_type = 1;
    break;
  case 3:
    parking_slot_info.special_slot_type = 2;
    break;
  default:
    break;
  }

  MSD_LOG(ERROR, "[checkTwoSidesVehicle] slot_length_type:%d",
          parking_slot_info.slot_length_type);
  MSD_LOG(ERROR, "[checkTwoSidesVehicle] special_slot_type:%d",
          parking_slot_info.special_slot_type);
  return;
}

void ParkingSlotManager::checkPerpendicularBottomWall(
    ParkingSlotInfo &parking_slot_info) {
  auto &corners = parking_slot_info.corners;
  if (corners.size() != 4) {
    return;
  }
  if (parking_slot_info.type.value != ParkingSlotType::PERPENDICULAR) {
    return;
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
      planning_math::Vec2d(cx, cy), ctheta, lot_length, lot_width);
  planning_math::Box2d slot_bottom(slot_box);

  planning_math::Vec2d down_vector((bx - fx) / 2.0, (by - fy) / 2.0);
  slot_bottom.Shift(down_vector);
  bool is_bottom_wall = false;
  for (auto &obs : world_model_->get_parking_ground_line_fusion()) {
    if (obs.type < GroundLineType::GROUND_LINE_USS_TYPE_UNKNOWN ||
        obs.type >= GroundLineType::GROUND_LINE_USS_TYPE_POINT ||
        obs.id != parking_slot_info.id ||
        obs.type == GroundLineType::GROUND_LINE_USS_TYPE_STEP) {
      continue;
    }
    for (int i = 0; i < obs.pts.size(); i++) {
      planning_math::Vec2d uss_point{obs.pts[i].x, obs.pts[i].y};
      if (slot_bottom.IsPointIn(uss_point)) {
        is_bottom_wall = true;
        break;
      }
    }
  }

  parking_slot_info.bottom_line_type = 0;
  if (is_bottom_wall) {
    parking_slot_info.bottom_line_type = 1;
  }
  return;
}

void ParkingSlotManager::updateSlotByVirtualCorner(
    ParkingSlotInfo &parking_slot_info) {
  if (parking_slot_info.type.value != ParkingSlotType::OBLIQUE) {
    return;
  }
  const ParkingSlotInfo::VirtualCorner &virtual_corner =
      parking_slot_info.virtual_corner;
  if (virtual_corner.slot_type != 4) {
    return;
  }
  static double max_corner_dis = 5.0;
  auto &corners = parking_slot_info.original_corners;
  if (corners.size() != 4) {
    return;
  }
  planning_math::Vec2d left_front{corners[0].x, corners[0].y};
  planning_math::Vec2d left_rear{corners[1].x, corners[1].y};
  planning_math::Vec2d right_rear{corners[2].x, corners[2].y};
  planning_math::Vec2d right_front{corners[3].x, corners[3].y};
  planning_math::Vec2d virtual_p{virtual_corner.p.x, virtual_corner.p.y};
  if (virtual_corner.index == 3 &&
      right_front.DistanceTo(virtual_p) < max_corner_dis) {
    planning_math::Vec2d direc_delta = left_rear - left_front;
    planning_math::Vec2d right_rear_new = virtual_p + direc_delta;
    corners[3].x = virtual_p.x();
    corners[3].y = virtual_p.y();
    corners[2].x = right_rear_new.x();
    corners[2].y = right_rear_new.y();
    return;
  }
  if (virtual_corner.index == 0 &&
      left_front.DistanceTo(virtual_p) < max_corner_dis) {
    planning_math::Vec2d direc_delta = right_rear - right_front;
    planning_math::Vec2d left_rear_new = virtual_p + direc_delta;
    corners[0].x = virtual_p.x();
    corners[0].y = virtual_p.y();
    corners[1].x = left_rear_new.x();
    corners[1].y = left_rear_new.y();
    return;
  }
}

bool ParkingSlotManager::refactorCenterLines(
    planning_math::LineSegment2d &center_line,
    ParkingSlotInfo &parking_slot_info, std::vector<double> &slot_dis) {
  if (parking_slot_info.type.value != ParkingSlotType::PERPENDICULAR) {
    return false;
  }
  if (!PlanningContext::Instance()->planning_status().wlc_info.is_valid) {
    return false;
  }
  if (!world_model_->get_ego_state().is_static) {
    return false;
  }
  if (slot_dis.size() != 4) {
    return false;
  }

  constexpr static double min_safe_dis = 0.22;
  constexpr static double min_wlc_tolerance = 0.01;
  const static double min_safe_width =
      VehicleParam::Instance()->width + 2 * min_safe_dis;
  const static double half_min_safe_width = min_safe_width / 2.0;
  const static double half_car_width = VehicleParam::Instance()->width / 2.0;

  double left_distance = slot_dis[0];
  double right_distance = slot_dis[1];
  double max_left_distance = slot_dis[2];
  double max_right_distance = slot_dis[3];
  double current_width = left_distance + right_distance;
  double half_current_width = current_width / 2.0;
  if (current_width < min_safe_width) {
    MSD_LOG(INFO,
            "[refactorCenterLines] return as current_width %.6f is smaller "
            "than min_safe_width %.6f",
            current_width, min_safe_width);
    return false;
  }

  // distance from ego to center line, + for right, - for left
  Pose2D ego_pose = world_model_->get_ego_state().ego_pose;
  planning_math::Vec2d end2ego = planning_math::Vec2d(
      ego_pose.x - center_line.end().x(), ego_pose.y - center_line.end().y());
  // end2ego.Normalize();
  const planning_math::Vec2d &center_direc = center_line.unit_direction();
  double ego_to_center = end2ego.CrossProd(center_direc);

  // distance from expected target to center line, + for right, - for left
  double expected_target_to_ego =
      PlanningContext::Instance()->planning_status().wlc_info.y_offset;
  double expected_target_to_center = ego_to_center + expected_target_to_ego;
  MSD_LOG(INFO,
          "[refactorCenterLines] ego_to_center=%.6f, "
          "expected_target_to_ego=%.6f, expected_target_to_center=%.6f",
          ego_to_center, expected_target_to_ego, expected_target_to_center);

  double abs_offset = std::abs(expected_target_to_center);
  if (abs_offset < min_wlc_tolerance) {
    MSD_LOG(INFO,
            "[refactorCenterLines] abs(%.6f) is smaller than "
            "min_wlc_tolerance, which is %.6f",
            expected_target_to_center, min_wlc_tolerance);
    return false;
  }

  bool is_right = expected_target_to_center > 0;
  double left_remain = std::max(0.0, max_left_distance - left_distance);
  double right_remain = std::max(0.0, max_right_distance - right_distance);
  planning_math::Vec2d left_shift_vec =
      center_line.unit_direction().rotate(M_PI_2);

  MSD_LOG(
      INFO,
      "[refactorCenterLines] is_right=%d, left_remain=%.6f, right_remain=%.6f",
      is_right, left_remain, right_remain);
  MSD_LOG(INFO, "[refactorCenterLines] left_distance=%.6f, right_distance=%.6f",
          left_distance, right_distance);
  MSD_LOG(
      INFO,
      "[refactorCenterLines] old start: x=%.6f y=%.6f old end: x=%.6f y=%.6f",
      center_line.start().x(), center_line.start().y(), center_line.end().x(),
      center_line.end().y());
  MSD_LOG(INFO, "[refactorCenterLines] left_shift_vec (%.6f, %.6f)",
          left_shift_vec.x(), left_shift_vec.y());

  if (is_right && right_remain > abs_offset - 1e-6) {
    MSD_LOG(INFO,
            "[refactorCenterLines] is_right and adjust width is sufficient");
    planning_math::Vec2d new_start_p =
        center_line.start() - left_shift_vec * abs_offset;
    planning_math::Vec2d new_end_p =
        center_line.end() - left_shift_vec * abs_offset;
    center_line = planning_math::LineSegment2d(new_start_p, new_end_p);
    right_distance += abs_offset;
    left_distance -= abs_offset;
  }

  if (!is_right && left_remain > abs_offset - 1e-6) {
    MSD_LOG(INFO,
            "[refactorCenterLines] is_left and adjust width is sufficient");
    planning_math::Vec2d new_start_p =
        center_line.start() + left_shift_vec * abs_offset;
    planning_math::Vec2d new_end_p =
        center_line.end() + left_shift_vec * abs_offset;
    center_line = planning_math::LineSegment2d(new_start_p, new_end_p);
    right_distance -= abs_offset;
    left_distance += abs_offset;
  }

  double extra_width = half_current_width - half_min_safe_width;
  double right_max_adjust_width = extra_width + right_remain;
  double left_max_adjust_width = extra_width + left_remain;
  MSD_LOG(INFO,
          "[refactorCenterLines] extra_width=%.6f, "
          "right_max_adjust_width=%.6f, left_max_adjust_width=%.6f",
          extra_width, right_max_adjust_width, left_max_adjust_width);
  if (is_right && right_remain < abs_offset) {
    MSD_LOG(INFO,
            "[refactorCenterLines] is_right and adjust width is insufficient");
    double adjust_extra_width = right_max_adjust_width < abs_offset
                                    ? extra_width
                                    : abs_offset - right_remain;
    double adjust_center_width = adjust_extra_width + right_remain;
    right_distance += right_remain;
    left_distance -= adjust_extra_width + adjust_center_width;
    planning_math::Vec2d new_start_p =
        center_line.start() - left_shift_vec * adjust_center_width;
    planning_math::Vec2d new_end_p =
        center_line.end() - left_shift_vec * adjust_center_width;
    center_line = planning_math::LineSegment2d(new_start_p, new_end_p);
  }

  if (!is_right && left_remain < abs_offset) {
    MSD_LOG(INFO,
            "[refactorCenterLines] is_left and adjust width is insufficient");
    double adjust_extra_width = left_max_adjust_width < abs_offset
                                    ? extra_width
                                    : abs_offset - left_remain;
    double adjust_center_width = adjust_extra_width + left_remain;
    left_distance += left_remain;
    right_distance -= adjust_extra_width + adjust_center_width;
    planning_math::Vec2d new_start_p =
        center_line.start() + left_shift_vec * adjust_center_width;
    planning_math::Vec2d new_end_p =
        center_line.end() + left_shift_vec * adjust_center_width;
    center_line = planning_math::LineSegment2d(new_start_p, new_end_p);
  }
  slot_dis[0] = left_distance;
  slot_dis[1] = right_distance;
  MSD_LOG(INFO,
          "[refactorCenterLines] after refactor left_distance=%.6f, "
          "right_distance=%.6f",
          left_distance, right_distance);
  MSD_LOG(INFO,
          "[refactorCenterLines] after refactor new start: x=%.3f y=%.3f new "
          "end: x=%.3f y=%.3f",
          center_line.start().x(), center_line.start().y(),
          center_line.end().x(), center_line.end().y());

  return true;
}

std::uint8_t ParkingSlotManager::GetParkingSlotType(int slot_type) {
  if (slot_type == 1) {
    return ParkingSlotType::PERPENDICULAR;
  } else if (slot_type == 2) {
    return ParkingSlotType::PARALLEL;
  } else if (slot_type == 3) {
    return ParkingSlotType::OBLIQUE;
  } else if (slot_type == 4) {
    return ParkingSlotType::OBLIQUE;
  } else {
    MSD_LOG(ERROR,
            "ParkingSlotManager::GetParkingSlotType: got %d which cannot trans "
            "to ParkingSlotType",
            slot_type);
    return ParkingSlotType::PERPENDICULAR;
  }
}

bool ParkingSlotManager::GetParkInPoint() {
  auto &parking_slot_info = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->parking_slot_info;
  auto refline_manager = RefLineManager::Instance();
  if (world_model_ == nullptr) {
    MSD_LOG(INFO, "GetParkInPoint failed: world model is none!");
    return false;
  }
  if (parking_slot_info.corners.size() != 4) {
    MSD_LOG(INFO, "GetParkInPoint failed: size of corners incorrect!");
    return false;
  }
  std::vector<Point3D> corners = parking_slot_info.corners;
  // corners sequence : 0 LF 1 LR 2 RR 3 RF
  auto frenet_coor = world_model_->get_frenet_coord();

  if (frenet_coor == nullptr) {
    MSD_LOG(INFO, "GetParkInPoint failed: frenet_coor is nullptr!");
    return false;
  }
  auto &ego_state = world_model_->get_ego_state();

  // height judgement
  double parking_slot_height =
      (corners[0].z + corners[1].z + corners[2].z + corners[3].z) / 4.0;

  if (std::abs(ego_state.ego_enu.position.z - parking_slot_height) > 1.0) {
    MSD_LOG(INFO, "GetParkInPoint failed: parking slot height incompatible!");
    return false;
  }

  Point2D apa_cart, apa_frenet;
  apa_cart.x = (corners[0].x + corners[3].x) / 2.0;
  apa_cart.y = (corners[0].y + corners[3].y) / 2.0;
  double parking_lot_heading =
      atan2(apa_cart.y - (corners[1].y + corners[2].y) / 2.0,
            apa_cart.x - (corners[1].x + corners[2].x) / 2.0);

  if (frenet_coor->CartCoord2FrenetCoord(apa_cart, apa_frenet) ==
      TRANSFORM_FAILED) {
    MSD_LOG(INFO, "GetParkInPoint failed: frenet transform failed!");
    return false;
  }
  // position check
  if (std::abs(apa_frenet.y) > 10.0 ||
      std::abs(apa_frenet.x - ego_state.ego_frenet.x) > 15.0) {
    MSD_LOG(INFO, "GetParkInPoint failed: distance too far!");
    return false;
  }
  // yaw check
  double relative_heading = parking_lot_heading - ego_state.ego_pose.theta;
  double relative_heading_refline =
      parking_lot_heading - frenet_coor->GetRefCurveCurvature(apa_frenet.x);

  if (std::abs(std::cos(relative_heading)) > std::cos(M_PI / 4) &&
      std::abs(std::cos(relative_heading_refline)) > std::cos(M_PI / 4)) {
    MSD_LOG(INFO, "GetParkInPoint failed: parking slot heading incompatible!");
    return false;
  }

  // find park in point
  double s_max_ = frenet_coor->GetSlength();
  double end_s = std::min(default_distance_apa_ + apa_frenet.x,
                          std::max(s_max_ - 5.0, 0.0));
  if (end_s > s_max_) {
    MSD_LOG(INFO, "GetParkInPoint failed: refline too short!");
    return false;
  }
  // while(std::abs(frenet_coor->GetRefCurveCurvature(end_s)) >
  // max_end_pose_curv_){
  //   end_s += s_step_;
  //   if (end_s > s_max_){
  //     return false;
  //   }
  // }
  if (std::cos(frenet_coor->GetRefCurveHeading(end_s) - parking_lot_heading) <
          std::cos(3 * M_PI / 4) ||
      (std::cos(frenet_coor->GetRefCurveHeading(end_s) - parking_lot_heading) <
           std::cos(10 * M_PI / 18) &&
       std::abs(frenet_coor->GetRefCurveCurvature(end_s)) >
           max_end_pose_curv_)) {
    end_s = std::min(default_distance_apa_ + apa_frenet.x,
                     std::max(s_max_ - 5.0, 0.0));
    while (std::abs(frenet_coor->GetRefCurveCurvature(end_s)) >
               max_end_pose_curv_ ||
           std::cos(frenet_coor->GetRefCurveHeading(end_s) -
                    parking_lot_heading) < std::cos(3 * M_PI / 4)) {
      end_s -= s_step_;
      if (end_s > s_max_ || end_s < 0.0) {
        MSD_LOG(INFO, "GetParkInPoint failed: refline too short!");
        return false;
      }
    }
  }
  double right_border_width = std::abs(
      planning_math::interps(refline_manager->get_right_road_border_distance(),
                             refline_manager->get_s(), end_s));
  double left_border_width = std::abs(
      planning_math::interps(refline_manager->get_left_road_border_distance(),
                             refline_manager->get_s(), end_s));
  double l_deviation = std::min(
      std::max((left_border_width - right_border_width) / 2.0, -2.0), 2.0);
  Point2D frenet_end_pos(end_s, l_deviation);
  Point2D cart_end_pose;
  if (frenet_coor->FrenetCoord2CartCoord(frenet_end_pos, cart_end_pose) ==
      TRANSFORM_STATUS::TRANSFORM_FAILED) {
    MSD_LOG(INFO, "GetParkInPoint failed: frenet transform failed!");
    return false;
  }

  // get heading
  double end_heading = frenet_coor->GetRefCurveHeading(end_s);
  if (std::cos(end_heading - ego_state.ego_pose.theta) <
      std::cos(3 * M_PI / 4)) {
    MSD_LOG(INFO, "GetParkInPoint failed: parking slot heading incompatible!");
    return false;
  }

  // end state
  parking_slot_info.park_in_point.point.x = cart_end_pose.x;
  parking_slot_info.park_in_point.point.y = cart_end_pose.y;
  parking_slot_info.park_in_point.point.theta = end_heading;
  parking_slot_info.park_in_point.available = true;

  MSD_LOG(INFO, "GetParkInPoint succeeded!");

  return true;
}

bool ParkingSlotManager::UpdateParkingSlotDistance() {
  auto &parking_slot_info = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->parking_slot_info;
  parking_slot_info.park_in_point.distance = 1e19;
  parking_slot_info.park_out_point.distance = 1e19;
  parking_slot_info.distance_to_parking_slot = 1e19;
  if (world_model_ == nullptr) {
    MSD_LOG(INFO, "UpdateParkingSlotDistance failed: world model is none!");
    return false;
  }
  auto frenet_coor = world_model_->get_frenet_coord();
  if (frenet_coor == nullptr) {
    MSD_LOG(INFO, "UpdateParkingSlotDistance failed: frenet_coor is nullptr!");
    return false;
  }

  auto &ego_state = world_model_->get_ego_state();
  std::vector<Point3D> corners = parking_slot_info.corners;
  // corners sequence : 0 LF 1 LR 2 RR 3 RF
  if (parking_slot_info.corners.size() != 4) {
    MSD_LOG(INFO,
            "UpdateParkingSlotDistance failed: size of corners incorrect!");
    return false;
  }
  double parking_lot_heading = atan2((corners[0].y + corners[3].y) / 2.0 -
                                         (corners[1].y + corners[2].y) / 2.0,
                                     (corners[0].x + corners[3].x) / 2.0 -
                                         (corners[1].x + corners[2].x) / 2.0);

  Point2D poi_cart, poi_frenet;
  poi_cart.x = (corners[0].x + corners[3].x) / 2.0;
  poi_cart.y = (corners[0].y + corners[3].y) / 2.0;
  if (frenet_coor->CartCoord2FrenetCoord(poi_cart, poi_frenet) !=
      TRANSFORM_FAILED) {
    parking_slot_info.distance_to_parking_slot =
        poi_frenet.x - ego_state.ego_frenet.x;
  }

  // yaw check
  double relative_heading = parking_lot_heading - ego_state.ego_pose.theta;
  if (std::cos(relative_heading) < std::cos(3 * M_PI / 4)) {
    MSD_LOG(
        INFO,
        "UpdateParkingSlotDistance failed: parking slot heading incompatible!");
    return false;
  }

  if (!parking_slot_info.park_in_point.available) {
    MSD_LOG(INFO,
            "UpdateParkingSlotDistance failed: park in point unavailable!");
    return false;
  }

  Point2D apa_cart, apa_frenet;
  apa_cart.x = parking_slot_info.park_in_point.point.x;
  apa_cart.y = parking_slot_info.park_in_point.point.y;
  if (frenet_coor->CartCoord2FrenetCoord(apa_cart, apa_frenet) ==
      TRANSFORM_FAILED) {
    MSD_LOG(INFO, "UpdateParkingSlotDistance failed: frenet transform failed!");
    return false;
  }
  if ((apa_frenet.x - ego_state.ego_frenet.x) < -15) {
    MSD_LOG(INFO, "UpdateParkingSlotDistance failed: distance too far!");
    return false;
  }
  parking_slot_info.park_in_point.distance =
      apa_frenet.x - ego_state.ego_frenet.x;
  if (parking_slot_info.park_in_point.distance > -5 &&
      parking_slot_info.park_in_point.distance < 5 &&
      std::cos(relative_heading) < std::cos(10 * M_PI / 18)) {
    parking_slot_info.distance_to_parking_slot = 0.0;
  }

  MSD_LOG(INFO, "UpdateParkingSlotDistance succeeded!");

  return true;
}

} // namespace parking

} // namespace msquare
