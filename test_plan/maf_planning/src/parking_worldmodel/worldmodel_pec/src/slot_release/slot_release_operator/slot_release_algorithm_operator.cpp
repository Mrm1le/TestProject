

#include "slot_release_algorithm_operator.h"
#include "pnc/define/parking_vision_info.h"
#include "utils/math/math_helper.hpp"
#include "utils/math/polygon.hpp"
#include <fstream>
#include <mlog_core/mlog.h>
#include "../parameters/slot_release_params.h"
#include "../parameters/slot_release_vehicle_param.h"

// #define GROUNDLINE_SAVE_AREA_EXTEND_LENGTH_OUT_PERPENDICULAR (4.3)
// #define GROUNDLINE_SAVE_AREA_EXTEND_LENGTH_OUT_PARALLEL (3.7)
// #define GROUNDLINE_SAVE_AREA_EXTEND_LENGTH_OUT_OBLIQUE (4.3)
// #define GROUNDLINE_SAVE_AREA_EXTEND_LENGTH_IN (1.2)
// #define VALID_PASSAGE_WIDTH 4.3
// #define VALID_PASSAGE_WIDTH_PARALLEL 3.7
// #define VALID_PASSAGE_WIDTH_OBLIQUE 4.3
// #define HALF_WIDTH (1.0)
// #define HORIZONTAL_PARKINGSLOT_EXTENDED_AREA_LENGTH (VEHICLE_LENGTH + 2 * 0.3)
// #define HORIZONTAL_PARKINGSLOT_EXTENDED_AREA_WIDTH (VEHICLE_WIDTH + 3.7)
// #define VERTICAL_PARKINGSLOT_EXTENDED_AREA_LENGTH (VEHICLE_LENGTH + 4.1)
// #define VERTICAL_PARKINGSLOT_EXTENDED_AREA_WIDTH (VEHICLE_WIDTH + 2 * 0.2)
// #define VEHICLE_LENGTH (5.098)
// #define VEHICLE_WIDTH (1.960)

#define DIS_TO_REAR (1.0)
#define DIS_TO_FRONT (4.0)

#define L_7_VEHICLE_WIDTH_REAL (2.11)
#define L_7_VEHICLE_LENGTH (5.098)

namespace worldmodel_pec {

void SlotReleaseHandler::process(
    const maf_perception_interface::FusionGroundLineResult &object_fusion,
    const maf_perception_interface::PerceptionFusionObjectResult
        &object_detection,
    SlotReleaseManager::EgoCarInfo ego_car_info,
    Eigen::Isometry3d local_ego_pose,
    std::deque<Eigen::Isometry3d> history_locs, bool is_move, int apa_target_id,
    std::string apa_status, int ego_parking_slot_track_id,
    const SlotReleaseParams &params, bool is_apa_mode,
    std::list<SlotReleaseManager::ParkingSlotUnion> &history_slots) {

  // std::cout << "loc:" << local_ego_pose.translation().x()
  //           << " " << local_ego_pose.translation().y()
  //           << " " << ego_car_info.velocity_ << " " << ego_car_info.yaw_
  //           << " " << ego_car_info.pitch_ << " target id:" << apa_target_id
  //           << " is move:" << is_move << "  hos traj:" << history_locs.size()
  //           << " apa status:" << apa_status << " " << params.compensate_time
  //           << " his slot size:" << history_slots.size() << std::endl;

  CommonHandler common_handler(local_ego_pose, object_fusion, params,
                               ego_car_info.yaw_, is_apa_mode);
  for (auto &union_slot : history_slots) {

    // 0.
    if (union_slot.fusion_slot_.corners_.size() != 4) {
      union_slot.fusion_slot_.empty_votes_ = 0;
      union_slot.failure_reason_ = -200;
      continue;
    }

    common_handler.feedFusionSlot(union_slot.fusion_slot_);
    common_handler.generateSlotFrame();
    common_handler.generateObjFusionToSlotFrame();

    EnvFeatureHandler env_feature_handler(&common_handler);
    EgoCarFeatureHandler ego_car_handler(
        &common_handler, history_locs, is_move, apa_target_id, apa_status,
        ego_car_info.velocity_, ego_car_info.pitch_);
    OtherFeatureHandler other_feature_handler(&common_handler);

    // std::cout << "===========the slot id is:" <<
    // union_slot.fusion_slot_.track_id_ << std::endl;

    env_feature_handler.extractFeature();
    ego_car_handler.extractFeature();
    other_feature_handler.extractFeature();

    union_slot.failure_reason_ = -1;
    // if(isPassTime(ego_car_info.gear_, union_slot)) {
    //   union_slot.pass_vel_ = ego_car_info.velocity_;
    // }
    // 1.
    if (!ego_car_handler.isMove() && is_apa_mode) {
      union_slot.fusion_slot_.empty_votes_ = 0;
      union_slot.failure_reason_ = -100;
      continue;
    }

    // 2. 泊入阶段库位全释放
    if (ego_car_handler.isParkingStatus()) {
      union_slot.fusion_slot_.empty_votes_ = 10;
      continue;
    }

    // 3.
    if (other_feature_handler.isSlotOccupiedFromPSD()) {
      union_slot.failure_reason_ = -2000 + union_slot.fusion_slot_.empty_votes_;
      if (!isOccupiedByVehicle(union_slot, object_detection)) {
        union_slot.fusion_slot_.empty_votes_ = 0;
      }
      continue;
    }

    // 4.
    if (!env_feature_handler.isSlotOpenAreaWidthEnough(union_slot)) {
      union_slot.fusion_slot_.empty_votes_ = 0;
      union_slot.failure_reason_ = -321;
      continue;
    }

    // 5.
    if (!env_feature_handler.isSlotOpenAreaLengthEnough(union_slot)) {
      union_slot.fusion_slot_.empty_votes_ = 0;
      union_slot.failure_reason_ = -322;
      continue;
    }

    // 6.
    // if (!env_feature_handler.isPassageWidthEnough(union_slot)) {
    //   union_slot.fusion_slot_.empty_votes_ = 0;
    //   union_slot.failure_reason_ = -340;
    //   continue;
    // }

    // 7.
    if (env_feature_handler.isDeadEndRoad() && common_handler.isParalleSlot()
        && is_apa_mode) {
      union_slot.fusion_slot_.empty_votes_ = 0;
      union_slot.failure_reason_ = -110;
      continue;
    }

    // 8. 车位的规划泊车安全区域无障碍物(Object)入侵 合格
    // if (!common_handler.isObliqueSlot() &&
    //     isParkingSafetyAreaOccupiedByObject(
    //       union_slot, object_detection,&common_handler) && is_apa_mode) {
    //   union_slot.fusion_slot_.empty_votes_ = 0;
    //   union_slot.failure_reason_ = -30;
    //   continue;
    // }

    // 9. 车在车位中所有库位不释放
    if (ego_car_handler.isCarInSlot() && ego_parking_slot_track_id != -1) {
      // union_slot.fusion_slot_.empty_votes_ = 0;
      // union_slot.failure_reason_ = -40;
      if (union_slot.fusion_slot_.charge_property_ == 1) {
        if (ego_parking_slot_track_id == union_slot.fusion_slot_.track_id_) {
          // 所在车位直接释放
          union_slot.fusion_slot_.empty_votes_ = 10;
          union_slot.failure_reason_ = -1;
        } else {
          // 所有其他空车位都不释放
          union_slot.fusion_slot_.empty_votes_ = 0;
          union_slot.failure_reason_ = -40;
        }
      } else {
          union_slot.fusion_slot_.empty_votes_ = 0;
          union_slot.failure_reason_ = -40;        
      }
      continue;
    }

    // 10.
    if (!ego_car_handler.isvelocityReasonable() && is_apa_mode) {
      union_slot.fusion_slot_.empty_votes_ = 0;
      union_slot.failure_reason_ = -501;
      continue;
    }

    // 11.
    if (!ego_car_handler.isYawReasonable() && is_apa_mode) {
      union_slot.fusion_slot_.empty_votes_ = 0;
      union_slot.failure_reason_ = -502;
      continue;
    }

    // 12.
    if (!ego_car_handler.isPitchReasonable() && is_apa_mode) {
      union_slot.fusion_slot_.empty_votes_ = 0;
      union_slot.failure_reason_ = -503;
      continue;
    }

    // 13.
    if (!ego_car_handler.isSlotCenterToEgoDisReasonable() && is_apa_mode) {
      union_slot.fusion_slot_.empty_votes_ = 0;
      union_slot.failure_reason_ = -51;
      continue;
    }

    // 14.
    if (!ego_car_handler.isSlotFullyObservedAccordHisTraj(union_slot) && is_apa_mode) {
      union_slot.fusion_slot_.empty_votes_ = 0;
      union_slot.failure_reason_ = -61;
      continue;
    }

    // 15.
    if (!ego_car_handler.isEgoCarPassSlotDownstream() && is_apa_mode) {
      union_slot.fusion_slot_.empty_votes_ = 0;
      union_slot.failure_reason_ = -62;
      continue;
    }

    // 16.
    if (!other_feature_handler.isObsSafeToEgoCar(union_slot) && is_apa_mode) {
      union_slot.fusion_slot_.empty_votes_ = 0;
      union_slot.failure_reason_ = -90;
      continue;
    }

    // 17.
    if (!other_feature_handler.isParallelSlotSizeReasonable() && is_apa_mode) {
      union_slot.fusion_slot_.empty_votes_ = 0;
      union_slot.failure_reason_ = -70;
      continue;
    }
  }
}

bool SlotReleaseHandler::isPassTime(
    int gear, const SlotReleaseManager::ParkingSlotUnion &parking_slot_union) {
  if (gear != 0) {
    if (std::abs(parking_slot_union.pass_vel_) < 1e-3) {
      return true;
    } else {
      return false;
    }
  }
  return false;
}

bool SlotReleaseHandler::isOccupiedByVehicle(
    const SlotReleaseManager::ParkingSlotUnion &parking_slot_union,
    const maf_perception_interface::PerceptionFusionObjectResult
        &object_detection) {

  std::vector<ParkingSlotPoint> parking_slot =
      parking_slot_union.fusion_slot_.corners_;

  std::vector<Eigen::Vector2d> extended_free_area;
  Eigen::Vector2d p0(parking_slot[0].position.x(),
                     parking_slot[0].position.y());
  Eigen::Vector2d p1(parking_slot[1].position.x(),
                     parking_slot[1].position.y());
  Eigen::Vector2d p2(parking_slot[2].position.x(),
                     parking_slot[2].position.y());
  Eigen::Vector2d p3(parking_slot[3].position.x(),
                     parking_slot[3].position.y());
  extended_free_area.push_back(p0);
  extended_free_area.push_back(p1);
  extended_free_area.push_back(p2);
  extended_free_area.push_back(p3);

  for (auto &p : object_detection.perception_fusion_objects_data) {
    if (p.type_info.type.value == p.type_info.type.OBJECT_TYPE_VEHICLE) {
      Eigen::Vector2d vehicle_center(p.position.x, p.position.y);
      if (putils::Polygon::pointInPolygon(vehicle_center, extended_free_area)) {
        MLOG_DEBUG("FOUND VEHICLE IN PARKING SLOT %d",
                   parking_slot_union.fusion_slot_.track_id_);
        return true;
      }
    }
  }
  return false;
}

bool SlotReleaseHandler::isParkingSafetyAreaOccupiedByObject(
    const SlotReleaseManager::ParkingSlotUnion &parking_slot_union,
    const maf_perception_interface::PerceptionFusionObjectResult
        &object_detection, CommonHandler *common_handler) {
  double HORIZONTAL_PARKINGSLOT_EXTENDED_AREA_LENGTH =
    CarParams::GetInstance()->vehicle_length + 2 * 0.3;
  double HORIZONTAL_PARKINGSLOT_EXTENDED_AREA_WIDTH =
    CarParams::GetInstance()->vehicle_width + 
    common_handler->getParams().parallel_passage_width - 0.2;
  double VERTICAL_PARKINGSLOT_EXTENDED_AREA_LENGTH = 
    CarParams::GetInstance()->vehicle_length + 
    common_handler->getParams().vertical_passage_width - 0.2;
  double VERTICAL_PARKINGSLOT_EXTENDED_AREA_WIDTH = 
    CarParams::GetInstance()->vehicle_width + 2 * 0.2;
  bool is_occupied = false;

  std::vector<ParkingSlotPoint> parking_slot =
      parking_slot_union.fusion_slot_.corners_;

  // parking_slot_center,parking_slot_heading_vec 分别表示车位中点和车位朝向
  Eigen::Vector2d parking_slot_center(0, 0);
  Eigen::Vector2d parking_slot_heading_vec(0, 1);
  Eigen::Vector2d parking_slot_lateral_vec(1, 0);
  bool vertical_parking_slot;
  for (auto &p : parking_slot) {
    parking_slot_center.x() += p.position.x();
    parking_slot_center.y() += p.position.y();
  }
  parking_slot_center.x() /= 4;
  parking_slot_center.y() /= 4;
  parking_slot_heading_vec.x() =
      parking_slot[0].position.x() - parking_slot[1].position.x();
  parking_slot_heading_vec.y() =
      parking_slot[0].position.y() - parking_slot[1].position.y();
  parking_slot_lateral_vec.x() =
      parking_slot[3].position.x() - parking_slot[0].position.x();
  parking_slot_lateral_vec.y() =
      parking_slot[3].position.y() - parking_slot[0].position.y();
  if (parking_slot_heading_vec.norm() > parking_slot_lateral_vec.norm()) {
    vertical_parking_slot = true;
  } else {
    // horizontal parking slot
    vertical_parking_slot = false;
  }
  parking_slot_heading_vec.normalize();
  parking_slot_lateral_vec.normalize();
  std::vector<Eigen::Vector2d> extended_free_area;
  if (vertical_parking_slot) {
    Eigen::Vector2d p0 = (-CarParams::GetInstance()->vehicle_length / 2) * 
                             parking_slot_heading_vec +
                         (-VERTICAL_PARKINGSLOT_EXTENDED_AREA_WIDTH / 2) *
                             parking_slot_lateral_vec +
                         parking_slot_center;
    Eigen::Vector2d p1 =
        (VERTICAL_PARKINGSLOT_EXTENDED_AREA_LENGTH - 
            CarParams::GetInstance()->vehicle_length / 2) *
            parking_slot_heading_vec +
        (-VERTICAL_PARKINGSLOT_EXTENDED_AREA_WIDTH / 2) *
            parking_slot_lateral_vec +
        parking_slot_center;
    Eigen::Vector2d p2 =
        (VERTICAL_PARKINGSLOT_EXTENDED_AREA_LENGTH - 
            CarParams::GetInstance()->vehicle_length / 2) *
            parking_slot_heading_vec +
        (VERTICAL_PARKINGSLOT_EXTENDED_AREA_WIDTH / 2) *
            parking_slot_lateral_vec +
        parking_slot_center;
    Eigen::Vector2d p3 = (-CarParams::GetInstance()->vehicle_length / 2) * 
                         parking_slot_heading_vec +
                         (VERTICAL_PARKINGSLOT_EXTENDED_AREA_WIDTH / 2) *
                             parking_slot_lateral_vec +
                         parking_slot_center;
    extended_free_area.push_back(p0);
    extended_free_area.push_back(p1);
    extended_free_area.push_back(p2);
    extended_free_area.push_back(p3);
  } else {
    Eigen::Vector2d p0 =
        (HORIZONTAL_PARKINGSLOT_EXTENDED_AREA_WIDTH - 
            CarParams::GetInstance()->vehicle_width  / 2) *
            parking_slot_heading_vec +
        (-HORIZONTAL_PARKINGSLOT_EXTENDED_AREA_LENGTH / 2) *
            parking_slot_lateral_vec +
        parking_slot_center;
    Eigen::Vector2d p1 =
        (HORIZONTAL_PARKINGSLOT_EXTENDED_AREA_WIDTH - 
            CarParams::GetInstance()->vehicle_width  / 2) *
            parking_slot_heading_vec +
        (HORIZONTAL_PARKINGSLOT_EXTENDED_AREA_LENGTH / 2) *
            parking_slot_lateral_vec +
        parking_slot_center;
    Eigen::Vector2d p2 = (-CarParams::GetInstance()->vehicle_width  / 2) * 
                             parking_slot_heading_vec +
                         (HORIZONTAL_PARKINGSLOT_EXTENDED_AREA_LENGTH / 2) *
                             parking_slot_lateral_vec +
                         parking_slot_center;
    Eigen::Vector2d p3 = (-CarParams::GetInstance()->vehicle_width  / 2) * 
                             parking_slot_heading_vec +
                         (-HORIZONTAL_PARKINGSLOT_EXTENDED_AREA_LENGTH / 2) *
                             parking_slot_lateral_vec +
                         parking_slot_center;
    extended_free_area.push_back(p0);
    extended_free_area.push_back(p1);
    extended_free_area.push_back(p2);
    extended_free_area.push_back(p3);
  }

  for (auto &p : object_detection.perception_fusion_objects_data) {

    if (p.type_info.type.value == p.type_info.type.OBJECT_TYPE_VRU ||
        p.type_info.type.value == p.type_info.type.OBJECT_TYPE_GENERAL_OBJECT) {
      Eigen::Vector2d pos(p.position.x, p.position.y);
      if (putils::Polygon::pointInPolygon(pos, extended_free_area)) {
        is_occupied = true;
        MLOG_DEBUG("FOUND VRU IN PARKING SLOT %d",
                   parking_slot_union.fusion_slot_.track_id_);
        // printf("FOUND VRU IN PARKING SLOT %d\n",
        // parking_slot_union.fusion_slot_.track_id_);
        break;
      }
    }

    if (p.type_info.type.value == p.type_info.type.OBJECT_TYPE_VEHICLE) {
      std::vector<Eigen::Vector2d> vehicle;
      vehicle.push_back(Eigen::Vector2d(
          p.position.x + cos(p.heading_yaw) * p.shape.length / 2 +
              sin(p.heading_yaw) * p.shape.width / 2,
          p.position.y + sin(p.heading_yaw) * p.shape.length / 2 -
              cos(p.heading_yaw) * p.shape.width / 2));
      vehicle.push_back(Eigen::Vector2d(
          p.position.x - cos(p.heading_yaw) * p.shape.length / 2 +
              sin(p.heading_yaw) * p.shape.width / 2,
          p.position.y - sin(p.heading_yaw) * p.shape.length / 2 -
              cos(p.heading_yaw) * p.shape.width / 2));
      vehicle.push_back(Eigen::Vector2d(
          p.position.x - cos(p.heading_yaw) * p.shape.length / 2 -
              sin(p.heading_yaw) * p.shape.width / 2,
          p.position.y - sin(p.heading_yaw) * p.shape.length / 2 +
              cos(p.heading_yaw) * p.shape.width / 2));
      vehicle.push_back(Eigen::Vector2d(
          p.position.x + cos(p.heading_yaw) * p.shape.length / 2 -
              sin(p.heading_yaw) * p.shape.width / 2,
          p.position.y + sin(p.heading_yaw) * p.shape.length / 2 +
              cos(p.heading_yaw) * p.shape.width / 2));
      if (putils::Polygon::getIntersectPolygonArea(vehicle,
                                                   extended_free_area) > 0.8) {
        is_occupied = true;
        MLOG_DEBUG("FOUND VEHICLE IN PARKING SLOT %d",
                   parking_slot_union.fusion_slot_.track_id_);
        break;
      }
    }
  }

  return is_occupied;
}

bool SlotReleaseHandler::isParkingSafetyAreaOccupiedByObjectFusion(
    CommonHandler *common_handler) {
  // printf("%s: %d\n", __FUNCTION__, __LINE__);
  bool is_occupied = false;

  const auto &object_fusion = common_handler->getObjectionFusion();
  const auto &parking_slot_union = common_handler->getFusionSlot();

  std::vector<ParkingSlotPoint> parking_slot =
      common_handler->getFusionSlot().corners_;
  if (parking_slot.size() < 4) {
    return true;
  }

  if (!(object_fusion.ground_line.available &
        object_fusion.ground_line.FUSION_GROUND_LINE_AVAILABLE_DATA)) {
    return false;
  }

  // parking_slot_center,parking_slot_heading_vec 分别表示车位开口中点和车位朝向
  Eigen::Vector2d parking_slot_open_center(0, 0);
  Eigen::Vector2d parking_slot_heading_vec(0, 1);
  Eigen::Vector2d parking_slot_lateral_vec(1, 0);
  parking_slot_open_center.x() =
      (parking_slot[0].position.x() + parking_slot[3].position.x()) / 2;
  parking_slot_open_center.y() =
      (parking_slot[0].position.y() + parking_slot[3].position.y()) / 2;
  parking_slot_heading_vec.x() =
      parking_slot[0].position.x() - parking_slot[1].position.x();
  parking_slot_heading_vec.y() =
      parking_slot[0].position.y() - parking_slot[1].position.y();
  parking_slot_lateral_vec.x() =
      parking_slot[3].position.x() - parking_slot[0].position.x();
  parking_slot_lateral_vec.y() =
      parking_slot[3].position.y() - parking_slot[0].position.y();

  parking_slot_heading_vec.normalize();
  parking_slot_lateral_vec.normalize();
  std::vector<Eigen::Vector2d> extended_free_area;

  double open_line_x_diff =
      (parking_slot[0].position.x() - parking_slot[3].position.x());
  double open_line_y_diff =
      (parking_slot[0].position.y() - parking_slot[3].position.y());
  double parking_slot_open_width =
      std::sqrt(open_line_x_diff * open_line_x_diff +
                open_line_y_diff * open_line_y_diff);
  bool is_parallel_slot = common_handler->isParalleSlot();
  if (is_parallel_slot) {
    parking_slot_open_width =
        CarParams::GetInstance()->vehicle_length + 0.2 + 
        common_handler->getParams().groundline_save_area_extend_width;
  } else {
    parking_slot_open_width = 1.6 + common_handler->getParams().groundline_save_area_extend_width;
  }
  double parking_slot_depth =
      std::hypot(parking_slot[0].position.x() - parking_slot[1].position.x(),
                 parking_slot[0].position.y() - parking_slot[1].position.y());

  // if (parking_slot_depth < parking_slot_open_width) {
  //   return true;
  // }

  double area_exten_length_out =
      common_handler->getParams().vertical_passage_width;
  if (is_parallel_slot) {
    area_exten_length_out = common_handler->getParams().parallel_passage_width;
  }

  double GROUNDLINE_SAVE_AREA_EXTEND_LENGTH_OUT =
      common_handler->isParalleSlot()
          ? common_handler->getParams().parallel_passage_width
          : common_handler->getParams().vertical_passage_width;
  if (common_handler->isObliqueSlot()) {
    GROUNDLINE_SAVE_AREA_EXTEND_LENGTH_OUT =
        common_handler->getParams().oblique_passage_width;
  }
  Eigen::Vector2d p0 =
      GROUNDLINE_SAVE_AREA_EXTEND_LENGTH_OUT * parking_slot_heading_vec +
      (parking_slot_open_width - common_handler->getParams().groundline_save_area_extend_width) / 2 *
          parking_slot_lateral_vec +
      parking_slot_open_center;
  Eigen::Vector2d p1 =
      GROUNDLINE_SAVE_AREA_EXTEND_LENGTH_OUT * parking_slot_heading_vec -
      (parking_slot_open_width - common_handler->getParams().groundline_save_area_extend_width) / 2 *
          parking_slot_lateral_vec +
      parking_slot_open_center;
  Eigen::Vector2d p2 =
      -common_handler->getParams().slot_in_extend_depth * parking_slot_heading_vec -
      (parking_slot_open_width - common_handler->getParams().groundline_save_area_extend_width) / 2 *
          parking_slot_lateral_vec +
      parking_slot_open_center;
  Eigen::Vector2d p3 =
      -common_handler->getParams().slot_in_extend_depth * parking_slot_heading_vec +
      (parking_slot_open_width - common_handler->getParams().groundline_save_area_extend_width) / 2 *
          parking_slot_lateral_vec +
      parking_slot_open_center;
  extended_free_area.push_back(p0);
  extended_free_area.push_back(p1);
  extended_free_area.push_back(p2);
  extended_free_area.push_back(p3);
  for (auto &p : object_fusion.ground_line.ground_line_data) {
    if ((int)p.track_id == parking_slot_union.track_id_ &&
        (p.type.value == (int)msquare::GroundLineType::
                             GROUND_LINE_USS_TYPE_SIDE_POINT_VEHICLE ||
         p.type.value == (int)msquare::GroundLineType::
                             GROUND_LINE_USS_TYPE_SIDE_POINT_ONLY_USS_UNKOWN ||
         p.type.value ==
             (int)msquare::GroundLineType::GROUND_LINE_USS_TYPE_SIDE_POINT ||
         p.type.value == (int)msquare::GroundLineType::
                             GROUND_LINE_USS_TYPE_SIDE_POINT_WALL ||
         p.type.value == (int)msquare::GroundLineType::
                             GROUND_LINE_USS_TYPE_SIDE_POINT_PILLAR ||
         p.type.value == (int)msquare::GroundLineType::
                             GROUND_LINE_USS_TYPE_SIDE_POINT_FENCE ||
         p.type.value == (int)msquare::GroundLineType::
                             GROUND_LINE_USS_TYPE_SIDE_POINT_STEP ||
         p.type.value == (int)msquare::GroundLineType::
                             GROUND_LINE_USS_TYPE_SIDE_POINT_SPECIAL)) {

      for (auto &point : p.local_points_fusion) {
        Eigen::Vector2d pos(point.x, point.y);
        if (putils::Polygon::pointInPolygon(pos, extended_free_area)) {
          is_occupied = true;
          return is_occupied;
        }
      }
    }
  }

  return is_occupied;
}

void CommonHandler::generateObjFusionToSlotFrame() {
  all_obj_fusions_to_slot_frame_.clear();
  opposite_obj_fusions_to_slot_frame_.clear();
  side_obj_fusions_to_slot_frame_.clear();
  obj_fusions_to_oblique_slot_frame_.clear();
  // Eigen::Isometry3d passage_frame_tf_inv = getSlotFrameTF();
  for (auto &p : object_fusion_.ground_line.ground_line_data) {
    if ((int)p.track_id == fusion_slot_.track_id_) {
      for (auto &point : p.local_points_fusion) {
        Eigen::Vector3d pos(point.x, point.y, point.z);
        Eigen::Vector3d pos_in_oblique_slot_frame = oblique_slot_frame_tf_inv_ * pos;
        Eigen::Vector3d pos_in_passage_frame = slot_frame_tf_inv_ * pos;
        if (p.type.value ==
            (int)msquare::GroundLineType::GROUND_LINE_USS_TYPE_POINT) {
          opposite_obj_fusions_to_slot_frame_.emplace_back(
              pos_in_passage_frame);
        } else if (p.type.value ==
                       (int)msquare::GroundLineType::
                           GROUND_LINE_USS_TYPE_SIDE_POINT_VEHICLE ||
                   p.type.value ==
                       (int)msquare::GroundLineType::
                           GROUND_LINE_USS_TYPE_SIDE_POINT_ONLY_USS_UNKOWN ||
                   p.type.value == (int)msquare::GroundLineType::
                                       GROUND_LINE_USS_TYPE_SIDE_POINT ||
                   p.type.value == (int)msquare::GroundLineType::
                                       GROUND_LINE_USS_TYPE_SIDE_POINT_WALL ||
                   p.type.value == (int)msquare::GroundLineType::
                                       GROUND_LINE_USS_TYPE_SIDE_POINT_PILLAR ||
                   p.type.value == (int)msquare::GroundLineType::
                                       GROUND_LINE_USS_TYPE_SIDE_POINT_FENCE ||
                   p.type.value == (int)msquare::GroundLineType::
                                       GROUND_LINE_USS_TYPE_SIDE_POINT_STEP ||
                   p.type.value ==
                       (int)msquare::GroundLineType::
                           GROUND_LINE_USS_TYPE_SIDE_POINT_SPECIAL) {
          side_obj_fusions_to_slot_frame_.emplace_back(pos_in_passage_frame);
        }
        all_obj_fusions_to_slot_frame_.emplace_back(pos_in_passage_frame);
        obj_fusions_to_oblique_slot_frame_.emplace_back(pos_in_oblique_slot_frame);
        obj_pos_vec_.emplace_back(pos);
      }
    }
  }

  // std::copy(all_obj_fusions_to_slot_frame_.begin(),
  // all_obj_fusions_to_slot_frame_.end(),
  //       std::back_inserter(opposite_obj_fusions_to_slot_frame_));
  // std::copy(all_obj_fusions_to_slot_frame_.begin(),
  // all_obj_fusions_to_slot_frame_.end(),
  //       std::back_inserter(side_obj_fusions_to_slot_frame_));
}

std::vector<Eigen::Vector3d>
CommonHandler::generateBottomObjFusionToSlotFrame(bool *has_wall) {
  std::vector<Eigen::Vector3d> bottom_obj_fusions_to_slot_frame;
  for (auto &p : object_fusion_.ground_line.ground_line_data) {
    if ((int)p.track_id != fusion_slot_.track_id_)
      continue;

    if (p.type.value ==
            (int)msquare::GroundLineType::GROUND_LINE_USS_TYPE_UNKNOWN ||
        p.type.value ==
            (int)msquare::GroundLineType::GROUND_LINE_USS_TYPE_WALL ||
        p.type.value ==
            (int)msquare::GroundLineType::GROUND_LINE_USS_TYPE_PILLAR ||
        p.type.value ==
            (int)msquare::GroundLineType::GROUND_LINE_USS_TYPE_FENCE ||
        p.type.value ==
            (int)msquare::GroundLineType::GROUND_LINE_USS_TYPE_STEP ||
        p.type.value ==
            (int)msquare::GroundLineType::GROUND_LINE_USS_TYPE_SPECIAL ||
        p.type.value ==
            (int)msquare::GroundLineType::GROUND_LINE_USS_TYPE_VEHICLE ||
        p.type.value ==
            (int)msquare::GroundLineType::GROUND_LINE_USS_TYPE_ONLY_USS_UNKNOWN) {
      for (auto &point : p.local_points_fusion) {
        Eigen::Vector3d pos(point.x, point.y, 0);
        Eigen::Vector3d pos_in_slot_frame = getSlotFrameTF() * pos;
        if (p.type.value !=
            (int)msquare::GroundLineType::GROUND_LINE_USS_TYPE_STEP) {
          *has_wall = true;
        }
        bottom_obj_fusions_to_slot_frame.emplace_back(pos_in_slot_frame);
      }
    }
  }
  return bottom_obj_fusions_to_slot_frame;
}

bool CommonHandler::isParalleSlot() {
  std::vector<ParkingSlotPoint> parking_slot_corners = fusion_slot_.corners_;
  double parking_slot_open_width =
      std::hypot(parking_slot_corners[0].position.x() -
                     parking_slot_corners[3].position.x(),
                 parking_slot_corners[0].position.y() -
                     parking_slot_corners[3].position.y());
  double parking_slot_depth =
      std::hypot(parking_slot_corners[0].position.x() -
                     parking_slot_corners[1].position.x(),
                 parking_slot_corners[0].position.y() -
                     parking_slot_corners[1].position.y());

  return parking_slot_open_width > parking_slot_depth;
}

bool CommonHandler::isSpaceSlot() {
  return fusion_slot_.source_from_uss_ && !fusion_slot_.source_from_vision_;
}

bool CommonHandler::isLineSlot() { return fusion_slot_.source_from_vision_; }

bool CommonHandler::isObliqueSlot() {
  if (isParalleSlot()) {
    return false;
  }

  Eigen::Vector3d psd_center{0.0, 0.0, 0.0};
  for (auto &pt : fusion_slot_.corners_) {
    psd_center += pt.position;
  }
  psd_center = psd_center / fusion_slot_.corners_.size();

  Eigen::Vector3d psd_center_ego = ego_pose_.inverse() * psd_center;
  if (fusion_slot_.corners_.size() == 4) {
    Eigen::Vector3d psd_front_center = (fusion_slot_.corners_[0].position +
                                        fusion_slot_.corners_[3].position) /
                                       2.0;
    Eigen::Vector3d psd_front_center_ego =
        ego_pose_.inverse() * psd_front_center;
    Eigen::Vector3d psd_direction = (psd_front_center_ego - psd_center_ego);
    psd_direction.normalize();
    return std::abs(psd_direction.x()) > std::sin(getParams().yaw_limit);
  }
  return false;
}

bool CommonHandler::isDriveAlongXPositive() {
  std::vector<ParkingSlotPoint> parking_slot = getFusionSlot().corners_;
  Eigen::Vector3d parking_slot_front_center =
      (parking_slot[0].position + parking_slot[3].position) / 2;
  Eigen::Vector3d parking_slot_front_lateral_vec =
      parking_slot[3].position - parking_slot[0].position;

  // 构造参考坐标系的tf矩阵
  double passage_frame_yaw;
  passage_frame_yaw = std::atan2(parking_slot_front_lateral_vec.y(),
                                 parking_slot_front_lateral_vec.x());
  return std::cos(yaw_ - passage_frame_yaw) > 0;
}

std::vector<Eigen::Vector3d> CommonHandler::generateObjFusionToEgoPoseFrame() {
  std::vector<Eigen::Vector3d> obj_fusions_to_ego_pose_frame;
  // std::cout << "============" << ego_pose_.translation().x() << "  "
  //           << " " << ego_pose_.translation().y() << std::endl;
  for (auto &p : object_fusion_.ground_line.ground_line_data) {
    if ((int)p.track_id == fusion_slot_.track_id_ &&
        (p.type.value == (int)msquare::GroundLineType::
                             GROUND_LINE_USS_TYPE_SIDE_POINT_VEHICLE ||
         p.type.value == (int)msquare::GroundLineType::
                             GROUND_LINE_USS_TYPE_SIDE_POINT_ONLY_USS_UNKOWN ||
         p.type.value ==
             (int)msquare::GroundLineType::GROUND_LINE_USS_TYPE_SIDE_POINT ||
         p.type.value == (int)msquare::GroundLineType::
                             GROUND_LINE_USS_TYPE_SIDE_POINT_WALL ||
         p.type.value == (int)msquare::GroundLineType::
                             GROUND_LINE_USS_TYPE_SIDE_POINT_PILLAR ||
         p.type.value == (int)msquare::GroundLineType::
                             GROUND_LINE_USS_TYPE_SIDE_POINT_FENCE ||
         p.type.value == (int)msquare::GroundLineType::
                             GROUND_LINE_USS_TYPE_SIDE_POINT_STEP ||
         p.type.value == (int)msquare::GroundLineType::
                             GROUND_LINE_USS_TYPE_SIDE_POINT_SPECIAL ||
         p.type.value ==
             (int)msquare::GroundLineType::GROUND_LINE_USS_TYPE_POINT)) {
      for (auto &point : p.local_points_fusion) {
        Eigen::Vector3d point_car =
            ego_pose_.inverse() * Eigen::Vector3d(point.x, point.y, point.z);
        obj_fusions_to_ego_pose_frame.emplace_back(std::move(point_car));
      }
    }
  }
  return obj_fusions_to_ego_pose_frame;
}

void CommonHandler::generateSlotFrame() {
  std::vector<ParkingSlotPoint> parking_slot = fusion_slot_.corners_;
  Eigen::Vector3d parking_slot_front_center =
      (parking_slot[0].position + parking_slot[3].position) / 2;
  Eigen::Vector3d parking_slot_front_lateral_vec =
      parking_slot[3].position - parking_slot[0].position;
  Eigen::Vector3d parking_slot_side_lateral_vec =
      parking_slot[1].position - parking_slot[0].position;

  // 构造库位坐标系
  double slot_frame_x = parking_slot_front_center.x();
  double slot_frame_y = parking_slot_front_center.y();
  double slot_frame_yaw = std::atan2(parking_slot_front_lateral_vec.y(),
                                     parking_slot_front_lateral_vec.x());
  double oblique_slot_frame_yaw = std::atan2(parking_slot_side_lateral_vec.y(),
                                  parking_slot_side_lateral_vec.x()) + M_PI / 2;

  Eigen::Matrix3d rot_max;
  Eigen::Vector3d trans_max(slot_frame_x, slot_frame_y, 0);
  Eigen::Isometry3d slot_frame_tf = Eigen::Isometry3d::Identity();
  
  rot_max = (Eigen::AngleAxisd(isObliqueSlot() ? oblique_slot_frame_yaw : slot_frame_yaw,
                              Eigen::Vector3d::UnitZ()));
  slot_frame_tf.rotate(rot_max);
  slot_frame_tf.pretranslate(trans_max);
  oblique_slot_frame_tf_inv_ = slot_frame_tf.inverse();

  if (isObliqueSlot()) {
    Eigen::Isometry3d raw_frame_tf = Eigen::Isometry3d::Identity();
    rot_max = (Eigen::AngleAxisd(slot_frame_yaw, Eigen::Vector3d::UnitZ()));
    raw_frame_tf.rotate(rot_max);
    raw_frame_tf.pretranslate(trans_max);
    slot_frame_tf_inv_ = raw_frame_tf.inverse();
  } else {
    slot_frame_tf_inv_ = oblique_slot_frame_tf_inv_;
  }
}

Eigen::Vector3d CommonHandler::getEgoPoseToSlotFrame() {
  Eigen::Vector3d ego_pose(ego_pose_.translation().x(),
                           ego_pose_.translation().y(), 0);
  return slot_frame_tf_inv_ * ego_pose;
}

std::pair<double, double> CommonHandler::getSlotWidthAndDepth() {
  std::vector<ParkingSlotPoint> parking_slot = fusion_slot_.corners_;
  double parking_slot_open_width =
      std::hypot(parking_slot[0].position.x() - parking_slot[3].position.x(),
                 parking_slot[0].position.y() - parking_slot[3].position.y());
  double parking_slot_depth =
      std::hypot(parking_slot[0].position.x() - parking_slot[1].position.x(),
                 parking_slot[0].position.y() - parking_slot[1].position.y());
  return std::make_pair(parking_slot_open_width, parking_slot_depth);
}

void EnvFeatureHandler::extractFeature() {
  // if (common_handler_->isSpaceSlot()) {
  //   extractFeaturSlotWidthFromSpaceSlot();
  //   extractOpenAreaOutLengthFromSpaceSlot();
  // } else {
  //   extractFeaturSlotLeftAndRightWidth();
  //   extractOpenAreaOutLength();
  // }
  extractFeaturSlotLeftAndRightWidth();
  extractOpenAreaOutLength();
  extractPassageWidth();
  extractDeadEndRoad();
}

void EnvFeatureHandler::extractFeaturSlotWidthFromSpaceSlot() {
  F_1_slot_left_width_ = -common_handler_->getSlotWidthAndDepth().first / 2;
  F_2_slot_right_width_ = common_handler_->getSlotWidthAndDepth().first / 2;
  //  std::cout << " F_1_slot_left_width_: " << F_1_slot_left_width_
  //            << " F_2_slot_right_width_: " << F_2_slot_right_width_ <<
  //            std::endl;
}

void EnvFeatureHandler::extractOpenAreaOutLengthFromSpaceSlot() {
  std::vector<Eigen::Vector3d> obj_fusions_in_slot_frame =
      common_handler_->getAllObjFusionToSlotFrame();
  Eigen::Vector3d ego_pose_in_slot_frame =
      common_handler_->getEgoPoseToSlotFrame();
  double extend_out_dis = 10.0;
  double extend_in_dis = -10.0;

  for (const auto &obj_fusion : obj_fusions_in_slot_frame) {
    if (obj_fusion.x() > F_2_slot_right_width_ ||
        obj_fusion.x() < F_1_slot_left_width_ || obj_fusion.y() < 0) {
      continue;
    }
    extend_out_dis = std::min(extend_out_dis, obj_fusion.y());
  }
  F_3_slot_open_area_out_length_ = extend_out_dis;
  //  std::cout << "F_3_slot_open_area_out_length_: "
  //            << F_3_slot_open_area_out_length_
  //            << std::endl;
}

void EnvFeatureHandler::extractFeaturSlotLeftAndRightWidth() {
  std::vector<Eigen::Vector3d> obj_fusions_to_slot_frame =
      common_handler_->getObjFusionToObliqueSlotFrame();
  Eigen::Vector3d ego_pose_to_slot_frame =
      common_handler_->getEgoPoseToSlotFrame();
  Eigen::Vector3d psd_front_center =
      (common_handler_->getFusionSlot().corners_[0].position +
       common_handler_->getFusionSlot().corners_[3].position) /
      2.0;
  Eigen::Vector3d psd_front_center_ego =
      common_handler_->getEgoPose().inverse() * psd_front_center;
  double area_exten_length_out =
      common_handler_->getParams().vertical_passage_width;
  if (common_handler_->isParalleSlot()) {
    area_exten_length_out = common_handler_->getParams().parallel_passage_width;
  }
  const auto &slot_width_and_depth = common_handler_->getSlotWidthAndDepth();
  double parking_slot_depth = slot_width_and_depth.second;
  F_4_slot_open_area_in_length_ = - (2 * parking_slot_depth) / 3;
  // F_4_slot_open_area_in_length_ =
  //     -std::fabs(5 - std::fabs(psd_front_center_ego.y()) + HALF_WIDTH);
  // if (common_handler_->isParalleSlot()) {
  //   F_4_slot_open_area_in_length_ = - (2 * parking_slot_depth) / 3;
  // }

  // if (common_handler_->isObliqueSlot()) {
  //   F_4_slot_open_area_in_length_ = -common_handler_->getParams().slot_in_extend_depth;
  // }

  double left_side_max_x = -10.0;
  double right_side_min_x = 10.0;
  size_t right_side_min_index = 0;
  size_t left_side_max_index = 0;
  size_t current_index = 0;

  for (const auto &obj_fusion : obj_fusions_to_slot_frame) {
    if (obj_fusion.y() > ego_pose_to_slot_frame.y() ||
        obj_fusion.y() < F_4_slot_open_area_in_length_) {
      ++current_index;
      continue;
    }
    if (obj_fusion.x() > 0.0 && obj_fusion.x() < right_side_min_x) {
      right_side_min_x = obj_fusion.x();
      right_side_min_index = current_index;
    }
    if (obj_fusion.x() < 0.0 && obj_fusion.x() > left_side_max_x) {
      left_side_max_x = obj_fusion.x();
      left_side_max_index = current_index;
    }
    ++current_index;
  }
  F_1_slot_left_width_ = left_side_max_x;
  F_2_slot_right_width_ = right_side_min_x;
  // auto obj_vec = common_handler_->getObjPosVec();
  // if ((left_side_max_index < obj_vec.size()) && (right_side_min_index < obj_vec.size())) {
  //   std::cout << "track id: " << common_handler_->getFusionSlot().track_id_ << std::endl;
  //   std::cout << "left max pos: " << obj_vec[left_side_max_index].x()
  //             << ", " << obj_vec[left_side_max_index].y() <<
  //             std::endl;
  //   std::cout << "right min pos: " << obj_vec[right_side_min_index].x()
  //             << ", " << obj_vec[right_side_min_index].y() <<
  //             std::endl;
  // }
}

void EnvFeatureHandler::extractOpenAreaOutLength() {
  std::vector<Eigen::Vector3d> obj_fusions_in_slot_frame =
      common_handler_->getAllObjFusionToSlotFrame();
  Eigen::Vector3d ego_pose_in_slot_frame =
      common_handler_->getEgoPoseToSlotFrame();
  double extend_out_dis = 10.0;
  double extend_in_dis = -10.0;

  double virtual_slot_width = 1.6;
  if (!common_handler_->isApaMode()) {
    virtual_slot_width = 1.4;
  }
  if (common_handler_->isParalleSlot()) {
    virtual_slot_width = CarParams::GetInstance()->vehicle_length + 0.2;
    if (!common_handler_->isApaMode()) {
      virtual_slot_width = CarParams::GetInstance()->vehicle_length;
    }
  }

  for (const auto &obj_fusion : obj_fusions_in_slot_frame) {
    if (obj_fusion.x() > 0.5 * virtual_slot_width ||
        obj_fusion.x() < -0.5 * virtual_slot_width ||
        obj_fusion.y() < ego_pose_in_slot_frame.y()) {
      continue;
    }
    extend_out_dis = std::min(extend_out_dis, obj_fusion.y());
  }

  // for(const auto& obj_fusion : obj_fusions_in_slot_frame) {
  //   if (obj_fusion.x() > 0.5 * virtual_slot_width ||
  //       obj_fusion.x() < -0.5 * virtual_slot_width ||
  //       obj_fusion.y() > 0) {
  //         continue;
  //   }
  //   extend_in_dis = std::max(extend_in_dis, obj_fusion.y());
  // }
  F_3_slot_open_area_out_length_ = extend_out_dis;
  // F_7_slot_open_area_in_length_ = extend_in_dis;
  // std::cout << "F_3_slot_open_area_out_length_: "
  //           << F_3_slot_open_area_out_length_
  //           << std::endl;
}

void EnvFeatureHandler::extractPassageWidth() {
  // const auto& obj_fusions_in_slot_frame =
  // common_handler_.generateObjFusionToSlotFrame(); double oppo_y = 10; double
  // side_y = 0;

  bool is_drive_along_x_positive = common_handler_->isDriveAlongXPositive();
  // std::cout << "is_drive_along_x_positive:" << is_drive_along_x_positive <<
  // std::endl;

  double half_slot_right_x = common_handler_->getSlotWidthAndDepth().first / 2;
  if (is_drive_along_x_positive) {
    F_5_passage_extend_length_ = half_slot_right_x + common_handler_->getParams().valid_passage_length;
  } else {
    F_5_passage_extend_length_ = -half_slot_right_x - common_handler_->getParams().valid_passage_length;
  }
  double oppo_y = 10;
  double side_y = 0;

  Eigen::Isometry3d passage_frame_tf_inv = common_handler_->getSlotFrameTF();
  const auto &side_obj_fusions = common_handler_->getSideObjFusionToSlotFrame();
  const auto &opposite_obj_fusuion =
      common_handler_->getOppositeObjFusionToSlotFrame();

  // std::cout << "the half_slot_right_x:" << half_slot_right_x << std::endl;

  for (const auto &obj_fusion : side_obj_fusions) {
    if (is_drive_along_x_positive) {
      if (obj_fusion.x() > half_slot_right_x + common_handler_->getParams().valid_passage_length ||
          obj_fusion.x() < half_slot_right_x) {
        continue;
      }
    } else {
      if (obj_fusion.x() < -half_slot_right_x - common_handler_->getParams().valid_passage_length ||
          obj_fusion.x() > -half_slot_right_x) {
        continue;
      }
    }
    side_y = std::max(side_y, obj_fusion.y());
  }

  for (const auto &obj_fusion : opposite_obj_fusuion) {
    if (is_drive_along_x_positive) {
      if (obj_fusion.x() > half_slot_right_x + common_handler_->getParams().valid_passage_length ||
          obj_fusion.x() < half_slot_right_x) {
        continue;
      }
    } else {
      if (obj_fusion.x() < -half_slot_right_x - common_handler_->getParams().valid_passage_length ||
          obj_fusion.x() > -half_slot_right_x) {
        continue;
      }
    }
    oppo_y = std::min(oppo_y, obj_fusion.y());
  }
  // std::cout << "=========the oppo_y is:" << oppo_y << "   " << side_y <<
  // std::endl;
  F_6_passage_width_ = oppo_y - side_y;
  // std::cout << "F_6_passage_width_: " << F_6_passage_width_ << std::endl;
}

void EnvFeatureHandler::extractDeadEndRoad() {
  std::vector<Eigen::Vector3d> obj_fusions_to_slot_frame =
      common_handler_->getAllObjFusionToSlotFrame();
  Eigen::Vector3d ego_pose_to_slot_frame =
      common_handler_->getEgoPoseToSlotFrame();
  bool is_drive_along_x_positive = common_handler_->isDriveAlongXPositive();

  double min_length = 10;
  double max_length = -10;
  double half_width = CarParams::GetInstance()->vehicle_width / 2;
  for (const auto &obj_fusion : obj_fusions_to_slot_frame) {
    if (is_drive_along_x_positive) {
      if (obj_fusion.y() < ego_pose_to_slot_frame.y() + half_width &&
          obj_fusion.y() > ego_pose_to_slot_frame.y() - half_width &&
          obj_fusion.x() > 0) {
        min_length = std::min(min_length, obj_fusion.x());
      }
    } else {
      if (obj_fusion.y() < ego_pose_to_slot_frame.y() + half_width &&
          obj_fusion.y() > ego_pose_to_slot_frame.y() - half_width &&
          obj_fusion.x() < 0) {
        max_length = std::max(max_length, obj_fusion.x());
      }
    }
  }
  if (is_drive_along_x_positive) {
    F_8_dead_end_road_length_ = min_length;
  } else {
    F_8_dead_end_road_length_ = max_length;
  }
}

bool EnvFeatureHandler::isSlotOpenAreaWidthEnough(
    SlotReleaseManager::ParkingSlotUnion &union_slot) {
  double safe_slot_width = common_handler_->getParams().vertical_slot_width;
  if (!common_handler_->isApaMode()) {
    safe_slot_width = 2.3;
  }
  safe_slot_width = safe_slot_width + 
    CarParams::GetInstance()->vehicle_width_real - L_7_VEHICLE_WIDTH_REAL;
  if (common_handler_->isObliqueSlot()) {
  } else if (common_handler_->isParalleSlot()) {
    safe_slot_width = CarParams::GetInstance()->vehicle_length + 0.6;
  }

  double virtual_slot_width_limit = 1.6;
  if (!common_handler_->isApaMode()) {
    safe_slot_width = 1.4;
  }
  if (common_handler_->isParalleSlot()) {
    virtual_slot_width_limit = CarParams::GetInstance()->vehicle_length + 0.2;
    if (!common_handler_->isApaMode()) {
      safe_slot_width = CarParams::GetInstance()->vehicle_length;
    }
  }

  double half_open_width = virtual_slot_width_limit / 2;
  double min_slot_width_diff = 0.15;

  union_slot.slot_left_open_width_trigger_.changeThresholds(
      half_open_width - min_slot_width_diff, half_open_width);

  union_slot.slot_right_open_width_trigger_.changeThresholds(
      half_open_width - min_slot_width_diff, half_open_width);

  if (std::fabs(std::fabs(F_1_slot_left_width_) -
                union_slot.pre_left_open_width_) > 3.0) {
    union_slot.slot_left_open_width_trigger_.resetPos(true);
  }

  if (std::fabs(std::fabs(F_2_slot_right_width_) -
                union_slot.pre_right_open_width_) > 3.0) {
    union_slot.slot_right_open_width_trigger_.resetPos(true);
  }

  union_slot.pre_left_open_width_ = F_1_slot_left_width_;
  union_slot.pre_right_open_width_ = F_2_slot_right_width_;

  if (!common_handler_->isSpaceSlot()) {
    if (union_slot.slot_left_open_width_trigger_.evaluate(
            std::fabs(F_1_slot_left_width_)) ||
        union_slot.slot_right_open_width_trigger_.evaluate(
            std::fabs(F_2_slot_right_width_))) {
      return false;
    }
  }

  double slot_width_diff = 
      common_handler_->getParams().vertical_slot_width_hys_scope;
  double virtual_slot_wdith =
      std::fabs(F_2_slot_right_width_ - F_1_slot_left_width_);

  union_slot.slot_open_width_trigger_.changeThresholds(
      safe_slot_width - slot_width_diff, safe_slot_width);

  if (std::fabs(virtual_slot_wdith - union_slot.pre_slot_open_width_) >
      3.0 * 2) {
    union_slot.slot_open_width_trigger_.resetPos(true);
  }

  union_slot.pre_slot_open_width_ = virtual_slot_wdith;

  if (union_slot.slot_open_width_trigger_.evaluate(virtual_slot_wdith)) {
    return false;
  }

  // if(common_handler_->isSpaceSlot()) {
  //   if(std::fabs(F_2_slot_right_width_ - F_1_slot_left_width_) >
  //   safe_slot_width) {
  //       return true;
  //     }
  // } else {
  //   if(std::fabs(F_2_slot_right_width_ - F_1_slot_left_width_) >
  //   safe_slot_width
  //     && std::fabs(F_2_slot_right_width_) > virtual_slot_width_limit / 2
  //     && std::fabs(F_1_slot_left_width_) > virtual_slot_width_limit / 2) {
  //       return true;
  //   }
  // }
  return true;
}

bool EnvFeatureHandler::isSlotOpenAreaLengthEnough(
    SlotReleaseManager::ParkingSlotUnion &union_slot) {
  double safe_extend_out_slot_length =
      common_handler_->getParams().vertical_passage_width;
  if (!common_handler_->isApaMode()) {
    safe_extend_out_slot_length = 4.0;
  }
  if (common_handler_->isObliqueSlot()) {
    safe_extend_out_slot_length =
        common_handler_->getParams().oblique_passage_width;
    if (!common_handler_->isApaMode()) {
      safe_extend_out_slot_length = 4.0;
    }
  } else if (common_handler_->isParalleSlot()) {
    safe_extend_out_slot_length =
        common_handler_->getParams().parallel_passage_width;
  }

  // if(common_handler_->isSpaceSlot()) {
  //   if(F_3_slot_open_area_out_length_ > safe_extend_out_slot_length) {
  //       return true;
  //     }
  // } else {
  //   if(F_3_slot_open_area_out_length_ > safe_extend_out_slot_length) {
  //       return true;
  //   }
  // }

  double slot_length_diff = 
      common_handler_->getParams().vertical_passage_width_hys_scope;
  union_slot.slot_open_length_trigger_.changeThresholds(
      safe_extend_out_slot_length - slot_length_diff,
      safe_extend_out_slot_length);

  if (std::fabs(F_3_slot_open_area_out_length_ -
                union_slot.pre_slot_open_length_) > 3.0) {
    union_slot.slot_open_length_trigger_.resetPos(true);
  }
  union_slot.pre_slot_open_length_ = F_3_slot_open_area_out_length_;

  return !union_slot.slot_open_length_trigger_.evaluate(
      F_3_slot_open_area_out_length_);
}

bool EnvFeatureHandler::isPassageWidthEnough(
    SlotReleaseManager::ParkingSlotUnion &union_slot) {
  double reasonable_passage_width = 
    common_handler_->getParams().vertical_passage_width;
  if (!common_handler_->isApaMode()) {
    reasonable_passage_width = 4.0;
  }
  if (common_handler_->isParalleSlot()) {
    reasonable_passage_width = 
      common_handler_->getParams().parallel_passage_width;
  } else if (common_handler_->isObliqueSlot()) {
    reasonable_passage_width = 
      common_handler_->getParams().oblique_passage_width;
    if (!common_handler_->isApaMode()) {
      reasonable_passage_width = 4.0;
    }
  }
  double passage_width_diff = 
    common_handler_->getParams().vertical_passage_width_hys_scope;
  union_slot.passage_width_trigger_.changeThresholds(
      reasonable_passage_width - passage_width_diff, reasonable_passage_width);
  if (std::fabs(F_6_passage_width_ - union_slot.pre_passage_width_) > 3.0) {
    union_slot.passage_width_trigger_.resetPos(true);
  }
  union_slot.pre_passage_width_ = F_6_passage_width_;
  return !union_slot.passage_width_trigger_.evaluate(F_6_passage_width_);
}

bool EnvFeatureHandler::isDeadEndRoad() {

  double end_dead_road_limit = 5.5;
  double half_slot_width = common_handler_->getSlotWidthAndDepth().first / 2;
  if (std::fabs(F_8_dead_end_road_length_) <
      half_slot_width + end_dead_road_limit) {
    return true;
  } else {
    return false;
  }
}

void EgoCarFeatureHandler::extractFeature() {
  extractSlotToEgoPose();
  extractSlotToHistoryTrajPose();
}

void EgoCarFeatureHandler::extractSlotToEgoPose() {
  maf_perception_interface::Point3f corner_pt{};
  Eigen::Vector3d psd_center{0.0, 0.0, 0.0};
  for (auto &pt : common_handler_->getFusionSlot().corners_) {
    psd_center += pt.position;
  }
  psd_center = psd_center / common_handler_->getFusionSlot().corners_.size();

  corner_pt.x = psd_center.x();
  corner_pt.y = psd_center.y();
  corner_pt.z = psd_center.z();

  Eigen::Vector3d corner_pt_self =
      common_handler_->getEgoPose().inverse() *
      Eigen::Vector3d(corner_pt.x, corner_pt.y, corner_pt.z);
  if (common_handler_->getFusionSlot().corners_.size() == 4) {
    Eigen::Vector3d psd_front_center =
        (common_handler_->getFusionSlot().corners_[0].position +
         common_handler_->getFusionSlot().corners_[3].position) /
        2.0;
    Eigen::Vector3d psd_front_center_ego =
        common_handler_->getEgoPose().inverse() * psd_front_center;
    Eigen::Vector3d psd_direction = (psd_front_center_ego - corner_pt_self);
    psd_direction.normalize();
    F_6_ego_to_slot_x_ = psd_front_center_ego.x();
    F_7_ego_to_slot_y_ = psd_front_center_ego.y();
    F_8_ego_to_slot_theta_ = psd_direction;
  }
  // std::cout << " F_6_ego_to_slot_x_: " << F_6_ego_to_slot_x_
  //           << " F_7_ego_to_slot_y_: " << F_7_ego_to_slot_y_
  //           << " F_8_ego_to_slot_theta_: " << F_8_ego_to_slot_theta_
  //           << std::endl;
}

void EgoCarFeatureHandler::extractSlotToHistoryTrajPose() {
  auto slot_to_pose_releative_pos_list =
      [](const ParkingSlotElement fusion_slot, const Eigen::Isometry3d &pose) {
        std::vector<Eigen::Vector3d> corner_pts_self;
        for (auto &pt : fusion_slot.corners_) {
          maf_perception_interface::Point3f corner_pt{};
          corner_pt.x = pt.position.x();
          corner_pt.y = pt.position.y();
          corner_pt.z = pt.position.z();
          Eigen::Vector3d corner_pt_self =
              pose.inverse() *
              Eigen::Vector3d(corner_pt.x, corner_pt.y, corner_pt.z);
          corner_pts_self.push_back(corner_pt_self);
        }

        std::sort(corner_pts_self.begin(), corner_pts_self.end(),
                  [](const auto &pre, const auto &next) {
                    return pre.x() < next.x();
                  });
        return corner_pts_self;
      };

  auto is_pt_valid = [this](const ParkingSlotElement fusion_slot,
                        const Eigen::Isometry3d &pose, bool is_parallel) {
    if (fusion_slot.corners_.size() == 4) {
      Eigen::Vector3d psd_front_center = (fusion_slot.corners_[0].position +
                                          fusion_slot.corners_[3].position) /
                                         2.0;
      Eigen::Vector3d psd_front_center_ego = pose.inverse() * psd_front_center;
      double uss_friendly_area_y_max_vertical = common_handler_->getParams().uss_friendly_area_y_max;
      if (VehicleParam::Instance()->car_type == "SG") {
        uss_friendly_area_y_max_vertical = 5.65;
      }
      double uss_frendly_area_y_max = is_parallel
                                          ? common_handler_->getParams().uss_friendly_area_y_max_parallel
                                          : uss_friendly_area_y_max_vertical;
      if (std::abs(psd_front_center_ego.y()) > common_handler_->getParams().uss_friendly_area_y_min &&
          std::abs(psd_front_center_ego.y()) < uss_frendly_area_y_max) {
        return true;
      }
      return false;
    } else {
      return false;
    }
  };

  double delay_compensate_dist =
      F_1_velocity_ * common_handler_->getParams().compensate_time;
  delay_compensate_dist = std::min(
      delay_compensate_dist, common_handler_->getParams().max_compensate_dist);

  double left_max_x = -10;
  double right_min_x = 10;
  for (size_t i = 0; i < history_locs_.size(); ++i) {

    const auto &trajectory_pt = history_locs_[i];
    std::vector<Eigen::Vector3d> corner_pts_self =
        slot_to_pose_releative_pos_list(common_handler_->getFusionSlot(),
                                        trajectory_pt);
    if (!is_pt_valid(common_handler_->getFusionSlot(), trajectory_pt,
                     common_handler_->isParalleSlot())) {
      // std::cout << "the pose is valid" << std::endl;
      continue;
    }
    if (corner_pts_self[1].x() > left_max_x) {
      left_max_x = corner_pts_self[1].x();
    }
    if (corner_pts_self[2].x() < right_min_x) {
      right_min_x = corner_pts_self[2].x();
    }
  }

  const auto &slot_to_ego_rel_pos_vector = slot_to_pose_releative_pos_list(
      common_handler_->getFusionSlot(), common_handler_->getEgoPose());
  F_9_min_trajectory_slot_to_ego_x_ = right_min_x;
  F_10_max_trajectory_slot_to_ego_x_ = left_max_x;
  F_11_slot_to_ego_x_ = slot_to_ego_rel_pos_vector[2].x();
  // std::cout << " F_9_min_trajectory_slot_to_ego_x_: " <<
  // F_9_min_trajectory_slot_to_ego_x_
  //           << " F_10_max_trajectory_slot_to_ego_x_: " <<
  //           F_10_max_trajectory_slot_to_ego_x_
  //           << " F_11_slot_to_ego_x_" << F_11_slot_to_ego_x_ << std::endl;
}

bool EgoCarFeatureHandler::isvelocityReasonable() {
  double velocity_limit = common_handler_->getParams().velocity_limit;
  if (VehicleParam::Instance()->car_type == "SG") {
    velocity_limit = 30.0 / 3.6;
  }
  if (VehicleParam::Instance()->car_type == "LYRIQ") {
    velocity_limit = 25.0 / 3.6;
  }
  return F_1_velocity_ < velocity_limit;
}

bool EgoCarFeatureHandler::isPitchReasonable() {
  return std::fabs(F_2_pitch_) < common_handler_->getParams().pitch_limit;
}

bool EgoCarFeatureHandler::isYawReasonable() {
  int slot_type = common_handler_->getFusionSlot().slot_type_;
  // if (common_handler_->isSpaceSlot()) {
  //   if (!common_handler_->getParams().enable_oblique_slot &&
  //       F_8_ego_to_slot_theta_.x() > std::sin(common_handler_->getParams().space_yam_max_limit)) {
  //     return false;
  //   }
  // } else {
  //   if (!common_handler_->getParams().enable_oblique_slot &&
  //       F_8_ego_to_slot_theta_.x() > std::sin(common_handler_->getParams().yaw_limit)) {
  //     return false;
  //   }
  // }
  if ((slot_type == 3 || slot_type == 4) && F_8_ego_to_slot_theta_.x() < 0) {
    return false;
  }
  return true;
}

bool EgoCarFeatureHandler::isMove() { return F_5_is_move_; }

bool EgoCarFeatureHandler::isParkingStatus() {

  if (F_4_apa_status_ == "apa_parking_in" &&
      common_handler_->getFusionSlot().track_id_ ==
          apa_target_parkingslot_track_id_) {
    return true;
  }
  return false;
}

bool EgoCarFeatureHandler::isCarInSlot() {
  if ((F_4_apa_status_ == "idle" || F_4_apa_status_ == "apa_wait") &&
      common_handler_->getFusionSlot().empty_votes_ > 0) {
    return true;
  }
  return false;
}

bool EgoCarFeatureHandler::isSlotCenterToEgoDisReasonable() {
  if (std::fabs(F_6_ego_to_slot_x_) > common_handler_->getParams().parkable_area_x) {
    return false;
  }

  double uss_frendly_area_y_min = common_handler_->getParams().uss_friendly_area_y_min;
  if (common_handler_->isSpaceSlot()) {
    uss_frendly_area_y_min = common_handler_->getParams().uss_friendly_area_y_min + 0.3;
  }

  double uss_friendly_area_max_y = common_handler_->getParams().uss_friendly_area_y_max;
  if (VehicleParam::Instance()->car_type == "SG") {
    uss_friendly_area_max_y = 5.65;
  }
  if (common_handler_->isParalleSlot()) {
    uss_friendly_area_max_y = common_handler_->getParams().uss_friendly_area_y_max_parallel;
  }
  Eigen::Vector3d ego_pose_to_slot_frame =
      common_handler_->getEgoPoseToSlotFrame();
  if (std::fabs(F_7_ego_to_slot_y_) < uss_frendly_area_y_min ||
      std::fabs(F_7_ego_to_slot_y_) > uss_friendly_area_max_y ||
      ego_pose_to_slot_frame.y() < 0.0) {
    return false;
  }
  return true;
}

bool EgoCarFeatureHandler::isSlotFullyObservedAccordHisTraj(
  SlotReleaseManager::ParkingSlotUnion &union_slot) {
  bool result = false;
  double cur_v = F_1_velocity_;
  if (union_slot.pre_result_ && cur_v < union_slot.pre_vel_) {
    cur_v = union_slot.pre_vel_;
  }
  // double forward_pass_dist = -1.57;
  // if (common_handler_->isParalleSlot()) {
  //   forward_pass_dist = FORWARD_PASS_DIST;
  // }
  double delay_compensate_dist =
      cur_v * common_handler_->getParams().compensate_time;
  double forward_pass_dist = common_handler_->getParams().forward_pass_dist;
  if (VehicleParam::Instance()->car_type == "SG" 
      || VehicleParam::Instance()->car_type == "UXE") {
    double slot_width = common_handler_->getSlotWidthAndDepth().first;
    forward_pass_dist = -2.0;
    if (!common_handler_->isParalleSlot()) {
      forward_pass_dist = 
          forward_pass_dist - CarParams::GetInstance()->vehicle_front_edge_to_rear;
    }
  }
  delay_compensate_dist = std::min(
      delay_compensate_dist, common_handler_->getParams().max_compensate_dist);
  double foward_threshold = 
    -(forward_pass_dist - common_handler_->getSlotWidthAndDepth().first 
    - delay_compensate_dist);
  if (F_9_min_trajectory_slot_to_ego_x_ < foward_threshold
      && F_10_max_trajectory_slot_to_ego_x_ >
          common_handler_->getParams().backward_pass_dist - delay_compensate_dist) {
    result = true;
  }
  union_slot.pre_vel_ = cur_v;
  union_slot.pre_result_ = result;
  return result;
}

bool EgoCarFeatureHandler::isEgoCarPassSlotDownstream() {
  if (!common_handler_->isParalleSlot()) {
    return true;
  }
  return F_11_slot_to_ego_x_ < 0.5;
}

void OtherFeatureHandler::extractFeature() {
  extractParallelWidthAndDepth();
  extractSlotOccupiedFromPSD();
  extractObsToEgoPoseDis();
}

void OtherFeatureHandler::extractObsToEgoPoseDis() {
  double min_obs_dis = 10.0;
  const auto &obj_fusions_to_ego_pose_frame =
      common_handler_->generateObjFusionToEgoPoseFrame();
  for (const auto &obj_fusion : obj_fusions_to_ego_pose_frame) {
    if (obj_fusion.x() > -DIS_TO_REAR && obj_fusion.x() < DIS_TO_FRONT) {
      if (std::fabs(obj_fusion.y()) < min_obs_dis) {
        min_obs_dis = std::fabs(obj_fusion.y());
      }
    }
  }
  F_2_obs_to_ego_y_dis_ = min_obs_dis;
}

void OtherFeatureHandler::extractParallelWidthAndDepth() {
  double min_bottom_line_distance = 2.46;
  bool has_wall = false;
  const auto &bottom_obj_fusions_to_slot_frame =
      common_handler_->generateBottomObjFusionToSlotFrame(&has_wall);
  const auto &slot_width_and_depth = common_handler_->getSlotWidthAndDepth();
  const auto &fusion_slot = common_handler_->getFusionSlot();
  std::vector<ParkingSlotPoint> parking_slot = fusion_slot.corners_;
  Eigen::Vector3d parking_slot_front_center =
      (parking_slot[0].position + parking_slot[3].position) / 2;
  double parking_slot_depth = slot_width_and_depth.second;
  double parking_slot_width = slot_width_and_depth.first;

  std::vector<std::vector<Eigen::Vector2d>> wall_line_points;

  const auto &object_fusion = common_handler_->getObjectionFusion();
  for (auto &p : object_fusion.ground_line.ground_line_data) {
    if ((int)p.track_id != fusion_slot.track_id_)
      continue;
    if (p.local_points_fusion.size() == 2) {
      Eigen::Vector2d first_pt(p.local_points_fusion.front().x,
                               p.local_points_fusion.front().y);
      Eigen::Vector2d second_pt(p.local_points_fusion.back().x,
                                p.local_points_fusion.back().y);
      double to_line_dist = putils::MathHelper::pointDistanceToLine(
          Eigen::Vector2d(parking_slot_front_center.x(),
                          parking_slot_front_center.y()),
          first_pt, second_pt);
      min_bottom_line_distance =
          std::min(min_bottom_line_distance, to_line_dist);
    }
  }
  double min_parking_slot_depth = common_handler_->getParams().min_length_parallel;
  parking_slot_depth = std::max(parking_slot_depth, min_bottom_line_distance);
  F_5_parallel_slot_bottom_is_wall_ = has_wall;
  F_4_parallel_slot_depth_ = parking_slot_depth;
  F_3_parallel_slot_width_ = parking_slot_width;
  // std::cout << " F_5_parallel_slot_bottom_is_wall_: " << has_wall
  //           << " F_4_parallel_slot_depth_: " << parking_slot_depth
  //           << " F_3_parallel_slot_width_: " << F_3_parallel_slot_width_
  //           << std::endl;
}

void OtherFeatureHandler::extractSlotOccupiedFromPSD() {
  F_6_is_slot_occupied_from_psd_ =
      common_handler_->getFusionSlot().empty_votes_ <= 0;
}

bool OtherFeatureHandler::isSlotOccupiedFromPSD() {
  return F_6_is_slot_occupied_from_psd_;
}

bool OtherFeatureHandler::isObsSafeToEgoCar(
    SlotReleaseManager::ParkingSlotUnion &union_slot) {
  // std::cout <<" F_2_obs_to_ego_y_dis_: " << F_2_obs_to_ego_y_dis_ <<
  // std::endl;
  double safe_dis_to_obs = CarParams::GetInstance()->vehicle_width / 2 + 0.4;
  double safe_dis_diff = 0.1;
  if (common_handler_->isParalleSlot()) {
    safe_dis_to_obs = CarParams::GetInstance()->vehicle_width / 2 + 0.3;
  }
  union_slot.ego_side_obstacel_distance_trigger_.changeThresholds(
      safe_dis_to_obs - safe_dis_diff, safe_dis_to_obs);
  return !union_slot.ego_side_obstacel_distance_trigger_.evaluate(
      F_2_obs_to_ego_y_dis_);
}

bool OtherFeatureHandler::isParallelSlotSizeReasonable() {
  // std::cout << " F_3_parallel_slot_width_: " << F_3_parallel_slot_width_
  //           << " F_4_parallel_slot_depth_: " << F_4_parallel_slot_depth_
  //           << " F_5_parallel_slot_bottom_is_wall_: " <<
  //           F_5_parallel_slot_bottom_is_wall_ << std::endl;
  if (!common_handler_->isParalleSlot()) {
    return true;
  }
  if (F_3_parallel_slot_width_ < common_handler_->getParams().min_width_parallel) {
    return false;
  }
  if (F_5_parallel_slot_bottom_is_wall_) {
    if (F_4_parallel_slot_depth_ < common_handler_->getParams().min_length_parallel) {
      return false;
    }
  } else {
    if (F_4_parallel_slot_depth_ < common_handler_->getParams().min_length_parallel_step_bottom) {
      return false;
    }
  }
  return true;
}
}
