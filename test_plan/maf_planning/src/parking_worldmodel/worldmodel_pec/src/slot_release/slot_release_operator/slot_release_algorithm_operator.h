

#pragma once
#include "../parameters/slot_release_params.h"
#include "../parking_slot_element/parking_slot_element.h"
#include "../schmitt_trigger/schmittTrigger.h"
#include "slot_release_manager.h"

namespace worldmodel_pec {

class CommonHandler {
public:
  CommonHandler() = default;
  CommonHandler(
      const Eigen::Isometry3d &ego_pose,
      const maf_perception_interface::FusionGroundLineResult &object_fusion,
      const SlotReleaseParams &params, double yaw, bool is_apa_mode)
      : ego_pose_(ego_pose), object_fusion_(object_fusion), params_(params),
        yaw_(yaw), is_apa_mode_(is_apa_mode) {}
  void generateObjFusionToSlotFrame();
  void generateSideObjFusionToSlotFrame();
  void generateOppositeObjFusionToSlotFrame();
  void feedFusionSlot(const ParkingSlotElement &fusion_slot) {
    fusion_slot_ = fusion_slot;
  }
  std::vector<Eigen::Vector3d>
  generateBottomObjFusionToSlotFrame(bool *has_wall);
  std::vector<Eigen::Vector3d> generateObjFusionToEgoPoseFrame();
  void generateSlotFrame();
  std::pair<double, double> getSlotWidthAndDepth();
  bool isParalleSlot();
  bool isObliqueSlot();
  bool isLineSlot();
  bool isSpaceSlot();
  bool isDriveAlongXPositive();
  bool isApaMode() { return is_apa_mode_; }

  Eigen::Isometry3d getEgoPose() { return ego_pose_; }
  ParkingSlotElement getFusionSlot() { return fusion_slot_; }
  maf_perception_interface::FusionGroundLineResult getObjectionFusion() {
    return object_fusion_;
  }

  const Eigen::Isometry3d &getSlotFrameTF() { return slot_frame_tf_inv_; }
  Eigen::Vector3d getEgoPoseToSlotFrame();
  std::vector<Eigen::Vector3d> getAllObjFusionToSlotFrame() {
    return all_obj_fusions_to_slot_frame_;
  }
  std::vector<Eigen::Vector3d> getSideObjFusionToSlotFrame() {
    return side_obj_fusions_to_slot_frame_;
  }
  std::vector<Eigen::Vector3d> getOppositeObjFusionToSlotFrame() {
    return opposite_obj_fusions_to_slot_frame_;
  }
  std::vector<Eigen::Vector3d> getObjFusionToObliqueSlotFrame() {
    return obj_fusions_to_oblique_slot_frame_;
  }
  std::vector<Eigen::Vector3d> getObjPosVec() {
    return obj_pos_vec_;
  }
  SlotReleaseParams getParams() { return params_; }

private:
  double yaw_;
  bool is_apa_mode_;
  SlotReleaseParams params_;
  Eigen::Isometry3d ego_pose_;
  ParkingSlotElement fusion_slot_;
  maf_perception_interface::FusionGroundLineResult object_fusion_;
  std::vector<Eigen::Vector3d> all_obj_fusions_to_slot_frame_;
  std::vector<Eigen::Vector3d> side_obj_fusions_to_slot_frame_;
  std::vector<Eigen::Vector3d> opposite_obj_fusions_to_slot_frame_;
  std::vector<Eigen::Vector3d> obj_fusions_to_oblique_slot_frame_;
  std::vector<Eigen::Vector3d> obj_pos_vec_;
  Eigen::Isometry3d slot_frame_tf_inv_;
  Eigen::Isometry3d oblique_slot_frame_tf_inv_;
};

class SlotReleaseHandler {
public:
  void
  process(const maf_perception_interface::FusionGroundLineResult &fusion_slot,
          const maf_perception_interface::PerceptionFusionObjectResult &object,
          SlotReleaseManager::EgoCarInfo loc, Eigen::Isometry3d local_ego_pose,
          std::deque<Eigen::Isometry3d> history_locs, bool is_move,
          int apa_target_id, std::string apa_status_,
          int ego_parking_slot_track_id, const SlotReleaseParams &params,
          bool is_apa_mode,
          std::list<SlotReleaseManager::ParkingSlotUnion> &history_slots);

private:
  bool isParkingSafetyAreaOccupiedByObject(
      const SlotReleaseManager::ParkingSlotUnion &parking_slot_union,
      const maf_perception_interface::PerceptionFusionObjectResult &object,
      CommonHandler *common_handler);
  bool isParkingSafetyAreaOccupiedByObjectFusion(CommonHandler *common_handler);
  bool
  isPassTime(int gear,
             const SlotReleaseManager::ParkingSlotUnion &parking_slot_union);
  bool isOccupiedByVehicle(
      const SlotReleaseManager::ParkingSlotUnion &parking_slot_union,
      const maf_perception_interface::PerceptionFusionObjectResult &object);
};

class EnvFeatureHandler {

public:
  explicit EnvFeatureHandler(CommonHandler *common_handler)
      : common_handler_(common_handler) {}
  bool
  isSlotOpenAreaWidthEnough(SlotReleaseManager::ParkingSlotUnion &union_slot);
  bool
  isSlotOpenAreaLengthEnough(SlotReleaseManager::ParkingSlotUnion &union_slot);
  bool isPassageWidthEnough(SlotReleaseManager::ParkingSlotUnion &union_slot);
  bool isDeadEndRoad();

public:
  void extractFeature();
  void extractFeaturSlotLeftAndRightWidth();
  void extractFeaturSlotWidthFromSpaceSlot();
  void extractFeaturSlotOutLengthFromSpaceSlot();
  void extractOpenAreaOutLengthFromSpaceSlot();
  void extractOpenAreaOutLength();
  void extractPassageWidth();
  void extractDeadEndRoad();

private:
  double F_1_slot_left_width_ = 0.0;
  double F_2_slot_right_width_ = 0.0;
  double F_3_slot_open_area_out_length_ = 0.0;
  double F_4_slot_open_area_in_length_ = 0.0;
  double F_5_passage_extend_length_ = 0.0;
  double F_6_passage_width_ = 0.0;
  double F_7_slot_open_area_in_length_ = 0.0;
  double F_8_dead_end_road_length_ = 0.0;

private:
  CommonHandler *common_handler_ = nullptr;
};

class EgoCarFeatureHandler {
public:
  explicit EgoCarFeatureHandler(CommonHandler *common_handler,
                                std::deque<Eigen::Isometry3d> history_locs,
                                bool is_move, int apa_target_id,
                                std::string apa_status, double velocity,
                                double pitch)
      : common_handler_(common_handler), history_locs_(history_locs),
        F_5_is_move_(is_move), apa_target_parkingslot_track_id_(apa_target_id),
        F_4_apa_status_(apa_status), F_1_velocity_(velocity),
        F_2_pitch_(pitch) {}

  void extractFeature();

  // void extractVelocity();
  // void extractPitch();
  // void extractYaw();
  // void extractAPAStatus();
  void extractSlotToEgoPose();
  void extractSlotToHistoryTrajPose();

public:
  bool isvelocityReasonable();
  bool isPitchReasonable();
  bool isYawReasonable();
  bool isMove();
  bool isCarInSlot();
  bool isParkingStatus();
  bool isSlotCenterToEgoDisReasonable();
  bool isSlotFullyObservedAccordHisTraj(SlotReleaseManager::ParkingSlotUnion &union_slot);
  bool isEgoCarPassSlotDownstream();

private:
  double F_1_velocity_ = 0.0;
  double F_2_pitch_ = 0.0;
  double F_3_yaw_ = 0.0;
  // 当前APA状态
  std::string F_4_apa_status_{};
  bool F_5_is_move_ = false;
  double F_6_ego_to_slot_x_ = 0.0;
  double F_7_ego_to_slot_y_ = 0.0;
  Eigen::Vector3d F_8_ego_to_slot_theta_;
  double F_9_min_trajectory_slot_to_ego_x_ = 0.0;
  double F_10_max_trajectory_slot_to_ego_x_ = 0.0;
  double F_11_slot_to_ego_x_ = 0.0;
  CommonHandler *common_handler_ = nullptr;
  std::deque<Eigen::Isometry3d> history_locs_;
  int apa_target_parkingslot_track_id_ = 0;
};

class OtherFeatureHandler {
public:
  explicit OtherFeatureHandler(CommonHandler *common_handler)
      : common_handler_(common_handler) {}

private:
  void extractObsToEgoPoseDis();
  void extractParallelWidthAndDepth();
  void extractSlotOccupiedFromPSD();

public:
  void extractFeature();
  bool isParallelSlotSizeReasonable();
  bool isObsSafeToEgoCar(SlotReleaseManager::ParkingSlotUnion &union_slot);
  bool isSlotOccupiedFromPSD();

private:
  double F_1_obs_to_ego_x_dis_ = 0.0;
  double F_2_obs_to_ego_y_dis_ = 0.0;
  double F_3_parallel_slot_width_ = 0.0;
  double F_4_parallel_slot_depth_ = 0.0;
  bool F_5_parallel_slot_bottom_is_wall_ = false;
  bool F_6_is_slot_occupied_from_psd_ = false;
  CommonHandler *common_handler_ = nullptr;
};

} // namespace worldmodel_pec