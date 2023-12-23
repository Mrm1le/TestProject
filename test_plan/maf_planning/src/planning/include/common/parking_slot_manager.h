#ifndef MSQUARE_DECISION_PLANNING_PLANNER_PARKING_SLOT_MANAGER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_PARKING_SLOT_MANAGER_H_

#include <cstdint>

#include "common/config/vehicle_param.h"
#include "common/math/box2d.h"
#include "common/math/line2d.h"
#include "common/math/vec2d.h"
#include "common/parking_lot.h"
#include "common/parking_obstacle_manager.h"
#include "common/parking_world_model.h"
#include "common/planning_context.h"
#include "planner/behavior_planner/deciders/apf_decider.h"

namespace msquare {

namespace parking {

namespace {
enum class SlotClosestSideType {
  NONE_OBSTACLE = 0,
  SIDE_SLOT = 1,
  SIDE_HIGH_OBSTACLE = 2,
  SIDE_CAR_OBSTACLE = 3,
};
}

class ParkingSlotManager {
  /**
   * @class ParkingSlotManager
   * @brief To pre-process parking slot info.
   */

public:
  ParkingSlotManager();
  ParkingSlotManager(const std::shared_ptr<WorldModel> &world_model);

  bool UpdateTargetParkingSlotInfo();
  bool UpdateEgoParkingSlotInfo();
  bool UpdateParkingSlotInfo(const int id);
  bool UpdateParkingSlotInfo(const ParkingLotDetectionInfo &parking_lot_info);
  bool UpdateWheelStopPoint(const bool collision);
  bool ClearParkingSlotInfo();
  bool ClearParkingSlotKeyPoint();

  bool OptimizeParkingSlotCorners(ParkingSlotInfo &parking_slot_info);

  bool GetParkInPoint();
  bool UpdateParkingSlotDistance();

  bool refactorCenterLines(planning_math::LineSegment2d& center_line, ParkingSlotInfo &parking_slot_info,std::vector<double> &slot_dis);

private:
  std::uint8_t GetParkingSlotType(int slot_type);
  void updateSlotByVirtualCorner(ParkingSlotInfo &parking_slot_info);
  void checkTwoSidesVehicle(ParkingSlotInfo &parking_slot_info);
  void checkPerpendicularBottomWall(ParkingSlotInfo &parking_slot_info);
  bool updateSlotBySingleSideObstacle(
      const planning_math::Line2d &bisector,
      const std::pair<SlotClosestSideType, SlotClosestSideType>
          &slot_two_sides_type,
      const double left_obstacle_to_origin_slot_center,
      const double right_obstacle_to_origin_slot_center,
      ParkingSlotInfo *const ptr_parking_slot_info);


  std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<ApfDecider> apf_decider_;

  double perpendicular_parking_slot_width_ = 2.7;
  double parallel_parking_slot_width_ = 6.5;
  double parking_slot_inflation_ratio_ = 1.0;
  double parallel_parking_slot_offset_ = 0.20; // according to i-Vista
  double default_distance_apa_ = 7.0;
  double default_distance_apoa_ = 8.0;
  double max_end_pose_curv_ = 0.05;
  double s_step_ = 1.0;
  int max_num_ = 2;
  double park_out_max_dis_ = 7.0;
  double body_tire_offset_ = 0.03;
  double body_tire_offset_space_slot_ = 0.03;
};
} // namespace parking

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_PLANNER_PARKING_LOT_SELECTOR_H_
