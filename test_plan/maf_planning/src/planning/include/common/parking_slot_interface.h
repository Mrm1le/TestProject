#pragma once

#include "common/config/vehicle_param.h"
#include "common/footprint_model.h"
#include "common/math/math_utils.h"
#include "planner/message_type.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "pnc/define/geometry.h"

namespace msquare {
namespace parking {

class ParkingSlotInterface {
public:
  ParkingSlotInterface();
  virtual ~ParkingSlotInterface();

  virtual void updateCorners(const std::vector<Point3D> lot_corners) = 0;
  virtual Pose2D getParkingInPose(double front_edge_to_rear,
                                  double back_edge_to_rear, double brake_buffer,
                                  const Pose2D &init_pose,
                                  double wheel_stop_depth = 0) = 0;
  virtual void
  processLinesAroundLot(std::vector<planning_math::LineSegment2d> &lines) = 0;
  /**
   * @brief generate lot walls based on the lot corners from map,
   * and adjust the side walls according to the size of car to avoid hard
   * constraint.
   *
   * @return [left_side_wall, bottom_wall, right_side_wlal]
   */
  virtual std::vector<planning_math::LineSegment2d>
  genLotWalls(double vehicle_width) = 0;

  virtual planning_math::Box2d getBox() const = 0;
  virtual planning_math::Box2d getOriginBox() const = 0;
  virtual bool isOnLeft() const = 0;
  virtual std::vector<planning_math::LineSegment2d>
  genFrontWings(double span) = 0;
  virtual planning_math::LineSegment2d genBottomWall() = 0;
  virtual void setSafeMode(bool left_safe, bool right_safe) = 0;

  /**
   * @brief change strategy of genLotWalls, etc.
   * @param
   *  width: width of the Road in front of lot.
   */
  virtual void matchRoadWidth(double width) = 0;
  virtual std::vector<TrajectoryPoint>
  getCenterLine(double front_edge_to_rear, double back_edge_to_rear,
                double brake_buffer = 0, double wheel_stop_depth = 0) = 0;
  virtual bool isLateralInside(const Pose2D &pose,
                               const FootprintModelPtr footprint_model) = 0;
  virtual bool isInside(const Pose2D &pose,
                        const FootprintModelPtr footprint_model) = 0;
  virtual bool isAlongsideLot(const Pose2D &pose, double thres) = 0;
  virtual bool isVehicleReached(const VehicleParam *vehicle_param,
                                const Pose2D &pose,
                                const FootprintModelPtr footprint_model,
                                double wheel_stop_depth = 0) = 0;
  virtual void customizePlanningParams(CarParams *car_params,
                                       HybridAstarConfig *hybridastar_config,
                                       StrategyParams *strategy_params) = 0;
  virtual std::vector<planning_math::LineSegment2d>
  getTshapedAreaLines(const TrajectoryPoint &ego_state,
                      const planning_math::Box2d &map_boundary,
                      std::vector<ObstacleBox> nearby_boxes,
                      const std::vector<planning_math::Vec2d> &points) = 0;
  virtual double getOpeningHeading() const = 0;
};

inline ParkingSlotInterface::ParkingSlotInterface() {}

inline ParkingSlotInterface::~ParkingSlotInterface() {}

} // namespace parking
} // namespace msquare
