#pragma once

#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/parking_lot.h"
#include "planner/message_type.h"

namespace msquare {

namespace parking {

class ParallelParkingSlot : public BaseParkingSlot {
public:
  ParallelParkingSlot(const std::string cfg_file,
                      std::vector<Point3D> lot_corners,
                      std::vector<planning_math::Vec2d> physical_corners,
                      bool is_relative_left = false);
  Pose2D getParkingInPose(double front_edge_to_rear, double back_edge_to_rear,
                          double brake_buffer, const Pose2D &init_pose,
                          double wheel_stop_depth = 0);
  virtual planning_math::LineSegment2d genBottomWall();
  virtual std::vector<planning_math::LineSegment2d> genFrontWings(double span);
  std::vector<TrajectoryPoint> getCenterLine(double front_edge_to_rear,
                                             double back_edge_to_rear,
                                             double brake_buffer = 0,
                                             double wheel_stop_depth = 0);
  virtual bool isLateralInside(const Pose2D &pose,
                               const FootprintModelPtr footprint_model);
  virtual bool isInside(const Pose2D &pose,
                        const FootprintModelPtr footprint_model);
  virtual bool isAlongsideLot(const Pose2D &pose, double thres);
  virtual bool isVehicleReached(const VehicleParam *vehicle_param,
                                const Pose2D &pose,
                                const FootprintModelPtr footprint_model,
                                double wheel_stop_depth = 0);
  virtual void customizePlanningParams(CarParams *car_params,
                                       HybridAstarConfig *hybridastar_config,
                                       StrategyParams *strategy_params);
  virtual std::vector<planning_math::LineSegment2d>
  getTshapedAreaLines(const TrajectoryPoint &ego_state,
                      const planning_math::Box2d &map_boundary,
                      std::vector<ObstacleBox> nearby_boxes,
                      const std::vector<planning_math::Vec2d> &points);
  virtual std::vector<planning_math::LineSegment2d>
  genLotWalls(double vehicle_width);

private:
  std::vector<planning_math::Vec2d> physical_corners_;
  bool isInsidePhysicalSlot(const Pose2D &pose,
                            const FootprintModelPtr footprint_model);
};

} // namespace parking

} // namespace msquare
