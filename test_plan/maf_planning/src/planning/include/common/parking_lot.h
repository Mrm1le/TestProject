#pragma once

#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "common/math/vec2d.h"
#include "common/parking_slot_interface.h"
#include "planner/message_type.h"

namespace msquare {

namespace parking {

bool isEgoLonInPlace(const Pose2D &ego_rear, const Pose2D &target,
                     const Pose2D &threshold);
bool isEgoLatInPlace(const Pose2D &ego_rear, const Pose2D &target,
                     const Pose2D &threshold);
bool isEgoHeadingInPlace(const Pose2D &ego_rear, const Pose2D &target,
                         const Pose2D &threshold);
bool isEgoInPlace(const Pose2D &ego_rear, const Pose2D &target,
                  double wheel_base, const Pose2D &threshold);
class BaseParkingSlot : public ParkingSlotInterface {
public:
  /**
   * @brief construct lot from 4 corners
   * corner should be ordered conter-clockwise
   * and corners[0] and corners[3] is the open edge
   */
  BaseParkingSlot(const std::string cfg_file, std::vector<Point3D> lot_corners,
                  bool is_relative_left = false);

  void updateCorners(const std::vector<Point3D> lot_corners);

  /**
   * @brief change strategy of genLotWalls, etc.
   * @param
   *  width: width of the Road in front of lot.
   */
  void matchRoadWidth(double width);
  Pose2D getParkingInPose(double front_edge_to_rear, double back_edge_to_rear,
                          double brake_buffer, const Pose2D &init_pose,
                          double wheel_stop_depth = 0);
  void processLinesAroundLot(std::vector<planning_math::LineSegment2d> &lines);
  /**
   * @brief generate lot walls based on the lot corners from map,
   * and adjust the side walls according to the size of car to avoid hard
   * constraint.
   *
   * @return [left_side_wall, bottom_wall, right_side_wlal]
   */
  std::vector<planning_math::LineSegment2d> genLotWalls(double vehicle_width);

  planning_math::Box2d getBox() const { return box_; }
  planning_math::Box2d getOriginBox() const;
  bool isOnLeft() const { return is_relative_left_; }
  std::vector<planning_math::LineSegment2d> genFrontWings(double span);
  planning_math::LineSegment2d genBottomWall();
  void setSafeMode(bool left_safe, bool right_safe);
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
                                       StrategyParams *strategy_params) {}
  virtual std::vector<planning_math::LineSegment2d>
  getTshapedAreaLines(const TrajectoryPoint &ego_state,
                      const planning_math::Box2d &map_boundary,
                      std::vector<ObstacleBox> nearby_boxes,
                      const std::vector<planning_math::Vec2d> &points);
  virtual double getOpeningHeading() const;

protected:
  void initConfig(const std::string cfg_file);
  void initBox();
  std::vector<Point3D> corners_;
  planning_math::Box2d box_;
  bool is_relative_left_;

  double lot_entrance_extra_gap_;
  double lot_entrance_extra_gap_left_;
  double lot_entrance_extra_gap_right_;
  double lot_side_entrance_clip_ratio_;
  double lot_side_entrance_clip_ratio_left_;
  double lot_side_entrance_clip_ratio_right_;

  double narrow_road_width_threshold_;
  double narrow_road_lot_entrance_extra_gap_;
  double narrow_road_lot_side_entrance_clip_ratio_;

  double distance_back_to_wheelstop_;
  double distance_wheel_to_wheelstop_;
  double back_distance_buffer_;

  double side_safety_lot_side_entrance_clip_ratio_;
  double side_safety_lot_entrance_extra_gap_;
  bool enable_get_heading_with_front_corners_;

  double default_depth_;
};

} // namespace parking

} // namespace msquare
