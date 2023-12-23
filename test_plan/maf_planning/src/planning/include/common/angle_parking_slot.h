#pragma once
#include "common/math/math_utils.h"
#include "common/parking_lot.h"

namespace msquare {
namespace parking {

class AngleParkingSlot : public BaseParkingSlot {
private:
  std::vector<planning_math::Vec2d> original_corners_;

public:
  AngleParkingSlot(const std::string cfg_file, std::vector<Point3D> lot_corners,
                   bool is_relative_left,
                   std::vector<planning_math::Vec2d> original_corners);
  ~AngleParkingSlot();

  virtual std::vector<planning_math::LineSegment2d>
  genFrontWings(double span) override;
  Pose2D getParkingInPose(double front_edge_to_rear, double back_edge_to_rear,
                          double brake_buffer, const Pose2D &init_pose,
                          double wheel_stop_depth = 0) override;
  virtual std::vector<planning_math::LineSegment2d>
  getTshapedAreaLines(const TrajectoryPoint &ego_state,
                      const planning_math::Box2d &map_boundary,
                      std::vector<ObstacleBox> nearby_boxes,
                      const std::vector<planning_math::Vec2d> &points) override;
  virtual double getOpeningHeading() const override;
};

} // namespace parking

} // namespace msquare
