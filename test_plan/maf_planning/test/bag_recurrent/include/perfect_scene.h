#ifndef PERFECT_SCENE
#define PERFECT_SCENE

#include "util.h"
#include "parking_scenario/parking_scenario.h"

namespace perfect_scene{

enum ObstacleType{
    CAR = 0,
    WALL
};

enum ParkingSlotType {
  PARALLEL = 0,
  VERTICAL,
  OBLIQUE,
  PARKOUT_PARALLEL,
  PARKOUT_VERTICAL,
  PARKOUT_OBLIQUE,
  UNKNOWN
};

enum ParkingSlotSide {
    SIDE_LEFT = 0,
    SIDE_RIGHT,
    SIDE_UNKNOWN
};

struct LabelPoint {
  double x;
  double y;
  LabelPoint() : x(0.0), y(0.0){}
  LabelPoint(double _x, double _y) : x(_x), y(_y){}
};

struct ObstacleLabelData{
    const ObstacleType obstacle_type;
    const std::vector<LabelPoint> label_points;
    ObstacleLabelData(ObstacleType _obstacle_type, std::vector<LabelPoint> _label_points)
    :obstacle_type(_obstacle_type), label_points(_label_points){}


    std::vector<msquare::planning_math::Vec2d> getOdoPoints() const;
    std::vector<msquare::planning_math::LineSegment2d> getOdoLines() const;
};

std::string perfectScenePlan(
    const parking_scenario::Point2d& init_pose, 
    const parking_scenario::Point2d& target_pose, 
    const std::vector<ObstacleLabelData>& label_datas,
    ParkingSlotType slot_type, 
    ParkingSlotSide slot_side = SIDE_UNKNOWN,
    const std::vector<parking_scenario::Point2d>& slot_corners = {},
    int init_speed = 0);

std::string
perfectScenePlanSimple(bool reverse, bool center_at_init, bool car_as_points,
                       const parking_scenario::Point2d &init_pose,
                       const parking_scenario::Point2d &target_pose,
                       const std::vector<ObstacleLabelData> &label_datas,
                       int init_speed = 0);

parking_scenario::Point2d convertLidarPose2EgoPose(
    const std::vector<double>& lidar_pose, 
    const std::vector<float>& vr, 
    const std::vector<float>& vt);

ParkingSlotType getSlotType(const parking_scenario::Point2d& init_pose, 
    const parking_scenario::Point2d& target_pose);
ParkingSlotType
getSlotTypeFromSlotCorners(const std::vector<parking_scenario::Point2d> &slot_corners);
ParkingSlotSide addVerticalTlines(msquare::parking::OpenspaceDeciderOutput &odo,
    ParkingSlotSide slot_side = SIDE_UNKNOWN);
ParkingSlotSide addParallelTlines(msquare::parking::OpenspaceDeciderOutput &odo,
    ParkingSlotSide slot_side = SIDE_UNKNOWN);
ParkingSlotSide
addObliqueTlines(const std::vector<parking_scenario::Point2d> &slot_corners,
                 msquare::parking::OpenspaceDeciderOutput &odo,
                 ParkingSlotSide slot_side = SIDE_UNKNOWN);

void addMapBoundary(msquare::parking::OpenspaceDeciderOutput &odo);
void addMapBoundaryWithSlotCorners(
    const std::vector<parking_scenario::Point2d> &slot_corners,
    msquare::parking::OpenspaceDeciderOutput &odo);

}   // end namespace perfect_scene

#endif