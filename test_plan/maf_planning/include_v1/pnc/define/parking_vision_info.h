#ifndef CP_MSQUARE_DECISION_PLANNING_COMMON_PARKING_VISION_INFO_H
#define CP_MSQUARE_DECISION_PLANNING_COMMON_PARKING_VISION_INFO_H

#include "pnc/define/geometry.h"
#include <vector>

namespace cp {

struct VisionHuman {
  int32_t track_id;
  float d3_x;
  float d3_yl;
  float d3_yr;
  float d3_vx;
  float d3_vy;
};

struct VisionCar {
  int32_t track_id;
  float d3_x;
  float d3_y;
  float theta;
  float d3_vx;
  float d3_vy;
  std::vector<Point3D> corners;
  int32_t closest_corner_idx;
  bool is_confident;
};

struct VisionFreeSpace {
  std::vector<Point3D> d3_border_pts;
  std::vector<int32_t> border_type;
};

enum class FusionObjectType {
  MSD_OBJECT_TYPE_STOP_LINE = -1,
  MSD_OBJECT_TYPE_UNKNOWN = 0,
  MSD_OBJECT_TYPE_COUPE = 1,
  MSD_OBJECT_TYPE_TRANSPORT_TRUCK = 2,
  MSD_OBJECT_TYPE_BUS = 3,
  MSD_OBJECT_TYPE_ENGINEERING_TRUCK = 4,
  MSD_OBJECT_TYPE_TRICYCLE = 5,
  MSD_OBJECT_TYPE_CAR = 6,

  MSD_OBJECT_TYPE_PEDESTRIAN = 10001,
  MSD_OBJECT_TYPE_OFO = 10002,
  MSD_OBJECT_TYPE_CONE_BUCKET = 20001,
  MSD_OBJECT_TYPE_RADAR_ONLY = 30001,
};

struct Shape3D {
  float height;
  float length;
  float width;
};

struct FusionObject {
  uint64_t track_id;
  FusionObjectType type;
  bool is_static;

  Shape3D shape;
  Shape3D shape_sigma;
  Point3D position;             // position in enu
  Point3D position_sigma;       // standard deviation of position
  Point3D velocity;             // velocity in enu, m/s
  Point3D velocity_sigma;       // standard deviation of velocity
  Point3D acceleration;         // acceleration in enu, m/s^2
  Point3D acceleration_sigma;   // standard deviation of acceleration
  float heading_yaw;            // heading yaw in enu
  float heading_yaw_sigma;      // standard deviation of heading yaw
  float angular_velocity;       // angular velocity of heading yaw
  float angular_velocity_sigma; // standard deviation of angular velocity of
                                // heading yaw

  Point2D relative_position; // relative position in car coordinating system.
  Point2D relative_position_sigma; // standard derivation of relative position.
  Point2D
      relative_velocity; // relative velocity in car coordinating system. m/s
  Point2D relative_velocity_sigma; // standard derivation of relative velocity.
  Point2D relative_acceleration;   // relative accleration in car coordinating
                                   // system. m/s^2
  Point2D relative_acceleration_sigma; // standard derivation of relative
                                       // acceleration.
  float relative_heading_yaw; // rad, relative heading yaw of object in car
                              // coordinating system.
  float relative_heading_yaw_sigma; // standard derivation of relative heading
                                    // yaw.
  float relative_angular_velocity;  // relative angular velocity
  float relative_angular_velocity_sigma; // standard deviation of relative
                                         // angular velocity

  Point2D velocity_relative_to_ground;       // velocity relative to ground. m/s
  Point2D velocity_relative_to_ground_sigma; // standard deviation of velocity
                                             // relative to ground. m/s
  Point2D
      acceleration_relative_to_ground; // accleration relative to ground. m/s^2
  Point2D acceleration_relative_to_ground_sigma; // accleration relative to
                                                 // ground. m/s^2

  float score;

  // TODO: add the rest of object attributes
  // Polygon3f polygon_bottom
  // Polygon3f polygon_top

  // string reserved_info
};

enum class GroundLineType {
  GROUND_LINE_TYPE_UNKNOWN = 0,
  GROUND_LINE_TYPE_WALL = 1,
  GROUND_LINE_TYPE_PILLAR = 2,
  GROUND_LINE_TYPE_FENCE = 3,
  GROUND_LINE_TYPE_STEP = 4,
  GROUND_LINE_TYPE_SPECIAL = 5,
  // line segment, for parking slot bottom line
  GROUND_LINE_USS_TYPE_UNKNOWN = 100,
  GROUND_LINE_USS_TYPE_WALL = 101,
  GROUND_LINE_USS_TYPE_PILLAR = 102,
  GROUND_LINE_USS_TYPE_FENCE = 103,
  GROUND_LINE_USS_TYPE_STEP = 104,
  GROUND_LINE_USS_TYPE_SPECIAL = 105,
  // point, for parking slot opposite obs
  GROUND_LINE_USS_TYPE_POINT = 200,
  // point, for parking slot side obs point
  GROUND_LINE_USS_TYPE_SIDE_POINT_VEHICLE = 236,
  GROUND_LINE_USS_TYPE_SIDE_POINT_ONLY_USS_UNKOWN = 237,
  GROUND_LINE_USS_TYPE_SIDE_POINT = 250,
  GROUND_LINE_USS_TYPE_SIDE_POINT_WALL = 251,
  GROUND_LINE_USS_TYPE_SIDE_POINT_PILLAR = 252,
  GROUND_LINE_USS_TYPE_SIDE_POINT_FENCE = 253,
  GROUND_LINE_USS_TYPE_SIDE_POINT_STEP = 254,
  GROUND_LINE_USS_TYPE_SIDE_POINT_SPECIAL = 255,
};

enum class FusionFreespacePointType {
  PFUSION_FREESPACE_UNKNOWN = 0,
  PFUSION_FREESPACE_NON_GROUNDING_POINT = 1,
  PFUSION_FREESPACE_ROAD_SURFACE_POINT = 2,
  PFUSION_FREESPACE_STATIC_BORDER = 3,
  PFUSION_FREESPACE_DYNAMIC_BORDER = 4,
  PFUSION_FREESPACE_LOWER_EDGE_LINE = 5,
};

enum class UssType {
  APA_FRONT_LEFT = 0,
  UPA_FRONT_LEFT = 1,
  UPA_FRONT_CENTER_LEFT = 2,
  UPA_FRONT_CENTER_RIGHT = 3,
  UPA_FRONT_RIGHT = 4,
  APA_FRONT_RIGHT = 5,
  APA_REAR_LEFT = 6,
  UPA_REAR_LEFT = 7,
  UPA_REAR_CENTER_LEFT = 8,
  UPA_REAR_CENTER_RIGHT = 9,
  UPA_REAR_RIGHT = 10,
  APA_REAR_RIGHT = 11,
};

struct FusionFreespacePoint {
  Point3D position;
  FusionFreespacePointType type;
  float confidence;
};

struct UssResult {
  UssType type;
  double range;
  std::vector<FusionFreespacePoint> uss_points;
  bool isAPA() const {
    return this->type == UssType::APA_FRONT_LEFT ||
           this->type == UssType::APA_FRONT_RIGHT ||
           this->type == UssType::APA_REAR_LEFT ||
           this->type == UssType::APA_REAR_RIGHT;
  }
  bool isUPA() const { return !this->isAPA(); }
  bool isFront() const { return (int)this->type <= (int)UssType::APA_FRONT_RIGHT; }
  bool isBack() const { return (int)this->type >= (int)UssType::APA_REAR_LEFT; }
  bool isCenter() const {
    return this->type == UssType::UPA_FRONT_CENTER_LEFT ||
           this->type == UssType::UPA_FRONT_CENTER_RIGHT ||
           this->type == UssType::UPA_REAR_CENTER_LEFT ||
           this->type == UssType::UPA_REAR_CENTER_RIGHT;
  }
};

struct ParkingVisionInfo {
  std::vector<VisionHuman> human_array;
  std::vector<VisionCar> car_array;
  VisionFreeSpace free_space;
};

struct GroundLine {
  int id = 0;
  std::vector<Point3D> pts;
  GroundLineType type = GroundLineType::GROUND_LINE_TYPE_UNKNOWN;
};

struct FusionInfo {
  std::vector<FusionObject> human_array;
  std::vector<FusionObject> car_array;
  std::vector<FusionObject> static_car_array;
  std::vector<FusionFreespacePoint> free_space;
  std::vector<FusionFreespacePoint> uss_fusion;
  std::vector<UssResult> uss_results;
  std::vector<GroundLine> ground_line_fusion;
};

} // namespace cp

#endif
