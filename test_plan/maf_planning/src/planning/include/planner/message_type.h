#ifndef MESSAGE_TYPE_H_
#define MESSAGE_TYPE_H_

#include "common/config/speed_optimizer_config.h"
#include "common/config_context.h"
#include "common/utils/cartesian_coordinate_system.h"
#include "common/utils/frenet_coordinate_system.h"
#include "common/utils/pose2d_utils.hpp"
#include "mpc_define.h"
#include "nlohmann/json.hpp"
#include "planner/planning_config.h"
#include "pnc.h"
#include "trajectory_point.h"
#include <string>

namespace msquare {

constexpr int kN = MPC_N;

constexpr double kMathEpsilon = 1e-10;

enum class MotionPlannerStatus : int {
  NORMAL = 0,
  UPDATE_FAIL = -1,
  LANE_CHANGE_SUCCESS = 1,
  LANE_CHANGE_FAIL = 2
};

enum class DriveState : int32_t {
  STOP = 0,
  START = 1,
  DRIVE = 2,
};

enum class GearState : unsigned {
  PARK = 0,
  REVERSE = 1,
  NEUTRAL = 2,
  DRIVE = 3,
  LOW = 4,
  NONE = 5
};

enum class PlanningType : int {
  LANE_KEEP = 0,
  LANE_CHANGE_LEFT = 1,
  LANE_CHANGE_RIGHT = 2,
  LANE_BORROW_LEFT = 3,
  LANE_BORROW_RIGHT = 4,
  LANE_DEPARTURE = 5,
};

enum class RoadType : int {
  UNKNOWN = 0,
  GO_STRAIGHT = 1,
  TURN_RIGHT = 2,
  TURN_LEFT = 4,
  U_TURN_LEFT = 8,
  U_TURN_RIGHT = 16,
};

enum class LaneAttribute : int {
  STRAIGHT = 1,
  TURN_RIGHT = 2,
  TURN_LEFT = 4,
  SINGLE = 8,
  DOUBLE = 16,
  MIX = 32,
  LONG = 64,
  SHORT = 128,
  ULTRAWIDE = 256,
  INTERSECTION = 512,
  VIA = 1024,
  VIRTUAL_BOX = 2048,
  UNKNOWN = 4096
};

enum class GoalType : int { NORMAL = 0, APA = 1, GATE = 2 };

struct SLPoint {
  SLPoint(double S, double L) : s(S), l(L) {}
  SLPoint() : s(0.0), l(0.0) {}
  double s;
  double l;
};

struct FrenetFramePoint {
  double s;
  double l;
  double dl;
  double ddl;
};

struct SpeedPoint {
  double s;
  double t;
  // speed (m/s)
  double v;
  // acceleration (m/s^2)
  double a;
  // jerk (m/s^3)
  double da;
};

struct ObstacleType {
  int id;
  double vel;
  double r_frenet;
  double s_frenet;
  double s_max;
  double s_min;
  double r_abs_min;
  double yaw_relative_frenet;
  string intention;
  bool isFreeMoveTraj;
  bool isLowPriority;
  double probability;
  bool isAtTargetLane;
  double cutin_score;
  double vel_rotation;
  double v_lat;
  double v_lon;
};

enum MSDObjectType {
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

struct Path {
  string name;
  PathPoint path_point;
};

enum class DrivingMode {
  AUTO = 0,
  MANUAL,
};

struct VehicleState {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double timestamp{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  double heading{0.0};
  double kappa{0.0};
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  double linear_acceleration{0.0};
  DrivingMode driving_mode{DrivingMode::AUTO};
  int gear_position{0};
  double steering_percentage{0.0};
  bool throttle_override{false};
};

struct PathTimeObstacle {
  int obstacle_id;
  Point2D bottom_left;
  Point2D upper_left;
  Point2D upper_right;
  Point2D bottom_right;
  double time_lower;
  double time_upper;
  double path_lower;
  double path_upper;
  double width_;  // width of obstacle vehicle
  double length_; // length of obstacle vehicle
};

struct PerceptionObstacle {
  int obstacle_id;
  double x;
  double y;
  double z;
  double theta;
  double v;
  double a;
  double length;
  double width;
  double height;
};

// struct PiecewiseJerkPoint{
//   double s;
//   double l;
//   double dl;
//   double ddl;
// };

// struct SLBoundingBox{
//   double s_corners[4];
//   double l_corners[4];
//   double s_center;
//   double l_center;
//   double yaw_rel;
//   char* type;
//   int id;
//   char* sp_direction;
// };
struct AnchorPoint {
  PathPoint path_point;
  double lateral_bound = 0.0;
  double longitudinal_bound = 0.0;
  // enforce smoother to strictly follow this reference point
  bool enforced = false;
};

struct ReflineInfo {
  std::vector<double> curvature;
  std::vector<double> current_lane_width;
  std::vector<double> left_lane_width;
  std::vector<double> right_lane_width;
  std::vector<double> right_road_border_distance;
  std::vector<double> left_road_border_distance;
  std::vector<double> theta_ref;
  std::vector<double> lat_offset;
  std::vector<bool> is_intersection; // parasoft-suppress AUTOSAR-A18_1_2-a
                                     // "modules's interface"
  std::vector<int> turn_type;
  std::vector<double> via_weight_base;
};

// struct RefLineInfoIndex{
//   std::vector<double> s_lane_start;
//   std::vector<int> lane_start_index;
//   std::vector<int> id_lane;
//   std::vector<int> count_lane_points;
//   std::vector<double> length_lane;
//   std::vector<int> lane_attribute;
//   std::vector<double> avg_curvature;
//   std::vector<bool> new_info;
//   std::vector<double> via_weight_base;

//   void clear(){
//     s_lane_start.clear();
//     id_lane.clear();
//     count_lane_points.clear();
//     length_lane.clear();
//     lane_attribute.clear();
//     lane_start_index.clear();
//     avg_curvature.clear();
//     new_info.clear();
//     via_weight_base.clear();
//   }
//   void resize(size_t count){
//     s_lane_start.resize(count);
//     id_lane.resize(count);
//     count_lane_points.resize(count);
//     length_lane.resize(count);
//     lane_attribute.resize(count);
//     lane_start_index.resize(count);
//     avg_curvature.resize(count);
//     new_info.resize(count);
//     via_weight_base.resize(count);
//   }
//  };

// struct RefLineInfoFrenet{
//   std::vector<std::string> track_id;
//   std::vector<double> s;
//   std::vector<double> l;
//   std::vector<double> x;
//   std::vector<double> y;
//   std::vector<double> theta;
//   std::vector<double> left_road_border_distance;
//   std::vector<double> right_road_border_distance;
//   std::vector<double> left_lane_border_distance;
//   std::vector<double> right_lane_border_distance;
//   std::vector<double> left_obstacle_distance;
//   std::vector<double> right_obstacle_distance;
//   std::vector<double> lane_width;
//   std::vector<double> curvature;
//   std::vector<double> curvature_theta;
//   std::vector<bool>   intersection;
//   std::vector<bool>   carriage_way;
//   std::vector<bool>   via_point;
//   std::vector<bool>   virtual_box;

//   void clear(){
//     track_id.clear();
//     s.clear();
//     l.clear();
//     x.clear();
//     y.clear();
//     left_road_border_distance.clear();
//     right_road_border_distance.clear();
//     left_lane_border_distance.clear();
//     right_lane_border_distance.clear();
//     left_obstacle_distance.clear();
//     right_obstacle_distance.clear();
//     lane_width.clear();
//     curvature.clear();
//     intersection.clear();
//     carriage_way.clear();
//     via_point.clear();
//     virtual_box.clear();
//   }
// };

// struct PiecewiseJerkTrajectory{
//   double s[30];
//   double l[30];
//   double dl[30];
//   double ddl[30];
//   bool success;
// };

// typedef PiecewiseJerkTrajectory *PiecewiseJerkTtraj;
enum class ObjectType {
  NOT_KNOW = 0,
  PEDESTRIAN = 1,
  OFO = 2,
  COUPE = 3,
  TRANSPORT_TRUNK = 4,
  BUS = 5,
  ENGINEER_TRUCK = 6,
  TRICYCLE = 7,
  CONE_BUCKET = 8,
  STOP_LINE = 9,
  GATE = 10,
  FREESPACE = 11,
  STEP = 12,
  WALL = 13
};

enum class IntentionObsType : int {
  NONE = 0,
  APPOARCHING_GATE = 1,
  APA = 2,
  APOA = 3,
  PULLOVER = 4,
  HIGHSPEEDDECEL = 5,
};

struct NonBlockingObstacle {
  ObjectType type =
      ObjectType::NOT_KNOW; // type of the obstacle -- car, human or ofo
  int obstacle_id;
  bool static_;
  bool reverse_;
  bool real_nbo_ = true;
  // r-t graph: assume that the lateral motion of the obstacle is linear
  std::vector<std::pair<Point2D, double>> boundary; // container sorted by time
  Point2D time_range;                               // x:start time y:end time
  // Point2D s_range;
  double s_min;
  double s_max;
  double r_min;
  Point2D start_rt;
  Point2D start_st;
  Point2D end_rt;
  Point2D end_st;
  std::vector<std::tuple<double, double, double>> obs_sv_; // s_min, s_max, vel

// get r constraint according time & s,return ROAD_WIDTH / 2 when there is no
// constraint
#define ROAD_WIDTH 3.4
  double get_r_constraint(double t, double s,
                          const double &time_resolution = 0.1) const {
    if (t < time_range.x || t > time_range.y) {
      return r_min > 0 ? ROAD_WIDTH / 2.0 : -ROAD_WIDTH / 2.0;
    }
    int index0 = (int)((t - time_range.x) / time_resolution);
    int index1 = std::min((int)boundary.size() - 1, index0 + 1);
    // consider ego vehicle length
    if (!(s + 2.0 * ConfigurationContext::Instance()
                          ->get_vehicle_param()
                          .length <
              std::min(boundary[index0].first.x, boundary[index1].first.x) ||
          s - ConfigurationContext::Instance()->get_vehicle_param().length >
              std::max(boundary[index0].first.y, boundary[index1].first.y))) {
      // interp r constraint
      return (boundary[index0].second + boundary[index1].second) / 2.0;
    }
    return r_min > 0 ? ROAD_WIDTH / 2.0 : -ROAD_WIDTH / 2.0;
  }

  std::pair<double, double> get_sr(const double &t,
                                   const double &time_resolution) {
    // time resolution = 0.2s!! in lane keeping
    std::pair<double, double> sr = {-10.0, 10.0};
    if (t < time_range.x || t > time_range.y) {
      return sr;
    }
    int index = (int)((t - time_range.x) / time_resolution);
    index = std::min((int)boundary.size() - 1, index);
    sr.first = (boundary.at(index).first.x + boundary.at(index).first.y) / 2.0;
    sr.second = boundary.at(index).second;
    return sr;
  }
};

struct InSightStaticObstacle {
  ObjectType type =
      ObjectType::NOT_KNOW; // type of the obstacle -- car, human or ofo
  int obstacle_id;
  // std::vector<std::pair<double, double> > obs_sv_;
  double current_s;
  double current_l;
  int counts = 0;
  bool detected = false;
  bool isTransverse = false;
};

struct InSightDynamicObstacle {
  ObjectType type =
      ObjectType::NOT_KNOW; // type of the obstacle -- car, human or ofo
  int obstacle_id;
  // std::vector<std::pair<double, double> > obs_sv_;
  double current_s;
  double current_l;
  // const double default_width = 1.5;
  // double obs_width;
  double ds;
  double dr;
  int counts = 0;
  bool detected = false;
};

struct SLBoundary {
  double start_s{std::numeric_limits<double>::max()};
  double end_s{std::numeric_limits<double>::lowest()};
  double start_l{std::numeric_limits<double>::max()};
  double end_l{std::numeric_limits<double>::lowest()};

  double s_start_point_s, s_start_point_l;
  double s_end_point_s, s_end_point_l;
  double l_start_point_s, l_start_point_l;
  double l_end_point_s, l_end_point_l;
  std::vector<Point3D> corners;
  SLBoundary(){};
  SLBoundary(double _start_s, double _end_s, double _start_l, double _end_l) {
    start_s = _start_s;
    end_s = _end_s;
    start_l = _start_l;
    end_l = _end_l;
  };
};

struct PathTimePoint {
  int obstacle_id;
  double s;
  double t;
};

struct SamplePoint {
  PathTimePoint path_time_point;
  double ref_v;
};

struct StopPoint {
  double s;
  enum Type { HARD, SOFT };
  Type type = HARD;
};

struct PlanningTarget {
  bool has_stop_point = false;
  StopPoint stop_point;
  double cruise_speed;
};

// const double GRAVITY = 9.81;
// static double getSafeDistance(double v_ego, double v_l, double tr_input) {
//  // return std::max(0.0, (v_ego - v_l)*tr_input);
//  // return std::max(0.0, v_l*tr_input);
//  const double v_low = 8.0 / 3.6;
//  const double v_up = 15.0 / 3.6;
//  const double lon_distance_buffer = 6.5;
//  vector<double> vhicle_ego = {v_low, v_up};
//  vector<double> vhicle_ego_trans = {0.0, v_up};
//
//  if (v_ego < v_up) {
//    if (v_ego < v_low) {
//      v_ego = 0.0;
//    } else {
//      v_ego = interp(v_ego, vhicle_ego, vhicle_ego_trans);
//    }
//  }
//  return std::max((v_ego * tr_input - (v_l - v_ego) * tr_input +
//                   v_ego * v_ego / (2 * GRAVITY) - v_l * v_l / (2 * GRAVITY)),
//                  -FLAGS_default_dis_leader + lon_distance_buffer);
//}

struct LcaStatus {
  bool lca_left_activated = false;
  bool lca_right_activated = false;
};

inline MSDObjectType from_msd_fusion_type(
    const maf_perception_interface::ObjectTypeInfo &type_info) {
  using namespace maf_perception_interface;
  MSDObjectType dst = MSD_OBJECT_TYPE_UNKNOWN;
  for (auto &property : type_info.type_properties) {
    switch (property.value) {
    case ObjectTypePropertyEnum::PROPERTY_OBJECT_TYPE_ENUM: {
      switch (type_info.type.value) {
      case ObjectTypeEnum::OBJECT_TYPE_GENERAL_OBJECT: {
        dst = MSD_OBJECT_TYPE_UNKNOWN;
      } break;
      case ObjectTypeEnum::OBJECT_TYPE_VEHICLE: {
        dst = MSD_OBJECT_TYPE_COUPE;
      } break;
      case ObjectTypeEnum::OBJECT_TYPE_VRU: {
        dst = MSD_OBJECT_TYPE_OFO;
      } break;
      case ObjectTypeEnum::OBJECT_TYPE_TRAFFIC_BARRIER: {
        dst = MSD_OBJECT_TYPE_CONE_BUCKET;
      } break;
      default:
        break;
      }
    } break;
    case ObjectTypePropertyEnum::PROPERTY_GENERAL_OBJECT_TYPE_ENUM: {
      switch (type_info.general_object_type.value) {
      case GeneralObjectTypeEnum::GENERAL_OBJECT_TYPE_POINT:
        // add radar only for highway
        dst = MSD_OBJECT_TYPE_RADAR_ONLY;
        break;
      default:
        dst = MSD_OBJECT_TYPE_UNKNOWN;
      }
      return dst;
    } break;
    case ObjectTypePropertyEnum::PROPERTY_VEHICLE_TYPE_ENUM: {
      switch (type_info.vehicle_type.value) {
      case VehicleTypeEnum::VEHICLE_TYPE_UNKNOWN: {
        dst = MSD_OBJECT_TYPE_COUPE;
      } break;
      case VehicleTypeEnum::VEHICLE_TYPE_CAR: {
        dst = MSD_OBJECT_TYPE_COUPE;
      } break;
      case VehicleTypeEnum::VEHICLE_TYPE_TRUCK: {
        dst = MSD_OBJECT_TYPE_TRANSPORT_TRUCK;
      } break;
      case VehicleTypeEnum::VEHICLE_TYPE_BUS: {
        dst = MSD_OBJECT_TYPE_BUS;
      } break;
      default:
        break;
      }
      break;
    }
    case ObjectTypePropertyEnum::PROPERTY_VEHICLE_SUB_TYPE_ENUM: {
      switch (type_info.vehicle_sub_type.value) {
      case VehicleSubTypeEnum::VEHICLE_SUB_TYPE_UNKNOWN_VEHICLE: {
        if (dst == MSD_OBJECT_TYPE_UNKNOWN) {
          dst = MSD_OBJECT_TYPE_COUPE;
        }
        return dst;
      } break;
      case VehicleSubTypeEnum::VEHICLE_SUB_TYPE_LAMP_UNKNOWN_VEHICLE: {
        if (dst == MSD_OBJECT_TYPE_UNKNOWN) {
          dst = MSD_OBJECT_TYPE_COUPE;
        }
        return dst;
      } break;
      case VehicleSubTypeEnum::VEHICLE_SUB_TYPE_UNKNOWN_CAR: {
        if (dst == MSD_OBJECT_TYPE_UNKNOWN) {
          dst = MSD_OBJECT_TYPE_COUPE;
        }
        return dst;
      } break;
      case VehicleSubTypeEnum::VEHICLE_SUB_TYPE_UNKNOWN_TRUCK: {
        if (dst == MSD_OBJECT_TYPE_UNKNOWN) {
          dst = MSD_OBJECT_TYPE_TRANSPORT_TRUCK;
        }
        return dst;
      } break;
      case VehicleSubTypeEnum::VEHICLE_SUB_TYPE_DANGER_TRANSPORT_TRUCK: {
        dst = MSD_OBJECT_TYPE_TRANSPORT_TRUCK;
        return dst;
      } break;
      case VehicleSubTypeEnum::VEHICLE_SUB_TYPE_ENGINEERING_TRUCK: {
        dst = MSD_OBJECT_TYPE_ENGINEERING_TRUCK;
        return dst;
      } break;
      default:
        break;
      }
    } break;
    case ObjectTypePropertyEnum::PROPERTY_VRU_TYPE_ENUM: {
      switch (type_info.vru_type.value) {
      case VRUTypeEnum::VRU_TYPE_UNKNOWN: {
        dst = MSD_OBJECT_TYPE_OFO;
      } break;
      case VRUTypeEnum::VRU_TYPE_PEDESTRIAN: {
        dst = MSD_OBJECT_TYPE_PEDESTRIAN;
        return dst;
      } break;
      case VRUTypeEnum::VRU_TYPE_BICYCLIST: {
        dst = MSD_OBJECT_TYPE_OFO;
      } break;
      default:
        break;
      }
    } break;
    case ObjectTypePropertyEnum::PROPERTY_VRU_SUB_TYPE_ENUM: {
      switch (type_info.vru_sub_type.value) {
      case VRUSubTypeEnum::VRU_SUB_TYPE_UNKNOWN_PEDESTRIAN: {
        dst = MSD_OBJECT_TYPE_PEDESTRIAN;
        return dst;
      } break;
      case VRUSubTypeEnum::VRU_SUB_TYPE_STAND_PEDESTRIAN: {
        dst = MSD_OBJECT_TYPE_PEDESTRIAN;
        return dst;
      } break;
      case VRUSubTypeEnum::VRU_SUB_TYPE_SQUATTER_PEDESTRIAN: {
        dst = MSD_OBJECT_TYPE_PEDESTRIAN;
        return dst;
      } break;
      case VRUSubTypeEnum::VRU_SUB_TYPE_LYING_PEDESTRIAN: {
        dst = MSD_OBJECT_TYPE_PEDESTRIAN;
        return dst;
      } break;
      default:
        dst = MSD_OBJECT_TYPE_OFO;
        return dst;
        break;
      }
    } break;
    case ObjectTypePropertyEnum::PROPERTY_TRAFFIC_BARRIER_TYPE_ENUM: {
      switch (type_info.traffic_barrier_type.value) {
      case TrafficBarrierTypeEnum::TRAFFIC_BARRIER_TYPE_UNKNOWN: {
        dst = MSD_OBJECT_TYPE_CONE_BUCKET;
        return dst;
      } break;
      case TrafficBarrierTypeEnum::TRAFFIC_BARRIER_TYPE_CONE: {
        dst = MSD_OBJECT_TYPE_CONE_BUCKET;
        return dst;
      } break;
      case TrafficBarrierTypeEnum::TRAFFIC_BARRIER_TYPE_WARNING_TRIANGLE: {
        dst = MSD_OBJECT_TYPE_CONE_BUCKET;
        return dst;
      } break;
      case TrafficBarrierTypeEnum::TRAFFIC_BARRIER_TYPE_WATER_BARRIER: {
        dst = MSD_OBJECT_TYPE_CONE_BUCKET;
        return dst;
      } break;
      default:
        break;
      }
    } break;
    default:
      break;
    }
  }
  return dst;
}
} // namespace msquare

#endif
