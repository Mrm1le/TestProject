#pragma once

#include "planner/message_type.h"
#include <string>
#include <tuple>
#include <vector>

namespace msquare {
namespace ddp {

using ObjectTrajectoryPoint = msquare::TrajectoryPoint;

typedef enum { NO_CHANGE, LEFT_CHANGE, RIGHT_CHANGE } RequestType;

typedef enum { NONE_SIDE, LEFT_SIDE, RIGHT_SIDE, BOTH_SIDE } LooseBoundType;

typedef enum {
  NO_REQUEST,
  INT_REQUEST,
  MAP_REQUEST,
  MODEL_REQUEST,
  ROUTE_REQUEST
} RequestSource;

enum class ObjectType {
  NOT_KNOW = 0,
  PEDESTRIAN = 1,
  OFO = 2,
  COUPE = 3,
  TRANSPORT_TRUCK = 4,
  BUS = 5,
  ENGINEER_TRUCK = 6,
  TRICYCLE = 7,
  CONE_BUCKET = 8,
  STOP_LINE = 9,
  VIRTUAL_SOILD = 10,
  VIRTUAL_BUS = 11,
};

// map info struct ------------------------------

template <typename T> struct Point3X {
  Point3X() {}
  Point3X(T x, T y) : x(x), y(y), z(0.0) {}
  Point3X(T x, T y, T z) : x(x), y(y), z(z) {}

  T x;
  T y;
  T z;
};

typedef Point3X<double> Point3d;
typedef Point3X<double> Point;
typedef Point3X<double> Vector3d;

template <typename T> struct Shape3X {
  Shape3X() {}
  Shape3X(T length, T width) : length(length), width(width), height(0.0) {}
  Shape3X(T length, T width, T height)
      : length(length), width(width), height(height) {}

  T length;
  T width;
  T height;
};
typedef Shape3X<double> Shape3d;
typedef Shape3X<double> Shape;

struct EgoPose {
  double timestamp;
  Point3d position;
  Vector3d velocity;
  double v;
  double a;
  double heading_angle;
};

enum class LineType : int {
  ROUTE_ON_ROUTE = 5,
  ROUTE_OFF_ROUTE = 6,
  ROUTE_BUS = 7,
  LANE_PROPOSAL_ON_ROUTE = 8,
  LANE_PROPOSAL_OFF_ROUTE = 9,
  LANE_PROPOSAL_BUS = 10,
  SOLID_LANE_BOUNDARY = 11,
  DASHED_LANE_BOUNDARY = 12,
  SOLID_ROAD_BOUNDARY = 13,
  DASHED_ROAD_BOUNDARY = 14,
  STATIC_OBSTACLE = 15,
  JUNCTION_LINE = 16,
  OTHER_LINE = 17,
};

struct Node {
  Node() {}
  Node(const Point &start_point, const Point &end_point, LineType type)
      : start_point(start_point), end_point(end_point), type(type) {}
  Node(const Point &start_point, const Point &end_point, LineType type, int id,
       bool b_in_intersection)
      : start_point(start_point), end_point(end_point), type(type), id(id),
        b_in_intersection(b_in_intersection) {}
  Point start_point;
  Point end_point;
  LineType type;
  int id = -100;
  bool b_in_intersection;
};

struct NodeLane {
  NodeLane() {}
  NodeLane(const std::vector<Node> &nodes, const std::string &id, LineType type,
           int color, float min_distance = 0.f, float on_route_dis = 0.f)
      : nodes(nodes), id(id), type(type), color(color),
        min_distance_to_ego(min_distance), on_route_dis(on_route_dis) {}

  std::vector<Node> nodes;
  std::string id;
  LineType type;
  int color;
  float min_distance_to_ego;
  float on_route_dis;
};

// -----------------------------------------

struct VirtualPoint2D {
  // TODO (@Haowen) for temporary usage
  double x = 0.0;
  double y = 0.0;

  VirtualPoint2D() = default;
  VirtualPoint2D(double xx, double yy) : x(xx), y(yy) {}
};

struct VirtualPoint3D {
  // TODO (@Haowen) for temporary usage
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  VirtualPoint3D() = default;
  VirtualPoint3D(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}
};

struct TrajectoryPoint {
  // enu
  double x = 0;
  double y = 0;
  double heading_angle = 0;
  double curvature = 0;
  double t = 0;
  double v = 0;
  double a = 0;

  // frenet
  double s = 0;
  double l = 0;
  bool frenet_valid = false;
};
using TrajectoryPoints = std::vector<TrajectoryPoint>;

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TrajectoryPoint, x, y, v, s)

typedef enum {
  CRUISE_KEEP = 0,
  CRUISE_CHANGE,
  CRUISE_WAIT,
  CRUISE_BACK,
  CRUISE_UTURN
} ScenarioStateEnum;

typedef enum { SCENARIO_CRUISE = 0, SCENARIO_LOW_SPEED } ScenarioEnum;

typedef enum { DDP_RAW = 0, DDP_LANE_CHANGE, DDP_UNKNOWN } TrajectoryTypeEnum;

typedef enum {
  OFF_ROUTE = 0,
  OFF_MAP,
  LATERAL_DIFF,
  LINE_PRESSING_DRIVING,
  BORDER_PRESSING_DRIVING
} FaultDiagnosisType;

struct DdpTrajectory {
  TrajectoryPoints trajectory;
  TrajectoryTypeEnum type = DDP_RAW;
  double logit = 0;
  std::vector<std::int64_t> track_ids;
};
using DdpTrajectorys = std::vector<DdpTrajectory>;

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DdpTrajectory, trajectory, type, logit,
                                   track_ids)

struct DdpObject {
  double timestamp_us = 0;
  double timestamp = 0;
  bool ddp_valid = false;
  std::vector<DdpTrajectory> trajectory_array;
};

enum class ResultTrajectoryType { REFERENCE_PATH, RAW_TRAJ, REFINED_TRAJ };

struct AccSafetyInfo {
  bool need_takeover = false;
};
struct ObstacleInformation {
  int obstacle_id{-1};
  int obstacle_type{-1};
  double obstacle_s{-1.0};
  double obstacle_v{-1.0};
  double duration{-1.0};
};

struct LeadoneInfo {
  bool has_leadone{false};
  ObstacleInformation leadone_information;
};

struct CutinInfo {
  bool has_cutin{false};
  std::vector<ObstacleInformation> cutin_information;
};

struct CIPVInfo {
  bool has_CIPV{false};
  std::vector<ObstacleInformation> CIPV_information;
};

struct LonDecisionInfo {
  LeadoneInfo leadone_info;
  CutinInfo cutin_info;
  CIPVInfo CIPV_info;
  AccSafetyInfo acc_safety_info;
  double map_velocity_limit{0.0};
};

struct LatDecisionInfo {
  int direction = 0;
  int id = 0;
  int type = 0;
};
using LatDecisionInfos = std::unordered_map<int, LatDecisionInfo>;

struct FaultDiagnosisInfo {
  bool able_to_auto{true};
  bool pre_able_to_auto{false};
  int fault_type = -1;
};

struct PlanningResult {
  int target_lane_id;
  ScenarioStateEnum target_scenario_state = CRUISE_KEEP;
  TrajectoryPoints raw_traj_points;
  TrajectoryPoints traj_points;
  RequestType turn_signal = NO_CHANGE;
  int use_backup_cnt = 0;
  double timestamp = 0.0;
  std::string extra_json;
  mjson::Json extra_json_raw;
};

struct PlanningInitPoint {
  double x;
  double y;
  double heading_angle;
  double curvature;
  double v;
  double a;
  double relative_time;
  FrenetState frenet_state;
};

class ReferencePath;
struct CoarsePlanningInfo {
  ScenarioStateEnum source_state;
  ScenarioStateEnum target_state;
  RequestSource lane_change_request_source;
  int source_lane_id;
  int target_lane_id;
  std::shared_ptr<ReferencePath> reference_path;
  TrajectoryPoints trajectory_points;
  // overtake_obstacles and yield_obstacles are used only under wait state
  std::vector<int> overtake_obstacles;
  std::vector<int> yield_obstacles;
};

struct FrenetBoundary {
  double s_start;
  double s_end;
  double l_start;
  double l_end;
};

struct FrenetObstacleBoundary {
  double s_start{std::numeric_limits<double>::max()};
  double s_end{std::numeric_limits<double>::lowest()};
  double l_start{std::numeric_limits<double>::max()};
  double l_end{std::numeric_limits<double>::lowest()};
};

enum CurrentState {
  INIT = 0,
  LANE_KEEPING,
  APPROACH_STOPLINE_SLOW,
  PASS_INTERSECTION,
  RED_LIGHT_STOP,
  COVER_LIGHT,
  INTO_WAIT_ZONE
};

} // namespace ddp
} // namespace msquare
