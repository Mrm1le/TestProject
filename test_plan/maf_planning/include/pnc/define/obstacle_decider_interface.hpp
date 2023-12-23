#pragma once
#include "nlohmann/json.hpp"
#include "planner_constants.hpp"
#include "path_planner_interface.hpp"
// #include "general_motion_planner_interface.hpp"
using json = nlohmann::json;

namespace odc_interface{

struct Point2d {
  double x;
  double y;
};

struct Pose2d{
  double x;
  double y;
  double theta;
  double s = 0.0;
  double kappa_ = 0.0;
};

struct EgoState{
  double ego_vel;
  double ego_steer_angle;
  double ego_v_cruise;
  double ego_acc;
  odc_interface::Pose2d ego_pose;
};

struct ObjState{
  Point2d pos;
  double accel{-200.0};
  double vel{-200.0};  // linear speed
  double heading{-200.0};
  double s{-200.0};
  double v_frenet{-200.0};
  double rel_s{-200.0};
};

struct SafetyMargin{
  double longitu;
  double lateral;
};


struct ObjPredSlice{
  SafetyMargin safety_margin;
  ObjState obj_state_local;
  ObjState obj_state_env;
  int lane_assignment;
};

struct ObsInfo {
  // enum Type { PEDESTRIAN = 0, OFO = 1, COUPE = 2, TRANSPORT_TRUNK = 4 };
  enum LonDecision {
    FOLLOW = 0,
    OVERTAKE = 1,
    LON_IGNORE = 2,
  };

  enum LatDecision {
    NUDGE = 0,
    LAT_IGNORE = 1,
  };

  enum NudgeType { LEFT_NUDGE = 0, RIGHT_NUDGE = 1 };

  int id;
  // Type type;
  LonDecision lon_decision;
  LatDecision lat_decision;
  NudgeType nudge_side;

  ObjPredSlice obj_state_init;

};

struct ObstacleDeciderInput{
  EgoState ego_state{};
  EgoState ego_state_cart{};
  
};

struct TrajectoryPoint{
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
  // local
  double x_local = 0.0;
  double y_local = 0.0;
};

struct DdpTrajectory {
  std::vector<odc_interface::TrajectoryPoint> trajectory;
  double logit;
};

struct ObstacleDeciderOutput{
  bool odc_decision_valid{false};
  std::vector<odc_interface::ObsInfo> obs_infos{};
  std::unordered_map<int,odc_interface::ObsInfo> obs_infos_map{};
  std::vector<odc_interface::TrajectoryPoint> current_lane_ddp_trajectory;
  std::vector<DdpTrajectory> ddp_multipath;
  
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point2d, x,
                                    y)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObjState, pos, accel,
                                    vel, heading, s, 
                                    v_frenet, rel_s)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SafetyMargin, longitu,
                                    lateral)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObjPredSlice, safety_margin,
                                    obj_state_local,obj_state_env,lane_assignment)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObsInfo, lon_decision,lat_decision,
                                    nudge_side, 
                                    obj_state_init)
                              
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TrajectoryPoint, x, y, heading_angle, curvature, t,
                                    v, a, s, l, frenet_valid,x_local,y_local)                              

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DdpTrajectory, trajectory,
                                    logit)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObstacleDeciderOutput,odc_decision_valid,
                                    obs_infos,obs_infos_map,current_lane_ddp_trajectory,ddp_multipath)                                    

}// namespace