#ifndef CP_INTERFACE_PLANNER_COMMON_H
#define CP_INTERFACE_PLANNER_COMMON_H

#include <stdint.h>
#include <string>
#include <vector>

#define COMMON_QUADRATURE_ORDER 5
#define COMMON_NUM_PATH_CONTROL_POINTS 6
#define COMMON_NUM_PATH_SEGMENTS 5
#define COMMON_NUM_SPEED_CONTROL_POINTS 7
#define COMMON_NUM_SPEED_SEGMENTS 6
#define COMMON_TOTAL_NUM_PARAMS 6

namespace cp_path_planner {
  
struct Point2d {
  double x = 0;
  double y = 0;
};


struct Pose2d{
  double x;
  double y;
  double theta;
};

struct EgoState{
  double ego_vel;
  double ego_steer_angle;
  double ego_v_cruise;
  double ego_acc;
  Pose2d ego_pose;
};

struct ObjState {
  cp_path_planner::Point2d pos;
  double accel = -200.0;
  double vel = -200.0;  // linear speed
  double heading = -200.0;
  double s = -200.0;
  double v_frenet = -200.0;
  double rel_s = -200.0;
};

struct SafetyMargin{
  double longitu = 0.0;
  double lateral = 0.0;
};

struct ObjPredSlice{
  SafetyMargin safety_margin;
  ObjState obj_state_local;
  ObjState obj_state_env;
  int32_t lane_assignment = 0;
};

struct VectorDouble {
  std::vector<double> vec;
  bool __convert_to_list___ = true;
};

struct VectorVectorDouble {
  std::vector<cp_path_planner::VectorDouble> vec_vec;
  bool __convert_to_list___ = true;
};

struct PairDoubleDouble {
  double first;
  double second;
  bool __convert_to_list___ = true;
};

struct PairIntDouble {
  int32_t first;
  double second;
  bool __convert_to_list___ = true;
};

struct PairIntBool {
  int32_t first;
  bool second;
  bool __convert_to_list___ = true;
};

}  // namespace

#endif