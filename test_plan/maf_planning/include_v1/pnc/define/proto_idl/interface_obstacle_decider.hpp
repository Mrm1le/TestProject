#ifndef CP_INTERFACE_OBSTACLE_DECIDER_H
#define CP_INTERFACE_OBSTACLE_DECIDER_H

#include <stdint.h>
#include <limits>
#include <string>
#include <vector>

#include "interface_planner_common.hpp"

namespace cp_odc_interface {

struct ObsInfoOBD {
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

  int32_t id;
  // Type type;
  uint8_t lon_decision;
  uint8_t lat_decision;
  uint8_t nudge_side;

  cp_path_planner::ObjPredSlice obj_state_init;
};

struct ObstacleDeciderInput {
  cp_path_planner::EgoState ego_state{};
  cp_path_planner::EgoState ego_state_cart{};
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
  // local
  double x_local = 0.0;
  double y_local = 0.0;
};

struct DdpTrajectoryODC {
  std::vector<cp_odc_interface::TrajectoryPoint> trajectory;
  double logit = 0.0;
};

struct PairIntObsInfo {
  int32_t first;
  cp_odc_interface::ObsInfoOBD second;
};

struct ObstacleDeciderOutput {
  bool odc_decision_valid = false;
  std::vector<cp_odc_interface::ObsInfoOBD> obs_infos;
  std::vector<PairIntObsInfo> obs_infos_map;
  std::vector<cp_odc_interface::TrajectoryPoint> current_lane_ddp_trajectory;
  std::vector<DdpTrajectoryODC> ddp_multipath;
};

} // namespace cp_odc_interface

#endif