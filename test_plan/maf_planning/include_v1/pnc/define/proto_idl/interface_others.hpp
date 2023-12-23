#ifndef CP_INTERFACE_OTHERS_H
#define CP_INTERFACE_OTHERS_H

#include <stdint.h>
#include <limits>
#include <string>
#include <vector>

#include "interface_planner_common.hpp"

namespace cp {

// struct VX_VY{
//   std::array<double, 200> vx;
//   std::array<double, 200> vy;
//   std::array<double, 200> delta_s;
// };

// struct RefLine_XY{
//   std::array<double, 200> x;
//   std::array<double, 200> y;
//   std::array<double, 200> z;
// }

// struct DebugJson{
//   VX_VY vx_xy;
//   RefLine_XY refline_xy;
// };

struct TrajectoryPointDDP {
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

struct DdpTrajectory {
  std::vector<TrajectoryPointDDP> trajectory;
  uint8_t type = 0;
  double logit = 0;
  std::vector<std::int64_t> track_ids;
};

}  // namespace cp

#endif