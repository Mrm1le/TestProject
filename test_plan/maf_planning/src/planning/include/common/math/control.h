#pragma once

#include "common/config/vehicle_param.h"
#include "planner/message_type.h"
#include "pnc/define/geometry.h"
#include <limits>
#include <string>
#include <vector>

namespace msquare {
namespace parking {

double estimate_delta_f(const std::vector<TrajectoryPoint> &traj,
                        const Pose2D &ego_pose,
                        const double lookahead_dist = 2.0, const double k = 0.1,
                        const double Kp = 1.0);

} // namespace parking
} // namespace msquare
