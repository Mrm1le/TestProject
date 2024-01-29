#ifndef SV_SPEED_GENERATOR_H
#define SV_SPEED_GENERATOR_H

#include "common/config/vehicle_param.h"
#include "common/math/polygon2d.h"
#include "common/utils/geometry.h"
namespace msquare {
namespace parking {

struct SvSpeedParam {
  bool has_ever_been_inside_slot;
  bool is_reverse;
  bool force_stop = false;
  bool no_slot = false;
  double out_slot_coeff = 1.0;
  double large_curv_coeff = 0.625;
  double in_slot_coeff = 0.5;
  double max_v = 0.8;
  double width_wo_rearview_mirror;
  double length;
  double curv_limit;
};

class SvSpeedGenerator {
public:
  static double getSpeed(const planning_math::Box2d &slot_box,
                         const Pose2D &ego_pose,
                         const std::vector<float> &traj_curvature,
                         const SvSpeedParam &sv_speed_param, bool &is_in_slot);
  static double regulateSpeed(const planning_math::Box2d &slot_box,
                              const Pose2D &ego_pose,
                              const std::vector<float> &traj_curvature,
                              const SvSpeedParam &sv_p, bool &is_in_slot);
};

} // namespace parking
} // namespace msquare
#endif // SV_SPEED_GENERATOR_H