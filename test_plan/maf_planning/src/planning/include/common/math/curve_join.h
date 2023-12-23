#ifndef MODULES_PLANNING_CURVE_JOIN_H_
#define MODULES_PLANNING_CURVE_JOIN_H_

#include "common/math/math_utils.h"
#include <vector>

namespace msquare {

bool CurveJoin(const Pose2D &start, const Pose2D &target, const double r_min,
               const double step_size, std::vector<Pose2D> &path);

void SmoothCurve(const std::vector<Pose2D> &key_points, const double step_size,
                 std::vector<Pose2D> &path);
} // namespace msquare

#endif /* MODULES_PLANNING_CURVE_JOIN_H_ */
