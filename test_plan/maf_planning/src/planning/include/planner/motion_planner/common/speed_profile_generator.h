#ifndef DECISION_PLANNING_OPTIMIZER_SPEED_PROFILE_GENERAOR_H_
#define DECISION_PLANNING_OPTIMIZER_SPEED_PROFILE_GENERAOR_H_

#include "common/path/discretized_path.h"
#include "common/speed/speed_data.h"
#include "planner/motion_planner/optimizers/optimal_speed_planner.h"

#include <utility>
#include <vector>

namespace msquare {

class SpeedProfileGenerator {
public:
  SpeedProfileGenerator() = delete;

  static SpeedData GenerateFallbackSpeed(double stop_distance, double init_v,
                                         const double init_a);

  static void FillEnoughSpeedPoints(SpeedData *const speed_data);

  static SpeedData GenerateFixedDistanceCreepProfile(double distance,
                                                     double max_speed);

  static SpeedData GenerateConstAccSpeed(double max_speed, double init_v,
                                         double acc, double t_limit);

  static SpeedData GenerateStopProfile(double init_speed, double init_acc,
                                       double acc);
};

} // namespace msquare

#endif // DECISION_PLANNING_OPTIMIZER_SPEED_PROFILE_GENERAOR_H_
