#ifndef MODULES_PLANNING_COMMON_TRAJECTORY_STITCHER_H_
#define MODULES_PLANNING_COMMON_TRAJECTORY_STITCHER_H_

#include <vector>

#include "common/math/linear_interpolation.h"
#include "common/trajectory/discretized_trajectory.h"
#include "planner/message_type.h"

namespace msquare {

class TrajectoryStitcher {
public:
  TrajectoryStitcher() = delete;

  static void
  TransformLastPublishedTrajectory(const double x_diff, const double y_diff,
                                   const double theta_diff,
                                   DiscretizedTrajectory *prev_trajectory);

  static DiscretizedTrajectory
  ConstructTrajFromPlanningResult(const PlanningResult &planning_result);

  static std::vector<TrajectoryPoint> ComputeStitchingTrajectory(
      const VehicleState &vehicle_state, const double current_timestamp,
      const double planning_cycle_time, const size_t preserved_points_num,
      const bool replan_by_offset, PlanningResult *prev_planning_result,
      std::string *replan_reason);

  static std::vector<TrajectoryPoint>
  ComputeReinitStitchingTrajectory(const double planning_cycle_time,
                                   const VehicleState &vehicle_state);

  static PublishedTrajectory
  TransformToPublishedTraj(std::vector<TrajectoryPoint> trajectory);

private:
  static std::pair<double, double>
  ComputePositionProjection(const double x, const double y,
                            const TrajectoryPoint &matched_trajectory_point);

  static TrajectoryPoint
  ComputeTrajectoryPointFromVehicleState(const VehicleState &vehicle_state);
};

} // namespace msquare

#endif /* MODULES_PLANNING_COMMON_TRAJECTORY_STITCHER_H_ */
