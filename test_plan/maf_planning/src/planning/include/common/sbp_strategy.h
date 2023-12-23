#pragma once

#include "common/parking_planner_types.h" // OpenspaceDeciderOutput && TrajectoryPoint
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/search_based_planner.h"

namespace msquare {
namespace parking {

bool GetTemporalProfile(SbpResult *result);
bool TrajectoryPartition(const SbpResult &result,
                         std::vector<SbpResult> *partitioned_result,
                         double init_v = 0, double init_a = 0);

bool GenerateSpeedAcceleration(SbpResult *result);

std::vector<TrajectoryPoint>
planWithStrategy(const SearchBasedPlannerPtr sbp, const StrategyParams *cfg,
                 const OpenspaceDeciderOutput &input_);

/**
 * @brief disassemble problem into a search problem
 */
OpenspaceDeciderOutput
extractSearchProblem(const StrategyParams *cfg,
                     const OpenspaceDeciderOutput &input_);

OpenspaceDeciderOutput genPartialProblem(const StrategyParams *cfg,
                                         const OpenspaceDeciderOutput &input_);

OpenspaceDeciderOutput genReverseProblem(const OpenspaceDeciderOutput &input_);

/**
 * @brief assemble problem based on search result
 */
bool assembleSearchProblem(const StrategyParams *cfg, SbpResult &sbp_result);

void regulateVelocity(const StrategyParams *cfg,
                      std::vector<TrajectoryPoint> &traj);

bool genOpenspacePath(const SearchBasedPlannerPtr sbp,
                      const OpenspaceDeciderOutput &input_,
                      SbpResult &sbp_result,
                      SearchProcessDebug *sp_debug = nullptr);

// std::vector<TrajectoryPoint> getStraightTraj(const Pose2D &ego_pose,
//                                              const Pose2D &target_pose,
//                                              SbpResult &sbp_result);

void extendSbpResult(SbpResult &sbp_result, double s_total, double ds);

inline std::vector<TrajectoryPoint>
convertSbpResult2Traj(const SbpResult &sbp_result) {
  std::vector<TrajectoryPoint> traj;
  traj.resize(sbp_result.x.size());
  for (size_t i = 0; i < sbp_result.x.size(); ++i) {
    traj.at(i).path_point.x = sbp_result.x.at(i);
    traj.at(i).path_point.y = sbp_result.y.at(i);
    traj.at(i).path_point.theta = sbp_result.phi.at(i);
    traj.at(i).v = sbp_result.v.at(i);
    traj.at(i).a = sbp_result.a.at(i);
    traj.at(i).steer = sbp_result.steer.at(i);
    traj.at(i).path_point.s = sbp_result.accumulated_s.at(i);
    if (std::abs(sbp_result.steer.at(i)) > 1e-6) {
      traj.at(i).path_point.rho = CarParams::GetInstance()->wheel_base /
                                  std::tan(sbp_result.steer.at(i));
    } else {
      traj.at(i).path_point.rho = std::numeric_limits<double>::infinity();
    }
    traj.at(i).path_point.kappa = 1.0 / traj.at(i).path_point.rho;
  }
  return traj;
}

void getObsTangentLine(planning_math::LineSegment2d line,
                       planning_math::Box2d parking_space_box,
                       const OpenspaceDeciderOutput &osd);

} // namespace parking
} // namespace msquare
