#pragma once

#include "planner/message_type.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/openspace_optimizer.h"
#include "planner/motion_planner/parking_motion_planner.h"
// #include "pnc/define/geometry.h"
#include "common/utils/trajectory_point_utils.h"
#include <exception>
#include <future>
#include <mutex>
#include <thread>

namespace msquare {
namespace parking {

class OpenspaceMotionPlanner : public MotionPlanner {
public:
  struct Config {
    const HybridAstarConfig *hybrid_astar_config_;
    const TrajectoryOptimizerConfig *trajectory_optimizer_config_;
    const CarParams *car_params_;
    const StrategyParams *strategy_params_;
  };
  OpenspaceMotionPlanner(const std::shared_ptr<WorldModel> &world_model);
  ~OpenspaceMotionPlanner();

  bool init_problem();
  bool calculate();

  std::vector<TrajectoryPoint> get_result() { return traj_points_; };

private:
  bool init_config();

  std::future<std::vector<TrajectoryPoint>> optimizer_future_;

  std::string OPENSPACE_CONFIG_FILE;
  std::string log_path;

private:
  Config cfg_;
  OpenspaceDeciderOutput input_;
  std::vector<TrajectoryPoint> traj_points_;
};

void dump_to_file(const std::string file_prefix,
                  const OpenspaceDeciderOutput &context);

// for teb smooth
bool smooth_teb_eigen(
    std::vector<TrajectoryPoint> &path,
    const std::vector<planning_math::LineSegment2d> &obs_lines);
bool smooth_teb_g2o(std::vector<TrajectoryPoint> &path,
                    const std::vector<planning_math::LineSegment2d> &obs_lines);

} // namespace parking
} // namespace msquare
