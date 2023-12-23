#pragma once

#include "planner/message_type.h"
#include "planner/motion_planner/openspace_motion_planner/path_sampler_statemachine.h"
#include "planner/motion_planner/openspace_motion_planner/zigzag_pathsampler.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/openspace_optimizer.h"
#include "planner/motion_planner/parking_motion_planner.h"
#include "pnc/define/geometry.h"
#include <exception>
#include <future>
#include <mutex>
#include <thread>

namespace msquare {
namespace parking {

class PathSampler : public MotionPlanner {
public:
  PathSampler(const std::shared_ptr<WorldModel> &world_model);
  ~PathSampler();

  bool calculate();
  bool calculate_sop();

  void set_path(const std::vector<TrajectoryPoint> &points);

  bool is_at_last_segment() const;

  std::string get_segment_str() const;

  bool is_arrived() const;
  bool is_arrived(const Pose2D &ego_pose, const double ending_distance) const;

  path_sampler_statemachine::SamplerFsmStateEnum getCurrentState();

private:
  std::unique_ptr<path_sampler_statemachine::MPeerRoot> fsm_;
  path_sampler_statemachine::Context fsm_ctx_;
  ZigzagPathSampler zigzag_pathsampler_;
  int last_segment_ = -1;
  bool smart_change_direction_flag = false;
  bool switch_traj_flag = false;
  bool mock_is_static_flag_ = false;
  bool not_use_reached_info = false;
};

} // namespace parking
} // namespace msquare
