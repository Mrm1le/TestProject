#pragma once

#include "common/world_model.h"
#include "data_driven_planner/common/basic_types.h"
#include "data_driven_planner/models/ddp_model.h"
#include <vector>

namespace msquare {
namespace ddp {

class ModelResultManager {
public:
  ModelResultManager(const std::shared_ptr<WorldModel> &world_model,
                     const std::shared_ptr<DdpModel> &ddp_model,
                     bool use_ddp_model_in_planning);
  ~ModelResultManager() = default;

  void update();

  // get
  const std::vector<DdpTrajectory> &get_ddp_trajectory_info() const {
    return ddp_trajectory_info_;
  }
  std::vector<DdpTrajectory> &get_mutable_ddp_trajectory_info() {
    return ddp_trajectory_info_;
  }
  const DdpTrajectory &get_current_lane_ddp_trajectory() const {
    return current_lane_ddp_trajectory_;
  }
  DdpTrajectory &get_mutable_current_lane_ddp_trajectory() {
    return current_lane_ddp_trajectory_;
  }

  bool get_ddp_valid() const { return ddp_valid_; }
  double get_timestamp() const { return timestamp_; }

  // utils
  static bool interpolate(const TrajectoryPoints traj_points,
                          const double start_time, const double delta_time,
                          const int num_point,
                          TrajectoryPoints &res_traj_points);

private:
  void update_with_ddp_model_in_planning();
  // void update_with_ddp_model_in_prediction();

private:
  // model
  std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<DdpModel> ddp_model_;
  bool use_ddp_model_in_planning_;

  // result
  std::vector<DdpTrajectory> ddp_trajectory_info_;
  DdpTrajectory current_lane_ddp_trajectory_;
  double timestamp_ = 0;
  bool ddp_valid_ = false;
};

} // namespace ddp
} // namespace msquare
