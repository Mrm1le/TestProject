#pragma once

#include "common/ego_model_manager.h"
#include "common/parking_obstacle_manager.h"
#include "common/parking_slot_interface.h"
#include "common/parking_world_model.h"

#include "common/grid_map/grid_map.h"

namespace msquare {
namespace parking {

class RemainDistDecider {
public:
  struct OutHoleParam {
    // parameter
    const int kGettingOutHoldCountThreshold = 30; // 1.5s
    const double kSafeLateralThreshold = 0.05;
    const double kSafeLateralDeltaDistThreshold = 1e-2;
    const double kCheckingRisingPathLength = 0.3;
    const double kSlowEgoVel = 0.3;
    const double kSafeRemainDist = 0.99; // set 0.99 just for easy debug

    bool prev_is_planner_update_plan_path = false;
    int getting_out_count = 0;
  };

public:
  RemainDistDecider(const std::shared_ptr<WorldModel> &world_model);
  ~RemainDistDecider() = default;

  bool MakeDecision(const EgoModelManager &ego_model, const bool is_moved,
                    FreespacePoint *const ptr_lead_point,
                    std::vector<std::vector<std::pair<double, double>>>
                        *const ptr_vec_debug_sl_points,
                    double *const ptr_side_safe_threshold,
                    OutHoleParam *const ptr_out_hole_param);

private:
  bool constructGridMap(const EgoModelManager &ego_model,
                        const double pilliar_extra);

  bool
  getSlBoundary(const Pose2DTrajectory &path, const bool is_reverse,
                const bool is_checking_plan_path,
                const double side_safe_threshold,
                std::vector<std::pair<double, double>> *const ptr_sl_points,
                std::pair<double, double> *const ptr_danger_sl,
                std::string *const ptr_debug_string);

  void getExtraPillarObstacle(
      const double pilliar_extra, const vector<Obstacle> &vec_pillar_origin,
      std::vector<planning_math::Vec2d> *const ptr_extra_pillar_pts);

  bool GetOutHole(const bool is_planner_update_plan_path,
                  const std::vector<std::vector<std::pair<double, double>>>
                      &vec_debug_sl_points,
                  const double side_safe_threshold,
                  FreespacePoint *const ptr_lead_point,
                  string *const ptr_debug_str);

private:
  std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<grid::ObstacleGrid> obs_grid_;
  std::shared_ptr<grid::MultiCircleFootprintModel> mc_footprint_model_;
  // int32_t current_lane_id_ = -1000;

  OutHoleParam *out_hole_param_;
};

} // namespace parking
} // namespace msquare
