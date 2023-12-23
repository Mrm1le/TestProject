#pragma once

// #include "planner/behavior_planner/deciders/openspace_decider_interface.h"
#include "common/math/math_utils.h"
#include "common/parking_world_model.h"
#include "planner/message_type.h"

namespace msquare {
namespace parking {

struct GridMapBoxLocal {
  double s_up = 0.0;
  double s_down = 0.0;
  double s_left = 0.0;
  double s_right = 0.0;

  void extend(const planning_math::Vec2d &pp) {
    s_up = std::max(pp.y(), s_up);
    s_down = std::min(pp.y(), s_down);
    s_left = std::min(pp.x(), s_left);
    s_right = std::max(pp.x(), s_right);
  };
};

bool get_gridmap_boundary_normal(const std::shared_ptr<WorldModel> world_model,
                                 const double &default_swell_distance,
                                 const double &default_step_size,
                                 const TrajectoryPoint &target_state,
                                 const TrajectoryPoint &ego_state,
                                 planning_math::Box2d &grid_map_bounding_box,
                                 GridMapBoxLocal &box_local,
                                 Pose2D &tmp_location_zero);

bool extend_boundary_with_refline(const std::shared_ptr<WorldModel> world_model,
                                  const double &default_swell_distance,
                                  const double &default_step_size,
                                  const TrajectoryPoint &target_state,
                                  const TrajectoryPoint &ego_state,
                                  planning_math::AABox2d &box_local,
                                  const Pose2D &tmp_location_zero,
                                  double start_s = 0);

void get_refline_info(const std::vector<double> s_consider,
                      std::vector<double> &left_road_border_distance,
                      std::vector<double> &right_road_border_distance,
                      std::vector<double> left_road_border_distance_vector,
                      std::vector<double> right_road_border_distance_vector,
                      std::vector<double> s_vector);
} // namespace parking
} // namespace msquare
