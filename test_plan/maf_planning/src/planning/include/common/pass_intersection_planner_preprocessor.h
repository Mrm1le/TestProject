#pragma once

#include "common/trajectory/trajectory_stitcher.h"
#include "common/utils/gauss_quad_constants.h"
#include "linear_interpolation_utility.hpp"
#include "planner/message_type.h"
#include "planner/motion_planner/planner_preprocessor.h"
#include "planner/tasks/task.h"
#include "pnc/define/pass_intersection_input_interface.hpp"
// #include "common/math/vec2d.h"

namespace msquare {

class PassIntersectionPlannerPreprocessor {
public:
  virtual ~PassIntersectionPlannerPreprocessor() = default;
  virtual void init(const std::shared_ptr<WorldModel> world_model) {
    world_model_ = world_model;
  }

  void process();

private:
  void generate_output();
  void set_init_params();
  void set_ego_pose();
  void set_reflinepoints();
  void set_target_pts();
  void plot_processor_logs();
  void check_intersection_condition();
  void update_muti_historical_info();
  void save_hist_refline();
  bool calc_lane_perception_obstacle();
  double distance(Pose2D pose1, Pose2D pose2);

  std::shared_ptr<WorldModel> world_model_;
};

bool divide_lane_to_linesegment(const maf_perception_interface::Lane &lane,
                                Transform &car2enu, double angle_accuracy,
                                double max_segment_length,
                                std::vector<std::vector<Traj_Point>> &lanes_,
                                std::vector<Traj_Point> &cur_lane_left_,
                                std::vector<Traj_Point> &cur_lane_right_);

bool divide_road_edge_to_linesegment(
    const maf_perception_interface::RoadEdge &road_edge, Transform &car2enu,
    double angle_accuracy, double max_segment_length,
    std::vector<std::vector<Traj_Point>> &road_edges_);

} // namespace msquare
