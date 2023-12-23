#ifndef MSQUARE_DECISION_PLANNING_COMMON_REFLINE_GENERATOR_H
#define MSQUARE_DECISION_PLANNING_COMMON_REFLINE_GENERATOR_H

#include "common/config/refline_generator_config.h"
#include "common/utils/frenet_coordinate_system.h"
#include "planner/motion_planner/apf_planner.h"
#include "pnc/define/pass_intersection_input_interface.hpp"
#include <memory>
#include <string>

using namespace pass_intersection_planner;

namespace msquare {

class WorldModel;

enum ReflineGeneratorCore { NoCoreType = 0, Apf = 1, HybridAstar = 2 };
enum ApfAttractType { Unknow = 0, Traj = 1, Theta = 2 };

class ReflineGenerator {

public:
  void init(ReflineGeneratorCore core_type);
  void default_init();
  bool
  update(const PassIntersectionPlannerInput &pass_intersection_planner_input,
         PiNotebookDebug *pi_nb_debug = nullptr);
  std::vector<Pose2D> refline_result() { return refline_result_; };
  void get_default_apf_config(ApfPlannerConfigPtr planner_cfg_);
  void get_apf_config(ApfPlannerConfigPtr planner_cfg_, double shrink_ratio);
  const pass_intersection_planner::ReflineCondition &condition() {
    return condition_;
  };
  const ApfAttractType &attract_type() { return attract_type_; };
  int get_selected_hist_vehicle_id() { return selected_hist_vehicle_id_; }
  int get_inter_intersection_count() { return inter_intersection_count_; }
  bool get_apf_lat_far_flag() { return apf_lat_far_flag_; }
  std::vector<double> get_target_pt() { return target_pt_; }
  int get_target_pt_count() { return target_pt_count_; }
  int get_follow_car_id() { return follow_car_id_; }
  Pose2D get_refline_origin() { return refline_origin_; }
  double get_road_ref_theta() { return road_ref_theta_; }
  int get_quit_cp() { return quit_cp_; }
  std::string get_apf_quit_cp_reason() { return apf_quit_cp_reason_; }
  double get_lat_dis_2_in_inter() { return lat_dis_2_in_inter_; }
  double get_max_ego2refline_lat_dis() { return max_ego2refline_lat_dis_; }
  double get_lat_dis_2_refline() { return lat_dis_2_refline_; }
  double get_cur_lane_left_border() { return cur_lane_left_border_; }
  double get_cur_lane_right_border() { return cur_lane_right_border_; }

  std::vector<std::vector<std::vector<double>>> get_lanes_info() {
    return lanes_info_;
  }
  std::vector<std::vector<std::vector<double>>> get_road_edge_info() {
    return road_edge_info_;
  }

private:
  bool calc_lane_perception_obstacle();
  bool calc_refline_origin(bool force_replan = false);
  bool calc_apf_attract_field(
      ApfAttractType &attract_type,
      std::shared_ptr<FrenetCoordinateSystem> &attract_frenet,
      double &attract_theta);
  double distance(Pose2D pose1, Pose2D pose2);
  bool calc_apf_road_ref_theta(double &road_ref_theta);
  bool calc_apf_leader_ref_theta(double &leader_ref_theta);
  bool
  calc_apf_leader_traj(std::shared_ptr<FrenetCoordinateSystem> &attract_frenet);
  bool
  calc_refline_traj(std::shared_ptr<FrenetCoordinateSystem> &attract_frenet);
  void judge_quit_CP();
  bool spline_fit_refline(PiNotebookDebug *pi_nb_debug); // quintic
  void extend_refline_backward(std::vector<Pose2D> &refline_result);
  void gen_refline_backup(double apf_min_length, double vel_pre_dis,
                          PiNotebookDebug *pi_nb_debug);

private:
  ReflineGeneratorCore core_type_;
  std::vector<Pose2D> refline_result_;
  std::vector<Pose2D> last_refline_result_;
  std::vector<planning_math::LineSegment2d> lane_perception_obstacle_;
  std::vector<planning_math::LineSegment2d> road_edge_perception_obstacle_;
  double replan_threshold_ = 1.0;
  double close_ref_point_threshold_ = 2.0;
  Pose2D refline_origin_;
  pass_intersection_planner::ReflineCondition condition_;
  pass_intersection_planner::ReflineCondition last_condition_;
  ApfAttractType attract_type_;
  ApfAttractType last_attract_type_;
  bool has_before_insection_lane_;
  bool has_after_insection_lane_;
  std::vector<HistoricalLeaderInfo> muti_historical_info_;
  FrenetCoordinateSystemParameters attract_coord_parameters_;
  std::unordered_map<int, std::vector<planning_math::Vec2d>> lanes_enu_points_;
  bool always_follow_road_ref_theta_flag;
  int selected_hist_vehicle_id_ = -1;
  std::vector<planning_math::Vec2d> adc_historical_info_;
  std::vector<double> target_pt_;
  int target_pt_count_ = 0;
  int inter_intersection_count_ = 0;
  bool is_target_pt_flag_{false};
  bool apf_lat_far_flag_{false};
  PassIntersectionPlannerInput input_;
  double road_ref_theta_{0.0};
  double lat_dis_in_intersection_{0.0};
  double lat_dis_out_intersection_{0.0};
  int quit_cp_{0};
  std::string apf_quit_cp_reason_ = "";
  int follow_car_id_{0};
  std::vector<double> refline_target_pt_{};
  double lat_dis_2_in_inter_{0.0};
  double max_ego2refline_lat_dis_{0.0};
  double lat_dis_2_refline_{0.0};
  std::vector<std::vector<std::vector<double>>> lanes_info_;
  std::vector<std::vector<std::vector<double>>> road_edge_info_;
  bool leader_car_cross_lane_{false};
  double cur_lane_left_border_{2.25};
  double cur_lane_right_border_{2.25};
};

void points_to_linesegment_filter(
    std::vector<planning_math::Vec2d> &points, double angle_accuracy,
    double max_segment_length,
    std::vector<planning_math::LineSegment2d> &lines);

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_COMMON_REFLINE_GENERATOR_H
