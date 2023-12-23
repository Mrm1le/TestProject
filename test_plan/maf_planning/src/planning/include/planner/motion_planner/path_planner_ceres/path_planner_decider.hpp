#pragma once
#include "lateral_active_avoid.hpp"

namespace path_planner {

struct AimPolygon {
  int id;
  int type;
  std::string avd_dir;
  double a;
  double max_l;
  double min_l;
  double desire_buffer;
  double lane_border_base_l;
  std::vector<Point2d> polygon;
  std::vector<double> distance_to_lane_line_history;

  bool empty() { return polygon.empty(); }

  Point2d max_y_point() {
    Point2d max_y_p{0, std::numeric_limits<double>::lowest()};
    for (const auto &p : polygon) {
      if (p.y > max_y_p.y)
        max_y_p = p;
    }
    return max_y_p;
  }

  Point2d min_y_point() {
    Point2d min_y_p{0, std::numeric_limits<double>::max()};
    for (const auto &p : polygon) {
      if (p.y < min_y_p.y)
        min_y_p = p;
    }
    return min_y_p;
  }
};

struct PathDeciderKeyParams {
  std::pair<double, double> pred_vilad_time_max;
  std::pair<double, double> pred_vilad_time_min;
};

class PathPlannerDecider {
public:
  PathPlannerDecider() = default;

  void update_input(const PathPlannerInput &input);

  const DecisionInfo *get_activation_decision_ptr() const {
    return &decision_info_;
  }

  const LaneLineActivationDist &get_lane_activation_dist() const {
    return lane_activation_dis_;
  }

  const LaneLineActivationDist &get_obj_activation_dist() const {
    return obj_activation_dis_;
  }

  const Point2d &get_lc_end_point() const { return lc_end_point_fren_; }

  const std::unordered_map<int, double> &get_active_target_offset() const {
    return active_target_l_;
  }

  Intelligent_Dodge_Info get_dodge_info() const {
    return pre_dodge_info_;
  }

  const AimPolygon &get_left_aim() const { return left_aim_; }
  const AimPolygon &get_right_aim() const { return right_aim_; }

private:
  void update_key_params(std::string mode = "default");
  void update_offset_list_and_active_from_ddp();
  void init_offset_list();
  void offset_active();
  void update_road_border_bound();
  void update_obstacle_bound();
  void update_desire_bound();
  void smooth_bound();
  void slash_activitation_dist();
  void sample_quintic_poly();
  void update_end_s_for_collision_avoidance(const double s_pred,
                                            const double l_pred,
                                            double &desire_end_s);
  void update_tail_inflation(const double &tail_s, const double &desire_l,
                             const double &lane_border_base_l,
                             const double &dddl, const std::string &direction);

  void update_head_inflation(const double &head_s, const double &desire_l,
                             const double &lane_border_base_l,
                             const double &dddl,
                             const bool &reverse_avd_concerned,
                             const std::string &direction);

  double compute_obstacle_inflation_jerk(
      const std::vector<double> &distance_to_lane_line_history);

  void update_desire_bound_quintic_poly(
      const msquare::planning_math::QuinticPoly1d *quintic_poly,
      const double &s_start, const double &s_end, const std::string &direction);

  void update_desire_bound_const_l(const double &s_start, const double &const_l,
                                   const std::string &direction);

  double compute_press_lane_border_buffer(
      const std::vector<double> &distance_to_lane_line_history);

  bool judge_overtake(const ObsInfo &obstacle);

  bool cal_ref_offset(const size_t &segment_index, const size_t &quad_index,
                      const std::vector<Point2d> &polygon,
                      const ObsInfo::NudgeType &nudge_side, const double &max_l,
                      const double &min_l, const double &lon_buffer_base,
                      double &ref_offset);

  double prediction_valid_time(const double &v);

  bool current_lane_end_distinguish();

  double cal_soft_road_border_buffer(const PathSampleInfo &sample_info,
                                     const std::string &direction,
                                     const bool is_merge,
                                     const double hard_buffer);

  double cal_hard_road_border_buffer(const PathSampleInfo &sample_info,
                                     const std::string &direction);

  PathPlannerInput input_;
  DecisionInfo decision_info_;
  LaneLineActivationDist lane_activation_dis_;
  LaneLineActivationDist obj_activation_dis_;
  Point2d lc_end_point_fren_;

  AimPolygon left_aim_;
  AimPolygon right_aim_;
  double half_self_width_;
  double half_self_length_;

  std::unordered_map<int, double> active_target_l_;
  Intelligent_Dodge_Info pre_dodge_info_;

  PathDeciderKeyParams key_params_;
};
} // namespace path_planner
