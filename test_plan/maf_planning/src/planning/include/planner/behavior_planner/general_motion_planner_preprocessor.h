#pragma once

#include "common/trajectory/trajectory_stitcher.h"
#include "common/utils/gauss_quad_constants.h"
#include "linear_interpolation_utility.hpp"
#include "planner/message_type.h"
#include "planner/motion_planner/planner_preprocessor.h"
#include "planner/tasks/task.h"

using namespace gmp_interface;
using gmp_interface::ObsInfo;

namespace msquare {

class GeneralMotionPlannerPreprocessor : public PlannerPreprocessor {

public:
  GeneralMotionPlannerPreprocessor() : PlannerPreprocessor(){};

  virtual void process();

  GeneralMotionPlannerInput *get_gmp_input() {
    return general_motion_planner_input_;
  };

  void mutable_external_input(GeneralMotionPlannerInput *gmp_input,
                              GeneralMotionPlannerOutput *gmp_output) {
    general_motion_planner_input_ = gmp_input;
    general_motion_planner_output_ = gmp_output;
  };

  void get_lc_request(LC_Dir request, const bool &valid_body,
                      const bool &is_steer_over_limit,
                      const bool &is_lca_state_activated,
                      const bool &is_lane_stable, const bool &is_solid_line,
                      const bool &gmp_should_cancel) {
    lc_request_ = request;
    valid_body_ = valid_body;
    is_steer_over_limit_ = is_steer_over_limit;
    is_lca_state_activated_ = is_lca_state_activated;
    is_lane_stable_ = is_lane_stable;
    is_solid_line_ = is_solid_line;
    gmp_should_cancel_ = gmp_should_cancel;
  };

private:
  void generate_time_array();
  void process_time_manager();
  double time_array_max_{10.0};
  int time_array_index_max_{100};
  double gmp_prec_created_time_{0.0};
  vector<double> time_array_{};
  vector<gmp_interface::ObsInfo> gmp_obj_infos_{};
  vector<gmp_interface::ObsInfo> concerned_obj_infos_{};
  int call_count_{0};
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;

private:
  gmp_interface::ObjPredSlice
  get_predslice_at_time(const double &time, const Obstacle *obstacle,
                        const double &ego_s,
                        std::shared_ptr<FrenetCoordinateSystem> frenet_coord);

  gmp_interface::ObjPredSlice
  get_predslice_at_time(const double &time, const Point2D &obj_sl,
                        const gmp_interface::ObjPredSlice &obj_info_base,
                        const double &pred_decel);

  Point2D get_obj_base_sl(const gmp_interface::ObjPredSlice &obj_info_base);

  vector<gmp_interface::ObsInfo> generate_gmp_object_info();

  void update_gmp_input_task_info();

  void update_gmp_input_obj();

  void update_gmp_input_ego_state();

  void update_concerned_objs();

  void update_gmp_output();

  void update_lane_info();

  void update_call_count() { call_count_++; };

  gmp_interface::Point2d env2local_pos(gmp_interface::Point2d point_env);

  double pred_valid_time_{1.5};

private:
  GeneralMotionPlannerInput *general_motion_planner_input_;
  GeneralMotionPlannerOutput *general_motion_planner_output_;

private:
  double ego_l0_{0.0};
  vector<double> ego_l0_buffer_{0.0, 0.0, 0.0};
  bool lane_cross_{false};
  Point2D get_ego_sl0(gmp_interface::Point2d ego_pose0);
  void is_lane_crossed();
  void update_lc_state();
  void update_lw_state();
  void update_lc_clear();
  void cal_lc_pass_info();
  void cal_lc_total_info();
  void update_smoothed_headaway();

  LC_Dir lc_request_{LC_Dir::NONE};
  bool valid_body_{false};
  bool is_steer_over_limit_{false};
  bool is_lca_state_activated_{false};
  bool is_lane_stable_;
  bool is_solid_line_;
  bool gmp_should_cancel_;
  LC_Dir lc_state_{LC_Dir::NONE};
  bool lc_wait_{false};
  bool lc_clear_{false};

  vector<LC_Dir> lc_request_buffer_{LC_Dir::NONE, LC_Dir::NONE};
  vector<int> motion_result_buffer_{1, 1};
  vector<int> lc_back_buffer_{0, 0, 0};
  double ego_speed_lc_trigger_{20.0};

  double lc_wait_time_{0.0};
  double lc_trigger_time_{0.0};

  unordered_map<int, double> obj_timer_cache_{};
  vector<gmp_interface::BehaviorCandidate> behavior_candidates_{};
  gmp_interface::DrivingTaskInfo lc_task_info_;

private:
  gmp_interface::TargetPoint target_point_default_{-100.0, 0.0, 0.0,
                                                   0.0,    0.0, 0};

  void generate_target_points(void);

  void generate_behavior_candidates(void);

  void
  generate_behavior_info(gmp_interface::BehaviorCandidate &behavior_candidate);

  void update_equivalent_behavior_info(
      gmp_interface::BehaviorCandidate &behavior_candidate);

  void
  update_target_point(gmp_interface::BehaviorCandidate &behavior_candidate);

  void
  update_target_point_LC(gmp_interface::BehaviorCandidate &behavior_candidate,
                         const double &target_speed);

  void
  update_target_point_LH(gmp_interface::BehaviorCandidate &behavior_candidate,
                         const double &target_speed);

  void set_target_point_speed(vector<gmp_interface::TargetPoint> &target_points,
                              const double &tp_speed);

  void set_target_point_type(vector<gmp_interface::TargetPoint> &target_points,
                             const int &tp_type);

  vector<double> cal_lc_refline_quinyic(double s, double l);

  double cal_refline_y(const vector<double> &c_poly, const double &x_max,
                       double x, int dev_order);

  void extend_frenet(const Point2D &frenet, gmp_interface::Point2d &local,
                     double &heading);

  template <typename T>
  T saturation(const T &x, const T &lo, const T &hi) const {
    return std::max(lo, std::min(hi, x));
  }

  bool is_in_zone(const double &x, const double &y,
                  const vector<pair<double, double>> &zone_lat,
                  const vector<pair<double, double>> &zone_lon);

  bool is_in_target_lane(const double &x, const double &y,
                         const vector<pair<double, double>> &zone_lat,
                         const vector<pair<double, double>> &zone_lon);

  void update_retreat_target_speed(void);

  void update_pos_lc_target_speed(void);

  double smooth_headaway(const double &headaway_current,
                         const double &headaway_target);

  void update_dheadaway(void);

  // bool first_retreat_{true};
  int retreat_count_{0};
  int retreat_count_max_{1};
  double t_pp_end_{10.0};

  double lc_time_pass_{0};
  double lc_time_remain_{7.0};
  double lc_dist_remain_{300};
  double lc_dist_pass_{0};
  double lc_time_total_{7.0};
  double lc_dist_total_{300};
  double lc_latdist_abs_past_{0.0};
  double last_lc_to_static_obj_time_{7.0};

  double lane_width_{3.5};
  double real_lane_width_{3.5};
  double t_delta_{0.1};
  vector<double> t_buffer_{0.0, 0.0};
  double headaway_{1.4};
  double dheadaway_{0.55};
  double dheadaway_shrink_{0.78};
  double dheadaway_origin_{0.55};
  double headaway_smoothed_{1.4};
  double target_speed_lc_wait_{33.3};
  double target_speed_lc_wait_max_{33.612};
  double set_speed_delta_{4.0};
  vector<double> lc_t_total_candidates_{7.0, 7.0, 10.0, 10.0, 7.0, 7.0};

  Transform enu2car_;
};

} // namespace msquare