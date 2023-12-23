#ifndef MSQUARE_DECISION_PLANNING_PLANNER_GENERAL_MOTION_PLANNER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_GENERAL_MOTION_PLANNER_H_

#include "common/extract_path_bound.h"
// #include "planner/behavior_planner/lateral_behavior_path_planner.h"
// #include "planner/behavior_planner/lateral_behavior_request_manager.h"
// #include "planner/behavior_planner/lateral_behavior_state_machine.h"
#include "common/config_context.h"
#include "planner/behavior_planner/general_motion_planner_preprocessor.h"
#include "planner/tasks/task.h"

#include <array>

namespace msquare {

struct PathSegment {
  double c0{0.0};
  double c1{0.0};
  double c2{0.0};
  double c3{0.0};
  double c4{0.0};
  double c5{0.0};
  double x_end{0.0}; // in path, this is lateral; in s-t, this is s
  double y_end{0.0}; // in path, this is longit.; in s-t, this is t
};

struct STContainer {
  double cost{200.0};
  vector<PathSegment> speed_segments{};
};

struct STSample {
  double s;
  double t;
  double v;
  double a;
};

struct STArray {
  vector<double> s{};
  vector<double> v{};
  vector<double> a{};
};

struct BehaviorAdvice {
  vector<PathSegment> path_segments{};
  vector<PathSegment> speed_segments{};
  int behavior_type{0};
  int motion_type{0};
  vector<SVPoint> s_v{}; // x:s, y:v_min, z: v_max
  double veh_speed_end{0.0};
  double cost{200.0};
  gmp_interface::TrajectoryGMP trajectory{};
  STArray st_array;
  int behavior_candidate_index{-1};
  gmp_interface::DrivingTaskInfo behavior_info{};
};

struct DodgeStrategy {
  int collision_posture;
  int speed_adjustment_bestshot;
  int collision_id; // relative id:  concerned_objs[collision_id]
  Point2D collision_ego_pos;
  Point2D collision_obj_pos;
  double t_col;
  vector<double> st_s_offset_bestshot{};
  double obj_safety_margin;
  double collision_obj_length;
  double collision_obj_vel;
  double end_accel;
  int collision_id_cci;
  int behavior_type;
  int motion_type;
};

class GeneralMotionPlanner : public Task {

public:
  GeneralMotionPlanner(const TaskConfig &config);

  virtual ~GeneralMotionPlanner();

  virtual void init(std::shared_ptr<WorldModel> world_model);

  void reset(const TaskConfig &config);

  void unset();

  TaskStatus execute(ScenarioFacadeContext *context);

public:
  void get_external_request(LC_Dir external_request, bool valid_body,
                            bool is_steer_over_limit,
                            bool is_lca_state_activated, bool is_lane_stable,
                            bool is_solid_line, bool has_olane, bool has_tlane,
                            bool gmp_should_cancel);

  LC_Dir lc_request_{LC_Dir::NONE};
  bool valid_body_;
  bool is_steer_over_limit_;
  bool is_lca_state_activated_;
  bool is_lane_stable_;
  bool is_solid_line_;
  bool has_olane_;
  bool has_tlane_;
  bool gmp_should_cancel_;
  int get_call_count() { return call_count_; };

private:
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;

  std::shared_ptr<BaseLineInfo> gmp_baseline_info_;

private:
  // gmp_interface::Point2d env2local_pos(gmp_interface::Point2d point_env);

  GeneralMotionPlannerInput general_motion_planner_input_{};

  GeneralMotionPlannerOutput general_motion_planner_output_{};

  std::shared_ptr<GeneralMotionPlannerPreprocessor> gmp_preprocessor_;

  int get_motion_type(const BehaviorCandidate &behavior_canditate) const;

  bool path_planning(const BehaviorCandidate &behavior_candidate,
                     BehaviorAdvice &behavior_advice);

  bool speed_planning(BehaviorAdvice &behavior);

  void warm_start_speed(BehaviorAdvice &behavior);

  void cruise_speed_planning(const double &set_speed, BehaviorAdvice &behavior);

  void extend_st_segment(BehaviorAdvice &behavior);

  void update_dodge_strategy(DodgeStrategy &dodge_strategy);

  void update_retreat_speed_adjustment(const DodgeStrategy &dodge_strategy);

  void update_retreat_speed_adjustment(const double &st_ay_o,
                                       const double &st_ay);

  void update_st_evasive(const DodgeStrategy &dodge_strategy,
                         const int &search_origin_index,
                         const int &parallel_search_index,
                         const bool &newseg_invalid, bool &st_search_evasive,
                         int &searching_step);
  double cal_inspired_acceleration(const DodgeStrategy &dodge_strategy,
                                   const BehaviorAdvice &behavior,
                                   const int &search_origin_index);

  void negotiate_safety_margin(DodgeStrategy &dodge_strategy,
                               const BehaviorAdvice &behavior,
                               const int &search_origin_index);

  void update_concerned_obj_safety_margin(const DodgeStrategy &dodge_strategy);

  DodgeStrategy
  collision_detect_dyn(BehaviorAdvice &behavior,
                       vector<gmp_interface::ObsInfo> &concerned_objs);

  bool collision_detect_dyn_1v1(const BehaviorAdvice &behavior,
                                const gmp_interface::ObsInfo &obj,
                                DodgeStrategy &dodge_strategy,
                                const int &t_index, const int &obj_index,
                                unordered_set<int> &ignored_objs);

  void update_speed_segment(
      BehaviorAdvice &behavior, vector<DodgeStrategy> &dodge_strategies,
      vector<STSample> &st_sample_cache, const vector<int> &search_progress,
      const int &search_origin_index, const int &parallel_search_index,
      const int &target_speed_search_index, double st_ay_ini, int &search_step);

  PathSegment plan_path_segment(const gmp_interface::TargetPoint &tp_start,
                                const gmp_interface::TargetPoint &tp_end);

  PathSegment plan_path_segment(const STSample &st_start,
                                const STSample &st_end);

  void get_path_point(BehaviorAdvice &behavior);

  gmp_interface::TrajectoryGMP
  get_path_point(const vector<PathSegment> &path_segments);

  double get_path_point(const int &order,
                        const vector<PathSegment> &path_segments,
                        const double &curr_sample);

  STArray generate_st_array(const BehaviorAdvice &behavior);

  vector<SVPoint>
  generate_sv_array(const vector<vector<PathSegment>> &st_results,
                    const BehaviorAdvice &behavior);

  void gerenate_s_at_control_quad_points();

  double interpolate_s_v_point(const vector<PathSegment> &st_result,
                               const double &s_point);

  vector<PathSegment> path_segments_{}; // C0~C5 and breakpoint

  vector<BehaviorAdvice> behavior_choices_{};

  int retreat_speed_adjustment_{
      0}; // 1: accelerate, 2: decelerate; 0: do nothing

  void update_task_type(void);

  void update_max_retreat_count(void);

  bool check_rss_safety(void);

  void update_lc_proceed_time(void);

  double infer_behavior_success_prob(const int &behavior_state);

  void conclude_lane_change_decision(void);

  void update_best_gap(const BehaviorAdvice &behavior);

  vector<ObjInfoRefined> extract_gap_info(const BehaviorAdvice &behavior);

  vector<ObjInfoRefined> extract_cipv_info(const BehaviorAdvice &behavior);

  vector<ObjInfoRefined> extract_lead_objs_info(const BehaviorAdvice &behavior);

  void update_warm_start(const BehaviorAdvice &behavior_advice_best);

  void update_warm_start(const int &motion_type);

  int task_type_{1};

  int lc_action_state_{0};

  int retreat_count_max_{1};

  double lc_proceed_time_{0.0};

  vector<int> lc_action_state_buffer_{0, 0};

  vector<int> behavior_success_buffer_{1, 1, 1, 1, 1};

  int back_count_{0};
  vector<gmp_interface::AllObjsMarginInfo> gmp_objs_sm_buffer_{};

  bool warm_start_usable_{false};

  BehaviorAdvice behavior_cache_{};

private:
  int check_current_process(const vector<int> &search_progress);

  int update_search_progress(const int &next_step);

  int get_search_origin(const vector<int> &search_progress_o,
                        const vector<int> &search_progress);

  void update_st_limit(const int &search_origin_index,
                       const BehaviorAdvice &behavior,
                       const DodgeStrategy &dodge_strategy);

  bool is_speed_unreachable(const BehaviorAdvice &behavior,
                            const int &current_step);

  void backtrack_speed_segment(BehaviorAdvice &behavior,
                               const vector<int> &search_progress);

  void update_task_result_buffer(const int &task_type, const int &motion_type);

  void update_task_result_buffer(const BehaviorAdvice &behavior_advice);

  void update_max_speed(void);

  vector<PathSegment>
  evaluate_speed_segments(const vector<vector<PathSegment>> &st_results);

  bool is_rear_wild(const int &id_rel);

  BehaviorAdvice evaluate_behavior(vector<BehaviorAdvice> &behavior_container);

  double st_speed_limit_{34.72};
  double accel_limit_control_{2.0};
  double decel_limit_control_{-3.0};
  double decel_limit_{-5.5};
  double accel_limit_{5.5};
  double v_max_{33.3};
  double lc_to_static_obj_time_{0.0};
  bool lc_to_static_obj_{false};

  vector<int> motion_result_buffer_{1, 1};
  vector<int> task_type_buffer_{1, 1};
  vector<double> st_a_buffer_{0.0, 0.0};

  gmp_interface::ObsInfo obj_cci_cache_;
  vector<int> obj_static_id_caches_{};

private:
  void
  update_general_motion_planner_output(const BehaviorAdvice &behavior_advice);

  void set_debug_info(void);

  void reset_debug_info(void);

  double cal_cipv_ttc();
  void update_lc_time();
  void update_call_count() { call_count_++; };

  void update_obj_safety_margin_display(const int &behavior_idx,
                                        const int &obj_index,
                                        const int &t_index,
                                        const double &new_sm_lon,
                                        const double &new_sm_lat);

  void update_obj_safety_margin_display(const BehaviorAdvice &behavior,
                                        const DodgeStrategy &dodge_strategy);
  void update_objs_display_sm();

  void add_ego_setspeed_virtual_obj(const BehaviorAdvice &behavior);

  int call_count_{0};

private:
  // internal signal trigger for offboard test
  void get_created_time(void);

  int simulate_lc_trigger(const int &direc, const double &time);

  double gmp_created_time_{0.0};

private:
  std::array<double, speed_planner::NUM_SPEED_CONTROL_POINTS>
      s_at_control_points_;
  std::array<std::array<double, path_planner::QUADRATURE_ORDER>,
             speed_planner::NUM_SPEED_SEGMENTS>
      s_at_quad_points_;
};

} // namespace msquare

#endif