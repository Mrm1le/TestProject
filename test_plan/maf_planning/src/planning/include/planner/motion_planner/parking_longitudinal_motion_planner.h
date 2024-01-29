#ifndef MSQUARE_DECISION_PLANNING_PLANNER_MOTION_PLANNER_PARKING_LONGITUDINAL_MOTION_PLANNER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_MOTION_PLANNER_PARKING_LONGITUDINAL_MOTION_PLANNER_H_

#include <dlfcn.h>

#include "common/obstacle_manager.h"
#include "common/utils/timer.h"
#include "planner/behavior_planner/deciders/collision_checker.h"
#include "planner/behavior_planner/parking/speed_margin_limiter.h"
#include "planner/message_type.h"
#include "planner/motion_planner/parking_motion_planner.h"
#include "planner/planning_config.h"

#include "planner/behavior_planner/parking/sv_speed_generator.h"

namespace msquare {

namespace parking {

class ParkingLongitudinalMotionPlanner : public MotionPlanner {
public:
  explicit ParkingLongitudinalMotionPlanner(
      const std::shared_ptr<WorldModel> &world_model);
  virtual ~ParkingLongitudinalMotionPlanner() = default;

  virtual bool calculate();

private:
  bool run_calculate();

private:
  bool clear_each_loop();

  bool compute(
      const LeaderPair lead_cars, const FreespacePoint free_space,
      const FreespaceLine fs_line,
      const std::vector<MultiDirectionalCars> multidirectional_cars,
      const std::vector<MultiDirectionalHuman> multidirectional_human,
      const std::vector<IntentionStatusObstacles> intention_status_obstacles,
      const std::vector<int> predcition_obstacles, const double v_ego,
      const double angle_steers, const StatusType status_type,
      const bool is_entrance, const int scene_avp,
      const std::vector<double> planning_mpc_diff);
  bool computeVel(
      const LeaderPair lead_cars, const FreespacePoint free_space,
      const FreespaceLine fs_line,
      const std::vector<MultiDirectionalCars> multidirectional_cars,
      const std::vector<MultiDirectionalHuman> multidirectional_human,
      const std::vector<IntentionStatusObstacles> intention_status_obstacles,
      const std::vector<int> predcition_obstacles, const double v_ego,
      const double angle_steers, const StatusType status_type,
      const bool is_entrance, const int scene_avp,
      const std::vector<double> planning_mpc_diff);
  bool computeVelV2(
      const LeaderPair lead_cars, const FreespacePoint free_space,
      const FreespaceLine fs_line,
      const std::vector<MultiDirectionalCars> multidirectional_cars,
      const std::vector<MultiDirectionalHuman> multidirectional_human,
      const std::vector<IntentionStatusObstacles> intention_status_obstacles,
      const std::vector<int> predcition_obstacles, const double v_ego,
      const double angle_steers, const StatusType status_type,
      const bool is_entrance, const int scene_avp,
      const std::vector<double> planning_mpc_diff);
  bool compute_speed_with_leads(const LeaderPair lead_cars, const double v_ego,
                                const bool isAPA);
  bool compute_speed_for_freespace(const FreespacePoint lead_point,
                                   const double v_ego);
  bool compute_speed_for_freespace(const FreespaceLine &fs_line,
                                   const double v_ego);
  bool compute_speed_for_parking(const double dist, const double v_ego,
                                 const bool isAPA = true);
  bool compute_speed_for_near_obstacle(const double v_ego, const int scene_avp);
  bool compute_speed_for_multidirectional_cars_frenet(
      const std::vector<MultiDirectionalCars> multidirectional_cars,
      const double v_ego);
  bool compute_speed_for_multidirectional_cars_ego(
      const std::vector<MultiDirectionalCars> multidirectional_cars,
      const double v_ego);
  bool compute_speed_for_multidirectional_human(
      const std::vector<MultiDirectionalHuman> multidirectional_human,
      const double v_ego);
  bool limit_accel_velocity_in_turns(const double v_ego,
                                     const double angle_steers);
  bool limit_speed_before_turns(const double v_ego, const int scene_avp);
  bool limit_speed_for_large_curvature(const double v_ego, const int scene_avp);
  bool limit_speed_for_avoid_obstacle(const double v_ego);
  bool compute_speed_for_intention_obstacle(
      const std::vector<IntentionStatusObstacles> intention_status_obstacles,
      const double v_ego);
  bool compute_speed_for_prediction_obstacle(
      const std::vector<int> predcition_obstacles, const double v_ego);
  bool compute_speed_for_apa();
  bool compute_speed_for_uxe(bool force_stop);
  bool compute_speed_for_gate(const double v_ego);
  bool limit_speed_for_radical_cars_while_APA();
  bool
  limit_speed_for_mpc_deviation(const std::vector<double> planning_mpc_diff);
  double process_a_lead(const double a_lead);
  double calc_desired_speed(const double d_rel, const double d_des,
                            const double v_lead_unp, const double a_lead_p);
  bool calc_acc_accel_limits(const double d_lead, const double d_des,
                             const double v_ego, const double v_lead_unp,
                             const double a_lead, const double v_target,
                             std::pair<double, double> &a_target);
  double calc_desired_distance(const double v_lead, const double v_ego,
                               const bool isAPA = false,
                               const bool is_freespace = false,
                               bool is_static = true,
                               IntentionObsType type = IntentionObsType::NONE);
  double calc_positive_accel_limit(const double d_lead, const double d_des,
                                   const double v_ego, const double v_rel,
                                   const double v_target,
                                   const double a_lead_contr,
                                   const double a_max_const);
  double clip(const double x, const double lo, const double hi);
  double calc_critical_decel(const double d_lead, const double v_rel,
                             const double d_offset, const double v_offset);
  bool calc_cruise_accel_limits(const double v_ego);
  double interp(double x, const std::vector<double> &xp,
                const std::vector<double> &fp);
  bool is_in_parking_slot();
  void set_planning_result(const double v_ego);

  // 设置规划结果
  void set_planning_result(const std::vector<double> &c_param);
  bool checkBlock(const LeaderPair &lead_cars, const FreespacePoint &free_space,
                  const FreespaceLine &fs_line);

  // sop speed planner
  bool limit_speed_for_margin_limit();
  bool marginVelocityLimit(const Pose2DTrajectory &plan_traj,
                           const std::vector<double> &curs);
  bool checkBlockV2(const LeaderPair &lead_cars,
                    const FreespacePoint &free_space,
                    const FreespaceLine &fs_line);
  bool compute_speed_with_leadsV2(const LeaderPair lead_cars,
                                  const double v_ego, const bool isAPA);
  void compute_speed_for_remain_distanceV2(const LeaderPair &lead_cars,
                                           const FreespacePoint &lead_point,
                                           const FreespaceLine &fs_line,
                                           bool is_reverse);
  bool compute_speed_for_prediction_obstacleV2(
      const std::vector<int> predcition_obstacles, const double v_ego);
  bool speed_qp_optimize(bool is_path_update);
  void compute_speed_use_osqp(const LeaderPair &lead_cars,
                              const FreespacePoint &lead_point,
                              const FreespaceLine &fs_line, bool is_reverse,
                              bool is_stop_status);
  double compute_remain_distance_for_qp(const LeaderPair &lead_cars,
                                        const FreespacePoint &lead_point,
                                        const FreespaceLine &fs_line);
  bool compute_speed_for_apaV2();
  void compute_speed_for_curvature();
  void set_planning_resultV2(const double v_ego);
  double getInitSpeedForQP(const SpeedData &speed_data, double ego_speed,
                           double *init_a);
  bool computeSTFromODObs(const LeaderPair lead_cars, const double v_ego,
                          const bool isAPA,
                          msquare::parking::VecST &first_st_obs,
                          msquare::parking::VecST &second_st_obs,
                          double delta_t);

  bool create_planning_debug_string(const std::pair<double, double> &a_target);

private:
  const std::vector<double> _A_CRUISE_MIN_BP{0.0, 5.0, 10.0, 20.0, 40.0};
  const std::vector<double> _A_CRUISE_MIN_V{-0.4, -0.4, -0.67, -0.5, -0.30};
  const std::vector<double> _A_CRUISE_MAX_BP{0.0, 5.0, 10.0, 20.0, 40.0};
  const std::vector<double> _A_CRUISE_MAX_V{0.2, 0.2, 0.8, 0.5, 0.30};
  const std::vector<double> _T_GAP_VEGO_BP{5.0, 15.0, 30.0};
  const std::vector<double> _T_GAP_VEGO_V{1.35, 1.55, 2.0};
  const std::vector<double> _L_SLOPE_BP{0.0, 3.0};
  const std::vector<double> _L_SLOPE_V{0.2, 0.1};
  const std::vector<double> _P_SLOPE_V{0.2, 0.1};
  const std::vector<double> _P_SLOPE_BP{0., 3.0};
  const std::vector<double> _A_LEAD_LOW_SPEED_V{0.0, 1.0};
  const std::vector<double> _A_LEAD_LOW_SPEED_BP{0.0, 10.0};
  const std::vector<double> _DECEL_OFFSET_V{-0.3, -0.5, -0.5, -0.4, -0.3};
  const std::vector<double> _DECEL_OFFSET_BP{0.0, 4.0, 15.0, 30.0, 40.0};
  const std::vector<double> _A_CORR_BY_SPEED_V{0.4, 0.4, 0.0};
  const std::vector<double> _A_CORR_BY_SPEED_BP{0.0, 2.0, 10.0};
  const std::vector<double> _A_TOTAL_MAX_V{1.5, 1.9, 3.2};
  const std::vector<double> _A_TOTAL_MAX_BP{0., 20., 40.};
  const std::vector<double> _AY_MAX_STEERS{0.1, 0.1, 0.8};
  const std::vector<double> _AY_MAX_ABS_BP{0., 1., 5.};
  // const std::vector<double> _AY_MAX_STEERS {2.5, 2.5, 1.7};
  // const std::vector<double> _AY_MAX_ABS_BP {5., 15., 30.};
  const std::vector<double> _DIST_TURNS_BP{0.0, 10.0, 15.0};
  const std::vector<double> _DIST_TURNS_BP_RAMP{0.0, 15.0, 30.0};
  const std::vector<double> _DIST_TURNS_V{1.39, 1.94, 2.22};
  const std::vector<double> _PREDICTION_BP{3.0, 6.0};
  const std::vector<double> _PREDICTION_V{1.67, 2.22};
  const double _A_MAX = 2.0;
  const double _A_MIN = -4.0;

  const double _V_MIN_APA = 0.3;

  std::vector<const Obstacle *> obstacles_;

  double v_target_ = 80.0 / 3.6;
  double static_distance_ = 4.0;
  std::pair<double, double> a_target_;
  std::pair<double, double> a_target_objective_;
  double v_apa_ = 80.0 / 3.6;
  double a_apa_ = 0.0;
  double v_cruise = 10.0 / 3.6;
  int block_count_ = 0;
  bool blocked_ = false;
  bool fs_blocked_ = false;
  bool lateral_planning_failed_ = false;
  bool pullover_enable_ = false;
  double block_timeout_duration_ = 5.0;
  log_t lane_keeping_log_;

  double rate_ = 10.0;
  double current_v_ = 0.0;
  double current_a_ = 0.0;
  double current_jerk_ = 0.0;
  double last_margin_v_ = -1.0;

  Timer blocker_timer_;
  Timer parking_stopper_timer_;
  GearState gear_ = GearState::PARK;
  CollisionChecker collision_checker_;

  double remaining_distance_ = 100;
  SpeedMarginLimiter speed_margin_limiter_;
  LastPose last_pose_;
  SpeedRes last_res_;
  SvSpeedParam sv_speed_param_;

  // debug info
  std::vector<double> vec_target_speed_debug_;
  int current_v_count_debug_ = 0;
};

} // namespace parking

} // namespace msquare

#endif
