#ifndef MSQUARE_DECISION_PLANNING_PLANNER_SPEED_BOUNDARY_DECIDER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_SPEED_BOUNDARY_DECIDER_H_

#include "common/config/task_config.h"
#include "planner/behavior_planner/deciders/st_graph_generator.h"
#include "planner/tasks/deciders/decider.h"

namespace msquare {

class SpeedBoundaryDecider : public Decider {
public:
  SpeedBoundaryDecider(const TaskConfig &config);

  virtual void init(std::shared_ptr<WorldModel> world_model);

  void reset(const TaskConfig &config);

  void unset();

  std::shared_ptr<StGraphGenerator> get_st_graph();

private:
  TaskStatus process();

  void set_speed_limit();

  void cp_curve_speed(double &vel_limit, double &a_cross, const double ego_vel,
                      PlanningStatus *planning_status);

  void acc_curve_speed(double &vel_limit, double &a_cross,
                       const double ego_vel);

  void getTargetLaneCurv(
    double &vel_limit, double &a_cross, const double curv_turn,
    const double max_centrifugal_acceleration, const double curv_filter_param,
    const double ego_vel, const double pre_brake_curv,
    PlanningStatus *planning_status);

  int n_prebrake_curv_ = 0;
  double last_v_lim_curv_ = 33.3;
  double last_a_cross_ = 0.0;
  int get_dbw_time_ = 0;
  bool just_in_dbw_ = false;
  bool last_acc_mode_ = false;

  static constexpr double kCurvKneePt1Mps = 3.0;
  static constexpr double kCurvKneePt2Mps = 6.0;
  static constexpr double kSteerAngleaFilter = 0.15;
  static constexpr double kYawRateFilter = 0.15;
  static constexpr double kMaxAy = 1.2;
  static constexpr int kDBWWaitTime = 3;

private:
  std::shared_ptr<StGraphGenerator> st_graph_generator_;
};

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_PLANNER_SPEED_BOUNDARY_DECIDER_H_
