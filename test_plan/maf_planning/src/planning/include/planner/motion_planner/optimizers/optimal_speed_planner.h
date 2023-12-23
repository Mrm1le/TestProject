#ifndef OPTIMAL_SPEED_PLANNER_H_
#define OPTIMAL_SPEED_PLANNER_H_

#include "common/math/math_utils.h"
#include "planner/message_type.h"
#include "planner/planning_config.h"
#include <dlfcn.h>

namespace msquare {
typedef void (*Speed_mpc_planner_init)(double, double, double, double, double,
                                       state_t *vehicle_state);

typedef int (*Run_speed_mpc_planner)(state_t *x0, log_t *solution,
                                     control_out_t *mpc_u_out, double *,
                                     double *, double *, double *, double *,
                                     double *, double *, double, double, double,
                                     double, double, double);

class OptimalSpeedPlanner {
public:
  OptimalSpeedPlanner();

  ~OptimalSpeedPlanner() = default;

  // first step  inital mpc planner
  bool InitalOptimalPLanner(state_t *vehicle_state);

  // second step set online data
  void SetSr(const vector<double> sr);

  void SetVr(const vector<double> vr);

  void SetSupper(const vector<double> s_upper);

  void SetSlower(const vector<double> s_lower);

  void SetCurvature(const vector<double> curvature);

  void SetAupper(const vector<double> a_upper);

  void SetAlower(const vector<double> a_lower);

  // third step  run mpc planner
  bool RunSpeedMpcPlanner(const state_t &vehicle_real_state,
                          const state_t &vehicle_state,
                          std::array<double, 6> mpc_checker = {
                              2.0, -7.0, 30.0, -1.0, 7.0, -6.0});

  // step four get speed mpc planner calculate result
  void GetSOut(vector<double> &s_out);

  void GetVelOut(vector<double> &vel_out);

  void GetAccOut(vector<double> &acc_out);

  void GetJerkOut(vector<double> &jerk_out);

  // Special step when you need  change  cost weight  use this function
  void SetWeightParam(const std::array<double, 5> &cost_weight);

private:
  bool GetOnlineData(speed_mpc_online_data &mpc_online_data);

  bool CheckOnlineData();

  void ClearOutData();

  void *handle_longitudinal_ = nullptr;

  state_t vehicle_real_state_;
  state_t vehicle_state_;
  std::array<double, 5> cost_weight_;
  Speed_mpc_planner_init speed_mpc_planner_init_;
  Run_speed_mpc_planner run_speed_mpc_planner_;

  // speed mpc planner online data
  std::vector<double> sr_;
  std::vector<double> vr_;
  std::vector<double> s_upper_;
  std::vector<double> s_lower_;
  std::vector<double> curvature_;
  std::vector<double> a_upper_;
  std::vector<double> a_lower_;

  // speed mpc planner out data
  std::vector<double> s_frenet_out_;
  std::vector<double> vel_out_;
  std::vector<double> acc_out_;
  std::vector<double> jerk_control_;

  // mpc planner calculate result
  log_t speed_planner_mpc_log_;
  control_out_t speed_planner_control_out_;
};

} // namespace msquare

#endif // OPTIMAL_SPEED_PLANNER_H_
