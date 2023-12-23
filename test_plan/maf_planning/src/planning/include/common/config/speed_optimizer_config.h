#ifndef SPEED_OPTIMIZERR_CONFIG_H_
#define SPEED_OPTIMIZERR_CONFIG_H_

#include "mpc_define.h"

namespace msquare {
namespace speed_optimize_config {

// cost weight
constexpr double default_curvature_weight = 0.0;
constexpr double default_accel_weight = 1.0;
constexpr double default_jerk_weight = 5.0;
constexpr double default_s_reference_weight = 5.0;
constexpr double default_v_reference_weight = 0.0;

// optimal_speed_planner param
constexpr int online_data_num = 7;

// optimization parameters
constexpr double delta_t = 0.2;
constexpr int horizon_step = MPC_N;

// typedef struct {
//   double s_frenet, r_frenet, theta_error,vel,acc,omega;
// } state_t;

// typedef struct {
//     double s_frenet[speed_optimize_config::horizon_step + 1];
//     double r_frenet[speed_optimize_config::horizon_step + 1];
//     double theta_error[speed_optimize_config::horizon_step + 1];
//     double vel[speed_optimize_config::horizon_step + 1];
//     double acc[speed_optimize_config::horizon_step + 1];
//     double omega[speed_optimize_config::horizon_step +1];
// } log_t;

// typedef struct {
//     double omega_rate[speed_optimize_config::horizon_step + 1];
//     double jerk[speed_optimize_config::horizon_step + 1];
// } control_out_t;

// fallback configuration
constexpr double FLAGS_fallback_time_unit = 0.2;
constexpr double FLAGS_fallback_total_time = 5.0;
constexpr double FLAGS_slowdown_profile_deceleration = -4.0;

enum class speed_planner_state { COMPUTE_ERROR, COMPUTE_CORRECT, INITAL_ERROR };

} // namespace speed_optimize_config
} // namespace msquare

#endif /* SPEED_OPTIMIZER_CONFIG_H_ */
