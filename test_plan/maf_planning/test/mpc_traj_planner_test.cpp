#include "planner/motion_planner/longitudinal_motion_planner.h"
#include "planner/motion_planner/motion_planner.h"

#define TS 0.2

using namespace msquare;

FILE *logger;

msquare::Mpc_traj_planner_init mpc_traj_planner_init;
msquare::Run_traj_planner_mpc run_traj_planner_mpc;

double c_curve_query(double s_vehicle, double c_curve[4]) {
  return c_curve[3] * pow(s_vehicle, 3.0) + c_curve[2] * pow(s_vehicle, 2.0) +
         c_curve[1] * s_vehicle + c_curve[0];
}
double c_curve_trans(double s_vehicle, double r_vehicle, double c_curve[4]) {
  return 1.0 / (1.0 - r_vehicle * c_curve_query(s_vehicle, c_curve));
}
state_t motion_function(state_t motion_state_k, double delta_f_rate, double acc,
                        double c_curve[4]) {
  state_t motion_state_k1;
  motion_state_k1.s_frenet =
      motion_state_k.s_frenet +
      TS * motion_state_k.vel * cos(motion_state_k.theta_error) *
          c_curve_trans(motion_state_k.s_frenet, motion_state_k.r_frenet,
                        c_curve);
  motion_state_k1.r_frenet =
      motion_state_k.r_frenet +
      TS * motion_state_k.vel * sin(motion_state_k.theta_error);
  motion_state_k1.theta_error =
      motion_state_k.theta_error +
      TS *
          (delta_f_rate - motion_state_k.vel * cos(motion_state_k.theta_error) *
                              c_curve_trans(motion_state_k.s_frenet,
                                            motion_state_k.r_frenet, c_curve) *
                              c_curve_query(motion_state_k.s_frenet, c_curve));
  motion_state_k1.vel = motion_state_k.vel + TS * acc;

  return motion_state_k1;
}

int main() {
  const int step = 150;
  static double delta_f_rate = 0.0;
  static double acc = 0.0;
  double vehicle_state_init[4] = {0.0, 0.0, 0.0, 0.0};

  void *handle = dlopen("libtraj_plannermpc.so", RTLD_LOCAL | RTLD_NOW);
  if (handle == NULL) {
    cout << dlerror() << endl;
    return false;
  }

  mpc_traj_planner_init =
      (msquare::Mpc_traj_planner_init)dlsym(handle, "mpc_traj_planner_init");
  run_traj_planner_mpc =
      (msquare::Run_traj_planner_mpc)dlsym(handle, "run_traj_planner_mpc");

  int i, j, num_sim;
  mpc_traj_planner_init(10.0, 1.0, 1.0, 2.0, 10.0, 10.0, vehicle_state_init);

  state_t current_state;
  log_t current_log;
  double c_curve_line[4] = {0.0, 0.0, 0.0, 0.0};
  double vr[kN + 1];
  const double v_set = 8.0;
  double Obstacle_test[kN + 1][8];
  double r_limit_test[kN + 1][2];

  for (i = 0; i <= kN; ++i) {
    vr[i] = v_set;
    Obstacle_test[i][0] = 20.0;
    Obstacle_test[i][1] = 1.0;
    Obstacle_test[i][2] = 40.0;
    Obstacle_test[i][3] = -1.0;
    Obstacle_test[i][4] = 80.0;
    Obstacle_test[i][5] = 3.0;
    Obstacle_test[i][6] = 160.0;
    Obstacle_test[i][7] = -1.0;
  }
  for (j = 0; j <= kN; ++j) {
    r_limit_test[j][0] = -1.75; // r_lower
    r_limit_test[j][1] = 1.75;  // r_upper
  }

  current_state.s_frenet = 0.0;
  current_state.r_frenet = 0.0;
  current_state.theta_error = 0.0;
  current_state.vel = 5.0;

  if ((logger = fopen("/e/logger_prediction.csv", "w+")) == NULL) {
    printf("open file error!\n");
    return 1;
  }

  static double delta_f_rate_last = 0.0;
  static double acc_last = 0.0;

  for (num_sim = 0; num_sim < step; ++num_sim) {
    int nwsr = run_traj_planner_mpc(&current_state, &current_log, c_curve_line,
                                    vr, Obstacle_test, r_limit_test);
    static log_t current_log_last = current_log;
    delta_f_rate = 0.;
    acc = 0.;
    if (nwsr == 0) {
      delta_f_rate = delta_f_rate_last;
      acc = acc_last;
      current_log = current_log_last;
    }

    current_state =
        motion_function(current_state, delta_f_rate, acc, c_curve_line);
    for (i = 0; i <= kN; i++) {
      fprintf(logger, "%f,%f,%f,%f\n", current_log.s_frenet[i],
              current_log.r_frenet[i], current_log.theta_error[i],
              current_log.vel[i]);
    }
    delta_f_rate_last = delta_f_rate;
    acc_last = acc;
    current_log_last = current_log;
    cout << "s_frenet " << current_state.s_frenet << " r_frenet "
         << current_state.r_frenet << "  theta_error "
         << current_state.theta_error << " vel " << current_state.vel
         << " delta_f_rate " << delta_f_rate << " acc " << acc << " nwsr "
         << nwsr << endl;
  }

  int ret;
  if ((ret = fclose(logger)) == -1) {
    printf("close file error!\n");
  }
}
