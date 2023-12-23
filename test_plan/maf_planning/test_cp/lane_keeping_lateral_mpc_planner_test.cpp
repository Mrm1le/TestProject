#include <time.h>
// #include "mpc_traj_planner.hpp"
#include "eigen3/Eigen/Dense"
#include <dlfcn.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "planner/message_type.h"

using namespace std;
using namespace Eigen;
using namespace msquare;

#define TS 0.2

FILE *logger;
FILE *logger1;

double c_curve_query(double s_vehicle, double c_curve[4]) {
  return c_curve[3] * pow(s_vehicle, 3.0) + c_curve[2] * pow(s_vehicle, 2.0) +
         c_curve[1] * s_vehicle + c_curve[0];
}
double c_curve_trans(double s_vehicle, double r_vehicle, double c_curve[4]) {
  return 1.0 / (1.0 - r_vehicle * c_curve_query(s_vehicle, c_curve));
}

state_t motion_function(state_t motion_state_k, double vs, double omega_rate,
                        double c_curve[4]) {
  state_t motion_state_k1;
  motion_state_k1.s_frenet = motion_state_k.s_frenet + TS * vs;
  motion_state_k1.r_frenet =
      motion_state_k.r_frenet +
      TS * vs * tan(motion_state_k.theta_error) *
          (1.0 - motion_state_k.r_frenet *
                     c_curve_query(motion_state_k.s_frenet, c_curve));
  motion_state_k1.theta_error =
      motion_state_k.theta_error +
      TS * (motion_state_k.omega -
            vs * c_curve_query(motion_state_k.s_frenet, c_curve));
  motion_state_k1.omega = motion_state_k.omega + TS * omega_rate;

  return motion_state_k1;
}

typedef void (*Mpc_traj_planner_init)(double, double, double, double);
typedef int (*Run_traj_planner_mpc)(state_t *x0, log_t *solution,
                                    control_out_t *mpc_u_out, double c_curve[4],
                                    double *, double *, double *, double *,
                                    double *, double *, double *, double *);

Mpc_traj_planner_init mpc_traj_planner_init;
Run_traj_planner_mpc run_traj_planner_mpc;

control_out_t
run_lane_keeping_mpc_planner(Run_traj_planner_mpc run_traj_planner_mpc,
                             Mpc_traj_planner_init mpc_traj_planner_init,
                             double sr[kN + 1], double vs[kN + 1],
                             double c_curve[4], state_t current_state,
                             double r_limit[kN + 1][2]) {
  // static int flagNBO  = 0;
  // double eta = 0.02;
  // const double dt_vvr = 3.0;
  // const double dv_lim = 5.0;
  // const double ds_lim = 5.0;
  // double v_set;
  // double sl = 1000000; // =  leader_ref[0][0];
  // double vl = 1000000; // =  leader_ref[0][1];
  double rr[kN + 1], r_lower[kN + 1], r_upper[kN + 1];
  double omega_rate_min[kN + 1], omega_rate_max[kN + 1], omega_min[kN + 1],
      omega_max[kN + 1], /*w1, w2,*/ dw1, dw2;
  double ss;
  int flagTurn, flagNBO;
  log_t current_log;
  control_out_t mpc_output;

  ss = current_state.s_frenet;
  flagTurn = 0;
  // while(ss<current_state.s_frenet + vs[0] * 8.0)
  while (ss < sr[kN]) {
    if (std::abs(c_curve[0] + c_curve[1] * ss + c_curve[2] * std::pow(ss, 2) +
                 c_curve[3] * std::pow(ss, 3)) > 0.01) {
      flagTurn = 1;
      break;
    };
    ss = ss + 5.0;
  }
  if (flagTurn == 1) {
    // w2 = std::min(10.0/(vs[0]+0.01),0.3+std::abs(vs[0]*(c_curve[0] +
    // c_curve[1]*current_state.s_frenet +
    // c_curve[2]*std::pow(current_state.s_frenet,2) +
    // c_curve[3]*std::pow(current_state.s_frenet,3))));  w2 =
    // std::min(0.6, 10.0/(vs[0]+0.01));  w1 = -w2;
    dw2 = 0.4; // 1.0;//0.2;
    dw1 = -dw2;
  } else {
    // w2 = std::min(10.0/(vs[0]+0.01),0.05+std::abs(vs[0]*(c_curve[0] +
    // c_curve[1]*current_state.s_frenet +
    // c_curve[2]*std::pow(current_state.s_frenet,2) +
    // c_curve[3]*std::pow(current_state.s_frenet,3))));  w2 =
    // std::min(0.6, 10.0/(vs[0]+0.01));  w1 = -w2;
    dw2 = 0.2; // 1.0;//0.02;
    dw1 = -dw2;
  }

  flagNBO = 0;
  for (int i = 0; i <= kN; ++i) {
    omega_max[i] =
        std::min(10.0 / (vs[i] + 0.01),
                 0.2 + std::abs(vs[i] * (c_curve[0] + c_curve[1] * sr[i] +
                                         c_curve[2] * std::pow(sr[i], 2) +
                                         c_curve[3] * std::pow(sr[i], 3))));
    omega_min[i] = -omega_max[i];
    omega_rate_min[i] = dw1;
    omega_rate_max[i] = dw2;
    rr[i] = (r_limit[i][0] + r_limit[i][1]) / 2.0; // smooth
    if (rr[i] > 0.05 || rr[i] < -0.05)
      flagNBO = 1;
    r_lower[i] = r_limit[i][0];
    r_upper[i] = r_limit[i][1];
  }

  if (flagTurn == 1) {
    if (flagNBO == 1) {
      mpc_traj_planner_init(1.0, 1.0, 10.0, 1.0);
      cout << "flagTurn & NBO" << endl;
    } else {
      mpc_traj_planner_init(10.0, 10.0, 1.0, 1.0);
      cout << "flagTurn" << endl;
    }
  } else {
    if (flagNBO == 1) {
      mpc_traj_planner_init(1.0, 1.0, 50.0, 1.0);
      cout << "flagCruise & NBO" << endl;
    } else {
      mpc_traj_planner_init(10.0, 10.0, 1000.0, 1.0);
      cout << "flagCruise" << endl;
    }
  }

  int nwsr = run_traj_planner_mpc(&current_state, &current_log, &mpc_output,
                                  c_curve, vs, r_lower, r_upper, rr, omega_min,
                                  omega_max, omega_rate_min, omega_rate_max);
  cout << "nwsr = " << nwsr << endl;
  // for(int i=0;i<=kN;++i)
  //{
  //    printf("%f,%f,%f,%f,%f\n", current_log.s_frenet[i],
  //    current_log.r_frenet[i], current_log.theta_error[i], current_log.vel[i],
  //    current_log.acc[i]);
  //
  //}
  return mpc_output;
}

int main() {
  const int step = 150;
  static double omega_rate = 0.0;

  void *handle = dlopen("liblane_keeping_lateralmpc.so", RTLD_LOCAL | RTLD_NOW);
  if (handle == NULL) {
    cout << dlerror() << endl;
    return false;
  }
  mpc_traj_planner_init =
      (Mpc_traj_planner_init)dlsym(handle, "mpc_traj_planner_init");
  run_traj_planner_mpc =
      (Run_traj_planner_mpc)dlsym(handle, "run_traj_planner_mpc");

  int i, /*j,*/ num_sim /*, flagNBO, flagTurn*/;
  mpc_traj_planner_init(10.0, 10.0, 50.0, 1.0); // 10,1,1,2,10,10

  state_t current_state;
  log_t current_log;
  control_out_t mpc_output_test;
  double c_curve_line[4] = {0.0, 0.0, 0.0, 0.0};
  double vs[kN + 1], sr[kN + 1];
  // double sr0 = 0.0;
  double vr0 = 4.0;
  double ar = 0.5;
  double sl0 = 50.0;
  double vl0 = 4.0;
  double al = 0.5;
  //  double r_lower[kN + 1], r_upper[kN + 1];
  double vr, sl, vl, s_l[kN + 1], /*v_l[kN + 1],*/ r_limit_test[kN + 1][2];
  double mindis = 1000.0;
  // MatrixXd Obstacle_test(kN + 1, 8);
  // MatrixXd r_limit_test(kN + 1, 2);

  current_state.s_frenet = 0.0;
  current_state.r_frenet = 0.3;
  current_state.theta_error = 0.1;
  current_state.omega = 0.0;
  // current_state.acc = 0;

  if ((logger = fopen("/home/ros/Downloads/logger_prediction.csv", "w+")) ==
      NULL) {
    printf("open file error!\n");
    return 1;
  }
  if ((logger1 = fopen("/home/ros/Downloads/solution.csv", "w+")) == NULL) {
    printf("open file error!\n");
    return 1;
  }

  //  static double omega_rate_last = 0.0;
  clock_t start, end;
  vl = vl0;

  for (num_sim = 0; num_sim < step; ++num_sim) {
    if (vl0 + al * num_sim * TS < 0) {
      sl0 = sl0 - vl0 * vl0 / 2.0 / al;
      vl0 = 0;
      al = 0;
    }
    sl = sl0 + vl0 * num_sim * TS + 0.5 * al * pow(num_sim * TS, 2);
    vl = vl0 + al * num_sim * TS;
    vr = vr0 + ar * num_sim * TS;
    // if(flagStop == 1){
    //    mpc_traj_planner_init(10.0, 1.0, 1.0, 50.0, 1.0 , 1.0, 10.0);
    //    cout<<"flagStop"<<endl;
    //    flagStop = 0;
    //}

    for (i = 0; i <= kN; ++i) {
      if (vl + al * i * TS > 0) {
        s_l[i] = sl + vl * i * TS + 0.5 * al * pow(i * TS, 2);
        //        v_l[i] = vl + al * i * TS;
      } else {
        if (al == 0.0) {
          s_l[i] = sl;
          //          v_l[i] = 0;
        } else {
          s_l[i] = sl - vl * vl / 2 / al;
          //          v_l[i] = 0;
        }
      }
      if (num_sim >= 0 && num_sim < 10) {
        if (i >= 2 && i <= 40) {
          // s_l[i] = current_state.s_frenet + 100;
          // v_l[i] = 15;//2.88;//13.3;
          // s_l[39] = 39.13;
          // v_l[39] = 2.85;
        }
      }
      sr[i] = current_state.s_frenet + vr * i * TS + 0.5 * ar * pow(i * TS, 2);
      vs[i] = vr + ar * i * TS;

      r_limit_test[i][0] = -0.7; // r_lower
      r_limit_test[i][1] = 0.7;  // r_upper
    }

    start = clock();
    mpc_output_test = run_lane_keeping_mpc_planner(
        run_traj_planner_mpc, mpc_traj_planner_init, sr, vs, c_curve_line,
        current_state, r_limit_test);

    end = clock();
    static log_t current_log_last = current_log;
    omega_rate = mpc_output_test.omega_rate[0]; // acadoVariables.u[0];

    for (i = 0; i <= kN; ++i) {
      if (s_l[i] < sl)
        sl = s_l[i];
    }
    fprintf(logger1, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", num_sim * TS,
            current_state.s_frenet, current_state.r_frenet,
            current_state.theta_error, current_state.omega, omega_rate, sr[0],
            sr[1], vs[0], vl, std::fabs(current_state.s_frenet - sl), sl,
            (end - start) / 1000.0);
    if (std::fabs(current_state.s_frenet - sl) < mindis) {
      mindis = std::fabs(current_state.s_frenet - sl);
    }

    // if(current_state.vel == 0 && current_state.acc == 0)
    //{
    //    delta_f_rate = 0.0;
    //    jerk = 0;
    //    mpc_traj_planner_init(10.0, 1.0, 1.0, 50.0, 1.0 , 100.0, 0.0001);
    //    //10,1,1,2,10,10
    //
    //}
    current_state =
        motion_function(current_state, vs[0], omega_rate, c_curve_line);

    for (i = 0; i <= kN; i++) {
      fprintf(logger, "%f,%f,%f,%f\n", current_log.s_frenet[i],
              current_log.r_frenet[i], current_log.theta_error[i],
              current_log.omega[i]);
    }
    //    omega_rate_last = omega_rate;
    current_log_last = current_log;
    cout << "s_frenet " << current_state.s_frenet << "r_frenet "
         << current_state.r_frenet << "theta_error "
         << current_state.theta_error << " omega " << current_state.omega
         << " omega_rate " << omega_rate << endl;
  }
  // cout<<"Run-time: "<< (end-start)/1000.0 <<" ms" << endl;
  cout << "MinDis: " << mindis << " m" << endl;
  int ret;
  if ((ret = fclose(logger)) == -1) {
    printf("close file error!\n");
  }
}
