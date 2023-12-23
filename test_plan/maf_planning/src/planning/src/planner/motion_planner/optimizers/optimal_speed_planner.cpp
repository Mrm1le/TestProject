#include "planner/motion_planner/optimizers/optimal_speed_planner.h"

#include <iostream>

#include "planning/common/common.h"

namespace msquare {
OptimalSpeedPlanner::OptimalSpeedPlanner() {
  cost_weight_ = {speed_optimize_config::default_s_reference_weight,
                  speed_optimize_config::default_v_reference_weight,
                  speed_optimize_config::default_accel_weight,
                  speed_optimize_config::default_jerk_weight,
                  speed_optimize_config::default_curvature_weight};

  vehicle_real_state_.s_frenet = 0.0;
  vehicle_real_state_.r_frenet = 0.0;
  vehicle_real_state_.theta_error = 0.0;
  vehicle_real_state_.acc = 0.0;
  vehicle_real_state_.omega = 0.0;
  vehicle_real_state_.vel = 0.0;

  vehicle_state_.s_frenet = 0.0;
  vehicle_state_.r_frenet = 0.0;
  vehicle_state_.theta_error = 0.0;
  vehicle_state_.acc = 0.0;
  vehicle_state_.omega = 0.0;
  vehicle_state_.vel = 0.0;

  for (int i = 0; i < speed_optimize_config::horizon_step + 1; ++i) {
    speed_planner_mpc_log_.s_frenet[i] = 0.0;
    speed_planner_mpc_log_.r_frenet[i] = 0.0;
    speed_planner_mpc_log_.theta_error[i] = 0.0;
    speed_planner_mpc_log_.acc[i] = 0.0;
    speed_planner_mpc_log_.omega[i] = 0.0;
    speed_planner_mpc_log_.vel[i] = 0.0;

    if (i < speed_optimize_config::horizon_step) {
      speed_planner_control_out_.jerk[i] = 0.0;
      speed_planner_control_out_.omega_rate[i] = 0.0;
    }
  }
}

bool OptimalSpeedPlanner::InitalOptimalPLanner(state_t *vehicle_state) {
  if (handle_longitudinal_ == nullptr) {
    handle_longitudinal_ =
        dlopen("libmotion_planner_for_dpmpc.so", RTLD_LOCAL | RTLD_NOW);
  }

  if (handle_longitudinal_ == NULL) {
    cout << dlerror() << endl;
    return false;
  }

  speed_mpc_planner_init_ = (Speed_mpc_planner_init)dlsym(
      handle_longitudinal_, "mpc_speed_planner_init");
  run_speed_mpc_planner_ = (Run_speed_mpc_planner)dlsym(
      handle_longitudinal_, "run_speed_planner_mpc");

  MSD_LOG(INFO, "vehicle_state: a %f v %f s %f \n", vehicle_state->acc,
          vehicle_state->vel, vehicle_state->s_frenet);

  speed_mpc_planner_init_(cost_weight_[0], cost_weight_[1], cost_weight_[2],
                          cost_weight_[3], cost_weight_[4], vehicle_state);

  return true;
}

void OptimalSpeedPlanner::SetWeightParam(
    const std::array<double, 5> &cost_weight) {
  cost_weight_ = cost_weight;
  (void)InitalOptimalPLanner(
      &vehicle_state_); // to do use which vehicle state to inital mpc planner
}

void OptimalSpeedPlanner::SetSr(const vector<double> sr) {
  mph_assert(sr.size() == speed_optimize_config::horizon_step + 1);
  sr_ = sr;
}

void OptimalSpeedPlanner::SetVr(const vector<double> vr) {
  mph_assert(vr.size() == speed_optimize_config::horizon_step + 1);
  vr_ = vr;
}

void OptimalSpeedPlanner::SetSupper(const vector<double> s_upper) {
  mph_assert(s_upper.size() == speed_optimize_config::horizon_step + 1);
  s_upper_ = s_upper;
}

void OptimalSpeedPlanner::SetSlower(const vector<double> s_lower) {
  mph_assert(s_lower.size() == speed_optimize_config::horizon_step + 1);
  s_lower_ = s_lower;
}

void OptimalSpeedPlanner::SetCurvature(const vector<double> curvature) {
  mph_assert(curvature.size() == speed_optimize_config::horizon_step + 1);
  curvature_ = curvature;
}

void OptimalSpeedPlanner::SetAupper(const vector<double> a_upper) {
  mph_assert(a_upper.size() == speed_optimize_config::horizon_step + 1);
  a_upper_ = a_upper;
}

void OptimalSpeedPlanner::SetAlower(const vector<double> a_lower) {
  mph_assert(a_lower.size() == speed_optimize_config::horizon_step + 1);
  a_lower_ = a_lower;
}

bool OptimalSpeedPlanner::GetOnlineData(
    speed_mpc_online_data &mpc_online_data) {
  if (!CheckOnlineData()) {
    MSD_LOG(ERROR, "online data nums step is not equal to (horizon_step + 1), "
                   "check your set online_data, maybe you ignore set "
                   "online_data ");
    return false;
  }

  for (size_t i = 0; i < speed_optimize_config::horizon_step + 1; ++i) {
    mpc_online_data.sr[i] = sr_.at(i);
    mpc_online_data.vr[i] = vr_.at(i);
    mpc_online_data.s_upper[i] = s_upper_.at(i);
    mpc_online_data.s_lower[i] = s_lower_.at(i);
    mpc_online_data.curvature[i] = curvature_.at(i);
    mpc_online_data.a_upper[i] = a_upper_.at(i);
    mpc_online_data.a_lower[i] = a_lower_.at(i);
    // std::cout << "online data[" << i << "] sr: " << mpc_online_data.sr[i]
    // << " vr: " << mpc_online_data.vr[i]
    // << " s_upper: " << mpc_online_data.s_upper[i]
    // << " s_lower: " << mpc_online_data.s_lower[i]
    // << " a_upper: " << mpc_online_data.a_upper[i]
    // << " a_lower: " << mpc_online_data.a_lower[i] << std::endl;
  }
  return true;
}

bool OptimalSpeedPlanner::CheckOnlineData() {
  const int data_num = speed_optimize_config::horizon_step + 1;
  if (sr_.size() != data_num || vr_.size() != data_num ||
      s_upper_.size() != data_num || s_lower_.size() != data_num ||
      a_upper_.size() != data_num || a_lower_.size() != data_num ||
      curvature_.size() != data_num) {
    return false;
  }
  return true;
}

bool OptimalSpeedPlanner::RunSpeedMpcPlanner(
    const state_t &vehicle_real_state, const state_t &vehicle_state,
    std::array<double, 6> mpc_checker) {
  vehicle_real_state_ = vehicle_real_state;
  vehicle_state_ = vehicle_state;

  speed_mpc_online_data speed_planner_online_data;
  if (!GetOnlineData(speed_planner_online_data)) {
    MSD_LOG(ERROR, "online data size is not equal : %d",
            speed_optimize_config::horizon_step + 1);
    return false;
  }

  ClearOutData();

  if (run_speed_mpc_planner_(
          &vehicle_state_, &speed_planner_mpc_log_, &speed_planner_control_out_,
          speed_planner_online_data.sr, speed_planner_online_data.vr,
          speed_planner_online_data.s_upper, speed_planner_online_data.s_lower,
          speed_planner_online_data.curvature,
          speed_planner_online_data.a_upper, speed_planner_online_data.a_lower,
          mpc_checker[0], mpc_checker[1], mpc_checker[2], mpc_checker[3],
          mpc_checker[4], mpc_checker[5])) {
    for (size_t i = 0; i < speed_optimize_config::horizon_step + 1; ++i) {
      s_frenet_out_.push_back(speed_planner_mpc_log_.s_frenet[i]);
      vel_out_.push_back(speed_planner_mpc_log_.vel[i]);
      acc_out_.push_back(speed_planner_mpc_log_.acc[i]);
      jerk_control_.push_back(speed_planner_control_out_.jerk[i]);
    }

    return true;
  }

  MSD_LOG(WARN, "speed mpc planner calculate error");
  return false;
}

void OptimalSpeedPlanner::ClearOutData() {
  s_frenet_out_.clear();
  vel_out_.clear();
  acc_out_.clear();
  jerk_control_.clear();
}

void OptimalSpeedPlanner::GetSOut(vector<double> &s_out) {
  s_out = s_frenet_out_;
}

void OptimalSpeedPlanner::GetVelOut(vector<double> &vel_out) {
  vel_out = vel_out_;
}

void OptimalSpeedPlanner::GetAccOut(vector<double> &acc_out) {
  acc_out = acc_out_;
}

void OptimalSpeedPlanner::GetJerkOut(vector<double> &jerk_out) {
  jerk_out = jerk_control_;
}

} // namespace msquare
