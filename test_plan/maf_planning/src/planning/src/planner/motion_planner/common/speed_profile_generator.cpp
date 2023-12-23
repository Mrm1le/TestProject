#include "planner/motion_planner/common/speed_profile_generator.h"

#include <iostream>

#include "common/config/speed_optimizer_config.h"
#include "common/config_context.h"
#include "planning/common/common.h"

namespace msquare {

SpeedData SpeedProfileGenerator::GenerateFallbackSpeed(double stop_distance,
                                                       double init_v,
                                                       double init_a) {
  MSD_LOG(INFO, "Fallback using piecewise jerk speed!");
  // if already stopped
  if (init_v <= 0.0 && init_a <= 0.0) {
    MSD_LOG(INFO, "Already stopped! Nothing to do in GenerateFallbackSpeed()");
    SpeedData speed_data;
    speed_data.AppendSpeedPoint(0.0, 0.0, 0.0, 0.0, 0.0);
    FillEnoughSpeedPoints(&speed_data);
    return speed_data;
  }

  // return GenerateStopProfile(init_v, init_a);

  // std::array<double, 3> init_s = {0.0, init_v, init_a};
  // std::array<double, 3> end_s = {stop_distance, 0.0, 0.0};

  // TODO(all): dt is too small;
  double delta_t = speed_optimize_config::FLAGS_fallback_time_unit;
  // double total_time = speed_optimize_config::FLAGS_fallback_total_time;
  const size_t num_of_knots = speed_optimize_config::horizon_step + 1;

  state_t init_state{0.0, 0.0, 0.0, init_v, init_a, 0.0};
  OptimalSpeedPlanner piecewise_jerk_problem;
  (void)piecewise_jerk_problem.InitalOptimalPLanner(&init_state);

  // std::array<double, 5>
  // cost_weights{speed_optimize_config::default_s_reference_weight,
  //                                    speed_optimize_config::default_v_reference_weight,
  //                                    speed_optimize_config::default_accel_weight,
  //                                    speed_optimize_config::default_jerk_weight,
  //                                    speed_optimize_config::default_curvature_weight};
  // std::array<double, 5> cost_weights{10.0, 1.0, 1.0, 1.0, 0.0};
  // piecewise_jerk_problem.SetWeightParam(cost_weights);
  const double s_gap = 0.05;
  std::vector<double> s_lower_bounds(num_of_knots, 0.0);
  std::vector<double> s_upper_bounds(num_of_knots, max(stop_distance, s_gap));
  piecewise_jerk_problem.SetSlower(s_lower_bounds);
  piecewise_jerk_problem.SetSupper(s_upper_bounds);

  std::vector<double> a_lower(
      num_of_knots,
      ConfigurationContext::Instance()->get_vehicle_param().max_deceleration);
  std::vector<double> a_upper(
      num_of_knots,
      ConfigurationContext::Instance()->get_vehicle_param().max_acceleration);
  piecewise_jerk_problem.SetAlower(a_lower);
  piecewise_jerk_problem.SetAupper(a_upper);

  std::vector<double> x_ref;
  std::vector<double> dx_ref;
  // std::vector<double> dx_ref(num_of_knots, 0.0);
  std::vector<double> curvatures(num_of_knots, 0.0);

  // constant acc motion
  double acc = 0.5 * (init_v * init_v) / (stop_distance + 0.0000000001);
  double curr_s = 0.0;
  double curr_v = init_v;
  for (size_t i = 0; i < num_of_knots; ++i) {
    x_ref.push_back(curr_s);
    dx_ref.push_back(curr_v);
    curr_v -= acc * delta_t;
    curr_s += curr_v * delta_t + 0.5 * acc * delta_t * delta_t;
    if (curr_s >= stop_distance) {
      curr_s = stop_distance;
    }
    if (curr_v <= 0.0) {
      curr_v = 0.0;
      acc = 0.0;
    }
  }

  // interface ref set (x)
  piecewise_jerk_problem.SetSr(x_ref);

  // interface ref set (dx)
  piecewise_jerk_problem.SetVr(dx_ref);

  // interface curvature set
  piecewise_jerk_problem.SetCurvature(curvatures);

  // Solve the problem
  if (!piecewise_jerk_problem.RunSpeedMpcPlanner(
          init_state, init_state, {5.0, -7.0, 30.0, -1.0, 7.0, -6.0})) {
    MSD_LOG(INFO, "Piecewise jerk fallback speed optimizer failed!");
    double dec = speed_optimize_config::FLAGS_slowdown_profile_deceleration;
    return GenerateStopProfile(init_v, init_a, dec);
  }

  // Extract output
  std::vector<double> s, ds, dds, ddds;
  piecewise_jerk_problem.GetSOut(s);
  piecewise_jerk_problem.GetVelOut(ds);
  piecewise_jerk_problem.GetAccOut(dds);
  piecewise_jerk_problem.GetJerkOut(ddds);

  // for (size_t i = 0; i < num_of_knots; ++i) {
  //   std::cout << "For[" << delta_t * static_cast<double>(i) << "], s = " <<
  //   s[i]
  //          << ", v = " << ds[i] << ", a = " << dds[i] << std::endl;
  // }

  SpeedData speed_data;
  speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  for (size_t i = 1; i < num_of_knots; ++i) {
    // Avoid the very last points when already stopped
    if (s[i] - s[i - 1] <= 0.0 || ds[i] <= 0.0) {
      break;
    }
    speed_data.AppendSpeedPoint(s[i], delta_t * static_cast<double>(i), ds[i],
                                dds[i], (dds[i] - dds[i - 1]) / delta_t);
  }
  FillEnoughSpeedPoints(&speed_data);
  return speed_data;
}

void SpeedProfileGenerator::FillEnoughSpeedPoints(SpeedData *const speed_data) {
  const SpeedPoint &last_point = speed_data->back();
  double fallback_total_time = speed_optimize_config::FLAGS_fallback_total_time;
  double fallback_time_unit = speed_optimize_config::FLAGS_fallback_time_unit;
  if (last_point.t >= fallback_total_time) {
    return;
  }
  for (double t = last_point.t + fallback_time_unit; t < fallback_total_time;
       t += fallback_time_unit) {
    speed_data->AppendSpeedPoint(last_point.s, t, 0.0, 0.0, 0.0);
  }
}

SpeedData SpeedProfileGenerator::GenerateStopProfile(double init_speed,
                                                     double init_acc,
                                                     double acc) {
  MSD_LOG(INFO,
          "Slowing down the car within a constant deceleration with fallback "
          "stopping profile.");
  SpeedData speed_data;

  const double max_t = speed_optimize_config::FLAGS_fallback_total_time;
  const double unit_t = speed_optimize_config::FLAGS_fallback_time_unit;

  double pre_s = 0.0;
  double pre_v = init_speed;

  speed_data.AppendSpeedPoint(0.0, 0.0, init_speed, init_acc, 0.0);
  for (double t = unit_t; t < max_t; t += unit_t) {
    double s = 0.0;
    double v = 0.0;
    s = std::fmax(pre_s,
                  pre_s + 0.5 * (pre_v + (pre_v + unit_t * acc)) * unit_t);
    v = std::fmax(0.0, pre_v + unit_t * acc);
    speed_data.AppendSpeedPoint(s, t, v, (v - pre_v) / unit_t, 0.0);
    pre_s = s;
    pre_v = v;
  }
  FillEnoughSpeedPoints(&speed_data);
  return speed_data;
}

SpeedData
SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(double distance,
                                                         double max_speed) {
  constexpr double kConstDeceleration = -0.8; // (~3sec to fully stop)
  constexpr double kProceedingSpeed = 2.23;   // (5mph proceeding speed)
  const double proceeding_speed = std::fmin(max_speed, kProceedingSpeed);
  const double distance_to_start_deceleration =
      proceeding_speed * proceeding_speed / kConstDeceleration / 2;
  bool is_const_deceleration_mode = distance < distance_to_start_deceleration;

  double a = kConstDeceleration;
  double t = 0.0;
  double s = 0.0;
  double v = proceeding_speed;

  constexpr double kDeltaT = 0.1;

  SpeedData speed_data;
  while (s < distance && v > 0) {
    if (is_const_deceleration_mode) {
      speed_data.AppendSpeedPoint(s, t, v, a, 0.0);
      t += kDeltaT;
      double v_new = std::max(0.0, v + a * t);
      s += kDeltaT * (v + v_new) / 2;
      v = v_new;
    } else {
      speed_data.AppendSpeedPoint(s, t, v, 0.0, 0.0);
      t += kDeltaT;
      s += kDeltaT * v;
      if (distance - s < distance_to_start_deceleration)
        is_const_deceleration_mode = true;
    }
  }

  return speed_data;
}

SpeedData SpeedProfileGenerator::GenerateConstAccSpeed(double max_speed,
                                                       double init_v,
                                                       double acc,
                                                       double t_limit) {
  constexpr double kDeltaT = 0.2;
  double t = 0.0;
  double v = init_v;
  double s = 0.0;
  double dec = -1.0;
  SpeedData speed_data;
  if (init_v > max_speed) {
    double a = dec;
    double dec_time = (init_v - max_speed) / dec;
    while (t < t_limit) {
      speed_data.AppendSpeedPoint(s, t, v, a, 0.0);
      t += kDeltaT;
      double v_new = std::min(max_speed, v + a * t);
      s += kDeltaT * (v + v_new) / 2;
      v = v_new;
      a = v_new > max_speed ? dec : 0.0;
    }
    return speed_data;
  } else {
    double a = dec;
    double acc_time = (max_speed - init_v) / acc;
    while (t < t_limit) {
      speed_data.AppendSpeedPoint(s, t, v, a, 0.0);
      t += kDeltaT;
      double v_new = std::min(max_speed, v + a * t);
      s += kDeltaT * (v + v_new) / 2;
      v = v_new;
      a = v_new > max_speed ? 0.0 : acc;
    }
    return speed_data;
  }
}

} // namespace msquare
