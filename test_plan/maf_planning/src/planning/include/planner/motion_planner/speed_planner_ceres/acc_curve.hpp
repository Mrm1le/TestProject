#pragma once

#include "common/config_context.h"
#include "linear_interpolation_utility.hpp"
#include "planner/motion_planner/planner_cubic_spline/fast_math.hpp"
#include "speed_planner_types.hpp"

namespace speed_planner {

// NOTE: the following convention is followed for ease of ACC curve design
// dv = ego_v - obj_v;
// ds = obj_s - ego_s;
// ds_offseted = ds - center_to_front;

// prediction with const accel
// TODO: replace with prediction using lookup table (similar to cos_sin_fun)

// template <typename T>
static inline void predict_obj_state_at_t_without_prediction(
    const double &s0, const double &init_v, const double &a0, const double &dt,
    double &s, double &v, double &a) {
  const double DECAY_JERK_MPS3 = 0.2;
  const double v0 = std::max(init_v, 0.0);
  // obj is already stopped
  if (a0 < 0.0 && dt > -v0 / a0) {
    a = (0);
    v = (0);
    s = (s0 + v0 * v0 / 2.0 / std::fabs(a0));
    return;
  }
  // obj is not stopped yet
  // acceling obj's accel is assumed to decay to zero to avoid over-reaction
  // deceling obj is assume to stop with const accel to avoid late braking
  if (a0 > 0.0) {
    // accel ramp down to 0 time
    double const_j_dt = (a0 / DECAY_JERK_MPS3);
    if (dt <= const_j_dt) {
      const double t2 = dt * dt;
      const double t3 = t2 * dt;
      a = a0 - DECAY_JERK_MPS3 * dt;
      v = v0 + a0 * dt - DECAY_JERK_MPS3 * t2 * 0.5;
      s = s0 + v0 * dt + 0.5 * a0 * t2 - 1.0 / 6.0 * DECAY_JERK_MPS3 * t3;
    } else {
      // accel already reached 0
      const double t2 = const_j_dt * const_j_dt;
      const double t3 = t2 * const_j_dt;
      a = (0);
      v = v0 + a0 * const_j_dt - DECAY_JERK_MPS3 * t2 * 0.5;
      //  1st segment is const jerk, 2nd segment is const v
      s = s0 + v0 * const_j_dt + 0.5 * a0 * t2 -
          1.0 / 6.0 * DECAY_JERK_MPS3 * t3 + v * (dt - const_j_dt);
    }
  } else {
    a = (a0);
    v = v0 + a0 * dt;
    s = s0 + v0 * dt + 0.5 * a0 * dt * dt;
  }
}

static inline void predict_obj_state_at_t(const bool &use_prediction,
                                          const path_planner::ObsInfo &obj,
                                          const double &time, double &s,
                                          double &v, double &a) {
  if (use_prediction) {
    path_planner::ObsPrediction object_at_t;
    object_at_t = msquare::planning_math::LinearInterpation::interpolate<
        path_planner::ObsPrediction>(obj.object_with_t, time);
    double end_time = obj.object_with_t.back().first;
    const auto &object_at_end_time = obj.object_with_t.back().second;

    if (time < end_time) {
      s = object_at_t.s;
      v = object_at_t.v_frenet;
      a = object_at_t.a;
    } else {
      v = object_at_end_time.v_frenet;
      s = object_at_end_time.s + (time - end_time) * v;
      a = object_at_end_time.a;
    }
  } else {
    predict_obj_state_at_t_without_prediction(
        obj.polygon_init.s, obj.polygon_init.v_frenet, obj.polygon_init.a, time,
        s, v, a);
  }
}

template <typename T>
T get_dv_safe(const T &ds, const double &center_to_front,
              const VelocityAccelJerkBoundInfo &vaj_bound_info,
              const double &ego_v) {
  using namespace msquare::planning_math;
  double ax_min_limit = LinearInterpation::interpolation(
      ego_v, vaj_bound_info.a_min_spd_pt, vaj_bound_info.a_min_value_pt);
  return fast_math::smooth_sqrt(2.0 * std::fabs(ax_min_limit) *
                                (ds - center_to_front));
}

template <typename T>
T get_follow_dist(const double &stop_offset, const double &desired_headway,
                  const T &obj_v) {
  // TODO: split follow dist into three regions, headway range is [1s-2s]
  // closeby use "offset+hw*v"; far away use "hw*v"; blend in between;
  // TODO: determine stop_offset in behavior planner, default 5m, VRU/truck has
  // larger offset
  using namespace msquare::planning_math;
  // double set_dist_hw10 = LinearInterpation::interpolation(
  //     fast_math::get_val(obj_v), EGO_SPEED_TABLE, HEADWAY_1_0_TABLE);
  // double set_dist_hw36 = LinearInterpation::interpolation(
  //     fast_math::get_val(obj_v), EGO_SPEED_TABLE, HEADWAY_3_6_TABLE);
  return T(stop_offset) + desired_headway * obj_v;
  //  T(LinearInterpation::interpolation(
  //                             desired_headway, HEADWAY_MIN_MAX_TABLE,
  //                             {set_dist_hw10, set_dist_hw36}));
}

template <typename T>
void update_dv_curve(const double &stop_offset, const double &desired_headway,
                     const T &obj_v, const T &ds, const double &center_to_front,
                     const double &deadband_region_ratio, T &ds_follow_offseted,
                     T &dv_curve, T &da_curve, T &deadband_dv,
                     const double &ego_vel) {
  // get desired follow dist
  const T ds_offseted = ds - center_to_front;
  ds_follow_offseted = get_follow_dist(stop_offset, desired_headway, obj_v);

  // acc curve is designed with 3-segment piecewise function
  // seg1 is linear, seg3 is const decel, seg2 is a blending linear function
  // const double &seg1_start_dv = -4.0;
  const double &seg1_start_dv =
      -std::fmin(8.0, std::fmax(fast_math::get_val(obj_v) * 0.5, 4));
  // const double &seg1_end_dv = 0.4;
  const double &seg1_end_dv = -0.1 * seg1_start_dv;
  const double &seg2_end_dv = 1.4;
  // TODO: design seg2 slope 0.3 at 0mps, slope 0.8 at 30mps
  const T &seg2_slope = T(0.5); // TODO: replace with "python cubit_fit(obj_v)"
  const T &seg1_end_s = 1.1 * ds_follow_offseted;
  const T &seg2_end_s = seg1_end_s + (seg2_end_dv - seg1_end_dv) / seg2_slope;

  // compute acc curve in 3-segments
  if (ds_offseted < seg1_end_s) {
    dv_curve = seg1_start_dv - ds_offseted / ds_follow_offseted * seg1_start_dv;
    // a = dvdt = dvds*dsdt = dvds*v
    da_curve = -dv_curve / ds_follow_offseted * seg1_start_dv;
  } else if (ds_offseted < seg2_end_s) {
    dv_curve = seg1_end_dv + (ds_offseted - seg1_end_s) * seg2_slope;
    // a = dvdt = dvds*dsdt = dvds*v
    da_curve = dv_curve * seg2_slope;
  } else {
    // TODO: redesign seg3_decel with "python cubit_fit(obj_v)"
    // 0mps is -0.8mps2, 30mps is -0.4mps2
    T seg3_decel_abs = T(std::fmax(
        0.4, 0.8 - ego_vel * 0.4 /
                       30.0)); // TODO: replace with "python cubit_fit(obj_v)"
    T quadratic_zero_s = seg2_end_s - fast_math::integer_pow_2(seg2_end_dv) /
                                          2.0 / seg3_decel_abs;
    dv_curve = fast_math::smooth_sqrt(2.0 * seg3_decel_abs *
                                      (ds_offseted - quadratic_zero_s));
    da_curve = seg3_decel_abs;
  }

  deadband_dv = dv_curve > 2.0 ? deadband_region_ratio * (dv_curve - 2.0) + 2.0
                               : dv_curve;
}

template <typename T>
void update_acc_curve_cost(
    const T &ego_dv, const T &ego_accel, const T &dv_curve, const T &dv_safe,
    const T &obj_v, const T &obj_ds_offseted, const T &ds_follow_offseted,
    const double &v_max, const T &comfort_accel_limit, const ACCParams &params,
    bool &acc_cost_active, const T &deadband_dv, T &acc_curve_cost,
    T &accel_ceiling_cost, bool need_smooth_brake,
    const SpeedPlannerInput *input,
    std::unordered_map<std::string, double> *sample_debug_map) {
  using namespace fast_math;
  // ACC curve cost design
  // 1) Decel region: ego_dv > dv_curve
  // 2) No accel region: dv_curve => ego_dv >= dv_deadband
  // 3) Accel region: dv_deadband >= ego_dv
  ACCRegionType region_type = ACCRegionType::NONE;
  T acc_accel_limit_scale = T(1.0);
  // CJ? v_max < v_cipv is not the sufficient condition of cruise mode
  const T K_KickStart_spd_mps = T(1.5);
  if (!input->use_eftp && ego_dv + obj_v > K_KickStart_spd_mps &&
      (v_max < deadband_dv + obj_v &&
           (ego_dv * ego_dv / T(2.0) / std::max(obj_ds_offseted, T(0.5)) <
                (ego_dv + obj_v - v_max) / T(8.0) ||
            ego_dv + obj_v < v_max) ||
       ego_dv < dv_curve && ego_dv + obj_v > v_max)) {
    acc_curve_cost += params.acc_accel_region_scale *
                      fast_math::smooth_abs(ego_dv + obj_v - v_max);
    acc_cost_active = false;

  } else {
    acc_cost_active = true;
    if (ego_dv > dv_curve) {
      // decel region
      const T &v_normalizer = dv_safe - dv_curve;
      double smooth_fac = need_smooth_brake ? 0.1 : 1;
      acc_curve_cost = params.acc_decel_region_scale *
                       fast_math::integer_pow_3(ego_dv - dv_curve) /
                       fast_math::integer_pow_2(v_normalizer) * smooth_fac;
      acc_accel_limit_scale = T(params.acc_decel_region_scale);
      region_type = ACCRegionType::DECEL_REGION;
    } else if (ego_dv > deadband_dv && !input->use_eftp) {
      // deadband region
      acc_curve_cost = T(0.0); // CJ? should add some ax and jerk cost?
      acc_accel_limit_scale = T(params.deadband_limit_scale);
      region_type = ACCRegionType::DEADBAND_REGION;
      // Don't lift speed above v_max, override acc_curve_cost
      if (v_max < ego_dv + obj_v) { // CJ? seems not possible to enter, v_max <
                                    // v_ego < v_curve, would enter line134
        // acc_curve_cost = params.acc_v_max_scale * (ego_dv + obj_v - v_max);
      }
    } else if (!input->use_eftp) {
      // accel region
      acc_curve_cost = params.acc_accel_region_scale * (deadband_dv - ego_dv);
      acc_accel_limit_scale =
          T(1.0); // TODO: reduce accel limit when close to deadband
      region_type = ACCRegionType::ACCEL_REGION;

      // Don't lift speed above v_max, override acc_curve_cost
      // CJ? seems not possible to enter
      if (v_max < deadband_dv + obj_v && ego_dv + obj_v > v_max) {
        // acc_curve_cost = params.acc_v_max_scale * (ego_dv + obj_v - v_max);
      }
    }
  }

  // Explicitly limit accel based on ACC region
  // for example, very small accel is allowed in deadband region
  // if (!input->use_eftp &&
  //     ego_accel > comfort_accel_limit * acc_accel_limit_scale &&
  //     acc_cost_active ==
  //         true) { // CJ? ceiling cost is not use in no objs Cruise mode, here
  //                 // Cuise mode should also be excluded
  //   accel_ceiling_cost =
  //       params.accel_ceiling_cost_scale *
  //       (ego_accel - comfort_accel_limit * acc_accel_limit_scale);
  // }

  using namespace fast_math;
  if (sample_debug_map) {
    (*sample_debug_map)["dv_safe"] = get_val(dv_safe);
    (*sample_debug_map)["dv_curve"] = get_val(dv_curve);
    (*sample_debug_map)["deadband_dv"] = get_val(deadband_dv);
    (*sample_debug_map)["region_type"] = static_cast<int>(region_type);
    (*sample_debug_map)["ego_dv"] = get_val(ego_dv);
    (*sample_debug_map)["ds_follow_offseted"] = get_val(ds_follow_offseted);
    (*sample_debug_map)["obj_ds_offseted"] = get_val(obj_ds_offseted);
    (*sample_debug_map)["obj_v"] = get_val(obj_v);
    (*sample_debug_map)["acc_curve_cost"] = get_val(acc_curve_cost);
    (*sample_debug_map)["accel_ceiling_cost"] = get_val(accel_ceiling_cost);
    (*sample_debug_map)["acc_cost_active"] = get_val(acc_cost_active);
    (*sample_debug_map)["v_max"] = get_val(v_max);
    (*sample_debug_map)["comfort_accel_limit"] = get_val(comfort_accel_limit);
  }
}

template <typename T>
void compute_acc_and_speed_cost(
    const CommonTerms<T> &ct, const SpeedPlannerInput *input,
    const double &v_max, const std::string &v_max_reason, const double &v_min,
    const std::string &v_min_reason, bool &acc_cost_active,
    const bool &use_eftp, T &acc_residual,
    std::unordered_map<std::string, double> *sample_debug_map) {

  if (sample_debug_map) {
    (*sample_debug_map)["hold_flag"] =
        fast_math::get_val(input->stop_point_info.hold_flag);
    (*sample_debug_map)["stop_mode"] =
        fast_math::get_val(input->stop_point_info.stop_mode);
    (*sample_debug_map)["has_stop_point"] =
        fast_math::get_val(input->stop_point_info.has_stop_point);
    (*sample_debug_map)["should_start"] =
        fast_math::get_val(input->stop_point_info.should_start);
  }

  // standstill mode, do not compute acc cost
  if (input->stop_point_info.hold_flag ||
      (input->stop_point_info.has_stop_point && ct.v < 0.0)) {
    return;
  }

  const auto &lon_objs = input->lon_obs;
  const auto &params = input->speed_tuning_params.get_acc_params();
  const auto &center_to_front = input->vehicle_param.center_to_front;

  // Speed cost strategy:
  // 1) if safety limit based on max decel is violated, a huge cost is applied
  // 2) if acc curve speed is smaller than v_max, a gentle acc cost is applied
  // 3) if acc curve speed is larger than v_max, we only lift speed to v_max

  // curv limit active in v_ref for eftp
  if (v_max_reason == "curvature_limit" && ct.v > v_max && !use_eftp) {
    acc_residual += 3.6 * fast_math::get_val(ct.v - v_max);
  }
  if (msquare::ConfigurationContext::Instance()
          ->planner_config()
          .longitudinal_motion_planner_config.enable_env_speed_limit) {
    const double K_EnvCostFac = 3.6 * 4;
    if (v_max_reason == "env_limit" && ct.v > v_max && !use_eftp) {
      acc_residual += K_EnvCostFac * fast_math::get_val(ct.v - v_max);
    }
  }
  if (input->cipv_lost_info.prohibit_acc) {
    constexpr double K_CipvLostCostFac = 10000.0;
    if (v_max_reason == "cipv_lost" && ct.v > v_max && !use_eftp) {
      acc_residual += K_CipvLostCostFac * fast_math::get_val(ct.v - v_max);
    }
  }
  const double K_SelectGapCostFac = 3.6 * 4; // default 3.6
  if (v_max_reason == "select_gap_decision" && ct.v > v_max) {
    acc_residual += K_SelectGapCostFac * fast_math::get_val(ct.v - v_max);
  }
  if (v_min_reason == "select_gap_decision" && ct.v < v_min) {
    acc_residual += K_SelectGapCostFac * fast_math::get_val(v_min - ct.v);
  }

  // no lon object, return early
  if (lon_objs.empty()) {
    acc_residual += params.cruise_scale * fast_math::smooth_abs(ct.v - v_max);
    acc_cost_active = false;

    if (sample_debug_map) {
      (*sample_debug_map)["acc_cost_active"] =
          fast_math::get_val(acc_cost_active);
      (*sample_debug_map)["v_max"] = fast_math::get_val(v_max);
    }
    return;
  }

  // velocity is smaller than 0.0, set acc cost to zero
  //  if (ct.v < T(fast_math::MathEpsilon)) {
  //    acc_residual += T(0);
  //    return;
  //  }
  // iterate through all lon objects to get v_safe and v_curve
  // select the most critical lon object to compute acc cost
  T min_safe_v = T(MAX_VALID_SPEED_MPS);
  LonObjData<T> critical_obj_data = {};
  critical_obj_data.dv_curve = T(MAX_VALID_SPEED_MPS);
  for (const auto &obj : lon_objs) {
    // predict object state using specified model
    // model can be const accel or from prediction
    double s, v, a;
    double dist_ratio = 1;
    dist_ratio = std::fmax(std::fmin(dist_ratio, 1.5), 0.5);
    double ax_ratio = 1; // 1.5 - dist_ratio;
    double use_prediciton_vx_thr = 5.0;

    predict_obj_state_at_t(input->use_prediction && obj.polygon_init.v_frenet >
                                                        use_prediciton_vx_thr,
                           obj, fast_math::get_val(ct.t), s, v, a);

    // get v_safe using max decel
    const T ds = std::max(T(s) - ct.s, T(center_to_front));
    const T dv_safe = get_dv_safe(ds, center_to_front, input->vaj_bound_info,
                                  fast_math::get_val(ct.v));
    min_safe_v = std::min(min_safe_v, T(v) + dv_safe);
    // get v_curve from desired acc curve
    T ds_follow_offseted, dv_curve, da_curve, dv_deadband;
    update_dv_curve(obj.polygon_init.stop_offset,
                    obj.polygon_init.desired_headway, T(v), ds, center_to_front,
                    params.deadband_region_ratio, ds_follow_offseted, dv_curve,
                    da_curve, dv_deadband, input->ego_state.v);
    // get most critical object
    // CJ:at hold state, don't change cipv until start off
    if (input->stop_point_info.hold_flag == true &&
        obj.id == input->cipv_info.cipv_id) {
      critical_obj_data.id = obj.id;
      critical_obj_data.ds_follow_offseted = ds_follow_offseted;
      critical_obj_data.dv_safe = dv_safe;
      critical_obj_data.dv_curve = dv_curve;
      critical_obj_data.dv_deadband = dv_deadband;
      critical_obj_data.da_curve = da_curve;
      critical_obj_data.obj_v = T(v);
      critical_obj_data.obj_a = T(a);
      critical_obj_data.obj_ds = ds;
      critical_obj_data.obj_headway = T(obj.polygon_init.desired_headway);
    } else if (dv_curve + v <
               critical_obj_data.dv_curve + critical_obj_data.obj_v) {
      critical_obj_data.id = obj.id;
      critical_obj_data.ds_follow_offseted = ds_follow_offseted;
      critical_obj_data.dv_safe = dv_safe;
      critical_obj_data.dv_curve = dv_curve;
      critical_obj_data.dv_deadband = dv_deadband;
      critical_obj_data.da_curve = da_curve;
      critical_obj_data.obj_v = T(v);
      critical_obj_data.obj_a = T(a);
      critical_obj_data.obj_ds = ds;
      critical_obj_data.obj_headway = T(obj.polygon_init.desired_headway);
    }
  }

  if (sample_debug_map) {
    (*sample_debug_map)["critical_obj_id"] =
        fast_math::get_val(critical_obj_data.id);
    (*sample_debug_map)["min_safe_v"] = fast_math::get_val(min_safe_v);
    (*sample_debug_map)["desired_headway"] =
        fast_math::get_val(critical_obj_data.obj_headway);
  }

  // Huge cost for v_safe violation to avoid collision
  if (ct.v > min_safe_v && !input->cipv_info.need_smooth_brake) {
    acc_residual +=
        params.safe_v_limit_scale * fast_math::integer_pow_3(ct.v - min_safe_v);
  }
  // Compute costs based on ACC curve
  T acc_curve_cost{0}, accel_ceiling_cost{0};

  // todo: for fix compile problems, tmp set to 0.2
  T comfort_accel_limit =
      std::max(T(0.0), MAX_ACC - MAX_ACC_DECREASE_RATE * ct.v);
  update_acc_curve_cost(
      ct.v - critical_obj_data.obj_v, ct.a, critical_obj_data.dv_curve,
      critical_obj_data.dv_safe, critical_obj_data.obj_v,
      critical_obj_data.obj_ds - center_to_front,
      critical_obj_data.ds_follow_offseted, v_max, comfort_accel_limit, params,
      acc_cost_active, critical_obj_data.dv_deadband, acc_curve_cost,
      accel_ceiling_cost, input->cipv_info.need_smooth_brake, input,
      sample_debug_map);
  // Accel ceiling cost to limit accel  //CJ? quad_weight use condition?
  // double dist = std::fmax(fast_math::get_val(critical_obj_data.obj_ds) -
  // center_to_front - 4, 0.5); double vx = fast_math::get_val(ct.v); if
  // (!(input->stop_point_info.has_stop_point == true && critical_obj_data.obj_v
  // < 0.1 && vx*vx/2/dist > 0.5)) {
  acc_residual += params.accel_ceiling_scale * accel_ceiling_cost;
  // Accel curve cost to return to acc curve
  // acc_residual += params.cruise_scale * ct.quad_weight * acc_curve_cost;
  acc_residual += acc_curve_cost;
  // ACC cost is active if v_max is higher than v_curve ????
  // acc_cost_active =
  //     (v_max > critical_obj_data.dv_curve + critical_obj_data.obj_v);
  // }
}

} // namespace speed_planner
