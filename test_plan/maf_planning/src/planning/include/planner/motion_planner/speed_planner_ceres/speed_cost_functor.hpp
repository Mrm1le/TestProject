#pragma once

#include "acc_curve.hpp"
#include "linear_interpolation_utility.hpp"
#include "speed_planner_types.hpp"
#include "speed_spline_wrapper.hpp"

namespace speed_planner {
class SpeedCostFunctor {
public:
  SpeedCostFunctor() = default;

  void update_cost_input(
      const SpeedPlannerInput *input,
      const PlannerCubicSpline<NUM_SPEED_CONTROL_POINTS> *acceleration_spline,
      path_planner::NotebookDebug *nb_debug = nullptr) {
    input_ = input;
    acceleration_spline_ = acceleration_spline;
    nb_debug_ = nb_debug;
  }

  template <typename T>
  void compute_curvature_speed_limit_cost(
      const CommonTerms<T> &ct, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    (void)ct;
    residual += T(0);
  }

  template <typename T>
  void compute_accel_limit_cost(
      const CommonTerms<T> &ct, const bool &need_smooth_brake, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    using namespace msquare::planning_math;
    double ax_max_limit = LinearInterpation::interpolation(
        fast_math::get_val(ct.v), input_->vaj_bound_info.a_max_spd_pt,
        input_->vaj_bound_info.a_max_value_pt);
    double ax_min_limit = LinearInterpation::interpolation(
        fast_math::get_val(ct.v), input_->vaj_bound_info.a_min_spd_pt,
        input_->vaj_bound_info.a_min_value_pt);
    const double K_IntersectionAxMax = 0.5;
    if (!input_->use_eftp && input_->vaj_bound_info.is_ddmap_intersection) {
      ax_max_limit = std::fmin(ax_max_limit, K_IntersectionAxMax);
    }

    const double K_SmoothBrake_AxMin = -3.0;
    if (need_smooth_brake) {
      ax_min_limit = std::fmax(ax_min_limit, K_SmoothBrake_AxMin);
    }

    if (sample_debug_map) {
      (*sample_debug_map)["need_smooth_brake"] =
          fast_math::get_val(need_smooth_brake);
      (*sample_debug_map)["ax_max_limit"] = fast_math::get_val(ax_max_limit);
      (*sample_debug_map)["ax_min_limit"] = fast_math::get_val(ax_min_limit);
    }

    if (ct.a > ax_max_limit) {
      residual +=
          input_->speed_tuning_params.accel_limit_scale * (ct.a - ax_max_limit);
    }

    if (ct.a < ax_min_limit) {
      residual +=
          input_->speed_tuning_params.accel_limit_scale * (ax_min_limit - ct.a);
    }
  }

  template <typename T>
  void compute_accel_cost(
      const CommonTerms<T> &ct, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    if (ct.a > 0.0) {
      residual += input_->speed_tuning_params.accel_scale * ct.a;
    } else if (ct.a > input_->vaj_bound_info.a_min) {
      residual += -input_->speed_tuning_params.accel_scale * ct.a;
    } else {
      residual += -2 * input_->speed_tuning_params.accel_scale * ct.a;
    }
  }

  template <typename T>
  void compute_jerk_limit_cost(
      const CommonTerms<T> &ct, const bool &need_smooth_brake,
      const std::string &v_max_reason, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    using namespace msquare::planning_math;
    double jerk_max_limit = LinearInterpation::interpolation(
        fast_math::get_val(ct.v), input_->vaj_bound_info.j_max_spd_pt,
        input_->vaj_bound_info.j_max_value_pt);
    double jerk_min_limit = LinearInterpation::interpolation(
        fast_math::get_val(ct.v), input_->vaj_bound_info.j_min_spd_pt,
        input_->vaj_bound_info.j_min_value_pt);

    if (!input_->use_eftp) {
      if (input_->cipv_info.is_need_pre_braking &&
          !input_->cipv_info.is_merge_flag) {
        jerk_min_limit = std::fmax(jerk_min_limit, J_MIN_PRE_BRAKING);
      }

      const double K_SmoothBrake_JerkMin = -3.0;
      if (need_smooth_brake && ct.a < 0.0) {
        jerk_min_limit = std::fmax(jerk_min_limit, K_SmoothBrake_JerkMin);
      }

      // const double K_ENV_JerkMin = -1.5;
      // if (v_max_reason == "env_limit") {
      //   jerk_min_limit = std::fmax(jerk_min_limit, K_ENV_JerkMin);
      // }

      if (ct.a > 0.0) {
        jerk_min_limit *= 4;
      } else if (ct.a < 0.0) {
        jerk_max_limit *= 4;
      }
    }

    if (ct.jerk > jerk_max_limit) {
      residual += input_->speed_tuning_params.jerk_limit_scale *
                  (ct.jerk - jerk_max_limit);
    }

    if (ct.jerk < jerk_min_limit) {
      residual += input_->speed_tuning_params.jerk_limit_scale * 100 *
                  (jerk_min_limit - ct.jerk);
    }

    if (sample_debug_map) {
      (*sample_debug_map)["jerk_min_limit"] =
          fast_math::get_val(jerk_min_limit);
      (*sample_debug_map)["jerk_max_limit"] =
          fast_math::get_val(jerk_max_limit);
    }
  }

  template <typename T>
  void compute_jerk_cost(
      const CommonTerms<T> &ct, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    residual +=
        input_->speed_tuning_params.jerk_scale * fast_math::smooth_abs(ct.jerk);
  }

  template <typename T>
  void compute_stopping_point_cost(
      const AccelerationSpline<T> &acceleration_spline,
      const CommonTerms<T> &ct, const CommonTerms<T> &last_quad_point_ct,
      bool &is_stopping_point_ct_computed, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    using namespace msquare::planning_math;
    // const double K_StopScaleMin = 0.1;
    // const double K_StopScaleMax = 1;
    // const double K_StopScaleMinSpd_mps = 5;
    // const double K_StopScaleMaxSpd_mps = 0;
    const auto &stop_point_info = input_->stop_point_info;
    const auto &cipv_info = input_->cipv_info;
    const auto &speed_tuning_params = input_->speed_tuning_params;

    // double cipv_vx = 100;
    // for (auto obj : input_->lon_obs) {
    //   if (obj.id == input_->stop_point_info.stop_id) {
    //     cipv_vx = std::fmax(obj.polygon_init.v_frenet, 0);
    //   }
    // }
    // if (input_->lon_obs.size() > 0) {
    //   cipv_vx = std::fmax(input_->lon_obs[0].polygon_init.v_frenet, 0);
    // }

    if (stop_point_info.has_stop_point &&
        cipv_info.cipv_id == stop_point_info.stop_id) {
      if (!input_->stop_point_info.hold_flag) {
        // double dynamic_stop_scale = 0;
        // double delta_v = std::fmax(0.0, K_StopScaleMinSpd_mps - cipv_vx);
        // dynamic_stop_scale =
        //     (delta_v / (K_StopScaleMinSpd_mps - K_StopScaleMaxSpd_mps) *
        //          (K_StopScaleMax - K_StopScaleMin) +
        //      K_StopScaleMin) *
        //     input_->speed_tuning_params.stop_point_scale;

        if (ct.s > stop_point_info.stop_s && ct.v > 0.0) {
          residual += input_->speed_tuning_params.stop_point_scale * ct.v;
        } else if (ct.s < stop_point_info.stop_s && ct.v < 0.0) {
          residual += -input_->speed_tuning_params.stop_point_scale * ct.v;
        }
        // if (ct.v < T(10.0) && ct.s - stop_point_info.stop_s < 1) {
        //   if (ct.a > 0.0) {
        //     residual += dynamic_stop_scale * ct.a;
        //   }
        // }
        // if (ct.v < stop_point_info.stop_target_accel_entry_velocity &&
        //     ct.s - stop_point_info.stop_s > 0.0) {
        //   residual +=
        //       input_->speed_tuning_params.stop_point_scale *
        //       fast_math::smooth_abs(ct.a +
        //       stop_point_info.stop_target_accel);
        // }

        if (ct.v < stop_point_info.stop_target_accel_entry_velocity) {
          double stop_target_accel = stop_point_info.stop_target_accel;
          if (cipv_info.cipv_id != -1 && cipv_info.ds_offseted < 2) {
            stop_target_accel = 3.0;
          }
          residual += input_->speed_tuning_params.stop_point_scale *
                      fast_math::smooth_abs(ct.a + stop_target_accel);
        }

        if (stop_point_info.ref_s_list.size() && !input_->enable_model_ref) {
          const double ref_v = LinearInterpation::interpolation(
              fast_math::get_val(ct.s), stop_point_info.ref_s_list,
              stop_point_info.ref_v_list);
          if (ct.s < stop_point_info.ref_s_list.back()) {
            if (ct.v > ref_v) {
              residual += input_->speed_tuning_params.stop_point_scale *
                          fast_math::smooth_abs(ct.v - ref_v);
            }

            if (ct.v < ref_v && stop_point_info.from_closest_obs) {
              residual += input_->speed_tuning_params.stop_point_scale *
                          fast_math::smooth_abs(ct.v - ref_v);
            }

            if (sample_debug_map) {
              (*sample_debug_map)["ref_v"] = fast_math::get_val(ref_v);
            }
          }
        }

        /*
        if (!is_stopping_point_ct_computed && ct.s > stop_point_info.stop_s &&
            input_->planning_init_state.s < stop_point_info.stop_s) {
          // stopping_point_ct is computed, set this flag to true
          is_stopping_point_ct_computed = true;
          // CJ: New Stop Cost method:
          SamplePointCoeffInfo sample_point_coeff_info_l;
          SamplePointCoeffInfo sample_point_coeff_info_r;
          sample_point_coeff_info_l.sample_s = stop_point_info.stop_s - 1.0;
          acceleration_spline.compute_sample_point_info(
              sample_point_coeff_info_l);
          sample_point_coeff_info_r.sample_s = stop_point_info.stop_s + 1.0;
          acceleration_spline.compute_sample_point_info(
              sample_point_coeff_info_r);
          CommonTerms<T> stop_point_ct_l = compute_common_terms(
              acceleration_spline, last_quad_point_ct, input_,
              &sample_point_coeff_info_l, "at_fix_sample_s", sample_debug_map);
          CommonTerms<T> stop_point_ct_r = compute_common_terms(
              acceleration_spline, last_quad_point_ct, input_,
              &sample_point_coeff_info_r, "at_fix_sample_s", sample_debug_map);
          if (stop_point_ct_l.v < 0.0) {
            // residual += input_->speed_tuning_params.stop_point_scale * 1 *
            //             (-stop_point_ct_l.v);
          }
          if (stop_point_ct_r.v > 0.0) {
            // residual += input_->speed_tuning_params.stop_point_scale * 2 *
            //             stop_point_ct_r.v;
          }

          SamplePointCoeffInfo sample_point_coeff_info;
          sample_point_coeff_info.sample_s = stop_point_info.stop_s;
          acceleration_spline.compute_sample_point_info(
              sample_point_coeff_info);

          // compute stop_point_ct only one time, use last_quad_point_ct
          // carefully!
          CommonTerms<T> stop_point_ct = compute_common_terms(
              acceleration_spline, last_quad_point_ct, input_,
              &sample_point_coeff_info, "at_fix_sample_s", sample_debug_map);
          if (fast_math::smooth_abs(stop_point_ct.v) < 0.1) {
            // residual += input_->speed_tuning_params.stop_point_scale *
            //             fast_math::smooth_abs(stop_point_ct.v);
          } else {
            // residual += input_->speed_tuning_params.stop_point_scale * 0.1 +
            //             100 * input_->speed_tuning_params.stop_point_scale *
            //                 fast_math::smooth_abs(
            //                     fast_math::smooth_abs(stop_point_ct.v) -
            //                     0.1);
          }

          if (sample_debug_map) {
            (*sample_debug_map)["stop_point_ct_s"] =
                fast_math::get_val(stop_point_ct.s);
          }
        }
        */
      } else if (ct.v > 0.0) {
        residual += input_->speed_tuning_params.stop_point_scale * ct.v;
      }
    } else {
      if (cipv_info.cipv_id != -1) {
        // has cipv
        if (cipv_info.dv_curve + cipv_info.vel > 0.0) {
          const double K_StopMinSpd_mps = 1.0;
          if (ct.v <= K_StopMinSpd_mps) {
            residual += input_->speed_tuning_params.stop_point_scale *
                        (K_StopMinSpd_mps - ct.v);
          }
        }
      } else {
        // without cipv
        if (ct.v <= 0.0) {
          residual += -input_->speed_tuning_params.stop_point_scale * ct.v;
        }
      }
    }

    if (sample_debug_map) {
      (*sample_debug_map)["has_stop_point"] = stop_point_info.has_stop_point;
      (*sample_debug_map)["stop_s"] = stop_point_info.stop_s;
      (*sample_debug_map)["is_replan"] = input_->is_replan;
    }
  }

  template <typename T>
  void compute_model_reference_speed_cost(
      const CommonTerms<T> &ct, const double &v_ref,
      const std::string &v_ref_reason, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    if (input_->model_trajectory.size() && input_->enable_model_ref) {
      if (ct.s < input_->model_trajectory.back().s) {
        double model_ref_scale = input_->speed_tuning_params.model_ref_scale;
        // change to curv limit scale
        if (v_ref_reason == "curvature_limit") {
          model_ref_scale = 3.6;
        }
        residual += model_ref_scale * fast_math::smooth_abs(ct.v - v_ref);
      } else {
        if (ct.v > v_ref && input_->b_dagger_longitudinal) {
          residual +=
              input_->speed_tuning_params.model_ref_scale * (ct.v - v_ref);
        }
      }
    }

    if (sample_debug_map) {
      (*sample_debug_map)["model_ref_v"] = v_ref;
    }
  }

  template <typename T>
  void compute_gmp_reference_speed_cost(
      const CommonTerms<T> &ct, const double &v_ref,
      const std::string &v_ref_reason, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    if (input_->gmp_valid == true) {
      double gmp_ref_scale = input_->speed_tuning_params.gmp_ref_scale;
      residual += gmp_ref_scale * fast_math::smooth_abs(ct.v - v_ref);
    }

    if (sample_debug_map) {
      (*sample_debug_map)["gmp_ref_v"] = v_ref;
    }
  }

  template <typename T>
  void compute_safety_cost(
      const CommonTerms<T> &ct, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    for (const auto &overlap_data : input_->overlap_info_map) {
      if (ct.s > overlap_data.second.s && ct.v > overlap_data.second.v) {
        residual += input_->speed_tuning_params.model_safety_scale *
                    fast_math::integer_pow_3(ct.v - overlap_data.second.v);
        if (sample_debug_map) {
          (*sample_debug_map)["safety_s"] = overlap_data.second.s;
          (*sample_debug_map)["safety_v"] = overlap_data.second.v;
          (*sample_debug_map)["safety_id"] = overlap_data.second.id;
          (*sample_debug_map)["safety_time"] = overlap_data.second.time;
        }
      }
    }
  }

  template <typename T>
  static CommonTerms<T> compute_common_terms(
      const AccelerationSpline<T> &acceleration_spline,
      const CommonTerms<T> &last_quad_point_ct, const SpeedPlannerInput *input,
      const void *input_data_ptr,
      const std::string &input_data_type = "at_quard_point",
      std::unordered_map<std::string, double> *sample_debug_map = nullptr) {
    // get Cartesian frame coordinates
    CommonTerms<T> ct;

    if (input_data_type == "at_quard_point") {
      // sample info of current index
      auto index = (Index<NUM_SPEED_CONTROL_POINTS> *)input_data_ptr;
      ct.s = acceleration_spline.get_s_at_index(*index);
      ct.a = acceleration_spline.get_acceleration_at_index(*index);
    } else if (input_data_type == "at_fix_sample_s") {
      auto sample_point_coeff_info = (SamplePointCoeffInfo *)input_data_ptr;
      ct.s = sample_point_coeff_info->sample_s;
      ct.a = acceleration_spline.get_acceleration_at_sample_s(
          *sample_point_coeff_info);
    }

    auto delta_s = ct.s - last_quad_point_ct.s;

    // last v is larger than 0.0
    if (last_quad_point_ct.v >= 0.0) {
      auto v_square_candidate =
          fast_math::integer_pow_2(last_quad_point_ct.v) +
          2.0 * ct.a * delta_s; // parasoft-suppress AUTOSAR-A8_5_0 "f-drop"
      if (v_square_candidate >= 0.0) {
        // still larger than 0.0
        ct.v = fast_math::math_sqrt(v_square_candidate);
      } else {
        // cross through zero
        T s_middle =
            last_quad_point_ct.s -
            fast_math::integer_pow_2(last_quad_point_ct.v) / (2.0 * ct.a);
        ct.v = -fast_math::math_sqrt(2.0 * ct.a * (s_middle - ct.s));
      }
    } else {
      auto v_square_candidate =
          fast_math::integer_pow_2(last_quad_point_ct.v) - 2.0 * ct.a * delta_s;
      // last v is smaller than 0.0
      if (v_square_candidate <= 0.0) {
        // cross through zero
        T s_middle =
            last_quad_point_ct.s +
            fast_math::integer_pow_2(last_quad_point_ct.v) / (2.0 * ct.a);
        ct.v = fast_math::math_sqrt(2.0 * ct.a * (ct.s - s_middle));
      } else {
        // still smaller than 0.0
        ct.v = -fast_math::math_sqrt(v_square_candidate);
      }
    }

    if (fast_math::smooth_abs(ct.a) > T(fast_math::MathEpsilon)) {
      ct.t = (ct.v - last_quad_point_ct.v) / ct.a + last_quad_point_ct.t;
    } else {
      ct.t = delta_s / std::max(ct.v, T(fast_math::MathEpsilon)) +
             last_quad_point_ct.t;
    }

    if (input_data_type == "at_quard_point") {
      auto index = (Index<NUM_SPEED_CONTROL_POINTS> *)input_data_ptr;
      T da_ds = acceleration_spline.get_derivative_at_index(*index);
      ct.jerk = da_ds * ct.v;
      ct.quad_weight = GAUSS_QUAD_5TH_WT[index->quad_index];
    } else if (input_data_type == "at_fix_sample_s") {
      auto sample_point_coeff_info = (SamplePointCoeffInfo *)input_data_ptr;
      T da_ds = acceleration_spline.get_derivative_at_sample_s(
          *sample_point_coeff_info);
      ct.jerk = da_ds * ct.v;
      ct.quad_weight = 0.0;
    }

    if (sample_debug_map) {
      (*sample_debug_map)["s"] = ct.s;
      (*sample_debug_map)["v"] = fast_math::get_val(ct.v);
      (*sample_debug_map)["a"] = fast_math::get_val(ct.a);
      (*sample_debug_map)["jerk"] = fast_math::get_val(ct.jerk);
      (*sample_debug_map)["t"] = fast_math::get_val(ct.t);
      (*sample_debug_map)["delta_s"] = fast_math::get_val(delta_s);
    }

    return ct;
  }

  // static double get_val(const double &val) { return val; }

  // static double get_val(const ceres::Jet<double, TOTAL_NUM_PARAMS> &val) {
  //   return val.a;
  // }

  template <typename T>
  bool operator()(const T *const params, T *residuals) const {
    // initialized all residuals to zero
    std::fill(residuals, residuals + TOTAL_NUM_RESIDUALS, T(0));

    const auto &planning_init_state = input_->planning_init_state;
    AccelerationSpline<T> acceleration_spline(
        acceleration_spline_, planning_init_state.a, planning_init_state.da_ds,
        params);

    CommonTerms<T> last_quad_point_ct;
    last_quad_point_ct.s = planning_init_state.s;
    last_quad_point_ct.t = T(0.0);
    last_quad_point_ct.v = T(planning_init_state.v);
    last_quad_point_ct.a = T(planning_init_state.a);
    last_quad_point_ct.jerk = T(planning_init_state.jerk);

    // stopping_point_ct is not computed, set this flag to false
    bool is_stopping_point_ct_computed = false;
    // iterate through all sampled quadrature points
    Index<NUM_SPEED_CONTROL_POINTS> index;
    while (index.advance()) {
      // sample info of current index
      const auto &quad_point_info =
          input_->speed_segments[index.segment_index][index.quad_index]
              .quad_point_info;

      // notebook debug info, which exposes as much data as you want for offline
      // debugging, but not used in car by assigning nullptr to it.
      std::unordered_map<std::string, double> *sample_debug_map = nullptr;
      if (nb_debug_ != nullptr) {
        const double sample_s = acceleration_spline.get_s_at_index(index);
        nb_debug_->s_to_samples[sample_s] = {};
        sample_debug_map = &nb_debug_->s_to_samples[sample_s].sample_data;
      }

      // compute common term at current quad point
      CommonTerms<T> ct =
          compute_common_terms(acceleration_spline, last_quad_point_ct, input_,
                               &index, "at_quard_point", sample_debug_map);

      // compute path planner residuals, note that cost = 0.5 * residual *
      // residual, compute curvature speed limit cost
      compute_curvature_speed_limit_cost(
          ct,
          residuals[static_cast<size_t>(
              SpeedPlannerResiduals::CURVATURE_SPEED_LIMIT)],
          sample_debug_map);

      // compute acc cost
      bool acc_cost_active = false;
      if (input_->use_eftp && !input_->b_dagger_longitudinal &&
              input_->cipv_info.is_stable ||
          !input_->use_eftp) {
        compute_acc_and_speed_cost(
            ct, input_, quad_point_info.v_max, quad_point_info.v_max_reason,
            quad_point_info.v_min, quad_point_info.v_min_reason,
            acc_cost_active, input_->use_eftp,
            residuals[static_cast<size_t>(SpeedPlannerResiduals::ACC)],
            sample_debug_map);
      } else {
        // set acc cost active at eftp mode
        acc_cost_active = true;
      }

      // comput acc limit cost
      compute_accel_limit_cost(
          ct, input_->cipv_info.need_smooth_brake,
          residuals[static_cast<size_t>(SpeedPlannerResiduals::ACCEL_LIMIT)],
          sample_debug_map);

      // compute jerk limit cost
      compute_jerk_limit_cost(
          ct, input_->cipv_info.need_smooth_brake, quad_point_info.v_max_reason,
          residuals[static_cast<size_t>(SpeedPlannerResiduals::JERK_LIMIT)],
          sample_debug_map);

      if (input_->use_eftp) {
        compute_model_reference_speed_cost(
            ct, quad_point_info.v_ref, quad_point_info.v_ref_reason,
            residuals[static_cast<size_t>(
                SpeedPlannerResiduals::MODEL_REFERENCE_SPEED)],
            sample_debug_map);

        if (!input_->b_dagger_longitudinal) {
          compute_safety_cost(
              ct, residuals[static_cast<size_t>(SpeedPlannerResiduals::SAFETY)],
              sample_debug_map);
        }
      }

      compute_gmp_reference_speed_cost(
          ct, quad_point_info.v_ref, quad_point_info.v_ref_reason,
          residuals[static_cast<size_t>(
              SpeedPlannerResiduals::GMP_REFERENCE_SPEED)],
          sample_debug_map);

      if (!input_->b_dagger_longitudinal) {
        // compute acc cost
        compute_accel_cost(
            ct, residuals[static_cast<size_t>(SpeedPlannerResiduals::ACCEL)],
            sample_debug_map);

        // compute jerk cost
        compute_jerk_cost(
            ct, residuals[static_cast<size_t>(SpeedPlannerResiduals::JERK)],
            sample_debug_map);

        // comput stop point cost
        compute_stopping_point_cost(
            acceleration_spline, ct, last_quad_point_ct,
            is_stopping_point_ct_computed,
            residuals[static_cast<size_t>(SpeedPlannerResiduals::STOP_POINT)],
            sample_debug_map);
      }

      if (sample_debug_map) {
        (*sample_debug_map)["cipv_id"] =
            fast_math::get_val(input_->cipv_info.cipv_id);
        (*sample_debug_map)["cipv_time"] =
            fast_math::get_val(input_->cipv_info.cipv_time);
        (*sample_debug_map)["use_eftp"] = fast_math::get_val(input_->use_eftp);
      }

      // update last quad point common terms
      last_quad_point_ct = ct;
    }

    return true;
  }

private:
  const PlannerCubicSpline<NUM_SPEED_CONTROL_POINTS> *acceleration_spline_ =
      nullptr;
  const SpeedPlannerInput *input_;
  path_planner::NotebookDebug *nb_debug_;
};

} // namespace speed_planner
