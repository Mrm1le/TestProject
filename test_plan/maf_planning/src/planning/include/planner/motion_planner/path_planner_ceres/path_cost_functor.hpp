#pragma once

#include "linear_interpolation_utility.hpp"
#include "path_planner_types.hpp"
#include "path_spline_wrapper.hpp"
#include "planner/motion_planner/planner_cubic_spline/fast_math.hpp"
namespace path_planner {
class PathCostFunctor {
public:
  PathCostFunctor() = default;

  void update_cost_input(
      const PathPlannerInput *input,
      const PlannerCubicSpline<NUM_PATH_CONTROL_POINTS> *path_offset_spline,
      const DecisionInfo *decision_info, NotebookDebug *nb_debug = nullptr) {
    input_ = input;
    path_offset_spline_ = path_offset_spline;
    decision_info_ = decision_info;
    nb_debug_ = nb_debug;
  }

  template <typename T>
  void compute_soft_boundary_cost(
      const CommonTerms<T> &ct, const double &left_limit,
      const double &right_limit, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    // left lane line cost
    if (ct.offset > left_limit) {
      residual += input_->path_tuning_params.soft_boundary_scale *
                  fast_math::integer_pow_2(ct.offset - left_limit);
    }

    // right lane line cost
    if (ct.offset < right_limit) {
      residual += input_->path_tuning_params.soft_boundary_scale *
                  fast_math::integer_pow_2(ct.offset - right_limit);
    }
  }

  template <typename T>
  void compute_hard_boundary_cost(
      const CommonTerms<T> &ct, const double &left_hard_bound,
      const double &right_hard_bound, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    if (ct.offset > left_hard_bound) {
      residual += input_->path_tuning_params.hard_boundary_scale *
                  fast_math::integer_pow_2(ct.offset - left_hard_bound);
    }

    if (ct.offset < right_hard_bound) {
      residual += input_->path_tuning_params.hard_boundary_scale *
                  fast_math::integer_pow_2(ct.offset - right_hard_bound);
    }
  }

  template <typename T>
  void compute_init_curvature_cost(const T init_curvature, T &residual) const {
    const double &ego_init_curvature = input_->planning_init_state.curvature;
    const double &v = input_->planning_init_state.v;

    T init_curvature_cost = T(0.);
    if ((input_->lc_decider_info.is_lane_change &&
         input_->lc_decider_info.lc_status_time < 1.0) ||
        (!input_->lc_decider_info.is_lane_change &&
         input_->lc_decider_info.lc_status_time > 0.)) {
      // HACK :
      T curv_error = fast_math::smooth_abs(init_curvature - ego_init_curvature);
      init_curvature_cost = input_->path_tuning_params.init_curvature_scale *
                            INIT_NORMAL_CURVATURE_RATIO * curv_error *
                            fast_math::integer_pow_2(std::max(v, 1.));

      T curv_limit_error =
          curv_error - std::abs(ego_init_curvature) * INIT_CURV_ERROR_LIMIT;
      if (curv_limit_error > 0.) {
        init_curvature_cost += input_->path_tuning_params.init_curvature_scale *
                               INIT_CURV_LIMIT_COST_SCALE * curv_limit_error *
                               fast_math::integer_pow_2(std::max(v, 1.));
      }
    } else {
      // HACK : fallback for over control
      double init_curvature_scale =
          input_->path_tuning_params.init_curvature_scale /
          INIT_CURVATURE_SCALE;
      init_curvature_cost =
          init_curvature_scale *
          fast_math::smooth_abs(init_curvature - ego_init_curvature);
    }

    residual += init_curvature_cost;
  }

  // template <typename T>
  // void compute_curvature_cost(
  //     const CommonTerms<T> &ct, T &residual,
  //     std::unordered_map<std::string, double> *sample_debug_map) const {
  //   T dc_dt = ct.dc_ds * ct.ds_dt;
  //   residual += fast_math::smooth_abs(dc_dt) *
  //               input_->path_tuning_params.curvature_limit_scale;
  //   if (fast_math::smooth_abs(dc_dt) > CURVATURE_RATE_LIMIT) {
  //     residual += (fast_math::smooth_abs(dc_dt) - CURVATURE_RATE_LIMIT) *
  //                 input_->path_tuning_params.curvature_limit_scale *
  //                 CURVATURE_RATE_LIMIT_SCALE;
  //   }
  // }

  template <typename T>
  void compute_lat_accel_cost(
      const CommonTerms<T> &ct, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    residual += input_->path_tuning_params.lat_accel_scale *
                fast_math::smooth_abs(ct.lat_accel);
  }

  template <typename T>
  void compute_lat_jerk_cost(
      const CommonTerms<T> &ct, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    double scale = input_->path_tuning_params.lat_jerk_scale +
                   input_->path_tuning_params.curvature_limit_scale /
                       std::max(std::pow(fast_math::get_val(ct.ds_dt), 2), 1.0);
    residual += scale * fast_math::smooth_abs(ct.lat_jerk);
    if (abs(ct.lat_jerk) > LAT_JERK_LIMIT_MPS3) {
      residual += LAT_JERK_LIMIT_COST_SCALE *
                  input_->path_tuning_params.lat_jerk_scale *
                  fast_math::smooth_abs(abs(ct.lat_jerk) - LAT_JERK_LIMIT_MPS3);
    }
    T dc_dt = ct.dc_ds * ct.ds_dt;
    if (fast_math::smooth_abs(dc_dt) > CURVATURE_RATE_LIMIT) {
      residual += (fast_math::smooth_abs(dc_dt) - CURVATURE_RATE_LIMIT) *
                  input_->path_tuning_params.curvature_limit_scale *
                  CURVATURE_RATE_LIMIT_SCALE;
    }
  }

  template <typename T>
  void compute_heading_cost(
      const CommonTerms<T> &ct, const double &cosh, const double &sinh,
      T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    residual += input_->path_tuning_params.heading_scale *
                fast_math::integer_pow_2(ct.dx_ds * sinh - ct.dy_ds * cosh);
  }

  template <typename T>
  void compute_ref_centering_cost(
      const CommonTerms<T> &ct, double ref_offset, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    // offset is w.r.t. lane center, so minimize offset is centering to lane
    // center closeby stay within lane; far away plan has to return to lane
    // center
    double time_based_scale = 0.0;
    double plan_length =
        input_->path_segments.back().end_segment_control_point_s;
    double s_proportion =
        (ct.sample_s - input_->planning_init_state.s) / plan_length;
    if (s_proportion < START_REF_CENTER_SCALE_INCREASE_PROPORTION) {
      time_based_scale = START_REF_CENTER_SCALE;
    } else if (s_proportion < START_REF_CENTER_SCALE_DECREASE_PROPORTION) {
      time_based_scale = std::min(
          START_REF_CENTER_SCALE +
              (s_proportion - START_REF_CENTER_SCALE_INCREASE_PROPORTION) *
                  REF_CENTER_SCALE_INCREASE_RATE,
          1.0);
    } else {
      time_based_scale = std::max(
          1.0 - (s_proportion - START_REF_CENTER_SCALE_DECREASE_PROPORTION) *
                    REF_CENTER_SCALE_DECREASE_RATE,
          0.0);
    }

    residual += input_->path_tuning_params.ref_centering_scale *
                time_based_scale *
                fast_math::integer_pow_2(ct.offset - ref_offset);
    if (sample_debug_map) {
      (*sample_debug_map)["cliped_offset"] = ref_offset;
      (*sample_debug_map)["time_based_scale"] = time_based_scale;
    }
  }

  template <typename T>
  void compute_prev_plan_centering_cost(
      const CommonTerms<T> &ct, bool is_replan, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    // Centering to previous ref within given time horizon, so that closeby
    // plan is stable and allow far away plan to absorb perception noise.
    if (!is_replan && ct.t <= MAX_TIME_TO_CENTER_TO_PREV_PLAN_S) {
      residual += input_->path_tuning_params.prev_plan_centering_scale *
                  fast_math::smooth_abs(ct.offset - ct.prev_plan_offset);
    }
  }

  template <typename T>
  void compute_obstacle_cost(
      const CommonTerms<T> &ct, const double &left_constrain,
      const double &right_constrain, const double &left_desire,
      const double &right_desire, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    if (ct.offset > left_constrain) {
      residual += input_->path_tuning_params.obstacle_constrain_scale *
                  fast_math::integer_pow_2(ct.offset - left_constrain);
    }

    if (ct.offset < right_constrain) {
      residual += input_->path_tuning_params.obstacle_constrain_scale *
                  fast_math::integer_pow_2(ct.offset - right_constrain);
    }

    if (ct.offset > left_desire) {
      residual += input_->path_tuning_params.obstacle_desire_scale *
                  fast_math::integer_pow_2(ct.offset - left_desire);
    }

    if (ct.offset < right_desire) {
      residual += input_->path_tuning_params.obstacle_desire_scale *
                  fast_math::integer_pow_2(ct.offset - right_desire);
    }

    if (sample_debug_map) {
      (*sample_debug_map)["obs_left_des"] = left_desire;
      (*sample_debug_map)["obs_right_des"] = right_desire;
    }
  }

  template <typename T>
  void compute_obstacle_inflation_cost(
      const CommonTerms<T> &ct, const double &left_desire,
      const double &right_desire, T &residual,
      std::unordered_map<std::string, double> *sample_debug_map) const {
    if (ct.offset > left_desire) {
      residual += input_->path_tuning_params.obstacle_inflation_scale *
                  fast_math::integer_pow_2(ct.offset - left_desire);
    }

    if (ct.offset < right_desire) {
      residual += input_->path_tuning_params.obstacle_inflation_scale *
                  fast_math::integer_pow_2(ct.offset - right_desire);
    }

    if (sample_debug_map) {
      (*sample_debug_map)["left_inflation"] = left_desire;
      (*sample_debug_map)["right_inflation"] = right_desire;
    }
  }

  template <typename T>
  static CommonTerms<T> compute_common_terms(
      const PathSpline<T> &path_spline, const PathPlannerInput *input,
      const void *input_data_ptr,
      const std::string &input_data_type = "at_quard_point",
      std::unordered_map<std::string, double> *sample_debug_map = nullptr) {
    // get Cartesian frame coordinates
    CommonTerms<T> ct;
    Vector2T<T> xy, dxy, d2xy, d3xy;
    path_planner::PathSampleInfo sample_info;

    if (input_data_type == "at_quard_point") {
      // sample info of current index
      auto index = (Index<NUM_PATH_CONTROL_POINTS> *)input_data_ptr;

      sample_info = input->path_segments[index->segment_index]
                        .quad_point_info[index->quad_index];

      ct.sample_s = path_spline.get_s_at_index(*index);
      xy = path_spline.get_pos_at_index(*index);
      dxy = path_spline.get_derivative_at_index(*index);
      d2xy = path_spline.get_second_derivative_at_index(*index);
      d3xy = path_spline.get_third_derivative_at_index(*index);
    } else if (input_data_type == "at_fix_sample_s") {
      // sample info of sample s
      auto sample_point_coeff_info = (SamplePointCoeffInfo *)input_data_ptr;

      sample_info = compute_sample_info(*input, path_spline,
                                        sample_point_coeff_info->sample_s);

      ct.sample_s = sample_point_coeff_info->sample_s;
      xy = path_spline.get_pos_at_sample_s(*sample_point_coeff_info);
      dxy = path_spline.get_derivative_at_sample_s(*sample_point_coeff_info);
      d2xy = path_spline.get_second_derivative_at_sample_s(
          *sample_point_coeff_info);
      d3xy = path_spline.get_third_derivative_at_sample_s(
          *sample_point_coeff_info);
    }

    ct.t = sample_info.refline_info.time;
    ct.x = xy.x; // parasoft-suppress AUTOSAR-A8_5_0 "f-drop"
    ct.y = xy.y;
    ct.dx_ds = dxy.x; // parasoft-suppress AUTOSAR-A8_5_0 "f-drop"
    ct.dy_ds = dxy.y;
    ct.d2x_ds2 = d2xy.x; // parasoft-suppress AUTOSAR-A8_5_0 "f-drop"
    ct.d2y_ds2 = d2xy.y;
    ct.d3x_ds3 = d3xy.x; // parasoft-suppress AUTOSAR-A8_5_0 "f-drop"
    ct.d3y_ds3 = d3xy.y;

    // get offset of sample point (frenet lateral offset)
    ct.offset = -(ct.x - sample_info.refline_info.x) *
                    sample_info.refline_info.sin_theta_ref +
                (ct.y - sample_info.refline_info.y) *
                    sample_info.refline_info.cos_theta_ref;

    ct.prev_plan_offset = -(sample_info.last_cart_traj.position_enu.x -
                            sample_info.refline_info.x) *
                              sample_info.refline_info.sin_theta_ref +
                          (sample_info.last_cart_traj.position_enu.y -
                           sample_info.refline_info.y) *
                              sample_info.refline_info.cos_theta_ref;

    // get curvature
    const T dl_sq = ct.dx_ds * ct.dx_ds + ct.dy_ds * ct.dy_ds;
    const T dl_1_pt_5 = pow(dl_sq, 1.5);
    const T dxy_cross = ct.dx_ds * ct.d2y_ds2 - ct.d2x_ds2 * ct.dy_ds;
    ct.curvature = dxy_cross / dl_1_pt_5;
    ct.dc_ds = ((ct.dx_ds * ct.d3y_ds3 - ct.d3x_ds3 * ct.dy_ds) * dl_1_pt_5 -
                3.0 * sqrt(dl_sq) * dxy_cross *
                    (ct.dx_ds * ct.d2x_ds2 + ct.dy_ds * ct.d2y_ds2)) /
               fast_math::integer_pow_3(dl_sq);

    // get speed
    ct.v = sample_info.speed_plan.v;
    ct.a = sample_info.speed_plan.a;
    ct.ds_dt = ct.v / sqrt(dl_sq);

    // lat_accel = v * v * curvature
    ct.lat_accel = ct.v * ct.v * ct.curvature;

    // lat_jerk = 2 * v * dv_dt * curvature + v * v * dc_ds *ds_dt
    ct.lat_jerk =
        2 * ct.v * ct.a * ct.curvature + ct.v * ct.v * ct.dc_ds * ct.ds_dt;

    if (sample_debug_map) {
      (*sample_debug_map)["s"] = ct.sample_s;
      (*sample_debug_map)["t"] = ct.t;
      (*sample_debug_map)["v"] = ct.v;
      (*sample_debug_map)["offset"] = fast_math::get_val(ct.offset);
      (*sample_debug_map)["prev_plan_offset"] =
          fast_math::get_val(ct.prev_plan_offset);
      (*sample_debug_map)["curvature"] = fast_math::get_val(ct.curvature);
      (*sample_debug_map)["dc_ds"] = fast_math::get_val(ct.dc_ds);
      (*sample_debug_map)["ds_dt"] = fast_math::get_val(ct.ds_dt);
      (*sample_debug_map)["lat_accel"] = fast_math::get_val(ct.lat_accel);
      (*sample_debug_map)["lat_jerk"] = fast_math::get_val(ct.lat_jerk);
      (*sample_debug_map)["dc_dt"] = fast_math::get_val(ct.dc_ds * ct.ds_dt);

      (*sample_debug_map)["x"] = fast_math::get_val(ct.x);
      (*sample_debug_map)["y"] = fast_math::get_val(ct.y);
      (*sample_debug_map)["refline_infox"] = (sample_info.refline_info.x);
      (*sample_debug_map)["refline_infoy"] = (sample_info.refline_info.y);
    }

    return ct;
  }

  template <typename T>
  static PathSampleInfo compute_sample_info(const PathPlannerInput &input,
                                            const PathSpline<T> &path_spline,
                                            double sample_s) {

    using namespace path_planner;
    using namespace msquare::planning_math;
    PathSampleInfo sample_info_lower = input.planning_init_state.sample_info;
    PathSampleInfo sample_info_upper;

    Index<NUM_PATH_CONTROL_POINTS> index;
    while (index.advance()) {
      if (path_spline.get_s_at_index(index) > sample_s) {
        sample_info_upper = input.path_segments[index.segment_index]
                                .quad_point_info[index.quad_index];
        break;
      }
      sample_info_lower = input.path_segments[index.segment_index]
                              .quad_point_info[index.quad_index];
    }

    return LinearInterpation::interpolate<path_planner::PathSampleInfo>(
        sample_info_lower, sample_info_upper, sample_s);
  }

  template <typename T>
  static T compute_init_curvature(const PathSpline<T> &path_spline,
                                  const double init_s) {
    SamplePointCoeffInfo sample_point_coeff_info;
    sample_point_coeff_info.sample_s = init_s;
    path_spline.compute_sample_point_info(sample_point_coeff_info);
    Vector2T<T> dxy =
        path_spline.get_derivative_at_sample_s(sample_point_coeff_info);
    Vector2T<T> d2xy =
        path_spline.get_second_derivative_at_sample_s(sample_point_coeff_info);

    T dx_ds = dxy.x;
    T dy_ds = dxy.y;
    T d2x_ds2 = d2xy.x;
    T d2y_ds2 = d2xy.y;

    const T dl_sq = dx_ds * dx_ds + dy_ds * dy_ds;
    const T dl_1_pt_5 = pow(dl_sq, 1.5);
    const T dxy_cross = dx_ds * d2y_ds2 - d2x_ds2 * dy_ds;
    return dxy_cross / dl_1_pt_5;
  }

  template <typename T>
  bool operator()(const T *const params, T *residuals) const {
    // initialized all residuals to zero
    std::fill(residuals, residuals + TOTAL_NUM_RESIDUALS, T(0));

    // populate end of segment data
    std::array<Eigen::Vector2d, NUM_PATH_SEGMENTS> ref_pos_at_end_of_segments;
    std::array<Eigen::Vector2d, NUM_PATH_SEGMENTS>
        ref_heading_at_end_of_segments;
    for (size_t i = 0; i < NUM_PATH_SEGMENTS; i++) {
      const auto &end_segment_info =
          input_->path_segments[i].end_segment_refline_info;
      ref_pos_at_end_of_segments[i](0) = end_segment_info.x;
      ref_pos_at_end_of_segments[i](1) = end_segment_info.y;
      ref_heading_at_end_of_segments[i](0) = end_segment_info.cos_theta_ref;
      ref_heading_at_end_of_segments[i](1) = end_segment_info.sin_theta_ref;
    }
    const auto &planning_init_state = input_->planning_init_state;
    PathSpline<T> path_spline(
        path_offset_spline_, planning_init_state.x, planning_init_state.y,
        planning_init_state.dx_ds, planning_init_state.dy_ds, params,
        ref_pos_at_end_of_segments, ref_heading_at_end_of_segments);

    // initial curvature cost
    compute_init_curvature_cost(
        compute_init_curvature(path_spline, input_->planning_init_state.s +
                                                fast_math::MathEpsilon),
        residuals[static_cast<size_t>(PathPlannerResiduals::INIT_CURVATURE)]);

    // iterate through all sampled quadrature points
    Index<NUM_PATH_CONTROL_POINTS> index;
    while (index.advance()) {
      // sample info of current index
      const auto &sample_info = input_->path_segments[index.segment_index]
                                    .quad_point_info[index.quad_index];
      const auto &sample_decison =
          decision_info_->at(index.segment_index)[index.quad_index];

      // notebook debug info, which exposes as much data as you want for offline
      // debugging, but not used in car by assigning nullptr to it.
      std::unordered_map<std::string, double> *sample_debug_map = nullptr;
      if (nb_debug_ != nullptr) {
        const double sample_s = path_spline.get_s_at_index(index);
        nb_debug_->s_to_samples[sample_s] = {};
        sample_debug_map = &nb_debug_->s_to_samples[sample_s].sample_data;
      }

      // compute common term at current quad point
      CommonTerms<T> ct = compute_common_terms(
          path_spline, input_, &index, "at_quard_point", sample_debug_map);

      // compute path planner residuals, note that cost = 0.5 * residual *
      // residual soft boundary cost, which includes lane line and object soft
      // extension
      compute_soft_boundary_cost(
          ct, sample_decison.left_activation_dist,
          sample_decison.right_activation_dist,
          residuals[static_cast<size_t>(PathPlannerResiduals::SOFT_BOUNDARY)],
          sample_debug_map);

      // hard boundary cost, which includes road edge, median, and objects
      compute_hard_boundary_cost(
          ct, sample_decison.left_map_hard_dist,
          sample_decison.right_map_hard_dist,
          residuals[static_cast<size_t>(PathPlannerResiduals::HARD_BOUNDARY)],
          sample_debug_map);

      // curvature limit cost
      // compute_curvature_cost(
      //     ct,
      //     residuals[static_cast<size_t>(PathPlannerResiduals::CURVATURE_LIMIT)],
      //     sample_debug_map);

      // lateral acceleration cost, which includes small regularization and hard
      // limit
      compute_lat_accel_cost(
          ct, residuals[static_cast<size_t>(PathPlannerResiduals::LAT_ACCEL)],
          sample_debug_map);

      // lateral jerk cost, which includes small regularization and hard limit
      compute_lat_jerk_cost(
          ct, residuals[static_cast<size_t>(PathPlannerResiduals::LAT_JERK)],
          sample_debug_map);

      // heading alignment cost
      compute_heading_cost(
          ct, sample_info.refline_info.cos_theta_ref,
          sample_info.refline_info.sin_theta_ref,
          residuals[static_cast<size_t>(PathPlannerResiduals::HEADING_ALIGN)],
          sample_debug_map);

      // reference line centering cost
      double ref_offset =
          decision_info_->at(index.segment_index)[index.quad_index].ref_offset;
      compute_ref_centering_cost(
          ct, ref_offset,
          residuals[static_cast<size_t>(PathPlannerResiduals::REF_CENTERING)],
          sample_debug_map);

      // previous plan centering cost
      compute_prev_plan_centering_cost(
          ct, input_->is_replan,
          residuals[static_cast<size_t>(
              PathPlannerResiduals::PREV_PLAN_CENTERING)],
          sample_debug_map);

      compute_obstacle_cost(
          ct, sample_decison.left_obs_constrain,
          sample_decison.right_obs_constrain, sample_decison.left_obs_desire,
          sample_decison.right_obs_desire,
          residuals[static_cast<size_t>(PathPlannerResiduals::OBSTACLE)],
          sample_debug_map);

      compute_obstacle_inflation_cost(
          ct, sample_decison.left_obs_inflation,
          sample_decison.right_obs_inflation,
          residuals[static_cast<size_t>(
              PathPlannerResiduals::OBSTACLE_INFLATION)],
          sample_debug_map);
    }

    return true;
  }

private:
  const PlannerCubicSpline<NUM_PATH_CONTROL_POINTS> *path_offset_spline_ =
      nullptr;
  const PathPlannerInput *input_;
  const DecisionInfo *decision_info_;
  NotebookDebug *nb_debug_;
};

} // namespace path_planner
