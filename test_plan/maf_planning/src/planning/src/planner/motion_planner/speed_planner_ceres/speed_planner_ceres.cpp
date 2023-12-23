#include "speed_planner_ceres.hpp"

namespace speed_planner {

SpeedPlanner::SpeedPlanner()
    : solver_options_(), ceres_problem_(ceres_options_) {
  // MSD_LOG(INFO, "Setting up ceres speed planner.");

  // set up cost function ownership
  ceres_options_.cost_function_ownership =
      ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  ceres_options_.loss_function_ownership =
      ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  ceres_options_.local_parameterization_ownership =
      ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;

  // set solver optons
  solver_options_.minimizer_type = ceres::LINE_SEARCH,
  solver_options_.line_search_direction_type =
      ceres::LineSearchDirectionType::LBFGS,
  solver_options_.use_approximate_eigenvalue_bfgs_scaling = true;
  solver_options_.nonlinear_conjugate_gradient_type =
      ceres::NonlinearConjugateGradientType::HESTENES_STIEFEL;

  // logging
  solver_options_.minimizer_progress_to_stdout = false;
  solver_options_.logging_type = ceres::LoggingType::SILENT;
  solver_options_.check_gradients = false;

  // solver time
  solver_options_.max_solver_time_in_seconds =
      0.02; // 0.02 debug printing takes time!
  solver_options_.max_num_iterations = 40;
  solver_options_.function_tolerance = 1e-10;
  solver_options_.gradient_tolerance = 1e-10;

  // init cost functor and function
  speed_cost_functor_ = new SpeedCostFunctor;
  speed_cost_function_ =
      new ceres::AutoDiffCostFunction<SpeedCostFunctor, TOTAL_NUM_RESIDUALS,
                                      TOTAL_NUM_PARAMS>(speed_cost_functor_);
  ceres_problem_.AddResidualBlock(speed_cost_function_, nullptr,
                                  opt_params_.data());
}

void SpeedPlanner::update_CIPV_info(SpeedPlannerInput *input) {
  const auto &planning_init_state = input->planning_init_state;
  CommonTerms<double> last_quad_point_ct;
  last_quad_point_ct.s = planning_init_state.s;
  last_quad_point_ct.t = 0.0;
  last_quad_point_ct.v = planning_init_state.v;
  last_quad_point_ct.a = planning_init_state.a;
  last_quad_point_ct.jerk = planning_init_state.jerk;

  Index<NUM_SPEED_CONTROL_POINTS> index;
  const auto &lon_objs = input->lon_obs;
  const auto &params = input->speed_tuning_params.get_acc_params();
  const auto &center_to_front = input->vehicle_param.center_to_front;

  auto &CIPV_map_mem = input->CIPV_map_mem;
  std::map<int, double> CIPV_map = {};
  const size_t CIPV_map_size = 6;
  while (index.advance()) {
    auto ct = SpeedCostFunctor::compute_common_terms(
        *result_acceleration_spline_, last_quad_point_ct, input, &index);
    last_quad_point_ct = ct;

    LonObjData<double> critical_obj_data = {};
    critical_obj_data.dv_curve = MAX_VALID_SPEED_MPS;
    for (const auto &obj : lon_objs) {
      // predict object state using specified model
      // model can be const accel or from prediction
      double s, v, a;
      double dist_ratio = 1;
      dist_ratio = std::fmax(std::fmin(dist_ratio, 1.5), 0.5);
      double ax_ratio = 1; // 1.5 - dist_ratio;

      predict_obj_state_at_t(input->use_prediction, obj,
                             fast_math::get_val(ct.t), s, v, a);

      // get v_safe using max decel
      const double ds = std::max(double(s) - ct.s, double(center_to_front));
      // get v_curve from desired acc curve
      double ds_follow_offseted, dv_curve, da_curve, dv_deadband;
      update_dv_curve(obj.polygon_init.stop_offset,
                      obj.polygon_init.desired_headway, double(v), ds,
                      center_to_front, params.deadband_region_ratio,
                      ds_follow_offseted, dv_curve, da_curve, dv_deadband,
                      input->ego_state.v);
      // get most critical object
      // CJ:at hold state don't change cipv until start off
      if (input->stop_point_info.hold_cnt == true &&
          obj.id == input->cipv_info.cipv_id) {
        critical_obj_data.id = obj.id;
        critical_obj_data.ds_follow_offseted = ds_follow_offseted;
        // critical_obj_data.dv_safe = dv_safe;
        critical_obj_data.dv_curve = dv_curve;
        critical_obj_data.dv_deadband = dv_deadband;
        critical_obj_data.da_curve = da_curve;
        critical_obj_data.obj_v = double(v);
        critical_obj_data.obj_a = double(a);
        critical_obj_data.obj_ds = ds;
        critical_obj_data.obj_headway =
            double(obj.polygon_init.desired_headway);
      } else if (dv_curve + v <
                 critical_obj_data.dv_curve + critical_obj_data.obj_v) {
        critical_obj_data.id = obj.id;
        critical_obj_data.ds_follow_offseted = ds_follow_offseted;
        // critical_obj_data.dv_safe = dv_safe;
        critical_obj_data.dv_curve = dv_curve;
        critical_obj_data.dv_deadband = dv_deadband;
        critical_obj_data.da_curve = da_curve;
        critical_obj_data.obj_v = double(v);
        critical_obj_data.obj_a = double(a);
        critical_obj_data.obj_ds = ds;
        critical_obj_data.obj_headway =
            double(obj.polygon_init.desired_headway);
      }
    }
    if (CIPV_map.find(critical_obj_data.id) == CIPV_map.end()) {
      CIPV_map[critical_obj_data.id] = ct.t;
    }
  }
  if (CIPV_map_mem.size() < CIPV_map_size) {
    CIPV_map_mem.push_back(CIPV_map);
  } else {
    for (int i = 0; i < CIPV_map_mem.size() - 1; ++i) {
      CIPV_map_mem[i] = CIPV_map_mem[i + 1];
    }
    CIPV_map_mem.back() = CIPV_map;
  }
}
void SpeedPlanner::update_input(const SpeedPlannerInput &input,
                                const bool verbose,
                                path_planner::NotebookDebug *nb_debug) {
  // Use specified max solver iterations
  solver_options_.max_num_iterations =
      input.speed_tuning_params.max_num_iterations;
  solver_options_.max_solver_time_in_seconds = nb_debug ? 100.0 : 0.02;

  acceleration_spline_.initialize(input.s_at_control_points);

  speed_cost_functor_->update_cost_input(&input, &acceleration_spline_,
                                         nb_debug);

  select_init_seed(input);

  run_ceres_opt();

  populate_output_info(&input, opt_params_, acceleration_spline_);

  debug_ = populate_debug_info(opt_params_);
}

void SpeedPlanner::select_init_seed(const SpeedPlannerInput &input) {
  opt_params_ = {};
  const auto &init_state = input.planning_init_state;
  const int SEED_SIZE = 5;
  std::array<double, SEED_SIZE> jerk_array = {-2, -1, 0, 1, 2};
  double delta_s = input.s_at_control_points[1] - input.s_at_control_points[0];
  double delta_t = delta_s / std::fmax(init_state.v, 0.1);

  auto compute_seed_with_const_jerk = [&](const double &jerk,
                                          const double &delta_t) {
    OptParamArray seed = {};
    for (size_t i = 0; i < speed_planner::TOTAL_NUM_PARAMS; i++) {
      seed[i] = (i + 1) * delta_t * jerk + init_state.a;
    }
    return seed;
  };

  double total_cost = std::numeric_limits<double>::max();
  OptParamArray select_seed = {};
  auto update_seed = [&](const OptParamArray &seed) {
    auto debug = populate_debug_info(seed);
    double cost = 0.0;
    for (auto residual : debug.residual_debug) {
      cost += 0.5 * residual * residual;
    }

    if (cost < total_cost) {
      total_cost = cost;
      select_seed = seed;
    }
  };

  for (int i = 0; i < SEED_SIZE; i++) {
    auto seed = compute_seed_with_const_jerk(jerk_array[i], delta_t);
    auto debug = populate_debug_info(seed);
    update_seed(seed);
  }

  update_seed(input.last_opt_params);
  opt_params_ = select_seed;

  double set_hw = 0;
  if (input.lon_obs.size() > 0) {
    set_hw = input.lon_obs[0].polygon_init.desired_headway;
  }
  // std::cout << "Set_Headway:" << set_hw << std::endl;
  // std::cout << "select_seed params is ";
  // for (auto & param : opt_params_) {
  //   std::cout << param << ", ";
  // }
  // std::cout << std::endl;

  // cout debug
  // auto debug_low_spd = input.stop_point_info.low_speed_state;
  // auto debug_hold_flag = input.stop_point_info.hold_flag;
  // auto debug_auto_go = input.stop_point_info.auto_go;
  // auto debug_go_indicator = input.stop_point_info.go_indicator;
  // auto debug_stop_mode = input.stop_point_info.stop_mode;
  // auto debug_hold_cnt = input.stop_point_info.hold_cnt;
  // auto debug_stop_timer = input.stop_point_info.stop_timer;
  // auto last_CIPV_id = input.stop_point_info.last_CIPV_id;
  // auto standstill = input.stop_point_info.standstill;
  // std::cout << "-------------" << std::endl;
  // std::cout << "     low_spd:" << debug_low_spd << std::endl;
  // std::cout << "   hold_flag:" << debug_hold_flag << std::endl;
  // std::cout << "     auto_go:" << debug_auto_go << std::endl;
  // std::cout << "go_indicator:" << debug_go_indicator << std::endl;
  // std::cout << " stop_mode:" << debug_stop_mode << std::endl;
  // std::cout << " hold_cnt:" << debug_hold_cnt << std::endl;
  // std::cout << " stop_timer:" << debug_stop_timer << std::endl;
  // std::cout << "last_CIPV_id:" << last_CIPV_id << std::endl;
  // std::cout << "  standstill:" << standstill << std::endl;
}

void SpeedPlanner::run_ceres_opt() {
  // TODO: MPILPNC-94 add seeding strategies
  Solve(solver_options_, &ceres_problem_, &ceres_summary_);

  // Ceres glog printing, only used offline when verbose_ is set to true
  if (verbose_) {
    // std::cout << ceres_summary_.FullReport() << std::endl;
  }

  solver_report_.type = ceres_summary_.termination_type;
  solver_report_.steps = ceres_summary_.num_successful_steps;
  solver_report_.total_time = ceres_summary_.total_time_in_seconds;
  solver_report_.init_cost = ceres_summary_.initial_cost;
  solver_report_.final_cost = ceres_summary_.final_cost;
  solver_report_.msg = ceres_summary_.message;
}

void SpeedPlanner::populate_output_info(
    const SpeedPlannerInput *input, const OptParamArray &opt_params,
    const PlannerCubicSpline<NUM_SPEED_CONTROL_POINTS> &acceleration_spline) {
  output_ = {};

  const auto &planning_init_state = input->planning_init_state;
  result_acceleration_spline_ = std::make_shared<AccelerationSpline<double>>(
      &acceleration_spline, planning_init_state.a, planning_init_state.da_ds,
      opt_params.data());
  CommonTerms<double> last_quad_point_ct;
  last_quad_point_ct.s = planning_init_state.s;
  last_quad_point_ct.t = 0.0;
  last_quad_point_ct.v = planning_init_state.v;
  last_quad_point_ct.a = planning_init_state.a;
  last_quad_point_ct.jerk = planning_init_state.jerk;

  Index<NUM_SPEED_CONTROL_POINTS> index;
  while (index.advance()) {
    auto ct = SpeedCostFunctor::compute_common_terms(
        *result_acceleration_spline_, last_quad_point_ct, input, &index);
    last_quad_point_ct = ct;

    const auto &quad_point_info =
        input->speed_segments[index.segment_index][index.quad_index]
            .quad_point_info;
    path_planner::PathPlannerPoint point{};
    point.t = ct.t;
    point.v = ct.v;
    point.a = ct.a;
    point.s = ct.s;
    point.jerk = ct.jerk;
    point.da_ds = result_acceleration_spline_->get_derivative_at_index(index);
    output_.path_planner_output.emplace_back(point);
  }
}

SpeedPlannerDebug
SpeedPlanner::populate_debug_info(const OptParamArray &opt_params) {
  SpeedPlannerDebug debug = {};

  // Put params, residuals, gradients in array format for Evaluate
  const double *x_eval[] = {opt_params.data()};
  double opt_residual[TOTAL_NUM_RESIDUALS];
  double opt_residual_gradients[TOTAL_NUM_RESIDUALS][TOTAL_NUM_PARAMS];
  double *opt_residual_gradients_rows[TOTAL_NUM_RESIDUALS];
  for (size_t i = 0; i < TOTAL_NUM_RESIDUALS; i++) {
    opt_residual_gradients_rows[i] = &opt_residual_gradients[i][0];
  }

  // re-evaluate at x_eval
  (void)speed_cost_function_->Evaluate(x_eval, opt_residual,
                                       opt_residual_gradients_rows);
  for (size_t i = 0; i < TOTAL_NUM_RESIDUALS; i++) {
    debug.residual_debug[i] = opt_residual[i];
  }
  // cout debug
  // std::cout << "residual & opt_params" << std::endl;
  // if (std::fabs(opt_params_[0]) > 0.0001)
  // {
  //   std::cout << std::endl;
  //   std::cout << "cur_spd_limit_cost:";
  //   std::cout << opt_residual[0] << std::endl;
  //   std::cout << "  accel_limit_cost:";
  //   std::cout << opt_residual[1] << std::endl;
  //   std::cout << "        accel_cost:";
  //   std::cout << opt_residual[2] << std::endl;
  //   std::cout << "   jerk_limit_cost:";
  //   std::cout << opt_residual[3] << std::endl;
  //   std::cout << "         jerk_cost:";
  //   std::cout << opt_residual[4] << std::endl;
  //   std::cout << "         stop_cost:";
  //   std::cout << opt_residual[5] << std::endl;
  //   std::cout << "          acc_cost:";
  //   std::cout << opt_residual[6] << std::endl;
  //   std::cout << "opt params is ";
  //   for (auto &param : opt_params_) {
  //     std::cout << param << ", ";
  //   }
  //   std::cout << std::endl;
  // }

  // get debug gradients
  for (size_t i = 0; i < TOTAL_NUM_PARAMS; i++) {
    for (size_t j = 0; j < TOTAL_NUM_RESIDUALS; j++) {
      // cost is sum(0.5 * residual^2), gradient is sum(residual*gradient)
      debug.per_residual_per_param_gradient[i][j] =
          opt_residual_gradients[j][i] * opt_residual[j];
      debug.total_gradients_per_param[i] +=
          debug.per_residual_per_param_gradient[i][j];
    }
  }
  return debug;
}

} // namespace speed_planner
