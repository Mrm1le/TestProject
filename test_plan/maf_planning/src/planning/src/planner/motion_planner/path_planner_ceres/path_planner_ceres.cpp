#include "path_planner_ceres.hpp"

namespace path_planner {

PathPlanner::PathPlanner() : solver_options_(), ceres_problem_(ceres_options_) {
  // MSD_LOG(INFO, "Setting up ceres path planner.");

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
  solver_options_.max_num_iterations = 60;
  solver_options_.function_tolerance = 1e-10;
  solver_options_.gradient_tolerance = 1e-10;
  solver_options_.parameter_tolerance = 1e-10;

  // init cost functor and function
  path_cost_functor_ = new PathCostFunctor;
  path_cost_function_ =
      new ceres::AutoDiffCostFunction<PathCostFunctor, TOTAL_NUM_RESIDUALS,
                                      TOTAL_NUM_PARAMS>(path_cost_functor_);
  ceres_problem_.AddResidualBlock(path_cost_function_, nullptr,
                                  opt_params_.data());
  path_planner_decider_ = std::make_shared<PathPlannerDecider>();
}

void PathPlanner::update_input(const PathPlannerInput &input,
                               const bool verbose, NotebookDebug *nb_debug) {
  // Use specified max solver iterations
  solver_options_.max_num_iterations =
      input.path_tuning_params.max_num_iterations;

  // Get control point s values
  std::array<double, NUM_PATH_CONTROL_POINTS> s_at_control_points;
  s_at_control_points[0] = input.planning_init_state.s;
  for (size_t i = 0; i < NUM_PATH_SEGMENTS; i++) {
    s_at_control_points[i + 1] =
        input.path_segments[i].end_segment_control_point_s;
  }
  path_offset_spline_.initialize(s_at_control_points);

  path_planner_decider_->update_input(input);

  path_cost_functor_->update_cost_input(
      &input, &path_offset_spline_,
      path_planner_decider_->get_activation_decision_ptr(), nb_debug);

  run_ceres_opt(input);

  populate_output_info(&input);

  populate_debug_info();

  add_other_info2nb_debug(nb_debug);
}

void PathPlanner::run_ceres_opt(const PathPlannerInput &input) {
  opt_params_ = {};

  init_opt_params(input);

  Solve(solver_options_, &ceres_problem_, &ceres_summary_);

  // Ceres glog printing, only used offline when verbose_ is set to true
  if (verbose_) {
    // std::cout << ceres_summary_.BriefReport() << std::endl;
  }

  solver_report_.type = ceres_summary_.termination_type;
  solver_report_.steps = ceres_summary_.num_successful_steps;
  solver_report_.total_time = ceres_summary_.total_time_in_seconds;
  solver_report_.init_cost = ceres_summary_.initial_cost;
  solver_report_.final_cost = ceres_summary_.final_cost;
  solver_report_.msg = ceres_summary_.message;
}

void PathPlanner::init_opt_params(const PathPlannerInput &input) {
  std::array<double, NUM_PATH_SEGMENTS> init_l;
  const auto decision_ptr =
      path_planner_decider_->get_activation_decision_ptr();
  for (int i = 0; i < opt_params_.size() && i < decision_ptr->size(); i++) {
    opt_params_[i] = decision_ptr->at(i).back().ref_offset;
  }
}

void PathPlanner::populate_output_info(const PathPlannerInput *input) {
  output_.path_planner_output = {};
  output_.avd_result_info = {};
  output_.debug_info_output.clear();

  // lc_end_clear_count_ = false;
  // if (input->lc_decider_info.lc_end ||
  //     input->lc_decider_info.lc_status !=
  //         path_planner::LaneChangeInfo::LaneKeep ||
  //     input->lc_decider_info.pre_lc_wait) {
  //   count_active_avd_ = 0;
  //   last_active_avd_state_ = "none";
  //   last_active_avd_obs_id_ = -1;
  //   last_active_avd_obs_id_count_ = 0;
  //   lc_end_clear_count_ = true;
  // }
  // populate end of segment data
  std::array<Eigen::Vector2d, NUM_PATH_SEGMENTS> ref_pos_at_end_of_segments;
  std::array<Eigen::Vector2d, NUM_PATH_SEGMENTS> ref_heading_at_end_of_segments;
  for (size_t i = 0; i < NUM_PATH_SEGMENTS; i++) {
    // TODO: update this part after interface change, use heading vector
    const auto &end_segment_info =
        input->path_segments[i].end_segment_refline_info;
    ref_pos_at_end_of_segments[i](0) = end_segment_info.x;
    ref_pos_at_end_of_segments[i](1) = end_segment_info.y;
    ref_heading_at_end_of_segments[i](0) = std::cos(end_segment_info.theta_ref);
    ref_heading_at_end_of_segments[i](1) = std::sin(end_segment_info.theta_ref);
  }

  const auto &planning_init_state = input->planning_init_state;
  result_path_spline_ = std::make_shared<PathSpline<double>>(
      &path_offset_spline_, planning_init_state.x, planning_init_state.y,
      planning_init_state.dx_ds, planning_init_state.dy_ds, opt_params_.data(),
      ref_pos_at_end_of_segments, ref_heading_at_end_of_segments);

  // SamplePointCoeffInfo sample_point_coeff_info;
  // sample_point_coeff_info.sample_s = input->planning_init_state.s;
  // result_path_spline_->compute_sample_point_info(sample_point_coeff_info);
  // auto ct = PathCostFunctor::compute_common_terms(
  //     *result_path_spline_, input, &sample_point_coeff_info,
  //     "at_fix_sample_s");
  // std::cout << "init l : " << input->planning_init_state.l
  //           << " curv : " << ct.curvature
  //           << " init : " << input->planning_init_state.curvature <<
  //           std::endl;
  Index<NUM_PATH_CONTROL_POINTS> index;
  while (index.advance()) {
    auto ct = PathCostFunctor::compute_common_terms(*result_path_spline_, input,
                                                    &index);
    const auto &sample_info = input->path_segments[index.segment_index]
                                  .quad_point_info[index.quad_index];
    path_planner::PathPlannerPoint point{
        ct.sample_s, ct.v,       0.0,         0.0,          0.0,        ct.x,
        ct.dx_ds,    ct.d2x_ds2, ct.y,        ct.dy_ds,     ct.d2y_ds2, 0.0,
        0.0,         0.0,        ct.lat_jerk, ct.curvature, sample_info};
    output_.path_planner_output.emplace_back(point);

    const auto dec = path_planner_decider_->get_activation_decision_ptr()->at(
        index.segment_index)[index.quad_index];
    double left_obs_desire =
        std::min(dec.left_obs_desire, dec.left_obs_inflation);
    double right_obs_desire =
        std::max(dec.right_obs_desire, dec.right_obs_inflation);
    path_planner::SampleDebugInfo debug_info{ct.x,
                                             ct.y,
                                             ct.offset,
                                             ct.curvature,
                                             ct.dc_ds * ct.ds_dt,
                                             ct.lat_accel,
                                             ct.lat_jerk,
                                             dec.ref_offset,
                                             dec.left_activation_dist,
                                             dec.right_activation_dist,
                                             dec.left_map_hard_dist,
                                             dec.right_map_hard_dist,
                                             left_obs_desire,
                                             right_obs_desire,
                                             dec.left_obs_constrain,
                                             dec.right_obs_constrain};
    output_.debug_info_output.emplace_back(debug_info);
    // std::cout << "res t : " << ct.t << " l : " << ct.offset
    //           << " jerk : " << ct.lat_jerk << " dcurv : " << ct.dc_ds * ct.v
    //           << " v : " << ct.v << std::endl;
  }

  // get dodge info
  const auto dodge_info = path_planner_decider_->get_dodge_info();
  count_active_avd_ = dodge_info.count_active_avd;
  last_active_avd_state_ = dodge_info.last_active_avd_state;
  last_active_avd_obs_id_ = dodge_info.last_active_avd_obs_id;
  last_active_avd_obs_id_count_ = dodge_info.last_active_avd_obs_id_count;
  lc_end_clear_count_ = dodge_info.lc_end_clear_count;
  // populate active avd info
  const size_t avd_frame_threshold = 3;
  // const auto active_target =
  // path_planner_decider_->get_active_target_offset();
  double max_avd_range = dodge_info.dodge_l;
  int max_avd_object_id = dodge_info.dodge_truck_id;
  if (dodge_info.state == State::Control && max_avd_object_id == -1) {
    max_avd_object_id = last_active_avd_obs_id_;
  }
  last_active_avd_obs_id_ = max_avd_object_id;

  if (dodge_info.state == State::Control) {
    last_active_avd_obs_id_count_ = 0;
    if (max_avd_range > 0.2) {
      if (last_active_avd_state_ != "left") {
        count_active_avd_ = 0;
      } else {
        ++count_active_avd_;
        count_active_avd_ = std::min(count_active_avd_, avd_frame_threshold);
      }
      last_active_avd_state_ = "left";
    } else if (max_avd_range < -0.2) {
      if (last_active_avd_state_ != "right") {
        count_active_avd_ = 0;
      } else {
        ++count_active_avd_;
        count_active_avd_ = std::min(count_active_avd_, avd_frame_threshold);
      }
      last_active_avd_state_ = "right";
    } else {
      if (last_active_avd_obs_id_ == int(max_avd_object_id) &&
          max_avd_range > 0) {
        last_active_avd_state_ = "left";
      } else if (last_active_avd_obs_id_ == int(max_avd_object_id) &&
                 max_avd_range < 0) {
        last_active_avd_state_ = "right";
      }
    }
  } else {
    if (last_active_avd_obs_id_count_ < 10 && last_active_avd_obs_id_ != -1) {
      max_avd_object_id = last_active_avd_obs_id_;
      last_active_avd_obs_id_count_++;
    } else {
      count_active_avd_ = 0;
      last_active_avd_state_ = "none";
      last_active_avd_obs_id_ = -1;
      last_active_avd_obs_id_count_ = 0;
    }
  }
  // std::cout << "last_active_avd_state_ = " << last_active_avd_state_
  //           << std::endl;
  // std::cout << "last_active_avd_obs_id_ = " << last_active_avd_obs_id_
  //           << std::endl;
  // std::cout << "count_active_avd_ = " << count_active_avd_ << std::endl;
  current_dlp_info_.is_in_dlp = false;
  const int DLP_deboune_count = 4;
  if (count_active_avd_ > 2) {
    output_.avd_result_info.avd_direction = last_active_avd_state_;
    output_.avd_result_info.object_id = last_active_avd_obs_id_;
    output_.avd_result_info.type = 1;
    last_active_avd_obs_id_ = max_avd_object_id;
    // std::cout << " --- truck avoid ---" << std::endl;
    // std::cout << "truck id = " << max_avd_object_id << std::endl;
    if (last_active_avd_obs_id_count_ < DLP_deboune_count) {
      check_truck_status(max_avd_object_id, input);
    }
  } else if (count_active_avd_ == 0) {
    output_.avd_result_info.avd_direction = "none";
    output_.avd_result_info.object_id = -1000;
    output_.avd_result_info.type = 1;
    // std::cout << " --- truck avoid not ---" << std::endl;
    output_.avd_result_info.ego_faster_truck = false;
    output_.avd_result_info.overlap_lane = false;
  }

  populate_avd_aim_info();
}

void PathPlanner::populate_debug_info() {
  debug_ = {};

  // Put params, residuals, gradients in array format for Evaluate
  const double *x_eval[] = {opt_params_.data()};
  double opt_residual[TOTAL_NUM_RESIDUALS];
  double opt_residual_gradients[TOTAL_NUM_RESIDUALS][TOTAL_NUM_PARAMS];
  double *opt_residual_gradients_rows[TOTAL_NUM_RESIDUALS];
  for (size_t i = 0; i < TOTAL_NUM_RESIDUALS; i++) {
    opt_residual_gradients_rows[i] = &opt_residual_gradients[i][0];
  }

  // re-evaluate at x_eval
  (void)path_cost_function_->Evaluate(x_eval, opt_residual,
                                      opt_residual_gradients_rows);

  // get debug residual
  for (size_t i = 0; i < TOTAL_NUM_RESIDUALS; i++) {
    debug_.residual_debug[i] = opt_residual[i];
  }

  // get debug gradients
  for (size_t i = 0; i < TOTAL_NUM_PARAMS; i++) {
    for (size_t j = 0; j < TOTAL_NUM_RESIDUALS; j++) {
      // cost is sum(0.5 * residual^2), gradient is sum(residual*gradient)
      debug_.per_residual_per_param_gradient[i][j] =
          opt_residual_gradients[j][i] * opt_residual[j];
      debug_.total_gradients_per_param[i] +=
          debug_.per_residual_per_param_gradient[i][j];
    }
  }

  // get debug activation distance
  debug_.lane_activation_dist =
      path_planner_decider_->get_lane_activation_dist();
  debug_.obj_activation_dist = path_planner_decider_->get_obj_activation_dist();
}

void PathPlanner::get_output_at_s(const PathPlannerInput *input,
                                  path_planner::PathPlannerPoint &output,
                                  double sample_s) const {
  SamplePointCoeffInfo sample_point_coeff_info;
  sample_point_coeff_info.sample_s = sample_s;
  result_path_spline_->compute_sample_point_info(sample_point_coeff_info);
  auto ct = PathCostFunctor::compute_common_terms(
      *result_path_spline_, input, &sample_point_coeff_info, "at_fix_sample_s");

  auto sample_info = PathCostFunctor::compute_sample_info(
      *input, *result_path_spline_, sample_s);

  output = path_planner::PathPlannerPoint{
      ct.sample_s, ct.v,       0.0,         0.0,          0.0,        ct.x,
      ct.dx_ds,    ct.d2x_ds2, ct.y,        ct.dy_ds,     ct.d2y_ds2, 0.0,
      0.0,         0.0,        ct.lat_jerk, ct.curvature, sample_info};
}

void PathPlanner::add_other_info2nb_debug(NotebookDebug *nb_debug) {
  if (nb_debug != nullptr) {
    // add output_
    // nb_debug->output = output_;
    // for (const auto output_tmp : output_.path_planner_output) {
    //   std::cout << "output_tmp.s = " << output_tmp.s << std::endl;
    // }
    // for (const auto sample_s_key : nb_debug->s_to_samples) {
    //   std::cout << "sample_s_key.first = " << sample_s_key.first <<
    //   std::endl;
    // }

    for (const auto output_tmp : output_.path_planner_output) {
      double s_min_matched = 1000000;
      double s_delta = 1000000;
      for (const auto sample_s_key : nb_debug->s_to_samples) {
        if (std::abs(sample_s_key.first - output_tmp.s) < s_delta) {
          s_delta = std::abs(sample_s_key.first - output_tmp.s);
          s_min_matched = sample_s_key.first;
        }
      }
      // std::cout << "s_min_matched = " << s_min_matched << std::endl;
      nb_debug->s_to_samples[s_min_matched].sample_data["dx_ds"] =
          output_tmp.dx_ds;
      nb_debug->s_to_samples[s_min_matched].sample_data["d2x_ds2"] =
          output_tmp.d2x_ds2;
      nb_debug->s_to_samples[s_min_matched].sample_data["dy_ds"] =
          output_tmp.dy_ds;
      nb_debug->s_to_samples[s_min_matched].sample_data["d2y_ds2"] =
          output_tmp.d2y_ds2;

      nb_debug->s_to_samples[s_min_matched]
          .sample_data["last_cart_traj_position_enu_x"] =
          output_tmp.sample_info.last_cart_traj.position_enu.x;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["last_cart_traj_position_enu_y"] =
          output_tmp.sample_info.last_cart_traj.position_enu.y;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["last_cart_traj_heading_yaw"] =
          output_tmp.sample_info.last_cart_traj.heading_yaw;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["last_cart_traj_curvature"] =
          output_tmp.sample_info.last_cart_traj.curvature;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["last_cart_traj_path_follow_strength"] =
          output_tmp.sample_info.last_cart_traj.path_follow_strength;

      nb_debug->s_to_samples[s_min_matched].sample_data["refline_info_time"] =
          output_tmp.sample_info.refline_info.time;
      nb_debug->s_to_samples[s_min_matched].sample_data["refline_info_x"] =
          output_tmp.sample_info.refline_info.x;
      nb_debug->s_to_samples[s_min_matched].sample_data["refline_info_y"] =
          output_tmp.sample_info.refline_info.y;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["refline_info_curvature"] =
          output_tmp.sample_info.refline_info.curvature;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["refline_info_current_lane_width"] =
          output_tmp.sample_info.refline_info.current_lane_width;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["refline_info_left_lane_width"] =
          output_tmp.sample_info.refline_info.left_lane_width;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["refline_info_right_lane_width"] =
          output_tmp.sample_info.refline_info.right_lane_width;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["refline_info_left_lane_border"] =
          output_tmp.sample_info.refline_info.left_lane_border;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["refline_info_right_lane_border"] =
          output_tmp.sample_info.refline_info.right_lane_border;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["refline_info_left_road_border"] =
          output_tmp.sample_info.refline_info.left_road_border;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["refline_info_right_road_border"] =
          output_tmp.sample_info.refline_info.right_road_border;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["refline_info_theta_ref"] =
          output_tmp.sample_info.refline_info.theta_ref;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["refline_info_cos_theta_ref"] =
          output_tmp.sample_info.refline_info.cos_theta_ref;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["refline_info_sin_theta_ref"] =
          output_tmp.sample_info.refline_info.sin_theta_ref;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["refline_info_left_road_border_type"] =
          output_tmp.sample_info.refline_info.left_road_border_type;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["refline_info_right_road_border_type"] =
          output_tmp.sample_info.refline_info.right_road_border_type;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["refline_info_left_lane_border_type"] =
          output_tmp.sample_info.refline_info.left_lane_border_type;
      nb_debug->s_to_samples[s_min_matched]
          .sample_data["refline_info_right_lane_border_type"] =
          output_tmp.sample_info.refline_info.right_lane_border_type;

      nb_debug->s_to_samples[s_min_matched].sample_data["speed_plan_v"] =
          output_tmp.sample_info.speed_plan.v;
      nb_debug->s_to_samples[s_min_matched].sample_data["speed_plan_a"] =
          output_tmp.sample_info.speed_plan.a;
      nb_debug->s_to_samples[s_min_matched].sample_data["speed_plan_j"] =
          output_tmp.sample_info.speed_plan.j;
    }
    nb_debug->avd_direction = output_.avd_result_info.avd_direction;
    nb_debug->object_id = output_.avd_result_info.object_id;
    nb_debug->type = output_.avd_result_info.type;

    // add debug_
    nb_debug->residual_debug = debug_.residual_debug;
    nb_debug->per_residual_per_param_gradient =
        debug_.per_residual_per_param_gradient;
    nb_debug->total_gradients_per_param = debug_.total_gradients_per_param;
    nb_debug->lane_activation_dist = debug_.lane_activation_dist;
  }
}

void PathPlanner::populate_avd_aim_info() {
  output_.left_aim_info = {};
  output_.right_aim_info = {};

  output_.left_aim_info.id = path_planner_decider_->get_left_aim().id;
  output_.left_aim_info.type = path_planner_decider_->get_left_aim().type;
  output_.left_aim_info.avd_direction =
      path_planner_decider_->get_left_aim().avd_dir;
  output_.left_aim_info.a = path_planner_decider_->get_left_aim().a;
  output_.left_aim_info.max_l = path_planner_decider_->get_left_aim().max_l;
  output_.left_aim_info.min_l = path_planner_decider_->get_left_aim().min_l;
  output_.left_aim_info.desire_buffer =
      path_planner_decider_->get_left_aim().desire_buffer;
  output_.left_aim_info.lane_border_base_l =
      path_planner_decider_->get_left_aim().lane_border_base_l;

  output_.right_aim_info.id = path_planner_decider_->get_right_aim().id;
  output_.right_aim_info.type = path_planner_decider_->get_right_aim().type;
  output_.right_aim_info.avd_direction =
      path_planner_decider_->get_right_aim().avd_dir;
  output_.right_aim_info.a = path_planner_decider_->get_right_aim().a;
  output_.right_aim_info.max_l = path_planner_decider_->get_right_aim().max_l;
  output_.right_aim_info.min_l = path_planner_decider_->get_right_aim().min_l;
  output_.right_aim_info.desire_buffer =
      path_planner_decider_->get_right_aim().desire_buffer;
  output_.right_aim_info.lane_border_base_l =
      path_planner_decider_->get_right_aim().lane_border_base_l;
}

void PathPlanner::check_truck_status(int truck_id,
                                     const PathPlannerInput *input) {
  for (const auto &obs : input->obs_list) {
    if (obs.id == truck_id) {
      double half_self_length = input->vehicle_param.length / 2.;
      double ego_head_behind_obs_head_thresh = 7.0;
      if (!obs.polygon_init.polygon_fren.empty()) {
        if (obs.polygon_init.polygon_fren.back().x <
                input->planning_init_state.s + half_self_length +
                    ego_head_behind_obs_head_thresh &&
            !input->pre_dlp_info.is_in_dlp) {
          return;
        }
      }
      current_dlp_info_.is_in_dlp = true;
      output_.avd_result_info.ego_faster_truck =
          input->planning_init_state.v > obs.polygon_init.v ? true : false;
      // std::cout << "ego_faster_truck "
      //           << output_.avd_result_info.ego_faster_truck << std::endl;
      Index<NUM_PATH_CONTROL_POINTS> index;
      while (index.advance()) {
        const auto &sample_info = input->path_segments[index.segment_index]
                                      .quad_point_info[index.quad_index];
        if (sample_info.sample_s > obs.polygon_init.s) {
          double max_l = -std::numeric_limits<double>::max();
          double min_l = std::numeric_limits<double>::max();
          for (const auto p : obs.polygon_init.polygon_fren) {
            max_l = std::max(max_l, p.y);
            min_l = std::min(min_l, p.y);
          }
          // std::cout << "max_l = " << max_l << std::endl;
          // std::cout << "min_l = " << min_l << std::endl;
          // std::cout << "sample_info.refline_info.right_lane_border = "
          //           << sample_info.refline_info.right_lane_border <<
          //           std::endl;
          // std::cout << "sample_info.refline_info.left_lane_border = "
          //           << sample_info.refline_info.left_lane_border <<
          //           std::endl;
          if (obs.nudge_side == ObsInfo::NudgeType::LEFT_NUDGE &&
              -sample_info.refline_info.right_lane_border < max_l) {
            output_.avd_result_info.overlap_lane = true;
            // std::cout << "left_nudge && overlap_lane" << std::endl;
          } else if (obs.nudge_side == ObsInfo::NudgeType::RIGHT_NUDGE &&
                     sample_info.refline_info.left_lane_border > min_l) {
            output_.avd_result_info.overlap_lane = true;
            // std::cout << "right_nudge && overlap_lane" << std::endl;
          }
          break;
        }
      }
      return;
    }
  }
  return;
}
} // namespace path_planner
