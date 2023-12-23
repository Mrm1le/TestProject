#include "planner/behavior_planner/general_motion_planner.h"
#include "planner/motion_planner/path_planner_ceres/path_planner_constants.hpp"
#include "planner/motion_planner/planner_cubic_spline/planner_cubic_spline_utility.hpp"
#include <math.h>
#include <string>
#include <utility>

namespace msquare {

GeneralMotionPlanner::GeneralMotionPlanner(const TaskConfig &config)
    : Task(config) {
  gmp_preprocessor_ = make_shared<GeneralMotionPlannerPreprocessor>();
  gmp_preprocessor_->mutable_external_input(&general_motion_planner_input_,
                                            &general_motion_planner_output_);
}

GeneralMotionPlanner::~GeneralMotionPlanner() {}

void GeneralMotionPlanner::init(std::shared_ptr<WorldModel> world_model) {
  Task::init(world_model);
}

void GeneralMotionPlanner::reset(const TaskConfig &config) {
  Task::reset(config);
  frenet_coord_ = nullptr;
  gmp_baseline_info_ = nullptr;
}

void GeneralMotionPlanner::unset(void) {
  frenet_coord_ = nullptr;
  gmp_baseline_info_ = nullptr;
}

void GeneralMotionPlanner::get_external_request(
    LC_Dir external_request, bool valid_body, bool is_steer_over_limit,
    bool is_lca_state_activated, bool is_lane_stable, bool is_solid_line,
    bool has_olane, bool has_tlane, bool gmp_should_cancel) {
  lc_request_ = external_request;
  valid_body_ = valid_body;
  is_steer_over_limit_ = is_steer_over_limit;
  is_lca_state_activated_ = is_lca_state_activated;
  is_lane_stable_ = is_lane_stable;
  is_solid_line_ = is_solid_line;
  has_olane_ = has_olane;
  has_tlane_ = has_tlane;
  gmp_should_cancel_ = gmp_should_cancel;
}

void GeneralMotionPlanner::get_created_time(void) {
  if (call_count_ == 1) {
    gmp_created_time_ = MTIME()->timestamp().sec();
  }
  return;
}

int GeneralMotionPlanner::simulate_lc_trigger(const int &direction,
                                              const double &time) {
  double time_curr = MTIME()->timestamp().sec();
  int lc_trigger = 0;

  if (time_curr > gmp_created_time_ + time) {
    lc_trigger = direction;
  }

  if (time_curr > gmp_created_time_ + time + 7) {
    lc_trigger = 0;
  }
  return lc_trigger;
}

void GeneralMotionPlanner::gerenate_s_at_control_quad_points(void) {
  const double SPEED_PLANNER_TIME = 5.0;
  const double MAX_LENGTH = 120.0;
  const double MIN_LENGTH = 5.0;
  const auto &planning_init_point =
      baseline_info_->get_ego_state().planning_init_point;

  double total_length =
      std::fmin(std::fmax(15.0, planning_init_point.v * SPEED_PLANNER_TIME),
                baseline_info_->get_frenet_coord()->GetLength() -
                    planning_init_point.path_point.s - fast_math::MathEpsilon);
  total_length = std::fmax(MIN_LENGTH, std::fmin(total_length, MAX_LENGTH));

  double delta_s = total_length / double(speed_planner::NUM_SPEED_SEGMENTS);

  // control point
  for (size_t segment_index = 0;
       segment_index < speed_planner::NUM_SPEED_CONTROL_POINTS;
       segment_index++) {
    s_at_control_points_.at(segment_index) =
        delta_s * segment_index + planning_init_point.path_point.s;
  }

  // quad point
  planner_spline::Index<speed_planner::NUM_SPEED_CONTROL_POINTS> index;
  int ii = 0;
  while (index.advance()) {
    s_at_quad_points_[index.segment_index][index.quad_index] =
        s_at_control_points_[index.segment_index] +
        (s_at_control_points_[index.segment_index + 1] -
         s_at_control_points_[index.segment_index]) *
            GAUSS_QUAD_5TH_POS[index.quad_index];
    MSD_LOG(INFO, "GMP s_quad[%d]: %.2f", ii,
            s_at_quad_points_[index.segment_index][index.quad_index]);
    ii++;
  }
}

PathSegment GeneralMotionPlanner::plan_path_segment(
    const gmp_interface::TargetPoint &tp_start,
    const gmp_interface::TargetPoint &tp_end) {

  PathSegment path_segment{0, 0, 0, 0, 0, 0, 0, 0};

  double x0 = tp_start.x, y0 = tp_start.y; // tp_start.x is longitu.(large)
  double xt = tp_end.x, yt = tp_end.y;

  double C0 = y0;
  double C1 = tp_start.heading;
  double xt_re = xt - x0;
  double end_slope = tp_end.heading;

  Eigen::MatrixXd A(2, 2);
  Eigen::MatrixXd B(2, 1);

  A << pow(xt_re, 2), pow(xt_re, 3), 2 * xt_re, 3 * pow(xt_re, 2);

  B << yt - C0 - C1 * xt_re, end_slope - C1;

  Eigen::MatrixXd PP_Temp = (A.transpose() * A).inverse() * A.transpose() * B;

  double C2 = PP_Temp(0, 0);
  double C3 = PP_Temp(1, 0);

  path_segment.c0 = C0 - C1 * x0 + C2 * pow(x0, 2) - C3 * pow(x0, 3);
  path_segment.c1 = C1 - 2 * C2 * x0 + 3 * C3 * pow(x0, 2);
  path_segment.c2 = C2 - 3 * C3 * x0;
  path_segment.c3 = C3;
  path_segment.c4 = 0.0;
  path_segment.c5 = 0.0;

  path_segment.x_end = tp_end.y; // lateral
  path_segment.y_end = tp_end.x; // longitu.

  return path_segment;
}

void GeneralMotionPlanner::get_path_point(BehaviorAdvice &behavior_advice) {
  using gmp_interface::TrajectoryGMP;
  using gmp_interface::TrajectoryPointGMP;

  TrajectoryGMP gmp_path{{}, 0, 0};
  const vector<PathSegment> &path_segments = behavior_advice.path_segments;

  if (path_segments.empty()) {
    behavior_advice.trajectory = gmp_path;
    return;
  }

  auto cmp = [](const double &b, const PathSegment &a) { return b <= a.y_end; };

  double sample_end = 200.0, sample_start = 0.0;
  double sample_step = 2.0;

  double curr_sample = sample_start + sample_step;

  double breakpoint_ini_lon = path_segments.front().y_end;
  double breakpoint_end_lon = path_segments.back().y_end;

  STArray st_array = generate_st_array(behavior_advice);
  behavior_advice.st_array = st_array;
  for (const double &curr_sample : st_array.s) {
    if (curr_sample >= breakpoint_end_lon) {
      break;
    }
    auto p_seg_index = std::upper_bound(path_segments.begin(),
                                        path_segments.end(), curr_sample, cmp);
    // MSD_LOG(INFO,"PP_Seg: %.2fm, sample:
    // %.2fm",(*p_seg_index).y_end,curr_sample);
    PathSegment pp_seg = *p_seg_index;

    double curr_lat =
        pp_seg.c0 + pp_seg.c1 * curr_sample + pp_seg.c2 * pow(curr_sample, 2) +
        pp_seg.c3 * pow(curr_sample, 3) + pp_seg.c4 * pow(curr_sample, 4) +
        pp_seg.c5 * pow(curr_sample, 5);

    TrajectoryPointGMP pp_point = {curr_sample, curr_lat, -1.0,
                                   -1.0,        -1.0,     -1.0};
    gmp_path.trajectory.emplace_back(pp_point);
  }
  gmp_path.motion_type = behavior_advice.motion_type;
  behavior_advice.trajectory = gmp_path;
  // return gmp_path;
}

gmp_interface::TrajectoryGMP
GeneralMotionPlanner::get_path_point(const vector<PathSegment> &path_segments) {
  using gmp_interface::TrajectoryGMP;
  using gmp_interface::TrajectoryPointGMP;

  TrajectoryGMP gmp_path{{}, 0, 0};
  auto cmp = [](const double &b, const PathSegment &a) { return b <= a.y_end; };

  double sample_end = 200.0, sample_start = 0.0;
  double sample_step = 2.0;

  double curr_sample = sample_start + sample_step;

  if (path_segments.empty()) {
    return gmp_path;
  }
  double breakpoint_ini_lon = path_segments.front().y_end;
  double breakpoint_end_lon = path_segments.back().y_end;

  while (curr_sample <= sample_end) {
    if (curr_sample >= breakpoint_end_lon) {
      break;
    }
    auto p_seg_index = std::upper_bound(path_segments.begin(),
                                        path_segments.end(), curr_sample, cmp);
    // MSD_LOG(INFO,"PP_Seg: %.2fm, sample:
    // %.2fm",(*p_seg_index).y_end,curr_sample);
    PathSegment pp_seg = *p_seg_index;

    double curr_lat =
        pp_seg.c0 + pp_seg.c1 * curr_sample + pp_seg.c2 * pow(curr_sample, 2) +
        pp_seg.c3 * pow(curr_sample, 3) + pp_seg.c4 * pow(curr_sample, 4) +
        pp_seg.c5 * pow(curr_sample, 5);

    TrajectoryPointGMP pp_point = {curr_sample, curr_lat, -1.0,
                                   -1.0,        -1.0,     -1.0};

    gmp_path.trajectory.emplace_back(pp_point);
    MSD_LOG(INFO, "Last Point %.2f, %.2f", pp_point.x, pp_point.y);
    curr_sample += sample_step;
  }
  // MSD_LOG(INFO,"Last Point %.2f, %.2f", gmp_path.trajectory.back().x,
  // gmp_path.trajectory.back().y);
  return gmp_path;
}

double
GeneralMotionPlanner::get_path_point(const int &order,
                                     const vector<PathSegment> &path_segments,
                                     const double &curr_sample) {

  auto cmp = [](const double &b, const PathSegment &a) { return b <= a.y_end; };

  if (path_segments.empty()) {
    return 0.0;
  }

  if (path_segments.empty()) {
    return 0.0;
  }

  double breakpoint_ini_lon = path_segments.front().y_end;
  double breakpoint_end_lon = path_segments.back().y_end;
  double curr_lat = 0.0;

  if (curr_sample > breakpoint_end_lon) {
    return curr_lat;
  }
  auto p_seg_index = std::upper_bound(path_segments.begin(),
                                      path_segments.end(), curr_sample, cmp);

  PathSegment pp_seg = *p_seg_index;
  if (order == 0) {
    curr_lat =
        pp_seg.c0 + pp_seg.c1 * curr_sample + pp_seg.c2 * pow(curr_sample, 2) +
        pp_seg.c3 * pow(curr_sample, 3) + pp_seg.c4 * pow(curr_sample, 4) +
        pp_seg.c5 * pow(curr_sample, 5);
  } else if (order == 1) {
    curr_lat = pp_seg.c1 + 2 * pp_seg.c2 * curr_sample +
               3 * pp_seg.c3 * pow(curr_sample, 2) +
               4 * pp_seg.c4 * pow(curr_sample, 3) +
               5 * pp_seg.c5 * pow(curr_sample, 4);
  } else if (order == 2) {
    curr_lat = 2 * pp_seg.c2 + 6 * pp_seg.c3 * curr_sample +
               12 * pp_seg.c4 * pow(curr_sample, 2) +
               20 * pp_seg.c5 * pow(curr_sample, 3);
  }

  return curr_lat;
}

bool GeneralMotionPlanner::path_planning(
    const BehaviorCandidate &behavior_candidate,
    BehaviorAdvice &behavior_advice) {
  const auto &target_points = behavior_candidate.target_points;
  if (target_points.empty() || target_points.size() != 4) {
    MSD_LOG(INFO, "GMP pp fail[%.2f]:tp_num not 4",
            MTIME()->timestamp().sec() - gmp_created_time_);
    return false;
  }
  if (target_points[1].x < 0.0) {
    MSD_LOG(INFO, "GMP pp fail[%.2f]:2nd tp_x negative %.2f",
            MTIME()->timestamp().sec() - gmp_created_time_, target_points[1].x);
    return false;
  }

  double veh_speed_target = std::max(target_points[1].vel, 0.0);
  behavior_advice.veh_speed_end = veh_speed_target;
  behavior_advice.behavior_type = target_points[1].behavior_type;
  behavior_advice.motion_type = get_motion_type(behavior_candidate);
  behavior_advice.behavior_candidate_index = behavior_candidate.behavior_index;
  behavior_advice.behavior_info = behavior_candidate.behavior_info;

  gmp_interface::TargetPoint tp_start{0.0, 0.0, 0.0, 0.0, 0.0, 0};

  for (const auto &target_point : target_points) {
    if (target_point.x < 0) {
      break;
    }
    behavior_advice.path_segments.push_back(
        plan_path_segment(tp_start, target_point));
    tp_start = target_point;
  }

  if (behavior_advice.path_segments.empty()) {
    return false;
  }

  cruise_speed_planning(veh_speed_target, behavior_advice);

  extend_st_segment(behavior_advice);
  get_path_point(behavior_advice);
  add_ego_setspeed_virtual_obj(behavior_advice);
  MSD_LOG(INFO, "GMP pp succeed[%.2f]",
          MTIME()->timestamp().sec() - gmp_created_time_);
  return true;
}

void GeneralMotionPlanner::update_max_speed(void) {
  const double &set_speed =
      general_motion_planner_input_.ego_state_cart.ego_v_cruise;
  const double &ego_speed = general_motion_planner_input_.ego_state.ego_vel;
  const double &target_speed_lc_wait_max =
      general_motion_planner_input_.target_speed_lc_wait_max;

  double v_curv = world_model_->get_v_curv();
  double a_curv = world_model_->get_a_curv();

  if (set_speed - v_curv > 0.1 && v_curv < ego_speed) {
    v_max_ = min(set_speed + 2.6, ego_speed + a_curv * 0.1);
  } else {
    v_max_ = set_speed + 2.6;
  }

  v_max_ = min(v_max_, target_speed_lc_wait_max - 0.5);

  MSD_LOG(INFO,
          "gmp curv_speed: v_curv: %f, a_curv: %f, v_max_: %f, "
          "set_speed: %f, ego_speed: %f",
          v_curv, a_curv, v_max_, set_speed, ego_speed);
}

void GeneralMotionPlanner::add_ego_setspeed_virtual_obj(
    const BehaviorAdvice &behavior) {
  using gmp_interface::ObsInfo;
  vector<ObsInfo> &concerned_objs = general_motion_planner_input_.obj_infos;
  if (!concerned_objs.empty()) {
    if (concerned_objs.back().noticed_timer >= 14.0) {
      concerned_objs.pop_back();
    }
  }
  if (behavior.path_segments.empty()) {
    return;
  }

  ObsInfo virtual_obj{};
  virtual_obj.id = 0;

  virtual_obj.obj_size.length = 5.0;
  virtual_obj.obj_size.width = 2.0;
  virtual_obj.noticed_timer = 15.0;

  double v_max = v_max_;
  double s0_bias = 3.0;
  double virtual_s0 = 0.5 * virtual_obj.obj_size.length + s0_bias;

  ObjPredSlice virtual_obj_slice_default{};

  virtual_obj_slice_default.lane_assignment = 1;

  virtual_obj_slice_default.obj_state_local.accel = 0.0;
  virtual_obj_slice_default.obj_state_local.vel = v_max;
  virtual_obj_slice_default.obj_state_local.pos.x = virtual_s0;
  virtual_obj_slice_default.obj_state_local.pos.y =
      get_path_point(0, behavior.path_segments, virtual_s0);
  virtual_obj_slice_default.obj_state_local.heading =
      get_path_point(1, behavior.path_segments, virtual_s0);

  virtual_obj_slice_default.safety_margin.lateral = 5.0;
  virtual_obj_slice_default.safety_margin.longitu =
      virtual_obj.obj_size.length + 1.5;

  virtual_obj.obj_state_init = virtual_obj_slice_default;

  virtual_obj.pred_trajectory.clear();
  for (int t_index = 0; t_index < 100; ++t_index) {
    double s_virtual = virtual_s0 + (t_index + 1) * 0.1 * v_max;

    ObjPredSlice obj_state_tx = virtual_obj_slice_default;

    obj_state_tx.obj_state_local.pos.x = s_virtual;
    obj_state_tx.obj_state_local.pos.y =
        get_path_point(0, behavior.path_segments, s_virtual);
    obj_state_tx.obj_state_local.heading =
        get_path_point(1, behavior.path_segments, s_virtual);
    if (t_index <= 14) {
      obj_state_tx.safety_margin.longitu = virtual_obj.obj_size.length;
    }
    virtual_obj.pred_trajectory.emplace_back(obj_state_tx);
  }
  concerned_objs.emplace_back(virtual_obj);

  return;
}

void GeneralMotionPlanner::warm_start_speed(BehaviorAdvice &behavior) {
  if (!warm_start_usable_) {
    return;
  }
  const int &task_type = behavior.behavior_type;
  const int &motion_type = behavior.motion_type;
  const int &path_choice = behavior.behavior_candidate_index;
  const auto &behavior_candidates =
      general_motion_planner_input_.behavior_candidates;

  if (path_choice >= static_cast<int>(behavior_candidates.size())) {
    return;
  }
  if (behavior_candidates[path_choice].target_points.size() <= 1) {
    return;
  }

  const double &target_speed =
      behavior_candidates[path_choice].target_points[1].vel;

  if (behavior_cache_.speed_segments.empty()) {
    return;
  }
  if (lc_action_state_buffer_.back() == -1 &&
      lc_action_state_buffer_.front() == -1 && motion_type == 5) {
    // about to back
    return;
  }

  const double &ego_speed = general_motion_planner_input_.ego_state.ego_vel;
  const double &ego_acc = general_motion_planner_input_.ego_state_cart.ego_acc;

  bool condition1 =
      (motion_type == 2 && abs(ego_speed - target_speed) <= 0.5) &&
      task_type > 1 && lc_proceed_time_ >= 0.2;

  bool condition0 = (motion_type == 3 || motion_type == 5) && task_type > 1 &&
                    lc_proceed_time_ >= 0.2;

  bool condition2 = motion_type == 5 && task_type > 1 && lc_proceed_time_ < 0.1;

  if (!(condition1 || condition2 || condition0)) {
    return;
  }

  STSample st_sample_t0{0.0, 0.0, ego_speed, ego_acc}; // s t v a
  vector<STSample> st_breakpoints{};
  st_breakpoints.emplace_back(st_sample_t0);

  for (const auto &st_seg : behavior_cache_.speed_segments) {
    double s_temp = st_seg.x_end;
    double t_temp = st_seg.y_end;
    double v_temp = get_path_point(1, behavior_cache_.speed_segments, t_temp);
    double a_temp = get_path_point(2, behavior_cache_.speed_segments, t_temp);
    STSample st_bkp_temp{s_temp, t_temp, v_temp, a_temp};
    st_breakpoints.emplace_back(st_bkp_temp);
  }

  behavior.speed_segments.clear();
  for (int i = 1; i < st_breakpoints.size(); ++i) {
    PathSegment st_seg1_update =
        plan_path_segment(st_breakpoints[i - 1], st_breakpoints[i]);
    double st_last_t = st_seg1_update.y_end;

    st_breakpoints[i].s = st_seg1_update.x_end;
    st_breakpoints[i].t = st_seg1_update.y_end;
    st_breakpoints[i].v = st_seg1_update.c1 +
                          2 * st_seg1_update.c2 * st_last_t +
                          3 * st_seg1_update.c3 * pow(st_last_t, 2) +
                          4 * st_seg1_update.c4 * pow(st_last_t, 3) +
                          5 * st_seg1_update.c5 * pow(st_last_t, 4);

    st_breakpoints[i].a = 2 * st_seg1_update.c2 +
                          6 * st_seg1_update.c3 * st_last_t +
                          12 * st_seg1_update.c4 * pow(st_last_t, 2) +
                          20 * st_seg1_update.c5 * pow(st_last_t, 3);
    behavior.speed_segments.emplace_back(st_seg1_update);
  }
}

void GeneralMotionPlanner::cruise_speed_planning(const double &set_speed,
                                                 BehaviorAdvice &behavior) {
  double veh_speed = general_motion_planner_input_.ego_state.ego_vel;
  double speed_error = set_speed - veh_speed;
  double t_achieve = 0.1;
  double t_output = 0.0;
  double Ax = 0.0;
  double s_end = 0.0;
  vector<double> speed_c(6, 0.0);
  bool is_in_lc = lc_proceed_time_ >= 0.1;
  bool is_decelerate = speed_error < 0.0;

  if (abs(speed_error) >= 0.2) {
    t_output = 0.3;

    if (abs(speed_error) < 14.0) {
      t_achieve = 1.25 * sqrt(abs(speed_error));
    } else {
      t_achieve = 0.0239 * pow(speed_error, 2);
    }

    if (is_in_lc && is_decelerate) {
      t_achieve = max(t_achieve, 3.0);
    } else {
      t_achieve = max(t_achieve, 1.0);
    }
    t_achieve = min(t_achieve, 7.0);

    double ax_ini = 0.0;

    double C0_h = ax_ini;
    double temp1 = -C0_h / t_achieve;
    double temp2 = 3 * (speed_error / pow(t_achieve, 2) - C0_h / t_achieve);
    double C1_h = 2 * (temp2 - temp1);
    double C2_h = (temp1 - C1_h) / t_achieve;

    Ax = C0_h + C1_h * t_output + C2_h * pow(t_output, 2);

    speed_c[1] = veh_speed;
    speed_c[2] = 0.5 * C0_h;
    speed_c[3] = C1_h / 6;
    speed_c[4] = C2_h / 12;

    s_end = speed_c[0] + speed_c[1] * t_achieve +
            speed_c[2] * pow(t_achieve, 2) + speed_c[3] * pow(t_achieve, 3) +
            speed_c[4] * pow(t_achieve, 4);
  } else {
    speed_c[1] = veh_speed;
    t_achieve = 10;
    s_end = 10 * veh_speed;
  }

  MSD_LOG(INFO, "GMP_cruise_control: %.2f, %.2f", set_speed, veh_speed);

  PathSegment st_segment0{speed_c[0], speed_c[1], speed_c[2], speed_c[3],
                          speed_c[4], 0.0,        s_end,      t_achieve};

  behavior.speed_segments.emplace_back(st_segment0);
  warm_start_speed(behavior);

  return;
}

void GeneralMotionPlanner::extend_st_segment(BehaviorAdvice &behavior) {
  if (behavior.speed_segments.empty() || behavior.path_segments.empty()) {
    return;
  }
  if (behavior.speed_segments.size() >= 5) {
    return;
  }
  double t_achieve = behavior.speed_segments.back().y_end;
  double s_end = behavior.speed_segments.back().x_end;
  double pp_end_y = behavior.path_segments.back().y_end;
  double t_end = 10.0;
  double v_end = get_path_point(1, behavior.speed_segments, t_achieve);

  if (s_end < pp_end_y) {
    t_end = t_achieve + (pp_end_y - s_end) / v_end;
    double st_temp1 = s_end - t_achieve * v_end;
    PathSegment st_seg_temp{st_temp1, v_end, 0.0,      0.0,
                            0.0,      0.0,   pp_end_y, t_end};
    behavior.speed_segments.push_back(st_seg_temp);
  }
  return;
}

void GeneralMotionPlanner::update_dodge_strategy(
    DodgeStrategy &dodge_strategy) {
  vector<double> st_s_offset_bestshot = {0.0, 0.0, 0.0, 0.0};
  double obj_safety_margin_b = dodge_strategy.obj_safety_margin;
  double obj_safety_margin_o = dodge_strategy.collision_obj_length;
  const int &collision_posture = dodge_strategy.collision_posture;
  const int &collision_id_cci = dodge_strategy.collision_id_cci;
  double collision_obj_y1 = dodge_strategy.collision_obj_pos.x;
  double collision_pos_y1 = dodge_strategy.collision_ego_pos.x;
  const double &t_col1 = dodge_strategy.t_col;
  const double &ego_speed = general_motion_planner_input_.ego_state.ego_vel;
  const int &task_type = dodge_strategy.behavior_type;

  double headaway = general_motion_planner_input_.headaway; // default 1.6
  double dheadaway = general_motion_planner_input_.dheadaway;
  bool is_static_obj =
      find(obj_static_id_caches_.begin(), obj_static_id_caches_.end(),
           collision_id_cci) != obj_static_id_caches_.end();
  if (collision_posture == 1) {
    st_s_offset_bestshot[0] = max(
        obj_safety_margin_b - (collision_pos_y1 - collision_obj_y1) + 0.1, 0.1);
    st_s_offset_bestshot[1] = max(
        obj_safety_margin_b - (collision_pos_y1 - collision_obj_y1) + 0.5, 0.1);
    st_s_offset_bestshot[2] = max(
        obj_safety_margin_b - (collision_pos_y1 - collision_obj_y1) + 1.0, 0.1);
    st_s_offset_bestshot[3] = max(
        obj_safety_margin_b - (collision_pos_y1 - collision_obj_y1) + 2.0, 0.1);

  } else if (collision_posture == 2) {
    double t_col1_thres = 3.0;
    double obj_safety_margin_temp = 12.0;
    if (collision_id_cci != -1 || t_col1 > t_col1_thres || is_static_obj) {
      obj_safety_margin_temp = obj_safety_margin_b;
    } else {
      obj_safety_margin_temp =
          obj_safety_margin_o + min((headaway - dheadaway), 0.7) * ego_speed;
      if (task_type > 1 && obj_safety_margin_temp > obj_safety_margin_b) {
        obj_safety_margin_temp = min(obj_safety_margin_temp, 22.5);
      }
      obj_safety_margin_temp = min(obj_safety_margin_temp, obj_safety_margin_b);
    }

    if (collision_id_cci != -1 || is_static_obj) {
      if (t_col1 <= 0.5) {
        st_s_offset_bestshot[0] =
            (collision_obj_y1 - collision_pos_y1) - obj_safety_margin_temp;
      } else {
        st_s_offset_bestshot[0] = (collision_obj_y1 - collision_pos_y1) -
                                  obj_safety_margin_temp - 0.4;
      }
      st_s_offset_bestshot[1] =
          (collision_obj_y1 - collision_pos_y1) - obj_safety_margin_temp - 1.0;

    } else {
      st_s_offset_bestshot[0] =
          (collision_obj_y1 - collision_pos_y1) - obj_safety_margin_temp - 0.2;
      st_s_offset_bestshot[1] =
          (collision_obj_y1 - collision_pos_y1) - obj_safety_margin_temp - 1.0;
    }
    st_s_offset_bestshot[2] =
        (collision_obj_y1 - collision_pos_y1) - obj_safety_margin_temp - 2.0;
    st_s_offset_bestshot[3] =
        (collision_obj_y1 - collision_pos_y1) - obj_safety_margin_temp - 3.0;

  } else if (collision_posture == 3) {
    st_s_offset_bestshot[0] =
        (collision_obj_y1 - collision_pos_y1) - obj_safety_margin_b - 0.2;
    st_s_offset_bestshot[1] =
        (collision_obj_y1 - collision_pos_y1) - obj_safety_margin_b - 1.0;
    st_s_offset_bestshot[2] =
        obj_safety_margin_b - (collision_pos_y1 - collision_obj_y1) + 1.0;
    st_s_offset_bestshot[3] =
        obj_safety_margin_b - (collision_pos_y1 - collision_obj_y1) + 0.2;
  }

  dodge_strategy.st_s_offset_bestshot = st_s_offset_bestshot;
  return;
}

int GeneralMotionPlanner::check_current_process(
    const vector<int> &search_progress) {
  int current_process = -1;

  int st_segnum_max = search_progress.size(); // default is 2

  if (search_progress.back() >= 0) {
    return st_segnum_max - 1;
  }

  int search_layer_o = search_progress.front();
  for (int i = 1; i < search_progress.size(); ++i) {
    int search_layer = search_progress[i];
    if (search_layer_o != -1 && search_layer == -1) {
      current_process = i - 1;
      break;
    }
    search_layer_o = search_progress[i];
  }
  //[-1,-1] -> -1; [1,-1] -> 0; [1,1] -> 1;
  return current_process;
}

bool GeneralMotionPlanner::is_speed_unreachable(const BehaviorAdvice &behavior,
                                                const int &current_step) {
  st_speed_limit_ = ConfigurationContext::Instance()
            ->planner_config()
            .lateral_behavior_planner_config.st_speed_limit;
  if (lc_proceed_time_ > 0.001) {
    accel_limit_control_ = 3.0;
    decel_limit_control_ = -3.5;
  } else {
    accel_limit_control_ = 2.0;
    decel_limit_control_ = -3.0;
  }

  if (behavior.st_array.v.size() < 55) {
    return true;
  }

  double min_speed = *min_element(behavior.st_array.v.begin(),
                                  behavior.st_array.v.begin() + 70);

  double max_speed = *max_element(behavior.st_array.v.begin(),
                                  behavior.st_array.v.begin() + 70);

  double init_speed = behavior.st_array.v.front();

  double accel2control = behavior.st_array.a[2];

  double min_accel = *min_element(behavior.st_array.a.begin(),
                                  behavior.st_array.a.begin() + 50);

  double max_accel = *max_element(behavior.st_array.a.begin(),
                                  behavior.st_array.a.begin() + 70);

  double decel_vel = behavior.st_array.v.front() - behavior.st_array.v[30];
  double accel_vel = behavior.st_array.v[30] - behavior.st_array.v.front();

  const int motion_type = behavior.motion_type;
  const double ego_vel = general_motion_planner_input_.ego_state.ego_vel;
  bool need_restrain_acc = current_step != -1;
  bool need_restrain_dec = !(current_step == -1 || lc_proceed_time_ >= 0.1 ||
                             (ego_vel < 10.0 && min_speed < 1.0));

  bool speed_unreachable =
      (min_speed < -2.0 && task_type_ == 1) ||
      (min_speed < -1.0 && task_type_ > 1 && motion_type >= 1 &&
       motion_type <= 3) ||
      (min_speed < -2.0 && task_type_ > 1 && motion_type >= 5) ||
      init_speed < ego_vel - 2.0 || max_speed > st_speed_limit_ ||
      min_accel <= decel_limit_ || max_accel > accel_limit_ ||
      accel2control > accel_limit_control_ ||
      accel2control < decel_limit_control_ ||
      (decel_vel > 7.5 && need_restrain_dec) ||
      (accel_vel > 7.5 && need_restrain_acc);

  MSD_LOG(INFO,
          "GMP is_speed_unreachable[%.1f]: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, "
          "%.2f,%d",
          MTIME()->timestamp().sec() - gmp_created_time_, min_speed, max_speed,
          accel2control, min_accel, max_accel, decel_vel, accel_vel,
          speed_unreachable);
  MSD_LOG(INFO,
          "GMP is_speed_unreachable2[%.1f]: %.2f, %.2f, %.2f, %.2f, %.2f, "
          "%.2f, %.2f, %.2f",
          MTIME()->timestamp().sec() - gmp_created_time_, init_speed, ego_vel,
          st_speed_limit_, decel_limit_, accel_limit_, accel_limit_control_,
          decel_limit_control_, lc_to_static_obj_time_);
  return speed_unreachable;
}

void GeneralMotionPlanner::backtrack_speed_segment(
    BehaviorAdvice &behavior, const vector<int> &search_progress) {
  for (int i = 0; i < search_progress.size(); ++i) {
    if (search_progress[i] == -1) { // backtrack
      while (i < behavior.speed_segments.size()) {
        behavior.speed_segments.pop_back();
      }
      break;
    }
  }
}

int GeneralMotionPlanner::update_search_progress(const int &next_step) {
  return 0;
}

double GeneralMotionPlanner::interpolate_s_v_point(
    const vector<PathSegment> &st_result, const double &s_point) {

  const auto &planning_init_point =
      baseline_info_->get_ego_state().planning_init_point;
  const double ego_speed = general_motion_planner_input_.ego_state.ego_vel;

  double s_curr = 0.0;
  double s_last = 0.0;
  double v_curr = 0.0;
  double v_last = ego_speed;
  double s0 = planning_init_point.path_point.s;
  double s2ego = s_point - s0;

  for (int i = 0; i < general_motion_planner_output_.t_array.size(); ++i) {
    double t_curr = general_motion_planner_output_.t_array[i];
    s_curr = get_path_point(0, st_result, t_curr);
    v_curr = get_path_point(1, st_result, t_curr);
    if (s2ego <= s_curr) {
      break;
    }
    s_last = s_curr;
    v_last = v_curr;
  }

  double v_point = 0.0;
  if (abs(s_curr - s_last) < 0.0001) {
    return v_point;
  } else {
    v_point = max(
        (s2ego - s_curr) / (s_curr - s_last) * (v_curr - v_last) + v_curr, 0.0);
  }

  return v_point;
}

vector<SVPoint> GeneralMotionPlanner::generate_sv_array(
    const vector<vector<PathSegment>> &st_results,
    const BehaviorAdvice &behavior) {
  vector<SVPoint> s_v_array;
  const double &target_speed_lc_wait_max =
      general_motion_planner_input_.target_speed_lc_wait_max;
  planner_spline::Index<speed_planner::NUM_SPEED_CONTROL_POINTS> index;
  double v_min = 0.0;
  double v_max = 0.0;
  double v_ref = 0.0;

  while (index.advance()) {
    const double s_point =
        s_at_quad_points_[index.segment_index][index.quad_index];

    auto &st_result = behavior.speed_segments;
    double v_ref = interpolate_s_v_point(st_result, s_point);
    v_max = max(v_ref, v_max);
    v_max = min(v_max, target_speed_lc_wait_max);
    SVPoint s_v_point{s_point, v_min, v_max, v_ref};
    s_v_array.push_back(s_v_point);
  }

  return s_v_array;
}

vector<PathSegment> GeneralMotionPlanner::evaluate_speed_segments(
    const vector<vector<PathSegment>> &st_results) {
  auto cmp = [](const STContainer &a, const STContainer &b) {
    return a.cost < b.cost;
  };
  vector<STContainer> st_container;
  const double &detaT = 0.1;

  for (auto st_result : st_results) {
    STContainer st_element{200.0, st_result};
    double cost = 200.0;
    double t_end = st_result.back().y_end;
    double s_end = st_result.back().x_end;
    double v_average = s_end / t_end;

    double v = 0.0, a = 0.0;
    double accel_max = 0.0;
    double decel_max = 0.0;
    double a_sum_abs = 0.0;

    for (const auto &t : general_motion_planner_output_.t_array) {
      if (t <= t_end) {
        v = get_path_point(1, st_result, t);
        a = get_path_point(2, st_result, t);
      } else {
        a = 0.0;
      }
      a_sum_abs += abs(a);

      if (a >= 0.0 && a > accel_max) {
        accel_max = a;
      } else if (a < 0.0 && a < decel_max) {
        decel_max = a;
      }
    }
    double a_abs_average = a_sum_abs / t_end;
    double J1 = 1 / (v_average + 0.01);
    double J2 = a_abs_average;
    double J3 = accel_max;
    double J4 = abs(decel_max);

    if (decel_max >= -9.0) {
      cost = 40.0 * J1 + 0.5 * J2 + 0.2 * J3 + 0.2 * J4;
    } else {
      cost = 200.0;
    }

    st_element.cost = cost;
    st_container.emplace_back(st_element);
  }
  sort(st_container.begin(), st_container.end(), cmp);

  return st_container.front().speed_segments;
}

bool GeneralMotionPlanner::speed_planning(BehaviorAdvice &behavior) {
  using gmp_interface::ObsInfo;
  using gmp_interface::Point2d;

  static constexpr int st_bkpoint_max = 2;
  static constexpr int candidate_num = 8;
  // max breakpoint (bkpoint) num in speed_segments{}, 3 segments has 2 bkpoints

  int iteration_num = 0;
  vector<DodgeStrategy> dodge_strategies(st_bkpoint_max);
  vector<int> obj_cci_id_cache_{};

  const STSample &st_sample_default{0.0, 0.0, 0.0, 0.0};
  vector<STSample> st_sample_cache(st_bkpoint_max, st_sample_default);

  vector<int> search_progress(st_bkpoint_max, -1);
  vector<int> search_progress_o(st_bkpoint_max, -1);

  int search_step = 1;
  bool st_search_evasive = false;
  int next_step = -1;

  Point2D col_pos_ego{500, 0};
  Point2D col_pos_obj{500, 0};

  vector<ObsInfo> &concerned_objs = general_motion_planner_input_.obj_infos;

  int motion_type = behavior.motion_type;

  DodgeStrategy dodge_strategy;

  const DodgeStrategy &dodge_strategy_default{
      0,
      0,
      -1,
      col_pos_ego,
      col_pos_obj,
      60.0,
      {0.0, 0.0, 0.0, 0.0},
      15.0,
      7.0,
      0.0,
      0.0, // collision_obj_vel, end_accel
      -1,
      behavior.behavior_type,
      behavior.motion_type};

  double task_time_remain = behavior.behavior_info.lc_time_remain;
  double task_time_pass = behavior.behavior_info.lc_time_pass;

  vector<vector<PathSegment>> st_results{};
  vector<PathSegment> speed_segments = behavior.speed_segments;

  while (iteration_num < 64) {
    MSD_LOG(INFO, "GMP st_evasive %d", st_search_evasive);
    search_progress_o = search_progress;
    //[-1,-1] -> -1; [1,-1] -> 0; [1,1] -> 1;
    int current_step = check_current_process(search_progress);
    double t_col_o = -2.0;

    if (current_step != -1) {
      t_col_o = dodge_strategies[current_step].t_col;
    }
    bool speed_unreachable = is_speed_unreachable(behavior, current_step);
    if (!speed_unreachable) {
      if (!st_search_evasive) {
        dodge_strategy = collision_detect_dyn(behavior, concerned_objs);
      }
    } else {
      dodge_strategy = dodge_strategy_default;
    }

    bool to_rear_car_safe_enough = false;

    if (dodge_strategy.collision_id != -1) {
      int col_index = floor(dodge_strategy.t_col * 10.0);
      double col_ttc = abs(
          (dodge_strategy.collision_ego_pos.x -
           dodge_strategy.collision_obj_pos.x) /
          (behavior.st_array.v[col_index] - dodge_strategy.collision_obj_vel));
      const auto &col_obj = concerned_objs[dodge_strategy.collision_id];
      const bool &hit_rear_obj = dodge_strategy.collision_ego_pos.x -
                                     dodge_strategy.collision_obj_pos.x >
                                 5.0;
      const bool &almost_collision_free =
          (dodge_strategy.collision_ego_pos.x -
           dodge_strategy.collision_obj_pos.x) >=
          (dodge_strategy.obj_safety_margin * 0.8);

      const bool hit_after_lc = dodge_strategy.t_col >= task_time_remain - 1.0;
      to_rear_car_safe_enough =
          hit_rear_obj && ((col_ttc >= 4.5 && almost_collision_free &&
                            lc_proceed_time_ > 1.5 && motion_type == 3) ||
                           hit_after_lc);
    } else {
      to_rear_car_safe_enough = true;
    }

    bool current_collision_free =
        (dodge_strategy.collision_id == -1 || (to_rear_car_safe_enough)) &&
        (!speed_unreachable) &&
        (!st_search_evasive); // TODO check whether correct

    bool new_seg_invalid =
        (dodge_strategy.t_col < t_col_o + 0.6 &&
         current_step < st_bkpoint_max - 1 && !st_search_evasive) &&
        (current_step == 0);

    bool search_next_level =
        dodge_strategy.collision_id > -1 && current_step < st_bkpoint_max - 1 &&
        (!speed_unreachable) && (!new_seg_invalid) && (!st_search_evasive);

    if (current_collision_free) {
      MSD_LOG(INFO,
              "GMP t_cols2[%.1fs][%d][%d,%.1f] %.1f custp:%d %d %d fre:%d "
              "ureac: %d",
              MTIME()->timestamp().sec() - gmp_created_time_, iteration_num,
              dodge_strategy.collision_id, dodge_strategy.t_col, t_col_o,
              current_step, search_next_level, st_search_evasive,
              current_collision_free, speed_unreachable);
    }

    MSD_LOG(
        INFO,
        "GMP t_cols[%.1fs][%d][%d,%.1f] %.1f custp:%d %d %d fre:%d ureac: %d",
        MTIME()->timestamp().sec() - gmp_created_time_, iteration_num,
        dodge_strategy.collision_id, dodge_strategy.t_col, t_col_o,
        current_step, search_next_level, st_search_evasive,
        current_collision_free, speed_unreachable);

    if (current_collision_free) {
      next_step = current_step;
      const int speed_choice_max = 20;
      if (st_results.size() > speed_choice_max) {
        MSD_LOG(INFO, "GMP st_results size is :%d", st_results.size());
        break;
      }
      st_results.emplace_back(behavior.speed_segments);
      if (current_step == -1) {
        break;
      }
    } else if (search_next_level) {
      next_step = current_step + 1; // current_step = 0, switch to 1
      dodge_strategies[next_step] = dodge_strategy;
    } else {
      next_step = current_step;
    }

    // ----------- searching progress update ----------- //
    if (next_step < 0)
      break; // maybe unnecessary
    int temp1 = search_progress[next_step];
    if (temp1 >= candidate_num - 1 && next_step == 0) {
      break; // end searching .
    } else {
      search_progress[next_step] = temp1 + search_step;
      for (int tempi = next_step; tempi >= 1; tempi--) {
        if (search_progress[tempi] >= candidate_num) {
          search_progress[tempi] -= (candidate_num + 1);
          search_progress[tempi - 1] += 1;

          new_seg_invalid = false;
        }
      }
      if (search_progress[0] >= candidate_num) {
        break;
      }
    }
    MSD_LOG(INFO, "GMP searching : %d,%d,%d", current_collision_free,
            search_next_level, new_seg_invalid);
    MSD_LOG(INFO, "GMP searching progress1[%d] %d %d", iteration_num,
            search_progress[0], search_progress[1]);

    // ----------- searching progress update ----------- //

    // ----------------------- SegEnd_Point update -----------------------//
    int search_origin_index =
        get_search_origin(search_progress_o, search_progress);
    int parallel_search_index_temp = search_progress[search_origin_index];
    int parallel_search_index =
        round((parallel_search_index_temp + 1) / 2.0) - 1;
    int target_speed_search_index = round(
        1.0 - (2.0 * parallel_search_index - parallel_search_index_temp + 1));
    MSD_LOG(INFO, "GMP searching progress2[%d]:%d %d %d %d evasive:%d",
            iteration_num, search_origin_index, parallel_search_index_temp,
            parallel_search_index, target_speed_search_index,
            st_search_evasive);

    dodge_strategy = dodge_strategies[search_origin_index];
    // ----------------------- SegEnd_Point update -----------------------//

    update_st_limit(search_origin_index, behavior, dodge_strategy);

    //-------------- special scenario handling --------------//

    if (parallel_search_index == 0 && target_speed_search_index == 0 &&
        obj_cci_id_cache_.empty()) {
      negotiate_safety_margin(dodge_strategy, behavior, search_origin_index);
      if (dodge_strategy.collision_id_cci != -1) {
        obj_cci_id_cache_.emplace_back(dodge_strategy.collision_id_cci);
        dodge_strategies[search_origin_index] = dodge_strategy;
      }
    }
    //-------------- special scenario handling --------------//

    update_dodge_strategy(dodge_strategy);
    dodge_strategies[search_origin_index] = dodge_strategy;

    // ------ Evade the curent cycle or not ------ //
    update_st_evasive(dodge_strategy, search_origin_index,
                      parallel_search_index, new_seg_invalid, st_search_evasive,
                      search_step);
    if (st_search_evasive) {
      iteration_num++;
      backtrack_speed_segment(behavior, search_progress);
      continue; // no st_segment_update
    }
    // ------ Evade the curent cycle or not  ------ //

    const double &set_speed =
        general_motion_planner_input_.ego_state_cart.ego_v_cruise;
    const double &ego_speed = general_motion_planner_input_.ego_state.ego_vel;
    const int &behavior_index = behavior.behavior_candidate_index;
    if (motion_type == 1 && search_origin_index == 0) {
      if (!(ego_speed > set_speed && behavior_index == 1)) {
        // skip speed_adjust_update if motionchoice 1 is faster than set_speed
        update_retreat_speed_adjustment(dodge_strategy);
      }
    }

    double st_ay_ini = cal_inspired_acceleration(dodge_strategy, behavior,
                                                 search_origin_index);

    update_speed_segment(behavior, dodge_strategies, st_sample_cache,
                         search_progress, search_origin_index,
                         parallel_search_index, target_speed_search_index,
                         st_ay_ini, search_step);

    get_path_point(behavior);
    iteration_num++;
  } // end of searching loop

  if (!obj_cci_id_cache_.empty()) {
    general_motion_planner_input_.obj_infos[obj_cci_id_cache_.front()] =
        obj_cci_cache_;
  }

  MSD_LOG(INFO, "GMP speed choice[%.1f]: %d",
          MTIME()->timestamp().sec() - gmp_created_time_, st_results.size());

  if (st_results.size() == 0) {
    MSD_LOG(INFO, "GMP sp failed[%.1f]",
            MTIME()->timestamp().sec() - gmp_created_time_);

    return false;
  } else {

    MSD_LOG(INFO, "GMP sp succeed[%.1f]-iteration[%d]",
            MTIME()->timestamp().sec() - gmp_created_time_, iteration_num);
    // behavior.speed_segments = st_results.back();
    behavior.speed_segments = evaluate_speed_segments(st_results);
    behavior.s_v = generate_sv_array(st_results, behavior);
    get_path_point(behavior);
  }

  MSD_LOG(INFO, "GMP st_candidate retreat adj[%.1f] %d",
          MTIME()->timestamp().sec() - gmp_created_time_,
          retreat_speed_adjustment_);

  return true;
}

void GeneralMotionPlanner::negotiate_safety_margin(
    DodgeStrategy &dodge_strategy, const BehaviorAdvice &behavior,
    const int &search_origin_index) {

  const double &ego_speed = general_motion_planner_input_.ego_state.ego_vel;
  const int &task_type = behavior.behavior_type;
  const int &motion_type = behavior.motion_type;
  const int &path_choice = behavior.behavior_candidate_index;
  const int &collision_posture = dodge_strategy.collision_posture;

  double collision_pos_y = dodge_strategy.collision_ego_pos.x;
  double collision_obj_y = dodge_strategy.collision_obj_pos.x;

  const int &task_type_o = task_type_buffer_.back();
  const int &motion_result_o = motion_result_buffer_.back();

  double t_col = dodge_strategy.t_col;

  double task_time_remain = behavior.behavior_info.lc_time_remain;
  const int &col_id_relative = dodge_strategy.collision_id;

  auto &col_obj = general_motion_planner_input_.obj_infos[col_id_relative];
  double obj_center_lat = col_obj.obj_state_init.obj_state_local.pos.y;
  const double &obj_length = col_obj.obj_size.length;

  double obj_safety_margin_end =
      col_obj.pred_trajectory.back().safety_margin.longitu;
  double obj_safety_margin_ini =
      col_obj.pred_trajectory.front().safety_margin.longitu;

  //  ------------------------------- //
  const double &veh_temp_dis = col_obj.obj_state_init.obj_state_local.pos.x;
  const double &veh_temp_speed = dodge_strategy.collision_obj_vel;
  const double &v_timer_temp = col_obj.noticed_timer;
  int collision_cci_id = 0;

  bool condition1 =
      (collision_posture == 2 || (task_type == 1 && collision_posture == 3 &&
                                  collision_obj_y >= 3.0 + collision_pos_y)) &&
      v_timer_temp <= 5.0 &&
      (veh_temp_dis <= obj_safety_margin_end - 1.0 ||
       (veh_temp_speed <= ego_speed - 10.0 && task_type == 6)) &&
      (task_type == 1 || task_type == 6 ||
       (task_type != 1 && motion_type == 5 ||
        (motion_type == 1 || motion_type == 3) && task_time_remain <= 3.5));

  bool cut_tail_in_lc =
      collision_posture == 2 && veh_temp_speed >= ego_speed - 1.5 &&
      veh_temp_dis <= obj_safety_margin_end - 1.0 &&
      veh_temp_dis > obj_safety_margin_ini &&
      (task_type != 1 && (motion_type == 1 || motion_type == 3) &&
       task_time_remain >= 1.5);
  // cut_tail_in_lc = false;

  bool cut_head_in_lc =
      collision_posture == 1 && obj_length < 7.0 &&
      ((abs(veh_temp_dis) < abs(obj_safety_margin_end - 1.0) &&
        task_type != 1 && task_type_o != 1 &&
        (veh_temp_speed < ego_speed && lc_proceed_time_ < 0.1 ||
         veh_temp_speed < ego_speed + 1.0 && lc_proceed_time_ >= 0.1) &&
        veh_temp_dis < -10.0) ||
       v_timer_temp < lc_proceed_time_ - 0.1) &&
      veh_temp_dis < -7.0 &&
      (task_type != 1 &&
       ((motion_type == 1 || motion_type == 3) && task_time_remain >= 1.5) &&
       lc_proceed_time_ <= 4.5);

  double static_front_obj_dist_thres_max =
      (ego_speed * ego_speed - veh_temp_speed * veh_temp_speed) / 5.0;

  double static_front_obj_dist_thres_min =
      (ego_speed * ego_speed - veh_temp_speed * veh_temp_speed) / 6.0;

  if (lc_to_static_obj_ == false) {
    lc_to_static_obj_ =
        collision_posture == 2 && task_type > 1 &&
        ego_speed > veh_temp_speed + 1.0 &&
        (motion_type >= 1 && motion_type <= 4) && veh_temp_speed < 6.0 &&
        veh_temp_dis - 10.0 >= static_front_obj_dist_thres_max &&
        veh_temp_dis > 10.0;
  } else if (lc_to_static_obj_ == true &&
             (task_type == 1 ||
              veh_temp_dis <= static_front_obj_dist_thres_min ||
              collision_posture != 2 || ego_speed < veh_temp_speed ||
              veh_temp_speed > 8.0 || (motion_type == 5 or motion_type == 6))) {
    lc_to_static_obj_ = false;
  }

  MSD_LOG(INFO,
          "GMP lc_to_static_obj flag: %d,%d,%d,%.2f,%.2f,%d,%.2f,%.2f,%.2f",
          lc_to_static_obj_, collision_posture, task_type, ego_speed,
          veh_temp_speed, motion_type, veh_temp_dis,
          static_front_obj_dist_thres_max, static_front_obj_dist_thres_min);

  if (t_col <= 3.6 && condition1) {
    lc_to_static_obj_time_ = 0.0;
    dodge_strategy.collision_id_cci = col_id_relative;
    double safety_margin_gap = 0.5 * pow(t_col, 2);
    double sm_lon_temp = 7.0;
    if (t_col <= 3.3) {
      sm_lon_temp = collision_obj_y - t_col * ego_speed + 0.3 * v_timer_temp;
    } else {
      sm_lon_temp = collision_obj_y - t_col * ego_speed + safety_margin_gap;
    }
    dodge_strategy.obj_safety_margin = max(sm_lon_temp, obj_safety_margin_ini);

  } else if (t_col <= 3.6 && cut_tail_in_lc) {
    lc_to_static_obj_time_ = 0.0;
    dodge_strategy.collision_id_cci = col_id_relative;
    double safety_margin_gap = 10.0;

    if (t_col >= 2.0) {
      safety_margin_gap = 0.8 * pow(t_col, 2) - 1.2;
    } else {
      safety_margin_gap = 0.5 * pow(t_col, 2);
    }
    double sm_lon_temp =
        collision_obj_y - t_col * ego_speed + safety_margin_gap;
    if (sm_lon_temp >= obj_safety_margin_end) {
      dodge_strategy.collision_id_cci = -1;
    } else {
      dodge_strategy.obj_safety_margin = sm_lon_temp;
    }
    dodge_strategy.obj_safety_margin =
        max(dodge_strategy.obj_safety_margin, obj_safety_margin_ini);

  } else if (t_col <= 3.6 && cut_head_in_lc) {
    lc_to_static_obj_time_ = 0.0;
    dodge_strategy.collision_id_cci = col_id_relative;
    double safety_margin_gap = 0.4 * pow(t_col, 2);
    double sm_lon_temp =
        t_col * ego_speed - collision_obj_y + safety_margin_gap - 0.8;
    if (sm_lon_temp >= obj_safety_margin_end ||
        sm_lon_temp <= obj_safety_margin_ini + 0.3) {
      dodge_strategy.collision_id_cci = -1;
    } else {
      dodge_strategy.obj_safety_margin = sm_lon_temp;
    }

    if (veh_temp_speed >= ego_speed + t_col * 1.3 && search_origin_index == 0) {
      double t_col_temp = (veh_temp_speed - ego_speed) / 1.3;
      t_col = min(t_col_temp, 10.0);
      int traj_index = floor(10.0 * t_col) - 1;

      dodge_strategy.t_col = t_col;
      dodge_strategy.collision_obj_pos.x =
          col_obj.pred_trajectory[traj_index].obj_state_local.pos.x;
      dodge_strategy.collision_ego_pos.x = ego_speed * t_col;
    }
  } else if (task_type != 1 && task_type_o != 1 && motion_result_o == 1 &&
             collision_posture == 1) {
    lc_to_static_obj_time_ = 0.0;
    if (veh_temp_speed >= ego_speed + t_col * 2.0 && search_origin_index == 0) {
      double t_col_temp = (veh_temp_speed - ego_speed) / 2.0;
      t_col = min(t_col_temp, 10.0);
      int traj_index = floor(10.0 * t_col) - 1;

      dodge_strategy.t_col = t_col;
      dodge_strategy.collision_obj_pos.x =
          col_obj.pred_trajectory[traj_index].obj_state_local.pos.x;
      dodge_strategy.collision_ego_pos.x = ego_speed * t_col;

      return;
    }
  } else if (lc_to_static_obj_) {
    dodge_strategy.collision_id_cci = col_id_relative;
    dodge_strategy.obj_safety_margin = obj_safety_margin_ini;
    // force shrink sm of static objs, each behavior can shrink one obj on top.
    // This will effects all behaviors.
    for (auto &pred_slice :
         general_motion_planner_input_.obj_infos[col_id_relative]
             .pred_trajectory) {
      pred_slice.safety_margin.longitu = dodge_strategy.obj_safety_margin;
    }
    int traj_index = floor(10.0 * t_col) - 1;
    double obj_x = col_obj.pred_trajectory[traj_index].obj_state_local.pos.x;
    double obj_heading =
        col_obj.pred_trajectory[traj_index].obj_state_local.heading;
    double obj_x_head = obj_x + 0.5 * obj_length * cos(obj_heading);
    double obj_sm_lower = obj_x_head - dodge_strategy.obj_safety_margin - 10.0;
    if (lc_to_static_obj_time_ - 0.0 < 0.001) {
      lc_to_static_obj_time_ = max(3.8, obj_sm_lower / ego_speed);
      lc_to_static_obj_time_ = min(4.5, lc_to_static_obj_time_);
    }

    double obj_speed = col_obj.pred_trajectory[traj_index].obj_state_local.vel;

    double t_ideal = max((ego_speed - obj_speed) / 2.0, 0.1);
    MSD_LOG(INFO, "GMP lc_to_static_obj [%.1f], %.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
            MTIME()->timestamp().sec() - gmp_created_time_, ego_speed,
            obj_speed, t_ideal, t_col, lc_to_static_obj_time_, obj_sm_lower);

    t_ideal = max(t_ideal, t_col);
    dodge_strategy.t_col = min(t_ideal, 9.0);

    int traj_index_new = floor(10.0 * dodge_strategy.t_col) - 1;

    dodge_strategy.collision_obj_pos.x =
        col_obj.pred_trajectory[traj_index_new].obj_state_local.pos.x +
        0.5 * obj_length;
    dodge_strategy.collision_ego_pos.x =
        max(dodge_strategy.collision_obj_pos.x - obj_length, 0.0);

    if (obj_static_id_caches_.empty() ||
        find(obj_static_id_caches_.begin(), obj_static_id_caches_.end(),
             col_id_relative) == obj_static_id_caches_.end()) {
      obj_static_id_caches_.emplace_back(col_id_relative);
    }

    MSD_LOG(INFO, "GMP lc_to_static_obj [%.1f], %d",
            MTIME()->timestamp().sec() - gmp_created_time_, col_id_relative);

  } else {
    lc_to_static_obj_time_ = 0.0;
    return;
  }
  update_concerned_obj_safety_margin(dodge_strategy);
  return;
}

void GeneralMotionPlanner::update_concerned_obj_safety_margin(
    const DodgeStrategy &dodge_strategy) {
  using gmp_interface::Point2d;
  using gmp_interface::SafetyMargin;

  const int &cci_id = dodge_strategy.collision_id_cci;

  if (cci_id == -1) {
    return;
  }

  auto &obj = general_motion_planner_input_.obj_infos[cci_id];
  obj_cci_cache_ = obj;

  int t_index = 0;

  while (t_index < obj.pred_trajectory.size()) {
    // double time = general_motion_planner_output_.t_array[t_index];

    double safety_margin_lon_new = dodge_strategy.obj_safety_margin;

    if (obj.pred_trajectory[t_index].safety_margin.longitu >
        safety_margin_lon_new) {
      obj.pred_trajectory[t_index].safety_margin.longitu =
          safety_margin_lon_new;
    }
    t_index++;
  }
}

void GeneralMotionPlanner::update_obj_safety_margin_display(
    const int &behavior_idx, const int &obj_index, const int &t_index,
    const double &new_sm_lon, const double &new_sm_lat) {
  if (obj_index >= gmp_objs_sm_buffer_[behavior_idx].all_objs_pred_sm.size()) {
    return;
  }
  gmp_objs_sm_buffer_[behavior_idx]
      .all_objs_pred_sm[obj_index]
      .pred_sm[t_index]
      .longitu = new_sm_lon;
  gmp_objs_sm_buffer_[behavior_idx]
      .all_objs_pred_sm[obj_index]
      .pred_sm[t_index]
      .lateral = new_sm_lat;
}

void GeneralMotionPlanner::update_obj_safety_margin_display(
    const BehaviorAdvice &behavior, const DodgeStrategy &dodge_strategy) {
  using gmp_interface::Point2d;
  using gmp_interface::SafetyMargin;

  const int &behavior_candidate_idx = behavior.behavior_candidate_index;
  const int &cci_id = dodge_strategy.collision_id_cci;
  if (cci_id == -1) {
    return;
  }

  auto &obj = general_motion_planner_input_.obj_infos[cci_id];
  int t_index = 0;
  if (cci_id >=
      gmp_objs_sm_buffer_[behavior_candidate_idx].all_objs_pred_sm.size()) {
    return;
  }
  while (t_index < obj.pred_trajectory.size()) {
    double safety_margin_lon_new = dodge_strategy.obj_safety_margin;

    if (obj.pred_trajectory[t_index].safety_margin.longitu >
        safety_margin_lon_new) {
      gmp_objs_sm_buffer_[behavior_candidate_idx]
          .all_objs_pred_sm[cci_id]
          .pred_sm[t_index]
          .longitu = safety_margin_lon_new;
    }
    t_index++;
  }
}

void GeneralMotionPlanner::update_objs_display_sm() {
  gmp_interface::AllObjsMarginInfo all_objs_sm;

  for (auto &obj : general_motion_planner_input_.obj_infos) {
    gmp_interface::ObjMarginInfo obj_pred_sm;

    for (auto &objSlice : obj.pred_trajectory) {
      obj_pred_sm.pred_sm.emplace_back(objSlice.safety_margin);
    }

    all_objs_sm.all_objs_pred_sm.emplace_back(obj_pred_sm);
  }

  const int candidate_num = 6;
  gmp_objs_sm_buffer_.clear();
  for (int i = 0; i < candidate_num; ++i) {
    gmp_objs_sm_buffer_.emplace_back(all_objs_sm);
  }
}

double GeneralMotionPlanner::cal_inspired_acceleration(
    const DodgeStrategy &dodge_strategy, const BehaviorAdvice &behavior,
    const int &search_origin_index) {
  double st_ay_ini = 0.0;
  const int &collision_posture = dodge_strategy.collision_posture;
  const double &t_col = dodge_strategy.t_col;
  const double &collision_obj_vel = dodge_strategy.collision_obj_vel;
  const double &ego_speed = general_motion_planner_input_.ego_state.ego_vel;
  const int &task_type = behavior.behavior_type;
  const int &motion_type = behavior.motion_type;
  const double &collision_pos_y = dodge_strategy.collision_ego_pos.x;

  if (collision_posture == 2 && ego_speed >= collision_obj_vel - 1.0 &&
      t_col <= 6.0 && search_origin_index == 0) {

    double st_ay_ini_thres = -2.5;
    double headaway = general_motion_planner_input_.headaway;
    double temp1 = collision_obj_vel - min(20.0, (headaway * ego_speed + 7.0));

    temp1 = max(1.0, temp1);
    st_ay_ini = -pow((ego_speed - collision_obj_vel), 2) / 2 / temp1;

    if (ego_speed >= 10.0) {
      st_ay_ini_thres = -2.5;
    } else if (ego_speed >= 10.0) {
      st_ay_ini_thres = -1.5;
    } else {
      st_ay_ini_thres = -(0.2 * ego_speed + 0.5);
    }
    st_ay_ini = max(st_ay_ini, st_ay_ini_thres);
  } else if (collision_posture == 1 && (task_type != 1 && motion_type != 5) &&
             search_origin_index == 0) {
    double temp1 = collision_pos_y + 7.0;
    st_ay_ini = 2 * (temp1 - ego_speed * t_col) / pow(t_col, 2);
    st_ay_ini = max(0.0, st_ay_ini);
    st_ay_ini = min(st_ay_ini, 0.8);
  } else {
    st_ay_ini = 0.0;
  }

  return st_ay_ini;
}

void GeneralMotionPlanner::update_speed_segment(
    BehaviorAdvice &behavior, vector<DodgeStrategy> &dodge_strategies,
    vector<STSample> &st_sample_cache, const vector<int> &search_progress,
    const int &search_origin_index, const int &parallel_search_index,
    const int &target_speed_search_index, double st_ay_ini, int &search_step) {

  Point2D col_pos_ego{500, 0};
  Point2D col_pos_obj{500, 0};
  const DodgeStrategy &dodge_strategy_default{
      0,
      0,
      -1,
      col_pos_ego,
      col_pos_obj,
      60.0,
      {0.0, 0.0, 0.0, 0.0},
      15.0,
      7.0,
      0.0,
      0.0, // collision_obj_vel, end_accel
      -1,
      behavior.behavior_type,
      behavior.motion_type};
  const STSample &st_sample_default{0.0, 0.0, 0.0, 0.0};

  const double &ego_speed = general_motion_planner_input_.ego_state.ego_vel;
  const double &ego_acc = general_motion_planner_input_.ego_state_cart.ego_acc;

  DodgeStrategy dodge_strategy = dodge_strategies[search_origin_index];

  STSample st_sample_end{0.0, 0.0, 0.0, 0.0};
  STSample st_sample_o{0.0, 0.0, 0.0, 0.0};
  STSample st_target{0.0, 0.0, 0.0, 0.0};
  PathSegment st_segment_default{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  double veh_speed_target = behavior.veh_speed_end;
  double collision_obj_vel = max(dodge_strategy.collision_obj_vel, 0.0);
  double st_evasion_bias =
      dodge_strategy.st_s_offset_bestshot[parallel_search_index];

  st_sample_end.s = dodge_strategy.collision_ego_pos.x + st_evasion_bias;

  st_sample_end.t = dodge_strategy.t_col;

  MSD_LOG(INFO, "GMP st_breakpoint0[%.1f]: %.2f, %.2f", st_sample_end.t,
          dodge_strategy.collision_ego_pos.x, st_evasion_bias);

  vector<double> st_seg1end_vels = {collision_obj_vel, veh_speed_target};
  MSD_LOG(INFO, "GMP st_breakpoint0 %.2f, %.2f", collision_obj_vel,
          veh_speed_target);
  if (search_origin_index > 0) {
    search_step = 2;
  } else {
    search_step = 1;
  }

  if (search_origin_index < 1) {
    st_sample_o.v = ego_speed;
    st_sample_o.a = ego_acc;
    // st_sample_o.a = min(st_a_buffer_.back(),1.5);
    // st_sample_o.a = max(st_sample_o.a,-2.5);

    st_sample_o.s = 0.0;
    st_sample_o.t = 0.0;
  } else {
    st_sample_o = st_sample_cache[search_origin_index - 1];
  }

  double st_seg1end_vel = 0.0;
  if (st_evasion_bias > 0.0) {
    st_seg1end_vel = 2.0 * (st_sample_end.s - st_sample_o.s) /
                         (st_sample_end.t - st_sample_o.t) -
                     st_sample_o.v;
    if (search_origin_index == 0) {
      st_seg1end_vel = max(st_seg1end_vel, collision_obj_vel);
    }
  } else {
    double st_vel_temp1 = max(2.0 * (st_sample_end.s - st_sample_o.s) /
                                      (st_sample_end.t - st_sample_o.t) -
                                  st_sample_o.v,
                              0.0);
    st_seg1end_vel = max(collision_obj_vel, 0.0);
    st_seg1end_vel = min(st_seg1end_vel, st_vel_temp1);
  }
  st_sample_end.v = st_seg1end_vel;
  st_sample_end.a = 0.0;

  st_sample_cache[search_origin_index] = st_sample_end;

  for (int i = 0; i < search_progress.size(); ++i) {
    if (search_progress[i] == -1) { // backtrack
      st_sample_cache[i] = st_sample_default;
      dodge_strategies[i] = dodge_strategy_default;
      while (i < behavior.speed_segments.size()) {
        behavior.speed_segments.pop_back();
      }
    }
  }

  st_sample_end = st_sample_cache[search_origin_index];
  // MSD_LOG(INFO,"GMP JustCheck3!");
  PathSegment st_seg1_update = plan_path_segment(st_sample_o, st_sample_end);

  st_sample_end.s = st_seg1_update.x_end;
  st_sample_end.t = st_seg1_update.y_end;
  st_sample_end.v = st_seg1_update.c1 +
                    2 * st_seg1_update.c2 * st_sample_end.t +
                    3 * st_seg1_update.c3 * pow(st_sample_end.t, 2) +
                    4 * st_seg1_update.c4 * pow(st_sample_end.t, 3) +
                    5 * st_seg1_update.c5 * pow(st_sample_end.t, 4);

  st_sample_end.a = 2 * st_seg1_update.c2 +
                    6 * st_seg1_update.c3 * st_sample_end.t +
                    12 * st_seg1_update.c4 * pow(st_sample_end.t, 2) +
                    20 * st_seg1_update.c5 * pow(st_sample_end.t, 3);

  st_sample_cache[search_origin_index] = st_sample_end;

  double veh_speed_end = st_seg1end_vels[target_speed_search_index];
  double pp_end_y = behavior.path_segments.back().y_end;

  if (abs(st_sample_end.v + veh_speed_end) >= 0.2 && veh_speed_end > 0.2) {
    st_target.t =
        2 * (pp_end_y - st_sample_end.s) / (st_sample_end.v + veh_speed_end) +
        st_sample_end.t;
    st_target.s = pp_end_y;
  } else {
    MSD_LOG(INFO, "GMP end_speed: %.2f, %.2f, %.2f", st_sample_end.s,
            st_sample_end.v, st_sample_end.t);
    MSD_LOG(INFO, "GMP end_speed: %.2f, %.2f, %.2f", st_sample_o.s,
            st_sample_o.v, st_sample_o.t);
    st_target.t = 10.0;
    st_target.s = st_sample_end.s;
  }
  st_target.v = veh_speed_end;
  st_target.a = 0.0;

  PathSegment st_seg2_update = plan_path_segment(st_sample_end, st_target);
  st_seg2_update.x_end = st_target.s;
  st_seg2_update.y_end = st_target.t;

  if (search_origin_index < behavior.speed_segments.size()) {
    behavior.speed_segments[search_origin_index] = st_seg1_update;
  } else {
    behavior.speed_segments.push_back(st_seg1_update);
  }

  if (search_origin_index + 1 < behavior.speed_segments.size()) {
    behavior.speed_segments[search_origin_index + 1] = st_seg2_update;
  } else {
    behavior.speed_segments.push_back(st_seg2_update);
  }
}

PathSegment GeneralMotionPlanner::plan_path_segment(const STSample &st_start,
                                                    const STSample &st_end) {

  PathSegment st_seg{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  double x0 = st_start.t, y0 = st_start.s;
  double xt = st_end.t, yt = st_end.s;

  double xt_re = xt - x0;

  double C0 = y0;
  double C1 = st_start.v;
  double C2 = st_start.a / 2.0;

  double b0 = yt - C0 - C1 * xt_re - C2 * xt_re * xt_re;
  double b1 = st_end.v - C1 - 2.0 * C2 * xt_re;
  double b2 = st_end.a - 2.0 * C2;

  double a00 = pow(xt_re, 3);
  double a01 = pow(xt_re, 4);
  double a10 = 3.0 * xt_re * xt_re;
  double a11 = 4.0 * pow(xt_re, 3);
  double a20 = 6.0 * xt_re;
  double a21 = 12.0 * xt_re * xt_re;

  Eigen::MatrixXd A(2, 11);
  Eigen::MatrixXd B(1, 11);
  A << a00, a10, a10, a10, a10, a10, a10, a10, a20, a20, a20, a01, a11, a11,
      a11, a11, a11, a11, a11, a21, a21, a21;
  B << b0, b1, b1, b1, b1, b1, b1, b1, b2, b2, b2;

  Eigen::MatrixXd C = (A * A.transpose()).inverse() * A * (B.transpose());

  // Eigen:: MatrixXd C = (A.transpose()*A).inverse()*A.transpose()*B;

  double C3 = C(0, 0);
  double C4 = C(1, 0);

  st_seg.c0 = C0 - C1 * x0 + C2 * x0 * x0 - C3 * x0 * x0 * x0 + C4 * pow(x0, 4);
  st_seg.c1 = C1 - 2 * C2 * x0 + 3 * C3 * x0 * x0 - 4 * C4 * x0 * x0 * x0;
  st_seg.c2 = C2 - 3 * C3 * x0 + 6 * C4 * x0 * x0;
  st_seg.c3 = C3 - 4 * C4 * x0;
  st_seg.c4 = C4;
  st_seg.c5 = 0.0;

  // st_seg.x_end = st_end.s;
  st_seg.y_end = st_end.t;

  st_seg.x_end = st_seg.c0 + st_seg.c1 * xt + st_seg.c2 * xt * xt +
                 st_seg.c3 * pow(xt, 3) + st_seg.c4 * pow(xt, 4) +
                 st_seg.c5 * pow(xt, 5);

  return st_seg;
}

void GeneralMotionPlanner::update_retreat_speed_adjustment(
    const double &st_ay_o, const double &st_ay) {
  const double &ego_speed = general_motion_planner_input_.ego_state.ego_vel;
  if (st_ay_o < st_ay) {
    retreat_speed_adjustment_ = 1; // accelerate
  } else if (st_ay_o > st_ay) {
    retreat_speed_adjustment_ = 2; // decelerate
  } else {
    return;
  }
}

void GeneralMotionPlanner::update_retreat_speed_adjustment(
    const DodgeStrategy &dodge_strategy) {
  const int collision_obj_index = dodge_strategy.collision_id;
  if (collision_obj_index < 0 ||
      collision_obj_index >=
          static_cast<int>(general_motion_planner_input_.obj_infos.size())) {
    return;
  }
  const auto &collision_obj =
      general_motion_planner_input_.obj_infos[collision_obj_index];
  const double collision_obj_center_x =
      collision_obj.obj_state_init.obj_state_local.pos.x;
  const double collision_obj_length = collision_obj.obj_size.length;
  const double collision_obj_heading =
      collision_obj.obj_state_init.obj_state_local.heading;

  const int collision_posture = dodge_strategy.collision_posture;
  const double collision_obj_vel = dodge_strategy.collision_obj_vel;
  const double ego_vel = general_motion_planner_input_.ego_state.ego_vel;
  const double collision_time = dodge_strategy.t_col;

  if (collision_posture == 1 && collision_obj_vel < v_max_) {
    retreat_speed_adjustment_ = 1;
  } else if (collision_posture == 2 ||
             (collision_posture == 1 && collision_obj_vel >= v_max_)) {
    double sug_v_delta_origin = ego_vel - collision_obj_vel;
    double sug_v_delta_end = sug_v_delta_origin - 4.0;
    double sug_dist = (sug_v_delta_origin + sug_v_delta_end) / 2.0 * 8.0;
    const double collision_obj_front_x =
        collision_obj_center_x +
        0.5 * collision_obj_length * std::cos(collision_obj_heading);
    if (sug_dist > collision_obj_front_x && sug_v_delta_end > 0.0) {
      retreat_speed_adjustment_ = 1;
    } else {
      retreat_speed_adjustment_ = 2;
    }
  } else {
    double temp1 = dodge_strategy.st_s_offset_bestshot[1];
    double temp2 = dodge_strategy.st_s_offset_bestshot[2];
    if ((temp2 > -temp1 + 2.0 ||
         (temp2 >= -temp1 && ego_vel <= collision_obj_vel) ||
         collision_time <= 0.5) &&
        (ego_vel <= collision_obj_vel + 4.0)) {
      retreat_speed_adjustment_ = 2;
    } else {
      retreat_speed_adjustment_ = 1;
    }
  }

  if (retreat_speed_adjustment_ == 2) {
    const bool lane_cross =
        general_motion_planner_input_.lc_task_info.lane_cross;
    const bool is_rear_collision_obj = collision_obj_center_x <= -7.5;
    const bool is_collision_obj_virtual = collision_obj.noticed_timer >= 14.0;
    if (is_collision_obj_virtual) {
      retreat_speed_adjustment_ = 1;
    } else if (lane_cross && is_rear_collision_obj) {
      retreat_speed_adjustment_ = 1;
    } else {
      const int index = floor(collision_time * 10.0);
      const int max_index =
          static_cast<int>(collision_obj.pred_trajectory.size()) - 1;
      if (index >= 0 && index <= max_index) {
        const bool obj_future_on_target_lane =
            collision_obj.pred_trajectory[index].lane_assignment == task_type_;
        const bool obj_now_behind_ego = collision_obj_center_x < 0.0;
        if (obj_future_on_target_lane && obj_now_behind_ego) {
          retreat_speed_adjustment_ = 0; // set_speed
        }
      }
    }
  }
}

void GeneralMotionPlanner::update_st_evasive(
    const DodgeStrategy &dodge_strategy, const int &search_origin_index,
    const int &parallel_search_index, const bool &newseg_invalid,
    bool &st_search_evasive, int &search_step) {

  st_search_evasive = false;
  if (dodge_strategy.st_s_offset_bestshot[parallel_search_index] == 0.0) {
    st_search_evasive = true;
    search_step = 1;
  }
}

void GeneralMotionPlanner::update_st_limit(
    const int &search_origin_index, const BehaviorAdvice &behavior,
    const DodgeStrategy &dodge_strategy) {
  const int &path_choice = behavior.behavior_candidate_index;
  const int &task_type = behavior.behavior_type;
  const int &motion_type = behavior.motion_type;
  const double &t_col = dodge_strategy.t_col;
  const double &task_time_remain = behavior.behavior_info.lc_time_remain;
  const double &task_time_pass = behavior.behavior_info.lc_time_pass;

  if (search_origin_index == 0) {
    if (task_type != 1) {
      if (t_col <= task_time_remain || path_choice == 1) {
        decel_limit_ = -5.5;
      } else {
        decel_limit_ = -11.0;
      }
    } else {
      if (motion_type == 6 && t_col <= 3.6) {
        decel_limit_ = -11.0;
      } else {
        decel_limit_ = -9.0;
      }
    }
  }
  return;
}

int GeneralMotionPlanner::get_search_origin(
    const vector<int> &search_progress_o, const vector<int> &search_progress) {

  int origin_index = 0;
  int st_segnum_max = search_progress.size(); // at least 1
  if (st_segnum_max == 0) {
    return origin_index;
  }
  for (int i = 0; i < st_segnum_max; i++) {
    if (search_progress_o[i] != search_progress[i] || i == st_segnum_max - 1) {
      origin_index = i;
      break;
    }
  }
  return origin_index;
};

DodgeStrategy GeneralMotionPlanner::collision_detect_dyn(
    BehaviorAdvice &behavior, vector<gmp_interface::ObsInfo> &concerned_objs) {
  using gmp_interface::Point2d;

  Point2D col_pos_ego{500, 0};
  Point2D col_pos_obj{500, 0};
  DodgeStrategy dodge_strategy{0,
                               0,
                               -1,
                               col_pos_ego,
                               col_pos_obj,
                               60.0,
                               {0.0, 0.0, 0.0, 0.0},
                               15.0,
                               7.0, // st_offsets, safetymargin, obj_length
                               0.0,
                               0.0, // collision_obj_vel, end_accel
                               -1,
                               behavior.behavior_type,
                               behavior.motion_type};

  if (concerned_objs.empty()) {
    return dodge_strategy;
  }
  unordered_set<int> ignored_objs{};
  const auto &t_array = general_motion_planner_output_.t_array;
  const int max_t_num = static_cast<int>(
      std::min(t_array.size(), behavior.trajectory.trajectory.size()));

  for (int i = 0; i < max_t_num; ++i) {
    for (int j = 0; j < static_cast<int>(concerned_objs.size()); ++j) {

      if (ignored_objs.count(j)) {
        continue;
      }

      if (collision_detect_dyn_1v1(behavior, concerned_objs[j], dodge_strategy,
                                   i, j, ignored_objs)) {
        dodge_strategy.collision_id = j;
        break;
      }
    }

    if (dodge_strategy.collision_id != -1) {
      MSD_LOG(INFO, "GMP CDT_dyn [%.2f]: %.2f, %.2f at %.2fs, posture: %d",
              MTIME()->timestamp().sec() - gmp_created_time_,
              concerned_objs[dodge_strategy.collision_id]
                  .obj_state_init.obj_state_local.pos.x,
              concerned_objs[dodge_strategy.collision_id]
                  .obj_state_init.obj_state_local.pos.y,
              t_array[i], dodge_strategy.collision_posture);
      break;
    }
  }

  return dodge_strategy;
}

int GeneralMotionPlanner::get_motion_type(
    const BehaviorCandidate &behavior_canditate) const {
  const auto type = behavior_canditate.behavior_type;
  const auto index = behavior_canditate.behavior_index;

  int motion_type = 1;
  if (type != LC_Dir::NONE) {
    if (index <= 1) {
      motion_type = 1;
    } else if (index <= 3) {
      motion_type = 2;
    } else if (index == 4) {
      motion_type = 3;
    } else if (index == 5) {
      motion_type = 5;
    }
  } else {
    if (index <= 1) {
      motion_type = 1;
    } else if (index == 2) {
      motion_type = 6;
    }
  }

  return motion_type;
}

bool GeneralMotionPlanner::collision_detect_dyn_1v1(
    const BehaviorAdvice &behavior, const gmp_interface::ObsInfo &obj,
    DodgeStrategy &dodge_strategy, const int &t_index, const int &obj_index,
    unordered_set<int> &ignored_objs) {
  // Todo: include heading in the TrajectoryPointGMP struct.delete the heading
  // input here.
  using gmp_interface::Point2d;

  double task_lon_dist_remain = behavior.behavior_info.lc_dist_remain;
  const double &task_time_remain = behavior.behavior_info.lc_time_remain;
  const double &task_time_total = behavior.behavior_info.lc_time_total;

  int task_type = behavior.behavior_type; // lh:1 lc2l:2 lc:r:3
  int motion_type = behavior.motion_type;
  const int &behavior_candidate_idx = behavior.behavior_candidate_index;

  if (motion_type >= 5 && obj.obj_state_init.obj_state_local.pos.x <= 2.5) {
    return false;
  }

  double sm_lon = obj.pred_trajectory[t_index].safety_margin.longitu;
  double sm_lat = obj.pred_trajectory[t_index].safety_margin.lateral;
  bool is_rear_obj = obj.obj_state_init.obj_state_local.pos.x <= -5.0;
  bool is_front_obj = obj.obj_state_init.obj_state_local.pos.x >= 3.0;
  bool is_virtual_obj = obj.noticed_timer >= 14.0;
  bool obj_to_nextnext_lane = obj.pred_trajectory.back().lane_assignment == -1;
  const double &ego_speed =
      general_motion_planner_input_.ego_state_cart.ego_vel;

  if (lc_proceed_time_ >= 0.1 && !is_virtual_obj) {
    if (is_rear_obj) {
      sm_lon -= 1.5;
    } else if (is_front_obj) {
      sm_lon -= 0.5;
    }
  }

  if (lc_proceed_time_ > 1.6 && is_rear_obj) {
    double proceed_time = min(lc_proceed_time_, 7.0);
    double delta_hw = (0.62 - 0.5) * (proceed_time - 1.6) / (7.0 - 1.6);
    sm_lon -= (delta_hw * ego_speed);
  }

  if (motion_type == 5) {
    sm_lat = 0.5 * obj.obj_size.width + 1.1;
  }

  // if (is_front_obj && !is_virtual_obj && t_index > 14) {
  //   // only necessary for sm_lon after 1.5s
  //   sm_lon = max(sm_lon, 10.0);
  // }

  if (is_rear_obj) {
    sm_lon = max(sm_lon, 8.0);
  }

  // update sm display
  update_obj_safety_margin_display(behavior_candidate_idx, obj_index, t_index,
                                   sm_lon, sm_lat);

  double heading_obj = obj.pred_trajectory[t_index].obj_state_local.heading;
  Point2d obj_point = obj.pred_trajectory[t_index].obj_state_local.pos;
  double obj_len_half = 0.5 * obj.obj_size.length;

  obj_point.x += obj_len_half * cos(heading_obj);
  obj_point.y += obj_len_half * sin(heading_obj);

  const double &obj_speed_ini = obj.obj_state_init.obj_state_local.vel;
  const double &ego_speed_ini =
      general_motion_planner_input_.ego_state_cart.ego_vel;
  const auto &ego_point = behavior.trajectory.trajectory[t_index];
  double e2o_distance = sqrt(pow(ego_point.x - obj_point.x, 2) +
                             pow(ego_point.y - obj_point.y, 2));

  // =============== use obj as base ===================//
  vector<double> o2e_vector{ego_point.x - obj_point.x,
                            ego_point.y - obj_point.y};
  vector<double> obj_heading_vector{cos(heading_obj), sin(heading_obj)};
  double e2o_distance_lon =
      (obj_heading_vector[0] * o2e_vector[0] +
       obj_heading_vector[1] * o2e_vector[1]); // ego front of obj: positive
  double e2o_distance_lat =
      sqrt(max(0.0, pow(e2o_distance, 2) - pow(e2o_distance_lon, 2)));

  bool has_overlap = false;
  bool overlap_ahead_obj =
      (e2o_distance_lon >= 0 && e2o_distance_lon <= sm_lon &&
       e2o_distance_lat <= sm_lat);

  bool overlap_behind_obj =
      (e2o_distance_lon < 0 && e2o_distance_lon >= -sm_lon &&
       e2o_distance_lat <= sm_lat);

  if (overlap_ahead_obj || overlap_behind_obj) {
    double t_col = general_motion_planner_output_.t_array[t_index];

    bool hit_front_half = (e2o_distance_lat <= 0.9 && e2o_distance_lon > 2.7) ||
                          (e2o_distance_lon > 5.0); // assume obj center at 2.5m
    bool hit_rear_half =
        (e2o_distance_lat <= 0.9 && e2o_distance_lon <= -2.0) ||
        (e2o_distance_lon <= -5.0) ||
        (t_col <= 2.2 &&
         (obj_speed_ini - ego_speed_ini) > 3.8); // assume obj center at 2.5m

    MSD_LOG(INFO,
            "GMP inCDT_dyn[%.2f]:e(%.2f, %.2f) o(%.2f,%.2f) %d %d %.1f %.1f "
            "%.1f %.1f %.1f",
            MTIME()->timestamp().sec() - gmp_created_time_, ego_point.y,
            ego_point.x, obj_point.y, obj_point.x, hit_front_half,
            hit_rear_half, t_col, e2o_distance_lat, e2o_distance_lon,
            obj_speed_ini, ego_speed_ini);

    if (hit_front_half) {
      bool hit_after_lc =
          ego_point.x > task_lon_dist_remain && ego_point.x > obj_point.x &&
          task_type > 1 && motion_type <= 3 &&
          motion_type >= 1; // TODO: add motiontype = 6 related here.

      if (t_col > 7 || hit_after_lc) {
        ignored_objs.insert(obj_index);
        return false;
      }
      dodge_strategy.collision_posture = 1; // accelerating only

    } else if (hit_rear_half) {
      if (t_col > 9) {
        return false;
      }
      dodge_strategy.collision_posture = 2; // decelerating only

    } else {
      if (t_col > 7) {
        ignored_objs.insert(obj_index);
        return false;
      }
      dodge_strategy.collision_posture = 3; // accel. and decel.
    }

    dodge_strategy.t_col = t_col;
    dodge_strategy.collision_ego_pos.x = ego_point.x;
    dodge_strategy.collision_ego_pos.y = ego_point.y;
    dodge_strategy.collision_obj_pos.x = obj_point.x;
    dodge_strategy.collision_obj_pos.y = obj_point.y;
    dodge_strategy.collision_obj_length = obj.obj_size.length;
    dodge_strategy.collision_obj_vel =
        obj.pred_trajectory[t_index].obj_state_local.vel;
    dodge_strategy.obj_safety_margin =
        obj.pred_trajectory.back().safety_margin.longitu;
    // TODO Add collisionposture here 2022-05-16.
    return true;
  }
  return false;
}

STArray
GeneralMotionPlanner::generate_st_array(const BehaviorAdvice &behavior) {
  STArray st_array = {{}, {}, {}};
  double s = 0.0, v = 0.0, a = 0.0;
  double detaT = general_motion_planner_output_.t_array[1] -
                 general_motion_planner_output_.t_array[0];

  if (behavior.speed_segments.empty()) {
    return st_array;
  }
  // st_array
  double t_end = behavior.speed_segments.back().y_end;

  for (const auto &t : general_motion_planner_output_.t_array) {
    if (t <= t_end) {
      v = max(0.0, get_path_point(1, behavior.speed_segments, t));
      a = (v < 0.001 && !st_array.s.empty())
              ? 0.0
              : get_path_point(2, behavior.speed_segments, t);
      s = (v < 0.001 && !st_array.s.empty())
              ? st_array.s.back()
              : get_path_point(0, behavior.speed_segments, t);
    } else {
      s += detaT * v;
      a = 0.0;
    }

    st_array.s.emplace_back(s);
    st_array.v.emplace_back(v);
    st_array.a.emplace_back(a);
  }
  return st_array;
}

void GeneralMotionPlanner::update_general_motion_planner_output(
    const BehaviorAdvice &behavior_advice) {
  if (behavior_advice.st_array.s.empty()) {
    return;
  }
  general_motion_planner_output_.speed_array.clear();
  general_motion_planner_output_.acce_array.clear();
  general_motion_planner_output_.speed_segments.clear();

  general_motion_planner_output_.speed_array = behavior_advice.st_array.v;

  general_motion_planner_output_.acce_array = behavior_advice.st_array.a;

  for (const auto &behav_speed_seg : behavior_advice.speed_segments) {
    vector<double> speed_segment = {
        behav_speed_seg.c0,    behav_speed_seg.c1,   behav_speed_seg.c2,
        behav_speed_seg.c3,    behav_speed_seg.c4,   behav_speed_seg.c5,
        behav_speed_seg.x_end, behav_speed_seg.y_end};
    general_motion_planner_output_.speed_segments.emplace_back(speed_segment);
  }
  general_motion_planner_output_.lc_wait_speed_adjust_advice =
      retreat_speed_adjustment_;

  // quad point
  planner_spline::Index<speed_planner::NUM_SPEED_CONTROL_POINTS> index;
  int ii = 0;
  double v_max_temp = behavior_advice.s_v.back().v_max;
  while (index.advance()) {
    general_motion_planner_output_
        .s_v_array_out[index.segment_index][index.quad_index] =
        behavior_advice.s_v[ii];

    general_motion_planner_output_
        .s_v_array_out[index.segment_index][index.quad_index]
        .v_max = v_max_temp;

    MSD_LOG(INFO, "GMP out_s_quad[%d]: %.2f %.2f %.2f %.2f", ii,
            general_motion_planner_output_
                .s_v_array_out[index.segment_index][index.quad_index]
                .s,
            general_motion_planner_output_
                .s_v_array_out[index.segment_index][index.quad_index]
                .v_min,
            general_motion_planner_output_
                .s_v_array_out[index.segment_index][index.quad_index]
                .v_max,
            general_motion_planner_output_
                .s_v_array_out[index.segment_index][index.quad_index]
                .v_ref);
    ii++;
  }
  general_motion_planner_output_.lc_wait_speed_adjust_count =
      retreat_count_max_;
  general_motion_planner_output_.chosen_behavior_candidate_index =
      behavior_advice.behavior_candidate_index;
  general_motion_planner_output_.lc_to_static_obj_time = lc_to_static_obj_time_;
}

void GeneralMotionPlanner::update_task_type(void) {
  const auto &behavior_candidates =
      general_motion_planner_input_.behavior_candidates;
  if (behavior_candidates.empty()) {
    return;
  }
  if (behavior_candidates.front().target_points.empty()) {
    return;
  }
  task_type_ = behavior_candidates.front().behavior_type;
  MSD_LOG(INFO, "GMP task_type[%.1f][%d]",
          MTIME()->timestamp().sec() - gmp_created_time_, task_type_);
}

void GeneralMotionPlanner::update_max_retreat_count(void) {
  retreat_count_max_ = 1;
}

vector<ObjInfoRefined>
GeneralMotionPlanner::extract_lead_objs_info(const BehaviorAdvice &behavior) {
  using gmp_interface::Pose2d;

  Pose2d pos_default{-200.0, 0.0, 0.0};
  ObjInfoRefined obj_info_refined_default{-1, -1, 2.0, pos_default};
  vector<ObjInfoRefined> lead_info_gmp(2, obj_info_refined_default);
  if (behavior.trajectory.trajectory.empty()) {
    return lead_info_gmp;
  }
  const bool &lane_cross = behavior.behavior_info.lane_cross;

  if (task_type_ > 1) {
    if (!lane_cross) {
      lead_info_gmp[0] = general_motion_planner_output_.cipv_info[0];
      lead_info_gmp[1] = general_motion_planner_output_.gap_info[0];
    } else {
      lead_info_gmp[0] = general_motion_planner_output_.gap_info[0];
    }
  } else {
    lead_info_gmp[0] = general_motion_planner_output_.cipv_info[0];
  }
  return lead_info_gmp;
}

vector<ObjInfoRefined>
GeneralMotionPlanner::extract_cipv_info(const BehaviorAdvice &behavior) {
  using gmp_interface::Pose2d;

  Pose2d pos_default{-200.0, 0.0, 0.0};
  ObjInfoRefined obj_info_refined_default{-1, -1, 2.0, pos_default};
  vector<ObjInfoRefined> cipv_info_gmp(2, obj_info_refined_default);

  if (behavior.trajectory.trajectory.empty()) {
    return cipv_info_gmp;
  }
  const double &lc_total = behavior.behavior_info.lc_dist_total;
  const double &lc_past = behavior.behavior_info.lc_dist_pass;
  const bool &lane_cross = behavior.behavior_info.lane_cross;

  double headaway = general_motion_planner_input_.headaway; // default 1.6
  double dheadaway = general_motion_planner_input_.dheadaway;
  double headaway_smoothed = general_motion_planner_input_.headaway_smoothed;
  // double dheadaway =  general_motion_planner_input_.dheadaway;

  double lc_crosslane = lc_total / 2.0;
  int t_check_index = 20;

  if (task_type_ > 1) {
    headaway -= dheadaway;
  } else {
    headaway = headaway_smoothed;
  }

  int t_index_lanecross = behavior.trajectory.trajectory.size() - 1;
  double ego_x_lanecross = behavior.trajectory.trajectory.back().x;

  for (int i = 0; i < behavior.trajectory.trajectory.size(); ++i) {
    if (behavior.trajectory.trajectory[i].x + lc_past >= lc_crosslane - 5.0) {
      t_index_lanecross = i;
      ego_x_lanecross = behavior.trajectory.trajectory[i].x;
      break;
    }
  }

  ego_x_lanecross = behavior.trajectory.trajectory[t_check_index].x;
  double front_obj_min_x = ego_x_lanecross + 500.0;
  ObjInfoRefined obj_info_front_nearest{-1, -1, 2.0, pos_default};

  int id_relative = 0;
  for (const auto &obj : general_motion_planner_input_.obj_infos) {
    if (obj.noticed_timer >= 14.0) {
      id_relative++;
      continue;
    }
    if (obj.pred_trajectory[t_check_index].lane_assignment != 1) {
      id_relative++;
      continue;
    }
    double obj_x = obj.pred_trajectory[t_check_index].obj_state_local.pos.x;
    if (obj_x < front_obj_min_x && obj_x > ego_x_lanecross) {
      front_obj_min_x = obj_x;
      obj_info_front_nearest.id = obj.id;
      obj_info_front_nearest.id_relative = id_relative;
      obj_info_front_nearest.pos_init.x =
          obj.obj_state_init.obj_state_local.pos.x;
      obj_info_front_nearest.pos_init.y =
          obj.obj_state_init.obj_state_local.pos.y;
      obj_info_front_nearest.pos_init.theta =
          obj.obj_state_init.obj_state_local.heading;
    }

    id_relative++;
  }

  double e2o_dist_abs_min = 500.0;
  double e2o_dist_abs = 500.0;
  double obj_vel = 50.0;
  int t_start_index = 0;
  int t_end_index = behavior.trajectory.trajectory.size();

  if (!lane_cross) {
    t_end_index = t_index_lanecross + 1;
  }

  if (obj_info_front_nearest.id_relative != -1) {
    const auto &obj = general_motion_planner_input_
                          .obj_infos[obj_info_front_nearest.id_relative];
    double obj_length = obj.obj_size.length;
    for (int i = t_start_index; i < t_end_index; ++i) {
      e2o_dist_abs = obj.pred_trajectory[i].obj_state_local.pos.x -
                     behavior.trajectory.trajectory[i].x;
      if (e2o_dist_abs < e2o_dist_abs_min) {
        e2o_dist_abs_min = e2o_dist_abs;
        obj_vel = obj.pred_trajectory[i].obj_state_local.vel;
      }
    }
    obj_info_front_nearest.headaway = min(
        (e2o_dist_abs_min - obj_length * 0.5) / max(obj_vel, 0.1), headaway);
  }

  cipv_info_gmp[0] = obj_info_front_nearest;

  MSD_LOG(INFO, "GMP OBJ_INFO:CIPV     [%.1f]:%.1f %.1f, %.2fs",
          MTIME()->timestamp().sec() - gmp_created_time_,
          obj_info_front_nearest.pos_init.y, obj_info_front_nearest.pos_init.x,
          obj_info_front_nearest.headaway);
  return cipv_info_gmp;
}

vector<ObjInfoRefined>
GeneralMotionPlanner::extract_gap_info(const BehaviorAdvice &behavior) {
  using gmp_interface::Pose2d;
  Pose2d pos_default{-200.0, 0.0, 0.0};
  ObjInfoRefined obj_info_refined_default{-1, -1, 2.0, pos_default};
  const double &ego_speed = general_motion_planner_input_.ego_state.ego_vel;

  vector<ObjInfoRefined> gap_info_gmp(2, obj_info_refined_default);
  const int &motion_type = behavior.motion_type;
  if (task_type_ == 1) {
    return gap_info_gmp;
  }
  if (behavior.trajectory.trajectory.empty()) {
    return gap_info_gmp;
  }

  double headaway = general_motion_planner_input_.headaway; // default 1.6
  double dheadaway = general_motion_planner_input_.dheadaway;
  double headaway_smoothed = general_motion_planner_input_.headaway_smoothed;

  const double &lc_total = behavior.behavior_info.lc_dist_total;
  double lc_past = behavior.behavior_info.lc_dist_pass;
  const double &lc_remain = behavior.behavior_info.lc_dist_remain;
  double lc_crosslane = lc_total / 2.0;
  double lc_crosslane_rt = lc_remain / 2.0;

  double gap_front_vel = -500.0;
  double gap_rear_vel = -500.0;
  double gap_target_vel = -500.0;

  if (task_type_ > 1 && motion_type == 5) {
    lc_past = 0.0;
  }
  if (task_type_ > 1) {
    headaway -= dheadaway;
  } else {
    headaway = headaway_smoothed;
  }

  int t_index_lanecross = 55;
  double ego_x_lanecross = 0.0;

  for (int i = 0; i < behavior.trajectory.trajectory.size(); ++i) {
    if (behavior.trajectory.trajectory[i].x + lc_past >=
        lc_crosslane + ego_speed) {
      t_index_lanecross = i;
      ego_x_lanecross = behavior.trajectory.trajectory[i].x;
      break;
    }
  }
  t_index_lanecross = max(t_index_lanecross, 40);
  ego_x_lanecross = behavior.trajectory.trajectory[t_index_lanecross].x;

  double front_obj_min_x = ego_x_lanecross + 500.0;
  ObjInfoRefined obj_info_front_nearest{-1, -1, 2.0, pos_default};

  double rear_obj_max_x = ego_x_lanecross - 500.0;
  ObjInfoRefined obj_info_rear_nearest{-1, -1, 2.0, pos_default};

  int id_relative = 0;
  for (const auto &obj : general_motion_planner_input_.obj_infos) {
    if (obj.noticed_timer >= 14.0) {
      id_relative++;
      continue;
    }
    if (obj.pred_trajectory[t_index_lanecross].lane_assignment != task_type_) {
      id_relative++;
      continue;
    }
    double obj_x = obj.pred_trajectory[t_index_lanecross].obj_state_local.pos.x;
    if (obj_x < front_obj_min_x && obj_x > ego_x_lanecross) {
      front_obj_min_x = obj_x;
      obj_info_front_nearest.id = obj.id;
      obj_info_front_nearest.id_relative = id_relative;
      obj_info_front_nearest.pos_init.x =
          obj.obj_state_init.obj_state_local.pos.x;
      obj_info_front_nearest.pos_init.y =
          obj.obj_state_init.obj_state_local.pos.y;
      obj_info_front_nearest.pos_init.theta =
          obj.obj_state_init.obj_state_local.heading;

    } else if (obj_x > rear_obj_max_x && obj_x < ego_x_lanecross) {
      rear_obj_max_x = obj_x;
      obj_info_rear_nearest.id = obj.id;
      obj_info_rear_nearest.id_relative = id_relative;
      obj_info_rear_nearest.pos_init.x =
          obj.obj_state_init.obj_state_local.pos.x;
      obj_info_rear_nearest.pos_init.y =
          obj.obj_state_init.obj_state_local.pos.y;
      obj_info_rear_nearest.pos_init.theta =
          obj.obj_state_init.obj_state_local.heading;
    }
    id_relative++;
  }
  double e2o_dist_abs_min = 500.0;
  double e2o_dist_abs = 500.0;
  double obj_vel = 50.0;

  if (obj_info_front_nearest.id_relative != -1) {
    const auto &obj = general_motion_planner_input_
                          .obj_infos[obj_info_front_nearest.id_relative];
    double obj_length = obj.obj_size.length;
    for (int i = t_index_lanecross; i < behavior.trajectory.trajectory.size();
         ++i) {
      e2o_dist_abs = obj.pred_trajectory[i].obj_state_local.pos.x -
                     behavior.trajectory.trajectory[i].x;
      if (e2o_dist_abs < e2o_dist_abs_min) {
        e2o_dist_abs_min = e2o_dist_abs;
        obj_vel = obj.pred_trajectory[i].obj_state_local.vel;
      }
    }
    gap_front_vel = obj_vel;
    obj_info_front_nearest.headaway = min(
        (e2o_dist_abs_min - obj_length * 0.5) / max(obj_vel, 0.1), headaway);
  }

  e2o_dist_abs_min = 500.0;
  e2o_dist_abs = 500.0;
  obj_vel = 50.0;

  if (obj_info_rear_nearest.id_relative != -1) {
    const auto &obj = general_motion_planner_input_
                          .obj_infos[obj_info_rear_nearest.id_relative];
    double obj_length = obj.obj_size.length;
    for (int i = t_index_lanecross; i < behavior.trajectory.trajectory.size();
         ++i) {
      e2o_dist_abs = behavior.trajectory.trajectory[i].x -
                     obj.pred_trajectory[i].obj_state_local.pos.x;

      if (e2o_dist_abs < e2o_dist_abs_min) {
        e2o_dist_abs_min = e2o_dist_abs;
        obj_vel = obj.pred_trajectory[i].obj_state_local.vel;
      }
    }
    gap_rear_vel = obj_vel;
    obj_info_rear_nearest.headaway =
        min((e2o_dist_abs_min - 5.0 - obj_length * 0.5) / max(obj_vel, 0.1),
            headaway);
  }

  gap_info_gmp[0] = obj_info_front_nearest;
  gap_info_gmp[1] = obj_info_rear_nearest;
  MSD_LOG(INFO, "GMP OBJ_INFO:gap front[%.1f]:%.1f %.1f, %.2fs, %d %.1f",
          MTIME()->timestamp().sec() - gmp_created_time_,
          obj_info_front_nearest.pos_init.y, obj_info_front_nearest.pos_init.x,
          obj_info_front_nearest.headaway, t_index_lanecross, lc_crosslane_rt);
  MSD_LOG(INFO, "GMP OBJ_INFO:gap rear [%.1f]:%.1f %.1f, %.2fs",
          MTIME()->timestamp().sec() - gmp_created_time_,
          obj_info_rear_nearest.pos_init.y, obj_info_rear_nearest.pos_init.x,
          obj_info_rear_nearest.headaway);

  if (gap_front_vel > 0.0 && gap_rear_vel > 0.0) {
    gap_target_vel = 0.7 * gap_front_vel + 0.3 * gap_rear_vel;
  } else if (gap_front_vel > 0.0) {
    gap_target_vel = gap_front_vel;
  } else if (gap_rear_vel > 0.0) {
    gap_target_vel = gap_rear_vel;
  }

  MSD_LOG(INFO, "GMP OBJ_INFO:gap  vel [%.1f]:%.1f %.1f %.1f",
          MTIME()->timestamp().sec() - gmp_created_time_, gap_front_vel,
          gap_front_vel, gap_target_vel);

  return gap_info_gmp;
}

bool GeneralMotionPlanner::is_rear_wild(const int &id_rel) {

  if (id_rel <= -1) {
    return false;
  }
  const double &ego_speed = general_motion_planner_input_.ego_state.ego_vel;
  const auto &rear_obj = general_motion_planner_input_.obj_infos[id_rel];
  const double &rear_length = rear_obj.obj_size.length;

  int t_check_index = 35;
  const double &rear_obj_vel_check =
      rear_obj.pred_trajectory[t_check_index].obj_state_local.vel;

  const double &rear_obj_dist = rear_obj.obj_state_init.obj_state_local.pos.x;
  if (rear_obj_vel_check >= v_max_) {
    return true;
  }
  if (rear_obj_vel_check <= ego_speed + 4.0 || rear_obj_dist >= -8) {
    return false;
  }
  double headaway_smoothed = general_motion_planner_input_.headaway_smoothed;

  double a_max = 1.0;
  double A = a_max * 0.5;
  double B = ego_speed - rear_obj_vel_check;
  double C = abs(rear_obj_dist);

  double pre_min_dist = 30.0;

  double t_symm = -B * 0.5 / A;

  double t_check2 = 5.0;
  double t_check1 = 2.6;
  double dist2 = A * t_check2 * t_check2 + B * t_check2 + C;
  double dist1 = A * t_check1 * t_check1 + B * t_check1 + C;

  if (t_check2 >= t_symm && t_check1 < t_symm) {
    pre_min_dist = (4 * A * C - B * B) * 0.25 / A;
  } else if (t_symm > t_check2) {
    pre_min_dist = dist2;
  } else {
    pre_min_dist = dist1;
  }

  if (pre_min_dist <
      min(headaway_smoothed * rear_obj_vel_check + rear_length, 25.0)) {
    return true;
  }

  return false;
}

BehaviorAdvice GeneralMotionPlanner::evaluate_behavior(
    vector<BehaviorAdvice> &behavior_container) {

  double st_ay_o = st_a_buffer_.back();
  const int &motion_result_o = motion_result_buffer_.back();
  const int &task_type_o = task_type_buffer_.back();
  const double &ego_speed = general_motion_planner_input_.ego_state.ego_vel;
  const double &target_speed_lc_wait_max =
      general_motion_planner_input_.target_speed_lc_wait_max;
  const double &lc_remain_time =
      general_motion_planner_input_.lc_task_info.lc_time_remain;
  const double &lc_total_time =
      general_motion_planner_input_.lc_task_info.lc_time_total;
  const double t_confirm = 0.4;

  for (auto &behavior : behavior_container) {
    int task_type = behavior.behavior_type;
    int motion_type = behavior.motion_type;
    int path_choice = behavior.behavior_candidate_index;

    double cost = 0.0;
    double st_ay = behavior.st_array.a[2]; // t_sample = 0.3s

    bool second_retreat_adjust = false;

    if (motion_type == 1 && path_choice == 0) {
      cost -= 35.0; // set_speed, 7s
    } else if (motion_type == 1 && path_choice == 1) {
      cost -= 24.0; // const speed, 7s
    } else if (motion_type == 2 && path_choice == 2) {
      cost -= 15.0; // set speed 10s
    } else if (motion_type == 2 && path_choice == 3) {
      cost -= 10.0;
    } else if (motion_type == 3) {
      cost -= 28.0; // hot-start, 7s
    } else if (motion_type == 5) {
      cost += 50.0; // lc back
    } else if (motion_type == 6) {
      cost += 40.0;
    }

    if (((motion_result_o >= 5 && motion_type <= 3 && motion_type >= 1) ||
         (motion_result_o == 5 && motion_type == 6)) &&
        task_type != 1 &&
        ((st_ay > 0.5 && st_ay_o <= -0.3) ||
         (st_ay < -0.6 && st_ay_o >= 0.2))) {
      // && task_type != 1 && (st_ay>0.5||st_ay<-0.88) && abs(st_ay_o -
      // st_ay)>=0.3){
      cost = 100.0;
      if (path_choice == 1) {
        second_retreat_adjust = true;
      }
    }

    if (abs(st_ay - st_ay_o) >= 0.8 && st_ay * st_ay_o < 0 &&
        motion_type <= 3 && motion_type >= 1 && motion_result_o <= 3 &&
        motion_result_o >= 1) {
      cost = 40.0;
      // second_retreat_adjust = true;
    }

    if (task_type_o == 1 && task_type != 1 && motion_type <= 3 &&
        motion_type >= 1 && abs(st_ay - st_ay_o) >= 1.8 && st_ay <= -1.1) {
      cost = 100.0;
      // second_retreat_adjust = true;
    } else if (task_type != 1 && motion_result_o <= 3 && motion_result_o >= 1 &&
               motion_type <= 3 && motion_type >= 1 && st_ay_o - st_ay >= 1.8 &&
               st_ay_o <= -0.8) {
      cost = 100.0;
      // second_retreat_adjust = true;
    }

    const auto &gap_info = extract_gap_info(behavior);
    const int &gap_front_id_rel = gap_info.front().id_relative;
    const int &gap_rear_id_rel = gap_info.back().id_relative;
    const auto &obj_infos = general_motion_planner_input_.obj_infos;

    double gap_front_v =
        (gap_front_id_rel < 0)
            ? 0.0
            : obj_infos[gap_front_id_rel].obj_state_init.obj_state_local.vel;

    double gap_rear_v =
        (gap_rear_id_rel < 0)
            ? 0.0
            : obj_infos[gap_rear_id_rel].obj_state_init.obj_state_local.vel;

    bool wait_to_proceed =
        (motion_type <= 3 && motion_type >= 1 &&
         (motion_result_o >= 5 || motion_result_o == 0) && task_type != 1) ||
        (task_type_o == 1 && task_type != 1 && motion_type <= 3 &&
         motion_type >= 1) ||
        (general_motion_planner_output_.lc_wait &&
         lc_remain_time > lc_total_time - t_confirm);
    bool is_in_lc =
        lc_proceed_time_ >= 0.2 && (motion_type <= 3 && motion_type >= 1);
    bool gap_front_too_close = false;
    bool gap_rear_too_close = false;
    bool gap_rear_too_fast = false;
    bool v_over_120kph = false;
    bool is_lane_error = false;

    double headaway_gate_keeper = 0.8;
    double dist_gate_keeper = headaway_gate_keeper * ego_speed;
    dist_gate_keeper = min(10.0, dist_gate_keeper);

    if (wait_to_proceed && gap_info[1].id != -1 &&
        gap_info[1].pos_init.x >= -dist_gate_keeper) {
      // gap rear car ahead of ego
      cost = 200.0;
      gap_rear_too_close = true;
    }
    if (wait_to_proceed && gap_info[0].id != -1 &&
        gap_info[0].pos_init.x < dist_gate_keeper) {
      // gap front car too close
      cost = 200.0;
      gap_front_too_close = true;
    }

    if (is_in_lc && gap_info[0].id != -1 && gap_info[0].pos_init.x < -3.0) {
      // gap front car behind ego
      cost = 200.0;
      gap_front_too_close = true;
    }
    if (is_in_lc && gap_info[1].id != -1 && gap_info[1].pos_init.x >= 0.0 &&
        gap_rear_v > 0.1) {
      // gap rear car ahead of ego
      cost = 200.0;
      gap_rear_too_close = true;
    }

    if (wait_to_proceed && gap_info[1].id != -1 &&
        is_rear_wild(gap_info[1].id_relative)) {
      // gap rear car too fast
      cost = 200.0;
      gap_rear_too_fast = true;
    }

    if (wait_to_proceed && ego_speed > target_speed_lc_wait_max) {
      cost = 200.0;
      v_over_120kph = true;
    }

    if (wait_to_proceed && (true == is_solid_line_ || false == valid_body_ ||
                            false == is_lane_stable_)) {
      cost = 200;
    }
    
    if (wait_to_proceed && (!has_tlane_ && has_olane_)) {
      cost = 200;
      is_lane_error = true;
    }

    if(is_in_lc && !has_tlane_ && has_olane_){
      cost = 200;
      is_lane_error = true;
    }

    if (second_retreat_adjust) {
      update_retreat_speed_adjustment(st_ay_o, st_ay);
      retreat_count_max_ = 2;
      // general_motion_planner_output_.lc_wait_speed_adjust_count = 2;
    }

    cost = min(cost, 200.0);
    behavior.cost = cost;

    MSD_LOG(INFO,
            "GMP behavior evaluate[%.2f][%d]: %.1f,a: %.1f %.1f p: %.1f "
            "%.1f,%d,%d,%d,%d,%d,%d,%d,%d",
            MTIME()->timestamp().sec() - gmp_created_time_,
            behavior.motion_type, behavior.cost, behavior.st_array.a[2],
            st_a_buffer_.back(), gap_info[0].pos_init.x, gap_info[1].pos_init.x,
            gap_rear_too_close, gap_front_too_close, gap_rear_too_fast,
            v_over_120kph, is_solid_line_, valid_body_, is_lane_stable_,is_lane_error);
  }
  auto cmp = [](const BehaviorAdvice &a, const BehaviorAdvice &b) {
    return a.cost < b.cost;
  };
  sort(behavior_container.begin(), behavior_container.end(), cmp);

  return behavior_container.front();
}

void GeneralMotionPlanner::update_best_gap(const BehaviorAdvice &behavior) {

  const int &motion_result = motion_result_buffer_.back();
  if (!(task_type_ > 1 && motion_result == 5)) {
    return;
  }

  const auto &obj_infos = general_motion_planner_input_.obj_infos;
  const double &ego_speed = general_motion_planner_input_.ego_state.ego_vel;
  const double &lc_wait_time = behavior.behavior_info.lc_time_wait;

  const double t_end = 10.0;
  const double ego_x0 = 0.1 * ego_speed;
  // const double ego_xe = t_end*ego_speed;
  const int t_check_index1 = 50; // check 5s
  if (behavior.st_array.s.size() <= t_check_index1) {
    return;
  }
  const double ego_xe = behavior.st_array.s[t_check_index1];

  int id_neck2neck = -1;
  bool should_update_best_gap = false;

  if (lc_wait_time < 3.0) { // t_confirm = 0.5
    return;
  }

  for (const auto &obj : obj_infos) {
    id_neck2neck++;
    const double &obj_x0 = obj.obj_state_init.obj_state_local.pos.x;
    const double &obj_xe =
        obj.pred_trajectory[t_check_index1].obj_state_local.pos.x;
    const double &sm_lon_ini =
        obj.pred_trajectory.front().safety_margin.longitu;
    const double &sm_lon_end =
        obj.pred_trajectory[t_check_index1].safety_margin.longitu;

    if (abs(obj_x0 - ego_x0) < sm_lon_ini &&
        abs(obj_xe - ego_xe) < sm_lon_end) {
      should_update_best_gap = true;
      break;
    }
  }
  MSD_LOG(INFO, "GMP update gap[%.1f][%d]",
          MTIME()->timestamp().sec() - gmp_created_time_,
          should_update_best_gap);

  if (!should_update_best_gap) {
    return;
  }

  const auto &current_gap = general_motion_planner_output_.gap_info;

  const auto &current_cipv = general_motion_planner_output_.cipv_info.front();

  vector<pair<double, int>> obj_row;
  int id_relative = 0;
  const int t_check_index = 50; // 5s

  obj_row.emplace_back(500.0, -1);

  for (const auto &obj : obj_infos) {
    if (obj.id == current_cipv.id ||
        obj.pred_trajectory[t_check_index].lane_assignment == task_type_) {
      obj_row.emplace_back(
          obj.pred_trajectory[t_check_index].obj_state_local.pos.x,
          id_relative);
    }
    id_relative++;
  }
  obj_row.emplace_back(-500.0, -1);

  if (obj_row.empty()) {
    return;
  }

  auto cmp = [](const pair<double, int> &a, const pair<double, int> &b) {
    return a.first > b.first;
  };
  sort(obj_row.begin(), obj_row.end(), cmp);

  vector<pair<double, pair<int, int>>>
      gaps_length; // <length of net gap deducting safetymargin, <relative id of
                   // front, rear car>>
  for (int i = 0; i < obj_row.size() - 1; ++i) {
    gaps_length.emplace_back(
        obj_row[i].first - obj_row[i + 1].first,
        make_pair(obj_row[i].second, obj_row[i + 1].second));
    if (obj_row[i].second != -1) {
      gaps_length.back().first -= obj_infos[obj_row[i].second]
                                      .pred_trajectory[t_check_index]
                                      .safety_margin.longitu;
    }
    if (obj_row[i + 1].second != -1) {
      gaps_length.back().first -= obj_infos[obj_row[i + 1].second]
                                      .pred_trajectory[t_check_index]
                                      .safety_margin.longitu;
    }
    if (gaps_length.back().second.second != -1) {
      MSD_LOG(INFO, "GMP update gap2[%.1f]: %.1f, %.1f",
              MTIME()->timestamp().sec() - gmp_created_time_,
              gaps_length.back().first,
              obj_infos[gaps_length.back().second.second]
                  .obj_state_init.obj_state_local.pos.x);
    }
  }

  bool second_retreat_adjust = false;
  double st_ay = 0.0;
  double st_ay_o = 0.0;
  const double &target_speed_lc_wait =
      general_motion_planner_output_.lc_wait_speed_advice;

  for (const auto &gap_length : gaps_length) {
    if (gap_length.second.second == current_gap.front().id_relative &&
        current_gap.front().id_relative == id_neck2neck &&
        gap_length.first >= 1.0 && target_speed_lc_wait <= ego_speed) {
      // if the gap in front of the current gap is available and gap front is
      // very close and decelerating
      second_retreat_adjust = true;
      st_ay = 0.1;

    } else if (gap_length.second.first == current_gap.back().id_relative &&
               current_gap.back().id_relative == id_neck2neck &&
               gap_length.first >= 1.0 && target_speed_lc_wait > ego_speed) {
      // if the gap behind of the current gap is available and gap rear is very
      // close and accelerating
      second_retreat_adjust = true;
      st_ay = -0.1;
    }
  }

  if (second_retreat_adjust) {
    update_retreat_speed_adjustment(st_ay_o, st_ay);
    retreat_count_max_ = 2;
  }

  return;
}

double GeneralMotionPlanner::cal_cipv_ttc() {
  double cipv_ttc = 10.0;
  const double &ego_speed =
      general_motion_planner_input_.ego_state_cart.ego_vel;
  const double &ego_acc = general_motion_planner_input_.ego_state_cart.ego_acc;
  if (general_motion_planner_output_.cipv_info.empty()) {
    return cipv_ttc;
  }
  if (general_motion_planner_output_.cipv_info.front().id_relative != -1 &&
      ego_speed > 0.5) {
    const auto &cipv_info =
        general_motion_planner_input_.obj_infos
            [general_motion_planner_output_.cipv_info.front().id_relative];
    double front_car_vel = cipv_info.obj_state_init.obj_state_local.vel;
    double front_car_pos = cipv_info.obj_state_init.obj_state_local.pos.x;
    double speed_rel = max(ego_speed - front_car_vel, 0.2);
    cipv_ttc = (front_car_pos - 2.5) / speed_rel;

    MSD_LOG(INFO, "GMP cal_cipv_ttc[%.1f]: %.1f, %.1f, %.1f, %.1f, %.1f",
            MTIME()->timestamp().sec() - gmp_created_time_, front_car_vel,
            front_car_pos, ego_speed, speed_rel, cipv_ttc);
  }

  return min(cipv_ttc, 10.0);
}

void GeneralMotionPlanner::update_lc_time() {
  const bool should_update_gmp_lc_info = lc_request_ != 1;
  double cipv_ttc = cal_cipv_ttc();

  MSD_LOG(INFO, "GMP update_lc_time[%.1f]: %d, %.1f",
          MTIME()->timestamp().sec() - gmp_created_time_,
          should_update_gmp_lc_info, cipv_ttc);

  if (should_update_gmp_lc_info && cipv_ttc < 7.0) {
    MSD_LOG(INFO, "GMP update_lc_time ok");
    general_motion_planner_input_.gmp_suggest_lc_time = max(5.0, cipv_ttc);
  }
}

bool GeneralMotionPlanner::check_rss_safety(void) {
  const bool lane_cross = general_motion_planner_input_.lc_task_info.lane_cross;
  if (lane_cross) {
    return true;
  }

  const double lc_latdist_abs_past =
      general_motion_planner_input_.lc_task_info.lc_latdist_abs_past;
  if (lc_latdist_abs_past < 0.7 || lc_latdist_abs_past > 1.1) {
    return true;
  }

  int task_type = task_type_buffer_.back();
  int motion_result = motion_result_buffer_.back();
  const int lc_action_state = general_motion_planner_output_.lc_action_state;
  const int lc_wait = general_motion_planner_output_.lc_wait;
  const bool gmp_decision_valid = general_motion_planner_output_.gmp_valid;
  const bool lc_in_proceed = (task_type > 1 && motion_result != 0 &&
                              (lc_action_state == 1 && lc_wait == 0 ||
                               lc_action_state == -1 && lc_wait == 0) &&
                              gmp_decision_valid);
  if (!lc_in_proceed) {
    return true;
  }

  const auto &current_gap = general_motion_planner_output_.gap_info;
  if (current_gap.empty() || current_gap.back().id_relative == -1) {
    return true;
  }

  const auto &obj =
      general_motion_planner_input_.obj_infos[current_gap.back().id_relative];
  double rear_car_x = current_gap.back().pos_init.x;
  double rear_car_length = obj.obj_size.length;
  double rear_gap_dist = abs(rear_car_x) - 0.5 * rear_car_length - 2.5;
  // distinguish rear car just pop in or not
  const double rear_dist_thres =
      (lc_proceed_time_ - obj.noticed_timer > 0.7) ? 2.0 : 4.0;
  double ego_vel = general_motion_planner_input_.ego_state.ego_vel;
  double rear_car_vel = obj.obj_state_init.obj_state_local.vel;
  if (rear_gap_dist < rear_dist_thres && ego_vel <= rear_car_vel + 0.6) {
    MSD_LOG(INFO, "GMP RSS[%.1f] rear obj unsafe %.1fm",
            MTIME()->timestamp().sec() - gmp_created_time_, rear_gap_dist);
    return false;
  }

  return true;
}

void GeneralMotionPlanner::update_lc_proceed_time(void) {
  lc_action_state_buffer_.front() = lc_action_state_buffer_.back();
  lc_action_state_buffer_.back() = lc_action_state_;

  static bool is_in_lc_proceed = false;
  if (lc_action_state_buffer_.back() == 1 &&
      lc_action_state_buffer_.front() == 0) {
    is_in_lc_proceed = true;
  } else if (is_in_lc_proceed && task_type_ == 1) {
    is_in_lc_proceed = false;
  }

  if (is_in_lc_proceed) {
    lc_proceed_time_ += 0.1;
  } else {
    lc_proceed_time_ = 0.0;
  }

  MSD_LOG(INFO, "GMP lc_in_proceed[%.1fs][%d]:%.1fs",
          MTIME()->timestamp().sec() - gmp_created_time_, is_in_lc_proceed,
          lc_proceed_time_);
  general_motion_planner_output_.lc_in_proceed_time = lc_proceed_time_;
  return;
}

double
GeneralMotionPlanner::infer_behavior_success_prob(const int &behavior_state) {
  int observe_width = behavior_success_buffer_.size();
  if (observe_width == 0) {
    return 1.0;
  }
  for (int i = 1; i < behavior_success_buffer_.size(); ++i) {
    behavior_success_buffer_[i - 1] = behavior_success_buffer_[i];
  }
  behavior_success_buffer_.back() = behavior_state;

  int success_count = 0;

  for (int i = 0; i < behavior_success_buffer_.size(); ++i) {
    if (behavior_success_buffer_[i] != 1) {
      continue;
    }
    success_count++;
  }

  return double(success_count) / double(observe_width);
}

void GeneralMotionPlanner::conclude_lane_change_decision(void) {
  // int lc_action_state = 0; // lc_action_state: wait: 0; back; -1: change:1
  update_lc_proceed_time();

  int lc_wait = 0;
  int motion_result = motion_result_buffer_.back();
  general_motion_planner_output_.motion_result = motion_result;
  int task_type = task_type_buffer_.back();
  const double &lc_remain_time =
      general_motion_planner_input_.lc_task_info.lc_time_remain;
  const double &lc_remain_dist =
      general_motion_planner_input_.lc_task_info.lc_dist_remain;
  const double &lc_total_time =
      general_motion_planner_input_.lc_task_info.lc_time_total;
  const double &lc_wait_time =
      general_motion_planner_input_.lc_task_info.lc_time_wait;
  const double t_confirm = 0.4;
  const bool &lane_cross =
      general_motion_planner_input_.lc_task_info.lane_cross;
  const double &ego_speed = general_motion_planner_input_.ego_state.ego_vel;
  const auto &current_gap = general_motion_planner_output_.gap_info;
  const auto &current_cipv = general_motion_planner_output_.cipv_info.front();

  if (check_rss_safety()) {
    if (task_type > 1 && motion_result <= 3 && motion_result >= 1 &&
        lc_action_state_ != -1) {
      if (lc_remain_time <= lc_total_time - t_confirm ||
          lc_proceed_time_ > 0.0) {
        lc_action_state_ = 1;
        lc_wait = 0;
      } else {
        lc_action_state_ = 0;
        lc_wait = 1;
      }
    } else if (task_type > 1 && (motion_result == 5 || motion_result == 0)) {
      if (lc_action_state_ == 0) {
        if (lc_proceed_time_ > 0.3 + t_confirm) {
          lc_action_state_ = -1;
          lc_wait = 0;
        } else {
          lc_action_state_ = 0;
          lc_wait = 1;
        }
      } else if (lc_action_state_ == 1 &&
                 ((motion_result == 5 &&
                   lc_proceed_time_ >
                       0.3 + t_confirm && // lc_pass_time > 0.3 + t_confirm
                   !lane_cross) ||
                  (motion_result == 0 && !lane_cross))) {
        lc_wait = 0;
        lc_action_state_ = -1; // back
      } else if (lc_action_state_ == 1 &&
                 (motion_result == 5 && lc_wait_time > 0.3 && !lane_cross)) {
        lc_wait = 0;
        lc_action_state_ = -1; // back
      } else if (lc_action_state_ == -1) {
        if (!lane_cross) {
          lc_wait = 0;
          lc_action_state_ = -1; // back
        } else {
          lc_wait = 0;
          lc_action_state_ = 1;
        }
      }
    } else if (task_type > 1 && motion_result <= 3 && motion_result >= 1 &&
               lc_action_state_ == -1) {
      lc_wait = 0;
      lc_action_state_ = 1; // keep lc
    } else {
      lc_action_state_ = 0;
      lc_wait = 0;
    }
  } else {
    // rss check fail
    if (task_type > 1) {
      lc_action_state_ = -1; // back
      lc_wait = 0;
    } else {
      lc_action_state_ = 0;
      lc_wait = 0;
    }
  }

  bool gmp_decision_valid = motion_result != 0;
  double headaway = general_motion_planner_input_.headaway; // default 1.6
  double headaway_smoothed = general_motion_planner_input_.headaway_smoothed;

  // add olane and tlane
  MSD_LOG(INFO, "gmp external_request[%.1f][%d]: %d %d %d %d %d %d %.1fm/s",
          MTIME()->timestamp().sec() - gmp_created_time_, gmp_decision_valid,
          lc_action_state_, lc_wait, motion_result, task_type_, has_tlane_,
          has_olane_, ego_speed);
  if (task_type_ != 1 && !has_tlane_ && !has_olane_ &&
             lc_request_ != 1) {
    MSD_LOG(INFO, "gmp external_request: no tlane and no olane");
    lc_action_state_ = 0;
    lc_wait = 0;
  }

  if (task_type_ == 1) {
    gmp_decision_valid = false;
  }
  MSD_LOG(INFO, "GMP lc_action_state[%.1f][%d]: %d %d %d %.1fm/s",
          MTIME()->timestamp().sec() - gmp_created_time_, gmp_decision_valid,
          lc_action_state_, lc_wait, motion_result, ego_speed);

  bool enable_new_gap_logic =
      ConfigurationContext::Instance()
          ->planner_config()
          .lateral_behavior_planner_config.enable_new_gap_logic;
  MSD_LOG(INFO, "enable_new_gap_logic : %d", enable_new_gap_logic);

  general_motion_planner_output_.gmp_valid = gmp_decision_valid;
  general_motion_planner_output_.lc_action_state = lc_action_state_;
  general_motion_planner_output_.lc_wait = lc_wait;
  general_motion_planner_output_.lc_remain_dist = lc_remain_dist;
  general_motion_planner_output_.lc_remain_time = lc_remain_time;

  bool gap_has_front_car = current_gap.front().id_relative != -1;
  bool pre_planning_fail = motion_result == 0;
  if (gap_has_front_car && !pre_planning_fail && task_type > 1) {
    if (current_gap.front().id_relative <
        general_motion_planner_input_.obj_infos.size()) {
      const auto &obj_front = general_motion_planner_input_
                                  .obj_infos[current_gap.front().id_relative];
      double front_car_length = obj_front.obj_size.length;
      double front_car_vel = obj_front.obj_state_init.obj_state_local.vel;
      double front_car_pos_x = obj_front.obj_state_init.obj_state_local.pos.x;
      if (retreat_speed_adjustment_ == 2 && front_car_vel <= ego_speed - 1.0) {
        // if front car is slower than ego, then real decelerating
        retreat_speed_adjustment_ = 3;
        general_motion_planner_output_.lc_wait_speed_adjust_advice =
            retreat_speed_adjustment_;
      }
    }
  }

  if (lc_action_state_ == 0 && lc_wait == 0) {
    retreat_speed_adjustment_ = 0;
    general_motion_planner_output_.lc_wait_speed_adjust_advice =
        retreat_speed_adjustment_;
    general_motion_planner_output_.lc_remain_dist = 0.0;
    general_motion_planner_output_.lc_remain_time = 0.0;
  }

  if (!enable_new_gap_logic) {
    general_motion_planner_output_.gmp_valid = false;
    general_motion_planner_output_.lc_remain_dist = -1.0;
  }
}

void GeneralMotionPlanner::update_warm_start(const BehaviorAdvice &behavior) {

  warm_start_usable_ = false;

  const int &task_type = behavior.behavior_type;
  const auto &gap_info = general_motion_planner_output_.gap_info;
  const int &motion_type = behavior.motion_type;

  if (!(gap_info[0].id == -1 && gap_info[1].id == -1) && motion_type >= 1 &&
      motion_type <= 3 && task_type > 1) {
    warm_start_usable_ = true;
    behavior_cache_ = behavior;
  }
}

void GeneralMotionPlanner::update_warm_start(const int &motion_type) {
  if (motion_type == 0) {
    warm_start_usable_ = false;
  }
}

TaskStatus GeneralMotionPlanner::execute(ScenarioFacadeContext *context) {
  MLOG_PROFILING(name_.c_str());
  update_call_count();
  get_created_time();

  if (Task::execute(context) != TaskStatus::STATUS_SUCCESS) {
    return TaskStatus::STATUS_FAILED;
  }
  if (world_model_ == nullptr) {
    MSD_LOG(INFO, "GMP: world model is none!");
    return TaskStatus::STATUS_FAILED;
  }
  if (!baseline_info_ || !baseline_info_->is_valid()) {
    MSD_LOG(INFO, "GMP: baseline info is invalid!");
    return TaskStatus::STATUS_FAILED;
  }
  gmp_baseline_info_ = world_model_->get_baseline_info(0);
  if (!gmp_baseline_info_ || !gmp_baseline_info_->is_valid()) {
    MSD_LOG(INFO, "GMP: real_baseline info is invalid!");
    return TaskStatus::STATUS_FAILED;
  }

  double start_time = MTIME()->timestamp().sec();

  frenet_coord_ = gmp_baseline_info_->get_frenet_coord();
  update_lc_time();
  //------------------------formulate gmp_input------------------------//
  gmp_preprocessor_->init(context_, world_model_, gmp_baseline_info_);
  gmp_preprocessor_->get_lc_request(
      lc_request_, valid_body_, is_steer_over_limit_, is_lca_state_activated_,
      is_lane_stable_, is_solid_line_, gmp_should_cancel_);
  gmp_preprocessor_->process();
  //------------------------formulate gmp_input------------------------//

  //------------------------ motion planning ------------------------//
  gerenate_s_at_control_quad_points();
  update_task_type();
  update_max_retreat_count();
  update_objs_display_sm();
  update_max_speed();

  vector<BehaviorAdvice> behavior_container{};
  general_motion_planner_output_.trajectories_gmp.clear();

  const auto &behavior_candidates =
      general_motion_planner_input_.behavior_candidates;
  obj_static_id_caches_.clear();
  for (const auto &behavior_candidate : behavior_candidates) {
    BehaviorAdvice behavior_advice;
    MSD_LOG(INFO, "Current behav cost: %.1f", behavior_advice.cost);
    if (path_planning(behavior_candidate, behavior_advice)) {
      if (speed_planning(behavior_advice)) {
        behavior_container.emplace_back(behavior_advice);
      }
    }
  }

  if (behavior_container.empty()) {
    MSD_LOG(INFO, "GMP_path_plan_fail[%.1f]!",
            MTIME()->timestamp().sec() - gmp_created_time_);
    update_task_result_buffer(task_type_, 0);
    update_warm_start(0);

  } else {
    BehaviorAdvice behavior_advice_best = // behavior_container.front();
        evaluate_behavior(behavior_container);

    if (behavior_advice_best.cost >= 199.9) {
      MSD_LOG(INFO, "GMP_path_plan_fail[%.1f]!",
              MTIME()->timestamp().sec() - gmp_created_time_);
      update_task_result_buffer(task_type_, 0);
    } else {
      general_motion_planner_output_.trajectories_gmp.emplace_back(
          behavior_advice_best.trajectory);

      general_motion_planner_output_.cipv_info =
          extract_cipv_info(behavior_advice_best);
      general_motion_planner_output_.gap_info =
          extract_gap_info(behavior_advice_best);
      general_motion_planner_output_.lead_objs_info =
          extract_lead_objs_info(behavior_advice_best);

      general_motion_planner_output_.s_v_array = behavior_advice_best.s_v;

      update_task_result_buffer(behavior_advice_best);

      update_general_motion_planner_output(behavior_advice_best);

      update_warm_start(behavior_advice_best);

      MSD_LOG(INFO, "GMP_path_plan_succeed! Current behav num %d",
              behavior_container.size());
    }
  }

  if (general_motion_planner_output_.trajectories_gmp.empty()) {
    MSD_LOG(INFO, "GMP_motion_plan_fail[%.1f]!",
            MTIME()->timestamp().sec() - gmp_created_time_);
  } else {
    MSD_LOG(INFO, "GMP_motion_plan_succeed[%.1f]!",
            MTIME()->timestamp().sec() - gmp_created_time_);
  }

  conclude_lane_change_decision();

  //------------------------end of motion planning------------------------//

  set_debug_info();
  double end_time = MTIME()->timestamp().sec();
  MSD_LOG(INFO, "GMP cost time[%.1f]: %.5f",
          MTIME()->timestamp().sec() - gmp_created_time_,
          end_time - start_time);

  return TaskStatus::STATUS_SUCCESS;
}

void GeneralMotionPlanner::update_task_result_buffer(
    const BehaviorAdvice &behavior_advice) {
  task_type_buffer_.front() = task_type_buffer_.back();
  task_type_buffer_.back() = behavior_advice.behavior_type;

  motion_result_buffer_.front() = motion_result_buffer_.back();
  motion_result_buffer_.back() = behavior_advice.motion_type;

  st_a_buffer_.front() = st_a_buffer_.back();
  st_a_buffer_.back() = behavior_advice.st_array.a[2]; // t = 0.3s
}

void GeneralMotionPlanner::update_task_result_buffer(const int &task_type,
                                                     const int &motion_type) {

  task_type_buffer_.front() = task_type_buffer_.back();
  task_type_buffer_.back() = task_type;

  motion_result_buffer_.front() = motion_result_buffer_.back();
  motion_result_buffer_.back() = motion_type;
  general_motion_planner_output_.chosen_behavior_candidate_index = -1;
  gmp_interface::Pose2d pos_default{-200.0, 0.0, 0.0};
  ObjInfoRefined obj_info_refined_default{-1, -1, 2.0, pos_default};
  vector<ObjInfoRefined> obj_infos_default(2, obj_info_refined_default);

  general_motion_planner_output_.cipv_info = obj_infos_default;
  general_motion_planner_output_.gap_info = obj_infos_default;
  general_motion_planner_output_.lead_objs_info = obj_infos_default;
}

void GeneralMotionPlanner::set_debug_info(void) {
  reset_debug_info();
  auto *planner_debug = context_->mutable_planner_debug();
  auto *planner_output = context_->mutable_general_motion_planner_output();
  // auto *planner_debug = PlanningContext::Instance()->mutable_planner_debug();

  // update sm display
  // if (!general_motion_planner_input_.obj_infos.empty()
  //       && general_motion_planner_input_.obj_infos.back().noticed_timer
  //       >= 14.0){
  //   general_motion_planner_input_.obj_infos.pop_back();
  // }
  int best_behavior_idx =
      general_motion_planner_output_.chosen_behavior_candidate_index;
  best_behavior_idx = max(best_behavior_idx, 0);
  int max_obj_valid =
      min(general_motion_planner_input_.obj_infos.size(),
          gmp_objs_sm_buffer_[best_behavior_idx].all_objs_pred_sm.size());
  for (int obj_idx = 0; obj_idx < max_obj_valid; obj_idx++) {
    auto &obj = general_motion_planner_input_.obj_infos[obj_idx];
    auto obj_sm_new =
        gmp_objs_sm_buffer_[best_behavior_idx].all_objs_pred_sm[obj_idx];

    for (int t_index = 0; t_index < obj.pred_trajectory.size(); t_index++) {
      general_motion_planner_input_.obj_infos[obj_idx]
          .pred_trajectory[t_index]
          .safety_margin = obj_sm_new.pred_sm[t_index];
    }
  }

  planner_debug->gmp_input = general_motion_planner_input_;
  planner_debug->gmp_output = general_motion_planner_output_;

  *planner_output = general_motion_planner_output_;
}

void GeneralMotionPlanner::reset_debug_info(void) {
  auto *planner_debug = context_->mutable_planner_debug();
  // auto *planner_debug = PlanningContext::Instance()->mutable_planner_debug();
  planner_debug->gmp_debug_info.refline_heading.clear();
  planner_debug->gmp_debug_info.refline_pos.clear();
  planner_debug->gmp_debug_info.trajectories_gmp.clear();

  planner_debug->gmp_input = {};
  planner_debug->gmp_output = {};
}

} // namespace msquare