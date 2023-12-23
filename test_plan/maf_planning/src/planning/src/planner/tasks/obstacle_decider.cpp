#include "planner/tasks/obstacle_decider.h"
#include "data_driven_planner/common/ddp_context.h"
#include "linear_interpolation_utility.hpp"
#include "planner/behavior_planner/deciders/st_graph_generator.h"
#include <string>
#include <utility>

namespace msquare {

using namespace msquare::planning_math;

ObstacleDecider::ObstacleDecider(const TaskConfig &config) : Task(config) {
  odc_preprocessor_ = make_shared<ObstacleDeciderPreprocessor>();
  odc_preprocessor_->mutable_external_input(&obstacle_decider_input_,
                                            &obstacle_decider_output_);
  veh_param_ = ConfigurationContext::Instance()->get_vehicle_param();
}

ObstacleDecider::~ObstacleDecider() = default;

void ObstacleDecider::init(std::shared_ptr<WorldModel> world_model) {
  Task::init(world_model);
}

void ObstacleDecider::reset(const TaskConfig &config) {
  Task::reset(config);
  frenet_coord_ = nullptr;
  odc_baseline_info_ = nullptr;
}

void ObstacleDecider::unset() {
  frenet_coord_ = nullptr;
  odc_baseline_info_ = nullptr;
}

void ObstacleDecider::stamp_create_time() {
  if (call_count_ == 1) {
    odc_created_time_ = MTIME()->timestamp().sec();
  }
  return;
}

void ObstacleDecider::fake_ddp_trajectory() {
  ddp::DdpTrajectory ddp_trajectory{
      {}, ddp::TrajectoryTypeEnum::DDP_RAW, 0.0, {}};
  if (!ddp_trajectories_.empty()) {
    ddp_trajectories_.clear();
  }
  ddp_trajectory.trajectory.clear();

  const double ego_speed = obstacle_decider_input_.ego_state_cart.ego_vel;

  const double t_end = 4.0;

  double t_delta = 0.2;
  double t_step = t_delta;
  double ego_s0 = odc_baseline_info_->get_ego_state().planning_start_state.s;

  while (t_step <= t_end + 0.1) {

    double s = ego_speed * t_step + ego_s0;
    double l = 0.0;
    Point2D traj_point_frenet{s, l};
    Point2D traj_point_cart{0.0, 0.0};

    if (frenet_coord_->FrenetCoord2CartCoord(
            traj_point_frenet, traj_point_cart) == TRANSFORM_SUCCESS) {
      ddp::TrajectoryPoint traj_point_ddp;
      traj_point_ddp.x = traj_point_cart.x;
      traj_point_ddp.y = traj_point_cart.y;
      traj_point_ddp.heading_angle =
          frenet_coord_->GetRefCurveHeading(traj_point_frenet.x);
      traj_point_ddp.curvature =
          frenet_coord_->GetRefCurveCurvature(traj_point_frenet.x);
      traj_point_ddp.t = t_step;
      traj_point_ddp.v = ego_speed;
      traj_point_ddp.a = 0.0;
      traj_point_ddp.s = s;
      traj_point_ddp.l = l;
      traj_point_ddp.frenet_valid = true;

      ddp_trajectory.trajectory.emplace_back(traj_point_ddp);
      MSD_LOG(INFO, "ODC[%.1f] ddp_traj[%.1f]: (%.1f, %.1f)",
              MTIME()->timestamp().sec() - odc_created_time_, t_step, l, s);
    } else {
      MSD_LOG(INFO, "ODC[%.1f]-ddp_traj[%.1f]: (%.1f, %.1f)",
              MTIME()->timestamp().sec() - odc_created_time_, t_step, l, s);
      break;
    }
    t_step += t_delta;
  }

  if (!ddp_trajectory.trajectory.empty()) {
    ddp_trajectories_.emplace_back(ddp_trajectory);
  }

  obstacle_decider_output_.current_lane_ddp_trajectory.clear();
  for (const auto &ddp_pt : ddp_trajectory.trajectory) {
    odc_interface::TrajectoryPoint pt;
    pt.x = ddp_pt.x;
    pt.y = ddp_pt.y;
    pt.heading_angle = ddp_pt.heading_angle;
    pt.curvature = ddp_pt.curvature;

    pt.t = ddp_pt.t;
    pt.v = ddp_pt.v;
    pt.a = ddp_pt.a;
    pt.s = ddp_pt.s;
    pt.l = ddp_pt.l;
    pt.frenet_valid = ddp_pt.frenet_valid;

    odc_interface::Point2d pt_env{pt.x, pt.y};
    odc_interface::Point2d pt_local = env2local_pos(pt_env);

    pt.x_local = pt_local.x;
    pt.y_local = pt_local.y;

    obstacle_decider_output_.current_lane_ddp_trajectory.emplace_back(pt);
  }
}

void ObstacleDecider::load_ddp_multipath(void) {
  const auto &ddp_multipath_origin = ddp::DdpContext::Instance()
                                         ->model_result_manager()
                                         ->get_ddp_trajectory_info();

  obstacle_decider_output_.ddp_multipath.clear();
  if (ddp_multipath_origin.empty()) {
    return;
  }

  for (const auto &ddp_path : ddp_multipath_origin) {
    if (ddp_path.trajectory.empty()) {
      continue;
    }
    odc_interface::DdpTrajectory trasform_ddp_path;
    for (const auto &ddp_pt : ddp_path.trajectory) {
      odc_interface::TrajectoryPoint pt;
      pt.x = ddp_pt.x;
      pt.y = ddp_pt.y;
      pt.heading_angle = ddp_pt.heading_angle;
      pt.curvature = ddp_pt.curvature;

      pt.t = ddp_pt.t;
      pt.v = ddp_pt.v;
      pt.a = ddp_pt.a;
      pt.s = ddp_pt.s;
      pt.l = ddp_pt.l;
      pt.frenet_valid = ddp_pt.frenet_valid;

      odc_interface::Point2d pt_env{pt.x, pt.y};
      odc_interface::Point2d pt_local = env2local_pos(pt_env);

      pt.x_local = pt_local.x;
      pt.y_local = pt_local.y;

      trasform_ddp_path.trajectory.emplace_back(pt);
    }
    trasform_ddp_path.logit = ddp_path.logit;
    obstacle_decider_output_.ddp_multipath.emplace_back(trasform_ddp_path);
  }
}

void ObstacleDecider::load_ddp_trajectory() {
  obstacle_decider_output_.odc_decision_valid =
      use_eftp_; // TODO use model valid from ddp_context

  MSD_LOG(INFO, "ODC-real_ddp_traj[%.1f][%d][%d]",
          MTIME()->timestamp().sec() - odc_created_time_, use_eftp_,
          world_model_->use_eftp());

  const auto &ddp_trajectory_origin = ddp::DdpContext::Instance()
                                          ->model_result_manager()
                                          ->get_current_lane_ddp_trajectory();

  MSD_LOG(INFO, "ODC-real_ddp_traj[%.1f][%d],size:[%d]",
          MTIME()->timestamp().sec() - odc_created_time_, use_eftp_,
          ddp_trajectory_origin.trajectory.size());

  if (ddp_trajectory_origin.trajectory.empty()) {
    return;
  }

  ddp::DdpTrajectory ddp_trajectory =
      process_ddp_trajectory(ddp_trajectory_origin);

  // check if ddp_trajectory vaild
  double s_max = 0.0;
  for (const auto &point : ddp_trajectory.trajectory) {
    s_max = std::max(s_max, point.s);
  }
  const double ego_speed = obstacle_decider_input_.ego_state_cart.ego_vel;
  Point2D ego_frenet{0.0, 0.0};
  Point2D ego_cart{obstacle_decider_input_.ego_state_cart.ego_pose.x,
                   obstacle_decider_input_.ego_state_cart.ego_pose.y};
  if (frenet_coord_->CartCoord2FrenetCoord(ego_cart, ego_frenet) ==
      TRANSFORM_SUCCESS) {
    // all ddp points are behind ego, ignore
    if (s_max < ego_frenet.x && ego_speed > 3.0) {
      world_model_->set_use_eftp(false);
      MSD_LOG(ERROR,
              "ODC-real_ddp_traj: all ddp points are behind ego, ignore!");
      return;
    }
  }

  ddp_trajectories_.clear();
  ddp_trajectories_.emplace_back(ddp_trajectory);

  obstacle_decider_output_.current_lane_ddp_trajectory.clear();
  if (ddp_trajectory.trajectory.empty()) {
    MSD_LOG(INFO, "ODC-real_ddp_traj[%.1f][%d],size:[%d]",
            MTIME()->timestamp().sec() - odc_created_time_, use_eftp_,
            ddp_trajectory.trajectory.size());
    return;
  }
  for (const auto &ddp_pt : ddp_trajectory.trajectory) {
    odc_interface::TrajectoryPoint pt;
    pt.x = ddp_pt.x;
    pt.y = ddp_pt.y;
    pt.heading_angle = ddp_pt.heading_angle;
    pt.curvature = ddp_pt.curvature;

    pt.t = ddp_pt.t;
    pt.v = ddp_pt.v;
    pt.a = ddp_pt.a;
    pt.s = ddp_pt.s;
    pt.l = ddp_pt.l;
    pt.frenet_valid = ddp_pt.frenet_valid;

    odc_interface::Point2d pt_env{pt.x, pt.y};
    odc_interface::Point2d pt_local = env2local_pos(pt_env);

    pt.x_local = pt_local.x;
    pt.y_local = pt_local.y;

    obstacle_decider_output_.current_lane_ddp_trajectory.emplace_back(pt);
  }
}

ddp::DdpTrajectory ObstacleDecider::process_ddp_trajectory(
    const ddp::DdpTrajectory &ddp_trajectory) {
  // update map info
  auto target_lane_id = context_->planning_status().lane_status.target_lane_id;
  static std::vector<RefPointFrenet> cur_lane, left_lane, right_lane;
  cur_lane.clear();
  left_lane.clear();
  right_lane.clear();
  world_model_->get_map_lateral_info(cur_lane, left_lane, right_lane,
                                     target_lane_id);
  InterpolationData<RefPointFrenet> current_lane_map_info_pair_list;
  current_lane_map_info_pair_list.reserve(cur_lane.size());
  for (const auto &p : cur_lane) {
    current_lane_map_info_pair_list.emplace_back(p.s, p);
  }

  double s_last = 0.0;
  double l_last = 0.0;
  double v_last = 0.0;
  double t_last = 0.0;
  ddp::DdpTrajectory ddp_trajectory_processed = ddp_trajectory;
  double ego_width = veh_param_.width;
  for (auto &pt : ddp_trajectory_processed.trajectory) {
    Point2D pt_frenet{0.0, 0.0};
    Point2D pt_cart{pt.x, pt.y};
    if (frenet_coord_->CartCoord2FrenetCoord(pt_cart, pt_frenet) ==
        TRANSFORM_SUCCESS) {
      pt.s = pt_frenet.x;
      pt.l = pt_frenet.y;
      pt.frenet_valid = true;
      s_last = pt.s;
      l_last = pt.l;
      v_last = pt.v;
      t_last = pt.t;
    } else {
      pt.s = s_last + v_last * (pt.t - t_last);
      pt.l = l_last;
      pt.frenet_valid = true;
    }

    auto lane_info = LinearInterpation::interpolate<RefPointFrenet>(
        current_lane_map_info_pair_list, pt.s);

    MSD_LOG(INFO,
            "ODC-origin_ddp_point:s-l-heading(%.1f, %.1f, %.1f), "
            "l_border_limit:[%.1f, %.1f]",
            pt.s, pt.l, pt.heading_angle,
            -std::max(lane_info.right_lane_border - 0.5 * ego_width - 0.3, 0.0),
            std::max(lane_info.left_lane_border - 0.5 * ego_width - 0.3, 0.0));

    if (pt.l >= 0.0) {
      pt.l = std::min(
          pt.l,
          std::max(lane_info.left_lane_border - 0.5 * ego_width - 0.3, 0.0));
    } else {
      pt.l = -std::min(
          std::abs(pt.l),
          std::max(lane_info.right_lane_border - 0.5 * ego_width - 0.3, 0.0));
    }
  }
  return ddp_trajectory_processed;
}

void ObstacleDecider::conclude_trajectory_decision(void) {
  if (ddp_trajectories_.empty()) {
    return;
  }
  return;
}

ddp::DdpTrajectory ObstacleDecider::extend_ddp_trajectory(
    const ddp::DdpTrajectory &ddp_trajectory) {
  ddp::DdpTrajectory ddp_trajectory_extend = ddp_trajectory;
  double t_end = 10.0;
  double min_traj_len = 20.0;
  double ddp_traj_end_v = ddp_trajectory.trajectory.back().v;
  double ddp_traj_end_t = ddp_trajectory.trajectory.back().t;
  double ddp_traj_end_s = ddp_trajectory.trajectory.back().s;
  double ddp_traj_end_l = ddp_trajectory.trajectory.back().l;
  double s = ddp_traj_end_s;
  double l = ddp_traj_end_l;
  double time = ddp_traj_end_t;
  double t_delta = 0.1; // 10s
  double a = 3.0;
  double set_speed = 40.0;
  double v_end = ddp_traj_end_v;
  double v_end0 = ddp_traj_end_v;
  double frenet_s_max = std::max(frenet_coord_->GetLength(), min_traj_len);
  double frenet_end_heading = frenet_coord_->GetRefCurveHeading(frenet_s_max);

  MSD_LOG(INFO, "ODC-ddp_traj_extend[%.1f][%d]",
          MTIME()->timestamp().sec() - odc_created_time_,
          ddp_trajectory_extend.trajectory.size());

  while ((time <= t_end) || (s < min_traj_len)) {
    time += t_delta;
    v_end = min(v_end0 + a * t_delta, set_speed);
    s += t_delta * (v_end + v_end0) / 2.0;

    if (s > frenet_s_max) {
      break;
    }

    Point2D traj_extend_pt_frenet{s, l};
    Point2D traj_extend_pt_cart{0.0, 0.0};

    if (frenet_coord_->FrenetCoord2CartCoord(
            traj_extend_pt_frenet, traj_extend_pt_cart) == TRANSFORM_SUCCESS) {

      ddp::TrajectoryPoint traj_extend_pt;
      traj_extend_pt.x = traj_extend_pt_cart.x;
      traj_extend_pt.y = traj_extend_pt_cart.y;
      traj_extend_pt.s = traj_extend_pt_frenet.x;
      traj_extend_pt.l = traj_extend_pt_frenet.y;

      traj_extend_pt.heading_angle =
          frenet_coord_->GetRefCurveHeading(traj_extend_pt_frenet.x);
      traj_extend_pt.curvature =
          frenet_coord_->GetRefCurveCurvature(traj_extend_pt_frenet.x);

      traj_extend_pt.t = time;
      traj_extend_pt.v = v_end;
      traj_extend_pt.a = a;
      traj_extend_pt.frenet_valid = true;

      ddp_trajectory_extend.trajectory.emplace_back(traj_extend_pt);
    } else {
      ddp::TrajectoryPoint traj_extend_pt;
      traj_extend_pt.x = traj_extend_pt_cart.x;
      traj_extend_pt.y = traj_extend_pt_cart.y;
      traj_extend_pt.s = traj_extend_pt_frenet.x;
      traj_extend_pt.l = traj_extend_pt_frenet.y;

      traj_extend_pt.heading_angle = frenet_end_heading;
      traj_extend_pt.curvature = 0.0;

      traj_extend_pt.t = time;
      traj_extend_pt.v = v_end;
      traj_extend_pt.a = a;
      traj_extend_pt.frenet_valid = true;

      ddp_trajectory_extend.trajectory.emplace_back(traj_extend_pt);
    }
    MSD_LOG(INFO, "ODC-ddp_traj_extend[%.1f][%.1f] %.1f,%.1f",
            MTIME()->timestamp().sec() - odc_created_time_, time,
            ddp_trajectory_extend.trajectory.back().s,
            ddp_trajectory_extend.trajectory.back().l);
    v_end0 = v_end;
  }

  MSD_LOG(INFO, "ODC-ddp_traj_extend[%.1f][%d]",
          MTIME()->timestamp().sec() - odc_created_time_,
          ddp_trajectory_extend.trajectory.size());

  return ddp_trajectory_extend;
}

void ObstacleDecider::conclude_lateral_obstacle_decision(void) {
  if (ddp_trajectories_.empty()) {
    return;
  }
  const auto ddp_trajectory = ddp_trajectories_.front();

  const auto &obstacle_manager_ = odc_baseline_info_->obstacle_manager();
  const auto &all_obstacles = obstacle_manager_.get_obstacles();
  Point2D ego_pos_cart{obstacle_decider_input_.ego_state_cart.ego_pose.x,
                       obstacle_decider_input_.ego_state_cart.ego_pose.y};

  double time = 0.0;
  int obj_id = -1;
  int ddp_traj_point_index = 0;

  double ego_length = veh_param_.length;
  double ego_width = veh_param_.width;
  pair<double, double> ego_size = make_pair(ego_length, ego_width);

  vector<pair<int, int>> obj_decisions; // id, collision_posture

  double longi_thres = 35.0;
  vector<odc_interface::ObsInfo> obj_infos;

  Point2D ego_frenet0{0.0, 0.0};
  if (frenet_coord_->CartCoord2FrenetCoord(ego_pos_cart, ego_frenet0) ==
      TRANSFORM_SUCCESS) {
  } else {
    return;
  }

  for (const auto &ptr_obj : all_obstacles.Items()) {
    double obj_s = 0.0;
    double obj_l = 0.0;
    double obj_vel = 0.0;
    double t_delta = 0.0;
    obj_id = ptr_obj->Id();
    double obj_length = ptr_obj->PerceptionBoundingBox().length();
    double obj_width = ptr_obj->PerceptionBoundingBox().width();
    pair<double, double> obj_size = make_pair(obj_length, obj_width);
    int overlap_posture = 0;
    bool obj_ignore = false;
    ddp_traj_point_index = 0;
    Point2D obj_pos_frenet{0.0, 0.0};
    Point2D obj_pos_cart{0.0, 0.0};
    for (const auto &pt : ddp_trajectory.trajectory) {
      // all overlap check is based on frenet coordinate. DONNOT use cart!
      if (ddp_traj_point_index == 0) {
        const auto &traj_point0 = ptr_obj->GetPointAtTime(0.0);
        obj_vel = traj_point0.v;
        Point2D obj_traj_point_cart0{traj_point0.path_point.x,
                                     traj_point0.path_point.y};
        obj_pos_cart = obj_traj_point_cart0;
        Point2D obj_traj_point_frenet0{0.0, 0.0};
        if (frenet_coord_->CartCoord2FrenetCoord(obj_traj_point_cart0,
                                                 obj_traj_point_frenet0) ==
            TRANSFORM_SUCCESS) {
          obj_pos_frenet = obj_traj_point_frenet0;
          obj_s = obj_traj_point_frenet0.x;
          obj_l = obj_traj_point_frenet0.y;
          overlap_posture = check_overlap(obj_traj_point_frenet0, obj_size,
                                          ego_frenet0, ego_size);
        } else {
          obj_ignore = true;
          break;
        }
      }

      if (overlap_posture != 0) {
        break;
      }

      t_delta = pt.t - time;
      time = pt.t;

      if (time < 4.0) {
        const auto &traj_point = ptr_obj->GetPointAtTime(time);
        obj_vel = traj_point.v;

        Point2D obj_traj_point_cart{traj_point.path_point.x,
                                    traj_point.path_point.y};
        Point2D obj_traj_point_frenet{0.0, 0.0};
        if (frenet_coord_->CartCoord2FrenetCoord(obj_traj_point_cart,
                                                 obj_traj_point_frenet) ==
            TRANSFORM_SUCCESS) {
          obj_s = obj_traj_point_frenet.x;
          obj_l = obj_traj_point_frenet.y;
        } else {
          obj_s += obj_vel * t_delta;
        }
      } else {
        obj_s += obj_vel * t_delta;
      }

      MSD_LOG(INFO, "ODC obj_lat_decision1[%.1f][%d]:(%.1f, %.1f)m",
              MTIME()->timestamp().sec() - odc_created_time_, obj_id, obj_s,
              obj_l);

      Point2D obj_pred_point_frenet{obj_s, obj_l};
      Point2D ego_traj_frenet{pt.s, pt.l};

      overlap_posture = check_overlap(obj_pred_point_frenet, obj_size,
                                      ego_traj_frenet, ego_size);
      if (overlap_posture != 0) {
        break;
      }
      ddp_traj_point_index++;
    }

    if (overlap_posture != 0) {
      obj_decisions.emplace_back(obj_id, overlap_posture);
      MSD_LOG(INFO, "ODC obj_lat_decision2[%.1f][%d][%d]:(%.1f, %.1f)m",
              MTIME()->timestamp().sec() - odc_created_time_, obj_id,
              overlap_posture, obj_pos_frenet.x, obj_pos_frenet.y);

      odc_interface::ObsInfo obj_info;
      obj_info.lat_decision = odc_interface::ObsInfo::LatDecision::NUDGE;
      obj_info.lon_decision = odc_interface::ObsInfo::LonDecision::LON_IGNORE;
      if (overlap_posture == 1) {
        obj_info.nudge_side = odc_interface::ObsInfo::NudgeType::LEFT_NUDGE;
      } else {
        obj_info.nudge_side = odc_interface::ObsInfo::NudgeType::RIGHT_NUDGE;
      }
      obj_info.id = ptr_obj->Id();

      odc_interface::Point2d obj_init_pos_env{obj_pos_cart.x, obj_pos_cart.y};
      odc_interface::Point2d obj_init_pos_local =
          env2local_pos(obj_init_pos_env);

      obj_info.obj_state_init.obj_state_local.pos.x = obj_init_pos_local.x;
      obj_info.obj_state_init.obj_state_local.pos.y = obj_init_pos_local.y;
      obstacle_decider_output_.obs_infos.emplace_back(obj_info);

      if (obstacle_decider_output_.obs_infos_map.find(obj_info.id) ==
          obstacle_decider_output_.obs_infos_map.end()) {
        obstacle_decider_output_.obs_infos_map[obj_info.id] = obj_info;
      } else {
        obstacle_decider_output_.obs_infos_map[obj_info.id].lat_decision =
            obj_info.lat_decision;
        obstacle_decider_output_.obs_infos_map[obj_info.id].nudge_side =
            obj_info.nudge_side;
      }
    }
  }

  return;
}

void ObstacleDecider::conclude_longitudinal_obstacle_decision(void) {
  if (ddp_trajectories_.empty()) {
    return;
  }
  const auto ddp_trajectory = extend_ddp_trajectory(ddp_trajectories_.front());

  const auto &obstacle_manager_ = odc_baseline_info_->obstacle_manager();
  const auto &all_obstacles = obstacle_manager_.get_obstacles();

  Point2D ego_pos_cart{obstacle_decider_input_.ego_state_cart.ego_pose.x,
                       obstacle_decider_input_.ego_state_cart.ego_pose.y};
  Point2D ego_frenet0{0.0, 0.0};

  double time = 0.0;
  double t_delta = 0.2;
  int obj_id = -1;
  int ddp_traj_point_index = 0;

  double ego_length = veh_param_.length;
  double ego_width = veh_param_.width;
  pair<double, double> ego_size = make_pair(ego_length, ego_width);

  vector<pair<int, int>> obj_decisions; // id, collision_posture

  if (frenet_coord_->CartCoord2FrenetCoord(ego_pos_cart, ego_frenet0) ==
      TRANSFORM_SUCCESS) {

  } else {
    return;
  }
  vector<odc_interface::ObsInfo> obj_infos;
  for (const auto &ptr_obj : all_obstacles.Items()) {
    double obj_s = 0.0;
    double obj_l = 0.0;
    double obj_vel = 0.0;
    obj_id = ptr_obj->Id();
    double obj_length = ptr_obj->PerceptionBoundingBox().length();
    double obj_width = ptr_obj->PerceptionBoundingBox().width();
    pair<double, double> obj_size = make_pair(obj_length, obj_width);

    int overlap_posture = 0;
    bool obj_ignore = false;
    ddp_traj_point_index = 0;
    Point2D obj_pos_frenet{0.0, 0.0};
    Point2D obj_pos_cart{0.0, 0.0};
    time = 0.0;
    int check_index = 0;

    while (time < 2.0) {
      const auto &traj_point = ptr_obj->GetPointAtTime(time);
      obj_vel = traj_point.v;

      Point2D obj_traj_point_cart{traj_point.path_point.x,
                                  traj_point.path_point.y};
      Point2D obj_traj_point_frenet{0.0, 0.0};

      if (frenet_coord_->CartCoord2FrenetCoord(obj_traj_point_cart,
                                               obj_traj_point_frenet) ==
          TRANSFORM_SUCCESS) {
        obj_s = obj_traj_point_frenet.x;
        obj_l = obj_traj_point_frenet.y;
        if (check_index == 0) {
          obj_pos_frenet = obj_traj_point_frenet;
          obj_pos_cart = obj_traj_point_cart;
        }
        const auto &adc_sl_boundary = odc_baseline_info_->get_adc_sl_boundary();
        if (check_index == 0 &&
            (adc_sl_boundary.end_s > (obj_s + 0.5 * obj_length))) {
          obj_ignore = true;
          break;
        }
      } else {
        if (check_index == 0) {
          obj_ignore = true;
          break;
        } else {
          obj_s += obj_vel * t_delta;
        }
      }

      Point2D obj_pred_point_frenet{obj_s, obj_l};
      overlap_posture = check_overlap(obj_pred_point_frenet, obj_size,
                                      ddp_trajectory, ego_size);
      if (overlap_posture != 0) {
        odc_interface::ObsInfo obj_info;
        obj_info.lon_decision = odc_interface::ObsInfo::LonDecision::FOLLOW;
        obj_info.lat_decision = odc_interface::ObsInfo::LatDecision::LAT_IGNORE;
        obj_info.id = ptr_obj->Id();

        odc_interface::Point2d obj_init_pos_env{obj_pos_cart.x, obj_pos_cart.y};
        odc_interface::Point2d obj_init_pos_local =
            env2local_pos(obj_init_pos_env);

        obj_info.obj_state_init.obj_state_local.pos.x = obj_init_pos_local.x;
        obj_info.obj_state_init.obj_state_local.pos.y = obj_init_pos_local.y;
        obstacle_decider_output_.obs_infos.emplace_back(obj_info);

        if (obstacle_decider_output_.obs_infos_map.find(obj_info.id) ==
            obstacle_decider_output_.obs_infos_map.end()) {
          obstacle_decider_output_.obs_infos_map[obj_info.id] = obj_info;
        } else {
          obstacle_decider_output_.obs_infos_map[obj_info.id].lon_decision =
              obj_info.lon_decision;
          obstacle_decider_output_.obs_infos_map[obj_info.id].lat_decision =
              obj_info.lat_decision;
        }
        break;
      }
      time += t_delta;
      check_index++;
    }

    if (obj_ignore) {
      continue;
    }

    if (overlap_posture != 0) {
      obj_decisions.emplace_back(obj_id, overlap_posture);
      MSD_LOG(INFO, "ODC obj_long_decision[%.2f][%d][%d]:(%.1f, %.1f)m, %.1f",
              MTIME()->timestamp().sec() - odc_created_time_, obj_id,
              overlap_posture, obj_pos_frenet.x, obj_pos_frenet.y,
              frenet_coord_->GetLength());
    }
  }
  return;
}

void ObstacleDecider::conclude_longitudinal_obstacle_decision_new(void) {
  if (ddp_trajectories_.empty()) {
    return;
  }
  const auto ddp_trajectory = extend_ddp_trajectory(ddp_trajectories_.front());
  const auto &all_obstacles =
      odc_baseline_info_->obstacle_manager().get_obstacles();
  auto vitual_lane = generate_ddp_virtual_lane(ddp_trajectory);

  MSD_LOG(INFO,
          "ODC-generate_ddp_virtual_lane[%.1f] ddp_trajectory_length_for_lon = "
          "[%.1f]",
          MTIME()->timestamp().sec() - odc_created_time_,
          ddp_trajectory.trajectory.back().s);

  for (const auto &ptr_obj : all_obstacles.Items()) {
    MSD_LOG(INFO, "ODC-pick_lon_object[%.1f] start to check object-[%d] ",
            MTIME()->timestamp().sec() - odc_created_time_, ptr_obj->Id());
    double obj_length = ptr_obj->PerceptionBoundingBox().length();
    double obj_width = ptr_obj->PerceptionBoundingBox().width();
    const auto &object_center = ptr_obj->GetPointAtTime(0.0);
    Point2D obj_traj_point_cart{object_center.path_point.x,
                                object_center.path_point.y};
    Point2D obj_traj_point_frenet{0.0, 0.0};

    if (frenet_coord_->CartCoord2FrenetCoord(
            obj_traj_point_cart, obj_traj_point_frenet) == TRANSFORM_SUCCESS) {
      double obj_s = obj_traj_point_frenet.x;
      double obj_l = obj_traj_point_frenet.y;

      const auto &adc_sl_boundary = odc_baseline_info_->get_adc_sl_boundary();
      MSD_LOG(INFO,
              "ODC-pick_lon_object-[%d] ego_end_s = [%.1f], obj_end_s = [%.1f]",
              ptr_obj->Id(), adc_sl_boundary.end_s, obj_s + 0.5 * obj_length);

      if (adc_sl_boundary.end_s > (obj_s + 0.5 * obj_length)) {
        MSD_LOG(INFO, "ODC-pick_lon_object-[%d] is behind ego, ignore",
                ptr_obj->Id());
        continue;
      }
    } else {
      MSD_LOG(INFO, "ODC-pick_lon_object-[%d] start pos sl convert fail",
              ptr_obj->Id());
      continue;
    }

    double prediction_use_length_time_s = 2.0;
    double delta_t = 0.2;
    for (size_t i = 0; i <= int(prediction_use_length_time_s / delta_t); ++i) {
      double t = i * delta_t;
      auto traj_point = ptr_obj->GetPointAtTime(t);
      auto obs_enu_polygon = ptr_obj->GetPolygonAtPoint(traj_point);

      // for debug
      {
        Point2D cart_point(traj_point.path_point.x, traj_point.path_point.y);
        Point2D fren_point;
        if (frenet_coord_->CartCoord2FrenetCoord(cart_point, fren_point) ==
            TRANSFORM_SUCCESS) {
          MSD_LOG(
              INFO,
              "ODC-pick_lon_object-[%d] convert predict point success t-[%f] "
              "predict_xy:(%.1f, %.1f) sl:(%.1f, %.1f)",
              ptr_obj->Id(), t, cart_point.x, cart_point.y, fren_point.x,
              fren_point.y);
        } else {
          MSD_LOG(INFO,
                  "ODC-pick_lon_object-[%d] convert predict point fail t-[%f] "
                  "predict_xy:(%.1f, %.1f) sl:(%.1f, %.1f)",
                  ptr_obj->Id(), t, cart_point.x, cart_point.y, fren_point.x,
                  fren_point.y);
        }
      }

      static std::vector<planning_math::Vec2d> fren_vertexes;
      static planning_math::Polygon2d convex_polygon;
      convex_polygon.clear();
      fren_vertexes.clear();
      for (auto cart_vertex : obs_enu_polygon.GetAllVertices()) {
        Point2D cart_point, fren_point;
        cart_point.x = cart_vertex.x();
        cart_point.y = cart_vertex.y();
        if (frenet_coord_->CartCoord2FrenetCoord(cart_point, fren_point) ==
                TRANSFORM_FAILED ||
            std::isnan(fren_point.x) || std::isnan(fren_point.y)) {
          MSD_LOG(INFO,
                  "ODC-pick_lon_object-[%d] polygon_point convert fail "
                  "xy:(%.1f, %.1f) sl:(%.1f, %.1f)",
                  ptr_obj->Id(), cart_point.x, cart_point.y, fren_point.x,
                  fren_point.y);
          continue;
        } else {
          MSD_LOG(INFO,
                  "ODC-pick_lon_object-[%d] polygon_point convert success "
                  "xy:(%.1f, %.1f) sl:(%.1f, %.1f)",
                  ptr_obj->Id(), cart_point.x, cart_point.y, fren_point.x,
                  fren_point.y);
          fren_vertexes.emplace_back(fren_point.x, fren_point.y);
        }
      }
      if (!planning_math::Polygon2d::ComputeConvexHull(fren_vertexes,
                                                       &convex_polygon)) {
        MSD_LOG(INFO, "ODC-pick_lon_object-[%d] ComputeConvexHull fail",
                ptr_obj->Id());
        continue;
      } else {
        for (auto frenet_vertex : convex_polygon.GetAllVertices()) {
          MSD_LOG(INFO,
                  "ODC-pick_lon_object-[%d] ComputeConvexHull success "
                  "sl:(%.1f, %.1f)",
                  ptr_obj->Id(), frenet_vertex.x(), frenet_vertex.y());
        }
        const auto &adc_sl_boundary = odc_baseline_info_->get_adc_sl_boundary();
        const auto &ego_heading =
            odc_baseline_info_->get_ego_state().ego_pose.theta;
        MSD_LOG(
            INFO,
            "ODC-pick_lon_object-[%d] ego_start_l = %.1f, ego_end_l = "
            "%.1f, ego_start_s = %.1f, ego_end_s = %.1f, ego_heading = %.1f",
            ptr_obj->Id(), adc_sl_boundary.start_l, adc_sl_boundary.end_l,
            adc_sl_boundary.start_s, adc_sl_boundary.end_s, ego_heading);
      }

      auto s_range =
          StGraphGenerator::sl_polygon_path_check(vitual_lane, convex_polygon);
      MSD_LOG(INFO,
              "ODC-pick_lon_object[%.1f] object-[%d] s_range [%.1f, %.1f],  "
              "ddp_trajectory_end_s [%.1f]",
              MTIME()->timestamp().sec() - odc_created_time_, ptr_obj->Id(),
              s_range.first, s_range.second,
              ddp_trajectory.trajectory.back().s);
      if (s_range.first < ddp_trajectory.trajectory.back().s) {
        MSD_LOG(INFO,
                "ODC-pick_lon_object[%.1f] object-[%d] is chosen lon object",
                MTIME()->timestamp().sec() - odc_created_time_, ptr_obj->Id());
        odc_interface::ObsInfo obj_info;
        obj_info.lon_decision = odc_interface::ObsInfo::LonDecision::FOLLOW;
        obj_info.lat_decision = odc_interface::ObsInfo::LatDecision::LAT_IGNORE;
        obj_info.id = ptr_obj->Id();

        odc_interface::Point2d obj_init_pos_env{obj_traj_point_cart.x,
                                                obj_traj_point_cart.y};
        odc_interface::Point2d obj_init_pos_local =
            env2local_pos(obj_init_pos_env);

        obj_info.obj_state_init.obj_state_local.pos.x = obj_init_pos_local.x;
        obj_info.obj_state_init.obj_state_local.pos.y = obj_init_pos_local.y;
        obstacle_decider_output_.obs_infos.emplace_back(obj_info);

        if (obstacle_decider_output_.obs_infos_map.find(obj_info.id) ==
            obstacle_decider_output_.obs_infos_map.end()) {
          obstacle_decider_output_.obs_infos_map[obj_info.id] = obj_info;
        } else {
          obstacle_decider_output_.obs_infos_map[obj_info.id].lon_decision =
              obj_info.lon_decision;
          obstacle_decider_output_.obs_infos_map[obj_info.id].lat_decision =
              obj_info.lat_decision;
        }
        break;
      }
    }
  }
}

std::vector<planning_math::Polygon2d>
ObstacleDecider::generate_ddp_virtual_lane(
    const ddp::DdpTrajectory &ddp_trajectory) {
  std::vector<planning_math::Polygon2d> ddp_virtual_lane;

  const auto &adc_sl_boundary = odc_baseline_info_->get_adc_sl_boundary();
  const double enough_far_dist = 10.0;
  const double max_expand_width = 0.6;
  auto *planner_debug = PlanningContext::Instance()->mutable_planner_debug();
  planner_debug->lon_debug_info.left_virtual_line.clear();
  planner_debug->lon_debug_info.right_virtual_line.clear();

  for (const auto &point : ddp_trajectory.trajectory) {
    double ego_front_dist = point.s - adc_sl_boundary.end_s;
    double ego_length = veh_param_.length;
    double ego_width = veh_param_.width;
    if (ego_front_dist <= ego_length) {
      ego_width += max_expand_width;
    } else if (ego_front_dist < enough_far_dist) {
      ego_width +=
          max_expand_width * (1.0 - std::min((ego_front_dist - ego_length) /
                                                 (enough_far_dist - ego_length),
                                             1.0));
    }
    MSD_LOG(INFO,
            "ODC-generate_ddp_virtual_lane[%.1f] point_sl_heading(%.1f, %.1f, "
            "%.1f) ego_front_dist = %.1f, width = %.2f",
            MTIME()->timestamp().sec() - odc_created_time_, point.s, point.l,
            point.heading_angle, ego_front_dist, ego_width);
    auto ego_enu_box = planning_math::Box2d(
        {point.x, point.y}, point.heading_angle, ego_length, ego_width);
    auto ego_enu_polygon = planning_math::Polygon2d(ego_enu_box);
    static std::vector<planning_math::Vec2d> fren_vertexes;
    static planning_math::Polygon2d convex_polygon;
    convex_polygon.clear();
    fren_vertexes.clear();

    for (const auto &cart_vertex : ego_enu_polygon.GetAllVertices()) {
      Point2D cart_point(cart_vertex.x(), cart_vertex.y());
      Point2D fren_point;
      if (frenet_coord_->CartCoord2FrenetCoord(cart_point, fren_point) ==
              TRANSFORM_FAILED ||
          std::isnan(fren_point.x) || std::isnan(fren_point.y)) {
        continue;
      }
      fren_vertexes.emplace_back(fren_point.x, fren_point.y);
    }

    if (!planning_math::Polygon2d::ComputeConvexHull(fren_vertexes,
                                                     &convex_polygon)) {
      for (const auto &frenet_vertex : convex_polygon.GetAllVertices()) {
        MSD_LOG(INFO,
                "ODC-generate_ddp_virtual_lane ComputeConvexHull fail "
                "sl:(%.1f, %.1f)",
                frenet_vertex.x(), frenet_vertex.y());
      }
    } else {
      for (const auto &frenet_vertex : convex_polygon.GetAllVertices()) {
        MSD_LOG(INFO,
                "ODC-generate_ddp_virtual_lane ComputeConvexHull success "
                "sl:(%.1f, %.1f)",
                frenet_vertex.x(), frenet_vertex.y());
      }
    }

    // for debug script
    if (use_eftp_ && (convex_polygon.GetAllVertices().size() == 4)) {
      auto frenet_points = convex_polygon.GetAllVertices();
      std::sort(frenet_points.begin(), frenet_points.end(),
                [](const auto &a, const auto &b) { return a.x() > b.x(); });
      std::vector<Vec2d> front_points{frenet_points[0], frenet_points[1]};
      std::sort(front_points.begin(), front_points.end(),
                [](const auto &a, const auto &b) { return a.y() > b.y(); });

      Point2D left_front_point(front_points[0].x(), front_points[0].y());
      Point2D cart_point;
      if (frenet_coord_->FrenetCoord2CartCoord(left_front_point, cart_point) ==
          TRANSFORM_SUCCESS) {
        auto left_border_point = env2local_pos(cart_point);
        planner_debug->lon_debug_info.left_virtual_line.push_back(
            left_border_point);
      } else {
        MSD_LOG(INFO, "ODC-generate_ddp_virtual_lane[for debug] convert left "
                      "border point fail");
      }
      Point2D right_front_point(front_points[1].x(), front_points[1].y());
      if (frenet_coord_->FrenetCoord2CartCoord(right_front_point, cart_point) ==
          TRANSFORM_SUCCESS) {
        auto right_border_point = env2local_pos(cart_point);
        planner_debug->lon_debug_info.right_virtual_line.push_back(
            right_border_point);
      } else {
        MSD_LOG(INFO, "ODC-generate_ddp_virtual_lane[for debug] convert right "
                      "border point fail");
      }
    }

    ddp_virtual_lane.push_back(convex_polygon);
  }

  return ddp_virtual_lane;
}

void ObstacleDecider::set_debug_info(void) {
  auto *planner_debug = context_->mutable_planner_debug();
  planner_debug->odc_output = obstacle_decider_output_;
}

int ObstacleDecider::check_overlap(const Point2D &obj_frenet,
                                   const pair<double, double> &obj_size,
                                   const ddp::DdpTrajectory &ddp_trajectory,
                                   const pair<double, double> &ego_size) {
  double obj_s = obj_frenet.x;
  double obj_l = obj_frenet.y;
  double ego_s0 = odc_baseline_info_->get_ego_state().planning_start_state.s;
  int has_overlap = 0;

  if (ddp_trajectory.trajectory.empty()) {
    return has_overlap;
  }
  if (obj_s >= ddp_trajectory.trajectory.back().s) {
    return has_overlap;
  }

  double ego_length = ego_size.first;
  double ego_width = ego_size.second;
  double obj_length = obj_size.first;
  double obj_width = obj_size.second;

  const auto &adc_sl_boundary = odc_baseline_info_->get_adc_sl_boundary();
  double enough_far_dist = 10.0;
  double max_expand_width = 0.6;
  double head_to_head_dist = obj_s + 0.5 * obj_length - adc_sl_boundary.end_s;
  if (head_to_head_dist <= ego_length) {
    ego_width += max_expand_width;
  } else if (head_to_head_dist < enough_far_dist) {
    ego_width += max_expand_width * (1.0 - (head_to_head_dist - ego_length) /
                                               (enough_far_dist - ego_length));
  }
  auto cmp = [](const double &b, const ddp::TrajectoryPoint &a) {
    return b <= a.s;
  };

  auto s_index = std::upper_bound(ddp_trajectory.trajectory.begin(),
                                  ddp_trajectory.trajectory.end(), obj_s, cmp);

  ddp::TrajectoryPoint nearest_pt = *s_index;
  int nearest_pt_index = -1;

  for (int i = 1; i < ddp_trajectory.trajectory.size(); ++i) {
    if (ddp_trajectory.trajectory[i - 1].s < obj_s &&
        ddp_trajectory.trajectory[i].s >= obj_s) {
      nearest_pt_index = i - 1;
      break;
    }
  }
  if (nearest_pt_index == -1) {
    return has_overlap;
  }
  double nearest_pt_l =
      0.5 * (ddp_trajectory.trajectory[nearest_pt_index].l +
             ddp_trajectory.trajectory[nearest_pt_index + 1].l);
  double nearest_pt_s =
      0.5 * (ddp_trajectory.trajectory[nearest_pt_index].s +
             ddp_trajectory.trajectory[nearest_pt_index + 1].s);

  MSD_LOG(INFO, "ODC obj_overlap[%.2f]:(%.2f, %.2f)m,%.1f %.1f",
          MTIME()->timestamp().sec() - odc_created_time_, obj_l, nearest_pt_l,
          obj_s - ego_s0, nearest_pt_s - ego_s0);

  if (abs(nearest_pt_l - obj_l) < 0.5 * (ego_width + obj_width)) {
    has_overlap = 1;
  }

  return has_overlap;
}

int ObstacleDecider::check_overlap(const Point2D &obj_frenet,
                                   const pair<double, double> &obj_size,
                                   const Point2D &ego_frenet,
                                   const pair<double, double> &ego_size) {

  // pair: length, width
  int collision_posture = 0; // 0: no collision, 1: collision_head 2:
                             // collision_tail 3: collision side
  double ego_length = ego_size.first;
  double ego_width = ego_size.second;
  double obj_length = obj_size.first;
  double obj_width = obj_size.second;

  double longi_thres = 35.0;

  bool longitu_overlap = abs(obj_frenet.x - ego_frenet.x) <= longi_thres;
  bool should_longi_dodge = abs(obj_frenet.y - ego_frenet.y) <= 0.5 * obj_width;
  bool too_faraway = abs(obj_frenet.y - ego_frenet.y) > 5.2;

  if (longitu_overlap && !should_longi_dodge && !too_faraway) {
    if (obj_frenet.y > ego_frenet.y) {
      // right dodge
      collision_posture = 2;
    } else {
      // left dodge
      collision_posture = 1;
    }
  }

  return collision_posture;
}

TaskStatus ObstacleDecider::execute(ScenarioFacadeContext *context) {
  MLOG_PROFILING(name_.c_str());
  update_call_count();
  stamp_create_time();

  if (Task::execute(context) != TaskStatus::STATUS_SUCCESS) {
    return TaskStatus::STATUS_FAILED;
  }
  if (world_model_ == nullptr) {
    MSD_LOG(INFO, "ODC: world model is none!");
    return TaskStatus::STATUS_FAILED;
  }
  if (!baseline_info_ || !baseline_info_->is_valid()) {
    MSD_LOG(INFO, "ODC: baseline info is invalid!");
    return TaskStatus::STATUS_FAILED;
  }
  odc_baseline_info_ = world_model_->get_baseline_info(0);
  if (!odc_baseline_info_ || !odc_baseline_info_->is_valid()) {
    MSD_LOG(INFO, "ODC: real_baseline info is invalid!");
    return TaskStatus::STATUS_FAILED;
  }

  double start_time = MTIME()->timestamp().sec();
  frenet_coord_ = odc_baseline_info_->get_frenet_coord();

  odc_preprocessor_->init(context_, world_model_, odc_baseline_info_);
  odc_preprocessor_->process();
  use_eftp_ = world_model_->use_eftp();

  clear_obstacle_decider_output();

  fake_ddp_trajectory();

  load_ddp_trajectory();

  load_ddp_multipath();

  conclude_trajectory_decision();

  // conclude_longitudinal_obstacle_decision();
  conclude_longitudinal_obstacle_decision_new();

  conclude_lateral_obstacle_decision();

  set_debug_info();

  set_obstacle_decider_output();

  double end_time = MTIME()->timestamp().sec();
  MSD_LOG(INFO, "ODC cost time[%.1f]: %.5f",
          MTIME()->timestamp().sec() - odc_created_time_,
          end_time - start_time);

  return TaskStatus::STATUS_SUCCESS;
}

void ObstacleDecider::set_obstacle_decider_output(void) {

  MSD_LOG(INFO, "ODC ddp manager [%.1f][%d]",
          MTIME()->timestamp().sec() - odc_created_time_,
          ddp::DdpContext::Instance()->planning_success());

  *ddp::DdpContext::Instance()->mutable_obstacle_decider_output() =
      obstacle_decider_output_;

  if (!ddp_trajectories_.empty()) {
    *ddp::DdpContext::Instance()->mutable_obstacle_decider_ddp_trajectory() =
        ddp_trajectories_.front();
  } else {
    ddp::DdpContext::Instance()
        ->mutable_obstacle_decider_ddp_trajectory()
        ->trajectory.clear();
  }
}

void ObstacleDecider::clear_obstacle_decider_output(void) {
  obstacle_decider_output_.obs_infos.clear();
  obstacle_decider_output_.obs_infos_map.clear();
}

odc_interface::Point2d
ObstacleDecider::env2local_pos(odc_interface::Point2d point_env) {
  using odc_interface::Point2d;
  auto &enu2car = world_model_->get_cart_ego_state_manager().get_enu2car();
  // auto &enu2car = enu2car_;
  Eigen::Vector3d car_point, enu_point;

  enu_point.x() = point_env.x;
  enu_point.y() = point_env.y;
  enu_point.z() = 0;

  car_point = enu2car * enu_point;

  Point2d point_local{};
  point_local.x = car_point.x();
  point_local.y = car_point.y();

  return point_local;
}

PointXY ObstacleDecider::env2local_pos(Point2D point_env) {
  auto &enu2car = world_model_->get_cart_ego_state_manager().get_enu2car();
  // auto &enu2car = enu2car_;
  Eigen::Vector3d car_point, enu_point;

  enu_point.x() = point_env.x;
  enu_point.y() = point_env.y;
  enu_point.z() = 0;

  car_point = enu2car * enu_point;

  PointXY point_local{};
  point_local.x = car_point.x();
  point_local.y = car_point.y();

  return point_local;
}

} // namespace msquare