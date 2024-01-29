#include "planner/behavior_planner/deciders/speed_boundary_decider.h"
#include "common/config_context.h"
#include "mtime_core/mtime.h"
#include "planning/common/common.h"
#include "planning/common/logging.h"

namespace msquare {

SpeedBoundaryDecider::SpeedBoundaryDecider(const TaskConfig &config)
    : Decider(config) {
  // mph_assert(config.has_speed_boundary_decider_config());
}

void SpeedBoundaryDecider::reset(const TaskConfig &config) {
  Task::reset(config);

  n_prebrake_curv_ = 0;
  st_graph_generator_ = nullptr;
  MSD_LOG(INFO, "reset!!!!");
}

void SpeedBoundaryDecider::unset() { st_graph_generator_ = nullptr; }

void SpeedBoundaryDecider::init(std::shared_ptr<WorldModel> world_model) {
  Task::init(world_model);
}

TaskStatus SpeedBoundaryDecider::process() {
  MLOG_PROFILING(name_.c_str());
  if (world_model_ == nullptr) {
    MSD_LOG(INFO, "world model is none!");
    return TaskStatus::STATUS_FAILED;
  }
  if (!baseline_info_ || !baseline_info_->is_valid()) {
    MSD_LOG(INFO, "baseline info is invalid!");
    return TaskStatus::STATUS_FAILED;
  }
  if (nullptr == st_graph_generator_) {
    st_graph_generator_ = std::make_shared<StGraphGenerator>(
        world_model_, baseline_info_, config_.speed_boundary_decider_config(),
        context_);
  }
  st_graph_generator_->update(world_model_, baseline_info_,
                              config_.speed_boundary_decider_config(),
                              context_);
  (void)st_graph_generator_->ComputeSTGraph();

  // set speed limit and smooth it to produce a raw speed data without
  // considering obstacle in ST graph
  set_speed_limit();

  return TaskStatus::STATUS_SUCCESS;
}

void SpeedBoundaryDecider::getTargetLaneCurv(
    double &vel_limit, double &a_cross, const double curv_turn,
    const double max_centrifugal_acceleration, const double curv_filter_param,
    const double ego_vel, const double pre_brake_curv,
    PlanningStatus *planning_status) {
  double ego_s = baseline_info_->get_ego_state().planning_start_state.s;
  double kLengthDs =
      config_.speed_boundary_decider_config().ds_length_curvature;
  double kStepDs = config_.speed_boundary_decider_config().ds_step_curvature;
  auto frenet_coord = baseline_info_->get_frenet_coord();
  const auto &map_info = world_model_->get_map_info();

  double kMinCurvature = config_.speed_boundary_decider_config().min_curvature;
  double c_max = 0.0;
  if (map_info.road_type() == Direction::GO_STRAIGHT ||
      map_info.distance_to_crossing() > 60.0) {
    double ss = ego_s + 10.0;
    double c_s[5], cs_v_lim;
    double ds_step = kStepDs;
    double c_max_point = 0.0;
    double c_mean = 0.0;
    double c_mean_polys = 0.0;
    double kLengthDs_acc =
        world_model_->get_map_info().current_refline_points().size() * 1.0;
    double kLengthDs_curv = kLengthDs;
    if (world_model_->is_acc_mode()) {
      kLengthDs_curv = kLengthDs_acc;
    }
    while (ss < std::min(ego_s + kLengthDs_curv, frenet_coord->GetSlength()) -
                    2.0 * kStepDs) {
      if (world_model_->is_acc_mode()) {
        break;
      }
      // cal curv by polys for ddmap
      // if (world_model_->is_ddmap()) {
      double current_refline_polys_max_x = 0.0;
      if (map_info.get_current_refline_polys_max_x(
              current_refline_polys_max_x)) {
        if (ss - ego_s < current_refline_polys_max_x) {
          if (map_info.get_curv_from_current_refline_polys(ss - ego_s,
                                                           c_mean_polys)) {
            c_mean = c_s[0] = c_s[1] = c_s[2] = c_s[3] = c_s[4] = c_mean_polys;
            MSD_LOG(INFO, "CURV_DEBUG, current_refline_polys c_mean: %f at %f",
                    c_mean_polys, ss - ego_s);
          }
        }
      } else {
        c_s[0] = std::abs(frenet_coord->GetRefCurveCurvature(
            std::min(ss, frenet_coord->GetSlength())));
        c_s[1] = std::abs(frenet_coord->GetRefCurveCurvature(
            std::min(ss - 2.0 * kStepDs, frenet_coord->GetSlength())));
        c_s[2] = std::abs(frenet_coord->GetRefCurveCurvature(
            std::min(ss - 1.0 * kStepDs, frenet_coord->GetSlength())));
        c_s[3] = std::abs(frenet_coord->GetRefCurveCurvature(
            std::min(ss + 1.0 * kStepDs, frenet_coord->GetSlength())));
        c_s[4] = std::abs(frenet_coord->GetRefCurveCurvature(
            std::min(ss + 2.0 * kStepDs, frenet_coord->GetSlength())));
        c_mean = (c_s[0] + c_s[2] + c_s[3]) / 3.0;
        MSD_LOG(INFO, "CURV_DEBUG, origin c_mean: %f at %f", c_mean, ss);
      }
      if (c_mean > c_max) {
        world_model_->set_max_curv(c_mean);
        world_model_->set_max_curv_distance(ss - ego_s);
      }
      c_max = c_mean > c_max ? c_mean : c_max;

      int cnt_num = 5;
      for (int i = 0; i < cnt_num; i++) {
        c_max_point = std::max(c_max_point, c_s[i]);
      }
      if (c_s[0] > curv_turn / 3 && c_s[2] > curv_turn / 3 &&
          c_s[3] > curv_turn / 3 && c_mean > curv_turn &&
          c_mean > c_max - 1e-5) {
        int n = 0;
        cs_v_lim = c_mean;
        double v_curv = std::sqrt(max_centrifugal_acceleration /
                                  (cs_v_lim + kMinCurvature));
        MSD_LOG(INFO, "CURV_DEBUG: v_curv = %.2f, vel_limit = %.2f", v_curv,
                vel_limit);
        if (vel_limit > v_curv) {
          vel_limit =
              (1 - curv_filter_param) * std::min(vel_limit, last_v_lim_curv_) +
              curv_filter_param * v_curv;
          MSD_LOG(INFO,
                  "CURV_DEBUG2: v_curv = %.2f, vel_limit = %.2f, "
                  "last_v_lim_curv_ = %.2f",
                  v_curv, vel_limit, last_v_lim_curv_);
          double lower_bound = -0.5;
          if (world_model_->is_ddmap()) {
            lower_bound = -1.5;
          }
          a_cross =
              (1 - curv_filter_param) * std::min(a_cross, last_a_cross_) +
              curv_filter_param *
                  std::min(a_cross, std::max(lower_bound,
                                             (vel_limit * vel_limit -
                                              std::pow(ego_vel, 2)) /
                                                 2.0 / (ss - ds_step - ego_s)));
          MSD_LOG(
              INFO,
              "CURV_DEBUG2: a_cross = %.2f, last_a_cross_ = %.2f, ss = %.2f",
              a_cross, last_a_cross_, ss);

          if (n_prebrake_curv_ == 0 ||
              vel_limit < planning_status->pre_action.v_lim_curv) {
            n_prebrake_curv_ = pre_brake_curv;
            planning_status->pre_action.v_lim_curv = vel_limit;
          }
          if (n_prebrake_curv_ == 0 ||
              a_cross < planning_status->pre_action.a_lim_curv) {
            n_prebrake_curv_ = pre_brake_curv;
            planning_status->pre_action.a_lim_curv = a_cross;
          }
        }
        MSD_LOG(INFO, "CURV_DEBUG2:2nd branch,acc_mode:%d,v_lim:%f",
                world_model_->is_acc_mode(), vel_limit);
        MSD_LOG(INFO, "CURV_DEBUG2:ss:%2.1f,c0:%f,c1:%f,c2:%f,c3:%f,c4:%f", ss,
                c_s[0], c_s[1], c_s[2], c_s[3], c_s[4]);
        MSD_LOG(INFO,
                "CURV_DEBUG: c_max = %f  c_mean = %f  c_lim=%f  c0 = %f  c1 = "
                "%f  c2 = %f  c3 = %f c4 = %f  ds = %f  v_ego = %f  vel_limit "
                "= %f  a_cross = %f",
                c_max, c_mean, cs_v_lim, c_s[1], c_s[2], c_s[0], c_s[3], c_s[4],
                ss - ego_s, ego_vel, vel_limit, a_cross);
      }
      ds_step = std::max(2.0, std::min(5.0, 0.016 / (c_mean + 1e-5)));
      ss += ds_step;
    }
  }
}

void SpeedBoundaryDecider::cp_curve_speed(double &vel_limit, double &a_cross,
                                          const double ego_vel,
                                          PlanningStatus *planning_status) {
  const bool dbw_status = world_model_->get_vehicle_dbw_status();
  const auto &vehicle_dbw_status_last =
      world_model_->get_last_vehicle_dbw_status();
  if (!dbw_status) {
    MSD_LOG(INFO, "Not in CP");
    return;
  }

  if ((dbw_status && !vehicle_dbw_status_last) || last_acc_mode_) {
    MSD_LOG(INFO, "just in CP");
    just_in_dbw_ = true;
  }

  if (just_in_dbw_) {
    get_dbw_time_++;
    if (get_dbw_time_ > kDBWWaitTime) {
      just_in_dbw_ = false;
      get_dbw_time_ = 0;
    } else {
      MSD_LOG(INFO, "return count CP = %d", get_dbw_time_);
      return;
    }
  }

  double max_centrifugal_acceleration =
      config_.speed_boundary_decider_config().max_centrifugal_acceleration;
  const double min_curvature =
      config_.speed_boundary_decider_config().min_curvature;
  double ego_s = baseline_info_->get_ego_state().planning_start_state.s;
  const auto &planning_result =
      context_->mutable_planning_status()->planning_result;
  const auto &traj_pose_array = planning_result.traj_pose_array;
  const auto &map_info = world_model_->get_map_info();
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord =
      baseline_info_->get_frenet_coord();
  double curv_turn = 0.006;
  double curv_filter_param = 1.0;
  int pre_brake_curv = 10;
  if (map_info.is_on_highway()) {
    curv_turn = 0.004;
  }

  if (world_model_->is_ddmap()) {
    curv_turn = 0.002;
    curv_filter_param = 0.35;
    pre_brake_curv = 20;

    const auto curv_limit_lat_acc_velocity_bp =
        ConfigurationContext::Instance()
            ->planner_config()
            .speed_boundary_decider_config.curv_limit_lat_acc_velocity_bp;

    auto curv_limit_lat_acc_value =
        ConfigurationContext::Instance()
            ->planner_config()
            .speed_boundary_decider_config.curv_limit_lat_acc_value;

    auto curv_limit_lat_acc_value_1 =
        ConfigurationContext::Instance()
            ->planner_config()
            .speed_boundary_decider_config.curv_limit_lat_acc_value_1;

    auto curv_limit_lat_acc_value_2 =
        ConfigurationContext::Instance()
            ->planner_config()
            .speed_boundary_decider_config.curv_limit_lat_acc_value_2;

    auto curv_limit_lat_acc_value_3 =
        ConfigurationContext::Instance()
            ->planner_config()
            .speed_boundary_decider_config.curv_limit_lat_acc_value_3;

    if (world_model_->get_driving_model_config() == SAFE_DRIVING_MODEL) {
      curv_limit_lat_acc_value = curv_limit_lat_acc_value_1;
    } else if (world_model_->get_driving_model_config() ==
               STEADY_DRIVING_MODEL) {
      curv_limit_lat_acc_value = curv_limit_lat_acc_value_2;
    } else if (world_model_->get_driving_model_config() ==
               RADICAL_DRIVING_MODEL) {
      curv_limit_lat_acc_value = curv_limit_lat_acc_value_3;
    }

    if (curv_limit_lat_acc_velocity_bp.size() > 0 &&
        (curv_limit_lat_acc_velocity_bp.size() ==
         curv_limit_lat_acc_value.size())) {
      max_centrifugal_acceleration = planning_math::interp(
          ego_vel, curv_limit_lat_acc_velocity_bp, curv_limit_lat_acc_value);
    }
    MSD_LOG(INFO, "CURV_DEBUG: max_centrifugal_acceleration: %f",
            max_centrifugal_acceleration);
  }

  if (map_info.road_type() == Direction::GO_STRAIGHT ||
      map_info.distance_to_crossing() > 60.0) {
    double cs_v_lim = 0.0;
    double c_max_point = 0.0;
    double c_mean = 0.0;
    double c_max = 0.0;

    auto lane_status = context_->mutable_planning_status()->lane_status;
    MSD_LOG(INFO,
            "CURV_DEBUG, lane_status.status = %d, "
            "lane_status.change_lane.status = %d",
            lane_status.status, lane_status.change_lane.status);

    if (lane_status.status == LaneStatus::Status::LANE_CHANGE &&
        lane_status.change_lane.status ==
            ChangeLaneStatus::Status::IN_CHANGE_LANE) {
      getTargetLaneCurv(vel_limit, a_cross, curv_turn,
                        max_centrifugal_acceleration, curv_filter_param,
                        ego_vel, pre_brake_curv, planning_status);
    } else {
      // cal curv by lateral_planning trajectory
      if (traj_pose_array.size()) {
        for (const auto &traj_pose : traj_pose_array) {
          Point2D cart, fren;
          cart.x = traj_pose.position_enu.x;
          cart.y = traj_pose.position_enu.y;
          baseline_info_->get_frenet_coord()->CartCoord2FrenetCoord(cart, fren);

          const double traj_pose_s = fren.x;
          c_mean = fabs(traj_pose.curvature);
          if (c_mean > c_max) {
            world_model_->set_max_curv(c_mean);
            world_model_->set_max_curv_distance(traj_pose_s - ego_s);
            c_max = c_mean;
          }
          // MSD_LOG(INFO, "c_mean = %.6f, c_max = %.6f, curv_turn = %.6f,
          // vel_limit = %.3f, dist_s = %.2f", c_mean,
          //         c_max, curv_turn, vel_limit, traj_pose_s - ego_s);

          if (c_mean > curv_turn) {
            cs_v_lim = c_mean;
            if (vel_limit > std::sqrt(max_centrifugal_acceleration /
                                      (cs_v_lim + min_curvature))) {
              vel_limit =
                  (1 - curv_filter_param) *
                      std::fmin(vel_limit, last_v_lim_curv_) +
                  curv_filter_param * std::sqrt(max_centrifugal_acceleration /
                                                (cs_v_lim + min_curvature));
              double lower_bound = -0.5;
              if (world_model_->is_ddmap()) {
                lower_bound = -1.5;
              }
              a_cross =
                  (1 - curv_filter_param) * std::fmin(a_cross, last_a_cross_) +
                  curv_filter_param *
                      std::fmin(a_cross,
                                std::fmax(lower_bound,
                                          (vel_limit * vel_limit -
                                           std::pow(ego_vel, 2)) /
                                              2.0 / (traj_pose_s - ego_s)));
              if (n_prebrake_curv_ == 0 ||
                  vel_limit < planning_status->pre_action.v_lim_curv) {
                n_prebrake_curv_ = pre_brake_curv;
                planning_status->pre_action.v_lim_curv = vel_limit;
              }
              if (n_prebrake_curv_ == 0 ||
                  a_cross < planning_status->pre_action.a_lim_curv) {
                n_prebrake_curv_ = pre_brake_curv;
                planning_status->pre_action.a_lim_curv = a_cross;
              }
            }
          }
        }
      }
    }
    MSD_LOG(INFO,
            "CURV_DEBUG  rt=%d c_s=%f     "
            "c_mean=%f  c_max=%f  v_curv=%f  v_lim=%f  "
            "v_ego=%f  n_curv_pre=%d  vlim_pre=%f alim_pre=%f",
            (int)map_info.road_type(),
            std::abs(frenet_coord->GetRefCurveCurvature(
                clip(ego_s, frenet_coord->GetSlength(), 0.0))),
            c_mean, c_max,
            std::sqrt(max_centrifugal_acceleration /
                      std::abs(frenet_coord->GetRefCurveCurvature(
                          clip(ego_s, frenet_coord->GetSlength(), 0.0)))),
            vel_limit, ego_vel, planning_status->pre_action.n_prebrake_curv,
            planning_status->pre_action.v_lim_curv,
            planning_status->pre_action.a_lim_curv);
  }
}

void SpeedBoundaryDecider::acc_curve_speed(double &vel_limit, double &a_cross,
                                           const double ego_vel) {
  // acc_mode
  double steer_angle = baseline_info_->get_ego_state().ego_steer_angle;
  static double sa_fil = 0;
  static double yr_fil = 0;
  double sa = steer_angle /
              ConfigurationContext::Instance()->get_vehicle_param().steer_ratio;
  double yawrate =
      std::fmin(std::fmax(world_model_->get_yawrate(), -0.2), 0.2); // rad/s
  sa_fil = (1 - kSteerAngleaFilter) * sa_fil + kSteerAngleaFilter * sa;
  yr_fil = (1 - kYawRateFilter) * yr_fil + kYawRateFilter * yawrate;

  double driver_curv = 0;
  double curv_k_lowspd = 0;
  if (ego_vel < kCurvKneePt1Mps) {
    curv_k_lowspd = 1.0;
  } else if (ego_vel < kCurvKneePt2Mps) {
    curv_k_lowspd =
        (ego_vel - kCurvKneePt1Mps) / (kCurvKneePt2Mps - kCurvKneePt1Mps);
  } else {
    curv_k_lowspd = 0;
  }

  const double wheel_base =
      ConfigurationContext::Instance()->get_vehicle_param().wheel_base;
  double curv_low_spd = sa_fil / wheel_base;
  double curv_high_spd = yr_fil / std::fmax(ego_vel, 0.1);
  driver_curv = std::fabs(curv_k_lowspd * curv_low_spd +
                          (1 - curv_k_lowspd) * curv_high_spd);
  vel_limit =
      std::fmin(std::sqrt(kMaxAy / std::fmax(driver_curv, 0.0001)), vel_limit);
  if (ego_vel > vel_limit) {
    a_cross = (vel_limit - ego_vel) / 1.0;
    a_cross = std::fmax(a_cross, -3.0);
  } else {
    a_cross = 0;
  }
  MSD_LOG(INFO,
          "VEL_DEBUG_CJ:vlim_curv:%2.2f,sa_fil:%2.2f,yr_fil:%2.2f,sa:%2.2f,"
          "yr:%2.2f,a_cross:%2.2f,ego_vel:%2.2f",
          vel_limit, sa_fil, yr_fil, sa, yawrate, a_cross, ego_vel);
  MSD_LOG(INFO, "CURV_DEBUG2:5th branch,acc_mode:%d,v_lim:%f",
          world_model_->is_acc_mode(), vel_limit);
}

void SpeedBoundaryDecider::set_speed_limit() {
  // cross speed limit
  auto planning_status = context_->mutable_planning_status();
  auto vehicle_state = baseline_info_->get_ego_state().mpc_vehicle_state;
  const double ego_vel = vehicle_state[0];

  // hack for new planner
  double vel_limit =
      std::fmax(33.3, world_model_->get_map_info().v_cruise() + 10.0 / 3.6);
  double a_cross = 0.0;
  n_prebrake_curv_ =
      std::max(0, planning_status->pre_action.n_prebrake_curv - 1);
  if (n_prebrake_curv_ == 0) {
    planning_status->pre_action.v_lim_curv = vel_limit;
    planning_status->pre_action.a_lim_curv = a_cross;
  }
  double curv_filter_param = 1.0;
  if (world_model_->is_ddmap()) {
    curv_filter_param = 0.35;
  }
  // last_v_lim_curv_ = vel_limit;
  // last_a_cross_ = a_cross;

  if (!world_model_->is_acc_mode()) {
    cp_curve_speed(vel_limit, a_cross, ego_vel, planning_status);
    last_acc_mode_ = false;
  } else {
    acc_curve_speed(vel_limit, a_cross, ego_vel);
    last_acc_mode_ = true;
  }

  if (n_prebrake_curv_ > 0) {
    vel_limit = std::min(vel_limit, planning_status->pre_action.v_lim_curv);
    a_cross = std::min(a_cross, planning_status->pre_action.a_lim_curv);
  }
  MSD_LOG(INFO, "CURV_DEBUG: n_prebrake_curv: %d, vel_limit: %f, a_cross: %f",
          n_prebrake_curv_, vel_limit, a_cross);

  MSD_LOG(INFO, "MODE_DEBUG:is_acc_mode:%d,is_ddmap:%d,is_active:%d",
          world_model_->is_acc_mode(), world_model_->is_ddmap(),
          world_model_->get_vehicle_dbw_status());
  last_v_lim_curv_ = vel_limit;
  last_a_cross_ = a_cross;
  world_model_->set_a_curv(a_cross);
  world_model_->set_v_curv(vel_limit);

  planning_status->v_limit = vel_limit;
  planning_status->a_limit = 0.0;
  planning_status->pre_action.n_prebrake_curv = n_prebrake_curv_;
}

std::shared_ptr<StGraphGenerator> SpeedBoundaryDecider::get_st_graph() {
  return st_graph_generator_;
}

} // namespace msquare
