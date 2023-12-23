#include "planner/behavior_planner/deciders/prediction_reviser.h"
#include "common/config_context.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "math.h"
#include "planner/behavior_planner/deciders/st_graph_generator.h"
#include "planner/motion_planner/common/speed_profile_generator.h"
#include "planning/common/common.h"
#include <cassert>

namespace msquare {

PredictionReviser::PredictionReviser(
    const std::shared_ptr<WorldModel> world_model,
    const std::shared_ptr<BaseLineInfo> baseline_info,
    ScenarioFacadeContext *context) {
  world_model_ = world_model;
  baseline_info_ = baseline_info;
  context_ = context;
}

bool PredictionReviser::lane_change_revise(
    std::shared_ptr<BaseLineInfo> target_baseline) {
  const auto &adc_sl_boundary = baseline_info_->get_adc_sl_boundary();
  auto &obstacles_items =
      baseline_info_->mutable_obstacle_manager().get_obstacles().Items();
  constexpr double kDefaultLCAcc = 0.4;
  double v_limit = context_->planning_status().v_limit;
  double lc_time = 4.0;
  double ego_v = baseline_info_->get_ego_state().ego_vel;
  double ego_s = baseline_info_->get_ego_state().ego_frenet.x;
  auto lc_speed_data = SpeedProfileGenerator::GenerateConstAccSpeed(
      v_limit, ego_v, kDefaultLCAcc, lc_time);
  for (auto &sp : lc_speed_data) {
    sp.s += ego_s;
  }
  // find leader car
  double init_follow_s = std::numeric_limits<double>::max();
  std::vector<int> leader_cars;
  for (auto &ptr_obstacle : obstacles_items) {
    const auto &st_boundary = ptr_obstacle->path_st_boundary();
    if (st_boundary.min_s() < adc_sl_boundary.start_s ||
        ptr_obstacle->PerceptionSLBoundary().end_s < adc_sl_boundary.start_s) {
      continue;
    }
    init_follow_s =
        std::min(init_follow_s, ptr_obstacle->PerceptionSLBoundary().start_s);
  }
  for (auto &ptr_obstacle : obstacles_items) {
    // for (auto id : target_lane_obstacles) {
    if (ptr_obstacle == nullptr) {
      continue;
    }
    auto ptr_obstacle_decision =
        context_->obstacle_decision_manager().find_obstacle_decision(
            ptr_obstacle->Id());
    if (ptr_obstacle->IsStatic() || !ptr_obstacle->has_sl_polygon_seq()) {
      continue;
    }
    const auto &sl_polygon_seq = ptr_obstacle->sl_polygon_seq();
    const auto &st_boundary = ptr_obstacle->path_st_boundary();
    if (st_boundary.IsEmpty() ||
        st_boundary.max_s() - 0.0 < adc_sl_boundary.start_s ||
        st_boundary.min_s() > adc_sl_boundary.end_s) {
      continue;
    }
    double collide_s = std::max(adc_sl_boundary.start_s, st_boundary.min_s());
    double obs_speed =
        ptr_obstacle->speed() * ptr_obstacle->Yaw_relative_frenet();
  }
}

double PredictionReviser::min_avoid_dec(Obstacle *obstacle,
                                        SpeedData speed_data) {
  double init_obs_v = obstacle->speed() * obstacle->Yaw_relative_frenet();
  double init_obs_s = obstacle->PerceptionSLBoundary().end_s;
  constexpr double kMinFollowDis = 2.0;
  constexpr double kSoftObsDec = -2.0;
  double min_avoid_dec = std::numeric_limits<double>::max();
  for (auto &sp : speed_data) {
    double upper_s =
        sp.s -
        ConfigurationContext::Instance()->get_vehicle_param().length / 2.0 -
        kMinFollowDis;
    double t = sp.t;
    double rel_s = upper_s - init_obs_s;
    if (rel_s < 0.0) {
      return std::numeric_limits<double>::lowest();
    }
    if (t < 1.e-2) {
      continue;
    }
    double average_obs_v = rel_s / t;
    double min_obs_v = std::max(0.0, init_obs_v - kSoftObsDec * t);
    // if (average_obs_v < (init_obs_v + min_obs_v) / 2.0) {
    //   return std::numeric_limits<double>::lowest();
    // }
    double cur_dec = 2.0 * (average_obs_v - init_obs_v) / t;
    if (init_obs_v - cur_dec * t < 0.0 || cur_dec < kSoftObsDec) {
      return std::numeric_limits<double>::lowest();
    }
    min_avoid_dec = std::min(min_avoid_dec, cur_dec);
  }
  return min_avoid_dec;
}

void PredictionReviser::revise_prediction_trajectory(Obstacle *obstacle,
                                                     SpeedData speed_data) {
  if (!obstacle->has_sl_polygon_seq()) {
    MSD_LOG(INFO, "[POMDP_PLANNERR] revise_prediction_trajectory failed %d",
            obstacle->Id());
    return;
  }
  const auto &sl_polygon_seq = obstacle->sl_polygon_seq();
  constexpr double kDeltaT = 0.2;
  constexpr double kPredictionTimeHorizon = 4.0;
  constexpr double kMinSweepDistance = 2.0;
  double min_obs_check_length =
      std::min(2.0, std::min(obstacle->PerceptionBoundingBox().width(),
                             obstacle->PerceptionBoundingBox().length()));
  int dimension = static_cast<int>(kPredictionTimeHorizon / kDeltaT);
  static SLPolygonSeq revised_sl_polygon_seq;
  revised_sl_polygon_seq.unset();

  // plan A: update sl_polygon_seq and do not consider prediction path
  planning_math::spline s_t_refline, s_v_refline;
  std::vector<double> t_seq, s_seq, v_seq;
  std::vector<TrajectoryPoint> revised_traj;
  for (auto point : speed_data) {
    t_seq.emplace_back(point.t);
    s_seq.emplace_back(point.s);
    v_seq.emplace_back(point.v);
  }
  s_t_refline.set_points(s_seq, t_seq);
  s_v_refline.set_points(s_seq, v_seq);
  double last_cal_dis = 0.0;
  auto frenet_coord = baseline_info_->get_frenet_coord();
  double obstacle_start_s = obstacle->GetPointAtTime(0.0).path_point.s;
  MSD_LOG(INFO,
          "[POMDP_PLANNER] revised obstacle %d start_s %f rival sp begin s %f",
          obstacle->Id(), obstacle_start_s, speed_data.front().s);
  for (size_t i = 0; i < dimension; ++i) {
    PolygonWithT cur_sl_polygon, revised_cur_sl_polygon;
    if (sl_polygon_seq.EvaluateByTime(i * kDeltaT, &cur_sl_polygon)) {
      double cur_s =
          (cur_sl_polygon.second.min_x() + cur_sl_polygon.second.max_x()) / 2.0;
      double cur_l =
          (cur_sl_polygon.second.min_y() + cur_sl_polygon.second.max_y()) / 2.0;
      if (cur_s - last_cal_dis < min_obs_check_length && i != 0 &&
          i != dimension - 1) {
        continue;
      }
      auto traj_point = obstacle->GetPointAtTime(i * kDeltaT);
      MSD_LOG(
          INFO,
          "[POMDP_PLANNER] before revision t %f cur_s %f cur_l %f vel %f s %f",
          i * kDeltaT, cur_s, cur_l, traj_point.v, traj_point.path_point.s);
      // double revised_t = s_t_refline.deriv(0, cur_s);
      // double revised_v = 1.0 / (1.e-3 + s_t_refline.deriv(1, cur_s));
      // revised_cur_sl_polygon = cur_sl_polygon;
      // revised_cur_sl_polygon.first = revised_t;
      // revised_sl_polygon_seq.emplace_back(revised_cur_sl_polygon);
      // TrajectoryPoint traj_point;
      // Point2D cart_point, frenet_point;
      // frenet_point.x = cur_s;
      // frenet_point.y = cur_l;
    }
  }
  if (!revised_sl_polygon_seq.empty()) {
    obstacle->SetSLPolygonSequence(revised_sl_polygon_seq);
  }

  // plan B: revise obstacle trajectory a.c. speed data and consider prediction
  // path
  const auto &traj = obstacle->Trajectory();
  DiscretizedPath discret_path;
  for (const auto &traj_point : traj) {
    discret_path.push_back(traj_point.path_point);
  }
  double begin_s = speed_data.front().s;
  for (size_t i = 0; i < dimension; ++i) {
    SpeedPoint sp;
    speed_data.EvaluateByTimeWithConstAcc(i * kDeltaT, &sp);
    if (sp.s - begin_s > discret_path.Length() || i >= traj.size()) {
      break;
    }
    auto point = discret_path.Evaluate(sp.s - begin_s + obstacle_start_s);
    // TrajectoryPoint traj_point = traj[i];
    TrajectoryPoint traj_point;
    traj_point.path_point = point;
    traj_point.v = sp.v;
    traj_point.relative_time = i * kDeltaT;

    revised_traj.emplace_back(traj_point);
  }
  obstacle->SetTrajectory(revised_traj);
  std::string fail_reason{};
  StGraphGenerator::compute_obs_sl_polygon_seq(
      obstacle, baseline_info_, {0.0, 8.0}, 0.2, true, fail_reason);

  const auto &new_sl_polygon_seq = obstacle->sl_polygon_seq();
  for (size_t i = 0; i < dimension; ++i) {
    PolygonWithT cur_sl_polygon, revised_cur_sl_polygon;
    if (new_sl_polygon_seq.EvaluateByTime(i * kDeltaT, &cur_sl_polygon)) {
      double cur_s =
          (cur_sl_polygon.second.min_x() + cur_sl_polygon.second.max_x()) / 2.0;
      double cur_l =
          (cur_sl_polygon.second.min_y() + cur_sl_polygon.second.max_y()) / 2.0;
      if (cur_s - last_cal_dis < min_obs_check_length && i != 0 &&
          i != dimension - 1) {
        continue;
      }
      auto traj_point = obstacle->GetPointAtTime(i * kDeltaT);
      MSD_LOG(INFO,
              "[POMDP_PLANNER] after revision t %f cur_s %f cur_l %f vel %f "
              "target s %f",
              i * kDeltaT, cur_s, cur_l, traj_point.v, traj_point.path_point.s);
      // MSD_LOG(INFO, "[POMDP_PLANNER] after revision t %f cur_s %f cur_l %f",
      // i * kDeltaT, cur_s, cur_l); double revised_t = s_t_refline.deriv(0,
      // cur_s); double revised_v = 1.0 / (1.e-3 + s_t_refline.deriv(1, cur_s));
      // revised_cur_sl_polygon = cur_sl_polygon;
      // revised_cur_sl_polygon.first = revised_t;
      // revised_sl_polygon_seq.emplace_back(revised_cur_sl_polygon);
      // TrajectoryPoint traj_point;
      // Point2D cart_point, frenet_point;
      // frenet_point.x = cur_s;
      // frenet_point.y = cur_l;
    }
  }
}

} // namespace msquare
