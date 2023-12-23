#include "common/extract_path_bound.h"
#include "common/config_context.h"
#include "common/math/math_utils.h"
#include "common/obstacle_manager.h"
#include "planner/behavior_planner/deciders/st_graph_generator.h"
#include "planner/behavior_planner/lateral_behavior_state.h"

namespace msquare {

ExtractPathBound::ExtractPathBound(
    ScenarioFacadeContext *context,
    const std::shared_ptr<WorldModel> &world_model,
    const LaneBorrowInfo lane_borrow_info,
    const std::shared_ptr<BaseLineInfo> &baseline_info,
    LateralBehaviorPlannerConfig config) {
  world_model_ = world_model;
  baseline_info_ = baseline_info;
  context_ = context;
  config_ = config;
  lane_borrow_info_ = lane_borrow_info;

  v_limit_.clear();
  a_limit_.clear();
  lat_offset_.clear();

  kPolygonErrorLatOffset_ = kPolygonErrorLatOffset;
  kLatBufferComp_ = 0;
  kLatBufferComp_atten_ = 0;
  kLatBufferCompMax_v_ = config_.kLatBufferCompMax_v;
  if (ConfigurationContext::Instance()
          ->synthetic_config()
          .sensor_configuration != "lidar") {
    kPolygonErrorLatOffset_ =
        kPolygonErrorLatOffset + config_.kPolygonLatError_nolidar;
    kLatBufferComp_ += config_.kLatBufferComp_noLidar;
    kLatBufferComp_atten_ += config_.kLatBufferComp_noLidar_atten;
  }
}

ExtractPathBound::~ExtractPathBound() {}

bool ExtractPathBound::process(double lb_width, double lb_length,
                               const std::vector<AvoidObstacle> &obs_avd,
                               const std::vector<AvoidObstacle> &obs_avd_prior,
                               BlockType &block_type) {
  MLOG_PROFILING("ExtractPathBound");
  frenet_coord_ = baseline_info_->get_frenet_coord();
  if (frenet_coord_ == nullptr) {
    MSD_LOG(ERROR, "[ExtractPathBound::process] frenet_coord_ is null!");
    return true;
  }

  FrenetState init_point;
  baseline_info_->get_ego_state_manager().get_lat_replan_state(init_point);
  adc_s_ = init_point.s;
  adc_l_ = init_point.r;
  adc_dl_ = init_point.dr_ds;
  adc_ddl_ = init_point.ddr_dsds;
  adc_vel_ = init_point.ds;
  adc_acc_ = init_point.dds;
  adc_sl_boundary_ = baseline_info_->get_adc_sl_boundary();

  update_delta_s();
  get_refline_info(baseline_info_->lane_id());

  // obstacle pretreatment
  sl_polygon_.clear();
  (void)GetSLPolygonSeq(obs_avd);
  (void)GetSLPolygonSeq(obs_avd_prior);
  update_obstacle_priority();

  for (const auto &obs : obs_avd_prior) {
    obs_priority_[std::get<0>(obs)] = 3; // obs_avd_prior 中的障碍物优先级为3
    if (sl_polygon_.find(std::get<0>(obs)) != sl_polygon_.end() &&
        !sl_polygon_[std::get<0>(obs)].empty()) {
      double start_s = sl_polygon_[std::get<0>(obs)].front().min_x() - 3;
      double end_s = sl_polygon_[std::get<0>(obs)].front().max_x() + 3;
      double start_l = sl_polygon_[std::get<0>(obs)].front().min_y();
      double end_l = sl_polygon_[std::get<0>(obs)].front().max_y();
      for (int i = 0; i < num_knots_; ++i) {
        if (adc_s_list_[i] > start_s && adc_s_list_[i] < end_s) {
          if (std::get<1>(obs) == "left") {
            refline_info_[i].left_lane_width =
                std::fmax(end_l +
                              ConfigurationContext::Instance()
                                  ->get_vehicle_param()
                                  .width +
                              1.0 - refline_info_[i].left_lane_border,
                          refline_info_[i]
                              .left_lane_width); // 根据避让幅度扩展左侧车道宽度
            MSD_LOG(INFO, "[ExtractPath] extend left lane width!!");
          } else if (std::get<1>(obs) == "right") {
            refline_info_[i].right_lane_width = std::fmax(
                -start_l +
                    ConfigurationContext::Instance()
                        ->get_vehicle_param()
                        .width +
                    1.0 - refline_info_[i].right_lane_border,
                refline_info_[i]
                    .right_lane_width); // 根据避让幅度扩展右侧车道宽度
            MSD_LOG(INFO, "[ExtractPath] extend right lane width!!");
          }
        }
      }
    }
  }

  auto &state_machine_output = context_->state_machine_output();
  int state = state_machine_output.curr_state;
  if (state_machine_output.lc_pause) {
    if ((state == ROAD_LC_LCHANGE) &&
        lane_borrow_info_ == LaneBorrowInfo::RIGHT_BORROW) {
      double overlane =
          adc_l_ +
          ConfigurationContext::Instance()->get_vehicle_param().width / 2.0 +
          0.2 - refline_info_.front().left_lane_border;
      lb_width = clip(overlane, lb_width, 0.1);
    } else if ((state == ROAD_LC_RCHANGE) &&
               lane_borrow_info_ == LaneBorrowInfo::LEFT_BORROW) {
      double overlane =
          -(adc_l_ -
            ConfigurationContext::Instance()->get_vehicle_param().width / 2.0 -
            0.2) -
          refline_info_.front().right_lane_border;
      lb_width = clip(overlane, lb_width, 0.1);
    }
  }
  block_type.clear();
  if (!GenerateRegularPathBound(lb_width, lb_length, obs_avd, obs_avd_prior,
                                block_type)) {
    return false;
  }

  return true;
}

void ExtractPathBound::update_delta_s() {
  adc_s_list_.clear();
  s_max_list_.clear();

  double delta_s_ = std::fmax(adc_vel_ * delta_t_, 0.3);
  double vel = adc_vel_;
  double s_max = adc_s_;
  max_vel_ = std::fmin(
      120, std::fmax(world_model_->get_map_info().v_cruise(),
                     world_model_->get_map_info().v_cruise_current()));
  for (size_t i = 0; i < num_knots_; ++i) {
    adc_s_list_.emplace_back(adc_s_ + i * delta_s_); // 自车匀速推演s

    s_max += vel * delta_t_ + max_acc_ * delta_t_ * delta_t_ / 2.0;
    vel += max_acc_ * delta_t_;
    vel = clip(vel, max_vel_, 0.0);
    s_max_list_.emplace_back(
        s_max + adc_vel_ * 1 +
        std::pow(vel, 2) / 2 *
            max_acc_); // 自车以最大加速度加速+最大减速度刹停+1秒时距
  }
}

void ExtractPathBound::get_refline_info(const int target_lane_id) {
  static std::vector<RefPointFrenet> cur_lane, left_lane, right_lane;
  cur_lane.clear();
  left_lane.clear();
  right_lane.clear();
  world_model_->get_map_lateral_info(cur_lane, left_lane, right_lane,
                                     target_lane_id);
  cal_inter_refline(cur_lane, left_lane, right_lane, refline_info_);

  double s_step = adc_s_;
  for (int i = 0; i < num_knots_; i++) {
    s_step = adc_s_list_[i];
    if (s_step < frenet_coord_->GetLength()) {
      refline_info_[i].curvature = frenet_coord_->GetRefCurveCurvature(s_step);

      double theta_ref_cur = frenet_coord_->GetRefCurveHeading(s_step) -
                             baseline_info_->get_ego_state().ego_pose.theta; //
      refline_info_[i].theta_ref =
          planning_math::NormalizeAngle(theta_ref_cur); //
    } else {
      refline_info_[i].curvature = 0;
      refline_info_[i].theta_ref = 0; //
    }
  }
}

void ExtractPathBound::cal_inter_refline(
    const std::vector<RefPointFrenet> &cur_lane,
    const std::vector<RefPointFrenet> &left_lane,
    const std::vector<RefPointFrenet> &right_lane,
    std::vector<path_planner::RefPointInfo> &refline_info) {
  refline_info.resize(num_knots_);

  int s_index_curr(0), s_index_left(0), s_index_right(0);
  double epsilon = 1.0e-6;
  double k_sbuffer = 0;
  for (int i = 0; i < num_knots_; i++) {
    double s_step = adc_s_list_[i];
    if (cur_lane.empty()) {
      break;
    } else if (cur_lane.size() == 1) {
      refline_info[i].current_lane_width = cur_lane[0].lane_width;
      refline_info[i].left_lane_border = cur_lane[0].left_lane_border;
      refline_info[i].right_lane_border = cur_lane[0].right_lane_border;
      refline_info[i].left_road_border = cur_lane[0].left_road_border;
      refline_info[i].right_road_border = cur_lane[0].right_road_border;
      refline_info[i].left_road_border_type = cur_lane[0].left_road_border_type;
      refline_info[i].right_road_border_type =
          cur_lane[0].right_road_border_type;
    } else {
      for (int j = s_index_curr; j < (int)cur_lane.size() - 1; j++) {
        if (s_step >= cur_lane[j].s && s_step <= cur_lane[j + 1].s) {
          if (fabs(cur_lane[j + 1].s - cur_lane[j].s) < epsilon) {
            k_sbuffer = 0;
          } else {
            k_sbuffer =
                (s_step - cur_lane[j].s) / (cur_lane[j + 1].s - cur_lane[j].s);
          }

          refline_info[i].current_lane_width =
              cur_lane[j].lane_width +
              (cur_lane[j + 1].lane_width - cur_lane[j].lane_width) * k_sbuffer;
          refline_info[i].left_lane_border =
              cur_lane[j].left_lane_border + (cur_lane[j + 1].left_lane_border -
                                              cur_lane[j].left_lane_border) *
                                                 k_sbuffer;
          refline_info[i].right_lane_border =
              cur_lane[j].right_lane_border +
              (cur_lane[j + 1].right_lane_border -
               cur_lane[j].right_lane_border) *
                  k_sbuffer;
          refline_info[i].left_road_border =
              cur_lane[j].left_road_border + (cur_lane[j + 1].left_road_border -
                                              cur_lane[j].left_road_border) *
                                                 k_sbuffer;
          refline_info[i].right_road_border =
              cur_lane[j].right_road_border +
              (cur_lane[j + 1].right_road_border -
               cur_lane[j].right_road_border) *
                  k_sbuffer;

          refline_info[i].left_road_border_type =
              cur_lane[j].left_road_border_type +
                          cur_lane[j + 1].left_road_border_type >
                      0
                  ? 1
                  : 0;
          refline_info[i].right_road_border_type =
              cur_lane[j].right_road_border_type +
                          cur_lane[j + 1].right_road_border_type >
                      0
                  ? 1
                  : 0;

          s_index_curr = j;
          break;
        } else if (j == cur_lane.size() - 2) {
          refline_info[i].current_lane_width = cur_lane.back().lane_width;
          refline_info[i].left_lane_border = cur_lane.back().left_lane_border;
          refline_info[i].right_lane_border = cur_lane.back().right_lane_border;
          refline_info[i].left_road_border = cur_lane.back().left_road_border;
          refline_info[i].right_road_border = cur_lane.back().right_road_border;
          refline_info[i].left_road_border_type =
              cur_lane.back().left_road_border_type;
          refline_info[i].right_road_border_type =
              cur_lane.back().right_road_border_type;
        }
      }
    }

    if (left_lane.empty()) {
      refline_info[i].left_lane_width = 0;
    } else if (left_lane.size() == 1) {
      refline_info[i].left_lane_width = left_lane[0].lane_width;
    } else {
      for (int j = s_index_left; j < (int)left_lane.size() - 1; j++) {
        if (fabs(left_lane[j + 1].s - left_lane[j].s) < epsilon) {
          k_sbuffer = 0;
        } else {
          k_sbuffer =
              (s_step - left_lane[j].s) / (left_lane[j + 1].s - left_lane[j].s);
        }
        if (s_step >= left_lane[j].s && s_step <= left_lane[j + 1].s) {
          refline_info[i].left_lane_width =
              left_lane[j].lane_width +
              (left_lane[j + 1].lane_width - left_lane[j].lane_width) *
                  k_sbuffer;

          s_index_left = j;
          break;
        } else if (j == left_lane.size() - 2) {
          refline_info[i].left_lane_width = left_lane.back().lane_width;
        }
      }
    }

    if (right_lane.empty()) {
      refline_info[i].right_lane_width = 0;
    } else if (right_lane.size() == 1) {
      refline_info[i].right_lane_width = right_lane[0].lane_width;
    } else {
      for (int j = s_index_right; j < (int)right_lane.size() - 1; j++) {
        if (fabs(right_lane[j + 1].s - right_lane[j].s) < epsilon) {
          k_sbuffer = 0;
        } else {
          k_sbuffer = (s_step - right_lane[j].s) /
                      (right_lane[j + 1].s - right_lane[j].s);
        }
        if (s_step >= right_lane[j].s && s_step <= right_lane[j + 1].s) {
          refline_info[i].right_lane_width =
              right_lane[j].lane_width +
              (right_lane[j + 1].lane_width - right_lane[j].lane_width);

          s_index_right = j;
          break;
        } else if (j == right_lane.size() - 2) {
          refline_info[i].right_lane_width = right_lane.back().lane_width;
        }
      }
    }
  }
}

bool ExtractPathBound::GenerateRegularPathBound(
    const double lb_width, const double lb_length,
    const std::vector<AvoidObstacle> &obs_avd,
    const std::vector<AvoidObstacle> &obs_avd_prior, BlockType &block_type) {
  MLOG_PROFILING("GenerateRegularPathBound");
  // 1. Initialize the path boundaries to be an indefinitely large area.
  PathBound path_bound, des_bound;
  if (!InitPathBoundary(&path_bound, &des_bound)) {
    return false;
  }
  std::vector<std::vector<int>> obs_lists;
  // 2. Decide a rough boundary based on lane info and ADC's position
  if (!obs_avd_prior.empty()) {
    if (!GetBoundaryFromLanesAndADC(&des_bound, &path_bound, block_type)) {
      MSD_LOG(INFO, "Failed to decide a rough boundary based on road border.");
    }
    if (!GetBoundaryFromStaticObstacles(obs_avd_prior, block_type, des_bound,
                                        path_bound, obs_lists)) {
      MSD_LOG(INFO, "Failed to decide fine tune the boundaries after "
                    "taking into consideration static prior obstacles.");
    }
  }
  if (!GetBoundaryFromLanesAndADC(lb_width, lb_length, 0.1, &des_bound,
                                  &path_bound, block_type)) {
    MSD_LOG(INFO,
            "Failed to decide a rough boundary based on road information.");
  }

  // 3. Check the boundary considering motion
  if (!check_motion_collide("road", obs_lists, &path_bound, block_type)) {
    MSD_LOG(INFO, "Failed to decide fine tune the road boundaries after "
                  "taking into consideration adc motion.");
  }

  // 4. Update path bound considering static obstacle
  if (!GetBoundaryFromStaticObstacles(obs_avd, block_type, des_bound,
                                      path_bound, obs_lists)) {
    MSD_LOG(INFO, "Failed to decide fine tune the boundaries after "
                  "taking into consideration static obstacles.");
  }

  PathBound path_bound_reverse = path_bound;
  PathBound des_bound_reverse = des_bound;
  // 5.1. Update path bound considering reverse obs
  if (!GetBoundaryListFromReverseObstacles(obs_avd, path_bound_reverse,
                                           des_bound_reverse, block_type)) {
    MSD_LOG(INFO, "Failed to decide fine tune the boundaries after "
                  "taking into consideration obstacles.");
  }
  GetLatOffset(des_bound_reverse);

  // 5.2 Adjust the boundary considering obstacles
  if (!GetBoundaryListFromObstacles(obs_avd, path_bound_reverse, block_type)) {
    MSD_LOG(INFO, "Failed to decide fine tune the boundaries after "
                  "taking into consideration obstacles.");
  }
  check_dynamic_ignore(block_type);

  // 6. Check the boundary considering motion
  if (!check_motion_collide("obstacle", obs_lists, &path_bound, block_type)) {
    MSD_LOG(INFO, "Failed to decide fine tune the obstacle boundaries after ");
  }
  // 7. Set lon info
  GetSpeedLimit(obs_avd, des_bound, des_bound_reverse, path_bound);

  // 8. Check static obstacle ignore
  CheckStaticIgnore(block_type);

  UpdateAvdInfo(block_type);

  if (block_type.type != BlockType::Type::NOBLOCK)
    return false;

  return true;
}

bool ExtractPathBound::InitPathBoundary(PathBound *const path_bound,
                                        PathBound *const des_bound) {
  path_bound->clear();
  des_bound->clear();
  // Starting from ADC's current position, increment until the horizon, and
  // set lateral bounds to be infinite at every spot.
  for (int i = 0; i < num_knots_; i++) {
    path_bound->emplace_back(adc_s_list_[i], -10., 10.);
    des_bound->emplace_back(adc_s_list_[i], -10., 10.);
  }

  // return.
  if (path_bound->empty()) {
    MSD_LOG(INFO, "Empty path boundary in InitPathBoundary");
    return false;
  }

  return true;
}

bool ExtractPathBound::GetBoundaryFromLanesAndADC(const double lb_width,
                                                  const double lb_length,
                                                  double ADC_buffer,
                                                  PathBound *const des_bound,
                                                  PathBound *const path_bound,
                                                  BlockType &block_type) {

  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 2. Get the neighbor lane widths at the current point.
    double curr_left_bound_lane =
        std::fmax(refline_info_[i].left_lane_border, 1.3);
    double curr_right_bound_lane =
        -std::fmax(refline_info_[i].right_lane_border, 1.3);

    if (lane_borrow_info_ == LaneBorrowInfo::BOTH_BORROW) {
      curr_left_bound_lane += (refline_info_[i].left_lane_width + 10);
      curr_right_bound_lane -= (refline_info_[i].right_lane_width + 10);
    } else if (lane_borrow_info_ > LaneBorrowInfo::NO_BORROW) {
      if (std::get<0>((*path_bound)[i]) <= lb_length + adc_s_)
        curr_left_bound_lane +=
            std::fmin(refline_info_[i].left_lane_width, lb_width);
    } else if (lane_borrow_info_ < LaneBorrowInfo::NO_BORROW) {
      if (std::get<0>((*path_bound)[i]) <= lb_length + adc_s_)
        curr_right_bound_lane -=
            std::fmin(refline_info_[i].right_lane_width, lb_width);
      if (lane_borrow_info_ == LaneBorrowInfo::RB_LEFT_NOLANE) {
        curr_left_bound_lane += (refline_info_[i].left_lane_width + 10);
      }
    }

    double offset_to_map = 0.0;
    // TODO : offset_to_map should be l of lane waypoint
    double KPhysicalRoadSafetyBufferWithLaneWidth = interp(
        refline_info_[i].current_lane_width, std::array<double, 2>{2.8, 3.7},
        std::array<double, 2>{0.2, KPhysicalRoadSafetyBuffer});
    double left_road_buffer = refline_info_[i].left_road_border_type == 0
                                  ? KRoadSafetyBuffer
                                  : KPhysicalRoadSafetyBufferWithLaneWidth;
    double right_road_buffer = refline_info_[i].right_road_border_type == 0
                                   ? KRoadSafetyBuffer
                                   : KPhysicalRoadSafetyBufferWithLaneWidth;
    double curr_left_bound =
        std::fmin(curr_left_bound_lane,
                  refline_info_[i].left_road_border - left_road_buffer) -
        offset_to_map;

    double curr_right_bound =
        std::fmax(curr_right_bound_lane,
                  -refline_info_[i].right_road_border + right_road_buffer) -
        offset_to_map;

    if (!UpdatePathBoundaryAndCenterLine(i, curr_left_bound, curr_right_bound,
                                         path_bound)) {
      block_type.type = BlockType::MAP;
      block_type.static_block_s = std::get<0>((*path_bound)[i]);

      return false;
    }

    double des_right =
        std::fmax(curr_right_bound_lane, -refline_info_[i].right_road_border +
                                             right_road_buffer + 0.2);
    double des_left =
        std::fmin(curr_left_bound_lane,
                  refline_info_[i].left_road_border - left_road_buffer - 0.2);
    std::get<1>((*des_bound)[i]) =
        std::fmax(std::get<1>((*des_bound)[i]),
                  des_right + GetBufferBetweenADCCenterAndEdge());
    std::get<2>((*des_bound)[i]) =
        std::fmin(std::get<2>((*des_bound)[i]),
                  des_left - GetBufferBetweenADCCenterAndEdge());
  }

  return true;
}

bool ExtractPathBound::GetBoundaryFromLanesAndADC(PathBound *const des_bound,
                                                  PathBound *const path_bound,
                                                  BlockType &block_type) {
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double offset_to_map = 0.0;
    double KPhysicalRoadSafetyBufferWithLaneWidth = interp(
        refline_info_[i].current_lane_width, std::array<double, 2>{2.8, 3.7},
        std::array<double, 2>{0.2, KPhysicalRoadSafetyBuffer});
    double left_road_buffer = refline_info_[i].left_road_border_type == 0
                                  ? KRoadSafetyBuffer
                                  : KPhysicalRoadSafetyBufferWithLaneWidth;
    double right_road_buffer = refline_info_[i].right_road_border_type == 0
                                   ? KRoadSafetyBuffer
                                   : KPhysicalRoadSafetyBufferWithLaneWidth;
    double curr_left_bound =
        refline_info_[i].left_road_border - left_road_buffer - offset_to_map;

    double curr_right_bound =
        -refline_info_[i].right_road_border + right_road_buffer - offset_to_map;

    if (!UpdatePathBoundaryAndCenterLine(i, curr_left_bound, curr_right_bound,
                                         path_bound)) {
      block_type.type = BlockType::MAP;
      block_type.static_block_s = std::get<0>((*path_bound)[i]);

      return false;
    }

    double des_right =
        -refline_info_[i].right_road_border + right_road_buffer + 0.2;
    double des_left =
        refline_info_[i].left_road_border - left_road_buffer - 0.2;
    std::get<1>((*des_bound)[i]) =
        std::fmax(std::get<1>((*des_bound)[i]),
                  des_right + GetBufferBetweenADCCenterAndEdge());
    std::get<2>((*des_bound)[i]) =
        std::fmin(std::get<2>((*des_bound)[i]),
                  des_left - GetBufferBetweenADCCenterAndEdge());
  }

  return true;
}

bool ExtractPathBound::GetBoundaryListFromReverseObstacles(
    const std::vector<AvoidObstacle> &obs_avd, PathBound &path_bound,
    PathBound &des_bound, BlockType &block_type) {
  MLOG_PROFILING("GetBoundaryListFromObstacles");

  std::vector<AvoidObstacle> reverse_obs;
  for (const auto &obs : obs_avd) {
    if (obs_reverse_.find(std::get<0>(obs)) != obs_reverse_.end()) {
      reverse_obs.emplace_back(obs);
    }
  }

  std::map<int, std::vector<double>> obs_block_time;
  for (size_t i = 0; i * delta_t_ < obs_active_time_default_ + 1.0e-6; ++i) {
    if (isSparseMode && i % 2 == 1) // 抽稀
      continue;
    std::vector<std::tuple<int, std::string, EdgePoints>> edge_list;
    get_obstacle_edge(int(i), reverse_obs, edge_list);
    std::unordered_set<int> block_obs; // vector<s, obs_list>
    if (!GetBoundaryFromObstacles(edge_list, block_type.static_block_s,
                                  path_bound, block_obs)) {
      for (auto &it : block_obs) {
        obs_block_time[it].emplace_back(i * 0.2);
      }
    }
    if (i < 10) {
      std::get<1>(des_bound[i]) =
          std::fmax(std::get<1>(des_bound[i]), std::get<1>(path_bound[i]));
      std::get<2>(des_bound[i]) =
          std::fmin(std::get<2>(des_bound[i]), std::get<2>(path_bound[i]));
    }
  }

  for (auto &obs : reverse_obs) {
    int id = std::get<0>(obs);
    if (obs_block_time.find(id) != obs_block_time.end()) {
      block_type.obs_avd_time[id] =
          std::make_tuple(false, std::fmax(obs_block_time[id].at(0) - 0.4, 0));
    } else {
      block_type.obs_avd_time[id] = std::make_tuple(true, 8);
    }
  }

  if (!obs_block_time.empty()) {
    return false;
  }

  return true;
}

bool ExtractPathBound::GetBoundaryListFromObstacles(
    const std::vector<AvoidObstacle> &obs_avd, const PathBound &path_bound,
    BlockType &block_type) {
  MLOG_PROFILING("GetBoundaryListFromObstacles");
  const auto &lateral_behavior_state =
      context_->state_machine_output().curr_state;
  // HACK : limit lane width , lw < 4.0
  PathBound static_path_bound = path_bound;
  if (lane_borrow_info_ == LaneBorrowInfo::NO_BORROW) {
    for (size_t i = 1;
         i < path_bound.size() &&
         std::get<0>(path_bound[i]) < block_type.static_block_s - 1.0e-6;
         ++i) {
      std::get<1>(static_path_bound[i]) =
          std::fmax(std::get<1>(path_bound[i]), -2.0);
      std::get<2>(static_path_bound[i]) =
          std::fmin(std::get<2>(path_bound[i]), 2.0);
      if (std::get<1>(static_path_bound[i]) >
          std::get<2>(static_path_bound[i])) {
        if (std::get<1>(path_bound[i]) > -2.0) {
          std::get<2>(static_path_bound[i]) =
              std::get<1>(static_path_bound[i]) + 0.2;
        } else if (std::get<2>(path_bound[i]) < 2.0) {
          std::get<1>(static_path_bound[i]) =
              std::get<2>(static_path_bound[i]) - 0.2;
        }
      }
    }
  }

  dynamic_obs_avd_.clear();
  auto &obstacle_manager = baseline_info_->obstacle_manager();
  for (const auto &obs : obs_avd) {
    if (sl_polygon_.find(std::get<0>(obs)) == sl_polygon_.end()) {
      continue;
    }
    auto ptr_obstacle = obstacle_manager.find_obstacle(std::get<0>(obs));
    if (!ptr_obstacle) {
      MSD_LOG(ERROR, "Lateral Behavior Planner Obstacle[%d] do not exist!",
              std::get<0>(obs));
      continue;
    }
    if (!ptr_obstacle->IsStatic() &&
        obs_reverse_.find(std::get<0>(obs)) == obs_reverse_.end()) {
      dynamic_obs_avd_.emplace_back(obs); // 非逆行动态障碍物
    }
  }
  check_side_collision(obs_avd, block_type);

  std::map<int, std::vector<double>> obs_block_time;
  for (size_t i = 0; i * delta_t_ < obs_active_time_default_ + 1.0e-6; ++i) {
    if (isSparseMode && i % 2 == 1)
      continue;
    std::vector<std::tuple<int, std::string, EdgePoints>> edge_list;
    get_obstacle_edge(int(i), dynamic_obs_avd_, edge_list);
    std::unordered_set<int> block_obs; // vector<s, obs_list>
    if (!GetBoundaryFromObstacles(edge_list, block_type.static_block_s,
                                  static_path_bound, block_obs)) {
      for (auto &it : block_obs) {
        obs_block_time[it].emplace_back(i * 0.2);
      }
    }
  }

  auto &pre_avd_info = context_->planning_status().planning_result.avd_info;

  for (auto &obs : dynamic_obs_avd_) {
    int id = std::get<0>(obs);
    if (obs_block_time.find(id) != obs_block_time.end()) {
      block_type.obs_avd_time[id] =
          std::make_tuple(false, std::fmax(obs_block_time[id].at(0) - 0.4, 0));
      if (pre_avd_info.find(id) != pre_avd_info.end()) {
        avd_loop_[id] = 0;
      }
    } else {
      if ((lateral_behavior_state == ROAD_NONE) &&
          check_dynamic_cutin(
              obs)) { // 车道保持时的障碍物置false，进st graph，从而更快减速
        block_type.obs_avd_time[id] = std::make_tuple(false, 8);
      } else {
        block_type.obs_avd_time[id] = std::make_tuple(true, 8);
      }
    }
  }

  if (!obs_block_time.empty()) {
    return false;
  }

  return true;
}

bool ExtractPathBound::GetBoundaryFromStaticObstacles(
    const std::vector<AvoidObstacle> &obs_avd, BlockType &block_type,
    PathBound &des_bound, PathBound &path_bound,
    std::vector<std::vector<int>> &obstacle_lists) {
  static_obs_avd_.clear();
  auto &obstacle_manager = baseline_info_->obstacle_manager();

  for (auto obs : obs_avd) {
    auto ptr_obstacle = obstacle_manager.find_obstacle(std::get<0>(obs));
    if (!ptr_obstacle) {
      continue;
    }
    if (sl_polygon_.find(std::get<0>(obs)) == sl_polygon_.end()) {
      continue;
    }
    if (ptr_obstacle->IsStatic()) {
      static_obs_avd_.emplace_back(obs);
    }
  }

  std::multiset<double, std::greater<double>> right_bounds;
  std::multiset<double> left_bounds;
  right_bounds.insert(std::numeric_limits<double>::lowest());
  left_bounds.insert(std::numeric_limits<double>::max());

  for (size_t i = 1;
       i < path_bound.size() &&
       std::get<0>(path_bound[i]) < block_type.static_block_s - 1.0e-6;
       ++i) {
    double curr_s = std::get<0>(path_bound[i]);
    double last_s = std::get<0>(path_bound[i - 1]);
    double next_s =
        std::get<0>(path_bound[std::min(i + 1, path_bound.size() - 1)]);

    std::vector<std::tuple<int, std::string, EdgePoints>> edge_list;
    get_obstacle_edge(0, static_obs_avd_, edge_list);
    std::vector<int> obs_ids;
    cal_bound_s(edge_list, curr_s, last_s, next_s, right_bounds, left_bounds,
                obs_ids);

    obstacle_lists.emplace_back(obs_ids); // 每一时刻对边界起作用的障碍物id
    // Update the boundary.
    if (!UpdatePathBoundaryAndCenterLine(i, *left_bounds.begin(),
                                         *right_bounds.begin(), &path_bound)) {
      block_type.type = BlockType::OBSTACLE;
      block_type.static_block_s = std::get<0>(path_bound[i]);
      for (auto &obs : static_obs_avd_) {
        if (find(obs_ids.begin(), obs_ids.end(), std::get<0>(obs)) !=
            obs_ids.end()) {
          block_type.obs_avd_time[std::get<0>(obs)] = std::make_tuple(false, 0);
          MSD_LOG(INFO, "[TimeInterval] blocked by static obs : %d",
                  std::get<0>(obs));
        } else {
          block_type.obs_avd_time[std::get<0>(obs)] = std::make_tuple(true, 8);
          MSD_LOG(INFO, "[TimeInterval] static obs after block_s : %d",
                  std::get<0>(obs));
        }
      }

      return false;
    }

    double des_right = *right_bounds.begin() + 0.5 + kPolygonErrorLatOffset_;
    double des_left = *left_bounds.begin() - 0.5 - kPolygonErrorLatOffset_;
    std::get<1>(des_bound[i]) =
        std::fmax(std::get<1>(des_bound[i]),
                  des_right + GetBufferBetweenADCCenterAndEdge());
    std::get<2>(des_bound[i]) =
        std::fmin(std::get<2>(des_bound[i]),
                  des_left - GetBufferBetweenADCCenterAndEdge());

    while (*left_bounds.begin() < 1.0e9) {
      left_bounds.erase(left_bounds.begin());
    }
    while (*right_bounds.begin() > -1.0e9) {
      right_bounds.erase(right_bounds.begin());
    }
  }
  return true;
}

void ExtractPathBound::get_obstacle_edge(
    int time_index, const std::vector<AvoidObstacle> &dynamic_obs_avd,
    std::vector<std::tuple<int, std::string, EdgePoints>> &edge_list) {
  auto &obstacle_manager = baseline_info_->obstacle_manager();
  for (const auto &obs : dynamic_obs_avd) {
    EdgePoints edge_points_temp;
    std::string obs_side_pass = std::get<1>(obs);

    auto ptr_obstacle = obstacle_manager.find_obstacle(std::get<0>(obs));
    if (!ptr_obstacle) {
      MSD_LOG(ERROR, "Lateral Behavior Planner Obstacle[%d] do not exist!",
              std::get<0>(obs));
      continue;
    }

    double active_time = obs_active_time_default_;
    if (sl_polygon_.find(std::get<0>(obs)) == sl_polygon_.end()) {
      continue;
    }
    auto ptr_priority = obs_priority_.find(std::get<0>(obs));
    if (ptr_priority != obs_priority_.end()) {
      if (ptr_priority->second == -1)
        active_time = 2.0;
      else if (ptr_priority->second == -2)
        active_time = 1.0;
    }
    if (ptr_obstacle->Prob() < 1.0) {
      active_time = std::fmax(active_time - 1.0, 1.0);
    }
    if (time_index * delta_t_ > active_time)
      continue;

    const auto &polygon_list = sl_polygon_[std::get<0>(obs)];
    int time_num = (int)(time_index / 2);
    if (time_num + 1 > polygon_list.size())
      continue;
    const auto &polygon_corners = polygon_list[time_num];
    if (polygon_corners.min_x() >
            s_max_list_[time_index] +
                ConfigurationContext::Instance()->get_vehicle_param().length /
                    2.0 &&
        !ptr_obstacle->IsStatic())
      continue;

    auto lat_traj_tmp = ptr_obstacle->GetPointAtTime(time_index * delta_t_);
    double theta_ref = frenet_coord_->GetRefCurveHeading(
        (polygon_corners.max_x() + polygon_corners.min_x()) / 2.);
    double lat_vel =
        lat_traj_tmp.v * sin(lat_traj_tmp.velocity_direction - theta_ref);
    if ((obs_side_pass == "left" && lat_vel > 0) ||
        (obs_side_pass == "right" && lat_vel < 0)) {
      lat_vel = std::fabs(lat_vel);
    } else {
      lat_vel = 0;
    }

    double latbuffer_coef_min =
        obs_reverse_.find(std::get<0>(obs)) != obs_reverse_.end() ? 0.3 : 1.0;
    double attenuation_coef = 1 - (time_index * delta_t_ - 1.0) / (3.0 - 1.0);
    attenuation_coef = clip(attenuation_coef, 1.0, latbuffer_coef_min);
    double lat_buffer_comp =
        kLatBufferComp_ - lat_traj_tmp.v * kLatBufferComp_atten_;
    double lat_buffer =
        (kStaticObstacleLatBuffer + lat_buffer_comp +
         std::min(lat_traj_tmp.v * lat_traj_tmp.v * 0.015, 0.8) +
         std::min(lat_vel * 1.5, 0.5)) *
        attenuation_coef;
    double start_buffer = kStaticObstacleLonStartBuffer +
                          std::min(lat_traj_tmp.v * 0.6, kLatBufferCompMax_v_);
    double end_buffer =
        kStaticObstacleLonEndBuffer + std::min(lat_traj_tmp.v * 0.3, 3.0);

    if (polygon_corners.num_points() >= 3) {
      cal_corners(polygon_corners.points(), obs_side_pass, lat_buffer,
                  start_buffer, end_buffer, edge_points_temp);
    } else {
      continue;
    }
    edge_list.emplace_back(std::get<0>(obs), obs_side_pass, edge_points_temp);
  }
}

// calculate boundary from all obstacles when t[time_index]
bool ExtractPathBound::GetBoundaryFromObstacles(
    const std::vector<std::tuple<int, std::string, EdgePoints>> &edge_list,
    const double static_block_s, PathBound &path_bound,
    std::unordered_set<int> &block_obs) {
  std::multiset<double, std::greater<double>> right_bounds;
  std::multiset<double> left_bounds;
  right_bounds.insert(std::numeric_limits<double>::lowest());
  left_bounds.insert(std::numeric_limits<double>::max());

  for (size_t i = 1; i < path_bound.size() &&
                     std::get<0>(path_bound[i]) < static_block_s - 1.0e-6;
       ++i) {
    double curr_s = std::get<0>(path_bound[i]);
    double last_s = std::get<0>(path_bound[i - 1]);
    double next_s =
        std::get<0>(path_bound[std::min(i + 1, path_bound.size() - 1)]);
    std::vector<int> obs_ids;
    cal_bound_s(edge_list, curr_s, last_s, next_s, right_bounds, left_bounds,
                obs_ids);

    // Update the boundary.
    if (!UpdatePathBoundaryAndCenterLine(i, *left_bounds.begin(),
                                         *right_bounds.begin(), &path_bound)) {
      for (auto id : obs_ids)
        block_obs.insert(id);
    }

    while (*left_bounds.begin() < 1.0e9) {
      left_bounds.erase(left_bounds.begin());
    }
    while (*right_bounds.begin() > -1.0e9) {
      right_bounds.erase(right_bounds.begin());
    }
  }
  if (!block_obs.empty())
    return false;

  return true;
}

// calculate bound from all obstacles in s when t[time_index]
void ExtractPathBound::cal_bound_s(
    const std::vector<std::tuple<int, std::string, EdgePoints>> &edge_list,
    const double curr_s, const double last_s, const double next_s,
    std::multiset<double, std::greater<double>> &right_bounds,
    std::multiset<double> &left_bounds, std::vector<int> &obs_ids) {
  for (const auto &obs : edge_list) {
    std::string obs_side_pass = std::get<1>(obs);
    const EdgePoints &edge_points_temp = std::get<2>(obs);

    double right_bound_temp = -1.0e6;
    double left_bound_temp = 1.0e6;
    if (edge_points_temp.size() > 0) { // polygon
      if ((std::get<0>(edge_points_temp.front()) < curr_s &&
           curr_s < std::get<0>(edge_points_temp.back())) ||
          (last_s < std::get<0>(edge_points_temp.front()) &&
           std::get<0>(edge_points_temp.front()) < next_s) ||
          (last_s < std::get<0>(edge_points_temp.back()) &&
           std::get<0>(edge_points_temp.back()) < next_s)) {
        if (obs_side_pass == "left") {
          right_bound_temp = cal_bound_temp(curr_s, last_s, next_s,
                                            edge_points_temp, obs_side_pass);
          right_bounds.insert(right_bound_temp);
        } else {
          left_bound_temp = cal_bound_temp(curr_s, last_s, next_s,
                                           edge_points_temp, obs_side_pass);
          left_bounds.insert(left_bound_temp);
        }
        obs_ids.emplace_back(std::get<0>(obs));
      }
    }
  }
}
// cal_corner
void ExtractPathBound::cal_corners(
    const std::vector<planning_math::Vec2d> &points, string side_pass,
    double lat_buffer, double start_buffer, double end_buffer,
    EdgePoints &edge_points) {
  double s_start_point[2],
      s_end_point[2]; // 对障碍物本身不好膨胀，那能否在提取边界后进行膨胀
  int s_min_i = -1;
  int s_max_i = -1;
  double s_max = std::numeric_limits<double>::lowest();
  double s_min = std::numeric_limits<double>::max();
  for (int i = 0; i < points.size(); i++) {
    auto &curr_point = points[i];
    if (s_max < curr_point.x()) {
      s_max = curr_point.x();
      s_max_i = i;
    }
    if (s_min > curr_point.x()) {
      s_min = curr_point.x();
      s_min_i = i;
    }
  }

  s_start_point[0] = points[s_min_i].x() - start_buffer;
  s_end_point[0] = points[s_max_i].x() + end_buffer;
  if (side_pass == "left") {
    s_start_point[1] = points[s_min_i].y() + lat_buffer;
    s_end_point[1] = points[s_max_i].y() + lat_buffer;
  } else {
    s_start_point[1] = points[s_min_i].y() - lat_buffer;
    s_end_point[1] = points[s_max_i].y() - lat_buffer;
  }
  edge_points.emplace_back(s_start_point[0], s_start_point[1]);
  edge_points.emplace_back(s_end_point[0], s_end_point[1]);

  // line
  for (int i = 0; i < points.size(); i++) {
    if (i == s_min_i || i == s_max_i)
      continue;
    // auto& curr_point = points[i];
    double l_tmp_i =
        points[s_min_i].y() + (points[i].x() - points[s_min_i].x()) *
                                  (points[s_max_i].y() - points[s_min_i].y()) /
                                  (points[s_max_i].x() - points[s_min_i].x());
    if (side_pass == "left") {
      if (l_tmp_i < points[i].y())
        edge_points.emplace_back(points[i].x(), points[i].y() + lat_buffer);
    } else {
      if (l_tmp_i > points[i].y())
        edge_points.emplace_back(points[i].x(), points[i].y() - lat_buffer);
    }
  }

  // sort()
  sort(edge_points.begin(), edge_points.end(),
       [](const std::tuple<double, double> &point1,
          const std::tuple<double, double> &point2) {
         return std::get<0>(point1) < std::get<0>(point2);
       });
}

double ExtractPathBound::cal_bound_temp(
    double curr_s, double last_s, double next_s,
    const std::vector<std::tuple<double, double>> &edge_points,
    string side_pass) {
  int index = -1;
  int l_limit_index = -1;
  double bound_temp = side_pass == "left"
                          ? std::numeric_limits<double>::max()
                          : std::numeric_limits<double>::lowest();
  double l_limit = side_pass == "left" ? std::numeric_limits<double>::lowest()
                                       : std::numeric_limits<double>::max();
  for (int i = 0; i < (int)edge_points.size() - 1; i++) {
    if (std::get<0>(edge_points[i]) < curr_s &&
        curr_s < std::get<0>(edge_points[i + 1])) {
      index = i;
      break;
    }
  }
  for (int i = 0; i < edge_points.size(); i++) {
    if (side_pass == "left" && l_limit < std::get<1>(edge_points[i])) {
      l_limit = std::get<1>(edge_points[i]);
      l_limit_index = i;
    }
    if (side_pass == "right" && l_limit > std::get<1>(edge_points[i])) {
      l_limit = std::get<1>(edge_points[i]);
      l_limit_index = i;
    }
  }

  if ((last_s < std::get<0>(edge_points[l_limit_index])) &&
      (std::get<0>(edge_points[l_limit_index]) < next_s)) { // protect tip
    bound_temp = std::get<1>(edge_points[l_limit_index]);
  } else if (curr_s <= std::get<0>(edge_points.front())) {
    bound_temp = std::get<1>(edge_points.front());
  } else if (curr_s >= std::get<0>(edge_points.back())) {
    bound_temp = std::get<1>(edge_points.back());
  } else if (index != -1) {
    if (std::fabs(std::get<0>(edge_points[index + 1]) -
                  std::get<0>(edge_points[index])) < 1.0e-2) {
      bound_temp = std::get<1>(edge_points[index]);
    } else {
      double k = (std::get<1>(edge_points[index + 1]) -
                  std::get<1>(edge_points[index])) /
                 (std::get<0>(edge_points[index + 1]) -
                  std::get<0>(edge_points[index]));
      bound_temp = std::get<1>(edge_points[index]) +
                   (curr_s - std::get<0>(edge_points[index])) * k;
    }
  }

  return bound_temp;
}

void ExtractPathBound::check_dynamic_ignore(BlockType &block_type) {
  double adc_length =
      ConfigurationContext::Instance()->get_vehicle_param().length;
  double adc_width =
      ConfigurationContext::Instance()->get_vehicle_param().width;
  for (const auto &obs : dynamic_obs_avd_) {
    int id = std::get<0>(obs);
    if (block_type.obs_avd_time.find(id) != block_type.obs_avd_time.end() &&
        std::get<0>(block_type.obs_avd_time[id])) {
      const auto &polygon_list = sl_polygon_[std::get<0>(obs)];
      if (polygon_list.empty())
        continue;

      const auto &init_corners = polygon_list[0];
      double min_s = init_corners.min_x();
      double max_s = init_corners.max_x();
      double min_l = init_corners.min_y();
      double max_l = init_corners.max_y();

      auto &obstacle_manager = baseline_info_->obstacle_manager();
      auto ptr_obstacle = obstacle_manager.find_obstacle(std::get<0>(obs));
      if (!ptr_obstacle) {
        MSD_LOG(ERROR, "Lateral Behavior Planner Obstacle[%d] do not exist!",
                std::get<0>(obs));
        continue;
      }

      if (adc_vel_ > 4 && adc_vel_ > ptr_obstacle->speed() &&
          min_s > adc_s_ + adc_length / 2.) {
        double dis_l = std::get<1>(obs) == "left"
                           ? max_l - adc_l_ + adc_width / 2.
                           : adc_l_ + adc_width / 2. - min_l;
        double dis_s = min_s - adc_s_ - adc_length / 2.;
        if (dis_s < adc_vel_ * 0.5 &&
            dis_l < 0.4 - dis_s / std::fmax(adc_vel_, 0.1)) {
          std::get<0>(block_type.obs_avd_time[id]) = false;
          MSD_LOG(INFO, "check ignore false : %d", id);
        }
      }
    }
  }
}

bool ExtractPathBound::check_motion_collide(
    std::string check_type, std::vector<std::vector<int>> &obstacle_lists,
    PathBound *const path_bound, BlockType &block_type) {
  double l_left = adc_l_;
  double dl_left = adc_dl_;
  double ddl_left = adc_ddl_;
  double l_right = adc_l_;
  double dl_right = adc_dl_;
  double ddl_right = adc_ddl_;
  double max_dddl =
      0.4 / ConfigurationContext::Instance()->get_vehicle_param().wheel_base /
      std::fmax(adc_vel_, 0.1);
  double max_ddl = std::fmin(
      6.0 / pow(fmax(adc_vel_, 0.1), 2.),
      ConfigurationContext::Instance()
              ->get_vehicle_param()
              .max_front_wheel_angle /
          ConfigurationContext::Instance()->get_vehicle_param().wheel_base);

  for (size_t i = 1;
       i < path_bound->size() &&
       std::get<0>((*path_bound)[i]) < block_type.static_block_s - 1.0e-6;
       ++i) {
    double ddl_min = -max_ddl - refline_info_[i].curvature;
    double ddl_max = max_ddl - refline_info_[i].curvature;
    double delta_s =
        std::get<0>((*path_bound)[i]) - std::get<0>((*path_bound)[i - 1]);
    double left_bound = cal_motion_buffer(delta_s, -max_dddl * 0.99,
                                          std::make_pair(ddl_min, ddl_max),
                                          l_left, dl_left, ddl_left);
    double right_bound = cal_motion_buffer(delta_s, max_dddl * 0.99,
                                           std::make_pair(ddl_min, ddl_max),
                                           l_right, dl_right, ddl_right);

    if (right_bound < std::get<1>((*path_bound)[i])) {
      if (check_type == "road") {
        block_type.type = BlockType::MAP_MOTION_RIGHT;
        std::get<1>((*path_bound)[i]) = right_bound - 1.0e-6;
      } else if (check_type == "obstacle") {
        block_type.type = BlockType::OBSTACLE_MOTION_RIGHT;

        if (i > obstacle_lists.size()) {
          break;
        }
        for (auto obs_id : obstacle_lists[i - 1]) {
          block_type.obs_avd_time[obs_id] = std::make_tuple(false, 0);
        }
        return false;
      }
    } else if (left_bound > std::get<2>((*path_bound)[i])) {
      if (check_type == "road") {
        block_type.type = BlockType::MAP_MOTION_LEFT;
        std::get<2>((*path_bound)[i]) = left_bound + 1.0e-6;
      } else if (check_type == "obstacle") {
        block_type.type = BlockType::OBSTACLE_MOTION_LEFT;

        if (i > obstacle_lists.size()) {
          break;
        }
        for (auto obs_id : obstacle_lists[i - 1]) {
          block_type.obs_avd_time[obs_id] = std::make_tuple(false, 0);
        }
        return false;
      }
    }
  }
  return true;
}

double ExtractPathBound::cal_motion_buffer(double delta_s, double dddl,
                                           std::pair<double, double> ddl_bound,
                                           double &l, double &dl, double &ddl) {
  l += dl * delta_s + ddl * pow(delta_s, 2) / 2. + dddl * pow(delta_s, 3) / 6.;
  dl += ddl * delta_s + dddl * pow(delta_s, 2) / 2.;
  ddl += dddl * delta_s;

  ddl = clip(ddl, ddl_bound.second, ddl_bound.first);
  return l;
}

double ExtractPathBound::GetBufferBetweenADCCenterAndEdge() {
  double adc_half_width =
      ConfigurationContext::Instance()->get_vehicle_param().width / 2.0;
  // TODO(all): currently it's a fixed number. But it can take into account many
  // factors such as: ADC length, possible turning angle, speed, etc.
  constexpr double kAdcEdgeBuffer = 0.0;

  return (adc_half_width + kAdcEdgeBuffer);
}

bool ExtractPathBound::UpdatePathBoundaryAndCenterLine(
    size_t idx, double left_bound, double right_bound,
    PathBound *const path_boundaries) {
  // Update the right bound (l_min):
  double new_l_min =
      std::fmax(std::get<1>((*path_boundaries)[idx]),
                right_bound + GetBufferBetweenADCCenterAndEdge());
  // Update the left bound (l_max):
  double new_l_max = std::fmin(std::get<2>((*path_boundaries)[idx]),
                               left_bound - GetBufferBetweenADCCenterAndEdge());

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    MSD_LOG(INFO, "Path is blocked at idx = %d", idx);
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;
  return true;
}

bool ExtractPathBound::UpdatePathBoundaryAndCenterLine(
    size_t idx, double left_bound, double right_bound,
    const PathBound path_boundaries) {
  // Update the right bound (l_min):
  double new_l_min =
      std::fmax(std::get<1>(path_boundaries[idx]),
                right_bound + GetBufferBetweenADCCenterAndEdge());
  // Update the left bound (l_max):
  double new_l_max = std::fmin(std::get<2>(path_boundaries[idx]),
                               left_bound - GetBufferBetweenADCCenterAndEdge());

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    return false;
  }
  return true;
}

bool ExtractPathBound::check_dynamic_cutin(const AvoidObstacle obs) {
  auto &obstacle_manager = baseline_info_->obstacle_manager();
  auto ptr_obstacle = obstacle_manager.find_obstacle(std::get<0>(obs));
  TrajectoryPoint lat_traj_tmp;
  if (ptr_obstacle != nullptr) {
    lat_traj_tmp = ptr_obstacle->GetPointAtTime(0.0);
  } else {
    return false;
  }

  if (sl_polygon_.find(std::get<0>(obs)) != sl_polygon_.end()) {
    const auto &polygon_list = sl_polygon_[std::get<0>(obs)];
    double block_threshold = 1.0;
    if (std::get<1>(obs) == "left") {
      for (const auto &corner : polygon_list) {
        if (corner.max_y() > -block_threshold) {
          double center_y_front =
              (polygon_list.front().max_y() + polygon_list.front().min_y()) /
              2.0;
          double center_y_back = (corner.max_y() + corner.min_y()) / 2.0;
          double deta_y = center_y_back - center_y_front;
          if (lat_traj_tmp.v > 1.0 &&
              deta_y >
                  ConfigurationContext::Instance()->get_vehicle_param().width /
                      2.0) {
            return true;
          }
        }
      }
    } else {
      for (const auto &corner : polygon_list) {
        if (corner.min_y() < block_threshold) {
          double center_y_front =
              (polygon_list.front().max_y() + polygon_list.front().min_y()) /
              2.0;
          double center_y_back = (corner.max_y() + corner.min_y()) / 2.0;
          double deta_y = center_y_back - center_y_front;
          if (lat_traj_tmp.v > 1.0 &&
              deta_y <
                  -ConfigurationContext::Instance()->get_vehicle_param().width /
                      2.0) {
            return true;
          }
        }
      }
    }
  }

  return false;
}

bool ExtractPathBound::GetSLPolygonSeq(
    const std::vector<AvoidObstacle> &obs_avd) {
  MLOG_PROFILING("GetSLPolygonSeq");
  auto &obstacle_manager = baseline_info_->mutable_obstacle_manager();
  auto ignorable_obs = baseline_info_->get_ignorable_obstacles();
  std::vector<planning_math::Polygon2d> t_polygon;
  StGraphData::DoublePair time_range{0.0,
                                     FLAGS_speed_lon_decision_time_horizon};
  double time_gap = 0.2;
  bool enable_heuristic = config_.enable_heuristic_frenet_search;
  for (auto obs : obs_avd) {
    t_polygon.clear();

    if (std::get<0>(obs) == -1000) {
      continue;
    }
    auto ptr_obstacle = obstacle_manager.find_obstacle(std::get<0>(obs));
    if (!ptr_obstacle) {
      MSD_LOG(ERROR, "Lateral Behavior Planner Obstacle[%d] do not exist!",
              std::get<0>(obs));
      continue;
    }

    if (!ptr_obstacle->has_sl_polygon_seq()) {
      std::string fail_reason{};
      if (!StGraphGenerator::compute_obs_sl_polygon_seq(
              ptr_obstacle, baseline_info_, time_range, time_gap,
              enable_heuristic, fail_reason)) {
        MSD_LOG(WARN, "obtacle [%d] polygon construction failed",
                ptr_obstacle->Id());
        continue;
      }
    }

    for (int i = 0; i < 25; ++i) {
      if (isSparseMode && i % 2 == 1)
        continue;

      PolygonWithT t_p;
      if (ptr_obstacle->sl_polygon_seq().EvaluateByTime(i * delta_t_, &t_p)) {
        t_polygon.emplace_back(t_p.second);
      } else {
        MSD_LOG(WARN, "obtacle [%d] polygon do not exist at time [%f]s",
                ptr_obstacle->Id(), i * delta_t_);
        break;
      }
    }
    sl_polygon_[std::get<0>(obs)] = t_polygon;

    // judge cutin
    distinguish_cutin(ptr_obstacle, std::get<1>(obs));
    MSD_LOG(INFO, "[cutin judge] obs_id = %d, is_cutin = %d", std::get<0>(obs),
            ptr_obstacle->IsCutin());
  }
  return true;
}

void ExtractPathBound::update_obstacle_priority() {
  obs_priority_.clear();
  const auto &pre_planning_result =
      PlanningContext::Instance()->planning_status().pre_planning_result;
  auto &obstacle_manager = baseline_info_->obstacle_manager();
  for (const auto &obs : sl_polygon_) {
    int priority = 1;
    auto ptr_obstacle = obstacle_manager.find_obstacle(obs.first);
    SLBoundary perception_sl_boundary;
    if (ptr_obstacle != nullptr) {
      perception_sl_boundary = ptr_obstacle->PerceptionSLBoundary();
    } else {
      return;
    }
    double theta_refline =
        (perception_sl_boundary.start_l + perception_sl_boundary.end_l >
         2.0 * adc_l_)
            ? std::atan2(perception_sl_boundary.end_l - adc_l_,
                         perception_sl_boundary.end_s - adc_s_)
            : std::atan2(perception_sl_boundary.start_l - adc_l_,
                         perception_sl_boundary.end_s - adc_s_);

    if (std::fabs(planning_math::WrapAngle(theta_refline) - M_PI) <
        M_PI / 3.0) {
      priority = -1;

      const auto &last_vel_array = pre_planning_result.traj_vel_array;
      int index_adc_distance = static_cast<int>(2.0 / FLAGS_trajectory_density);
      int index_obs_distance = static_cast<int>(2.0 / 0.4);
      if (index_adc_distance < last_vel_array.size() &&
          index_obs_distance < obs.second.size()) {
        if (last_vel_array[index_adc_distance].distance + adc_s_ +
                ConfigurationContext::Instance()->get_vehicle_param().width /
                    2.0 >
            obs.second[index_obs_distance].max_x()) {
          priority = -2;
        }
      }
    }
    obs_priority_[obs.first] = priority;

    if ((std::fabs(
             planning_math::WrapAngle(ptr_obstacle->Yaw_relative_frenet()) -
             M_PI) < M_PI / 3.0) &&
        ptr_obstacle->speed() > 2.0) {
      obs_reverse_.insert(obs.first);
      MSD_LOG(INFO, "[TimeInterval] reverse obs : %d", obs.first);
    }
  }
}

void ExtractPathBound::GetSpeedLimit(const std::vector<AvoidObstacle> &obs_avd,
                                     const PathBound &des_bound,
                                     const PathBound &des_bound_reverse,
                                     const PathBound &path_bound) {
  bool is_VRU = false;
  bool is_TRUCK = false;
  bool is_transverse = false;
  bool is_retrograde = false;
  double nearest_nbo_s = std::numeric_limits<double>::infinity();
  double nearest_nbo_s_end = std::numeric_limits<double>::infinity();
  double nearest_nbo_l = std::numeric_limits<double>::infinity();
  double nearest_nbo_relative_l = std::numeric_limits<double>::infinity();
  std::string nearest_nbo_dir = "none";
  int nearest_nbo_id = 0;
  constexpr double kMinVLimit = 3.0;
  constexpr double kMinVLimitTRUCK = 6.0;
  constexpr double kMaxVLimit = 50.0 / 3.6;

  auto &obstacle_manager = baseline_info_->obstacle_manager();
  for (auto obs : obs_avd) { // 只考虑静态障碍物
    if (std::get<0>(obs) == -1000) {
      continue;
    }
    auto ptr_obstacle = obstacle_manager.find_obstacle(std::get<0>(obs));
    if (!ptr_obstacle) {
      MSD_LOG(ERROR, "Lateral Behavior Planner Obstacle[%d] do not exist!",
              std::get<0>(obs));
      continue;
    }

    if (ptr_obstacle->IsStatic() &&
        ptr_obstacle->Type() != ObjectType::CONE_BUCKET) {
      constexpr double collision_safety_range = 1.0;
      const double obstacle_back_s =
          ptr_obstacle->PerceptionSLBoundary().start_s;
      bool is_close_on_left = (std::get<1>(obs) == "left") &&
                              (adc_l_ -
                                   ConfigurationContext::Instance()
                                       ->get_vehicle_param()
                                       .right_edge_to_center -
                                   collision_safety_range <
                               ptr_obstacle->PerceptionSLBoundary().end_l);

      bool is_close_on_right = (std::get<1>(obs) == "right") &&
                               (ptr_obstacle->PerceptionSLBoundary().start_l -
                                    collision_safety_range <
                                adc_l_ + ConfigurationContext::Instance()
                                             ->get_vehicle_param()
                                             .left_edge_to_center);

      if (obstacle_back_s > adc_s_ && obstacle_back_s < nearest_nbo_s &&
          (is_close_on_left || is_close_on_right)) {
        nearest_nbo_id = std::get<0>(obs);
        nearest_nbo_dir = std::get<1>(obs);
        nearest_nbo_s = obstacle_back_s;
        nearest_nbo_s_end = ptr_obstacle->PerceptionSLBoundary().end_s;
        nearest_nbo_l = is_close_on_left
                            ? ptr_obstacle->PerceptionSLBoundary().end_l
                            : ptr_obstacle->PerceptionSLBoundary().start_l;
        nearest_nbo_relative_l =
            is_close_on_left ? adc_sl_boundary_.start_l -
                                   ptr_obstacle->PerceptionSLBoundary().end_l
                             : ptr_obstacle->PerceptionSLBoundary().start_l -
                                   adc_sl_boundary_.end_l;
        is_VRU = ptr_obstacle->Type() == ObjectType::PEDESTRIAN ||
                 ptr_obstacle->Type() == ObjectType::OFO;
        is_TRUCK = ptr_obstacle->Type() == ObjectType::TRANSPORT_TRUNK ||
                   ptr_obstacle->Type() == ObjectType::BUS ||
                   ptr_obstacle->Type() == ObjectType::ENGINEER_TRUCK;
        is_transverse =
            std::fabs(ptr_obstacle->PerceptionSLBoundary().end_l -
                      ptr_obstacle->PerceptionSLBoundary().start_l) >
            0.8 * std::fabs(ptr_obstacle->PerceptionSLBoundary().end_s -
                            ptr_obstacle->PerceptionSLBoundary().start_s);
      }
    }
  }

  double dv_nbo_min =
      (is_TRUCK && !is_transverse) ? kMinVLimitTRUCK : kMinVLimit;
  double dv_nbo_max = kMaxVLimit;
  double l_min = (is_VRU || is_transverse || is_TRUCK) ? 0.5 : 0.1;
  double dl_min = (is_VRU || is_transverse || is_TRUCK) ? 1.0 : 0.5;
  double s_min = 20.0;
  double ds_min = 20.0;
  double dv_nbo = dv_nbo_min +
                  max(nearest_nbo_s - adc_s_ - s_min, 0.0) *
                      std::abs(dv_nbo_max - dv_nbo_min) / ds_min +
                  max(nearest_nbo_relative_l - l_min, 0.0) *
                      std::abs(dv_nbo_max - dv_nbo_min) / dl_min;
  double vel_limit = clip(dv_nbo, kMaxVLimit, kMinVLimit);
  if (nearest_nbo_id > 0) {
    MSD_LOG(INFO,
            "[Extract] nearest_nbo_id[%d] nearest_nbo_s = %.3f nearest_nbo_l = "
            "%.3f nearest_nbo_dir = %s "
            "dv_nbo = %.3f kMaxVLimit = %.3f kMinVLimit = %.3f",
            nearest_nbo_id, nearest_nbo_s, nearest_nbo_l,
            nearest_nbo_dir.c_str(), dv_nbo, kMaxVLimit, kMinVLimit);
  }

  double space_min = 1.0e6;
  for (int i = 1; i < path_bound.size(); ++i) {
    if ((nearest_nbo_s < std::get<0>(path_bound[i]) &&
         nearest_nbo_s_end > std::get<0>(path_bound[i])) ||
        (nearest_nbo_s > std::get<0>(path_bound[i - 1]) &&
         nearest_nbo_s_end < std::get<0>(path_bound[i]))) {
      if (nearest_nbo_dir == "left") {
        double map_space = std::get<2>(path_bound[i]) - nearest_nbo_l;
        space_min = std::fmin(map_space, space_min);
      } else {
        double map_space = nearest_nbo_l - std::get<1>(path_bound[i]);
        space_min = std::fmin(map_space, space_min);
      }
    }
  }
  for (const auto &obj : obs_avd) {
    if (sl_polygon_.find(std::get<0>(obj)) != sl_polygon_.end() &&
        std::get<0>(obj) != nearest_nbo_id &&
        std::get<1>(obj) != nearest_nbo_dir) {
      const auto &polygon_list = sl_polygon_[std::get<0>(obj)];
      for (int i = 1; i < polygon_list.size(); ++i) {
        if ((polygon_list[i].min_x() < nearest_nbo_s_end &&
             polygon_list[i].max_x() > nearest_nbo_s) ||
            (polygon_list[i - 1].max_x() < nearest_nbo_s &&
             polygon_list[i].min_x() > nearest_nbo_s_end)) {
          if (std::get<1>(obj) == "left") {
            space_min =
                std::fmin(nearest_nbo_l - polygon_list[i].max_y(), space_min);
          } else {
            space_min =
                std::fmin(polygon_list[i].min_y() - nearest_nbo_l, space_min);
          }
        }
      }
    }
  }
  if (space_min > 4.5) {
    vel_limit = std::fmax(vel_limit, 8.0);
  }
  MSD_LOG(INFO,
          "[Extract] vel_limit = %.3f nearest_nbo_s = %.3f nearest_nbo_s_end = "
          "%.3f",
          vel_limit, nearest_nbo_s, nearest_nbo_s_end);
  v_limit_.emplace_back(nearest_nbo_s, vel_limit);
  v_limit_.emplace_back(nearest_nbo_s_end, vel_limit);

  for (int i = 0; i < des_bound.size() && i < des_bound_reverse.size(); i++) {
    double curr_s = std::get<0>(des_bound[i]);
    double left_bound = std::get<2>(des_bound[i]);
    double right_bound = std::get<1>(des_bound[i]);
    if (i < 10 || curr_s - std::get<0>(des_bound[0]) < 20) {
      left_bound = std::fmin(left_bound, std::get<2>(des_bound_reverse[i]));
      right_bound = std::fmax(right_bound, std::get<1>(des_bound_reverse[i]));
    }
    double avd_width = left_bound - right_bound;
    double v_max = max_vel_;
    if (curr_s - adc_s_ < 60 && avd_width < 0) {
      v_max = (avd_width + 1) * 6 + std::fmax(curr_s - adc_s_ - 4, 0) * 0.02;
      v_max = clip(v_max, max_vel_, 2.0);
      v_limit_.emplace_back(curr_s, v_max);
      MSD_LOG(INFO, "[Extract] curr_s = %.3f  v_max = %.3f", curr_s, v_max);
    }
  }
}

// 根据横向期望偏移，纵向扩展虚拟车道
void ExtractPathBound::GetLatOffset(const PathBound &des_bound) {
  const auto &lateral_behavior_state =
      context_->state_machine_output().curr_state;
  const auto &lateral_output = context_->lateral_behavior_planner_output();
  double lat_behavior_offset = 0;
  if (lateral_output.premoving) { // 变道等待偏移　
    if (lateral_behavior_state == ROAD_LC_RWAIT ||
        lateral_behavior_state == ROAD_LC_RBACK) {
      lat_behavior_offset = std::min(
          -0.5 * (lateral_output.flane_width -
                  ConfigurationContext::Instance()->get_vehicle_param().width) +
              0.15,
          lat_behavior_offset);
    } else if (lateral_behavior_state == ROAD_LC_LWAIT ||
               lateral_behavior_state == ROAD_LC_LBACK) {
      lat_behavior_offset = std::max(
          0.5 * (lateral_output.flane_width -
                 ConfigurationContext::Instance()->get_vehicle_param().width) -
              0.15,
          lat_behavior_offset);
    }
  }

  std::vector<std::tuple<double, double>> offset_list;
  double last_offset = lat_behavior_offset;
  for (int i = 1; i < num_knots_; i++) {
    double offset = lat_behavior_offset;
    double right_offset = std::get<1>(des_bound[i]);
    double left_offset = std::get<2>(des_bound[i]);

    if (left_offset > right_offset) {
      if (offset < right_offset) {
        offset = right_offset;
      } else if (offset > left_offset) {
        offset = left_offset;
      }
      double left_bound = std::fmin(last_offset + 0.15, left_offset);
      double right_bound = std::fmax(last_offset - 0.15, right_offset);
      if (left_bound > right_bound && i > 1)
        offset = clip(offset, left_bound, right_bound);
    } else {
      offset = (left_offset + right_offset) / 2.;
    }

    offset_list.emplace_back(std::get<0>(des_bound[i]), offset);
    last_offset = offset;
  }

  for (int i = offset_list.size() - 1; i > 0; i--) {
    double offset = std::get<1>(offset_list[i]);
    double right_offset = std::get<1>(des_bound[i]);
    double left_offset = std::get<2>(des_bound[i]);
    if (left_offset > right_offset) {
      double left_bound = std::fmin(last_offset + 0.15, left_offset);
      double right_bound = std::fmax(last_offset - 0.15, right_offset);
      if (left_bound > right_bound && i < offset_list.size() - 1) {
        offset = clip(offset, left_bound, right_bound);
        std::get<1>(offset_list[i]) = offset;
      }
    }
    last_offset = offset;
  }

  lat_offset_.emplace_back(offset_list[0]);
  for (int i = 1; i < (int)offset_list.size() - 1; i++) {
    if (std::abs(std::get<1>(offset_list[i]) -
                 std::get<1>(lat_offset_.back())) > 0.14) {
      lat_offset_.emplace_back(offset_list[i]);
    }
  }
  lat_offset_.emplace_back(offset_list.back());
}

void ExtractPathBound::CheckStaticIgnore(BlockType &block_type) {
  std::vector<AvoidObstacle> danger_obs(static_obs_avd_);
  for (const auto &obs : dynamic_obs_avd_) {
    if (block_type.obs_avd_time.find(std::get<0>(obs)) !=
            block_type.obs_avd_time.end() &&
        !std::get<0>(block_type.obs_avd_time[std::get<0>(obs)]) &&
        sl_polygon_.find(std::get<0>(obs)) != sl_polygon_.end()) {
      const auto &polygon_list = sl_polygon_[std::get<0>(obs)];
      if (polygon_list.empty()) {
        continue;
      }
      const auto &polygon_init = polygon_list[0];
      double centre_x = (polygon_init.min_x() + polygon_init.max_x()) / 2;
      double centre_y = (polygon_init.min_y() + polygon_init.max_y()) / 2;
      double half_obs_length =
          (polygon_init.max_x() - polygon_init.min_x()) / 2;
      if (std::abs(adc_s_ - centre_x) <
              half_obs_length +
                  ConfigurationContext::Instance()->get_vehicle_param().length /
                      2 &&
          polygon_init.min_y() - 4 < adc_l_ &&
          polygon_init.max_y() + 4 > adc_l_) {
        double block_time =
            std::get<1>(block_type.obs_avd_time[std::get<0>(obs)]) + 0.4;
        double follow_time =
            std::min(adc_vel_ / std::abs(min_acc_), block_time);
        double adc_follow_s = adc_s_ + adc_vel_ * follow_time +
                              min_acc_ * pow(follow_time, 2) / 2;
        double overtake_time = block_time;
        double adc_overtake_s = adc_s_ + adc_vel_ * overtake_time +
                                max_acc_ * pow(overtake_time, 2) / 2;
        size_t index = static_cast<size_t>(block_time / 0.4);
        if (index + 1 > polygon_list.size()) {
          continue;
        }
        const auto &polygon = polygon_list[index];
        if (polygon.min_x() < adc_follow_s + ConfigurationContext::Instance()
                                                     ->get_vehicle_param()
                                                     .length /
                                                 2 &&
            polygon.max_x() > adc_overtake_s - ConfigurationContext::Instance()
                                                       ->get_vehicle_param()
                                                       .length /
                                                   2) {
          danger_obs.push_back(obs);
          std::map<int, std::vector<double>> obs_block_time;
          std::vector<std::tuple<int, std::string, EdgePoints>> edge_list;
          get_obstacle_edge(static_cast<int>(block_time / 0.4), danger_obs,
                            edge_list);

          std::unordered_set<int> block_obs;
          PathBound path_bound_tmp;
          for (int i = 0; i < num_knots_; i++) {
            path_bound_tmp.emplace_back(adc_s_list_[i], -10., 10.);
            path_bound_tmp.emplace_back(adc_s_list_[i], -10., 10.);
          }
          if (!GetBoundaryFromObstacles(edge_list, block_type.static_block_s,
                                        path_bound_tmp, block_obs)) {
            for (auto &it : block_obs) {
              if (it != std::get<0>(obs)) {
                block_type.obs_avd_time[it] = std::make_tuple(false, 0);
                MSD_LOG(INFO,
                        "[Extract] CheckStaticIgnore obs_id[%d] ignore[%d]", it,
                        false);
              }
            }
          }
          danger_obs.pop_back();
        }
      }
    }
  }
}

void ExtractPathBound::distinguish_cutin(const Obstacle *ptr_obstacle,
                                         std::string obs_side_pass) {
  PlanningStatus *planning_status = context_->mutable_planning_status();
  PlanningResult &planning_result = planning_status->planning_result;
  auto scene_status = planning_status->lane_status.status;
  if (!ptr_obstacle || scene_status != LaneStatus::LANE_KEEP) {
    MSD_LOG(INFO, "distinguish_cutin disable");
    return;
  }

  const double kDeltaT = 0.2;
  const auto &obs_sl_polygon_seq = ptr_obstacle->sl_polygon_seq();
  PolygonWithT obs_sl_polygon_start, obs_sl_polygon_mid, obs_sl_polygon_end;
  (void)obs_sl_polygon_seq.EvaluateByTime(0.0, &obs_sl_polygon_start);
  (void)obs_sl_polygon_seq.EvaluateByTime(FLAGS_lon_decision_time_horizon / 2.0,
                                          &obs_sl_polygon_mid);
  (void)obs_sl_polygon_seq.EvaluateByTime(
      FLAGS_lon_decision_time_horizon - kDeltaT, &obs_sl_polygon_end);
  if (obs_sl_polygon_start.second.points().size() < 3 ||
      obs_sl_polygon_end.second.points().size() < 3 ||
      obs_sl_polygon_mid.second.points().size() < 3) {
    MSD_LOG(INFO, "distinguish_cutin polygon size is small");
    return;
  }

  double ref_curv_avg_abs_max = 0.;
  double ref_curv_avg_pos = 0.;
  double ref_curv_avg_neg = 0.;
  int num_pos = 0;
  int num_neg = 0;
  for (auto tmp_s : adc_s_list_) {
    if (tmp_s < frenet_coord_->GetLength() &&
        frenet_coord_->GetRefCurveCurvature(tmp_s) > 0.) {
      ref_curv_avg_pos += frenet_coord_->GetRefCurveCurvature(tmp_s);
      num_pos++;
    } else if (tmp_s < frenet_coord_->GetLength() &&
               frenet_coord_->GetRefCurveCurvature(tmp_s) < 0.) {
      ref_curv_avg_neg += frenet_coord_->GetRefCurveCurvature(tmp_s);
      num_neg++;
    }
  }
  ref_curv_avg_pos = (num_pos == 0 ? 0. : ref_curv_avg_pos / num_pos);
  ref_curv_avg_neg = (num_neg == 0 ? 0. : ref_curv_avg_neg / num_neg);
  ref_curv_avg_abs_max = std::max(ref_curv_avg_pos, std::abs(ref_curv_avg_neg));

  // obs prediction interference adc
  bool obs_interference = false;
  auto polygon_list = sl_polygon_[ptr_obstacle->Id()];
  const double lk_buffer = 0.3;
  int interference_index = -1;
  if (obs_side_pass == "left" &&
      obs_sl_polygon_end.second.max_y() >
          obs_sl_polygon_start.second.max_y() + 0.2) {
    for (int i = 0; i < polygon_list.size(); i++) {
      if ((ref_curv_avg_abs_max < 0.004 && ptr_obstacle->speed() < 15.0 &&
           i > 10) ||
          (ref_curv_avg_abs_max < 0.004 && ptr_obstacle->speed() > 15.0 &&
           i > 7))
        break;
      else if (ref_curv_avg_abs_max > 0.004 && i > 5)
        break;
      if (polygon_list[i].max_y() >
              -ConfigurationContext::Instance()->get_vehicle_param().width /
                      2.0 -
                  lk_buffer &&
          ptr_obstacle->PerceptionSLBoundary().end_l <
              -ConfigurationContext::Instance()->get_vehicle_param().width /
                      2.0 -
                  lk_buffer) {
        obs_interference = true;
        interference_index = i;
        break;
      }
    }
  } else if (obs_side_pass == "right" &&
             obs_sl_polygon_end.second.min_y() <
                 obs_sl_polygon_start.second.min_y() - 0.2) {
    for (int i = 0; i < polygon_list.size(); i++) {
      if ((ref_curv_avg_abs_max < 0.004 && ptr_obstacle->speed() < 15.0 &&
           i > 10) ||
          (ref_curv_avg_abs_max < 0.004 && ptr_obstacle->speed() > 15.0 &&
           i > 7))
        break;
      else if (ref_curv_avg_abs_max > 0.004 && i > 5)
        break;
      if (polygon_list[i].min_y() <
              ConfigurationContext::Instance()->get_vehicle_param().width /
                      2.0 +
                  lk_buffer &&
          ptr_obstacle->PerceptionSLBoundary().start_l >
              ConfigurationContext::Instance()->get_vehicle_param().width /
                      2.0 +
                  lk_buffer) {
        obs_interference = true;
        interference_index = i;
        break;
      }
    }
  }

  // params:
  double r_center_checker = 1.7;
  double r_bound_checker = 1.3;
  double PI = 3.1415926535;
  double theta_merge = PI / 9.0;

  auto obs_center_l = (ptr_obstacle->PerceptionSLBoundary().start_l +
                       ptr_obstacle->PerceptionSLBoundary().end_l) /
                      2.0;
  auto dyn_obs_r_abs_min =
      min(min(std::abs(ptr_obstacle->PerceptionSLBoundary().start_l),
              std::abs(ptr_obstacle->PerceptionSLBoundary().end_l)),
          max(0.0, ptr_obstacle->PerceptionSLBoundary().start_l *
                       ptr_obstacle->PerceptionSLBoundary().end_l));

  bool is_cutin =
      obs_interference ||
      (std::fabs(obs_center_l) < r_center_checker &&
       ptr_obstacle->PerceptionSLBoundary().end_s >
           adc_s_ +
               ConfigurationContext::Instance()->get_vehicle_param().length /
                   2 &&
       (std::abs(obs_sl_polygon_mid.second.max_y() +
                 obs_sl_polygon_mid.second.min_y()) < 2.5)) ||
      (dyn_obs_r_abs_min < r_bound_checker &&
       ptr_obstacle->PerceptionSLBoundary().end_s >
           adc_s_ +
               ConfigurationContext::Instance()->get_vehicle_param().length /
                   2 &&
       (std::abs((obs_sl_polygon_mid.second.max_y() +
                  obs_sl_polygon_mid.second.min_y()) /
                 2.0) < 2.5));

  // ptr_obstacle->SetCutinProperty(is_cutin);

  MSD_LOG(INFO,
          "distinguish_cutin id = %d, is_cutin = %d, obs_interference = %d, "
          "interference_index = %d, "
          "obs_center_l = %.2f, dyn_obs_r_abs_min = "
          "%.2f, Yaw_relative_frenet = %.3f, ref_curv_avg_pos = %.4f, "
          "ref_curv_avg_neg = %.4f, ref_curv_avg_abs_max = %.4f",
          ptr_obstacle->Id(), is_cutin, obs_interference, interference_index,
          obs_center_l, dyn_obs_r_abs_min,
          std::fabs(ptr_obstacle->Yaw_relative_frenet()), ref_curv_avg_pos,
          ref_curv_avg_neg, ref_curv_avg_abs_max);

  return;
}
void ExtractPathBound::check_side_collision(
    const std::vector<AvoidObstacle> &obs_avd, const BlockType &block_type) {
  std::string avd_direction = "none";
  for (auto &offset : lat_offset_) {
    if (std::get<1>(offset) > 0.6 && adc_l_ - std::get<1>(offset) < -0.4) {
      avd_direction = "left";
      break;
    } else if (std::get<1>(offset) < -0.6 &&
               adc_l_ - std::get<1>(offset) > 0.4) {
      avd_direction = "right";
      break;
    }
  }

  bool change_ignore = false;
  if (avd_direction == "left") {
    for (const auto &obs : dynamic_obs_avd_) {
      if (std::get<1>(obs) == "right" &&
          !sl_polygon_[std::get<0>(obs)].empty()) {
        const auto &polygon_list = sl_polygon_[std::get<0>(obs)];
        const auto &edge_curr = polygon_list.front();
        if (edge_curr.min_x() > adc_s_ +
                                    ConfigurationContext::Instance()
                                            ->get_vehicle_param()
                                            .length /
                                        2.0 +
                                    0.5 ||
            edge_curr.min_y() - adc_l_ -
                    ConfigurationContext::Instance()
                            ->get_vehicle_param()
                            .width /
                        2.0 >
                1.5 ||
            edge_curr.max_x() < adc_s_ -
                                    ConfigurationContext::Instance()
                                            ->get_vehicle_param()
                                            .length /
                                        2.0 -
                                    20)
          continue;
        if (edge_curr.max_x() < adc_s_ -
                                    ConfigurationContext::Instance()
                                            ->get_vehicle_param()
                                            .length /
                                        2.0 -
                                    0.5 &&
            adc_l_ + ConfigurationContext::Instance()
                             ->get_vehicle_param()
                             .width /
                         2.0 >
                edge_curr.min_y())
          continue;
        if (edge_curr.min_x() < adc_s_ + ConfigurationContext::Instance()
                                                 ->get_vehicle_param()
                                                 .length /
                                             2.0 &&
            edge_curr.max_x() > adc_s_ - ConfigurationContext::Instance()
                                                 ->get_vehicle_param()
                                                 .length /
                                             2.0 &&
            edge_curr.min_y() - adc_l_ -
                    ConfigurationContext::Instance()
                            ->get_vehicle_param()
                            .width /
                        2.0 <
                1.0) {
          change_ignore = true;
          break;
        }

        bool loop_end = false;
        for (int i = 0; i < polygon_list.size(); ++i) {
          for (int j = 0; j < (int)lat_offset_.size() - 1; ++j) {
            if (polygon_list[i].max_x() > std::get<0>(lat_offset_[j]) &&
                polygon_list[i].max_x() < std::get<0>(lat_offset_[j + 1])) {
              if (polygon_list[i].min_y() < std::get<0>(lat_offset_[j]) +
                                                ConfigurationContext::Instance()
                                                        ->get_vehicle_param()
                                                        .width /
                                                    2.0 +
                                                0.2) {
                double s = adc_s_ + adc_vel_ * 2 * i * delta_t_ +
                           0.3 * pow(2 * i * delta_t_, 2) / 2.0;
                if (s - ConfigurationContext::Instance()
                                ->get_vehicle_param()
                                .length /
                            2.0 >
                    polygon_list[i].max_x()) {
                  change_ignore = true;
                }
                loop_end = true;
              }
            }
            break;
          }
          if (loop_end)
            break;
        }
        if (change_ignore)
          break;
      }
    }
    if (change_ignore) {
      for (const auto &obs : obs_avd) {
        if (std::get<1>(obs) == "left" &&
            block_type.obs_avd_time.find(std::get<0>(obs)) !=
                block_type.obs_avd_time.end() &&
            std::get<0>(block_type.obs_avd_time.at(std::get<0>(obs)))) {
          if (!sl_polygon_[std::get<0>(obs)].empty() &&
              sl_polygon_[std::get<0>(obs)].front().min_x() >
                  adc_s_ + ConfigurationContext::Instance()
                                   ->get_vehicle_param()
                                   .length /
                               2.0) {
            dynamic_obs_avd_.emplace_back(obs);
            MSD_LOG(INFO, "dynamic insert : %d", std::get<0>(obs));
          }
        }
      }
    }
  } else if (avd_direction == "right") {
    for (const auto &obs : dynamic_obs_avd_) {
      if (std::get<1>(obs) == "left" &&
          !sl_polygon_[std::get<0>(obs)].empty()) {
        const auto &polygon_list = sl_polygon_[std::get<0>(obs)];
        const auto &edge_curr = polygon_list.front();
        if (edge_curr.min_x() > adc_s_ +
                                    ConfigurationContext::Instance()
                                            ->get_vehicle_param()
                                            .length /
                                        2.0 +
                                    0.5 ||
            adc_l_ -
                    ConfigurationContext::Instance()
                            ->get_vehicle_param()
                            .width /
                        2.0 -
                    edge_curr.max_y() >
                1.5 ||
            edge_curr.max_x() < adc_s_ -
                                    ConfigurationContext::Instance()
                                            ->get_vehicle_param()
                                            .length /
                                        2.0 -
                                    20)
          continue;
        if (edge_curr.max_x() < adc_s_ -
                                    ConfigurationContext::Instance()
                                            ->get_vehicle_param()
                                            .length /
                                        2.0 -
                                    0.5 &&
            adc_l_ - ConfigurationContext::Instance()
                             ->get_vehicle_param()
                             .width /
                         2.0 <
                edge_curr.max_y())
          continue;
        if (edge_curr.min_x() < adc_s_ + ConfigurationContext::Instance()
                                                 ->get_vehicle_param()
                                                 .length /
                                             2.0 &&
            edge_curr.max_x() > adc_s_ - ConfigurationContext::Instance()
                                                 ->get_vehicle_param()
                                                 .length /
                                             2.0 &&
            adc_l_ -
                    ConfigurationContext::Instance()
                            ->get_vehicle_param()
                            .width /
                        2.0 -
                    edge_curr.max_y() <
                1.0) {
          change_ignore = true;
          break;
        }

        bool loop_end = false;
        for (int i = 0; i < polygon_list.size(); ++i) {
          for (int j = 0; j < (int)lat_offset_.size() - 1; ++j) {
            if (polygon_list[i].max_x() > std::get<0>(lat_offset_[j]) &&
                polygon_list[i].max_x() < std::get<0>(lat_offset_[j + 1])) {
              if (polygon_list[i].max_y() > std::get<0>(lat_offset_[j]) -
                                                ConfigurationContext::Instance()
                                                        ->get_vehicle_param()
                                                        .width /
                                                    2.0 -
                                                0.2) {
                double s = adc_s_ + adc_vel_ * 2 * i * delta_t_ +
                           0.3 * pow(2 * i * delta_t_, 2) / 2.0;
                if (s - ConfigurationContext::Instance()
                                ->get_vehicle_param()
                                .length /
                            2.0 >
                    polygon_list[i].max_x()) {
                  change_ignore = true;
                }
                loop_end = true;
              }
            }
            break;
          }
          if (loop_end)
            break;
        }
        if (change_ignore)
          break;
      }
    }
    if (change_ignore) {
      for (const auto &obs : obs_avd) {
        if (std::get<1>(obs) == "right" &&
            block_type.obs_avd_time.find(std::get<0>(obs)) !=
                block_type.obs_avd_time.end() &&
            std::get<0>(block_type.obs_avd_time.at(std::get<0>(obs)))) {
          if (!sl_polygon_[std::get<0>(obs)].empty() &&
              sl_polygon_[std::get<0>(obs)].front().min_x() >
                  adc_s_ + ConfigurationContext::Instance()
                                   ->get_vehicle_param()
                                   .length /
                               2.0) {
            dynamic_obs_avd_.emplace_back(obs);
            MSD_LOG(INFO, "dynamic insert : %d", std::get<0>(obs));
          }
        }
      }
    }
  }
}

void ExtractPathBound::UpdateAvdInfo(const BlockType &block_type) {
  PlanningStatus *planning_status = context_->mutable_planning_status();
  PlanningResult &planning_result = planning_status->planning_result;
  auto &obstacle_manager = baseline_info_->obstacle_manager();
  for (auto &obs : sl_polygon_) {
    int loop = 0;
    if (avd_loop_.find(obs.first) != avd_loop_.end()) {
      loop = avd_loop_[obs.first];
    }
    bool ignore = true;
    double time_buffer = 8;
    if (block_type.obs_avd_time.find(obs.first) !=
        block_type.obs_avd_time.end()) {
      ignore = std::get<0>(block_type.obs_avd_time.at(obs.first));
      time_buffer = std::get<1>(block_type.obs_avd_time.at(obs.first));
    }
    // 不在 block_type 中的障碍物，时间区间为8
    // 如705行中自车最快也追不上的障碍物，不会进入block_type!!!

    // cutin  set false forced
    auto ptr_obstacle = obstacle_manager.find_obstacle(obs.first);
    // if (ptr_obstacle && ptr_obstacle->IsCutin())
    //   ignore = false;

    AvdInfo obs_info{obs_priority_[obs.first], loop, ignore, time_buffer};
    planning_result.avd_info[obs.first] = obs_info;
  }
}

} // namespace msquare