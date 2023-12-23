#include "common/baseline_info.h"
#include "common/config_context.h"
#include "common/math/math_utils.h"
#include "common/utils/polyfit.h"
#include "common/world_model.h"
#include <cmath>

namespace msquare {

BaseLineInfo::BaseLineInfo() {
  obstacle_manager_ = std::make_unique<ObstacleManager>();

  frenet_parameters_.zero_speed_threshold = 0.1;
  frenet_parameters_.coord_transform_precision = 0.01;
  frenet_parameters_.step_s = 0.3;
  frenet_parameters_.coarse_step_s = 1.0;
  frenet_parameters_.optimization_gamma = 0.5;
  frenet_parameters_.max_iter = 15;

  is_valid_ = false;
  is_update_ = false;
}

void BaseLineInfo::update(const std::shared_ptr<WorldModel> &world_model,
                          int lane_id) {
  double start_time = MTIME()->timestamp().ms();
  lane_id_ = lane_id;
  is_valid_ = true;
  is_update_ = true;
  MLOG_PROFILING("construct_reference_line");
  MSD_LOG(INFO, "%s: %d", __FUNCTION__, world_model->is_acc_mode());
  MSD_LOG(INFO, "DEBUG_CJ3:lane_id:%d", lane_id);
  if (!world_model->is_acc_mode()) {
    if (!construct_reference_line(world_model)) {
      MSD_LOG(INFO,
              "construct baseline [%d] failed when building reference line.",
              lane_id);
      is_valid_ = false;
      return;
    }
  } else {
    if (!construct_reference_line_cjacc(world_model)) {
      MSD_LOG(INFO,
              "ERROR_CJ0 construct baseline [%d] failed when building "
              "reference line.",
              lane_id);
      is_valid_ = false;
      return;
    } else {
      MSD_LOG(INFO, "DEBUG_CJ3 Time:%3.2f", MTIME()->timestamp().sec());
      MSD_LOG(INFO, "DEBUG_CJ3 construct reference line cjacc success!");
    }
  }

  MLOG_PROFILING("construct_obstacle_manager");
  if (!construct_obstacle_manager(world_model)) {
    MSD_LOG(
        INFO,
        "construct baseline [%d] failed when constructing obstacle manager.",
        lane_id);
    is_valid_ = false;
    return;
  }

  // set properties for current lane
  if (!is_adc_on_lane()) {
    is_change_lane_ = true;
  } else {
    is_change_lane_ = false;
  }

  constexpr double virtual_lane_width = 1.8;
  compute_convex_hull(virtual_lane_width, virtual_lane_width);

  if (lane_id == 0) {
    CostTime cost_time =
        CostTime{"baseline", MTIME()->timestamp().ms() - start_time};
    auto *planner_debug = PlanningContext::Instance()->mutable_planner_debug();
    planner_debug->cost_time.emplace_back(cost_time);
  }
}

bool BaseLineInfo::construct_reference_line(
    const std::shared_ptr<WorldModel> &world_model) {
  static std::vector<double> vx, vy;
  vx.clear();
  vy.clear();
  frenet_enu_x_.clear();
  frenet_enu_y_.clear();
  const auto &map_info_manager = world_model->get_map_info_manager();
  const auto &cart_ego_state_manager =
      world_model->get_cart_ego_state_manager();

  if (lane_id_ == 0) {
    lane_type_ = static_cast<uint8_t>(
        map_info_manager.get_map_info().current_lane_type());
  } else if (lane_id_ == -1) {
    lane_type_ =
        static_cast<uint8_t>(map_info_manager.get_map_info().left_lane_type());
  } else if (lane_id_ == 1) {
    lane_type_ =
        static_cast<uint8_t>(map_info_manager.get_map_info().right_lane_type());
  }

  const auto &map_poi_info = world_model->get_map_info().map_pois_data();
  const auto &map_info = world_model->get_map_info();

  MSD_LOG(INFO, "[baseline] id %d type %d", lane_id_, int(lane_type_));

  // get all cartesian related info
  auto &baseline_ego_state = ego_state_manager_.get_mutable_ego_state();
  const auto &cart_ego_state = cart_ego_state_manager.get_cart_ego_state();
  baseline_ego_state.ego_pose = cart_ego_state.ego_pose;
  baseline_ego_state.ego_pose_raw = cart_ego_state.ego_pose_raw;
  baseline_ego_state.ego_enu = cart_ego_state.ego_enu;
  baseline_ego_state.ego_acc = cart_ego_state.ego_acc;
  baseline_ego_state.ego_carte = cart_ego_state.ego_carte;
  baseline_ego_state.ego_vel = cart_ego_state.ego_vel;
  baseline_ego_state.ego_steer_angle = cart_ego_state.ego_steer_angle;
  baseline_ego_state.flag_is_replan = cart_ego_state.flag_is_replan;
  baseline_ego_state.ego_blinker = cart_ego_state.ego_blinker;
  baseline_ego_state.real_init_point = cart_ego_state.real_init_point;
  baseline_ego_state.planning_init_point = cart_ego_state.planning_init_point;
  baseline_ego_state.stitch_trajectory = cart_ego_state.stitch_trajectory;

  ego_state_manager_.set_ego_carte(
      convert_pose2point(ego_state_manager_.get_ego_state().ego_pose));

  if (map_info_manager.ref_trajectories().find(lane_id_) ==
      map_info_manager.ref_trajectories().end()) {
    MSD_LOG(WARN, "reference line [%d] not exist", lane_id_);
    return false;
  }

  if (map_info_manager.ref_trajectories().at(lane_id_).size() < 2) {
    MSD_LOG(WARN, "reference line [%d] invalid", lane_id_);
    return false;
  }

  raw_refline_points_.reserve(
      world_model->get_map_info().current_refline_points().size());
  if (lane_id_ == 0) {
    raw_refline_points_.assign(
        world_model->get_map_info().current_refline_points().begin(),
        world_model->get_map_info().current_refline_points().end());
  } else if (lane_id_ == 1) {
    raw_refline_points_.assign(
        world_model->get_map_info().right_refline_points().begin(),
        world_model->get_map_info().right_refline_points().end());
  } else if (lane_id_ == -1) {
    raw_refline_points_.assign(
        world_model->get_map_info().left_refline_points().begin(),
        world_model->get_map_info().left_refline_points().end());
  } else {
    MSD_LOG(WARN, "invalid baseline id!");
    return false;
  }

  reference_line_.Clear();
  double s = 0;
  auto &car2enu = world_model->get_cart_ego_state_manager().get_car2enu();
  auto &enu_refline_points = raw_refline_points_;
  // MapInfoManager::frame_trans_from_car2enu(enu_refline_points, car2enu);
  mph_assert(!enu_refline_points.empty());
  static std::vector<PathPoint> path_points;
  path_points.clear();
  static std::vector<SLPoint> left_border, right_border;
  left_border.clear();
  right_border.clear();
  auto begin_raw_point = enu_refline_points.front();
  for (const auto &raw_point : enu_refline_points) {
    s += std::hypot(raw_point.enu_point.x - begin_raw_point.enu_point.x,
                    raw_point.enu_point.y - begin_raw_point.enu_point.y);

    PathPoint path_point;
    path_point.x = raw_point.enu_point.x;
    path_point.y = raw_point.enu_point.y;
    path_point.s = s;

    SLPoint left_point, right_point;
    left_point.s = s;
    left_point.l = std::min(1.9, raw_point.lane_width / 2.0);
    right_point.s = s;
    right_point.l = std::max(-1.9, -raw_point.lane_width / 2.0);
    left_border.emplace_back(left_point);
    right_border.emplace_back(right_point);
    path_points.emplace_back(path_point);
    begin_raw_point = raw_point;
  }

  (void)reference_line_.SetDiscretizedPath(DiscretizedPath(path_points));
  (void)reference_line_.SetPathBorder(PolygonalLine(left_border),
                                      PolygonalLine(right_border));

  vx.reserve(map_info_manager.ref_trajectories().at(lane_id_).size());
  vy.reserve(map_info_manager.ref_trajectories().at(lane_id_).size());

  MSD_LOG(ERROR, "FAR_DEBUG: update vx vy");
  auto vx_json = mjson::Json::array(200, mjson::Json(0.0));
  auto vy_json = mjson::Json::array(200, mjson::Json(0.0));
  auto delta_s_json = mjson::Json::array(200, mjson::Json(0.0));
  vx_json.clear();
  vy_json.clear();
  delta_s_json.clear();
  for (auto &p : map_info_manager.ref_trajectories().at(lane_id_)) {
    if (vx.size() > 0) {
      double delta_s =
          std::hypot(p.enu_point.x - vx.back(), p.enu_point.y - vy.back());
      //      MSD_LOG(ERROR, "FAR_DEBUG: vx: %f, vy: %f, delta_s: %f",
      //      p.enu_point.x,
      //              p.enu_point.y, delta_s);
      vx_json.emplace_back(mjson::Json(p.enu_point.x));
      vy_json.emplace_back(mjson::Json(p.enu_point.y));
      delta_s_json.emplace_back(mjson::Json(delta_s));
    }
    vx.emplace_back(p.enu_point.x);
    vy.emplace_back(p.enu_point.y);
  }
  if (lane_id_ == 0) {
    auto vx_vy_json = mjson::Json(mjson::Json::object());
    vx_vy_json["vx"] = mjson::Json(vx_json);
    vx_vy_json["vy"] = mjson::Json(vy_json);
    vx_vy_json["delta_s"] = mjson::Json(delta_s_json);

    auto *planning_status =
        PlanningContext::Instance()->mutable_planning_status();
    planning_status->planning_result.debug_json["vx_xy"] =
        mjson::Json(vx_vy_json);
  }

  double v_caculate_s = std::max(
      10.0 / 3.6, std::max(map_info_manager.get_map_info().v_cruise(),
                           ego_state_manager_.get_ego_state().ego_vel));
  if (nullptr == frenet_coord_) {
    frenet_coord_ = std::make_shared<FrenetCoordinateSystem>();
  }
  frenet_coord_->Update(vx, vy, frenet_parameters_, 0.0,
                        30.0 + v_caculate_s *
                                   (FLAGS_trajectory_time_length + 1 / 10.0));

  ego_state_manager_.set_frenet_coord_system(frenet_coord_);
  Point2D ego_frenet;

  if (frenet_coord_->CartCoord2FrenetCoord(
          ego_state_manager_.get_ego_state().ego_carte, ego_frenet) ==
      TRANSFORM_FAILED) {
    construct_failed_reason_ =
        "[MAP_ERROR] baseline transformed failed ego cart Point2D";
    MSD_LOG(
        WARN,
        "[MAP_ERROR] baseline[%d] transformed Point2D failed ego cart %f %f",
        lane_id_, ego_state_manager_.get_ego_state().ego_carte.x,
        ego_state_manager_.get_ego_state().ego_carte.y);
    return false;
  } else {
    ego_state_manager_.get_mutable_ego_state().ego_frenet = ego_frenet;
  }

  frenet_enu_x_ = vx;
  frenet_enu_y_ = vy;

  // update start point in baseline
  Point2D cart, fren;
  CartesianState cs0;
  FrenetState start_state;
  cart.x = baseline_ego_state.planning_init_point.path_point.x;
  cart.y = baseline_ego_state.planning_init_point.path_point.y;
  cs0.x = baseline_ego_state.planning_init_point.path_point.x;
  cs0.y = baseline_ego_state.planning_init_point.path_point.y;
  cs0.yaw = baseline_ego_state.planning_init_point.path_point.theta;
  cs0.speed = std::max(0.0, baseline_ego_state.planning_init_point.v);
  cs0.acceleration = baseline_ego_state.planning_init_point.a;
  cs0.curvature = baseline_ego_state.planning_init_point.path_point.kappa;
  // if (frenet_coord_->CartCoord2FrenetCoord(cart, fren) ==
  //     TRANSFORM_FAILED) {
  //   MSD_LOG(INFO, "debug2 transformed failed cart %f %f",cart.x, cart.y);
  //   return false;
  // }
  if (frenet_coord_->CartState2FrenetState(cs0, start_state) ==
      TRANSFORM_FAILED) {
    construct_failed_reason_ =
        "[MAP_ERROR] baseline transformed failed FrenetState";
    // MSD_LOG(INFO, " [MAP_ERROR] baseline[%d] transformed FrenetState failed
    // cart %f %f", lane_id_, cs0.x, cs0.y);
    return false;
  }
  baseline_ego_state.planning_init_point.path_point.s = start_state.s;
  baseline_ego_state.planning_start_state.s = start_state.s;
  baseline_ego_state.planning_start_state.r = start_state.r;
  baseline_ego_state.planning_start_state.ds = std::max(0.0, start_state.ds);
  baseline_ego_state.planning_start_state.dr_ds = start_state.dr_ds;
  baseline_ego_state.planning_start_state.dr = start_state.dr;
  baseline_ego_state.planning_start_state.dds = start_state.dds;
  baseline_ego_state.planning_start_state.ddr_dsds = start_state.ddr_dsds;
  baseline_ego_state.planning_start_state.ddr = start_state.ddr;
  baseline_ego_state.mpc_vehicle_state[0] =
      baseline_ego_state.planning_init_point.v;
  baseline_ego_state.mpc_vehicle_state[1] =
      baseline_ego_state.planning_init_point.a;
  baseline_ego_state.mpc_vehicle_state[2] =
      baseline_ego_state.planning_init_point.path_point.theta -
      frenet_coord_->GetRefCurveHeading(fren.x);

  cart.x = baseline_ego_state.real_init_point.path_point.x;
  cart.y = baseline_ego_state.real_init_point.path_point.y;
  if (frenet_coord_->CartCoord2FrenetCoord(cart, fren) == TRANSFORM_FAILED) {
    construct_failed_reason_ =
        "[MAP_ERROR] baseline transformed failed real_init_point";
    // MSD_LOG(INFO, "[MAP_ERROR] baseline[%d] transformed failed
    // real_init_point cart %f %f", lane_id_, cart.x, cart.y);
    return false;
  }

  baseline_ego_state.real_init_point.path_point.s = fren.x;
  Point2D front_center_cart, front_center_fren;
  front_center_cart.x =
      cart.x +
      std::cos(baseline_ego_state.planning_init_point.path_point.theta) *
          ConfigurationContext::Instance()->get_vehicle_param().length / 2.0;
  front_center_cart.y =
      cart.y +
      std::sin(baseline_ego_state.planning_init_point.path_point.theta) *
          ConfigurationContext::Instance()->get_vehicle_param().length / 2.0;

  if (frenet_coord_->CartCoord2FrenetCoord(
          front_center_cart, front_center_fren) == TRANSFORM_FAILED) {
    construct_failed_reason_ =
        "[MAP_ERROR] baseline transformed failed front_center_cart";
    // MSD_LOG(INFO, "[MAP_ERROR] baseline[%d] transformed failed
    // front_center_cart %f %f", lane_id_, front_center_cart.x,
    // front_center_cart.y);
    return false;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// CJ:用于ACC的参考线生成函数，和construct_reference_line()互斥，不可同时调用
bool BaseLineInfo::construct_reference_line_cjacc(
    const std::shared_ptr<WorldModel> &world_model) {
  MSD_LOG(INFO, "Entering %s:%d ", __FUNCTION__, __LINE__);
  static std::vector<double> vx, vy;
  vx.clear();
  vy.clear();
  const auto &map_info_manager = world_model->get_map_info_manager();
  const auto &cart_ego_state_manager =
      world_model->get_cart_ego_state_manager();

  if (lane_id_ == 0) {
    lane_type_ = static_cast<uint8_t>(
        map_info_manager.get_map_info().current_lane_type());
  } else if (lane_id_ == -1) {
    lane_type_ =
        static_cast<uint8_t>(map_info_manager.get_map_info().left_lane_type());
  } else if (lane_id_ == 1) {
    lane_type_ =
        static_cast<uint8_t>(map_info_manager.get_map_info().right_lane_type());
  } else {
    MSD_LOG(WARN, "ERROR_CJ3 %s: invalid baseline id!", __FUNCTION__);
    return false;
  }

  const auto &map_poi_info = world_model->get_map_info().map_pois_data();
  const auto &map_info = world_model->get_map_info();
  static double sa_fil = 0;
  static double yr_fil = 0;
  const float default_lane_width = 3.75;

  MSD_LOG(INFO, "[baseline] id %d type %d", lane_id_, int(lane_type_));

  // get all cartesian related info

  auto &baseline_ego_state = ego_state_manager_.get_mutable_ego_state();
  const auto &cart_ego_state = cart_ego_state_manager.get_cart_ego_state();

  baseline_ego_state.ego_pose = cart_ego_state.ego_pose;
  baseline_ego_state.ego_pose_raw = cart_ego_state.ego_pose_raw;
  baseline_ego_state.ego_enu = cart_ego_state.ego_enu;
  baseline_ego_state.ego_acc = cart_ego_state.ego_acc;
  baseline_ego_state.ego_carte = cart_ego_state.ego_carte;
  baseline_ego_state.ego_vel = cart_ego_state.ego_vel;
  baseline_ego_state.ego_steer_angle = cart_ego_state.ego_steer_angle;
  baseline_ego_state.flag_is_replan = cart_ego_state.flag_is_replan;
  baseline_ego_state.ego_blinker = cart_ego_state.ego_blinker;
  baseline_ego_state.real_init_point = cart_ego_state.real_init_point;
  baseline_ego_state.planning_init_point = cart_ego_state.planning_init_point;
  baseline_ego_state.stitch_trajectory = cart_ego_state.stitch_trajectory;

  ego_state_manager_.set_ego_carte(
      convert_pose2point(ego_state_manager_.get_ego_state().ego_pose));

  // CJ:计算曲率
  const auto &ego_vel = ego_state_manager_.get_ego_state().ego_vel;
  double sa = ego_state_manager_.get_ego_state().ego_steer_angle /
              ConfigurationContext::Instance()->get_vehicle_param().steer_ratio;
  const double irregular_yawrate_thr = 0.75;
  const double yawrate_lim = 0.5;
  double yawrate =
      std::fmin(std::fmax(world_model->get_yawrate(), -yawrate_lim),
                yawrate_lim); // rad/s
  double sa_fil_k = 0.15;
  double yr_fil_k = 0.15;
  sa_fil = (1 - sa_fil_k) * sa_fil + sa_fil_k * sa;
  yr_fil = (1 - yr_fil_k) * yr_fil + yr_fil_k * yawrate;
  MSD_LOG(INFO, "DEBUG_CJ11:yawrate:%3.6f,yr_fil:%3.6f,sa:%f,sa_fil:%f",
          world_model->get_yawrate(), yr_fil, sa, sa_fil);
  double driver_curv = 0;
  double curv_k_lowspd = 0;
  double curv_knee_pt1_mps = 3;
  double curv_knee_pt2_mps = 6;
  if (ego_vel < curv_knee_pt1_mps) {
    curv_k_lowspd = 1;
  } else if (ego_vel < curv_knee_pt2_mps) {
    curv_k_lowspd =
        (ego_vel - curv_knee_pt1_mps) / (curv_knee_pt2_mps - curv_knee_pt1_mps);
  } else {
    curv_k_lowspd = 0;
  }
  if (std::fabs(world_model->get_yawrate()) > irregular_yawrate_thr) {
    curv_k_lowspd = 1;
  }

  double wb = ConfigurationContext::Instance()->get_vehicle_param().wheel_base;
  double curv_low_spd = sa_fil / wb;
  double curv_high_spd = yr_fil / std::fmax(ego_vel, 0.1);
  driver_curv =
      curv_k_lowspd * curv_low_spd + (1 - curv_k_lowspd) * curv_high_spd;

  // CJ:这里是有符号的，注意一下
  double driver_radius = 10000;
  if (driver_curv >= 0) {
    driver_radius = clip(1 / std::fmax(driver_curv, 0.0001), 10000.0, -10000.0);
  } else {
    driver_radius =
        clip(1 / std::fmin(driver_curv, -0.0001), 10000.0, -10000.0);
  }

  // CJ:预测驾驶员的行驶轨迹点
  ReferenceLinePointDerived raw_refline_point_acc;
  // CJ:最终构建的参考线点列
  static std::vector<ReferenceLinePointDerived> enu_refline_points;
  enu_refline_points.clear();

  double s_step = 2;
  double s_start = -20;
  double s_end = 150;

  bool has_lane_flag = false;
  auto current_refline_points =
      world_model->get_map_info().current_refline_points();
  vector<double> line_s, line_left_l, line_right_l, line_x, line_y;

  for (int i = 0; i < current_refline_points.size(); ++i) {
    MSD_LOG(INFO, "Refline_DEBUG_0:p[%d]:[%2.2f,%2.2f]", i,
            current_refline_points[i].car_point.x,
            current_refline_points[i].car_point.y);
  }

  // CJ:建立左右道线关于s的曲线line_left_l,line_right_l,建立地图参考线点列line_x,line_y
  // CJ:extended refline to min length
  std::vector<ReferenceLinePointDerived> expanded_refline_points;
  car_refline_points_.reserve(current_refline_points.size());
  leftborder_points_.reserve(current_refline_points.size());
  rightborder_points_.reserve(current_refline_points.size());
  car_refline_points_.clear();
  leftborder_points_.clear();
  rightborder_points_.clear();
  double R = driver_radius;
  MSD_LOG(INFO, "Driving_Radius:%f", R);

  if (current_refline_points.size() > 40) {
    MSD_LOG(INFO, "RefLine_DEBUG_1:cjacc Has_Lane");
    has_lane_flag = true;
    double s = 0;
    auto last_point = current_refline_points.front();

    for (int i = 0; i < current_refline_points.size(); ++i) {
      auto point = current_refline_points[i];
      s += std::hypot(point.car_point.x - last_point.car_point.x,
                      point.car_point.y - last_point.car_point.y);
      last_point = current_refline_points[i];
      line_s.push_back(s);

      // clip distance_to_lane_border
      if (point.distance_to_left_lane_border > 10) {
        if (expanded_refline_points.size() > 0) {
          point.distance_to_left_lane_border =
              expanded_refline_points.back().distance_to_left_lane_border;
        } else {
          point.distance_to_left_lane_border = default_lane_width / 2;
        }
      } else if (point.distance_to_left_lane_border < 1.1) {
        if (expanded_refline_points.size() > 0) {
          point.distance_to_left_lane_border =
              expanded_refline_points.back().distance_to_left_lane_border;
        } else {
          point.distance_to_left_lane_border = 1.1;
        }
      }
      if (point.distance_to_right_lane_border > 10) {
        if (expanded_refline_points.size() > 0) {
          point.distance_to_right_lane_border =
              expanded_refline_points.back().distance_to_right_lane_border;
        } else {
          point.distance_to_right_lane_border = default_lane_width / 2;
        }
      } else if (point.distance_to_right_lane_border < 1.1) {
        if (expanded_refline_points.size() > 0) {
          point.distance_to_right_lane_border =
              expanded_refline_points.back().distance_to_right_lane_border;
        } else {
          point.distance_to_right_lane_border = 1.1;
        }
      }

      expanded_refline_points.push_back(point);

      line_left_l.push_back(point.distance_to_left_lane_border);
      line_right_l.push_back(point.distance_to_right_lane_border);
      line_x.push_back(current_refline_points[i].car_point.x);
      line_y.push_back(current_refline_points[i].car_point.y);
    }
    for (auto point : expanded_refline_points) {
      MSD_LOG(INFO,
              "RefLine_DEBUG_1:x:%3.1f,y:%3.1f,left_b:%3.1f,right_b:%3.1f",
              point.car_point.x, point.car_point.y,
              point.distance_to_left_lane_border,
              point.distance_to_right_lane_border);
    }
  } else {
    MSD_LOG(INFO, "RefLine_DEBUG_1:cjacc Has_No_Lane");
    has_lane_flag = false;
  }

  double min_length = 150;
  double current_refline_length = 0.0;
  if (line_s.size() > 0) {
    current_refline_length = line_s.back();
  }
  min_length = std::max(std::max(80.0, ego_vel * 8.0),
                        std::min(150.0, current_refline_length * 5.0));
  s_end = std::max(s_end, min_length);
  double ref_kappa = 0.00001;
  double ref_r = 1 / ref_kappa;
  // CJ:ref_traj记录参考线点列用于拟合
  std::vector<Point2D> ref_traj{};
  // CJ:参考线拟合参数，三次多项式
  std::vector<double> poly_coef_ref{};
  ref_traj.clear();
  poly_coef_ref.clear();
  Eigen::Vector3d car_point, enu_point;
  auto &enu2car = world_model->get_cart_ego_state_manager().get_enu2car();
  auto &car2enu = world_model->get_cart_ego_state_manager().get_car2enu();
  // CJ:当参考线过短时进行延长,和ref_traj的区别在于点的类型

  double refline_length = 0;

  if (has_lane_flag == true) {
    refline_length = line_s.back();

    for (auto point : current_refline_points) {
      Point2D pt;
      pt.x = point.car_point.x;
      pt.y = point.car_point.y;
      ref_traj.push_back(pt);
    }

    if (!Polyfit(ref_traj, 3, poly_coef_ref) || poly_coef_ref.size() != 4) {
      for (int i = 0; i < 4; ++i) {
        poly_coef_ref[i] = 0;
      }
    }
    // extent refline to ego back
    if (ref_traj.front().x > -20) {
      MSD_LOG(INFO, "RefLine_DEBUG_2:HEAD TOO SHORT");
      vector<double> line_s_0, line_left_l_0, line_right_l_0, line_x_0,
          line_y_0;
      std::vector<ReferenceLinePointDerived> expanded_refline_points_0;
      Point2D pred_car, last_pred_car;
      last_pred_car.x = -21;
      last_pred_car.y = calc_line_y(poly_coef_ref, last_pred_car.x);
      double s = 0;
      double len = refline_length;
      for (; last_pred_car.x < ref_traj.front().x;) {
        ReferenceLinePointDerived pred_ref_point;

        pred_car.x = last_pred_car.x + 1;
        pred_car.y = calc_line_y(poly_coef_ref, pred_car.x);
        line_x_0.push_back(pred_car.x);
        line_y_0.push_back(pred_car.y);
        car_point.x() = pred_car.x;
        car_point.y() = pred_car.y;
        car_point.z() = 0;
        enu_point = car2enu * car_point;
        pred_ref_point.enu_point.x = enu_point.x();
        pred_ref_point.enu_point.y = enu_point.y();
        pred_ref_point.enu_point.z = enu_point.z();
        pred_ref_point.car_point.x = car_point.x();
        pred_ref_point.car_point.y = car_point.y();
        pred_ref_point.car_point.z = car_point.z();

        double first_dev = calc_line_dev(poly_coef_ref, pred_car.x, 1);
        double second_dev = calc_line_dev(poly_coef_ref, pred_car.x, 2);
        pred_ref_point.is_experience_data = false;
        pred_ref_point.on_route = true;
        pred_ref_point.longitudinal_slope = 0;
        pred_ref_point.horizontal_slope = 0;
        pred_ref_point.curvature =
            second_dev / std::pow(std::sqrt(1 + std::pow(first_dev, 2)), 3);
        pred_ref_point.yaw = std::atan(first_dev);
        pred_ref_point.distance_to_left_road_border =
            expanded_refline_points.front().distance_to_left_road_border;
        pred_ref_point.distance_to_right_road_border =
            expanded_refline_points.front().distance_to_right_road_border;
        pred_ref_point.distance_to_left_lane_border =
            expanded_refline_points.front().distance_to_left_lane_border;
        pred_ref_point.distance_to_right_lane_border =
            expanded_refline_points.front().distance_to_right_lane_border;
        pred_ref_point.distance_to_left_obstacle = 20;
        pred_ref_point.distance_to_right_obstacle = -20;
        pred_ref_point.lane_width = std::fabs(
            expanded_refline_points.front().distance_to_left_lane_border +
            expanded_refline_points.front().distance_to_right_lane_border);
        pred_ref_point.max_velocity = 120 / 3.6;
        pred_ref_point.track_id = 0; // CJ:暂时赋值
        pred_ref_point.left_road_border_type.value.value =
            maf_worldmodel::LaneBoundaryForm::UNKNOWN;
        pred_ref_point.right_road_border_type.value.value =
            maf_worldmodel::LaneBoundaryForm::UNKNOWN;

        s += std::hypot(pred_car.x - last_pred_car.x,
                        pred_car.y - last_pred_car.y);
        len += std::hypot(pred_car.x - last_pred_car.x,
                          pred_car.y - last_pred_car.y);

        expanded_refline_points_0.push_back(pred_ref_point);

        last_pred_car = pred_car;

        line_s_0.push_back(s);
        line_left_l_0.push_back(pred_ref_point.distance_to_left_lane_border);
        line_right_l_0.push_back(pred_ref_point.distance_to_right_lane_border);
      }
      refline_length = len;
      double line_s_0_len = line_s_0.back();
      for (int i = 0; i < line_s.size(); ++i) {
        line_s_0.push_back(line_s[i] + line_s_0_len);
        line_left_l_0.push_back(line_left_l[i]);
        line_right_l_0.push_back(line_right_l[i]);
        line_x_0.push_back(line_x[i]);
        line_y_0.push_back(line_y[i]);
      }
      line_s_0.swap(line_s);
      line_left_l_0.swap(line_left_l);
      line_right_l_0.swap(line_right_l);
      line_x_0.swap(line_x);
      line_y_0.swap(line_y);
      for (int i = 0; i < expanded_refline_points.size(); ++i) {
        expanded_refline_points_0.push_back(expanded_refline_points[i]);
      }
      expanded_refline_points_0.swap(expanded_refline_points);
      for (auto point : expanded_refline_points) {
        MSD_LOG(INFO,
                "RefLine_DEBUG_2:x:%3.1f,y:%3.1f,left_b:%3.1f,right_b:%3.1f",
                point.car_point.x, point.car_point.y,
                point.distance_to_left_lane_border,
                point.distance_to_right_lane_border);
      }
    } else {
      MSD_LOG(INFO, "RefLine_DEBUG_2:HEAD OK");
    }

    // CJ:when map refline is too short, predict to min_length m
    if (refline_length < min_length - s_start) {
      MSD_LOG(INFO, "RefLine_DEBUG_3:TAIL TOO SHORT!");
      Point2D pred_car, last_pred_car;
      last_pred_car.x = ref_traj.back().x;
      last_pred_car.y = calc_line_y(poly_coef_ref, last_pred_car.x);
      double s = refline_length;
      for (; s <= min_length - s_start;) { // CJ:prediction point pred_ref_point
        ReferenceLinePointDerived pred_ref_point;

        pred_car.x = last_pred_car.x + 1;
        pred_car.y = calc_line_y(poly_coef_ref, pred_car.x);
        line_x.push_back(pred_car.x);
        line_y.push_back(pred_car.y);
        car_point.x() = pred_car.x;
        car_point.y() = pred_car.y;
        car_point.z() = 0;
        enu_point = car2enu * car_point;
        pred_ref_point.enu_point.x = enu_point.x();
        pred_ref_point.enu_point.y = enu_point.y();
        pred_ref_point.enu_point.z = enu_point.z();
        pred_ref_point.car_point.x = car_point.x();
        pred_ref_point.car_point.y = car_point.y();
        pred_ref_point.car_point.z = car_point.z();

        double first_dev = calc_line_dev(poly_coef_ref, pred_car.x, 1);
        double second_dev = calc_line_dev(poly_coef_ref, pred_car.x, 2);
        pred_ref_point.is_experience_data = false;
        pred_ref_point.on_route = true;
        pred_ref_point.longitudinal_slope = 0;
        pred_ref_point.horizontal_slope = 0;
        pred_ref_point.curvature =
            second_dev / std::pow(std::sqrt(1 + std::pow(first_dev, 2)), 3);
        pred_ref_point.yaw = std::atan(first_dev);
        pred_ref_point.distance_to_left_road_border =
            expanded_refline_points.back().distance_to_left_road_border;
        pred_ref_point.distance_to_right_road_border =
            expanded_refline_points.back().distance_to_right_road_border;
        pred_ref_point.distance_to_left_lane_border =
            expanded_refline_points.back().distance_to_left_lane_border;
        pred_ref_point.distance_to_right_lane_border =
            expanded_refline_points.back().distance_to_right_lane_border;
        pred_ref_point.distance_to_left_obstacle = 20;
        pred_ref_point.distance_to_right_obstacle = -20;
        pred_ref_point.lane_width = std::fabs(
            expanded_refline_points.back().distance_to_left_lane_border +
            expanded_refline_points.back().distance_to_right_lane_border);
        pred_ref_point.max_velocity = 120 / 3.6;
        pred_ref_point.track_id = 0; // CJ:暂时赋值
        pred_ref_point.left_road_border_type.value.value =
            maf_worldmodel::LaneBoundaryForm::UNKNOWN;
        pred_ref_point.right_road_border_type.value.value =
            maf_worldmodel::LaneBoundaryForm::UNKNOWN;

        s += std::hypot(pred_car.x - last_pred_car.x,
                        pred_car.y - last_pred_car.y);

        expanded_refline_points.push_back(pred_ref_point);

        last_pred_car = pred_car;

        line_s.push_back(s);
        line_left_l.push_back(pred_ref_point.distance_to_left_lane_border);
        line_right_l.push_back(pred_ref_point.distance_to_right_lane_border);
      }
      refline_length = s;
      for (auto point : expanded_refline_points) {
        MSD_LOG(INFO,
                "RefLine_DEBUG_3:x:%3.1f,y:%3.1f,left_b:%3.1f,right_b:%3.1f",
                point.car_point.x, point.car_point.y,
                point.distance_to_left_lane_border,
                point.distance_to_right_lane_border);
      }
    }
  }

  std::shared_ptr<FrenetCoordinateSystem> ref_frenet_coord;
  if (has_lane_flag == true) {
    ref_frenet_coord.reset(new FrenetCoordinateSystem(
        line_x, line_y, frenet_parameters_, 0.0, s_end));
  }
  // CJ:build driving path point
  int cross_pt = 0;
  bool cross_flag = false;

  for (double s = s_start; s <= min_length; s += s_step) {
    double theta = atan(s / driver_radius);

    double left_y = 20;
    double right_y = -20;

    raw_refline_point_acc.car_point.x =
        R * sin(theta) -
        ConfigurationContext::Instance()->get_vehicle_param().length / 2;
    raw_refline_point_acc.car_point.y = R * (1 - cos(theta));
    raw_refline_point_acc.car_point.z = 0.0;

    auto &car2enu = world_model->get_cart_ego_state_manager().get_car2enu();
    Eigen::Vector3d car_point, enu_point;
    car_point.x() = raw_refline_point_acc.car_point.x;
    car_point.y() = raw_refline_point_acc.car_point.y;
    car_point.z() = 0.0;
    enu_point = car2enu * car_point;
    if (std::fabs(enu_point.x()) < 0.01 && std::fabs(enu_point.y()) < 0.01) {
      MSD_LOG(WARN, "ERROR_CJ3 %s: invalid enu_point!", __FUNCTION__);
      return false;
    }
    raw_refline_point_acc.enu_point.x = enu_point.x();
    raw_refline_point_acc.enu_point.y = enu_point.y();
    raw_refline_point_acc.enu_point.z = enu_point.z();
    raw_refline_point_acc.distance_to_left_road_border = 20;
    raw_refline_point_acc.distance_to_right_road_border = -20;
    raw_refline_point_acc.distance_to_left_lane_border = 20;
    raw_refline_point_acc.distance_to_right_lane_border = -20;
    raw_refline_point_acc.distance_to_left_obstacle = 20;
    raw_refline_point_acc.distance_to_right_obstacle = -20;
    raw_refline_point_acc.curvature = driver_curv;
    raw_refline_point_acc.yaw =
        ego_state_manager_.get_ego_state().ego_pose.theta + theta;
    // CJ:这里还没有考虑道线yaw导致的道宽偏差
    raw_refline_point_acc.lane_width = std::fabs(left_y - right_y);
    // CJ:无地图时只能依靠TSR信息
    raw_refline_point_acc.max_velocity = 120 / 3.6;
    raw_refline_point_acc.track_id = 0; // CJ:暂时赋值
    raw_refline_point_acc.left_road_border_type.value.value =
        maf_worldmodel::LaneBoundaryForm::UNKNOWN;
    raw_refline_point_acc.right_road_border_type.value.value =
        maf_worldmodel::LaneBoundaryForm::UNKNOWN;
    raw_refline_point_acc.is_experience_data = false;
    raw_refline_point_acc.on_route = true;
    raw_refline_point_acc.longitudinal_slope = 0;
    raw_refline_point_acc.horizontal_slope = 0;

    MSD_LOG(INFO, "DEBUG_CJ5: ori_ref_x: %3.1f,ori_ref_y: %3.1f", car_point.x(),
            car_point.y());

    Point2D raw_frenet, veh_head_frenet;
    // CJ:lane cross cut
    // CJ:if driving corridor and lane border crossed, raw_refline_point_acc
    // would be changed
    const double CrossTurn_R = 20;
    if (has_lane_flag == true && std::fabs(R) > CrossTurn_R) {
      double refline_c1 = 0;
      int ego_num = BaseLineInfo::find_num_by_x(expanded_refline_points, 0);
      ReferenceLinePointDerived first_p =
          expanded_refline_points[std::fmax(ego_num - 3, 0)];
      ReferenceLinePointDerived last_p = expanded_refline_points[std::fmin(
          ego_num + 3, (int)expanded_refline_points.size() - 1)];
      refline_c1 = (last_p.car_point.y - first_p.car_point.y) /
                   std::fmax(last_p.car_point.x - first_p.car_point.x, 1);
      if (std::fabs(refline_c1) < 0.5) {
        if (ref_frenet_coord->CartCoord2FrenetCoord(
                {car_point.x(), car_point.y()}, raw_frenet) ==
                TRANSFORM_SUCCESS &&
            ref_frenet_coord->CartCoord2FrenetCoord({0, 0}, veh_head_frenet) ==
                TRANSFORM_SUCCESS) {

          left_y = interpolation(raw_frenet.x, line_s, line_left_l);
          right_y = -interpolation(raw_frenet.x, line_s, line_right_l);

          MSD_LOG(INFO,
                  "DEBUG_CJ5: ref_x:%3.1f,ref_y: %3.1f,left_y: %3.1f,right_y: "
                  "%3.1f",
                  raw_frenet.x, raw_frenet.y, left_y, right_y);
          MSD_LOG(INFO,
                  "DEBUG_CJ5: veh_x:%3.1f,veh_y: %3.1f,left_y: %3.1f,right_y: "
                  "%3.1f",
                  veh_head_frenet.x, veh_head_frenet.y, left_y, right_y);
          double half_width =
              ConfigurationContext::Instance()->get_vehicle_param().width / 2;
          double half_length =
              ConfigurationContext::Instance()->get_vehicle_param().length / 2;
          double lane_width =
              expanded_refline_points[0].distance_to_left_lane_border +
              expanded_refline_points[0].distance_to_right_lane_border;
          raw_refline_point_acc.distance_to_left_lane_border =
              left_y - raw_frenet.y;
          raw_refline_point_acc.distance_to_right_lane_border =
              raw_frenet.y - right_y;

          if (car_point.x() > half_length && s - s_start < refline_length &&
              s > half_length) {
            if ((raw_frenet.y > left_y - half_width &&
                 veh_head_frenet.y < lane_width / 2 - half_width) ||
                (raw_frenet.y < right_y + half_width &&
                 veh_head_frenet.y > -lane_width / 2 + half_width)) {
              if (raw_frenet.y > left_y - half_width) {
                MSD_LOG(INFO, "DEBUG_CJ5: Left REF_LINE_CROSS!Dist:%2.2f",
                        raw_frenet.x - veh_head_frenet.x);
              } else {
                MSD_LOG(INFO, "DEBUG_CJ5: Right REF_LINE_CROSS!Dist:%2.2f",
                        raw_frenet.x - veh_head_frenet.x);
              }

              raw_refline_point_acc = BaseLineInfo::find_point_by_x(
                  expanded_refline_points, car_point.x());

              if (raw_frenet.y > left_y - half_width) {
                car_point.y() =
                    raw_refline_point_acc.car_point.y +
                    raw_refline_point_acc.distance_to_left_lane_border -
                    half_width;
                raw_refline_point_acc.distance_to_right_lane_border +=
                    raw_refline_point_acc.distance_to_left_lane_border -
                    half_width;
                raw_refline_point_acc.distance_to_left_lane_border = half_width;
              } else {
                car_point.y() =
                    raw_refline_point_acc.car_point.y -
                    raw_refline_point_acc.distance_to_right_lane_border +
                    half_width;
                raw_refline_point_acc.distance_to_left_lane_border +=
                    raw_refline_point_acc.distance_to_right_lane_border -
                    half_width;
                raw_refline_point_acc.distance_to_right_lane_border =
                    half_width;
              }
              enu_point = car2enu * car_point;
              raw_refline_point_acc.car_point.x = car_point.x();
              raw_refline_point_acc.car_point.y = car_point.y();
              raw_refline_point_acc.car_point.z = car_point.z();
              raw_refline_point_acc.enu_point.x = enu_point.x();
              raw_refline_point_acc.enu_point.y = enu_point.y();
              raw_refline_point_acc.enu_point.z = enu_point.z();
              cross_flag = true;
            }

            if (veh_head_frenet.y > lane_width / 2 - half_width &&
                std::atan(poly_coef_ref[1]) < -0.02 &&
                raw_frenet.y >
                    left_y + lane_width / 2 - half_width) // CJ:left lane change
            {
              MSD_LOG(INFO, "DEBUG_CJ5: REF_LINE_CROSS!LLC! Dist:%2.2f",
                      raw_frenet.x - veh_head_frenet.x);
              raw_refline_point_acc = BaseLineInfo::find_point_by_x(
                  expanded_refline_points, car_point.x());
              car_point.y() =
                  raw_refline_point_acc.car_point.y +
                  raw_refline_point_acc.distance_to_left_lane_border +
                  lane_width / 2 - half_width;
              raw_refline_point_acc.distance_to_left_lane_border =
                  lane_width / 2 + half_width;
              raw_refline_point_acc.distance_to_right_lane_border =
                  lane_width / 2 - half_width;

              enu_point = car2enu * car_point;
              raw_refline_point_acc.car_point.x = car_point.x();
              raw_refline_point_acc.car_point.y = car_point.y();
              raw_refline_point_acc.car_point.z = car_point.z();
              raw_refline_point_acc.enu_point.x = enu_point.x();
              raw_refline_point_acc.enu_point.y = enu_point.y();
              raw_refline_point_acc.enu_point.z = enu_point.z();
              cross_flag = true;
            } else if (veh_head_frenet.y < -lane_width / 2 + half_width &&
                       std::atan(poly_coef_ref[1]) > 0.02 &&
                       raw_frenet.y < right_y - lane_width / 2 +
                                          half_width) // CJ:right lane change
            {
              MSD_LOG(INFO, "DEBUG_CJ5: REF_LINE_CROSS!RLC! Dist:%2.2f",
                      raw_frenet.x - veh_head_frenet.x);
              raw_refline_point_acc = BaseLineInfo::find_point_by_x(
                  expanded_refline_points, car_point.x());
              car_point.y() =
                  raw_refline_point_acc.car_point.y -
                  raw_refline_point_acc.distance_to_right_lane_border -
                  lane_width / 2 + half_width;
              raw_refline_point_acc.distance_to_left_lane_border =
                  lane_width / 2 - half_width;
              raw_refline_point_acc.distance_to_right_lane_border =
                  lane_width / 2 + half_width;

              enu_point = car2enu * car_point;
              raw_refline_point_acc.car_point.x = car_point.x();
              raw_refline_point_acc.car_point.y = car_point.y();
              raw_refline_point_acc.car_point.z = car_point.z();
              raw_refline_point_acc.enu_point.x = enu_point.x();
              raw_refline_point_acc.enu_point.y = enu_point.y();
              raw_refline_point_acc.enu_point.z = enu_point.z();
              cross_flag = true;
            }
          }
          if (cross_flag == true && s - s_start > refline_length) {
            MSD_LOG(INFO, "REF_LINE_END1 at:%f",
                    refline_length - veh_head_frenet.x);
            break;
          }
        } else {
          MSD_LOG(INFO, "DEBUG_CJ5 Frenet Failed at s = %f,len=%f", s - s_start,
                  refline_length);
          if (s - s_start > refline_length / 2) {
            MSD_LOG(INFO, "REF_LINE_END2 at:%f",
                    refline_length - veh_head_frenet.x);
            break;
          }
        }
      }
    }
    MSD_LOG(INFO, "DEBUG_CJ5: fnl_ref_x: %3.1f,fnl_ref_y: %3.1f", car_point.x(),
            car_point.y());
    car_refline_points_.push_back(car_point);
    enu_refline_points.push_back(raw_refline_point_acc);
    if (cross_flag == true && cross_pt == 0) {
      cross_pt = enu_refline_points.size();
    }
  }

  //  MSD_LOG(INFO, "DEBUG_CJ Time:%3.1f,ref_line_length: %d: ",
  //  MTIME()->timestamp().sec(),enu_refline_points.size());

  if (enu_refline_points.size() < 3) {
    MSD_LOG(WARN, "ERROR_CJ3 %s: construct refline failed P1!", __FUNCTION__);
    return false;
  }

  // if (cross_pt > 0) {
  //   int start_p =
  //       std::fmax(cross_pt - 20,
  //                 BaseLineInfo::find_num_by_x(
  //                     enu_refline_points, 0.0)); // CJ:from ego vehicle
  //                     position
  //   int end_p = std::fmin(cross_pt + 20, (int)enu_refline_points.size() - 1);
  //   if (start_p != end_p) {
  //     ReferenceLinePointDerived start_point, end_point;
  //     start_point = enu_refline_points[start_p];
  //     end_point = enu_refline_points[end_p];
  //     vector<double> smooth_coef{0, 0, 0, 0};
  //     double y_s = start_point.car_point.y;
  //     double y_e = end_point.car_point.y;
  //     double x_s = start_point.car_point.x;
  //     double x_e = end_point.car_point.x;
  //     double yaw_s = 0; // start_point.yaw;
  //     double yaw_e = 0; // end_point.yaw;

  //     smooth_coef[3] =
  //         (std::tan(yaw_s) + std::tan(yaw_e) - 2 * (y_e - y_s) / (x_e - x_s))
  //         / std::pow(x_e - x_s, 2);
  //     smooth_coef[2] =
  //         (std::tan(yaw_s) - (y_e - y_s) / (x_e - x_s) -
  //          smooth_coef[3] * (2 * x_s * x_s - x_e * x_s - x_e * x_e)) /
  //         (x_s - x_e);
  //     smooth_coef[1] = std::tan(yaw_s) - 2 * smooth_coef[2] * x_s -
  //                      3 * smooth_coef[3] * x_s * x_s;
  //     smooth_coef[0] = y_s - smooth_coef[1] * x_s - smooth_coef[2] * x_s *
  //     x_s -
  //                      smooth_coef[3] * std::pow(x_s, 3);
  //     smooth_coef[0] = std::fmin(std::fmax(smooth_coef[0],-10),10);
  //     smooth_coef[1] = std::fmin(std::fmax(smooth_coef[1],-2),2);
  //     smooth_coef[2] = std::fmin(std::fmax(smooth_coef[2],-0.1),0.1);
  //     smooth_coef[3] = std::fmin(std::fmax(smooth_coef[3],-0.01),0.01);

  //     for (int i = start_p; i <= end_p; ++i) {
  //       auto &car2enu =
  //       world_model->get_cart_ego_state_manager().get_car2enu();
  //       Eigen::Vector3d car_point, enu_point;
  //       car_point.x() = enu_refline_points[i].car_point.x;
  //       car_point.y() = calc_line_y(smooth_coef, car_point.x());
  //       car_point.z() = 0;
  //       enu_point = car2enu * car_point;
  //       MSD_LOG(INFO, "Smooth DEBUG1:before_x:%2.2f,before_y:%2.2f",
  //               enu_refline_points[i].car_point.x,
  //               enu_refline_points[i].car_point.y);
  //       enu_refline_points[i].distance_to_left_lane_border -=
  //           car_point.y() - enu_refline_points[i].car_point.y;
  //       enu_refline_points[i].distance_to_right_lane_border +=
  //           car_point.y() - enu_refline_points[i].car_point.y;
  //       enu_refline_points[i].car_point.x = car_point.x();
  //       enu_refline_points[i].car_point.y = car_point.y();
  //       enu_refline_points[i].car_point.z = car_point.z();
  //       enu_refline_points[i].enu_point.x = enu_point.x();
  //       enu_refline_points[i].enu_point.y = enu_point.y();
  //       enu_refline_points[i].enu_point.z = enu_point.z();
  //       enu_refline_points[i].yaw =
  //           std::atan(calc_line_dev(smooth_coef, car_point.x(), 1));
  //       MSD_LOG(INFO, "Smooth DEBUG1:end_x:%2.2f,end_y:%2.2f",
  //               enu_refline_points[i].car_point.x,
  //               enu_refline_points[i].car_point.y);
  //       MSD_LOG(INFO, "Smooth DEBUG2:num:%d,yaw:%2.5f", i,
  //               enu_refline_points[i].yaw);
  //     }
  //   }
  //   MSD_LOG(INFO, "Smooth DEBUG3:start:%d,end:%d", start_p, end_p);
  // }

  reference_line_.Clear();

  static std::vector<PathPoint> path_points;
  path_points.reserve(enu_refline_points.size());
  path_points.clear();

  auto begin_raw_point = enu_refline_points.front();
  // 获得笛卡尔下所有的相关信息，根据车道宽度,获取左右边界信息
  double s = 0;
  for (auto raw_point : enu_refline_points) {
    s += std::hypot(raw_point.enu_point.x - begin_raw_point.enu_point.x,
                    raw_point.enu_point.y - begin_raw_point.enu_point.y);

    PathPoint path_point;
    path_point.x = raw_point.enu_point.x;
    path_point.y = raw_point.enu_point.y;
    path_point.s = s;

    path_points.emplace_back(path_point);
    begin_raw_point = raw_point;
    MSD_LOG(INFO, "CJ_ACC_DEBUG,x:%3.2f,y:%3.2f", raw_point.car_point.x,
            raw_point.car_point.y);
  }

  (void)reference_line_.SetDiscretizedPath(DiscretizedPath(path_points));

  vx.reserve(path_points.size());
  vy.reserve(path_points.size());
  for (auto &p : path_points) {
    vx.emplace_back(p.x);
    vy.emplace_back(p.y);
  }

  double v_caculate_s = std::max(
      10.0 / 3.6, std::max(map_info_manager.get_map_info().v_cruise(),
                           ego_state_manager_.get_ego_state().ego_vel));

  if (nullptr == frenet_coord_) {
    frenet_coord_ = std::make_shared<FrenetCoordinateSystem>();
  }
  frenet_coord_->Update(vx, vy, frenet_parameters_, 0.0,
                        30.0 + v_caculate_s *
                                   (FLAGS_trajectory_time_length + 1 / 10.0));
  // remove debug log
  // for (double s = 0; s < frenet_coord_->GetSlength(); s += 2) {
  //   MSD_LOG(INFO, "Curv DEBUG:s:%2f,curv:%2.5f", s,
  //           frenet_coord_->GetRefCurveCurvature(s));
  // }
  if (current_refline_points.size() > 0) {
    MSD_LOG(INFO, "Refline_DEBUG1:Refline_Length:%f",
            current_refline_points.back().car_point.x);
  }
  if (expanded_refline_points.size() > 0) {
    MSD_LOG(INFO, "Refline_DEBUG1:Ext Refline_Length:%f",
            expanded_refline_points.back().car_point.x);
  }
  if (enu_refline_points.size() > 0) {
    MSD_LOG(INFO, "Refline_DEBUG1:Final Refline_Length:%f",
            enu_refline_points.back().car_point.x);
  }

  ego_state_manager_.set_frenet_coord_system(frenet_coord_);

  static std::vector<SLPoint> left_border, right_border;
  left_border.clear();
  right_border.clear();
  s = 0;
  begin_raw_point = enu_refline_points.front();

  for (auto raw_point : enu_refline_points) {
    double s1 = 0 - s_start;
    double s2 = 10 - s_start;
    double s3 = fmax(s2, ego_vel * 2.1 - s_start);
    double s4 = fmax(s3, ego_vel * 3.6 - s_start);
    double y1 = ConfigurationContext::Instance()->get_vehicle_param().width / 2;
    double y2 = 1.2;
    double y3 = ConfigurationContext::Instance()->get_vehicle_param().width / 2;
    double y4 = 1.0;
    vector<double> driving_corridor_s = {s1, s2, s3, s4};
    vector<double> driving_corridor_y = {y1, y2, y3, y4};

    s += std::hypot(raw_point.enu_point.x - begin_raw_point.enu_point.x,
                    raw_point.enu_point.y - begin_raw_point.enu_point.y);

    begin_raw_point = raw_point;
    double drive_width = 0;

    if (s < driving_corridor_s.front()) {
      drive_width = driving_corridor_y.front();
    } else if (s > driving_corridor_s.back()) {
      drive_width = driving_corridor_y.back();
    } else {
      for (int i = 0; i < driving_corridor_s.size(); ++i) {
        if (s > driving_corridor_s[i] && s < driving_corridor_s[i + 1]) {
          drive_width = planning_math::lerp(
              driving_corridor_y[i], driving_corridor_s[i],
              driving_corridor_y[i + 1], driving_corridor_s[i + 1], s);
          break;
        }
      }
    }

    SLPoint left_point, right_point;
    left_point.s = s;
    right_point.s = s;

    left_point.l = drive_width;
    right_point.l = -drive_width;

    Point2D ref_carte, ref_frenet;
    if (frenet_coord_->FrenetCoord2CartCoord({s, 0}, ref_carte) ==
        TRANSFORM_SUCCESS) {
      if (has_lane_flag == true &&
          ref_frenet_coord->CartCoord2FrenetCoord(ref_carte, ref_frenet) ==
              TRANSFORM_SUCCESS) {
        double left_y = interpolation(ref_frenet.x, line_s, line_left_l);
        double right_y = -interpolation(ref_frenet.x, line_s, line_right_l);
        if (left_y - ref_frenet.y < -7000) {
          MSD_LOG(WARN, "ERROR!");
        }
        if (left_y > right_y && left_y - ref_frenet.y > 0 &&
            right_y - ref_frenet.y < 0 // CJ:to be revised
        ) {
          left_point.l = std::fmin(drive_width, left_y - ref_frenet.y);
          right_point.l = std::fmax(-drive_width, right_y - ref_frenet.y);
        }
        MSD_LOG(INFO, "DEBUG_CJ5:S,ref_s:%3.1f,ref_l:3.1f,lb:%3.1f,rb:%3.1f",
                ref_frenet.x, ref_frenet.y, left_point.l, right_point.l);
      } else {
        MSD_LOG(INFO, "DEBUG_CJ5:F1,ref_s:%3.1f,ref_l:0,lb:%3.1f,rb:%3.1f", s,
                left_point.l, right_point.l);
      }
    } else {
      MSD_LOG(INFO, "DEBUG_CJ5:F2,ref_s:%3.1f,lb:%3.1f,rb:%3.1f", s,
              left_point.l, right_point.l);
    }

    left_border.emplace_back(left_point);
    right_border.emplace_back(right_point);

    Eigen::Vector3d tmp_point;
    tmp_point.x() = raw_point.car_point.x;
    tmp_point.y() = raw_point.car_point.y + left_point.l;
    tmp_point.z() = 0.0;
    leftborder_points_.push_back(tmp_point);

    tmp_point.x() = raw_point.car_point.x;
    tmp_point.y() = raw_point.car_point.y + right_point.l;
    tmp_point.z() = 0.0;
    rightborder_points_.push_back(tmp_point);
  }
  (void)reference_line_.SetPathBorder(PolygonalLine(left_border),
                                      PolygonalLine(right_border));

  Point2D ego_frenet;
  if (frenet_coord_->CartCoord2FrenetCoord(
          ego_state_manager_.get_ego_state().ego_carte, ego_frenet) ==
      TRANSFORM_FAILED) {
    construct_failed_reason_ =
        "[MAP_ERROR] baseline transformed failed ego cart Point2D";
    MSD_LOG(WARN, "ERROR_CJ3 TimeStamp:%3.2f:", MTIME()->timestamp().sec());
    MSD_LOG(WARN, "ERROR_CJ3 %s: construct refline failed P2!", __FUNCTION__);

    return false;
  } else {
    ego_state_manager_.get_mutable_ego_state().ego_frenet = ego_frenet;
  }

  // update start point in baseline
  Point2D cart, fren;
  CartesianState cs0;
  FrenetState start_state;
  cart.x = baseline_ego_state.planning_init_point.path_point.x;
  cart.y = baseline_ego_state.planning_init_point.path_point.y;
  cs0.x = baseline_ego_state.planning_init_point.path_point.x;
  cs0.y = baseline_ego_state.planning_init_point.path_point.y;
  cs0.yaw = baseline_ego_state.planning_init_point.path_point.theta;
  cs0.speed = std::max(0.0, baseline_ego_state.planning_init_point.v);
  cs0.acceleration = baseline_ego_state.planning_init_point.a;
  cs0.curvature = baseline_ego_state.planning_init_point.path_point.kappa;
  if (frenet_coord_->CartState2FrenetState(cs0, start_state) ==
      TRANSFORM_FAILED) {
    construct_failed_reason_ =
        "[MAP_ERROR] baseline transformed failed FrenetState";
    MSD_LOG(
        WARN,
        " [MAP_ERROR] baseline[%d] transformed FrenetState failed cart %f %f",
        lane_id_, cs0.x, cs0.y);
    MSD_LOG(WARN, "ERROR_CJ3 %s: construct refline failed P3!", __FUNCTION__);
    return false;
  }
  baseline_ego_state.planning_init_point.path_point.s = start_state.s;
  baseline_ego_state.planning_start_state.s = start_state.s;
  baseline_ego_state.planning_start_state.r = start_state.r;
  baseline_ego_state.planning_start_state.ds = std::max(0.0, start_state.ds);
  baseline_ego_state.planning_start_state.dr_ds = start_state.dr_ds;
  baseline_ego_state.planning_start_state.dr = start_state.dr;
  baseline_ego_state.planning_start_state.dds = start_state.dds;
  baseline_ego_state.planning_start_state.ddr_dsds = start_state.ddr_dsds;
  baseline_ego_state.planning_start_state.ddr = start_state.ddr;
  baseline_ego_state.mpc_vehicle_state[0] =
      baseline_ego_state.planning_init_point.v;
  baseline_ego_state.mpc_vehicle_state[1] =
      baseline_ego_state.planning_init_point.a;
  baseline_ego_state.mpc_vehicle_state[2] =
      baseline_ego_state.planning_init_point.path_point.theta -
      frenet_coord_->GetRefCurveHeading(fren.x);

  cart.x = baseline_ego_state.real_init_point.path_point.x;
  cart.y = baseline_ego_state.real_init_point.path_point.y;

  if (frenet_coord_->CartCoord2FrenetCoord(cart, fren) == TRANSFORM_FAILED) {
    construct_failed_reason_ =
        "[MAP_ERROR] baseline transformed failed real_init_point";
    MSD_LOG(WARN,
            "[MAP_ERROR] baseline[%d] transformed failed real_init_point cart "
            "%f %f",
            lane_id_, cart.x, cart.y);
    MSD_LOG(WARN, "ERROR_CJ3 %s: construct refline failed P4!", __FUNCTION__);
    return false;
  }

  baseline_ego_state.real_init_point.path_point.s = fren.x;
  Point2D front_center_cart, front_center_fren;
  front_center_cart.x =
      cart.x +
      std::cos(baseline_ego_state.planning_init_point.path_point.theta) *
          ConfigurationContext::Instance()->get_vehicle_param().length / 2.0;
  front_center_cart.y =
      cart.y +
      std::sin(baseline_ego_state.planning_init_point.path_point.theta) *
          ConfigurationContext::Instance()->get_vehicle_param().length / 2.0;

  if (frenet_coord_->CartCoord2FrenetCoord(
          front_center_cart, front_center_fren) == TRANSFORM_FAILED) {
    construct_failed_reason_ =
        "[MAP_ERROR] baseline transformed failed front_center_cart";
    MSD_LOG(
        WARN,
        "[MAP_ERROR] baseline[%d] transformed failed front_center_cart %f %f",
        lane_id_, front_center_cart.x, front_center_cart.y);
    MSD_LOG(WARN, "ERROR_CJ3 %s: construct refline failed P5!", __FUNCTION__);
    return false;
  }
  return true;
}

ReferenceLinePointDerived
BaseLineInfo::find_point_by_x(std::vector<ReferenceLinePointDerived> &ref_line,
                              double x) {
  int i = 0;
  ReferenceLinePointDerived point;

  if (ref_line.size() < 2) {
    return ref_line[0];
  }
  for (i = 0; i < ref_line.size(); ++i) {
    if (ref_line[i].car_point.x > x) {
      break;
    }
  }
  if (i == 0) {
    return ref_line[0];
  } else if (i == ref_line.size()) {
    return ref_line[i - 1];
  } else {
    if (ref_line[i].car_point.x - ref_line[i - 1].car_point.x < 0.1) {
      return ref_line[i];
    }
    double rat = (x - ref_line[i - 1].car_point.x) /
                 (ref_line[i].car_point.x - ref_line[i - 1].car_point.x);
    point.enu_point.x =
        ref_line[i - 1].enu_point.x +
        rat * (ref_line[i].enu_point.x - ref_line[i - 1].enu_point.x);
    point.enu_point.y =
        ref_line[i - 1].enu_point.y +
        rat * (ref_line[i].enu_point.y - ref_line[i - 1].enu_point.y);
    point.enu_point.z =
        ref_line[i - 1].enu_point.z +
        rat * (ref_line[i].enu_point.z - ref_line[i - 1].enu_point.z);
    point.curvature = ref_line[i - 1].curvature +
                      rat * (ref_line[i].curvature - ref_line[i - 1].curvature);
    point.distance_to_left_road_border =
        ref_line[i - 1].distance_to_left_road_border +
        rat * (ref_line[i].distance_to_left_road_border -
               ref_line[i - 1].distance_to_left_road_border);
    point.distance_to_right_road_border =
        ref_line[i - 1].distance_to_right_road_border +
        rat * (ref_line[i].distance_to_right_road_border -
               ref_line[i - 1].distance_to_right_road_border);
    point.distance_to_left_lane_border =
        ref_line[i - 1].distance_to_left_lane_border +
        rat * (ref_line[i].distance_to_left_lane_border -
               ref_line[i - 1].distance_to_left_lane_border);
    point.distance_to_right_lane_border =
        ref_line[i - 1].distance_to_right_lane_border +
        rat * (ref_line[i].distance_to_right_lane_border -
               ref_line[i - 1].distance_to_right_lane_border);
    point.distance_to_left_obstacle =
        ref_line[i - 1].distance_to_left_obstacle +
        rat * (ref_line[i].distance_to_left_obstacle -
               ref_line[i - 1].distance_to_left_obstacle);
    point.distance_to_right_obstacle =
        ref_line[i - 1].distance_to_right_obstacle +
        rat * (ref_line[i].distance_to_right_obstacle -
               ref_line[i - 1].distance_to_right_obstacle);
    point.lane_width =
        ref_line[i - 1].lane_width +
        rat * (ref_line[i].lane_width - ref_line[i - 1].lane_width);
    point.max_velocity =
        ref_line[i - 1].max_velocity +
        rat * (ref_line[i].max_velocity - ref_line[i - 1].max_velocity);
    point.track_id = ref_line[i].track_id;
    point.is_experience_data = ref_line[i].is_experience_data;
    point.left_road_border_type = ref_line[i].left_road_border_type;
    point.right_road_border_type = ref_line[i].right_road_border_type;
    point.horizontal_slope =
        ref_line[i - 1].horizontal_slope +
        rat * (ref_line[i].horizontal_slope - ref_line[i - 1].horizontal_slope);
    point.direction = ref_line[i].direction;
    point.in_intersection = ref_line[i].in_intersection;
    point.yaw =
        ref_line[i - 1].yaw + rat * (ref_line[i].yaw - ref_line[i - 1].yaw);
    point.car_point.x =
        ref_line[i - 1].car_point.x +
        rat * (ref_line[i].car_point.x - ref_line[i - 1].car_point.x);
    point.car_point.y =
        ref_line[i - 1].car_point.y +
        rat * (ref_line[i].car_point.y - ref_line[i - 1].car_point.y);
    point.car_point.z =
        ref_line[i - 1].car_point.z +
        rat * (ref_line[i].car_point.z - ref_line[i - 1].car_point.z);
    return point;
  }
}

int BaseLineInfo::find_num_by_x(
    std::vector<ReferenceLinePointDerived> &ref_line, double x) {
  int i = 0;

  if (ref_line.size() < 2) {
    return 0;
  }
  for (i = 0; i < ref_line.size(); ++i) {
    if (ref_line[i].car_point.x > x) {
      break;
    }
  }

  return std::max(i - 1, 0);
}

bool BaseLineInfo::construct_obstacle_manager(
    const std::shared_ptr<WorldModel> &world_model) {
  obstacle_manager_->clear();
  double adc_start_s(std::numeric_limits<double>::max());
  double adc_end_s(std::numeric_limits<double>::lowest());
  double adc_start_l(std::numeric_limits<double>::max());
  double adc_end_l(std::numeric_limits<double>::lowest());
  static std::vector<planning_math::Vec2d> adc_points;
  adc_points.clear();
  const auto &adc_point =
      ego_state_manager_.get_ego_state().planning_init_point.path_point;
  planning_math::Vec2d center(adc_point.x, adc_point.y);
  planning_math::Box2d adc_box(
      center, adc_point.theta,
      ConfigurationContext::Instance()->get_vehicle_param().length,
      ConfigurationContext::Instance()->get_vehicle_param().width);
  adc_box.GetAllCorners(&adc_points);
  for (const planning_math::Vec2d &adc_point : adc_points) {
    Point2D frenet_point, carte_point;
    carte_point.x = adc_point.x();
    carte_point.y = adc_point.y();
    if (frenet_coord_->CartCoord2FrenetCoord(carte_point, frenet_point) ==
        TRANSFORM_FAILED) {
      return false;
    }
    adc_start_s = std::min(adc_start_s, frenet_point.x);
    adc_end_s = std::max(adc_end_s, frenet_point.x);
    adc_start_l = std::min(adc_start_l, frenet_point.y);
    adc_end_l = std::max(adc_end_l, frenet_point.y);
  }
  adc_sl_boundary_.start_s = adc_start_s;
  adc_sl_boundary_.end_s = adc_end_s;
  adc_sl_boundary_.start_l = adc_start_l;
  adc_sl_boundary_.end_l = adc_end_l;

  // construct obstacle and obstaclemanager accordingly
  for (const auto &prediction_object : world_model->get_prediction_info()) {
    bool is_static = prediction_object.speed < 0.1 ||
                     prediction_object.trajectory_array.empty();
    double prediction_rel_t = prediction_object.delay_time;
    if (prediction_object.type == MSD_OBJECT_TYPE_CONE_BUCKET) {
      maf_perception_interface::PerceptionFusionObjectData cone_bucket_fusion;
      for (const auto &fusion : world_model->get_fusion_info()) {
        if (fusion.track_id == prediction_object.id) {
          cone_bucket_fusion = fusion;
          break;
        }
      }

      if (nullptr == obs_ptr) {
        obs_ptr = std::make_shared<Obstacle>(prediction_object.id, cone_bucket_fusion,
                                             is_static);
      }
      obs_ptr->update(prediction_object.id, cone_bucket_fusion, is_static);
      auto mutable_obs = obstacle_manager_->add_obstacle(*obs_ptr);
      SLBoundary perception_sl{};
      mutable_obs->ComputeSlBoundary(frenet_coord_, &perception_sl);
      mutable_obs->SetPerceptionSlBoundary(perception_sl);
      obstacle_manager_->mutable_hash_id_map()[prediction_object.id].push_back(
          prediction_object.id);
      continue;
    }
    if (prediction_object.trajectory_array.empty()) {
      if (nullptr == obs_ptr) {
        obs_ptr = std::make_shared<Obstacle>(prediction_object.id, is_static,
                                             prediction_object, true,
                                             prediction_rel_t);
      }
      obs_ptr->update(prediction_object.id, is_static, prediction_object, true,
                      prediction_rel_t);
      auto mutable_obs = obstacle_manager_->add_obstacle(*obs_ptr);
      SLBoundary perception_sl{};
      mutable_obs->ComputeSlBoundary(frenet_coord_, &perception_sl);
      mutable_obs->SetPerceptionSlBoundary(perception_sl);
      obstacle_manager_->mutable_hash_id_map()[prediction_object.id].push_back(
          prediction_object.id);
      continue;
    }
    for (size_t i = 0; i < prediction_object.trajectory_array.size(); ++i) {
      if (prediction_object.trajectory_array[i].b_minor_modal) {
        continue;
      }
      int hash_id = hash_prediction_id(prediction_object.id, i);
      if (nullptr == obs_ptr) {
        obs_ptr =                       // parasoft-suppress AUTOSAR-M6_5_3
            std::make_shared<Obstacle>( // parasoft-suppress AUTOSAR-M6_5_3
                hash_id, i, // parasoft-suppress AUTOSAR-M6_5_3 "f-drop"
                prediction_object, is_static, prediction_rel_t);
      }
      obs_ptr->update(hash_id, i, prediction_object, is_static,
                      prediction_rel_t);
      auto mutable_obs = obstacle_manager_->add_obstacle(*obs_ptr);
      SLBoundary perception_sl{};
      mutable_obs->ComputeSlBoundary(frenet_coord_, &perception_sl);
      mutable_obs->SetPerceptionSlBoundary(perception_sl);
      obstacle_manager_->mutable_hash_id_map()[prediction_object.id].push_back(
          hash_id);
    }
  }

  // construct obstacle according to lidar road edge
  if (ConfigurationContext::Instance()
          ->planner_config()
          .longitudinal_motion_planner_config.use_lidar_rb) {
    double start_time = MTIME()->timestamp().ms();
    generate_road_boundary_obstacles(world_model);
    double end_time = MTIME()->timestamp().ms();
    MSD_LOG(INFO, "DBRB1212 cost time: %f", end_time - start_time);
  }
  return true;
}

bool BaseLineInfo::is_adc_on_lane() {
  static std::vector<planning_math::Vec2d> adc_points, adc_fren_points;
  adc_points.clear();
  adc_fren_points.clear();
  const auto &adc_point = get_ego_state().ego_pose;
  planning_math::Vec2d center(adc_point.x, adc_point.y);
  planning_math::Box2d adc_box(
      center, adc_point.theta,
      ConfigurationContext::Instance()->get_vehicle_param().length,
      ConfigurationContext::Instance()->get_vehicle_param().width);
  bool is_adc_sl_box_valid = true;
  adc_box.GetAllCorners(&adc_points);
  for (const planning_math::Vec2d &obs_point : adc_points) {
    Point2D frenet_point, carte_point;
    carte_point.x = obs_point.x();
    carte_point.y = obs_point.y();
    if (frenet_coord_->CartCoord2FrenetCoord(carte_point, frenet_point) ==
        TRANSFORM_FAILED) {
      is_adc_sl_box_valid = false;
      break;
    }
    planning_math::Vec2d fren_p(frenet_point.x, frenet_point.y);
    adc_fren_points.emplace_back(fren_p);
  }
  if (!is_adc_sl_box_valid) {
    adc_fren_points.clear();
    planning_math::Vec2d left_begin(adc_sl_boundary_.start_s,
                                    adc_sl_boundary_.end_l);
    planning_math::Vec2d left_end(adc_sl_boundary_.end_s,
                                  adc_sl_boundary_.end_l);
    planning_math::Vec2d right_begin(adc_sl_boundary_.start_s,
                                     adc_sl_boundary_.start_l);
    planning_math::Vec2d right_end(adc_sl_boundary_.end_s,
                                   adc_sl_boundary_.start_l);
    adc_fren_points = {left_begin, left_end, right_end, right_begin};
  }
  planning_math::Polygon2d adc_polygon(adc_fren_points);
  const auto &left_border = reference_line_.LeftBorder();
  const auto &right_border = reference_line_.RightBorder();
  constexpr double kNearLaneSRange = 5.0;
  constexpr double kDefaultLaneWidth = 3.8;
  const auto &adc_sl = get_ego_state().ego_frenet;
  const auto &adc_carte = get_ego_state().ego_carte;
  PathPoint adc_cart_point;
  adc_cart_point.x = adc_carte.x;
  adc_cart_point.y = adc_carte.y;
  double matched_lane_s =
      reference_line_.discretized_path().QueryMatchedS(adc_cart_point);
  double left_begin_l{kDefaultLaneWidth / 2.0},
      left_end_l{kDefaultLaneWidth / 2.0},
      right_begin_l{-kDefaultLaneWidth / 2.0},
      right_end_l{-kDefaultLaneWidth / 2.0};
  (void)left_border.EvaluateByS(std::max(0.0, matched_lane_s - kNearLaneSRange),
                                &left_begin_l);
  (void)left_border.EvaluateByS(std::max(0.0, matched_lane_s + kNearLaneSRange),
                                &left_end_l);
  (void)right_border.EvaluateByS(
      std::max(0.0, matched_lane_s - kNearLaneSRange), &right_begin_l);
  (void)right_border.EvaluateByS(
      std::max(0.0, matched_lane_s + kNearLaneSRange), &right_end_l);
  planning_math::Vec2d left_begin_point(
      std::max(0.0, adc_sl.x - kNearLaneSRange), left_begin_l);
  planning_math::Vec2d left_end_point(std::max(0.0, adc_sl.x + kNearLaneSRange),
                                      left_end_l);
  planning_math::Vec2d right_begin_point(
      std::max(0.0, adc_sl.x - kNearLaneSRange), right_begin_l);
  planning_math::Vec2d right_end_point(
      std::max(0.0, adc_sl.x + kNearLaneSRange), right_end_l);
  static std::vector<planning_math::Vec2d> reference_line_vertexes;
  reference_line_vertexes.clear();
  reference_line_vertexes.push_back(left_begin_point);
  reference_line_vertexes.push_back(left_end_point);
  reference_line_vertexes.push_back(right_end_point);
  reference_line_vertexes.push_back(right_begin_point);
  planning_math::Polygon2d reference_line_segment;
  if (!planning_math::Polygon2d::ComputeConvexHull(reference_line_vertexes,
                                                   &reference_line_segment)) {
    return false;
  }
  planning_math::Polygon2d overlap_polygon;
  constexpr double kOnLaneAreaRatio = 0.9;
  if (adc_polygon.ComputeOverlap(reference_line_segment, &overlap_polygon)) {
    MSD_LOG(INFO, "is on lane debug: overlap_polygon %f ratio %f",
            overlap_polygon.area(),
            overlap_polygon.area() / adc_polygon.area());
    if (overlap_polygon.area() / adc_polygon.area() > kOnLaneAreaRatio) {
      return true;
    }
  }
  return false;
}

void BaseLineInfo::compute_convex_hull(double left_width, double right_width) {
  mph_assert(!reference_line_.discretized_path().empty());
  const auto &path = reference_line_.discretized_path();
  auto s_length = path.Length();
  MSD_LOG(INFO, "reference_line[%d] length %f", lane_id_, s_length);
  constexpr double kMinSweepDis = 2.0;
  constexpr double kMaxSweepDis = 10.0;
  constexpr double kMaxSweepLength = 120.0;
  constexpr double kIgnoreCurvatureRadius = 50.0;
  static std::vector<planning_math::Vec2d> left_points, right_points;
  left_points.clear();
  right_points.clear();
  double last_insert_s = 0.0;
  double sweep_length = std::min(kMaxSweepLength, frenet_coord_->GetLength());
  for (double cur_s = 0.0; cur_s < sweep_length; cur_s += kMinSweepDis) {
    Point2D cur_left_fren(cur_s, left_width);
    Point2D cur_right_fren(cur_s, -right_width);
    Point2D cur_left_cart, cur_right_cart;
    auto curvature = frenet_coord_->GetRefCurveCurvature(cur_s);
    if (std::abs(curvature) < 1.0 / kIgnoreCurvatureRadius) {
      if (cur_s - last_insert_s < kMaxSweepDis && !left_points.empty() &&
          cur_s + kMinSweepDis < sweep_length) {
        continue;
      }
    }

    if (frenet_coord_->FrenetCoord2CartCoord(cur_left_fren, cur_left_cart) ==
        TRANSFORM_FAILED) {
      continue;
    }
    if (frenet_coord_->FrenetCoord2CartCoord(cur_right_fren, cur_right_cart) ==
        TRANSFORM_FAILED) {
      continue;
    }
    left_points.emplace_back(
        planning_math::Vec2d(cur_left_cart.x, cur_left_cart.y));
    right_points.emplace_back(
        planning_math::Vec2d(cur_right_cart.x, cur_right_cart.y));
    last_insert_s = cur_s;
  }
  static std::vector<planning_math::Vec2d> total_points;
  total_points.assign(left_points.begin(), left_points.end());
  std::reverse(right_points.begin(), right_points.end());
  total_points.insert(total_points.end(), right_points.begin(),
                      right_points.end());
  (void)planning_math::Polygon2d::ComputeConvexHull(total_points,
                                                    &convex_hull_);
  MSD_LOG(INFO, "debug baseline total points %d convex_hull size %d",
          total_points.size(), convex_hull_.points().size());
}

bool BaseLineInfo::is_obstacle_intersect_with_lane(const Obstacle *obstacle) {
  if (obstacle->IsStatic() || obstacle->Trajectory().empty() ||
      std::fabs(obstacle->Trajectory().back().path_point.s -
                obstacle->Trajectory().front().path_point.s) < 1.e-2) {
    auto obs_box = obstacle->PerceptionBoundingBox();
    planning_math::Polygon2d obs_polygon(obs_box);
    return convex_hull_.HasOverlap(obs_polygon);
  } else {
    static std::vector<PathPoint> path_points;
    path_points.clear();
    for (const auto &traj_point : obstacle->Trajectory()) {
      path_points.emplace_back(traj_point.path_point);
    }
    DiscretizedPath obs_path(path_points);
    constexpr double kMaxSweepStepLength = 2.5;
    constexpr double kMinSweepStepLength = 1.0;
    constexpr double kMaxConsiderLength = 250.0;
    double obs_size = 0.8 * std::min(obstacle->PerceptionBoundingBox().length(),
                                     obstacle->PerceptionBoundingBox().width());
    double sweep_step_length =
        std::max(kMinSweepStepLength, std::min(kMaxSweepStepLength, obs_size));
    // MSD_LOG(INFO, "obstacle[%d] length %f %f speed %f path point size %d",
    // obstacle->Id(), obs_path.front().s, obs_path.back().s, obstacle->speed(),
    // path_points.size());
    for (double cur_s = obs_path.front().s;
         cur_s < std::min(kMaxConsiderLength, obs_path.back().s + 1.e-3);
         cur_s += sweep_step_length) {
      auto cur_point = obs_path.Evaluate(cur_s);
      TrajectoryPoint cur_traj_point;
      cur_traj_point.path_point = cur_point;
      auto cur_obs_box = obstacle->GetBoundingBox(cur_traj_point);
      planning_math::Polygon2d cur_obs_polygon(cur_obs_box);
      if (convex_hull_.HasOverlap(cur_obs_polygon)) {
        return true;
      }
    }
    return false;
  }
}

void BaseLineInfo::generate_road_boundary_obstacles(
    const std::shared_ptr<WorldModel> &world_model) {
  const auto &frenet_coord = frenet_coord_;
  const auto &ego_frenet = ego_state_manager_.get_ego_state().ego_frenet;
  const auto &perception_lidar_road_edges =
      world_model->get_perception_lidar_road_edges();
  const auto &road_edges =
      perception_lidar_road_edges.road_edge_perception.road_edges;
  auto &obstacle_manager = *obstacle_manager_;
  const double K_lat_offset = std::abs(lane_id_) * 3.5;
  const double K_LonBoundThr = 80;
  const double K_LatBuffer = 0.05;
  const double K_LonBufferThr = 0.1;
  const double K_diff_theta = M_PI / 6.0;
  const double K_VirtualLaneWidth = 2.8;
  const double K_LaneWidthFactor = 0.5;
  const double K_HalfVehicleLen =
      ConfigurationContext::Instance()->get_vehicle_param().length * 0.5;
  const double ego_enu_x =
      ego_state_manager_.get_ego_state().ego_enu.position.x;
  const double ego_enu_y =
      ego_state_manager_.get_ego_state().ego_enu.position.y;
  const double ego_enu_z =
      ego_state_manager_.get_ego_state().ego_enu.position.z;
  const double ego_fre_x = ego_state_manager_.get_ego_state().ego_frenet.x;
  const double ego_fre_y = ego_state_manager_.get_ego_state().ego_frenet.y;

  lidar_rb_id_vec_.clear();
  MSD_LOG(INFO, "DEBUG_LX: rb_brake_to_stop---------------------------");
  MSD_LOG(INFO, "DEBUG_LX: num of road_edges: %d", road_edges.size());
  MSD_LOG(INFO, "DEBUG_LX: init num of obstacles: %d",
          obstacle_manager.get_obstacles().Dict().size());
  MSD_LOG(INFO, "DEBUG_LX: ego_enu_xyz: %f, %f, %f", ego_enu_x, ego_enu_y,
          ego_enu_z);

  auto is_point_in_range = [&](const Point2D &point) -> bool {
    return point.x > ego_frenet.x + K_HalfVehicleLen &&
           point.x < ego_frenet.x + K_LonBoundThr &&
           point.y > -K_lat_offset - K_LaneWidthFactor * K_VirtualLaneWidth &&
           point.y < K_lat_offset + K_LaneWidthFactor * K_VirtualLaneWidth;
  };

  auto is_theta_in_range = [&](const Point2D &point1, const Point2D &point2,
                               double &theta) -> bool {
    double dl = point2.y - point1.y;
    double ds = point2.x - point1.x;
    theta = atan2(dl, ds);
    return (theta > K_diff_theta && theta < M_PI - K_diff_theta) ||
           (theta > K_diff_theta - M_PI && theta < -K_diff_theta);
  };

  MSD_LOG(INFO, "DBRB:----------------------------");
  MSD_LOG(INFO, "DBRB: ego_fre_x %f", ego_fre_x);
  MSD_LOG(INFO, "DBRB: ego_fre_y %f", ego_fre_y);

  for (int i = 0; i < road_edges.size(); ++i) {
    if (road_edges[i].lidar_roadedge_type.value ==
            maf_perception_interface::LidarRoadEdgeTypeEnum::
                LIDAR_ROADEDGE_TYPE_CONSTRUCTION ||
        road_edges[i].lidar_roadedge_type.value ==
            maf_perception_interface::LidarRoadEdgeTypeEnum::
                LIDAR_ROADEDGE_TYPE_RESERVED) {
      continue;
    }
    if (!is_edge_in_boundary(road_edges[i])) {
      continue;
    }
    MSD_LOG(INFO, "DBRB: id %d", road_edges[i].track_id);
    std::vector<Point2D> road_boundary; // enu
    road_boundary_interpolation(road_edges[i], road_boundary);

    bool has_segment_within_range = false;
    std::vector<std::vector<Point2D>> segments;
    std::vector<Point2D> segment;
    std::vector<std::vector<double>> coarse_outlines;
    double coarse_min_s = std::numeric_limits<double>::max();
    double coarse_max_s = std::numeric_limits<double>::lowest();
    double coarse_min_l = std::numeric_limits<double>::max();
    double coarse_max_l = std::numeric_limits<double>::lowest();
    for (int j = 0; j < road_boundary.size(); ++j) {
      // cal car point by enu point
      Point2D fren_point, cart_point;
      cart_point.x = road_boundary[j].x;
      cart_point.y = road_boundary[j].y;
      if (frenet_coord->CartCoord2FrenetCoord(cart_point, fren_point) ==
          TRANSFORM_FAILED) {
        continue;
      }

      const bool point_in_range = is_point_in_range(fren_point);
      if (point_in_range) {
        segment.emplace_back(fren_point);
        coarse_max_s = std::fmax(coarse_max_s, fren_point.x);
        coarse_min_s = std::fmin(coarse_min_s, fren_point.x);
        coarse_max_l = std::fmax(coarse_max_l, fren_point.y);
        coarse_min_l = std::fmin(coarse_min_l, fren_point.y);
        has_segment_within_range = true;
      }
      if ((!point_in_range && has_segment_within_range) ||
          (j == road_boundary.size() - 1 && has_segment_within_range)) {
        segments.emplace_back(segment);
        segment.clear();
        std::vector<double> coarse_outline;
        coarse_outline.emplace_back(coarse_max_s);
        coarse_outline.emplace_back(coarse_min_s);
        coarse_outline.emplace_back(coarse_max_l);
        coarse_outline.emplace_back(coarse_min_l);
        coarse_outlines.emplace_back(coarse_outline);
        coarse_min_s = std::numeric_limits<double>::max();
        coarse_max_s = std::numeric_limits<double>::lowest();
        coarse_min_l = std::numeric_limits<double>::max();
        coarse_max_l = std::numeric_limits<double>::lowest();
        has_segment_within_range = false;
      }
    }

    if (segments.empty()) {
      MSD_LOG(INFO, "DBRB: no segment in range");
      continue;
    }

    std::vector<std::vector<double>> outlines;
    const bool is_in_intersection =
        world_model->get_map_info().is_in_intersection();
    for (size_t j = 0; j < segments.size(); ++j) {
      MSD_LOG(INFO, "DBRB: coarse_outline %d", j);
      MSD_LOG(INFO, "DBRB: %f", coarse_outlines[j][0]);
      MSD_LOG(INFO, "DBRB: %f", coarse_outlines[j][1]);
      MSD_LOG(INFO, "DBRB: %f", coarse_outlines[j][2]);
      MSD_LOG(INFO, "DBRB: %f", coarse_outlines[j][3]);

      const double theta_rad =
          std::atan2(coarse_outlines[j][2] - coarse_outlines[j][3],
                     coarse_outlines[j][0] - coarse_outlines[j][1]);
      const double theta_thr_rad = is_in_intersection ? 0.4 : 0.2;
      if (theta_rad < theta_thr_rad) {
        MSD_LOG(INFO, "DBRB: case 1");
        MSD_LOG(INFO, "DBRB: theta %f", theta_rad);
        continue;
      }

      if (segments[j].size() == 1) {
        if (0.5 * K_VirtualLaneWidth - coarse_outlines[j][3] < 0.25 ||
            coarse_outlines[j][2] + 0.5 * K_VirtualLaneWidth < 0.25) {
          MSD_LOG(INFO, "DBRB: case 2");
          MSD_LOG(INFO, "DBRB: maxl %f", coarse_outlines[j][2]);
          MSD_LOG(INFO, "DBRB: minl %f", coarse_outlines[j][3]);
        } else {
          std::vector<double> outline;
          outline.emplace_back(coarse_outlines[j][0] + K_LonBufferThr);
          outline.emplace_back(coarse_outlines[j][1] - K_LonBufferThr);
          outline.emplace_back(coarse_outlines[j][2] + K_LatBuffer);
          outline.emplace_back(coarse_outlines[j][3] - K_LatBuffer);
          outlines.emplace_back(outline);
          MSD_LOG(INFO, "DBRB: case 3");
          MSD_LOG(INFO, "DBRB: %f", outline[0]);
          MSD_LOG(INFO, "DBRB: %f", outline[1]);
          MSD_LOG(INFO, "DBRB: %f", outline[2]);
          MSD_LOG(INFO, "DBRB: %f", outline[3]);
        }
        continue;
      }

      if (coarse_outlines[j][2] - coarse_outlines[j][3] > 1.0 ||
          (coarse_outlines[j][2] > 0 && coarse_outlines[j][3] < 0)) {
        std::vector<double> outline;
        outline.emplace_back(coarse_outlines[j][0] + K_LonBufferThr);
        outline.emplace_back(coarse_outlines[j][1] - K_LonBufferThr);
        outline.emplace_back(coarse_outlines[j][2] + K_LatBuffer);
        outline.emplace_back(coarse_outlines[j][3] - K_LatBuffer);
        outlines.emplace_back(outline);
        MSD_LOG(INFO, "DBRB: case 4");
        MSD_LOG(INFO, "DBRB: %f", outline[0]);
        MSD_LOG(INFO, "DBRB: %f", outline[1]);
        MSD_LOG(INFO, "DBRB: %f", outline[2]);
        MSD_LOG(INFO, "DBRB: %f", outline[3]);
        continue;
      }

      double min_s = std::numeric_limits<double>::max();
      double max_s = std::numeric_limits<double>::lowest();
      double min_l = std::numeric_limits<double>::max();
      double max_l = std::numeric_limits<double>::lowest();
      bool has_segment_in_theta_range = false;
      for (size_t k = 0; k < segments[j].size() - 1; ++k) {
        double theta = 0.0;
        const bool theta_in_range =
            is_theta_in_range(segments[j][k], segments[j][k + 1], theta);
        MSD_LOG(INFO, "DBRB_THETA: theta %f", k, theta);
        if (theta_in_range) {
          min_s = std::fmin(min_s,
                            std::fmin(segments[j][k].x, segments[j][k + 1].x));
          max_s = std::fmax(max_s,
                            std::fmax(segments[j][k].x, segments[j][k + 1].x));
          min_l = std::fmin(min_l,
                            std::fmin(segments[j][k].y, segments[j][k + 1].y));
          max_l = std::fmax(max_l,
                            std::fmax(segments[j][k].y, segments[j][k + 1].y));
          has_segment_in_theta_range = true;
        }
        if ((!theta_in_range && has_segment_in_theta_range) ||
            (k == segments[j].size() - 2 && has_segment_in_theta_range)) {
          std::vector<double> outline;
          outline.emplace_back(max_s + K_LonBufferThr);
          outline.emplace_back(min_s - K_LonBufferThr);
          outline.emplace_back(max_l + K_LatBuffer);
          outline.emplace_back(min_l - K_LatBuffer);
          outlines.emplace_back(outline);
          has_segment_in_theta_range = false;
          min_s = std::numeric_limits<double>::max();
          max_s = std::numeric_limits<double>::lowest();
          min_l = std::numeric_limits<double>::max();
          max_l = std::numeric_limits<double>::lowest();
          MSD_LOG(INFO, "DBRB: case 5");
          MSD_LOG(INFO, "DBRB: %f", outline[0]);
          MSD_LOG(INFO, "DBRB: %f", outline[1]);
          MSD_LOG(INFO, "DBRB: %f", outline[2]);
          MSD_LOG(INFO, "DBRB: %f", outline[3]);
        }
      }
    }

    if (outlines.empty()) {
      MSD_LOG(INFO, "DBRB: no outline in range");
      continue;
    }

    MSD_LOG(INFO, "DBRB: final outline");
    MSD_LOG(INFO, "DBRB: %f", outlines.front()[0]);
    MSD_LOG(INFO, "DBRB: %f", outlines.front()[1]);
    MSD_LOG(INFO, "DBRB: %f", outlines.front()[2]);
    MSD_LOG(INFO, "DBRB: %f", outlines.front()[3]);

    MSD_LOG(INFO, "DB_LX: processed rb %d", road_edges[i].track_id);
    // sort outlines according to min_s
    // find the most critical outline
    auto cmp = [](const std::vector<double> &v1,
                  const std::vector<double> &v2) -> bool {
      return v1[1] < v2[1];
    };
    std::sort(outlines.begin(), outlines.end(), cmp);
    const auto critical_outline = outlines.front();
    // generate obstacle's attribute in cartesian coordinate system
    Point2D sl_point, xy_point;
    sl_point.x = 0.5 * (critical_outline[0] + critical_outline[1]);
    sl_point.y = 0.5 * (critical_outline[2] + critical_outline[3]);
    frenet_coord->FrenetCoord2CartCoord(sl_point, xy_point);

    const auto ptr = obstacle_manager_->find_obstacle(road_edges[i].track_id);
    int id = road_edges[i].track_id;
    if (ptr != nullptr) {
      id *= 10;
    }
    MSD_LOG(INFO, "DBRB: modified id %d", id);
    PredictionObject prediction_object;
    prediction_object.id = id;
    lidar_rb_id_vec_.emplace_back(prediction_object.id);
    // prediction_object.type = -1;//ObjectType::STOP_LINE;
    prediction_object.timestamp_us = 0.0;
    prediction_object.delay_time = 0.0;
    prediction_object.position_x = xy_point.x;
    prediction_object.position_y = xy_point.y;
    prediction_object.length = critical_outline[0] - critical_outline[1];
    prediction_object.width = critical_outline[2] - critical_outline[3];
    prediction_object.speed = 0.0;
    prediction_object.yaw = frenet_coord->GetRefCurveHeading(sl_point.x);
    prediction_object.acc = 0.0;
    prediction_object.is_ego_lane_overlap = true;

    bool is_static = true;
    double prediction_rel_t = prediction_object.delay_time;
    auto obstacle_ptr =
        std::make_shared<Obstacle>(prediction_object.id, is_static,
                                   prediction_object, true, prediction_rel_t);

    auto mutable_obs = obstacle_manager.add_obstacle(*obstacle_ptr);
    SLBoundary perception_sl{};
    mutable_obs->ComputeSlBoundary(frenet_coord_, &perception_sl);
    mutable_obs->SetPerceptionSlBoundary(perception_sl);
    obstacle_manager.mutable_hash_id_map()[prediction_object.id].push_back(
        prediction_object.id);

    outlines.clear();

    MSD_LOG(INFO, "DEBUG_LX: new_obs_ID:%d", obstacle_ptr->Id());
    MSD_LOG(INFO, "DEBUG_LX: new_obs_max_s:%f", critical_outline[0]);
    MSD_LOG(INFO, "DEBUG_LX: new_obs_min_s:%f", critical_outline[1]);
    MSD_LOG(INFO, "DEBUG_LX: new_obs_max_l:%f", critical_outline[2]);
    MSD_LOG(INFO, "DEBUG_LX: new_obs_min_l:%f", critical_outline[3]);
  }
  MSD_LOG(INFO, "DEBUG_LX: final num of obstacles: %d",
          obstacle_manager.get_obstacles().Dict().size());
  MSD_LOG(INFO, "DEBUG_LX_lidar_rb_id_vec size: %d", lidar_rb_id_vec_.size());
}

void BaseLineInfo::road_boundary_interpolation(
    const maf_perception_interface::RoadEdge &road_edge,
    std::vector<Point2D> &road_boundary) {
  const double target_spacing = 0.25;

  const auto &points_3d_x = road_edge.points_3d_x;
  const auto &points_3d_y = road_edge.points_3d_y;
  const auto &points_3d_z = road_edge.points_3d_z;
  if (points_3d_x.empty()) {
    return;
  }

  if (points_3d_x.size() == 1) {
    Point2D point;
    point.x = points_3d_x[0];
    point.y = points_3d_y[0];
    road_boundary.emplace_back(point);
    return;
  }
  // MSD_LOG(INFO, "DEBUG_LX: rb init size ", road_edge.points_3d_x.size());
  Point2D first_point;
  first_point.x = points_3d_x[0];
  first_point.y = points_3d_y[0];
  road_boundary.emplace_back(first_point);
  for (int i = 0; i < points_3d_x.size() - 1; ++i) {
    double dx = points_3d_x[i + 1] - points_3d_x[i];
    double dy = points_3d_y[i + 1] - points_3d_y[i];
    double dist = std::sqrt(dx * dx + dy * dy);
    int seg_num = std::ceil(dist / target_spacing);
    for (int j = 1; j <= seg_num; ++j) {
      Point2D point;
      point.x = points_3d_x[i] + dx * j / seg_num;
      point.y = points_3d_y[i] + dy * j / seg_num;
      road_boundary.emplace_back(point);
    }
  }
  // MSD_LOG(INFO, "DEBUG_LX: rb final size ", road_boundary.size());
}

bool BaseLineInfo::is_edge_in_boundary(
    const maf_perception_interface::RoadEdge &road_edge) {
  const auto &frenet_coord = frenet_coord_;
  const auto &ego_frenet = ego_state_manager_.get_ego_state().ego_frenet;
  const auto &points_3d_x = road_edge.points_3d_x;
  const auto &points_3d_y = road_edge.points_3d_y;
  const auto &points_3d_z = road_edge.points_3d_z;

  const double lat_offset = std::abs(lane_id_) * 3.5;

  const double K_LatBoundThr = 3.0;
  const double K_LonBoundThr = 100;
  const double K_HalfVehicleLen =
      ConfigurationContext::Instance()->get_vehicle_param().length;

  // MSD_LOG(INFO, "DEBUG_LX: processing road edge %d", road_edge.track_id);
  if (points_3d_x.size() != points_3d_y.size() ||
      points_3d_y.size() != points_3d_z.size() ||
      points_3d_z.size() != points_3d_x.size()) {
    // MSD_LOG(INFO, "DEBUG_LX: %d road edge has unequal num of point3d",
    // road_edge.track_id);
    MSD_LOG(INFO, "DEBUG_LX: %d road edge has unequal num of point3d",
            road_edge.track_id);
    return false;
  }

  int count = 0;
  int fail_num = 0;
  for (int i = 0; i < points_3d_x.size(); ++i) {
    Point2D fren_point, cart_point;
    cart_point.x = points_3d_x[i];
    cart_point.y = points_3d_y[i];

    // MSD_LOG(INFO, "DEBUG_LX: car-x %f", cart_point.x);
    // MSD_LOG(INFO, "DEBUG_LX: car-y %f", cart_point.y);

    if (frenet_coord->CartCoord2FrenetCoord(cart_point, fren_point) ==
        TRANSFORM_FAILED) {
      ++fail_num;
      continue;
    }

    // MSD_LOG(INFO, "DEBUG_LX: fren-x %f", fren_point.x);
    // MSD_LOG(INFO, "DEBUG_LX: fren-y %f", fren_point.y);

    if (fren_point.x > ego_frenet.x + K_LonBoundThr ||
        fren_point.x < ego_frenet.x ||
        fren_point.y > lat_offset + K_LatBoundThr ||
        fren_point.y < -lat_offset - K_LatBoundThr) {
      ++count;
    }
  }
  // MSD_LOG(INFO, "DEBUG_LX: total points %d", points_3d_x.size());
  // MSD_LOG(INFO, "DEBUG_LX: no-in range points %d", count);
  // MSD_LOG(INFO, "DEBUG_LX: fail points %d", fail_num);
  if (count + fail_num < points_3d_x.size()) {
    return true;
  }
  return false;
}

} // namespace msquare
