#include "common/world_model.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/obstacle_manager.h"
#include "common/planning_context.h"
#include "planning/common/common.h"
#include "planning/common/sysconf.h"

namespace msquare {

WorldModel::WorldModel() {
  traffic_light_decision_ = std::make_shared<TrafficLightDecision>();
  refline_generator_ = std::make_shared<ReflineGenerator>();

  frenet_parameters_.zero_speed_threshold = 0.1;
  frenet_parameters_.coord_transform_precision = 0.01;
  frenet_parameters_.step_s = 0.3;
  frenet_parameters_.coarse_step_s = 1.0;
  frenet_parameters_.optimization_gamma = 0.5;
  frenet_parameters_.max_iter = 15;
}

WorldModel::~WorldModel() {}

bool WorldModel::init(SceneType scene_type) {
  scene_type_ = scene_type;
  lateral_obstacle_ = std::make_unique<LateralObstacle>();
  refline_generator_->default_init();
  return true;
}

// only update cartesian info
void WorldModel::update_members() {
  MLOG_PROFILING("update_members");
  bool dbw_status_flag = vehicle_dbw_status_;
  if (dbw_status_flag) {
    MSD_LOG(INFO, "dbw msg true");
  } else {
    MSD_LOG(INFO, "dbw msg false!");
  }

  dbw_status_ = true;
  ego_state_manager_.set_ego_carte(
      convert_pose2point(ego_state_manager_.get_cart_ego_state().ego_pose));

  if (map_info_manager_.ref_trajectories().empty() && !is_acc_mode_) {
    MSD_LOG(ERROR, "Reference trajectory empty!!!");
    pretreatment_status_ = false;
    return;
  }
}

bool WorldModel::construct_baseline_infos() {
  MLOG_PROFILING("construct_baseline");
  if (baseline_infos_.find(-1) == baseline_infos_.end() ||
      nullptr == baseline_infos_[-1]) {
    baseline_infos_[-1] = std::make_shared<BaseLineInfo>();
  }
  if (!baseline_infos_[-1]->is_update()) {
    baseline_infos_[-1]->update(shared_from_this(), -1);
  }

  if (baseline_infos_.find(0) == baseline_infos_.end() ||
      nullptr == baseline_infos_[0]) {
    baseline_infos_[0] = std::make_shared<BaseLineInfo>();
  }
  if (!baseline_infos_[0]->is_update()) {
    baseline_infos_[0]->update(shared_from_this(), 0);
  }

  if (baseline_infos_.find(1) == baseline_infos_.end() ||
      nullptr == baseline_infos_[1]) {
    baseline_infos_[1] = std::make_shared<BaseLineInfo>();
  }
  if (!baseline_infos_[1]->is_update()) {
    baseline_infos_[1]->update(shared_from_this(), 0);
  }

  bool has_valid_baseline = false;
  for (auto &baseline : baseline_infos_) {
    if (!baseline.second->is_valid()) {
      MSD_LOG(INFO, "baseline[%d] construct failed", baseline.first);
    } else {
      has_valid_baseline = true;
    }
  }
  return has_valid_baseline;
}

void WorldModel::construct_baseline_info(int relative_lane_id) {
  MLOG_PROFILING("construct_baseline");
  if (baseline_infos_.find(relative_lane_id) == baseline_infos_.end() ||
      nullptr == baseline_infos_[relative_lane_id]) {
    baseline_infos_[relative_lane_id] = std::make_shared<BaseLineInfo>();
  }
  if (!baseline_infos_[relative_lane_id]->is_update()) {
    baseline_infos_[relative_lane_id]->update(shared_from_this(),
                                              relative_lane_id);
    (void)construct_baseline_priority();
  }
}

void WorldModel::generate_prediction_delay_time() {
  double next_planning_start_time = PlanningContext::Instance()
                                        ->planning_status()
                                        .planning_result.next_timestamp_sec;
  auto init_relative_time =
      ego_state_manager_.get_cart_ego_state().planning_init_point.relative_time;
  for (auto &prediction_object : prediction_info_) {
    double prediction_rel_t = prediction_object.timestamp_us / 1.e+6 -
                              next_planning_start_time - init_relative_time;
    prediction_rel_t = clip(prediction_rel_t, 0.0, -1.0);
    prediction_object.delay_time = prediction_rel_t;
    MSD_LOG(ERROR,
            "[prediction delay time] obstacle[%d] absolute start time %f "
            "relative time %f start time %f init_relative_time %f",
            prediction_object.id, prediction_object.timestamp_us,
            prediction_object.timestamp_us / 1.e+6 - next_planning_start_time,
            prediction_rel_t, init_relative_time);
  }
}

void WorldModel::truncate_prediction_info(
    std::shared_ptr<BaseLineInfo> baseline_info) {
  truncated_prediction_info_.clear();
  if (!baseline_info) {
    return;
  }
  const auto &ego_state = baseline_info->get_ego_state();
  auto &obstacle_manager = baseline_info->mutable_obstacle_manager();
  for (const auto &prediction_object : prediction_info_) {
    PredictionObject cur_predicion_obj;
    cur_predicion_obj.id = prediction_object.id;
    cur_predicion_obj.type = prediction_object.type;
    cur_predicion_obj.timestamp_us = prediction_object.timestamp_us;
    cur_predicion_obj.delay_time = prediction_object.delay_time;
    cur_predicion_obj.intention = prediction_object.intention;
    cur_predicion_obj.b_backup_freemove = prediction_object.b_backup_freemove;
    cur_predicion_obj.cutin_score = prediction_object.cutin_score;
    cur_predicion_obj.position_x = prediction_object.position_x;
    cur_predicion_obj.position_y = prediction_object.position_y;
    cur_predicion_obj.length = prediction_object.length;
    cur_predicion_obj.width = prediction_object.width;
    cur_predicion_obj.speed = prediction_object.speed;
    cur_predicion_obj.yaw = prediction_object.yaw;
    cur_predicion_obj.acc = prediction_object.acc;
    cur_predicion_obj.bottom_polygon_points =
        prediction_object.bottom_polygon_points;
    cur_predicion_obj.top_polygon_points = prediction_object.top_polygon_points;
    cur_predicion_obj.trajectory_array.resize(
        prediction_object.trajectory_array.size());
    double delay_time = prediction_object.delay_time;
    size_t traj_index = 0;
    for (const auto &prediction_traj : prediction_object.trajectory_array) {
      auto &cur_prediction_traj =
          cur_predicion_obj.trajectory_array[traj_index];
      int hash_id = hash_prediction_id(prediction_object.id, traj_index);
      Obstacle *ptr_obstacle = obstacle_manager.find_obstacle(hash_id);
      traj_index++;
      if (!ptr_obstacle) {
        continue;
      }
      cur_prediction_traj.trajectory.clear();
      cur_prediction_traj.prob = prediction_traj.prob;
      cur_prediction_traj.prediction_interval =
          prediction_traj.prediction_interval;
      cur_prediction_traj.num_of_points = prediction_traj.num_of_points;
      cur_prediction_traj.intention = prediction_traj.intention;
      cur_prediction_traj.source = prediction_traj.source;
      cur_prediction_traj.b_valid_sigma = prediction_traj.b_valid_sigma;
      // cur_prediction_traj.const_vel_prob = prediction_traj.const_vel_prob;
      // cur_prediction_traj.const_acc_prob = prediction_traj.const_acc_prob;
      // cur_prediction_traj.coord_turn_prob = prediction_traj.coord_turn_prob;
      // cur_prediction_traj.still_prob = prediction_traj.still_prob;
      cur_prediction_traj.b_minor_modal = prediction_traj.b_minor_modal;

      for (int i = 0; i < 41; i++) {
        double relative_time = i * 0.2;
        auto traj_point = ptr_obstacle->GetPointAtTime(relative_time);
        PredictionTrajectoryPoint cur_point;
        cur_point.x = traj_point.path_point.x;
        cur_point.y = traj_point.path_point.y;
        cur_point.theta = traj_point.path_point.theta;
        cur_point.speed = traj_point.v;
        cur_point.std_dev_x = traj_point.sigma_x;
        cur_point.std_dev_y = traj_point.sigma_y;
        cur_point.prob = traj_point.prediction_prob;
        cur_point.yaw = traj_point.velocity_direction;
        // cur_point.relative_ego_x = traj_point.relative_ego_x;
        // cur_point.relative_ego_y = traj_point.relative_ego_y;
        cur_point.relative_ego_yaw = traj_point.relative_ego_yaw;
        // cur_point.relative_ego_speed = traj_point.relative_ego_speed;
        // cur_point.relative_ego_std_dev_x = traj_point.relative_ego_std_dev_x;
        // cur_point.relative_ego_std_dev_y = traj_point.relative_ego_std_dev_y;
        // cur_point.relative_ego_std_dev_yaw =
        // traj_point.relative_ego_std_dev_yaw;
        // cur_point.relative_ego_std_dev_speed =
        // traj_point.relative_ego_std_dev_speed;
        cur_prediction_traj.trajectory.emplace_back(cur_point);
      }
    }
    truncated_prediction_info_.emplace_back(cur_predicion_obj);
  }
}

bool WorldModel::update() {
  // update pre planning status
  MLOG_PROFILING("worldmodel");
  auto *planning_status =
      PlanningContext::Instance()->mutable_planning_status();
  pretreatment_status_ = true;

  // traffic_light_decision_->update(shared_from_this());

  map_info_frenet_.clear();

  if (!map_info_manager_.update(shared_from_this())) {
    planning_status->planning_loop = 0;
    pretreatment_status_ = false;
    MSD_LOG(WARN, "ERROR_CJ9 !map_info_manager_update");
    return false;
  }

  update_members();

  if (!pretreatment_status_) {
    planning_status->planning_loop = 0;
    MSD_LOG(WARN, "ERROR_CJ10 !pretreatment_status_");
    return false;
  }
  // set planning start point
  ego_state_manager_.compute_stitch_trajectory(vehicle_dbw_status_,
                                               throttle_override_);
  for (const auto &base_line : baseline_infos_) {
    base_line.second->clear();
  }

  MSD_LOG(INFO, "DEBUG_CJ4 baseline_infos_ CLEARED");

  generate_prediction_delay_time();

  if (pretreatment_status_ == false) {
    planning_status->planning_loop = 0;
    return false;
  }

  // TODO: when updated failed, reset planning_loop to 0
  planning_status->planning_loop++;

  if (!vehicle_dbw_status_) {
    planning_status->planning_loop = 0;
    // return false;
  }
  return true;
}

bool WorldModel::construct_baseline_priority() {
  // build stop decision
  constexpr double kDefaultMaxStopDistance = 200.0;
  auto &map_info = map_info_manager_.get_map_info();
  const auto &MergeInfo = map_info.next_merge_info();
  int current_lane_index = map_info.current_lane_index();
  bool right_lane_change{false}, left_lane_change{false};
  bool has_same_lane_direction = false;
  bool is_current_lane_on_right_direction =
      (map_info.current_lane_marks() & map_info.traffic_light_direction());

  int final_target_lane = map_info.current_lane_index();
  for (auto direction : map_info.current_tasks()) {
    if (direction * (final_target_lane - current_lane_index) < 0) {
      break;
    }
    final_target_lane += direction;
  }

  int cross_lanes_num = std::abs(final_target_lane - current_lane_index);
  constexpr double kMinLaneChangeBuffer = 20.0;
  double current_lane_change_buffer =
      std::max(0, cross_lanes_num - 1) * kMinLaneChangeBuffer;

  if (cross_lanes_num == 0) {
    if (baseline_infos_.find(0) != baseline_infos_.end() &&
        baseline_infos_.at(0) && baseline_infos_[0]->is_update()) {
      (void)baseline_infos_[0]->set_is_target_lane(true);
    }
  } else if (final_target_lane > current_lane_index) {
    if (baseline_infos_.find(1) != baseline_infos_.end() &&
        baseline_infos_.at(1) && baseline_infos_[1]->is_update()) {
      (void)baseline_infos_[1]->set_is_target_lane(true);
    }
  } else if (final_target_lane < current_lane_index) {
    if (baseline_infos_.find(-1) != baseline_infos_.end() &&
        baseline_infos_.at(-1) && baseline_infos_[-1]->is_update()) {
      (void)baseline_infos_[-1]->set_is_target_lane(true);
    }
  }

  if (is_current_lane_on_right_direction) {
    if (baseline_infos_.find(0) != baseline_infos_.end() &&
        baseline_infos_.at(0) && baseline_infos_[0]->is_update()) {
      (void)baseline_infos_[0]->set_is_target_lane(true);
    }
  }

  if (map_info.left_lane_marks() & map_info.traffic_light_direction()) {
    if (baseline_infos_.find(-1) != baseline_infos_.end() &&
        baseline_infos_.at(-1) && baseline_infos_[-1]->is_update()) {
      (void)baseline_infos_[-1]->set_is_target_lane(true);
    }
  }

  if (map_info.right_lane_marks() & map_info.traffic_light_direction()) {
    if (baseline_infos_.find(1) != baseline_infos_.end() &&
        baseline_infos_.at(1) && baseline_infos_[1]->is_update()) {
      (void)baseline_infos_[1]->set_is_target_lane(true);
    }
  }

  if (baseline_infos_.find(0) != baseline_infos_.end() &&
      baseline_infos_.at(0) && baseline_infos_[0]->is_update()) {
    // set current line right_of_way
    if (MergeInfo.orientation.value != Orientation::UNKNOWN &&
        !MergeInfo.is_continue) {
      baseline_infos_[0]->set_right_of_way_status(BaseLineInfo::UNPROTECTED);
    }
  }

  return true;
}

void WorldModel::get_current_lane_ref(int target_lane_id,
                                      std::vector<Point2D> &current_lane_ref) {
  auto &map_info = map_info_manager_.get_map_info();
  const Transform &car2enu = ego_state_manager_.get_car2enu();
  Eigen::Vector3d car_point, enu_point;

  auto baseline_info = get_baseline_info(target_lane_id);
  if (baseline_info == nullptr) {
    MSD_LOG(ERROR, "[get_current_lane_ref] invalid baseline[%d]",
            target_lane_id);
    return;
  }

  auto frenet_coord = baseline_info->get_frenet_coord();
  if (frenet_coord == nullptr) {
    MSD_LOG(ERROR, "[get_current_lane_ref] invalid frenet_coord [%d]",
            target_lane_id);
    return;
  }

  Point2D lane_point, lane_point_frenet;
  for (const auto &p : map_info.current_refline_points()) {
    car_point.x() = p.car_point.x;
    car_point.y() = p.car_point.y;
    car_point.z() = 0.0;
    enu_point = car2enu * car_point;

    lane_point.x = enu_point.x();
    lane_point.y = enu_point.y();

    (void)frenet_coord->CartCoord2FrenetCoord(lane_point, lane_point_frenet);

    if (lane_point_frenet.x > 1.0e-4 && lane_point_frenet.x < 100) {
      current_lane_ref.push_back(lane_point_frenet);
    }
  }
}

void WorldModel::get_map_lateral_info(std::vector<RefPointFrenet> &cur_lane,
                                      std::vector<RefPointFrenet> &left_lane,
                                      std::vector<RefPointFrenet> &right_lane,
                                      int target_lane_id) {
  MLOG_PROFILING("get_map_lateral_info");
  if (map_info_frenet_.find(target_lane_id) != map_info_frenet_.end()) {
    left_lane.reserve(map_info_frenet_[target_lane_id][0].size());
    left_lane.assign(map_info_frenet_[target_lane_id][0].begin(),
                     map_info_frenet_[target_lane_id][0].end());
    cur_lane.reserve(map_info_frenet_[target_lane_id][1].size());
    cur_lane.assign(map_info_frenet_[target_lane_id][1].begin(),
                    map_info_frenet_[target_lane_id][1].end());
    right_lane.reserve(map_info_frenet_[target_lane_id][2].size());
    right_lane.assign(map_info_frenet_[target_lane_id][2].begin(),
                      map_info_frenet_[target_lane_id][2].end());
    return;
  }
  cur_lane.clear();
  left_lane.clear();
  right_lane.clear();

  static std::vector<ReferenceLinePointDerived> target_lane;
  target_lane.clear();
  auto target_baseline = get_baseline_info(target_lane_id);
  if (!target_baseline || !target_baseline->is_valid()) {
    MSD_LOG(ERROR, "illegal target lane id in func get_map_lateral_info()");
    return;
  }
  auto target_lane_frenet_coord = target_baseline->get_frenet_coord();

  std::string which_lane = "";
  if (target_lane_id == -1) {
    which_lane = "left_line";
  } else if (target_lane_id == 1) {
    which_lane = "right_line";
  }

  auto &map_info = map_info_manager_.get_map_info();
  bool is_smooth = map_info_manager_.is_refline_smooth();

  if (which_lane == "left_line") {
    trans_lane_points_from_baseline(cur_lane, get_baseline_info(-1), is_smooth);
    trans_lane_points_from_baseline(right_lane, get_baseline_info(0), false);
  } else if (which_lane == "right_line") {
    trans_lane_points_from_baseline(cur_lane, get_baseline_info(1), is_smooth);
    trans_lane_points_from_baseline(left_lane, get_baseline_info(0), false);
  } else {
    trans_lane_points_from_baseline(cur_lane, get_baseline_info(0), is_smooth);
    trans_lane_points_from_baseline(left_lane, get_baseline_info(-1), false);
    trans_lane_points_from_baseline(right_lane, get_baseline_info(1), false);
  }

  if (right_lane.empty()) {
    right_lane.resize(cur_lane.size());
    for (size_t i = 0; i < cur_lane.size(); i++) {
      right_lane[i].lane_width = fmin(
          fmax(cur_lane[i].right_road_border - cur_lane[i].right_lane_border,
               0.),
          15);
    }
  }

  if (left_lane.empty()) {
    left_lane.resize(cur_lane.size());
    for (size_t i = 0; i < cur_lane.size(); i++) {
      left_lane[i].lane_width = fmin(
          fmax(cur_lane[i].left_road_border - cur_lane[i].left_lane_border, 0.),
          15);
    }
  }

  for (size_t i = 0; i < cur_lane.size(); i++) {
    if (left_lane.empty()) {
      cur_lane[i].left_road_border =
          std::fmin(cur_lane[i].left_road_border, 4.0);
    }
    if (right_lane.empty()) {
      cur_lane[i].right_road_border =
          std::fmin(cur_lane[i].right_road_border, 4.0);
    }
  }
  static std::array<std::vector<RefPointFrenet>, 3> frenet_map_info;
  frenet_map_info[0].reserve(left_lane.size());
  frenet_map_info[0].assign(left_lane.begin(), left_lane.end());
  frenet_map_info[1].reserve(cur_lane.size());
  frenet_map_info[1].assign(cur_lane.begin(), cur_lane.end());
  frenet_map_info[2].reserve(right_lane.size());
  frenet_map_info[2].assign(right_lane.begin(), right_lane.end());
  map_info_frenet_[target_lane_id] = frenet_map_info;
}

void WorldModel::trans_lane_points(
    const std::vector<ReferenceLinePointDerived> &lane_points,
    std::vector<RefPointFrenet> &refline,
    std::shared_ptr<FrenetCoordinateSystem> frenet_coord, bool smoother_comp) {

  const Transform &car2enu = ego_state_manager_.get_car2enu();
  Eigen::Vector3d car_point, enu_point;

  // auto before_smooth_refline_points =
  //     map_info_manager_.get_map_info().current_refline_points();
  // Point2D point_smooth, point_fren_smooth;
  // std::vector<Point2D> fren_smooth;
  // for (const auto &p : before_smooth_refline_points) {
  //   point_smooth.x = p.enu_point.x;
  //   point_smooth.y = p.enu_point.y;
  //   frenet_coord->CartCoord2FrenetCoord(point_smooth, point_fren_smooth);
  //   fren_smooth.push_back(point_fren_smooth);
  // }
  // trajectories 从 enu 转到 frenet，在 ref_point_fren.s 中找最近点，看 l
  // 偏移了多少，根据 l 的正负进行道路边缘距离的修正:
  // 如果为正，左边道路边缘距离加 l，右边道路边缘减 l
  // 如果为负，左边道路边缘距离减 l，左边道路边缘加 l

  RefPointFrenet ref_point_fren;
  Point2D point, point_fren;

  for (const auto &p : lane_points) {
    car_point.x() = p.car_point.x;
    car_point.y() = p.car_point.y;
    car_point.z() = 0.0;

    enu_point = car2enu * car_point;
    point.x = enu_point.x();
    point.y = enu_point.y();
    // todo:zzd get_target_baseline()->get_frenet_coord()
    (void)frenet_coord->CartCoord2FrenetCoord(point, point_fren);
    // TODO : replace hack width with lane width
    if (point_fren.x > 1.0e-3 && point_fren.x < 100.) {
      ref_point_fren.s = point_fren.x;
      ref_point_fren.lane_width = std::fmin(p.lane_width, 7);
      double smoother_comp_l = (smoother_comp ? point_fren.y : 0);
      // MSD_LOG(INFO, "zylcompdebug : %f", smoother_comp_l);
      ref_point_fren.left_lane_border =
          std::fmin(p.distance_to_left_lane_border, 3.5) + smoother_comp_l;
      ref_point_fren.right_lane_border =
          std::fmin(p.distance_to_right_lane_border, 3.5) - smoother_comp_l;

      if (p.left_road_border_type.value.value == LaneBoundaryForm::PHYSICAL) {
        ref_point_fren.left_road_border =
            std::fmin(p.distance_to_left_obstacle,
                      p.distance_to_left_road_border) +
            smoother_comp_l;
        ref_point_fren.left_road_border_type = 1;
      } else if (std::fmin(p.distance_to_left_obstacle, 100) <
                 std::fmin(p.distance_to_left_road_border, 100) + 0.2) {
        ref_point_fren.left_road_border =
            p.distance_to_left_obstacle + smoother_comp_l;
        ref_point_fren.left_road_border_type = 1;
      } else {
        ref_point_fren.left_road_border =
            p.distance_to_left_road_border + smoother_comp_l;
        ref_point_fren.left_road_border_type = 0;
      }

      if (p.right_road_border_type.value.value == LaneBoundaryForm::PHYSICAL) {
        ref_point_fren.right_road_border =
            std::fmin(p.distance_to_right_obstacle,
                      p.distance_to_right_road_border) -
            smoother_comp_l;
        ref_point_fren.right_road_border_type = 1;
      } else if (std::fmin(p.distance_to_right_obstacle, 100) <
                 std::fmin(p.distance_to_right_road_border, 100) + 0.2) {
        ref_point_fren.right_road_border =
            p.distance_to_right_obstacle - smoother_comp_l;
        ref_point_fren.right_road_border_type = 1;
      } else {
        ref_point_fren.right_road_border =
            p.distance_to_right_road_border - smoother_comp_l;
        ref_point_fren.right_road_border_type = 0;
      }

      ref_point_fren.left_road_border =
          std::fmin(ref_point_fren.left_road_border, 1000);
      ref_point_fren.right_road_border =
          std::fmin(ref_point_fren.right_road_border, 1000);

      refline.push_back(ref_point_fren);
    }
  }
}

void WorldModel::trans_lane_points_from_baseline(
    std::vector<RefPointFrenet> &refline,
    std::shared_ptr<BaseLineInfo> baseline, bool smoother_comp) {
  if (baseline == nullptr || !baseline->is_valid()) {
    return;
  }
  const auto &path_data = baseline->get_path_data();
  const auto &raw_ref_points = baseline->get_raw_refline_points();
  auto initial_cart_point = baseline->get_frenet_coord()->GetRefCurvePoint(0.0);
  PathPoint cart_point;
  cart_point.x = initial_cart_point.x;
  cart_point.y = initial_cart_point.y;
  double matched_lane_s =
      path_data.discretized_path().QueryMatchedS(cart_point);
  for (size_t i = 0; i < path_data.discretized_path().size(); i++) {
    const auto &lane_point = path_data.discretized_path()[i];
    if (i >= raw_ref_points.size()) {
      break;
    }
    const auto &raw_ref_point = raw_ref_points[i];
    if (lane_point.s < matched_lane_s - 1.e-4 ||
        lane_point.s - matched_lane_s >
            std::min(100.0,
                     baseline->get_frenet_coord()->GetLength() - 1.e-4)) {
      continue;
    }
    RefPointFrenet ref_point_fren;
    double frenet_s = std::max(0.0, lane_point.s - matched_lane_s);
    auto point_cart = baseline->get_frenet_coord()->GetRefCurvePoint(frenet_s);
    double theta = baseline->get_frenet_coord()->GetRefCurveHeading(frenet_s);

    ref_point_fren.s = frenet_s;
    auto lane_width = path_data.getWidth(lane_point.s);
    // ref_point_fren.lane_width =
    //     std::fmin(lane_width.first - lane_width.second, 7.0);
    double smoother_comp_l = 0.0;
    if (smoother_comp) {
      Point2D ref_cart_point(lane_point.x, lane_point.y);
      Point2D ref_frenet_point;
      (void)baseline->get_frenet_coord()->CartCoord2FrenetCoord(
          ref_cart_point, ref_frenet_point);
      if (ref_frenet_point.x < 1.0e-3 || ref_frenet_point.x > 100.) {
        continue;
      }
      smoother_comp_l = ref_frenet_point.y;
      ref_point_fren.s = ref_frenet_point.x;
    }
    // smoother_comp_l = clip(smoother_comp_l, -0.2, 0.2);

    double lane_default_width = is_ddmap() ? 2.25 : 3.5;
    ref_point_fren.left_lane_border =
        std::fmin(raw_ref_point.distance_to_left_lane_border,
                  lane_default_width) +
        smoother_comp_l;
    ref_point_fren.right_lane_border =
        std::fmin(raw_ref_point.distance_to_right_lane_border,
                  lane_default_width) -
        smoother_comp_l;
    ref_point_fren.lane_width =
        ref_point_fren.left_lane_border + ref_point_fren.right_lane_border;

    if (raw_ref_point.left_road_border_type.value.value ==
        LaneBoundaryForm::PHYSICAL) {
      ref_point_fren.left_road_border =
          std::fmin(raw_ref_point.distance_to_left_obstacle,
                    raw_ref_point.distance_to_left_road_border) +
          smoother_comp_l;
      ref_point_fren.left_road_border_type = 1;
    } else if (std::fmin(raw_ref_point.distance_to_left_obstacle, 100) <
               std::fmin(raw_ref_point.distance_to_left_road_border, 100) +
                   0.2) {
      ref_point_fren.left_road_border =
          raw_ref_point.distance_to_left_obstacle + smoother_comp_l;
      ref_point_fren.left_road_border_type = 1;
    } else {
      ref_point_fren.left_road_border =
          raw_ref_point.distance_to_left_road_border + smoother_comp_l;
      ref_point_fren.left_road_border_type = 0;
    }

    if (raw_ref_point.right_road_border_type.value.value ==
        LaneBoundaryForm::PHYSICAL) {
      ref_point_fren.right_road_border =
          std::fmin(raw_ref_point.distance_to_right_obstacle,
                    raw_ref_point.distance_to_right_road_border) -
          smoother_comp_l;
      ref_point_fren.right_road_border_type = 1;
    } else if (std::fmin(raw_ref_point.distance_to_right_obstacle, 100) <
               std::fmin(raw_ref_point.distance_to_right_road_border, 100) +
                   0.2) {
      ref_point_fren.right_road_border =
          raw_ref_point.distance_to_right_obstacle - smoother_comp_l;
      ref_point_fren.right_road_border_type = 1;
    } else {
      ref_point_fren.right_road_border =
          raw_ref_point.distance_to_right_road_border - smoother_comp_l;
      ref_point_fren.right_road_border_type = 0;
    }

    ref_point_fren.left_road_border =
        std::fmin(ref_point_fren.left_road_border, 1000.0);
    ref_point_fren.right_road_border =
        std::fmin(ref_point_fren.right_road_border, 1000.0);

    refline.push_back(ref_point_fren);
  }
}

void WorldModel::feed_map_info(const MSDMapInfo &map_info) {
  double ego_v_cruise = ego_state_manager_.get_cart_ego_state().ego_v_cruise;
  double ego_vel = ego_state_manager_.get_cart_ego_state().ego_vel;
  map_info_manager_.set_map_info(map_info, shared_from_this(), ego_vel,
                                 ego_v_cruise, is_ddmap_);
}

void WorldModel::replace_refline_points(
    std::vector<Pose2D> refline_result, double cur_lane_left_border,
    double cur_lane_right_border,
    std::vector<ReferenceLinePointDerived> &refline_points) {
  if (refline_result.empty() || refline_result.size() < 3) {
    return;
  }
  ReferenceLinePointDerived last_point;
  std::vector<ReferenceLinePointDerived> current_refline_;
  current_refline_.clear();
  // auto enu2car = world_model->get_cart_ego_state_manager().get_enu2car();
  auto enu2car = get_cart_ego_state_manager().get_enu2car();
  MSD_LOG(INFO,
          "replace refline width cur_lane_left_border = %.2f, "
          "cur_lane_right_border = %.2f",
          cur_lane_left_border, cur_lane_right_border);
  cur_lane_left_border = std::max(cur_lane_left_border, 1.3);
  cur_lane_right_border = std::max(cur_lane_right_border, 1.3);
  last_point.curvature = 0.0;
  last_point.distance_to_left_road_border = cur_lane_left_border + 0.5;
  last_point.distance_to_right_road_border = cur_lane_right_border + 0.5;
  last_point.distance_to_left_lane_border = cur_lane_left_border;
  last_point.distance_to_right_lane_border = cur_lane_right_border;
  last_point.distance_to_left_obstacle = 1000.0;
  last_point.distance_to_right_obstacle = 1000.0;
  last_point.lane_width = cur_lane_left_border + cur_lane_right_border;

  for (auto refline_pose : refline_result) {
    Eigen::Vector3d car_point, enu_point;
    enu_point.x() = refline_pose.x;
    enu_point.y() = refline_pose.y;
    enu_point.z() = 0.0;
    car_point = enu2car * enu_point;
    auto point = last_point;
    point.car_point.x = car_point.x();
    point.car_point.y = car_point.y();
    point.car_point.z = car_point.z();
    point.enu_point.x = enu_point.x();
    point.enu_point.y = enu_point.y();
    point.enu_point.z = enu_point.z();
    current_refline_.push_back(point);
  }
  refline_points = current_refline_;

  // auto *planner_debug = PlanningContext::Instance()->mutable_planner_debug();
  // for (const auto p : refline_points) {
  //   std::vector<double> x_y_tmp;
  //   x_y_tmp.push_back(p.enu_point.x);
  //   x_y_tmp.push_back(p.enu_point.y);
  //   planner_debug->apf_debug_info.apf_refline_points.push_back(x_y_tmp);
  // }
  return;
}

} // namespace msquare
