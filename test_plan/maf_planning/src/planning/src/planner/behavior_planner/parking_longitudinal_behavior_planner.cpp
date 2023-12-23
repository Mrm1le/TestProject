#include "planner/behavior_planner/parking_longitudinal_behavior_planner.h"
#include "common/parking_obstacle_manager.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include <vector>
// #define USE_ASTAR true

namespace msquare {

namespace parking {

namespace {
constexpr double kSafeRemainDistance = 1.2;
constexpr double kSlowSpeedThreshold = 0.4;

constexpr double kLonConsiderDistance = 1.0;
} // namespace

ParkingLongitudinalBehaviorPlanner::ParkingLongitudinalBehaviorPlanner(
    const std::shared_ptr<WorldModel> &world_model)
    : BehaviorPlanner(world_model) {
  world_model_ = world_model;
  collision_checker_ = CollisionChecker();
  // ego_model_ = EgoModelManager();
}

ParkingLongitudinalBehaviorPlanner::~ParkingLongitudinalBehaviorPlanner() {}

bool ParkingLongitudinalBehaviorPlanner::calculate() {
  // const MapInfo &map_info = world_model_->get_map_info();
  const auto &parking_slot_info =
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->parking_slot_info;

  // 从 WorldModel 获取 state
  const FrenetState &planning_start_state =
      world_model_->get_planning_start_state();
  world_model_->update_map_boundary();
  const State &mpc_vehicle_state = world_model_->get_mpc_vehicle_state();

  // 计算 vehicle_state_;
  vehicle_state_.s_frenet = planning_start_state.s;
  vehicle_state_.r_frenet = planning_start_state.r;
  vehicle_state_.vel = mpc_vehicle_state[0];
  vehicle_state_.acc = mpc_vehicle_state[1];
  vehicle_state_.theta_error = mpc_vehicle_state[2];
  vehicle_state_.omega = 0.0; // 纵向决策中没有使用该变量

  // output ego model for visualize
  bool is_reverse =
      (PlanningContext::Instance()->planning_status().planning_result.gear ==
       GearState::REVERSE);
  auto ptr_params = collision_checker_.mutable_collision_params();
  ptr_params->update();
  double back_comp_length =
      is_reverse ? ptr_params->back_comp_r : ptr_params->back_comp_d;
  (void)collision_checker_
      .set_deviation_length(VehicleParam::Instance()->front_edge_to_center -
                            VehicleParam::Instance()->length / 2.0)
      .set_cut_length(back_comp_length)
      .set_reverse(is_reverse);

  EgoModelManager &ego_model = collision_checker_.get_ego_model();
  ego_model.set_model_type(EgoModelType::HEXADECAGON);
  PathPoint base_pt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  *PlanningContext::Instance()->mutable_full_body_model_contour() =
      ego_model.get_ego_model_polygon(EgoModelType::HEXADECAGON, base_pt)
          .points();

  // 获取 compute 的输出结果指针
  LongitudinalBehaviorPlannerOutput *output =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output();

  // debug info
  // gap time
  static auto last_gap_time = MTIME()->timestamp().ms();
  double gap_time = MTIME()->timestamp().ms() - last_gap_time;
  last_gap_time = MTIME()->timestamp().ms();
  *PlanningContext::Instance()->mutable_planning_debug_info() +=
      ", [gap]" + std::to_string(gap_time).substr(0, 5) + "ms";
  if (VehicleParam::Instance()->geometry_contour_.is_using_geometry_contour()) {
    *PlanningContext::Instance()->mutable_planning_debug_info() += ",[contour]";
  } else {
    *PlanningContext::Instance()->mutable_planning_debug_info() += ",[not contour]";
  }
  //
  std::vector<double> time_duration;
  auto last_time = MTIME()->timestamp().ms();
  double start_lead = MTIME()->timestamp().sec();
  (void)compute(output->lead_cars);
  time_duration.emplace_back(MTIME()->timestamp().ms() - last_time);
  last_time = MTIME()->timestamp().ms();
  // debug info
  const auto &lead_one = output->lead_cars.first;
  const auto &lead_obs =
      world_model_->mutable_obstacle_manager().find_obstacle(lead_one.id);
  auto ptr_debug_info =
      PlanningContext::Instance()->mutable_planning_debug_info();

  if (lead_one.id != -1) {
    bool is_collision = lead_obs->LongitudinalDecision().has_lead() &&
                        lead_obs->LongitudinalDecision().lead().is_collision;
    if (is_collision) {
      std::string ss;
      ss = "\n[leadOne]id" + std::to_string(lead_one.id) + ",type(" +
           std::to_string((int)lead_obs->SFType()) + "," +
           std::to_string((int)lead_one.type) + "),v" +
           std::to_string(lead_obs->PerceptionSpeed()).substr(0, 5) +
           ",static(" + std::to_string(lead_one.is_sf_static) + "," +
           std::to_string(lead_one.is_static) + "),slxy=(" +
           std::to_string(lead_one.d_rel).substr(0, 5) + "," +
           std::to_string(lead_one.d_path).substr(0, 5) + "," +
           std::to_string(lead_obs->PerceptionBoundingBox().center().x())
               .substr(0, 5) +
           "," +
           std::to_string(lead_obs->PerceptionBoundingBox().center().y())
               .substr(0, 5) +
           ")";
      *ptr_debug_info += ss;
    }
  }
  double after_lead = MTIME()->timestamp().sec();
  auto update_lead = after_lead - start_lead;
  // std::cout << "check_time_count: update_lead " << update_lead << std::endl;
  double start_fs = MTIME()->timestamp().sec();
  FreespacePoint lead_point_polygon;
  std::vector<std::pair<double, double>> old_mpc_sl_points;
  double side_safe_threshold = CarParams::GetInstance()->lat_inflation() * 0.5;
  bool is_sop_collsion_model =
      (parking_slot_info.type.value == ParkingSlotType::PARALLEL)
          ? (CarParams::GetInstance()
                 ->car_config.common_config.use_sop_openspace_planner_parallel)
          : (CarParams::GetInstance()
                 ->car_config.common_config.use_sop_openspace_planner);

  if (!is_sop_collsion_model)
    (void)compute(lead_point_polygon, &old_mpc_sl_points);
  time_duration.emplace_back(MTIME()->timestamp().ms() - last_time); // 2 - point
  last_time = MTIME()->timestamp().ms();
  double after_fs = MTIME()->timestamp().sec();
  auto update_fs = after_fs - start_fs;
  // std::cout << "check_time_count: update_fs " << update_fs << std::endl;

  // remain dist decider
  FreespacePoint lead_point_grid;
  bool remain_decider_status = false;
  std::vector<std::vector<std::pair<double, double>>> vec_sl_points;
  if (is_sop_collsion_model) {
    std::unique_ptr<RemainDistDecider> remain_dist_decider;
    remain_dist_decider = std::make_unique<RemainDistDecider>(world_model_);
    remain_decider_status = remain_dist_decider->MakeDecision(ego_model, 
        PlanningContext::Instance()
            ->parking_behavior_planner_output()
            .has_moved,
        &lead_point_grid, &vec_sl_points, &side_safe_threshold,
        this->mutable_out_hole_param());
  }

  time_duration.emplace_back(MTIME()->timestamp().ms() - last_time); // 3 - remain
  last_time = MTIME()->timestamp().ms();
  // collision check model switch
  if (is_sop_collsion_model) {
    output->free_space = lead_point_grid;
  } else {
    output->free_space = lead_point_polygon;
  }
  (void)MakeSpecialCase(side_safe_threshold, &output->free_space);

  freespace_point_debug(lead_point_grid, lead_point_polygon, vec_sl_points,
                        remain_decider_status);
  *PlanningContext::Instance()->mutable_vec_sl_points() = vec_sl_points;

  (void)compute(output->fs_line);
  time_duration.emplace_back(MTIME()->timestamp().ms() - last_time);
  last_time = MTIME()->timestamp().ms();
  (void)compute_multidirectional_cars_frenet(output->multidirectional_cars);
  (void)compute_multidirectional_cars_ego(output->multidirectional_cars);
  (void)compute_multidirectional_human(output->multidirectional_human);
  if (world_model_->is_parking_svp()) {
    (void)compute_intention_status_obstacles(
        output->intention_status_obstacles);
  }
  (void)compute_prediction_obstacles(output->prediction_obstacles);

  // std::to_string(MTIME()->timestamp().sec()).substr(6, 7);
  static double max_comsume_time2 = 0.0, max_comsume_time3 = 0.0;
  std::string time_duration_str = "\n[vp]dur(";
  int i = 0;
  for (const auto &it : time_duration) {
    std::string duration_ms_str = std::to_string(it).substr(0, 5);
    time_duration_str += duration_ms_str + ",";
    i++;
    if (i == 2) {
      if (it > max_comsume_time2)
        max_comsume_time2 = it;
    }
    if (i == 3) {
      if (it > max_comsume_time3)
        max_comsume_time3 = it;
    }
  }
  time_duration_str += "),max_t=(" + std::to_string(max_comsume_time2).substr(0, 5) +
                       ", " + std::to_string(max_comsume_time3).substr(0, 5) + ") ";
  *PlanningContext::Instance()->mutable_planning_debug_info() +=
      time_duration_str;

  return true;
}

bool ParkingLongitudinalBehaviorPlanner::compute(LeaderPair &lead_cars) {
  const PlanningStatus &planning_status =
      PlanningContext::Instance()->planning_status();
  const PlanningResult &planning_result = planning_status.planning_result;
  // auto lateral_output =
  // PlanningContext::Instance()->mutable_parking_lateral_behavoir_planner_output();
  obstacles_ = world_model_->obstacle_manager().get_obstacles().Items();
  const StatusType &status_type =
      PlanningContext::Instance()->planning_status().scenario.status_type;
  SLBoundary sl_boundary;
  const msquare::planning_math::Box2d &ego_box =
      world_model_->get_ego_state().ego_box;

  lead_cars.first.id = -1;
  lead_cars.second.id = -1;
  double ego_s = 0.0;
  if (planning_result.gear == GearState::REVERSE) {
    ego_s = world_model_->get_ego_state_planning().ego_frenet.x +
            VehicleParam::Instance()->back_edge_to_center;
  } else {
    ego_s = world_model_->get_ego_state_planning().ego_frenet.x +
            VehicleParam::Instance()->front_edge_to_center;
  }
  for (const auto &obstacle : obstacles_) {
    // if
    // (std::find(lateral_output->sidepass_left_ignore.begin(),lateral_output->sidepass_left_ignore.end(),obstacle->Id())
    // != lateral_output->sidepass_left_ignore.end()){
    //   sl_boundary = obstacle->PerceptionSLBoundary();
    //   std::cout <<"sidepass left ignore id = " << obstacle->Id()<<std::endl;
    // }
    // else
    // {
    //   sl_boundary = obstacle->PerceptionSLBoundaryOrigin();
    // }

    auto lon_decision = obstacle->LongitudinalDecision();

    if (lon_decision.has_lead()) {
      if (lon_decision.lead().is_collision) {
        // std::cout << "ID: " << obstacle->Id()
        // << "  sl_boundary: s & l: "<< sl_boundary.start_s << "  " <<
        // sl_boundary.end_s <<  "  " << sl_boundary.start_l <<  "  " <<
        // sl_boundary.end_l
        // << "  is_collision:  " <<  lon_decision.lead().is_collision
        // << "  distance_s: " << lon_decision.lead().distance_s << std::endl;
        double d_rel;
        d_rel = std::max(lon_decision.lead().distance_s, 0.0);
        if (d_rel < lead_cars.first.d_rel || lead_cars.first.id == -1) {
          lead_cars.second.id = lead_cars.first.id;
          lead_cars.second.d_rel = lead_cars.first.d_rel;
          lead_cars.second.is_apa_status = lead_cars.first.is_apa_status;
          lead_cars.second.is_need_fillet_cutting =
              lead_cars.first.is_need_fillet_cutting;
          lead_cars.first.id = obstacle->Id();
          lead_cars.first.d_rel = d_rel;
          lead_cars.first.is_apa_status =
              (obstacle->IsApa() &&
               (ego_box.DistanceTo(obstacle->PerceptionBoundingBox()) < 0.5));
          if (lead_cars.first.is_need_fillet_cutting == false)
            lead_cars.first.is_need_fillet_cutting =
                obstacle->IsNeedFilletCutting();

        } else if (d_rel < lead_cars.second.d_rel ||
                   lead_cars.second.id == -1) {
          lead_cars.second.id = obstacle->Id();
          lead_cars.second.d_rel = d_rel;
          lead_cars.second.is_apa_status =
              (obstacle->IsApa() &&
               (ego_box.DistanceTo(obstacle->PerceptionBoundingBox()) < 0.5));
          if (lead_cars.second.is_need_fillet_cutting == false)
            lead_cars.second.is_need_fillet_cutting =
                obstacle->IsNeedFilletCutting();
        }
      }
    }
  }
  if (lead_cars.first.id == -1) {
    return true;
  }
  auto obstacle = world_model_->mutable_obstacle_manager().find_obstacle(
      lead_cars.first.id);

  if (obstacle == nullptr) {
    return true;
  }
  // std::cout << "LeadOne ID: " << lead_cars.first.id << std::endl;
  lead_cars.first.type = obstacle->Type();
  lead_cars.first.direction = obstacle->GetPoseDirection();
  lead_cars.first.is_static = (obstacle->IsStatic() == 1);
  lead_cars.first.is_sf_static = (obstacle->IsSFStatic() == 1);
  sl_boundary = obstacle->PerceptionSLBoundaryPlanning();
  double d_path = 10.0;
  if ((sl_boundary.start_l * sl_boundary.end_l) < 0.0) {
    d_path = 0.0;
  } else {
    d_path =
        std::min(std::abs(sl_boundary.start_l), std::abs(sl_boundary.end_l));
  }
  lead_cars.first.d_path = d_path;
  lead_cars.first.yaw_relative_frenet =
      obstacle->Speed_yaw_relative_planning_frenet();
  const Pose2D lead_first_carte_pose = world_model_->mutable_obstacle_manager()
                                           .find_obstacle(lead_cars.first.id)
                                           ->GetCartePosewrtEgo();
  const Point2D lead_first_carte_vel = world_model_->mutable_obstacle_manager()
                                           .find_obstacle(lead_cars.first.id)
                                           ->GetCarteVelwrtEgo();
  if (lead_cars.first.is_static &&
      (status_type == StatusType::APA || status_type == StatusType::APOA)) {
    lead_cars.first.v_lat = 0.0;
    lead_cars.first.v_lon = 0.0;
    lead_cars.first.a_lat = 0.0;
    lead_cars.first.a_lon = 0.0;
  } else {
    if (std::hypot(lead_first_carte_pose.x, lead_first_carte_pose.y) < 4.0) {
      lead_cars.first.v_lat = lead_first_carte_vel.y;
      lead_cars.first.v_lon = lead_first_carte_vel.x;
      lead_cars.first.a_lat =
          obstacle->acceleration() * std::sin(lead_first_carte_pose.theta);
      lead_cars.first.a_lon =
          obstacle->acceleration() * std::cos(lead_first_carte_pose.theta);
    } else {
      lead_cars.first.v_lat =
          obstacle->speed() * std::sin(lead_cars.first.yaw_relative_frenet);
      lead_cars.first.v_lon =
          obstacle->speed() * std::cos(lead_cars.first.yaw_relative_frenet);
      lead_cars.first.a_lat = obstacle->acceleration() *
                              std::sin(lead_cars.first.yaw_relative_frenet);
      lead_cars.first.a_lon = obstacle->acceleration() *
                              std::cos(lead_cars.first.yaw_relative_frenet);
    }
  }
  // std::cout << "lead_cars.first.d_path" << lead_cars.first.d_path
  // <<std::endl; std::cout << "lead_cars.first.yaw_relative_frenet" <<
  // lead_cars.first.yaw_relative_frenet <<std::endl; std::cout <<
  // "lead_cars.first.v_lat" << lead_cars.first.v_lat <<std::endl; std::cout <<
  // "lead_cars.first.v_lon" << lead_cars.first.v_lon <<std::endl; std::cout <<
  // "lead_cars.first.is_need_fillet_cutting" <<
  // lead_cars.first.is_need_fillet_cutting <<std::endl;
  lead_cars.first.is_sidepass_obj = is_side_pass_obj(lead_cars.first.id);
  lead_cars.first.is_approaching_gate = obstacle->IsApproachingGate();
  lead_cars.first.is_apa_status =
      (obstacle->IsApa() &&
       (ego_box.DistanceTo(obstacle->PerceptionBoundingBox()) < 0.5));
  // TODO: use speed yaw to calculate
  if (lead_cars.second.id == -1) {
    return true;
  }

  obstacle = world_model_->mutable_obstacle_manager().find_obstacle(
      lead_cars.second.id);

  if (obstacle == nullptr) {
    return true;
  }
  // std::cout << "LeadTwo ID: " << lead_cars.second.id << std::endl;
  lead_cars.second.type = obstacle->Type();
  lead_cars.second.is_static = (obstacle->IsStatic() == 1);
  lead_cars.second.is_sf_static = (obstacle->IsSFStatic() == 1);
  lead_cars.second.direction = obstacle->GetPoseDirection();
  sl_boundary = obstacle->PerceptionSLBoundaryPlanning();
  if ((sl_boundary.start_l * sl_boundary.end_l) < 0.0) {
    d_path = 0.0;
  } else {
    d_path =
        std::min(std::abs(sl_boundary.start_l), std::abs(sl_boundary.end_l));
  }
  lead_cars.second.d_path = d_path;
  lead_cars.second.yaw_relative_frenet =
      obstacle->Speed_yaw_relative_planning_frenet();
  const Pose2D lead_second_carte_pose = world_model_->mutable_obstacle_manager()
                                            .find_obstacle(lead_cars.second.id)
                                            ->GetCartePosewrtEgo();
  const Point2D lead_second_carte_vel = world_model_->mutable_obstacle_manager()
                                            .find_obstacle(lead_cars.second.id)
                                            ->GetCarteVelwrtEgo();

  if (lead_cars.second.is_static &&
      (status_type == StatusType::APA || status_type == StatusType::APOA)) {
    lead_cars.second.v_lat = 0.0;
    lead_cars.second.v_lon = 0.0;
    lead_cars.second.a_lat = 0.0;
    lead_cars.second.a_lon = 0.0;
  } else {
    if (std::hypot(lead_second_carte_pose.x, lead_second_carte_pose.y) < 4.0) {
      lead_cars.second.v_lat = lead_second_carte_vel.y;
      lead_cars.second.v_lon = lead_second_carte_vel.x;
      lead_cars.second.a_lat =
          obstacle->acceleration() * std::sin(lead_second_carte_pose.theta);
      lead_cars.second.a_lon =
          obstacle->acceleration() * std::cos(lead_second_carte_pose.theta);
    } else {
      lead_cars.second.v_lat =
          obstacle->speed() * std::sin(lead_cars.second.yaw_relative_frenet);
      lead_cars.second.v_lon =
          obstacle->speed() * std::cos(lead_cars.second.yaw_relative_frenet);
      lead_cars.second.a_lat = obstacle->acceleration() *
                               std::sin(lead_cars.second.yaw_relative_frenet);
      lead_cars.second.a_lon = obstacle->acceleration() *
                               std::cos(lead_cars.second.yaw_relative_frenet);
    }
  }
  lead_cars.second.is_sidepass_obj = is_side_pass_obj(lead_cars.second.id);
  lead_cars.second.is_approaching_gate = obstacle->IsApproachingGate();
  lead_cars.second.is_apa_status =
      (obstacle->IsApa() &&
       (ego_box.DistanceTo(obstacle->PerceptionBoundingBox()) < 0.5));

  return true;
}

bool ParkingLongitudinalBehaviorPlanner::compute(
    FreespacePoint &lead_point,
    std::vector<std::pair<double, double>> *const ptr_old_mpc_sl_points) {
  points_ = world_model_->obstacle_manager().get_points().Items();
  bool is_reverse =
      (PlanningContext::Instance()->planning_status().planning_result.gear ==
       GearState::REVERSE);
  bool is_moved =
      PlanningContext::Instance()->parking_behavior_planner_output().has_moved;
  int zigzag_num = PlanningContext::Instance()->planning_status().zigzag_num;

  double deviation = 0.0;
  if (is_reverse) {
    deviation = 0.0;
  } else {
    deviation = VehicleParam::Instance()->front_edge_to_center;
  }

  // auto t1=std::chrono::steady_clock::now();
  int count = 0;
  const double obs_to_ego_threshold_square = 5.0 * 5.0;
  const Pose2D &ego_pose = world_model_->get_ego_state().ego_pose;
  const double rectify_x = ego_pose.x + std::cos(ego_pose.theta) * deviation;
  const double rectify_y = ego_pose.y + std::sin(ego_pose.theta) * deviation;
  const double risky_lat = CarParams::GetInstance()->lat_inflation() * 0.5;
  const double secure_lat = CarParams::GetInstance()->lat_inflation() * 0.9;
  const double risky_thres = std::max(risky_lat, 0.05);
  const double secure_thres = std::max(secure_lat, 0.05);
  const double risky_extra =
      std::min(risky_lat, CarParams::GetInstance()->lon_inflation_min - 0.1);
  const double secure_extra =
      std::min(secure_lat, CarParams::GetInstance()->lon_inflation_min - 0.1);
  bool use_secure_absolutely = (!is_reverse && zigzag_num < 1);

  double obs_to_ego_square;
  double collision_threshold;
  double extra_thres;
  double ratio;
  bool use_secure;
  ObsPtsWithId obs_pts;
  for (const auto &point : points_) {
    planning_math::Vec2d obs_p(point->point().x, point->point().y, point->Id());
    // relative in car head frame, of car rear frame
    obs_to_ego_square =
        planning_math::sum_square(obs_p.x() - rectify_x, obs_p.y() - rectify_y);
    if (obs_to_ego_square > obs_to_ego_threshold_square) {
      continue;
    }

    count++;
    use_secure =
        (use_secure_absolutely || point->Type() == ObjectType::FREESPACE);
    collision_threshold = use_secure ? secure_thres : risky_thres;
    extra_thres = use_secure ? secure_extra : risky_extra;

    obs_pts.emplace_back(collision_threshold, extra_thres,
                         CollisionCheckStatus(), obs_p, point->point().z,
                         Pose2D(0.0, 0.0, 0.0, 0.0));
  }

  const Pose2DTrajectory &plan_traj =
      PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .plan_path; // use no extend plan path
  // const Pose2DTrajectory plan_traj;
  const Pose2DTrajectory &mpc_traj =
      PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .refactored_mpc_trajectory;

  // donot consider too fast mpc trajectory
  Pose2DTrajectory filter_mpc_traj;
  for (const auto& it : mpc_traj) {
    if (it.s < kLonConsiderDistance) {
      filter_mpc_traj.emplace_back(it);
    } else {
      break;
    }
  }
  // obs_pts is obstacle points which ,
  std::string cc_debug_str;
  if(!filter_mpc_traj.empty())
    cc_debug_str += ",mpc_s" + std::to_string(filter_mpc_traj.back().s).substr(0, 5) + ", ";
  collision_checker_.remainDisCheck(obs_pts, filter_mpc_traj, plan_traj, is_reverse,
                                    is_moved, ego_pose, &lead_point,
                                    &cc_debug_str, ptr_old_mpc_sl_points);
  *PlanningContext::Instance()->mutable_planning_debug_info() += cc_debug_str;

  // output = lead_point.(x, y, d_ref = remain_dist, id, d_path=useless)
  // distribute collision result
  lead_point.id = -1;
  for (const auto &obs_result : obs_pts) {
    const CollisionCheckStatus &result = obs_result.result;
    if (!result.is_valid) {
      break;
    }
    // TODO: can de deleted ?
    Obstacle *point_ptr =
        world_model_->mutable_obstacle_manager().find_point(obs_result.p.Id());
    if (point_ptr) {
      ObjectDecisionType object_decision;
      auto mutable_lead_decision = object_decision.mutable_lead();
      mutable_lead_decision->distance_s = result.s;
      mutable_lead_decision->min_distance = result.min_distance;
      mutable_lead_decision->is_collision = result.is_collision;
      mutable_lead_decision->type = ObjectType::NOT_KNOW;
      point_ptr->AddParkingLongitudinalDecision("point_decision",
                                                object_decision);
    }

    if (!result.is_collision) {
      continue;
    }
    // double d_rel = result.s + CarParams::GetInstance()->lat_inflation() * 0.5;
    double d_rel = result.s;
    if (d_rel < lead_point.d_rel || lead_point.id == -1) {
      lead_point.d_rel = d_rel;
      lead_point.id = obs_result.p.Id();
      lead_point.s = d_rel;
      lead_point.l = result.min_distance;
      lead_point.x = obs_result.p.x();
      lead_point.y = obs_result.p.y();
      lead_point.ego_danger_location = obs_result.ego_danger_location;
      if (result.is_collision)
        lead_point.is_collision = true;
      else
        lead_point.is_collision = false;
    }
  } // end distribute collision result
  if (lead_point.id == -1)
    lead_point.is_collision = false;
  // auto t2=std::chrono::steady_clock::now();
  // double chrono_ms=std::chrono::duration<double, std::milli>(t2-t1).count();

  bool is_lead_uss = false;
  if (lead_point.id == -1) {
    return true;
  }
  Obstacle *point;
  if (lead_point.id != 20000) {
    point = world_model_->mutable_obstacle_manager().find_point(lead_point.id);
    if (point == nullptr) {
      lead_point.id = -1;
    }
  }

  double d_path = 10.0;
  lead_point.d_path = d_path;
  lead_point.x =
      is_lead_uss ? 0.0 : point->PerceptionBoundingBox().center().x();
  lead_point.y =
      is_lead_uss ? 0.0 : point->PerceptionBoundingBox().center().y();

  return true;
}

bool ParkingLongitudinalBehaviorPlanner::compute(FreespaceLine &lead_line) {
  double fs_collision_check_start = MTIME()->timestamp().sec();
  std::vector<const Obstacle *> lines_all_;
  lines_ = world_model_->obstacle_manager().get_lines().Items();
  pillars_ = world_model_->obstacle_manager().get_pillars().Items();
  road_borders_ = world_model_->obstacle_manager().get_road_borders().Items();
  gates_ = world_model_->obstacle_manager().get_gates().Items();

  lines_all_.insert(lines_all_.end(), lines_.begin(), lines_.end());
  lines_all_.insert(lines_all_.end(), pillars_.begin(), pillars_.end());
  lines_all_.insert(lines_all_.end(), road_borders_.begin(),
                    road_borders_.end());
  lines_all_.insert(lines_all_.end(), gates_.begin(), gates_.end());
  // std::cout <<"size of fs pillar border = "<<lines_.size()<<"
  // "<<pillars_.size()<<" "<<road_borders_.size()<<"
  // "<<lines_all_.size()<<std::endl;
  SLBoundary sl_boundary;

  lead_line.id = -1;

  auto path_points_real = PlanningContext::Instance()
                              ->planning_status()
                              .planning_result.path_points_real;
  // double lat_offset_start = 0.0;
  // double lat_offset_end = 0.0;

  double ego_s = 0.0;
  const StatusType &status_type =
      PlanningContext::Instance()->planning_status().scenario.status_type;
  if (PlanningContext::Instance()->planning_status().planning_result.gear ==
      GearState::REVERSE) {
    ego_s = world_model_->get_ego_state_planning().ego_frenet.x +
            VehicleParam::Instance()->back_edge_to_center;
  } else {
    ego_s = world_model_->get_ego_state_planning().ego_frenet.x +
            VehicleParam::Instance()->front_edge_to_center;
  }
  // std::cout<<"lxrdebug lonbehavior =
  // "<<world_model_->get_ego_state_planning().ego_carte.x<<","<<world_model_->get_ego_state().ego_carte.y<<std::endl;
  // double ego_l = world_model_->get_ego_state_planning().ego_frenet.y;
  const planning_math::Box2d exclusive_box =
      PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .exclusive_box;
  bool parking_lot_box_available =
      (PlanningContext::Instance()
           ->mutable_parking_behavior_planner_output()
           ->parking_lot != nullptr);
  planning_math::LineSegment2d lead_gate_edge;
  for (const auto &map_obs : lines_all_) {
    sl_boundary = map_obs->PerceptionSLBoundaryPlanning();
    // if (status_type == StatusType::AVP || status_type == StatusType::SEARCH){
    //   lat_offset_start = calc_lat_offset(path_points_real,
    //   sl_boundary.start_s); lat_offset_end =
    //   calc_lat_offset(path_points_real, sl_boundary.end_s);
    // }
    // if (((std::abs(sl_boundary.start_l) > 5.0 && std::abs(sl_boundary.end_l)
    // > 5.0) && ((sl_boundary.end_l) * (sl_boundary.start_l) > 0.0))
    //     || ((sl_boundary.start_s - ego_s) > 7.0 && (sl_boundary.end_s -
    //     ego_s) > 7.0)
    //     || ((sl_boundary.start_s - ego_s + VehicleParam::Instance()->length)
    //     < 0.0 && (sl_boundary.end_s - ego_s +
    //     VehicleParam::Instance()->length) < 0.0)) {
    //   continue;
    // }
    if (map_obs->Type() == ObjectType::NOT_KNOW ||
        map_obs->Type() == ObjectType::FREESPACE) {
      // if (status_type == StatusType::APA || status_type == StatusType::APOA)
      // {
      //   if (exclusive_box.IsPointIn(map_obs->PerceptionLine().start()) &&
      //       exclusive_box.IsPointIn(map_obs->PerceptionLine().end())) {
      //     continue;
      //   }
      //   if (parking_lot_box_available) {
      //     const planning_math::Box2d parking_lot_box =
      //         PlanningContext::Instance()
      //             ->parking_behavior_planner_output()
      //             .parking_lot->getBox();
      //     const planning_math::Box2d parking_lot_box_inf =
      //     planning_math::Box2d(
      //         parking_lot_box.center() +
      //             0.5 * planning_math::Vec2d::CreateUnitVec2d(
      //                       parking_lot_box.heading()),
      //         parking_lot_box.heading(), parking_lot_box.length() + 1.2,
      //         parking_lot_box.width() + 0.2);

      //     if (parking_lot_box_inf.IsPointIn(
      //             map_obs->PerceptionLine().start()) ||
      //         parking_lot_box_inf.IsPointIn(map_obs->PerceptionLine().end()))
      //         {
      //       continue;
      //     }
      //   }
      // }
      ObjectDecisionType map_obs_decision;
      auto mutable_map_obs_decision = map_obs_decision.mutable_lead();

      CollisionCheckStatus result =
          check_fs_line_collision(map_obs->PerceptionLine(), map_obs->LineFusionType());
      if (result.is_collision /*  && result.s > 0.0 */) {
        double d_rel =
            result.s + CarParams::GetInstance()->lat_inflation() * 0.5;

        if (d_rel < lead_line.d_rel || lead_line.id == -1) {
          lead_line.d_rel = d_rel;
          lead_line.id = map_obs->Id();
        }
      }
      mutable_map_obs_decision->distance_s = result.s;
      mutable_map_obs_decision->min_distance = result.min_distance;
      mutable_map_obs_decision->is_collision = result.is_collision;
      mutable_map_obs_decision->type = ObjectType::NOT_KNOW;
      (void)world_model_->mutable_obstacle_manager()
          .add_parking_static_obs_longitudinal_decision(
              "map_obs_decision", map_obs->Id(), map_obs_decision);
    } else if (map_obs->Type() == ObjectType::GATE) {
      ObjectDecisionType map_obs_decision;
      auto mutable_map_obs_decision = map_obs_decision.mutable_lead();
      double temp_d_rel = 100.0;
      double temp_min_distance = 100.0;
      bool temp_is_collision = false;
      for (auto &gate_edge : map_obs->PerceptionBoundingBox().GetAllEdges()) {
        CollisionCheckStatus result = check_fs_line_collision(gate_edge, map_obs->LineFusionType());
        if (result.is_collision) {
          temp_is_collision = true;
          if (result.s < temp_d_rel) {
            temp_d_rel = result.s;
            temp_min_distance = result.min_distance;
          }
          if (temp_d_rel < lead_line.d_rel || lead_line.id == -1) {
            lead_gate_edge = gate_edge;
            lead_line.d_rel = temp_d_rel;
            lead_line.id = map_obs->Id();
            lead_line.type = ObjectType::GATE;
          }
        }
      }
      mutable_map_obs_decision->distance_s = temp_d_rel;
      mutable_map_obs_decision->min_distance = temp_min_distance;
      mutable_map_obs_decision->is_collision = temp_is_collision;
      mutable_map_obs_decision->type = ObjectType::GATE;
      (void)world_model_->mutable_obstacle_manager()
          .add_parking_static_obs_longitudinal_decision(
              "map_obs_decision", map_obs->Id(), map_obs_decision);
    }
  }

  auto &parking_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;
  int special_slot_type = parking_slot_info.special_slot_type;
  double parking_slot_iou = 0.0;
  if (parking_slot_info.original_corners.size() == 4) {
    std::vector<planning_math::Vec2d> origin_corners;
    for (const auto& corner : parking_slot_info.original_corners) {
      origin_corners.emplace_back(corner.x, corner.y);
    }
    planning_math::Polygon2d overlap_polygon;
    planning_math::Polygon2d ego_polygon(world_model_->get_ego_state().ego_box);
    planning_math::Polygon2d slot_polygon(origin_corners);
    (void)ego_polygon.ComputeOverlap(slot_polygon, &overlap_polygon);
    parking_slot_iou = overlap_polygon.area() / ego_polygon.area();
  }
  if(parking_slot_info.type.value == ParkingSlotType::PARALLEL) {
    if(special_slot_type == 0 && parking_slot_iou > 0.1) {
      lead_line.d_rel -= 0.05;
    }
  }

  double fs_collision_check_end = MTIME()->timestamp().sec();

  double fs_collision_check_time =
      fs_collision_check_start - fs_collision_check_end;

  // std::cout << "fs_collision_check_time: " << fs_collision_check_time <<
  // std::endl;

  if (lead_line.id == -1) {
    return true;
  }
  Obstacle *map_obs;

  map_obs = world_model_->mutable_obstacle_manager().find_pillar(lead_line.id);
  if (map_obs == nullptr) {
    map_obs =
        world_model_->mutable_obstacle_manager().find_road_border(lead_line.id);
    if (map_obs == nullptr) {
      map_obs =
          world_model_->mutable_obstacle_manager().find_gate(lead_line.id);
      if (map_obs == nullptr) {
        map_obs =
            world_model_->mutable_obstacle_manager().find_line(lead_line.id);
      }
    }
  }

  if (map_obs == nullptr) {
    lead_line.id = -1;
    return true;
  }
  sl_boundary = map_obs->PerceptionSLBoundary();
  lead_line.s_start = sl_boundary.start_s;
  lead_line.l_start = sl_boundary.start_l;
  lead_line.s_end = sl_boundary.end_s;
  lead_line.l_end = sl_boundary.end_l;
  if (lead_line.type == ObjectType::NOT_KNOW) {
    lead_line.x_start = map_obs->PerceptionLine().start().x();
    lead_line.y_start = map_obs->PerceptionLine().start().y();
    lead_line.x_end = map_obs->PerceptionLine().end().x();
    lead_line.y_end = map_obs->PerceptionLine().end().y();
  } else if (lead_line.type == ObjectType::GATE) {
    lead_line.x_start = lead_gate_edge.start().x();
    lead_line.y_start = lead_gate_edge.start().y();
    lead_line.x_end = lead_gate_edge.end().x();
    lead_line.y_end = lead_gate_edge.end().y();
  }

  // std::cout<<"lead_line.id = "<<lead_line.id<<std::endl;
  // std::cout<<"lead_line.d_rel = "<<lead_line.d_rel<<std::endl;

  return true;
}

bool ParkingLongitudinalBehaviorPlanner::MakeSpecialCase(
    const double side_safe_threshold, FreespacePoint *const ptr_lead_point) {
  // for out_put
  auto& lead_point = *ptr_lead_point;
  double chrono_ms = -1.0;
  MSD_LOG(ERROR, "begin: %.3f %d %.3f end", lead_point.d_rel,
          lead_point.id, chrono_ms);

  auto &parking_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;
  double parking_slot_iou = 0.0;
  if (parking_slot_info.original_corners.size() == 4) {
    std::vector<planning_math::Vec2d> origin_corners;
    for (const auto& corner : parking_slot_info.original_corners) {
      origin_corners.emplace_back(corner.x, corner.y);
    }
    planning_math::Polygon2d overlap_polygon;
    planning_math::Polygon2d ego_polygon(world_model_->get_ego_state().ego_box);
    planning_math::Polygon2d slot_polygon(origin_corners);
    (void)ego_polygon.ComputeOverlap(slot_polygon, &overlap_polygon);
    parking_slot_iou = overlap_polygon.area() / ego_polygon.area();
  }

  if (VehicleParam::Instance()->car_type == "C03") {
    int bottom_line_type = parking_slot_info.bottom_line_type;
    if (bottom_line_type == 1 && parking_slot_iou > 0.25) {
      const double TTC_LENGTH_SHORTEN_FOR_WALL = 0.05;
      lead_point.d_rel += TTC_LENGTH_SHORTEN_FOR_WALL;
    }
  }

  int special_slot_type = parking_slot_info.special_slot_type;
  if(parking_slot_info.type.value == ParkingSlotType::PARALLEL) {
    if(special_slot_type == 0 && parking_slot_iou > 0.1) {
      lead_point.d_rel -= 0.05;
    }
  }

  // TODO(root): alignment to 1.5.6 lon performance
  if (true) {
  // if (parking_slot_iou > 0.25 &&
  //     std::abs(world_model_->get_ego_state().ego_vel) < kSlowSpeedThreshold) {
    lead_point.d_rel += side_safe_threshold;
    *PlanningContext::Instance()->mutable_planning_debug_info() +=
        ", in_slot-8.5cm, ";
  }

  return true;
}

bool ParkingLongitudinalBehaviorPlanner::fill_multidirectional_cars_decision(
    double v_ego) {
  obstacles_ = world_model_->obstacle_manager().get_obstacles().Items();
  // const EgoState &ego_state = world_model_->get_ego_state();

  for (auto &obstacle : obstacles_) {
    SLBoundary sl_boundary;
    sl_boundary = obstacle->PerceptionSLBoundaryPlanning();
    ObjectDecisionType multidirectional_cars_decision;
    auto mutable_multidirectional_cars_decision =
        multidirectional_cars_decision.mutable_other_cars_yield();
    if (std::abs(obstacle->speed()) > 1.5 && obstacle->Is_angle_consistent() &&
        obstacle->Is_confident()) {
      double obs_v_lat =
          obstacle->speed() * cos(obstacle->Speed_yaw_relative_frenet());
      double obs_v_lon =
          obstacle->speed() * sin(obstacle->Speed_yaw_relative_frenet());
      // obstacle->Set_TTC_pwj((sl_boundary.start_l) / obs_v_lat);
      // obstacle->Set_s_prediction(sl_boundary.start_s +obstacle.TTC_pwj *
      // obs_v_lon);
      mutable_multidirectional_cars_decision->ID = obstacle->Id();
      mutable_multidirectional_cars_decision->TTC_pwj =
          std::max(0.0, sl_boundary.start_l / obs_v_lat);
      mutable_multidirectional_cars_decision->s_prediction =
          sl_boundary.start_s +
          mutable_multidirectional_cars_decision->TTC_pwj * obs_v_lon;
      mutable_multidirectional_cars_decision->TTC_ego = std::max(
          0.0, mutable_multidirectional_cars_decision->s_prediction / v_ego);

      if (mutable_multidirectional_cars_decision->TTC_ego < 1.5 &&
          std::abs(obstacle->speed()) < 2.5 && std::abs(obs_v_lat) < 2.0) {
        mutable_multidirectional_cars_decision->decay_rate = 1.0;
      } else {
        if ((mutable_multidirectional_cars_decision->TTC_pwj <
             mutable_multidirectional_cars_decision->TTC_ego + 2.0) &&
            (mutable_multidirectional_cars_decision->s_prediction < 4.0)) {
          mutable_multidirectional_cars_decision->decay_rate = 0.0;
        } else {
          mutable_multidirectional_cars_decision->decay_rate = std::min(
              mutable_multidirectional_cars_decision->TTC_ego * 0.1, 1.0);
        }
      }
    }
    mutable_multidirectional_cars_decision->type = obstacle->Type();

    (void)world_model_->mutable_obstacle_manager()
        .add_parking_longitudinal_decision("multidirectional_cars_decision",
                                           obstacle->Id(),
                                           multidirectional_cars_decision);
  }
  return true;
}

bool ParkingLongitudinalBehaviorPlanner::compute_multidirectional_cars_frenet(
    std::vector<MultiDirectionalCars> &multidirectional_cars) {
  // frenet_coord_ = world_model_->get_frenet_coord();
  // if(frenet_coord_ == nullptr)
  // {
  //   return false;
  // }

  multidirectional_cars.clear();
  // const double PI  =3.141592653589793238463;
  const PlanningStatus &planning_status =
      PlanningContext::Instance()->planning_status();
  const PlanningResult &planning_result = planning_status.planning_result;
  const StatusType &status_type =
      PlanningContext::Instance()->planning_status().scenario.status_type;
  obstacles_ = world_model_->obstacle_manager().get_obstacles().Items();
  SLBoundary sl_boundary;
  const EgoState &ego_state = world_model_->get_ego_state();
  double v_ego = ego_state.ego_vel;
  double ego_theta = ego_state.ego_pose.theta;
  int scene_avp = planning_result.scene_avp;
  // std::cout<<"scenedebug scene_avp = "<<scene_avp<<std::endl;
  // fill_multidirectional_cars_decision(v_ego);
  // multidirectional_cars.coordinate = 1;
  // multidirectional_cars.id = -1;
  // multidirectional_cars.TTC_pwj = 10.0;
  double ego_s = 0;
  if (planning_result.gear == GearState::REVERSE) {
    ego_s = world_model_->get_ego_state_planning().ego_frenet.x +
            VehicleParam::Instance()->back_edge_to_center;
  } else {
    ego_s = world_model_->get_ego_state_planning().ego_frenet.x +
            VehicleParam::Instance()->front_edge_to_center;
  }
  // std::cout << "ego_s = " <<ego_s<<std::endl;
  // std::cout << "ego_theta = "<<ego_theta<<std::endl;
  // std::cout << "v_ego = " <<v_ego<<std::endl;
  // for(const auto &gates: gate_obstacles_){
  //   std::cout<<"gate_id = "<<gates->Id()<<std::endl;
  // }
  for (const auto &obstacle : obstacles_) {
    // std::cout << "obsdebug id : direction = " << obstacle->Id()<<" :
    // "<<obstacle->GetPoseDirection()<<std::endl;
    if (obstacle->Id() == 1016268 || obstacle->Id() == 2001144) {
      //   sl_boundary = obstacle->PerceptionSLBoundaryPlanning();
      // std::cout <<"lxrdebug : id inroad beside cross ="<<obstacle->Id()<<"
      // "<<obstacle->IsInRoad()<<" "<<obstacle->IsBesideRoad()<<"
      // "<<obstacle->IsAcrossRoadBorder()<<std::endl;
      // PseudoPredictionTrajectory prediction_traj =
      // obstacle->GetPseudoPredictionTraj(); if(prediction_traj.enable){
      //   std::cout <<"lxrdebug: id enable type ttc meets =
      //   "<<prediction_traj.id<<" "<<prediction_traj.enable<<"
      //   "<<prediction_traj.coordinate<<" "<<prediction_traj.TTC<<"
      //   "<<prediction_traj.meet_s<<std::endl;
      // }
      // for(int i = 0;i<prediction_traj.time_series.size(); i++){
      //   std::cout <<"lxrdebug: id time relative_s velo =
      //   "<<prediction_traj.id<<" "<<prediction_traj.time_series.at(i)<<"
      //   "<<prediction_traj.relative_s.at(i)<<"
      //   "<<prediction_traj.velocity.at(i)<<std::endl;
      // }
      // std::cout<<"lxrdebug: id gate apa apoa pullover = "<<obstacle->Id()<<"
      // "<<obstacle->IsApproachingGate()<<" "<<obstacle->IsApa()<<"
      // "<<obstacle->IsApoa()<<" "<<obstacle->IsPullover()<<std::endl;
      // std::cout<<"lxrdebug: id isdecel = "<<obstacle->Id()<<"
      // "<<obstacle->IsHighspeedDecel()<<std::endl; std::cout << "lxrdebug id :
      // gate = "<<obstacle->Id()<<" ,
      // "<<obstacle->IsApproachingGate()<<std::endl; sl_boundary =
      // obstacle->PerceptionSLBoundaryPlanning(); for(auto corner :
      // sl_boundary.corners){
      //   std::cout<<"lxrdebug corners = "<<corner.x<<",
      //   "<<corner.y<<std::endl;
      // }
      // std::cout<<std::endl;
      // std::cout << "lxrdebug:ego_frenet_x ego_s =
      // "<<world_model_->get_ego_state_planning().ego_frenet.x<<" "
      // <<ego_s<<std::endl;
      //   std::cout << "lxrdebug:ego_theta = "<<ego_theta<<std::endl;
      //   std::cout << "lxrdebug:v_ego = " <<v_ego<<std::endl;
      // std::cout<<"lxrdebug id fillet_cut = "<<obstacle->Id()<<"
      // "<<obstacle->IsNeedFilletCutting()<<"
      // "<<obstacle->FilletCuttingLength()<<std::endl;
      // std::cout<<"lxrdebug wrtego = "<<obstacle->GetCartePosewrtEgo().x<<"
      // "<<obstacle->GetCartePosewrtEgo().y<<"
      // "<<obstacle->GetCartePosewrtEgo().theta<<std::endl; std::cout
      // <<"lxrdebug: id  alongfrenet = "<<obstacle->Id()<<"
      // "<<obstacle->IsAlongFrenet()<<std::endl; std::cout <<
      // "lxrdebug:SLBoundary:=
      // <"<<sl_boundary.start_s<<","<<sl_boundary.end_s<<","<<sl_boundary.start_l<<","<<sl_boundary.end_l<<">"<<std::endl;
      double speed_yaw = obstacle->Speed_yaw_relative_planning_frenet();
      double obs_v_lat = obstacle->speed() * sin(speed_yaw);
      double obs_v_lon = obstacle->speed() * cos(speed_yaw);
      // std::cout << "lxrdebug:speed_yaw =
      // "<<obstacle->Speed_yaw_relative_planning_frenet()<<std::endl; std::cout
      // << "lxrdebug: direction = "<<obstacle->GetPoseDirection()<<std::endl;
      // std::cout << "lxrdebug: id : speed_tuple = "<<obstacle->Id()<<":
      // <"<<obstacle->speed()<<","<<obs_v_lat<<","<<obs_v_lon<<">"<<std::endl;
      // std::cout << "lxrdebug: id : speed_tuple = "<<obstacle->Id()<<":
      // <"<<obstacle->IsLonStaticWrtEgo()<<","<<obstacle->IsLatStaticWrtEgo()<<","<<obstacle->IsLonHighspeedWrtEgo()<<","<<obstacle->IsLonOppositeWrtEgo()<<">,<"<<obstacle->IsLonStaticWrtFrenet()<<","<<obstacle->IsLatStaticWrtFrenet()<<","<<obstacle->IsLonHighspeedWrtFrenet()<<","<<obstacle->IsLonOppositeWrtFrenet()<<">"<<std::endl;
      // std::cout << "lxrdebug: id inroad across beside bend intersection =
      // "<<obstacle->Id()<<" "<<obstacle->IsInRoad()<<"
      // "<<obstacle->IsAcrossRoadBorder()<<" "<<obstacle->IsBesideRoad()<<"
      // "<<obstacle->IsInBend()<<"
      // "<<obstacle->IsBesideIntersection()<<std::endl;
      // std::cout << "lxrdebug: id towardsside towardscenter =
      // "<<obstacle->Id()<<" "<<obstacle->IsTowardsRoadSide()<<"
      // "<<obstacle->IsTowardsRoadCenter()<<std::endl;
      //   double TTC_pwj = std::min(10.0,
      //   std::abs(std::min(std::abs(sl_boundary.start_l),std::abs(sl_boundary.end_l))
      //   / obs_v_lat)); std::cout << "lxrdebug:TTC_pwj =
      //   "<<TTC_pwj<<std::endl; double s_prediction = sl_boundary.start_s
      //   -ego_s + TTC_pwj * obs_v_lon; std::cout << "lxrdebug:s_prediction =
      //   "<<s_prediction<<std::endl; double TTC_ego = (s_prediction > 0)?
      //   (s_prediction/v_ego):100.0; std::cout << "lxrdebug:TTC_ego =
      //   "<<TTC_ego<<std::endl;
    }
    if (obstacle->Type() != ObjectType::COUPE) {
      continue;
    } else {
      sl_boundary = obstacle->PerceptionSLBoundaryPlanning();
      MultiDirectionalCars multicars_frenet;
      multicars_frenet.id = obstacle->Id();
      multicars_frenet.type = obstacle->Type();
      multicars_frenet.coordinate = 1;
      double speed_yaw = obstacle->Speed_yaw_relative_planning_frenet();
      double obs_v_lat = obstacle->speed() * sin(speed_yaw);
      double obs_v_lon = obstacle->speed() * cos(speed_yaw);
      if (!(obstacle->IsLatStaticWrtEgo() == 1 &&
            obstacle->IsLatStaticWrtFrenet() == 1) &&
          obstacle->Type() == ObjectType::COUPE &&
          obstacle->Is_angle_consistent() &&
          ((obstacle->IsBesideRoad() && obstacle->IsInBend()) ||
           (obstacle->IsInRoad() || obstacle->IsAcrossRoadBorder()) ||
           (obstacle->IsBesideRoad() &&
            std::abs(obs_v_lat) > 0.6)) && // to compensate fusion result
          obstacle->Is_confident()) {
        // std::cout<<"lxrdebug: first filter!!"<<std::endl;
        // std::cout << "obs_property_filter works!" <<std::endl;
        double TTC_pwj = 100.0; // TTC_ego=100.0;
        if (obs_v_lat * sl_boundary.start_l < 0) {
          if (std::min(std::abs(sl_boundary.start_l),
                       std::abs(sl_boundary.end_l)) -
                  VehicleParam::Instance()->width / 2.0 <
              0.0)
            continue;
          TTC_pwj =
              std::min(10.0, std::abs((std::min(std::abs(sl_boundary.start_l),
                                                std::abs(sl_boundary.end_l)) -
                                       VehicleParam::Instance()->width / 2.0) /
                                      obs_v_lat));
        }

        if (((sl_boundary.start_s + sl_boundary.end_s) / 2 - ego_s < 10) &&
            ((sl_boundary.start_s + sl_boundary.end_s) / 2 - ego_s > 0) &&
            std::abs((sl_boundary.start_l + sl_boundary.end_l) / 2) < 8) {
          if (TTC_pwj < 10.0) {
            if (std::abs(sl_boundary.end_s - sl_boundary.start_s) > 8.0) {
              continue;
            }
            multicars_frenet.id = obstacle->Id();
            multicars_frenet.type = obstacle->Type();
            multicars_frenet.TTC_pwj = TTC_pwj;
            multicars_frenet.s_prediction =
                sl_boundary.start_s - ego_s + TTC_pwj * obs_v_lon;
            multicars_frenet.TTC_ego =
                (multicars_frenet.s_prediction > 0)
                    ? ((multicars_frenet.s_prediction -
                        obstacle->PerceptionBoundingBox().width() / 2.0) /
                       v_ego)
                    : 100.0;
            if (multicars_frenet.TTC_ego < 0.0)
              continue;
            multicars_frenet.TTC_ego =
                std::max(0.0, std::min(10.0, multicars_frenet.TTC_ego));
            if (status_type == StatusType::APA ||
                status_type == StatusType::APOA) {
              multicars_frenet.decay_rate = 0.0;
            } else {
              if ((multicars_frenet.TTC_ego < 1.5 &&
                   std::abs(obstacle->speed()) < 2.0 &&
                   std::abs(obs_v_lat) < 1.5) ||
                  std::fabs(multicars_frenet.TTC_ego - 100.0) < 1e-5 ||
                  std::abs(multicars_frenet.TTC_pwj -
                           multicars_frenet.TTC_ego) > 5.0) {
                multicars_frenet.decay_rate = 1.0;
              }
              // if{
              if ((multicars_frenet.TTC_pwj < multicars_frenet.TTC_ego + 2.0) &&
                  (multicars_frenet.s_prediction < 4.0)) {
                multicars_frenet.decay_rate = 0.0;
              } else {
                multicars_frenet.decay_rate = std::max(
                    0.0, std::min(multicars_frenet.TTC_ego * 0.1, 1.0));
              }
              // }
            }
          }
        }
      }
      if (multicars_frenet.s_prediction < 20.0 &&
          multicars_frenet.TTC_ego < 10.0) {
        // std::cout << "id = " << multicars_frenet.id <<std::endl;
        // std::cout << "TTC_pwj = " << multicars_frenet.TTC_pwj <<std::endl;
        // std::cout << "TTC_ego = " << multicars_frenet.TTC_ego <<std::endl;
        // std::cout << "s_prediction = " << multicars_frenet.s_prediction
        // <<std::endl; std::cout << "decay_rate = " <<
        // multicars_frenet.decay_rate <<std::endl; std::cout << "coordinate =
        // "<<multicars_frenet.coordinate<<std::endl; std::cout <<"lxrdebug: is
        // considered"<<std::endl;
        multidirectional_cars.push_back(multicars_frenet);
      }
    }
  }
  // std::cout << "lxrdebug: size of multicars =
  // "<<multidirectional_cars.size()<<std::endl;
  return true;
}

bool ParkingLongitudinalBehaviorPlanner::compute_multidirectional_cars_ego(
    std::vector<MultiDirectionalCars> &multidirectional_cars) {
  // const double PI  =3.141592653589793238463;
  const PlanningStatus &planning_status =
      PlanningContext::Instance()->planning_status();
  const PlanningResult &planning_result = planning_status.planning_result;
  const StatusType &status_type =
      PlanningContext::Instance()->planning_status().scenario.status_type;
  obstacles_ = world_model_->obstacle_manager().get_obstacles().Items();
  SLBoundary sl_boundary;
  const EgoState &ego_state = world_model_->get_ego_state_planning();
  double v_ego = ego_state.ego_vel;
  double ego_x = ego_state.ego_pose.x;
  double ego_y = ego_state.ego_pose.y;
  double ego_theta = ego_state.ego_pose.theta;

  double ego_s = 0;
  if (planning_result.gear == GearState::REVERSE) {
    ego_s = world_model_->get_ego_state_planning().ego_frenet.x +
            VehicleParam::Instance()->back_edge_to_center;
    ego_x = world_model_->get_ego_state_planning().ego_carte.x -
            cos(ego_theta) * VehicleParam::Instance()->back_edge_to_center;
    ego_y = world_model_->get_ego_state_planning().ego_carte.y -
            sin(ego_theta) * VehicleParam::Instance()->back_edge_to_center;
  } else {
    ego_s = world_model_->get_ego_state_planning().ego_frenet.x +
            VehicleParam::Instance()->front_edge_to_center;
    ego_x = world_model_->get_ego_state_planning().ego_carte.x +
            cos(ego_theta) * VehicleParam::Instance()->front_edge_to_center;
    ego_y = world_model_->get_ego_state_planning().ego_carte.y +
            sin(ego_theta) * VehicleParam::Instance()->front_edge_to_center;
  }
  // std::cout << "lxrdebug: front&back =
  // "<<VehicleParam::Instance()->front_edge_to_center<<","<<VehicleParam::Instance()->back_edge_to_center
  // <<std::endl; std::cout << "lxrdebug: ego_center =
  // "<<world_model_->get_ego_state().ego_carte.x<<","<<world_model_->get_ego_state().ego_carte.y
  // <<std::endl; std::cout << "lxrdebug: ego_theta = "<<ego_theta <<std::endl;
  // std::cout << "lxrdebug: ego_position = "<<ego_x<<","<<ego_y <<std::endl;
  for (const auto &obstacle : obstacles_) {
    if (obstacle->Type() != ObjectType::COUPE) {
      continue;
    } else {
      if ((planning_result.gear == GearState::REVERSE &&
           obstacle->GetCartePosewrtEgo().x > 0) ||
          (planning_result.gear == GearState::DRIVE &&
           obstacle->GetCartePosewrtEgo().x < 0)) {
        continue;
      }
      MultiDirectionalCars multicars_ego;
      multicars_ego.id = obstacle->Id();
      multicars_ego.type = obstacle->Type();
      multicars_ego.coordinate = 2;
      const Pose2D &obs_pose = obstacle->GetCartePosewrtEgo();
      const Point2D &obs_vel = obstacle->GetCarteVelwrtEgo();
      sl_boundary = obstacle->PerceptionSLBoundaryPlanning();
      planning_math::Box2d obs_bbox = obstacle->PerceptionBoundingBox();
      double obs_center_x = obs_pose.x;
      double obs_center_y = obs_pose.y;
      double speed_yaw_relative_ego = obstacle->Speed_yaw_relative_ego();
      double obs_v_lat = obs_vel.y;
      double obs_v_lon = obs_vel.x;

      if (obstacle->Type() == ObjectType::COUPE &&
          obstacle->Is_angle_consistent() &&
          // !obstacle->IsBesideRoad() &&
          !(obstacle->IsLatStaticWrtEgo() == 1 &&
            obstacle->IsLatStaticWrtFrenet() == 1)) {
        double TTC_pwj = 100.0; // TTC_ego = 100.0;
        if (obs_center_y * obs_v_lat < 0) {
          if ((std::abs(obs_center_y +
                        sin(speed_yaw_relative_ego) *
                            obstacle->PerceptionBoundingBox().length() / 2.0 -
                        cos(speed_yaw_relative_ego) *
                            obstacle->PerceptionBoundingBox().width() / 2.0) -
               VehicleParam::Instance()->width / 2.0) < 0.0)
            continue;
          TTC_pwj = std::min(
              10.0,
              std::abs(
                  (std::abs(
                       obs_center_y +
                       sin(speed_yaw_relative_ego) *
                           obstacle->PerceptionBoundingBox().length() / 2.0 -
                       cos(speed_yaw_relative_ego) *
                           obstacle->PerceptionBoundingBox().width() / 2.0) -
                   VehicleParam::Instance()->width / 2.0) /
                  obs_v_lat));
        }
        if (std::abs(obs_center_x) < 10 && std::abs(obs_center_y) < 10) {
          if (TTC_pwj < 10.0) {
            multicars_ego.id = obstacle->Id();
            multicars_ego.type = obstacle->Type();
            multicars_ego.TTC_pwj = TTC_pwj;
            multicars_ego.s_prediction = obs_center_x + TTC_pwj * obs_v_lon;
            multicars_ego.TTC_ego =
                (multicars_ego.s_prediction > 0)
                    ? ((multicars_ego.s_prediction -
                        obstacle->PerceptionBoundingBox().width() / 2.0) /
                       v_ego)
                    : 100.0;
            if (multicars_ego.TTC_ego < 0.0)
              continue;
            multicars_ego.TTC_ego =
                std::max(0.0, std::min(10.0, multicars_ego.TTC_ego));
            if (status_type == StatusType::APA ||
                status_type == StatusType::APOA) {
              multicars_ego.decay_rate = 0.0;
            } else {
              if ((multicars_ego.TTC_ego < 1.5 &&
                   std::abs(obstacle->speed()) < 2.0 &&
                   std::abs(obs_v_lat) < 1.5) ||
                  std::fabs(multicars_ego.TTC_ego - 100.0) < 1e-4 ||
                  std::abs(multicars_ego.TTC_pwj - multicars_ego.TTC_ego) >
                      5.0) {
                multicars_ego.decay_rate = 1.0;
              }
              // else{
              if ((multicars_ego.TTC_pwj < multicars_ego.TTC_ego + 2.0) &&
                  (multicars_ego.s_prediction < 4.0)) {
                multicars_ego.decay_rate = 0.0;
              } else {
                multicars_ego.decay_rate =
                    std::max(0.0, std::min(multicars_ego.TTC_ego * 0.1, 1.0));
              }
              // }
            }
          }
        }
      }
      if (multicars_ego.s_prediction < 20.0 && multicars_ego.TTC_ego < 10.0) {
        // std::cout<<"lat_dist = "<<obs_center_y +
        // sin(obs_pose.theta)*obstacle->PerceptionBoundingBox().length()/2.0-
        // cos(obs_pose.theta)*obstacle->PerceptionBoundingBox().width()/2.0 -
        // VehicleParam::Instance()->width/2.0<<std::endl; std::cout <<
        // "multicars_ego.s_prediction=
        // "<<multicars_ego.s_prediction<<std::endl; std::cout <<
        // "multicars_ego.TTC_ego= "<<multicars_ego.TTC_ego<<std::endl;
        // std::cout << "multicars_ego.decay_rate=
        // "<<multicars_ego.decay_rate<<std::endl; std::cout <<
        // "multicars_ego.coordinate= "<<multicars_ego.coordinate<<std::endl;
        multidirectional_cars.push_back(multicars_ego);
      }
    }
  }
  return true;
}

bool ParkingLongitudinalBehaviorPlanner::compute_multidirectional_human(
    std::vector<MultiDirectionalHuman> &multidirectional_human) {
  multidirectional_human.clear();
  // const double PI  =3.141592653589793238463;
  const PlanningStatus &planning_status =
      PlanningContext::Instance()->planning_status();
  const PlanningResult &planning_result = planning_status.planning_result;
  // const StatusType &status_type =
  // PlanningContext::Instance()->planning_status().scenario.status_type;
  obstacles_ = world_model_->obstacle_manager().get_obstacles().Items();
  SLBoundary sl_boundary;
  const EgoState &ego_state = world_model_->get_ego_state();
  double v_ego = ego_state.ego_vel;
  double ego_theta = ego_state.ego_pose.theta;

  double ego_s = 0;
  if (planning_result.gear == GearState::REVERSE) {
    ego_s = world_model_->get_ego_state_planning().ego_frenet.x +
            VehicleParam::Instance()->back_edge_to_center;
  } else {
    ego_s = world_model_->get_ego_state_planning().ego_frenet.x +
            VehicleParam::Instance()->front_edge_to_center;
  }
  // std::cout << "ego_s = " <<ego_s<<std::endl;
  // std::cout << "ego_theta = "<<ego_theta<<std::endl;
  // std::cout << "v_ego = " <<v_ego<<std::endl;
  for (const auto &obstacle : obstacles_) {
    if (obstacle->Type() != ObjectType::PEDESTRIAN) {
      continue;
    } else {
      MultiDirectionalHuman multihuman;
      // multihuman.id = obstacle->Id();
      // multihuman.type = obstacle->Type();
      sl_boundary = obstacle->PerceptionSLBoundaryPlanning();

      double speed_yaw = obstacle->Speed_yaw_relative_planning_frenet();
      double obs_v_lat = obstacle->speed() * sin(speed_yaw);
      double obs_v_lon = obstacle->speed() * cos(speed_yaw);
      const Pose2D &obs_pose = obstacle->GetCartePosewrtEgo();
      const Point2D &obs_vel = obstacle->GetCarteVelwrtEgo();
      if (obstacle->speed() > 0.5 && obstacle->speed() < 3.0) {
        if (obs_v_lat * sl_boundary.start_l < 0) {
          if ((sl_boundary.start_s + sl_boundary.end_s) / 2 - ego_s < 10.0 &&
              (sl_boundary.start_s + sl_boundary.end_s) / 2 - ego_s > 7.0 &&
              obs_v_lon < -0.5) {
            multihuman.id = obstacle->Id();
            multihuman.type = obstacle->Type();
            multihuman.decel_level = 1;
          } else if ((sl_boundary.start_s + sl_boundary.end_s) / 2 - ego_s <
                         7.0 &&
                     (sl_boundary.start_s + sl_boundary.end_s) / 2 - ego_s >
                         5.0 &&
                     obs_v_lon < 1.0) {
            multihuman.id = obstacle->Id();
            multihuman.type = obstacle->Type();
            multihuman.decel_level = 2;
          } else if ((sl_boundary.start_s + sl_boundary.end_s) / 2 - ego_s <
                         5.0 &&
                     (sl_boundary.start_s + sl_boundary.end_s) / 2 - ego_s >
                         0.0 &&
                     std::abs((sl_boundary.start_l + sl_boundary.end_l) / 2) <
                         2.5) {
            multihuman.id = obstacle->Id();
            multihuman.type = obstacle->Type();
            multihuman.decel_level = 3;
          }
          // std::cout << "PEDESTRIAN decel level = " << multihuman.decel_level
          // << std::endl;
        }
        if (obs_pose.y * obs_vel.y < 0) {
          double obs_dist =
              std::sqrt(obs_pose.x * obs_pose.x + obs_pose.y * obs_pose.y);
          if (obs_pose.x > 4.0 && obs_pose.x < 7.0 &&
              std::abs(obs_pose.y) < 6.0 && obstacle->speed() > 0.8 &&
              obs_vel.y < 0.0) {
            multihuman.id = obstacle->Id();
            multihuman.type = obstacle->Type();
            multihuman.decel_level = std::max(2, multihuman.decel_level);
          } else if (obs_pose.x > 0.0 && obs_pose.x < 4.0 &&
                     std::abs(obs_pose.y) < 5.0 && obstacle->speed() > 0.5 &&
                     obs_vel.y < 0.0) {
            multihuman.id = obstacle->Id();
            multihuman.type = obstacle->Type();
            multihuman.decel_level = std::max(3, multihuman.decel_level);
          }
        }
        if (multihuman.decel_level != 0) {
          multidirectional_human.push_back(multihuman);
        }
      }
    }
  }
  return true;
}

bool ParkingLongitudinalBehaviorPlanner::compute_intention_status_obstacles(
    std::vector<IntentionStatusObstacles> &intention_status_obstacles) {
  intention_status_obstacles.clear();
  obstacles_ = world_model_->obstacle_manager().get_obstacles().Items();
  for (const auto &obstacle : obstacles_) {
    if (obstacle->IsApa() || obstacle->IsApoa() ||
        obstacle->IsHighspeedDecel() || obstacle->IsPullover()) {
      IntentionStatusObstacles intention_obs;
      intention_obs.id = obstacle->Id();
      if (obstacle->IsApa()) {
        intention_obs.type = IntentionObsType::APA;
      }
      if (obstacle->IsApoa()) {
        intention_obs.type = IntentionObsType::APOA;
      }
      if (obstacle->IsHighspeedDecel()) {
        if (obstacle->IsPullover()) {
          intention_obs.type = IntentionObsType::PULLOVER;
        } else {
          intention_obs.type = IntentionObsType::HIGHSPEEDDECEL;
        }
      }
      // std::cout <<"id type"<<intention_obs.id<<" "<<(intention_obs.type ==
      // IntentionObsType::APA)<<(intention_obs.type ==
      // IntentionObsType::APOA)<<(intention_obs.type ==
      // IntentionObsType::PULLOVER)<<(intention_obs.type ==
      // IntentionObsType::HIGHSPEEDDECEL)<<std::endl;
      intention_status_obstacles.push_back(intention_obs);
    }
  }

  return true;
}

bool ParkingLongitudinalBehaviorPlanner::compute_prediction_obstacles(
    std::vector<int> &prediction_obstacles) {
  prediction_obstacles.clear();
  obstacles_ = world_model_->obstacle_manager().get_obstacles().Items();
  for (const auto &obstacle : obstacles_) {
    if (obstacle->GetPseudoPredictionTraj().enable == true) {
      prediction_obstacles.emplace_back(obstacle->GetPseudoPredictionTraj().id);
    } else
      continue;
  }
  return true;
}

double ParkingLongitudinalBehaviorPlanner::calc_lat_offset(
    std::vector<PathPoint> path_points_real, const double s) {
  double lat_offset = 0.0;
  if (path_points_real.size() == 0) {
    return lat_offset;
  } else if (s < path_points_real.front().s) {
    lat_offset = path_points_real.front().l;
    return lat_offset;
  } else if (s > path_points_real.back().s) {
    lat_offset = path_points_real.back().l;
    return lat_offset;
  }
  for (int i = 0; i < (int)path_points_real.size() - 1; i++) {
    if (s >= path_points_real.at(i).s && s <= path_points_real.at(i + 1).s) {
      double k = (s - path_points_real.at(i).s) /
                 (path_points_real.at(i + 1).s - path_points_real.at(i).s);
      lat_offset =
          path_points_real.at(i).l +
          (path_points_real.at(i + 1).l - path_points_real.at(i).l) * k;
    }
  }
  return lat_offset;
}

CollisionCheckStatus
ParkingLongitudinalBehaviorPlanner::check_fs_point_collision(
    const planning_math::Vec2d &fs_point, double ratio) {
  auto scene_avp =
      PlanningContext::Instance()->planning_status().planning_result.scene_avp;
  planning_math::Box2d ego_box;
  std::vector<Pose2D> trajectory = PlanningContext::Instance()
                                       ->longitudinal_behavior_planner_output()
                                       .trajectory;
  std::vector<Pose2D> mpc_trajectory =
      PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .mpc_trajectory;
  CollisionCheckStatus result;
  const bool reverse =
      PlanningContext::Instance()->planning_status().planning_result.gear ==
      GearState::REVERSE;
  const StatusType &status_type =
      PlanningContext::Instance()->planning_status().scenario.status_type;
  double back_comp_scaler = reverse ? -1.0 : 1.0;
  // double back_comp_length = - VehicleParam::Instance()->front_edge_to_center
  // + VehicleParam::Instance()->length;
  double back_comp_length =
      reverse ? 0.0
              : -VehicleParam::Instance()->front_edge_to_center +
                    VehicleParam::Instance()->length;
  double comp_length = VehicleParam::Instance()->front_edge_to_center -
                       VehicleParam::Instance()->length / 2.0 +
                       back_comp_length * back_comp_scaler / 2.0;
  double ego_x = world_model_->get_ego_state().ego_pose.x;
  double ego_y = world_model_->get_ego_state().ego_pose.y;
  double ego_theta = world_model_->get_ego_state().ego_pose.theta;
  double center_x;
  double center_y;
  center_x = ego_x + cos(ego_theta) * (comp_length);
  center_y =
      ego_y + sin(world_model_->get_ego_state().ego_pose.theta) * (comp_length);

  ego_box =
      planning_math::Box2d(planning_math::Vec2d(center_x, center_y), ego_theta,
                           VehicleParam::Instance()->length - back_comp_length,
                           VehicleParam::Instance()->width);
  double collision_threshold =
      (status_type == StatusType::APA || status_type == StatusType::APOA ||
       PlanningContext::Instance()
               ->parking_behavior_planner_output()
               .planner_type == PlannerType::OPENSPACE)
          ? CarParams::GetInstance()->lat_inflation() * ratio
          : 0.1;

  collision_threshold = std::max(collision_threshold, 0.05);
  // MSD_LOG(WARN, "collision_threshold = %f.", collision_threshold);
  // EgoModelManager &ego_model_manager = collision_checker_.get_ego_model();

  // mpc
  EgoModelManager &ego_model_ = collision_checker_.get_ego_model();
  (void)collision_checker_
      .set_deviation_length(VehicleParam::Instance()->front_edge_to_center -
                            VehicleParam::Instance()->length / 2.0)
      .set_cut_length(back_comp_length)
      .set_reverse(reverse);
  if (ego_box.DistanceTo(fs_point) > 6.0) {
    (void)ego_model_.set_model_type(EgoModelType::ORIGIN);
  } else {
    if (PlanningContext::Instance()->planning_status().scenario.status_type ==
            StatusType::APA ||
        PlanningContext::Instance()->planning_status().scenario.status_type ==
            StatusType::APOA ||
        PlanningContext::Instance()->planning_status().scenario.status_type ==
            StatusType::RPA_STRAIGHT) {
      (void)ego_model_.set_model_type(EgoModelType::HEXADECAGON);
    } else if (PlanningContext::Instance()->has_scene(
                   scene_avp, ParkingSceneType::SCENE_TURN) ||
               PlanningContext::Instance()->has_scene(
                   scene_avp, ParkingSceneType::SCENE_ENTRANCE) ||
               PlanningContext::Instance()->has_scene(
                   scene_avp, ParkingSceneType::SCENE_SIDEPASS)) {
      (void)ego_model_.set_model_type(EgoModelType::HEXADECAGON);

    } else {
      (void)ego_model_.set_model_type(EgoModelType::POLYGON);
    }
  }

  if (!PlanningContext::Instance()
           ->parking_behavior_planner_output()
           .has_moved) {
    result = collision_checker_.collision_check(
        world_model_->get_ego_state().ego_pose, fs_point,
        CarParams::GetInstance()->lat_inflation());
    if (result.is_collision) {
      return result;
    }
  }

  result = collision_checker_.collision_check(mpc_trajectory, fs_point,
                                              collision_threshold);

  if (result.is_collision) {
    return result;
  }

  // planning
  // collision_checker_.set_params(VehicleParam::Instance()->front_edge_to_center
  // - VehicleParam::Instance()->length / 2.0, back_comp_length, reverse);

  result = collision_checker_.collision_check(trajectory, fs_point,
                                              collision_threshold);
  // std::cout << "planning: " << result.min_distance << ", " << result.s << ",
  // " << result.is_collision << std::endl;

  return result;
}

CollisionCheckStatus
ParkingLongitudinalBehaviorPlanner::check_fs_point_collision(
    const std::vector<Pose2D> trajectory,
    const planning_math::Vec2d &fs_point) {
  auto scene_avp =
      PlanningContext::Instance()->planning_status().planning_result.scene_avp;
  planning_math::Box2d ego_box;
  CollisionCheckStatus result;
  const bool reverse =
      PlanningContext::Instance()->planning_status().planning_result.gear ==
      GearState::REVERSE;
  const StatusType &status_type =
      PlanningContext::Instance()->planning_status().scenario.status_type;
  double back_comp_scaler = reverse ? -1.0 : 1.0;
  // double back_comp_length = - VehicleParam::Instance()->front_edge_to_center
  // + VehicleParam::Instance()->length;
  double back_comp_length =
      reverse ? 0.0
              : -VehicleParam::Instance()->front_edge_to_center +
                    VehicleParam::Instance()->length;
  double comp_length = VehicleParam::Instance()->front_edge_to_center -
                       VehicleParam::Instance()->length / 2.0 +
                       back_comp_length * back_comp_scaler / 2.0;
  double ego_x = world_model_->get_ego_state().ego_pose.x;
  double ego_y = world_model_->get_ego_state().ego_pose.y;
  double ego_theta = world_model_->get_ego_state().ego_pose.theta;
  double center_x;
  double center_y;
  center_x = ego_x + cos(ego_theta) * (comp_length);
  center_y =
      ego_y + sin(world_model_->get_ego_state().ego_pose.theta) * (comp_length);

  ego_box =
      planning_math::Box2d(planning_math::Vec2d(center_x, center_y), ego_theta,
                           VehicleParam::Instance()->length - back_comp_length,
                           VehicleParam::Instance()->width);
  double collision_threshold =
      CarParams::GetInstance()->lat_inflation_min / 2.0;

  EgoModelManager &ego_model_ = collision_checker_.get_ego_model();
  (void)collision_checker_
      .set_deviation_length(VehicleParam::Instance()->front_edge_to_center -
                            VehicleParam::Instance()->length / 2.0)
      .set_cut_length(back_comp_length)
      .set_reverse(reverse);

  (void)ego_model_.set_model_type(EgoModelType::HEXADECAGON);
  // ego_model_.set_model_type(EgoModelType::POLYGON);

  if (!trajectory.empty()) {
    result = collision_checker_.collision_check(trajectory, fs_point,
                                                collision_threshold);
  } else {
    std::vector<Pose2D> mpc_trajectory =
        PlanningContext::Instance()
            ->longitudinal_behavior_planner_output()
            .mpc_trajectory;
    result = collision_checker_.collision_check(mpc_trajectory, fs_point,
                                                collision_threshold);
  }

  return result;
}

CollisionCheckStatus
ParkingLongitudinalBehaviorPlanner::check_fs_line_collision(
    const planning_math::LineSegment2d &fs_line, const GroundLineType fusion_type) {
  auto scene_avp =
      PlanningContext::Instance()->planning_status().planning_result.scene_avp;
  planning_math::Box2d ego_box;
  std::vector<Pose2D> trajectory = PlanningContext::Instance()
                                       ->longitudinal_behavior_planner_output()
                                       .trajectory;
  std::vector<Pose2D> mpc_trajectory =
      PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .mpc_trajectory;
  double center_x;
  double center_y;
  CollisionCheckStatus result;
  const bool reverse =
      PlanningContext::Instance()->planning_status().planning_result.gear ==
      GearState::REVERSE;
  const StatusType &status_type =
      PlanningContext::Instance()->planning_status().scenario.status_type;
  double back_comp_scaler = reverse ? -1.0 : 1.0;
  // double back_comp_length = - VehicleParam::Instance()->front_edge_to_center
  // + VehicleParam::Instance()->length;
  double back_comp_length =
      reverse ? 0.0
              : -VehicleParam::Instance()->front_edge_to_center +
                    VehicleParam::Instance()->length;
  double comp_length = VehicleParam::Instance()->front_edge_to_center -
                       VehicleParam::Instance()->length / 2.0 +
                       back_comp_length * back_comp_scaler / 2.0;
  double ego_x = world_model_->get_ego_state().ego_pose.x;
  double ego_y = world_model_->get_ego_state().ego_pose.y;
  double ego_theta = world_model_->get_ego_state().ego_pose.theta;
  // calc current dist to obstacle and determin the collision_threshold
  center_x = ego_x + cos(ego_theta) * (comp_length);
  center_y =
      ego_y + sin(world_model_->get_ego_state().ego_pose.theta) * (comp_length);

  ego_box =
      planning_math::Box2d(planning_math::Vec2d(center_x, center_y), ego_theta,
                           VehicleParam::Instance()->length - back_comp_length,
                           VehicleParam::Instance()->width);
  double collision_threshold =
      (status_type == StatusType::APA || status_type == StatusType::APOA ||
       PlanningContext::Instance()
               ->parking_behavior_planner_output()
               .planner_type == PlannerType::OPENSPACE)
          ? CarParams::GetInstance()->lat_inflation() * 0.5
          : 0.1;
  collision_threshold = std::max(
      std::min(ego_box.DistanceTo(fs_line) - 0.01, collision_threshold), 0.05);

  // mpc
  EgoModelManager &ego_model_ = collision_checker_.get_ego_model();
  (void)collision_checker_
      .set_deviation_length(VehicleParam::Instance()->front_edge_to_center -
                            VehicleParam::Instance()->length / 2.0)
      .set_cut_length(back_comp_length)
      .set_reverse(reverse);

  // ego_model_.set_params(VehicleParam::Instance()->front_edge_to_center -
  //                           VehicleParam::Instance()->length / 2.0,
  //                       back_comp_length, reverse);

  if (PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .planner_type == PlannerType::OPENSPACE) {
    if (fusion_type == GroundLineType::GROUND_LINE_USS_TYPE_STEP) {
      MSD_LOG(ERROR, "[%s] low line fusion type: %d", __FUNCTION__, (int)fusion_type);
      (void)ego_model_.set_model_type(EgoModelType::WHEEL_BASE);
    } else {
      (void)ego_model_.set_model_type(EgoModelType::POLYGON);
    }
  } else {
    (void)ego_model_.set_model_type(EgoModelType::ORIGIN);
  }
  // collision_checker_.set_params(VehicleParam::Instance()->front_edge_to_center
  // - VehicleParam::Instance()->length / 2.0, back_comp_length, true,
  // EgoModelType::TETRADECAGON);

  result = collision_checker_.collision_check(mpc_trajectory, fs_line,
                                              collision_threshold);
  // auto ego_model  = result.ego_model_;
  // std::cout<<"line ego model size = "<<ego_model.num_points()<<std::endl;
  // std::cout << "mpc: " << result.min_distance << ", " << result.s << ", " <<
  // result.is_collision << std::endl;
  if (result.is_collision) {
    return result;
  }
  // planning
  // collision_checker_.set_params(VehicleParam::Instance()->front_edge_to_center
  // - VehicleParam::Instance()->length / 2.0, back_comp_length, reverse);

  // collision_checker_.set_params(VehicleParam::Instance()->front_edge_to_center
  // - VehicleParam::Instance()->length / 2.0, back_comp_length, true,
  // EgoModelType::TETRADECAGON);

  result = collision_checker_.collision_check(trajectory, fs_line,
                                              collision_threshold);
  // std::cout << "planning: " << result.min_distance << ", " << result.s << ",
  // " << result.is_collision << std::endl;

  return result;
}

bool ParkingLongitudinalBehaviorPlanner::is_side_pass_obj(const int id) {
  std::string traj_tag = PlanningContext::Instance()
                             ->parking_lateral_behavoir_planner_output()
                             .traj_tag;
  auto lateral_planning_info = PlanningContext::Instance()
                                   ->parking_lateral_behavoir_planner_output()
                                   .lateral_planning_info;
  if (lateral_planning_info.find(traj_tag) == lateral_planning_info.end()) {
    // std::cout << "Error: lateral_planning_info can not find the key: "  <<
    // traj_tag << std::endl;
    return false;
  }
  auto sidepass_obj_list = lateral_planning_info.at(traj_tag).sidepass_obj;
  for (auto &obj : sidepass_obj_list) {
    if (obj.first == id) {
      return true;
    }
  }
  return false;
}

void ParkingLongitudinalBehaviorPlanner::freespace_point_debug(
    const msquare::parking::FreespacePoint &lead_point_grid,
    const msquare::parking::FreespacePoint &lead_point_polygon,
    std::vector<std::vector<std::pair<double, double>>> &vec_sl_points,
    const bool remain_decider_status) {
  // check trajectory kappa
  const Pose2DTrajectory &plan_traj =
      PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .plan_path; // use no extend plan path
  const Pose2DTrajectory &planning_path =
      PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .trajectory; // use no extend plan path
  const Pose2DTrajectory &mpc_traj =
      PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .refactored_mpc_trajectory;

  if (!plan_traj.empty()) {
    // get plan path's max kappa
    //std::vector<std::pair<double, double>> vec_s_kappa;
    std::pair<double, double> max_s_kappa = std::make_pair(0.0, 0.0);
    for (const auto &pt : planning_path) {
      double abs_kappa = std::abs(pt.kappa());
      if (abs_kappa > max_s_kappa.second) {
        max_s_kappa = std::make_pair(pt.const_s(), pt.kappa());
      }
      //vec_s_kappa.emplace_back(std::make_pair(pt.const_s(), pt.kappa()));
    }
    Pose2D final_path_point = plan_traj.size() > 3
                                  ? plan_traj.at(plan_traj.size() - 3)
                                  : plan_traj.back();

    if (!mpc_traj.empty()) {
      // check mpc init point
      double diff_between_ego_mpc = std::sqrt(
          (world_model_->get_ego_state().ego_pose.x - mpc_traj.front().x) *
              (world_model_->get_ego_state().ego_pose.x - mpc_traj.front().x) +
          (world_model_->get_ego_state().ego_pose.y - mpc_traj.front().y) *
              (world_model_->get_ego_state().ego_pose.y - mpc_traj.front().y));
      if (diff_between_ego_mpc > 0.05) {
        *PlanningContext::Instance()->mutable_planning_debug_info() +=
            "[danger!]mpc_diff=" +
            std::to_string(diff_between_ego_mpc).substr(0, 5) + ", ";
      }
    }

    *PlanningContext::Instance()->mutable_planning_debug_info() +=
        "\n[vp]egoV=" +
        std::to_string(world_model_->get_ego_state().ego_vel).substr(0, 5) +
        ",gear" +
        std::to_string((int)PlanningContext::Instance()
                           ->planning_status()
                           .planning_result.gear) +
        ",pp" + std::to_string(plan_traj.size()) + ">3,s" +
        std::to_string(final_path_point.s).substr(0, 5) + ",len" +
        std::to_string(PlanningContext::Instance()
                           ->longitudinal_behavior_planner_output()
                           .traj_length)
            .substr(0, 5) +
        ", mpc" + std::to_string(mpc_traj.size()) + "max(s" +
        std::to_string(max_s_kappa.first).substr(0, 5) + ",kap" +
        std::to_string(max_s_kappa.second).substr(0, 5) + "), ";
  }

  // debug1
  const auto &planning_result =
      PlanningContext::Instance()->planning_status().planning_result;
  const auto &target_speed = planning_result.v_target;
  static bool start_record_remian = false, prev_start_record_remain = false;
  static std::string start_time;
  static vector<double> old_remain, new_remain;
  if (std::abs(target_speed) > 0.01) {
    start_record_remian = true;
  }
  if (start_record_remian) {
    if (!prev_start_record_remain) {
      start_time = std::to_string(MTIME()->timestamp().sec()).substr(6, 7);
    }
    prev_start_record_remain = start_record_remian;
    if (lead_point_polygon.id >= 0) {
      old_remain.emplace_back(lead_point_polygon.s);
    } else
      old_remain.emplace_back(kSafeRemainDistance);
    new_remain.emplace_back(lead_point_grid.s);

    // std::cout << "remain record\n";
    // std::cout << "start_time = " << start_time << "\n\n";
    // std::cout << "array1 = [";
    // for (size_t i = 0; i < old_remain.size(); ++i) {
    //   std::cout << old_remain.at(i) << ", ";
    // }
    // std::cout << "] \n\narray2 = [";
    // for (size_t i = 0; i < new_remain.size(); ++i) {
    //   std::cout << new_remain.at(i) << ", ";
    // }
    // std::cout << "]\n";
  }

  std::string remain_ss1;
  if (lead_point_polygon.is_collision) {
    remain_ss1 =
        "\n[vp debug] compare d_rel. \npolygon(is_coll, id,s,l,x,y)=(" +
        std::to_string(lead_point_polygon.id) + ", " +
        std::to_string(lead_point_polygon.s).substr(0, 5) + ", " +
        std::to_string(lead_point_polygon.l).substr(0, 5) + ", " +
        std::to_string(lead_point_polygon.x).substr(0, 5) + ", " +
        std::to_string(lead_point_polygon.y).substr(0, 5) + ", " +
        std::to_string(lead_point_polygon.d_rel).substr(0, 5) + ")";
                 std::to_string(lead_point_polygon.d_rel).substr(0, 5) + ")";
  }
  *PlanningContext::Instance()->mutable_planning_debug_info() += remain_ss1;

  // debug
  if (!vec_sl_points.empty() && remain_decider_status &&
      (lead_point_polygon.is_collision || lead_point_grid.is_collision)) {
    // get local (x,y)
    const Pose2D &ego_pose = lead_point_polygon.ego_danger_location;
    double dx = lead_point_polygon.x - ego_pose.x;
    double dy = lead_point_polygon.y - ego_pose.y;
    double obs_transformed_x =
        dx * cos(ego_pose.theta) + dy * sin(ego_pose.theta);
    double obs_transformed_y =
        -dx * sin(ego_pose.theta) + dy * cos(ego_pose.theta);

    // debug get diff between polygon and multi-circle
    double closest_s = 1000.0;
    int min_iter = 0;
    const auto &debug_sl_points = vec_sl_points.at(0);
    for (int i = 0; i < debug_sl_points.size(); ++i) {
      const auto &iter = debug_sl_points.at(i);
      double pos_dist = std::abs(iter.first - lead_point_polygon.s);
      if (pos_dist < closest_s) {
        closest_s = pos_dist;
        min_iter = i;
      }
    }
    std::pair<double, double> closest_sl_grid;
    if (!debug_sl_points.empty())
      closest_sl_grid = debug_sl_points.at(min_iter); // debug mpc
    // std::cout << "!!!!!!! closest remain min index = " << min_iter
    //           << ", s = " << closest_sl_grid.first
    //           << ". closest_dist=" << closest_s << std::endl;

    std::string remain_ss;
    if (lead_point_grid.is_collision) {
      remain_ss = "\ncircle(is_coll, id,s,l,x,y)=(" +
                  std::to_string(lead_point_grid.is_collision) + ", " +
                  std::to_string(lead_point_grid.id) + ", " +
                  std::to_string(lead_point_grid.s).substr(0, 5) + ", " +
                  std::to_string(lead_point_grid.l).substr(0, 5) + ", " +
                  std::to_string(lead_point_grid.x).substr(0, 5) + ", " +
                  std::to_string(lead_point_grid.y).substr(0, 5) + ", " +
                  std::to_string(lead_point_grid.d_rel).substr(0, 5) +
                  ")"
                  // + ", s_diff="
                  // + std::to_string(lead_point_grid.s - lead_point_polygon.s)
                  //        .substr(0, 5)
                  + "\n";
    }

    // if (closest_s < 0.1) { // valid s
    //   remain_ss << "    ((" << obs_transformed_x << ", " << obs_transformed_y
    //             << "), \'dl="
    //             << std::to_string(closest_sl_grid.second - lead_point_polygon.l)
    //             << ", ds="
    //             << std::to_string(lead_point_grid.s - lead_point_polygon.s)
    //             << "\'"
    //             << "), # dan_loc = (s " << lead_point_polygon.s << " )"
    //             << closest_sl_grid.second << " - " << lead_point_polygon.l;
    // }
    // if (lead_point_polygon.is_collision != lead_point_grid.is_collision) {
    //   remain_ss << "\n. diff result!";
    // }
    *PlanningContext::Instance()->mutable_planning_debug_info() +=
        remain_ss;
  }
  // end
  //////////
  // add compare checker
  static int unsame_collision_result = 0;
  static int new_is_bigger = 0;
  if (lead_point_polygon.is_collision || lead_point_grid.is_collision) {
    if (lead_point_polygon.s < 2.0 && lead_point_grid.s < 2.0) {
      if (std::abs(lead_point_polygon.s - lead_point_grid.s) > 0.01) {
        ++unsame_collision_result;
        if (lead_point_polygon.s < lead_point_grid.s)
          ++new_is_bigger;
      }
    }
  }
  // if (unsame_collision_result) {
  //   *PlanningContext::Instance()->mutable_planning_debug_info() +=
  //       "\n[!!!!unsame!!!!]collison result = " +
  //       std::to_string(unsame_collision_result) +
  //       "; new_is_bigger = " + std::to_string(new_is_bigger) + "\n";
  // }
  // end of compare checker
  //////////
}

bool ParkingLongitudinalBehaviorPlanner::compute_lead_obs_info(
    const ParkingSlotInfo &park_info,
    LongitudinalBehaviorPlannerOutput::RemainDistInfo *ptr_remain_dist_info, 
    int* ptr_is_need_pause) {
  auto &fs_point = PlanningContext::Instance()
                       ->longitudinal_behavior_planner_output()
                       .free_space;
  auto &lead_one = PlanningContext::Instance()
                       ->longitudinal_behavior_planner_output()
                       .lead_cars.first;
  auto &fs_line = PlanningContext::Instance()
                      ->longitudinal_behavior_planner_output()
                      .fs_line;

  std::vector<double> remaining_dist_debug_vec;
  double remaining_distance = 1e9; // remain_distance = 10.0;
  ptr_remain_dist_info->set_value(
      remaining_distance, -1, lead_one.type, lead_one.is_static,
      CarParams::GetInstance()->lon_inflation_min, false /*default*/);

  const double LON_INFLATION_FOR_STEP = 0.3;
  double fs_point_lon_inflation =
      (fs_point.type == ObjectType::STEP &&
       PlanningContext::Instance()
               ->parking_behavior_planner_output()
               .parking_slot_info.type.value != ParkingSlotType::PARALLEL)
                     ? LON_INFLATION_FOR_STEP
          : CarParams::GetInstance()->lon_inflation_min;
  if (fs_point.id >= 0) {
    double remain_temp = fs_point.d_rel - fs_point_lon_inflation;
    if (remain_temp <= remaining_distance) {
      remaining_distance = remain_temp;
      ptr_remain_dist_info->set_value(
          fs_point.d_rel, fs_point.id, ObjectType::FREESPACE, true,
          fs_point_lon_inflation, false /*default*/);
      ptr_remain_dist_info->obstacle_type_ = 1;
    }
  }
  remaining_dist_debug_vec.emplace_back(remaining_distance);

  if (lead_one.id >= 0 && lead_one.type == ObjectType::PEDESTRIAN) {
    double remain_temp = lead_one.d_rel - CarParams::GetInstance()->lon_inflation_min;
    if (remain_temp <= remaining_distance) {
      remaining_distance = remain_temp;
      bool is_static = true;
      ptr_remain_dist_info->set_value(
          lead_one.d_rel, lead_one.id, lead_one.type, lead_one.is_static,
          CarParams::GetInstance()->lon_inflation_min, !lead_one.is_static);
      ptr_remain_dist_info->obstacle_type_ = 2;
    }
  }
  if (lead_one.id >= 0 && lead_one.type == ObjectType::COUPE &&
      !lead_one.is_static) {
    double remain_temp = lead_one.d_rel - CarParams::GetInstance()->lon_inflation_min;
    if (remain_temp <= remaining_distance) {
      remaining_distance = remain_temp;
      ptr_remain_dist_info->set_value(
          lead_one.d_rel, lead_one.id, lead_one.type, lead_one.is_static,
          CarParams::GetInstance()->lon_inflation_min, true);
      ptr_remain_dist_info->obstacle_type_ = 3;
    }
  }
  remaining_dist_debug_vec.emplace_back(remaining_distance);

  if (fs_line.id >= 0) {
    double remain_temp = fs_line.d_rel - CarParams::GetInstance()->lon_inflation_min;
    if (remain_temp <= remaining_distance) {
      remaining_distance = remain_temp;
      ptr_remain_dist_info->set_value(
          fs_line.d_rel, fs_line.id, ObjectType::FREESPACE, true,
          CarParams::GetInstance()->lon_inflation_min, false /*default*/);
      ptr_remain_dist_info->obstacle_type_ = 4;
      *PlanningContext::Instance()->mutable_planning_debug_info() +=
          ", fs_line[(" + std::to_string(fs_line.x_start).substr(0, 5) + "," +
          std::to_string(fs_line.y_start).substr(0, 5) + "),(" +
          std::to_string(fs_line.x_end).substr(0, 5) + "," +
          std::to_string(fs_line.y_end).substr(0, 5) + ")] ";
    }
  }
  remaining_dist_debug_vec.emplace_back(remaining_distance);

  // calculate is_need_pause
  if (ptr_remain_dist_info->remaining_distance_ < 0.1 ||
      (PlanningContext::Instance()->planning_status().blocked &&
          (lead_one.id <= 0 ||
           lead_one.d_rel >
               CarParams::GetInstance()->lon_inflation() + 0.1))) { // deadzone
    *ptr_is_need_pause = 1;
  } else {
    *ptr_is_need_pause = 0;
  }

  // debug message
  std::string debug_string;
  auto ptr_debug_info =
      PlanningContext::Instance()->mutable_planning_debug_info();
  debug_string += "\nremain=(pt" + std::to_string(fs_point.d_rel).substr(0, 5) +
                  "-" + std::to_string(fs_point_lon_inflation).substr(0, 5) +
                  "]";
  for (const auto &dist : remaining_dist_debug_vec) {
    debug_string += std::to_string(dist).substr(0, 5) + ",";
  }
  debug_string +=
      "), [vp]tarSpd" + std::to_string(PlanningContext::Instance()
                                           ->planning_status()
                                           .planning_result.v_target)
                            .substr(0, 5);
  // planning sum time
  debug_string +=
      ", whole" +
      std::to_string(MTIME()->timestamp().ms() -
                     PlanningContext::Instance()->planning_start_time_ms())
          .substr(0, 5) +
      "ms\n";
  *ptr_debug_info += debug_string;

  // add time debug
  ptr_remain_dist_info->slot_type_ = (int)park_info.type.value;
  ptr_remain_dist_info->now_time_seq_str_ =
      *PlanningContext::Instance()->mutable_now_time_seq_str();
  return true;
}

} // namespace parking

} // namespace msquare
