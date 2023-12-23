#include "common/parking_world_model.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
// #include "common/obstacle_manager.h"
#include "planning/common/common.h"
#include <bitset>

namespace msquare {

namespace parking {

WorldModel::WorldModel() {
  obstacle_manager_ = std::make_unique<ObstacleManager>();
  ground_line_decider_ = std::make_unique<GroundLineDecider>(1, 0.5);
  // traffic_light_decision_ = std::make_shared<TrafficLightDecision>();

  frenet_parameters_.zero_speed_threshold = 0.1;
  frenet_parameters_.coord_transform_precision = 0.01;
  frenet_parameters_.step_s = 0.3;
  frenet_parameters_.coarse_step_s = 1.0;
  frenet_parameters_.optimization_gamma = 0.5;
  frenet_parameters_.max_iter = 15;
  refline_manager_ = RefLineManager::Instance();
  auto env = std::getenv("RealitySimulation");
  if (env != nullptr) {
    if (std::strcmp(env, "simulation") == 0) {
      is_simulation_ = true;
    }
  }
  wireless_charger_report_data_.detected_state.value = 255;
}

WorldModel::~WorldModel() {}

bool WorldModel::init(SceneType scene_type) {
  scene_type_ = scene_type;

  (void)obstacle_manager_->init(shared_from_this());
  refline_manager_->init(shared_from_this());
  return true;
}

bool WorldModel::update() {
  // must update message first each frame
  return update_parking();
}

void WorldModel::update_members() {
  // ApfStatus apf_status = PlanningContext::Instance()->apf_status();
  // OpenSpacePath last_apf_path = PlanningContext::Instance()->last_apf_path();
  // map_info_manager_.set_map_info_from_apf();

  bool dbw_status_flag = vehicle_dbw_status_;
  if (dbw_status_flag) {
    // std::cout << "dbw msg true" << std::endl;
  } else {
    // std::cout << "dbw msg false!" << std::endl;
  }
  dbw_status_ = dbw_status_flag;

  // if (apf_status.use_apf_refline_ && is_parking_apa()) {
  //   if ((!apf_status.reasonable_ || apf_status.empty_)) {
  //     if (PlanningContext::Instance()->planning_status().scenario.status_type
  //     !=
  //             StatusType::APA &&
  //         PlanningContext::Instance()->planning_status().scenario.status_type
  //         !=
  //             StatusType::APOA &&
  //         PlanningContext::Instance()->planning_status().scenario.status_type
  //         !=
  //             StatusType::WAIT &&
  //         PlanningContext::Instance()
  //                 ->parking_behavior_planner_output()
  //                 .planner_type != PlannerType::OPENSPACE) {
  //       printf("Reference trajectory empty!!!");
  //       pretreatment_status_ = false;
  //       return;
  //     }
  //   }
  // } else {
  if (map_info_manager_.get_map_info().ref_trajectory.empty()) {
    if (PlanningContext::Instance()->planning_status().scenario.status_type !=
            StatusType::APA &&
        PlanningContext::Instance()->planning_status().scenario.status_type !=
            StatusType::APOA &&
        PlanningContext::Instance()->planning_status().scenario.status_type !=
            StatusType::WAIT &&
        PlanningContext::Instance()->planning_status().scenario.status_type !=
            StatusType::RPA_STRAIGHT_STANDBY &&
        PlanningContext::Instance()->planning_status().scenario.status_type !=
            StatusType::RPA_STRAIGHT &&
        PlanningContext::Instance()
                ->parking_behavior_planner_output()
                .planner_type != PlannerType::OPENSPACE) {
      pretreatment_status_ = false;
      return;
    }
  }
  // }

  // FIXME: change FrenetCoordinateSystem signature
  auto &planning_status = PlanningContext::Instance()->planning_status();
  // if (planning_status.scenario.status_type == StatusType::WAIT){
  //   return;
  // }
  std::vector<double> vx, vy;
  std::vector<Pose2D> traj_pose_array =
      planning_status.planning_result.traj_pose_array;
  // std::cout << "ego_planning: size: " <<traj_pose_array.size()<<std::endl;
  if (PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .planner_type == PlannerType::PARKING_LATERAL) {
    (void)extend_traj(traj_pose_array);
  }

  if (map_info_manager_.get_map_info().ref_trajectory.size() == 0) {
    // map
    double x, y;
    for (int i = -5; i < 15; i++) {
      x = ego_state_manager_.get_ego_state().ego_pose.x +
          i * cos(ego_state_manager_.get_ego_state().ego_pose.theta);
      y = ego_state_manager_.get_ego_state().ego_pose.y +
          i * sin(ego_state_manager_.get_ego_state().ego_pose.theta);
      vx.push_back(x);
      vy.push_back(y);
    }
    // std::cout << "Map Info Refline is empty! Use virtual trajectory to
    // generate frenet coords" << std::endl;
    double v_caculate_s = std::max(
        10.0 / 3.6, std::max(map_info_manager_.get_map_info().v_cruise,
                             ego_state_manager_.get_ego_state().ego_vel));
    if (nullptr == frenet_coord_) {
      frenet_coord_ = std::make_shared<FrenetCoordinateSystem>();
    }
    frenet_coord_->Update(vx, vy, frenet_parameters_, 0.0,
                          ego_state_manager_.get_ego_state().ego_frenet.x +
                              v_caculate_s *
                                  (FLAGS_trajectory_time_length + 1 / 10.0));
    // plannning
    if (traj_pose_array.size() > 0) {
      vx.clear();
      vy.clear();
      for (auto &p : traj_pose_array) {
        vx.push_back(p.x);
        vy.push_back(p.y);
      }
    }
    if (nullptr == frenet_coord_planning_) {
      frenet_coord_planning_ = std::make_shared<FrenetCoordinateSystem>();
    }
    frenet_coord_planning_->Update(vx, vy, frenet_parameters_, 0.0, 100.0);
  } else {
    // map

    // if (apf_status.reasonable_ && !apf_status.empty_ &&
    //     apf_status.use_apf_refline_ && is_parking_apa()) {
    //   double dx, dy;
    //   dx = last_apf_path.path_poses.at(0).pos.x -
    //        last_apf_path.path_poses.at(1).pos.x;
    //   dy = last_apf_path.path_poses.at(0).pos.y -
    //        last_apf_path.path_poses.at(1).pos.y;
    //   for (int i = 5; i > 0; i--) {
    //     vx.push_back(last_apf_path.path_poses.at(0).pos.x + dx * i);
    //     vy.push_back(last_apf_path.path_poses.at(0).pos.y + dy * i);
    //   }
    //   for (auto &p : last_apf_path.path_poses) {
    //     vx.push_back(p.pos.x);
    //     vy.push_back(p.pos.y);
    //   }
    //   std::cout << "zjt debug: frenet: use apf" << std::endl;
    //   std::cout << "zjt debug: frenet: size: "
    //             << last_apf_path.path_poses.size() << std::endl;

    // } else {
    for (auto &p : map_info_manager_.get_map_info().ref_trajectory) {
      vx.push_back(p.x);
      vy.push_back(p.y);
    }

    //   std::cout << "zjt debug: frenet: use map" << std::endl;
    // }

    // for (int index = 0; index < vx.size(); index++) {
    //   std::cout << "zjt debug: frenet: x: " << vx.at(index)
    //             << " y: " << vy.at(index) << std::endl;
    // }

    double v_caculate_s = std::max(
        10.0 / 3.6, std::max(map_info_manager_.get_map_info().v_cruise,
                             ego_state_manager_.get_ego_state().ego_vel));
    if (nullptr == frenet_coord_) {
      frenet_coord_ = std::make_shared<FrenetCoordinateSystem>();
    }
    frenet_coord_->Update(vx, vy, frenet_parameters_, 0.0,
                          ego_state_manager_.get_ego_state().ego_frenet.x +
                              v_caculate_s *
                                  (FLAGS_trajectory_time_length + 1 / 10.0));

    // // zjt debug
    // for(int index = 0; index < vx.size();index ++){
    //   std::cout << "frenet point: " << vx[index] << " " << vy[index] <<
    //   std::endl;
    // }

    // plannning
    if (traj_pose_array.size() > 3) {
      vx.clear();
      vy.clear();
      for (auto &p : traj_pose_array) {
        vx.push_back(p.x);
        vy.push_back(p.y);
      }
    }
    if (nullptr == frenet_coord_planning_) {
      frenet_coord_planning_ = std::make_shared<FrenetCoordinateSystem>();
    }
    frenet_coord_planning_->Update(vx, vy, frenet_parameters_, 0.0, 100.0);
  }

  ego_state_manager_.set_ego_carte(
      convert_pose2point(ego_state_manager_.get_ego_state().ego_pose));
  // planning
  ego_state_manager_.set_ego_state_planning();
  if (frenet_coord_planning_->CartCoord2FrenetCoord(
          ego_state_manager_.get_ego_state_planning().ego_carte,
          ego_state_manager_.get_mutable_ego_state_planning().ego_frenet) ==
      TRANSFORM_FAILED) {
    // std::cout << "Ego state transform failed on planning frenet coordinates"
    // << std::endl;
  }

  if (frenet_coord_->CartCoord2FrenetCoord(
          ego_state_manager_.get_ego_state().ego_carte,
          ego_state_manager_.get_mutable_ego_state().ego_frenet) ==
      TRANSFORM_FAILED) {
    pretreatment_status_ = false;
    return;
  }
}

bool WorldModel::update_parking() {

  pretreatment_status_ = true;
  // traffic_light_decision_->update(shared_from_this());
  double start = MTIME()->timestamp().sec();

  PlanningContext::Instance()
      ->mutable_planning_status()
      ->rerouting.need_rerouting = false;
  if (!get_localization_status()) {
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->rerouting.need_rerouting = true;
    return false;
  }
  // if (is_parking_lvp()) {
  //   map_info_manager_.set_map_info(shared_from_this());
  // }
  if (is_parking_apa() || is_parking_lvp()) {
    map_info_manager_.set_map_info_from_apf();
  }

  if (!map_info_manager_.update(shared_from_this())) {
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->rerouting.need_rerouting = true;
    if (PlanningContext::Instance()->planning_status().scenario.status_type !=
            StatusType::APA &&
        PlanningContext::Instance()->planning_status().scenario.status_type !=
            StatusType::APOA &&
        PlanningContext::Instance()->planning_status().scenario.status_type !=
            StatusType::WAIT &&
        PlanningContext::Instance()->planning_status().scenario.status_type !=
            StatusType::RPA_STRAIGHT_STANDBY &&
        PlanningContext::Instance()->planning_status().scenario.status_type !=
            StatusType::RPA_STRAIGHT &&
        PlanningContext::Instance()
                ->parking_behavior_planner_output()
                .planner_type != PlannerType::OPENSPACE) {
      pretreatment_status_ = false;
      return false;
    }
  }

  double after_update_msgs = MTIME()->timestamp().sec();
  auto update_map_info = after_update_msgs - start;
  // std::cout << "check_time_count: update_map_info " <<
  // update_map_info << std::endl;

  update_members();

  double after_update_msgs22 = MTIME()->timestamp().sec();
  auto update_map_info22 = after_update_msgs22 - after_update_msgs;
  // std::cout << "check_time_count: update_map_info22 " <<
  // update_map_info22 << std::endl;
  // if(PlanningContext::Instance()->planning_status().scenario.status_type ==
  // StatusType::WAIT)
  //   return true;

  auto *planning_status =
      PlanningContext::Instance()->mutable_planning_status();

  // TODO: when updated failed, reset planning_loop to 0
  planning_status->planning_loop++;

  if (!dbw_status_) {
    planning_status->planning_loop = 0;
    // return false;
  }
  // TODO: update self info (ego_pose_, ego_acc_, etc.)
  // TODO: update map info (reference_line_, frenet_coord_ etc.)
  ego_state_manager_.set_planning_start_state_parking(shared_from_this());

  double start2 = MTIME()->timestamp().sec();
  // get_map_lateral_info_parking();

  // zjt debug
  // std::cout << "refline_manager_->update(): start" << std::endl;
  refline_manager_->update();
  // std::cout << "refline_manager_->update(): stop" << std::endl;
  // debug end

  double after_update_msgs2 = MTIME()->timestamp().sec();
  auto update_map_info2 = after_update_msgs2 - start2;
  // std::cout << "check_time_count: update_map_info2 " <<

  // using map lateral info
  (void)construct_obstacle_manager_parking();
  double after_update_msgs333 = MTIME()->timestamp().sec();
  auto update_map_info333 = after_update_msgs333 - after_update_msgs2;
  // std::cout << "check_time_count: update_map_info333 " <<
  // update_map_info333 << std::endl;

  // update_scenes temp
  update_scenes();
  return true;
}

bool WorldModel::construct_obstacle_manager_parking() {
  // 1. original dev obsInterfaceLaneKeep API(could just be used by old
  // optimal_planner)

  // compute adc sl boundary
  obstacle_manager_->clear();
  double obs_start_s(std::numeric_limits<double>::max());
  double obs_end_s(std::numeric_limits<double>::lowest());
  double obs_start_l(std::numeric_limits<double>::max());
  double obs_end_l(std::numeric_limits<double>::lowest());
  std::vector<planning_math::Vec2d> adc_points;
  const auto &adc_point = ego_state_manager_.get_ego_state().ego_pose;
  planning_math::Vec2d center(
      adc_point.x + VehicleParam::Instance()->center_to_geometry_center *
                        cos(adc_point.theta),
      adc_point.y + VehicleParam::Instance()->center_to_geometry_center *
                        sin(adc_point.theta));
  planning_math::Box2d adc_box(center, adc_point.theta,
                               VehicleParam::Instance()->length,
                               VehicleParam::Instance()->width);
  adc_box.GetAllCorners(&adc_points);
  for (const planning_math::Vec2d &obs_point : adc_points) {
    Point2D frenet_point, carte_point;
    carte_point.x = obs_point.x();
    carte_point.y = obs_point.y();
    if (frenet_coord_->CartCoord2FrenetCoord(carte_point, frenet_point) ==
        TRANSFORM_FAILED) {
      continue;
    }
    obs_start_s = std::min(obs_start_s, frenet_point.x);
    obs_end_s = std::max(obs_end_s, frenet_point.x);
    obs_start_l = std::min(obs_start_l, frenet_point.y);
    obs_end_l = std::max(obs_end_l, frenet_point.y);
  }
  adc_sl_boundary_.start_s = obs_start_s;
  adc_sl_boundary_.end_s = obs_end_s;
  adc_sl_boundary_.start_l = obs_start_l;
  adc_sl_boundary_.end_l = obs_end_l;

  // static vehicle detection results
  for (auto car_object : parking_fusion_vision_info_.car_array) {
    if (!car_object.is_static) {
      continue;
    }
    bool is_static =
        true; // std::hypot(car_object.velocity.x, car_object.velocity.y) < 0.5;
    int hash_id = car_object.track_id + 4000000;
    if (nullptr == obs_ptr_) {
      obs_ptr_ = std::make_shared<Obstacle>(hash_id, car_object, is_static);
    } else {
      obs_ptr_->update(Obstacle(hash_id, car_object, is_static));
    }
    auto mutable_obs = obstacle_manager_->add_static_obstacle(*obs_ptr_);
    // note: no need to calculate s&l boundary because we dont use static
    // vehicle to do planning while in AVP phase
  }

  // construct obstacle and obstaclemanager accordingly
  // human_preception and car_preception
  // if (use_fusion_result_) {
  const FusionInfo *parking_vision_info = &parking_fusion_vision_info_;
  // for car_preception
  for (auto car_object : parking_vision_info->car_array) {
    bool is_static = true;
    if (VehicleParam::Instance()->car_type == "C03") {
      is_static = car_object.is_static;
    }
    // [EPL3-338863] temp solution, make ofo static
    if (car_object.type == FusionObjectType::MSD_OBJECT_TYPE_OFO) {
      is_static = car_object.is_static;
    }
    // true; // std::hypot(ego_state_manager_.get_ego_state().ego_vel
    //  + car_object.d3_vx, car_object.d3_vy) < 0.5;
    planning_math::Polygon2d perception_sl_polygon;
    int hash_id = car_object.track_id + 1000000;
    if (nullptr == obs_ptr_) {
      obs_ptr_ = std::make_shared<Obstacle>(hash_id, car_object, is_static);
    } else {
      obs_ptr_->update(Obstacle(hash_id, car_object, is_static));
    }
    auto mutable_obs = obstacle_manager_->add_obstacle(*obs_ptr_);
    SLBoundary perception_sl{};
    SLBoundary perception_sl_planning{};
    mutable_obs->ComputeSlBoundary(frenet_coord_planning_,
                                   &perception_sl_planning, true);
    mutable_obs->SetPerceptionSlBoundaryPlanning(perception_sl_planning);
    mutable_obs->ComputeSlBoundary(frenet_coord_, &perception_sl);
    mutable_obs->SetPerceptionSlBoundaryOrigin(perception_sl);
    mutable_obs->SetPerceptionSlBoundary(perception_sl);
    (void)obstacle_manager_->set_beside_intersation(mutable_obs->Id());
    (void)obstacle_manager_->set_road_status(mutable_obs->Id());
    if (mutable_obs->isInRoadLoose() && mutable_obs->IsInBend()) {
      mutable_obs->ComputePolygon2dSlBoundary(frenet_coord_,
                                              &perception_sl_polygon);
      mutable_obs->SetPerceptionSlPolygon(perception_sl_polygon);
      if (!perception_sl_polygon.GetAllVertices().empty())
        mutable_obs->SetPreciseInterpFlag(true);
    }

    // double left_border_dist_start =
    // planning_math::interps(ref_trajectory_info_.left_road_border_distance,
    //                                         ref_trajectory_info_.s,
    //                                         perception_sl.start_s);
    // double left_border_dist_end =
    // planning_math::interps(ref_trajectory_info_.left_road_border_distance,
    //                                         ref_trajectory_info_.s,
    //                                         perception_sl.end_s);
    // mutable_obs->SetLeftSpaceBorder(left_border_dist_start,
    // left_border_dist_end); double right_border_dist_start =
    // planning_math::interps(ref_trajectory_info_.right_road_border_distance,
    //                                         ref_trajectory_info_.s,
    //                                         perception_sl.start_s);
    // double right_border_dist_end =
    // planning_math::interps(ref_trajectory_info_.right_road_border_distance,
    //                                         ref_trajectory_info_.s,
    //                                         perception_sl.end_s);
    // mutable_obs->SetRightSpaceBorder(right_border_dist_start,
    // right_border_dist_end);
  }

  for (auto human_object : parking_vision_info->human_array) {
    // std::cout << "human: " << human_object.track_id << std::endl;
    // Relative velocity
    bool is_static = true;
    int hash_id = human_object.track_id + 2000000;
    if (nullptr == obs_ptr_) {
      obs_ptr_ = std::make_shared<Obstacle>(hash_id, human_object, is_static);
    } else {
      obs_ptr_->update(Obstacle(hash_id, human_object, is_static));
    }
    auto mutable_obs = obstacle_manager_->add_obstacle(*obs_ptr_);
    SLBoundary perception_sl{};
    SLBoundary perception_sl_planning{};
    mutable_obs->ComputeSlBoundary(frenet_coord_planning_,
                                   &perception_sl_planning, true);
    mutable_obs->SetPerceptionSlBoundaryPlanning(perception_sl_planning);
    mutable_obs->ComputeSlBoundary(frenet_coord_, &perception_sl);
    mutable_obs->SetPerceptionSlBoundary(perception_sl);
    (void)obstacle_manager_->set_beside_intersation(mutable_obs->Id());
    (void)obstacle_manager_->set_road_status(mutable_obs->Id());
    // double left_border_dist_start =
    // planning_math::interps(ref_trajectory_info_.left_road_border_distance,
    //                                         ref_trajectory_info_.s,
    //                                         perception_sl.start_s);
    // double left_border_dist_end =
    // planning_math::interps(ref_trajectory_info_.left_road_border_distance,
    //                                         ref_trajectory_info_.s,
    //                                         perception_sl.end_s);
    // mutable_obs->SetLeftSpaceBorder(left_border_dist_start,
    // left_border_dist_end); double right_border_dist_start =
    // planning_math::interps(ref_trajectory_info_.right_road_border_distance,
    //                                         ref_trajectory_info_.s,
    //                                         perception_sl.start_s);
    // double right_border_dist_end =
    // planning_math::interps(ref_trajectory_info_.right_road_border_distance,
    //                                         ref_trajectory_info_.s,
    //                                         perception_sl.end_s);
    // mutable_obs->SetRightSpaceBorder(right_border_dist_start,
    // right_border_dist_end);
  }

  // For square mapping
  for (const auto &gate : map_info_manager_.get_map_info()
                              .square_mapping_result.obstacles.unknown) {
    // std::cout << "gate: " << gate.id << "  " << gate.pts[0].z << std::endl;
    // bool is_static =
    // true;//std::hypot(ego_state_manager_.get_ego_state().ego_vel +
    // car_object.d3_vx, car_object.d3_vy) < 2.0;
    planning_math::Polygon2d perception_sl_polygon;
    int hash_id = gate.id + 3000000;
    if (nullptr == obs_ptr_) {
      obs_ptr_ = std::make_shared<Obstacle>(hash_id, gate);
    } else {
      obs_ptr_->update(Obstacle(hash_id, gate));
    }
    auto mutable_obs = obstacle_manager_->add_gate(*obs_ptr_);
    SLBoundary perception_sl{};
    SLBoundary perception_sl_planning{};
    mutable_obs->ComputeSlBoundary(frenet_coord_, &perception_sl);
    mutable_obs->ComputeSlBoundary(frenet_coord_planning_,
                                   &perception_sl_planning, true);
    mutable_obs->SetPerceptionSlBoundaryPlanning(perception_sl_planning);
    mutable_obs->SetPerceptionSlBoundaryOrigin(perception_sl);
    // on one side of the reference line and
    mutable_obs->SetPerceptionSlBoundary(perception_sl);
    mutable_obs->ComputePolygon2dSlBoundary(frenet_coord_,
                                            &perception_sl_polygon);
    mutable_obs->SetPerceptionSlPolygon(perception_sl_polygon);
    if (!perception_sl_polygon.GetAllVertices().empty())
      mutable_obs->SetPreciseInterpFlag(true);
    // obstacle_manager_->set_road_status(mutable_obs->Id(),
    // ref_trajectory_info_); for(double s = perception_sl.start_s; s <
    // perception_sl.end_s; s += (perception_sl.end_s - perception_sl.start_s)
    // / 10.0){
    //   std::cout << s << "  " << mutable_obs->InterpBounds(s, "right") <<
    //   std::endl;
    // }
    double left_border_dist_start = planning_math::interps(
        refline_manager_->get_left_road_border_distance(),
        refline_manager_->get_s(), perception_sl.start_s);
    double left_border_dist_end = planning_math::interps(
        refline_manager_->get_left_road_border_distance(),
        refline_manager_->get_s(), perception_sl.end_s);
    mutable_obs->SetLeftSpaceBorder(left_border_dist_start,
                                    left_border_dist_end, perception_sl);
    double right_border_dist_start = planning_math::interps(
        refline_manager_->get_right_road_border_distance(),
        refline_manager_->get_s(), perception_sl.start_s);
    double right_border_dist_end = planning_math::interps(
        refline_manager_->get_right_road_border_distance(),
        refline_manager_->get_s(), perception_sl.end_s);
    mutable_obs->SetRightSpaceBorder(right_border_dist_start,
                                     right_border_dist_end, perception_sl);
  }

  // for static filter
  (void)obstacle_manager_->obj_direction_filter();
  (void)obstacle_manager_->obj_speed_filter();
  (void)obstacle_manager_->obj_intention_status_manager();
  (void)obstacle_manager_->obj_pseudo_prediction();
  (void)obstacle_manager_->obj_specific_manager();

  update_groundline();
  return true;
}

// void WorldModel::update_and_filter_freespace(
//     const std::vector<planning_math::LineSegment2d> &map_boundarys) {

//   if (!frenet_coord_) {
//     return;
//   }
//   const auto &freespace_uss =
//       parking_fusion_vision_info_
//           .uss_fusion; // parking_vision_info_.free_space.d3_border_pts;
//   const auto &freespace_fusion = parking_fusion_vision_info_.free_space;
//   const auto &ego_state = ego_state_manager_.get_ego_state();

//   using namespace planning_math;

//   // int counter = 0;
//   base_lines_.Init(frenet_coord_, ego_state.ego_frenet.x,
//   ego_state.ego_pose); double left_border = planning_math::interps(
//       refline_manager_->get_left_road_border_distance(),
//       refline_manager_->get_s(), adc_sl_boundary_.start_s);
//   double right_border = planning_math::interps(
//       refline_manager_->get_right_road_border_distance(),
//       refline_manager_->get_s(), adc_sl_boundary_.start_s);
//   base_lines_.SetBoard(left_border, -right_border);

//   std::vector<std::vector<LineSegment2d>> filter_freespace_group;
//   std::vector<LineSegment2d> filter_freespace;
//   int last_line_id = -1;
//   // std::cout << "xzj freespace uss size is " << freespace_uss.size() <<
//   // std::endl; std::cout << "xzj freespace fusion size is " <<
//   // freespace_fusion.size() << std::endl; std::cout << "xzj ego state is "
//   <<
//   // ego_state.ego_box.center_x() << ","
//   //  << ego_state.ego_box.center_y() << "," << ego_state.ego_box.heading()
//   <<
//   //  std::endl;
//   for (size_t point_id = 0; point_id < freespace_uss.size(); point_id++) {
//     auto point_ptr =
//         std::make_shared<Obstacle>(point_id,
//         freespace_uss[point_id].position);

//     auto mutable_obs = obstacle_manager_->add_uss_point(*point_ptr);
//   }

//   for (size_t point_id = 0; point_id < freespace_fusion.size(); point_id++) {

//     // Point3D start_point, end_point;
//     // if (line_id == freespace_single.size() - 1) {
//     //   start_point = freespace_single.at(line_id).position;
//     //   end_point   = freespace_single.front().position;
//     // } else {
//     //   start_point = freespace_single.at(line_id).position;
//     //   end_point   = freespace_single.at(line_id + 1).position;
//     // }

//     // auto start_vec = Vec2d(start_point.x, start_point.y, line_id);
//     // auto end_vec   = Vec2d(end_point.x, end_point.y, line_id + 1);

//     // std::cout << "xzj origin: " << start_point.x << "," << start_point.y
//     <<
//     // ","
//     // << end_point.x << "," << end_point.y<< std::endl;
//     // add line for lon plan
//     // if (freespace_filter(LineSegment2d(start_vec, end_vec), "lon")) {
//     //   continue;
//     // }
//     auto point_ptr = std::make_shared<Obstacle>(
//         point_id, freespace_fusion[point_id].position);

//     auto mutable_obs = obstacle_manager_->add_point(*point_ptr);

//     // SLBoundary perception_sl{};
//     // SLBoundary perception_sl_planning{};
//     // mutable_obs->ComputeSlLine(frenet_coord_planning_,
//     // &perception_sl_planning, true);
//     // mutable_obs->SetPerceptionSlBoundaryPlanning(perception_sl_planning);
//     // mutable_obs->ComputeSlLine(frenet_coord_, &perception_sl);
//     // mutable_obs->SetPerceptionSlBoundary(perception_sl);
//   }
//   // std::cout<<"fsdebug fssize = "<<freespace_uss.size()<<std::endl;
//   int line_count = 0;

//   for (size_t line_id = 0; line_id < freespace_fusion.size(); line_id++) {

//     // std::cout << "srbdebug_fs_origin:  " << freespace_fusion[line_id].x <<
//     "
//     // " << freespace_fusion[line_id].y << std::endl;

//     Point3D start_point, end_point;
//     if (line_id == freespace_fusion.size() - 1) {
//       start_point = freespace_fusion.at(line_id).position;
//       end_point = freespace_fusion.front().position;
//       if (freespace_fusion.size() == 1) {
//         end_point.x += 0.1;
//         end_point.y += 0.1;
//       }
//     } else {
//       start_point = freespace_fusion.at(line_id).position;
//       end_point = freespace_fusion.at(line_id + 1).position;
//     }

//     auto start_vec = Vec2d(start_point.x, start_point.y, line_id);
//     auto end_vec = Vec2d(end_point.x, end_point.y, line_id + 1);

//     // std::cout << "xzj origin: " << start_point.x << "," << start_point.y
//     <<
//     // ","
//     //           << end_point.x << "," << end_point.y<< std::endl;

//     // std::cout << "srbdebug_fs_origin:  " << mutable_obs->Id()
//     //     << "  " << mutable_obs->PerceptionLine().start().x()
//     //     << "  " << mutable_obs->PerceptionLine().start().y()
//     //     << "  " << mutable_obs->PerceptionLine().end().x()
//     //     << "  " << mutable_obs->PerceptionLine().end().y()
//     //     << std::endl;
//     LineSegment2d cur_line(start_vec, end_vec);

//     // if (freespace_filter(cur_line, "lon")) {
//     //   continue;
//     // }
//     // add line for lat plan
//     if (freespace_filter(cur_line, "lat") ||
//         cur_line.length() > VehicleParam::Instance()->width) {
//       continue;
//     }

//     if (line_id != (last_line_id + 1) && !filter_freespace.empty()) {
//       filter_freespace_group.emplace_back(filter_freespace);
//       filter_freespace.clear();
//       // std::cout << "srbdebug freespace!!! " << std::endl;
//     }
//     filter_freespace.emplace_back(cur_line);
//     last_line_id = line_id;
//     line_count++;
//     // std::cout << "xzj origin: " << start_point.x << "," << start_point.y
//     <<
//     // ","
//     //           << end_point.x << "," << end_point.y << std::endl;
//   }
//   filter_freespace_group.emplace_back(filter_freespace);
//   filter_freespace_group.clear(); // TODO: refactor
//   // std::cout << "srbdebug:freespace: " << line_count << " " <<
//   // filter_freespace_group.size() << std::endl;
//   int obstacle_id = -1;
//   std::vector<std::shared_ptr<Obstacle>> freespace_add;
//   if (line_count <= 10) {
//     for (const auto &freespace_array : filter_freespace_group) {
//       obstacle_id++;
//       for (size_t index = 0; index < freespace_array.size(); index++) {
//         SLBoundary line_sl{};
//         auto obs_ptr =
//             std::make_shared<Obstacle>(obstacle_id, freespace_array[index]);
//         obs_ptr->ComputeSlBoundary(frenet_coord_, &line_sl);

//         if (freespace_filter(freespace_array[index], "lat", &line_sl)) {
//           continue;
//         }

//         freespace_add.push_back(obs_ptr);
//         auto mutable_obs = obs_ptr;
//         // auto mutable_obs =
//         // obstacle_manager_->add_freespace_obstacle(*obs_ptr);

//         if (mutable_obs->PerceptionBoundingBox().DistanceTo(
//                 get_ego_state().ego_box) < 0.05)
//           continue;
//         SLBoundary perception_sl = line_sl;
//         // mutable_obs->ComputeSlBoundary(frenet_coord_, &perception_sl);
//         mutable_obs->SetPerceptionSlBoundary(line_sl);

//         // std::cout << "srbdebu_cut: " << obstacle_id
//         // << "  " << freespace_array[index].start().x()
//         // << "  " << freespace_array[index].start().y()
//         // << "  " << freespace_array[index].end().x()
//         // << "  " << freespace_array[index].end().y()
//         // << " " << line_sl.start_s << " " << line_sl.end_s
//         // << std::endl;

//         double left_border_dist_start = planning_math::interps(
//             refline_manager_->get_left_road_border_distance(),
//             refline_manager_->get_s(), perception_sl.start_s);
//         double left_border_dist_end = planning_math::interps(
//             refline_manager_->get_left_road_border_distance(),
//             refline_manager_->get_s(), perception_sl.end_s);
//         mutable_obs->SetLeftSpaceBorder(left_border_dist_start,
//                                         left_border_dist_end, line_sl);

//         double right_border_dist_start = planning_math::interps(
//             refline_manager_->get_right_road_border_distance(),
//             refline_manager_->get_s(), perception_sl.start_s);
//         double right_border_dist_end = planning_math::interps(
//             refline_manager_->get_right_road_border_distance(),
//             refline_manager_->get_s(), perception_sl.end_s);
//         mutable_obs->SetRightSpaceBorder(right_border_dist_start,
//                                          right_border_dist_end, line_sl);

//         double left_lane_dist_start = planning_math::interps(
//             refline_manager_->get_left_lane_border_distance(),
//             refline_manager_->get_s(), perception_sl.start_s);
//         double left_lane_dist_end = planning_math::interps(
//             refline_manager_->get_left_lane_border_distance(),
//             refline_manager_->get_s(), perception_sl.end_s);
//         mutable_obs->SetLeftSpaceLane(left_lane_dist_start,
//         left_lane_dist_end,
//                                       line_sl);

//         double right_lane_dist_start = planning_math::interps(
//             refline_manager_->get_right_lane_border_distance(),
//             refline_manager_->get_s(), perception_sl.start_s);
//         double right_lane_dist_end = planning_math::interps(
//             refline_manager_->get_right_lane_border_distance(),
//             refline_manager_->get_s(), perception_sl.end_s);
//         mutable_obs->SetRightSpaceLane(right_lane_dist_start,
//                                        right_lane_dist_end, line_sl);

//         obstacle_id++;
//       }
//     }
//   } else {
//     for (const auto &freespace_array : filter_freespace_group) {
//       int last_id = 0;
//       obstacle_id++;
//       // std::cout << "srbdebug: freespace_array " << freespace_array.size()
//       <<
//       // std::endl;
//       for (size_t index = 0; index < freespace_array.size(); index++) {
//         LineSegment2d line;
//         if (freespace_array.size() < 5) {
//           line = freespace_array[index];
//         } else {
//           if ((index != 0 && !(index % 10)) ||
//               index == freespace_array.size() - 1) {
//             line = LineSegment2d(freespace_array[last_id].start(),
//                                  freespace_array[index].end());
//             last_id = index + 1;
//           } else {
//             continue;
//           }
//         }

//         auto obs_ptr = std::make_shared<Obstacle>(obstacle_id, line);

//         SLBoundary line_sl{};
//         obs_ptr->ComputeSlBoundary(frenet_coord_, &line_sl);

//         if (freespace_filter(line, "lat", &line_sl)) {
//           continue;
//         }

//         // auto mutable_obs =
//         // obstacle_manager_->add_freespace_obstacle(*obs_ptr);
//         if (obs_ptr->PerceptionBoundingBox().DistanceTo(
//                 get_ego_state().ego_box) < 0.05)
//           continue;
//         freespace_add.push_back(obs_ptr);
//         auto mutable_obs = obs_ptr;

//         SLBoundary perception_sl = line_sl;
//         // mutable_obs->ComputeSlBoundary(frenet_coord_, &perception_sl);
//         mutable_obs->SetPerceptionSlBoundary(line_sl);

//         // std::cout << "srbdebu_cut: " << obstacle_id
//         // << "  " << line.start().x()
//         // << "  " << line.start().y()
//         // << "  " << line.end().x()
//         // << "  " << line.end().y()
//         // << " " << line_sl.start_s << " " << line_sl.end_s
//         // << std::endl;

//         double left_border_dist_start = planning_math::interps(
//             refline_manager_->get_left_road_border_distance(),
//             refline_manager_->get_s(), perception_sl.start_s);
//         double left_border_dist_end = planning_math::interps(
//             refline_manager_->get_left_road_border_distance(),
//             refline_manager_->get_s(), perception_sl.end_s);
//         mutable_obs->SetLeftSpaceBorder(left_border_dist_start,
//                                         left_border_dist_end, line_sl);

//         double right_border_dist_start = planning_math::interps(
//             refline_manager_->get_right_road_border_distance(),
//             refline_manager_->get_s(), perception_sl.start_s);
//         double right_border_dist_end = planning_math::interps(
//             refline_manager_->get_right_road_border_distance(),
//             refline_manager_->get_s(), perception_sl.end_s);
//         mutable_obs->SetRightSpaceBorder(right_border_dist_start,
//                                          right_border_dist_end, line_sl);

//         double left_lane_dist_start = planning_math::interps(
//             refline_manager_->get_left_lane_border_distance(),
//             refline_manager_->get_s(), perception_sl.start_s);
//         double left_lane_dist_end = planning_math::interps(
//             refline_manager_->get_left_lane_border_distance(),
//             refline_manager_->get_s(), perception_sl.end_s);
//         mutable_obs->SetLeftSpaceLane(left_lane_dist_start,
//         left_lane_dist_end,
//                                       line_sl);

//         double right_lane_dist_start = planning_math::interps(
//             refline_manager_->get_right_lane_border_distance(),
//             refline_manager_->get_s(), perception_sl.start_s);
//         double right_lane_dist_end = planning_math::interps(
//             refline_manager_->get_right_lane_border_distance(),
//             refline_manager_->get_s(), perception_sl.end_s);
//         mutable_obs->SetRightSpaceLane(right_lane_dist_start,
//                                        right_lane_dist_end, line_sl);

//         obstacle_id++;
//       }
//     }
//   }
//   obstacle_manager_->add_freespace_with_filter(freespace_add);
// }

void WorldModel::update_groundline() {
  if (false && is_parking_lvp() || is_parking_svp()) {
    const auto &groundline = parking_fusion_vision_info_.free_space;
    auto groundline_clusters = ground_line_decider_->execute(groundline);
    int cluster_id = 5000000;
    for (auto &groundline_cluster : groundline_clusters) {
      cluster_id += 1;
      if (nullptr == obs_ptr_) {
        obs_ptr_ = std::make_shared<Obstacle>(cluster_id, groundline_cluster);
      } else {
        obs_ptr_->update(Obstacle(cluster_id, groundline_cluster));
      }
      auto mutable_obs = obstacle_manager_->add_groundline_obstacle(*obs_ptr_);
      SLBoundary perception_sl{};
      mutable_obs->ComputePointsSlBoundary(frenet_coord_, &perception_sl);
      mutable_obs->SetPerceptionSlBoundary(perception_sl);

      double left_border_dist_start = planning_math::interps(
          refline_manager_->get_left_road_border_distance(),
          refline_manager_->get_s(), perception_sl.start_s);
      double left_border_dist_end = planning_math::interps(
          refline_manager_->get_left_road_border_distance(),
          refline_manager_->get_s(), perception_sl.end_s);
      mutable_obs->SetLeftSpaceBorder(left_border_dist_start,
                                      left_border_dist_end, perception_sl);

      double right_border_dist_start = planning_math::interps(
          refline_manager_->get_right_road_border_distance(),
          refline_manager_->get_s(), perception_sl.start_s);
      double right_border_dist_end = planning_math::interps(
          refline_manager_->get_right_road_border_distance(),
          refline_manager_->get_s(), perception_sl.end_s);
      mutable_obs->SetRightSpaceBorder(right_border_dist_start,
                                       right_border_dist_end, perception_sl);

      double left_lane_dist_start = planning_math::interps(
          refline_manager_->get_left_lane_border_distance(),
          refline_manager_->get_s(), perception_sl.start_s);
      double left_lane_dist_end = planning_math::interps(
          refline_manager_->get_left_lane_border_distance(),
          refline_manager_->get_s(), perception_sl.end_s);
      mutable_obs->SetLeftSpaceLane(left_lane_dist_start, left_lane_dist_end,
                                    perception_sl);

      double right_lane_dist_start = planning_math::interps(
          refline_manager_->get_right_lane_border_distance(),
          refline_manager_->get_s(), perception_sl.start_s);
      double right_lane_dist_end = planning_math::interps(
          refline_manager_->get_right_lane_border_distance(),
          refline_manager_->get_s(), perception_sl.end_s);
      mutable_obs->SetRightSpaceLane(right_lane_dist_start, right_lane_dist_end,
                                     perception_sl);
    }
  }
  const auto &freespace_uss =
      parking_fusion_vision_info_
          .uss_fusion; // parking_vision_info_.free_space.d3_border_pts;
  const auto &freespace_fusion = parking_fusion_vision_info_.free_space;
  const auto &ego_state = ego_state_manager_.get_ego_state();

  // for (size_t point_id = 0; point_id < freespace_uss.size(); point_id++) {
  //   auto point_ptr =
  //       std::make_shared<Obstacle>(point_id,
  //       freespace_uss[point_id].position);

  //   auto mutable_obs = obstacle_manager_->add_uss_point(*point_ptr);
  // }

  // for (size_t point_id = 0; point_id < freespace_fusion.size(); point_id++) {
  //   if (nullptr == obs_ptr_) {
  //     obs_ptr_ = std::make_shared<Obstacle>(
  //         point_id, freespace_fusion[point_id].position);
  //   } else {
  //     obs_ptr_->update(Obstacle(point_id,
  //     freespace_fusion[point_id].position));
  //   }
  //   auto mutable_obs = obstacle_manager_->add_point(*obs_ptr_);
  // }

  size_t ground_line_id = 0;
  IndexedList<int, Obstacle> realtime_pts;
  auto &parking_slot_info = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->parking_slot_info;
  const auto &parking_lots_detection_result =
      parking_map_info_.parking_lots_detection_fusion_results;
  const auto &target_slot_info =
      std::find_if(parking_lots_detection_result.begin(),
                   parking_lots_detection_result.end(),
                   [&parking_slot_info](const ParkingLotDetectionInfo &data) {
                     return data.id == parking_slot_info.id;
                   });
  bool is_space_slot = false;
  bool is_oblique_slot = false;
  bool is_vertical_slot = false;
  bool is_parallel_slot = false;
  if (target_slot_info != parking_lots_detection_result.end()) {
    is_space_slot = target_slot_info->is_space_slot;
    parking_slot_info.is_space_slot = is_space_slot;
    is_oblique_slot = parking_slot_info.type.value == ParkingSlotType::OBLIQUE;
    is_vertical_slot =
        parking_slot_info.type.value == ParkingSlotType::PERPENDICULAR;
    is_parallel_slot =
        parking_slot_info.type.value == ParkingSlotType::PARALLEL;
    // std::cout << "----------> the type is verrical" << is_vertical_slot
    //           << "  oblique: " << is_oblique_slot << " is space slot: "
    //           << is_space_slot << std::endl;
  }
  // auto target_id = .id;
  for (auto &obs : parking_fusion_vision_info_.ground_line_fusion) {
    if (obs.id != parking_slot_info.id) {
      continue;
    }
    if (obs.type != GroundLineType::GROUND_LINE_USS_TYPE_STEP) {
      for (int i = 0; i < obs.pts.size(); i++) {
        bool is_pillar =
            obs.type == GroundLineType::GROUND_LINE_USS_TYPE_SIDE_POINT_PILLAR;
        if (nullptr == obs_ptr_) {
          obs_ptr_ = std::make_shared<Obstacle>(ground_line_id, obs.pts[i],
                                                is_pillar, obs.type);
        } else {
          obs_ptr_->update(
              Obstacle(ground_line_id, obs.pts[i], is_pillar, obs.type));
        }
        if (obs.type ==
            GroundLineType::GROUND_LINE_TYPE_USS_REALTIME_OBSTACLE) {
          auto mutable_obs = realtime_pts.Add(ground_line_id, *obs_ptr_);
        } else {
          auto mutable_obs = obstacle_manager_->add_point(*obs_ptr_);
        }
        ground_line_id++;
      }
    } else {
      for (int i = 0; i < obs.pts.size(); i++) {
        ObjectType type = ObjectType::STEP;
        if (VehicleParam::Instance()->car_type != "C03") {
          if (is_vertical_slot && !is_space_slot) {
            continue;
          }
        }
        if (nullptr == obs_ptr_) {
          obs_ptr_ = std::make_shared<Obstacle>(
              ground_line_id, obs.pts[i], false /*is_not_pillar*/, obs.type);
          *obs_ptr_->mutable_Type() = type;
        } else {
          obs_ptr_->update(Obstacle(ground_line_id, obs.pts[i],
                                    false /*is_not_pillar*/, obs.type));
          *obs_ptr_->mutable_Type() = type;
        }
        auto mutable_obs = obstacle_manager_->add_point(*obs_ptr_);
        ground_line_id++;
      }
    }

    if (is_simulation_) {
      for (int i = 0; i < obs.pts.size(); ++i) {
        bool is_pillar =
            obs.type == GroundLineType::GROUND_LINE_USS_TYPE_PILLAR;
        if (nullptr == obs_ptr_) {
          obs_ptr_ = std::make_shared<Obstacle>(
              ground_line_id,
              planning_math::LineSegment2d(
                  planning_math::Vec2d(obs.pts[i].x, obs.pts[i].y),
                  planning_math::Vec2d(obs.pts[i + 1].x, obs.pts[i + 1].y)),
              obs.type);
        } else {
          obs_ptr_->update(Obstacle(
              ground_line_id,
              planning_math::LineSegment2d(
                  planning_math::Vec2d(obs.pts[i].x, obs.pts[i].y),
                  planning_math::Vec2d(obs.pts[i + 1].x, obs.pts[i + 1].y)),
              obs.type));
        }
      }
    } else {
      if (obs.type == GroundLineType::GROUND_LINE_USS_TYPE_UNKNOWN ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_WALL ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_PILLAR ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_FENCE ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_STEP ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_SPECIAL ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_VEHICLE ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_ONLY_USS_UNKNOWN) {
        if (is_parallel_slot && obs.pts.size() % 2 == 0) {
          for (int i = 0; i < obs.pts.size(); i = i + 2) {
            if (nullptr == obs_ptr_) {
              obs_ptr_ = std::make_shared<Obstacle>(
                  ground_line_id,
                  planning_math::LineSegment2d(
                      planning_math::Vec2d(obs.pts[i].x, obs.pts[i].y),
                      planning_math::Vec2d(obs.pts[i + 1].x,
                                           obs.pts[i + 1].y)), 
                  obs.type);
            } else {
              obs_ptr_->update(
                  Obstacle(ground_line_id,
                           planning_math::LineSegment2d(
                               planning_math::Vec2d(obs.pts[i].x, obs.pts[i].y),
                               planning_math::Vec2d(obs.pts[i + 1].x,
                                                    obs.pts[i + 1].y)),
                           obs.type));
            }
            auto mutable_obs = obstacle_manager_->add_line(*obs_ptr_);
            ground_line_id++;
          }
        }
      }
    }
  }
  for (const auto &data : obstacle_manager_->get_points().Items()) {
    obstacle_manager_->add_all_point(*data);
  }
  for (const auto &data : realtime_pts.Items()) {
    obstacle_manager_->add_all_point(*data);
  }
  // auto & gl = obstacle_manager_->get_groundline_obstacles().Items();
  // for (auto  cl : gl) {
  //   std::cout <<"GroundLine Obj: " <<  (cl->Id()) <<std::endl;
  //   std::cout << (cl->PerceptionPoints().size()) <<std::endl;

  //   std::cout << (cl->PerceptionSLBoundary().start_s) <<std::endl;
  //   std::cout << (cl->PerceptionSLBoundary().start_l) <<std::endl;
  //   std::cout << (cl->PerceptionSLBoundary().end_s) <<std::endl;
  //   std::cout << (cl->PerceptionSLBoundary().end_l) <<std::endl;

  // }
}

void WorldModel::update_map_boundary() {
  auto map_obstacles = get_square_map_response_avp().obstacles;
  auto road_borders = get_square_map_response_avp().road_borders;
  auto pillars = map_obstacles.pillar;
  const auto &ego_state = ego_state_manager_.get_ego_state();
  const auto &freespace_uss =
      parking_fusion_vision_info_
          .uss_fusion; // parking_vision_info_.free_space.d3_border_pts;
  const auto &freespace_fusion = parking_fusion_vision_info_.free_space;
  double fs_size = freespace_fusion.size();
  int total_road_border_count = 0;
  double road_border_count = 0;
  for (int i = 0; i < road_borders.size(); i++) {
    if (road_borders[i].type == LaneLineType::PHYSICAL) {
      total_road_border_count += road_borders[i].pts.size();
      // std::cout<<"fsdebug border count =
      // "<<total_road_border_count<<std::endl;
      for (int j = 1; j < road_borders[i].pts.size(); j++) {
        if (std::hypot(road_borders[i].pts[j - 1].x - ego_state.ego_carte.x,
                       road_borders[i].pts[j - 1].y - ego_state.ego_carte.y) <
            10) {
          //   break;
          // }
          // else{
          auto &pt1 = road_borders[i].pts[j - 1];
          auto &pt2 = road_borders[i].pts[j];
          if (nullptr == obs_ptr_) {
            obs_ptr_ = std::make_shared<Obstacle>(
                fs_size + total_road_border_count + j, pt1, pt2);
          } else {
            obs_ptr_->update(
                Obstacle(fs_size + total_road_border_count + j, pt1, pt2));
          }
          auto mutable_obs = obstacle_manager_->add_road_border(*obs_ptr_);
          // std::cout <<"lxrdebug border := ["<<pt1.x<<","<<pt1.y<<"],
          // ["<<pt2.x<<","<<pt2.y<<"]"<<std::endl;
          SLBoundary perception_sl{};
          SLBoundary perception_sl_planning{};
          mutable_obs->ComputeSlLine(frenet_coord_planning_,
                                     &perception_sl_planning, true);
          mutable_obs->SetPerceptionSlBoundaryPlanning(perception_sl_planning);
          mutable_obs->ComputeSlLine(frenet_coord_, &perception_sl);
          mutable_obs->SetPerceptionSlBoundary(perception_sl);
          road_border_count++;
          continue;
        }
      }
    }
  }

  double pillar_count = 0;
  int total_pillar_count = 0;
  for (int i = 0; i < pillars.size(); i++) {
    bool flag = 0;
    total_pillar_count += pillars[i].pts.size();
    // std::cout<<"fsdebug pillar count = "<<total_pillar_count<<std::endl;
    for (int j = 1; j <= pillars[i].pts.size(); j++) {
      if (std::hypot(pillars[i].pts[j - 1].x - ego_state.ego_carte.x,
                     pillars[i].pts[j - 1].y - ego_state.ego_carte.y) > 10) {
        break;
      } else {
        auto &pt1 = pillars[i].pts[j - 1];
        auto &pt2 = pillars[i].pts[std::fmod(j, 4)];
        if (nullptr == obs_ptr_) {
          obs_ptr_ = std::make_shared<Obstacle>(
              fs_size + total_road_border_count + total_pillar_count + j, pt1,
              pt2);
        } else {
          obs_ptr_->update(Obstacle(fs_size + total_road_border_count +
                                        total_pillar_count + j,
                                    pt1, pt2));
        }
        auto mutable_obs = obstacle_manager_->add_pillar(*obs_ptr_);
        // std::cout <<"lxrdebug pillar := ["<<pt1.x<<","<<pt1.y<<"],
        // ["<<pt2.x<<","<<pt2.y<<"]"<<std::endl;
        SLBoundary perception_sl{};
        SLBoundary perception_sl_planning{};
        mutable_obs->ComputeSlLine(frenet_coord_planning_,
                                   &perception_sl_planning, true);
        mutable_obs->SetPerceptionSlBoundaryPlanning(perception_sl_planning);
        mutable_obs->ComputeSlLine(frenet_coord_, &perception_sl);
        mutable_obs->SetPerceptionSlBoundary(perception_sl);
        flag = 1;
        // continue;
      }
    }
    if (flag == 1)
      pillar_count++;
  }
}

// bool WorldModel::freespace_filter(const planning_math::LineSegment2d &line,
//                                   const std::string type,
//                                   const SLBoundary *line_sl) {
//   using namespace planning_math;

//   const auto &ego_pose = ego_state_manager_.get_ego_state().ego_pose;
//   Vec2d ego_vec2d(
//       ego_pose.x +
//           cos(ego_pose.theta) *
//           VehicleParam::Instance()->front_edge_to_center,
//       ego_pose.y +
//           sin(ego_pose.theta) *
//           VehicleParam::Instance()->front_edge_to_center);

//   LineSegment2d line1(ego_vec2d, line.start());
//   LineSegment2d line2(ego_vec2d, line.end());

//   Pose2D vehicle_head;
//   vehicle_head.x =
//       ego_pose.x +
//       cos(ego_pose.theta) * VehicleParam::Instance()->front_edge_to_center;
//   vehicle_head.y =
//       ego_pose.y +
//       sin(ego_pose.theta) * VehicleParam::Instance()->front_edge_to_center,
//   vehicle_head.theta = ego_pose.theta;

//   // auto start = tf2d_inv(vehicle_head, line.start());
//   // auto end   = tf2d_inv(vehicle_head, line.end());
//   auto start = line.start();
//   auto end = line.end();

//   // bool outside_road = false;
//   if (type == "lat") {
//     if (!base_lines_.IsInRoad(start.x(), start.y()) &&
//         !base_lines_.IsInRoad(end.x(), end.y())) {
//       // outside_road = true;
//       return true;
//     }
//   }

//   if (type == "lat") {
//     if (line.DistanceTo(ego_vec2d) > 20.0)
//       return true;
//   }

//   double forward_length = 8.0;
//   double backward_length = 13.0;
//   if (type == "lat") {
//     backward_length = 1.5;
//   }

//   bool forward_impassable_area =
//       ((line1.length() > forward_length) &&
//        std::fabs(planning_math::NormalizeAngle(line1.heading() -
//                                                ego_pose.theta)) < M_PI / 2)
//                                                &&
//       ((line2.length() > forward_length) &&
//        std::fabs(planning_math::NormalizeAngle(line2.heading() -
//                                                ego_pose.theta)) < M_PI / 2);
//   if (forward_impassable_area)
//     return true;
//   bool backward_impassable_area =
//       ((line1.length() > backward_length) &&
//        std::fabs(planning_math::NormalizeAngle(line1.heading() -
//                                                ego_pose.theta)) > M_PI / 2)
//                                                &&
//       ((line2.length() > backward_length) &&
//        std::fabs(planning_math::NormalizeAngle(line2.heading() -
//                                                ego_pose.theta)) > M_PI / 2);
//   if (backward_impassable_area)
//     return true;

//   bool ignore_in_corner = false;
//   if (type == "lat") {
//     if (line_sl != nullptr && line_sl->start_s > adc_sl_boundary_.end_s &&
//         line_sl->start_s < adc_sl_boundary_.end_s + 15) {
//       double delta_heading_angle =
//           std::fabs(frenet_coord_->GetRefCurveHeading(adc_sl_boundary_.end_s)
//           -
//                     frenet_coord_->GetRefCurveHeading(line_sl->start_s));
//       double s = clip(line_sl->start_s, frenet_coord_->GetSlength(), 0.0);
//       double vehicle_head_curvature = frenet_coord_->GetRefCurveCurvature(s);
//       ignore_in_corner =
//           (delta_heading_angle > M_PI / 3) && (vehicle_head_curvature < 0.1);
//       if (ignore_in_corner)
//         return true;
//     }
//   }

//   return false;
// }

bool WorldModel::extend_traj(std::vector<Pose2D> &traj_points) {
  // find nearest point on traj
  if (traj_points.empty())
    return false;
  std::vector<Pose2D>::iterator iter = traj_points.begin();
  std::vector<Pose2D>::iterator iter_min = traj_points.begin();

  double s = 0;
  double dist = 100.0;
  double dist_min = 100.0;
  auto ego_state = ego_state_manager_.get_ego_state();

  for (iter = traj_points.begin(); iter != traj_points.end(); iter++) {
    dist = std::hypot(iter->x - ego_state.ego_pose.x,
                      iter->y - ego_state.ego_pose.y);
    if (dist < dist_min) {
      dist_min = dist;
      iter_min = iter;
    }
  }
  for (iter = iter_min; iter != traj_points.end(); iter++) {
    s += std::hypot(iter->x - iter_min->x, iter->y - iter_min->y);
    iter_min = iter;
  }
  double s_extend;
  double step_size = 0.5;
  int count;
  if (s < 20.0) {
    s_extend = 20.0 - s;
    count = std::ceil(s_extend / step_size);
    for (int i = 0; i < count; i++) {
      Pose2D p;
      p.x = traj_points.back().x + step_size * cos(traj_points.back().theta);
      p.y = traj_points.back().y + step_size * sin(traj_points.back().theta);
      p.theta = traj_points.back().theta;
      traj_points.push_back(p);
    }
  }

  s_extend = 10.0;
  count = std::ceil(s_extend / step_size);
  std::vector<Pose2D> traj_front;
  for (int i = 1; i < count; i++) {
    Pose2D p;
    p.x =
        traj_points.front().x - i * step_size * cos(traj_points.front().theta);
    p.y =
        traj_points.front().y - i * step_size * sin(traj_points.front().theta);
    p.theta = traj_points.front().theta;
    traj_front.push_back(p);
  }
  std::reverse(traj_front.begin(), traj_front.end());
  traj_points.insert(traj_points.begin(), traj_front.begin(), traj_front.end());
  return true;
}

std::string WorldModel::get_neareat_refline_points() {
  auto &current_refline_points =
      map_info_manager_.get_map_info().current_refline_points;
  if (current_refline_points.empty())
    return "";
  for (size_t i = 0; i < current_refline_points.size(); i++) {
    // std::cout << "current_refline_points: " << current_refline_points[i].x <<
    // "  " << current_refline_points[i].y << std::endl;
  }
  double min_distance = 1e19;
  double index = 0;
  std::string index_str = current_refline_points[0].track_id;
  for (size_t i = 0;
       i < map_info_manager_.get_map_info().current_refline_points.size();
       i++) {
    double tmp_distance =
        std::hypot(current_refline_points[i].x - get_ego_state().ego_carte.x,
                   current_refline_points[i].y - get_ego_state().ego_carte.y);
    if (tmp_distance < min_distance) {
      min_distance = tmp_distance;
      index_str = current_refline_points[i].track_id;
      index = i;
    }
  }
  // std::cout << "indexmin: " << index << "  " << min_distance << std::endl;
  return index_str;
}

// void WorldModel::get_map_lateral_info_parking() {
//   ref_trajectory_info_last_ = ref_trajectory_info_;
//   // ref_info_index_last_ = ref_info_index_;
//   ref_trajectory_info_.clear();
//   // ref_info_index_.clear();

//   const Transform &car2enu = ego_state_manager_.get_car2enu();
//   Point2D point, point_fren;
//   Eigen::Vector3d car_point, enu_point;

//   double lane_width;
//   double left_lane_border_distance;
//   double left_road_border_distance;
//   double right_lane_border_distance;
//   double right_road_border_distance;

//   double last_lane_width = 3.5;
//   double last_left_lane_border_distance = 3.5;
//   double last_left_road_border_distance = 3.5;
//   double last_right_lane_border_distance = 3.5;
//   double last_right_road_border_distance = 3.5;

//   int index = 0;

//   double s_pt = -1.0;

//   if (map_info_manager_.get_map_info().current_refline_points.empty())
//     return;

//   for (auto p : map_info_manager_.get_map_info().current_refline_points) {

//     car_point.x() = p.x;
//     car_point.y() = p.y;
//     car_point.z() = 0.0;
//     // if (is_parking_svp()) {
//     //   enu_point = car2enu * car_point;
//     // }
//     // else {
//     enu_point = car_point;
//     // }

//     point.x = enu_point.x();
//     point.y = enu_point.y();
//     // std::cout << "srnbdebug: " << point.x << "  " << point.y << std::endl;
//     frenet_coord_->CartCoord2FrenetCoord(point, point_fren);

//     // TODO : replace hack width with lane width
//     if (point_fren.x > 1.0e-9 && point_fren.x < 100.) {
//       if (s_pt < 0.0) {
//         s_pt = std::hypot(point.x - get_map_info().ref_trajectory.front().x,
//                           point.y - get_map_info().ref_trajectory.front().y);
//         last_lane_width = std::abs(p.lane_width) > 10.0 ? 10.0 :
//         p.lane_width; last_left_lane_border_distance =
//             std::abs(p.left_lane_border_distance) > 10.0
//                 ? 3.5
//                 : p.left_lane_border_distance;
//         last_left_road_border_distance =
//             std::abs(p.left_road_border_distance) > 10.0
//                 ? 3.5
//                 : p.left_road_border_distance;
//         last_right_lane_border_distance =
//             std::abs(p.right_lane_border_distance) > 10.0
//                 ? 3.5
//                 : p.right_lane_border_distance;
//         last_right_road_border_distance =
//             std::abs(p.right_road_border_distance) > 10.0
//                 ? 3.5
//                 : p.right_road_border_distance;
//         // std::cout << "kkk: " << last_lane_width << " " <<
//         // last_left_lane_border_distance << " " <<
//         // last_left_road_border_distance << " "
//         //           << last_right_lane_border_distance << "  " <<
//         //           last_right_road_border_distance << std::endl;
//       } else
//         s_pt += std::hypot(point.x - ref_trajectory_info_.x.back(),
//                            point.y - ref_trajectory_info_.y.back());
//       ref_trajectory_info_.x.emplace_back(point.x);
//       ref_trajectory_info_.y.emplace_back(point.y);
//       refline_manager_->get_s().emplace_back(s_pt);
//       ref_trajectory_info_.l.emplace_back(point_fren.y);

//       if (std::abs(p.left_road_border_distance) > 100.0 ||
//           std::abs(p.right_road_border_distance) > 100.0)
//         ref_trajectory_info_.intersection.emplace_back(true);
//       else
//         ref_trajectory_info_.intersection.emplace_back(false);

//       lane_width = clip(double(p.left_road_border_distance +
//       p.right_road_border_distance), 10.0, 1.0); lane_width =
//       std::fabs(lane_width - 10.0) < 1e-2 ? last_lane_width : lane_width;

//       left_lane_border_distance =
//       clip(std::abs((double)p.left_lane_border_distance), 10.0, 0.0);
//       left_lane_border_distance = ref_trajectory_info_.intersection.back()?
//             last_left_lane_border_distance : left_lane_border_distance;
//       // if(std::abs(std::abs(p.left_lane_border_distance) -
//       std::abs(last_left_lane_border_distance)) > 2.0)
//       //   MSD_LOG(WARN, "left_lane_border_distance trembles exceeds 2.0
//       meter!");

//       left_road_border_distance =
//       clip(std::abs((double)p.left_road_border_distance), 10.0, 0.0);
//       left_road_border_distance = ref_trajectory_info_.intersection.back() ?
//             last_left_road_border_distance : left_road_border_distance;
//       // if(std::abs(std::abs(p.left_road_border_distance) -
//       std::abs(last_left_road_border_distance)) > 2.0)
//       //   MSD_LOG(WARN, "left_road_border_distance trembles exceeds 2.0
//       meter!");

//       right_lane_border_distance =
//           clip(std::abs((double)p.right_lane_border_distance), 10.0, 0.0);
//       right_lane_border_distance = ref_trajectory_info_.intersection.back()
//                                        ? last_right_lane_border_distance
//                                        : right_lane_border_distance;
//       // if(std::abs(std::abs(p.right_lane_border_distance) -
//       // std::abs(last_right_lane_border_distance)) > 2.0)
//       //   MSD_LOG(WARN, "right_lane_border_distance trembles exceeds 2.0
//       //   meter!");

//       right_road_border_distance =
//           clip(std::abs((double)p.right_road_border_distance), 10.0, 0.0);
//       right_road_border_distance = ref_trajectory_info_.intersection.back()
//                                        ? last_right_road_border_distance
//                                        : right_road_border_distance;
//       // if(std::abs(std::abs(p.right_road_border_distance) -
//       // std::abs(last_right_road_border_distance)) > 2.0)
//       //   MSD_LOG(WARN, "right_road_border_distance trembles exceeds 2.0
//       //   meter!");
//       // MSD_LOG(WARN, "last---lane_width, left_lane_border_distance,
//       // right_lane_border_distance, left_road_border_distance,
//       // right_road_border_distance=%f, %f, %f, %f, %f.",
//       //   last_lane_width, last_left_lane_border_distance,
//       //   last_right_lane_border_distance, last_left_road_border_distance,
//       //   last_right_road_border_distance);
//       // MSD_LOG(WARN, "new----lane_width, left_lane_border_distance,
//       // right_lane_border_distance, left_road_border_distance,
//       // right_road_border_distance=%f, %f, %f, %f, %f.",
//       //   lane_width, left_lane_border_distance, right_lane_border_distance,
//       //   left_road_border_distance, right_road_border_distance);

//       ref_trajectory_info_.lane_width.emplace_back(lane_width);
//       refline_manager_->get_left_lane_border_distance().emplace_back(
//           left_lane_border_distance);
//       refline_manager_->get_left_road_border_distance().emplace_back(
//           left_road_border_distance);
//       refline_manager_->get_right_lane_border_distance().emplace_back(
//           right_lane_border_distance);
//       refline_manager_->get_right_road_border_distance().emplace_back(
//           right_road_border_distance);
//       ref_trajectory_info_.curvature.emplace_back(
//           frenet_coord_->GetRefCurveCurvature(point_fren.x));
//       ref_trajectory_info_.track_id.emplace_back(p.track_id);
//       // std::cout << "curvature " << p.track_id << " " << point.x << " " <<
//       // point.y << " " << point_fren.x << "  " << point_fren.y << " "
//       //           << p.left_road_border_distance << "  " <<
//       //           p.right_road_border_distance
//       //           << "  " << p.left_lane_border_distance << "  " <<
//       //           p.right_lane_border_distance
//       //           << "  " <<
//       frenet_coord_->GetRefCurveCurvature(point_fren.x)
//       //           << " " << frenet_coord_->GetRefCurveHeading(point_fren.x)
//       <<
//       //           std::endl;

//       bool is_carriage_way =
//           (std::abs(left_lane_border_distance - left_road_border_distance) >
//                1.0 &&
//            std::abs((left_lane_border_distance + right_lane_border_distance)
//            -
//                     (left_road_border_distance - left_lane_border_distance))
//                     <
//                2.0);
//       ref_trajectory_info_.carriage_way.emplace_back(is_carriage_way);

//       last_lane_width = lane_width;
//       last_left_lane_border_distance = left_lane_border_distance;
//       last_left_road_border_distance = left_road_border_distance;
//       last_right_lane_border_distance = right_lane_border_distance;
//       last_right_road_border_distance = right_road_border_distance;
//     }
//   }
//   // // modify refline s
//   // for(size_t i = 1; i < refline_manager_->get_s().size() - 1; i++){
//   //   if(!(refline_manager_->get_s()[i] > refline_manager_->get_s()[i - 1]
//   &&
//   //   refline_manager_->get_s()[i] < refline_manager_->get_s()[i + 1]))
//   //     refline_manager_->get_s()[i] = (refline_manager_->get_s()[i - 1] +
//   //     refline_manager_->get_s()[i + 1]) * 0.5;
//   // }
// }

// bool WorldModel::cal_intersection(const double s) {
//   if (refline_manager_->get_s().empty())
//     return true;
//   if (refline_manager_->get_s().size() == 1)
//     return true;
//   for (std::size_t j = 0; j < refline_manager_->get_s().size() - 1; j++) {
//     if (s >= refline_manager_->get_s()[j] && s <= refline_manager_->get_s()[j
//     + 1]) {
//       return ref_trajectory_info_.intersection[j];
//     }
//   }
//   return ref_trajectory_info_.intersection.back();
// }

// bool WorldModel::cal_carriage_way(const double s) {
//   if (refline_manager_->get_s().empty())
//     return true;
//   if (refline_manager_->get_s().size() == 1)
//     return true;
//   for (std::size_t j = 0; j < refline_manager_->get_s().size() - 1; j++) {
//     if (s >= refline_manager_->get_s()[j] && s <= refline_manager_->get_s()[j
//     + 1]) {
//       return ref_trajectory_info_.carriage_way[j];
//     }
//   }
//   return ref_trajectory_info_.carriage_way.back();
// }

// double WorldModel::cal_theta(const double s){
//   if(refline_manager_->get_s().empty()) return true;
//   if(refline_manager_->get_s().size() == 1) return true;
//   for(std::size_t j = 0; j < refline_manager_->get_s().size() - 1; j++){
//     if(s >= refline_manager_->get_s()[j] && s <=
//     refline_manager_->get_s()[j+1]){
//       return ref_trajectory_info_.theta[j];
//     }
//   }
//   return ref_trajectory_info_.theta.back();
// }

// int WorldModel::cal_lane_id(const double s){
//   if(ref_info_index_.id_lane.empty()) return 0.0;
//   if(ref_info_index_.id_lane.size() == 1) return
//   ref_info_index_.id_lane.front(); for(size_t i = 0; i <
//   ref_info_index_.id_lane.size(); i++){
//     if(ref_info_index_.s_lane_start[i] > s)
//       return ref_info_index_.id_lane[std::max(0, int(i) - 1)];
//   }
//   return ref_info_index_.id_lane.back();
// }

void WorldModel::reset_square_map_response() {
  square_map_response_ = SquareMapResponse();
}

bool WorldModel::get_aimed_poi_projection_on_route(Pose2D *pose,
                                                   double lat_offset) {
  if (!frenet_coord_)
    return false;
  // // get center of amied poi
  // auto& poi_coreners = parking_map_info_.aimed_poi_info.corners;

  // double center_x = 0;
  // double center_y = 0;
  // for(Point3D& corner : poi_coreners)
  // {
  //   center_x += corner.x;
  //   center_y += corner.y;
  // }
  // center_x = center_x / poi_coreners.size();
  // center_y = center_y / poi_coreners.size();

  double frenet_poi_s =
      get_ego_state().ego_frenet.x + get_map_info().distance_to_aimed_poi;
  if (frenet_poi_s < 0)
    return false;
  Point2D frenet_poi_pose(frenet_poi_s, lat_offset);
  // Point2D frenet_poi_pose;
  // if(frenet_coord_->CartCoord2FrenetCoord(Point2D{center_x, center_y},
  // frenet_poi_pose) == TRANSFORM_STATUS::TRANSFORM_FAILED) {
  //   std::cout << "get_aimed_poi_projection_on_route failed due to invalid
  //   frenet coord!\n"; return false;
  // }
  // if(abs(frenet_poi_pose.y) > VehicleParams::Instance()->length()*2) //
  // ensure that poi near route
  //   return false;
  // frenet_poi_pose.y = lat_offset;
  Point2D cart_poi_pose;
  if (frenet_coord_->FrenetCoord2CartCoord(frenet_poi_pose, cart_poi_pose) ==
      TRANSFORM_STATUS::TRANSFORM_FAILED) {
    // std::cout << "get_aimed_poi_projection_on_route failed!\n";
    return false;
  }

  pose->x = cart_poi_pose.x;
  pose->y = cart_poi_pose.y;
  pose->theta = frenet_coord_->GetRefCurveHeading(frenet_poi_pose.x);

  return true;
}

void WorldModel::update_scenes() {
  //   std::vector<string> ramp_list = {"49286", "34080", "33165", "33169",
  //   "32989", "33169", "33026", "33019", "48645", "34091", "33059", "50584",
  //   "33006", "33150", "33148", "34172", "33030"};

  //   // std::vector<string> entrance_list = {"32989-4", "32989-5", "32989-6",
  //   "32989-7", "32989-8",
  //   //             "32989-9", "32989-10", "32989-11", "32989-12", "32989-13",
  //   "32989-44"};

  //   auto id = get_neareat_refline_points();
  // // std::cout << "srbdebug_hack:  " << id << std::endl;
  //   if(id == "") return;

  auto &planning_result =
      PlanningContext::Instance()->mutable_planning_status()->planning_result;
  int &scene_avp = planning_result.scene_avp;
  //   scene_avp = PlanningContext::Instance()->add_scene(scene_avp,
  //   ParkingSceneType::SCENE_AVP); scene_avp =
  //   PlanningContext::Instance()->remove_scene(scene_avp,
  //   ParkingSceneType::SCENE_RAMP); for(size_t i = 0; i < ramp_list.size();
  //   i++){
  //     if(id.find(ramp_list[i]) != std::string::npos){
  //       scene_avp = PlanningContext::Instance()->add_scene(scene_avp,
  //       ParkingSceneType::SCENE_RAMP); break;
  //     }
  //     else
  //       scene_avp = PlanningContext::Instance()->remove_scene(scene_avp,
  //       ParkingSceneType::SCENE_RAMP);
  //   }

  if (map_info_manager_.get_map_info().is_on_ramp) {
    scene_avp = PlanningContext::Instance()->add_scene(
        scene_avp, ParkingSceneType::SCENE_RAMP);
  } else {
    scene_avp = PlanningContext::Instance()->remove_scene(
        scene_avp, ParkingSceneType::SCENE_RAMP);
  }
  // for(int i = 0; i < entrance_list.size(); i++)
  //   if(id.find(entrance_list[i]) != std::string::npos)
  //     planning_result.scene_avp += ParkingSceneType::SCENE_ENTRANCE;

  const ObstacleManager &obs_manager = obstacle_manager();
  auto ego_frenet = get_ego_state().ego_frenet;
  for (auto &obs : obs_manager.get_gates().Items()) {
    SLBoundary slboundary = obs->PerceptionSLBoundary();
    // std::cout << "srbdebug_gate: " << obs->Id() << " " << slboundary.start_s
    // << " "
    //           << "  " << slboundary.end_s << " " << slboundary.start_l << " "
    //           << slboundary.end_l << std::endl;
    if (slboundary.start_s < (ego_frenet.x + 25.0) &&
        slboundary.end_s > (ego_frenet.x - 5.0) &&
        std::abs(slboundary.start_l) < 3.0) {
      scene_avp = PlanningContext::Instance()->add_scene(
          scene_avp, ParkingSceneType::SCENE_ENTRANCE);
      break;
    } else
      scene_avp = PlanningContext::Instance()->remove_scene(
          scene_avp, ParkingSceneType::SCENE_ENTRANCE);
  }
}

} // namespace parking

} // namespace msquare
