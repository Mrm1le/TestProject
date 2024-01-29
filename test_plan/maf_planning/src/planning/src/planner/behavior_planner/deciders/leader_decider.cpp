#include "planner/behavior_planner/deciders/leader_decider.h"
#include "common/math/math_utils.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planning/common/common.h"

namespace msquare {

namespace parking {

namespace {
constexpr int kExtendPathPointNumber = 8;
constexpr double kExceptPathPointS = 0.15;
// refactor mpc path
constexpr double kMpcDeltaLength = 0.04;
constexpr double kConsiderMpcLength = 1.5;
constexpr int kConsiderMpcSize = 32; // 27 + 5. temp solution
} // namespace

LeaderDecider::LeaderDecider(const std::shared_ptr<WorldModel> &world_model) {
  world_model_ = world_model;
  collision_checker_ = CollisionChecker();
  // ego_model_ = EgoModelManager();
}

bool LeaderDecider::execute() {
  // find leader
  if (world_model_ == nullptr) {
    MSD_LOG(INFO, "world model is none!");
    return false;
  }
  construct_exclusve_box();
  const double leader_range = 1.1;
  obstacles_ = world_model_->obstacle_manager().get_obstacles().Items();
  auto &planning_result =
      PlanningContext::Instance()->planning_status().planning_result;
  const StatusType &status_type =
      PlanningContext::Instance()->planning_status().scenario.status_type;
  // auto lateral_output =
  // PlanningContext::Instance()->mutable_parking_lateral_behavoir_planner_output();
  // auto &path_points_real =
  // PlanningContext::Instance()->planning_status().planning_result.path_points_real;
  double ego_s = 0.0;
  if (planning_result.gear == GearState::REVERSE) {
    ego_s = world_model_->get_ego_state_planning().ego_frenet.x +
            VehicleParam::Instance()->back_edge_to_center;
  } else {
    ego_s = world_model_->get_ego_state_planning().ego_frenet.x +
            VehicleParam::Instance()->front_edge_to_center;
  }

  // double lat_offset = 0.0;
  if (msquare::CarParams::GetInstance()
          ->car_config.lon_config.use_sop_algorithm) {
    (void)clip_traj_sop();
  } else {
    (void)clip_traj();
  }
  const planning_math::Box2d exclusive_box =
      PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .exclusive_box;
  bool parking_lot_box_available =
      (PlanningContext::Instance()
           ->mutable_parking_behavior_planner_output()
           ->parking_lot != nullptr);
  double dist_to_apa_target_point = 100000;
  if (parking_lot_box_available) {
    Pose2D target_pose = PlanningContext::Instance()
                             ->parking_behavior_planner_output()
                             .parking_lot->getParkingInPose(
                                 VehicleParam::Instance()->front_edge_to_center,
                                 VehicleParam::Instance()->back_edge_to_center,
                                 0, world_model_->get_ego_state().ego_pose);
    dist_to_apa_target_point =
        std::hypot(target_pose.x - world_model_->get_ego_state().ego_pose.x,
                   target_pose.y - world_model_->get_ego_state().ego_pose.y);
  }
  //   std::vector<int> static_obstacle_beside_poi_id;
  //   if (status_type == StatusType::APA || status_type == StatusType::APOA){
  //     static_obstacle_beside_poi_id = get_static_obstacle_beside_poi();
  // // std::cout<<"static size =
  // "<<static_obstacle_beside_poi_id.size()<<std::endl;
  //   }
  for (const auto &obstacle : obstacles_) {
    // std::cout << obstacle->Id()  << " " <<
    // obstacle->PerceptionBoundingBox().center_x() << " " <<
    // obstacle->PerceptionBoundingBox().center_y() << " "
    // <<obstacle->PerceptionBoundingBox().heading() << " " <<
    // obstacle->PerceptionBoundingBox().length() << " " <<
    // obstacle->PerceptionBoundingBox().width() <<  std::endl;
    if ((status_type == StatusType::APA || status_type == StatusType::APOA) &&
        !obstacle->Is_confident()) {
      continue;
    }

    planning_math::Box2d obs_bbox = obstacle->PerceptionBoundingBox();

    if (status_type == StatusType::APA) {
      // const std::vector<planning_math::Vec2d> corners =
      // obstacle->PerceptionBoundingBox().GetAllCorners(); bool is_exclusive =
      // true; for (auto &corner : corners){
      //   if(!exclusive_box.IsPointIn(corner)){
      //     is_exclusive = false;
      //     break;
      //   }
      // }
      bool is_exclusive =
          !world_model_->is_simulation() && (dist_to_apa_target_point < 1.0) &&
          exclusive_box.HasOverlap(obstacle->PerceptionBoundingBox());
      if (parking_lot_box_available && is_exclusive) {
        const planning_math::Box2d parking_lot_box =
            PlanningContext::Instance()
                ->parking_behavior_planner_output()
                .parking_lot->getBox();
        is_exclusive = is_exclusive && !parking_lot_box.HasOverlap(
                                           obstacle->PerceptionBoundingBox());
      }

      // auto &nearby_obstacle_id = PlanningContext::Instance()
      //                                ->parking_behavior_planner_output()
      //                                .parking_slot_info.nearby_obstacle_id;
      // for (auto id : nearby_obstacle_id) {
      //   if (id == obstacle->Id()) {
      //     is_exclusive = true;
      //   }
      // }

      if (is_exclusive) {
        continue;
      }
    }
    // if (status_type == StatusType::APA || status_type == StatusType::APOA) {
    //   // for(int i=0; i<static_obstacle_beside_poi_id.size();++i){
    //   //   auto static_obstacle =
    //   //
    //   world_model_->mutable_obstacle_manager().find_static_obstacle(static_obstacle_beside_poi_id.at(i));
    //   //   if(static_obstacle == nullptr){
    //   //     continue;
    //   //   }
    //   //
    //   if(static_obstacle->PerceptionPolygon().ComputeIoU(obstacle->PerceptionPolygon())
    //   //   > 0.2){
    //   //     std::cout<<"removed fusion id : static id = "<<obstacle->Id()<<"
    //   :
    //   //     "<<static_obstacle->Id()<< "
    //   //
    //   "<<static_obstacle->PerceptionPolygon().ComputeIoU(obstacle->PerceptionPolygon())<<std::endl;
    //   //     overlap_fusion_flag = true;
    //   //     continue;
    //   //   }
    //   // }
    //   if (std::hypot(obstacle->GetCartePosewrtEgo().x +
    //                      VehicleParam::Instance()->length / 2.0,
    //                  obstacle->GetCartePosewrtEgo().y) < 10.0 &&
    //       obstacle->Type() == ObjectType::COUPE) {
    //     for (auto &static_obs :
    //          world_model_->obstacle_manager().get_static_obstacles().Items())
    //          {
    //       if (obstacle->PerceptionPolygon().DistanceTo(
    //               static_obs->PerceptionPolygon()) > 0.1)
    //         continue;
    //       if (obstacle->PerceptionPolygon().ComputeIoU(
    //               static_obs->PerceptionPolygon()) > 0.6) {
    //         obs_bbox = static_obs->PerceptionBoundingBox();
    //         world_model_->mutable_obstacle_manager()
    //             .find_obstacle(obstacle->Id())
    //             ->SetStatic(
    //                 1); // compulsorily set static for overlapped obstacle
    //         break;
    //       }
    //     }
    //   }
    // }

    // SLBoundary sl_boundary;
    // sl_boundary = obstacle->PerceptionSLBoundaryPlanning();
    // if
    // (std::find(lateral_output->sidepass_left_ignore.begin(),lateral_output->sidepass_left_ignore.end(),obstacle->Id())
    // != lateral_output->sidepass_left_ignore.end()){
    //   sl_boundary = obstacle->PerceptionSLBoundary();
    // }
    // else
    // {
    //   sl_boundary = obstacle->PerceptionSLBoundaryOrigin();
    // }

    ObjectDecisionType leader_decision;
    auto mutable_leader_decision = leader_decision.mutable_lead();
    // if
    // (PlanningContext::Instance()->parking_behavior_planner_output().planner_type
    // == PlannerType::PARKING_LATERAL){
    //   lat_offset = calc_lat_offset(path_points_real, (sl_boundary.start_s +
    //   sl_boundary.end_s) / 2);
    // }
    // if(obstacle->Id()==1014856){
    //   CollisionCheckStatus result =
    //   calc_bounding_box_collision_dist(obstacle->PerceptionBoundingBox());
    //   std::cout<<"modeldebug result = "<<result.is_collision<<"
    //   "<<result.s<<" "<<result.min_distance<<std::endl;
    // }
    // bool is_front_car = (sl_boundary.start_s - ego_s) * (sl_boundary.end_s -
    // ego_s) < 0.0 || (sl_boundary.start_s - ego_s) > 0.0 ; if
    // (std::abs(sl_boundary.start_l) > 5.0 && std::abs(sl_boundary.end_l)
    // > 5.0) {
    //   continue;
    // }
    // else if ((sl_boundary.start_s - ego_s) > 12.0 && (sl_boundary.end_s -
    // ego_s) > 12.0) {
    //   continue;
    // }
    bool is_car_and_is_not_static = false;
    if (!obstacle->IsStatic() && obstacle->Type() == ObjectType::COUPE &&
        VehicleParam::Instance()->car_type == "C03") {
      is_car_and_is_not_static = true;
    }
    CollisionCheckStatus result =
        calc_bounding_box_collision_dist(obs_bbox, obstacle);
    if (result.s < 0.5 && result.is_collision &&
        (status_type == StatusType::APA || status_type == StatusType::APOA) &&
        !is_car_and_is_not_static &&
        obstacle->Type() != ObjectType::PEDESTRIAN) {
      world_model_->mutable_obstacle_manager()
          .find_obstacle(obstacle->Id())
          ->SetIsNeedFilletCutting(1);
      planning_math::Polygon2d fillet_cutting_polygon =
          generate_fillet_cutting_polygon(obs_bbox,
                                          obstacle->FilletCuttingLength());
      result = calc_polygon_collision_dist(obs_bbox, fillet_cutting_polygon);
    }
    MSD_LOG(ERROR, "the obstacle type is: %d, is static : %d, the result is %d",
            int(obstacle->Type()), obstacle->IsStatic(), result.is_collision);
    mutable_leader_decision->distance_s = result.s;
    mutable_leader_decision->min_distance = result.min_distance;
    mutable_leader_decision->type = obstacle->Type();
    mutable_leader_decision->is_collision = result.is_collision;

    (void)world_model_->mutable_obstacle_manager()
        .add_parking_longitudinal_decision("leader_decider", obstacle->Id(),
                                           leader_decision);
  }
  return true;
}

bool LeaderDecider::check_traj(std::vector<Pose2D> &traj_pose_array_) {
  // find leader
  if (world_model_ == nullptr) {
    MSD_LOG(INFO, "world model is none!");
    return false;
  }

  auto &ego_state = world_model_->get_ego_state();

  if (PlanningContext::Instance()->planning_status().fs_block &&
      !PlanningContext::Instance()->planning_status().is_pullover) {
    auto frenet_coor = world_model_->get_frenet_coord();
    Point2D fs_point = PlanningContext::Instance()
                           ->longitudinal_behavior_planner_output()
                           .last_block_fs;
    Point2D frenet_fs_point;

    if (frenet_coor == nullptr ||
        world_model_->get_frenet_coord()->CartCoord2FrenetCoord(
            fs_point, frenet_fs_point) == TRANSFORM_FAILED ||
        ego_state.ego_frenet.x - frenet_fs_point.x < 5.0) {
      return false;
    }
  }
  obstacles_ = world_model_->obstacle_manager().get_obstacles().Items();
  auto &planning_result =
      PlanningContext::Instance()->planning_status().planning_result;

  std::vector<PathPoint> origin_mpc_trajectory =
      world_model_->get_mpc_trajectory();
  std::vector<Pose2D> &trajectory =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->trajectory;
  std::vector<Pose2D> &mpc_trajectory =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->mpc_trajectory;
  trajectory.clear();
  mpc_trajectory.clear();
  double dist = 100.0;
  double dist_min = 100.0;

  std::vector<Pose2D>::iterator iter = traj_pose_array_.begin();
  std::vector<Pose2D>::iterator iter_min = traj_pose_array_.begin();
  Pose2D traj_point;
  if (traj_pose_array_.size() == 0) {
    return false;
  }

  for (iter = traj_pose_array_.begin(); iter != traj_pose_array_.end();
       iter++) {
    dist = std::hypot(iter->x - ego_state.ego_pose.x,
                      iter->y - ego_state.ego_pose.y);
    if (dist < dist_min) {
      dist_min = dist;
      iter_min = iter;
    }
  }
  for (iter = iter_min; iter != traj_pose_array_.end(); iter++) {
    // std::cout << iter->x << ", "<< iter->y << ", " << iter->theta << ";" <<
    // std::endl;
    traj_point.x = iter->x;
    traj_point.y = iter->y;
    traj_point.theta = iter->theta;
    trajectory.push_back(traj_point);
    iter_min = iter;
  }

  // mpc
  for (size_t i = 0; i < origin_mpc_trajectory.size(); ++i) {
    Pose2D mpc_point;
    mpc_point.x = origin_mpc_trajectory.at(i).x;
    mpc_point.y = origin_mpc_trajectory.at(i).y;
    mpc_point.theta = origin_mpc_trajectory.at(i).theta;

    mpc_trajectory.push_back(mpc_point);
    // if (i > origin_mpc_trajectory.size() / 2){
    //   break;
    // }
  }

  for (const auto &obstacle : obstacles_) {
    if (obstacle->IsLonHighspeedWrtEgo() == 1 ||
        obstacle->IsLonHighspeedWrtFrenet() == 1) {
      continue;
    }
    CollisionCheckStatus result =
        calc_bounding_box_collision_dist(obstacle->PerceptionBoundingBox());
    if (result.is_collision) {
      return false;
    }
  }
  // std::cout << "is_teb_ok-check_traj success\n";
  return true;
}

double LeaderDecider::calc_lat_offset(std::vector<PathPoint> path_points_real,
                                      const double s) {
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

bool LeaderDecider::clip_traj_sop() {
  double max_length = 20.0;
  auto &planning_result =
      PlanningContext::Instance()->planning_status().planning_result;
  // auto &mpc_trajectory_enu = PlanningContext::Instance()
  //                                ->mutable_planning_status()
  //                                ->planning_result.mpc_path;
  auto &ego_state = world_model_->get_ego_state();
  // const StatusType &status_type =
  // PlanningContext::Instance()->planning_status().scenario.status_type;

  double ego_s = 0.0;
  std::vector<Pose2D> &trajectory =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->trajectory;
  std::vector<double> &curvatures =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->curvatures;
  std::vector<double> &relative_s =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->relative_s;
  std::vector<Pose2D> &mpc_trajectory =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->mpc_trajectory;
  bool &deviated = PlanningContext::Instance()
                       ->mutable_longitudinal_behavior_planner_output()
                       ->deviated;
  double &ego_lat_diff = PlanningContext::Instance()
                             ->mutable_longitudinal_behavior_planner_output()
                             ->ego_lat_diff;
  std::vector<double> &planning_mpc_diff =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->planning_mpc_diff;
  trajectory.clear();
  mpc_trajectory.clear();
  curvatures.clear();
  relative_s.clear();
  // mpc_trajectory_enu.clear();
  planning_mpc_diff.clear();
  std::vector<PathPoint> origin_mpc_trajectory =
      world_model_->get_mpc_trajectory();
  // std::vector<PathPoint> mpc_trajectory_undiluted =
  //     world_model_->get_mpc_trajectory_undiluted();
  // std::vector<Pose2D> mpc_trajectory_undiluted;
  // int mpc_trajectory_undiluted_size = origin_mpc_trajectory_undiluted.size()
  // /2;
  ego_s = ego_state.ego_frenet.x;
  // std::cout << "ego_s : " << ego_s << std::endl;
  // std::cout << "ego_l : " << ego_state.ego_frenet.y << std::endl;
  // std::cout << "ego_theta : " << ego_state.ego_pose.theta << std::endl;
  // std::cout << "ego_x : " << ego_state.ego_pose.x << std::endl;
  // std::cout << "ego_y : " << ego_state.ego_pose.y << std::endl;
  double s = 0.0;
  double traj_length = 0.0;
  double dist = 100.0;
  double dist_min = 100.0;
  deviated = false;

  // for (size_t i = 0; i < origin_mpc_trajectory.size(); ++i) {
  //   PathPose mpc_point;
  //   mpc_point.pos.x = origin_mpc_trajectory.at(i).x;
  //   mpc_point.pos.y = origin_mpc_trajectory.at(i).y;
  //   mpc_point.pos.z = 0.0; //
  //   world_model_->get_ego_state().ego_enu.position.z;
  //   // auto q = tf::createQuaternionFromRPY(0, 0,
  //   // origin_mpc_trajectory.at(i).theta);
  //   using namespace Eigen;
  //   // Roll pitch and yaw in Radians
  //   Quaternionf q;
  //   q = AngleAxisf(0, Vector3f::UnitX()) * AngleAxisf(0, Vector3f::UnitY()) *
  //       AngleAxisf(origin_mpc_trajectory.at(i).theta, Vector3f::UnitZ());
  //   // std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
  //   mpc_point.orient.x = q.x();
  //   mpc_point.orient.y = q.y();
  //   mpc_point.orient.z = q.z();
  //   mpc_point.orient.w = q.w();
  //   mpc_trajectory_enu.emplace_back(mpc_point);
  // }

  // planning
  std::vector<Pose2D> traj_pose_array_ = planning_result.traj_pose_array;
  std::vector<Pose2D>::iterator iter = traj_pose_array_.begin();
  std::vector<Pose2D>::iterator iter_min = traj_pose_array_.begin();
  std::vector<float> traj_vel_array_ = planning_result.traj_vel_array;
  const std::vector<float> &traj_curvature = planning_result.traj_curvature;

  // std::vector<double> traj_dist_to_ego;
  // std::vector<double> mpc_dist_to_ego;
  Pose2D traj_point;
  int index = 0;
  int index_min = 0;
  for (iter = traj_pose_array_.begin(); iter != traj_pose_array_.end();
       iter++) {
    dist = std::hypot(iter->x - ego_state.ego_pose.x,
                      iter->y - ego_state.ego_pose.y);
    if (dist < dist_min) {
      dist_min = dist;
      iter_min = iter;
      index_min = index;
    }
    index++;
  }
  Pose2D projection_point;
  if (traj_pose_array_.size() == 0) {
    deviated = true;
  }
  if (iter_min != traj_pose_array_.begin()) {
    projection_point = planning_math::calc_projection_point(
        Pose2D((iter_min - 1)->x, (iter_min - 1)->y, (iter_min - 1)->theta),
        Pose2D((iter_min)->x, (iter_min)->y, (iter_min)->theta),
        Pose2D(ego_state.ego_pose.x, ego_state.ego_pose.y, 0.0));
    // std::cout << "TB deviation1: " << std::hypot(projection_point.x -
    // ego_state.ego_pose.x, projection_point.y - ego_state.ego_pose.y) <<
    // std::endl; std::cout << "TB deviation1: " << ego_state.ego_pose.theta <<
    // " " << projection_point.theta << std::endl;
    if (std::hypot(projection_point.x - ego_state.ego_pose.x,
                   projection_point.y - ego_state.ego_pose.y) > 0.3 ||
        std::abs(ego_state.ego_pose.theta - projection_point.theta) > 0.2) {
      deviated = true;
    }
    if (std::abs(traj_vel_array_.at(index_min - 1)) > 0.01) {
      s += std::hypot(projection_point.x - iter_min->x,
                      projection_point.y - iter_min->y);
      if (s > 0.01) {
        traj_point.x = projection_point.x;
        traj_point.y = projection_point.y;
        traj_point.theta = projection_point.theta;
        trajectory.push_back(traj_point);
        curvatures.push_back(
            0.5 * (traj_curvature[index_min - 1] + traj_curvature[index_min]));
        relative_s.push_back(s);
        // std::cout << projection_point.x << ", "<< projection_point.y << ", "
        // << projection_point.theta << ";" <<  std::endl;
      }
    }
  }
  if ((iter_min) != traj_pose_array_.end() &&
      (iter_min + 1) != traj_pose_array_.end()) {
    projection_point = planning_math::calc_projection_point(
        Pose2D((iter_min)->x, (iter_min)->y, (iter_min)->theta),
        Pose2D((iter_min + 1)->x, (iter_min + 1)->y, (iter_min + 1)->theta),
        Pose2D(ego_state.ego_pose.x, ego_state.ego_pose.y, 0.0));
    // std::cout << "TB deviation2: " << std::hypot(projection_point.x -
    // ego_state.ego_pose.x, projection_point.y - ego_state.ego_pose.y) <<
    // std::endl; std::cout << "TB deviation2: " << ego_state.ego_pose.theta <<
    // " " << projection_point.theta << std::endl;
    if (std::hypot(projection_point.x - ego_state.ego_pose.x,
                   projection_point.y - ego_state.ego_pose.y) > 0.3 ||
        std::abs(ego_state.ego_pose.theta - projection_point.theta) > 0.2) {
      deviated = true;
    }
    if (std::abs(traj_vel_array_.at(index_min)) > 0.01 && trajectory.empty()) {
      s += std::hypot(projection_point.x - (iter_min + 1)->x,
                      projection_point.y - (iter_min + 1)->y);
      index_min++;
      iter_min++;
      if (s > 0.01) {
        traj_point.x = projection_point.x;
        traj_point.y = projection_point.y;
        traj_point.theta = projection_point.theta;
        trajectory.push_back(traj_point);
        curvatures.push_back(
            0.5 * (traj_curvature[index_min + 1] + traj_curvature[index_min]));
        relative_s.push_back(s);
      }
    }
  }
  index = index_min;
  bool reach_end = false;
  for (iter = iter_min; iter != traj_pose_array_.end(); iter++) {
    s += std::hypot(iter->x - iter_min->x, iter->y - iter_min->y);
    // std::cout << iter->x << ", "<< iter->y << ", " << iter->theta << ";" <<
    // std::endl;
    traj_point.x = iter->x;
    traj_point.y = iter->y;
    traj_point.theta = iter->theta;
    trajectory.push_back(traj_point);
    curvatures.push_back(
        traj_curvature[(unsigned int)(iter - traj_pose_array_.begin())]);
    relative_s.push_back(s);
    iter_min = iter;
    if (std::abs(traj_vel_array_.at(index)) < 0.01 && !reach_end) {
      traj_length = s;
      reach_end = true;
    }
    if (s > max_length) {
      break;
    }
    index++;
  }
  if (!reach_end) {
    traj_length = s;
  }
  // PlanningContext::Instance()
  //     ->mutable_longitudinal_behavior_planner_output()
  //     ->traj_length = traj_length;
  double remain_s_use_control_way = planning_math::getRemainDistanceControlWay(
      traj_pose_array_, traj_vel_array_, ego_state.ego_pose);

  // set trajectory's s
  calculatePathS(&trajectory);
  calculatePathKappa(&trajectory);

  // get plan path
  std::vector<Pose2D> &plan_path_output =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->plan_path;
  const std::vector<Pose2D> &trajectory_const =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->trajectory;
  plan_path_output.clear();
  if (trajectory_const.size() > kExtendPathPointNumber) {
    for (auto iter = trajectory_const.begin();
         iter != trajectory_const.end() - kExtendPathPointNumber; ++iter) {
      plan_path_output.emplace_back(
          Pose2D(iter->x, iter->y, iter->theta, iter->s, iter->kappa()));
    }
  } else if (trajectory_const.size() > 0) {
    for (auto iter = trajectory_const.begin(); iter != trajectory_const.end();
         ++iter) {
      if (iter->s > kExceptPathPointS)
        break;
      plan_path_output.emplace_back(
          Pose2D(iter->x, iter->y, iter->theta, iter->s, iter->kappa()));
    }
  }

  PlanningContext::Instance()
      ->mutable_longitudinal_behavior_planner_output()
      ->traj_length = remain_s_use_control_way;
  MSD_LOG(ERROR,
          "the planning remaining distance is: %f, the control remaining "
          "distance is:%f",
          traj_length, remain_s_use_control_way);
  // std::cout << "TBDEBUG: " << index << " "
  //           << index_min << " "
  //           << traj_length << " "
  //           << s << " "
  //           << deviated << " "
  //           << std::endl;
  // std::cout << "mpc:" <<  std::endl;
  // mpc_trajectory_undiluted_size = origin_mpc_trajectory_undiluted.size() /2;
  if (PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .planner_type == PlannerType::PARKING_LATERAL) {
    // PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()->traj_length
    // = s; mpc
    s = 0;
    for (size_t i = 0; i < origin_mpc_trajectory.size(); ++i) {
      // std::cout<<"mpcdebug origin size i = "<<origin_mpc_trajectory.size()<<"
      // "<<i<<std::endl;
      Pose2D mpc_point;
      mpc_point.x = origin_mpc_trajectory.at(i).x;
      mpc_point.y = origin_mpc_trajectory.at(i).y;
      mpc_point.theta = origin_mpc_trajectory.at(i).theta;
      // std::cout << "mpcdebug "<<i<<" : "<<mpc_point.x << ", "<< mpc_point.y
      // << ", " << mpc_point.theta << ";" <<  std::endl;
      if (mpc_trajectory.size() > 0) {
        s += std::hypot(mpc_trajectory.back().x - mpc_point.x,
                        mpc_trajectory.back().y - mpc_point.y);
      }

      if (/*i > origin_mpc_trajectory.size() / 2 ||*/ s > 1.5) {
        if (!deviated) {
          break;
        }
        // else if (s > 1.5 && i > origin_mpc_trajectory.size() / 2){
        //   break;
        // }
      }
      mpc_trajectory.push_back(mpc_point);
    }
    // for(size_t i = 0; i<mpc_trajectory_undiluted_size; ++i){

    //   //
    //   mpc_dist_to_ego.emplace_back(std::hypot(origin_mpc_trajectory_undiluted.at(i).x
    //   - ego_state.ego_pose.x, origin_mpc_trajectory_undiluted.at(i).y -
    //   ego_state.ego_pose.y)); Pose2D mpc_point; mpc_point.x =
    //   origin_mpc_trajectory_undiluted.at(i).x; mpc_point.y =
    //   origin_mpc_trajectory_undiluted.at(i).y; mpc_point.theta =
    //   origin_mpc_trajectory_undiluted.at(i).theta;

    //   mpc_trajectory_undiluted.emplace_back(mpc_point);
    // }

  } else if (PlanningContext::Instance()
                 ->parking_behavior_planner_output()
                 .planner_type == PlannerType::OPENSPACE) {
    // mpc
    s = 0;
    for (size_t i = 0; i < origin_mpc_trajectory.size(); ++i) {
      Pose2D mpc_point;
      mpc_point.x = origin_mpc_trajectory.at(i).x;
      mpc_point.y = origin_mpc_trajectory.at(i).y;
      mpc_point.theta = origin_mpc_trajectory.at(i).theta;
      // std::cout << mpc_point.x << ", "<< mpc_point.y << ", " <<
      // mpc_point.theta << ";" <<  std::endl;
      if (mpc_trajectory.size() > 0) {
        s += std::hypot(mpc_trajectory.back().x - mpc_point.x,
                        mpc_trajectory.back().y - mpc_point.y);
      }

      // if (i > origin_mpc_trajectory.size() / 2){
      //   break;
      // }
      mpc_trajectory.push_back(mpc_point);
    }
    // for(size_t i = 0; i<mpc_trajectory_undiluted_size; ++i){

    //   mpc_dist_to_ego.emplace_back(std::hypot(origin_mpc_trajectory_undiluted.at(i).x
    //   - ego_state.ego_pose.x, origin_mpc_trajectory_undiluted.at(i).y -
    //   ego_state.ego_pose.y)); Pose2D mpc_point; mpc_point.x =
    //   origin_mpc_trajectory_undiluted.at(i).x; mpc_point.y =
    //   origin_mpc_trajectory_undiluted.at(i).y; mpc_point.theta =
    //   origin_mpc_trajectory_undiluted.at(i).theta;

    //   mpc_trajectory_undiluted.emplace_back(mpc_point);
    // }
  }

  calculatePathS(&mpc_trajectory);
  calculatePathKappa(&mpc_trajectory);
  if (!mpc_trajectory.empty()) {
    std::vector<Pose2D> mpc_refactored =
        LeaderDecider::InterpolatePathPoints(mpc_trajectory, kMpcDeltaLength);
    mpc_trajectory = mpc_refactored;
  }

  // planning_mpc_diff.resize(mpc_trajectory.size());
  std::vector<Pose2D> planning_proj_mpc;
  // if(trajectory.size() > 6 && traj_dist_to_ego.at(6)>2.0){
  //   for(size_t i = 0; i<mpc_trajectory.size();++i){
  //     for(size_t j = 0;j<=6;++j){
  //     }

  //   }
  // }
  if (!origin_mpc_trajectory.empty()) {
    PathPoint ego_pose;
    ego_pose.x = ego_state.ego_pose.x;
    ego_pose.y = ego_state.ego_pose.y;
    ego_pose.theta = ego_state.ego_pose.theta;
    origin_mpc_trajectory.insert(origin_mpc_trajectory.begin(), ego_pose);

    for (size_t i = 0; i < trajectory.size(); ++i) {
      std::vector<double> dist_traj_to_mpc;
      for (size_t j = 0; j < origin_mpc_trajectory.size(); ++j) {
        dist_traj_to_mpc.emplace_back(
            std::hypot(trajectory.at(i).x - origin_mpc_trajectory.at(j).x,
                       trajectory.at(i).y - origin_mpc_trajectory.at(j).y));
      }

      std::vector<double>::iterator closet_point =
          std::min_element(dist_traj_to_mpc.begin(), dist_traj_to_mpc.end());
      int index_closet_point =
          std::distance(dist_traj_to_mpc.begin(), closet_point);
      Pose2D point1 =
          Pose2D(origin_mpc_trajectory.at(index_closet_point).x,
                 origin_mpc_trajectory.at(index_closet_point).y,
                 origin_mpc_trajectory.at(index_closet_point).theta);
      Pose2D point2;

      if ((closet_point + 1) == dist_traj_to_mpc.end()) {
        break;
      }
      if (closet_point == dist_traj_to_mpc.begin() ||
          *(closet_point + 1) < *(closet_point - 1)) {
        point2 = Pose2D(origin_mpc_trajectory.at(index_closet_point + 1).x,
                        origin_mpc_trajectory.at(index_closet_point + 1).y,
                        origin_mpc_trajectory.at(index_closet_point + 1).theta);
      } else {
        point2 = Pose2D(origin_mpc_trajectory.at(index_closet_point - 1).x,
                        origin_mpc_trajectory.at(index_closet_point - 1).y,
                        origin_mpc_trajectory.at(index_closet_point - 1).theta);
      }

      projection_point = planning_math::calc_projection_point(point1, point2,
                                                              trajectory.at(i));
      deviated = deviated ||
                 (std::hypot(projection_point.x - trajectory.at(i).x,
                             projection_point.y - trajectory.at(i).y) > 0.3);

      planning_mpc_diff.emplace_back(
          std::hypot(projection_point.x - trajectory.at(i).x,
                     projection_point.y - trajectory.at(i).y));
    }
  }
  // std::cout<<"diffdebug proj size  = "<<planning_mpc_diff.size()<<std::endl;

  Pose2D curr_ego_pose = ego_state.ego_pose;
  std::vector<double> dist_egopose_to_traj;
  if (trajectory.empty()) {
    ego_lat_diff = 0.0;
  } else {
    for (size_t j = 0; j < trajectory.size(); ++j) {
      dist_egopose_to_traj.emplace_back(
          std::hypot(curr_ego_pose.x - trajectory.at(j).x,
                     curr_ego_pose.y - trajectory.at(j).y));
    }

    std::vector<double>::iterator closet_point = std::min_element(
        dist_egopose_to_traj.begin(), dist_egopose_to_traj.end());
    int index_closet_point =
        std::distance(dist_egopose_to_traj.begin(), closet_point);
    Pose2D point1 = Pose2D(trajectory.at(index_closet_point).x,
                           trajectory.at(index_closet_point).y,
                           trajectory.at(index_closet_point).theta);
    Pose2D point2;

    if ((closet_point + 1) == dist_egopose_to_traj.end()) {
      ego_lat_diff = 0.0;
    } else {
      if (closet_point == dist_egopose_to_traj.begin() ||
          *(closet_point + 1) < *(closet_point - 1)) {
        point2 = Pose2D(trajectory.at(index_closet_point + 1).x,
                        trajectory.at(index_closet_point + 1).y,
                        trajectory.at(index_closet_point + 1).theta);
      } else {
        point2 = Pose2D(trajectory.at(index_closet_point - 1).x,
                        trajectory.at(index_closet_point - 1).y,
                        trajectory.at(index_closet_point - 1).theta);
      }

      projection_point =
          planning_math::calc_projection_point2(point1, point2, curr_ego_pose);
      ego_lat_diff = std::hypot(projection_point.x - curr_ego_pose.x,
                                projection_point.y - curr_ego_pose.y);
    }
  }

  return true;
}

bool LeaderDecider::clip_traj() {
  double max_length = 20.0;
  auto &planning_result =
      PlanningContext::Instance()->planning_status().planning_result;
  // auto &mpc_trajectory_enu = PlanningContext::Instance()
  //                                ->mutable_planning_status()
  //                                ->planning_result.mpc_path;
  auto &ego_state = world_model_->get_ego_state();
  // const StatusType &status_type =
  // PlanningContext::Instance()->planning_status().scenario.status_type;

  double ego_s = 0.0;
  std::vector<Pose2D> &trajectory =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->trajectory;
  std::vector<Pose2D> &mpc_trajectory =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->mpc_trajectory;
  std::vector<Pose2D> &refactored_mpc_trajectory =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->refactored_mpc_trajectory;
  bool &deviated = PlanningContext::Instance()
                       ->mutable_longitudinal_behavior_planner_output()
                       ->deviated;
  double &ego_lat_diff = PlanningContext::Instance()
                             ->mutable_longitudinal_behavior_planner_output()
                             ->ego_lat_diff;
  std::vector<double> &planning_mpc_diff =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->planning_mpc_diff;
  trajectory.clear();
  mpc_trajectory.clear();
  // mpc_trajectory_enu.clear();
  planning_mpc_diff.clear();
  std::vector<PathPoint> origin_mpc_trajectory =
      world_model_->get_mpc_trajectory();
  // std::vector<PathPoint> mpc_trajectory_undiluted =
  //     world_model_->get_mpc_trajectory_undiluted();
  // std::vector<Pose2D> mpc_trajectory_undiluted;
  // int mpc_trajectory_undiluted_size = origin_mpc_trajectory_undiluted.size()
  // /2;
  ego_s = ego_state.ego_frenet.x;
  // std::cout << "ego_s : " << ego_s << std::endl;
  // std::cout << "ego_l : " << ego_state.ego_frenet.y << std::endl;
  // std::cout << "ego_theta : " << ego_state.ego_pose.theta << std::endl;
  // std::cout << "ego_x : " << ego_state.ego_pose.x << std::endl;
  // std::cout << "ego_y : " << ego_state.ego_pose.y << std::endl;
  double s = 0.0;
  double traj_length = 0.0;
  double dist = 100.0;
  double dist_min = 100.0;
  deviated = false;

  // for (size_t i = 0; i < origin_mpc_trajectory.size(); ++i) {
  //   PathPose mpc_point;
  //   mpc_point.pos.x = origin_mpc_trajectory.at(i).x;
  //   mpc_point.pos.y = origin_mpc_trajectory.at(i).y;
  //   mpc_point.pos.z = 0.0; //
  //   world_model_->get_ego_state().ego_enu.position.z;
  //   // auto q = tf::createQuaternionFromRPY(0, 0,
  //   // origin_mpc_trajectory.at(i).theta);
  //   using namespace Eigen;
  //   // Roll pitch and yaw in Radians
  //   Quaternionf q;
  //   q = AngleAxisf(0, Vector3f::UnitX()) * AngleAxisf(0, Vector3f::UnitY()) *
  //       AngleAxisf(origin_mpc_trajectory.at(i).theta, Vector3f::UnitZ());
  //   // std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
  //   mpc_point.orient.x = q.x();
  //   mpc_point.orient.y = q.y();
  //   mpc_point.orient.z = q.z();
  //   mpc_point.orient.w = q.w();
  //   mpc_trajectory_enu.emplace_back(mpc_point);
  // }

  // planning
  std::vector<Pose2D> traj_pose_array_ = planning_result.traj_pose_array;
  std::vector<Pose2D>::iterator iter = traj_pose_array_.begin();
  std::vector<Pose2D>::iterator iter_min = traj_pose_array_.begin();
  std::vector<float> traj_vel_array_ = planning_result.traj_vel_array;
  // std::vector<double> traj_dist_to_ego;
  // std::vector<double> mpc_dist_to_ego;
  Pose2D traj_point;
  int index = 0;
  int index_min = 0;
  for (iter = traj_pose_array_.begin(); iter != traj_pose_array_.end();
       iter++) {
    dist = std::hypot(iter->x - ego_state.ego_pose.x,
                      iter->y - ego_state.ego_pose.y);
    if (dist < dist_min) {
      dist_min = dist;
      iter_min = iter;
      index_min = index;
    }
    index++;
  }
  Pose2D projection_point;
  if (traj_pose_array_.size() == 0) {
    deviated = true;
  }
  if (iter_min != traj_pose_array_.begin()) {
    projection_point = planning_math::calc_projection_point(
        Pose2D((iter_min - 1)->x, (iter_min - 1)->y, (iter_min - 1)->theta),
        Pose2D((iter_min)->x, (iter_min)->y, (iter_min)->theta),
        Pose2D(ego_state.ego_pose.x, ego_state.ego_pose.y, 0.0));
    // std::cout << "TB deviation1: " << std::hypot(projection_point.x -
    // ego_state.ego_pose.x, projection_point.y - ego_state.ego_pose.y) <<
    // std::endl; std::cout << "TB deviation1: " << ego_state.ego_pose.theta <<
    // " " << projection_point.theta << std::endl;
    if (std::hypot(projection_point.x - ego_state.ego_pose.x,
                   projection_point.y - ego_state.ego_pose.y) > 0.3 ||
        std::abs(ego_state.ego_pose.theta - projection_point.theta) > 0.2) {
      deviated = true;
    }
    if (std::abs(traj_vel_array_.at(index_min - 1)) > 0.01) {
      s += std::hypot(projection_point.x - iter_min->x,
                      projection_point.y - iter_min->y);
      if (s > 0.01) {
        traj_point.x = projection_point.x;
        traj_point.y = projection_point.y;
        traj_point.theta = projection_point.theta;
        trajectory.push_back(traj_point);
        // std::cout << projection_point.x << ", "<< projection_point.y << ", "
        // << projection_point.theta << ";" <<  std::endl;
      }
    }
  }
  if ((iter_min) != traj_pose_array_.end() &&
      (iter_min + 1) != traj_pose_array_.end()) {
    projection_point = planning_math::calc_projection_point(
        Pose2D((iter_min)->x, (iter_min)->y, (iter_min)->theta),
        Pose2D((iter_min + 1)->x, (iter_min + 1)->y, (iter_min + 1)->theta),
        Pose2D(ego_state.ego_pose.x, ego_state.ego_pose.y, 0.0));
    // std::cout << "TB deviation2: " << std::hypot(projection_point.x -
    // ego_state.ego_pose.x, projection_point.y - ego_state.ego_pose.y) <<
    // std::endl; std::cout << "TB deviation2: " << ego_state.ego_pose.theta <<
    // " " << projection_point.theta << std::endl;
    if (std::hypot(projection_point.x - ego_state.ego_pose.x,
                   projection_point.y - ego_state.ego_pose.y) > 0.3 ||
        std::abs(ego_state.ego_pose.theta - projection_point.theta) > 0.2) {
      deviated = true;
    }
    if (std::abs(traj_vel_array_.at(index_min)) > 0.01 && trajectory.empty()) {
      s += std::hypot(projection_point.x - (iter_min + 1)->x,
                      projection_point.y - (iter_min + 1)->y);
      index_min++;
      iter_min++;
      if (s > 0.01) {
        traj_point.x = projection_point.x;
        traj_point.y = projection_point.y;
        traj_point.theta = projection_point.theta;
        trajectory.push_back(traj_point);
        // std::cout <<"trajdebug: " <<projection_point.x << ", "<<
        // projection_point.y << ", " << projection_point.theta << ";" <<
        // std::endl;
      }
    }
  }
  index = index_min;
  bool reach_end = false;
  for (iter = iter_min; iter != traj_pose_array_.end(); iter++) {
    s += std::hypot(iter->x - iter_min->x, iter->y - iter_min->y);
    // std::cout << iter->x << ", "<< iter->y << ", " << iter->theta << ";" <<
    // std::endl;
    traj_point.x = iter->x;
    traj_point.y = iter->y;
    traj_point.theta = iter->theta;
    trajectory.push_back(traj_point);
    // std::cout <<"trajdebug: " <<traj_point.x << ", "<< traj_point.y << ", "
    // << traj_point.theta << ";" <<  std::endl;
    // traj_dist_to_ego.emplace_back(std::hypot(iter->x - ego_state.ego_pose.x,
    // iter->y - ego_state.ego_pose.y)); std::cout <<"trajdebug dist =
    // "<<std::hypot(iter->x - ego_state.ego_pose.x, iter->y -
    // ego_state.ego_pose.y)<<std::endl;
    iter_min = iter;
    if (std::abs(traj_vel_array_.at(index)) < 0.01 && !reach_end) {
      traj_length = s;
      reach_end = true;
    }
    if (s > max_length) {
      break;
    }
    index++;
  }
  if (!reach_end) {
    traj_length = s;
  }

  // set trajectory's s
  calculatePathS(&trajectory);
  calculatePathKappa(&trajectory);

  // get plan path
  std::vector<Pose2D> &plan_path_output =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->plan_path;
  const std::vector<Pose2D> &trajectory_const =
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->trajectory;
  plan_path_output.clear();
  if (trajectory_const.size() > kExtendPathPointNumber) {
    for (auto iter = trajectory_const.begin();
         iter != trajectory_const.end() - kExtendPathPointNumber; ++iter) {
      plan_path_output.emplace_back(
          Pose2D(iter->x, iter->y, iter->theta, iter->s, iter->kappa()));
    }
  } else if (trajectory_const.size() > 0) {
    for (auto iter = trajectory_const.begin(); iter != trajectory_const.end();
         ++iter) {
      if (iter->s > kExceptPathPointS)
        break;
      plan_path_output.emplace_back(
          Pose2D(iter->x, iter->y, iter->theta, iter->s, iter->kappa()));
    }
  }

  PlanningContext::Instance()
      ->mutable_longitudinal_behavior_planner_output()
      ->traj_length = traj_length;
  // std::cout << "TBDEBUG: " << index << " "
  //           << index_min << " "
  //           << traj_length << " "
  //           << s << " "
  //           << deviated << " "
  //           << std::endl;
  // std::cout << "mpc:" <<  std::endl;
  // mpc_trajectory_undiluted_size = origin_mpc_trajectory_undiluted.size() /2;
  if (PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .planner_type == PlannerType::PARKING_LATERAL) {
    // PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()->traj_length
    // = s; mpc
    s = 0;
    for (size_t i = 0; i < origin_mpc_trajectory.size(); ++i) {
      // std::cout<<"mpcdebug origin size i = "<<origin_mpc_trajectory.size()<<"
      // "<<i<<std::endl;
      Pose2D mpc_point;
      mpc_point.x = origin_mpc_trajectory.at(i).x;
      mpc_point.y = origin_mpc_trajectory.at(i).y;
      mpc_point.theta = origin_mpc_trajectory.at(i).theta;
      // std::cout << "mpcdebug "<<i<<" : "<<mpc_point.x << ", "<< mpc_point.y
      // << ", " << mpc_point.theta << ";" <<  std::endl;
      if (mpc_trajectory.size() > 0) {
        s += std::hypot(mpc_trajectory.back().x - mpc_point.x,
                        mpc_trajectory.back().y - mpc_point.y);
      }

      if (/*i > origin_mpc_trajectory.size() / 2 ||*/ s > 1.5) {
        if (!deviated) {
          break;
        }
        // else if (s > 1.5 && i > origin_mpc_trajectory.size() / 2){
        //   break;
        // }
      }
      mpc_trajectory.push_back(mpc_point);
    }
    // for(size_t i = 0; i<mpc_trajectory_undiluted_size; ++i){

    //   //
    //   mpc_dist_to_ego.emplace_back(std::hypot(origin_mpc_trajectory_undiluted.at(i).x
    //   - ego_state.ego_pose.x, origin_mpc_trajectory_undiluted.at(i).y -
    //   ego_state.ego_pose.y)); Pose2D mpc_point; mpc_point.x =
    //   origin_mpc_trajectory_undiluted.at(i).x; mpc_point.y =
    //   origin_mpc_trajectory_undiluted.at(i).y; mpc_point.theta =
    //   origin_mpc_trajectory_undiluted.at(i).theta;

    //   mpc_trajectory_undiluted.emplace_back(mpc_point);
    // }

  } else if (PlanningContext::Instance()
                 ->parking_behavior_planner_output()
                 .planner_type == PlannerType::OPENSPACE) {
    // mpc
    s = 0;
    for (size_t i = 0; i < origin_mpc_trajectory.size(); ++i) {
      Pose2D mpc_point;
      mpc_point.x = origin_mpc_trajectory.at(i).x;
      mpc_point.y = origin_mpc_trajectory.at(i).y;
      mpc_point.theta = origin_mpc_trajectory.at(i).theta;
      // std::cout << mpc_point.x << ", "<< mpc_point.y << ", " <<
      // mpc_point.theta << ";" <<  std::endl;
      if (mpc_trajectory.size() > 0) {
        s += std::hypot(mpc_trajectory.back().x - mpc_point.x,
                        mpc_trajectory.back().y - mpc_point.y);
      }

      // if (i > origin_mpc_trajectory.size() / 2){
      //   break;
      // }
      mpc_trajectory.push_back(mpc_point);
    }
    // for(size_t i = 0; i<mpc_trajectory_undiluted_size; ++i){

    //   mpc_dist_to_ego.emplace_back(std::hypot(origin_mpc_trajectory_undiluted.at(i).x
    //   - ego_state.ego_pose.x, origin_mpc_trajectory_undiluted.at(i).y -
    //   ego_state.ego_pose.y)); Pose2D mpc_point; mpc_point.x =
    //   origin_mpc_trajectory_undiluted.at(i).x; mpc_point.y =
    //   origin_mpc_trajectory_undiluted.at(i).y; mpc_point.theta =
    //   origin_mpc_trajectory_undiluted.at(i).theta;

    //   mpc_trajectory_undiluted.emplace_back(mpc_point);
    // }
  }

  calculatePathS(&mpc_trajectory);
  calculatePathKappa(&mpc_trajectory);
  if (!mpc_trajectory.empty()) {
    std::vector<Pose2D> mpc_refactored =
        LeaderDecider::InterpolatePathPoints(mpc_trajectory, kMpcDeltaLength);
    refactored_mpc_trajectory = mpc_refactored;
  }

  // planning_mpc_diff.resize(mpc_trajectory.size());
  std::vector<Pose2D> planning_proj_mpc;
  // if(trajectory.size() > 6 && traj_dist_to_ego.at(6)>2.0){
  //   for(size_t i = 0; i<mpc_trajectory.size();++i){
  //     for(size_t j = 0;j<=6;++j){
  //     }

  //   }
  // }
  if (!origin_mpc_trajectory.empty()) {
    PathPoint ego_pose;
    ego_pose.x = ego_state.ego_pose.x;
    ego_pose.y = ego_state.ego_pose.y;
    ego_pose.theta = ego_state.ego_pose.theta;
    origin_mpc_trajectory.insert(origin_mpc_trajectory.begin(), ego_pose);

    for (size_t i = 0; i < trajectory.size(); ++i) {
      std::vector<double> dist_traj_to_mpc;
      for (size_t j = 0; j < origin_mpc_trajectory.size(); ++j) {
        dist_traj_to_mpc.emplace_back(
            std::hypot(trajectory.at(i).x - origin_mpc_trajectory.at(j).x,
                       trajectory.at(i).y - origin_mpc_trajectory.at(j).y));
      }

      std::vector<double>::iterator closet_point =
          std::min_element(dist_traj_to_mpc.begin(), dist_traj_to_mpc.end());
      int index_closet_point =
          std::distance(dist_traj_to_mpc.begin(), closet_point);
      Pose2D point1 =
          Pose2D(origin_mpc_trajectory.at(index_closet_point).x,
                 origin_mpc_trajectory.at(index_closet_point).y,
                 origin_mpc_trajectory.at(index_closet_point).theta);
      Pose2D point2;

      if ((closet_point + 1) == dist_traj_to_mpc.end()) {
        break;
      }
      if (closet_point == dist_traj_to_mpc.begin() ||
          *(closet_point + 1) < *(closet_point - 1)) {
        point2 = Pose2D(origin_mpc_trajectory.at(index_closet_point + 1).x,
                        origin_mpc_trajectory.at(index_closet_point + 1).y,
                        origin_mpc_trajectory.at(index_closet_point + 1).theta);
      } else {
        point2 = Pose2D(origin_mpc_trajectory.at(index_closet_point - 1).x,
                        origin_mpc_trajectory.at(index_closet_point - 1).y,
                        origin_mpc_trajectory.at(index_closet_point - 1).theta);
      }

      projection_point = planning_math::calc_projection_point(point1, point2,
                                                              trajectory.at(i));
      deviated = deviated ||
                 (std::hypot(projection_point.x - trajectory.at(i).x,
                             projection_point.y - trajectory.at(i).y) > 0.3);

      planning_mpc_diff.emplace_back(
          std::hypot(projection_point.x - trajectory.at(i).x,
                     projection_point.y - trajectory.at(i).y));
    }
  }
  // std::cout<<"diffdebug proj size  = "<<planning_mpc_diff.size()<<std::endl;

  Pose2D curr_ego_pose = ego_state.ego_pose;
  std::vector<double> dist_egopose_to_traj;
  if (trajectory.empty()) {
    ego_lat_diff = 0.0;
  } else {
    for (size_t j = 0; j < trajectory.size(); ++j) {
      dist_egopose_to_traj.emplace_back(
          std::hypot(curr_ego_pose.x - trajectory.at(j).x,
                     curr_ego_pose.y - trajectory.at(j).y));
    }

    std::vector<double>::iterator closet_point = std::min_element(
        dist_egopose_to_traj.begin(), dist_egopose_to_traj.end());
    int index_closet_point =
        std::distance(dist_egopose_to_traj.begin(), closet_point);
    Pose2D point1 = Pose2D(trajectory.at(index_closet_point).x,
                           trajectory.at(index_closet_point).y,
                           trajectory.at(index_closet_point).theta);
    Pose2D point2;

    if ((closet_point + 1) == dist_egopose_to_traj.end()) {
      ego_lat_diff = 0.0;
    } else {
      if (closet_point == dist_egopose_to_traj.begin() ||
          *(closet_point + 1) < *(closet_point - 1)) {
        point2 = Pose2D(trajectory.at(index_closet_point + 1).x,
                        trajectory.at(index_closet_point + 1).y,
                        trajectory.at(index_closet_point + 1).theta);
      } else {
        point2 = Pose2D(trajectory.at(index_closet_point - 1).x,
                        trajectory.at(index_closet_point - 1).y,
                        trajectory.at(index_closet_point - 1).theta);
      }

      projection_point =
          planning_math::calc_projection_point(point1, point2, curr_ego_pose);
      ego_lat_diff = std::hypot(projection_point.x - curr_ego_pose.x,
                                projection_point.y - curr_ego_pose.y);
    }
  }

  return true;
}

// bool LeaderDecider::check_bounding_box_collision(const planning_math::Box2d
// &obstacle_box){
//   planning_math::Box2d ego_box;
//   std::vector<Pose2D> trajectory =
//     PlanningContext::Instance()->longitudinal_behavior_planner_output().trajectory;
//   double center_x;
//   double center_y;
//   const bool reverse =
//   PlanningContext::Instance()->planning_status().planning_result.gear ==
//   GearState::REVERSE; const StatusType &status_type =
//   PlanningContext::Instance()->planning_status().scenario.status_type; double
//   back_comp_scaler = reverse ? -1.0 : 1.0; double back_comp_length = -
//   VehicleParam::Instance()->front_edge_to_center +
//   VehicleParam::Instance()->length; double collision_threshold = (status_type
//   == StatusType::APA||status_type ==
//   StatusType::APOA)?(CarParams::GetInstance()->vehicle_width -
//   VehicleParam::Instance()->width) / 2 * 0.5:0.1;

//   for (const auto &point : trajectory){
//     center_x = point.x + cos(point.theta) *
//     (VehicleParam::Instance()->front_edge_to_center -
//     VehicleParam::Instance()->length / 2.0 + back_comp_length *
//     back_comp_scaler / 2.0); center_y = point.y + sin(point.theta) *
//     (VehicleParam::Instance()->front_edge_to_center -
//     VehicleParam::Instance()->length / 2.0 + back_comp_length *
//     back_comp_scaler / 2.0); ego_box =
//     planning_math::Box2d(planning_math::Vec2d(center_x, center_y),
//     point.theta, VehicleParam::Instance()->length - back_comp_length,
//     VehicleParam::Instance()->width); if (ego_box.DistanceTo(obstacle_box) <
//     collision_threshold){
//       return true;
//     }
//   }
//   return false;
// }

// std::pair<bool, double> LeaderDecider::calc_bounding_box_collision_dist(const
// planning_math::Box2d &obstacle_box){
//   planning_math::Box2d ego_box;
//   std::vector<Pose2D> trajectory =
//     PlanningContext::Instance()->longitudinal_behavior_planner_output().trajectory;
//   std::vector<PathPoint> mpc_trajectory = world_model_->get_mpc_trajectory();
//   double center_x;
//   double center_y;
//   std::pair<bool, double> result;
//   double s = 0;
//   const bool reverse =
//   PlanningContext::Instance()->planning_status().planning_result.gear ==
//   GearState::REVERSE; const StatusType &status_type =
//   PlanningContext::Instance()->planning_status().scenario.status_type; double
//   back_comp_scaler = reverse ? -1.0 : 1.0; double back_comp_length = -
//   VehicleParam::Instance()->front_edge_to_center +
//   VehicleParam::Instance()->length; double comp_length =
//   VehicleParam::Instance()->front_edge_to_center -
//   VehicleParam::Instance()->length / 2.0 + back_comp_length *
//   back_comp_scaler / 2.0; double ego_x =
//   world_model_->get_ego_state().ego_pose.x; double ego_y =
//   world_model_->get_ego_state().ego_pose.y; double ego_theta =
//   world_model_->get_ego_state().ego_pose.theta;
//   // calc current dist to obstacle and determin the collision_threshold
//   center_x = ego_x
//              + cos(ego_theta)
//              * (comp_length);
//   center_y = ego_y
//              + sin(world_model_->get_ego_state().ego_pose.theta)
//              * (comp_length);

//   ego_box = planning_math::Box2d(planning_math::Vec2d(center_x, center_y),
//   ego_theta, VehicleParam::Instance()->length - back_comp_length,
//   VehicleParam::Instance()->width); double collision_threshold = (status_type
//   == StatusType::APA||status_type ==
//   StatusType::APOA)?(CarParams::GetInstance()->vehicle_width -
//   VehicleParam::Instance()->width) / 2 * 0.5:0.1; collision_threshold =
//   std::max(std::min(ego_box.DistanceTo(obstacle_box) - 0.01,
//   collision_threshold), 0.05);

//   // mpc
//   std::vector<PathPoint>::iterator iter_mpc = mpc_trajectory.begin();
//   for (const auto &point: mpc_trajectory) {
//     s += std::hypot(iter_mpc->x - point.x, iter_mpc->y - point.y);
//     if
//     (PlanningContext::Instance()->parking_behavior_planner_output().planner_type
//     == PlannerType::OPENSPACE){
//       center_x = ego_x + cos(ego_theta) * (point.x +
//       VehicleParam::Instance()->front_edge_to_center) - sin(ego_theta) *
//       (point.y) + cos(ego_theta + point.theta) * (comp_length); center_y =
//       ego_y + sin(ego_theta) * (point.x +
//       VehicleParam::Instance()->front_edge_to_center) + cos(ego_theta) *
//       (point.y) + sin(ego_theta + point.theta) * (comp_length);
//     }
//     else{
//       center_x = ego_x + cos(ego_theta) * (point.x +
//       VehicleParam::Instance()->front_edge_to_center) - sin(ego_theta) *
//       (point.y) + cos(ego_theta + point.theta) * (comp_length); center_y =
//       ego_y + sin(ego_theta) * (point.x +
//       VehicleParam::Instance()->front_edge_to_center) + cos(ego_theta) *
//       (point.y) + sin(ego_theta + point.theta) * (comp_length);
//     }

//     std::cout << ego_x + cos(ego_theta) * (point.x +
//     VehicleParam::Instance()->front_edge_to_center) - sin(ego_theta) *
//     (point.y) << ", "
//               << ego_y + sin(ego_theta) * (point.x +
//               VehicleParam::Instance()->front_edge_to_center) +
//               cos(ego_theta) * (point.y) << ", "
//               << ego_theta + point.theta << ";" << std::endl;

//     ego_box = planning_math::Box2d(planning_math::Vec2d(center_x, center_y),
//     ego_theta + point.theta, VehicleParam::Instance()->length -
//     back_comp_length, VehicleParam::Instance()->width); if
//     (ego_box.DistanceTo(obstacle_box) < collision_threshold){
//       std::cout << "mpc: " << ego_box.DistanceTo(obstacle_box) << ", " << s
//       << std::endl; result.first = true; result.second = s; return result;
//     }
//     if (point.x != mpc_trajectory.front().x || point.y !=
//     mpc_trajectory.front().y){
//       iter_mpc++;
//     }

//   }
//   // planning
//   s = 0;
//   std::vector<Pose2D>::iterator iter = trajectory.begin();
//   for (const auto &point : trajectory){
//     s += std::hypot(iter->x - point.x, iter->y - point.y);
//     center_x = point.x + cos(point.theta) * (comp_length);
//     center_y = point.y + sin(point.theta) * (comp_length);

//     ego_box = planning_math::Box2d(planning_math::Vec2d(center_x, center_y),
//     point.theta, VehicleParam::Instance()->length - back_comp_length,
//     VehicleParam::Instance()->width); if (ego_box.DistanceTo(obstacle_box) <
//     collision_threshold){
//       std::cout << "planning: " << ego_box.DistanceTo(obstacle_box) << ", "
//       << s << std::endl; result.first = true; result.second = s; return
//       result;
//     }
//     if (point.x != trajectory.front().x || point.y != trajectory.front().y){
//       iter++;
//     }
//   }
//   result.first = false;
//   result.second = s;
//   return result;
// }

CollisionCheckStatus LeaderDecider::calc_bounding_box_collision_dist(
    const planning_math::Box2d &obstacle_box, const Obstacle *obstacle) {
  planning_math::Box2d ego_box;
  std::vector<Pose2D> trajectory = PlanningContext::Instance()
                                       ->longitudinal_behavior_planner_output()
                                       .trajectory;
  std::vector<Pose2D> mpc_trajectory =
      PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .mpc_trajectory;
  auto scene_avp =
      PlanningContext::Instance()->planning_status().planning_result.scene_avp;

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
          ? (CarParams::GetInstance()->vehicle_width -
             VehicleParam::Instance()->width) /
                2 * 0.5
          : 0.1;
  if (status_type == StatusType::APA &&
      VehicleParam::Instance()->car_type == "C03" &&
      (!obstacle->IsStatic() && obstacle->Type() == ObjectType::COUPE)) {
    collision_threshold = 0.9;
  }
  if (status_type == StatusType::APA &&
      (obstacle->Type() == ObjectType::PEDESTRIAN)) {
    collision_threshold = 0.1;
  }
  // std::cout << "the before collision_thresjold:" << collision_threshold <<
  // std::endl;
  collision_threshold = std::max(
      std::min(ego_box.DistanceTo(obstacle_box) - 0.01, collision_threshold),
      0.05);
  // mpc
  // if(collision_checker_.get_ego_model()==nullptr){
  //   std::cout<<"no ego model ptr "<<std::endl;
  // }
  (void)collision_checker_
      .set_deviation_length(VehicleParam::Instance()->front_edge_to_center -
                            VehicleParam::Instance()->length / 2.0)
      .set_cut_length(back_comp_length)
      .set_reverse(reverse);

  result = collision_checker_.collision_check(mpc_trajectory, obstacle_box,
                                              collision_threshold);

  // std::cout << "modeldebug mpc: " << result.min_distance << ", " << result.s
  // << ", " << result.is_collision << std::endl;
  if (result.is_collision) {
    return result;
  }
  double min_dis = result.min_distance;

  // planning
  result = collision_checker_.collision_check(trajectory, obstacle_box,
                                              collision_threshold);

  // std::cout << "modeldebug planning: " << result.min_distance << ", " <<
  // result.s << ", " << result.is_collision << std::endl;
  if (result.min_distance > min_dis) {
    result.min_distance = min_dis;
  }
  return result;
}

CollisionCheckStatus LeaderDecider::calc_polygon_collision_dist(
    const planning_math::Box2d &obstacle_box,
    const planning_math::Polygon2d &fillet_cutting_polygon) {
  planning_math::Box2d ego_box;
  std::vector<Pose2D> trajectory = PlanningContext::Instance()
                                       ->longitudinal_behavior_planner_output()
                                       .trajectory;
  std::vector<Pose2D> mpc_trajectory =
      PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .mpc_trajectory;
  auto scene_avp =
      PlanningContext::Instance()->planning_status().planning_result.scene_avp;

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
          ? (CarParams::GetInstance()->vehicle_width -
             VehicleParam::Instance()->width) /
                2 * 0.5
          : 0.1;
  collision_threshold = std::max(
      std::min(ego_box.DistanceTo(obstacle_box) - 0.01, collision_threshold),
      0.05);

  (void)collision_checker_
      .set_deviation_length(VehicleParam::Instance()->front_edge_to_center -
                            VehicleParam::Instance()->length / 2.0)
      .set_cut_length(back_comp_length)
      .set_reverse(reverse);
  result = collision_checker_.collision_check(
      mpc_trajectory, fillet_cutting_polygon, collision_threshold);
  if (result.is_collision) {
    return result;
  }
  double min_dis = result.min_distance;
  result = collision_checker_.collision_check(
      trajectory, fillet_cutting_polygon, collision_threshold);
  if (result.min_distance > min_dis) {
    result.min_distance = min_dis;
  }
  return result;
}

planning_math::Polygon2d LeaderDecider::generate_fillet_cutting_polygon(
    const planning_math::Box2d &obstacle_box, double fillet_cutting_length) {
  planning_math::Polygon2d fillet_cutting_polygon;
  std::vector<planning_math::Vec2d> polygon_points;

  double obs_center_x = obstacle_box.center_x();
  double obs_center_y = obstacle_box.center_y();
  double half_length = obstacle_box.half_length();
  double half_width = obstacle_box.half_width();
  double heading = obstacle_box.heading();
  polygon_points.emplace_back(
      obs_center_x - cos(heading) * half_length + sin(heading) * half_width,
      obs_center_y - sin(heading) * half_length - cos(heading) * half_width);
  polygon_points.emplace_back(
      obs_center_x + cos(heading) * (half_length - fillet_cutting_length) +
          sin(heading) * half_width,
      obs_center_y + sin(heading) * (half_length - fillet_cutting_length) -
          cos(heading) * half_width);
  polygon_points.emplace_back(
      obs_center_x + cos(heading) * half_length +
          sin(heading) * (half_width - fillet_cutting_length),
      obs_center_y + sin(heading) * (half_length)-cos(heading) *
                         (half_width - fillet_cutting_length));
  polygon_points.emplace_back(
      obs_center_x + cos(heading) * half_length -
          sin(heading) * (half_width - fillet_cutting_length),
      obs_center_y + sin(heading) * (half_length) +
          cos(heading) * (half_width - fillet_cutting_length));
  polygon_points.emplace_back(
      obs_center_x + cos(heading) * (half_length - fillet_cutting_length) -
          sin(heading) * half_width,
      obs_center_y + sin(heading) * (half_length - fillet_cutting_length) +
          cos(heading) * half_width);
  polygon_points.emplace_back(
      obs_center_x - cos(heading) * half_length - sin(heading) * half_width,
      obs_center_y - sin(heading) * half_length + cos(heading) * half_width);

  fillet_cutting_polygon = planning_math::Polygon2d(polygon_points);
  return fillet_cutting_polygon;
}

void LeaderDecider::construct_exclusve_box() {
  if (PlanningContext::Instance()->planning_status().scenario.status_type ==
          StatusType::APA &&
      PlanningContext::Instance()
              ->parking_behavior_planner_output()
              .parking_slot_info.type.value != ParkingSlotType::PARALLEL) {
    // get lot corners
    std::vector<Point3D> lot_corners = PlanningContext::Instance()
                                           ->parking_behavior_planner_output()
                                           .parking_slot_info.corners;
    if (lot_corners.empty()) {
      return;
    }
    Pose2D box_center_pose;
    box_center_pose.theta = atan2(lot_corners[0].y - lot_corners[1].y,
                                  lot_corners[0].x - lot_corners[1].x);

    double box_length = 10.0;
    double box_width = 10.0;
    double tolerance = 0.0;

    box_center_pose.x =
        (lot_corners[1].x + lot_corners[2].x) / 2 -
        cos(box_center_pose.theta) * (box_length / 2 - tolerance);
    box_center_pose.y =
        (lot_corners[1].y + lot_corners[2].y) / 2 -
        sin(box_center_pose.theta) * (box_length / 2 - tolerance);

    PlanningContext::Instance()
        ->mutable_longitudinal_behavior_planner_output()
        ->exclusive_box = planning_math::Box2d(
        planning_math::Vec2d{box_center_pose.x, box_center_pose.y},
        box_center_pose.theta, box_length, box_width);
  } else {
    PlanningContext::Instance()
        ->mutable_longitudinal_behavior_planner_output()
        ->exclusive_box =
        planning_math::Box2d(planning_math::Vec2d{0.0, 0.0}, 0.0, 0.01, 0.01);
  }

  return;
}

std::vector<int> LeaderDecider::get_static_obstacle_beside_poi() {
  parking_lot_ = PlanningContext::Instance()
                     ->mutable_parking_behavior_planner_output()
                     ->parking_lot;
  const planning_math::Box2d &lot_box = parking_lot_->getBox();
  planning_math::Box2d lot_box_on_left_(lot_box);
  planning_math::Box2d lot_box_on_right_(lot_box);
  const std::vector<planning_math::Vec2d> &lot_corners =
      lot_box.GetAllCorners();
  lot_box_on_right_.Shift(lot_corners.at(0) - lot_corners.at(1));
  lot_box_on_left_.Shift(lot_corners.at(1) - lot_corners.at(0));
  planning_math::Polygon2d lot_polygon_on_left_(lot_box_on_left_);
  planning_math::Polygon2d lot_polygon_on_right_(lot_box_on_right_);
  std::vector<int> static_obstacle_biside_poi_id;
  for (auto &obj :
       world_model_->obstacle_manager().get_static_obstacles().Items()) {
    // planning_math::Polygon2d
    // static_obstacle_polygon(obj->PerceptionPolygon())
    if (lot_polygon_on_left_.ComputeIoU(obj->PerceptionPolygon()) > 0.5 ||
        lot_polygon_on_right_.ComputeIoU(obj->PerceptionPolygon()) > 0.5) {
      static_obstacle_biside_poi_id.emplace_back(obj->Id());
      // std::cout<<"static beside poi id = "<<obj->Id()<<std::endl;
    }
  }
  return static_obstacle_biside_poi_id;
}

void LeaderDecider::calculatePathS(std::vector<Pose2D> *const points) {
  if (points->size() == 0)
    return;
  double distance = 0.0;
  points->at(0).s = distance;
  for (size_t i = 1; i < points->size(); ++i) {
    double curr_x = points->at(i).const_x();
    double curr_y = points->at(i).const_y();
    double prev_x = points->at(i - 1).const_x();
    double prev_y = points->at(i - 1).const_y();
    distance += std::hypotf(prev_x - curr_x, prev_y - curr_y);
    points->at(i).s = distance;
  }
}

Pose2D LeaderDecider::GetInterpolateByLinearApproximation(
    const Pose2D &base, const Pose2D &p, const double s_tmp) const {
  double s0 = base.const_s();
  double s1 = p.const_s();

  double r = (s_tmp - s0) / (s1 - s0);
  double x_tmp = planning_math::lerp(base.const_x(), p.const_x(), r);
  double y_tmp = planning_math::lerp(base.const_y(), p.const_y(), r);
  double theta_tmp = planning_math::slerp(base.const_theta(), base.const_s(),
                                          p.const_theta(), p.const_s(), s_tmp);
  double kappa_tmp = planning_math::lerp(base.kappa(), p.kappa(), r);
  Pose2D res_pt(x_tmp, y_tmp, theta_tmp, s_tmp, kappa_tmp);
  return res_pt;
}

std::vector<Pose2D>
LeaderDecider::InterpolatePathPoints(const std::vector<Pose2D> &raw_points,
                                     const double step) {
  if (raw_points.size() < 2) {
    return raw_points;
  }
  std::vector<Pose2D> raw_path_points = raw_points;
  std::vector<Pose2D> interpolated_path_points;
  interpolated_path_points.emplace_back(raw_path_points[0]);
  int32_t path_size = raw_path_points.size();
  double interpolated_s = step;
  double cur_s = interpolated_s;
  int i = 1;

  while (i < path_size) {
    if (cur_s > raw_path_points[i].const_s()) {
      i++;
      continue;
    }
    interpolated_path_points.emplace_back(GetInterpolateByLinearApproximation(
        raw_path_points[i - 1], raw_path_points[i], cur_s));
    cur_s += interpolated_s;
  }
  interpolated_path_points.emplace_back(
      raw_path_points[raw_path_points.size() - 1]);
  return interpolated_path_points;
}

void LeaderDecider::calculatePathKappa(std::vector<Pose2D> *const points) {
  if (points->size() < 2)
    return;
  // calc kappa
  for (int32_t i = 0; i < points->size(); ++i) {
    double kappa = 0.0;
    double delta_s = 0.0;
    if (i == 0) {
      delta_s = points->at(i + 1).const_s() - points->at(i).const_s();
      kappa = planning_math::NormalizeAngle(points->at(i + 1).const_theta() -
                                            points->at(i).const_theta()) /
              delta_s;
    } else if (i == points->size() - 1) {
      delta_s = points->at(i).const_s() - points->at(i - 1).const_s();
      kappa = planning_math::NormalizeAngle(points->at(i).const_theta() -
                                            points->at(i - 1).const_theta()) /
              delta_s;
    } else {
      delta_s = points->at(i + 1).const_s() - points->at(i - 1).const_s();
      kappa = planning_math::NormalizeAngle(points->at(i + 1).const_theta() -
                                            points->at(i - 1).const_theta()) /
              delta_s;
    }
    points->at(i).set_kappa(kappa);
  }
}

} // namespace parking

} // namespace msquare
