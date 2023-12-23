#include "common/parking_map_info_manager.h"
#include "common/math/linear_interpolation.h"
#include "common/parking_world_model.h"
#include "planning/common/common.h"
namespace msquare {

namespace parking {
// void find_lane_rids(int lane_status, int lane_id,
//     const std::vector<int> &lane_line_bitlist,
//     const std::vector<int> &rids, std::array<int, 2> &lane_rids) {

//   int left_rid = kInvalidRelativeId;
//   int right_rid = kInvalidRelativeId;
//   if (lane_status == LaneStatusEx::BOTH_AVAILABLE ||
//       lane_status == LaneStatusEx::LEFT_AVAILABLE) {
//     if ((size_t)(lane_id + 1) <= lane_line_bitlist.size()) {
//       int left_rid_idx =
//           std::accumulate(lane_line_bitlist.begin(),
//                           lane_line_bitlist.begin() + lane_id + 1, 0);
//       if (left_rid_idx - 1 >= 0 && (size_t)(left_rid_idx - 1) < rids.size())
//       {
//         left_rid = rids[left_rid_idx - 1];
//       }
//     }
//   }
//   if (lane_status == LaneStatusEx::BOTH_AVAILABLE ||
//       lane_status == LaneStatusEx::RIGHT_AVAILABLE) {
//     if ((size_t)(lane_id + 2) <= lane_line_bitlist.size()) {
//       int right_rid_idx =
//           std::accumulate(lane_line_bitlist.begin(),
//                           lane_line_bitlist.begin() + lane_id + 2, 0);
//       if (right_rid_idx - 1 >= 0 && (size_t)(right_rid_idx - 1) <
//       rids.size()) {
//         right_rid = rids[right_rid_idx - 1];
//       }
//     }
//   }
//   lane_rids[0] = left_rid;
//   lane_rids[1] = right_rid;
// }

// MapInfo::MapInfo():c_raw_refline(RefLinePosition::CURR_REFLINE),
// l_raw_refline(RefLinePosition::LEFT_REFLINE),
// r_raw_refline(RefLinePosition::RIGHT_REFLINE),
// clane(LanePosition::CURR_POS, &c_raw_refline),
// llane(LanePosition::LEFT_POS, &l_raw_refline),
// rlane(LanePosition::RIGHT_POS, &r_raw_refline),
// rrlane(LanePosition::RIGHT_RIGHT_POS),
// lllane(LanePosition::LEFT_LEFT_POS), flane(LaneProperty::FIX_LANE),
// olane(LaneProperty::ORIGIN_LANE), tlane(LaneProperty::TARGET_LANE){
MapInfo::MapInfo() {
  // std::array<RawRefLine*, 2> neighbours{&l_raw_refline, &r_raw_refline};
  // c_raw_refline.set_neighbours(neighbours);

  // neighbours[0] = nullptr;
  // neighbours[1] = &c_raw_refline;
  // l_raw_refline.set_neighbours(neighbours);

  // neighbours[0] = &c_raw_refline;
  // neighbours[1] = nullptr;
  // r_raw_refline.set_neighbours(neighbours);

  // clane.set_raw_refline(&c_raw_refline);
  // llane.set_raw_refline(&l_raw_refline);
  // rlane.set_raw_refline(&r_raw_refline);

  // std::array<Lane*, 2> lane_neighbours{&llane, &rlane};
  // clane.set_neighbours(lane_neighbours);

  // lane_neighbours[0] = &clane;
  // lane_neighbours[1] = &rlane;
  // rlane.set_neighbours(lane_neighbours);

  // lane_neighbours[0] = &rlane;
  // lane_neighbours[1] = nullptr;
  // rrlane.set_neighbours(lane_neighbours);

  // lane_neighbours[0] = &lllane;
  // lane_neighbours[1] = &clane;
  // llane.set_neighbours(lane_neighbours);

  // lane_neighbours[0] = nullptr;
  // lane_neighbours[1] = &llane;
  // lllane.set_neighbours(lane_neighbours);

  flane_update = false;

  road_type = RoadType::GO_STRAIGHT;
  scenario_type = ScenarioType::STATE_INIT;
  lane_change_type = PlanningType::LANE_KEEP;
  last_lane_change_type = PlanningType::LANE_DEPARTURE;
  prev_lane_status.status = LaneStatus::Status::LANE_KEEP;
  curr_lane_width.resize(400);
  target_lane_width.resize(400);
  for (size_t i = 0; i < 400; ++i) {
    curr_lane_width[i] = std::make_pair(i * 0.2, 3.4);
    target_lane_width[i] = std::make_pair(i * 0.2, 3.4);
  }
}

void MapInfoManager::set_map_info(const MapInfo &map_info) {
  map_info_.first_task_ranges = map_info.first_task_ranges;
  map_info_.is_in_intersection = map_info.is_in_intersection;
  map_info_.road_type = map_info.road_type;
  map_info_.distance_to_crossing = map_info.distance_to_crossing;

  map_info_.current_refline_points = map_info.current_refline_points;
  map_info_.left_refline_points = map_info.left_refline_points;
  map_info_.right_refline_points = map_info.right_refline_points;

  map_info_.left_boundary_info = map_info.left_boundary_info;
  map_info_.right_boundary_info = map_info.right_boundary_info;

  map_info_.current_lane_index = map_info.current_lane_index;
  map_info_.current_parking_lot_id = map_info.current_parking_lot_id;

  map_info_.lanes_info_marks = map_info.lanes_info_marks;
  map_info_.current_tasks = map_info.current_tasks;

  map_info_.traffic_light = map_info.traffic_light;

  map_info_.next_merge_type = map_info.next_merge_type;

  map_info_.distance_to_stop_line = map_info.distance_to_stop_line;

  map_info_.distance_to_aimed_poi = map_info.distance_to_aimed_poi;

  map_info_.square_mapping_result = map_info.square_mapping_result;
}

void MapInfoManager::set_map_info_from_apf() {
  // ApfStatus apf_status = PlanningContext::Instance()->apf_status();
  // OpenSpacePath last_apf_path = PlanningContext::Instance()->last_apf_path();
  ReferenceLine reference_line = PlanningContext::Instance()->reference_line();
  std::vector<RefLinePoint> current_refline_points;
  std::vector<RefLinePoint> ref_trajectory;
  if (!reference_line.available() ||
      reference_line.refline_points().size() < 2) {
    map_info_.current_refline_points.clear();
    map_info_.ref_trajectory.clear();
    return;
  } else {
    auto &refline_points = reference_line.refline_points();
    double dx, dy;
    dx = refline_points.at(0).path_point.x - refline_points.at(1).path_point.x;
    dy = refline_points.at(0).path_point.y - refline_points.at(1).path_point.y;
    for (int i = 10; i > 0; i--) {
      RefLinePoint refline_point;
      refline_point.x = refline_points.at(0).path_point.x + dx * i;
      refline_point.y = refline_points.at(0).path_point.y + dy * i;
      refline_point.left_road_border_distance = 4.0;
      refline_point.right_road_border_distance = 3.0;
      refline_point.lane_width = 6.0;
      current_refline_points.emplace_back(refline_point);
    }
    for (auto &p : refline_points) {
      RefLinePoint refline_point;
      refline_point.x = p.path_point.x;
      refline_point.y = p.path_point.y;
      refline_point.left_road_border_distance = 4.0;
      refline_point.right_road_border_distance = 3.0;
      refline_point.lane_width = 6.0;
      current_refline_points.emplace_back(refline_point);
      ref_trajectory.emplace_back(refline_point);
    }
    map_info_.current_refline_points = current_refline_points;
    map_info_.ref_trajectory = ref_trajectory;
  }
}
void MapInfoManager::set_map_info(
    const std::shared_ptr<WorldModel> &world_model) {
  Pose2D ego_pose =
      world_model->get_ego_state_manager().get_ego_state().ego_pose;
  if (!PlanningContext::Instance()
           ->mutable_planning_status()
           ->planning_result.is_loaded) {
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->planning_result.is_loaded = trajectory_loader_.LoadTrajectory();
  }
  std::vector<RefTrajectoryPoint> trajectory =
      trajectory_loader_.get_trajectory();
  std::vector<RefLinePoint> current_refline_points;
  std::vector<RefTrajectoryPoint>::iterator iter = trajectory.begin();
  std::vector<RefTrajectoryPoint>::iterator iter_min = trajectory.begin();
  PlanningContext::Instance()
      ->mutable_planning_status()
      ->planning_result.ref_trajectory.clear();
  double dist = 100;
  double dist_min = 100;
  for (iter = trajectory.begin(); iter != trajectory.end(); iter++) {
    dist = std::hypot(iter->x - ego_pose.x, iter->y - ego_pose.y);
    if (dist < dist_min) {
      dist_min = dist;
      iter_min = iter;
    }
  }

  double s = 0;

  for (iter = iter_min; iter != trajectory.begin(); iter--) {
    s += std::hypot(iter->x - (iter - 1)->x, iter->y - (iter - 1)->y);
    if (s > 20.0) {
      break;
    }
  }

  s = 0;
  iter_min = iter;
  for (iter; iter != trajectory.end(); iter++) {
    s += std::hypot(iter->x - iter_min->x, iter->y - iter_min->y);
    RefLinePoint refline_point;
    refline_point.x = iter->x;
    refline_point.y = iter->y;
    refline_point.left_road_border_distance = 4.0;
    refline_point.right_road_border_distance = 3.0;
    refline_point.lane_width = 6.0;
    current_refline_points.emplace_back(refline_point);
    // ref_traj
    PathPose ref_traj_point;
    ref_traj_point.pos.x = iter->x;
    ref_traj_point.pos.y = iter->y;
    ref_traj_point.pos.z = iter->z;
    ref_traj_point.orient.x = iter->quaternion.x;
    ref_traj_point.orient.y = iter->quaternion.y;
    ref_traj_point.orient.z = iter->quaternion.z;
    ref_traj_point.orient.w = iter->quaternion.w;
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->planning_result.ref_trajectory.emplace_back(ref_traj_point);
    iter_min = iter;
    if (s > 70) {
      break;
    }
  }

  // map_info_.first_task_ranges = map_info.first_task_ranges;
  // map_info_.is_in_intersection = map_info.is_in_intersection;
  // map_info_.road_type = map_info.road_type;
  // map_info_.distance_to_crossing = map_info.distance_to_crossing;

  map_info_.current_refline_points = current_refline_points;
  //   map_info_.left_refline_points = map_info.left_refline_points;
  //   map_info_.right_refline_points = map_info.right_refline_points;

  //   map_info_.left_boundary_info  = map_info.left_boundary_info;
  //   map_info_.right_boundary_info = map_info.right_boundary_info;

  //   map_info_.current_lane_index = map_info.current_lane_index;
  map_info_.current_parking_lot_id = PlanningContext::Instance()
                                         ->parking_behavior_planner_output()
                                         .current_parking_lot_id;

  //   map_info_.lanes_info_marks = map_info.lanes_info_marks;
  //   map_info_.current_tasks = map_info.current_tasks;

  //   map_info_.traffic_light = map_info.traffic_light;

  //   map_info_.next_merge_type = map_info.next_merge_type;

  //   map_info_.distance_to_stop_line =
  //   map_info.distance_to_stop_line.distance_to_stop_line;

  //   map_info_.distance_to_aimed_poi = map_info.distance_to_aimed_poi;

  //   map_info_.square_mapping_result = map_info.square_mapping_result;
}

bool MapInfoManager::update(const std::shared_ptr<WorldModel> &world_model) {
  // keep the last ref_trajectory if incoming ref_trajectory is empty
  // map_info_.ref_trajectory.clear();
  if (!get_ref_trajectory(world_model)) {
    return false;
  }

  map_info_.v_cruise = 7.0;
  map_info_.last_lane_change_type = PlanningType::LANE_DEPARTURE;
  map_info_.scenario_type = ScenarioType::STATE_LANEFOLLOW;
  map_info_.last_lane_change_type = map_info_.lane_change_type;
  // auto planning_status =
  // PlanningContext::Instance()->mutable_planning_status();

  return true;
}

void MapInfoManager::set_range_of_trajectory() {
  static const double DIST_FROM_BEHIND = 30; // Horizon lower bound
  static const double DIST_TO_FRONT = 100;   // Horizon upper bound
  // std::vector<geometry_msgs::Point> mid_ref_trajectory;
  if (map_info_.ref_trajectory.empty()) {
    MSD_LOG(INFO,
            "Set Range of the trajectory: Incoming trajectory is empty!!!");
    return;
  }

  int start = 0, end = (int)map_info_.ref_trajectory.size() - 1;
  while (start <= end) {
    double x = map_info_.ref_trajectory[start].x;
    double y = map_info_.ref_trajectory[start].y;
    double dist_to_origin = std::sqrt(std::pow(x, 2.0) + std::pow(y, 2.0));

    if (dist_to_origin <= DIST_FROM_BEHIND) {
      break;
    }
    start++;
  }

  while (start <= end) {
    double x = map_info_.ref_trajectory[end].x;
    double y = map_info_.ref_trajectory[end].y;

    double dist_to_origin = std::sqrt(std::pow(x, 2.0) + std::pow(y, 2.0));

    if (dist_to_origin <= DIST_TO_FRONT) {
      break;
    }
    end--;
  }
  if (start > 0) {
    for (int i = 0, j = start; j <= end; i++, j++) {
      map_info_.ref_trajectory[i] = map_info_.ref_trajectory[j];
    }
  }
  map_info_.ref_trajectory.resize(end - start + 1);
}

bool MapInfoManager::get_ref_trajectory(
    const std::shared_ptr<WorldModel> &world_model) {

  std::vector<RefLinePoint> ref_trajectory_in;

  get_ref_line(world_model, ref_trajectory_in);

  if (ref_trajectory_in.empty() || ref_trajectory_in.size() < 2) {
    MSD_LOG(INFO, "Incoming reference trajectory is empty!");
    return false;
  }
  map_info_.ref_trajectory = ref_trajectory_in;
  cut_loop(map_info_.ref_trajectory);
  linear_interpolation_parking();
  // if (world_model->is_parking_svp()) {
  //   set_range_of_trajectory();

  //   // lat_offset_ = optimal_info_.lat_offset;
  //   auto &car2enu = world_model->get_ego_state_manager().get_car2enu();
  //   frame_trans_from_car2enu(0.0, car2enu);
  // }

  return true;
}

void MapInfoManager::frame_trans_from_car2enu(float lat_offset,
                                              const Transform &car2enu) {
  if (map_info_.ref_trajectory.empty()) {
    MSD_LOG(INFO,
            "Trajectory frame transformation: Incoming trajectory is empty!!!");
    return;
  }

  Eigen::Vector3d car_point, enu_point;
  for (auto &p : map_info_.ref_trajectory) {
    car_point.x() = p.x;
    car_point.y() = p.y; // + lat_offset;
    car_point.z() = 0.0;

    enu_point = car2enu * car_point;
    p.x = enu_point.x();
    p.y = enu_point.y();
  }
}

void MapInfoManager::linear_interpolation_parking() {
  const double INTERVAL = 0.3;
  // int insert_point_count = 0;

  std::vector<double> xx_point, yy_point;
  for (std::size_t i = 1; i < map_info_.ref_trajectory.size(); i++) {
    double ss = sqrt(
        ((map_info_.ref_trajectory[i].x - map_info_.ref_trajectory[i - 1].x) *
             (map_info_.ref_trajectory[i].x -
              map_info_.ref_trajectory[i - 1].x) +
         (map_info_.ref_trajectory[i].y - map_info_.ref_trajectory[i - 1].y) *
             (map_info_.ref_trajectory[i].y -
              map_info_.ref_trajectory[i - 1].y)));
    // std::cout << i << " ss: " << ss t<< std::endl;
    if (ss > INTERVAL) {
      int ndiff = ceil(ss / INTERVAL);
      double x_step =
          (map_info_.ref_trajectory[i].x - map_info_.ref_trajectory[i - 1].x) /
          ndiff;
      double y_step =
          (map_info_.ref_trajectory[i].y - map_info_.ref_trajectory[i - 1].y) /
          ndiff;

      xx_point.emplace_back(map_info_.ref_trajectory[i - 1].x);
      yy_point.emplace_back(map_info_.ref_trajectory[i - 1].y);
      for (int j = 1; j < ndiff; j++) {
        xx_point.emplace_back(xx_point[xx_point.size() - 1] + x_step);
        yy_point.emplace_back(yy_point[yy_point.size() - 1] + y_step);
      }
    } else {
      xx_point.emplace_back(map_info_.ref_trajectory[i - 1].x);
      yy_point.emplace_back(map_info_.ref_trajectory[i - 1].y);
    }
  }

  // delete same point
  if (xx_point.size() <= 1 || yy_point.size() <= 1) {
    return;
  }
  for (int i = 1; i < (int)xx_point.size() - 1; i++) {
    if (abs(xx_point[i] - xx_point[i - 1]) < 1e-6 &&
        abs(yy_point[i] - yy_point[i - 1]) < 1e-6) {
      xx_point[i] = (xx_point[i - 1] + xx_point[i + 1]) / 2.0;
      yy_point[i] = (yy_point[i - 1] + yy_point[i + 1]) / 2.0;
    }
  }

  if (abs(xx_point[xx_point.size() - 1] - xx_point[xx_point.size() - 2]) <
          1e-6 &&
      abs(yy_point[yy_point.size() - 1] - yy_point[yy_point.size() - 2]) <
          1e-6) {
    xx_point.erase(xx_point.begin() + xx_point.size() - 1);
    yy_point.erase(yy_point.begin() + yy_point.size() - 1);
  }
  xx_point.emplace_back(
      map_info_.ref_trajectory[map_info_.ref_trajectory.size() - 1].x);
  yy_point.emplace_back(
      map_info_.ref_trajectory[map_info_.ref_trajectory.size() - 1].y);
  map_info_.ref_trajectory.clear();
  for (std::size_t i = 0; i < xx_point.size(); i++) {
    RefLinePoint point_to_insert;
    point_to_insert.x = xx_point[i];
    point_to_insert.y = yy_point[i];
    map_info_.ref_trajectory.emplace_back(point_to_insert);
  }
}

void MapInfoManager::get_ref_line(
    const std::shared_ptr<WorldModel> &world_model,
    std::vector<RefLinePoint> &ref_trajectory) {
  // if (world_model->get_optimal_info().which_lane == "left_line") {
  //   ref_trajectory = map_info_.left_refline_points;
  // } else if (world_model->get_optimal_info().which_lane == "right_line") {
  //   ref_trajectory = map_info_.right_refline_points;
  // } else {
  ref_trajectory = map_info_.current_refline_points;
  // }
}

void MapInfoManager::cut_loop(std::vector<RefLinePoint> &ref_trajectory) {
  if (ref_trajectory.empty())
    return;
  std::string id_first = ref_trajectory.front().track_id;
  size_t loop_index = 0;
  for (int i = (int)ref_trajectory.size() - 1; i > 0; i--) {
    if (ref_trajectory[i].track_id == id_first) {
      loop_index = i;
      break;
    }
    // std::cout << "ref track id: " << ref_trajectory[i].track_id << std::endl;
  }
  if (loop_index > 0) {
    ref_trajectory.erase(ref_trajectory.begin() + loop_index,
                         ref_trajectory.end());
  }
}

} // namespace parking

} // namespace msquare
