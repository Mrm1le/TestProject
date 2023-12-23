#include "data_driven_planner/models/vision_lane_manager.h"
#include "common/planning_fail_tracer.h"
#include "data_driven_planner/common/ddp_context.h"
#include "data_driven_planner/common/ddp_utils.h"
#include "data_driven_planner/models/ego_pose_manager.h"
#include <iostream>

namespace msquare {
namespace ddp {

Point transform_car_enu(const Point &p_origin, const EgoPose &ego_pose) {
  Point p_dest{};

  float sin_y = std::sin(-ego_pose.heading_angle);
  float cos_y = std::cos(-ego_pose.heading_angle);
  p_dest.x = cos_y * p_origin.x + sin_y * p_origin.y + ego_pose.position.x;
  p_dest.y = -sin_y * p_origin.x + cos_y * p_origin.y + ego_pose.position.y;
  p_dest.z = 0.f;

  return p_dest;
}

void VisionLaneManager::feed_vision_lane(
    double timestamp,
    const maf_perception_interface::RoadLinePerception &road_line_perception) {
  EgoPose ego_pose_navi;
  auto ok = DdpContext::Instance()->ego_pose_manager()->get_ego_pose(
      timestamp, ego_pose_navi);

  LaneBoundaryGroup lane_boundary_group{};
  lane_boundary_group.timestamp = timestamp;

  LaneBoundaryGroup lane_proposal_group{};
  lane_proposal_group.timestamp = timestamp;

  if (ok) {
    // transform position from ego to enu!!!!
    // push back lanes
    auto lanes_data = road_line_perception.lane_perception.lanes;
    // std::cout<<"lanes_data.size"<<lanes_data.size()<<std::endl;
    for (auto temp_lane : lanes_data) {
      if (temp_lane.camera_source.value ==
              maf_perception_interface::CameraSourceEnum::
                  CAMERA_SOURCE_FRONT_WIDE ||
          temp_lane.is_failed_3d) {
        continue;
      }
      LaneBoundary temp_laneboundary{};
      // lane_type need to consider!!!!
      temp_laneboundary.type = temp_lane.lane_type.value;
      auto &lane_points_3d_x = temp_lane.points_3d_x;
      auto &lane_points_3d_y = temp_lane.points_3d_y;
      auto lane_points_size =
          std::min(lane_points_3d_x.size(), lane_points_3d_y.size());
      // std::cout<<"lane_points_size:"<<lane_points_size<<std::endl;
      for (size_t i = 0; i < lane_points_size; ++i) {
        // convert from rear axle to center
        double x =
            lane_points_3d_x[i] -
            (ConfigurationContext::Instance()->get_vehicle_param().length /
                 2.0 -
             ConfigurationContext::Instance()
                 ->get_vehicle_param()
                 .rear_bumper_to_rear_axle);
        Point temp_point{x, lane_points_3d_y[i], 0.f};
        Point temp_point_new = transform_car_enu(temp_point, ego_pose_navi);
        temp_laneboundary.points.push_back(std::move(temp_point_new));
      }
      if (temp_lane.is_centerline) {
        temp_laneboundary.id = temp_lane.index;
        lane_proposal_group.lane_boundaries.push_back(
            std::move(temp_laneboundary));
      } else {
        lane_boundary_group.lane_boundaries.push_back(
            std::move(temp_laneboundary));
      }
    }
    // std::cout<<"aferlane_lane_boundary_group.size"<<lane_boundary_group.lane_boundaries.size()<<std::endl;
    // push_back roadedge!!!!
    auto roadedges_data = road_line_perception.road_edge_perception.road_edges;
    for (auto temp_roadedge : roadedges_data) {
      if (temp_roadedge.camera_source.value ==
              maf_perception_interface::CameraSourceEnum::
                  CAMERA_SOURCE_FRONT_WIDE ||
          temp_roadedge.is_failed_3d) {
        continue;
      }
      LaneBoundary temp_laneboundary{};
      // lane_type need to consider!!!!
      temp_laneboundary.type = 5;
      auto &lane_points_3d_x = temp_roadedge.points_3d_x;
      auto &lane_points_3d_y = temp_roadedge.points_3d_y;
      auto lane_points_size =
          std::min(lane_points_3d_x.size(), lane_points_3d_y.size());
      // std::cout<<"road_lane_points_size:"<<lane_points_size<<std::endl;
      for (size_t i = 0; i < lane_points_size; ++i) {
        // convert from rear axle to center
        double x =
            lane_points_3d_x[i] -
            (ConfigurationContext::Instance()->get_vehicle_param().length /
                 2.0 -
             ConfigurationContext::Instance()
                 ->get_vehicle_param()
                 .rear_bumper_to_rear_axle);
        Point temp_point{x, lane_points_3d_y[i], 0.f};
        Point temp_point_new = transform_car_enu(temp_point, ego_pose_navi);
        temp_laneboundary.points.push_back(std::move(temp_point_new));
      }
      lane_boundary_group.lane_boundaries.push_back(
          std::move(temp_laneboundary));
    }
    // std::cout<<"aferroad_lane_boundary_group_size:"<<lane_boundary_group.lane_boundaries.size()<<std::endl;
    lane_boundary_groups_.push_front(std::move(lane_boundary_group));
    if (lane_boundary_groups_.size() > 10) {
      lane_boundary_groups_.pop_back();
    }

    lane_proposal_groups_.push_front(std::move(lane_proposal_group));
    if (lane_proposal_groups_.size() > 10) {
      lane_proposal_groups_.pop_back();
    }
  }
}

LineType VisionLaneManager::get_vectornet_lane_boundary_type(
    const LaneBoundary &lane_boundary_seg) const {
  if (lane_boundary_seg.type == 1) {
    return LineType::DASHED_LANE_BOUNDARY;
  } else if (lane_boundary_seg.type == 2 || lane_boundary_seg.type == 3 ||
             lane_boundary_seg.type == 4) {
    return LineType::SOLID_LANE_BOUNDARY;
  } else if (lane_boundary_seg.type == 5) {
    return LineType::SOLID_ROAD_BOUNDARY;
  }
  return LineType::SOLID_LANE_BOUNDARY;
}

void VisionLaneManager::get_lane_boundary_group(
    std::vector<LaneBoundary> &lane_boundaries, double latest_fusion_timestamp,
    double &lanes_ts) {
  size_t near_index = 0;
  double min_delta_time = 10000.0;
  for (size_t i = 0; i < lane_boundary_groups_.size(); ++i) {
    if (std::fabs(latest_fusion_timestamp -
                  lane_boundary_groups_[i].timestamp) < min_delta_time) {
      min_delta_time = std::fabs(latest_fusion_timestamp -
                                 lane_boundary_groups_[i].timestamp);
      near_index = i;
    }
  }

  lane_boundaries = lane_boundary_groups_[near_index].lane_boundaries;
  lanes_ts = lane_boundary_groups_[near_index].timestamp;
}

bool VisionLaneManager::get_vision_lane(double timestamp, const Point &position,
                                        std::vector<NodeLane> &lanes,
                                        double &lanes_ts) {
  const double spatial_distance = 4.9;
  const int kMaxNodeNumBoundary = 10;

  lanes.clear();

  std::vector<LaneBoundary> current_lane_boundaries;
  get_lane_boundary_group(current_lane_boundaries, timestamp, lanes_ts);

  for (auto lane_boundary : current_lane_boundaries) {
    std::vector<Node> lane_boundary_nodes;
    lane_boundary_nodes.clear();
    float min_dis_to_ego = 10000000.f;
    bool b_start_point = true;
    Point pre_point{};
    pre_point.x = 0.0;
    pre_point.y = 0.0;
    pre_point.z = 0.0;
    // boundary_type is not use here!!!
    LineType boundary_type = LineType::OTHER_LINE;
    boundary_type = get_vectornet_lane_boundary_type(lane_boundary);
    for (size_t i = 0; i < lane_boundary.points.size(); ++i) {
      Point current_point = lane_boundary.points[i];

      //          bool b_in_valid_area = fabs(position.x - current_point.x) <
      //          search_range &&
      //                                 fabs(position.y - current_point.y) <
      //                                 search_range;
      bool b_in_valid_area = true;

      // need to consider!!!!!
      bool b_last_point = i == lane_boundary.points.size() - 1;

      bool b_valid_point_dis =
          b_start_point || b_last_point ||
          std::pow(pre_point.x - current_point.x, 2) +
                  std::pow(pre_point.y - current_point.y, 2) >
              std::pow(spatial_distance, 2);

      if (b_in_valid_area && b_valid_point_dis) {
        float pt_dis_to_ego =
            std::sqrt(std::pow(position.x - current_point.x, 2) +
                      std::pow(position.y - current_point.y, 2));
        min_dis_to_ego = std::min(pt_dis_to_ego, min_dis_to_ego);
        if (b_start_point) {
          pre_point = current_point;
          b_start_point = false;
        } else {
          lane_boundary_nodes.push_back(
              Node(pre_point, current_point, boundary_type));
          pre_point = current_point;
          if (lane_boundary_nodes.size() == kMaxNodeNumBoundary) {
            lanes.emplace_back(lane_boundary_nodes, std::to_string(-1),
                               boundary_type, -1, min_dis_to_ego);
            lane_boundary_nodes.clear();
            min_dis_to_ego = 10000000.f;
          }
        }
      }
    }
    // should consider how many need to save!!!
    if (lane_boundary_nodes.size() > 0) {
      lanes.emplace_back(lane_boundary_nodes, std::to_string(-1), boundary_type,
                         -1, min_dis_to_ego);
    }
  }
  // sort lanes
  sort(lanes.begin(), lanes.end(),
       [](const NodeLane &lane1, const NodeLane &lane2) {
         auto dis1 = lane1.min_distance_to_ego;
         auto dis2 = lane2.min_distance_to_ego;
         return dis1 < dis2;
       });

  return true;
}

void VisionLaneManager::get_lane_proposal_group(
    std::vector<LaneBoundary> &lane_boundaries, double latest_fusion_timestamp,
    double &lane_proposals_ts) {
  size_t near_index = 0;
  double min_delta_time = 10000.0;
  for (size_t i = 0; i < lane_proposal_groups_.size(); ++i) {
    if (std::fabs(latest_fusion_timestamp -
                  lane_proposal_groups_[i].timestamp) < min_delta_time) {
      min_delta_time = std::fabs(latest_fusion_timestamp -
                                 lane_proposal_groups_[i].timestamp);
      near_index = i;
    }
  }

  lane_boundaries = lane_proposal_groups_[near_index].lane_boundaries;
  lane_proposals_ts = lane_proposal_groups_[near_index].timestamp;
}

bool VisionLaneManager::get_lane_proposal(double timestamp,
                                          const Point &position,
                                          std::vector<NodeLane> &lanes,
                                          double &lane_proposals_ts) {
  const double spatial_distance = 9.9;

  lanes.clear();

  std::vector<LaneBoundary> current_lane_boundaries;
  get_lane_proposal_group(current_lane_boundaries, timestamp,
                          lane_proposals_ts);

  for (auto lane_boundary : current_lane_boundaries) {
    std::vector<Node> lane_boundary_nodes;
    lane_boundary_nodes.clear();
    float min_dis_to_ego = 10000000.f;
    bool b_start_point = true;
    Point pre_point{};
    pre_point.x = 0.0;
    pre_point.y = 0.0;
    pre_point.z = 0.0;
    // boundary_type is not use here!!!
    LineType boundary_type = LineType::LANE_PROPOSAL_ON_ROUTE;
    double on_route_dis = 1000.0;
    for (size_t i = 0; i < lane_boundary.points.size(); ++i) {
      Point current_point = lane_boundary.points[i];

      //          bool b_in_valid_area = fabs(position.x - current_point.x) <
      //          search_range &&
      //                                 fabs(position.y - current_point.y) <
      //                                 search_range;
      bool b_in_valid_area = true;

      // need to consider!!!!!
      bool b_last_point = i == lane_boundary.points.size() - 1;

      bool b_valid_point_dis =
          b_start_point || b_last_point ||
          std::pow(pre_point.x - current_point.x, 2) +
                  std::pow(pre_point.y - current_point.y, 2) >
              std::pow(spatial_distance, 2);

      if (b_in_valid_area && b_valid_point_dis) {
        float pt_dis_to_ego =
            std::sqrt(std::pow(position.x - current_point.x, 2) +
                      std::pow(position.y - current_point.y, 2));
        min_dis_to_ego = std::min(pt_dis_to_ego, min_dis_to_ego);
        if (b_start_point) {
          pre_point = current_point;
          b_start_point = false;
        } else {
          lane_boundary_nodes.push_back(Node(pre_point, current_point,
                                             boundary_type, lane_boundary.id,
                                             false));
          pre_point = current_point;
        }
      }
    }
    // should consider how many need to save!!!
    if (lane_boundary_nodes.size() > 0) {
      lanes.emplace_back(lane_boundary_nodes, std::string("proposal"),
                         boundary_type, -1, min_dis_to_ego, on_route_dis);
    }
  }

  return true;
}

} // namespace ddp
} // namespace msquare
