#include "perfect_scene.h"

#include "planner_interface.h"

#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

using namespace cv;

namespace perfect_scene{

std::vector<msquare::planning_math::Vec2d> ObstacleLabelData::getOdoPoints() const{
    std::vector<msquare::planning_math::Vec2d> odo_points;
    for(auto label_p: label_points){
        odo_points.emplace_back(label_p.x, label_p.y);
    }

    return odo_points;
}

std::vector<msquare::planning_math::LineSegment2d> ObstacleLabelData::getOdoLines() const{
    std::vector<msquare::planning_math::LineSegment2d> odo_lines;
    std::vector<msquare::planning_math::Vec2d> odo_points = getOdoPoints();

    for(int i = 0;i < odo_points.size()-1;i++){
        odo_lines.emplace_back(odo_points[i], odo_points[i+1]);
    }

    if(odo_points.size()>2){
        odo_lines.emplace_back(odo_points.back(), odo_points.front());
    }

    return odo_lines;
}

std::string
perfectScenePlan(const parking_scenario::Point2d &init_pose,
                 const parking_scenario::Point2d &target_pose,
                 const std::vector<ObstacleLabelData> &label_datas,
                 ParkingSlotType slot_type,
                 ParkingSlotSide slot_side /* = SIDE_UNKNOWN*/,
                 const std::vector<parking_scenario::Point2d> &slot_corners,
                 int init_speed /* = 0*/) {
  msquare::parking::OpenspaceDeciderOutput odo;

  odo.target_state.path_point.x = init_pose.x;
  odo.target_state.path_point.y = init_pose.y;
  odo.target_state.path_point.theta = init_pose.theta;

  odo.target_state.v = init_speed;

  std::cout << "target state velocity: " << odo.target_state.v << std::endl;

  odo.init_state.path_point.x = target_pose.x;
  odo.init_state.path_point.y = target_pose.y;
  odo.init_state.path_point.theta = target_pose.theta;

  if (slot_corners.empty()) {
    addMapBoundary(odo); // todo
  } else {
    addMapBoundaryWithSlotCorners(slot_corners, odo);
  }

  for (auto &d : label_datas) {
    if (d.obstacle_type == ObstacleType::WALL) {
      std::vector<msquare::planning_math::Vec2d> odo_points = d.getOdoPoints();
      for (auto &p : odo_points) {
        if (odo.map_boundary.IsPointIn(p)) {
          odo.points.push_back(p);
        }
      }
    } else if (d.obstacle_type == ObstacleType::CAR) {
      std::vector<msquare::planning_math::LineSegment2d> odo_lines =
          d.getOdoLines();
      for (auto &l : odo_lines) {
        if (odo.map_boundary.HasOverlap(l)) {
          odo.obstacle_lines.push_back(l);
        }
      }
    }
  }

  // if (slot_type == ParkingSlotType::UNKNOWN) {
  //   slot_type = getSlotType(init_pose, target_pose);
  // }

  if (slot_type == ParkingSlotType::PARALLEL) {
    slot_side = addParallelTlines(odo, slot_side);
  } else if (slot_type == ParkingSlotType::VERTICAL) {
    slot_side = addVerticalTlines(odo, slot_side);
  } else if (slot_type == ParkingSlotType::OBLIQUE) {
    slot_side = addObliqueTlines(slot_corners, odo, slot_side);
  }

  std::cout << "prefect scene plan:" << std::endl;

  msquare::SbpResult planner_res = planner_interface::planInterface(odo);

  std::cout << "res size:" << planner_res.x.size() << std::endl;

  nlohmann::json json_result = planner_res;
  nlohmann::json json_odo = odo;
  nlohmann::json json_slot_side = slot_side;
  nlohmann::json json_final;
  json_final["res"] = json_result;
  json_final["odo"] = json_odo;
  json_final["slot_side"] = json_slot_side;

  return json_final.dump();
}

std::string
perfectScenePlanSimple(bool reverse, bool center_at_init, bool car_as_points,
                       const parking_scenario::Point2d &init_pose,
                       const parking_scenario::Point2d &target_pose,
                       const std::vector<ObstacleLabelData> &label_datas,
                       int init_speed) {
  msquare::parking::OpenspaceDeciderOutput odo;

  odo.init_state.path_point.x = init_pose.x;
  odo.init_state.path_point.y = init_pose.y;
  odo.init_state.path_point.theta = init_pose.theta;
  odo.init_state.v = init_speed;
  odo.target_state.path_point.x = target_pose.x;
  odo.target_state.path_point.y = target_pose.y;
  odo.target_state.path_point.theta = target_pose.theta;
  odo.target_state.v = 0.0;
  if (reverse) {
    std::swap(odo.init_state, odo.target_state);
  }

  msquare::planning_math::Vec2d boundary_center(init_pose.x, init_pose.y);
  msquare::planning_math::Vec2d other_state(target_pose.x, target_pose.y);
  double boundary_heading = 0.0;
  if (center_at_init) {
    boundary_heading = init_pose.theta;
  } else {
    std::swap(boundary_center, other_state);
    boundary_heading = target_pose.theta;
  }
  double dx = other_state.x() - boundary_center.x();
  double dy = other_state.y() - boundary_center.y();
  double distance_in_length = std::abs(dx * std::cos(boundary_heading) +
                                       dy * std::sin(boundary_heading));
  double distance_in_width = std::abs(-dx * std::sin(boundary_heading) +
                                      dy * std::cos(boundary_heading));
  double boundary_length = std::max((distance_in_length + 3.0) * 2.0, 15.0);
  double boundary_width = std::max((distance_in_width + 3.0) * 2.0, 15.0);
  odo.map_boundary = msquare::planning_math::Box2d(
      boundary_center, boundary_heading, boundary_length, boundary_width);

  for (auto &d : label_datas) {
    if (d.obstacle_type == ObstacleType::WALL ||
        (d.obstacle_type == ObstacleType::CAR && car_as_points)) {
      std::vector<msquare::planning_math::Vec2d> odo_points = d.getOdoPoints();
      for (auto &p : odo_points) {
        odo.points.push_back(p);
      }
    } else if (d.obstacle_type == ObstacleType::CAR && !car_as_points) {
      std::vector<msquare::planning_math::LineSegment2d> odo_lines =
          d.getOdoLines();
      for (auto &l : odo_lines) {
        odo.obstacle_lines.push_back(l);
      }
    }
  }

  nlohmann::json json_odo = odo;
  nlohmann::json json_final;
  json_final["odo"] = json_odo;
  return json_final.dump();
}

Eigen::Isometry3d buildIsoLidar2rr(const std::vector<float>& vr, 
    const std::vector<float>& vt) {
    cv::Mat rvec = (cv::Mat_<double>(3, 1)<<vr[0], vr[1], vr[2]);
    cv::Mat tvec = (cv::Mat_<double>(3, 1)<<vt[0], vt[1], vt[2]);
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d rot;
    Eigen::Vector3d pos;
    cv::cv2eigen(R, rot);
    cv::cv2eigen(tvec, pos);
    Eigen::Isometry3d iso_lidar2rr = Eigen::Isometry3d::Identity();
    iso_lidar2rr.linear() = rot;
    iso_lidar2rr.translation() = pos;
    return iso_lidar2rr;
}

double regularizeAngle(double angle) {
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

double rad2Angle(double x)
{
    return x/M_PI*180.0; 
}

Eigen::Vector3d extractYawPitchRollFromRotMat(const Eigen::Matrix3d &rot_mat) {
    Eigen::Vector3d euler = rot_mat.eulerAngles(2, 1, 0);
    // TODO : 3*(analyze this threshold)
    if (std::abs(euler(1)) + std::abs(euler(2)) > 0.8 * M_PI) {
        euler(0) = regularizeAngle(euler(0) + M_PI);
        euler(1) = regularizeAngle(euler(1) + M_PI);
        euler(2) = regularizeAngle(euler(2) + M_PI);
    } else {
        euler(0) = regularizeAngle(euler(0));
        euler(1) = regularizeAngle(euler(1));
        euler(2) = regularizeAngle(euler(2));
    }
    return euler;
}


parking_scenario::Point2d convertLidarPose2EgoPose(
    const std::vector<double>& lidar_pose, 
    const std::vector<float>& vr, 
    const std::vector<float>& vt) {
    
    Eigen::Isometry3d iso_lidar2rr = buildIsoLidar2rr(vr, vt);
    Eigen::Isometry3d iso_rr2lidar = iso_lidar2rr.inverse();

    Eigen::Isometry3d iso_lidar2world = Eigen::Isometry3d::Identity();
    iso_lidar2world.translation() = Eigen::Vector3d(lidar_pose[0], lidar_pose[1],lidar_pose[2]);
    iso_lidar2world.linear() = Eigen::Quaterniond(lidar_pose[3], lidar_pose[4], lidar_pose[5], lidar_pose[6]).toRotationMatrix();
    Eigen::Isometry3d iso_rr2world = iso_lidar2world * iso_rr2lidar;
    Eigen::Vector3d pos = iso_rr2world.translation();
    Eigen::Matrix3d rot = iso_rr2world.linear();
    Eigen::Quaterniond quat(rot);
    Eigen::Vector3d euler = extractYawPitchRollFromRotMat(rot);

    parking_scenario::Point2d res(pos.x(), pos.y(), euler.x());
    return res;
    //sprintf(msg, "%lld %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", timestamp_us,
    //    pos.x(), pos.y(), pos.z(), quat.w(), quat.x(), quat.y(), quat.z(), rad2Angle(euler.x()), rad2Angle(euler.y()), rad2Angle(euler.z()));
    
}

ParkingSlotType getSlotType(const parking_scenario::Point2d& init_pose, const parking_scenario::Point2d& target_pose){
    double angle_diff = std::abs(msquare::planning_math::NormalizeAngle(init_pose.theta - target_pose.theta));

    if(angle_diff > M_PI_4 && angle_diff < 3 * M_PI_4){
        return ParkingSlotType::VERTICAL;
    }

    return ParkingSlotType::PARALLEL;
}

ParkingSlotType
getSlotTypeFromSlotCorners(const std::vector<parking_scenario::Point2d> &slot_corners) {
    msquare::planning_math::Vec2d p1_to_p0(slot_corners[0].x - slot_corners[1].x,
                                  slot_corners[0].y - slot_corners[1].y);
    msquare::planning_math::Vec2d p1_to_p2(slot_corners[2].x - slot_corners[1].x,
                                  slot_corners[2].y - slot_corners[1].y);

    if(fabs(p1_to_p0.InnerProd(p1_to_p2)) > M_PI/12) {
        return ParkingSlotType::OBLIQUE;
    }

    if (p1_to_p0.Length() > p1_to_p2.Length())
    {
      return ParkingSlotType::VERTICAL;
    }
    return ParkingSlotType::PARALLEL;    
}

void addMapBoundary(msquare::parking::OpenspaceDeciderOutput &odo){
    msquare::planning_math::Vec2d center(odo.init_state.path_point.x, odo.init_state.path_point.y);
    odo.map_boundary = msquare::planning_math::Box2d(
        center,
        odo.init_state.path_point.theta,
        15.0,
        15.0
    );
    return;
}

void addMapBoundaryWithSlotCorners(
    const std::vector<parking_scenario::Point2d> &slot_corners,
    msquare::parking::OpenspaceDeciderOutput &odo) {
  double center_x = (slot_corners[0].x + slot_corners[3].x) / 2;
  double center_y = (slot_corners[0].y + slot_corners[3].y) / 2;
  double theta = atan2(slot_corners[3].y - slot_corners[0].y,
                       slot_corners[3].x - slot_corners[0].x);
  msquare::planning_math::Vec2d center(center_x, center_y);
  odo.map_boundary = msquare::planning_math::Box2d(center, theta, 15.0, 15.0);
}

ParkingSlotSide addVerticalTlines(msquare::parking::OpenspaceDeciderOutput &odo,
    ParkingSlotSide slot_side/* = SIDE_UNKNOWN*/){
    double map_length = 18.0;

    const double DEFAULT_RULE_SLOT_WIDTH = 3.1;

    //  ego box
    double length = msquare::VehicleParam::Instance()->length;
    double width = msquare::VehicleParam::Instance()->width;
    double xx = msquare::VehicleParam::Instance()->center_to_geometry_center;
    double yy = 0;


    Pose2D local_frame{odo.init_state.path_point.x, 
        odo.init_state.path_point.y,
        odo.init_state.path_point.theta- M_PI_2};
    

    // get local poses
    Pose2D init_pose = msquare::planning_math::tf2d(local_frame,
                                    Pose2D(odo.target_state.path_point.x,
                                            odo.target_state.path_point.y,
                                            odo.target_state.path_point.theta));
    Pose2D target_pose = msquare::planning_math::tf2d(local_frame,
                                        Pose2D(odo.init_state.path_point.x,
                                            odo.init_state.path_point.y,
                                            odo.init_state.path_point.theta));

    msquare::planning_math::Vec2d init_vec(cos(init_pose.theta), sin(init_pose.theta));
    msquare::planning_math::Vec2d i2t_vec(target_pose.x - init_pose.x,
                                target_pose.y - init_pose.y);

    bool is_on_left = false;
    if (slot_side == SIDE_LEFT) {
        is_on_left = true;
    } else if (slot_side == SIDE_RIGHT) {
        is_on_left = false;
    } else {
        is_on_left = init_vec.CrossProd(i2t_vec) > 0;
        slot_side = is_on_left ? ParkingSlotSide::SIDE_LEFT : ParkingSlotSide::SIDE_RIGHT;
    }

    msquare::planning_math::Box2d local_ego(msquare::planning_math::Vec2d(0.0, 0.0), M_PI/2.0, length, width);

  
    std::cout<<"is on left:"<<is_on_left<<std::endl;
    // add map_noundary
    double mx = init_pose.x/2.0;
    double my = 4;
    msquare::planning_math::Vec2d mc(mx, my);

    msquare::planning_math::Box2d local_map = msquare::planning_math::Box2d(
        mc,
        0,
        map_length,
        map_length 
    );

    odo.map_boundary = tf2d_inv(local_frame, local_map);

    double mb_l = map_length / 2.0 - mx;
    double mb_r = map_length / 2.0 + mx;
    double mb_front = map_length - 5.0;
    double mb_back = 5.0;
    double scale_coeff = 1.4;

    std::vector<msquare::planning_math::Box2d> local_boxes;
    std::vector<msquare::planning_math::Vec2d> local_points;
    std::vector<msquare::planning_math::LineSegment2d> local_lines;

    for (auto &box : odo.obstacle_boxs) {
        local_boxes.push_back(msquare::planning_math::tf2d(local_frame, box));
    }
    for (auto &p : odo.points) {
        local_points.push_back(msquare::planning_math::tf2d(local_frame, p));
    }
    for (auto &l : odo.obstacle_lines) {
        local_lines.push_back(msquare::planning_math::tf2d(local_frame, l));
    }

    msquare::planning_math::Box2d left_box(msquare::planning_math::Vec2d(-width, 0.0 + xx + length*(scale_coeff-1.0)/2.0 ),  M_PI/2.0, length*scale_coeff, width*scale_coeff);
    msquare::planning_math::Box2d right_box(msquare::planning_math::Vec2d(width, 0.0 + xx + length*(scale_coeff-1.0)/2.0),  M_PI/2.0, length*scale_coeff, width*scale_coeff);

    double lx, ly, rx, ry;

    lx  = -DEFAULT_RULE_SLOT_WIDTH/2.0;    // max / max
    ly  = is_on_left ?  xx + length /2.0 - length *2/3.0 : xx + length /2.0;        // max /max
    rx  = DEFAULT_RULE_SLOT_WIDTH/2.0; // min /min
    ry = is_on_left ? xx + length /2.0 :  xx + length /2.0 - length *2/3.0; // max / max

    for (const auto &box : local_boxes) {
        if (left_box.HasOverlap(box)) {
            ly = std::max(box.max_y(), ly);
            lx = std::max(box.max_x(), lx);
        }
        
        if (right_box.HasOverlap(box)) {
            ry = std::max(box.max_y(), ry);
            rx = std::min(box.min_x(), rx);
        }
       
    }

    for (auto &p : local_points) {
        if (left_box.IsPointIn(p)) {
            ly = std::max(p.y(), ly);
            lx = std::max(p.x(), lx);
        }
        
        if (right_box.IsPointIn(p)) {
            ry = std::max(p.y(), ry);
            rx = std::min(p.x(), rx);
        }   
        
    }

    for (const auto &l : local_lines) {
        if (left_box.HasOverlap(l)) {
            ly = std::max(l.max_y(), ly);
            lx = std::max(l.max_x(), lx);
        }
        
        if (right_box.HasOverlap(l)) {
            ry = std::max(l.max_y(), ry);
            rx = std::min(l.min_x(), rx);
        }
      
    }



  // avoid lower region being too wide for parallel slot

    std::vector<msquare::planning_math::LineSegment2d> local_T_lines, T_lines;

    // upper
    // upper line 
    double mid_line = 8;
    double upper_line = mb_front;
    for (const auto &box : local_boxes) {
        if(!local_map.HasOverlap(box)){
            continue;
        }
        if(box.max_y() < mid_line){
            continue;
        }
        if(box.min_y() < mid_line){
            continue;
        }
        upper_line = std::min(upper_line, box.min_y());
    }

    for (auto &p : local_points) {
        if(!local_map.IsPointIn(p)){
            continue;
        }
        
        if(p.y() < mid_line){
            continue;
        }
        upper_line = std::min(upper_line, p.y());
    }

    for (const auto &l : local_lines) {
        if(!local_map.HasOverlap(l)){
            continue;
        }
        if(l.max_y() < mid_line){
            continue;
        }
        if(l.min_y() < mid_line){
            continue;
        }
        upper_line = std::min(upper_line, l.min_y());
    }


    local_T_lines.push_back(
        msquare::planning_math::LineSegment2d(msquare::planning_math::Vec2d(-mb_l , upper_line),
                        msquare::planning_math::Vec2d(mb_r, upper_line)));


    local_T_lines.push_back(
        msquare::planning_math::LineSegment2d(msquare::planning_math::Vec2d(-mb_l, ly),
                        msquare::planning_math::Vec2d(lx, ly)));
    local_T_lines.push_back(
        msquare::planning_math::LineSegment2d(msquare::planning_math::Vec2d(lx, ly),
                        msquare::planning_math::Vec2d(lx, -mb_back)));
    local_T_lines.push_back(
        msquare::planning_math::LineSegment2d(msquare::planning_math::Vec2d(rx, -mb_back),
                        msquare::planning_math::Vec2d(rx, ry)));
    local_T_lines.push_back(
        msquare::planning_math::LineSegment2d(msquare::planning_math::Vec2d(rx, ry),
                        msquare::planning_math::Vec2d(mb_r, ry)));

    for (auto &line : local_T_lines) {
        T_lines.push_back(tf2d_inv(local_frame, line));
    }

    odo.T_lines.road_upper_bound = T_lines[0];
    odo.T_lines.road_lower_left_bound = T_lines[1];
    odo.T_lines.slot_left_bound = T_lines[2];
    odo.T_lines.slot_right_bound = T_lines[3];
    odo.T_lines.road_lower_right_bound = T_lines[4];
    
    return slot_side;
}

ParkingSlotSide addParallelTlines(msquare::parking::OpenspaceDeciderOutput &odo,
    ParkingSlotSide slot_side/* = SIDE_UNKNOWN*/){
    
    double map_length = 18.0;
    Pose2D local_frame{odo.init_state.path_point.x, 
        odo.init_state.path_point.y,
        odo.init_state.path_point.theta};
    
    //  ego box
    double length = msquare::VehicleParam::Instance()->length;
    double width = msquare::VehicleParam::Instance()->width;
    double xx = msquare::VehicleParam::Instance()->center_to_geometry_center;
    double yy = 0;
    msquare::planning_math::Box2d local_ego(msquare::planning_math::Vec2d(xx, yy), 0.0, length, width);

    // get local poses
    Pose2D init_pose = msquare::planning_math::tf2d(local_frame,
                                  Pose2D(odo.target_state.path_point.x,
                                         odo.target_state.path_point.y,
                                         odo.target_state.path_point.theta));
  
    bool is_on_left = false;
    if (slot_side == SIDE_LEFT) {
        is_on_left = true;
    } else if (slot_side == SIDE_RIGHT) {
        is_on_left = false;
    } else {
        is_on_left = (init_pose.y < 0 ? true : false);
        slot_side = is_on_left ? ParkingSlotSide::SIDE_LEFT : ParkingSlotSide::SIDE_RIGHT;
    }

    std::cout<<"is on left:"<<is_on_left<<std::endl;

    // add map_noundary
    double mx = init_pose.x/2.0;
    double my = is_on_left ? -4.5 : 4.5;
    msquare::planning_math::Vec2d mc(mx, my);

    msquare::planning_math::Box2d local_map = msquare::planning_math::Box2d(
        mc,
        0,
        map_length,
        map_length - 3.0
    );

    odo.map_boundary = tf2d_inv(local_frame, local_map);


    double mb_l = is_on_left ? 3.0 :  map_length - 2 * 3.0;
    double mb_r = is_on_left ? map_length - 2 * 3.0 : 3.0;
    double mb_front = map_length /2.0 + mx;
    double mb_back = map_length / 2.0 - mx;
    double scale_coeff = 1.4;

    std::vector<msquare::planning_math::Box2d> local_boxes;
    std::vector<msquare::planning_math::Vec2d> local_points;
    std::vector<msquare::planning_math::LineSegment2d> local_lines;

    for (auto &box : odo.obstacle_boxs) {
        local_boxes.push_back(msquare::planning_math::tf2d(local_frame, box));
    }
    for (auto &p : odo.points) {
        local_points.push_back(msquare::planning_math::tf2d(local_frame, p));
    }
    for (auto &l : odo.obstacle_lines) {
        local_lines.push_back(msquare::planning_math::tf2d(local_frame, l));
    }

    double soffset = is_on_left? -1.0 : 1.0;
    msquare::planning_math::Box2d front_box(msquare::planning_math::Vec2d(length, 0.0 + soffset * (width*scale_coeff-1.0)/2.0), 0.0, length*scale_coeff, width*scale_coeff);
    msquare::planning_math::Box2d back_box(msquare::planning_math::Vec2d(-length, 0.0 + soffset * (width*scale_coeff-1.0)/2.0), 0.0, length*scale_coeff, width*scale_coeff);

    double lx, ly, rx, ry;
    double lower_y = (scale_coeff-1.0) * std::hypot(length, width) + width / 2.0;

    lx  = is_on_left? xx + 1 * length : xx - 1.0 * length;    // min / max
    ly  = is_on_left? width / 2.0 : -width / 2.0;        // min /up
    rx  = is_on_left? xx - 1.0 * length : xx + 1.0 * length; // max /min
    ry = is_on_left? width / 2.0 : -width / 2.0; // min / up

    for (const auto &box : local_boxes) {
        if(is_on_left){
            if (front_box.HasOverlap(box)) {
                ly = is_on_left? std::min(box.min_y(), ly):std::max(box.max_y(), ly);
                lx = is_on_left? std::min(box.min_x(), lx):std::max(box.max_x(), lx);
            }
            
            if (back_box.HasOverlap(box)) {
                ry = is_on_left? std::min(box.min_y(), ry):std::max(box.max_y(), ry);
                rx = is_on_left? std::max(box.max_x(), rx):std::min(box.min_x(), rx);
            }
        }else{
            if (back_box.HasOverlap(box)) {
                ly = std::max(box.max_y(), ly);
                lx = std::max(box.max_x(), lx);
            }
            
            if (front_box.HasOverlap(box)) {
                ry = std::max(box.max_y(), ry);
                rx = std::min(box.min_x(), rx);
            }
        }
    }

    for (auto &p : local_points) {
        if(is_on_left){
            if (front_box.IsPointIn(p)) {
                ly = is_on_left? std::min(p.y(), ly):std::max(p.y(), ly);
                lx = is_on_left? std::min(p.x(), lx):std::max(p.x(), lx);
            }
            
            if (back_box.IsPointIn(p)) {
                ry = is_on_left? std::min(p.y(), ry):std::max(p.y(), ry);
                rx = is_on_left? std::max(p.x(), rx):std::min(p.x(), rx);
            }   
        }else{
            if (back_box.IsPointIn(p)) {
                ly = is_on_left? std::min(p.y(), ly):std::max(p.y(), ly);
                lx = is_on_left? std::min(p.x(), lx):std::max(p.x(), lx);
            }
            
            if (front_box.IsPointIn(p)) {
                ry = is_on_left? std::min(p.y(), ry):std::max(p.y(), ry);
                rx = is_on_left? std::max(p.x(), rx):std::min(p.x(), rx);
            }   
        }
    }

    for (const auto &l : local_lines) {
        if(is_on_left){
            if (front_box.HasOverlap(l)) {
                ly = is_on_left? std::min(l.min_y(), ly):std::max(l.max_y(), ly);
                lx = is_on_left? std::min(l.min_x(), lx):std::max(l.max_x(), lx);
            }
            
            if (back_box.HasOverlap(l)) {
                ry = is_on_left? std::min(l.min_y(), ry):std::max(l.max_y(), ry);
                rx = is_on_left? std::max(l.max_x(), rx):std::min(l.min_x(), rx);
            }
        }else{
            if (back_box.HasOverlap(l)) {
                ly = std::max(l.max_y(), ly);
                lx = std::max(l.max_x(), lx);
            }
            
            if (front_box.HasOverlap(l)) {
                ry = std::max(l.max_y(), ry);
                rx = std::min(l.min_x(), rx);
            }
        }
    }



  // avoid lower region being too wide for parallel slot
  double max_adjust_width = 3.0;
  double max_region_width = max_adjust_width +length;
    if (std::abs(rx-lx) > max_region_width) {
        if(is_on_left){
            if (rx > -max_region_width / 2 + xx){
                lx = max_region_width + rx;
            }else if(lx < max_region_width / 2 + xx){
                rx = lx - max_region_width;
            }else{
                rx =  -max_region_width / 2 + xx;
                lx =  max_region_width / 2 + xx;
            }
        }
        else{
            if (lx > -max_region_width / 2 + xx){
                rx = max_region_width + lx;
            }else if(rx < max_region_width / 2 + xx){
                lx = rx - max_region_width;
            }else{
                lx =  -max_region_width / 2 + xx;
                rx =  max_region_width / 2 + xx;
            }
        }
    }
    

    std::vector<msquare::planning_math::LineSegment2d> local_T_lines, T_lines;

    // upper line 
    double mid_line = is_on_left ? -5 : 5;
    double upper_line = is_on_left ? - mb_r: mb_l;
    for (const auto &box : local_boxes) {
        if(!local_map.HasOverlap(box)){
            continue;
        }
        if(is_on_left){
            if(box.min_y() > mid_line){
                continue;
            }
            if(box.max_y() > mid_line){
                continue;
            }
            upper_line = std::max(upper_line, box.max_y());

        }
        else{
            
            if(box.max_y() < mid_line){
                continue;
            }
            if(box.min_y() < mid_line){
                continue;
            }
            upper_line = std::min(upper_line, box.min_y());
        }
    }

    for (auto &p : local_points) {
        if(!local_map.IsPointIn(p)){
            continue;
        }
        if(is_on_left){
            if(p.y() > mid_line){
                continue;
            }
            upper_line = std::max(upper_line, p.y());
        }else{
            if(p.y() < mid_line){
                continue;
            }
            upper_line = std::min(upper_line, p.y());
        }
    }

    for (const auto &l : local_lines) {
        if(!local_map.HasOverlap(l)){
            continue;
        }
        if(is_on_left){
            if(l.min_y() > mid_line){
                continue;
            }
            if(l.max_y() > mid_line){
                continue;
            }
            upper_line = std::max(upper_line, l.max_y());

        }
        else{
            
            if(l.max_y() < mid_line){
                continue;
            }
            if(l.min_y() < mid_line){
                continue;
            }
            upper_line = std::min(upper_line, l.min_y());
        }
    }



    local_T_lines.push_back(
        msquare::planning_math::LineSegment2d(msquare::planning_math::Vec2d(mb_front , upper_line),
                        msquare::planning_math::Vec2d(-mb_back,upper_line)));

    local_T_lines.push_back(
        msquare::planning_math::LineSegment2d(msquare::planning_math::Vec2d(is_on_left? mb_front: -mb_back, ly),
                        msquare::planning_math::Vec2d(lx, ly)));
    local_T_lines.push_back(
        msquare::planning_math::LineSegment2d(msquare::planning_math::Vec2d(lx, ly),
                        msquare::planning_math::Vec2d(lx, is_on_left? mb_l: -mb_r)));
    local_T_lines.push_back(
        msquare::planning_math::LineSegment2d(msquare::planning_math::Vec2d(rx, is_on_left? mb_l :-mb_r),
                        msquare::planning_math::Vec2d(rx, ry)));
    local_T_lines.push_back(
        msquare::planning_math::LineSegment2d(msquare::planning_math::Vec2d(rx, ry),
                        msquare::planning_math::Vec2d(is_on_left?-mb_back :mb_front, ry)));

    for (auto &line : local_T_lines) {
        T_lines.push_back(tf2d_inv(local_frame, line));
    }

    odo.T_lines.road_upper_bound = T_lines[0];
    odo.T_lines.road_lower_left_bound = T_lines[1];
    odo.T_lines.slot_left_bound = T_lines[2];
    odo.T_lines.slot_right_bound = T_lines[3];
    odo.T_lines.road_lower_right_bound = T_lines[4];
    
    return slot_side;
}

ParkingSlotSide addObliqueTlines(const std::vector<parking_scenario::Point2d> &slot_corners,
                 msquare::parking::OpenspaceDeciderOutput &odo,
                 ParkingSlotSide slot_side) {
  msquare::planning_math::Vec2d slot_front_left(slot_corners[0].x, slot_corners[0].y);
  msquare::planning_math::Vec2d slot_front_right(slot_corners[3].x, slot_corners[3].y);
  msquare::planning_math::Vec2d slot_front_center =
      (slot_front_left + slot_front_right) / 2;
  Pose2D local_frame_pose{slot_front_center.x(), slot_front_center.y(),
                          (slot_front_right - slot_front_left).Angle()};

  msquare::planning_math::Vec2d local_front_left =
      msquare::planning_math::tf2d(local_frame_pose, slot_front_left);
  msquare::planning_math::Vec2d local_front_right =
      msquare::planning_math::tf2d(local_frame_pose, slot_front_right);

  // get local poses
  Pose2D init_pose = msquare::planning_math::tf2d(
      local_frame_pose,
      Pose2D(odo.target_state.path_point.x, odo.target_state.path_point.y,
             odo.target_state.path_point.theta));
  Pose2D target_pose = msquare::planning_math::tf2d(
      local_frame_pose,
      Pose2D(odo.init_state.path_point.x, odo.init_state.path_point.y,
             odo.init_state.path_point.theta));

  msquare::planning_math::Vec2d init_vec(cos(init_pose.theta),
                                         sin(init_pose.theta));
  msquare::planning_math::Vec2d i2t_vec(target_pose.x - init_pose.x,
                                        target_pose.y - init_pose.y);
  bool is_on_left = false;
  if (slot_side == ParkingSlotSide::SIDE_LEFT) {
    is_on_left = true;
  } else if (slot_side == ParkingSlotSide::SIDE_RIGHT) {
    is_on_left = false;
  } else {
    is_on_left = init_vec.CrossProd(i2t_vec) > 0;
    slot_side =
        is_on_left ? ParkingSlotSide::SIDE_LEFT : ParkingSlotSide::SIDE_RIGHT;
  }

  //  ego box
  double length = msquare::VehicleParam::Instance()->length;
  double width = msquare::VehicleParam::Instance()->width;
  double delta = msquare::VehicleParam::Instance()->center_to_geometry_center;
  double xx = target_pose.x + delta * cos(target_pose.theta);
  double yy = target_pose.y + delta * sin(target_pose.theta);
  msquare::planning_math::Box2d local_ego_box(msquare::planning_math::Vec2d(xx, yy),
                                     target_pose.theta, length, width);

  std::vector<msquare::planning_math::LineSegment2d> local_T_lines, T_lines;

  msquare::planning_math::Box2d local_map_boundary =
      msquare::planning_math::tf2d(local_frame_pose, odo.map_boundary);

  std::vector<msquare::planning_math::Box2d> local_boxes;
  std::vector<msquare::planning_math::Vec2d> local_points;
  std::vector<msquare::planning_math::LineSegment2d> local_lines;
  std::vector<msquare::planning_math::Box2d> local_lower_boxes;
  std::vector<msquare::planning_math::Vec2d> local_lower_points;
  std::vector<msquare::planning_math::LineSegment2d> local_lower_lines;
  for (auto &box : odo.obstacle_boxs) {
    local_boxes.push_back(msquare::planning_math::tf2d(local_frame_pose, box));
  }
  for (auto &p : odo.points) {
    local_points.push_back(msquare::planning_math::tf2d(local_frame_pose, p));
  }
  for (auto &l : odo.obstacle_lines) {
    local_lines.push_back(msquare::planning_math::tf2d(local_frame_pose, l));
  }

  Pose2D start_pose = msquare::planning_math::tf2d(
      local_frame_pose,
      Pose2D(odo.target_state.path_point.x, odo.target_state.path_point.y,
             odo.target_state.path_point.theta));
  double mid_line = start_pose.y + 2.0;
  double upper_line = local_map_boundary.max_y();
  for (const auto &box : local_boxes) {
    if (!local_map_boundary.HasOverlap(box)) {
      continue;
    }
    if (box.max_y() < mid_line) {
      local_lower_boxes.push_back(box);
      continue;
    }
    if (box.min_y() < mid_line) {
      continue;
    }
    upper_line = std::min(upper_line, box.min_y());
  }

  for (auto &p : local_points) {
    if (!local_map_boundary.IsPointIn(p)) {
      continue;
    }

    if (p.y() < mid_line) {
      local_lower_points.push_back(p);
      continue;
    }
    upper_line = std::min(upper_line, p.y());
  }

  for (const auto &l : local_lines) {
    if (!local_map_boundary.HasOverlap(l)) {
      continue;
    }
    if (l.max_y() < mid_line) {
      local_lower_lines.push_back(l);
      continue;
    }
    if (l.min_y() < mid_line) {
      continue;
    }
    upper_line = std::min(upper_line, l.min_y());
  }
  local_T_lines.push_back(msquare::planning_math::LineSegment2d(
      msquare::planning_math::Vec2d(local_map_boundary.min_x(), upper_line),
      msquare::planning_math::Vec2d(local_map_boundary.max_x(), upper_line)));

  msquare::planning_math::Box2d slot_box_on_left(local_ego_box);
  slot_box_on_left.SetLength(slot_box_on_left.length() + 2.0);
  msquare::planning_math::Box2d slot_box_on_right(local_ego_box);
  slot_box_on_right.SetLength(slot_box_on_right.length() + 2.0);
  slot_box_on_right.Shift(local_front_right - local_front_left);
  slot_box_on_left.Shift(local_front_left - local_front_right);

  msquare::planning_math::Vec2d slot_back_left(slot_corners[1].x, slot_corners[1].y);
  msquare::planning_math::Vec2d local_back_left =
      msquare::planning_math::tf2d(local_frame_pose, slot_back_left);
  double local_heading = atan2(local_front_left.y() - local_back_left.y(),
                               local_front_left.x() - local_back_left.x());
  double left_lower_height = local_map_boundary.min_y();
  double right_lower_height = local_map_boundary.min_y();
  double left_x = local_map_boundary.min_x();
  double right_x = local_map_boundary.max_x();
  for (const auto &box : local_lower_boxes) {
    if (box.max_y() < -msquare::VehicleParam::Instance()->front_edge_to_center) {
      continue;
    }
    if (box.HasOverlap(slot_box_on_left)) {
      // process left box
      left_lower_height = std::max(box.max_y(), left_lower_height);
      for (auto pt : box.GetAllCorners()) {
        left_x = std::max(pt.x() - pt.y() / tan(local_heading), left_x);
      }
    } else if (box.HasOverlap(slot_box_on_right)) {
      // process right box
      right_lower_height = std::max(box.max_y(), right_lower_height);
      for (auto pt : box.GetAllCorners()) {
        right_x = std::min(pt.x() - pt.y() / tan(local_heading), right_x);
      }
    }
  }
  for (auto &p : local_lower_points) {
    if (p.y() < -msquare::VehicleParam::Instance()->front_edge_to_center) {
      continue;
    }
    if (slot_box_on_left.IsPointIn(p)) {
      left_lower_height = std::max(p.y(), left_lower_height);
      left_x = std::max(p.x() - p.y() / tan(local_heading), left_x);
    } else if (slot_box_on_right.IsPointIn(p)) {
      right_lower_height = std::max(p.y(), right_lower_height);
      right_x = std::min(p.x() - p.y() / tan(local_heading), right_x);
    }
  }

  for (const auto &line : local_lower_lines) {
    if (line.max_y() < -msquare::VehicleParam::Instance()->front_edge_to_center) {
      continue;
    }
    if (slot_box_on_left.HasOverlap(line)) {
      // process left box
      left_lower_height = std::max(line.max_y(), left_lower_height);
      auto l_start = line.start();
      auto l_end = line.end();
      left_x = std::max(l_start.x() - l_start.y() / tan(local_heading), left_x);
      left_x = std::max(l_end.x() - l_end.y() / tan(local_heading), left_x);
    } else if (slot_box_on_right.HasOverlap(line)) {
      // process right box
      right_lower_height = std::max(line.max_y(), right_lower_height);
      auto l_start = line.start();
      auto l_end = line.end();
      right_x =
          std::min(l_start.x() - l_start.y() / tan(local_heading), right_x);
      right_x = std::min(l_end.x() - l_end.y() / tan(local_heading), right_x);
    }
  }

  const double DEFAULT_RULE_SLOT_WIDTH = 3.1 / sin(local_heading);
  if (is_on_left) {
    right_x =
        std::min(right_x, std::max(DEFAULT_RULE_SLOT_WIDTH / 2, right_x - 0.5));
    right_lower_height = -0.5;
    left_x =
        std::max(left_x, std::min(-DEFAULT_RULE_SLOT_WIDTH / 2, left_x + 0.5));
    left_lower_height = std::max(-2.0 * length / 3.0,
                                 left_lower_height);
  } else {
    left_x =
        std::max(left_x, std::min(-DEFAULT_RULE_SLOT_WIDTH / 2, left_x + 0.5));
    left_lower_height = -0.5;
    right_x =
        std::min(right_x, std::max(DEFAULT_RULE_SLOT_WIDTH / 2, right_x - 0.5));
    right_lower_height = std::max(-2.0 * length / 3.0,
                                  right_lower_height);
  }
  left_x = std::max(left_x, -DEFAULT_RULE_SLOT_WIDTH / 2);
  right_x = std::min(right_x, DEFAULT_RULE_SLOT_WIDTH / 2);

  local_T_lines.push_back(
      msquare::planning_math::LineSegment2d(msquare::planning_math::Vec2d(local_map_boundary.min_x(), left_lower_height),
                    msquare::planning_math::Vec2d(left_lower_height / tan(local_heading) + left_x,
                          left_lower_height)));
  local_T_lines.push_back(msquare::planning_math::LineSegment2d(
      msquare::planning_math::Vec2d(left_lower_height / tan(local_heading) + left_x, left_lower_height),
      msquare::planning_math::Vec2d(local_map_boundary.min_y() / tan(local_heading) + left_x,
            local_map_boundary.min_y())));
  local_T_lines.push_back(msquare::planning_math::LineSegment2d(
      msquare::planning_math::Vec2d(local_map_boundary.min_y() / tan(local_heading) + right_x,
            local_map_boundary.min_y()),
      msquare::planning_math::Vec2d(right_lower_height / tan(local_heading) + right_x,
            right_lower_height)));
  local_T_lines.push_back(
     msquare::planning_math::LineSegment2d(msquare::planning_math::Vec2d(right_lower_height / tan(local_heading) + right_x,
                          right_lower_height),
                    msquare::planning_math::Vec2d(local_map_boundary.max_x(), right_lower_height)));

  for (auto &line : local_T_lines) {
    T_lines.push_back(msquare::planning_math::tf2d_inv(local_frame_pose, line));
  }

  odo.T_lines.road_upper_bound = T_lines[0];
  odo.T_lines.road_lower_left_bound = T_lines[1];
  odo.T_lines.slot_left_bound = T_lines[2];
  odo.T_lines.slot_right_bound = T_lines[3];
  odo.T_lines.road_lower_right_bound = T_lines[4];
  return slot_side;
}

}   // end namespace perfect_scene