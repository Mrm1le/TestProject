#include "common/parking_obstacle.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <utility>

#include "common/config/vehicle_param.h"
#include "common/math/common_utils.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"

namespace msquare {

namespace parking {

namespace {
const double kStBoundaryDeltaS = 0.2;       // meters
const double kStBoundarySparseDeltaS = 1.0; // meters
const double kStBoundaryDeltaT = 0.05;      // seconds
const double kHackStepObstacleHeight = 0.1;
const double kHackDefaultObstacleHeight = 1.0;
} // namespace

// static const int pedestrain_type_id = 10001;

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         Obstacle::ObjectTagCaseHash>
    Obstacle::s_longitudinal_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0},
        {ObjectDecisionType::kOvertake, 100},
        {ObjectDecisionType::kFollow, 200},
        {ObjectDecisionType::kYield, 300},
        {ObjectDecisionType::kStop, 400}};

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         Obstacle::ObjectTagCaseHash>
    Obstacle::s_lateral_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0}, {ObjectDecisionType::kNudge, 100}};

// Obstacle::Obstacle(int id, const Point3D &point,
//                    const double v_ego, const Pose2D &ego_pose)
//     : id_(id), perception_id_(id), is_static_(true),
//       perception_bounding_box_(
//           {ego_pose.x + point.x * cos(ego_pose.theta) - point.y *
//           sin(ego_pose.theta) + cos(ego_pose.theta) *
//           VehicleParam::Instance()->front_edge_to_center, ego_pose.y +
//           point.x * sin(ego_pose.theta) + point.y * cos(ego_pose.theta) +
//           sin(ego_pose.theta) *
//           VehicleParam::Instance()->front_edge_to_center}, ego_pose.theta,
//           0.1,
//           0.1) {

//   is_caution_level_obstacle_ = true;
//   is_truncated_ = false;
//   is_st_boundary_constructed_ = false;
//   is_zoomed_ = false;
//   truncated_time_ = std::numeric_limits<double>::infinity();
//   type_ = ObjectType::NOT_KNOW;

//   // auto speed = perception_obstacle.velocity;
//   perception_speed_ = 0.0;
//   // todo: add priority for obstacle

//   is_virtual_ = id_ < 0;
//   prob_ = 1.0;
//   speed_ = 0.0;
//   carte_pose_wrt_ego_ = Pose2D(point.x, point.y, 0);

// }

Obstacle::Obstacle(int id, const Point3D &point, bool is_freespace,
                   const GroundLineType fusion_type)
    : id_(id), perception_id_(id), is_static_(true),
      perception_bounding_box_({point.x, point.y}, 0.0, 0.1, 0.1),
      point_({point.x, point.y, point.z}) {

  is_caution_level_obstacle_ = true;
  is_truncated_ = false;
  is_st_boundary_constructed_ = false;
  is_zoomed_ = false;
  truncated_time_ = std::numeric_limits<double>::infinity();
  type_ = is_freespace ? ObjectType::FREESPACE : ObjectType::NOT_KNOW;

  // auto speed = perception_obstacle.velocity;
  perception_speed_ = 0.0;
  // todo: add priority for obstacle

  is_virtual_ = id_ < 0;
  prob_ = 1.0;
  speed_ = 0.0;

  // step obstacle's height need small large than other obstacle
  if (/* fusion_type == GroundLineType::GROUND_LINE_USS_TYPE_STEP ||  */
      fusion_type == GroundLineType::GROUND_LINE_USS_TYPE_SIDE_POINT_STEP ||
      fusion_type == GroundLineType::GROUND_LINE_USS_TYPE_POINT_GENERAL_STEP) {
    point_.z = kHackStepObstacleHeight;
  } else {
    point_.z = kHackDefaultObstacleHeight;
  }
}

Obstacle::Obstacle(int id, const Point3D &point, const ObjectType &type)
    : id_(id), perception_id_(id), is_static_(true),
      perception_bounding_box_({point.x, point.y}, 0.0, 0.1, 0.1), type_(type) {

  is_caution_level_obstacle_ = true;
  is_truncated_ = false;
  is_st_boundary_constructed_ = false;
  is_zoomed_ = false;
  truncated_time_ = std::numeric_limits<double>::infinity();

  // auto speed = perception_obstacle.velocity;
  perception_speed_ = 0.0;
  // todo: add priority for obstacle

  is_virtual_ = id_ < 0;
  prob_ = 1.0;
  speed_ = 0.0;
}

Obstacle::Obstacle(int id, const std::vector<planning_math::Vec2d> &points)
    : id_(id), perception_id_(id), is_static_(true),
      perception_points_(points) {

  is_caution_level_obstacle_ = true;
  is_truncated_ = false;
  is_st_boundary_constructed_ = false;
  is_zoomed_ = false;
  truncated_time_ = std::numeric_limits<double>::infinity();
  type_ = ObjectType::FREESPACE;

  // auto speed = perception_obstacle.velocity;
  perception_speed_ = 0.0;
  // todo: add priority for obstacle

  is_virtual_ = id_ < 0;
  prob_ = 1.0;
  speed_ = 0.0;
}

// Obstacle::Obstacle(int id, const Point3D &point1, const Point3D &point2,
//                    const double v_ego, const Pose2D &ego_pose)
//     : id_(id), perception_id_(id), is_static_(true),
//       perception_line_(
//         planning_math::Vec2d{
//           ego_pose.x + point1.x * cos(ego_pose.theta) - point1.y *
//           sin(ego_pose.theta) + cos(ego_pose.theta) *
//           VehicleParam::Instance()->front_edge_to_center, ego_pose.y +
//           point1.x * sin(ego_pose.theta) + point1.y * cos(ego_pose.theta) +
//           sin(ego_pose.theta) *
//           VehicleParam::Instance()->front_edge_to_center},
//         planning_math::Vec2d{
//           ego_pose.x + point2.x * cos(ego_pose.theta) - point2.y *
//           sin(ego_pose.theta) + cos(ego_pose.theta) *
//           VehicleParam::Instance()->front_edge_to_center, ego_pose.y +
//           point2.x * sin(ego_pose.theta) + point2.y * cos(ego_pose.theta) +
//           sin(ego_pose.theta) *
//           VehicleParam::Instance()->front_edge_to_center}
//       ) {

//   is_caution_level_obstacle_ = true;
//   is_truncated_ = false;
//   is_st_boundary_constructed_ = false;
//   is_zoomed_ = false;
//   truncated_time_ = std::numeric_limits<double>::infinity();
//   type_ = ObjectType::NOT_KNOW;

//   // auto speed = perception_obstacle.velocity;
//   perception_speed_ = 0.0;
//   // todo: add priority for obstacle

//   is_virtual_ = id_ < 0;
//   prob_ = 1.0;
//   speed_ = 0.0;
// }

// Obstacle::Obstacle(int id, const planning_math::LineSegment2d &line,
//                    const double v_ego, const Pose2D &ego_pose)
//     : id_(id), perception_id_(id), is_static_(true),
//       perception_bounding_box_(
//           {ego_pose.x + line.center().x() * cos(ego_pose.theta) -
//           line.center().y() * sin(ego_pose.theta) + std::cos(ego_pose.theta)
//           * VehicleParam::Instance()->front_edge_to_center,
//            ego_pose.y + line.center().x() * sin(ego_pose.theta) +
//            line.center().y() * cos(ego_pose.theta) + std::sin(ego_pose.theta)
//            * VehicleParam::Instance()->front_edge_to_center}, line.heading()
//            + ego_pose.theta, line.length(), 0.1),
//       perception_line_(
//         planning_math::Vec2d{
//           ego_pose.x + line.start().x() * cos(ego_pose.theta) -
//           line.start().y() * sin(ego_pose.theta) + cos(ego_pose.theta) *
//           VehicleParam::Instance()->front_edge_to_center, ego_pose.y +
//           line.start().x() * sin(ego_pose.theta) + line.start().y() *
//           cos(ego_pose.theta) + sin(ego_pose.theta) *
//           VehicleParam::Instance()->front_edge_to_center},
//         planning_math::Vec2d{
//           ego_pose.x + line.end().x() * cos(ego_pose.theta) - line.end().y()
//           * sin(ego_pose.theta) + cos(ego_pose.theta) *
//           VehicleParam::Instance()->front_edge_to_center, ego_pose.y +
//           line.end().x() * sin(ego_pose.theta) + line.end().y() *
//           cos(ego_pose.theta) + sin(ego_pose.theta) *
//           VehicleParam::Instance()->front_edge_to_center}
//       )
// {

//   is_caution_level_obstacle_ = true;
//   is_truncated_ = false;
//   is_st_boundary_constructed_ = false;
//   is_zoomed_ = false;
//   truncated_time_ = std::numeric_limits<double>::infinity();
//   type_ = ObjectType::FREESPACE;

//   // auto speed = perception_obstacle.velocity;
//   perception_speed_ = 0.0;
//   // todo: add priority for obstacle

//   is_virtual_ = id_ < 0;
//   prob_ = 1.0;
//   speed_ = 0.0;
// }

Obstacle::Obstacle(int id, const planning_math::LineSegment2d &line,
                   const GroundLineType fusion_type)
    : id_(id), perception_id_(id), is_static_(true),
      perception_bounding_box_({line.center().x(), line.center().y()},
                               line.heading(), line.length(), 0.1),
      perception_line_(planning_math::Vec2d{line.start().x(), line.start().y()},
                       planning_math::Vec2d{line.end().x(), line.end().y()}) {

  is_caution_level_obstacle_ = true;
  is_truncated_ = false;
  is_st_boundary_constructed_ = false;
  is_zoomed_ = false;
  truncated_time_ = std::numeric_limits<double>::infinity();
  type_ = ObjectType::FREESPACE;
  line_fusion_type_ = fusion_type;

  // auto speed = perception_obstacle.velocity;
  perception_speed_ = 0.0;
  // todo: add priority for obstacle

  is_virtual_ = id_ < 0;
  prob_ = 1.0;
  speed_ = 0.0;
}

// Obstacle::Obstacle(int id,
//                    const momenta_msgs::FusionObject &perception_obstacle,
//                    const bool is_static)
//     : id_(id), perception_id_(perception_obstacle.track_id),
//       is_static_(is_static), perception_obstacle_(perception_obstacle),
//       perception_bounding_box_(
//           {perception_obstacle.position.x, perception_obstacle.position.y},
//           perception_obstacle.theta, perception_obstacle.length,
//           perception_obstacle.width) {
//   is_caution_level_obstacle_ = true;
//   is_truncated_ = false;
//   is_st_boundary_constructed_ = false;
//   is_zoomed_ = false;
//   truncated_time_ = std::numeric_limits<double>::infinity();
//   switch (perception_obstacle.type) {
//   case 10001:
//     type_ = ObjectType::PEDESTRIAN;
//     break;
//   case 10002:
//     type_ = ObjectType::OFO;
//     break;
//   case 0:
//     type_ = ObjectType::NOT_KNOW;
//     break;
//   case 1:
//     type_ = ObjectType::COUPE;
//     break;
//   case 2:
//     type_ = ObjectType::TRANSPORT_TRUNK;
//     break;
//   case 3:
//     type_ = ObjectType::BUS;
//     break;
//   case 4:
//     type_ = ObjectType::ENGINEER_TRUCK;
//     break;
//   case 5:
//     type_ = ObjectType::TRICYCLE;
//     break;
//   case -1:
//     type_ = ObjectType::STOP_LINE;
//     break;
//   default:
//     type_ = ObjectType::NOT_KNOW;
//     break;
//   }
//   auto speed = perception_obstacle.velocity;
//   perception_speed_ = std::hypot(speed.x, std::hypot(speed.y, speed.z));
//   // todo: add priority for obstacle

//   std::vector<planning_math::Vec2d> polygon_points;
//   // todo: add polygon with more than 2 points interface
//   perception_bounding_box_.GetAllCorners(&polygon_points);
//   mph_assert(planning_math::Polygon2d::ComputeConvexHull(polygon_points,
//                                                      &perception_polygon_));
//   is_virtual_ = id_ < 0;
//   prob_ = 1.0;
//   speed_ = std::hypot(perception_obstacle.velocity.z,
//                       std::hypot(perception_obstacle.velocity.x,
//                                  perception_obstacle.velocity.y));
// }
Obstacle::Obstacle(int id, const FusionObject &object, bool is_static)
    : id_(id), perception_id_(object.track_id), is_static_(is_static),
      is_sf_static_(object.is_static), sf_type_(object.type),
      point_({object.position.x, object.position.y, 0.0}),
      perception_bounding_box_({object.position.x, object.position.y},
                               object.heading_yaw, object.shape.length,
                               object.shape.width) {
  const double PI = 3.141592653589793238463;
  is_caution_level_obstacle_ = true;
  is_truncated_ = false;
  is_st_boundary_constructed_ = false;
  is_zoomed_ = false;
  is_angle_consistent_ = true;
  truncated_time_ = std::numeric_limits<double>::infinity();
  switch (static_cast<int>(object.type)) {
  case 10001:
    type_ = ObjectType::PEDESTRIAN;
    is_static_ = false;
    break;
  case 10002:
    // type_ = ObjectType::OFO;
    type_ = ObjectType::PEDESTRIAN;
    // is_static_ = false;
    break;
  case 20001:
    type_ = ObjectType::CONE_BUCKET;
    is_static_ = true;
    break;
  case 0:
    type_ = ObjectType::NOT_KNOW;
    break;
  case 1:
    type_ = ObjectType::COUPE;
    break;
  case 2:
    type_ = ObjectType::COUPE;
    // type_ = ObjectType::TRANSPORT_TRUNK;
    break;
  case 3:
    type_ = ObjectType::COUPE;
    // type_ = ObjectType::BUS;
    break;
  case 4:
    type_ = ObjectType::COUPE;
    // type_ = ObjectType::ENGINEER_TRUCK;
    break;
  case 5:
    type_ = ObjectType::COUPE;
    // type_ = ObjectType::TRICYCLE;
    break;
  case -1:
    type_ = ObjectType::STOP_LINE;
    break;
  default:
    type_ = ObjectType::NOT_KNOW;
    break;
  }
  // auto speed = perception_obstacle.velocity;
  perception_speed_ =
      std::hypot(object.relative_velocity.x, object.relative_velocity.y);
  // todo: add priority for obstacle
  speed_yaw_relative_ego_ =
      std::atan2(object.relative_velocity.y, object.relative_velocity.x);

  std::vector<planning_math::Vec2d> polygon_points;
  // todo: add polygon with more than 2 points interface

  perception_bounding_box_.GetAllCorners(&polygon_points);
  (void)planning_math::Polygon2d::ComputeConvexHull(polygon_points,
                                                    &perception_polygon_);

  is_virtual_ = id_ < 0;
  prob_ = 1.0;
  speed_ = perception_speed_;
  speed_yaw_ = std::atan2(object.velocity.y, object.velocity.x);
  is_confident_ = true;
  const auto &box = perception_bounding_box_;
  box_base_center_ = planning_math::Vec2d(
      box.center_x() - box.cos_heading() * box.half_length(),
      box.center_y() - box.sin_heading() * box.half_length());
  box_theta_ = box.heading();
  box_length_ = box.length();
  box_width_ = box.width();

  Pose2D carte_pose_wrt_ego{object.relative_position.x -
                                VehicleParam::Instance()->front_edge_to_center,
                            object.relative_position.y,
                            object.relative_heading_yaw};
  Point2D carte_vel_wrt_ego{object.relative_velocity.x,
                            object.relative_velocity.y};
  carte_pose_wrt_ego_ = carte_pose_wrt_ego;
  carte_vel_wrt_ego_ = carte_vel_wrt_ego;

  double consistent_threshold = 30.0;
  double angle_difference = std::abs(box_theta_ - speed_yaw_) / PI * 180;
  // if(std::abs(speed_) > 1e-3){
  //   is_angle_consistent_ =
  //   !(std::abs(std::abs(std::fmod(angle_difference,180))-90)<consistent_threshold);
  //   //consistent when the angle difference is smaller than 90-30 degree
  // }
}

Obstacle::Obstacle(int id, const FusionObject &object, bool is_static,
                   const Pose2D &ego_pose)
    : id_(id), perception_id_(object.track_id), is_static_(is_static),
      perception_bounding_box_(
          {ego_pose.x + object.relative_position.x * cos(ego_pose.theta) -
               object.relative_position.y * sin(ego_pose.theta),
           ego_pose.y + object.relative_position.x * sin(ego_pose.theta) +
               object.relative_position.y * cos(ego_pose.theta)},
          ego_pose.theta + object.relative_heading_yaw, object.shape.length,
          object.shape.width) {
  const double PI = 3.141592653589793238463;
  is_caution_level_obstacle_ = true;
  is_truncated_ = false;
  is_st_boundary_constructed_ = false;
  is_zoomed_ = false;
  is_angle_consistent_ = true;
  truncated_time_ = std::numeric_limits<double>::infinity();
  switch (static_cast<int>(object.type)) {
  case 10001:
    type_ = ObjectType::PEDESTRIAN;
    is_static_ = false;
    break;
  case 10002:
    type_ = ObjectType::PEDESTRIAN;
    is_static_ = false;
    // type_ = ObjectType::OFO;
    break;
  case 20001:
    type_ = ObjectType::CONE_BUCKET;
    is_static_ = true;
    break;
  case 0:
    type_ = ObjectType::NOT_KNOW;
    break;
  case 1:
    type_ = ObjectType::COUPE;
    break;
  case 2:
    type_ = ObjectType::COUPE;
    // type_ = ObjectType::TRANSPORT_TRUNK;
    break;
  case 3:
    type_ = ObjectType::COUPE;
    // type_ = ObjectType::BUS;
    break;
  case 4:
    type_ = ObjectType::COUPE;
    // type_ = ObjectType::ENGINEER_TRUCK;
    break;
  case 5:
    type_ = ObjectType::COUPE;
    // type_ = ObjectType::TRICYCLE;
    break;
  case -1:
    type_ = ObjectType::STOP_LINE;
    break;
  default:
    type_ = ObjectType::NOT_KNOW;
    break;
  }
  // auto speed = perception_obstacle.velocity;
  perception_speed_ =
      std::hypot(object.relative_velocity.x, object.relative_velocity.y);
  // todo: add priority for obstacle
  speed_yaw_relative_ego_ =
      std::atan2(object.relative_velocity.y, object.relative_velocity.x);

  std::vector<planning_math::Vec2d> polygon_points;
  // todo: add polygon with more than 2 points interface

  perception_bounding_box_.GetAllCorners(&polygon_points);
  (void)planning_math::Polygon2d::ComputeConvexHull(polygon_points,
                                                    &perception_polygon_);

  is_virtual_ = id_ < 0;
  prob_ = 1.0;
  speed_ = perception_speed_;
  speed_yaw_ = std::atan2(object.velocity.y, object.velocity.x);
  is_confident_ = true;
  const auto &box = perception_bounding_box_;
  box_base_center_ = planning_math::Vec2d(
      box.center_x() - box.cos_heading() * box.half_length(),
      box.center_y() - box.sin_heading() * box.half_length());
  box_theta_ = box.heading();
  box_length_ = box.length();
  box_width_ = box.width();

  Pose2D carte_pose_wrt_ego{object.relative_position.x -
                                VehicleParam::Instance()->front_edge_to_center,
                            object.relative_position.y,
                            object.relative_heading_yaw};
  Point2D carte_vel_wrt_ego{object.relative_velocity.x,
                            object.relative_velocity.y};
  carte_pose_wrt_ego_ = carte_pose_wrt_ego;
  carte_vel_wrt_ego_ = carte_vel_wrt_ego;

  double consistent_threshold = 30.0;
  double angle_difference = std::abs(box_theta_ - speed_yaw_) / PI * 180;
  // if(std::abs(speed_) > 1e-3){
  //   is_angle_consistent_ =
  //   !(std::abs(std::abs(std::fmod(angle_difference,180))-90)<consistent_threshold);
  //   //consistent when the angle difference is smaller than 90-30 degree
  // }
}

// Obstacle::Obstacle(int id, const VisionCar &vision_car_obstacle,
//                   bool is_static, const double v_ego, const Pose2D &ego_pose,
//                   bool use_v_ego)
//     : id_(id), perception_id_(vision_car_obstacle.track_id),
//     is_static_(is_static),
//       perception_bounding_box_(
//           {ego_pose.x + vision_car_obstacle.d3_x * cos(ego_pose.theta) -
//           vision_car_obstacle.d3_y * sin(ego_pose.theta) +
//           std::cos(ego_pose.theta) *
//           VehicleParam::Instance()->front_edge_to_center, ego_pose.y +
//           vision_car_obstacle.d3_x * sin(ego_pose.theta) +
//           vision_car_obstacle.d3_y * cos(ego_pose.theta) +
//           std::sin(ego_pose.theta) *
//           VehicleParam::Instance()->front_edge_to_center},
//           vision_car_obstacle.theta + ego_pose.theta,
//           std::hypot(vision_car_obstacle.corners[1].x -
//           vision_car_obstacle.corners[3].x,
//                      vision_car_obstacle.corners[1].y -
//                      vision_car_obstacle.corners[3].y),
//           std::hypot(vision_car_obstacle.corners[0].x -
//           vision_car_obstacle.corners[1].x,
//                     vision_car_obstacle.corners[0].y -
//                     vision_car_obstacle.corners[1].y)) {
//   const double PI  =3.141592653589793238463;
//   is_caution_level_obstacle_ = true;
//   is_truncated_ = false;
//   is_st_boundary_constructed_ = false;
//   is_zoomed_ = false;
//   is_angle_consistent_ = true;
//   truncated_time_ = std::numeric_limits<double>::infinity();
//   type_ = ObjectType::COUPE;

//   if(use_v_ego){
//     // auto speed = perception_obstacle.velocity;
//     perception_speed_ = std::hypot(v_ego + vision_car_obstacle.d3_vx,
//     vision_car_obstacle.d3_vy);
//     // todo: add priority for obstacle
//     speed_yaw_relative_ego_ = std::atan2(vision_car_obstacle.d3_vy, v_ego +
//     vision_car_obstacle.d3_vx);
//   }
//   else{
//     // auto speed = perception_obstacle.velocity;
//     perception_speed_ = std::hypot(vision_car_obstacle.d3_vx,
//     vision_car_obstacle.d3_vy);
//     // todo: add priority for obstacle
//     speed_yaw_relative_ego_ = std::atan2(vision_car_obstacle.d3_vy,
//     vision_car_obstacle.d3_vx);
//   }

//   std::vector<planning_math::Vec2d> polygon_points;
//   // todo: add polygon with more than 2 points interface

//   double cosego = cos(ego_pose.theta);
//   double sinego = sin(ego_pose.theta);
//   for (auto &corner : vision_car_obstacle.corners){
//     polygon_points.emplace_back(planning_math::Vec2d(ego_pose.x + corner.x *
//     cosego - corner.y * sinego,
//                                     ego_pose.y +  corner.x * sinego +
//                                     corner.y * cosego));
//   }
//   planning_math::Polygon2d::ComputeConvexHull(polygon_points,
//                                                      &perception_polygon_);

//   is_virtual_ = id_ < 0;
//   prob_ = 1.0;
//   speed_ = perception_speed_;
//   speed_yaw_ = speed_yaw_relative_ego_ + ego_pose.theta;
//   is_confident_ = vision_car_obstacle.is_confident;
//   const auto& box = perception_bounding_box_;
//   box_base_center_ = planning_math::Vec2d(box.center_x() - box.cos_heading()
//   * box.half_length(),
//                                           box.center_y() - box.sin_heading()
//                                           * box.half_length());
//   box_theta_  = box.heading();
//   box_length_ = box.length();
//   box_width_  = box.width();

//   Pose2D
//   carte_pose_wrt_ego{vision_car_obstacle.d3_x,vision_car_obstacle.d3_y,vision_car_obstacle.theta};
//   Point2D
//   carte_vel_wrt_ego{vision_car_obstacle.d3_vx,vision_car_obstacle.d3_vy};
//   carte_pose_wrt_ego_ = carte_pose_wrt_ego;
//   carte_vel_wrt_ego_  = carte_vel_wrt_ego;

//   double consistent_threshold = 30.0;
//   double angle_difference = std::abs(box_theta_-speed_yaw_)/PI*180;
//   // if(std::abs(speed_) > 1e-3){
//   //   is_angle_consistent_ =
//   !(std::abs(std::abs(std::fmod(angle_difference,180))-90)<consistent_threshold);
//   //consistent when the angle difference is smaller than 90-30 degree
//   // }

// }

// Obstacle::Obstacle(int id, const VisionHuman &vision_human_obstacle,
//                     bool is_static, const double v_ego, const Pose2D
//                     &ego_pose, bool use_v_ego)
//     : id_(id), perception_id_(vision_human_obstacle.track_id),
//     is_static_(is_static),
//       perception_bounding_box_(
//       {ego_pose.x + vision_human_obstacle.d3_x * cos(ego_pose.theta) -
//       vision_human_obstacle.d3_yl * sin(ego_pose.theta) +
//       std::cos(ego_pose.theta) *
//       VehicleParam::Instance()->front_edge_to_center, ego_pose.y +
//       vision_human_obstacle.d3_x * sin(ego_pose.theta) +
//       vision_human_obstacle.d3_yl * cos(ego_pose.theta) +
//       std::sin(ego_pose.theta) *
//       VehicleParam::Instance()->front_edge_to_center}, ego_pose.theta, 0.3,
//       0.3){
//   is_caution_level_obstacle_ = true;
//   is_truncated_ = false;
//   is_st_boundary_constructed_ = false;
//   is_zoomed_ = false;
//   truncated_time_ = std::numeric_limits<double>::infinity();
//   type_ = ObjectType::PEDESTRIAN;

//   // auto speed = perception_obstacle.velocity;
//   if(use_v_ego){
//     perception_speed_ = std::hypot(v_ego + vision_human_obstacle.d3_vx,
//     vision_human_obstacle.d3_vy); speed_yaw_relative_ego_ =
//     std::atan2(vision_human_obstacle.d3_vy, v_ego +
//     vision_human_obstacle.d3_vx);
//   }
//   else{
//     perception_speed_ = std::hypot(vision_human_obstacle.d3_vx,
//     vision_human_obstacle.d3_vy); speed_yaw_relative_ego_ =
//     std::atan2(vision_human_obstacle.d3_vy, vision_human_obstacle.d3_vx);
//   }
//   // todo: add priority for obstacle

//   std::vector<planning_math::Vec2d> polygon_points;
//   // todo: add polygon with more than 2 points interface
//   perception_bounding_box_.GetAllCorners(&polygon_points);
//   mph_assert(planning_math::Polygon2d::ComputeConvexHull(polygon_points,
//                                                      &perception_polygon_));
//   is_virtual_ = id_ < 0;
//   prob_ = 1.0;
//   speed_ = perception_speed_;
//   speed_yaw_ = speed_yaw_relative_ego_ + ego_pose.theta;
//   Pose2D
//   carte_pose_wrt_ego{vision_human_obstacle.d3_x,vision_human_obstacle.d3_yl,0.0};
//   Point2D
//   carte_vel_wrt_ego{vision_human_obstacle.d3_vx,vision_human_obstacle.d3_vy};
//   carte_pose_wrt_ego_ = carte_pose_wrt_ego;
//   carte_vel_wrt_ego_  = carte_vel_wrt_ego;
// }

// }

Obstacle::Obstacle(int id, const ObstacleItem &map_polygon_obstacle)
    : id_(id), perception_id_(id), is_static_(true) {
  is_caution_level_obstacle_ = true;
  is_truncated_ = false;
  is_st_boundary_constructed_ = false;
  is_zoomed_ = false;
  truncated_time_ = std::numeric_limits<double>::infinity();
  type_ = ObjectType::GATE;

  // auto speed = perception_obstacle.velocity;
  perception_speed_ = 0.0;
  speed_yaw_relative_ego_ = 0.0;
  // todo: add priority for obstacle

  // construct polygon
  std::vector<planning_math::Vec2d> points;
  for (const auto &pt : map_polygon_obstacle.pts) {
    points.emplace_back(planning_math::Vec2d(pt.x, pt.y));
  }
  perception_polygon_.set_points(points);
  perception_bounding_box_ = perception_polygon_.MinAreaBoundingBox();
  // do not care
  // mph_assert(planning_math::Polygon2d::ComputeConvexHull(points,
  //                                                    &perception_polygon_));
  is_virtual_ = id_ < 0;
  prob_ = 1.0;
  speed_ = perception_speed_;
  speed_yaw_ = 0.0;
}

Obstacle::Obstacle(int id, const Point3D &point1, const Point3D &point2)
    : id_(id), perception_id_(id), is_static_(true),
      perception_line_(planning_math::Vec2d{point1.x, point1.y},
                       planning_math::Vec2d{point2.x, point2.y}) {

  is_caution_level_obstacle_ = true;
  is_truncated_ = false;
  is_st_boundary_constructed_ = false;
  is_zoomed_ = false;
  truncated_time_ = std::numeric_limits<double>::infinity();
  type_ = ObjectType::NOT_KNOW;

  perception_speed_ = 0.0;
  // todo: add priority for obstacle

  is_virtual_ = id_ < 0;
  prob_ = 1.0;
  speed_ = 0.0;
}

void Obstacle::update(const Obstacle &obstacle) {
  id_ = obstacle.id_;
  perception_id_ = obstacle.perception_id_;

  is_static_ = obstacle.is_static_; // -1(default, unknown), 0(false), 1(true)

  point_ = obstacle.point_;

  is_lon_static_wrt_ego_ = obstacle.is_lon_static_wrt_ego_;
  is_lat_static_wrt_ego_ = obstacle.is_lat_static_wrt_ego_;
  is_lon_highspeed_wrt_ego_ = obstacle.is_lon_highspeed_wrt_ego_;
  is_lon_opposite_wrt_ego_ = obstacle.is_lon_opposite_wrt_ego_;

  is_lon_static_wrt_frenet_ = obstacle.is_lon_static_wrt_frenet_;
  is_lat_static_wrt_frenet_ = obstacle.is_lat_static_wrt_frenet_;
  is_lon_highspeed_wrt_frenet_ = obstacle.is_lon_highspeed_wrt_frenet_;
  is_lon_opposite_wrt_frenet_ = obstacle.is_lon_opposite_wrt_frenet_;

  is_virtual_ = obstacle.is_virtual_;
  is_freemove_ = obstacle.is_freemove_;

  is_beside_road_ = obstacle.is_beside_road_;

  is_in_road_ = obstacle.is_in_road_;
  is_across_road_border_ = obstacle.is_across_road_border_;
  is_in_bend_ = obstacle.is_in_bend_;

  is_in_road_loose_ = obstacle.is_in_road_loose_;

  // int relative_obstacle_id_ = obstacle.-1;

  cutin_score_ = obstacle.cutin_score_;
  is_traj_sigma_valid_ = obstacle.is_traj_sigma_valid_;
  intention_ = obstacle.intention_;
  // std::string prediction_source_;
  speed_ = obstacle.speed_;
  speed_yaw_ = obstacle.speed_yaw_;
  speed_yaw_relative_frenet_ = obstacle.speed_yaw_relative_frenet_;
  speed_yaw_relative_planning_frenet_ =
      obstacle.speed_yaw_relative_planning_frenet_;
  speed_yaw_relative_ego_ = obstacle.speed_yaw_relative_ego_;
  is_confident_ = obstacle.is_confident_;
  is_angle_consistent_ = obstacle.is_angle_consistent_;
  is_along_frenet_ = obstacle.is_along_frenet_;
  pose_direction_ = obstacle.pose_direction_;
  is_approaching_gate_ = obstacle.is_approaching_gate_;
  is_towards_road_center_ =
      obstacle.is_towards_road_center_; // check if obs is parking
                                        // out(especially for vectical parking
                                        // lot)
  is_towards_road_side_ =
      obstacle.is_towards_road_side_; // check if obs is parking in
  is_apa_ = obstacle.is_apa_;
  is_apoa_ = obstacle.is_apoa_;
  is_pullover_ = obstacle.is_pullover_;
  is_highspeed_decel_ = obstacle.is_highspeed_decel_;
  is_beside_intersation_ = obstacle.is_beside_intersation_;
  dist_across_road_boader_ = obstacle.dist_across_road_boader_;
  is_in_middle_road_ = obstacle.is_in_middle_road_;
  // road_side 1 right 2 left 3 unknown
  road_side_ = obstacle.road_side_;
  // road_type 1 single 2 carriage way 3 mix
  road_type_ = obstacle.road_type_;
  acc_ = obstacle.acc_;
  prob_ = obstacle.prob_;
  gear_ = obstacle.gear_;
  type_ = obstacle.type_;
  sf_type_ = obstacle.sf_type_;
  is_sf_static_ = obstacle.is_sf_static_;
  line_fusion_type_ = obstacle.line_fusion_type_;

  trajectory_ = obstacle.trajectory_;
  perception_obstacle_ = obstacle.perception_obstacle_;
  perception_bounding_box_ = obstacle.perception_bounding_box_;
  perception_bounding_box_virtual_ = obstacle.perception_bounding_box_virtual_;
  perception_polygon_ = obstacle.perception_polygon_;
  fillet_cutting_polygon_ = obstacle.fillet_cutting_polygon_;
  fillet_cutting_length_ = obstacle.fillet_cutting_length_;
  is_need_fillet_cutting_ = obstacle.is_need_fillet_cutting_;
  perception_line_ = obstacle.perception_line_;
  perception_points_ = obstacle.perception_points_;
  perception_speed_ = obstacle.perception_speed_;
  perception_speed_yaw_ = obstacle.perception_speed_yaw_;
  perception_bounding_box_extend_ = obstacle.perception_bounding_box_extend_;

  sl_boundary_ = obstacle.sl_boundary_;
  sl_boundary_origin_ = obstacle.sl_boundary_origin_;
  sl_boundary_planning_ = obstacle.sl_boundary_planning_;
  sl_boundary_extend_ = obstacle.sl_boundary_extend_;
  sl_boundary_virtual_ = obstacle.sl_boundary_virtual_;

  perception_sl_polygon_ = obstacle.perception_sl_polygon_;
  precise_boundary_interp_ = obstacle.precise_boundary_interp_;
  using_virtual_box_ = obstacle.using_virtual_box_;
  s_max_index = obstacle.s_max_index;
  s_min_index = obstacle.s_min_index;

  r_frenet_ = obstacle.r_frenet_;
  yaw_relative_frenet_ = obstacle.yaw_relative_frenet_;
  r_planning_frenet_ = obstacle.r_planning_frenet_;
  yaw_relative_planning_frenet_ = obstacle.yaw_relative_planning_frenet_;
  s_min = obstacle.s_min;
  s_max = obstacle.s_max;

  left_space_lane_ = obstacle.left_space_lane_;
  right_space_lane_ = obstacle.right_space_lane_;
  left_space_border_ = obstacle.left_space_border_;
  right_space_border_ = obstacle.right_space_border_;

  // STBoundary reference_line_st_boundary_;
  // STBoundary path_st_boundary_;

  decisions_ = obstacle.decisions_;
  decider_tags_ = obstacle.decider_tags_;
  lateral_decision_ = obstacle.lateral_decision_;
  longitudinal_decision_ = obstacle.longitudinal_decision_;

  box_theta_ = obstacle.box_theta_;
  box_length_ = obstacle.box_length_;
  box_width_ = obstacle.box_width_;
  box_base_center_ = obstacle.box_base_center_;
  box_base_center_sl_ = obstacle.box_base_center_sl_;
  carte_pose_wrt_ego_ = obstacle.carte_pose_wrt_ego_;
  carte_vel_wrt_ego_ = obstacle.carte_vel_wrt_ego_;
  pseudo_predicition_traj_ = obstacle.pseudo_predicition_traj_;

  sub_box_ = obstacle.sub_box_;

  // for keep_clear usage only
  is_blocking_obstacle_ = obstacle.is_blocking_obstacle_;

  is_lane_blocking_ = obstacle.is_lane_blocking_;

  is_lane_change_blocking_ = obstacle.is_lane_change_blocking_;

  is_caution_level_obstacle_ = obstacle.is_caution_level_obstacle_;

  min_radius_stop_distance_ = obstacle.min_radius_stop_distance_;

  // indicate whether the st boundary has already been constructed and
  // up-to-date
  is_st_boundary_constructed_ = obstacle.is_st_boundary_constructed_;

  // info about revision of prediction trajectory
  is_truncated_ = obstacle.is_truncated_;
  truncated_time_ = obstacle.truncated_time_;

  // info about st graph zoom manipulation
  is_zoomed_ = obstacle.is_zoomed_;
}

void Obstacle::SetPerceptionSlBoundary(const SLBoundary &sl_boundary) {
  sl_boundary_ = sl_boundary;
}

void Obstacle::SetPerceptionSlBoundaryOrigin(
    const SLBoundary &sl_boundary_origin) {
  sl_boundary_origin_ = sl_boundary_origin;
}

void Obstacle::SetPerceptionSlBoundaryPlanning(
    const SLBoundary &sl_boundary_planning) {
  sl_boundary_planning_ = sl_boundary_planning;
}

void Obstacle::SetPerceptionSlPolygon(
    const planning_math::Polygon2d &perception_polygon) {
  perception_sl_polygon_ = perception_polygon;
}

void Obstacle::SetPerceptionSlBoundaryVirtual(
    const SLBoundary &sl_boundary_virtual) {
  sl_boundary_virtual_ = sl_boundary_virtual;
}

void Obstacle::ComputeSlBoundary(
    std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
    SLBoundary *const sl_boundary,
    // planning_math::Box2d *const box,
    const bool planning_flag) {
  double obs_start_s(std::numeric_limits<double>::max());
  double obs_end_s(std::numeric_limits<double>::lowest());
  double obs_start_l(std::numeric_limits<double>::max());
  double obs_end_l(std::numeric_limits<double>::lowest());
  std::vector<planning_math::Vec2d> obstacle_points;

  perception_bounding_box_.GetAllCorners(&obstacle_points);
  for (const planning_math::Vec2d &obs_point : obstacle_points) {
    Point2D frenet_point, carte_point;
    carte_point.x = obs_point.x();
    carte_point.y = obs_point.y();
    if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
        TRANSFORM_FAILED) {
      continue;
    }

    sl_boundary->corners.push_back(
        Point3D(frenet_point.x, frenet_point.y, 0.0));
    obs_start_s = std::min(obs_start_s, frenet_point.x);
    obs_end_s = std::max(obs_end_s, frenet_point.x);
    obs_start_l = std::min(obs_start_l, frenet_point.y);
    obs_end_l = std::max(obs_end_l, frenet_point.y);
  }
  sl_boundary->start_s = obs_start_s;
  sl_boundary->end_s = obs_end_s;
  sl_boundary->start_l = obs_start_l;
  sl_boundary->end_l = obs_end_l;

  // sort slboundary corners
  std::sort(
      sl_boundary->corners.begin(), sl_boundary->corners.end(),
      [](const Point3D &lhs, const Point3D &rhs) { return lhs.x < rhs.x; });

  for (double s = obs_start_s; s < obs_end_s; s += 0.5) {
    // std::cout << "obs: " << Id() << " " << s << " " <<
    // frenet_coord->GetRefCurveCurvature(s) << std::endl;
    if (std::abs(frenet_coord->GetRefCurveCurvature(s)) > 0.1 &&
        !planning_flag) {
      is_in_bend_ = true;
    }
  }

  // if (box != nullptr) {
  //   return ;
  // }

  Point2D frenet_point, carte_point;
  carte_point.x = perception_bounding_box_.center_x();
  carte_point.y = perception_bounding_box_.center_y();
  if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
      TRANSFORM_FAILED) {
    if (planning_flag) {
      r_planning_frenet_ = 100.0;
      yaw_relative_planning_frenet_ = 0.0;
      speed_yaw_relative_planning_frenet_ = speed_yaw_relative_ego_;
    } else {
      r_frenet_ = 100.0;
      yaw_relative_frenet_ = 0.0;
      speed_yaw_relative_frenet_ = speed_yaw_relative_ego_;
    }
    return;
  } else {
    if (planning_flag) {
      r_planning_frenet_ = frenet_point.y;
    } else {
      r_frenet_ = frenet_point.y;
    }
  }
  double s_frenet = frenet_point.x;
  double curve_heading = frenet_coord->GetRefCurveHeading(s_frenet);
  if (planning_flag) {
    yaw_relative_planning_frenet_ =
        perception_bounding_box_.heading() - curve_heading;
    speed_yaw_relative_planning_frenet_ = speed_yaw_ - curve_heading;
    speed_yaw_relative_planning_frenet_ =
        planning_math::NormalizeAngle(speed_yaw_relative_planning_frenet_);
  } else {
    yaw_relative_frenet_ = perception_bounding_box_.heading() - curve_heading;
    speed_yaw_relative_frenet_ = speed_yaw_ - curve_heading;
    speed_yaw_relative_frenet_ =
        planning_math::NormalizeAngle(speed_yaw_relative_frenet_);
  }
  return;
}

void Obstacle::ComputeSlBoundary(
    std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
    const planning_math::Box2d &box, SLBoundary *const sl_boundary) {

  double obs_start_s(std::numeric_limits<double>::max());
  double obs_end_s(std::numeric_limits<double>::lowest());
  double obs_start_l(std::numeric_limits<double>::max());
  double obs_end_l(std::numeric_limits<double>::lowest());
  std::vector<planning_math::Vec2d> obstacle_points;

  box.GetAllCorners(&obstacle_points);
  for (const planning_math::Vec2d &obs_point : obstacle_points) {
    Point2D frenet_point, carte_point;
    carte_point.x = obs_point.x();
    carte_point.y = obs_point.y();
    if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
        TRANSFORM_FAILED) {
      continue;
    }

    sl_boundary->corners.push_back(
        Point3D(frenet_point.x, frenet_point.y, 0.0));
    obs_start_s = std::min(obs_start_s, frenet_point.x);
    obs_end_s = std::max(obs_end_s, frenet_point.x);
    obs_start_l = std::min(obs_start_l, frenet_point.y);
    obs_end_l = std::max(obs_end_l, frenet_point.y);
  }
  sl_boundary->start_s = obs_start_s;
  sl_boundary->end_s = obs_end_s;
  sl_boundary->start_l = obs_start_l;
  sl_boundary->end_l = obs_end_l;

  return;
}

void Obstacle::ComputePointsSlBoundary(
    std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
    SLBoundary *const sl_boundary) {
  double obs_start_s(std::numeric_limits<double>::max());
  double obs_end_s(std::numeric_limits<double>::lowest());
  double obs_start_l(std::numeric_limits<double>::max());
  double obs_end_l(std::numeric_limits<double>::lowest());

  for (const planning_math::Vec2d &obs_point : perception_points_) {
    Point2D frenet_point, carte_point;
    carte_point.x = obs_point.x();
    carte_point.y = obs_point.y();
    if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
        TRANSFORM_FAILED) {
      continue;
    }

    sl_boundary->corners.push_back(
        Point3D(frenet_point.x, frenet_point.y, 0.0));
    obs_start_s = std::min(obs_start_s, frenet_point.x);
    obs_end_s = std::max(obs_end_s, frenet_point.x);
    obs_start_l = std::min(obs_start_l, frenet_point.y);
    obs_end_l = std::max(obs_end_l, frenet_point.y);
  }

  // sort slboundary corners
  std::sort(
      sl_boundary->corners.begin(), sl_boundary->corners.end(),
      [](const Point3D &lhs, const Point3D &rhs) { return lhs.x < rhs.x; });

  sl_boundary->start_s = obs_start_s;
  sl_boundary->end_s = obs_end_s;
  sl_boundary->start_l = obs_start_l;
  sl_boundary->end_l = obs_end_l;
  return;
}

void Obstacle::ComputePolygon2dSlBoundary(
    std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
    planning_math::Polygon2d *const perception_polygon,
    planning_math::Box2d *const box, double precise) {

  std::vector<planning_math::Vec2d> obstacle_points;
  std::vector<planning_math::Vec2d> pts;
  std::vector<planning_math::Vec2d> pts_sl;
  planning_math::Box2d *box_ptr;
  if (box == nullptr) {
    box_ptr = &perception_bounding_box_;
    perception_bounding_box_.GetAllCorners(&obstacle_points);
  } else {
    box_ptr = box;
    box->GetAllCorners(&obstacle_points);
  }
  // for(size_t i = 0; i < obstacle_points.size(); i++)
  // std::cout << "index: " << i << obstacle_points[i].x() << "  " <<
  // obstacle_points[i].y() << std::endl;

  // std::cout << box_ptr->width() << "  " <<  box_ptr->length() << std::endl;

  double theta = box_ptr->heading();
  double st = sin(theta);
  double ct = cos(theta);
  const double measure[4] = {box_ptr->width(), box_ptr->length(),
                             box_ptr->width(), box_ptr->length()};
  const double st_array[4] = {ct, -st, -ct, st};
  const double ct_array[4] = {-st, -ct, st, ct};
  double perimeter = box_ptr->width() * 2.0 + box_ptr->length() * 2.0;
  int count = std::ceil(perimeter / precise);
  double interp_step = perimeter / count;
  // interplitation around boundary
  // int obstacle_index = 0;
  int base_index = 0;
  int base_index2 = 0;
  // std::cout << "count " << count << std::endl;
  for (int i = 0; i < count; i++) {
    double x = obstacle_points[base_index].x() +
               ct_array[base_index] * interp_step * (i - base_index2);
    double y = obstacle_points[base_index].y() +
               st_array[base_index] * interp_step * (i - base_index2);
    planning_math::Vec2d pt(x, y);
    pts.push_back(pt);
    if ((pt.DistanceTo(obstacle_points[base_index]) + interp_step) >
        measure[base_index]) {
      if (base_index >= 3)
        break;
      pts.push_back(obstacle_points[++base_index]);
      base_index2 = i;
    }
  }
  double s_max = -1e19;
  double s_min = 1e19;
  int index = 0;
  double x_sum = 0.0;
  double y_sum = 0.0;
  for (const planning_math::Vec2d &pt : pts) {
    Point2D frenet_point, carte_point;
    carte_point.x = pt.x();
    carte_point.y = pt.y();
    if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
        TRANSFORM_FAILED) {
      continue;
    }
    // std::cout << "polygon: " << index << " " << carte_point.x
    //           << "  " << carte_point.y
    //           << "  " << frenet_point.x
    //           << "  " << frenet_point.y << std::endl;
    pts_sl.push_back(planning_math::Vec2d(frenet_point.x, frenet_point.y));
    if (frenet_point.x > s_max) {
      s_max = frenet_point.x;
      s_max_index = index;
    }
    if (frenet_point.x < s_min) {
      s_min = frenet_point.x;
      s_min_index = index;
    }
    index++;
    x_sum += carte_point.x;
    y_sum += carte_point.y;
  }
  Point2D frenet_point, carte_point;
  carte_point.x = x_sum / pts_sl.size();
  carte_point.y = y_sum / pts_sl.size();
  if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
      TRANSFORM_FAILED) {
    return;
  }
  // std::cout << "center polygon: " << carte_point.x << "  " << carte_point.y
  // << "  " << frenet_point.x << "  " << frenet_point.y << std::endl; std::cout
  // << "index " << s_min_index << " " <<  s_max_index << std::endl;
  if (pts_sl.size() > 3) {
    box_base_center_sl_.set_x(frenet_point.x);
    box_base_center_sl_.set_y(frenet_point.y);
    perception_polygon->set_points(pts_sl);
  }
}

double Obstacle::InterpBounds(double s, std::string dir) const {
  // get poinsts
  // int index_mid = (s_max_index + s_min_index) / 2.0;
  const std::vector<planning_math::Vec2d> &pts =
      perception_sl_polygon_.points();
  double l_min = 1e19;
  double l_max = -1e19;
  if (s <= sl_boundary_.start_s)
    return pts[s_min_index].y();
  if (s >= sl_boundary_.end_s)
    return pts[s_max_index].y();
  for (int i = 0; i < (int)perception_sl_polygon_.points().size() - 1; i++) {
    if ((perception_sl_polygon_.points()[i].x() > s &&
         perception_sl_polygon_.points()[i + 1].x() < s) ||
        (perception_sl_polygon_.points()[i].x() < s &&
         perception_sl_polygon_.points()[i + 1].x() > s)) {
      double ll = pts[i].y() + (s - pts[i].x()) /
                                   (pts[i + 1].x() - pts[i].x()) *
                                   (pts[i + 1].y() - pts[i].y());
      if (dir == "right") {
        if (ll < l_min)
          l_min = ll;
      } else if (dir == "left") {
        if (ll > l_max)
          l_max = ll;
      }
    }
  }
  if (dir == "right")
    return l_min;
  if (dir == "left")
    return l_max;
  return 0.0;
}

// void Obstacle::ComputeSlPoint(
//     std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
//     SLBoundary *const sl_boundary,
//     const bool planning_flag) {
//   double obs_start_s(std::numeric_limits<double>::max());
//   double obs_end_s(std::numeric_limits<double>::lowest());
//   double obs_start_l(std::numeric_limits<double>::max());
//   double obs_end_l(std::numeric_limits<double>::lowest());
//   std::vector<planning_math::Vec2d> obstacle_points;
//   // const auto& first_traj_point = trajectory_[0];
//   // const auto& first_point = first_traj_point.path_point;
//   // // todo: get info all from Fusion msg
//   // planning_math::Vec2d center(first_point.x, first_point.y);
//   // planning_math::Box2d object_moving_box(center, first_point.theta,
//   //                                         perception_obstacle_.length,
//   //                                         perception_obstacle_.width);
//   // object_moving_box.GetAllCorners(&obstacle_points);
//   Point2D frenet_point, carte_point;
//   carte_point.x = perception_bounding_box_.center_x();
//   carte_point.y = perception_bounding_box_.center_y();
//   if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
//       TRANSFORM_FAILED) {
//       if (planning_flag) {
//         r_planning_frenet_ = 100.0;
//       }
//       else{
//         r_frenet_ = 100.0;
//       }
//   }
//   else {
//       obs_start_s = std::min(obs_start_s, frenet_point.x);
//       obs_end_s = std::max(obs_end_s, frenet_point.x);
//       obs_start_l = std::min(obs_start_l, frenet_point.y);
//       obs_end_l = std::max(obs_end_l, frenet_point.y);
//       if (planning_flag) {
//         r_planning_frenet_ = frenet_point.y;
//       }
//       else{
//         r_frenet_ = frenet_point.y;
//       }
//   }
//   sl_boundary->start_s = obs_start_s;
//   sl_boundary->end_s = obs_end_s;
//   sl_boundary->start_l = obs_start_l;
//   sl_boundary->end_l = obs_end_l;
//   double s_frenet = frenet_point.x;
//   double curve_heading = frenet_coord->GetRefCurveHeading(s_frenet);
//   if (planning_flag) {
//     yaw_relative_planning_frenet_ = perception_obstacle_.heading_yaw -
//     curve_heading;
//   }
//   else{
//     yaw_relative_frenet_ = perception_obstacle_.heading_yaw - curve_heading;
//   }
//   return;
// }

void Obstacle::ComputeSlLine(
    std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
    SLBoundary *const sl_boundary, const bool planning_flag) {
  double obs_start_s(std::numeric_limits<double>::max());
  double obs_end_s(std::numeric_limits<double>::lowest());
  double obs_start_l(std::numeric_limits<double>::max());
  double obs_end_l(std::numeric_limits<double>::lowest());
  std::vector<planning_math::Vec2d> obstacle_points;
  // const auto& first_traj_point = trajectory_[0];
  // const auto& first_point = first_traj_point.path_point;
  // // todo: get info all from Fusion msg
  // planning_math::Vec2d center(first_point.x, first_point.y);
  // planning_math::Box2d object_moving_box(center, first_point.theta,
  //                                         perception_obstacle_.length,
  //                                         perception_obstacle_.width);
  // object_moving_box.GetAllCorners(&obstacle_points);
  // center
  Point2D frenet_point, carte_point;
  carte_point.x = perception_line_.center().x();
  carte_point.y = perception_line_.center().y();
  if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
      TRANSFORM_FAILED) {
    if (planning_flag) {
      // double r_planning_frenet_ = 100.0;
    } else {
      // double r_frenet_ = 100.0;
    }
  } else {
    // double r_frenet_ = frenet_point.y;
  }
  double s_frenet = frenet_point.x;
  // start
  carte_point.x = perception_line_.start().x();
  carte_point.y = perception_line_.start().y();
  if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
      TRANSFORM_FAILED) {
    // std::cout << "Line Start Transfrom Failed! " << std::endl;
  }
  obs_start_s = std::min(obs_start_s, frenet_point.x);
  obs_start_l = std::min(obs_start_l, frenet_point.y);
  // end
  carte_point.x = perception_line_.end().x();
  carte_point.y = perception_line_.end().y();
  if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
      TRANSFORM_FAILED) {
    // std::cout << "Line End Transfrom Failed! " << std::endl;
  }
  obs_end_s = std::max(obs_end_s, frenet_point.x);
  obs_end_l = std::max(obs_end_l, frenet_point.y);

  sl_boundary->start_s = obs_start_s;
  sl_boundary->end_s = obs_end_s;
  sl_boundary->start_l = obs_start_l;
  sl_boundary->end_l = obs_end_l;
  double curve_heading = frenet_coord->GetRefCurveHeading(s_frenet);
  if (planning_flag) {
    yaw_relative_planning_frenet_ =
        perception_obstacle_.heading_yaw - curve_heading;
  } else {
    yaw_relative_frenet_ = perception_obstacle_.heading_yaw - curve_heading;
  }
  return;
}

void Obstacle::ComputeSlLine(
    std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
    const planning_math::LineSegment2d &line, const Pose2D &ego_pose,
    SLBoundary *const sl_boundary) {
  double obs_start_s(std::numeric_limits<double>::max());
  double obs_end_s(std::numeric_limits<double>::lowest());
  double obs_start_l(std::numeric_limits<double>::max());
  double obs_end_l(std::numeric_limits<double>::lowest());

  planning_math::LineSegment2d perception_line_(
      planning_math::Vec2d{
          ego_pose.x + line.start().x() * cos(ego_pose.theta) -
              line.start().y() * sin(ego_pose.theta) +
              cos(ego_pose.theta) *
                  VehicleParam::Instance()->front_edge_to_center,
          ego_pose.y + line.start().x() * sin(ego_pose.theta) +
              line.start().y() * cos(ego_pose.theta) +
              sin(ego_pose.theta) *
                  VehicleParam::Instance()->front_edge_to_center},
      planning_math::Vec2d{
          ego_pose.x + line.end().x() * cos(ego_pose.theta) -
              line.end().y() * sin(ego_pose.theta) +
              cos(ego_pose.theta) *
                  VehicleParam::Instance()->front_edge_to_center,
          ego_pose.y + line.end().x() * sin(ego_pose.theta) +
              line.end().y() * cos(ego_pose.theta) +
              sin(ego_pose.theta) *
                  VehicleParam::Instance()->front_edge_to_center});

  Point2D frenet_point, carte_point;

  // start
  carte_point.x = perception_line_.start().x();
  carte_point.y = perception_line_.start().y();
  if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
      TRANSFORM_FAILED) {
  } else {
  }
  obs_start_s = std::min(obs_start_s, frenet_point.x);
  obs_end_s = std::max(obs_end_s, frenet_point.x);
  obs_start_l = std::min(obs_start_l, frenet_point.y);
  obs_end_l = std::max(obs_end_l, frenet_point.y);

  // end
  carte_point.x = perception_line_.end().x();
  carte_point.y = perception_line_.end().y();
  if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
      TRANSFORM_FAILED) {
  } else {
  }
  obs_start_s = std::min(obs_start_s, frenet_point.x);
  obs_end_s = std::max(obs_end_s, frenet_point.x);
  obs_start_l = std::min(obs_start_l, frenet_point.y);
  obs_end_l = std::max(obs_end_l, frenet_point.y);

  sl_boundary->start_s = obs_start_s;
  sl_boundary->end_s = obs_end_s;
  sl_boundary->start_l = obs_start_l;
  sl_boundary->end_l = obs_end_l;
  return;
}

void Obstacle::extendboundingbox(
    const double theta, const double longtitude, const double latitude,
    const std::shared_ptr<FrenetCoordinateSystem> frenet_coord) {
  double shift_length = std::hypot(longtitude, latitude) / 2;
  planning_math::Vec2d shift_vec(cos(theta) * shift_length,
                                 sin(theta) * shift_length);
  perception_bounding_box_extend_ = perception_bounding_box_;
  perception_bounding_box_extend_.Shift(shift_vec);
  perception_bounding_box_extend_.LongitudinalExtend(
      shift_length * std::abs(cos(theta - perception_bounding_box_.heading())));
  perception_bounding_box_extend_.LateralExtend(
      shift_length * std::abs(sin(theta - perception_bounding_box_.heading())));
  auto tmp = perception_bounding_box_;
  perception_bounding_box_ = perception_bounding_box_extend_;
  // std::cout << "extend: " << id_ << " " << longtitude << "  " << shift_length
  // << " " << theta << std::endl;
  ComputeSlBoundary(frenet_coord, &sl_boundary_extend_);
  perception_bounding_box_ = tmp;
}

const std::vector<std::string> &Obstacle::decider_tags() const {
  return decider_tags_;
}

const std::vector<ObjectDecisionType> &Obstacle::decisions() const {
  return decisions_;
}

bool Obstacle::IsLateralDecision(const ObjectDecisionType &decision) {
  return decision.has_ignore() || decision.has_nudge();
}

bool Obstacle::IsParkingLateralDecision(const ObjectDecisionType &decision) {
  return decision.has_ignore() || decision.has_sidepass();
}

bool Obstacle::IsLongitudinalDecision(const ObjectDecisionType &decision) {
  return decision.has_ignore() || decision.has_stop() || decision.has_yield() ||
         decision.has_follow() || decision.has_overtake() ||
         decision.has_other_cars_yield();
}

bool Obstacle::IsParkingLongitudinalDecision(
    const ObjectDecisionType &decision) {
  return decision.has_lead() || decision.has_other_cars_yield();
}

const ObjectDecisionType &Obstacle::LongitudinalDecision() const {
  return longitudinal_decision_;
}

const ObjectDecisionType &Obstacle::LateralDecision() const {
  return lateral_decision_;
}

const ObjectDecisionType &Obstacle::ParkingLateralDecision() const {
  return lateral_decision_;
}

ObjectDecisionType
Obstacle::MergeLongitudinalDecision(const ObjectDecisionType &lhs,
                                    const ObjectDecisionType &rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {
    return rhs;
  } else if (lhs_val > rhs_val) {
    return lhs;
  } else {
    if (lhs.has_ignore()) {
      return rhs;
    } else if (lhs.has_stop()) {
      return lhs.stop().distance_s < rhs.stop().distance_s ? lhs : rhs;
    } else if (lhs.has_yield()) {
      return lhs.yield().distance_s < rhs.yield().distance_s ? lhs : rhs;
    } else if (lhs.has_follow()) {
      return lhs.follow().distance_s < rhs.follow().distance_s ? lhs : rhs;
    } else if (lhs.has_overtake()) {
      return lhs.overtake().distance_s > rhs.overtake().distance_s ? lhs : rhs;
    } else {
      // std::cout << "Unknown decision" << std::endl;
    }
  }
  return lhs; // stop compiler complaining
}

ObjectDecisionType
Obstacle::MergeParkingLongitudinalDecision(const ObjectDecisionType &lhs,
                                           const ObjectDecisionType &rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  return lhs; // stop compiler complaining
}

bool Obstacle::IsIgnore() const {
  return IsLongitudinalIgnore() && IsLateralIgnore();
}

bool Obstacle::IsLongitudinalIgnore() const {
  return longitudinal_decision_.has_ignore();
}

bool Obstacle::IsLateralIgnore() const {
  return lateral_decision_.has_ignore();
}

ObjectDecisionType
Obstacle::MergeLateralDecision(const ObjectDecisionType &lhs,
                               const ObjectDecisionType &rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {
    return rhs;
  } else if (lhs_val > rhs_val) {
    return lhs;
  } else {
    if (lhs.has_ignore()) {
      return rhs;
    } else if (lhs.has_nudge()) {
      mph_assert(lhs.nudge().type == rhs.nudge().type);
      if (!rhs.nudge().is_longitunidal_ignored) {
        return rhs;
      } else if (!lhs.nudge().is_longitunidal_ignored) {
        return lhs;
      }
      return std::fabs(lhs.nudge().distance_l) >
                     std::fabs(rhs.nudge().distance_l)
                 ? lhs
                 : rhs;
    }
  }
  // std::cout << "Does not have rule to merge decision: " << std::endl;
  return lhs;
}

ObjectDecisionType
Obstacle::MergeParkingLateralDecision(const ObjectDecisionType &lhs,
                                      const ObjectDecisionType &rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  return rhs;
}

bool Obstacle::HasLateralDecision() const {
  return lateral_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

bool Obstacle::HasParkingLateralDecision() const {
  return lateral_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

bool Obstacle::HasLongitudinalDecision() const {
  return longitudinal_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

bool Obstacle::HasNonIgnoreDecision() const {
  return (HasLateralDecision() && !IsLateralIgnore()) ||
         (HasLongitudinalDecision() && !IsLongitudinalIgnore());
}

SLBoundary Obstacle::PerceptionSLBoundary() const { return sl_boundary_; }
SLBoundary Obstacle::PerceptionSLBoundaryOrigin() const {
  return sl_boundary_origin_;
}
SLBoundary Obstacle::PerceptionSLBoundaryPlanning() const {
  return sl_boundary_planning_;
}
SLBoundary Obstacle::PerceptionSLBoundaryExtend() const {
  return sl_boundary_extend_;
}
SLBoundary Obstacle::PerceptionSLBoundaryVirtual() const {
  return sl_boundary_virtual_;
}

void Obstacle::AddLongitudinalDecision(const std::string &decider_tag,
                                       const ObjectDecisionType &decision) {
  mph_assert(IsLongitudinalDecision(decision));
  longitudinal_decision_ =
      MergeLongitudinalDecision(longitudinal_decision_, decision);
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

void Obstacle::AddParkingLongitudinalDecision(
    const std::string &decider_tag, const ObjectDecisionType &decision) {
  mph_assert(IsParkingLongitudinalDecision(decision));
  longitudinal_decision_ =
      MergeParkingLongitudinalDecision(longitudinal_decision_, decision);
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

void Obstacle::AddLateralDecision(const std::string &decider_tag,
                                  const ObjectDecisionType &decision) {
  mph_assert(IsLateralDecision(decision));
  lateral_decision_ = MergeLateralDecision(lateral_decision_, decision);
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

void Obstacle::AddParkingLateralDecision(const std::string &decider_tag,
                                         const ObjectDecisionType &decision) {
  mph_assert(IsParkingLateralDecision(decision));
  lateral_decision_ = MergeParkingLateralDecision(lateral_decision_, decision);
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

void Obstacle::ClearLongitudinalDecision() {
  ObjectDecisionType none_decision;
  longitudinal_decision_ = none_decision;
}

void Obstacle::ClearLateralDecision() {
  ObjectDecisionType none_decision;
  lateral_decision_ = none_decision;
}

void Obstacle::ClearParkingLateralDecision() {
  ObjectDecisionType none_decision;
  lateral_decision_ = none_decision;
}

void Obstacle::SetLeftSpaceLane(const double left_lane_width_start,
                                const double left_lane_width_end,
                                const SLBoundary &sl_boundary) {
  left_space_lane_ = std::min(left_lane_width_start - sl_boundary.end_l,
                              left_lane_width_end - sl_boundary.end_l);
}

void Obstacle::SetRightSpaceLane(const double right_lane_width_start,
                                 const double right_lane_width_end,
                                 const SLBoundary &sl_boundary) {
  right_space_lane_ = std::min(right_lane_width_start + sl_boundary.start_l,
                               right_lane_width_end + sl_boundary.start_l);
}

void Obstacle::SetLeftSpaceBorder(const double left_road_width_start,
                                  const double left_road_width_width_end,
                                  const SLBoundary &sl_boundary) {
  left_space_border_ = std::min(left_road_width_start - sl_boundary.end_l,
                                left_road_width_width_end - sl_boundary.end_l);
}

void Obstacle::SetRightSpaceBorder(const double right_road_width_start,
                                   const double right_road_width_end,
                                   const SLBoundary &sl_boundary) {
  right_space_border_ = std::min(right_road_width_start + sl_boundary.start_l,
                                 right_road_width_end + sl_boundary.start_l);
}

} // namespace parking

} // namespace msquare
