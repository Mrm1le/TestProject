#include <algorithm>
#include <cassert>
#include <iostream>
#include <utility>

#include "common/config_context.h"
#include "common/math/common_utils.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/obstacle.h"
#include "common/speed/st_boundary.h"
#include "common/trajectory/discretized_trajectory.h"
#include "planning/common/common.h"

namespace msquare {

namespace {
const double kStBoundaryDeltaS = 0.2;       // meters
const double kStBoundarySparseDeltaS = 1.0; // meters
const double kStBoundaryDeltaT = 0.05;      // seconds
const double kSizeEpsilon = 0.001;
} // namespace

// static const int pedestrian_type_id = 10001;

Obstacle::Obstacle(int id,
                   const maf_perception_interface::PerceptionFusionObjectData
                       &perception_obstacle,
                   const bool is_static)
    : id_(id), perception_id_(perception_obstacle.track_id),
      is_static_(is_static), perception_obstacle_(perception_obstacle),
      perception_bounding_box_(
          {perception_obstacle.position.x, perception_obstacle.position.y},
          perception_obstacle.heading_yaw,
          perception_obstacle.shape.length + kSizeEpsilon,
          perception_obstacle.shape.width + kSizeEpsilon) {
  update(id, perception_obstacle, is_static);
}

Obstacle::Obstacle(int id, std::size_t traj_index,
                   const PredictionObject &prediction_object,
                   const bool is_static, double start_timestamp)
    : id_(id), perception_id_(prediction_object.id), is_static_(is_static),
      is_freemove_(prediction_object.b_backup_freemove),
      cutin_score_(prediction_object.cutin_score),
      perception_bounding_box_(
          {prediction_object.position_x, prediction_object.position_y},
          prediction_object.yaw, prediction_object.length,
          prediction_object.width) {
  update(id, traj_index, prediction_object, is_static, start_timestamp);
}

void Obstacle::clear() {
  id_ = 0;
  perception_id_ = 0;
  is_static_ = false;
  is_frenet_invalid_ = false;
  is_pred_frenet_invalid_ = false;
  is_virtual_ = false;
  is_freemove_ = false;
  b_minor_modal_ = false;
  cutin_score_ = 0.0;
  max_sigma_ = 0.0;
  is_traj_sigma_valid_ = false;
  intention_ = "none";
  prediction_source_ = "";
  speed_ = 0.0;
  speed_direction_ = 0.0;
  acc_ = 0.0;
  prob_ = 0;
  type_ = ObjectType::NOT_KNOW;
  is_cutin_ = false;

  trajectory_.clear();
  // maf_perception_interface::PerceptionFusionObjectData perception_obstacle_;
  perception_bounding_box_.clear();
  prediction_bounding_box_.clear();
  perception_polygon_.clear();
  car_ego_polygon_.clear();
  perception_speed_ = 0;

  sl_boundary_.corners.clear();
  pred_sl_boundary_.corners.clear();
  r_frenet_ = 0;
  s_frenet_ = 0;
  yaw_relative_frenet_ = 0;
  s_min = 0;
  s_max = 0;
  pred_sl_boundary_relative_time_ = 0;

  reference_line_st_boundary_.clear();
  path_st_boundary_.clear();

  sl_polygon_seq_.clear();
  // to be delete
  has_sl_polygon_seq_ = false;
  invalid_sl_polygon_seq_ = false;

  decisions_.clear();
  decider_tags_.clear();
  lat_decider_tags_.clear();
  lon_decider_tags_.clear();
  lateral_decision_.clear();
  longitudinal_decision_.clear();

  // for keep_clear usage only
  is_blocking_obstacle_ = false;

  is_lane_blocking_ = false;

  is_lane_change_blocking_ = false;

  is_caution_level_obstacle_ = false;

  min_radius_stop_distance_ = -1.0;

  // avoidance distance buffer
  avd_dis_buffer_ = 0.2;

  // indicate whether the st boundary has already been constructed and
  // up-to-date
  is_st_boundary_constructed_ = false;

  // info about revision of prediction trajectory
  is_truncated_ = false;
  truncated_time_ = std::numeric_limits<double>::infinity();

  // info about st graph zoom manipulation
  is_zoomed_ = false;
}

void Obstacle::update(int id,
                      const maf_perception_interface::PerceptionFusionObjectData
                          &perception_obstacle,
                      const bool is_static) {
  clear();

  id_ = id;
  perception_id_ = perception_obstacle.track_id;
  is_static_ = is_static;
  perception_obstacle_ = perception_obstacle;
  perception_bounding_box_ = planning_math::Box2d(
      {perception_obstacle.position.x, perception_obstacle.position.y},
      perception_obstacle.heading_yaw,
      perception_obstacle.shape.length + kSizeEpsilon,
      perception_obstacle.shape.width + kSizeEpsilon);
  is_caution_level_obstacle_ = true;
  is_truncated_ = false;
  is_st_boundary_constructed_ = false;
  is_zoomed_ = false;
  truncated_time_ = std::numeric_limits<double>::infinity();
  switch (from_msd_fusion_type(perception_obstacle.type_info)) {
  case 10001:
    type_ = ObjectType::PEDESTRIAN;
    break;
  case 10002:
    type_ = ObjectType::OFO;
    break;
  case 20001:
    type_ = ObjectType::CONE_BUCKET;
    break;
  case 0:
    type_ = ObjectType::NOT_KNOW;
    break;
  case 1:
    type_ = ObjectType::COUPE;
    break;
  case 2:
    type_ = ObjectType::TRANSPORT_TRUNK;
    break;
  case 3:
    type_ = ObjectType::BUS;
    break;
  case 4:
    type_ = ObjectType::ENGINEER_TRUCK;
    break;
  case 5:
    type_ = ObjectType::TRICYCLE;
    break;
  case -1:
    type_ = ObjectType::STOP_LINE;
    break;
  default:
    type_ = ObjectType::NOT_KNOW;
    break;
  }
  auto speed = perception_obstacle.velocity;
  perception_speed_ = std::hypot(speed.x, std::hypot(speed.y, speed.z));
  // todo: add priority for obstacle

  std::vector<planning_math::Vec2d> polygon_points;
  if (perception_obstacle.polygon_bottom.points.size() < 3) {
    perception_bounding_box_.GetAllCorners(&polygon_points);
  } else {
    for (const auto &point : perception_obstacle.polygon_bottom.points) {
      polygon_points.emplace_back(planning_math::Vec2d(point.x, point.y));
    }
    polygon_points.erase(polygon_points.begin());
    for (int i = 0;
         i < (int)perception_obstacle.polygon_bottom.points.size() - 1; ++i) {
      auto &point = perception_obstacle.polygon_bottom.points[i];
      polygon_points.emplace_back(planning_math::Vec2d(point.x, point.y));
    }
  }
  // MSD_LOG(INFO, "cone type %d points raw size %d static: %d",
  // perception_obstacle.type, polygon_points.size(), is_static);
  auto extracted_polygon_points = polygon_points;
  ExtractPointAtSpecifiedResolution(extracted_polygon_points);
  if (extracted_polygon_points.size() < 3) {
    extracted_polygon_points = polygon_points;
  }
  // for (auto point : extracted_polygon_points) {
  //   MSD_LOG(INFO, "cone after remove redundant point x %f y %f", point.x(),
  //   point.y()); MSD_LOG(INFO, "cone after remove redundant rel point x %f y
  //   %f", point.x() - perception_obstacle_.position.x,
  //         point.y()-perception_obstacle_.position.y);
  // }
  if (!planning_math::Polygon2d::ComputeConvexHull(extracted_polygon_points,
                                                   &perception_polygon_)) {
    MSD_LOG(INFO, "invalid cone polygon");
  }
  std::vector<planning_math::Vec2d> ego_polygon_points;
  for (const auto &point : perception_polygon_.points()) {
    ego_polygon_points.emplace_back(
        planning_math::Vec2d(point.x() - perception_obstacle.position.x,
                             point.y() - perception_obstacle.position.y));
  }
  if (!planning_math::Polygon2d::ComputeConvexHull(ego_polygon_points,
                                                   &car_ego_polygon_)) {
    MSD_LOG(INFO, "invalid ego cone polygon");
  }

  is_virtual_ = id_ < 0;
  prob_ = 1.0;
  speed_ = perception_speed_;
  acc_ = perception_obstacle.acceleration_relative_to_ground.x;
}

void Obstacle::update(int id, std::size_t traj_index,
                      const PredictionObject &prediction_object,
                      const bool is_static, double start_timestamp) {
  clear();

  id_ = id;
  perception_id_ = prediction_object.id;
  is_static_ = is_static;
  is_freemove_ = prediction_object.b_backup_freemove;
  cutin_score_ = prediction_object.cutin_score;
  is_cutin_ = prediction_object.is_cutin;
  perception_bounding_box_ = planning_math::Box2d(
      {prediction_object.position_x, prediction_object.position_y},
      prediction_object.yaw, prediction_object.length, prediction_object.width);
  is_caution_level_obstacle_ = true;
  is_truncated_ = false;
  is_st_boundary_constructed_ = false;
  is_zoomed_ = false;
  is_ego_lane_overlap_ = prediction_object.is_ego_lane_overlap;
  truncated_time_ = std::numeric_limits<double>::infinity();
  switch (prediction_object.type) {
  case 10001:
    type_ = ObjectType::PEDESTRIAN;
    break;
  case 10002:
    type_ = ObjectType::OFO;
    break;
  case 20001:
    type_ = ObjectType::CONE_BUCKET;
    break;
  case 0:
    type_ = ObjectType::NOT_KNOW;
    break;
  case 1:
    type_ = ObjectType::COUPE;
    break;
  case 2:
    type_ = ObjectType::TRANSPORT_TRUNK;
    break;
  case 3:
    type_ = ObjectType::BUS;
    break;
  case 4:
    type_ = ObjectType::ENGINEER_TRUCK;
    break;
  case 5:
    type_ = ObjectType::TRICYCLE;
    break;
  case (unsigned int)-1:
    type_ = ObjectType::STOP_LINE;
    break;
  default:
    type_ = ObjectType::NOT_KNOW;
    break;
  }

  perception_obstacle_.position.x = prediction_object.position_x;
  perception_obstacle_.position.y = prediction_object.position_y;
  perception_obstacle_.position.z = 0.0;
  perception_obstacle_.shape.length = prediction_object.length;
  perception_obstacle_.shape.width = prediction_object.width;
  perception_obstacle_.heading_yaw =
      planning_math::NormalizeAngle(prediction_object.yaw);
  perception_obstacle_.acceleration.x = prediction_object.acc;
  perception_obstacle_.velocity.x =
      prediction_object.speed * std::cos(perception_obstacle_.heading_yaw);
  perception_obstacle_.velocity.y =
      prediction_object.speed * std::sin(perception_obstacle_.heading_yaw);

  perception_obstacle_.shape.height = 0.0;
  perception_speed_ = prediction_object.speed;
  speed_direction_ = perception_obstacle_.heading_yaw;

  std::vector<planning_math::Vec2d> polygon_points;
  if (prediction_object.bottom_polygon_points.size() < 3) {
    perception_bounding_box_.GetAllCorners(&polygon_points);
  } else {
    // MSD_LOG(INFO, "raw size %d",
    // prediction_object.bottom_polygon_points.size());
    for (int i = 0; i < (int)prediction_object.bottom_polygon_points.size() - 1;
         ++i) {
      auto &point = prediction_object.bottom_polygon_points[i];
      polygon_points.emplace_back(planning_math::Vec2d(point.x, point.y));
      // MSD_LOG(INFO, "point x %f y %f", polygon_points[i].x(),
      // polygon_points[i].y()); MSD_LOG(INFO, "rel point x %f y %f",
      // polygon_points[i].x() - perception_obstacle_.position.x,
      // polygon_points[i].y()-perception_obstacle_.position.y);
    }
    // polygon_points.erase(polygon_points.begin());
  }
  auto extracted_polygon_points = polygon_points;
  ExtractPointAtSpecifiedResolution(extracted_polygon_points);
  if (extracted_polygon_points.size() < 3) {
    extracted_polygon_points = polygon_points;
  }
  // MSD_LOG(INFO, "after remove redundant point size: %d",
  // extracted_polygon_points.size()); for (auto point :
  // extracted_polygon_points) {
  //   MSD_LOG(INFO, "after remove redundant point x %f y %f", point.x(),
  //   point.y()); MSD_LOG(INFO, "after remove redundant rel point x %f y %f",
  //   point.x() - perception_obstacle_.position.x,
  //         point.y()-perception_obstacle_.position.y);
  // }
  if (!planning_math::Polygon2d::ComputeConvexHull(extracted_polygon_points,
                                                   &perception_polygon_)) {
    MSD_LOG(INFO, "polygon_debug invalid cart polygon");
  }
  // MSD_LOG(INFO, "after convex hull size %d %d %d",
  // polygon_points.size(),result_p.points().size(),
  // perception_polygon_.points().size());
  std::vector<planning_math::Vec2d> ego_polygon_points;
  // MSD_LOG(INFO, "obstacle[%d] polygon size : %d %d, ego x %f y %f",
  // prediction_object.bottom_polygon_points.size(), polygon_points.size(),
  // perception_obstacle_.position.x, perception_obstacle_.position.y);
  for (const auto &point : perception_polygon_.points()) {
    ego_polygon_points.emplace_back(
        planning_math::Vec2d(point.x() - perception_obstacle_.position.x,
                             point.y() - perception_obstacle_.position.y));
    // MSD_LOG(INFO, "ego point x %f y %f", ego_polygon_points.back().x(),
    // ego_polygon_points.back().y());
  }
  // MSD_LOG(INFO, "obstacle[%d] last polygon size : %d",
  // polygon_points.size());
  if (!planning_math::Polygon2d::ComputeConvexHull(ego_polygon_points,
                                                   &car_ego_polygon_)) {
    MSD_LOG(INFO, "polygon_debug invalid ego polygon");
  }
  is_virtual_ = id_ < 0;
  speed_ = prediction_object.speed;
  acc_ = prediction_object.acc;

  if (traj_index < 0 || prediction_object.trajectory_array.empty()) {
    prob_ = 1.0;
    b_minor_modal_ = false;
  } else {
    b_minor_modal_ =
        prediction_object.trajectory_array[traj_index].b_minor_modal;
    prob_ = prediction_object.trajectory_array[traj_index].prob;
  }

  if (traj_index < 0 || is_static) {
    return;
  }
  auto &prediction_trajectory =
      prediction_object.trajectory_array[traj_index].trajectory;
  if (prediction_trajectory.empty()) {
    return;
  }
  intention_ = prediction_object.trajectory_array[traj_index].intention;
  prediction_source_ = prediction_object.trajectory_array[traj_index].source;
  is_traj_sigma_valid_ =
      prediction_object.trajectory_array[traj_index].b_valid_sigma;
  max_sigma_ = 0.0;

  trajectory_.clear();
  double relative_time = start_timestamp;
  double cumulative_s = 0.0;
  double start_vel_direction = prediction_trajectory[0].yaw;
  double sigma_t = 0.0;
  constexpr double InvalidSigma = 10.0;
  for (size_t i = 0; i < prediction_trajectory.size(); ++i) {
    TrajectoryPoint tp;
    auto &traj_point = prediction_trajectory[i];
    tp.v = traj_point.speed;
    tp.a = 0;
    tp.prediction_prob = traj_point.prob;
    tp.path_point.x = traj_point.x;
    tp.path_point.y = traj_point.y;
    tp.sigma_x = traj_point.std_dev_x;
    tp.sigma_y = traj_point.std_dev_y;
    tp.path_point.theta = planning_math::NormalizeAngle(traj_point.theta);
    tp.velocity_direction = planning_math::NormalizeAngle(traj_point.yaw);
    // tp.relative_ego_x = traj_point.relative_ego_x;
    // tp.relative_ego_y = traj_point.relative_ego_y;
    tp.relative_ego_yaw = traj_point.relative_ego_yaw;
    // tp.relative_ego_speed = traj_point.relative_ego_speed;
    // tp.relative_ego_std_dev_x = traj_point.relative_ego_std_dev_x;
    // tp.relative_ego_std_dev_y = traj_point.relative_ego_std_dev_y;
    // tp.relative_ego_std_dev_yaw = traj_point.relative_ego_std_dev_yaw;
    // tp.relative_ego_std_dev_speed = traj_point.relative_ego_std_dev_speed;
    tp.path_point.s = cumulative_s;
    tp.relative_time = relative_time;
    // todo: get relative time from prediction msg !!!
    relative_time += 0.2; // prediction time step
    if (i % 5 == 0) {
      if (is_traj_sigma_valid_ && tp.sigma_x < InvalidSigma &&
          tp.sigma_y < InvalidSigma) {
        sigma_t = std::sqrt(std::pow(tp.sigma_x, 2) + std::pow(tp.sigma_y, 2));
        if (sigma_t > max_sigma_) {
          max_sigma_ = sigma_t;
        }
      }
    }
    if (i >= 1) {
      cumulative_s +=
          std::hypot(trajectory_[i - 1].path_point.x - tp.path_point.x,
                     trajectory_[i - 1].path_point.y - tp.path_point.y);
    }
    trajectory_.emplace_back(tp);
  }
  is_static_ = std::fabs(trajectory_.back().path_point.s -
                         trajectory_.front().path_point.s) < 5.e-3;
  DiscretizedTrajectory::CompensateTrajectory(trajectory_,
                                              FLAGS_lon_decision_time_horizon);

  // reset perception info matched with current timestamp
  auto init_point = GetPointAtTime(0.0);
  perception_polygon_ = GetPolygonAtPoint(init_point);
  perception_bounding_box_ = GetBoundingBox(init_point);
  speed_direction_ = init_point.velocity_direction;
}

void Obstacle::update(const Obstacle &obstacle) { *this = obstacle; }

void Obstacle::ExtractPointAtSpecifiedResolution(
    std::vector<planning_math::Vec2d> &points) {
  const double PI = std::atan(1.0) * 4.0;
  constexpr int kMinPointsNum = 4;
  constexpr double kMinDist = 0.5;
  const double kMinCosTheta = cos(0.8 * PI);
  if (points.size() <= kMinPointsNum) {
    return;
  }
  size_t i = 0;
  size_t j = 1;
  while (i < points.size() && j + 1 < points.size()) {
    planning_math::LineSegment2d seg(points.at(i), points.at(j + 1));
    double cos_theta =
        planning_math::InnerProd(points.at(j), points.at(i), points.at(j + 1)) /
        (points.at(j).DistanceTo(points.at(i)) *
         points.at(j).DistanceTo(points.at(j + 1)));

    if (cos_theta > kMinCosTheta ||
        seg.DistanceSquareTo(points.at(j)) > kMinDist * kMinDist) {
      ++i;
      if (i != j) {
        points.at(i) = points.at(j);
      }
    }
    ++j;
  }
  points.at(++i) = points.back();
  points.resize(i + 1);
}

TrajectoryPoint Obstacle::GetPointAtTime(const double relative_time) const {
  const auto &points = trajectory_;
  if (points.size() < 2) {
    TrajectoryPoint point;
    point.path_point.x = perception_obstacle_.position.x;
    point.path_point.y = perception_obstacle_.position.y;
    point.path_point.z = perception_obstacle_.position.z;
    point.path_point.theta = perception_obstacle_.heading_yaw;
    point.velocity_direction = 0.0;
    point.path_point.s = 0.0;
    point.path_point.kappa = 0.0;
    point.path_point.dkappa = 0.0;
    point.path_point.ddkappa = 0.0;
    point.v = 0.0;
    point.a = 0.0;
    point.sigma_x = 0.0;
    point.sigma_y = 0.0;
    point.relative_time = 0.0;
    // point.relative_ego_x = 0.0;
    // point.relative_ego_y = 0.0;
    point.relative_ego_yaw = 0.0;
    // point.relative_ego_speed = 0.0;
    return point;
  } else {
    auto comp = [](const TrajectoryPoint p, const double time) {
      return p.relative_time < time;
    };

    auto it_lower =
        std::lower_bound(points.begin(), points.end(), relative_time, comp);

    if (it_lower == points.begin()) {
      return *points.begin();
    } else if (it_lower == points.end()) {
      return *points.rbegin();
    }
    return planning_math::InterpolateUsingLinearApproximation(
        *(it_lower - 1), *it_lower, relative_time);
  }
}

planning_math::Box2d
Obstacle::GetBoundingBox(const TrajectoryPoint &point) const {
  return planning_math::Box2d(
      {point.path_point.x, point.path_point.y}, point.path_point.theta,
      perception_obstacle_.shape.length, perception_obstacle_.shape.width);
}

planning_math::Polygon2d
Obstacle::GetPolygonAtPoint(const TrajectoryPoint &point) const {
  std::vector<planning_math::Vec2d> polygon_points;
  double rel_theta = point.path_point.theta - perception_obstacle_.heading_yaw;

  for (const auto &ego_point : car_ego_polygon_.points()) {
    polygon_points.emplace_back(planning_math::Vec2d(
        ego_point.x() * cos(rel_theta) - ego_point.y() * sin(rel_theta) +
            point.path_point.x,
        ego_point.y() * cos(rel_theta) + ego_point.x() * sin(rel_theta) +
            point.path_point.y));
  }
  planning_math::Polygon2d polygon;
  if (!planning_math::Polygon2d::ComputeConvexHull(polygon_points, &polygon)) {
    MSD_LOG(INFO, "polygon_debug : get position %f %f failed",
            point.path_point.x, point.path_point.y);
    for (auto p : polygon_points) {
      MSD_LOG(INFO, "polygon_debug invald point x %f y %f",
              p.x() - point.path_point.x, p.y() - point.path_point.y);
    }
    if (!planning_math::Polygon2d::ComputeConvexHull(
            GetBoundingBox(point).GetAllCorners(), &polygon)) {
      MSD_LOG(INFO, "polygon_debug : invalid box polygon");
    }
  }
  return polygon;
}

SLBoundary
Obstacle::GetSLBoundary(std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
                        const TrajectoryPoint &point) const {
  SLBoundary sl_boundary;
  planning_math::Box2d object_moving_box = GetBoundingBox(point);

  double obs_start_s(std::numeric_limits<double>::max());
  double obs_end_s(std::numeric_limits<double>::lowest());
  double obs_start_l(std::numeric_limits<double>::max());
  double obs_end_l(std::numeric_limits<double>::lowest());
  std::vector<planning_math::Vec2d> obstacle_points;
  object_moving_box.GetAllCorners(&obstacle_points);
  for (const planning_math::Vec2d &obs_point : obstacle_points) {
    Point2D frenet_point, carte_point;
    carte_point.x = obs_point.x();
    carte_point.y = obs_point.y();
    if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
        TRANSFORM_FAILED) {
      continue;
    }
    obs_start_s = std::min(obs_start_s, frenet_point.x);
    obs_end_s = std::max(obs_end_s, frenet_point.x);
    obs_start_l = std::min(obs_start_l, frenet_point.y);
    obs_end_l = std::max(obs_end_l, frenet_point.y);
  }
  sl_boundary.start_s = obs_start_s;
  sl_boundary.end_s = obs_end_s;
  sl_boundary.start_l = obs_start_l;
  sl_boundary.end_l = obs_end_l;
  return sl_boundary;
}

bool Obstacle::IsValidPredictionObject(const PredictionObject &object) {
  if (object.length <= 0.0) {
    MSD_LOG(INFO, "invalid obstacle length:%lf", object.length);
    return false;
  }
  if (object.width <= 0.0) {
    MSD_LOG(INFO, "invalid obstacle width:%lf", object.width);
    return false;
  }

  return true;
}

std::list<std::unique_ptr<Obstacle>>
Obstacle::CreateObstacles(const std::vector<PredictionObject> &predictions) {
  std::list<std::unique_ptr<Obstacle>> obstacles;
  for (const auto &prediction_obstacle : predictions) {
    if (!IsValidPredictionObject(prediction_obstacle)) {
      MSD_LOG(INFO, "Invalid perception obstacle: ");
      continue;
    }
    const auto perception_id = std::to_string(prediction_obstacle.id);
    if (prediction_obstacle.trajectory_array[0].trajectory.empty()) {
      // todo: add backup for prdiction without trajectory
      //   obstacles.emplace_back(
      //       new Obstacle(perception_id,
      //       prediction_obstacle.perception_obstacle(),
      //                    prediction_obstacle.priority().priority(),
      //                    prediction_obstacle.is_static()));
      continue;
    }

    int trajectory_index = 0;
    for (const auto &trajectory : prediction_obstacle.trajectory_array) {
      bool is_valid_trajectory = true;
      for (const auto &point : trajectory.trajectory) {
        if (!IsValidTrajectoryPoint(point)) {
          MSD_LOG(INFO, "obj:%s TrajectoryPoint: is NOT valid.",
                  perception_id.c_str());
          is_valid_trajectory = false;
          break;
        }
      }
      if (!is_valid_trajectory) {
        continue;
      }

      obstacles.emplace_back(new Obstacle(prediction_obstacle.id,
                                          trajectory_index, prediction_obstacle,
                                          false, 0.0));
      ++trajectory_index;
    }
  }
  return obstacles;
}

std::unique_ptr<Obstacle> Obstacle::CreateStaticVirtualObstacles(
    const int &id, const planning_math::Box2d &obstacle_box) {
  // create a "virtual" perception_obstacle
  maf_perception_interface::PerceptionFusionObjectData perception_obstacle;
  // simulator needs a valid integer
  int negative_id = std::hash<int>{}(id);
  // set the first bit to 1 so negative_id became negative number
  perception_obstacle.track_id = static_cast<int32_t>(negative_id);
  perception_obstacle.position.x = obstacle_box.center().x();
  perception_obstacle.position.y = obstacle_box.center().y();
  perception_obstacle.heading_yaw = obstacle_box.heading();
  perception_obstacle.velocity.x = 0;
  perception_obstacle.velocity.y = 0;
  perception_obstacle.velocity.z = 0;
  perception_obstacle.shape.length = obstacle_box.length();
  perception_obstacle.shape.width = obstacle_box.width();

  std::vector<planning_math::Vec2d> corner_points;
  obstacle_box.GetAllCorners(&corner_points);
  auto *obstacle = new Obstacle(id, perception_obstacle, true);
  obstacle->is_virtual_ = true;
  return std::unique_ptr<Obstacle>(obstacle);
}

// todo : unify prediction trajectory attribute
bool Obstacle::IsValidTrajectoryPoint(const PredictionTrajectoryPoint &point) {
  return !(std::isnan(point.x) || std::isnan(point.y) ||
           std::isnan(point.yaw) || std::isnan(point.speed));
}

void Obstacle::SetPerceptionSlBoundary(const SLBoundary &sl_boundary) {
  sl_boundary_ = sl_boundary;
}
void Obstacle::ComputePredictSlBoundary(
    std::shared_ptr<FrenetCoordinateSystem> frenet_coord) {
  double obs_start_s(std::numeric_limits<double>::max());
  double obs_end_s(std::numeric_limits<double>::lowest());
  double obs_start_l(std::numeric_limits<double>::max());
  double obs_end_l(std::numeric_limits<double>::lowest());
  bool frenet_valid;
  std::vector<planning_math::Vec2d> obstacle_points;
  is_pred_frenet_invalid_ = true;
  if (trajectory_.empty())
    return;
  for (auto trajectory_point : trajectory_) {
    obs_start_s = std::numeric_limits<double>::max();
    obs_end_s = std::numeric_limits<double>::lowest();
    obs_start_l = std::numeric_limits<double>::max();
    obs_end_l = std::numeric_limits<double>::lowest();
    planning_math::Box2d prediction_bounding_box_tmp = planning_math::Box2d(
        {trajectory_point.path_point.x, trajectory_point.path_point.y},
        trajectory_point.path_point.theta,
        perception_bounding_box_.length() + kSizeEpsilon,
        perception_bounding_box_.width() + kSizeEpsilon);
    prediction_bounding_box_tmp.GetAllCorners(&obstacle_points);
    frenet_valid = true;
    for (const planning_math::Vec2d &obs_point : obstacle_points) {
      Point2D frenet_point, carte_point;
      carte_point.x = obs_point.x();
      carte_point.y = obs_point.y();
      if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
          TRANSFORM_FAILED) {
        frenet_valid = false;
        break;
      }
      if (std::isnan(frenet_point.x) || std::isnan(frenet_point.y)) {
        frenet_valid = false;
        break;
      }
      obs_start_s = std::min(obs_start_s, frenet_point.x);
      obs_end_s = std::max(obs_end_s, frenet_point.x);
      obs_start_l = std::min(obs_start_l, frenet_point.y);
      obs_end_l = std::max(obs_end_l, frenet_point.y);
    }
    if (!frenet_valid)
      continue;
    if (obs_start_l * obs_end_l > 0)
      continue;
    is_pred_frenet_invalid_ = false;
    pred_sl_boundary_.start_s = obs_start_s;
    pred_sl_boundary_.end_s = obs_end_s;
    pred_sl_boundary_.start_l = obs_start_l;
    pred_sl_boundary_.end_l = obs_end_l;
    prediction_bounding_box_ = prediction_bounding_box_tmp;
    double curve_heading = frenet_coord->GetRefCurveHeading(obs_start_s);
    yaw_relative_frenet_ = speed_direction_ - curve_heading;
    pred_sl_boundary_relative_time_ = trajectory_point.relative_time;
    return;
  }
  return;
}
void Obstacle::ComputeSlBoundary(
    std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
    SLBoundary *const sl_boundary) {
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
  perception_bounding_box_.GetAllCorners(&obstacle_points);
  is_frenet_invalid_ = false;
  for (const planning_math::Vec2d &obs_point : obstacle_points) {
    Point2D frenet_point, carte_point;
    carte_point.x = obs_point.x();
    carte_point.y = obs_point.y();
    MSD_LOG(INFO, "DEBUG_CJ_913: ID:%d, Corner1:x:%f,y:%f", Id(), obs_point.x(),
            obs_point.y());
    if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
        TRANSFORM_FAILED) {
      is_frenet_invalid_ = true;
      ComputePredictSlBoundary(frenet_coord);
      MSD_LOG(INFO, "DEBUG_CJ_913: ID:%d, P1", Id());
      return;
    }
    if (std::isnan(frenet_point.x) || std::isnan(frenet_point.y)) {
      is_frenet_invalid_ = true;
      ComputePredictSlBoundary(frenet_coord);
      MSD_LOG(INFO, "DEBUG_CJ_913: ID:%d, P2", Id());
      return;
    }
    obs_start_s = std::min(obs_start_s, frenet_point.x);
    obs_end_s = std::max(obs_end_s, frenet_point.x);
    obs_start_l = std::min(obs_start_l, frenet_point.y);
    obs_end_l = std::max(obs_end_l, frenet_point.y);
  }
  sl_boundary->start_s = obs_start_s;
  sl_boundary->end_s = obs_end_s;
  sl_boundary->start_l = obs_start_l;
  sl_boundary->end_l = obs_end_l;

  Point2D frenet_point, carte_point;
  carte_point.x = perception_obstacle_.position.x;
  carte_point.y = perception_obstacle_.position.y;
  if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
      TRANSFORM_FAILED) {
    r_frenet_ = obs_end_l;
    s_frenet_ = obs_end_s;
    yaw_relative_frenet_ = 0.0;
    is_frenet_invalid_ = true;
    ComputePredictSlBoundary(frenet_coord);
    MSD_LOG(INFO, "DEBUG_CJ_913: ID:%d, P3", Id());
    return;
  } else {
    r_frenet_ = frenet_point.y;
    s_frenet_ = frenet_point.x;
  }
  if (std::isnan(r_frenet_) || std::isnan(s_frenet_)) {
    is_frenet_invalid_ = true;
    ComputePredictSlBoundary(frenet_coord);
    MSD_LOG(INFO, "DEBUG_CJ_913: ID:%d, P4", Id());
    return;
  }
  double curve_heading = frenet_coord->GetRefCurveHeading(s_frenet_);
  yaw_relative_frenet_ = speed_direction_ - curve_heading;
  MSD_LOG(INFO, "DEBUG_CJ_YAW: ================================");
  MSD_LOG(INFO, "DEBUG_CJ_YAW: ID:%d, TIME:%.3f", Id(),
          MTIME()->timestamp().sec());
  MSD_LOG(INFO, "DEBUG_CJ_YAW: ID:%d", Id());
  MSD_LOG(INFO, "DEBUG_CJ_YAW: ID:%d, spd_yaw:%f", Id(), speed_direction_);
  MSD_LOG(INFO, "DEBUG_CJ_YAW: ID:%d, curv_yaw:%f", Id(), curve_heading);
  is_pred_frenet_invalid_ = true;
  return;
}

void Obstacle::BuildReferenceLineStBoundary(
    const DiscretizedPath &reference_line,
    std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
    const double adc_start_s) {
  const double adc_width =
      ConfigurationContext::Instance()->get_vehicle_param().width;
  if (is_static_ || trajectory_.empty()) {
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    double start_s = sl_boundary_.start_s;
    double end_s = sl_boundary_.end_s;
    if (end_s - start_s < kStBoundaryDeltaS) {
      end_s = start_s + kStBoundaryDeltaS;
    }
    const planning_math::Vec2d center = perception_bounding_box_.center();
    double radius_sqr = perception_bounding_box_.diagonal() / 2.0 + adc_width;
    radius_sqr = radius_sqr + 1e-10;
    double cul_s = 0.0;
    DiscretizedPath reference_line_path(reference_line);
    bool isBlockRoad = false;
    while (cul_s <= reference_line.Length()) {
      PathPoint pp = reference_line.Evaluate(cul_s);
      if (std::hypot(pp.x - center.x(), pp.y - center.y()) < radius_sqr) {
        isBlockRoad = true;
        break;
      }
      cul_s += radius_sqr;
      if (cul_s > reference_line.Length()) {
        cul_s = reference_line.Length();
      }
    }
    if (!isBlockRoad) {
      return;
    }
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, 0.0),
                             STPoint(end_s - adc_start_s, 0.0));
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, FLAGS_st_max_t),
                             STPoint(end_s - adc_start_s, FLAGS_st_max_t));
    reference_line_st_boundary_ = STBoundary(point_pairs);
  } else {
    if (BuildTrajectoryStBoundary(reference_line, frenet_coord, adc_start_s,
                                  &reference_line_st_boundary_)) {
      MSD_LOG(INFO, "Found st_boundary for obstacle %d", id_);
      MSD_LOG(INFO,
              "st_boundary: min_t = %lf, max_t = %lf, min_s = %lf, max_s = %lf",
              reference_line_st_boundary_.min_t(),
              reference_line_st_boundary_.max_t(),
              reference_line_st_boundary_.min_s(),
              reference_line_st_boundary_.max_s());
    } else {
      MSD_LOG(INFO, "No st_boundary for obstacle %d", id_);
    }
  }
}

bool Obstacle::BuildTrajectoryStBoundary(
    const DiscretizedPath &reference_line,
    std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
    const double adc_start_s, STBoundary *const st_boundary) {
  if (!IsValidObstacle(perception_obstacle_)) {
    MSD_LOG(ERROR, "Fail to build trajectory st boundary because object is not "
                   "valid. FusionObject: ");
    ;
    return false;
  }

  const double object_width = perception_obstacle_.shape.width;
  const double object_length = perception_obstacle_.shape.length;
  if (trajectory_.empty()) {
    MSD_LOG(ERROR, "object %d has no trajectory", id_);
    return false;
  }
  const double adc_length =
      ConfigurationContext::Instance()->get_vehicle_param().length;
  const double adc_half_length = adc_length / 2.0;
  const double adc_width =
      ConfigurationContext::Instance()->get_vehicle_param().width;
  planning_math::Box2d min_box({0, 0}, 1.0, 1.0, 1.0);
  planning_math::Box2d max_box({0, 0}, 1.0, 1.0, 1.0);
  std::vector<std::pair<STPoint, STPoint>> polygon_points;

  SLBoundary last_sl_boundary{};
  last_sl_boundary.start_l = 0.0;
  last_sl_boundary.end_l = 0.0;
  int last_index = 0;

  for (size_t i = 1; i < trajectory_.size(); ++i) {
    const auto &first_traj_point = trajectory_[i - 1];
    const auto &second_traj_point = trajectory_[i];
    const auto &first_point = first_traj_point.path_point;
    const auto &second_point = second_traj_point.path_point;

    double total_length =
        object_length + util::DistanceXY(first_point, second_point);

    planning_math::Vec2d center((first_point.x + second_point.x) / 2.0,
                                (first_point.y + second_point.y) / 2.0);
    planning_math::Box2d object_moving_box(center, first_point.theta,
                                           total_length, object_width);
    SLBoundary object_boundary;
    // NOTICE: this method will have errors when the reference line is not
    // straight. Need double loop to cover all corner cases.
    const double distance_xy = util::DistanceXY(
        trajectory_[last_index].path_point, trajectory_[i].path_point);

    if (last_sl_boundary.start_l > distance_xy ||
        last_sl_boundary.end_l < -distance_xy) {
      continue;
    }

    // const double mid_s =
    //     (last_sl_boundary.start_s + last_sl_boundary.end_s) / 2.0;
    // const double start_s = std::fmax(0.0, mid_s - 2.0 * distance_xy);
    // const double end_s = (i == 1) ? reference_line.Length()
    // : std::fmin(reference_line.Length(),
    //             mid_s + 2.0 * distance_xy);

    // construct rectangular SLboundary for obstacle in frenet coordinate
    // system
    double obs_start_s(std::numeric_limits<double>::max());
    double obs_end_s(std::numeric_limits<double>::lowest());
    double obs_start_l(std::numeric_limits<double>::max());
    double obs_end_l(std::numeric_limits<double>::lowest());
    std::vector<planning_math::Vec2d> obstacle_points;
    object_moving_box.GetAllCorners(&obstacle_points);
    for (const planning_math::Vec2d &obs_point : obstacle_points) {
      Point2D frenet_point, carte_point;
      carte_point.x = obs_point.x();
      carte_point.y = obs_point.y();
      if (frenet_coord->CartCoord2FrenetCoord(carte_point, frenet_point) ==
          TRANSFORM_FAILED) {
        continue;
      }
      obs_start_s = std::min(obs_start_s, frenet_point.x);
      obs_end_s = std::max(obs_end_s, frenet_point.x);
      obs_start_l = std::min(obs_start_l, frenet_point.y);
      obs_end_l = std::max(obs_end_l, frenet_point.y);
    }
    object_boundary.start_s = obs_start_s;
    object_boundary.end_s = obs_end_s;
    object_boundary.start_l = obs_start_l;
    object_boundary.end_l = obs_end_l;

    // update history record
    last_sl_boundary = object_boundary;
    last_index = i;

    // lateral filter: skip if object is entirely on one side of reference
    // line.
    constexpr double kSkipLDistanceFactor = 0.4;
    const double skip_l_distance =
        (object_boundary.end_s - object_boundary.start_s) *
            kSkipLDistanceFactor +
        adc_width / 2.0;

    if (!IsCautionLevelObstacle() &&
        (std::fmin(object_boundary.start_l, object_boundary.end_l) >
             skip_l_distance ||
         std::fmax(object_boundary.start_l, object_boundary.end_l) <
             -skip_l_distance)) {
      continue;
    }

    if (!IsCautionLevelObstacle() && object_boundary.end_s < 0) {
      // longitudinal filter: skip if behind reference line
      continue;
    }

    constexpr double kSparseMappingS = 20.0;
    const double st_boundary_delta_s =
        (std::fabs(object_boundary.start_s - adc_start_s) > kSparseMappingS)
            ? kStBoundarySparseDeltaS
            : kStBoundaryDeltaS;
    const double object_s_diff =
        object_boundary.end_s - object_boundary.start_s;
    if (object_s_diff < st_boundary_delta_s) {
      continue;
    }

    const double delta_t =
        second_traj_point.relative_time - first_traj_point.relative_time;
    double low_s = std::max(object_boundary.start_s - adc_half_length, 0.0);
    bool has_low = false;
    double high_s =
        std::min(object_boundary.end_s + adc_half_length, FLAGS_st_max_s);
    bool has_high = false;
    while (low_s + st_boundary_delta_s < high_s && !(has_low && has_high)) {
      if (!has_low) {
        auto low_ref = reference_line.Evaluate(low_s);
        planning_math::Box2d ref_ego_box({low_ref.x, low_ref.y}, low_ref.theta,
                                         adc_length, adc_width);
        has_low = object_moving_box.HasOverlap(ref_ego_box);
        low_s += st_boundary_delta_s;
      }
      if (!has_high) {
        auto high_ref = reference_line.Evaluate(high_s);
        planning_math::Box2d ref_ego_box({high_ref.x, high_ref.y},
                                         high_ref.theta, adc_length, adc_width);
        has_high = object_moving_box.HasOverlap(ref_ego_box);
        high_s -= st_boundary_delta_s;
      }
    }
    if (has_low && has_high) {
      low_s -= st_boundary_delta_s;
      high_s += st_boundary_delta_s;
      double low_t =
          (first_traj_point.relative_time +
           std::fabs((low_s - object_boundary.start_s) / object_s_diff) *
               delta_t);
      polygon_points.emplace_back(
          std::make_pair(STPoint{low_s - adc_start_s, low_t},
                         STPoint{high_s - adc_start_s, low_t}));
      double high_t =
          (first_traj_point.relative_time +
           std::fabs((high_s - object_boundary.start_s) / object_s_diff) *
               delta_t);
      if (high_t - low_t > 0.05) {
        polygon_points.emplace_back(
            std::make_pair(STPoint{low_s - adc_start_s, high_t},
                           STPoint{high_s - adc_start_s, high_t}));
      }
    }
  }

  if (!polygon_points.empty()) {
    std::sort(polygon_points.begin(), polygon_points.end(),
              [](const std::pair<STPoint, STPoint> &a,
                 const std::pair<STPoint, STPoint> &b) {
                return a.first.t() < b.first.t();
              });
    auto last = std::unique(polygon_points.begin(), polygon_points.end(),
                            [](const std::pair<STPoint, STPoint> &a,
                               const std::pair<STPoint, STPoint> &b) {
                              return std::fabs(a.first.t() - b.first.t()) <
                                     kStBoundaryDeltaT;
                            });
    polygon_points.erase(last, polygon_points.end());
    if (polygon_points.size() > 2) {
      *st_boundary = STBoundary(polygon_points);
    }
  } else {
    return false;
  }
  return true;
}

const STBoundary &Obstacle::reference_line_st_boundary() const {
  return reference_line_st_boundary_;
}

const STBoundary &Obstacle::path_st_boundary() const {
  return path_st_boundary_;
}

const SLPolygonSeq &Obstacle::sl_polygon_seq() const { return sl_polygon_seq_; }

bool Obstacle::has_sl_polygon_seq() const { return has_sl_polygon_seq_; }

SLBoundary Obstacle::PerceptionSLBoundary() const { return sl_boundary_; }

SLBoundary Obstacle::PredictionSLBoundary() const { return pred_sl_boundary_; }

void Obstacle::set_path_st_boundary(const STBoundary &boundary) {
  path_st_boundary_ = boundary;
}

void Obstacle::SetStBoundaryType(const STBoundary::BoundaryType &type) {
  path_st_boundary_.SetBoundaryType(type);
}

void Obstacle::EraseStBoundary() { path_st_boundary_ = STBoundary(); }

void Obstacle::SetReferenceLineStBoundary(const STBoundary &boundary) {
  reference_line_st_boundary_ = boundary;
}

void Obstacle::SetReferenceLineStBoundaryType(
    const STBoundary::BoundaryType &type) {
  reference_line_st_boundary_.SetBoundaryType(type);
}

void Obstacle::EraseReferenceLineStBoundary() {
  reference_line_st_boundary_ = STBoundary();
}

void Obstacle::SetSLPolygonSequence(const SLPolygonSeq &sl_polygon_seq) {
  sl_polygon_seq_.reserve(50);
  sl_polygon_seq_.assign(sl_polygon_seq.begin(), sl_polygon_seq.end());
  sl_polygon_seq_.update(sl_polygon_seq.GetTimeStep(),
                         sl_polygon_seq.GetIsUniformTimeStep(),
                         sl_polygon_seq.GetInvalidTimeSections());
  has_sl_polygon_seq_ = true;
}

void Obstacle::SetSLPolygonSequenceInvalid(bool is_invalid) {
  invalid_sl_polygon_seq_ = is_invalid;
}

bool Obstacle::is_sl_polygon_seq_invalid() const {
  return invalid_sl_polygon_seq_;
}

bool Obstacle::IsValidObstacle(
    const maf_perception_interface::PerceptionFusionObjectData
        &perception_obstacle) {
  const double object_width = perception_obstacle.shape.width;
  const double object_length = perception_obstacle.shape.length;

  const double kMinObjectDimension = 1.0e-6;
  return !std::isnan(object_width) && !std::isnan(object_length) &&
         object_width > kMinObjectDimension &&
         object_length > kMinObjectDimension;
}

bool Obstacle::IsIrrelevant(const DiscretizedPath &reference_line,
                            const SLBoundary &adc_sl) {
  if (!IsCautionLevelObstacle()) {
    return false;
  }
  // ignore obstacle far away the current lane in longitudinal direction
  if (sl_boundary_.end_s > reference_line.Length()) {
    return true;
  }
  bool obsIsOnlane = false;
  double lane_left_width = 1.6;
  double lane_right_width = 1.6;
  if (sl_boundary_.end_s < 0.0 ||
      sl_boundary_.start_s > reference_line.Length()) {
    obsIsOnlane = false;
  } else if (sl_boundary_.start_l <= lane_left_width &&
             sl_boundary_.end_l >= -lane_right_width) {
    obsIsOnlane = true;
  }

  // if adc is on the road, and obstacle(initial pose) behind adc, ignore
  // todo: refactor this func in reference line class
  // or add is_on_reference_line_flag to check if adc on this reference_line
  if (obsIsOnlane) {
    if (sl_boundary_.end_s < adc_sl.end_s || sl_boundary_.end_s < 0.0) {
      return true;
    }
  }
  return false;
}

void Obstacle::CheckLaneBlocking(const DiscretizedPath &reference_line) {
  if (!IsStatic()) {
    is_lane_blocking_ = false;
    return;
  }

  if (sl_boundary_.start_l * sl_boundary_.end_l < 0.0) {
    is_lane_blocking_ = true;
    return;
  }

  // driving_width means the left lateral space for the ego car to occupy
  // in order to avoid the obstacle
  // todo: add lane class to get lane width according to s
  double lane_left_width = 1.6;
  double lane_right_width = 1.6;
  double driving_width = std::max(lane_left_width - sl_boundary_.end_l,
                                  lane_right_width + sl_boundary_.start_l);
  driving_width = std::min(lane_left_width + lane_right_width, driving_width);
  // const double driving_width =
  // reference_line.GetDrivingWidth(sl_boundary_);

  bool isOnlane = false;
  if (sl_boundary_.end_s < 0 ||
      sl_boundary_.start_s > reference_line.Length()) {
    isOnlane = false;
  } else if (sl_boundary_.start_l <= lane_left_width &&
             sl_boundary_.end_l >= -lane_right_width) {
    isOnlane = true;
  }
  if (isOnlane &&
      driving_width <
          ConfigurationContext::Instance()->get_vehicle_param().width +
              FLAGS_static_decision_nudge_l_buffer) {
    is_lane_blocking_ = true;
    return;
  }

  is_lane_blocking_ = false;
}

void Obstacle::SetLaneChangeBlocking(const bool is_distance_clear) {
  is_lane_change_blocking_ = is_distance_clear;
}

void Obstacle::TruncateByTime() {
  if (!is_truncated_) {
    return;
  }
  if (trajectory_.empty()) {
    return;
  }

  // cut off trajectory
  auto func = [](const TrajectoryPoint &tp, const double rel_time) {
    return tp.relative_time < rel_time;
  };
  auto it_lower = std::lower_bound(trajectory_.begin(), trajectory_.end(),
                                   truncated_time_, func);
  trajectory_.erase(trajectory_.begin(), it_lower);

  // cut off STboundary
  auto boundary_id = path_st_boundary_.id();
  auto charater_len = path_st_boundary_.characteristic_length();
  auto boundary_type = path_st_boundary_.boundary_type();
  path_st_boundary_ = path_st_boundary_.CutOffByT(truncated_time_);
  path_st_boundary_.set_id(boundary_id);
  path_st_boundary_.SetCharacteristicLength(charater_len);
  path_st_boundary_.SetBoundaryType(boundary_type);
}

} // namespace msquare
