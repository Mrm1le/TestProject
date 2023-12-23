#pragma once

#include "planner/message_type.h"
#include <vector>

namespace msquare {

#define CLEAR_LON_DECISION                                                     \
  do {                                                                         \
    const auto &lon_decison_output = context_->mutable_lon_decison_output();   \
    lon_decison_output->lon_decision_info = {};                                \
  } while (false)

#define LOG_LON_DECISION_INFO(ID, DECISION, MESSAGE)                           \
  do {                                                                         \
    const auto &lon_decison_output = context_->mutable_lon_decison_output();   \
    std::string data = DECISION + std::string(": ") +                          \
                       std::string(MSD_FILENAME(__FILE__)) +                   \
                       std::string(":") + std::to_string(__LINE__) +           \
                       std::string(": ") + MESSAGE;                            \
    lon_decison_output->lon_decision_info.lon_decision_info_map[ID]            \
        .emplace_back(data);                                                   \
  } while (false)

inline void convert_type(ObjectType obstacle_type,
                         path_planner::ObsInfo &obstacle_info) {
  switch (obstacle_type) {
  case ObjectType::NOT_KNOW:
    obstacle_info.type = path_planner::ObsInfo::Type::NOT_KNOW;
    break;
  case ObjectType::PEDESTRIAN:
    obstacle_info.type = path_planner::ObsInfo::Type::PEDESTRIAN;
    break;
  case ObjectType::OFO:
    obstacle_info.type = path_planner::ObsInfo::Type::OFO;
    break;
  case ObjectType::COUPE:
    obstacle_info.type = path_planner::ObsInfo::Type::COUPE;
    break;
  case ObjectType::TRANSPORT_TRUNK:
    obstacle_info.type = path_planner::ObsInfo::Type::TRANSPORT_TRUNK;
    break;
  case ObjectType::BUS:
    obstacle_info.type = path_planner::ObsInfo::Type::BUS;
    break;
  case ObjectType::ENGINEER_TRUCK:
    obstacle_info.type = path_planner::ObsInfo::Type::ENGINEER_TRUCK;
    break;
  case ObjectType::TRICYCLE:
    obstacle_info.type = path_planner::ObsInfo::Type::TRICYCLE;
    break;
  case ObjectType::CONE_BUCKET:
    obstacle_info.type = path_planner::ObsInfo::Type::CONE_BUCKET;
    break;
  case ObjectType::STOP_LINE:
    obstacle_info.type = path_planner::ObsInfo::Type::STOP_LINE;
    break;
  case ObjectType::GATE:
    obstacle_info.type = path_planner::ObsInfo::Type::GATE;
    break;
  case ObjectType::FREESPACE:
    obstacle_info.type = path_planner::ObsInfo::Type::FREESPACE;
    break;
  default:
    obstacle_info.type = path_planner::ObsInfo::Type::NOT_KNOW;
    break;
  }
}

inline std::vector<path_planner::Point2d>
get_polygon_edge_with_decision(const planning_math::Polygon2d &polygon,
                               const ObstacleDecision *ptr_obstacle_decision) {
  using namespace msquare::planning_math;
  std::vector<path_planner::Point2d> output_polygon_edge;
  auto comp1 = [](const Vec2d &point1, const Vec2d &point2) {
    return point1.x() < point2.x();
  };
  auto comp2 = [](const path_planner::Point2d &point1,
                  const path_planner::Point2d &point2) {
    return point1.x < point2.x;
  };

  auto points = polygon.GetAllVertices();
  sort(points.begin(), points.end(), comp1);
  LineSegment2d line(points.front(), points.back());
  output_polygon_edge.emplace_back(
      path_planner::Point2d{points.front().x(), points.front().y()});
  output_polygon_edge.emplace_back(
      path_planner::Point2d{points.back().x(), points.back().y()});

  for (const auto &point : points) {
    const double prod =
        planning_math::CrossProd(line.start(), line.end(), point);
    if (ptr_obstacle_decision->HasLateralDecision() &&
        ptr_obstacle_decision->LateralDecision().has_nudge()) {
      if (std::abs(prod) > msquare::kMathEpsilon) {

        const auto nudge_type =
            ptr_obstacle_decision->LateralDecision().nudge().type;
        if (nudge_type == ObjectNudge::LEFT_NUDGE && prod > 0.0) {
          output_polygon_edge.push_back(
              path_planner::Point2d{point.x(), point.y()});
        } else if (nudge_type == ObjectNudge::RIGHT_NUDGE && prod < 0.0) {
          output_polygon_edge.push_back(
              path_planner::Point2d{point.x(), point.y()});
        }
      }
    } else if (ptr_obstacle_decision->HasLongitudinalDecision()) {
      output_polygon_edge.push_back(
          path_planner::Point2d{point.x(), point.y()});
    }
  }

  sort(output_polygon_edge.begin(), output_polygon_edge.end(), comp2);
  return output_polygon_edge;
}

inline path_planner::ObsPrediction
get_polygon_at_time(const double &time, const Obstacle *obstacle,
                    const ObstacleDecision *ptr_obstacle_decision,
                    const double &ego_s,
                    std::shared_ptr<FrenetCoordinateSystem> frenet_coord) {
  path_planner::ObsPrediction obstacle_prediction{};

  const auto &traj_point = obstacle->GetPointAtTime(time);
  const auto &sl_polygon_seq = obstacle->sl_polygon_seq();
  PolygonWithT polygon_with_t;
  if (obstacle->sl_polygon_seq().EvaluateByTime(time, &polygon_with_t)) {
    obstacle_prediction.polygon_fren = get_polygon_edge_with_decision(
        polygon_with_t.second, ptr_obstacle_decision);
    for (const auto &point_frent : obstacle_prediction.polygon_fren) {
      Point2D cart, fren;
      fren.x = point_frent.x;
      fren.y = point_frent.y;
      (void)frenet_coord->FrenetCoord2CartCoord(fren, cart);
      obstacle_prediction.polygon_cart.emplace_back(
          path_planner::Point2d{cart.x, cart.y});
    }
    obstacle_prediction.rel_s =
        obstacle_prediction.polygon_fren.front().x - ego_s;
    obstacle_prediction.heading = traj_point.path_point.theta;
    obstacle_prediction.a = obstacle->acceleration();
    obstacle_prediction.s = obstacle_prediction.polygon_fren.front().x;

    double theta_ref = frenet_coord->GetRefCurveHeading(obstacle_prediction.s);
    // use fusion speed at 0.0 time
    if (std::fabs(time) < kMathEpsilon) {
      obstacle_prediction.v = obstacle->speed();
      obstacle_prediction.v_frenet = std::fmax(
          0.0, obstacle->speed() *
                   std::cos(traj_point.velocity_direction - theta_ref));
    } else {
      obstacle_prediction.v = traj_point.v;
      obstacle_prediction.v_frenet =
          std::fmax(0.0, traj_point.v * std::cos(traj_point.velocity_direction -
                                                 theta_ref));
    }
  }
  return obstacle_prediction;
}

template <size_t SEGMENTS>
inline std::vector<path_planner::ObsInfo> generate_obstacle_info_with_decision(
    std::shared_ptr<msquare::BaseLineInfo> baseline_info,
    msquare::ScenarioFacadeContext *context,
    const std::array<std::array<double, path_planner::QUADRATURE_ORDER>,
                     SEGMENTS> &time_at_quad_point,
    const ObstacleDecision &decision, const bool &is_mrc_inlane_brake) {
  using namespace path_planner;
  const auto &obstacle_manager = baseline_info->obstacle_manager();
  const auto &obstacle_decision_manager =
      context->mutable_obstacle_decision_manager();

  std::vector<path_planner::ObsInfo> obs_list{};

  for (const auto *ptr_obstacle : obstacle_manager.get_obstacles().Items()) {
    path_planner::ObsInfo obstacle_info{};
    obstacle_info.id = ptr_obstacle->Id();
    auto ptr_obstacle_decision =
        obstacle_decision_manager.find_obstacle_decision(ptr_obstacle->Id());
    if (ptr_obstacle_decision == nullptr)
      continue;

    const auto &lat_decision = ptr_obstacle_decision->LateralDecision();
    const auto &lon_decision = ptr_obstacle_decision->LongitudinalDecision();
    if (lon_decision.has_follow()) {
      obstacle_info.lon_decision = path_planner::ObsInfo::LonDecision::FOLLOW;
    } else if (lon_decision.has_overtake()) {
      obstacle_info.lon_decision = path_planner::ObsInfo::LonDecision::OVERTAKE;
    } else if (lat_decision.has_nudge() &&
               lat_decision.nudge().is_longitunidal_ignored) {
      obstacle_info.lon_decision =
          path_planner::ObsInfo::LonDecision::LON_IGNORE;
    } else {
      obstacle_info.lon_decision =
          path_planner::ObsInfo::LonDecision::LON_IGNORE;
    }

    if (lat_decision.has_nudge() &&
        lat_decision.nudge().type != msquare::ObjectNudge::NO_NUDGE) {
      obstacle_info.lat_decision = path_planner::ObsInfo::LatDecision::NUDGE;
      obstacle_info.nudge_side =
          lat_decision.nudge().type == msquare::ObjectNudge::LEFT_NUDGE
              ? path_planner::ObsInfo::NudgeType::LEFT_NUDGE
              : path_planner::ObsInfo::NudgeType::RIGHT_NUDGE;
    } else {
      obstacle_info.lat_decision =
          path_planner::ObsInfo::LatDecision::LAT_IGNORE;
    }

    if (decision.HasLateralDecision() &&
        decision.LateralDecision().has_nudge() &&
        obstacle_info.lat_decision ==
            path_planner::ObsInfo::LatDecision::LAT_IGNORE) {
      continue;
    }

    if (decision.HasLongitudinalDecision() &&
        decision.LongitudinalDecision().has_follow() &&
        (obstacle_info.lon_decision !=
         path_planner::ObsInfo::LonDecision::FOLLOW)) {
      continue;
    }

    convert_type(ptr_obstacle->Type(), obstacle_info);

    const double ego_s = baseline_info->get_ego_state().ego_frenet.x;
    obstacle_info.polygon_init =
        get_polygon_at_time(0.0, ptr_obstacle, ptr_obstacle_decision, ego_s,
                            baseline_info->get_frenet_coord());

    for (int segment_index = 0; segment_index < time_at_quad_point.size();
         segment_index++) {
      std::vector<ObsPrediction> obs_prediction_polygon_list{};
      for (int quad_index = 0;
           quad_index < time_at_quad_point[segment_index].size();
           quad_index++) {
        auto obs_prediction_polygon = get_polygon_at_time(
            time_at_quad_point[segment_index][quad_index], ptr_obstacle,
            ptr_obstacle_decision, ego_s, baseline_info->get_frenet_coord());
        if (obs_prediction_polygon.polygon_fren.size() == 0) {
          continue;
        }

        obs_prediction_polygon_list.push_back(obs_prediction_polygon);
      }
      obstacle_info.polygon_list.push_back(obs_prediction_polygon_list);
    }
    obs_list.emplace_back(std::move(obstacle_info));
  }

  // create virtual obstacle for mrc inlane stop
  if (is_mrc_inlane_brake) {
    path_planner::ObsInfo obstacle_info{};
    double planning_init_v =
        baseline_info->get_ego_state().planning_init_point.v;
    double planning_init_s =
        baseline_info->get_ego_state().planning_init_point.path_point.s;
    double mrc_distance = planning_init_v * planning_init_v / (2 * 1.0);
    const double half_length = 2.5;
    const double stop_offset = 2.0;
    double virtual_obs_s =
        mrc_distance + planning_init_s + half_length + stop_offset;
    if (virtual_obs_s < baseline_info->get_frenet_coord()->GetSlength() - 1.0) {
      path_planner::ObsPrediction obstacle_prediction{};
      obstacle_prediction.polygon_fren.emplace_back(
          path_planner::Point2d{virtual_obs_s, 0.0});
      Point2D cart, fren;
      fren.x = obstacle_prediction.polygon_fren.front().x;
      fren.y = obstacle_prediction.polygon_fren.front().y;
      baseline_info->get_frenet_coord()->FrenetCoord2CartCoord(fren, cart);
      obstacle_prediction.polygon_cart.emplace_back(
          path_planner::Point2d{cart.x, cart.y});
      obstacle_prediction.rel_s =
          obstacle_prediction.polygon_fren.front().x - planning_init_s;
      obstacle_prediction.heading =
          baseline_info->get_ego_state().ego_pose.theta;
      obstacle_prediction.a = 0.0;
      obstacle_prediction.s = obstacle_prediction.polygon_fren.front().x;
      obstacle_prediction.v = 0.0;
      obstacle_prediction.v_frenet = 0.0;

      obstacle_info.id = -100;
      obstacle_info.lon_decision = path_planner::ObsInfo::LonDecision::FOLLOW;
      obstacle_info.lat_decision =
          path_planner::ObsInfo::LatDecision::LAT_IGNORE;
      obstacle_info.type = path_planner::ObsInfo::Type::COUPE;
      obstacle_info.polygon_init = obstacle_prediction;
      std::vector<ObsPrediction> obs_prediction_polygon_list{};
      for (int segment_index = 0; segment_index < time_at_quad_point.size();
           segment_index++) {
        obs_prediction_polygon_list.push_back(obstacle_prediction);
      }
      obstacle_info.polygon_list.push_back(obs_prediction_polygon_list);

      obs_list.emplace_back(std::move(obstacle_info));
    }
  }

  return obs_list;
}

} // namespace msquare
