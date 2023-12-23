#include "planner/behavior_planner/deciders/ground_line_decider.h"
namespace msquare {
namespace parking {

bool GroundLineDecider::update_params(const int min_pts, const double eps) {
  min_pts_ = min_pts;
  eps_ = eps;

  return true;
}

std::vector<std::vector<planning_math::Vec2d>> GroundLineDecider::execute(
    const std::vector<FusionFreespacePoint> &ground_line_points) {
  std::vector<std::vector<planning_math::Vec2d>> result;
  (void)update_points(ground_line_points);
  for (auto &point : points_) {
    if (point.status == GroundLinePoint::Status::UNCLASSIFIED) {
      std::vector<planning_math::Vec2d> cluster = expand_cluster(point);
      if (cluster.size() > 0) {
        result.emplace_back(cluster);
      }
    }
  }
  return result;
}

bool GroundLineDecider::update_points(
    const std::vector<FusionFreespacePoint> &ground_line_points) {
  points_.clear();
  for (auto &pt : ground_line_points) {
    GroundLinePoint point;
    point.point = planning_math::Vec2d(pt.position.x, pt.position.y);
    point.status = GroundLinePoint::Status::UNCLASSIFIED;
    points_.emplace_back(point);
  }

  return true;
}

std::vector<int> GroundLineDecider::calc_cluster(GroundLinePoint &point) {
  std::vector<int> cluster_index;

  for (int i = 0; i < points_.size(); i++) {
    if (point != points_[i] && calc_distance(point, points_[i]) <= eps_) {
      cluster_index.emplace_back( // parasoft-suppress AUTOSAR-M6_5_3 "f-drop"
          i);                     // parasoft-suppress AUTOSAR-M6_5_3 "f-drop"
    }
  }
  return cluster_index;
}

std::vector<planning_math::Vec2d>
GroundLineDecider::expand_cluster(GroundLinePoint &point) {
  std::vector<planning_math::Vec2d> result;
  result.clear();

  std::vector<int> cluster = calc_cluster(point);
  if (cluster.size() + 1 < min_pts_) {
    point.status = GroundLinePoint::Status::NOISE;
  } else {
    point.status = GroundLinePoint::Status::CLASSIFIED;
    result.emplace_back(point.point);
    for (int i = 0; i < cluster.size(); i++) {
      std::vector<int> cluster_exp = calc_cluster(points_.at(cluster[i]));
      points_.at(cluster[i]).status = GroundLinePoint::Status::CLASSIFIED;
      result.emplace_back(points_.at(cluster[i]).point);

      if (cluster_exp.size() >= min_pts_) {
        for (int j = 0; j < cluster_exp.size(); j++) {
          if (points_.at(cluster_exp[j]).status ==
              GroundLinePoint::Status::UNCLASSIFIED) {
            if (find(cluster.begin(), cluster.end(), cluster_exp[j]) ==
                cluster.end()) {
              cluster.push_back(cluster_exp[j]);
            }
          } else if (points_.at(cluster_exp[j]).status ==
                     GroundLinePoint::Status::NOISE) {
            points_.at(cluster_exp[j]).status =
                GroundLinePoint::Status::CLASSIFIED;
            result.emplace_back(points_.at(cluster_exp[j]).point);
          }
        }
      }
    }
  }

  return result;
}

inline double GroundLineDecider::calc_distance(const GroundLinePoint &point1,
                                               const GroundLinePoint &point2) {
  return point1.point.DistanceTo(point2.point);
}

} // namespace parking
} // namespace msquare