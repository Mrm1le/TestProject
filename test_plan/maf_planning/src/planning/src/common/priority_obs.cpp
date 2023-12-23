#include "common/priority_obs.h"
#include <algorithm>

namespace msquare {

GridObsManager::GridObsManager(
    const OpenspaceDeciderOutput &openspace_decider_result) {
  if (msquare::HybridAstarConfig::GetInstance()->use_t_line_) {
    use_line_ = true;
  } else {
    use_line_ = false;
  }

  // std::cout << (use_line_ ? "use line" : "not use line") << std::endl;

  initLocal(openspace_decider_result);

  if (use_line_ && t_lines_local_.is_inited) {
    filterByArmpitRegion(openspace_decider_result);
    obs_beyond_to_check_ = line_grid_obs_[0];
    obs_within_to_check_ = line_grid_obs_[1];
  }
  obs_all_to_check_ = local_obs_all_;
}

void GridObsManager::initLocal(
    const OpenspaceDeciderOutput &openspace_decider_result) {
  std::vector<Vec2d> corners =
      openspace_decider_result.map_boundary.GetAllCorners();
  local_frame_ = Pose2D(corners[3].x(), corners[3].y(),
                        openspace_decider_result.map_boundary.heading());

  addLocalObs(openspace_decider_result);

  const TshapedAreaLines &t_lines = openspace_decider_result.T_lines;
  t_lines_local_.is_inited = t_lines.is_inited;
  t_lines_local_.road_upper_bound =
      tf2d(local_frame_, t_lines.road_upper_bound);
  t_lines_local_.road_lower_left_bound =
      tf2d(local_frame_, t_lines.road_lower_left_bound);
  t_lines_local_.slot_left_bound = tf2d(local_frame_, t_lines.slot_left_bound);
  t_lines_local_.road_lower_right_bound =
      tf2d(local_frame_, t_lines.road_lower_right_bound);
  t_lines_local_.slot_right_bound =
      tf2d(local_frame_, t_lines.slot_right_bound);

  ref_line_height_ = t_lines_local_.road_upper_bound.max_x();
}

/**
 * @brief in map_boundary: x = length = heading axis, width = y
 *
 * @param openspace_decider_result
 * @return LineSegment2d
 */
std::vector<LineSegment2d> GridObsManager::getRefLine(
    const OpenspaceDeciderOutput &openspace_decider_result) {

  // the following is for display
  std::vector<LineSegment2d> global_lines = {
      tf2d_inv(local_frame_, t_lines_local_.road_lower_left_bound),
      tf2d_inv(local_frame_, t_lines_local_.road_lower_right_bound),
      tf2d_inv(local_frame_, t_lines_local_.road_upper_bound),
      tf2d_inv(local_frame_, t_lines_local_.slot_left_bound),
      tf2d_inv(local_frame_, t_lines_local_.slot_right_bound),
      tf2d_inv(local_frame_, LineSegment2d(Vec2d(0, 0), Vec2d(5, 0)))};
  for (auto &local_b : local_box_obs_) {
    auto global_b = tf2d_inv(local_frame_, local_b);
    global_lines.push_back(LineSegment2d(global_b.GetAllCorners()[0],
                                         global_b.GetAllCorners()[2]));
  }

  return global_lines;
}

void GridObsManager::rearrangeObstacleToCheck(
    const planning_math::Vec2d &global_point) {
  Vec2d local_point = tf2d(local_frame_, global_point);
  double front_edge_to_rear_ = VehicleParam::Instance()->front_edge_to_center;
  double back_edge_to_rear_ = VehicleParam::Instance()->back_edge_to_center;
  double step_size_ = HybridAstarConfig::GetInstance()->step_size;
  double next_node_range =
      std::hypot(front_edge_to_rear_,
                 CarParams::GetInstance()->vehicle_width / 2) +
      step_size_;
  next_node_range = std::max(
      next_node_range, std::hypot(back_edge_to_rear_,
                                  CarParams::GetInstance()->vehicle_width / 2) +
                           step_size_);
  if (!use_line_) {
    obs_all_to_check_.clear();

    for (auto obs_ptr : local_obs_all_) {
      if (obs_ptr->getDistance(local_point) <= next_node_range) {
        obs_all_to_check_.push_back(obs_ptr);
      }
    }
    std::sort(obs_all_to_check_.begin(), obs_all_to_check_.end(),
              [local_point](const SbpObstaclePtr &a, const SbpObstaclePtr &b) {
                return a->getDistance(local_point) <
                       b->getDistance(local_point);
              });
    return;
  }

  obs_beyond_to_check_.clear();
  obs_within_to_check_.clear();

  for (auto obs_ptr : line_grid_obs_[0]) {
    if (obs_ptr->getDistance(local_point) <= next_node_range) {
      obs_beyond_to_check_.push_back(obs_ptr);
    }
  }
  for (auto obs_ptr : line_grid_obs_[1]) {
    if (obs_ptr->getDistance(local_point) <= next_node_range) {
      obs_within_to_check_.push_back(obs_ptr);
    }
  }
  std::sort(obs_beyond_to_check_.begin(), obs_beyond_to_check_.end(),
            [local_point](const SbpObstaclePtr &a, const SbpObstaclePtr &b) {
              return a->getDistance(local_point) < b->getDistance(local_point);
            });
  std::sort(obs_within_to_check_.begin(), obs_within_to_check_.end(),
            [local_point](const SbpObstaclePtr &a, const SbpObstaclePtr &b) {
              return a->getDistance(local_point) < b->getDistance(local_point);
            });
}

bool GridObsManager::checkCollision(const SearchNodePtr &node,
                                    const FootprintModelPtr &footprint_model) {
  Pose2D global_pose(node->x, node->y, node->theta);
  Pose2D local_pose = tf2d(local_frame_, global_pose);
  footprint_model->updatePose(local_pose);

  SearchNodePtr local_node = std::make_shared<SearchNode>(
      local_pose.x, local_pose.y, local_pose.theta);

  bool is_collision;
  if (use_line_ && t_lines_local_.is_inited) {
    is_collision = checkCollisionWithLineGrid(local_node, footprint_model);
  } else {
    is_collision = checkCollisionNoGrid(local_node, footprint_model);
  }

  return is_collision;
}

bool GridObsManager::overLine1(const FootprintModelPtr &footprint_model) {

  if (footprint_model->max_x() < ref_line_height_) {
    return false;
  }

  return true;
}

bool GridObsManager::checkCollisionWithLineGrid(
    const SearchNodePtr &node, const FootprintModelPtr &footprint_model) {

  if (overLine1(footprint_model)) {
    for (auto &obs : obs_beyond_to_check_) {
      if (obs->checkCollision(node, footprint_model)) {
        return true;
      }
    }
  }

  for (auto &obs : obs_within_to_check_) {
    if (obs->checkCollision(node, footprint_model)) {
      return true;
    }
  }

  return false;
}

double GridObsManager::getCost(const SearchNodePtr &node,
                               const FootprintModelPtr &footprint_model) {
  Pose2D global_pose(node->x, node->y, node->theta);
  Pose2D local_pose = tf2d(local_frame_, global_pose);
  SearchNodePtr local_node = std::make_shared<SearchNode>(
      local_pose.x, local_pose.y, local_pose.theta);

  double cost = 0.0;
  for (SbpObstaclePtr &obs_ptr : local_obs_all_) {
    cost += obs_ptr->getCost(local_node, footprint_model);
  }
  return cost;
}

double GridObsManager::getDistance(const Vec2d &point) {
  Vec2d local_p = tf2d(local_frame_, point);
  double min_dist = std::numeric_limits<double>::infinity();
  for (const SbpObstaclePtr &obs_ptr : local_obs_all_) {
    min_dist = std::min(min_dist, obs_ptr->getDistance(local_p));
  }
  return min_dist;
}
std::vector<Vec2d>
GridObsManager::getNearestPoints(const LineSegment2d &ego_centerline) {
  LineSegment2d local_ego_centerline = tf2d(local_frame_, ego_centerline);
  std::vector<Vec2d> nearest_pts_all;
  for (const SbpObstaclePtr &obs_ptr : local_obs_all_) {
    std::vector<Vec2d> nearest_pts =
        obs_ptr->getNearestPoints(local_ego_centerline);
    nearest_pts_all.insert(nearest_pts_all.end(), nearest_pts.begin(),
                           nearest_pts.end());
  }
  return nearest_pts_all;
}

void GridObsManager::addLocalObs(
    const OpenspaceDeciderOutput &openspace_decider_result) {
  Box2d local_map_boundary =
      tf2d(local_frame_, openspace_decider_result.map_boundary);

  for (auto &p : openspace_decider_result.points) {
    auto local_p = tf2d(local_frame_, p);
    if (local_map_boundary.IsPointIn(local_p)) {
      local_point_obs_.push_back(local_p);
    }
  }
  local_obs_all_.push_back(
      std::make_shared<SbpObstaclePoint>(local_point_obs_));

  for (auto &l : openspace_decider_result.lines) {
    auto local_l = tf2d(local_frame_, l);
    if (local_map_boundary.HasOverlap(local_l)) {
      local_mapline_obs_.push_back(local_l);
    }
  }
  local_obs_all_.push_back(std::make_shared<SbpMapLine>(local_mapline_obs_));

  for (auto &l : openspace_decider_result.obstacle_lines) {
    auto local_l = tf2d(local_frame_, l);
    if (local_map_boundary.HasOverlap(local_l)) {
      local_line_obs_.push_back(local_l);
    }
  }
  local_obs_all_.push_back(std::make_shared<SbpObstacleLine>(local_line_obs_));

  for (auto &b : openspace_decider_result.obstacle_boxs) {
    auto local_b = tf2d(local_frame_, b);
    if (local_map_boundary.HasOverlap(local_b)) {
      local_box_obs_.push_back(local_b);
      local_obs_all_.push_back(std::make_shared<SbpObstacleBox>(local_b));
    }
  }
}

void GridObsManager::filterByArmpitRegion(
    const OpenspaceDeciderOutput &openspace_decider_result) {
  double local_ref_line_height = t_lines_local_.road_upper_bound.min_x();

  double l_max_x, l_min_x, l_max_y, l_min_y, r_max_x, r_min_x, r_max_y, r_min_y;
  l_max_x = std::max(t_lines_local_.road_lower_left_bound.max_x(),
                     t_lines_local_.slot_left_bound.max_x());
  l_min_x = std::min(t_lines_local_.road_lower_left_bound.min_x(),
                     t_lines_local_.slot_left_bound.min_x());
  l_max_y = std::max(t_lines_local_.road_lower_left_bound.max_y(),
                     t_lines_local_.slot_left_bound.max_y());
  l_min_y = std::min(t_lines_local_.road_lower_left_bound.min_y(),
                     t_lines_local_.slot_left_bound.min_y());
  r_max_x = std::max(t_lines_local_.road_lower_right_bound.max_x(),
                     t_lines_local_.slot_right_bound.max_x());
  r_min_x = std::min(t_lines_local_.road_lower_right_bound.min_x(),
                     t_lines_local_.slot_right_bound.min_x());
  r_max_y = std::max(t_lines_local_.road_lower_right_bound.max_y(),
                     t_lines_local_.slot_right_bound.max_y());
  r_min_y = std::min(t_lines_local_.road_lower_right_bound.min_y(),
                     t_lines_local_.slot_right_bound.min_y());

  auto is_pt_in_region = [l_max_x, l_min_x, l_max_y, l_min_y, r_max_x, r_min_x,
                          r_max_y, r_min_y](Vec2d p) {
    if (p.x() < l_max_x && p.x() > l_min_x && p.y() < l_max_y &&
        p.y() > l_min_y) {
      return true;
    }

    if (p.x() < r_max_x && p.x() > r_min_x && p.y() < r_max_y &&
        p.y() > r_min_y) {
      return true;
    }

    return false;
  };

  auto is_line_box_in_region = [l_max_x, l_min_x, l_max_y, l_min_y, r_max_x,
                                r_min_x, r_max_y, r_min_y](auto &shape) {
    if (shape.max_x() < l_max_x && shape.min_x() > l_min_x &&
        shape.max_y() < l_max_y && shape.min_y() > l_min_y) {
      return true;
    }

    if (shape.max_x() < r_max_x && shape.min_x() > r_min_x &&
        shape.max_y() < r_max_y && shape.min_y() > r_min_y) {
      return true;
    }

    return false;
  };

  std::vector<std::vector<Vec2d>> point_obs(2, std::vector<Vec2d>());
  std::vector<std::vector<LineSegment2d>> line_obs(
      2, std::vector<LineSegment2d>());
  std::vector<std::vector<LineSegment2d>> mapline_obs(
      2, std::vector<LineSegment2d>());
  std::vector<std::vector<Box2d>> box_obs(2, std::vector<Box2d>());
  line_grid_obs_.resize(2, std::vector<SbpObstaclePtr>());

  for (auto &p : local_point_obs_) {
    if (is_pt_in_region(p)) {
      continue;
    }

    if (p.x() > local_ref_line_height) {
      point_obs[0].push_back(p);
      continue;
    }

    point_obs[1].push_back(p);
  }

  for (auto &l : local_line_obs_) {
    if (is_line_box_in_region(l)) {
      continue;
    }

    double p_max = l.max_x();
    double p_min = l.min_x();

    if (p_min > local_ref_line_height) {
      line_obs[0].push_back(l);
      continue;
    }
    line_obs[1].push_back(l);
  }

  for (auto &l : local_mapline_obs_) {
    if (is_line_box_in_region(l)) {
      continue;
    }
    double p_max = l.max_x();
    double p_min = l.min_x();

    if (p_min > local_ref_line_height) {
      mapline_obs[0].push_back(l);
      continue;
    }
    mapline_obs[1].push_back(l);
  }

  for (auto &b : local_box_obs_) {
    if (is_line_box_in_region(b)) {
      continue;
    }

    double p_max = b.max_x();
    double p_min = b.min_x();

    if (p_min > local_ref_line_height) {
      box_obs[0].push_back(b);
      continue;
    }
    box_obs[1].push_back(b);
  }

  //  add four lines
  line_obs[1].push_back(t_lines_local_.road_lower_left_bound);
  line_obs[1].push_back(t_lines_local_.road_lower_right_bound);
  line_obs[1].push_back(t_lines_local_.slot_left_bound);
  line_obs[1].push_back(t_lines_local_.slot_right_bound);

  // combine obs
  for (int i = 0; i < 2; i++) {
    if (mapline_obs[i].size() > 0) {
      line_grid_obs_[i].push_back(std::make_shared<SbpMapLine>(mapline_obs[i]));
    }

    if (point_obs[i].size() > 0) {
      line_grid_obs_[i].push_back(
          std::make_shared<SbpObstaclePoint>(point_obs[i]));
    }

    if (box_obs[i].size() > 0) {
      for (auto one_box : box_obs[i]) {
        line_grid_obs_[i].push_back(std::make_shared<SbpObstacleBox>(one_box));
      }
    }

    if (line_obs[i].size() > 0) {
      line_grid_obs_[i].push_back(
          std::make_shared<SbpObstacleLine>(line_obs[i]));
    }
  }
}

bool GridObsManager::checkCollisionNoGrid(
    const SearchNodePtr &current_state,
    const FootprintModelPtr &footprint_model) {
  for (const SbpObstaclePtr &obs_ptr : obs_all_to_check_) {
    if (obs_ptr->checkCollision(current_state, footprint_model)) {
      return true;
    }
  }

  return false;
}

} // end namespace msquare