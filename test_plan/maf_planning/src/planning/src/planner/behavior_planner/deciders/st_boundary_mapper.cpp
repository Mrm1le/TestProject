#include "planner/behavior_planner/deciders/st_boundary_mapper.h"
#include "common/config_context.h"
#include "mph_assert.h"
#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>

namespace msquare {

STBoundaryMapper::STBoundaryMapper(
    std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
    const PathData &reference_line, TrajectoryPoint init_point,
    const double planning_distance, const double planning_time)
    : frenet_coord_(frenet_coord), reference_line_(reference_line),
      init_point_(init_point), planning_max_distance_(planning_distance),
      planning_max_time_(planning_time) {}

bool STBoundaryMapper::ComputeSTBoundary(
    ObstacleManager *obs_manager,
    ObstacleDecisionManager *obs_decision_manager) const {
  mph_assert(planning_max_time_ > 0.0);

  if (reference_line_.discretized_path().size() < 2) {
    MSD_LOG(INFO,
            "Fail to get params because of too few path points. path points "
            "size: %d",
            reference_line_.discretized_path().size());
    return false;
  }

  Obstacle *stop_obstacle = nullptr;
  ObjectDecisionType stop_decision;
  double min_stop_s = std::numeric_limits<double>::max();

  for (const auto *ptr_obstacle_item : obs_manager->get_obstacles().Items()) {
    Obstacle *ptr_obstacle =
        obs_manager->find_obstacle(ptr_obstacle_item->Id());
    mph_assert(ptr_obstacle != nullptr);
    auto ptr_obstacle_decision =
        obs_decision_manager->find_obstacle_decision(ptr_obstacle->Id());
    mph_assert(ptr_obstacle_decision != nullptr);
    if (!ptr_obstacle_decision->HasLongitudinalDecision()) {
      ComputeSTBoundary(ptr_obstacle);
      continue;
    }

    const auto &decision = ptr_obstacle_decision->LongitudinalDecision();
    if (decision.has_stop()) {
      // TODO(all): store ref. s value in stop decision; refine the code then.
      const double stop_s = decision.stop().distance_s;

      if (stop_s < min_stop_s) {
        stop_obstacle = ptr_obstacle;
        min_stop_s = stop_s;
        stop_decision = decision;
      }
    } else if (decision.has_follow() || decision.has_overtake() ||
               decision.has_yield()) {
      ComputeSTBoundaryWithDecision(ptr_obstacle, decision);
    } else if (!decision.has_ignore()) {
      // std::cout << "No mapping for decision: " << std::endl;
    }
  }

  if (stop_obstacle) {
    bool success = MapStopDecision(stop_obstacle, stop_decision);
    if (!success) {
      std::string msg = "Fail to MapStopDecision.";
      // std::cerr << msg;
      return false;
    }
  }
  return true;
}

// attention here!
// distance_s in stop is not the s component of stop point's frenet coordinate
// (s,l) but relative s between stop point and min s of stop obstacle boundary
// and st_stop_s = s - ego_s - car_length/2
bool STBoundaryMapper::MapStopDecision(
    Obstacle *stop_obstacle, const ObjectDecisionType &stop_decision) const {
  mph_assert(stop_decision.has_stop());
  double stop_s = stop_decision.stop().distance_s;

  Point2D sp, sp_sl;
  sp.x = stop_decision.stop().stop_point.x();
  sp.y = stop_decision.stop().stop_point.y();
  (void)frenet_coord_->CartCoord2FrenetCoord(sp, sp_sl);

  const double stop_ref_s =
      sp_sl.x -
      ConfigurationContext::Instance()->get_vehicle_param().length / 2.0;

  auto obs_boundary = stop_obstacle->PerceptionSLBoundary();
  // s_from_perception equals stop_ref_s
  // double s_from_perception =
  // -ConfigurationContext::Instance()->get_vehicle_param().length / 2.0 +
  //    (obs_boundary.start_s + obs_boundary.end_s)/2.0 + stop_s;

  double st_stop_s = 0.0;
  if (stop_ref_s > frenet_coord_->GetLength()) {
    st_stop_s = reference_line_.discretized_path().Length();
  } else {
    st_stop_s = stop_ref_s - init_point_.path_point.s;
    // st_stop_s = stop_ref_s;
  }

  const double s_min = std::fmax(0.0, st_stop_s);
  const double s_max =
      std::fmax(s_min, std::fmax(planning_max_distance_,
                                 reference_line_.discretized_path().Length()));

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  point_pairs.emplace_back(STPoint(s_min, 0.0), STPoint(s_max, 0.0));
  point_pairs.emplace_back(
      STPoint(s_min, planning_max_time_),
      STPoint(s_max + speed_boundary_buffer, planning_max_time_));
  auto boundary = STBoundary(point_pairs);
  // std::cout << "debug_for_mapper: stop obstacle[" << stop_obstacle->Id() <<
  // "] s_min: "
  // << s_min << " s_max: " << s_max << std::endl;
  boundary.SetBoundaryType(STBoundary::BoundaryType::STOP);
  boundary.SetCharacteristicLength(speed_boundary_buffer);
  boundary.set_id(stop_obstacle->Id());
  stop_obstacle->set_path_st_boundary(boundary);
  return true;
}

void STBoundaryMapper::ComputeSTBoundary(Obstacle *obstacle) const {
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  if (obstacle->IsStBoundarySolid()) {
    return;
  }

  if (!GetOverlapBoundaryPoints(reference_line_.discretized_path(), *obstacle,
                                &upper_points, &lower_points)) {
    MSD_LOG(INFO, "not get boundary points for obstacle[%d].", obstacle->Id());
    return;
  }

  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);
  boundary.set_id(obstacle->Id());

  // TODO(all): potential bug here.
  // const auto& prev_st_boundary = obstacle->path_st_boundary();
  // const auto& ref_line_st_boundary = obstacle->reference_line_st_boundary();
  // if (!prev_st_boundary.IsEmpty()) {
  //   boundary.SetBoundaryType(prev_st_boundary.boundary_type());
  // } else if (!ref_line_st_boundary.IsEmpty()) {
  //   boundary.SetBoundaryType(ref_line_st_boundary.boundary_type());
  // }
  boundary.SetBoundaryType(STBoundary::BoundaryType::UNKNOWN);
  obstacle->set_path_st_boundary(boundary);
  obstacle->SetStBoudanrySolid(true);
}

void STBoundaryMapper::ComputeSTBoundaryWithDecision(
    Obstacle *obstacle, const ObjectDecisionType &decision) const {
  mph_assert(decision.has_follow() || decision.has_yield() ||
             decision.has_overtake());

  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  if (obstacle->IsStBoundarySolid()) {
    lower_points = obstacle->path_st_boundary().lower_points();
    upper_points = obstacle->path_st_boundary().upper_points();
    // std::cout << "debug: obstacle[" <<obstacle->Id() << "] solid size: " <<
    // lower_points.size() << std::endl;
  } else {
    if (!GetOverlapBoundaryPoints(reference_line_.discretized_path(), *obstacle,
                                  &upper_points, &lower_points)) {
      return;
    }
  }

  // std::cout << "debug: obstacle[" <<obstacle->Id() << "] final size: " <<
  // lower_points.size() << std::endl; if(lower_points.empty())
  // {
  //   std::cout << "debug: obstacle[" <<obstacle->Id() << "] empty" <<
  //   std::endl;
  // }
  mph_assert(!lower_points.empty());
  if (decision.has_follow() && lower_points.back().t() < planning_max_time_) {
    const double diff_s = lower_points.back().s() - lower_points.front().s();
    const double diff_t = lower_points.back().t() - lower_points.front().t();
    double extend_lower_s =
        diff_s / diff_t * (planning_max_time_ - lower_points.front().t()) +
        lower_points.front().s();
    const double extend_upper_s =
        extend_lower_s + (upper_points.back().s() - lower_points.back().s());
    upper_points.emplace_back(extend_upper_s, planning_max_time_);
    lower_points.emplace_back(extend_lower_s, planning_max_time_);
  }

  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);

  // get characteristic_length and boundary_type.
  STBoundary::BoundaryType b_type = STBoundary::BoundaryType::UNKNOWN;
  double characteristic_length = 0.0;
  if (decision.has_follow()) {
    // std::cout <<"debug_for_mapper: obstacle[" << obstacle->Id() <<
    //   "] follow decision" << std::endl;
    characteristic_length = std::fabs(decision.follow().distance_s);
    b_type = STBoundary::BoundaryType::FOLLOW;
  } else if (decision.has_yield()) {
    // std::cout <<"debug_for_mapper: obstacle[" << obstacle->Id() <<
    //   "] yield decision" << std::endl;
    characteristic_length = std::fabs(decision.yield().distance_s);
    boundary = STBoundary::CreateInstance(lower_points, upper_points)
                   .ExpandByS(characteristic_length);
    b_type = STBoundary::BoundaryType::YIELD;
  } else if (decision.has_overtake()) {
    // std::cout <<"debug_for_mapper: obstacle[" << obstacle->Id() <<
    //   "] overtake decision" << std::endl;
    characteristic_length = std::fabs(decision.overtake().distance_s);
    b_type = STBoundary::BoundaryType::OVERTAKE;
  } else {
    MSD_LOG(INFO, "Obj decision should be either yield or overtake: ");
  }
  boundary.SetBoundaryType(b_type);
  boundary.set_id(obstacle->Id());
  boundary.SetCharacteristicLength(characteristic_length);
  obstacle->set_path_st_boundary(boundary);
  obstacle->SetStBoudanrySolid(true);
}

bool STBoundaryMapper::GetOverlapBoundaryPoints(
    const DiscretizedPath &path_points, const Obstacle &obstacle,
    std::vector<STPoint> *upper_points,
    std::vector<STPoint> *lower_points) const {
  mph_assert(upper_points != nullptr);
  mph_assert(lower_points != nullptr);
  mph_assert(upper_points->empty());
  mph_assert(lower_points->empty());
  mph_assert(path_points.size() > 0);

  if (path_points.empty()) {
    // std::cerr << "No points in path data" << std::endl;
    return false;
  }

  const auto &trajectory = obstacle.Trajectory();
  if (trajectory.empty()) {
    if (!obstacle.IsStatic()) {
      // std::cerr << "Non-static obstacle[" << obstacle.Id()
      //           << "] has no prediction trajectory." << std::endl;
    }
    for (const auto &curr_point_on_path : path_points) {
      if (curr_point_on_path.s > planning_max_distance_) {
        break;
      }
      const planning_math::Box2d &obs_box = obstacle.PerceptionBoundingBox();

      if (CheckOverlap(curr_point_on_path, obs_box, obstacle.avdDisBuffer())) {
        const double backward_distance = -ConfigurationContext::Instance()
                                              ->get_vehicle_param()
                                              .front_edge_to_center;
        const double forward_distance = obs_box.length();
        double low_s = std::fmax(0.0, curr_point_on_path.s + backward_distance -
                                          path_points.front().s);
        double high_s = std::fmin(planning_max_distance_,
                                  curr_point_on_path.s + forward_distance -
                                      path_points.front().s);
        lower_points->emplace_back(low_s, 0.0);
        lower_points->emplace_back(low_s, planning_max_time_);
        upper_points->emplace_back(high_s, 0.0);
        upper_points->emplace_back(high_s, planning_max_time_);
        break;
      }
    }
  } else {
    const int default_num_point = 50;
    DiscretizedPath discretized_path;
    if (path_points.size() > 2 * default_num_point) {
      const auto ratio = path_points.size() / default_num_point;
      std::vector<PathPoint> sampled_path_points;
      for (size_t i = 0; i < path_points.size(); ++i) {
        if (i % ratio == 0) {
          sampled_path_points.push_back(path_points[i]);
        }
      }
      discretized_path = DiscretizedPath(sampled_path_points);
    } else {
      discretized_path = DiscretizedPath(path_points);
    }

    // construct ST polygon boundary according to obstacle prediction trajectory
    // with defined timestep
    for (int i = 0; i < trajectory.size(); ++i) {
      const TrajectoryPoint &trajectory_point = trajectory[i];
      const planning_math::Box2d obs_box =
          obstacle.GetBoundingBox(trajectory_point);

      double trajectory_point_time = trajectory_point.relative_time;
      constexpr double kNegtiveTimeThreshold = -1.0;
      if (trajectory_point_time < kNegtiveTimeThreshold) {
        continue;
      }

      // heuristically search using last trajectory point s_range
      double cur_collide_s = 0.0;
      bool find_collide_s = false;
      if (i > 0 && !lower_points->empty()) {
        double last_mid_s =
            (lower_points->back().s() + upper_points->front().s()) / 2.0;
        if (last_mid_s > 0.0 && last_mid_s < discretized_path.Length()) {
          const auto curr_adc_path_point = discretized_path.Evaluate(
              last_mid_s + discretized_path.front().s);
          if (CheckOverlap(curr_adc_path_point, obs_box,
                           obstacle.avdDisBuffer())) {
            cur_collide_s = last_mid_s;
            find_collide_s = true;
          }
        }
      }

      const double step_length = ConfigurationContext::Instance()
                                     ->get_vehicle_param()
                                     .front_edge_to_center;
      auto path_len =
          std::min(FLAGS_max_trajectory_len, discretized_path.Length());
      for (double path_s = 0.0; path_s < path_len; path_s += step_length) {
        if (find_collide_s) {
          path_s = cur_collide_s; // parasoft-suppress AUTOSAR-M6_5_3 "f-drop"
        } else {
          const auto curr_adc_path_point =
              discretized_path.Evaluate(path_s + discretized_path.front().s);
          find_collide_s = CheckOverlap(curr_adc_path_point, obs_box,
                                        obstacle.avdDisBuffer());
        }
        if (find_collide_s) {
          // found overlap, start searching with higher resolution
          // std::cout << "found overlap in " << i << "th prediction traj point"
          // << std::endl;
          const double backward_distance = -step_length;
          const double forward_distance =
              ConfigurationContext::Instance()->get_vehicle_param().length +
              ConfigurationContext::Instance()->get_vehicle_param().width +
              obs_box.length() + obs_box.width();
          const double default_min_step = 0.1; // in meters
          const double fine_tuning_step_length = std::fmin(
              default_min_step, discretized_path.Length() / default_num_point);

          bool find_low = false;
          bool find_high = false;
          double low_s = std::fmax(0.0, path_s + backward_distance);
          double high_s =
              std::fmin(discretized_path.Length(), path_s + forward_distance);

          while (low_s < high_s) {
            if (find_low && find_high) {
              break;
            }
            if (!find_low) {
              const auto &point_low =
                  discretized_path.Evaluate(low_s + discretized_path.front().s);
              if (!CheckOverlap(point_low, obs_box, obstacle.avdDisBuffer())) {
                low_s += fine_tuning_step_length;
              } else {
                find_low = true;
              }
            }
            if (!find_high) {
              const auto &point_high = discretized_path.Evaluate(
                  high_s + discretized_path.front().s);
              if (!CheckOverlap(point_high, obs_box, obstacle.avdDisBuffer())) {
                high_s -= fine_tuning_step_length;
              } else {
                find_high = true;
              }
            }
          }
          if (find_high && find_low) {
            // std::cout << "found real overlap in " << i << "th prediction traj
            // point " << "time: " << trajectory_point_time << " low_s: " <<
            // low_s << " high_s: " << high_s << std::endl;
            lower_points->emplace_back(low_s - point_extension,
                                       trajectory_point_time);
            upper_points->emplace_back(high_s + point_extension,
                                       trajectory_point_time);
          }
          break;
        }
      }
    }
  }
  mph_assert(lower_points->size() == upper_points->size());
  return lower_points->size() > 1 && upper_points->size() > 1;
}

bool STBoundaryMapper::CheckOverlap(const PathPoint &path_point,
                                    const planning_math::Box2d &obs_box,
                                    const double buffer) const {
  // planning_math::Vec2d
  // ego_center_map_frame((ConfigurationContext::Instance()->get_vehicle_param().front_edge_to_center
  // -
  //                             ConfigurationContext::Instance()->get_vehicle_param().back_edge_to_center)
  //                             *
  //                                0.5,
  //                            (ConfigurationContext::Instance()->get_vehicle_param().left_edge_to_center
  //                            -
  //                             ConfigurationContext::Instance()->get_vehicle_param().right_edge_to_center)
  //                             *
  //                                0.5);

  planning_math::Vec2d ego_center_map_frame(0.0, 0.0);

  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x);
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y);

  auto line_border = reference_line_.getWidth(path_point.s);
  double path_check_width =
      max(ConfigurationContext::Instance()->get_vehicle_param().width,
          std::abs(line_border.first - line_border.second));

  planning_math::Box2d adc_box(
      ego_center_map_frame, path_point.theta,
      ConfigurationContext::Instance()->get_vehicle_param().length,
      path_check_width + 2 * buffer);
  return obs_box.HasOverlap(adc_box);
}

} // namespace msquare
