#ifndef DECISION_PLANNING_DECIDER_ST_BOUNDARY_MAP_H_
#define DECISION_PLANNING_DECIDER_ST_BOUNDARY_MAP_H_

#include <string>
#include <vector>

#include "common/obstacle.h"
#include "common/obstacle_decision_manager.h"
#include "common/obstacle_manager.h"
#include "common/path/path_data.h"
#include "common/speed/speed_limit.h"
#include "common/world_model.h"
#include "planner/message_type.h"

namespace msquare {

class STBoundaryMapper {
public:
  STBoundaryMapper(std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
                   const PathData &reference_line, TrajectoryPoint init_point,
                   const double planning_distance, const double planning_time);

  ~STBoundaryMapper() = default;

  bool ComputeSTBoundary(ObstacleManager *obs_manager,
                         ObstacleDecisionManager *obs_decision_manager) const;

  bool CheckOverlap(const PathPoint &path_point,
                    const planning_math::Box2d &obs_box,
                    const double buffer) const;

  void ComputeSTBoundary(Obstacle *obstacle) const;

private:
  // bool CheckOverlap(const PathPoint& path_point, const planning_math::Box2d&
  // obs_box,
  //                     const double buffer) const;

  /**
   * Creates valid st boundary upper_points and lower_points
   * If return true, upper_points.size() > 1 and
   * upper_points.size() = lower_points.size()
   */
  bool GetOverlapBoundaryPoints(const DiscretizedPath &path_points,
                                const Obstacle &obstacle,
                                std::vector<STPoint> *upper_points,
                                std::vector<STPoint> *lower_points) const;

  // void ComputeSTBoundary(Obstacle* obstacle) const;

  bool MapStopDecision(Obstacle *stop_obstacle,
                       const ObjectDecisionType &decision) const;

  void ComputeSTBoundaryWithDecision(Obstacle *obstacle,
                                     const ObjectDecisionType &decision) const;

private:
  // const SpeedBoundsDeciderConfig& speed_bounds_config_;
  // const ReferenceLine& reference_line_;
  // const PathData& path_data_;
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;
  PathData reference_line_;
  TrajectoryPoint init_point_;
  const double planning_max_distance_;
  const double planning_max_time_;
  const double speed_boundary_buffer = 0.3;
  const double point_extension = 1.0;
  const double FLAGS_max_trajectory_len = 120.0;
};

} // namespace msquare

#endif // DECISION_PLANNING_DECIDER_ST_BOUNDARY_MAP_H_
