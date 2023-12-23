#ifndef MSQUARE_PRIORITY_OBS
#define MSQUARE_PRIORITY_OBS

#include "common/math/box2d.h"
#include "common/parking_planner_types.h"
#include "common/sbp_map_line.h"
#include "common/sbp_obstacle_box.h"
#include "common/sbp_obstacle_line.h"
#include "common/sbp_obstacle_point.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/State.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/openspace_footprint_model.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/sbp_obstacle_interface.h"

#include <iostream>

namespace msquare {

using namespace planning_math;
using OpenspaceDeciderOutput = parking::OpenspaceDeciderOutput;
using TshapedAreaLines = parking::TshapedAreaLines;

// m2
class GridObsManager : public SbpObstacleInterface {
public:
  GridObsManager(const OpenspaceDeciderOutput &openspace_decider_result);

  ~GridObsManager() {}

  virtual double getCost(const SearchNodePtr &node,
                         const FootprintModelPtr &footpint_model);
  virtual bool checkCollision(const SearchNodePtr &node,
                              const FootprintModelPtr &footpint_model);
  virtual double getDistance(const planning_math::Vec2d &point);
  virtual std::vector<planning_math::Vec2d>
  getNearestPoints(const planning_math::LineSegment2d &ego_centerline);

  /**
   * @brief Get ref line for display, this should not be called in real planning
   *
   * @param openspace_decider_result
   * @return std::vector<planning_math::LineSegment2d>
   */
  std::vector<planning_math::LineSegment2d>
  getRefLine(const OpenspaceDeciderOutput &openspace_decider_result);

  void rearrangeObstacleToCheck(const planning_math::Vec2d &global_point);

private:
  void initLocal(const OpenspaceDeciderOutput &openspace_decider_result);

  /**
   * @brief add all obstacle and transform to local
   *
   * @param openspace_decider_result
   */
  void addLocalObs(const OpenspaceDeciderOutput &openspace_decider_result);
  void
  filterByArmpitRegion(const OpenspaceDeciderOutput &openspace_decider_result);

  bool checkCollisionNoGrid(const SearchNodePtr &node,
                            const FootprintModelPtr &footprint_model);
  bool checkCollisionWithLineGrid(const SearchNodePtr &node,
                                  const FootprintModelPtr &footprint_model);

  bool overLine1(const FootprintModelPtr &footprint_model);

private:
  std::vector<planning_math::Vec2d> local_point_obs_;
  std::vector<planning_math::LineSegment2d> local_line_obs_;
  std::vector<planning_math::LineSegment2d> local_mapline_obs_;
  std::vector<planning_math::Box2d> local_box_obs_;

  std::vector<SbpObstaclePtr> obs_beyond_to_check_;
  std::vector<SbpObstaclePtr> obs_within_to_check_;
  std::vector<SbpObstaclePtr> obs_all_to_check_;

  std::vector<std::vector<SbpObstaclePtr>> line_grid_obs_;
  std::vector<SbpObstaclePtr> local_obs_all_;

  double ref_line_height_;
  bool use_line_;
  bool start_point_above_middle_;

  Pose2D local_frame_;
  TshapedAreaLines t_lines_local_;
  Vec2d local_start_point_;
};

typedef std::shared_ptr<GridObsManager> GridObsManagerPtr;

} // end namespace msquare

#endif