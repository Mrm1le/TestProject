#pragma once

#include "common/parking_planner_types.h"
#include "search_based_planner.h"

namespace msquare {

class PatternPlanner : public SearchBasedPlanner {
private:
  std::string pattern_file_;

public:
  PatternPlanner(const std::string &pattern_file);
  ~PatternPlanner();

  /**
   * @brief Update and re-init SearchBasedPlanner.
   * @param map_boundary planning boundary
   * @param map obstacles in the current environment
   * @return no param
   */
  virtual void Update(const planning_math::Box2d map_boundary,
                      const std::vector<planning_math::LineSegment2d> &map =
                          std::vector<planning_math::LineSegment2d>());

  /** @name Plan a trajectory */

  /**
   * @brief Plan a trajectory based on an initial reference plan.
   * @param obstacles in the current environment
   * @return \c true if planning was successful, \c false otherwise
   */
  bool Plan(const std::vector<SbpObstaclePtr> &obs_ptrs,
            parking::SearchProcessDebug *sp_debug = nullptr);

  /**
   * @brief get the planned trajectory
   * @param no param
   */

  virtual SbpResult getResult();

  /**
   * @brief for visualization,get all the searchpoints
   * @param no param
   */

  virtual std::vector<Pose2D> getSearchPoints();
};

} // namespace msquare
