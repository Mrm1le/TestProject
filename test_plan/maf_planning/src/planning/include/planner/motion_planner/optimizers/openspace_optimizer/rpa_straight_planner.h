#include "common/config/vehicle_param.h"
#include "common/math/polygon2d.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/collision_checker.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/parallel_path_segments.h"
#include "search_based_planner.h"

namespace msquare {

class RpaStraightPlanner : public SearchBasedPlanner {

public:
  RpaStraightPlanner(const parking::OpenspaceDeciderOutput &odo);
  ~RpaStraightPlanner() {}

  virtual void Update(const planning_math::Box2d map_boundary,
                      const std::vector<planning_math::LineSegment2d> &map =
                          std::vector<planning_math::LineSegment2d>());
  virtual bool Plan(const std::vector<SbpObstaclePtr> &obs_ptrs,
                    parking::SearchProcessDebug *sp_debug = nullptr);
  virtual SbpResult getResult();
  virtual std::vector<Pose2D> getSearchPoints();

private:
  // preprocess
  void initScenario(const parking::OpenspaceDeciderOutput &input);

  // plan
  bool planStraightPath(clothoid::StraightCurve &straight_curve);
  bool checkStartPose();

  /* util functions */
  void fillPara(const parking::OpenspaceDeciderOutput &odo,
                clothoid::Parameter &para);
  void generatePath(const clothoid::StraightCurve &straight_curve);

  Pose2D init_pose_;
  Pose2D target_pose_;
  std::vector<Pose2D> key_points_; // not used

  ParallelImpl impl_;        // generated parameter
  clothoid::Parameter para_; // constant parameter
  clothoid::CollisionChecker checker_;
  clothoid::CollisionShapeGenerator csg_;
};

} // namespace msquare