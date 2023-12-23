#include "common/config/vehicle_param.h"
#include "common/math/polygon2d.h"
#include "search_based_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/local_log.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/collision_checker.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/parallel_path_segments.h"

namespace msquare {

class ParallelOutRulePlanner : public SearchBasedPlanner {

public:
  ParallelOutRulePlanner(const parking::OpenspaceDeciderOutput &odo);
  ~ParallelOutRulePlanner() {}

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
  void refactorRightCorner(std::vector<planning_math::Vec2d> &obs_pts,
                           std::vector<planning_math::Vec2d> &step_pts);

  // plan
  bool planParkoutHalf(InSlotPathSegemnts &path_segments);
  bool planDiffRadius(const double backward_max, const std::vector<double>& radiu_list, 
                      InSlotPathSegemnts &path_segments);
  bool isParkoutSuccess(const Pose2D &start_pose, const double radiu_rotate,
                        const bool is_init, Pose2D &escape_pose, bool &valid);
  bool isParkoutForLowerTline(const Pose2D &start_pose, const double radiu_rotate,
                              const bool is_init, Pose2D &escape_pose, bool &valid);
  bool isEscapable(const Pose2D &key_pose, Pose2D &escape_pose, bool &valid,
                   const bool is_init);
  bool isEscapableForLowerTline(const Pose2D &key_pose, Pose2D &escape_pose,
                                bool &valid, const bool is_init);
  bool isEscapableStraight(const Pose2D &key_pose, Pose2D &escape_pose,
                           bool &valid, const bool is_init);
  bool checkStartPose();
  bool getLeftForwardPose(const Pose2D &start_pose, const double radiu_rotate,
                          const bool is_init, Pose2D &next_pose);
  bool getRealRotatePose(const Pose2D &start_pose, const double radiu_rotate,
                         Pose2D &next_pose);
  bool getRightBackwardPose(const Pose2D &start_pose, const double radiu_rotate, 
                            const bool is_init, Pose2D &next_pose);
  bool getBackwardPose(const Pose2D &start_pose, const double radiu_rotate,
                       const double lat, const double lon, const bool is_init,
                       Pose2D &next_pose);
  std::vector<std::pair<double, std::vector<double>>> getBackdisRadiusKv();

  /* util functions */
  void fillPara(const parking::OpenspaceDeciderOutput &odo,
                clothoid::Parameter &para);
  bool isInSlot(const Pose2D &pose);
  void generatePath(const double radiu_used, InSlotPathSegemnts &path_segments);
  /* data */
  bool park_half_;
  double radiu_used_;
  std::vector<Pose2D> key_points_;

  Pose2D init_pose_;
  Pose2D target_pose_;

  ParallelParkoutImpl impl_;  // generated parameter
  clothoid::Parameter para_; // constant parameter
  clothoid::CollisionChecker checker_;
  clothoid::CollisionShapeGenerator csg_;
};

} // namespace msquare