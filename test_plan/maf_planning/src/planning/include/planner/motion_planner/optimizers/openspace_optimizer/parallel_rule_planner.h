#include "common/config/vehicle_param.h"
#include "common/math/polygon2d.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/clothoid_solver.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/collision_checker.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/parallel_path_segments.h"
#include "search_based_planner.h"

namespace msquare {

class ParallelRulePlanner : public SearchBasedPlanner {

public:
  ParallelRulePlanner(const parking::OpenspaceDeciderOutput &odo);
  ~ParallelRulePlanner() { LOCAL_LOG(LOCAL_INFO, "quit planner"); }

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
  void addSideVirtualWall(std::vector<planning_math::Vec2d> &obs_pts);
  void refactorRightCorner(std::vector<planning_math::Vec2d> &obs_pts);

  // in process judgement
  bool checkStartEndPose();
  bool isInSlot(const Pose2D &pose);
  bool isInHalfSlot(const Pose2D &pose);

  /* find escape pose */
  bool findEscapePose(Pose2D &escape_pose, InSlotPathSegemnts &path_segments);
  bool findEscapePoseV2(Pose2D &escape_pose, InSlotPathSegemnts &path_segments);


  /* out slot plan */
  bool planOneTry(const Pose2D &escape_pose, const Pose2D &init_pose,int block_direc);
  bool planOutslot(const Pose2D &escape_pose, const Pose2D &init_pose,
                   OutSlotPathSegments &opti_path_segments, int block_direc);
  bool spiralCscPath(const Pose2D &escape_pose, const Pose2D &start_pose,
                     OutSlotPathSegments &path_segments,
                     OutSlotAdjustPath &last_segments, double r2,
                     int block_direc);
  bool adjustOutslotCP(const Pose2D &start_pose,
                       std::vector<OutSlotAdjustPath> &path_segments);
  bool adjustOutSlotCPPCs(const Pose2D &ego_pose,
                          const OutSlotAdjustPath &last_segments,
                          std::vector<OutSlotAdjustPath> &path_segments);
  bool adjustOutSlotCPPC(const Pose2D &ego_pose,
                         const OutSlotAdjustPath &last_segments,
                         OutSlotAdjustPath &path_segments, double r,
                         double move_height);
  bool adjustOutslotLargeC(const Pose2D &ego_pose,
                           std::vector<OutSlotAdjustPath> &path_segments);
  bool adjustOutslotSPCP(const Pose2D &start_pose,
                         std::vector<OutSlotAdjustPath> &path_segments);

  /***********************************/
  /********** in slot plan **********/
  /***********************************/
  bool planInSlot(std::vector<Pose2D> &in_slot_path);
  bool inslotCSPath(InSlotPathSegemnts &path_segments, const Pose2D &pose,
                    bool is_init, int block_direc, bool is_final);
  bool planInSlotCurve(InSlotPathSegemnts &path_segments, const Pose2D &pose,
                       bool is_init, int block_direc, bool is_final);
  bool planInSlotKeepStraight(InSlotPathSegemnts &path_segments,
                              const Pose2D &pose, bool is_init,
                              int block_direc);
  bool planInSlotOffset(std::vector<Pose2D> &local_planning_key_points, 
                        const Pose2D &pose, double min_step_size, 
                        double theta_epsilon, double max_offset_desired, 
                        bool is_init, int block_direc);
  bool planInSlotHorizon(std::vector<Pose2D> &local_planning_key_points, 
                        const Pose2D &pose, double min_step_size, 
                        double theta_epsilon, double max_offset_desired,
                        bool is_init, int block_direc);
  double getMaxOffsetCover(const Pose2D &pose, double min_step_size, 
                        double theta_epsilon);
  void getStartPoseAndDirect(const std::vector<Pose2D> &local_planning_key_points, 
                        const double min_step_size, const Pose2D &pose, 
                        int &path_block_direc, double &added_dis);
  bool isEscapable(const Pose2D &start_pose, const int steer_dirct, 
                    const int travel_direct, const bool is_init,
                    double obs_distance);
  bool getRealRotatePose(const Pose2D &start_pose, const double min_step_size, 
                         const double theta_epsilon, Pose2D &next_pose);
  bool getRightBackwardPose(const Pose2D &start_pose, const double min_step_size, 
                            const double theta_epsilon, Pose2D &next_pose);
  bool getLeftForwardPose(const Pose2D &start_pose, const double min_step_size, 
                            const double theta_epsilon, Pose2D &next_pose);

  /* util functions */
  void fillPara(const parking::OpenspaceDeciderOutput &odo,
                clothoid::Parameter &para);
  void generatePath(const std::vector<Pose2D> &local_path);
  void getGlobalPath(std::vector<Pose2D> &global_path,
                     const std::vector<Pose2D> &local_path,
                     const Pose2D &local_frame);
  double calcPose2DDistance(const Pose2D &pose1, const Pose2D &pose2);

  /* data */
  double veh_min_r = 5.015;
  bool is_on_left = false;
  bool is_front_empty_ = false; // there is no obstacle at the front of the slot

  double inflation_for_points_;

  double in_slot_x_ = 0.5;
  double in_slot_y_ = 1.0;
  double lon_left_dis_ = 0.03; // m
  double corner_safe_distance_ = 0.3; // m

  double max_steer_;

  std::vector<Pose2D> key_points_;

  Pose2D init_pose_;
  Pose2D target_pose_;

  // clothoid
  ParallelImpl impl_;        // generated parameter
  clothoid::Parameter para_; // constant parameter
  clothoid::CollisionChecker checker_;
  clothoid::CollisionShapeGenerator csg_;
  clothoid::ClothoidSolver clo_solver_;
};

} // namespace msquare