#include <unordered_map>

#include "common/config/vehicle_param.h"
#include "common/math/polygon2d.h"
#include "common/parking_planner_types.h"
#include "search_based_planner.h"

namespace msquare {

class PerpendicularOutRulePlanner : public SearchBasedPlanner {

public:
  PerpendicularOutRulePlanner(const parking::OpenspaceDeciderOutput &odo);
  ~PerpendicularOutRulePlanner() {}

  virtual void Update(const planning_math::Box2d map_boundary,
                      const std::vector<planning_math::LineSegment2d> &map =
                          std::vector<planning_math::LineSegment2d>());
  virtual bool Plan(const std::vector<SbpObstaclePtr> &obs_ptrs,
                    parking::SearchProcessDebug *sp_debug = nullptr);
  virtual SbpResult getResult();
  virtual std::vector<Pose2D> getSearchPoints();

private:
  bool PlanDirectOut(std::vector<Pose2D> &path_to_fill);

  bool PlanWithBuffer(double safe_buffer, int max_zigzag);
  bool tooNearUpperWall(const Pose2D &turning_pose,
                        const planning_math::Vec2d &center);
  bool startWithStraightLine(const Pose2D &start_pose, double safe_buffer,
                             int max_zigzag);
  bool calcAdjustPoses(const Pose2D &start_pose, const Pose2D &end_pose,
                       double radius);
  bool startWithSweetPose(const Pose2D &start_pose, double safe_buffer,
                          int max_zigzag);
  bool calcCurvePath(const Pose2D &start_pose, int max_zigzag);

  void extractKeyInfo(const parking::OpenspaceDeciderOutput &input);
  void extendKeyPointsToResult(double step_size);

  Pose2D calcTurningPose(const Pose2D &start_pose, double safe_buffer,
                         planning_math::Vec2d &arc_center);

  Pose2D calcExpectedPose(const Pose2D &start_pose, int steer_direction,
                          int travel_direction);
  Pose2D calcExpectedPoseReverse(const Pose2D &start_pose, int steer_direction,
                                 int travel_direction, double radius);

  /* data */
  double veh_min_r;
  bool is_on_left = false;

  double half_width;
  double front_to_rear;
  double back_to_rear;
  double front_corner_width = 0.0;
  double front_corner_length = 0.0;
  double inflation_for_points_;

  double l_1;
  double l_2;
  double gamma_4;

  double slot_width_;
  std::vector<std::pair<double, int>> buffer_zigzag_pairs_{
      {0.15, 4}, {0.16, 4}, {0.17, 4}, {0.18, 4}, {0.19, 4}, {0.20, 4},
      {0.21, 4}, {0.22, 4}, {0.23, 4}, {0.24, 4}, {0.25, 4}, {0.26, 4},
      {0.27, 3}, {0.28, 2}, {0.31, 1}, {0.34, 1}, {0.37, 1}, {0.40, 1}};
  std::unordered_map<double, std::vector<Pose2D>> sweet_spot_;

  std::vector<Pose2D> key_points_;

  Pose2D init_pose;
  double init_vel_;
  Pose2D target_pose;

  double upper_height_;
  planning_math::Vec2d P_0_; // slot_left_corner_
  planning_math::Vec2d P_1_; // slot_right_corner_

  std::vector<planning_math::LineSegment2d> obstacles;
  std::vector<planning_math::Vec2d> points_of_obstacles;
};
} // namespace msquare