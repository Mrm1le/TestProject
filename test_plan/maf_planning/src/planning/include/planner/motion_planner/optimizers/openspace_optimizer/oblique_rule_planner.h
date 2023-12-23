#include <unordered_map>

#include "common/config/vehicle_param.h"
#include "common/math/polygon2d.h"
#include "common/parking_planner_types.h"
#include "search_based_planner.h"

namespace msquare {

class ObliqueRulePlanner : public SearchBasedPlanner {

public:
  ObliqueRulePlanner(const parking::OpenspaceDeciderOutput &odo);
  ~ObliqueRulePlanner() {}

  virtual void Update(const planning_math::Box2d map_boundary,
                      const std::vector<planning_math::LineSegment2d> &map =
                          std::vector<planning_math::LineSegment2d>());
  virtual bool Plan(const std::vector<SbpObstaclePtr> &obs_ptrs,
                    parking::SearchProcessDebug *sp_debug = nullptr);
  virtual SbpResult getResult();
  virtual std::vector<Pose2D> getSearchPoints();

private:
  bool PlanInSlot();

  bool PlanInSlotSub_1(std::vector<Pose2D> &path_to_fill);
  bool PlanInSlotSub0(std::vector<Pose2D> &path_to_fill);
  void PlanInSlotSub1(std::vector<Pose2D> &path_to_fill, double ratio); // CCS
  void PlanInSlotSub2(std::vector<Pose2D> &path_to_fill, double ratio); // CCCCS

  bool isMidPlan();
  bool planMidway(const Pose2D &mid_pose, bool is_reverse);
  bool planMidway2(const Pose2D &mid_pose, bool is_reverse);
  bool planRestartWay(const Pose2D &mid_pose);

  bool PlanWithBuffer(double safe_buffer, int max_zigzag);

  bool planOneShot(const Pose2D &start_pose);
  bool startWithStraightLine(const Pose2D &start_pose, double safe_buffer,
                             int max_zigzag);

  bool calcAdjustPoses(const Pose2D &end_pose, double radius);
  bool startWithSweetPose(double safe_buffer, int max_zigzag);
  bool calcCurvePath(const Pose2D &start_pose, int max_zigzag);

  void extractKeyInfo(const parking::OpenspaceDeciderOutput &input);
  bool isInnerSideVehicle(const parking::OpenspaceDeciderOutput &input);
  bool checkSinglePose(const Pose2D &pose, bool use_lon);

  void extendKeyPointsToResult(double step_size);

  Pose2D calcTurningPose(const Pose2D &start_pose, double safe_buffer,
                         planning_math::Vec2d &arc_center);

  Pose2D calcExpectedPose(const Pose2D &start_pose, int steer_direction,
                          int travel_direction);

  Pose2D calcExpectedPoseReverse(const Pose2D &start_pose, int steer_direction,
                                 int travel_direction, double radius);
  Pose2D calcExpectedPoseBoth(const Pose2D &start_pose, int steer_direction,
                              int travel_direction);

  /**
   * @brief get max backward-distance pose
   *
   * @param start_pose
   * @return bool
   */
  void getEPRectCorners(const Pose2D &pose,
                        std::vector<planning_math::Vec2d> &corners,
                        std::vector<planning_math::LineSegment2d> &lines);
  double calBackStraightDist(const Pose2D &start_pose);
  double calForwardStraightDist(const Pose2D &start_pose);

  /* data */
  double veh_min_r;
  bool is_on_left = false;

  double l_1;
  double l_2;

  double half_width;
  double front_to_rear;
  double back_to_rear;
  double front_corner_width = 0.0;
  double front_corner_length = 0.0;
  double inflation_for_points_;

  std::vector<std::pair<double, int>> buffer_zigzag_pairs_{
      {0.05, 4}, {0.06, 4}, {0.07, 4}, {0.08, 4}, {0.09, 4}, {0.10, 4},
      {0.11, 4}, {0.12, 4}, {0.13, 4}, {0.14, 4}, {0.15, 4}, {0.16, 4},
      {0.17, 3}, {0.18, 2}, {0.21, 1}, {0.24, 1}, {0.27, 1}, {0.30, 1}};
  double gamma_4;

  double in_slot_x_ = 0.5;
  double in_slot_theta_ = M_PI / 6;
  double in_slot_higher_dist_ = 1.0;

  double oneshot_higher_dist_ = 1.5;

  double slot_width_;
  double slot_angle_;

  double slot_open_height_;

  double sweet_x_min_ = 2.0;
  double sweet_x_max_ = 8.01;
  double sweet_y_min_ = 1.79;
  double sweet_y_max_ = 2.2;
  std::vector<Pose2D> sweet_spot_;

  double step_size_;

  std::vector<Pose2D> key_points_;

  Pose2D init_pose;
  double init_vel_;
  Pose2D target_pose;

  planning_math::Vec2d P_0_; // slot_left_corner_
  planning_math::Vec2d P_1_; // slot_right_corner_

  std::vector<planning_math::LineSegment2d> obstacles;
  std::vector<planning_math::Vec2d> points_of_obstacles;
  parking::TshapedAreaLines t_lines_;
};

} // namespace msquare