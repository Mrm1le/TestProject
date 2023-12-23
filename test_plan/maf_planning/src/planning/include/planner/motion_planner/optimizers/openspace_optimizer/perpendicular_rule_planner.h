#include <unordered_map>

#include "common/config/vehicle_param.h"
#include "common/math/polygon2d.h"
#include "common/parking_planner_types.h"
#include "search_based_planner.h"

namespace msquare {

class PerpendicularRulePlanner : public SearchBasedPlanner {

public:
  PerpendicularRulePlanner(const parking::OpenspaceDeciderOutput &odo);
  ~PerpendicularRulePlanner() {}

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
  bool PlanInSlotSub0_5(std::vector<Pose2D> &path_to_fill);
  void PlanInSlotSub1(std::vector<Pose2D> &path_to_fill, double ratio); // CCS
  void PlanInSlotSub2(std::vector<Pose2D> &path_to_fill, double ratio); // CCCCS

  bool isMidPlan();
  bool planMidway(const Pose2D &mid_pose, bool is_reverse);
  bool planMidway2(const Pose2D &mid_pose, bool is_reverse);
  bool planMidwayBackward(const Pose2D &mid_pose, bool is_reverse);
  bool planMidwayForward(const Pose2D &mid_pose, bool is_reverse);
  bool planMidway3(const Pose2D &mid_pose);
  bool planRestartWay(const Pose2D &mid_pose);

  bool PlanWithBuffer(double safe_buffer, int max_zigzag);
  bool PlanWithBufferReverse(double safe_buffer, int max_zigzag);

  bool planOneShot(const Pose2D &start_pose);
  bool tooNearUpperWall(const Pose2D &turning_pose,
                        const planning_math::Vec2d &center);
  bool startWithStraightLine(double safe_buffer, int max_zigzag);

  bool calcSCSAdjustPoses(const Pose2D &start_pose, const Pose2D &end_pose,
                          std::vector<Pose2D> &head_kps);
  bool calcAdjustPoses(const Pose2D &end_pose, double radius);
  bool startWithSweetPose(double safe_buffer, int max_zigzag);
  bool calcCurvePath(const Pose2D &start_pose, int max_zigzag);

  void extractKeyInfo(const parking::OpenspaceDeciderOutput &input);
  bool isInnerSideVehicle(const parking::OpenspaceDeciderOutput &input);
  bool isNarrowScenario();
  bool isNarrowChannelWidthScenario();
  bool isTwoSideIsEmptyScenario();
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
  void generateSweetSpots();

  bool PlanWithBuffer2(double safe_buffer, int max_zigzag);
  void constructNewTlines();
  bool PlanWithBuffer2FromInitPose(double safe_buffer, int max_zigzag);
  bool PlanWithBuffer2FromLowerPose(double safe_buffer, int max_zigzag);
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
  double calInnerSideDist(const Pose2D &start_pose);

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
      {0.15, 4}, {0.16, 4}, {0.17, 4}, {0.18, 4}, {0.19, 4}, {0.20, 4},
      {0.21, 4}, {0.22, 4}, {0.23, 4}, {0.24, 4}, {0.25, 4}, {0.26, 4},
      {0.27, 3}, {0.28, 2}, {0.31, 1}, {0.34, 1}, {0.37, 1}, {0.40, 1}};
  double gamma_4;

  double in_slot_x_ = 0.5;
  double in_slot_theta_ = M_PI / 6;
  double in_slot_higher_dist_ = 1.0;

  double oneshot_higher_dist_ = 2.0;

  std::unordered_map<double, std::vector<Pose2D>> sweet_spot_;
  std::vector<Pose2D> safe_sweet_poses_;
  double slot_width_;

  double step_size_;

  std::vector<Pose2D> key_points_;

  Pose2D init_pose;
  Pose2D target_pose;

  double init_vel_;
  double target_vel_;

  planning_math::Vec2d P_0_; // slot_left_corner_
  planning_math::Vec2d P_1_; // slot_right_corner_
  double upper_height_;

  std::vector<planning_math::LineSegment2d> obstacles;
  std::vector<planning_math::Vec2d> points_of_obstacles;
  parking::TshapedAreaLines t_lines_;

  double move_ditsance_ = 0.0;
  double up_bound_move_distance_ = 0.0;
};

class OdoAdaptor {
public:
  enum class PillarInvadeType {
    OUT_SIDE_PILLAR_INVADE_ONLY = 1,
    IN_SIDE_PILLAR_INVADE_ONLY = 2,
    OUT_AND_IN_SIDE_PILLAR_INVADE = 3,
    NO_PILLAR_INVADE = 4
  };

private:
  parking::OpenspaceDeciderOutput odo_;
  double up_bound_move_distance_;
  double move_distance_;

public:
  parking::OpenspaceDeciderOutput generateNewOdo();
  double getMoveDistance() { return move_distance_; };
  double getUpBoundMoveDistance() { return up_bound_move_distance_; }
  OdoAdaptor(const parking::OpenspaceDeciderOutput &odo) : odo_(odo) {}
  PillarInvadeType identifySpecialPillarScene(bool is_on_left);
  void handleOutSidePillar();
};

} // namespace msquare