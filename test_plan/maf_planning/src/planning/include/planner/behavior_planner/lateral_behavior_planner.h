#ifndef MSQUARE_DECISION_PLANNING_PLANNER_LATERAL_BEHAVIOR_PLANNER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_LATERAL_BEHAVIOR_PLANNER_H_

#include "common/extract_path_bound.h"
#include "common/obstacle.h"
#include "planner/behavior_planner/lateral_behavior_path_planner.h"
#include "planner/behavior_planner/lateral_behavior_request_manager.h"
#include "planner/behavior_planner/lateral_behavior_state_machine.h"
#include "planner/tasks/task.h"

#include <array>

namespace msquare {

using AvoidObstacle = std::tuple<int, std::string>;

typedef enum {
  MANEUVER_NONE,
  MANEUVER_CHANGE,
  MANEUVER_BACK,
} ManeuverType;

typedef enum { NORMAL_ROAD = 1, INTERSECT = 2 } AlgorithmScene;

typedef enum {
  LANE_CHANGE_LEFT = 1,
  LANE_CHANGE_RIGHT = 2,
  LANE_BORROW_LEFT = 4,
  LANE_BORROW_RIGHT = 8,
  INTERSECT_GO_STRAIGHT = 16,
  INTERSECT_TURN_LEFT = 32,
  INTERSECT_TURN_RIGHT = 64,
  INTERSECT_U_TURN = 128,
  LANE_BORROW_IN_NON_MOTORIZED_LANE = 256
} AlgorithmAction;

typedef enum {
  LANE_CHANGE_WAITING = 1,
  LANE_CHANGEING = 2,
  LANE_CHANGE_BACK = 4,
  LANE_BORROWING = 8,
  LANE_BORROW_BACK = 16,
  LANE_BORROW_RETURN = 32,
  LANE_BORROW_SUSPEND = 64
} AlgorithmStatus;

class LateralBehaviorPlanner : public Task {
public:
  LateralBehaviorPlanner(const TaskConfig &config);
  virtual ~LateralBehaviorPlanner();

  virtual void init(std::shared_ptr<WorldModel> world_model);

  void reset(const TaskConfig &config);

  void unset();

  TaskStatus execute(ScenarioFacadeContext *context);

private:
  bool calculate();

  bool update(bool active);

  double update_antsides_strict();
  bool update_lfrontavds_info(bool no_near_car);
  bool update_rfrontavds_info(bool no_near_car);
  bool update_lsideavds_info(bool no_near_car);
  bool update_rsideavds_info(bool no_near_car);
  void update_avoid_cars();

  void update_path_planner();
  void update_vel_sequence();
  void update_ignore_track_id();

  void
  update_avd_info(std::vector<double> &avd_car_info,
                  const std::vector<const TrackedObject *> &front_side_tracks,
                  double ego_l, int avd_priority);

  bool update_lateral_info();

  void update_avd_info();
  void update_avdobstacles_info();
  void update_obstacle_time_interval();
  void ignore_cutin_avd_obstacles();
  bool update_cutin_obstacles(Obstacle *obstacle);

  bool update_planner_output();
  void update_planner_status();
  void print_planner_output();

  void restore_context();
  void save_context() const;
  void output_lat_dec_info();

  bool no_sp_car_ = true;

  bool is_ncar_ = false;
  double t_avd_car_ = 3.0;
  double t_avd_sp_car_ = 3.0;
  double final_y_rel_ = 10;

  int ncar_change_ = 0;
  int flag_avd_ = 0;
  int avd_back_cnt_ = 0;
  int avd_leadone_ = 0;
  int pre_leadone_id_ = 0;

  int ignore_track_id_ = -10000;

  std::vector<double> vel_sequence_;
  std::map<int, AvdMsg> avd_info_;
  std::vector<AvoidObstacle> avd_obstacles;
  std::vector<AvoidObstacle> avd_obstacle_prior_;

  std::set<int> ignore_change_false_;
  std::set<int> ignore_change_true_;

  std::vector<std::tuple<double, double>> s_v_limit_; // (s, v_max)
  std::vector<std::tuple<double, double>> s_a_limit_; //(s, a_max)
  std::vector<std::tuple<double, double>> s_r_offset_;

  std::array<std::vector<double>, 2> avd_car_past_;
  std::array<std::vector<double>, 2> avd_sp_car_past_;

  std::unique_ptr<VirtualLaneManager> virtual_lane_mgr_ = nullptr;
  std::unique_ptr<PathPlanner> path_planner_ = nullptr;
};

} // namespace msquare

#endif
