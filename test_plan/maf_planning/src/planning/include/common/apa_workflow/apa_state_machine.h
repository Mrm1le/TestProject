#ifndef MSQUARE_DECISION_PLANNING_COMMON_PARKING_TASK_MANAGER_H_
#define MSQUARE_DECISION_PLANNING_COMMON_PARKING_TASK_MANAGER_H_

#include "common/parking_obstacle_manager.h"
#include "common/parking_task_config.h"
#include "common/parking_world_model.h"
#include "common/planning_context.h"
#include "time.h"
#include <map>
#include <string>
// #include "common/apa_key_point_selector.h"
#include "common/hfsm/machine_single.hpp"
#include "common/parking_slot_manager.h"
#include "planner/behavior_planner/deciders/collision_checker.h"

#include "openspace_state_machine.h"
#include "apa_behavior_calculator_parkin.h"
#include "apa_behavior_calculator_parkout.h"
#include "apa_behavior_decider_parkin.h"
#include "planner/motion_planner/openspace_motion_planner/path_sampler.h"
#include "planner/behavior_planner/parking_longitudinal_behavior_planner.h"
#include "planner/motion_planner/parking_longitudinal_motion_planner.h"


// simpify the definition of State with callback
#ifndef DEFINE_STATE_CLASS
#define DEFINE_STATE_CLASS(classname, baseclassname)                           \
  class classname : public baseclassname {                                     \
  public:                                                                      \
    void enter(const Context &context) {                                       \
      context.callback_on_entry<classname>();                                  \
    }                                                                          \
    void transition(Control &control, Context &context) {                      \
      context.callback_on_transition<classname>(control);                      \
    }                                                                          \
    void update(const Context &context) {                                      \
      context.callback_on_update<classname>();                                 \
    }                                                                          \
    void leave(const Context &context) {                                       \
      context.callback_on_leave<classname>();                                  \
    }                                                                          \
  };
#endif

namespace msquare {

namespace parking {

//[fenix.refactor.sm] function def moved from parking_scenario_manager.h
bool isEgoParallelInChannel(const Pose2D &ego_pose, Pose2D &target_pose);

class WorldModel;

class ApaStateMachine {
public:
  ApaStateMachine(const std::shared_ptr<WorldModel> &world_model);
  ~ApaStateMachine();

  bool update();

private:
  class Context {
    std::map<std::string, std::function<void()>> entry_callback_map;
    std::map<std::string, std::function<void()>> update_callback_map;
    std::map<std::string,
             std::function<void(hfsm::Machine<Context>::Control &)>>
        transition_callback_map;
    std::map<std::string, std::function<void()>> leave_callback_map;

  public:
    template <typename T>
    void register_entry_callback(std::function<void()> func) {
      entry_callback_map[typeid(T).name()] = func;
    }

    template <typename T>
    void register_update_callback(std::function<void()> func) {
      update_callback_map[typeid(T).name()] = func;
    }

    template <typename T>
    void register_transition_callback(
        std::function<void(hfsm::Machine<Context>::Control &)> func) {
      transition_callback_map[typeid(T).name()] = func;
    }

    template <typename T>
    void register_leave_callback(std::function<void()> func) {
      leave_callback_map[typeid(T).name()] = func;
    }

    template <typename T> void callback_on_entry() const {
      if (entry_callback_map.find(typeid(T).name()) !=
          entry_callback_map.end()) {
        entry_callback_map.at(typeid(T).name())();
      } else {
        // std::cout << "entry callback for " << typeid(T).name() << " not
        // found" << std::endl;
      }
    }

    template <typename T> void callback_on_update() const {
      if (update_callback_map.find(typeid(T).name()) !=
          update_callback_map.end()) {
        update_callback_map.at(typeid(T).name())();
      } else {
        // std::cout << "update callback for " << typeid(T).name() << " not
        // found" << std::endl;
      }
    }

    template <typename T>
    void callback_on_transition(hfsm::Machine<Context>::Control &control) {
      if (transition_callback_map.find(typeid(T).name()) !=
          transition_callback_map.end()) {
        transition_callback_map.at(typeid(T).name())(control);
      } else {
        // std::cout << "transition callback for " << typeid(T).name() << " not
        // found" << std::endl;
      }
    }

    template <typename T> void callback_on_leave() const {
      if (leave_callback_map.find(typeid(T).name()) !=
          leave_callback_map.end()) {
        leave_callback_map.at(typeid(T).name())();
      } else {
        // std::cout << "leave callback for " << typeid(T).name() << " not
        // found" << std::endl;
      }
    }
  };

  DEFINE_STATE_CLASS(ParkIn, hfsm::Machine<Context>::Base);
  DEFINE_STATE_CLASS(ParkOut, hfsm::Machine<Context>::Base);
  DEFINE_STATE_CLASS(Wait, hfsm::Machine<Context>::Base);
  DEFINE_STATE_CLASS(RpaStraightStandby, hfsm::Machine<Context>::Base);
  DEFINE_STATE_CLASS(RpaStraight, hfsm::Machine<Context>::Base);

  /*** task state ***/
  // entry callback
  void onEntryParkIn();
  void onEntryParkOut();
  void onEntryWait();
  void onEntryRpaStraightStandby();
  void onEntryRpaStraight();

  // update callback
  void onUpdateParkIn();
  void onUpdateParkOut();
  void onUpdateWait();
  void onUpdateRpaStraightStandby();
  void onUpdateRpaStraight();

  // transition callback
  void onTransitionParkIn(hfsm::Machine<Context>::Control &control);
  void onTransitionParkOut(hfsm::Machine<Context>::Control &control);
  void onTransitionWait(hfsm::Machine<Context>::Control &control);
  void onTransitionRpaStraightStandby(hfsm::Machine<Context>::Control &control);
  void onTransitionRpaStraight(hfsm::Machine<Context>::Control &control);

  // leave callback
  void onLeaveParkIn();
  void onLeaveParkOut();
  void onLeaveWait();
  void onLeaveRpaStraightStandby();
  void onLeaveRpaStraight();

  using COMPOSITE =
      hfsm::Machine<Context>::CompositePeers<Wait, ParkIn, ParkOut,
                                             RpaStraightStandby, RpaStraight>;

  Context context_;
  std::unique_ptr<hfsm::Machine<Context>::PeerRoot<COMPOSITE>> machine_;

  void setupCallbackFunctions();

  enum ParkingTaskType {
    WAIT = 0,
    PARKIN,
    PARKOUT,
    RPA_STRAIGHT_STANDBY,
    RPA_STRAIGHT,
    DONE,
  };

  class ParkingTask {
  public:
    std::string task_name;
    ParkingTaskType task_type;
    unsigned int poi_id;
    std::string poi_type;
    ParkOutDirectionType park_out_direction;
    RpaStraightDirectionType rpa_straight_direction;
  };

  std::vector<ParkingTask> parking_task_list_;
  std::vector<ParkingTask>::iterator current_task_;

  // for recommend park out
  void saveParkinData();
  void loadParkinData();

  bool generate_designate_parkin_tasks();
  bool generate_parkout_tasks();
  bool generate_rpa_straight_tasks();
  bool update_status();
  bool is_in_parking_slot();
  bool is_request_to_ego_slot();
  std::string status_text;


  StatusType current_state_ = StatusType::WAIT;
  StatusType next_state_ = StatusType::WAIT;
  void init();
  int plan_times_ = 0;
  bool need_re_parkin_ = false;
  bool check_park_out_direction_ = false;
  std::vector<int> unavailable_parking_lot_list_;
  std::shared_ptr<WorldModel> world_model_;
  ParkingTaskConfig parking_task_config_;
  CollisionChecker collision_checker_;
  ParkingSlotManager parking_slot_manager_;
  int control_test_id_{0};

  //[fenix.refactor.sm] rpa params & functions
  clothoid::CollisionChecker clo_checker_;
  clothoid::CollisionShapeGenerator csg_;
  clothoid::Parameter para_; // constant parameter

  void getRPARemainDis(double &forward_dis, double &backward_dis);
  //

  //[fenix.refactor.sm] parkout var & functions
  FlagFilter is_teb_ok_APOA_filter_;

  //

  //[fenix.refacotr.sm] openspace state machine

  std::unique_ptr<openspace_state_machine::OpenspaceStateMachine>
      openspace_state_machine_;

  //

  //[fenix.refactor] strategy decider && behavior calulator
  std::unique_ptr<APABehaviorDeciderParkIn> behavior_decider_parkin_;
  std::unique_ptr<APABehaviorCalculatorParkIn> behavior_calculator_parkin_;
  std::unique_ptr<APABehaviorCalculatorParkOut> behavior_calculator_parkout_;

  //[fenix.refactor] dynamic planning :TODO: move to somewhere else
  bool dynamic_planning_adjust_trajectory_tail();

  //[fenix.refactor.sm] moved from parking_scenario_manager.h
  bool checkTwoSides(bool &is_left_car, bool &is_right_car);

  FootprintModelPtr box_model_;
  std::shared_ptr<PathSampler> path_sampler_;
  std::shared_ptr<ParkingLongitudinalBehaviorPlanner>
      parking_longitudinal_behavior_planner_;
  std::unique_ptr<ParkingLongitudinalMotionPlanner>
      parking_longitudinal_motion_planner_;
  std::unique_ptr<LeaderDecider> leader_decider_;

  struct StateParkingConfig {
    Pose2D apa_psd_drift_threshold;
    Pose2D apa_min_adjust_threshold;
    double apa_simple_mpc_entry_threshold;
    Pose2D mpc_predict_threshold;
  };

  StateParkingConfig state_parking_cfg_;
  
  //[fenix.refactor] dynamic planning :TODO: move to somewhere else
  struct DynamicPlanSimpleMpcResult {
    bool traj_success;
    bool traj_collide;
  };

  DynamicPlanSimpleMpcResult
  dynamic_planning_simple_mpc(const Pose2D &ego_pose,
                              const Pose2D &target_pose_lot, 
                              std::vector<TrajectoryPoint>& curve_traj);

  //parkout behavior functions
  //TODO: move to parkout behavior planner
  bool manualSelectParkoutDirection(uint32_t direction);
  bool autoSelectParkOutDirection();
  bool setParkOutInitAndTarget();

};

} // namespace parking

} // namespace msquare

#endif
