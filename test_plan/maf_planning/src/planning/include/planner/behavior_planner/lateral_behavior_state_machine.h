#ifndef MSQUARE_DECISION_PLANNING_PLANNER_LATERAL_BEHAVIOR_STATE_MACHINE_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_LATERAL_BEHAVIOR_STATE_MACHINE_H_

#include "common/scenario_facade_context.h"
#include "common/world_model.h"
#include "data_driven_planner/data_driven_planner.h"
#include "planner/behavior_planner/general_motion_planner.h"
#include "planner/behavior_planner/lateral_behavior_intersect_state.h"
#include "planner/behavior_planner/lateral_behavior_request_manager.h"
#include "planner/behavior_planner/lateral_behavior_road_state.h"
#include "planner/tasks/obstacle_decider.h"
#include "pnc/define/request_manager_interface.hpp"

#include <array>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace msquare {

using LateralFsm = M::PeerRoot<
    M::Composite<
        RoadState, RoadState::None,
        M::Composite<RoadState::LC, RoadState::LC::LWait, RoadState::LC::RWait,
                     RoadState::LC::LChange, RoadState::LC::RChange,
                     RoadState::LC::LBack, RoadState::LC::RBack>,

        M::Composite<RoadState::LB, RoadState::LB::LBorrow,
                     RoadState::LB::RBorrow, RoadState::LB::LBack,
                     RoadState::LB::RBack, RoadState::LB::LReturn,
                     RoadState::LB::RReturn, RoadState::LB::LSuspend,
                     RoadState::LB::RSuspend>>,

    M::Composite<
        InterState,
        M::Composite<
            InterState::GS, InterState::GS::None,

            M::Composite<InterState::GS::LC, InterState::GS::LC::LWait,
                         InterState::GS::LC::RWait, InterState::GS::LC::LChange,
                         InterState::GS::LC::RChange, InterState::GS::LC::LBack,
                         InterState::GS::LC::RBack>,

            M::Composite<InterState::GS::LB, InterState::GS::LB::LBorrow,
                         InterState::GS::LB::RBorrow, InterState::GS::LB::LBack,
                         InterState::GS::LB::RBack, InterState::GS::LB::LReturn,
                         InterState::GS::LB::RReturn,
                         InterState::GS::LB::LSuspend,
                         InterState::GS::LB::RSuspend>>,

        M::Composite<
            InterState::TR, InterState::TR::None,

            M::Composite<InterState::TR::LC, InterState::TR::LC::LWait,
                         InterState::TR::LC::RWait, InterState::TR::LC::LChange,
                         InterState::TR::LC::RChange, InterState::TR::LC::LBack,
                         InterState::TR::LC::RBack>,

            M::Composite<InterState::TR::LB, InterState::TR::LB::LBorrow,
                         InterState::TR::LB::RBorrow, InterState::TR::LB::LBack,
                         InterState::TR::LB::RBack, InterState::TR::LB::LReturn,
                         InterState::TR::LB::RReturn,
                         InterState::TR::LB::LSuspend,
                         InterState::TR::LB::RSuspend>>,

        M::Composite<
            InterState::TL, InterState::TL::None,

            M::Composite<InterState::TL::LC, InterState::TL::LC::LWait,
                         InterState::TL::LC::RWait, InterState::TL::LC::LChange,
                         InterState::TL::LC::RChange, InterState::TL::LC::LBack,
                         InterState::TL::LC::RBack>,

            M::Composite<InterState::TL::LB, InterState::TL::LB::LBorrow,
                         InterState::TL::LB::RBorrow, InterState::TL::LB::LBack,
                         InterState::TL::LB::RBack, InterState::TL::LB::LReturn,
                         InterState::TL::LB::RReturn,
                         InterState::TL::LB::LSuspend,
                         InterState::TL::LB::RSuspend>>,

        M::Composite<InterState::UT, InterState::UT::None>>>;

typedef struct {
  std::pair<int, int> gap;
  TrafficFlowInfo target_lane_traffic_flow;
  bool gap_valid{false};
  bool gap_approached{false};
  bool gap_insertable{false};
  bool side_approach{false};
  bool enable_gap_protection{false};
  int gap_protection_counter{0};
  bool should_suspend{false};
  bool should_ready{false};

  // lc valid related
  bool lc_pause{false};
  int lc_pause_id{-1000};
  bool should_premove{false};
  std::string lc_invalid_reason{"none"};
  bool enable_interactive_mode{false};
  bool enable_lane_change_traj_checker{false};

  // lc back related
  double tr_pause_dv{0.0};
  double tr_pause_l{0.0};
  double tr_pause_s{-100.0};

  // back to state machine only for debug
  bool lc_should_back{false};
  bool lc_should_reset{false};
  bool lc_valid{false};
  std::string lc_back_reason{"none"};

} LaneChangeStageInfo;

class MSDStateMachine {
public:
  MSDStateMachine(const std::shared_ptr<WorldModel> &world_model);
  virtual ~MSDStateMachine();

  struct LBObjectInfo {
    int id;
    double d_rel;
    double d_min_cpath;
    double d_max_cpath;
    double length;
  };

  bool update();

  void update_state_machine();

  void reset_state_machine();

  void clear_lc_variables();

  void clear_lc_pause_variables();

  void check_vision_lane_stable();

  void calc_baseline_overlap(const std::shared_ptr<BaseLineInfo> &baseline_info,
                             const PlanningStatus *planning_status);

  template <typename T> void change_state_external() {
    if (fsm_context_.state == type2int<T>::value) {
      return;
    }

    MSD_LOG(INFO, "change_state_external from [%s] to [%s]",
            fsm_context_.name.c_str(), type2name<T>::name);

    fsm_context_.external = true;
    lateral_fsm_.changeTo<T>();
    lateral_fsm_.update();

    fsm_context_.external = false;
    fsm_context_.state = type2int<T>::value;
    fsm_context_.name = type2name<T>::name;
  }

  bool check_lc_valid(VirtualLaneManager &virtual_lane_mgr, int direction,
                      int property = TARGET_LANE);
  bool check_lc_body_valid(int direction);
  LaneChangeStageInfo
  compute_lc_valid_info(VirtualLaneManager &virtual_lane_mgr, int direction,
                        int property = TARGET_LANE);
  LaneChangeStageInfo decide_lc_valid_info(VirtualLaneManager &virtual_lane_mgr,
                                           int direction,
                                           int property = TARGET_LANE);
  LaneChangeStageInfo compute_lc_back_info(VirtualLaneManager &virtual_lane_mgr,
                                           int direction);
  LaneChangeStageInfo decide_lc_back_info(VirtualLaneManager &virtual_lane_mgr,
                                          int direction);
  bool check_head_crosss(VirtualLaneManager &virtual_lane_mgr, int direction);
  bool check_lc_finish(VirtualLaneManager &virtual_lane_mgr,
                       RequestType direction);
  static bool check_lc_back_finish(VirtualLaneManager &virtual_lane_mgr,
                                   RequestType direction);

  void post_process();
  void scenario_process();
  // std::shared_ptr<ScenarioFacade> produce_scenario_facade(const
  // ScenarioFacadeConfig &config);
  void update_decision_result();

  void update_hfsm_debug(void);

  void update_state_machine_output(
      const VirtualLaneManager &virtual_lane_mgr,
      const std::shared_ptr<ScenarioFacadeContext> &context);

  void generate_state_machine_output(
      int target_state, std::string target_state_name,
      const VirtualLaneManager &virtual_lane_mgr,
      const LaneChangeStageInfo &lc_info,
      const std::shared_ptr<ScenarioFacadeContext> &context);

  int scenario() const { return scenario_; }
  int current_state() const { return fsm_context_.state; }
  const std::string &state_name() const { return fsm_context_.name; }
  const FsmContext &fsm_context() const { return fsm_context_; }
  int turn_light() const { return turn_light_; }
  int lc_back_cnt() const { return lc_back_cnt_; }
  int lc_valid_cnt() const { return lc_valid_cnt_; }
  void set_lc_valid(bool is_valid, std::string reason) {
    lc_valid_ = is_valid;
    if (!is_valid) {
      lc_invalid_reason_ = reason;
    }
  }
  void set_entry_time(double t) { state_entry_time_ = t; }
  double entry_time() { return state_entry_time_; }
  void clear_lc_valid_cnt() { lc_valid_cnt_ = 0; }
  void clear_lc_back_cnt() { lc_back_cnt_ = 0; }
  void add_invalid_lc_count() { invalid_lc_change_freeze_count_++; }
  int get_invalid_lc_count() { return invalid_lc_change_freeze_count_; }
  void clear_invalid_lc_count() { invalid_lc_change_freeze_count_ = 0; }
  bool lc_valid() const { return lc_valid_; }
  bool lc_valid_back() const { return lc_valid_back_; }
  bool lc_should_back() const { return lc_should_back_; }
  bool lc_pause() const { return lc_pause_; }
  int lc_pause_id() const { return lc_pause_id_; }
  double tr_pause_l() const { return tr_pause_l_; }
  double tr_pause_s() const { return tr_pause_s_; }
  bool should_premove() const { return should_premove_; }
  bool should_suspend() const { return should_suspend_; }
  const std::string &lc_back_reason() const { return lc_back_reason_; }
  void set_lc_back_reason(std::string reason) { lc_back_reason_ = reason; }
  const std::string &lc_invalid_reason() const { return lc_invalid_reason_; }
  void set_lc_invalid_reason(std::string reason) {
    lc_invalid_reason_ = reason;
  }
  const std::string &invalid_back_reason() const {
    return invalid_back_reason_;
  }
  void set_state_change_reason(std::string reason) {
    state_change_reason_ = reason;
  }
  const std::string &state_change_reason() const {
    return state_change_reason_;
  }

  void set_gmp_should_cancel(bool gmp_should_cancel) {
    gmp_should_cancel_ = gmp_should_cancel;
  }
  const bool &gmp_should_cancel() const { return gmp_should_cancel_; }

  void set_lc_state(std::string reason) { lc_state_ = reason; }
  const std::string &lc_state() const { return lc_state_; }
  void set_active_request_finished(bool active_request_finished) {
    active_request_finished_ = active_request_finished;
  }
  void set_must_change_lane(bool must_change_lane) {
    must_change_lane_ = must_change_lane;
  }
  const bool active_request_finished() { return active_request_finished_; }
  void set_active_request_finished_dir(int active_request_finished_dir) {
    active_request_finished_dir_ = active_request_finished_dir;
  }
  int active_request_finished_dir() { return active_request_finished_dir_; }
  void set_map_request_finished(bool map_request_finished) {
    map_request_finished_ = map_request_finished;
  }
  const bool map_request_finished() { return map_request_finished_; }
  const std::vector<TrackInfo> &near_cars_target() const {
    return near_cars_target_;
  }
  const std::vector<TrackInfo> &near_cars_origin() const {
    return near_cars_origin_;
  }
  const TrackInfo &lc_invalid_track() const { return lc_invalid_track_; }
  const TrackInfo &lc_back_track() const { return lc_back_track_; }
  int curr_intsect_task() const { return curr_inter_task_; }
  void set_get_dist_lane(bool value) { get_dist_lane_ = value; }
  void set_turn_light(int value) { turn_light_ = value; }
  bool use_backup_baseline() { return use_backup_baseline_; }

  double lat_gap() const { return lat_gap_; }
  IntCancelReasonType &int_cancel_reason() { return int_cancel_reason_; }

  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr() {
    return virtual_lane_mgr_;
  }

  std::shared_ptr<WorldModel> world_model() { return world_model_; }

  void update_virtual_lane_mgr(const VirtualLaneManager &source) {
    *virtual_lane_mgr_ = source;
  }

  std::shared_ptr<LaneTracksManager> lane_tracks_mgr() {
    return lane_tracks_mgr_;
  }

  std::shared_ptr<LCRequestManager> lc_request_manager() { return lc_req_mgr_; }

  bool is_lc_lane_stable(int direction) const;

private:
  FsmContext fsm_context_;
  LateralFsm lateral_fsm_;

  int scenario_ = LOCATION_ROAD;
  int last_state_ = ROAD_NONE;
  bool use_backup_baseline_ = false;
  IntCancelReasonType int_cancel_reason_ = NO_CANCEL;

  double state_entry_time_{0.0};

  int turn_light_ = 0;

  int lc_back_cnt_ = 0;
  int lc_valid_cnt_ = 0;

  bool lc_valid_ = true;
  bool lc_valid_back_ = true;
  bool lc_should_back_ = false;
  bool must_change_lane_ = false;
  bool gmp_should_cancel_ = false;

  bool is_steer_over_limit_ = false;
  bool is_lca_state_activated_ = false;

  bool lc_pause_ = false;
  int lc_pause_id_ = -1000;
  double tr_pause_l_ = 0.0;
  double tr_pause_s_ = -100.0;
  double tr_pause_dv_ = 0.0;

  std::string lc_back_reason_ = "none";
  std::string lc_invalid_reason_ = "none";
  std::string invalid_back_reason_ = "none";
  std::string state_change_reason_ = "none";
  std::string lc_state_ = "none";

  std::vector<TrackInfo> near_cars_target_;
  std::vector<TrackInfo> near_cars_origin_;

  std::vector<int> llane_stable_list_;
  std::vector<int> rlane_stable_list_;
  int left_lane_stable_cnt_ = 0;
  int right_lane_stable_cnt_ = 0;

  TrackInfo lc_invalid_track_;
  TrackInfo lc_back_track_;

  bool get_dist_lane_ = false;

  bool should_premove_ = false;
  bool should_suspend_ = false;

  int curr_inter_task_ = 1;
  double start_move_dist_lane_ = 0;

  double lat_gap_ = 0.0;

  int invalid_lc_change_freeze_count_ = 0;

  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr_;
  std::shared_ptr<LaneTracksManager> lane_tracks_mgr_;
  std::shared_ptr<LCRequestManager> lc_req_mgr_;
  LaneChangeStageInfo lc_stage_info_;

  int active_request_finished_{false};
  int active_request_finished_dir_ = NO_CHANGE;
  int map_request_finished_{false};
  std::vector<CartEgoState> ego_state_queue_;

  std::shared_ptr<WorldModel> world_model_;
  std::unique_ptr<msquare::ddp::DataDrivenPlanner> data_driven_planner_;
  std::shared_ptr<ObstacleDecider> obstacle_decider_; // 2022-06-07 SHUAI
  std::shared_ptr<GeneralMotionPlanner>
      general_motion_planner_; // 2022-05-12 SHUAI
};

struct FsmUserContext {
  double safety_dist;
  std::array<double, 4> delay_time;
  MSDStateMachine *state_machine;
};

} // namespace msquare

#endif