#ifndef MSQUARE_DECISION_PLANNING_PLANNER_BEHAVIOR_PLANNER_REQUEST_MANAGER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_BEHAVIOR_PLANNER_REQUEST_MANAGER_H_

#include "common/ego_state_manager.h"
#include "common/lateral_obstacle.h"
#include "common/lateral_virtual_lane.h"
#include "common/map_info_manager.h"
#include "pnc/define/request_manager_interface.hpp"
#include "request_manager_constants.hpp"

namespace msquare {

class LaneChangeRequest {
public:
  LaneChangeRequest();
  virtual ~LaneChangeRequest() = default;

  void gen_request(RequestType direction);
  void finish();

  int request() const { return request_; }
  int turn_light() const { return turn_light_; }
  double tstart() const { return tstart_; }
  double tfinish() const { return tfinish_; }

protected:
  int request_ = NO_CHANGE;
  int turn_light_ = NO_CHANGE;
  double tstart_ = 0.0;
  double tfinish_ = 0.0;
  std::shared_ptr<RequestManagerOutput> request_manager_output_;
};

class MapRequest : public LaneChangeRequest {
public:
  MapRequest(std::shared_ptr<RequestManagerOutput> request_manager_output);
  virtual ~MapRequest() = default;

  void update() {}

  void reset() {}

  void restore_context(const MapRequestContext &context){};
  void save_context(const MapRequestContext &context) const {};
};

class ActRequest : public LaneChangeRequest {
public:
  ActRequest(std::shared_ptr<RequestManagerOutput> request_manager_output);
  virtual ~ActRequest() = default;

  void reset(){};

  void update(const RequestManagerInput &request_manager_input);

  bool is_request_manager_input_valid(
      const RequestManagerInput &request_manager_input) const;

  bool is_map_info_valid(const MapInfo &map_info) const;

  void clear_act_request_output();

  void update_efficiency_cost_history(
      const RequestManagerInput &request_manager_input,
      std::deque<double> &ego_history, std::deque<double> &left_history,
      std::deque<double> &right_history) const;

  double calculate_ego_lane_efficiency_cost(
      const RequestManagerInput &request_manager_input);

  double calculate_left_lane_efficiency_cost(
      const RequestManagerInput &request_manager_input);

  double calculate_right_lane_efficiency_cost(
      const RequestManagerInput &request_manager_input);

  double calculate_object_cost(const RequestManagerInput &request_manager_input,
                               const SimpilfiedTrackedObject &object);

  double calculate_weight(const RequestManagerInput &request_manager_input,
                          const double d_rel, const double object_v) const;

  void update_filtered_efficiency_cost(const size_t min_history_size,
                                       double &current_cost,
                                       std::deque<double> &cost_history) const;

  void
  decide_lane_change_intention(const RequestManagerInput &request_manager_input,
                               const double ego_lane_cost,
                               const double left_lane_cost,
                               const double right_lane_cost);

  bool is_in_operational_design_domain(
      const RequestManagerInput &request_manager_input) const;

  bool is_changing_state(int state) const;

  void process_state_none(const RequestManagerInput &request_manager_input,
                          const double ego_lane_cost,
                          const double left_lane_cost,
                          const double right_lane_cost);

  bool is_left_lane_optional(const RequestManagerInput &request_manager_input,
                             const double delta_l);

  bool is_right_lane_optional(const RequestManagerInput &request_manager_input,
                              const double delta_r);

  bool is_back_one_safe_enough(const RequestManagerInput &request_manager_input,
                               const SimpilfiedTrackedObject &object) const;

  void process_state_left_change_wait(
      const RequestManagerInput &request_manager_input,
      const double ego_lane_cost, const double left_lane_cost);

  void process_state_right_change_wait(
      const RequestManagerInput &request_manager_input,
      const double ego_lane_cost, const double right_lane_cost);

  void process_state_left_change();

  void process_state_right_change();

  void process_state_change_back();

  std::string act_request_source() { return act_request_source_; }

  void restore_context(const ActRequestContext &context){};
  void save_context(const ActRequestContext &context) const {};

private:
  std::string act_request_source_{"none"};
  SimpilfiedTrackedObject left_back_one_;
  SimpilfiedTrackedObject right_back_one_;
  NotLaneChangeReason not_left_change_reason_;
  NotLaneChangeReason not_right_change_reason_;
};

class IntRequest : public LaneChangeRequest {
public:
  IntRequest(std::shared_ptr<RequestManagerOutput> request_manager_output);
  virtual ~IntRequest() = default;

  const IntCancelReasonType get_request_cancel_reason() const {
    return request_cancel_reason_;
  }
  bool get_exist_dash_line(void) const { return exist_process_dash_line_; }

  bool need_clear_ilc_ego_blinker(void) const {
    return need_clear_ilc_ego_blinker_;
  }
  void reset_need_clear_ilc_ego_blinker(void) {
    need_clear_ilc_ego_blinker_ = false;
  }

  void update(const RequestManagerInput &request_manager_input);

  void reset() {
    counter_left_ = 0;
    counter_right_ = 0;
    counter_cancel_left_ = 0;
    counter_cancel_right_ = 0;
    counter_cancel_vel_ = 0;
    exist_process_dash_line_ = false;
    request_cancel_reason_ = NO_CANCEL;
  }

  void finish_and_clear();

  void restore_context(const IntRequestContext &context);
  void save_context(IntRequestContext &context) const;

private:
  int counter_left_ = 0;
  int counter_right_ = 0;
  int counter_cancel_left_ = 0;
  int counter_cancel_right_ = 0;
  int counter_cancel_vel_ = 0;
  // to judge if exist dash line to cancel request (for L)
  bool exist_process_dash_line_ = false;
  bool need_clear_ilc_ego_blinker_ = false;
  IntCancelReasonType request_cancel_reason_ = NO_CANCEL;
};

class LCRequestManager {
public:
  LCRequestManager();
  virtual ~LCRequestManager() = default;

  void finish_request();

  LaneChangeDirection
  cal_lane_change_direction(const MapInfoManager &map_info_mgr);

  const RequestManagerInput &
  update_input(bool enable_ilc, bool enable_alc, bool enable_recommend_alc,
               std::shared_ptr<WorldModel> world_model,
               VirtualLaneManager &virtual_lane_mgr,
               LaneTracksManager &lane_tracks_mgr, int lc_status);

  void update(const RequestManagerInput &request_manager_input);

  double get_req_tstart(int source) const;
  double get_req_tfinish(int source) const;

  int request() const { return request_; }
  int request_source() const { return request_source_; }
  IntCancelReasonType get_int_request_cancel_reason() const {
    return int_request_cancel_reason_;
  }
  bool is_int_exist_dash_line(void) const { return int_exist_dash_line_; }
  bool need_clear_ilc_ego_blinker(void) const {
    return int_request_->need_clear_ilc_ego_blinker();
  }
  void reset_need_clear_ilc_ego_blinker(void) {
    int_request_->reset_need_clear_ilc_ego_blinker();
  }
  std::string act_request_source() {
    return request_source_ == ACT_REQUEST ? act_request_->act_request_source()
                                          : "none";
  }

  int turn_light() const { return turn_light_; }

  const RequestManagerOutput &request_manager_output() const {
    return *request_manager_output_;
  }

  void restore_context(const LCRequestManagerContext &context);
  void save_context(LCRequestManagerContext &context) const;

private:
  int request_ = NO_CHANGE;
  int request_source_ = NO_REQUEST;
  IntCancelReasonType int_request_cancel_reason_;
  bool int_exist_dash_line_;
  int turn_light_ = 0;

  std::shared_ptr<IntRequest> int_request_;
  std::shared_ptr<MapRequest> map_request_;
  std::shared_ptr<ActRequest> act_request_;
  std::shared_ptr<RequestManagerOutput> request_manager_output_;
};

} // namespace msquare

#endif
