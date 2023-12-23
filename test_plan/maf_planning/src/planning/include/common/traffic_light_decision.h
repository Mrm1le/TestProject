#ifndef MSQUARE_DECISION_PLANNING_COMMON_TRAFFICLIGHT_DECISION_H_
#define MSQUARE_DECISION_PLANNING_COMMON_TRAFFICLIGHT_DECISION_H_

#include <memory>
#include <string>
#include <vector>

#include "common/map_info_manager.h"

using namespace maf_perception_interface;
namespace msquare {

class WorldModel;

class TrafficLightDecision {
public:
  TrafficLightDecision();
  ~TrafficLightDecision() = default;

  void update(const std::shared_ptr<WorldModel> &world_model);

  int get_current_state() const { return (int)current_state_; }
  int get_straight_light_status_at_right() const {
    return straight_light_status_at_right_;
  }
  bool get_stop_flag() const { return stop_flag_; }
  bool get_is_traffic_light_in_field() const {
    return is_traffic_light_in_field_;
  }
  bool get_is_passed_stop_line() const { return is_passed_stopline_; }
  double get_stop_point() const { return stop_point_; }
  int get_RED_LIGHT_STOP_CLASS() const { return RED_LIGHT_STOP_CLASS_; }
  bool get_traffic_light_status_trigger() const {
    return traffic_light_status_trigger;
  }
  bool get_is_GREENBLINKING_go_() const { return is_GREENBLINKING_go_; }

private:
  enum CurrentState {
    INIT = 0,
    LANE_KEEPING,
    CROSSING,
    APPROACH_STOPLINE,
    RED_LIGHT_STOP,
    COVER_LIGHT,
    INTO_WAIT_ZONE
  };

  void init();
  std::string translate_direction(const int &direction_traffic_light);
  void intersection_decision(const double &distance_to_intersection);
  void get_traffic_light_status(int &light_status, double &duration_time,
                                double &remain_time);
  std::string trans_state_to_sting(const int &light_status);

  void decision_compute(
      double dist_to_stopline, int direction,
      const std::vector<maf_perception_interface::TrafficLightPatternEnum>
          &light_patterns,
      const std::vector<maf_perception_interface::TrafficLightDecision>
          &traffic_light,
      double ego_vel, double ego_acc, bool is_inside_intersection,
      double crossing_length);

  void state_machine();
  void compute_output(const bool &stop_decision);
  double set_stopline_distance();
  double compute_jerk(const std::pair<double, double> comfort_param);
  std::pair<double, double> compute_comfort_param(bool ignore_length);

  static constexpr double kDistNoStopLine = 500.0;
  static constexpr double kStopDisBuffer = 3.0;
  static constexpr double kStopDisPreChecker = 3.0;
  static constexpr double kThresCheckTrafficLight = 120.0;
  static constexpr double kGreenBlinkingDuration = 2.0;
  static constexpr double kComfortDec = 2.0;
  static constexpr double kYellowDuration = 2.5;
  static constexpr double kPassingStopLineBuffer = 10.0;
  static constexpr double kStopLineBuffer = 0.0;
  static constexpr double kCrossingDistanceBuffer = 0.0;
  static constexpr double kStopChecker = 0.3;
  static constexpr double kStopCheckerDistance = 1.0;
  static constexpr double kStopCheckerFollowObs = 1.0;
  static constexpr double kDistanceToIntersectionChecker = -7.0;
  static constexpr double kDistanceToCrossingChecker = -10.0;
  static constexpr double kVelToCrossingChecker = 3.0;
  static constexpr double kPassedIntersectionTimeBuffer = 2.0;
  static constexpr double kCOVERTimeBuffer = 2.5;
  // static constexpr double kComfortMaxjerk = -2.5;
  // static constexpr double kComfortMaxAcc = -2.1;
  static constexpr double kDistanceToIntersectionCheckerRatio = -0.25;
  static constexpr int flag_light_status_OFF = 50;
  static constexpr int flag_light_status_YELLOW = 50;
  static constexpr int FlagLonFollowObs = 4;
  static constexpr double kLongCrossingLength = 64.0;
  static constexpr double kTrafficLightInField = 1.0;
  static constexpr double kGREENBLINKINGTimeBuffer = 0.3;
  static constexpr double kYELLOWTimeBuffer = 1.2;
  static constexpr double IntersectionLowSpeedDistance = 10.0;
  static constexpr double kGREENBLINKINGByLonFollow = 0.1;
  static constexpr int kCOVERWaitTime = 12;
  static constexpr double kFlowDistance = 15.0;
  static constexpr double kFlowYawRelativeFrenet = 0.7;
  static constexpr double kFlowComfortDec = 2.0;
  static constexpr double kFlowBaselineL = 1.0;
  static constexpr double kDistPlanningBuffer = -1.0;
  static constexpr double kDistComfortBuffer = 8.0;
  static constexpr double kComfortDecEmer = 3.0;
  static constexpr double kCOVERGREENdist = 20.0;

  std::vector<maf_perception_interface::TrafficLightDecision> traffic_light_;

  int current_state_ = CurrentState::INIT;
  double dist_to_stopline_ = kDistNoStopLine;
  double crossing_length_ = kDistNoStopLine;
  int state_cycles_ = 1;
  int current_direction_ = Direction::UNKNOWN;
  std::vector<maf_perception_interface::TrafficLightPatternEnum>
      light_patterns_;
  double ego_vel_ = 40.0 / 3.6;
  double ego_acc_ = 0;
  bool is_passed_stopline_ = false;
  double dist_to_stopline_pre_ = kDistNoStopLine;
  bool stop_flag_ = false;
  double stop_point_ = kDistNoStopLine;
  bool entered_intersection_ = false;
  bool is_inside_intersection_ = false;
  bool intersection_decision_flag_ = false;
  double ditance_to_intersection_ = kDistNoStopLine;
  bool dbw_status_ = false;
  bool dbw_status_pre_ = false;
  bool is_in_intersection_pre_ = false;
  double passed_intersection_time_ = 0.0;
  int light_status_pre_ = TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_UNKNOWN;
  bool OFF_FLAG_ = true;
  int number_light_status_OFF_ = 0;
  double comfort_max_jerk_ = -2.5;
  double comfort_max_acc_ = -2.1;
  int straight_light_status_at_right_ = 0;
  int RED_LIGHT_STOP_CLASS_ = 0;
  //  1: CROSSING, 2: GREENBLINKING, 3: YELLOW, 4:RED, 5: COVER  6 : OFF
  bool is_traffic_light_in_field_ = true;
  bool turn_right_flag_ = false;
  int COVER_state_number_ = 0;
  bool traffic_light_status_trigger = false;
  bool EvaluateByTime_right_1 = true;
  // bool EvaluateByTime_right_2 = true;
  // bool EvaluateByTime_right_3 = true;
  // bool EvaluateByTime_right_4 = true;
  bool EvaluateByTime_right_time = true;
  bool EvaluateByTime_right_start = true;
  bool EvaluateByTime_right_end = true;
  double lon_Evaluate_1 = 0.0;
  // double lon_Evaluate_2 = 0.0;
  // double lon_Evaluate_3 = 0.0;
  // double lon_Evaluate_4 = 0.0;
  double lon_Evaluate_time = 0.0;
  double lon_Evaluate_start = 0.0;
  double lon_Evaluate_end = 0.0;
  // double lon_s_Evaluate_1 = 0.0;
  // double lon_s_Evaluate_2 = 0.0;
  // double lon_s_Evaluate_3 = 0.0;
  double lon_s_Evaluate_time = 0.0;
  double lon_Evaluate_intersection = 0.0;
  bool YELLOW_FLAG_ = true;
  int number_light_status_YELLOW_ = 0;
  bool action_reconsider_go_FLAG_ = false;
  // bool is_crossing_when_GREENBLINKING_ =  false;
  bool is_GREENBLINKING_go_ = false;
  bool is_left_car_go_ = false;
  bool is_right_car_go_ = false;
  bool curr_direct_traffic_ = true;
};

} // namespace msquare

#endif
