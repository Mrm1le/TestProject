#ifndef MSQUARE_DECISION_PLANNING_PLANNER_LANE_CHANGE_DECIDER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_LANE_CHANGE_DECIDER_H_

#include "common/obstacle_manager.h"
#include "common/world_model.h"
#include "planner/tasks/deciders/decider.h"

namespace msquare {

typedef enum {
  URBAN = 1,
  HIGHWAY = 2,
} ScenarioType;

typedef enum { EXTERNAL = 0, INTERNAL = 1 } UpdateType;

struct GapInfo {
  int rear_id;
  int front_id;
  int base_car_id;
  bool acc_valid; // acc valid if front car speed is much larger than rear car
                  // speed
  bool valid;
  double cost;
  double acc_time;

  double base_car_drel;
  double base_car_vrel;
};

struct TargetObstacle {
  int id;
  int type;
  double d_rel;
  double accel;
  double start_s{0.0};
  double end_s{0.0};
  double v_rel;
  std::string intention{"none"};
  int temp_lead{-1};
  double temp_lead_v_rel{30.0};
};

class LaneChangeDecider : public Decider {
public:
  explicit LaneChangeDecider(const TaskConfig &config);
  virtual void init(std::shared_ptr<WorldModel> world_model);
  std::pair<int, int> get_target_gap() { return target_gap_; }
  double get_target_gap_cost() { return target_gap_cost_; }
  double dis_to_change_point() { return dis_to_change_point_; }
  double average_gap_length() { return average_gap_length_; }
  bool is_target_gap_frozen() { return is_target_gap_frozen_; }
  bool updata_real_obstacle_on_target(UpdateType rqt, int origin_id,
                                      int target_id);
  int get_cloest_front_id() const { return closest_car_front_id; }
  int get_cloest_rear_id() const { return closest_car_id_; }
  double get_cloest_front_ttc() const { return cloest_front_ttc_; }
  double get_cloest_rear_ttc() const { return cloest_rear_ttc_; }
  double get_cloest_front_nttc() const { return cloest_front_nttc_; }
  double get_cloest_rear_nttc() const { return cloest_rear_nttc_; }
  double get_cloest_front_dis() const { return cloest_front_dis_; }
  double get_cloest_rear_dis() const { return cloest_rear_dis_; }
  double get_cloest_front_dis_p1() const { return cloest_front_dis_p1_; }
  double get_cloest_rear_dis_p1() const { return cloest_rear_dis_p1_; }
  std::vector<int> get_real_obstacle_on_target(void) const {
    return real_obstacle_on_target_;
  };
  double get_nearset_ttc(void) const { return nearest_ttc_; }
  TrafficFlowInfo get_target_lane_traffic_flow() {
    return target_lane_traffic_flow_info_;
  }

  void reset(const TaskConfig &config);

  void unset();

private:
  TaskStatus process();
  bool is_on_target(const Obstacle *obstacle);
  bool is_pred_on_target(const Obstacle *obstacle, const double v_ego);
  std::string identify_intention(const Obstacle *obstacle);
  bool is_gap_insertable(const GapInfo &gap);
  GapInfo check_gap_valid(const TargetObstacle &rear_car,
                          const TargetObstacle &front_car);
  static bool compare_distance_asc(const TargetObstacle &obs1,
                                   const TargetObstacle &obs2);
  static bool compare_distance_drel(const TargetObstacle &obs1,
                                    const TargetObstacle &obs2);
  double calc_lane_width(const double &s,
                         const std::vector<RefPointFrenet> &ref_line);
  double calc_equation_ttc(const double &a, const double &b, const double &c);
  double calc_predict_drel(const Obstacle *object, const PathPoint &egopoint);
  double clip(const double x, const double lo, const double hi);
  double calc_time_for_lane_change(TargetObstacle base_car,
                                   TargetObstacle front_car, GapInfo gap_info,
                                   const double safety_distance,
                                   const double max_v);
  double calc_desired_distance(const double v_lead, const double v_ego);
  double calc_desired_speed(const double d_lead, const double d_des,
                            const double v_lead_unp);
  static bool compare_cost_asc(const GapInfo &gap1, const GapInfo &gap2);
  double interp(double x, const std::vector<double> &xp,
                const std::vector<double> &fp);

  const std::vector<double> _T_GAP_VEGO_BP{5.0, 15.0, 30.0};
  const std::vector<double> _T_GAP_VEGO_V{1.35, 1.55, 2.0};
  const std::vector<double> _L_SLOPE_BP{0.0, 40.0};
  const std::vector<double> _L_SLOPE_V{0.35, 0.08};
  const std::vector<double> _P_SLOPE_V{0.8, 0.2};
  const std::vector<double> _P_SLOPE_BP{0., 40.0};

  bool called_in_state_machine_{false};
  TargetObstacle lead_car_;
  std::vector<TargetObstacle> obstacle_on_target_;
  const Obstacle *most_front_obstacle_in_target_line_{nullptr};
  const Obstacle *nearest_obstacle_in_solid_line_{nullptr};
  std::pair<int, int> target_gap_{-10, -10};
  double target_gap_cost_{std::numeric_limits<double>::max()};
  int lane_change_direction_;
  double v_limit_;
  double map_v_limit_;
  double min_l_threshold_ = 1.3;
  double max_l_threshold_ = 5.0;
  TargetObstacle most_front_car_ = {-1, 1, 1000.0, 100.0, 100.0, 20.0};
  TargetObstacle most_rear_car_ = {-2, 1, -1100.0, -110.0, -110.0, -20.0};
  std::vector<GapInfo> gap_list_;
  std::vector<RefPointFrenet> cur_lane_, target_lane_;
  std::vector<int> real_obstacle_on_target_;
  double dis_to_change_point_;
  double min_gap_s_;
  bool exist_fast_approach_car_from_behind_{false};
  std::string gap_selection_scenario_;
  double lat_offset_;
  double most_front_static_obs_dis_{0.0};
  double nearest_ttc_{0.0};
  double cloest_rear_ttc_{0.0};
  double cloest_rear_nttc_{0.0};
  double cloest_front_ttc_{0.0};
  double cloest_front_nttc_{0.0};
  double cloest_rear_dis_{0.0};
  double cloest_rear_dis_p1_{0.0};
  double cloest_front_dis_{0.0};
  double cloest_front_dis_p1_{0.0};

  int target_lane_id_;
  int current_lane_id_;

  ScenarioType scenario_type_;
  int overtake_obs_id_{-5};
  int closest_car_id_{-200};
  int closest_car_front_id{-200};
  int closest_fast_approach_car_id_{-200};
  double lc_wait_time_{0.0};
  double average_gap_length_{20.0};
  TrafficFlowInfo target_lane_traffic_flow_info_;
  int lane_change_cross_num_{0};
  std::vector<std::pair<double, double>> target_lane_invisible_s_sections_;
  bool is_most_front_gap_invisible_{true};
  bool is_most_front_gap_exist_and_invisible_{false};
  bool is_target_gap_frozen_{false};
};

} // namespace msquare

#endif
