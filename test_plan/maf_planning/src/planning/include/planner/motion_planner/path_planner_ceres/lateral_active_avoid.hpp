#pragma once

#include "common/math/quintic_poly_1d.h"
#include "path_planner_constants.hpp"
#include "path_planner_types.hpp"
#include "path_spline_wrapper.hpp"

namespace path_planner {
using State = Intelligent_Dodge_Info::State;
class ActiveAvoid {

public:
  ActiveAvoid(const PathPlannerInput &input);

  void offset_active(DecisionInfo &decision_info,
                     std::unordered_map<int, double> &active_target_l,
                     Intelligent_Dodge_Info &pre_dodge_info);

  static bool
  optimize_quintic_poly(const PathPlannerInput &input, const double &target_l,
                        const double &dddl_ref, const std::string direction,
                        msquare::planning_math::QuinticPoly1d &quintic_poly_opt,
                        double &end_s_opt, double end_s_ref = 0.);

private:
  bool is_parallel_driving(const ObsInfo &obs);
  bool front_truck_avd_target(const std::vector<ObsInfo> &front_trucks,
                              double &target_s, double &target_l,
                              std::string &avd_direction,
                              std::unordered_map<int, double> &active_target_l);
  bool truck_avd_invalid(const std::vector<ObsInfo> &front_trucks);
  bool truck_avd_should_end(const std::vector<ObsInfo> &front_trucks,
                            const Intelligent_Dodge_Info &pre_dodge_info);
  bool truck_avd_should_trigger(const std::vector<ObsInfo> &front_trucks,
                                const Intelligent_Dodge_Info &pre_dodge_info);
  bool lateral_keeping(const std::vector<ObsInfo> &overlap_obs);
  void update_decision_info_lateral_keeping(DecisionInfo &decision_info);
  void update_decision_info(DecisionInfo &decision_info, const double &target_l,
                            const double &target_s,
                            const std::string &avd_direction);
  void select_trucks(std::vector<ObsInfo> &front_trucks,
                     std::vector<ObsInfo> &overlap_trucks);

  PathPlannerInput input_;
  double half_self_width_{2.5};
  double half_self_length_{1.0};
};

} // namespace path_planner