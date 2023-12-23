#include "planner/motion_planner/speed_planner_ceres/obstacle_headway_preprocessor.h"
#include "linear_interpolation_utility.hpp"
#include "planner/motion_planner/speed_planner_ceres/speed_planner_constants.hpp"

using namespace path_planner;
using namespace speed_planner;
using namespace msquare::planning_math;

namespace msquare {

inline void erase_invalid_obstacle(
    std::unordered_map<int, ObstacleExtraInfo> &obstacle_extra_info_map,
    bool is_lane_change) {
  std::vector<int> erase_obstacles{};
  for (const auto &obs : obstacle_extra_info_map) {
    int obstacle_id = obs.first;
    if (obs.second.is_alive == false) {
      erase_obstacles.push_back(obstacle_id);
    }

    if (!is_lane_change &&
        obs.second.type == ObstacleExtraInfo::ObstacleType::IN_ORIGIN_LANE) {
      erase_obstacles.push_back(obstacle_id);
    }
  }

  for (const auto &erase_id : erase_obstacles) {
    obstacle_extra_info_map.erase(erase_id);
  }
}

inline void update_stop_offset(path_planner::ObsInfo &obs, const bool &use_eftp,
                               const msquare::PlannerConfig &planner_config) {
  if (use_eftp) {
    obs.polygon_init.stop_offset =
        planner_config.longitudinal_motion_planner_config.eftp_stop_offset;
  } else {
    switch (obs.type) {
    case path_planner::ObsInfo::Type::TRANSPORT_TRUNK:
      obs.polygon_init.stop_offset =
          planner_config.longitudinal_motion_planner_config
              .transport_trunk_stop_offset;
      break;
    case path_planner::ObsInfo::Type::COUPE:
      obs.polygon_init.stop_offset =
          planner_config.longitudinal_motion_planner_config.couple_stop_offset;
      break;
    case path_planner::ObsInfo::Type::OFO:
      obs.polygon_init.stop_offset =
          planner_config.longitudinal_motion_planner_config.vru_stop_offset;
      break;
    case path_planner::ObsInfo::Type::PEDESTRIAN:
      obs.polygon_init.stop_offset =
          planner_config.longitudinal_motion_planner_config.vru_stop_offset;
      break;
    default:
      obs.polygon_init.stop_offset =
          planner_config.longitudinal_motion_planner_config.vru_stop_offset;
      break;
    }
  }
}

inline double compute_init_headway(const double &rel_s, const double &ego_v,
                                   const double &obj_v,
                                   const double &set_headway) {
  const double ego_dv = ego_v - obj_v;
  const double accel = 2.0;
  double init_set_distance =
      ego_dv > 0 ? std::fmax(0.0, rel_s - ego_dv * ego_dv / (2 * accel))
                 : std::fmax(0.0, rel_s + ego_dv * ego_dv / (2 * accel));
  double init_headway = init_set_distance / std::fmax(1.0, obj_v);
  double headway_real = std::fmax(0.0, rel_s) / std::fmax(1.0, obj_v);

  return std::fmin(
      std::fmax(0.5 * std::fmin(headway_real, set_headway), init_headway),
      set_headway);
}

inline std::vector<double>
get_headway_according_driving_style(const DrivingStyle &driving_style) {
  switch (driving_style) {
  case DrivingStyle::DRIVING_STYLE_AGGRESSIVE:
    return AGGRESSIVE_HEADWAY_TABLE;
  case DrivingStyle::DRIVING_STYLE_NORMAL:
    return NORMAL_HEADWAY_TABLE;
  case DrivingStyle::DRIVING_STYLE_CONSERVATIVE:
    return CONSERVATIVE_HEADWAY_TABLE;
  default:
    return NORMAL_HEADWAY_TABLE;
  }
}

inline std::vector<double>
get_headway_according_style(const bool is_lane_change,
                            const LaneChangingStyle &lane_changing_style,
                            const DrivingStyle &driving_style) {
  if (is_lane_change) {
    switch (lane_changing_style) {
    case LaneChangingStyle::LANECHANGING_STYLE_AGGRESSIVE:
      return AGGRESSIVE_HEADWAY_TABLE;
    case LaneChangingStyle::LANECHANGING_STYLE_NORMAL:
      return get_headway_according_driving_style(driving_style);
    case LaneChangingStyle::LANECHANGING_STYLE_CONSERVATIVE:
      return CONSERVATIVE_HEADWAY_TABLE;
    default:
      return get_headway_according_driving_style(driving_style);
    }
  } else {
    return get_headway_according_driving_style(driving_style);
  }
}

void ObstacleHeadWayPreprocessor::process() {
  object_extraction();
  update_headway();
}

void ObstacleHeadWayPreprocessor::object_extraction() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  auto &obstacle_extra_info_map = speed_planner_input->obstacle_extra_info_map;
  const auto &cipv_info = speed_planner_input->cipv_info;

  auto lane_status = context_->planning_status().lane_status;
  bool is_lane_change = lane_status.status == LaneStatus::Status::LANE_CHANGE;
  const auto headway_table = get_headway_according_style(
      is_lane_change, world_model_->get_lane_changing_style(),
      world_model_->get_driving_style());
  const double desired_headway =
      headway_table[world_model_->get_navi_ttc_gear()];

  const auto &obstacle_manager = baseline_info_->obstacle_manager();
  const auto &obstacle_decision_manager =
      context_->mutable_obstacle_decision_manager();
  const auto &ego_state =
      baseline_info_->get_ego_state_manager().get_ego_state();
  const double min_headway =
      ConfigurationContext::Instance()
          ->planner_config()
          .longitudinal_motion_planner_config.min_headway;
  MSD_LOG(INFO, "min_headway is %f", min_headway);

  const auto &lc_status = context_->lateral_behavior_planner_output().lc_status;
  speed_planner_input->is_lane_change =
      lc_status == "left_lane_change" || lc_status == "right_lane_change";

  for (auto &obs : obstacle_extra_info_map) {
    obs.second.is_alive = false;
  }

  // adjust headway according scene
  for (const auto &obs : speed_planner_input->lon_obs) {
    auto ptr_obstacle_decision =
        obstacle_decision_manager.find_obstacle_decision(obs.id);
    bool in_origin_lane = false;
    if (ptr_obstacle_decision) {
      in_origin_lane =
          ptr_obstacle_decision->LongitudinalDecision().has_follow() &&
          ptr_obstacle_decision->LongitudinalDecision().follow().in_origin_lane;
    }

    if (obstacle_extra_info_map.find(obs.id) == obstacle_extra_info_map.end()) {
      if (speed_planner_input->is_lane_change) {
        // origin lane obstalce at lane change scene
        if (in_origin_lane) {
          auto &obs_info = obstacle_extra_info_map[obs.id];
          obs_info.counter = 0;
          obs_info.is_alive = true;
          obs_info.type = ObstacleExtraInfo::ObstacleType::IN_ORIGIN_LANE;
        }
      } else {
        // fist apperance CIPV, adjust HeadWay
        bool start_compute_init_headway = false;
        if (!speed_planner_input->use_eftp) {
          start_compute_init_headway = true;
        } else if (cipv_info.cipv_id != -1 &&
                   cipv_info.cipv_time >
                       ENALE_ACC_DECEL_REGION_TIME_AT_EFTP_MODE) {
          start_compute_init_headway = true;
        }
        if (cipv_info.cipv_id == obs.id && start_compute_init_headway) {
          // has cipv
          auto &obs_info = obstacle_extra_info_map[obs.id];
          obs_info.counter = 0;
          obs_info.is_alive = true;
          obs_info.type = ObstacleExtraInfo::ObstacleType::IS_CIPV;
          double init_headway =
              compute_init_headway(obs.polygon_init.rel_s, ego_state.ego_vel,
                                   obs.polygon_init.v, desired_headway);
          obs_info.init_headway = std::fmax(min_headway, init_headway);
          // obs_info.init_headway = std::fmax(
          //     min_headway * desired_headway,
          //     obs.polygon_init.rel_s / std::fmax(1.0, obs.polygon_init.v));
        }
      }
    } else {
      auto &obs_info = obstacle_extra_info_map[obs.id];
      obs_info.is_alive = true;
      ++obs_info.counter;
      obs_info.duration_s = obs_info.counter * 0.1;
    }
  }

  erase_invalid_obstacle(obstacle_extra_info_map,
                         speed_planner_input->is_lane_change);
}

void ObstacleHeadWayPreprocessor::update_headway() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  const double headway_adjust_rate =
      ConfigurationContext::Instance()
          ->planner_config()
          .longitudinal_motion_planner_config.headway_adjust_rate;
  const double lane_change_headway_rate =
      ConfigurationContext::Instance()
          ->planner_config()
          .longitudinal_motion_planner_config.lane_change_headway_rate;
  MSD_LOG(INFO, "headway_adjust_rate is %f", headway_adjust_rate);
  const auto &obstacle_extra_info_map =
      speed_planner_input->obstacle_extra_info_map;

  auto lane_status = context_->planning_status().lane_status;
  bool is_lane_change = lane_status.status == LaneStatus::Status::LANE_CHANGE;
  const auto headway_table = get_headway_according_style(
      is_lane_change, world_model_->get_lane_changing_style(),
      world_model_->get_driving_style());
  const double desired_headway =
      headway_table[world_model_->get_navi_ttc_gear()];

  for (auto &obs : speed_planner_input->lon_obs) {
    if (obstacle_extra_info_map.find(obs.id) != obstacle_extra_info_map.end()) {
      const auto &obstacle_extra_info = obstacle_extra_info_map.at(obs.id);
      switch (obstacle_extra_info.type) {
      case ObstacleExtraInfo::ObstacleType::IS_CIPV:
        obs.polygon_init.desired_headway =
            std::fmin(desired_headway,
                      obstacle_extra_info.init_headway +
                          obstacle_extra_info.duration_s / headway_adjust_rate);
        break;
      case ObstacleExtraInfo::ObstacleType::IN_ORIGIN_LANE:
        obs.polygon_init.desired_headway =
            std::fmax(LANE_CHANGE_HEADWAY_MIN,
                      desired_headway - obstacle_extra_info.duration_s *
                                            lane_change_headway_rate);
        break;

      default:
        obs.polygon_init.desired_headway = desired_headway;
        break;
      }
    } else {
      obs.polygon_init.desired_headway = desired_headway;
    }

    const bool ACC_OvertakeAssistSwitch =
        ConfigurationContext::Instance()
            ->planner_config()
            .longitudinal_motion_planner_config.acc_overtake_assist_switch;
    speed_planner_input->acc_overtake_assist_switch = ACC_OvertakeAssistSwitch;
    if (world_model_->is_acc_mode() && ACC_OvertakeAssistSwitch) {
      lc_assist_decide();
    }
    // adjust offset according obstacle type
    const auto &planner_config =
        ConfigurationContext::Instance()->planner_config();
    update_stop_offset(obs, speed_planner_input->use_eftp, planner_config);
  }

  set_headway_from_gap_select_decison();
}

void ObstacleHeadWayPreprocessor::set_headway_from_gap_select_decison() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  const auto &select_gap_decision_info =
      PlanningContext::Instance()->general_motion_planner_output();
  const auto &lead_objs_info = select_gap_decision_info.lead_objs_info;
  if (!select_gap_decision_info.gmp_valid || lead_objs_info.size() < 2)
    return;

  for (auto &obs : speed_planner_input->lon_obs) {
    for (const auto &gap_obs : lead_objs_info)
      if (gap_obs.id != -1 && gap_obs.id == obs.id) {
        obs.polygon_init.desired_headway = gap_obs.headaway;
      }
  }
}

void ObstacleHeadWayPreprocessor::lc_assist_decide() {
  const auto obstacles_items =
      baseline_info_->obstacle_manager().get_obstacles().Items();
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  double cipv_v = 0;
  double cipv_s = 0;
  bool find_cipv = false;
  auto const &ego_state = baseline_info_->get_ego_state();
  double ego_s = ego_state.ego_frenet.x;
  double ego_v = ego_state.ego_vel;
  double lane_width = 3.75;
  bool left_acc_obj = true;
  bool left_dec_obj = false;
  bool right_acc_obj = true;
  bool right_dec_obj = false;
  double half_width =
      ConfigurationContext::Instance()->get_vehicle_param().width / 2.0;
  double K_LC_wait_time_max = 3.0;
  auto &lc_info = speed_planner_input->ACC_lc_assist_info;
  auto vehicle_light_status =
      world_model_->get_vehicle_status()
          .vehicle_light.vehicle_light_data.turn_signal_type.value;
  const auto headway_table = get_headway_according_style(
      false, world_model_->get_lane_changing_style(),
      world_model_->get_driving_style());
  const double desired_headway =
      headway_table[world_model_->get_navi_ttc_gear()];
  const double lane_change_headway_rate =
      ConfigurationContext::Instance()
          ->planner_config()
          .longitudinal_motion_planner_config.lane_change_headway_rate;
  const double cycle = 0.1;
  double nearest_obs_dist = 200.0;
  double nearest_obs_spd = 0.0;
  int nearest_obs_id = -1;
  MSD_LOG(INFO, "DEBUG_LC------------------------------------");
  MSD_LOG(INFO, "DEBUG_LC_light: %d", vehicle_light_status);
  if (vehicle_light_status == TurnSignalType::LEFT ||
      vehicle_light_status == TurnSignalType::RIGHT) {
    if (speed_planner_input->cipv_info.cipv_id > 0) {
      lc_info.lc_cipv_id = speed_planner_input->cipv_info.cipv_id;
    } else {
      lc_info.lc_cipv_id = -1;
    }
    lc_info.turn_signal = vehicle_light_status;
    lc_info.is_ACC_lane_change = true;
    speed_planner_input->is_ACC_lane_change = true;
    MSD_LOG(INFO, "DEBUG_LC_is_lc: %d", lc_info.is_ACC_lane_change);
  } else {
    lc_info.lc_cipv_id = -1;
    lc_info.left_acc = false;
    lc_info.left_dec = false;
    lc_info.right_acc = false;
    lc_info.right_dec = false;
    lc_info.adjust_cnt = 0;
    lc_info.ACC_lane_change_cnt = 0;
    lc_info.is_ACC_lane_change = false;
    lc_info.turn_signal = vehicle_light_status;
    speed_planner_input->is_ACC_lane_change = false;
    MSD_LOG(INFO, "DEBUG_LC:no light so lc_info is null");
  }
  MSD_LOG(INFO, "DEBUG_LC_cipv_id: %d", speed_planner_input->cipv_info.cipv_id);
  MSD_LOG(INFO, "DEBUG_LC_lc_cipv_id: %d", lc_info.lc_cipv_id);
  for (const auto &obj : speed_planner_input->lon_obs) {
    if (obj.id == lc_info.lc_cipv_id) {
      find_cipv = true;
      cipv_v = obj.polygon_init.v_frenet;
      cipv_s = obj.polygon_init.s;
    }
  }
  if (find_cipv == false) {
    left_dec_obj = false;
    left_acc_obj = false;
    right_dec_obj = false;
    right_acc_obj = false;
    lc_info.left_acc = false;
    lc_info.left_dec = false;
    lc_info.right_acc = false;
    lc_info.right_dec = false;
    lc_info.adjust_cnt = 0;
    MSD_LOG(INFO, "DEBUG_LC:no cipv");
  } else {
    for (const auto &obstacle : obstacles_items) {
      double theta =
          planning_math::NormalizeAngle(obstacle->Yaw_relative_frenet());
      double obj_vx = obstacle->speed() * std::cos(theta);
      if (obstacle->S_frenet() > ego_s && obstacle->S_frenet() < cipv_s &&
          obj_vx <
              cipv_v + (cipv_s - obstacle->S_frenet()) / K_LC_wait_time_max) {
        if (obstacle->R_frenet() > lane_width / 2 + half_width &&
            obstacle->R_frenet() < 1.5 * lane_width + half_width) {
          left_dec_obj = true;
          left_acc_obj = false;
          if (obstacle->S_frenet() < nearest_obs_dist &&
              obstacle->S_frenet() > ego_s) {
            nearest_obs_dist = obstacle->S_frenet();
            nearest_obs_spd = obj_vx;
            nearest_obs_id = obstacle->Id();
          }
        } else if (obstacle->R_frenet() < -lane_width / 2 - half_width &&
                   obstacle->R_frenet() > -1.5 * lane_width - half_width) {
          right_dec_obj = true;
          right_acc_obj = false;
          if (obstacle->S_frenet() < nearest_obs_dist &&
              obstacle->S_frenet() > ego_s) {
            nearest_obs_dist = obstacle->S_frenet();
            nearest_obs_spd = obj_vx;
            nearest_obs_id = obstacle->Id();
          }
        }
      }
    }
  }

  double headway_rate = lane_change_headway_rate;
  if (vehicle_light_status == TurnSignalType::LEFT && left_dec_obj ||
      vehicle_light_status == TurnSignalType::RIGHT && right_dec_obj) {
    if (lc_info.adjust_cnt >= 0) {
      double need_time;
      double ds = nearest_obs_dist - ego_s;
      double desired_dist = desired_headway * nearest_obs_spd + 4.5;
      if (ego_v > nearest_obs_spd && ds < desired_dist) {
        lc_info.adjust_cnt += 4;
        need_time = std::fmin(10.0, (ego_v - nearest_obs_spd) / 2.0);
        need_time = std::fmax(1.0, need_time);
      } else {
        lc_info.adjust_cnt += 2;
        need_time = std::fmin(10.0, std::fabs(ego_v - nearest_obs_spd) / 1.0);
        need_time = std::fmax(1.0, need_time);
      }
      headway_rate =
          std::fmin(std::fmax(10.0 / need_time, lane_change_headway_rate),
                    2.0 * lane_change_headway_rate);
    } else {
      lc_info.adjust_cnt = 0;
    }
    lc_info.adjust_cnt = std::fmin(lc_info.adjust_cnt, 20);
    if (vehicle_light_status == TurnSignalType::LEFT) {
      lc_info.left_acc = false;
      lc_info.left_dec = true;
      lc_info.right_acc = false;
      lc_info.right_dec = false;
      MSD_LOG(INFO, "DEBUG_LC: left_dec");
    } else if (vehicle_light_status == TurnSignalType::RIGHT) {
      lc_info.left_acc = false;
      lc_info.left_dec = false;
      lc_info.right_acc = false;
      lc_info.right_dec = true;
      MSD_LOG(INFO, "DEBUG_LC: rigth_dec");
    }
    MSD_LOG(INFO, "DEBUG_LC_adjust_cnt: %d", lc_info.adjust_cnt);
    MSD_LOG(INFO, "DEBUG_LC_nearest_obs_id: %d", nearest_obs_id);
  } else if (vehicle_light_status == TurnSignalType::LEFT && left_acc_obj ||
             vehicle_light_status == TurnSignalType::RIGHT && right_acc_obj) {
    if (lc_info.adjust_cnt <= 0) {
      lc_info.adjust_cnt -= 2;
    } else {
      lc_info.adjust_cnt = 0;
    }
    lc_info.adjust_cnt = std::fmax(lc_info.adjust_cnt, -20);
    if (vehicle_light_status == TurnSignalType::LEFT) {
      lc_info.left_acc = true;
      lc_info.left_dec = false;
      lc_info.right_acc = false;
      lc_info.right_dec = false;
      MSD_LOG(INFO, "DEBUG_LC: left_acc");
    } else if (vehicle_light_status == TurnSignalType::RIGHT) {
      lc_info.left_acc = false;
      lc_info.left_dec = false;
      lc_info.right_acc = true;
      lc_info.right_dec = false;
      MSD_LOG(INFO, "DEBUG_LC: right_acc");
    }
    MSD_LOG(INFO, "DEBUG_LC_adjust_cnt: %d", lc_info.adjust_cnt);
  }
  for (auto &obj : speed_planner_input->lon_obs) {
    if (obj.id == lc_info.lc_cipv_id) {
      MSD_LOG(INFO, "DEBUG_LC_cipv_init_desi_hw: %f",
              obj.polygon_init.desired_headway);
      MSD_LOG(INFO, "DEBUG_LC_look_table_hw: %f", desired_headway);
      if (lc_info.ACC_lane_change_cnt < 10.0 / cycle) {
        MSD_LOG(INFO, "DEBUG_LC:cnt < 100");
        obj.polygon_init.desired_headway +=
            lc_info.adjust_cnt * cycle * headway_rate;
        obj.polygon_init.desired_headway = std::fmax(
            obj.polygon_init.desired_headway, LANE_CHANGE_HEADWAY_MIN);
        obj.polygon_init.desired_headway = std::fmin(
            obj.polygon_init.desired_headway, LANE_CHAGNE_HEADWAY_MAX);
      } else {
        if (std::fabs(obj.polygon_init.desired_headway - desired_headway) >
            0.1) {
          if (obj.polygon_init.desired_headway > desired_headway) {
            MSD_LOG(INFO, "DEBUG_LC:cnt > 100 & obs_desired_hw > desired_hw");
            obj.polygon_init.desired_headway -= cycle * headway_rate;
            obj.polygon_init.desired_headway =
                std::fmax(desired_headway, obj.polygon_init.desired_headway);
          } else {
            MSD_LOG(INFO, "DEBUG_LC:cnt > 100 & obs_desired_hw < desired_hw");
            obj.polygon_init.desired_headway += cycle * headway_rate;
            obj.polygon_init.desired_headway =
                std::fmin(desired_headway, obj.polygon_init.desired_headway);
          }
        }
      }
      MSD_LOG(INFO, "DEBUG_LC_cipv_modify_desi_hw: %f",
              obj.polygon_init.desired_headway);
      if ((vehicle_light_status == TurnSignalType::LEFT ||
           vehicle_light_status == TurnSignalType::RIGHT) &&
          obj.polygon_init.desired_headway >= LANE_CHANGE_HEADWAY_MIN &&
          obj.polygon_init.desired_headway <= LANE_CHAGNE_HEADWAY_MAX) {
        lc_info.ACC_lane_change_cnt++;
        MSD_LOG(INFO, "DEBUG_LC:cnt increase");
      } else {
        lc_info.ACC_lane_change_cnt = 0;
        MSD_LOG(INFO, "DEBUG_LC:cnt no change");
      }
      MSD_LOG(INFO, "DEBUG_LC_HW: hw1:%f", obj.polygon_init.desired_headway);
    }
  }
  speed_planner_input->ACC_lane_change_cnt = lc_info.ACC_lane_change_cnt;
  MSD_LOG(INFO, "DEBUG_LC_lc_cnt: %d", lc_info.ACC_lane_change_cnt);
}

} // namespace msquare
