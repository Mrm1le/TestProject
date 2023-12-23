#include "planner/behavior_planner/lateral_behavior_planner.h"
#include "planning/common/logging.h"

namespace msquare {

void LateralBehaviorPlanner::update_avd_info(
    std::vector<double> &avd_car_info,
    const std::vector<const TrackedObject *> &front_side_tracks, double ego_l,
    int avd_priority) {

  auto &map_info = world_model_->get_map_info();
  auto &map_info_mgr = world_model_->get_map_info_manager();
  auto &lateral_obstacle = world_model_->mutable_lateral_obstacle();
  auto &ego_state = baseline_info_->get_ego_state();
  auto &lateral_output = context_->lateral_behavior_planner_output();
  auto &state_machine_output = context_->state_machine_output();
  auto &flane = virtual_lane_mgr_->get_fix_lane();
  double dist_to_center_line = flane.dist_to_center_line();

  int state = state_machine_output.curr_state;

  double lat_offset = path_planner_->lat_offset();
  double lane_width = path_planner_->lane_width();
  double v_ego = ego_state.ego_vel;
  double l_ego = ego_state.ego_frenet.y;
  float blocked_time_begin = 0.0;
  float blocked_time_end = 5.0;

  TrackedObject *lead_one = lateral_obstacle.leadone();

  if (avd_car_info.size() == 0) {
    return;
  }

  int first_ncar_type = 1;
  int track_id = (int)avd_car_info[0];
  int default_avd_priority = 100;

  if (avd_car_info[5] > lat_offset) {
    if (track_id != 0 && track_id != -1 &&
        avd_info_.find(track_id) == avd_info_.end()) {
      avd_info_[track_id] = {track_id,        "NBO",        true,
                             "right",         avd_priority, blocked_time_begin,
                             blocked_time_end};

      for (auto tr : front_side_tracks) {
        if (tr->track_id != track_id &&
            ((tr->d_rel >= 0 &&
              tr->d_rel <
                  std::max(std::min(std::fabs(15 * tr->v_rel), 60.0), 20.0) &&
              tr->v_rel < 1) ||
             (tr->d_rel > std::min(-10.0 - tr->v_rel, -5.0) &&
              tr->d_rel < 0))) {
          if (tr->v_lat > -0.5 &&
              ((tr->type < 10000 && (tr->trajectory.intersection == 0 ||
                                     (tr->trajectory.intersection < 10000 &&
                                      !map_info.is_in_intersection()))) ||
               tr->type > 10000) &&
              lat_offset - tr->d_max_cpath >= 1.3 &&
              lat_offset - tr->d_max_cpath <= 1.5 * lane_width &&
              -dist_to_center_line - tr->d_max_cpath >= 1.3 &&
              -dist_to_center_line - tr->d_max_cpath <= 1.5 * lane_width) {
            avd_info_[tr->track_id] = {tr->track_id,
                                       "NBO",
                                       true,
                                       "left",
                                       default_avd_priority,
                                       blocked_time_begin,
                                       blocked_time_end};
          } else if (((tr->d_max_cpath < 0 && tr->d_rel >= 0) ||
                      (tr->d_max_cpath < -1.1 && tr->d_rel < 0 &&
                       tr->v_rel > 1.0)) &&
                     tr->d_max_cpath > -2.5 * lane_width &&
                     tr->trajectory.intersection != 10000) {
            avd_info_[tr->track_id] = {tr->track_id,
                                       "NBO",
                                       false,
                                       "left",
                                       default_avd_priority,
                                       blocked_time_begin,
                                       blocked_time_end};
          }
        }
      }

      for (auto tr : front_side_tracks) {
        if (tr->track_id == track_id) {
          first_ncar_type = tr->type;
          if (std::fabs(avd_car_info[1] - tr->trajectory.intersection) > 1e-5) {
            avd_car_info[1] = tr->trajectory.intersection;
          }

          break;
        }
      }

      if (avd_car_info[5] - lat_offset < 1.25 || avd_car_info[4] < -0.5 ||
          (std::fabs(avd_car_info[1]) > 1e-5 && first_ncar_type < 10000 &&
           map_info.is_in_intersection()) ||
          (state == ROAD_NONE && dist_to_center_line + avd_car_info[5] < 1.3 &&
           avd_car_info[3] < 1 && avd_car_info[3] > -3.5)) {
        avd_info_[track_id].ignore = false;
      }
      if ((std::fabs(avd_car_info[1] - 10000) < 1e-5) &&
          (!(avd_car_info[3] > -3.5 && avd_car_info[3] < 1 &&
             avd_car_info[5] - l_ego < 1.5))) {
        avd_info_.erase(track_id);
      }
    } else if (track_id == 0 && avd_car_info.size() == 11) {
      int temp_track_id = (int)avd_car_info[10];
      avd_car_info[0] = temp_track_id;

      if (avd_info_.find(temp_track_id) == avd_info_.end()) {
        avd_info_[temp_track_id] = {
            temp_track_id,      "NBO",           true, "right", avd_priority,
            blocked_time_begin, blocked_time_end};

        for (auto tr : front_side_tracks) {
          if (tr->track_id != temp_track_id &&
              ((tr->d_rel >= 0 &&
                tr->d_rel <
                    std::max(std::min(std::fabs(15 * tr->v_rel), 60.0), 20.0) &&
                tr->v_rel < 1) ||
               (tr->d_rel > std::min(-10.0 - tr->v_rel, -5.0) &&
                tr->d_rel < 0))) {
            if (tr->v_lat > -0.5 &&
                ((tr->type < 10000 && (tr->trajectory.intersection == 0 ||
                                       (tr->trajectory.intersection < 10000 &&
                                        !map_info.is_in_intersection()))) ||
                 tr->type > 10000) &&
                lat_offset - tr->d_max_cpath >= 1.3 &&
                lat_offset - tr->d_max_cpath <= 1.5 * lane_width &&
                -dist_to_center_line - tr->d_max_cpath >= 1.3 &&
                -dist_to_center_line - tr->d_max_cpath <= 1.5 * lane_width) {
              avd_info_[tr->track_id] = {tr->track_id,
                                         "NBO",
                                         true,
                                         "left",
                                         default_avd_priority,
                                         blocked_time_begin,
                                         blocked_time_end};
            } else if (((tr->d_max_cpath < 0 && tr->d_rel >= 0) ||
                        (tr->d_max_cpath < -1.1 && tr->d_rel < 0 &&
                         tr->v_rel > 1.0)) &&
                       tr->d_max_cpath > -2.5 * lane_width &&
                       tr->trajectory.intersection != 10000) {
              avd_info_[tr->track_id] = {tr->track_id,
                                         "NBO",
                                         false,
                                         "left",
                                         default_avd_priority,
                                         blocked_time_begin,
                                         blocked_time_end};
            }
          }
        }

        for (auto tr : front_side_tracks) {
          if (tr->track_id == temp_track_id) {
            first_ncar_type = tr->type;
            if (std::fabs(avd_car_info[1] - tr->trajectory.intersection) >
                1e-5) {
              avd_car_info[1] = tr->trajectory.intersection;
            }

            break;
          }
        }

        if (avd_car_info[5] - lat_offset < 1.25 || avd_car_info[4] < -0.5 ||
            (std::fabs(avd_car_info[1]) > 1e-5 && first_ncar_type < 10000 &&
             map_info.is_in_intersection()) ||
            (state == ROAD_NONE &&
             dist_to_center_line + avd_car_info[5] < 1.3 &&
             avd_car_info[3] < 1 && avd_car_info[3] > -3.5)) {
          avd_info_[temp_track_id].ignore = false;
        }
        if ((std::fabs(avd_car_info[1] - 10000) < 1e-5) &&
            (!(avd_car_info[3] > -3.5 && avd_car_info[3] < 1 &&
               avd_car_info[5] - l_ego < 1.5))) {
          avd_info_.erase(temp_track_id);
        }
      }
    } else if (track_id == -1 && lead_one != nullptr &&
               avd_info_.find(lead_one->track_id) == avd_info_.end()) {
      avd_info_[lead_one->track_id] = {
          lead_one->track_id, "NBO",           true, "right", avd_priority,
          blocked_time_begin, blocked_time_end};
      avd_car_info[0] = lead_one->track_id;
      if (lead_one->d_max_cpath < -1.1 && lead_one->v_lat < -0.3 &&
          lead_one->v_rel > 1.0) {
        avd_info_[lead_one->track_id].avd_direction = "left";
      }

      for (auto tr : front_side_tracks) {
        if (tr->track_id != lead_one->track_id &&
            ((tr->d_rel >= 0 &&
              tr->d_rel <
                  std::max(std::min(std::fabs(15 * tr->v_rel), 60.0), 20.0) &&
              tr->v_rel < 1) ||
             (tr->d_rel > std::min(-10.0 - tr->v_rel, -5.0) &&
              tr->d_rel < 0))) {
          if (tr->v_lat > -0.5 &&
              ((tr->type < 10000 && (tr->trajectory.intersection == 0 ||
                                     (tr->trajectory.intersection < 10000 &&
                                      !map_info.is_in_intersection()))) ||
               tr->type > 10000) &&
              lat_offset - tr->d_max_cpath >= 1.3 &&
              lat_offset - tr->d_max_cpath <= 1.5 * lane_width &&
              -dist_to_center_line - tr->d_max_cpath >= 1.3 &&
              -dist_to_center_line - tr->d_max_cpath <= 1.5 * lane_width) {
            avd_info_[tr->track_id] = {tr->track_id,
                                       "NBO",
                                       true,
                                       "left",
                                       default_avd_priority,
                                       blocked_time_begin,
                                       blocked_time_end};
          } else if (((tr->d_max_cpath < 0 && tr->d_rel >= 0) ||
                      (tr->d_max_cpath < -1.1 && tr->d_rel < 0 &&
                       tr->v_rel > 1.0)) &&
                     tr->d_max_cpath > -2.5 * lane_width &&
                     tr->trajectory.intersection != 10000) {
            avd_info_[tr->track_id] = {tr->track_id,
                                       "NBO",
                                       false,
                                       "left",
                                       default_avd_priority,
                                       blocked_time_begin,
                                       blocked_time_end};
          }
        } else if (tr->track_id == lead_one->track_id &&
                   std::fabs(avd_car_info[1] - tr->trajectory.intersection) >
                       1e-5) {
          avd_car_info[1] = tr->trajectory.intersection;
        }
      }

      if (lead_one->d_min_cpath - lat_offset < 1.25 ||
          lead_one->d_path_self < 1.25 || lead_one->v_lat < -0.5 ||
          (std::fabs(avd_car_info[1]) > 1e-5 && lead_one->type < 10000 &&
           map_info.is_in_intersection()) ||
          (state == ROAD_NONE && dist_to_center_line + avd_car_info[5] < 1.3 &&
           avd_car_info[3] < 1 && avd_car_info[3] > -3.5)) {
        avd_info_[lead_one->track_id].ignore = false;
      }
      if ((std::fabs(avd_car_info[1] - 10000) < 1e-5 ||
           lead_one->v_lat < -1.) &&
          (!(avd_car_info[3] > -3.5 && avd_car_info[3] < 1 &&
             avd_car_info[5] - l_ego < 1.5))) {
        avd_info_.erase(lead_one->track_id);
      }
    }
  } else if (std::fabs(lat_offset) > 1e-5) {
    if (track_id != 0 && track_id != -1 &&
        avd_info_.find(track_id) == avd_info_.end()) {
      avd_info_[track_id] = {track_id,        "NBO",        true,
                             "left",          avd_priority, blocked_time_begin,
                             blocked_time_end};
      if (lat_offset < avd_car_info[6] + 1.3 &&
          lane_width / 2 - avd_car_info[6] < 2.4 &&
          lateral_output.scenario == LOCATION_ROAD &&
          map_info.lanes_num() == 1 && avd_car_info[3] > 3 &&
          avd_car_info[5] > 2.4 - lane_width / 2 &&
          std::fabs(avd_car_info[7] - 20001) < 1e-5) {
        avd_info_[track_id].avd_direction = "right";
      }

      for (auto tr : front_side_tracks) {
        if (tr->track_id != track_id &&
            ((tr->d_rel >= 0 &&
              tr->d_rel <
                  std::max(std::min(std::fabs(15 * tr->v_rel), 60.0), 20.0) &&
              tr->v_rel < 1) ||
             (tr->d_rel > std::min(-10.0 - tr->v_rel, -5.0) &&
              tr->d_rel < 0))) {
          if (tr->v_lat > -0.5 &&
              ((tr->type < 10000 && (tr->trajectory.intersection == 0 ||
                                     (tr->trajectory.intersection < 10000 &&
                                      !map_info.is_in_intersection()))) ||
               tr->type > 10000) &&
              -lat_offset + tr->d_min_cpath >= 1.3 &&
              -lat_offset + tr->d_min_cpath <= 1.5 * lane_width &&
              dist_to_center_line + tr->d_min_cpath >= 1.3 &&
              dist_to_center_line + tr->d_min_cpath <= 1.5 * lane_width) {
            avd_info_[tr->track_id] = {tr->track_id,
                                       "NBO",
                                       true,
                                       "right",
                                       default_avd_priority,
                                       blocked_time_begin,
                                       blocked_time_end};
          } else if (((tr->d_min_cpath > 0 && tr->d_rel >= 0) ||
                      (tr->d_min_cpath > 1.1 && tr->d_rel < 0 &&
                       tr->v_rel > 1.0)) &&
                     tr->d_min_cpath < 2.5 * lane_width &&
                     tr->trajectory.intersection != 10000) {
            avd_info_[tr->track_id] = {tr->track_id,
                                       "NBO",
                                       false,
                                       "right",
                                       default_avd_priority,
                                       blocked_time_begin,
                                       blocked_time_end};
          }
        }
      }

      for (auto tr : front_side_tracks) {
        if (tr->track_id == track_id) {
          first_ncar_type = tr->type;
          if (std::fabs(avd_car_info[1] - tr->trajectory.intersection) > 1e-5) {
            avd_car_info[1] = tr->trajectory.intersection;
          }

          break;
        }
      }

      if (lat_offset - avd_car_info[6] < 1.3 || avd_car_info[4] < -0.5 ||
          (std::fabs(avd_car_info[1]) > 1e-5 && first_ncar_type < 10000 &&
           map_info.is_in_intersection())) {
        avd_info_[track_id].ignore = false;
      }
      if ((std::fabs(avd_car_info[1] - 10000) < 1e-5) &&
          (!(avd_car_info[3] > -3.5 && avd_car_info[3] < 1 &&
             l_ego - avd_car_info[6] < 1.5))) {
        avd_info_.erase(track_id);
      }
    } else if (track_id == 0 && avd_car_info.size() == 11) {
      int temp_track_id = (int)avd_car_info[10];
      avd_car_info[0] = temp_track_id;

      if (avd_info_.find(temp_track_id) == avd_info_.end()) {
        avd_info_[temp_track_id] = {
            temp_track_id,      "NBO",           true, "left", avd_priority,
            blocked_time_begin, blocked_time_end};
        if (lat_offset < avd_car_info[6] + 1.3 &&
            lane_width / 2 - avd_car_info[6] < 2.4 &&
            lateral_output.scenario == LOCATION_ROAD &&
            map_info.lanes_num() == 1 && avd_car_info[3] > 3 &&
            avd_car_info[5] > 2.4 - lane_width / 2 &&
            std::fabs(avd_car_info[7] - 20001) < 1e-5) {
          avd_info_[temp_track_id].avd_direction = "right";
        }

        for (auto tr : front_side_tracks) {
          if (tr->track_id != temp_track_id &&
              ((tr->d_rel >= 0 &&
                tr->d_rel <
                    std::max(std::min(std::fabs(15 * tr->v_rel), 60.0), 20.0) &&
                tr->v_rel < 1) ||
               (tr->d_rel > std::min(-10.0 - tr->v_rel, -5.0) &&
                tr->d_rel < 0))) {
            if (tr->v_lat > -0.5 &&
                ((tr->type < 10000 && (tr->trajectory.intersection == 0 ||
                                       (tr->trajectory.intersection < 10000 &&
                                        !map_info.is_in_intersection()))) ||
                 tr->type > 10000) &&
                -lat_offset + tr->d_min_cpath >= 1.3 &&
                -lat_offset + tr->d_min_cpath <= 1.5 * lane_width &&
                dist_to_center_line + tr->d_min_cpath >= 1.3 &&
                dist_to_center_line + tr->d_min_cpath <= 1.5 * lane_width) {
              avd_info_[tr->track_id] = {tr->track_id,
                                         "NBO",
                                         true,
                                         "right",
                                         default_avd_priority,
                                         blocked_time_begin,
                                         blocked_time_end};
            } else if (((tr->d_min_cpath > 0 && tr->d_rel >= 0) ||
                        (tr->d_min_cpath > 1.1 && tr->d_rel < 0 &&
                         tr->v_rel > 1.0)) &&
                       tr->d_min_cpath < 2.5 * lane_width &&
                       tr->trajectory.intersection != 10000) {
              avd_info_[tr->track_id] = {tr->track_id,
                                         "NBO",
                                         false,
                                         "right",
                                         default_avd_priority,
                                         blocked_time_begin,
                                         blocked_time_end};
            }
          }
        }

        for (auto tr : front_side_tracks) {
          if (tr->track_id == temp_track_id) {
            first_ncar_type = tr->type;
            if (std::fabs(avd_car_info[1] - tr->trajectory.intersection) >
                1e-5) {
              avd_car_info[1] = tr->trajectory.intersection;
            }

            break;
          }
        }

        if (lat_offset - avd_car_info[6] < 1.25 || avd_car_info[4] < -0.5 ||
            (std::fabs(avd_car_info[1]) > 1e-5 && first_ncar_type < 10000 &&
             map_info.is_in_intersection())) {
          avd_info_[temp_track_id].ignore = false;
        }
        if ((std::fabs(avd_car_info[1] - 10000) < 1e-5) &&
            (!(avd_car_info[3] > -3.5 && avd_car_info[3] < 1 &&
               l_ego - avd_car_info[6] < 1.5))) {
          avd_info_.erase(temp_track_id);
        }
      }
    } else if (track_id == -1 && lead_one != nullptr &&
               avd_info_.find(lead_one->track_id) == avd_info_.end()) {
      avd_info_[lead_one->track_id] = {
          lead_one->track_id, "NBO",           true, "left", avd_priority,
          blocked_time_begin, blocked_time_end};
      if (lead_one->d_min_cpath >= 0 && lead_one->type < 10000) {
        avd_info_[lead_one->track_id].avd_direction = "right";
      }
      if (lat_offset < avd_car_info[6] + 1.3 &&
          lane_width / 2 - avd_car_info[6] < 2.4 &&
          lateral_output.scenario == LOCATION_ROAD &&
          map_info.lanes_num() == 1 && int(avd_car_info[7]) == 20001) {
        avd_info_[lead_one->track_id].avd_direction = "right";
        avd_car_info[0] = lead_one->track_id;
      }

      for (auto tr : front_side_tracks) {
        if (tr->track_id != lead_one->track_id &&
            ((tr->d_rel >= 0 &&
              tr->d_rel <
                  std::max(std::min(std::fabs(15 * tr->v_rel), 60.0), 20.0) &&
              tr->v_rel < 1) ||
             (tr->d_rel > std::min(-10.0 - tr->v_rel, -5.0) &&
              tr->d_rel < 0))) {
          if (tr->v_lat > -0.5 &&
              ((tr->type < 10000 && (tr->trajectory.intersection == 0 ||
                                     (tr->trajectory.intersection < 10000 &&
                                      !map_info.is_in_intersection()))) ||
               tr->type > 10000) &&
              -lat_offset + tr->d_min_cpath >= 1.3 &&
              -lat_offset + tr->d_min_cpath <= 1.5 * lane_width &&
              dist_to_center_line + tr->d_min_cpath >= 1.3 &&
              dist_to_center_line + tr->d_min_cpath <= 1.5 * lane_width) {
            avd_info_[tr->track_id] = {tr->track_id,
                                       "NBO",
                                       true,
                                       "right",
                                       default_avd_priority,
                                       blocked_time_begin,
                                       blocked_time_end};
          } else if (((tr->d_min_cpath > 0 && tr->d_rel >= 0) ||
                      (tr->d_min_cpath > 1.1 && tr->d_rel < 0 &&
                       tr->v_rel > 1.0)) &&
                     tr->d_min_cpath < 2.5 * lane_width &&
                     tr->trajectory.intersection != 10000) {
            avd_info_[tr->track_id] = {tr->track_id,
                                       "NBO",
                                       false,
                                       "right",
                                       default_avd_priority,
                                       blocked_time_begin,
                                       blocked_time_end};
          }
        } else if (tr->track_id == lead_one->track_id &&
                   std::fabs(avd_car_info[1] - tr->trajectory.intersection) >
                       1e-5) {
          avd_car_info[1] = tr->trajectory.intersection;
        }
      }

      if (lat_offset - lead_one->d_max_cpath < 1.25 ||
          lead_one->d_path_self < 1.25 || lead_one->v_lat < -0.5 ||
          (int(avd_car_info[1]) != 0 && lead_one->type < 10000 &&
           map_info.is_in_intersection())) {
        avd_info_[lead_one->track_id].ignore = false;
      }
      if ((int(avd_car_info[1]) == 10000 || lead_one->v_lat < -1.) &&
          (!(avd_car_info[3] > -3.5 && avd_car_info[3] < 1 &&
             l_ego - avd_car_info[6] < 1.5))) {
        avd_info_.erase(lead_one->track_id);
      }
    }
  } else {
    if (track_id == 0 && avd_car_info.size() == 11) {
      track_id = avd_car_info[10];
    } else if (track_id == -1 && lead_one != nullptr) {
      track_id = lead_one->track_id;
    }
    for (auto tr : front_side_tracks) {
      if (tr->track_id == track_id) {
        if (std::fabs(avd_car_info[1] - tr->trajectory.intersection) > 1e-5) {
          avd_car_info[1] = tr->trajectory.intersection;
        }
        avd_car_info[3] = tr->d_rel;
        break;
      }
    }

    if (avd_car_info[5] > -0.5 && l_ego - avd_car_info[6] < 0.3 &&
        ((avd_car_info[2] + v_ego < 0.5 && abs(avd_car_info[4]) < 0.2 &&
          int(avd_car_info[1]) != 10000) ||
         (avd_car_info[3] > -3.5 && avd_car_info[3] < 1 &&
          avd_car_info[5] - l_ego < 1.5))) {
      avd_info_[track_id] = {track_id,        "NBO",        true,
                             "right",         avd_priority, blocked_time_begin,
                             blocked_time_end};
    } else if (avd_car_info[6] < 0.5 && avd_car_info[5] - l_ego < 0.3 &&
               ((avd_car_info[2] + v_ego < 0.5 && abs(avd_car_info[4]) < 0.2 &&
                 int(avd_car_info[1]) != 10000) ||
                (avd_car_info[3] > -3.5 && avd_car_info[3] < 1 &&
                 l_ego - avd_car_info[6] < 1.5)) &&
               (map_info.is_in_intersection() ||
                lane_width / 2 - avd_car_info[6] > 2.4)) {
      avd_info_[track_id] = {track_id,        "NBO",        true,
                             "left",          avd_priority, blocked_time_begin,
                             blocked_time_end};
    }
    if (int(avd_car_info[9]) == 0 &&
        avd_info_.find(track_id) != avd_info_.end() &&
        (avd_car_info[5] - l_ego < 0.7 || l_ego - avd_car_info[6] < 0.7)) {
      avd_info_.erase(track_id);
    }
  }
  if (avd_car_info.size() != 0 &&
      avd_info_.find(avd_car_info[0]) != avd_info_.end()) {
    for (auto &tr : lateral_obstacle.side_tracks()) {
      if (int(tr.track_id) == int(avd_car_info[0]) && tr.d_rel < -8.0) {
        avd_info_.erase(tr.track_id);
        break;
      }
    }
  }
}

void LateralBehaviorPlanner::update_avd_info() {
  avd_info_.clear();

  double ego_s = DBL_MAX;
  double ego_l = DBL_MAX;
  double theta = DBL_MAX;
  float blocked_time_begin = 0.0;
  float blocked_time_end = 5.0;
  int default_avd_priority = 100;

  auto &map_info = world_model_->get_map_info();
  auto &map_info_mgr = world_model_->get_map_info_manager();
  auto &state_machine_output = context_->state_machine_output();

  auto &f_refline = virtual_lane_mgr_->mutable_fix_refline();
  auto &tlane = virtual_lane_mgr_->mutable_target_lane();
  auto &lateral_obstacle = world_model_->mutable_lateral_obstacle();
  auto &ego_state = baseline_info_->get_ego_state();

  int state = state_machine_output.curr_state;
  int straight_light_status_at_right =
      world_model_->get_traffic_light_decision()
          ->get_straight_light_status_at_right();
  int fix_lane_index = map_info.current_lane_index() + f_refline.position();
  bool isRedLightStop =
      world_model_->get_traffic_light_decision()->get_stop_flag();
  bool isTrafficLight_in_field = world_model_->get_traffic_light_decision()
                                     ->get_is_traffic_light_in_field();

  double lat_offset = path_planner_->lat_offset();
  double lane_width = path_planner_->lane_width();
  double v_ego = ego_state.ego_vel;
  double l_ego = ego_state.ego_frenet.y;
  double car_width = 2.2;

  int lc_request = state_machine_output.lc_request;
  auto lead_one = lateral_obstacle.leadone();
  auto lead_two = lateral_obstacle.leadtwo();
  auto temp_leadone = lateral_obstacle.tleadone();
  auto temp_leadtwo = lateral_obstacle.tleadtwo();
  auto &lateral_output = context_->mutable_lateral_behavior_planner_output();
  auto traffic_light_direction = map_info.traffic_light_direction();
  uint8_t left_boundary_type = LaneBoundaryForm::UNKNOWN;
  uint8_t right_boundary_type = LaneBoundaryForm::UNKNOWN;
  double left_boundary_length = 0.;
  double right_boundary_length = 0.;
  bool obj_disappear = true;
  bool avd_car_past0_lost = true;
  bool avd_car_past1_lost = true;
  bool LCHANGE = (state == ROAD_LC_LCHANGE);
  bool RCHANGE = (state == ROAD_LC_RCHANGE);
  if (tlane.has_master() && virtual_lane_mgr_->is_on_lane(TARGET_LANE)) {
    if (map_info.left_boundary_info().size() > 0 &&
        map_info.right_boundary_info().size() > 0) {
      left_boundary_type = map_info.right_boundary_info()[0].type.value.value;
      left_boundary_length = map_info.right_boundary_info()[0].length;
      right_boundary_type = map_info.left_boundary_info()[0].type.value.value;
      right_boundary_length = map_info.left_boundary_info()[0].length;
    } else if (map_info.right_boundary_info().size() > 0) {
      left_boundary_type = map_info.right_boundary_info()[0].type.value.value;
      left_boundary_length = map_info.right_boundary_info()[0].length;
      right_boundary_type = LaneBoundaryForm::DASH;
      right_boundary_length = std::max(map_info.dist_to_intsect() - 30.,
                                       map_info.dist_to_last_intsect());
    } else if (map_info.left_boundary_info().size() > 0) {
      left_boundary_type = LaneBoundaryForm::DASH;
      left_boundary_length = std::max(map_info.dist_to_intsect() - 30.,
                                      map_info.dist_to_last_intsect());
      right_boundary_type = map_info.left_boundary_info()[0].type.value.value;
      right_boundary_length = map_info.left_boundary_info()[0].length;
    }
  } else {
    if (map_info.left_boundary_info().size() > 0) {
      left_boundary_type = map_info.left_boundary_info()[0].type.value.value;
      left_boundary_length = map_info.left_boundary_info()[0].length;
    }
    if (map_info.right_boundary_info().size() > 0) {
      right_boundary_type = map_info.right_boundary_info()[0].type.value.value;
      right_boundary_length = map_info.right_boundary_info()[0].length;
    }
  }

  LaneTracksManager lane_tracks_mgr(
      world_model_->get_mutable_map_info_manager(), lateral_obstacle,
      *virtual_lane_mgr_);
  lane_tracks_mgr.update();
  std::vector<TrackedObject> *front_tracks_l = nullptr;
  std::vector<TrackedObject> *front_tracks_r = nullptr;
  std::vector<TrackedObject> *front_tracks_c = nullptr;
  TrackedObject *c_lead_one = nullptr;
  TrackedObject *c_lead_two = nullptr;
  TrackedObject *c_lead_three = nullptr;
  TrackedObject *l_lead_one = nullptr;
  TrackedObject *l_lead_two = nullptr;
  TrackedObject *l_lead_three = nullptr;
  TrackedObject *r_lead_one = nullptr;
  TrackedObject *r_lead_two = nullptr;
  TrackedObject *r_lead_three = nullptr;
  front_tracks_l = lane_tracks_mgr.get_lane_tracks(LaneProperty::LEFT_LANE,
                                                   TrackType::FRONT_TRACK);
  front_tracks_r = lane_tracks_mgr.get_lane_tracks(LaneProperty::RIGHT_LANE,
                                                   TrackType::FRONT_TRACK);
  front_tracks_c = lane_tracks_mgr.get_lane_tracks(LaneProperty::CURRENT_LANE,
                                                   TrackType::FRONT_TRACK);

  for (auto &tr : *front_tracks_r) {
    if (r_lead_one == nullptr || tr.d_rel < r_lead_one->d_rel) {
      r_lead_one = &tr;
    } else if (r_lead_two == nullptr ||
               (tr.d_rel < r_lead_two->d_rel && tr.d_rel > r_lead_one->d_rel)) {
      r_lead_two = &tr;
    } else if (tr.d_rel > r_lead_two->d_rel && tr.type < 10000) {
      r_lead_three = &tr;
      break;
    }
  }
  for (auto &tr : *front_tracks_l) {
    if (l_lead_one == nullptr || tr.d_rel < l_lead_one->d_rel) {
      l_lead_one = &tr;
    } else if (l_lead_two == nullptr ||
               (tr.d_rel < l_lead_two->d_rel && tr.d_rel > l_lead_one->d_rel)) {
      l_lead_two = &tr;
    } else if (tr.d_rel > l_lead_two->d_rel && tr.type < 10000) {
      l_lead_three = &tr;
      break;
    }
  }

  for (auto &tr : *front_tracks_c) {
    if (c_lead_one == nullptr || tr.d_rel < c_lead_one->d_rel) {
      c_lead_one = &tr;
    } else if (c_lead_two == nullptr ||
               (tr.d_rel < c_lead_two->d_rel && tr.d_rel > c_lead_one->d_rel)) {
      if ((r_lead_one == nullptr && l_lead_one == nullptr) ||
          (l_lead_one == nullptr && r_lead_one != nullptr &&
           tr.track_id != r_lead_one->track_id) ||
          (r_lead_one == nullptr && l_lead_one != nullptr &&
           tr.track_id != l_lead_one->track_id) ||
          (r_lead_one != nullptr && l_lead_one != nullptr &&
           tr.track_id != r_lead_one->track_id &&
           tr.track_id != l_lead_one->track_id)) {
        c_lead_two = &tr;
      }
    } else if (tr.d_rel > c_lead_two->d_rel && tr.type < 10000) {
      c_lead_three = &tr;
      break;
    }
  }

  AvdMsg temp{-1000,
              "none",
              false,
              "none",
              default_avd_priority,
              blocked_time_begin,
              blocked_time_end};
  avd_info_.insert(std::make_pair(-1000, temp));

  std::vector<const TrackedObject *> front_side_tracks;

  for (auto &tr : lateral_obstacle.front_tracks_copy()) {
    front_side_tracks.push_back(&tr);
  }

  for (auto &tr : lateral_obstacle.side_tracks()) {
    front_side_tracks.push_back(&tr);
  }

  if (avd_car_past_[0].size() > 0 && int(avd_car_past_[0][1]) == 10000) {
    avd_car_past_[0] = avd_car_past_[1];
    avd_car_past_[1].clear();
  } else if (avd_car_past_[1].size() > 0 && int(avd_car_past_[1][1]) == 10000) {
    avd_car_past_[1].clear();
  }
  if (avd_car_past_[0].size() > 0) {
    for (auto tr : front_side_tracks) {
      if (tr->track_id == int(avd_car_past_[0][0])) {
        avd_car_past0_lost = false;
      } else if (int(avd_car_past_[0][0]) == 0 &&
                 avd_car_past_[0].size() == 11 &&
                 tr->track_id == int(avd_car_past_[0][10])) {
        avd_car_past0_lost = false;
      }
      if (avd_car_past_[1].size() > 0) {
        if (tr->track_id == int(avd_car_past_[1][0])) {
          avd_car_past1_lost = false;
        } else if (int(avd_car_past_[1][0]) == 0 &&
                   avd_car_past_[1].size() == 11 &&
                   tr->track_id == int(avd_car_past_[1][10])) {
          avd_car_past1_lost = false;
        }
      } else if (!avd_car_past0_lost) {
        break;
      }
      if (!avd_car_past0_lost && !avd_car_past1_lost) {
        break;
      }
    }
  }
  if (avd_car_past0_lost) {
    if (avd_car_past1_lost) {
      avd_car_past_[0].clear();
      avd_car_past_[1].clear();
    } else {
      avd_car_past_[0] = avd_car_past_[1];
      avd_car_past_[1].clear();
    }
  } else if (avd_car_past1_lost) {
    avd_car_past_[1].clear();
  }

  update_avd_info(avd_car_past_[0], front_side_tracks, ego_l, 1);
  update_avd_info(avd_car_past_[1], front_side_tracks, ego_l, 2);

  double left_lane_width = 3.8;
  double right_lane_width = 3.8;

  if (!map_info.is_in_intersection() &&
      map_info.left_refline_points().size() > 0) {
    for (auto &p : map_info.left_refline_points()) {
      if (p.car_point.x >= 0 && p.car_point.x <= 20 && !p.in_intersection) {
        left_lane_width = p.lane_width;
        if (left_lane_width < 100) {
          break;
        }
      }
    }
  } else if (!map_info.is_in_intersection()) {
    left_lane_width = 0.;
  }

  if (!map_info.is_in_intersection() &&
      map_info.right_refline_points().size() > 0) {
    for (auto &p : map_info.right_refline_points()) {
      if (p.car_point.x >= 0 && p.car_point.x <= 20 && !p.in_intersection) {
        right_lane_width = p.lane_width;
        if (right_lane_width < 100) {
          break;
        }
      }
    }
  } else if (!map_info.is_in_intersection()) {
    right_lane_width = 0.;
  }

  if (right_lane_width > 100.) {
    right_lane_width = 0.;
  }

  for (auto tr : front_side_tracks) {
    if (avd_info_.find(tr->track_id) == avd_info_.end() &&
        tr->d_rel > -20 + std::min(-2 * tr->v_rel, 0.0) && tr->d_rel < 60) {
      if ((tr->type < 10000 && (tr->trajectory.intersection == 0 ||
                                (tr->trajectory.intersection < 10000 &&
                                 !map_info.is_in_intersection()))) ||
          (tr->type > 10000 && tr->v_lat > -0.5)) {
        if (lat_offset - tr->d_max_cpath <=
                std::max(1.5 * lane_width, 1.5 * lane_width - 2 * tr->v_lat) &&
            ((tr->d_max_cpath < 0 && tr->d_rel >= -5) ||
             (l_ego - tr->d_max_cpath >= 1.1 && tr->d_rel < -5) ||
             (tr->d_max_cpath >= 0. && tr->d_rel >= -5 && l_ego > -0.6 &&
              lane_width / 2. - car_width - 0.4 > tr->d_max_cpath &&
              tr->v_lead < 3. &&
              (map_info.dist_to_intsect() - tr->d_rel > 50. ||
               l_ego - tr->d_max_cpath >= 0.7)) ||
             (tr->type == 20001 && tr->d_max_cpath < 0.3) ||
             (l_ego > 0.6 &&
              (tr->d_max_cpath < 0.7 * lane_width ||
               map_info.is_in_intersection()) &&
              l_ego - 0.5 > tr->d_max_cpath &&
              (tr->d_rel < 1 ||
               ((RCHANGE || LCHANGE) &&
                tr->d_rel < std::max(std::max(-tr->v_rel, 0.) * 2.5, 3.))) &&
              tr->d_rel > (std::min(-7.0 - tr->v_rel * 2, -5.0)) &&
              (map_info.current_lane_index() > 0 ||
               l_ego > 0.5 * lane_width)))) {
          if (lat_offset - tr->d_max_cpath >= 1.3 ||
              (tr->d_rel < 1 &&
               tr->d_rel > (std::min(-7.0 - tr->v_rel * 2, -5.0)) &&
               l_ego - 0.5 > tr->d_max_cpath)) {
            avd_info_[tr->track_id] = {tr->track_id,
                                       "NBO",
                                       true,
                                       "left",
                                       default_avd_priority,
                                       blocked_time_begin,
                                       blocked_time_end};
          } else if (((tr->d_max_cpath < 0 || tr->type == 20001) &&
                      tr->d_rel >= (std::min(-7.0 - tr->v_rel * 2, -5.0))) ||
                     (tr->d_max_cpath < -1.1 &&
                      tr->d_rel < (std::min(-7.0 - tr->v_rel * 2, -5.0)) &&
                      tr->v_rel > 1.0)) {
            avd_info_[tr->track_id] = {tr->track_id,
                                       "NBO",
                                       false,
                                       "left",
                                       default_avd_priority,
                                       blocked_time_begin,
                                       blocked_time_end};
          }
        } else if (-lat_offset + tr->d_min_cpath <=
                       std::max(1.5 * lane_width,
                                1.5 * lane_width - 2 * tr->v_lat) &&
                   ((tr->d_min_cpath > 0 && tr->d_rel >= -5) ||
                    (tr->d_min_cpath - l_ego >= 1.1 && tr->d_rel < -5) ||
                    (tr->type == 20001 && tr->d_min_cpath > -0.3) ||
                    (l_ego < -0.6 &&
                     (tr->d_min_cpath > -0.7 * lane_width ||
                      map_info.is_in_intersection()) &&
                     l_ego + 0.5 < tr->d_min_cpath &&
                     (tr->d_rel < 1 ||
                      ((RCHANGE || LCHANGE) &&
                       tr->d_rel <
                           std::max(std::max(-tr->v_rel, 0.) * 2.5, 3.))) &&
                     tr->d_rel > (std::min(-7.0 - tr->v_rel * 2, -5.0)) &&
                     (map_info.current_lane_index() <
                          map_info.lanes_num() - 1 ||
                      l_ego < -0.5 * lane_width)))) {
          if (-lat_offset + tr->d_min_cpath >= 1.3 ||
              (tr->d_rel < 1 &&
               tr->d_rel > (std::min(-7.0 - tr->v_rel * 2, -5.0)) &&
               l_ego + 0.5 < tr->d_min_cpath)) {
            avd_info_[tr->track_id] = {tr->track_id,
                                       "NBO",
                                       true,
                                       "right",
                                       default_avd_priority,
                                       blocked_time_begin,
                                       blocked_time_end};
          } else if (((tr->d_min_cpath > 0 || tr->type == 20001) &&
                      tr->d_rel >= (std::min(-7.0 - tr->v_rel * 2, -5.0))) ||
                     (tr->d_min_cpath > 1.1 &&
                      tr->d_rel < (std::min(-7.0 - tr->v_rel * 2, -5.0)) &&
                      tr->v_rel > 1.0)) {
            avd_info_[tr->track_id] = {tr->track_id,
                                       "NBO",
                                       false,
                                       "right",
                                       default_avd_priority,
                                       blocked_time_begin,
                                       blocked_time_end};
          }
        }
      } else if ((tr->type > 10000 && tr->v_lat <= -0.5) ||
                 (tr->type < 10000 &&
                  (tr->trajectory.intersection != 0 || tr->v_lat <= -0.5))) {
        if (((tr->d_max_cpath < 0 && tr->d_rel >= 0) ||
             (tr->d_max_cpath < -1.1 && tr->d_rel < 0 && tr->v_rel > 1.0) ||
             (tr->d_rel >= (std::min(-7.0 - tr->v_rel * 2, -5.0)) &&
              tr->d_rel < 1 && tr->d_max_cpath < 0.7 * lane_width &&
              l_ego - 0.5 > tr->d_max_cpath)) &&
            tr->trajectory.intersection != 10000) {
          avd_info_[tr->track_id] = {tr->track_id,
                                     "NBO",
                                     false,
                                     "left",
                                     default_avd_priority,
                                     blocked_time_begin,
                                     blocked_time_end};
        } else if (((tr->d_min_cpath > 0 && tr->d_rel >= 0) ||
                    (tr->d_min_cpath > 1.1 && tr->d_rel < 0 &&
                     tr->v_rel > 1.0) ||
                    (tr->d_rel >= (std::min(-7.0 - tr->v_rel * 2, -5.0)) &&
                     tr->d_rel < 1 && tr->d_min_cpath > -0.7 * lane_width &&
                     l_ego + 0.5 < tr->d_min_cpath)) &&
                   tr->trajectory.intersection != 10000) {
          avd_info_[tr->track_id] = {tr->track_id,
                                     "NBO",
                                     false,
                                     "right",
                                     default_avd_priority,
                                     blocked_time_begin,
                                     blocked_time_end};
        }
      }
      if (tr->trajectory.intersection == 10000) {
        if (tr->d_rel > -3.5 && tr->d_rel < 1 &&
            ((l_ego - tr->d_max_cpath < 2. && l_ego > tr->d_max_cpath) ||
             (tr->d_min_cpath - l_ego < 2. && l_ego < tr->d_min_cpath))) {
          if (avd_info_.find(tr->track_id) != avd_info_.end()) {

          } else if (l_ego > tr->d_max_cpath) {
            avd_info_[tr->track_id] = {tr->track_id,
                                       "NBO",
                                       false,
                                       "left",
                                       default_avd_priority,
                                       blocked_time_begin,
                                       blocked_time_end};
          } else if (l_ego < tr->d_min_cpath) {
            avd_info_[tr->track_id] = {tr->track_id,
                                       "NBO",
                                       false,
                                       "right",
                                       default_avd_priority,
                                       blocked_time_begin,
                                       blocked_time_end};
          }
        } else {
          avd_info_.erase(tr->track_id);
        }
      }
    } else if (avd_info_.find(tr->track_id) != avd_info_.end() &&
               (tr->d_rel < 1 ||
                ((RCHANGE || LCHANGE) &&
                 tr->d_rel < std::max(std::max(-tr->v_rel, 0.) * 2.5, 3.))) &&
               tr->d_rel > (std::min(-7.0 - tr->v_rel * 2, -5.0))) {
      if (l_ego > 0.6 && l_ego - 0.6 > tr->d_max_cpath &&
          avd_info_[tr->track_id].avd_direction == "right") {
        // avd_info_[tr->track_id].avd_direction = "left";
      } else if (l_ego < -0.6 && l_ego + 0.6 < tr->d_min_cpath &&
                 avd_info_[tr->track_id].avd_direction == "left") {
        // avd_info_[tr->track_id].avd_direction = "right";
      }
    }
    if (tr->v_lead < -1. && tr->d_rel > -5 && tr->d_min_cpath < 0.2 &&
        tr->d_max_cpath > 0) {
      double lend = tr->d_min_cpath;
      double pi = std::atan(1.0) * 4;
      if (tr->trajectory.intersection == 10000) {
        auto &x = tr->trajectory.x;
        auto &y = tr->trajectory.y;
        static std::vector<double> ego_x;
        ego_x.clear();
        static std::vector<double> ego_y;
        ego_y.clear();
        static std::vector<double> ego_speed;
        ego_speed.clear();
        static std::vector<double> ego_yaw;
        ego_yaw.clear();
        double ego_fx = std::cos(ego_state.ego_pose_raw.theta);
        double ego_fy = std::sin(ego_state.ego_pose_raw.theta);
        double ego_lx = -ego_fy;
        double ego_ly = ego_fx;
        double theta = 0.0;
        double send = 0.0;
        int end_idx = 0;
        lend = 0.0;
        for (int i = 0; i < x.size(); i++) {
          double dx = x[i] - ego_state.ego_pose_raw.x;
          double dy = y[i] - ego_state.ego_pose_raw.y;

          double rel_x = dx * ego_fx + dy * ego_fy;
          double rel_y = dx * ego_lx + dy * ego_ly;
          ego_x.push_back(rel_x);
          ego_y.push_back(rel_y);
        }
        int end_range = min((int)ego_x.size() - 1, 25);
        for (int i = end_range; i >= 0; i--) {
          if (ego_x[i] < 0 || ego_x[i] > 80) {
            continue;
          }

          end_idx = i;
          break;
        }
        f_refline.cartesian_frenet(ego_x[end_idx], ego_y[end_idx], send, lend,
                                   theta, false);
        lend = lend - tr->width - 0.5;
      }
    }
    if (avd_sp_car_past_[0].size() > 0 &&
        tr->track_id == int(avd_sp_car_past_[0][0])) {
      obj_disappear = false;
    }
    if (avd_sp_car_past_[0].size() > 0 &&
        tr->track_id == int(avd_sp_car_past_[0][0]) &&
        tr->trajectory.intersection == 10000 &&
        !(tr->d_rel > -3.5 && tr->d_rel < 1 &&
          ((l_ego - tr->d_max_cpath < 1.5 && l_ego > tr->d_max_cpath) ||
           (tr->d_min_cpath - l_ego < 1.5 && l_ego < tr->d_min_cpath)))) {
      avd_sp_car_past_[0].clear();
      avd_sp_car_past_[1].clear();
    }
  }

  if (avd_sp_car_past_[0].size() > 0) {
    int track_id = (int)avd_sp_car_past_[0][0];
    int type = 0;
    double drel = 0.0;
    double vlead = 0.0;
    for (auto tr : front_side_tracks) {
      if (tr->track_id == track_id) {
        type = tr->type;
        drel = tr->d_rel;
        vlead = tr->v_lead;
        avd_sp_car_past_[0][3] = drel;
        break;
      }
    }

    if ((int)avd_sp_car_past_[0][9] == LEFT_CHANGE) {

      avd_info_[track_id] = {track_id,
                             "LBO",
                             false,
                             "left",
                             default_avd_priority,
                             blocked_time_begin,
                             blocked_time_end};
      if (lead_one != nullptr && track_id != lead_one->track_id &&
          (avd_info_.find(lead_one->track_id) == avd_info_.end() ||
           lead_one->type == 20001) &&
          lead_one->trajectory.intersection != 10000 &&
          lead_one->d_max_cpath < lead_one->width / 4.0 + lane_width / 2.0 &&
          lead_one->v_lead < 1 &&
          (((left_boundary_type == LaneBoundaryForm::DASH &&
             (left_boundary_length - lead_one->d_rel > 10 ||
              ((type == 20001 || lead_one->type == 20001) &&
               lead_one->d_rel - avd_sp_car_past_[0][3] < 10)) &&
             !isRedLightStop) ||
            (left_boundary_type == LaneBoundaryForm::SOLID &&
             (type == 20001 || lead_one->type == 20001) &&
             lead_one->d_rel - avd_sp_car_past_[0][3] < 10)) ||
           map_info.is_in_intersection())) {
        avd_info_[lead_one->track_id] = {lead_one->track_id,
                                         "LBO",
                                         false,
                                         "left",
                                         default_avd_priority,
                                         blocked_time_begin,
                                         blocked_time_end};
      } else if (lead_one != nullptr && lead_two != nullptr &&
                 track_id == lead_one->track_id &&
                 (avd_info_.find(lead_two->track_id) == avd_info_.end() ||
                  lead_two->type == 20001) &&
                 lead_two->trajectory.intersection != 10000 &&
                 lead_two->d_max_cpath <
                     lead_two->width / 4.0 + lane_width / 2.0 &&
                 lead_two->v_lead < 1 &&
                 (((left_boundary_type == LaneBoundaryForm::DASH &&
                    (left_boundary_length - lead_two->d_rel > 10 ||
                     ((type == 20001 || lead_two->type == 20001) &&
                      lead_two->d_rel - lead_one->d_rel < 10)) &&
                    !isRedLightStop) ||
                   (left_boundary_type == LaneBoundaryForm::SOLID &&
                    (lead_one->type == 20001 || lead_two->type == 20001) &&
                    lead_two->d_rel - lead_one->d_rel < 10)) ||
                  map_info.is_in_intersection())) {
        avd_info_[lead_two->track_id] = {lead_two->track_id,
                                         "LBO",
                                         false,
                                         "left",
                                         default_avd_priority,
                                         blocked_time_begin,
                                         blocked_time_end};
      } else if (temp_leadone != nullptr &&
                 track_id != temp_leadone->track_id &&
                 (avd_info_.find(temp_leadone->track_id) == avd_info_.end() ||
                  temp_leadone->type == 20001) &&
                 temp_leadone->trajectory.intersection != 10000 &&
                 temp_leadone->d_max_cpath <
                     temp_leadone->width / 4.0 + lane_width / 2.0 &&
                 temp_leadone->v_lead < 1 &&
                 (((left_boundary_type == LaneBoundaryForm::DASH &&
                    (left_boundary_length - temp_leadone->d_rel > 10 ||
                     ((type == 20001 || temp_leadone->type == 20001) &&
                      temp_leadone->d_rel - avd_sp_car_past_[0][3] < 10)) &&
                    !isRedLightStop) ||
                   (left_boundary_type == LaneBoundaryForm::SOLID &&
                    (type == 20001 || temp_leadone->type == 20001) &&
                    temp_leadone->d_rel - avd_sp_car_past_[0][3] < 10)) ||
                  map_info.is_in_intersection())) {
        avd_info_[temp_leadone->track_id] = {temp_leadone->track_id,
                                             "LBO",
                                             false,
                                             "left",
                                             default_avd_priority,
                                             blocked_time_begin,
                                             blocked_time_end};
      } else if (temp_leadone != nullptr && temp_leadtwo != nullptr &&
                 track_id == temp_leadone->track_id &&
                 (avd_info_.find(temp_leadtwo->track_id) == avd_info_.end() ||
                  temp_leadtwo->type == 20001) &&
                 temp_leadtwo->trajectory.intersection != 10000 &&
                 temp_leadtwo->d_max_cpath <
                     temp_leadtwo->width / 4.0 + lane_width / 2.0 &&
                 temp_leadtwo->v_lead < 1 &&
                 (((left_boundary_type == LaneBoundaryForm::DASH &&
                    (left_boundary_length - temp_leadtwo->d_rel > 10 ||
                     ((type == 20001 || temp_leadtwo->type == 20001) &&
                      temp_leadtwo->d_rel - temp_leadone->d_rel < 10)) &&
                    !isRedLightStop) ||
                   (left_boundary_type == LaneBoundaryForm::SOLID &&
                    (temp_leadone->type == 20001 ||
                     temp_leadtwo->type == 20001) &&
                    temp_leadtwo->d_rel - temp_leadone->d_rel < 10)) ||
                  map_info.is_in_intersection())) {
        avd_info_[temp_leadtwo->track_id] = {temp_leadtwo->track_id,
                                             "LBO",
                                             false,
                                             "left",
                                             default_avd_priority,
                                             blocked_time_begin,
                                             blocked_time_end};
      }

      for (auto &tr : lateral_obstacle.front_tracks_copy()) {
        if (tr.d_min_cpath > lat_offset + 1.0 &&
            tr.d_min_cpath < lat_offset + 2.0 && tr.d_rel < 60 &&
            avd_info_.find(tr.track_id) == avd_info_.end() &&
            tr.trajectory.intersection != 10000) {
          avd_info_[tr.track_id] = {tr.track_id,
                                    "NBO",
                                    true,
                                    "right",
                                    default_avd_priority,
                                    blocked_time_begin,
                                    blocked_time_end};
        }
        if (tr.d_max_cpath > lane_width / 2 - (car_width + 0.3) &&
            tr.d_max_cpath < lane_width / 2 && tr.d_rel < 60 &&
            (avd_info_.find(tr.track_id) == avd_info_.end() ||
             tr.type == 20001) &&
            tr.v_lead < 1 && tr.trajectory.intersection != 10000 &&
            ((virtual_lane_mgr_->has_origin_lane() &&
              !virtual_lane_mgr_->is_on_lane(ORIGIN_LANE) &&
              map_info.right_boundary_info().size() > 0 &&
              ((map_info.right_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::DASH &&
                (map_info.right_boundary_info()[0].length - tr.d_rel > 10 ||
                 ((type == 20001 || tr.type == 20001) &&
                  tr.d_rel - avd_sp_car_past_[0][3] < 10))) ||
               (map_info.right_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::SOLID &&
                (type == 20001 || tr.type == 20001) &&
                tr.d_rel - avd_sp_car_past_[0][3] < 10))) ||
             (virtual_lane_mgr_->has_origin_lane() &&
              virtual_lane_mgr_->is_on_lane(ORIGIN_LANE) &&
              map_info.left_boundary_info().size() > 0 &&
              ((map_info.left_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::DASH &&
                (map_info.left_boundary_info()[0].length - tr.d_rel > 10 ||
                 ((type == 20001 || tr.type == 20001) &&
                  tr.d_rel - avd_sp_car_past_[0][3] < 10))) ||
               (map_info.left_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::SOLID &&
                (type == 20001 || tr.type == 20001) &&
                tr.d_rel - avd_sp_car_past_[0][3] < 10)))) &&
            (!isRedLightStop ||
             (traffic_light_direction == Direction::TURN_RIGHT &&
              straight_light_status_at_right != 4))) {
          avd_info_[tr.track_id] = {tr.track_id,
                                    "LBO",
                                    false,
                                    "left",
                                    default_avd_priority,
                                    blocked_time_begin,
                                    blocked_time_end};
        }
      }

      for (auto &tr : lateral_obstacle.side_tracks_r()) {
        if (tr.d_max_cpath > -0.9 && tr.d_max_cpath < lane_width / 2 &&
            tr.d_rel > -5 &&
            (avd_info_.find(tr.track_id) == avd_info_.end() ||
             tr.type == 20001) &&
            ((virtual_lane_mgr_->has_origin_lane() &&
              !virtual_lane_mgr_->is_on_lane(ORIGIN_LANE) &&
              map_info.right_boundary_info().size() > 0 &&
              ((map_info.right_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::DASH &&
                (map_info.right_boundary_info()[0].length - tr.d_rel > 10 ||
                 ((type == 20001 || tr.type == 20001) &&
                  tr.d_rel - avd_sp_car_past_[0][3] < 10))) ||
               (map_info.right_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::SOLID &&
                (type == 20001 || tr.type == 20001) &&
                tr.d_rel - avd_sp_car_past_[0][3] < 10))) ||
             (virtual_lane_mgr_->has_origin_lane() &&
              virtual_lane_mgr_->is_on_lane(ORIGIN_LANE) &&
              map_info.left_boundary_info().size() > 0 &&
              ((map_info.left_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::DASH &&
                (map_info.left_boundary_info()[0].length - tr.d_rel > 10 ||
                 ((type == 20001 || tr.type == 20001) &&
                  tr.d_rel - avd_sp_car_past_[0][3] < 10))) ||
               (map_info.left_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::SOLID &&
                (type == 20001 || tr.type == 20001) &&
                tr.d_rel - avd_sp_car_past_[0][3] < 10))))) {
          avd_info_[tr.track_id] = {tr.track_id,
                                    "LBO",
                                    false,
                                    "left",
                                    default_avd_priority,
                                    blocked_time_begin,
                                    blocked_time_end};
        }
      }

      for (auto &tr : lateral_obstacle.side_tracks_l()) {
        if (tr.d_min_cpath > lat_offset + 1.0 &&
            tr.d_min_cpath < lat_offset + 2.0 && tr.d_rel + tr.v_rel > -8.0 &&
            tr.d_rel + tr.v_rel < 2 &&
            avd_info_.find(tr.track_id) == avd_info_.end()) {
          avd_info_[tr.track_id] = {tr.track_id,
                                    "NBO",
                                    true,
                                    "right",
                                    default_avd_priority,
                                    blocked_time_begin,
                                    blocked_time_end};
        }
      }
      if (map_info.merge_split_point_data().size() > 0 &&
          (map_info.merge_split_point_data()[0].distance < 5. &&
           map_info.merge_split_point_data()[0].distance +
                   std::max(map_info.merge_split_point_data()[0].length, 25.0) >
               drel &&
           vlead < 1.)) {
        avd_info_[track_id].avd_priority = 1000;
      }
    } else if ((int)avd_sp_car_past_[0][9] == RIGHT_CHANGE) {
      avd_info_[track_id] = {track_id,
                             "LBO",
                             false,
                             "right",
                             default_avd_priority,
                             blocked_time_begin,
                             blocked_time_end};
      if (lead_one != nullptr && track_id != lead_one->track_id &&
          (avd_info_.find(lead_one->track_id) == avd_info_.end() ||
           lead_one->type == 20001) &&
          lead_one->d_min_cpath > -(lead_one->width / 4.0 + lane_width / 2.0) &&
          lead_one->trajectory.intersection != 10000 && lead_one->v_lead < 1 &&
          (((right_boundary_type == LaneBoundaryForm::DASH &&
             (right_boundary_length - lead_one->d_rel > 10 ||
              ((type == 20001 || lead_one->type == 20001) &&
               lead_one->d_rel - avd_sp_car_past_[0][3] < 10)) &&
             !isRedLightStop) ||
            (right_boundary_type == LaneBoundaryForm::SOLID &&
             (type == 20001 || lead_one->type == 20001) &&
             lead_one->d_rel - avd_sp_car_past_[0][3] < 10)) ||
           map_info.is_in_intersection())) {
        avd_info_[lead_one->track_id] = {lead_one->track_id,
                                         "LBO",
                                         false,
                                         "right",
                                         default_avd_priority,
                                         blocked_time_begin,
                                         blocked_time_end};
      } else if (lead_one != nullptr && lead_two != nullptr &&
                 track_id == lead_one->track_id &&
                 (avd_info_.find(lead_two->track_id) == avd_info_.end() ||
                  lead_two->type == 20001) &&
                 lead_two->trajectory.intersection != 10000 &&
                 lead_two->d_min_cpath >
                     -(lead_two->width / 4.0 + lane_width / 2.0) &&
                 lead_two->v_lead < 1 &&
                 (((right_boundary_type == LaneBoundaryForm::DASH &&
                    (right_boundary_length - lead_two->d_rel > 10 ||
                     ((type == 20001 || lead_two->type == 20001) &&
                      lead_two->d_rel - lead_one->d_rel < 10)) &&
                    !isRedLightStop) ||
                   (right_boundary_type == LaneBoundaryForm::SOLID &&
                    (lead_one->type == 20001 || lead_two->type == 20001) &&
                    lead_two->d_rel - lead_one->d_rel < 10)) ||
                  map_info.is_in_intersection())) {
        avd_info_[lead_two->track_id] = {lead_two->track_id,
                                         "LBO",
                                         false,
                                         "right",
                                         default_avd_priority,
                                         blocked_time_begin,
                                         blocked_time_end};
      } else if (temp_leadone != nullptr &&
                 track_id != temp_leadone->track_id &&
                 (avd_info_.find(temp_leadone->track_id) == avd_info_.end() ||
                  temp_leadone->type == 20001) &&
                 temp_leadone->trajectory.intersection != 10000 &&
                 temp_leadone->d_min_cpath >
                     -(temp_leadone->width / 4.0 + lane_width / 2.0) &&
                 temp_leadone->v_lead < 1 &&
                 (((right_boundary_type == LaneBoundaryForm::DASH &&
                    (right_boundary_length - temp_leadone->d_rel > 10 ||
                     ((type == 20001 || temp_leadone->type == 20001) &&
                      temp_leadone->d_rel - avd_sp_car_past_[0][3] < 10)) &&
                    !isRedLightStop) ||
                   (right_boundary_type == LaneBoundaryForm::SOLID &&
                    (type == 20001 || temp_leadone->type == 20001) &&
                    temp_leadone->d_rel - avd_sp_car_past_[0][3] < 10)) ||
                  map_info.is_in_intersection())) {
        avd_info_[temp_leadone->track_id] = {temp_leadone->track_id,
                                             "LBO",
                                             false,
                                             "right",
                                             default_avd_priority,
                                             blocked_time_begin,
                                             blocked_time_end};
      } else if (temp_leadone != nullptr && temp_leadtwo != nullptr &&
                 track_id == temp_leadone->track_id &&
                 (avd_info_.find(temp_leadtwo->track_id) == avd_info_.end() ||
                  temp_leadtwo->type == 20001) &&
                 temp_leadtwo->trajectory.intersection != 10000 &&
                 temp_leadtwo->d_min_cpath >
                     -(temp_leadtwo->width / 4.0 + lane_width / 2.0) &&
                 temp_leadtwo->v_lead < 1 &&
                 (((right_boundary_type == LaneBoundaryForm::DASH &&
                    (right_boundary_length - temp_leadtwo->d_rel > 10 ||
                     ((type == 20001 || temp_leadtwo->type == 20001) &&
                      temp_leadtwo->d_rel - temp_leadone->d_rel < 10)) &&
                    !isRedLightStop) ||
                   (right_boundary_type == LaneBoundaryForm::SOLID &&
                    (temp_leadone->type == 20001 ||
                     temp_leadtwo->type == 20001) &&
                    temp_leadtwo->d_rel - temp_leadone->d_rel < 10)) ||
                  map_info.is_in_intersection())) {
        avd_info_[temp_leadtwo->track_id] = {temp_leadtwo->track_id,
                                             "LBO",
                                             false,
                                             "right",
                                             default_avd_priority,
                                             blocked_time_begin,
                                             blocked_time_end};
      }

      for (auto &tr : lateral_obstacle.front_tracks_copy()) {
        if (tr.d_max_cpath < lat_offset - 1.0 &&
            tr.d_max_cpath > lat_offset - 2.0 && tr.d_rel < 60 &&
            avd_info_.find(tr.track_id) == avd_info_.end() &&
            tr.trajectory.intersection != 10000) {
          avd_info_[tr.track_id] = {tr.track_id,
                                    "NBO",
                                    true,
                                    "left",
                                    default_avd_priority,
                                    blocked_time_begin,
                                    blocked_time_end};
        }
        if (tr.d_min_cpath < car_width + 0.3 - lane_width / 2 &&
            (tr.d_min_cpath > -lane_width / 2 - 0.3) && tr.d_rel < 60 &&
            (avd_info_.find(tr.track_id) == avd_info_.end() ||
             (avd_info_[tr.track_id].avd_direction == "left" &&
              tr.d_rel - drel < 12.)) &&
            tr.trajectory.intersection != 10000 && tr.v_lead < 1 &&
            ((virtual_lane_mgr_->has_origin_lane() &&
              !virtual_lane_mgr_->is_on_lane(ORIGIN_LANE) &&
              map_info.left_boundary_info().size() > 0 &&
              ((map_info.left_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::DASH &&
                ((map_info.left_boundary_info()[0].length - tr.d_rel > 10) ||
                 ((type == 20001 || tr.type == 20001) &&
                  tr.d_rel - drel < 10))) ||
               (map_info.left_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::SOLID &&
                (type == 20001 || tr.type == 20001) &&
                tr.d_rel - drel < 10))) ||
             (virtual_lane_mgr_->has_origin_lane() &&
              virtual_lane_mgr_->is_on_lane(ORIGIN_LANE) &&
              map_info.right_boundary_info().size() > 0 &&
              ((map_info.right_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::DASH &&
                ((map_info.right_boundary_info()[0].length - tr.d_rel > 10) ||
                 ((type == 20001 || tr.type == 20001) &&
                  tr.d_rel - drel < 10))) ||
               (map_info.right_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::SOLID &&
                (type == 20001 || tr.type == 20001) &&
                tr.d_rel - drel < 10))) ||
             (map_info.current_lane_index() == map_info.lanes_num() - 1 &&
              (map_info.dist_to_intsect() > 50. ||
               ((type == 20001 || tr.type == 20001) &&
                tr.d_rel - drel < 10)))) &&
            !isRedLightStop) {
          avd_info_[tr.track_id] = {tr.track_id,
                                    "LBO",
                                    false,
                                    "right",
                                    default_avd_priority,
                                    blocked_time_begin,
                                    blocked_time_end};
        }
      }

      for (auto &tr : lateral_obstacle.side_tracks_l()) {
        if (tr.d_min_cpath < 0.9 && tr.d_min_cpath > -lane_width / 2 &&
            ((l_ego <= -0.4 &&
                  ((tr.v_lat > -0.5 &&
                    tr.d_rel > (std::min(-7.0 - std::max(tr.v_rel, 0.) *
                                                    (tr.d_min_cpath - l_ego) /
                                                    car_width * 4,
                                         -5.0)))) ||
              (tr.v_lat <= -0.5 && tr.d_rel >= -8.0)) ||
             (l_ego > -0.4 && ((v_ego >= 1 && tr.d_rel >= -7.0) ||
                               (v_ego < 1 && tr.d_rel >= -5.0)))) &&
            (avd_info_.find(tr.track_id) == avd_info_.end() ||
             (avd_info_[tr.track_id].avd_direction == "left" &&
              l_ego < -1.5)) &&
            ((virtual_lane_mgr_->has_origin_lane() &&
              !virtual_lane_mgr_->is_on_lane(ORIGIN_LANE) &&
              map_info.left_boundary_info().size() > 0 &&
              ((map_info.left_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::DASH &&
                (map_info.left_boundary_info()[0].length - tr.d_rel > 10 ||
                 ((type == 20001 || tr.type == 20001) &&
                  tr.d_rel - avd_sp_car_past_[0][3] < 10))) ||
               (map_info.left_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::SOLID &&
                (type == 20001 || tr.type == 20001) &&
                tr.d_rel - avd_sp_car_past_[0][3] < 10))) ||
             (virtual_lane_mgr_->has_origin_lane() &&
              virtual_lane_mgr_->is_on_lane(ORIGIN_LANE) &&
              map_info.right_boundary_info().size() > 0 &&
              ((map_info.right_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::DASH &&
                (map_info.right_boundary_info()[0].length - tr.d_rel > 10 ||
                 ((type == 20001 || tr.type == 20001) &&
                  tr.d_rel - avd_sp_car_past_[0][3] < 10))) ||
               (map_info.right_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::SOLID &&
                (type == 20001 || tr.type == 20001) &&
                tr.d_rel - avd_sp_car_past_[0][3] < 10))) ||
             (map_info.current_lane_index() == map_info.lanes_num() - 1))) {
          avd_info_[tr.track_id] = {tr.track_id,
                                    "LBO",
                                    false,
                                    "right",
                                    default_avd_priority,
                                    blocked_time_begin,
                                    blocked_time_end};
        }
      }

      for (auto &tr : lateral_obstacle.side_tracks_r()) {
        if (tr.d_max_cpath < lat_offset - 1.0 &&
            tr.d_max_cpath > lat_offset - 2.0 && tr.d_rel + tr.v_rel > -8 &&
            tr.d_rel + tr.v_rel < 2 &&
            avd_info_.find(tr.track_id) == avd_info_.end()) {
          avd_info_[tr.track_id] = {tr.track_id,
                                    "NBO",
                                    true,
                                    "left",
                                    default_avd_priority,
                                    blocked_time_begin,
                                    blocked_time_end};
        }
      }
      if (map_info.merge_split_point_data().size() > 0 &&
          (map_info.merge_split_point_data()[0].distance < 5. &&
           map_info.merge_split_point_data()[0].distance +
                   std::max(map_info.merge_split_point_data()[0].length, 25.0) >
               drel &&
           vlead < 1.)) {
        avd_info_[track_id].avd_priority = 1000;
      }
    }
  }

  if (avd_sp_car_past_[0].size() > 0 &&
      ((map_info.lanes_num() == 1 && 0.5 * lane_width + lat_offset > 0.5) ||
       (avd_sp_car_past_[0][4] > 0.6 &&
        (((int)avd_sp_car_past_[0][9] == RIGHT_CHANGE &&
          ((std::fabs(avd_sp_car_past_[0][5]) >=
            std::fabs(avd_sp_car_past_[0][6])) ||
           (avd_sp_car_past_[0][5] > 1.1))) ||
         ((int)avd_sp_car_past_[0][9] == LEFT_CHANGE &&
          ((std::fabs(avd_sp_car_past_[0][5]) <=
            std::fabs(avd_sp_car_past_[0][6])) ||
           (avd_sp_car_past_[0][6] < -1.1))))) ||
       (avd_sp_car_past_[0][3] > 20 && avd_sp_car_past_[0][4] < -0.5) ||
       (int(avd_sp_car_past_[0][9]) == RIGHT_CHANGE &&
        (1.5 * lane_width + avd_sp_car_past_[0][5] < car_width + 0.5 ||
         (avd_sp_car_past_[0][5] - l_ego < -0.6 &&
          map_info.dist_to_intsect() > 200. && avd_sp_car_past_[0][3] > 5.))) ||
       (int(avd_sp_car_past_[0][9]) == LEFT_CHANGE &&
        (1.5 * lane_width - avd_sp_car_past_[0][6] < car_width + 0.5 ||
         (avd_sp_car_past_[0][6] - l_ego > 0.6 &&
          map_info.dist_to_intsect() > 200. && avd_sp_car_past_[0][3] > 5.))) ||
       obj_disappear)) {
    avd_sp_car_past_[0].clear();
    avd_sp_car_past_[1].clear();
  }

  if ((state == ROAD_LC_LCHANGE) && ignore_track_id_ != -10000) {
    avd_info_[ignore_track_id_] = {ignore_track_id_, "LBO", true,
                                   "left",           0,     blocked_time_begin,
                                   blocked_time_end};
  }

  if ((state == ROAD_LC_RCHANGE) && ignore_track_id_ != -10000) {
    avd_info_[ignore_track_id_] = {ignore_track_id_, "LBO", true,
                                   "right",          0,     blocked_time_begin,
                                   blocked_time_end};
  }

  if (state_machine_output.lc_request_source == MAP_REQUEST) {
    if (state_machine_output.lc_request == LEFT_CHANGE &&
        temp_leadone != nullptr && state == ROAD_LC_LCHANGE &&
        avd_info_.find(temp_leadone->track_id) == avd_info_.end()) {
      avd_info_[temp_leadone->track_id] = {
          temp_leadone->track_id, "LBO",           false, "left", 0,
          blocked_time_begin,     blocked_time_end};
    } else if (state_machine_output.lc_request == RIGHT_CHANGE &&
               temp_leadone != nullptr && state == ROAD_LC_RCHANGE &&
               avd_info_.find(temp_leadone->track_id) == avd_info_.end()) {
      avd_info_[temp_leadone->track_id] = {
          temp_leadone->track_id, "LBO",           false, "right", 0,
          blocked_time_begin,     blocked_time_end};
    }
  }
}

void LateralBehaviorPlanner::update_avdobstacles_info() {
  avd_obstacles.clear();
  avd_obstacle_prior_.clear();
  for (auto it = avd_info_.begin(); it != avd_info_.end(); ++it) {
    avd_obstacles.push_back(make_pair(it->second.id, it->second.avd_direction));

    MSD_LOG(INFO,
            "[LateralBehaviorPlanner] it->second.id[%d] "
            "it->second.avd_direction[%s]",
            it->second.id, it->second.avd_direction.c_str());
  }
}

void LateralBehaviorPlanner::update_obstacle_time_interval() {
  MLOG_PROFILING("update_obstacle_time_interval");
  auto &lateral_obstacle = world_model_->mutable_lateral_obstacle();
  auto &state_machine_output = context_->state_machine_output();
  auto &map_info = world_model_->get_map_info();

  auto &f_refline = virtual_lane_mgr_->get_fix_refline();
  auto baseline_info = world_model_->get_baseline_info(f_refline.position());
  if (baseline_info == nullptr || !baseline_info->is_valid()) {
    MSD_LOG(ERROR, "[update_obstacle_time_interval] baseline_info invalid");
    return;
  }

  int state = state_machine_output.curr_state;
  std::string lc_status = "none";
  if (state == ROAD_LC_LCHANGE) {
    lc_status = "left_lane_change";
  } else if (state == ROAD_LC_RCHANGE) {
    lc_status = "right_lane_change";
  }

  double lane_borrow_length = 1.0e6;
  LaneBorrowInfo lane_borrow_info = LaneBorrowInfo::NO_BORROW;
  if (lc_status == "left_lane_change") {
    lane_borrow_info = LaneBorrowInfo::RIGHT_BORROW;

    if (map_info.left_boundary_info().size() >= 2 &&
        map_info.left_boundary_info()[0].type.value.value !=
            LaneBoundaryForm::SOLID) {
      double dash_length = 0;
      for (const auto &boundary : map_info.left_boundary_info()) {
        if (boundary.type.value.value == LaneBoundaryForm::DASH ||
            boundary.type.value.value == LaneBoundaryForm::VIRTUAL) {
          dash_length += boundary.length;
        } else if (boundary.type.value.value == LaneBoundaryForm::SOLID ||
                   boundary.type.value.value == LaneBoundaryForm::PHYSICAL) {
          lane_borrow_length = dash_length;
          break;
        }
      }
    }
  } else if (lc_status == "right_lane_change") {
    lane_borrow_info = LaneBorrowInfo::LEFT_BORROW;

    if (map_info.right_boundary_info().size() >= 2 &&
        map_info.right_boundary_info()[0].type.value.value !=
            LaneBoundaryForm::SOLID) {
      double dash_length = 0;
      for (const auto &boundary : map_info.right_boundary_info()) {
        if (boundary.type.value.value == LaneBoundaryForm::DASH ||
            boundary.type.value.value == LaneBoundaryForm::VIRTUAL) {
          dash_length += boundary.length;
        } else if (boundary.type.value.value == LaneBoundaryForm::SOLID ||
                   boundary.type.value.value == LaneBoundaryForm::PHYSICAL) {
          lane_borrow_length = dash_length;
          break;
        }
      }
    }
  }

  ExtractPathBound extract_path_bound(
      context_, world_model_, lane_borrow_info, baseline_info,
      config_.lateral_behavior_planner_config());
  bool has_drivable_space;
  BlockType block_type;
  TrackedObject temp;
  for (auto iter = ignore_change_true_.begin();
       iter != ignore_change_true_.end();) {
    if (lateral_obstacle.find_track(*iter, temp) == false) {
      iter = ignore_change_true_.erase(iter);
    } else {
      iter++;
    }
  }

  for (auto iter = ignore_change_false_.begin();
       iter != ignore_change_false_.end();) {
    if (lateral_obstacle.find_track(*iter, temp) == false) {
      iter = ignore_change_false_.erase(iter);
    } else {
      iter++;
    }
  }

  double lane_borrow_width = 1.0e6;

  auto calc_start = MTIME()->timestamp();
  has_drivable_space = extract_path_bound.process(
      lane_borrow_width, lane_borrow_length, avd_obstacles, avd_obstacle_prior_,
      block_type);

  auto calc_end = MTIME()->timestamp();

  auto calculate_cost_time = (calc_end - calc_start).sec();
  MSD_LOG(INFO,
          "===================rel_time LATERAL PATH BOUND EXTRACT TIME: %f s",
          calculate_cost_time);

  MSD_LOG(INFO,
          "[LateralBehaviorPlanner] has_drivable_space[%d] block_type.type[%d]",
          has_drivable_space, block_type.type);

  int current_lane_index = world_model_->get_map_info().current_lane_index();
  std::string lane_bias =
      world_model_->get_mutable_map_info_manager().lane_bias();
  double dis_left_lane_border = 1.8;
  double dis_right_lane_border = 1.8;
  for (const auto p : world_model_->get_map_info().current_refline_points()) {
    if (p.car_point.x > 0) {
      dis_left_lane_border = p.distance_to_left_lane_border;
      dis_right_lane_border = p.distance_to_right_lane_border;
      break;
    }
  }

  for (auto it = avd_info_.begin(); it != avd_info_.end(); ++it) {
    bool cutin_inter = false;
    if (it->second.ignore == false &&
        (lc_status != "left_lane_change" && lc_status != "right_lane_change") &&
        ((it->second.avd_direction == "right" && lane_bias == "left" &&
          dis_right_lane_border > 1.9) ||
         (it->second.avd_direction == "left" && lane_bias == "right" &&
          dis_left_lane_border > 1.9))) {
      TrackedObject temp;
      if (lateral_obstacle.find_track(it->first, temp)) {
        if (temp.v_lead > 0.5) {
          cutin_inter = true;
        }
      }
    }
    if (block_type.obs_avd_time.find(it->first) !=
        block_type.obs_avd_time.end()) {
      it->second.blocked_time_end =
          std::get<1>(block_type.obs_avd_time[it->first]);
      if (cutin_inter)
        continue;
      it->second.ignore = std::get<0>(block_type.obs_avd_time[it->first]);
    } else {
      it->second.blocked_time_end = 8;
      if (cutin_inter)
        continue;
      it->second.ignore = true;
    }
  }

  const auto &obstacle_manager = baseline_info_->obstacle_manager();
  for (auto it = avd_info_.begin(); it != avd_info_.end(); ++it) {
    auto &obs_priority = extract_path_bound.get_obs_priority();
    auto ptr_priority = obs_priority.find(it->first);
    if (ptr_priority != obs_priority.end()) {
      it->second.avd_priority = ptr_priority->second;
    } else {
      it->second.avd_priority = 1;
    }
  }

  extract_path_bound.update_lon_info(s_v_limit_, s_a_limit_, s_r_offset_);
}

bool LateralBehaviorPlanner::update_cutin_obstacles(Obstacle *obstacle) {
  // decide CutIn attribute

  const EgoState &ego_state = baseline_info_->get_ego_state();
  const double half_length =
      ConfigurationContext::Instance()->get_vehicle_param().length / 2.0;
  const double half_width =
      ConfigurationContext::Instance()->get_vehicle_param().width / 2.0;
  const auto &frenet_coord = baseline_info_->get_frenet_coord();
  double lat_dist_head = 99, lat_dist_tail = 99, lat_dist_head_ego = 99;
  auto traj_point_0 = obstacle->GetPointAtTime(0.0);
  auto polygon_0 = obstacle->GetPolygonAtPoint(traj_point_0);
  double theta = planning_math::NormalizeAngle(obstacle->Yaw_relative_frenet());
  double obj_vx = obstacle->speed() * std::cos(theta);
  double obj_vy = obstacle->speed() * std::sin(theta);
  double long_ttc_tail = 99; // ttc between ego head and obj tail
  double long_ttc_head = 99; // ttc between ego head and obj head
  double rel_vx = ego_state.ego_vel - obj_vx;
  double ego_s = ego_state.ego_frenet.x;
  double ego_l = ego_state.ego_frenet.y;
  double min_l = 99, max_l = -99;
  double min_s = 99, max_s = -99;
  double lat_ttc_head = 99, lat_ttc_tail = 99;
  Point2D obj_frenet_p{0.0, 0.0}, obj_carte_p{0.0, 0.0};
  MSD_LOG(INFO, "DEBUG_CJ_Yaw:ID:%d,fusion_yaw:%.2f,pred_yaw:%.2f",
          obstacle->Id(), obstacle->PerceptionInfo().heading_yaw * 57.3,
          traj_point_0.path_point.theta * 57.3);
  // CJ: calculate ttc vy etc.
  for (auto carte_point : polygon_0.points()) {
    obj_carte_p.x = carte_point.x();
    obj_carte_p.y = carte_point.y();
    if (frenet_coord->CartCoord2FrenetCoord(obj_carte_p, obj_frenet_p) ==
        TRANSFORM_SUCCESS) {
      MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, obj_corner:[%.2f,%.2f]", obstacle->Id(),
              obj_frenet_p.x, obj_frenet_p.y);
      if (min_l > obj_frenet_p.y) {
        min_l = obj_frenet_p.y;
      }
      if (max_l < obj_frenet_p.y) {
        max_l = obj_frenet_p.y;
      }
      if (min_s > obj_frenet_p.x) {
        min_s = obj_frenet_p.x;
      }
      if (max_s < obj_frenet_p.x) {
        max_s = obj_frenet_p.x;
      }
    } else {
      min_l = obstacle->R_frenet();
      max_l = obstacle->R_frenet();
      min_s = obstacle->S_frenet();
      max_s = obstacle->S_frenet();
    }
  }
  if (rel_vx > 0.1 && min_s - ego_state.ego_frenet.x - half_length > 0.1) {
    long_ttc_tail = (min_s - ego_state.ego_frenet.x - half_length) / rel_vx;
  }
  if (rel_vx > 0.1 && max_s - ego_state.ego_frenet.x - half_length > 0.1) {
    long_ttc_head = (max_s - ego_state.ego_frenet.x - half_length) / rel_vx;
  }
  if (obj_vy < -0.1) {
    lat_dist_head = min_l;
    lat_dist_head_ego = min_l - ego_l;
    lat_dist_tail = max_l - ego_l;
  } else if (obj_vy > 0.1) {
    lat_dist_head = max_l;
    lat_dist_head_ego = max_l - ego_l;
    lat_dist_tail = min_l - ego_l;
  } else {
    lat_dist_head = std::fabs(min_l) < std::fabs(max_l) ? min_l : max_l;
    lat_dist_head_ego = lat_dist_head - ego_l;
    lat_dist_tail = std::fabs(min_l) > std::fabs(max_l) ? min_l : max_l - ego_l;
  }

  if (obj_vy * lat_dist_head < 0) {
    lat_ttc_head = std::fmax((std::fabs(lat_dist_head) - half_width), 0) /
                   std::fmax(0.1, std::fabs(obj_vy));
  } else {
    lat_ttc_head = 0;
  }
  if (obj_vy * lat_dist_tail < 0) {
    lat_ttc_tail = (std::fabs(lat_dist_tail) + half_width) /
                   std::fmax(0.1, std::fabs(obj_vy));
  } else {
    lat_ttc_tail = 0;
  }
  double refline_cur = frenet_coord->GetRefCurveCurvature(max_s);

  bool cut_in_flag = false;
  const double K_CutInVyThr1 = 0.3;
  const double K_CutInDyTrh1 = half_width + 2.0;
  const double K_CutInLatTTCThr = 7.0;
  const double K_CutInHeadwayRange_s = 3.6;
  const double K_CutInDistMin_m = 30;
  const double K_CutInDistMax_m = 60;
  const double K_LargeYawThr_rad = 5 / 57.3;
  const double K_LargeYawLatBuff_m = 1.0;
  const double K_LargeYawVyThr_mps = 0.1;
  const double K_CutInVyThr_mps = 0.1;
  const double K_CutInSpeedLimit = 0.5;
  auto ref_line = world_model_->get_map_info().current_refline_points();
  double lane_width = 3.75;
  if (ref_line.size() > 0) {
    lane_width = std::fmin(ref_line[0].lane_width, 3.75);
  }
  MSD_LOG(INFO, "DEBUG_CJ884 ID:%d, Lane_Width:%.2f,ego_width:%.2f",
          obstacle->Id(), lane_width, half_width * 2);
  double CutInDistRange = ego_state.ego_vel * K_CutInHeadwayRange_s;
  CutInDistRange = std::fmax(CutInDistRange, K_CutInDistMin_m);
  CutInDistRange = std::fmin(CutInDistRange, K_CutInDistMax_m);

  bool long_dis_obj = long_ttc_head * rel_vx > 80.0;
  // CJ: 1.ego car is not stopped
  // 2.obj is getting closer in lat
  // 3.obj is ahead of ego
  // 4.obj is not too far(far obj doesn't need quick cut-in detection)
  // 5.lat ttc is small & obj vy is obvious & lat dist is short
  // 6.low speed large yaw obj
  const double ego_stop_spd_mps = 0.5;
  const double K_FusionDisableCurThr = 0.0015;
  bool rule_base_cutin =
      obj_vy * lat_dist_head < 0 && max_s - ego_s > half_length &&
      max_s - ego_s < CutInDistRange && obstacle->speed() > K_CutInSpeedLimit &&
      std::fabs(refline_cur) < K_FusionDisableCurThr &&
      ((lat_ttc_head < K_CutInLatTTCThr &&
        std::fabs(lat_dist_head) < K_CutInDyTrh1 &&
        std::fabs(obj_vy) > K_CutInVyThr_mps) ||
       (theta * lat_dist_head < 0 && std::fabs(theta) > K_LargeYawThr_rad &&
        std::fabs(lat_dist_head) < lane_width / 2 + K_LargeYawLatBuff_m &&
        std::fabs(obj_vy) > K_LargeYawVyThr_mps));
  MSD_LOG(
      INFO,
      "zh_debug ID:%d, objs_s:%.2f,ego_s:%.2f,s_lim:%.2f,prediction_cutin:%d, "
      "rule_base_cutin: %d",
      obstacle->Id(), max_s, ego_s, CutInDistRange, obstacle->IsCutin(),
      rule_base_cutin);
  MSD_LOG(INFO, "zh_debug ID:%d, lat_dist_head = %.2f", obstacle->Id(),
          lat_dist_head);
  MSD_LOG(INFO, "zh_debug ID:%d, obj_vy = %.2f, obstacle->speed() = %.2f",
          obstacle->Id(), obj_vy, obstacle->speed());
  MSD_LOG(INFO, "zh_debug ID:%d, std::fabs(refline_cur) = %.2f", obstacle->Id(),
          std::fabs(refline_cur));
  MSD_LOG(INFO, "zh_debug ID:%d, lat_ttc_head = %.2f", obstacle->Id(),
          lat_ttc_head);
  MSD_LOG(INFO, "zh_debug ID:%d, theta = %.2f", obstacle->Id(), theta);

  const double ftp_cutin_deactivation_speed = 100 / 3.6;

  const double ego_vel = baseline_info_->get_ego_state().ego_vel;
  if ((ftp_cutin_deactivation_speed > ego_vel) &&
      (ego_vel > ego_stop_spd_mps) &&
      (obstacle->IsCutin() || rule_base_cutin)) {
    cut_in_flag = true;
  } else if (ego_vel > ftp_cutin_deactivation_speed && rule_base_cutin) {
    cut_in_flag = true;
  }
  obstacle->SetCutinProperty(cut_in_flag);
  return cut_in_flag;
}

void LateralBehaviorPlanner::ignore_cutin_avd_obstacles() {
  auto &obstacle_manager = baseline_info_->obstacle_manager();
  if (!baseline_info_ || !baseline_info_->is_valid()) {
    MSD_LOG(INFO, "LateralBehaviorPlanner baseline %d invalid",
            virtual_lane_mgr_->mutable_fix_refline().position());
    return;
  }

  for (auto it = avd_info_.begin(); it != avd_info_.end();) {
    MSD_LOG(INFO, "it first id = %d", it->first);
    auto obstacle =
        const_cast<Obstacle *>(obstacle_manager.find_obstacle(it->first));
    if (!obstacle) {
      it++;
      continue;
    }

    // bool is_cutin =
    // baseline_info_->is_obstacle_intersect_with_lane(obstacle);
    bool is_cutin = update_cutin_obstacles(obstacle);
    MSD_LOG(INFO, "id = %d, is_cutin = %d", it->first, is_cutin);
    if (is_cutin) {
      MSD_LOG(INFO, "erase avd info id = %d", it->first);
      it = avd_info_.erase(it++);
    } else {
      it++;
    }
  }
}

} // namespace msquare
