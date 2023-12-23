#include "planner/behavior_planner/lateral_behavior_path_planner.h"
#include "common/lateral_virtual_lane.h"
#include "common/obstacle_manager.h"
#include "planner/behavior_planner/lateral_behavior_state.h"

namespace msquare {

void calc_desired_path(const std::array<double, 4> &l_poly,
                       const std::array<double, 4> &r_poly, double l_prob,
                       double r_prob, double intercept_width,
                       std::array<double, 4> &d_poly) {
  std::array<double, 4> half_lane_poly{0, 0, 0, intercept_width / 2};

  if (l_prob + r_prob > 0) {
    for (size_t i = 0; i < d_poly.size(); i++) {
      d_poly[i] = ((l_poly[i] - half_lane_poly[i]) * l_prob +
                   (r_poly[i] + half_lane_poly[i]) * r_prob) /
                  (l_prob + r_prob);
      if (std::isnan(d_poly[i])) {
        d_poly[i] = 0.;
      }
    }
  } else {
    d_poly.fill(0);
  }
}

PathPlanner::PathPlanner(const std::shared_ptr<WorldModel> &world_model,
                         VirtualLaneManager &virtual_lane_mgr)
    : world_model_(world_model), virtual_lane_mgr_(virtual_lane_mgr) {
  c_poly_.fill(0);
  d_poly_.fill(0);
  l_poly_.fill(0);
  r_poly_.fill(0);
}

void PathPlanner::update(
    int status, bool flag_avd, bool should_premove, bool should_suspend,
    const std::array<std::vector<double>, 2> &avd_car_past,
    const std::array<std::vector<double>, 2> &avd_sp_car_past) {

  auto &flane = virtual_lane_mgr_.get_fix_lane();

  c_poly_[3] = flane.dist_to_center_line();
  d_poly_[3] = flane.dist_to_center_line();
  l_poly_.fill(0);
  r_poly_.fill(0);

  if (status == ROAD_LC_LWAIT || status == ROAD_LC_RWAIT ||
      status == ROAD_LC_LBACK || status == ROAD_LC_RBACK) {

    update_premove_path(status, should_premove, should_suspend, avd_car_past);
  } else {
    premoving_ = false;
  }

  if (status >= ROAD_NONE && status <= ROAD_LC_RBACK) {
    update_avoidance_path(status, flag_avd, should_premove, avd_car_past,
                          avd_sp_car_past);
  } else {
    lat_offset_ = 0;
  }
}

void PathPlanner::update_premove_path(
    int status, bool should_premove, bool should_suspend,
    const std::array<std::vector<double>, 2> &avd_car_past) {

  auto &flane = virtual_lane_mgr_.get_fix_lane();
  auto &ego_state =
      world_model_->get_baseline_info(flane.position())->get_ego_state();

  if (flane.status() == BOTH_MISSING) {
    return;
  }

  double car_width = 2.2;

  double v_ego = ego_state.ego_vel;

  double lane_width = flane.width();

  l_poly_.fill(0);
  r_poly_.fill(0);
  l_poly_[3] = flane.intercepts()[0];
  r_poly_[3] = flane.intercepts()[1];

  std::array<double, 3> xp1{10, 20, 30};
  std::array<double, 3> fp1{0.15, 0.3, 0.45};

  double norminal_move =
      0.5 * (lane_width - car_width) - interp(v_ego, xp1, fp1);

  std::array<double, 2> xp2{0.5 * lane_width, 0.5 * (lane_width + car_width)};
  std::array<double, 2> fp2{0, norminal_move};

  double temp_ego = std::min(10.0 / std::max(v_ego, 0.1), 1.0);
  d_poly_[3] = flane.dist_to_center_line();

  if ((status == ROAD_LC_LWAIT && should_premove) || status == ROAD_LC_LBACK) {
    premoving_ = true;

    if (avd_car_past[0].size() > 0 && avd_car_past[0][5] > 0 &&
        avd_car_past[0][5] < lane_width) {

      lat_offset_ = interp(avd_car_past[0][5], xp2, fp2) * temp_ego;
      d_poly_[3] = flane.dist_to_center_line() + lat_offset_;
    } else if (avd_car_past[1].size() > 0 && avd_car_past[1][5] > 0 &&
               avd_car_past[1][5] < lane_width) {

      lat_offset_ = interp(avd_car_past[1][5], xp2, fp2) * temp_ego;
      d_poly_[3] = flane.dist_to_center_line() + lat_offset_;

    } else {
      lat_offset_ = norminal_move;

      if (should_suspend && lat_offset_ < -d_poly_[3]) {
        lat_offset_ = -d_poly_[3];
      }

      d_poly_[3] += lat_offset_;
    }
  } else if ((status == ROAD_LC_RWAIT && should_premove) ||
             status == ROAD_LC_RBACK) {
    premoving_ = true;

    if (avd_car_past[0].size() > 0 && avd_car_past[0][6] < 0 &&
        avd_car_past[0][6] > -lane_width) {
      lat_offset_ = -interp(avd_car_past[0][6], xp2, fp2) * temp_ego;
      d_poly_[3] = flane.dist_to_center_line() + lat_offset_;
    } else if (avd_car_past[1].size() > 0 && avd_car_past[1][6] < 0 &&
               avd_car_past[1][6] > -lane_width_) {
      lat_offset_ = -interp(avd_car_past[1][6], xp2, fp2) * temp_ego;
      d_poly_[3] = flane.dist_to_center_line() + lat_offset_;
    } else {
      lat_offset_ = -norminal_move;

      if (should_suspend && lat_offset_ > -d_poly_[3]) {
        lat_offset_ = -d_poly_[3];
      }

      d_poly_[3] += lat_offset_;
    }
  } else {
    premoving_ = false;
  }
  MSD_LOG(INFO, "WR: premoving_[%d]", premoving_);
}

void PathPlanner::update_avoidance_path(
    int status, bool flag_avd, bool should_premove,
    const std::array<std::vector<double>, 2> &avd_car_past,
    const std::array<std::vector<double>, 2> &avd_sp_car_past) {

  auto &map_info = world_model_->get_map_info();
  auto &map_info_mgr = world_model_->get_map_info_manager();
  auto &flane = virtual_lane_mgr_.get_fix_lane();
  auto &f_refline = virtual_lane_mgr_.get_fix_refline();
  auto &ego_state =
      world_model_->get_baseline_info(f_refline.position())->get_ego_state();

  double v_cruise = map_info_mgr.get_map_info().v_cruise();
  double lane_width = flane.width();
  double min_width = flane.min_width();
  double max_width = flane.max_width();
  lane_width = clip(lane_width, max_width, min_width);

  double entrance_lane_width = lane_width;

  if (map_info.is_in_intersection()) {
    for (auto &p : map_info.current_refline_points()) {
      if (p.car_point.x >= 0 && p.in_intersection == false &&
          p.distance_to_left_lane_border < 10. &&
          p.distance_to_right_lane_border < 10.) {
        entrance_lane_width =
            p.distance_to_left_lane_border + p.distance_to_right_lane_border;
        break;
      }
    }

    if (entrance_lane_width < lane_width) {
      if (std::fabs(map_info.intsect_length() - DBL_MAX) < 1e-2) {
        std::array<double, 2> xp{20, 30};
        std::array<double, 2> fp{entrance_lane_width, lane_width};
        lane_width = interp(map_info.dist_to_last_intsect(), xp, fp);
      }
    }
  } else {
    for (auto &p : map_info.current_refline_points()) {
      if (p.car_point.x >= 0 && p.in_intersection == false) {
        if (p.lane_width < 10.) {
          lane_width = p.lane_width;
        } else if (p.distance_to_left_lane_border < 10. &&
                   p.distance_to_right_lane_border < 10.) {
          lane_width =
              p.distance_to_left_lane_border + p.distance_to_right_lane_border;
        } else if (p.distance_to_left_lane_border < 10.) {
          lane_width = p.distance_to_left_lane_border * 2;
        } else if (p.distance_to_right_lane_border < 10.) {
          lane_width = p.distance_to_right_lane_border * 2;
        }
        break;
      }
    }
  }

  double avd_limit_l = 0.15 * lane_width;
  double avd_limit_r = 0.15 * lane_width;

  bool isRedLightStop =
      world_model_->get_traffic_light_decision()->get_stop_flag();

  int lane_type = flane.type();

  l_poly_.fill(0);
  r_poly_.fill(0);
  if (flane.status() != LaneStatusEx::BOTH_MISSING) {
    l_poly_[3] = flane.intercepts()[0];
    r_poly_[3] = flane.intercepts()[1];
  }

  std::array<double, 3> near_car_vrel_v{0.06 * lane_width, 0.02 * lane_width,
                                        0};
  std::array<double, 3> near_car_vrel_bp{-7.5, -3.5, 1};

  std::array<double, 3> near_car_drel_v{0.08 * lane_width, 0.05 * lane_width,
                                        0};
  std::array<double, 3> near_car_drel_bp{0, 20, 60};

  std::array<double, 3> t_gap_vego_v{1.35, 1.55, 2.0};
  std::array<double, 3> t_gap_vego_bp{5, 15, 30};

  double lat_offset = 0;
  double lat_offset1 = 0;
  double lat_offsetl = 0;
  double lat_offsetr = 0;

  int two_ncar = -1000;
  int one_ncarl = -1000;
  int one_ncarr = -1000;

  double v_ego = ego_state.ego_vel;
  double l_ego = ego_state.ego_frenet.y;
  double safety_dist = 2.0 + v_ego * 0.2;
  double t_gap = interp(v_ego, t_gap_vego_bp, t_gap_vego_v);

  double d_offset = 3.5;
  double car_width = 2.2;
  double avd_normal_thr = lane_width / 2 - 0.3 - car_width / 2;

  if (map_info.current_lane_index() != 0) {
    avd_normal_thr = lane_width / 2 - 0.1 - car_width / 2;
  }

  avd_car_past_ = avd_car_past;

  if (status == ROAD_NONE || status == ROAD_LC_LCHANGE ||
      status == ROAD_LC_RCHANGE || status == ROAD_LC_LWAIT ||
      status == ROAD_LC_RWAIT || status == ROAD_LC_LBACK ||
      status == ROAD_LC_RBACK) {
    if (avd_car_past[0].size() > 0) {
      double plus1 =
          interp(avd_car_past[0][2], near_car_vrel_bp, near_car_vrel_v);
      double plus1_rel =
          interp(avd_car_past[0][3], near_car_drel_bp, near_car_drel_v);
      double lat_compen1 = 0.5 * plus1 + 0.5 * plus1_rel;

      if (avd_car_past[1].size() > 0) {
        double v_near_car2 = v_ego + avd_car_past[1][2];
        double desired_dist2 = d_offset + std::fabs(v_near_car2) * t_gap;
        double diff_dncar = avd_car_past[1][3] - avd_car_past[0][3];
        double diff_vncar = avd_car_past[1][2] - avd_car_past[0][2];
        double d_avd_ncar1 = avd_car_past[0][3] + 5.0 + safety_dist;

        double t_avd_car1 = 0;
        // if(avd_car_past[0][2] != 0){
        if (equal_zero(avd_car_past[0][2]) == false) {
          if (avd_car_past[0][2] < -1.0e-3) {
            t_avd_car1 = -d_avd_ncar1 / avd_car_past[0][2];
          } else {
            t_avd_car1 = 5;
          }
        } else {
          t_avd_car1 = (v_ego < 1) ? 5 : 0;
        }

        double plus2 =
            interp(avd_car_past[1][2], near_car_vrel_bp, near_car_vrel_v);
        double plus2_rel =
            interp(avd_car_past[1][3], near_car_drel_bp, near_car_drel_v);
        double lat_compen2 = 0.5 * plus2 + 0.5 * plus2_rel;

        if (t_avd_car1 > 0) {
          diff_dncar += diff_vncar * t_avd_car1;
          if (avd_car_past[0][5] > 0 && avd_car_past[1][5] < 0) {
            if (diff_dncar < desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego &&
                avd_car_past[0][5] - avd_car_past[1][6] > 2.8) {
              lat_offset = avd_car_past[0][5] -
                           (avd_car_past[0][5] - avd_car_past[1][6]) / 2;

              if (lat_compen1 > lat_compen2) {
                if (lat_offset - (lat_compen1 - lat_compen2) -
                        avd_car_past[1][6] >=
                    1.4) {
                  lat_offset -= lat_compen1 - lat_compen2;
                } else {
                  lat_offset -= 0.1;
                }
              } else if (lat_compen1 < lat_compen2) {
                if (avd_car_past[0][5] -
                        (lat_offset - (lat_compen1 - lat_compen2)) >=
                    1.4) {
                  lat_offset -= lat_compen1 - lat_compen2;
                } else {
                  lat_offset += 0.1;
                }
              }

              if (lat_offset >= 0) {
                if (avd_normal_thr > 0) {
                  lat_offset = std::min(
                      lat_offset, std::min(avd_normal_thr, avd_car_past[0][9]));
                } else {
                  lat_offset =
                      std::min(lat_offset, std::min(0.5 * lane_width - 0.9,
                                                    avd_car_past[0][9]));
                }
              } else {
                if (avd_normal_thr > 0) {
                  lat_offset =
                      -std::min(std::fabs(lat_offset),
                                std::min(avd_normal_thr, avd_car_past[0][9]));
                } else {
                  lat_offset = -std::min(
                      std::fabs(lat_offset),
                      std::min(0.5 * lane_width - 0.9, avd_car_past[0][9]));
                }
              }

              curr_time_ = get_system_time();
            } else if (avd_car_past[1][6] < -1.5 &&
                       diff_dncar >=
                           desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
              if (get_system_time() - curr_time_ > 2) {
                lat_offset = 0.5 * (lane_width - car_width / 2 -
                                    std::fabs(avd_car_past[0][5])) +
                             lat_compen1;

                lat_offset = std::max(lat_offset, 0.0);

                if (avd_normal_thr > 0) {
                  lat_offset = -std::min(
                      lat_offset, std::min(avd_normal_thr, avd_car_past[0][9]));
                } else {
                  lat_offset =
                      -std::min(lat_offset, std::min(0.5 * lane_width - 0.9,
                                                     avd_car_past[0][9]));
                }
              } else {
                lat_offset = -d_poly_[3];
              }
            } else {
              if (avd_car_past[0][2] <= avd_car_past[1][2] ||
                  avd_car_past[0][2] - avd_car_past[1][2] < 2) {
                if (std::fabs(avd_car_past[0][5] - 100) > 1e-5) {
                  lat_offset = 0.9 * (lane_width - car_width / 2 -
                                      std::fabs(avd_car_past[0][5])) +
                               lat_compen1;
                }

                lat_offset = std::max(lat_offset, 0.0);
                lat_offset =
                    -std::min(lat_offset, std::min(0.5 * lane_width - 0.9,
                                                   avd_car_past[0][9]));

                if (avd_car_past[1][6] >= 0 && int(avd_car_past[1][6]) != 100 &&
                    (avd_car_past[1][2] + v_ego < 1.5 ||
                     avd_car_past[0][3] < 0) &&
                    avd_car_past[0][4] > -0.5 && avd_car_past_[0][4] < 0.3 &&
                    std::fabs(avd_car_past[0][1]) < 1e-5) {
                  lat_offset =
                      avd_car_past[1][5] -
                      (avd_car_past[1][5] + 1.8 + avd_car_past[0][9]) / 2;

                  lat_offset = std::max(lat_offset, -0.5 * lane_width + 0.9);
                } else if ((std::fabs(avd_car_past[0][7] - 20001) < 1e-5 ||
                            std::fabs(avd_car_past[1][7] - 20001) < 1e-5) &&
                           map_info.is_in_intersection()) {
                  if (std::fabs(avd_car_past[1][5] - 1.0) >
                      (avd_car_past[0][6] + 1.3)) { // need consider more
                    lat_offset = avd_car_past[0][6] + 1.3;
                  } else {
                    lat_offset = avd_car_past[1][5] - 1.3;
                  }
                } else {
                  avd_car_past_[1].clear();
                }
              } else {
                if (avd_car_past[0][5] - avd_car_past[1][6] > 2.6) {
                  lat_offset = avd_car_past[0][5] -
                               (avd_car_past[0][5] - avd_car_past[1][6]) / 2;
                } else {
                  if (int(avd_car_past[1][6]) != 100) {
                    lat_offset = 0.9 * (lane_width - car_width / 2 -
                                        std::fabs(avd_car_past[1][6])) +
                                 lat_compen1;
                  }

                  lat_offset = std::max(lat_offset, 0.0);
                  lat_offset =
                      std::min(lat_offset, std::min(0.5 * lane_width - 0.9,
                                                    avd_car_past[0][9]));
                  avd_car_past_[0] = avd_car_past[1];
                  avd_car_past_[1].clear();
                }
              }
            }
          } else if (avd_car_past[0][5] < 0 && avd_car_past[1][5] > 0) {
            if (diff_dncar < desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego &&
                std::fabs(avd_car_past[0][6] - avd_car_past[1][5]) > 2.8) {
              lat_offset = avd_car_past[0][6] -
                           (avd_car_past[0][6] - avd_car_past[1][5]) / 2;
              if (lat_compen1 > lat_compen2) {
                if (avd_car_past[1][5] -
                        (lat_offset + (lat_compen1 - lat_compen2)) >=
                    1.4) {
                  lat_offset += lat_compen1 - lat_compen2;
                } else {
                  lat_offset += 0.1;
                }
              } else if (lat_compen1 < lat_compen2) {
                if (lat_offset + (lat_compen1 - lat_compen2) -
                        avd_car_past[0][6] >=
                    1.4) {
                  lat_offset += lat_compen1 - lat_compen2;
                } else {
                  lat_offset -= 0.1;
                }
              }

              if (lat_offset >= 0) {
                if (avd_normal_thr > 0) {
                  lat_offset = std::min(
                      lat_offset, std::min(avd_normal_thr, avd_car_past[0][9]));
                } else {
                  lat_offset =
                      std::min(lat_offset, std::min(0.5 * lane_width - 0.9,
                                                    avd_car_past[0][9]));
                }
              } else {
                if (avd_normal_thr > 0) {
                  lat_offset =
                      -std::min(std::fabs(lat_offset),
                                std::min(avd_normal_thr, avd_car_past[0][9]));
                } else {
                  lat_offset = -std::min(
                      std::fabs(lat_offset),
                      std::min(0.5 * lane_width - 0.9, avd_car_past[0][9]));
                }
              }

              curr_time_ = get_system_time();
            } else if (avd_car_past[0][6] < -1.5 &&
                       diff_dncar >=
                           desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
              if (get_system_time() - curr_time_ > 2) {
                lat_offset = 0.5 * (lane_width - car_width / 2 -
                                    std::fabs(avd_car_past[0][6])) +
                             lat_compen1;
                lat_offset = std::max(lat_offset, 0.0);

                if (avd_normal_thr > 0) {
                  lat_offset = std::min(
                      lat_offset, std::min(avd_normal_thr, avd_car_past[0][9]));
                } else {
                  lat_offset =
                      std::min(lat_offset, std::min(0.5 * lane_width - 0.9,
                                                    avd_car_past[0][9]));
                }
              } else {
                lat_offset = -d_poly_[3];
              }
            } else {
              if (avd_car_past[0][2] <= avd_car_past[1][2] ||
                  avd_car_past[0][2] - avd_car_past[1][2] < 2) {
                if (int(avd_car_past[0][6]) != 100) {
                  lat_offset = 0.9 * (lane_width - car_width / 2 -
                                      std::fabs(avd_car_past[0][6])) +
                               lat_compen1;
                }

                lat_offset = std::max(lat_offset, 0.0);
                lat_offset =
                    std::min(lat_offset, std::min(0.5 * lane_width - 0.9,
                                                  avd_car_past[0][9]));

                if (avd_car_past[0][6] >= 0 && int(avd_car_past[0][6]) != 100 &&
                    (avd_car_past[0][2] + v_ego < 1.5 ||
                     avd_car_past[0][3] < 0) &&
                    avd_car_past[0][4] > -0.5 && avd_car_past[0][4] < 0.3 &&
                    std::fabs(avd_car_past[0][1]) < 1e-5) {
                  lat_offset =
                      avd_car_past[0][5] -
                      (avd_car_past[0][5] + 1.8 + avd_car_past[0][9]) / 2;

                  lat_offset = std::max(lat_offset, -0.5 * lane_width + 0.9);
                } else if ((std::fabs(avd_car_past[0][7] - 20001) < 1e-5 ||
                            std::fabs(avd_car_past[1][7] - 20001) < 1e-5) &&
                           map_info.is_in_intersection()) {
                  if (std::fabs(avd_car_past[0][5] - 1.0) >
                      (avd_car_past[1][6] + 1.3)) { // need consider more
                    lat_offset = avd_car_past[1][6] + 1.3;
                  } else {
                    lat_offset = avd_car_past[0][5] - 1.3;
                  }
                } else {
                  avd_car_past_[1].clear();
                }
              } else {
                if (std::fabs(avd_car_past[0][6] - avd_car_past[1][5]) > 2.6) {
                  lat_offset = avd_car_past[0][6] -
                               (avd_car_past[0][6] - avd_car_past[1][5]) / 2;
                } else {
                  if (int(avd_car_past[1][5]) != 100) {
                    lat_offset = 0.9 * (lane_width - car_width / 2 -
                                        std::fabs(avd_car_past[1][5])) +
                                 lat_compen1;
                  }

                  lat_offset = std::max(lat_offset, 0.0);
                  lat_offset =
                      -std::min(lat_offset, std::min(0.5 * lane_width - 0.9,
                                                     avd_car_past[0][9]));
                  avd_car_past_[0] = avd_car_past[1];
                  avd_car_past_[1].clear();
                }
              }
            }
          } else if (avd_car_past[0][5] > 0 && avd_car_past[1][5] > 0) {
            if (diff_dncar < desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
              if (int(avd_car_past[0][5]) != 100 ||
                  int(avd_car_past[1][5]) != 100) {
                lat_offset =
                    std::min(0.5 * (lane_width - car_width / 2 -
                                    std::fabs(std::min(avd_car_past[0][5],
                                                       avd_car_past[1][5]))) +
                                 lat_compen1,
                             -(std::fabs(std::min(avd_car_past[0][5],
                                                  avd_car_past[1][5])) -
                               car_width / 2 - 1.0));
              }
            } else {
              if (int(avd_car_past[0][5]) != 100) {
                lat_offset = std::min(
                    0.5 * (lane_width - car_width / 2 -
                           std::fabs(avd_car_past[0][5])) +
                        lat_compen1,
                    -(std::fabs(avd_car_past[0][5]) - car_width / 2 - 1.0));
              }
            }

            lat_offset = std::max(lat_offset, 0.0);

            if ((map_info.current_lane_index() != map_info.lanes_num() - 1 &&
                 !map_info_mgr.rlane_.exist()) ||
                std::fabs(avd_limit_l - 0.2) < 1e-5 ||
                (avd_car_past[0][2] + v_ego >= 1.5 &&
                 avd_car_past[0][3] >= 0)) {
              if (avd_normal_thr > 0 && map_info.dist_to_intsect() > 0) {
                lat_offset =
                    -std::min(std::min(lat_offset, avd_normal_thr),
                              std::min(avd_car_past[0][9], avd_limit_l));
              } else {
                lat_offset =
                    -std::min(std::min(lat_offset, 0.5 * lane_width - 0.9),
                              std::min(avd_car_past[0][9], avd_limit_l));
              }
            }

            if (lat_offset > 0) {
              lat_offset = -d_poly_[3];
            }

            if (flane.status() != LaneStatusEx::BOTH_MISSING &&
                flane.intercepts()[1] < 0 && flane.intercepts()[1] > -1.0 &&
                map_info.current_lane_index() == map_info.lanes_num() - 1 &&
                std::fabs(avd_limit_l - 0.2) < 1e-5) {
              lat_offset =
                  std::min(lat_offset + flane.intercepts()[1] + 1.0, 0.0);
            }
          } else if (avd_car_past[0][5] < 0 && avd_car_past[1][5] < 0) {
            if (diff_dncar < desired_dist2 + 5.0 + safety_dist + 0.5 * v_ego) {
              if (int(avd_car_past[0][6]) != 100 &&
                  int(avd_car_past[1][6]) != 100 && avd_car_past[0][6] < 0 &&
                  avd_car_past[1][6] < 0) {
                lat_offset =
                    std::min(0.5 * (lane_width - car_width / 2 -
                                    std::fabs(std::max(avd_car_past[0][6],
                                                       avd_car_past[1][6]))) +
                                 lat_compen1,
                             std::max(avd_car_past[0][6], avd_car_past[1][6]) +
                                 car_width / 2 + 1.0);
              } else if (int(avd_car_past[0][6]) != 100 &&
                         avd_car_past[0][6] < 0) {
                lat_offset =
                    std::min(0.5 * (lane_width - car_width / 2 -
                                    std::fabs(avd_car_past[0][6])) +
                                 lat_compen1,
                             (avd_car_past[0][6] + car_width / 2 + 1.0));
              } else if (int(avd_car_past[1][6]) != 100 &&
                         avd_car_past[1][6] < 0) {
                lat_offset =
                    std::min(0.5 * (lane_width - car_width / 2 -
                                    std::fabs(avd_car_past[1][6])) +
                                 lat_compen1,
                             (avd_car_past[1][6] + car_width / 2 + 1.0));
              }
            } else {
              if (int(avd_car_past[0][6]) != 100) {
                lat_offset = 0.5 * (lane_width - car_width / 2 -
                                    std::fabs(avd_car_past[0][6])) +
                             lat_compen1;
              }
            }

            if (avd_car_past[0][6] < 0 || avd_car_past[1][6] < 0) {
              lat_offset = std::max(lat_offset, 0.0);

              if (avd_normal_thr > 0) {
                lat_offset =
                    std::min(std::min(lat_offset, avd_normal_thr),
                             std::min(avd_car_past[0][9], avd_limit_r));
              } else {
                lat_offset =
                    std::min(std::min(lat_offset, 0.5 * lane_width - 0.9),
                             std::min(avd_car_past[0][9], avd_limit_r));
              }
            }

            if (flane.status() != LaneStatusEx::BOTH_MISSING &&
                flane.intercepts()[0] > 0 && flane.intercepts()[0] < 1.0 &&
                map_info.current_lane_index() == 0) {
              lat_offset =
                  std::max(lat_offset + flane.intercepts()[0] - 1.0, 0.0);
            }
          }
        }

        if (!(lat_offset > 0.8 || lat_offset < -0.8)) {
          if (std::fabs(lat_offset1) >= std::fabs(d_poly_[3]) &&
              ((d_poly_[3] > 0 && lat_offset < 0 &&
                d_poly_[3] + lat_offset > 0) ||
               (d_poly_[3] < 0 && lat_offset > 0 &&
                d_poly_[3] + lat_offset < 0))) {
            d_poly_[3] = 0;
          } else if (std::pow(avd_car_past[0][2] - 1, 2) / 4 >
                         avd_car_past[0][3] - 2 &&
                     avd_car_past[0][3] > -5) {
            d_poly_[3] += lat_offset;
            two_ncar = d_poly_[3];
          } else {
            d_poly_[3] += lat_offset;
            two_ncar = d_poly_[3];
          }
        }
      } else if (avd_car_past[0][5] > 0) {
        if ((int)avd_car_past[0][1] == -100) {
          lat_offset =
              -0.15 * lane_width * std::fabs(avd_car_past[0][10]) / 3.2;

          if (lane_type == 3 || std::fabs(d_poly_[1]) > 0.0001 ||
              (map_info.lanes_num() > 1 &&
               map_info.current_lane_index() == map_info.lanes_num() - 1)) {
            lat_offset = 0.8 * lat_offset;
          }

          if (std::fabs(avd_limit_l - 0.2) < 1e-5) {
            lat_offset = std::max(lat_offset, -avd_limit_l);
          }

          if (flane.status() != LaneStatusEx::BOTH_MISSING &&
              flane.intercepts()[1] < 0 && flane.intercepts()[1] > -1.0) {
            lat_offset += flane.intercepts()[1] + 1.0;
          }

          if (d_poly_[3] > 1.0) {
            lat_offset = -d_poly_[3] - 0.5;
          }
        } else if (((int)avd_car_past[0][0] == 0 &&
                    avd_car_past[0][3] > -6.0) ||
                   ((int)avd_car_past[0][0] != 0 &&
                    avd_car_past[0][3] >= -3.0)) {
          if (true) {
            if (int(avd_car_past[0][5]) != 100) {
              lat_offset = std::min(
                  0.5 * (lane_width - car_width / 2 -
                         std::fabs(avd_car_past[0][5])) +
                      lat_compen1,
                  -(std::fabs(avd_car_past[0][5]) - car_width / 2 - 1.0));
            }

            lat_offset = std::max(lat_offset, 0.0);
            if (avd_normal_thr > 0 && map_info.dist_to_intsect() > 0) {
              lat_offset = -std::min(std::min(lat_offset, avd_normal_thr),
                                     std::min(avd_car_past[0][9], avd_limit_l));
            } else {
              lat_offset =
                  -std::min(std::min(lat_offset, 0.5 * lane_width - 0.9),
                            std::min(avd_car_past[0][9], avd_limit_l));
            }

            if (avd_car_past[0][5] < 1.5) {
              if ((map_info.current_lane_index() != map_info.lanes_num() - 1 &&
                   !map_info_mgr.rlane_.exist()) ||
                  (avd_car_past[0][2] + v_ego >= 1.5 &&
                   avd_car_past[0][3] >= 0)) {
                lat_offset =
                    avd_car_past[0][5] -
                    (avd_car_past[0][5] + 1.8 + avd_car_past[0][9]) / 2;
                lat_offset = std::max(lat_offset, -0.5 * lane_width + 0.9);
              }

              if (map_info.lanes_num() > 1 &&
                  map_info.current_lane_index() == map_info.lanes_num() - 1) {
                lat_offset *= 0.8;
              }
            }

            if (std::fabs(avd_limit_l - 0.2) < 1e-5) {
              lat_offset = std::max(lat_offset, -avd_limit_l);
            }

            if (flane.status() != LaneStatusEx::BOTH_MISSING &&
                flane.intercepts()[1] < 0 && flane.intercepts()[1] > -1.0 &&
                map_info.current_lane_index() == map_info.lanes_num() - 1 &&
                std::fabs(avd_limit_l - 0.2) < 1e-5) {
              lat_offset =
                  std::min(lat_offset + flane.intercepts()[1] + 1.0, 0.0);
            }
          } else {
            if (int(avd_car_past[0][5]) != 100) {
              lat_offset = 0.5 * (lane_width - car_width / 2 -
                                  std::fabs(avd_car_past[0][5])) +
                           lat_compen1;
            }

            lat_offset = std::max(lat_offset, 0.0);
            lat_offset = -std::min(lat_offset, std::min(0.5 * lane_width - 0.9,
                                                        avd_car_past[0][9]));

            if (avd_car_past[0][5] < 1.5) {
              lat_offset = avd_car_past[0][5] -
                           (avd_car_past[0][5] + 1.8 + avd_car_past[0][9]) / 2;

              if (avd_car_past[0][5] >= 1.1) {
                lat_offset = std::max(lat_offset, -0.5 * lane_width + 0.9);
              } else if (lane_width <= 3.8) {
                lat_offset = std::max(lat_offset, -0.5 * lane_width + 0.9);
              } else {
                lat_offset =
                    std::max(lat_offset, -0.24 * lane_width / 4.4 * lane_width);
              }
            }
          }

          lat_offsetl = lat_offset;
        } else {
          if (map_info.dist_to_intsect() > 0) {
            lat_offset = std::min(-d_poly_[3], 0.);
          } else {
            lat_offset = std::min(
                std::max(avd_car_past[0][5] - 1.6,
                         avd_car_past[0][5] -
                             (avd_car_past[0][5] + 1.8 + avd_car_past[0][9]) /
                                 2),
                0.0);
          }

          if (std::fabs(lat_offset) > 0.5 * lane_width - 0.9) {
            lat_offset = -0.5 * lane_width + 0.9;
          }
        }

        if (!(lat_offset < -0.8 || lat_offset > 0.8)) {
          if (std::fabs(lat_offsetl) >= std::fabs(d_poly_[3]) &&
              d_poly_[3] > 0 && d_poly_[3] + lat_offset > 0) {
            d_poly_[3] = 0;
          } else if (std::pow(avd_car_past[0][2] - 1, 2) / 4 >
                         avd_car_past[0][3] - 2 &&
                     avd_car_past[0][3] > -5) {
            d_poly_[3] += lat_offset;
            two_ncar = d_poly_[3];
          } else {
            d_poly_[3] += lat_offset;
          }

          one_ncarl = d_poly_[3];
        }
      } else {
        if ((int)avd_car_past[0][1] == -200) {
          lat_offset = 0.15 * lane_width * std::fabs(avd_car_past[0][10]) / 3.2;

          if (lane_type == LaneType::PARKING ||
              std::fabs(d_poly_[1]) > 0.0001 ||
              (map_info.lanes_num() > 1 &&
               map_info.current_lane_index() == 0)) {
            lat_offset *= 0.8;
          }

          if (std::fabs(avd_limit_r - 0.2) < 1e-5) {
            lat_offset = std::min(lat_offset, avd_limit_r);
          }

          if (flane.status() != LaneStatusEx::BOTH_MISSING &&
              flane.intercepts()[0] > 0 && flane.intercepts()[0] < 1.0) {
            lat_offset += flane.intercepts()[0] - 1.0;
          }
        } else if (((int)avd_car_past[0][0] == 0 &&
                    avd_car_past[0][3] > -6.0) ||
                   ((int)avd_car_past[0][0] != 0 &&
                    avd_car_past[0][3] >= -3.0)) {
          if (true) {
            if (int(avd_car_past[0][6]) != 100) {
              lat_offset = std::min(0.5 * (lane_width - car_width / 2 -
                                           std::fabs(avd_car_past[0][6])) +
                                        lat_compen1,
                                    (avd_car_past[0][6] + car_width / 2 + 1.0));
            }

            lat_offset = std::max(lat_offset, 0.0);

            if (avd_normal_thr > 0) {
              lat_offset = std::min(std::min(lat_offset, avd_normal_thr),
                                    std::min(avd_car_past[0][9], avd_limit_r));
            } else {
              lat_offset =
                  std::min(std::min(lat_offset, 0.5 * lane_width - 0.9),
                           std::min(avd_car_past[0][9], avd_limit_r));
            }

            if (avd_car_past[0][6] <
                    -0.9 + std::max(lane_width / 2 - 1.8, 0.0) &&
                avd_car_past[0][6] > -1.5) {
              lat_offset = avd_car_past[0][6] -
                           (avd_car_past[0][6] - 1.8 - avd_car_past[0][9]) / 2;
              lat_offset = std::min(lat_offset, 0.15 * lane_width);

              if (map_info.lanes_num() > 1 &&
                  map_info.current_lane_index() == 0) {
                lat_offset *= 0.8;
              }
            }

            if (std::fabs(avd_limit_r - 0.2) < 1e-5) {
              lat_offset = std::min(lat_offset, avd_limit_r);
            }

            if (flane.status() != LaneStatusEx::BOTH_MISSING &&
                flane.intercepts()[0] > 0 && flane.intercepts()[0] < 1.0 &&
                map_info.current_lane_index() == 0) {
              lat_offset =
                  std::max(lat_offset + flane.intercepts()[0] - 1.0, 0.0);
            }
          } else {
            if (int(avd_car_past[0][6]) != 100) {
              lat_offset = 0.5 * (lane_width - car_width / 2 -
                                  std::fabs(avd_car_past[0][6])) +
                           lat_compen1;
            }

            lat_offset = std::max(lat_offset, 0.0);
            lat_offset = std::min(lat_offset, std::min(0.5 * lane_width - 0.9,
                                                       avd_car_past[0][9]));

            if (avd_car_past[0][6] > -1.5) {
              lat_offset = avd_car_past[0][6] -
                           (avd_car_past[0][6] - 1.8 - avd_car_past[0][9]) / 2;

              if (avd_car_past[0][6] < -1.1) {
                if (map_info.left_refline_points().size() != 0 ||
                    map_info.dist_to_last_intsect() > 20) {
                  lat_offset = std::min(lat_offset, 0.5 * lane_width - 0.9);
                } else {
                  if (avd_normal_thr > 0) {
                    lat_offset = std::min(lat_offset, avd_normal_thr);
                  } else {
                    lat_offset = std::min(lat_offset, 0.5 * lane_width - 0.9);
                  }
                }
              } else if (lane_width <= 3.8) {
                lat_offset =
                    std::min(lat_offset, std::min(0.5 * lane_width - 0.9, 0.7));
              } else {
                lat_offset =
                    std::min(lat_offset, 0.24 * lane_width / 4.4 * lane_width);
              }
            }
          }

          lat_offsetr = lat_offset;
        } else {
          if (map_info.dist_to_intsect() > 0) {
            lat_offset = std::max(-d_poly_[3], 0.);
          } else {
            lat_offset = std::max(
                std::min(1.6 + avd_car_past[0][6],
                         avd_car_past[0][6] -
                             (avd_car_past[0][6] - 1.8 - avd_car_past[0][9]) /
                                 2),
                0.0);
          }

          if (std::fabs(lat_offset) > 0.5 * lane_width - 0.9 &&
              avd_car_past[0][6] < 0) {
            lat_offset = 0.5 * lane_width - 0.9;
          }
        }

        if (!(lat_offset > 0.8 || lat_offset < -0.8)) {
          if (std::fabs(lat_offsetr) >= std::fabs(d_poly_[3]) &&
              d_poly_[3] < 0 && d_poly_[3] + lat_offset < 0) {
            d_poly_[3] = 0;
          } else if (std::pow(avd_car_past[0][2] - 1, 2) / 4 >
                         avd_car_past[0][3] - 2 &&
                     avd_car_past[0][3] > -5) {
            d_poly_[3] += lat_offset;
            two_ncar = d_poly_[3];
          } else {
            d_poly_[3] += lat_offset;
          }

          one_ncarr = d_poly_[3];
        }
      }
    }
  }

  two_ncar_ = two_ncar;
  one_ncarl_ = one_ncarl;
  one_ncarr_ = one_ncarr;
  lane_width_ = lane_width;

  if (premoving_) {
    lat_offset_ += lat_offset;
  } else {
    lat_offset_ = lat_offset;
  }
}

void PathPlanner::restore_context(const PathPlannerContext &context) {
  premoving_ = context.premoving;
  lane_width_ = context.lane_width;
  lat_offset_ = context.lat_offset;
  curr_time_ = context.curr_time;
  c_poly_ = context.c_poly;
  d_poly_ = context.d_poly;
  avd_car_past_ = context.avd_car_past;
}

void PathPlanner::save_context(PathPlannerContext &context) const {
  context.premoving = premoving_;
  context.lane_width = lane_width_;
  context.lat_offset = lat_offset_;
  context.curr_time = curr_time_;
  context.c_poly = c_poly_;
  context.d_poly = d_poly_;
  context.avd_car_past = avd_car_past_;
}

} // namespace msquare
