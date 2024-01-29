#include "common/map_info_manager.h"
#include "common/config_context.h"
#include "common/math/linear_interpolation.h"
#include "common/pass_intersection_planner_preprocessor.h"
#include "common/planning_context.h"
#include "common/utils/lateral_utils.h"
#include "common/utils/polyfit.h"
#include "common/utils/pose2d_utils.hpp"
#include "common/world_model.h"
#include "planning/common/common.h"
#include "pnc/define/pass_intersection_input_interface.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
using namespace pass_intersection_planner;

namespace msquare {

void MSDMapInfo::update_lane_boundary_segments() {
  for (auto &lane_data : lane_) {
    if (lane_data.left_lane_boundary.segments.size() > 0) {
      static std::vector<maf_worldmodel::LaneBoundarySegment> segments{};
      segments.clear();
      for (int i = 0; i < lane_data.left_lane_boundary.segments.size(); i++) {
        // consider none solid as dash
        if (lane_data.left_lane_boundary.segments[i].type.value.value !=
            LaneBoundaryForm::DASH) {
          lane_data.left_lane_boundary.segments[i].type.value.value =
              LaneBoundaryForm::SOLID;
        }
        if (i == 0) {
          segments.push_back(lane_data.left_lane_boundary.segments[0]);
          continue;
        }
        // set standalone short( < 8m) solid line to dash
        if (lane_data.left_lane_boundary.segments[i].type.value.value ==
                LaneBoundaryForm::SOLID &&
            lane_data.left_lane_boundary.segments[i].length < 8.0 &&
            segments.back().type.value.value != LaneBoundaryForm::SOLID &&
            i < ((int)lane_data.left_lane_boundary.segments.size() - 1) &&
            lane_data.left_lane_boundary.segments[i + 1].type.value.value !=
                LaneBoundaryForm::SOLID) {
          lane_data.left_lane_boundary.segments[i].type.value.value =
              LaneBoundaryForm::DASH;
        }
        // squash same type segment
        if (segments.back().type.value.value ==
            lane_data.left_lane_boundary.segments[i].type.value.value) {
          segments.back().length +=
              lane_data.left_lane_boundary.segments[i].length;
        } else {
          segments.push_back(lane_data.left_lane_boundary.segments[i]);
        }
      }
      lane_data.left_lane_boundary.segments = segments;
    }
    if (lane_data.right_lane_boundary.segments.size() > 0) {
      static std::vector<maf_worldmodel::LaneBoundarySegment> segments{};
      segments.clear();
      for (int i = 0; i < lane_data.right_lane_boundary.segments.size(); i++) {
        // consider none solid as dash
        if (lane_data.right_lane_boundary.segments[i].type.value.value !=
            LaneBoundaryForm::DASH) {
          lane_data.right_lane_boundary.segments[i].type.value.value =
              LaneBoundaryForm::SOLID;
        }
        if (i == 0) {
          segments.push_back(lane_data.right_lane_boundary.segments[0]);
          continue;
        }
        // set standalone short( < 8m) solid line to dash
        if (lane_data.right_lane_boundary.segments[i].type.value.value ==
                LaneBoundaryForm::SOLID &&
            lane_data.right_lane_boundary.segments[i].length < 8.0 &&
            segments.back().type.value.value != LaneBoundaryForm::SOLID &&
            i < ((int)lane_data.right_lane_boundary.segments.size() - 1) &&
            lane_data.right_lane_boundary.segments[i + 1].type.value.value !=
                LaneBoundaryForm::SOLID) {
          lane_data.right_lane_boundary.segments[i].type.value.value =
              LaneBoundaryForm::DASH;
        }
        if (segments.back().type.value.value ==
            lane_data.right_lane_boundary.segments[i].type.value.value) {
          segments.back().length +=
              lane_data.right_lane_boundary.segments[i].length;
        } else {
          segments.push_back(lane_data.right_lane_boundary.segments[i]);
        }
      }
      lane_data.right_lane_boundary.segments = segments;
    }
  }
}

void MSDMapInfo::update_lane_tasks() {
  current_tasks_.clear();
  left_lane_tasks_.clear();
  right_lane_tasks_.clear();

  auto fill_lane_tasks =
      [](const std::vector<LaneChangeState> &lane_change_state,
         std::vector<int8_t> &lane_tasks) {
        for (auto state : lane_change_state) {
          if (state.value == LaneChangeState::TURN_LEFT) {
            lane_tasks.push_back(-1);
          } else if (state.value == LaneChangeState::TURN_RIGHT) {
            lane_tasks.push_back(1);
          }
        }
      };

  fill_lane_tasks(lane_strategy_.strategy_start_with_current_lane,
                  current_tasks_);

  fill_lane_tasks(lane_strategy_.strategy_start_with_left_lane,
                  left_lane_tasks_);

  fill_lane_tasks(lane_strategy_.strategy_start_with_right_lane,
                  right_lane_tasks_);
}

void MSDMapInfo::update_speed_limit(double ego_vel, double ego_v_cruise,
                                    bool is_ddmap) {
  if (!self_position_.in_map_area) {
    v_cruise_ = ego_v_cruise;
    return;
  }

  auto &refline_points = current_refline_points();
  if (refline_points.size() > 1) {
    double last_speed;
    bool find_last = false;
    bool find_change = false;
    double acc_brake_min = 100.0;
    for (size_t i = 1; i < refline_points.size(); ++i) {
      if (!find_last && refline_points[i].car_point.x > 0.0) {
        find_last = true;
        last_speed = refline_points[i - 1].max_velocity;
        current_lane_speed_limit_ = last_speed;
        continue;
      }

      if (find_last &&
          std::fabs(refline_points[i].max_velocity - last_speed) > 1e-5) {
        double acc_brake =
            (std::pow(refline_points[i].max_velocity / 3.6, 2) -
             std::pow(ego_vel, 2)) /
            std::max(1.0,
                     std::sqrt(std::pow(refline_points[i].car_point.x, 2) +
                               std::pow(refline_points[i].car_point.y, 2)));
        if (acc_brake < acc_brake_min) {
          acc_brake_min = acc_brake;
          find_change = true;
          speed_change_point_.x = refline_points[i].car_point.x;
          speed_change_point_.y = refline_points[i].car_point.y;
          speed_change_point_.speed = refline_points[i].max_velocity;
        }
      }
    }

    if (!find_change) {
      speed_change_point_.x = refline_points.back().car_point.x;
      speed_change_point_.y = refline_points.back().car_point.y;
      speed_change_point_.speed = refline_points.back().max_velocity;
    }
  }

  // hack for 10~60 map speed limit, higher 20% for none intersect and on ramp
  if (distance_to_crossing() > 200.0 && is_on_ramp()) {
    if (speed_change_point_.speed > 10.0 && speed_change_point_.speed < 60.0) {
      speed_change_point_.speed *= 1.2;
    }
    if (current_lane_speed_limit_ > 10.0 && current_lane_speed_limit_ < 60.0) {
      current_lane_speed_limit_ *= 1.2;
    }
  }

  current_lane_speed_limit_ =
      std::min(current_lane_speed_limit_, ego_v_cruise * 3.6);

  v_cruise_ =
      std::min(current_lane_speed_limit_, speed_change_point_.speed) / 3.6;

  // use ego_v_cruise(80kph) and highway scene in ddmap
  if (is_ddmap) {
    current_lane_speed_limit_ = ego_v_cruise * 3.6;
    v_cruise_ = ego_v_cruise;
    speed_change_point_.x = 0.0;
    speed_change_point_.y = 0.0;
    scene_type_ = "highway";
  }
}

bool MSDMapInfo::is_on_highway() const {
  auto scene_type = ConfigurationContext::Instance()->scene_type();
  return scene_type == SceneType::HIGHWAY;
}

void MSDMapInfo::update_refline_points(
    const std::shared_ptr<WorldModel> &world_model) {
  const auto &enu2car = world_model->get_cart_ego_state_manager().get_enu2car();

  current_refline_points_.clear();
  left_refline_points_.clear();
  right_refline_points_.clear();
  left_left_refline_points_.clear();
  right_right_refline_points_.clear();

  auto fill_refline_points =
      [this](int index, std::vector<ReferenceLinePointDerived> &refline_points,
             const Transform &enu2car) {
        if (index < 0 || index >= lanes_num()) {
          return;
        }

        auto &refline_points_source =
            lane_[index].reference_line.reference_line_points;
        refline_points.resize(refline_points_source.size());
        for (size_t i = 0; i < refline_points.size(); i++) {
          refline_points[i].curvature = refline_points_source[i].curvature;
          refline_points[i].distance_to_left_lane_border =
              refline_points_source[i].distance_to_left_lane_border;
          refline_points[i].distance_to_left_obstacle =
              refline_points_source[i].distance_to_left_obstacle;
          refline_points[i].distance_to_left_road_border =
              refline_points_source[i].distance_to_left_road_border;
          refline_points[i].distance_to_right_lane_border =
              refline_points_source[i].distance_to_right_lane_border;
          refline_points[i].distance_to_right_obstacle =
              refline_points_source[i].distance_to_right_obstacle;
          refline_points[i].distance_to_right_road_border =
              refline_points_source[i].distance_to_right_road_border;
          refline_points[i].enu_point.x = refline_points_source[i].enu_point.x;
          refline_points[i].enu_point.y = refline_points_source[i].enu_point.y;
          refline_points[i].enu_point.z = 0.0;
          refline_points[i].lane_width = refline_points_source[i].lane_width;
          refline_points[i].left_road_border_type =
              refline_points_source[i].left_road_border_type;
          refline_points[i].max_velocity =
              refline_points_source[i].max_velocity;
          refline_points[i].on_route = refline_points_source[i].on_route;
          refline_points[i].right_road_border_type =
              refline_points_source[i].right_road_border_type;
          refline_points[i].track_id = refline_points_source[i].track_id;

          // cal car point by enu point
          Eigen::Vector3d car_point, enu_point;
          enu_point.x() = refline_points[i].enu_point.x;
          enu_point.y() = refline_points[i].enu_point.y;
          enu_point.z() = 0.0;
          car_point = enu2car * enu_point;
          refline_points[i].car_point.x = car_point.x();
          refline_points[i].car_point.y = car_point.y();
          refline_points[i].car_point.z = car_point.z();

          // use last valid lane border
          if (i > 0) {
            if (refline_points[i].distance_to_left_lane_border > 10 &&
                refline_points[i - 1].distance_to_left_lane_border < 10) {
              refline_points[i].distance_to_left_lane_border =
                  refline_points[i - 1].distance_to_left_lane_border;
            }
            if (refline_points[i].distance_to_right_lane_border > 10 &&
                refline_points[i - 1].distance_to_right_lane_border < 10) {
              refline_points[i].distance_to_right_lane_border =
                  refline_points[i - 1].distance_to_right_lane_border;
            }
          }
        }

        auto &refline_segments =
            lane_[index].reference_line.reference_line_segments;
        for (auto &segment : refline_segments) {
          for (size_t i = 0; i < refline_points.size(); i++) {
            if (i >= segment.begin_index && i <= segment.end_index) {
              refline_points[i].direction = segment.direction.value;
              refline_points[i].in_intersection = segment.is_in_intersection;
            }
          }
        }
      };

  fill_refline_points(current_lane_index(), current_refline_points_, enu2car);
  fill_refline_points(current_lane_index() - 1, left_refline_points_, enu2car);
  fill_refline_points(current_lane_index() + 1, right_refline_points_, enu2car);
  fill_refline_points(current_lane_index() - 2, left_left_refline_points_,
                      enu2car);
  fill_refline_points(current_lane_index() + 2, right_right_refline_points_,
                      enu2car);
  if (current_refline_points_.size() > 0) {
    world_model->set_current_last_car_point_x(
        current_refline_points_.back().car_point.x);

    Point2D last_enu_point;
    last_enu_point.x = current_refline_points_.back().enu_point.x;
    last_enu_point.y = current_refline_points_.back().enu_point.y;
    world_model->set_current_last_enu_point(last_enu_point);
  }

  world_model->set_ddmap_intersection(false);
  current_refline_polys_valid_ = false;
  auto refline_generator = world_model->get_refline_generator();
  if (world_model->is_reset() ||
      world_model->get_vehicle_status().gear.gear_data.gear_status.value ==
          GearType::REVERSE) {
    // refline_generator->clear_hist_refline();
    PlanningContext::Instance()
        ->mutable_pass_intersection_planner_input()
        ->hist_refline_.clear();
    MSD_LOG(INFO, "refline_generator cond, clear_hist_refline");
  }
  if (!world_model->get_vehicle_dbw_status()) {
    world_model->set_apf_lat_far_flag(false);
    MSD_LOG(INFO,
            "refline_generator cond judge quit cp: reset apf_lat_far_flag");
  }
  if (world_model->is_ddmap() && !world_model->is_acc_mode()) {
    extend_refline_points_for_ddmap(-1, left_refline_points_, world_model);
    extend_refline_points_for_ddmap(0, current_refline_points_, world_model);
    extend_refline_points_for_ddmap(1, right_refline_points_, world_model);

    polyfit_raw_vision_lane_for_ddmap(world_model);

    double start_time = MTIME()->timestamp().ms();
    world_model->set_refline_last_attract_type(
        refline_generator->attract_type());
    world_model->set_refline_last_condition(
        refline_generator->condition().reflinecondition);
    // const auto &pass_intersection_planner_input =
    // context_->pass_intersection_planner_input();
    PassIntersectionPlannerPreprocessor pass_intersection_planner_preprocessor;
    pass_intersection_planner_preprocessor.init(world_model);
    pass_intersection_planner_preprocessor.process();
    const auto &pass_intersection_planner_input =
        PlanningContext::Instance()->pass_intersection_planner_input();
    if (!refline_generator->update(pass_intersection_planner_input)) {
      MSD_LOG(WARN, "DEBUG_ERROR !refline_generator_update");
    }
    MSD_LOG(INFO, "REFLINE_GENERATOR: Condition is : %d",
            refline_generator->condition());
    CostTime cost_time =
        CostTime{"refline_generator", MTIME()->timestamp().ms() - start_time};
    auto *planner_debug = PlanningContext::Instance()->mutable_planner_debug();
    planner_debug->cost_time.emplace_back(cost_time);

    // planner_debug->apf_debug_info = {};
    planner_debug->apf_debug_info.apf_refline_points.clear();
    if (refline_generator->condition().reflinecondition ==
            pass_intersection_planner::ReflineCondition::
                FIRST_HALF_INTERSECTION ||
        refline_generator->condition().reflinecondition ==
            pass_intersection_planner::ReflineCondition::
                LAST_HALF_INTERSECTION ||
        refline_generator->condition().reflinecondition ==
            pass_intersection_planner::ReflineCondition::IN_INTERSECTION) {
      // refline_generator->replace_refline_points(0, current_refline_points_,
      //                                           world_model);
      world_model->replace_refline_points(
          refline_generator->refline_result(),
          refline_generator->get_cur_lane_left_border(),
          refline_generator->get_cur_lane_right_border(),
          current_refline_points_);
      for (const auto p : refline_generator->refline_result()) {
        std::vector<double> x_y_tmp;
        x_y_tmp.push_back(p.x);
        x_y_tmp.push_back(p.y);
        planner_debug->apf_debug_info.apf_refline_points.push_back(x_y_tmp);
      }
    }
    world_model->set_apf_lat_far_flag(
        refline_generator->get_apf_lat_far_flag());
    // save refline condition
    world_model->set_refline_attract_type(refline_generator->attract_type());
    world_model->set_refline_condition(
        refline_generator->condition().reflinecondition);

    planner_debug->apf_debug_info.pi_condition =
        (int)pass_intersection_planner_input.condition_.reflinecondition;
    planner_debug->apf_debug_info.pi_attract_type =
        (int)refline_generator->attract_type();
    planner_debug->apf_debug_info.pi_follow_car_id =
        refline_generator->get_follow_car_id();
    planner_debug->apf_debug_info.refline_origin_theta =
        refline_generator->get_refline_origin().theta;
    planner_debug->apf_debug_info.road_ref_theta =
        refline_generator->get_road_ref_theta();
    planner_debug->apf_debug_info.ego_pose_theta =
        pass_intersection_planner_input.ego_pose_.theta;
    planner_debug->apf_debug_info.apf_quit_cp =
        refline_generator->get_quit_cp();
    planner_debug->apf_debug_info.apf_quit_cp_reason =
        refline_generator->get_apf_quit_cp_reason();
    planner_debug->apf_debug_info.max_ego2refline_lat_dis =
        refline_generator->get_max_ego2refline_lat_dis();
    planner_debug->apf_debug_info.lat_dis_2_refline =
        refline_generator->get_lat_dis_2_refline();
    planner_debug->apf_debug_info.lat_dis_2_in_inter =
        refline_generator->get_lat_dis_2_in_inter();
    planner_debug->apf_debug_info.lanes_info =
        refline_generator->get_lanes_info();
    planner_debug->apf_debug_info.road_edge_info =
        refline_generator->get_road_edge_info();
    // planner_debug->apf_debug_info.pi_target_points.clear();
  } else {
    world_model->set_refline_attract_type(0);
    world_model->set_refline_condition(0);
    world_model->set_refline_last_attract_type(0);
    world_model->set_refline_last_condition(0);
    // refline_generator->clear_hist_refline();
    PlanningContext::Instance()
        ->mutable_pass_intersection_planner_input()
        ->hist_refline_.clear();
    auto *planner_debug = PlanningContext::Instance()->mutable_planner_debug();
    planner_debug->apf_debug_info = {};
    MSD_LOG(INFO, "refline_generator cond clear for not pilot mode!!");
  }
}

void MSDMapInfo::extend_refline_points_for_ddmap(
    int relative_id, std::vector<ReferenceLinePointDerived> &refline_points,
    const std::shared_ptr<WorldModel> &world_model) {
  const auto &ego_state =
      world_model->get_cart_ego_state_manager().get_cart_ego_state();
  const auto &car2enu = world_model->get_cart_ego_state_manager().get_car2enu();
  double ego_vel = ego_state.ego_vel;

  if (refline_points.size() < 5) {
    MSD_LOG(WARN,
            "extend_refline_points_for_ddmap, refline_points "
            "too short, do not extend, relative_id: %d,  size: %d",
            relative_id, refline_points.size());
    if (relative_id == 0) {
      world_model->set_ddmap_intersection(true);
    }
    return;
  }

  const auto first_point = refline_points[0];
  const auto mid_point = refline_points[refline_points.size() / 2];
  const auto last_point = refline_points[refline_points.size() - 1];

  double current_refline_length =
      std::hypot(first_point.car_point.x - last_point.car_point.x,
                 first_point.car_point.y - last_point.car_point.y);

  if (current_refline_length < 10.0) {
    MSD_LOG(WARN,
            "extend_refline_points_for_ddmap, refline_points "
            "too short, do not extend, relative_id: %d, length: %f",
            relative_id, current_refline_length);
    if (relative_id == 0) {
      world_model->set_ddmap_intersection(true);
    }
    // return;
  }

  if (std::abs(first_point.car_point.x - last_point.car_point.x) <
      std::abs(first_point.car_point.y - last_point.car_point.y)) {
    MSD_LOG(WARN,
            "extend_refline_points_for_ddmap, refline_points "
            "delta yaw too large, do not extend, relative_id: %d",
            relative_id);
    if (relative_id == 0) {
      world_model->set_ddmap_intersection(true);
    }
    return;
  }

  if (first_point.car_point.x - last_point.car_point.x > 0) {
    MSD_LOG(WARN,
            "extend_refline_points_for_ddmap, refline_points "
            "reversed, do not extend, relative_id: %d",
            relative_id);
    if (relative_id == 0) {
      world_model->set_ddmap_intersection(true);
    }
    return;
  }

  if (relative_id == 0 &&
      (last_point.car_point.x < std::min(10.0, ego_vel * 1.5))) {
    MSD_LOG(WARN, "extend_refline_points_for_ddmap, current lane too short, "
                  "limit acceleration");
    world_model->set_ddmap_intersection(true);
  }

  double max_length = std::max(std::max(80.0, ego_vel * 8.0),
                               std::min(150.0, current_refline_length * 5.0));

  // fit coeffs from car points
  double fit_start_time = MTIME()->timestamp().ms();
  static std::vector<double> poly_coeff{};
  poly_coeff.clear();
  static std::vector<Pose2D> traj{};
  traj.clear();

  for (const auto &point : refline_points) {
    Pose2D car_point;
    car_point.x = point.car_point.x;
    car_point.y = point.car_point.y;
    traj.push_back(car_point);
  }

  if (!Polyfit(traj, 2, poly_coeff) || poly_coeff.size() != 3) {
    MSD_LOG(WARN,
            "extend_refline_points_for_ddmap, polyfit error, relative_id: %d",
            relative_id);
    return;
  }

  std::reverse(poly_coeff.begin(), poly_coeff.end());

  double fit_end_time = MTIME()->timestamp().ms();
  MSD_LOG(
      INFO,
      "extend_refline_points_for_ddmap, relative_id: %d, polys fit time: %f ms",
      relative_id, fit_end_time - fit_start_time);
  MSD_LOG(INFO,
          "extend_refline_points_for_ddmap, relative_id: %d, polys fit coeffs: "
          "%f %f %f",
          relative_id, poly_coeff[0], poly_coeff[1], poly_coeff[2]);

  // recalculate origin car/enu points
  for (auto &point : refline_points) {
    point.car_point.y = calc_poly1d(poly_coeff, point.car_point.x);

    Eigen::Vector3d car_point, enu_point;
    car_point.x() = point.car_point.x;
    car_point.y() = point.car_point.y;
    car_point.z() = point.car_point.z;
    enu_point = car2enu * car_point;

    point.enu_point.x = enu_point.x();
    point.enu_point.y = enu_point.y();
    point.enu_point.z = enu_point.z();
  }

  double mid_point_offset =
      calc_poly1d(poly_coeff, mid_point.car_point.x) - mid_point.car_point.y;
  MSD_LOG(INFO,
          "extend_refline_points_for_ddmap, relative_id: %d, x: %f, "
          "mid_point_offset: %f",
          relative_id, last_point.car_point.x, mid_point_offset);

  double origin_poly_coeff_2 = poly_coeff[2];
  double last_point_offset =
      calc_poly1d(poly_coeff, last_point.car_point.x) - last_point.car_point.y;
  // poly_coeff[2] = origin_poly_coeff_2 - last_point_offset;
  MSD_LOG(INFO,
          "extend_refline_points_for_ddmap forward, relative_id: %d, x: %f, "
          "last_point_offset: %f",
          relative_id, last_point.car_point.x, last_point_offset);

  double linear_cos = 1 / std::sqrt(1 + std::pow(poly_coeff[1], 2));

  MSD_LOG(INFO,
          "extend_refline_points_for_ddmap forward, relative_id: %d, "
          "linear_cos: %f",
          relative_id, linear_cos);

  if (linear_cos < 0.5) {
    MSD_LOG(INFO,
            "extend_refline_points_for_ddmap forward, linear_cos too "
            "small, do not extend, relative_id: %d",
            relative_id);
    world_model->set_ddmap_intersection(true);
    return;
  }

  double step = 2.0 * linear_cos;
  double max_x = max_length * linear_cos;

  MSD_LOG(
      INFO,
      "extend_refline_points_for_ddmap, relative_id: %d, step: %f, max_x: %f",
      relative_id, step, max_x);

  // extend forward
  for (double x = last_point.car_point.x + step; x < max_x; x += step) {
    // cal car point by enu point
    Eigen::Vector3d car_point, enu_point;
    car_point.x() = x;
    car_point.y() = calc_poly1d(poly_coeff, x);
    car_point.z() = last_point.car_point.z;
    enu_point = car2enu * car_point;

    auto point = last_point;
    point.car_point.x = car_point.x();
    point.car_point.y = car_point.y();
    point.car_point.z = car_point.z();
    point.enu_point.x = enu_point.x();
    point.enu_point.y = enu_point.y();
    point.enu_point.z = enu_point.z();

    refline_points.push_back(point);
  }

  // extend backward
  double backward_max_length = std::max(50.0, ego_vel * 2.0);
  if (first_point.car_point.x > -backward_max_length) {
    double first_point_offset =
        calc_poly1d(poly_coeff, first_point.car_point.x) -
        first_point.car_point.y;
    // poly_coeff[2] = origin_poly_coeff_2 - first_point_offset;
    MSD_LOG(INFO,
            "extend_refline_points_for_ddmap backward, relative_id: %d, x: %f, "
            "first_point_offset: %f",
            relative_id, first_point.car_point.x, first_point_offset);

    std::reverse(refline_points.begin(), refline_points.end());

    for (double x = first_point.car_point.x - step;
         x > -(backward_max_length + 2.0); x -= step) {
      Eigen::Vector3d car_point, enu_point;
      car_point.x() = x;
      car_point.y() = calc_poly1d(poly_coeff, x);
      car_point.z() = first_point.car_point.z;
      enu_point = car2enu * car_point;

      auto point = first_point;
      point.car_point.x = car_point.x();
      point.car_point.y = car_point.y();
      point.car_point.z = car_point.z();
      point.enu_point.x = enu_point.x();
      point.enu_point.y = enu_point.y();
      point.enu_point.z = enu_point.z();

      refline_points.push_back(point);
    }

    std::reverse(refline_points.begin(), refline_points.end());
  }

  if (relative_id == 0) {
    current_refline_polys_valid_ = true;
    current_refline_polys_ = poly_coeff;
    current_refline_polys_[2] = 0.0;
    current_refline_polys_min_x_ = first_point.car_point.x;
    current_refline_polys_max_x_ = last_point.car_point.x;
  }
}

void MSDMapInfo::polyfit_raw_vision_lane_for_ddmap(
    const std::shared_ptr<WorldModel> &world_model) {
  auto &lane_perception =
      world_model->get_perception_vision_lane().lane_perception;

  const auto &ego_state =
      world_model->get_cart_ego_state_manager().get_cart_ego_state();
  const auto &car2enu = world_model->get_cart_ego_state_manager().get_car2enu();
  double ego_vel = ego_state.ego_vel;

  left_raw_lane_polys_valid_ = false;
  right_raw_lane_polys_valid_ = false;

  if (!lane_perception.available) {
    return;
  }

  double cloest_left_intercept = 20;
  double cloest_right_intercept = -20;
  int left_lane_index = -1;
  int right_lane_index = -1;
  int left_lane_track_id = -1;
  int right_lane_track_id = -1;

  for (int i = 0; i < lane_perception.lanes.size(); i++) {
    auto &lane = lane_perception.lanes[i];
    // fusion lane, find left and right lane by index
    if (!lane.is_failed_3d && lane.index != 0 && !lane.is_centerline &&
        lane.camera_source.value == CameraSourceEnum::CAMERA_SOURCE_FRONT_MID &&
        lane.points_3d_x.size() == lane.points_3d_y.size() &&
        lane.points_3d_x.size() > 3) {
      if (lane.index == -1) {
        left_lane_track_id = (int)lane.track_id;
      } else if (lane.index == 1) {
        right_lane_track_id = (int)lane.track_id;
      }
    }
  }

  for (int i = 0; i < lane_perception.lanes.size(); i++) {
    auto &lane = lane_perception.lanes[i];
    // raw vision lane, find left and right lane by track id
    if (lane.is_failed_3d && lane.index == 0 && !lane.is_centerline &&
        lane.points_3d_x.size() == lane.points_3d_y.size() &&
        lane.points_3d_x.size() > 3) {
      double intercept = lane.points_3d_y[0];
      if (left_lane_track_id != -1 &&
          left_lane_track_id == int(lane.track_id)) {
        left_lane_index = i;
        cloest_left_intercept = intercept;
      } else if (right_lane_track_id != -1 &&
                 right_lane_track_id == int(lane.track_id)) {
        right_lane_index = i;
        cloest_right_intercept = intercept;
      }
    }
  }

  MSD_LOG(INFO,
          "raw_lane_debug: left_lane_index: %d, right_lane_index: %d, "
          "raw_lane_debug: left_lane_track_id: %d, right_lane_track_id: %d, "
          "cloest_left_intercept: %f, cloest_right_intercept: %f",
          left_lane_index, right_lane_index, left_lane_track_id,
          right_lane_track_id, cloest_left_intercept, cloest_right_intercept);

  auto cal_traj_by_raw_lane = [this](const std::vector<float> &x,
                                     const std::vector<float> &y,
                                     std::vector<Pose2D> &traj) {
    const double min_line_point_gap = 1.0;
    std::vector<std::vector<size_t>> groups{};
    for (size_t i = 0; i < x.size(); i++) {
      if (groups.empty() ||
          (!groups.back().empty() &&
           std::abs(x[i] - x[groups.back().front()]) > min_line_point_gap)) {
        groups.push_back(std::vector<size_t>{});
      }
      groups.back().push_back(i);
    }

    for (const auto &group : groups) {
      if (group.empty()) {
        continue;
      } else {
        double avg_x = 0, avg_y = 0;
        double cnt = static_cast<double>(group.size());
        for (const size_t &i : group) {
          avg_x += x[i];
          avg_y += y[i];
        }
        avg_x = avg_x / cnt;
        avg_y = avg_y / cnt;

        Pose2D car_point;
        // from rear axle to center
        car_point.x =
            avg_x -
            (ConfigurationContext::Instance()->get_vehicle_param().length / 2 -
             ConfigurationContext::Instance()
                 ->get_vehicle_param()
                 .rear_bumper_to_rear_axle);
        car_point.y = avg_y;
        traj.push_back(car_point);
        MSD_LOG(INFO, "TEST_DEBUG: x: %f, y: %f", car_point.x, car_point.y);
      }
    }

    // drop last three points
    if (traj.size() > 2) {
      traj.pop_back();
      traj.pop_back();
      traj.pop_back();
    }
  };

  if (left_lane_index != -1) {
    auto &x = lane_perception.lanes[left_lane_index].points_3d_x;
    auto &y = lane_perception.lanes[left_lane_index].points_3d_y;
    if ((x.back() - x.front()) > 15.0 && x.front() < 10.0 &&
        std::abs(x.back() - x.front()) > std::abs(y.back() - y.front())) {
      static std::vector<Pose2D> traj{};
      traj.clear();
      MSD_LOG(INFO, "TEST_DEBUG: left:");
      cal_traj_by_raw_lane(x, y, traj);

      if (x.back() > std::max(4.0 * ego_vel, 50.0)) {
        // cubic function
        if (Polyfit(traj, 3, left_raw_lane_polys_) &&
            left_raw_lane_polys_.size() == 4) {
          std::reverse(left_raw_lane_polys_.begin(),
                       left_raw_lane_polys_.end());
          left_raw_lane_polys_valid_ = true;
          // avoid edging curvture
          left_raw_lane_polys_min_x_ = traj.front().x + 5;
          left_raw_lane_polys_max_x_ = traj.back().x - 5;

          MSD_LOG(INFO,
                  "raw_lane_debug: left_raw_lane_polys_: %f %f %f %f,  min x: "
                  "%f, max x: %f",
                  left_raw_lane_polys_[0], left_raw_lane_polys_[1],
                  left_raw_lane_polys_[2], left_raw_lane_polys_[3],
                  left_raw_lane_polys_min_x_, left_raw_lane_polys_max_x_);
        }
      } else if (x.back() > std::max(2.0 * ego_vel, 25.0)) {
        // quadratic function
        if (Polyfit(traj, 2, left_raw_lane_polys_) &&
            left_raw_lane_polys_.size() == 3) {
          std::reverse(left_raw_lane_polys_.begin(),
                       left_raw_lane_polys_.end());
          left_raw_lane_polys_valid_ = true;
          // avoid edging curvture
          left_raw_lane_polys_min_x_ = traj.front().x + 5;
          left_raw_lane_polys_max_x_ = traj.back().x - 5;

          MSD_LOG(INFO,
                  "raw_lane_debug: left_raw_lane_polys_: %f %f %f,  min x: "
                  "%f, max x: %f",
                  left_raw_lane_polys_[0], left_raw_lane_polys_[1],
                  left_raw_lane_polys_[2], left_raw_lane_polys_min_x_,
                  left_raw_lane_polys_max_x_);
        }
      }
    }
  }

  if (right_lane_index != -1) {
    auto &x = lane_perception.lanes[right_lane_index].points_3d_x;
    auto &y = lane_perception.lanes[right_lane_index].points_3d_y;
    if ((x.back() - x.front()) > 15.0 && x.front() < 10.0 &&
        std::abs(x.back() - x.front()) > std::abs(y.back() - y.front())) {
      static std::vector<Pose2D> traj{};
      traj.clear();
      MSD_LOG(INFO, "TEST_DEBUG: right:");
      cal_traj_by_raw_lane(x, y, traj);

      if (x.back() > std::max(4.0 * ego_vel, 50.0)) {
        // cubic function
        if (Polyfit(traj, 3, right_raw_lane_polys_) &&
            right_raw_lane_polys_.size() == 4) {
          std::reverse(right_raw_lane_polys_.begin(),
                       right_raw_lane_polys_.end());
          right_raw_lane_polys_valid_ = true;
          // avoid edging curvture
          right_raw_lane_polys_min_x_ = traj.front().x + 5;
          right_raw_lane_polys_max_x_ = traj.back().x - 5;

          MSD_LOG(INFO,
                  "raw_lane_debug: right_raw_lane_polys_: %f %f %f %f,  min x: "
                  "%f, max x: %f",
                  right_raw_lane_polys_[0], right_raw_lane_polys_[1],
                  right_raw_lane_polys_[2], right_raw_lane_polys_[3],
                  right_raw_lane_polys_min_x_, right_raw_lane_polys_max_x_);
        }
      } else if (x.back() > std::max(2.0 * ego_vel, 25.0)) {
        // quadratic function
        if (Polyfit(traj, 2, right_raw_lane_polys_) &&
            right_raw_lane_polys_.size() == 3) {
          std::reverse(right_raw_lane_polys_.begin(),
                       right_raw_lane_polys_.end());
          right_raw_lane_polys_valid_ = true;
          // avoid edging curvture
          right_raw_lane_polys_min_x_ = traj.front().x + 5;
          right_raw_lane_polys_max_x_ = traj.back().x - 5;

          MSD_LOG(INFO,
                  "raw_lane_debug: right_raw_lane_polys_: %f %f %f,  min x: "
                  "%f, max x: %f",
                  right_raw_lane_polys_[0], right_raw_lane_polys_[1],
                  right_raw_lane_polys_[2], right_raw_lane_polys_min_x_,
                  right_raw_lane_polys_max_x_);
        }
      }
    }
  }
}

void MSDMapInfo::calculate_reflines_by_c_poly(
    const Lane &c_lane, const Lane &l_lane, const Lane &r_lane,
    const std::shared_ptr<WorldModel> &world_model) {
  current_refline_points_.clear();
  left_refline_points_.clear();
  right_refline_points_.clear();
}

void MSDMapInfo::update_vision_lanes() {
  if (self_position_.in_map_area) {
    return;
  }

  std::vector<LaneData> lanes_data_split;
  lanes_data_split.clear();
  double min_width = 2.9, default_width = 3.8;

  auto &lanes_data = lane_;
  for (size_t i = 0; i < lanes_data.size(); i++) {
    if (lanes_data[i].left_lane_boundary.existence == true &&
        lanes_data[i].right_lane_boundary.existence == true) {
      auto temp_width = calc_lane_width(
          lanes_data[i].left_lane_boundary.polyline.poly_coefficient,
          lanes_data[i].right_lane_boundary.polyline.poly_coefficient);

      if (temp_width > 2 * min_width && temp_width < 3 * default_width) {
        // insert one more lane
        std::vector<double> poly_coefficient;
        for (size_t j = 0;
             j <
             lanes_data[i].left_lane_boundary.polyline.poly_coefficient.size();
             j++) {
          auto coeff =
              (lanes_data[i].left_lane_boundary.polyline.poly_coefficient[j] +
               lanes_data[i].right_lane_boundary.polyline.poly_coefficient[j]) /
              2;
          poly_coefficient.push_back(coeff);
        }
        lanes_data_split.push_back(lanes_data[i]);
        lanes_data_split.back().right_lane_boundary.existence = false;
        lanes_data_split.back().right_lane_boundary.polyline.poly_coefficient =
            poly_coefficient;

        lanes_data_split.push_back(lanes_data[i]);
        lanes_data_split.back().left_lane_boundary.existence = false;
        lanes_data_split.back().left_lane_boundary.polyline.poly_coefficient =
            poly_coefficient;

        MSD_INF_TOPIC_LIST(
            "/update_vision_lanes", "origin_left_boundary",
            lanes_data[i].left_lane_boundary.polyline.poly_coefficient);
        MSD_INF_TOPIC_LIST(
            "/update_vision_lanes", "origin_right_boundary",
            lanes_data[i].right_lane_boundary.polyline.poly_coefficient);
        MSD_INF_TOPIC_LIST("/update_vision_lanes", "insert_boundary",
                           poly_coefficient);
      } else if (temp_width > 3 * default_width) {
        // insert two more lanes
        std::vector<double> poly_coefficient_1, poly_coefficient_2;
        for (size_t j = 0;
             j <
             lanes_data[i].left_lane_boundary.polyline.poly_coefficient.size();
             j++) {
          auto coeff_1 =
              (lanes_data[i].left_lane_boundary.polyline.poly_coefficient[j] /
                   3 +
               lanes_data[i].right_lane_boundary.polyline.poly_coefficient[j] *
                   2 / 3);
          poly_coefficient_1.push_back(coeff_1);

          auto coeff_2 =
              (lanes_data[i].left_lane_boundary.polyline.poly_coefficient[j] *
                   2 / 3 +
               lanes_data[i].right_lane_boundary.polyline.poly_coefficient[j] /
                   3);
          poly_coefficient_2.push_back(coeff_2);
        }
        lanes_data_split.push_back(lanes_data[i]);
        lanes_data_split.back().right_lane_boundary.existence = false;
        lanes_data_split.back().right_lane_boundary.polyline.poly_coefficient =
            poly_coefficient_1;

        lanes_data_split.push_back(lanes_data[i]);
        lanes_data_split.back().left_lane_boundary.existence = false;
        lanes_data_split.back().left_lane_boundary.polyline.poly_coefficient =
            poly_coefficient_1;
        lanes_data_split.back().right_lane_boundary.existence = false;
        lanes_data_split.back().right_lane_boundary.polyline.poly_coefficient =
            poly_coefficient_2;

        lanes_data_split.push_back(lanes_data[i]);
        lanes_data_split.back().left_lane_boundary.existence = false;
        lanes_data_split.back().left_lane_boundary.polyline.poly_coefficient =
            poly_coefficient_2;

        MSD_INF_TOPIC_LIST(
            "/update_vision_lanes", "origin_left_boundary",
            lanes_data[i].left_lane_boundary.polyline.poly_coefficient);
        MSD_INF_TOPIC_LIST(
            "/update_vision_lanes", "origin_right_boundary",
            lanes_data[i].right_lane_boundary.polyline.poly_coefficient);
        MSD_INF_TOPIC_LIST("/update_vision_lanes", "insert_boundary_1",
                           poly_coefficient_1);
        MSD_INF_TOPIC_LIST("/update_vision_lanes", "insert_boundary_2",
                           poly_coefficient_2);
      } else {
        lanes_data_split.push_back(lanes_data[i]);
      }

    } else {
      lanes_data_split.push_back(lanes_data[i]);
    }
  }

  // update relative_id
  if (lanes_data_split.size() != lanes_data.size()) {
    int current_lane_index = -1;
    for (size_t i = 0; i < lanes_data_split.size(); i++) {
      if ((i == 0) &&
          (lanes_data_split[i].left_lane_boundary.existence == false) &&
          (lanes_data_split[i]
               .right_lane_boundary.polyline.poly_coefficient.size() == 4) &&
          (lanes_data_split[i]
               .right_lane_boundary.polyline.poly_coefficient[0] <= 0.0)) {
        current_lane_index = i;
        break;
      } else if ((i == ((int)lanes_data_split.size() - 1)) &&
                 (lanes_data_split[i].right_lane_boundary.existence == false) &&
                 (lanes_data_split[i]
                      .left_lane_boundary.polyline.poly_coefficient.size() ==
                  4) &&
                 (lanes_data_split[i]
                      .left_lane_boundary.polyline.poly_coefficient[0] >=
                  0.0)) {
        current_lane_index = i;
        break;
      } else if ((lanes_data_split[i]
                      .left_lane_boundary.polyline.poly_coefficient.size() ==
                  4) &&
                 (lanes_data_split[i]
                      .left_lane_boundary.polyline.poly_coefficient[0] >=
                  0.0) &&
                 (lanes_data_split[i]
                      .right_lane_boundary.polyline.poly_coefficient.size() ==
                  4) &&
                 (lanes_data_split[i]
                      .right_lane_boundary.polyline.poly_coefficient[0] <=
                  0.0)) {
        current_lane_index = i;
        break;
      }
    }
    if (current_lane_index < 0) {
      MSD_LOG(WARN, "invalid lane index of lanes_data_split, use original "
                    "relative id");
      current_lane_index = -lanes_data[0].relative_id;
    }
    for (size_t i = 0; i < lanes_data_split.size(); i++) {
      lanes_data_split[i].relative_id = i - current_lane_index;
    }

    lanes_data = lanes_data_split;
  }
}

void MSDMapInfo::update_lane_boundary_polyline() {
  lane_boundary_polyline_.clear();

  auto &lanes_data = lane_;
  for (size_t i = 0; i < lanes_data.size(); i++) {
    if (i == 0) {
      if ((lanes_data[i].left_lane_boundary.existence == true) &&
          (lanes_data[i].left_lane_boundary.polyline.poly_coefficient.size() >
           0)) {

        lane_boundary_polyline_.push_back(
            lanes_data[i].left_lane_boundary.polyline);
      }

      if ((lanes_data[i].right_lane_boundary.existence == true) &&
          (lanes_data[i].right_lane_boundary.polyline.poly_coefficient.size() >
           0)) {
        lane_boundary_polyline_.push_back(
            lanes_data[i].right_lane_boundary.polyline);
      }
    } else {
      auto &last_boundary = lanes_data[i - 1].right_lane_boundary;
      auto &curr_boundary = lanes_data[i].left_lane_boundary;

      if ((curr_boundary.existence == true) &&
          (curr_boundary.polyline.poly_coefficient.size() > 0) &&
          (last_boundary.existence == false ||
           last_boundary.polyline.poly_coefficient.size() == 0 ||
           std::fabs(curr_boundary.polyline.poly_coefficient[0] -
                     last_boundary.polyline.poly_coefficient[0]) > 1e-8)) {
        lane_boundary_polyline_.push_back(curr_boundary.polyline);
      }

      if ((lanes_data[i].right_lane_boundary.existence == true) &&
          (lanes_data[i].right_lane_boundary.polyline.poly_coefficient.size() >
           0)) {
        lane_boundary_polyline_.push_back(
            lanes_data[i].right_lane_boundary.polyline);
      }
    }
  }

  size_t index = 0;
  for (index = 0; index < lane_boundary_polyline_.size(); index++) {
    if (lane_boundary_polyline_[index].poly_coefficient[0] < 0) {
      break;
    }
  }

  for (size_t i = 0; i < lane_boundary_polyline_.size(); i++) {
    if (i < index) {
      lane_boundary_polyline_[i].relative_id = i - index;
    } else {
      lane_boundary_polyline_[i].relative_id = i - index + 1;
    }
  }
}

void find_lane_rids(int lane_status, int lane_id,
                    const std::vector<int> &lane_line_bitlist,
                    const std::vector<int> &rids,
                    std::array<int, 2> &lane_rids) {

  int left_rid = kInvalidRelativeId;
  int right_rid = kInvalidRelativeId;
  if (lane_status == LaneStatusEx::BOTH_AVAILABLE ||
      lane_status == LaneStatusEx::LEFT_AVAILABLE) {
    if ((size_t)(lane_id + 1) <= lane_line_bitlist.size()) {
      int left_rid_idx =
          std::accumulate(lane_line_bitlist.begin(),
                          lane_line_bitlist.begin() + lane_id + 1, 0);
      if (left_rid_idx - 1 >= 0 && (size_t)(left_rid_idx - 1) < rids.size()) {
        left_rid = rids[left_rid_idx - 1];
      }
    }
  }
  if (lane_status == LaneStatusEx::BOTH_AVAILABLE ||
      lane_status == LaneStatusEx::RIGHT_AVAILABLE) {
    if ((size_t)(lane_id + 2) <= lane_line_bitlist.size()) {
      int right_rid_idx =
          std::accumulate(lane_line_bitlist.begin(),
                          lane_line_bitlist.begin() + lane_id + 2, 0);
      if (right_rid_idx - 1 >= 0 && (size_t)(right_rid_idx - 1) < rids.size()) {
        right_rid = rids[right_rid_idx - 1];
      }
    }
  }
  lane_rids[0] = left_rid;
  lane_rids[1] = right_rid;
}

MapInfoManager::MapInfoManager() {
  std::array<RawRefLine *, 2> neighbours{&l_raw_refline_, &r_raw_refline_};
  c_raw_refline_.set_neighbours(neighbours);

  neighbours[0] = nullptr;
  neighbours[1] = &c_raw_refline_;
  l_raw_refline_.set_neighbours(neighbours);

  neighbours[0] = &c_raw_refline_;
  neighbours[1] = nullptr;
  r_raw_refline_.set_neighbours(neighbours);

  clane_.set_raw_refline(&c_raw_refline_);
  llane_.set_raw_refline(&l_raw_refline_);
  rlane_.set_raw_refline(&r_raw_refline_);
  lllane_.set_raw_refline(&ll_raw_refline_);
  rrlane_.set_raw_refline(&rr_raw_refline_);

  std::array<Lane *, 2> lane_neighbours{&llane_, &rlane_};
  clane_.set_neighbours(lane_neighbours);

  lane_neighbours[0] = &clane_;
  lane_neighbours[1] = &rrlane_;
  rlane_.set_neighbours(lane_neighbours);

  lane_neighbours[0] = &rlane_;
  lane_neighbours[1] = nullptr;
  rrlane_.set_neighbours(lane_neighbours);

  lane_neighbours[0] = &lllane_;
  lane_neighbours[1] = &clane_;
  llane_.set_neighbours(lane_neighbours);

  lane_neighbours[0] = nullptr;
  lane_neighbours[1] = &llane_;
  lllane_.set_neighbours(lane_neighbours);
}

void MapInfoManager::set_map_info(
    const MSDMapInfo &map_info, const std::shared_ptr<WorldModel> &world_model,
    double ego_vel, double ego_v_cruise, bool is_ddmap) {
  map_info_ = map_info;
  map_info_.update(world_model, ego_vel, ego_v_cruise, is_ddmap);
}

bool MapInfoManager::update(const std::shared_ptr<WorldModel> &world_model) {
  map_info_.hack_lc_end_dis(
      world_model->get_cart_ego_state_manager().get_cart_ego_state());

  auto *planner_debug = PlanningContext::Instance()->mutable_planner_debug();
  double start_time = MTIME()->timestamp().ms();
  ref_trajectory_.clear();
  // do not construct ref trajectories when acc mode
  if (!world_model->is_acc_mode()) {
    if (!construct_ref_trajectories(world_model)) {
      MSD_LOG(WARN, "ERROR_CJ91 !construct_ref_trajectories");
      return false;
    }
  }
  double end_time = MTIME()->timestamp().ms();
  MSD_LOG(ERROR, "time_cost, construct_ref_trajectories time: %f",
          end_time - start_time);
  CostTime task_cost =
      CostTime{"construct_ref_trajectories", end_time - start_time};
  planner_debug->cost_time.emplace_back(task_cost);

  clane_.update_worldmodel(world_model);
  llane_.update_worldmodel(world_model);
  rlane_.update_worldmodel(world_model);
  rrlane_.update_worldmodel(world_model);
  lllane_.update_worldmodel(world_model);

  traffic_light_stop_point_ =
      world_model->get_traffic_light_decision()->get_stop_point();
  traffic_light_stop_flag_ =
      world_model->get_traffic_light_decision()->get_stop_flag();
  traffic_light_stop_reason_ =
      world_model->get_traffic_light_decision()->get_RED_LIGHT_STOP_CLASS();
  traffic_light_faster_pass_ =
      world_model->get_traffic_light_decision()->get_is_GREENBLINKING_go_();

  // lateral bp process
  mmp_update_ = world_model->is_acc_mode() ? true : map_info_.is_in_map_area();
  if (world_model->is_acc_mode()) {
    return true;
  }
  if (!update_lanes(world_model->is_ddmap())) {
    MSD_LOG(ERROR, "update_lanes failed!");
    MSD_LOG(WARN, "ERROR_CJ92 !update_lanes");
    return false;
  }

  if (!update_raw_reflines(world_model)) {
    MSD_LOG(ERROR, "update_raw_reflines failed!");
    MSD_LOG(WARN, "ERROR_CJ93 !update_raw_reflines");
    return false;
  }

  clane_.update(map_info_.lane_boundary_polyline());
  rlane_.update(map_info_.lane_boundary_polyline());
  llane_.update(map_info_.lane_boundary_polyline());
  lllane_.update(map_info_.lane_boundary_polyline());
  rrlane_.update(map_info_.lane_boundary_polyline());
  MSD_LOG(INFO, "debug right dash line %f  left %f stopline %f",
          map_info_.get_distance_to_dash_line("right"),
          map_info_.get_distance_to_dash_line("left"),
          map_info_.get_distance_to_stop_line());

  return true;
}

bool MapInfoManager::construct_ref_trajectories(
    const std::shared_ptr<WorldModel> &world_model) {
  static int solver_count_ = 1; // smoother solver count
  // todo: change key to lane id
  ref_trajectories_[-1] = map_info_.left_refline_points();
  ref_trajectories_[1] = map_info_.right_refline_points();
  ref_trajectories_[0] = map_info_.current_refline_points();
  ref_trajectories_bias_[-1] = map_info_.left_refline_points();
  ref_trajectories_bias_[1] = map_info_.right_refline_points();
  ref_trajectories_bias_[0] = map_info_.current_refline_points();

  MSD_LOG(INFO, "input reference lines size: %d %d %d",
          int(ref_trajectories_[-1].size()), int(ref_trajectories_[1].size()),
          int(ref_trajectories_[0].size()));

  auto target_lane_id = PlanningContext::Instance()
                            ->prev_planning_status()
                            .lane_status.target_lane_id;
  int common_point_num_current = 0;
  int common_point_num_left = 0;
  int common_point_num_right = 0;
  for (const auto &p : ref_trajectories_[0]) {
    if (last_point_ids_[0].find(p.track_id) != last_point_ids_[0].end()) {
      common_point_num_current++;
    }
    if (last_point_ids_[1].find(p.track_id) != last_point_ids_[1].end()) {
      common_point_num_right++;
    }
    if (last_point_ids_[-1].find(p.track_id) != last_point_ids_[-1].end()) {
      common_point_num_left++;
    }
  }
  int max_num_p =
      std::max(std::max(common_point_num_current, common_point_num_left),
               common_point_num_right);
  if (common_point_num_current == max_num_p) {
  } else if (common_point_num_left == max_num_p) {
    if (ref_trajectories_smooth_.find(1) != ref_trajectories_smooth_.end()) {
      ref_trajectories_smooth_[1].clear();
    }
    if (ref_trajectories_smooth_.find(0) != ref_trajectories_smooth_.end()) {
      ref_trajectories_smooth_[1] = ref_trajectories_smooth_[0];
      ref_trajectories_smooth_[0].clear();
    }
    if (ref_trajectories_smooth_.find(-1) != ref_trajectories_smooth_.end()) {
      ref_trajectories_smooth_[0] = ref_trajectories_smooth_[-1];
      ref_trajectories_smooth_[-1].clear();
    }
  } else if (common_point_num_right == max_num_p) {
    if (ref_trajectories_smooth_.find(-1) != ref_trajectories_smooth_.end()) {
      ref_trajectories_smooth_[-1].clear();
    }
    if (ref_trajectories_smooth_.find(0) != ref_trajectories_smooth_.end()) {
      ref_trajectories_smooth_[-1] = ref_trajectories_smooth_[0];
      ref_trajectories_smooth_[0].clear();
    }
    if (ref_trajectories_smooth_.find(1) != ref_trajectories_smooth_.end()) {
      ref_trajectories_smooth_[0] = ref_trajectories_smooth_[1];
      ref_trajectories_smooth_[1].clear();
    }
  }

  static std::unordered_map<int, double> trigger_curv;
  static std::vector<ReferenceLinePointDerived> deviant_point;
  static std::vector<std::tuple<int, double>> kappa_list;
  trigger_curv.clear();
  theta_deviant_point_.clear();
  cur_frame_car_pos_.clear();
  last_point_ids_.clear();
  for (auto &ref_line : ref_trajectories_) {
    for (const auto &p : ref_line.second) {
      last_point_ids_[ref_line.first].insert(p.track_id);
    }

    if (ref_line.second.size() >= 2) {
      auto iter = ref_line.second.begin() + 1;
      while (iter != ref_line.second.end()) {
        double dis =
            sqrt(std::pow(iter->enu_point.x - (iter - 1)->enu_point.x, 2) +
                 std::pow(iter->enu_point.y - (iter - 1)->enu_point.y, 2));
        if (dis < 0.01) {
          MSD_LOG(INFO,
                  "refline filter enu_point1:(%.3f, %.3f) enu_point2:(%.3f, "
                  "%.3f)",
                  iter->enu_point.x, iter->enu_point.y, (iter - 1)->enu_point.x,
                  (iter - 1)->enu_point.y);
          MSD_LOG(INFO,
                  "refline filter car_point1:(%.3f, %.3f) car_point2:(%.3f, "
                  "%.3f)",
                  iter->car_point.x, iter->car_point.y, (iter - 1)->car_point.x,
                  (iter - 1)->car_point.y);
          iter = ref_line.second.erase(iter);
        } else {
          ++iter;
        }
      }
    }

    if (ref_line.second.size() < 2) {
      MSD_LOG(INFO, "reference line [%d] empty!", ref_line.first);
      ref_line.second.clear();
      ref_trajectories_smooth_[ref_line.first].clear();
      continue;
    }

    linear_interpolation(ref_line.second);
    set_range_of_trajectory(ref_line.second);

    if (ref_line.second.size() < 5) {
      MSD_LOG(INFO, "reference line [%d] truncate empty!", ref_line.first);
      ref_line.second.clear();
      continue;
    }

    const auto &vehicle_status = world_model->get_vehicle_status();
    deviant_point.clear();
    kappa_list.clear();
    double smooth_curv = 0;
    double last_kappa = 0;
    maf_perception_interface::Point3d last_point = ref_line.second[0].enu_point;
    for (int i = 1; i < (int)ref_line.second.size() - 1; i++) {
      if (ref_line.second[i].car_point.x < -5 ||
          ref_line.second[i].car_point.x > 180)
        continue;
      // cal kappa:
      double x[] = {((ref_line.second)[i - 1]).enu_point.x,
                    ((ref_line.second)[i]).enu_point.x,
                    ((ref_line.second)[i + 1]).enu_point.x};
      double y[] = {((ref_line.second)[i - 1]).enu_point.y,
                    ((ref_line.second)[i]).enu_point.y,
                    ((ref_line.second)[i + 1]).enu_point.y};
      double a =
          std::sqrt(std::pow((x[0] - x[1]), 2) + std::pow((y[0] - y[1]), 2));
      double b =
          std::sqrt(std::pow((x[0] - x[2]), 2) + std::pow((y[0] - y[2]), 2));
      double c =
          std::sqrt(std::pow((x[1] - x[2]), 2) + std::pow((y[1] - y[2]), 2));
      double r = (a * b * c) / std::sqrt((a + b - c) * (a - b + c) *
                                         (b + c - a) * (a + b + c));
      double kappa = 1 / ((isnan(r) || isinf(r)) ? 10000.0 : r);
      // S(P1,P2,P3)=|y1 y2 y3|= (x1-x3)*(y2-y3)-(y1-y3)*(x2-x3)
      if ((x[0] - x[2]) * (y[1] - y[2]) - (y[0] - y[2]) * (x[1] - x[2]) < 0) {
        kappa = -kappa;
      }
      kappa_list.emplace_back( // parasoft-suppress AUTOSAR-M6_5_3
          i, kappa);           // parasoft-suppress AUTOSAR-M6_5_3 "f-drop"
    }

    double dis_Accu = 0;
    for (auto i = 0; i < kappa_list.size(); i++) {
      int index_p = std::get<0>(kappa_list[i]);
      double kappa_average = 0;
      for (int j = -5; j <= 5; j++) {
        if (i + j >= 0 && i + j < kappa_list.size())
          kappa_average += std::get<1>(kappa_list[i + j]);
        else if (i + j < 0)
          kappa_average += std::get<1>(kappa_list.front());
        else
          kappa_average += std::get<1>(kappa_list.back());
      }
      kappa_average /= 11.0;

      double delta_s = std::sqrt(
          std::pow((ref_line.second[index_p].enu_point.x - last_point.x), 2) +
          std::pow((ref_line.second[index_p].enu_point.y - last_point.y), 2));
      double dkappa =
          std::abs(std::get<1>(kappa_list[i]) - last_kappa) / delta_s;
      double kappa_average_abs = std::fabs(kappa_average);
      double dkappa_abs = std::fabs(dkappa); // 0.04 --> 0.038 sil
      if ((kappa_average_abs > 0.01 && dkappa_abs > 0.038) ||
          (kappa_average_abs <= 0.01 && dkappa_abs > 0.02) ||
          (kappa_average_abs <= 0.007 && dkappa_abs > 0.015) ||
          (kappa_average_abs <= 0.006 && dkappa_abs > 0.011) ||
          (kappa_average_abs <= 0.004 && dkappa_abs > 0.009) ||
          (kappa_average_abs <= 0.003 && dkappa_abs > 0.007) ||
          (kappa_average_abs <= 0.002 && kappa_average_abs > 0.001 &&
           dkappa_abs > 0.0008 + (kappa_average_abs - 0.001) * 1.7) ||
          (kappa_average_abs <= 0.001 && dkappa_abs > 0.0008) ||
          std::pow(vehicle_status.velocity.heading_velocity.value_mps, 2) *
                  std::get<1>(kappa_list[i]) >
              16.0) {
        ReferenceLinePointDerived theta_deviant_tmp;
        theta_deviant_tmp.enu_point.x = ref_line.second[index_p].enu_point.x;
        theta_deviant_tmp.enu_point.y = ref_line.second[index_p].enu_point.y;
        theta_deviant_tmp.car_point.x = ref_line.second[index_p].car_point.x;
        theta_deviant_tmp.car_point.y = ref_line.second[index_p].car_point.y;
        deviant_point.push_back(theta_deviant_tmp);
        if (smooth_curv < kappa_average_abs) {
          smooth_curv = kappa_average_abs;
        }
        MSD_LOG(INFO,
                "MAP_INFO lane_id : %d, smooth trigger x : %.2f, y : %.2f, "
                "kappa : %.4f, dk : %.4f, k_average : %.4f, id : %d",
                ref_line.first, ref_line.second[index_p].car_point.x,
                ref_line.second[index_p].car_point.y,
                std::get<1>(kappa_list[i]), dkappa, kappa_average,
                ref_line.first);
      }
      if (!deviant_point.empty() && i > 0) {
        dis_Accu +=
            std::sqrt(std::pow(ref_line.second[index_p].car_point.x -
                                   ref_line.second[index_p - 1].car_point.x,
                               2) +
                      std::pow(ref_line.second[index_p].car_point.y -
                                   ref_line.second[index_p - 1].car_point.y,
                               2));
        if (dis_Accu > 25)
          break;
      }

      last_point = ref_line.second[index_p].enu_point;
      last_kappa = std::get<1>(kappa_list[i]);
    }
    theta_deviant_point_[ref_line.first] = deviant_point;
    trigger_curv[ref_line.first] = smooth_curv;

    for (int i = 0; i < (int)ref_line.second.size() - 1; i++) {
      if (((ref_line.second)[i + 1]).car_point.x > 0 &&
          ((ref_line.second)[i]).car_point.x < 0) {
        cur_frame_car_pos_[ref_line.first] = ref_line.second[i];
        break;
      }
    }

    // double begin_x = ref_line.second.back().car_point.x;
    // double begin_y = ref_line.second.back().car_point.y;
    // if (ref_line.second.back().car_point.x < 90 &&
    //     ref_line.second.back().car_point.y < 3 && abs(begin_x) > 90 &&
    //     abs(begin_y) > 3) {
    //   // we need to insure that there are 100m in the trajectory in
    //   front. If
    //   // the coming trajectory has 100m but after cut, it is not 100m,
    //   then
    //   // report error.
    //   MSD_LOG(ERROR, "there is something wrong with the trajectory %d
    //   cut!",
    //           ref_line.first);
    // }
  }
  static std::vector<int> smooth_lane_list;
  smooth_lane_list.clear();
  if (target_lane_id == 0) {
    smooth_lane_list.push_back(0);
    smooth_lane_list.push_back(1);
    smooth_lane_list.push_back(-1);
  } else {
    smooth_lane_list.push_back(target_lane_id);
    smooth_lane_list.push_back(0);
    smooth_lane_list.push_back(-target_lane_id);
  }
  if (world_model->is_ddmap()) {
    ref_trajectories_smooth_.clear();
    MSD_LOG(INFO, "[refsmooth] is_ddmap then smooth segment clear !!!");
  }
  MSD_LOG(INFO, "[refsmooth] is_ddmap[%d]", world_model->is_ddmap());

  for (auto lane_id : smooth_lane_list) {
    if (!ref_trajectories_smooth_[lane_id].empty()) {
      if (splice_smooth_seg(lane_id, ref_trajectories_smooth_[lane_id],
                            ref_trajectories_[lane_id])) {
        is_refline_smooth_flag_ = true;
        MSD_LOG(INFO,
                "[refsmooth] splice success(using hist smooth seg)!!!  lane_id "
                "= %d",
                lane_id);
      }
    }
  }

  static std::vector<ReferenceLinePointDerived> segment_line;
  static std::vector<ReferenceLinePointDerived> ref_trajectory_tmp;
  static std::vector<double> smoother_vx_m, smoother_vy_m;
  for (auto lane_id : smooth_lane_list) {
    if (world_model->get_refline_condition().reflinecondition ==
            pass_intersection_planner::ReflineCondition::NO_INTERSECTION ||
        world_model->get_refline_condition().reflinecondition ==
            pass_intersection_planner::ReflineCondition::UNKNOW ||
        ConfigurationContext::Instance()
            ->planner_config()
            .common_config.disable_refline_smoother) {
      MSD_LOG(INFO, "[refsmooth] scenario is not intersection then disenable "
                    "refline smoother !");
      ref_trajectories_smooth_.clear();
      break;
    }
    if (ref_trajectories_[lane_id].size() < 2)
      continue;

    if (ref_trajectories_smooth_[lane_id].empty()) {
      bool Flags_smooth = false;
      segment_line.clear();
      Judge_smooth_flags(lane_id, ref_trajectories_[lane_id], Flags_smooth,
                         segment_line);
      MSD_LOG(INFO, "[refsmooth] lane_id[%d]  segment_line.empty[%d]", lane_id,
              segment_line.empty());
      if (Flags_smooth && !segment_line.empty()) {
        ref_trajectory_tmp.clear();
        smoother_vx_m.clear();
        smoother_vy_m.clear();
        // refline_provider_ =
        // std::make_unique<msquare::ReferenceLineProvider>();
        if (nullptr == refline_provider_) {
          refline_provider_ =
              std::make_unique<msquare::ReferenceLineProvider>();
        } else {
          refline_provider_->clear();
        }
        double smooth_bound = clip(trigger_curv[lane_id] * 10.0, 0.1, 0.01);
        refline_provider_->SetLatBound(smooth_bound);
        refline_provider_->SetTrajectoryPoints(segment_line);
        (void)refline_provider_->GetReferenceLine(true);
        refline_provider_->GetSmoothTrajectory(smoother_vx_m, smoother_vy_m);
        refline_provider_->GetSolveStatus(solver_status_);
        ref_trajectory_tmp.resize(smoother_vx_m.size());
        for (int i = 0; i < ref_trajectory_tmp.size(); i++) {
          (ref_trajectory_tmp[i]).enu_point.x = smoother_vx_m[i];
          (ref_trajectory_tmp[i]).enu_point.y = smoother_vy_m[i];
        }
        if (solver_status_ != 1 && solver_status_ != 2)
          solver_count_++;
        if ((solver_status_ == 1 || solver_status_ == 2) &&
            solver_count_ <= 2) {
          MSD_LOG(INFO,
                  "[refsmooth] Smoother Solve Successfully! Solver Count : %d, "
                  "lane id : %d",
                  solver_count_, lane_id);
          solver_count_ = 1;
          ref_trajectories_smooth_[lane_id].assign(
              ref_trajectory_tmp.begin(),
              ref_trajectory_tmp.end()); // save smooth result
        } else if (solver_count_ > 2) {
          MSD_LOG(INFO,
                  "[refsmooth] Smoother Solve Failed! Solver Count : %d, "
                  "lane_id : %d",
                  solver_count_, lane_id);
          solver_count_ = 1;
          ref_trajectories_smooth_[lane_id].assign(segment_line.begin(),
                                                   segment_line.end());
        }
      }
      if (!ref_trajectories_smooth_[lane_id].empty()) {
        if (splice_smooth_seg(lane_id, ref_trajectories_smooth_[lane_id],
                              ref_trajectories_[lane_id])) {
          is_refline_smooth_flag_ = true;
          MSD_LOG(INFO, "[refsmooth] splice success!!!   lane_id = %d",
                  lane_id);
          break;
        }
      }
    }
  }

  // filter refline_points for same points:
  for (auto &ref_line : ref_trajectories_) {
    if (ref_line.second.size() >= 2) {
      auto iter = ref_line.second.begin() + 1;
      while (iter != ref_line.second.end()) {
        double dis =
            sqrt(std::pow(iter->enu_point.x - (iter - 1)->enu_point.x, 2) +
                 std::pow(iter->enu_point.y - (iter - 1)->enu_point.y, 2));
        if (dis < 0.01) {
          MSD_LOG(INFO,
                  "refline filter enu_point1:(%.3f, %.3f) enu_point2:(%.3f, "
                  "%.3f)",
                  iter->enu_point.x, iter->enu_point.y, (iter - 1)->enu_point.x,
                  (iter - 1)->enu_point.y);
          MSD_LOG(INFO,
                  "refline filter car_point1:(%.3f, %.3f) car_point2:(%.3f, "
                  "%.3f)",
                  iter->car_point.x, iter->car_point.y, (iter - 1)->car_point.x,
                  (iter - 1)->car_point.y);
          iter = ref_line.second.erase(iter);
        } else {
          ++iter;
        }
      }
    }
  }

  return true;
}

void MapInfoManager::Judge_smooth_flags(
    const int lane_id, const std::vector<ReferenceLinePointDerived> &ref_line,
    bool &Flags_smooth, std::vector<ReferenceLinePointDerived> &segment_line) {
  static std::vector<int> deviant_index;
  deviant_index.clear();
  int cur_front_index = -1, cur_back_index = -1;
  bool car_front = false, car_back = false;

  MSD_LOG(INFO, "[refsmooth] lane_id[%d] deviant_points.size[%d]",
          theta_deviant_point_[lane_id].size());
  for (int i = 0; i < theta_deviant_point_[lane_id].size(); i++) {
    for (int j = 0; j < (int)ref_line.size() - 1; j++) {
      if (std::abs(ref_line[j].enu_point.x -
                   theta_deviant_point_[lane_id][i].enu_point.x) < 0.01 &&
          std::abs(ref_line[j].enu_point.y -
                   theta_deviant_point_[lane_id][i].enu_point.y) < 0.01) {
        deviant_index.emplace_back( // parasoft-suppress AUTOSAR-M6_5_3
            j);                     // parasoft-suppress AUTOSAR-M6_5_3 "f-drop"
        break;
      }
    }
  }
  if (!deviant_index.empty()) {
    for (int j = (int)deviant_index.size() - 1; j >= 0; j--) {
      double dis_front = 0.;
      for (int i = deviant_index[j]; i < (int)ref_line.size() - 1; i++) {
        double dis_between_points = std::sqrt(
            std::pow(ref_line[i].enu_point.x - ref_line[i + 1].enu_point.x, 2) +
            std::pow(ref_line[i].enu_point.y - ref_line[i + 1].enu_point.y, 2));
        dis_front += dis_between_points;
        if (dis_front > 50 || i == ref_line.size() - 2) { // 60 -> 40 -> 35
          car_front = true;
          cur_front_index = i;
          MSD_LOG(INFO,
                  "[zyl debug]lane_id = %d  cur_front_index: %d  dis_front "
                  "= %f",
                  lane_id, cur_front_index, dis_front);
          break;
        }
      }
      if (car_front)
        break;
    }

    double dis_rear = 0.;
    for (int i = deviant_index.front(); i > 0; i--) {
      double dis_between_points = std::sqrt(
          std::pow(ref_line[i].enu_point.x - ref_line[i - 1].enu_point.x, 2) +
          std::pow(ref_line[i].enu_point.y - ref_line[i - 1].enu_point.y, 2));
      dis_rear += dis_between_points;
      if (dis_rear > 50 || i == 1) {
        car_back = true;
        cur_back_index = i - 1;
        MSD_LOG(INFO,
                "[zyl debug]lane_id = %d  cur_back_index: %d  dis_rear = %f",
                lane_id, cur_back_index, dis_rear);
        ReferenceLinePointDerived rear_p;
        rear_p.enu_point = ref_line[cur_back_index].enu_point;
        start_smooth_point_[lane_id] = rear_p;
        break;
      }
    }
  }

  if (car_back && car_front) {
    segment_line.clear();
    segment_line.assign(ref_line.begin() + cur_back_index,
                        ref_line.begin() + cur_front_index +
                            1); // segment line need to smooth
    Flags_smooth = true;
    return;
  }

  Flags_smooth = false;
  return;
}

// smooth_segment_line 不为空，说明还没有驶过不平滑路段
bool MapInfoManager::splice_smooth_seg(
    int lane_id, std::vector<ReferenceLinePointDerived> &smooth_segment_line,
    std::vector<ReferenceLinePointDerived> &cur_trajectories) {
  int insert_front_index = -1, insert_back_index = -1,
      smooth_segment_line_begin_index = -1;
  bool has_passed =
      true; // has_passed过程应该放到Judge_smooth_flags里面更合理，因为放到splice_smooth_seg里刚驶过异常点的这一帧是没有拼接过程的
  double s = 0;
  for (int i = (int)smooth_segment_line.size() - 1; i > 0; i--) {
    double dis_between_points =
        std::sqrt(std::pow(((smooth_segment_line[i]).enu_point.x -
                            (smooth_segment_line[i - 1]).enu_point.x),
                           2) +
                  std::pow(((smooth_segment_line[i]).enu_point.y -
                            (smooth_segment_line[i - 1]).enu_point.y),
                           2));
    s += dis_between_points;
    if (std::abs(((smooth_segment_line[i]).enu_point.x -
                  cur_frame_car_pos_[lane_id].enu_point.x)) < 1.0 &&
        std::abs(((smooth_segment_line[i]).enu_point.y -
                  cur_frame_car_pos_[lane_id].enu_point.y)) < 2.0) {
      MSD_LOG(INFO,
              "[refsmooth]lane_id = %d car has passed smooth seg!  "
              "remain_s = %f",
              lane_id, s);
      break;
    }
    if (s > 20) {
      has_passed = false; // 自车没有驶过曲率异常点，置false
      MSD_LOG(INFO,
              "[refsmooth]lane_id = %d car has not passed smooth seg!  "
              "remain_s = %f",
              lane_id, s);
      break;
    }
  }

  if (std::abs(smooth_segment_line.back().enu_point.x -
               cur_trajectories.back().enu_point.x) < 1.0 &&
      std::abs(smooth_segment_line.back().enu_point.y -
               cur_trajectories.back().enu_point.y) < 1.0) {
    has_passed = false;
    MSD_LOG(INFO, "[refsmooth]lane_id = %d car is nearing goal");
  }

  if (has_passed) {
    smooth_segment_line.clear();
    MSD_LOG(INFO,
            "[refsmooth] smooth_segment clear!!![has passed] lane_id : %d",
            lane_id);
    return false;
  }

  for (int i = (int)cur_trajectories.size() - 1; i >= 0; i--) {
    if (std::abs(((cur_trajectories[i]).enu_point.x -
                  smooth_segment_line.back().enu_point.x)) < 0.5 &&
        std::abs(((cur_trajectories[i]).enu_point.y -
                  smooth_segment_line.back().enu_point.y)) < 0.5) {
      insert_front_index = i; // front insert pos
      MSD_LOG(INFO, "[zyl debug]lane_id = %d insert_front_index : %d", lane_id,
              insert_front_index);
      break;
    }
  }

  if (insert_front_index == -1) { // 自车已驶过异常点，无法完成匹配，清空
                                  // ref_trajectories_smooth_
                                  // 进行下一次截取和平滑
    smooth_segment_line.clear();
    MSD_LOG(INFO,
            "[refsmooth] smooth_segment clear!!!![no find front index] lane_id "
            ": %d",
            lane_id);
    return false;
  }

  for (int i = 0; i < (int)cur_trajectories.size() - 1; i++) {
    if (std::abs(((cur_trajectories[i]).enu_point.x -
                  smooth_segment_line.front().enu_point.x)) < 0.5 &&
        std::abs(((cur_trajectories[i]).enu_point.y -
                  smooth_segment_line.front().enu_point.y)) < 0.5) {
      insert_back_index = i;
      smooth_segment_line_begin_index = 0;
      MSD_LOG(INFO, "[zyl debug]lane_id = %d insert_back_index : %d", lane_id,
              insert_back_index);
      break;
    }
  }

  if (insert_back_index == -1 ||
      insert_back_index >=
          std::min(insert_front_index + 1, int(cur_trajectories.size()))) {
    for (int i = 0; i < (int)smooth_segment_line.size() - 1; i++) {
      if (std::abs(((smooth_segment_line[i]).enu_point.x -
                    cur_trajectories.front().enu_point.x)) < 1.0 &&
          std::abs(((smooth_segment_line[i]).enu_point.y -
                    cur_trajectories.front().enu_point.y)) < 1.0) {
        insert_back_index = 0;
        smooth_segment_line_begin_index = i;
        MSD_LOG(INFO,
                "[zyl debug]lane_id = %d insert_back_index: %d, "
                "smooth_segment_line_begin_index: %d",
                lane_id, insert_back_index, smooth_segment_line_begin_index);
        break;
      }
    }
  }

  // for refline smooth debug:
  // ofstream outfile1("/home/ros/Downloads/output1.txt",ios::app);
  // ofstream outfile2("/home/ros/Downloads/output2.txt",ios::app);
  // if (insert_back_index >= std::min(insert_front_index + 1,
  // int(cur_trajectories.size()))) { // 异常情况触发
  //   MSD_LOG(INFO, "zyl debug ==== back_index=%d  front_index = %d  size =
  //   %d
  //   ",
  //                  insert_back_index, insert_front_index,
  //                  cur_trajectories.size());
  //   for (int i = 0; i < cur_trajectories.size(); i++) {
  //     outfile1 <<  (cur_trajectories[i]).enu_point.x << " " <<
  //     (cur_trajectories[i]).enu_point.y << std::endl;
  //   }
  //   outfile1 << "------------------" << std::endl;
  //   for (int i = 0; i < smooth_segment_line.size(); i++) {
  //     outfile2 << (smooth_segment_line[i]).enu_point.x << " " <<
  //     (smooth_segment_line[i]).enu_point.y <<  std::endl;
  //   }
  //   outfile2 << "------------------" << std::endl;
  //   outfile1.close();
  //   outfile2.close();
  //   return false;
  // }

  // splice
  if (insert_back_index != -1) {
    cur_trajectories.erase(
        cur_trajectories.begin() + insert_back_index,
        cur_trajectories.begin() +
            std::min(insert_front_index + 1,
                     int(cur_trajectories.size()))); // delete old data
    MSD_LOG(INFO, "[zyl debug]after erase cur_trajectories.size() : %d",
            cur_trajectories.size());
    cur_trajectories.insert(cur_trajectories.begin() + insert_back_index,
                            smooth_segment_line.begin() +
                                smooth_segment_line_begin_index,
                            smooth_segment_line.end());
    MSD_LOG(INFO, "[zyl debug]insert length : %d",
            int(smooth_segment_line.size()) - smooth_segment_line_begin_index);
    MSD_LOG(INFO, "[zyl debug]after insert cur_trajectories.size() : %d",
            cur_trajectories.size());
    return true;
  }

  return false;
}

void MapInfoManager::set_range_of_trajectory(
    std::vector<ReferenceLinePointDerived> &ref_trajectory) {
  static const double DIST_FROM_BEHIND = 30; // Horizon lower bound
  static const double DIST_TO_FRONT = 200;   // Horizon upper bound

  if (ref_trajectory.empty()) {
    MSD_LOG(WARN,
            "Set Range of the trajectory: Incoming trajectory is empty!!!");
    return;
  }

  int start = 0, end = (int)ref_trajectory.size() - 1;
  while (start <= end) {
    double x = ref_trajectory[start].car_point.x;
    double y = ref_trajectory[start].car_point.y;
    double dist_to_origin = std::sqrt(std::pow(x, 2.0) + std::pow(y, 2.0));

    if (dist_to_origin <= DIST_FROM_BEHIND) {
      break;
    }
    start++;
  }

  while (start <= end) {
    double x = ref_trajectory[end].car_point.x;
    double y = ref_trajectory[end].car_point.y;

    double dist_to_origin = std::sqrt(std::pow(x, 2.0) + std::pow(y, 2.0));

    if (dist_to_origin <= DIST_TO_FRONT) {
      break;
    }
    end--;
  }
  if (start > 0) {
    for (int i = 0, j = start; j <= end; i++, j++) {
      ref_trajectory[i] = ref_trajectory[j];
    }
  }
  ref_trajectory.resize(end - start + 1);
}

void MapInfoManager::frame_trans_from_car2enu(
    std::vector<ReferenceLinePointDerived> &ref_trajectory,
    const Transform &car2enu) {
  if (ref_trajectory.empty()) {
    MSD_LOG(WARN,
            "Trajectory frame transformation: Incoming trajectory is empty!!!");
    return;
  }

  Eigen::Vector3d car_point, enu_point;
  for (auto &p : ref_trajectory) {
    car_point.x() = p.car_point.x;
    car_point.y() = p.car_point.y;
    car_point.z() = 0.0;
    enu_point = car2enu * car_point;
    p.car_point.x = enu_point.x();
    p.car_point.y = enu_point.y();
  }
}

void MapInfoManager::linear_interpolation(
    std::vector<ReferenceLinePointDerived> &ref_trajectory) {
  static const double INTERVAL = 3; // m
  int insert_point_count = 0;

  if (ref_trajectory.empty()) {
    MSD_LOG(WARN, "Linear Interpolation: Incoming trajectory is empty!!!");
    return;
  }

  for (int i = 0; i < (int)ref_trajectory.size() - 1;
       i += insert_point_count + 1) {
    double last_point_x = ref_trajectory[i + 1].car_point.x;
    double last_point_y = ref_trajectory[i + 1].car_point.y;
    double first_point_x = ref_trajectory[i].car_point.x;
    double first_point_y = ref_trajectory[i].car_point.y;
    double last_point_x_enu = ref_trajectory[i + 1].enu_point.x;
    double last_point_y_enu = ref_trajectory[i + 1].enu_point.y;
    double first_point_x_enu = ref_trajectory[i].enu_point.x;
    double first_point_y_enu = ref_trajectory[i].enu_point.y;

    double y_dist = last_point_y_enu - first_point_y_enu;
    double x_dist = last_point_x_enu - first_point_x_enu;
    double dist = std::sqrt(std::pow(x_dist, 2.0) + std::pow(y_dist, 2.0));

    // the interval is 0.3m
    if (dist <= INTERVAL) {
      insert_point_count = 0;
      continue;
    }

    ReferenceLinePointDerived point_to_insert;
    // double current_x = ref_trajectory[i].car_point.x;
    // double current_y = ref_trajectory[i].car_point.y;
    double current_x_enu = ref_trajectory[i].enu_point.x;
    double current_y_enu = ref_trajectory[i].enu_point.y;
    double dist_from_current_to_end = dist;
    auto it = ref_trajectory.begin() + i + 1;

    insert_point_count = 0;
    int interval_num = 1;

    while (dist_from_current_to_end > INTERVAL + 1.e-2) {
      if (interval_num * INTERVAL > dist - 1.e-2) {
        break;
      }
      point_to_insert.car_point.x = planning_math::lerp(
          first_point_x, 0, last_point_x, dist, interval_num * INTERVAL);
      point_to_insert.car_point.y = planning_math::lerp(
          first_point_y, 0, last_point_y, dist, interval_num * INTERVAL);
      point_to_insert.enu_point.x =
          planning_math::lerp(first_point_x_enu, 0, last_point_x_enu, dist,
                              interval_num * INTERVAL);
      point_to_insert.enu_point.y =
          planning_math::lerp(first_point_y_enu, 0, last_point_y_enu, dist,
                              interval_num * INTERVAL);

      current_x_enu = point_to_insert.enu_point.x;
      current_y_enu = point_to_insert.enu_point.y;

      double y_dist_to_last = last_point_y_enu - current_y_enu;
      double x_dist_to_last = last_point_x_enu - current_x_enu;
      dist_from_current_to_end = std::sqrt(std::pow(x_dist_to_last, 2.0) +
                                           std::pow(y_dist_to_last, 2.0));

      ref_trajectory.insert(it, point_to_insert);
      ++interval_num;
      ++insert_point_count;
      it = ref_trajectory.begin() + i + 1 +
           insert_point_count; // points to the last point of the interval
    }
  }
}

void MapInfoManager::get_lane_change_point(
    const std::shared_ptr<WorldModel> &world_model) {
  double distance_to_point = std::numeric_limits<double>::max();
  double dis_lc_ratio = 1.0;
  double dis_lc_buffer = 0.0;
  if (map_info_.first_task_ranges().size() > 0) {
    distance_to_point =
        dis_lc_buffer + dis_lc_ratio * map_info_.first_task_ranges().back().end;
    if (distance_to_point < std::numeric_limits<double>::max()) {
      MSD_LOG(INFO, "LC_DEBUG distance to lane change point : %f",
              distance_to_point);
    }
  }

  distance_to_lane_change_point_ = distance_to_point;
}

// add environment
bool MapInfoManager::update_raw_reflines(
    const std::shared_ptr<WorldModel> &world_model) {
  double ego_vel =
      world_model->get_cart_ego_state_manager().get_cart_ego_state().ego_vel;
  if (mmp_update_ == true) {
    if (map_info_.current_refline_points().size() < 2) {
      MSD_LOG(ERROR, "[Enviroment::update_raw_reflines] Current refline points "
                     "number < 2");
      MSD_LOG(WARN, "ERROR_CJ931 current_refline_points().size() < 2");
      return false;
    }

    c_raw_refline_.update(ref_trajectories_bias_[0], ego_vel);

    if (llane_.exist() && map_info_.left_refline_points().size() == 0) {
      MSD_LOG(INFO,
              "[Enviroment::update_raw_reflines] curr_lane_id[%d] "
              "lane_num[%d]",
              map_info_.current_lane_index(), map_info_.lanes_num());
      MSD_LOG(ERROR, "[Enviroment::update_raw_reflines] No refline points "
                     "in left lane");
      return false;
    }

    l_raw_refline_.update(ref_trajectories_bias_[-1], ego_vel);

    if (rlane_.exist() && map_info_.right_refline_points().size() == 0) {
      MSD_LOG(INFO,
              "[Enviroment::update_raw_reflines] curr_lane_id[%d] "
              "lane_num[%d]",
              map_info_.current_lane_index(), map_info_.lanes_num());
      MSD_LOG(ERROR, "[Enviroment::update_raw_reflines] No refline points in "
                     "right lane");
      return false;
    }

    r_raw_refline_.update(ref_trajectories_bias_[1], ego_vel);

    if (lllane_.exist() && map_info_.left_left_refline_points().size() == 0 &&
        !world_model->is_ddmap()) {
      MSD_LOG(INFO,
              "[Enviroment::update_raw_reflines] curr_lane_id[%d] "
              "curr_lane_num[%d]",
              map_info_.current_lane_index(), map_info_.lanes_num());
      MSD_LOG(ERROR,
              "[Enviroment::update_raw_reflines] No refline points in ll lane");
      // return false;
    }

    ll_raw_refline_.update(map_info_.left_left_refline_points(), ego_vel);
    if (rrlane_.exist() && map_info_.right_right_refline_points().size() == 0 &&
        !world_model->is_ddmap()) {
      MSD_LOG(INFO,
              "[Enviroment::update_raw_reflines] rr_lane_id[%d] "
              "curr_lane_num[%d]",
              map_info_.current_lane_index(), map_info_.lanes_num());
      MSD_LOG(ERROR,
              "[Enviroment::update_raw_reflines] No refline points in rr lane");
      // return false;
    }

    rr_raw_refline_.update(map_info_.right_right_refline_points(), ego_vel);
  }

  return true;
}

bool MapInfoManager::update_lanes(bool is_ddmap) {
  if (mmp_update_ == true) {
    auto &lanes_data = map_info_.lane_;

    auto lane_source = LaneSource::MAP_SOURCE;
    if (is_ddmap) {
      lane_source = LaneSource::FUSION_SOURCE;
    }

    int lane_id = 0;
    if (map_info_.lanes_num() > 0) {
      lane_id = map_info_.current_lane_index();
    }

    std::array<int, 2> lane_rids;
    lane_rids.fill(kInvalidRelativeId);

    if (map_info_.lanes_num() > 0 ||
        map_info_.current_refline_points().size() > 0) {
      clane_.set_exist(true);
      clane_.set_type(LaneType::NORMAL);
      clane_.set_source(lane_source);
    } else {
      clane_.reset();
    }

    lane_rids.fill(kInvalidRelativeId);

    lane_id = map_info_.current_lane_index() - 1;
    if (lane_id >= 0 && lane_id < map_info_.lanes_num()) {
      llane_.set_exist(true);
      llane_.set_type(LaneType::NORMAL);
      llane_.set_source(lane_source);
    } else {
      llane_.reset();
    }

    lane_rids.fill(kInvalidRelativeId);

    lane_id = map_info_.current_lane_index() + 1;
    if (lane_id >= 0 && lane_id < map_info_.lanes_num()) {
      rlane_.set_exist(true);
      rlane_.set_type(LaneType::NORMAL);
      rlane_.set_source(lane_source);
    } else {
      rlane_.reset();
    }

    lane_rids.fill(kInvalidRelativeId);

    lane_id = map_info_.current_lane_index() - 2;
    if (lane_id >= 0 && lane_id < map_info_.lanes_num()) {
      lllane_.set_exist(true);
      lllane_.set_type(LaneType::NORMAL);
      lllane_.set_source(lane_source);
    } else {
      lllane_.reset();
    }

    lane_rids.fill(kInvalidRelativeId);

    lane_id = map_info_.current_lane_index() + 2;
    if (lane_id >= 0 && lane_id < map_info_.lanes_num()) {
      rrlane_.set_exist(true);
      rrlane_.set_type(LaneType::NORMAL);
      rrlane_.set_source(lane_source);
    } else {
      rrlane_.reset();
    }
  }
  return true;
}

bool MapInfoManager::has_lane(int index) const {
  bool result = false;
  switch (index) {
  case LanePosition::CURR_POS:
    result = clane_.exist();
    break;
  case LanePosition::RIGHT_POS:
    result = rlane_.exist();
    break;
  case LanePosition::RIGHT_RIGHT_POS:
    result = rrlane_.exist();
    break;
  case LanePosition::LEFT_POS:
    result = llane_.exist();
    break;
  case LanePosition::LEFT_LEFT_POS:
    result = lllane_.exist();
    break;
  default:
    MSD_LOG(ERROR, "[MapInfoManager::has_lane] Illegal lane index[%d]!", index);
    break;
  }

  return result;
}

bool MapInfoManager::is_solid_line(int side) const {
  if (!(side == 0 || side == 1)) {
    return false;
  }
  if (side == 0) {
    if (map_info_.left_boundary_info().size() > 0 &&
        map_info_.left_boundary_info()[0].type.value.value ==
            LaneBoundaryForm::SOLID) {
      return true;
    }
  } else if (side == 1) {
    if (map_info_.right_boundary_info().size() > 0 &&
        map_info_.right_boundary_info()[0].type.value.value ==
            LaneBoundaryForm::SOLID) {
      return true;
    }
  }
  return false;
}

} // namespace msquare
