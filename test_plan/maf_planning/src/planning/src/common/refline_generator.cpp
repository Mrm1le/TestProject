#include "common/refline_generator.h"
#include "common/math/quintic_poly_2d.h"
#include "common/utils/polyfit.h"
#include "planner/motion_planner/apf_planner.h"
#include <string>

using namespace std;

namespace msquare {

void ReflineGenerator::init(ReflineGeneratorCore core_type) {
  core_type_ = core_type;
  replan_threshold_ = 1.0;
  close_ref_point_threshold_ = 2.0;
  last_condition_.reflinecondition =
      pass_intersection_planner::ReflineCondition::UNKNOW;
  condition_.reflinecondition =
      pass_intersection_planner::ReflineCondition::UNKNOW;
  muti_historical_info_.clear();
  adc_historical_info_.clear();

  attract_coord_parameters_.zero_speed_threshold = 0.1;
  attract_coord_parameters_.coord_transform_precision = 0.01;
  attract_coord_parameters_.step_s = 0.3;
  attract_coord_parameters_.coarse_step_s = 1.0;
  attract_coord_parameters_.optimization_gamma = 0.5;
  attract_coord_parameters_.max_iter = 15;
}

void ReflineGenerator::default_init() {
  if (ReflineGeneratorConfig::Instance()->core_type == 0) {
    core_type_ = ReflineGeneratorCore::Apf;
  } else if (ReflineGeneratorConfig::Instance()->core_type == 1) {
    core_type_ = ReflineGeneratorCore::HybridAstar;
  } else {
    core_type_ = ReflineGeneratorCore::NoCoreType;
  }
  replan_threshold_ = 1.0;
  close_ref_point_threshold_ = 2.0;
  last_condition_.reflinecondition =
      pass_intersection_planner::ReflineCondition::UNKNOW;
  condition_.reflinecondition =
      pass_intersection_planner::ReflineCondition::UNKNOW;
  muti_historical_info_.clear();
  adc_historical_info_.clear();

  attract_coord_parameters_.zero_speed_threshold = 0.1;
  attract_coord_parameters_.coord_transform_precision = 0.01;
  attract_coord_parameters_.step_s = 0.3;
  attract_coord_parameters_.coarse_step_s = 1.0;
  attract_coord_parameters_.optimization_gamma = 0.5;
  attract_coord_parameters_.max_iter = 15;
}

void ReflineGenerator::get_default_apf_config(ApfPlannerConfigPtr planner_cfg) {
  auto cfg = ReflineGeneratorConfig::Instance();
  double shrink_ratio = cfg->shrink_ratio;
  planner_cfg->step_size_ = cfg->step_size;
  planner_cfg->min_turning_radius_ = cfg->min_turning_radius;
  planner_cfg->max_dtheta_ =
      planner_cfg->step_size_ / planner_cfg->min_turning_radius_;
  planner_cfg->vehicle_length_ = cfg->vehicle_length * shrink_ratio;
  planner_cfg->center_to_geometry_center_ =
      cfg->center_to_geometry_center * shrink_ratio;
  planner_cfg->vehicle_width_ = cfg->vehicle_width * shrink_ratio;
  planner_cfg->stop_gradient_value_ = cfg->stop_gradient_value;
}

void ReflineGenerator::get_apf_config(ApfPlannerConfigPtr planner_cfg,
                                      double shrink_ratio) {
  // auto cfg = ReflineGeneratorConfig::Instance();
  planner_cfg->step_size_ = input_.pi_params_.step_size;
  planner_cfg->min_turning_radius_ = input_.pi_params_.min_turning_radius;
  planner_cfg->max_dtheta_ =
      planner_cfg->step_size_ / planner_cfg->min_turning_radius_;
  planner_cfg->vehicle_length_ =
      input_.pi_params_.vehicle_length * shrink_ratio;
  planner_cfg->center_to_geometry_center_ =
      input_.pi_params_.center_to_geometry_center * shrink_ratio;
  planner_cfg->vehicle_width_ = input_.pi_params_.vehicle_width * shrink_ratio;
  planner_cfg->stop_gradient_value_ = input_.pi_params_.stop_gradient_value;
}

bool ReflineGenerator::update(
    const PassIntersectionPlannerInput &pass_intersection_planner_input,
    PiNotebookDebug *pi_nb_debug) {
  // std::cout << " >>>>>>>> refline generator update <<<<<<<< " << std::endl;
  input_ = pass_intersection_planner_input;
  attract_coord_parameters_.zero_speed_threshold =
      input_.pi_params_.zero_speed_threshold;
  attract_coord_parameters_.coord_transform_precision =
      input_.pi_params_.coord_transform_precision;
  attract_coord_parameters_.step_s = input_.pi_params_.step_s;
  attract_coord_parameters_.coarse_step_s = input_.pi_params_.coarse_step_s;
  attract_coord_parameters_.optimization_gamma =
      input_.pi_params_.optimization_gamma;
  attract_coord_parameters_.max_iter = input_.pi_params_.max_iter;
  if (input_.pi_params_.core_type == 0) {
    core_type_ = ReflineGeneratorCore::Apf;
  } else if (input_.pi_params_.core_type == 1) {
    core_type_ = ReflineGeneratorCore::HybridAstar;
  } else {
    core_type_ = ReflineGeneratorCore::NoCoreType;
  }

  refline_result_.clear();
  bool is_replan = false;
  last_condition_ = input_.last_condition_;
  last_attract_type_ = attract_type_;
  condition_ = input_.condition_;
  last_refline_result_.clear();
  for (const auto p : input_.last_refline_result_) {
    last_refline_result_.emplace_back(Pose2D(p.x, p.y, p.theta));
  }

  if (!calc_lane_perception_obstacle()) {
    return false;
  }

  double shrink_ratio = input_.pi_params_.shrink_ratio;
  if (condition_.reflinecondition ==
      pass_intersection_planner::ReflineCondition::NO_INTERSECTION) {
    // last_refline_result_.clear();
    selected_hist_vehicle_id_ = -1;
    attract_type_ = ApfAttractType::Unknow;
    target_pt_.clear();
    target_pt_count_ = 0;
    is_target_pt_flag_ = false;
    apf_lat_far_flag_ = false;
    follow_car_id_ = -1;
    road_ref_theta_ = 0;
    refline_origin_ = {};
    quit_cp_ = 0;
    lat_dis_2_in_inter_ = 0;
    max_ego2refline_lat_dis_ = 0;
    lat_dis_2_refline_ = 0;
    apf_quit_cp_reason_ = "";
    return true;
  }

  follow_car_id_ = -1;
  road_ref_theta_ = 0;
  lat_dis_in_intersection_ = 0;
  lat_dis_out_intersection_ = 0;
  quit_cp_ = 0;
  refline_target_pt_.clear();

  if (last_condition_.reflinecondition ==
          pass_intersection_planner::ReflineCondition::NO_INTERSECTION ||
      last_condition_.reflinecondition ==
          pass_intersection_planner::ReflineCondition::UNKNOW) {
    is_replan == true;
    last_refline_result_.clear();
  }
  if (is_replan || last_refline_result_.empty()) {
    shrink_ratio = 0.6;
    is_replan = true;
  }

  if (!calc_refline_origin(is_replan)) {
    return false;
  }
  // apf core
  if (core_type_ == ReflineGeneratorCore::Apf) {
    // calc direction
    ApfAttractType attract_type{ApfAttractType::Unknow};
    std::shared_ptr<FrenetCoordinateSystem> attract_frenet;
    double attract_theta;

    // insert new algorithm
    if (condition_.reflinecondition ==
            pass_intersection_planner::ReflineCondition::
                LAST_HALF_INTERSECTION &&
        input_.refline_points_.size() > 2 && input_.pi_params_.use_quintic) {
      double start_x = input_.ego_pose_.x;
      double start_y = input_.ego_pose_.y;
      double start_yaw = input_.ego_pose_.theta;
      double end_x = input_.refline_points_.front().x;
      double end_y = input_.refline_points_.front().y;
      double judge_theta = planning_math::NormalizeAngle(
          atan2(end_y - start_y, end_x - start_x) - start_yaw);
      // else skip to apf
      if (abs(judge_theta) < M_PI / 2) {
        if (spline_fit_refline(pi_nb_debug)) {
          judge_quit_CP();
          return true;
        }
      }
    }

    (void)calc_apf_attract_field(attract_type, attract_frenet, attract_theta);
    attract_type_ = attract_type;

    judge_quit_CP();

    ApfPlannerConfigPtr planner_cfg_ = std::make_shared<ApfPlannerConfig>();

    ObstaclePotentialParam line_obstalce_param(
        input_.pi_params_.line_repulsion_weight,
        input_.pi_params_.line_repulsion_max_distance,
        input_.pi_params_.line_repulsion_2_weight,
        input_.pi_params_.line_repulsion_2_attenuation);
    get_apf_config(planner_cfg_, shrink_ratio);

    // set planner
    double apf_max_length =
        std::max(std::min(input_.ego_pose_.vel * 6.0, 150.0),
                 input_.pi_params_.max_length);
    double apf_min_length = input_.pi_params_.min_length;

    std::unique_ptr<ApfPlanner> planner_ =
        std::make_unique<ApfPlanner>(planner_cfg_);
    planner_->set_origin(refline_origin_);
    planner_->set_traj_length(apf_max_length, apf_min_length);

    // set element
    std::vector<ApfElementPtr> element_ptr_vector;
    if (!(attract_type == ApfAttractType::Traj && attract_frenet != nullptr)) {
      for (auto &obstacle : lane_perception_obstacle_) {
        ApfElementPtr obs_ptr =
            std::make_shared<ApfObstacleLine>(obstacle, line_obstalce_param);
        element_ptr_vector.emplace_back(obs_ptr);
      }
    }
    for (auto &obstacle : road_edge_perception_obstacle_) {
      ApfElementPtr obs_ptr =
          std::make_shared<ApfObstacleLine>(obstacle, line_obstalce_param);
      element_ptr_vector.emplace_back(obs_ptr);
    }
    if (attract_type == ApfAttractType::Theta) {
      ApfElementPtr dir_ptr = std::make_shared<ApfDirectionalField>(
          input_.pi_params_.direction_field_intensity,
          planning_math::NormalizeAngle(attract_theta + M_PI),
          planning_math::Vec2d(refline_origin_.x, refline_origin_.y));
      element_ptr_vector.emplace_back(dir_ptr);
    } else if (attract_type == ApfAttractType::Traj &&
               attract_frenet != nullptr) {
      ApfElementPtr frenet_ptr = std::make_shared<ApfFrenetField>(
          input_.pi_params_.direction_field_intensity, 0.0, attract_frenet);
      element_ptr_vector.emplace_back(frenet_ptr);
    }

    planner_->set_obstacles(element_ptr_vector);

    // plan and result
    if (planner_->plan()) {
      auto &result = planner_->get_result();
      for (auto &point : result) {
        refline_result_.emplace_back(point.pose);
      }
      extend_refline_backward(refline_result_);
    } else if (!planner_->plan() && last_refline_result_.size() > 0) {
      refline_result_ = last_refline_result_;
    } else {
      last_refline_result_.clear();
      refline_result_.clear();
      gen_refline_backup(apf_min_length, max(input_.ego_pose_.vel, 0.2) * 6.0,
                         pi_nb_debug);
    }
  } else if (core_type_ == ReflineGeneratorCore::HybridAstar) {
    last_refline_result_.clear();
    refline_result_.clear();
    return false;
  } else {
    last_refline_result_.clear();
    refline_result_.clear();
    return false;
  }

  if (pi_nb_debug != nullptr) {

    for (auto refline_pose : refline_result_) {
      std::vector<double> point_tmp;
      point_tmp.push_back(refline_pose.x);
      point_tmp.push_back(refline_pose.y);
      pi_nb_debug->apf_refline.push_back(point_tmp);
    }
    pi_nb_debug->follow_car_id = follow_car_id_;
    pi_nb_debug->refline_condition = (int)condition_.reflinecondition;
    pi_nb_debug->attract_type = (int)attract_type_;
    pi_nb_debug->ego_theta = input_.ego_pose_.theta;
    pi_nb_debug->road_theta = road_ref_theta_;
    pi_nb_debug->lat_dis_in_intersection = lat_dis_in_intersection_;
    pi_nb_debug->lat_dis_out_intersection = lat_dis_out_intersection_;
    pi_nb_debug->quit_cp = quit_cp_;
    pi_nb_debug->ego_pos.push_back(input_.ego_pose_.x);
    pi_nb_debug->ego_pos.push_back(input_.ego_pose_.y);
    if (refline_target_pt_.size() > 1) {
      pi_nb_debug->refline_target_pt.push_back(refline_target_pt_.at(0));
      pi_nb_debug->refline_target_pt.push_back(refline_target_pt_.at(1));
    }
  }
  return true;
}

bool ReflineGenerator::calc_lane_perception_obstacle() {
  lane_perception_obstacle_.clear();
  road_edge_perception_obstacle_.clear();
  road_edge_info_.clear();
  lanes_info_.clear();
  if (!input_.lane_road_edges_avail_) {
    return false;
  }
  for (auto &lane : input_.lanes_) {
    std::vector<planning_math::LineSegment2d> lines;
    std::vector<planning_math::Vec2d> points;
    for (auto pt : lane) {
      points.emplace_back(planning_math::Vec2d(pt.x, pt.y));
    }
    points_to_linesegment_filter(points,
                                 input_.pi_params_.thin_out_angle_accuracy,
                                 input_.pi_params_.thin_out_max_length, lines);

    // refline generator lane debug info
    for (const auto line : lines) {
      std::vector<std::vector<double>> line_tmp;
      std::vector<double> line_start_pt, line_end_pt;
      line_tmp.clear();
      line_start_pt.push_back(line.start().x());
      line_start_pt.push_back(line.start().y());
      line_end_pt.push_back(line.end().x());
      line_end_pt.push_back(line.end().y());
      line_tmp.push_back(line_start_pt);
      line_tmp.push_back(line_end_pt);
      lanes_info_.push_back(line_tmp);
    }
    lane_perception_obstacle_.insert(lane_perception_obstacle_.end(),
                                     lines.begin(), lines.end());
  }

  for (auto &road_edge : input_.road_edges_) {
    std::vector<planning_math::LineSegment2d> lines;
    std::vector<planning_math::Vec2d> points;
    for (auto pt : road_edge) {
      points.emplace_back(planning_math::Vec2d(pt.x, pt.y));
    }
    points_to_linesegment_filter(points,
                                 input_.pi_params_.thin_out_angle_accuracy,
                                 input_.pi_params_.thin_out_max_length, lines);

    // refline generator road_edge debug info
    for (auto line : lines) {
      std::vector<std::vector<double>> road_edge_tmp;
      std::vector<double> line_start_pt, line_end_pt;
      road_edge_tmp.clear();
      line_start_pt.push_back(line.start().x());
      line_start_pt.push_back(line.start().y());
      line_end_pt.push_back(line.end().x());
      line_end_pt.push_back(line.end().y());
      road_edge_tmp.push_back(line_start_pt);
      road_edge_tmp.push_back(line_end_pt);
      road_edge_info_.push_back(road_edge_tmp);
    }
    road_edge_perception_obstacle_.insert(road_edge_perception_obstacle_.end(),
                                          lines.begin(), lines.end());
  }

  return true;
}

bool ReflineGenerator::calc_apf_road_ref_theta(double &road_ref_theta) {

  if (leader_car_cross_lane_) {
    road_ref_theta = input_.ego_pose_.theta;
    // std::cout << "using ego theta to go straight " << std::endl;
    return true;
  }

  if (input_.current_lane_index_ >= input_.lane_size_ &&
      (int)input_.hist_refline_.size() == 0) {
    road_ref_theta = refline_origin_.theta;
    return false;
  }

  std::vector<Traj_Point> refline_points;

  if (input_.current_last_car_point_x_ < 5.0 ||
      condition_.reflinecondition ==
          pass_intersection_planner::ReflineCondition::
              FIRST_HALF_INTERSECTION) {
    refline_points = input_.hist_refline_;
  } else if (condition_.reflinecondition ==
             pass_intersection_planner::ReflineCondition::
                 LAST_HALF_INTERSECTION) {
    if (input_.current_lane_index_ >= input_.lane_size_) {
      return false;
    }
    refline_points = input_.refline_points_;
  } else {
    refline_points = input_.hist_refline_;
  }

  if (refline_points.empty() || refline_points.size() < 3) {
    road_ref_theta = refline_origin_.theta;
    return false;
  }

  planning_math::Vec2d ego_dir_vec(cos(input_.ego_pose_.theta),
                                   sin(input_.ego_pose_.theta));
  planning_math::Vec2d ego_postion(input_.ego_pose_.x, input_.ego_pose_.y);
  planning_math::Vec2d start_postion(refline_points.front().x,
                                     refline_points.front().y);
  planning_math::Vec2d end_postion(refline_points.back().x,
                                   refline_points.back().y);
  auto start_vec = start_postion - ego_postion;
  auto end_vec = end_postion - ego_postion;
  double start_displacement = start_vec.InnerProd(ego_dir_vec);
  double end_displacement = end_vec.InnerProd(ego_dir_vec);

  if (condition_.reflinecondition ==
          pass_intersection_planner::ReflineCondition::
              FIRST_HALF_INTERSECTION ||
      condition_.reflinecondition ==
          pass_intersection_planner::ReflineCondition::IN_INTERSECTION) {
    int begin_index, end_index;
    if (abs(start_displacement) < abs(end_displacement)) {
      end_index = 0;
      begin_index = std::min(3, (int)(refline_points.size()) - 1);
    } else {
      end_index = (int)refline_points.size() - 1;
      begin_index = std::max((int)refline_points.size() - 4, 0);
      double dis = 0.;
      for (int i = (int)refline_points.size() - 1; i > 0; i--) {
        dis += std::sqrt(
            (refline_points.at(i).x - refline_points.at(i - 1).x) *
                (refline_points.at(i).x - refline_points.at(i - 1).x) +
            (refline_points.at(i).y - refline_points.at(i - 1).y) *
                (refline_points.at(i).y - refline_points.at(i - 1).y));
        if (dis > 10.0) {
          begin_index = i;
          break;
        }
      }
    }
    road_ref_theta =
        atan2(refline_points[end_index].y - refline_points[begin_index].y,
              refline_points[end_index].x - refline_points[begin_index].x);
    return true;
  } else if (condition_.reflinecondition ==
             pass_intersection_planner::ReflineCondition::
                 LAST_HALF_INTERSECTION) {
    int end_index;
    double lood_ahead_distance = std::max(
        std::min(input_.pi_params_.look_ahead_time * input_.ego_pose_.vel,
                 input_.pi_params_.max_look_ahead_distance),
        input_.pi_params_.min_look_ahead_distance);
    if (abs(start_displacement) < abs(end_displacement)) {
      double current_dis = 0;
      end_index = (int)refline_points.size() - 1;
      for (int index = 0; index < (int)refline_points.size() - 1; index++) {
        if (current_dis > lood_ahead_distance) {
          end_index = index;
          break;
        }
        planning_math::Vec2d vec(
            refline_points[index + 1].x - refline_points[index].x,
            refline_points[index + 1].y - refline_points[index].y);
        current_dis += vec.Length();
      }
    } else {
      double current_dis = 0;
      end_index = 0;
      for (int index = (int)refline_points.size() - 1; index > 0; index--) {
        if (current_dis > lood_ahead_distance) {
          end_index = index;
          break;
        }
        planning_math::Vec2d vec(
            refline_points[index - 1].x - refline_points[index].x,
            refline_points[index - 1].y - refline_points[index].y);
        current_dis += vec.Length();
      }
    }
    road_ref_theta = atan2(refline_points[end_index].y - refline_origin_.y,
                           refline_points[end_index].x - refline_origin_.x);

    refline_target_pt_.push_back(refline_points[end_index].x);
    refline_target_pt_.push_back(refline_points[end_index].y);
    return true;
  } else {
    road_ref_theta = refline_origin_.theta;
    return false;
  }
  return false;
}

bool ReflineGenerator::calc_apf_leader_ref_theta(double &leader_ref_theta) {
  planning_math::Vec2d ego_vec =
      planning_math::Vec2d(input_.ego_pose_.x, input_.ego_pose_.y);
  planning_math::Vec2d l_pose_vec =
      planning_math::Vec2d(cos(input_.ego_pose_.theta + M_PI_2),
                           sin(input_.ego_pose_.theta + M_PI_2));

  std::vector<pass_intersection_planner::SimpleLeaderInfo> vehicle_selected;
  vehicle_selected.clear();
  for (auto &vehicle_info : input_.muti_historical_info_) {
    if (vehicle_info.now_car_x_ < 0) {
      continue;
    }
    planning_math::Vec2d position_vec(
        vehicle_info.now_car_x_ - input_.ego_pose_.x,
        vehicle_info.now_car_y_ - input_.ego_pose_.y);
    planning_math::Vec2d info_vec =
        planning_math::Vec2d(vehicle_info.now_car_x_, vehicle_info.now_car_y_);
    double angle_diff = planning_math::NormalizeAngle(position_vec.Angle() -
                                                      input_.ego_pose_.theta);
    double yaw_diff = planning_math::NormalizeAngle(
        vehicle_info.infos_.back().yaw_ - input_.ego_pose_.theta);
    double distance = position_vec.Length();
    if (distance < input_.pi_params_.max_leader_distance &&
        distance > input_.pi_params_.min_leader_distance &&
        abs(angle_diff) < input_.pi_params_.max_angle_diff &&
        abs(yaw_diff) < input_.pi_params_.max_yaw_diff &&
        abs((info_vec - ego_vec).InnerProd(l_pose_vec)) <
            input_.pi_params_.max_l_tolerance) {
      vehicle_selected.emplace_back(vehicle_info.infos_.back());
    }
  }
  std::string theta_cars = "";
  int theta_car_id = -1;
  if (vehicle_selected.empty()) {
    leader_ref_theta = input_.refline_origin_.theta;
    return false;
  } else {
    leader_ref_theta = input_.ego_pose_.theta;
    double min_l = 1000.0;
    for (int index = 0; index < vehicle_selected.size(); index++) {
      double dtheta = 0;
      // only using front car:
      planning_math::Vec2d info_vec =
          planning_math::Vec2d(vehicle_selected[index].position_x_,
                               vehicle_selected[index].position_y_);
      if (abs((info_vec - ego_vec).InnerProd(l_pose_vec)) < min_l) {
        dtheta = vehicle_selected[index].yaw_diff_ *
                     (1 - input_.pi_params_.use_agnle_diff_ratio) +
                 vehicle_selected[index].angle_diff_ *
                     input_.pi_params_.use_agnle_diff_ratio;
        leader_ref_theta =
            planning_math::NormalizeAngle(leader_ref_theta + dtheta);
        min_l = abs((info_vec - ego_vec).InnerProd(l_pose_vec));
        theta_car_id = vehicle_selected[index].id_;
      }
    }
    if (abs(planning_math::NormalizeAngle(input_.ego_pose_.theta -
                                          leader_ref_theta)) <
        input_.pi_params_.max_angle_diff) {
      return true;
    } else {
      return false;
    }
  }

  return false;
}

bool ReflineGenerator::calc_apf_leader_traj(
    std::shared_ptr<FrenetCoordinateSystem> &attract_frenet) {
  pass_intersection_planner::HistoricalLeaderInfo selected_vehicle_info(-1);
  bool find_leader_traj = false;
  selected_hist_vehicle_id_ = input_.selected_hist_vehicle_id_;
  inter_intersection_count_ = input_.inter_intersection_count_;
  planning_math::Vec2d ego_vec =
      planning_math::Vec2d(input_.ego_pose_.x, input_.ego_pose_.y);
  planning_math::Vec2d pose_vec = planning_math::Vec2d(
      cos(input_.ego_pose_.theta), sin(input_.ego_pose_.theta));
  planning_math::Vec2d l_pose_vec =
      planning_math::Vec2d(cos(input_.ego_pose_.theta + M_PI_2),
                           sin(input_.ego_pose_.theta + M_PI_2));
  bool first_select_car = false;
  if (selected_hist_vehicle_id_ == -1 || inter_intersection_count_ < 5) {
    first_select_car = true;
    inter_intersection_count_++;
  }

  for (auto &vehicle_info : input_.muti_historical_info_) {
    if (vehicle_info.infos().empty() || vehicle_info.infos().size() < 3) {
      continue;
    }
    // filter the car which is behind ego
    if (vehicle_info.infos().size() > 1) {
      if (vehicle_info.now_car_x_ < 0) {
        continue;
      }
      if (vehicle_info.now_car_x_ > 65.0) {
        continue;
      }
      if (abs(vehicle_info.now_car_y_) > 3.6) {
        continue;
      }
    }

    planning_math::Vec2d begin_vec =
        planning_math::Vec2d(vehicle_info.infos().front().position_x_,
                             vehicle_info.infos().front().position_y_);
    planning_math::Vec2d end_vec =
        planning_math::Vec2d(vehicle_info.infos().back().position_x_,
                             vehicle_info.infos().back().position_y_);
    double vehicle_traj_dis = (end_vec - begin_vec).Length();
    if (vehicle_traj_dis < 2.0) {
      continue;
    }

    for (auto &info : vehicle_info.infos()) {
      planning_math::Vec2d info_vec =
          planning_math::Vec2d(info.position_x_, info.position_y_);
      if (abs((info_vec - ego_vec).InnerProd(pose_vec)) <
              input_.pi_params_.traj_select_s_tolerance &&
          abs((info_vec - ego_vec).InnerProd(l_pose_vec)) <
              input_.pi_params_.traj_select_l_tolerance &&
          abs(planning_math::NormalizeAngle(input_.ego_pose_.theta -
                                            info.yaw_)) <
              input_.pi_params_.traj_select_yaw_tolerance) {
        find_leader_traj = true;
        selected_vehicle_info = vehicle_info;
        // std::cout << "vehicle " << vehicle_info.leader_id() << " is selected
        // " << std::endl;
      }
    }

    // filter object which is merge
    int obj2adc_min_adc_index = 0;
    double obj2adc_min_dis = std::numeric_limits<double>::max();
    for (int i = 0; i < vehicle_info.infos().size(); i++) {
      planning_math::Vec2d info_vec =
          planning_math::Vec2d(vehicle_info.infos().at(i).position_x_,
                               vehicle_info.infos().at(i).position_y_);
      if ((info_vec - ego_vec).Length() < obj2adc_min_dis) {
        obj2adc_min_dis = (info_vec - ego_vec).Length();
        obj2adc_min_adc_index = i;
      }
    }
    int obj_traj_num_after_adc = 0;
    int obj_traj_far_pt_num = 0;
    for (int i = 0; i < 15 && i < obj2adc_min_adc_index; i++) {
      planning_math::Vec2d info_vec =
          planning_math::Vec2d(vehicle_info.infos().at(i).position_x_,
                               vehicle_info.infos().at(i).position_y_);
      double obj_traj2adc_traj = std::numeric_limits<double>::max();
      for (int j = 0; j < input_.adc_historical_info_.size(); j++) {
        planning_math::Vec2d adc_vec =
            planning_math::Vec2d(input_.adc_historical_info_.at(j).x,
                                 input_.adc_historical_info_.at(j).y);
        if ((info_vec - adc_vec).Length() < obj_traj2adc_traj) {
          obj_traj2adc_traj = (info_vec - adc_vec).Length();
        }
      }
      if (obj_traj2adc_traj > 1.5) {
        obj_traj_far_pt_num++;
      }
      obj_traj_num_after_adc++;
    }

    double ego2obj_traj_min = std::numeric_limits<double>::max();
    for (int i = 0; i < vehicle_info.infos().size(); i++) {
      planning_math::Vec2d info_vec =
          planning_math::Vec2d(vehicle_info.infos().at(i).position_x_,
                               vehicle_info.infos().at(i).position_y_);
      if ((info_vec - ego_vec).Length() < ego2obj_traj_min) {
        ego2obj_traj_min = (info_vec - ego_vec).Length();
      }
    }

    if (obj_traj_num_after_adc > 0 &&
        1.0 * obj_traj_far_pt_num / obj_traj_num_after_adc > 0.6) {
      find_leader_traj = false;
      pass_intersection_planner::HistoricalLeaderInfo selected_vehicle_info_tmp(
          -1);
      if (selected_vehicle_info.leader_id() == vehicle_info.leader_id()) {
        selected_vehicle_info = selected_vehicle_info_tmp;
      }
      // selected_vehicle_info = selected_vehicle_info_tmp;
      // std::cout << "vehicle " << vehicle_info.leader_id()
      //           << " is mergind and continue " << std::endl;
      continue;
    }

    if (true) {
      bool can_select = true;
      for (auto &info : vehicle_info.infos()) {
        planning_math::Vec2d info_vec =
            planning_math::Vec2d(info.position_x_, info.position_y_);
        if (abs((info_vec - ego_vec).InnerProd(l_pose_vec)) < 1.5 &&
            abs(planning_math::NormalizeAngle(input_.ego_pose_.theta -
                                              info.yaw_)) < 0.15) {

        } else {
          can_select = false;
          break;
        }
      }
      // std::cout << "vehicle " << vehicle_info.leader_id()
      //           << " can_select = " << can_select
      //           << " traj_size = " << vehicle_info.leader_id()
      //           << " selected_vehicle_id = "
      //           << selected_vehicle_info.leader_id()
      //           << " selected_vehicle_traj_size = "
      //           << selected_vehicle_info.infos().size() << std::endl;
      if (selected_vehicle_info.leader_id() != -1 && can_select &&
          vehicle_info.infos().size() > 0 &&
          selected_vehicle_info.infos().size() > 0) {

        // comp ego_pose2traj
        double adc2select_traj = std::numeric_limits<double>::max();
        double adc2vehicle_traj = std::numeric_limits<double>::max();
        for (int i = 0; i < selected_vehicle_info.infos().size(); i++) {
          planning_math::Vec2d info_vec = planning_math::Vec2d(
              selected_vehicle_info.infos().at(i).position_x_,
              selected_vehicle_info.infos().at(i).position_y_);
          if ((info_vec - ego_vec).Length() < adc2select_traj) {
            adc2select_traj = (info_vec - ego_vec).Length();
          }
        }
        for (int i = 0; i < vehicle_info.infos().size(); i++) {
          planning_math::Vec2d info_vec =
              planning_math::Vec2d(vehicle_info.infos().at(i).position_x_,
                                   vehicle_info.infos().at(i).position_y_);
          if ((info_vec - ego_vec).Length() < adc2vehicle_traj) {
            adc2vehicle_traj = (info_vec - ego_vec).Length();
          }
        }

        // std::cout << ">>> selected_vehicle id = "
        //           << selected_vehicle_info.leader_id()
        //           << " y = " << selected_vehicle_info.now_car_y_
        //           << "  adc2select_traj = " << adc2select_traj
        //           << " vehicle_id = " << vehicle_info.leader_id()
        //           << " y = " << vehicle_info.now_car_y_
        //           << "  adc2vehicle_traj = " << adc2vehicle_traj <<
        //           std::endl;
        // if (abs(vehicle_info.now_car_y_) <
        //         abs(selected_vehicle_info.now_car_y_) &&
        //     adc2vehicle_traj - adc2select_traj < 0.5) {
        //   find_leader_traj = true;
        //   selected_vehicle_info = vehicle_info;
        // }
        if (abs(vehicle_info.now_car_y_) -
                    abs(selected_vehicle_info.now_car_y_) <
                -1.5 ||
            (abs(vehicle_info.now_car_y_ - selected_vehicle_info.now_car_y_) <
                 1.0 &&
             (adc2vehicle_traj - adc2select_traj < 0.5))) {
          // std::cout << "using " << vehicle_info.leader_id() << " instead of "
          //           << selected_vehicle_info.leader_id() << " because "
          //           << vehicle_info.leader_id() << " lateral dis is more
          //           small "
          //           << std::endl;
          find_leader_traj = true;
          selected_vehicle_info = vehicle_info;
        }

      } else if (can_select) {
        find_leader_traj = true;
        selected_vehicle_info = vehicle_info;
      } else {
      }
    }

    // 解决路口对不齐，导致自车没有选中前车轨迹进行跟车的case
    int lap_count = 0;
    for (int i = (int)adc_historical_info_.size() - 1;
         i > std::max(0, (int)adc_historical_info_.size() - 50); i--) {
      planning_math::Vec2d adc_pos_vec = adc_historical_info_.at(i);
      for (int j = (int)vehicle_info.infos().size() - 1; j > 0; j--) {
        planning_math::Vec2d vehicle_pos_vec =
            planning_math::Vec2d(vehicle_info.infos().at(j).position_x_,
                                 vehicle_info.infos().at(j).position_y_);
        if ((adc_pos_vec - vehicle_pos_vec).Length() < 1.0) {
          lap_count++;
          continue;
        }
      }
    }

    if (lap_count > 30 && (end_vec - ego_vec).Length() > 10.0 &&
        abs((end_vec - ego_vec).InnerProd(l_pose_vec)) < 2.2 &&
        abs(planning_math::NormalizeAngle(
            input_.ego_pose_.theta - vehicle_info.infos().back().yaw_)) < 3.0) {
      find_leader_traj = true;
      selected_vehicle_info = vehicle_info;
    }

    // debug info
    planning_math::Vec2d obs_pos_info_vec =
        planning_math::Vec2d(vehicle_info.infos().back().position_x_,
                             vehicle_info.infos().back().position_y_);

    // follow the same car when pas intersection!!
    for (auto &vehicle_info : input_.muti_historical_info_) {
      if (first_select_car) {
        break;
      }
      if ((!find_leader_traj &&
           vehicle_info.leader_id() == selected_hist_vehicle_id_) ||
          (find_leader_traj &&
           selected_vehicle_info.leader_id() != selected_hist_vehicle_id_ &&
           vehicle_info.leader_id() == selected_hist_vehicle_id_)) {
        find_leader_traj = true;
        selected_vehicle_info = vehicle_info;
        selected_hist_vehicle_id_ = selected_vehicle_info.leader_id();
      }
    }

    if (find_leader_traj) {
      selected_hist_vehicle_id_ = selected_vehicle_info.leader_id();
      // break;
    }
  }

  // filter car turn left or right!!!
  if (selected_vehicle_info.leader_id() != -1) {
    planning_math::Vec2d obs_pos_info_vec =
        planning_math::Vec2d(selected_vehicle_info.infos().back().position_x_,
                             selected_vehicle_info.infos().back().position_y_);

    planning_math::Vec2d begin_vec =
        planning_math::Vec2d(selected_vehicle_info.infos().front().position_x_,
                             selected_vehicle_info.infos().front().position_y_);
    planning_math::Vec2d end_vec =
        planning_math::Vec2d(selected_vehicle_info.infos().back().position_x_,
                             selected_vehicle_info.infos().back().position_y_);

    // std::cout << ">>>> filter car turn left or right " << std::endl;
    // std::cout << "ego_theta = " << input_.ego_pose_.theta
    //           << " selected_car_theta = "
    //           << selected_vehicle_info.infos().back().yaw_ << " theta_diff =
    //           "
    //           << input_.ego_pose_.theta -
    //                  selected_vehicle_info.infos().back().yaw_
    //           << " l_dis = "
    //           << abs((obs_pos_info_vec - ego_vec).InnerProd(l_pose_vec))
    //           << " car_ego_dis = " << (end_vec - ego_vec).Length() <<
    //           std::endl;
    if (abs(planning_math::NormalizeAngle(
            input_.ego_pose_.theta -
            selected_vehicle_info.infos().back().yaw_)) >
            input_.pi_params_.traj_select_yaw_tolerance &&
        ((end_vec - ego_vec).Length() < 35 ||
         abs((obs_pos_info_vec - ego_vec).InnerProd(l_pose_vec)) > 7.2)) {
      find_leader_traj = false;
      selected_hist_vehicle_id_ = -1;
      pass_intersection_planner::HistoricalLeaderInfo selected_vehicle_info_tmp(
          -1);
      selected_vehicle_info = selected_vehicle_info_tmp;
    }
  }

  if (selected_hist_vehicle_id_ != -1) {
    // find_leader_traj = true;
    for (auto &vehicle_info : input_.muti_historical_info_) {
      if (vehicle_info.leader_id() == selected_hist_vehicle_id_) {
        find_leader_traj = true;
        selected_vehicle_info = vehicle_info;
        break;
      }
    }
  }

  // id jump
  if (selected_vehicle_info.leader_id() != -1 &&
      selected_vehicle_info.infos().size() > 3) {
    planning_math::Vec2d first_end_vec = planning_math::Vec2d(
        selected_vehicle_info.infos()
            .at((int)selected_vehicle_info.infos().size() - 1)
            .position_x_,
        selected_vehicle_info.infos()
            .at((int)selected_vehicle_info.infos().size() - 1)
            .position_y_);
    planning_math::Vec2d second_end_vec = planning_math::Vec2d(
        selected_vehicle_info.infos()
            .at((int)selected_vehicle_info.infos().size() - 2)
            .position_x_,
        selected_vehicle_info.infos()
            .at((int)selected_vehicle_info.infos().size() - 2)
            .position_y_);
    double latest_move_dist = (first_end_vec - second_end_vec).Length();
    if (latest_move_dist > 3.0) {
      find_leader_traj = false;
      selected_hist_vehicle_id_ = -1;
      pass_intersection_planner::HistoricalLeaderInfo selected_vehicle_info_tmp(
          -1);
      selected_vehicle_info = selected_vehicle_info_tmp;
    }
  }

  if (find_leader_traj) {
    // std::cout << "refline_generator follow car = "
    //           << selected_vehicle_info.leader_id() << std::endl;
  }

  // judge lead car crossing perception lane
  if (input_.cur_lane_left_.size() > 2 &&
      selected_vehicle_info.leader_id() != -1) {
    if (input_.cur_lane_left_.front().x < -10 &&
        input_.cur_lane_left_.back().x > 10) {
      for (auto lane_pt : input_.cur_lane_left_) {
        if (selected_vehicle_info.now_car_y_ + 0.9 > lane_pt.y &&
            lane_pt.x > 0) {
          find_leader_traj = false;
          leader_car_cross_lane_ = true;
          return false;
        }
      }
    }
  }
  if (input_.cur_lane_right_.size() > 2 &&
      selected_vehicle_info.leader_id() != -1) {
    if (input_.cur_lane_right_.front().x < -10 &&
        input_.cur_lane_right_.back().x > 10) {
      for (auto lane_pt : input_.cur_lane_right_) {
        if (selected_vehicle_info.now_car_y_ - 0.9 < lane_pt.y &&
            lane_pt.x > 0) {
          find_leader_traj = false;
          leader_car_cross_lane_ = true;
          return false;
        }
      }
    }
  }

  // extract lane width
  cur_lane_left_border_ = 2.25;
  cur_lane_right_border_ = 2.25;
  if (input_.cur_lane_left_.size() > 2 &&
      selected_vehicle_info.leader_id() != -1) {
    if (input_.cur_lane_left_.front().x < -10 &&
        input_.cur_lane_left_.back().x > 5.0) {
      for (auto lane_pt : input_.cur_lane_left_) {
        if (lane_pt.y < cur_lane_left_border_ && lane_pt.x > 0) {
          cur_lane_left_border_ = lane_pt.y;
        }
      }
    }
  }

  if (input_.cur_lane_right_.size() > 2 &&
      selected_vehicle_info.leader_id() != -1) {
    if (input_.cur_lane_right_.front().x < -10 &&
        input_.cur_lane_right_.back().x > 5.0) {
      for (auto lane_pt : input_.cur_lane_right_) {
        if (lane_pt.y < cur_lane_right_border_ && lane_pt.x > 0) {
          cur_lane_right_border_ = lane_pt.y;
        }
      }
    }
  }

  if (find_leader_traj && selected_vehicle_info.leader_id() != -1 &&
      selected_vehicle_info.infos().size() > 0) {
    follow_car_id_ = selected_vehicle_info.leader_id();

    std::vector<double> x_vector, y_vector;
    double current_s;
    double ego2follow_obj_traj_min = std::numeric_limits<double>::max();
    for (int i = 0; i < selected_vehicle_info.infos().size(); i++) {
      planning_math::Vec2d info_vec =
          planning_math::Vec2d(selected_vehicle_info.infos().at(i).position_x_,
                               selected_vehicle_info.infos().at(i).position_y_);
      if ((info_vec - ego_vec).Length() < ego2follow_obj_traj_min) {
        ego2follow_obj_traj_min = (info_vec - ego_vec).Length();
      }
    }
    if (ego2follow_obj_traj_min > 5.0) {
      x_vector.emplace_back(input_.ego_pose_.x);
      y_vector.emplace_back(input_.ego_pose_.y);
    }
    x_vector.emplace_back(selected_vehicle_info.infos().front().position_x_);
    y_vector.emplace_back(selected_vehicle_info.infos().front().position_y_);
    planning_math::Vec2d begin_vec =
        planning_math::Vec2d(selected_vehicle_info.infos().front().position_x_,
                             selected_vehicle_info.infos().front().position_y_);
    current_s = (begin_vec - ego_vec).InnerProd(pose_vec);
    for (auto &info : selected_vehicle_info.infos()) {
      planning_math::Vec2d info_vec =
          planning_math::Vec2d(info.position_x_, info.position_y_);
      double s = (info_vec - ego_vec).InnerProd(pose_vec);
      if ((s - current_s) > 2.0) {
        x_vector.emplace_back(info.position_x_);
        y_vector.emplace_back(info.position_y_);
        current_s = s;
      }
    }
    if (x_vector.size() < 3) {
      return false;
    }
    attract_frenet = std::make_shared<FrenetCoordinateSystem>(
        x_vector, y_vector, attract_coord_parameters_);
    return true;
  }

  return false;
}

bool ReflineGenerator::calc_apf_attract_field(
    ApfAttractType &attract_type,
    std::shared_ptr<FrenetCoordinateSystem> &attract_frenet,
    double &attract_theta) {
  double road_ref_theta;
  double leader_ref_theta;
  bool attract_frenet_exist, leader_theta_exist, road_theta_exist;
  attract_frenet_exist = calc_apf_leader_traj(attract_frenet);
  road_theta_exist = calc_apf_road_ref_theta(road_ref_theta);
  leader_theta_exist = calc_apf_leader_ref_theta(leader_ref_theta);
  bool attract_refline_frenet_exist = false;
  if (!attract_frenet_exist &&
      condition_.reflinecondition !=
          pass_intersection_planner::ReflineCondition::LAST_HALF_INTERSECTION &&
      condition_.reflinecondition !=
          pass_intersection_planner::ReflineCondition::NO_INTERSECTION) {
    // attract_refline_frenet_exist = calc_refline_traj(attract_frenet);
  }
  road_ref_theta_ = road_ref_theta;

  attract_type = ApfAttractType::Unknow;
  if (condition_.reflinecondition ==
          pass_intersection_planner::ReflineCondition::LAST_HALF_INTERSECTION &&
      road_theta_exist) {
    attract_theta = road_ref_theta;
    attract_type = ApfAttractType::Theta;
    return true;
  } else if (attract_frenet_exist) {
    attract_type = ApfAttractType::Traj;
    return true;
  } else if ((condition_.reflinecondition !=
                  pass_intersection_planner::ReflineCondition::
                      LAST_HALF_INTERSECTION &&
              condition_.reflinecondition !=
                  pass_intersection_planner::ReflineCondition::
                      NO_INTERSECTION) &&
             attract_refline_frenet_exist && input_.pi_target_pt_exist_ &&
             input_.pi_params_.enable_follow_target_pt) {
    attract_type = ApfAttractType::Traj;
    is_target_pt_flag_ = true;
    return true;
  } else if (condition_.reflinecondition ==
                 pass_intersection_planner::ReflineCondition::
                     FIRST_HALF_INTERSECTION &&
             leader_theta_exist) {
    attract_theta = leader_ref_theta;
    attract_type = ApfAttractType::Theta;
    return true;
  } else if (road_theta_exist) {
    attract_theta = road_ref_theta;
    attract_type = ApfAttractType::Theta;
    if (abs(attract_theta - input_.ego_pose_.theta) > 0.06) {
      attract_theta = input_.ego_pose_.theta;
    }
    // write enter intersection and no lead car  then follow road_ref_theta
    // always!!
    return true;
  } else if (leader_theta_exist) {
    attract_theta = leader_ref_theta;
    attract_type = ApfAttractType::Theta;
    return true;
  } else {
    attract_theta = refline_origin_.theta;
    attract_type = ApfAttractType::Theta;
    if (abs(attract_theta - input_.ego_pose_.theta) > 0.06) {
      attract_theta = input_.ego_pose_.theta;
    }
    return true;
  }
  return false;
}

bool ReflineGenerator::calc_refline_origin(bool force_replan) {
  Pose2D ego_pose;
  ego_pose.x = input_.ego_pose_.x;
  ego_pose.y = input_.ego_pose_.y;
  ego_pose.theta = input_.ego_pose_.theta;
  if (force_replan) {
    refline_origin_ = ego_pose;
    return true;
  }
  if (!last_refline_result_.empty()) {
    double min_dis = 10.0;
    int index = -1;
    for (int i = 0; i < last_refline_result_.size(); i++) {
      if (distance(last_refline_result_[i], ego_pose) < min_dis) {
        index = i;
        min_dis = distance(last_refline_result_[i], ego_pose);
      }
    }
    if (min_dis < input_.pi_params_.replan_threshold && index != -1) {
      refline_origin_ = last_refline_result_[index];
    } else {
      refline_origin_ = ego_pose;
    }
  } else {
    refline_origin_ = ego_pose;
  }

  return true;
}

double ReflineGenerator::distance(Pose2D pose1, Pose2D pose2) {
  double dis = std::sqrt((pose1.x - pose2.x) * (pose1.x - pose2.x) +
                         (pose1.y - pose2.y) * (pose1.y - pose2.y));
  return dis;
}

void ReflineGenerator::judge_quit_CP() {
  planning_math::Vec2d l_pose_vec_now =
      planning_math::Vec2d(cos(input_.ego_pose_.theta + M_PI_2),
                           sin(input_.ego_pose_.theta + M_PI_2));
  planning_math::Vec2d ego_vec =
      planning_math::Vec2d(input_.ego_pose_.x, input_.ego_pose_.y);

  if (input_.in_intersection_pos_.size() > 1 &&
      abs(input_.in_intersection_ego_theta_) > 0.0000001) {
    planning_math::Vec2d l_pose_vec_in =
        planning_math::Vec2d(cos(input_.in_intersection_ego_theta_ + M_PI_2),
                             sin(input_.in_intersection_ego_theta_ + M_PI_2));
    planning_math::Vec2d in_pos_vec = planning_math::Vec2d(
        input_.in_intersection_pos_.at(0), input_.in_intersection_pos_.at(1));
    lat_dis_in_intersection_ =
        abs((in_pos_vec - ego_vec).InnerProd(l_pose_vec_in));
    if (abs((in_pos_vec - ego_vec).InnerProd(l_pose_vec_in)) >
            input_.pi_params_.quit_cp_lat_dis_in_inter &&
        condition_.reflinecondition !=
            pass_intersection_planner::ReflineCondition::
                LAST_HALF_INTERSECTION) {
      apf_lat_far_flag_ = true;
      apf_quit_cp_reason_ = string("ego_to_in large");
      lat_dis_2_in_inter_ =
          abs((in_pos_vec - ego_vec).InnerProd(l_pose_vec_in));
      quit_cp_ = 1;
    }

    if (condition_.reflinecondition ==
        pass_intersection_planner::ReflineCondition::LAST_HALF_INTERSECTION) {
      if (input_.current_lane_index_ >= input_.lane_size_) {
        return;
      }
      auto &refline_points = input_.refline_points_;
      if (refline_points.empty()) {
        return;
      }
      planning_math::Vec2d refline_first_pt =
          planning_math::Vec2d(refline_points.at(0).x, refline_points.at(0).y);
      double refline_consider_dis = 0.0;
      double max_ego2refline_lat_dis = 0.0;
      for (int i = 0; i < refline_points.size(); i++) {
        planning_math::Vec2d refline_pt_tmp = planning_math::Vec2d(
            refline_points.at(i).x, refline_points.at(i).y);
        if (i - 1 > 0) {
          planning_math::Vec2d refline_pt_tmp_last = planning_math::Vec2d(
              refline_points.at(i - 1).x, refline_points.at(i - 1).y);
          refline_consider_dis +=
              (refline_pt_tmp - refline_pt_tmp_last).Length();
        }
        if (refline_consider_dis >
            input_.pi_params_.quit_cp_consider_refline_length) {
          break;
        }
        if (abs((ego_vec - refline_pt_tmp).InnerProd(l_pose_vec_now)) >
            max_ego2refline_lat_dis) {
          max_ego2refline_lat_dis =
              abs((ego_vec - refline_pt_tmp).InnerProd(l_pose_vec_now));
        }
      }

      lat_dis_out_intersection_ =
          abs((ego_vec - refline_first_pt).InnerProd(l_pose_vec_now));
      if (max_ego2refline_lat_dis > input_.pi_params_.quit_cp_lat_dis_refline) {
        apf_lat_far_flag_ = true;
        apf_quit_cp_reason_ = string("ego_to_refline large");
        max_ego2refline_lat_dis_ = max_ego2refline_lat_dis;
        lat_dis_2_refline_ =
            abs((ego_vec - refline_first_pt).InnerProd(l_pose_vec_now));
        quit_cp_ = 1;
      }
    }
  }
}

void ReflineGenerator::gen_refline_backup(double apf_min_length,
                                          double vel_pre_dis,
                                          PiNotebookDebug *pi_nb_debug) {
  refline_result_.clear();
  double start_x = input_.ego_pose_.x;
  double start_y = input_.ego_pose_.y;
  double start_yaw = input_.ego_pose_.theta;
  double start_v = std::max(input_.ego_pose_.vel, 0.1);

  if (std::fabs(std::fabs(start_yaw) - M_PI / 2.0) < 1e-4) {
    double reflint_dis = 0.0;
    double dir = 1.0;
    const double step = 1.0;
    if (start_yaw > 0) {
      dir = 1.0;
    } else {
      dir = -1.0;
    }
    for (int i = 0; reflint_dis < max(apf_min_length, vel_pre_dis); i++) {
      refline_result_.emplace_back(
          Pose2D(start_x, start_y + dir * step * i, start_yaw));
      if ((int)refline_result_.size() >= 2) {
        reflint_dis += std::hypotf(
            refline_result_.back().y -
                refline_result_.at((int)refline_result_.size() - 2).y,
            refline_result_.back().x -
                refline_result_.at((int)refline_result_.size() - 2).x);
      }
    }
  } else {
    double reflint_dis = 0.0;
    double dir = 0;
    if (std::fabs(start_yaw) < M_PI / 2.0) {
      dir = 1.0;
    } else if (std::fabs(start_yaw) > M_PI / 2.0) {
      dir = -1.0;
    }
    double enu_k = tan(start_yaw);
    double step = 1.0 * std::fabs(cos(start_yaw));
    for (int i = 0; reflint_dis < max(apf_min_length, vel_pre_dis); i++) {
      Pose2D pt_tmp;
      pt_tmp.x = start_x + dir * step * i;
      pt_tmp.y = enu_k * (pt_tmp.x - start_x) + start_y;
      pt_tmp.theta = start_yaw;
      refline_result_.push_back(pt_tmp);
      if ((int)refline_result_.size() >= 2) {
        reflint_dis += std::hypotf(
            refline_result_.back().y -
                refline_result_.at((int)refline_result_.size() - 2).y,
            refline_result_.back().x -
                refline_result_.at((int)refline_result_.size() - 2).x);
      }
    }
  }

  // extend backward
  if (refline_result_.size() < 3) {
    return;
  }
  int extend_index = -1;
  for (int i = 1; i < refline_result_.size(); i++) {
    double dx_tmp = refline_result_[i].x - refline_result_[0].x;
    double dy_tmp = refline_result_[i].y - refline_result_[0].y;
    if (std::hypotf(dx_tmp, dy_tmp) > 1.0) {
      extend_index = i;
      break;
    }
  }
  extend_index =
      extend_index > 0 ? extend_index : (int)refline_result_.size() - 1;
  double dx = refline_result_[extend_index].x - refline_result_[0].x;
  double dy = refline_result_[extend_index].y - refline_result_[0].y;
  double theta = atan2(dy, dx);

  for (int i = 1; i <= input_.pi_params_.back_extend_points_num; i++) {
    Pose2D point_sample;
    point_sample.x = start_x - dx * i;
    point_sample.y = start_y - dy * i;
    point_sample.theta = theta;
    refline_result_.insert(refline_result_.begin(), point_sample);
  }

  // if (pi_nb_debug != nullptr) {
  //   pi_nb_debug->quintic_refline.clear();
  //   for (auto refline_pose : refline_result_) {
  //     Traj_Point traj_pt;
  //     traj_pt.x = refline_pose.x;
  //     traj_pt.y = refline_pose.y;
  //     pi_nb_debug->quintic_refline.push_back(traj_pt);
  //   }
  // }

  return;
}

void points_to_linesegment_filter(
    std::vector<planning_math::Vec2d> &points, double angle_accuracy,
    double max_segment_length,
    std::vector<planning_math::LineSegment2d> &lines) {
  int segment_start_index = 0;
  for (int current_index = 0; current_index < points.size(); current_index++) {
    if (current_index == points.size() - 1) {
      lines.emplace_back(planning_math::LineSegment2d(
          points[segment_start_index], points[current_index]));
    } else if (current_index == segment_start_index ||
               current_index == segment_start_index + 1) {
    } else {
      planning_math::LineSegment2d last_segment(points[segment_start_index],
                                                points[current_index - 1]);
      planning_math::LineSegment2d new_segment(points[current_index - 1],
                                               points[current_index]);
      double d_theta = planning_math::NormalizeAngle(last_segment.heading() -
                                                     new_segment.heading());
      if ((last_segment.length() + new_segment.length() > max_segment_length) ||
          std::abs(d_theta) > angle_accuracy) {
        lines.emplace_back(last_segment);
        segment_start_index = current_index - 1;
        current_index--; // parasoft-suppress AUTOSAR-M6_5_3 "f-drop"
      }
    }
  }
}

double calc_y(const std::vector<double> &coefs, double x) {
  double result = 0;

  for (std::size_t i = 0; i < coefs.size(); i++) {
    result = result * x + coefs[i];
  }

  return result;
}

// quintic
bool ReflineGenerator::spline_fit_refline(PiNotebookDebug *pi_nb_debug) {
  std::vector<Traj_Point> refline_points;
  refline_points = input_.refline_points_;

  double start_x = input_.ego_pose_.x;
  double start_y = input_.ego_pose_.y;
  double start_yaw = input_.ego_pose_.theta;
  double start_v = std::max(input_.ego_pose_.vel, 0.1);
  double start_a = 0.0;
  if (refline_points.size() < 3) {
    return false;
  }
  double end_x = refline_points.front().x;
  double end_y = refline_points.front().y;
  double end_x_1 = refline_points.at(1).x;
  double end_y_1 = refline_points.at(1).y;
  double end_yaw = atan2((end_y_1 - end_y), (end_x_1 - end_x));

  // find a proper end
  double ref_x, ref_y;
  if (abs(end_yaw - M_PI) < 1e-4) {
    ref_x = end_x;
    ref_y = start_y + tan(start_yaw) * (ref_x - start_x);
  } else if (abs(start_yaw - M_PI) < 1e-4) {
    ref_x = start_x;
    ref_y = end_y + tan(end_yaw) * (ref_x - end_x);
  } else {
    ref_x =
        (start_y - end_y + tan(end_yaw) * end_x - tan(start_yaw) * start_x) /
        (tan(end_yaw) - tan(start_yaw));
    ref_y = end_y + tan(end_yaw) * (ref_x - end_x);
  }

  int ref_size = std::max(0, (int)refline_points.size() - 1);
  double near_dis = std::numeric_limits<double>::max();
  int ref_index;
  int farthest_index = input_.pi_params_.farthest_index;
  farthest_index = min(ref_size, farthest_index);

  for (int i = 0; i <= farthest_index; i++) {
    if (std::hypotf(ref_y - refline_points.at(i).y,
                    ref_x - refline_points.at(i).x) < near_dis) {
      near_dis = std::hypotf(ref_y - refline_points.at(i).y,
                             ref_x - refline_points.at(i).x);
      ref_index = i;
    }
  }

  // if the cross point is behind the ego, get the farthest refpoint
  if (abs(atan2(ref_y - start_y, ref_x - start_x) - start_yaw) > M_PI / 2) {
    ref_index = farthest_index;
  }

  // get relative pos at car-coord
  double rx = (refline_points.at(ref_index).x - start_x) * cos(-start_yaw) -
              (refline_points.at(ref_index).y - start_y) * sin(-start_yaw);
  double ry = (refline_points.at(ref_index).x - start_x) * sin(-start_yaw) +
              (refline_points.at(ref_index).y - start_y) * cos(-start_yaw);

  if (6.0 < abs(rx / ry) <= 7.0) {
    ref_index = min(ref_index * 3, ref_size);
  } else if (5.0 < abs(rx / ry) <= 6.0) {
    ref_index = min(ref_index * 4, ref_size);
  } else if (4.0 < abs(rx / ry) <= 5.0) {
    ref_index = min(ref_index * 5, ref_size);
  } else if (abs(rx / ry) <= 4.0) {
    ref_index = min(ref_index * 6, ref_size);
  }

  end_x = refline_points.at(ref_index).x;
  end_y = refline_points.at(ref_index).y;

  if (ref_index > 0) {
    end_yaw = atan2(end_y - refline_points.at(ref_index - 1).y,
                    end_x - refline_points.at(ref_index - 1).x);
  } else {
    end_yaw = atan2(refline_points.at(ref_index + 1).y - end_y,
                    refline_points.at(ref_index + 1).x - end_x);
  }

  double end_v = start_v;
  double end_a = start_a;

  planning_math::Vec2d vec(end_x - start_x, end_y - start_y);
  double len_size = vec.Length();

  double polyfit_time = len_size / start_v;

  std::array<double, 5> start = {
      {start_x, start_y, start_yaw, start_v, start_a}};
  std::array<double, 5> end = {{end_x, end_y, end_yaw, end_v, end_a}};

  msquare::planning_math::QuinticPoly2d quintic_poly_2d(start, end,
                                                        polyfit_time);

  Pose2D point_sample;
  double dis_length = 0;
  double delta_y;
  double delta_x;
  int point_number = len_size / 0.3;

  if (point_number == 0) {
    return false;
  }

  for (int i = 0; i <= point_number; i++) {

    double t = i * polyfit_time / point_number;
    double x_sample = quintic_poly_2d.get_x_dis(0, t);
    double y_sample = quintic_poly_2d.get_y_dis(0, t);

    point_sample.x = x_sample;
    point_sample.y = y_sample;

    if (i == 0) {
      point_sample.theta = input_.ego_pose_.theta;
    } else {
      delta_y = y_sample - quintic_poly_2d.get_y_dis(0, (i - 1) * polyfit_time /
                                                            point_number);
      delta_x = x_sample - quintic_poly_2d.get_x_dis(0, (i - 1) * polyfit_time /
                                                            point_number);
      point_sample.theta = atan2(delta_y, delta_x);
      dis_length += std::hypotf(delta_y, delta_x);
    }
    refline_result_.emplace_back(point_sample);
    if (pi_nb_debug != nullptr) {
      std::vector<double> pt{point_sample.x, point_sample.y};
      pi_nb_debug->quintic_refline.emplace_back(std::move(pt));
    }
  }

  double max_length = std::max(std::min(input_.ego_pose_.vel * 6.0, 150.0),
                               input_.pi_params_.max_length);
  double min_length = input_.pi_params_.min_length;

  // cat(quintic, refline_points)
  if (refline_points.size() > 2) {
    for (int i = ref_index + 1;
         i < (int)refline_points.size() && dis_length <= max_length; i++) {

      point_sample.x = refline_points[i].x;
      point_sample.y = refline_points[i].y;
      point_sample.theta = refline_points[i].theta;
      refline_result_.emplace_back(point_sample);

      delta_y = point_sample.y - refline_points[i - 1].y;
      delta_x = point_sample.x - refline_points[i - 1].x;
      dis_length += std::hypotf(delta_y, delta_x);

      if (pi_nb_debug != nullptr) {
        std::vector<double> pt{point_sample.x, point_sample.y};
        pi_nb_debug->origin_refline.emplace_back(std::move(pt));
      }
    }
  }

  // satisfy the max length
  if (dis_length < max_length) {
    if (refline_result_.size() < 3) {
      return false;
    }
    int extend_index = -1;
    for (int i = (int)refline_result_.size() - 1; i >= 0; i--) {
      double dx_tmp = refline_result_.at(i).x - refline_result_.back().x;
      double dy_tmp = refline_result_.at(i).y - refline_result_.back().y;
      if (std::hypotf(dx_tmp, dy_tmp) > 0.8) {
        extend_index = i;
        break;
      }
    }
    extend_index = extend_index > 0 ? extend_index : 0;
    double dx = refline_result_.at(extend_index).x - refline_result_.back().x;
    double dy = refline_result_.at(extend_index).y - refline_result_.back().y;
    double theta = atan2(dy, dx);

    double start_x = refline_result_.back().x;
    double start_y = refline_result_.back().y;

    for (int i = 1; dis_length < max_length; i++) {
      double delta_y = start_x - dx * i - refline_result_.back().x;
      double delta_x = start_y - dy * i - refline_result_.back().y;
      dis_length += std::hypotf(delta_y, delta_x);
      refline_result_.emplace_back(start_x - dx * i, start_y - dy * i, theta);
      if (pi_nb_debug != nullptr) {
        std::vector<double> pt{start_x - dx * i, start_y - dy * i};
        pi_nb_debug->extend_refline.emplace_back(std::move(pt));
      }
    }
  }

  // extend backward
  extend_refline_backward(refline_result_);

  if (pi_nb_debug != nullptr) {
    for (auto refline_pose : refline_result_) {
      std::vector<double> point_tmp;
      point_tmp.push_back(refline_pose.x);
      point_tmp.push_back(refline_pose.y);
      pi_nb_debug->apf_refline.push_back(point_tmp);
    }
    pi_nb_debug->refline_condition = (int)condition_.reflinecondition;
    pi_nb_debug->ego_pos.push_back(input_.ego_pose_.x);
    pi_nb_debug->ego_pos.push_back(input_.ego_pose_.y);
    pi_nb_debug->quintic_end.push_back(end_x);
    pi_nb_debug->quintic_end.push_back(end_y);
  }
  return true;
}

void ReflineGenerator::extend_refline_backward(
    std::vector<Pose2D> &refline_result) {
  int extend_index = -1;
  for (int i = 1; i < refline_result.size(); i++) {
    double dx_tmp = refline_result[i].x - refline_result[0].x;
    double dy_tmp = refline_result[i].y - refline_result[0].y;
    if (std::hypotf(dx_tmp, dy_tmp) > 1.0) {
      extend_index = i;
      break;
    }
  }
  extend_index =
      extend_index > 0 ? extend_index : (int)refline_result.size() - 1;
  double dx = refline_result[extend_index].x - refline_result[0].x;
  double dy = refline_result[extend_index].y - refline_result[0].y;
  double theta = atan2(dy, dx);

  Pose2D point_sample;
  double start_x = refline_result.front().x;
  double start_y = refline_result.front().y;
  for (int i = 1; i <= input_.pi_params_.back_extend_points_num; i++) {
    point_sample.x = start_x - dx * i;
    point_sample.y = start_y - dy * i;
    point_sample.theta = theta;
    refline_result.insert(refline_result.begin(), point_sample);
  }
  return;
}
} // namespace msquare