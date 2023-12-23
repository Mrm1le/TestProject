#include "common/pass_intersection_planner_preprocessor.h"
#include "common/map_info_manager.h"
#include "common/utils/polyfit.h"
#include "common/world_model.h"
#include "planner/motion_planner/apf_planner.h"

namespace msquare {

const double close_ref_point_threshold_ = 2.0;
// using namespace pass_intersection_planner;

void PassIntersectionPlannerPreprocessor::process() {
  const auto &pass_intersection_planner_input =
      PlanningContext::Instance()->mutable_pass_intersection_planner_input();
  pass_intersection_planner_input->last_condition_.reflinecondition =
      pass_intersection_planner_input->condition_.reflinecondition;

  // 在每一帧 input 更新之前，先更新上一帧及历史某一帧信息:
  auto refline_generator = world_model_->get_refline_generator();
  pass_intersection_planner_input->selected_hist_vehicle_id_ =
      refline_generator->get_selected_hist_vehicle_id();
  pass_intersection_planner_input->inter_intersection_count_ =
      refline_generator->get_inter_intersection_count();
  if (refline_generator->refline_result().size() > 0) {
    pass_intersection_planner_input->last_refline_result_.clear();
  }
  for (auto p : refline_generator->refline_result()) {
    Traj_Point last_ref_point;
    last_ref_point.x = p.x;
    last_ref_point.y = p.y;
    last_ref_point.theta = p.theta;
    last_ref_point.vel = 0.0;
    pass_intersection_planner_input->last_refline_result_.push_back(
        last_ref_point);
  }
  pass_intersection_planner_input->target_pt_last_ =
      refline_generator->get_target_pt();
  pass_intersection_planner_input->target_pt_count_ =
      refline_generator->get_target_pt_count();

  generate_output();

  if (pass_intersection_planner_input->condition_.reflinecondition ==
          ReflineCondition::NO_INTERSECTION ||
      pass_intersection_planner_input->condition_.reflinecondition ==
          ReflineCondition::UNKNOW) {
    pass_intersection_planner_input->last_refline_result_.clear();
    pass_intersection_planner_input->inter_intersection_count_ = 0;
    pass_intersection_planner_input->is_in_intersection_cond_ = true;
    pass_intersection_planner_input->in_intersection_ego_theta_ = 0;
    pass_intersection_planner_input->in_intersection_pos_.clear();
    pass_intersection_planner_input->selected_hist_vehicle_id_ = -1;

    // clear object when can not see in intersection
    auto &truncated_prediction_info =
        world_model_->get_truncated_prediction_info();
    auto it = pass_intersection_planner_input->muti_historical_info_.begin();
    while (it != pass_intersection_planner_input->muti_historical_info_.end()) {
      bool obj_exist = false;
      for (auto &vehicle_info : truncated_prediction_info) {
        if (it->leader_id() == vehicle_info.id) {
          obj_exist = true;
          break;
        }
      }
      if (!obj_exist) {
        it = pass_intersection_planner_input->muti_historical_info_.erase(it);
      } else {
        it++;
      }
    }
  }
}

void PassIntersectionPlannerPreprocessor::generate_output() {
  set_init_params();
  set_ego_pose();
  set_reflinepoints();
  set_target_pts();
  check_intersection_condition();
  update_muti_historical_info();
  save_hist_refline();
  (void)calc_lane_perception_obstacle();
  return;
}

void PassIntersectionPlannerPreprocessor::set_init_params() {
  const auto &pass_intersection_planner_input =
      PlanningContext::Instance()->mutable_pass_intersection_planner_input();
  auto generator_cfg_ = ReflineGeneratorConfig::Instance();
  pass_intersection_planner_input->pi_params_.core_type =
      generator_cfg_->core_type;
  pass_intersection_planner_input->pi_params_.enable_follow_target_pt =
      generator_cfg_->enable_follow_target_pt;
  pass_intersection_planner_input->pi_params_.shrink_ratio =
      generator_cfg_->shrink_ratio;
  pass_intersection_planner_input->pi_params_.step_size =
      generator_cfg_->step_size;
  pass_intersection_planner_input->pi_params_.min_turning_radius =
      generator_cfg_->min_turning_radius;
  pass_intersection_planner_input->pi_params_.vehicle_length =
      generator_cfg_->vehicle_length;
  pass_intersection_planner_input->pi_params_.vehicle_width =
      generator_cfg_->vehicle_width;
  pass_intersection_planner_input->pi_params_.center_to_geometry_center =
      generator_cfg_->center_to_geometry_center;
  pass_intersection_planner_input->pi_params_.line_repulsion_weight =
      generator_cfg_->line_repulsion_weight;
  pass_intersection_planner_input->pi_params_.line_repulsion_max_distance =
      generator_cfg_->line_repulsion_max_distance;
  pass_intersection_planner_input->pi_params_.line_repulsion_2_weight =
      generator_cfg_->line_repulsion_2_weight;
  pass_intersection_planner_input->pi_params_.line_repulsion_2_attenuation =
      generator_cfg_->line_repulsion_2_attenuation;
  pass_intersection_planner_input->pi_params_.direction_field_intensity =
      generator_cfg_->direction_field_intensity;
  pass_intersection_planner_input->pi_params_.direction_field_intensity_l = 0.0;
  pass_intersection_planner_input->pi_params_.back_extend_points_num =
      generator_cfg_->back_extend_points_num;
  pass_intersection_planner_input->pi_params_.thin_out_angle_accuracy =
      generator_cfg_->thin_out_angle_accuracy;
  pass_intersection_planner_input->pi_params_.thin_out_max_length =
      generator_cfg_->thin_out_max_length;
  pass_intersection_planner_input->pi_params_.stop_gradient_value =
      generator_cfg_->stop_gradient_value;
  pass_intersection_planner_input->pi_params_.max_length =
      generator_cfg_->max_length;
  pass_intersection_planner_input->pi_params_.min_length =
      generator_cfg_->min_length;
  pass_intersection_planner_input->pi_params_.lane_select_score =
      generator_cfg_->lane_select_score;
  pass_intersection_planner_input->pi_params_.max_look_ahead_distance =
      generator_cfg_->max_look_ahead_distance;
  pass_intersection_planner_input->pi_params_.min_look_ahead_distance =
      generator_cfg_->min_look_ahead_distance;
  pass_intersection_planner_input->pi_params_.look_ahead_time =
      generator_cfg_->look_ahead_time;
  pass_intersection_planner_input->pi_params_.max_leader_distance =
      generator_cfg_->max_leader_distance;
  pass_intersection_planner_input->pi_params_.min_leader_distance =
      generator_cfg_->min_leader_distance;
  pass_intersection_planner_input->pi_params_.max_yaw_diff =
      generator_cfg_->max_yaw_diff;
  pass_intersection_planner_input->pi_params_.max_angle_diff =
      generator_cfg_->max_angle_diff;
  pass_intersection_planner_input->pi_params_.use_agnle_diff_ratio =
      generator_cfg_->use_agnle_diff_ratio;
  pass_intersection_planner_input->pi_params_.max_l_tolerance =
      generator_cfg_->max_l_tolerance;
  pass_intersection_planner_input->pi_params_.discard_time =
      generator_cfg_->muti_leader_discard_time;
  pass_intersection_planner_input->pi_params_.max_historical_info_length =
      generator_cfg_->max_historical_info_length;
  pass_intersection_planner_input->pi_params_.traj_select_s_tolerance =
      generator_cfg_->traj_select_s_tolerance;
  pass_intersection_planner_input->pi_params_.traj_select_l_tolerance =
      generator_cfg_->traj_select_l_tolerance;
  pass_intersection_planner_input->pi_params_.traj_select_yaw_tolerance =
      generator_cfg_->traj_select_yaw_tolerance;
  pass_intersection_planner_input->pi_params_.refline_real_length =
      generator_cfg_->refline_real_length;
  pass_intersection_planner_input->pi_params_.quit_cp_lat_dis_in_inter =
      generator_cfg_->quit_cp_lat_dis_in_inter;
  pass_intersection_planner_input->pi_params_.quit_cp_lat_dis_refline =
      generator_cfg_->quit_cp_lat_dis_refline;
  pass_intersection_planner_input->pi_params_.quit_cp_consider_refline_length =
      generator_cfg_->quit_cp_consider_refline_length;
  pass_intersection_planner_input->pi_params_.use_quintic =
      generator_cfg_->use_quintic;
  pass_intersection_planner_input->pi_params_.farthest_index =
      generator_cfg_->farthest_index;
  pass_intersection_planner_input->pi_params_.replan_threshold = 1.0;
  pass_intersection_planner_input->pi_params_.zero_speed_threshold = 0.1;
  pass_intersection_planner_input->pi_params_.coord_transform_precision = 0.01;
  pass_intersection_planner_input->pi_params_.step_s = 0.3;
  pass_intersection_planner_input->pi_params_.coarse_step_s = 1.0;
  pass_intersection_planner_input->pi_params_.optimization_gamma = 0.5;
  pass_intersection_planner_input->pi_params_.max_iter = 15;
}

void PassIntersectionPlannerPreprocessor::set_ego_pose() {
  const auto &pass_intersection_planner_input =
      PlanningContext::Instance()->mutable_pass_intersection_planner_input();
  auto &ego_pose = pass_intersection_planner_input->ego_pose_;
  auto &ego_state_manager = world_model_->get_cart_ego_state_manager();
  auto &ego_state = ego_state_manager.get_cart_ego_state();
  ego_pose.x = ego_state.ego_pose.x;
  ego_pose.y = ego_state.ego_pose.y;
  ego_pose.theta = ego_state.ego_pose.theta;
  ego_pose.vel = ego_state.ego_vel;
  return;
}

void PassIntersectionPlannerPreprocessor::set_reflinepoints() {
  const auto &pass_intersection_planner_input =
      PlanningContext::Instance()->mutable_pass_intersection_planner_input();
  auto &refline_points_ = pass_intersection_planner_input->refline_points_;
  refline_points_.clear();
  auto &current_lane_index_ =
      pass_intersection_planner_input->current_lane_index_;
  auto &lane_size_ = pass_intersection_planner_input->lane_size_;
  auto &current_last_car_point_x_ =
      pass_intersection_planner_input->current_last_car_point_x_;

  auto &map_info = world_model_->get_map_info();

  current_lane_index_ = map_info.current_lane_index();
  lane_size_ = map_info.lane_.size();
  current_last_car_point_x_ = world_model_->get_current_last_car_point_x();

  if (map_info.current_lane_index() >= map_info.lane_.size()) {
    return;
  }

  for (auto p : map_info.lane_[map_info.current_lane_index()]
                    .reference_line.reference_line_points) {
    Traj_Point tmp_point;
    tmp_point.x = p.enu_point.x;
    tmp_point.y = p.enu_point.y;
    refline_points_.push_back(tmp_point);
  }

  return;
}

void PassIntersectionPlannerPreprocessor::set_target_pts() {
  const auto &pass_intersection_planner_input =
      PlanningContext::Instance()->mutable_pass_intersection_planner_input();
  auto &map_info = world_model_->get_map_info();
  pass_intersection_planner_input->pi_target_pt_exist_ =
      map_info.pi_target_pt_exist_;
  auto &target_pt_ = pass_intersection_planner_input->target_pt_;
  // pass_intersection_planner_input->target_pt_ =
  //     pass_intersection_planner_input->target_pt_last_;
  auto refline_generator = world_model_->get_refline_generator();
  // if (refline_generator->get_target_pt_last().size() > 0) {
  //   pass_intersection_planner_input->target_pt_last_.push_back(refline_generator->get_target_pt_last().at(0));
  //   pass_intersection_planner_input->target_pt_last_.push_back(refline_generator->get_target_pt_last().at(1));
  // } else {
  //   pass_intersection_planner_input->target_pt_last_.clear();
  // }

  target_pt_.clear();

  if (map_info.pi_target_pt_exist_) {
    pass_intersection_planner_input->pi_target_pt_exist_ =
        map_info.pi_target_pt_exist_;
    target_pt_ = world_model_->get_map_info().get_pi_target_pt_enu();
    if (target_pt_.size() > 0) {
      // std::cout << "has target pt " << std::endl;
    } else {
    }
  } else {
    pass_intersection_planner_input->pi_target_pt_exist_ = false;
  }
}

void PassIntersectionPlannerPreprocessor::check_intersection_condition() {
  const auto &pass_intersection_planner_input =
      PlanningContext::Instance()->mutable_pass_intersection_planner_input();
  auto &condition_ =
      pass_intersection_planner_input->condition_.reflinecondition;

  auto &map_info = world_model_->get_map_info();
  if (map_info.current_lane_index() >= map_info.lane_.size()) {
    condition_ = ReflineCondition::IN_INTERSECTION;
    return;
  }
  auto &refline_points = map_info.lane_[map_info.current_lane_index()]
                             .reference_line.reference_line_points;
  auto &ego_state =
      world_model_->get_cart_ego_state_manager().get_cart_ego_state();
  double min_ref_point_distance = 1e19;
  double start_displacement, end_displacement;
  planning_math::Vec2d ego_dir_vec(cos(ego_state.ego_pose.theta),
                                   sin(ego_state.ego_pose.theta));
  planning_math::Vec2d ego_postion(ego_state.ego_pose.x, ego_state.ego_pose.y);

  if (refline_points.empty()) {
    condition_ = ReflineCondition::IN_INTERSECTION;
    return;
  }
  for (auto &point : refline_points) {
    planning_math::Vec2d point_position(point.enu_point.x, point.enu_point.y);
    min_ref_point_distance = std::min(min_ref_point_distance,
                                      (point_position - ego_postion).Length());
  }

  bool has_close_ref_point =
      (min_ref_point_distance < close_ref_point_threshold_);

  planning_math::Vec2d start_postion(refline_points.front().enu_point.x,
                                     refline_points.front().enu_point.y);
  planning_math::Vec2d end_postion(refline_points.back().enu_point.x,
                                   refline_points.back().enu_point.y);
  auto start_vec = start_postion - ego_postion;
  auto end_vec = end_postion - ego_postion;
  start_displacement = start_vec.InnerProd(ego_dir_vec);
  end_displacement = end_vec.InnerProd(ego_dir_vec);

  bool has_ahead_ref_point = (start_displacement > 0 || end_displacement > 0);

  bool has_after_ref_point = (start_displacement < 0 || end_displacement < 0);

  bool near_intersection = false;
  if (end_vec.Length() < 3.0) {
    near_intersection = true;
  }

  if (has_after_ref_point && has_ahead_ref_point && !near_intersection) {
    condition_ = ReflineCondition::NO_INTERSECTION;
  } else if ((has_after_ref_point && !has_ahead_ref_point) ||
             near_intersection) {
    condition_ = ReflineCondition::FIRST_HALF_INTERSECTION;
    if (pass_intersection_planner_input->is_in_intersection_cond_) {
      pass_intersection_planner_input->is_in_intersection_cond_ = false;
      pass_intersection_planner_input->in_intersection_ego_theta_ =
          ego_state.ego_pose.theta;
      pass_intersection_planner_input->in_intersection_pos_.push_back(
          ego_state.ego_pose.x);
      pass_intersection_planner_input->in_intersection_pos_.push_back(
          ego_state.ego_pose.y);
    }
  } else if (!has_after_ref_point && has_ahead_ref_point) {
    condition_ = ReflineCondition::LAST_HALF_INTERSECTION;
    pass_intersection_planner_input->hist_refline_.clear();
  } else {
    condition_ = ReflineCondition::UNKNOW;
  }
}

void PassIntersectionPlannerPreprocessor::update_muti_historical_info() {
  const auto &pass_intersection_planner_input =
      PlanningContext::Instance()->mutable_pass_intersection_planner_input();
  auto &muti_historical_info_ =
      pass_intersection_planner_input->muti_historical_info_;
  auto &adc_historical_info_ =
      pass_intersection_planner_input->adc_historical_info_;
  auto &ego_state_manager = world_model_->get_cart_ego_state_manager();
  auto &fusion_info = world_model_->get_fusion_info();
  auto &truncated_prediction_info =
      world_model_->get_truncated_prediction_info();
  auto &ego_state = ego_state_manager.get_cart_ego_state();
  auto generator_cfg_ = ReflineGeneratorConfig::Instance();
  auto enu2car = world_model_->get_cart_ego_state_manager().get_enu2car();

  for (auto &vehicle_info : truncated_prediction_info) {
    if ((int)vehicle_info.type == -1 || (int)vehicle_info.type == 0 ||
        (int)vehicle_info.type == 10001 || (int)vehicle_info.type == 10002 ||
        (int)vehicle_info.type == 20001 || (int)vehicle_info.type == 30001) {
      continue;
    }

    Eigen::Vector3d vehicle_car_point, vehicle_enu_point;
    vehicle_enu_point.x() = vehicle_info.position_x;
    vehicle_enu_point.y() = vehicle_info.position_y;
    vehicle_enu_point.z() = 0.0;
    vehicle_car_point = enu2car * vehicle_enu_point;

    if (std::abs(vehicle_car_point.y()) > 2.5 &&
        pass_intersection_planner_input->condition_.reflinecondition ==
            pass_intersection_planner::ReflineCondition::NO_INTERSECTION) {
      continue;
    } else if (std::abs(vehicle_car_point.y()) > 5.0 ||
               vehicle_car_point.x() < -10.0) {
      continue;
    }

    planning_math::Vec2d position_vec(
        vehicle_info.position_x - ego_state.ego_pose.x,
        vehicle_info.position_y - ego_state.ego_pose.y);
    double angle_diff = planning_math::NormalizeAngle(position_vec.Angle() -
                                                      ego_state.ego_pose.theta);
    double yaw_diff = planning_math::NormalizeAngle(vehicle_info.yaw -
                                                    ego_state.ego_pose.theta);
    pass_intersection_planner::SimpleLeaderInfo new_info(
        vehicle_info.id, vehicle_info.position_x, vehicle_info.position_y,
        vehicle_info.speed, vehicle_info.acc, vehicle_info.yaw, yaw_diff,
        angle_diff);
    auto iter = std::find_if(
        muti_historical_info_.begin(), muti_historical_info_.end(),
        [vehicle_info](
            pass_intersection_planner::HistoricalLeaderInfo historical) {
          return vehicle_info.id == historical.leader_id();
        });
    if (iter != muti_historical_info_.end()) {
      double neighbor_pos_dis = 0.0;
      if (iter->infos().size() > 0)
        neighbor_pos_dis = std::sqrt(
            (vehicle_info.position_x - iter->infos().back().position_x_) *
                (vehicle_info.position_x - iter->infos().back().position_x_) +
            (vehicle_info.position_y - iter->infos().back().position_x_) *
                (vehicle_info.position_y - iter->infos().back().position_y_));
      if (neighbor_pos_dis < 0.3) {
        continue;
      }
      iter->add(new_info);
      iter->now_car_x_ = vehicle_car_point.x();
      iter->now_car_y_ = vehicle_car_point.y();
    } else {
      pass_intersection_planner::HistoricalLeaderInfo new_leader(
          vehicle_info.id);
      new_leader.add(new_info);
      new_leader.now_car_x_ = vehicle_car_point.x();
      new_leader.now_car_y_ = vehicle_car_point.y();
      muti_historical_info_.push_back(new_leader);
    }
  }

  // double time_now = MTIME()->timestamp().sec();
  // auto it = muti_historical_info_.begin();
  // while (it != muti_historical_info_.end()) {
  //   if ((time_now - it->update_time()) >
  //       generator_cfg_->muti_leader_discard_time) {
  //     it = muti_historical_info_.erase(it);
  //   } else {
  //     it++;
  //   }
  // }

  auto item = muti_historical_info_.begin();
  while (item != muti_historical_info_.end()) {
    if (std::abs(item->now_car_y_) > 2.5 &&
        pass_intersection_planner_input->condition_.reflinecondition ==
            pass_intersection_planner::ReflineCondition::NO_INTERSECTION) {
      item = muti_historical_info_.erase(item);
    } else {
      item++;
    }
  }

  for (auto &historical_info : muti_historical_info_) {
    auto &mutable_infos = historical_info.mutable_infos();
    double vehicle_traj_length = 0;
    for (int i = (int)mutable_infos.size() - 1; i > 0; i--) {
      vehicle_traj_length +=
          std::sqrt((mutable_infos.at(i).position_x_ -
                     mutable_infos.at(i - 1).position_x_) *
                        (mutable_infos.at(i).position_x_ -
                         mutable_infos.at(i - 1).position_x_) +
                    (mutable_infos.at(i).position_y_ -
                     mutable_infos.at(i - 1).position_y_) *
                        (mutable_infos.at(i).position_y_ -
                         mutable_infos.at(i - 1).position_y_));
    }
    if (mutable_infos.size() > generator_cfg_->max_historical_info_length) {
      mutable_infos.erase(
          mutable_infos.begin(),
          mutable_infos.begin() +
              std::max(0, int(mutable_infos.size() -
                              generator_cfg_->max_historical_info_length)));
    }
  }

  // update adc historicla info
  Traj_Point adc_position_vec{ego_state.ego_pose.x, ego_state.ego_pose.y, 0.0};
  adc_historical_info_.push_back(adc_position_vec);
  if (adc_historical_info_.size() >
      generator_cfg_->max_historical_info_length) {
    adc_historical_info_.erase(
        adc_historical_info_.begin(),
        adc_historical_info_.begin() +
            std::max(0, (int)adc_historical_info_.size() -
                            generator_cfg_->max_historical_info_length));
  }
}

void PassIntersectionPlannerPreprocessor::save_hist_refline() {
  const auto &pass_intersection_planner_input =
      PlanningContext::Instance()->mutable_pass_intersection_planner_input();
  auto &hist_refline_ = pass_intersection_planner_input->hist_refline_;

  auto generator_cfg_ = ReflineGeneratorConfig::Instance();
  auto &map_info = world_model_->get_map_info();
  auto &ego_state_manager = world_model_->get_cart_ego_state_manager();
  auto &ego_state = ego_state_manager.get_cart_ego_state();
  if (map_info.current_lane_index() >= map_info.lane_.size()) {
    return;
  }
  auto &refline_points = map_info.lane_[map_info.current_lane_index()]
                             .reference_line.reference_line_points;
  auto *planner_debug = PlanningContext::Instance()->mutable_planner_debug();
  planner_debug->apf_debug_info.origin_refline_points.clear();
  for (const auto p : refline_points) {
    std::vector<double> x_y_tmp;
    x_y_tmp.push_back(p.enu_point.x);
    x_y_tmp.push_back(p.enu_point.y);
    planner_debug->apf_debug_info.origin_refline_points.push_back(x_y_tmp);
  }
  if (refline_points.empty()) {
    return;
  }

  double dis_near_car_min = 1000.0;
  int near_car_point_index = -1;
  for (int i = 0; i < (int)refline_points.size() - 1; i++) {
    double near_dis_tmp = std::sqrt(
        (refline_points.at(i).enu_point.x - ego_state.ego_pose.x) *
            (refline_points.at(i).enu_point.x - ego_state.ego_pose.x) +
        (refline_points.at(i).enu_point.y - ego_state.ego_pose.y) *
            (refline_points.at(i).enu_point.y - ego_state.ego_pose.y));
    if (near_dis_tmp < dis_near_car_min) {
      near_car_point_index = i;
      dis_near_car_min = near_dis_tmp;
    }
  }
  if (dis_near_car_min < 1.0 && near_car_point_index > 0) {
    Traj_Point refpoint;
    refpoint.x = refline_points[near_car_point_index].enu_point.x;
    refpoint.y = refline_points[near_car_point_index].enu_point.y;
    double near_dis = 1000.0;
    if (hist_refline_.size() > 0) {
      near_dis = std::sqrt((refpoint.x - hist_refline_.back().x) *
                               (refpoint.x - hist_refline_.back().x) +
                           (refpoint.y - hist_refline_.back().y) *
                               (refpoint.y - hist_refline_.back().y));
    }
    if (near_dis < 0.5) {

    } else {
      hist_refline_.push_back(refpoint);
    }
  }

  double dis = 0;
  int index = -1;
  for (int i = (int)hist_refline_.size() - 1; i > 0; i--) {
    dis += std::sqrt((hist_refline_[i].x - hist_refline_[i - 1].x) *
                         (hist_refline_[i].x - hist_refline_[i - 1].x) +
                     (hist_refline_[i].y - hist_refline_[i - 1].y) *
                         (hist_refline_[i].y - hist_refline_[i - 1].y));
    if (dis > generator_cfg_->refline_real_length) { // 30.0
      index = i;
      break;
    }
  }
  if (index > 0) {
    hist_refline_.erase(hist_refline_.begin(),
                        hist_refline_.begin() + index - 1);
  }
}

bool PassIntersectionPlannerPreprocessor::calc_lane_perception_obstacle() {
  const auto &pass_intersection_planner_input =
      PlanningContext::Instance()->mutable_pass_intersection_planner_input();
  auto &lanes_ = pass_intersection_planner_input->lanes_;
  auto &road_edges_ = pass_intersection_planner_input->road_edges_;
  auto &cur_lane_left_ = pass_intersection_planner_input->cur_lane_left_;
  auto &cur_lane_right_ = pass_intersection_planner_input->cur_lane_right_;
  lanes_.clear();
  road_edges_.clear();
  cur_lane_left_.clear();
  cur_lane_right_.clear();
  auto &lane_road_edges_avail_ =
      pass_intersection_planner_input->lane_road_edges_avail_;

  auto &lane_perception =
      world_model_->get_perception_vision_lane().lane_perception;
  auto &road_edge_perception =
      world_model_->get_perception_vision_lane().road_edge_perception;
  auto car2enu = world_model_->get_cart_ego_state_manager().get_car2enu();
  auto generator_cfg_ = ReflineGeneratorConfig::Instance();
  // auto *planner_debug =
  // PlanningContext::Instance()->mutable_planner_debug();
  if ((!lane_perception.available) || !(road_edge_perception.available)) {
    lane_road_edges_avail_ = false;
    return false;
  } else {
    lane_road_edges_avail_ = true;
  }
  for (auto &lane : lane_perception.lanes) {
    if (!lane.is_centerline && !lane.is_horizontal_line &&
        lane.score > generator_cfg_->lane_select_score &&
        lane.camera_source.value == CameraSourceEnum::CAMERA_SOURCE_FRONT_MID) {
      (void)divide_lane_to_linesegment(lane, car2enu,
                                       generator_cfg_->thin_out_angle_accuracy,
                                       generator_cfg_->thin_out_max_length,
                                       lanes_, cur_lane_left_, cur_lane_right_);
    }
  }

  for (auto &road_edge : road_edge_perception.road_edges) {
    if (road_edge.camera_source.value !=
        CameraSourceEnum::CAMERA_SOURCE_FRONT_MID) {
      continue;
    }
    (void)divide_road_edge_to_linesegment(
        road_edge, car2enu, generator_cfg_->thin_out_angle_accuracy,
        generator_cfg_->thin_out_max_length, road_edges_);
  }
  return true;
}

bool divide_lane_to_linesegment(const maf_perception_interface::Lane &lane,
                                Transform &car2enu, double angle_accuracy,
                                double max_segment_length,
                                std::vector<std::vector<Traj_Point>> &lanes_,
                                std::vector<Traj_Point> &cur_lane_left_,
                                std::vector<Traj_Point> &cur_lane_right_) {

  std::vector<Traj_Point> points;
  if ((!lane.points_3d_x.empty()) && (lane.points_3d_x.size() >= 2)) {
    for (int i = 0; i < lane.points_3d_x.size(); i++) {
      Eigen::Vector3d car_point, enu_point;
      Eigen::Vector3d test_car_point, test_enu_point;
      // convert from rear axle to head
      car_point.x() =
          lane.points_3d_x[i] -
          (ConfigurationContext::Instance()->get_vehicle_param().length -
           ConfigurationContext::Instance()
               ->get_vehicle_param()
               .rear_bumper_to_rear_axle);
      car_point.y() = lane.points_3d_y[i];
      car_point.z() = 0.0;
      Traj_Point lane_car_pt_tmp{car_point.x(), car_point.y(), 0, 0};
      if (lane.index == 1) {
        cur_lane_right_.push_back(lane_car_pt_tmp);
      } else if (lane.index == -1) {
        cur_lane_left_.push_back(lane_car_pt_tmp);
      }
      enu_point = car2enu * car_point;
      Traj_Point tmp;
      tmp.x = enu_point.x();
      tmp.y = enu_point.y();
      points.emplace_back(tmp);
    }
    lanes_.push_back(points);
  } else {
    return false;
  }
  return true;
}

bool divide_road_edge_to_linesegment(
    const maf_perception_interface::RoadEdge &road_edge, Transform &car2enu,
    double angle_accuracy, double max_segment_length,
    std::vector<std::vector<Traj_Point>> &road_edges_) {
  std::vector<Traj_Point> points;
  if ((!road_edge.points_3d_x.empty()) && (road_edge.points_3d_x.size() >= 2)) {
    for (int i = 0; i < road_edge.points_3d_x.size(); i++) {
      Eigen::Vector3d car_point, enu_point;
      // convert from rear axle to head
      car_point.x() =
          road_edge.points_3d_x[i] -
          (ConfigurationContext::Instance()->get_vehicle_param().length -
           ConfigurationContext::Instance()
               ->get_vehicle_param()
               .rear_bumper_to_rear_axle);
      car_point.y() = road_edge.points_3d_y[i];
      car_point.z() = 0.0;
      enu_point = car2enu * car_point;
      Traj_Point tmp;
      tmp.x = enu_point.x();
      tmp.y = enu_point.y();

      points.emplace_back(tmp);
    }
    road_edges_.push_back(points);
  } else {
    return false;
  }
  return true;
}

double PassIntersectionPlannerPreprocessor::distance(Pose2D pose1,
                                                     Pose2D pose2) {
  double dis = std::sqrt((pose1.x - pose2.x) * (pose1.x - pose2.x) +
                         (pose1.y - pose2.y) * (pose1.y - pose2.y));
  return dis;
}

} // namespace msquare
