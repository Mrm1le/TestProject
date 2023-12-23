#include "planner/tasks/obstacle_decider_preprocessor.h"

namespace msquare {

void ObstacleDeciderPreprocessor::process_time_manager() {
  if (call_count_ == 1) {
    odc_prec_created_time_ = MTIME()->timestamp().sec();
  }
  t_buffer_.front() = t_buffer_.back();
  t_buffer_.back() = MTIME()->timestamp().sec();

  double t_delta_tmp = t_buffer_.back() - t_buffer_.front();
  if (abs(t_delta_tmp - 0.1) > 0.2) {
    t_delta_ = 0.1;
  } else {
    t_delta_ = t_delta_tmp;
  }
  MSD_LOG(INFO, "ODC_t_delta: %.3f s", t_delta_);
}

void ObstacleDeciderPreprocessor::update_input_ego_state() {
  using odc_interface::Point2d;
  const auto &ego_state_cart = baseline_info_->get_ego_state();
  const double &veh_set_speed = world_model_->get_map_info().v_cruise();

  obstacle_decider_input_->ego_state_cart.ego_vel = ego_state_cart.ego_vel;

  obstacle_decider_input_->ego_state_cart.ego_steer_angle =
      ego_state_cart.ego_steer_angle;

  obstacle_decider_input_->ego_state_cart.ego_v_cruise = veh_set_speed;

  obstacle_decider_input_->ego_state_cart.ego_acc = ego_state_cart.ego_acc;

  obstacle_decider_input_->ego_state_cart.ego_pose.x =
      ego_state_cart.ego_pose.x;

  obstacle_decider_input_->ego_state_cart.ego_pose.y =
      ego_state_cart.ego_pose.y;

  obstacle_decider_input_->ego_state_cart.ego_pose.theta =
      ego_state_cart.ego_pose.theta;

  Point2d ego_pos_cart{obstacle_decider_input_->ego_state_cart.ego_pose.x,
                       obstacle_decider_input_->ego_state_cart.ego_pose.y};

  Point2d ego_pos_local = env2local_pos(ego_pos_cart);

  obstacle_decider_input_->ego_state = obstacle_decider_input_->ego_state_cart;
  obstacle_decider_input_->ego_state.ego_pose.x = ego_pos_local.x;
  obstacle_decider_input_->ego_state.ego_pose.y = ego_pos_local.y;
  obstacle_decider_input_->ego_state.ego_pose.theta = 0.0;
}

void ObstacleDeciderPreprocessor::process() {

  update_call_count();
  process_time_manager();
  MSD_LOG(INFO, "ODC_preprocessed![%.1fs]",
          MTIME()->timestamp().sec() - odc_prec_created_time_);

  frenet_coord_ = baseline_info_->get_frenet_coord();

  update_input_ego_state();
}

odc_interface::Point2d
ObstacleDeciderPreprocessor::env2local_pos(odc_interface::Point2d point_env) {
  using odc_interface::Point2d;
  // auto &enu2car = world_model_->get_cart_ego_state_manager().get_enu2car();
  auto &enu2car = enu2car_;
  Eigen::Vector3d car_point, enu_point;

  enu_point.x() = point_env.x;
  enu_point.y() = point_env.y;
  enu_point.z() = 0;

  car_point = enu2car * enu_point;

  Point2d point_local{};
  point_local.x = car_point.x();
  point_local.y = car_point.y();

  return point_local;
}

} // namespace msquare