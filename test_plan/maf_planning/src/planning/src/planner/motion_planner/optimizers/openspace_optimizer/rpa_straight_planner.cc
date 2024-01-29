#include "planner/motion_planner/optimizers/openspace_optimizer/rpa_straight_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/local_log.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"

namespace msquare {

RpaStraightPlanner::RpaStraightPlanner(
    const parking::OpenspaceDeciderOutput &odo) {
  LOCAL_LOG(LOCAL_INFO, ""); // empty line
  LOCAL_LOG(LOCAL_INFO, ""); // empty line
  LOCAL_LOG(LOCAL_INFO, "***************************************");
  LOCAL_LOG(LOCAL_INFO, "************ ENTER RpaStraightPlanner ******");
  LOCAL_LOG(LOCAL_INFO, "***************************************");

  fillPara(odo, para_);
  initScenario(odo);
}

void RpaStraightPlanner::fillPara(const parking::OpenspaceDeciderOutput &odo,
                                  clothoid::Parameter &para) {
  para_.width = VehicleParam::Instance()->width_wo_rearview_mirror;
  para_.length = VehicleParam::Instance()->length;
  para_.width_with_rearview = VehicleParam::Instance()->width;
  para_.obs_inflation =
      std::max(HybridAstarConfig::GetInstance()->inflation_for_points_, 0.0);
  para_.lat = std::max(0.0, CarParams::GetInstance()->lat_inflation());
  para_.lon = std::max(0.0, CarParams::GetInstance()->lon_inflation());
  para_.min_radius = CarParams::GetInstance()->min_turn_radius;
  para_.max_steering_angle = CarParams::GetInstance()->max_steer_angle;
  para_.step = msquare::HybridAstarConfig::GetInstance()->step_size;
  para_.wheel_radius = VehicleParam::Instance()->wheel_rolling_radius;
  para_.wheel_base = VehicleParam::Instance()->wheel_base;
  para_.steering_speed = 270;
  para_.steering_angle_ratio = CarParams::GetInstance()->steer_ratio;
  para_.init_steer_angle = std::min(300.0, para_.max_steering_angle);

  /* */
  para_.straight_lat = std::max(0.2, para_.lat);
  para_.straight_lon = std::max(0.3, para_.lon);
  para_.side_safe_distance = 0.3;

  /* define by general */
  para_.max_alpha =
      1 / para_.min_radius /
      (para_.turning_speed * (para_.max_steering_angle / para_.steering_speed));
  para_.radical_alpha = 1 / para_.min_radius /
                        (para_.radical_turning_speed *
                         (para_.max_steering_angle / para_.steering_speed));

  para_.front_to_rear = VehicleParam::Instance()->front_edge_to_center;
  para_.back_to_rear = VehicleParam::Instance()->back_edge_to_center;
  para_.rear_to_center =
      0.5 * std::abs(para_.front_to_rear - para_.back_to_rear);
  para_.virtual_wall_interval = 0.1;
  para_.invalid_lat_offset = 0.6;

  impl_.half_width = 0.5 * para_.width;

  LOCAL_LOG(LOCAL_INFO, "length:%.3f, width:%.3f", para_.length, para_.width);
  LOCAL_LOG(LOCAL_INFO, "alpha:%f min radius:%.3f block direc:%d",
            para_.max_alpha, para_.min_radius, impl_.block_direc);
  LOCAL_LOG(LOCAL_INFO, "lat:%.3f, lon:%.3f, fr:%.3f, br:%.3f, rr:%.3f",
            para_.lat, para_.lon, para_.front_to_rear, para_.back_to_rear,
            para_.rear_to_center);
}

void RpaStraightPlanner::initScenario(
    const parking::OpenspaceDeciderOutput &input) {
  local_frame_pose_ =
      Pose2D(input.init_state.path_point.x, input.init_state.path_point.y,
             input.init_state.path_point.theta);
  init_pose_ =
      Pose2D(input.init_state.path_point.x, input.init_state.path_point.y,
             input.init_state.path_point.theta);
  target_pose_ =
      Pose2D(input.target_state.path_point.x, input.target_state.path_point.y,
             input.target_state.path_point.theta);
  LOCAL_LOG(LOCAL_INFO, "target_pose_ x:%.6f, y:%.6f, z:%.6f", target_pose_.x,
            target_pose_.y, target_pose_.theta);
  LOCAL_LOG(LOCAL_INFO, "init_pose_ x:%.6f, y:%.6f, z:%.6f", init_pose_.x,
            init_pose_.y, init_pose_.theta);

  std::vector<planning_math::LineSegment2d> obs_lines;
  std::vector<planning_math::Vec2d> obs_pts;
  std::vector<planning_math::LineSegment2d> step_lines;
  std::vector<planning_math::Vec2d> step_pts;

  for (auto line : input.obstacle_lines) {
    auto local_line = planning_math::tf2d(local_frame_pose_, line);
    obs_lines.push_back(line);
  }
  for (auto line : obs_lines) {
    obs_pts.push_back(line.start());
    obs_pts.push_back(line.end());
  }
  for (auto point : input.points) {
    obs_pts.push_back(point);
  }
  for (auto line : input.lines) {
    step_lines.push_back(line);
  }
  for (auto obstacle : step_lines) {
    step_pts.push_back(obstacle.start());
    step_pts.push_back(obstacle.end());
  }

  csg_ = clothoid::CollisionShapeGenerator(para_);
  checker_ = clothoid::CollisionChecker(csg_);
  checker_.setData(obs_lines, obs_pts, step_lines, step_pts);
}

bool RpaStraightPlanner::Plan(const std::vector<SbpObstaclePtr> &obs_ptrs,
                              parking::SearchProcessDebug *sp_debug) {
  if (!checkStartPose()) {
    return false;
  }

  clothoid::StraightCurve straight_curve;

  if (!planStraightPath(straight_curve)) {
    LOCAL_LOG(LOCAL_DEBUG, "failed to plan an valid path");
    return false;
  }
  LOCAL_LOG(LOCAL_DEBUG, "successful to plan a path");

  generatePath(straight_curve);
  return true;
}

bool RpaStraightPlanner::planStraightPath(
    clothoid::StraightCurve &straight_curve) {
  double theta_diff = std::abs(init_pose_.theta - target_pose_.theta);
  double s_total =
      std::hypot(init_pose_.x - target_pose_.x, init_pose_.y - target_pose_.y);
  if (theta_diff > 1e-3) {
    LOCAL_LOG(LOCAL_DEBUG, "theta_diff %.6f > 1e-3", theta_diff);
    return false;
  }

  if (s_total < para_.min_block_len) {
    LOCAL_LOG(LOCAL_DEBUG, "s_total %.6f is tiny", s_total);
    return false;
  }

  double cs = std::cos(init_pose_.theta);
  double ss = std::sin(init_pose_.theta);
  double dot_direc = cs * (target_pose_.x - init_pose_.x) +
                     ss * (target_pose_.y - init_pose_.y);
  bool is_forward = dot_direc > 0.0;

  double allowed_dis = 0;
  if (is_forward) {
    allowed_dis = checker_.moveForward(init_pose_, para_.lat, para_.lon,
                                       clothoid::ShapeType::RAW);
    LOCAL_LOG(LOCAL_DEBUG, "moveForward");
  } else {
    LOCAL_LOG(LOCAL_DEBUG, "moveBackward");
    allowed_dis = checker_.moveBackward(init_pose_, para_.lat, para_.lon,
                                        clothoid::ShapeType::RAW);
  }

  if (allowed_dis < para_.min_block_len) {
    LOCAL_LOG(LOCAL_DEBUG, "allowed_dis %.6f is tiny", s_total);
    return false;
  }

  double real_dis = std::min(s_total, allowed_dis);
  LOCAL_LOG(LOCAL_INFO, "s_total: %.6f, allowed_dis: %.6f, real_dis: %.6f",
            s_total, allowed_dis, real_dis);

  straight_curve = clothoid::linspace(
      init_pose_, is_forward ? real_dis : -real_dis, para_.step);
  return true;
}

bool RpaStraightPlanner::checkStartPose() {
  if (!checker_.checkTerminalPose(init_pose_)) {
    LOCAL_LOG(LOCAL_DEBUG, "start pose is not valid.");
    result_.status = SbpStatus::START_INFEASIBLE;
    return false;
  }

  return true;
}

void RpaStraightPlanner::generatePath(
    const clothoid::StraightCurve &straight_curve) {
  result_.clear();
  for (auto pt : straight_curve) {
    result_.x.push_back(pt.x);
    result_.y.push_back(pt.y);
    result_.phi.push_back(pt.theta);
  }
}

void RpaStraightPlanner::Update(
    const planning_math::Box2d map_boundary,
    const std::vector<planning_math::LineSegment2d> &map) {}

SbpResult RpaStraightPlanner::getResult() { return result_; }
std::vector<Pose2D> RpaStraightPlanner::getSearchPoints() {
  return key_points_;
}

} // end namespace msquare