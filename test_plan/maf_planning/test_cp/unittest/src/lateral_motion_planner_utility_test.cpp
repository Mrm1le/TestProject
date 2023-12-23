#include "gtest/gtest.h"
#define private public
#define protected public
#include "planner/motion_planner/lateral_motion_planner_utility.h"
#include "../src/planner/motion_planner/lateral_motion_planner_utility.cpp"
#include <cmath>

namespace msquare {

class LateralMotionPlannerUtilityTest : public ::testing::Test {
public:
  virtual void SetUp() {
    frenet_coord_ = std::make_shared<FrenetCoordinateSystem>();
    scenario_facade_context_ = new ScenarioFacadeContext();
    frenet_parameters_.zero_speed_threshold = 0.1;
    frenet_parameters_.coord_transform_precision = 0.01;
    frenet_parameters_.step_s = 0.3;
    frenet_parameters_.coarse_step_s = 1.0;
    frenet_parameters_.optimization_gamma = 0.5;
    frenet_parameters_.max_iter = 15;

    init_state_ = {12.0, 0.0, 0.0, 0.0};
    std::vector<double> vx, vy;
    for (int i = 0; i < 11; ++i) {
      vx.push_back(0.0);
      vy.push_back(10.0 * i);
    }
    frenet_coord_->Update(vx, vy, frenet_parameters_, 0.0, 100.0);

    world_model_ = std::make_shared<WorldModel>();
    baseline_info_ = std::make_shared<BaseLineInfo>();
    auto &ego_state =
        baseline_info_->ego_state_manager_.get_mutable_ego_state();
    auto &planning_init_point = ego_state.planning_init_point.path_point;
    ego_state.flag_is_replan = false;
    planning_init_point.x = 0.1;
    planning_init_point.y = init_state_[0];
    planning_init_point.theta = M_PI_2;
    planning_init_point.kappa = 0.0;

    maf_perception_interface::PerceptionFusionObjectData perception_obstacle{};
    perception_obstacle.position.x = 2.0;
    perception_obstacle.position.y = 5.0;
    perception_obstacle.heading_yaw = M_PI_2;
    perception_obstacle.shape.length = 5.0;
    perception_obstacle.shape.width = 2.3;
    Obstacle obs(1, perception_obstacle, true);

    auto &obstacle_manager = baseline_info_->mutable_obstacle_manager();
    auto mutable_obs = obstacle_manager.add_obstacle(obs);
    SLBoundary perception_sl{};
    mutable_obs->ComputeSlBoundary(frenet_coord_, &perception_sl);
    mutable_obs->SetPerceptionSlBoundary(perception_sl);

    auto &obstacle_decision_manager =
      scenario_facade_context_->mutable_obstacle_decision_manager();
    ObstacleDecision obs_decision(obs.Id());
    obstacle_decision_manager.add_obstacle_decision(obs_decision);
    ObjectDecisionType nudge_decision;
    auto mutable_nudge_decision = nudge_decision.mutable_nudge();
    mutable_nudge_decision->type = ObjectNudge::LEFT_NUDGE;
    nudge_decision.setDecisionSource("path_decider");
    obstacle_decision_manager.add_lateral_decision(
        "path_decider", 1, nudge_decision);

    PlanningResult &planning_result =
        scenario_facade_context_->mutable_planning_status()->planning_result;

    planning_result.traj_vel_array.reserve(TOTAL_POINT_SIZE);
    planning_result.traj_acceleration.reserve(TOTAL_POINT_SIZE);
    planning_result.traj_pose_array.reserve(TOTAL_POINT_SIZE);
    for (size_t i = 0; i < TOTAL_POINT_SIZE; ++i) {
      maf_planning::VelocityPoint vel_point;
      vel_point.distance = init_state_[0] + i * adc_vel_ * DELTA_T;
      vel_point.relative_time = i * DELTA_T;
      vel_point.target_velocity = 10.0;
      planning_result.traj_vel_array.emplace_back(vel_point);
      planning_result.traj_acceleration.emplace_back(0.0);

      maf_planning::PathPoint path_point;
      path_point.curvature = 0.0;
      path_point.heading_yaw = M_2_PI;
      path_point.path_follow_strength = 0.0;
      path_point.position_enu.x = 0.0;
      path_point.position_enu.y = init_state_[0] + i * adc_vel_ * DELTA_T;
      planning_result.traj_pose_array.emplace_back(path_point);
    }
  }

  std::array<double, 4> init_state_;;
  double adc_vel_ = 10.0;
  const double DELTA_T = 0.025;
  const double TOTAL_TIME_LENGTH = 5.0;
  const size_t TOTAL_POINT_SIZE = size_t(TOTAL_TIME_LENGTH / DELTA_T);
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;
  FrenetCoordinateSystemParameters frenet_parameters_;
  ScenarioFacadeContext *scenario_facade_context_;
  std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<BaseLineInfo> baseline_info_;
};

TEST_F(LateralMotionPlannerUtilityTest, test) {
  path_planner::PathPlannerOutput pre_output;
  path_planner::PathPlannerOutput prev_quintic_poly;
  LateralMotionPlannerUtility lat_motion_planner_utility;
  lat_motion_planner_utility.update_input(
      init_state_, adc_vel_, scenario_facade_context_,
      frenet_coord_, world_model_, baseline_info_, pre_output,
      prev_quintic_poly);
  lat_motion_planner_utility.generate_output();
  const auto &output = scenario_facade_context_->path_planner_input();
  const auto &s_at_control_quad_points =
      lat_motion_planner_utility.s_at_control_quad_points_;
  const size_t NUM_TOTAL_POINTS = s_at_control_quad_points.size();

  EXPECT_DOUBLE_EQ(output.planning_init_state.s, init_state_[0]);
  EXPECT_DOUBLE_EQ(output.planning_init_state.x, 0.1);
  EXPECT_DOUBLE_EQ(output.planning_init_state.y, init_state_[0]);
  EXPECT_DOUBLE_EQ(output.planning_init_state.curvature, 0.0);
  EXPECT_NEAR(output.planning_init_state.dx_ds, 0.0, 1e-6);
  EXPECT_NEAR(output.planning_init_state.dy_ds, 1.0, 1e-6);
  EXPECT_FALSE(output.is_replan);
  EXPECT_DOUBLE_EQ(s_at_control_quad_points[0], init_state_[0]);
  EXPECT_DOUBLE_EQ(s_at_control_quad_points[NUM_TOTAL_POINTS - 1],
                   init_state_[0] +
                       (TOTAL_POINT_SIZE - 1) * DELTA_T * adc_vel_);
  EXPECT_DOUBLE_EQ(output.path_segments.front().end_segment_refline_info.y,
                   output.planning_init_state.y +
                       (s_at_control_quad_points[NUM_TOTAL_POINTS - 1] -
                        s_at_control_quad_points.front()) /
                           4.0);
}

} // namespace msquare
