#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "common/math/math_utils.h"

#include "gtest/gtest.h"

namespace msquare {
  static std::string car_param_file = "/home/ros/catkin_ws/src/maf_planning/resource/"
                              "config/scenario_configs_json/parking/"
                              "vehicle_param.yaml";
  static std::string config_file_name = "/home/ros/catkin_ws/src/maf_planning/"
                                  "resource/config/scenario_configs_json/"
                                  "parking/apa.yaml";

TEST(HybridAstarConfigTest, loadFile) {
  auto ha_cfg = HybridAstarConfig::GetInstance();
  ha_cfg->loadFile(config_file_name);
  EXPECT_TRUE(ha_cfg != nullptr);

  EXPECT_TRUE(ha_cfg->is_loaded);
  EXPECT_TRUE(HybridAstarConfig::GetInstance()->use_t_line_);
  EXPECT_EQ(HybridAstarConfig::GetInstance()->max_zigzag_allowd, 7);
  EXPECT_TRUE(HybridAstarConfig::GetInstance()->enable_delta_cost);
  EXPECT_FALSE(HybridAstarConfig::GetInstance()->enable_analytic_expansion);
  EXPECT_LT(HybridAstarConfig::GetInstance()->max_analytic_expansion_length, 0);
  EXPECT_DOUBLE_EQ(HybridAstarConfig::GetInstance()->analytic_expansion_end_size_threshold, 0.1);
  EXPECT_EQ(HybridAstarConfig::GetInstance()->max_iter, HybridAstarConfig::GetInstance()->max_iter_base);
}

TEST(TrajectoryOptimizerConfigTest, loadFile) {
  auto to_cfg = TrajectoryOptimizerConfig::GetInstance();
  EXPECT_TRUE(to_cfg != nullptr);

  to_cfg->loadFile(config_file_name);
  EXPECT_TRUE(to_cfg->is_loaded);
  EXPECT_FALSE(to_cfg->OBCA_running);
  EXPECT_TRUE(to_cfg->param_FLAGS_use_dual_variable_warm_start);
  EXPECT_EQ(to_cfg->param_is_near_destination_threshold, 0.1);
  
  EXPECT_LT(to_cfg->param_min_safe_dist, 1e-3);
  EXPECT_DOUBLE_EQ(to_cfg->param_max_steer_angle, 0.51);
  EXPECT_STREQ(to_cfg->param_linear_solver.c_str(), "ma57");
}

TEST(CarParamsTest, loadFile) {
  auto car_prm = CarParams::GetInstance();
  car_prm->loadFile(car_param_file);
  car_prm->loadFile4Plan(config_file_name);
  EXPECT_TRUE(car_prm != nullptr);

  EXPECT_NEAR(car_prm->vehicle_width_wo_rearview_mirror, 1.89, 1e-5);
  EXPECT_NEAR(car_prm->lat_inflation_max, 0.22, 1e-5);
  EXPECT_NEAR(car_prm->lon_inflation_min, 0.4, 1e-6);
  EXPECT_DOUBLE_EQ(car_prm->max_steer_angle, 445);
  EXPECT_FALSE(car_prm->enable_multiple_steer_modes);

  double vehicle_width = car_prm->vehicle_width_wo_rearview_mirror + 2 * car_prm->lat_inflation_max;
  double vehicle_length = car_prm->vehicle_length_real + 2 * car_prm->lon_inflation_max;
  EXPECT_NEAR(car_prm->vehicle_width, vehicle_width, 1e-5);
  EXPECT_NEAR(car_prm->vehicle_length, vehicle_length, 1e-5);

  double min_turn_radius = car_prm->wheel_base /
      tan(car_prm->max_steer_angle / car_prm->steer_ratio / 180 * M_PI);
  EXPECT_NEAR(car_prm->min_turn_radius, min_turn_radius, 1e-5);
}

TEST(CarParamsTest, setAndGet) {
  auto car_prm = CarParams::GetInstance();
  car_prm->loadFile(car_param_file);
  car_prm->loadFile4Plan(config_file_name);

  double origin_lat_inflation = car_prm->lat_inflation_max;
  double origin_lon_inflation = car_prm->lon_inflation_max;

  double new_lat_inflation = 0.3;
  double new_lon_inflation = 0.4;
  EXPECT_FALSE(car_prm->setLatInflation(new_lat_inflation));
  EXPECT_FALSE(car_prm->setLonInflation(new_lon_inflation));

  car_prm->resetInflation();
  double vehicle_width = car_prm->vehicle_width_wo_rearview_mirror + 2 * origin_lat_inflation;
  double vehicle_length = car_prm->vehicle_length_real + 2 * origin_lon_inflation;
  EXPECT_NEAR(car_prm->vehicle_width, vehicle_width, 1e-5);
  EXPECT_NEAR(car_prm->vehicle_length, vehicle_length, 1e-5);

  double max_steer = 425.0, max_steer_angle_rate = 380;
  double max_steer_angle_rear = 50, max_steer_angle_rate_rear = 39.5;
  car_prm->setMaxSteer(max_steer);
  car_prm->setMaxSteerRate(max_steer_angle_rate);
  car_prm->setMaxSteerRear(max_steer_angle_rear);
  car_prm->setMaxSteerRateRear(max_steer_angle_rate_rear);
  EXPECT_DOUBLE_EQ(car_prm->max_steer_angle, 425);
  EXPECT_DOUBLE_EQ(car_prm->max_steer_angle_rate, 380);
  EXPECT_DOUBLE_EQ(car_prm->max_steer_angle_rear, 50);
  EXPECT_DOUBLE_EQ(car_prm->max_steer_angle_rate_rear, 39.5);

  double min_turn_radius = car_prm->wheel_base /
      tan(max_steer / car_prm->steer_ratio / 180 * M_PI);
  EXPECT_NEAR(car_prm->min_turn_radius, min_turn_radius, 1e-5);

}

TEST(CarParamsTest, Box) {
  auto car_prm = CarParams::GetInstance();
  car_prm->loadFile(car_param_file);
  car_prm->loadFile4Plan(config_file_name);

  planning_math::Box2d box = car_prm->getBox(0, 0, 0);
  EXPECT_DOUBLE_EQ(box.length(), car_prm->vehicle_length);
  EXPECT_DOUBLE_EQ(box.width(), car_prm->vehicle_width);

  planning_math::Box2d box_real = car_prm->getBoxReal(0, 0, 0);
  EXPECT_DOUBLE_EQ(box_real.length(), car_prm->vehicle_length_real);
  EXPECT_DOUBLE_EQ(box_real.width(), car_prm->vehicle_width_wo_rearview_mirror);

  planning_math::Box2d box_shrinked = car_prm->getBoxShrinked(0, 0, 0);
  EXPECT_DOUBLE_EQ(box_shrinked.length(), car_prm->vehicle_length_real *
                                              car_prm->shrink_ratio_for_lines_);
  EXPECT_DOUBLE_EQ(box_shrinked.width(),
                   car_prm->vehicle_width_wo_rearview_mirror *
                       car_prm->shrink_ratio_for_lines_);
}

TEST(StrategyParamsTest, loadFile) {
  auto stg_prm = StrategyParams::GetInstance();
  stg_prm->loadFile(config_file_name);

  EXPECT_DOUBLE_EQ(stg_prm->default_v_, 0.5);
  EXPECT_TRUE(stg_prm->bcheckendsegment);
  EXPECT_DOUBLE_EQ(stg_prm->default_check_endsegent_len_, -1.2);
  EXPECT_TRUE(stg_prm->enable_revserse_search_);
  EXPECT_FALSE(stg_prm->enable_smooth_);

  stg_prm->setForceTerminate(false);
  EXPECT_FALSE(stg_prm->getForceTerminate());
}
}
