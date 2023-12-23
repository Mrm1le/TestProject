#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "common/sbp_obstacle_point.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/heuristic_cost.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/hybrid_a_star_2.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/tmp_usage_only_global_variables.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/transform.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/util.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/virtual_obstacle_creator.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/sbp_obstacle_interface.h"
#include "planning/common/timer.h"

namespace msquare {

namespace hybrid_a_star_2 {

using namespace planning_math;
using msd_planning::utils::Timer;
using msd_planning::utils::TotalTimer;

#ifdef HASTAR2_HAS_INTERNAL_MODEL
#define MULTI_CIRCLE_MODEL_LOADER(type)                                        \
  [](MultiCircleFootprintModel &model) {                                       \
    extern std::uint8_t start[] asm(                                           \
        "_binary_car_configs_multi_circle_footprint_model_" #type              \
        "_yaml_start");                                                        \
    extern std::uint8_t end[] asm(                                             \
        "_binary_car_configs_multi_circle_footprint_model_" #type              \
        "_yaml_end");                                                          \
    model.loadFromContent(std::string(start, end));                            \
  }
#else // HASTAR2_HAS_INTERNAL_MODEL
#define MULTI_CIRCLE_MODEL_LOADER(type)                                        \
  [config_folder](MultiCircleFootprintModel &model) {                          \
    model.loadFromFile(config_folder +                                         \
                       "car_configs/multi_circle_footprint_model_" #type       \
                       ".yaml");                                               \
  }
#endif // HASTAR2_HAS_INTERNAL_MODEL

#ifdef HASTAR2_HAS_INTERNAL_MODEL
#define HEURISTIC_COST_LOADER(cost)                                            \
  [](HeuristicCost &heuristic_cost) {                                          \
    extern std::uint8_t start[] asm(                                           \
        "_binary_heuristic_offline_gear_switch_cost_" #cost "_bin_start");     \
    extern std::uint8_t end[] asm(                                             \
        "_binary_heuristic_offline_gear_switch_cost_" #cost "_bin_end");       \
    heuristic_cost.loadContent(std::string(start, end));                       \
  }
#else // HASTAR2_HAS_INTERNAL_MODEL
#define HEURISTIC_COST_LOADER(cost)                                            \
  [config_folder](HeuristicCost &heuristic_cost) {                             \
    heuristic_cost.loadFile(                                                   \
        config_folder + "heuristic_offline/gear_switch_cost_" #cost ".bin");   \
  }
#endif // HASTAR2_HAS_INTERNAL_MODEL

HybridAstar2::HybridAstar2(const std::string &config_folder)
    : nodes_pool_(500000 + nodes_pool_extra_size_), simple_rs_nodes_pool_(1000),
      state_map_cost_func_(nodes_pool_),
      state_map_coarse_(state_map_cost_func_, nodes_pool_.invalidKey(),
                        {4, 4, 5}, 200),
      state_map_fine_end_(state_map_cost_func_, nodes_pool_.invalidKey(),
                          {4, 4, 5}, 70),
      pq_cost_func_(nodes_pool_), pq_(pq_cost_func_, 0.01f, 60000, 3000, 100),
      slot_type_(SlotType::VERTICAL), state_expansion_(),
      simple_rs_curve_pool_(300), heuristic_cost_from_inside_(1024 * 1024) {
  searchPoints_.clear();
  obs_grid_.init(1200, 1200, 0.025f, -15.0f, -15.0f);

  vehicle_to_config_["C03"] = {MULTI_CIRCLE_MODEL_LOADER(wls_c03), 400.0f,
                               0.05f};
  vehicle_to_config_["SG"] = {MULTI_CIRCLE_MODEL_LOADER(lhs_sg), 450.0f, 0.05f};
  vehicle_to_config_["UXE"] = {MULTI_CIRCLE_MODEL_LOADER(lhs_uxe), 450.0f,
                               0.0f};
  vehicle_to_config_["LS7"] = {MULTI_CIRCLE_MODEL_LOADER(lbcar), 450.0f, 0.05f};
  vehicle_to_config_["L7"] = {MULTI_CIRCLE_MODEL_LOADER(lacar), 427.0f, 0.05f};
  vehicle_to_config_["A02"] = {MULTI_CIRCLE_MODEL_LOADER(bys_a02), 450.0f,
                               0.05f};
  vehicle_to_config_["S450L"] = {MULTI_CIRCLE_MODEL_LOADER(harz_s450l), 427.0f,
                                 0.0f};
  vehicle_to_config_["LYRIQ"] = {MULTI_CIRCLE_MODEL_LOADER(rocky_lyriq), 400.0f,
                                 0.05f};
  vehicle_to_config_["ESEA"] = {MULTI_CIRCLE_MODEL_LOADER(lhs_esea), 450.0f,
                                0.0f};
  vehicle_to_config_["LC6"] = {MULTI_CIRCLE_MODEL_LOADER(lccar), 427.0f, 0.05f};

  HEURISTIC_COST_LOADER(1)(heuristic_cost_[1.0f]);
  HEURISTIC_COST_LOADER(3)(heuristic_cost_[3.0f]);
  HEURISTIC_COST_LOADER(5)(heuristic_cost_[5.0f]);
  HEURISTIC_COST_LOADER(7)(heuristic_cost_[7.0f]);
  HEURISTIC_COST_LOADER(10)(heuristic_cost_[10.0f]);

#ifdef HASTAR2_HAS_INTERNAL_MODEL
  extern std::uint8_t start[] asm(
      "_binary_heuristic_offline_boundary_cost_bin_start");
  extern std::uint8_t end[] asm(
      "_binary_heuristic_offline_boundary_cost_bin_end");
  boundary_cost_.loadFromContent(std::string(start, end));
#else  // HASTAR2_HAS_INTERNAL_MODEL
  boundary_cost_.loadFromFile(config_folder +
                              "heuristic_offline/boundary_cost.bin");
#endif // HASTAR2_HAS_INTERNAL_MODEL

  simple_rs_methods_for_slot_type_.resize(std::size_t(SlotType::TOTAL_COUNT));
  simple_rs_methods_for_slot_type_[std::size_t(SlotType::VERTICAL)] = {
      &HybridAstar2::simpleRSAnalyticExpansionEulerSpiral};
  simple_rs_methods_for_slot_type_[std::size_t(SlotType::OBLIQUE)] = {
      &HybridAstar2::simpleRSAnalyticExpansionEulerSpiral};
  simple_rs_methods_for_slot_type_[std::size_t(SlotType::PARALLEL)] = {
      &HybridAstar2::simpleRSAnalyticExpansionEulerSpiral,
      &HybridAstar2::simpleRSAnalyticExpansionArc,
      &HybridAstar2::simpleRSAnalyticExpansionLineOnArc};
  simple_rs_methods_for_slot_type_[std::size_t(SlotType::VERTICAL_OUT)] = {
      &HybridAstar2::simpleRSAnalyticExpansionEulerSpiral,
      &HybridAstar2::simpleRSAnalyticExpansionLineOnPoint,
      &HybridAstar2::simpleRSAnalyticExpansionDirect};
  simple_rs_methods_for_slot_type_[std::size_t(SlotType::OBLIQUE_OUT)] = {
      &HybridAstar2::simpleRSAnalyticExpansionEulerSpiral,
      &HybridAstar2::simpleRSAnalyticExpansionLineOnPoint,
      &HybridAstar2::simpleRSAnalyticExpansionDirect};
  simple_rs_methods_for_slot_type_[std::size_t(SlotType::PARALLEL_OUT)] = {
      &HybridAstar2::simpleRSAnalyticExpansionEulerSpiral,
      &HybridAstar2::simpleRSAnalyticExpansionDirect};

  updateConfig();
}

void HybridAstar2::updateConfig() {
  config_valid_ = false;
  const HybridAstarConfig &config = *HybridAstarConfig::GetInstance();
  xy_grid_resolution_ = config.xy_grid_resolution;
  phi_grid_resolution_ = config.phi_grid_resolution;
  float wheel_base = CarParams::GetInstance()->wheel_base;
  max_iter_base_ = config.max_iter_base;
  max_iter_max_ = config.max_iter_max;
  lon_inflation_ = CarParams::GetInstance()->lon_inflation();
  lat_inflation_ = CarParams::GetInstance()->lat_inflation();
  lat_inflation_low_ = CarParams::GetInstance()->lat_inflation_low;
  vehicle_length_real_ = CarParams::GetInstance()->vehicle_length_real;
  max_zigzag_allowd_ = config.max_zigzag_allowd;

  const std::string &vehicle_type = CarParams::GetInstance()->type;

  float deg2rad = float(M_PI) / 180.0f;
  float rad2deg = 180.0f / float(M_PI);

  max_steer_rate_ = 0.0f;
  float min_turn_radius_buffer = 0.0f;
  if (mc_footprint_model_.vehicleType() != vehicle_type) {
    mc_footprint_model_.deInit();
  }
  if (vehicle_to_config_.find(vehicle_type) != vehicle_to_config_.end()) {
    const auto &config = vehicle_to_config_[vehicle_type];
    max_steer_rate_ = config.max_steer_rate * deg2rad;
    min_turn_radius_buffer = config.min_turn_radius_buffer;
    if (!mc_footprint_model_.inited()) {
      config.mc_model_loader(mc_footprint_model_);
    }
  }
  if (max_steer_rate_ == 0.0f) {
    return;
  }

  min_turn_radius_ =
      (1.0f + min_turn_radius_buffer) * wheel_base /
      std::tan(CarParams::GetInstance()->max_delta_angle * deg2rad);

  float max_curvature_rate =
      max_steer_rate_ / (CarParams::GetInstance()->steer_ratio * wheel_base);
  float max_speed_buffer = 0.1f;
  float max_speed_forward =
      TrajectoryOptimizerConfig::GetInstance()->param_max_speed_forward +
      max_speed_buffer;
  float max_speed_backward =
      TrajectoryOptimizerConfig::GetInstance()->param_max_speed_reverse +
      max_speed_buffer;
  simple_rs_min_spiral_scale_forward_ =
      calcSpiralScale(max_speed_forward, max_curvature_rate);
  simple_rs_min_spiral_scale_backward_ =
      calcSpiralScale(max_speed_backward, max_curvature_rate);
  spiral_straight_max_t_ = std::sqrt(
      TmpGlobals::CONFIG_SIMPLE_RS_EULER_SPIRAL_STRAIGHT_DIRECTION_TH *
      deg2rad);
  min_length_per_segment_ =
      CarParams::GetInstance()->car_config.common_config.min_block_len + 1e-5f;
  min_length_per_output_point_ = 1e-3f;
  simple_rs_arc_max_arc_len_ = TmpGlobals::CONFIG_SIMPLE_RS_ARC_MAX_ARC_LENGTH;
  simple_rs_euler_spiral_max_arc_len_ = 10.0f;

  nodes_pool_.expand(max_iter_max_ + nodes_pool_extra_size_);

  if (slot_type_ == SlotType::PARALLEL ||
      slot_type_ == SlotType::PARALLEL_OUT) {
    plan_in_slot_x_min_ = -0.6f;
    plan_in_slot_x_max_ = 0.6f;
    plan_in_slot_y_th_ = 0.5f;
  } else {
    plan_in_slot_x_min_ = -1.0f;
    plan_in_slot_x_max_ = 3.0f;
    plan_in_slot_y_th_ = 1.0f;
  }

  lat_inflation_add_parallel_long_reverse_ = 0.05f;
  lat_inflation_add_slot_entry_ =
      TmpGlobals::CONFIG_LAT_INFLATION_ADD_SLOT_ENTRY;
  lat_inflation_add_out_slot_ = TmpGlobals::CONFIG_LAT_INFLATION_ADD_OUT_SLOT;
  lat_inflation_add_in_slot_x_min_ =
      TmpGlobals::CONFIG_LAT_INFLATION_ADD_IN_SLOT_X_MIN;
  lat_inflation_add_in_slot_x_max_ =
      TmpGlobals::CONFIG_LAT_INFLATION_ADD_IN_SLOT_X_MAX;
  lat_inflation_add_in_slot_y_th_ =
      TmpGlobals::CONFIG_LAT_INFLATION_ADD_IN_SLOT_Y_TH;
  lat_inflation_add_in_slot_theta_th_ =
      TmpGlobals::CONFIG_LAT_INFLATION_ADD_IN_SLOT_THETA_TH;
  lat_inflation_add_slot_entry_x_min_ =
      TmpGlobals::CONFIG_LAT_INFLATION_ADD_SLOT_ENTRY_X_MIN;
  lat_inflation_add_slot_entry_x_max_ =
      TmpGlobals::CONFIG_LAT_INFLATION_ADD_SLOT_ENTRY_X_MAX;
  lat_inflation_add_slot_entry_y_th_ =
      TmpGlobals::CONFIG_LAT_INFLATION_ADD_SLOT_ENTRY_Y_TH;
  lat_inflation_add_slot_entry_theta_th_ =
      TmpGlobals::CONFIG_LAT_INFLATION_ADD_SLOT_ENTRY_THETA_TH;

  float max_close_gear_switch_speed = 0.2f;
  min_spiral_scale_close_gear_switch_ =
      calcSpiralScale(max_close_gear_switch_speed, max_curvature_rate);

  node_config_.traj_forward_penalty = config.traj_forward_penalty;
  node_config_.traj_back_penalty = config.traj_back_penalty;
  node_config_.traj_obstacle_distance_2_penalty =
      config.traj_obstacle_distance_2_penalty;
  node_config_.traj_obstacle_distance_3_penalty =
      config.traj_obstacle_distance_3_penalty;
  node_config_.traj_boundary_cost_penalty = config.traj_boundary_cost_penalty;
  if (slot_type_ == SlotType::PARALLEL ||
      slot_type_ == SlotType::PARALLEL_OUT) {
    node_config_.traj_steer_penalty = 0.0f;
    node_config_.traj_steer_change_penalty_gear_switch = 0.0f;
    node_config_.traj_steer_change_penalty = 0.0f;
    node_config_.traj_s_turn_penalty = 0.0f;
    node_config_.traj_obstacle_distance_1_penalty = 0.0f;
  } else if (slot_type_ == SlotType::VERTICAL_OUT ||
             slot_type_ == SlotType::OBLIQUE_OUT) {
    node_config_.traj_steer_penalty = config.traj_steer_penalty * rad2deg;
    node_config_.traj_steer_change_penalty_gear_switch =
        config.traj_steer_change_penalty_gear_switch * rad2deg;
    node_config_.traj_steer_change_penalty =
        config.traj_steer_change_penalty * rad2deg;
    node_config_.traj_s_turn_penalty = 0.0f;
    node_config_.traj_obstacle_distance_1_penalty = 0.0f;
  } else {
    node_config_.traj_steer_penalty = config.traj_steer_penalty * rad2deg;
    node_config_.traj_steer_change_penalty_gear_switch =
        config.traj_steer_change_penalty_gear_switch * rad2deg;
    node_config_.traj_steer_change_penalty =
        config.traj_steer_change_penalty * rad2deg;
    node_config_.traj_s_turn_penalty = config.traj_s_turn_penalty;
    node_config_.traj_obstacle_distance_1_penalty =
        config.traj_obstacle_distance_1_penalty;
  }
  if (isParkOut()) {
    node_config_.traj_end_offset_penalty = 0.0f;
  } else {
    node_config_.traj_end_offset_penalty = config.traj_end_offset_penalty;
  }

  curve::Curve::setWheelBase(wheel_base);
  curve::Curve::setMaxResolution(0.01f);
  bool search_max_distance =
      slot_type_ == SlotType::PARALLEL || slot_type_ == SlotType::PARALLEL_OUT;
  no_steer_change_gear_switch_ =
      slot_type_ != SlotType::PARALLEL &&
      slot_type_ != SlotType::PARALLEL_OUT &&
      CarParams::GetInstance()
          ->car_config.common_config
          .sop_openspace_planner_no_steer_change_gear_switch;
  float acceleration_forward = 1.0f;
  float acceleration_backward = 1.0f;
  state_expansion_.init(
      true, min_turn_radius_, xy_grid_resolution_, phi_grid_resolution_,
      max_speed_forward, max_speed_backward, acceleration_forward,
      acceleration_backward, min_length_per_segment_, max_zigzag_allowd_,
      search_max_distance, lat_inflation_, lat_inflation_low_, lon_inflation_,
      no_steer_change_gear_switch_, max_curvature_rate,
      max_close_gear_switch_speed,
      TmpGlobals::CONFIG_VARIABLE_STEP_SIZE_DISTANCE_OPTION_COUNT);

  if (slot_type_ == SlotType::VERTICAL_OUT ||
      slot_type_ == SlotType::OBLIQUE_OUT) {
    node_config_.traj_gear_switch_penalty = 3.0f;
  } else {
    node_config_.traj_gear_switch_penalty = config.traj_gear_switch_penalty;
  }
  enable_offline_boundary_cost_ =
      slot_type_ == SlotType::VERTICAL || slot_type_ == SlotType::OBLIQUE;

  check_collision_max_stride_ = 0.3f;
  simple_rs_end_x_range_ = 0.0f;
  simple_rs_end_y_range_ = 0.0f;
  simple_rs_end_theta_range_ = 0.0f;
  simple_rs_range_based_end_ = false;
  config_valid_ = true;
}

void HybridAstar2::updateStateMap(const SearchNode &start,
                                  const SearchNode &end) {
  state_map_coarse_.setStep({float(xy_grid_resolution_),
                             float(xy_grid_resolution_),
                             float(phi_grid_resolution_)});
  int coarse_theta_count =
      int(std::ceil(2.0 * M_PI / phi_grid_resolution_)) + 4;
  float min_x = 0.0f;
  float min_y = 0.0f;
  float max_x = 0.0f;
  float max_y = 0.0f;
  if (slot_type_ == SlotType::VERTICAL) {
    min_x = std::min(-0.5f, start.x() - 1.0f);
    max_x = std::max(12.5f, start.x() + 1.0f);
    min_y = std::min(-16.0f, start.y() - 1.0f);
    max_y = std::max(16.0f, start.y() + 1.0f);
  } else if (slot_type_ == SlotType::OBLIQUE) {
    min_x = std::min(-16.0f, start.x() - 1.0f);
    max_x = std::max(16.0f, start.x() + 1.0f);
    min_y = std::min(-16.0f, start.y() - 1.0f);
    max_y = std::max(16.0f, start.y() + 1.0f);
  } else if (slot_type_ == SlotType::PARALLEL) {
    min_x = std::min(-2.0f, start.x() - 1.0f);
    max_x = std::max(8.0f, start.x() + 1.0f);
    min_y = std::min(-4.0f, start.y() - 1.0f);
    max_y = std::max(4.0f, start.y() + 1.0f);
  } else if (slot_type_ == SlotType::VERTICAL_OUT) {
    min_x = std::min(-12.5f, end.x() - 1.0f);
    max_x = std::max(12.5f, end.x() + 1.0f);
    min_y = std::min(-16.0f, end.y() - 1.0f);
    max_y = std::max(16.0f, end.y() + 1.0f);
  } else if (slot_type_ == SlotType::OBLIQUE_OUT) {
    min_x = std::min(-16.0f, end.x() - 1.0f);
    max_x = std::max(16.0f, end.x() + 1.0f);
    min_y = std::min(-16.0f, end.y() - 1.0f);
    max_y = std::max(16.0f, end.y() + 1.0f);
  } else if (slot_type_ == SlotType::PARALLEL_OUT) {
    min_x = std::min(-2.0f, end.x() - 1.0f);
    max_x = std::max(8.0f, end.x() + 1.0f);
    min_y = std::min(-4.0f, end.y() - 1.0f);
    max_y = std::max(4.0f, end.y() + 1.0f);
  }
  int coarse_x_count = std::ceil((max_x - min_x) / float(xy_grid_resolution_));
  int coarse_y_count = std::ceil((max_y - min_y) / float(xy_grid_resolution_));
  state_map_coarse_.setSize(
      {coarse_x_count, coarse_y_count, coarse_theta_count});
  state_map_coarse_.setOrigin({min_x, min_y,
                               -0.5f * float(state_map_coarse_.size()[2]) *
                                   state_map_coarse_.step()[2]});
  state_map_coarse_.clear();

  float fine_x_min = 0.0f;
  float fine_x_max = 0.0f;
  float fine_y_min = 0.0f;
  float fine_y_max = 0.0f;
  float fine_theta_min = 0.0f;
  float fine_theta_max = 0.0f;
  if (slot_type_ == SlotType::VERTICAL || slot_type_ == SlotType::OBLIQUE) {
    state_map_fine_end_.setStep({float(xy_grid_resolution_) * 0.25f,
                                 float(xy_grid_resolution_) * 0.25f,
                                 float(phi_grid_resolution_)});
    fine_x_min = simple_rs_euler_spiral_min_straight_len_;
    fine_x_max = simple_rs_euler_spiral_min_straight_len_ + 4.0f;
    fine_y_min = -1.0f;
    fine_y_max = 1.0f;
    fine_theta_min = -float(M_PI) * 0.25f;
    fine_theta_max = float(M_PI) * 0.25f;
  } else if (slot_type_ == SlotType::PARALLEL ||
             slot_type_ == SlotType::PARALLEL_OUT) {
    state_map_fine_end_.setStep({float(xy_grid_resolution_) * 0.25f,
                                 float(xy_grid_resolution_) * 0.25f,
                                 float(phi_grid_resolution_) * 0.5f});
    fine_x_min = -0.8f;
    fine_x_max = 0.8f;
    fine_y_min = -0.3f;
    fine_y_max = 0.3f;
    fine_theta_min = -float(M_PI) * 0.25f;
    fine_theta_max = float(M_PI) * 0.25f;
  } else {
    state_map_fine_end_.setStep({float(xy_grid_resolution_),
                                 float(xy_grid_resolution_),
                                 float(phi_grid_resolution_)});
  }
  int fine_x_count =
      std::ceil((fine_x_max - fine_x_min) / state_map_fine_end_.step()[0]);
  int fine_y_count =
      std::ceil((fine_y_max - fine_y_min) / state_map_fine_end_.step()[1]);
  int fine_theta_count = std::ceil((fine_theta_max - fine_theta_min) /
                                   state_map_fine_end_.step()[2]);
  state_map_fine_end_.setSize({fine_x_count, fine_y_count, fine_theta_count});
  state_map_fine_end_.setOrigin({fine_x_min, fine_y_min, fine_theta_min});
  state_map_fine_end_.clear();
}

void HybridAstar2::Update(
    const planning_math::Box2d map_boundary,
    const std::vector<planning_math::LineSegment2d> &map) {
  updateConfig();

  // init local coordinate system
  Vec2d tmp_origin_corner = map_boundary.GetAllCorners().at(3);
  local_frame_pose_.x = tmp_origin_corner.x();
  local_frame_pose_.y = tmp_origin_corner.y();
  local_frame_pose_.theta = map_boundary.heading();
  x_bound_ = map_boundary.length();
  y_bound_ = map_boundary.width();
}

HybridAstar2::~HybridAstar2() {}

const SearchNode *
HybridAstar2::simpleRSAnalyticExpansion(const SearchNode &current,
                                        const SearchNode &end) {
  simple_rs_nodes_pool_.releaseAll();
  simple_rs_curve_pool_.releaseAll();
  float min_cost = std::numeric_limits<float>::infinity();
  const SearchNode *min_result = nullptr;
  for (const auto method :
       simple_rs_methods_for_slot_type_[std::size_t(slot_type_)]) {
    const SearchNode *result = nullptr;
    try {
      result = (this->*method)(current, end);
    } catch (const ResourcePool<curve::Curve>::BadAlloc &) {
    } catch (const ResourcePool<SearchNode>::BadAlloc &) {
    }
    if (result != nullptr && result->trajectory_cost() < min_cost) {
      min_cost = result->trajectory_cost();
      min_result = result;
    }
  }
  return min_result;
}

const SearchNode *
HybridAstar2::simpleRSAnalyticExpansionArc(const SearchNode &current,
                                           const SearchNode &end) {
  if (current.type() != curve::Type::ARC &&
      current.type() != curve::Type::LINE &&
      current.type() != curve::Type::EULER_SPIRAL &&
      current.type() != curve::Type::START &&
      current.type() != curve::Type::ACCELERATE_SPIRAL) {
    return nullptr;
  }
  if (current.travel_distance() == 0.0f) {
    return nullptr;
  }
  if (current.zigzag_num() + (current.type() == curve::Type::START ? 1 : 2) >
      max_zigzag_allowd_) {
    return nullptr;
  }
  float arc_direction = (current.type() == curve::Type::START ? 1.0 : -1.0) *
                        current.travel_distance();

  curve::State previous = current.start();
  float max_arc_len = simple_rs_arc_max_arc_len_;
  max_arc_len = std::min(max_arc_len, 2.33f * min_turn_radius_);
  float dtheta_1 = AngleDiff(current.theta(), end.theta());
  float dtheta_2 = AngleDiff(previous.theta, end.theta());
  float abs_dtheta_1 = std::abs(dtheta_1);
  float abs_dtheta_2 = std::abs(dtheta_2);
  float min_dtheta = std::min(abs_dtheta_1, abs_dtheta_2);
  if (abs_dtheta_1 < float(M_PI) * 0.5f && abs_dtheta_2 < float(M_PI) * 0.5f &&
      dtheta_1 * dtheta_2 < 0.0f) {
    min_dtheta = 0.0f;
  }
  if (min_dtheta * min_turn_radius_ > max_arc_len) {
    return nullptr;
  }
  float d_min_1 = min_turn_radius_ * (1.0f - std::cos(abs_dtheta_1));
  float d_max_1 = max_arc_len * (1.0f - std::cos(abs_dtheta_1)) / abs_dtheta_1;
  if (abs_dtheta_1 < 1e-6f) {
    d_max_1 = 0.0f;
  }
  d_min_1 -= simple_rs_end_offset_th_;
  d_max_1 += simple_rs_end_offset_th_;
  float d_min_2 = min_turn_radius_ * (1.0f - std::cos(abs_dtheta_2));
  float d_max_2 = max_arc_len * (1.0f - std::cos(abs_dtheta_2)) / abs_dtheta_2;
  if (abs_dtheta_2 < 1e-6f) {
    d_max_2 = 0.0f;
  }
  d_min_2 -= simple_rs_end_offset_th_;
  d_max_2 += simple_rs_end_offset_th_;
  float y_in_end_1 = std::abs(getRelativeY(current, end));
  float y_in_end_2 = std::abs(getRelativeY(previous, end));
  if ((y_in_end_1 < d_min_1 || y_in_end_1 > d_max_1) &&
      (y_in_end_2 < d_min_2 || y_in_end_2 > d_max_2)) {
    return nullptr;
  }

  float end_x = end.x();
  float end_y = end.y();
  float end_theta = end.theta();
  float end_sin_theta = end.sin_theta();
  float end_cos_theta = end.cos_theta();
  float max_straight_distance =
      -maxStraightDistance(end, arc_direction > 0.0f ? 0.5f : -0.5f);
  if (std::abs(max_straight_distance) < min_length_per_segment_) {
    return nullptr;
  }
  float min_straight_distance =
      (arc_direction > 0.0f ? -1.0f : 1.0f) * min_length_per_segment_;

  float best_r = 0.0f;
  float best_t = 0.0f;
  float best_direction = 0.0f;
  float best_offset = std::numeric_limits<float>::infinity();
  curve::State best_start_state;
  float best_travel_distance = 0.0f;
  float total_travel_distance = 0.0f;
  float distance_from_gear_switch =
      current.distance_from_gear_switch() - std::abs(current.travel_distance());
  auto interpolate_result =
      current.interpolate(TmpGlobals::CONFIG_SIMPLE_RS_START_SEARCH_STEP);
  for (const auto &start : interpolate_result.states) {
    total_travel_distance += interpolate_result.step;
    if (current.type() != curve::Type::START &&
        distance_from_gear_switch + total_travel_distance <
            min_length_per_segment_) {
      continue;
    }

    float dx_left_end =
        start.x - (end_x - simple_rs_end_offset_th_ * end_sin_theta);
    float dy_left_end =
        start.y - (end_y + simple_rs_end_offset_th_ * end_cos_theta);
    float x_in_end = dx_left_end * end_cos_theta + dy_left_end * end_sin_theta;
    float y_in_left_end =
        -dx_left_end * end_sin_theta + dy_left_end * end_cos_theta;
    float y_in_end = y_in_left_end + simple_rs_end_offset_th_;
    float y_in_right_end = y_in_left_end + 2.0f * simple_rs_end_offset_th_;
    float dtheta = AngleDiff(start.theta, end_theta);
    float one_cos_dtheta = 1.0f - std::cos(dtheta);
    float sin_dtheta = std::sin(dtheta);
    float inv_sin_dtheta = 1.0f / sin_dtheta;
    float inv_one_cos_dtheta = 1.0f / one_cos_dtheta;
    if (std::isinf(inv_sin_dtheta) || std::isinf(inv_one_cos_dtheta)) {
      continue;
    }

    std::array<std::pair<float, float>, 2> y_change_bounds;
    y_change_bounds[0] =
        std::make_pair(y_in_left_end, std::min(0.0f, y_in_right_end));
    y_change_bounds[1] =
        std::make_pair(std::max(0.0f, y_in_left_end), y_in_right_end);

    for (const auto y_change_bound : y_change_bounds) {
      if (y_change_bound.second < y_change_bound.first) {
        continue;
      }

      float direction = 0.0f;
      float mult = 0.0f;
      if (y_change_bound.first >= 0.0f) {
        if (arc_direction * dtheta < 0.0f) {
          continue;
        }
        mult = 1.0f;
      } else {
        if (arc_direction * dtheta > 0.0f) {
          continue;
        }
        mult = -1.0f;
      }
      if (dtheta < 0.0f) {
        direction = mult * -1.0f;
      } else {
        direction = mult * 1.0f;
      }

      float min_r_by_offset = mult * y_change_bound.first * inv_one_cos_dtheta;
      float max_r_by_offset = mult * y_change_bound.second * inv_one_cos_dtheta;
      if (y_change_bound.first < 0.0f) {
        std::swap(min_r_by_offset, max_r_by_offset);
      }
      float r_when_no_offset = mult * y_in_end * inv_one_cos_dtheta;
      float min_r_by_straight =
          mult * (-x_in_end - min_straight_distance) * inv_sin_dtheta;
      float max_r_by_straight =
          mult * (-x_in_end - max_straight_distance) * inv_sin_dtheta;
      float min_r =
          std::max(std::max(std::max(min_r_by_offset, min_r_by_straight),
                            min_turn_radius_),
                   min_length_per_segment_ / std::abs(dtheta));
      float max_r = std::min(std::min(max_r_by_offset, max_r_by_straight),
                             max_arc_len / std::abs(dtheta));
      if (min_r >= max_r) {
        continue;
      }
      float r = 0.0f;
      if (r_when_no_offset >= min_r && r_when_no_offset <= max_r) {
        r = r_when_no_offset;
      } else if (r_when_no_offset < min_r) {
        r = min_r;
      } else {
        r = max_r;
      }
      r = mult * r;
      float t = x_in_end + r * sin_dtheta;
      float offset = y_in_end - r * one_cos_dtheta;

      if (std::abs(offset) < std::abs(best_offset)) {
        best_r = r;
        best_t = t;
        best_start_state = start;
        best_travel_distance = total_travel_distance;
        best_direction = direction;
        best_offset = offset;
      }
    }
  }
  if (std::abs(best_offset) > simple_rs_end_offset_th_) {
    return nullptr;
  }

  float best_end_x = end_x + best_offset * -end_sin_theta;
  float best_end_y = end_y + best_offset * end_cos_theta;

  const SearchNode *real_start;
  if (current.type() == curve::Type::START) {
    real_start = &current;
  } else {
    curve::Curve &start_curve = simple_rs_curve_pool_.allocateForce();
    start_curve.initByCut(
        current.curve(), 0.0f,
        std::abs(best_travel_distance / current.travel_distance()));
    real_start = &simple_rs_nodes_pool_.allocateAndInit(
        node_config_, &start_curve, best_start_state.x, best_start_state.y,
        best_start_state.theta, current.previous(), best_start_state.cos_theta,
        best_start_state.sin_theta);
  }

  curve::Curve &arc_curve = simple_rs_curve_pool_.allocateForce();
  arc_curve.initAsArc(
      best_direction *
          std::abs(AngleDiff(real_start->theta(), end_theta) * best_r),
      1.0f / best_r);
  SearchNode &arc_end = simple_rs_nodes_pool_.allocateAndInit(
      node_config_, &arc_curve, best_t * end_cos_theta + best_end_x,
      best_t * end_sin_theta + best_end_y, end_theta, real_start, end_cos_theta,
      end_sin_theta);

  curve::Curve &line_curve = simple_rs_curve_pool_.allocateForce();
  line_curve.initAsLine(-best_t);
  SearchNode &line_end = simple_rs_nodes_pool_.allocateAndInit(
      node_config_, &line_curve, best_end_x, best_end_y, end_theta, &arc_end,
      end_cos_theta, end_sin_theta);
  if (real_start == &current) {
    return simpleRSAnalyticExpansionConstruct<2>({&arc_end, &line_end},
                                                 best_offset);
  } else {
    return simpleRSAnalyticExpansionConstruct<3>(
        {real_start, &arc_end, &line_end}, best_offset);
  }
}

const SearchNode *
HybridAstar2::simpleRSAnalyticExpansionEulerSpiral(const SearchNode &current,
                                                   const SearchNode &end) {
  if (current.type() != curve::Type::ARC &&
      current.type() != curve::Type::EULER_SPIRAL &&
      current.type() != curve::Type::ACCELERATE_SPIRAL) {
    return nullptr;
  }

  curve::State previous = current.start();
  float dtheta_1 = AngleDiff(end.theta(), current.theta());
  float dtheta_2 = AngleDiff(end.theta(), previous.theta);
  if (std::abs(dtheta_1) > 0.5f + simple_rs_end_theta_range_ &&
      std::abs(dtheta_2) > 0.5f + simple_rs_end_theta_range_) {
    return nullptr;
  }

  float to_end_x_current = getRelativeX(current, end);
  float to_end_x_previous = getRelativeX(previous, end);
  if (current.travel_distance() < 0.0f &&
      to_end_x_current < -simple_rs_end_x_range_ &&
      to_end_x_previous < -simple_rs_end_x_range_) {
    return nullptr;
  }
  if (current.travel_distance() > 0.0f &&
      to_end_x_current > simple_rs_end_x_range_ &&
      to_end_x_previous > simple_rs_end_x_range_) {
    return nullptr;
  }

  bool possible = [&]() -> bool {
    std::array<float, 3> curvatures = {
        current.curvature_start(), current.curvature_end(),
        0.5f * (current.curvature_start() + current.curvature_end())};
    int curvature_count = current.type() == curve::Type::ARC ? 1 : 3;
    for (int i = 0; i < curvature_count; i++) {
      float curvature = curvatures[i];
      if (curvature == 0.0f) {
        continue;
      }
      for (const auto &theta : {current.theta(), previous.theta}) {
        float d_theta = planning_math::AngleDiff(end.theta(), theta);
        if (d_theta / curvature * current.travel_distance() > 0.0f) {
          continue;
        }
        float spiral_t;
        float spiral_scale;
        if (simple_rs_range_based_end_) {
          spiral_scale = min_spiral_scale_close_gear_switch_;
          spiral_t = 0.5f * std::abs(curvature) * spiral_scale;
        } else {
          spiral_t = std::sqrt(std::abs(d_theta));
          spiral_scale = 2.0f * spiral_t / std::abs(curvature);
          if (spiral_t * spiral_scale > simple_rs_euler_spiral_max_arc_len_) {
            continue;
          }
        }
        simple_rs_temp_curve_.initAsEulerSpiral(
            current.travel_distance() > 0.0f, curvature, 0.0f, spiral_scale,
            1000.0f);
        float spiral_straight_len =
            std::min(spiral_t, spiral_straight_max_t_) * spiral_scale;
        float cos_theta = std::cos(theta);
        float sin_theta = std::sin(theta);
        for (const auto &x : {current.x(), previous.x}) {
          for (const auto &y : {current.y(), previous.y}) {
            curve::State joint = simple_rs_temp_curve_.endAfter(
                x, y, theta, cos_theta, sin_theta);
            float end_x = getRelativeX(joint, end);
            float end_y = getRelativeY(joint, end);
            float end_theta = AngleDiff(end.theta(), joint.theta);
            if (simple_rs_range_based_end_) {
              if (inRangeBasedEnd(joint, end)) {
                return true;
              }
            } else {
              float straight_len_in_current_direction =
                  -getRelativeX(joint, end) *
                  (current.travel_distance() > 0.0f ? 1.0f : -1.0f);
              if (straight_len_in_current_direction + spiral_straight_len <
                      simple_rs_euler_spiral_min_straight_len_ ||
                  straight_len_in_current_direction <= 0.0f) {
                continue;
              }
              // add 5mm buffer just to cover more potential success
              if (std::abs(getRelativeY(joint, end)) >
                  simple_rs_end_offset_th_ + 0.005f) {
                continue;
              }
              return true;
            }
          }
        }
      }
    }
    return false;
  }();
  if (!possible) {
    return nullptr;
  }

  float min_spiral_scale = current.travel_distance() > 0.0f
                               ? simple_rs_min_spiral_scale_forward_
                               : simple_rs_min_spiral_scale_backward_;
  float best_offset = std::numeric_limits<float>::infinity();
  float best_spiral_scale = 0.0f;
  float best_spiral_t = 0.0f;
  float best_straight_len = 0.0f;
  curve::State best_start;
  float best_travel_distance = 0.0f;
  curve::State best_joint;
  float total_travel_distance = 0.0f;
  bool is_arc = current.type() == curve::Type::ARC;
  float current_inv_curvature = current.inv_curvature_start();
  float distance_from_gear_switch =
      current.distance_from_gear_switch() - std::abs(current.travel_distance());
  auto interpolate_result =
      current.interpolate(TmpGlobals::CONFIG_SIMPLE_RS_START_SEARCH_STEP);
  for (const auto &start : interpolate_result.states) {
    total_travel_distance += interpolate_result.step;

    if (!is_arc) {
      if (start.curvature == 0.0f) {
        continue;
      }
      current_inv_curvature = 1.0f / start.curvature;
    }

    float spiral_t;
    float spiral_scale;
    if (simple_rs_range_based_end_) {
      spiral_scale = min_spiral_scale_close_gear_switch_;
      spiral_t = 0.5f * std::abs(start.curvature) * spiral_scale;
    } else {
      float d_theta = planning_math::AngleDiff(end.theta(), start.theta);
      if (d_theta * current_inv_curvature * current.travel_distance() > 0.0f) {
        continue;
      }
      spiral_t = std::sqrt(std::abs(d_theta));
      spiral_scale = 2.0f * spiral_t * std::abs(current_inv_curvature);
      if (spiral_scale < min_spiral_scale ||
          spiral_t * spiral_scale > simple_rs_euler_spiral_max_arc_len_) {
        continue;
      }
    }
    simple_rs_temp_curve_.initAsEulerSpiral(current.travel_distance() > 0.0f,
                                            start.curvature, 0.0f, spiral_scale,
                                            1000.0f);
    curve::State joint = simple_rs_temp_curve_.endAfter(start);
    float offset = getRelativeY(joint, end);
    float distance_from_gear_switch_to_joint =
        distance_from_gear_switch + total_travel_distance +
        std::abs(simple_rs_temp_curve_.distance());

    float straight_len = 0.0f;
    if (simple_rs_range_based_end_) {
      if (distance_from_gear_switch_to_joint < min_length_per_segment_ ||
          !inRangeBasedEnd(joint, end)) {
        continue;
      }
    } else {
      if (std::abs(offset) > simple_rs_end_offset_th_) {
        continue;
      }
      straight_len = -getRelativeX(joint, end);
      if (distance_from_gear_switch_to_joint + std::abs(straight_len) <
          min_length_per_segment_) {
        continue;
      }
      float spiral_straight_len =
          std::min(spiral_t, spiral_straight_max_t_) * spiral_scale;
      float straight_len_in_current_direction =
          straight_len * (current.travel_distance() > 0.0f ? 1.0f : -1.0f);
      if (straight_len_in_current_direction + spiral_straight_len <
              simple_rs_euler_spiral_min_straight_len_ ||
          straight_len_in_current_direction <= 0.0f) {
        continue;
      }
    }

    if (std::abs(offset) < std::abs(best_offset)) {
      best_offset = offset;
      best_spiral_scale = spiral_scale;
      best_spiral_t = spiral_t;
      best_straight_len = straight_len;
      best_start = start;
      best_travel_distance = total_travel_distance;
      best_joint = joint;
    }
  }
  if (best_offset == std::numeric_limits<float>::infinity()) {
    return nullptr;
  }

  curve::Curve &start_curve = simple_rs_curve_pool_.allocateForce();
  start_curve.initByCut(
      current.curve(), 0.0f,
      std::abs(best_travel_distance / current.travel_distance()));
  SearchNode &start = simple_rs_nodes_pool_.allocateAndInit(
      node_config_, &start_curve, best_start.x, best_start.y, best_start.theta,
      current.previous(), best_start.cos_theta, best_start.sin_theta);

  curve::Curve &spiral_curve = simple_rs_curve_pool_.allocateForce();
  spiral_curve.initAsEulerSpiral(current.travel_distance() > 0.0f,
                                 best_start.curvature, 0.0f, best_spiral_scale);
  SearchNode &spiral_end = simple_rs_nodes_pool_.allocateAndInit(
      node_config_, &spiral_curve, best_joint.x, best_joint.y, best_joint.theta,
      &start, best_joint.cos_theta, best_joint.sin_theta);

  if (simple_rs_range_based_end_) {
    return simpleRSAnalyticExpansionConstruct<2>({&start, &spiral_end},
                                                 best_offset);
  } else {
    curve::Curve &line_curve = simple_rs_curve_pool_.allocateForce();
    line_curve.initAsLine(best_straight_len);
    curve::State line_end_state = line_curve.endAfter(best_joint);
    SearchNode &line_end = simple_rs_nodes_pool_.allocateAndInit(
        node_config_, &line_curve, line_end_state.x, line_end_state.y,
        line_end_state.theta, &spiral_end, line_end_state.cos_theta,
        line_end_state.sin_theta);
    return simpleRSAnalyticExpansionConstruct<3>(
        {&start, &spiral_end, &line_end}, best_offset);
  }
}

// TODO: Jinwei: this might cause curvature discontinuous
const SearchNode *
HybridAstar2::simpleRSAnalyticExpansionLineOnArc(const SearchNode &current,
                                                 const SearchNode &end) {
  if (current.type() != curve::Type::ARC) {
    return nullptr;
  }

  curve::State previous = current.start();
  float dtheta_1 = AngleDiff(current.theta(), end.theta());
  float dtheta_2 = AngleDiff(previous.theta, end.theta());
  if (!(std::abs(dtheta_1) < float(M_PI) * 0.5f &&
        std::abs(dtheta_2) < float(M_PI) * 0.5f &&
        dtheta_1 * dtheta_2 < 0.0f)) {
    return nullptr;
  }

  float ratio =
      (end.theta() - previous.theta) / (current.theta() - previous.theta);
  curve::Curve &arc_curve = simple_rs_curve_pool_.allocateForce();
  arc_curve.initByCut(current.curve(), 0.0f, ratio);
  curve::State start_state = arc_curve.endAfter(previous);
  float offset = getRelativeY(start_state, end);
  if (std::abs(offset) > simple_rs_end_offset_th_) {
    return nullptr;
  }

  float line_distance = -getRelativeX(start_state, end);
  float line_distance_in_current_direction =
      (line_distance * current.travel_distance() > 0.0f ? 1.0f : -1.0f) *
      std::abs(line_distance);

  float same_dir_line_distance = std::max(
      0.0f, min_length_per_segment_ - current.distance_from_gear_switch() +
                std::abs(current.travel_distance() - arc_curve.distance()));
  if (line_distance_in_current_direction > 0.0f) {
    same_dir_line_distance =
        std::max(same_dir_line_distance, std::abs(line_distance));
  }
  float reverse_dir_line_distance = 0.0f;
  if (same_dir_line_distance > line_distance_in_current_direction) {
    reverse_dir_line_distance =
        same_dir_line_distance - line_distance_in_current_direction;
    if (reverse_dir_line_distance < min_length_per_segment_) {
      float diff = min_length_per_segment_ - reverse_dir_line_distance;
      same_dir_line_distance += diff;
      reverse_dir_line_distance += diff;
    }
  }
  float current_direction_mult =
      current.travel_distance() > 0.0f ? 1.0f : -1.0f;
  same_dir_line_distance *= current_direction_mult;
  reverse_dir_line_distance *= -current_direction_mult;
  if (reverse_dir_line_distance != 0.0f &&
      current.zigzag_num() + 1 > max_zigzag_allowd_) {
    return nullptr;
  }

  SearchNode &start = simple_rs_nodes_pool_.allocateAndInit(
      node_config_, &arc_curve, start_state.x, start_state.y, start_state.theta,
      current.previous(), start_state.cos_theta, start_state.sin_theta);

  curve::Curve &same_dir_line_curve = simple_rs_curve_pool_.allocateForce();
  same_dir_line_curve.initAsLine(same_dir_line_distance);
  float same_dir_line_x =
      same_dir_line_distance * end.cos_theta() + start_state.x;
  float same_dir_line_y =
      same_dir_line_distance * end.sin_theta() + start_state.y;
  curve::Curve &reverse_dir_line_curve = simple_rs_curve_pool_.allocateForce();
  reverse_dir_line_curve.initAsLine(reverse_dir_line_distance);
  float reverse_dir_line_x =
      reverse_dir_line_distance * end.cos_theta() + same_dir_line_x;
  float reverse_dir_line_y =
      reverse_dir_line_distance * end.sin_theta() + same_dir_line_y;

  if (same_dir_line_distance != 0.0f && reverse_dir_line_distance != 0.0f) {
    SearchNode &same_dir_line_end = simple_rs_nodes_pool_.allocateAndInit(
        node_config_, &same_dir_line_curve, same_dir_line_x, same_dir_line_y,
        end.theta(), &start, end.cos_theta(), end.sin_theta());
    SearchNode &reverse_dir_line_end = simple_rs_nodes_pool_.allocateAndInit(
        node_config_, &reverse_dir_line_curve, reverse_dir_line_x,
        reverse_dir_line_y, end.theta(), &same_dir_line_end, end.cos_theta(),
        end.sin_theta());
    return simpleRSAnalyticExpansionConstruct<3>(
        {&start, &same_dir_line_end, &reverse_dir_line_end}, offset);
  } else if (same_dir_line_distance == 0.0f &&
             reverse_dir_line_distance != 0.0f) {
    SearchNode &reverse_dir_line_end = simple_rs_nodes_pool_.allocateAndInit(
        node_config_, &reverse_dir_line_curve, reverse_dir_line_x,
        reverse_dir_line_y, end.theta(), &start, end.cos_theta(),
        end.sin_theta());
    return simpleRSAnalyticExpansionConstruct<2>(
        {&start, &reverse_dir_line_end}, offset);
  } else if (same_dir_line_distance != 0.0f &&
             reverse_dir_line_distance == 0.0f) {
    SearchNode &same_dir_line_end = simple_rs_nodes_pool_.allocateAndInit(
        node_config_, &same_dir_line_curve, same_dir_line_x, same_dir_line_y,
        end.theta(), &start, end.cos_theta(), end.sin_theta());
    return simpleRSAnalyticExpansionConstruct<2>({&start, &same_dir_line_end},
                                                 offset);
  } else {
    return nullptr;
  }
}

const SearchNode *
HybridAstar2::simpleRSAnalyticExpansionLineOnPoint(const SearchNode &current,
                                                   const SearchNode &end) {
  if (current.type() != curve::Type::START &&
      current.type() != curve::Type::ARC &&
      current.type() != curve::Type::LINE &&
      current.type() != curve::Type::EULER_SPIRAL &&
      current.type() != curve::Type::ACCELERATE_SPIRAL) {
    return nullptr;
  }
  if (current.curvature_end() != 0.0f) {
    return nullptr;
  }
  if (std::abs(current.theta() - end.theta()) > 1e-3f) {
    return nullptr;
  }

  float distance = getRelativeX(end, current);
  float line_end_x = current.x() + distance * current.cos_theta();
  float line_end_y = current.y() + distance * current.sin_theta();
  float dx_end = line_end_x - end.x();
  float dy_end = line_end_y - end.y();
  float offset = dx_end * -end.sin_theta() + dy_end * end.cos_theta();
  if (std::abs(offset) > simple_rs_end_offset_th_) {
    return nullptr;
  }
  if (distance * current.travel_distance() < 0.0f &&
      (current.zigzag_num() + 1 > max_zigzag_allowd_ ||
       current.type() == curve::Type::START)) {
    return nullptr;
  }

  curve::Curve &line_curve = simple_rs_curve_pool_.allocateForce();
  line_curve.initAsLine(distance);
  SearchNode &line_end = simple_rs_nodes_pool_.allocateAndInit(
      node_config_, &line_curve, line_end_x, line_end_y, current.theta(),
      &current, current.cos_theta(), current.sin_theta());
  if (line_end.distance_from_gear_switch() < min_length_per_segment_) {
    return nullptr;
  }
  return simpleRSAnalyticExpansionConstruct<1>({&line_end}, offset);
}

const SearchNode *
HybridAstar2::simpleRSAnalyticExpansionDirect(const SearchNode &current,
                                              const SearchNode &end) {
  if (!simple_rs_range_based_end_) {
    return nullptr;
  }
  if (current.type() != curve::Type::LINE) {
    return nullptr;
  }
  if (!inRangeBasedEnd(current, end) &&
      !inRangeBasedEnd(current.start(), end)) {
    return nullptr;
  }

  float best_offset = std::numeric_limits<float>::infinity();
  curve::State best_end_state;
  float best_travel_distance = 0.0f;
  float total_travel_distance = 0.0f;
  float distance_from_gear_switch =
      current.distance_from_gear_switch() - std::abs(current.travel_distance());
  auto interpolate_result =
      current.interpolate(TmpGlobals::CONFIG_SIMPLE_RS_START_SEARCH_STEP);
  for (const auto &end_state : interpolate_result.states) {
    total_travel_distance += interpolate_result.step;
    if (distance_from_gear_switch + total_travel_distance <
            min_length_per_segment_ ||
        !inRangeBasedEnd(end_state, end)) {
      continue;
    }

    float y_in_end = getRelativeY(end_state, end);
    if (std::abs(y_in_end) < std::abs(best_offset)) {
      best_end_state = end_state;
      best_travel_distance = total_travel_distance;
      best_offset = y_in_end;
    }
  }
  if (best_offset == std::numeric_limits<float>::infinity()) {
    return nullptr;
  }

  curve::Curve &start_curve = simple_rs_curve_pool_.allocateForce();
  start_curve.initByCut(
      current.curve(), 0.0f,
      std::abs(total_travel_distance / current.travel_distance()));
  SearchNode &real_end = simple_rs_nodes_pool_.allocateAndInit(
      node_config_, &start_curve, best_end_state.x, best_end_state.y,
      best_end_state.theta, current.previous(), best_end_state.cos_theta,
      best_end_state.sin_theta);
  return simpleRSAnalyticExpansionConstruct<1>({&real_end}, best_offset);
}

template <int NodeCount>
const SearchNode *HybridAstar2::simpleRSAnalyticExpansionConstruct(
    std::array<const SearchNode *, NodeCount> nodes, float offset) {
  const SearchNode *previous = nodes[0]->previous();
  SearchNode *generated_previous = nullptr;
  for (const SearchNode *node : nodes) {
    float last_ratio = 0.0f;
    auto interpolate_result = node->interpolate(check_collision_max_stride_);
    float ratio_step =
        std::abs(interpolate_result.step / node->travel_distance());
    for (const auto &state : interpolate_result.states) {
      curve::Curve &curve = simple_rs_curve_pool_.allocateForce();
      float ratio = last_ratio + ratio_step;
      curve.initByCut(node->curve(), last_ratio, ratio);

      SearchNode &new_node = simple_rs_nodes_pool_.allocateAndInit(
          node_config_, &curve, state.x, state.y, state.theta, previous,
          state.cos_theta, state.sin_theta);
      if (checkCollisionUsingMultiCircle(new_node)) {
        return nullptr;
      }
      previous = &new_node;
      generated_previous = &new_node;
      last_ratio = ratio;
    }
  }
  generated_previous->addEndOffsetCost(node_config_, offset);
  return generated_previous;
}

void HybridAstar2::combineTrajectory(const SearchNode *current,
                                     const Transform &trans) {
  result_.clear();
  if (current == nullptr) {
    result_.status = SbpStatus::EXCEPTION;
    return;
  }

  float rad2deg = 180.0f / float(M_PI);
  for (const SearchNode *node = current; node != nullptr;
       node = node->previous()) {
    const auto &interp =
        node->interpolate(TmpGlobals::CONFIG_MAX_PATH_LENGTH_PER_NODE).states;
    for (auto i = interp.rbegin(); i != interp.rend(); i++) {
      if (result_.phi.size() > 0 &&
          std::hypot(i->x - result_.x.back(), i->y - result_.y.back()) <
              min_length_per_output_point_) {
        continue;
      }
      result_.x.push_back(i->x);
      result_.y.push_back(i->y);
      result_.phi.push_back(i->theta);
      result_.steer.push_back(curve::Curve::toSteer(i->curvature) * rad2deg);
    }
  }
  for (std::size_t i = 0; i < result_.x.size(); i++) {
    trans.fromSelf(result_.x[i], result_.y[i], result_.phi[i]);
  }
  result_.wheel_base_offset.resize(result_.x.size(), 0.0);

  result_.status = SbpStatus::SUCCESS;
}

bool HybridAstar2::isPlanInSlot(const SearchNode &start) {
  return start.inRange(plan_in_slot_x_min_, plan_in_slot_x_max_,
                       plan_in_slot_y_th_);
}

bool HybridAstar2::isReplanInSlot(const SearchNode &start) {
  return std::abs(start.travel_distance()) != 0.0f && isPlanInSlot(start);
}

bool HybridAstar2::Plan(const std::vector<SbpObstaclePtr> &obs_ptrs,
                        parking::SearchProcessDebug *sp_debug) {
                          std::cout << "astar Plan!" << std::endl;
#ifdef BUILD_IN_TEST_BAG_RECURRENT
  constexpr bool enable_timer = true;
#else  // BUILD_IN_TEST_BAG_RECURRENT
  constexpr bool enable_timer = true;
#endif // BUILD_IN_TEST_BAG_RECURRENT
  constexpr int timer_iter = 1000;

  auto plan_timer = Timer<enable_timer>("plan");
  auto pq_insert_timer = TotalTimer<enable_timer, timer_iter>("pq insert");
  auto pq_pop_timer = TotalTimer<enable_timer, timer_iter>("pq pop");
  auto pq_erase_timer = TotalTimer<enable_timer, timer_iter>("pq erase");
  auto pq_pop_back_timer = TotalTimer<enable_timer, timer_iter>("pq pop back");
  auto open_set_coarse_find_timer =
      TotalTimer<enable_timer, timer_iter>("open set coarse find");
  auto open_set_coarse_insert_timer =
      TotalTimer<enable_timer, timer_iter>("open set coarse insert");
  auto open_set_fine_end_find_timer =
      TotalTimer<enable_timer, timer_iter>("open set fine end find");
  auto open_set_fine_end_insert_timer =
      TotalTimer<enable_timer, timer_iter>("open set fine end insert");
  auto analytic_expansion_timer =
      TotalTimer<enable_timer, timer_iter>("analytic expansion");
  auto get_next_states_timer =
      TotalTimer<enable_timer, timer_iter>("get next states");
  auto check_collision_timer =
      TotalTimer<enable_timer, timer_iter>("check collision");
  auto sp_debug_timer = TotalTimer<enable_timer, timer_iter>("sp debug");
  auto heuristic_cost_timer =
      TotalTimer<enable_timer, timer_iter>("heuristic cost");

  plan_timer.Tic();

  if (start_node_ == nullptr || end_node_ == nullptr ||
      !mc_footprint_model_.inited() || max_steer_rate_ == 0.0f ||
      !config_valid_) {
    result_.status = SbpStatus::EXCEPTION;
    return false;
  }

  Transform trans;
  if (isParkIn()) {
    trans = Transform(start_node_->x, start_node_->y,
                      planning_math::NormalizeAngle(start_node_->theta));
  } else if (isParkOut()) {
    trans = Transform(end_node_->x, end_node_->y,
                      planning_math::NormalizeAngle(end_node_->theta));
  }

  bool plan_for_lpnp = start_node_->vel == 2.0;
  float start_v = -end_node_->vel;
  bool require_forward = false;
  bool require_backward = false;
  if (start_v < 0.0f || (plan_for_lpnp && start_v == 0.0f)) {
    require_forward = false;
    require_backward = true;
  } else if (start_v > 0.0f) {
    require_forward = true;
    require_backward = false;
  }

  curve::Curve start_curve;
  start_curve.initAsStart(require_forward, require_backward);
  SearchNode start_node(node_config_, &start_curve, end_node_->x, end_node_->y,
                        NormalizeAngle(end_node_->theta));
  curve::Curve end_curve;
  end_curve.initAsEnd();
  SearchNode end_node(node_config_, &end_curve, start_node_->x, start_node_->y,
                      NormalizeAngle(start_node_->theta));

  std::vector<SbpObstaclePtr> all_obs_ptrs = obs_ptrs;
  if (slot_type_ == SlotType::VERTICAL) {
    VirtualObstacleCreator::addVirtualWheelStops(
        end_node.x(), end_node.y(), end_node.theta(), start_node.x(),
        start_node.y(), start_node.theta(), mc_footprint_model_,
        CarParams::GetInstance()->vehicle_width_wo_rearview_mirror,
        lat_inflation_low_, all_obs_ptrs);
  } else if (slot_type_ == SlotType::VERTICAL_OUT) {
    VirtualObstacleCreator::addVirtualWheelStops(
        start_node.x(), start_node.y(), start_node.theta(), end_node.x(),
        end_node.y(), end_node.theta(), mc_footprint_model_,
        CarParams::GetInstance()->vehicle_width_wo_rearview_mirror,
        lat_inflation_low_, all_obs_ptrs);
  }

  trans.toSelf(start_node);
  trans.toSelf(end_node);
  if (slot_type_ == SlotType::PARALLEL) {
    simple_rs_euler_spiral_min_straight_len_ = 0.0f;
    if (isPlanInSlot(start_node)) {
      simple_rs_end_offset_th_ = 0.15f;
      node_config_.traj_penalty_start_at_reverse_as_gear_switch = false;
    } else {
      simple_rs_end_offset_th_ = 0.03f;
      node_config_.traj_penalty_start_at_reverse_as_gear_switch = true;
    }
  } else if (isParkOut()) {
    simple_rs_euler_spiral_min_straight_len_ = 0.0f;
    simple_rs_end_offset_th_ = 0.03f;
    node_config_.traj_penalty_start_at_reverse_as_gear_switch = false;
  } else {
    if (isReplanInSlot(start_node)) {
      simple_rs_euler_spiral_min_straight_len_ = TmpGlobals::
          CONFIG_SIMPLE_RS_EULER_SPIRAL_REPLAN_IN_SLOT_MIN_STRAIGHT_LENGTH;
    } else {
      simple_rs_euler_spiral_min_straight_len_ =
          TmpGlobals::CONFIG_SIMPLE_RS_EULER_SPIRAL_MIN_STRAIGHT_LENGTH;
    }
    simple_rs_end_offset_th_ = 0.03f;
    node_config_.traj_penalty_start_at_reverse_as_gear_switch = false;
  }

  //[Hybrid Astar2.0]  obstacle grid construction
  auto obstacle_grid_cost_timer =
      Timer<enable_timer>("obstacle grid construct");

  // this config (1200*1200*0.025 might be slower than 600*600*0.05, but precise
  // (tested max error ~ 0.02-0.03m))
  obstacle_grid_cost_timer.Tic();
  std::vector<SbpObstaclePoint> discrete_obs;
  for (const auto &obs_ptr : all_obs_ptrs) {
    std::vector<planning_math::Vec2d> points = obs_ptr->getDiscretePoints(0.1);
    ObstacleHeightType type = obs_ptr->getHeightType();
    if (type == ObstacleHeightType::MAP_LINE) {
      if (slot_type_ == SlotType::PARALLEL ||
          slot_type_ == SlotType::PARALLEL_OUT) {
        type = ObstacleHeightType::LOW;
      } else {
        continue;
      }
    }
    for (auto &point : points) {
      trans.toSelf(point);
    }
    discrete_obs.emplace_back(points);
    discrete_obs.back().setHeightType(type);
  }

  float extra_inflation = 0.025f * std::sqrt(2.0f) * 0.5f + 0.02f;
  obs_grid_.constructFromSbpObstacleMultiHeights(
      discrete_obs, mc_footprint_model_, end_node,
      lat_inflation_ + extra_inflation, lat_inflation_low_ + extra_inflation);

  obstacle_grid_cost_timer.Toc();

  if (TmpGlobals::CONFIG_OBSTACLE_GRID_DEBUG) {
    // create a obstacle grid test class to build gt and do testing stuff
    obs_grid_test_.init(obs_grid_.rows_, obs_grid_.cols_, obs_grid_.res_,
                        obs_grid_.origin_x_, obs_grid_.origin_y_);
    obs_grid_test_.constructGridGTWithoutVoroniFromSbpObstaclePoints(
        discrete_obs);
    obs_grid_test_.dumpTestDataToFile("../build/obstacle_grid_test_data_0.txt");

    obs_grid_test_.testGridDiff(
        obs_grid_.grid_data_u16_[(int)ObstacleHeightType::HIGH].data(),
        obs_grid_.grid_data_scale_);
    obs_grid_test_.getTestGridDiffResult();
  }
#ifdef OBSTACLE_GRID_DEBUG_DUMP_IMAGE
  obs_grid_.dumpGridImage("/tmp/obstacle_grid_debug_img_low.png",
                          ObstacleHeightType::LOW);
  obs_grid_.dumpGridImage("/tmp/obstacle_grid_debug_img_high.png",
                          ObstacleHeightType::HIGH);

  //  obs_grid_legacy_.dumpGridImage("/tmp/obstacle_grid_legacy_debug_img.png");
#endif

  Pose2D local_frame_pose = local_frame_pose_;
  trans.toSelf(local_frame_pose);
  state_expansion_.setLocalInfo(local_frame_pose, x_bound_, y_bound_);

  const HeuristicCost *heuristic_cost = nullptr;
  float min_gear_switch_cost_diff = std::numeric_limits<float>::max();
  for (const auto &item : heuristic_cost_) {
    float diff = std::abs(item.first - node_config_.traj_gear_switch_penalty);
    if (diff < min_gear_switch_cost_diff && item.second.inited()) {
      heuristic_cost = &item.second;
      min_gear_switch_cost_diff = diff;
    }
  }
  if (heuristic_cost == nullptr) {
    result_.status = SbpStatus::EXCEPTION;
    return false;
  }

  float heuristic_cost_from_inside_weight = 0.0f;
  if (useHeuristicCostFromInside()) {
    heuristic_cost_from_inside_weight = 1.0f;
    auto build_heuristic_cost_from_inside_timer =
        Timer<enable_timer>("build heuristic cost from inside");
    build_heuristic_cost_from_inside_timer.Tic();
    createHeuristicCostFromInside(100000, node_config_.traj_gear_switch_penalty,
                                  -0.5f, 12.5f, -4.0f, 4.0f, local_frame_pose,
                                  true, trans, sp_debug);
    build_heuristic_cost_from_inside_timer.Toc();
  }

  nodes_pool_.releaseAll();
  int start_node_key = nodes_pool_.allocate().key;
  nodes_pool_[start_node_key].copy(start_node);
  pq_.clear();
  pq_.push(start_node_key);
  updateStateMap(start_node, end_node);

  const SearchNode *best_expansion_start = nullptr;
  float best_expansion_cost = std::numeric_limits<float>::infinity();
  float max_possible_pq_cost = std::numeric_limits<float>::infinity();
  for (iter = 0; iter < max_iter_max_ &&
                 !(iter >= max_iter_base_ && best_expansion_start != nullptr) &&
                 !pq_.empty();
       iter++) {
    int max_remain_iters = max_iter_max_ - iter;
    while (pq_.size() > max_remain_iters) {
      pq_pop_back_timer.Tic();
      int key = pq_.pop_back();
      max_possible_pq_cost = std::min(max_possible_pq_cost, pq_cost_func_(key));
      nodes_pool_.release(key);
      pq_pop_back_timer.Toc();
    }

    pq_pop_timer.Tic();
    SearchNode &current = nodes_pool_[pq_.pop()];
    pq_pop_timer.Toc();

    analytic_expansion_timer.Tic();
    const SearchNode *result = simpleRSAnalyticExpansion(current, end_node);
    if (result != nullptr) {
      if (best_expansion_start == nullptr ||
          result->trajectory_cost() < best_expansion_cost) {
        best_expansion_start = &current;
        best_expansion_cost = result->trajectory_cost();
      }
    }
    analytic_expansion_timer.Toc();

    parking::SearchDebugNode debug_curr(
        current.x(), current.y(), current.theta(), current.trajectory_cost(),
        current.heuristic_cost());
    std::vector<parking::SearchDebugEdge> debug_edges_vec;
    if (sp_debug != nullptr) {
      trans.fromSelf(debug_curr);
    }

    if (!current.distance_to_obstacle_set()) {
      calcMinDistanceToObstacle(current);
    }
    if (enable_offline_boundary_cost_) {
      current.setBoundaryCost(
          boundary_cost_.getCost(current.x(), current.y(), current.theta()));
    }

    get_next_states_timer.Tic();
    const std::vector<SearchNode *> &next = state_expansion_.getNextStates(
        node_config_, current, mc_footprint_model_, obs_grid_);
    get_next_states_timer.Toc();

    for (size_t i = 0; i < next.size(); i++) {
      std::array<float, 3> to_insert{next[i]->x(), next[i]->y(),
                                     next[i]->theta()};
      open_set_fine_end_find_timer.Tic();
      auto state_insert_res_fine_end =
          state_map_fine_end_.tryInsert(to_insert, *next[i]);
      open_set_fine_end_find_timer.Toc();

      decltype(state_insert_res_fine_end) state_insert_res_coarse;
      if (!state_insert_res_fine_end.belong) {
        open_set_coarse_find_timer.Tic();
        state_insert_res_coarse =
            state_map_coarse_.tryInsert(to_insert, *next[i]);
        open_set_coarse_find_timer.Toc();
      }

      bool override_prune = false;
      if (state_insert_res_fine_end.belong) {
        if (!state_insert_res_fine_end.success &&
            SearchNode::isParentOrBrother(
                nodes_pool_[state_insert_res_fine_end.key], *next[i])) {
          override_prune = true;
        }
      } else if (state_insert_res_coarse.belong) {
        if (!state_insert_res_coarse.success &&
            SearchNode::isParentOrBrother(
                nodes_pool_[state_insert_res_coarse.key], *next[i])) {
          override_prune = true;
        }
      }

      if (state_insert_res_coarse.success ||
          state_insert_res_fine_end.success || override_prune) {
        check_collision_timer.Tic();
        bool check_collision_result = checkCollisionUsingMultiCircle(*next[i]);
#ifdef OBSTACLE_GRID_DEBUG_DUMP_IMAGE
        MultiModelMinDistances min_dis = calcMinDistanceToObstacle(*next[i]);

        float node_x = next[i]->x();
        float node_y = next[i]->y();
        float node_theta = next[i]->theta();

        float min_dis_low_wheels =
            min_dis.distances[MultiModelMinDistances::LOW_WHEELS];
        float min_dis_not_allow_close =
            min_dis.distances[MultiModelMinDistances::NOT_ALLOW_CLOSE];
        float min_dis_full_body =
            min_dis.distances[MultiModelMinDistances::FULL_BODY];

        if (iter < 20 || (iter % 100 == 0 && i == 0)) {
          char debug_str[512];
          sprintf(debug_str,
                  "min_dis = (%.2f, %.2f %.2f), pose = %.2f %.2f %.2f",
                  min_dis_low_wheels, min_dis_not_allow_close,
                  min_dis_full_body, node_x, node_y, node_theta);
          char file_name[512];
          sprintf(file_name, "/tmp/obstacle_[%d][%02d]_low.png", (int)iter,
                  (int)i);
          obs_grid_.dumpcalcMinDistanceImage(file_name, mc_footprint_model_,
                                             ObstacleHeightType::LOW,
                                             debug_str);

          sprintf(file_name, "/tmp/obstacle_[%d][%02d]_high.png", (int)iter,
                  (int)i);
          obs_grid_.dumpcalcMinDistanceImage(file_name, mc_footprint_model_,
                                             ObstacleHeightType::HIGH,
                                             debug_str);
        }
#endif
        check_collision_timer.Toc();
        if (check_collision_result) {
          continue;
        }

        auto allocate_res = nodes_pool_.allocate();
        if (!allocate_res.success) {
          continue;
        }
        SearchNode &copied_next = nodes_pool_[allocate_res.key];
        copied_next.copy(*next[i]);

        heuristic_cost_timer.Tic();
        float heuristic = 0.0f;
        bool use_heuristic_cost_offline = false;
        if (heuristic_cost_from_inside_weight > 0.0f) {
          float heuristic_cost_from_inside = heuristic_cost_from_inside_.get(
              true, end_node.x(), end_node.y(), end_node.theta(),
              end_node.cos_theta(), end_node.sin_theta(), copied_next.x(),
              copied_next.y(), copied_next.theta());
          heuristic +=
              heuristic_cost_from_inside * heuristic_cost_from_inside_weight;
          use_heuristic_cost_offline = heuristic_cost_from_inside ==
                                       heuristic_cost_from_inside_.max_cost();
        } else {
          use_heuristic_cost_offline = true;
        }
        if (use_heuristic_cost_offline) {
          heuristic += heuristic_cost->get(
              copied_next.travel_distance() > 0.0f, copied_next.x(),
              copied_next.y(), copied_next.theta(), copied_next.cos_theta(),
              copied_next.sin_theta(), end_node.x(), end_node.y(),
              end_node.theta());
        }
        copied_next.setHeuristicCost(heuristic);
        heuristic_cost_timer.Toc();

        if (sp_debug != nullptr) {
          sp_debug_timer.Tic();
          parking::SearchDebugNode debug_next(
              copied_next.x(), copied_next.y(), copied_next.theta(),
              copied_next.trajectory_cost(), copied_next.heuristic_cost());
          trans.fromSelf(debug_next);
          debug_edges_vec.emplace_back(debug_curr, debug_next);
          sp_debug_timer.Toc();
        }

        if (pq_cost_func_(allocate_res.key) >= max_possible_pq_cost) {
          nodes_pool_.release(allocate_res.key);
          continue;
        }

        pq_insert_timer.Tic();
        pq_.push(allocate_res.key);
        pq_insert_timer.Toc();
        int replaced = nodes_pool_.invalidKey();
        if (state_insert_res_fine_end.belong) {
          if (state_insert_res_fine_end.success) {
            open_set_fine_end_insert_timer.Tic();
            replaced = state_map_fine_end_.insert(
                state_insert_res_fine_end.bucket_id,
                state_insert_res_fine_end.offset, allocate_res.key);
            open_set_fine_end_insert_timer.Toc();
          }
        } else if (state_insert_res_coarse.belong) {
          if (state_insert_res_coarse.success) {
            open_set_coarse_insert_timer.Tic();
            replaced = state_map_coarse_.insert(
                state_insert_res_coarse.bucket_id,
                state_insert_res_coarse.offset, allocate_res.key);
            open_set_coarse_insert_timer.Toc();
          }
        }
        if (replaced != nodes_pool_.invalidKey() &&
            !SearchNode::isBrother(nodes_pool_[replaced],
                                   nodes_pool_[allocate_res.key])) {
          pq_erase_timer.Tic();
          if (pq_.erase(replaced)) {
            nodes_pool_.release(replaced);
          }
          pq_erase_timer.Toc();
        }
      }
    }
    if (sp_debug != nullptr) {
      sp_debug_timer.Tic();
      sp_debug->added_edges.push_back(debug_edges_vec);
      sp_debug_timer.Toc();
    }
  }

  if (best_expansion_start) {
    combineTrajectory(
        simpleRSAnalyticExpansion(*best_expansion_start, end_node), trans);
  } else if (pq_.empty()) {
    result_.status = SbpStatus::INFEASIBLE;
  } else {
    result_.status = SbpStatus::TIMEOUT;
  }

  result_.iteration_times = iter;
  result_.debug_string = "iteration_times = " + std::to_string(iter);

  plan_timer.Toc();
  return result_.status == SbpStatus::SUCCESS;
}

SbpResult HybridAstar2::getResult() { return result_; }

void HybridAstar2::setSlotType(SlotType value) { slot_type_ = value; }

void HybridAstar2::setSlotTypeByPlanningCore(int value) {
  slot_type_ = planningCoreToSlotType(value);
}

std::vector<Pose2D> HybridAstar2::getSearchPoints() { return searchPoints_; }

MultiModelMinDistances
HybridAstar2::calcMinDistanceToObstacle(float x, float y, float cos_theta,
                                        float sin_theta) {
  mc_footprint_model_.updatePose(x, y, cos_theta, sin_theta);
  return obs_grid_.calcMultiModelMinDistance(mc_footprint_model_);
}

MultiModelMinDistances
HybridAstar2::calcMinDistanceToObstacle(SearchNode &current) {
  MultiModelMinDistances min_dis = calcMinDistanceToObstacle(
      current.x(), current.y(), current.cos_theta(), current.sin_theta());
  current.setDistanceToObstacle(
      std::min(min_dis.distances[MultiModelMinDistances::FULL_BODY],
               min_dis.distances[MultiModelMinDistances::LOW_WHEELS]));
  return min_dis;
}

float HybridAstar2::getLatInflation(const SearchNode &current) {
  if (slot_type_ != SlotType::PARALLEL &&
      slot_type_ != SlotType::PARALLEL_OUT) {
    if (current.inRange(lat_inflation_add_in_slot_x_min_,
                        lat_inflation_add_in_slot_x_max_,
                        lat_inflation_add_in_slot_y_th_,
                        lat_inflation_add_in_slot_theta_th_)) {
      return lat_inflation_;
    } else if (current.inRange(lat_inflation_add_slot_entry_x_min_,
                               lat_inflation_add_slot_entry_x_max_,
                               lat_inflation_add_slot_entry_y_th_,
                               lat_inflation_add_slot_entry_theta_th_)) {
      return lat_inflation_ + lat_inflation_add_slot_entry_;
    } else {
      return lat_inflation_ + lat_inflation_add_out_slot_;
    }
  } else if (slot_type_ == SlotType::PARALLEL) {
    if (current.travel_distance() < 0.0f &&
        current.distance_from_gear_switch() > 2.0f) {
      return lat_inflation_ + lat_inflation_add_parallel_long_reverse_;
    } else {
      return lat_inflation_;
    }
  } else {
    return lat_inflation_;
  }
}

bool HybridAstar2::checkCollisionUsingMultiCircle(float x, float y,
                                                  float cos_theta,
                                                  float sin_theta,
                                                  float inflation,
                                                  float inflation_low) {
  MultiModelMinDistances min_dis =
      calcMinDistanceToObstacle(x, y, cos_theta, sin_theta);
  return min_dis.distances[MultiModelMinDistances::FULL_BODY] < inflation ||
         min_dis.distances[MultiModelMinDistances::LOW_WHEELS] < inflation_low;
}

bool HybridAstar2::checkCollisionUsingMultiCircleForLonInflation(
    bool forward, const curve::State &current, float lat_inflation,
    float lon_inflation, float lat_inflation_low) {
  float lon_inflation_adjuster =
      (lon_inflation - lat_inflation) * (forward ? 1.0f : -1.0f);
  return checkCollisionUsingMultiCircle(
      current.x + lon_inflation_adjuster * current.cos_theta,
      current.y + lon_inflation_adjuster * current.sin_theta, current.cos_theta,
      current.sin_theta, lat_inflation, lat_inflation_low);
}

bool HybridAstar2::checkCollisionUsingMultiCircleForLatInflation(
    SearchNode &current, float lat_inflation, float lon_inflation,
    float lat_inflation_low) {
  MultiModelMinDistances min_dis = calcMinDistanceToObstacle(current);
  if (min_dis.distances[MultiModelMinDistances::FULL_BODY] < lon_inflation ||
      min_dis.distances[MultiModelMinDistances::LOW_WHEELS] <
          lat_inflation_low) {
    return true;
  }
  if (min_dis.distances[MultiModelMinDistances::FULL_BODY] > lat_inflation) {
    return false;
  }
  float adjuster = lat_inflation - lon_inflation;
  float x = current.x();
  float y = current.y();
  float sin_theta = current.sin_theta();
  float cos_theta = current.cos_theta();
  return checkCollisionUsingMultiCircle(
             x + adjuster * -sin_theta, y + adjuster * cos_theta, cos_theta,
             sin_theta, lon_inflation, lat_inflation_low) ||
         checkCollisionUsingMultiCircle(
             x + adjuster * sin_theta, y + adjuster * -cos_theta, cos_theta,
             sin_theta, lon_inflation, lat_inflation_low);
}

bool HybridAstar2::checkCollisionUsingMultiCircle(SearchNode &current,
                                                  float lat_inflation,
                                                  float lon_inflation,
                                                  float lat_inflation_low) {
  if (lat_inflation > lon_inflation) {
    return checkCollisionUsingMultiCircleForLatInflation(
        current, lat_inflation, lon_inflation, lat_inflation_low);
  }

  if (lon_inflation > lat_inflation && current.previous() != nullptr &&
      current.previous()->travel_distance() * current.travel_distance() <
          0.0f) {
    if (checkCollisionUsingMultiCircleForLonInflation(
            current.previous()->forward(), current.previous()->state(),
            lat_inflation, lon_inflation, lat_inflation_low)) {
      return true;
    }
  }

  MultiModelMinDistances min_dis = calcMinDistanceToObstacle(current);
  return (min_dis.distances[MultiModelMinDistances::FULL_BODY] <
              lat_inflation ||
          min_dis.distances[MultiModelMinDistances::LOW_WHEELS] <
              lat_inflation_low);
}

bool HybridAstar2::checkCollisionUsingMultiCircle(SearchNode &current) {
  return checkCollisionUsingMultiCircle(current, getLatInflation(current),
                                        lon_inflation_, lat_inflation_low_);
}

bool HybridAstar2::checkCollisionAfterStraight(const SearchNode &current,
                                               float distance) {
  return checkCollisionIfGearSwitch(
      distance > 0.0f, current.x() + distance * current.cos_theta(),
      current.y() + distance * current.sin_theta(), current.cos_theta(),
      current.sin_theta(), lat_inflation_, lat_inflation_low_, lon_inflation_,
      mc_footprint_model_, obs_grid_);
}

float HybridAstar2::maxStraightDistance(const SearchNode &current, float max,
                                        float resolution) {
  if (!checkCollisionAfterStraight(current, max)) {
    return max;
  }
  float lower_bound = 0.0f;
  float upper_bound = max;
  while (std::abs(lower_bound - upper_bound) > resolution) {
    float check = 0.5f * (lower_bound + upper_bound);
    if (checkCollisionAfterStraight(current, check)) {
      upper_bound = check;
    } else {
      lower_bound = check;
    }
  }
  return lower_bound;
}

bool HybridAstar2::planningCoreSupported(int value) {
  return planningCoreToSlotType(value) != SlotType::UNKNOWN;
}

HybridAstar2::SlotType HybridAstar2::planningCoreToSlotType(int value) {
  if (value == 0) {
    return SlotType::VERTICAL;
  } else if (value == 8) {
    return SlotType::OBLIQUE;
  } else if (value == 9) {
    return SlotType::PARALLEL;
  } else if (value == 10) {
    return SlotType::VERTICAL_OUT;
  } else if (value == 11) {
    return SlotType::OBLIQUE_OUT;
  } else if (value == 12) {
    return SlotType::PARALLEL_OUT;
  } else {
    return SlotType::UNKNOWN;
  }
}

void HybridAstar2::setStartRange(double x_range, double y_range,
                                 double theta_range) {
  simple_rs_end_x_range_ = x_range;
  simple_rs_end_y_range_ = y_range;
  simple_rs_end_theta_range_ = theta_range;
  simple_rs_range_based_end_ = isParkOut() && simple_rs_end_x_range_ > 0.0f &&
                               simple_rs_end_y_range_ > 0.0f &&
                               simple_rs_end_theta_range_ > 0.0f;
}

void HybridAstar2::createHeuristicCostFromInside(
    int max_iter, float gear_switch_cost, float min_x, float max_x, float min_y,
    float max_y, const Pose2D &local_frame_pose, bool require_forward,
    const Transform &trans, parking::SearchProcessDebug *sp_debug) {
  float xy_resolution_coarse = float(xy_grid_resolution_);
  float theta_resolution_coarse = float(phi_grid_resolution_);
  int x_count_left = std::ceil(-min_x / xy_resolution_coarse);
  int x_count_right = std::ceil(max_x / xy_resolution_coarse);
  int x_count = x_count_left + x_count_right + 1;
  int y_count_left = std::ceil(-min_y / xy_resolution_coarse);
  int y_count_right = std::ceil(max_y / xy_resolution_coarse);
  int y_count = y_count_left + y_count_right + 1;
  int theta_count_per_side = std::ceil(float(M_PI) / theta_resolution_coarse);
  int theta_count = theta_count_per_side * 2 + 1;

  pq_.clear();
  nodes_pool_.releaseAll();
  state_map_coarse_.setStep(
      {xy_resolution_coarse, xy_resolution_coarse, theta_resolution_coarse});
  state_map_coarse_.setSize({x_count, y_count, theta_count});
  state_map_coarse_.setOrigin(
      {-float(x_count_left) * xy_resolution_coarse,
       -float(y_count_left) * xy_resolution_coarse,
       -float(theta_count_per_side) * theta_resolution_coarse});
  state_map_coarse_.clear();

  float xy_resolution_fine = float(xy_grid_resolution_) * 0.25f;
  float theta_resolution_fine = float(phi_grid_resolution_);
  state_map_fine_end_.setStep(
      {xy_resolution_fine, xy_resolution_fine, theta_resolution_fine});
  float x_min_fine = simple_rs_euler_spiral_min_straight_len_;
  float x_max_fine = simple_rs_euler_spiral_min_straight_len_ + 4.0f;
  float y_min_fine = -0.2f;
  float y_max_fine = 0.2f;
  float theta_min_fine = -float(M_PI) * 0.2f;
  float theta_max_fine = float(M_PI) * 0.2f;
  int x_count_fine =
      std::ceil((x_max_fine - x_min_fine) / state_map_fine_end_.step()[0]);
  int y_count_fine =
      std::ceil((y_max_fine - y_min_fine) / state_map_fine_end_.step()[1]);
  int theta_count_fine = std::ceil((theta_max_fine - theta_min_fine) /
                                   state_map_fine_end_.step()[2]);
  state_map_fine_end_.setSize({x_count_fine, y_count_fine, theta_count_fine});
  state_map_fine_end_.setOrigin({x_min_fine, y_min_fine, theta_min_fine});
  state_map_fine_end_.clear();

  VariableStepSizeStateExpansion state_expansion;
  state_expansion.init(false, min_turn_radius_, xy_resolution_coarse,
                       theta_resolution_coarse, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f,
                       max_zigzag_allowd_, false, 0.0f, 0.0f, 0.0f, false, 0.0f,
                       0.0f, 1);
  state_expansion.setLocalInfo(local_frame_pose, x_bound_, y_bound_);

  SearchNode::Config node_config;
  node_config.traj_forward_penalty = 1.0f;
  node_config.traj_back_penalty = 1.0f;
  node_config.traj_gear_switch_penalty = gear_switch_cost;
  node_config.traj_boundary_cost_penalty = 0.0f;
  node_config.traj_steer_penalty = 0.0f;
  node_config.traj_steer_change_penalty_gear_switch = 0.0f;
  node_config.traj_steer_change_penalty = 0.0f;
  node_config.traj_s_turn_penalty = 0.0f;
  node_config.traj_obstacle_distance_1_penalty = 0.0f;
  node_config.traj_obstacle_distance_2_penalty = 0.0f;
  node_config.traj_obstacle_distance_3_penalty = 0.0f;
  node_config.traj_end_offset_penalty = 0.0f;
  node_config.traj_penalty_start_at_reverse_as_gear_switch = false;

  float max_cost = 50.0f;
  float xy_resolution = float(xy_grid_resolution_) * 2.0f;
  float theta_resolution = float(phi_grid_resolution_) * 3.0f;
  int x_count_map_left = std::ceil(-min_x / xy_resolution);
  int x_count_map_right = std::ceil(max_x / xy_resolution);
  int x_count_map = x_count_map_left + x_count_map_right + 1;
  int y_count_map_left = std::ceil(-min_y / xy_resolution);
  int y_count_map_right = std::ceil(max_y / xy_resolution);
  float min_y_map = -float(y_count_map_left) * xy_resolution;
  int y_count_map = y_count_map_left + y_count_map_right + 1;
  int theta_count_per_side_map = std::ceil(float(M_PI) / theta_resolution);
  int theta_count_map = theta_count_per_side_map * 2 + 1;
  heuristic_cost_from_inside_.init(
      max_cost / 255.0f, -float(x_count_map_left) * xy_resolution,
      -float(y_count_map_left) * xy_resolution,
      -float(theta_count_per_side_map) * theta_resolution, xy_resolution,
      xy_resolution, theta_resolution, x_count_map, y_count_map,
      theta_count_map, false);

  float inflation_shrink = 0.0f;
  float lat_inflation = lat_inflation_ - inflation_shrink;
  float lon_inflation = lon_inflation_ - inflation_shrink;
  int start_node_key = nodes_pool_.allocate().key;
  curve::Curve start_curve;
  start_curve.initAsStart(require_forward, !require_forward);
  SearchNode &start_node = nodes_pool_[start_node_key];
  start_node.init(node_config, &start_curve, 0.0f, 0.0f, 0.0f);
  start_node.setDistanceToObstacle(100.0f);
  pq_.push(start_node_key);
  for (int i = 0; i < max_iter && !pq_.empty(); i++) {
    const SearchNode &current = nodes_pool_[pq_.pop()];
    heuristic_cost_from_inside_.update(
        current.x(), current.y(), current.theta(), current.trajectory_cost());
    const std::vector<SearchNode *> &next = state_expansion.getNextStates(
        node_config, current, mc_footprint_model_, obs_grid_);

    parking::SearchDebugNode debug_curr(
        current.x(), current.y(), current.theta(), current.trajectory_cost(),
        current.heuristic_cost());
    std::vector<parking::SearchDebugEdge> debug_edges_vec;
    if (sp_debug != nullptr) {
      trans.fromSelf(debug_curr);
    }

    for (size_t i = 0; i < next.size(); i++) {
      std::array<float, 3> to_insert{next[i]->x(), next[i]->y(),
                                     next[i]->theta()};
      auto state_insert_res_fine_end =
          state_map_fine_end_.tryInsert(to_insert, *next[i]);
      decltype(state_insert_res_fine_end) state_insert_res_coarse;
      if (!state_insert_res_fine_end.belong) {
        state_insert_res_coarse =
            state_map_coarse_.tryInsert(to_insert, *next[i]);
      }
      bool override_prune = false;
      if (state_insert_res_fine_end.belong) {
        if (!state_insert_res_fine_end.success &&
            SearchNode::isParentOrBrother(
                nodes_pool_[state_insert_res_fine_end.key], *next[i])) {
          override_prune = true;
        }
      } else if (state_insert_res_coarse.belong) {
        if (!state_insert_res_coarse.success &&
            SearchNode::isParentOrBrother(
                nodes_pool_[state_insert_res_coarse.key], *next[i])) {
          override_prune = true;
        }
      }
      if (state_insert_res_coarse.success ||
          state_insert_res_fine_end.success || override_prune) {
        if (checkCollisionUsingMultiCircle(*next[i])) {
          continue;
        }
        auto allocate_res = nodes_pool_.allocate();
        if (!allocate_res.success) {
          continue;
        }
        auto &node = nodes_pool_[allocate_res.key];
        node.copy(*next[i]);
        node.setHeuristicCost(0.0f);

        if (sp_debug != nullptr) {
          parking::SearchDebugNode debug_next(node.x(), node.y(), node.theta(),
                                              node.trajectory_cost(),
                                              node.heuristic_cost());
          trans.fromSelf(debug_next);
          debug_edges_vec.emplace_back(debug_curr, debug_next);
        }

        pq_.push(allocate_res.key);
        int replaced = nodes_pool_.invalidKey();
        if (state_insert_res_fine_end.belong) {
          if (state_insert_res_fine_end.success) {
            replaced = state_map_fine_end_.insert(
                state_insert_res_fine_end.bucket_id,
                state_insert_res_fine_end.offset, allocate_res.key);
          }
        } else if (state_insert_res_coarse.belong) {
          if (state_insert_res_coarse.success) {
            replaced = state_map_coarse_.insert(
                state_insert_res_coarse.bucket_id,
                state_insert_res_coarse.offset, allocate_res.key);
          }
        }
        if (replaced != nodes_pool_.invalidKey() &&
            !SearchNode::isBrother(nodes_pool_[replaced],
                                   nodes_pool_[allocate_res.key])) {
          if (pq_.erase(replaced)) {
            nodes_pool_.release(replaced);
          }
        }
      }
    }
    if (sp_debug != nullptr) {
      sp_debug->added_edges.push_back(debug_edges_vec);
    }
  }
}

} // namespace hybrid_a_star_2

} // namespace msquare
