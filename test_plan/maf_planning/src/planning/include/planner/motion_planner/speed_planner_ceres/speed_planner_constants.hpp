#pragma once
#include "pnc/define/planner_constants.hpp"
#include <vector>

namespace speed_planner {

// planner tuning parameters
static constexpr int32_t MAX_NUM_ITERATIONS = 40;
static constexpr double JERK_MAX_LIMIT = 1.5;
static constexpr double JERK_MIN_LIMIT = -3.0;
static constexpr double ACCELERATION_MAX_LIMIT = 2.0;
static constexpr double ACCELERATION_MIN_LIMIT = -6.0;

// accel jerk params
static constexpr double MAX_ACC = 2.0;
static constexpr double MAX_ACC_DECREASE_RATE = 1.0 / 30.0;
static constexpr double ACCEL_JERK_RATIO = 3.0;
static constexpr double ACCELERATION_LIMIT_SCALE = 50.0;
static constexpr double ACCELERATION_SCALE = 0.3;
static constexpr double JERK_LIMIT_SCALE = 30.0;
static constexpr double JERK_SCALE = 3.0;

static constexpr double MAX_TIME_LIMIT_COMFORT_PLAN_S = 1.0;
static constexpr double ACTIVATION_LIMIT_COMFORT_START_ACCELERATION = 1.5;
static constexpr double ACTIVATION_LIMIT_COMFORT_END_ACCELERATION = 3.0;
static constexpr double ACTIVATION_LIMIT_COMFORT_START_JERK = 2.0;
static constexpr double ACTIVATION_LIMIT_COMFORT_END_JERK = 3.0;

// stop point params
static constexpr double STOP_POINT_SCALE = 30.0;

// model ref speed params
static constexpr double MODEL_REF_SCALE = 10.0;
static constexpr double MODEL_SAFETY_SCALE = 30.0;
static constexpr double GMP_REF_SCALE = 3.6;

// headway params
static const std::vector<double> AGGRESSIVE_HEADWAY_TABLE{1.2, 1.4, 1.6, 1.8,
                                                          2.0};
static const std::vector<double> NORMAL_HEADWAY_TABLE{1.2, 1.5, 2.0, 2.5, 3.0};
static const std::vector<double> CONSERVATIVE_HEADWAY_TABLE{1.2, 1.8, 2.5, 3.0,
                                                            4.0};
// static const std::vector<double> HEADWAY_MIN_MAX_TABLE{1.0, 3.6};
// static const std::vector<double> HEADWAY_1_0_TABLE{0, 6, 17, 23, 31, 56};
// static const std::vector<double> HEADWAY_3_6_TABLE{0, 8, 42, 80, 106, 106};
// static const std::vector<double> EGO_SPEED_TABLE{0, 2.6, 17, 25, 35, 60};
static const double LANE_CHANGE_HEADWAY_MIN = 0.8;
static const double LANE_CHAGNE_HEADWAY_MAX = 4.0;

// stop offset
static const double COUPLE_STOP_OFFSET = 4.5;
static const double TRANSPORT_TRUNK_STOP_OFFSET = 4.5;
static const double VRU_STOP_OFFSET = 4.5;
static const double EFTP_STOP_OFFSET = 3.5;

// acc params
static constexpr double MAX_VALID_SPEED_MPS = 100.0;
static constexpr double MAX_ACC_ACCEL_MPS2 = 3.0;
static constexpr double MAX_ACC_DECEL_MPS2 = 3.5;

static constexpr double SAFE_V_LIMIE_SCALE = 10.0;
static constexpr double ACCEL_CEILING_SCALE = 1.0;
static constexpr double CRUISE_SCALE = 1.0;
static constexpr double DEADBAND_REGION_RATIO = 0.5;
static constexpr double DEADBAND_LIMIT_SCALE = 5.0;
static constexpr double ACCEL_CEILING_COST_SCALE = 1.0;
static constexpr double ACC_DECEL_REGION_SCALE = 10.0;
static constexpr double ACC_ACCEL_REGION_SCALE = 2.0;
static constexpr double ACC_VMAX_SCALE = 10.0;

// accel and jerk bound info
static const std::vector<double> AGGRESSIVE_A_MAX_VALUE_PT = {2.5, 1.9, 1.5,
                                                              1.3, 0.9};
static const std::vector<double> NORMAL_A_MAX_VALUE_PT = {1.7, 1.4, 1.0, 0.8,
                                                          0.6};
static const std::vector<double> CONSERVATIVE_A_MAX_VALUE_PT = {1.2, 1.0, 0.7,
                                                                0.6, 0.4};
static const std::vector<double> A_MAX_SPEED_PT = {5, 10, 20, 25, 35};
static const std::vector<double> A_MIN_VALUE_PT = {-4.4, -3.08};
static const std::vector<double> A_MIN_SPEED_PT = {5, 20};
static const std::vector<double> J_MIN_VALUE_PT = {-5, -5};
static const std::vector<double> J_MIN_SPEED_PT = {5, 20};
static const std::vector<double> J_MAX_VALUE_PT = {10, 10};
static const std::vector<double> J_MAX_SPEED_PT = {5, 20};
static const std::vector<double> J_MIN_VALUE_ACCORDING_TIME_PT = {-5, -5};
static const std::vector<double> J_MIN_TIME_PT = {0, 1};
static const double J_MIN_PRE_BRAKING = -2.0;

static constexpr double ENALE_ACC_DECEL_REGION_TIME_AT_EFTP_MODE = 0.5;
} // namespace speed_planner