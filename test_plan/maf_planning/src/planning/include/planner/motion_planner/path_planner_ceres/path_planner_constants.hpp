#pragma once
#include "common/utils/gauss_quad_constants.h"
#include "pnc/define/planner_constants.hpp"
namespace path_planner {

static constexpr size_t TOTAL_QUAD_POINTS =
    NUM_PATH_SEGMENTS * QUADRATURE_ORDER;

// planner tuning parameters
static constexpr double MAX_TIME_TO_CENTER_TO_PREV_PLAN_S = 1.5;
static constexpr double START_REF_CENTER_SCALE = 0.4; // at ego_s
static constexpr double START_REF_CENTER_SCALE_INCREASE_PROPORTION = 1.0 / 5.0;
static constexpr double START_REF_CENTER_SCALE_DECREASE_PROPORTION = 2.0 / 5.0;
static constexpr double REF_CENTER_SCALE_INCREASE_RATE = 3.0;
static constexpr double REF_CENTER_SCALE_DECREASE_RATE = 4.0;

static constexpr int32_t MAX_NUM_ITERATIONS = 40;
static constexpr double SOFT_BOUNDARY_SCALE = 2.8;
static constexpr double HARD_BOUNDARY_SCALE = 100.0;
static constexpr double LAT_ACCEL_SCALE = 0.3;
static constexpr double HEADING_SCALE = 95.0;
static constexpr double REF_CENTERING_SCALE = 15.0;
static constexpr double PREV_PLAN_CENTERING_SCALE = 2.0;
static constexpr double OBSTACLE_CONSTRAIN_SCALE = 500.0;
static constexpr double OBSTACLE_DESIRE_SCALE = 30.0;
static constexpr double OBSTACLE_INFLATION_SCALE = 5.0;

// initial curvature setting
static constexpr double INIT_CURVATURE_SCALE = 1000.0;
static constexpr double INIT_NORMAL_CURVATURE_RATIO = 0.1;
static constexpr double INIT_CURV_LIMIT_COST_SCALE = 100.;
static constexpr double INIT_CURV_ERROR_LIMIT = 0.1;

// map boundary setting
static constexpr double LANE_LINE_ACTIVATION_DIST_M = 1.70;
static constexpr double ROAD_BORDER_AVD_DIST_SOFT = 0.5;
static constexpr double ROAD_BORDER_AVD_DIST_HARD = 0.4;
static constexpr double ROAD_BORDER_AVD_SPEED_COEF = 0.02;
static constexpr double ROAD_BORDER_AVD_LAT_ACCEL_COEF = 0.4;
static constexpr double LANE_BORDER_AVD_DIST_M = 0.3;

// limit jerk when enter auto drive or at intersection
static constexpr double JERK_LIMIT_TIME = 3.0;
static constexpr double MAX_JERK_FIRST_START_AUTO_DRIVE = 4.0;
static constexpr double JERK_IN_INTERSECTION = 8.0;
static constexpr double NORMAL_JERK = 0.2;
static constexpr double KEEP_IN_INTERSECTION_TIME = 5.0;

// limit jerk when center line jump
static constexpr double CENTER_JUMP_JERK_MAX = 8.0;
static constexpr double JERK_DECAY_RATE = 1.6;
static constexpr double CENTER_JUMP_RANGE_MIN = 0.1;
static constexpr double CENTER_JUMP_RANGE_MAX = 3.0;

// lat jerk limit cost
static constexpr double LAT_JERK_LIMIT_COST_SCALE = 1000.0;
static constexpr double LAT_JERK_LIMIT_MPS3 = 1.9;

// curvature rate limit cost
static constexpr double CURVATURE_RATE_SCALE = 20.;
static constexpr double CURVATURE_RATE_LIMIT = 0.04;
static constexpr double CURVATURE_RATE_LIMIT_SCALE = 1000.0;

// limit curvature rate when enter auto drive
static constexpr double CURVATURE_RATE_LIMIT_TIME = 3.0;
static constexpr double MAX_CURVATURE_RATE_FIRST_START_AUTO_DRIVE = 20.0;

// lane change parameters
static constexpr double LCQuinticPolySampleT = 0.5;
static constexpr double LAT_VEL_DESIRE = 4. / 5.5;

// obstacle inflation parameters
static constexpr size_t OBSTACLE_HISTORY_INFO_SIZE = 5;
static constexpr double INFLATION_JERK_BASE = 1.0;
static constexpr double INFLATION_JERK_MAX = 3.0;
static constexpr double INFLATION_JERK_MIN = 0.5;
static constexpr double OBSTACLE_CONFIDENCE = 0.3;
static constexpr double AVD_WIDTH_DESIRE = 0.8;
static constexpr double AVD_WIDTH_DESIRE_VRU = 1.0;
static constexpr double AVD_WIDTH_DESIRE_INCREASE_RATE_VS_VEL = 0.015;
static constexpr double AVD_WIDTH_CONSTRAIN = 0.2;
static constexpr std::pair<double, double> PredViladTimeMaxVSVel(1., 3.0);
static constexpr std::pair<double, double> PredViladTimeMinVSVel(10., 1.0);

static constexpr double LON_INFLATION_BASE = 1.0;
static constexpr double LON_INFLATION_TIME_LIMIT = 2.0;
static constexpr double PRESS_LANE_AVD_BUFFER_MAX = 0.2;

// active offset params
static constexpr double TruckOffsetRef = 0.3;
static constexpr double OffsetLaneBoundLimit = 0.3;
static constexpr double OffsetRoadBoundLimit = 0.7;
static constexpr double StartOffsetLatDis = 2.0;
static constexpr double SaturationOffsetLatDis = 1.5;
static constexpr double StartoffsetCurvature = 0.004;
static constexpr double EndoffsetCurvature = 0.005;
static constexpr double TruckLengthMin = 6.5;
static constexpr size_t COOLING_TIME = 50;
static constexpr size_t CONTROL_TIME = 30;
static constexpr size_t LANE_CHANGE_TIME = 30;
static constexpr size_t CP_TIME = 30;
static constexpr size_t MAX_TIME = 100;
static constexpr size_t END_DEBOUNCE_TIME = 3;
static constexpr double AVD_PLAN_LEN = 200.0;
static constexpr double AVD_LB_LIMIT = 0.2;
static constexpr double AVD_RB_LIMIT = 0.5;

// ddp key params
static constexpr std::pair<double, double> PredViladTimeMaxVSVelDDP(1., 3.0);
static constexpr std::pair<double, double> PredViladTimeMinVSVelDDP(10., 2.0);

// init offset
static constexpr double MAX_REF_JERK_V = 80. / 3.6;
static constexpr double MAX_REF_JERK = 2.0;
static constexpr double MAX_REF_JERK_INTERSECTION = 0.5;

} // namespace path_planner