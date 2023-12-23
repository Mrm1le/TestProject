#ifndef PLANNER_INTERFACE
#define PLANNER_INTERFACE

#include "nlohmann/json.hpp"
#include "util.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "common/config/vehicle_param.h"
#include "common/parking_planner_types.h"
#include "common/sbp_strategy.h"
#include "parking_scenario/parking_scenario.h"


namespace planner_interface{


void initSingletonParams(const std::string& car_params_file, const std::string& config_param_file);

const std::string getCarSizeParams();
const std::string getAstarPlannerParams();

std::string planInterfaceSerializeParams(std::string odo_with_params_str);

std::string planInterfaceSerialize(std::string ods_str);


std::string planInterfaceStr(msquare::parking::OpenspaceDeciderOutput odo);

/**
 * @brief planner  base caller
 * 
 * @param odo 
 * @return msquare::SbpResult 
 */
msquare::SbpResult planInterface(msquare::parking::OpenspaceDeciderOutput odo);

void checkCollision(msquare::SbpResult& sbp_res, const msquare::parking::OpenspaceDeciderOutput& odo);

double getEgoMinObsDistance(const parking_scenario::Point2d& ego_pose, 
    const std::string& odo_str);

msquare::planning_math::Polygon2d getEgoPolygon(const Pose2D& ego_pose, double lat = 0.0, double lon=0.0);

std::vector<parking_scenario::Point2d> getEgoCorners(const parking_scenario::Point2d& ego_pose, double lat = 0.0, double lon=0.0);
Pose2D convertPose(const parking_scenario::Point2d& ego_pose);
parking_scenario::Point2d convertPose(const Pose2D& ego_pose);

} // end namespace planner_interface

#endif