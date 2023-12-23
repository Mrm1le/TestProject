#ifndef PYBIND_PARKING_SCENARIO
#define PYBIND_PARKING_SCENARIO

#include "common/config/vehicle_param.h"
#include "common/parking_planner_types.h"
#include "common/sbp_strategy.h"
#include "nlohmann/json.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star.h"

namespace parking_scenario {

struct Point2d {
  double x;
  double y;
  double theta;
  Point2d() : x(0.0), y(0.0), theta(0.0) {}
  Point2d(double _x, double _y, double _theta = 0) : x(_x), y(_y), theta(_theta) {}
};

struct PlanResult {
  Point2d point;
  bool is_success = false;
  std::string debug_info;
};

using BatchResult = std::vector<PlanResult>;

struct ScenarioResult {
  BatchResult br;
  std::vector<std::string> debug_info;
};

enum ScenarioType {
  PARALLEL = 0,
  VERTICAL_BASE,
  VERTICAL_SINGLE_CAR,
  OBLIQUE_SLOT,
  PARALLEL_OUT,
  VERTICAL_BASE_OUT,
  VERTICAL_SINGLE_CAR_OUT,
  OBLIQUE_OUT,
};
static std::string car_param_road = "../../../resource/config/scenario_configs_json";

struct PlanParams {
  // constant
  const double BOUNDARY_WIDTH = 20.0;
  const double BOUNDARY_MARGIN = 1.0;
  const double SLOT_DEPTH_MARGIN = 0.2;

  // change not so often
  std::string apa_file = "/home/ros/catkin_ws/src/maf_planning/"
                         "resource/config/scenario_configs_json/"
                         "parking/apa.yaml";
  double step = 0.4;
  double left_space = 2.0;
  double right_space = 2.0;
  double top_space = 2.0;
  double bottom_space = 2.0;
  double theta = 0.0;   // default theta
  double oblique_angle = 0.0;
  bool left_blocked = false;
  bool right_blocked = false;
  double opposite_width = 0.0;

  double channel_width;

  // tuned params, change frequently
  std::string vehicle_param_file;
  double slot_margin; // parallel is margin, vertical is slot lemgth
  ScenarioType scenario_type;
};


ScenarioResult scenarioPlan(const PlanParams& params, bool use_multi_process = false);
ScenarioResult extractResult(const BatchResult& br, msquare::parking::OpenspaceDeciderOutput& odo);
BatchResult singleThreadPlan(msquare::parking::OpenspaceDeciderOutput& odo,
    const std::vector<Point2d>& init_points);
BatchResult multiThreadPlan(msquare::parking::OpenspaceDeciderOutput& odo,
    const std::vector<Point2d>& init_points);
BatchResult multiProcessPlan(msquare::parking::OpenspaceDeciderOutput& odo,
    const std::vector<Point2d>& init_points);


void apaPlan(const msquare::parking::OpenspaceDeciderOutput odo,
             msquare::SbpResult &sbp_result,
             msquare::parking::SearchProcessDebug *sp_debug = nullptr);
void loadPlanParams(const std::string &apa_file, const std::string &vehicle_param_file);
void generateParallelScenario(
    const PlanParams& params, 
    msquare::parking::OpenspaceDeciderOutput& odo,
    std::vector<Point2d>& init_points
);

void generateVerticalScenario(
    const PlanParams& params, 
    msquare::parking::OpenspaceDeciderOutput& odo,
    std::vector<Point2d>& init_points
);
void generateVerticalScenario2(
    const PlanParams& params, 
    msquare::parking::OpenspaceDeciderOutput& odo,
    std::vector<Point2d>& init_points
);
void generateObliqueScenario(
    const PlanParams &params,
    msquare::parking::OpenspaceDeciderOutput &odo,
    std::vector<Point2d> &init_points
);

void extractSingleResult(const msquare::SbpResult &sbp_result,
             const msquare::parking::SearchProcessDebug *sp_debug,
             PlanResult &plan_res
);

} // namespace parking_scenario

#endif
