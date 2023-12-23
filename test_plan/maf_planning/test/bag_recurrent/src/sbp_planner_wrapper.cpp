#include "sbp_planner_wrapper.h"
#include "parking_scenario/parking_scenario.h"
#include "pnc/define/geometry.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/hybrid_a_star_2.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/perpendicular_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/perpendicular_out_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/parallel_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/parallel_out_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/pattern_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/oblique_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/rpa_straight_planner.h"


BagAnalysis::BagAnalysis(){
  std::string car_param_file = "../resources/vehicle_param_l.yaml";
  std::string config_file_name = "../resources/apa_parallel.yaml";

  msquare::HybridAstarConfig::GetInstance()->loadFile(config_file_name);
  msquare::TrajectoryOptimizerConfig::GetInstance()->loadFile(config_file_name);
  msquare::CarParams::GetInstance()->loadFile(car_param_file);
  msquare::CarParams::GetInstance()->loadFile4Plan(config_file_name);
  msquare::StrategyParams::GetInstance()->loadFile(config_file_name);
  msquare::VehicleParam::Instance()->loadFile(car_param_file);
  msquare::CarParams::GetInstance()->loadFile4Car(parking_scenario::car_param_road);

  // initialize params
  car_size_params_.reset();

  astar2_solver_ = std::make_shared<msquare::hybrid_a_star_2::HybridAstar2>();
}

void BagAnalysis::loadBagAndAnalyze(std::vector<std::string> param_strs){
  param_string_ = param_strs;
  bag_odos_.clear();
  bag_results_.clear();
  bag_sp_debugs_.clear();

  extractEnvironmentInfos();
  calcBagResults();
}

void BagAnalysis::extractEnvironmentInfos() {
  planner_params_.clear();
  for(auto param_str : param_string_) {
    msquare::parking::OpenspaceDeciderOutput odo;
    feedRequest(param_str, odo);
    msquare::parking::AstarPlannerParams planner_params{};
    planner_params.reset();
    planner_params_.push_back(planner_params);
    bag_odos_.push_back(odo);
  }

  car_size_params_.reset();
}

void BagAnalysis::calcBagResults() {
  for(auto param_str : param_string_) {
    msquare::parking::OpenspaceDeciderOutput odo;
    feedRequest(param_str, odo);

    msquare::parking::SearchProcessDebug* sp_debug = new msquare::parking::SearchProcessDebug();
    std::cout << "begin plan" << std::endl;
    bag_results_.push_back(plan(odo, sp_debug));
    bag_sp_debugs_.push_back(sp_debug);
  }
}

std::vector<std::string> BagAnalysis::getParamString(){
  return param_string_;
}


std::vector<std::string> BagAnalysis::getBagEnvironmentInfos() {
  for(auto odo : bag_odos_) {
    nlohmann::json json_odo = odo;
    odo_output_.push_back(json_odo.dump());
  }
  return odo_output_;
}
std::vector<std::string> BagAnalysis::getBagResults() {
  for(auto result : bag_results_) {
    nlohmann::json json_res = result;
    result_output_.push_back(json_res.dump());
  }
  return result_output_;
}
std::vector<msquare::parking::SearchProcessDebug*> BagAnalysis::getBagResDebugs() {
  return bag_sp_debugs_;
}

std::string BagAnalysis::getCarSizeParams(){
  nlohmann::json car_param_json = car_size_params_;
  return car_param_json.dump();
}

std::string BagAnalysis::getPlannerParams(){
  nlohmann::json planner_params_json = planner_params_;
  return planner_params_json.dump();
}

std::string BagAnalysis::plan(std::string planner_str, msquare::parking::SearchProcessDebug *sp_debug){
  nlohmann::json planner_input_json = nlohmann::json::parse(planner_str);
  msquare::parking::AstarPlannerInput planner_input = planner_input_json;

  msquare::parking::OpenspaceDeciderOutput temp_odo;
  feedRequest(planner_input.param_string, temp_odo);


  if(planner_input.car_type == "devcar"){
    std::cout<<"planning for devcar ..."<<std::endl;
    msquare::parking::loadDevcarParams();
  }else if(planner_input.car_type == "epcar"){
    std::cout<<"planning for epcar ..."<<std::endl;
    msquare::parking::loadEpcarParams();
  }

  planner_input.params.update();

  msquare::parking::OpenspaceDeciderOutput odo = planner_input.odo;

  msquare::SbpResult planner_res = plan(odo, sp_debug);
  msquare::parking::CarSizeParams car_params{};
  car_params.reset();
  msquare::parking::AstarPlannerOutput planner_out{};
  planner_out.sbp_result = planner_res;
  planner_out.car_params = car_params;
  nlohmann::json json_result = planner_out;
  return json_result.dump();
}

msquare::SbpResult
BagAnalysis::plan(msquare::parking::OpenspaceDeciderOutput odo,
                  msquare::parking::SearchProcessDebug *sp_debug) {
  using msquare::hybrid_a_star_2::HybridAstar2;

  msquare::SbpResult planner_res;

  std::shared_ptr<msquare::SearchBasedPlanner> solver;
  if (HybridAstar2::planningCoreSupported(
          msquare::HybridAstarConfig::GetInstance()->planning_core)) {
    astar2_solver_->setSlotTypeByPlanningCore(
        msquare::HybridAstarConfig::GetInstance()->planning_core);
    solver = astar2_solver_;
    solver->Update(odo.map_boundary);
  } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 2) {
    solver = std::make_shared<msquare::PatternPlanner>(odo.pattern_path);
  } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 1) {
    solver = std::make_shared<msquare::PerpendicularRulePlanner>(odo);
  } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 3) {
    solver = std::make_shared<msquare::ParallelRulePlanner>(odo);
  } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 4) {
    solver = std::make_shared<msquare::PerpendicularOutRulePlanner>(odo);
  } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 5) {
    solver = std::make_shared<msquare::ParallelOutRulePlanner>(odo);
  } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 6) {
    solver = std::make_shared<msquare::ObliqueRulePlanner>(odo);
  } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 7) {
    solver = std::make_shared<msquare::RpaStraightPlanner>(odo);
  } else {
    // throw std::logic_error("unsupported planner!");
  }
  msquare::parking::genOpenspacePath(solver, odo, planner_res, sp_debug);

  if (planner_res.x.empty()) {
    std::cout << "Sbp method fails!!!!!!!!" << std::endl;
  } else {
    std::cout << "Sbp method succeeds!!!!!!!!!" << std::endl;
  }

  std::vector<msquare::planning_math::Vec2d> global_points{};
  solver->get_global_env_points(global_points);
  std::cout << "global_points size: " << global_points.size() << std::endl;

  nlohmann::json debug_json;
  debug_json["environment_of_slot"] = global_points;
  planner_res.debug_string = debug_json.dump();

  return planner_res;
}