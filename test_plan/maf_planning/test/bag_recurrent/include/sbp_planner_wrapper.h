#ifndef SBP_PLANNER_WRAPPER
#define SBP_PLANNER_WRAPPER

#include "nlohmann/json.hpp"
#include "util.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/hybrid_a_star_2.h"
#include "common/config/vehicle_param.h"
#include "common/parking_planner_types.h"
#include "common/sbp_strategy.h"


class BagAnalysis {
public:
  BagAnalysis();

  void loadBagAndAnalyze(std::vector<std::string> param_strs);
  void extractEnvironmentInfos();

  void calcBagResults();

  std::string getCarSizeParams();
  std::string getPlannerParams();

  std::string plan(std::string ods_str,
                   msquare::parking::SearchProcessDebug *sp_debug = nullptr);
  msquare::SbpResult
  plan(msquare::parking::OpenspaceDeciderOutput odo,
       msquare::parking::SearchProcessDebug *sp_debug = nullptr);

  std::vector<std::string> getBagEnvironmentInfos();
  std::vector<std::string> getBagResults();
  std::vector<std::string> getParamString();
  std::vector<msquare::parking::SearchProcessDebug*> getBagResDebugs();

private:  
  std::vector<std::string> param_strs_;
  std::vector<msquare::SbpResult> bag_results_;
  std::vector<msquare::parking::SearchProcessDebug*> bag_sp_debugs_;

  std::vector<msquare::parking::OpenspaceDeciderOutput> bag_odos_;
  std::vector<std::string> result_output_;
  std::vector<std::string> odo_output_;
  std::vector<std::string> param_string_;

  msquare::parking::CarSizeParams car_size_params_;
  std::vector<msquare::parking::AstarPlannerParams> planner_params_;

  std::shared_ptr<msquare::hybrid_a_star_2::HybridAstar2> astar2_solver_;
};

#endif // SBP_PLANNER_WRAPPER


