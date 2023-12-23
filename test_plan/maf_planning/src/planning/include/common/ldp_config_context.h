#pragma once

#include "common/utils/macro.h"
#include "nlohmann/json.hpp"
#include <fstream>

namespace msquare {

struct LkaConfig {
  double K_lka_inner_dlc_thr = 0.45;
  double K_lka_inner_tlc_line_thr = 0.25;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LkaConfig, K_lka_inner_dlc_thr,
                                   K_lka_inner_tlc_line_thr)

struct LdpPlannerConfig {
  LkaConfig lka_config;
  LkaConfig elk_config;
  LkaConfig absm_config;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LdpPlannerConfig, lka_config, elk_config,
                                   absm_config)

class LdpConfigurationContext {
private:
  // this is a singleton class
  DECLARE_SINGLETON(LdpConfigurationContext);

public:
  const LdpPlannerConfig &ldp_planner_config() { return ldp_planner_config_; }

  void load_ldp_planner_config(const std::string &config_file_dir) {
    std::string config_file_name = "ldp_planner_config.json";
    std::ifstream fjson(config_file_dir +
                        "/scenario_configs_json/planner_config/" +
                        config_file_name);
    std::string json_str((std::istreambuf_iterator<char>(fjson)),
                         std::istreambuf_iterator<char>());
    nlohmann::json input_json = nlohmann::json::parse(json_str);
    ldp_planner_config_ = input_json;
  }

private:
  LdpPlannerConfig ldp_planner_config_{};
};

} // namespace msquare
