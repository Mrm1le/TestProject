#pragma once
#include <nlohmann/json.hpp>
// #include "common/world_model.h"
// #include "planner/tasks/deciders/decider.h"
namespace msquare {
struct LateralDecisionDataOutput {
public:
  explicit LateralDecisionDataOutput() = default;
  virtual ~LateralDecisionDataOutput(){};
  bool gap_valid_logic = false;
  bool gap_inserable_logic = false;
  int gap_first_id_scalar = -1;
  int gap_second_id_scalar = -1;
  std::vector<double> gap_first_obj_v3 = {};
  std::vector<double> gap_second_obj_v3 = {};

  NLOHMANN_DEFINE_TYPE_INTRUSIVE(LateralDecisionDataOutput, gap_valid_logic)

};
} // namespace msquare

