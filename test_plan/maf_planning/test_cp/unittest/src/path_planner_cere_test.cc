#include "gtest/gtest.h"
#include <fstream>
#include "planner/motion_planner/path_planner_ceres/path_planner_ceres.hpp"

namespace path_planner {

json get_json_from(std::string json_test_file) {
  std::string json_string =
      std::string(TEST_DATA_DIR) + json_test_file;
  std::ifstream input_stream(json_string);
  json json_test;
  input_stream >> json_test;
  return json_test;
}

TEST(PATH_PLANNER_TEST, PATH_PLANNER_CERE_TEST) {
  NotebookDebug nb;
  path_planner::PathPlanner path_planner;
  path_planner::PathPlannerOutput path_planner_output;
  path_planner::PathPlannerInput path_planner_input;
  path_planner::PathPlannerPoint out_point;
  
  // lane keep test case
  auto json_test = get_json_from("path_planner_test.json");
  path_planner_input = json_test;
  path_planner.update_input(path_planner_input, true, &nb);
  path_planner.get_output(path_planner_output);
  path_planner.get_output_at_s(&path_planner_input, out_point,
                               path_planner_input.planning_init_state.s + 1.0);
  EXPECT_FALSE(nb.s_to_samples.empty());

  // lat avid obstacle test case
  auto json_test_nudge = get_json_from("path_planner_test_nudge.json");
  path_planner_input = json_test_nudge;
  nb.clear();
  path_planner.update_input(path_planner_input, true, &nb);
  path_planner.get_output(path_planner_output);
  path_planner.get_output_at_s(&path_planner_input, out_point,
                               path_planner_input.planning_init_state.s + 1.0);
  EXPECT_TRUE(path_planner_input.obs_list.size() > 0);
  
  // lane change test case
  auto json_test_lane_change = get_json_from("path_planner_test_lane_change.json");
  path_planner_input = json_test_lane_change;
  nb.clear();
  path_planner.update_input(path_planner_input, true, &nb);
  path_planner.get_output(path_planner_output);
  path_planner.get_output_at_s(&path_planner_input, out_point,
                               path_planner_input.planning_init_state.s + 1.0);
  EXPECT_TRUE(path_planner_input.lc_decider_info.is_lane_change);

}
}
