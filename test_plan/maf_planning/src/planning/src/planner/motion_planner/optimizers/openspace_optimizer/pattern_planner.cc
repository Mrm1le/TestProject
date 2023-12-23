#include "planner/motion_planner/optimizers/openspace_optimizer/pattern_planner.h"
#include "nlohmann/json.hpp"
#include <fstream>

namespace msquare {

inline SbpResult tf2d_inv(Pose2D local_frame_pose, SbpResult sbp_result) {
  using namespace planning_math;
  SbpResult res = {};
  for (int i = 0; i < sbp_result.x.size(); ++i) {
    Pose2D pose_local{sbp_result.x.at(i), sbp_result.y.at(i),
                      sbp_result.phi.at(i)};
    Pose2D pose = tf2d_inv(local_frame_pose, pose_local);
    res.x.push_back(pose.x);
    res.y.push_back(pose.y);
    res.phi.push_back(pose.theta);

    res.wheel_base_offset.push_back(sbp_result.wheel_base_offset.at(i));
  }
  return res;
}

PatternPlanner::PatternPlanner(const std::string &pattern_file)
    : pattern_file_(pattern_file) {}

PatternPlanner::~PatternPlanner() {}

void PatternPlanner::Update(
    const planning_math::Box2d map_boundary,
    const std::vector<planning_math::LineSegment2d> &map) {}

bool PatternPlanner::Plan(const std::vector<SbpObstaclePtr> &obs_ptrs,
                          parking::SearchProcessDebug *sp_debug) {
  std::fstream fs;
  // fs.open(pattern_file_, std::ios_base::openmode::_S_in);
  fs.open(pattern_file_, std::ios::in);
  nlohmann::json json;
  fs >> json;
  fs.close();
  json["debug_string"] = "";
  SbpResult local_result = json;
  std::vector<double> r_copy;
  r_copy.resize(local_result.x.size());
  std::reverse_copy(local_result.x.begin(), local_result.x.end(),
                    r_copy.begin());
  local_result.x.insert(local_result.x.end(), r_copy.begin(), r_copy.end());
  std::reverse_copy(local_result.y.begin(), local_result.y.end(),
                    r_copy.begin());
  local_result.y.insert(local_result.y.end(), r_copy.begin(), r_copy.end());
  std::reverse_copy(local_result.phi.begin(), local_result.phi.end(),
                    r_copy.begin());
  local_result.phi.insert(local_result.phi.end(), r_copy.begin(), r_copy.end());
  std::reverse_copy(local_result.wheel_base_offset.begin(),
                    local_result.wheel_base_offset.end(), r_copy.begin());
  local_result.wheel_base_offset.insert(local_result.wheel_base_offset.end(),
                                        r_copy.begin(), r_copy.end());
  Pose2D start_pose(start_node_->x, start_node_->y, start_node_->theta);
  result_ = tf2d_inv(start_pose, local_result);
  return true;
}

SbpResult PatternPlanner::getResult() { return result_; }

std::vector<Pose2D> PatternPlanner::getSearchPoints() {
  return std::vector<Pose2D>();
}

} // namespace msquare
