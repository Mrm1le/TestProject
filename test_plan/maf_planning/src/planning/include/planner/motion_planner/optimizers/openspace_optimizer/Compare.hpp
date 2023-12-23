#ifndef COMPARE_HPP
#define COMPARE_HPP

#include "State.hpp"
#include "common/math/math_utils.h"
#include "sbp_obstacle_interface.h"
#include <memory>
#include <queue>
#include <string>
#include <vector>

namespace msquare {

class Compare {
private:
  float xy_grid_resolution_;
  float grid_Dijkstra_xy_resolution_;
  float holonomic_with_obs_heuristic_;
  float non_holonomic_without_obs_heuristic_;

public:
  static SearchNode target;
  static int **grid_obs_map;
  static float **shortest_2d;
  // static std::vector<std::vector<int>> grid_obs_map;
  // static std::vector<std::vector<float>> shortest_2d;
  static int DX_;
  static int DY_;

  static Pose2D cmp_frame_pose_;
  Compare();
  void loadFrom(const std::vector<SbpObstaclePtr> &obstacles,
                const double x_bound, const double y_bound);

  bool operator()(const std::shared_ptr<SearchNode> s1,
                  std::shared_ptr<SearchNode> s2); // search

  float non_holonomic_without_obs(std::shared_ptr<SearchNode> src); // search
  float holonomic_with_obs(std::shared_ptr<SearchNode> src);        // search

  void runDijkstra(const double x_bound, const double y_bound);
};

} // namespace msquare

#endif
