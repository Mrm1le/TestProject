#include "planner/motion_planner/optimizers/openspace_optimizer/Compare.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include <iostream>
#include <string.h>

namespace msquare {

Compare::Compare() {
  const HybridAstarConfig *config = HybridAstarConfig::GetInstance();
  xy_grid_resolution_ = config->xy_grid_resolution;
  grid_Dijkstra_xy_resolution_ = config->grid_dijkstra_xy_resolution;
  holonomic_with_obs_heuristic_ = config->holonomic_with_obs_heuristic;
  non_holonomic_without_obs_heuristic_ =
      config->non_holonomic_without_obs_heuristic;
}

SearchNode Compare::target;
Pose2D Compare::cmp_frame_pose_;
float **Compare::shortest_2d = nullptr;
int **Compare::grid_obs_map = nullptr;
// std::vector<std::vector<int>> Compare::grid_obs_map;
// std::vector<std::vector<float>> Compare::shortest_2d;
int Compare::DX_ = 0;
int Compare::DY_ = 0;

void Compare::loadFrom(const std::vector<SbpObstaclePtr> &obstacles,
                       const double x_bound, const double y_bound) {
  DX_ = std::max(1, int(ceil(x_bound / grid_Dijkstra_xy_resolution_)));
  DY_ = std::max(1, int(ceil(y_bound / grid_Dijkstra_xy_resolution_)));
  delete[] grid_obs_map;
  // grid_obs_map.resize(DX_);
  grid_obs_map = new int *[DX_];
  for (int i = 0; i < DX_; i++) {
    grid_obs_map[i] = new int[DY_];
    // grid_obs_map[i].resize(DY_);
    for (int j = 0; j < DY_; j++) {
      grid_obs_map[i][j] = 0;
      for (const SbpObstaclePtr &obstacle : obstacles) {
        planning_math::Vec2d temp_grid = msquare::planning_math::tf2d_inv(
            cmp_frame_pose_,
            planning_math::Vec2d((i + 0.5) * grid_Dijkstra_xy_resolution_,
                                 (j + 0.5) * grid_Dijkstra_xy_resolution_));
        if (obstacle->getDistance(temp_grid) <
            sqrt(0.5) * grid_Dijkstra_xy_resolution_)
        // planning_math::Vec2d(i * grid_Dijkstra_xy_resolution_, j *
        // grid_Dijkstra_xy_resolution_)) <
        // sqrt(2)*grid_Dijkstra_xy_resolution_)
        {
          grid_obs_map[i][j] = 1;
          break;
        }
      }
    }
  }
}

bool Compare::operator()(const std::shared_ptr<SearchNode> s1,
                         const std::shared_ptr<SearchNode> s2) {
  // TODO: utilize State->cost3d as second heuristic
  return (s1->trajcost + s1->obs_cost +
          holonomic_with_obs_heuristic_ * holonomic_with_obs(s1) +
          non_holonomic_without_obs_heuristic_ *
              non_holonomic_without_obs(s1)) >
         (s2->trajcost + s2->obs_cost +
          holonomic_with_obs_heuristic_ * holonomic_with_obs(s2) +
          non_holonomic_without_obs_heuristic_ * non_holonomic_without_obs(s2));
}

typedef bool (*compare2dSignature)(SearchNode, SearchNode);

bool compare2d(SearchNode a, SearchNode b) {
  return a.cost2d > b.cost2d; // simple dijkstra
}

// currently uses dijkstra's algorithm in x-y space
float Compare::holonomic_with_obs(std::shared_ptr<SearchNode> src) {
  Pose2D temp_src = msquare::planning_math::tf2d(
      cmp_frame_pose_, Pose2D(src->x, src->y, src->theta));
  return shortest_2d[(int)(temp_src.x / grid_Dijkstra_xy_resolution_)]
                    [(int)(temp_src.y / grid_Dijkstra_xy_resolution_)];
  // return shortest_2d[(int)(src->x /
  // grid_Dijkstra_xy_resolution_)][(int)(src->y /
  // grid_Dijkstra_xy_resolution_)];
}

void Compare::runDijkstra(const double x_bound, const double y_bound) {
  SearchNode src = Compare::target;
  Pose2D temp_tar(target.x, target.y, target.theta);
  temp_tar = msquare::planning_math::tf2d(cmp_frame_pose_, temp_tar);
  SearchNode temp_end(temp_tar.x, temp_tar.y, temp_tar.theta, 0, 0);
  target.dx = temp_end.gx * xy_grid_resolution_ / grid_Dijkstra_xy_resolution_;
  target.dy = temp_end.gy * xy_grid_resolution_ / grid_Dijkstra_xy_resolution_;
  src.dx =
      target.dx; // src.gx * xy_grid_resolution_ / grid_Dijkstra_xy_resolution_;
  src.dy =
      target.dy; // src.gy * xy_grid_resolution_ / grid_Dijkstra_xy_resolution_;

  std::priority_queue<SearchNode, std::vector<SearchNode>, compare2dSignature>
      frontier(&compare2d);
  // int vis[DX_][DY_];

  delete[] shortest_2d;
  shortest_2d = new float *[DX_];
  // shortest_2d.resize(DX_);
  for (int i = 0; i < DX_; i++) {
    shortest_2d[i] = new float[DY_];
    // shortest_2d[i].resize(DY_);
    for (int j = 0; j < DY_; j++)
      shortest_2d[i][j] = 0;
  }

  for (int i = 0; i < DX_; i++)
    for (int j = 0; j < DY_; j++)
      shortest_2d[i][j] = 10000;
  shortest_2d[src.dx][src.dy] = 0;

  frontier.push(src);
  int count = 0;
  while (!frontier.empty()) {
    SearchNode current = frontier.top();
    frontier.pop();

    int x = current.dx;
    int y = current.dy;

    // if (vis[x][y])
    //   continue;

    // vis[x][y] = 1;

    for (int i = -1; i <= 1; i++)
      for (int j = -1; j <= 1; j++) {
        if (x + i < 0 || x + i >= DX_ || y + j < 0 || y + j >= DY_)
          continue;
        if ((i == 0 && j == 0))
          continue;

        if (shortest_2d[x + i][y + j] >
                shortest_2d[x][y] + sqrt(i * i + j * j) ||
            Compare::grid_obs_map[x + i][y + j] != 0) {
          if (Compare::grid_obs_map[x + i][y + j] == 0) // no obstacle
          {
            shortest_2d[x + i][y + j] =
                shortest_2d[x][y] +
                sqrt(i * i + j * j) * grid_Dijkstra_xy_resolution_;
          }
          SearchNode tempstate;
          tempstate.dx = current.dx + i;
          tempstate.dy = current.dy + j;
          tempstate.cost2d = shortest_2d[x + i][y + j];
          frontier.push(tempstate);
        }
      }
  }
}

float Compare::non_holonomic_without_obs(std::shared_ptr<SearchNode> src) {
  return src->heuristic_cost_;
}

} // namespace msquare
