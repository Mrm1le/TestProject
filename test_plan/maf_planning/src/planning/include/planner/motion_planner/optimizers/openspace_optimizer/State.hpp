#ifndef STATE_HPP
#define STATE_HPP

#include "common/math/box2d.h"
#include <memory>
#include <string>
#include <vector>

namespace msquare {

class SearchNode {
public:
  SearchNode();
  virtual ~SearchNode() = default;
  SearchNode(double x, double y, double theta, double vel = 0, double delta = 0,
             double wheel_base_offset = 0);
  void ComputeStringIndex() {
    index = std::to_string(gx) + "_" + std::to_string(gy) + "_" +
            std::to_string(gtheta);
  }
  std::string GetIndex() { return index; };
  virtual void set_node(double x, double y, double theta, double vel = 0,
                        double delta = 0, double wheel_base_offset = 0);
  virtual void setTrajCost();

  std::string getNodeInfoStr() {
    char tmp[512];
    sprintf(tmp,
            "pose[%f %f %f, %d %d %d] delta[%f], traj_cost=[%f], "
            "heucost=[%f] priority=[%f]",
            x, y, theta, gx, gy, gtheta, delta, trajcost, heuristic_cost_,
            heuristic_cost_ + trajcost);
    return std::string(tmp);
  }

  double x;
  double y;
  double theta;

  // gx, gy and gtheta are co-ordinates in the 80X80 grid
  int gx;
  int gy;
  int gtheta;

  // for running dijkstra
  int dx;
  int dy;

  double delta;
  double vel;
  double cost2d;
  double side_diff = 0.0;
  double trajcost;
  double obs_cost = 0;
  double heuristic_cost_ = 0;
  double wheel_base_offset_ = 0;

  int zigzags = 0;

  std::string index;

  std::shared_ptr<SearchNode> previous = nullptr;

  // d*Lite use
  int num;
  bool open;

  double g_value;
  double rhs_value;

  double change;
  double velocity;

  std::shared_ptr<SearchNode> next = nullptr;

  // box
  std::shared_ptr<planning_math::Box2d> box;
  std::shared_ptr<planning_math::Box2d> box_real;

  std::pair<double, double> key;
};

typedef std::shared_ptr<SearchNode> SearchNodePtr;

} // namespace msquare
#endif
