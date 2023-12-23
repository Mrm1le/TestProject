#pragma once

#include "common/math/vec2d.h"
#include "pnc/define/geometry.h"
#include <climits>
#include <vector>

namespace msquare {

struct Channel {
  double x1 = 0;
  double x2 = 0;
  double dx12 = 0;
  double x3 = 0;
  double x4 = 0;
  double dx34 = 0;
  double y1 = 0;
  double y2 = 0;
  double dy12 = 0;
  bool inited = false;
  void reset() {
    double x1 = 0;
    double x2 = 0;
    double dx12 = 0;
    double x3 = 0;
    double x4 = 0;
    double dx34 = 0;
    double y1 = 0;
    double y2 = 0;
    double dy12 = 0;
    bool inited = false;
  }
};

struct Environment {
  Environment() = default;
  double channel_width = 0;
  double slot_width = 0;
  double left_slot_width = 0;
  double right_slot_width = 0;
  double left_slot_obs_height = 0;
  double right_slot_obs_height = 0;
  std::map<int, Channel> channels{};
  std::vector<planning_math::Vec2d> top_points{};
  std::vector<planning_math::Vec2d> bottom_points{};
  bool inited = false;
  void reset() {
    channel_width = 0;
    slot_width = 0;
    left_slot_width = 0;
    right_slot_width = 0;
    left_slot_obs_height = 0;
    right_slot_obs_height = 0;
    channels.clear();
    top_points.clear();
    bottom_points.clear();
    bool inited = false;
  }
};

struct Parameters {
  const double slot_width = 2.5;
  std::vector<int> virtual_slot_ids = {-1, 0, 1, 2, 3};
};

class EnvironmentGenerator {
private:
  bool inited_ = false;
  double left_boundary_ = -3.75;
  double right_boundary_ = 8.75;
  double top_boundary_ = 14;
  double bottom_boundary_ = 0;
  double base_y_ = 6; // y of init pose
  double max_slot_y_ = 0.0;
  planning_math::Vec2d p_left_{};
  bool is_on_left_ = false;
  Pose2D local_frame_pose_;
  Parameters param_;
  // center of virtual slot
  std::vector<double> center_x_of_virtual_top_slots_;
  std::vector<double> center_x_of_virtual_bottom_slots_;
  // threshold of virtual slot
  std::vector<double> threshold_x_of_virtual_top_slots_;
  std::vector<double> threshold_x_of_virtual_bottom_slots_;
  // 落在虚拟车位所在空间的障碍点
  std::map<int, std::vector<planning_math::Vec2d>>
      points_in_virtural_top_slots_;
  std::map<int, std::vector<planning_math::Vec2d>>
      points_in_virtural_bottom_slots_;
  // 车位环境
  Environment env_;

public:
  EnvironmentGenerator(/* args */);
  ~EnvironmentGenerator();
  bool process();
  bool init(planning_math::Vec2d &p_left, planning_math::Vec2d &p_right,
            std::vector<planning_math::Vec2d> &points_of_obstacles,
            Pose2D &init_pose, Pose2D &local_frame_pose, bool &is_on_left);
  void
  set_obstacle_points(std::vector<planning_math::Vec2d> &points_of_obstacles);
  void get_local_env_points(std::vector<planning_math::Vec2d> &points);
  void get_global_env_points(std::vector<planning_math::Vec2d> &points);
  Environment get_environment();
  bool deinit();

private:
  bool get_virtual_slot_index(int &index, double x, std::vector<double> vec);
  bool get_min_x(double &x, std::vector<planning_math::Vec2d> &points);
  bool get_max_x(double &x, std::vector<planning_math::Vec2d> &points);
  bool get_min_y(double &y, std::vector<planning_math::Vec2d> &points);
  bool get_max_y(double &y, std::vector<planning_math::Vec2d> &points);
  void get_left_right(double &left, double &right, double &center_x,
                      std::vector<planning_math::Vec2d> &points,
                      double top = 1e8, double bottom = -1e8);
  void get_min_y1_y2(double &y1, double &y2, double &center_x,
                     std::vector<planning_math::Vec2d> &points);
  void get_max_y1_y2(double &y1, double &y2, double &center_x,
                     std::vector<planning_math::Vec2d> &points);
  void set_top_environment();
  void set_bottom_environment();
  bool set_channel_of_env();
};

} // namespace msquare