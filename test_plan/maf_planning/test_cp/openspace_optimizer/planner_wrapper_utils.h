#pragma once

#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "planner/message_type.h"
#include "planner_wrapper.h"
#include <vector>

std::vector<msquare::planning_math::Box2d> get_input_boxes(Box2d *boxes,
                                                           int num) {
  std::vector<msquare::planning_math::Box2d> result;
  for (int i = 0; i < num; ++i) {
    result.push_back(msquare::planning_math::Box2d(
        msquare::planning_math::Vec2d(boxes[i].center.x,
                                      boxes[i].center.y), // center
        boxes[i].heading, boxes[i].length, boxes[i].width));
  }
  return result;
}

msquare::TrajectoryPoint get_input_state(TrajectoryPoint start_state) {
  msquare::TrajectoryPoint init_state;

  init_state.path_point.x = start_state.path_point_.x_;
  init_state.path_point.y = start_state.path_point_.y_;
  init_state.path_point.theta = start_state.path_point_.theta_;
  init_state.v = start_state.v_;
  init_state.a = start_state.a_;
  // init_state.da_ = start_state.da_;
  init_state.steer = start_state.steer_;
  init_state.relative_time = start_state.relative_time_;

  return init_state;
}

std::vector<msquare::planning_math::LineSegment2d>
get_input_lines(Line2d *lines, int num) {
  std::vector<msquare::planning_math::LineSegment2d> result;

  for (int i = 0; i < num; ++i) {
    result.push_back(msquare::planning_math::LineSegment2d(
        msquare::planning_math::Vec2d(lines[i].point[0].x, lines[i].point[0].y),
        msquare::planning_math::Vec2d(lines[i].point[1].x,
                                      lines[i].point[1].y)));
  }

  return result;
}