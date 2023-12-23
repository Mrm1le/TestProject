#pragma once

#include "common/sbp_obstacle_line.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/sbp_obstacle_interface.h"

namespace msquare {

namespace hybrid_a_star_2 {

class VirtualObstacleCreator {
private:
  static constexpr float VIRTUAL_WHEEL_STOP_OPPOSITE_SIDE_DIS = 12.0f;
  static constexpr float VIRTUAL_WHEEL_STOP_TARGET_SIDE_DIS = 0.7f;
  static constexpr float VIRTUAL_WHEELSTOP_EXTEND_LENGTH = 10.0f;
  static constexpr float VIRTUAL_WHEELSTOP_TARGET_SIDE_START_DIS = 0.2f;
  static constexpr float VIRTUAL_WHEELSTOP_WHEEL_WIDTH = 0.32f;

public:
  static void addVirtualWheelStops(float target_x, float target_y,
                                   float target_theta, float current_x,
                                   float current_y, float current_theta,
                                   MultiCircleFootprintModel &footprint,
                                   float vehicle_width, float lat_inflation_low,
                                   std::vector<SbpObstaclePtr> &out) {

    float cos_target_theta = std::cos(target_theta);
    float sin_target_theta = std::sin(target_theta);

    footprint.updatePose(current_x, current_y, std::cos(current_theta),
                         std::sin(current_theta));
    float opposite_distance = VIRTUAL_WHEEL_STOP_OPPOSITE_SIDE_DIS;
    float target_side_distance = VIRTUAL_WHEEL_STOP_TARGET_SIDE_DIS;
    for (const auto &circle : footprint.circles()) {
      float dx = circle.center_x - target_x;
      float dy = circle.center_y - target_y;
      float distance = dx * cos_target_theta + dy * sin_target_theta;
      opposite_distance =
          std::max(opposite_distance, distance + circle.radius + 0.5f);
      target_side_distance =
          std::min(target_side_distance, distance - circle.radius - 0.5f);
    }

    std::vector<planning_math::LineSegment2d> lines;

    float extend_x = -VIRTUAL_WHEELSTOP_EXTEND_LENGTH * sin_target_theta;
    float extend_y = VIRTUAL_WHEELSTOP_EXTEND_LENGTH * cos_target_theta;

    float start_dx = -(VIRTUAL_WHEELSTOP_TARGET_SIDE_START_DIS +
                       0.5f * vehicle_width + lat_inflation_low) *
                     sin_target_theta;
    float start_dy = (VIRTUAL_WHEELSTOP_TARGET_SIDE_START_DIS +
                      0.5f * vehicle_width + lat_inflation_low) *
                     cos_target_theta;

    // target_slot side
    float target_side_x = target_x + target_side_distance * cos_target_theta;
    float target_side_y = target_y + target_side_distance * sin_target_theta;
    planning_math::Vec2d line0_start(target_side_x + start_dx,
                                     target_side_y + start_dy);
    planning_math::Vec2d line0_end(target_side_x + extend_x,
                                   target_side_y + extend_y);
    planning_math::LineSegment2d line0(line0_start, line0_end);

    planning_math::Vec2d line1_start(target_side_x - start_dx,
                                     target_side_y - start_dy);
    planning_math::Vec2d line1_end(target_side_x - extend_x,
                                   target_side_y - extend_y);
    planning_math::LineSegment2d line1(line1_start, line1_end);

    float under_length = 0.5f * vehicle_width - lat_inflation_low -
                         VIRTUAL_WHEELSTOP_WHEEL_WIDTH -
                         VIRTUAL_WHEELSTOP_TARGET_SIDE_START_DIS;
    float under_dx = -under_length * sin_target_theta;
    float under_dy = under_length * cos_target_theta;
    planning_math::Vec2d line2_start(target_side_x + under_dx,
                                     target_side_y + under_dy);
    planning_math::Vec2d line2_end(target_side_x - under_dx,
                                   target_side_y - under_dy);
    planning_math::LineSegment2d line2(line2_start, line2_end);

    // opposite_side
    float opposide_side_x = target_x + opposite_distance * cos_target_theta;
    float opposide_side_y = target_y + opposite_distance * sin_target_theta;
    planning_math::Vec2d line3_start(opposide_side_x + extend_x,
                                     opposide_side_y + extend_y);
    planning_math::Vec2d line3_end(opposide_side_x - extend_x,
                                   opposide_side_y - extend_y);

    planning_math::LineSegment2d line3(line3_start, line3_end);

    lines.emplace_back(line0);
    lines.emplace_back(line1);
    lines.emplace_back(line2);
    lines.emplace_back(line3);

    SbpObstaclePtr obstacle = std::make_shared<SbpObstacleLine>(lines);
    obstacle->setHeightType(ObstacleHeightType::LOW);
    out.emplace_back(obstacle);

    return;
  }
};

} // namespace hybrid_a_star_2

} // namespace msquare
