#pragma once

#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "common/math/vec2d.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"

namespace msquare {

/**
 * @brief shrink inflation of car_params to avoid collision with obstacle
 * @param car_param the param used for inflation of ego box, only valid in
 * planning
 * @param ego_box ground truth geometry of ego box
 * @param obs_line obstacle that invade the inflated ego box
 */
void modify_inflation_for_obstacle(CarParams *car_params,
                                   planning_math::Box2d ego_box,
                                   planning_math::LineSegment2d obs_line,
                                   double lat_margin = 0.1);
void modify_inflation_for_obstacle(CarParams *car_params,
                                   planning_math::Box2d ego_box,
                                   planning_math::Box2d obs_box,
                                   double lat_margin = 0.1);
void modify_inflation_for_obstacle(const CarParams *car_params,
                                   planning_math::Box2d ego_box,
                                   planning_math::Vec2d obs_point,
                                   double lat_margin = 0.1);

} // namespace msquare