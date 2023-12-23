#include "planner/behavior_planner/deciders/openspace_utils.h"
#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "common/math/vec2d.h"

namespace msquare {
namespace parking {
bool get_gridmap_boundary_normal(const std::shared_ptr<WorldModel> world_model,
                                 const double &default_swell_distance,
                                 const double &default_step_size,
                                 const TrajectoryPoint &target_state,
                                 const TrajectoryPoint &ego_state,
                                 planning_math::Box2d &grid_map_bounding_box,
                                 GridMapBoxLocal &box_local,
                                 Pose2D &tmp_location_zero) {

  planning_math::Box2d end_state_box(
      planning_math::Vec2d(target_state.path_point.x,
                           target_state.path_point.y),
      target_state.path_point.theta, VehicleParam::Instance()->length,
      VehicleParam::Instance()->width); // local
  end_state_box.LongitudinalExtend(default_swell_distance);
  end_state_box.LateralExtend(default_swell_distance);

  planning_math::Box2d ego_state_box(
      planning_math::Vec2d(ego_state.path_point.x, ego_state.path_point.y),
      ego_state.path_point.theta, VehicleParam::Instance()->length,
      VehicleParam::Instance()->width); // local
  // Pose2D tmp_location_zero;
  tmp_location_zero.x = ego_state_box.center_x();
  tmp_location_zero.y = ego_state_box.center_y();
  tmp_location_zero.theta = ego_state_box.heading();

  ego_state_box.LongitudinalExtend(default_swell_distance);
  ego_state_box.LateralExtend(default_swell_distance);

  // std::cout << "end_state_openspace_box: " << end_state_box.center_x() << " "
  //           << end_state_box.center_y() << "  "
  //           << end_state_box.heading() << "  "
  //           << ego_state_box.center_x() << " "
  //           << ego_state_box.center_y() << " "
  //           << ego_state_box.heading() << std::endl;

  // rotation center: start_state_box third corners

  // std::cout << "grid map localframe_tmp: " << tmp_location_zero.x << "   "
  //           << tmp_location_zero.y << "  "
  //           << tmp_location_zero.theta << std::endl;

  // start bounding box
  auto start_bounding_tmp =
      planning_math::tf2d(tmp_location_zero, ego_state_box);
  auto end_bounding_box_tmp =
      planning_math::tf2d(tmp_location_zero, end_state_box);

  // std::cout << "end_state_openspace_box_local_temp: " <<
  // end_bounding_box_tmp.center_x() << "   "
  //         << end_bounding_box_tmp.center_y() << "  "
  //         << end_bounding_box_tmp.heading() << "  "
  //         << start_bounding_tmp.center_x() << " "
  //         << start_bounding_tmp.center_y() << " "
  //         << start_bounding_tmp.heading() << std::endl;

  // first vec2d
  //    2 ---- 1
  //    |      |
  //    |      |
  //    3 ---- 4

  // for end bounding
  std::vector<planning_math::Vec2d> end_bounding_corners =
      end_bounding_box_tmp.GetAllCorners();
  for (auto &pp : start_bounding_tmp.GetAllCorners())
    box_local.extend(pp);
  for (auto &pp : end_bounding_box_tmp.GetAllCorners())
    box_local.extend(pp);

  // std::cout << "sbox: " << box_local.s_left << "   " << box_local.s_right
  //           << "  " << box_local.s_up << " " << box_local.s_down <<
  //           std::endl;

  // from refline
  auto frenet_coor = world_model->get_frenet_coord();
  if (!frenet_coor) {
    // throw std::logic_error("get_gridmap_boundary_normal: null frenet");
    return false;
  }
  double end_s = target_state.path_point.s;
  std::vector<double> s_consider;
  std::vector<double> left_road_border_distance;
  std::vector<double> right_road_border_distance;
  s_consider.push_back(world_model->get_ego_state().ego_frenet.x);
  int s_count =
      (end_s - world_model->get_ego_state().ego_frenet.x) / default_step_size;
  for (int i = 0; i < s_count; i++)
    s_consider.push_back(s_consider.back() + default_step_size);
  s_consider.push_back(end_s);
  auto refline_manager = world_model->get_refline_manager();
  get_refline_info(s_consider, left_road_border_distance,
                   right_road_border_distance,
                   refline_manager->get_left_road_border_distance(),
                   refline_manager->get_right_road_border_distance(),
                   refline_manager->get_s());
  for (std::size_t i = 0; i < s_consider.size(); i++) {
    Point2D frenet_end_pos(s_consider[i], 0.0);
    Point2D cart_end_pose;
    if (frenet_coor->FrenetCoord2CartCoord(frenet_end_pos, cart_end_pose) ==
        TRANSFORM_STATUS::TRANSFORM_FAILED)
      return false;
    double heading = planning_math::NormalizeAngle(
        frenet_coor->GetRefCurveHeading(s_consider[i]));
    Pose2D cur_pose_local;
    cur_pose_local.x = cart_end_pose.x;
    cur_pose_local.y = cart_end_pose.y;
    cur_pose_local.theta = heading;
    // local:
    auto point = planning_math::tf2d(tmp_location_zero, cur_pose_local);
    // left boundary
    if (i >= left_road_border_distance.size() ||
        i >= right_road_border_distance.size()) {
      break;
    }
    planning_math::Vec2d left_point(
        point.x + left_road_border_distance[i] * cos(point.theta + M_PI / 2),
        point.y + left_road_border_distance[i] * sin(point.theta + M_PI / 2));
    box_local.extend(left_point);
    planning_math::Vec2d right_point(
        point.x + right_road_border_distance[i] * cos(point.theta - M_PI / 2),
        point.y + right_road_border_distance[i] * sin(point.theta - M_PI / 2));
    box_local.extend(right_point);
  }
  // get grid map bounding box enu
  planning_math::Vec2d corner1 = planning_math::tf2d_inv(
      tmp_location_zero,
      planning_math::Vec2d(box_local.s_right, box_local.s_up));
  planning_math::Vec2d corner2 = planning_math::tf2d_inv(
      tmp_location_zero,
      planning_math::Vec2d(box_local.s_left, box_local.s_up));
  ;
  planning_math::Vec2d corner3 = planning_math::tf2d_inv(
      tmp_location_zero,
      planning_math::Vec2d(box_local.s_right, box_local.s_down));
  ;
  planning_math::Vec2d corner4 = planning_math::tf2d_inv(
      tmp_location_zero,
      planning_math::Vec2d(box_local.s_left, box_local.s_down));
  ;

  double center_x =
      (corner1.x() + corner2.x() + corner3.x() + corner4.x()) / 4.0;
  double center_y =
      (corner1.y() + corner2.y() + corner3.y() + corner4.y()) / 4.0;

  planning_math::Vec2d corner_center(center_x, center_y);

  planning_math::Box2d grid_map_bounding_box_(
      corner_center, tmp_location_zero.theta,
      box_local.s_right - box_local.s_left, box_local.s_up - box_local.s_down);

  grid_map_bounding_box = grid_map_bounding_box_;

  tmp_location_zero.x = corner3.x();
  tmp_location_zero.y = corner3.y();
  tmp_location_zero.theta = tmp_location_zero.theta;

  return true;
}

void get_refline_info(const std::vector<double> s_consider,
                      std::vector<double> &left_road_border_distance,
                      std::vector<double> &right_road_border_distance,
                      std::vector<double> left_road_border_distance_vector,
                      std::vector<double> right_road_border_distance_vector,
                      std::vector<double> s_vector) {
  if (s_consider.size() == 0) {
    return;
  }
  planning_math::interpsVector(left_road_border_distance_vector, s_vector,
                               s_consider, left_road_border_distance);

  planning_math::interpsVector(right_road_border_distance_vector, s_vector,
                               s_consider, right_road_border_distance);
}

bool extend_boundary_with_refline(const std::shared_ptr<WorldModel> world_model,
                                  const double &default_swell_distance,
                                  const double &default_step_size,
                                  const TrajectoryPoint &target_state,
                                  const TrajectoryPoint &ego_state,
                                  planning_math::AABox2d &box_local,
                                  const Pose2D &tmp_location_zero,
                                  double start_s) {
  // from refline
  auto frenet_coor = world_model->get_frenet_coord();
  if (!frenet_coor) {
    // throw std::logic_error("extend_boundary_with_refline: null frenet");
    return false;
  }

  double end_s = target_state.path_point.s;

  Point2D target_pose_frenet;
  if (frenet_coor->CartCoord2FrenetCoord(
          Point2D{target_state.path_point.x, target_state.path_point.y},
          target_pose_frenet, target_state.path_point.theta) ==
      TRANSFORM_STATUS::TRANSFORM_FAILED) {
    end_s = frenet_coor->GetSlength();
  } else {
    end_s = target_pose_frenet.x;
  }

  std::vector<double> s_consider;
  std::vector<double> left_road_border_distance;
  std::vector<double> right_road_border_distance;
  if (std::fabs(start_s) < 1e-5) {
    start_s = world_model->get_ego_state().ego_frenet.x;
  }
  s_consider.push_back(start_s);
  int s_count = std::floor((end_s - start_s) / default_step_size);
  for (int i = 0; i < s_count; i++)
    s_consider.push_back(s_consider.back() + default_step_size);
  s_consider.push_back(end_s);
  auto refline_manager = world_model->get_refline_manager();
  get_refline_info(s_consider, left_road_border_distance,
                   right_road_border_distance,
                   refline_manager->get_left_road_border_distance(),
                   refline_manager->get_right_road_border_distance(),
                   refline_manager->get_s());
  for (std::size_t i = 0; i < s_consider.size(); i++) {
    Point2D frenet_end_pos(s_consider[i], 0.0);
    Point2D cart_end_pose;
    if (frenet_coor->FrenetCoord2CartCoord(frenet_end_pos, cart_end_pose) ==
        TRANSFORM_STATUS::TRANSFORM_FAILED)
      return false;
    // std::cout << "extend_boundary_with_refline: " << cart_end_pose.x << ", "
    // << cart_end_pose.y << ", " << 0 << std::endl;
    double heading = planning_math::NormalizeAngle(
        frenet_coor->GetRefCurveHeading(s_consider[i]));
    Pose2D cur_pose_local;
    cur_pose_local.x = cart_end_pose.x;
    cur_pose_local.y = cart_end_pose.y;
    cur_pose_local.theta = heading;
    // local:
    auto point = planning_math::tf2d(tmp_location_zero, cur_pose_local);
    // std::cout << "extend_boundary_with_refline-s_consider[i]" <<
    // s_consider[i] << std::endl; std::cout <<
    // "extend_boundary_with_refline-left_road_border_distance[i]" <<
    // left_road_border_distance[i] << std::endl; std::cout <<
    // "extend_boundary_with_refline-right_road_border_distance[i]" <<
    // right_road_border_distance[i] << std::endl;
    // // left boundary
    if (i >= left_road_border_distance.size() ||
        i >= right_road_border_distance.size()) {
      break;
    }
    planning_math::Vec2d left_point(
        point.x + left_road_border_distance[i] * cos(point.theta + M_PI / 2),
        point.y + left_road_border_distance[i] * sin(point.theta + M_PI / 2));
    box_local.MergeFrom(left_point);
    planning_math::Vec2d right_point(
        point.x + right_road_border_distance[i] * cos(point.theta - M_PI / 2),
        point.y + right_road_border_distance[i] * sin(point.theta - M_PI / 2));
    box_local.MergeFrom(right_point);
  }

  return true;
}
} // namespace parking
} // namespace msquare