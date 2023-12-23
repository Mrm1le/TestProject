#include "common/config/vehicle_param.h"
#include "common/math/polygon2d.h"
#include "common/parking_planner_types.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "search_based_planner.h"

namespace msquare {

// functions of Mirror injection with Y-axis
planning_math::Vec2d MirrorInjection(const planning_math::Vec2d &point);
planning_math::LineSegment2d
MirrorInjection(const planning_math::LineSegment2d &line);
planning_math::Box2d MirrorInjection(const planning_math::Box2d &box);
Pose2D MirrorInjection(const Pose2D &pose);

void addInflationPoints(std::vector<planning_math::Vec2d> &points,
                        const planning_math::Vec2d &p,
                        double inflation_for_points);

void getEPCorners(const Pose2D &pose,
                  std::vector<planning_math::Vec2d> &corners,
                  std::vector<planning_math::LineSegment2d> &lines,
                  double lon_change = 0, double lat_change = 0);

std::vector<double> calcMaxTheta(const planning_math::Vec2d &arc_center,
                                 const planning_math::LineSegment2d &line,
                                 const planning_math::Vec2d &point);
Pose2D calcNextPose(const std::vector<planning_math::LineSegment2d> obstacles,
                    const std::vector<planning_math::Vec2d> points_of_obstacles,
                    const Pose2D init_pose, const Pose2D &start_pose,
                    int steer_direction, int travel_direction, double radius);

bool checkStraightLine(
    const std::vector<planning_math::LineSegment2d> obstacles,
    const std::vector<planning_math::Vec2d> points_of_obstacles,
    const Pose2D &start_pose, const Pose2D &end_pose);

bool isRSPathSafe(const std::vector<planning_math::LineSegment2d> obstacles,
                  const std::vector<planning_math::Vec2d> points_of_obstacles,
                  const Pose2D init_pose,
                  const std::vector<Pose2D> &path_key_points);

std::vector<Pose2D> InterpolatePath(double step_size,
                                    const std::vector<Pose2D> key_points);
msquare::SbpResult convertToSbpResult(const std::vector<Pose2D> &traj);
} // namespace msquare