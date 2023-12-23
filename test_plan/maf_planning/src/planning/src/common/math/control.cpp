#include "common/math/control.h"

namespace msquare {

namespace parking {

double estimate_delta_f(const std::vector<TrajectoryPoint> &traj,
                        const Pose2D &ego_pose, const double lookahead_dist,
                        const double k, const double Kp) {
  if (traj.size() < 3)
    return 0.0;
  // get ego pose closest point
  auto itr_ego = traj.begin();
  double min_dist_ego = 1e19;
  for (auto itr = traj.begin(); itr < traj.end(); itr++) {
    double dist = std::hypot(itr->path_point.x - ego_pose.x,
                             itr->path_point.y - ego_pose.y);
    if (dist < min_dist_ego) {
      min_dist_ego = dist;
      itr_ego = itr;
    }
  }
  // get lookahead point
  double dist_to_itr_ego = 0.0;
  auto itr_lookahead_point = traj.end() - 1;
  for (auto itr = itr_ego + 1; itr < traj.end(); itr++) {
    dist_to_itr_ego += std::hypot(itr->path_point.x - (itr - 1)->path_point.x,
                                  itr->path_point.y - (itr - 1)->path_point.y);
    if (dist_to_itr_ego > lookahead_dist) {
      itr_lookahead_point = itr;
      break;
    }
  }
  Point2D pursuit_goal(itr_lookahead_point->path_point.x,
                       itr_lookahead_point->path_point.y);
  double pursuit_alpha =
      std::atan2(pursuit_goal.y - ego_pose.y, pursuit_goal.x - ego_pose.x) -
      ego_pose.theta;
  double pursuit_dist =
      std::hypot(pursuit_goal.x - ego_pose.x, pursuit_goal.y - ego_pose.y);
  return std::atan2(2.0 * VehicleParam::Instance()->wheel_base *
                        std::sin(pursuit_alpha),
                    pursuit_dist);
}

} // namespace parking
} // namespace msquare