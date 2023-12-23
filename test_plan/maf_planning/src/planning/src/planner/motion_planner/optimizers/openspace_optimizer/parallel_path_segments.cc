#include "planner/motion_planner/optimizers/openspace_optimizer/parallel_path_segments.h"

namespace msquare {

bool PathSegments::getPath(std::vector<Pose2D> &path) {
  if (is_accumed && is_valid) {
    path.assign(final_path.begin(), final_path.end());
    return true;
  }

  if (!accumPath()) {
    return false;
  }

  path.assign(final_path.begin(), final_path.end());
  return true;
}

bool PathSegments::accumPath(double accum_error) {
  final_path.clear();
  bool is_first = false;
  Pose2D last_pose;
  for (auto &s : segments) {
    if (s.empty()) {
      continue;
    }
    if (!is_first) {
      is_first = true;
      last_pose = s.front();
      final_path.push_back(last_pose);
    }
    bool is_seg_first = true;
    for (auto &p : s) {
      if (is_seg_first) {
        is_seg_first = false;
        double real_error = std::hypot(p.x - last_pose.x, p.y - last_pose.y);

        if (real_error > accum_error) {
          LOCAL_LOG(LOCAL_DEBUG, "accum error:%f which is larger than %f",
                    real_error, accum_error);
          is_valid = false;
          is_accumed = true;
          return false;
        }
        continue;
      }

      final_path.push_back(p);
    } // end for
    last_pose = s.back();
  } // end for

  is_valid = true;
  is_accumed = true;
  return true;
}

std::vector<Pose2D> InSlotPathSegemnts::interpolatePath(double step,
                                                        double theta_step) {
  std::vector<Pose2D> path;
  for (int i = 0; i + 1 < key_pose.size(); ++i) {
    if (fabs(key_pose[i].theta - key_pose[i + 1].theta) < 1e-5) {
      // straight
      path.push_back(key_pose[i]);
      double seg_length = std::hypot(key_pose[i + 1].x - key_pose[i].x,
                                     key_pose[i + 1].y - key_pose[i].y);
      // std::cout << "seg_length: " << seg_length << std::endl;
      int seg_num = floor(seg_length / step);
      double dx = (key_pose[i + 1].x - key_pose[i].x) / seg_length;
      double dy = (key_pose[i + 1].y - key_pose[i].y) / seg_length;

      for (int j = 1; j <= seg_num; ++j) {
        path.emplace_back(key_pose[i].x + j * dx * step,
                          key_pose[i].y + j * dy * step, key_pose[i].theta);
      }
    } else {
      // curve
      path.push_back(key_pose[i]);

      // judge backwoards or forwards
      planning_math::Vec2d vec_former_latter(key_pose[i + 1].x - key_pose[i].x,
                                             key_pose[i + 1].y - key_pose[i].y);
      double vec_heading = vec_former_latter.Angle();
      double turning_radius =
          sqrt(std::pow(vec_former_latter.Length(), 2) / 2 /
               (1 - cos(key_pose[i].theta - key_pose[i + 1].theta)));
      int towards = fabs(vec_heading - key_pose[i].theta) < M_PI_2 ? 1 : -1;
      int is_counter = key_pose[i + 1].theta > key_pose[i].theta ? 1 : -1;
      int steer = towards * is_counter;

      double center_x =
          key_pose[i].x - steer * turning_radius * sin(key_pose[i].theta);
      double center_y =
          key_pose[i].y + steer * turning_radius * cos(key_pose[i].theta);

      int seg_num =
          floor(fabs(key_pose[i + 1].theta - key_pose[i].theta) / theta_step);
      double real_theta_step =
          fabs(key_pose[i + 1].theta - key_pose[i].theta) / (seg_num + 1);
      int theta_steps = real_theta_step < 1e-3 ? 0 : seg_num + 1;
      for (int j = 1; j <= theta_steps; ++j) {
        double inter_theta =
            key_pose[i].theta + is_counter * j * real_theta_step;
        double inter_x = center_x + steer * turning_radius * sin(inter_theta);
        double inter_y = center_y - steer * turning_radius * cos(inter_theta);

        path.emplace_back(inter_x, inter_y, inter_theta);
      }
    }
  }
  path.push_back(key_pose.back());
  return path;
}
} // namespace msquare