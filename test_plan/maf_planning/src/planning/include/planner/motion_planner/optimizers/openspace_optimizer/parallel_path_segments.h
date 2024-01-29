#ifndef PARALLEL_PATH_SEGMENTS_H
#define PARALLEL_PATH_SEGMENTS_H

#include "common/math/polygon2d.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/curve.h"

namespace msquare {

struct ParallelImpl {
  double half_width;

  double last_v = 0.0;
  int block_direc = 0;
  bool is_on_left;
  double safe_radius;

  planning_math::Vec2d left_corner;
  planning_math::Vec2d right_corner;
  double slot_height;
  double upper_height;

  std::vector<double> rs; // radius to try
  unsigned int rs_size;
};

struct ParallelParkoutImpl : ParallelImpl {
  std::vector<planning_math::Vec2d> zero_6_corners;
  double c4_add_theta;
  double c4_to_center;
  double min_block_theta;
  double tline_height_thres;
  bool is_tline_lower;
};

struct PathSegments {
  std::vector<std::vector<Pose2D>> segments;
  std::vector<Pose2D> final_path;
  double score;
  bool is_accumed = false;
  bool is_valid = false;

  double remain_s = 0;
  bool is_forward = false;

  bool accumPath(double accum_error = 1e-3);
  bool getPath(std::vector<Pose2D> &path);
  bool operator<(const PathSegments &osps) const // ascending sort
  {
    return score < osps.score;
  }
};

struct OutSlotAdjustPath : PathSegments {};

struct OutSlotPathSegments : PathSegments {
  clothoid::CircularCurve cir1;
  clothoid::ClothoidCurve clo2;
  clothoid::StraightCurve str3;
  clothoid::ClothoidCurve clo4;
  clothoid::CircularCurve cir5;
  clothoid::ClothoidCurve clo6;
  clothoid::StraightCurve str7;
  // std::vector<std::vector<Pose2D>> segments;

  double radius2;
};

struct InSlotPathSegemnts : PathSegments {
  std::vector<Pose2D> key_pose;
  std::vector<Pose2D> final_path;

  void reset() {
    key_pose.clear();
    final_path.clear();
  }

  std::vector<Pose2D> interpolatePath(double step, double theta_step);
};

inline planning_math::Vec2d localMirrorX(const planning_math::Vec2d &point) {
  return planning_math::Vec2d(point.x(), -point.y());
}

inline planning_math::LineSegment2d
localMirrorX(const planning_math::LineSegment2d &line) {
  return planning_math::LineSegment2d(
      planning_math::Vec2d(line.start().x(), -line.start().y()),
      planning_math::Vec2d(line.end().x(), -line.end().y()));
}

inline Pose2D localMirrorX(const Pose2D &pose) {
  return Pose2D(pose.x, -pose.y, -pose.theta);
}

} // namespace msquare

#endif