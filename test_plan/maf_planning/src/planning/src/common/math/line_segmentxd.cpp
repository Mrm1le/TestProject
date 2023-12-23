#include "common/math/line_segmentxd.h"

namespace msquare {

namespace planning_math {

LineSegmentXd::LineSegmentXd(const Vec2d &start, const Vec2d &end,
                             const double max_length) {
  start_ = start;
  end_ = end;
  subSegment_max_length_ = max_length;
  length_ = start_.DistanceTo(end_);
  Init(start, end);
};

LineSegmentXd::LineSegmentXd(const Point3D &start, const Point3D &end,
                             const double max_length) {
  start_ = Vec2d(start.x, start.y);
  end_ = Vec2d(end.x, end.y);
  subSegment_max_length_ = max_length;
  length_ = start_.DistanceTo(end_);
  Init(Vec2d(start.x, start.y), Vec2d(end.x, end.y));
};

LineSegmentXd::LineSegmentXd(){

};

void LineSegmentXd::Init(
    const std::shared_ptr<FrenetCoordinateSystem> &frenet_coord,
    const double ego_frenet_s, const Pose2D &ego_pose) {
  // const auto& frenet_coord = world_model->get_frenet_coord();
  // const auto& ego_frenet_s = world_model->get_ego_state().ego_frenet.x;
  // const auto& ego_pose = world_model->get_ego_state().ego_pose;
  subSegment2d_.clear();

  Point2D start_point, end_point;
  (void)frenet_coord->FrenetCoord2CartCoord(Point2D(ego_frenet_s, 0.0),
                                            start_point);

  for (double s = ego_frenet_s + 1.0; s < ego_frenet_s + 10.0; s += 1.0) {
    (void)frenet_coord->FrenetCoord2CartCoord(Point2D(s, 0.0), end_point);
    double tmp_s = clip(s, frenet_coord->GetSlength(), 0.0);
    if (frenet_coord->GetRefCurveCurvature(tmp_s) > 0.1) {
      subSegment2d_.emplace_back(Vec2d(start_point.x, start_point.y),
                                 Vec2d(end_point.x, end_point.y));
      start_point = end_point;
    }
  }
  subSegment2d_.emplace_back(Vec2d(start_point.x, start_point.y),
                             Vec2d(end_point.x, end_point.y));
}

void LineSegmentXd::Init(Vec2d start, Vec2d end) {
  // mph_assert(start.DistanceTo(end) > 0.0);
  while (start.DistanceTo(end) > subSegment_max_length_) {
    Vec2d vec_diff = end - start;
    Vec2d temp = start + vec_diff / vec_diff.Length() * subSegment_max_length_;
    subSegment2d_.emplace_back(start, temp);
    start = temp;
  }
  subSegment2d_.emplace_back(start, end);
}

const std::vector<LineSegment2d> &LineSegmentXd::GetSubSegment2d() const {
  return subSegment2d_;
}

int LineSegmentXd::Size() const { return subSegment2d_.size(); }

double LineSegmentXd::TotalLength() const { return length_; }

void LineSegmentXd::SetBoard(double left_board, double right_board) {
  left_board_ = left_board;
  right_board_ = right_board;
}

double LineSegmentXd::DistanceTo(double x, double y) const {
  double abs_min = 10000.0;
  double result = 10000.0;
  for (const auto &line : subSegment2d_) {
    double distance = line.ProductOntoUnit(Vec2d(x, y));
    if (std::abs(distance) < abs_min) {
      abs_min = distance;
      result = distance;
    }
  }
  return result;
}

bool LineSegmentXd::IsInRoad(double x, double y) const {
  double distance = DistanceTo(x, y);
  if (distance > right_board_ && distance < left_board_) {
    return true;
  }

  return false;
}

} // namespace planning_math
} // namespace msquare