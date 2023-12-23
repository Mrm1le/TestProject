#include "common/path/path_data.h"

namespace msquare {

PolygonalLine::PolygonalLine(std::vector<SLPoint> sl_points)
    : std::vector<SLPoint>(std::move(sl_points)) {
  std::sort(begin(), end(),
            [](const SLPoint &p1, const SLPoint &p2) { return p1.s < p2.s; });
}

bool PolygonalLine::EvaluateByS(const double s, double *border) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().s < s + 1.0e-6 && s - 1.0e-6 < back().s)) {
    return false;
  }
  auto comp = [](const SLPoint &sp, const double s) { return sp.s < s; };

  auto it_lower = std::lower_bound(begin(), end(), s, comp);
  if (it_lower == end()) {
    *border = back().l;
  } else if (it_lower == begin()) {
    *border = front().l;
  } else {
    const auto &p0 = *(it_lower - 1);
    const auto &p1 = *it_lower;
    double s0 = p0.s;
    double s1 = p1.s;
    *border = planning_math::lerp(p0.l, s0, p1.l, s1, s);
  }
  return true;
}

double PolygonalLine::TotalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().s - front().s;
}

bool PathData::SetDiscretizedPath(DiscretizedPath path) {
  discretized_path_ = std::move(path);

  return true;
}

bool PathData::SetPathBorder(PolygonalLine left_border,
                             PolygonalLine right_border) {
  left_border_ = std::move(left_border);
  right_border_ = std::move(right_border);

  return true;
}

const DiscretizedPath &PathData::discretized_path() const {
  return discretized_path_;
}

const PolygonalLine &PathData::LeftBorder() const { return left_border_; }

const PolygonalLine &PathData::RightBorder() const { return right_border_; }

std::pair<double, double> PathData::getWidth(const double path_s) const {
  constexpr double kDefaultHalfwidth = 1.6;
  double left_l = kDefaultHalfwidth;
  double right_l = -kDefaultHalfwidth;
  (void)left_border_.EvaluateByS(path_s, &left_l);
  (void)right_border_.EvaluateByS(path_s, &right_l);
  return {left_l, right_l};
}

bool PathData::Empty() const { return discretized_path_.empty(); }

PathPoint PathData::GetPathPointWithPathS(const double s) const {
  return discretized_path_.Evaluate(s);
}

void PathData::Clear() {
  discretized_path_.clear();
  left_border_.clear();
  right_border_.clear();
}

void PathData::set_path_label(const std::string &label) { path_label_ = label; }

const std::string &PathData::path_label() const { return path_label_; }

} // namespace msquare
