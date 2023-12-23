#ifndef MODULES_PLANNING_COMMON_PATH_DATA_H_
#define MODULES_PLANNING_COMMON_PATH_DATA_H_

#include "common/path/discretized_path.h"

namespace msquare {

class PolygonalLine : public std::vector<SLPoint> {
public:
  PolygonalLine() = default;

  virtual ~PolygonalLine() = default;

  explicit PolygonalLine(std::vector<SLPoint> sl_points);

  bool EvaluateByS(const double s, double *border) const;

  double TotalLength() const;
};

class PathData {
public:
  PathData() = default;

  bool SetDiscretizedPath(DiscretizedPath path);

  bool SetPathBorder(PolygonalLine left_border, PolygonalLine right_border);

  const DiscretizedPath &discretized_path() const;

  const PolygonalLine &LeftBorder() const;

  const PolygonalLine &RightBorder() const;

  PathPoint GetPathPointWithPathS(const double s) const;

  std::pair<double, double> getWidth(const double path_s) const;

  void Clear();

  bool Empty() const;

  void set_path_label(const std::string &label);

  const std::string &path_label() const;

  void set_blocking_obstacle_id(int obs_id) { blocking_obstacle_id_ = obs_id; }

  int blocking_obstacle_id() const { return blocking_obstacle_id_; }

private:
  DiscretizedPath discretized_path_;
  PolygonalLine left_border_, right_border_;
  int blocking_obstacle_id_;
  std::string path_label_ = "";
};

} // namespace msquare

#endif /* MODULES_PLANNING_COMMON_PATH_DATA_H_ */
