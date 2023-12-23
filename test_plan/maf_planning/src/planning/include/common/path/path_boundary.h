#pragma once

#include "common/config/vehicle_param.h"
#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "planner/message_type.h"
#include <chrono>
#include <string>
#include <utility>
#include <vector>

namespace msquare {
namespace parking {

// PathBoundPoint contains: (s, l_min, l_max).
using PathBoundPoint = std::tuple<double, double, double>;
// PathBound contains a vector of PathBoundPoints.
using PathBound = std::vector<PathBoundPoint>;
// RefBoundPoint contains : (s, l_min, l_max).
using RefBoundPoint = std::tuple<double, double, double>;
// RefBound contains a vector of RefBoundPoints.
using RefBound = std::vector<RefBoundPoint>;
// ObstacleEdge contains: (is_start_s, s, l_min, l_max, obstacle_id,
// direction). diretion(-1 : right, 1 : left)
using ObstacleEdge = std::tuple<int, double, double, double, double, double,
                                double, std::string, std::string>;
// boundsInfo contains: (idx, count, s, obstacle_id, direction, type, isstatic,
// )

struct BoundaryPointInfo {
  int idx;
  double s;
  int count;
  std::vector<std::string> obstacle_id_str;
  std::vector<int> obstacle_id;
  std::vector<int> direction;
  std::vector<ObjectType> type;
  std::vector<int> is_static;
  std::vector<int> is_precise;
  std::vector<double> l_min;
  std::vector<double> l_max;
  std::vector<std::string> obstacle_id_str_end;
  std::vector<int> obstacle_id_end;

  BoundaryPointInfo() : idx(-1), s(0), count(0) {
    obstacle_id_str.clear();
    obstacle_id.clear();
    direction.clear();
    type.clear();
    is_static.clear();
    is_precise.clear();
    l_min.clear();
    l_max.clear();
  }
  // TODO: add decision type
};

enum class StatusBounds : int {
  FALLBACK = 0,
  FALLBACKERROR = 1,
  FALLBACKEMPTY = 2,
  PULLOVER = 1,
  REGULAR = 2,
};

enum class LaneBorrowInfo {
  LEFT_BORROW,
  NO_BORROW,
  RIGHT_BORROW,
};

struct SLBoundaryEx {
  double start_s;
  double end_s;
  double start_l;
  double end_l;
  std::string id;
  std::string side_pass;

  SLBoundaryEx(const double &a, const double &b, const double &c, double d,
               std::string e, std::string f)
      : start_s(a), end_s(b), start_l(c), end_l(d), id(e), side_pass(f) {}
  SLBoundaryEx() {}
};

struct DynamicObs {
  std::string id;
  std::vector<SLBoundary> Trajectory;
  std::string side_pass;
  double start_time;
  double time_buffer;
  std::vector<std::string> decider_tags;
};

class PathBoundary {
public:
  PathBoundary(const double start_s, const double delta_s,
               const std::vector<std::pair<double, double>> &path_boundary,
               const std::vector<double> &ref_line,
               const std::vector<double> &interp_ref_line);

  virtual ~PathBoundary() = default;

  double start_s() const;

  double delta_s() const;

  void set_boundary(const std::vector<std::pair<double, double>> &boundary);
  const std::vector<std::pair<double, double>> &boundary() const;

  const std::vector<double> &reference_line() const;

  const std::vector<double> &interp_reference_line() const;

  void set_label(const std::string &label);
  const std::string &label() const;

  void set_blocking_obstacle_id(const std::string &obs_id);
  const std::string &blocking_obstacle_id() const;

private:
  double start_s_ = 0.0;
  double delta_s_ = 0.0;
  std::vector<std::pair<double, double>> boundary_;
  std::vector<double> ref_line_;
  std::vector<double> interp_ref_line_;
  std::string label_ = "regular";
  std::string blocking_obstacle_id_ = "";
};

} // namespace parking

} // namespace msquare