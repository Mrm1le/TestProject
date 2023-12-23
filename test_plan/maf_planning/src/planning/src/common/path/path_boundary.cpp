#include "common/path/path_boundary.h"

namespace msquare {
namespace parking {

PathBoundary::PathBoundary(
    const double start_s, const double delta_s,
    const std::vector<std::pair<double, double>> &path_boundary,
    const std::vector<double> &ref_line,
    const std::vector<double> &interp_ref_line)
    : start_s_(start_s), delta_s_(delta_s), boundary_(path_boundary),
      ref_line_(ref_line), interp_ref_line_(interp_ref_line) {}

double PathBoundary::start_s() const { return start_s_; }

double PathBoundary::delta_s() const { return delta_s_; }

void PathBoundary::set_boundary(
    const std::vector<std::pair<double, double>> &boundary) {
  boundary_ = boundary;
}

const std::vector<std::pair<double, double>> &PathBoundary::boundary() const {
  return boundary_;
}

const std::vector<double> &PathBoundary::reference_line() const {
  return ref_line_;
}

const std::vector<double> &PathBoundary::interp_reference_line() const {
  return interp_ref_line_;
}

void PathBoundary::set_label(const std::string &label) { label_ = label; }

const std::string &PathBoundary::label() const { return label_; }

void PathBoundary::set_blocking_obstacle_id(const std::string &obs_id) {
  blocking_obstacle_id_ = obs_id;
}

const std::string &PathBoundary::blocking_obstacle_id() const {
  return blocking_obstacle_id_;
}

} // namespace parking

} // namespace msquare