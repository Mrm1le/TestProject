#include "common/speed/st_graph_data.h"

namespace msquare {

void StGraphData::LoadData(const std::vector<const STBoundary *> &st_boundaries,
                           const double min_s_on_st_boundaries,
                           const TrajectoryPoint &init_point,
                           const SpeedLimit &speed_limit,
                           const double path_data_length,
                           const double total_time_by_conf) {
  init_ = true;
  st_boundaries_ = st_boundaries;
  min_s_on_st_boundaries_ = min_s_on_st_boundaries;
  init_point_ = init_point;
  speed_limit_ = speed_limit;
  path_data_length_ = path_data_length;
  total_time_by_conf_ = total_time_by_conf;
}

void StGraphData::resetBoundary(
    const std::vector<const STBoundary *> &st_boundaries) {
  st_boundaries_ = st_boundaries;
}

const std::vector<const STBoundary *> &StGraphData::st_boundaries() const {
  return st_boundaries_;
}

double StGraphData::min_s_on_st_boundaries() const {
  return min_s_on_st_boundaries_;
}

const TrajectoryPoint &StGraphData::init_point() const { return init_point_; }

const SpeedLimit &StGraphData::speed_limit() const { return speed_limit_; }

double StGraphData::path_length() const { return path_data_length_; }

double StGraphData::total_time_by_conf() const { return total_time_by_conf_; }

} // namespace msquare