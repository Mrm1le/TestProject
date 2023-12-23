#ifndef MSQUARE_ZIGZAG_PATH_H_
#define MSQUARE_ZIGZAG_PATH_H_

#include "common/parking_planner_types.h"

namespace msquare {

/**
 * @brief trajectory point with direction
 *
 */
struct DirTrajectoryPoint : public TrajectoryPoint {
  int direction;
  DirTrajectoryPoint() : direction(0) {
    v = 0;
    a = 0;
    relative_time = 0;

    path_point.x = 0;
    path_point.y = 0;
    path_point.z = 0;
    path_point.theta = 0;
    path_point.kappa = 0;
    path_point.s = 0;
    path_point.dkappa = 0;
    path_point.ddkappa = 0;
    path_point.lane_id = "unkown";
    path_point.x_derivative = 0;
    path_point.y_derivative = 0;
  }
  DirTrajectoryPoint(const TrajectoryPoint &tp)
      : TrajectoryPoint(tp), direction(0) {}
};

class ZigzagPath {
public:
  ZigzagPath();
  explicit ZigzagPath(size_t size);
  ZigzagPath(const std::vector<DirTrajectoryPoint> &points);
  ZigzagPath(const std::vector<TrajectoryPoint> &points);

  const std::vector<DirTrajectoryPoint> &get_points() const { return points_; };
  size_t get_num_stages() const { return stages_info_.size(); }
  size_t
  get_stage_idx(std::vector<DirTrajectoryPoint>::const_iterator iter) const;

  std::vector<DirTrajectoryPoint>::const_iterator
  get_stage_lower(std::vector<DirTrajectoryPoint>::const_iterator iter) const;
  std::vector<DirTrajectoryPoint>::const_iterator
  get_stage_upper(std::vector<DirTrajectoryPoint>::const_iterator iter) const;

  std::vector<DirTrajectoryPoint> get_segment_traj(size_t seg_index) const;

  void reset();

  // size_t get_stage_idx(
  //   std::vector<DirTrajectoryPoint>::const_iterator iter
  // );

  DirTrajectoryPoint &operator[](size_t idx) { return points_[idx]; }

private:
  class StageInfo {
  public:
    StageInfo(std::vector<DirTrajectoryPoint>::const_iterator start,
              std::vector<DirTrajectoryPoint>::const_iterator end);

    bool is_contains(std::vector<DirTrajectoryPoint>::const_iterator x) const;
    std::vector<DirTrajectoryPoint>::const_iterator get_lower() const {
      return start_;
    }
    std::vector<DirTrajectoryPoint>::const_iterator get_upper() const {
      return end_;
    }
    std::size_t get_len() const { return len_; }

  private:
    std::vector<DirTrajectoryPoint>::const_iterator start_;
    std::vector<DirTrajectoryPoint>::const_iterator end_;
    std::size_t len_;
  };

  void init();
  void gen_directions();
  void gen_stages();

  std::vector<DirTrajectoryPoint> points_;
  std::vector<StageInfo> stages_info_;
};

} // namespace msquare

#endif