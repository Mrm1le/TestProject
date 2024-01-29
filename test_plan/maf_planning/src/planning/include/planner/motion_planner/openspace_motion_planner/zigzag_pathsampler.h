#ifndef MSQUARE_ZIGZAG_PATHSAMPLER_H_
#define MSQUARE_ZIGZAG_PATHSAMPLER_H_
#include "zigzag_path.h"
#include <vector>

namespace msquare {
namespace parking {

class ZigzagPathSampler {
public:
  ZigzagPathSampler();

  std::vector<DirTrajectoryPoint> &
  sample(ZigzagPath &path,
         std::vector<DirTrajectoryPoint>::const_iterator iter);
  std::vector<DirTrajectoryPoint> &
  sample_sop(ZigzagPath &path,
             std::vector<DirTrajectoryPoint>::const_iterator iter);
  std::vector<DirTrajectoryPoint>
  get_second_traj(ZigzagPath &path,
                  std::vector<DirTrajectoryPoint>::const_iterator iter);
  void reset_stage_idx();
  void set_extend_length(double extend_length) {
    DIST_FIRST_EXTEND_ = extend_length;
  }

private:
  const double DELTA_T_;
  const double TIME_RANGE_;
  const double MIN_VALID_V_;
  double DIST_FIRST_EXTEND_;

  void update_points(ZigzagPath &path,
                     std::vector<DirTrajectoryPoint>::const_iterator iter);
  void extend_points();
  void extend_points(bool is_first_segment);
  void extend_points_curv();
  double get_back_curvature();
  size_t stage_idx_;
  std::vector<DirTrajectoryPoint> stage_points_;
  std::vector<DirTrajectoryPoint> extended_stage_points_;
  int special_slot_type_ = 0;
};

} // namespace parking
} // namespace msquare

#endif