#ifndef MSQUARE_DECISION_PLANNING_PLANNER_LATERAL_BEHAVIOR_PATH_PLANNER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_LATERAL_BEHAVIOR_PATH_PLANNER_H_

#include "common/world_model.h"

namespace msquare {

typedef enum {
  NO_REJECTION,
  BIAS_L,
  BIAS_R,
  WIDE_REJECTION_L,
  WIDE_REJECTION_R,
  SHORT_REJECTION,
  NARROW_REJECTION
} RejectReason;

class PathPlanner {
public:
  PathPlanner(const std::shared_ptr<WorldModel> &world_model,
              VirtualLaneManager &virtual_lane_mgr);
  virtual ~PathPlanner() = default;

  void update(int status, bool flag_avd, bool should_premove,
              bool should_suspend,
              const std::array<std::vector<double>, 2> &avd_car_past,
              const std::array<std::vector<double>, 2> &avd_sp_car_past);

  bool premoving() const { return premoving_; }
  double lane_width() const { return lane_width_; }
  double lat_offset() const { return lat_offset_; }
  const std::array<double, 4> &c_poly() const { return c_poly_; }
  const std::array<double, 4> &d_poly() const { return d_poly_; }
  const std::array<double, 4> &l_poly() const { return l_poly_; }
  const std::array<double, 4> &r_poly() const { return r_poly_; }
  const std::array<std::vector<double>, 2> &avd_car_past() const {
    return avd_car_past_;
  }

  void restore_context(const PathPlannerContext &context);
  void save_context(PathPlannerContext &context) const;

private:
  // void update_basic_path(int status);

  void
  update_premove_path(int status, bool should_premove, bool should_suspend,
                      const std::array<std::vector<double>, 2> &avd_car_past);

  void update_avoidance_path(
      int status, bool flag_avd, bool should_premove,
      const std::array<std::vector<double>, 2> &avd_car_past,
      const std::array<std::vector<double>, 2> &avd_sp_car_past);

  bool premoving_ = false;
  bool l_reject_ = false;
  bool r_reject_ = false;
  double lane_width_ = 3.8;
  double lat_offset_ = 0;
  double curr_time_ = 0;
  double two_ncar_ = -1000;
  double one_ncarl_ = -1000;
  double one_ncarr_ = -1000;
  int reject_reason_ = NO_REJECTION;
  double intercept_width_ = 3.8;
  std::array<double, 4> c_poly_;
  std::array<double, 4> d_poly_;
  std::array<double, 4> l_poly_;
  std::array<double, 4> r_poly_;
  std::array<std::vector<double>, 2> avd_car_past_;
  std::shared_ptr<WorldModel> world_model_;
  VirtualLaneManager &virtual_lane_mgr_;
};

} // namespace msquare

#endif
