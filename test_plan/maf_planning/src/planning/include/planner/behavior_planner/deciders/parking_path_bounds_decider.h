#pragma once

#include "common/parking_world_model.h"
#include "common/path/path_boundary.h"
#include "planner/message_type.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <set>
#include <tuple>
#include <unordered_map>
#include <vector>

// constexpr double kStepDuration = 0.2;
// constexpr int PathSteps = 30;
// TODO(all): Update extra tail point base on vehicle speed.
// constexpr int kNumExtraTailBoundPoint = 30;

namespace msquare {
namespace parking {
class ParkingPathBoundsDecider {

#define left_bounds_to_ref_const(left_bound, lref_const)                       \
  (left_bound - lref_const)
#define right_bounds_to_ref_const(right_bound, lref_const)                     \
  (lref_const - right_bound)

public:
  ParkingPathBoundsDecider(const std::shared_ptr<WorldModel> &world_model,
                           int num_knots);

  // virtual ~PathBoundsDecider() = default;

  int process();

  void set_lane_width(const std::vector<double> lane_width,
                      const std::vector<double> lane_left_width,
                      const std::vector<double> lane_right_width) {
    adc_lane_width_ = lane_width;
    adc_left_lane_width_ = lane_left_width;
    adc_right_lane_width_ = lane_right_width;
  }

  void set_road_border(const std::vector<double> left_road_border,
                       const std::vector<double> right_road_border) {
    left_road_border_ = left_road_border;
    right_road_border_ = right_road_border;
  }

  void set_pull_over_status(bool pull_over_status) {
    pull_over_status_ = pull_over_status;
  }

  void set_init_state(std::array<double, 3> init_state, double start_s,
                      double speed, double delta_s);

  void set_end_state(std::array<double, 3> init_state, double end_s);

  void unset_end_state(int using_end_state) {
    using_end_state_ = using_end_state;
  }

  void set_static_object(const std::vector<SLBoundaryEx> &static_obstacles) {
    static_obstacles_ = static_obstacles;
  }

  void set_scene(int scene_avp) { scene_ = scene_avp; }

  void set_latoffset(std::vector<double> latoffset) { latoffset_ = latoffset; }

  // void set_motion_bound(const double max_dddl,
  //     std::vector<std::pair<double, double>> lateral_bound_dd){
  //   max_dddl_ = max_dddl;
  //   lateral_bound_dd_ = lateral_bound_dd;
  // }

  void set_border_flag(std::vector<bool> border_flag) {
    border_flag_ = border_flag; // 1: road 0: lane
  }

  const std::vector<PathBoundary> &GetCandidatePathBoundaries() const;

  // enum class LaneBorrowInfo {
  //   LEFT_BORROW,
  //   NO_BORROW,
  //   RIGHT_BORROW,
  // };

  void set_num_knots(size_t n) { num_knots_ = n; }

  const std::vector<BoundaryPointInfo> &get_boundary_decision_info() const {
    return boundary_decision_info_;
  }

  const std::vector<BoundaryPointInfo> &
  get_boundary_decision_info_expend() const {
    return boundary_decision_info_expend_;
  }

private:
  /** @brief Initializes an empty path boundary.
   */
  bool InitPathBoundary(PathBound *const path_bound, RefBound *const ref_bound);

  /** @brief The regular path boundary generation considers the ADC itself
   *   and other static environments:
   *   - ADC's position (lane-changing considerations)
   *   - lane info
   *   - static obstacles
   *   The philosophy is: static environment must be and can only be taken
   *   care of by the path planning.
   * @param reference_line_info
   * @param The generated regular path_boundary, if there is one.
   * @param The blocking obstacle's id. If none, then it's not modified.
   * @return A failure message. If succeeded, return "" (empty string).
   */
  std::string
  GenerateRegularPathBound(PathBound *const path_bound,
                           std::string *const blocking_obstacle_id,
                           std::vector<double> *const ref_line,
                           std::vector<double> *const interp_ref_line);

  /** @brief The fallback path only considers:
   *   - ADC's position (so that boundary must contain ADC's position)
   *   - lane info
   *   It is supposed to be the last resort in case regular path generation
   *   fails so that speed decider can at least have some path and won't
   *   fail drastically.
   *   Therefore, it be reliable so that optimizer will not likely to
   *   fail with this boundary, and therefore doesn't consider any static
   *   obstacle. When the fallback path is used, stopping before static
   *   obstacles should be taken care of by the speed decider. Also, it
   *   doesn't consider any lane-borrowing.
   * @param reference_line_info
   * @param The generated fallback path_boundary, if there is one.
   * @return A failure message. If succeeded, return "" (empty string).
   */
  std::string
  GenerateFallbackPathBound(PathBound *const path_bound,
                            std::vector<double> *const ref_line,
                            std::vector<double> *const interp_ref_line);

  /** @brief Refine the boundary based on the road-info.
   *  The returned boundary is with respect to the lane-center (NOT the
   *  reference_line), though for most of the times reference_line's
   *  deviation from lane-center is negligible.
   */
  // bool GetBoundaryFromRoads(
  //     const ReferenceLineInfo& reference_line_info,
  //     std::vector<std::tuple<double, double, double>>* const path_bound);

  /** @brief Refine the boundary based on lane-info and ADC's location.
   *   It will comply to the lane-boundary. However, if the ADC itself
   *   is out of the given lane(s), it will adjust the boundary
   *   accordingly to include ADC's current position.
   */
  bool GetBoundaryFromLanesAndADC(double ADC_buffer,
                                  PathBound *const path_bound,
                                  RefBound *const ref_bound);

  double cal_motion_buffer(double dddl, std::pair<double, double>, double &l,
                           double &dl, double &ddl);

  /** @brief Refine the boundary based on static obstacles. It will make sure
   *   the boundary doesn't contain any static obstacle so that the path
   *   generated by optimizer won't collide with any static obstacle.
   */
  bool GetBoundaryFromStaticObstacles(PathBound *const path_boundaries,
                                      std::string *const blocking_obstacle_id);

  std::vector<ObstacleEdge>
  SortObstaclesForSweepLine(std::vector<ObstacleEdge> &sorted_obstacles_expend);

  /** @brief Get the distance between ADC's center and its edge.
   * @return The distance.
   */
  double GetBufferBetweenADCCenterAndEdge();

  bool check_motion_collide(PathBound *const path_boundaries);

  /** @brief Update the path_boundary at "idx", as well as the new center-line.
   *        It also checks if ADC is blocked (lmax < lmin).
   * @param The current index of the path_bounds
   * @param The minimum left boundary (l_max)
   * @param The maximum right boundary (l_min)
   * @param The path_boundaries (its content at idx will be updated)
   * @param The center_line (to be updated)
   * @return If path is good, true; if path is blocked, false.
   */
  bool UpdatePathBoundaryAndCenterLine(size_t idx, double left_bound,
                                       double right_bound,
                                       PathBound *const path_boundaries);

  /** @brief Trim the path bounds starting at the idx where path is blocked.
   */
  // void TrimPathBounds(const int path_blocked_idx,
  //     PathBound* const path_boundaries);

  void InitBoundaryDecisionInfo();
  // void feetBoundaryDecisionInfo(int idx, double s, std::string id, int
  // direction, ObjectType type,
  //   bool is_static, double l_min, double l_max, bool is_end);

  void
  EstimateLref(const std::vector<BoundaryPointInfo> &boundary_decision_info,
               PathBound *const lat_boundaries,
               std::vector<double> *const ref_line,
               std::vector<double> *const interp_ref_line);
  void EstimateLrefFallback(PathBound *const lat_boundaries,
                            std::vector<double> *const ref_line,
                            std::vector<double> *const interp_ref_line);

  void
  EstimateLref_dp(const std::vector<BoundaryPointInfo> &boundary_decision_info,
                  PathBound *const lat_boundaries,
                  std::vector<double> *const ref_line,
                  std::vector<double> *const interp_ref_line);

  bool
  checkInterpVal(std::vector<std::pair<int, std::pair<int, int>>> &node_list,
                 std::vector<double> &ref_line, const size_t start_idx,
                 size_t end_idx, PathBound *const lat_boundaries);
  static bool my_compare(std::tuple<double, double, double, int> a,
                         std::tuple<double, double, double, int> b);
  static bool my_compare_with_end(std::tuple<double, double, double, int> a,
                                  std::tuple<double, double, double, int> b);

  std::shared_ptr<WorldModel> world_model_;

  static double end_l_offset_;

  int scene_;

  size_t num_knots_;
  double delta_s_;
  double max_dddl_;
  double adc_frenet_s_;
  double adc_frenet_sd_;
  double adc_frenet_l_;
  double adc_frenet_ld_;
  double adc_frenet_ldd_;
  double adc_l_to_lane_center_ = 0.0;

  double end_s_;
  double end_l_;
  int using_end_state_;

  std::vector<double> adc_lane_width_;
  std::vector<double> adc_left_lane_width_;
  std::vector<double> adc_right_lane_width_;
  std::vector<double> left_road_border_;
  std::vector<double> right_road_border_;
  std::vector<std::pair<double, double>> lateral_bound_dd_;

  std::vector<bool> border_flag_;

  bool pull_over_status_;

  std::vector<SLBoundaryEx> static_obstacles_;
  // std::vector<DynamicObs> dynamic_obstacles_;
  // std::vector<double> lon_s_list_;
  std::vector<PathBoundary> candidate_path_boundaries_;
  std::vector<BoundaryPointInfo> boundary_decision_info_;
  std::vector<BoundaryPointInfo> boundary_decision_info_expend_;

  double obstacle_lon_start_buffer_;
  double obstacle_lon_end_buffer_;
  double obstacle_lon_start_ped_buffer_;
  double obstacle_lon_end_ped_buffer_;
  double obstacle_lat_buffer_;
  double obstacle_lon_start_offset_;
  double obstacle_lon_end_offset_;
  double obstacle_lon_start_ped_offset_;
  double obstacle_lon_end_ped_offset_;
  double static_obstacle_lat_offset_;
  double dynamic_bstacle_lat_offset_;
  double boundary_to_center_width_min_;

  double dist_to_ped_min = 0.4;
  double dist_to_ped_max = 0.9;

  double dist_to_couple_min = 0.5;
  double dist_to_couple_max = 0.7;

  double dist_to_fpt_min = 0.5;
  double dist_to_fpt_max = 0.7;

  double latoffset_entrance = 0.0;
  double latoffset_avp = 0.2;

  double lref_interp_step = 0.1;

  std::vector<double> latoffset_;

  const bool calc_bound_block_ = false;
  const bool using_dp = false;
};

} // namespace parking

} // namespace msquare