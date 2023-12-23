#ifndef LATERAL_VIRTUAL_LANE_H
#define LATERAL_VIRTUAL_LANE_H

#include "common/refline.h"
#include "common/tracked_object.h"
#include "common/utils/lateral_utils.h"

namespace msquare {
class WorldModel;

typedef enum { NO_CHANGE, LEFT_CHANGE, RIGHT_CHANGE } RequestType;

typedef enum { FRONT_TRACK, SIDE_TRACK, FISHEYE_TRACK } TrackType;

typedef enum {
  NONE_BETTER = 0,
  NO_CAR_LEFT_LC,
  NO_CAR_RIGHT_LC,
  BETTER_LEFT_LC,
  BETTER_RIGHT_LC,
  FASTER_LEFT_NORMAL_LC,
  FASTER_RIGHT_NORMAL_LC
} RqtSource;

typedef enum {
  UNKNOWN_POS = -100,
  LEFT_LEFT_POS = -2,
  LEFT_POS = -1,
  CURR_POS = 0,
  RIGHT_POS = 1,
  RIGHT_RIGHT_POS = 2
} LanePosition;

typedef enum {
  BOTH_AVAILABLE,
  LEFT_AVAILABLE,
  RIGHT_AVAILABLE,
  BOTH_MISSING
} LaneStatusEx;

typedef enum { UNKNOWN_SOURCE, MAP_SOURCE, FUSION_SOURCE } LaneSource;

typedef enum {
  FIX_LANE,
  ORIGIN_LANE,
  TARGET_LANE,
  CURRENT_LANE,
  LEFT_LANE,
  RIGHT_LANE,
  LEFT_LEFT_LANE,
  RIGHT_RIGHT_LANE,
} LaneProperty;

typedef enum {
  NO_REQUEST,
  INT_REQUEST,
  MAP_REQUEST,
  ACT_REQUEST
} RequestSource;

const unsigned long kInvalidTrackId = ULONG_MAX;
const int kInvalidRelativeId = INT_MAX;

double calc_lane_width(const std::vector<double> &l_poly,
                       const std::vector<double> &r_poly);

constexpr double kDefaultWidth = 3.8;
constexpr double kMinWidth = 2.0;
constexpr double kMaxWidth = 4.5;
class Lane {
public:
  explicit Lane(int position = LanePosition::UNKNOWN_POS,
                RawRefLine *raw_refline = nullptr);
  virtual ~Lane() = default;

  void set_raw_refline(RawRefLine *raw_refline) { raw_refline_ = raw_refline; }

  void set_neighbours(const std::array<Lane *, 2> &neighbours) {
    neighbours_ = neighbours;
  }

  void set_exist(bool exist) { exist_ = exist; }

  const bool exist() const { return exist_; }

  void set_type(int lane_type) { type_ = lane_type; }

  void set_source(int source) { source_ = source; }

  void set_status(int status) { status_ = status; }

  void reset() {
    status_ = LaneStatusEx::BOTH_MISSING;
    type_ = maf_worldmodel::LaneType::UNKNOWN;
    world_model_ = nullptr;
    exist_ = false;
  }

  void update_worldmodel(const std::shared_ptr<WorldModel> &wm) {
    world_model_ = wm;
  }

  void update(
      const std::vector<LaneBoundaryPolylineDerived> &lane_boundary_polyline);

  const bool has_raw_refline() const { return raw_refline_ != nullptr; }

  RawRefLine *raw_refline() const { return raw_refline_; }

  bool has_lines(int side = INT_MAX) const;

  int type() const { return type_; }

  double width() const { return width_; }

  double front_width() const { return front_width_; }

  int position() const { return position_; }

  const bool has_polys() const {
    return (status_ != LaneStatusEx::BOTH_MISSING);
  }

  const bool is_legal_4_lc() const {
    return (exist_ && raw_refline_ != nullptr && width_ > kMinWidth &&
            front_width_ > kMinWidth);
  }

  double dis_to_line(double s, double l, int side, RefLine &fix_refline);

  double dis_to_side_line(double x, double y, int side, RefLine &fix_refline);

  bool is_track_on(const TrackedObject &tr, int track_type,
                   RefLine &fix_refline);

  const double dist_to_center_line() const { return dist_to_center_line_; }
  const double curvature() const { return curvature_; }
  const double relative_theta() const { return relative_theta_; }

  const std::array<double, 2> &intercepts() const { return intercepts_; }
  const std::array<Lane *, 2> &neighbours() const { return neighbours_; }
  int status() const { return status_; }
  int source() const { return source_; }

  double min_width() const {
    if (type_ < 0 || type_ >= (int)min_width_.size()) {
      return kDefaultWidth;
    }

    return min_width_[type_];
  }

  double max_width() const {
    if (type_ < 0 || type_ >= (int)max_width_.size()) {
      return kDefaultWidth;
    }

    return max_width_[type_];
  }

protected:
  std::array<double, 12> min_width_;
  std::array<double, 12> max_width_;
  std::array<double, 2> intercepts_;
  double dist_to_center_line_;
  double curvature_;
  double relative_theta_;
  std::array<Lane *, 2> neighbours_;
  RawRefLine *raw_refline_;
  std::shared_ptr<WorldModel> world_model_ = nullptr;

  int type_;
  double width_;
  double front_width_;
  int status_;
  int source_;
  int position_;
  bool exist_;
};

class VirtualLane : public Lane {
public:
  explicit VirtualLane(int property);
  virtual ~VirtualLane();

  Lane *master() const { return master_; }
  void attach(Lane *other) { master_ = other; }
  void detach() { master_ = nullptr; }
  bool has_master() const { return (master_ != nullptr); }

  void update_master(const std::vector<Lane *> &others, int direction);
  void regain_master(const VirtualLane &partner,
                     const std::vector<Lane *> &others, int direction);

  int get_common_point_num(const RawRefLine &raw_refline);

  void update();

  void save_context(VirtualLaneContext &context) const;
  void restore_context(const VirtualLaneContext &context);

private:
  Lane *master_;
  int property_;
};

class VirtualLaneManager {
public:
  VirtualLaneManager(Lane &clane, Lane &llane, Lane &rlane, Lane &rrlane,
                     Lane &lllane, RawRefLine &c_raw_refline,
                     RawRefLine &l_raw_refline, RawRefLine &r_raw_refline,
                     RawRefLine &ll_raw_refline, RawRefLine &rr_raw_refline);

  VirtualLaneManager(VirtualLaneManager &source);

  virtual ~VirtualLaneManager() = default;

  void update(const VirtualLaneManager &source);

  bool assign_lc_lanes(int direction);
  void update_lc_lanes(int direction, int state);

  void clear_lc_lanes() {
    tlane_.detach();
    tlane_.update();
    olane_.detach();
    olane_.update();
  }

  void update_fix_lane();
  void update_fix_lane_info();
  bool update_fix_lane(int relative_id);
  void update_debug_info();

  bool is_on_lane(int lane);
  double dis_to_fixrefline();
  double dist_mline(int direction);

  void set_fix_lane(LaneProperty lane);
  void set_fix_lane(Lane *lane) { flane_.attach(lane); }

  bool has_target_lane() const { return tlane_.has_master(); }
  bool has_origin_lane() const { return olane_.has_master(); }

  bool lc_lanes_lost() const {
    return (!tlane_.has_master() && !olane_.has_master());
  }

  bool flane_update() const { return flane_update_; }

  const VirtualLane &get_fix_lane() const { return flane_; }
  const VirtualLane &get_origin_lane() const { return olane_; }
  const VirtualLane &get_target_lane() const { return tlane_; }
  const RefLine &get_fix_refline() const { return f_refline_; }
  const Lane &get_llane() const { return llane_; }
  const Lane &get_rlane() const { return rlane_; }

  VirtualLane &mutable_fix_lane() { return flane_; }
  VirtualLane &mutable_origin_lane() { return olane_; }
  VirtualLane &mutable_target_lane() { return tlane_; }
  RefLine &mutable_fix_refline() { return f_refline_; }

  VirtualLaneManager &operator=(const VirtualLaneManager &source);

  void save_context(VirtualLaneManagerContext &context) const;
  void restore_context(const VirtualLaneManagerContext &context);

private:
  bool flane_update_{false};
  VirtualLane flane_{LaneProperty::FIX_LANE};
  VirtualLane olane_{LaneProperty::ORIGIN_LANE};
  VirtualLane tlane_{LaneProperty::TARGET_LANE};
  RefLine f_refline_;

  Lane &clane_;
  Lane &llane_;
  Lane &rlane_;
  Lane &rrlane_;
  Lane &lllane_;

  RawRefLine &c_raw_refline_;
  RawRefLine &l_raw_refline_;
  RawRefLine &r_raw_refline_;
  RawRefLine &ll_raw_refline_;
  RawRefLine &rr_raw_refline_;
};

} // namespace msquare

#endif
