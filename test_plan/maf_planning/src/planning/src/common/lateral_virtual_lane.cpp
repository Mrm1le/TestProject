#include "common/lateral_virtual_lane.h"
#include "common/config_context.h"
#include "common/world_model.h"
#include "planner/behavior_planner/lateral_behavior_state.h"
#include "planning/common/common.h"

namespace msquare {

double calc_lane_width(const std::vector<double> &l_poly,
                       const std::vector<double> &r_poly) {
  if (l_poly.size() < 2 || r_poly.size() < 2) {
    return 0;
  }

  return std::max(std::fabs(l_poly[0] - r_poly[0]) /
                      std::sqrt(1 + pow(l_poly[1] + r_poly[1], 2) / 4.0),
                  2.0);
}

Lane::Lane(int position, RawRefLine *raw_refline) {
  min_width_.fill(kMinWidth);
  max_width_.fill(kMaxWidth);
  intercepts_.fill(DBL_MAX);
  dist_to_center_line_ = DBL_MAX;
  curvature_ = 0.0;
  relative_theta_ = 0.0;
  neighbours_.fill(nullptr);
  raw_refline_ = raw_refline;

  type_ = LaneType::UNKNOWN;
  width_ = kDefaultWidth;
  front_width_ = kDefaultWidth;
  status_ = LaneStatusEx::BOTH_MISSING;
  source_ = LaneSource::UNKNOWN_SOURCE;
  position_ = position;
  exist_ = false;
}

void Lane::update(
    const std::vector<LaneBoundaryPolylineDerived> &lane_boundary_polyline) {
  if (raw_refline_ == nullptr) {
    status_ = BOTH_MISSING;
    return;
  }
  double dist_to_center_line_square = raw_refline_->min_square_dist(0.0, 0.0);
  if (std::sqrt(std::abs(dist_to_center_line_square)) > 10 * max_width()) {
    status_ = BOTH_MISSING;
    return;
  }
  if (dist_to_center_line_square >= 0) {
    dist_to_center_line_ = -std::sqrt(dist_to_center_line_square);
  } else {
    dist_to_center_line_ = std::sqrt(-dist_to_center_line_square);
  }
  auto neanearest_waypoint = raw_refline_->nearest_waypoint();
  auto front_waypoint = raw_refline_->front_waypoint();

  if (!neanearest_waypoint.empty()) {
    width_ = neanearest_waypoint[0].lane_width;
    if (width_ > 2 * max_width()) {
      width_ = kDefaultWidth;
    }
    auto distance_to_left_lane_border =
        neanearest_waypoint[0].distance_to_left_lane_border;
    auto distance_to_right_lane_border =
        neanearest_waypoint[0].distance_to_right_lane_border;
    auto distance_to_left_road_border =
        neanearest_waypoint[0].distance_to_left_road_border;
    auto distance_to_right_road_border =
        neanearest_waypoint[0].distance_to_right_road_border;
    double left_dist =
        std::min(distance_to_left_lane_border, distance_to_left_road_border);
    double right_dist =
        std::min(distance_to_right_lane_border, distance_to_right_road_border);
    intercepts_[0] = dist_to_center_line_ + left_dist;
    intercepts_[1] = dist_to_center_line_ - right_dist;
    if (std::abs(left_dist) > max_width() &&
        std::abs(right_dist) < max_width()) {
      intercepts_[0] = intercepts_[1] + width_;
      status_ = RIGHT_AVAILABLE;
    } else if (std::abs(left_dist) < max_width() &&
               std::abs(right_dist) > max_width()) {
      intercepts_[1] = intercepts_[0] - width_;
      status_ = LEFT_AVAILABLE;
    } else if (std::abs(left_dist) > max_width() &&
               std::abs(right_dist) > max_width()) {
      intercepts_[0] = dist_to_center_line_ + width_ / 2;
      intercepts_[1] = dist_to_center_line_ - width_ / 2;
      status_ = BOTH_MISSING;
    } else {
      status_ = BOTH_AVAILABLE;
    }
    MSD_LOG(WARN,
            "Lane::update, position: %d, neanearest_waypoint; %f %f, "
            "dist_to_center_line_: %f, intercepts_: %f %f",
            position_, neanearest_waypoint[0].car_point.x,
            neanearest_waypoint[0].car_point.y, dist_to_center_line_,
            intercepts_[0], intercepts_[1]);
  } else {
    status_ = BOTH_MISSING;
    MSD_LOG(WARN, "Lane::update, neanearest_waypoint empty, %d", position_);
  }

  if (!front_waypoint.empty()) {
    front_width_ = front_waypoint[0].lane_width;
  }

  MSD_LOG(WARN, "Lane::update, position: %d, status: %d", position_, status_);

  curvature_ = 0.0;
  relative_theta_ = 0.0;
  if (world_model_ != nullptr) {
    auto baseline = world_model_->get_baseline_info(position_);
    auto ego_theta = world_model_->get_cart_ego_state_manager()
                         .get_cart_ego_state()
                         .ego_pose.theta;
    if (baseline != nullptr) {
      auto ego_frenet = baseline->get_ego_state().ego_frenet;
      auto frenet_coord = baseline->get_frenet_coord();
      if (frenet_coord != nullptr) {
        double curvature = frenet_coord->GetRefCurveCurvature(ego_frenet.x);
        double curve_heading = frenet_coord->GetRefCurveHeading(ego_frenet.x);
        double relative_theta = ego_theta - curve_heading;

        curvature_ = curvature;
        relative_theta_ = relative_theta;
        MSD_LOG(WARN,
                "Lane::update, position: %d, curvature; %f, relative_theta: %f",
                position_, curvature_, relative_theta_);
      }
    }
  }
}

bool Lane::has_lines(int side) const {
  mph_assert(side == INT_MAX || side == 0 || side == 1);

  if (side == INT_MAX) {
    return (status_ == LaneStatusEx::BOTH_AVAILABLE);
  } else if (side == 0) {
    return (status_ == LaneStatusEx::BOTH_AVAILABLE ||
            status_ == LaneStatusEx::LEFT_AVAILABLE);
  } else {
    return (status_ == LaneStatusEx::BOTH_AVAILABLE ||
            status_ == LaneStatusEx::RIGHT_AVAILABLE);
  }

  return false;
}

double Lane::dis_to_side_line(double x, double y, int side,
                              RefLine &fix_refline) {
  // 0 left 1 right
  mph_assert(side == 0 || side == 1);
  double temp_s, theta;
  double temp_l = 0.0;
  fix_refline.cartesian_frenet(x, y, temp_s, temp_l, theta);
  auto raw_refline = fix_refline.master();
  if (raw_refline == nullptr)
    return double(-100);
  auto waypoint = raw_refline->nearest_waypoint();
  if (waypoint.size() < 1)
    return double(-100);

  MSD_LOG(ERROR, "check_head_crosss d2l:%.2f,d2r:%.2f,temp_l:%.2f",
          waypoint.front().distance_to_left_lane_border,
          waypoint.front().distance_to_right_lane_border, temp_l);
  if (side == 0) {
    return waypoint.front().distance_to_left_lane_border - temp_l;
  } else {
    return waypoint.front().distance_to_right_lane_border + temp_l;
  }
  return double(-100.0);
}

double Lane::dis_to_line(double s, double l, int side, RefLine &fix_refline) {
  mph_assert(side == 0 || side == 1);

  double rx, ry;
  fix_refline.frenet_cartesian(s, 0.0, rx, ry);
  double ego_s, ego_l, theta;
  fix_refline.cartesian_frenet(0, 0, ego_s, ego_l, theta);

  double d_ref, d_line, d_line_ref;
  double d_tr_ref;

  if (raw_refline_ == nullptr) {
    return kInvalidDist;
  }

  if (raw_refline_->position() == fix_refline.position()) {
    d_tr_ref = 0;
  } else {
    double d_tr_ref_square = raw_refline_->min_square_dist(rx, ry);
    if (d_tr_ref_square >= 0) {
      d_tr_ref = -std::sqrt(d_tr_ref_square);
    } else {
      d_tr_ref = std::sqrt(-d_tr_ref_square);
    }
  }

  if (side == 0) {
    d_line_ref = d_tr_ref + 0.5 * kDefaultWidth;
  } else {
    d_line_ref = d_tr_ref - 0.5 * kDefaultWidth;
  }

  d_ref = l;
  d_line = d_ref - d_line_ref;

  return d_line;
}

bool Lane::is_track_on(const TrackedObject &tr, int track_type,
                       RefLine &fix_refline) {
  mph_assert(track_type == TrackType::FRONT_TRACK ||
             track_type == TrackType::SIDE_TRACK);

  mph_assert(position_ == LanePosition::CURR_POS ||
             position_ == LanePosition::LEFT_POS ||
             position_ == LanePosition::RIGHT_POS);

  if (exist_ == false) {
    return false;
  }

  if (tr.d_rel > 80 || tr.d_rel < -40 || tr.d_min_cpath > 7 ||
      tr.d_max_cpath < -7) {
    return false;
  }

  double d_min_l = dis_to_line(tr.s_min, tr.d_min_cpath, 0, fix_refline);
  double d_max_r = dis_to_line(tr.s_max, tr.d_max_cpath, 1, fix_refline);
  double check_offset = 0.0;

  std::array<double, 5> xp{-30, -20, 20, 30, 60};
  std::array<double, 5> fp{0.45, 0.2, 0.3, 0.45, 0.6};
  check_offset = interp(tr.d_rel, xp, fp);

  return (d_min_l < -check_offset && d_max_r > check_offset);
}

VirtualLane::VirtualLane(int property) {
  master_ = nullptr;
  property_ = property;
}

VirtualLane::~VirtualLane() {
  if (raw_refline_ != nullptr) {
    delete raw_refline_;
    raw_refline_ = nullptr;
  }
}

int VirtualLane::get_common_point_num(const RawRefLine &raw_refline) {
  if (raw_refline_ == nullptr) {
    return 0;
  }

  const std::map<int32_t, bool> &point_ids = raw_refline_->point_ids();
  const std::map<int32_t, bool> &other_point_ids = raw_refline.point_ids();

  int common_point_num = 0;
  for (auto &p : other_point_ids) {
    if (point_ids.find(p.first) != point_ids.end()) {
      common_point_num++;
    }
  }

  return common_point_num;
}

void VirtualLane::update_master(const std::vector<Lane *> &others,
                                int direction = RIGHT_CHANGE) {
  if (has_master() == false) {
    return;
  }

  bool master_update = false;

  auto lane_source = master_->source();
  if (lane_source == LaneSource::MAP_SOURCE) {
    double max_common_ratio = 0.1;

    for (size_t i = 0; i < others.size(); i++) {
      if (others[i] == nullptr || others[i]->exist() == false ||
          others[i]->raw_refline() == nullptr) {
        continue;
      }

      int common_point_num = get_common_point_num(*(others[i]->raw_refline()));
      double common_ratio = static_cast<double>(common_point_num) /
                            others[i]->raw_refline()->point_ids().size();
      if (common_ratio > max_common_ratio) {
        max_common_ratio = common_ratio;
        master_update = true;
        master_ = others[i];
      }
    }
  } else {
    // update master by intercepts if lane souce isn't map
    double max_coincide = 0;
    double max_coincide_rate = 0;
    const double COINCIDE_THRES = 0.75;
    for (const auto lane : others) {
      if (lane->exist() != true ||
          lane->status() != LaneStatusEx::BOTH_AVAILABLE)
        continue;
      if ((std::min(intercepts_[0], lane->intercepts()[0]) -
               std::max(intercepts_[1], lane->intercepts()[1]) >
           max_coincide) &&
          (intercepts_[0] - intercepts_[1] > 0)) {
        max_coincide = std::min(intercepts_[0], lane->intercepts()[0]) -
                       std::max(intercepts_[1], lane->intercepts()[1]);
        max_coincide_rate = max_coincide / (intercepts_[0] - intercepts_[1]);
        if (max_coincide_rate > COINCIDE_THRES) {
          master_ = lane;
          master_update = true;
          MSD_LOG(INFO,
                  "update master: coincide success! position[%d] updated "
                  "position[%d]",
                  position_, lane->position());
        }
      }
    }

    MSD_LOG(INFO, "update master: max conside[%.2f] rate[%.2f] position[%d]",
            max_coincide, max_coincide_rate, position_);

    double min_delta_center_lane_dist = DBL_MAX;
    const double CENTER_LANE_THRES = 0.5;
    if (!master_update) {
      for (const auto lane : others) {
        if (lane->exist() != true ||
            std::fabs(lane->dist_to_center_line()) > 10 * max_width())
          continue;
        double delta_center_lane_dist =
            std::fabs(dist_to_center_line_ - lane->dist_to_center_line());
        if (delta_center_lane_dist < min_delta_center_lane_dist) {
          min_delta_center_lane_dist = delta_center_lane_dist;
          if (delta_center_lane_dist < CENTER_LANE_THRES) {
            master_ = lane;
            master_update = true;
            MSD_LOG(INFO,
                    "update master: center lane success! position[%d] updated "
                    "position[%d]",
                    position_, lane->position());
          }
        }
      }
    }

    MSD_LOG(INFO,
            "update master: min_delta_center_lane_dist[%.2f] position[%d]",
            min_delta_center_lane_dist, position_);
  }

  if (master_update == false) {
    MSD_LOG(INFO, "update master: failed! position[%d]", position_);
    detach();
  }
}

void VirtualLane::regain_master(const VirtualLane &partner,
                                const std::vector<Lane *> &others,
                                int direction) {
  MSD_LOG(INFO, "regain master: position[%d]", position_);
  auto lane_source = partner.source();
  if (lane_source == LaneSource::MAP_SOURCE) {
    if ((property_ == LaneProperty::ORIGIN_LANE &&
         direction == RequestType::RIGHT_CHANGE) ||
        (property_ == LaneProperty::TARGET_LANE &&
         direction == RequestType::LEFT_CHANGE)) {

      for (size_t i = 0; i < others.size(); i++) {
        if (others[i] == nullptr || others[i]->exist() == false ||
            others[i]->raw_refline() == nullptr ||
            partner.raw_refline() == nullptr) {
          continue;
        }

        const std::array<RawRefLine *, 2> &neighbours =
            partner.raw_refline()->neighbours();

        if (neighbours[0] == nullptr) {
          continue;
        }

        if (others[i]->raw_refline()->position() == neighbours[0]->position()) {
          master_ = others[i];
          break;
        }
      }
    } else if ((property_ == LaneProperty::TARGET_LANE &&
                direction == RequestType::RIGHT_CHANGE) ||
               (property_ == LaneProperty::ORIGIN_LANE &&
                direction == RequestType::LEFT_CHANGE)) {
      for (size_t i = 0; i < others.size(); i++) {
        if (others[i] == nullptr || others[i]->exist() == false ||
            others[i]->raw_refline() == nullptr ||
            partner.raw_refline() == nullptr) {
          continue;
        }

        const std::array<RawRefLine *, 2> &neighbours =
            partner.raw_refline()->neighbours();

        if (neighbours[1] == nullptr) {
          continue;
        }

        if (others[i]->raw_refline()->position() == neighbours[1]->position()) {
          master_ = others[i];
          break;
        }
      }
    }
  } else {
    // regain master by intercepts if lane souce isn't map
    // use the fact that target and origin lane are always adjacent
    // for master lane: it should have at least one line (status != 3) and exist
    // cases that we should attach self to the partner's left lane
    const double INTERCEPT_THRES = 0.5;
    if ((property_ == LaneProperty::ORIGIN_LANE &&
         direction == RequestType::RIGHT_CHANGE) ||
        (property_ == LaneProperty::TARGET_LANE &&
         direction == RequestType::LEFT_CHANGE)) {
      for (auto lane : others) {
        if (partner.has_lines(0) && lane->exist() && lane->has_lines(1) &&
            std::abs(partner.intercepts()[0] - lane->intercepts()[1]) <
                INTERCEPT_THRES) {
          MSD_LOG(INFO, "regain master: success! position[%d]",
                  partner.position());
          master_ = lane;
          break;
        }
      }
    } else if ((property_ == LaneProperty::TARGET_LANE &&
                direction == RequestType::RIGHT_CHANGE) ||
               (property_ == LaneProperty::ORIGIN_LANE &&
                direction == RequestType::LEFT_CHANGE)) {
      for (auto lane : others) {
        if (partner.has_lines(1) && lane->exist() && lane->has_lines(0) &&
            std::abs(partner.intercepts()[1] - lane->intercepts()[0]) <
                INTERCEPT_THRES) {
          MSD_LOG(INFO, "regain master: success position[%d]",
                  partner.position());
          master_ = lane;
          break;
        }
      }
    }
  }
}

void VirtualLane::update() {
  if (master_ != nullptr) {
    intercepts_ = master_->intercepts();
    dist_to_center_line_ = master_->dist_to_center_line();
    curvature_ = master_->curvature();
    relative_theta_ = master_->relative_theta();
    type_ = master_->type();
    width_ = master_->width();
    front_width_ = master_->front_width();
    status_ = master_->status();
    source_ = master_->source();
    position_ = master_->position();
    exist_ = master_->exist();
    neighbours_ = master_->neighbours();

    if (raw_refline_ == nullptr) {
      raw_refline_ = new RawRefLine(*master_->raw_refline());
    } else {
      *raw_refline_ = *master_->raw_refline();
    }
  } else {
    intercepts_.fill(DBL_MAX);
    dist_to_center_line_ = DBL_MAX;
    curvature_ = 0.0;
    relative_theta_ = 0.0;
    neighbours_.fill(nullptr);

    if (raw_refline_ != nullptr) {
      delete raw_refline_;
    }

    raw_refline_ = nullptr;

    type_ = LaneType::UNKNOWN;
    width_ = kDefaultWidth;
    front_width_ = kDefaultWidth;
    status_ = LaneStatusEx::BOTH_MISSING;
    source_ = LaneSource::UNKNOWN_SOURCE;
    position_ = LanePosition::UNKNOWN_POS;
    exist_ = false;
  }
}

void VirtualLane::save_context(VirtualLaneContext &context) const {
  context.exist = exist_;
  context.type = type_;
  context.status = status_;
  context.source = source_;
  context.position = position_;
  context.width = width_;
  context.front_width = front_width_;
  context.intercepts = intercepts_;
  context.dist_to_center_line = dist_to_center_line_;
  context.curvature = curvature_;
  context.relative_theta = relative_theta_;

  if (raw_refline_ != nullptr) {
    raw_refline_->save_context(context.raw_refline);
    context.has_raw_refline = true;
  } else {
    context.has_raw_refline = false;
  }

  if (master_ != nullptr) {
    context.master_position = master_->position();
  } else {
    context.master_position = LanePosition::UNKNOWN_POS;
  }
}

void VirtualLane::restore_context(const VirtualLaneContext &context) {
  exist_ = context.exist;
  type_ = context.type;
  status_ = context.status;
  source_ = context.source;
  position_ = context.position;
  width_ = context.width;
  front_width_ = context.front_width;
  intercepts_ = context.intercepts;
  dist_to_center_line_ = context.dist_to_center_line;
  curvature_ = context.curvature;
  relative_theta_ = context.relative_theta;

  if (context.has_raw_refline) {
    if (raw_refline_ == nullptr) {
      raw_refline_ = new RawRefLine(-100);
    } else {
      raw_refline_->reset();
    }

    raw_refline_->restore_context(context.raw_refline);
  }
}

VirtualLaneManager::VirtualLaneManager(Lane &clane, Lane &llane, Lane &rlane,
                                       Lane &rrlane, Lane &lllane,
                                       RawRefLine &c_raw_refline,
                                       RawRefLine &l_raw_refline,
                                       RawRefLine &r_raw_refline,
                                       RawRefLine &ll_raw_refline,
                                       RawRefLine &rr_raw_refline)
    : clane_(clane), llane_(llane), rlane_(rlane), rrlane_(rrlane),
      lllane_(lllane), c_raw_refline_(c_raw_refline),
      l_raw_refline_(l_raw_refline), r_raw_refline_(r_raw_refline),
      ll_raw_refline_(ll_raw_refline), rr_raw_refline_(rr_raw_refline) {}

VirtualLaneManager::VirtualLaneManager(VirtualLaneManager &source)
    : clane_(source.clane_), llane_(source.llane_), rlane_(source.rlane_),
      rrlane_(source.rrlane_), lllane_(source.lllane_),
      c_raw_refline_(source.c_raw_refline_),
      l_raw_refline_(source.l_raw_refline_),
      r_raw_refline_(source.r_raw_refline_),
      ll_raw_refline_(source.ll_raw_refline_),
      rr_raw_refline_(source.rr_raw_refline_) {
  VirtualLaneManagerContext context;
  source.save_context(context);
  restore_context(context);
}

void VirtualLaneManager::update(const VirtualLaneManager &source) {
  clane_ = source.clane_;
  llane_ = source.llane_;
  rlane_ = source.rlane_;
  rrlane_ = source.rrlane_;
  lllane_ = source.lllane_;
  c_raw_refline_ = source.c_raw_refline_;
  l_raw_refline_ = source.l_raw_refline_;
  r_raw_refline_ = source.r_raw_refline_;
  ll_raw_refline_ = source.ll_raw_refline_;
  rr_raw_refline_ = source.rr_raw_refline_;

  static VirtualLaneManagerContext context;
  source.save_context(context);
  restore_context(context);
}

bool VirtualLaneManager::assign_lc_lanes(int direction) {
  mph_assert(tlane_.has_master() == false && olane_.has_master() == false);

  if (direction == RequestType::LEFT_CHANGE) {
    if (llane_.is_legal_4_lc()) {
      tlane_.attach(&llane_);
      olane_.attach(&clane_);
      tlane_.update();
      olane_.update();
      return true;
    } else {
      return false;
    }
  } else if (direction == RequestType::RIGHT_CHANGE) {
    if (rlane_.is_legal_4_lc()) {
      tlane_.attach(&rlane_);
      olane_.attach(&clane_);
      tlane_.update();
      olane_.update();
      return true;
    } else {
      return false;
    }
  } else {
    MSD_LOG(
        ERROR,
        "[VirtualLaneManager::assign_lc_lanes] Illegal direction[%d] argument",
        direction);
    return false;
  }

  return true;
}

void VirtualLaneManager::update_lc_lanes(int direction, int state) {
  if (direction != RequestType::LEFT_CHANGE &&
      direction != RequestType::RIGHT_CHANGE) {
    MSD_LOG(
        ERROR,
        "[VirtualLaneManager::update_lc_lanes] Illegal direction[%d] argument!",
        direction);
    return;
  }

  mph_assert(tlane_.has_master() == true || olane_.has_master() == true);

  std::vector<Lane *> lanes{&clane_, &rlane_, &llane_};
  tlane_.update_master(lanes, direction);
  olane_.update_master(lanes, direction);
  tlane_.update();
  olane_.update();
  update_debug_info();

  if (tlane_.has_master() && !olane_.has_master()) {
    olane_.regain_master(tlane_, lanes, direction);
    if (olane_.has_master()) {
      olane_.update();
    }
  } else if (olane_.has_master() && !tlane_.has_master()) {
    tlane_.regain_master(olane_, lanes, direction);
    if (tlane_.has_master()) {
      tlane_.update();
    }
  }
}

void VirtualLaneManager::update_fix_lane() {
  auto lane_source = flane_.source();
  if (lane_source != LaneSource::MAP_SOURCE) {
    // set fix lane to clane when lane source isn't map
    set_fix_lane(CURRENT_LANE);
    return;
  }

  std::vector<Lane *> others{&clane_, &rlane_, &llane_};

  Lane *matched_lane = nullptr;
  bool master_update = false;
  double max_common_ratio = 0.3;

  for (size_t i = 0; i < others.size(); i++) {
    if (others[i] == nullptr || others[i]->exist() == false ||
        others[i]->raw_refline() == nullptr) {
      continue;
    }

    RawRefLine *raw_refline = others[i]->raw_refline();
    int common_point_num = flane_.get_common_point_num(*raw_refline);

    double common_ratio =
        static_cast<double>(common_point_num) / raw_refline->point_ids().size();
    if (common_ratio > max_common_ratio) {
      max_common_ratio = common_ratio;
      matched_lane = others[i];
      set_fix_lane(others[i]);
      master_update = true;
    }
  }

  if (matched_lane != nullptr && matched_lane != &clane_) {
    RawRefLine *raw_refline = matched_lane->raw_refline();
    if (raw_refline != nullptr &&
        std::fabs(raw_refline->min_square_dist()) > 2.0 * 2.0) {
      set_fix_lane(CURRENT_LANE);
    }
  }

  if (master_update == false) {
    set_fix_lane(CURRENT_LANE);
  }
}

bool VirtualLaneManager::update_fix_lane(int relative_id) {
  std::vector<Lane *> others{&llane_, &clane_, &rlane_};
  int lane_index = relative_id + 1;
  if (others[lane_index] != nullptr && others[lane_index]->exist() &&
      others[lane_index]->raw_refline() != nullptr) {
    set_fix_lane(others[lane_index]);
    return true;
  }

  MSD_LOG(WARN,
          "[VirtualLaneManager::update_fix_lane] Failed, relative_id: %d, lane "
          "vaild: %d, lane exist: %d, raw_refline valid: %d",
          relative_id, others[lane_index] != nullptr,
          others[lane_index]->exist(),
          others[lane_index]->raw_refline() != nullptr);
  return false;
}

void VirtualLaneManager::update_fix_lane_info() {
  flane_.update();

  if (flane_.has_master() == false) {
    MSD_LOG(WARN,
            "[VirtualLaneManager::update_fix_lane_info] Warining! Fix lane "
            "has no master");
  }

  f_refline_.set_master(flane_.raw_refline());
  f_refline_.update_pathpoints();
  flane_update_ = true;
}

bool VirtualLaneManager::is_on_lane(int lane) {
  if (lane == LaneProperty::TARGET_LANE) {
    return tlane_.has_master() && tlane_.master() == &clane_;
  } else if (lane == LaneProperty::ORIGIN_LANE) {
    if (!(olane_.has_master() == true)) {
      olane_.attach(&clane_);
    }
    return olane_.master() == &clane_;
  }

  return false;
}

double VirtualLaneManager::dis_to_fixrefline() {
  double s, theta;
  double l = 0.0;
  f_refline_.cartesian_frenet(0, 0, s, l, theta);
  return l;
}

double VirtualLaneManager::dist_mline(int direction) {
  mph_assert(direction == LEFT_CHANGE || direction == RIGHT_CHANGE);

  double s, l, theta;
  f_refline_.cartesian_frenet(0, 0, s, l, theta);

  if (tlane_.has_master()) {
    int side = (direction == LEFT_CHANGE) ? 1 : 0;
    int sign = (direction == LEFT_CHANGE) ? -1 : 1;

    if (tlane_.has_lines(side)) {
      double dist = std::abs(tlane_.intercepts()[side]);
      MSD_LOG(WARN, "dist_mline, tlane: [%f]", dist);
      return dist;
    } else {
      double dist_tlane = tlane_.dis_to_line(s, l, side, f_refline_);
      double dist = std::abs(dist_tlane);
      MSD_LOG(WARN, "dist_mline, tlane: [%f]", dist);
      return dist;
    }
  }

  if (olane_.has_master()) {
    int side = (direction == LEFT_CHANGE) ? 0 : 1;
    int sign = (direction == RIGHT_CHANGE) ? -1 : 1;

    if (olane_.has_lines(side)) {
      double dist = std::abs(olane_.intercepts()[side]);
      MSD_LOG(WARN, "dist_mline, olane: [%f]", dist);
      return dist;
    } else {
      double dist_olane = olane_.dis_to_line(s, l, side, f_refline_);
      double dist = std::abs(dist_olane);
      MSD_LOG(WARN, "dist_mline, olane: [%f]", dist);
      return dist;
    }
  }

  return DBL_MAX;
}

void VirtualLaneManager::set_fix_lane(LaneProperty lane) {
  if (lane == LaneProperty::TARGET_LANE) {
    mph_assert(tlane_.has_master() == true);
    flane_.attach(tlane_.master());
  } else if (lane == LaneProperty::ORIGIN_LANE) {
    mph_assert(olane_.has_master());
    flane_.attach(olane_.master());
  } else if (lane == LaneProperty::CURRENT_LANE) {
    flane_.attach(&clane_);
  } else {
    MSD_LOG(ERROR,
            "[VirtualLaneManager::set_fix_lane] Illegal lane[%d] argument",
            lane);
    flane_.attach(&clane_);
  }
}

VirtualLaneManager &
VirtualLaneManager::operator=(const VirtualLaneManager &source) {
  if (this == &source) {
    return *this;
  }
  VirtualLaneManagerContext context;
  source.save_context(context);
  restore_context(context);
  return *this;
}

void VirtualLaneManager::save_context(
    VirtualLaneManagerContext &context) const {
  context.flane_update = flane_update_;
  flane_.save_context(context.flane);
  olane_.save_context(context.olane);
  tlane_.save_context(context.tlane);
  f_refline_.save_context(context.f_refline);
}

void VirtualLaneManager::restore_context(
    const VirtualLaneManagerContext &context) {
  auto restore_master = [this](VirtualLane &target_lane, int master_position) {
    switch (master_position) {
    case LEFT_LEFT_POS:
      target_lane.attach(&lllane_);
      target_lane.set_neighbours(lllane_.neighbours());
      if (target_lane.has_raw_refline() && lllane_.has_raw_refline()) {
        auto raw_refline = target_lane.raw_refline();
        raw_refline->set_neighbours(lllane_.raw_refline()->neighbours());
      }
      break;

    case LEFT_POS:
      target_lane.attach(&llane_);
      target_lane.set_neighbours(llane_.neighbours());
      if (target_lane.has_raw_refline() && llane_.has_raw_refline()) {
        auto raw_refline = target_lane.raw_refline();
        raw_refline->set_neighbours(llane_.raw_refline()->neighbours());
      }
      break;

    case CURR_POS:
      target_lane.attach(&clane_);
      target_lane.set_neighbours(clane_.neighbours());
      if (target_lane.has_raw_refline() && clane_.has_raw_refline()) {
        auto raw_refline = target_lane.raw_refline();
        raw_refline->set_neighbours(clane_.raw_refline()->neighbours());
      }
      break;

    case RIGHT_POS:
      target_lane.attach(&rlane_);
      target_lane.set_neighbours(rlane_.neighbours());
      if (target_lane.has_raw_refline() && rlane_.has_raw_refline()) {
        auto raw_refline = target_lane.raw_refline();
        raw_refline->set_neighbours(rlane_.raw_refline()->neighbours());
      }
      break;

    case RIGHT_RIGHT_POS:
      target_lane.attach(&rrlane_);
      target_lane.set_neighbours(rrlane_.neighbours());
      if (target_lane.has_raw_refline() && rrlane_.has_raw_refline()) {
        auto raw_refline = target_lane.raw_refline();
        raw_refline->set_neighbours(rrlane_.raw_refline()->neighbours());
      }
      break;

    default:
      target_lane.detach();
      target_lane.update();
      break;
    }
  };

  flane_update_ = context.flane_update;
  flane_.restore_context(context.flane);
  olane_.restore_context(context.olane);
  tlane_.restore_context(context.tlane);

  restore_master(flane_, context.flane.master_position);
  restore_master(olane_, context.olane.master_position);
  restore_master(tlane_, context.tlane.master_position);

  f_refline_.restore_context(context.f_refline);
  switch (f_refline_.position()) {
  case CURR_REFLINE:
    f_refline_.set_master(&c_raw_refline_);
    break;
  case LEFT_REFLINE:
    f_refline_.set_master(&l_raw_refline_);
    break;
  case RIGHT_REFLINE:
    f_refline_.set_master(&r_raw_refline_);
    break;
  case LEFT_LEFT_REFLINE:
    f_refline_.set_master(&ll_raw_refline_);
    break;
  case RIGHT_RIGHT_REFLINE:
    f_refline_.set_master(&rr_raw_refline_);
    break;
  default:
    break;
  }
}
void VirtualLaneManager::update_debug_info(void) {
  auto planner_debug = PlanningContext::Instance()->mutable_planner_debug();
  std::vector<Lane *> lanes{&clane_, &rlane_, &llane_};
  // MSD_LOG(INFO,"fengxiaotong0507 size is %d",lanes.size());
  planner_debug->lat_dec_info.lane_list.clear();
  for (auto lane : lanes) {
    if (lane->exist()) {
      VirtualLaneDebugInfo temp_lane;
      temp_lane.id = lane->position();
      temp_lane.intercept_l = lane->intercepts()[0];
      temp_lane.intercept_r = lane->intercepts()[1];
      planner_debug->lat_dec_info.lane_list.emplace_back(temp_lane);
    }
  }
}

} // namespace msquare
