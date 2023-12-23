#include "common/lateral_obstacle.h"
#include "planning/common/common.h"

namespace msquare {

LateralObstacle::LateralObstacle() {
  double curr_time = get_system_time();
  warning_timer_[0] = curr_time;
  warning_timer_[1] = curr_time - 1;
  warning_timer_[2] = curr_time - 2;
  warning_timer_[3] = curr_time - 3;
  warning_timer_[4] = curr_time;

  lead_cars_.lead_one = nullptr;
  lead_cars_.lead_two = nullptr;
  lead_cars_.temp_lead_one = nullptr;
  lead_cars_.temp_lead_two = nullptr;
}

LateralObstacle::~LateralObstacle() { lead_cars_.clear(); }

bool LateralObstacle::update(const EgoState &ego_state,
                             const std::vector<PredictionObject> &predictions,
                             const RefLine &f_refline, bool isRedLightStop) {
  (void)update_sensors(ego_state, predictions, f_refline, isRedLightStop);
  return true;
}

bool LateralObstacle::update_sensors(
    const EgoState &ego_state, const std::vector<PredictionObject> &predictions,
    const RefLine &f_refline, bool isRedLightStop) {

  double fusion_timeout = 0.5;
  double curr_time = PlanningContext::Instance()
                         ->planning_status()
                         .planning_result.next_timestamp_sec;
  std::vector<TrackedObject> tracked_objects;

  if (prediction_update_) {
    maintainer_.apply_update(ego_state, predictions, f_refline, tracked_objects,
                             lead_cars_, isRedLightStop);
    update_tracks(tracked_objects);

    prediction_update_ = false;
    fvf_time_ = curr_time;
    svf_time_ = curr_time;

    fvf_dead_ = false;
    svf_dead_ = false;
  } else if (curr_time - fvf_time_ > fusion_timeout) {
    front_tracks_.clear();
    front_tracks_copy_.clear();
    front_tracks_l_.clear();
    front_tracks_r_.clear();
    side_tracks_.clear();
    side_tracks_l_.clear();
    side_tracks_r_.clear();

    lead_cars_.clear();
    fvf_dead_ = true;
    svf_dead_ = true;

    MSD_LOG(ERROR,
            "[LateralObstacle::update_sensors] Fusion lagging too large");

    double abs_time = get_system_time();
    if (abs_time - warning_timer_[2] > 5.0) {
      MSD_LOG(ERROR,
              "[LateralObstacle::update_sensors] frontview fusion unavailable");
      warning_timer_[2] = abs_time;
    }

    if (abs_time - warning_timer_[1] > 5.0) {
      MSD_LOG(ERROR,
              "[LateralObstacle::update_sensors] sideview fusion unavailable");
      warning_timer_[1] = abs_time;
    }
  }

  return true;
}

void LateralObstacle::update_tracks(
    const std::vector<TrackedObject> &tracked_objects) {

  front_tracks_.clear();
  front_tracks_copy_.clear();
  front_tracks_l_.clear();
  front_tracks_r_.clear();
  side_tracks_.clear();
  side_tracks_l_.clear();
  side_tracks_r_.clear();

  for (auto &tr : tracked_objects) {
    if (tr.d_rel >= 0.0) {
      auto it = front_tracks_.begin();
      while (it != front_tracks_.end() && it->d_rel < tr.d_rel) {
        ++it;
      }

      if (it != front_tracks_.end()) {
        front_tracks_.insert(it, tr);
      } else {
        front_tracks_.push_back(tr);
      }
    } else {
      auto it = side_tracks_.begin();
      while (it != side_tracks_.end() && it->d_rel > tr.d_rel) {
        ++it;
      }

      if (it != side_tracks_.end()) {
        side_tracks_.insert(it, tr);
      } else {
        side_tracks_.push_back(tr);
      }
    }
  }

  front_tracks_copy_ = front_tracks_;

  for (auto &tr : front_tracks_) {
    if (tr.y_center_rel >= 0) {
      front_tracks_l_.push_back(tr);
    } else {
      front_tracks_r_.push_back(tr);
    }
  }

  for (auto &tr : side_tracks_) {
    if (tr.y_center_rel >= 0) {
      side_tracks_l_.push_back(tr);
    } else {
      side_tracks_r_.push_back(tr);
    }
  }
}

bool LateralObstacle::find_track(int track_id, TrackedObject &dest) {
  for (auto &tr : front_tracks_) {
    if (tr.track_id == track_id) {
      dest = tr;
      return true;
    }
  }

  for (auto &tr : side_tracks_) {
    if (tr.track_id == track_id) {
      dest = tr;
      return true;
    }
  }

  return false;
}

LaneTracksManager::LaneTracksManager(MapInfoManager &map_info_mgr,
                                     LateralObstacle &lateral_obstacle,
                                     VirtualLaneManager &virtual_lane_mgr)
    : map_info_mgr_(map_info_mgr), lateral_obstacle_(lateral_obstacle),
      virtual_lane_mgr_(virtual_lane_mgr) {}

std::vector<TrackedObject> *LaneTracksManager::get_lane_tracks(int lane,
                                                               int track_type) {
  if (lane != LaneProperty::TARGET_LANE && lane != LaneProperty::ORIGIN_LANE &&
      lane != LaneProperty::CURRENT_LANE && lane != LaneProperty::LEFT_LANE &&
      lane != LaneProperty::RIGHT_LANE) {
    MSD_LOG(ERROR,
            "[LaneTracksManager::get_lane_tracks] Illegal lane property[%d]",
            lane);
    return nullptr;
  }

  if (track_type != TrackType::SIDE_TRACK &&
      track_type != TrackType::FISHEYE_TRACK &&
      track_type != TrackType::FRONT_TRACK) {
    MSD_LOG(ERROR,
            "[LaneTracksManager::get_lane_tracks] Illegal track_type[%d]",
            track_type);
    return nullptr;
  }

  auto &clane = map_info_mgr_.clane_;
  auto &llane = map_info_mgr_.llane_;
  auto &rlane = map_info_mgr_.rlane_;
  auto &lllane = map_info_mgr_.lllane_;
  auto &rrlane = map_info_mgr_.rrlane_;
  auto &olane = virtual_lane_mgr_.mutable_origin_lane();
  auto &tlane = virtual_lane_mgr_.mutable_target_lane();

  if (lane == LaneProperty::TARGET_LANE) {
    mph_assert(tlane.has_master());

    if (tlane.has_master() == false) {
      MSD_LOG(ERROR,
              "[LaneTracksManager::get_lane_tracks] target lane has no master");
      return &empty_tracks_;
    }

    Lane *master = tlane.master();
    if (master != &clane && master != &llane && master != &rlane) {
      MSD_LOG(
          ERROR,
          "[LaneTracksManager::get_lane_tracks] Wrong target lane position[%d]",
          master->position());
      return &empty_tracks_;
    }
  }

  if (lane == LaneProperty::ORIGIN_LANE) {
    // mph_assert(map_info_manager_.olane_.has_master());
    if (!(olane.has_master() == true)) {
      olane.attach(&clane);
    }

    Lane *master = olane.master();
    if (master != &clane && master != &llane && master != &rlane) {
      MSD_LOG(
          ERROR,
          "[LateralObstacle::get_lane_tracks] Wrong origin lane position[%d]",
          master->position());
      return &empty_tracks_;
    }
  }

  if (lane == LaneProperty::LEFT_LANE ||
      (lane == LaneProperty::TARGET_LANE && tlane.master() == &llane) ||
      (lane == LaneProperty::ORIGIN_LANE && olane.master() == &llane)) {
    update_lane_tracks(track_type, LanePosition::LEFT_POS);

    if (track_type == TrackType::SIDE_TRACK) {
      return &side_tracks_llane_;
    } else if (track_type == TrackType::FRONT_TRACK) {
      return &front_tracks_llane_;
    }
  }

  if (lane == LaneProperty::RIGHT_LANE ||
      (lane == LaneProperty::TARGET_LANE && tlane.master() == &rlane) ||
      (lane == LaneProperty::ORIGIN_LANE && olane.master() == &rlane)) {
    update_lane_tracks(track_type, LanePosition::RIGHT_POS);

    if (track_type == TrackType::SIDE_TRACK) {
      return &side_tracks_rlane_;
    } else if (track_type == TrackType::FRONT_TRACK) {
      return &front_tracks_rlane_;
    }
  }

  if (lane == LaneProperty::LEFT_LEFT_LANE ||
      (lane == LaneProperty::TARGET_LANE && tlane.master() == &lllane) ||
      (lane == LaneProperty::ORIGIN_LANE && olane.master() == &lllane)) {
    update_lane_tracks(track_type, LanePosition::LEFT_LEFT_POS);

    if (track_type == TrackType::SIDE_TRACK) {
      return &side_tracks_lllane_;
    } else if (track_type == TrackType::FRONT_TRACK) {
      return &front_tracks_lllane_;
    }
  }

  if (lane == LaneProperty::RIGHT_RIGHT_LANE ||
      (lane == LaneProperty::TARGET_LANE && tlane.master() == &rrlane) ||
      (lane == LaneProperty::ORIGIN_LANE && olane.master() == &rrlane)) {
    update_lane_tracks(track_type, LanePosition::RIGHT_RIGHT_POS);

    if (track_type == TrackType::SIDE_TRACK) {
      return &side_tracks_rrlane_;
    } else if (track_type == TrackType::FRONT_TRACK) {
      return &front_tracks_rrlane_;
    }
  }

  if (lane == LaneProperty::CURRENT_LANE ||
      (lane == LaneProperty::TARGET_LANE && tlane.master() == &clane) ||
      (lane == LaneProperty::ORIGIN_LANE && olane.master() == &clane)) {
    update_lane_tracks(track_type, LanePosition::CURR_POS);

    if (track_type == TrackType::SIDE_TRACK) {
      return &side_tracks_clane_;
    } else if (track_type == TrackType::FRONT_TRACK) {
      return &front_tracks_clane_;
    }
  }

  return nullptr;
}

void LaneTracksManager::update_lane_tracks(int track_type, int position) {
  std::tuple<int, int> track_info(track_type, position);

  auto &clane = map_info_mgr_.clane_;
  auto &llane = map_info_mgr_.llane_;
  auto &rlane = map_info_mgr_.rlane_;
  auto &lllane = map_info_mgr_.lllane_;
  auto &rrlane = map_info_mgr_.rrlane_;
  auto &f_refline = virtual_lane_mgr_.mutable_fix_refline();
  auto &front_tracks = lateral_obstacle_.front_tracks();
  auto &side_tracks = lateral_obstacle_.side_tracks();

  if (track_type == TrackType::FRONT_TRACK) {
    if (position == LanePosition::CURR_POS) {
      if (lane_tracks_update_.count(track_info) == 0) {
        front_tracks_clane_.clear();
        for (auto &tr : front_tracks) {
          if (clane.is_track_on(tr, track_type, f_refline)) {
            front_tracks_clane_.push_back(tr);
          }
        }

        lane_tracks_update_.insert(track_info);
      }
    } else if (position == LanePosition::LEFT_POS) {
      if (lane_tracks_update_.count(track_info) == 0) {
        front_tracks_llane_.clear();
        for (auto &tr : front_tracks) {
          if (llane.is_track_on(tr, track_type, f_refline)) {
            front_tracks_llane_.push_back(tr);
          }
        }

        lane_tracks_update_.insert(track_info);
      }
    } else if (position == LanePosition::RIGHT_POS) {
      if (lane_tracks_update_.count(track_info) == 0) {
        front_tracks_rlane_.clear();
        for (auto &tr : front_tracks) {
          if (rlane.is_track_on(tr, track_type, f_refline)) {
            front_tracks_rlane_.push_back(tr);
          }
        }

        lane_tracks_update_.insert(track_info);
      }
    } else if (position == LanePosition::LEFT_LEFT_POS) {
      if (lane_tracks_update_.count(track_info) == 0) {
        front_tracks_lllane_.clear();
        for (auto &tr : front_tracks) {
          if (lllane.is_track_on(tr, track_type, f_refline)) {
            front_tracks_lllane_.push_back(tr);
          }
        }

        lane_tracks_update_.insert(track_info);
      }
    } else if (position == LanePosition::RIGHT_RIGHT_POS) {
      if (lane_tracks_update_.count(track_info) == 0) {
        front_tracks_rrlane_.clear();
        for (auto &tr : front_tracks) {
          if (rrlane.is_track_on(tr, track_type, f_refline)) {
            front_tracks_rrlane_.push_back(tr);
          }
        }

        lane_tracks_update_.insert(track_info);
      }
    } else {
      MSD_LOG(
          ERROR,
          "[LateralObstacle::update_lane_tracks] Illegal position[%d] argument",
          position);
      return;
    }
  } else if (track_type == TrackType::SIDE_TRACK) {
    if (position == LanePosition::CURR_POS) {
      if (lane_tracks_update_.count(track_info) == 0) {
        side_tracks_clane_.clear();
        for (auto &tr : side_tracks) {
          if (clane.is_track_on(tr, track_type, f_refline)) {
            side_tracks_clane_.push_back(tr);
          }
        }

        lane_tracks_update_.insert(track_info);
      }
    } else if (position == LanePosition::LEFT_POS) {
      if (lane_tracks_update_.count(track_info) == 0) {
        side_tracks_llane_.clear();
        for (auto &tr : side_tracks) {
          if (llane.is_track_on(tr, track_type, f_refline)) {
            side_tracks_llane_.push_back(tr);
          }
        }

        lane_tracks_update_.insert(track_info);
      }
    } else if (position == LanePosition::RIGHT_POS) {
      if (lane_tracks_update_.count(track_info) == 0) {
        side_tracks_rlane_.clear();
        for (auto &tr : side_tracks) {
          if (rlane.is_track_on(tr, track_type, f_refline)) {
            side_tracks_rlane_.push_back(tr);
          }
        }

        lane_tracks_update_.insert(track_info);
      }
    } else if (position == LanePosition::LEFT_LEFT_POS) {
      if (lane_tracks_update_.count(track_info) == 0) {
        side_tracks_lllane_.clear();
        for (auto &tr : side_tracks) {
          if (lllane.is_track_on(tr, track_type, f_refline)) {
            side_tracks_lllane_.push_back(tr);
          }
        }

        lane_tracks_update_.insert(track_info);
      }
    } else if (position == LanePosition::RIGHT_RIGHT_POS) {
      if (lane_tracks_update_.count(track_info) == 0) {
        side_tracks_rrlane_.clear();
        for (auto &tr : side_tracks) {
          if (rrlane.is_track_on(tr, track_type, f_refline)) {
            side_tracks_rrlane_.push_back(tr);
          }
        }

        lane_tracks_update_.insert(track_info);
      }
    } else {
      MSD_LOG(
          ERROR,
          "[LateralObstacle::update_lane_tracks] Illegal position[%d] argument",
          position);
      return;
    }
  }
}

} // namespace msquare
