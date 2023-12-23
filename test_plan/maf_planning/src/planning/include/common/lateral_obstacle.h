#ifndef LATERAL_OBSTACLE_H
#define LATERAL_OBSTACLE_H

#include "common/ego_state_manager.h"
#include "common/map_info_manager.h"
#include "common/prediction_object.h"
#include "common/tracked_object.h"
#include "common/tracklet_maintainer.h"

#include <utility>

namespace msquare {

class LateralObstacle {
public:
  LateralObstacle();
  virtual ~LateralObstacle();

  bool update(const EgoState &ego_state,
              const std::vector<PredictionObject> &predictions,
              const RefLine &f_refline, bool isRedLightStop);

  bool fvf_dead() const { return fvf_dead_; }
  bool svf_dead() const { return svf_dead_; }
  bool sensors_okay() const { return (!fvf_dead_ && !svf_dead_); }

  TrackedObject *leadone() { return lead_cars_.lead_one; }
  TrackedObject *leadtwo() { return lead_cars_.lead_two; }
  TrackedObject *tleadone() { return lead_cars_.temp_lead_one; }
  TrackedObject *tleadtwo() { return lead_cars_.temp_lead_two; }

  LeadCars &get_lead_cars() { return lead_cars_; }

  void set_prediction_update(bool value) { prediction_update_ = value; }

  const std::vector<TrackedObject> &front_tracks() const {
    return front_tracks_;
  }

  const std::vector<TrackedObject> &front_tracks_copy() const {
    return front_tracks_copy_;
  }

  const std::vector<TrackedObject> &front_tracks_l() const {
    return front_tracks_l_;
  }

  const std::vector<TrackedObject> &front_tracks_r() const {
    return front_tracks_r_;
  }

  const std::vector<TrackedObject> &side_tracks() const { return side_tracks_; }

  const std::vector<TrackedObject> &side_tracks_l() const {
    return side_tracks_l_;
  }

  const std::vector<TrackedObject> &side_tracks_r() const {
    return side_tracks_r_;
  }

  bool find_track(int track_id, TrackedObject &dest);

private:
  bool update_sensors(const EgoState &ego_state,
                      const std::vector<PredictionObject> &predictions,
                      const RefLine &f_refline, bool isRedLightStop);
  void update_tracks(const std::vector<TrackedObject> &tracked_objects);

  double fvf_time_ = 0.0;
  bool fvf_dead_ = true;
  double svf_time_ = 0.0;
  bool svf_dead_ = true;
  double warning_timer_[5];
  bool prediction_update_ = false;

  std::vector<TrackedObject> front_tracks_;
  std::vector<TrackedObject> front_tracks_copy_;
  std::vector<TrackedObject> front_tracks_l_;
  std::vector<TrackedObject> front_tracks_r_;
  std::vector<TrackedObject> side_tracks_;
  std::vector<TrackedObject> side_tracks_l_;
  std::vector<TrackedObject> side_tracks_r_;

  LeadCars lead_cars_;
  TrackletMaintainer maintainer_;
};

class LaneTracksManager {
public:
  LaneTracksManager(MapInfoManager &map_info_mgr,
                    LateralObstacle &lateral_obstacle,
                    VirtualLaneManager &virtual_lane_mgr);
  virtual ~LaneTracksManager() = default;

  void update_ego_state(const EgoState &ego_state) { ego_state_ = ego_state; }

  const std::vector<TrackedObject> &front_tracks_l() const {
    return front_tracks_llane_;
  }
  const std::vector<TrackedObject> &front_tracks_r() const {
    return front_tracks_rlane_;
  }
  const std::vector<TrackedObject> &front_tracks_c() const {
    return front_tracks_clane_;
  }
  const std::vector<TrackedObject> &get_front_tracks() const {
    return lateral_obstacle_.front_tracks();
  }
  const std::vector<TrackedObject> &get_side_tracks() const {
    return lateral_obstacle_.side_tracks();
  }
  std::vector<TrackedObject> *get_lane_tracks(int lane, int track_type);

  void save_context(const LaneTracksManagerContext &context) const {}
  void restore_context(const LaneTracksManagerContext &context) {}

  void update() { lane_tracks_update_.clear(); }

private:
  void update_lane_tracks(int track_type, int position);

  std::set<std::tuple<int, int>> lane_tracks_update_;
  std::vector<TrackedObject> front_tracks_clane_;
  std::vector<TrackedObject> front_tracks_llane_;
  std::vector<TrackedObject> front_tracks_rlane_;
  std::vector<TrackedObject> front_tracks_lllane_;
  std::vector<TrackedObject> front_tracks_rrlane_;
  std::vector<TrackedObject> side_tracks_clane_;
  std::vector<TrackedObject> side_tracks_llane_;
  std::vector<TrackedObject> side_tracks_rlane_;
  std::vector<TrackedObject> side_tracks_lllane_;
  std::vector<TrackedObject> side_tracks_rrlane_;
  std::vector<TrackedObject> empty_tracks_;

  MapInfoManager &map_info_mgr_;
  LateralObstacle &lateral_obstacle_;
  VirtualLaneManager &virtual_lane_mgr_;
  EgoState ego_state_;
};

} // namespace msquare

#endif
