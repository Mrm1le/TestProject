#pragma once
#include <data_driven_planner/common/basic_types.h>
#include <deque>

namespace msquare {
namespace ddp {

class EgoPoseManager {
public:
  EgoPoseManager() = default;
  ~EgoPoseManager() = default;

  // feed
  void feed_ego_velocity(double timestamp,
                         const maf_mla_localization::MLAVelocity &velocity);
  void feed_ego_position(double timestamp,
                         const maf_vehicle_status::LocationENU &position,
                         double yaw);
  void feed_ego_cruise_velocity(double cruise_velocity);
  void feed_navi_ttc_gear(uint8_t navi_ttc_gear);

  // get
  bool get_ego_pose(double timestamp, EgoPose &ego_pose);

  double get_ego_cruise_velocity() { return cruise_velocity_; }
  uint8_t get_navi_ttc_gear() { return navi_ttc_gear_; }

private:
  std::deque<EgoPose> ego_position_list; // todo:后续将ego_position_list,
                                         // ego_velocity_list进行合并
  std::deque<EgoPose> ego_velocity_list;

  float cruise_velocity_ = 120.f; // km/h
  uint8_t navi_ttc_gear_ = 2;
};

} // namespace ddp
} // namespace msquare
