#include "data_driven_planner/models/ego_pose_manager.h"
#include "common/planning_fail_tracer.h"
#include "data_driven_planner/common/ddp_utils.h"
// #include "data_driven_planner/models/hdmap_adapter.h"

namespace msquare {
namespace ddp {

void EgoPoseManager::feed_ego_cruise_velocity(double cruise_velocity) {
  cruise_velocity_ = cruise_velocity;
}

void EgoPoseManager::feed_navi_ttc_gear(uint8_t navi_ttc_gear) {
  navi_ttc_gear_ = navi_ttc_gear;
}

void EgoPoseManager::feed_ego_velocity(
    double timestamp, const maf_mla_localization::MLAVelocity &velocity) {
  EgoPose vel_pose;
  vel_pose.timestamp = timestamp;
  vel_pose.velocity =
      Vector3d(velocity.velocity_local.vx, velocity.velocity_local.vy,
               velocity.velocity_local.vz);
  ego_velocity_list.push_back(vel_pose);

  if (ego_velocity_list.size() > 200) {
    ego_velocity_list.pop_front();
  }
}

void EgoPoseManager::feed_ego_position(
    double timestamp, const maf_vehicle_status::LocationENU &position,
    double yaw) {
  EgoPose position_pose;
  position_pose.timestamp = timestamp;
  position_pose.position = Point3d(position.x, position.y, position.z);
  // convert from head to center
  position_pose.position.x -=
      std::cos(yaw) *
      ConfigurationContext::Instance()->get_vehicle_param().length / 2.0;
  position_pose.position.y -=
      std::sin(yaw) *
      ConfigurationContext::Instance()->get_vehicle_param().length / 2.0;
  position_pose.heading_angle = yaw;
  ego_position_list.push_back(position_pose);

  if (ego_position_list.size() > 200) {
    ego_position_list.pop_front();
  }
}

bool EgoPoseManager::get_ego_pose(double timestamp, EgoPose &ego_pose) {
  if (ego_position_list.empty()) {
    PLANNING_FAIL_TRACE("ego_position_list is empty");
    MSD_LOG(ERROR, "get_ego_pose: ego_position_list is empty!");
    return false;
  }
  if (ego_velocity_list.empty()) {
    PLANNING_FAIL_TRACE("ego_velocity_list is empty");
    MSD_LOG(ERROR, "get_ego_pose: ego_velocity_list is empty!");
    return false;
  }

  if (timestamp < ego_position_list.front().timestamp ||
      timestamp > ego_position_list.back().timestamp) {
    PLANNING_FAIL_TRACE("ego_position_list timestamp check failed");
    MSD_LOG(ERROR,
            "get_ego_pose: ego_position_list timestamp check failed, "
            "timestamp: %f,  front timestamp: %f, end timestamp: %f, list "
            "size: %d !",
            timestamp, ego_position_list.front().timestamp,
            ego_position_list.back().timestamp, ego_position_list.size());
    return false;
  }

  if (timestamp < ego_velocity_list.front().timestamp ||
      timestamp > ego_velocity_list.back().timestamp) {
    PLANNING_FAIL_TRACE("ego_velocity_list timestamp check failed");
    MSD_LOG(ERROR,
            "get_ego_pose: ego_velocity_list timestamp check failed, "
            "timestamp: %f,  front timestamp: %f, end timestamp: %f, list "
            "size: %d !",
            timestamp, ego_velocity_list.front().timestamp,
            ego_velocity_list.back().timestamp, ego_velocity_list.size());
    return false;
  }

  for (size_t i = 0; i < ego_position_list.size() - 1; i++) {
    auto &pre_pose = ego_position_list[i];
    auto &next_pose = ego_position_list[i + 1];
    if (pre_pose.timestamp <= timestamp and timestamp <= next_pose.timestamp) {
      ego_pose.timestamp = timestamp;
      ego_pose.position.x =
          interpolate(pre_pose.timestamp, pre_pose.position.x,
                      next_pose.timestamp, next_pose.position.x, timestamp);
      ego_pose.position.y =
          interpolate(pre_pose.timestamp, pre_pose.position.y,
                      next_pose.timestamp, next_pose.position.y, timestamp);
      ego_pose.position.z =
          interpolate(pre_pose.timestamp, pre_pose.position.z,
                      next_pose.timestamp, next_pose.position.z, timestamp);
      ego_pose.heading_angle = interpolate_angle(
          pre_pose.timestamp, pre_pose.heading_angle, next_pose.timestamp,
          next_pose.heading_angle, timestamp);
      break;
    }
  }

  for (size_t i = 0; i < ego_velocity_list.size() - 1; i++) {
    auto &pre_pose = ego_velocity_list[i];
    auto &next_pose = ego_velocity_list[i + 1];
    if (pre_pose.timestamp <= timestamp and timestamp <= next_pose.timestamp) {
      ego_pose.velocity.x =
          interpolate(pre_pose.timestamp, pre_pose.velocity.x,
                      next_pose.timestamp, next_pose.velocity.x, timestamp);
      ego_pose.velocity.y =
          interpolate(pre_pose.timestamp, pre_pose.velocity.y,
                      next_pose.timestamp, next_pose.velocity.y, timestamp);
      ego_pose.velocity.z = 0;
      break;
    }
  }

  ego_pose.v = std::hypot( // parasoft-suppress AUTOSAR-A8_5_0 "f-drop"
      ego_pose.velocity.x, ego_pose.velocity.y);
  ego_pose.a = 0.0;

  return true;
}

} // namespace ddp
} // namespace msquare
