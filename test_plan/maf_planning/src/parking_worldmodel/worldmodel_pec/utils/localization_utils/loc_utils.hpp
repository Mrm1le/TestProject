#pragma once

#include "maf_interface/maf_perception_interface.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace putils {

class LocUtils {

public:
  static bool isValid(const maf_mla_localization::MLALocalization &loc) {
    // TODO: this is only a temporary usage;
    return loc.status.status_info.type != 0;
  }

  static bool isHighAccuracy() {
    // TODO: this is only a temporary usage;
    return true;
  }

  static bool isDeadReckoning() {
    // TODO: this is only a temporary usage;
    return false;
  }

  static Eigen::Isometry3d
  poseFromMLA(const maf_mla_localization::MLALocalization &loc) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    if (!isValid(loc)) {
      // printf("[WARNING](LocUtils::poseFromMLA) trying to use a invalid MLA "
      //        "pose, return identity \n");
      return pose;
    }

    pose.translation().x() = loc.position.position_local.x;
    pose.translation().y() = loc.position.position_local.y;
    pose.translation().z() = loc.position.position_local.z;

    Eigen::Quaterniond quat;
    quat.w() = loc.orientation.quaternion_local.w;
    quat.x() = loc.orientation.quaternion_local.x;
    quat.y() = loc.orientation.quaternion_local.y;
    quat.z() = loc.orientation.quaternion_local.z;

    pose.linear() = quat.toRotationMatrix();

    return pose;
  }

private:
  LocUtils() = delete;
  virtual ~LocUtils();
};

} // namespace putils
