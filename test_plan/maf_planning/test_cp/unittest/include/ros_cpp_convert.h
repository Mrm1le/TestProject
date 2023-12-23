#pragma once

#include "convert/convert.h"

// ros topic message type
#include "mla_localization_msgs/MLALocalization.h"
#include "perception_interface_msgs/PerceptionFusionObjectResult.h"


// maf message type
#include "maf_interface/maf_mla_localization.h"
#include "maf_interface/maf_perception_interface.h"

inline std::shared_ptr<maf_perception_interface::PerceptionFusionObjectResult> convertFusionObjects(
    const perception_interface_msgs::PerceptionFusionObjectResult
        &fusion_objects) {
  auto maf_fusion_objects = std::make_shared<
      maf_perception_interface::PerceptionFusionObjectResult>();
  ros_cpp_struct_convert::from_ros(fusion_objects, *maf_fusion_objects);
  return maf_fusion_objects;
}


inline std::shared_ptr<maf_mla_localization::MLALocalization> convertEgoPose(const mla_localization_msgs::MLALocalization &msg) {
  auto ego_pose = std::make_shared<maf_mla_localization::MLALocalization>();
  ros_cpp_struct_convert::from_ros(msg, *ego_pose);
  return ego_pose;
}

inline std::shared_ptr<maf_perception_interface::FusionGroundLineResult> convertFusionGroundlines(
    const perception_interface_msgs::FusionGroundLineResult
        &fusion_groundlines) {

auto maf_fusion_groundlines =
    std::make_shared<maf_perception_interface::FusionGroundLineResult>();
ros_cpp_struct_convert::from_ros(fusion_groundlines,
                                    *maf_fusion_groundlines);
    return maf_fusion_groundlines;
}

inline std::shared_ptr<maf_worldmodel::FusionAPA> convertSlots(
    const worldmodel_msgs::FusionAPA &parking_slots) {

    auto maf_worldmodel_parking_slots =
        std::make_shared<maf_worldmodel::FusionAPA>();
    ros_cpp_struct_convert::from_ros(parking_slots,
                                    *maf_worldmodel_parking_slots);
    return maf_worldmodel_parking_slots;
}