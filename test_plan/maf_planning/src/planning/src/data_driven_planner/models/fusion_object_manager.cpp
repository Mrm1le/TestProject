#include "data_driven_planner/models/fusion_object_manager.h"
#include "common/math/vec2d.h"
#include "data_driven_planner/common/ddp_utils.h"
#include <set>

namespace msquare {
namespace ddp {

inline ObjectType from_perception_fusion_type(
    const maf_perception_interface::ObjectTypeInfo &type_info) {
  ObjectType dst = ObjectType::NOT_KNOW;
  for (auto &property : type_info.type_properties) {
    switch (property.value) {
    case maf_perception_interface::ObjectTypePropertyEnum::
        PROPERTY_OBJECT_TYPE_ENUM: {
      switch (type_info.type.value) {
      case maf_perception_interface::ObjectTypeEnum::
          OBJECT_TYPE_GENERAL_OBJECT: {
        dst = ObjectType::NOT_KNOW;
      } break;
      case maf_perception_interface::ObjectTypeEnum::OBJECT_TYPE_VEHICLE: {
        dst = ObjectType::COUPE;
      } break;
      case maf_perception_interface::ObjectTypeEnum::OBJECT_TYPE_VRU: {
        dst = ObjectType::OFO;
      } break;
      case maf_perception_interface::ObjectTypeEnum::
          OBJECT_TYPE_TRAFFIC_BARRIER: {
        dst = ObjectType::CONE_BUCKET;
      } break;
      default:
        break;
      }
    } break;
    case maf_perception_interface::ObjectTypePropertyEnum::
        PROPERTY_GENERAL_OBJECT_TYPE_ENUM: {
      dst = ObjectType::NOT_KNOW;
      return dst;
    } break;
    case maf_perception_interface::ObjectTypePropertyEnum::
        PROPERTY_VEHICLE_TYPE_ENUM: {
      switch (type_info.vehicle_type.value) {
      case maf_perception_interface::VehicleTypeEnum::VEHICLE_TYPE_UNKNOWN: {
        dst = ObjectType::COUPE;
      } break;
      case maf_perception_interface::VehicleTypeEnum::VEHICLE_TYPE_CAR: {
        dst = ObjectType::COUPE;
      } break;
      case maf_perception_interface::VehicleTypeEnum::VEHICLE_TYPE_TRUCK: {
        dst = ObjectType::TRANSPORT_TRUCK;
      } break;
      case maf_perception_interface::VehicleTypeEnum::VEHICLE_TYPE_BUS: {
        dst = ObjectType::BUS;
      } break;
      default:
        dst = ObjectType::COUPE; // TODO
        break;
      }
      break;
    }
    case maf_perception_interface::ObjectTypePropertyEnum::
        PROPERTY_VEHICLE_SUB_TYPE_ENUM: {
      switch (type_info.vehicle_sub_type.value) {
      case maf_perception_interface::VehicleSubTypeEnum::
          VEHICLE_SUB_TYPE_ENGINEERING_TRUCK: {
        dst = ObjectType::ENGINEER_TRUCK;
        return dst;
      } break;
      default: {
        dst = ObjectType::COUPE; // TODO
        break;
      }
      }
    } break;
    case maf_perception_interface::ObjectTypePropertyEnum::
        PROPERTY_VRU_TYPE_ENUM: {
      switch (type_info.vru_type.value) {
      case maf_perception_interface::VRUTypeEnum::VRU_TYPE_UNKNOWN: {
        dst = ObjectType::OFO;
      } break;
      case maf_perception_interface::VRUTypeEnum::VRU_TYPE_PEDESTRIAN: {
        dst = ObjectType::PEDESTRIAN;
        return dst;
      } break;
      case maf_perception_interface::VRUTypeEnum::VRU_TYPE_BICYCLIST: {
        dst = ObjectType::OFO;
      } break;
      case maf_perception_interface::VRUTypeEnum::VRU_TYPE_TRICYCLIST: {
        dst = ObjectType::OFO;
      } break;
      default: {
        dst = ObjectType::OFO;
      } break;
      }
    } break;
    case maf_perception_interface::ObjectTypePropertyEnum::
        PROPERTY_VRU_SUB_TYPE_ENUM: {
      dst = ObjectType::OFO;
    } break;
    case maf_perception_interface::ObjectTypePropertyEnum::
        PROPERTY_TRAFFIC_BARRIER_TYPE_ENUM: {
      switch (type_info.traffic_barrier_type.value) {
      case maf_perception_interface::TrafficBarrierTypeEnum::
          TRAFFIC_BARRIER_TYPE_CONE: {
        dst = ObjectType::CONE_BUCKET;
      } break;
      default:
        break;
      }
    }
    default:
      break;
    }
  }
  return dst;
}

FusionObjectManager::FusionObjectManager() {}

void FusionObjectManager::feed_fusion_object_info(
    const maf_perception_interface::PerceptionFusionObjectResult
        &fusion_object_result) {
  double timestamp = fusion_object_result.meta.sensor_timestamp_us / 1e6;
  last_timestamp_ = timestamp;

  // Step 1) delete eliminated object
  std::set<ObjectId> track_ids;
  for (auto &fusion_object_data :
       fusion_object_result.perception_fusion_objects_data) {
    track_ids.insert(fusion_object_data.track_id);
  }

  for (auto it = fusion_object_info_.begin();
       it != fusion_object_info_.end();) {
    if (track_ids.find(it->first) == track_ids.end()) {
      it = fusion_object_info_.erase(it);
    } else {
      ++it;
    }
  }

  // Step 2) update object info
  for (auto &perception_object :
       fusion_object_result.perception_fusion_objects_data) {
    auto track_id = perception_object.track_id;

    FusionObject fusion_object;
    fusion_object.id = track_id;
    fusion_object.type =
        from_perception_fusion_type(perception_object.type_info);
    fusion_object.position =
        Point3d{perception_object.position.x, perception_object.position.y,
                perception_object.position.z};
    fusion_object.velocity =
        Vector3d{perception_object.velocity.x, perception_object.velocity.y,
                 perception_object.velocity.z};
    fusion_object.heading_angle = perception_object.heading_yaw;
    fusion_object.shape =
        Shape3d{perception_object.shape.length, perception_object.shape.width,
                perception_object.shape.height};
    fusion_object.timestamp = timestamp;

    auto it = fusion_object_info_.find(track_id);
    if (it == fusion_object_info_.end()) {
      fusion_object_info_.insert(
          std::make_pair(track_id, FusionObjectHistory{{fusion_object}}));
    } else {
      it->second.push_back(fusion_object);
      if (it->second.size() > 50) {
        it->second.erase(it->second.begin());
      }
    }
  }
}

bool FusionObjectManager::get_nearest_fusion_object_info(
    const EgoPose &ego_pose, int num, const std::vector<double> &timestamp_list,
    FusionObjectInfo *res_fusion_object_info,
    std::unordered_map<int, std::map<double, std::vector<double>>>
        &fusion_obj_id_ts_info) const {
  res_fusion_object_info->clear();

  fusion_obj_id_ts_info.clear();

  // object sort by distance
  std::vector<std::pair<ObjectId, double>> object_distances;
  for (auto &it : fusion_object_info_) {
    auto object_id = it.first;
    auto &least_object = it.second.back();
    auto delta =
        planning_math::Vec2d(least_object.position.x - ego_pose.position.x,
                             least_object.position.y - ego_pose.position.y);
    object_distances.emplace_back(object_id, delta.Length());
  }
  std::sort(
      object_distances.begin(), object_distances.end(),
      [](const std::pair<int, double> &p1, const std::pair<int, double> &p2) {
        return p1.second < p2.second or
               (p1.second == p2.second and p1.first < p2.first);
      });

  for (auto it = object_distances.begin(); it != object_distances.end(); ++it) {
    auto &fusion_object_history = fusion_object_info_.find(it->first)->second;
    FusionObjectHistory interpolated_history;
    std::map<double, std::vector<double>> fusion_obj_ts_info{};
    for (auto timestamp : timestamp_list) {
      if (timestamp < fusion_object_history.front().timestamp) {
        break;
      }
      std::vector<double> fusion_obj_ts_list{};

      for (size_t i = 0; i < fusion_object_history.size() - 1; ++i) {
        auto &pre_object = fusion_object_history[i];
        auto &next_object = fusion_object_history[i + 1];

        if (pre_object.timestamp <= timestamp and
            timestamp <= next_object.timestamp) {
          fusion_obj_ts_list.emplace_back(pre_object.timestamp);
          fusion_obj_ts_list.emplace_back(next_object.timestamp);
          FusionObject interpolated_object = pre_object;
          interpolated_object.timestamp = timestamp;
          interpolated_object.position.x = interpolate(
              pre_object.timestamp, pre_object.position.x,
              next_object.timestamp, next_object.position.x, timestamp);
          interpolated_object.position.y = interpolate(
              pre_object.timestamp, pre_object.position.y,
              next_object.timestamp, next_object.position.y, timestamp);
          interpolated_object.position.z = interpolate(
              pre_object.timestamp, pre_object.position.z,
              next_object.timestamp, next_object.position.z, timestamp);
          interpolated_object.velocity.x = interpolate(
              pre_object.timestamp, pre_object.velocity.x,
              next_object.timestamp, next_object.velocity.x, timestamp);
          interpolated_object.velocity.y = interpolate(
              pre_object.timestamp, pre_object.velocity.y,
              next_object.timestamp, next_object.velocity.y, timestamp);
          interpolated_object.velocity.z = interpolate(
              pre_object.timestamp, pre_object.velocity.z,
              next_object.timestamp, next_object.velocity.z, timestamp);
          interpolated_object.shape.length = interpolate(
              pre_object.timestamp, pre_object.shape.length,
              next_object.timestamp, next_object.shape.length, timestamp);
          interpolated_object.shape.width = interpolate(
              pre_object.timestamp, pre_object.shape.width,
              next_object.timestamp, next_object.shape.width, timestamp);
          interpolated_object.shape.height = interpolate(
              pre_object.timestamp, pre_object.shape.height,
              next_object.timestamp, next_object.shape.height, timestamp);
          interpolated_object.heading_angle = interpolate_angle(
              pre_object.timestamp, pre_object.heading_angle,
              next_object.timestamp, next_object.heading_angle, timestamp);
          interpolated_history.push_back(interpolated_object);
          break;
        }
      }
      fusion_obj_ts_info.insert(std::make_pair(timestamp, fusion_obj_ts_list));
    }
    fusion_obj_id_ts_info.insert(std::make_pair(it->first, fusion_obj_ts_info));
    if (not interpolated_history.empty()) {
      res_fusion_object_info->insert(
          std::make_pair(it->first, interpolated_history));

      if (static_cast<int>(res_fusion_object_info->size()) >= num) {
        break;
      }
    }
  }

  return true;
}

} // namespace ddp
} // namespace msquare