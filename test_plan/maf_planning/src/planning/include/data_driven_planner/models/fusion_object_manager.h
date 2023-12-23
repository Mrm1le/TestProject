#pragma once

#include "data_driven_planner/common/basic_types.h"

namespace msquare {
namespace ddp {
using ObjectId = int;
struct FusionObject {
  ObjectId id;
  ObjectType type;
  double timestamp;
  Point3d position;
  Vector3d velocity;
  Shape3d shape;
  double heading_angle = 0.0;
};
using FusionObjectHistory = std::vector<FusionObject>;
using FusionObjectInfo = std::unordered_map<ObjectId, FusionObjectHistory>;

class FusionObjectManager {
public:
  FusionObjectManager();
  ~FusionObjectManager() {}

  // feed
  void feed_fusion_object_info(
      const maf_perception_interface::PerceptionFusionObjectResult
          &fusion_object_result);

  // get
  double get_last_fusion_timestamp() const { return last_timestamp_; }
  bool get_nearest_fusion_object_info(
      const EgoPose &ego_pose, int num,
      const std::vector<double> &timestamp_list,
      FusionObjectInfo *fusion_object_info,
      std::unordered_map<int, std::map<double, std::vector<double>>>
          &fusion_obj_id_ts_info) const;

private:
  FusionObjectInfo fusion_object_info_;
  double last_timestamp_;
};

} // namespace ddp
} // namespace msquare
