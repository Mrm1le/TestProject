#include "maf_interface/maf_worldmodel.h"
#include "mjson/mjson.hpp"
#include "worldmodel/common.h"
#include "worldmodel/fusion_filter_interface.hpp"
#include "worldmodel/utility.hpp"
#include <cmath>

namespace msd_worldmodel {
namespace worldmodel_v1 {

using namespace maf_worldmodel;

class FusionFilterImpl : public FusionFilter {
public:
  explicit FusionFilterImpl() {

    coord_transformer_ = CoordinateTransformer::make();
  }

  void filter(std::vector<maf_worldmodel::ObjectInterface> &fusion_objects,
              const ProcessedMapPtr &processed_map_ptr,
              const MLALocalizationPtr &localization_ptr,
              bool is_ddmap) override {

    auto &lanes = processed_map_ptr->processed_map_data.lanes;
    int cur_lane_index = INVALID_INDEX;
    for (size_t i = 0; i < lanes.size(); i++) {
      if (lanes[i].relative_id == 0) {
        cur_lane_index = i;
        break;
      }
    }
    if (cur_lane_index == INVALID_INDEX) {
      return;
    }
    if (!lanes[cur_lane_index].left_lane_boundary.existence) {
      return;
    }

    auto &cur_reference_line_points =
        lanes[cur_lane_index].reference_line.reference_line_points;
    update_ct(localization_ptr);

    std::vector<maf_perception_interface::Point3f> car_positions{};
    car_positions.reserve(cur_reference_line_points.size());

    for (auto &reference_point : cur_reference_line_points) {
      auto &enu_position = reference_point.enu_point;
      auto car_position = coord_transformer_->ENUToCar(
          {enu_position.x, enu_position.y, enu_position.z});
      maf_perception_interface::Point3f tmp_position = {
          .x = float(car_position.x()),
          .y = float(car_position.y()),
          .z = float(car_position.z())};

      car_positions.emplace_back(std::move(tmp_position));
    }

    for (auto &fusion_ob : fusion_objects) {
      // MSD_LOG(DEBUG, "track_id: %d, car_position: x: %.2f y: %.2f ",
      //         fusion_ob.object_fusion_data.track_id,
      //         fusion_ob.object_fusion_data.relative_position.x,
      //         fusion_ob.object_fusion_data.relative_position.y);

      if (fusion_ob.is_ignore == false) {
        bool is_ignore = line_filter(cur_reference_line_points, car_positions,
                                     fusion_ob.object_fusion_data, is_ddmap);

        fusion_ob.is_ignore = is_ignore;
      }

      // TODO : yaw filter
      // if (fusion_ob.shape.length > 6.0) {
      //   yaw_filter(ref_line_points[index], fusion_ob);
      // }
    }
  }

private:
  void update_ct(const MLALocalizationPtr &localization_ptr) {
    auto &local_position = localization_ptr->position.position_local;
    auto &local_euler = localization_ptr->orientation.euler_local;
    coord_transformer_->setENUOrigin(
        {local_position.x, local_position.y, local_position.z},
        local_euler.roll, local_euler.pitch, local_euler.yaw);
  }

  float cal_dis(const maf_perception_interface::Point3d &p1,
                const maf_perception_interface::Point3f &p2) {
    float x1 = float(p1.x);
    float y1 = float(p1.y);

    float x2 = p2.x;
    float y2 = p2.y;
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  }

  int find_nearest_point(
      const std::vector<maf_perception_interface::Point3f> &car_positions,
      const maf_perception_interface::PerceptionFusionObjectData &fusion_ob) {

    auto &target_point = fusion_ob.relative_position;
    double nearest_dis = 10000.0;
    size_t nearest_index = INVALID_INDEX;

    if (car_positions.size() == 0) {
      return INVALID_INDEX;
    }

    for (size_t i = 1; i < car_positions.size() - 1; i++) {
      auto dis = cal_dis(target_point, car_positions[i]);
      // MSD_LOG(DEBUG, "i: %d, target_position: x: %.2f , y: %.2f ; point x:
      // %.2f; y: %.2f dis: %.2f",
      //   i, target_point.x, target_point.y, car_positions[i].x,
      //   car_positions[i].y, dis);

      if (dis < nearest_dis) {
        nearest_dis = dis;
        nearest_index = i;
      }
    }
    return nearest_index;
  }

  double cross_product(double u1, double v1, double u2, double v2) {
    return u1 * v2 - u2 * v1;
  }

  std::vector<std::vector<double>> get_corner(
      const maf_perception_interface::PerceptionFusionObjectData &fusion_ob) {
    std::vector<std::vector<double>> res;
    res.reserve(4);
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        std::vector<double> point;
        point.reserve(2);
        double x = std::pow(-1, i) * (fusion_ob.shape.length / 2);
        double y = std::pow(-1, j) * (fusion_ob.shape.width / 2);
        double yaw = fusion_ob.relative_heading_yaw;
        double x_car = x * std::cos(yaw) - y * std::sin(yaw) +
                       fusion_ob.relative_position.x;
        double y_car = x * std::sin(yaw) + y * std::cos(yaw) +
                       fusion_ob.relative_position.y;
        point.push_back(x_car);
        point.push_back(y_car);
        res.push_back(std::move(point));
      }
    }
    return res;
  }

  bool line_filter(
      const std::vector<ReferenceLinePoint> &ref_lines,
      const std::vector<maf_perception_interface::Point3f> &car_positions,
      maf_perception_interface::PerceptionFusionObjectData &fusion_ob,
      bool is_ddmap) {

    auto id = find_nearest_point(car_positions, fusion_ob);
    if (id == INVALID_INDEX) {
      return false;
    }

    double LPoint_y1{};
    double LPoint_y2{};

    if (is_ddmap) {
      // if (fusion_ob.relative_position.x > DDMAP_IGNORE_X_DISTANCE) {
      //   return true;
      // }

      LPoint_y1 = ref_lines[id - 1].distance_to_left_lane_border;
      LPoint_y2 = ref_lines[id + 1].distance_to_left_lane_border;
    } else {
      LPoint_y1 = ref_lines[id - 1].distance_to_left_road_border;
      LPoint_y2 = ref_lines[id + 1].distance_to_left_road_border;
    }

    double x = car_positions[id + 1].x - car_positions[id - 1].x;
    double y = car_positions[id + 1].y - car_positions[id - 1].y;

    double yaw = std::atan2(y, x);
    double fusion_car_x = fusion_ob.relative_position.x;
    // double fusion_car_y = fusion_ob.relative_position.y;
    double left = 1.0;
    // double right = -1.0;
    std::vector<std::vector<double>> fusion_ob_corner = get_corner(fusion_ob);

    if (!std::isinf(LPoint_y1) && !std::isinf(LPoint_y2) &&
        LPoint_y1 < VALID_DISTANCE && LPoint_y2 < VALID_DISTANCE) {
      double LPoint_car_x1 =
          -LPoint_y1 * std::sin(yaw) + car_positions[id - 1].x;
      double LPoint_car_y1 =
          LPoint_y1 * std::cos(yaw) + car_positions[id - 1].y;

      double LPoint_car_x2 =
          -LPoint_y2 * std::sin(yaw) + car_positions[id + 1].x;
      double LPoint_car_y2 =
          LPoint_y2 * std::cos(yaw) + car_positions[id + 1].y;

      left = -1;
      for (size_t i = 0; i < fusion_ob_corner.size(); i++) {
        double fusion_ob_car_x = fusion_ob_corner[i][0];
        double fusion_ob_car_y = fusion_ob_corner[i][1];

        if (cross_product((fusion_ob_car_x - LPoint_car_x1),
                          (fusion_ob_car_y - LPoint_car_y1),
                          (LPoint_car_x2 - LPoint_car_x1),
                          (LPoint_car_y2 - LPoint_car_y1)) > 0) {

          left = 1;
        }
      }
    }

    if (left < 0) {
      if (is_ddmap) {
        double car_abs_vx = fusion_ob.velocity_relative_to_ground.x;
        if (car_abs_vx < DDMAP_IGNORE_VX) {
          return true;
        }
      } else {
        if (fusion_car_x > IGNORE_X_DISTANCE) {
          return true;
        }
      }
    }
    return false;
  }

private:
  static constexpr auto INVALID_INDEX = -1;
  static constexpr auto DDMAP_IGNORE_VX = -3.0;
  static constexpr auto IGNORE_X_DISTANCE = 20;
  static constexpr auto VALID_DISTANCE = 100.0;

  std::shared_ptr<CoordinateTransformer> coord_transformer_{};
};

std::shared_ptr<FusionFilter> FusionFilter::make() {
  return std::make_shared<FusionFilterImpl>();
}

} // namespace worldmodel_v1
} // namespace msd_worldmodel
