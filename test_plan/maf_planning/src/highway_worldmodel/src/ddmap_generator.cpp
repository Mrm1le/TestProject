#include "maf_interface/maf_worldmodel.h"
#include "worldmodel/common.h"
#include "worldmodel/ddmap_generator_interface.hpp"
#include <atomic>
namespace msd_worldmodel {
namespace worldmodel_v1 {
namespace {

uint8_t convertLaneColor(const uint8_t &lane_color) {
  switch (lane_color) {
  case maf_perception_interface::LaneColorEnum::LANE_COLOR_WHITE:
    return maf_worldmodel::LaneBoundaryColor::WHITE;
  case maf_perception_interface::LaneColorEnum::LANE_COLOR_YELLOW:
    return maf_worldmodel::LaneBoundaryColor::YELLOW;
  default:
    return maf_worldmodel::LaneBoundaryColor::OTHER;
  }
}

uint8_t convertLaneType(const uint8_t &lane_type) {
  switch (lane_type) {
  case maf_perception_interface::LaneTypeEnum::LANE_TYPE_DASHED:
    return maf_worldmodel::LaneBoundaryForm::DASH;
  case maf_perception_interface::LaneTypeEnum::LANE_TYPE_SOLID:
    return maf_worldmodel::LaneBoundaryForm::SOLID;
  case maf_perception_interface::LaneTypeEnum::LANE_TYPE_DOUBLE_SOLID:
    return maf_worldmodel::LaneBoundaryForm::DOUBLE_SOLID;
  case maf_perception_interface::LaneTypeEnum::LANE_TYPE_DOUBLE_DASHED_SOLID:
    return maf_worldmodel::LaneBoundaryForm::DASH_SOLID;
  case maf_perception_interface::LaneTypeEnum::LANE_TYPE_UNKNOWN:
    return maf_worldmodel::LaneBoundaryForm::UNKNOWN;
  default:
    return maf_worldmodel::LaneBoundaryForm::UNKNOWN;
  }
}
} // namespace

bool line_on_ref_left(const maf_perception_interface::Lane &line,
                      const maf_perception_interface::Lane &ref_line) {
  size_t count_on_left = 0;
  size_t line_size = std::min(line.points_3d_x.size(), line.points_3d_y.size());
  size_t ref_line_size =
      std::min(ref_line.points_3d_x.size(), ref_line.points_3d_y.size());
  // add overlap size check
  size_t overlap_size = 0;
  if (line_size == 0 || ref_line_size == 0) {
    return false;
  }
  for (size_t i = 0; i < line_size; ++i) {
    if (line.points_3d_x[i] < ref_line.points_3d_x.front() ||
        line.points_3d_x[i] > ref_line.points_3d_x.back()) {
      continue;
    }
    ++overlap_size;
    float min_dist = std::numeric_limits<float>::max();
    size_t j = 0;
    while (j < ref_line_size) {
      float dist =
          std::sqrt(std::pow(line.points_3d_x[i] - ref_line.points_3d_x[j], 2) +
                    std::pow(line.points_3d_y[i] - ref_line.points_3d_y[j], 2));
      if (dist < min_dist) {
        ++j;
        min_dist = dist;
      } else {
        break;
      }
    }
    --j;
    if (line.points_3d_y[i] > ref_line.points_3d_y[j]) {
      count_on_left += 1;
    }
  }

  // no overlap, check nearest point roughly
  if (overlap_size == 0) {
    if (line.points_3d_x.back() < ref_line.points_3d_x.front()) {
      return line.points_3d_y.back() > ref_line.points_3d_y.front();
    } else {
      return line.points_3d_y.front() > ref_line.points_3d_y.back();
    }
  }

  return count_on_left >= 0.5 * overlap_size;
}

bool update_center_lane_and_find_current_index(
    const maf_perception_interface::LanePerception &lane_perception,
    std::vector<InterCenterLine> &center_lines,
    IntersectionPoint &intersection_point) {
  auto &lanes = lane_perception.lanes;
  auto &reserved_infos = lane_perception.reserved_infos;

  center_lines.clear();

  std::unordered_map<std::string, int> trackid_index_map{};

  // preprocess the lane
  for (size_t i = 0; i < lanes.size(); i++) {
    auto &lane = lanes[i];
    if (lane.camera_source.value != maf_perception_interface::CameraSourceEnum::
                                        CAMERA_SOURCE_FRONT_MID ||
        lane.is_failed_3d) {
      continue;
    }

    if (lane.is_centerline) { // center_line
      InterCenterLine center_line{};
      center_line.center_line_index = i;
      center_line.left_lane_index = INVALID_INDEX;
      center_line.right_lane_index = INVALID_INDEX;
      center_lines.push_back(std::move(center_line));
    } else if (lane.index == 0 && lane.points_3d_x.size() == 1 &&
               lane.points_3d_y.size() == 1 &&
               lane.points_3d_z.size() == 1) { // intersection point
      if (reserved_infos.size() > i) {
        std::string err{};
        auto intersection_point_json =
            mjson::Json::parse(reserved_infos[i], err);
        if (err.empty() &&
            intersection_point_json.has_key(KEY_IS_INTERSECTION_POINT)) {
          if (intersection_point_json[KEY_IS_INTERSECTION_POINT].bool_value() ==
              true) {
            intersection_point.is_valid = true;
            intersection_point.intersection_point_car.x = lane.points_3d_x[0];
            intersection_point.intersection_point_car.y = lane.points_3d_y[0];
            intersection_point.intersection_point_car.z = lane.points_3d_z[0];
          }
        }
      }
    } else { // lane

      std::string track_id = std::to_string(lane.track_id);
      trackid_index_map[track_id] = i;
    }
  }

  bool all_lane_is_valid = true;
  for (size_t i = 0; i < center_lines.size(); i++) {
    auto &center_line = center_lines[i];
    auto center_line_index = center_line.center_line_index;
    bool both_lane_is_valid = false;
    if ((int)reserved_infos.size() > center_line_index) {
      auto &reserved_info = reserved_infos[center_line_index];
      std::string err{};
      auto center_line_reserved_info_json =
          mjson::Json::parse(reserved_info, err);
      if (err.empty() &&
          center_line_reserved_info_json.has_key(KEY_LEFT_LANE) &&
          center_line_reserved_info_json.has_key(KEY_RIGHT_LANE)) {
        auto track_id_left_lane =
            center_line_reserved_info_json[KEY_LEFT_LANE].string_value();
        auto track_id_right_lane =
            center_line_reserved_info_json[KEY_RIGHT_LANE].string_value();
        bool left_is_valid = false;
        if (trackid_index_map.count(track_id_left_lane) > 0) {
          center_line.left_lane_index = trackid_index_map[track_id_left_lane];
          left_is_valid = true;
        }
        bool right_is_valid = false;
        if (trackid_index_map.count(track_id_right_lane) > 0) {
          center_line.right_lane_index = trackid_index_map[track_id_right_lane];
          right_is_valid = true;
        }
        if (left_is_valid && right_is_valid) {
          both_lane_is_valid = true;
        }
      }
    }

    if (!both_lane_is_valid) {
      all_lane_is_valid = false;
      MSD_LOG(WARN, "the center line index %d is not valid, the track_id: %ld",
              center_line_index, lanes[center_line_index].track_id);
      if ((int)reserved_infos.size() > center_line_index) {
        MSD_LOG(WARN, "the reserved_info is: %s ",
                reserved_infos[center_line_index].c_str());
      }
      break;
    }
  }

  if (!all_lane_is_valid) {
    center_lines.clear();
    MSD_LOG(WARN, "not all center line has left and right lane");
    return false;
  }

  const float lane_sorting_x = 5.0;
  std::sort(center_lines.begin(), center_lines.end(),
            [&lanes](const InterCenterLine &center_line1,
                     const InterCenterLine &center_line2) {
              const auto &lane1 = lanes[center_line1.center_line_index];
              const auto &lane2 = lanes[center_line2.center_line_index];

              return line_on_ref_left(lane1, lane2);
            });

  int cur_center_line_index = INVALID_INDEX;
  for (size_t i = 0; i < center_lines.size(); i++) {
    auto &center_line = center_lines[i];

    auto &left_lane = lanes[center_line.left_lane_index];
    auto &right_lane = lanes[center_line.right_lane_index];

    const float max_lane_sorting_x = 20.0;
    float left_lane_sorting_x = lane_sorting_x;
    if (left_lane.points_3d_x.size() > 0) {
      left_lane_sorting_x =
          std::min(std::max(left_lane_sorting_x, left_lane.points_3d_x[0]),
                   max_lane_sorting_x);
    }
    auto lane_left_y =
        getPolyLaneYCoordinate(left_lane.coefficient_bv, left_lane_sorting_x);

    float right_lane_sorting_x = lane_sorting_x;
    if (right_lane.points_3d_x.size() > 0) {
      right_lane_sorting_x =
          std::min(std::max(right_lane_sorting_x, right_lane.points_3d_x[0]),
                   max_lane_sorting_x);
    }
    auto lane_right_y =
        getPolyLaneYCoordinate(right_lane.coefficient_bv, right_lane_sorting_x);
    if ((lane_left_y * lane_right_y) < 0) {
      cur_center_line_index = i;
      break;
    }
  }

  // check nearest center lane if not found
  const double lateral_thres = 2.2;
  double min_center_lane_y = 1000.0;
  double min_center_lane_index = INVALID_INDEX;
  if (cur_center_line_index == INVALID_INDEX) {
    for (size_t i = 0; i < center_lines.size(); i++) {
      auto &center_lane = lanes[center_lines[i].center_line_index];
      auto center_lane_y =
          getPolyLaneYCoordinate(center_lane.coefficient_bv, lane_sorting_x);
      // ignore lane not covered
      if (center_lane.points_3d_x.size() < 3 ||
          center_lane.points_3d_x.front() > -5.0 ||
          center_lane.points_3d_x.back() < 5.0) {
        continue;
      }
      if (std::fabs(center_lane_y) < min_center_lane_y) {
        min_center_lane_y = std::fabs(center_lane_y);
        min_center_lane_index = i;
      }
    }
    if (int(min_center_lane_index) != int(INVALID_INDEX) &&
        min_center_lane_y < lateral_thres) {
      cur_center_line_index = min_center_lane_index;
    }
  }

  if (int(cur_center_line_index) == int(INVALID_INDEX)) {
    center_lines.clear();
    MSD_LOG(WARN, "not found cur center line!!");
    return false;
  }

  for (size_t i = 0; i < center_lines.size(); i++) {
    center_lines[i].relative_id = i - cur_center_line_index;
    center_lines[i].index = center_lines.size() - i - 1;
  }

  return true;
}

class DdmapGeneratorImpl : public DdmapGenerator {
public:
  explicit DdmapGeneratorImpl() {
    coord_transformer_ = CoordinateTransformer::make();
  }

  void updateLocation(MLALocalizationPtr localization_ptr) override {
    localization_buffer_.emplace_back(localization_ptr);
    if (localization_buffer_.size() > MAX_LOCLIZATION_BUFFER_SIZE) {
      localization_buffer_.pop_front();
    }
  }

  std::shared_ptr<CoordinateTransformer> getCoordTransformer() override {
    return coord_transformer_;
  }

  bool processLanePtr(const PerceptionLanePtr lane_ptr,
                      ProcessedMapPtr &processed_map) override {
    if (reset_.load()) {
      localization_buffer_.clear();
      reset_.store(false);
    }
    if (!getNearestLocalization(lane_ptr->meta.sensor_timestamp_us,
                                localization_ptr_)) {
      MSD_LOG(INFO, "ddmap doesn't found the valid egopose , please check "
                    "the egopose!!!");
      return false;
    }
    MSD_LOG(DEBUG, "the lane sensor time: %ju",
            lane_ptr->meta.sensor_timestamp_us);

    auto &local_position = localization_ptr_->position.position_local;
    auto &local_euler = localization_ptr_->orientation.euler_local;
    coord_transformer_->setENUOrigin(
        {local_position.x, local_position.y, local_position.z},
        local_euler.roll, local_euler.pitch, local_euler.yaw);

    preProcessLanePtr(lane_ptr);
    convertLane(lane_ptr, processed_map);

    return true;
  }

  void convertTimeStamp(ProcessedMapPtr &processed_map) override {
    processed_map->meta.egopose_timestamp_us =
        localization_ptr_->meta.timestamp_us;
  }

  void convertOther(ProcessedMapPtr &processed_map) override {
    processed_map->processed_map_data.available |=
        maf_worldmodel::ProcessedMapData::MAP_POI_INFO;
    processed_map->processed_map_data.available |=
        maf_worldmodel::ProcessedMapData::INTERSECTION;
    processed_map->processed_map_data.available |=
        maf_worldmodel::ProcessedMapData::LANE_STRATEGY;
    processed_map->processed_map_data.available |=
        maf_worldmodel::ProcessedMapData::LANE_MERGING_SPLITTING_POINT;
  }

  void convertSelfPosition(ProcessedMapPtr &processed_map) override {
    processed_map->processed_map_data.available |=
        maf_worldmodel::ProcessedMapData::SELF_POSITION;
    processed_map->processed_map_data.self_position.in_intersection = false;
    processed_map->processed_map_data.self_position.in_map_area = true;
    processed_map->processed_map_data.self_position.on_ramp = false;
  }

  void convertExtraInfo(ProcessedMapPtr &processed_map) override {
    if (intersection_point_.is_valid) {
      processed_map->processed_map_data.available |=
          maf_worldmodel::ProcessedMapData::EXTRA_INFO;
      processed_map->processed_map_data.extra_info.available |=
          maf_worldmodel::ExtraInfo::RESERVED_INFO;

      mjson::Json intersection_point_json{};
      auto &intersection_point_object_value =
          intersection_point_json.object_value();

      mjson::Json::object intersection_point_car{};
      intersection_point_car["x"] =
          intersection_point_.intersection_point_car.x;
      intersection_point_car["y"] =
          intersection_point_.intersection_point_car.y;
      intersection_point_car["z"] =
          intersection_point_.intersection_point_car.z;
      intersection_point_object_value["intersection_point_car"] =
          mjson::Json(std::move(intersection_point_car));

      auto point_enu = coord_transformer_->CarToENU(
          {intersection_point_.intersection_point_car.x,
           intersection_point_.intersection_point_car.y,
           intersection_point_.intersection_point_car.z});

      mjson::Json::object intersection_point_enu{};
      intersection_point_enu["x"] = point_enu.x();
      intersection_point_enu["y"] = point_enu.y();
      intersection_point_enu["z"] = point_enu.z();
      intersection_point_object_value["intersection_point_enu"] =
          mjson::Json(std::move(intersection_point_enu));

      processed_map->processed_map_data.extra_info.reserved_info =
          intersection_point_json.dump();
    }
  }

  void convertLane(const PerceptionLanePtr perception_lane_ptr,
                   ProcessedMapPtr &processed_map) override {

    processed_map->processed_map_data.available |=
        maf_worldmodel::ProcessedMapData::LANE;
    int lane_num = center_lines_.size();

    auto &perception_lanes = perception_lane_ptr->lane_perception.lanes;
    auto &perception_road_edges =
        perception_lane_ptr->road_edge_perception.road_edges;

    processed_map->processed_map_data.lanes.resize(lane_num);
    auto &output_lanes = processed_map->processed_map_data.lanes;
    for (int i = 0; i < lane_num; i++) {
      auto &inner_center_line = center_lines_[i];
      auto &output_lane = output_lanes[i];

      // MSD_LOG(DEBUG, "relative_id: %d ", inner_center_line.relative_id);

      output_lane.relative_id = inner_center_line.relative_id;
      output_lane.track_id = inner_center_line.index;
      output_lane.lane_marks.value = maf_worldmodel::Direction::GO_STRAIGHT;
      output_lane.lane_type.value = maf_worldmodel::LaneType::NORMAL;

      output_lane.reference_line.available = true;
      auto &center_lane = perception_lanes[inner_center_line.center_line_index];
      center_line_points_3d_x_ = center_lane.points_3d_x;
      center_line_points_3d_y_ = center_lane.points_3d_y;
      center_line_points_size_ = center_line_points_3d_x_.size();
      output_lane.reference_line.reference_line_points.resize(
          center_line_points_size_);

      auto &left_lane_index = inner_center_line.left_lane_index;
      auto &right_lane_index = inner_center_line.right_lane_index;

      for (int j = 0; j < center_line_points_size_; j++) {

        auto &reference_line_point =
            output_lane.reference_line.reference_line_points[j];

        auto &center_line_point_3d_x = center_line_points_3d_x_[j];
        auto &center_line_point_3d_y = center_line_points_3d_y_[j];

        auto point_enu = coord_transformer_->CarToENU(
            {center_line_point_3d_x, center_line_point_3d_y, 0});

        reference_line_point.enu_point.x = static_cast<double>(point_enu.x());
        reference_line_point.enu_point.y = static_cast<double>(point_enu.y());
        reference_line_point.enu_point.z = static_cast<double>(point_enu.z());

        reference_line_point.distance_to_left_lane_border = INVALID_DISTANCE;
        reference_line_point.distance_to_right_lane_border = INVALID_DISTANCE;
        reference_line_point.distance_to_left_road_border = INVALID_DISTANCE;
        reference_line_point.distance_to_right_road_border = INVALID_DISTANCE;
        reference_line_point.distance_to_left_obstacle = INVALID_DISTANCE;
        reference_line_point.distance_to_right_obstacle = INVALID_DISTANCE;
        reference_line_point.lane_width = INVALID_DISTANCE;

        reference_line_point.max_velocity = 0;
        reference_line_point.is_experience_data = false;
        reference_line_point.longitudinal_slope = 0;
        reference_line_point.horizontal_slope = 0;
        reference_line_point.on_route = true;
        reference_line_point.curvature = 0;

        reference_line_point.left_road_border_type.value.value =
            maf_worldmodel::LaneBoundaryForm::PHYSICAL;
        reference_line_point.right_road_border_type.value.value =
            maf_worldmodel::LaneBoundaryForm::PHYSICAL;
        reference_line_point.left_road_border_type.width.value =
            maf_worldmodel::LaneBoundaryWidth::UNKNOWN;
        reference_line_point.right_road_border_type.width.value =
            maf_worldmodel::LaneBoundaryWidth::UNKNOWN;
        if (left_lane_index != INVALID_INDEX) {
          auto &left_lane_width = perception_lanes[left_lane_index].lane_width;
          if ((left_lane_width > 0.0) && (left_lane_width < 0.3)) {
            reference_line_point.left_road_border_type.width.value =
                maf_worldmodel::LaneBoundaryWidth::THIN;
          }
          if (left_lane_width > 0.3) {
            reference_line_point.left_road_border_type.width.value =
                maf_worldmodel::LaneBoundaryWidth::THICK;
          }

          if (right_lane_index != INVALID_INDEX) {
            auto &right_lane_width =
                perception_lanes[right_lane_index].lane_width;
            if ((right_lane_width > 0.0) && (right_lane_width < 0.3)) {
              reference_line_point.right_road_border_type.width.value =
                  maf_worldmodel::LaneBoundaryWidth::THIN;
            }
            if (right_lane_width > 0.3) {
              reference_line_point.right_road_border_type.width.value =
                  maf_worldmodel::LaneBoundaryWidth::THICK;
            }
          }
        }
      }

      if (left_lane_index != INVALID_INDEX) {
        auto &left_lane = perception_lanes[left_lane_index];
        auto &lane_points_3d_x = left_lane.points_3d_x;
        auto &lane_points_3d_y = left_lane.points_3d_y;
        auto lane_points_size =
            std::min(lane_points_3d_x.size(), lane_points_3d_y.size());

        calculateLaneBoundary(left_lane, lane_points_size,
                              output_lane.left_lane_boundary);
        calculateNearestRotatedDistance(
            lane_points_3d_x, lane_points_3d_y, lane_points_size, 0,
            [&output_lane](int index, float distance) {
              auto &reference_line_point =
                  output_lane.reference_line.reference_line_points[index];
              reference_line_point.distance_to_left_lane_border = distance;
            });
      }

      if (right_lane_index != INVALID_INDEX) {
        auto &right_lane = perception_lanes[right_lane_index];
        auto &lane_points_3d_x = right_lane.points_3d_x;
        auto &lane_points_3d_y = right_lane.points_3d_y;
        auto lane_points_size =
            std::min(lane_points_3d_x.size(), lane_points_3d_y.size());
        calculateLaneBoundary(right_lane, lane_points_size,
                              output_lane.right_lane_boundary);

        calculateNearestRotatedDistance(
            lane_points_3d_x, lane_points_3d_y, lane_points_size, 1,
            [&output_lane](int index, float distance) {
              auto &reference_line_point =
                  output_lane.reference_line.reference_line_points[index];
              reference_line_point.distance_to_right_lane_border = distance;
            });
      }

      if (cur_road_edges_.left_road_edge_exist) {
        auto &left_road_edge =
            perception_road_edges[cur_road_edges_.left_road_edge_index];
        auto &road_edge_points_3d_x = left_road_edge.points_3d_x;
        auto &road_edge_points_3d_y = left_road_edge.points_3d_y;
        auto road_edge_points_size = std::min(road_edge_points_3d_x.size(),
                                              road_edge_points_3d_y.size());

        calculateRoadEdge(left_road_edge, road_edge_points_size,
                          output_lane.left_road_edge);

        calculateNearestRotatedDistance(
            road_edge_points_3d_x, road_edge_points_3d_y, road_edge_points_size,
            0, [&output_lane](int index, float distance) {
              auto &reference_line_point =
                  output_lane.reference_line.reference_line_points[index];
              reference_line_point.distance_to_left_road_border = distance;
            });
      }

      if (cur_road_edges_.right_road_edge_exist) {
        auto &right_road_edge =
            perception_road_edges[cur_road_edges_.right_road_edge_index];
        auto &road_edge_points_3d_x = right_road_edge.points_3d_x;
        auto &road_edge_points_3d_y = right_road_edge.points_3d_y;
        auto road_edge_points_size = std::min(road_edge_points_3d_x.size(),
                                              road_edge_points_3d_y.size());

        calculateRoadEdge(right_road_edge, road_edge_points_size,
                          output_lane.right_road_edge);
        calculateNearestRotatedDistance(
            road_edge_points_3d_x, road_edge_points_3d_y, road_edge_points_size,
            1, [&output_lane](int index, float distance) {
              auto &reference_line_point =
                  output_lane.reference_line.reference_line_points[index];
              reference_line_point.distance_to_right_road_border = distance;
            });
      }

      for (auto &reference_line_point :
           output_lane.reference_line.reference_line_points) {
        if (std::fabs(reference_line_point.distance_to_left_lane_border -
                      INVALID_DISTANCE) > 1e-2 &&
            std::fabs(reference_line_point.distance_to_right_lane_border -
                      INVALID_DISTANCE) > 1e-2) {
          reference_line_point.lane_width =
              reference_line_point.distance_to_left_lane_border +
              reference_line_point.distance_to_right_lane_border;
        }
      }
    }
  }

  void reset() override { reset_.store(true); }

private:
  std::shared_ptr<CoordinateTransformer> coord_transformer_;
  std::deque<MLALocalizationPtr> localization_buffer_{};
  std::atomic_bool reset_{false};

  int center_line_points_size_ = 0;
  std::vector<float> center_line_points_3d_y_{};
  std::vector<float> center_line_points_3d_x_{};

  MLALocalizationPtr localization_ptr_{};
  std::vector<InterCenterLine> center_lines_{};
  InterRoadEdge cur_road_edges_;

  IntersectionPoint intersection_point_;

  static constexpr uint64_t MAX_LOCLIZATION_BUFFER_SIZE = 100;
  static constexpr uint64_t MAX_TIMESTAMP_DIFF = 500000;
  static constexpr uint64_t LOC_MAX_TIMESTAMP_ERROR = 10000; // 10ms

  void calculateRoadEdge(const maf_perception_interface::RoadEdge &road_edge,
                         const int &road_edge_points_size,
                         maf_worldmodel::RoadEdge &output_road_edge) override {
    output_road_edge.existence = true;
    output_road_edge.points.resize(road_edge_points_size);
    output_road_edge.type = maf_worldmodel::LaneBoundaryForm::PHYSICAL;

    auto &points_3d_x = road_edge.points_3d_x;
    auto &points_3d_y = road_edge.points_3d_y;

    // float lane_length = 0;
    for (int k = 0; k < road_edge_points_size; k++) {
      auto point_enu =
          coord_transformer_->CarToENU({points_3d_x[k], points_3d_y[k], 0});
      output_road_edge.points[k].x = point_enu.x();
      output_road_edge.points[k].y = point_enu.y();
      output_road_edge.points[k].z = point_enu.z();
    }
  }

  void
  calculateLaneBoundary(const maf_perception_interface::Lane &lane,
                        const int &lane_points_size,
                        maf_worldmodel::LaneBoundary &lane_boundary) override {
    lane_boundary.existence = true;
    auto &lane_points_3d_x = lane.points_3d_x;
    auto &lane_points_3d_y = lane.points_3d_y;
    lane_boundary.points.resize(lane_points_size);
    float lane_length = 0;
    for (int k = 0; k < lane_points_size; k++) {
      auto point_enu = coord_transformer_->CarToENU(
          {lane_points_3d_x[k], lane_points_3d_y[k], 0});
      lane_boundary.points[k].x = point_enu.x();
      lane_boundary.points[k].y = point_enu.y();
      lane_boundary.points[k].z = point_enu.z();

      if (k < lane_points_size - 1) {

        auto &cur_point_x = lane_points_3d_x[k];
        auto &cur_point_y = lane_points_3d_y[k];
        auto &next_point_x = lane_points_3d_x[k + 1];
        auto &next_point_y = lane_points_3d_y[k + 1];

        lane_length += calculateDistance(cur_point_x, cur_point_y, next_point_x,
                                         next_point_y);
      }
    }
    maf_worldmodel::LaneBoundarySegment lane_boundary_segment{};
    lane_boundary_segment.length = lane_length;
    lane_boundary_segment.type.value.value =
        convertLaneType(lane.lane_type.value);

    lane_boundary_segment.type.color.value =
        convertLaneColor(lane.lane_color.value);

    lane_boundary.segments.emplace_back(std::move(lane_boundary_segment));
  }

  void preProcessLanePtr(const PerceptionLanePtr perception_lane_ptr) override {
    intersection_point_.is_valid = false;

    if (!update_center_lane_and_find_current_index(
            perception_lane_ptr->lane_perception, center_lines_,
            intersection_point_)) {
      return;
    }

    auto &road_edges = perception_lane_ptr->road_edge_perception.road_edges;
    cur_road_edges_.left_road_edge_exist = false;
    cur_road_edges_.right_road_edge_exist = false;
    // preprocess the road edge
    for (size_t i = 0; i < road_edges.size(); i++) {
      auto &road_edge = road_edges[i];
      if (road_edge.camera_source.value !=
              maf_perception_interface::CameraSourceEnum::
                  CAMERA_SOURCE_FRONT_MID ||
          road_edge.is_failed_3d) {
        continue;
      }
      if (road_edge.index == -1) {
        cur_road_edges_.left_road_edge_index = i;
        cur_road_edges_.left_road_edge_exist = true;
      } else if (road_edge.index == 1) {
        cur_road_edges_.right_road_edge_index = i;
        cur_road_edges_.right_road_edge_exist = true;
      }
    }
  }

  void calculateNearestRotatedDistance(
      const std::vector<float> &lane_points_3d_x,
      const std::vector<float> &lane_points_3d_y, const int &lane_points_size,
      const int direction,
      const std::function<void(int index, float distance)> &post_processor)
      override {
    // negative when direction is 1:right
    double sign = (direction == 1) ? -1.0 : 1.0;
    if (lane_points_size >= 2) {
      int k = 0;
      for (int j = 0; j < center_line_points_size_; j++) {
        int first_index = j;
        int second_index = j + 1;
        if (j == center_line_points_size_ - 1) {
          second_index = j;
          first_index = j - 1;
        }

        auto yaw = std::atan2(center_line_points_3d_y_[second_index] -
                                  center_line_points_3d_y_[first_index],
                              center_line_points_3d_x_[second_index] -
                                  center_line_points_3d_x_[first_index]);

        auto rotate_matrix = getRoateMatrix(yaw);

        auto rotated_center_line_point =
            rotatePoint(rotate_matrix, {center_line_points_3d_x_[first_index],
                                        center_line_points_3d_y_[first_index]});

        while (k < lane_points_size - 1) {
          auto rotated_lane_point_a = rotatePoint(
              rotate_matrix, {lane_points_3d_x[k], lane_points_3d_y[k]});
          auto rotated_lane_point_b =
              rotatePoint(rotate_matrix,
                          {lane_points_3d_x[k + 1], lane_points_3d_y[k + 1]});

          auto rotated_distance =
              sign * scanLineToSegment(rotated_center_line_point,
                                       rotated_lane_point_a,
                                       rotated_lane_point_b);

          if (rotated_lane_point_a.x() <= rotated_center_line_point.x() &&
              rotated_lane_point_b.x() >= rotated_center_line_point.x()) {
            post_processor(j, float(rotated_distance));

            break;
          }

          if (rotated_lane_point_a.x() > rotated_center_line_point.x()) {
            break;
          }

          if (rotated_lane_point_b.x() < rotated_center_line_point.x()) {
            k++;
          }
        }
      }
    }
  }
  bool getNearestLocalization(const uint64_t &query_timestamp,
                              MLALocalizationPtr &localization_ptr) override {
    if (localization_buffer_.empty()) {
      return false;
    }

    while (localization_buffer_.size() > 1 &&
           localization_buffer_.front()->meta.timestamp_us +
                   uint64_t(LOC_MAX_TIMESTAMP_ERROR) <
               query_timestamp) {
      localization_buffer_.pop_front();
    }

    if (localization_buffer_.front()->meta.timestamp_us +
            uint64_t(MAX_TIMESTAMP_DIFF) <
        query_timestamp) {
      localization_buffer_.pop_front();
      return false;
    }
    localization_ptr = std::move(localization_buffer_.front());
    localization_buffer_.pop_front();

    bool mla_valid = (int8_t(localization_ptr->status.status_info.type) > 0);

    return mla_valid;
  }
};

std::shared_ptr<DdmapGenerator> DdmapGenerator::make() {
  return std::make_shared<DdmapGeneratorImpl>();
}
} // namespace worldmodel_v1
} // namespace msd_worldmodel
