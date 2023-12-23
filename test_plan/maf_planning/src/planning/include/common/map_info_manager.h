#ifndef MSQUARE_DECISION_PLANNING_COMMON_MAP_INFO_MANAGER_H
#define MSQUARE_DECISION_PLANNING_COMMON_MAP_INFO_MANAGER_H

#include <memory>
#include <string>
#include <unordered_set>

#include "common/ego_state_manager.h"
#include "common/lateral_virtual_lane.h"
#include "common/math/transform.h"
#include "common/planning_context.h"
#include "common/refline.h"
#include "common/refline/reference_line_provider.h"

#include "path/discretized_path.h"
#include "planner/message_type.h"
#include "planning/common/common.h"
#include "pnc.h"

namespace msquare {

class WorldModel;

using namespace msd_planning;
using namespace maf_worldmodel;

struct SpeedChangePoint {
  double x;
  double y;
  double speed;
};

class MSDMapInfo {
public:
  MSDMapInfo() {}

  MSDMapInfo(const MSDMapInfo &map_info) {
    lane_ = map_info.lane_;
    map_poi_info_ = map_info.map_poi_info_;
    intersection_ = map_info.intersection_;
    self_position_ = map_info.self_position_;
    lane_strategy_ = map_info.lane_strategy_;
    lane_merging_splitting_point_ = map_info.lane_merging_splitting_point_;
    scene_type_ = map_info.scene_type_;
    dis_to_ramp_ = map_info.dis_to_ramp_;
    ramp_length_ = map_info.ramp_length_;
  }

  MSDMapInfo(const maf_worldmodel::ProcessedMapData &processed_map_data) {
    lane_ = processed_map_data.lanes;
    map_poi_info_ = processed_map_data.map_poi_info;
    intersection_ = processed_map_data.intersections;
    self_position_ = processed_map_data.self_position;
    lane_strategy_ = processed_map_data.lane_strategy;
    lane_merging_splitting_point_ =
        processed_map_data.lane_merging_splitting_points;
    MSD_LOG(INFO, "zyl target pt debug %d",
            processed_map_data.extra_info.available);
    if (processed_map_data.extra_info.available &
        maf_worldmodel::ExtraInfo::SCENE_TYPE) {
      scene_type_ = processed_map_data.extra_info.scene_type;
      MSD_LOG(INFO, "zyl target pt debug SCENE_TYPE");
    }
    if (processed_map_data.extra_info.available &
        maf_worldmodel::ExtraInfo::RAMP_MANEUVER) {
      // fetch ramp maneuver from wm json
      dis_to_ramp_.clear();
      ramp_length_.clear();

      dis_to_ramp_.push_back(
          processed_map_data.extra_info.ramp_maneuver.dis_to_ramp);
      ramp_length_.push_back(
          processed_map_data.extra_info.ramp_maneuver.length);
      MSD_LOG(INFO, "zyl target pt debug RAMP_MANEUVER");
    }
    // perception target point
    if (processed_map_data.extra_info.available &
        maf_worldmodel::ExtraInfo::RESERVED_INFO) {
      MSD_LOG(INFO, "zyl target pt debug RESERVED_INFO");
      auto target_pt_str_reader =
          mjson::Reader(processed_map_data.extra_info.reserved_info);
      auto target_pt_enu_str = target_pt_str_reader.get<std::string>(
          "intersection_point_enu", false, "");
      auto target_pt_car_str = target_pt_str_reader.get<std::string>(
          "intersection_point_car", false, "");
      if (target_pt_enu_str != "" && target_pt_car_str != "") {
        auto pt_enu_reader = mjson::Reader(target_pt_enu_str);
        auto pt_car_reader = mjson::Reader(target_pt_car_str);
        pi_target_pt_enu_.push_back(pt_enu_reader.get<double>("x", false, 0));
        pi_target_pt_enu_.push_back(pt_enu_reader.get<double>("y", false, 0));
        pi_target_pt_car_.push_back(pt_car_reader.get<double>("x", false, 0));
        pi_target_pt_car_.push_back(pt_car_reader.get<double>("y", false, 0));
        pi_target_pt_exist_ = true;
        if (pi_target_pt_enu_.size() > 1 && pi_target_pt_car_.size() > 1) {
          MSD_LOG(INFO, "zyl target pt debug (%.2f %.2f) (%.2f %.2f)",
                  pi_target_pt_enu_.at(0), pi_target_pt_enu_.at(1),
                  pi_target_pt_car_.at(0), pi_target_pt_car_.at(1));
        } else {
          MSD_LOG(INFO, "zyl target pt debug enu_pt_size[%d] car_pt_size[%d]",
                  pi_target_pt_enu_.size(), pi_target_pt_car_.size());
        }
      }
    }
  }

  MSDMapInfo &operator=(const MSDMapInfo &map_info) {
    if (this == &map_info) {
      return *this;
    }
    lane_ = map_info.lane_;
    map_poi_info_ = map_info.map_poi_info_;
    intersection_ = map_info.intersection_;
    self_position_ = map_info.self_position_;
    lane_strategy_ = map_info.lane_strategy_;
    lane_merging_splitting_point_ = map_info.lane_merging_splitting_point_;
    scene_type_ = map_info.scene_type_;
    dis_to_ramp_ = map_info.dis_to_ramp_;
    ramp_length_ = map_info.ramp_length_;
    pi_target_pt_exist_ = map_info.pi_target_pt_exist_;
    pi_target_pt_enu_ = map_info.pi_target_pt_enu_;
    pi_target_pt_car_ = map_info.pi_target_pt_car_;
    return *this;
  }

  void update(const std::shared_ptr<WorldModel> &world_model, double ego_vel,
              double ego_v_cruise, bool is_ddmap) {
    update_lane_boundary_segments();
    update_lane_tasks();
    update_refline_points(world_model);
    update_speed_limit(ego_vel, ego_v_cruise, is_ddmap);
    // update_vision_lanes();
    // update_lane_boundary_polyline();
  }

  int lanes_num() const { return lane_.size(); }
  int current_lane_index() const {
    if (lane_.size() > 0) {
      return 0 - lane_[0].relative_id;
    }

    return 0;
  }

  bool is_in_map_area() const { return self_position_.in_map_area; }

  bool is_in_intersection() const { return self_position_.in_intersection; }

  bool is_on_ramp() const { return self_position_.on_ramp; }

  bool has_task() const {
    return !(lane_strategy_.strategy_start_with_current_lane.empty());
  }

  int current_tasks_id() const {
    int current_tasks = 0;
    if (current_tasks_.empty()) {
      return 0;
    }
    for (int i = 0; i < current_tasks_.size(); i++) {
      if (current_tasks_[i] != current_tasks_[0]) {
        break;
      }
      current_tasks += current_tasks_[i];
    }

    // clip tasks according to lane nums
    int lane_index = current_lane_index();
    int right_lane_nums = std::max(lanes_num() - lane_index - 1, 0);
    current_tasks = std::min(current_tasks, right_lane_nums);

    return current_tasks;
  }

  int left_lane_tasks_id() const {
    int left_lane_tasks = 0;
    if (left_lane_tasks_.empty()) {
      return 0;
    }
    for (int i = 0; i < left_lane_tasks_.size(); i++) {
      if (left_lane_tasks_[i] != left_lane_tasks_[0]) {
        break;
      }
      left_lane_tasks += left_lane_tasks_[i];
    }

    // clip left_lane_tasks according to current_tasks_id
    if (current_tasks_id() >= 0) {
      left_lane_tasks = std::min(left_lane_tasks, current_tasks_id() + 1);
    }

    return left_lane_tasks;
  }

  int right_lane_tasks_id() const {
    int right_lane_tasks = 0;
    if (right_lane_tasks_.empty()) {
      return 0;
    }
    for (int i = 0; i < right_lane_tasks_.size(); i++) {
      if (right_lane_tasks_[i] != right_lane_tasks_[0]) {
        break;
      }
      right_lane_tasks += right_lane_tasks_[i];
    }

    // clip right_lane_tasks according to current_tasks_id
    if (current_tasks_id() > 0) {
      right_lane_tasks = std::min(right_lane_tasks, current_tasks_id() - 1);
    }

    return right_lane_tasks;
  }

  int lc_map_decision() const { return current_tasks_id(); }

  int lc_map_decision_left() const { return left_lane_tasks_id(); }

  int lc_map_decision_right() const { return right_lane_tasks_id(); }

  int current_lane_out_id() const {
    for (size_t i = 1; i < current_refline_points().size(); ++i) {
      if (!current_refline_points()[i].in_intersection &&
          current_refline_points()[i - 1].in_intersection &&
          current_refline_points()[i - 1].car_point.x > 0) {
        auto track_id = current_refline_points()[i].track_id;
        int id = track_id & 0xf;
        if (is_in_intersection()) {
          if (intersection_.size() < 1 || id < 0 ||
              id > intersection_[0].lanes_out.size() - 1) {
            return -1;
          }
          return intersection_[0].lanes_out.size() - 1 - id;
        } else {
          if (intersection_.size() < 2 || id < 0 ||
              id > intersection_[1].lanes_out.size() - 1) {
            return -1;
          }
          return intersection_[1].lanes_out.size() - 1 - id;
        }
      }
    }
    return -1;
  }

  int left_lane_out_id() const {
    for (size_t i = 1; i < left_refline_points().size(); ++i) {
      if (!left_refline_points()[i].in_intersection &&
          left_refline_points()[i - 1].in_intersection &&
          left_refline_points()[i - 1].car_point.x > 0) {
        auto track_id = left_refline_points()[i].track_id;
        int id = track_id & 0xf;
        if (is_in_intersection()) {
          if (intersection_.size() < 1 || id < 0 ||
              id > intersection_[0].lanes_out.size() - 1) {
            return -1;
          }
          return intersection_[0].lanes_out.size() - 1 - id;
        } else {
          if (intersection_.size() < 2 || id < 0 ||
              id > intersection_[1].lanes_out.size() - 1) {
            return -1;
          }
          return intersection_[1].lanes_out.size() - 1 - id;
        }
      }
    }
    return -1;
  }

  int right_lane_out_id() const {
    for (size_t i = 1; i < right_refline_points().size(); ++i) {
      if (!right_refline_points()[i].in_intersection &&
          right_refline_points()[i - 1].in_intersection &&
          right_refline_points()[i - 1].car_point.x > 0) {
        auto track_id = right_refline_points()[i].track_id;
        int id = track_id & 0xf;
        if (is_in_intersection()) {
          if (intersection_.size() < 1 || id < 0 ||
              id > intersection_[0].lanes_out.size() - 1) {
            return -1;
          }
          return intersection_[0].lanes_out.size() - 1 - id;
        } else {
          if (intersection_.size() < 2 || id < 0 ||
              id > intersection_[1].lanes_out.size() - 1) {
            return -1;
          }
          return intersection_[1].lanes_out.size() - 1 - id;
        }
      }
    }
    return -1;
  }

  int curr_intsect_task() const {
    if (is_in_intersection()) {
      if (intersection_.size() < 1) {
        return Direction::UNKNOWN;
      } else {
        return intersection_[0].direction.value;
      }
    } else {
      if (intersection_.size() < 2) {
        return Direction::UNKNOWN;
      } else {
        return intersection_[1].direction.value;
      }
    }
  }

  int last_intsect_task() const {
    if (intersection_.size() < 1) {
      return Direction::UNKNOWN;
    }

    return intersection_[0].direction.value;
  }

  int road_type() const { return curr_intsect_task(); }

  int next_merge_type() const {
    auto &point_data = lane_merging_splitting_point_;
    if (point_data.empty()) {
      return Orientation::UNKNOWN;
    }

    return point_data[0].orientation.value;
  }

  LaneMergingSplittingPointData next_merge_info() const {
    auto &point_data = lane_merging_splitting_point_;
    if (point_data.empty()) {
      LaneMergingSplittingPointData result{};
      result.orientation.value = Orientation::UNKNOWN;
      return result;
    }

    return point_data[0];
  }

  uint8_t lanes_merge_type(int lane_index = INT_MAX) const {
    if (lane_index == INT_MAX) {
      lane_index = current_lane_index();
    }
    if (lane_index < 0 || lane_index >= lane_.size()) {
      return MergeType::UNKNOWN;
    }
    auto &merge_point = lane_[lane_index].merge_point;

    return merge_point.type.value;
  }

  double distance_to_lanes_merge(int lane_index = INT_MAX) const {
    if (lane_index == INT_MAX) {
      lane_index = current_lane_index();
    }

    if (lane_index < 0 || lane_index >= lane_.size()) {
      return DBL_MAX;
    }
    auto &merge_point = lane_[lane_index].merge_point;

    return merge_point.distance;
  }

  uint8_t lanes_y_point_type(int lane_index = INT_MAX) const {
    if (lane_index == INT_MAX) {
      lane_index = current_lane_index();
    }
    if (lane_index < 0 || lane_index >= lane_.size()) {
      return MergeType::UNKNOWN;
    }
    auto &Y_point = lane_[lane_index].y_point;

    return Y_point.type.value;
  }

  double distance_to_lanes_y_point(int lane_index = INT_MAX) const {
    if (lane_index == INT_MAX) {
      lane_index = current_lane_index();
    }
    if (lane_index < 0 || lane_index >= lane_.size()) {
      return DBL_MAX;
    }
    auto &Y_point = lane_[lane_index].y_point;

    return Y_point.distance;
  }

  double lc_start_dis() const {
    auto &msd_ranges = lane_strategy_.current_lane_change_available_interval;
    if (msd_ranges.empty()) {
      return DBL_MAX;
    }

    return msd_ranges[0].begin;
  }

  double hack_lc_end_dis(CartEgoState ego_state) {
    // hack for jinhu
    double start_x_jh = 49123.9406643;
    double start_y_jh = -18191.6014521;
    // hack for shenhai
    double start_x_sh = 58874.7383105;
    double start_y_sh = -26040.4420989;

    double dis_to_start_point_jh =
        std::hypot(ego_state.ego_enu.position.x - start_x_jh,
                   ego_state.ego_enu.position.y - start_y_jh);
    double dis_to_start_point_sh =
        std::hypot(ego_state.ego_enu.position.x - start_x_sh,
                   ego_state.ego_enu.position.y - start_y_sh);

    if ((dis_to_start_point_jh < 30.0) || (dis_to_start_point_sh < 30.0)) {
      hack_ = true;
    }

    if (hack_ && lc_map_decision() < 0) {
      new_lc_end_dis_ =
          std::max(0.0, double(int(6000 - std::min(dis_to_start_point_jh,
                                                   dis_to_start_point_sh)) %
                               1500));
    } else {
      new_lc_end_dis_ = DBL_MAX;
    }

    if ((dis_to_start_point_jh > 3000.0) && (dis_to_start_point_sh > 3000.0)) {
      hack_ = false;
    }

    MSD_LOG(INFO, "xzj hack: %d", hack_);
    MSD_LOG(INFO, "xzj dis_to_start_point_jh: %f", dis_to_start_point_jh);
    MSD_LOG(INFO, "xzj dis_to_start_point_sh: %f", dis_to_start_point_sh);
    MSD_LOG(INFO, "xzj new_lc_end_dis_: %f", new_lc_end_dis_);

    return true;
  }

  double lc_end_dis() const {
    auto &msd_ranges = lane_strategy_.current_lane_change_available_interval;
    if (msd_ranges.empty()) {
      return DBL_MAX;
    }

    auto &point_data = lane_merging_splitting_point_;

    double split_distance = 5000.0;
    for (int i = 0; i < point_data.size(); i++) {
      if (!point_data[i].is_continue && point_data[i].is_split &&
          point_data[i].orientation.value == Orientation::RIGHT) {
        split_distance = point_data[i].distance;
      }
    }

    if (split_distance >= 0 && is_on_highway()) {
      return std::min(
          std::min(msd_ranges[msd_ranges.size() - 1].end, split_distance),
          new_lc_end_dis_);
    } else {
      return std::min(msd_ranges[msd_ranges.size() - 1].end, new_lc_end_dis_);
    }
  }

  double lc_end_dis_left() const {
    auto &msd_ranges = lane_strategy_.left_lane_change_available_interval;
    if (msd_ranges.empty()) {
      return DBL_MAX;
    }

    auto &point_data = lane_merging_splitting_point_;

    double split_distance = 5000.0;
    for (int i = 0; i < point_data.size(); i++) {
      if (!point_data[i].is_continue && point_data[i].is_split &&
          point_data[i].orientation.value == Orientation::RIGHT) {
        split_distance = point_data[i].distance;
      }
    }

    if (split_distance >= 0 && is_on_highway()) {
      return std::min(
          std::min(msd_ranges[msd_ranges.size() - 1].end, split_distance),
          new_lc_end_dis_);
    } else {
      return std::min(msd_ranges[msd_ranges.size() - 1].end, new_lc_end_dis_);
    }
  }

  double lc_end_dis_right() const {
    auto &msd_ranges = lane_strategy_.right_lane_change_available_interval;
    if (msd_ranges.empty()) {
      return DBL_MAX;
    }

    auto &point_data = lane_merging_splitting_point_;

    double split_distance = 5000.0;
    for (int i = 0; i < point_data.size(); i++) {
      if (!point_data[i].is_continue && point_data[i].is_split &&
          point_data[i].orientation.value == Orientation::RIGHT) {
        split_distance = point_data[i].distance;
      }
    }

    if (split_distance >= 0 && is_on_highway()) {
      return std::min(
          std::min(msd_ranges[msd_ranges.size() - 1].end, split_distance),
          new_lc_end_dis_);
    } else {
      return std::min(msd_ranges[msd_ranges.size() - 1].end, new_lc_end_dis_);
    }
  }

  double dis_to_ramp() const {
    auto &point_data = lane_merging_splitting_point_;

    double split_distance = 5000.0;
    for (int i = 0; i < point_data.size(); i++) {
      if (!point_data[i].is_continue && point_data[i].is_split &&
          point_data[i].orientation.value == Orientation::RIGHT) {
        split_distance = point_data[i].distance;
      } else if (!point_data[i].is_continue && !point_data[i].is_split &&
                 point_data[i].orientation.value == Orientation::RIGHT) {
        split_distance = point_data[i].distance;
      }
    }

    double ramp_distance = 5000.0;
    if (dis_to_ramp_.size() > 0) {
      if (dis_to_ramp_[0] > 0) {
        ramp_distance = dis_to_ramp_[0];
      } else if ((dis_to_ramp_[0] + ramp_length_[0]) > 0) {
        ramp_distance = dis_to_ramp_[0] + ramp_length_[0];
      }
    }

    ramp_distance = std::min(split_distance, ramp_distance);

    return ramp_distance;
  }

  double dist_to_tollgate() const { return DBL_MAX; }

  double dis_to_road_split() const {
    double dis_to_split = DBL_MAX;
    for (auto poi : map_poi_info_) {
      if (poi.type.value == MapPOIType::ROAD_SPLIT) {
        if (dis_to_split > poi.distance) {
          dis_to_split = poi.distance;
        }
      }
    }

    return dis_to_split;
  }

  double intsect_length() const {
    if (is_in_intersection()) {
      if (intersection_.size() < 1) {
        return DBL_MAX;
      } else {
        return intersection_[0].length;
      }
    } else {
      if (intersection_.size() < 2) {
        return DBL_MAX;
      } else {
        return intersection_[1].length;
      }
    }
  }

  double dist_to_intsect() const {
    if (is_in_intersection()) {
      if (intersection_.size() < 1) {
        return DBL_MAX;
      } else {
        return intersection_[0].distance_to_stop_line.distance_to_stop_line;
      }
    } else {
      if (intersection_.size() < 2) {
        return DBL_MAX;
      } else {
        return intersection_[1].distance_to_stop_line.distance_to_stop_line;
      }
    }
  }

  double dist_to_last_intsect() const {
    if (intersection_.size() < 1) {
      return DBL_MAX;
    }

    return intersection_[0].distance_to_stop_line.distance_to_stop_line +
           intersection_[0].length;
  }

  double distance_to_stop_line() const { return dist_to_intsect(); }

  double distance_to_last_stop_line() const {
    if (intersection_.size() < 1) {
      return DBL_MAX;
    }

    return intersection_[0].distance_to_stop_line.distance_to_stop_line;
  }

  double global_end_dis() const {
    double global_end_dis = DBL_MAX;

    for (auto poi : map_poi_info_) {
      if (poi.type.value == MapPOIType::DESTINATION) {
        global_end_dis = poi.distance;
      }
    }

    return global_end_dis;
  }

  double crossing_length() const { return intsect_length(); }

  double distance_to_crossing() const { return dist_to_intsect(); }

  double distance_to_next_merge() const {
    auto &point_data = lane_merging_splitting_point_;
    if (point_data.empty()) {
      return DBL_MAX;
    }

    return point_data[0].distance;
  }

  double curr_merge_length() const {
    auto &point_data = lane_merging_splitting_point_;
    if (point_data.empty()) {
      return DBL_MAX;
    }

    return point_data[0].length;
  }

  bool is_on_highway() const;

  uint8_t current_lane_marks() const {
    int index = current_lane_index();
    if (index < 0 || index >= lane_.size()) {
      return 0;
    }

    return lane_[index].lane_marks.value;
  }

  uint8_t left_lane_marks() const {
    int index = current_lane_index() - 1;
    if (index < 0 || index >= lane_.size()) {
      return 0;
    }

    return lane_[index].lane_marks.value;
  }

  uint8_t right_lane_marks() const {
    int index = current_lane_index() + 1;
    if (index < 0 || index >= lane_.size()) {
      return 0;
    }

    return lane_[index].lane_marks.value;
  }

  uint8_t left_lane_type() const {
    int index = current_lane_index() - 1;
    if (index < 0 || index >= lane_.size()) {
      return 0;
    }

    return lane_[index].lane_type.value;
  }

  uint8_t right_lane_type() const {
    int index = current_lane_index() + 1;
    if (index < 0 || index >= lane_.size()) {
      return 0;
    }

    return lane_[index].lane_type.value;
  }

  uint8_t find_lane_marks(int index) const {
    if (index < 0 || index >= lane_.size()) {
      return 0;
    }
    return lane_[index].lane_marks.value;
  }

  int lane_marks_size(int index) const {
    if (index < 0 || index >= lane_.size()) {
      return 0;
    }

    int size = 0;
    uint8_t value = lane_[index].lane_marks.value;
    while (value) {
      size++;
      value &= (value - 1);
    }

    return size;
  }

  bool is_next_rightmost() const { return false; }

  double v_cruise() const { return v_cruise_; };
  double v_cruise_current() const { return current_lane_speed_limit_ / 3.6; };
  double v_cruise_change_dis() const {
    return std::sqrt(std::pow(speed_change_point_.x, 2) +
                     std::pow(speed_change_point_.y, 2));
  };

  int traffic_light_direction() const {
    if (is_on_highway()) {
      return Direction::GO_STRAIGHT;
    }
    if (intersection_.size() < 2) {
      return Direction::UNKNOWN;
    } else {
      return intersection_[1].direction.value;
    }
  }

  int intsection_lanes_out_num() const {
    if (is_in_intersection()) {
      if (intersection_.size() < 1) {
        return 1;
      } else {
        return intersection_[0].lanes_out.size();
      }
    } else {
      if (intersection_.size() < 2) {
        return 1;
      } else {
        return intersection_[1].lanes_out.size();
      }
    }
  }

  double get_distance_to_dash_line(std::string direction) const {
    double dis_to_dash_line = std::numeric_limits<double>::lowest();
    if (is_in_intersection()) {
      dis_to_dash_line = dist_to_last_intsect();
      if (!intersection_.empty()) {
        auto cur_lane_out_id = current_lane_out_id();
        if (cur_lane_out_id >= 0 &&
            cur_lane_out_id < intersection_[0].lanes_out.size()) {
          auto lane_data = intersection_[0].lanes_out[cur_lane_out_id];
          if (direction == "left") {
            for (int i = 0; i < lane_data.left_lane_boundary.segments.size();
                 i++) {
              if (lane_data.left_lane_boundary.segments[i].type.value.value !=
                  LaneBoundaryForm::SOLID) {
                dis_to_dash_line +=
                    lane_data.left_lane_boundary.segments[i].length;
              } else {
                break;
              }
            }
          } else if (direction == "right") {
            for (int i = 0; i < lane_data.right_lane_boundary.segments.size();
                 i++) {
              if (lane_data.right_lane_boundary.segments[i].type.value.value !=
                  LaneBoundaryForm::SOLID) {
                dis_to_dash_line +=
                    lane_data.right_lane_boundary.segments[i].length;
              } else {
                break;
              }
            }
          }
        }
      }
    } else {
      dis_to_dash_line = 0.;
      if (direction == "left") {
        for (int i = 0; i < left_boundary_info().size(); i++) {
          if (left_boundary_info()[i].type.value.value !=
                  LaneBoundaryForm::SOLID &&
              left_boundary_info()[i].type.value.value !=
                  LaneBoundaryForm::PHYSICAL) {
            dis_to_dash_line += left_boundary_info()[i].length;
          } else {
            if (left_boundary_info()[i].length < 20. &&
                i < (int)left_boundary_info().size() - 1 &&
                left_boundary_info()[i + 1].type.value.value !=
                    LaneBoundaryForm::SOLID &&
                left_boundary_info()[i + 1].type.value.value !=
                    LaneBoundaryForm::PHYSICAL &&
                left_boundary_info()[i + 1].length > 10.) {
              dis_to_dash_line += left_boundary_info()[i].length;
            } else {
              break;
            }
          }
        }
        if (std::fabs(dis_to_dash_line - dist_to_intsect()) < 1.) {
          dis_to_dash_line += intsect_length();
        }
      } else if (direction == "right") {
        for (int i = 0; i < right_boundary_info().size(); i++) {
          if (right_boundary_info()[i].type.value.value !=
                  LaneBoundaryForm::SOLID &&
              right_boundary_info()[i].type.value.value !=
                  LaneBoundaryForm::PHYSICAL) {
            dis_to_dash_line += right_boundary_info()[i].length;
          } else {
            if (right_boundary_info()[i].length < 20. &&
                i < (int)right_boundary_info().size() - 1 &&
                right_boundary_info()[i + 1].type.value.value !=
                    LaneBoundaryForm::SOLID &&
                right_boundary_info()[i + 1].type.value.value !=
                    LaneBoundaryForm::PHYSICAL &&
                right_boundary_info()[i + 1].length > 10.) {
              dis_to_dash_line += right_boundary_info()[i].length;
            } else {
              break;
            }
          }
        }
        if (std::fabs(dis_to_dash_line - dist_to_intsect()) < 1.) {
          dis_to_dash_line += intsect_length();
        }
      }
    }
    return dis_to_dash_line;
  }

  double get_distance_to_laneout_dash_line(std::string lane,
                                           std::string direction) const {
    double dis_to_laneout_dash_line = 0.;
    if (lane == "clane") {
      if (direction == "left") {
        for (int i = 0; i < cout_left_boundary_info().size(); i++) {
          if (cout_left_boundary_info()[i].type.value.value !=
              LaneBoundaryForm::SOLID) {
            dis_to_laneout_dash_line += cout_left_boundary_info()[i].length;
          } else {
            break;
          }
        }
      } else if (direction == "right") {
        for (int i = 0; i < cout_right_boundary_info().size(); i++) {
          if (cout_right_boundary_info()[i].type.value.value !=
              LaneBoundaryForm::SOLID) {
            dis_to_laneout_dash_line += cout_right_boundary_info()[i].length;
          } else {
            break;
          }
        }
      }
    } else if (lane == "llane") {
      if (direction == "left") {
        for (int i = 0; i < lout_left_boundary_info().size(); i++) {
          if (lout_left_boundary_info()[i].type.value.value !=
              LaneBoundaryForm::SOLID) {
            dis_to_laneout_dash_line += lout_left_boundary_info()[i].length;
          } else {
            break;
          }
        }
      } else if (direction == "right") {
        for (int i = 0; i < lout_right_boundary_info().size(); i++) {
          if (lout_right_boundary_info()[i].type.value.value !=
              LaneBoundaryForm::SOLID) {
            dis_to_laneout_dash_line += lout_right_boundary_info()[i].length;
          } else {
            break;
          }
        }
      }
    } else if (lane == "rlane") {
      if (direction == "left") {
        for (int i = 0; i < rout_left_boundary_info().size(); i++) {
          if (rout_left_boundary_info()[i].type.value.value !=
              LaneBoundaryForm::SOLID) {
            dis_to_laneout_dash_line += rout_left_boundary_info()[i].length;
          } else {
            break;
          }
        }
      } else if (direction == "right") {
        for (int i = 0; i < rout_right_boundary_info().size(); i++) {
          if (rout_right_boundary_info()[i].type.value.value !=
              LaneBoundaryForm::SOLID) {
            dis_to_laneout_dash_line += rout_right_boundary_info()[i].length;
          } else {
            break;
          }
        }
      }
    }
    return dis_to_laneout_dash_line;
  }

  double get_distance_to_final_dash_line(std::string direction) const {
    double dis_to_dash_line = std::numeric_limits<double>::lowest();
    int l_solid_combine_cnt = 0;
    int r_solid_combine_cnt = 0;
    for (int i = (int)left_boundary_info().size() - 1; i > -1; i--) {
      if (left_boundary_info()[i].type.value.value != LaneBoundaryForm::SOLID &&
          left_boundary_info()[i].type.value.value !=
              LaneBoundaryForm::PHYSICAL) {
        if (i > 1 &&
            left_boundary_info()[0].type.value.value !=
                LaneBoundaryForm::SOLID &&
            left_boundary_info()[0].type.value.value !=
                LaneBoundaryForm::PHYSICAL &&
            left_boundary_info()[i].length < 10.) {
          for (int j = i - 1; j > -1; j--) {
            if (left_boundary_info()[j].type.value.value ==
                    LaneBoundaryForm::SOLID ||
                left_boundary_info()[j].type.value.value ==
                    LaneBoundaryForm::PHYSICAL ||
                (j > 1 && j < i - 1 && left_boundary_info()[j].length < 10. &&
                 (left_boundary_info()[j - 1].type.value.value ==
                      LaneBoundaryForm::SOLID ||
                  left_boundary_info()[j - 1].type.value.value ==
                      LaneBoundaryForm::PHYSICAL))) {
              l_solid_combine_cnt += 1;
            } else {
              if (j < i - 1)
                l_solid_combine_cnt += 1;
              break;
            }
          }
          break;
        } else {
          break;
        }
      } else {
        l_solid_combine_cnt += 1;
      }
    }
    for (int i = (int)right_boundary_info().size() - 1; i > -1; i--) {
      if (right_boundary_info()[i].type.value.value !=
              LaneBoundaryForm::SOLID &&
          right_boundary_info()[i].type.value.value !=
              LaneBoundaryForm::PHYSICAL) {
        if (i > 1 &&
            right_boundary_info()[0].type.value.value !=
                LaneBoundaryForm::SOLID &&
            right_boundary_info()[0].type.value.value !=
                LaneBoundaryForm::PHYSICAL &&
            right_boundary_info()[i].length < 10.) {
          for (int j = i - 1; j > -1; j--) {
            if (right_boundary_info()[j].type.value.value ==
                    LaneBoundaryForm::SOLID ||
                right_boundary_info()[j].type.value.value ==
                    LaneBoundaryForm::PHYSICAL ||
                (j > 1 && j < i - 1 && right_boundary_info()[j].length < 10. &&
                 (right_boundary_info()[j - 1].type.value.value ==
                      LaneBoundaryForm::SOLID ||
                  right_boundary_info()[j - 1].type.value.value ==
                      LaneBoundaryForm::PHYSICAL))) {
              r_solid_combine_cnt += 1;
            } else {
              if (j < i - 1)
                r_solid_combine_cnt += 1;
              break;
            }
          }
          break;
        } else {
          break;
        }
      } else {
        r_solid_combine_cnt += 1;
      }
    }
    if (is_in_intersection()) {
      dis_to_dash_line = dist_to_last_intsect();
      if (!intersection_.empty()) {
        auto cur_lane_out_id = current_lane_out_id();
        if (cur_lane_out_id >= 0 &&
            cur_lane_out_id < intersection_[0].lanes_out.size()) {
          auto lane_data = intersection_[0].lanes_out[cur_lane_out_id];
          if (direction == "left") {
            for (int i = 0; i < lane_data.left_lane_boundary.segments.size();
                 i++) {
              if (lane_data.left_lane_boundary.segments[i].type.value.value !=
                  LaneBoundaryForm::SOLID) {
                dis_to_dash_line +=
                    lane_data.left_lane_boundary.segments[i].length;
              } else {
                break;
              }
            }
          } else if (direction == "right") {
            for (int i = 0; i < lane_data.right_lane_boundary.segments.size();
                 i++) {
              if (lane_data.right_lane_boundary.segments[i].type.value.value !=
                  LaneBoundaryForm::SOLID) {
                dis_to_dash_line +=
                    lane_data.right_lane_boundary.segments[i].length;
              } else {
                break;
              }
            }
          }
        }
      }
    } else {
      dis_to_dash_line = 0.;
      if (direction == "left") {
        for (int i = 0;
             i < (int)left_boundary_info().size() - l_solid_combine_cnt; i++) {
          // if (left_boundary_info().size() > 2) {
          //   dis_to_dash_line = std::numeric_limits<double>::max();
          //   return dis_to_dash_line;
          // }
          // if (left_boundary_info()[i].type.value.value !=
          // LaneBoundaryForm::SOLID
          // ||
          //     i < (int)left_boundary_info().size() - 1) {
          //   dis_to_dash_line += left_boundary_info()[i].length;
          // } else {
          //   break;
          // }
          dis_to_dash_line += left_boundary_info()[i].length;
        }
        if (left_boundary_info().size() == 1 &&
            left_boundary_info()[0].type.value.value ==
                LaneBoundaryForm::DASH) {
          dis_to_dash_line += intsect_length();
        }
      } else if (direction == "right") {
        // if (right_boundary_info().size() > 2) {
        //   dis_to_dash_line = std::numeric_limits<double>::max();
        //   return dis_to_dash_line;
        // }
        for (int i = 0;
             i < (int)right_boundary_info().size() - r_solid_combine_cnt; i++) {
          // if (right_boundary_info()[i].type.value.value !=
          // LaneBoundaryForm::SOLID
          // ||
          //     i < (int)right_boundary_info().size() - 1) {
          //   dis_to_dash_line += right_boundary_info()[i].length;
          // } else {
          //   break;
          // }
          dis_to_dash_line += right_boundary_info()[i].length;
        }
        if (right_boundary_info().size() == 1 &&
            right_boundary_info()[0].type.value.value ==
                LaneBoundaryForm::DASH) {
          dis_to_dash_line += intsect_length();
        }
      }
    }
    return dis_to_dash_line;
  }

  double get_distance_to_stop_line() const {
    if (is_in_intersection()) {
      if (intersection_.size() < 2) {
        return DBL_MAX;
      } else {
        return intersection_[1].distance_to_stop_line.distance_to_stop_line;
      }
    } else {
      if (intersection_.size() < 2) {
        return DBL_MAX;
      } else {
        return intersection_[1].distance_to_stop_line.distance_to_stop_line;
      }
    }
  }

  double get_distance_to_road_border(std::string direction, double distance,
                                     int f_position) const {
    std::vector<ReferenceLinePointDerived> f_refline_points;
    double border_dist = 1000.;
    if (f_position == 0) {
      f_refline_points = current_refline_points();
    } else if (f_position == -1) {
      f_refline_points = left_refline_points();
    } else if (f_position == 1) {
      f_refline_points = right_refline_points();
    }
    for (auto &p : f_refline_points) {
      if (p.car_point.x > 0. && p.car_point.x < distance + 10.) {
        if (direction == "left") {
          if (p.distance_to_left_road_border < border_dist) {
            border_dist = p.distance_to_left_road_border;
          }
        } else if (direction == "right") {
          if (p.distance_to_right_road_border < border_dist) {
            border_dist = p.distance_to_right_road_border;
          }
        }
      }
    }
    return border_dist;
  }

  double get_distance_to_lane_border(std::string direction, double distance,
                                     int f_position) const {
    std::vector<ReferenceLinePointDerived> f_refline_points;
    double border_dist = 1000.;
    if (f_position == 0) {
      f_refline_points = current_refline_points();
    } else if (f_position == -1) {
      f_refline_points = left_refline_points();
    } else if (f_position == 1) {
      f_refline_points = right_refline_points();
    }
    for (auto &p : f_refline_points) {
      if (p.car_point.x > 0. && p.car_point.x < distance + 10.) {
        if (direction == "left") {
          if (p.distance_to_left_lane_border < border_dist) {
            border_dist = p.distance_to_left_lane_border;
          }
        } else if (direction == "right") {
          if (p.distance_to_right_lane_border < border_dist) {
            border_dist = p.distance_to_right_lane_border;
          }
        }
      }
    }
    return border_dist;
  }

  bool has_same_direction_lanes_out() const {
    int num = intsection_lanes_out_num();
    if (is_in_intersection()) {
      if (intersection_.size() < 1) {
        return false;
      } else {
        if (current_lane_out_id() >= 0 && current_lane_out_id() <= num - 1) {
          if (curr_intsect_task() == Direction::TURN_LEFT) {
            if (current_lane_out_id() < num - 1) {
              return ((traffic_light_direction() &
                       intersection_[0]
                           .lanes_out[current_lane_out_id() + 1]
                           .lane_marks.value) ||
                      intersection_[0]
                              .lanes_out[current_lane_out_id() + 1]
                              .lane_type.value == LaneType::NON_MOTOR);
            } else {
              return false;
            }
          } else if (curr_intsect_task() == Direction::TURN_RIGHT) {
            if (current_lane_out_id() > 0) {
              return traffic_light_direction() &
                     intersection_[0]
                         .lanes_out[current_lane_out_id() - 1]
                         .lane_marks.value;
            } else {
              return false;
            }
          } else {
            return ((intersection_[0].lanes_out[0].lane_marks.value &
                     intersection_[0].lanes_out[1].lane_marks.value) &&
                    (traffic_light_direction() &
                     intersection_[0].lanes_out[1].lane_marks.value)) ||
                   ((intersection_[0].lanes_out[num - 1].lane_marks.value &
                     intersection_[0].lanes_out[num - 2].lane_marks.value) &&
                    (traffic_light_direction() &
                     intersection_[0].lanes_out[num - 2].lane_marks.value));
          }
        } else {
          return false;
        }
      }
    } else {
      if (intersection_.size() < 2) {
        return false;
      } else {
        if (num > 1) {
          if (curr_intsect_task() == Direction::TURN_LEFT) {
            return traffic_light_direction() &
                   intersection_[1].lanes_out[1].lane_marks.value;
          } else if (curr_intsect_task() == Direction::TURN_RIGHT) {
            return traffic_light_direction() &
                   intersection_[1].lanes_out[num - 2].lane_marks.value;
          } else {
            return ((intersection_[1].lanes_out[0].lane_marks.value &
                     intersection_[1].lanes_out[1].lane_marks.value) &&
                    (traffic_light_direction() &
                     intersection_[1].lanes_out[1].lane_marks.value)) ||
                   ((intersection_[1].lanes_out[num - 1].lane_marks.value &
                     intersection_[1].lanes_out[num - 2].lane_marks.value) &&
                    (traffic_light_direction() &
                     intersection_[1].lanes_out[num - 2].lane_marks.value));
          }
        } else {
          return false;
        }
      }
    }
  }

  int second_traffic_light_direction() const {
    if (is_on_highway()) {
      return Direction::GO_STRAIGHT;
    }
    if (intersection_.size() < 3) {
      return Direction::UNKNOWN;
    } else {
      return intersection_[2].direction.value;
    }
  }

  uint64_t current_lane_out_marks() const {
    int index = current_lane_out_id();
    if (is_in_intersection()) {
      if (intersection_.size() < 1 || index < 0 ||
          index >= intersection_[0].lanes_out.size()) {
        return 0;
      }
      return intersection_[0].lanes_out[index].lane_marks.value;
    } else {
      if (intersection_.size() < 2 || index < 0 ||
          index >= intersection_[1].lanes_out.size()) {
        return 0;
      }
      return intersection_[1].lanes_out[index].lane_marks.value;
    }
  }

  uint64_t left_lane_out_marks() const {
    int index = left_lane_out_id();
    if (is_in_intersection()) {
      if (intersection_.size() < 1 || index < 0 ||
          index >= intersection_[0].lanes_out.size()) {
        return 0;
      }
      return intersection_[0].lanes_out[index].lane_marks.value;
    } else {
      if (intersection_.size() < 2 || index < 0 ||
          index >= intersection_[1].lanes_out.size()) {
        return 0;
      }
      return intersection_[1].lanes_out[index].lane_marks.value;
    }
  }

  uint64_t right_lane_out_marks() const {
    int index = right_lane_out_id();
    if (is_in_intersection()) {
      if (intersection_.size() < 1 || index < 0 ||
          index >= intersection_[0].lanes_out.size()) {
        return 0;
      }
      return intersection_[0].lanes_out[index].lane_marks.value;
    } else {
      if (intersection_.size() < 2 || index < 0 ||
          index >= intersection_[1].lanes_out.size()) {
        return 0;
      }
      return intersection_[1].lanes_out[index].lane_marks.value;
    }
  }

  uint64_t left_lane_out_type() const {
    int index = left_lane_out_id();
    if (is_in_intersection()) {
      if (intersection_.size() < 1 || index < 0 ||
          index >= intersection_[0].lanes_out.size()) {
        return 0;
      }
      return intersection_[0].lanes_out[index].lane_type.value;
    } else {
      if (intersection_.size() < 2 || index < 0 ||
          index >= intersection_[1].lanes_out.size()) {
        return 0;
      }
      return intersection_[1].lanes_out[index].lane_type.value;
    }
  }

  uint64_t right_lane_out_type() const {
    int index = right_lane_out_id();
    if (is_in_intersection()) {
      if (intersection_.size() < 1) {
        return LaneType::UNKNOWN;
      }
      if (index < 0 || index >= intersection_[0].lanes_out.size()) {
        if (intsection_lanes_out_num() - current_lane_out_id() - 1 <= 0) {
          return 0;
        } else if (current_lane_out_id() >= 0 &&
                   current_lane_out_id() < intersection_[0].lanes_out.size()) {
          index = current_lane_out_id() + 1;
        }
      }
      return intersection_[0].lanes_out[index].lane_type.value;
    } else {
      if (intersection_.size() < 2) {
        return LaneType::UNKNOWN;
      }
      if (index < 0 || index >= intersection_[1].lanes_out.size()) {
        if (intsection_lanes_out_num() - current_lane_out_id() - 1 <= 0) {
          return 0;
        } else if (current_lane_out_id() >= 0 &&
                   current_lane_out_id() < intersection_[1].lanes_out.size()) {
          index = current_lane_out_id() + 1;
        }
      }
      return intersection_[1].lanes_out[index].lane_type.value;
    }
  }

  const std::vector<maf_worldmodel::LaneBoundarySegment> &
  cout_left_boundary_info() const {
    int index = current_lane_out_id();
    if (is_in_intersection()) {
      if (intersection_.size() < 1 || index < 0 ||
          index >= intersection_[0].lanes_out.size()) {
        return default_lane_.left_lane_boundary.segments;
      }
      return intersection_[0].lanes_out[index].left_lane_boundary.segments;
    } else {
      if (intersection_.size() < 2 || index < 0 ||
          index >= intersection_[1].lanes_out.size()) {
        return default_lane_.left_lane_boundary.segments;
      }
      return intersection_[1].lanes_out[index].left_lane_boundary.segments;
    }
  }

  const std::vector<maf_worldmodel::LaneBoundarySegment> &
  cout_right_boundary_info() const {
    int index = current_lane_out_id();
    if (is_in_intersection()) {
      if (intersection_.size() < 1 || index < 0 ||
          index >= intersection_[0].lanes_out.size()) {
        return default_lane_.right_lane_boundary.segments;
      }
      return intersection_[0].lanes_out[index].right_lane_boundary.segments;
    } else {
      if (intersection_.size() < 2 || index < 0 ||
          index >= intersection_[1].lanes_out.size()) {
        return default_lane_.right_lane_boundary.segments;
      }
      return intersection_[1].lanes_out[index].right_lane_boundary.segments;
    }
  }

  const std::vector<maf_worldmodel::LaneBoundarySegment> &
  lout_left_boundary_info() const {
    int index = left_lane_out_id();
    if (is_in_intersection()) {
      if (intersection_.size() < 1 || index < 0 ||
          index >= intersection_[0].lanes_out.size()) {
        return default_lane_.left_lane_boundary.segments;
      }
      return intersection_[0].lanes_out[index].left_lane_boundary.segments;
    } else {
      if (intersection_.size() < 2 || index < 0 ||
          index >= intersection_[1].lanes_out.size()) {
        return default_lane_.left_lane_boundary.segments;
      }
      return intersection_[1].lanes_out[index].left_lane_boundary.segments;
    }
  }

  const std::vector<maf_worldmodel::LaneBoundarySegment> &
  lout_right_boundary_info() const {
    int index = left_lane_out_id();
    if (is_in_intersection()) {
      if (intersection_.size() < 1 || index < 0 ||
          index >= intersection_[0].lanes_out.size()) {
        return default_lane_.right_lane_boundary.segments;
      }
      return intersection_[0].lanes_out[index].right_lane_boundary.segments;
    } else {
      if (intersection_.size() < 2 || index < 0 ||
          index >= intersection_[1].lanes_out.size()) {
        return default_lane_.right_lane_boundary.segments;
      }
      return intersection_[1].lanes_out[index].right_lane_boundary.segments;
    }
  }

  const std::vector<maf_worldmodel::LaneBoundarySegment> &
  rout_left_boundary_info() const {
    int index = right_lane_out_id();
    if (is_in_intersection()) {
      if (intersection_.size() < 1 || index < 0 ||
          index >= intersection_[0].lanes_out.size()) {
        return default_lane_.left_lane_boundary.segments;
      }
      return intersection_[0].lanes_out[index].left_lane_boundary.segments;
    } else {
      if (intersection_.size() < 2 || index < 0 ||
          index >= intersection_[1].lanes_out.size()) {
        return default_lane_.left_lane_boundary.segments;
      }
      return intersection_[1].lanes_out[index].left_lane_boundary.segments;
    }
  }

  const std::vector<maf_worldmodel::LaneBoundarySegment> &
  rout_right_boundary_info() const {
    int index = right_lane_out_id();
    if (is_in_intersection()) {
      if (intersection_.size() < 1 || index < 0 ||
          index >= intersection_[0].lanes_out.size()) {
        return default_lane_.right_lane_boundary.segments;
      }
      return intersection_[0].lanes_out[index].right_lane_boundary.segments;
    } else {
      if (intersection_.size() < 2 || index < 0 ||
          index >= intersection_[1].lanes_out.size()) {
        return default_lane_.right_lane_boundary.segments;
      }
      return intersection_[1].lanes_out[index].right_lane_boundary.segments;
    }
  }

  const std::vector<MapTrafficLightGroup> &traffic_light_groups() const {
    if (is_in_intersection()) {
      if (intersection_.size() < 1) {
        return default_intersection_.traffic_light_groups;
      } else {
        return intersection_[0].traffic_light_groups;
      }
    } else {
      if (intersection_.size() < 2) {
        return default_intersection_.traffic_light_groups;
      } else {
        return intersection_[1].traffic_light_groups;
      }
    }
  }

  const std::vector<int8_t> &current_tasks() const { return current_tasks_; }

  const std::vector<int8_t> &left_lane_tasks() const {
    return left_lane_tasks_;
  }

  const std::vector<int8_t> &right_lane_tasks() const {
    return right_lane_tasks_;
  }

  const std::vector<Interval> &first_task_ranges() const {
    return lane_strategy_.current_lane_change_available_interval;
  }

  bool must_execute_task() const {
    if (current_lane_marks() & traffic_light_direction() ||
        (current_lane_marks() == Direction::UNKNOWN &&
         current_lane_type() == LaneType::NORMAL &&
         std::fabs(lc_map_decision()) <= 1) ||
        traffic_light_direction() == Direction::UNKNOWN) {
      if (!is_in_intersection() && current_lane_out_id() >= 0 &&
          !(current_lane_out_marks() & second_traffic_light_direction())) {
        double l_dash_length = 0.;
        double r_dash_length = 0.;
        for (int i = 0; i < cout_left_boundary_info().size(); i++) {
          if (cout_left_boundary_info()[i].type.value.value !=
              LaneBoundaryForm::SOLID) {
            l_dash_length += cout_left_boundary_info()[i].length;
          } else {
            break;
          }
        }
        for (int i = 0; i < cout_right_boundary_info().size(); i++) {

          if (cout_right_boundary_info()[i].type.value.value !=
              LaneBoundaryForm::SOLID) {
            r_dash_length += cout_right_boundary_info()[i].length;
          } else {
            break;
          }
        }
        if (lc_map_decision() > 0 &&
            ((cout_right_boundary_info().size() > 0 &&
              ((cout_right_boundary_info()[0].type.value.value ==
                    LaneBoundaryForm::DASH &&
                cout_right_boundary_info()[0].length < 30.) ||
               (cout_right_boundary_info()[0].type.value.value ==
                LaneBoundaryForm::SOLID))) ||
             (cout_right_boundary_info().size() == 0))) {
          return true;
        } else if (lc_map_decision() < 0 &&
                   ((cout_left_boundary_info().size() > 0 &&
                     ((cout_left_boundary_info()[0].type.value.value ==
                           LaneBoundaryForm::DASH &&
                       cout_left_boundary_info()[0].length < 30.) ||
                      (cout_left_boundary_info()[0].type.value.value ==
                       LaneBoundaryForm::SOLID))) ||
                    (cout_left_boundary_info().size() == 0))) {
          return true;
        } else {
          return false;
        }
      } else {
        return false;
      }
    } else {
      return true;
    }
  }

  const bool get_curv_from_current_refline_polys(double x, double &curv) const {
    if (current_refline_polys_valid_) {
      // quadratic function
      if (current_refline_polys_.size() != 3) {
        return false;
      }
      x = clip(x, current_refline_polys_max_x_, current_refline_polys_min_x_);
      double dy = 2 * current_refline_polys_[0] * x + current_refline_polys_[1];
      double ddy = 2 * current_refline_polys_[0];
      curv = std::fabs(ddy) / std::pow(1 + dy * dy, 1.5);
      return true;
    } else {
      return false;
    }
  }

  const bool get_curv_from_left_raw_lane_polys(double x, double &curv) const {
    if (left_raw_lane_polys_valid_) {
      // cubic function
      if (left_raw_lane_polys_.size() == 4) {
        x = clip(x, left_raw_lane_polys_max_x_, left_raw_lane_polys_min_x_);
        double dy = 3 * left_raw_lane_polys_[0] * std::pow(x, 2) +
                    2 * left_raw_lane_polys_[1] * x + left_raw_lane_polys_[2];
        double ddy =
            6 * left_raw_lane_polys_[0] * x + 2 * left_raw_lane_polys_[1];
        curv = std::fabs(ddy) / std::pow(1 + dy * dy, 1.5);
        return true;
      } else if (left_raw_lane_polys_.size() == 3) {
        // quadratic function
        x = clip(x, left_raw_lane_polys_max_x_, left_raw_lane_polys_min_x_);
        double dy = 2 * left_raw_lane_polys_[0] * x + left_raw_lane_polys_[1];
        double ddy = 2 * left_raw_lane_polys_[0];
        curv = std::fabs(ddy) / std::pow(1 + dy * dy, 1.5);
        return true;
      } else {
        return false;
      }

    } else {
      return false;
    }
  }

  const bool get_curv_from_right_raw_lane_polys(double x, double &curv) const {
    if (right_raw_lane_polys_valid_) {
      // cubic function
      if (right_raw_lane_polys_.size() == 4) {
        x = clip(x, right_raw_lane_polys_max_x_, right_raw_lane_polys_min_x_);
        double dy = 3 * right_raw_lane_polys_[0] * std::pow(x, 2) +
                    2 * right_raw_lane_polys_[1] * x + right_raw_lane_polys_[2];
        double ddy =
            6 * right_raw_lane_polys_[0] * x + 2 * right_raw_lane_polys_[1];
        curv = std::fabs(ddy) / std::pow(1 + dy * dy, 1.5);
        return true;
      } else if (right_raw_lane_polys_.size() == 3) {
        // quadratic function
        x = clip(x, right_raw_lane_polys_max_x_, right_raw_lane_polys_min_x_);
        double dy = 2 * right_raw_lane_polys_[0] * x + right_raw_lane_polys_[1];
        double ddy = 2 * right_raw_lane_polys_[0];
        curv = std::fabs(ddy) / std::pow(1 + dy * dy, 1.5);
        return true;
      } else {
        return false;
      }

    } else {
      return false;
    }
  }

  const bool get_current_refline_polys(std::vector<double> &polys) const {
    if (current_refline_polys_valid_) {
      polys = current_refline_polys_;
      return true;
    } else {
      return false;
    }
  }

  const bool get_current_refline_polys_max_x(double &max_x) const {
    if (current_refline_polys_valid_) {
      max_x = current_refline_polys_max_x_;
      return true;
    } else {
      return false;
    }
  }

  const bool get_left_raw_lane_polys_min_x(double &min_x) const {
    if (left_raw_lane_polys_valid_) {
      min_x = left_raw_lane_polys_min_x_;
      return true;
    } else {
      return false;
    }
  }

  const bool get_left_raw_lane_polys_max_x(double &max_x) const {
    if (left_raw_lane_polys_valid_) {
      max_x = left_raw_lane_polys_max_x_;
      return true;
    } else {
      return false;
    }
  }

  const bool get_right_raw_lane_polys_min_x(double &min_x) const {
    if (right_raw_lane_polys_valid_) {
      min_x = right_raw_lane_polys_min_x_;
      return true;
    } else {
      return false;
    }
  }

  const bool get_right_raw_lane_polys_max_x(double &max_x) const {
    if (right_raw_lane_polys_valid_) {
      max_x = right_raw_lane_polys_max_x_;
      return true;
    } else {
      return false;
    }
  }

  const std::vector<ReferenceLinePointDerived> &current_refline_points() const {
    return current_refline_points_;
  }

  const std::vector<ReferenceLinePointDerived> &left_refline_points() const {
    return left_refline_points_;
  }

  const std::vector<ReferenceLinePointDerived> &right_refline_points() const {
    return right_refline_points_;
  }

  const std::vector<ReferenceLinePointDerived> &
  left_left_refline_points() const {
    return left_left_refline_points_;
  }

  const std::vector<ReferenceLinePointDerived> &
  right_right_refline_points() const {
    return right_right_refline_points_;
  }

  const std::vector<ReferenceLineSegment> &current_refline_segments() const {
    int index = current_lane_index();
    if (index < 0 || index >= lane_.size()) {
      return default_lane_.reference_line.reference_line_segments;
    } else {
      return lane_[index].reference_line.reference_line_segments;
    }
  }

  const std::vector<ReferenceLineSegment> &left_refline_segments() const {
    int index = current_lane_index() - 1;
    if (index < 0 || index >= lane_.size()) {
      return default_lane_.reference_line.reference_line_segments;
    } else {
      return lane_[index].reference_line.reference_line_segments;
    }
  }

  const std::vector<ReferenceLineSegment> &right_refline_segments() const {
    int index = current_lane_index() + 1;
    if (index < 0 || index >= lane_.size()) {
      return default_lane_.reference_line.reference_line_segments;
    } else {
      return lane_[index].reference_line.reference_line_segments;
    }
  }

  const std::vector<ReferenceLineSegment> &left_left_refline_segments() const {
    int index = current_lane_index() - 2;
    if (index < 0 || index >= lane_.size()) {
      return default_lane_.reference_line.reference_line_segments;
    } else {
      return lane_[index].reference_line.reference_line_segments;
    }
  }

  const std::vector<ReferenceLineSegment> &
  right_right_refline_segments() const {
    int index = current_lane_index() + 2;
    if (index < 0 || index >= lane_.size()) {
      return default_lane_.reference_line.reference_line_segments;
    } else {
      return lane_[index].reference_line.reference_line_segments;
    }
  }

  const std::vector<maf_worldmodel::LaneBoundarySegment> &
  left_boundary_info() const {
    int index = current_lane_index();
    if (index < 0 || index >= lane_.size()) {
      return default_lane_.left_lane_boundary.segments;
    } else {
      return lane_[index].left_lane_boundary.segments;
    }
  }

  const std::vector<maf_worldmodel::LaneBoundarySegment> &
  right_boundary_info() const {
    int index = current_lane_index();
    if (index < 0 || index >= lane_.size()) {
      return default_lane_.right_lane_boundary.segments;
    } else {
      return lane_[index].right_lane_boundary.segments;
    }
  }

  const std::vector<LaneBoundaryPolylineDerived> &
  lane_boundary_polyline() const {
    return lane_boundary_polyline_;
  }

  const std::vector<MapPOIInfoData> &map_pois_data() const {
    return map_poi_info_;
  }

  const uint8_t lane_type(int index) const {
    if (index < 0 || index >= lanes_num()) {
      return LaneType::UNKNOWN;
    }

    return lane_[index].lane_type.value;
  }

  const uint8_t current_lane_type() const {
    int index = current_lane_index();
    if (index < 0 || index >= lane_.size()) {
      return LaneType::UNKNOWN;
    }

    return lane_[index].lane_type.value;
  }

  const std::vector<LaneMergingSplittingPointData> &
  merge_split_point_data() const {
    return lane_merging_splitting_point_;
  }

  void extend_refline_points_for_ddmap(
      int relative_id, std::vector<ReferenceLinePointDerived> &refline_points,
      const std::shared_ptr<WorldModel> &world_model);

  void polyfit_raw_vision_lane_for_ddmap(
      const std::shared_ptr<WorldModel> &world_model);

  void
  calculate_reflines_by_c_poly(const Lane &c_lane, const Lane &l_lane,
                               const Lane &r_lane,
                               const std::shared_ptr<WorldModel> &world_model);

  // int8_t current_lane_bias() const {
  //   int index = current_lane_index();
  //   if (index < 0 || index >= lane_.size()) {
  //     return MSD_LANE_BIAS_NONE;
  //   } else {
  //     return lane_[index].lane_bias;
  //   }
  // }

  // int8_t left_lane_bias() const {
  //   int index = current_lane_index() - 1;
  //   if (index < 0 || index >= lane_.size()) {
  //     return MSD_LANE_BIAS_NONE;
  //   } else {
  //     return lane_[index].lane_bias;
  //   }
  // }

  // int8_t right_lane_bias() const {
  //   int index = current_lane_index() + 1;
  //   if (index < 0 || index >= lane_.size()) {
  //     return MSD_LANE_BIAS_NONE;
  //   } else {
  //     return lane_[index].lane_bias;
  //   }
  // }

  bool get_pi_target_pt_exist() const { return pi_target_pt_exist_; }
  std::vector<double> get_pi_target_pt_enu() const { return pi_target_pt_enu_; }
  std::vector<double> get_pi_target_pt_car() const { return pi_target_pt_car_; }

  std::vector<LaneData> lane_{};
  std::vector<MapPOIInfoData> map_poi_info_{};
  std::vector<IntersectionData> intersection_{};
  SelfPositionData self_position_{};
  LaneStrategyData lane_strategy_{};
  std::vector<LaneMergingSplittingPointData> lane_merging_splitting_point_{};
  std::string scene_type_{""};
  std::vector<double> dis_to_ramp_{};
  std::vector<double> ramp_length_{};
  bool pi_target_pt_exist_{false};

private:
  void update_lane_boundary_segments();
  void update_lane_tasks();
  void update_speed_limit(double ego_vel, double ego_v_cruise, bool is_ddmap);
  void update_refline_points(const std::shared_ptr<WorldModel> &world_model);
  void update_vision_lanes();
  void update_lane_boundary_polyline();

  LaneData default_lane_{};
  IntersectionData default_intersection_{};
  IntersectionData current_intersection_{};
  std::vector<int8_t> current_tasks_{};
  std::vector<int8_t> left_lane_tasks_{};
  std::vector<int8_t> right_lane_tasks_{};
  bool current_refline_polys_valid_{false};
  std::vector<double> current_refline_polys_{};
  double current_refline_polys_min_x_{0.0};
  double current_refline_polys_max_x_{0.0};

  // for raw vision lane
  bool left_raw_lane_polys_valid_{false};
  std::vector<double> left_raw_lane_polys_{};
  double left_raw_lane_polys_min_x_{0.0};
  double left_raw_lane_polys_max_x_{0.0};

  bool right_raw_lane_polys_valid_{false};
  std::vector<double> right_raw_lane_polys_{};
  double right_raw_lane_polys_min_x_{0.0};
  double right_raw_lane_polys_max_x_{0.0};

  std::vector<ReferenceLinePointDerived> current_refline_points_{};
  std::vector<ReferenceLinePointDerived> left_refline_points_{};
  std::vector<ReferenceLinePointDerived> right_refline_points_{};
  std::vector<ReferenceLinePointDerived> left_left_refline_points_{};
  std::vector<ReferenceLinePointDerived> right_right_refline_points_{};
  std::vector<LaneBoundaryPolylineDerived> lane_boundary_polyline_{};
  SpeedChangePoint speed_change_point_{};
  double current_lane_speed_limit_{0.0};
  double v_cruise_{0.0};

  bool hack_ = false;
  double new_lc_end_dis_ = DBL_MAX;
  std::vector<double> pi_target_pt_enu_;
  std::vector<double> pi_target_pt_car_;
};

class MapInfoManager {
public:
  MapInfoManager();

  bool update(const std::shared_ptr<WorldModel> &world_model);

  const MSDMapInfo &get_map_info() const { return map_info_; }
  MSDMapInfo &get_mutable_map_info() { return map_info_; }
  void set_map_info(const MSDMapInfo &map_info,
                    const std::shared_ptr<WorldModel> &world_model,
                    double ego_vel, double ego_v_cruise, bool is_ddmap);

  void get_lane_change_point(const std::shared_ptr<WorldModel> &world_model);

  bool has_lane(int index) const;
  bool is_solid_line(int side) const;

  static void frame_trans_from_car2enu(
      std::vector<ReferenceLinePointDerived> &ref_trajectory,
      const Transform &car2enu);

  const std::vector<ReferenceLinePointDerived> &ref_trajectory() const {
    return ref_trajectory_;
  }

  const std::unordered_map<int, std::vector<ReferenceLinePointDerived>> &
  ref_trajectories() const {
    return ref_trajectories_;
  }

  const std::unordered_map<int, std::vector<ReferenceLinePointDerived>> &
  ref_trajectories_bias() const {
    return ref_trajectories_bias_;
  }

  double traffic_light_stop_point() const { return traffic_light_stop_point_; }

  bool traffic_light_stop_flag() const { return traffic_light_stop_flag_; }

  int traffic_light_stop_reason() const { return traffic_light_stop_reason_; }

  bool traffic_light_faster_pass() const { return traffic_light_faster_pass_; }

  double distance_to_lane_change_point() const {
    return distance_to_lane_change_point_;
  }

  double get_distance_to_dash_line(std::string direction) const;

  double get_distance_to_laneout_dash_line(std::string lane,
                                           std::string direction) const;
  double get_distance_to_final_dash_line(std::string direction) const;

  double get_distance_to_stop_line() const;
  double get_distance_to_road_border(std::string direction, double distance,
                                     int f_position) const;
  double get_distance_to_lane_border(std::string direction, double distance,
                                     int f_position) const;

  bool is_refline_smooth() { return is_refline_smooth_flag_; };

  std::string lane_bias() { return lane_bias_; }

  // int8_t get_fix_lane_bias() const {
  //   if (flane_.position() == 0) {
  //     return map_info_.current_lane_bias();
  //   } else if (flane_.position() == -1) {
  //     return map_info_.left_lane_bias();
  //   } else if (flane_.position() == 1) {
  //     return map_info_.right_lane_bias();
  //   }
  //   return MSD_LANE_BIAS_NONE;
  // }

private:
  void
  linear_interpolation(std::vector<ReferenceLinePointDerived> &ref_trajectory);
  void set_range_of_trajectory(
      std::vector<ReferenceLinePointDerived> &ref_trajectory);
  bool
  construct_ref_trajectories(const std::shared_ptr<WorldModel> &world_model);
  void Judge_smooth_flags(
      const int lane_id, const std::vector<ReferenceLinePointDerived> &ref_line,
      bool &Flags_smooth, std::vector<ReferenceLinePointDerived> &segment_line);
  bool
  splice_smooth_seg(int lane_id,
                    std::vector<ReferenceLinePointDerived> &smooth_segment_line,
                    std::vector<ReferenceLinePointDerived> &cur_trajectories);

  // add environment function
  bool update_raw_reflines(const std::shared_ptr<WorldModel> &world_model);
  bool update_lanes(bool is_ddmap);

  MSDMapInfo map_info_;

  std::string lane_bias_ = "none";
  std::vector<ReferenceLinePointDerived> ref_trajectory_;
  std::unordered_map<int, std::vector<ReferenceLinePointDerived>>
      ref_trajectories_;
  std::unordered_map<int, std::vector<ReferenceLinePointDerived>>
      ref_trajectories_bias_;

  std::unique_ptr<msquare::ReferenceLineProvider> refline_provider_;
  std::unordered_map<int, std::vector<ReferenceLinePointDerived>>
      ref_trajectories_smooth_; // Cache smooth points;
  std::unordered_map<int, std::vector<ReferenceLinePointDerived>>
      theta_deviant_point_; // deviant theta point
  std::unordered_map<int, ReferenceLinePointDerived>
      cur_frame_car_pos_; // car position
  int solver_status_;     // smoother solver status  -2 : max iteration
  std::unordered_map<int, ReferenceLinePointDerived> start_smooth_point_;
  std::unordered_map<int, std::unordered_set<int32_t>> last_point_ids_;

  double traffic_light_stop_point_{DBL_MAX};
  double distance_to_lane_change_point_{DBL_MAX};
  bool traffic_light_stop_flag_{false};
  bool is_refline_smooth_flag_{false};
  int traffic_light_stop_reason_{4};
  bool traffic_light_faster_pass_{false};

public:
  // TODO(shike): rename member variables and add public get function
  bool mmp_update_{false};
  double max_speed_{40.0};

  RawRefLine c_raw_refline_{RefLinePosition::CURR_REFLINE};
  RawRefLine l_raw_refline_{RefLinePosition::LEFT_REFLINE};
  RawRefLine r_raw_refline_{RefLinePosition::RIGHT_REFLINE};
  RawRefLine ll_raw_refline_{RefLinePosition::LEFT_LEFT_REFLINE};
  RawRefLine rr_raw_refline_{RefLinePosition::RIGHT_RIGHT_REFLINE};

  Lane clane_{LanePosition::CURR_POS, &c_raw_refline_};
  Lane llane_{LanePosition::LEFT_POS, &l_raw_refline_};
  Lane rlane_{LanePosition::RIGHT_POS, &r_raw_refline_};
  Lane rrlane_{LanePosition::RIGHT_RIGHT_POS, &rr_raw_refline_};
  Lane lllane_{LanePosition::LEFT_LEFT_POS, &ll_raw_refline_};
};

} // namespace msquare

#endif
