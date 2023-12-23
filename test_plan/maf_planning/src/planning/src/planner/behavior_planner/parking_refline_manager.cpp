#include "common/math/linear_interpolation.h"
#include "common/parking_world_model.h"

namespace msquare {

namespace parking {

// RefLineManager::RefLineManager(const std::shared_ptr<WorldModel>
// &world_model) {
//   mph_assert(world_model != nullptr);
//   world_model_ = world_model;
//   via_point_decider_cfg_ = ViaPointDeciderConfig::Instance();
// }

// RefLineManager::~RefLineManager(){};
void RefLineManager::init(const std::shared_ptr<WorldModel> &world_model) {
  mph_assert(world_model != nullptr);
  world_model_ = world_model;
  via_point_decider_cfg_ = ViaPointDeciderConfig::Instance();
}

void RefLineManager::update() {

  refline_attribute_lane_last_ = refline_attribute_lane_;
  refline_attribute_point_last_ = refline_attribute_point_;
  refline_attribute_lane_.clear();
  refline_attribute_point_.clear();

  frenet_coord_ = world_model_->get_frenet_coord();
  if (!frenet_coord_)
    return;

  // cal refline attribute point
  if (!cal_refline_attribute_point(refline_attribute_point_)) {
    // std::cout << "cal_refline_attribute_point failed" << std::endl;
    return;
  }
  // resize
  if (!resize_attribute_point(refline_attribute_point_)) {
    // std::cout << "resize_attribute_point failed" << std::endl;
    return;
  }
  // transform data
  if (!transform()) {
    // std::cout << "transform failed" << std::endl;
    return;
  }
  // lane divide
  for (int i = 0; i < refline_attribute_point_.size(); i++) {
    int id_lane = refline_attribute_point_[i].id_lane;
    if (!refline_attribute_lane_.empty() &&
        refline_attribute_lane_.back().id_lane != id_lane) {
      refline_attribute_lane_.back().length_lane =
          refline_attribute_point_[i].s -
          refline_attribute_lane_.back().s_lane_start;
      refline_attribute_lane_.back().count_lane_points =
          i - refline_attribute_lane_.back().lane_start_index;
    }
    if (refline_attribute_lane_.empty() ||
        refline_attribute_lane_.back().id_lane != id_lane) {
      RefLineAttributeLane new_lane;
      new_lane.id_lane = id_lane;
      new_lane.lane_start_index = i;
      new_lane.s_lane_start = refline_attribute_point_[i].s;
      new_lane.via_weight_base = via_point_decider_cfg_->WEIGHT_VIA_BASE_LANE_;
      refline_attribute_lane_.emplace_back(new_lane);
    }
  }

  if (!refline_attribute_point_.empty()) {
    refline_attribute_lane_.back().length_lane =
        refline_attribute_point_.back().s -
        refline_attribute_lane_.back().s_lane_start;
    refline_attribute_lane_.back().count_lane_points =
        refline_attribute_point_.size() - 1 -
        refline_attribute_lane_.back().lane_start_index;
  }

  // curvature_theta
  for (size_t i = 0; i < refline_attribute_point_.size() - 1; i++) {
    refline_attribute_point_[i].curvature_theta =
        1.0 / std::abs(std::hypot(refline_attribute_point_[i + 1].y -
                                      refline_attribute_point_[i].y,
                                  refline_attribute_point_[i + 1].x -
                                      refline_attribute_point_[i].x) /
                       planning_math::NormalizeAngle(
                           refline_attribute_point_[i + 1].theta -
                           refline_attribute_point_[i].theta));
  }
  refline_attribute_point_.back().curvature_theta =
      refline_attribute_point_[refline_attribute_point_.size() - 2]
          .curvature_theta;

  // for more attribute
  int start_same_index = 0;
  for (int i = 0; i < refline_attribute_lane_last_.size(); i++) {
    if (refline_attribute_lane_last_[i].id_lane ==
        refline_attribute_lane_.front().id_lane) {
      start_same_index = i;
      break;
    }
  }

  for (int i = start_same_index; i < refline_attribute_lane_last_.size();
       i++) { // add -1
    if (refline_attribute_lane_last_[i].id_lane !=
        refline_attribute_lane_[i - start_same_index].id_lane)
      break;
    refline_attribute_lane_[i - start_same_index].lane_attribute =
        refline_attribute_lane_last_[i].lane_attribute;
    refline_attribute_lane_[i - start_same_index].avg_curvature =
        refline_attribute_lane_last_[i].avg_curvature;
    refline_attribute_lane_[i - start_same_index].length_lane =
        std::max(refline_attribute_lane_[i - start_same_index].length_lane,
                 refline_attribute_lane_last_[i].length_lane);
    refline_attribute_lane_[i - start_same_index].new_info = false;
    // weight base
    if ((refline_attribute_lane_[i - start_same_index].lane_attribute &
         (int)LaneAttribute::INTERSECTION) == 0)
      refline_attribute_lane_[i - start_same_index].via_weight_base =
          std::min(via_point_decider_cfg_->WEIGHT_VIA_BASE_LANE_,
                   refline_attribute_lane_last_[i].via_weight_base);
    else
      refline_attribute_lane_[i - start_same_index].via_weight_base =
          std::min(via_point_decider_cfg_->WEIGHT_VIA_BASE_INTERSECTION_,
                   refline_attribute_lane_last_[i].via_weight_base);
  }

  for (int i = 0; i < refline_attribute_lane_.size(); i++) {
    if (refline_attribute_lane_[i].new_info == false &&
        (refline_attribute_lane_[i].lane_attribute &
         (int)LaneAttribute::UNKNOWN) == 0) {
      refline_attribute_lane_[i].new_info = false;
      // std::cout << "id_old: " << ref_info_index_.id_lane[i] << std::endl;
      continue;
    }
    refline_attribute_lane_[i].new_info = true;
    double theta_diff = 0.0;
    refline_attribute_lane_[i].lane_attribute = 0;

    if (i == refline_attribute_lane_.size() - 1) {
      if ((refline_attribute_lane_[i].lane_attribute &
           (int)LaneAttribute::UNKNOWN) == 0)
        refline_attribute_lane_[i].lane_attribute |=
            (int)LaneAttribute::UNKNOWN;

      theta_diff = planning_math::NormalizeAngle(
          frenet_coord_->GetRefCurveHeading(
              std::min(frenet_coord_->GetLength(),
                       refline_attribute_lane_[i].s_lane_start +
                           refline_attribute_lane_[i].length_lane)) -
          frenet_coord_->GetRefCurveHeading(
              std::min(refline_attribute_lane_[i].s_lane_start,
                       frenet_coord_->GetLength())));
    } else {
      // std::cout << "attribute: " << i << " " << ref_info_index_.id_lane[i] <<
      // " " << frenet_coord_->GetRefCurveHeading(ref_info_index_.s_lane_start[i
      // + 1])
      //         << " " <<
      //         frenet_coord_->GetRefCurveHeading(ref_info_index_.s_lane_start[i])
      //         << " " << ref_info_index_.s_lane_start[i + 1]
      //         << " " << ref_info_index_.s_lane_start[i]
      //         << std::endl;
      if ((refline_attribute_lane_[i].lane_attribute &
           (int)LaneAttribute::UNKNOWN) != 0)
        refline_attribute_lane_[i].lane_attribute ^=
            (int)LaneAttribute::UNKNOWN;

      theta_diff = planning_math::NormalizeAngle(
          frenet_coord_->GetRefCurveHeading(
              std::min(refline_attribute_lane_[i + 1].s_lane_start,
                       frenet_coord_->GetLength())) -
          frenet_coord_->GetRefCurveHeading(
              std::min(refline_attribute_lane_[i].s_lane_start,
                       frenet_coord_->GetLength())));
    }
    // for attribute must have complete segment
    // turn
    if (theta_diff > via_point_decider_cfg_->curve_theta_check_limit_)
      refline_attribute_lane_[i].lane_attribute |=
          (int)LaneAttribute::TURN_LEFT;
    else if (theta_diff < -via_point_decider_cfg_->curve_theta_check_limit_)
      refline_attribute_lane_[i].lane_attribute |=
          (int)LaneAttribute::TURN_RIGHT;
    else
      refline_attribute_lane_[i].lane_attribute |= (int)LaneAttribute::STRAIGHT;
    // length
    if (refline_attribute_lane_[i].length_lane >
        via_point_decider_cfg_->long_lane_min_)
      refline_attribute_lane_[i].lane_attribute |= (int)LaneAttribute::LONG;
    else
      refline_attribute_lane_[i].lane_attribute |= (int)LaneAttribute::SHORT;
    // average curvature
    if (std::fabs(refline_attribute_lane_[i].length_lane) < 1e-5 && i > 0)
      refline_attribute_lane_[i].avg_curvature =
          refline_attribute_lane_[i - 1].avg_curvature;
    else
      refline_attribute_lane_[i].avg_curvature =
          1.0 / std::abs(refline_attribute_lane_[i].length_lane / theta_diff);
    double right_road_border_dist = 0.0;
    double left_road_border_dist = 0.0;
    double right_lane_border_dist = 0.0;
    double left_lane_border_dist = 0.0;
    bool bintersection = false;
    for (int j = refline_attribute_lane_[i].lane_start_index;
         j < (refline_attribute_lane_[i].lane_start_index +
              refline_attribute_lane_[i].count_lane_points - 1);
         j++) {
      // single && carriage way
      right_road_border_dist +=
          refline_attribute_point_[j].right_road_border_distance;
      left_road_border_dist +=
          refline_attribute_point_[j].left_road_border_distance;
      right_lane_border_dist +=
          refline_attribute_point_[j].right_lane_border_distance;
      left_lane_border_dist +=
          refline_attribute_point_[j].left_lane_border_distance;
      // intersection
      bintersection |= refline_attribute_point_[j].intersection;
    }
    right_road_border_dist /= refline_attribute_lane_[i].count_lane_points;
    left_road_border_dist /= refline_attribute_lane_[i].count_lane_points;
    right_lane_border_dist /= refline_attribute_lane_[i].count_lane_points;
    left_lane_border_dist /= refline_attribute_lane_[i].count_lane_points;
    if (bintersection)
      refline_attribute_lane_[i].lane_attribute |=
          (int)LaneAttribute::INTERSECTION;

    if (std::abs(left_lane_border_dist - left_road_border_dist) >
            via_point_decider_cfg_->double_lane_border_distance_ &&
        std::abs((left_lane_border_dist + right_lane_border_dist) -
                 (left_road_border_dist - left_lane_border_dist)) <
            (via_point_decider_cfg_->double_lane_border_distance_ * 2.0))
      refline_attribute_lane_[i].lane_attribute |= (int)LaneAttribute::DOUBLE;
    else
      refline_attribute_lane_[i].lane_attribute |= (int)LaneAttribute::SINGLE;
    if (!bintersection)
      refline_attribute_lane_[i].via_weight_base = std::min(
          WEIGHT_VIA_BASE_LANE, refline_attribute_lane_[i].via_weight_base);
    else
      refline_attribute_lane_[i].via_weight_base =
          std::min(WEIGHT_VIA_BASE_INTERSECTION,
                   refline_attribute_lane_[i].via_weight_base);
  }
  // via point
  for (int i = 0; i < refline_attribute_lane_.size(); i++) {
    // reset attribute
    if (!refline_attribute_lane_[i].new_info) {
      if ((refline_attribute_lane_[i].lane_attribute &
           (int)LaneAttribute::VIA) != 0)
        refline_attribute_lane_[i].lane_attribute ^= (int)LaneAttribute::VIA;
    }

    if (((refline_attribute_lane_[std::max(0, i - 1)].lane_attribute &
          (int)LaneAttribute::DOUBLE) != 0) &&
        ((refline_attribute_lane_
              [std::min((int)refline_attribute_lane_.size() - 1, i + 1)]
                  .lane_attribute &
          (int)LaneAttribute::DOUBLE) != 0)) {
      if ((refline_attribute_lane_[i].lane_attribute &
           (int)LaneAttribute::TURN_LEFT) != 0)
        refline_attribute_lane_[i].lane_attribute |= (int)LaneAttribute::VIA;
    }
    if (((refline_attribute_lane_[i].lane_attribute &
          (int)LaneAttribute::INTERSECTION) != 0) &&
        ((refline_attribute_lane_[i].lane_attribute &
          (int)LaneAttribute::STRAIGHT) == 0) &&
        (refline_attribute_lane_[i].avg_curvature >
         via_point_decider_cfg_->avg_curvature_limit_)) {
      if (i != 0 &&
          (straight_lane_length_before(i) <
               via_point_decider_cfg_->straight_lane_length_before_limit_ &&
           (refline_attribute_lane_[std::max(0, i - 1)].lane_attribute &
            (int)LaneAttribute::INTERSECTION) == 0)) {
        // std::cout << "1" << std::endl;
        // continue;
        // ref_info_index_.lane_attribute[i] -= (int)LaneAttribute::VIA;
      } else if ((i != refline_attribute_lane_.size() - 1) &&
                 (straight_lane_length_after(i) <
                      via_point_decider_cfg_
                          ->straight_lane_length_after_limit_ &&
                  (refline_attribute_lane_
                       [std::min(int(refline_attribute_lane_.size() - 1),
                                 i + 1)]
                           .lane_attribute &
                   (int)LaneAttribute::INTERSECTION) == 0)) {
        // std::cout << "2" << std::endl;
      }
      //       else if(((ref_info_index_.lane_attribute[std::max(0, i - 1)] &
      //       (int)LaneAttribute::DOUBLE) != 0)
      //           &&
      //           ((ref_info_index_.lane_attribute[std::min((int)ref_info_index_.id_lane.size()
      //           - 1, i + 1)] & (int)LaneAttribute::DOUBLE) != 0)){
      // // std::cout << "3" << std::endl;
      //         if((ref_info_index_.lane_attribute[i] &
      //         (int)LaneAttribute::TURN_LEFT) != 0 )
      //           ref_info_index_.lane_attribute[i] += (int)LaneAttribute::VIA;
      //       }
      else {
        auto start_point = refline_attribute_point_[std::max(
            0, refline_attribute_lane_[i].lane_start_index - 1)];
        auto end_point = refline_attribute_point_[std::max(
            0, refline_attribute_lane_[i].lane_start_index +
                   refline_attribute_lane_[i].count_lane_points - 1)];
        double value = -(start_point.left_road_border_distance -
                         start_point.right_road_border_distance) /
                       2.0;
        -(end_point.left_road_border_distance -
          end_point.right_road_border_distance) /
            2.0;
        // std::cout << "value: " << value << std::endl;
        // std::cout << "start_point.left_road_border_distance: " <<
        // start_point.left_road_border_distance << std::endl; std::cout <<
        // "start_point.right_road_border_distance: " <<
        // start_point.right_road_border_distance << std::endl;
        double road_width_start = start_point.lane_width;
        double road_width_end = end_point.lane_width;

        // std::cout << "width: " << s_start << " " << s_end << " " <<
        // road_width_start << " " << road_width_end
        // << " " << value
        // << std::endl;

        if ((road_width_start >
                 via_point_decider_cfg_->road_width_middle_min_ &&
             road_width_start <=
                 via_point_decider_cfg_->road_width_middle_max_) ||
            (road_width_end > via_point_decider_cfg_->road_width_middle_min_ &&
             road_width_end <=
                 via_point_decider_cfg_->road_width_middle_max_) &&
                (refline_attribute_lane_[i].lane_attribute &
                 (int)LaneAttribute::TURN_LEFT) != 0) {
          if (value < via_point_decider_cfg_->turn_width_diff_limit_)
            refline_attribute_lane_[i].lane_attribute |=
                (int)LaneAttribute::VIA;
        } else if (road_width_start >
                       via_point_decider_cfg_->road_width_middle_max_ ||
                   road_width_end >
                       via_point_decider_cfg_->road_width_middle_max_) {
          refline_attribute_lane_[i].lane_attribute |= (int)LaneAttribute::VIA;
        }
      }
    }
  }
  // for carriage way via point
  for (int i = 1; i < refline_attribute_lane_.size() - 1; i++) {
    if (((refline_attribute_lane_[i].lane_attribute &
          (int)LaneAttribute::INTERSECTION) == 0) &&
        ((refline_attribute_lane_[i].lane_attribute &
          (int)LaneAttribute::STRAIGHT) != 0) &&
        ((refline_attribute_lane_[i].lane_attribute &
          (int)LaneAttribute::DOUBLE) != 0) &&
        ((refline_attribute_lane_[std::max(i - 1, 0)].lane_attribute &
          (int)LaneAttribute::STRAIGHT) == 0) &&
        ((refline_attribute_lane_[i + 1].lane_attribute &
          (int)LaneAttribute::STRAIGHT) == 0)) {
      if ((refline_attribute_lane_[i].lane_attribute &
           (int)LaneAttribute::VIA) == 0 &&
          (refline_attribute_lane_[i].length_lane <
               via_point_decider_cfg_->double_lane_length_middle_max_ &&
           refline_attribute_lane_[i].length_lane >
               via_point_decider_cfg_->double_lane_length_middle_min_)) {
        refline_attribute_lane_[i].lane_attribute |= (int)LaneAttribute::VIA;
        refline_attribute_lane_[i].via_weight_base =
            std::min(WEIGHT_VIA_BASE_INTERSECTION,
                     refline_attribute_lane_[i].via_weight_base);
      }
    }
  }

  for (int i = 0; i < refline_attribute_lane_.size(); i++) {
    if ((refline_attribute_lane_[i].lane_attribute & (int)LaneAttribute::VIA) !=
        0) {
      int end_count = refline_attribute_lane_[i].lane_start_index +
                      refline_attribute_lane_[i].count_lane_points;
      // if((ref_info_index_.lane_attribute[i] &
      // (int)LaneAttribute::INTERSECTION) != 0 )
      //   end_count = ref_info_index_.lane_start_index[i] + (end_count -
      //   ref_info_index_.lane_start_index[i]) / 3 * 2;
      for (int j = refline_attribute_lane_[i].lane_start_index; j < end_count;
           j++) {
        refline_attribute_point_[j].via_point = true;
      }
    } else {
      if ((refline_attribute_lane_[i].lane_attribute &
           (int)LaneAttribute::INTERSECTION) != 0) {
        if ((refline_attribute_lane_[i].lane_attribute &
             (int)LaneAttribute::STRAIGHT) != 0) {
          if (refline_attribute_lane_[i].length_lane >
              via_point_decider_cfg_->long_lane_min_) {
            for (int j = refline_attribute_lane_[i].lane_start_index;
                 j < refline_attribute_lane_[i].lane_start_index +
                         refline_attribute_lane_[i].count_lane_points;
                 j++) {
              if (refline_attribute_point_[j].curvature_theta <
                      via_point_decider_cfg_->curve_lane_point_curv_max_ &&
                  refline_attribute_point_[j].curvature_theta >
                      via_point_decider_cfg_->curve_lane_point_curv_mmin_)
                refline_attribute_point_[j].via_point = true;
            }
          }
        }
      }
      if ((refline_attribute_lane_[i].lane_attribute &
           (int)LaneAttribute::INTERSECTION) == 0) {
        for (int j = refline_attribute_lane_[i].lane_start_index;
             j < refline_attribute_lane_[i].lane_start_index +
                     refline_attribute_lane_[i].count_lane_points;
             j++) {
          // if(ref_trajectory_info_.curvature_theta[j] < 0.15 &&
          // ref_trajectory_info_.curvature_theta[j] > 0.03)
          //   ref_trajectory_info_.via_point[j] = true;
          if (refline_attribute_point_[j].curvature_theta >
              via_point_decider_cfg_->curve_lane_point_curv_mmin_) {
            refline_attribute_point_[j].via_point = check_turn(j, i);
            // std::cout << "via_point: " <<
            // ref_trajectory_info_.curvature_theta[j] << " " <<
            // ref_trajectory_info_.curvature[i]
          }
        }
      }
    }
  }

  double via_s = 0.0;
  bool lock = false;
  for (int i = 0; i < refline_attribute_point_.size(); i++) {
    if (refline_attribute_point_[i].via_point &&
        i < (refline_attribute_point_.size() - 2) && i > 0) {
      if ((refline_attribute_point_[i + 1].s - refline_attribute_point_[i].s) <
              via_point_decider_cfg_->ref_trajectory_info_via_delta_s_filter_ &&
          via_s <
              via_point_decider_cfg_->ref_trajectory_info_via_delta_s_filter_ &&
          !refline_attribute_point_[i + 1].via_point && !lock) {
        via_s +=
            refline_attribute_point_[i + 1].s - refline_attribute_point_[i].s;
        refline_attribute_point_[i + 1].via_point = true;
        if ((via_s + refline_attribute_point_[i + 2].s -
             refline_attribute_point_[i + 1].s) >
            via_point_decider_cfg_->ref_trajectory_info_via_delta_s_filter_) {
          via_s = 0.0;
          lock = true;
        }
      } else {
        if (refline_attribute_point_[i + 1].via_point && !lock) {
          via_s +=
              refline_attribute_point_[i + 1].s - refline_attribute_point_[i].s;
        } else {
          via_s = 0.0;
          lock = false;
        }
      }
    }
  }

  // virtual box
  for (int i = 0; i < refline_attribute_lane_.size(); i++) {
    if ((refline_attribute_lane_[i].lane_attribute &
         (int)LaneAttribute::VIRTUAL_BOX) != 0)
      refline_attribute_lane_[i].lane_attribute ^=
          (int)LaneAttribute::VIRTUAL_BOX;
    // std::cout << "attribute: " << i << " " << ref_info_index_.id_lane[i] << "
    // " << ref_info_index_.avg_curvature[i]
    //           << " " << ref_info_index_.length_lane[i]
    //           << " " << (ref_info_index_.lane_attribute[i] &
    //           (int)LaneAttribute::INTERSECTION)
    //           << " " << (ref_info_index_.lane_attribute[i] &
    //           (int)LaneAttribute::STRAIGHT)
    //           << " " << ref_info_index_.s_lane_start[i]
    //           << " " << ref_info_index_.avg_curvature[i]
    //           << std::endl;
    if (((refline_attribute_lane_[i].lane_attribute &
          (int)LaneAttribute::INTERSECTION) != 0) &&
        ((refline_attribute_lane_[i].lane_attribute &
          (int)LaneAttribute::STRAIGHT) == 0) &&
        (refline_attribute_lane_[i].avg_curvature >
         via_point_decider_cfg_->avg_curvature_limit_)) {
      refline_attribute_lane_[i].lane_attribute |=
          (int)LaneAttribute::VIRTUAL_BOX;
    }
  }

  for (int i = 0; i < refline_attribute_lane_.size(); i++) {
    if ((refline_attribute_lane_[i].lane_attribute &
         (int)LaneAttribute::VIRTUAL_BOX) != 0) {
      int end_count = refline_attribute_lane_[i].lane_start_index +
                      refline_attribute_lane_[i].count_lane_points;
      if ((refline_attribute_lane_[i].lane_attribute &
           (int)LaneAttribute::INTERSECTION) != 0)
        end_count =
            refline_attribute_lane_[i].lane_start_index +
            (end_count - refline_attribute_lane_[i].lane_start_index) / 3 * 2;
      for (int j = refline_attribute_lane_[i].lane_start_index; j < end_count;
           j++) {
        refline_attribute_point_[j].no_virtual_box = true;
      }
    } else {
      if ((refline_attribute_lane_[i].lane_attribute &
           (int)LaneAttribute::INTERSECTION) == 0) {
        for (int j = refline_attribute_lane_[i].lane_start_index;
             j < refline_attribute_lane_[i].lane_start_index +
                     refline_attribute_lane_[i].count_lane_points;
             j++) {
          if (refline_attribute_point_[j].curvature_theta >
              via_point_decider_cfg_->curve_lane_point_curv_mmin_) {
            refline_attribute_point_[j].no_virtual_box = check_turn(j, i);
          }
        }
      }
    }
  }

  via_s = 0.0;
  lock = false;
  for (int i = 0; i < refline_attribute_point_.size(); i++) {
    if (refline_attribute_point_[i].no_virtual_box &&
        i < (refline_attribute_point_.size() - 2) && i > 0) {
      if ((refline_attribute_point_[i + 1].s - refline_attribute_point_[i].s) <
              via_point_decider_cfg_->ref_trajectory_info_via_delta_s_filter_ &&
          via_s <
              via_point_decider_cfg_->ref_trajectory_info_via_delta_s_filter_ &&
          !refline_attribute_point_[i + 1].no_virtual_box && !lock) {
        via_s +=
            refline_attribute_point_[i + 1].s - refline_attribute_point_[i].s;
        refline_attribute_point_[i + 1].no_virtual_box = true;
        if ((via_s + refline_attribute_point_[i + 2].s -
             refline_attribute_point_[i + 1].s) >
            via_point_decider_cfg_->ref_trajectory_info_via_delta_s_filter_) {
          via_s = 0.0;
          lock = true;
        }
      } else {
        if (refline_attribute_point_[i + 1].no_virtual_box && !lock) {
          via_s +=
              refline_attribute_point_[i + 1].s - refline_attribute_point_[i].s;
        } else {
          via_s = 0.0;
          lock = false;
        }
      }
    }
  }

  // print ref_trajectory_info_

  // for(int i = 0;i< refline_attribute_point_.size();i++){
  //   std::cout << "refline_attribute_point_: " << i << " "
  //             << "s: " << refline_attribute_point_[i].s << " "
  //             << "no_virtual_box: " <<
  //             refline_attribute_point_[i].no_virtual_box << " "
  //             << "via_point: " << refline_attribute_point_[i].via_point << "
  //             "
  //             // << "x: " << refline_attribute_point_[i].x << " "
  //             // << "y: " << refline_attribute_point_[i].y << " "
  //             // << "curvature: " << refline_attribute_point_[i].curvature <<
  //             " "
  //             // << "curvature_theta: " <<
  //             refline_attribute_point_[i].curvature_theta << " "
  //             << "lane_width: " << refline_attribute_point_[i].lane_width <<
  //             " "
  //             << "left_road_border: " <<
  //             refline_attribute_point_[i].left_road_border_distance << " "
  //             << "right_road_border: " <<
  //             refline_attribute_point_[i].right_road_border_distance << " "
  //             << "left_lane_border: " <<
  //             refline_attribute_point_[i].left_lane_border_distance << " "
  //             << "right_lane_border: " <<
  //             refline_attribute_point_[i].right_lane_border_distance << " "
  //             << std::endl;
  // }

  // print lane attribute

  // for(int i = 0; i < refline_attribute_lane_.size(); i++){
  //   std::cout << "new_lane_attribute: " << i
  //             << " " << refline_attribute_lane_[i].id_lane
  //             << " " << refline_attribute_lane_[i].length_lane
  //             << " " << (refline_attribute_lane_[i].lane_attribute &
  //             (int)LaneAttribute::STRAIGHT)
  //             << " " << (refline_attribute_lane_[i].lane_attribute &
  //             (int)LaneAttribute::DOUBLE)
  //             << " " << (refline_attribute_lane_[i].lane_attribute &
  //             (int)LaneAttribute::INTERSECTION)
  //             << " " << (refline_attribute_lane_[i].lane_attribute &
  //             (int)LaneAttribute::VIRTUAL_BOX)
  //             << " " << (refline_attribute_lane_[i].lane_attribute &
  //             (int)LaneAttribute::VIA)
  //             << " " << refline_attribute_lane_[i].s_lane_start
  //             << " " << refline_attribute_lane_[i].avg_curvature
  //             << std::endl;
  // }
}

bool RefLineManager::cal_refline_attribute_point(
    std::vector<RefLineAttributePoint> &attribute_point) {
  auto frenet_coord_ = world_model_->get_frenet_coord();

  Point2D point, point_fren;
  // Eigen::Vector3d car_point, enu_point;
  double lane_width;
  double left_lane_border_distance;
  double left_road_border_distance;
  double right_lane_border_distance;
  double right_road_border_distance;

  double last_lane_width = 7.0;
  double last_left_lane_border_distance = 3.5;
  double last_left_road_border_distance = 3.5;
  double last_right_lane_border_distance = 3.5;
  double last_right_road_border_distance = 3.5;
  int index = 0;

  double s_pt = -1.0;

  if (world_model_->get_map_info().current_refline_points.empty() ||
      world_model_->get_map_info().current_refline_points.size() < 3) {
    return false;
  }
  // std::cout << world_model_->get_map_info().current_refline_points.size() <<
  // std::endl;
  for (auto p : world_model_->get_map_info().current_refline_points) {

    point.x = p.x;
    point.y = p.y;
    // std::cout << "srnbdebug: " << point.x << "  " << point.y << std::endl;
    (void)frenet_coord_->CartCoord2FrenetCoord(point, point_fren);

    // TODO : replace hack width with lane width
    if (point_fren.x > 1.0e-9 && point_fren.x < 100.) {
      RefLineAttributePoint new_point;
      int id_lane, in_point;
      new_point.x = point.x;
      new_point.y = point.y;
      if (s_pt < 0.0) {
        s_pt = std::hypot(
            point.x - world_model_->get_map_info().ref_trajectory.front().x,
            point.y - world_model_->get_map_info().ref_trajectory.front().y);
      } else {
        s_pt += std::hypot(point.x - attribute_point.back().x,
                           point.y - attribute_point.back().y);
      }
      new_point.s = s_pt;
      new_point.l = point_fren.y;

      if (false && world_model_->is_parking_lvp()) {
        new_point.intersection = false;
        lane_width = 7.0;
        left_lane_border_distance = 3.5;
        left_road_border_distance = 3.5;
        right_lane_border_distance = 3.5;
        right_road_border_distance = 3.5;
        id_lane = (int)(index / 40);
      } else {
        if (s_pt < 0.0) {
          last_lane_width = std::abs(p.lane_width) > 10.0 ? 10.0 : p.lane_width;
          last_left_lane_border_distance =
              std::abs(p.left_lane_border_distance) > 10.0
                  ? 3.5
                  : p.left_lane_border_distance;
          last_left_road_border_distance =
              std::abs(p.left_road_border_distance) > 10.0
                  ? 3.5
                  : p.left_road_border_distance;
          last_right_lane_border_distance =
              std::abs(p.right_lane_border_distance) > 10.0
                  ? 3.5
                  : p.right_lane_border_distance;
          last_right_road_border_distance =
              std::abs(p.right_road_border_distance) > 10.0
                  ? 3.5
                  : p.right_road_border_distance;
        }
        if (std::abs(p.left_road_border_distance) > 100.0 ||
            std::abs(p.right_road_border_distance) > 100.0)
          new_point.intersection = true;
        else
          new_point.intersection = false;

        lane_width = clip(
            double(p.left_road_border_distance + p.right_road_border_distance),
            10.0, 1.0);
        lane_width =
            std::fabs(lane_width - 10.0) < 1e-2 ? last_lane_width : lane_width;
        left_lane_border_distance =
            clip(std::abs((double)p.left_lane_border_distance), 10.0, 0.0);
        left_lane_border_distance = new_point.intersection
                                        ? last_left_lane_border_distance
                                        : left_lane_border_distance;
        // if(std::abs(std::abs(p.left_lane_border_distance) -
        // std::abs(last_left_lane_border_distance)) > 2.0)
        //   MSD_LOG(WARN, "left_lane_border_distance trembles exceeds 2.0
        //   meter!");

        left_road_border_distance =
            clip(std::abs((double)p.left_road_border_distance), 10.0, 0.0);
        left_road_border_distance = new_point.intersection
                                        ? last_left_road_border_distance
                                        : left_road_border_distance;
        // if(std::abs(std::abs(p.left_road_border_distance) -
        // std::abs(last_left_road_border_distance)) > 2.0)
        //   MSD_LOG(WARN, "left_road_border_distance trembles exceeds 2.0
        //   meter!");

        right_lane_border_distance =
            clip(std::abs((double)p.right_lane_border_distance), 10.0, 0.0);
        right_lane_border_distance = new_point.intersection
                                         ? last_right_lane_border_distance
                                         : right_lane_border_distance;
        // if(std::abs(std::abs(p.right_lane_border_distance) -
        // std::abs(last_right_lane_border_distance)) > 2.0)
        //   MSD_LOG(WARN, "right_lane_border_distance trembles exceeds 2.0
        //   meter!");

        right_road_border_distance =
            clip(std::abs((double)p.right_road_border_distance), 10.0, 0.0);
        right_road_border_distance = new_point.intersection
                                         ? last_right_road_border_distance
                                         : right_road_border_distance;
        // if(std::abs(std::abs(p.right_road_border_distance) -
        // std::abs(last_right_road_border_distance)) > 2.0)
        //   MSD_LOG(WARN, "right_road_border_distance trembles exceeds 2.0
        //   meter!");
        // MSD_LOG(WARN, "last---lane_width, left_lane_border_distance,d
        // right_lane_border_distance, left_road_border_distance,
        // right_road_border_distance=%f, %f, %f, %f, %f.",
        //   last_lane_width, last_left_lane_border_distance,
        //   last_right_lane_border_distance, last_left_road_border_distance,
        //   last_right_road_border_distance);
        // MSD_LOG(WARN, "new----lane_width, left_lane_border_distance,
        // right_lane_border_distance, left_road_border_distance,
        // right_road_border_distance=%f, %f, %f, %f, %f.",
        //   lane_width, left_lane_border_distance, right_lane_border_distance,
        //   left_road_border_distance, right_road_border_distance);
        if (!track_id_separate(p.track_id, "-", id_lane, in_point))
          return false;
      }
      new_point.lane_width = lane_width;
      new_point.left_lane_border_distance = left_lane_border_distance;
      new_point.left_road_border_distance = left_road_border_distance;
      new_point.right_lane_border_distance = right_lane_border_distance;
      new_point.right_road_border_distance = right_road_border_distance;
      new_point.curvature = frenet_coord_->GetRefCurveCurvature(point_fren.x);
      new_point.theta = frenet_coord_->GetRefCurveHeading(point_fren.x);
      new_point.id_lane = id_lane;
      bool is_carriage_way =
          (std::abs(left_lane_border_distance - left_road_border_distance) >
               1.0 &&
           std::abs((left_lane_border_distance + right_lane_border_distance) -
                    (left_road_border_distance - left_lane_border_distance)) <
               2.0);
      new_point.carriage_way = is_carriage_way;
      new_point.interpolation = false;
      attribute_point.emplace_back(new_point);
      // set last value
      last_lane_width = lane_width;
      last_left_lane_border_distance = left_lane_border_distance;
      last_left_road_border_distance = left_road_border_distance;
      last_right_lane_border_distance = right_lane_border_distance;
      last_right_road_border_distance = right_road_border_distance;
      index++;
    }
  }
  // // modify refline s
  // for(size_t i = 1; i < ref_trajectory_info_.s.size() - 1; i++){
  //   if(!(ref_trajectory_info_.s[i] > ref_trajectory_info_.s[i - 1] &&
  //   ref_trajectory_info_.s[i] < ref_trajectory_info_.s[i + 1]))
  //     ref_trajectory_info_.s[i] = (ref_trajectory_info_.s[i - 1] +
  //     ref_trajectory_info_.s[i + 1]) * 0.5;
  // }

  return true;
}

// bool RefLineManager::refline_info2attribute_point(
//     const RefLineInfoFrenet &refline_info,
//     std::vector<RefLineAttributePoint> &attribute_point) {
//     // std::cout << " refline_info.s.size() " << refline_info.s.size()
//     //           << " refline_info.l.size() " << refline_info.l.size()
//     //           << " refline_info.x.size() " << refline_info.x.size()
//     //           << " refline_info.y.size() " << refline_info.y.size()
//     //           << " refline_info.theta.size() " <<
//     refline_info.theta.size()
//     //           << " refline_info.left_road_border_distance.size() " <<
//     refline_info.left_road_border_distance.size()
//     //           << " refline_info.right_road_border_distance.size() " <<
//     refline_info.right_road_border_distance.size()
//     //           << " refline_info.left_lane_border_distance.size() " <<
//     refline_info.left_lane_border_distance.size()
//     //           << " refline_info.right_lane_border_distance.size() " <<
//     refline_info.right_lane_border_distance.size()
//     //           << " refline_info.left_obstacle_distance.size() " <<
//     refline_info.left_obstacle_distance.size()
//     //           << " refline_info.right_obstacle_distance.size() " <<
//     refline_info.right_obstacle_distance.size()
//     //           << " refline_info.lane_width.size() " <<
//     refline_info.lane_width.size()
//     //           << " refline_info.curvature.size() " <<
//     refline_info.curvature.size()
//     //           << " refline_info.curvature_theta.size() " <<
//     refline_info.curvature_theta.size()
//     //           << " refline_info.intersection.size() " <<
//     refline_info.intersection.size()
//     //           << " refline_info.carriage_way.size() " <<
//     refline_info.carriage_way.size()
//     //           << std::endl;

//   for (int i = 0; i < refline_info.s.size(); i++) {
//     int id_lane, in_point;
//     RefLineAttributePoint new_point;
//     if (!track_id_separate(refline_info.track_id[i], "-", id_lane, in_point))
//       return false;
//     new_point.id_lane = id_lane;
//     new_point.s = refline_info.s[i];
//     new_point.l = refline_info.l[i];
//     new_point.x = refline_info.x[i];
//     new_point.y = refline_info.y[i];
//     new_point.theta = 0.0;
//     new_point.left_road_border_distance =
//         refline_info.left_road_border_distance[i];
//     new_point.right_road_border_distance =
//         refline_info.right_road_border_distance[i];
//     new_point.left_lane_border_distance =
//         refline_info.left_lane_border_distance[i];
//     new_point.right_lane_border_distance =
//         refline_info.right_lane_border_distance[i];
//     new_point.left_obstacle_distance = 1e19;
//     new_point.right_obstacle_distance = 1e19;
//     new_point.lane_width = refline_info.lane_width[i];
//     new_point.curvature = refline_info.curvature[i];
//     new_point.curvature_theta = 0.0;
//     new_point.intersection = refline_info.intersection[i];
//     new_point.carriage_way = refline_info.carriage_way[i];
//     new_point.interpolation = false;
//     attribute_point.emplace_back(new_point);
//   }

//   return true;
// }

bool RefLineManager::resize_attribute_point(
    std::vector<RefLineAttributePoint> &attribute_point) {
  if (attribute_point.empty())
    return false;
  if (attribute_point.size() == 1)
    return false;
  int count = 0;
  for (int index = 0; index < attribute_point.size() - 1; index++) {
    if ((attribute_point[index + 1].s - attribute_point[index].s) > 3.0 &&
        attribute_point[index].curvature < 0.05) {
      RefLineAttributePoint new_point;
      new_point.id_lane = attribute_point[index].id_lane;
      new_point.s = attribute_point[index].s + 2.0;
      new_point.l = planning_math::lerp(
          attribute_point[index].l, attribute_point[index].s,
          attribute_point[index + 1].l, attribute_point[index + 1].s,
          new_point.s);
      new_point.x = planning_math::lerp(
          attribute_point[index].x, attribute_point[index].s,
          attribute_point[index + 1].x, attribute_point[index + 1].s,
          new_point.s);
      new_point.y = planning_math::lerp(
          attribute_point[index].y, attribute_point[index].s,
          attribute_point[index + 1].y, attribute_point[index + 1].s,
          new_point.s);
      new_point.theta = planning_math::lerp(
          attribute_point[index].theta, attribute_point[index].s,
          attribute_point[index + 1].theta, attribute_point[index + 1].s,
          new_point.s);
      new_point.left_road_border_distance =
          attribute_point[index].left_road_border_distance;
      new_point.right_road_border_distance =
          attribute_point[index].right_road_border_distance;
      new_point.left_lane_border_distance =
          attribute_point[index].left_lane_border_distance;
      new_point.right_lane_border_distance =
          attribute_point[index].right_lane_border_distance;
      new_point.left_obstacle_distance =
          attribute_point[index].left_obstacle_distance;
      new_point.right_obstacle_distance =
          attribute_point[index].right_obstacle_distance;

      new_point.lane_width = attribute_point[index].lane_width;
      new_point.curvature = attribute_point[index].curvature;
      new_point.curvature_theta = attribute_point[index].curvature_theta;
      new_point.intersection = attribute_point[index].intersection;
      new_point.carriage_way = attribute_point[index].carriage_way;
      new_point.interpolation = true;
      attribute_point.insert(attribute_point.begin() + index + 1, new_point);
    }
    if (!attribute_point[index].interpolation) {
      count = 0;
    } else {
      count++;
    }
    if (count > 20) {
      // std::cout << "zjt error: dead loop " << std::endl;
      return false;
    }
  }
  return true;
}

// bool RefLineManager::resize_attribute_point() {
//           std::cout << "s1" << std::endl;
//   std::vector<RefLineAttributePoint>::iterator itr =
//   refline_attribute_point_.begin(); if (refline_attribute_point_.empty())
//     return false;
//   if (refline_attribute_point_.size() == 1)
//     return false;
//   int count = 0;
//   while (itr != refline_attribute_point_.end() - 2) {
//     std::cout << "s2" << std::endl;
//     auto next_itr = itr + 1;
//     if ((next_itr->s - itr->s) > 3.0 && itr->curvature < 0.05) {
//       std::cout << "s3" << std::endl;
//       RefLineAttributePoint new_point;
//       new_point.id_lane = itr->id_lane;
//       new_point.s = itr->s + 2.0;
//       new_point.l = planning_math::lerp(itr->l, itr->s, next_itr->l,
//                                         next_itr->s, new_point.s);
//       new_point.x = planning_math::lerp(itr->x, itr->s, next_itr->x,
//                                         next_itr->s, new_point.s);
//       new_point.y = planning_math::lerp(itr->y, itr->s, next_itr->y,
//                                         next_itr->s, new_point.s);
//       std::cout << "s4" << std::endl;
//       new_point.theta = itr->theta;
//       new_point.left_road_border_distance = itr->left_road_border_distance;
//       new_point.right_road_border_distance = itr->right_road_border_distance;
//       new_point.left_lane_border_distance = itr->left_lane_border_distance;
//       new_point.right_lane_border_distance = itr->right_lane_border_distance;
//       new_point.left_obstacle_distance = itr->left_obstacle_distance;
//       new_point.right_obstacle_distance = itr->right_obstacle_distance;
//       std::cout << "s5" << std::endl;
//       new_point.lane_width = itr->lane_width;
//       new_point.curvature = itr->curvature;
//       new_point.curvature_theta = itr->curvature_theta;
//       new_point.intersection = itr->intersection;
//       new_point.carriage_way = itr->carriage_way;
//       new_point.interpolation = true;
//       std::cout << "s6" << std::endl;
//       refline_attribute_point_.insert(itr + 1, new_point);
//       std::cout << "s7" << std::endl;
//     }
//     std::cout << "s8" << std::endl;
//     if (!itr->interpolation) {
//       count = 0;
//     } else {
//       count ++;
//     }
//     if (count > 10) {
//       std::cout << "zjt error: dead loop " << std::endl;
//       return false;
//     }
//     std::cout << "s10" << std::endl;
//     itr ++;
//     // std::cout << itr->s <<" "<<attribute_point.size()<< std::endl;
//   }
//   return true;
// }

bool RefLineManager::get_attribute_point_index(double s, int &idx) {
  if (refline_attribute_point_.empty()) {
    idx = 0;
    return false;
  }
  if (refline_attribute_point_.size() == 1) {
    idx = 0;
    return false;
  }

  if (s < refline_attribute_point_.front().s) {
    idx = -1;
    return true;
  } else if (s > refline_attribute_point_.back().s) {
    idx = refline_attribute_point_.size() - 1;
    return true;
  }
  for (int j = 0; j < (int)refline_attribute_point_.size() - 1; j++) {
    if (s >= refline_attribute_point_[j].s &&
        s <= refline_attribute_point_[j + 1].s) {
      idx = j;
      return true;
    }
  }
  idx = 0;
  return false;
}

double RefLineManager::cal_left_road_border_distance(double s) {
  int index = 0;
  if (!get_attribute_point_index(s, index))
    return 0.0;
  if (index == refline_attribute_point_.size() - 1) {
    return refline_attribute_point_.back().left_road_border_distance;
  } else if (index == -1) {
    return refline_attribute_point_.front().left_road_border_distance;
  } else {
    return planning_math::lerp(
        refline_attribute_point_[index].left_road_border_distance,
        refline_attribute_point_[index].s,
        refline_attribute_point_[index + 1].left_road_border_distance,
        refline_attribute_point_[index + 1].s, s);
  }
}

double RefLineManager::cal_left_lane_border_distance(double s) {
  int index = 0;
  if (!get_attribute_point_index(s, index))
    return 0.0;
  if (index == refline_attribute_point_.size() - 1) {
    return refline_attribute_point_.back().left_lane_border_distance;
  } else if (index == -1) {
    return refline_attribute_point_.front().left_lane_border_distance;
  } else {
    return planning_math::lerp(
        refline_attribute_point_[index].left_lane_border_distance,
        refline_attribute_point_[index].s,
        refline_attribute_point_[index + 1].left_lane_border_distance,
        refline_attribute_point_[index + 1].s, s);
  }
}

double RefLineManager::cal_right_road_border_distance(double s) {
  int index = 0;
  if (!get_attribute_point_index(s, index))
    return 0.0;
  if (index == refline_attribute_point_.size() - 1) {
    return refline_attribute_point_.back().right_road_border_distance;
  } else if (index == -1) {
    return refline_attribute_point_.front().right_road_border_distance;
  } else {
    return planning_math::lerp(
        refline_attribute_point_[index].right_road_border_distance,
        refline_attribute_point_[index].s,
        refline_attribute_point_[index + 1].right_road_border_distance,
        refline_attribute_point_[index + 1].s, s);
  }
}

double RefLineManager::cal_right_lane_border_distance(double s) {
  int index = 0;
  if (!get_attribute_point_index(s, index))
    return 0.0;
  if (index == refline_attribute_point_.size() - 1) {
    return refline_attribute_point_.back().right_lane_border_distance;
  } else if (index == -1) {
    return refline_attribute_point_.front().right_lane_border_distance;
  } else {
    return planning_math::lerp(
        refline_attribute_point_[index].right_lane_border_distance,
        refline_attribute_point_[index].s,
        refline_attribute_point_[index + 1].right_lane_border_distance,
        refline_attribute_point_[index + 1].s, s);
  }
}

double RefLineManager::cal_lane_width(double s) {
  int index = 0;
  if (!get_attribute_point_index(s, index))
    return 0.0;
  if (index == refline_attribute_point_.size() - 1) {
    return refline_attribute_point_.back().lane_width;
  } else if (index == -1) {
    return refline_attribute_point_.front().lane_width;
  } else {
    return planning_math::lerp(refline_attribute_point_[index].lane_width,
                               refline_attribute_point_[index].s,
                               refline_attribute_point_[index + 1].lane_width,
                               refline_attribute_point_[index + 1].s, s);
  }
}

double RefLineManager::cal_curvature(double s) {
  int index = 0;
  if (!get_attribute_point_index(s, index))
    return 0.0;
  if (index == refline_attribute_point_.size() - 1) {
    return refline_attribute_point_.back().curvature;
  } else if (index == -1) {
    return refline_attribute_point_.front().curvature;
  } else {
    return planning_math::lerp(refline_attribute_point_[index].curvature,
                               refline_attribute_point_[index].s,
                               refline_attribute_point_[index + 1].curvature,
                               refline_attribute_point_[index + 1].s, s);
  }
}

double RefLineManager::cal_curvature_theta(double s) {
  int index = 0;
  if (!get_attribute_point_index(s, index))
    return 0.0;
  if (index == refline_attribute_point_.size() - 1) {
    return refline_attribute_point_.back().curvature_theta;
  } else if (index == -1) {
    return refline_attribute_point_.front().curvature_theta;
  } else {
    return planning_math::lerp(
        refline_attribute_point_[index].curvature_theta,
        refline_attribute_point_[index].s,
        refline_attribute_point_[index + 1].curvature_theta,
        refline_attribute_point_[index + 1].s, s);
  }
}

RefLineAttributePoint RefLineManager::interp_attribute_point(double s) {
  RefLineAttributePoint new_point = {};
  int index;
  if (!get_attribute_point_index(s, index))
    return new_point;
  if (index == refline_attribute_point_.size() - 1) {
    new_point = refline_attribute_point_.back();
    new_point.s = s;
  } else if (index == -1) {
    new_point = refline_attribute_point_.front();
    new_point.s = s;
  } else {
    new_point.s = s;
    new_point.l = planning_math::lerp(
        refline_attribute_point_[index].l, refline_attribute_point_[index].s,
        refline_attribute_point_[index + 1].l,
        refline_attribute_point_[index + 1].s, new_point.s);
    new_point.x = planning_math::lerp(
        refline_attribute_point_[index].x, refline_attribute_point_[index].s,
        refline_attribute_point_[index + 1].x,
        refline_attribute_point_[index + 1].s, new_point.s);
    new_point.y = planning_math::lerp(
        refline_attribute_point_[index].y, refline_attribute_point_[index].s,
        refline_attribute_point_[index + 1].y,
        refline_attribute_point_[index + 1].s, new_point.s);
    new_point.theta = refline_attribute_point_[index].theta;
    new_point.left_road_border_distance = planning_math::lerp(
        refline_attribute_point_[index].left_road_border_distance,
        refline_attribute_point_[index].s,
        refline_attribute_point_[index + 1].left_road_border_distance,
        refline_attribute_point_[index + 1].s, new_point.s);
    new_point.right_road_border_distance = planning_math::lerp(
        refline_attribute_point_[index].right_road_border_distance,
        refline_attribute_point_[index].s,
        refline_attribute_point_[index + 1].right_road_border_distance,
        refline_attribute_point_[index + 1].s, new_point.s);
    new_point.left_lane_border_distance = planning_math::lerp(
        refline_attribute_point_[index].left_lane_border_distance,
        refline_attribute_point_[index].s,
        refline_attribute_point_[index + 1].left_lane_border_distance,
        refline_attribute_point_[index + 1].s, new_point.s);
    new_point.right_lane_border_distance = planning_math::lerp(
        refline_attribute_point_[index].right_lane_border_distance,
        refline_attribute_point_[index].s,
        refline_attribute_point_[index + 1].right_lane_border_distance,
        refline_attribute_point_[index + 1].s, new_point.s);
    new_point.left_obstacle_distance = planning_math::lerp(
        refline_attribute_point_[index].left_obstacle_distance,
        refline_attribute_point_[index].s,
        refline_attribute_point_[index + 1].left_obstacle_distance,
        refline_attribute_point_[index + 1].s, new_point.s);
    new_point.right_obstacle_distance = planning_math::lerp(
        refline_attribute_point_[index].right_obstacle_distance,
        refline_attribute_point_[index].s,
        refline_attribute_point_[index + 1].right_obstacle_distance,
        refline_attribute_point_[index + 1].s, new_point.s);
    new_point.lane_width =
        planning_math::lerp(refline_attribute_point_[index].lane_width,
                            refline_attribute_point_[index].s,
                            refline_attribute_point_[index + 1].lane_width,
                            refline_attribute_point_[index + 1].s, new_point.s);
    new_point.curvature =
        planning_math::lerp(refline_attribute_point_[index].curvature,
                            refline_attribute_point_[index].s,
                            refline_attribute_point_[index + 1].curvature,
                            refline_attribute_point_[index + 1].s, new_point.s);
    new_point.curvature_theta =
        planning_math::lerp(refline_attribute_point_[index].curvature_theta,
                            refline_attribute_point_[index].s,
                            refline_attribute_point_[index + 1].curvature_theta,
                            refline_attribute_point_[index + 1].s, new_point.s);
    new_point.intersection = refline_attribute_point_[index].intersection &&
                             refline_attribute_point_[index + 1].intersection;
    new_point.carriage_way = refline_attribute_point_[index].carriage_way &&
                             refline_attribute_point_[index + 1].carriage_way;
    new_point.interpolation = true;
    new_point.via_point = refline_attribute_point_[index].via_point &&
                          refline_attribute_point_[index + 1].via_point;
    new_point.no_virtual_box =
        refline_attribute_point_[index].no_virtual_box &&
        refline_attribute_point_[index + 1].no_virtual_box;
  }
  return new_point;
}

void RefLineManager::reset() {
  refline_attribute_lane_last_.clear();
  refline_attribute_point_last_.clear();
  refline_attribute_lane_.clear();
  refline_attribute_point_.clear();
}

bool RefLineManager::track_id_separate(const std::string &track_id,
                                       const std::string &delimiter,
                                       int &id_lane, int &in_point) {
  std::size_t found_pos = track_id.find(delimiter);
  if (found_pos == std::string::npos)
    return false;
  id_lane = std::stoi(track_id.substr(0, found_pos));
  in_point = std::stoi(track_id.substr(found_pos + 1, track_id.length()));
  return true;
}

double RefLineManager::cal_theta(const double s) {
  if (refline_attribute_point_.empty())
    return 0.0;
  if (refline_attribute_point_.size() == 1)
    return refline_attribute_point_.front().theta;
  for (int j = 0; j < (int)refline_attribute_point_.size() - 1; j++) {
    if (s >= refline_attribute_point_[j].s &&
        s <= refline_attribute_point_[j + 1].s) {
      return refline_attribute_point_[j].theta;
    }
  }
  return refline_attribute_point_.back().theta;
}

bool RefLineManager::check_turn(const int &index_traj, const int &index_lane) {
  const double delta_s = via_point_decider_cfg_->check_turn_delta_s_;
  // std::cout << "check_turn: " << index_traj << " " <<
  // ref_info_index_.id_lane[index_lane] << std::endl;
  double mid_s = refline_attribute_point_[index_traj].s;
  int positive_count = 0;
  int negative_count = 0;
  int invalid_count = 0;
  for (int i = 1; i < via_point_decider_cfg_->check_turn_pt_count_; i++) {
    double s_front = mid_s - i * delta_s;
    double s_end = mid_s + i * delta_s;
    double curv = (cal_theta(s_end) - cal_theta(s_front)) / (2.0 * i * delta_s);
    if (curv > via_point_decider_cfg_->curve_lane_point_curv_mmin_)
      positive_count++;
    else if (curv < -via_point_decider_cfg_->curve_lane_point_curv_mmin_)
      negative_count++;
    else
      invalid_count++;
    // std::cout << i << " " << curv << " " << cal_theta(s_end) << " " <<
    // cal_theta(s_front) << " " << positive_count << " " << negative_count << "
    // " << invalid_count << std::endl;
  }
  if (invalid_count > via_point_decider_cfg_->check_turn_pt_invalid_limit_)
    return false;
  else
    return true;
}

double RefLineManager::straight_lane_length_before(const int &index) {
  double length = 0.0;
  int idx = index - 1;
  if ((refline_attribute_lane_[idx].lane_attribute &
       (int)LaneAttribute::STRAIGHT) == 0 &&
      refline_attribute_lane_[idx].length_lane >
          via_point_decider_cfg_->straight_lane_length_before_limit_)
    return via_point_decider_cfg_->straight_lane_length_before_limit_ +
           via_point_decider_cfg_->epsilon_;
  while (idx >= 0) {
    if ((refline_attribute_lane_[idx].lane_attribute &
         (int)LaneAttribute::STRAIGHT) != 0)
      length += refline_attribute_lane_[idx].length_lane;
    else
      return length;
    idx--;
  }
  return length;
}

double RefLineManager::straight_lane_length_after(const int &index) {
  double length = 0.0;
  int idx = index + 1;
  while (idx < refline_attribute_lane_.size()) {
    if (refline_attribute_lane_[idx].length_lane >
        via_point_decider_cfg_->straight_lane_length_after_limit_) {
      double s_start = refline_attribute_lane_[idx].s_lane_start;
      double s_end = std::min(
          frenet_coord_->GetLength(),
          s_start + via_point_decider_cfg_->straight_lane_length_after_limit_);
      // std::cout << "straight_lane_length_after: " << idx << " " << s_start <<
      // " " << s_end
      //           << " " << frenet_coord_->GetRefCurveHeading(s_start) << " "
      //           << frenet_coord_->GetRefCurveHeading(s_end) << std::endl;
      double theta_diff = planning_math::NormalizeAngle(
          frenet_coord_->GetRefCurveHeading(s_end) -
          frenet_coord_->GetRefCurveHeading(s_start));
      if (theta_diff < via_point_decider_cfg_->curve_theta_check_limit_ &&
          theta_diff > -via_point_decider_cfg_->curve_theta_check_limit_)
        return s_end - s_start + via_point_decider_cfg_->epsilon_;
      else
        return via_point_decider_cfg_->straight_lane_length_after_limit_;
    }
    if ((refline_attribute_lane_[idx].lane_attribute &
         (int)LaneAttribute::STRAIGHT) != 0)
      length += refline_attribute_lane_[idx].length_lane;
    else
      return length;
    idx++;
  }
  return length;
}

bool RefLineManager::is_intersection(const double s) {
  if (refline_attribute_point_.empty())
    return false;
  if (refline_attribute_point_.size() == 1)
    return false;
  for (int j = 0; j < (int)refline_attribute_point_.size() - 1; j++) {
    if (s >= refline_attribute_point_[j].s &&
        s <= refline_attribute_point_[j + 1].s) {
      return refline_attribute_point_[j].intersection;
    }
  }
  return refline_attribute_point_.back().intersection;
}

bool RefLineManager::is_carriage_way(const double s) {

  if (refline_attribute_point_.empty())
    return false;
  if (refline_attribute_point_.size() == 1)
    return false;
  for (int j = 0; j < (int)refline_attribute_point_.size() - 1; j++) {
    if (s >= refline_attribute_point_[j].s &&
        s <= refline_attribute_point_[j + 1].s) {
      return refline_attribute_point_[j].carriage_way;
    }
  }
  return refline_attribute_point_.back().carriage_way;
}

bool RefLineManager::is_no_virtual_box(const double s) {
  if (refline_attribute_point_.empty())
    return true;
  if (refline_attribute_point_.size() == 1)
    return true;
  for (int j = 0; j < (int)refline_attribute_point_.size() - 1; j++) {
    if (s >= refline_attribute_point_[j].s &&
        s <= refline_attribute_point_[j + 1].s) {
      return refline_attribute_point_[j].no_virtual_box &&
             refline_attribute_point_[j + 1].no_virtual_box;
    }
  }
  return refline_attribute_point_.back().no_virtual_box;
}

bool RefLineManager::is_no_virtual_box(const double s, double &idx) {
  if (refline_attribute_point_.empty())
    return true;
  if (refline_attribute_point_.size() == 1)
    return true;
  for (int j = 0; j < (int)refline_attribute_point_.size() - 1; j++) {
    if (s >= refline_attribute_point_[j].s &&
        s <= refline_attribute_point_[j + 1].s) {
      idx = j;
      return refline_attribute_point_[j].no_virtual_box &&
             refline_attribute_point_[j + 1].no_virtual_box;
    }
  }
  idx = refline_attribute_point_.size() - 1;
  return refline_attribute_point_.back().no_virtual_box;
}

double RefLineManager::cal_dist2virtual_box(const double s) {
  double dist = 0.0;
  double idx = 0;
  if (is_no_virtual_box(s, idx)) {
    while (idx >= 0 && refline_attribute_point_[idx].no_virtual_box) {
      if (idx > 0)
        dist += std::abs(refline_attribute_point_[idx].s -
                         refline_attribute_point_[idx - 1].s);
      idx--;
    }
    if (idx < (int)refline_attribute_point_.size() - 1 && idx > 0) {
      if (std::abs(refline_attribute_point_[idx + 1].s -
                   refline_attribute_point_[idx].s) > 2.0 &&
          !refline_attribute_point_[idx - 1].no_virtual_box) {
        dist = std::max(0.0, dist -
                                 std::abs(refline_attribute_point_[idx + 1].s -
                                          refline_attribute_point_[idx].s) +
                                 2.0);
      }
    }
    dist = -dist;
  } else {
    while (idx < refline_attribute_point_.size() &&
           refline_attribute_point_[idx].no_virtual_box) {
      if (int(idx) != refline_attribute_point_.size() - 2)
        dist += std::abs(refline_attribute_point_[idx].s -
                         refline_attribute_point_[idx + 1].s);
      idx++;
    }
  }
  return dist;
}

bool RefLineManager::is_via_point(const double s) {
  if (refline_attribute_point_.empty())
    return false;
  if (refline_attribute_point_.size() == 1)
    return false;
  for (int j = 0; j < (int)refline_attribute_point_.size() - 1; j++) {
    if (s >= refline_attribute_point_[j].s &&
        s <= refline_attribute_point_[j + 1].s) {
      return refline_attribute_point_[j].via_point &&
             refline_attribute_point_[j + 1].via_point;
    }
  }
  return refline_attribute_point_.back().via_point;
}

void RefLineManager::update_via_weight_base(const double via_weight_base,
                                            const int id) {
  auto id_itr = std::find_if(
      refline_attribute_lane_.begin(), refline_attribute_lane_.end(),
      [id](RefLineAttributeLane lane) { return lane.id_lane = id; });
  if (id_itr != refline_attribute_lane_.end()) {
    size_t index = std::distance(refline_attribute_lane_.begin(), id_itr);
    refline_attribute_lane_[index].via_weight_base = via_weight_base;
  } else {
    return;
  }
}

double RefLineManager::get_via_weight_base(const double s) {
  if (refline_attribute_lane_.empty())
    return 0.0;
  if (refline_attribute_lane_.size() == 1)
    return refline_attribute_lane_.front().via_weight_base;
  for (size_t i = 0; i < refline_attribute_lane_.size(); i++) {
    if (refline_attribute_lane_[i].s_lane_start > s)
      return refline_attribute_lane_[std::max(0, int(i) - 1)].via_weight_base;
  }
  return refline_attribute_lane_.back().via_weight_base;
}

int RefLineManager::get_lane_id(const double s) {
  if (refline_attribute_lane_.empty())
    return 0;
  if (refline_attribute_lane_.size() == 1)
    return refline_attribute_lane_.front().id_lane;
  for (size_t i = 0; i < refline_attribute_lane_.size(); i++) {
    if (refline_attribute_lane_[i].s_lane_start > s)
      return refline_attribute_lane_[std::max(0, int(i) - 1)].id_lane;
  }
  return refline_attribute_lane_.back().id_lane;
}

bool RefLineManager::is_intersection_lane(const int id) {
  if (refline_attribute_lane_.empty())
    return false;
  for (size_t i = 0; i < refline_attribute_lane_.size(); i++) {
    if (refline_attribute_lane_[i].id_lane == id)
      return ((refline_attribute_lane_[i].lane_attribute &
               (int)LaneAttribute::INTERSECTION) != 0);
  }
  return false;
}

bool RefLineManager::transform() {
  if (refline_attribute_point_.empty())
    return false;
  s_vector_.clear();
  lane_width_vector_.clear();
  left_road_border_distance_vector_.clear();
  right_road_border_distance_vector_.clear();
  left_lane_border_distance_vector_.clear();
  right_lane_border_distance_vector_.clear();
  curvature_vector_.clear();
  s_vector_.resize(refline_attribute_point_.size());
  lane_width_vector_.resize(refline_attribute_point_.size());
  left_road_border_distance_vector_.resize(refline_attribute_point_.size());
  right_road_border_distance_vector_.resize(refline_attribute_point_.size());
  left_lane_border_distance_vector_.resize(refline_attribute_point_.size());
  right_lane_border_distance_vector_.resize(refline_attribute_point_.size());
  curvature_vector_.resize(refline_attribute_point_.size());
  std::transform(refline_attribute_point_.begin(),
                 refline_attribute_point_.end(), s_vector_.begin(),
                 [](RefLineAttributePoint point) { return point.s; });
  std::transform(refline_attribute_point_.begin(),
                 refline_attribute_point_.end(), lane_width_vector_.begin(),
                 [](RefLineAttributePoint point) { return point.lane_width; });
  std::transform(refline_attribute_point_.begin(),
                 refline_attribute_point_.end(),
                 left_road_border_distance_vector_.begin(),
                 [](RefLineAttributePoint point) {
                   return point.left_road_border_distance;
                 });
  std::transform(refline_attribute_point_.begin(),
                 refline_attribute_point_.end(),
                 right_road_border_distance_vector_.begin(),
                 [](RefLineAttributePoint point) {
                   return point.right_road_border_distance;
                 });
  std::transform(refline_attribute_point_.begin(),
                 refline_attribute_point_.end(),
                 left_lane_border_distance_vector_.begin(),
                 [](RefLineAttributePoint point) {
                   return point.left_lane_border_distance;
                 });
  std::transform(refline_attribute_point_.begin(),
                 refline_attribute_point_.end(),
                 right_lane_border_distance_vector_.begin(),
                 [](RefLineAttributePoint point) {
                   return point.right_lane_border_distance;
                 });
  std::transform(refline_attribute_point_.begin(),
                 refline_attribute_point_.end(), curvature_vector_.begin(),
                 [](RefLineAttributePoint point) { return point.curvature; });

  return true;
}

} // namespace parking

} // namespace msquare