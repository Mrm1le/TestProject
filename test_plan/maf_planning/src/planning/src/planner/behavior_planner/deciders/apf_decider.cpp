#include "planner/behavior_planner/deciders/apf_decider.h"
#include "common/math/box2d.h"
#include "common/math/math_utils.h"
#include "mjson/mjson.hpp"
#include "mlog_core/mlog_data_stream.h"
#include <boost/filesystem.hpp>

namespace msquare {

namespace parking {

using planning_math::Box2d;
using planning_math::LineSegment2d;
using planning_math::NormalizeAngle;
using planning_math::Vec2d;

ApfDecider::ApfDecider(const std::shared_ptr<WorldModel> &world_model) {
  world_model_ = world_model;
  traj_tag_ = "normal";
  apf_decider_cfg_ = ApfDeciderConfig::Instance();
  if (apf_decider_cfg_->is_parking_lot_from_file_) {
    load_parking_lot_file();
  }
  is_turning_ = false;
  clear_info();
  clear_traj();
  clear_traj_old();
  if (apf_decider_cfg_->is_store_) {
    boost::filesystem::path path(apf_decider_cfg_->store_path_);
    if (apf_decider_cfg_->clear_files_) {
      boost::filesystem::remove_all(path);
    }
    if (!boost::filesystem::exists(path)) {
      (void)boost::filesystem::create_directories(path);
    }
  }
}

ApfDecider::~ApfDecider() {}

bool ApfDecider::calculate() {
  traj_tag_ = "normal";
  bool find_road_theta = false;

  // start debug
  double apf_start = MTIME()->timestamp().sec();
  // std::cout << "zjt debug: start apf calculate" << std::endl;

  // replace old trajectory
  replace_old_traj();

  // clear info and traj
  clear_info();
  clear_traj();

  // gather info
  gather_ego_state();
  gather_map_info();

  // pre calc
  calc_apf_start_pose();

  // double calc_start =
  //     MTIME()->timestamp().sec();
  // is_road_theta_exist_ = calc_road_direction();
  is_road_theta_exist_ = calc_road_direction_two();
  // double calc_end =
  //     MTIME()->timestamp().sec();
  // std::cout<< "is road theta exist : " << boolalpha << is_road_theta_exist_<<
  // std::endl;
  plan();
  (void)check_traj_status();
  if (need_replan_) {
    clear_traj();
    clear_traj_old();
    reset_status();
    calc_apf_start_pose();
    is_road_theta_exist_ = calc_road_direction_two();
    replan();
    (void)check_traj_status();
  }

  set_status_and_info(false);
  add_historical_traj_point();
  // print list
  // print_traj_result();

  // store
  if (apf_decider_cfg_->is_store_) {
    store_count_++;
    if (store_count_ >= apf_decider_cfg_->store_freq_div_) {
      double store_start = MTIME()->timestamp().sec();
      store_num_++;
      store_scenery();
      double store_end = MTIME()->timestamp().sec();
      auto store_time = store_end - store_start;
      // cout << "time cost: store_time " << store_time << std::endl;
      store_count_ = 0;
    }
  }

  // debug end
  double apf_end = MTIME()->timestamp().sec();
  auto apf_time = apf_end - apf_start;
  // cout << "time cost: apf_time " << apf_time << std::endl;

  return is_traj_reasonable_;
}

void ApfDecider::plan() {

  // std::cout << "plan: " << std::endl;
  calc_apf_trajectory();
  set_traj_points_from_apf();
}

void ApfDecider::replan() {
  // std::cout << "replan: " << std::endl;
  calc_apf_trajectory();
  set_traj_points_from_apf();
}

void ApfDecider::reset_all() {
  reset_status();
  clear_traj();
  clear_traj_old();
  clear_info();
  // PlanningContext::Instance()->mutable_last_apf_path()->stash(get_result());
}

void ApfDecider::reset_status() {

  need_replan_ = false;
  is_traj_reasonable_ = false;
  is_traj_changed_ = true;
  is_turning_ = false;
  in_parking_lot_ = false;

  traj_status_.use_apf_refline_ = apf_decider_cfg_->use_apf_refline_;
  traj_status_.traj_size_ = 0;
  traj_status_.empty_ = true;
  traj_status_.changed_ = false;
  traj_status_.traj_s_ = 0.0;
  traj_status_.reasonable_ = false;
  traj_status_.historical_ = false;
  traj_status_.in_parking_lot_ = false;
  // ApfStatus *mutable_apf_status =
  //     PlanningContext::Instance()->mutable_apf_status();
  last_traj_status_ = traj_status_;
}

void ApfDecider::clear_traj() {
  apf_trajectory_.clear();
  traj_points_.clear();

  forward_traj_.clear();
  left_traj_.clear();
  right_traj_.clear();
}

void ApfDecider::clear_info() {
  apf_wall_vector_.clear();
  apf_pillar_vector_.clear();
  apf_parking_lot_line_vector_.clear();
  ground_point_vector_.clear();
  parking_lots_detection_fusion_results_.clear();
}

void ApfDecider::clear_traj_old() {
  apf_trajectory_old_.clear();
  traj_points_old_.clear();
}

void ApfDecider::gather_map_info() {
  // std::cout << "gather map info" << std::endl;
  // clear old info
  apf_wall_vector_.clear();
  apf_pillar_vector_.clear();
  apf_parking_lot_line_vector_.clear();
  ground_point_vector_.clear();
  parking_lots_detection_fusion_results_.clear();

  // get ground points
  for (auto point : world_model_->obstacle_manager().get_points().Items()) {
    //   std::cout<< "ground_points: " <<
    //   point->PerceptionBoundingBox().center().x()<<" "<<
    //   point->PerceptionBoundingBox().center().y()<< std::endl;
    ground_point_vector_.emplace_back(point->PerceptionBoundingBox().center());
  }

  // parking lot detection
  if (!apf_decider_cfg_->is_parking_lot_from_file_) {
    const std::vector<ParkingLotDetectionInfo> &detection_fusion_results =
        world_model_->get_parking_map_info()
            .parking_lots_detection_fusion_results;
    if (!detection_fusion_results.empty()) {
      parking_lots_detection_fusion_results_.assign(
          detection_fusion_results.begin(), detection_fusion_results.end());
      parking_lot_map_.clear();
      for (auto info : parking_lots_detection_fusion_results_) {
        if (info.is_good && info.corners.size() >= 4) {
          SimpleParkingLot parking_lot(info);
          parking_lot_map_.insert({parking_lot.id(), parking_lot});
        }
      }
    } else {
      parking_lot_map_.clear();
    }
    // if (!detection_fusion_results.empty()) {
    //   parking_lots_detection_fusion_results_.assign(
    //       detection_fusion_results.begin(), detection_fusion_results.end());
    // for (auto info : parking_lots_detection_fusion_results_) {
    //   if (info.is_good && info.corners.size() >= 4) {
    //     SimpleParkingLot parking_lot(info);
    //     if (parking_lot_map_.empty()) {
    //       parking_lot_map_.insert({parking_lot.id(), parking_lot});
    //     } else if (parking_lot_map_.find(parking_lot.id()) !=
    //                parking_lot_map_.end()) {
    //       // if (parking_lot.average_corner_confidence() >=
    //       //     parking_lot_map_[parking_lot.id()]
    //       // .average_corner_confidence()) {
    //       parking_lot_map_[parking_lot.id()] = parking_lot;
    //       // }
    //     } else {
    //       parking_lot_map_.insert({parking_lot.id(), parking_lot});
    //     }
    //   }
    // }
    // }
  }

  // get wall
  for (size_t i = 0;
       i < world_model_->get_square_map_response_avp().road_borders.size();
       i++) {
    if (world_model_->get_square_map_response_avp().road_borders[i].type ==
        LaneLineType::PHYSICAL) {
      for (size_t j = 1; j < world_model_->get_square_map_response_avp()
                                 .road_borders[i]
                                 .pts.size();
           j++) {
        auto pt1 = world_model_->get_square_map_response_avp()
                       .road_borders[i]
                       .pts[j - 1];
        auto pt2 =
            world_model_->get_square_map_response_avp().road_borders[i].pts[j];
        LineSegment2d line_cur(Vec2d(pt1.x, pt1.y), Vec2d(pt2.x, pt2.y));
        apf_wall_vector_.emplace_back(line_cur);
        // std::cout << "zjt wall: " << pt1.x << " " << pt1.y << " " << pt2.x
        //           << " " << pt2.y << " z: " << pt1.z << std::endl;
      }
    }
  }
  // get pillar
  for (int i = 0;
       i < world_model_->get_square_map_response_avp().obstacles.pillar.size();
       i++) {
    for (int j = 1; j < world_model_->get_square_map_response_avp()
                            .obstacles.pillar[i]
                            .pts.size();
         j++) {
      auto pt1 = world_model_->get_square_map_response_avp()
                     .obstacles.pillar[i]
                     .pts[j - 1];
      auto pt2 = world_model_->get_square_map_response_avp()
                     .obstacles.pillar[i]
                     .pts[j];
      LineSegment2d line_cur(Vec2d(pt1.x, pt1.y), Vec2d(pt2.x, pt2.y));
      apf_pillar_vector_.emplace_back(line_cur);
      // std::cout << "zjt pillar: " << pt1.x << " " << pt1.y << " " << pt2.x
      //           << " " << pt2.y << " z: " << pt1.z << std::endl;
    }
    auto pt1 = world_model_->get_square_map_response_avp()
                   .obstacles.pillar[i]
                   .pts.front();
    auto pt2 = world_model_->get_square_map_response_avp()
                   .obstacles.pillar[i]
                   .pts.back();
    LineSegment2d line_cur(Vec2d(pt1.x, pt1.y), Vec2d(pt2.x, pt2.y));
    apf_pillar_vector_.emplace_back(line_cur);
    // std::cout << "zjt pillar: " << pt1.x << " " << pt1.y << " " << pt2.x << "
    // "
    //           << pt2.y << " z: " << pt1.z << std::endl;
  }
  // get parking lot
  // std::cout << "parking_lot_circle_center: "<< parking_lot_circle_center_.x()
  // <<" "<< parking_lot_circle_center_.y() << std::endl;

  // std::cout << "zjt debug: " << parking_lot_map_.size() << std::endl;
  for (auto element : parking_lot_map_) {
    auto parking_lot = element.second;
    // std::cout << "11111111111111111: "
    //           << parking_lot_circle_center_.x() << " "
    //           << parking_lot_circle_center_.y() << " "
    //           << parking_lot.center().x() << " "
    //           << parking_lot.center().y() << " "
    //           << (parking_lot.center().DistanceTo(parking_lot_circle_center_)
    //           <= apf_decider_cfg_->parking_lot_circle_radius_)
    //           << " 2222222222222: "
    //           << ego_z_ << " "
    //           << parking_lot.z() << " "
    //           << (abs(ego_z_ - parking_lot.z()) <=
    //           apf_decider_cfg_->praking_lot_z_deviation_)
    //           << std::endl;
    if (parking_lot.center().DistanceTo(parking_lot_circle_center_) <=
            apf_decider_cfg_->parking_lot_circle_radius_ &&
        abs(ego_z_ - parking_lot.z()) <=
            apf_decider_cfg_->praking_lot_z_deviation_) {
      // std::cout << "parking_lot: " << parking_lot.center().x() <<" "<<
      // parking_lot.center().y()<< std::endl;
      for (auto parking_lot_line : parking_lot.lines()) {
        apf_parking_lot_line_vector_.emplace_back(parking_lot_line);
      }
    }
  }
}

void ApfDecider::gather_obstacle_info() {
  int count = 0;
  const ObstacleManager &obs_manager = world_model_->obstacle_manager();
  for (auto &obs : obs_manager.get_obstacles().Items()) {
    count++;
  }
}

void ApfDecider::gather_ego_state() {
  double x, y, theta;
  double circle_x, circle_y;
  ego_state_ = world_model_->get_ego_state();
  ego_pose_.theta = ego_state_.ego_pose.theta;
  ego_pose_.x = ego_state_.ego_pose.x;
  ego_pose_.y = ego_state_.ego_pose.y;
  ego_z_ = ego_state_.ego_enu.position.z;
  ego_center_ = planning_math::Vec2d(
      ego_pose_.x +
          apf_decider_cfg_->vehicle_length_added_ * std::cos(ego_pose_.theta),
      ego_pose_.y +
          apf_decider_cfg_->vehicle_length_added_ * std::sin(ego_pose_.theta));
  ego_box_ = planning_math::Box2d(ego_center_, ego_pose_.theta,
                                  apf_decider_cfg_->vehicle_length_,
                                  apf_decider_cfg_->vehicle_width_);
  circle_x = ego_center_.x() + apf_decider_cfg_->parking_lot_circle_offset_ *
                                   std::cos(ego_pose_.theta);
  circle_y = ego_center_.y() + apf_decider_cfg_->parking_lot_circle_offset_ *
                                   std::sin(ego_pose_.theta);
  parking_lot_circle_center_ = planning_math::Vec2d(circle_x, circle_y);
  // std::cout << "circle_center: " <<circle_x << " " << circle_y<< endl;
  // std::cout << "gather ego state" << std::endl;
  // std::cout << "ego_theta: " << ego_pose_.theta << " ego pose: x "
  //           << ego_pose_.x << " y: " << ego_pose_.y << " z: " << ego_z_
  //           << " ego_center: x " << ego_center_.x() << " y " <<
  //           ego_center_.y()
  //           << std::endl;
}

void ApfDecider::load_parking_lot_file() {
  double load_file_start = MTIME()->timestamp().sec();
  // std::cout << "zjt debug" << std::endl;
  // std::cout << "file_path: " << apf_decider_cfg_->parking_lot_file_
  //           << std::endl;
  std::ifstream input_file(apf_decider_cfg_->parking_lot_file_);
  mlog::MLogDataStream tmp;
  tmp << input_file.rdbuf();
  std::string input_data = tmp.str();
  mjson::Json root_node;
  std::string err_info;
  root_node.parse(input_data, err_info);
  parking_lot_jsons_ = root_node.array_value();
  for (auto parking_lot_json : parking_lot_jsons_) {
    SimpleParkingLot parking_lot(parking_lot_json,
                                 apf_decider_cfg_->parking_lot_z_offset_);
    parking_lot_map_.insert({parking_lot.id(), parking_lot});
  }
  // debug end
  double load_file_end = MTIME()->timestamp().sec();
  auto load_file_time = load_file_end - load_file_start;
  cout << "time cost: load_file_time: " << load_file_time << std::endl;
}

bool ApfDecider::calc_road_direction() {

  // std::cout << "road direction calculate " << std::endl;
  std::vector<SimpleParkingLot> parking_lot_considered_vector;
  std::vector<SimpleParkingLot> parking_lot_left_vector;
  std::vector<SimpleParkingLot> parking_lot_right_vector;
  std::vector<LineSegment2d> wall_considered_vector;
  std::vector<double> parking_lot_dis_vector;
  double init_theta = 0.0;
  bool has_wall = false;
  // gather information
  for (auto wall : apf_wall_vector_) {
    if (wall.DistanceTo(ego_center_) <
        apf_decider_cfg_->road_direction_circle_radius_wall_) {
      if (wall.length() >=
          apf_decider_cfg_->road_direction_wall_considered_length_) {
        wall_considered_vector.emplace_back(wall);
        // std::cout << "wall: "<< wall.start().x() << " " << wall.start().y()
        // << " " << wall.end().x() << " " << wall.end().y()<< " "<<
        // wall.heading()<< std::endl;
      }
    }
  }
  for (auto element : parking_lot_map_) {
    auto parking_lot = element.second;
    if (parking_lot.Box().DistanceTo(ego_center_) <=
        apf_decider_cfg_->road_direction_circle_radius_lot_) {
      parking_lot_considered_vector.emplace_back(parking_lot);
      parking_lot_dis_vector.emplace_back(
          parking_lot.Box().DistanceTo(ego_center_));
    }
  }
  // init road direction
  // std::cout << "calc init theta" << std::endl;
  if (!wall_considered_vector.empty()) {
    init_theta =
        abs(NormalizeAngle(wall_considered_vector[0].heading() -
                           ego_pose_.theta)) > M_PI_2
            ? NormalizeAngle(wall_considered_vector[0].heading() + M_PI)
            : wall_considered_vector[0].heading();
    if (abs(NormalizeAngle(init_theta - ego_pose_.theta)) > M_PI / 4.0) {
      // std::cout << " road theta: The heading angle deviates from the road "
      //              "structure, can not calc road direction"
      //           << std::endl;
      return false;
    }
    if (wall_considered_vector.size() >= 2) {
      for (int index = 1; index < wall_considered_vector.size(); index++) {
        if (abs(NormalizeAngle(wall_considered_vector[index].heading() -
                               init_theta)) < M_PI / 10.0) {
          init_theta = NormalizeAngle(
              (init_theta * (index + 1) +
               NormalizeAngle(wall_considered_vector[index].heading() -
                              init_theta)) /
              (index + 1));
        } else if (abs(NormalizeAngle(wall_considered_vector[index].heading() +
                                      M_PI - init_theta)) < M_PI / 10.0) {
          init_theta = NormalizeAngle(
              (init_theta * (index + 1) +
               NormalizeAngle(wall_considered_vector[index].heading() + M_PI -
                              init_theta)) /
              (index + 1));
        } else {
          // std::cout
          //     << "road theta: at intersection, can not calc road direction"
          //     << std::endl;
          return false;
        }
      }
    }
    has_wall = true;
    // std::cout << "determined init road theta from wall" << std::endl;

  } else {
    if (!parking_lot_considered_vector.empty()) {
      std::vector<double>::iterator min_distance = std::min_element(
          parking_lot_dis_vector.begin(), parking_lot_dis_vector.end());
      if (*min_distance < 1.5) {
        // std::cout << "road theta: try to determined init road theat from ego
        // "
        //              "pose, but some parking lots are too close"
        //           << std::endl;
        return false;
      } else {
        // std::cout << "determined init road theta from ego pose" << std::endl;
        // init_theta = ego_pose_.theta;
      }
    } else {
      // std::cout << "road theta: no wall and no parking lot, can not calc road
      // "
      //              "direction"
      //           << std::endl;
      return false;
    }
  }
  // std::cout << "init theta: " << init_theta << std::endl;

  // divide parking lot
  // std::cout << "divide parking lot" << std::endl;
  for (auto parking_lot : parking_lot_considered_vector) {
    std::vector<double> ls;
    ls.clear();
    for (auto corner : parking_lot.corners()) {
      double l =
          (corner.x() - ego_center_.x()) * std::cos(init_theta + M_PI_2) +
          (corner.y() - ego_center_.y()) * std::sin(init_theta + M_PI_2);
      ls.emplace_back(l);
    }
    std::vector<double>::iterator min_l =
        std::min_element(ls.begin(), ls.end());
    std::vector<double>::iterator max_l =
        std::max_element(ls.begin(), ls.end());
    if (*min_l < 0 && *max_l < 0) {
      parking_lot_right_vector.emplace_back(parking_lot);
      // std::cout<<"right parking lot: "<< *min_l << *max_l<< std::endl;
    } else if (*min_l > 0 && *max_l > 0) {
      parking_lot_left_vector.emplace_back(parking_lot);
      // std::cout<<"left parking lot: "<< *min_l << *max_l<< std::endl;
    } else {
      // std::cout << "road theta: can not divede parking lot, can not calc road
      // "
      //              "direction"
      //           << std::endl;
      return false;
    }
  }
  // std::cout << "divided parking loat, left parking lot num: "
  //           << parking_lot_left_vector.size()
  //           << " right parking lot num: " << parking_lot_right_vector.size()
  //           << std::endl;
  // check parking lot direction
  // std::cout << "check parking lot direction" << std::endl;
  if (has_wall) {
    for (auto parking_lot : parking_lot_considered_vector) {
      if (abs(NormalizeAngle(4 * (parking_lot.theta() - init_theta))) >
          M_PI / 3.0) {
        // std::cout << " road theta: parking lot dirction error " << std::endl;
        // std::cout << " init theta from wall: " << init_theta
        //           << " parking lot theta: " << parking_lot.theta() <<
        //           std::endl;
        return false;
      }
    }
  } else {
    for (int index = 0; index < parking_lot_considered_vector.size(); index++) {
      if (abs(NormalizeAngle(4 * (parking_lot_considered_vector[index].theta() -
                                  init_theta))) <
          max(M_PI / (index + 1), M_PI / 3.0)) {
        double dtheta =
            NormalizeAngle(4 * (parking_lot_considered_vector[index].theta() -
                                init_theta)) /
            4;
        init_theta = NormalizeAngle(init_theta + dtheta / (index + 1));
      } else {
        // std::cout << " road theta: parking lot dirction error " << std::endl;
        // std::cout << " init theta from ego: " << ego_pose_.theta
        //           << " parking lot theta: "
        //           << parking_lot_considered_vector[index].theta() <<
        //           std::endl;
        return false;
      }
    }
  }
  road_theta_ = init_theta;
  // std::cout << "road theta: " << road_theta_ << " has wall: " << boolalpha
  //           << has_wall << std::endl;

  return true;
}

bool ApfDecider::calc_road_direction_two() {

  // std::cout << "road direction calculate from psd and parking lot "
  //           << std::endl;
  std::vector<SimpleParkingLot> parking_lot_considered_vector;
  std::vector<SimpleParkingLot> parking_lot_left_vector;
  std::vector<SimpleParkingLot> parking_lot_right_vector;
  std::vector<double> parking_lot_dis_vector;
  double init_theta = 0.0;
  road_theta_ = ego_pose_.theta;

  // gather parking_lot
  for (auto element : parking_lot_map_) {
    auto parking_lot = element.second;
    if (parking_lot.Box().DistanceTo(ego_center_) <=
        apf_decider_cfg_->road_direction_circle_radius_lot_) {
      parking_lot_considered_vector.emplace_back(parking_lot);
      parking_lot_dis_vector.emplace_back(
          parking_lot.Box().DistanceTo(ego_center_));
    }
  }

  // init theta from nearest parking lot
  if (parking_lot_considered_vector.empty()) {
    // std::cout << "no parking lot, no change of road theta" << std::endl;
    return false;
  }
  int nearest_index =
      std::distance(parking_lot_dis_vector.begin(),
                    std::min_element(parking_lot_dis_vector.begin(),
                                     parking_lot_dis_vector.end()));
  SimpleParkingLot nearest_parking_lot =
      parking_lot_considered_vector.at(nearest_index);
  for (int i = 0; i < 4; i++) {
    if (abs(NormalizeAngle(nearest_parking_lot.theta() + i * M_PI_2 -
                           ego_pose_.theta)) <= M_PI / 4) {
      init_theta = nearest_parking_lot.theta() + i * M_PI_2;
      break;
    }
  }

  // divide other parking lot
  for (auto parking_lot : parking_lot_considered_vector) {
    std::vector<double> ls;
    ls.clear();
    for (auto corner : parking_lot.corners()) {
      double l =
          (corner.x() - ego_center_.x()) * std::cos(init_theta + M_PI_2) +
          (corner.y() - ego_center_.y()) * std::sin(init_theta + M_PI_2);
      ls.emplace_back(l);
    }
    std::vector<double>::iterator min_l =
        std::min_element(ls.begin(), ls.end());
    std::vector<double>::iterator max_l =
        std::max_element(ls.begin(), ls.end());
    if (*min_l < 0 && *max_l < 0) {
      parking_lot_right_vector.emplace_back(parking_lot);
      // std::cout<<"right parking lot: "<< *min_l << *max_l<< std::endl;
    } else if (*min_l > 0 && *max_l > 0) {
      parking_lot_left_vector.emplace_back(parking_lot);
      // std::cout<<"left parking lot: "<< *min_l << *max_l<< std::endl;
    } else {
      // std::cout << "road theta: can not divede parking lot, can not calc road
      // "
      //              "direction, no change of road theta"
      //           << std::endl;

      return false;
    }
  }
  // std::cout << "divided parking loat, left parking lot num: "
  //           << parking_lot_left_vector.size()
  //           << " right parking lot num: " << parking_lot_right_vector.size()
  //           << std::endl;

  // check other parking lot directions
  // std::cout << "check parking lot direction" << std::endl;
  for (int index = 0; index < parking_lot_considered_vector.size(); index++) {
    if (abs(NormalizeAngle(
            4 * (parking_lot_considered_vector[index].theta() - init_theta))) <
        max(M_PI / (index + 1), M_PI / 3.0)) {
      double dtheta =
          NormalizeAngle(
              4 * (parking_lot_considered_vector[index].theta() - init_theta)) /
          4;
      init_theta = NormalizeAngle(init_theta + dtheta / (index + 1));
    } else {
      // std::cout << " road theta: parking lot dirction error " << std::endl;
      // std::cout << " init theta: " << init_theta << " parking lot theta: "
      //           << parking_lot_considered_vector[index].theta() << std::endl;
      return false;
    }
  }

  road_theta_ = init_theta;
  // std::cout << "road theta: " << road_theta_ << std::endl;

  return true;
}

void ApfDecider::calc_apf_start_pose() {
  // for find start pose
  double min_dis = 10.0;
  ApfPoint start_point;
  ApfTrajectory::iterator it;
  if (!apf_trajectory_old_.empty()) {
    for (it = apf_trajectory_old_.begin(); it != apf_trajectory_old_.end();
         ++it) {
      if (distance(it->pose, ego_pose_) < min_dis) {
        min_dis = distance(it->pose, ego_pose_);
        start_point = *it;
      }
    }
    if (min_dis <= apf_decider_cfg_->apf_replan_threshold_) {
      apf_start_pose_.x = start_point.pose.x;
      apf_start_pose_.y = start_point.pose.y;
      apf_start_pose_.theta = start_point.pose.theta;
      // std::cout << "set apf start point from last apf traj" << std::endl;
    } else {
      apf_start_pose_.x = ego_pose_.x;
      apf_start_pose_.y = ego_pose_.y;
      apf_start_pose_.theta = ego_pose_.theta;
      // std::cout << "set apf start point from ego pose" << std::endl;
    }
  } else {
    apf_start_pose_.x = ego_pose_.x;
    apf_start_pose_.y = ego_pose_.y;
    apf_start_pose_.theta = ego_pose_.theta;
    // std::cout << "set apf start point from ego pose" << std::endl;
  }
  // std::cout << "start pose: x " << apf_start_pose_.x << " y "
  //           << apf_start_pose_.y << " theta " << apf_start_pose_.theta
  //           << std::endl;
}

bool ApfDecider::check_traj_status() {
  // const ApfStatus last_apf_status =
  // PlanningContext::Instance()->apf_status();

  if (traj_points_old_.empty()) {
    is_traj_changed_ = true;
  } else {
    if (abs(traj_points_.back().path_point.x -
            traj_points_old_.back().path_point.x) > 3.0 ||
        abs(traj_points_.back().path_point.y -
            traj_points_old_.back().path_point.y) > 3.0 ||
        abs(traj_points_.back().path_point.theta -
            traj_points_old_.back().path_point.theta) > M_1_PI / 4) {
      is_traj_changed_ = true;
    } else {
      is_traj_changed_ = false;
    }
  }

  if (traj_points_.size() <= 3 || traj_points_.empty()) {

    traj_s_ = 0;
  } else {
    double sum_s = 0.0;
    double dis, cruvature;
    for (int i = 1; i < traj_points_.size(); i++) {
      dis =
          distance(traj_points_[i - 1].path_point, traj_points_[i].path_point);
      cruvature = abs(NormalizeAngle(traj_points_[i].path_point.theta -
                                     traj_points_[i - 1].path_point.theta)) /
                  dis;
      sum_s += dis;
      if (cruvature > 1 / apf_decider_cfg_->min_turning_radius_ * 2) {
        is_traj_reasonable_ = false;
        // std::cout << "hhhhh" << std::endl;
      }
    }
    traj_s_ = sum_s;
  }
  if (traj_s_ < apf_decider_cfg_->min_reasonable_s_) {
    is_traj_reasonable_ = false;
  } else {
    is_traj_reasonable_ = true;
  }

  if (!is_traj_reasonable_ && !last_traj_status_.reasonable_) {
    need_replan_ = true;
  } else {
    need_replan_ = false;
  }
  return is_traj_reasonable_;
}

void ApfDecider::print_traj_result() {

  // std::cout << "zjt traj result: "
  //           << "is_teb_replan_: " << is_teb_replan_ << std::endl;
  // if (!apf_trajectory_.empty()) {
  //   std::cout << "zjt traj result: has apf traj " << std::endl;
  //   for (int index = 0; index < apf_trajectory_.size(); index++) {
  //     std::cout << "zjt traj result: "
  //               << "apf point: "
  //               << "point id: " << index << " x "
  //               << apf_trajectory_[index].pose.x << " y "
  //               << apf_trajectory_[index].pose.y << " theta "
  //               << apf_trajectory_[index].pose.theta << std::endl;
  //   }
  // } else {
  //   std::cout << "zjt traj result: no apf traj " << std::endl;
  // }

  // if (!teb_trajectory_.empty()) {
  //   std::cout << "zjt traj result: has teb traj " << std::endl;
  //   for (int index = 0; index < teb_trajectory_.size(); index++) {
  //     std::cout << "zjt traj result: "
  //               << "teb point: "
  //               << "point id: " << index << " x " <<
  //               teb_trajectory_[index].x()
  //               << " y " << teb_trajectory_[index].y() << " theta "
  //               << teb_trajectory_[index].theta() << std::endl;
  //   }
  // } else {
  //   std::cout << "zjt traj result: no teb traj " << std::endl;
  // }
  // if (!traj_points_old_.empty())
  // {
  //     std::cout << "zjt traj result: has real traj " << std::endl;
  //     for (int index = 0;index <  traj_points_old_.size(); index ++)
  //     {
  //         std::cout << "zjt traj result: " << " traj point: " << "point id: "
  //         << index << " x " << traj_points_old_[index].path_point.x << " y "
  //         << traj_points_old_[index].path_point.y << " theta " <<
  //         traj_points_old_[index].path_point.theta << std::endl;
  //     }
  // }
  // else{
  //     std::cout << "zjt traj result: no real traj " << std::endl;
  // }
}

void ApfDecider::set_status_and_info(bool historical) {
  // set status
  traj_status_.historical_ = historical;
  if (historical) {
    traj_status_.use_apf_refline_ = apf_decider_cfg_->use_apf_refline_;
    traj_status_.traj_size_ = traj_points_.size();
    traj_status_.empty_ = traj_points_.empty();
    traj_status_.changed_ = false;
    traj_status_.traj_s_ = traj_s_;
    traj_status_.reasonable_ = true;
    traj_status_.in_parking_lot_ = in_parking_lot_;
  } else {
    traj_status_.use_apf_refline_ = apf_decider_cfg_->use_apf_refline_;
    traj_status_.traj_size_ = traj_points_.size();
    traj_status_.empty_ = traj_points_.empty();
    traj_status_.changed_ = is_traj_changed_;
    traj_status_.traj_s_ = traj_s_;
    traj_status_.reasonable_ = is_traj_reasonable_;
    traj_status_.in_parking_lot_ = in_parking_lot_;
  }

  // set plannig context status
  // ApfStatus *mutable_apf_status =
  //     PlanningContext::Instance()->mutable_apf_status();
  last_traj_status_ = traj_status_;

  // set info
  traj_info_.status = traj_status_;
  traj_info_.tag = traj_tag_;
  traj_info_.target_pose = traj_target_pose_;
  traj_info_.traj = traj_points_;

  // print list
  // std::cout << "zjt debug apf status : reasonable: " << boolalpha
  //           << traj_status_.reasonable_ << " empty: " << boolalpha
  //           << traj_status_.empty_ << " size: " << traj_status_.traj_size_
  //           << " traj_s: " << traj_status_.traj_s_ << " changed: " <<
  //           boolalpha
  //           << traj_status_.changed_ << std::endl;
}

void ApfDecider::replace_old_traj() {
  apf_trajectory_old_.assign(apf_trajectory_.begin(), apf_trajectory_.end());
  traj_points_old_.assign(traj_points_.begin(), traj_points_.end());
}

void ApfDecider::set_traj_points_from_apf() {
  int id = 0;
  traj_points_.clear();
  if (apf_decider_cfg_->move_apf_traj_) {
    double ds = apf_decider_cfg_->move_apf_traj_ds_;
    for (auto apf_point : apf_trajectory_) {
      TrajectoryPoint point(
          apf_point.pose.x + ds * cos(apf_point.pose.theta - M_PI_2),
          apf_point.pose.y + ds * sin(apf_point.pose.theta - M_PI_2), 0,
          apf_point.pose.theta, 0, 1, 0.5, 0, 0);
      traj_points_.emplace_back(point);
      // std::cout << "point id: " << id << " x: " << apf_point.pose.x << " y: "
      // << apf_point.pose.y  << " theta: " << apf_point.pose.theta <<
      // std::endl;
      id++;
    }
  } else {
    for (auto apf_point : apf_trajectory_) {
      TrajectoryPoint point(apf_point.pose.x, apf_point.pose.y, 0,
                            apf_point.pose.theta, 0, 1, 0.5, 0, 0);
      traj_points_.emplace_back(point);
      // std::cout << "point id: " << id << " x: " << apf_point.pose.x << " y: "
      // << apf_point.pose.y  << " theta: " << apf_point.pose.theta <<
      // std::endl;
      id++;
    }
  }
  // std::cout << "zjt debug: traj size: " << traj_points_.size() << std::endl;
}

void ApfDecider::create_directional_gradient(Point2D &gradient, double theta,
                                             double intensity) {
  double heading = NormalizeAngle(theta + M_PI);
  gradient.x = std::cos(heading) * intensity;
  gradient.y = std::sin(heading) * intensity;
  // std::cout<<"dirctional gradient x: " <<  gradient.x << " dirctional
  // gradient y: " << gradient.y << std::endl;
}

void ApfDecider::create_gradient(Point2D &all_gradient,
                                 const Point2D &directional_gradient,
                                 const Pose2D &pose) {
  all_gradient.x = 0;
  all_gradient.y = 0;
  double repulsion_weight = 0;
  double rho0 = 0;
  double repulsion_2_weight = 0;
  double repulsion_2_attenuation = 0;
  double delta_s = 0.00001;
  double min_distance = 0;
  Point2D gradient;
  // bbox

  Vec2d center = planning_math::Vec2d(
      pose.x + apf_decider_cfg_->vehicle_length_added_ * std::cos(pose.theta),
      pose.y + apf_decider_cfg_->vehicle_length_added_ * std::sin(pose.theta));
  Box2d box = planning_math::Box2d(center, pose.theta,
                                   apf_decider_cfg_->vehicle_length_,
                                   apf_decider_cfg_->vehicle_width_);
  Vec2d center_dx = planning_math::Vec2d(
      delta_s + pose.x +
          apf_decider_cfg_->vehicle_length_added_ * std::cos(pose.theta),
      pose.y + apf_decider_cfg_->vehicle_length_added_ * std::sin(pose.theta));
  Box2d box_dx = planning_math::Box2d(center_dx, pose.theta,
                                      apf_decider_cfg_->vehicle_length_,
                                      apf_decider_cfg_->vehicle_width_);
  Vec2d center_dy = planning_math::Vec2d(
      pose.x + apf_decider_cfg_->vehicle_length_added_ * std::cos(pose.theta),
      delta_s + pose.y +
          apf_decider_cfg_->vehicle_length_added_ * std::sin(pose.theta));
  Box2d box_dy = planning_math::Box2d(center_dy, pose.theta,
                                      apf_decider_cfg_->vehicle_length_,
                                      apf_decider_cfg_->vehicle_width_);

  // wall gradient
  if (apf_decider_cfg_->use_map_potential_field_) {
    repulsion_weight = apf_decider_cfg_->wall_repulsion_weight_;
    rho0 = apf_decider_cfg_->wall_repulsion_max_distance_;
    repulsion_2_weight = apf_decider_cfg_->wall_repulsion_2_weight_;
    repulsion_2_attenuation = apf_decider_cfg_->wall_repulsion_2_attenuation_;
    for (auto wall : apf_wall_vector_) {
      min_distance = box.DistanceTo(wall);
      if (min_distance < 5 * repulsion_2_attenuation && min_distance > 0.03) {
        double u_0 =
            repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
        if (min_distance < rho0) {
          u_0 += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
                 (1 / min_distance - 1 / rho0);
        }
        min_distance = box_dx.DistanceTo(wall);
        double u_dx =
            repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
        if (min_distance < rho0) {
          u_dx += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
                  (1 / min_distance - 1 / rho0);
        }
        gradient.x = (u_dx - u_0) / delta_s;
        min_distance = box_dy.DistanceTo(wall);
        double u_dy =
            repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
        if (min_distance < rho0) {
          u_dy += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
                  (1 / min_distance - 1 / rho0);
        }
        gradient.y = (u_dy - u_0) / delta_s;
        // std::cout << "obstacle: "<< wall.start().x() << " " <<
        // wall.start().y()
        // << " " << wall.end().x() << " " << wall.end().y() << std::endl;
        // std::cout << "ego: "<< center.x() << " " << center.y() << " " <<
        // pose.theta  << std::endl; std::cout << "u_0: "<< u_0 << std::endl;
        // std::cout << "u_x: "<< u_dx<<std::endl;
        // std::cout << "gradient.x : "<< gradient.x<<std::endl;
        // std::cout << "gradient.y : "<< gradient.y<<std::endl;

      } else if (min_distance < 0.03) {
        gradient.x = 0;
        gradient.y = 0;
        // std::cout << "too close obstacle" << std::endl;
      } else {
        gradient.x = 0;
        gradient.y = 0;
      }
      all_gradient.x += gradient.x;
      all_gradient.y += gradient.y;
    }
  }
  // pillar gradient
  if (apf_decider_cfg_->use_map_potential_field_) {
    repulsion_weight = apf_decider_cfg_->pillar_repulsion_weight_;
    rho0 = apf_decider_cfg_->pillar_repulsion_max_distance_;
    repulsion_2_weight = apf_decider_cfg_->pillar_repulsion_2_weight_;
    repulsion_2_attenuation = apf_decider_cfg_->pillar_repulsion_2_attenuation_;
    for (auto pillar : apf_pillar_vector_) {
      min_distance = box.DistanceTo(pillar);
      if (min_distance < 5 * repulsion_2_attenuation && min_distance > 0.03) {
        double u_0 =
            repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
        if (min_distance < rho0) {
          u_0 += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
                 (1 / min_distance - 1 / rho0);
        }
        min_distance = box_dx.DistanceTo(pillar);
        double u_dx =
            repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
        if (min_distance < rho0) {
          u_dx += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
                  (1 / min_distance - 1 / rho0);
        }
        gradient.x = (u_dx - u_0) / delta_s;
        min_distance = box_dy.DistanceTo(pillar);
        double u_dy =
            repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
        if (min_distance < rho0) {
          u_dy += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
                  (1 / min_distance - 1 / rho0);
        }
        gradient.y = (u_dy - u_0) / delta_s;
        // std::cout << "obstacle: "<< pillar.start().x() << " " <<
        // pillar.start().y() << " " << pillar.end().x() << " " <<
        // pillar.end().y() << std::endl; std::cout << "ego: "<< center.x() << "
        // "
        // << center.y() << " " << pose.theta  << std::endl; std::cout << "u_0:
        // "<< u_0 << std::endl; std::cout << "u_x: "<< u_dx<<std::endl;
        // std::cout
        // << "gradient.x : "<< gradient.x<<std::endl;

      } else if (min_distance <= 0.03) {
        gradient.x = 0;
        gradient.y = 0;
        // std::cout << "too close obstacle" << std::endl;
      } else {
        gradient.x = 0;
        gradient.y = 0;
      }
      all_gradient.x += gradient.x;
      all_gradient.y += gradient.y;
    }
  }
  // parking lot gradient
  repulsion_weight = apf_decider_cfg_->parking_lot_repulsion_weight_;
  rho0 = apf_decider_cfg_->parking_lot_repulsion_max_distance_;
  repulsion_2_weight = apf_decider_cfg_->parking_lot_repulsion_2_weight_;
  repulsion_2_attenuation =
      apf_decider_cfg_->parking_lot_repulsion_2_attenuation_;
  for (auto parking_lot_line : apf_parking_lot_line_vector_) {
    min_distance = box.DistanceTo(parking_lot_line);
    if (min_distance < 5 * repulsion_2_attenuation && min_distance > 0.03) {
      double u_0 =
          repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
      if (min_distance < rho0) {
        u_0 += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
               (1 / min_distance - 1 / rho0);
      }
      min_distance = box_dx.DistanceTo(parking_lot_line);
      double u_dx =
          repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
      if (min_distance < rho0) {
        u_dx += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
                (1 / min_distance - 1 / rho0);
      }
      gradient.x = (u_dx - u_0) / delta_s;
      min_distance = box_dy.DistanceTo(parking_lot_line);
      double u_dy =
          repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
      if (min_distance < rho0) {
        u_dy += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
                (1 / min_distance - 1 / rho0);
      }
      gradient.y = (u_dy - u_0) / delta_s;
      // std::cout << "obstacle: "<< pillar.start().x() << " " <<
      // pillar.start().y() << " " << pillar.end().x() << " " <<
      // pillar.end().y() << std::endl; std::cout << "ego: "<< center.x() << " "
      // << center.y() << " " << pose.theta  << std::endl; std::cout << "u_0:
      // "<< u_0 << std::endl; std::cout << "u_x: "<< u_dx<<std::endl; std::cout
      // << "gradient.x : "<< gradient.x<<std::endl;
    } else if (min_distance <= 0.03) {
      gradient.x = 0;
      gradient.y = 0;
      // std::cout << "too close obstacle" << std::endl;
    } else {
      gradient.x = 0;
      gradient.y = 0;
    }
    all_gradient.x += gradient.x;
    all_gradient.y += gradient.y;
  }

  // ground point gradient
  if (apf_decider_cfg_->use_ground_point_potential_field_) {
    repulsion_weight = apf_decider_cfg_->ground_point_repulsion_weight_;
    rho0 = apf_decider_cfg_->ground_point_repulsion_max_distance_;
    repulsion_2_weight = apf_decider_cfg_->ground_point_repulsion_2_weight_;
    repulsion_2_attenuation =
        apf_decider_cfg_->ground_point_repulsion_2_attenuation_;
    for (auto ground_point : ground_point_vector_) {
      min_distance = box.DistanceTo(ground_point);
      if (min_distance < 5 * repulsion_2_attenuation && min_distance > 0.03) {
        double u_0 =
            repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
        if (min_distance < rho0) {
          u_0 += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
                 (1 / min_distance - 1 / rho0);
        }
        min_distance = box_dx.DistanceTo(ground_point);
        double u_dx =
            repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
        if (min_distance < rho0) {
          u_dx += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
                  (1 / min_distance - 1 / rho0);
        }
        gradient.x = (u_dx - u_0) / delta_s;
        min_distance = box_dy.DistanceTo(ground_point);
        double u_dy =
            repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
        if (min_distance < rho0) {
          u_dy += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
                  (1 / min_distance - 1 / rho0);
        }
        gradient.y = (u_dy - u_0) / delta_s;
        // std::cout << "obstacle: "<< pillar.start().x() << " " <<
        // pillar.start().y() << " " << pillar.end().x() << " " <<
        // pillar.end().y() << std::endl; std::cout << "ego: "<< center.x() << "
        // "
        // << center.y() << " " << pose.theta  << std::endl; std::cout << "u_0:
        // "<< u_0 << std::endl; std::cout << "u_x: "<< u_dx<<std::endl;
        // std::cout
        // << "gradient.x : "<< gradient.x<<std::endl;
      } else if (min_distance <= 0.03) {
        gradient.x = 0;
        gradient.y = 0;
        // std::cout << "too close obstacle" << std::endl;
      } else {
        gradient.x = 0;
        gradient.y = 0;
      }
      all_gradient.x += gradient.x;
      all_gradient.y += gradient.y;
    }
  }
  // directional gradient
  all_gradient.x += directional_gradient.x;
  all_gradient.y += directional_gradient.y;
}

void ApfDecider::get_apf_start_pose(Pose2D &pose) {
  pose.x = apf_start_pose_.x;
  pose.y = apf_start_pose_.y;
  pose.theta = apf_start_pose_.theta;
}

bool ApfDecider::create_new_pose(Pose2D &pose, const ApfPoint &apf_point) {
  double dtheta;
  dtheta = NormalizeAngle(apf_point.gradient_theta + M_PI - pose.theta);
  if (apf_point.gradient_value > apf_decider_cfg_->stop_gradient_value_ ||
      abs(dtheta) > M_PI / 2) {
    return 0;
  } else {
    pose.theta = NormalizeAngle(max(
        pose.theta - apf_decider_cfg_->max_dtheta_,
        min(pose.theta + dtheta, pose.theta + apf_decider_cfg_->max_dtheta_)));
    pose.x = apf_point.pose.x + apf_decider_cfg_->gradient_descent_distance_ *
                                    std::cos(pose.theta);
    pose.y = apf_point.pose.y + apf_decider_cfg_->gradient_descent_distance_ *
                                    std::sin(pose.theta);
    return 1;
  }
  return 1;
}

bool ApfDecider::create_apf_trajectory(Pose2D pose, double direction,
                                       int max_point_num, int min_point_num,
                                       ApfTrajectory &trajectory) {
  Point2D gradient;
  Point2D directional_gradient;
  trajectory.clear();
  create_directional_gradient(directional_gradient, direction,
                              apf_decider_cfg_->forward_gradient_intensity_);
  for (int index = 0; index < max_point_num; index++) {
    create_gradient(gradient, directional_gradient, pose);
    ApfPoint apf_point(pose, gradient);
    trajectory.emplace_back(apf_point);
    // std::cout<< pose.x << " " << pose.y << " "<< pose.theta<<std::endl;
    if (!create_new_pose(pose, apf_point)) {
      break;
    }
  }
  if (trajectory.size() >= min_point_num) {
    return true;
  } else {
    return false;
  }
}

void ApfDecider::calc_apf_trajectory() {
  Pose2D pose;
  Pose2D start_pose = apf_start_pose_;
  pose = start_pose;
  // for no turning mode
  // std::cout << "turning mode: " << is_turning_ << std::endl;
  if (is_turning_ == 0) {
    forward_traj_.clear();
    left_traj_.clear();
    right_traj_.clear();
    // find forward track first
    double forward_theta;
    ApfTrajectory turning_traj;
    if (is_road_theta_exist_) {
      forward_theta = road_theta_;
    } else {
      forward_theta = ego_pose_.theta;
    }
    forward_traj_status_ = create_apf_trajectory(
        pose, forward_theta, apf_decider_cfg_->max_trajectory_points_num_,
        apf_decider_cfg_->min_trajectory_points_num_, forward_traj_);
    // check forward track
    if (forward_traj_status_) {
      apf_trajectory_.assign(forward_traj_.begin(), forward_traj_.end());
      // std::cout << "turning decision: find forward road " << std::endl;
    } else {
      if (forward_traj_.size() > apf_decider_cfg_->back_points_num_) {
        pose =
            (*(forward_traj_.end() - apf_decider_cfg_->back_points_num_)).pose;
        right_traj_.assign(forward_traj_.begin(),
                           forward_traj_.end() -
                               apf_decider_cfg_->back_points_num_);
        left_traj_.assign(forward_traj_.begin(),
                          forward_traj_.end() -
                              apf_decider_cfg_->back_points_num_);
      } else {
        pose = start_pose;
      }
      left_traj_status_ = create_apf_trajectory(
          pose,
          forward_theta +
              M_PI_2 * apf_decider_cfg_->turning_gradient_dtheta_ratio_,
          apf_decider_cfg_->max_trajectory_points_num_,
          apf_decider_cfg_->min_turning_trajectory_points_num_, turning_traj);

      left_traj_.insert(left_traj_.end(), turning_traj.begin(),
                        turning_traj.end());
      right_traj_status_ = create_apf_trajectory(
          pose,
          forward_theta -
              M_PI_2 * apf_decider_cfg_->turning_gradient_dtheta_ratio_,
          apf_decider_cfg_->max_trajectory_points_num_,
          apf_decider_cfg_->min_turning_trajectory_points_num_, turning_traj);
      right_traj_.insert(right_traj_.end(), turning_traj.begin(),
                         turning_traj.end());
      if (left_traj_status_) {
        apf_trajectory_.assign(left_traj_.begin(), left_traj_.end());
        turning_direction_ =
            forward_theta +
            M_PI_2 * apf_decider_cfg_->turning_gradient_dtheta_ratio_;
        forward_direction_ = forward_theta;
        is_turning_ = 1;
        // std::cout << "turning decision: enter pre turn left  " << std::endl;
      } else if (right_traj_status_) {
        apf_trajectory_.assign(right_traj_.begin(), right_traj_.end());
        turning_direction_ =
            forward_theta -
            M_PI_2 * apf_decider_cfg_->turning_gradient_dtheta_ratio_;
        forward_direction_ = forward_theta;
        is_turning_ = 1;
        // std::cout << "turning decision: enter pre turn right" << std::endl;
      } else {
        apf_trajectory_.assign(forward_traj_.begin(), forward_traj_.end());
        // std::cout << "turning decision: forward road short, but no turnning"
        //           << std::endl;
      }
    }
  } else if (is_turning_ == 1) {
    ApfTrajectory forward_traj;
    bool forward_traj_status;
    ApfTrajectory turning_traj;
    bool turning_traj_status;
    forward_traj_status = create_apf_trajectory(
        pose, forward_direction_, apf_decider_cfg_->max_trajectory_points_num_,
        apf_decider_cfg_->min_trajectory_points_num_, forward_traj);
    if (forward_traj.size() > apf_decider_cfg_->back_points_num_) {
      pose = (*(forward_traj.end() - apf_decider_cfg_->back_points_num_)).pose;
      apf_trajectory_.assign(forward_traj.begin(),
                             forward_traj.end() -
                                 apf_decider_cfg_->back_points_num_);
      // std::cout << "turning decision: keep pre turning" << std::endl;
    } else {
      pose = start_pose;
      is_turning_ = 2;
      // std::cout << "turning decision: enter turning" << std::endl;
    }
    turning_traj_status = create_apf_trajectory(
        pose, turning_direction_, apf_decider_cfg_->max_trajectory_points_num_,
        apf_decider_cfg_->min_turning_trajectory_points_num_, turning_traj);
    apf_trajectory_.insert(apf_trajectory_.end(), turning_traj.begin(),
                           turning_traj.end());
  } else {
    ApfTrajectory turning_traj;
    bool turning_traj_status;
    turning_traj_status = create_apf_trajectory(
        pose, turning_direction_, apf_decider_cfg_->max_trajectory_points_num_,
        apf_decider_cfg_->min_turning_trajectory_points_num_, turning_traj);
    apf_trajectory_.insert(apf_trajectory_.end(), turning_traj.begin(),
                           turning_traj.end());
    if (abs(NormalizeAngle(ego_pose_.theta - turning_direction_)) <
        M_PI / 2.0 * apf_decider_cfg_->relieve_fixed_gradient_dtheta_) {
      is_turning_ = 0;
      turning_direction_ = 0;
      // std::cout << "turning decision: stop turnning" << std::endl;
    } else {
      // std::cout << "turning decision: keep turnning" << std::endl;
    }
  }

  // std::cout << "apf_trajectory size: " << apf_trajectory_.size() <<
  // std::endl;
}

double ApfDecider::distance(PathPoint point1, Pose2D pose2) {
  double dis = std::sqrt((point1.x - pose2.x) * (point1.x - pose2.x) +
                         (point1.y - pose2.y) * (point1.y - pose2.y));
  return dis;
}

double ApfDecider::distance(PathPoint point1, PathPoint point2) {
  double dis = std::sqrt((point1.x - point2.x) * (point1.x - point2.x) +
                         (point1.y - point2.y) * (point1.y - point2.y));
  return dis;
}

double ApfDecider::distance(Pose2D pose1, Pose2D pose2) {
  double dis = std::sqrt((pose1.x - pose2.x) * (pose1.x - pose2.x) +
                         (pose1.y - pose2.y) * (pose1.y - pose2.y));
  return dis;
}

SimpleParkingLot::SimpleParkingLot(mjson::Json lot_json, double z_offset) {
  lot_json_ = lot_json;
  id_ = lot_json_["id"].int_value();
  double sum_x = 0;
  double sum_y = 0;
  double sum_z = 0;
  Point2D front(0.0, 0.0), back(0.0, 0.0);
  for (auto point : lot_json_["points"].array_value()) {
    points_.emplace_back(Point3D(point["x"].number_value(),
                                 point["y"].number_value(),
                                 point["z"].number_value()));
    corners_.emplace_back(
        Vec2d(point["x"].number_value(), point["y"].number_value()));
    sum_x += point["x"].number_value();
    sum_y += point["y"].number_value();
    sum_z += point["z"].number_value();
  }
  for (int i = 1; i < corners_.size(); i++) {
    lines_.emplace_back(LineSegment2d(corners_[i - 1], corners_[i]));
  }
  lines_.emplace_back(
      LineSegment2d(corners_[corners_.size() - 1], corners_[0]));
  center_ = Vec2d(sum_x / points_.size(), sum_y / points_.size());
  z_ = sum_z / points_.size() + z_offset;
  length_ = lines_[0].length();
  width_ = lines_[1].length();
  if (width_ <= length_) {
    is_vertical_ = true;
  } else {
    is_vertical_ = false;
  }
  front.x = (corners_[0].x() + corners_[3].x()) / 2.0;
  front.y = (corners_[0].y() + corners_[3].y()) / 2.0;
  back.x = (corners_[1].x() + corners_[2].x()) / 2.0;
  back.y = (corners_[1].y() + corners_[2].y()) / 2.0;
  theta_ = atan2(front.y - back.y, front.x - back.x);
  box_ = planning_math::Box2d(center_, theta_, length_, width_);
  average_corner_confidence_ = 1.0;
}

SimpleParkingLot::SimpleParkingLot(ParkingLotDetectionInfo info) {
  id_ = info.id;
  double sum_x = 0;
  double sum_y = 0;
  double sum_z = 0;
  double sum_confidence = 0.0;
  Point2D front(0.0, 0.0), back(0.0, 0.0);
  for (auto corner : info.corners) {
    points_.emplace_back(corner.position);
    corners_.emplace_back(Vec2d(corner.position.x, corner.position.y));
    sum_x += corner.position.x;
    sum_y += corner.position.y;
    sum_z += corner.position.z;
    sum_confidence += corner.confidence;
  }
  for (int i = 1; i < corners_.size(); i++) {
    lines_.emplace_back(LineSegment2d(corners_[i - 1], corners_[i]));
  }
  lines_.emplace_back(
      LineSegment2d(corners_[corners_.size() - 1], corners_[0]));
  center_ = Vec2d(sum_x / points_.size(), sum_y / points_.size());
  z_ = sum_z / points_.size();
  length_ = lines_[0].length();
  width_ = lines_[1].length();
  if (width_ <= length_) {
    is_vertical_ = true;
  } else {
    is_vertical_ = false;
  }
  front.x = (corners_[0].x() + corners_[3].x()) / 2.0;
  front.y = (corners_[0].y() + corners_[3].y()) / 2.0;
  back.x = (corners_[1].x() + corners_[2].x()) / 2.0;
  back.y = (corners_[1].y() + corners_[2].y()) / 2.0;
  theta_ = atan2(front.y - back.y, front.x - back.x);
  box_ = planning_math::Box2d(center_, theta_, length_, width_);
  average_corner_confidence_ = sum_confidence / info.corners.size();
  // std::cout << "parking lot " << id_ << " is vertical: " << is_vertical_
  //           << std::endl;
}

SimpleParkingLot &SimpleParkingLot::operator=(const SimpleParkingLot &copy) {
  if (this == &copy) {
    return *this;
  }
  // std::cout << "zjt test: sub4: 1 " << std::endl;

  lines_.assign(copy.lines_.begin(), copy.lines_.end());
  corners_.assign(copy.corners_.begin(), copy.corners_.end());
  points_.assign(copy.points_.begin(), copy.points_.end());
  // std::cout << "zjt test: sub4: 2 " << std::endl;
  z_ = copy.z_;
  theta_ = copy.theta_;
  lot_json_ = copy.lot_json_;
  id_ = copy.id_;
  length_ = copy.length_;
  width_ = copy.width_;
  // std::cout << "zjt test: sub4: 3 " << std::endl;
  center_ = Vec2d(copy.center_.x(), copy.center_.y());
  box_ = Box2d(center_, theta_, length_, width_);
  average_corner_confidence_ = copy.average_corner_confidence_;
  is_vertical_ = copy.is_vertical_;
  // std::cout << "zjt test: sub4: 4 " << std::endl;

  return *this;
}

SimpleParkingLot::~SimpleParkingLot() {}

/*************************** for show  ***********************/
ApfDecider::ApfDecider(std::string config_file) {
  traj_tag_ = "test";
  apf_decider_cfg_ = ApfDeciderConfig::Instance();
  (void)apf_decider_cfg_->loadFile(config_file);
  clear_info();
  clear_traj();
  clear_traj_old();

  // for test json
  // load_parking_lot_file();
  // nlohmann_demo();
  // nlohmann_demo_2();
  // mjson_demo();
  // mjson_demo_2();
}

double ApfDecider::cal_potential_field_point(double x, double y) {
  double repulsion_weight = 0;
  double rho0 = 0;
  double repulsion_2_weight = 0;
  double repulsion_2_attenuation = 0;
  double min_distance = 0;
  Vec2d point(x, y);
  double u_0 = 0;

  // wall gradient
  if (apf_decider_cfg_->use_map_potential_field_) {
    repulsion_weight = apf_decider_cfg_->wall_repulsion_weight_;
    rho0 = apf_decider_cfg_->wall_repulsion_max_distance_;
    repulsion_2_weight = apf_decider_cfg_->wall_repulsion_2_weight_;
    repulsion_2_attenuation = apf_decider_cfg_->wall_repulsion_2_attenuation_;
    for (auto wall : apf_wall_vector_) {
      min_distance = wall.DistanceTo(point);
      if (min_distance < 5 * repulsion_2_attenuation && min_distance > 0.03) {
        u_0 +=
            repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
        if (min_distance < rho0) {
          u_0 += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
                 (1 / min_distance - 1 / rho0);
        }
      } else if (min_distance < 0.03) {
        min_distance = 0.03;
        u_0 +=
            repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
        u_0 += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
               (1 / min_distance - 1 / rho0);
      }
    }
  }

  // pillar gradient
  if (apf_decider_cfg_->use_map_potential_field_) {
    repulsion_weight = apf_decider_cfg_->pillar_repulsion_weight_;
    rho0 = apf_decider_cfg_->pillar_repulsion_max_distance_;
    repulsion_2_weight = apf_decider_cfg_->pillar_repulsion_2_weight_;
    repulsion_2_attenuation = apf_decider_cfg_->pillar_repulsion_2_attenuation_;
    for (auto pillar : apf_pillar_vector_) {
      min_distance = pillar.DistanceTo(point);
      if (min_distance < 5 * repulsion_2_attenuation && min_distance > 0.03) {
        u_0 +=
            repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
        if (min_distance < rho0) {
          u_0 += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
                 (1 / min_distance - 1 / rho0);
        }
      } else if (min_distance < 0.03) {
        min_distance = 0.03;
        u_0 +=
            repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
        u_0 += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
               (1 / min_distance - 1 / rho0);
      }
    }
  }
  // parking lot gradient
  repulsion_weight = apf_decider_cfg_->parking_lot_repulsion_weight_;
  rho0 = apf_decider_cfg_->parking_lot_repulsion_max_distance_;
  repulsion_2_weight = apf_decider_cfg_->parking_lot_repulsion_2_weight_;
  repulsion_2_attenuation =
      apf_decider_cfg_->parking_lot_repulsion_2_attenuation_;
  for (auto parking_lot_line : apf_parking_lot_line_vector_) {
    min_distance = parking_lot_line.DistanceTo(point);
    if (min_distance < 5 * repulsion_2_attenuation && min_distance > 0.03) {
      u_0 += repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
      if (min_distance < rho0) {
        u_0 += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
               (1 / min_distance - 1 / rho0);
      }
    } else if (min_distance < 0.03) {
      min_distance = 0.03;
      u_0 += repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
      u_0 += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
             (1 / min_distance - 1 / rho0);
    }
  }
  // groud point potential
  if (apf_decider_cfg_->use_ground_point_potential_field_) {
    repulsion_weight = apf_decider_cfg_->ground_point_repulsion_weight_;
    rho0 = apf_decider_cfg_->ground_point_repulsion_max_distance_;
    repulsion_2_weight = apf_decider_cfg_->ground_point_repulsion_2_weight_;
    repulsion_2_attenuation =
        apf_decider_cfg_->ground_point_repulsion_2_attenuation_;
    for (auto ground_point : ground_point_vector_) {
      min_distance = ground_point.DistanceTo(point);
      if (min_distance < 5 * repulsion_2_attenuation && min_distance > 0.03) {
        u_0 +=
            repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
        if (min_distance < rho0) {
          u_0 += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
                 (1 / min_distance - 1 / rho0);
        }
      } else if (min_distance < 0.03) {
        min_distance = 0.03;
        u_0 +=
            repulsion_2_weight * exp(-min_distance / repulsion_2_attenuation);
        u_0 += 0.5 * repulsion_weight * (1 / min_distance - 1 / rho0) *
               (1 / min_distance - 1 / rho0);
      }
    }
  }
  // Point2D end_point;
  // end_point.x = apf_start_pose_.x + cos(apf_start_pose_.theta)* 40;
  // end_point.y = apf_start_pose_.y + sin(apf_start_pose_.theta)* 40;
  // u_0 += sqrt((end_point.x - x) * (end_point.x - x) + (end_point.y - y) *
  // (end_point.y - y)) * 4 - 40 * 4;

  // // road theta
  // double dx = x - apf_start_pose_.x;
  // double dy = y - apf_start_pose_.y;
  // double ds = cos(apf_start_pose_.theta) * dx + sin(apf_start_pose_.theta)
  // * dy; double dl = abs(cos(apf_start_pose_.theta + M_PI_2) * dx +
  // sin(apf_start_pose_.theta + M_PI_2) * dy); u_0 += -5 * ds - 4* dl;

  return u_0;
}

void ApfDecider::store_scenery() {

  std::string file_path =
      apf_decider_cfg_->store_path_ + to_string(store_num_) + ".json";
  // std::ofstream fout(file_path.c_str());
  std::string output_data;
  mjson::Json root_node = mjson::Json(mjson::Json::object());

  // ground point
  mjson::Json::array ground_point_node;
  for (auto point : ground_point_vector_) {
    mjson::Json point_json = mjson::Json(mjson::Json::object());
    point_json["x"] = point.x();
    point_json["y"] = point.y();
    ground_point_node.emplace_back(point_json);
  }
  root_node["ground_point"] = ground_point_node;

  // parking lot detection
  mjson::Json::array parking_lot_detection_node;
  for (auto lot : parking_lots_detection_fusion_results_) {
    mjson::Json lot_json = mjson::Json(mjson::Json::object());
    lot_json["id"] = lot.id;
    lot_json["is_good"] = lot.is_good;
    lot_json["is_empty"] = lot.is_empty;
    lot_json["is_car_in"] = lot.is_car_in;
    lot_json["is_on_map_list"] = lot.is_on_map_list;
    mjson::Json::array corners_json;
    for (auto corner : lot.corners) {
      mjson::Json corner_json = mjson::Json(mjson::Json::object());
      corner_json["is_visible"] = corner.is_visible;
      corner_json["confidence"] = corner.confidence;
      mjson::Json::array position_json;
      position_json.emplace_back(mjson::Json(corner.position.x));
      position_json.emplace_back(mjson::Json(corner.position.y));
      position_json.emplace_back(mjson::Json(corner.position.z));
      corner_json["position"] = position_json;
      corners_json.emplace_back(corner_json);
    }
    lot_json["corners"] = corners_json;
    parking_lot_detection_node.emplace_back(lot_json);
  }
  root_node["parking_lot_detection"] = parking_lot_detection_node;
  // parking lot
  mjson::Json::array parking_lot_node;
  for (auto line : apf_parking_lot_line_vector_) {
    mjson::Json line_json = mjson::Json(mjson::Json::object());
    mjson::Json::array start_json;
    start_json.emplace_back(mjson::Json(line.start().x()));
    start_json.emplace_back(mjson::Json(line.start().y()));
    line_json["start"] = start_json;
    mjson::Json::array end_json;
    end_json.emplace_back(mjson::Json(line.end().x()));
    end_json.emplace_back(mjson::Json(line.end().y()));
    line_json["end"] = end_json;
    parking_lot_node.push_back(line_json);
  }
  root_node["parking_lot"] = parking_lot_node;

  // wall
  mjson::Json::array wall_node;
  for (auto line : apf_wall_vector_) {
    mjson::Json line_json = mjson::Json(mjson::Json::object());
    mjson::Json::array start_json;
    start_json.emplace_back(mjson::Json(line.start().x()));
    start_json.emplace_back(mjson::Json(line.start().y()));
    line_json["start"] = start_json;
    mjson::Json::array end_json;
    end_json.emplace_back(mjson::Json(line.end().x()));
    end_json.emplace_back(mjson::Json(line.end().y()));
    line_json["end"] = end_json;
    wall_node.push_back(line_json);
  }
  root_node["wall"] = wall_node;

  // pillar

  mjson::Json::array pillar_node;
  for (auto line : apf_pillar_vector_) {
    mjson::Json line_json = mjson::Json(mjson::Json::object());
    mjson::Json::array start_json;
    start_json.emplace_back(mjson::Json(line.start().x()));
    start_json.emplace_back(mjson::Json(line.start().y()));
    line_json["start"] = start_json;
    mjson::Json::array end_json;
    end_json.emplace_back(mjson::Json(line.end().x()));
    end_json.emplace_back(mjson::Json(line.end().y()));
    line_json["end"] = end_json;
    pillar_node.push_back(line_json);
  }
  root_node["pillar"] = pillar_node;

  // // traj

  mjson::Json::array apf_traj_node;
  for (auto p : apf_trajectory_) {
    mjson::Json p_json = mjson::Json(mjson::Json::object());
    p_json["x"] = p.pose.x;
    p_json["y"] = p.pose.y;
    p_json["theta"] = p.pose.theta;
    p_json["gradient_x"] = p.gradient.x;
    p_json["gradient_y"] = p.gradient.y;
    apf_traj_node.push_back(p_json);
  }
  root_node["apf_traj"] = apf_traj_node;

  mjson::Json::array traj_node;
  for (auto p : traj_points_) {
    mjson::Json p_json = mjson::Json(mjson::Json::object());
    p_json["x"] = p.path_point.x;
    p_json["y"] = p.path_point.y;
    p_json["theta"] = p.path_point.theta;
    traj_node.push_back(p_json);
  }
  root_node["traj"] = traj_node;

  if (apf_decider_cfg_->store_old_traj_) {
    mjson::Json::array apf_traj_old_node;
    for (auto p : apf_trajectory_old_) {
      mjson::Json p_json = mjson::Json(mjson::Json::object());
      p_json["x"] = p.pose.x;
      p_json["y"] = p.pose.y;
      p_json["theta"] = p.pose.theta;
      p_json["gradient_x"] = p.gradient.x;
      p_json["gradient_y"] = p.gradient.y;
      apf_traj_old_node.push_back(p_json);
    }
    root_node["apf_traj_old"] = apf_traj_old_node;

    mjson::Json::array traj_old_node;
    for (auto p : traj_points_old_) {
      mjson::Json p_json = mjson::Json(mjson::Json::object());
      p_json["x"] = p.path_point.x;
      p_json["y"] = p.path_point.y;
      p_json["theta"] = p.path_point.theta;
      traj_old_node.push_back(p_json);
    }
    root_node["traj_old"] = traj_old_node;
  }

  mjson::Json info_node = mjson::Json(mjson::Json::object());

  mjson::Json ego_pose_node = mjson::Json(mjson::Json::object());
  ego_pose_node["x"] = ego_pose_.x;
  ego_pose_node["y"] = ego_pose_.y;
  ego_pose_node["theta"] = ego_pose_.theta;
  info_node["ego_pose"] = ego_pose_node;

  mjson::Json apf_start_pose_node = mjson::Json(mjson::Json::object());
  apf_start_pose_node["x"] = apf_start_pose_.x;
  apf_start_pose_node["y"] = apf_start_pose_.y;
  apf_start_pose_node["theta"] = apf_start_pose_.theta;
  info_node["apf_start_pose"] = apf_start_pose_node;

  info_node["is_turning"] = is_turning_;
  info_node["turning_direction"] = turning_direction_;
  info_node["forward_direction"] = forward_direction_;
  if (is_turning_ == 0) {
    info_node["forward_traj_status"] = forward_traj_status_;
    mjson::Json::array forward_traj_node;
    for (auto p : forward_traj_) {
      mjson::Json p_json = mjson::Json(mjson::Json::object());
      p_json["x"] = p.pose.x;
      p_json["y"] = p.pose.y;
      p_json["theta"] = p.pose.theta;
      p_json["gradient_x"] = p.gradient.x;
      p_json["gradient_y"] = p.gradient.y;
      forward_traj_node.push_back(p_json);
    }
    root_node["forward_traj"] = forward_traj_node;

    if (forward_traj_status_ == false) {
      info_node["left_traj_status"] = left_traj_status_;
      info_node["right_traj_status"] = right_traj_status_;
      mjson::Json::array left_traj_node;
      for (auto p : left_traj_) {
        mjson::Json p_json = mjson::Json(mjson::Json::object());
        p_json["x"] = p.pose.x;
        p_json["y"] = p.pose.y;
        p_json["theta"] = p.pose.theta;
        p_json["gradient_x"] = p.gradient.x;
        p_json["gradient_y"] = p.gradient.y;
        left_traj_node.push_back(p_json);
      }
      root_node["left_traj"] = left_traj_node;

      mjson::Json::array right_traj_node;
      for (auto p : right_traj_) {
        mjson::Json p_json = mjson::Json(mjson::Json::object());
        p_json["x"] = p.pose.x;
        p_json["y"] = p.pose.y;
        p_json["theta"] = p.pose.theta;
        p_json["gradient_x"] = p.gradient.x;
        p_json["gradient_y"] = p.gradient.y;
        right_traj_node.push_back(p_json);
      }
      root_node["right_traj"] = right_traj_node;
    }
  }

  info_node["store_old_traj"] = apf_decider_cfg_->store_old_traj_;
  root_node["info"] = info_node;
  root_node.dump(output_data);
  // fout << output_data;
}

void ApfDecider::restore_scenery(std::string path) {

  std::ifstream input_file(path);
  mlog::MLogDataStream tmp;
  tmp << input_file.rdbuf();
  std::string input_data = tmp.str();
  auto root_reader = mjson::Reader(input_data);

  // ground point
  ground_point_vector_.clear();
  mjson::Json::array ground_point_node = root_reader.get<mjson::Json::array>(
      "ground_point", false, mjson::Json::array());
  for (auto json : ground_point_node) {
    auto p_reader = mjson::Reader(json);
    Vec2d p(p_reader.get<double>("x", false, 0.0),
            p_reader.get<double>("y", false, 0.0));
    ground_point_vector_.push_back(p);
  }

  // parking lot detection
  parking_lots_detection_fusion_results_.clear();
  mjson::Json::array parking_lot_detection_node =
      root_reader.get<mjson::Json::array>("parking_lot_detection", false,
                                          mjson::Json::array());
  for (auto lot_json : parking_lot_detection_node) {
    auto lot_reader = mjson::Reader(lot_json);
    ParkingLotDetectionInfo lot;
    lot.id = lot_reader.get<int>("id", false, 0);
    lot.is_good = lot_reader.get<bool>("is_good", false, false);
    lot.is_empty = lot_reader.get<bool>("is_empty", false, false);
    lot.is_car_in = lot_reader.get<bool>("is_car_in", false, false);
    lot.is_on_map_list = lot_reader.get<bool>("is_on_map_list", false, false);
    lot.corners.clear();
    mjson::Json::array corner_node = lot_reader.get<mjson::Json::array>(
        "corners", false, mjson::Json::array());
    for (auto corner_json : corner_node) {
      ParkingLotDetectionInfo::CornerPoint corner;
      auto corner_reader = mjson::Reader(corner_json);
      corner.is_visible = corner_reader.get<bool>("is_visible", false, false);
      corner.confidence = corner_reader.get<double>("confidence", false, 0.0);
      mjson::Json::array position_node = corner_reader.get<mjson::Json::array>(
          "position", false, mjson::Json::array());
      corner.position.x = position_node[0].number_value();
      corner.position.y = position_node[1].number_value();
      corner.position.z = position_node[2].number_value();
      lot.corners.emplace_back(corner);
    }
    parking_lots_detection_fusion_results_.emplace_back(lot);
  }

  // parking lot
  apf_parking_lot_line_vector_.clear();
  mjson::Json::array parking_lot_node = root_reader.get<mjson::Json::array>(
      "parking_lot", false, mjson::Json::array());
  for (auto json : parking_lot_node) {
    auto reader = mjson::Reader(json);
    mjson::Json::array start_node =
        reader.get<mjson::Json::array>("start", false, mjson::Json::array());
    mjson::Json::array end_node =
        reader.get<mjson::Json::array>("end", false, mjson::Json::array());
    Vec2d start(start_node[0].number_value(), start_node[1].number_value());
    Vec2d end(end_node[0].number_value(), end_node[1].number_value());
    LineSegment2d line(start, end);
    apf_parking_lot_line_vector_.emplace_back(line);
  }

  // // wall
  apf_wall_vector_.clear();
  mjson::Json::array wall_node =
      root_reader.get<mjson::Json::array>("wall", false, mjson::Json::array());
  for (auto json : wall_node) {
    auto reader = mjson::Reader(json);
    mjson::Json::array start_node =
        reader.get<mjson::Json::array>("start", false, mjson::Json::array());
    mjson::Json::array end_node =
        reader.get<mjson::Json::array>("end", false, mjson::Json::array());
    Vec2d start(start_node[0].number_value(), start_node[1].number_value());
    Vec2d end(end_node[0].number_value(), end_node[1].number_value());
    LineSegment2d line(start, end);
    apf_wall_vector_.emplace_back(line);
  }

  // pillar
  apf_pillar_vector_.clear();
  mjson::Json::array pillar_node = root_reader.get<mjson::Json::array>(
      "pillar", false, mjson::Json::array());
  for (auto json : pillar_node) {
    auto reader = mjson::Reader(json);
    mjson::Json::array start_node =
        reader.get<mjson::Json::array>("start", false, mjson::Json::array());
    mjson::Json::array end_node =
        reader.get<mjson::Json::array>("end", false, mjson::Json::array());
    Vec2d start(start_node[0].number_value(), start_node[1].number_value());
    Vec2d end(end_node[0].number_value(), end_node[1].number_value());
    LineSegment2d line(start, end);
    apf_pillar_vector_.emplace_back(line);
  }

  // info
  mjson::Json info_node =
      root_reader.get<mjson::Json>("info", false, mjson::Json());
  auto info_reader = mjson::Reader(info_node);
  mjson::Json ego_pose_node =
      info_reader.get<mjson::Json>("ego_pose", false, mjson::Json());
  auto ego_pose_reader = mjson::Reader(ego_pose_node);
  ego_pose_.x = ego_pose_reader.get<double>("x", false, 0.0);
  ego_pose_.y = ego_pose_reader.get<double>("y", false, 0.0);
  ego_pose_.theta = ego_pose_reader.get<double>("theta", false, 0.0);

  mjson::Json apf_start_pose_node =
      info_reader.get<mjson::Json>("apf_start_pose", false, mjson::Json());
  auto apf_start_pose_reader = mjson::Reader(apf_start_pose_node);

  apf_start_pose_.x = apf_start_pose_reader.get<double>("x", false, 0.0);
  apf_start_pose_.y = apf_start_pose_reader.get<double>("y", false, 0.0);
  apf_start_pose_.theta =
      apf_start_pose_reader.get<double>("theta", false, 0.0);
  // std::cout << "ego_pose_.theta: " << ego_pose_.theta  <<std::endl;
  is_turning_ = info_reader.get<int>("is_turning", false, 0);
  turning_direction_ = info_reader.get<double>("turning_direction", false, 0.0);

  forward_direction_ = info_reader.get<double>("forward_direction", false, 0.0);

  // traj
  apf_trajectory_.clear();
  mjson::Json::array apf_traj_node = root_reader.get<mjson::Json::array>(
      "apf_traj", false, mjson::Json::array());
  for (auto json : apf_traj_node) {
    auto reader = mjson::Reader(json);
    Pose2D pose(reader.get<double>("x", false, 0.0),
                reader.get<double>("y", false, 0.0),
                reader.get<double>("theta", false, 0.0));
    Point2D grad(reader.get<double>("gradient_x", false, 0.0),
                 reader.get<double>("gradient_y", false, 0.0));
    ApfPoint point(pose, grad);
    apf_trajectory_.emplace_back(point);
  }

  traj_points_.clear();
  mjson::Json::array traj_node =
      root_reader.get<mjson::Json::array>("traj", false, mjson::Json::array());
  for (auto json : traj_node) {
    auto reader = mjson::Reader(json);
    TrajectoryPoint point(reader.get<double>("x", false, 0.0),
                          reader.get<double>("y", false, 0.0), 0,
                          reader.get<double>("theta", false, 0.0), 0, 1, 0.5, 0,
                          0);
    traj_points_.emplace_back(point);
  }

  // old traj
  if (info_reader.get<bool>("store_old_traj", false, false)) {
    apf_trajectory_old_.clear();
    mjson::Json::array apf_traj_old_node = root_reader.get<mjson::Json::array>(
        "apf_traj_old", false, mjson::Json::array());
    for (auto json : apf_traj_old_node) {
      auto reader = mjson::Reader(json);
      Pose2D pose(reader.get<double>("x", false, 0.0),
                  reader.get<double>("y", false, 0.0),
                  reader.get<double>("theta", false, 0.0));
      Point2D grad(reader.get<double>("gradient_x", false, 0.0),
                   reader.get<double>("gradient_y", false, 0.0));
      ApfPoint point(pose, grad);
      apf_trajectory_old_.emplace_back(point);
    }

    traj_points_old_.clear();
    mjson::Json::array traj_old_node = root_reader.get<mjson::Json::array>(
        "traj_old", false, mjson::Json::array());

    for (auto json : traj_old_node) {
      auto reader = mjson::Reader(json);
      TrajectoryPoint point(reader.get<double>("x", false, 0.0),
                            reader.get<double>("y", false, 0.0), 0,
                            reader.get<double>("theta", false, 0.0), 0, 1, 0.5,
                            0, 0);
      traj_points_old_.emplace_back(point);
    }
  }

  if (is_turning_ == 0) {
    forward_traj_status_ =
        info_reader.get<bool>("forward_traj_status", false, false);
    forward_traj_.clear();
    mjson::Json::array forward_traj_node = root_reader.get<mjson::Json::array>(
        "forward_traj", false, mjson::Json::array());
    for (auto json : forward_traj_node) {
      auto reader = mjson::Reader(json);
      Pose2D pose(reader.get<double>("x", false, 0.0),
                  reader.get<double>("y", false, 0.0),
                  reader.get<double>("theta", false, 0.0));
      Point2D grad(reader.get<double>("gradient_x", false, 0.0),
                   reader.get<double>("gradient_y", false, 0.0));
      ApfPoint point(pose, grad);
      forward_traj_.emplace_back(point);
    }

    if (forward_traj_status_ == false) {
      left_traj_status_ =
          info_reader.get<bool>("left_traj_status", false, false);
      right_traj_status_ =
          info_reader.get<bool>("right_traj_status", false, false);

      left_traj_.clear();
      mjson::Json::array left_traj_node = root_reader.get<mjson::Json::array>(
          "left_traj", false, mjson::Json::array());
      for (auto json : left_traj_node) {
        auto reader = mjson::Reader(json);
        Pose2D pose(reader.get<double>("x", false, 0.0),
                    reader.get<double>("y", false, 0.0),
                    reader.get<double>("theta", false, 0.0));
        Point2D grad(reader.get<double>("gradient_x", false, 0.0),
                     reader.get<double>("gradient_y", false, 0.0));
        ApfPoint point(pose, grad);
        left_traj_.emplace_back(point);
      }

      right_traj_.clear();
      mjson::Json::array right_traj_node = root_reader.get<mjson::Json::array>(
          "right_traj", false, mjson::Json::array());
      for (auto json : right_traj_node) {
        auto reader = mjson::Reader(json);
        Pose2D pose(reader.get<double>("x", false, 0.0),
                    reader.get<double>("y", false, 0.0),
                    reader.get<double>("theta", false, 0.0));
        Point2D grad(reader.get<double>("gradient_x", false, 0.0),
                     reader.get<double>("gradient_y", false, 0.0));
        ApfPoint point(pose, grad);
        right_traj_.emplace_back(point);
      }
    }
  }
}

/*************************** for park out pose  ***********************/
void ApfDecider::add_historical_traj_point() {
  TrajectoryPoint point(apf_start_pose_.x, apf_start_pose_.y, 0,
                        apf_start_pose_.theta, 0, 1, 0.5, 0, 0);
  if (historical_traj_points_.empty()) {
    historical_traj_points_.emplace_back(point);
  } else if (distance(historical_traj_points_.back().path_point,
                      apf_start_pose_) > 0.01) {
    historical_traj_points_.emplace_back(point);
  }
}

void ApfDecider::store_historical_traj_points() {
  std::string file_path =
      apf_decider_cfg_->store_path_ + "historical_traj_points.json";
  // std::ofstream fout(file_path.c_str());
  mjson::Json root_node = mjson::Json(mjson::Json::object());
  mjson::Json::array traj_node;
  for (auto point : historical_traj_points_) {
    mjson::Json node = mjson::Json(mjson::Json::object());
    node["x"] = point.path_point.x;
    node["y"] = point.path_point.y;
    node["theta"] = point.path_point.theta;
    traj_node.emplace_back(node);
  }
  root_node["traj"] = traj_node;
  std::string data;
  root_node.dump(data);
  // fout << data;
}

void ApfDecider::restore_historical_traj_points(std::string file_path) {
  historical_traj_points_.clear();

  mjson::Json root_node = mjson::Json(mjson::Json::object());
  std::fstream input_file(file_path);
  mlog::MLogDataStream tmp;
  tmp << input_file.rdbuf();
  std::string input_data = tmp.str();
  std::string err_info;
  root_node.parse(input_data, err_info);
  mjson::Json::array traj_node;
  traj_node = root_node["traj"].array_value();
  for (auto p : traj_node) {
    TrajectoryPoint point(p["x"].number_value(), p["y"].number_value(), 0,
                          p["theta"].number_value(), 0, 1, 0.5, 0, 0);
    historical_traj_points_.emplace_back(point);
  }
}

bool ApfDecider::park_out_pose_from_traj(
    std::vector<TrajectoryPoint> traj_candidate,
    std::vector<TrajectoryPoint> &traj_output, Pose2D &pose_output) {
  double cumulative_s = 0.0;
  if (traj_candidate.empty()) {
    return false;
  } else if (traj_candidate.size() < 2) {
    return false;
  } else {
    for (int index = 1; index < traj_candidate.size(); index++) {
      double s = distance(traj_candidate[index].path_point,
                          traj_candidate[index - 1].path_point);
      if (s < max(0.5, 2 * apf_decider_cfg_->gradient_descent_distance_)) {
        cumulative_s += s;
      } else {
        return false;
      }
      if (cumulative_s > 5.0) {
        traj_output.assign(traj_candidate.begin(),
                           traj_candidate.begin() + index);
        if (check_traj_far_from_obstacle(traj_output)) {
          pose_output.x = traj_candidate[index].path_point.x;
          pose_output.y = traj_candidate[index].path_point.y;
          pose_output.theta = traj_candidate[index].path_point.theta;
          return true;
        } else {
          traj_output.clear();
          return false;
        }
      }
    }
  }

  return false;
}

// bool ApfDecider::calculate_park_out(std::string dir, double offset) {

//   // traj tag
//   traj_tag_ = "park_out";
//   // clear info, status, traj
//   reset_all();

//   // gather info
//   gather_ego_state();
//   gather_map_info();
//   // calc park out state
//   calc_park_out_state(dir, offset);

//   return plan_park_out_from_current(apf_start_pose_);
//   // for find historical
//   // if (plan_park_out_from_historical()) {
//   //   return true;
//   // } else {
//     // if (plan_park_out_from_current()) {
//     //   return true;
//     // }
//   // }
// }

bool ApfDecider::calculate_park_out(Pose2D start_pose) {

  // traj tag
  traj_tag_ = "park_out";
  // clear info, status, traj
  reset_all();

  // gather info
  gather_ego_state();
  gather_map_info();

  return plan_park_out_from_current(start_pose);
  // for find historical
  // if (plan_park_out_from_historical()) {
  //   return true;
  // } else {
  // if (plan_park_out_from_current()) {
  //   return true;
  // }
  // }
}

bool ApfDecider::check_traj_far_from_obstacle(Vec2d start, Vec2d end) {
  LineSegment2d traj(start, end);
  for (auto ground_point : ground_point_vector_) {
    if (traj.DistanceSquareTo(ground_point) < 1.0) {
      return false;
    }
  }
  return true;
}

bool ApfDecider::check_traj_far_from_obstacle(
    std::vector<TrajectoryPoint> traj) {
  bool safety = true;
  if (traj.empty()) {
    // std::cout << "zjt debug: traj checked is empty" << std::endl;
    return false;
  } else {
    for (auto traj_point : traj) {
      Vec2d point(
          traj_point.path_point.x + apf_decider_cfg_->vehicle_length_added_ *
                                        std::cos(traj_point.path_point.theta),
          traj_point.path_point.y + apf_decider_cfg_->vehicle_length_added_ *
                                        std::sin(traj_point.path_point.theta));
      Box2d bbox(point, traj_point.path_point.theta,
                 apf_decider_cfg_->vehicle_length_,
                 apf_decider_cfg_->vehicle_width_);
      for (auto ground_point : ground_point_vector_) {
        if (bbox.DistanceTo(ground_point) < 0.2) {
          safety = false;
        }
      }
    }
  }
  return safety;
}

// bool ApfDecider::plan_park_out_from_historical() {
//   Pose2D *mutable_park_out_target_pose =
//       PlanningContext::Instance()->mutable_park_out_target_pose();
//   Pose2D pose;
//   std::vector<TrajectoryPoint> traj;
//   std::string historical_file_path =
//       apf_decider_cfg_->store_path_ + "historical_traj_points.json";
//   if (boost::filesystem::exists(historical_file_path)) {
//     restore_historical_traj_points(historical_file_path);
//     if (park_out_pose_from_traj(historical_traj_points_, traj, pose)) {
//       *mutable_park_out_target_pose = pose;
//       traj_points_.assign(traj.begin(), traj.end());
//       traj_target_pose_ = pose;
//       check_traj_status();
//       set_status_and_info(true);
//       return true;
//     }
//   }
//   return false;
// }

// void ApfDecider::calc_park_out_state(std::string dir, double offset) {
//   double min_dis = 10000.0;
//   double dir_theta = 0;
//   SimpleParkingLot parking_lot_matched;
//   if (dir == "left"){
//     dir_theta = 0;
//   }
//   else if(dir == "right"){
//     dir_theta = M_PI;
//   }
//   else {
//     dir_theta = 0;
//   }

//   for (auto element : parking_lot_map_) {
//     if (element.second.center().DistanceTo(ego_center_) < min_dis) {
//       parking_lot_matched = element.second;
//       min_dis = element.second.center().DistanceTo(ego_center_);
//     }
//   }
//   if (min_dis < 2.0) {
//     in_parking_lot_ = true;
//   } else {
//     in_parking_lot_ = false;
//   }
//   if (in_parking_lot_) {
//     apf_start_pose_.x =
//         parking_lot_matched.center().x() +
//         (parking_lot_matched.length() / 2 + offset) *
//         std::cos(parking_lot_matched.theta());
//     apf_start_pose_.y =
//         parking_lot_matched.center().y() +
//         (parking_lot_matched.length() / 2 + offset) *
//         std::sin(parking_lot_matched.theta());
//     apf_start_pose_.theta =
//         NormalizeAngle(parking_lot_matched.theta() + M_PI_2 + dir_theta);
//   } else {
//     if (min_dis < 8.0) {
//       if (check_traj_far_from_obstacle(parking_lot_matched.center(),
//       ego_center_))
//       {
//         apf_start_pose_.x = parking_lot_matched.center().x() +
//                             (parking_lot_matched.length() / 2 + offset) *
//                                 std::cos(parking_lot_matched.theta());
//         apf_start_pose_.y = parking_lot_matched.center().y() +
//                             (parking_lot_matched.length() / 2 + offset) *
//                                 std::sin(parking_lot_matched.theta());
//         if (abs(NormalizeAngle(parking_lot_matched.theta() + M_PI_2 -
//                                ego_pose_.theta)) < M_PI_2) {
//           apf_start_pose_.theta =
//               NormalizeAngle(parking_lot_matched.theta() + M_PI_2);

//         } else {
//           apf_start_pose_.theta =
//               NormalizeAngle(parking_lot_matched.theta() - M_PI_2);
//         }
//       } else {
//         apf_start_pose_.x = ego_center_.x();
//         apf_start_pose_.y = ego_center_.y();
//         apf_start_pose_.theta = NormalizeAngle(ego_pose_.theta);
//       }

//     } else {
//       apf_start_pose_.x = ego_center_.x();
//       apf_start_pose_.y = ego_center_.y();
//       apf_start_pose_.theta = NormalizeAngle(ego_pose_.theta);
//     }
//   }
//   // apf_start_pose_.x =
//   //     ego_center_.x() +
//   //     apf_decider_cfg_->vehicle_length_ * 1.2 * std::cos(ego_pose_.theta);
//   // apf_start_pose_.y =
//   //     ego_center_.y() +
//   //     apf_decider_cfg_->vehicle_length_ * 1.2 * std::sin(ego_pose_.theta);
//   // apf_start_pose_.theta = NormalizeAngle(ego_pose_.theta + M_PI_2);
// }

bool ApfDecider::plan_park_out_from_current(Pose2D start_pose) {

  ApfTrajectory traj;
  bool is_traj_found = false;

  if (create_apf_trajectory(start_pose, start_pose.theta, 100, 30, traj)) {
    is_traj_found = true;
  }
  // if (in_parking_lot_) {
  //   if (create_apf_trajectory(start_pose, start_pose.theta, 100, 30, traj)) {
  //     is_traj_found = true;
  //   } else if (create_apf_trajectory(start_pose,
  //                                    NormalizeAngle(start_pose.theta + M_PI),
  //                                    100, 30, traj)) {
  //     is_traj_found = true;
  //   }
  // } else {
  //   if (create_apf_trajectory(start_pose, start_pose.theta, 100, 30, traj)) {
  //     is_traj_found = true;
  //   }
  // }
  if (is_traj_found) {
    apf_trajectory_.assign(traj.begin(), traj.end());
    traj_target_pose_ = traj.back().pose;
  } else {
    traj_target_pose_ = ego_pose_;
    apf_trajectory_.clear();
  }
  set_traj_points_from_apf();
  is_traj_found = is_traj_found && check_traj_status();
  set_status_and_info(false);
  if (apf_decider_cfg_->is_store_) {
    store_count_++;
    if (store_count_ >= apf_decider_cfg_->store_freq_div_) {
      double store_start = MTIME()->timestamp().sec();
      store_num_++;
      store_scenery();
      double store_end = MTIME()->timestamp().sec();
      auto store_time = store_end - store_start;
      // cout << "time cost: store_time " << store_time << std::endl;
      store_count_ = 0;
    }
  }
  return is_traj_found;
}

/*********************** json demo ********************************/
void mjson_demo() {
  std::string file_path = "/home/ros/Downloads/test_mjson.json";
  mjson::Json root_node = mjson::Json(mjson::Json::object());
  mjson::Json::array traj_node;
  for (int i = 0; i < 10; i++) {
    mjson::Json node = mjson::Json(mjson::Json::object());
    node["x"] = i;
    node["y"] = -i;
    traj_node.emplace_back(node);
  }
  root_node["traj"] = traj_node;
  std::string data_from_file;
  root_node.dump(data_from_file);
  // std::ofstream fout(file_path.c_str());
  // fout << data_from_file;
}
void mjson_demo_2() {
  std::string file_path = "/home/ros/Downloads/test_mjson.json";
  std::ifstream input_file(file_path);
  mlog::MLogDataStream tmp;
  tmp << input_file.rdbuf();
  std::string str1 = tmp.str();
  // std::string str1(std::istreambuf_iterator<char>(input_file_),
  // std::istreambuf_iterator<char>());
  // std::cout << str1 << std::endl;
  auto reader = mjson::Reader(str1);
  // std::cout << "1111" << std::endl;
  mjson::Json::array traj_node =
      reader.get<mjson::Json::array>("traj", false, mjson::Json::array());
  std::vector<double> xs, ys;
  // std::cout << "2222" << std::endl;
  for (auto traj_point : traj_node) {
    auto traj_point_reader = mjson::Reader(traj_point);
    xs.emplace_back(traj_point_reader.get<double>("x", false, 0.0));
    ys.emplace_back(traj_point_reader.get<double>("y", false, 0.0));
  }
  // std::cout << "mjson_demo_2: xs_size: " << xs.size() << " x0: " << xs[0]
  //           << std::endl;
}

// void nlohmann_demo() {
//   std::string file_path = "/home/ros/Downloads/test_nlohmann.json";
//   nlohmann::json root_node;
//   nlohmann::json point_node;
//   for (int i = 0; i < 10; i++) {
//     nlohmann::json point_json;
//     point_json["x"] = i;
//     point_json["y"] = i;
//     point_node.push_back(point_json);
//   }
//   root_node["point"] = point_node;
//   std::ofstream fout(file_path.c_str());
//   fout << root_node;
// }
// void nlohmann_demo_2() {
//   std::string file_path = "/home/ros/Downloads/test.json";
//   nlohmann::json root_node;
//   std::fstream input_file(file_path);
//   input_file >> root_node;
//   nlohmann::json point_node = root_node["point"];
//   std::vector<Vec2d> points;
//   for (auto json : point_node) {
//     Vec2d p(json["x"].get<double>(), json["y"].get<double>());
//     points.push_back(p);
//   }
// }

} // namespace parking

} // namespace msquare