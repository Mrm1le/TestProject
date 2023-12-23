#include "planner/behavior_planner/deciders/freespace_decider.h"
#include "common/config/vehicle_param.h"
#include "planning/common/common.h"
#include <chrono>

namespace msquare {
namespace parking {
FreespaceDecider::FreespaceDecider(
    const std::shared_ptr<WorldModel> &world_model) {
  world_model_ = world_model;
  plan_start = MTIME()->timestamp().sec();
}

bool FreespaceDecider::execute() {

  if (world_model_ == nullptr) {
    MSD_LOG(INFO, "world model is none!");
    return false;
  }

  double process_end = MTIME()->timestamp().sec();
  double process_span = process_end - plan_start;

  // if(process_span > SEND_INTERVAL) {
  //   set_map_boundary();
  //   PlanningContext::Instance()->mutable_freespace_decider_output()->is_send_map_square
  //   = true; plan_start = process_end;
  // } else {
  //   PlanningContext::Instance()->mutable_freespace_decider_output()->is_send_map_square
  //   = false;
  // }

  get_map_boundary();

  world_model_->update_and_filter_freespace(map_boundarys_);

  return true;
}

void FreespaceDecider::set_map_boundary() {
  auto output = PlanningContext::Instance()->mutable_square_map2();
  auto pose = world_model_->get_ego_state().ego_pose;
  planning_math::Box2d boundary_box(planning_math::Vec2d(pose.x, pose.y),
                                    pose.theta, MAP_BOX_LENGTH, MAP_BOX_WIDTH);
  // auto corners = boundary_box.GetAllCorners();
  for (size_t i = 0; i < boundary_box.GetAllCorners().size(); i++) {
    output->box_corners[i].x = boundary_box.GetAllCorners()[i].x();
    output->box_corners[i].y = boundary_box.GetAllCorners()[i].y();
  }
}

// bool FreespaceDecider::get_freespace_vector(){
//   fs_pt_.clear();
//   for(auto &obs : world_model_->obstacle_manager().get_points().Items()){
//     if(obs->Type() == ObjectType::NOT_KNOW){
//       std::pair<int, planning_math::Vec2d> fs_pair(obs->Id(),
//       obs->PerceptionBoundingBox().center()); fs_pt_.push_back(fs_pair);
//       // std::cout << "fs_fly: " << fs_pair.first << " " <<
//       fs_pair.second.x() << " " << fs_pair.second.y() << std::endl;
//     }
//   }

//   std::sort(fs_pt_.begin(), fs_pt_.end(), [](std::pair<int,
//   planning_math::Vec2d> a, std::pair<int, planning_math::Vec2d> b){
//     return a.first < b.first;
//   });
//   return true;
// }

bool FreespaceDecider::get_map_boundary() {
  map_boundarys_.clear();
  // TODO:: add message receiver check
  // std::cout << "square_map_fs: " <<
  // world_model_->get_square_map_response2().road_borders.size() << "  " <<
  // world_model_->get_square_map_response2().obstacles.pillar.size() <<
  // std::endl;
  if (world_model_->get_square_map_response2().road_borders.size() != 0) {
    for (size_t i = 0;
         i < world_model_->get_square_map_response2().road_borders.size();
         i++) {
      for (size_t j = 1;
           j <
           world_model_->get_square_map_response2().road_borders[i].pts.size();
           j++) {
        planning_math::Vec2d pt1(world_model_->get_square_map_response2()
                                     .road_borders[i]
                                     .pts[j - 1]
                                     .x,
                                 world_model_->get_square_map_response2()
                                     .road_borders[i]
                                     .pts[j - 1]
                                     .y);
        planning_math::Vec2d pt2(
            world_model_->get_square_map_response2().road_borders[i].pts[j].x,
            world_model_->get_square_map_response2().road_borders[i].pts[j].y);
        map_boundarys_.push_back(planning_math::LineSegment2d(pt1, pt2));
        // std::cout << "freespace boundary: " << pt1.x() << " " << pt1.y() << "
        // " << pt2.x() << " " << pt2.y() << std::endl;
      }
    }
  }
  return true;
}

// void FreespaceDecider::feed_planning_context(){
//   FreespaceDeciderOutput *planning_status =
//     PlanningContext::Instance()->mutable_freespace_decider_output();
//   planning_status->fs_pts.clear();
//   for(int i = 0; i < fs_pt_array_.size(); i++){
//     std::vector<std::pair<int, Point3D>> pts_tmp;
//     for(int j = 0; j < fs_pt_array_[i].size(); j++){
//       pts_tmp.push_back(std::pair<int, Point3D>(fs_pt_array_[i][j].first,
//         Point3D(fs_pt_array_[i][j].second.x(), fs_pt_array_[i][j].second.y(),
//         0.0)));
//         // std::cout << "pts_fly: " << fs_pt_array_[i][j].second.x() << "  "
//         << fs_pt_array_[i][j].second.y() << std::endl;
//     }
//     planning_status->fs_pts.push_back(pts_tmp);
//   }
// }

// std::vector<std::vector<std::pair<int, planning_math::Vec2d>>>
// FreespaceDecider::freespace_filter(
//     const std::vector<std::pair<int, planning_math::Vec2d>> &fs_pt_,
//     const std::vector<planning_math::LineSegment2d> &boundarys,
//     const Pose2D &ego_pose){
//   std::vector<std::vector<std::pair<int, planning_math::Vec2d>>>
//   remaining_fs_pt; std::vector<std::pair<int, planning_math::Vec2d>> fs_pts;

//   for(int i = 0; i < fs_pt_.size(); i++){
//     double dist = sqrt(pow(fs_pt_[i].second.x() - ego_pose.x,2) +
//     pow(fs_pt_[i].second.y() - ego_pose.y,2));
//     // std::cout << "fs_dist: " << dist << std::endl;
//     if(dist < FREESPACE_FILTER_DIST){
//       // linesegment2d:
//       planning_math::LineSegment2d temp_line(planning_math::Vec2d(ego_pose.x,
//       ego_pose.y), fs_pt_[i].second); bool blind = false; for(int j = 0; j <
//       boundarys.size(); j++){
//         if(temp_line.HasIntersect(boundarys[j])){
//           // std::cout << i << "  " << j << std::endl;
//           blind = true;
//           break;
//         }
//       }
//       if(!blind){
//         if(fs_pts.size() == 0){
//           fs_pts.push_back(fs_pt_[i]);
//         }
//         else if((fs_pts.back().first + 1) == fs_pt_[i].first){
//           fs_pts.push_back(fs_pt_[i]);
//         }
//         else{
//           remaining_fs_pt.push_back(fs_pts);
//           fs_pts.clear();
//           fs_pts.push_back(fs_pt_[i]);
//           // std::cout << "fs_pts_: " << fs_pt_[i].first << "  " <<
//           fs_pt_[i].second.x() << "  " << fs_pt_[i].second.y() << std::endl;
//         }
//       }
//     }
//   }
//   if (fs_pts.size() != 0){
//     remaining_fs_pt.push_back(fs_pts);
//   }
//   // std::cout << "remaining_fs_pt: " << remaining_fs_pt.size() << std::endl;
//   return remaining_fs_pt;
// }
} // namespace parking
} // namespace msquare
