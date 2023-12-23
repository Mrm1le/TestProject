#include "data_driven_planner/common/model_result_manager.h"
#include "data_driven_planner/common/ddp_context.h"
#include "data_driven_planner/models/fusion_object_manager.h"

#include "common/planning_fail_tracer.h"
#include "data_driven_planner/common/ddp_debug_logger.h"
#include "data_driven_planner/common/ddp_utils.h"
#include "data_driven_planner/models/ego_pose_manager.h"
#include "data_driven_planner/models/fusion_object_manager.h"

namespace msquare {
namespace ddp {
ModelResultManager::ModelResultManager(
    const std::shared_ptr<WorldModel> &world_model,
    const std::shared_ptr<DdpModel> &ddp_model,
    bool use_ddp_model_in_planning) {
  world_model_ = world_model;
  ddp_model_ = ddp_model;
  use_ddp_model_in_planning_ = use_ddp_model_in_planning;
}

void ModelResultManager::update() {
  MDEBUG_JSON_BEGIN_DICT(ModelResultManager)
  MDEBUG_JSON_ADD_ITEM(use_ddp_model_in_planning, use_ddp_model_in_planning_,
                       ModelResultManager)
  if (use_ddp_model_in_planning_) {
    update_with_ddp_model_in_planning();
  }
  // } else {
  //   update_with_ddp_model_in_prediction();
  // }
  MDEBUG_JSON_END_DICT(ModelResultManager)
}

void ModelResultManager::update_with_ddp_model_in_planning() {
  ddp_valid_ = false;
  ddp_trajectory_info_.clear();

  auto fusion_timestamp = DdpContext::Instance()
                              ->fusion_object_manager()
                              ->get_last_fusion_timestamp();

  EgoPose ego_pose;
  double z = DdpContext::Instance()->ego_pose_manager()->get_ego_pose(
                 fusion_timestamp, ego_pose)
                 ? ego_pose.position.z
                 : 0.0;

  bool is_replan = world_model_->is_replan();
  DdpTrajectorys ddp_trajectorys;
  std::string ddp_model_input;
  auto ok = ddp_model_->run(fusion_timestamp, ddp_trajectorys, ddp_model_input,
                            is_replan);

  *PlanningContext::Instance()->mutable_ddp_model_input() = ddp_model_input;
  auto planner_debug = PlanningContext::Instance()->mutable_planner_debug();
  planner_debug->ddp_debug_info.ddp_trajectory_point_num.clear();
  for (size_t i = 0; i < ddp_trajectorys.size(); ++i) {
    planner_debug->ddp_debug_info.ddp_trajectory_point_num.emplace_back(
        ddp_trajectorys[i].trajectory.size());
  }

  MDEBUG_JSON_ADD_ITEM(ddp_valid, ok, ModelResultManager)
  MDEBUG_JSON_ADD_ITEM(ddp_timestamp, fusion_timestamp, ModelResultManager)

  if (ok) {
    // parse ddp_trajectorys
    ddp_valid_ = true;
    timestamp_ = fusion_timestamp;

    MDEBUG_JSON_BEGIN_ARRAY(trajectory_array)
    for (size_t i = 0; i < ddp_trajectorys.size(); ++i) {
      auto &traj = ddp_trajectorys[i];
      MSD_LOG(
          INFO,
          "[DDP-ModelResultManager] ddp_object_traj: %d, type: %d, size: %d", i,
          traj.type, traj.trajectory.size());
      if (traj.trajectory.empty() == false) {
        MSD_LOG(INFO,
                "[DDP-ModelResultManager] ddp_object_traj start_x: %f, "
                "start_y: %f, end_x: %f, end_y: %f",
                traj.trajectory.front().x, traj.trajectory.front().y,
                traj.trajectory.back().x, traj.trajectory.back().y);
      }
      assert(traj.type == DDP_LANE_CHANGE);
      if (traj.type == DDP_LANE_CHANGE) {
        ddp_trajectory_info_.push_back(traj);
      }

      std::vector<std::string> tags{};
      tags.push_back("logit: " + std::to_string(traj.logit));
      MDEBUG_JSON_BEGIN_OBJECT(object1)
      MDEBUG_JSON_ADD_ITEM(logit, traj.logit, object1)
      MDEBUG_JSON_BEGIN_ARRAY(trajectory)
      for (size_t j = 0; j < traj.trajectory.size(); ++j) {
        MDEBUG_JSON_BEGIN_OBJECT(object2)
        const auto point = traj.trajectory[j];
        MDEBUG_JSON_ADD_ITEM(x, point.x, object2)
        MDEBUG_JSON_ADD_ITEM(y, point.y, object2)
        MDEBUG_JSON_ADD_ITEM(heading_angle, point.heading_angle, object2)
        MDEBUG_JSON_ADD_ITEM(curvature, point.curvature, object2)
        MDEBUG_JSON_ADD_ITEM(t, point.t, object2)
        MDEBUG_JSON_ADD_ITEM(v, point.v, object2)
        MDEBUG_JSON_ADD_ITEM(a, point.a, object2)
        MDEBUG_JSON_END_OBJECT(object2)
      }
      MDEBUG_JSON_END_ARRAY(trajectory)
      MDEBUG_JSON_END_OBJECT(object1)
      if (!npp::common::running_on_cpu_limited_system()) {
        dump_mdebug_polyline(traj.trajectory, z + 0.01, 228, 238, 236, 0.08,
                             "raw_model_traj", tags);
      }
    }
    MDEBUG_JSON_END_ARRAY(trajectory_array)

    // update current lane ddp trajectory
    auto current_frenet_coord =
        world_model_->get_baseline_info(0)->get_frenet_coord();
    current_lane_ddp_trajectory_.trajectory.clear();
    int current_lane_traj_index = -1;
    double max_total_l = DBL_MAX;
    bool current_lane_id_machted = false;
    for (int i = 0; i < ddp_trajectory_info_.size(); i++) {
      if (ddp_trajectory_info_[i].trajectory.size() < 3) {
        MSD_LOG(INFO,
                "[DDP-ModelResultManager] ddp_object_traj traj empy: index: %d",
                i);
        continue;
      }
      double total_l = 0.0;
      for (const auto &point : ddp_trajectory_info_[i].trajectory) {
        Point2D frenet_point, carte_point;
        carte_point.x = point.x;
        carte_point.y = point.y;
        if (current_frenet_coord->CartCoord2FrenetCoord(
                carte_point, frenet_point) == TRANSFORM_FAILED) {
          MSD_LOG(INFO,
                  "[DDP-ModelResultManager] ddp_object_traj frenet transform "
                  "failed: index: %d x: %f, y: %f",
                  i, point.x, point.y);
          continue;
        }
        total_l += std::fabs(frenet_point.y);
      }
      if (ddp_trajectory_info_[i].track_ids.size() > 0) {
        MSD_LOG(INFO,
                "[DDP-ModelResultManager] ddp_object_traj: index: %d total_l: "
                "%f id: %d",
                i, total_l, ddp_trajectory_info_[i].track_ids[0]);
      }
      if (total_l < max_total_l) {
        current_lane_traj_index = i;
        max_total_l = total_l;
      }

      if (ddp_trajectory_info_[i].track_ids.size() > 0 &&
          ddp_trajectory_info_[i].track_ids[0] == 0) {
        current_lane_ddp_trajectory_ = ddp_trajectory_info_[i];
        current_lane_id_machted = true;
        MSD_LOG(
            INFO,
            "[DDP-ModelResultManager] ddp_object_traj id matched: index: %d",
            i);

        // calc current lane traj's sl
        auto current_baseline_info = world_model_->get_baseline_info(0);
        if (current_baseline_info && current_baseline_info->is_valid() &&
            current_baseline_info->get_frenet_coord()) {
          auto frenet_coord = current_baseline_info->get_frenet_coord();
          for (auto &pt : ddp_trajectory_info_[i].trajectory) {
            Point2D pt_frenet{0.0, 0.0};
            Point2D pt_cart{pt.x, pt.y};
            if (frenet_coord->CartCoord2FrenetCoord(pt_cart, pt_frenet) ==
                TRANSFORM_SUCCESS) {
              pt.s = pt_frenet.x;
              pt.l = pt_frenet.y;
              pt.frenet_valid = true;
            }
          }
        }
      }
    }
    if (!current_lane_id_machted && current_lane_traj_index != -1) {
      current_lane_ddp_trajectory_ =
          ddp_trajectory_info_[current_lane_traj_index];
      MSD_LOG(
          INFO,
          "[DDP-ModelResultManager] ddp_object_traj index matched: index: %d",
          current_lane_traj_index);
    }
    if (current_lane_id_machted && current_lane_traj_index != -1 &&
        ddp_trajectory_info_[current_lane_traj_index].track_ids[0] != 0) {
      MSD_LOG(
          INFO,
          "[DDP-ModelResultManager] ddp_object_traj mismatch: index: %d id: %d",
          current_lane_traj_index,
          ddp_trajectory_info_[current_lane_traj_index].track_ids[0]);
    }
  } else {
    PLANNING_FAIL_TRACE("failed to run ddp_model");
    MSD_LOG(ERROR, "[DDP-ModelResultManager] failed to run ddp_model!");
  }

  if (current_lane_ddp_trajectory_.trajectory.size() > 0) {
    const double delta_time = 0.2;
    const int num_point = 21;
    auto planning_status =
        PlanningContext::Instance()->mutable_planning_status();
    // interpolate by plan start timestamp, start time limit to 1.0s
    double start_time =
        clip(planning_status->planning_result.next_timestamp_sec - timestamp_,
             1.0, 0.0);
    MSD_LOG(ERROR, "[DDP-ModelResultManager] ddp_object_traj start time: %f",
            start_time);
    // clear traj when delay too large
    if (start_time > 0.99) {
      current_lane_ddp_trajectory_.trajectory.clear();
      MSD_LOG(ERROR, "[DDP-ModelResultManager] ddp_object_traj start time "
                     "delay too large!");
      return;
    }
    if (!ModelResultManager::interpolate(
            current_lane_ddp_trajectory_.trajectory, start_time, delta_time,
            num_point, current_lane_ddp_trajectory_.trajectory)) {
      MSD_LOG(ERROR, "[DDP-ModelResultManager] ddp_object_traj current lane "
                     "interpolate failed!");
    }
  } else {
    MSD_LOG(ERROR,
            "[DDP-ModelResultManager] ddp_object_traj current lane empty!");
  }
}

// void ModelResultManager::update_with_ddp_model_in_prediction() {
//   DdpObject ddp_object;
//   world_model_->get_data_driven_planning_object(ddp_object);
//   MSD_LOG(INFO, "[DDP-ModelResultManager] ddp_object: %d, timestamp: %f",
//           ddp_object.ddp_valid, ddp_object.timestamp);
//   MDEBUG_JSON_ADD_ITEM(ddp_object_valid, ddp_object.ddp_valid,
//                        ModelResultManager)
//   MDEBUG_JSON_ADD_ITEM(ddp_object_timestamp, ddp_object.timestamp,
//                        ModelResultManager)

//   ddp_trajectory_info_.clear();
//   if (ddp_object.ddp_valid) {
//     ddp_valid_ = ddp_object.ddp_valid;
//     timestamp_ = ddp_object.timestamp;
//     MDEBUG_JSON_BEGIN_ARRAY(trajectory_array)
//     for (size_t i = 0; i < ddp_object.trajectory_array.size(); ++i) {
//       const auto &traj = ddp_object.trajectory_array[i];
//       MSD_LOG(
//           INFO,
//           "[DDP-ModelResultManager] ddp_object_traj: %d, type: %d, size: %d",
//           i, traj.type, traj.trajectory.size());
//       if (traj.trajectory.empty() == false) {
//         MSD_LOG(INFO,
//                 "[DDP-ModelResultManager] ddp_object_traj start_x: %f, "
//                 "start_y: %f, end_x: %f, end_y: %f",
//                 traj.trajectory.front().x, traj.trajectory.front().y,
//                 traj.trajectory.back().x, traj.trajectory.back().y);
//       }
//       if (traj.type != DDP_LANE_CHANGE) {
//         continue;
//       }
//       ddp_trajectory_info_.push_back(traj);

//       MDEBUG_JSON_BEGIN_OBJECT(object)
//       MDEBUG_JSON_ADD_ITEM(logit, traj.logit, object)
//       MDEBUG_JSON_BEGIN_ARRAY(trajectory)
//       for (size_t j = 0; j < traj.trajectory.size(); ++j) {
//         MDEBUG_JSON_BEGIN_OBJECT(object)
//         const auto point = traj.trajectory[j];
//         MDEBUG_JSON_ADD_ITEM(x, point.x, object)
//         MDEBUG_JSON_ADD_ITEM(y, point.y, object)
//         MDEBUG_JSON_ADD_ITEM(heading_angle, point.heading_angle, object);
//         MDEBUG_JSON_ADD_ITEM(curvature, point.curvature, object);
//         MDEBUG_JSON_ADD_ITEM(t, point.t, object)
//         MDEBUG_JSON_ADD_ITEM(v, point.v, object)
//         MDEBUG_JSON_ADD_ITEM(a, point.a, object)
//         MDEBUG_JSON_END_OBJECT(object)
//       }
//       MDEBUG_JSON_END_ARRAY(trajectory)
//       MDEBUG_JSON_END_OBJECT(object)
//     }
//     MDEBUG_JSON_END_ARRAY(trajectory_array)
//   }
// }

bool ModelResultManager::interpolate(const TrajectoryPoints traj_points,
                                     const double start_time,
                                     const double delta_time,
                                     const int num_point,
                                     TrajectoryPoints &res_traj_points) {
  // Step 1) clear
  res_traj_points.clear();
  if (traj_points.size() <= 2) {
    PLANNING_FAIL_TRACE(
        "[ModelResultManager::interpolator] traj_points.size() <= 2");
    return false;
  }

  // Step 2) interpolate traj to current time
  size_t i = 1; // index of input traj points
  for (int j = 0; j < num_point; ++j) {
    double t_j = j * delta_time + start_time;
    while (i + 1 < traj_points.size() && i * delta_time < t_j) {
      ++i;
    }

    double ratio = (t_j - (i - 1) * delta_time) / delta_time;
    if (isnan(ratio) || isinf(ratio)) {
      PLANNING_FAIL_TRACE("[ModelResultManager::interpolator] abnormal_number");
      res_traj_points.clear();
      return false;
    }

    TrajectoryPoint traj_pt;
    traj_pt.t = j * delta_time;
    traj_pt.x = traj_points[i - 1].x +
                ratio * (traj_points[i].x - traj_points[i - 1].x);
    traj_pt.y = traj_points[i - 1].y +
                ratio * (traj_points[i].y - traj_points[i - 1].y);
    traj_pt.v = traj_points[i - 1].v +
                ratio * (traj_points[i].v - traj_points[i - 1].v);
    traj_pt.a = traj_points[i - 1].a +
                ratio * (traj_points[i].a - traj_points[i - 1].a);
    traj_pt.heading_angle = interpolate_angle(
        traj_points[i - 1].heading_angle, traj_points[i].heading_angle, ratio);
    res_traj_points.emplace_back(traj_pt);
  }

  return true;
}
} // namespace ddp
} // namespace msquare
