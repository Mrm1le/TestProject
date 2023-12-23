#include "slot_release_manager.h"
#include "localization_utils/loc_utils.hpp"
#include "pnc/define/parking_vision_info.h"
#include "slot_release_algorithm_operator.h"
#include "planning/common/logging.h"
#include "utils/math/math_helper.hpp"
#include "utils/math/polygon.hpp"
#include <fstream>
#include <mlog_core/mlog.h>
#include "../parameters/slot_release_params.h"
#include "slot_release_config.h"

namespace worldmodel_pec {

SlotReleaseManager::SlotReleaseManager() {
  inited_ = false;
  has_start_pose_ = false;
  is_moved_ = false;
  last_updated_timestamp_us_ = 0;
  ego_parking_slot_map_id_ = -1;
  ego_parking_slot_track_id_ = -1;
  loc_valid_ = false;
  suggested_parking_slot_map_id_ = -1;
  suggested_parking_slot_track_id_ = -1;
  last_suggested_parking_slot_track_id_ = -1;
  suggested_parking_slot_wlc_track_id_ = -1;
  last_suggested_parking_slot_wlc_track_id_ = -1;
  apa_status_ = "idle";
  apa_target_parkingslot_track_id_ = -1;
  ego_car_info_.velocity_ = 0.0;
  ego_car_info_.pitch_ = 0.0;
  ego_car_info_.yaw_ = 0.0;
  ego_car_info_.gear_ = 0;
  is_apa_mode_ = true;
}

SlotReleaseManager::~SlotReleaseManager() {}

bool SlotReleaseManager::isInited() const { return inited_; }

bool SlotReleaseManager::init(const SlotReleaseParams &params) {
  // params_ = params;
  //其实什么事情也不干，只是重置了一下各变量
  ego_parking_slot_map_id_ = -1;
  ego_parking_slot_track_id_ = -1;
  loc_valid_ = false;
  suggested_parking_slot_map_id_ = -1;
  suggested_parking_slot_track_id_ = -1;
  last_suggested_parking_slot_track_id_ = -1;
  suggested_parking_slot_wlc_track_id_ = -1;
  last_suggested_parking_slot_wlc_track_id_ = -1;

  white_list_.clear();
  black_list_.clear();
  history_slots_.clear();
  history_locs_.clear();

  inited_ = true;
  is_moved_ = false;
  has_start_pose_ = false;
  ego_car_info_.velocity_ = 0.0;
  ego_car_info_.pitch_ = 0.0;
  ego_car_info_.yaw_ = 0.0;
  ego_car_info_.gear_ = 0;
  is_apa_mode_ = true;
  auto config_file_dir = 
    SlotReleaseContext::Instance()->get_config_file_dir() + 
    "/scenario_configs_json";
  char *calib_dir_env = std::getenv("CALIB_DIR");

  std::string vehicle_param_file;
  if (calib_dir_env == nullptr) {
    vehicle_param_file =
        config_file_dir + "/parking/vehicle_param.yaml";
  } else {
    std::string calib_dir(calib_dir_env);
    std::string vehicle_param_path = calib_dir + "/vehicle.yaml";
    MLOG_ERROR("VehicleParam: CALIB_DIR %s yaml dir %s.\n", calib_dir.c_str(),
            vehicle_param_path.c_str());
    if (access(vehicle_param_path.c_str(), F_OK) == -1) {
      MLOG_ERROR("VehicleParam: vehicle.yaml not exist!\n");
      vehicle_param_file =
          config_file_dir + "/parking/vehicle_param.yaml";
    } else {
      vehicle_param_file = vehicle_param_path;
    }
  }
  if (!CarParams::GetInstance()->loadFile(vehicle_param_file)) {
    MLOG_ERROR("std::logic_error: CarParams load from vehicle_param_file failed!\n");
  }

  if (!CarParams::GetInstance()->loadFileSlotRelease(config_file_dir)) {
    MLOG_ERROR("std::logic_error: CarParams load from vehicle_param_file failed!\n");
  }

  params_ = CarParams::GetInstance()->slot_release_config_;

  if (!VehicleParam::Instance()->loadFile(vehicle_param_file)) {
    MSD_LOG(
        ERROR,
        "std::logic_error: VehicleParam load from vehicle_param_file failed!");
  }
  MLOG_ERROR("slot release car width is %f car length is %f, and the real car width is %f.\n", 
    CarParams::GetInstance()->vehicle_width, CarParams::GetInstance()->vehicle_length,
    CarParams::GetInstance()->vehicle_width_real);
  MLOG_ERROR("slot release vertical passage width is :%f vertical passage width hys is: %f.\n", 
    params_.vertical_passage_width, params_.vertical_passage_width_hys_scope);
  MLOG_ERROR("the car type is: %s\n", (VehicleParam::Instance()->car_type).c_str());
  return true;
}

bool SlotReleaseManager::reset() {
  ego_parking_slot_map_id_ = -1;
  ego_parking_slot_track_id_ = -1;
  loc_valid_ = false;
  suggested_parking_slot_map_id_ = -1;
  suggested_parking_slot_track_id_ = -1;
  last_suggested_parking_slot_track_id_ = -1;
  suggested_parking_slot_wlc_track_id_ = -1;
  last_suggested_parking_slot_wlc_track_id_ = -1;

  history_slots_.clear();
  MSD_LOG(ERROR, "%s: %d", __FUNCTION__, __LINE__);
  history_locs_.clear();

  inited_ = true;
  is_moved_ = false;
  has_start_pose_ = false;
  ego_car_info_.velocity_ = 0.0;
  ego_car_info_.pitch_ = 0.0;
  ego_car_info_.yaw_ = 0.0;
  ego_car_info_.gear_ = 0;
  is_apa_mode_ = true;
  return true;
}

bool SlotReleaseManager::loadWhiteListFromMap(const std::string &map_folder) {
  //如果map_folder为空，表明是需要清除白名单
  if (map_folder.empty()) {
    white_list_.clear();
    return true;
  }
  //如果map_folder不为空，则通过地图文件夹加载白名单
  white_list_.clear();
  std::string file_name =
      map_folder + "/worldmodel/random_search_parkingslot_list.txt";
  std::ifstream ifs(file_name);
  //如果文件读取失败，则报错
  if (!ifs.is_open()) {
    MLOG_ERROR(
        "[SlotReleaseManager::loadWhiteListFromMap] open file failed: %s",
        file_name.c_str());
    return true;
  }
  int id;
  while (ifs >> id) {
    white_list_.push_back(id);
  }
  MLOG_INFO("[SlotReleaseManager::loadWhiteListFromMap] loaded %d parking "
            "slot IDs as white list ",
            (int)white_list_.size());
  ifs.close();
  return true;
}

bool SlotReleaseManager::feedPerceptionPSDFusion(
    const maf_perception_interface::FusionParkingSlotResult &psd_fusion) {
  //检查AVAILABLE标志位
  last_updated_timestamp_us_ = psd_fusion.meta.sensor_timestamp_us;

  //循环遍历每个感知融合parking_slot，并构建成ParkingSlotElement
  std::vector<uint64_t> track_id_list;
  for (unsigned int i = 0; i < psd_fusion.parking_slot.parking_slot_data.size();
       i++) {
    const maf_perception_interface::FusionParkingSlotData &ps =
        psd_fusion.parking_slot.parking_slot_data[i];

    // 如果车位背对自车，则跳过该车位
    if (!isParkingSlotFacingEgoCar(ps)) {
      continue;
    }

    ParkingSlotElement pse{};
    //从meta字段内的时间戳填充last_updated_timestamp_us_
    pse.last_updated_timestamp_us_ = psd_fusion.meta.sensor_timestamp_us;
    pse.last_updated_timestamp_sec_ =
        1.0e-6 * (double)psd_fusion.meta.sensor_timestamp_us;
    pse.charge_property_ = ps.charge_property;
    pse.track_id_ = ps.track_id;
    pse.empty_votes_ = ps.empty_votes;
    pse.slot_type_ = ps.slot_type.value;
    pse.source_from_uss_ =
        ps.available & ps.FUSION_PARKING_SLOT_SOURCE_ULTRASONIC;
    pse.source_from_vision_ =
        ps.available & ps.FUSION_PARKING_SLOT_SOURCE_VISON;

    track_id_list.push_back(pse.track_id_);

    // 复制reserved info(挡轮器信息)
    if (psd_fusion.parking_slot.parking_slot_data.size() ==
        psd_fusion.parking_slot.reserved_info.size()) {
      pse.reserved_info_ = psd_fusion.parking_slot.reserved_info[i];
    } else {
      MLOG_ERROR(
          "PSD Fusion parking_slot_data size(%d) != reserved_info size(%d)!",
          psd_fusion.parking_slot.parking_slot_data.size(),
          psd_fusion.parking_slot.reserved_info.size());
    }

    //填充corner，计算center
    pse.center_.setZero();
    pse.corners_.clear();
    pse.corners_.reserve(4);
    int points_num = ps.local_points_fusion.size();
    for (int i = 0; i < points_num; i++) {
      const auto &corner = ps.local_points_fusion[i];
      ParkingSlotPoint p{};
      p.position = Eigen::Vector3d(corner.x, corner.y, corner.z);
      p.visible = true;
      pse.corners_.push_back(p);
      pse.center_ += p.position;
    }
    pse.applyVritualCorner();

    for (size_t i = 0; i < ps.local_wheel_stop_points.size(); i++) {
      const auto &point = ps.local_wheel_stop_points[i];
      Eigen::Vector3d p{point.x, point.y, point.z};
      pse.wheel_stop_points_.push_back(p);
    }

    if (pse.corners_.empty()) {
      MLOG_ERROR("perception fusion slot corners num is zero\n");
      continue;
    }
    pse.center_ *= (1.0 / pse.corners_.size());

    insertPerceptionSlot(pse);
  }

  (void)removeSlotsNotObserved(track_id_list);

  //计算自车所在的车位，计算失败的话则输出自车不在车位内。不需要处理失败
  (void)calculateEgoParkingSlotID();

  return true;
}

bool SlotReleaseManager::feedObjectFusion(
    const maf_perception_interface::FusionGroundLineResult &groundline) {
  object_fusion_ = groundline;
  return true;
}

bool SlotReleaseManager::feedObjectDetection(
    const maf_perception_interface::PerceptionFusionObjectResult &object) {
  object_detection_ = object;
  return true;
}

bool SlotReleaseManager::feedAPAMode(
    const maf_system_manager::SysPlanningRequest &planning_request) {
  if (planning_request.cmd.value ==
      maf_system_manager::SystemCmdTypeEnum::PLANNING_HIGHWAY_FUNCTION_MODE) {
    if (planning_request.highway_info.function_mode.value ==
        maf_system_manager::FunctionModeEnum::PNP) {
      is_apa_mode_ = false;
    } else {
      is_apa_mode_ = true;
    }
  }

  return false;
}

void SlotReleaseManager::insertPerceptionSlot(const ParkingSlotElement &slot) {
  bool has_matched = false;
  for (auto &his_slot : history_slots_) {
    // 判断相同ID
    if (slot.track_id_ == (int)his_slot.global_id_) {
      has_matched = true;
      his_slot.fusion_slot_ = slot;
      break;
    }
  }

  if (false == has_matched) {
    ParkingSlotUnion union_slot{};
    union_slot.global_id_ = slot.track_id_;
    union_slot.fusion_slot_ = slot;

    union_slot.passage_width_trigger_.beginTH(triggerType_t::INVERTING,
                                              params_.vertical_passage_width - 
                                              params_.vertical_passage_width_hys_scope,
                                              params_.parallel_passage_width, 1);
    union_slot.ego_side_obstacel_distance_trigger_.beginTH(
        triggerType_t::INVERTING, params_.ego_side_obstacel_distance_low,
        params_.ego_side_obstacel_distance_high, 1);
    union_slot.slot_open_width_trigger_.beginTH(triggerType_t::INVERTING, 2.3,
                                                2.5, 1);
    union_slot.slot_left_open_width_trigger_.beginTH(triggerType_t::INVERTING,
                                                     0.7, 0.8, 1);
    union_slot.slot_right_open_width_trigger_.beginTH(triggerType_t::INVERTING,
                                                      0.7, 0.8, 1);
    union_slot.slot_open_length_trigger_.beginTH(triggerType_t::INVERTING,
                                                 params_.vertical_passage_width - 
                                                 params_.vertical_passage_width_hys_scope,
                                                 params_.parallel_passage_width, 1);

    history_slots_.push_back(union_slot);
  }
  return;
}

bool SlotReleaseManager::removeSlotsNotObserved(
    std::vector<uint64_t> &track_id_list) {
  for (std::list<ParkingSlotUnion>::iterator it = history_slots_.begin();
       it != history_slots_.end();) {
    bool observed_in_fusion_psd = false;
    for (auto track_id : track_id_list) {
      if (it->global_id_ == (int)track_id) {
        observed_in_fusion_psd = true;
        break;
      }
    }
    // delete history slot that is not observed in this frame
    if (!observed_in_fusion_psd) {
      it = history_slots_.erase(it);
    } else {
      it++;
    }
  }
  return true;
}

bool SlotReleaseManager::feedLocalization(
    const maf_mla_localization::MLALocalization &loc) {
  //保存定位信息，并记录定位是否有效的标志位
  loc_valid_ = putils::LocUtils::isValid(loc);
  if (loc.velocity.available &
      maf_mla_localization::MLAVelocity::MLA_VEL_LOCAL) {
    ego_car_info_.velocity_ = std::hypot(loc.velocity.velocity_local.vx,
                                         loc.velocity.velocity_local.vy);
  }

  if (loc.orientation.available &
      maf_mla_localization::MLAOrientation::MLA_EULER_LOCAL) {
    ego_car_info_.pitch_ = loc.orientation.euler_local.pitch;
    ego_car_info_.yaw_ = loc.orientation.euler_local.yaw;
  }

  double vel_dot_yaw =
      loc.velocity.velocity_local.vx * std::cos(ego_car_info_.yaw_) +
      loc.velocity.velocity_local.vy * std::sin(ego_car_info_.yaw_);
  if (ego_car_info_.velocity_ > 0.1) {
    if (vel_dot_yaw < 0) {
      // printf("%s: %d\n", __FUNCTION__, __LINE__);
      ego_car_info_.gear_ = -1;
    } else {
      // printf("%s: %d\n", __FUNCTION__, __LINE__);
      ego_car_info_.gear_ = 1;
    }
  }

  loc_rear_pose_ = putils::LocUtils::poseFromMLA(loc);
  if (loc_valid_ && !has_start_pose_) {
    loc_rear_pose_start_ = loc_rear_pose_;
    MSD_LOG(ERROR, "%s: %d", __FUNCTION__, __LINE__);
    history_locs_.push_back(loc_rear_pose_);
    // std::cout << "the local rear pose is:" <<
    // loc_rear_pose_.translation().x()
    //           << " the y is:" << loc_rear_pose_.translation().y() <<
    //           std::endl;
    for (const auto &data : history_locs_) {
      // std::cout << "history pose is:" << data.translation().x()
      //           << " the y is:" << data.translation().y() << std::endl;
    }
    has_start_pose_ = true;
  }

  if (!is_moved_ && has_start_pose_) {
    double move_dis = (loc_rear_pose_start_.translation().topRows(2) -
                       loc_rear_pose_.translation().topRows(2))
                          .norm();
    if (move_dis > 0.5) {
      is_moved_ = true;
    }
  }

  const double HISTORY_LOCS_INTERVAL = 0.3;
  const double HISTORY_LOCS_LENGTH = 30;
  if (loc_valid_) {
    if (!history_locs_.empty()) {
      Eigen::Isometry3d &last_loc = history_locs_.back();
      if ((loc_rear_pose_.translation().topRows(2) -
           last_loc.translation().topRows(2))
              .norm() > HISTORY_LOCS_INTERVAL) {
        history_locs_.push_back(loc_rear_pose_);
        // printf("push loc to history(%d)\n", history_locs_.size());
      }

      if (history_locs_.size() > HISTORY_LOCS_LENGTH / HISTORY_LOCS_INTERVAL) {
        // printf("pop front\n");
        history_locs_.pop_front();
      }
    }
  }

  return true;
}

bool SlotReleaseManager::feedWirelessChargerReport(
    const maf_endpoint::WirelessChargerReport &wireless_charger_report) {
  wireless_charger_report_ = wireless_charger_report;
  return true;
}

bool SlotReleaseManager::setBlackList(const std::vector<int> &black_list) {
  //设置blacklist
  black_list_.clear();
  for (const auto &id : black_list) {
    black_list_.push_back(id);
  }
  return true;
}

maf_worldmodel::FusionAPA SlotReleaseManager::generateOutput() {
  //获取当前的系统时间戳
  auto ts_usec = MTIME()->timestamp().us();

  pec_output_ = maf_worldmodel::FusionAPA{};
  pec_output_.header.frame_id = "map";       // to make parking_rviz work
  pec_output_.header.stamp = ts_usec * 1000; // header的时间戳为纳秒
  pec_output_.meta.timestamp_us = last_updated_timestamp_us_; //此消息的时间戳
  pec_output_.meta.pipeline_start_timestamp_us =
      0; //并不清楚pipeline开始的时间（因为有几个不同的输入定位/PSD），这里不赋值

  //填充environment_data和environment_data.fusion_apa相关的available位
  pec_output_.available = 0;

  // 填充 EGO_PARKING_SLOT_MAP_ID &  EGO_PARKING_SLOT_TRACK_ID
  //当且仅当有egopose的时候，这个字段才有意义
  if (loc_valid_) {
    pec_output_.available |= maf_worldmodel::FusionAPA::EGO_PARKING_SLOT_MAP_ID;
    pec_output_.available |=
        maf_worldmodel::FusionAPA::EGO_PARKING_SLOT_TRACK_ID;
  }
  pec_output_.ego_parking_slot_map_id = ego_parking_slot_map_id_;
  pec_output_.ego_parking_slot_track_id = ego_parking_slot_track_id_;

  //填充fusion_apa.parking_slots
  pec_output_.available |= maf_worldmodel::FusionAPA::PARKING_SLOTS;
  pec_output_.available |= maf_worldmodel::FusionAPA::RESERVED_INFO;
  for (const auto &union_slot : history_slots_) {
    //限制只输出本车附近的车位信息
    if (loc_valid_) {
      const Eigen::Vector3d &slot_center = union_slot.fusion_slot_.center_;
      double xy_dis =
          (slot_center.topRows(2) - loc_rear_pose_.translation().topRows(2))
              .norm();
      double z_dis =
          std::abs(slot_center.z() - loc_rear_pose_.translation().z());
      if (z_dis > 1.0 || xy_dis > KEEP_SLOT_DISTANCE) {
        continue;
      }
    }
    struct maf_worldmodel::ParkingSlotFusionAPAData slot_apa_data {};
    std::string reserved_info;
    bool ret = constructParkingSlotFusionAPADataFromParkingSlotUnion(
        union_slot, slot_apa_data);
    if (ret) {
      pec_output_.parking_slots.push_back(slot_apa_data);
      pec_output_.reserved_info.push_back(
          union_slot.fusion_slot_.reserved_info_);
    } else {
      MLOG_ERROR("[SlotReleaseManager::generateOutput] "
                 "constructParkingSlotFusionAPADataFromParkingSlotUnion "
                 "fail\n");
    }
  }
  MLOG_DEBUG("Generate %d parking slots\n", pec_output_.parking_slots.size());

  pec_output_.available |=
      maf_worldmodel::FusionAPA::SUGGESTED_PARKING_SLOT_MAP_ID;
  pec_output_.available |=
      maf_worldmodel::FusionAPA::SUGGESTED_PARKING_SLOT_TRACK_ID;

  pec_output_.suggested_parking_slot_map_id = suggested_parking_slot_map_id_;
  // pec_output_.suggested_parking_slot_track_id =
  //     suggested_parking_slot_track_id_;
  if (apa_status_ == "apa_parking_in") {
    pec_output_.suggested_parking_slot_track_id =
        apa_target_parkingslot_track_id_;
  } else {
    pec_output_.suggested_parking_slot_track_id =
        suggested_parking_slot_wlc_track_id_ == -1
            ? suggested_parking_slot_track_id_
            : suggested_parking_slot_wlc_track_id_;
  }

  return pec_output_;
}

bool SlotReleaseManager::constructParkingSlotFusionAPADataFromParkingSlotUnion(
    const worldmodel_pec::SlotReleaseManager::ParkingSlotUnion &union_slot,
    maf_worldmodel::ParkingSlotFusionAPAData &wm_apa_data) const {
  //填充available字段
  wm_apa_data.available =
      maf_worldmodel::ParkingSlotFusionAPAData::PARKING_SLOT |
      maf_worldmodel::ParkingSlotFusionAPAData::APA_INFO;

  //填充parking_slot内的属性
  wm_apa_data.parking_slot.available = 0;
  if (union_slot.fusion_slot_.source_from_vision_) {
    wm_apa_data.parking_slot.available |=
        wm_apa_data.parking_slot.FUSION_PARKING_SLOT_SOURCE_VISON;
  }
  if (union_slot.fusion_slot_.source_from_uss_) {
    wm_apa_data.parking_slot.available |=
        wm_apa_data.parking_slot.FUSION_PARKING_SLOT_SOURCE_ULTRASONIC;
  }
  if (0 == wm_apa_data.parking_slot.available) {
    MLOG_ERROR("[SlotReleaseManager::"
               "constructParkingSlotFusionAPADataFromParkingSlotUnion] slot "
               "type error\n");
    return false;
  }
  wm_apa_data.parking_slot.charge_property = union_slot.fusion_slot_.charge_property_;
  wm_apa_data.parking_slot.track_id = union_slot.global_id_;
  wm_apa_data.parking_slot.empty_votes = union_slot.fusion_slot_.empty_votes_;
  wm_apa_data.parking_slot.slot_type.value = union_slot.fusion_slot_.slot_type_;
  const auto &fusion_slot_corners = union_slot.fusion_slot_.corners_;
  for (int i = 0; i < fusion_slot_corners.size(); i++) {
    auto corner = fusion_slot_corners[i];
    if (union_slot.fusion_slot_.has_virtual_point_ &&
        i == union_slot.fusion_slot_.virtual_point_index_) {
      corner.position = union_slot.fusion_slot_.init_point_;
    }
    maf_perception_interface::Point3f corner_pt{};
    corner_pt.x = corner.position.x();
    corner_pt.y = corner.position.y();
    corner_pt.z = corner.position.z();
    maf_perception_interface::ParkingSlotPointTypeEnum type_pt{};
    type_pt.value = corner.visible
                        ? maf_perception_interface::ParkingSlotPointTypeEnum::
                              PARKING_SLOT_POINT_TYPE_REAL
                        : maf_perception_interface::ParkingSlotPointTypeEnum::
                              PARKING_SLOT_POINT_TYPE_FAKE;
    wm_apa_data.parking_slot.local_points_fusion.push_back(corner_pt);
    wm_apa_data.parking_slot.local_points_fusion_confidence.push_back(
        corner.confidence);
    wm_apa_data.parking_slot.local_points_fusion_type.push_back(type_pt);

    Eigen::Vector3d corner_pt_self =
        loc_rear_pose_.inverse() *
        Eigen::Vector3d(corner_pt.x, corner_pt.y, corner_pt.z);
    maf_perception_interface::Point3f corner_pt_world{};
    corner_pt_world.x = corner_pt_self.x();
    corner_pt_world.y = corner_pt_self.y();
    corner_pt_world.z = corner_pt_self.z();
    wm_apa_data.parking_slot.points_fusion.push_back(corner_pt_world);
    wm_apa_data.parking_slot.points_fusion_confidence.push_back(
        corner.confidence);
    wm_apa_data.parking_slot.points_fusion_type.push_back(type_pt);

    //角点只填充local_points_fusion, points_fusion，不考虑其他字段
  }

  for (const auto &point : union_slot.fusion_slot_.wheel_stop_points_) {
    maf_perception_interface::Point3f p{};
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    wm_apa_data.parking_slot.local_wheel_stop_points.push_back(p);
  }

  //填充APA_INFO
  wm_apa_data.apa_info.map_id = union_slot.failure_reason_; // no map id in APA
  wm_apa_data.apa_info.last_update_timestamp = last_updated_timestamp_us_;

  // std::cout << "the id is: " << union_slot.fusion_slot_.track_id_
  //           << "the empty_votes is: " << union_slot.fusion_slot_.empty_votes_
  //           << "the failure reason: " << union_slot.failure_reason_ <<
  //           std::endl;

  //基于empty_votes_ 填充车位状态
  if (union_slot.fusion_slot_.empty_votes_ > 0) {
    wm_apa_data.apa_info.status = maf_worldmodel::APAInfo::VACANT;
  } else if (union_slot.fusion_slot_.empty_votes_ < 0) {
    wm_apa_data.apa_info.status = maf_worldmodel::APAInfo::OCCUPIED;
  } else {
    wm_apa_data.apa_info.status = maf_worldmodel::APAInfo::UNKNOWN;
  }

  return true;
}

bool SlotReleaseManager::calculateEgoParkingSlotID() {
  //虽然外部条件应当控制了没有定位的数据不会进入处理流程中
  //但还是做了异常处理，仅处理有定位的情况
  ego_parking_slot_map_id_ = -1;
  ego_parking_slot_track_id_ = -1;
  if (loc_valid_) {
    //使用自车定位（后轴中心）位置查找所在的车位
    // bool found = false;
    for (const auto &union_slot : history_slots_) {
      const auto &slot = union_slot.fusion_slot_;
      if (slot.pointInParkingSlot(loc_rear_pose_.translation())) {
        ego_parking_slot_map_id_ = -1; // NO map id In APA
        ego_parking_slot_track_id_ = union_slot.global_id_;
        break;
      }
    }
    return true;
  } else //无定位的时候，直接赋值不在车位里的数值
  {
    ego_parking_slot_map_id_ = -2;
    ego_parking_slot_track_id_ = -2;
    return false;
  }
}

bool SlotReleaseManager::calculateSuggestedParkingSlot() {
  // static const double SUGGEST_SLOT_DISTANCE = 10.0;
  suggested_parking_slot_map_id_ = -1;
  suggested_parking_slot_track_id_ = -1;
  float last_suggested_psd_id_distance = 10.0;
  double min_distance = 1e20;
  int min_distance_psd_id = -1;

  if (apa_status_ == "apa_parking_in") {
    suggested_parking_slot_track_id_ = apa_target_parkingslot_track_id_;
    return true;
  }

  //虽然外部条件应当控制了没有定位的数据不会进入处理流程中
  //但还是做了异常处理，仅处理有定位的情况
  if (!loc_valid_) {
    return false;
  }

  // 1. 筛选符合规则的车位：空车位 + 地图车位 + 白名单车位 + 不在黑名单里
  for (const auto &union_slot : history_slots_) {
    //必须为空车位
    if (union_slot.fusion_slot_.empty_votes_ <= 0) {
      continue;
    }

    const auto &current_slot_center = union_slot.fusion_slot_.center_;
    double dis = (current_slot_center - loc_rear_pose_.translation()).norm();
    if (union_slot.fusion_slot_.track_id_ ==
        last_suggested_parking_slot_track_id_) {
      last_suggested_psd_id_distance = dis;
    }

    if (dis < min_distance) {
      min_distance = dis;
      min_distance_psd_id = union_slot.fusion_slot_.track_id_;
    }
  }

  if (last_suggested_parking_slot_track_id_ == -1 ||
      min_distance < last_suggested_psd_id_distance - 0.1) // 滞回
  {
    last_suggested_parking_slot_track_id_ = suggested_parking_slot_track_id_;
    suggested_parking_slot_track_id_ = min_distance_psd_id;
  }

  return true;
}

bool SlotReleaseManager::calculateSuggestedWirelessChargerParkingSlot() {
  suggested_parking_slot_wlc_track_id_ = -1;
  float last_suggested_psd_id_distance = 10.0;
  double min_distance = 1e20;
  int min_distance_psd_id = -1;

  //虽然外部条件应当控制了没有定位的数据不会进入处理流程中
  //但还是做了异常处理，仅处理有定位的情况
  if (!loc_valid_) {
    return false;
  }

  // 1. 筛选符合规则的车位：空车位 + 地图车位 + 白名单车位 + 不在黑名单里
  for (const auto &union_slot : history_slots_) {
    //必须为空车位
    if (union_slot.fusion_slot_.empty_votes_ <= 0) {
      continue;
    }
    //必须为无线充电车位
    if (union_slot.fusion_slot_.charge_property_ != 1) {
      continue;
    }
    // 必须连接成功
    if (wireless_charger_report_.wireless_charger_report_data.link_state
            .value !=
        maf_endpoint::WrlsChrgWifiSts::WIRELESS_CHARGER_WIFI_CONNECTED) {
      continue;
    }

    const auto &current_slot_center = union_slot.fusion_slot_.center_;
    double dis = (current_slot_center - loc_rear_pose_.translation()).norm();
    if (union_slot.fusion_slot_.track_id_ ==
        last_suggested_parking_slot_wlc_track_id_) {
      last_suggested_psd_id_distance = dis;
    }

    if (dis < min_distance) {
      min_distance = dis;
      min_distance_psd_id = union_slot.fusion_slot_.track_id_;
    }
  }

  if (last_suggested_parking_slot_wlc_track_id_ == -1 ||
      min_distance < last_suggested_psd_id_distance - 0.1) // 滞回
  {
    last_suggested_parking_slot_wlc_track_id_ =
        suggested_parking_slot_wlc_track_id_;
    suggested_parking_slot_wlc_track_id_ = min_distance_psd_id;
  }
  // printf("suggested_parking_slot_wlc_track_id_ = %d\n",
  //        suggested_parking_slot_wlc_track_id_);

  return true;
}

bool SlotReleaseManager::setAPAStatus(const std::string &status,
                                      const int target_id) {
  apa_status_ = status;
  apa_target_parkingslot_track_id_ = target_id;
  return true;
}

maf_worldmodel::FusionAPA SlotReleaseManager::ProcessSlotReleaseAndGetResult() {

  // std::cout << "=============Run===========" << std::endl;
  auto ts_usec_start = MTIME()->timestamp().us();
  SlotReleaseHandler slot_release_handler;
  slot_release_handler.process(
      object_fusion_, object_detection_, ego_car_info_, loc_rear_pose_,
      history_locs_, is_moved_, apa_target_parkingslot_track_id_, apa_status_,
      ego_parking_slot_track_id_, params_, is_apa_mode_, history_slots_);

  (void)calculateSuggestedParkingSlot();
  MSD_LOG(ERROR, "wlc %s: %d", __FUNCTION__, __LINE__);
  (void)calculateSuggestedWirelessChargerParkingSlot();
  auto ts_usec_end = MTIME()->timestamp().us();
  // std::cout << "spend time is:" << ts_usec_end - ts_usec_start << std::endl;
  //调用parking_slot_operator_接口，返回输出结果
  return generateOutput();
}

bool SlotReleaseManager::isParkingSlotFacingEgoCar(
    const maf_perception_interface::FusionParkingSlotData &psd) {
  // 直线方程为：Ax+By+C=0,直线是由其上两点(x1,y1)，(x2,y2)确定的
  // A=y2-y1
  // B=x1-x2
  // C=x2*y1-x1*y2
  // D=A*xp + B*yp + C
  // 若D<0，则点p在直线的左侧；若D>0，则点p在直线的右侧；若D＝0，则点p在直线上。
  auto psd_bottom_line_func = [&](float xp, float yp) {
    float x1 = psd.points_fusion[1].x;
    float x2 = psd.points_fusion[2].x;
    float y1 = psd.points_fusion[1].y;
    float y2 = psd.points_fusion[2].y;
    float A = y2 - y1;
    float B = x1 - x2;
    float C = x2 * y1 - x1 * y2;
    return A * xp + B * yp + C;
  };

  float result =
      psd_bottom_line_func(psd.points_fusion[0].x, psd.points_fusion[0].y) *
      psd_bottom_line_func(0.0, 0.0);

  // // 后轴中点
  if (result > 0.0) {
    return true;
  } else {
    return false;
  }
}

} /* namespace worldmodel_pec */
