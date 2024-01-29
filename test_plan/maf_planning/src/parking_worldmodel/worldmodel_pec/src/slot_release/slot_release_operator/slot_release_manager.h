#pragma once
#include "../parameters/slot_release_params.h"
#include "../parameters/slot_release_vehicle_param.h"
#include "../parking_slot_element/parking_slot_element.h"
#include "../schmitt_trigger/schmittTrigger.h"
#include "maf_interface.h"
#include "mtime_core/mtime.h"
#include <Eigen/Eigen>
#include <atomic>
#include <deque>
#include <list>

namespace worldmodel_pec {

// 车位释放算法车位信息整合（感知+地图）和车位推荐逻辑相关算法的入口
//	1. 输入地图车位，感知车位，自车定位信息
//  2. 地图和感知车位的信息整合
//	3. 车位推荐逻辑处理
//  4. 辅助功能逻辑，例如自车在某个车位里的判断
class SlotReleaseManager {

public:
  struct ParkingSlotUnion {
    int global_id_;
    ParkingSlotElement fusion_slot_;
    int failure_reason_;
    double pass_vel_ = 0;

    double pre_vel_ = 0.0;
    bool pre_result_ = false;

    double pre_slot_open_length_ = 10.0;
    double pre_slot_open_width_ = 20.0;
    double pre_passage_width_ = 10.0;
    double pre_left_open_width_ = 10.0;
    double pre_right_open_width_ = 10.0;
    // 记录车道宽度的schmittTrigger

    schmittTrigger passage_width_trigger_;
    schmittTrigger ego_side_obstacel_distance_trigger_;

    schmittTrigger slot_open_width_trigger_;
    schmittTrigger slot_left_open_width_trigger_;
    schmittTrigger slot_right_open_width_trigger_;
    schmittTrigger slot_open_length_trigger_;
  };

  struct EgoCarInfo {
    double velocity_;
    double pitch_;
    double yaw_;
    int gear_;
  };

private:
  const static int KEEP_SLOT_DISTANCE = 15;
  // 多个传感器库位

  SlotReleaseParams params_;

  // 是否初始化的标志
  bool inited_;

  // 开机后自车是否运动过, 是否有初始位姿
  bool is_moved_;
  bool has_start_pose_;

  uint64_t last_updated_timestamp_us_; // 感知时间戳
  std::list<ParkingSlotUnion> history_slots_;

  // 地图白名单车位ID列表，存储map_id
  std::vector<int> white_list_;

  // 车位黑名单（规控泊入失败的车位ID列表），在有地图模式下，存储的是map_id，否则存储的是track_id
  std::vector<int> black_list_;

  // 当前自车所在的车位ID（map_id）
  int ego_parking_slot_map_id_;

  // 当前自车所在的车位ID（track_id）
  int ego_parking_slot_track_id_;

  // 定位结果
  Eigen::Isometry3d loc_rear_pose_start_;

  // 定位是否有效的标志位
  bool loc_valid_;

  // 推荐车位ID（map_id）
  int suggested_parking_slot_map_id_;

  // 推荐车位ID（track_id）
  int suggested_parking_slot_track_id_;
  int last_suggested_parking_slot_track_id_;

  // 推荐WLC车位ID（track_id）
  int suggested_parking_slot_wlc_track_id_;
  int last_suggested_parking_slot_wlc_track_id_;

  // 最新的输入
  maf_perception_interface::FusionParkingSlotResult psd_fusion_input_;

  maf_endpoint::WirelessChargerReport wireless_charger_report_;
  maf_perception_interface::FusionGroundLineResult object_fusion_;
  maf_perception_interface::PerceptionFusionObjectResult object_detection_;

  int apa_target_parkingslot_track_id_;

  std::deque<Eigen::Isometry3d> history_locs_;

public:
  // 返回是否初始化
  bool isInited() const;

  // 初始化
  bool init(const SlotReleaseParams &params);

  bool reset();

  // 加载地图车位白名单
  bool loadWhiteListFromMap(const std::string &map_folder);

  // 输入感知融合车位
  bool feedPerceptionPSDFusion(
      const maf_perception_interface::FusionParkingSlotResult &psd_fusion);

  // 输入自车定位
  bool feedLocalization(const maf_mla_localization::MLALocalization &loc);

  // 输入无线充电信号
  bool feedWirelessChargerReport(
      const maf_endpoint::WirelessChargerReport &wireless_charger_report);
  bool feedObjectFusion(
      const maf_perception_interface::FusionGroundLineResult &groundline);

  bool feedObjectDetection(
      const maf_perception_interface::PerceptionFusionObjectResult &object);

  bool
  feedAPAMode(const maf_system_manager::SysPlanningRequest &planning_request);

  // 设置车位黑名单
  bool setBlackList(const std::vector<int> &blacklist);

  // 更新当前APA的状态(空闲、等待、泊入、泊出)
  bool setAPAStatus(const std::string &status, const int target_id);

  // 生成输出信息
  maf_worldmodel::FusionAPA generateOutput();

  maf_worldmodel::FusionAPA ProcessSlotReleaseAndGetResult();

  // 最终输出的结果，避免每次申请释放
  maf_worldmodel::FusionAPA pec_output_;

  // 计算推荐车位
  //  suggested_parking_slot_map_id_， suggested_parking_slot_track_id_
  bool calculateSuggestedParkingSlot();
  bool calculateSuggestedWirelessChargerParkingSlot();

  // 记录车道宽度的schmittTrigger
  schmittTrigger passage_width_trigger_;
  schmittTrigger ego_side_obstacel_distance_trigger_;
  schmittTrigger ego_side_obs_dis_side_slot_trigger_;

private:
  EgoCarInfo ego_car_info_;
  Eigen::Isometry3d loc_rear_pose_;
  // 当前APA状态
  std::string apa_status_;
  std::atomic_bool is_apa_mode_{true};
  // bool is_apa_mode_;

private:
  // 计算自车所在的车位ID
  //  ego_parking_slot_map_id_， ego_parking_slot_track_id_
  bool calculateEgoParkingSlotID();

  void insertPerceptionSlot(const ParkingSlotElement &slot);
  bool removeSlotsNotObserved(std::vector<uint64_t> &track_id_list);

  bool constructParkingSlotFusionAPADataFromParkingSlotUnion(
      const worldmodel_pec::SlotReleaseManager::ParkingSlotUnion &union_slot,
      maf_worldmodel::ParkingSlotFusionAPAData &output_data) const;

  // 车位开口是否朝向自车(自车后轴中点、车位开口点是否在同一边)
  bool isParkingSlotFacingEgoCar(
      const maf_perception_interface::FusionParkingSlotData &psd);

public:
  SlotReleaseManager();
  ~SlotReleaseManager();
};

} /* namespace worldmodel_pec */
