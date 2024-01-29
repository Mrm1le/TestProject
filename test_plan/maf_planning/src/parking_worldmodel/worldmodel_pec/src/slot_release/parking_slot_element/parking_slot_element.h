#pragma once

#include <Eigen/Core>
#include <vector>
namespace worldmodel_pec {

struct ParkingSlotPoint {
  Eigen::Vector3d position;
  double confidence;
  bool visible;
};

struct VirtualParkingSlotPoint {
  /**
    0：UNKOWN；
    1：垂直；
    2：平行；
    3：普通斜列；
    4：矩形斜列；
     */
  int slot_type = 0;
  Eigen::Vector3d position;
  int index;
};

// PEC算法中，使用的ParkingSlot数据结构，存储
//	1. 常规的车位信息，如角点，ID等
//	2. 地图信息，如地图ID
// 提供了由感知融合数据消息
// 提供了向maf_worldmodel::ParkingSlotFusionAPAData转换的代码
// 提供了几个必要的计算函数
class ParkingSlotElement {

public:
  // 角点位置，所有的位置全部是local坐标系，不保存和处理自车坐标系下的信息
  std::vector<ParkingSlotPoint> corners_;

  // 中心点位置（4个角点位置的平均值）
  Eigen::Vector3d center_;

  // 无线充电桩属性
  uint8_t charge_property_;

  // 感知融合给出的track-id
  int track_id_;

  // 感知融合给出的空车位投票计数
  int empty_votes_;

  // 最近一次更新时间戳（秒），更新时间取的是感知融合数据的时间
  double last_updated_timestamp_sec_;

  // 最近一次更新时间戳（微秒），更新时间取的是感知融合数据的时间
  long long last_updated_timestamp_us_;

  // 车位来源的类型
  bool source_from_vision_;
  bool source_from_uss_;

  // 保留字段：挡轮器等信息
  std::string reserved_info_;

  std::vector<Eigen::Vector3d> wheel_stop_points_;

  bool has_virtual_point_ = false;
  int virtual_point_index_ = 0;
  Eigen::Vector3d init_point_;

  uint8_t slot_type_;

public:
  bool isMatchWith(const ParkingSlotElement &other) const;

  // 判断某个点是否在车位内 条件： 1. Z坐标在范围内
  //  2.XY坐标在车位角点围成的多边形内 输入： 3D点 输出： true - 在车位内  false
  //  - 不在车位内
  bool pointInParkingSlot(const Eigen::Vector3d &point) const;

public:
  ParkingSlotElement();
  ~ParkingSlotElement();
  void applyVritualCorner();

private:
  VirtualParkingSlotPoint extractVritualCorner() const;
};

} /* namespace worldmodel_pec */
