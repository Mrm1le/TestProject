#pragma once

#include "maf_interface.h"
#include <string>

namespace worldmodel_pec {

struct SystemManagerRequestBlackList {
  //当前实现不太严谨，有HDMAP（推荐了地图ID）的情况下这个ID是地图ID，反之是trackID，需要算法内部基于当前“是否加载了地图车位列表”进行判断
  //后续接口再做升级
  std::vector<int> blacklist_id_;
};

// WorldModelPEC模块的算法接口
//执行机制：每个函数都是阻塞式的，内部没有多线程，执行完毕函数返回
//线程安全性：绝大多数函数（isInited除外）都带同一个互斥锁，保证不会被多个线程同时执行
class WorldModelPEC {

public:
  //单例模式设计，通过此接口获取一个WorldModelPEC算法模组的唯一实例的指针。使用完毕无需释放
  static WorldModelPEC *getInstance();

public:
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //	函数名称	:	isInited
  //	描述		:	返回是否初始化成功
  //	输入		:	无
  //	输出		:	true ： 初始化成功  false ： 初始化未成功
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isInited() = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //	函数名称	:	init
  //	描述		:	初始化WorldModelPEC功能
  //	输入		:	calib_folder
  //标定参数文件夹（读取相机标定参数和车体参数） 	输出		:
  // true ： 初始化成功  false ： 初始化未成功
  //	特别注意	：
  //初始化是不读取地图车位白名单的，白名单信息通过setSystemManagerRequestChangeRandomSearchList输入
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool init(std::string params_json_str) = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //	函数名称	:	reset
  //	描述		:   重置系统状态，清除缓存
  //	输出		:	true ： 重置成功  false ： 重置未成功
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool reset() = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //	函数名称	:	feedLocalization
  //	描述		:	输入自车定位结果
  //	输入		:	loc 定位结果
  //	输出		:	true : 正常执行  false ：异常执行
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool
  feedLocalization(const maf_mla_localization::MLALocalization &loc) = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //	函数名称	:	feedWirelessChargerReport
  //	描述		:	输入无线充电器状态
  //	输入		:	wireless_charger_report 无线充电器状态
  //	输出		:	true : 正常执行  false ：异常执行
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool feedWirelessChargerReport(
      const maf_endpoint::WirelessChargerReport &wireless_charger_report) = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //	函数名称	:	feedParkingSlotPerceptionFusion
  //	描述		:	输入车位感知融合结果
  //	输入		:	psd_fusion 车位感知融合结果
  //	输出		:	true : 正常执行  false ：异常执行
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool feedParkingSlotPerceptionFusion(
      const maf_perception_interface::FusionParkingSlotResult &psd_fusion) = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //	函数名称	:	feedFusionGroundline
  //	描述		:	输入接地线融合信息
  //	输入		:	groundline_fusion
  //以FusionAPA形式填充的地图车位信息
  //	输出		:	true : 正常执行  false ：异常执行
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool
  feedFusionGroundline(const maf_perception_interface::FusionGroundLineResult
                           &groundline_fusion) = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //	函数名称	:	feedFusionObject
  //	描述		:	输入接地线融合信息
  //	输入		:	object_fusion 以FusionAPA形式填充的地图车位信息
  //	输出		:	true : 正常执行  false ：异常执行
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool
  feedFusionObject(const maf_perception_interface::PerceptionFusionObjectResult
                       &object_fusion) = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //	函数名称	:	getParkingSlotOutput
  //	描述		:	返回填充了车位相关信息的world_model结构体
  //	输入		:	无
  //	输出		:	maf_worldmodel::FusionAPA
  //				其中有效字段为：
  //					fusion_apa.parking_slots
  //					fusion_apa.ego_parking_slot_track_id
  //					fusion_apa.ego_parking_slot_map_id
  //					fusion_apa.suggested_parking_slot_track_id
  //					fusion_apa.suggested_parking_slot_map_id
  //				其余字段无效
  //	有效更新	：  当输入了定位或车位感知结果后，此函数更新有效  （
  // feedLocalization || feedParkingSlotPerceptionFusion ）
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual maf_worldmodel::FusionAPA getParkingSlotOutput() = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //	函数名称   	:	setSystemManagerRequestBlackList
  //	描述		:	处理system_manager请求（设置车位黑名单）
  //	输入		:	request_data 黑名单请求
  //	输出		:	true : 正常执行且成功  false ：异常执行或失败
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool setSystemManagerRequestBlackList(
      const SystemManagerRequestBlackList &request_data) = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //	函数名称	:	setSystemManagerRequestChangeRandomSearchList
  //	描述		:	处理system_manager请求（修改随机搜索白名单）
  //	输入		:	map_folder
  //地图文件夹（全路径）。当前的随机搜索白名单列表是保存在地图文件夹里，因此采用这种方式实现，后续会改成直接通过接口由外部发送。
  //			：
  //特殊情况1：如果map_folder为空，则清除白名单。会返回正常执行且成功。例如从SVP切APA的时候。
  //			：
  //特殊情况2：如果通过此函数输入了白名单，但没有输入地图车位列表（feedMapParkingSlot），则白名单功能无效，但输入的白名单ID列表仍然有效保存
  //			：
  //致命错误：如果map_folder不为空，但读取不到指定的地图文件，则会报错，清除掉白名单，并将整个WorldModelPEC
  // instance设置为未初始化状态，无法接受任何指令，直到重新初始化为止 	输出
  //:	true : 正常执行且成功  false ：异常执行或失败
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool setSystemManagerRequestChangeRandomSearchList(
      const std::string &map_folder) = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //	函数名称	:	setAPAStatus输入APA当前的任务状态
  //	描述		:	更新APA当前的任务状态
  //	输入		:	"idle" 空闲
  //             :   "apa_wait" 等待
  //             :   "apa_parking_in" 泊入 , "target_id" 目标车位
  //             :   "apa_parking_out" 泊出
  //	输出		:	true : 正常执行且成功  false ：异常执行或失败
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool setAPAStatus(const std::string &status, const int target_id) = 0;

  virtual bool feedInitAPAMode(
      const maf_system_manager::SysPlanningRequest &planning_request) = 0;

public:
  WorldModelPEC(){};
  virtual ~WorldModelPEC(){};
};

} // namespace worldmodel_pec
