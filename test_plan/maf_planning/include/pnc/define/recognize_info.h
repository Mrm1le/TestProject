//
// Created by hzy on 5/22/20.
// recognize interface msg
//

#ifndef COMMON_RECOGNIZE_INFO_
#define COMMON_RECOGNIZE_INFO_

#include <chrono>
#include <string>
#include <unordered_map>

namespace msquare {

/*
std::unordered_map<ScenarioRecognizeType, std::string, EnumClassHash>
scenario_type_map_name = { {ScenarioRecognizeType::CUT_IN, "CUT_IN"},
    {ScenarioRecognizeType::LANE_CHANGE, "LANE_CHANGE"},
    {ScenarioRecognizeType::MAP_LANE_CHANGE, "MAP_LANE_CHANGE"},
    {ScenarioRecognizeType::POI_REGION_PASS, "POI_REGION_PASS"},
    {ScenarioRecognizeType::OBSTACLE_JEEVES, "OBSTACLE_JEEVES"},
    {ScenarioRecognizeType::OBSTACLE_TRAVERSE, "OBSTACLE_TRAVERSE"},
    {ScenarioRecognizeType::MERGING, "MERGING"},
    {ScenarioRecognizeType::OBSTACLE_NEGATIVE_DIRECTION,
"OBSTACLE_NEGATIVE_DIRECTION"}, {ScenarioRecognizeType::TRAFFIC_LIGHT,
"TRAFFIC_LIGHT"}, {ScenarioRecognizeType::VEHICLE_REVERSE, "VEHICLE_REVERSE"},
    {ScenarioRecognizeType::OTHERS, "OTHERS"}
};

*/

struct RecognizeInfo {

  long long current_time;

  std::string which_car;

  std::string scenario_name;

  int durable_time = 0.0;

  // 需要上传到平台的json信息(包含这个场景时间段的所有信息里的list)， 暂定都上传
  std::string record_json_str;
};
} // namespace msquare

#endif
