//
// Created by ros on 5/26/20.
//

// #include <cpr/cpr.h>
#include <ctime>
#include <exception>
#include <iostream>

#include "nlohmann/json.hpp"
#include "upload_msd_test.h"

namespace msquare {

void UploadMsdTest::upload(const RecognizeInfo &recognize_info) {

  nlohmann::json recognize_info_json = nlohmann::json::object();
  nlohmann::json record_json = nlohmann::json::object();

  std::chrono::milliseconds ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  recognize_info_json["uuid"] = std::to_string(ms.count());
  recognize_info_json["which_car"] = recognize_info.which_car;
  recognize_info_json["upload_time"] =
      std::to_string(recognize_info.current_time);
  recognize_info_json["scenario_name"] = recognize_info.scenario_name;
  recognize_info_json["durable_time"] = recognize_info.durable_time;

  record_json["record_json_str"] =
      nlohmann::json::parse(recognize_info.record_json_str);
  recognize_info_json["details"] = record_json;

  std::string record_json_str = recognize_info_json.dump();

  // auto r =
  // cpr::Post(cpr::Url{"https://10.196.69.68:5000/api/v2/road/scenario"},
  //                 cpr::Body{record_json_str},
  //                 cpr::Header{{"Content-Type", "application/json"}},
  //                 cpr::VerifySsl(false));
}
} // namespace msquare
