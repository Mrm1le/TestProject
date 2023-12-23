#pragma once
#include "common.h"
#include <limits>
#include <string>

namespace msquare {

enum FppTaskType { OPEN_LOOP, CLOSED_LOOP, STRESS_TEST, EXTRACT_FRAMES };

enum FppTaskResultType { FPP_SUCCESS, FAIL, ERROR };

class FppTaskResult {
public:
  FppTaskResult(FppTaskResultType type, const std::string &your_msg)
      : result_type(type), msg(your_msg){};
  FppTaskResultType result_type = FppTaskResultType::FPP_SUCCESS;
  std::string msg;
  static FppTaskResult success(const std::string &msg) {
    return FppTaskResult(FppTaskResultType::FPP_SUCCESS, msg);
  }
  static FppTaskResult error(const std::string &msg) {
    return FppTaskResult(FppTaskResultType::ERROR, msg);
  }
  static FppTaskResult fail(const std::string &msg) {
    return FppTaskResult(FppTaskResultType::FAIL, msg);
  }
};

struct GeneratePlanningResultOptions {
  int max_frame_count = std::numeric_limits<int>::max();
  bool is_pnp = false;
  bool is_apa = false;
  int debug_frame = -1;
  bool is_closed_loop = true;
  float start_time = 0;
  float end_time = 0;
  int switch_frame = FPP_DEFAULT_SWITCH_FRAME;
  int start_output_frame = 0;
  int speed_limit = 0;
  int navi_time_distance = 0;
  bool ignore_env_car = false;
  bool is_ddld = false;
  int pause_frame = -1;
  int interactive = 0;
};

FppTaskResult
generate_planning_result(const std::string &input_msg_bag_path,
                         const std::string &planning_result_bag_path,
                         const GeneratePlanningResultOptions &options);

bool stress_test(const std::string &input_msg_bag_path);
bool production_test(const std::string &input_msg_bag_path);
bool echo(const std::string &input_msg_bag_path, const std::string &topic);

} // namespace msquare
