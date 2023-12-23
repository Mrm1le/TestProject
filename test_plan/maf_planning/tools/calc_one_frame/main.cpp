#include <fstream>
#include <iostream>
#include <mjson/json.hpp>
#include <string>

#include "calc_one_frame.h"
#include "common.h"
#include "stdlib.h"
#include <chrono>

#include "absl/debugging/failure_signal_handler.h"
#include "absl/debugging/symbolize.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

// for gui
DEFINE_bool(export_mfr, false, "export mfr message");
DEFINE_int32(interactive, 0, "interactive port");
DEFINE_bool(dryrun, false, "play original bag");
// for simulation platform

DEFINE_string(input, "", "input bag");
DEFINE_string(output, "", "output bag");
DEFINE_string(result, "", "result json");
DEFINE_double(end_time, 100, "sim end time");
DEFINE_double(shadow_mode, 1.5, "switch time");
DEFINE_string(simmeta, "", "simmeta");
DEFINE_double(speed_limit, 0, "sim speed limit");
DEFINE_bool(ignore_env_car, false, "ignore-env-car");
DEFINE_bool(delete_car, false, "delete-car");

// for calc_one_frame
DEFINE_string(output_dir, "", "output_dir");
DEFINE_string(calc_one_frame, "", "path to a frame file to be calculated"); 
DEFINE_string(debug_frame, "", "debug_frame");
DEFINE_string(is_closed_loop, "", "is_closed_loop");
DEFINE_string(start_time, "", "closed loop start time");
DEFINE_string(pause_frame, "", "pause_frame");

namespace msquare {

bool is_calc_one_frame() { return !FLAGS_calc_one_frame.empty(); } 
bool is_closed_loop() { return !FLAGS_is_closed_loop.empty(); }

int get_debug_frame() {
  try {
    return FLAGS_debug_frame.empty() ? msquare::DEFAULT_DEBUG_FRAME
                                     : std::stoi(FLAGS_debug_frame);
  } catch (...) {
    LOG(INFO) << "wrong debug_frame: \"" << FLAGS_debug_frame << "\"";
  }
  return msquare::DEFAULT_DEBUG_FRAME;
}

int get_pause_frame() {
  try {
    return FLAGS_pause_frame.empty() ? -1 : std::stoi(FLAGS_pause_frame);
  } catch (...) {
    LOG(INFO) << "wrong pause: \"" << FLAGS_pause_frame << "\"";
  }
  return -1;
}

void get_start_time(msquare::GeneratePlanningResultOptions &options) {
  if (is_closed_loop()) {
    options.switch_frame = FLAGS_shadow_mode * 10;
  }
}

int calc_one_frame() {
  msquare::GeneratePlanningResultOptions options;
  options.max_frame_count = MAX_FRAME_COUNT;
  options.is_pnp = false;
  options.is_apa = true;
  options.debug_frame = get_debug_frame();
  options.pause_frame = get_pause_frame();
  options.is_closed_loop = is_closed_loop();

  get_start_time(options);
  auto frame_file_path = FLAGS_calc_one_frame;
  const std::string output_file_path{
      FLAGS_output_dir + make_plan_result_file_path(FLAGS_calc_one_frame)};
  FppTaskResult ret = msquare::generate_planning_result(
      frame_file_path, output_file_path, options);

  return ret.result_type == FppTaskResultType::ERROR ? RETURN_CODE_FAILED
                                                     : RETURN_CODE_SUCCESS;
}

int run_in_simulation() {

  float switch_time;
  LOG(INFO) << std::endl
            << "[RECITE]" << std::endl
            << "input " << FLAGS_input << std::endl
            << "output " << FLAGS_output << std::endl
            << "result " << FLAGS_result
            << std::endl
            // << "start_time " << FLAGS_start_time << std::endl
            // << "end_time " << FLAGS_end_time << std::endl
            << "switch_time " << FLAGS_shadow_mode << std::endl
            << "speed_limit " << FLAGS_speed_limit << std::endl
            << "ignore_env_car " << FLAGS_ignore_env_car << std::endl
            << "delete_car " << FLAGS_delete_car << std::endl
            << "simmeta" << FLAGS_simmeta;

  //   auto start_frame = npp::FPP_DEFAULT_START_FRAME;

  msquare::GeneratePlanningResultOptions option{};
  option.switch_frame = std::max(0, option.switch_frame);
  option.is_apa = true;
  option.interactive = FLAGS_interactive;
  auto start = std::chrono::steady_clock::now();
  auto res = generate_planning_result(FLAGS_input, FLAGS_output, option);
  auto end = std::chrono::steady_clock::now();
  LOG(INFO) << "fpp_time_cost: "
            << std::chrono::duration<double>(end - start).count() << std::endl;

  auto response = mjson::Json(mjson::Json::object());
  response["input"] = mjson::Json(FLAGS_input);
  response["output"] = mjson::Json(FLAGS_output);
  response["code"] = res.result_type;
  response["message"] = res.msg;
  std::ofstream of(FLAGS_result);
  of << response.dump();
  LOG(INFO) << response.dump();
  return 0;
}

} // namespace msquare

int main(int argc, char *argv[]) {
  absl::InitializeSymbolizer(argv[0]);
  absl::FailureSignalHandlerOptions options;
  absl::InstallFailureSignalHandler(options);

  gflags::AllowCommandLineReparsing();
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = 1;
  google::InitGoogleLogging(argv[0]);

  int rc = 0;
  LOG(INFO) << "start";
  if (msquare::is_calc_one_frame()) {
    rc = msquare::calc_one_frame();
  } else if (!FLAGS_input.empty()) {
    rc = msquare::run_in_simulation();
  } else {
    rc = 1;
  }
  LOG(INFO) << "end";

  return rc;
}
