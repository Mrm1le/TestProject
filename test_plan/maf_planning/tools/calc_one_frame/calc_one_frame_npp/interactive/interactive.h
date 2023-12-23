#pragma once
#include <glog/logging.h>

#include "mfr_publisher.h"
#include "planning_task_on_running.h"
#include <chrono>
#include <cstdlib>
#include <functional>
#include <maf_interface/maf_system_manager.h>
#include <mjson/reader.hpp>
#include <mutex>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <thread>
#include <websocketpp/common/connection_hdl.hpp>
#include <websocketpp/common/system_error.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/frame.hpp>
#include <websocketpp/server.hpp>

namespace isim {

typedef websocketpp::server<websocketpp::config::asio> server;
class Interactive {
public:
  Interactive(int port, int start_frame, int end_frame,
              msquare::PlanningTaskOnRunning *planning_task_on_running)
      : ws_port(port), start_offset_frame(start_frame),
        end_offset_frame(end_frame), frame_to_play(1),
        planning_task_on_running(planning_task_on_running) {
    thread = std::thread(&Interactive::websocket_thread, this);
  }
  void frame_callback(msc::MafMessageCache *msg_cache, int frame) {
    current_frame = frame;
    total_frame = msg_cache->origin_frame_count();
    publish_mfr(msg_cache, frame - 1);
    wait_for_isim();
  }

private:
  void send_state() {
    nlohmann::json root;
    root["current"] = start_offset_frame + current_frame + 1;
    root["total"] = start_offset_frame + total_frame;
    root["state"] = frame_to_play > 0 ? "playing" : "paused";
    // if (!msg.empty()) {
    //   root["message"] = msg;
    // }
    for (auto it = hdls.begin(); it != hdls.end();) {
      if (it->expired())
        hdls.erase(it++);
      else {
        LOG(INFO) << "send state";
        m_endpoint.send(*it, root.dump(), websocketpp::frame::opcode::text);
        it++;
      }
    }
  }
  void wait_for_isim() {
    std::unique_lock<std::mutex> lk(mtx);
    LOG(INFO) << "frame_to_play " << frame_to_play;
    cv.wait(lk, [this] {
      for (auto &req : system_control) {
        parse_cmd(req);
      }
      system_control.clear();

      return frame_to_play > 0;
    });
    LOG(INFO) << "got lock";
    frame_to_play--;
    lk.unlock();
    wait_for_turn();
    send_state();
  }
  void publish_mfr(msc::MafMessageCache *msg_cache, int frame) {
    if (frame < 0)
      return;
    LOG(INFO) << "publish frame " << frame;
    publisher.publish_frame(*msg_cache, frame);
  }

  void websocket_thread() {
    m_endpoint.set_error_channels(websocketpp::log::elevel::all);
    m_endpoint.set_access_channels(websocketpp::log::alevel::all);
    m_endpoint.set_reuse_addr(true);
    m_endpoint.init_asio();
    // m_endpoint.set_message_handler();
    m_endpoint.set_open_handler([this](websocketpp::connection_hdl hdl) {
      LOG(INFO) << "COMMING ";
      hdls.push_back(hdl);
      send_state();
    });

    m_endpoint.set_message_handler(
        [this](websocketpp::connection_hdl hdl, server::message_ptr msg) {
          LOG(INFO) << "LOAD " << msg->get_payload();
          lock_guard<mutex> lk(mtx);
          system_control.push_back(msg->get_payload());
          cv.notify_all();
        });
    m_endpoint.set_close_handler(
        [](websocketpp::connection_hdl hdl) { LOG(INFO) << "LEAVE "; });
    m_endpoint.set_fail_handler(
        [](websocketpp::connection_hdl hdl) { LOG(INFO) << "FAILED "; });

    // Start the Asio io_service run loop
    m_endpoint.listen(ws_port);
    m_endpoint.start_accept();
    m_endpoint.run();
  }

  void wait_for_turn() {
    auto now = std::chrono::system_clock::now();
    auto expect =
        std::chrono::milliseconds((long)(100 * speed)) + last_play_time;
    last_play_time = expect;
    if (now < expect) {
      LOG(INFO) << "bit sleep";
      std::this_thread::sleep_until(expect);
    } else
      LOG(INFO) << "no sleep";
  }

private:
  std::chrono::system_clock::time_point last_play_time;
  double speed = 1;
  int ws_port;
  int start_offset_frame = 0;
  int end_offset_frame = 0;
  int current_frame = 0;
  int total_frame;
  std::thread thread;
  unsigned int frame_to_play = 0;
  std::vector<std::string> system_control;

  server m_endpoint;
  std::vector<websocketpp::connection_hdl> hdls;
  std::mutex mtx;
  std::condition_variable cv;
  fpp::MFRPublisher publisher;

  msquare::PlanningTaskOnRunning *planning_task_on_running; // for planning

private:
  void parse_cmd(string &payload) {
    auto root = nlohmann::json::parse(payload);
    auto cmd = root["command"];

    if (cmd == "play") {
      frame_to_play = 1000;
      last_play_time = std::chrono::system_clock::now();
    } else if (cmd == "pause") {
      frame_to_play = 0;
      send_state();
    } else if (cmd == "step") {
      frame_to_play = 1;
    } else if (cmd == "tuner") {
      // npp::PlanningTaskOnRunning::ReinitConfig();
    } else if (cmd == "mode") {
      string mode = root["mode"];
      parse_cmd_mode(mode);
    } else if (cmd == "planning") {
      parse_cmd_planning(root);

    } else {
      LOG(WARNING) << "unrecognized command " << cmd;
      return;
    }
  }
  void parse_cmd_mode(string &mode) {
    static std::array<
        msc::MessageWrapper<maf_system_manager::ModuleControlCmdRequest, false>,
        1>
        cmd_requests{};
    auto cmd_request = cmd_requests[0].mutable_msg();
    if (mode == "parking") {
      cmd_request->running_mode.value =
          maf_system_manager::RunningModeEnum::PARKING;
      cmd_request->module_control_cmd.value =
          maf_system_manager::ModuleControlCmdEnum::RESUME;
    } else if (mode == "highway") {
      cmd_request->running_mode.value =
          maf_system_manager::RunningModeEnum::PILOT;
      cmd_request->module_control_cmd.value =
          maf_system_manager::ModuleControlCmdEnum::RESUME;
    } else {
      LOG(WARNING) << "invalid mode " << mode;
      return;
    }
    planning_task_on_running->feed_planning_control_cmd(cmd_requests);
  }
  void parse_cmd_planning(nlohmann::json &root) {
    static std::array<
        msc::MessageWrapper<maf_system_manager::SysPlanningRequest, false>, 1>
        planning_requests{};
    auto request = planning_requests[0].mutable_msg();
    if (root.contains("direction")) {
      request->cmd.value =
          maf_system_manager::SystemCmdTypeEnum::PLANNING_HIGHWAY_LANE_CHANGE;
      request->highway_info.lane_change_cmd.type.value =
          maf_system_manager::LaneChangeTypeEnum::INTERACTIVE;
      if (root["direction"] == "left") {
        request->highway_info.lane_change_cmd.direction.value =
            maf_system_manager::LaneChangeDirectionEnum::LEFT;
      } else if (root["direction"] == "right") {
        request->highway_info.lane_change_cmd.direction.value =
            maf_system_manager::LaneChangeDirectionEnum::RIGHT;
      }
    } else if (root.contains("speed")) {
      request->cmd.value =
          maf_system_manager::SystemCmdTypeEnum::PLANNING_HIGHWAY_NAVI_SETTINGS;
      request->highway_info.navi_settings.navi_time_distance.value = 200;
      request->highway_info.navi_settings.navi_max_speed = root["speed"];
    } else if (root.contains("val")) {
      if (root["val"] == "apa") {
        request->cmd.value =
            maf_system_manager::SystemCmdTypeEnum::PLANNING_CHANGE_MODE;
        request->parking_info.mode.value =
            maf_system_manager::SysPlanningModeTypeForParking::PLANNING_APA;
      } else if (root["val"] == "pause") {
        request->cmd.value =
            maf_system_manager::SystemCmdTypeEnum::PLANNING_PAUSE;
      } else if (root["val"] == "continue") {
        request->cmd.value =
            maf_system_manager::SystemCmdTypeEnum::PLANNING_CONTINUE;
      } else if (root["val"] == "stop") {
        request->cmd.value =
            maf_system_manager::SystemCmdTypeEnum::PLANNING_STOP;
      } else if (root["val"] == "parking") {
        request->cmd.value =
            maf_system_manager::SystemCmdTypeEnum::PLANNING_DESIGNATE_PARKIN;
        request->parking_info.mode.value = maf_system_manager::SysPlanningModeTypeForParking::PLANNING_APA;
        request->parking_info.id = root["slot"];
      }
    } else {
      LOG(WARNING) << "invalid cmd " << root.dump();
      return;
    }

    planning_task_on_running->feed_planning_request(planning_requests);
  }
};
} // namespace isim