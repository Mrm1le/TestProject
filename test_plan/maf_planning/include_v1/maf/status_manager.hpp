#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

#include "maf_interface.h"
#include "monitor_define/node_status_define.h"
#include "mtime_core/mtime.h"

namespace maf {

class StatusManager;

class StatusManager : public std::enable_shared_from_this<StatusManager> {
public:
  using NodeStatusCallback =
      std::function<void(const maf_framework_status::NodeStatus &)>;

  explicit StatusManager() : is_notifier_running_(false) {
    current_status_.status =
        static_cast<uint16_t>(node_status::Status::STARTING);
  }

  ~StatusManager() { stop_notifier(); }

  void set_node_type(node_status::NodeType node_type) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    current_status_.node_type = static_cast<uint16_t>(node_type);
  }

  node_status::NodeType get_node_type() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return static_cast<node_status::NodeType>(current_status_.node_type);
  }

  bool is_notifier_running() const { return is_notifier_running_; }

  bool can_change_to(node_status::Status target_status) {
    // uint16_t tmp_target_status;
    node_status::Status tmp_current_status;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      tmp_current_status =
          static_cast<node_status::Status>(current_status_.status);
    }
    return tmp_current_status != target_status;
    // return (tmp_current_status == node_status::Status::STARTING &&
    //         (target_status == node_status::Status::PENDING)) ||
    //        (tmp_current_status == node_status::Status::RUNNING &&
    //         (target_status == node_status::Status::RUNNING_ERROR ||
    //          target_status == node_status::Status::PENDING ||
    //          target_status == node_status::Status::RUNNING_WARNING ||
    //          target_status == node_status::Status::STOP)) ||
    //        (tmp_current_status == node_status::Status::RUNNING_WARNING &&
    //         (target_status == node_status::Status::RUNNING ||
    //          target_status == node_status::Status::RUNNING_ERROR)) ||
    //        (tmp_current_status == node_status::Status::RUNNING_ERROR &&
    //         (target_status == node_status::Status::STOP ||
    //          target_status == node_status::Status::PENDING)) ||
    //        (tmp_current_status == node_status::Status::PENDING &&
    //         (target_status == node_status::Status::RUNNING ||
    //          target_status == node_status::Status::RUNNING_ERROR ||
    //          target_status == node_status::Status::STOP ||
    //          target_status == node_status::Status::PENDING));
  }

  bool try_change_status(
      node_status::Status target_status,
      const std::function<void()> &pre_transition_func = nullptr) {
    if (can_change_to(target_status)) {
      if (pre_transition_func) {
        pre_transition_func();
      }
      {
        std::unique_lock<std::mutex> lock(status_mutex_);
        current_status_.status = static_cast<uint16_t>(target_status);
        lock.unlock();
      }
      cv_.notify_one();
      return true;
    }
    return false;
  }

  maf_framework_status::NodeStatus get_status() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    current_status_.timestamp_us = MTIME()->timestamp().us();
    return current_status_;
  }

  void set_node_status_message(const std::string &message) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    current_status_.message = message;
  }

  void set_node_status_detail_status(const uint16_t detailed_status) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    current_status_.detail_status = detailed_status;
  }

  void set_node_status_task_type(uint16_t type) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    current_status_.task_type = type;
  }

  void set_node_status_running_mode(
      const maf_system_manager::RunningModeEnum &mode) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    current_status_.running_mode = mode;
  }

  void set_notifier_callback(const NodeStatusCallback &cb) {
    node_status_cb_ = cb;
  }

  void start_notifier(uint update_interval_ms = 1000) {
    if (!is_notifier_running_) {
      is_notifier_running_ = true;
      worker_thread_ = std::thread([this, update_interval_ms]() {
        while (is_notifier_running_) {
          std::unique_lock<std::mutex> lock(status_mutex_);
          cv_.wait_for(lock, std::chrono::milliseconds(update_interval_ms));
          if (!is_notifier_running_)
            break;
          if (node_status_cb_) {
            current_status_.timestamp_us = MTIME()->timestamp().us();
            node_status_cb_(current_status_);
          }
        }
      });
    }
  }

  void stop_notifier() {
    is_notifier_running_ = false;
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

  static std::shared_ptr<StatusManager> make() {
    return std::make_shared<StatusManager>();
  }

private:
  maf_framework_status::NodeStatus current_status_;
  std::mutex status_mutex_;

  NodeStatusCallback node_status_cb_;

  bool is_notifier_running_;
  std::thread worker_thread_;
  std::condition_variable cv_;
};

} // namespace maf
