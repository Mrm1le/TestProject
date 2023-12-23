#ifndef CP_MAF_BASE_HPP
#define CP_MAF_BASE_HPP

#include "maf_interface.h"
#include "status_manager.hpp"

namespace maf {
class CoreBase {
 public:
  CoreBase() : status_manager_(StatusManager::make()){};

  virtual ~CoreBase() = default;

  virtual bool init() = 0;

  virtual bool start() = 0;

  virtual bool stop() = 0;

  virtual bool reset() = 0;

  virtual bool exit() = 0;

 public:
  // 本模块发出NodeStatus的回调函数
  // 目前core
  // base没有引入mtime，因此timestamp_us是不赋值的，需要Adaptor跟据需要填入
  void set_node_status_callback(const StatusManager::NodeStatusCallback &func) {
    status_manager_->set_notifier_callback(func);
  }

  // 本模块处理启停请求后的回调函数，参数为seq_id, success
  void set_system_control_response_callback(
      const std::function<void(uint32_t, bool, node_status::Status)> &func) {
    response_callback_ = func;
  }

  // 默认的SystemControlRequest处理函数，可以被override
  virtual void feed_system_control_request(
      std::shared_ptr<maf_system_manager::ModuleControlCmdRequest> &msg) {
    bool ret = false;
    if (msg->module_control_cmd.value == msg->module_control_cmd.PAUSE) {
      ret = stop();
    } else if (msg->module_control_cmd.value ==
               msg->module_control_cmd.RESUME) {
      ret = start();
    }
    // Response
    if (response_callback_) {
      response_callback_(msg->header.seq, ret,
                         static_cast<node_status::Status>(
                             status_manager_->get_status().status));
    }
  }

  maf_framework_status::NodeStatus get_status() {
    return status_manager_->get_status();
  }

 protected:
  std::function<void(uint32_t, bool, node_status::Status)> response_callback_;
  std::shared_ptr<maf::StatusManager> status_manager_;
};

}  // namespace maf

#endif
