#include "mtaskflow/mtaskflow.hpp"
#include <thread>

namespace msquare {

class TimerTask : public mtaskflow::FlowTask {
public:
  TimerTask(int hz, mtaskflow::FlowPublisher<uint64_t> tick_publisher)
      : tick_count_(0), stop_flag_(true), frequence_hz_(hz),
        tick_publisher_(tick_publisher) {}

  void on_init() {
    auto hz = frequence_hz_;
    tick_count_ = 0;
    stop_flag_ = false;
    auto &stop_flag = stop_flag_;
    auto &tick_count = tick_count_;
    auto &tick_publisher = tick_publisher_;
    thread_ = std::make_shared<std::thread>([hz, &tick_count, &tick_publisher,
                                             &stop_flag]() {
      while (!stop_flag) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000000 / hz));
        tick_publisher->publish(tick_count++);
      }
    });
  };

  void on_stoping() {
    stop_flag_ = true;
    if (thread_ != nullptr) {
      thread_->join();
      thread_ = nullptr;
    }
  }

  void on_terminate() { on_stoping(); }

private:
  uint64_t tick_count_;
  bool stop_flag_;
  int frequence_hz_;
  mtaskflow::FlowPublisher<uint64_t> tick_publisher_;
  std::shared_ptr<std::thread> thread_;
};

} // namespace msquare
